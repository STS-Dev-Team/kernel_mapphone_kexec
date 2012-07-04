/*
     Copyright (C) 2011 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio_mapping.h>
#include <linux/radio_ctrl/radio_class.h>
#include <linux/radio_ctrl/g4852_ctrl.h>
#include <asm/bootinfo.h>

#define GPIO_MAX_NAME 30
#define SIM_SWITCH_GPIO	84

static int ap_wakeup_ste_gpio = -1;

enum g4852_status {
	G4852_STATUS_NORMAL,
	G4852_STATUS_FLASH,
	G4852_STATUS_PANIC,
	G4852_STATUS_OFF,
	G4852_STATUS_UNDEFINED,
};

static const char *g4852_status_str[] = {
	[G4852_STATUS_NORMAL]		= RADIO_STATUS_NORMAL_NAME,
	[G4852_STATUS_FLASH]		= RADIO_STATUS_FLASH_NAME,
	[G4852_STATUS_PANIC]		= RADIO_STATUS_PANIC_NAME,
	[G4852_STATUS_OFF]		= RADIO_STATUS_OFF_NAME,
	[G4852_STATUS_UNDEFINED]	= RADIO_STATUS_UNDEFINED_NAME,
};

struct g4852_info {
	unsigned int gsm_pwron;
	char gsm_pwron_name[GPIO_MAX_NAME];

	unsigned int ap_reset_gsm;
	char ap_reset_gsm_name[GPIO_MAX_NAME];

	unsigned int gsm_panic;
	char gsm_panic_name[GPIO_MAX_NAME];

	unsigned int gsm_flash_en;
	char gsm_flash_en_name[GPIO_MAX_NAME];

	unsigned int gsm_status1;
	char gsm_status1_name[GPIO_MAX_NAME];

	unsigned int bplog_to_ap_en;
	char bplog_to_ap_en_name[GPIO_MAX_NAME];

	unsigned int bplog_to_jtag_en;
	char bplog_to_jtag_en_name[GPIO_MAX_NAME];

	bool boot_flash;
	enum g4852_status status;

	u16 pad_uart3rx;
	u16 pad_uart3tx;
	struct radio_dev rdev;
};

struct simswitch_info {
	int simswitch;
	char simswitch_name[GPIO_MAX_NAME];
	struct radio_dev rdev;
};


static ssize_t g4852_status_show(struct radio_dev *rdev, char *buff)
{
	struct g4852_info *info =
		container_of(rdev, struct g4852_info, rdev);

	pr_debug("%s: g4852_status = %d\n", __func__, info->status);
	if (info->status > G4852_STATUS_UNDEFINED)
		info->status = G4852_STATUS_UNDEFINED;

	return snprintf(buff, RADIO_STATUS_MAX_LENGTH, "%s\n",
		g4852_status_str[info->status]);
}

static ssize_t g4852_do_powerdown(struct g4852_info *info)
{
	int i, value;

	pr_info("%s: powering down\n", __func__);

	/*
	 * The user space program sends PSCPOF AT command to modem to initiate
	 * the power off sequence. When STE G4852 is ready to cut off power,
	 * it asserts GSM_STATUS1 to inform AP. AP waits for up to 5sec for
	 * this signal.
	 */
	for (i = 0; i < 10; i++) {
		value = gpio_get_value(info->gsm_status1);
		if (value) {
			/*BP trace wakeup is a 6us high/6us low pulse
			double check here*/
			usleep_range(10, 20);
			value = gpio_get_value(info->gsm_status1);
		}
		pr_info("%s: gsm_status1 %d value = %d\n", __func__,
			info->gsm_status1, value);
		if (value)
			break;
		msleep(500);
	}

	msleep(100);
	gpio_direction_output(info->gsm_pwron, 0);
	info->status = G4852_STATUS_OFF;

	/* set to low-level after power-down */
	gpio_direction_output(info->gsm_flash_en, 0);
	gpio_direction_output(ap_wakeup_ste_gpio, 0);

	return 0;
}

static ssize_t g4852_do_powerup(struct g4852_info *info)
{

	pr_debug("%s: enter\n", __func__);

	msleep(100);

	/* power on in normal or flash mode */
	if (info->boot_flash)
		gpio_direction_output(info->gsm_flash_en, 0);
	else {
		gpio_direction_output(info->gsm_flash_en, 1);
		gpio_direction_output(ap_wakeup_ste_gpio, 1);
	}

	gpio_direction_output(info->gsm_pwron, 1);

	if (info->boot_flash) {
		pr_debug("%s: started g4852 in flash mode\n",
			__func__);
		info->status = G4852_STATUS_FLASH;
	} else {
		pr_debug("%s: started g4852 in normal mode\n",
				__func__);
		info->status = G4852_STATUS_NORMAL;
	}
	return 0;
}

static ssize_t g4852_do_reset(struct g4852_info *info)
{

	pr_debug("%s: enter\n", __func__);

	/* power on in normal or flash mode */
	if (info->boot_flash)
		gpio_direction_output(info->gsm_flash_en, 0);
	else
		gpio_direction_output(info->gsm_flash_en, 1);

	gpio_direction_output(info->ap_reset_gsm, 1);
	msleep(100);
	gpio_direction_output(info->ap_reset_gsm, 0);

	return 0;
}


static ssize_t g4852_do_tatmode(struct g4852_info *info)
{

	pr_debug("%s: enter\n", __func__);

	set_ste_tat_mode();
	g4852_do_reset(info);

	return 0;
}

static ssize_t g4852_do_bplog(struct g4852_info *info, int on)
{

	pr_debug("%s: enter\n", __func__);

	if (on) {
		omap_writew(info->pad_uart3rx, OMAP443X_CTRL_BASE + 0x144);
		omap_writew(info->pad_uart3tx, OMAP443X_CTRL_BASE + 0x146);
		gpio_direction_output(info->bplog_to_jtag_en, 0);
		gpio_direction_output(info->bplog_to_ap_en, 1);
	} else {
		omap_writew(7, OMAP443X_CTRL_BASE + 0x144);
		omap_writew(7, OMAP443X_CTRL_BASE + 0x146);
		gpio_direction_output(info->bplog_to_ap_en, 0);
		gpio_direction_output(info->bplog_to_jtag_en, 1);
	}
	set_ste_bplog_mode(on);
	return 0;
}


static ssize_t g4852_set_flash_mode(struct g4852_info *info, bool enable)
{
	pr_debug("%s: set boot state to %d\n", __func__, enable);
	info->boot_flash = enable;
	return 0;
}

static ssize_t g4852_command(struct radio_dev *rdev, char *cmd)
{
	struct g4852_info *info =
		container_of(rdev, struct g4852_info, rdev);

	pr_info("%s: user command = %s\n", __func__, cmd);

	if (strcmp(cmd, "shutdown") == 0)
		return g4852_do_powerdown(info);
	else if (strcmp(cmd, "powerup") == 0)
		return g4852_do_powerup(info);
	else if (strcmp(cmd, "reset") == 0)
		return g4852_do_reset(info);
	else if (strcmp(cmd, "tatmode") == 0)
		return g4852_do_tatmode(info);
	else if (strcmp(cmd, "bplog_on") == 0)
		return g4852_do_bplog(info, 1);
	else if (strcmp(cmd, "bplog_off") == 0)
		return g4852_do_bplog(info, 0);
	else if (strcmp(cmd, "bootmode_normal") == 0)
		return g4852_set_flash_mode(info, 0);
	else if (strcmp(cmd, "bootmode_flash") == 0)
		return g4852_set_flash_mode(info, 1);

	pr_err("%s: command %s not supported\n", __func__, cmd);
	return -EINVAL;
}

static ssize_t simswitch_command(struct radio_dev *rdev, char *cmd)
{
	ssize_t ret = 0;
	struct simswitch_info *info =
		container_of(rdev, struct simswitch_info, rdev);

	pr_info("%s: user command = %s\n", __func__, cmd);

	if (strcmp(cmd, "on") == 0)
		gpio_direction_output(info->simswitch, 1);
	else if (strcmp(cmd, "off") == 0)
		gpio_direction_output(info->simswitch, 0);
	else {
		pr_err("%s: command %s not supported\n", __func__, cmd);
		ret = -EINVAL;
	}

	return ret;
}

static irqreturn_t g4852_panic_fn(int irq, void *data)
{
	struct g4852_info *info = (struct g4852_info *) data;
	pr_info("%s:  GSM panic irq (%d) fired\n", __func__, irq);
	if (info->rdev.dev)
		kobject_uevent(&info->rdev.dev->kobj, KOBJ_CHANGE);
	return IRQ_HANDLED;
}

static irqreturn_t g4852_panic_isr(int irq, void *data)
{
	struct g4852_info *info = (struct g4852_info *) data;
	pr_debug("%s:  GSM panic irq (%d) fired\n", __func__, irq);
	info->status = G4852_STATUS_PANIC;
	return IRQ_WAKE_THREAD;
}

static int __devinit g4852_ctrl_probe(struct platform_device *pdev)
{
	struct g4852_ctrl_platform_data *pdata = pdev->dev.platform_data;
	struct g4852_info *info;
	struct simswitch_info *info2;
	int panic_irq, err = 0;

	dev_info(&pdev->dev, "g4852_probe");
	pr_debug("%s: %s\n", __func__, dev_name(&pdev->dev));

	if (bi_powerup_reason() == PU_REASON_CHARGER) {
		dev_info(&pdev->dev, "g4852_probe: GSM not started in COM, exit.\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct g4852_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto err_exit;
	}

	info2 = kzalloc(sizeof(struct simswitch_info), GFP_KERNEL);
	if (!info2) {
		err = -ENOMEM;
		goto err_exit2;
	}

	platform_set_drvdata(pdev, info);

	/* setup radio_class device */
	info->rdev.name = pdata->name;
	info->rdev.status = g4852_status_show;
	info->rdev.command = g4852_command;

	/* setup simswitch device */
	info2->rdev.name = "simswitch";
	info2->rdev.status = NULL;
	info2->rdev.command = simswitch_command;


	/* gsm_pwron */
	pr_debug("%s: setup gsm_pwron\n", __func__);
	info->gsm_pwron = pdata->gsm_pwron;
	snprintf(info->gsm_pwron_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "gsm_pwron");
	err = gpio_request(info->gsm_pwron, info->gsm_pwron_name);
	if (err) {
		pr_err("%s: err requesting gsm_pwron gpio\n", __func__);
		goto err_pwron;
	}
	gpio_direction_output(info->gsm_pwron, 1);

	/* ap_reset_gsm */
	pr_debug("%s: setup ap_reset_gsm\n", __func__);
	info->ap_reset_gsm = pdata->ap_reset_gsm;
	snprintf(info->ap_reset_gsm_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "ap_reset_gsm");
	err = gpio_request(info->ap_reset_gsm, info->ap_reset_gsm_name);
	if (err) {
		pr_err("%s: err requesting ap_reset_gsm gpio\n", __func__);
		/* goto err_reset; */
	}
	gpio_direction_output(info->ap_reset_gsm, 0);

	/* gsm_flash_en */
	pr_debug("%s: setup gsm_flash_en\n", __func__);
	info->gsm_flash_en = pdata->gsm_flash_en;
	snprintf(info->gsm_flash_en_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "gsm_flash_en");
	err = gpio_request(info->gsm_flash_en, info->gsm_flash_en_name);
	if (err) {
		pr_err("%s: err requesting gsm_flash_en gpio\n", __func__);
		goto err_flash;
	}
	gpio_direction_output(info->gsm_flash_en, 1);

	/* gsm_status1 */
	/* gsm_status1 used as Trace wakeup and shutdown ack both,
	so it cann't be request/free here.*/
	pr_debug("%s: setup gsm_status1\n", __func__);
	info->gsm_status1 = pdata->gsm_status1;
	snprintf(info->gsm_status1_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "gsm_status1");

	/* bplog_to_ap_en */
	pr_debug("%s: setup bplog_to_ap_en\n", __func__);
	info->bplog_to_ap_en = pdata->bplog_to_ap_en;
	snprintf(info->bplog_to_ap_en_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "bplog_to_ap_en");
	err = gpio_request(info->bplog_to_ap_en, info->bplog_to_ap_en_name);
	if (err) {
		pr_err("%s: err requesting bplog_to_ap_en\n", __func__);
		goto err_bplog_to_ap_en;
	}

	/* bplog_to_jtag_en */
	pr_debug("%s: setup bplog_to_jtag_en\n", __func__);
	info->bplog_to_jtag_en = pdata->bplog_to_jtag_en;
	snprintf(info->bplog_to_jtag_en_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "bplog_to_jtag_en");
	err = gpio_request(info->bplog_to_jtag_en, info->bplog_to_jtag_en_name);
	if (err) {
		pr_err("%s: err requesting bplog_to_jtag_en\n", __func__);
		goto err_bplog_to_jtag_en;
	}

	/* By default,route BP log to JTAG if AP doesn't use JTAG as console.*/
	if (!strstr(saved_command_line, "console=ttyO2")) {
		/* Disconnect AP UART3 and BP UART0. */
		info->pad_uart3rx = omap_readw(OMAP443X_CTRL_BASE + 0x144);
		info->pad_uart3tx = omap_readw(OMAP443X_CTRL_BASE + 0x146);
		omap_writew(7, OMAP443X_CTRL_BASE + 0x144);
		omap_writew(7, OMAP443X_CTRL_BASE + 0x146);
		gpio_direction_output(info->bplog_to_ap_en, 0);
		gpio_direction_output(info->bplog_to_jtag_en, 1);
		pr_info("ttyO2 is not used for console.\n");
	}

	/* gsm_panic */
	pr_debug("%s: setup gsm_panic\n", __func__);
	info->gsm_panic = pdata->gsm_panic;
	snprintf(info->gsm_panic_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "gsm_panic");
	err = gpio_request(info->gsm_panic, info->gsm_panic_name);
	if (err) {
		pr_err("%s: err requesting gsm_panic\n", __func__);
		goto err_panic;
	}
	gpio_direction_input(info->gsm_panic);
	panic_irq = gpio_to_irq(info->gsm_panic);

	/* ap_wakeup_ste */
	ap_wakeup_ste_gpio = get_gpio_by_name("ap_wakeup_ste");
	if (ap_wakeup_ste_gpio < 0)
		printk(KERN_ERR "%s: can't get ap_wakeup_ste", __func__);

	err = request_threaded_irq(panic_irq, g4852_panic_isr,
		g4852_panic_fn, IRQ_TYPE_EDGE_RISING, info->gsm_panic_name,
		info);
	if (err) {
		pr_err("%s: request irq (%d) %s failed\n",
			__func__, panic_irq, info->gsm_panic_name);
		gpio_free(info->gsm_panic);
		goto err_panic;
	}

	/* try to determine the boot up mode of the device */
	if (gpio_get_value(info->gsm_flash_en)) {
		info->boot_flash = 0;
		info->status = G4852_STATUS_NORMAL;
	} else {
		info->boot_flash = 1;
		info->status = G4852_STATUS_FLASH;
	}

	pr_debug("%s: initial status = %s\n", __func__,
		g4852_status_str[info->status]);

	err = radio_dev_register(&info->rdev);
	if (err) {
		pr_err("%s: failed to register radio device\n", __func__);
		goto err_dev_register;
	}

	/* SIM hot switch*/
	info2->simswitch = get_gpio_by_name("slot_sel");
	if (info2->simswitch < 0)
		info2->simswitch = SIM_SWITCH_GPIO;
		snprintf(info2->simswitch_name, GPIO_MAX_NAME, "%s-%s",
			dev_name(&pdev->dev), "simswitch");
	gpio_request(info2->simswitch, info2->simswitch_name);
	gpio_direction_output(info2->simswitch, 0);

	err = radio_dev_register(&info2->rdev);
	if (err)
		pr_err("%s: failed to register simswitch device\n", __func__);

	return 0;

err_dev_register:
	free_irq(panic_irq, info);
	gpio_free(info->gsm_panic);
err_panic:
	gpio_free(info->bplog_to_jtag_en);
err_bplog_to_jtag_en:
	gpio_free(info->bplog_to_ap_en);
err_bplog_to_ap_en:
err_status1:
	gpio_free(info->gsm_flash_en);
err_flash:
	gpio_free(info->ap_reset_gsm);
err_reset:
	gpio_free(info->gsm_pwron);
err_pwron:
	platform_set_drvdata(pdev, NULL);
	kfree(info2);
err_exit2:
	kfree(info);
err_exit:
	return err;
}

static void __devexit g4852_shutdown(struct platform_device *pdev)
{
	struct g4852_info *info = platform_get_drvdata(pdev);
	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	(void) g4852_do_powerdown(info);
}

static int __devexit g4852_remove(struct platform_device *pdev)
{
	struct g4852_info *info = platform_get_drvdata(pdev);

	pr_debug("%s: %s\n", __func__, dev_name(&pdev->dev));

	radio_dev_unregister(&info->rdev);

	free_irq(gpio_to_irq(info->gsm_panic), info);
	gpio_free(info->gsm_panic);
	gpio_free(info->bplog_to_jtag_en);
	gpio_free(info->bplog_to_ap_en);
	gpio_free(info->gsm_flash_en);
	gpio_free(info->ap_reset_gsm);
	gpio_free(info->gsm_pwron);

	platform_set_drvdata(pdev, NULL);
	kfree(info);

	return 0;
}

static struct platform_driver g4852_ctrl_driver = {
	.probe = g4852_ctrl_probe,
	.remove = __devexit_p(g4852_remove),
	.shutdown = __devexit_p(g4852_shutdown),
	.driver = {
		.name = G4852_CTRL_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init g4852_ctrl_init(void)
{
	printk(KERN_DEBUG "g4852_ctrl_init\n");
	return platform_driver_register(&g4852_ctrl_driver);
}

static void __exit g4852_ctrl_exit(void)
{
	printk(KERN_DEBUG "g4852_exit\n");
	platform_driver_unregister(&g4852_ctrl_driver);
}

module_init(g4852_ctrl_init);
module_exit(g4852_ctrl_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("STE G4852 Control Driver");
MODULE_VERSION("1.1.4");
MODULE_LICENSE("GPL");
