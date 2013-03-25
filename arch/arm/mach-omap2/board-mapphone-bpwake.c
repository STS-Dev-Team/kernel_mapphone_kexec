/*
 * linux/arch/arm/mach-omap2/board-mapphone-bpwake.c
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/gpio_mapping.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/prom.h>
#include "board-mapphone.h"

#ifdef CONFIG_PM
extern void omap_uart_block_sleep(int num);
#endif

static struct wake_lock baseband_wakeup_wakelock;

#define MAPPHONE_AP_UART 0

static irqreturn_t mapphone_bpwake_irqhandler(int irq, void *unused)
{
#ifdef CONFIG_PM
	omap_uart_block_sleep(MAPPHONE_AP_UART);
#endif
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	return IRQ_HANDLED;
}

static irqreturn_t mapphone_stewake_irqhandler(int irq, void *unused)
{
	int up = MAPPHONE_AP_UART;

	/*
	 *For G4852,we use uart1 for ipc transfer,uart2 for bplog.
	 */
	if (modem_is_ste_g4852())
		up = 1;
#ifdef CONFIG_PM
	omap_uart_block_sleep(up);
#endif
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	omap_uart_start_ste_ipc_tx(up);

	return IRQ_HANDLED;
}

static irqreturn_t mapphone_stelogwake_irqhandler(int irq, void *unused)
{
	if (modem_is_ste_g4852() && get_ste_bplog_mode())
#ifdef CONFIG_PM
		omap_uart_block_sleep(2);
#endif
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	omap_uart_trace_set_ap_ready();
	return IRQ_HANDLED;
}

static int mapphone_bpwake_probe(struct platform_device *pdev)
{
	int rc;
	int apwake_trigger_gpio = *(int *)(pdev->dev.platform_data);

	wake_lock_init(&baseband_wakeup_wakelock, WAKE_LOCK_SUSPEND, "bpwake");

	if (apwake_trigger_gpio >= 0) {

		gpio_request(apwake_trigger_gpio, "BP -> AP IPC trigger");
		gpio_direction_input(apwake_trigger_gpio);

		rc = request_irq(gpio_to_irq(apwake_trigger_gpio),
				mapphone_bpwake_irqhandler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"Remote Wakeup", NULL);
		if (rc) {
			wake_lock_destroy(&baseband_wakeup_wakelock);
			printk(KERN_ERR
			    "Failed requesting APWAKE_TRIGGER irq (%d)\n", rc);
			return rc;
		}
		enable_irq_wake(gpio_to_irq(apwake_trigger_gpio));

	}

	apwake_trigger_gpio = get_gpio_by_name("ste_wakeup_ap");
	if (apwake_trigger_gpio >= 0) {

		gpio_request(apwake_trigger_gpio, "STE -> AP IPC trigger");
		gpio_direction_input(apwake_trigger_gpio);

		rc = request_irq(gpio_to_irq(apwake_trigger_gpio),
				mapphone_stewake_irqhandler,
				IRQF_TRIGGER_RISING,
				"STE Remote Wakeup", NULL);
		if (rc) {
			wake_lock_destroy(&baseband_wakeup_wakelock);
			printk(KERN_ERR
			    "Failed requesting APWAKE_TRIGGER irq (%d)\n", rc);
			return rc;
		}
		enable_irq_wake(gpio_to_irq(apwake_trigger_gpio));

	}
	return 0;
}

int mapphone_ste_log_wake_enable(void)
{
	int rc;
	int ste_log_wakeup_ap_gpio;

	ste_log_wakeup_ap_gpio = get_gpio_by_name("gsm_status1");
	if (ste_log_wakeup_ap_gpio >= 0) {
		gpio_request(ste_log_wakeup_ap_gpio, "STE -> AP trace trigger");
		gpio_direction_input(ste_log_wakeup_ap_gpio);

		rc = request_irq(gpio_to_irq(ste_log_wakeup_ap_gpio),
				 mapphone_stelogwake_irqhandler,
				 IRQF_TRIGGER_FALLING,
				 "STE Trace Remote Wakeup", NULL);
		if (rc) {
			printk(KERN_ERR
			       "Failed requesting STE -> AP trace irq (%d)\n",
				 rc);
			return rc;
		}
		enable_irq_wake(gpio_to_irq(ste_log_wakeup_ap_gpio));
	}
	return 0;
}

int mapphone_ste_log_wake_disable(void)
{
	int ste_log_wakeup_ap_gpio;

	ste_log_wakeup_ap_gpio = get_gpio_by_name("gsm_status1");
	if (ste_log_wakeup_ap_gpio >= 0)
		free_irq(gpio_to_irq(ste_log_wakeup_ap_gpio), NULL);
	gpio_free(ste_log_wakeup_ap_gpio);
	return 0;
}

static int mapphone_bpwake_remove(struct platform_device *pdev)
{
	int apwake_trigger_gpio = *(int *)(pdev->dev.platform_data);

	wake_lock_destroy(&baseband_wakeup_wakelock);

	if (apwake_trigger_gpio >= 0)
		free_irq(gpio_to_irq(apwake_trigger_gpio), NULL);

	apwake_trigger_gpio = get_gpio_by_name("ste_wakeup_ap");
	if (apwake_trigger_gpio >= 0)
		free_irq(gpio_to_irq(apwake_trigger_gpio), NULL);

        gpio_free(apwake_trigger_gpio);
	return 0;
}

static int mapphone_bpwake_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int mapphone_bpwake_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mapphone_bpwake_driver = {
	.probe		= mapphone_bpwake_probe,
	.remove		= mapphone_bpwake_remove,
	.suspend	= mapphone_bpwake_suspend,
	.resume		= mapphone_bpwake_resume,
	.driver		= {
		.name		= "mapphone_bpwake",
		.owner		= THIS_MODULE,
	},
};

static int __init mapphone_bpwake_init(void)
{
	return platform_driver_register(&mapphone_bpwake_driver);
}

static void __exit mapphone_bpwake_exit(void)
{
	platform_driver_unregister(&mapphone_bpwake_driver);
}

module_init(mapphone_bpwake_init);
module_exit(mapphone_bpwake_exit);

MODULE_DESCRIPTION("Mapphone BP Wake Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola Mobility");
