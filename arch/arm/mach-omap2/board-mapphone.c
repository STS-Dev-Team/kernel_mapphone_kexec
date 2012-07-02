/*
 * linux/arch/arm/mach-omap2/board-mapphone.c
 *
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * Modified from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/led-cpcap-lm3554.h>
#include <linux/led-cpcap-lm3559.h>
#include <linux/gpio_mapping.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <sound/pcm.h>

#include <plat/board-mapphone.h>
#include <plat/board-mapphone-sensors.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/gpmc.h>
#include <linux/delay.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/hdq.h>
#include <mach/system.h>
#include <linux/wakelock.h>
#include <plat/hwspinlock.h>
#include <asm/hardware/cache-l2x0.h>
#include "smartreflex-class3.h"
#include <plat/voltage.h>
#include <plat/opp-cpcap.h>
#include <plat/opp-max.h>
#include <plat/hdq.h>
#include <plat/mcasp.h>

#include "cm-regbits-34xx.h"

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "prcm-common.h"
#include "cm.h"
#include "clock.h"

#ifdef CONFIG_KEYBOARD_ADP5588
#include <linux/i2c/adp5588.h>
#endif

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
#include <linux/adp5588_keypad.h>
#endif

#include <media/v4l2-int-device.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#endif
#if defined(CONFIG_VIDEO_OV8810) || defined(CONFIG_VIDEO_OV8810_MODULE)
#include <media/ov8810.h>
#endif
#if defined(CONFIG_VIDEO_OV5650) || defined(CONFIG_VIDEO_OV5650_MODULE)
#include <media/ov5650.h>
#endif

#if defined(CONFIG_LEDS_BD7885)
#include <linux/leds-bd7885.h>
#endif
#if defined(CONFIG_LEDS_BU9847)
#include <linux/leds-bu9847.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif

#ifdef CONFIG_EMU_UART_DEBUG
#include <plat/board-mapphone-emu_uart.h>
#endif

#ifdef CONFIG_INPUT_ALS_IR_ISL29030
#include <linux/isl29030.h>
#endif

#ifdef CONFIG_INPUT_ISL29032
#include <linux/isl29032.h>
#endif

#ifdef CONFIG_INPUT_CT405
#include <linux/ct405.h>
#endif

#include <../drivers/w1/w1_family.h>
#include <linux/pn544.h>

#define MAPPHONE_AUDIO_PATH_GPIO	143
#define MAPPHONE_BP_READY2_AP_GPIO	59
#define MAPPHONE_POWER_OFF_GPIO		176
#define MAPPHONE_BPWAKE_STROBE_GPIO	157
#define MAPPHONE_APWAKE_TRIGGER_GPIO	141
#define MAPPHONE_AIRC_INT_GPIO        180

#define MAPPHONE_MMCPROBE_ENABLED 0

#define CAMERA_FLASH_ID 0

/* CPCAP Defines */
#define CPCAP_SMPS_VOL_OPP1        0x02
#define CPCAP_SMPS_VOL_OPP2        0x03

/* SMPS I2C voltage control register Address*/
#define CPCAP_SRI2C_VDD_CONTROL        0x00
/* SMPS I2C Address for VDD1 */
#define CPCAP_SRI2C_SLAVE_ADDR_VDD1    0x1
/* SMPS I2C Address for VDD2 */
#define CPCAP_SRI2C_SLAVE_ADDR_VDD2    0x2
/* SMPS I2C voltage control register Address, used for SR command */
#define CPCAP_SMPS_VOL_CNTL        0x01

#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12

#define MCASP_FCLK_RATE_44100	22578947

char *bp_model = "CDMA";

static char boot_mode[BOOT_MODE_MAX_LEN+1];
int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	boot_mode[BOOT_MODE_MAX_LEN] = '\0';
	pr_debug("boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

static void __init mapphone_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
//#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
//#endif
	gic_init_irq();
	sr_class3_init();
}

#if defined(CONFIG_SND_OMAP_SOC_MCASP)
static struct omap_mcasp_configs spdif_configs[] = {
	{
	.sampling_rate = SNDRV_PCM_RATE_44100,
	.aclkxdiv = 0,
	.ahclkxdiv = 3,
	},
};

static struct omap_mcasp_platform_data mapphone_mcasp_data = {
	.mcasp_fclk_rate = MCASP_FCLK_RATE_44100,
	.mcasp_configs = spdif_configs,
	.num_configs = ARRAY_SIZE(spdif_configs),
};

static struct platform_device codec_spdif = {
	.name   = "spdif-dit",
	.id     = 0,
};

static inline void mapphone_spdif_audio_init(void)
{
	omap_init_mcasp(&mapphone_mcasp_data);
	platform_device_register(&codec_spdif);
}
#endif

#if defined(CONFIG_SND_OMAP_SOC_HDMI)
static struct platform_device mapphone_hdmi_audio_device = {
	.name = "hdmi-dai",
	.id = -1,
};

static inline void mapphone_hdmi_audio_init(void)
{
	platform_device_register(&mapphone_hdmi_audio_device);
}
#endif

#if defined(CONFIG_VIDEO_MIPI_DLI_TEST)
static struct platform_device mapphone_mipi_dli_device = {
	.name = "mipi_dli_tester",
	.id = -1,
};
#endif

/* RST_TIME1>4ms will trigger CPCAP to trigger a system cold reset */
static void mapphone_pm_set_rstime1(void)
{
	s16 prcm_mod_base = 0;
	s16 prcm_mod_offset = 0;

	if (cpu_is_omap24xx()) {
		prcm_mod_base = WKUP_MOD;
		prcm_mod_offset = OMAP2_RM_RSTTIME;
	} else if (cpu_is_omap34xx()) {
		prcm_mod_base = OMAP3430_GR_MOD;
		prcm_mod_offset = OMAP3_PRM_RSTTIME_OFFSET;
	} else if (cpu_is_omap44xx()) {
		prcm_mod_base = OMAP4430_PRM_DEVICE_MOD;
		prcm_mod_offset = OMAP4_RM_RSTTIME;
	} else
		WARN_ON(1);

	/* Configure RST_TIME1 to 6ms  */
	prm_rmw_mod_reg_bits(OMAP_RSTTIME1_MASK,
			0xc8<<OMAP_RSTTIME1_SHIFT,
			prcm_mod_base,
			prcm_mod_offset);
}

static int set_cold_reset(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	/* set cold reset rst time1 */
	u32 result = prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
	OMAP4_RM_RSTTIME);
	printk(KERN_ERR"Read RST TIME=%x \n", result);

	mapphone_pm_set_rstime1();
	result = prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD, OMAP4_RM_RSTTIME);
	printk(KERN_ERR"Reset time is configured!=%x\n", result);

	return NOTIFY_DONE;
}

static struct notifier_block mapphone_panic_notifier = {
	.notifier_call = set_cold_reset,
	.priority = 3,
};

static struct notifier_block mapphone_reboot_notifier = {
	.notifier_call = set_cold_reset,
	.priority = 3,
};

static void mapphone_rst_init(void)
{
#ifdef CONFIG_MOT_ENG_PHONE_RESET
	atomic_notifier_chain_register(&panic_notifier_list,
					&mapphone_panic_notifier);
	register_reboot_notifier(&mapphone_reboot_notifier);
#else
	set_cold_reset(NULL, 0, NULL);
#endif
}

static struct i2c_board_info __initdata mapphone_i2c_bus1_master_board_info[];
static struct i2c_board_info __initdata mapphone_i2c_bus2_master_board_info[];

extern void __init mapphone_touch_init(struct i2c_board_info *i2c_info);

static struct lm3554_platform_data mapphone_camera_flash_3554 = {
	.flags	= 0x0,
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x78,
	.flash_duration_def = 0x28,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x01,
	.gpio_reg_def = 0x0,
};

static struct lm3559_platform_data mapphone_camera_flash_3559;

static struct lm3559_platform_data mapphone_camera_flash_3559 = {
	.flags = (LM3559_PRIVACY | LM3559_TORCH |
				LM3559_FLASH | LM3559_FLASH_LIGHT |
				LM3559_MSG_IND | LM3559_ERROR_CHECK),
	.enable_reg_def = 0x00,
	.gpio_reg_def = 0x00,
	.adc_delay_reg_def = 0xc0,
	.vin_monitor_def = 0xff,
	.torch_brightness_def = 0x5b,
	.flash_brightness_def = 0xaa,
	.flash_duration_def = 0x0f,
	.flag_reg_def = 0x00,
	.config_reg_1_def = 0x6a,
	.config_reg_2_def = 0x00,
	.privacy_reg_def = 0x10,
	.msg_ind_reg_def = 0x00,
	.msg_ind_blink_reg_def = 0x1f,
	.pwm_reg_def = 0x00,
	.torch_enable_val = 0x1a,
	.flash_enable_val = 0x1b,
	.privacy_enable_val = 0x19,
	.pwm_val = 0x02,
	.msg_ind_val = 0xa0,
	.msg_ind_blink_val = 0x1f,
};

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus2_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus3_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus4_board_info[I2C_BUS_MAX_DEVICES];

struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = -1,
	.ven_gpio = -1,
	.firmware_gpio = -1,
};

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_master_board_info[] = {
	{
		I2C_BOARD_INFO("lm3532", 0x38),
		.platform_data = &mp_lm3532_pdata,
	},
	{
		I2C_BOARD_INFO("pn544", 0x28),
		.platform_data = &pn544_pdata,
		.irq = OMAP_GPIO_IRQ(31),
	},
};

static struct i2c_board_info __initdata
	mapphone_i2c_bus2_master_board_info[] = {
	{
		I2C_BOARD_INFO("invalid_touch", 0x02),
		.platform_data = NULL,
		.irq = OMAP_GPIO_IRQ(183),  /* Legacy val */
	},

#ifdef CONFIG_INPUT_ALS_IR_ISL29030
	{
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &mp_isl29030_pdata,
	},
#endif
#ifdef CONFIG_INPUT_ISL29032
	{
		I2C_BOARD_INFO(LD_ISL29032_NAME, 0x44),
		.platform_data = &mp_isl29032_pdata,
	},
#endif
#ifdef CONFIG_INPUT_CT405
	{
		I2C_BOARD_INFO(LD_CT405_NAME, 0x39),
		.platform_data = &mp_ct405_pdata,
	},
#endif
};

#ifdef CONFIG_KEYBOARD_ADP5588
extern struct adp5588_kpad_platform_data mapphone_adp5588_pdata;
#endif

static struct i2c_board_info __initdata
	mapphone_i2c_bus3_master_board_info[] = {
	{
		I2C_BOARD_INFO("lm3554_led", 0x53),
		.platform_data = &mapphone_camera_flash_3554,
	},

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	{
		I2C_BOARD_INFO("mt9p012", 0x36),
		.platform_data = &mapphone_mt9p012_platform_data,
	},
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
	{
		I2C_BOARD_INFO("HP_GEN_LENS", 0x04),
		.platform_data = &mapphone_hplens_platform_data,
	},
#endif

#ifdef CONFIG_HDMI_TDA19989
	{
		I2C_BOARD_INFO("tda19989", 0x70),
	},
#endif

#if defined(CONFIG_VIDEO_OV8810)
	{
		I2C_BOARD_INFO("ov8810", OV8810_I2C_ADDR),
		.platform_data = &mapphone_ov8810_platform_data,
	},
#endif
#if defined(CONFIG_VIDEO_OV5650)
	{
		I2C_BOARD_INFO("ov5650", OV5650_I2C_ADDR),
		.platform_data = &mapphone_ov5650_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
	{
		I2C_BOARD_INFO("HP_GEN_LENS", 0x04),
		.platform_data = &mapphone_hplens_platform_data,
	},
#endif

#if defined(CONFIG_LEDS_BD7885)
	{
		I2C_BOARD_INFO(BD7885_DEVICE_NAME, BD7885_SLAVE_ADDR),
	},
#endif	/* CONFIG_LEDS_BD7885 */

#if defined(CONFIG_LEDS_BU9847)
	{
		I2C_BOARD_INFO(BU9847_DEVICE_NAME, BU9847_SLAVE_ADDR),
	},
#endif/*CONFIG_LEDS_BU9847*/

	/* LM3559 must be the last element in the array,
		new devices need to be added above */
	{
		I2C_BOARD_INFO("lm3559_led", 0x53),
		.platform_data = &mapphone_camera_flash_3559,
	},
};

static struct i2c_board_info __initdata
	mapphone_i2c_bus4_master_board_info[] = {
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &mp_kxtf9_pdata,
	},
	{
		I2C_BOARD_INFO("lis3dh", 0x19),
		.platform_data = &mp_lis3dh_pdata,
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &mp_akm8975_pdata,
		.irq = OMAP_GPIO_IRQ(175),
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &mp_l3g4200d_pdata,
	},
	{
		I2C_BOARD_INFO("bmp085", 0x77),
		.platform_data = &mp_bmp085_pdata,
		.irq = OMAP_GPIO_IRQ(178),
	},
	{
		I2C_BOARD_INFO("msp430", 0x48),
		.platform_data = &mp_msp430_data,
	},
};

static struct i2c_board_info *get_board_info
(
	char *dev_name,
	int bus_num,
	struct i2c_board_info *board_info_table,
	int size
)
{
	int i;
	char *entry_name;

	if (dev_name != NULL && board_info_table) {
		/* search for the name in the table */
		for (i = 0; i < size; i++) {
			entry_name = \
				board_info_table[i].type;
			if ( strncmp(\
					entry_name, \
					dev_name,   \
					strlen(entry_name)) == 0)
				return &board_info_table[i];
		}
	}

	return NULL;
}

void initialize_device_specific_data(void)
{
#ifdef CONFIG_ARM_OF
	u8 dev_available = 0;
	struct device_node *node;
	int len = 0;
	const uint32_t *val;

	/* Check camera flash led type */
	/* LM3559 */
	node = of_find_node_by_path(DT_PATH_LM3559);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			dev_available =  *(u8 *)val;
	}

	if (dev_available) {
		val =
			of_get_property(
				node, "lm3559_flags", &len);
		if (val && len)
			mapphone_camera_flash_3559.flags = *val;
		else
			pr_err("%s: Can't get flags\n", __func__);
	}

	/* LM3554 */
	node = of_find_node_by_path(DT_PATH_LM3554);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			mapphone_camera_flash_3554.flags = 1;
	}

#if defined(CONFIG_VIDEO_MIPI_DLI_TEST)
	/* MIPI DLI */
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node != NULL) {
		val =
		of_get_property(node, "feature_mipi_cam", NULL);
		if (NULL != val) {
			if (*val) {
				platform_device_register(
					&mapphone_mipi_dli_device);
				printk(KERN_INFO "Enabling MIPI DLI Test");
			}
		}
	}
#endif

#endif /*CONFIG_ARM_OF*/
}

static int initialize_i2c_bus_info
(
	int bus_num,
	struct i2c_board_info *board_info,
	int info_size,
	struct i2c_board_info *master_board_info,
	int master_info_size
)
{
	int dev_cnt = 0;
#ifdef CONFIG_ARM_OF
	struct device_node *bus_node;
	const void *feat_prop;
	char *device_names;
	char dev_name[I2C_MAX_DEV_NAME_LEN];
	int device_name_len, i, j;
	struct i2c_board_info *master_entry;
	char prop_name[I2C_BUS_PROP_NAME_LEN];

	j = 0;

	bus_node = of_find_node_by_path(DT_PATH_I2C);
	if (bus_node == NULL || board_info == NULL)
		return dev_cnt;

	snprintf(prop_name, I2C_BUS_PROP_NAME_LEN,
		"bus%1ddevices", bus_num);

	feat_prop = of_get_property(bus_node,
			prop_name, NULL);
	if (NULL != feat_prop) {
		device_names = (char *)feat_prop;
		printk(KERN_INFO
			"I2C-%d devices: %s\n", bus_num, device_names);
		device_name_len = strlen(device_names);

		memset(dev_name, 0x0, I2C_MAX_DEV_NAME_LEN);

		for (i = 0; i < device_name_len; i++) {

			if (device_names[i] != '\0' &&
				device_names[i] != ',')
				dev_name[j++] = device_names[i];
			/* parse for ',' in string */
			if (device_names[i] == ',' ||
				(i == device_name_len-1)) {

				if (dev_cnt < info_size) {
					master_entry =
						get_board_info(dev_name,
							bus_num,
							master_board_info,
							master_info_size);
					if (master_entry != NULL) {
						memcpy(
							&board_info[dev_cnt++],
							master_entry,
							sizeof(
							struct i2c_board_info));
						printk(KERN_INFO
							"%s -> I2C bus-%d\n",
							master_entry->type,
							bus_num);

					}
					j = 0;
					memset(
							dev_name,
							0x0,
							I2C_MAX_DEV_NAME_LEN);
				}
			}
		}
	}
#endif
	return dev_cnt;
}

static struct omap_i2c_bus_board_data __initdata mapphone_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_4_bus_pdata;

static void __init omap_i2c_hwspinlock_init(int bus_id, struct omap_i2c_bus_board_data *pdata)
{
	pdata->handle = hwspinlock_request_specific(bus_id - 1);
	if (pdata->handle != NULL) {
		pdata->hwspinlock_lock = hwspinlock_lock;
		pdata->hwspinlock_unlock = hwspinlock_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static int __init mapphone_i2c_init(void)
{
	int i2c_bus_devices = 0;

	initialize_device_specific_data();

	omap_i2c_hwspinlock_init(1, &mapphone_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, &mapphone_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, &mapphone_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, &mapphone_i2c_4_bus_pdata);

	/* Populate I2C bus 1 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			1, mapphone_i2c_bus1_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus1_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus1_master_board_info));
	omap_register_i2c_bus(1, 400, &mapphone_i2c_1_bus_pdata,
			mapphone_i2c_bus1_board_info, i2c_bus_devices);

	/* Populate I2C bus 2 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			2, mapphone_i2c_bus2_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus2_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus2_master_board_info));
	omap_register_i2c_bus(2, 400, &mapphone_i2c_2_bus_pdata,
			mapphone_i2c_bus2_board_info, i2c_bus_devices);

	/* Populate I2C bus 3 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			3, mapphone_i2c_bus3_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus3_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus3_master_board_info));
	omap_register_i2c_bus(3, 400, &mapphone_i2c_3_bus_pdata,
			mapphone_i2c_bus3_board_info, i2c_bus_devices);

	/* Populate I2C bus 4 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			4, mapphone_i2c_bus4_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus4_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus4_master_board_info));
	omap_register_i2c_bus(4, 400, &mapphone_i2c_4_bus_pdata,
			mapphone_i2c_bus4_board_info, i2c_bus_devices);
	return 0;
}

extern void __init mapphone_flash_init(void);
extern void __init mapphone_gpio_iomux_init(void);



static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = -1,
	.dev_name = "/dev/ttyO3",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};

static struct platform_device wl128x_device = {
	.name = "kim",          /* named after init manager for ST */
	.id = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
static void __init mapphone_bt_init(void)
{
	int bt_enable_gpio;

	bt_enable_gpio = get_gpio_by_name("bt_reset_b");
	if (bt_enable_gpio < 0) {
		printk(KERN_DEBUG "mapphone_bt_init: cannot retrieve bt_reset_b gpio from device tree\n");
		bt_enable_gpio = -1;
	}
	wilink_pdata.nshutdown_gpio = bt_enable_gpio;

	platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_START   0x8E000000
#define RAM_CONSOLE_SIZE    0x20000
static struct resource ram_console_resource = {
	.start  = RAM_CONSOLE_START,
	.end    = (RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1),
	.flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = 0,
	.num_resources  = 1,
	.resource       = &ram_console_resource,
};

static inline void mapphone_ramconsole_init(void)
{
	platform_device_register(&ram_console_device);
}

static inline void omap2_ramconsole_reserve_sdram(void)
{
	reserve_bootmem(RAM_CONSOLE_START, RAM_CONSOLE_SIZE, 0);
}
#endif

static struct platform_device mapphone_bpwake_device = {
	.name		= "mapphone_bpwake",
	.id		= -1,
	.num_resources	= 0,
};

static inline void mapphone_bpwake_init(void)
{
	platform_device_register(&mapphone_bpwake_device);
}

static int power_off_gpio = -1;

static void __init mapphone_bp_model_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *bp_node;
	const void *bp_prop;

	bp_node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (bp_node) {
		bp_prop = of_get_property(bp_node, DT_PROP_CHOSEN_BP, NULL);
		if (bp_prop)
			bp_model = (char *)bp_prop;

		of_node_put(bp_node);
	}
#endif
}

static void mapphone_pm_power_off(void)
{
	printk(KERN_INFO "mapphone_pm_power_off start...\n");
	local_irq_disable();

	gpio_direction_output(power_off_gpio, 0);

	do {} while (1);

	local_irq_enable();
}


static void mapphone_pm_reset(void)
{
	arch_reset('h', NULL);
}

static int cpcap_charger_connected_probe(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_reset;
	return 0;
}

static int cpcap_charger_connected_remove(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_power_off;
	return 0;
}

static struct platform_driver cpcap_charger_connected_driver = {
	.probe		= cpcap_charger_connected_probe,
	.remove		= cpcap_charger_connected_remove,
	.driver		= {
		.name	= "cpcap_charger_connected",
		.owner	= THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	power_off_gpio = get_gpio_by_name("power_off");
	if (power_off_gpio < 0) {
		printk(KERN_INFO "Can't get power_off gpio from devtree\n");
		power_off_gpio = MAPPHONE_POWER_OFF_GPIO;
	}

	gpio_request(power_off_gpio, "mapphone power off");
	gpio_direction_output(power_off_gpio, 1);

	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode                   = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode                   = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode                   = MUSB_PERIPHERAL,
#endif
	.power                  = 100,
};

static void __init mapphone_musb_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int use_utmi = 0;
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);

	if (node) {
		prop = of_get_property(node, "feature_musb_utmi", &size);
		if (prop && size) {
			use_utmi = *(u8 *)prop;
			pr_debug("Using %s as the MUSB Mode \n",
				use_utmi ? "UTMI" : "ULPI");

		} else
			pr_debug("USB Defaulting to ULPI \n");
		of_node_put(node);
	}

	if (use_utmi)
		musb_board_data.interface_type = MUSB_INTERFACE_UTMI;

	usb_musb_init(&musb_board_data);
}

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "cpcap",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x44,
	.i2c_vreg = 0x00,
	.i2c_cmdreg = 0x01,
	.vsel_to_uv = omap_cpcap_vsel_to_uv,
	.uv_to_vsel = omap_cpcap_uv_to_vsel,
	.onforce_cmd = omap_cpcap_onforce_cmd,
	.on_cmd = omap_cpcap_on_cmd,
	.sleepforce_cmd = omap_cpcap_sleepforce_cmd,
	.sleep_cmd = omap_cpcap_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x3c,
	.vc_offmode_chnl_cmd = 1,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "max8952",
	.slew_rate = 16000,
	.step_size = 10000,
	.i2c_addr = 0x60,
	.i2c_vreg = 0x03,
	.i2c_cmdreg = 0x03,
	.vsel_to_uv = omap_max8952_vsel_to_uv,
	.uv_to_vsel = omap_max8952_uv_to_vsel,
	.onforce_cmd = omap_max8952_onforce_cmd,
	.on_cmd = omap_max8952_on_cmd,
	.sleepforce_cmd = omap_max8952_sleepforce_cmd,
	.sleep_cmd = omap_max8952_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x0d,
	.vp_vlimitto_vddmax = 0x3a,
	.vc_offmode_chnl_cmd = 0,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "cpcap",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x02,
	.i2c_vreg = 0x00,
	.i2c_cmdreg = 0x01,
	.vsel_to_uv = omap_cpcap_vsel_to_uv,
	.uv_to_vsel = omap_cpcap_uv_to_vsel,
	.onforce_cmd = omap_cpcap_onforce_cmd,
	.on_cmd = omap_cpcap_on_cmd,
	.sleepforce_cmd = omap_cpcap_sleepforce_cmd,
	.sleep_cmd = omap_cpcap_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x3c,
	.vc_offmode_chnl_cmd = 1,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1375000,        /* 1.375v */
	.vdd0_onlp = 1375000,      /* 1.375v */
	.vdd0_ret = 837500,       /* 0.8375v */
	.vdd0_off = 770000,       /* 0.77v */
	.vdd0_oncmd = 0x3a,
	.vdd0_offcmd = 0x0d,
	.vdd1_on = 1300000,        /* 1.3v */
	.vdd1_onlp = 1300000,      /* 1.3v */
	.vdd1_ret = 837500,       /* 0.8375v */
	.vdd1_off = 600000,       /* 0.6v */
	.vdd1_oncmd = 8,
	.vdd1_offcmd = 0,
	.vdd2_on = 1125000,        /* 1.125v */
	.vdd2_onlp = 1125000,      /* 1.125v */
	.vdd2_ret = 837500,       /* .8375v */
	.vdd2_off = 600000,       /* 0.6v */
	.vdd2_oncmd = 8,
	.vdd2_offcmd = 0,
};

static void omap_lpddr2_init(void)
{
	u32 reg;
	u16 control_lpddr2_io_1_3;
	u16 control_lpddr2_io_2_3;

	control_lpddr2_io_1_3 = OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO1_3;
	control_lpddr2_io_2_3 = OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO2_3;

	reg = omap4_ctrl_pad_readl(control_lpddr2_io_1_3);
	/* Clear last 4bits to disable the VERF_EN_CA/VERF_EN_QA.
	   This will save about 2mA current drain in RET mode   */
	reg &= 0xfffffff0;
	omap4_ctrl_pad_writel(reg, control_lpddr2_io_1_3);

	reg = omap4_ctrl_pad_readl(control_lpddr2_io_2_3);
	reg &= 0xfffffff0;
	omap4_ctrl_pad_writel(reg, control_lpddr2_io_2_3);
}

static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &elpida_2G_S4,
};




static void __init mapphone_voltage_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	const uint32_t *val;
#endif
	int size;

#ifdef CONFIG_ARM_OF
	node = of_find_node_by_path("/System@0/PwrMgmt@0");
	if (node) {
		prop = of_get_property(node, "i2c_addr_iva", &size);
		if (prop && size)
			omap_pmic_iva.i2c_addr = *(u8 *)prop;
		prop = of_get_property(node, "i2c_addr_core", &size);
		if (prop && size)
			omap_pmic_core.i2c_addr = *(u8 *)prop;

		/* The enable_off_mode must be set before omap4_pm_init */
		val = of_get_property(node, "enable_off_mode", NULL);
		if (val != NULL) {
			if (*val)
				enable_off_mode = 1;
		}
		of_node_put(node);
	}
#endif

	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");

	omap_voltage_init_vc(&vc_config);
}
static unsigned int __init get_ap_ddr_size(void)
{
	/*If no ddr size defined, default value is 512 MB*/
	unsigned int sz = 512;

#ifdef CONFIG_ARM_OF
	struct device_node *sz_node;
	const void *sz_prop;

	sz_node = of_find_node_by_path(DT_PATH_CHOSEN);

	if (sz_node != NULL) {
		sz_prop = of_get_property(sz_node, \
			DT_PROP_CHOSEN_AP_DDR_SIZE, NULL);
		if (sz_prop)
			sz = *(unsigned int *)sz_prop;
		of_node_put(sz_node);
	} else
		printk(KERN_ERR"%s can not find Chosen@0 node\n", __func__);
#endif

	printk(KERN_INFO"%s: AP DDR size = %d MB\n", __func__, sz);
	return sz;
}

static int no_trimmed_panic(struct notifier_block *this,
						unsigned long event, void *ptr)
{
	check_omap4430_trim_chip();
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = no_trimmed_panic,
	.priority = 1,
};

static void __init mapphone_pn544_init(void)
{
	int gpio = -1;

#ifdef CONFIG_ARM_OF
	gpio = get_gpio_by_name("pn544_irq");
	if (gpio < 0) {
		printk(KERN_DEBUG "mapphone_pn544_init: cannot retrieve pn544_irq gpio from device tree\n");
		pn544_pdata.irq_gpio = -1;
		return;
	}
	pn544_pdata.irq_gpio = gpio;

	gpio = get_gpio_by_name("pn544_reset");
	if (gpio < 0) {
		printk(KERN_DEBUG "mapphone_pn544_init: cannot retrieve pn544_reset gpio from device tree\n");
		pn544_pdata.ven_gpio = -1;
		pn544_pdata.irq_gpio = -1;
		return;
	}
	pn544_pdata.ven_gpio = gpio;

	gpio = get_gpio_by_name("pn544_firmware");
	if (gpio < 0) {
		printk(KERN_DEBUG "mapphone_pn544_init: cannot retrieve pn544_firmware gpio from device tree\n");
		pn544_pdata.firmware_gpio = -1;
		return;
	}
	pn544_pdata.firmware_gpio = gpio;
#endif
}

#ifdef CONFIG_OMAP_HSI
extern void __init omap_init_hsi(void);

static void __init mapphone_hsi_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int feature_hsi = 0;

#ifdef CONFIG_ARM_OF
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_hsi", &size);
		if (prop && size) {
			feature_hsi = *(u8 *)prop;
			pr_debug("%s HSI is in the Platform \n",
				feature_hsi ? "Enabling" : "Disabling");
		}
		of_node_put(node);
	}
#endif
	if (feature_hsi)
		omap_init_hsi();
}
#endif

static void __init mapphone_init(void)
{
	printk(KERN_INFO "Mapphone Init Started \n");
	mapphone_rst_init();
	omap_lpddr2_init();
	if (get_ap_ddr_size() > 512)
		emif_devices.cs1_device = &elpida_2G_S4;
	omap_emif_setup_device_details(&emif_devices, &emif_devices);
	omap_init_emif_timings();
	/* gpio_mapping_init() must run before touch_init() */
	mapphone_gpio_mapping_init();
	/* touch_init() must run before i2c_init() */
	mapphone_touch_init(&mapphone_i2c_bus2_master_board_info[0]);

	mapphone_i2c_init();
	mapphone_voltage_init();
	mapphone_bp_model_init();
	mapphone_omap44xx_padconf_init();
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	mapphone_ramconsole_init();
#endif
	mapphone_mdm_ctrl_init();
	mapphone_spi_init(boot_mode);
	mapphone_cpcap_client_init();
	mapphone_serial_init();
	mapphone_panel_init();
	mapphone_sensors_init();
	mapphone_vibrator_init();
	mapphone_pn544_init();
	mapphone_usbhost_init();
	mapphone_musb_init();
#ifdef CONFIG_OMAP_HSI
	mapphone_hsi_init();
#endif
	omap_hdq1w_init(&mapphone_hdq_data);
	mapphone_bt_init();
	mapphone_hsmmc_init();
	mapphone_power_off_init();
	mapphone_gadget_init(boot_mode);
	mapphone_bpwake_init();
#ifdef CONFIG_SND_OMAP_SOC_HDMI
	mapphone_hdmi_audio_init();
#endif
#if defined(CONFIG_ISPM7MO)
	mapphone_ispm7mo_init();
#endif
#ifdef CONFIG_SND_OMAP_SOC_MCASP
	mapphone_spdif_audio_init();
#endif

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
}

/**
 * mapphone_pm_off_mode_init
 *
 * Initializes the device off mode.
 *
 * Make sure omap4_pm_off_mode_enable is called after
 * omap4_pm_init which is defined as late_initcall,
 * define it as a late_initcall_sync function.
 */
static int __init mapphone_pm_off_mode_init(void)
{
	/* Init the omap4 pm off mode */
	if (cpu_is_omap44xx())
		omap4_pm_off_mode_enable(enable_off_mode);

	return 0;
}
late_initcall_sync(mapphone_pm_off_mode_init);

static void __init mapphone_map_io(void)
{
	int ret;

	/* reserve the top 52MB of SDRAM for the ducatti */
	ret = reserve_bootmem(0x9CC00000, 52*1024*1024, BOOTMEM_EXCLUSIVE);
	if (ret < 0)
		pr_err("Failed to reserve Ducatti memory: %d\n", ret);

	/* reserve 3MB of SDRAM at 0x9C900000 for security module */
	ret = reserve_bootmem(0x9C900000, 3*1024*1024, BOOTMEM_EXCLUSIVE);
	if (ret < 0)
		pr_err("Failed to reserve SMC memory: %d\n", ret);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	omap2_ramconsole_reserve_sdram();
#endif
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(MAPPHONE, "mapphone_")
	/* Maintainer: Motorola, Inc. */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= mapphone_map_io,
	.init_irq	= mapphone_init_irq,
	.init_machine	= mapphone_init,
	.timer		= &omap_timer,
MACHINE_END
