/*
 * kernel/arch/arm/mach-omap2/board-mapphone-usb.c
 *
 * Copyright (C) 2010-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <asm/mach-types.h>

#include <plat/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/mdm6600_usb.h>
#include <plat/omap-pm.h>

#if defined(CONFIG_USB_MOT_ANDROID) && defined(CONFIG_USB_MUSB_OTG)
#include <linux/spi/cpcap.h>
#include <linux/usb/musb.h>
#endif
#include <linux/usb/oob_wake.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "cm-regbits-34xx.h"
#include "clock.h"


#define MAPPHONE_BP_READY2_AP_GPIO      59
#define MAPPHONE_IPC_USB_SUSP_GPIO	95
#define DIE_ID_REG_BASE			(L4_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218
#define DIE_ID_REG_BASE_44XX		(L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET_44XX		0x200
#define MAX_USB_SERIAL_NUM		17
#define MAPPHONE_VENDOR_ID		0x22B8
#define MAPPHONE_PRODUCT_ID		0x41D9
#define MAPPHONE_ADB_PRODUCT_ID		0x41DB
#define MAPPHONE_RNDIS_PRODUCT_ID	0x41E4
#define MAPPHONE_RNDIS_ADB_PRODUCT_ID	0x41E5
#define FACTORY_PRODUCT_ID		0x41E3
#define FACTORY_ADB_PRODUCT_ID		0x41E2

#ifdef CONFIG_USB_MOT_ANDROID
#define MAPPHONE_PHONE_PORTAL_PRODUCT_ID               0x41D8
#define MAPPHONE_PHONE_PORTAL_ADB_PRODUCT_ID           0x41DA
#define MAPPHONE_MTP_PRODUCT_ID                        0x41D6
#define MAPPHONE_MTP_ADB_PRODUCT_ID                    0x41DC
#endif

#if defined(CONFIG_USB_MOT_ANDROID) && defined(CONFIG_USB_MUSB_OTG)
struct cpcap_accy_platform_data {
	enum cpcap_accy accy;
};
#endif


static char device_serial[MAX_USB_SERIAL_NUM];



static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	"acm",
	"usbnet",
	"mtp",
#elif defined(CONFIG_USB_ANDROID_ACM)
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
};

#ifdef CONFIG_USB_MOT_ANDROID
static char *usb_functions_phone_portal[] = {
	"acm",
	"usbnet",
	"mtp",
};

static char *usb_functions_phone_portal_adb[] = {
	"acm",
	"usbnet",
	"mtp",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

static char *bp_usb_functions_bp[] = {
	"acm",
	"usbnet"
};

static char *bp_usb_functions_bp_adb[] = {
	"acm",
	"usbnet",
	"adb"
};

static char *bp_usb_functions_rndis_bp[] = {
	"rndis",
	"acm",
	"usbnet"
};

static char *bp_usb_functions_all[] = {
	"rndis",
	"acm",
	"usbnet",
	"adb"
};

#endif


static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	{
		.product_id     = MAPPHONE_PHONE_PORTAL_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal),
		.functions      = usb_functions_phone_portal,
	},
	{
		.product_id     = MAPPHONE_PHONE_PORTAL_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal_adb),
		.functions      = usb_functions_phone_portal_adb,
	},
	{
		.product_id     = MAPPHONE_MTP_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = MAPPHONE_PHONE_PORTAL_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
#endif
	{
		.product_id	= MAPPHONE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= MAPPHONE_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id	= MAPPHONE_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= MAPPHONE_RNDIS_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#endif
};

static struct android_usb_product bp_usb_products[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	{
		.product_id     = 0x7093,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp),
		.functions      = bp_usb_functions_bp,
	},
	{
		.product_id     = 0x7094,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp_adb),
		.functions      = bp_usb_functions_bp_adb,
	},
	{
		.product_id     = 0x7095,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_rndis_bp),
		.functions      = bp_usb_functions_rndis_bp,
	},
	{
		.product_id     = 0x7096,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_all),
		.functions      = bp_usb_functions_all,
	},
#endif
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id      = 0x22b8,
	.product_id     = 0x41DA,
	.product_name   = "A853",
	.manufacturer_name	= "Motorola",
	.serial_number		= device_serial,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct android_usb_platform_data andusb_plat_bp = {
	.vendor_id = 0x22b8,
	.product_id = 0x7094,
	.manufacturer_name = "Motorola",
	.serial_number		= device_serial,
	.num_products = ARRAY_SIZE(bp_usb_products),
	.products = bp_usb_products,
	.num_functions = ARRAY_SIZE(bp_usb_functions_all),
	.functions = bp_usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor			= "Motorola",
	.product		= "A853",
	.release		= 1,
	.nluns			= 1,
	.cdrom_lun_num          = 0,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usbms_plat,
	},
};


#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x22b8,
	.vendorDescr	= "Motorola",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct acm_platform_data acm_pdata = {
	/* Modify num_inst at runtime depending on boot_mode */
	.num_inst       = 2,
};

static struct platform_device acm_device = {
	.name   = "acm",
	.id     = -1,
	.dev    = {
		.platform_data = &acm_pdata,
	},
};

static int insertion_count = 1;

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	struct cpcap_accy_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->accy == CPCAP_ACCY_USB_DEVICE) {
		printk(KERN_INFO "SW:CPCAP_ACCY_USB_DEVICE Connected\n");
#if defined(CONFIG_USB_MOT_ANDROID) && defined(CONFIG_USB_MUSB_OTG)
		/*
		 * FIXME: To be removed if there is a better way to
		 * handle this. First time connection in host mode
		 * fails. Retrying again fixes the issue.
		 */
		if (insertion_count) {
			cpcap_musb_notifier_call(USB_EVENT_ID);
			msleep(5);
			cpcap_musb_notifier_call(USB_EVENT_NONE);
			msleep(5);
			insertion_count = 0;
		}
		cpcap_musb_notifier_call(USB_EVENT_ID);
#endif
	} else {
#if defined(CONFIG_USB_MOT_ANDROID) && defined(CONFIG_USB_MUSB_OTG)
		cpcap_musb_notifier_call(USB_EVENT_VBUS);
#endif
		android_usb_set_connected(1, pdata->accy);
	}

	return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
	struct cpcap_accy_platform_data *pdata = pdev->dev.platform_data;

#if defined(CONFIG_USB_MOT_ANDROID) && defined(CONFIG_USB_MUSB_OTG)
	cpcap_musb_notifier_call(USB_EVENT_NONE);
#endif

	if (pdata->accy == CPCAP_ACCY_USB_DEVICE)
		printk(KERN_INFO "SW:CPCAP_ACCY_USB_DEVICE removed \n");
	else
		android_usb_set_connected(0, pdata->accy);

	return 0;
}

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};

#ifdef CONFIG_ARM_OF
#define USB_FUNC_NAME_SIZE     20

struct omap_usb_pid_entry {
	char name[USB_FUNC_NAME_SIZE];
	u16 usb_pid;
} __attribute__ ((__packed__));

void trim_usb_name_string(char *s)
{
int i;

	/* ignore all characters behind space key */
	for (i = 0; i < USB_FUNC_NAME_SIZE; i++) {
		if (' ' == s[i]) {
			s[i] = '\0';
			return;
		}
	}

	printk(KERN_ERR "Gadget Driver - usb function name is too long!\n");
}

void __init usb_pid_mapping_init(void)
{
	struct device_node *node;
	const void *prop;
	int i, size, unit_size;
	char name[USB_FUNC_NAME_SIZE];

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		printk(KERN_ERR
			"Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}

	unit_size = sizeof(struct omap_usb_pid_entry);
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_PIDS, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
			DT_PROP_CHOSEN_USB_PIDS);
			of_node_put(node);
		return;
	}

	for (i = 0; i < size / unit_size; i++) {
		struct omap_usb_pid_entry *p =
		(struct omap_usb_pid_entry *) prop;

		memcpy((void *) name, p->name, USB_FUNC_NAME_SIZE);
		trim_usb_name_string(name);
		android_usb_set_pid(name, p->usb_pid);
		prop += unit_size;
	}

	of_node_put(node);
	printk(KERN_INFO "DT overwrite of  USB PID's done!\n");
}

void mapphone_init_cdrom_lun_num(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_CDROM_LUN_NUM, NULL);
	if (prop) {
		pr_err("USB Overwrite CDROM Lun Num with %d\n", *(char *)prop);
		usbms_plat.cdrom_lun_num = *(char *)prop;
	}

	of_node_put(node);
	return;
}

void mapphone_init_nluns(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_NLUNS, NULL);
	if (prop) {
		pr_err("USB Overwrite nLuns %d\n", *(char *)prop);
		usbms_plat.nluns = *(char *)prop;
	}

	of_node_put(node);
	return;
}

void mapphone_get_product_name(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}

	prop = of_get_property(node, DT_PROP_CHOSEN_USB_PROD_NAME, NULL);
	if (prop) {
		andusb_plat.product_name = (char *)prop;
		usbms_plat.product = (char *)prop;
	} else {
		pr_err("Read property %s error!\n",
		       DT_PROP_CHOSEN_USB_PROD_NAME);
	}

	of_node_put(node);
	return;
}

#endif

void mapphone_get_serial_number(void)
{
	unsigned int val[2];
	unsigned int reg;
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
#endif

	if (cpu_is_omap44xx())
		reg = DIE_ID_REG_BASE_44XX + DIE_ID_REG_OFFSET_44XX;
	else
		reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;

	val[0] = omap_readl(reg);

	if (cpu_is_omap44xx())
		/* OMAP4 has the id_code register at die_id_0 + 4*/
		val[1] = omap_readl(reg + 8);
	else
		val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);

#ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = device_serial;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif
}


void mapphone_gadget_init(char *boot_mode)
{
	mapphone_get_serial_number();
#ifdef CONFIG_ARM_OF
	mapphone_get_product_name();
	/* Initialize the USB nluns from device tree */
	mapphone_init_nluns();
	mapphone_init_cdrom_lun_num();
	/* Initialize the USB PID's from the device tree */
	usb_pid_mapping_init();
#endif
	if (!strncmp(boot_mode, "bp-tools", BOOT_MODE_MAX_LEN)) {
		androidusb_device.dev.platform_data = &andusb_plat_bp;
		acm_pdata.num_inst = 4;
		acm_pdata.use_iads = 1;
	}
	platform_device_register(&acm_device);

	platform_device_register(&usb_mass_storage_device);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&androidusb_device);
	platform_driver_register(&cpcap_usb_connected_driver);

}

/* USBHost Related  Board/Platform Data Starts Here*/

static int mapphone_usb_fsport_startup(void)
{
	int r;
	r = gpio_request(MAPPHONE_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
	if (r < 0) {
		printk(KERN_WARNING "Could not request GPIO %d"
		" for IPC_USB_SUSP\n",
		MAPPHONE_IPC_USB_SUSP_GPIO);
		return r;
	}
	gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
	printk(KERN_INFO "%s - Configured GPIO 95 for USB Suspend \n",
			__func__);
	return 0;
}

static int mapphone_usb_fsport_suspend(int on)
{
	if (on)
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 1);
	else
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);

	return 0;
}

static struct resource oob_wake_resources[] = {
        [0] = {
                .flags = IORESOURCE_IRQ,
        },
};

/* LTE USB Out-of-Band Wakeup Device */
static struct oob_wake_platform_data oob_wake_pdata = {
        .vendor = 0x22b8,
        .product = 0x4267,
};

static struct platform_device oob_wake_device = {
        .name   = "oob-wake",
        .id     = -1,
        .dev    = {
                .platform_data = &oob_wake_pdata,
        },
        .resource = oob_wake_resources,
        .num_resources = ARRAY_SIZE(oob_wake_resources),
};

static struct mdm6600_usb_platform_data mdm6600_usb_pdata = {
	.modem_interface = 4,
	.remote_wake_gpio = -1
};

static struct platform_device mdm6600_usb_device = {
	.name   = "mdm6600_usb",
	.id     = -1,
	.dev    = {
		.platform_data = &mdm6600_usb_pdata,
	},
};

static struct usbhs_omap_platform_data usbhs_pdata = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
	.ohci_phy_suspend = mapphone_usb_fsport_suspend
};


void __init mapphone_usbhost_init(void)
{
	struct device_node *node;
	const void *prop;
	int size, rwkup_gpio;
	int feature_usbhost = 1;

	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_usbhost_en", &size);
		if (prop && size) {
			feature_usbhost = *(u8 *)prop;
			printk(KERN_NOTICE "%s USBHost in the Platform \n",
				feature_usbhost ? "Enabling" : "Disabling");
		}
		of_node_put(node);
	}

	if (!feature_usbhost)
		return;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node) {
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT0, &size);
		if (prop && size) {
			usbhs_pdata.port_mode[0] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port0 Mode : %d\n",
				*(u8 *)prop);
		}
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT1, &size);
		if (prop && size) {
			usbhs_pdata.port_mode[1] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port1 Mode : %d\n",
				*(u8 *)prop);
		}
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT2, &size);
		if (prop && size) {
			usbhs_pdata.port_mode[2] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port2 Mode : %d\n",
				*(u8 *)prop);
		}
		of_node_put(node);
	}


	usb_uhhtll_init(&usbhs_pdata);
	mapphone_usb_fsport_startup();

	rwkup_gpio = get_gpio_by_name("mdm6600_usb_rwkup");

	if (rwkup_gpio >= 0)
		mdm6600_usb_pdata.remote_wake_gpio = rwkup_gpio;

	platform_device_register(&mdm6600_usb_device);

	rwkup_gpio = get_gpio_by_name("lte_wan_hostwake");
	oob_wake_pdata.reset_gpio = get_gpio_by_name("lte_reset_mcu");

	if (rwkup_gpio >= 0 && oob_wake_pdata.reset_gpio >= 0) {
		oob_wake_resources[0].start = gpio_to_irq(rwkup_gpio);
		oob_wake_resources[0].end = gpio_to_irq(rwkup_gpio);
		platform_device_register(&oob_wake_device);
	} else
		pr_debug("Out of Band Remote Wakeup for CDC  not defined\n");
}
