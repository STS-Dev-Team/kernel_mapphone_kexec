/*
 * linux/arch/arm/mach-omap2/usb-musb.c
 *
 * This file will contain the board specific details for the
 * MENTOR USB OTG controller on OMAP3430
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Vikram Pandita
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/usb/android_composite.h>

#include <linux/usb/musb.h>
#include <linux/pm_runtime.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/omap-pm.h>

#include <asm/mach-types.h>

#define CONTROL_DEV_CONF                0x300
#define PHY_PD				(1 << 0)

#ifdef CONFIG_ARCH_OMAP4
#define DIE_ID_REG_BASE         (L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET               0x200
#else
#define DIE_ID_REG_BASE         (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET               0x218
#endif /* CONFIG_ARCH_OMAP4 */

#ifdef CONFIG_USB_MUSB_SOC

#define CLKSEL_60M       24      /* Bit position */
#define CLKSEL_UTMI      (0 << CLKSEL_60M)
#define CLKSEL_ULPI      (1 << CLKSEL_60M)

#define OPTFCLKEN_XCLK   8	/* Bit Position */
#define OPTFCLK_DISABLE  (0 << OPTFCLKEN_XCLK)
#define OPTFCLK_ENABLE   (1 << OPTFCLKEN_XCLK)
#define L3INIT_HSUSBOTG_CLKCTRL   0x9360

static const char name[] = "musb_hdrc";
#define MAX_OMAP_MUSB_HWMOD_NAME_LEN	16

static struct omap_hwmod *oh_p;

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor		= "Texas Instruments Inc.",
	.product	= "OMAP4",
	.release	= 1,
	.nluns		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name		= "usb_mass_storage",
	.id		= -1,
	.dev		= {
		.platform_data = &usbms_plat,
	},
};
#endif

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	/* .clock is set dynamically */
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 50,			/* up to 100 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static int usb_idle_hwmod(struct omap_device *od)
{
	struct omap_hwmod *oh = *od->hwmods;

	if (irqs_disabled())
		_omap_hwmod_idle(oh);
	else
		omap_device_idle_hwmods(od);
	return 0;
}

static int usb_enable_hwmod(struct omap_device *od)
{
	struct omap_hwmod *oh = *od->hwmods;

	if (irqs_disabled())
		_omap_hwmod_enable(oh);
	else
		omap_device_enable_hwmods(od);
	return 0;
}

static struct omap_device_pm_latency omap_musb_latency[] = {
	  {
		.deactivate_func = usb_idle_hwmod,
		.activate_func	 = usb_enable_hwmod,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	  },
};

void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	char oh_name[MAX_OMAP_MUSB_HWMOD_NAME_LEN];
	struct omap_hwmod *oh;
	struct omap_device *od;
	struct platform_device *pdev;
	struct device	*dev;
	int l, bus_id = -1;
	struct musb_hdrc_platform_data *pdata;
	int usb_clkctrl = 0;

	if (!board_data) {
		pr_err("Board data is required for hdrc device register\n");
		return;
	}
	l = snprintf(oh_name, MAX_OMAP_MUSB_HWMOD_NAME_LEN,
						"usb_otg_hs");
	WARN(l >= MAX_OMAP_MUSB_HWMOD_NAME_LEN,
			"String buffer overflow in MUSB device setup\n");

	oh = omap_hwmod_lookup(oh_name);

	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
	} else {
		/*
		 * REVISIT: This line can be removed once all the platforms
		 * using musb_core.c have been converted to use use clkdev.
		 */
		musb_plat.clock = "ick";
		musb_plat.board_data = board_data;
		musb_plat.power = board_data->power >> 1;
		musb_plat.mode = board_data->mode;
		musb_plat.device_enable = omap_device_enable;
		musb_plat.device_idle = omap_device_idle;
		musb_plat.enable_wakeup = omap_device_enable_wakeup;
		musb_plat.disable_wakeup = omap_device_disable_wakeup;
#ifdef CONFIG_PM
		musb_plat.set_min_bus_tput = omap_pm_set_min_bus_tput;
#endif
		/*
		 * Errata 1.166 idle_req/ack is broken in omap3430
		 * workaround is to disable the autodile bit for omap3430.
		 */
		if (cpu_is_omap3430())
			oh->flags |= HWMOD_NO_OCP_AUTOIDLE;

		musb_plat.oh = oh;
		oh_p = oh;
		pdata = &musb_plat;

		od = omap_device_build(name, bus_id, oh, pdata,
				       sizeof(struct musb_hdrc_platform_data),
							omap_musb_latency,
				       ARRAY_SIZE(omap_musb_latency), false);
		if (IS_ERR(od)) {
			pr_err("Could not build omap_device for %s %s\n",
						name, oh_name);
		} else {

			pdev = &od->pdev;
			dev = &pdev->dev;
			get_device(dev);
			dev->dma_mask = &musb_dmamask;
			dev->coherent_dma_mask = musb_dmamask;
			put_device(dev);
		}

		if (machine_is_mapphone()) {
			void __iomem *l4_base = ioremap(L4_44XX_BASE, SZ_4K);

			if (WARN_ON(!l4_base))
				return;

			usb_clkctrl = __raw_readl(l4_base +
					L3INIT_HSUSBOTG_CLKCTRL);
			printk(KERN_INFO "USB MUSB Init-Initial value "
					"of CLKCTRL is 0x%x \n", usb_clkctrl);
			if (board_data->interface_type == MUSB_INTERFACE_UTMI)
				usb_clkctrl &= ~(CLKSEL_ULPI | OPTFCLK_ENABLE);
			else
				usb_clkctrl |= CLKSEL_ULPI | OPTFCLK_ENABLE;
			__raw_writel(usb_clkctrl,
					l4_base+L3INIT_HSUSBOTG_CLKCTRL);
			usb_clkctrl = __raw_readl(l4_base +
					L3INIT_HSUSBOTG_CLKCTRL);
			printk(KERN_INFO "USB MUSB Post-Initial value "
					"of CLKCTRL is 0x%x \n", usb_clkctrl);
		}

	}
		/*powerdown the phy*/
		omap_writel(PHY_PD, DIE_ID_REG_BASE + CONTROL_DEV_CONF);
}

void musb_context_save_restore(enum musb_state state)
{
	struct omap_hwmod *oh = oh_p;
	struct omap_device *od = oh->od;
	struct platform_device *pdev = &od->pdev;
	struct device *dev = &pdev->dev;
	struct device_driver *drv = dev->driver;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	struct clk *phyclk;
	struct clk *clk48m;
	struct resource	*iomem;
	void __iomem	*base;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return;

	phyclk = clk_get(NULL, "ocp2scp_usb_phy_ick");
	if (!phyclk) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_ick\n");
		return;
	}
	clk48m = clk_get(NULL, "ocp2scp_usb_phy_phy_48m");
	if (!clk48m) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_phy_48m\n");
		return;
	}

	base = ioremap(iomem->start, resource_size(iomem));
	if (!base) {
		dev_err(dev, "ioremap failed\n");
		return;
	}

	if (drv) {
#ifdef CONFIG_PM_RUNTIME
		struct musb_hdrc_platform_data *pdata = dev->platform_data;
		const struct dev_pm_ops *pm = drv->pm;

		if (plat->is_usb_active)
			if (!pdata->is_usb_active(dev)) {

				switch (state) {
				case save_context:
				/*Save the context, set the sysconfig setting
				 *  to force standby force idle during idle and
				 *  disable the clock.
				 */
				pm->suspend(dev);
				pdata->device_idle(pdev);
				break;

				case disable_clk:

				/* set the sysconfig setting to force standby,
				 * force idle during idle and disable
				 * the clock.
				 */
				if (data->interface_type ==
						MUSB_INTERFACE_UTMI) {
					/* Disable the 48Mhz phy clock and
					 * module mode
					 */
					clk_disable(phyclk);
					clk_disable(clk48m);
				}
				/* Enable ENABLEFORCE bit*/
				__raw_writel(0x1, base + 0x414);
				pdata->device_idle(pdev);
				break;

				case restore_context:
				/* Enable the clock, set the sysconfig setting
				 * back to no idle and no stndby
				 * after wakeup, restore the context.
				 */
				pdata->device_enable(pdev);
				pm->resume_noirq(dev);
				break;

				case enable_clk:
				/* set the sysconfig setting back to no idle
				 * and no stndby after wakeup and enable
				 * the clock.
				 */
				pdata->device_enable(pdev);

				if (data->interface_type ==
						MUSB_INTERFACE_UTMI) {
					/* Enable phy 48Mhz clock and module
					 * mode bit
					 */
					clk_enable(phyclk);
					clk_enable(clk48m);
				}
				/* Disable ENABLEFORCE bit*/
				__raw_writel(0x1, base + 0x414);

				break;

				default:
					break;
				}
			}
#endif

	}
	iounmap(base);
}

#else
void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	int l;

	if (board_data && board_data->interface_type == MUSB_INTERFACE_UTMI)
		/*powerdown the phy*/
		omap_writel(PHY_PD, DIE_ID_REG_BASE + CONTROL_DEV_CONF);
}
void musb_context_save_restore(enum musb_state state)
{
}
#endif /* CONFIG_USB_MUSB_SOC */
