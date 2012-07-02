/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Copyright (C) 2009 Nokia Corporation
 *	Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * TODO (last updated Feb 12, 2010):
 *	- add kernel-doc
 *	- enable AUTOIDLE
 *	- add suspend/resume
 *	- move workarounds to board-files
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/ulpi.h>
#include <plat/usb.h>

/* EHCI Register Set */
#define EHCI_INSNREG04					(0xA0)
#define EHCI_INSNREG04_DISABLE_UNSUSPEND		(1 << 5)
#define	EHCI_INSNREG05_ULPI				(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0
#define L3INIT_HSUSBTLL_CLKCTRL				0x4A009368
#define USB_INT_EN_RISE_CLR_0				0x4A06280F
#define USB_INT_EN_FALL_CLR_0				0x4A062812
#define USB_INT_EN_RISE_CLR_1				0x4A06290F
#define USB_INT_EN_FALL_CLR_1				0x4A062912
#define OTG_CTRL_SET_0					0x4A06280B
#define OTG_CTRL_SET_1					0x4A06290B

/*-------------------------------------------------------------------------*/

static inline void ehci_omap_writel(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 ehci_omap_readl(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

static inline void ehci_omap_writeb(void __iomem *base, u8 reg, u8 val)
{
	__raw_writeb(val, base + reg);
}

static inline u8 ehci_omap_readb(void __iomem *base, u8 reg)
{
	return __raw_readb(base + reg);
}

/*-------------------------------------------------------------------------*/

struct ehci_hcd_omap {
	struct ehci_hcd		*ehci;
	struct device		*dev;
	struct usbhs_omap_resource res;
	struct usbhs_omap_platform_data platdata;

};

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver;

static void omap_ehci_soft_phy_reset(struct ehci_hcd_omap *omap, u8 port)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned reg = 0;

	reg = ULPI_FUNC_CTRL_RESET
		/* FUNCTION_CTRL_SET register */
		| (ULPI_SET(ULPI_FUNC_CTRL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		/* Write */
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
		/* PORTn */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		/* start ULPI access*/
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_omap_writel(omap->res.regs, EHCI_INSNREG05_ULPI, reg);

	/* Wait for ULPI access completion */
	while ((ehci_omap_readl(omap->res.regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(omap->dev, "phy reset operation timed out\n");
			break;
		}
	}
}

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_probe - initialize TI-based HCDs
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int ehci_hcd_omap_probe(struct platform_device *pdev)
{
	struct uhhtll_apis *uhhtllp = pdev->dev.platform_data;
	struct ehci_hcd_omap *omap;
	struct usbhs_omap_platform_data *pdata;
	struct usbhs_omap_resource *omapresp;
	struct usb_hcd *hcd;
	int ret = -ENODEV;

	if (!uhhtllp) {
		dev_dbg(&pdev->dev, "missing platform_data\n");
		goto err_end;
	}

	if (usb_disabled())
		goto err_end;

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		ret = -ENOMEM;
		goto err_end;
	}

	pdata = &omap->platdata;
	if (uhhtllp->get_platform_data(pdata) != 0) {
		ret = -EINVAL;
		goto err_mem;
	}

	omapresp = &omap->res;
	if (uhhtllp->get_resource(OMAP_EHCI, omapresp) != 0) {
		ret = -EINVAL;
		goto err_mem;
	}

	if (!omapresp->regs) {
		dev_dbg(&pdev->dev, "failed to EHCI regs\n");
		goto err_mem;
	}

	hcd = usb_create_hcd(&ehci_omap_hc_driver, &pdev->dev,
			dev_name(&pdev->dev));
	if (!hcd) {
		dev_dbg(&pdev->dev, "failed to create hcd with err %d\n", ret);
		ret = -ENOMEM;
		goto err_mem;
	}

	platform_set_drvdata(pdev, omap);
	omap->dev		= &pdev->dev;
	omap->ehci		= hcd_to_ehci(hcd);
	omap->ehci->sbrn	= 0x20;

	hcd->rsrc_start = omapresp->start;
	hcd->rsrc_len = omapresp->len;
	hcd->regs =  omapresp->regs;

	/* we know this is the memory we want, no need to ioremap again */
	omap->ehci->caps = hcd->regs;

	ret = uhhtllp->store(OMAP_EHCI, OMAP_USB_HCD, hcd);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to store hcd\n");
		goto err_regs;
	}

	ret = uhhtllp->enable(OMAP_EHCI);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to start ehci\n");
		goto err_regs;
	}

	if (cpu_is_omap34xx()) {

		/*
		 * An undocumented "feature" in the OMAP3 EHCI controller,
		 * causes suspended ports to be taken out of suspend when
		 * the USBCMD.Run/Stop bit is cleared (for example when
		 * we do ehci_bus_suspend).
		 * This breaks suspend-resume if the root-hub is allowed
		 * to suspend. Writing 1 to this undocumented register bit
		 * disables this feature and restores normal behavior.
		 */
		ehci_omap_writel(omap->res.regs, EHCI_INSNREG04,
					EHCI_INSNREG04_DISABLE_UNSUSPEND);

		/* Soft reset the PHY using PHY reset command over ULPI */
		if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_PHY)
			omap_ehci_soft_phy_reset(omap, 0);
		if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_PHY)
			omap_ehci_soft_phy_reset(omap, 1);
	}

	if (cpu_is_omap44xx()) {

		/*
		 * Undocumented HW Errata for OMAP4 TLL
		 * the IDGND ULPI bits generate an interrupt when the
		 * controller is enabled from OFF mode. This causes the
		 * port status to be reported as disabled. Mask these
		 * interrupts since TLL does not require them.
		 */
		if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_TLL) {
			omap_writeb(0x10, USB_INT_EN_RISE_CLR_0);
			omap_writeb(0x10, USB_INT_EN_FALL_CLR_0);
			omap_writeb(0x01, OTG_CTRL_SET_0);
		}
		if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_TLL) {
			omap_writeb(0x10, USB_INT_EN_RISE_CLR_1);
			omap_writeb(0x10, USB_INT_EN_FALL_CLR_1);
			omap_writeb(0x01, OTG_CTRL_SET_1);
		}
	}

	omap->ehci->regs = hcd->regs
		+ HC_LENGTH(readl(&omap->ehci->caps->hc_capbase));

	dbg_hcs_params(omap->ehci, "reset");
	dbg_hcc_params(omap->ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	omap->ehci->hcs_params = readl(&omap->ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, omapresp->irq, IRQF_DISABLED | IRQF_SHARED);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to add hcd with err %d\n", ret);
		goto err_add_hcd;
	}

	/* root ports should always stay powered */
	ehci_port_power(omap->ehci, 1);

	return 0;

err_add_hcd:
	uhhtllp->disable(OMAP_EHCI);

err_regs:
	usb_put_hcd(hcd);

err_mem:
	kfree(omap);

err_end:
	return ret;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_remove - shutdown processing for EHCI HCDs
 * @pdev: USB Host Controller being removed
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static int ehci_hcd_omap_remove(struct platform_device *pdev)
{
	struct uhhtll_apis *uhhtllp = pdev->dev.platform_data;
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);

	if (hcd->state == HC_STATE_SUSPENDED)
		uhhtllp->resume(OMAP_EHCI);

	usb_remove_hcd(hcd);
	uhhtllp->disable(OMAP_EHCI);
	usb_put_hcd(hcd);
	kfree(omap);

	return 0;
}

static void ehci_hcd_omap_shutdown(struct platform_device *pdev)
{
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);
	struct uhhtll_apis *uhhtllp = pdev->dev.platform_data;
	int ret = 0;

	if (uhhtllp && uhhtllp->resume)
		ret = uhhtllp->resume(OMAP_EHCI);

	if (!ret && hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static int omap_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	u32 __iomem	*hostpc_reg = NULL;
	u32		temp, temp1, status;
	unsigned long	flags;
	int		retval = 0;
	u32		runstop, temp_reg;

	spin_lock_irqsave(&ehci->lock, flags);
	switch (typeReq) {
	case GetPortStatus:
		wIndex--;
		status = 0;
		temp = ehci_readl(ehci, status_reg);

		/* wPortChange bits */
		if (temp & PORT_CSC)
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		if (temp & PORT_PEC)
			status |= USB_PORT_STAT_C_ENABLE << 16;

		if ((temp & PORT_OCC) && !ignore_oc) {
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;

			/*
			 * Hubs should disable port power on over-current.
			 * However, not all EHCI implementations do this
			 * automatically, even if they _do_ support per-port
			 * power switching; they're allowed to just limit the
			 * current.  khubd will turn the power back on.
			 */
			if (HCS_PPC(ehci->hcs_params)) {
				ehci_writel(ehci,
					temp & ~(PORT_RWC_BITS | PORT_POWER),
					status_reg);
			}
		}

		/* whoever resumes must GetPortStatus to complete it!! */
		if (temp & PORT_RESUME) {

			/* Remote Wakeup received? */
			if (!ehci->reset_done[wIndex]) {
				/* resume signaling for 20 msec */
				ehci->reset_done[wIndex] = jiffies
						+ msecs_to_jiffies(20);
				/* check the port again */
				mod_timer(&ehci_to_hcd(ehci)->rh_timer,
						ehci->reset_done[wIndex]);
			}

			/* resume completed? */
			else if (time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
				clear_bit(wIndex, &ehci->suspended_ports);
				set_bit(wIndex, &ehci->port_c_suspend);
				ehci->reset_done[wIndex] = 0;

				/*
				 * Workaround for OMAP errata:
				 * To Stop Resume Signalling, it is required
				 * to Stop the Host Controller and disable the
				 * TLL Functional Clock.
				 */

				/* Stop the Host Controller */
				runstop = ehci_readl(ehci,
							&ehci->regs->command);
				ehci_writel(ehci, (runstop & ~CMD_RUN),
							&ehci->regs->command);
				(void) ehci_readl(ehci, &ehci->regs->command);
				handshake(ehci, &ehci->regs->status,
							STS_HALT,
							STS_HALT,
							2000);

				temp_reg = omap_readl(L3INIT_HSUSBTLL_CLKCTRL);
				temp_reg &= ~(1 << 9);

				/* stop resume signaling */
				temp = ehci_readl(ehci, status_reg);
				ehci_writel(ehci,
					temp & ~(PORT_RWC_BITS | PORT_RESUME),
					status_reg);

				/* Disable the Channel 1 Optional Fclk */
				omap_writel(temp_reg, L3INIT_HSUSBTLL_CLKCTRL);

				retval = handshake(ehci, status_reg,
					   PORT_RESUME, 0, 2000 /* 2msec */);

				/*
				 * Enable the Host Controller and start the
				 * Channel 1 Optional Fclk since resume has
				 * finished.
				 */
				udelay(2);
				omap_writel((temp_reg | (1 << 9)),
						L3INIT_HSUSBTLL_CLKCTRL);
				ehci_writel(ehci, (runstop),
						&ehci->regs->command);
				(void) ehci_readl(ehci, &ehci->regs->command);

				if (retval != 0) {
					ehci_err(ehci,
						"port %d resume error %d\n",
						wIndex + 1, retval);
					spin_unlock_irqrestore(&ehci->lock,
									flags);
					return -EPIPE;
				}
				temp &= ~(PORT_SUSPEND|PORT_RESUME|(3<<10));
			}
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if ((temp & PORT_RESET)
				&& time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
			status |= USB_PORT_STAT_C_RESET << 16;
			ehci->reset_done[wIndex] = 0;

			/* force reset to complete */
			ehci_writel(ehci, temp & ~(PORT_RWC_BITS | PORT_RESET),
					status_reg);
			/* REVISIT:  some hardware needs 550+ usec to clear
			 * this bit; seems too long to spin routinely...
			 */
			retval = handshake(ehci, status_reg,
					PORT_RESET, 0, 1000);
			if (retval != 0) {
				ehci_err(ehci, "port %d reset error %d\n",
					wIndex + 1, retval);
				spin_unlock_irqrestore(&ehci->lock, flags);
				return -EPIPE;
			}

			/* see what we found out */
			temp = check_reset_complete(ehci, wIndex, status_reg,
					ehci_readl(ehci, status_reg));
		}

		if (!(temp & (PORT_RESUME|PORT_RESET)))
			ehci->reset_done[wIndex] = 0;

		/* transfer dedicated ports to the companion hc */
		if ((temp & PORT_CONNECT) &&
				test_bit(wIndex, &ehci->companion_ports)) {
			temp &= ~PORT_RWC_BITS;
			temp |= PORT_OWNER;
			ehci_writel(ehci, temp, status_reg);
			ehci_dbg(ehci, "port %d --> companion\n", wIndex + 1);
			temp = ehci_readl(ehci, status_reg);
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */

		if (temp & PORT_CONNECT) {
			status |= USB_PORT_STAT_CONNECTION;
			/* status may be from integrated TT */
			if (ehci->has_hostpc) {
				temp1 = ehci_readl(ehci, hostpc_reg);
				status |= ehci_port_speed(ehci, temp1);
			} else
				status |= ehci_port_speed(ehci, temp);
		}
		if (temp & PORT_PE)
			status |= USB_PORT_STAT_ENABLE;

		/* maybe the port was unsuspended without our knowledge */
		if (temp & (PORT_SUSPEND|PORT_RESUME)) {
			status |= USB_PORT_STAT_SUSPEND;
		} else if (test_bit(wIndex, &ehci->suspended_ports)) {
			clear_bit(wIndex, &ehci->suspended_ports);
			ehci->reset_done[wIndex] = 0;
			if (temp & PORT_PE)
				set_bit(wIndex, &ehci->port_c_suspend);
		}

		if (temp & PORT_OC)
			status |= USB_PORT_STAT_OVERCURRENT;
		if (temp & PORT_RESET)
			status |= USB_PORT_STAT_RESET;
		if (temp & PORT_POWER)
			status |= USB_PORT_STAT_POWER;
		if (test_bit(wIndex, &ehci->port_c_suspend))
			status |= USB_PORT_STAT_C_SUSPEND << 16;

#ifndef	VERBOSE_DEBUG
	if (status & ~0xffff)	/* only if wPortChange is interesting */
#endif
		dbg_port(ehci, "GetStatus", wIndex + 1, temp);
		put_unaligned_le32(status, buf);
		break;
	default:
		spin_unlock_irqrestore(&ehci->lock, flags);
		retval = ehci_hub_control(hcd, typeReq, wValue, wIndex,
								buf, wLength);
		spin_lock_irqsave(&ehci->lock, flags);
	}
	spin_unlock_irqrestore(&ehci->lock, flags);
	return retval;
}



static struct platform_driver ehci_hcd_omap_driver = {
	.probe			= ehci_hcd_omap_probe,
	.remove			= ehci_hcd_omap_remove,
	.shutdown		= ehci_hcd_omap_shutdown,
	.driver = {
		.name		= "ehci-omap",
	}
};


static int ehci_omap_bus_suspend(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	struct uhhtll_apis *uhhtllp = dev->platform_data;
	int ret = 0;

	ret = ehci_bus_suspend(hcd);

	if (ret != 0)
		dev_dbg(dev, "ehci_bus_suspend failed %d\n", ret);

	if (!ret && uhhtllp && uhhtllp->suspend)
		ret = uhhtllp->suspend(OMAP_EHCI);

	return ret;
}



static int ehci_omap_bus_resume(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	struct uhhtll_apis *uhhtllp = dev->platform_data;
	int ret = 0;


	if (uhhtllp && uhhtllp->resume)
		ret = uhhtllp->resume(OMAP_EHCI);

	if (!ret)
		ret = ehci_bus_resume(hcd);

	return ret;
}



/*-------------------------------------------------------------------------*/
static const struct hc_driver ehci_omap_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "OMAP-EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= omap_ehci_hub_control,
	.bus_suspend		= ehci_omap_bus_suspend,
	.bus_resume		= ehci_omap_bus_resume,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

MODULE_ALIAS("platform:omap-ehci");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

