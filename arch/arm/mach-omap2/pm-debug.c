/*
 * OMAP Power Management debug routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#include <plat/clock.h>
#include <plat/board.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/voltage.h>
#include <plat/dmtimer.h>
#include <plat/control.h>

#include <mach/omap4-common.h>

#include "prm.h"
#include "cm.h"
#include "pm.h"

#ifdef CONFIG_ARCH_OMAP4
#define WAKEUP_SOURCE_LEN 512
#define OMAP4_WKG_ENB_A_0		0x10
#define GIC_DIST_PENDING_SET		0x200
#define OMAP4_GPIO_IRQSTATUS0		0x002c
#define OMAP4_GPIO_IRQWAKEN0		0x0044
#define OMAP4_GPIO1_BASE		0x4A310000
#define OMAP4_GPIO2_BASE		0x48055000
#define CONTROL_PADCONF_UART3_RX	0x4A100144

static int get_wakeup_gpio(int bank, char *pbuf, char* buf)
{
	u32 gpio_num = 0;
	u32 base;
	int len;
	u32 val;

	if (bank == 1)
		base = OMAP4_GPIO1_BASE;
	else
		base = OMAP4_GPIO2_BASE + (bank-2)*0x2000;

	gpio_num = omap_readl(base + OMAP4_GPIO_IRQSTATUS0)
			& omap_readl(base + OMAP4_GPIO_IRQWAKEN0);

	len = WAKEUP_SOURCE_LEN - (pbuf - buf);

	val = constant_fls(gpio_num) - 1 + 32*(bank - 1);

	if (gpio_num && len > 11)
		pbuf += snprintf(pbuf, len, ",GPIO_%d", val);

	return gpio_num;
}

void pm_dbg_show_wakeup_source(void)
{
	static char buf[WAKEUP_SOURCE_LEN];
	char *pbuf;
	int len;
	int i;
	u32 io_st = 0;
	u32 val = 0;
	u32 pending_irqs[5];
	u32 gpio_num = 0;
	u32 gpio_module = 0;
	u32 wakeup_irq = 0;
	int wv;

	memset(buf, 0, WAKEUP_SOURCE_LEN);
	pbuf = buf;
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (len > 16)
		pbuf += snprintf(pbuf, len, "AWOKEN BY:");

	/* get all pending interrupts */
	for (i = 0; i < 5; i++) {
		pending_irqs[i] = readl(gic_dist_base_addr +
				GIC_DIST_PENDING_SET + i*4);

	}

	/* get the wakeup interrupt number */
	for (i = 1; i < 5; i++) {
		val = readl(wakeupgen_base + OMAP4_WKG_ENB_A_0 + (i - 1) * 4);
		wakeup_irq = pending_irqs[i] & val;
		if (wakeup_irq) {
			val = constant_fls(wakeup_irq) + 32 * i - 1;
			len = WAKEUP_SOURCE_LEN - (pbuf - buf);
			if (wakeup_irq && len > 12)
				pbuf += snprintf(pbuf, len, "IRQ(%d)", val);

			break;
		}

	}

	/* check io_st bit in PRM_IRQSTATUS_MPU_A9 */
	val = prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
				OMAP4_PRM_IRQSTATUS_MPU_OFFSET);
	io_st = val & (1<<9);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (len > 50)
		pbuf += snprintf(pbuf, len, ",MPU_IRQSTATUS_A9(0x%x),IO_ST(%d)",
				val, io_st>>9);

	val = prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
				OMAP4_PRM_IRQSTATUS_MPU_2_OFFSET);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (len > 50)
		pbuf += snprintf(pbuf, len, ",MPU_IRQSTATUS_A9_2(0x%x)", val);


	for (i = 0; i < 7; i++) {
		wv = OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_0 + i * 4;
		val = omap4_ctrl_pad_readl(wv);
		if (val) {
			len = WAKEUP_SOURCE_LEN - (pbuf - buf);
			if (len > 30)
				pbuf += snprintf(pbuf, len,
						",CORE_WKEVENT_%d(0x%x)",
						i, val);
		}
	}

	wv = OMAP4_CTRL_MODULE_PAD_WKUP_PADCONF_WAKEUPEVENT_0;
	val = omap_readl(OMAP4_CTRL_MODULE_PAD_WKUP + wv);
	if (val) {
		len = WAKEUP_SOURCE_LEN - (pbuf - buf);
		if (len > 30)
			pbuf += snprintf(pbuf, len, ",WKUP_WKEVENT(0x%x)", val);
	}

	/* check GPIO1(PD_WAKEUP),no matter IO chain */
	gpio_module = (pending_irqs[1] & (1 << 29)); /* MA_IRQ_29 */
	if (gpio_module) {
		/*  GPIO1_IRQWAKEN & GPIO_IRQSTATUS */
		gpio_num = get_wakeup_gpio(1, pbuf, buf);

	}

	/* GPIO2~6 should be working with IO pad chain enabled */
	if (io_st) {
		int ma_irq = 30; /* MA_IRQ_30 for GPIO2*/
		int group = 0;
		int mask;
		for (i = 2; i <= 6; i++, ma_irq++) {
			mask = ma_irq % 32;
			group = ma_irq > 31 ? 2 : 1;
			gpio_module = pending_irqs[group] & (1 << mask);
			if (gpio_module)
				gpio_num = get_wakeup_gpio(i, pbuf, buf);
		}

	}

	if (io_st && !gpio_num) {
		/* possible it is the serial console, check UART3_RX */
		val = omap_readl(CONTROL_PADCONF_UART3_RX);
		val = (!(val & 0x3)) && (val & (1 << 15));
		len = WAKEUP_SOURCE_LEN - (pbuf - buf);
		if (val && len > 6)
			pbuf += snprintf(pbuf, len, ",UART3");
		else if (!val && len > 13)
			pbuf += snprintf(pbuf, len, ",OTHER_IO_PAD");
	}

	printk(KERN_INFO "%s\n", buf);

}
EXPORT_SYMBOL(pm_dbg_show_wakeup_source);
#endif

#ifdef CONFIG_PM_DEBUG
int omap2_pm_debug;
u32 sleep_while_idle = 1;
u32 omap4_device_off_counter = 0;
int pmd_clks_enable;
int dpll_cascade_global_state;

#define DUMP_PRM_MOD_REG(mod, reg)    \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = prm_read_mod_reg(mod, reg)
#define DUMP_CM_MOD_REG(mod, reg)     \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = cm_read_mod_reg(mod, reg)
#define DUMP_PRM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_CM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_INTC_REG(reg, off) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = \
			 __raw_readl(OMAP2_L4_IO_ADDRESS(0x480fe000 + (off)))

void omap2_pm_dump(int mode, int resume, unsigned int us)
{
	struct reg {
		const char *name;
		u32 val;
	} regs[32];
	int reg_count = 0, i;
	const char *s1 = NULL, *s2 = NULL;

	if (!resume) {
#if 0
		/* MPU */
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRM_IRQENABLE_MPU_OFFSET);
		DUMP_CM_MOD_REG(MPU_MOD, OMAP2_CM_CLKSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, OMAP2_PM_PWSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, OMAP2_PM_PWSTST);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_WKDEP);
#endif
#if 0
		/* INTC */
		DUMP_INTC_REG(INTC_MIR0, 0x0084);
		DUMP_INTC_REG(INTC_MIR1, 0x00a4);
		DUMP_INTC_REG(INTC_MIR2, 0x00c4);
#endif
#if 0
		DUMP_CM_MOD_REG(CORE_MOD, CM_FCLKEN1);
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(CORE_MOD, OMAP24XX_CM_FCLKEN2);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKEMUL_CTRL_OFFSET);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKSRC_CTRL_OFFSET);
		}
		DUMP_CM_MOD_REG(WKUP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_CLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_AUTOIDLE);
		DUMP_PRM_MOD_REG(CORE_MOD, OMAP2_PM_PWSTST);
#endif
#if 0
		/* DSP */
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_FCLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_ICLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_IDLEST);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_AUTOIDLE);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSEL);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_CM_CLKSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_RM_RSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_RM_RSTST);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_PM_PWSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_PM_PWSTST);
		}
#endif
	} else {
		DUMP_PRM_MOD_REG(CORE_MOD, PM_WKST1);
		if (cpu_is_omap24xx())
			DUMP_PRM_MOD_REG(CORE_MOD, OMAP24XX_PM_WKST2);
		DUMP_PRM_MOD_REG(WKUP_MOD, PM_WKST);
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRCM_IRQSTATUS_MPU_OFFSET);
#if 1
		DUMP_INTC_REG(INTC_PENDING_IRQ0, 0x0098);
		DUMP_INTC_REG(INTC_PENDING_IRQ1, 0x00b8);
		DUMP_INTC_REG(INTC_PENDING_IRQ2, 0x00d8);
#endif
	}

	switch (mode) {
	case 0:
		s1 = "full";
		s2 = "retention";
		break;
	case 1:
		s1 = "MPU";
		s2 = "retention";
		break;
	case 2:
		s1 = "MPU";
		s2 = "idle";
		break;
	}

	if (!resume)
#ifdef CONFIG_NO_HZ
		printk(KERN_INFO
		       "--- Going to %s %s (next timer after %u ms)\n", s1, s2,
		       jiffies_to_msecs(get_next_timer_interrupt(jiffies) -
					jiffies));
#else
		printk(KERN_INFO "--- Going to %s %s\n", s1, s2);
#endif
	else
		printk(KERN_INFO "--- Woke up (slept for %u.%03u ms)\n",
			us / 1000, us % 1000);

	for (i = 0; i < reg_count; i++)
		printk(KERN_INFO "%-20s: 0x%08x\n", regs[i].name, regs[i].val);
}
#endif

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void pm_dbg_regset_store(u32 *ptr);

static struct dentry *pm_dbg_dir;
struct dentry  *pm_dbg_main_dir;
static int pm_dbg_init_done;

static int __init pm_dbg_init(void);

enum {
	DEBUG_FILE_COUNTERS = 0,
	DEBUG_FILE_TIMERS,
};

struct pm_module_def {
	char name[16]; /* Name of the module */
	short type; /* CM or PRM */
	unsigned short offset;
	int low; /* First register address on this module */
	int high; /* Last register address on this module */
};

#define MOD_CM 0
#define MOD_PRM 1

static const struct pm_module_def *pm_dbg_reg_modules;
static const struct pm_module_def omap3_pm_reg_modules[] = {
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0, 0x4c },
	{ "OCP", MOD_CM, OCP_MOD, 0, 0x10 },
	{ "MPU", MOD_CM, MPU_MOD, 4, 0x4c },
	{ "CORE", MOD_CM, CORE_MOD, 0, 0x4c },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0, 0x4c },
	{ "WKUP", MOD_CM, WKUP_MOD, 0, 0x40 },
	{ "CCR", MOD_CM, PLL_MOD, 0, 0x70 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0, 0x4c },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0, 0x4c },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0, 0x4c },
	{ "EMU", MOD_CM, OMAP3430_EMU_MOD, 0x40, 0x54 },
	{ "NEON", MOD_CM, OMAP3430_NEON_MOD, 0x20, 0x48 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0, 0x4c },

	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0x50, 0xfc },
	{ "OCP", MOD_PRM, OCP_MOD, 4, 0x1c },
	{ "MPU", MOD_PRM, MPU_MOD, 0x58, 0xe8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0x58, 0xf8 },
	{ "SGX", MOD_PRM, OMAP3430ES2_SGX_MOD, 0x58, 0xe8 },
	{ "WKUP", MOD_PRM, WKUP_MOD, 0xa0, 0xb0 },
	{ "CCR", MOD_PRM, PLL_MOD, 0x40, 0x70 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0x58, 0xe8 },
	{ "CAM", MOD_PRM, OMAP3430_CAM_MOD, 0x58, 0xe8 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0x58, 0xe8 },
	{ "EMU", MOD_PRM, OMAP3430_EMU_MOD, 0x58, 0xe4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x20, 0xe4 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0x58, 0xe8 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0x58, 0xe8 },
	{ "", 0, 0, 0, 0 },
};

static const struct pm_module_def omap4_pm_reg_modules[] = {
	{ "OCP_CM1", MOD_CM, OMAP4430_CM1_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN_CM1", MOD_CM, OMAP4430_CM1_CKGEN_MOD, 0x0, 0x180 },
	{ "MPU", MOD_CM, OMAP4430_CM1_MPU_MOD, 0x0, 0x20 },
	{ "TESLA", MOD_CM, OMAP4430_CM1_TESLA_MOD, 0x0, 0x20 },
	{ "ABE", MOD_CM, OMAP4430_CM1_ABE_MOD, 0x0, 0x88 },
	{ "RESTORE", MOD_CM, OMAP4430_CM1_RESTORE_MOD, 0x0, 0x40 },

	{ "OCP_CM2", MOD_CM, OMAP4430_CM2_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN_CM2", MOD_CM, OMAP4430_CM2_CKGEN_MOD, 0x0, 0xec },
	{ "ALWAYS_ON", MOD_CM, OMAP4430_CM2_ALWAYS_ON_MOD, 0x0, 0x40 },
	{ "CORE", MOD_CM, OMAP4430_CM2_CORE_MOD, 0x0, 0x740 },
	{ "IVAHD", MOD_CM, OMAP4430_CM2_IVAHD_MOD, 0x0, 0x28 },
	{ "CAM", MOD_CM, OMAP4430_CM2_CAM_MOD, 0x0, 0x28 },
	{ "DSS", MOD_CM, OMAP4430_CM2_DSS_MOD, 0x0, 0x28 },
	{ "GFX", MOD_CM, OMAP4430_CM2_GFX_MOD, 0x0, 0x20 },
	{ "L3INIT", MOD_CM, OMAP4430_CM2_L3INIT_MOD, 0x0, 0xe0 },
	{ "L4PER", MOD_CM, OMAP4430_CM2_L4PER_MOD, 0x0, 0x1d8 },
	{ "CEFUSE", MOD_CM, OMAP4430_CM2_CEFUSE_MOD, 0x0, 0x20 },
	{ "RESTORE", MOD_CM, OMAP4430_CM2_RESTORE_MOD, 0x0, 0x5c },

	{ "EMU_CM", MOD_CM, OMAP4430_PRM_EMU_CM_MOD, 0x0, 0x20 },
	{ "WKUP_CM", MOD_CM, OMAP4430_PRM_WKUP_CM_MOD, 0x0, 0x88 },

	{ "OCP", MOD_PRM, OMAP4430_PRM_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN", MOD_PRM, OMAP4430_PRM_CKGEN_MOD, 0x0, 0x10 },
	{ "MPU", MOD_PRM, OMAP4430_PRM_MPU_MOD, 0x0, 0x24 },
	{ "TESLA", MOD_PRM, OMAP4430_PRM_TESLA_MOD, 0x0, 0x24 },
	{ "ABE", MOD_PRM, OMAP4430_PRM_ABE_MOD, 0x0, 0x8c },
	{ "ALWAYS_ON", MOD_PRM, OMAP4430_PRM_ALWAYS_ON_MOD, 0x24, 0x3c },
	{ "CORE", MOD_PRM, OMAP4430_PRM_CORE_MOD, 0x0, 0x744 },
	{ "IVAHD", MOD_PRM, OMAP4430_PRM_IVAHD_MOD, 0x0, 0x2c },
	{ "CAM", MOD_PRM, OMAP4430_PRM_CAM_MOD, 0x0, 0x2c },
	{ "DSS", MOD_PRM, OMAP4430_PRM_DSS_MOD, 0x0, 0x2c },
	{ "GFX", MOD_PRM, OMAP4430_PRM_GFX_MOD, 0x0, 0x24 },
	{ "L3INIT", MOD_PRM, OMAP4430_PRM_L3INIT_MOD, 0x0, 0xe4 },
	{ "L4PER", MOD_PRM, OMAP4430_PRM_L4PER_MOD, 0x0, 0x1dc },
	{ "CEFUSE", MOD_PRM, OMAP4430_PRM_CEFUSE_MOD, 0x0, 0x24 },
	{ "WKUP", MOD_PRM, OMAP4430_PRM_WKUP_MOD, 0x24, 0x84 },
	{ "EMU", MOD_PRM, OMAP4430_PRM_EMU_MOD, 0x0, 0x24 },
	{ "DEVICE", MOD_PRM, OMAP4430_PRM_DEVICE_MOD, 0x0, 0xf8 },

	{ "MPU_OCP", MOD_PRM, OMAP4430_PRCM_MPU_OCP_SOCKET_PRCM_MOD, 0x0, 0x0 },
	{ "MPU_DEVICE", MOD_PRM, OMAP4430_PRCM_MPU_DEVICE_PRM_MOD, 0x0, 0x4 },
	{ "MPU_CPU0", MOD_PRM, OMAP4430_PRCM_MPU_CPU0_MOD, 0x0, 0x18 },
	{ "MPU_CPU1", MOD_PRM, OMAP4430_PRCM_MPU_CPU1_MOD, 0x0, 0x18 },
	{ "", 0, 0, 0, 0 },
};

#define OMAP3_PADCONF_BASE		0x48002000
#define OMAP4_PADCONF_CORE_BASE		0x4A100000
#define OMAP4_PADCONF_WKUP_BASE		0x4A31E000
#define PADCONF_WAKEUP_ENABLE_MASK	(1 << 14)

struct pad_range {
	unsigned int start;
	unsigned int end;
	unsigned int base;
};

static const struct pad_range omap3_pad_range[] = {
	{ 0x0030, 0x014C, OMAP3_PADCONF_BASE },
	{ 0x0158, 0x0264, OMAP3_PADCONF_BASE },
	{ 0x05A0, 0x05F8, OMAP3_PADCONF_BASE },
	{ 0x0A00, 0x0A24, OMAP3_PADCONF_BASE },
	{ 0x0A4C, 0x0A58, OMAP3_PADCONF_BASE },
};

static const struct pad_range omap4_pad_range[] = {
	{ 0x0040, 0x01d4, OMAP4_PADCONF_CORE_BASE },
	{ 0x0040, 0x0074, OMAP4_PADCONF_WKUP_BASE },
};


#define PM_DBG_MAX_REG_SETS 4

static void *pm_dbg_reg_set[PM_DBG_MAX_REG_SETS];


static int pm_dbg_get_regset_size(void)
{
	static int regset_size;

	if (regset_size == 0) {
		int i = 0;

		while (pm_dbg_reg_modules[i].name[0] != 0) {
			regset_size += pm_dbg_reg_modules[i].high +
				4 - pm_dbg_reg_modules[i].low;
			i++;
		}
	}
	return regset_size;
}

static int pm_dbg_show_regs(struct seq_file *s, void *unused)
{
	int i, j;
	unsigned long val;
	int reg_set = (int)s->private;
	u32 *ptr;
	void *store = NULL;
	int regs;
	int linefeed;
	u32 base_mask, base_addr;

	if (reg_set == 0) {
		store = kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);
		if (!store) {
			WARN_ON(1);
			return -ENOMEM;
		}
		ptr = store;
		pm_dbg_regset_store(ptr);
	} else {
		ptr = pm_dbg_reg_set[reg_set - 1];
	}

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		regs = 0;
		linefeed = 0;
		if (cpu_is_omap34xx())
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				seq_printf(s, "MOD: CM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(OMAP3430_CM_BASE +
					pm_dbg_reg_modules[i].offset));
			else
				seq_printf(s, "MOD: PRM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(OMAP3430_PRM_BASE +
					pm_dbg_reg_modules[i].offset));
		else if (cpu_is_omap44xx()) {
			base_mask = abs(pm_dbg_reg_modules[i].offset) >> BASE_ID_SHIFT;
			if (pm_dbg_reg_modules[i].type == MOD_CM) {
				if (base_mask == PRM_BASE)
					base_addr = OMAP4430_PRM_BASE;
				else if (base_mask == CM2_BASE)
					base_addr = OMAP4430_CM2_BASE;
				else
					base_addr = OMAP4430_CM_BASE;
				seq_printf(s, "MOD: CM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(base_addr +
					(pm_dbg_reg_modules[i].offset & MOD_MASK)));
			} else {
				if (base_mask == PRCM_MPU_BASE)
					base_addr = OMAP4430_PRCM_MPU_BASE;
				else
					base_addr = OMAP4430_PRM_BASE;
				seq_printf(s, "MOD: PRM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(base_addr +
					(pm_dbg_reg_modules[i].offset & MOD_MASK)));
			}
		}

		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			val = *(ptr++);
			if (val != 0) {
				regs++;
				if (linefeed) {
					seq_printf(s, "\n");
					linefeed = 0;
				}
				seq_printf(s, "  %02x => %08lx", j, val);
				if (regs % 4 == 0)
					linefeed = 1;
			}
		}
		seq_printf(s, "\n");
		i++;
	}

	if (store != NULL)
		kfree(store);

	return 0;
}

void pm_dbg_regset_dump(int from, int to)
{
	if (pm_dbg_reg_set[from-1] == NULL)
		return;

	if (pm_dbg_reg_set[to-1] == NULL)
		return;

	memcpy(pm_dbg_reg_set[to-1], pm_dbg_reg_set[from-1],
		pm_dbg_get_regset_size());
}

static void pm_dbg_regset_store(u32 *ptr)
{
	int i, j;
	u32 val;

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				val = cm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			else
				val = prm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			*(ptr++) = val;
		}
		i++;
	}
}

int pm_dbg_regset_save(int reg_set)
{
	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -EINVAL;

	pm_dbg_regset_store(pm_dbg_reg_set[reg_set-1]);

	return 0;
}

static int pm_dbg_show_padconf_regs(struct seq_file *s, void *unused)
{
	unsigned int i, j;
	int counts;
	unsigned short val;
	u32 base;
	int pad_range_size;
	const struct pad_range *pad_range = NULL;

	if (cpu_is_omap34xx()) {
		pad_range = omap3_pad_range;
		pad_range_size = ARRAY_SIZE(omap3_pad_range);
	} else if (cpu_is_omap44xx()) {
		pad_range = omap4_pad_range;
		pad_range_size = ARRAY_SIZE(omap4_pad_range);
	} else {
		seq_printf(s, "Only OMAP3 and OMAP4 supported\n");
		return 0;
	}

	for (i = 0; i < pad_range_size; i++) {
		base = pad_range[i].base;
		seq_printf(s, "padconf base=%08X range=[ %04X --> %04X] ",
				base, pad_range[i].start, pad_range[i].end);
		counts = 0;
		for (j = pad_range[i].start; j <= pad_range[i].end; j += 4) {
			if (counts++ % 4 == 0)
				seq_printf(s, "\n%04X:  ", j);

			/* read the high 16 bits */
			val = omap_readw(base + j + 2);
			seq_printf(s, "	%04x%s", val,
				(val & PADCONF_WAKEUP_ENABLE_MASK) ? "*" : " ");

			/* read the low 16 bits */
			val = omap_readw(base + j);
			seq_printf(s, " %04x%s", val,
				(val & PADCONF_WAKEUP_ENABLE_MASK) ? "*" : " ");
		}
		seq_printf(s, "\n");
	}

	return 0;
}

static const char pwrdm_state_names[][PWRDM_MAX_PWRSTS] = {
	"OFF",
	"RET",
	"INA",
	"ON"
};

void pm_dbg_update_time(struct powerdomain *pwrdm, int prev)
{
	s64 t;

	if (!pm_dbg_init_done)
		return ;

	/* Update timer for previous state */
	t = sched_clock();

	pwrdm->state_timer[prev] += t - pwrdm->timer;

	pwrdm->timer = t;
}

void omap2_pm_wakeup_on_timer(u32 seconds, u32 milliseconds)
{
	u32 tick_rate, cycles;

	if (!seconds && !milliseconds)
		return;

	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer_wakeup));
	cycles = tick_rate * seconds + tick_rate * milliseconds / 1000;
	omap_dm_timer_stop(gptimer_wakeup);
	omap_dm_timer_set_load_start(gptimer_wakeup, 0, 0xffffffff - cycles);

	pr_info("PM: Resume timer in %u.%03u secs"
		" (%d ticks at %d ticks/sec.)\n",
		seconds, milliseconds, cycles, tick_rate);
}

static int clkdm_dbg_show_counter(struct clockdomain *clkdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(clkdm->name, "emu_clkdm") == 0 ||
		strcmp(clkdm->name, "wkup_clkdm") == 0 ||
		strncmp(clkdm->name, "dpll", 4) == 0)
		return 0;

	seq_printf(s, "%s->%s (%d)", clkdm->name,
			clkdm->pwrdm.ptr->name,
			atomic_read(&clkdm->usecount));
	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	if (pwrdm->state != pwrdm_read_pwrst(pwrdm))
		printk(KERN_ERR "pwrdm state mismatch(%s) %d != %d\n",
			pwrdm->name, pwrdm->state, pwrdm_read_pwrst(pwrdm));

	seq_printf(s, "%s (%s)", pwrdm->name,
			pwrdm_state_names[pwrdm->state]);
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		seq_printf(s, ",%s:%d", pwrdm_state_names[i],
			pwrdm->state_counter[i]);

	seq_printf(s, ",RET-LOGIC-OFF:%d", pwrdm->ret_logic_off_counter);
	for (i = 0; i < pwrdm->banks; i++)
		seq_printf(s, ",RET-MEMBANK%d-OFF:%d", i + 1,
				pwrdm->ret_mem_off_counter[i]);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	pwrdm_state_switch(pwrdm);

	seq_printf(s, "%s (%s)", pwrdm->name,
		pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < 4; i++)
		seq_printf(s, ",%s:%lld", pwrdm_state_names[i],
			pwrdm->state_timer[i]);

	seq_printf(s, "\n");
	return 0;
}

static int pm_dbg_show_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_counter, s);
	if (cpu_is_omap44xx())
		seq_printf(s, "DEVICE-OFF:%d\n", omap4_device_off_counter);
	clkdm_for_each(clkdm_dbg_show_counter, s);

	return 0;
}

static int pm_dbg_show_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_timer, s);
	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	switch ((int)inode->i_private) {
	case DEBUG_FILE_COUNTERS:
		return single_open(file, pm_dbg_show_counters,
			&inode->i_private);
	case DEBUG_FILE_TIMERS:
	default:
		return single_open(file, pm_dbg_show_timers,
			&inode->i_private);
	};
}

static int pm_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_regs, inode->i_private);
}

static int pm_dbg_reg_padconf_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_padconf_regs, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations debug_reg_fops = {
	.open           = pm_dbg_reg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations debug_reg_padconf_fops = {
	.open		= pm_dbg_reg_padconf_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int pm_dbg_regset_init(int reg_set)
{
	char name[2];

	if (!pm_dbg_init_done)
		pm_dbg_init();

	if (reg_set < 1 || reg_set > PM_DBG_MAX_REG_SETS ||
		pm_dbg_reg_set[reg_set-1] != NULL)
		return -EINVAL;

	pm_dbg_reg_set[reg_set-1] =
		kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);

	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -ENOMEM;

	if (pm_dbg_dir != NULL) {
		sprintf(name, "%d", reg_set);

		(void) debugfs_create_file(name, S_IRUGO,
			pm_dbg_dir, (void *)reg_set, &debug_reg_fops);
	}

	return 0;
}

static int pwrdm_suspend_get(void *data, u64 *val)
{
	int ret = -EINVAL;

	if (cpu_is_omap34xx())
		ret = omap3_pm_get_suspend_state((struct powerdomain *)data);
	*val = ret;

	if (ret >= 0)
		return 0;
	return *val;
}

static int pwrdm_suspend_set(void *data, u64 val)
{
	if (cpu_is_omap34xx())
		return omap3_pm_set_suspend_state(
			(struct powerdomain *)data, (int)val);
	return -EINVAL;
}

DEFINE_SIMPLE_ATTRIBUTE(pwrdm_suspend_fops, pwrdm_suspend_get,
			pwrdm_suspend_set, "%llu\n");

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *dir)
{
	int i;
	s64 t;
	struct dentry *d;

	t = sched_clock();

	for (i = 0; i < 4; i++)
		pwrdm->state_timer[i] = 0;

	pwrdm->timer = t;

	if (strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	d = debugfs_create_dir(pwrdm->name, (struct dentry *)dir);

	(void) debugfs_create_file("suspend", S_IRUGO|S_IWUSR, d,
			(void *)pwrdm, &pwrdm_suspend_fops);

	return 0;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	if (option == &wakeup_timer_milliseconds && val >= 1000)
		return -EINVAL;

	if (cpu_is_omap44xx() && (omap_type() == OMAP2_DEVICE_TYPE_GP))
		*option = 0;
	else
		*option = val;

	if (option == &enable_off_mode) {
		if (cpu_is_omap34xx())
			omap3_pm_off_mode_enable(val);
		else if (cpu_is_omap44xx())
			omap4_pm_off_mode_enable(val);
	}
	if (option == &enable_sr_vp_debug && val)
		pr_notice("Beware that enabling this option will allow user "
			"to override the system defined vp and sr parameters "
			"All the updated parameters will take effect next "
			"time smartreflex is enabled. Also this option "
			"disables the automatic vp errorgain and sr errormin "
			"limit changes as per the voltage. Users will have "
			"to explicitly write values into the debug fs "
			"entries corresponding to these if they want to see "
			"them changing according to the VDD voltage\n");

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_option_fops, option_get, option_set, "%llu\n");

static int pmd_clks_set(void *data, u64 val)
{
	int *option = data;

	if (option != &pmd_clks_enable)
		return -EINVAL;

	if(cpu_is_omap44xx()) {
		struct clk *l3_instr_ick, *l3_main_3_ick, *ocp_wp_noc_ick;

		l3_instr_ick = clk_get(NULL, "l3_instr_ick");
		l3_main_3_ick = clk_get(NULL, "l3_main_3_ick");
		ocp_wp_noc_ick = clk_get(NULL, "ocp_wp_noc_ick");

		/* enable PMD clocks */
		if (!*option && val) {
			clk_enable(l3_instr_ick);
			clk_enable(l3_main_3_ick);
			clk_enable(ocp_wp_noc_ick);

			*option = val;
		/* disable PMD clocks */
		} else if (*option && !val) {
			clk_disable(l3_instr_ick);
			clk_disable(l3_main_3_ick);
			clk_disable(ocp_wp_noc_ick);

			*option = val;
		}
	}

	return 0;
}

static int pmd_clks_get(void *data, u64 *val)
{
	int *option = data;

	if (option != &pmd_clks_enable)
		return -EINVAL;

	*val = *option;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_debug_pmd_clks_fops, pmd_clks_get, pmd_clks_set,
		"%llu\n");

static int __init omap4_pmd_clks_init(void)
{
	struct clk *l3_instr_ick, *l3_main_3_ick, *ocp_wp_noc_ick;

	l3_instr_ick = clk_get(NULL, "l3_instr_ick");
	l3_main_3_ick = clk_get(NULL, "l3_main_3_ick");
	ocp_wp_noc_ick = clk_get(NULL, "ocp_wp_noc_ick");

#ifdef CONFIG_OMAP4_PMD_CLKS_ENABLE
	pmd_clks_enable = 1;

	if (!(l3_instr_ick->usecount))
		clk_enable(l3_instr_ick);
	if (!(l3_main_3_ick->usecount))
		clk_enable(l3_main_3_ick);
	if (!(ocp_wp_noc_ick->usecount))
		clk_enable(ocp_wp_noc_ick);
#else
	pmd_clks_enable = 0;

	if (l3_instr_ick->usecount)
		clk_disable(l3_instr_ick);
	if (l3_main_3_ick->usecount)
		clk_disable(l3_main_3_ick);
	if (ocp_wp_noc_ick->usecount)
		clk_disable(ocp_wp_noc_ick);
#endif

	return 0;
}

static int dpll_cascading_set(void *data, u64 val)
{
	int *option = data;

	if (option != &dpll_cascade_global_state)
		return -EINVAL;

	if (*option == 0 && val == 1) {
		*option = val;
		omap4_dpll_low_power_cascade_enter();
	} else if (*option == 1 && val == 0) {
		omap4_dpll_low_power_cascade_exit();
		*option = val;
	}

	return 0;
}

static int dpll_cascading_get(void *data, u64 *val)
{
	int *option = data;

	if (option != &dpll_cascade_global_state)
		return -EINVAL;

	*val = *option;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_debug_dpll_cascading_fops, dpll_cascading_get,
		dpll_cascading_set, "%llu\n");

static int __init pm_dbg_init(void)
{
	int i;
	struct dentry *d;
	char name[2];

	if (pm_dbg_init_done)
		return 0;

	if (cpu_is_omap34xx())
		pm_dbg_reg_modules = omap3_pm_reg_modules;
	else if (cpu_is_omap44xx())
		pm_dbg_reg_modules = omap4_pm_reg_modules;
	else
		printk(KERN_ERR "%s: only OMAP3 supported\n", __func__);

	d = debugfs_create_dir("pm_debug", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);

	(void) debugfs_create_file("count", S_IRUGO,
		d, (void *)DEBUG_FILE_COUNTERS, &debug_fops);
	(void) debugfs_create_file("time", S_IRUGO,
		d, (void *)DEBUG_FILE_TIMERS, &debug_fops);

	pwrdm_for_each(pwrdms_setup, (void *)d);

	pm_dbg_dir = debugfs_create_dir("registers", d);
	if (IS_ERR(pm_dbg_dir))
		return PTR_ERR(pm_dbg_dir);

	(void) debugfs_create_file("current", S_IRUGO,
		pm_dbg_dir, (void *)0, &debug_reg_fops);

	for (i = 0; i < PM_DBG_MAX_REG_SETS; i++)
		if (pm_dbg_reg_set[i] != NULL) {
			sprintf(name, "%d", i+1);
			(void) debugfs_create_file(name, S_IRUGO,
				pm_dbg_dir, (void *)(i+1), &debug_reg_fops);

		}

	(void) debugfs_create_file("padconf", S_IRUGO,
		pm_dbg_dir, NULL, &debug_reg_padconf_fops);

	(void) debugfs_create_file("enable_off_mode", S_IRUGO | S_IWUGO, d,
				   &enable_off_mode, &pm_dbg_option_fops);
	(void) debugfs_create_file("sleep_while_idle", S_IRUGO | S_IWUGO, d,
				   &sleep_while_idle, &pm_dbg_option_fops);
	(void) debugfs_create_file("wakeup_timer_seconds", S_IRUGO | S_IWUGO, d,
				   &wakeup_timer_seconds, &pm_dbg_option_fops);
	(void) debugfs_create_file("wakeup_timer_milliseconds",
			S_IRUGO | S_IWUGO, d, &wakeup_timer_milliseconds,
			&pm_dbg_option_fops);
	(void) debugfs_create_file("enable_sr_vp_debug",  S_IRUGO | S_IWUGO, d,
				   &enable_sr_vp_debug, &pm_dbg_option_fops);

	if (cpu_is_omap44xx()) {
		omap4_pmd_clks_init();
		debugfs_create_file("pmd_clks_enable", S_IRUGO|S_IWUGO, d,
				&pmd_clks_enable, &pm_debug_pmd_clks_fops);

		debugfs_create_file("dpll_cascade_enable", S_IRUGO|S_IWUGO, d,
				&dpll_cascade_global_state,
				&pm_debug_dpll_cascading_fops);
	}

	pm_dbg_main_dir = d;
	pm_dbg_init_done = 1;

	return 0;
}
postcore_initcall(pm_dbg_init);

#endif
