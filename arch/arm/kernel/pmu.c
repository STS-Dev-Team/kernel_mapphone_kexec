/*
 *  linux/arch/arm/kernel/pmu.c
 *
 *  Copyright (C) 2009 picoChip Designs Ltd, Jamie Iles
 *  Copyright (C) 2010 ARM Ltd, Will Deacon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "PMU: " fmt

#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/pmu.h>

#ifdef CONFIG_ARCH_OMAP4
#include <linux/io.h>
#include <plat/io.h>
#endif

static volatile long pmu_lock;

static struct platform_device *pmu_devices[ARM_NUM_PMU_DEVICES];

static int __devinit pmu_device_probe(struct platform_device *pdev)
{

	if (pdev->id < 0 || pdev->id >= ARM_NUM_PMU_DEVICES) {
		pr_warning("received registration request for unknown "
				"device %d\n", pdev->id);
		return -EINVAL;
	}

	if (pmu_devices[pdev->id])
		pr_warning("registering new PMU device type %d overwrites "
				"previous registration!\n", pdev->id);
	else
		pr_info("registered new PMU device of type %d\n",
				pdev->id);

	pmu_devices[pdev->id] = pdev;
	return 0;
}

static struct platform_driver pmu_driver = {
	.driver		= {
		.name	= "arm-pmu",
	},
	.probe		= pmu_device_probe,
};

static int __init register_pmu_driver(void)
{
	return platform_driver_register(&pmu_driver);
}
device_initcall(register_pmu_driver);

struct platform_device *
reserve_pmu(enum arm_pmu_type device)
{
	struct platform_device *pdev;

	if (test_and_set_bit_lock(device, &pmu_lock)) {
		pdev = ERR_PTR(-EBUSY);
	} else if (pmu_devices[device] == NULL) {
		clear_bit_unlock(device, &pmu_lock);
		pdev = ERR_PTR(-ENODEV);
	} else {
		pdev = pmu_devices[device];
	}

	return pdev;
}
EXPORT_SYMBOL_GPL(reserve_pmu);

int
release_pmu(struct platform_device *pdev)
{
	if (WARN_ON(pdev != pmu_devices[pdev->id]))
		return -EINVAL;
	clear_bit_unlock(pdev->id, &pmu_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(release_pmu);

static int
set_irq_affinity(int irq,
		 unsigned int cpu)
{
#ifdef CONFIG_SMP
	int err = irq_set_affinity(irq, cpumask_of(cpu));
	if (err)
		pr_warning("unable to set irq affinity (irq=%d, cpu=%u)\n",
			   irq, cpu);
	return err;
#else
	return 0;
#endif
}

#ifdef CONFIG_ARCH_OMAP4
#define OMAP4_CTI0_BASE (L4_EMU_44XX_BASE + 0x148000)
#define OMAP4_CTI1_BASE (L4_EMU_44XX_BASE + 0x149000)
#define OMAP4_CTI_CTRL_OFFSET 0x000
#define OMAP4_CTI_INTACK_OFFSET 0x010
#define OMAP4_CTI_LOCK_OFFSET 0xFB0
#define OMAP4_CTI_TRIGIN1_OFFSET 0x024
#define OMAP4_CTI_TRIGOUT6_OFFSET 0x0B8

#define OMAP4_CM_L3INSTR_L3_3_CLKCTRL 0x4A008E20
#define OMAP4_CM_L3INSTR_L3_INSTR_CLKCTRL 0x4A008E28
#define OMAP4_CM_EMU_CLKSTCTRL 0x4a307a00

static unsigned long CTI[2] = {OMAP4_CTI0_BASE, OMAP4_CTI1_BASE};

void pmu_ack_cti_int()
{
	unsigned int cpu = smp_processor_id();
	__raw_writel(1 << 6,  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
				+ OMAP4_CTI_INTACK_OFFSET));
}

void pmu_l3clk_init()
{
	omap_writel(1, OMAP4_CM_L3INSTR_L3_3_CLKCTRL);
	omap_writel(1, OMAP4_CM_L3INSTR_L3_INSTR_CLKCTRL);
	omap_writel(2, OMAP4_CM_EMU_CLKSTCTRL);
	while ((omap_readl(OMAP4_CM_EMU_CLKSTCTRL) & 0x300) != 0x300)
		;
}

void pmu_cti_init(int cpu)
{
	unsigned int lock_pattern = 0xC5ACCE55;

	/* unlock the module so application writes can go through */
	__raw_writel(lock_pattern, OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_LOCK_OFFSET));
	/* Enable TRIGIN[1] to go to channel 2 */
	__raw_writel(1 << (2 + cpu),  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_TRIGIN1_OFFSET));
	/* Enable channel 2 to go to TRIGOUT[6] */
	__raw_writel(1 << (2 + cpu),  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_TRIGOUT6_OFFSET));
	/* Enable CTI0 module */
	__raw_writel(0x1, OMAP2_EMU_IO_ADDRESS(CTI[cpu]
				+ OMAP4_CTI_CTRL_OFFSET));
}

#endif

static int
init_cpu_pmu(void)
{
	int i, err = 0;
	struct platform_device *pdev = pmu_devices[ARM_PMU_DEVICE_CPU];

	if (!pdev) {
		err = -ENODEV;
		goto out;
	}

	for (i = 0; i < pdev->num_resources; ++i) {
		err = set_irq_affinity(platform_get_irq(pdev, i), i);
		if (err)
			break;
	}

#ifdef CONFIG_ARCH_OMAP4
	if (i == pdev->num_resources)
		pmu_l3clk_init();

	for (i = 0; i < pdev->num_resources; ++i)
		pmu_cti_init(i);
#endif

out:
	return err;
}

int
init_pmu(enum arm_pmu_type device)
{
	int err = 0;

	switch (device) {
	case ARM_PMU_DEVICE_CPU:
		err = init_cpu_pmu();
		break;
	default:
		pr_warning("attempt to initialise unknown device %d\n",
				device);
		err = -EINVAL;
	}

	return err;
}
EXPORT_SYMBOL_GPL(init_pmu);
