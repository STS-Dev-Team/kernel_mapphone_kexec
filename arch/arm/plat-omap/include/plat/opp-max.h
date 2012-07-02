/*
 * opp_cpcap.h - MAX-specific headers for the OPP code
 *
 * Copyright (C) 2010 Motorola.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ARCH_ARM_PLAT_OMAP_OPP_MAX_H
#define _ARCH_ARM_PLAT_OMAP_OPP_MAX_H

#include <linux/kernel.h>

unsigned long omap_max8952_vsel_to_uv(const u8 vsel);
u8 omap_max8952_uv_to_vsel(unsigned long uV);

u8 omap_max8952_onforce_cmd(const u8 vsel);
u8 omap_max8952_on_cmd(const u8 vsel);
u8 omap_max8952_sleepforce_cmd(const u8 vsel);
u8 omap_max8952_sleep_cmd(const u8 vsel);

#endif
