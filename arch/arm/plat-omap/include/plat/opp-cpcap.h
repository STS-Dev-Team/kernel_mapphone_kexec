/*
 * opp_cpcap.h - CPCAP-specific headers for the OPP code
 *
 * Copyright (C) 2010 Motorola.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ARCH_ARM_PLAT_OMAP_OPP_CPCAP_H
#define _ARCH_ARM_PLAT_OMAP_OPP_CPCAP_H

#include <linux/kernel.h>

unsigned long omap_cpcap_vsel_to_uv(const u8 vsel);
u8 omap_cpcap_uv_to_vsel(unsigned long uV);
u8 omap_cpcap_onforce_cmd(const u8 vsel);
u8 omap_cpcap_on_cmd(const u8 vsel);
u8 omap_cpcap_sleepforce_cmd(const u8 vsel);
u8 omap_cpcap_sleep_cmd(const u8 vsel);

#endif
