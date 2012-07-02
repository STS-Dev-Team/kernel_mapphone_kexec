/*
 * opp-cpcap.c - CPCAP-specific functions for the OPP code
 *
 * Copyright (C) 2010 Motorola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <plat/opp-cpcap.h>

/**
 * omap_cpcap_vsel_to_vdc - convert CPCAP VSEL value to microvolts DC
 * @vsel: CPCAP VSEL value to convert
 *
 * Returns the microvolts DC that the CPCAP PMIC should generate when
 * programmed with @vsel.
 */
unsigned long omap_cpcap_vsel_to_uv(unsigned char vsel)
{
	if (vsel > 0x44)
		vsel = 0x44;
	return (((vsel * 125) + 6000)) * 100;
}

/**
 * omap_cpcap_uv_to_vsel - convert microvolts DC to CPCAP VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the CPCAP PMIC to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_cpcap_uv_to_vsel(unsigned long uv)
{
	if (uv < 600000)
		uv = 600000;
	else if (uv > 1450000)
		uv = 1450000;
	return DIV_ROUND_UP(uv - 600000, 12500);
}

u8 omap_cpcap_onforce_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_cpcap_on_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_cpcap_sleepforce_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_cpcap_sleep_cmd(const u8 vsel)
{
	return vsel;
}
