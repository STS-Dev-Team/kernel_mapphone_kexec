/*
 * opp-max.c - MAX-specific functions for the OPP code
 *
 * Copyright (C) 2010 Motorola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <plat/opp-max.h>

/**
 * omap_max8952_vsel_to_vdc - convert MAX8952 VSEL value to microvolts DC
 * @vsel: MAX8952 VSEL value to convert
 *
 * Returns the microvolts DC that the MAX8952 Regulator should generate when
 * programmed with @vsel.
 */
unsigned long omap_max8952_vsel_to_uv(unsigned char vsel)
{
	if (vsel > 0x3F)
		vsel = 0x3F;
	return (((vsel * 100) + 7700)) * 100;
}

/**
 * omap_max8952_uv_to_vsel - convert microvolts DC to MAX8952 VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the MAX8952 Regulator to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_max8952_uv_to_vsel(unsigned long uv)
{
	if (uv < 770000)
		uv = 770000;
	else if (uv > 1400000)
		uv = 1400000;
	return DIV_ROUND_UP(uv - 770000, 10000);
}

u8 omap_max8952_onforce_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_max8952_on_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_max8952_sleepforce_cmd(const u8 vsel)
{
	return vsel;
}

u8 omap_max8952_sleep_cmd(const u8 vsel)
{
	return vsel;
}
