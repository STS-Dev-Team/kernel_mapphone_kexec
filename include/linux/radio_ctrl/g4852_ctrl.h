/*
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307  USA
 */
#ifndef __LINUX_G4852_CTRL_H__
#define __LINUX_G4852_CTRL_H__

#ifdef __KERNEL__
#define G4852_CTRL_MODULE_NAME "g4852_ctrl"

struct g4852_ctrl_platform_data {
	char *name;
	unsigned int gsm_pwron;
	unsigned int ap_reset_gsm;
	unsigned int gsm_flash_en;
	unsigned int gsm_panic;
	unsigned int gsm_status1;
	unsigned int bplog_to_ap_en;
	unsigned int bplog_to_jtag_en;
};

#define RADIO_STATUS_PANIC_NAME		"panic"

extern void set_ste_tat_mode(void);
extern void set_ste_bplog_mode(int on);
extern int modem_is_ste_g4852(void);
extern int is_dmds_phone(void);
#endif
#endif /* __LINUX_G4852_CTRL_H__ */
