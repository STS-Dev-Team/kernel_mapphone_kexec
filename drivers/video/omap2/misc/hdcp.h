/*
 * hdcp.h
 *
 * HDCP driver definitions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Fabrice Olivero <f-olivero@ti.com>
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#ifndef _HDCP_H_
#define _HDCP_H_

/********************************/
/* Structures related to ioctl  */
/********************************/

/* HDCP key size in 32-bit words */
#define DESHDCP_KEY_SIZE 160

/* HDCP ioctl */
#include <linux/ioctl.h>

struct hdcp_encrypt_control {
	uint32_t in_key[DESHDCP_KEY_SIZE];
	uint32_t *out_key;
};

struct hdcp_enable_control {
	uint32_t key[DESHDCP_KEY_SIZE];
	int nb_retry;
};

#define HDCP_IOCTL_MAGIC 'h'
#define HDCP_ENABLE	  _IOW( HDCP_IOCTL_MAGIC, 0, \
				struct hdcp_enable_control)
#define HDCP_DISABLE	  _IO(  HDCP_IOCTL_MAGIC, 1)
#define HDCP_ENCRYPT_KEY  _IOWR(HDCP_IOCTL_MAGIC, 2, \
				struct hdcp_encrypt_control)
#define HDCP_QUERY_STATUS _IOWR(HDCP_IOCTL_MAGIC, 3, uint32_t)

#define HDCP_STATE_DISABLED		0
#define HDCP_STATE_INIT     		1
#define HDCP_STATE_AUTH_1ST_STEP	2
#define HDCP_STATE_AUTH_2ND_STEP	3
#define HDCP_STATE_AUTH_3RD_STEP	4
#define HDCP_STATE_AUTH_FAIL_RESTARTING 5
#define HDCP_STATE_AUTH_FAILURE         6

#ifdef __KERNEL__

/***************************/
/* HW specific definitions */
/***************************/

/* DESHDCP base address */
/*----------------------*/

#define DSS_SS_FROM_L3__DESHDCP 0x58007000

/* DESHDCP registers */
#define DESHDCP__DHDCP_CTRL   0x020
#define DESHDCP__DHDCP_DATA_L 0x024
#define DESHDCP__DHDCP_DATA_H 0x028

/* DESHDCP CTRL bits */
#define DESHDCP__DHDCP_CTRL__DIRECTION_POS_F 2
#define DESHDCP__DHDCP_CTRL__DIRECTION_POS_L 2

#define DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F 0
#define DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L 0

/* HDMI WP base address */
/*----------------------*/
#define HDMI_WP			0x58006000

/* HDMI CORE SYSTEM base address */
/*-------------------------------*/

#define HDMI_IP_CORE_SYSTEM 0x400

/* HDMI CORE registers */
#define HDMI_IP_CORE_SYSTEM__HDCP_CTRL	0x03C
#define HDMI_IP_CORE_SYSTEM__AKSV0	0x074
#define HDMI_IP_CORE_SYSTEM__AKSV1	0x078
#define HDMI_IP_CORE_SYSTEM__AKSV2	0x07C
#define HDMI_IP_CORE_SYSTEM__AKSV3	0x080
#define HDMI_IP_CORE_SYSTEM__AKSV4	0x084

/* HDMI CORE AV base address */
/*---------------------------*/

#define HDMI_CORE_AV_BASE	0x900
#define HDMI_CORE_AV_HDMI_CTRL  0xBC


/* HDMI CORE GAMUT base address */
/*------------------------------*/

#define HDMI_IP_CORE_GAMUT 0x800

/* HSA control register */
#define HDMI_IP_CORE_GAMUT__HSA_CTRL1	0x080
#define HDMI_IP_CORE_GAMUT__HSA_STATUS	0x088

/* HSA control bits */
#define HSA_CTRL__SINK_TYPE_HDMI_SET	0x01
#define HSA_CTRL__HSA_SW_SET		0x02
#define HSA_CTRL__REAUTH_CTL_SET	0x04
#define HSA_CTRL__COPP_PROTLEVEL_SET	0x08

#define HSA_CTRL__R0_CALC_TIME_POS_F	7
#define HSA_CTRL__R0_CALC_TIME_POS_L	4

/* HSA status bits */
#define HSA_STATUS__HW_CS_POS_F	7
#define HSA_STATUS__HW_CS_POS_L	4

enum hsa_state {
	HSA_STATE_IDLE = 0,
	HSA_STATE_INIT,
	HSA_STATE_POWER_DOWN,
	HSA_STATE_CONFIG,
	HSA_STATE_PREPARATION,
	HSA_STATE_1ST_STEP_AUTH,
	HSA_STATE_2ND_STEP_AUTH,
	HSA_STATE_LINK_INTEGRITY_CHECK,
	HSA_STATE_AUTH_FAILURE,
	HSA_NB_STATES
};

/***************************/
/* Definitions             */
/***************************/
#define HDCP_OK    0
#define HDCP_ERROR 1

/* FIXME: should be 300ms delay between HDMI start frame event and HDCP enable
 * (to respect 7 VSYNC delay in 24 Hz)
 */
#define HDCP_ENABLE_DELAY 1500
/* Delay after HDCP enabling before checking authentication status */
#define HDCP_AUTH_DELAY 200

/* Workqueue events */
#define HDCP_ENABLE_REQ		0
#define HDCP_DISABLE_REQ	1
#define HDCP_START_FRAME_EVENT  2
#define HDCP_STOP_FRAME_EVENT   3
#define HDCP_CHECK_AUTH_STATUS  4

/* Power management functions */
#ifdef DSS_INACTIVITY
#define hdcp_request_dss() request_dss()
#define hdcp_release_dss() release_dss()
#else
#define hdcp_request_dss()
#define hdcp_release_dss()
#endif


/***************************/
/* Macros for accessing HW */
/***************************/

#define WR_REG_32(base, offset, val)	__raw_writel(val, base + offset)
#define RD_REG_32(base, offset)		__raw_readl(base + offset)


#define FLD_MASK(start, end)	(((1 << (start - end + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << end) & FLD_MASK(start, end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define WR_FIELD_32(base, offset, start, end, val) \
	WR_REG_32(base, offset, FLD_MOD(RD_REG_32(base, offset), val, \
		  start, end))

#define RD_FIELD_32(base, offset, start, end) \
	((RD_REG_32(base, offset) & FLD_MASK(start, end)) >> (end))


/***************************/
/* Function prototypes     */
/***************************/

static void hdcp_3des_enc_key(struct hdcp_encrypt_control *enc_ctrl,
			      uint32_t out_key[DESHDCP_KEY_SIZE]);
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg);
static int __init hdcp_init(void);
static void __exit hdcp_exit(void);

#endif /* __KERNEL__ */

#endif /* _HDCP_H_ */
