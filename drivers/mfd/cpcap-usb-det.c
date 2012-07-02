/*
 * Copyright (C) 2007 - 2011 Motorola, Inc.
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
 * 02111-1307, USA
 *
 * Brief overview about the switches used
 * ======================================
 *
 * wsdev (Whisper switch)
 *	DOCKED      : PPD(UART) dock is detected.
 *	UNDOCKED    : SMART dock is detected.
 *                    No dock is connected.
 *
 * dsdev (Standard Dock switch)
 *      NO_DOCK	    : No dock is connected.
 *		      LAP/MOBILE(SMART) dock with LID open detected.
 *	DESK_DOCK   : DESK (PPD) dock is detected.
 *		      HD(SMART) dock is detected.
 *	CAR_DOCK    : CAR (PPD) dock is detected.
 *
 * edsdev (Motorola Dock switch)
 *      NO_DOCK	    : No dock is connected.
 *	DESK_DOCK   : DESK (PPD) dock is detected.
 *	CAR_DOCK    : CAR (PPD) dock is detected.
 *	MOBILE_DOCK : LAP/MOBILE(SMART) dock is detected.
 *	HD_DOCK     : HD(SMART) dock is detected.
 *
 * emusdev (Audio switch)
 *      NO_DEVICE   : Audio cable not present.
 *	EMU_OUT     : Audio cable detected on a PPD dock.
 *	SPDIF_AUDIO_OUT : Audio cable detected on a SMART dock.
 *
 * sdsdev (Smart Dock switch) - Used only by Dock Status App
 *	DOCKED      : SMART dock is detected.
 *	UNDOCKED    : No dock is connected.
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#define CPCAP_SENSE4_LS		8
#define CPCAP_BIT_DP_S_LS	(CPCAP_BIT_DP_S << CPCAP_SENSE4_LS)
#define CPCAP_BIT_DM_S_LS	(CPCAP_BIT_DM_S << CPCAP_SENSE4_LS)
#define GPIO_BIT_SWST1          0x0020

#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_USB_FLASH     (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY_MASK   (~(CPCAP_BIT_DP_S_LS | \
			      CPCAP_BIT_CHRGCURR1_S))

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER_IND   (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_DP_S_LS     | \
			     GPIO_BIT_SWST1)

#define SENSE_CHARGER_MASK  (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_WHISPER_PPD   (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_PPD_MASK   (CPCAP_BIT_ID_FLOAT_S | \
				  CPCAP_BIT_SESSVLD_S)

#define SENSE_WHISPER_CABLE (CPCAP_BIT_CHRGCURR1_S)

#define SENSE_WHISPER_SMART (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define UNDETECT_TRIES		5
#define ADC_AUDIO_THRES     0x12C

#define CPCAP_USB_DET_PRINT_STATUS (1U << 0)
#define CPCAP_USB_DET_PRINT_TRANSITION (1U << 1)
static int cpcap_usb_det_debug_mask;

module_param_named(cpcap_usb_det_debug_mask, cpcap_usb_det_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

#define pr_cpcap_usb_det(debug_level_mask, args...) \
	do { \
		if (cpcap_usb_det_debug_mask & \
		    CPCAP_USB_DET_PRINT_##debug_level_mask) { \
			pr_info(args); \
		} \
	} while (0)

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	IDENTIFY_WHISPER,
	USB,
	FACTORY,
	CHARGER,
	WHISPER_PPD,
	WHISPER_SMART,
	CHARGER_INDUCTIVE
};

enum {
	NO_DOCK,
	DESK_DOCK,
	CAR_DOCK,
	MOBILE_DOCK,
	HD_DOCK,
};

enum {
	NO_DEVICE,
	EMU_OUT = 0x02,
	SPDIF_AUDIO_OUT = 0x4,
};

enum {
	AUTH_NOT_STARTED,
	AUTH_IN_PROGRESS,
	AUTH_FAILED,
	AUTH_PASSED,
};

enum {
	EXT_NO_DOCK,
	EXT_DESK_DOCK,
	EXT_CAR_DOCK,
	EXT_MOBILE_DOCK,
	EXT_HD_DOCK,
};

enum {
	UNDOCKED,
	DOCKED,
};

static const char *accy_names[8] = {
	"USB",
	"FACTORY",
	"CHARGER",
	"WHISPER PPD",
	"WHISPER SMART",
	"USB DEVICE",
	"NONE",
	"UNKNOWN",
};

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	struct workqueue_struct *wq;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
	struct platform_device *usb_dev;
	struct platform_device *usb_connected_dev;
	struct platform_device *charger_connected_dev;
	struct regulator *regulator;
	struct wake_lock wake_lock;
	unsigned char is_vusb_enabled;
	unsigned char undetect_cnt;
	char bpass_mod;
	struct cpcap_batt_ac_data ac;
	struct switch_dev wsdev; /* Whisper switch */
	struct switch_dev dsdev; /* Standard Dock switch */
	struct switch_dev emusdev; /* Audio switch */
	struct switch_dev edsdev; /* Motorola Dock switch */
	struct switch_dev sdsdev; /* Smart Dock Switch */
	unsigned char audio;
	short whisper_auth;
	enum cpcap_irqs irq;
	enum cpcap_support_status usb_drv_ctrl_via_ulpi;
	struct cpcap_usb_mux *usb_mux;
};

static const char *accy_devices[] = {
	"cpcap_usb_charger",
	"cpcap_factory",
	"cpcap_charger",
	"cpcap_whisper_ppd",
	"cpcap_whisper_smart",
};

#ifdef CONFIG_USB_TESTING_POWER
static int testing_power_enable = -1;
module_param(testing_power_enable, int, 0644);
MODULE_PARM_DESC(testing_power_enable, "Enable factory cable power "
	"supply function for testing");
#endif

static void vusb_enable(struct cpcap_usb_det_data *data)
{
	if (!data->is_vusb_enabled) {
		wake_lock(&data->wake_lock);
		regulator_enable(data->regulator);
		data->is_vusb_enabled = 1;
	}
}

static void vusb_disable(struct cpcap_usb_det_data *data)
{
	if (data->is_vusb_enabled) {
		wake_unlock(&data->wake_lock);
		regulator_disable(data->regulator);
		data->is_vusb_enabled = 0;
	}
}

static ssize_t dock_print_name(struct switch_dev *switch_dev, char *buf)
{
	switch (switch_get_state(switch_dev)) {
	case NO_DOCK:
		return sprintf(buf, "None\n");
	case DESK_DOCK:
		return sprintf(buf, "DESK\n");
	case CAR_DOCK:
		return sprintf(buf, "CAR\n");
	case MOBILE_DOCK:
		return sprintf(buf, "MOBILE\n");
	case HD_DOCK:
		return sprintf(buf, "HD\n");
	}

	return -EINVAL;
}

static ssize_t emu_audio_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case EMU_OUT:
		return sprintf(buf, "Stereo out\n");
	case SPDIF_AUDIO_OUT:
		return sprintf(buf, "SPDIF audio out\n");
	}

	return -EINVAL;
}

void whisper_toggle_audio_switch_for_spdif(struct cpcap_device *cpcap,
					   int state)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	struct cpcap_platform_data *pdata;
	if (!data)
		return;

	if (!switch_get_state(&data->dsdev))
		return;

	pdata = cpcap->spi->controller_data;

	pdata->spdif_audio->enable_spdif_audio(state);
	state = ((state > 0) ? SPDIF_AUDIO_OUT : NO_DEVICE);
	switch_set_state(&data->emusdev, state);

	pr_info("%s: Audio cable %s present\n", __func__,
		(state ? "is" : "not"));
}

static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;

	if (!data)
		return -EFAULT;
	cpcap = data->cpcap;
	pdata = cpcap->spi->controller_data;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I));
	if (retval)
		return retval;

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT4,
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I),
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I));
	if (retval)
		return retval;

	data->sense |= (value & (CPCAP_BIT_DP_S |
			       CPCAP_BIT_DM_S)) << CPCAP_SENSE4_LS;

	if (pdata->ind_chrg->check_inductive_path != NULL)
		if (pdata->ind_chrg->check_inductive_path() == 0)
			data->sense |= GPIO_BIT_SWST1;

	pr_cpcap_usb_det(STATUS, "CPCAP USB DET Sense = 0x%4X", data->sense);

	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data,
			      enum cpcap_accy accy)
{
	int retval = -EFAULT;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;

	cpcap = data->cpcap;
	pdata = cpcap->spi->controller_data;

	if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
		/* Take control of pull up from ULPI. */
		retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

	switch (accy) {
	case CPCAP_ACCY_USB:
	case CPCAP_ACCY_FACTORY:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);

		/* Give USB driver control of pull up via ULPI. */
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     0,
						     CPCAP_BIT_PU_SPI |
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);

		if ((data->cpcap->vendor == CPCAP_VENDOR_ST) &&
			(data->cpcap->revision == CPCAP_REVISION_2_0))
				vusb_enable(data);

		break;

	case CPCAP_ACCY_CHARGER:
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_WHISPER_PPD:
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(1);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     CPCAP_BIT_RVRSMODE,
					     CPCAP_BIT_RVRSMODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_WHISPER_SMART:
	case CPCAP_ACCY_USB_DEVICE:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		/* Give USB driver control of pull up via ULPI. */
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED) {
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     0,
						     CPCAP_BIT_PU_SPI |
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC2,
						     CPCAP_BIT_USBXCVREN,
						     CPCAP_BIT_USBXCVREN);
		}
		break;

	case CPCAP_ACCY_UNKNOWN:
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD |
					     CPCAP_BIT_ID100KPU);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_EMUMODE2 |
					     CPCAP_BIT_EMUMODE1 |
					     CPCAP_BIT_EMUMODE0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		data->whisper_auth = AUTH_NOT_STARTED;
		break;

	case CPCAP_ACCY_NONE:
	default:
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		if (pdata->ind_chrg->force_inductive_path != NULL)
			pdata->ind_chrg->force_inductive_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBSUSPEND,
					     CPCAP_BIT_USBXCVREN |
					     CPCAP_BIT_USBSUSPEND);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL,
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);
		data->whisper_auth = AUTH_NOT_STARTED;
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

static unsigned char vbus_valid_adc_check(struct cpcap_usb_det_data *data)
{
	struct cpcap_adc_request req;
	int ret;

	req.format = CPCAP_ADC_FORMAT_CONVERTED;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);
	if (ret) {
		dev_err(&data->cpcap->spi->dev,
		 "%s: ADC Read failed\n", __func__);
		return false;
	}
	return ((req.result[CPCAP_ADC_CHG_ISENSE] < 50) &&
		(req.result[CPCAP_ADC_VBUS] <
		(req.result[CPCAP_ADC_BATTP]+100))) ? false : true;
}

static void whisper_audio_check(struct cpcap_usb_det_data *data)
{
	struct cpcap_adc_request req;
	int ret;
	unsigned short value;

	if (!switch_get_state(&data->dsdev))
		return;

	cpcap_regacc_read(data->cpcap, CPCAP_REG_USBC1, &value);
	value &= CPCAP_BIT_ID100KPU;

	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, CPCAP_BIT_IDPUCNTRL,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	msleep(200);

	req.format = CPCAP_ADC_FORMAT_RAW;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);
	if (ret) {
		dev_err(&data->cpcap->spi->dev,
			 "%s: ADC Read failed\n", __func__);
	}

	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, value,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	if (!ret) {
		data->audio = (req.result[CPCAP_ADC_USB_ID] >
			       ADC_AUDIO_THRES) ? EMU_OUT : NO_DEVICE;
		switch_set_state(&data->emusdev, data->audio);

		dev_info(&data->cpcap->spi->dev,
			 "%s: Audio cable %s present\n", __func__,
			 (data->audio ? "is" : "not"));
	}
}

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	dev_info(&data->cpcap->spi->dev, "notify_accy: accy=%s\n",
		accy_names[accy]);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL)) {
		if (!((data->usb_accy == CPCAP_ACCY_USB_DEVICE) &&
		    (accy == CPCAP_ACCY_CHARGER))) {
			platform_device_del(data->usb_dev);
			data->usb_dev = NULL;
		}
	}

	configure_hardware(data, accy);
	data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		if (!data->usb_dev) {
			if (accy == CPCAP_ACCY_USB_DEVICE)
				data->usb_dev = platform_device_alloc(
					accy_devices[CPCAP_ACCY_CHARGER], -1);
			else
				data->usb_dev = platform_device_alloc(
					accy_devices[accy], -1);
			if (data->usb_dev) {
				data->usb_dev->dev.platform_data = data->cpcap;
				platform_set_drvdata(data->usb_dev, &data->ac);
				platform_device_add(data->usb_dev);
			}
		}
	} else
		vusb_disable(data);

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY) ||
	    (accy == CPCAP_ACCY_USB_DEVICE)) {
		if (!data->usb_connected_dev) {
			data->usb_connected_dev =
			    platform_device_alloc("cpcap_usb_connected", -1);
			if (data->usb_connected_dev) {
				platform_device_add_data(
					data->usb_connected_dev,
					&data->usb_accy,
					sizeof(data->usb_accy));
				platform_device_add(data->usb_connected_dev);
			}
		}
	} else if (data->usb_connected_dev) {
		platform_device_del(data->usb_connected_dev);
		data->usb_connected_dev = NULL;
	}

	if (accy == CPCAP_ACCY_CHARGER) {
		if (!data->charger_connected_dev) {
			data->charger_connected_dev =
			    platform_device_alloc("cpcap_charger_connected",
						  -1);
			if (data->charger_connected_dev)
				platform_device_add(
					data->charger_connected_dev);
		}
	} else if (data->charger_connected_dev) {
		platform_device_del(data->charger_connected_dev);
		data->charger_connected_dev = NULL;
	}
}

static void notify_whisper_switch(struct cpcap_usb_det_data *data,
				enum cpcap_accy accy)
{
	if ((accy == CPCAP_ACCY_CHARGER) ||
		(accy == CPCAP_ACCY_WHISPER_PPD)) {
		switch_set_state(&data->wsdev, DOCKED);
	} else {
		switch_set_state(&data->wsdev, UNDOCKED);
		switch_set_state(&data->dsdev, NO_DOCK);
		switch_set_state(&data->edsdev, NO_DOCK);
		switch_set_state(&data->emusdev, NO_DEVICE);
		if (accy == CPCAP_ACCY_WHISPER_SMART)
			switch_set_state(&data->sdsdev, DOCKED);
		else
			switch_set_state(&data->sdsdev, UNDOCKED);
	}
}

static void detection_work(struct work_struct *work)
{
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);
	unsigned char isVBusValid = 0;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;

	cpcap = data->cpcap;
	pdata = cpcap->spi->controller_data;

	switch (data->state) {
	case CONFIG:
		vusb_enable(data);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);


		configure_hardware(data, CPCAP_ACCY_UNKNOWN);

		data->undetect_cnt = 0;
		data->state = SAMPLE_1;
		queue_delayed_work(data->wq, &data->work,
					msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		get_sense(data);
		data->state = SAMPLE_2;
		queue_delayed_work(data->wq, &data->work,
					msecs_to_jiffies(100));
		break;

	case SAMPLE_2:
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			queue_delayed_work(data->wq, &data->work,
					      msecs_to_jiffies(100));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work,
					      msecs_to_jiffies(100));
		} else {
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

		if ((data->sense == SENSE_USB) ||
		    (data->sense == SENSE_USB_FLASH)) {
			pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(USB_OTG);
			notify_accy(data, CPCAP_ACCY_USB);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		} else if ((data->sense & SENSE_FACTORY_MASK) ==
			   SENSE_FACTORY) {
			pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(USB_OTG);
#ifdef CONFIG_USB_TESTING_POWER
			if (testing_power_enable > 0) {
				notify_accy(data, CPCAP_ACCY_NONE);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_DET);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_CURR1);
				cpcap_irq_unmask(data->cpcap,
				CPCAP_IRQ_VBUSVLD);
				break;
			}
#endif
			notify_accy(data, CPCAP_ACCY_FACTORY);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);

			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if ((data->sense == SENSE_CHARGER_FLOAT) ||
			   (data->sense == SENSE_CHARGER) ||
			   (data->sense == SENSE_WHISPER_PPD) ||
			   (data->sense == SENSE_WHISPER_CABLE) ||
			   (data->sense == SENSE_WHISPER_SMART)) {
				data->state = IDENTIFY_WHISPER;
				queue_delayed_work(data->wq, &data->work, 0);
		} else if (data->sense == SENSE_CHARGER_IND) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			data->ac.model = CPCAP_BATT_AC_IND;
			notify_accy(data, CPCAP_ACCY_CHARGER);
			data->state = CHARGER_INDUCTIVE;
			if (pdata->ind_chrg->force_inductive_path != NULL)
				pdata->ind_chrg->force_inductive_path(1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
		} else if ((vbus_valid_adc_check(data)) &&
				(data->usb_accy == CPCAP_ACCY_NONE)) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			data->state = CONFIG;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

		} else {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			data->ac.model = CPCAP_BATT_AC_NONE;
			notify_accy(data, CPCAP_ACCY_NONE);
			notify_whisper_switch(data, CPCAP_ACCY_NONE);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);

			/* When a charger is unpowered by unplugging from the
			 * wall, VBUS voltage will drop below CHRG_DET (3.5V)
			 * until the ICHRG bits are cleared.  Once ICHRG is
			 * cleared, VBUS will rise above CHRG_DET, but below
			 * VBUSVLD (4.4V) briefly as it decays.  If the charger
			 * is re-powered while VBUS is within this window, the
			 * VBUSVLD interrupt is needed to trigger charger
			 * detection.
			 *
			 * VBUSVLD must be masked before going into suspend.
			 * See cpcap_usb_det_suspend() for details.
			 */
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		}
		break;

	case USB:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);


		if ((data->sense & CPCAP_BIT_SE1_S) ||
			(data->sense & CPCAP_BIT_ID_GROUND_S) ||
			(!isVBusValid)) {
				data->state = CONFIG;
				queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = USB;
			data->undetect_cnt = 0;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case FACTORY:
		get_sense(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
#ifdef CONFIG_TTA_CHARGER
			enable_tta();
#endif
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
		}
		break;

	case IDENTIFY_WHISPER:
		data->state = CONFIG;

		if ((data->sense == SENSE_CHARGER_FLOAT) ||
		    (data->sense == SENSE_CHARGER) ||
		    ((data->sense & SENSE_CHARGER_MASK) ==
		     CPCAP_BIT_SESSVLD_S)) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			data->ac.model = CPCAP_BATT_AC_CABLE;
			notify_accy(data, CPCAP_ACCY_CHARGER);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			data->state = CHARGER;
			notify_whisper_switch(data, CPCAP_ACCY_CHARGER);
		} else if ((data->sense == SENSE_WHISPER_PPD) ||
			   !(data->sense & SENSE_WHISPER_PPD_MASK) ||
			   (data->sense == SENSE_WHISPER_CABLE)) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			notify_accy(data, CPCAP_ACCY_WHISPER_PPD);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			data->state = WHISPER_PPD;
			notify_whisper_switch(data, CPCAP_ACCY_WHISPER_PPD);
		} else if (data->sense == SENSE_WHISPER_SMART) {
			pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(USB_OTG);
			notify_accy(data, CPCAP_ACCY_USB_DEVICE);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			data->state = WHISPER_SMART;
			notify_whisper_switch(data, CPCAP_ACCY_WHISPER_SMART);
		}
		break;

	case CHARGER:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (!(data->sense & CPCAP_BIT_SESSVLD_S) &&
		    !(data->sense & CPCAP_BIT_ID_FLOAT_S) &&
		    (data->whisper_auth == AUTH_PASSED)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			data->state = IDENTIFY_WHISPER;
			queue_delayed_work(data->wq, &data->work, 0);
		} else if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			    data->irq == CPCAP_IRQ_IDGND &&
			    data->whisper_auth == AUTH_PASSED) {
				whisper_audio_check(data);
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case WHISPER_PPD:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (data->sense & CPCAP_BIT_ID_FLOAT_S) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   (isVBusValid) &&
			   (data->sense & CPCAP_BIT_SESSVLD_S)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			data->state = IDENTIFY_WHISPER;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if (data->whisper_auth == AUTH_FAILED &&
			    data->usb_accy == CPCAP_ACCY_WHISPER_PPD) {
				notify_accy(data, CPCAP_ACCY_NONE);
			}

			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			    data->irq == CPCAP_IRQ_IDGND &&
			    data->whisper_auth == AUTH_PASSED) {
				whisper_audio_check(data);
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case WHISPER_SMART:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (data->whisper_auth == AUTH_FAILED &&
		    data->usb_accy == CPCAP_ACCY_USB_DEVICE &&
		    isVBusValid) {
			notify_accy(data, CPCAP_ACCY_CHARGER);
		}

		if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = WHISPER_SMART;
			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			    isVBusValid)
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case CHARGER_INDUCTIVE:
		get_sense(data);
		if ((data->sense & CPCAP_BIT_ID_GROUND_S) ||
		    !(data->sense & CPCAP_BIT_ID_FLOAT_S) ||
		    !(data->sense & CPCAP_BIT_DP_S_LS) ||
		    (data->sense & CPCAP_BIT_SE1_S)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
			if (pdata->ind_chrg->force_charge_complete != NULL)
				pdata->ind_chrg->force_charge_complete(1);
			data->state = CONFIG;
		} else if (!(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			data->state = CHARGER_INDUCTIVE;
		}

		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		vusb_disable(data);
		data->state = CONFIG;
		queue_delayed_work(data->wq, &data->work, 0);
		break;
	}
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	usb_det_data->irq = int_event;
	queue_delayed_work(usb_det_data->wq, &(usb_det_data->work), 0);
}

int cpcap_accy_whisper(struct cpcap_device *cpcap,
		       unsigned long cmd)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	struct cpcap_platform_data *pdata;

	int retval = -EAGAIN;
	int dock = NO_DOCK;
	unsigned short value = 0;
	data->whisper_auth = AUTH_FAILED;

	if (!data)
		return -ENODEV;

	pdata = cpcap->spi->controller_data;

	if ((data->state == CHARGER) ||
	    (data->state == WHISPER_PPD)) {
		if (cmd & CPCAP_WHISPER_ENABLE_UART) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			data->whisper_auth = AUTH_IN_PROGRESS;
			pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(UART_2);
		}

		retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
			value, CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
			CPCAP_BIT_EMUMODE0);

		value = (cmd & CPCAP_WHISPER_MODE_PU) ?
			CPCAP_BIT_ID100KPU : 0;

		retval |= cpcap_regacc_write(cpcap, CPCAP_REG_USBC1,
					     value, CPCAP_BIT_ID100KPU);

		if (value) {
			retval |= cpcap_regacc_write(cpcap, CPCAP_REG_USBC2,
				(CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE0),
				(CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
				CPCAP_BIT_EMUMODE0));
			data->whisper_auth = AUTH_PASSED;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		}

		/* Report dock type to system. */
		dock = (cmd & CPCAP_WHISPER_ACCY_MASK) >>
			CPCAP_WHISPER_ACCY_SHFT;
		switch_set_state(&data->dsdev, dock);
		switch_set_state(&data->edsdev, dock);
		if (!(cmd & CPCAP_WHISPER_ENABLE_UART) ==
			STATUS_SUPPORTED) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			whisper_audio_check(data);
		}
	} else if (data->state == WHISPER_SMART) {
		/* Report dock type to system. */
		dock = (cmd & CPCAP_WHISPER_ACCY_MASK) >>
			CPCAP_WHISPER_ACCY_SHFT;
		switch (dock) {
		case HD_DOCK:
			switch_set_state(&data->dsdev, DESK_DOCK);
			break;
		case MOBILE_DOCK:
		default:
			switch_set_state(&data->dsdev, NO_DOCK);
			break;
		}
		switch_set_state(&data->edsdev, dock);
		retval = 0;
	}

	return retval;
}

static int __init cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;
	struct cpcap_platform_data *pdata;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	data->wq = create_singlethread_workqueue("cpcap_accy");
	platform_set_drvdata(pdev, data);
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "usb");
	data->undetect_cnt = 0;
	data->bpass_mod = 'a';
	data->ac.model = CPCAP_BATT_AC_NONE;

	data->wsdev.name = "whisper";
	switch_dev_register(&data->wsdev);

	data->dsdev.name = "dock";
	data->dsdev.print_name = dock_print_name;
	switch_dev_register(&data->dsdev);

	data->emusdev.name = "emuconn";
	data->emusdev.print_name = emu_audio_print_name;
	switch_dev_register(&data->emusdev);

	data->edsdev.name = "extdock";
	data->edsdev.print_name = dock_print_name;
	switch_dev_register(&data->edsdev);

	data->sdsdev.name = "smartdock";
	switch_dev_register(&data->sdsdev);

	data->whisper_auth = AUTH_NOT_STARTED;
	data->irq = CPCAP_IRQ__START;

	pdata = data->cpcap->spi->controller_data;
	data->usb_drv_ctrl_via_ulpi = pdata->usb_drv_ctrl_via_ulpi;

	data->regulator = regulator_get(NULL, "vusb");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_usb\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}
	regulator_set_voltage(data->regulator, 3300000, 3300000);


	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				    int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_VBUSVLD,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDFLOAT,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DPI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DMI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SESSVLD,
				     int_handler, data);

	if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
		/* Now that HW initialization is done,
		   give USB control via ULPI. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0, CPCAP_BIT_ULPI_SPI_SEL);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_irqs;
	}

	data->cpcap->accydata = data;
	dev_set_drvdata(&pdev->dev, data);

	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Perform initial detection */
	detection_work(&(data->work.work));

	return 0;

free_irqs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);
	regulator_put(data->regulator);

free_mem:
	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->emusdev);
	switch_dev_unregister(&data->edsdev);
	switch_dev_unregister(&data->sdsdev);

	wake_lock_destroy(&data->wake_lock);
	kfree(data);

	return retval;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);


	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);
	destroy_workqueue(data->wq);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL))
		platform_device_del(data->usb_dev);

	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->emusdev);
	switch_dev_unregister(&data->edsdev);
	switch_dev_unregister(&data->sdsdev);

	vusb_disable(data);
	regulator_put(data->regulator);

	wake_lock_destroy(&data->wake_lock);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int cpcap_usb_det_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	/* VBUSVLD cannot be unmasked when entering suspend. If left
	 * unmasked, a false interrupt will be received, keeping the
	 * device out of suspend. The interrupt does not need to be
	 * unmasked when resuming from suspend since the use case
	 * for having the interrupt unmasked is over.
	 */
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);

	return 0;
}
#else
#define cpcap_usb_det_suspend NULL
#endif

static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.suspend	= cpcap_usb_det_suspend,
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return cpcap_driver_register(&cpcap_usb_det_driver);
}
/* The CPCAP USB detection driver must be started later to give the MUSB
 * driver time to complete its initialization. */
late_initcall(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
