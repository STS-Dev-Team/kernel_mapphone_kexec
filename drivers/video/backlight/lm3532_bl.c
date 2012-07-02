/*
 * Backlight driver for National Semiconductor LM3532 Backlight Devices
 *
 * Copyright 2010-2011 Motorola Mobility Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/i2c/lm3532.h>

#define LM3532_OUTPUT_CFGR	0x10  /* Output Configuration */
#define LM3532_SUSD_RAMP	0x11  /* Start-up/Shutdown Ramp Rate */
#define LM3532_RUNTIME_RAMP	0x12  /* Run Time Ramp Rate */
#define LM3532_CTRL_A_PWM	0x13  /* Control A PWM */
#define LM3532_CTRL_B_PWM	0x14  /* Control B PWM */
#define LM3532_CTRL_C_PWM	0x15  /* Control C PWM */
#define LM3532_CTRL_A_BRT	0x16  /* Control A Brightness */
#define LM3532_CTRL_A_FSC	0x17  /* Control A Full Scale Current */
#define LM3532_CTRL_B_BRT	0x18  /* Control B Brightness */
#define LM3532_CTRL_B_FSC	0x19  /* Control B Full Scale Current */
#define LM3532_CTRL_C_BRT	0x1A  /* Control C Brightness */
#define LM3532_CTRL_C_FSC	0x1B  /* Control C Full Scale Current */
#define LM3532_FDBCK_EN		0x1C  /* Feedback Enable */
#define LM3532_CTRL_EN		0x1D  /* Control Enable */

#define LM3532_ALS1_RES_SLT	0x20  /* ALS1 Resistor Select */
#define LM3532_ALS2_RES_SLT	0x21  /* ALS2 Resistor Select */
#define LM3532_ALS_DWN_DELAY	0x22  /* ALS Down Delay */
#define LM3532_ALS_CFGR		0x23  /* ALS Configure */
#define LM3532_ALS_ZN_INFO	0x24  /* ALS Zone Information */
#define LM3532_ALS_BRT_ZN	0x25  /* ALS Brightness Zone */
#define LM3532_ALS_UP_ONLY_ZN	0x26  /* ALS Up Only Zone */
#define LM3532_ADC		0x27  /* ADC */
#define LM3532_ADC_AVG		0x28  /* ADC Average */

#define LM3532_REVISION		0xCC  /* Hardware Revision */

#define LM3532_ALS_ZB_1_HIGH	0x60  /* ALS Zone Boundry 1 High */
#define LM3532_ALS_ZB_1_LOW	0x61  /* ALS Zone Boundry 1 Low */
#define LM3532_ALS_ZB_2_HIGH	0x62  /* ALS Zone Boundry 2 High */
#define LM3532_ALS_ZB_2_LOW	0x63  /* ALS Zone Boundry 2 Low */
#define LM3532_ALS_ZB_3_HIGH	0x64  /* ALS Zone Boundry 3 High */
#define LM3532_ALS_ZB_3_LOW	0x65  /* ALS Zone Boundry 3 Low */
#define LM3532_ALS_ZB_4_HIGH	0x66  /* ALS Zone Boundry 4 High */
#define LM3532_ALS_ZB_4_LOW	0x67  /* ALS Zone Boundry 4 Low */

#define LM3532_CTRL_A_ZT_0	0x70  /* Control A Zone Target 0 */
#define LM3532_CTRL_A_ZT_1	0x71  /* Control A Zone Target 1 */
#define LM3532_CTRL_A_ZT_2	0x72  /* Control A Zone Target 2 */
#define LM3532_CTRL_A_ZT_3	0x73  /* Control A Zone Target 3 */
#define LM3532_CTRL_A_ZT_4	0x74  /* Control A Zone Target 4 */
#define LM3532_CTRL_B_ZT_0	0x75  /* Control B Zone Target 0 */
#define LM3532_CTRL_B_ZT_1	0x76  /* Control B Zone Target 1 */
#define LM3532_CTRL_B_ZT_2	0x77  /* Control B Zone Target 2 */
#define LM3532_CTRL_B_ZT_3	0x78  /* Control B Zone Target 3 */
#define LM3532_CTRL_B_ZT_4	0x79  /* Control B Zone Target 4 */
#define LM3532_CTRL_C_ZT_0	0x7A  /* Control C Zone Target 0 */
#define LM3532_CTRL_C_ZT_1	0x7B  /* Control C Zone Target 1 */
#define LM3532_CTRL_C_ZT_2	0x7C  /* Control C Zone Target 2 */
#define LM3532_CTRL_C_ZT_3	0x7D  /* Control C Zone Target 3 */
#define LM3532_CTRL_C_ZT_4	0x7E  /* Control C Zone Target 4 */

#define LM3532_COM_PTRN_0	0x80  /* COM Pattern 0 */
#define LM3532_COM_PTRN_1	0x81  /* COM Pattern 1 */
#define LM3532_COM_PTRN_2	0x82  /* COM Pattern 2 */
#define LM3532_COM_PTRN_3	0x83  /* COM Pattern 3 */
#define LM3532_COM_PTRN_4	0x84  /* COM Pattern 4 */
#define LM3532_COM_PTRN_5	0x85  /* COM Pattern 5 */
#define LM3532_COM_PTRN_6	0x86  /* COM Pattern 6 */
#define LM3532_COM_PTRN_7	0x87  /* COM Pattern 7 */
#define LM3532_COM_PTRN_8	0x88  /* COM Pattern 8 */
#define LM3532_COM_PTRN_9	0x89  /* COM Pattern 9 */
#define LM3532_COM_PTRN_10	0x8A  /* COM Pattern 10 */
#define LM3532_COM_PTRN_11	0x8B  /* COM Pattern 10 */
#define LM3532_COM_PTRN_12	0x8C  /* COM Pattern 10 */
#define LM3532_COM_PTRN_13	0x8D  /* COM Pattern 10 */
#define LM3532_COM_PTRN_14	0x8E  /* COM Pattern 10 */

#define LM3532_COM_OUT_VLT	0x90  /* COM Mode Output Voltage */
#define LM3532_COM_PTRN_CTRL	0x91  /* COM Pattern Control */

#define LM3532_FLAG_LED_MASK	0x3
#define LM3532_FDBCK_DIS_MASK	0x7

#define I2C_CTRL		(1 << 0)
#define LINEAR_MAP		(1 << 1)
#define ZONE_TARGET_4		(4 << 2)

#define LM3532_REV0		0xF6
#define LM3532_REV1		0xFF
#define LM3532_REV2		0xF3
#define LM3532_REV3		0xF4
#define LM3532_REV4		0xF8

struct lm3532_led {
	struct led_classdev	cdev;
	struct work_struct	work;
	struct i2c_client	*client;
	enum led_brightness	new_brightness;
	int			control;
};

struct lm3532_bl {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct lm3532_backlight_platform_data *pdata;
	struct mutex lock;
	unsigned long cached_daylight_max;
	int current_brightness;
	struct delayed_work work;
	uint8_t backlight_controller;
	struct lm3532_led *led_a;
	struct lm3532_led *led_b;
	struct lm3532_led *led_c;
	int revid;
};

static int lm3532_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}


static int lm3532_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int lm3532_set_bits(struct i2c_client *client, int reg, uint8_t bit_mask)
{
	struct lm3532_bl *data = i2c_get_clientdata(client);
	uint8_t reg_val;
	int ret;

	mutex_lock(&data->lock);

	ret = lm3532_read(client, reg, &reg_val);

	if (!ret && ((reg_val & bit_mask) != bit_mask)) {
		reg_val |= bit_mask;
		ret = lm3532_write(client, reg, reg_val);
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int lm3532_clr_bits(struct i2c_client *client, int reg, uint8_t bit_mask)
{
	struct lm3532_bl *data = i2c_get_clientdata(client);
	uint8_t reg_val;
	int ret;

	mutex_lock(&data->lock);

	ret = lm3532_read(client, reg, &reg_val);

	if (!ret && (reg_val & bit_mask)) {
		reg_val &= ~bit_mask;
		ret = lm3532_write(client, reg, reg_val);
	}

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * Independent sink / LED
 */
static void lm3532_led_work(struct work_struct *work)
{
	struct lm3532_led *led = container_of(work, struct lm3532_led, work);
	lm3532_write(led->client, LM3532_CTRL_A_ZT_4 + (led->control * 5),
			 led->new_brightness);
}

static void lm3532_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct lm3532_led *led;

	led = container_of(led_cdev, struct lm3532_led, cdev);
	led->new_brightness = value;
	schedule_work(&led->work);
}

static int __devinit lm3532_led_probe(struct i2c_client *client, int control)
{
	struct lm3532_backlight_platform_data *pdata =
		client->dev.platform_data;
	struct lm3532_bl *data = i2c_get_clientdata(client);
	struct lm3532_led *led_dat;
	int ret;

	led_dat = kzalloc(sizeof(*led_dat), GFP_KERNEL);
	if (led_dat == NULL) {
		dev_err(&client->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	led_dat->control = control;

	if (control == LM3532_CNTRL_A) {
		led_dat->cdev.name = pdata->ctrl_a_name;
		data->led_a = led_dat;
	} else if (control == LM3532_CNTRL_B) {
		led_dat->cdev.name = pdata->ctrl_b_name;
		data->led_b = led_dat;
	} else if (control == LM3532_CNTRL_C) {
		led_dat->cdev.name = pdata->ctrl_c_name;
		data->led_c = led_dat;
	} else {
		dev_err(&client->dev, "Invalid LED Control %d\n",
			control);
		ret = -EINVAL;
		goto err_free;
	}

	led_dat->cdev.brightness_set = lm3532_led_set;
	led_dat->cdev.brightness = LED_OFF;
	led_dat->client = client;
	led_dat->new_brightness = LED_OFF;
	INIT_WORK(&led_dat->work, lm3532_led_work);

	ret = led_classdev_register(&client->dev, &led_dat->cdev);
	if (ret) {
		dev_err(&client->dev, "failed to register LED %d\n",
			led_dat->control);
		goto err_free;
	}

	ret |= lm3532_clr_bits(client,
		LM3532_CTRL_A_BRT + (led_dat->control * 2),
		0xFF);
	ret |= lm3532_set_bits(client,
		LM3532_CTRL_A_BRT + (led_dat->control * 2),
		LINEAR_MAP | I2C_CTRL | ZONE_TARGET_4);
	ret |= lm3532_set_bits(client, LM3532_CTRL_EN,
		1 << led_dat->control);
	if (ret) {
		dev_err(&client->dev, "failed to write\n");
		goto err;
	}

	return 0;

 err:
	led_classdev_unregister(&led_dat->cdev);
	cancel_work_sync(&led_dat->work);

 err_free:
	kfree(led_dat);

	return ret;
}

static int __devexit lm3532_led_remove(struct lm3532_bl *data)
{
	if (data->led_a) {
		led_classdev_unregister(&data->led_a->cdev);
		cancel_work_sync(&data->led_a->work);
		kfree(data->led_a);
	}
	if (data->led_b) {
		led_classdev_unregister(&data->led_b->cdev);
		cancel_work_sync(&data->led_b->work);
		kfree(data->led_b);
	}
	if (data->led_c) {
		led_classdev_unregister(&data->led_c->cdev);
		cancel_work_sync(&data->led_c->work);
		kfree(data->led_c);
	}

	return 0;
}

static int lm3532_bl_set(struct backlight_device *bl, int brightness)
{
	struct lm3532_bl *data = bl_get_data(bl);
	struct i2c_client *client = data->client;
	int ret = 0;

	if (data->revid == LM3532_REV1)
		return lm3532_write(client,
			LM3532_CTRL_A_ZT_4 + (data->backlight_controller * 5),
			brightness);

	if (data->pdata->en_ambl_sens) {
		if ((brightness > 0) && (brightness < LM3532_MAX_BRIGHTNESS)) {
			/* Disable Ambient Light auto adjust */
			ret |= lm3532_set_bits(client,
				LM3532_CTRL_A_BRT
				+ (data->backlight_controller * 2),
				I2C_CTRL);
			ret |= lm3532_write(client,
				LM3532_CTRL_A_ZT_4
				+ (data->backlight_controller * 5),
				brightness);
		} else {
			/*
			 * MAX_BRIGHTNESS -> Enable Ambient Light auto adjust
			 * restore daylight l1 sysfs brightness
			 */
			ret |= lm3532_write(client,
				LM3532_CTRL_A_ZT_4
				+ (data->backlight_controller * 5),
				data->cached_daylight_max);
			ret |= lm3532_clr_bits(client,
				LM3532_CTRL_A_BRT
				+ (data->backlight_controller * 2),
				I2C_CTRL);
		}
	} else
		ret |= lm3532_write(client,
				LM3532_CTRL_A_ZT_4
				+ (data->backlight_controller * 5),
				brightness);

	if (data->current_brightness && brightness == 0)
		ret |= lm3532_clr_bits(client, LM3532_CTRL_EN,
				1 << data->backlight_controller);
	else if (data->current_brightness == 0 && brightness)
		ret |= lm3532_set_bits(client, LM3532_CTRL_EN,
				1 << data->backlight_controller);

	if (!ret)
		data->current_brightness = brightness;

	return ret;
}

static int lm3532_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return lm3532_bl_set(bl, brightness);
}

static int lm3532_bl_get_brightness(struct backlight_device *bl)
{
	struct lm3532_bl *data = bl_get_data(bl);

	return data->current_brightness;
}

static const struct backlight_ops lm3532_bl_ops = {
	.update_status	= lm3532_bl_update_status,
	.get_brightness	= lm3532_bl_get_brightness,
};

static void lm3532_bl_work(struct work_struct *work)
{
	struct lm3532_bl *data = container_of((struct delayed_work *)work,
						struct lm3532_bl,
						work);
	struct i2c_client *client = data->client;
	struct lm3532_backlight_platform_data *pdata = data->pdata;
	int ret = 0;

	ret |= lm3532_write(client, LM3532_CTRL_A_PWM, pdata->ctrl_a_pwm);
	ret |= lm3532_write(client, LM3532_CTRL_B_PWM, pdata->ctrl_b_pwm);
	ret |= lm3532_write(client, LM3532_CTRL_C_PWM, pdata->ctrl_c_pwm);

	if (ret)
		dev_err(&client->dev, "failed to initialize pwm\n");
}

static int lm3532_als_setup(struct lm3532_bl *data)
{
	struct i2c_client *client = data->client;
	struct lm3532_backlight_platform_data *pdata = data->pdata;
	int ret = 0;

	ret |= lm3532_write(client, LM3532_ALS1_RES_SLT,
		pdata->als1_res);
	ret |= lm3532_write(client, LM3532_ALS2_RES_SLT,
		pdata->als2_res);
	ret |= lm3532_write(client, LM3532_ALS_DWN_DELAY,
		pdata->als_dwn_delay);
	ret |= lm3532_write(client, LM3532_ALS_CFGR,
		pdata->als_cfgr);

	ret |= lm3532_write(client, LM3532_CTRL_A_ZT_4,
		pdata->ctrl_a_l4_daylight);
	ret |= lm3532_write(client, LM3532_CTRL_A_ZT_3,
		pdata->ctrl_a_l3_bright);
	ret |= lm3532_write(client, LM3532_CTRL_A_ZT_2,
		pdata->ctrl_a_l2_office);
	ret |= lm3532_write(client, LM3532_CTRL_A_ZT_2,
		pdata->ctrl_a_l1_indoor);
	ret |= lm3532_write(client, LM3532_CTRL_A_ZT_0,
		pdata->ctrl_a_l0_dark);

	ret |= lm3532_write(client, LM3532_CTRL_B_ZT_4,
		pdata->ctrl_b_l4_daylight);
	ret |= lm3532_write(client, LM3532_CTRL_B_ZT_3,
		pdata->ctrl_b_l3_bright);
	ret |= lm3532_write(client, LM3532_CTRL_B_ZT_2,
		pdata->ctrl_b_l2_office);
	ret |= lm3532_write(client, LM3532_CTRL_B_ZT_1,
		pdata->ctrl_b_l1_indoor);
	ret |= lm3532_write(client, LM3532_CTRL_B_ZT_0,
		pdata->ctrl_b_l0_dark);

	ret |= lm3532_write(client, LM3532_CTRL_C_ZT_4,
		pdata->ctrl_c_l4_daylight);
	ret |= lm3532_write(client, LM3532_CTRL_C_ZT_3,
		pdata->ctrl_c_l3_bright);
	ret |= lm3532_write(client, LM3532_CTRL_C_ZT_2,
		pdata->ctrl_c_l2_office);
	ret |= lm3532_write(client, LM3532_CTRL_C_ZT_1,
		pdata->ctrl_c_l1_indoor);
	ret |= lm3532_write(client, LM3532_CTRL_C_ZT_0,
		pdata->ctrl_c_l0_dark);

	ret |= lm3532_write(client, LM3532_ALS_ZB_4_HIGH,
		pdata->l4_high);
	ret |= lm3532_write(client, LM3532_ALS_ZB_4_LOW,
		pdata->l4_low);
	ret |= lm3532_write(client, LM3532_ALS_ZB_3_HIGH,
		pdata->l3_high);
	ret |= lm3532_write(client, LM3532_ALS_ZB_3_LOW,
		pdata->l3_low);
	ret |= lm3532_write(client, LM3532_ALS_ZB_2_HIGH,
		pdata->l2_high);
	ret |= lm3532_write(client, LM3532_ALS_ZB_2_LOW,
		pdata->l2_low);
	ret |= lm3532_write(client, LM3532_ALS_ZB_1_HIGH,
		pdata->l1_high);
	ret |= lm3532_write(client, LM3532_ALS_ZB_1_LOW,
		pdata->l1_low);

	return ret;
}

static int __devinit lm3532_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct backlight_device *bl;
	struct lm3532_bl *data;
	struct lm3532_backlight_platform_data *pdata =
		client->dev.platform_data;
	uint8_t reg_val;
	int ret;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	lm3532_write(client, LM3532_REVISION, 0xFF);
	ret = lm3532_read(client, LM3532_REVISION, &reg_val);
	if (ret < 0)
		return -EIO;

	if (reg_val == LM3532_REV0) {
		dev_err(&client->dev, "rev0 not supported\n");
		return -ENODEV;
	}

	if (reg_val != LM3532_REV1
			&& reg_val != LM3532_REV2
			&& reg_val != LM3532_REV3
			&& reg_val != LM3532_REV4) {
		dev_err(&client->dev, "unknown revision (Rev. %X)\n", reg_val);
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->revid = reg_val;
	data->client = client;
	data->pdata = pdata;
	data->current_brightness = 0;
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	if (data->revid == LM3532_REV1) {
		/* backlight must use LM3532_CNTRL_A and LM3532_LED_D1
		   since rev1 i2c read does not work */
		dev_info(&client->dev, "rev1 basic init only\n");
		data->backlight_controller = LM3532_CNTRL_A;
		ret |= lm3532_write(client, LM3532_CTRL_A_BRT, 0xF3);
		ret |= lm3532_write(client, LM3532_FDBCK_EN, 0xF9);
		ret |= lm3532_write(client, LM3532_CTRL_EN, 0xF9);
		if (ret) {
			ret = -EIO;
			goto out2;
		}

		INIT_DELAYED_WORK(&data->work, lm3532_bl_work);

		bl = backlight_device_register(dev_driver_string(&client->dev),
			&client->dev, data, &lm3532_bl_ops, NULL);
		if (IS_ERR(bl)) {
			dev_err(&client->dev, "failed to register backlight\n");
			ret = PTR_ERR(bl);
			goto out2;
		}

		bl->props.max_brightness =
			bl->props.brightness = LM3532_MAX_BRIGHTNESS;

		data->bl = bl;

		dev_info(&client->dev, "Rev. %X Backlight\n", data->revid);

		return 0;
	}

	ret |= lm3532_write(client, LM3532_CTRL_A_FSC, pdata->ctrl_a_fsc);
	ret |= lm3532_write(client, LM3532_CTRL_B_FSC, pdata->ctrl_b_fsc);
	ret |= lm3532_write(client, LM3532_CTRL_C_FSC, pdata->ctrl_c_fsc);

	ret |= lm3532_write(client, LM3532_SUSD_RAMP, pdata->susd_ramp);
	ret |= lm3532_write(client, LM3532_RUNTIME_RAMP, pdata->runtime_ramp);

	ret |= lm3532_clr_bits(client, LM3532_OUTPUT_CFGR, 0x03);
	ret |= lm3532_set_bits(client, LM3532_OUTPUT_CFGR,
		pdata->led1_controller);

	ret |= lm3532_clr_bits(client, LM3532_OUTPUT_CFGR, 0x0C);
	ret |= lm3532_set_bits(client, LM3532_OUTPUT_CFGR,
		(pdata->led2_controller << 2));

	ret |= lm3532_clr_bits(client, LM3532_OUTPUT_CFGR, 0x30);
	ret |= lm3532_set_bits(client, LM3532_OUTPUT_CFGR,
		(pdata->led3_controller << 4));

	ret |= lm3532_clr_bits(client, LM3532_FDBCK_EN, 0x03);
	if (pdata->ctrl_a_usage == LM3532_BACKLIGHT_DEVICE) {
		data->backlight_controller = LM3532_CNTRL_A;
		data->cached_daylight_max = pdata->ctrl_a_l4_daylight;

		if (pdata->led1_controller == LM3532_CNTRL_A)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x01);
		if (pdata->led2_controller == LM3532_CNTRL_A)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x02);
		if (pdata->led3_controller == LM3532_CNTRL_A)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x04);

		ret |= lm3532_write(client, LM3532_CTRL_A_BRT,
				LINEAR_MAP | I2C_CTRL | ZONE_TARGET_4);

		ret |= lm3532_set_bits(client, LM3532_CTRL_EN, 0x01);
	} else if (pdata->ctrl_b_usage == LM3532_BACKLIGHT_DEVICE) {
		data->backlight_controller = LM3532_CNTRL_B;
		data->cached_daylight_max = pdata->ctrl_b_l4_daylight;

		if (pdata->led1_controller == LM3532_CNTRL_B)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x01);
		if (pdata->led2_controller == LM3532_CNTRL_B)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x02);
		if (pdata->led3_controller == LM3532_CNTRL_B)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x04);

		ret |= lm3532_write(client, LM3532_CTRL_B_BRT,
				LINEAR_MAP | I2C_CTRL | ZONE_TARGET_4);

		ret |= lm3532_set_bits(client, LM3532_CTRL_EN, 0x02);
	} else if (pdata->ctrl_c_usage == LM3532_BACKLIGHT_DEVICE) {
		data->backlight_controller = LM3532_CNTRL_C;
		data->cached_daylight_max = pdata->ctrl_c_l4_daylight;

		if (pdata->led1_controller == LM3532_CNTRL_C)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x01);
		if (pdata->led2_controller == LM3532_CNTRL_C)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x02);
		if (pdata->led3_controller == LM3532_CNTRL_C)
			ret |= lm3532_set_bits(client, LM3532_FDBCK_EN, 0x04);

		ret |= lm3532_write(client, LM3532_CTRL_C_BRT,
				LINEAR_MAP | I2C_CTRL | ZONE_TARGET_4);

		ret |= lm3532_set_bits(client, LM3532_CTRL_EN, 0x04);
	} else {
		dev_err(&client->dev, "no backlight controller defined\n");
		goto out2;
	}

	if (ret)
		goto out2;

	if (pdata->en_ambl_sens) {
		ret = lm3532_als_setup(data);
		if (ret)
			goto out2;
	}

	INIT_DELAYED_WORK(&data->work, lm3532_bl_work);

	bl = backlight_device_register(dev_driver_string(&client->dev),
			&client->dev, data, &lm3532_bl_ops, NULL);
	if (IS_ERR(bl)) {
		dev_err(&client->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto out2;
	}

	bl->props.max_brightness =
		bl->props.brightness = LM3532_MAX_BRIGHTNESS;

	data->bl = bl;

	dev_info(&client->dev, "Rev. %X Backlight\n", data->revid);

	if (pdata->pwm_init_delay_ms)
		schedule_delayed_work(&data->work,
				msecs_to_jiffies(pdata->pwm_init_delay_ms));
	else {
		ret |= lm3532_write(client, LM3532_CTRL_A_PWM,
						pdata->ctrl_a_pwm);
		ret |= lm3532_write(client, LM3532_CTRL_B_PWM,
						pdata->ctrl_b_pwm);
		ret |= lm3532_write(client, LM3532_CTRL_C_PWM,
						pdata->ctrl_c_pwm);
		if (ret)
			goto out;
	}

	if (pdata->ctrl_a_usage == LM3532_LED_DEVICE)
		lm3532_led_probe(client, LM3532_CNTRL_A);
	if (pdata->ctrl_b_usage == LM3532_LED_DEVICE)
		lm3532_led_probe(client, LM3532_CNTRL_B);
	if (pdata->ctrl_c_usage == LM3532_LED_DEVICE)
		lm3532_led_probe(client, LM3532_CNTRL_C);

	return 0;

out:
	backlight_device_unregister(bl);
out2:
	cancel_delayed_work_sync(&data->work);
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&data->lock);
	kfree(data);

	return ret;
}

static int __devexit lm3532_remove(struct i2c_client *client)
{
	struct lm3532_bl *data = i2c_get_clientdata(client);

	if ((data->led_a) || (data->led_b) || (data->led_c))
		lm3532_led_remove(data);

	backlight_device_unregister(data->bl);
	cancel_delayed_work_sync(&data->work);
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&data->lock);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int lm3532_i2c_suspend(struct i2c_client *client, pm_message_t message)
{

	return 0;
}

static int lm3532_i2c_resume(struct i2c_client *client)
{

	return 0;
}
#else
#define lm3532_i2c_suspend NULL
#define lm3532_i2c_resume NULL
#endif

static const struct i2c_device_id lm3532_id[] = {
	{ "lm3532", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3532_id);

static struct i2c_driver lm3532_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe    = lm3532_probe,
	.remove   = __devexit_p(lm3532_remove),
	.suspend = lm3532_i2c_suspend,
	.resume  = lm3532_i2c_resume,
	.id_table = lm3532_id,
};

static int __init lm3532_init(void)
{
	return i2c_add_driver(&lm3532_driver);
}
module_init(lm3532_init);

static void __exit lm3532_exit(void)
{
	i2c_del_driver(&lm3532_driver);
}
module_exit(lm3532_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Motorola Mobility");
MODULE_DESCRIPTION("LM3532 Backlight driver");
MODULE_ALIAS("platform:lm3532-backlight");
