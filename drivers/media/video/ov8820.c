/*
 * OmniVision OV8820 sensor driver
 *
 * Based on OmniVision OV5650/OV5640 driver
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include <media/ov8820.h>

//8820 only gives data in raw rgb also
/* OV5650 has only one fixed colorspace per pixelcode */
struct ov8820_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

//just support 8mp for now...who needs vga and what not on the main camera?
enum ov8820_size {
	OV8820_SIZE_8MP,
	OV8820_SIZE_LAST,
};

//updated, just biggest amount of pixels possible
static const struct v4l2_frmsize_discrete ov8820_frmsizes[OV8820_SIZE_LAST] = {
	{ 3264, 2448 },
};

/* Find a frame size in an array */
static int ov8820_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < OV8820_SIZE_LAST; i++) {
		if ((ov8820_frmsizes[i].width >= width) &&
		    (ov8820_frmsizes[i].height >= height))
			break;
	}

	/* If not found, select biggest */
	if (i >= OV8820_SIZE_LAST)
		i = OV8820_SIZE_LAST - 1;

	return i;
}

struct ov8820 {
	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrl_handler;

	const struct ov8820_platform_data *pdata;

	struct v4l2_ctrl *pixel_rate;

	/* HW control */
	struct clk *xvclk;
	struct regulator *avdd;
	struct regulator *dovdd;
};

static inline struct ov8820 *to_ov8820(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov8820, subdev);
}

/**
 * struct ov8820_reg - ov8820 register format
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * @length: length of the register
 *
 * Define a structure for OV8820 register initialization values
 */
struct ov8820_reg {
	u16	reg;
	u8	val;
};

//these registers will probly be a WIP until i figure out what ones we need fully...and what config they go into
//updated for now
//Also should dump these from stock rom sensor, will give us lenscalibrate and w/e

static const struct ov8820_reg configscript_common1[] = {
	//we disable DVP and because we just use MIPI to save some power
	{ 0x3100, 0x88 }, //reset clks
	{ 0x3101, 0x77 }, //disable DVP clks
	//think we need to be in single lane mode, could try 2 later for higher speed
	{ 0x300f, 0x08 }, //set 1 lane mode
	{ 0x3601, 0x00 }, //think we need this to move lane...not sure tho
	{ 0x300d, 0x00 }, //PLL charge pump control...might need to change it to 0x01 like 5640
	{ 0x3010, 0x32 }, //pll multiplier...im sure this will have to be changed
	{ 0x3300, 0xef }, //turn off lenc for now...dont have omnivisions tool to calibrate -_-
	{ 0x3301, 0x0a }, //turn on digital gain clk, might as well, needed for BLC
};

//updated for now
static const struct ov8820_reg configscript_common2[] = {
	{ 0x3013, 0x3F }, //turn on automatic correction stuff, aec, agc, banding, flicker
	{ 0x3014, 0x48 }, //flicker freq auto, laec enable...rather not calibrate stuff, just let it figure it out for now
	{ 0x30ec, 0xa7 }, //vcm controll..taking 5640 values
	{ 0x3601, 0x0f }, //reset this one for ecc calibration, copied from 5640
	{ 0x3100, 0x01 }, //reset ISP 
	{ 0x3100, 0x84 }, //reset DVP
};

static const struct ov8820_reg timing_cfg[][16] = {
	[OV8820_SIZE_8MP] = {
		//this stuff starts at 0x3024
		// 8mp is 3264x2448
		// 5mp is 2592x1944
		/*{ 0x3800, 0x02 }, // HREF Start Point: 596 
		{ 0x3801, 0x54 },

		{ 0x3803, 0x0c }, // VREF Start Point: 12

		{ 0x3804, 0x0a }, // HREF Width: 2592 
		{ 0x3805, 0x20 },

		{ 0x3806, 0x07 }, // HREF Height: 1944 
		{ 0x3807, 0x98 },

		{ 0x380c, 0x0c }, // Total Horizontal Size: 3252 
		{ 0x380d, 0xb4 },

		{ 0x380e, 0x07 }, // Total Vertical Size: 1968 
		{ 0x380f, 0xb0 },

		{ 0x3810, 0x40 }, */ // lets hope default works..will kill our framerate a bit though
	},
};

/**
 * ov8820_reg_read - Read a value from a register in an ov8820 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an ov8820 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8820_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 data[2] = {0};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;
	mdelay(3);
	msg.flags = I2C_M_RD;
	msg.len = 1;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	*val = data[0];
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/**
 * Write a value to a register in ov8820 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8820_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { (u8)(reg >> 8), (u8)(reg & 0xff), val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/**
 * Initialize a list of ov8820 registers.
 * The list of registers is terminated by the pair of values
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8820_reg_writes(struct i2c_client *client,
			     const struct ov8820_reg reglist[],
			     int size)
{
	int err = 0, i;

	for (i = 0; i < size; i++) {
		err = ov8820_reg_write(client, reglist[i].reg,
				reglist[i].val);
		if (err)
			return err;
	}
	return 0;
}

static int ov8820_reg_set(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 tmpval = 0;

	ret = ov8820_reg_read(client, reg, &tmpval);
	if (ret)
		return ret;

	return ov8820_reg_write(client, reg, tmpval | val);
}

static int ov8820_reg_clr(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 tmpval = 0;

	ret = ov8820_reg_read(client, reg, &tmpval);
	if (ret)
		return ret;

	return ov8820_reg_write(client, reg, tmpval & ~val);
}

static int ov8820_config_timing(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov8820 *ov8820 = to_ov8820(sd);
	int ret = 0, i;

	i = ov8820_find_framesize(ov8820->format.width, ov8820->format.height);

	ret = ov8820_reg_writes(client, timing_cfg[i],
			ARRAY_SIZE(timing_cfg[i]));
	return ret;
};

static struct v4l2_mbus_framefmt *
__ov8820_get_pad_format(struct ov8820 *ov8820, struct v4l2_subdev_fh *fh,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov8820->format;
	default:
		return NULL;
	}
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ov8820_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov8820 *ov8820 = to_ov8820(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;

	//jakedebug
	if (on) {
		int ret;

		if (ov8820->pdata->pre_poweron) {
			ret = ov8820->pdata->pre_poweron(sd);
			if (ret) {
				dev_err(dev,
					"Error in pre_poweron (%d)\n", ret);
				return ret;
			}
		}
		
		if (ov8820->dovdd) {
			ret = regulator_enable(ov8820->dovdd);
			if (ret) {
				dev_err(dev,
					"Error in enabling DOVDD (%d)\n", ret);
				if (ov8820->pdata->post_poweroff)
					ov8820->pdata->post_poweroff(sd);
				return ret;
			}
		}

		if (ov8820->avdd) {
			ret = regulator_enable(ov8820->avdd);
			if (ret) {
				dev_err(dev,
					"Error in enabling AVDD (%d)\n", ret);
				if (ov8820->dovdd)
					regulator_disable(ov8820->dovdd);
				if (ov8820->pdata->post_poweroff)
					ov8820->pdata->post_poweroff(sd);
				return ret;
			}
			usleep_range(5000, 5000);
		}

		ret = clk_enable(ov8820->xvclk);
		if (ret) {
			dev_err(dev, "Error in enabling XVCLK (%d)\n", ret);
			if (ov8820->avdd)
				regulator_disable(ov8820->avdd);
			if (ov8820->dovdd)
				regulator_disable(ov8820->dovdd);
			if (ov8820->pdata->post_poweroff)
				ov8820->pdata->post_poweroff(sd);
			return ret;
		}
		if (gpio_is_valid(ov8820->pdata->gpio_pwdn)) {
			gpio_set_value(ov8820->pdata->gpio_pwdn,
				       ov8820->pdata->is_gpio_pwdn_acthi ?
				       1 : 0);
		}
		usleep_range(2000, 2000);
	} else {
		if (gpio_is_valid(ov8820->pdata->gpio_pwdn)) {
			gpio_set_value(ov8820->pdata->gpio_pwdn,
				       ov8820->pdata->is_gpio_pwdn_acthi ?
				       0 : 1);
		}
		clk_disable(ov8820->xvclk);
		if (ov8820->avdd)
			regulator_disable(ov8820->avdd);
		if (ov8820->dovdd)
			regulator_disable(ov8820->dovdd);
		if (ov8820->pdata->post_poweroff)
			ov8820->pdata->post_poweroff(sd);
	}

	return 0;
}

static int ov8820_registered(struct v4l2_subdev *subdev)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct ov8820 *ov8820 = to_ov8820(subdev);
	int ret = 0;
	u8 chipid[2];

	ret = ov8820_s_power(subdev, 1);
	if (ret < 0) {
		dev_err(&client->dev, "OV8820 power up failed\n");
		return ret;
	}
	ret = ov8820_reg_read(client, 0x300A, &chipid[0]); // same
	if (ret) {
		dev_err(&client->dev, "Failure to read Chip ID (high byte)\n");
		goto out;
	}

	ret = ov8820_reg_read(client, 0x300B, &chipid[1]); // same
	if (ret) {
		dev_err(&client->dev, "Failure to read Chip ID (low byte)\n");
		goto out;
	}

	if ((chipid[0] != 0x88) || ((chipid[1] & 0x20) != chipid[1])) { // updated
		dev_err(&client->dev, "Chip ID: %x%x not supported!\n",
			chipid[0], chipid[1]);
		ret = -ENODEV;
		goto out;
	}

	dev_info(&client->dev, "Detected a OV%x%x chip, revision %s\n",
			chipid[0], chipid[1],
			(chipid[1] == 0x20) ? "1A": "1B/1C"); // only 1 revision, oh well
	/* Init controls */
	ret = v4l2_ctrl_handler_init(&ov8820->ctrl_handler, 1);
	if (ret)
		goto out;

	ov8820->pixel_rate = v4l2_ctrl_new_std(
		&ov8820->ctrl_handler, NULL,
		V4L2_CID_PIXEL_RATE,
		0, 0, 1, 0);

	subdev->ctrl_handler = &ov8820->ctrl_handler;
out:
	ov8820_s_power(subdev, 0);

	return ret;
}

static int ov8820_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_try_format(fh, 0);
	format->code = V4L2_MBUS_FMT_SGRBG8_1X8;
	format->width = ov8820_frmsizes[OV8820_SIZE_8MP].width;
	format->height = ov8820_frmsizes[OV8820_SIZE_8MP].height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int ov8820_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int ov8820_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov8820 *ov8820 = to_ov8820(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (enable) {
		/* SW Reset */
		ret = ov8820_reg_set(client, 0x30FA, 0x80); // updated
		if (ret)
			goto out;

		msleep(2);

		ret = ov8820_reg_clr(client, 0x30FA, 0x80); // updated
		if (ret)
			goto out;

		/* SW Powerdown */
		ret = ov8820_reg_set(client, 0x30FA, 0x00); // not sure if this works ...possibly 3012
		if (ret)
			goto out;

		ret = ov8820_reg_writes(client, configscript_common1,
				ARRAY_SIZE(configscript_common1));
		if (ret)
			goto out;

		ret = ov8820_config_timing(sd);
		if (ret)
			return ret;

		ret = ov8820_reg_writes(client, configscript_common2,
				ARRAY_SIZE(configscript_common2));
		if (ret)
			goto out;

		switch ((u32)ov8820->format.code) {
		//1x10 is always true for raw10..ignoring ix8 values
		case V4L2_MBUS_FMT_SGRBG8_1X8:
			ret = ov8820_reg_write(client, 0x300f, 0x8a);
			if (ret)
				return ret;

			ret = ov8820_reg_write(client, 0x3010, 0x10);
			if (ret)
				return ret;

			ret = ov8820_reg_write(client, 0x3011, 0x10);
			if (ret)
				return ret;

			ret = ov8820_reg_write(client, 0x3012, 0x02);
			if (ret)
				return ret;

			break;
		case V4L2_MBUS_FMT_SGRBG10_1X10:
			ret = ov8820_reg_write(client, 0x300f, 0x08);
			if (ret)
				return ret;
			/*
			ret = ov8820_reg_write(client, 0x3010, 0x10);
			if (ret)
				return ret;

			ret = ov8820_reg_write(client, 0x3011, 0x14);
			if (ret)
				return ret;

			ret = ov8820_reg_write(client, 0x3012, 0x02);
			if (ret)
				return ret;
			*/ //lets try defaults
			break;
		default:
			/* This shouldn't happen */
			ret = -EINVAL;
			return ret;
		}

		ret = ov8820_reg_clr(client, 0x30FA, 0x00); //lets leave this
		if (ret)
			goto out;

	} else {
		ret = ov8820_reg_set(client, 0x30FA, 0x00); // lets leave this
		if (ret)
			goto out;
	}

out:
	return ret;
}

static int ov8820_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *format)
{
	struct ov8820 *ov8820 = to_ov8820(sd);

	format->format = *__ov8820_get_pad_format(ov8820, fh, format->pad,
						   format->which);

	return 0;
}

static int ov8820_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *format)
{
	struct ov8820 *ov8820 = to_ov8820(sd);
	struct v4l2_mbus_framefmt *__format;

	__format = __ov8820_get_pad_format(ov8820, fh, format->pad,
					    format->which);

	*__format = format->format;

	/* NOTE: This is always true for now, revisit later. */
	/* For RAW10, pixelrate is 80% lower, since the same
	 * Mbps ratio is preserved, but more bits per pixel are
	 * transmitted.
	 */
	if (__format->code == V4L2_MBUS_FMT_SGRBG10_1X10)
		ov8820->pixel_rate->cur.val64 = 96000000;
	else if (__format->code == V4L2_MBUS_FMT_SGRBG8_1X8)
		ov8820->pixel_rate->cur.val64 = 120000000;

	return 0;
}

static struct v4l2_subdev_internal_ops ov8820_subdev_internal_ops = {
	.registered = ov8820_registered,
	.open = ov8820_open,
	.close = ov8820_close,
};

static int ov8820_enum_fmt(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= 2)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = V4L2_MBUS_FMT_SGRBG8_1X8;
		break;
	case 1:
		code->code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;
	}
	return 0;
}

static int ov8820_enum_framesizes(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if ((fse->index >= OV8820_SIZE_LAST) ||
	    (fse->code != V4L2_MBUS_FMT_SGRBG8_1X8 &&
	     fse->code != V4L2_MBUS_FMT_SGRBG10_1X10))
		return -EINVAL;

	fse->min_width = ov8820_frmsizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov8820_frmsizes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov8820_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	/* REVISIT */
	*frames = 3;

	return 0;
}

static struct v4l2_subdev_core_ops ov8820_subdev_core_ops = {
	.s_power	= ov8820_s_power,
};

static struct v4l2_subdev_video_ops ov8820_subdev_video_ops = {
	.s_stream	= ov8820_s_stream,
};

static struct v4l2_subdev_pad_ops ov8820_subdev_pad_ops = {
	.enum_mbus_code = ov8820_enum_fmt,
	.enum_frame_size = ov8820_enum_framesizes,
	.get_fmt = ov8820_g_fmt,
	.set_fmt = ov8820_s_fmt,
};

static struct v4l2_subdev_sensor_ops ov8820_subdev_sensor_ops = {
	.g_skip_frames	= &ov8820_g_skip_frames,
};

static struct v4l2_subdev_ops ov8820_subdev_ops = {
	.core	= &ov8820_subdev_core_ops,
	.video	= &ov8820_subdev_video_ops,
	.pad	= &ov8820_subdev_pad_ops,
	.sensor	= &ov8820_subdev_sensor_ops,
};

static int ov8820_get_resources(struct ov8820 *ov8820, struct device *dev)
{
	const struct ov8820_platform_data *pdata = ov8820->pdata;
	int ret = 0;

	ov8820->xvclk = clk_get(dev, pdata->clk_xvclk);
	if (IS_ERR(ov8820->xvclk)) {
		dev_err(dev, "Unable to get XVCLK (%s)\n", pdata->clk_xvclk);
		return -ENODEV;
	}

	if (clk_round_rate(ov8820->xvclk, 24000000) != 24000000)
		dev_warn(dev, "XVCLK set to rounded aproximate (%lu Hz)",
			 clk_round_rate(ov8820->xvclk, 24000000));

	if (clk_set_rate(ov8820->xvclk,
			 clk_round_rate(ov8820->xvclk, 24000000))) {
		dev_err(dev, "Unable to change XVCLK (%s) rate!\n",
			pdata->clk_xvclk);
		ret = -EINVAL;
		goto err_clk_set_rate;
	}

	if (!pdata->reg_avdd)
		goto get_reg_dovdd;

	ov8820->avdd = regulator_get(dev, pdata->reg_avdd);
	if (IS_ERR(ov8820->avdd)) {
		dev_err(dev, "Unable to get AVDD (%s) regulator\n",
			pdata->reg_avdd);
		ret = -ENODEV;
		goto err_reg_avdd;
	}

	//fixed voltage regulator
	/*if (regulator_set_voltage(ov8820->avdd, 2800000, 2800000)) {
		dev_err(dev, "Unable to set valid AVDD (%s) regulator"
			" voltage to: 2.8V\n", pdata->reg_avdd);
		ret = -ENODEV;
		regulator_put(ov8820->avdd);
		goto err_reg_avdd;
	}*/

get_reg_dovdd:
	if (!pdata->reg_dovdd)
		goto get_gpio_pwdn;

	ov8820->dovdd = regulator_get(dev, pdata->reg_dovdd);
	if (IS_ERR(ov8820->dovdd)) {
		dev_err(dev, "Unable to get DOVDD (%s) regulator\n",
			pdata->reg_dovdd);
		ret = -ENODEV;
		goto err_reg_dovdd;
	}
	//cant change voltage, fixed voltage
	/*if (regulator_set_voltage(ov8820->dovdd, 1800000, 1800000)) {
		dev_err(dev, "Unable to set valid DOVDD (%s) regulator"
			" voltage to: 1.8V\n", pdata->reg_dovdd);
		ret = -ENODEV;
		regulator_put(ov8820->dovdd);
		goto err_reg_dovdd;
	}*/

get_gpio_pwdn:
	if (!gpio_is_valid(pdata->gpio_pwdn))
		goto get_gpio_resetb;

	if (gpio_request_one(pdata->gpio_pwdn,
			     pdata->is_gpio_pwdn_acthi ?
			     GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "OV8820_PWDN")) {
		dev_err(dev, "Cannot request GPIO %d\n", pdata->gpio_pwdn);
		ret = -ENODEV;
		goto err_gpio_pwdn;
	}

get_gpio_resetb:
	if (!gpio_is_valid(pdata->gpio_resetb))
		goto out;

	if (gpio_request_one(pdata->gpio_resetb,
			     pdata->is_gpio_resetb_acthi ?
			     GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "OV8820_RESETB")) {
		dev_err(dev, "Cannot request GPIO %d\n", pdata->gpio_resetb);
		ret = -ENODEV;
		goto err_gpio_resetb;
	}

out:
	return 0;

err_gpio_resetb:
	if (gpio_is_valid(pdata->gpio_pwdn))
		gpio_free(pdata->gpio_pwdn);
err_gpio_pwdn:
	regulator_put(ov8820->dovdd);
err_reg_dovdd:
	regulator_put(ov8820->avdd);
err_reg_avdd:
err_clk_set_rate:
	clk_put(ov8820->xvclk);

	return ret;
}

static void ov8820_put_resources(struct ov8820 *ov8820)
{
	if (gpio_is_valid(ov8820->pdata->gpio_resetb))
		gpio_free(ov8820->pdata->gpio_resetb);
	if (gpio_is_valid(ov8820->pdata->gpio_pwdn))
		gpio_free(ov8820->pdata->gpio_pwdn);
	regulator_put(ov8820->dovdd);
	regulator_put(ov8820->avdd);
	clk_put(ov8820->xvclk);
}

static int ov8820_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov8820 *ov8820;
	int ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "No platform data!!\n");
		return -ENODEV;
	}

	ov8820 = kzalloc(sizeof(*ov8820), GFP_KERNEL);
	if (!ov8820)
		return -ENOMEM;

	ov8820->pdata = client->dev.platform_data;

	ret = ov8820_get_resources(ov8820, &client->dev);
	if (ret) {
		kfree(ov8820);
		return ret;
	}

	ov8820->format.code = V4L2_MBUS_FMT_SGRBG8_1X8;
	ov8820->format.width = ov8820_frmsizes[OV8820_SIZE_8MP].width;
	ov8820->format.height = ov8820_frmsizes[OV8820_SIZE_8MP].height;
	ov8820->format.field = V4L2_FIELD_NONE;
	ov8820->format.colorspace = V4L2_COLORSPACE_SRGB;

	v4l2_i2c_subdev_init(&ov8820->subdev, client, &ov8820_subdev_ops);
	ov8820->subdev.internal_ops = &ov8820_subdev_internal_ops;
	ov8820->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov8820->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ov8820->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ov8820->subdev.entity, 1, &ov8820->pad, 0);
	if (ret < 0) {
		media_entity_cleanup(&ov8820->subdev.entity);
		ov8820_put_resources(ov8820);
		kfree(ov8820);
	}

	return ret;
}

static int ov8820_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ov8820 *ov8820 = to_ov8820(subdev);

	v4l2_ctrl_handler_free(&ov8820->ctrl_handler);
	media_entity_cleanup(&subdev->entity);
	v4l2_device_unregister_subdev(subdev);
	ov8820_put_resources(ov8820);
	kfree(ov8820);
	return 0;
}

static const struct i2c_device_id ov8820_id[] = {
	{ "ov8820", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov8820_id);

static struct i2c_driver ov8820_i2c_driver = {
	.driver = {
		.name = "ov8820",
	},
	.probe		= ov8820_probe,
	.remove		= ov8820_remove,
	.id_table	= ov8820_id,
};

static int __init ov8820_mod_init(void)
{
	return i2c_add_driver(&ov8820_i2c_driver);
}

static void __exit ov8820_mod_exit(void)
{
	i2c_del_driver(&ov8820_i2c_driver);
}

module_init(ov8820_mod_init);
module_exit(ov8820_mod_exit);

MODULE_DESCRIPTION("OmniVision OV8820 Camera driver");
MODULE_AUTHOR("JakeTheSnake651");
MODULE_LICENSE("GPL v2");
