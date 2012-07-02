/*
 * Copyright (C) 2011 Motorola Mobility, Inc.
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
 */

#include <linux/ct405.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define CT405_I2C_RETRIES	5
#define CT405_I2C_RETRY_DELAY	10

#define CT405_COMMAND_SELECT		0x80
#define CT405_COMMAND_AUTO_INCREMENT	0x20
#define CT405_COMMAND_SPECIAL_FUNCTION	0x60
#define CT405_COMMAND_PROX_INT_CLEAR	0x05
#define CT405_COMMAND_ALS_INT_CLEAR	0x06

#define CT405_ENABLE			0x00
#define CT405_ENABLE_PIEN		(1<<5)
#define CT405_ENABLE_AIEN		(1<<4)
#define CT405_ENABLE_PEN		(1<<2)
#define CT405_ENABLE_AEN		(1<<1)
#define CT405_ENABLE_PON		(1<<0)

#define CT405_ATIME			0x01
#define CT405_PTIME			0x02
#define CT405_WTIME			0x03
#define CT405_AILTL			0x04
#define CT405_AILTH			0x05
#define CT405_AIHTL			0x06
#define CT405_AIHTH			0x07
#define CT405_PILTL			0x08
#define CT405_PILTH			0x09
#define CT405_PIHTL			0x0A
#define CT405_PIHTH			0x0B
#define CT405_PERS			0x0C
#define CT405_CONFIG			0x0D
#define CT405_PPCOUNT			0x0E

#define CT405_CONTROL			0x0F
#define CT405_CONTROL_PDIODE_CH0	0x10
#define CT405_CONTROL_PDIODE_CH1	0x20
#define CT405_CONTROL_PGAIN_HALF	0x04

#define CT405_ID			0x12

#define CT405_STATUS			0x13
#define CT405_STATUS_PINT		(1<<5)
#define CT405_STATUS_AINT		(1<<4)

#define CT405_C0DATA			0x14
#define CT405_C0DATAH			0x15
#define CT405_C1DATA			0x16
#define CT405_C1DATAH			0x17
#define CT405_PDATA			0x18
#define CT405_PDATAH			0x19

#define CT405_PROXIMITY_NEAR		30	/* 30 mm */
#define CT405_PROXIMITY_FAR		1000	/* 1 meter */

struct ct405_data {
	struct input_dev *dev;
	struct i2c_client *client;
	struct regulator *regulator;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct ct405_platform_data *pdata;
	struct miscdevice miscdevice;
	struct notifier_block pm_notifier;
	struct mutex mutex;
	unsigned int suspended;
	unsigned int regs_initialized;
	unsigned int oscillator_enabled;
	unsigned int prox_enabled;
	unsigned int als_enabled;
	unsigned int als_requested;
};

static struct ct405_data *ct405_misc_data;

static struct ct405_reg {
	const char *name;
	u8 reg;
} ct405_regs[] = {
	{ "ENABLE",	CT405_ENABLE },
	{ "ATIME",	CT405_ATIME },
	{ "PTIME",	CT405_PTIME },
	{ "WTIME",	CT405_WTIME },
	{ "AILTL",	CT405_AILTL },
	{ "AILTH",	CT405_AILTH },
	{ "AIHTL",	CT405_AIHTL },
	{ "AIHTH",	CT405_AIHTH },
	{ "PILTL",	CT405_PILTL },
	{ "PILTH",	CT405_PILTH },
	{ "PIHTL",	CT405_PIHTL },
	{ "PIHTH",	CT405_PIHTH },
	{ "PERS",	CT405_PERS },
	{ "CONFIG",	CT405_CONFIG },
	{ "PPCOUNT",	CT405_PPCOUNT },
	{ "CONTROL",	CT405_CONTROL },
	{ "ID",		CT405_ID },
	{ "STATUS",	CT405_STATUS },
	{ "C0DATA",	CT405_C0DATA },
	{ "C0DATAH",	CT405_C0DATAH },
	{ "C1DATA",	CT405_C1DATA },
	{ "C1DATAH",	CT405_C1DATAH },
	{ "PDATA",	CT405_PDATA },
	{ "PDATAH",	CT405_PDATAH },
};

#define CT405_DBG_INPUT			0x00000001
#define CT405_DBG_POWER_ON_OFF		0x00000002
#define CT405_DBG_ENABLE_DISABLE	0x00000004
#define CT405_DBG_IOCTL			0x00000008
#define CT405_DBG_SUSPEND_RESUME	0x00000010
static u32 ct405_debug = 0x00000000;
module_param_named(debug_mask, ct405_debug, uint, 0644);

static int ct405_i2c_read(struct ct405_data *ct, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = ct->client->addr,
			.flags = ct->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = ct->client->addr,
			.flags = (ct->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	buf[0] |= CT405_COMMAND_SELECT;
	do {
		err = i2c_transfer(ct->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(CT405_I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < CT405_I2C_RETRIES));

	if (err != 2) {
		pr_err("%s: read transfer error.\n", __func__);
		dev_err(&ct->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int ct405_i2c_write(struct ct405_data *ct, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = ct->client->addr,
			.flags = ct->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] |= CT405_COMMAND_SELECT;
	do {
		err = i2c_transfer(ct->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(CT405_I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < CT405_I2C_RETRIES));

	if (err != 1) {
		pr_err("%s: write transfer error.\n", __func__);
		dev_err(&ct->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int ct405_write_enable(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0};
	reg_data[0] = CT405_ENABLE;
	if (ct->oscillator_enabled || ct->als_enabled || ct->prox_enabled) {
		reg_data[1] |= CT405_ENABLE_PON;
		if (!ct->oscillator_enabled) {
			error = ct405_i2c_write(ct, reg_data, 1);
			if (error < 0)
				return error;
			msleep(3);
			ct->oscillator_enabled = 1;
		}
		if (ct->als_enabled)
			reg_data[1] |= CT405_ENABLE_AEN | CT405_ENABLE_AIEN;
		if (ct->prox_enabled)
			reg_data[1] |= CT405_ENABLE_PEN | CT405_ENABLE_PIEN;
	}
	if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
		pr_info("%s: writing ENABLE=0x%02x\n", __func__, reg_data[1]);

	return ct405_i2c_write(ct, reg_data, 1);
}

static int ct405_set_als_enable(struct ct405_data *ct,
				unsigned int enable)
{
	int error = 0;
	if (ct->als_enabled != enable) {
		ct->als_enabled = enable;
		if (ct->regs_initialized)
			error = ct405_write_enable(ct);
	}
	return error;
}

static int ct405_set_prox_enable(struct ct405_data *ct,
				 unsigned int enable)
{
	int error = 0;
	if (ct->prox_enabled != enable) {
		ct->prox_enabled = enable;
		if (ct->regs_initialized)
			error = ct405_write_enable(ct);
	}
	return error;
}

static int ct405_clear_als_flag(struct ct405_data *ct)
{
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_COMMAND_SPECIAL_FUNCTION
		| CT405_COMMAND_ALS_INT_CLEAR;
	return ct405_i2c_write(ct, reg_data, 0);
}

static int ct405_clear_prox_flag(struct ct405_data *ct)
{
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_COMMAND_SPECIAL_FUNCTION
		| CT405_COMMAND_PROX_INT_CLEAR;
	return ct405_i2c_write(ct, reg_data, 0);
}

static int ct405_init_registers(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0};

	/* write ALS integration time = ~100 ms */
	reg_data[0] = CT405_ATIME;
	reg_data[1] = 0xDB; /* ~101 ms */
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		return error;

	/* write IR LED pulse count = 2 */
	reg_data[0] = CT405_PPCOUNT;
	reg_data[1] = 2;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		return error;

	/* write proximity diode = ch1, gain = 1/2 */
	reg_data[0] = CT405_CONTROL;
	reg_data[1] = CT405_CONTROL_PDIODE_CH1 | CT405_CONTROL_PGAIN_HALF;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		return error;

	return 0;
}

static void ct405_report_prox(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0};
	unsigned int pdata = 0;
	int distance = CT405_PROXIMITY_FAR;
	int pilt = 50;
	int piht = 100;

	reg_data[0] = (CT405_PDATA | CT405_COMMAND_AUTO_INCREMENT);
	error = ct405_i2c_read(ct, reg_data, 2);
	if (error < 0)
		return;

	pdata = (reg_data[1] << 8) | reg_data[0];

	if (ct405_debug & CT405_DBG_INPUT)
		pr_info("%s: PDATA = %d\n", __func__, pdata);

	/* ITERATION 2: implement algorithm from CT405_Algorithm.doc */
	if (pdata < pilt)
		distance = CT405_PROXIMITY_FAR;
	if (pdata > piht)
		distance = CT405_PROXIMITY_NEAR;

	input_report_abs(ct->dev, ABS_DISTANCE, distance);
	input_sync(ct->dev);

	ct405_clear_prox_flag(ct);
}

static void ct405_report_als(struct ct405_data *ct)
{
	int error;
	u8 reg_data[4] = {0};
	unsigned int c0data;
	unsigned int c1data;
	unsigned int lux;

	reg_data[0] = (CT405_C0DATA | CT405_COMMAND_AUTO_INCREMENT);
	error = ct405_i2c_read(ct, reg_data, 4);
	if (error < 0)
		return;

	c0data = (reg_data[1] << 8) | reg_data[0];
	c1data = (reg_data[3] << 8) | reg_data[2];

	if (ct405_debug & CT405_DBG_INPUT)
		pr_info("%s: C0DATA = %d, C1DATA = %d\n",
			 __func__, c0data, c1data);

	lux = c0data; /* ITERATION 2: get & use correct lux equation. */

	/* input.c filters consecutive LED_MISC values <=1. */
	lux = (lux >= 2) ? lux : 2;

	input_event(ct->dev, EV_LED, LED_MISC, lux);
	input_sync(ct->dev);

	ct405_clear_als_flag(ct);
}

static unsigned int ct405_get_avg_noise_floor(struct ct405_data *ct)
{
	int err = -EINVAL;
	unsigned int num_samples = ct->pdata->prox_samples_for_noise_floor;
	unsigned int i, sum = 0, avg = 0;
	u8 reg_data[2] = {0};

	/* turn off prox sensor and wait */
	ct405_set_prox_enable(ct, 0);
	msleep(2);

	for (i = 0; i < num_samples; i++) {
		/* turn on prox sensor and wait */
		err = ct405_set_prox_enable(ct, 1);
		if (err) {
			pr_err("%s: Error enabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2);

		reg_data[0] = (CT405_PDATA | CT405_COMMAND_AUTO_INCREMENT);
		err = ct405_i2c_read(ct, reg_data, 2);
		if (err) {
			pr_err("%s: Error reading prox data: %d\n",
				__func__, err);
			break;
		}
		sum += (reg_data[1] << 8) | reg_data[0];

		/* turn off prox sensor and wait */
		err = ct405_set_prox_enable(ct, 0);
		if (err) {
			pr_err("%s: Error disabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2);
	}

	if (!err)
		avg = sum / num_samples;

	if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
		pr_info("%s: Noise floor is 0x%x ", __func__, avg);

	return avg;
}

static int ct405_set_prox_thresholds(struct ct405_data *ct,
				     int noise_floor)
{
	u8 reg_data[5] = {0};
	unsigned int pilt;
	unsigned int piht;

	/* ITERATION 2: implement algorithm from CT405_Algorithm.doc */
	if (noise_floor > ct->pdata->prox_saturation_threshold)
		noise_floor = ct->pdata->prox_saturation_threshold;

	pilt = noise_floor + ct->pdata->prox_uncovered_offset;
	piht = noise_floor + ct->pdata->prox_covered_offset;

	reg_data[0] = (CT405_PILTL | CT405_COMMAND_AUTO_INCREMENT);
	reg_data[1] = (pilt & 0xFF);
	reg_data[2] = ((pilt >> 8) & 0xFF);
	reg_data[3] = (piht & 0xFF);
	reg_data[4] = ((piht >> 8) & 0xFF);

	return ct405_i2c_write(ct, reg_data, 4);
}

static void ct405_device_power_off(struct ct405_data *ct)
{
	int error;

	if (ct405_debug & CT405_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, ct->regs_initialized);

	if (ct->regs_initialized && ct->regulator) {
		disable_irq_nosync(ct->client->irq);
		error = regulator_disable(ct->regulator);
		if (error) {
			pr_err("%s: regulator_disable failed: %d\n",
				__func__, error);
			enable_irq(ct->client->irq);
			return;
		}
		ct->regs_initialized = 0;
		ct->oscillator_enabled = 0;
	}
}

static int ct405_device_power_on(struct ct405_data *ct)
{
	int error;

	if (ct405_debug & CT405_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, ct->regs_initialized);

	if (!ct->regs_initialized) {
		if (ct->regulator) {
			error = regulator_enable(ct->regulator);
			if (error) {
				pr_err("%s: regulator_enable failed: %d\n",
					__func__, error);
				return error;
			}
		}

		error = ct405_init_registers(ct);
		if (error < 0) {
			pr_err("%s: init_registers failed: %d\n",
				__func__, error);
			if (ct->regulator)
				regulator_disable(ct->regulator);
			return error;
		}

		enable_irq(ct->client->irq);

		ct->regs_initialized = 1;
	}

	return 0;
}

static int ct405_enable_prox(struct ct405_data *ct)
{
	int error;
	int avg_noise_floor;

	if (ct->suspended) {
		if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering on\n", __func__);
		error = ct405_device_power_on(ct);
		if (error)
			return error;
	}

	if (!ct->prox_enabled) {
		avg_noise_floor = ct405_get_avg_noise_floor(ct);
		error = ct405_set_prox_thresholds(ct, avg_noise_floor);
		if (error) {
			pr_err("%s: Error setting prox thresholds: %d\n",
				__func__, error);
			return error;
		}

		error = ct405_set_prox_enable(ct, 1);
		if (error) {
			pr_err("%s: Error enabling prox: %d\n",
				__func__, error);
			return error;
		}

		ct405_report_prox(ct);
	}

	return 0;
}

static int ct405_disable_prox(struct ct405_data *ct)
{
	int error;

	if (ct->prox_enabled) {
		error = ct405_set_prox_enable(ct, 0);
		if (error) {
			pr_err("%s: Unable to turn off prox: %d\n",
				__func__, error);
			return error;
		}
		ct405_clear_prox_flag(ct);
	}

	if (ct->suspended && ct->regs_initialized) {
		if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering off\n", __func__);
		ct405_device_power_off(ct);
	}

	return 0;
}

static int ct405_enable_als(struct ct405_data *ct)
{
	int error;

	ct->als_requested = 1;
	if (!ct->als_enabled && !ct->suspended) {
		error = ct405_set_als_enable(ct, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		ct405_report_als(ct);
	}

	return 0;
}

static int ct405_disable_als(struct ct405_data *ct)
{
	int error;

	ct->als_requested = 0;
	if (ct->als_enabled) {
		error = ct405_set_als_enable(ct, 0);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
	}

	return 0;
}

static int ct405_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = ct405_misc_data;

	return 0;
}

static long ct405_misc_ioctl_locked(struct ct405_data *ct,
				    unsigned int cmd,
				    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;

	if (ct405_debug & CT405_DBG_IOCTL)
		pr_info("%s\n", __func__);

	switch (cmd) {
	case CT405_IOCTL_SET_PROX_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			ct405_enable_prox(ct);
		else
			ct405_disable_prox(ct);

		break;

	case CT405_IOCTL_GET_PROX_ENABLE:
		enable = ct->prox_enabled;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	case CT405_IOCTL_SET_LIGHT_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			ct405_enable_als(ct);
		else
			ct405_disable_als(ct);

		break;

	case CT405_IOCTL_GET_LIGHT_ENABLE:
		enable = ct->als_requested;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static long ct405_misc_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct ct405_data *ct = file->private_data;
	int error;

	if (ct405_debug & CT405_DBG_IOCTL)
		pr_info("%s: cmd = 0x%08X\n", __func__, cmd);

	mutex_lock(&ct->mutex);
	error = ct405_misc_ioctl_locked(ct, cmd, arg);
	mutex_unlock(&ct->mutex);

	return error;
}

static const struct file_operations ct405_misc_fops = {
	.owner = THIS_MODULE,
	.open = ct405_misc_open,
	.unlocked_ioctl = ct405_misc_ioctl,
};

static ssize_t ct405_registers_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ct405_data *ct = i2c_get_clientdata(client);
	int error = 0;
	unsigned int i, n, reg_count;
	u8 reg_data[1] = {0};

	reg_count = sizeof(ct405_regs) / sizeof(ct405_regs[0]);
	mutex_lock(&ct->mutex);
	for (i = 0, n = 0; i < reg_count; i++) {
		reg_data[0] = ct405_regs[i].reg;
		error = ct405_i2c_read(ct, reg_data, 1);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"%-20s = 0x%02X\n",
			ct405_regs[i].name,
			reg_data[0]);
	}
	mutex_unlock(&ct->mutex);

	return n;
}

static ssize_t ct405_registers_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ct405_data *ct = i2c_get_clientdata(client);
	unsigned int i, reg_count;
	unsigned int value;
	u8 reg_data[2] = {0};
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -EMSGSIZE;
	}

	if (sscanf(buf, "%30s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EINVAL;
	}

	reg_count = sizeof(ct405_regs) / sizeof(ct405_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, ct405_regs[i].name)) {
			mutex_lock(&ct->mutex);
			error = ct405_i2c_write(ct, reg_data, 1);
			mutex_unlock(&ct->mutex);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return error;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -EINVAL;
}

static DEVICE_ATTR(registers, 0644, ct405_registers_show,
		   ct405_registers_store);

static irqreturn_t ct405_irq_handler(int irq, void *dev)
{
	struct ct405_data *ct = dev;

	disable_irq_nosync(ct->client->irq);
	queue_work(ct->workqueue, &ct->work);

	return IRQ_HANDLED;
}

static void ct405_work_func_locked(struct ct405_data *ct)
{
	int error;
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_STATUS;
	error = ct405_i2c_read(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: Unable to read interrupt register: %d\n",
			__func__, error);
		return;
	}

	if (ct->als_enabled && (reg_data[0] & CT405_STATUS_AINT))
		ct405_report_als(ct);

	if (ct->prox_enabled && (reg_data[0] & CT405_STATUS_PINT))
		ct405_report_prox(ct);
}

static void ct405_work_func(struct work_struct *work)
{
	struct ct405_data *ct =
		container_of(work, struct ct405_data, work);

	mutex_lock(&ct->mutex);
	if (ct->regs_initialized)
		ct405_work_func_locked(ct);
	mutex_unlock(&ct->mutex);

	enable_irq(ct->client->irq);
}

static int ct405_suspend(struct ct405_data *ct)
{
	int error;

	if (ct->als_enabled) {
		error = ct405_set_als_enable(ct, 0);
		ct405_clear_als_flag(ct);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
		if (ct405_debug & CT405_DBG_SUSPEND_RESUME)
			pr_info("%s: turned off ALS\n", __func__);
	}

	if (!ct->prox_enabled)
		ct405_device_power_off(ct);

	ct->suspended = 1;

	return 0;
}

static int ct405_resume(struct ct405_data *ct)
{
	int error;

	ct405_device_power_on(ct);

	if (ct->als_requested) {
		error = ct405_set_als_enable(ct, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		ct405_report_als(ct);
	}

	ct->suspended = 0;

	return 0;
}

static int ct405_pm_event(struct notifier_block *this,
			  unsigned long event,
			  void *ptr)
{
	struct ct405_data *ct = container_of(this,
		struct ct405_data, pm_notifier);

	if (ct405_debug & CT405_DBG_SUSPEND_RESUME)
		pr_info("%s: event = %lu\n", __func__, event);

	mutex_lock(&ct->mutex);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		ct405_suspend(ct);
		break;
	case PM_POST_SUSPEND:
		ct405_resume(ct);
		break;
	}

	mutex_unlock(&ct->mutex);

	return NOTIFY_DONE;
}

static int ct405_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ct405_platform_data *pdata = client->dev.platform_data;
	struct ct405_data *ct;
	int error = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	client->irq = pdata->irq;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

	ct = kzalloc(sizeof(struct ct405_data), GFP_KERNEL);
	if (ct == NULL) {
		error = -ENOMEM;
		goto error_alloc_data_failed;
	}

	ct->client = client;
	ct->pdata = pdata;

	if (ct->pdata->regulator_name[0] != '\0') {
		ct->regulator = regulator_get(NULL,
			ct->pdata->regulator_name);
		if (IS_ERR(ct->regulator)) {
			pr_err("%s: cannot acquire regulator [%s]\n",
				__func__, ct->pdata->regulator_name);
			error = PTR_ERR(ct->regulator);
			goto error_regulator_get_failed;
		}
	}

	ct->dev = input_allocate_device();
	if (!ct->dev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed;
	}

	ct->dev->name = "light-prox";
	input_set_capability(ct->dev, EV_LED, LED_MISC);
	input_set_capability(ct->dev, EV_ABS, ABS_DISTANCE);

	ct405_misc_data = ct;
	ct->miscdevice.minor = MISC_DYNAMIC_MINOR;
	ct->miscdevice.name = LD_CT405_NAME;
	ct->miscdevice.fops = &ct405_misc_fops;
	error = misc_register(&ct->miscdevice);
	if (error < 0) {
		pr_err("%s: misc_register failed\n", __func__);
		goto error_misc_register_failed;
	}

	ct->pm_notifier.notifier_call = ct405_pm_event;
	error = register_pm_notifier(&ct->pm_notifier);
	if (error < 0) {
		pr_err("%s: register_pm_notifier failed\n", __func__);
		goto error_register_pm_notifier_failed;
	}

	ct->regs_initialized = 0;
	ct->suspended = 0;

	ct->oscillator_enabled = 0;
	ct->prox_enabled = 0;
	ct->als_enabled = 0;
	ct->als_requested = 0;

	ct->workqueue = create_singlethread_workqueue("als_wq");
	if (!ct->workqueue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&ct->work, ct405_work_func);

	error = request_irq(client->irq, ct405_irq_handler,
		IRQF_TRIGGER_LOW, LD_CT405_NAME, ct);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_req_irq_failed;
	}
	enable_irq_wake(client->irq);
	disable_irq(client->irq);

	i2c_set_clientdata(client, ct);

	mutex_init(&ct->mutex);

	error = input_register_device(ct->dev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
			error);
		goto error_input_register_failed;
	}

	error = device_create_file(&client->dev, &dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_create_registers_file_failed;
	}

	error = ct405_device_power_on(ct);
	if (error < 0) {
		pr_err("%s:power_on failed: %d\n", __func__, error);
		goto error_power_on_failed;
	}

	return 0;

error_power_on_failed:
	device_remove_file(&client->dev, &dev_attr_registers);
error_create_registers_file_failed:
	input_unregister_device(ct->dev);
	ct->dev = NULL;
error_input_register_failed:
	mutex_destroy(&ct->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(ct->client->irq, ct);
error_req_irq_failed:
	destroy_workqueue(ct->workqueue);
error_create_wq_failed:
	unregister_pm_notifier(&ct->pm_notifier);
error_register_pm_notifier_failed:
	misc_deregister(&ct->miscdevice);
error_misc_register_failed:
	input_free_device(ct->dev);
error_input_allocate_failed:
	regulator_put(ct->regulator);
error_regulator_get_failed:
	kfree(ct);
error_alloc_data_failed:
	return error;
}

static int ct405_remove(struct i2c_client *client)
{
	struct ct405_data *ct = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_registers);

	ct405_device_power_off(ct);

	input_unregister_device(ct->dev);

	mutex_destroy(&ct->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(ct->client->irq, ct);

	destroy_workqueue(ct->workqueue);

	unregister_pm_notifier(&ct->pm_notifier);

	misc_deregister(&ct->miscdevice);

	regulator_put(ct->regulator);

	kfree(ct);

	return 0;
}

static const struct i2c_device_id ct405_id[] = {
	{LD_CT405_NAME, 0},
	{}
};

static struct i2c_driver ct405_i2c_driver = {
	.probe		= ct405_probe,
	.remove		= ct405_remove,
	.id_table	= ct405_id,
	.driver = {
		.name = LD_CT405_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ct405_init(void)
{
	return i2c_add_driver(&ct405_i2c_driver);
}

static void __exit ct405_exit(void)
{
	i2c_del_driver(&ct405_i2c_driver);
}

module_init(ct405_init);
module_exit(ct405_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for CT405");
MODULE_AUTHOR("Motorola Mobility");
MODULE_LICENSE("GPL");
