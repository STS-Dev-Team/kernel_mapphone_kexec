/*
 * arch/arm/mach-omap2/board-mapphone-vibrator.c
 *
 * Copyright (C) 2011 Motorola Mobility Inc.
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

#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/vib-timed.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

#include <plat/dmtimer.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAX_VIBS		2
#define MAX_PWMS		4

#define VIB_TYPE_GENENIC_ROTARY	0x00000017
#define VIB_TYPE_GENENIC_LINEAR	0x00000002

#define ACTIVE_HIGH		1
#define ACTIVE_LOW		0

#define SIGNAL_GPIO		0x1F
#define SIGNAL_PWM		0x2F

#define MIN_TIMEOUT		0
#define MAX_TIMEOUT		60000000

#define REGULATOR_NAME_SIZE	16

#define VIB_EN			"vib_en"
#define VIB_DIR			"vib_dir"

#define SIGNAL_ENABLE		0x2A
#define SIGNAL_DIRECTION	0x3A

#define GPIO_STAGE_ACTIVE	1
#define GPIO_STAGE_INACTIVE	2

struct vib_pwm {
	unsigned int time;
	unsigned int period;
	unsigned int duty_en;
	unsigned int duty_dir;
};

struct vib_ctrl_gpio {
	struct hrtimer hrtimer;
	int stage;
	unsigned int active_us;
	unsigned int inactive_us;
};

struct vib_ctrl_pwm {
	struct omap_dm_timer *pwm_timer;
	unsigned int active_us;
	int cycles;
};

struct vib_signal;

struct vib_of_signal {
	int type;
	int active_level;
	int pwm;
	int gpio;
};

struct vib_ctrl_ops {
	int (*init)(struct vib_signal *);
	int (*configure)(struct vib_signal *, unsigned int,
			unsigned int, unsigned int);
	int (*activate)(struct vib_signal *);
	int (*deactivate)(struct vib_signal *);
};

struct vib_signal {
	int signal_type;
	struct vib_of_signal of;
	struct vib_ctrl_pwm pwmc;
	struct vib_ctrl_gpio gpioc;
	struct vib_ctrl_ops *ops;
	const char *name;
};


struct vib_control {
	struct vib_signal vib_en;
	struct vib_signal vib_dir;
	struct vib_pwm vib_pwm[MAX_PWMS];
};

struct vibrator {
	int type;
	struct regulator *regulator;
	struct wake_lock wakelock;
	char regulator_name[REGULATOR_NAME_SIZE];
	struct vib_control ctrl;
	unsigned int min_us;
	unsigned int max_us;
};

struct vibrator vibrators[MAX_VIBS]; /* dev_data */
struct vib_timed vib_timeds[MAX_VIBS]; /* pdata */
const char *vib_name[MAX_VIBS] = {"vibrator", "vibrator1"};

static void vib_signal_print(struct vib_signal *vibs)
{
	struct vib_of_signal *of = &vibs->of;
	if (vibs->name)
		zprintk("%s: %x %d %d %d\n", vibs->name, of->type,
			of->active_level, of->pwm, of->gpio);
}
static void vib_print_pwm(struct vib_pwm *pwm)
{
	if (pwm->time) {
		zprintk("%u %u %u %u\n", pwm->time, pwm->period,
			pwm->duty_en, pwm->duty_dir);
	}
}

static enum hrtimer_restart gpioc_hrtimer_func(struct hrtimer *hrtimer)
{
	struct vib_ctrl_gpio *gpioc = container_of(hrtimer,
				struct vib_ctrl_gpio, hrtimer);
	struct vib_signal *vibs = container_of(gpioc,
				struct vib_signal, gpioc);
	struct vib_of_signal *of = &vibs->of;

	if (gpioc->stage == GPIO_STAGE_ACTIVE) {
		if (vibs->signal_type == SIGNAL_ENABLE) {
			gpio_direction_output(of->gpio, !of->active_level);
			dvib_tprint("g-t %s\n", vibs->name);
			gpioc->stage = GPIO_STAGE_INACTIVE;
		}
		if (gpioc->inactive_us) {
			if (vibs->signal_type == SIGNAL_DIRECTION) {
				gpio_direction_output(of->gpio,
						!of->active_level);
				dvib_tprint("g-t %s\n", vibs->name);
				gpioc->stage = GPIO_STAGE_INACTIVE;
			}
			hrtimer_start(&gpioc->hrtimer,
				ns_to_ktime((u64) gpioc->inactive_us
					* NSEC_PER_USEC),
				HRTIMER_MODE_REL);
		}
	} else {
		if (gpioc->active_us) {
			dvib_tprint("g+t %s\n", vibs->name);
			gpio_direction_output(of->gpio, of->active_level);
			gpioc->stage = GPIO_STAGE_ACTIVE;
			hrtimer_start(&gpioc->hrtimer,
				ns_to_ktime((u64) gpioc->active_us
					* NSEC_PER_USEC),
				HRTIMER_MODE_REL);
		}
	}
	return HRTIMER_NORESTART;
}

static int vib_ctrl_gpio_init(struct vib_signal *vibs)
{
	struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
	struct vib_of_signal *of = &vibs->of;
	int ret;
	ret = gpio_request(of->gpio, "vib_ctrl");
	if (ret) {
		zfprintk("gpio request %d failed %d\n", of->gpio, ret);
		return ret;
	}
	ret = gpio_direction_output(of->gpio, !of->active_level);
	if (ret) {
		zfprintk("gpio %d output %d failed %d\n", of->gpio,
			!of->active_level, ret);
		return ret;
	}

	hrtimer_init(&gpioc->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gpioc->hrtimer.function = gpioc_hrtimer_func;
	return 0;
}

static int vib_ctrl_gpio_activate(struct vib_signal *vibs)
{
	struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
	struct vib_of_signal *of = &vibs->of;
	int ret;
	dvib_tprint("g+ %s\n", vibs->name);
	if (!gpioc->active_us)
		return 0;
	ret = gpio_direction_output(of->gpio, of->active_level);
	if (ret) {
		zfprintk("gpio %d output %d failed %d\n", of->gpio,
			of->active_level, ret);
		return ret;
	}
	gpioc->stage = GPIO_STAGE_ACTIVE;
	ret = hrtimer_start(&gpioc->hrtimer,
		ns_to_ktime((u64) gpioc->active_us * NSEC_PER_USEC),
		HRTIMER_MODE_REL);
	if (ret)
		dvib_tprint("started timer %p while active.\n",
				&gpioc->hrtimer);
	return 0;
}

static int vib_ctrl_gpio_deactivate(struct vib_signal *vibs)
{
	struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
	struct vib_of_signal *of = &vibs->of;
	int ret;
	ret = hrtimer_cancel(&gpioc->hrtimer);
	dvib_tprint("g- %s %s\n", vibs->name, ret ? "a" : "na");
	ret = gpio_direction_output(of->gpio, !of->active_level);
	if (ret) {
		zfprintk("gpio %d output %d failed %d\n", of->gpio,
			!of->active_level, ret);
		return ret;
	}
	gpioc->stage = GPIO_STAGE_INACTIVE;
	return 0;
}

static int vib_ctrl_gpio_config(struct vib_signal *vibs, unsigned int total_us,
			unsigned int period_us, unsigned int duty_us)
{
	struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
	vib_ctrl_gpio_deactivate(vibs);
	gpioc->active_us = duty_us;
	gpioc->inactive_us = period_us - duty_us;
	dvib_print("\t\t%s\tT %u A %u N %u\n", vibs->name,
		total_us, gpioc->active_us, gpioc->inactive_us);
	return 0;
}

static irqreturn_t omap_pwm_timer_intr(int irq, void *data)
{
	struct vib_signal *vibs = data;
	struct vib_ctrl_pwm *pwmc = &vibs->pwmc;
	struct omap_dm_timer *dmtimer = pwmc->pwm_timer;
	uint32_t status = omap_dm_timer_read_status(dmtimer);
	omap_dm_timer_write_status(dmtimer, status);
	if (status & OMAP_TIMER_INT_OVERFLOW) {
		if (pwmc->cycles-- == 0) {
			dvib_tprint("p-i %s\n", vibs->name);
			omap_dm_timer_stop(dmtimer);
		}
	}
	return IRQ_HANDLED;
}

static int vib_ctrl_pwm_init(struct vib_signal *vibs)
{
	struct vib_ctrl_pwm *pwmc = &vibs->pwmc;
	struct vib_of_signal *of = &vibs->of;
	struct omap_dm_timer *dmtimer;
	int ret;

	dmtimer = omap_dm_timer_request_specific(of->pwm);
	if (!dmtimer) {
		zfprintk("request timer %d failed\n", of->pwm);
		return -ENODEV;
	}
	ret = request_irq(omap_dm_timer_get_irq(dmtimer), omap_pwm_timer_intr,
			IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
			"vib_pwm", vibs);
	if (ret) {
		zfprintk("request irq for timer %d failed %d\n", of->pwm, ret);
		return ret;
	}
	pwmc->pwm_timer = dmtimer;
	return 0;
}

static int vib_ctrl_pwm_activate(struct vib_signal *vibs)
{
	struct vib_ctrl_pwm *pwmc = &vibs->pwmc;
	struct omap_dm_timer *dmtimer = pwmc->pwm_timer;
	dvib_tprint("p+ %s\n", vibs->name);
	if (!pwmc->active_us)
		return 0;
	omap_dm_timer_start(dmtimer);
	return 0;
}

static int vib_ctrl_pwm_deactivate(struct vib_signal *vibs)
{
	struct vib_ctrl_pwm *pwmc = &vibs->pwmc;
	struct omap_dm_timer *dmtimer = pwmc->pwm_timer;
	dvib_tprint("p- %s\n", vibs->name);
	omap_dm_timer_enable(dmtimer);
	omap_dm_timer_set_int_disable(dmtimer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_stop(dmtimer);
	return 0;
}

static int vib_ctrl_pwm_config(struct vib_signal *vibs, unsigned int total_us,
			unsigned int period_us, unsigned int duty_us)
{
	struct vib_ctrl_pwm *pwmc = &vibs->pwmc;
	struct omap_dm_timer *dmtimer = pwmc->pwm_timer;
	unsigned long rate = clk_get_rate(omap_dm_timer_get_fclk(dmtimer));
	unsigned int load_reg, match_reg;
	u64 tmp;
#define DM_FILLER		20

	vib_ctrl_pwm_deactivate(vibs);
	dvib_print("\t\t%s\tT %u P %u D %u\n", vibs->name,
			total_us, period_us, duty_us);
	pwmc->active_us = duty_us;
	if (duty_us == 0)
		return 0;

	tmp = (u64) rate * period_us;
	do_div(tmp, USEC_PER_SEC);
	load_reg = tmp;

	tmp = (u64) rate * (period_us - duty_us);
	do_div(tmp, USEC_PER_SEC);
	match_reg = tmp;

	if (total_us == period_us) {
		load_reg += DM_FILLER;
		match_reg += DM_FILLER;
	}
	dvib_print("\t\t\tL %u M %u\n", load_reg, match_reg);

	omap_dm_timer_enable(dmtimer);

	omap_dm_timer_set_load(dmtimer, 1 /* auto_reload */, -load_reg);
	omap_dm_timer_set_match(dmtimer, 1 /* do_match */, -match_reg);

	omap_dm_timer_set_pwm(dmtimer, !(vibs->of.active_level), 1 /* Toggle */,
		       OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(dmtimer, 0xfffffffe);
	omap_dm_timer_invalidate_saved_context(dmtimer);

	if (total_us == period_us) {
		pwmc->cycles = 1;
		omap_dm_timer_set_int_enable(dmtimer,
				OMAP_TIMER_INT_OVERFLOW);
	}
	return 0;
}

static struct vib_ctrl_ops vib_ctrl_gpio_ops = {
	.init		= vib_ctrl_gpio_init,
	.configure	= vib_ctrl_gpio_config,
	.activate	= vib_ctrl_gpio_activate,
	.deactivate	= vib_ctrl_gpio_deactivate,
};

static struct vib_ctrl_ops vib_ctrl_pwm_ops = {
	.init		= vib_ctrl_pwm_init,
	.configure	= vib_ctrl_pwm_config,
	.activate	= vib_ctrl_pwm_activate,
	.deactivate	= vib_ctrl_pwm_deactivate,
};

static int vib_signal_config(struct vib_signal *vibs, unsigned int total_us,
		unsigned int period_us, unsigned int duty_us)
{
	int ret = 0;
	if (vibs->of.type) {
		if (duty_us == period_us) {
			period_us = total_us;
			duty_us = total_us;
		}
		dvib_print("\t\t%s: %u %u %u\n", vibs->name, total_us,
				period_us, duty_us);
		ret = vibs->ops->configure(vibs, total_us,
					period_us, duty_us);
	}
	return ret;
}

static int vib_signal_activate(struct vib_signal *vibs)
{
	int ret = 0;
	if (vibs->of.type)
		ret = vibs->ops->activate(vibs);
	return ret;
}

static int vib_signal_deactivate(struct vib_signal *vibs)
{
	int ret = 0;
	if (vibs->of.type)
		ret = vibs->ops->deactivate(vibs);
	return ret;
}

static int vibrator_regulator_init(struct vibrator *vib)
{
	struct regulator *reg;
	if (!vib->regulator_name[0])
		return 0;

	reg = regulator_get(NULL, vib->regulator_name);
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	vib->regulator = reg;
	return 0;
}

static int vibrator_regulator_enable(struct vibrator *vib)
{
	if (vib->regulator)
		return regulator_enable(vib->regulator);
	return 0;
}

static int vibrator_regulator_disable(struct vibrator *vib)
{
	if (vib->regulator)
		return regulator_disable(vib->regulator);
	return 0;
}

/* Select the shortest pwm with the time member greater or equal to request.
 * If not available, pick up the pwm with the biggest time member.
 */
static struct vib_pwm *vib_select_pwm(struct vibrator *vib, int values_ms)
{
	struct vib_pwm *pwm, *tmp, *l = NULL, *ge = NULL;
	int i;
	int us = USEC_PER_MSEC * values_ms;
	for (i = 0; i < MAX_PWMS; i++) {
		tmp = &vib->ctrl.vib_pwm[i];
		if (tmp->time) {
			if (tmp->time < us) {
				if (!l || l->time < tmp->time)
					l = tmp;
			} else {
				if (!ge || ge->time >= tmp->time)
					ge = tmp;
			}
		} else {
			break;
		}
	}
	pwm = ge ? ge : l;
	if (pwm)
		dvib_print("\t\ts\t%u %u %u %u\n", pwm->time,
			pwm->period, pwm->duty_en, pwm->duty_dir);
	return pwm;
}

static void vibrator_dump(void *data)
{
	struct vib_timed *vib_timed = (struct vib_timed *)data;
	struct vibrator *vib = vib_timed->dev_data;
	int i;
	zprintk("---------\n");
	zprintk("%s type 0x%x %s\n", vib_timed->name,
		vib->type, vib->regulator_name);
	zprintk("%u ~ %u us\n", vib->min_us, vib->max_us);
	vib_signal_print(&vib->ctrl.vib_en);
	vib_signal_print(&vib->ctrl.vib_dir);
	for (i = 0; i < MAX_PWMS; i++)
		vib_print_pwm(&vib->ctrl.vib_pwm[i]);
	zprintk("---------\n");
}

static int vibrator_power_on(int value_ms, void *data)
{
	struct vib_timed *vib_timed = (struct vib_timed *)data;
	struct vibrator *vib = vib_timed->dev_data;
	struct vib_pwm *pwm;
	unsigned long total_us = value_ms * USEC_PER_MSEC;
	int ret;

	ret = vibrator_regulator_enable(vib);
	if (ret) {
		zfprintk("regulator enable %s failed\n",
			vib->regulator_name);
		return ret;
	}
	wake_lock(&vib->wakelock);

	pwm = vib_select_pwm(vib, value_ms);
	if (!pwm)
		return 0;
	if (total_us < pwm->period)
		total_us = pwm->period;
	vib_signal_config(&vib->ctrl.vib_dir, total_us,
				pwm->period, pwm->duty_dir);
	vib_signal_config(&vib->ctrl.vib_en, total_us,
				pwm->period, pwm->duty_en);
	vib_signal_activate(&vib->ctrl.vib_dir);
	vib_signal_activate(&vib->ctrl.vib_en);
	do_div(total_us, USEC_PER_MSEC);
	value_ms = total_us;

	return value_ms;
}

static int vibrator_power_off(void *data)
{
	struct vib_timed *vib_timed = (struct vib_timed *)data;
	struct vibrator *vib = vib_timed->dev_data;
	vib_signal_deactivate(&vib->ctrl.vib_en);
	vib_signal_deactivate(&vib->ctrl.vib_dir);
	vibrator_regulator_disable(vib);
	wake_unlock(&vib->wakelock);
	return 0;
}

static int vib_signal_init(struct vib_signal *vibs)
{
	struct vib_of_signal *of = &vibs->of;
	int ret;

	switch (of->type) {
	case SIGNAL_GPIO:
		ret = vib_ctrl_gpio_init(vibs);
		vibs->ops = &vib_ctrl_gpio_ops;
		break;
	case SIGNAL_PWM:
		ret = vib_ctrl_pwm_init(vibs);
		vibs->ops = &vib_ctrl_pwm_ops;
		break;
	default:
		zfprintk("%s unknown signal type %d\n", vibs->name, of->type);
		of->type = 0;
		ret = -1;
	}
	return ret;
}

#ifdef CONFIG_ARM_OF
static int vib_of_init(struct vibrator *vib, int vib_nr)
{
	struct device_node *node;
	const void *prop = NULL;
	int len = 0;
	char dt_path_vib[sizeof(DT_PATH_VIB) + 3];

	snprintf(dt_path_vib, sizeof(DT_PATH_VIB) + 2, "%s%1d",
		DT_PATH_VIB, vib_nr % MAX_VIBS);
	node = of_find_node_by_path(dt_path_vib);
	if (!node)
		return -ENODEV;

	prop = of_get_property(node, DT_PROP_VIB_TYPE, &len);
	if (prop && len)
		vib->type = *((int *)prop);
	else
		return -ENODEV;

	if ((vib->type != VIB_TYPE_GENENIC_ROTARY)
		&& (vib->type != VIB_TYPE_GENENIC_LINEAR))
		return -ENODEV;

	prop = of_get_property(node, VIB_EN, &len);
	if (prop && len) {
		vib->ctrl.vib_en.of = *((struct vib_of_signal *)prop);
		vib->ctrl.vib_en.name = VIB_EN;
		vib->ctrl.vib_en.signal_type = SIGNAL_ENABLE;
		if (vib_signal_init(&vib->ctrl.vib_en)) {
			zfprintk("vib_en init failed\n");
			return -ENODEV;
		}
	} else {
		zfprintk("vib_en not found in %s\n", dt_path_vib);
		return -ENODEV;
	}

	prop = of_get_property(node, "pwm", &len);
	if (prop && len) {
		int i, j = len / sizeof(struct vib_pwm);
		dprintk("pwm len %d size %d\n", len,
				len/sizeof(struct vib_pwm));
		if (j > MAX_PWMS)
			j = MAX_PWMS;
		for (i = 0; i < j; i++)
			vib->ctrl.vib_pwm[i] = *(((struct vib_pwm *)prop) + i);
	} else {
		zfprintk("pwm not found in %s\n", dt_path_vib);
		return -ENODEV;
	}

	prop = of_get_property(node, VIB_DIR, &len);
	if (prop && len) {
		vib->ctrl.vib_dir.of = *((struct vib_of_signal *)prop);
		vib->ctrl.vib_dir.name = VIB_DIR;
		vib->ctrl.vib_dir.signal_type = SIGNAL_DIRECTION;
		if (vib_signal_init(&vib->ctrl.vib_dir)) {
			zfprintk("vib_dir init failed\n");
			return -ENODEV;
		}
	} else {
		if (vib->type == VIB_TYPE_GENENIC_LINEAR) {
			zfprintk("vib_dir not found in %s\n", dt_path_vib);
			return -ENODEV;
		}
	}

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(vib->regulator_name, (char *)prop,
				REGULATOR_NAME_SIZE - 1);
		vib->regulator_name[REGULATOR_NAME_SIZE - 1] = '\0';
		if (vibrator_regulator_init(vib)) {
			zfprintk("regulator init %s failed\n",
				vib->regulator_name);
			return -ENODEV;
		}
	}

	prop = of_get_property(node, "min", &len);
	if (prop && len)
		vib->min_us = *((unsigned int *)prop);
	else
		vib->min_us = MIN_TIMEOUT;

	prop = of_get_property(node, "max", &len);
	if (prop && len)
		vib->max_us = *((unsigned int *)prop);
	else
		vib->max_us = MAX_TIMEOUT;

	of_node_put(node);
	return 0;
}
#else
static inline int vib_of_init(struct vibrator *, int) { return 0; }
#endif /* CONFIG_ARM_OF */

struct vib_timed_platform_data vib_timed_pdata = {
	.vibs = vib_timeds,
};

static struct platform_device vib_timed_dev = {
	.name = VIB_TIMED_NAME,
	.dev = {
		.platform_data = &vib_timed_pdata,
	},
};

void __init mapphone_vibrator_init(void)
{
	int i, count;
	struct vibrator *vib;
	struct vib_timed *vib_timed;
	for (i = 0, count = 0; i < MAX_VIBS; i++) {
		vib = &vibrators[count];
		vib_timed = &vib_timeds[count];

		if (vib_of_init(vib, i))
			continue;

		vib_timed->dev_data = vib;
		vib_timed->min_timeout = vib->min_us / USEC_PER_MSEC;
		vib_timed->max_timeout = vib->max_us / USEC_PER_MSEC;
		vib_timed->power_on = vibrator_power_on;
		vib_timed->power_off = vibrator_power_off;
		vib_timed->dump = vibrator_dump;
		vib_timed->name = vib_name[count];
		wake_lock_init(&vib->wakelock, WAKE_LOCK_SUSPEND,
				vib_timed->name);
		count++;
		dprintk("%s type 0x%x min %d max %d\n",
			vib_timed->name, vib->type,
			vib_timed->min_timeout, vib_timed->max_timeout);
	}
	if (count) {
		vib_timed_pdata.count = count;
		platform_device_register(&vib_timed_dev);
	}
}
