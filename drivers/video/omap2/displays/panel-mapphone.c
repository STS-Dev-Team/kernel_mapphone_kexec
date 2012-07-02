#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <plat/display.h>
#include <plat/dma.h>
#include <asm/atomic.h>
#include <plat/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include <asm/prom.h>

#include <mach/dt_path.h>

#include <plat/panel.h>
#include <plat/mapphones_dsi_panel.h>
#include "../dss/dss.h"

#ifndef CONFIG_ARM_OF
#error CONFIG_ARM_OF must be defined for Mapphone to compile
#endif
#ifndef CONFIG_USER_PANEL_DRIVER
#error CONFIG_USER_PANEL_DRIVER must be defined for Mapphone to compile
#endif

/*#define DEBUG 1*/

#ifdef DEBUG
static unsigned int panel_debug;
#define DBG(format, ...) \
do { \
	if (panel_debug) \
		printk(KERN_DEBUG "mapphone-panel: " format, ## __VA_ARGS__); \
} while (0)

#else
#define DBG(format, ...)
#endif

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_GET_POWER_MODE       0x0A
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B
#define EDISCO_CMD_SET_TEAR_OFF		0x34
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44
#define EDISCO_CMD_SET_CABC		0x55
#define EDISCO_CMD_READ_CABC		0x56
#define EDISCO_CMD_READ_DDB_START	0xA1
#define EDISCO_CMD_SET_MCS		0xB2
#define EDISCO_CMD_SET_DISPLAY_MODE     0xB3
#define EDISCO_CMD_SET_BCKLGHT_PWM	0xB4
#define EDISCO_CMD_DATA_LANE_CONFIG	0xB5
#define EDISCO_CMD_READ_DA		0xDA
#define EDISCO_CMD_READ_DB		0xDB
#define EDISCO_CMD_READ_DC		0xDC
#define EDISCO_CMD_RDDSDR		0x0F

#define EDISCO_CMD_DATA_LANE_ONE	0x0
#define EDISCO_CMD_DATA_LANE_TWO	0x01
#define EDISCO_CMD_SLEEP_MODE_OUT	0x10

#define EDISCO_CMD_MCS_ON		0x3
#define EDISCO_CMD_MCS_OFF		0x0

#define EDISCO_LONG_WRITE		0x29
#define EDISCO_SHORT_WRITE_1		0x23
#define EDISCO_SHORT_WRITE_0		0x13

#define EDISCO_CMD_VC   0
#define EDISCO_VIDEO_VC 1

#define DCS_CMD_RETRY_MAX 10

#define PANEL_OFF     0x0
#define PANEL_ON      0x1

#define CABC_MIN_VAL	0x01
#define CABC_MAX_VAL	0x04

/* Todo:  should NOT show vendor name, replace it with panel id later */
/* DDB Controller Supplier IDs used for run_test(1) */
#define CTL_SUPPLIER_ID_LEN	2
#define CTL_SUPPLIER_ID_AUO 	0x0186
#define CTL_SUPPLIER_ID_AUO_43	0x0126	/* Shadow AUO panel reports 0x126 */
#define CTL_SUPPLIER_ID_TMD 	0x0126
#define CTL_SUPPLIER_ID_SMD 	0x010B

#define INVALID_VALUE	0xFFFF

/*
 *This must match with schema.xml section "device-id-value"
 *Name convention:
 *MOT_DISP_PROTOCOL_MODE_SIZE_RESOLUTION{_PANEL_MANUFACTURE_ORDER}
 *PROTOCOL: MIPI
 *MODE: CM for cmd mode,VM for video mode
 *SIZE: panel physical size
 *RESOLUTION: width * height in pixel
 *PANEL_MANUFACTURE_ORDER: for multiple panel manufacture only
 *value are 1,2,3...starting from default value 1
 */
#define MOT_DISP_MIPI_CM_480_854		0x000a0001
#define MOT_DISP_MIPI_CM_430_480_854		0x001a0000
#define MOT_DISP_MIPI_CM_370_480_854		0x001a0001
#define MOT_DISP_MIPI_CM_430_540_960_3		0x001a0002
#define MOT_DISP_MIPI_CM_400_540_960		0x00090004
#define MOT_DISP_MIPI_CM_430_540_960		0x00090005
#define MOT_DISP_MIPI_CM_430_540_960_AMOLED	0x00090006
#define MOT_DISP_MIPI_VM_248_320_240		0x00090002
#define MOT_DISP_MIPI_VM_280_320_240		0x00090003
#define MOT_DISP_MIPI_CM_310_320_480_1		0x001f0000
#define MOT_DISP_MIPI_CM_310_320_480_2		0x000a0003

/*ESD spec require 10ms, select 8ms */
#define MAPPHONE_ESD_CHECK_PERIOD   msecs_to_jiffies(8000)

struct panel_regulator {
	struct regulator *regulator;
	const char *name;
};

static int column_address_offset;
#define MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX 255

static void free_regulators(struct panel_regulator *regulators, int n)
{
}


static int init_regulators(struct omap_dss_device *dssdev,
				struct panel_regulator *regulators, int n)
{
	return 0;
}

/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 * @regulators: array of panel regulators
 * @num_regulators: number of regulators in the array
 */
struct panel_config {
	const char *name;
	int type;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	struct panel_regulator *regulators;
	int num_regulators;
};

enum {
	PANEL_MAPPHONE,
};

static struct panel_config panel_configs[] = {
	{
		.name           = "mapphone",
		.type           = PANEL_MAPPHONE,
		.sleep          = {
			.sleep_in       = 5,
			.sleep_out      = 5,
			.hw_reset       = 5,
			.enable_te      = 100, /* possible panel bug */
		},
		.reset_sequence = {
			.high           = 10,
			.low            = 10,
		},
	},
};

struct mapphone_data {
	struct mutex lock;

	struct backlight_device *bldev;

	struct omap_dss_device *dssdev;

	bool enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;
	struct delayed_work te_timeout_work;
	struct workqueue_struct *te_wq;
	struct work_struct te_framedone_work;

	bool use_dsi_bl;

	bool cabc_broken;
	unsigned cabc_mode;

	bool intro_printed;
	bool force_update;
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;

	struct panel_config *panel_config;
};

static inline struct mapphone_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct mapphone_dsi_panel_data *) dssdev->data;
}

static void mapphone_panel_power_off(struct omap_dss_device *dssdev);
static int mapphone_panel_power_on(struct omap_dss_device *dssdev);

static void mapphone_esd_work(struct work_struct *work)
{

	struct mapphone_data *mp_data = container_of(work, struct mapphone_data,
			esd_work.work);
	struct omap_dss_device *dssdev = mp_data->dssdev;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	u8 power_mode;
	u8 expected_mode;
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&mp_data->lock);

	if (!mp_data->enabled) {
		mutex_unlock(&mp_data->lock);
		return;
	}

	dsi_bus_lock(ix);
	dss_mainclk_enable();

	r = dsi_vc_dcs_read(ix, EDISCO_CMD_VC, EDISCO_CMD_GET_POWER_MODE,
			&power_mode, 1);
	if (r != 1) {
		dev_err(&dssdev->dev, "Failed to get power mode, r = %d\n", r);
		goto err;
	}

	if (atomic_read(&panel_data->state) == PANEL_ON)
		expected_mode = 0x9c;
	else
		expected_mode = 0x98;

	DBG("ESD Check - read mode = 0x%02x, expected = 0x%02x\n", power_mode,
		expected_mode);

	if (power_mode != expected_mode) {
		dev_err(&dssdev->dev,
			"Power mode in incorrect state, "
			"mode = 0x%02x, expected = 0x%02x\n",
			power_mode, expected_mode);
		goto err;
	}

	dss_mainclk_disable();
	dsi_bus_unlock(ix);

	queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
			 MAPPHONE_ESD_CHECK_PERIOD);

	mutex_unlock(&mp_data->lock);
	return;
err:
	dev_err(&dssdev->dev, "ESD: performing LCD reset\n");
	printk(KERN_INFO"ESD: mapphone_panel_power_off.\n");
	mapphone_panel_power_off(dssdev);
	printk(KERN_INFO"ESD: mdelay 20ms.\n");
	mdelay(20);
	printk(KERN_INFO"ESD: mapphone_panel_power_on.\n");
	mapphone_panel_power_on(dssdev);

	dss_mainclk_disable();
	dsi_bus_unlock(ix);

	queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
			 MAPPHONE_ESD_CHECK_PERIOD);

	mutex_unlock(&mp_data->lock);
}

static void hw_guard_start(struct mapphone_data *mp_data, int guard_msec)
{
}

static void hw_guard_wait(struct mapphone_data *mp_data)
{
}

static int mapphone_panel_display_on(struct omap_dss_device *dssdev);

static void mapphone_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	enum omap_dsi_index ix;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	/* Turn on display when framedone */
	mapphone_panel_display_on(dssdev);

	/* queue_work(mp_data->te_wq, &mp_data->te_framedone_work); */
}

static int mapphone_set_update_window(enum omap_dsi_index ix,
					u16 x, u16 y, u16 w, u16 h)
{
	u8 buf[5];
	int ret;

	u16 x1 = x + column_address_offset;
	u16 x2 = x + column_address_offset + w - 1;
	u16 y1 = y;
	u16 y2 = y + h - 1;

	/*
	 * set page, column address cmd using dsi_vc_dcs_write()
	 * with BTA sync. without BTA sync, DSS_L3_ICLK is not gated
	 * when framedone CM_DSS_CLKSTCTRL = 0x00000f03, which result
	 * in DSS does not go into idle CM_DSS_DSS_CLKCTRL= 0x00050702
	 * work around is: send cmd with BTA sync. with it
	 * when framedone:CM_DSS_CLKSTCTRL = 0x00000e03
	 * CM_DSS_DSS_CLKCTRL= 0x00060702
	 */
	buf[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;
	ret = dsi_vc_dcs_write_nosync(ix, EDISCO_CMD_VC, buf, 5);
	if (ret)
		goto err;

	buf[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;
	ret = dsi_vc_dcs_write(ix, EDISCO_CMD_VC, buf, 5);
	if (ret)
		goto err;

	return 0;
err:
	return ret;

}

static int mapphone_panel_update(struct omap_dss_device *dssdev,
					u16 x, u16 y, u16 w, u16 h)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&mp_data->lock);
	dsi_bus_lock(ix);
	dss_mainclk_enable();

	if (!mp_data->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	r = mapphone_set_update_window(ix, x, y, w, h);
	if (r)
		goto err;

	if (mp_data->te_enabled && panel_data->use_ext_te) {
		mp_data->update_region.x = x;
		mp_data->update_region.y = y;
		mp_data->update_region.w = w;
		mp_data->update_region.h = h;
		barrier();
		schedule_delayed_work(&mp_data->te_timeout_work,
					msecs_to_jiffies(250));
		atomic_set(&mp_data->do_update, 1);
	} else {
		/* We use VC(1) for VideoPort Data and VC(0) for L4 data */
		if (cpu_is_omap44xx())
			r = omap_dsi_update(dssdev, EDISCO_VIDEO_VC, x, y, w, h,
						mapphone_framedone_cb, dssdev);
		else
			r = omap_dsi_update(dssdev, EDISCO_CMD_VC, x, y, w, h,
						mapphone_framedone_cb, dssdev);
		if (r)
			goto err;
	}

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&mp_data->lock);

	return 0;
err:
	dss_mainclk_disable();
	dsi_bus_unlock(ix);
	mutex_unlock(&mp_data->lock);
	return r;
}
static int mapphone_panel_sync(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("sync\n");

	mutex_lock(&mp_data->lock);
	dsi_bus_lock(ix);
	dsi_bus_unlock(ix);
	mutex_unlock(&mp_data->lock);

	DBG("sync done\n");
	return 0;
}

static void mapphone_panel_get_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static u8 mapphone_panel_get_rotate(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mp_data->lock);
	r = mp_data->rotate;
	mutex_unlock(&mp_data->lock);

	return r;
}

static void mapphone_panel_get_resolution(struct omap_dss_device *dssdev,
						u16 *xres, u16 *yres)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	if (mp_data->rotate == 0 || mp_data->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static void mapphone_panel_get_dimension(struct omap_dss_device *dssdev,
						u32 *width, u32 *height)
{
	*width = dssdev->panel.timings.w;
	*height = dssdev->panel.timings.h;
}

static void mapphone_panel_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	/* ignore */
}

static int mapphone_panel_get_te(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mp_data->lock);
	r = mp_data->te_enabled;
	mutex_unlock(&mp_data->lock);

	return r;
}

static void te_work_callback(struct work_struct *work)
{
	struct mapphone_data *mp_data = container_of(work, struct mapphone_data,
						te_framedone_work);
	struct omap_dss_device *dssdev = mp_data->dssdev;
	u16 x_res = dssdev->panel.timings.x_res;
	u16 y_res = dssdev->panel.timings.y_res;

	mapphone_panel_update(dssdev, 0, 0, x_res, y_res);
}

static int mapphone_panel_memory_read(struct omap_dss_device *dssdev,
						void *buf, size_t size,
						u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (size < w * h * 3)
		return -ENOMEM;

	mutex_lock(&mp_data->lock);

	if (!mp_data->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * 3,
	dssdev->panel.timings.x_res *
	dssdev->panel.timings.y_res * 3);

	dsi_bus_lock(ix);

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	* use short packets. plen 32 works, but bigger packets seem to cause
	* an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	mapphone_set_update_window(ix, x, y, w, h);

	r = dsi_vc_set_max_rx_packet_size(ix, EDISCO_CMD_VC, plen);
	if (r)
		goto err2;

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = dsi_vc_dcs_read(ix, EDISCO_CMD_VC, dcs_cmd,
		buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(&dssdev->dev, "read error\n");
			goto err3;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&dssdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
						"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}

	r = buf_used;

err3:
	dsi_vc_set_max_rx_packet_size(ix, EDISCO_CMD_VC, 1);
err2:
	dsi_bus_unlock(ix);
err1:
	mutex_unlock(&mp_data->lock);
	return r;
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev);

static int dsi_mipi_vm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	int ret;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dsi_disable_vid_vc_enable_cmd_vc(lcd_ix);
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, &data, 1);
	dsi_disable_cmd_vc_enable_vid_vc(lcd_ix);

	return ret;
}

static int dsi_mipi_cm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	/*
	 *Issue:
	 *See corrupted image on the end of last line of display panel
	 *sometimes when resume the phone. It happens more often
	 *when OMAP4 run at high freq such as 600M/800M/1G, so far locking
	 *arm at 300M does not see the issue.
	 *
	 *It is observed on omap4-based products built with SMD pentile panel
	 *controller. The corruption happens within the controller's
	 *framebuffer after receive the first update and 29h/disp on cmd,
	 *the interval between EOT and 29h is about 16us, it probably isn't
	 *long enough for Pentile processing to get the data into memory.
	 *The following update have no problem.
	 *
	 *Adding delay before sending 29h/disp on cmd can make the issue
	 *go away based on test, it is test value*2.
	 */
	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_400_540_960:
	case MOT_DISP_MIPI_CM_430_540_960:
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		udelay(24);
		break;
	}
	/* Called in interrupt context, send cmd without sync */
	return dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, &data, 1);
}


static int mapphone_panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (atomic_cmpxchg(&panel_data->state, PANEL_OFF, PANEL_ON) ==
						PANEL_OFF) {
		switch (dssdev->panel.panel_id) {
		case MOT_DISP_MIPI_VM_248_320_240:
		case MOT_DISP_MIPI_VM_280_320_240:
			ret = dsi_mipi_vm_panel_on(dssdev);
			break;
		case MOT_DISP_MIPI_CM_480_854:
		case MOT_DISP_MIPI_CM_370_480_854:
		case MOT_DISP_MIPI_CM_430_480_854:
		case MOT_DISP_MIPI_CM_400_540_960:
		case MOT_DISP_MIPI_CM_430_540_960:
		case MOT_DISP_MIPI_CM_430_540_960_3:
		case MOT_DISP_MIPI_CM_310_320_480_1:
		case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
			ret = dsi_mipi_cm_panel_on(dssdev);
			break;
		default:
			printk(KERN_ERR "unsupport panel =0x%lx \n",
			dssdev->panel.panel_id);
			ret = -EINVAL;
		}

		if (ret == 0)
			printk(KERN_INFO "Panel is turned on \n");
	}

	return ret;
}

static u16 read_panel_manufacture(struct omap_dss_device *dssdev)
{
	enum omap_dsi_index lcd_ix;
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
			EDISCO_CMD_READ_DA, &data, 1) != 1)
		printk(KERN_ERR "Mapphone panel: failed to read panel manufacturer ID\n");
	else {
		id = data >> 6;
		printk(KERN_INFO "Mapphone panel: panel manufacturer ID (DAh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_panel_controller_version(struct omap_dss_device *dssdev)
{
	enum omap_dsi_index lcd_ix;
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
			EDISCO_CMD_READ_DB, &data, 1) != 1)
		printk(KERN_ERR "Mapphone panel: failed to read controller version \n");
	else {
		id = data;
		printk(KERN_INFO "Mapphone panel: controller version (DBh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_panel_controller_driver_version(struct omap_dss_device *dssdev)
{
	enum omap_dsi_index lcd_ix;
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
			EDISCO_CMD_READ_DC, &data, 1) != 1)
		printk(KERN_ERR "Mapphone panel: failed to read controller driver ID\n");
	else {
		id = data;
		printk(KERN_INFO "Mapphone panel: controller driver ID(DCh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_supplier_id(struct omap_dss_device *dssdev)
{
	static u16 id = INVALID_VALUE;
	u8 data[2];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC, 2))
		goto end;

	if (dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
			    EDISCO_CMD_READ_DDB_START, data, 2) == 2) {
		id = (data[0] << 8) | data[1];
		printk(KERN_INFO "Mapphone panel: controller supplier id(A1h)=0x%x\n",
			 id);
	} else
		printk(KERN_ERR "Mapphone panel: failed to read controller supplier ID\n");

	dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC, 1);
end:
	return id;
}

static ssize_t mapphone_panel_supplier_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	static u16 supplier_id = INVALID_VALUE;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (supplier_id == INVALID_VALUE) {

		if (!mp_data->enabled)
			goto end;

		dsi_bus_lock(ix);
		dss_mainclk_enable();

		switch (dssdev->panel.panel_id) {
		case MOT_DISP_MIPI_CM_480_854:
		case MOT_DISP_MIPI_CM_370_480_854:
			supplier_id = read_supplier_id(dssdev);
			break;
		case MOT_DISP_MIPI_CM_400_540_960:
		case MOT_DISP_MIPI_CM_430_540_960:
		case MOT_DISP_MIPI_CM_430_540_960_3:
		case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
			supplier_id = read_panel_manufacture(dssdev);
			break;
		default:
			printk(KERN_ERR "Do not support supplier_id of the panel\n");
		}

		dss_mainclk_disable();
		dsi_bus_unlock(ix);
	}
end:
	return sprintf(buf, "%d\n", supplier_id);
}

static DEVICE_ATTR(supplier_id, S_IRUGO,
			mapphone_panel_supplier_id_show, NULL);

static bool mapphone_cabc_support(struct omap_dss_device *dssdev)
{
	bool ret = true;

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		ret = false;
		break;
	default:
		ret = true;
	}

	return ret;
}

static ssize_t panel_cabc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	u8 data = 0xff;
	enum omap_dsi_index lcd_ix;

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk(KERN_ERR " mapphone panel is NOT active \n");
		data = -EAGAIN;
		goto err;
	}

	if (mapphone_cabc_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support this op \n");
		data = -EPERM;
		goto err;
	}

	dss_mainclk_enable();
	dsi_bus_lock(lcd_ix);

	if ((dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
				EDISCO_CMD_READ_CABC, &data, 1) == 1) &&
			((data == CABC_MIN_VAL) || (data == CABC_MAX_VAL))) {
		printk(KERN_INFO "Mapphone panel: CABC_mode=0x%x\n", data);
	} else
		printk(KERN_ERR "Mapphone panel: fail to read CABC. "
				"data=0x%x\n", data);

	dsi_bus_unlock(lcd_ix);
	dss_mainclk_disable();

err:
	return sprintf(buf, "%d\n", data);
}


static int mapphone_panel_lp_cmd_wrt_sync(struct omap_dss_device *dssdev,
					bool dcs_cmd, int write_dt,
					u8 *write_data, int write_len,
					int read_cmd, int read_len,
					int chk_val, int chk_mask);

static ssize_t panel_cabc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	unsigned long cabc_val, r;
	enum omap_dsi_index lcd_ix;
	u8 data[2];

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk(KERN_ERR " mapphone panel is NOT active \n");
		r = -EAGAIN;
		goto err;
	}

	if (mapphone_cabc_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support this op \n");
		r = -EPERM;
		goto err;
	}

	r = strict_strtoul(buf, 0, &cabc_val);
	if ((r) || ((cabc_val != CABC_MIN_VAL) && (cabc_val != CABC_MAX_VAL))) {
		printk(KERN_ERR "Invalid CABC=%lu \n", cabc_val);
		r = -EINVAL;
		goto err;
	}

	dss_mainclk_enable();
	dsi_bus_lock(lcd_ix);

	data[0] = EDISCO_CMD_SET_CABC;
	data[1] = (u8)cabc_val;
	r = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_READ_CABC, 1,
					(u8)cabc_val, 0x7);
	if (r) {
		printk(KERN_ERR "failed to set EDISCO_CMD_SET_CABC \n");
		r = -EIO;
	}

	dsi_bus_unlock(lcd_ix);
	dss_mainclk_disable();
err:
	return r ? r : count;
}

static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWGRP,
				panel_cabc_show, panel_cabc_store);

static void mapphone_hw_reset(struct omap_dss_device *dssdev)
{
}

static void mapphone_panel_print_config(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "DT: x_res=%d y_res=%d phy_w_mm=%d phy_h_mm=%d\n",
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res,
		dssdev->panel.timings.w, dssdev->panel.timings.h);

	printk(KERN_INFO "DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
		dssdev->panel.timings.hfp, dssdev->panel.timings.hsw,
		dssdev->panel.timings.hbp, dssdev->panel.timings.vfp,
		dssdev->panel.timings.vsw, dssdev->panel.timings.vbp);

	printk(KERN_INFO "DT: clk_lane=%d clk_pos=%d d1_lane=%d d1_pos=%d\n",
		dssdev->phy.dsi.clk_lane, dssdev->phy.dsi.clk_pol,
		dssdev->phy.dsi.data1_lane, dssdev->phy.dsi.data1_pol);

	printk(KERN_INFO "DT: d2_lane= %d d2_pos= %d xfer_mode= %d\n",
		dssdev->phy.dsi.data2_lane, dssdev->phy.dsi.data2_pol,
		dssdev->phy.dsi.xfer_mode);

	printk(KERN_INFO "DT: panel_id=0x%lx\n", dssdev->panel.panel_id);

	printk(KERN_INFO "DT: regn=%d regm=%d regm3=%d regm4=%d"
			" lp_clk_div=%d lck_div=%d pck_div=%d\n",
		dssdev->phy.dsi.div.regn, dssdev->phy.dsi.div.regm,
		dssdev->phy.dsi.div.regm_dispc, dssdev->phy.dsi.div.regm_dsi,
		dssdev->phy.dsi.div.lp_clk_div, dssdev->phy.dsi.div.lck_div,
		dssdev->phy.dsi.div.pck_div);
}

static int dsi_mipi_cm_430_540_960_amoled_bl_probe(
	struct omap_dss_device *dssdev,	struct mapphone_data *mp_data);

static int mapphone_panel_probe(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;
	int r, i;

	DBG("probe\n");

	if (!panel_data || !panel_data->name) {
		r = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		r = -EINVAL;
		goto err;
	}

	mp_data = kmalloc(sizeof(struct mapphone_data), GFP_KERNEL);
	if (!mp_data) {
		r = -ENOMEM;
		goto err;
	}

	memset(mp_data, 0, sizeof(struct mapphone_data));

	dssdev->panel.config = OMAP_DSS_LCD_TFT;

	mp_data->dssdev = dssdev;
	mp_data->panel_config = panel_config;

	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_540_960_AMOLED) {
		r = dsi_mipi_cm_430_540_960_amoled_bl_probe(dssdev, mp_data);
		if (r)
			goto err_reg;
	}

	mutex_init(&mp_data->lock);

	atomic_set(&mp_data->do_update, 0);

	r = init_regulators(dssdev, panel_config->regulators,
				panel_config->num_regulators);
	if (r)
		goto err_reg;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err_wq;
	}

	mp_data->esd_wq = create_singlethread_workqueue("mapphone_esd");
	if (mp_data->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&mp_data->esd_work, mapphone_esd_work);

	dev_set_drvdata(&dssdev->dev, mp_data);

	if (cpu_is_omap44xx())
		mp_data->force_update = true;

	if (panel_data->use_ext_te || mp_data->force_update) {
		if (mp_data->force_update) {
			if (dssdev->channel == OMAP_DSS_CHANNEL_LCD) {
				mp_data->te_wq =
			create_singlethread_workqueue("mapphone_panel wq");
			INIT_WORK(&mp_data->te_framedone_work,
						te_work_callback);
			}
		}
	}

	r = device_create_file(&dssdev->dev, &dev_attr_supplier_id);
	if (r < 0) {
		dev_err(&dssdev->dev, "controller supplier_id create"
			" failed:%d\n", r);
		goto err_wq;
	}

	r = device_create_file(&dssdev->dev, &dev_attr_cabc_mode);
	if (r < 0) {
		dev_err(&dssdev->dev, "Display CABC create"
			" failed:%d\n", r);
		goto removeattr;
	}

	return 0;

removeattr:
	device_remove_file(&dssdev->dev, &dev_attr_supplier_id);
err_wq:
	free_regulators(panel_config->regulators, panel_config->num_regulators);
err_reg:
	kfree(mp_data);
err:
	return r;
}

static void mapphone_panel_remove(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	DBG("remove\n");

	device_remove_file(&dssdev->dev, &dev_attr_supplier_id);
	device_remove_file(&dssdev->dev, &dev_attr_cabc_mode);

	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	cancel_delayed_work(&mp_data->esd_work);
	destroy_workqueue(mp_data->esd_wq);
	if (mp_data->force_update)
		destroy_workqueue(mp_data->te_wq);
	/* reset, to be sure that the panel is in a valid state */
	mapphone_hw_reset(dssdev);

	free_regulators(mp_data->panel_config->regulators,
	mp_data->panel_config->num_regulators);

	kfree(mp_data);

	return;
}

/* - In the LP mode, some panels have problems to receive command correctly
 * so we will send command out and read it back to make sure the write
 * command is accepted
 * - if the dsi_vc_dcs_write() request, then we will not care about the
 * write_dt (data type) */
static int mapphone_panel_lp_cmd_wrt_sync(struct omap_dss_device *dssdev,
					bool dcs_cmd, int write_dt,
					u8 *write_data, int write_len,
					int read_cmd, int read_len,
					int chk_val, int chk_mask)
{
	int i, ret;
	u8 data[7];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;


	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		if (dcs_cmd == true) {
			ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC,
						write_data, write_len);
			DBG("call dsi_vc_dcs_write"
				"(len=0%d, p1/p2/p3/p4=0x%x/0x%x/0x%x/0x%x)\n",
				write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		} else {
			ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC, write_dt,
						write_data, write_len);
			DBG("call dsi_vc_write"
				"(dt=0x%x len=%d, p1/p2/p3/p4 = "
				"0x%x/0x%x/0x%x/0x%x)\n",
				write_dt, write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		}

		if (ret) {
			printk(KERN_ERR "failed to send cmd=0x%x \n",
							 write_data[0]);
			continue;
		}

		mdelay(1);

		/* TODO. Do not know how to handle and to check if more than
		 * 1 byte to read is requested*/
		if (read_len < 0 || read_len > 1) {
			printk(KERN_ERR "Invalid read_len=%d\n", read_len);
			return -1;
		}

		/* Read the data back to make sure write_command is working */
		data[0] = 0;
		ret = dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC, read_cmd,
						&data[0], read_len);

		DBG("read_chk_cmd dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x "
				"read_val=0x%x \n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);

		if (ret < 0)
			DBG("fail to read 0x%x cmd and "
					"will try it again \n", read_cmd);

		if ((data[0] & chk_mask) == chk_val) {
			/* break if read back the same writing value*/
			ret  = 0;
			break;
		}
	}

	if (i >= DCS_CMD_RETRY_MAX) {
		printk(KERN_ERR "failed to read dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x\n"
				"read_val=0x%x \n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);
		ret = -1;
	}

	return ret;
}

/***************************************************************************
 * Start of panel enable API
 ****************************************************************************/
/*
 * pls follow panel enable API name convention when add new panels
 * for OMAP4-based products.
 *
 * API name is composed of:
 * PROTOCOL: 	dsi mipi
 * MODE:	cm for cmd mode, vm for video mode
 * SIZE: 	physical dimension(4''/4.3'/3.7''')
 * RESOLUTION: width * height in pixel
 * MANUFACTURE ID: identify panel manufacture staring from 1
 * CONTROLLER VERSION:	for mulitple version such as ES1,ES2,ES5
 * dsi_mipi_mode_size_resolution{_manufactureid_version}_panel_enable
 */
static int dsi_mipi_248_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG(" dsi_mipi_248_vm_320_240_panel_enable() \n");

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_280_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[10];
	int ret;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG(" dsi_mipi_280_vm_320_240_panel_enable() \n");

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC,
				EDISCO_SHORT_WRITE_1, data, 2);

	/* Internal display set up */
	data[0] = 0xC0;
	data[1] = 0x11;
	data[2] = 0x04;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* Internal voltage set up */
	data[0] = 0xD3;
	data[1] = 0x1F;
	data[2] = 0x01;
	data[3] = 0x02;
	data[4] = 0x15;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal voltage set up */
	data[0] = 0xD4;
	data[1] = 0x62;
	data[2] = 0x1E;
	data[3] = 0x00;
	data[4] = 0xB7;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal display set up */
	data[0] = 0xC5;
	data[1] = 0x01;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC,
				EDISCO_SHORT_WRITE_1, data, 2);

	/* Load optimized red gamma (+) settings*/
	data[0] = 0xE9;
	data[1] = 0x01;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* Load optimized red gamma (-) settings*/
	data[0] = 0xEA;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* Load optimized green gamma (+) settings*/
	data[0] = 0xEB;
	data[1] = 0x02;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* Load optimized green gamma (-) settings*/
	data[0] = 0xEC;
	data[1] = 0x05;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* Load optimized blue gamma (+) settings*/
	data[0] = 0xED;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* Load optimized blue gamma (-) settings*/
	data[0] = 0xEE;
	data[1] = 0x07;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 10);

	/* turn on mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x03;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC,
					EDISCO_SHORT_WRITE_1, data, 2);

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("dsi_mipi_cm_480_854_panel_enable() \n");

	/* Check if the display we are using is actually a TMD display */
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) {
		if (read_supplier_id(dssdev) ==  CTL_SUPPLIER_ID_TMD) {
			DBG("dsi_mipi_cm_480_854_panel_enable() - TMD panel\n");
			dssdev->panel.panel_id = MOT_DISP_MIPI_CM_480_854;
		}
	}

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
						false, EDISCO_SHORT_WRITE_1,
						data, 2,
						EDISCO_CMD_SET_MCS, 1,
						EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS \n");


	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	data[3] = 0x00;
	ret = dsi_vc_write(lcd_ix, EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 4);

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	data[3] = 0x00;
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_480_854)
		/* Reading lane_config and it will return
		* 0x63 or 2-lanes, 0x60 for 1-lane (1st source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 4,
					0xef, 1,
					0x63, 0x63);
	else
		/* Reading lane_config and it will return
		* 0x1 for 2-lanes, 0x0 for 1-lane (2nd source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 4,
					EDISCO_CMD_DATA_LANE_CONFIG, 1,
					0x1, 0x1);

	if (ret)
		printk(KERN_ERR "failed to send LANE_CONFIG \n");

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = EDISCO_CMD_SET_DISPLAY_MODE;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_SET_DISPLAY_MODE, 1,
					data[1], 0x01);
	if (ret)
		printk(KERN_ERR "failed to send SET_DISPLAY_MODE \n");

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF for 1st source display only);
	 * D[1]=0 (Grama correction On for 1st source display only);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	/* AUO displays require a different setting */
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854)
		data[1] = 0x09;
	else
		data[1] = 0x1f;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_SET_BCKLGHT_PWM, 1,
					data[1], 0x1f);
	if (ret)
		printk(KERN_ERR "failed to send CABC/PWM \n");

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE \n");
		goto error;
	}

	mdelay(200);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_430_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;
	DBG("dsi_mipi_430_cm_480_854_panel_enable() \n");

	/* Exit sleep mode */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE \n");
		goto error;
	}

	/* 120ms delay for internal block stabilization */
	msleep(120);

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
				false, EDISCO_SHORT_WRITE_1,
				data, 2,
				EDISCO_CMD_SET_MCS, 1,
				EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS \n");


	/* Enable 2 data lanes */
	data[0] = EDISCO_CMD_DATA_LANE_CONFIG;
	data[1] = EDISCO_CMD_DATA_LANE_TWO;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			false, EDISCO_SHORT_WRITE_1,
			data, 2,
			EDISCO_CMD_DATA_LANE_CONFIG, 1,
			EDISCO_CMD_DATA_LANE_TWO, EDISCO_CMD_DATA_LANE_TWO);
	if (ret)
		printk(KERN_ERR "failed to send DATA_LANE_CONFIG \n");

	msleep(10);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 ;
	 * D[1]=0 ;
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	data[1] = 0xd9;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
				true, 0x00,
				data, 2,
				EDISCO_CMD_SET_BCKLGHT_PWM, 1,
				data[1], 0xff);
	if (ret)
		printk(KERN_ERR "failed to send CABC/PWM \n");

	mdelay(200);

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_310_1_cm_320_480_panel_enable(
				struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;


	DBG("dsi_mipi_310_1_cm_320_480_panel_enable() \n");

	data[0] = EDISCO_CMD_SOFT_RESET;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(15);

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(15);

	/*Dimming function setting */
	data[0] = 0x69;
	data[1] = 0x00;
	data[2] = 0xFF;
	data[3] = 0x00;
	data[4] = 0x14;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	/*Setting display brightness */
	data[0] = 0x51;
	data[1] = 0xFF;
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/*Setting CABC mode */
	data[0] = EDISCO_CMD_SET_CABC;
	data[1] = 0x02;	/* 0x02 = On 0x00 = OFF */
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/* setting CABC */
	data[0] = 0x53;
	data[1] = 0x16;	/* Enable CABC. BCTRL=1, DD=1, BL=1*/
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/* setting PWM */
	data[0] = 0x6B;
	data[1] = 0x00;
	data[2] = 0x01;	/* 0x01 = 31.26kHz */
	ret = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, data, 3);
	if (ret)
		goto error;

	return 0;

error:
	return -EINVAL;

}

static int dsi_mipi_cm_400_540_960_m1_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("%s\n", __func__);

	buf[0] = EDISCO_CMD_SOFT_RESET;
	r = dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(10);

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(10);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;
	buf[0] = 0xD2;
	buf[1] = 0x04;
	buf[2] = 0x35;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xD2;
	buf[1] = 0x05;
	buf[2] = 0x35;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xF7;
	buf[1] = 0x00;
	buf[2] = 0xD0;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xC1;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;
	buf[0] = 0xC3;
	buf[1] = 0x00;
	buf[2] = 0x03;
	buf[3] = 0x4C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 4);
	if (r)
		goto err;
	buf[0] = 0xF2;
	buf[1] = 0x0C;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x06;
	buf[5] = 0x04;
	buf[6] = 0x50;
	buf[7] = 0xF0;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x00;
	buf[12] = 0x01;
	buf[13] = 0x00;
	buf[14] = 0x00;
	buf[15] = 0x00;
	buf[16] = 0x55;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 17);
	if (r)
		goto err;
	buf[0] = 0xF3;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0xD0;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 5);
	if (r)
		goto err;
	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x45;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x2B;
	buf[12] = 0x2B;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 14);
	if (r)
		goto err;
	buf[0] = 0xF6;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x06;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;
	buf[0] = 0xF8;
	buf[1] = 0x0F;
	buf[2] = 0x0A;
	buf[3] = 0x20;
	buf[4] = 0x1E;
	buf[5] = 0x54;
	buf[6] = 0x54;
	buf[7] = 0x54;
	buf[8] = 0x54;
	buf[9] = 0x0A;
	buf[10] = 0x12;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR"\n****Failed to init the display****\n");
	return r;

}
static int dsi_mipi_cm_400_540_960_m1_v2_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("%s\n", __func__);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x00;
	buf[4] = 0x80;
	buf[5] = 0xC7;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0x66;
	buf[2] = 0xF6;
	buf[3] = 0x46;
	buf[4] = 0x9F;
	buf[5] = 0x90;
	buf[6] = 0x99;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x53;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x2F;
	buf[12] = 0x2F;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 14);
	if (r)
		goto err;

	buf[0] = 0xF8;
	buf[1] = 0x4B;
	buf[2] = 0x04;
	buf[3] = 0x10;
	buf[4] = 0x1A;
	buf[5] = 0x2C;
	buf[6] = 0x2C;
	buf[7] = 0x2C;
	buf[8] = 0x2C;
	buf[9] = 0x14;
	buf[10] = 0x12;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x30;
	buf[4] = 0x12;
	buf[5] = 0x0E;
	buf[6] = 0x0C;
	buf[7] = 0x22;
	buf[8] = 0x27;
	buf[9] = 0x31;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x30;
	buf[4] = 0x12;
	buf[5] = 0x0E;
	buf[6] = 0x0C;
	buf[7] = 0x22;
	buf[8] = 0x27;
	buf[9] = 0x31;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x37;
	buf[4] = 0x15;
	buf[5] = 0x15;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2A;
	buf[11] = 0x05;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x37;
	buf[4] = 0x15;
	buf[5] = 0x15;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2A;
	buf[11] = 0x05;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x3F;
	buf[4] = 0x16;
	buf[5] = 0x1F;
	buf[6] = 0x15;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2B;
	buf[11] = 0x06;
	buf[12] = 0x0B;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x3F;
	buf[4] = 0x16;
	buf[5] = 0x1F;
	buf[6] = 0x15;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2B;
	buf[11] = 0x06;
	buf[12] = 0x0B;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x20;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x34;
	buf[4] = 0x15;
	buf[5] = 0x1A;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x23;
	buf[9] = 0x2D;
	buf[10] = 0x29;
	buf[11] = 0x02;
	buf[12] = 0x08;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x34;
	buf[4] = 0x15;
	buf[5] = 0x1A;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x23;
	buf[9] = 0x2D;
	buf[10] = 0x29;
	buf[11] = 0x02;
	buf[12] = 0x08;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display**** \n");
	return r;

}

static int dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("dsi_mipi_400_cm_540_960_panel_enable\n");

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xD2;
	buf[1] = 0x04;
	buf[2] = 0x4D;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xD2;
	buf[1] = 0x05;
	buf[2] = 0x4D;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	/* DBLC MODE select, mode 2, normal mode set */
	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x00;
	buf[4] = 0x80;
	buf[5] = 0xC7;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0x66;
	buf[2] = 0xF6;
	buf[3] = 0x46;
	buf[4] = 0x9F;
	buf[5] = 0x90;
	buf[6] = 0x99;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x53;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x2F;
	buf[12] = 0x2F;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 14);
	if (r)
		goto err;

	buf[0] = 0xF8;
	buf[1] = 0x4B;
	buf[2] = 0x04;
	buf[3] = 0x10;
	buf[4] = 0x1A;
	buf[5] = 0x2C;
	buf[6] = 0x2C;
	buf[7] = 0x2C;
	buf[8] = 0x2C;
	buf[9] = 0x14;
	buf[10] = 0x12;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x1F;
	buf[3] = 0x23;
	buf[4] = 0x11;
	buf[5] = 0x0B;
	buf[6] = 0x02;
	buf[7] = 0x1B;
	buf[8] = 0x1F;
	buf[9] = 0x26;
	buf[10] = 0x28;
	buf[11] = 0x09;
	buf[12] = 0x16;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x0F;
	buf[2] = 0x3F;
	buf[3] = 0x11;
	buf[4] = 0x00;
	buf[5] = 0x0B;
	buf[6] = 0x02;
	buf[7] = 0x25;
	buf[8] = 0x33;
	buf[9] = 0x38;
	buf[10] = 0x36;
	buf[11] = 0x07;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x1F;
	buf[3] = 0x2C;
	buf[4] = 0x18;
	buf[5] = 0x0D;
	buf[6] = 0x06;
	buf[7] = 0x1F;
	buf[8] = 0x23;
	buf[9] = 0x25;
	buf[10] = 0x2A;
	buf[11] = 0x06;
	buf[12] = 0x14;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x0F;
	buf[2] = 0x3F;
	buf[3] = 0x1A;
	buf[4] = 0x04;
	buf[5] = 0x0D;
	buf[6] = 0x06;
	buf[7] = 0x29;
	buf[8] = 0x35;
	buf[9] = 0x37;
	buf[10] = 0x34;
	buf[11] = 0x07;
	buf[12] = 0x0D;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x1F;
	buf[3] = 0x3E;
	buf[4] = 0x19;
	buf[5] = 0x13;
	buf[6] = 0x12;
	buf[7] = 0x24;
	buf[8] = 0x25;
	buf[9] = 0x2C;
	buf[10] = 0x2A;
	buf[11] = 0x07;
	buf[12] = 0x14;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x0F;
	buf[2] = 0x3F;
	buf[3] = 0x2B;
	buf[4] = 0x05;
	buf[5] = 0x13;
	buf[6] = 0x12;
	buf[7] = 0x2E;
	buf[8] = 0x37;
	buf[9] = 0x3E;
	buf[10] = 0x34;
	buf[11] = 0x08;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x20;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x1F;
	buf[3] = 0x2E;
	buf[4] = 0x16;
	buf[5] = 0x0F;
	buf[6] = 0x10;
	buf[7] = 0x1D;
	buf[8] = 0x1F;
	buf[9] = 0x29;
	buf[10] = 0x2A;
	buf[11] = 0x0A;
	buf[12] = 0x14;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x0F;
	buf[2] = 0x3F;
	buf[3] = 0x20;
	buf[4] = 0x03;
	buf[5] = 0x0F;
	buf[6] = 0x10;
	buf[7] = 0x27;
	buf[8] = 0x31;
	buf[9] = 0x39;
	buf[10] = 0x34;
	buf[11] = 0x08;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
	if (r)
		goto err;

	/*  PWM output control */
	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display**** \n");
	return r;

}


static int dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("%s\n", __func__);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x00;
	buf[4] = 0x80;
	buf[5] = 0xD7;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0x66;
	buf[2] = 0xF6;
	buf[3] = 0x46;
	buf[4] = 0x9F;
	buf[5] = 0x90;
	buf[6] = 0x99;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 11);
	if (r)
		goto err;
	/* change to 0x7A to resolve the flicker and blackout issues */
	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0x7A;
	buf[3] = 0x46;
	buf[4] = 0x20;
	buf[5] = 0x00;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x14;
	buf[11] = 0x2F;
	buf[12] = 0x2F;
	buf[13] = 0x01;
	buf[14] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 15);
	if (r)
		goto err;

	buf[0] = 0xF6;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x06;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x0C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 7);
	if (r)
		goto err;

	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_400_540_960) {
		buf[0] = 0xF8;
		buf[1] = 0x1B;
		buf[2] = 0x1C;
		buf[3] = 0x00;
		buf[4] = 0x18;
		buf[5] = 0x49;
		buf[6] = 0x49;
		buf[7] = 0x49;
		buf[8] = 0X49;
		buf[9] = 0X14;
		buf[10] = 0X16;
		buf[11] = 0X01;
		buf[12] = 0X64;
		buf[13] = 0X64;
		buf[14] = 0X02;
		buf[15] = 0X24;
		buf[16] = 0X64;
		buf[17] = 0X00;
		buf[18] = 0X00;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 19);
		if (r)
			goto err;
		/* Red */
		buf[0] = 0xF9;
		buf[1] = 0x04;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x25;
		buf[4] = 0x11;
		buf[5] = 0x02;
		buf[6] = 0x01;
		buf[7] = 0x20;
		buf[8] = 0x24;
		buf[9] = 0x2f;
		buf[10] = 0x30;
		buf[11] = 0x08;
		buf[12] = 0x0d;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x25;
		buf[4] = 0x11;
		buf[5] = 0x02;
		buf[6] = 0x01;
		buf[7] = 0x20;
		buf[8] = 0x24;
		buf[9] = 0x2f;
		buf[10] = 0x30;
		buf[11] = 0x08;
		buf[12] = 0x0d;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		/* Green */
		buf[0] = 0xF9;
		buf[1] = 0x02;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x2b;
		buf[4] = 0x12;
		buf[5] = 0x07;
		buf[6] = 0x03;
		buf[7] = 0x22;
		buf[8] = 0x29;
		buf[9] = 0x31;
		buf[10] = 0x2e;
		buf[11] = 0x08;
		buf[12] = 0x0e;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x2b;
		buf[4] = 0x12;
		buf[5] = 0x07;
		buf[6] = 0x03;
		buf[7] = 0x22;
		buf[8] = 0x29;
		buf[9] = 0x31;
		buf[10] = 0x2e;
		buf[11] = 0x08;
		buf[12] = 0x0e;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		/* Blue */
		buf[0] = 0xF9;
		buf[1] = 0x01;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x10;
		buf[2] = 0x25;
		buf[3] = 0x2a;
		buf[4] = 0x0f;
		buf[5] = 0x12;
		buf[6] = 0x05;
		buf[7] = 0x20;
		buf[8] = 0x27;
		buf[9] = 0x2d;
		buf[10] = 0x2c;
		buf[11] = 0x08;
		buf[12] = 0x0a;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x10;
		buf[2] = 0x25;
		buf[3] = 0x2a;
		buf[4] = 0x0f;
		buf[5] = 0x12;
		buf[6] = 0x05;
		buf[7] = 0x20;
		buf[8] = 0x27;
		buf[9] = 0x2d;
		buf[10] = 0x2c;
		buf[11] = 0x08;
		buf[12] = 0x0a;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		/* White */
		buf[0] = 0xF9;
		buf[1] = 0x20;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x2f;
		buf[3] = 0x27;
		buf[4] = 0x0c;
		buf[5] = 0x0f;
		buf[6] = 0x10;
		buf[7] = 0x22;
		buf[8] = 0x28;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x09;
		buf[12] = 0x11;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x2f;
		buf[3] = 0x27;
		buf[4] = 0x0c;
		buf[5] = 0x0f;
		buf[6] = 0x10;
		buf[7] = 0x22;
		buf[8] = 0x28;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x09;
		buf[12] = 0x11;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;
	} else if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_540_960) {
		buf[0] = 0xF8;
		buf[1] = 0x15;
		buf[2] = 0x1C;
		buf[3] = 0x00;
		buf[4] = 0x18;
		buf[5] = 0x49;
		buf[6] = 0x49;
		buf[7] = 0x49;
		buf[8] = 0X49;
		buf[9] = 0X14;
		buf[10] = 0X16;
		buf[11] = 0X01;
		buf[12] = 0X64;
		buf[13] = 0X64;
		buf[14] = 0X02;
		buf[15] = 0X24;
		buf[16] = 0X64;
		buf[17] = 0X00;
		buf[18] = 0X00;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 19);
		if (r)
			goto err;

		/* Red */
		buf[0] = 0xF9;
		buf[1] = 0x04;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x29;
		buf[4] = 0x16;
		buf[5] = 0x08;
		buf[6] = 0x01;
		buf[7] = 0x22;
		buf[8] = 0x25;
		buf[9] = 0x2f;
		buf[10] = 0x30;
		buf[11] = 0x07;
		buf[12] = 0x0c;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x29;
		buf[4] = 0x16;
		buf[5] = 0x08;
		buf[6] = 0x01;
		buf[7] = 0x22;
		buf[8] = 0x25;
		buf[9] = 0x2f;
		buf[10] = 0x30;
		buf[11] = 0x07;
		buf[12] = 0x0c;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		/* Green */
		buf[0] = 0xF9;
		buf[1] = 0x02;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x30;
		buf[4] = 0x17;
		buf[5] = 0x0b;
		buf[6] = 0x03;
		buf[7] = 0x22;
		buf[8] = 0x26;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x07;
		buf[12] = 0x0b;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;
		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x25;
		buf[3] = 0x30;
		buf[4] = 0x17;
		buf[5] = 0x0b;
		buf[6] = 0x03;
		buf[7] = 0x22;
		buf[8] = 0x26;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x07;
		buf[12] = 0x0b;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		/* Blue */
		buf[0] = 0xF9;
		buf[1] = 0x01;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x10;
		buf[2] = 0x25;
		buf[3] = 0x2f;
		buf[4] = 0x14;
		buf[5] = 0x0f;
		buf[6] = 0x05;
		buf[7] = 0x21;
		buf[8] = 0x27;
		buf[9] = 0x2d;
		buf[10] = 0x2d;
		buf[11] = 0x07;
		buf[12] = 0x0a;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x10;
		buf[2] = 0x25;
		buf[3] = 0x2f;
		buf[4] = 0x14;
		buf[5] = 0x0f;
		buf[6] = 0x05;
		buf[7] = 0x21;
		buf[8] = 0x27;
		buf[9] = 0x2d;
		buf[10] = 0x2d;
		buf[11] = 0x07;
		buf[12] = 0x0a;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;
		/* White */
		buf[0] = 0xF9;
		buf[1] = 0x20;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
		if (r)
			goto err;

		buf[0] = 0xFA;
		buf[1] = 0x04;
		buf[2] = 0x2f;
		buf[3] = 0x27;
		buf[4] = 0x0c;
		buf[5] = 0x0f;
		buf[6] = 0x10;
		buf[7] = 0x22;
		buf[8] = 0x28;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x09;
		buf[12] = 0x11;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;

		buf[0] = 0xFB;
		buf[1] = 0x04;
		buf[2] = 0x2f;
		buf[3] = 0x27;
		buf[4] = 0x0c;
		buf[5] = 0x0f;
		buf[6] = 0x10;
		buf[7] = 0x22;
		buf[8] = 0x28;
		buf[9] = 0x31;
		buf[10] = 0x2f;
		buf[11] = 0x09;
		buf[12] = 0x11;
		r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 13);
		if (r)
			goto err;
	}

	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display**** \n");
	return r;
}

static int dsi_mipi_cm_430_540_960_m3_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	static u16 manufacture_id = INVALID_VALUE;
	static u16 controller_ver = INVALID_VALUE;

	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	enum omap_dsi_index lcd_ix =
		(dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("%s \n", __func__);

	if (controller_ver == INVALID_VALUE) {
		read_supplier_id(dssdev);
		manufacture_id = read_panel_manufacture(dssdev);
		controller_ver = read_panel_controller_version(dssdev);
		read_panel_controller_driver_version(dssdev);
		if (manufacture_id == INVALID_VALUE ||
			controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, manufacture_id"
				" =%d, controller_ver =%d.\n ", manufacture_id,
				controller_ver);
			return -EINVAL;
		}
	}

	/* Read revision,  ES1 or ES2 panel, ES1 can not run high mipi clk */
	if (controller_ver == 1) {
		reconfigure_dsi_pll(dssdev);
		/* Don't want to run TE in ES1 panel */
		panel_data->te_support = false;

		buf[0] = EDISCO_CMD_SOFT_RESET;
		r = dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, buf, 1);
		if (r)
			goto err;

		msleep(10);
	}

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0x51;
	buf[1] = 0xFF;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x03;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;
	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display**** \n");
	return r;

}

static int mapphone_panel_enable_te_locked(struct omap_dss_device *dssdev,
				bool enable);

static int dsi_mipi_cm_430_540_960_amoled_bl_set_locked(
	struct omap_dss_device *dssdev, int level, enum omap_dsi_index lcd_ix);

static int dsi_mipi_430_cm_540_960_amoled_panel_enable(
						struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[30];
	enum omap_dsi_index lcd_ix;

	DBG("dsi_mipi_430_cm_540_960_amoled_panel_enable\n");
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	column_address_offset = 30;

	msleep(20);

	/* Unlock key for LV2 command access */
	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xB1;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x16;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 4);
	if (r)
		goto err;

	buf[0] = 0xB2;
	buf[1] = 0x06;
	buf[2] = 0x06;
	buf[3] = 0x06;
	buf[4] = 0x06;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 5);
	if (r)
		goto err;

	r = dsi_mipi_cm_430_540_960_amoled_bl_set_locked(dssdev,
			mp_data->bldev->props.brightness, lcd_ix);
	if (r)
		goto err;

	/* Set internal timing setting block for Scan and EL driver */
	buf[0]  = 0xF8;
	buf[1]  = 0x28;
	buf[2]  = 0x28;
	buf[3]  = 0x08;
	buf[4]  = 0x08;
	buf[5]  = 0x40;
	buf[6]  = 0xB0;
	buf[7]  = 0x50;
	buf[8]  = 0x90;
	buf[9]  = 0x10;
	buf[10] = 0x30;
	buf[11] = 0x10;
	buf[12] = 0x00;
	buf[13] = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 14);
	if (r)
		goto err;

	/* Source AMP setting register */
	buf[0]  = 0xF6;
	buf[1]  = 0x00;
	buf[2]  = 0x84;
	buf[3]  = 0x09;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 4);
	if (r)
		goto err;

	/* Set the starting address of the parameter of next coming command */
	buf[0]  = 0xB0;
	buf[1]  = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xc0;
	buf[1]  = 0x00;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x09;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0x64;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x0b;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0xa4;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x0c;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0x7e;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x0d;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0x20;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x08;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xfd;
	buf[1]  = 0xf8;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x04;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xf2;
	buf[1]  = 0x4d;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	/* Exit sleep mode */
	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 1);
	if (r)
		goto err;

	msleep(120);

	/* Set column address */
	buf[0]  = 0x2A;
	buf[1]  = 0x00;
	buf[2]  = 0x1e;
	buf[3]  = 0x02;
	buf[4]  = 0x39;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 5);
	if (r)
		goto err;

	/* Set page address */
	buf[0]  = 0x2B;
	buf[1]  = 0x00;
	buf[2]  = 0x00;
	buf[3]  = 0x03;
	buf[4]  = 0xbf;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 5);
	if (r)
		goto err;

	/* ACL on */
	buf[0]  = 0xC0;
	buf[1]  = 0x01;
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display**** \n");
	return r;
}

/****************************************************************************
 *End of panel enable API
 ****************************************************************************/

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev);

static int mapphone_panel_MIPI_540_960_detect(struct omap_dss_device *dssdev)
{
	enum omap_dsi_index lcd_ix;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int ret;
	static u16 manufacture_id = INVALID_VALUE;
	static u16 controller_ver = INVALID_VALUE;

	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (manufacture_id  ==  INVALID_VALUE) {
		read_supplier_id(dssdev);
		manufacture_id = read_panel_manufacture(dssdev);
		controller_ver = read_panel_controller_version(dssdev);
		read_panel_controller_driver_version(dssdev);
		if (manufacture_id == INVALID_VALUE ||
			controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, manufacture_id"
				" =%d, controller_ver =%d.\n ", manufacture_id,
				controller_ver);
			return -EINVAL;
		}
	}
	/*
	 * Issue: see qHD panel dim screen sometimes, there is
	 * no PWM output, signal stays low, it happens randomly.
	 * The reason is CABC setting registers 0x53 or 0x55 are
	 * not programmed correctly in LP mode,0x54/0x56 read back
	 * 0x0 when they are supposed to be 0x2C and 0x01. qHD
	 * panel have communication problems sometimes in LP mode.
	 *
	 * To work it around, send qHD panel power up sequence
	 * in HS mode. ES2.0 and later support both LP and HS mode.
	 */
	omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, true);

	switch (manufacture_id) {
	case 00: /* 1st panel manufacture */
		switch (controller_ver) {
		case 00: /*ES1*/
			omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, false);

			reconfigure_dsi_pll(dssdev);
			/* Don't want to run TE in ES1 panel */
			panel_data->te_support = false;

			ret = dsi_mipi_cm_400_540_960_m1_v1_panel_enable(
				dssdev);
			break;
		case 01: /*ES2,3*/
			ret = dsi_mipi_cm_400_540_960_m1_v2_panel_enable(
				dssdev);
			break;
		case 02: /*ES4*/
		case 03: /*ES5*/
			ret = dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
				dssdev);
			break;
		default:
			ret = dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
				dssdev);
			printk(KERN_ERR " Not support controller version = %d,"
				" use E4/5 panel init sequence for now.\n",
				 controller_ver);
		}
		break;
	case 01: /* 2nd panel manufacture */
		switch (controller_ver) {
		case 00:
		case 01: /*ES1*/
		case 02: /*ES2*/
			ret = dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
				dssdev);
			break;
		default:
			ret = dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
				dssdev);
			printk(KERN_ERR " Not support controller version = %d,"
				" use ES2 panel init sequence for now.\n",
				 controller_ver);
		}
		break;
	default: /* 3rd and 4th panel manufacture */
		printk(KERN_ERR " Not support this panel manufacture = %d\n",
				manufacture_id);
		ret = -EINVAL;
	}

	return ret;
}

static int mapphone_panel_power_on(struct omap_dss_device *dssdev)
{
	static bool first_boot = true;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;
	int ret;
	u8 power_mode;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	ret = omapdss_dsi_display_enable(dssdev);
	if (ret) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	if (!first_boot)
		if (dssdev->platform_enable) {
			ret = dssdev->platform_enable(dssdev);
			if (ret)
				return ret;
		}

	mapphone_hw_reset(dssdev);

	omapdss_dsi_vc_enable_hs(ix, EDISCO_CMD_VC, false);

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_480_854:
	case MOT_DISP_MIPI_CM_370_480_854:
		ret = dsi_mipi_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_480_854:
		ret = dsi_mipi_430_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_310_320_480_1:
		ret = dsi_mipi_310_1_cm_320_480_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_400_540_960:
	case MOT_DISP_MIPI_CM_430_540_960:
		ret = mapphone_panel_MIPI_540_960_detect(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_540_960_3:
		ret = dsi_mipi_cm_430_540_960_m3_v1_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		ret = dsi_mipi_430_cm_540_960_amoled_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_VM_248_320_240:
		ret = dsi_mipi_248_vm_320_240_panel_enable(dssdev) ;
		break;
	case MOT_DISP_MIPI_VM_280_320_240:
		ret = dsi_mipi_280_vm_320_240_panel_enable(dssdev) ;
		break;
	default:
		printk(KERN_ERR "unsupport panel =0x%lx\n",
						dssdev->panel.panel_id);
		goto err;
	}

	if (ret)
		goto err;

	/* The first time we enable the display, read the current power mode.
	 * This is done to ensure AP + display are in sync for the display's
	 * state, as the bootloader can change the display's initial state.
	 * This is needed to ensure ESD check is done correctly.
	 */
	if (first_boot) {
		if (dsi_vc_dcs_read(ix, EDISCO_CMD_VC,
			EDISCO_CMD_GET_POWER_MODE, &power_mode, 1) != 1) {
			printk(KERN_ERR "Failed to read 'get_power_mode'"
				"first time\n");
			ret = -EINVAL;
			goto err;
		}

		/* Set the panel state to on if the display reports it is
		 * already on
		 */
		if (power_mode & 0x04)
			atomic_set(&panel_data->state, PANEL_ON);

		first_boot = false;
	}

	mp_data->enabled = 1;

	omapdss_dsi_vc_enable_hs(ix, EDISCO_CMD_VC, true);

	ret = mapphone_panel_enable_te_locked(dssdev, mp_data->te_enabled);
	if (ret)
		goto err;

	dss_mainclk_disable();

	printk(KERN_INFO "Mapphone Display is ENABLE\n");
	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	mapphone_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev);
err0:
	dss_mainclk_disable();
	return ret;
}

static int mapphone_panel_start(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&mp_data->lock);

	if (mp_data->force_update)
		mp_data->te_enabled = 1;

	dsi_bus_lock(ix);

	r = mapphone_panel_power_on(dssdev);

	dsi_bus_unlock(ix);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		if (panel_data->use_esd_check)
			queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
					MAPPHONE_ESD_CHECK_PERIOD);
	}
	mutex_unlock(&mp_data->lock);
	return r;
}


static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	DBG("enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED){
		printk(KERN_WARNING "panel state is not disabled, returns\n");
		return -EINVAL;
	}

	mapphone_panel_print_config(dssdev);

	return mapphone_panel_start(dssdev);
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev)
{
	u8 data[1];
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	atomic_set(&panel_data->state, PANEL_OFF);

	/*
	 * Change panel power down sequence to be aligned with spec.
	 * The cmd order has to be 10h  and then 28h, inverting them may
	 * cause exessive current according to spec.
	 * Note: might have sticking image, but did not see it yet.
	 */

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write_nosync(lcd_ix, EDISCO_CMD_VC, data, 1);

	msleep(120);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

}
static void mapphone_panel_power_off(struct omap_dss_device *dssdev)
{
	void *handle;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	DBG("mapphone_panel_disable\n");

	dss_mainclk_enable();

	handle = panel_data->panel_handle;

	omap_panel_fod_dss_state(handle, 0);
	if (omap_panel_fod_enabled(handle)) {
		DBG("Freezing the last frame on the display\n");
		return;
	}

	omap_panel_fod_panel_state(handle, 0);

	mapphone_panel_disable_local(dssdev);

	/*
	 * clk will be released in below after context
	 * is saved before enter RET
	 */
	omapdss_dsi_display_disable(dssdev);

	mp_data->enabled = 0;

}
static void mapphone_panel_stop(struct omap_dss_device *dssdev)
{

	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&mp_data->lock);

	if (mp_data->force_update)
		mp_data->te_enabled = 0;

	if (panel_data->use_esd_check)
		cancel_delayed_work(&mp_data->esd_work);

	dsi_bus_lock(ix);

	mapphone_panel_power_off(dssdev);

	dsi_bus_unlock(ix);

	mutex_unlock(&mp_data->lock);
}
static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{

	DBG("disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE ||
	    dssdev->state == OMAP_DSS_DISPLAY_TRANSITION)
		mapphone_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static bool mapphone_panel_support_te(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (!panel_data->te_support)
		return false;
	else
		return true;
}

static int mapphone_panel_enable_te_locked(struct omap_dss_device *dssdev,
					bool enable)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	enum omap_dsi_index ix;
	int r;
	u8 data[3];

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (enable) {
		data[0] = EDISCO_CMD_SET_TEAR_ON;
		data[1] = 0x00;
		r = dsi_vc_dcs_write(ix, EDISCO_CMD_VC, data, 2);
		if (r)
			goto error;

		data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
		data[1] = (panel_data->te_scan_line & 0xff00) >> 8;
		data[2] = (panel_data->te_scan_line & 0xff);
		r = dsi_vc_dcs_write(ix, EDISCO_CMD_VC, data, 3);
		if (r)
			goto error;
	} else {
		data[0] = EDISCO_CMD_SET_TEAR_OFF;
		r = dsi_vc_dcs_write(ix, EDISCO_CMD_VC, data, 1);

		if (r)
			goto error;
	}

	r = omapdss_dsi_enable_te(dssdev, enable, panel_data->te_type);

error:
	return r;
}

static int mapphone_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	int r = 0;
	struct mapphone_data *map_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;
	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&map_data->lock);

	if (map_data->te_enabled != enable) {
		dsi_bus_lock(ix);
		dss_mainclk_enable();

		r = mapphone_panel_enable_te_locked(dssdev, enable);
		if (!r) {
			map_data->te_enabled = enable;
			DBG("Changed TE state, TE = %d\n", enable);
		}

		dss_mainclk_disable();
		dsi_bus_unlock(ix);
	}

	mutex_unlock(&map_data->lock);

	return r;
}

static int mapphone_panel_get_hs_mode_timing(struct omap_dss_device *dssdev)
{
	/* The following time values are required for MIPI timing
	per OMAP spec */
	dssdev->phy.dsi.hs_timing.ths_prepare = 70;
	dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 175;
	dssdev->phy.dsi.hs_timing.ths_trail = 60;
	dssdev->phy.dsi.hs_timing.ths_exit = 145;
	dssdev->phy.dsi.hs_timing.tlpx_half = 25;
	dssdev->phy.dsi.hs_timing.tclk_trail = 60;
	dssdev->phy.dsi.hs_timing.tclk_prepare = 65;
	dssdev->phy.dsi.hs_timing.tclk_zero = 260;

	/* These values are required for the following spec panels */
	if ((dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_480_854) ||
		(dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854)) {
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 454;
	}

	DBG("Programmed values: ths_prepare=%u ths_prepare_ths_zero=%u\n"
		" ths_trail=%u ths_exit=%u tlpx_half=%u \n"
		" tclk_trail =%u tclk_prepare=%u tclk_zero=%u\n",
		dssdev->phy.dsi.hs_timing.ths_prepare,
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero,
		dssdev->phy.dsi.hs_timing.ths_trail,
		dssdev->phy.dsi.hs_timing.ths_exit,
		dssdev->phy.dsi.hs_timing.tlpx_half,
		dssdev->phy.dsi.hs_timing.tclk_trail,
		dssdev->phy.dsi.hs_timing.tclk_prepare,
		dssdev->phy.dsi.hs_timing.tclk_zero);

	return 0;
}

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if ((dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) ||
		(dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_480_854))
			panel_data->manual_te_trigger = true;
	else
		panel_data->manual_te_trigger = false;
}

static bool mapphone_panel_manual_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	return panel_data->manual_te_trigger;
}

static int mapphone_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int mapphone_panel_mirror(struct omap_dss_device *display, bool enable)
{
	return 0;
}

/* Todo:  should NOT show Vendor name, replace it with panel id later */
static int mapphone_panel_run_test(struct omap_dss_device *display,
					int test_num)
{
	int r = -1; /* Returns -1 if no dssdev or test isn't supported */
		/* Returns 0 on success, or the test_num on failure */
	u8 data[CTL_SUPPLIER_ID_LEN];
	u16 id = 0xFFFF;
	enum omap_dsi_index lcd_ix;
	lcd_ix = (display->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (!display)
		return r;

	if (test_num == 1)	{
		if ((display->panel.panel_id == MOT_DISP_MIPI_CM_430_480_854) ||
		    (display->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) ||
		    (display->panel.panel_id == MOT_DISP_MIPI_CM_480_854)) {
			r = test_num;
			/* Status check to ensure communication */
			dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC,
					CTL_SUPPLIER_ID_LEN);
			if (dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC,
					EDISCO_CMD_READ_DDB_START,
					data, CTL_SUPPLIER_ID_LEN)
					> 0) {
				id = (data[0] << 8) | data[1];

				switch (display->panel.panel_id) {
				case MOT_DISP_MIPI_CM_430_480_854:
					if (id == CTL_SUPPLIER_ID_AUO_43)
						r = 0;
					break;
				case MOT_DISP_MIPI_CM_370_480_854:
					if (id == CTL_SUPPLIER_ID_AUO)
						r = 0;
					break;
				case MOT_DISP_MIPI_CM_480_854:
					if (id == CTL_SUPPLIER_ID_TMD)
						r = 0;
					break;
				default:
					r = 0;
					break;
				}
			}
			dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC, 1);
		} else {
			/* If check not supported, return success */
			r = 0;
		}
	}
	return r;
}

static int mapphone_panel_suspend(struct omap_dss_device *dssdev)
{
	DBG("suspend\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE ||
            dssdev->state == OMAP_DSS_DISPLAY_TRANSITION)
		mapphone_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int mapphone_panel_resume(struct omap_dss_device *dssdev)
{
	int r;

	printk(KERN_INFO "panel resume.\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		printk(KERN_WARNING "panel state is not suspended, returns\n");
		return -EINVAL;
	}

	r = mapphone_panel_start(dssdev);
	if(r)
		printk(KERN_ERR "mapphone_panel_start returns err = %d.\n" ,r);

	return r;

}

static int mapphone_panel_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	if (mp_data->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode mapphone_panel_get_update_mode(
		struct omap_dss_device *dssdev)
{
	/*
	 * Currently all our displays perform manual update. In future, we might
	 * need to support different kind of pannels, then we will use
	 * the panelID to determine the display protocol type.
	 */
	return OMAP_DSS_UPDATE_MANUAL;
}

static int mapphone_panel_reg_read(struct omap_dss_device *dssdev,
				u8 address, u16 size, u8 *buf,
				u8 use_hs_mode)
{
	int r = -1;
	enum omap_dsi_index lcd_ix;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("read reg, address = 0x%X, size = %d\n", address, size);
	mutex_lock(&mp_data->lock);
	dsi_bus_lock(lcd_ix);
	dss_mainclk_enable();

	if (use_hs_mode)
		omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, true);
	else
		omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, false);

	r = dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC, size);
	if (r != 0)
		goto end;

	r = dsi_vc_dcs_read(lcd_ix, EDISCO_CMD_VC, address, buf, size);
	if (r != size)
		goto end;

	r = dsi_vc_set_max_rx_packet_size(lcd_ix, EDISCO_CMD_VC, 1);
	if (r != 0)
		goto end;

	r = 0;

end:
	dss_mainclk_disable();
	dsi_bus_unlock(lcd_ix);
	mutex_unlock(&mp_data->lock);
	DBG("read reg done, r = %d\n", r);
	return r;
}

static int mapphone_panel_reg_write(struct omap_dss_device *dssdev,
				u16 size, u8 *buf, u8 use_hs_mode)
{
	int r = -1;
	enum omap_dsi_index lcd_ix;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	DBG("write reg, size = %d\n", size);
	mutex_lock(&mp_data->lock);
	dsi_bus_lock(lcd_ix);
	dss_mainclk_enable();

	if (use_hs_mode)
		omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, true);
	else
		omapdss_dsi_vc_enable_hs(lcd_ix, EDISCO_CMD_VC, false);
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, size);

	dss_mainclk_disable();
	dsi_bus_unlock(lcd_ix);
	mutex_unlock(&mp_data->lock);
	DBG("write reg done, r = %d\n", r);
	return r;
}

static int dsi_mipi_cm_430_540_960_amoled_bl_set_locked(
	struct omap_dss_device *dssdev, int level, enum omap_dsi_index lcd_ix)
{
	int r = 0;
	u8 buf[2] = {0xfa, 0x03};
	int index;
	int range;
	int num_bl_steps;

	/* index 0 = dimmest */
	u8 bl_data[][26] = {
		{0xfa, 0x02, 0x20, 0x00, 0x20, 0xa0, 0x00, 0xa0, 0xd2, 0xa0,
		 0xd2, 0xd9, 0xd3, 0xd4, 0xb7, 0xbe, 0xb1, 0xc8, 0xd2, 0xc6,
		 0x00, 0x69, 0x00, 0x55, 0x00, 0x7a}, /*70 */
		{0xfa, 0x02, 0x20, 0x00, 0x20, 0xba, 0x00, 0xc7, 0xcc, 0xc0,
		 0xc8, 0xd4, 0xd8, 0xd2, 0xb3, 0xbd, 0xad, 0xc3, 0xcd, 0xc0,
		 0x00, 0x88, 0x00, 0x71, 0x00, 0xa0}, /* 150 */
		{0xfa, 0x02, 0x20, 0x00, 0x20, 0xca, 0x00, 0xca, 0xcb, 0xc8,
		 0xc8, 0xd1, 0xd7, 0xce, 0xae, 0xba, 0xa9, 0xc0, 0xca, 0xbc,
		 0x00, 0x9d, 0x00, 0x83, 0x00, 0xb8}, /* 220 */
		{0xfa, 0x02, 0x20, 0x20, 0x20, 0xda, 0x9c, 0xd9, 0xc4, 0xbb,
		 0xc0, 0xd1, 0xcf, 0xcf, 0xaa, 0xa9, 0xa5, 0xbe, 0xbf, 0xba,
		 0x00, 0xaf, 0x00, 0x93, 0x00, 0xcf} /* 300 */
	};

	num_bl_steps = (sizeof(bl_data)/sizeof(bl_data[0]));

	if (level < 0)
		level = 0;
	else if (level > MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX)
		level = MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;

	range = (MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX /
		num_bl_steps) + 1;

	index = level / range;

	if (index >= num_bl_steps)
		index = num_bl_steps - 1;

	DBG("Set 430_540_960_amoled bl, level = %d, index = %d", level, index);
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, bl_data[index],
			sizeof(bl_data[index]));
	if (r)
		goto err;

	/* Gamma set update enable */
	r = dsi_vc_dcs_write(lcd_ix, EDISCO_CMD_VC, buf, sizeof(buf));
	if (r)
		goto err;

	return 0;

err:
	printk(KERN_ERR "Failed to set AMOLED BL index = %d, r = %d\n",
		index, r);
	return r;
}

static int dsi_mipi_cm_430_540_960_amoled_bl_get_brightness(
	struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	/* Return 0 if backlight is off */
	return 0;
}

static int dsi_mipi_cm_430_540_960_amoled_bl_update_status(
	struct backlight_device *bl)
{
	int r = -1;
	int level;
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index lcd_ix;
	lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = 0;
	} else {
		mutex_lock(&mp_data->lock);
		dsi_bus_lock(lcd_ix);
		dss_mainclk_enable();
		r = dsi_mipi_cm_430_540_960_amoled_bl_set_locked(dssdev,
								level, lcd_ix);
		dss_mainclk_disable();
		dsi_bus_unlock(lcd_ix);
		mutex_unlock(&mp_data->lock);
	}

	return r;
}

static const struct backlight_ops dsi_mipi_cm_430_540_960_amoled_bl_ops = {
	.get_brightness = dsi_mipi_cm_430_540_960_amoled_bl_get_brightness,
	.update_status = dsi_mipi_cm_430_540_960_amoled_bl_update_status,
};

static int dsi_mipi_cm_430_540_960_amoled_bl_probe(
	struct omap_dss_device *dssdev, struct mapphone_data *mp_data)
{
	struct backlight_properties bl_props;
	struct backlight_device *bl;
	int r = 0;

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.max_brightness =
		MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;
	bl = backlight_device_register("430_540_960_amoled_bl",
				&dssdev->dev, dssdev,
				&dsi_mipi_cm_430_540_960_amoled_bl_ops,
				&bl_props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		return r;
	}
	mp_data->bldev = bl;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness =
		MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;
	return r;
}


static struct omap_dss_driver mapphone_panel_driver = {
	.probe		= mapphone_panel_probe,
	.remove		= mapphone_panel_remove,

	.enable		= mapphone_panel_enable,
	.framedone	= mapphone_panel_display_on,
	.disable	= mapphone_panel_disable,
	.suspend	= mapphone_panel_suspend,
	.resume		= mapphone_panel_resume,

	.set_update_mode	= mapphone_panel_set_update_mode,
	.get_update_mode	= mapphone_panel_get_update_mode,

	.update		= mapphone_panel_update,
	.sync		= mapphone_panel_sync,

	.get_resolution		= mapphone_panel_get_resolution,
	.get_dimension          = mapphone_panel_get_dimension,
	.get_recommended_bpp	= omapdss_default_get_recommended_bpp,

	.set_timings	= mapphone_panel_set_timings,

	.hs_mode_timing		= mapphone_panel_get_hs_mode_timing,
	.support_te             = mapphone_panel_support_te,
	.enable_te		= mapphone_panel_enable_te,
	.get_te			= mapphone_panel_get_te,

	.manual_te_trigger	= mapphone_panel_manual_te_trigger,
	.set_rotate		= mapphone_panel_rotate,
	.get_rotate		= mapphone_panel_get_rotate,
	.set_mirror		= mapphone_panel_mirror,
	.run_test		= mapphone_panel_run_test,
	.memory_read		= mapphone_panel_memory_read,
	.get_timings		= mapphone_panel_get_timings,
	.reg_read		= mapphone_panel_reg_read,
	.reg_write		= mapphone_panel_reg_write,

	.driver = {
		.name = "mapphone-panel",
		.owner = THIS_MODULE,
	},
};


static int __init mapphone_panel_init(void)
{
	DBG("mapphone_panel_init\n");

	omap_dss_register_driver(&mapphone_panel_driver);

	return 0;
}

static void __exit mapphone_panel_exit(void)
{
	DBG("mapphone_panel_exit\n");
	omap_dss_unregister_driver(&mapphone_panel_driver);
}

module_init(mapphone_panel_init);
module_exit(mapphone_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
