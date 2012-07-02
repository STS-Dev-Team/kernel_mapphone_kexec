/*
 * panel-mapphone-omap4-hdtv.c
 *
 * The starting point for this code was TI's DSS HDMI component.
 * The original licensing information is below.
 *
 * ===========================================================================
 *
 * Copyright (C) 2009 Texas Instruments
 * Author: Yong Zhi
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <plat/display.h>
#include <plat/cpu.h>
#include <plat/hdmi_lib.h>
#include <plat/gpio.h>
#include <plat/omap-pm.h>

#include "../dss/dss.h"

/*#define HDTV_DEBUG*/
#ifdef HDTV_DEBUG
#define HDTVDBG(format, ...) \
	printk("hdtv: " format, ## __VA_ARGS__)
#else
#define HDTVDBG(format, ...)
#endif

#define HDMI_EDID_BLOCK_LENGTH  (128)
#define HDMI_EDID_MAX_BLOCKS    (4)
#define HDMI_EDID_MAX_LENGTH    (HDMI_EDID_MAX_BLOCKS * HDMI_EDID_BLOCK_LENGTH)
#define HDMI_EDID_NUM_EXT_BLKS  (0x7E)

#define HDMI_PLLCTRL		0x58006200
#define HDMI_PHY		0x58006300

#define PLLCTRL_PLL_CONTROL	0x0ul
#define PLLCTRL_PLL_STATUS	0x4ul
#define PLLCTRL_PLL_GO		0x8ul
#define PLLCTRL_CFG1		0xCul
#define PLLCTRL_CFG2		0x10ul
#define PLLCTRL_CFG3		0x14ul
#define PLLCTRL_CFG4		0x20ul

#define HDMI_TXPHY_TX_CTRL	0x0ul
#define HDMI_TXPHY_DIGITAL_CTRL	0x4ul
#define HDMI_TXPHY_POWER_CTRL	0x8ul
#define HDMI_TXPHY_PAD_CFG_CTRL	0xCul

#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))
#define REG_FLD_MOD(b, i, v, s, e) \
	hdmi_write_reg(b, i, FLD_MOD(hdmi_read_reg(b, i), v, s, e))

#define HDMI_STATUS_CONNECT       (1 << 0)
#define HDMI_STATUS_DISCONNECT    (1 << 1)
#define HDMI_STATUS_HOTPLUG       (1 << 2)

struct hdmi_hvsync_pol {
	int vsync_pol;
	int hsync_pol;
};

enum hdmi_s3d_frame_structure {
	HDMI_S3D_FRAME_PACKING          = 0,
	HDMI_S3D_FIELD_ALTERNATIVE      = 1,
	HDMI_S3D_LINE_ALTERNATIVE       = 2,
	HDMI_S3D_SIDE_BY_SIDE_FULL      = 3,
	HDMI_S3D_L_DEPTH                = 4,
	HDMI_S3D_L_DEPTH_GP_GP_DEPTH    = 5,
	HDMI_S3D_SIDE_BY_SIDE_HALF      = 8
};

/* Subsampling types used for Sterioscopic 3D over HDMI. Below HOR
stands for Horizontal, QUI for Quinxcunx Subsampling, O for odd fields,
E for Even fields, L for left view and R for Right view*/
enum hdmi_s3d_subsampling_type {
	HDMI_S3D_HOR_OL_OR = 0,/*horizontal subsampling with odd fields
		from left view and even fields from the right view*/
	HDMI_S3D_HOR_OL_ER = 1,
	HDMI_S3D_HOR_EL_OR = 2,
	HDMI_S3D_HOR_EL_ER = 3,
	HDMI_S3D_QUI_OL_OR = 4,
	HDMI_S3D_QUI_OL_ER = 5,
	HDMI_S3D_QUI_EL_OR = 6,
	HDMI_S3D_QUI_EL_ER = 7
};

/* This is the structure which has all supported timing values
 * that OMAP4 supports
 */
const struct omap_video_timings video_timing_table[] = {
	{640, 480, 25200, 96, 16, 48, 2, 10, 33},
	{1280, 720, 74250, 40, 440, 220, 5, 5, 20},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	{720, 480, 27027, 62, 16, 60, 6, 9, 30},
	{2880, 576, 108000, 256, 48, 272, 5, 5, 39},
	{1440, 240, 27000, 124, 38, 114, 3, 4, 15},
	{1440, 288, 27000, 126, 24, 138, 3, 2, 19},
	{1920, 540, 74250, 44, 528, 148, 5, 2, 15},
	{1920, 540, 74250, 44, 88, 148, 5, 2, 15},
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},
	{720, 576, 27000, 64, 12, 68, 5, 5, 39},
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39},
	{1920, 1080, 148500, 44, 528, 148, 5, 4, 36},
	{2880, 480, 108000, 248, 64, 240, 6, 9, 30},
	{1920, 1080, 74250, 44, 638, 148, 5, 4, 36},
	/* Vesa from here */
#define VESA_TIMING_IDX_START 15
	{640, 480, 25175, 96, 16, 48, 2, 10, 33},
	{800, 600, 40000, 128, 40, 88, 4 , 1, 23},
	{848, 480, 33750, 112, 16, 112, 8 , 6, 23},
	{1280, 768, 79500, 128, 64, 192, 7 , 3, 20},
	{1280, 800, 83500, 128, 72, 200, 6 , 3, 22},
	{1360, 768, 85500, 112, 64, 256, 6 , 3, 18},
	{1280, 960, 108000, 112, 96, 312, 3 , 1, 36},
	{1280, 1024, 108000, 112, 48, 248, 3 , 1, 38},
	{1024, 768, 65000, 136, 24, 160, 6, 3, 29},
	{1400, 1050, 121750, 144, 88, 232, 4, 3, 32},
	{1440, 900, 106500, 152, 80, 232, 6, 3, 25},
	{1680, 1050, 146250, 176 , 104, 280, 6, 3, 30},
	{1366, 768, 85500, 143, 70, 213, 3, 3, 24},
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},
	{1280, 768, 68250, 32, 48, 80, 7, 3, 12},
	{1400, 1050, 101000, 32, 48, 80, 4, 3, 23},
	{1680, 1050, 119000, 32, 48, 80, 6, 3, 21},
	{1280, 800, 79500, 32, 48, 80, 6, 3, 14},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	/* supported 3d timings UNDEROVER full frame */
	{1280, 1470, 148350, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 440, 220, 5, 5, 20}
};

#define TIMING_TABLE_SIZE (ARRAY_SIZE(video_timing_table))

/* This is a static Mapping array which maps the timing values with
 * corresponding CEA / VESA code
 */
static const int code_index[TIMING_TABLE_SIZE] = {
	1,  19,  4,  2, 37,  6, 21, 20,  5, 16, 17,
	29, 31, 35, 32,
	/* <--15 CEA 22--> vesa*/
	4, 9, 0xE, 0x17, 0x1C, 0x27, 0x20, 0x23, 0x10, 0x2A,
	0x2F, 0x3A, 0x51, 0x52, 0x16, 0x29, 0x39, 0x1B, 0x55, 4,
	4, 19
};

/* Static mapping of the Timing values with the corresponding Vsync and
 * Hsync polarity
 */
static const struct hdmi_hvsync_pol hvpol_mapping[TIMING_TABLE_SIZE] = {
	{0, 0}, {1, 1}, {1, 1}, {0, 0},
	{0, 0}, {0, 0}, {0, 0}, {1, 1},
	{1, 1}, {1, 1}, {0, 0}, {0, 0},
	{1, 1}, {0, 0}, {1, 1}, /* VESA */
	{0, 0}, {1, 1}, {1, 1}, {1, 0},
	{1, 0}, {1, 1}, {1, 1}, {1, 1},
	{0, 0}, {1, 0}, {1, 0}, {1, 0},
	{1, 1}, {1, 1}, {0, 1}, {0, 1},
	{0, 1}, {0, 1}, {1, 1}, {1, 1},
	{1, 1}, {1, 1}
};

/* Map CEA code to the corresponding timing values (10 entries/line) */
static const int code_cea[] = {
	-1,  0,  3,  3,  2,  8,  5,  5, -1, -1,
	-1, -1, -1, -1, -1, -1,  9, 10, 10,  1,
	7,   6,  6, -1, -1, -1, -1, -1, -1, 11,
	11, 12, 14, -1, -1, 13, 13,  4,  4
};

/* Map CEA code to the corresponding 3D timing values */
static const int s3d_code_cea[] = {
	-1, -1, -1, -1, 35, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, 36,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1
};

/* Map VESA code to the corresponding timing values */
static const int code_vesa[] = {
	-1, -1, -1, -1, 15, -1, -1, -1, -1, 16,
	-1, -1, -1, -1, 17, -1, 23, -1, -1, -1,
	-1, -1, 29, 18, -1, -1, -1, 32, 19, -1,
	-1, -1, 21, -1, -1, 22, -1, -1, -1, 20,
	-1, 30, 24, -1, -1, -1, -1, 25, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, 31, 26, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, 27, 28, -1, -1, 33
};

struct hdmi_s3d_info {
	bool subsamp;
	int  structure;
	int  subsamp_pos;
};

struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};

struct hdmi_dev_data {
	struct kobject  kobj;
	void __iomem   *base_phy;
	void __iomem   *base_pll;

	struct omap_display_platform_data *pdata;
	struct platform_device            *pdev;
	struct omap_dss_device            *dssdev;

	struct mutex lock; /* locks struct variables */

	/* Platform defined data */
	int def_code;
	int def_mode;
	int clkin_khz;
	u8  inv_lane_d0;
	u8  inv_lane_d1;
	u8  inv_lane_d2;
	u8  inv_lane_clk;

	int hpd_gpio;
	int hpd_irq;

	/* State and configuration data */
	int code;
	int mode;
	bool s3d_enabled;

	struct hdmi_config cfg;

	bool det_en;
	bool det_resume;
	bool out_en;
	bool attached;
	bool connected;

	bool hpd_state; /* Last reported HPD state */

	/* IRQ handling data */
	int                      working;
	struct work_struct       work;
	struct workqueue_struct *workqueue;
	spinlock_t               irqstatus_lock; /* IRQ status lock*/
	int                      irqstatus;

	/* EDID related data */
	u8                       edid[HDMI_EDID_MAX_LENGTH];
	int                      edid_len;
	bool                     edid_valid;

	/* Client specified timing data */
	bool                      client_timing;
	struct omap_video_timings timing;

	int test;
};

static struct hdmi_s3d_info hdmi_s3d;

static struct hdmi_dev_data gdev;

static int  hdmi_set_hot_plug_status(struct omap_dss_device *dssdev, bool en);

static int  hdmi_pll_start(int pclk_khz);
static void hdmi_pll_stop(void);
static int  hdmi_phy_start(int pclk_khz, u8 ds_percent);
static void hdmi_phy_stop(void);
static int  hdmi_output_start(struct omap_dss_device *dssdev);
static void hdmi_output_stop(void);
static int  hdmi_detect_start(struct omap_dss_device *dssdev);
static void hdmi_detect_stop(struct omap_dss_device *dssdev);
static int  hdmi_attach_start(struct omap_dss_device *dssdev);
static void hdmi_attach_stop(struct omap_dss_device *dssdev);
static int  hdmi_irq_init(void);
static void hdmi_irq_term(void);
static int  hdmi_irq_connect_enable(int en);
static int  hdmi_irq_hpd_enable(int en);
static void hdmi_irq_force_chk(void);

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static int  hdmi_set_edid_timing(struct omap_dss_device *dssdev,
			enum omap_dss_edid_timing_mode mode, int code);
static void hdmi_dump_edid(struct omap_dss_device *dssdev);
static int  hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
static int  hdmi_get_edid(struct omap_dss_device *dssdev, u8 *edid, int len);
static int  hdmi_enable_hpd(struct omap_dss_device *dssdev, bool enable);
static bool hdmi_get_s3d_enabled(struct omap_dss_device *dssdev);
static int  hdmi_enable_s3d(struct omap_dss_device *dssdev, bool enable);
static int  hdmi_set_s3d_disp_type(struct omap_dss_device *dssdev,
			struct s3d_disp_info *info);

/*===========================================================================
 * Utility functions
 */

static inline void hdmi_write_reg(u32 base, u16 idx, u32 val)
{
	void __iomem *b;

	switch (base) {
	case HDMI_PHY:
		b = gdev.base_phy;
		break;
	case HDMI_PLLCTRL:
		b = gdev.base_pll;
		break;
	default:
		BUG();
	}
	__raw_writel(val, b + idx);
}

static inline u32 hdmi_read_reg(u32 base, u16 idx)
{
	void __iomem *b;
	u32 l;

	switch (base) {
	case HDMI_PHY:
		b = gdev.base_phy;
		break;
	case HDMI_PLLCTRL:
		b = gdev.base_pll;
		break;
	default:
		BUG();
	}
	l = __raw_readl(b + idx);

	return l;
}

static int get_s3d_timings_index(void)
{
	int code;

	code = s3d_code_cea[gdev.code];

	if (code == -1) {
		gdev.s3d_enabled   = false;
		gdev.client_timing = false;
		gdev.mode = gdev.def_mode;
		gdev.code = gdev.def_code;
		code = (gdev.mode == OMAP_DSS_EDID_TIMING_MODE_VESA) ?
				code_vesa[gdev.code] : code_cea[gdev.code];
	}
	return code;
}

static int get_timings_index(void)
{
	int code;

	code = (gdev.mode == OMAP_DSS_EDID_TIMING_MODE_VESA) ?
				code_vesa[gdev.code] : code_cea[gdev.code];

	if (code == -1)	{
		gdev.client_timing = false;
		gdev.mode = gdev.def_mode;
		gdev.code = gdev.def_code;
		code = (gdev.mode == OMAP_DSS_EDID_TIMING_MODE_VESA) ?
				code_vesa[gdev.code] : code_cea[gdev.code];
	}

	return code;
}

static int is_mode_code_valid(int mode, int code)
{
	int c = (mode == OMAP_DSS_EDID_TIMING_MODE_VESA) ?
				code_vesa[code] : code_cea[code];

	return (c != -1);
}

static void hdmi_enable_clocks(int enable)
{
	int clks = (DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M | DSS_CLK_96M);

	if (enable) {
		dss_clk_enable(clks);
		hdmi_opt_clock_enable();
	} else {
		hdmi_opt_clock_disable();
		dss_clk_disable(clks);
	}
}

static int hdmi_perform_edid_read(u8 *edid, int len)
{
	return HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, edid, len);
}

static void hdmi_validate_edid(void)
{
	int i, s;

	/* Just perform a basic first block verification */

	gdev.edid_valid = false;
	if (gdev.edid_len < HDMI_EDID_BLOCK_LENGTH)
		return;

	if (gdev.edid[0]   != 0x00
	   || gdev.edid[1] != 0xFF
	   || gdev.edid[2] != 0xFF
	   || gdev.edid[3] != 0xFF
	   || gdev.edid[4] != 0xFF
	   || gdev.edid[5] != 0xFF
	   || gdev.edid[6] != 0xFF
	   || gdev.edid[7] != 0x00)
		return;

	for (i = 0, s = 0; i < HDMI_EDID_BLOCK_LENGTH; i++)
		s += gdev.edid[i];

	if ((s & 0xFF) == 0)
		gdev.edid_valid = true;
}

static int hdmi_get_edid_cea_blk_offset(void)
{
	int offset = 0;
	int cnt;
	int idx;

	/* Function assumes a valid EDID */

	cnt = gdev.edid[HDMI_EDID_NUM_EXT_BLKS];
	if (cnt == 0) {
		HDTVDBG("No EDID Ext Blocks\n");
	} else {
		if (cnt > (HDMI_EDID_MAX_BLOCKS - 1))
			cnt = (HDMI_EDID_MAX_BLOCKS - 1);

		idx = HDMI_EDID_BLOCK_LENGTH;
		while (cnt > 0) {
			if (gdev.edid[idx] == 2 && gdev.edid[idx + 1] == 3) {
				offset = idx;
				break;
			}

			cnt--;
			idx += HDMI_EDID_BLOCK_LENGTH;
		}
	}

	return offset;
}

static int hdmi_get_edid_cea_vsdb_offset(int cea_offset)
{
	int idx = cea_offset + 4;
	int end = idx + (gdev.edid[idx] - 4);
	int type;
	int size;
	int offset = 0;

	/* Function assumes a valid EDID */

	while (idx < end) {
		type = (gdev.edid[idx] & 0xE0) >> 5;
		size = (gdev.edid[idx] & 0x1F);
		idx += 1;

		if (type == 3) {
			offset = idx - 1;
			break;
		}

		idx += size;
	}

	return offset;
}

static bool hdmi_is_ai_supported(void)
{
	bool supported = false;
	int  offset;

	/* Make sure we have a valid EDID */
	if (gdev.edid_valid) {
		/* We are good */
	} else if (hdmi_perform_edid_read(gdev.edid,
					HDMI_EDID_MAX_LENGTH) != 0) {
		HDTVDBG("WARNING: No EDID Data\n");
		goto exit;
	} else {
		gdev.edid_valid = true;
		gdev.edid_len   = HDMI_EDID_MAX_LENGTH;
	}

	/* Find the AI info */
	offset = hdmi_get_edid_cea_blk_offset();
	if (offset == 0)
		goto exit;

	offset = hdmi_get_edid_cea_vsdb_offset(offset);
	if (offset == 0)
		goto exit;

	if ((gdev.edid[offset] & 0x1F) < 6)
		goto exit;

	if (gdev.edid[offset + 6] & 0x80)
		supported = true;

exit:
	return supported;
}

static int hdmi_is_timing_set(void)
{
	int curr = 0;

	if (gdev.client_timing) {
		if (gdev.code == 0
		   && gdev.timing.pixel_clock == gdev.cfg.pixel_clock
		   && gdev.timing.x_res       == gdev.cfg.ppl
		   && gdev.timing.y_res       == gdev.cfg.lpp
		   && gdev.timing.hfp         == gdev.cfg.hfp
		   && gdev.timing.hsw         == gdev.cfg.hsw
		   && gdev.timing.hbp         == gdev.cfg.hbp
		   && gdev.timing.vfp         == gdev.cfg.vfp
		   && gdev.timing.vsw         == gdev.cfg.vsw
		   && gdev.timing.vbp         == gdev.cfg.vbp)
			curr = 1;
	} else {
		if (gdev.mode == gdev.cfg.hdmi_dvi
		   && gdev.code == gdev.cfg.video_format)
			curr = 1;
	}

	return curr;
}

static void hdmi_determine_timings(struct omap_dss_device *dssdev)
{
	int idx;
	const struct omap_video_timings *p;

	if (gdev.client_timing) {
		p = &gdev.timing;
		gdev.cfg.h_pol = (p->sp & 0x01);
		gdev.cfg.v_pol = (p->sp & 0x02) >> 1;
	} else {
		if (gdev.s3d_enabled
		   && (hdmi_s3d.structure == HDMI_S3D_FRAME_PACKING))
			idx = get_s3d_timings_index();
		else
			idx = get_timings_index();

		p = &video_timing_table[idx];
		gdev.cfg.h_pol = hvpol_mapping[idx].hsync_pol;
		gdev.cfg.v_pol = hvpol_mapping[idx].vsync_pol;
	}

	memcpy(&dssdev->panel.timings, p, sizeof(dssdev->panel.timings));

	gdev.cfg.ppl         = p->x_res;
	gdev.cfg.lpp         = p->y_res;
	gdev.cfg.pixel_clock = p->pixel_clock;
	gdev.cfg.hfp         = p->hfp;
	gdev.cfg.hsw         = p->hsw;
	gdev.cfg.hbp         = p->hbp;
	gdev.cfg.vfp         = p->vfp;
	gdev.cfg.vsw         = p->vsw;
	gdev.cfg.vbp         = p->vbp;

	if (gdev.s3d_enabled) {
		gdev.cfg.vsi_enabled   = true;
		gdev.cfg.s3d_structure = hdmi_s3d.structure;
		gdev.cfg.subsamp_pos   = hdmi_s3d.subsamp_pos;
	} else {
		gdev.cfg.vsi_enabled = false;
	}

	gdev.cfg.hdmi_dvi     = gdev.mode;
	gdev.cfg.video_format = gdev.code;

	if (gdev.mode == OMAP_DSS_EDID_TIMING_MODE_CEA) {
		switch (gdev.code) {
		case OMAP_DSS_EDID_TIMING_CEA_1920x1080I_50Hz:
		case OMAP_DSS_EDID_TIMING_CEA_1920x1080I_60Hz:
		case OMAP_DSS_EDID_TIMING_CEA_720x480I_60Hz:
		case OMAP_DSS_EDID_TIMING_CEA_720x480I_60Hz_WIDE:
		case OMAP_DSS_EDID_TIMING_CEA_720x576I_50Hz:
		case OMAP_DSS_EDID_TIMING_CEA_720x576I_50Hz_WIDE:
			gdev.cfg.interlace = 1;
			break;
		default:
			gdev.cfg.interlace = 0;
			break;
		}
	}

	HDTVDBG("TIMING/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d\n",
		gdev.cfg.hdmi_dvi, gdev.cfg.video_format, gdev.cfg.supports_ai,
		gdev.cfg.ppl, gdev.cfg.lpp, gdev.cfg.pixel_clock, gdev.cfg.hfp,
		gdev.cfg.hsw, gdev.cfg.hbp, gdev.cfg.vfp, gdev.cfg.vsw,
		gdev.cfg.vbp, gdev.cfg.h_pol, gdev.cfg.v_pol);
}

/*
 * This function is used to configure limited range/full range for
 * RGB format.  YUV format Full range is not supported.
 *
 * Please refer to section 6.6 Video quantization ranges in HDMI 1.3a
 * specification for more details.
 *
 * Conversion to full range or limited range can either be done at display
 * controller or HDMI IP.  This function allows to select either.
 *
 * Please note: When converting to full range it is better to convert the
 * video in the dispc to full range as there will be no loss of data.
 * If a limited range data is sent to HDMI and converted to full range in
 * HDMI the data quality will not be good.
 */
static void hdmi_configure_lr_fr(void)
{
	if (gdev.mode == 0 || (gdev.mode == 1 && gdev.code == 1)) {
		if (hdmi_configure_lrfr(HDMI_FULL_RANGE, 1) == 0)
			dispc_setup_color_fr_lr(1);
	} else {
		if (hdmi_configure_lrfr(HDMI_LIMITED_RANGE, 1) == 0)
			dispc_setup_color_fr_lr(0);
	}
}

/*===========================================================================
 * sysfs related functions
 */

static ssize_t hdmi_attached_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", gdev.attached ? "1" : "0");
}

static ssize_t hdmi_attached_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

static DEVICE_ATTR(attached, S_IRUGO, hdmi_attached_show, hdmi_attached_store);

#ifdef CONFIG_DEBUG_FS
static ssize_t hdmi_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gdev.test);
}

static ssize_t hdmi_test_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long test;
	int rc;

	rc = strict_strtoul(buf, 0, &test);
	if (rc || test > 2)
		return -EINVAL;

	gdev.test = test;

	/* The use of the backlight function is just a test hack */
	if (gdev.dssdev->set_backlight)
		gdev.dssdev->set_backlight(gdev.dssdev, test);

	return size;
}

static DEVICE_ATTR(test, S_IRUGO|S_IWUSR, hdmi_test_show, hdmi_test_store);
#endif

static int hdmi_set_hot_plug_status(struct omap_dss_device *dssdev, bool en)
{
	int rc = 0;

	if (gdev.det_en)
		hdmi_notify_hpd(en);

	omap_dss_notify(dssdev, en ? OMAP_DSS_HOTPLUG_CONNECT :
		OMAP_DSS_HOTPLUG_DISCONNECT);

	DSSINFO("HPD Event (%d)\n", en);
	rc = kobject_uevent(&dssdev->dev.kobj, en ? KOBJ_ADD : KOBJ_REMOVE);

	return rc;
}

/*===========================================================================
 * PLL related functions
 */

static void hdmi_pll_compute(int clkin, int phy, int n,
					struct hdmi_pll_info *pi)
{
	static const u32 freq_lo_limit  = 500;    /* 0.5 MHz */
	static const u32 freq_hi_limit  = 2500;   /* 2.5 MHz */
	static const u32 dco_freq_limit = 100000; /* 100 MHz */
	u32 clkinKHz = clkin;
	u32 phyKHz   = phy;
	u32 phy_x_nplus1;
	u32 t;

	/* Validate N, adjust as required */
	t = clkinKHz / (n + 1);
	if (t < freq_lo_limit) {
		printk(KERN_WARNING "hdmi_compute_pll: N too large (%d)\n", n);
		n = (clkinKHz / freq_lo_limit) - 1;
		printk(KERN_WARNING "hdmi_compute_pll: reduced N to %d\n", n);
	} else if (t > freq_hi_limit) {
		printk(KERN_WARNING "hdmi_compute_pll: N too small (%d)\n", n);
		n = (clkinKHz / freq_hi_limit) + 1;
		printk(KERN_WARNING "hdmi_compute_pll: reduced N to %d\n", n);
	}

	/* Passed in */
	pi->regn = n;

	/* CLKOUTLDO is not needed, so m2 is 1 */
	pi->regm2 = 1;

	/* Cached multiple for math below */
	phy_x_nplus1 = phyKHz * (n + 1);
	/* m = (((phy) * (n+1)) / clkin)
	 * The m register does a /10, so we need a value 10 times larger here.
	 * In order to maintain the most significant digits, the clkin value
	 * is divide by 10 here which results in a value 10 times larger.
	 */
	pi->regm = ((10 * phy_x_nplus1) / clkinKHz);
	/* mf (fractional part) = (remainder from regm) * (2^18) / clkin
	 *
	 * Unfortunately, there is a potential of overflowing a u32 if we mult
	 * by (2^18).  So, instead, the remainder will mult'd by (2^8) prior to
	 * the div by clkin and finally mult'd by (2^10) to ensure the right
	 * magnitude is maintained.  This method still results in a offset
	 * from the desired value, so to get very precise, the remainder from
	 * this division is mult'd by (2^10), divided, and that result is
	 * added to create the final very precise value.
	 *
	 * mf  = (((remainder of regm) << 8) / clkin) << 10
	 * mf += ((remainder of above) << 10) / clkin
	 */
	t = ((10 * phy_x_nplus1) % clkinKHz);
	pi->regmf = ((t << 8) / clkinKHz) << 10;
	t = ((t << 8) % clkinKHz);
	pi->regmf += (t << 10) / clkinKHz;

	/* If (HDMI 1080P or DVI?) use DCO FREQ */
	if (phyKHz > dco_freq_limit) {
		/* Use DCO FREQ */
		pi->dcofreq = 1;
		/* Ceiling of (((regm * clkin) / (n+1)) / 256) + 0.5 */
		pi->regsd = ((((pi->regm * clkinKHz)/(n + 1))/256) + 500)/1000;
	} else {
		/* Do not use DCO FREQ */
		pi->dcofreq = 0;
		pi->regsd = 0;
	}

	HDTVDBG("N/%d; M/%d; Mf/%d; DCO/%d; SD/%d;\n",
		pi->regn, pi->regm, pi->regmf, pi->dcofreq, pi->regsd);
}

static int hdmi_pll_init(struct hdmi_pll_info *fmt)
{
	u32 cfg1, cfg2, cfg3, cfg4;
	u32 pll = HDMI_PLLCTRL;
	u32 dco;
	u32 sd;
	unsigned t = 500000;

	cfg1 = 0;
	cfg2 = 0;
	cfg3 = 0;
	cfg4 = 0;

	cfg2 = FLD_MOD(cfg2, 0x1, 13, 13); /* PLL_REFEN */
	hdmi_write_reg(pll, PLLCTRL_CFG2, cfg2);

	/* Config Reg N first */
	cfg1 = FLD_MOD(cfg1, fmt->regn, 8, 1);
	hdmi_write_reg(pll, PLLCTRL_CFG1, cfg1);

	/* Config SELFREQDCO */
	if (fmt->dcofreq) {
		/* Divider programming for 1080p */
		dco = 0x4;  /* 1000MHz and 2000MHz */
		sd  = fmt->regsd;
	} else {
		dco = 0x2;  /* 500MHz and 1000MHz */
		sd  = 0;
	}
	cfg3 = FLD_MOD(cfg3, sd, 17, 10);
	hdmi_write_reg(pll, PLLCTRL_CFG3, cfg3);
	cfg2 = FLD_MOD(cfg2, dco, 3, 1);
	hdmi_write_reg(pll, PLLCTRL_CFG2, cfg2);

	/* Config Reg M, M2, & MF */
	cfg1 = FLD_MOD(cfg1, fmt->regm, 20, 9);
	hdmi_write_reg(pll, PLLCTRL_CFG1, cfg1);
	cfg4 = FLD_MOD(cfg4, fmt->regm2, 24, 18);
	cfg4 = FLD_MOD(cfg4, fmt->regmf, 17, 0);
	hdmi_write_reg(pll, PLLCTRL_CFG4, cfg4);

	/* Go now */
	REG_FLD_MOD(pll, PLLCTRL_PLL_GO, 0x1ul, 0, 0);

	/* Wait for bit change */
	while (FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_GO), 0, 0))
		udelay(1);

	/* Wait till the lock bit is set */
	while (0 == FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_STATUS), 1, 1)) {
		udelay(1);
		if (!--t) {
			printk(KERN_WARNING "HDMI: cannot lock PLL\n");
			HDTVDBG("PLL CFG1/%x; CFG2/%x; CFG3/%x; CFG4/%x;\n",
				hdmi_read_reg(pll, PLLCTRL_CFG1),
				hdmi_read_reg(pll, PLLCTRL_CFG2),
				hdmi_read_reg(pll, PLLCTRL_CFG3),
				hdmi_read_reg(pll, PLLCTRL_CFG4));
			return -EIO;
		}
	}

	HDTVDBG("PLL is locked\n");

	return 0;
}

static int hdmi_pll_reset(void)
{
	int t = 0;

	/* SYSRESET controlled by power FSM */
	REG_FLD_MOD(HDMI_PLLCTRL, PLLCTRL_PLL_CONTROL, 0x0, 3, 3);

	/* READ 0x0 reset is in progress */
	while (!FLD_GET(hdmi_read_reg(HDMI_PLLCTRL, PLLCTRL_PLL_STATUS),
								0, 0)) {
		udelay(1);
		if (t++ > 1000) {
			ERR("Failed to sysrest PLL\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int hdmi_pll_start(int pclk_khz)
{
	struct hdmi_pll_info pll_data;
	u32 rc;

	/* FIXME: 'n' should to be auto-determined?
	 * "15" is correct for 38.4MHz
	 * "10" is correct for 26MHz
	 */
	hdmi_pll_compute(gdev.clkin_khz, pclk_khz, 10, &pll_data);

	/* Power off PLL */
	rc = HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
	if (rc)
		goto failed;

	/* Power on PLL */
	rc = HDMI_W1_SetWaitPllPwrState(HDMI_WP,
					HDMI_PLLPWRCMD_BOTHON_ALLCLKS);
	if (rc)
		goto failed;

	hdmi_pll_reset();

	rc = hdmi_pll_init(&pll_data);

failed:
	return rc;
}

static void hdmi_pll_stop(void)
{
	HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
}

/*===========================================================================
 * PHY related functions
 */

static int hdmi_phy_start(int pclk_khz, u8 ds_percent)
{
	int r;
	int tmds_freq = 0;

	/* wait till PHY_PWR_STATUS=LDOON */
	/* HDMI_PHYPWRCMD_LDOON = 1 */
	r = HDMI_W1_SetWaitPhyPwrState(HDMI_WP, 1);
	if (r)
		return r;

	/* wait till PHY_PWR_STATUS=TXON */
	r = HDMI_W1_SetWaitPhyPwrState(HDMI_WP, 2);
	if (r) {
		HDMI_W1_SetWaitPhyPwrState(HDMI_WP, 0);
		return r;
	}

	/* read address 0 in order to get the SCPreset done completed */
	/* Dummy access performed to solve resetdone issue */
	hdmi_read_reg(HDMI_PHY, HDMI_TXPHY_TX_CTRL);

	/* write to phy address 0 to configure the clock */
	/* TMDS freq_out in the PHY should be set based on the TMDS clock */
	if (pclk_khz <= 50000)
		tmds_freq = 0x0;
	else if (pclk_khz <= 100000)
		tmds_freq = 0x1;
	else
		tmds_freq = 0x2;

	REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_TX_CTRL, tmds_freq, 31, 30);

	/* write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(HDMI_PHY, HDMI_TXPHY_DIGITAL_CTRL, 0xF0000000);

	/* Flip polarity on channels as needed */
	if (gdev.inv_lane_clk)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 27, 27);
	if (gdev.inv_lane_d2)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 28, 28);
	if (gdev.inv_lane_d1)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 29, 29);
	if (gdev.inv_lane_d0)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 30, 30);

	/* Set the appropriate drive strength */
	if (ds_percent > 75)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_TX_CTRL, 0, 2, 1);
	else if (ds_percent > 50)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_TX_CTRL, 3, 2, 1);
	else if (ds_percent > 25)
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_TX_CTRL, 2, 2, 1);
	else
		REG_FLD_MOD(HDMI_PHY, HDMI_TXPHY_TX_CTRL, 1, 2, 1);

	return 0;
}

static void hdmi_phy_stop(void)
{
	/* wait till PHY_PWR_STATUS=OFF */
	/* HDMI_PHYPWRCMD_OFF = 0 */
	HDMI_W1_SetWaitPhyPwrState(HDMI_WP, 0);
}

/*===========================================================================
 * Output related functions
 */

static int hdmi_output_start(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *p;
	int rc;

	if (!gdev.attached) /* Must be attached */
		return -EINVAL;

	if (gdev.out_en)
		return 0; /* Already started */

	HDTVDBG("Output Start\n");

	if (!hdmi_is_timing_set()) {
		hdmi_determine_timings(dssdev);
		omap_dss_notify(dssdev, OMAP_DSS_SIZE_CHANGE);
	}

	/* Setup this field only prior to starting the output */
	gdev.cfg.supports_ai = hdmi_is_ai_supported();

	p = &dssdev->panel.timings;

	/* Set the L3 bus throughput to 200MHz to reduce flicker */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 800000);

	/* Wait for wrapper reset */
	HDMI_W1_SetWaitSoftReset();

	/* Config the PLL and PHY first */
	rc = hdmi_pll_start(p->pixel_clock);
	if (rc) {
		printk(KERN_ERR "HDMI OutEn: PLL Program Failed\n");
		rc = -EIO;
		goto failed_pll;
	}

	rc = hdmi_phy_start(p->pixel_clock, dssdev->phy.hdmi.ds_percent);
	if (rc) {
		printk(KERN_ERR "HDMI OutEn: Init PHY Failed\n");
		rc = -EIO;
		goto failed_phy;
	}

	hdmi_lib_enable(&gdev.cfg); /* FIXME: chk return code */

	/* Alert audio of this event */
	hdmi_notify_pwrchange(HDMI_EVENT_POWERON);

	/* Configure the limited or full range */
	hdmi_configure_lr_fr();

	/* These settings are independent of overlays */
	dss_switch_tv_hdmi(1);

	/* By-pass TV gamma table*/
	dispc_enable_gamma_table(0);

	/* Allow idle mode */
	dispc_set_idle_mode();

#ifndef CONFIG_OMAP4_ES1
	/* The default reset value for DISPC.DIVISOR1 LCD is 4
	 * in ES2.0 and the clock will run at 1/4th the speed
	 * resulting in the sync_lost_digit
	 */
	dispc_set_tv_divisor();
#endif

	/* Set the TV size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	rc = HDMI_W1_StartVideoFrame(HDMI_WP);
	if (rc) {
		printk(KERN_ERR "HDMI OutEn: Start Failed\n");
		rc = -EIO;
		goto failed_start;
	}

	dispc_enable_digit_out(1);
	gdev.out_en = 1;

	return 0;

failed_start:
	/* The PHY is not stopped since it is still needed for detection */
failed_phy:
	hdmi_pll_stop();
failed_pll:
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, -1);
	return rc;
}

static void hdmi_output_stop(void)
{
	if (!gdev.out_en)
		return; /* Already stopped */

	HDTVDBG("Output Stop\n");

	gdev.out_en = 0;
	dispc_enable_digit_out(0);
	HDMI_W1_StopVideoFrame(HDMI_WP);
	omap_pm_set_min_bus_tput(&gdev.dssdev->dev, OCP_INITIATOR_AGENT, -1);

	/* Alert audio of this event */
	hdmi_notify_pwrchange(HDMI_EVENT_POWEROFF);

	/* The PHY & PLL are not stopped, still needed for detection */
}

/*===========================================================================
 * Cable Detection related functions
 */

static int hdmi_detect_start(struct omap_dss_device *dssdev)
{
	int rc = 0;

	if (gdev.det_en)
		return 0; /* Already started */

	HDTVDBG("Detect Start\n");

	/* Perform the platform setup (power supplies, etc.) */
	if (dssdev->platform_enable_hpd)
		dssdev->platform_enable_hpd(dssdev);

	/* Enable the hot-plug interrupt */
	rc = hdmi_irq_hpd_enable(1);
	if (rc)
		goto failed_irq;

	gdev.det_en = 1;

	/* Force an initial test of the HPD state */
	hdmi_irq_force_chk();

	return 0;

failed_irq:
	if (dssdev->platform_disable_hpd)
		dssdev->platform_disable_hpd(dssdev);

	return rc;
}

static void hdmi_detect_stop(struct omap_dss_device *dssdev)
{
	if (!gdev.det_en)
		return; /* Already stopped */

	HDTVDBG("Detect Stop\n");

	gdev.det_en = 0;

	hdmi_irq_hpd_enable(0);

	if (dssdev->platform_disable_hpd)
		dssdev->platform_disable_hpd(dssdev);
}

/*===========================================================================
 * Cable Attach related functions
 */

static int hdmi_attach_start(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *p;
	int rc = 0;

	if (!gdev.det_en) /* Detect must be enabled */
		return -EINVAL;

	if (gdev.attached)
		return 0; /* Already started */

	HDTVDBG("Attach Start\n");

	/* Make sure the EDID is re-read */
	gdev.edid_valid = false;

	/* Perform the platform setup (power supplies, etc.) */
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	rc = omap_dss_start_device(dssdev);
	if (rc) {
		printk(KERN_ERR "HDMI start device failed\n");
		goto failed_start;
	}

	/* The output is not truly going to be activated, but we need to place
	 * the DSS device into the active state in order to properly handle
	 * suspend/resume cases.  While this appears to be a necessary step,
	 * it does have some side effects.  For instances, a new "enable"
	 * function had to be added, "smart_enable", in order to ignore the
	 * state when enable is called.  Also, a few places in the DSS2 code
	 * have to check for an HDMI device and by-pass some precessing.
	 * Ideally another state would be added to relect this case, but that
	 * would require a significant effort, so it's postponed for now.
	 */
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	dss_mainclk_state_enable();
	hdmi_opt_clock_enable();

	rc = hdmi_irq_connect_enable(1);
	if (rc)
		goto failed_irq;

	/* The PLL has to be running in order to read the EDID */
	/* Force a low clock rate to reduce current draw */
	gdev.mode = OMAP_DSS_EDID_TIMING_MODE_CEA;
	gdev.code = OMAP_DSS_EDID_TIMING_CEA_640x480_60Hz;
	gdev.client_timing = false;
	hdmi_determine_timings(dssdev);
	p = &dssdev->panel.timings;

	/* Wait for wrapper reset */
	HDMI_W1_SetWaitSoftReset();

	/* Config the PLL and PHY first */
	rc = hdmi_pll_start(p->pixel_clock);
	if (rc) {
		printk(KERN_ERR "HDMI Attach: PLL Program Failed\n");
		rc = -EIO;
		goto failed_pll;
	}

	rc = hdmi_phy_start(p->pixel_clock, 25);
	if (rc) {
		printk(KERN_ERR "HDMI Attach: Init PHY Failed\n");
		goto failed_phy;
	}

	hdmi_lib_enable(&gdev.cfg);  /* FIXME: chk return code */
	hdmi_set_irqs(0);

	gdev.attached = 1;
	hdmi_set_hot_plug_status(dssdev, true);

	return 0;

failed_phy:
	hdmi_pll_stop();
failed_pll:
	hdmi_irq_connect_enable(0);
failed_irq:
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	hdmi_opt_clock_disable();
	dss_mainclk_state_disable(true);
failed_start:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return rc;
}

static void hdmi_attach_stop(struct omap_dss_device *dssdev)
{
	if (!gdev.attached)
		return; /* Already stopped */

	HDTVDBG("Attach Stop\n");

	/* Do the state change first since some calls below depend on it */
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	hdmi_set_irqs(1);

	hdmi_set_hot_plug_status(dssdev, false);
	gdev.attached = 0;

	/* Allow audio to fully shutdown */
	msleep(100);

	hdmi_phy_stop();

	hdmi_pll_stop();

	hdmi_irq_connect_enable(0);

	hdmi_opt_clock_disable();
	dss_mainclk_state_disable(true);

	omap_dss_stop_device(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* Make sure the EDID is not looked at */
	gdev.edid_valid = false;
}

/*===========================================================================
 * Hot-plug IRQ related functions
 */

bool hdmi_irq_get_hpd(void)
{
	const int debounce_delay = 5; /* 5ms */
	const int debounce_cnt   = 5; /* delay * cnt = 25ms */

	int cnt = 40;   /* max debounce of 200ms */
	int debounce = debounce_cnt;
	bool prev = false;
	bool new = false;

	while (cnt--) {
		debounce--;
		new = (gpio_get_value(gdev.hpd_gpio)) ? true : false;
		if (new != prev)
			debounce = debounce_cnt;
		else if (debounce <= 0)
			break;

		prev = new;
		mdelay(debounce_delay);
	}
	HDTVDBG("HPD Debounce (%d/%d)\n", new, cnt);

	return new;
}

void hdmi_irq_work_queue(struct work_struct *work)
{
	struct omap_dss_device *dssdev;
	unsigned long flags;
	int stat;
	bool hpd;

	spin_lock_irqsave(&gdev.irqstatus_lock, flags);

	/* gdev.dssdev has to be valid or init would have failed */
	dssdev = gdev.dssdev;
	hpd    = false;

	gdev.working = 1;

	while (1) {
		/* Get the current status */
		stat = gdev.irqstatus;
		gdev.irqstatus = 0;

		/* Exit if there is not action */
		if (stat == 0)
			break;

		spin_unlock_irqrestore(&gdev.irqstatus_lock, flags);

		/* Filter connects and update connected */
		if (stat & HDMI_STATUS_CONNECT) {
			if (gdev.connected)
				stat &= ~HDMI_STATUS_CONNECT;
			gdev.connected = true;
		}
		/* Filter disconnects and update connected */
		if (stat & HDMI_STATUS_DISCONNECT) {
			if (!gdev.connected)
				stat &= ~HDMI_STATUS_DISCONNECT;
			gdev.connected = false;
		}
		/* Debounce and filter HPD */
		if (stat & HDMI_STATUS_HOTPLUG) {
			hpd = hdmi_irq_get_hpd();
			if (hpd == gdev.attached)
				stat &= ~HDMI_STATUS_HOTPLUG;
		}

		HDTVDBG("IRQ: stat/%d; state/%d;\n", stat, dssdev->state);

		if (stat != 0) {

			mutex_lock(&gdev.lock);

			/* FIXME: maybe DISCONNECT also? */
			if ((stat & HDMI_STATUS_HOTPLUG) && !hpd) {
				if (gdev.out_en)
					hdmi_output_stop();

				hdmi_attach_stop(dssdev);
			}

			if ((stat & HDMI_STATUS_HOTPLUG) && hpd)
				hdmi_attach_start(dssdev);

			mutex_unlock(&gdev.lock);

		}

		spin_lock_irqsave(&gdev.irqstatus_lock, flags);
	}

	gdev.working = 0;

	spin_unlock_irqrestore(&gdev.irqstatus_lock, flags);
}

static void hdmi_irq_common_handler(int stat)
{
	unsigned long flags;

	spin_lock_irqsave(&gdev.irqstatus_lock, flags);

	/* Make sure hdmi_irq_work_queue() exited, if it was going to */
	if (!gdev.working)
		mdelay(1);

	/* If new DISCONNECT, remove any previous CONNECT status */
	if (stat & HDMI_STATUS_DISCONNECT)
		gdev.irqstatus &= ~HDMI_STATUS_CONNECT;
	/* If new CONNECT, remove any previous DISCONNECT status */
	if (stat & HDMI_STATUS_CONNECT)
		gdev.irqstatus &= ~HDMI_STATUS_DISCONNECT;
	/* Add the new status */
	gdev.irqstatus |= stat;

	HDTVDBG("IRQ: stat/%d; det_en/%d;\n", stat, gdev.det_en);

	if (!gdev.working && gdev.det_en)
		queue_work(gdev.workqueue, &gdev.work);

	spin_unlock_irqrestore(&gdev.irqstatus_lock, flags);
}

static irqreturn_t hdmi_irq_connect(int irq, void *arg)
{
	int irq_s = 0;
	int stat = 0;

	HDMI_W1_HPD_handler(&irq_s);
	if (irq_s & HDMI_CONNECT)
		stat = HDMI_STATUS_CONNECT;
	if (irq_s & HDMI_DISCONNECT)
		stat = HDMI_STATUS_DISCONNECT;

	HDTVDBG("HDMI CONNECT IRQ (%d)\n", stat);

	if (stat != 0)
		hdmi_irq_common_handler(stat);

	return IRQ_HANDLED;
}

static irqreturn_t hdmi_irq_hpd(int irq, void *arg)
{
	HDTVDBG("HDMI HPD IRQ\n");

	hdmi_irq_common_handler(HDMI_STATUS_HOTPLUG);

	return IRQ_HANDLED;
}

static int hdmi_irq_init(void)
{
	int rc = 0;

	/* OMAP4 hardcodes, not platform specific */
	gdev.hpd_gpio = 63;
	gdev.hpd_irq  = gpio_to_irq(gdev.hpd_gpio);

	rc = gpio_request(gdev.hpd_gpio, "HDMI_HPD_INT");
	if (rc) {
		printk(KERN_ERR "HDMI HPD request failed\n");
		goto failed_gpio;
	}
	gpio_direction_input(gdev.hpd_gpio);
	set_irq_type(gdev.hpd_irq, IRQ_TYPE_EDGE_BOTH);

failed_gpio:
	return rc;
}

static void hdmi_irq_term(void)
{
	gpio_free(gdev.hpd_gpio);
}

static int hdmi_irq_connect_enable(int en)
{
	int rc = 0;

	if (en) {
		rc = request_irq(OMAP44XX_IRQ_DSS_HDMI, hdmi_irq_connect,
						0, "OMAP HDMI", (void *)0);
		if (rc) {
			printk(KERN_ERR "HDMI request IRQ failed\n");
		}
	} else {
		free_irq(OMAP44XX_IRQ_DSS_HDMI, NULL);
	}

	return rc;
}

static int hdmi_irq_hpd_enable(int en)
{
	int rc = 0;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	if (en) {
		rc = request_irq(gdev.hpd_irq, hdmi_irq_hpd,
					flags, "OMAP HDMI HPD", (void *)0);

		if (rc) {
			printk(KERN_ERR "HDMI HPD request IRQ failed\n");
		}
	} else {
		free_irq(gdev.hpd_irq, NULL);
	}

	return rc;
}

static void hdmi_irq_force_chk(void)
{
	HDTVDBG("HDMI HPD Force Check\n");

	hdmi_irq_common_handler(HDMI_STATUS_HOTPLUG);
}

/*===========================================================================
 * Panel driver related functions
 */

static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	int idx;

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;

	idx = get_timings_index();

	dssdev->panel.timings = video_timing_table[idx];

	HDTVDBG("HDMI Probe: x/%d; y/%d;\n",
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res);

	mdelay(50);

	return 0;
}

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
}

static bool hdmi_panel_is_enabled(struct omap_dss_device *dssdev)
{
	return gdev.out_en;
}

static bool hdmi_panel_is_detected(struct omap_dss_device *dssdev)
{
	HDTVDBG("attached=%d, connected=%d, out_en=%d\n",
			gdev.attached, gdev.connected, gdev.out_en);
	return gdev.attached;
}

static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	int rc = 0;

	HDTVDBG("HDMI En Panel\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		return 0;

	mutex_lock(&gdev.lock);

	if (!gdev.attached) {
		HDTVDBG("No Panel Attached\n");
		rc = -EIO;
		goto failed;
	}

	if (gdev.out_en) {
		HDTVDBG("Panel Already Enabled\n");
		rc = -EBUSY;
		goto failed;
	}

	rc = hdmi_output_start(dssdev);

failed:
	mutex_unlock(&gdev.lock);

	return rc;
}

static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	HDTVDBG("HDMI Dis Display\n");

	mutex_lock(&gdev.lock);

	if (gdev.out_en)
		hdmi_output_stop();

	mutex_unlock(&gdev.lock);
}

static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	HDTVDBG("HDMI Suspend\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	mutex_lock(&gdev.lock);

	if (gdev.out_en) {
		HDTVDBG("Suspend while output active\n");
		hdmi_output_stop();
	}

	if (gdev.attached) {
		HDTVDBG("Suspend while attached\n");
		hdmi_attach_stop(dssdev);
	}

	if (gdev.det_en) {
		HDTVDBG("Suspend while detection active\n");
		gdev.det_resume = true;
		hdmi_detect_stop(dssdev);
	}

	mutex_unlock(&gdev.lock);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	HDTVDBG("HDMI Resume\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	mutex_lock(&gdev.lock);

	if (gdev.det_resume) {
		HDTVDBG("Resuming detection\n");
		gdev.det_resume = false;
		hdmi_detect_start(dssdev);
	}

	mutex_unlock(&gdev.lock);

	return 0;
}

static struct omap_dss_driver hdmi_driver = {
	.probe			= hdmi_panel_probe,
	.remove			= hdmi_panel_remove,

	.disable		= hdmi_panel_disable,
	.smart_enable		= hdmi_panel_enable,
	.smart_is_enabled	= hdmi_panel_is_enabled,
	.is_detected		= hdmi_panel_is_detected,
	.suspend		= hdmi_panel_suspend,
	.resume			= hdmi_panel_resume,
	.get_timings		= hdmi_get_timings,
	.set_timings		= hdmi_set_timings,
	.check_timings		= hdmi_check_timings,
	.get_edid		= hdmi_get_edid,
	.dump_edid		= hdmi_dump_edid,
	.set_edid_timing	= hdmi_set_edid_timing,
	.hpd_enable		= hdmi_enable_hpd,

	.enable_s3d		= hdmi_enable_s3d,
	.get_s3d_enabled	= hdmi_get_s3d_enabled,
	.set_s3d_disp_type	= hdmi_set_s3d_disp_type,

	.driver	= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};

int hdmi_set_audio_power(bool _audio_on)
{
	/* XXX stub */
	return -ENODEV;
}

static int hdmi_enable_hpd(struct omap_dss_device *dssdev, bool enable)
{
	int rc = 0;

	HDTVDBG("HDMI Enable HPD(%d/%d)\n", enable, gdev.det_en);

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		dssdev->activate_after_resume = enable;

	if (gdev.det_en == enable)
		return 0;

	if (enable) {
		mutex_lock(&gdev.lock);

		rc = hdmi_detect_start(dssdev);
		if (rc)
			printk(KERN_ERR "HDMI Enable HPD Failed\n");

		mutex_unlock(&gdev.lock);
	} else {
		mutex_lock(&gdev.lock);

		if (gdev.out_en)
			hdmi_output_stop();

		if (gdev.attached)
			hdmi_attach_stop(dssdev);

		hdmi_detect_stop(dssdev);

		/* Make sure the detect resume flag is disabled */
		gdev.det_resume = false;

		mutex_unlock(&gdev.lock);
	}

	return rc;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

/* convert timings to code/mode.. return error if invalid timings, otherwise
 * code/mode returned by reference
 */
static int timings2code(struct omap_video_timings *timing,
		int *mode, int *code)
{
	int i = 0, temp_vsync = 0, temp_hsync = 0;
	int timing_vsync = 0, timing_hsync = 0;
	const struct omap_video_timings *temp;

	for (i = 0; i < TIMING_TABLE_SIZE; i++) {
		temp = &video_timing_table[i];
		if (temp->pixel_clock != timing->pixel_clock ||
		    temp->x_res != timing->x_res ||
		    temp->y_res != timing->y_res)
			continue;

		/* not sure if we need to keep this fuzzy logic.. it was in
		 * original hdmi driver, and supposedly helped with some certain
		 * model of monitor.. which I don't have to confirm whether or
		 * not that is indeed true.  I would not be heartbroken if this
		 * was removed and changed back to a memcmp()
		 */
		temp_hsync = temp->hfp + temp->hsw + temp->hbp;
		timing_hsync = timing->hfp + timing->hsw + timing->hbp;
		temp_vsync = temp->vfp + temp->vsw + temp->vbp;
		timing_vsync = timing->vfp + timing->vsw + timing->vbp;

		DSSDBG("Temp_hsync = %d, temp_vsync = %d, "
			"timing_hsync = %d, timing_vsync = %d",
			temp_hsync, temp_hsync, timing_hsync, timing_vsync);

		if (temp_hsync == timing_hsync && temp_vsync == timing_vsync) {
			if (mode) {
				*mode = i >= VESA_TIMING_IDX_START ?
						OMAP_DSS_EDID_TIMING_MODE_VESA :
						OMAP_DSS_EDID_TIMING_MODE_CEA;
			}
			if (code)
				*code = code_index[i];
			return 0;
		}
	}
	return -EINVAL;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	int mode, code;

	if (timings2code(timings, &mode, &code) == 0) {
		hdmi_set_edid_timing(dssdev, mode, code);
	} else {
		gdev.client_timing = true;
		memcpy(&gdev.timing, timings, sizeof(gdev.timing));
		if (timings->sp & 0x80) /* Force DVI audio? */
			gdev.mode = OMAP_DSS_EDID_TIMING_MODE_CEA;
		else
			gdev.mode = OMAP_DSS_EDID_TIMING_MODE_VESA;
		gdev.code = 0;

		if (gdev.out_en) {
			/* Stop and restart the output to use the new values */
			hdmi_output_stop();
			hdmi_output_start(dssdev);
		} else {
			/* Even if output is not enabled yet, we should update
			 * the timings, otherwise other things that depend on
			 * the resolution that is being set (ie. check_overlay)
			 * will fail
			 */
			hdmi_determine_timings(dssdev);
			omap_dss_notify(dssdev, OMAP_DSS_SIZE_CHANGE);
		}
	}
}

static int hdmi_set_edid_timing(struct omap_dss_device *dssdev,
				enum omap_dss_edid_timing_mode mode, int code)
{
	HDTVDBG("HDMI Set EDID Timing (%d/%d)\n", mode, code);

	if (!is_mode_code_valid(mode, code))
		return -1;

	mutex_lock(&gdev.lock);

	if (gdev.client_timing || mode != gdev.mode || code != gdev.code) {
		gdev.client_timing = false;
		gdev.mode = mode;
		gdev.code = code;

		if (gdev.out_en) {
			/* Stop and restart the output to use the new values */
			hdmi_output_stop();
			hdmi_output_start(dssdev);
		} else {
			/* Even if output is not enabled yet, we should update
			 * the timings, otherwise other things that depend on
			 * the resolution that is being set (ie. check_overlay)
			 * will fail
			 */
			hdmi_determine_timings(dssdev);
			omap_dss_notify(dssdev, OMAP_DSS_SIZE_CHANGE);
		}
	}

	mutex_unlock(&gdev.lock);

	return 0;
}

static void hdmi_dump_edid(struct omap_dss_device *dssdev)
{
	/* Do nothing here, but the function needs to exist so that the
	 * DSS Mgr code can determine if EDID's are supported by the display
	 */
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	return timings2code(timings, NULL, NULL);
}

static int hdmi_get_edid(struct omap_dss_device *dssdev, u8 *edid, int len)
{
	int rc = -1;
	int size = HDMI_EDID_MAX_LENGTH;

	if (edid == NULL) {
		HDTVDBG("hdmi_get_edid: Invalid edid pointer\n");
	} else if (!gdev.attached) {
		HDTVDBG("hdmi_get_edid: Cable not attached\n");
	} else if (gdev.edid_valid) {
		size = (len > gdev.edid_len) ? gdev.edid_len : len;
		memcpy(edid, gdev.edid, size);
		rc = 0;
	} else {
		rc = hdmi_perform_edid_read(edid, len);
		if (rc == 0) {
			size = (len > size) ? size : len;
			memcpy(gdev.edid, edid, size);
			gdev.edid_len = size;
			hdmi_validate_edid();
		}
	}

	return rc;
}

static int hdmi_set_s3d_disp_type(struct omap_dss_device *dssdev,
						struct s3d_disp_info *info)
{
	int r = -EINVAL;
	struct hdmi_s3d_info tinfo;

	tinfo.structure = 0;
	tinfo.subsamp = false;
	tinfo.subsamp_pos = 0;

	printk(KERN_INFO "set s3d\n");

	switch (info->type) {
	case S3D_DISP_OVERUNDER:
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_NONE) {
			tinfo.structure = HDMI_S3D_FRAME_PACKING;
			r = 0;
		} else {
			goto err;
		}
		break;
	case S3D_DISP_SIDEBYSIDE:
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_H) {
			tinfo.structure = HDMI_S3D_SIDE_BY_SIDE_HALF;
			tinfo.subsamp = true;
			tinfo.subsamp_pos = HDMI_S3D_HOR_EL_ER;
			r = 0;
		} else {
			goto err;
		}
		break;
	default:
		goto err;
	}
	hdmi_s3d = tinfo;
err:
	return r;
}

static int hdmi_enable_s3d(struct omap_dss_device *dssdev, bool enable)
{
	int r = -EINVAL;

	printk(KERN_INFO "enable_s3d (%d)\n", enable);
	if (enable == true) {
		/*enable format*/
		hdmi_output_stop();
		gdev.s3d_enabled = true;
		r = hdmi_output_start(dssdev);
		if (r == 0 && gdev.s3d_enabled == true) {
			switch (hdmi_s3d.structure) {
			case HDMI_S3D_FRAME_PACKING:
				dssdev->panel.s3d_info.type =
							S3D_DISP_OVERUNDER;
				dssdev->panel.s3d_info.gap = 30;
				if (hdmi_s3d.subsamp == true)
					dssdev->panel.s3d_info.sub_samp =
							S3D_DISP_SUB_SAMPLE_V;
				break;
			case HDMI_S3D_SIDE_BY_SIDE_HALF:
				dssdev->panel.s3d_info.type =
							S3D_DISP_SIDEBYSIDE;
				dssdev->panel.s3d_info.gap = 0;
				if (hdmi_s3d.subsamp == true)
					dssdev->panel.s3d_info.sub_samp =
							S3D_DISP_SUB_SAMPLE_H;
				break;
			default:
				r = -EINVAL;
				break;
			}
			dssdev->panel.s3d_info.order = S3D_DISP_ORDER_L;
		}
	} else {
		gdev.s3d_enabled   = false;
		gdev.client_timing = false;
		gdev.mode = gdev.def_mode;
		gdev.code = gdev.def_code;

		hdmi_output_stop();
		r = hdmi_output_start(dssdev);
	}

	return r;
}

static bool hdmi_get_s3d_enabled(struct omap_dss_device *dssdev)
{
	return gdev.s3d_enabled;
}

/*===========================================================================
 * Used in the drivers/media/video/omap/omap_vout.c file
 *
 * Clean up?
 */

bool is_hdmi_interlaced(void)
{
	return gdev.cfg.interlace;
}

/*===========================================================================
 * omap_dss_device - exposed in dss.h
 */

int hdmi_init(struct platform_device *pdev)
{
	struct omap_dss_device *dev = NULL;
	struct omap_dss_board_info *info;
	struct resource *hdmi_mem;
	int rc = 0;
	int i;
	int idx;

	HDTVDBG("HDMI Init\n");

	memset(&gdev, 0, sizeof(gdev));
	mutex_init(&gdev.lock);
	spin_lock_init(&gdev.irqstatus_lock);
	gdev.pdata = pdev->dev.platform_data;
	gdev.pdev = pdev;

	hdmi_s3d.structure = HDMI_S3D_FRAME_PACKING;
	hdmi_s3d.subsamp = false;
	hdmi_s3d.subsamp_pos = 0;

	if (pdev) {
		info = gdev.pdata->board_data;

		for (i = 0; i < info->num_devices; i++) {
			dev = info->devices[i];
			if (dev->type == OMAP_DISPLAY_TYPE_HDMI)
				break;
		}

		if (i >= info->num_devices)
			dev = NULL;
	}

	if (!dev) {
		rc = -EINVAL;
		goto failed_dev;
	} else {
		gdev.dssdev       = dev;
		gdev.def_mode     = dev->phy.hdmi.default_mode;
		gdev.def_code     = dev->phy.hdmi.default_code;
		gdev.clkin_khz    = dev->phy.hdmi.clkin_khz;
		gdev.inv_lane_d0  = dev->phy.hdmi.inv_lane_d0;
		gdev.inv_lane_d1  = dev->phy.hdmi.inv_lane_d1;
		gdev.inv_lane_d2  = dev->phy.hdmi.inv_lane_d2;
		gdev.inv_lane_clk = dev->phy.hdmi.inv_lane_clk;
	}

	/* Validate and, if invalid, set some hopefully sane defaults */
	if (!gdev.clkin_khz)
		gdev.clkin_khz = 38400;

	idx = (gdev.mode == OMAP_DSS_EDID_TIMING_MODE_VESA) ?
			code_vesa[gdev.def_code] : code_cea[gdev.def_code];
	if (idx == -1) {
		gdev.def_mode = OMAP_DSS_EDID_TIMING_MODE_CEA;
		gdev.def_code = OMAP_DSS_EDID_TIMING_CEA_1920x1080P_60Hz;
	}

	gdev.mode = gdev.def_mode;
	gdev.code = gdev.def_code;
	gdev.client_timing = false;

	/* Init the drive strength percent to full power */
	dev->phy.hdmi.ds_percent = 100;

	hdmi_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!hdmi_mem) {
		ERR("No resources\n");
		rc = -ENOMEM;
		goto failed_mem;
	}

	gdev.base_pll = ioremap((hdmi_mem->start + 0x200),
						resource_size(hdmi_mem));
	if (!gdev.base_pll) {
		ERR("can't ioremap pll\n");
		rc = -ENOMEM;
		goto failed_pll;
	}
	gdev.base_phy = ioremap((hdmi_mem->start + 0x300), 64);

	if (!gdev.base_phy) {
		ERR("can't ioremap phy\n");
		rc = -ENOMEM;
		goto failed_phy;
	}

	hdmi_enable_clocks(1);
	rc = hdmi_lib_init();
	hdmi_enable_clocks(0);
	if (rc) {
		DSSERR("HDMI: Cound not register character device\n");
		goto failed_lib;
	}

	gdev.workqueue = create_singlethread_workqueue("HDMIWORK");
	if (gdev.workqueue == NULL) {
		DSSERR("HDMI: Workqueue alloc failed\n");
		rc = -ENOMEM;
		goto failed_work;
	}
	INIT_WORK(&gdev.work, hdmi_irq_work_queue);
	gdev.working = 0;

	rc = hdmi_irq_init();
	if (rc) {
		DSSERR("HDMI: IRQ init failed\n");
		goto failed_irq;
	}

	rc = omap_dss_register_driver(&hdmi_driver);
	if (rc) {
		DSSERR("HDMI: Driver register failed\n");
		goto failed_drvreg;
	}

	return 0;

failed_drvreg:
	hdmi_irq_term();
failed_irq:
	flush_workqueue(gdev.workqueue);
	destroy_workqueue(gdev.workqueue);
failed_work:
	hdmi_lib_exit();
failed_lib:
	iounmap(gdev.base_phy);
failed_phy:
	iounmap(gdev.base_pll);
failed_pll:
failed_mem:
failed_dev:
	return rc;
}

void hdmi_exit(void)
{
	omap_dss_unregister_driver(&hdmi_driver);
	hdmi_irq_term();
	flush_workqueue(gdev.workqueue);
	destroy_workqueue(gdev.workqueue);
	hdmi_lib_exit();
	iounmap(gdev.base_pll);
	iounmap(gdev.base_phy);
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	if (device_create_file(&dssdev->dev, &dev_attr_attached))
		DSSERR("failed to create attached sysfs file\n");

#ifdef CONFIG_DEBUG_FS
	if (device_create_file(&dssdev->dev, &dev_attr_test))
		DSSERR("failed to create test sysfs file\n");
#endif

	return 0;
}

