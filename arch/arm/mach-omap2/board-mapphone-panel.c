/*
 * linux/arch/arm/mach-omap2/board-mapphone-panel.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/omapfb.h>

#include <plat/display.h>
#include <plat/gpio.h>
#include <plat/panel.h>
#include <plat/control.h>

#include <mach/dt_path.h>
#include <asm/prom.h>

#include <plat/mapphones_dsi_panel.h>

#include "pm.h"

#ifdef DEBUG
static unsigned int board_panel_debug;
#define PANELDBG(format, ...) \
	if (board_panel_debug) \
			printk(KERN_DEBUG "board_panel: " format, \
					## __VA_ARGS__)
#else /* DEBUG */
#define PANELDBG(format, ...)
#endif

static bool mapphone_panel_device_read_dt; /* This is by default false */

/* This must be match in the DT */
enum omap_dss_device_disp_intf {
	OMAP_DSS_DISP_INTF_RGB16	= 1,
	OMAP_DSS_DISP_INTF_RGB24	= 2,
	OMAP_DSS_DISP_INTF_MIPI_L4_CM	= 3,
	OMAP_DSS_DISP_INTF_MIPI_VP_CM	= 4,
	OMAP_DSS_DISP_INTF_MIPI_L4_VM	= 5,
	OMAP_DSS_DISP_INTF_MIPI_VP_VM	= 6
};

/* these enum must be matched with MOT DT */
enum omap_dss_device_disp_pxl_fmt {
	OMAP_DSS_DISP_PXL_FMT_RGB565	= 1,
	OMAP_DSS_DISP_PXL_FMT_RGB888	= 5
};
struct regulator *display_regulator;

static int mapphone_panel_enable(struct omap_dss_device *dssdev);
static void mapphone_panel_disable(struct omap_dss_device *dssdev);

static struct omap_video_timings mapphone_panel_timings = {
	.x_res          = 480,
	.y_res          = 854,
	/*.pixel_clock  = 25000,*/
	.hfp            = 0,
	.hsw            = 2,
	.hbp            = 2,
	.vfp            = 0,
	.vsw            = 1,
	.w              = 50,
	.h              = 89,
};

static struct mapphone_dsi_panel_data mapphone_panel_data = {
	.name   = "mapphone",
	.reset_gpio     = 0,
	.use_ext_te	= false,
	.use_esd_check	= true,
	.set_backlight	= NULL,
	.te_support	= true,
	.te_scan_line	= 300,
	.te_type	= OMAP_DSI_TE_MIPI_PHY,
};

static struct omap_dss_device mapphone_lcd_device = {
	.name = "lcd",
	.driver_name = "mapphone-panel",
	.type = OMAP_DISPLAY_TYPE_DSI,
	.data                   = &mapphone_panel_data,
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.div.regn = 13,
	.phy.dsi.div.regm = 170,
	.phy.dsi.div.regm_dispc = 5,
	.phy.dsi.div.regm_dsi = 5,
	.phy.dsi.div.lck_div = 1,
	.phy.dsi.div.pck_div = 4,
	.phy.dsi.div.lp_clk_div = 7,
	.phy.dsi.xfer_mode = OMAP_DSI_XFER_CMD_MODE,
	.platform_enable = mapphone_panel_enable,
	.platform_disable = mapphone_panel_disable,
};

#define DISPL_VOL_SUPPLY_NAME_SIZE      32
/* gpio 96 by default */
static int mapphone_displ_pwr_sup_en = 96;
/* sw5 by default for qHD */
static char mapphone_displ_pwr_sup[DISPL_VOL_SUPPLY_NAME_SIZE] = "sw5";

/* dss powerdomain */
static struct powerdomain *dss_pwrdm;

static int  mapphone_feature_hdmi;
static int  mapphone_hdmi_5v_enable;       /* 0 by default */
static int  mapphone_hdmi_platform_hpd_en; /* 0 by default */
static int  mapphone_hdmi_5v_force_off;    /* 0 by default */
#define HDMI_DAC_REGULATOR_NAME_SIZE  (32)
static char mapphone_hdmi_dac_reg_name[HDMI_DAC_REGULATOR_NAME_SIZE + 1];
struct regulator *mapphone_hdmi_dac_reg;

static int  mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev);
static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev);
static int  mapphone_panel_enable_hpd_hdtv(struct omap_dss_device *dssdev);
static void mapphone_panel_disable_hpd_hdtv(struct omap_dss_device *dssdev);
static int  mapphone_panel_hdtv_test(struct omap_dss_device *dssdev, int level);

static struct omap_dss_device mapphone_hdtv_device = {
	.name                  = "hdmi",
	.driver_name           = "hdmi_panel",
	.type                  = OMAP_DISPLAY_TYPE_HDMI,
	.phy.hdmi.default_mode = OMAP_DSS_EDID_TIMING_MODE_CEA,
	.phy.hdmi.default_code = OMAP_DSS_EDID_TIMING_CEA_1920x1080P_60Hz,
	.phy.hdmi.clkin_khz    = 26000,
	.phy.hdmi.inv_lane_d0  = 1,
	.phy.hdmi.inv_lane_d1  = 1,
	.phy.hdmi.inv_lane_d2  = 1,
	.phy.hdmi.inv_lane_clk = 1,
	.platform_enable       = mapphone_panel_enable_hdtv,
	.platform_disable      = mapphone_panel_disable_hdtv,
	.platform_enable_hpd   = mapphone_panel_enable_hpd_hdtv,
	.platform_disable_hpd  = mapphone_panel_disable_hpd_hdtv,
#ifdef CONFIG_DEBUG_FS
	/* Used as a simple engineering test interface */
	.set_backlight         = mapphone_panel_hdtv_test,
#endif
};

static void mapphone_panel_reset(bool enable)
{
	static bool first_boot = true;

	if (enable) { /* enable panel */
		if (!first_boot) {
			gpio_set_value(mapphone_panel_data.reset_gpio, 0);
			msleep(10);
			gpio_set_value(mapphone_panel_data.reset_gpio, 1);
			msleep(10);
		} else
			first_boot = false;
	} else { /* disable panel */
		gpio_set_value(mapphone_panel_data.reset_gpio, 0);
		msleep(1);
	}
}

static int mapphone_panel_regulator_enable(void)
{
	if (!display_regulator) {
		PANELDBG("Display power supply = %s\n",
				 mapphone_displ_pwr_sup);
		display_regulator = regulator_get(NULL, mapphone_displ_pwr_sup);
		if (IS_ERR(display_regulator)) {
			printk(KERN_ERR "fail to get regulator for displ.\n");
			return PTR_ERR(display_regulator);
		}
	}

	regulator_enable(display_regulator);
	if (mapphone_displ_pwr_sup_en != 0) {
		gpio_set_value(mapphone_displ_pwr_sup_en, 1);
		msleep(10);
	}

	return 0;
}

static void mapphone_panel_regulator_disable(void)
{
	if (mapphone_displ_pwr_sup_en != 0)
		gpio_set_value(mapphone_displ_pwr_sup_en, 0);

	if (display_regulator)
		regulator_disable(display_regulator);
}

static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{       int ret;
	/* change the DSS power state to INACTIVE with LCD on */
	if (dss_pwrdm)
		omap4_set_pwrdm_state(dss_pwrdm, PWRDM_POWER_INACTIVE);

	ret = mapphone_panel_regulator_enable();
	if (!ret)
		mapphone_panel_reset(true);

	return ret;
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	mapphone_panel_reset(false);
	mapphone_panel_regulator_disable();

	/* change the DSS power state to RET with LCD off */
	if (dss_pwrdm)
		omap4_set_pwrdm_state(dss_pwrdm, PWRDM_POWER_RET);
}
static struct omapfb_platform_data mapphone_fb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			{
				.format = OMAPFB_COLOR_ARGB32,
				.format_used = 1,
			},
		},
	},
	.feature_fb1_to_vid2 = 0,
	.num_fbs = 1,
};

static struct omap_dss_device *mapphone_dss_devices[] = {
	&mapphone_lcd_device,
	&mapphone_hdtv_device,
};

static struct omap_dss_board_info mapphone_dss_data = {
	.num_devices = ARRAY_SIZE(mapphone_dss_devices),
	.devices = mapphone_dss_devices,
	.default_device = &mapphone_lcd_device,
};

static int mapphone_dt_get_dsi_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int disp_intf;
	int len = 0;

	PANELDBG("dt_get_dsi_panel_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL)
		return -ENODEV;

	/* Retrieve the panel information */
	panel_prop = of_get_property(panel_node, "dsi_clk_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.clk_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_clk_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.clk_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data1_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data1_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data1_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data1_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data2_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data2_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data2_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data2_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "gpio_reset", NULL);
	if (panel_prop != NULL)
		mapphone_panel_data.reset_gpio = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "displ_pwr_sup", &len);
	if ((panel_prop != NULL) && len) {
		strncpy(mapphone_displ_pwr_sup, (char *)panel_prop,
			sizeof(mapphone_displ_pwr_sup) - 1);
		mapphone_displ_pwr_sup[sizeof(mapphone_displ_pwr_sup) - 1]
			= '\0';
	}

	panel_prop = of_get_property(panel_node, "displ_pwr_sup_en", NULL);
	if (panel_prop != NULL)
		mapphone_displ_pwr_sup_en = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "type", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.panel.panel_id = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regn", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regn = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm3", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm_dispc = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm4", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm_dsi = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lp_clk_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.lp_clk_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.lck_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "pck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.pck_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "disp_intf", NULL);
	if (panel_prop != NULL) {
		disp_intf = *(u8 *)panel_prop;

		if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_CM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_CM))
			mapphone_lcd_device.phy.dsi.xfer_mode =
				OMAP_DSI_XFER_CMD_MODE;
		else if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_VM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_VM))
			mapphone_lcd_device.phy.dsi.xfer_mode =
				OMAP_DSI_XFER_VIDEO_MODE;
		else {
			printk(KERN_ERR "Invalid disp_intf in dt = %d\n",
				disp_intf);
			return -ENODEV;
		}
	}

	of_node_put(panel_node);

	return 0;
}

static int mapphone_dt_get_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int panel_pixel_fmt;
	int pixel_size = 24;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	PANELDBG("mapphone_dt_get_panel_info\n");

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node != NULL) {
		/* Retrieve the panel DSI timing */
		panel_prop = of_get_property(panel_node, "width", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.x_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "height", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.y_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"phy_width_mm", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.w = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"phy_height_mm", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.h = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"te_support", NULL);
		if (panel_prop != NULL)
			panel_data->te_support = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"te_scan_line", NULL);
		if (panel_prop != NULL)
			panel_data->te_scan_line =  *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node, "te_type", NULL);
		if (panel_prop != NULL)
			panel_data->te_type = *(enum omap_dsi_te_type *)
				panel_prop;

		panel_prop = of_get_property(panel_node, "pixel_fmt", NULL);
		if (panel_prop != NULL) {
			panel_pixel_fmt = *(u32 *)panel_prop;
			if (panel_pixel_fmt == OMAP_DSS_DISP_PXL_FMT_RGB888)
				pixel_size = 24;
			else if (panel_pixel_fmt ==
					OMAP_DSS_DISP_PXL_FMT_RGB565)
				pixel_size = 16;
			else {
				printk(KERN_ERR " Invalid panel_pxl_fmt=%d",
							panel_pixel_fmt);
				return -ENODEV;
			}
		}

		of_node_put(panel_node);

		mapphone_lcd_device.ctrl.pixel_size  =  pixel_size;
		mapphone_lcd_device.panel.timings  =  mapphone_panel_timings;

	}

	return panel_node ? 0 : -ENODEV;

}



static int mapphone_dt_get_dsi_vm_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_dsi_vm_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "dsi_timing_hsa", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hsa = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_hfp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hfp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_hbp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hbp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vsa", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vsa = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vfp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vfp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vbp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vbp = *(u16 *)panel_prop;

	of_node_put(panel_node);

	PANELDBG("DT: phy.dsi.vm_timing: hsa=%d hfp=%d \
		hbp=%d vsa=%d vfp=%d vbp=%d\n",
		mapphone_lcd_device.phy.dsi.vm_timing.hsa,
		mapphone_lcd_device.phy.dsi.vm_timing.hfp,
		mapphone_lcd_device.phy.dsi.vm_timing.hbp,
		mapphone_lcd_device.phy.dsi.vm_timing.vsa,
		mapphone_lcd_device.phy.dsi.vm_timing.vfp,
		mapphone_lcd_device.phy.dsi.vm_timing.vbp);

	return 0;
}

static int mapphone_dt_get_hdtv_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	struct omap_ovl2mgr_mapping *read_ovl2mgr_mapping = NULL;
	int len = 0, i = 0;

	PANELDBG("dt_get_hdtv_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "max_width", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_device.panel.timings.x_res = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "max_height", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_device.panel.timings.y_res = *(u32 *)panel_prop;

	if (mapphone_hdtv_device.panel.timings.x_res > 2048)
		mapphone_hdtv_device.panel.timings.x_res = 1920;

	if (mapphone_hdtv_device.panel.timings.y_res > 2048)
		mapphone_hdtv_device.panel.timings.x_res = 1080;

	/* Get the mapping data for the overlay to map to its manager*/
	panel_prop = of_get_property(panel_node, "overlay_mgr_mapping", &len);
	mapphone_dss_data.ovl2mgr_mapping_cnt = 0;

	if (len)
		mapphone_dss_data.ovl2mgr_mapping_cnt = len /
			sizeof(struct omap_ovl2mgr_mapping);

	if (panel_prop != NULL) {
		read_ovl2mgr_mapping =
			(struct omap_ovl2mgr_mapping *)panel_prop;

		for (i = 0; i < mapphone_dss_data.ovl2mgr_mapping_cnt; i++) {
			mapphone_dss_data.ovl2mgr_mapping[i].overlay_idx =
				read_ovl2mgr_mapping[i].overlay_idx;
			mapphone_dss_data.ovl2mgr_mapping[i].overlay_mgr =
				read_ovl2mgr_mapping[i].overlay_mgr;
			PANELDBG("Mapping data for overlay %u"
					" is to manager %u\n",
			mapphone_dss_data.ovl2mgr_mapping[i].overlay_idx,
			mapphone_dss_data.ovl2mgr_mapping[i].overlay_mgr);
		}
	} else {
		PANELDBG("Panel property is 0 for ovl to mgr mapping\n");
	}

	PANELDBG("Getting the xres = %u yres = %u\n",
		mapphone_hdtv_device.panel.timings.x_res,
		mapphone_hdtv_device.panel.timings.y_res);

	panel_prop = of_get_property(panel_node, "gpio_pwr_en", NULL);
	if (panel_prop != NULL)
		mapphone_hdmi_5v_enable = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dac_reg_name", NULL);
	if (panel_prop != NULL) {
		strncpy(mapphone_hdmi_dac_reg_name,
				(char *)panel_prop,
				(HDMI_DAC_REGULATOR_NAME_SIZE));
		mapphone_hdmi_dac_reg_name \
			[HDMI_DAC_REGULATOR_NAME_SIZE] = '\0';
	}

	of_node_put(panel_node);
	return 0;
}

static int mapphone_dt_get_feature_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_feature_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "feature_hdmi", NULL);
	if (panel_prop != NULL)
		mapphone_feature_hdmi = *(u8 *)panel_prop;

	return 0;
}

static void panel_print_dt(void)
{
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	printk(KERN_INFO"DT: width= %d height= %d\n",
		mapphone_lcd_device.panel.timings.x_res,
		mapphone_lcd_device.panel.timings.y_res);

	printk(KERN_INFO"DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
		mapphone_lcd_device.panel.timings.hfp,
		mapphone_lcd_device.panel.timings.hsw,
		mapphone_lcd_device.panel.timings.hbp,
		mapphone_lcd_device.panel.timings.vfp,
		mapphone_lcd_device.panel.timings.vsw,
		mapphone_lcd_device.panel.timings.vbp);

	printk(KERN_INFO"DT: gpio_reset=%d panel_id=0x%lx\n",
		mapphone_panel_data.reset_gpio,
		mapphone_lcd_device.panel.panel_id);

	printk(KERN_INFO"DT: phy.dsi: pixel_size=%d te_scan_line=%d\n",
		mapphone_lcd_device.ctrl.pixel_size,
		panel_data->te_scan_line);

	printk(KERN_INFO"DT: phy.dsi: data1_lane=%d data1_pol=%d\n",
		mapphone_lcd_device.phy.dsi.data1_lane,
		mapphone_lcd_device.phy.dsi.data1_pol);

	printk(KERN_INFO"DT: phy.dsi: data2_lane=%d data2_pol=%d\n",
		mapphone_lcd_device.phy.dsi.data2_lane,
		mapphone_lcd_device.phy.dsi.data2_pol);

	printk(KERN_INFO"DT: phy.dsi.div : regn=%d regm=%d\n",
		mapphone_lcd_device.phy.dsi.div.regn,
		mapphone_lcd_device.phy.dsi.div.regm);

	printk(KERN_INFO"DT: phy.dsi.div : regm3=%d regm4=%d\n",
		mapphone_lcd_device.phy.dsi.div.regm_dispc,
		mapphone_lcd_device.phy.dsi.div.regm_dsi);

	printk(KERN_INFO "DT: phy.dsi.div : lp_clk_div=%d, lck_div=%d, pck_div=%d\n",
		mapphone_lcd_device.phy.dsi.div.lp_clk_div,
		mapphone_lcd_device.phy.dsi.div.lck_div,
		mapphone_lcd_device.phy.dsi.div.pck_div);

	printk(KERN_INFO"DT: phy.dsi.xfer_mode=%d\n",
		mapphone_lcd_device.phy.dsi.xfer_mode);
}

static int __init mapphone_dt_panel_init(void)
{
	int ret = 0;

	PANELDBG("dt_panel_init\n");

	if (mapphone_panel_device_read_dt == false) {
		if (mapphone_dt_get_feature_info() != 0) {
			printk(KERN_ERR "failed to parse feature info\n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_dsi_panel_info() != 0) {
			printk(KERN_ERR "failed to parse DSI panel info\n");
			ret = -ENODEV;
		} else if ((mapphone_lcd_device.phy.dsi.xfer_mode ==
						OMAP_DSI_XFER_VIDEO_MODE) &&
				(mapphone_dt_get_dsi_vm_info() != 0)) {
			printk(KERN_ERR "failed to parse DSI VM info\n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_panel_info() != 0) {
			printk(KERN_ERR "failed to parse panel info\n");
			ret = -ENODEV;
		} else if (mapphone_feature_hdmi &&
				mapphone_dt_get_hdtv_info() != 0) {
			printk(KERN_ERR "failed to parse hdtv info\n");
			ret = -ENODEV;
		} else {
			mapphone_panel_device_read_dt = true;
		}
	}
	return ret;
}


static int mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev)
{
	int rc = 0;

	PANELDBG("mapphone_panel_enable_hdtv\n");

	if (!mapphone_hdmi_dac_reg) {
		if (mapphone_hdmi_dac_reg_name[0] == 0) {
			printk(KERN_ERR "No HDMI regulator defined\n");
			rc = -1;
			goto exit;
		}
		mapphone_hdmi_dac_reg = regulator_get(NULL,
					mapphone_hdmi_dac_reg_name);
		if (IS_ERR(mapphone_hdmi_dac_reg)) {
			printk(KERN_ERR "Failed HDMI regulator_get\n");
			rc = -1;
			goto exit;
		}
	}

	rc = regulator_enable(mapphone_hdmi_dac_reg);
	if (rc != 0) {
		printk(KERN_ERR "Failed HDMI regulator_enable (%d)\n", rc);
	} else {
		printk(KERN_DEBUG "Enabled HDMI DAC regulator\n");
		/* Settling time */
		msleep(2);
	}

exit:
	return rc;
}

static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev)
{
	PANELDBG("mapphone_panel_disable_hdtv\n");

	if (mapphone_hdmi_dac_reg)
		regulator_disable(mapphone_hdmi_dac_reg);
}

static int  mapphone_panel_enable_hpd_hdtv(struct omap_dss_device *dssdev)
{
	int rc = 0;

	PANELDBG("mapphone_panel_enable_hpd_hdtv\n");

	mapphone_hdmi_platform_hpd_en = 1;

	if (mapphone_hdmi_5v_enable && !mapphone_hdmi_5v_force_off)
		gpio_set_value(mapphone_hdmi_5v_enable, 1);

	return rc;
}

static void mapphone_panel_disable_hpd_hdtv(struct omap_dss_device *dssdev)
{
	PANELDBG("mapphone_panel_disable_hpd_hdtv\n");

	mapphone_hdmi_platform_hpd_en = 0;

	if (mapphone_hdmi_5v_enable)
		gpio_set_value(mapphone_hdmi_5v_enable, 0);
}

static int mapphone_panel_hdtv_test(struct omap_dss_device *not_used, int tst)
{
	if (tst == 0) {
		mapphone_hdmi_5v_force_off = 0;
		if (mapphone_hdmi_platform_hpd_en && mapphone_hdmi_5v_enable)
			gpio_set_value(mapphone_hdmi_5v_enable, 1);
	} else if (tst == 1) {
		mapphone_hdmi_5v_force_off = 1;
		if (mapphone_hdmi_platform_hpd_en && mapphone_hdmi_5v_enable)
			gpio_set_value(mapphone_hdmi_5v_enable, 0);
	}

	return 0;
}

static struct platform_device omap_panel_device = {
       .name = "omap-panel",
       .id = -1,
};

static struct platform_device omap_dssmgr_device = {
	.name = "omap-dssmgr",
	.id = -1,
};

static void mapphone_panel_get_fb_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	mapphone_fb_data.feature_fb1_to_vid2 = mapphone_feature_hdmi;

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return;

	panel_prop = of_get_property(panel_node, "num_fbs", NULL);
	if (panel_prop != NULL)
		mapphone_fb_data.num_fbs = *(u32 *)panel_prop;

	of_node_put(panel_node);
}

void __init mapphone_panel_init(void)
{
	int ret;

	dss_pwrdm = pwrdm_lookup("dss_pwrdm");
	if (!dss_pwrdm)
		pr_info("%s: Not found dss_pwrdm\n", __func__);

	if (mapphone_dt_panel_init())
		printk(KERN_INFO "panel: using non-dt configuration\n");

	mapphone_panel_get_fb_info();
	omapfb_set_platform_data(&mapphone_fb_data);

	ret = gpio_request(mapphone_panel_data.reset_gpio, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto failed_reset;
	}

	if (mapphone_displ_pwr_sup_en != 0) {
		printk(KERN_INFO "DT: display power supply en = %d\n",
					mapphone_displ_pwr_sup_en);
		ret = gpio_request(mapphone_displ_pwr_sup_en, "LCD-pwr_sup_en");
		if (ret) {
			printk(KERN_ERR "failed to req for LCD-pwr_sup_en\n");
			goto failed_reset;
		}

		gpio_direction_output(mapphone_displ_pwr_sup_en, 1);
	}

	if (mapphone_feature_hdmi && mapphone_hdmi_5v_enable != 0) {
		ret = gpio_request(mapphone_hdmi_5v_enable, "HDMI-5V-En");
		if (ret) {
			printk(KERN_ERR "Failed hdmi 5v en gpio request\n");
			goto failed_hdmi_5v;
		} else {
			printk(KERN_DEBUG "Enabing hdmi 5v gpio\n");
			gpio_direction_output(mapphone_hdmi_5v_enable, 1);
			gpio_set_value(mapphone_hdmi_5v_enable, 0);
		}

		platform_device_register(&omap_dssmgr_device);
	} else {
		/* Remove HDTV from the DSS device list */
		mapphone_dss_data.num_devices--;
	}

	platform_device_register(&omap_panel_device);
	omap_display_init(&mapphone_dss_data);

	return;

failed_hdmi_5v:
	gpio_free(mapphone_hdmi_5v_enable);
failed_reset:
	gpio_free(mapphone_panel_data.reset_gpio);
}
