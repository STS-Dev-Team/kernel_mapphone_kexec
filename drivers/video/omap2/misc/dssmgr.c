/*
 * drivers/video/omap2/misc/dssmgr.c
 *
 * Copyright (C) 2010 Motorola Inc.
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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/slab.h> /* For kzalloc/kfree*/

#include <plat/display.h>
#include <plat/vout.h>
#include <plat/hdmi_lib.h>

#include <linux/omap-dssmgr.h>
#include "dssmgr-oly.h"

#include "../dss/dss.h"

#define DEVICE_NAME  "omap-dssmgr"

/*#define DM_DEBUG*/
#ifdef DM_DEBUG
#define DMDBG(format, ...) \
	printk("DSSMGR: " format, ## __VA_ARGS__)
#else
#define DMDBG(format, ...)
#endif

#define EDID_BLOCK_SIZE      (DSSMGR_EDID_BLOCK_LEN)
#define EDID_MAX_BLOCKS      (4)
#define EDID_MAX_SIZE        (EDID_MAX_BLOCKS * EDID_BLOCK_SIZE)
#define EDID_HEADER_SIZE     (8)
#define EDID_NUM_EXT_BLKS    (0x7E)

static const u8 header[EDID_HEADER_SIZE] = { 0x0,  0xff, 0xff, 0xff
					   , 0xff, 0xff, 0xff, 0x0};

struct dssmgr_device {
	struct mutex   mtx; /* Lock for all device accesses */

	int major;
	struct class  *cls;
	struct device *dev;

	u32 client_next_id;
	int client_cnt;

	int num_dpys;
	struct omap_dss_device      *dpys[DSSMGR_MAX_DISPLAYS];
	int num_mgrs;
	struct omap_overlay_manager *mgrs[DSSMGR_MAX_MANAGERS];
	int num_olys;
	struct omap_overlay         *olys[DSSMGR_MAX_OVERLAYS];

	/* Overlay manipulation specific data */
	struct dssmgr_oly_data       oly_data;
};

static struct dssmgr_device *g_dev;

/*=== Local Functions ==================================================*/

static int dssmgr_enable_dpy(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device *dpy;
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en  = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		printk(KERN_ERR "DSSMGR: Invalid Display ID\n");
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		printk(KERN_ERR "DSSMGR: Invalid Display Enable\n");
		rc = -EINVAL;
		goto failed;
	}

	dpy = g_dev->dpys[idx];
	if (en) {
		/* Use smart_enable if present */
		if (dpy->driver->smart_enable)
			rc = dpy->driver->smart_enable(dpy);
		else
			rc = dpy->driver->enable(dpy);
		DMDBG("Enabled display (%s/%d)\n", dpy->name, rc);
	} else {
		dpy->driver->disable(dpy);
		DMDBG("Disabled display (%s)\n", dpy->name);
	}

failed:
	return rc;
}

static int dssmgr_attach_oly_2_mgr(struct dssmgr_cmd *cmd)
{
	struct omap_overlay         *oly;
	struct omap_overlay_manager *mgr, *old;
	struct dssmgr_oly_data      *od;
	int rc = 0;
	int rc2 = 0;
	int o_idx, m_idx;
	int rm_force = 0;

	o_idx = cmd->comp_id1 - 1;
	m_idx = cmd->comp_id2 - 1;

	if (o_idx < 0 || o_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (m_idx < 0 || m_idx >= g_dev->num_mgrs) {
		rc = -EINVAL;
		goto failed;
	}

	oly = g_dev->olys[o_idx];
	mgr = g_dev->mgrs[m_idx];
	if (oly->manager != mgr) {
		od = &g_dev->oly_data;
		if (!dssmgr_oly_get_force_disable(od, o_idx)) {
			dssmgr_oly_set_force_disable(od, o_idx, 1);
			rm_force = 1;
		}

		old = NULL;
		if (oly->manager) {
			old = oly->manager;
			oly->unset_manager(oly);
			old->apply(old);
		}

		rc = oly->set_manager(oly, mgr);
		if (rc != 0) {
			printk(KERN_ERR "DSSMGR: Set MGR Failure\n");
			if (old != NULL) {
				oly->set_manager(oly, old);
				old->apply(old);
			}
		}

		if (rm_force) {
			/* Does the mgr apply */
			rc2 = dssmgr_oly_set_force_disable(od, o_idx, 0);
		} else if (rc == 0) {
			/* the set_manager succeeded, so apply */
			rc2 = mgr->apply(mgr);
		}

		if (rc2 != 0 && rc == 0)
			rc = rc2;

		DMDBG("Oly(%s)->Mgr(%s)/%d\n", oly->name, mgr->name, rc);
	}

failed:
	return rc;
}

static int dssmgr_attach_mgr_2_dpy(struct dssmgr_cmd *cmd)
{
	struct omap_overlay_manager *mgr;
	struct omap_dss_device      *dpy;
	int rc = 0;
	int m_idx, d_idx;

	m_idx = cmd->comp_id1 - 1;
	d_idx = cmd->comp_id2 - 1;

	if (m_idx < 0 || m_idx >= g_dev->num_mgrs) {
		rc = -EINVAL;
		goto failed;
	}
	if (d_idx < 0 || d_idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}

	mgr = g_dev->mgrs[m_idx];
	dpy = g_dev->dpys[d_idx];
	if (mgr->device != dpy) {
		if (mgr->device)
			mgr->unset_device(mgr);

		rc = mgr->set_device(mgr, dpy);
		if (rc != 0)
			printk(KERN_ERR "DSSMGR: Set DPY Failure\n");
		else
			rc = mgr->apply(mgr);

		DMDBG("Mgr(%s)->Dpy(%s)/%d\n", mgr->name, dpy->name, rc);
	}

failed:
	return rc;
}

static int getResModeCode(struct dssmgr_resolution *res, int *mode, int *code)
{
	int rc = -1;
	int m = OMAP_DSS_EDID_TIMING_MODE_VESA;
	int c = -1;

	int w = res->width;
	int h = res->height;
	int i = ((res->flags & DSSMGR_RES_FLAG_INTERLACED) != 0);
	int r60hz = (res->rate == 60);
	int r50hz = (res->rate == 50);

	if (res->flags & DSSMGR_RES_FLAG_HDMI) {
		m = OMAP_DSS_EDID_TIMING_MODE_CEA;

		switch (h) {
		case 480:
			if (w == 640 && !i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_640x480_60Hz;
			else if (w == 720 && i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_720x480I_60Hz;
			else if (w == 720 && !i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_720x480P_60Hz;
			break;
		case 576:
			if (w == 720 && i && r50hz)
				c = OMAP_DSS_EDID_TIMING_CEA_720x576I_50Hz;
			else if (w == 720 && !i && r50hz)
				c = OMAP_DSS_EDID_TIMING_CEA_720x576P_50Hz;
			break;
		case 720:
			if (w == 1280 && !i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1280x720P_60Hz;
			else if (w == 1280 && !i && r50hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1280x720P_50Hz;
			break;
		case 1080:
			if (w == 1920 && i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1920x1080I_60Hz;
			else if (w == 1920 && !i && r60hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1920x1080P_60Hz;
			else if (w == 1920 && i && r50hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1920x1080I_50Hz;
			else if (w == 1920 && !i && r50hz)
				c = OMAP_DSS_EDID_TIMING_CEA_1920x1080P_50Hz;
			break;
		}
	} else if (!i && r60hz) {
		switch (h) {
		case 480:
			if (w == 640)
				c = OMAP_DSS_EDID_TIMING_VESA_640x480_60Hz;
			else if (w == 848)
				c = OMAP_DSS_EDID_TIMING_VESA_848x480_60Hz;
			break;
		case 600:
			if (w == 800)
				c = OMAP_DSS_EDID_TIMING_VESA_800x600_60Hz;
			break;
		case 768:
			if (w == 1024)
				c = OMAP_DSS_EDID_TIMING_VESA_1024x768_60Hz;
			else if (w == 1280)
				c = OMAP_DSS_EDID_TIMING_VESA_1280x768_60Hz;
			else if (w == 1360)
				c = OMAP_DSS_EDID_TIMING_VESA_1360x768_60Hz;
			else if (w == 1366)
				c = OMAP_DSS_EDID_TIMING_VESA_1366x768_60Hz;
			break;
		case 800:
			if (w == 1280)
				c = OMAP_DSS_EDID_TIMING_VESA_1280x800_60Hz;
			break;
		case 900:
			if (w == 1440)
				c = OMAP_DSS_EDID_TIMING_VESA_1440x900_60Hz;
			break;
		case 960:
			if (w == 1280)
				c = OMAP_DSS_EDID_TIMING_VESA_1280x960_60Hz;
			break;
		case 1024:
			if (w == 1280)
				c = OMAP_DSS_EDID_TIMING_VESA_1280x1024_60Hz;
			break;
		case 1050:
			if (w == 1400)
				c = OMAP_DSS_EDID_TIMING_VESA_1400x1050_60Hz;
			else if (w == 1680)
				c = OMAP_DSS_EDID_TIMING_VESA_1680x1050_60Hz;
			break;
		case 1080:
			if (w == 1920)
				c = OMAP_DSS_EDID_TIMING_VESA_1920x1080_60Hz;
			break;
		}
	}

	if (c != -1) {
		*mode = m;
		*code = c;
		rc = 0;
	}

	return rc;
}

static int dssmgr_set_resolution(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device    *dpy;
	struct omap_video_timings  timings;
	int rc = 0;
	int idx;
	int mode, code;
	int reenable = 0;

	idx = cmd->comp_id1 - 1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}

	DMDBG("Resolution: %dx%d @ %d (%04x)\n",
		cmd->resolution.width, cmd->resolution.height,
		cmd->resolution.rate, cmd->resolution.flags);

	dpy = g_dev->dpys[idx];

	if (hdmi_w1_get_video_state()) {
		reenable = 1;
		dpy->driver->disable(dpy);
	}

	if (cmd->resolution.flags & DSSMGR_RES_FLAG_MIN_DS)
		dpy->phy.hdmi.ds_percent = 25;
	else
		dpy->phy.hdmi.ds_percent = 100;

	if ((cmd->resolution.flags & DSSMGR_RES_FLAG_HDMI)
	   || !(cmd->resolution.flags & DSSMGR_RES_FLAG_TIMING)) {
		/* Set via EDID timing mode/code */
		if (dpy->driver->set_edid_timing == NULL) {
			rc = -EINVAL;
			goto failed;
		}

		rc = getResModeCode(&cmd->resolution, &mode, &code);
		if (rc != 0) {
			rc = -EINVAL;
			goto failed;
		}

		rc = dpy->driver->set_edid_timing(dpy, mode, code);
	} else {
		/* Set via direct timing data */
		if (dpy->driver->set_timings == NULL) {
			rc = -EINVAL;
			goto failed;
		}

		timings.x_res       = cmd->resolution.width;
		timings.y_res       = cmd->resolution.height;
		timings.pixel_clock = cmd->resolution.timing.clk;
		timings.hfp         = cmd->resolution.timing.hfp;
		timings.hsw         = cmd->resolution.timing.hsw;
		timings.hbp         = cmd->resolution.timing.hbp;
		timings.vfp         = cmd->resolution.timing.vfp;
		timings.vsw         = cmd->resolution.timing.vsw;
		timings.vbp         = cmd->resolution.timing.vbp;
		timings.sp          = cmd->resolution.timing.sp;

		/* FIXME: For now add the audio flag to the sp value, which
		 *        can be done since only the lower two bits of sp
		 *        are used.
		 */
		timings.sp |= (cmd->resolution.timing.audio) ? 0x80 : 0x00;

		dpy->driver->set_timings(dpy, &timings);
	}

	dmmgr_oly_reconfig_for_dpy(&g_dev->oly_data, dpy);

failed:

	if (reenable) {
		if (dpy->driver->smart_enable)
			rc = dpy->driver->smart_enable(dpy);
		else
			rc = dpy->driver->enable(dpy);
	}

	return rc;
}

static int dssmgr_enable_hpd(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device *dpy;
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en  = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dpy = g_dev->dpys[idx];

	if (dpy->driver->hpd_enable == NULL) {
		rc = -EINVAL;
		goto failed;
	}

	rc = dpy->driver->hpd_enable(dpy, en);
	DMDBG("Set HPD Enable on display to %d (%s/%d)\n", en, dpy->name, rc);

failed:
	return rc;
}

static int dssmgr_reset_oly(struct dssmgr_cmd *cmd)
{
	int rc = 0;
	int idx;

	idx = cmd->comp_id1 - 1;

	DMDBG("dssmgr_reset_oly/%d\n", idx);

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_reset_oly(&g_dev->oly_data, idx);

failed:
	return rc;
}

static int dssmgr_force_disable_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, dis;

	idx = cmd->comp_id1 - 1;
	dis = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (dis != 0 && dis != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_disable(&g_dev->oly_data, idx, dis);

failed:
	return rc;
}

static int dssmgr_force_rotation_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, rot;

	idx = cmd->comp_id1 - 1;
	rot = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	switch (rot) {
	case DSSMGR_ROTATION_IGNORE:
	case DSSMGR_ROTATION_0:
	case DSSMGR_ROTATION_90:
	case DSSMGR_ROTATION_180:
	case DSSMGR_ROTATION_270:
		break;
	default:
		rc = -EINVAL;
		goto failed;
		break;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_rotation(&g_dev->oly_data, idx,
					(enum dssmgr_rotation) rot);

failed:
	return rc;
}

static int dssmgr_force_scale_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, scale, percent;

	idx     = cmd->comp_id1 - 1;
	scale   = cmd->val1;
	percent = cmd->val2;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	switch (scale) {
	case DSSMGR_SCALE_IGNORE:
	case DSSMGR_SCALE_FILL_SCREEN:
	case DSSMGR_SCALE_FIT_TO_SCREEN:
	case DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN:
	case DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN:
		break;
	default:
		rc = -EINVAL;
		goto failed;
		break;
	}
	if ((scale == DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN
	    || scale == DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN
	    ) && (percent < 40 || percent > 100)) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_scale(&g_dev->oly_data, idx,
					(enum dssmgr_scale) scale, percent);

failed:
	return rc;
}

static int dssmgr_force_mirror_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int s_idx, d_idx;

	s_idx = cmd->comp_id1 - 1;
	d_idx = cmd->comp_id2 - 1;

	if (s_idx < 0 || s_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	/* The -1 case is to disable the forced mirroring */
	if (d_idx < -1 || d_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}

	if (d_idx == -1)
		d_idx = DSSMGR_OLY_INVALID_IDX;

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_mirror(&g_dev->oly_data, s_idx, d_idx);

failed:
	return rc;
}

static int dssmgr_force_enable_oly_nf(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_enable_nf(&g_dev->oly_data, idx, en);

failed:
	return rc;
}

static int dssmgr_force_exclusive_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_exclusive(&g_dev->oly_data, idx, en);

failed:
	return rc;
}

static int dssmgr_s_cmd(struct dssmgr_cmd *client_cmd, u32 client_id)
{
	struct dssmgr_cmd cmd;
	int rc = 0;

	/* Serialize these commands */
	mutex_lock(&g_dev->mtx);

	rc = copy_from_user(&cmd, client_cmd, sizeof(cmd));
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR: S_CMD copy from user failed\n");
		goto failed;
	}

	dss_mainclk_enable();

	switch (cmd.cmd) {
	case DSSMGR_CMD_ENABLE_DPY:
		rc = dssmgr_enable_dpy(&cmd);
		break;
	case DSSMGR_CMD_ATTACH_OLY2MGR:
		rc = dssmgr_attach_oly_2_mgr(&cmd);
		break;
	case DSSMGR_CMD_ATTACH_MGR2DPY:
		rc = dssmgr_attach_mgr_2_dpy(&cmd);
		break;
	case DSSMGR_CMD_SET_RESOLUTION:
		rc = dssmgr_set_resolution(&cmd);
		break;
	case DSSMGR_CMD_ENABLE_HPD:
		rc = dssmgr_enable_hpd(&cmd);
		break;
	case DSSMGR_CMD_RESET_OLY:
		rc = dssmgr_reset_oly(&cmd);
		break;
	case DSSMGR_CMD_FORCE_DISABLE_OLY:
		rc = dssmgr_force_disable_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_ROTATION_OLY:
		rc = dssmgr_force_rotation_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_SCALE_OLY:
		rc = dssmgr_force_scale_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_MIRROR_OLY:
		rc = dssmgr_force_mirror_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_ENABLE_OLY_NEXT_FRAME:
		rc = dssmgr_force_enable_oly_nf(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_EXCLUSIVE_OLY:
		rc = dssmgr_force_exclusive_oly(&cmd, client_id);
		break;
	default:
		printk(KERN_ERR "DSSMGR: Invalid CMD (%x)\n", cmd.cmd);
		rc = -EINVAL;
		break;
	}

	dss_mainclk_disable();

failed:
	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dssmgr_querycfg(struct dssmgr_cfg *client_cfg)
{
	struct dssmgr_cfg            cfg;
	struct omap_dss_device      *dssdev;
	struct dssmgr_display       *dpy;
	struct omap_overlay_manager *dssmgr;
	struct dssmgr_manager       *mgr;
	struct omap_overlay         *dssoly;
	struct dssmgr_overlay       *oly;
	int i, j;

	memset(&cfg, 0, sizeof(struct dssmgr_cfg));

	for (i = 0; i < g_dev->num_dpys; i++) {
		dssdev = g_dev->dpys[i];
		dpy    = &cfg.displays[i];
		dpy->id = i + 1;
		strncpy(dpy->name, dssdev->name, DSSMGR_MAX_NAME_SIZE);
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			dpy->flags |= DSSMGR_DPY_FLAG_ACTIVE;
		if (dssdev->driver->get_edid != NULL)
			dpy->flags |= DSSMGR_DPY_FLAG_EDID;
		if (dssdev->driver->hpd_enable != NULL)
			dpy->flags |= DSSMGR_DPY_FLAG_HOTPLUG;

		dpy->resolution.width  = dssdev->panel.timings.x_res;
		dpy->resolution.height = dssdev->panel.timings.y_res;
		dpy->resolution.rate   = 0;
		dpy->resolution.flags  = 0;
	}

	for (i = 0; i < g_dev->num_mgrs; i++) {
		dssmgr = g_dev->mgrs[i];
		mgr    = &cfg.managers[i];
		mgr->id = i + 1;
		strncpy(mgr->name, dssmgr->name, DSSMGR_MAX_NAME_SIZE);
		for (j = 0; j < g_dev->num_dpys; j++) {
			if (dssmgr->device == g_dev->dpys[j]) {
				mgr->disp_id = j + 1;
				break;
			}
		}
	}

	for (i = 0; i < g_dev->num_olys; i++) {
		dssoly = g_dev->olys[i];
		oly    = &cfg.overlays[i];
		oly->id = i + 1;
		strncpy(oly->name, dssoly->name, DSSMGR_MAX_NAME_SIZE);
		for (j = 0; j < g_dev->num_mgrs; j++) {
			if (dssoly->manager == g_dev->mgrs[j]) {
				oly->mgr_id = j + 1;
				break;
			}
		}
		oly->active = dssoly->info.enabled;

		/* Client Enabled Flag */
		oly->rsvd[0] = (u8) g_dev->oly_data.info[i].enabled;
		/* Force Disable Flag */
		oly->rsvd[1] = (u8) g_dev->oly_data.flds[i].force_disable;
		/* Force Rotation */
		oly->rsvd[2] = (u8) g_dev->oly_data.flds[i].force_rotation;
		/* Force Scale */
		oly->rsvd[3] = (u8) g_dev->oly_data.flds[i].force_scale;
		/* Force Mirror Destination Index */
		oly->rsvd[4] = g_dev->oly_data.flds[i].force_mirror_dst_idx;
		/* Force Enable on Next Client Frame */
		oly->rsvd[5] = (u8) g_dev->oly_data.flds[i].force_enable_nf;
		/* Force Exclusive Overlay */
		oly->rsvd[6] = (u8) g_dev->oly_data.flds[i].force_exclusive;
		/* Client Frame Count */
		oly->fcnt    = (u8) g_dev->oly_data.flds[i].fcnt;
	}

	return copy_to_user(client_cfg, &cfg, sizeof(cfg));
}

static int dssmgr_g_edid(struct dssmgr_edid *client_edid)
{
	static u8 edid[EDID_MAX_SIZE];

	struct omap_dss_device *dssdev;
	struct dssmgr_edid      c_edid;
	int rc = 0;
	int idx;
	int blk;
	int max;
	int i;

	rc = copy_from_user(&c_edid, client_edid, sizeof(c_edid));
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR: G_EDID copy from user failed\n");
		goto failed;
	}

	idx = c_edid.dpy_id - 1;
	blk = c_edid.block;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		printk(KERN_ERR "DSSMGR: G_EDID invalid display idx\n");
		rc = -EINVAL;
		goto failed;
	}

	dssdev = g_dev->dpys[idx];

	if (dssdev->driver->get_edid == NULL)	{
		printk(KERN_ERR "DSSMGR: G_EDID non-EDID display\n");
		rc = -EINVAL;
		goto failed;
	}

	if (blk < 0 && blk >= EDID_MAX_BLOCKS) {
		printk(KERN_ERR "DSSMGR: G_EDID invalid EDID block\n");
		rc = -EINVAL;
		goto failed;
	}

	/* Due to an issue in the TI driver, if you try to read the EDID
	 * a second time the data shifts, thus we cache all the EDID data
	 * when the first block is requested and read from the cache
	 * when the further blocks are read.
	 * The TI bug for this issue is CSR OMAPS00228236.
	 */
	if (blk == 0) {
		if (dssdev->driver->get_edid(dssdev, edid,
						EDID_MAX_SIZE) != 0) {
			printk(KERN_WARNING "HDMI failed to read EDID\n");
			rc = -EIO;
			goto failed;
		}
	}

	for (i = 0; i < EDID_HEADER_SIZE; i++) {
		if (edid[i] == header[i])
			continue;
		break;
	}

	if (i != EDID_HEADER_SIZE) {
		printk(KERN_WARNING "HDMI EDID Invalid (%d)\n", i);
		rc = -EIO;
		goto failed;
	}

	max = edid[EDID_NUM_EXT_BLKS];
	if (max > (EDID_MAX_BLOCKS - 1))
		max = (EDID_MAX_BLOCKS - 1);

	if (blk > max) {
		printk(KERN_ERR "Invalid EDID Block Requested\n");
		rc = -EINVAL;
	} else {
		memcpy(c_edid.data, &edid[blk * EDID_BLOCK_SIZE],
							EDID_BLOCK_SIZE);
	}

	if (rc == 0)
		rc = copy_to_user(client_edid, &c_edid, sizeof(c_edid));

failed:
	return rc;
}

/*=== Driver Interface Functions =======================================*/

static int dssmgr_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	file->private_data = (void *) g_dev->client_next_id++;
	g_dev->client_cnt++;

	DMDBG("dssmgr_open/%d/%d\n", g_dev->client_cnt, g_dev->client_next_id);

	mutex_unlock(&g_dev->mtx);
	return rc;
}

static int dssmgr_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	u32 client_id;

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	client_id = (u32) file->private_data;

	mutex_lock(&g_dev->mtx);

	DMDBG("dssmgr_release/%d/%d\n", g_dev->client_cnt, client_id);

	if (g_dev->client_cnt > 0)
		g_dev->client_cnt--;

	if (g_dev->client_cnt == 0)
		dssmgr_oly_reset(&g_dev->oly_data);
	else
		dssmgr_oly_reset_client(&g_dev->oly_data, client_id);

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dssmgr_ioctl(struct inode *inode, struct file *file,
							u_int cmd, u_long arg)
{
	int rc = 0;
	u32 client_id;

	if (unlikely(_IOC_TYPE(cmd) != DSSMGR_IOCTL_MAGIC)) {
		printk(KERN_ERR "DSSMGR: Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case DSSMGR_QUERYCFG:
		rc = dssmgr_querycfg((struct dssmgr_cfg *) arg);
		break;
	case DSSMGR_S_CMD:
		client_id = (u32) file->private_data;
		DMDBG("dssmgr_ioctl/cmd/%d/%d\n", client_id,
					((struct dssmgr_cmd *) arg)->cmd);
		rc = dssmgr_s_cmd((struct dssmgr_cmd *) arg, client_id);
		break;
	case DSSMGR_G_EDID:
		rc = dssmgr_g_edid((struct dssmgr_edid *) arg);
		break;
	default:
		printk(KERN_ERR "DSSMGR: Invalid ioctl (%x)\n", cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations dssmgr_fops = {
	.owner   = THIS_MODULE,
	.open    = dssmgr_open,
	.release = dssmgr_release,
	.ioctl   = dssmgr_ioctl,
};

static struct miscdevice dssmgr_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &dssmgr_fops,
};

static int __init dssmgr_probe(struct platform_device *pdev)
{
	struct omap_dss_device *dssdev;
	int rc = 0;
	int i, n;

	DMDBG("dssmgr_probe\n");

	g_dev = (struct dssmgr_device *)
			kzalloc(sizeof(struct dssmgr_device), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	memset(g_dev, 0, sizeof(g_dev));

	mutex_init(&g_dev->mtx);

	g_dev->client_next_id = 1;
	g_dev->client_cnt     = 0;

	rc = misc_register(&dssmgr_misc_device);
	if (rc) {
		printk(KERN_ERR "dssmgr: misc register failed (%d)\n", rc);
		goto failed_dev;
	}

	g_dev->num_dpys = 0;
	dssdev = NULL;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		if (g_dev->num_dpys < DSSMGR_MAX_DISPLAYS)
			g_dev->dpys[g_dev->num_dpys++] = dssdev;
	}

	if (g_dev->num_dpys <= 0) {
		dev_err(&pdev->dev, "No displays\n");
		rc = -EINVAL;
		goto failed_dpy;
	}

	n = omap_dss_get_num_overlay_managers();
	g_dev->num_mgrs = (n < DSSMGR_MAX_MANAGERS) ? n : DSSMGR_MAX_MANAGERS;
	for (i = 0; i < g_dev->num_mgrs; i++)
		g_dev->mgrs[i] = omap_dss_get_overlay_manager(i);

	n = omap_dss_get_num_overlays();
	g_dev->num_olys = (n < DSSMGR_MAX_OVERLAYS) ? n : DSSMGR_MAX_OVERLAYS;
	for (i = 0; i < g_dev->num_olys; i++)
		g_dev->olys[i] = omap_dss_get_overlay(i);

	rc = dssmgr_oly_init(&g_dev->oly_data);
	if (rc != 0) {
		printk(KERN_ERR "dssmgr-oly init failed\n");
		goto failed_dpy;
	}

	g_dev->dev = device_create(g_dev->cls, g_dev->dev,
				MKDEV(g_dev->major, 0), NULL, DEVICE_NAME);

	return 0;

failed_dpy:
	misc_deregister(&dssmgr_misc_device);
failed_dev:
	kfree(g_dev);
	g_dev = NULL;
	return rc;
}

static int dssmgr_remove(struct platform_device *pdev)
{
	DMDBG("dssmgr_remove\n");

	if (g_dev) {
		dssmgr_oly_remove(&g_dev->oly_data);
		misc_deregister(&dssmgr_misc_device);
		kfree(g_dev);
		g_dev = NULL;
	}

	return 0;
}

static struct platform_driver dssmgr_driver = {
	.remove		= dssmgr_remove,
	.driver		= {
		.name   = DEVICE_NAME,
	},
};

static int __init dssmgr_init(void)
{
	int rc;

	DMDBG("dssmgr_init\n");

	rc = platform_driver_probe(&dssmgr_driver, dssmgr_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed dssmgr register/probe %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit dssmgr_exit(void)
{
	DMDBG("dssmgr_exit\n");

	platform_driver_unregister(&dssmgr_driver);
}

device_initcall_sync(dssmgr_init);
module_exit(dssmgr_exit);

MODULE_DESCRIPTION("DSS2 Manager");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

