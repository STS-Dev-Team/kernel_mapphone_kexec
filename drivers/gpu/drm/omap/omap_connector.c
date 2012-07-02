/*
 * linux/drivers/gpu/drm/omap/omap_connector.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
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

#include <linux/omap_gpu.h>
#include "omap_gpu_priv.h"

#include "drm_crtc.h"
#include "drm_crtc_helper.h"

/*
 * connector funcs
 */

#define to_omap_connector(x) container_of(x, struct omap_connector, base)

struct omap_connector {
	struct drm_connector base;
	struct omap_dss_device *dssdev;
};


static inline void copy_timings_omap_to_drm(struct drm_display_mode *mode,
		struct omap_video_timings *timings)
{
	mode->clock = timings->pixel_clock;

	mode->hdisplay = timings->x_res;
	mode->hsync_start = mode->hdisplay + timings->hfp;
	mode->hsync_end = mode->hsync_start + timings->hsw;
	mode->htotal = mode->hsync_end + timings->hbp;

	mode->vdisplay = timings->y_res;
	mode->vsync_start = mode->vdisplay + timings->vfp;
	mode->vsync_end = mode->vsync_start + timings->vsw;
	mode->vtotal = mode->vsync_end + timings->vbp;

	/* note: whether or not it is interlaced, +/- h/vsync, etc,
	 * which should be set in the mode flags, is not exposed in
	 * the omap_video_timings struct.. but hdmi driver tracks
	 * those separately so all we have to have to set the mode
	 * is the way to recover these timings values, and the
	 * omap_dss_driver would do the rest.
	 */
}

static inline void copy_timings_drm_to_omap(struct omap_video_timings *timings,
		struct drm_display_mode *mode)
{
	timings->pixel_clock = mode->clock;

	timings->x_res = mode->hdisplay;
	timings->hfp = mode->hsync_start - mode->hdisplay;
	timings->hsw = mode->hsync_end - mode->hsync_start;
	timings->hbp = mode->htotal - mode->hsync_end;

	timings->y_res = mode->vdisplay;
	timings->vfp = mode->vsync_start - mode->vdisplay;
	timings->vsw = mode->vsync_end - mode->vsync_start;
	timings->vbp = mode->vtotal - mode->vsync_end;
}

extern bool omap_crtc_first;
void omap_connector_dpms(struct drm_connector *connector, int mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	if (omap_crtc_first || !dssdrv)
		return;

	DBG("%s: %d", dssdev->name, mode);

	if (dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		/* Ignoring HDMI configuration since user space
		 * will handle this
		 */
	} else if (mode == DRM_MODE_DPMS_ON) {
		if (dssdrv->smart_enable)
			dssdrv->smart_enable(dssdev);
		else if (dssdrv->enable)
			dssdrv->enable(dssdev);
	} else {
		/* TODO: add API in DSS to suspend/resume individual displays..
		 */
		if (dssdrv->disable)
			dssdrv->disable(dssdev);
	}
}

enum drm_connector_status omap_connector_detect(
		struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	enum drm_connector_status ret;
	bool enabled;

	if (dssdrv->is_detected(dssdev)) {
		ret = connector_status_connected;
	} else {
		ret = connector_status_disconnected;
	}

	DBG("%s: %d", omap_connector->dssdev->name, ret);

	return ret;
}

static void omap_connector_destroy(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	DBG("%s", omap_connector->dssdev->name);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(omap_connector);
}

#define MAX_EDID  256

static int omap_connector_get_modes(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	int n = 0;

	DBG("%s", omap_connector->dssdev->name);

	/* if display exposes EDID, then we parse that in the normal way to
	 * build table of supported modes.. otherwise (ie. fixed resolution
	 * LCD panels) we just return a single mode corresponding to the
	 * currently configured timings:
	 */
	if (dssdrv->get_edid) {
		void *edid = kzalloc(MAX_EDID, GFP_KERNEL);

		dssdrv->get_edid(dssdev, edid, MAX_EDID);

		drm_mode_connector_update_edid_property(connector, edid);
		n = drm_add_edid_modes(connector, edid);

		kfree(connector->display_info.raw_edid);
		connector->display_info.raw_edid = edid;

	} else {
		struct drm_display_mode *mode = drm_mode_create(connector->dev);
		struct omap_video_timings timings;

		dssdrv->get_timings(dssdev, &timings);

		copy_timings_omap_to_drm(mode, &timings);

		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);

		n = 1;
	}

	return n;
}


static int omap_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	struct omap_video_timings timings = {0};
	int ret = MODE_BAD;

	copy_timings_drm_to_omap(&timings, mode);

	if (dssdev == NULL) {
		/* Just return an error */
	} else if (dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		/* Ignoring HDMI configuration since user space
		 * will handle this
		 */
		ret = MODE_OK;
	} else if (!dssdrv->check_timings(dssdev, &timings)) {
		ret = MODE_OK;
	}

#if 0
	DBG("connector: mode %s: "
			"%d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			(ret == MODE_OK) ? "valid" : "invalid",
			mode->base.id, mode->name, mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal, mode->type, mode->flags);
#endif

	return ret;
}

struct drm_encoder * omap_connector_attached_encoder (
		struct drm_connector *connector)
{
	int i;
	struct omap_connector *omap_connector = to_omap_connector(connector);

	for (i = 0; i < DRM_CONNECTOR_MAX_ENCODER; i++) {
		struct drm_mode_object *obj;

		if (connector->encoder_ids[i] == 0)
			break;

		obj = drm_mode_object_find(connector->dev,
				connector->encoder_ids[i],
				DRM_MODE_OBJECT_ENCODER);

		if (obj) {
			struct drm_encoder *encoder = obj_to_encoder(obj);
			struct omap_overlay_manager *mgr =
					omap_encoder_get_manager(encoder);
			DBG("%s: found %s", omap_connector->dssdev->name, mgr->name);
			return encoder;
		}
	}

	DBG("%s: no encoder", omap_connector->dssdev->name);

	return NULL;
}

static const struct drm_connector_funcs omap_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = omap_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = omap_connector_destroy,
};

static const struct drm_connector_helper_funcs omap_connector_helper_funcs = {
	.get_modes = omap_connector_get_modes,
	.mode_valid = omap_connector_mode_valid,
	.best_encoder = omap_connector_attached_encoder,
};

/* called from encoder when mode is set, to propagate settings to the dssdev
 */
void omap_connector_mode_set(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	struct omap_video_timings timings;

	copy_timings_drm_to_omap(&timings, mode);

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			omap_connector->dssdev->name,
			mode->base.id, mode->name, mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal, mode->type, mode->flags);

	if (dssdev == NULL) {
		/* Just return */
		return;
	} else if (dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		/* Ignoring HDMI configuration since user space
		 * will handle this
		 */
		return;
	} else if (dssdrv->check_timings(dssdev, &timings)) {
		dev_err(dev->dev, "could not set timings\n");
		return;
	}

	dssdrv->set_timings(dssdev, &timings);
}

/* this really only makes sense for LCD panels, and fixed displays that do
 * not have an EDID.. otherwise this returns false and doesnt' update w/h
 */
bool omap_connector_get_dimension(struct drm_connector *connector,
		u32 *width, u32 *height)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	if (! dssdrv->get_dimension)
		return false;

	dssdrv->get_dimension(dssdev, width, height);

	return true;
}
EXPORT_SYMBOL(omap_connector_get_dimension);

enum omap_dss_update_mode omap_connector_get_update_mode(
		struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s", omap_connector->dssdev->name);

	if (dssdrv->get_update_mode) {
		return dssdrv->get_update_mode(dssdev);
	}

	return -1;
}
EXPORT_SYMBOL(omap_connector_get_update_mode);

int omap_connector_set_update_mode(struct drm_connector *connector,
		enum omap_dss_update_mode mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s: %d", omap_connector->dssdev->name, mode);

	if (dssdrv->set_update_mode) {
		return dssdrv->set_update_mode(dssdev, mode);
	}

	return -EINVAL;
}
EXPORT_SYMBOL(omap_connector_set_update_mode);

int omap_connector_sync(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s", omap_connector->dssdev->name);

	if (dssdrv->sync) {
		return dssdrv->sync(dssdev);
	}

	return -EINVAL;
}
EXPORT_SYMBOL(omap_connector_sync);

/* flush an area of the framebuffer (in case of manual update display that
 * is not automatically flushed)
 */
void omap_connector_flush(struct drm_connector *connector,
		int x, int y, int w, int h)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	if (dssdev_manually_updated(dssdev)) {
		u16 xres, yres;

		DBG("%s: %d,%d, %dx%d", omap_connector->dssdev->name, x, y, w, h);

		dssdrv->get_resolution(dssdev, &xres, &yres);

		/* for virtual displays, we could get asked to flush something
		 * beyond the bounds of our device..
		 */
		w = min((int)xres, w);
		h = min((int)yres, h);

		if (dssdrv->sched_update) {
			/* scheduled partial screen updates don't seem to work, at
			 * least for TAAL panel.. so for now always do full-screen
			 * updates.  Remove this when partial scheduled updates
			 * are handled properly in DSS.
			 */
			x = y = 0;
			w = xres;
			h = yres;

			dssdrv->sched_update(dssdev, x, y, w, h);
		} else if (dssdrv->update) {
			dssdrv->update(dssdev, x, y, w, h);
		}
	}
}

struct omap_dss_device *omap_connector_get_device(
		struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	return omap_connector->dssdev;
}

/* initialize connector */
struct drm_connector *omap_connector_init(struct drm_device *dev,
		int connector_type, struct omap_dss_device *dssdev)
{
	struct drm_connector *connector = NULL;
	struct omap_connector *omap_connector;

	DBG("%s", dssdev->name);

	omap_connector = kzalloc(sizeof(struct omap_connector), GFP_KERNEL);
	if (!omap_connector) {
		dev_err(dev->dev, "could not allocate connector\n");
		goto fail;
	}

	omap_connector->dssdev = dssdev;
	connector = &omap_connector->base;

	drm_connector_init(dev, connector, &omap_connector_funcs, connector_type);
	drm_connector_helper_add(connector, &omap_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_sysfs_connector_add(connector);

	omapdss_display_enable(dssdev);

	return connector;

fail:
	if (connector) {
		drm_connector_cleanup(connector);
		kfree(omap_connector);
	}

	return NULL;
}
