/*
 * drivers/video/omap2/misc/dssmgr-oly.c
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
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <plat/display.h>
#include <plat/dma.h>
#include <mach/tiler.h>

#include <linux/omap-dssmgr.h>
#include "dssmgr-oly.h"

/*#define DMO_DEBUG*/
#ifdef DMO_DEBUG
#define DMODBG(format, ...) \
	printk(KERN_DEBUG "DSSMGR: " format, ## __VA_ARGS__)
#else
#define DMODBG(format, ...)
#endif

#define DMA_TX_TIMEOUT   (1000)

struct dssmgr_oly_data *g_data;

static int dssmgr_oly_override_and_set(struct dssmgr_oly_data *data,
				int idx, struct omap_overlay_info *info);
static int dssmgr_oly_set_info(struct dssmgr_oly_data *data,
				struct omap_overlay *oly,
				struct omap_overlay_info *info);

/*=== Local Functions ==================================================*/

static int dssmgr_oly_set_and_apply(struct dssmgr_oly_data *data, uint8_t idx)
{
	struct omap_overlay_info info;
	int rc = 0;

	if (data->mirror.dst_idx != idx)
		memcpy(&info, &data->info[idx], sizeof(info));
	else
		data->flds[idx].get_func(data->olys[idx], &info);

	rc = dssmgr_oly_override_and_set(data, idx, &info);
	if (rc != 0)
		printk(KERN_ERR "DSSMGR-OLY: Set Info Failure\n");
	else
		rc = data->olys[idx]->manager->apply(
					data->olys[idx]->manager);

	return rc;
}

static void dssmgr_oly_get_dpy_size(struct dssmgr_oly_data *data,
						uint8_t idx, int *w, int *h)
{
	if (data->olys[idx]->manager->device) {
		*w = data->olys[idx]->manager->device->panel.timings.x_res;
		*h = data->olys[idx]->manager->device->panel.timings.y_res;
	} else { /* Assume WB */
		struct omap_writeback *wb;
		wb = omap_dss_get_wb(0);
		if (wb) {
			*w = wb->info.height;
			*h = wb->info.width;
		} else {
			printk(KERN_ERR "DSSMGR-OLY: No Device or Panel\n");
			/* FIXME: arbitary defaults */
			*w = 640;
			*h = 480;
		}
	}
}

static void dssmgr_oly_get_norm_input(struct omap_overlay_info *info,
						int *in_w, int *in_h)
{
	int rot = info->rotation;

	if (rot == OMAP_DSS_ROT_90 || rot == OMAP_DSS_ROT_270) {
		*in_w  = info->height;
		*in_h  = info->width;
	} else {
		*in_w  = info->width;
		*in_h  = info->height;
	}
}

static void dssmgr_oly_rot_input(int rot, int *in_w, int *in_h)
{
	int t;

	if (rot == OMAP_DSS_ROT_90 || rot == OMAP_DSS_ROT_270) {
		t      = *in_w;
		*in_w  = *in_h;
		*in_h  = t;
	}
}

static int dssmgr_oly_mirror_init(struct dssmgr_oly_data *data, uint8_t idx,
					struct omap_overlay_info *info)
{
	struct dssmgr_oly_mirror *mirror;
	enum tiler_fmt            fmt;
	int rc = -1;
	int cnt;
	int bytespp;
	u32 stride;

	mirror = &data->mirror;

	mirror->w    = info->width;
	mirror->h    = info->height;
	mirror->mode = info->color_mode;

	switch (mirror->mode) {
	case OMAP_DSS_COLOR_ARGB16:
	case OMAP_DSS_COLOR_RGB16:
		fmt = TILFMT_16BIT;
		bytespp = 2;
		break;
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		fmt = TILFMT_32BIT;
		bytespp = 4;
		break;
	default:
		printk(KERN_ERR "DSSMGR-OLY: Invalid Mirror Color Format\n");
		goto failed;
		break;
	}

	cnt = 2;

	tiler_alloc_packed(&cnt, fmt, mirror->w, mirror->h,
					(void **) mirror->buf,
					(void **) mirror->buf_alloc, 1);
	if (cnt != 2)
		printk(KERN_ERR "DSSMGR-OLY: Failed Mirror Alloc\n");
	else
		rc = 0;

	mirror->src_idx = idx;
	mirror->dst_idx = data->flds[idx].force_mirror_dst_idx;
	mirror->buf_idx = 0;

	mirror->dma_en = (mirror->w * bytespp) / 4; /* 32 bit ES */
	mirror->dma_fn = mirror->h;
	mirror->dma_src_fi = (info->screen_width * bytespp)
						- (mirror->dma_en * 4) + 1;
	stride = tiler_stride(tiler_get_natural_addr((void *) mirror->buf[0]));
	mirror->dma_dst_fi = stride - (mirror->dma_en * 4) + 1;;

failed:
	return rc;
}

static void dssmgr_oly_mirror_term(struct dssmgr_oly_data *data, uint8_t idx)
{
	struct dssmgr_oly_mirror *mirror;
	int i;

	mirror = &data->mirror;

	for (i = 0; i < 2; i++) {
		if (mirror->buf_alloc[i])
			tiler_free(mirror->buf_alloc[i]);
		mirror->buf_idx = 0;
		mirror->buf[i] = 0;
		mirror->buf_alloc[i] = 0;
	}

	mirror->src_idx = DSSMGR_OLY_INVALID_IDX;
	mirror->dst_idx = DSSMGR_OLY_INVALID_IDX;
}

static int dssmgr_oly_mirror_dma(struct dssmgr_oly_data *data, uint8_t s_idx)
{
	struct dssmgr_oly_mirror *mirror;
	u32 src_paddr;
	u32 dst_paddr;

	mirror = &data->mirror;
	mirror->buf_idx = (mirror->buf_idx) ? 0 : 1;

	src_paddr = data->info[s_idx].paddr;
	dst_paddr = mirror->buf[mirror->buf_idx];

	omap_set_dma_transfer_params(mirror->dma_ch, OMAP_DMA_DATA_TYPE_S32,
			mirror->dma_en, mirror->dma_fn, OMAP_DMA_SYNC_ELEMENT,
			mirror->dma_id, 0x0);
	omap_set_dma_src_params(mirror->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			src_paddr, 1, mirror->dma_src_fi);
	omap_set_dma_src_data_pack(mirror->dma_ch, 1);
	omap_set_dma_src_burst_mode(mirror->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_params(mirror->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			dst_paddr, 1, mirror->dma_dst_fi);
	omap_set_dma_dest_data_pack(mirror->dma_ch, 1);
	omap_set_dma_dest_burst_mode(mirror->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_dma_set_prio_lch(mirror->dma_ch, 1, 1);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0xFF, 0);

	mirror->dma_complete = false;
	omap_start_dma(mirror->dma_ch);
	wait_event_interruptible_timeout(mirror->dma_wait,
					mirror->dma_complete, DMA_TX_TIMEOUT);

	if (!mirror->dma_complete) {
		printk(KERN_WARNING "DSSMGR-OLY: DMA timeout\n");
		omap_stop_dma(mirror->dma_ch);
		return -EIO;
	}

	return 0;
}

/* This functions wakes up the application once the DMA transfer to
 * Tiler space is completed.
 */
static void dssmgr_oly_mirror_dma_cb(int lch, u16 ch_status, void *pdata)
{
	struct dssmgr_oly_data *data;

	data = (struct dssmgr_oly_data *) pdata;

	data->mirror.dma_complete = true;
	wake_up_interruptible(&data->mirror.dma_wait);
}

static void dssmgr_oly_mirror(struct dssmgr_oly_data *data, uint8_t s_idx)
{
	struct omap_overlay_info  s_info, d_info;
	struct dssmgr_oly_mirror *mirror;
	struct omap_overlay      *oly;
	int rc = 0;
	int w, h;
	uint8_t d_idx;

	/* Assumptions: The source overlay is the non-tiler based GFX plane */

	d_idx = data->flds[s_idx].force_mirror_dst_idx;

	if (data->mirror.src_idx != DSSMGR_OLY_INVALID_IDX
					&& s_idx != data->mirror.src_idx) {
		/* Ignore, no mirroring for this overlay */
	} else if (d_idx != DSSMGR_OLY_INVALID_IDX) {
		/* Handle the case where Mirroring is enabled */

		/* FIXME: should chk if important things have changed,
		 * like width, height, etc.  No needed now since we
		 * know the source overlay has a static size
		 */

		mirror = &data->mirror;

		data->flds[d_idx].get_func(data->olys[d_idx], &d_info);

		if (mirror->buf_alloc[0] == 0) {
			data->flds[s_idx].get_func(data->olys[s_idx], &s_info);
			rc = dssmgr_oly_mirror_init(data, s_idx, &s_info);

			/* Setup the initial dst overlay info */
			memset(&d_info, 0, sizeof(d_info));
			d_info.enabled       = true;
			d_info.width         = mirror->w;
			d_info.height        = mirror->h;
			d_info.screen_width  = d_info.width;
			d_info.color_mode    = mirror->mode;
			d_info.global_alpha  = 0xff;
			d_info.zorder        = 3; /* Hardcoded for now */

			/* Just config for non-rotated full screen, it will be
			 * adjusted later based on overrides
			 */
			d_info.rotation      = OMAP_DSS_ROT_0;
			d_info.rotation_type = OMAP_DSS_ROT_TILER;
			d_info.pos_x         = 0;
			d_info.pos_y         = 0;
			dssmgr_oly_get_dpy_size(data, d_idx, &w, &h);
			d_info.out_width     = w;
			d_info.out_height    = h;

			/* Just to keep TI's hack happy */
			d_info.min_x_decim = 1;
			d_info.max_x_decim = 1;
			d_info.min_y_decim = 1;
			d_info.max_y_decim = 1;
		}

		/* If mirroring is active and the destination overlay is not
		 * already enabled by the client, perform the mirroring
		 */
		if (rc != 0) {
			/* Ignore the frame */
		} else if (mirror->excl_idx != DSSMGR_OLY_INVALID_IDX
				&& data->info[mirror->excl_idx].enabled) {
			oly = data->olys[d_idx];
			data->flds[d_idx].get_func(oly, &d_info);
			if (d_info.enabled != false) {
				d_info.enabled = false;
				data->flds[d_idx].set_func(oly,	&d_info);
				oly->manager->apply(oly->manager);
			}
		} else if (!data->info[d_idx].enabled) {
			dssmgr_oly_mirror_dma(data, s_idx);

			/* Update the frame pointer */
			d_info.enabled = true;
			d_info.paddr   = mirror->buf[mirror->buf_idx];

			rc = dssmgr_oly_override_and_set(data, d_idx, &d_info);
			if (rc != 0)
				printk(KERN_ERR "DSSMGR-OLY: Mirror Failed\n");
			else
				rc = data->olys[d_idx]->manager->apply(
						data->olys[d_idx]->manager);
		}

	} else if (data->mirror.buf_alloc[0] != 0) {
		/* Handle the case where Mirroring is just disabled */
		d_idx = data->mirror.dst_idx;
		dssmgr_oly_mirror_term(data, s_idx);
		dssmgr_oly_set_and_apply(data, d_idx);
	}
}

static void dssmgr_oly_scale(int scale, int percent,
				int dpy_w, int dpy_h, int in_w, int in_h,
				int *out_x, int *out_y, int *out_w, int *out_h)
{
	int w, h;
	int tw, th;
	int ratio;
	const int shift = 10; /* Fixed point shift factor */
	const int ratio_limit = (4 << shift); /* 4X scaling limit */

	if (scale == DSSMGR_SCALE_FILL_SCREEN) {
		*out_x = 0;
		*out_y = 0;
		*out_w = dpy_w;
		*out_h = dpy_h;
	} else if (scale == DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN) {
		w = (dpy_w * percent)/100;
		h = (dpy_h * percent)/100;
		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_FIT_TO_SCREEN) {
		tw = (dpy_w << shift) / in_w;
		th = (dpy_h << shift) / in_h;
		ratio = (tw < th) ? tw : th;
		if (ratio > ratio_limit)
			ratio = ratio_limit;
		w = (in_w * ratio) >> shift;
		h = (in_h * ratio) >> shift;
		w = (w + 1) & ~1;
		h = (h + 1) & ~1;

		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN) {
		tw = (((dpy_w * percent)/100) << shift) / in_w;
		th = (((dpy_h * percent)/100) << shift) / in_h;
		ratio = (tw < th) ? tw : th;
		if (ratio > ratio_limit)
			ratio = ratio_limit;
		w = (in_w * ratio) >> shift;
		h = (in_h * ratio) >> shift;
		w = (w + 1) & ~1;
		h = (h + 1) & ~1;

		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	}
}

static void dssmgr_oly_override(struct dssmgr_oly_data *data, uint8_t idx,
				struct omap_overlay_info *info)
{
	struct dssmgr_oly_flds   *flds;
	struct omap_overlay_info  m_info;
	struct omap_overlay      *oly;
	int in_w, in_h;
	int out_x, out_y, out_w, out_h;
	int dpy_w, dpy_h;
	int rot;
	int m_idx;

	flds = &data->flds[idx];

	if (data->client_frame && flds->force_enable_nf)
		info->enabled = true;

	if (flds->force_disable)
		info->enabled = false;

	/* Gather some info */
	dssmgr_oly_get_dpy_size(data, idx, &dpy_w, &dpy_h);

	in_w  = info->width;
	in_h  = info->height;
	rot   = info->rotation;
	out_x = info->pos_x;
	out_y = info->pos_y;
	out_w = info->out_width;
	out_h = info->out_height;

	if (info->rotation_type == OMAP_DSS_ROT_TILER) {
		dssmgr_oly_get_norm_input(info, &in_w, &in_h);

		if (flds->force_rotation != DSSMGR_ROTATION_IGNORE) {
			switch (flds->force_rotation) {
			case DSSMGR_ROTATION_90:
				rot = OMAP_DSS_ROT_90;
				break;
			case DSSMGR_ROTATION_180:
				rot = OMAP_DSS_ROT_180;
				break;
			case DSSMGR_ROTATION_270:
				rot = OMAP_DSS_ROT_270;
				break;
			case DSSMGR_ROTATION_0:
			default:
				rot = OMAP_DSS_ROT_0;
				break;
			}
		}
		dssmgr_oly_rot_input(rot, &in_w, &in_h);
	}

	dssmgr_oly_scale(flds->force_scale, flds->force_scale_percent,
				dpy_w, dpy_h, in_w, in_h,
				&out_x, &out_y, &out_w, &out_h);

	info->width        = in_w;
	info->height       = in_h;
	info->rotation     = rot;
	info->pos_x        = out_x;
	info->pos_y        = out_y;
	info->out_width    = out_w;
	info->out_height   = out_h;

	if (info->rotation_type == OMAP_DSS_ROT_TILER)
		info->screen_width = info->width;

	dssmgr_oly_mirror(data, idx);

	/* Check if the mirror exclusive overlay is active, if so, disable the
	 * mirror overlay
	 */
	if (data->mirror.src_idx != DSSMGR_OLY_INVALID_IDX &&
			idx == data->mirror.excl_idx && info->enabled) {
		m_idx = data->mirror.dst_idx;
		oly = data->olys[m_idx];
		if (oly->info.enabled != false) {
			data->flds[m_idx].get_func(oly, &m_info);
			m_info.enabled = false;
			data->flds[m_idx].set_func(oly, &m_info);
			oly->manager->apply(oly->manager);
		}
	}
}

/* Similar to "dssmgr_oly_set_info" except the info is not from the client */
static int dssmgr_oly_override_and_set(struct dssmgr_oly_data *data, int idx,
					struct omap_overlay_info *info)
{
	/* Override as needed */
	dssmgr_oly_override(data, idx, info);

	/* Set the info to the DSS */
	return data->flds[idx].set_func(data->olys[idx], info);
}

static int dssmgr_oly_set_info(struct dssmgr_oly_data *data,
				struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	struct omap_overlay_info local_info;
	int rc = 0;
	int i;
	uint8_t idx = DSSMGR_OLY_INVALID_IDX;

	for (i = 0; i < DSSMGR_MAX_OVERLAYS; i++) {
		if (data->olys[i] == oly) {
			idx = i;
			break;
		}
	}

	if (idx == DSSMGR_OLY_INVALID_IDX) {
		rc = -EINVAL;
		goto failed;
	}

	data->flds[idx].fcnt++;

	/* Save as most recent client provide info */
	memcpy(&data->info[idx], info, sizeof(*info));

	/* Don't use the client provided pointer since we may change the data
	 * and that may be reflected in the client kept data structure
	 */
	memcpy(&local_info, info, sizeof(*info));

	rc = dssmgr_oly_override_and_set(data, idx, &local_info);

failed:
	return rc;
}

/* This is the substitute "set_overlay_info" function.
 * All calls to the "set_overlay_info" function of any overlay will
 * come here, so the overlay info can be adjusted as requested by
 * the client
 */
static int dssmgr_oly_set_info_lock(struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	int rc = 0;

	mutex_lock(&g_data->mtx);

	g_data->client_frame = 1;

	rc = dssmgr_oly_set_info(g_data, oly, info);

	g_data->client_frame = 0;

	mutex_unlock(&g_data->mtx);

	return rc;
}

/* This is the substitute "get_overlay_info" function.
 * All calls to the "get_overlay_info" function of any overlay will
 * come here, so the correct overlay info can be returned
 */
static void dssmgr_oly_get_info_lock(struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	int i;

	mutex_lock(&g_data->mtx);

	for (i = 0; i < DSSMGR_MAX_OVERLAYS; i++) {
		if (g_data->olys[i] == oly) {
			/* Return most recently set info */
			memcpy(info, &g_data->info[i], sizeof(*info));
			break;
		}
	}

	mutex_unlock(&g_data->mtx);
}

void dssmgr_oly_reset_oly_idx(struct dssmgr_oly_data *data, uint8_t idx)
{
	data->flds[idx].force_disable        = 0;
	data->flds[idx].force_rotation       = DSSMGR_ROTATION_IGNORE;
	data->flds[idx].force_scale          = DSSMGR_SCALE_IGNORE;
	data->flds[idx].force_mirror_dst_idx = DSSMGR_OLY_INVALID_IDX;
	data->flds[idx].force_enable_nf      = 0;
	data->flds[idx].force_exclusive      = 0;
	data->flds[idx].fcnt                 = 0;
	if (data->mirror.excl_idx == idx)
		data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
}

/*=== Interface Functions ==============================================*/

int dssmgr_oly_init(struct dssmgr_oly_data *data)
{
	struct omap_overlay *oly;
	int rc = 0;
	int num;
	int i;

	BUG_ON(data == NULL);

	if (g_data != NULL) {
		printk(KERN_ERR "dssmgr-oly already init'd\n");
		rc = -EINVAL;
		goto failed;
	}

	/* Store for use in the set/get_info_lock functions */
	g_data = data;

	mutex_init(&data->mtx);

	/* Alloc the mirror DMA channel */
	data->mirror.dma_id = OMAP_DMA_NO_DEVICE;
	data->mirror.dma_ch = -1;
	rc = omap_request_dma(data->mirror.dma_id, "MIRROR DMA",
				dssmgr_oly_mirror_dma_cb,
				(void *) data,
				&data->mirror.dma_ch);
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR OLY: DMA Alloc Failed\n");
		goto failed_dma;
	}
	init_waitqueue_head(&data->mirror.dma_wait);

	/* Hook into the set/get_overlay_info call paths */
	num = omap_dss_get_num_overlays();
	num = (num < DSSMGR_MAX_OVERLAYS) ? num : DSSMGR_MAX_OVERLAYS;
	for (i = 0; i < num; i++) {
		oly = omap_dss_get_overlay(i);
		data->olys[i] = oly;
		memset(&data->flds[i], 0, sizeof(data->flds[i]));
		data->flds[i].set_func = oly->set_overlay_info;
		data->flds[i].get_func = oly->get_overlay_info;
		oly->set_overlay_info = dssmgr_oly_set_info_lock;
		oly->get_overlay_info = dssmgr_oly_get_info_lock;
		data->flds[i].get_func(oly, &data->info[i]);
	}
	data->num = num;

	/* Reset the oly fields */
	for (i = 0; i < data->num; i++)
		dssmgr_oly_reset_oly_idx(data, i);

	/* Init the mirror indexes */
	data->mirror.src_idx  = DSSMGR_OLY_INVALID_IDX;
	data->mirror.dst_idx  = DSSMGR_OLY_INVALID_IDX;
	data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;

	return 0;

failed_dma:
	g_data = NULL;
failed:
	return rc;
}

void dssmgr_oly_remove(struct dssmgr_oly_data *data)
{
	struct omap_overlay *oly;
	int i;

	BUG_ON(data == NULL);

	if (g_data) {
		omap_free_dma(data->mirror.dma_ch);

		for (i = 0; i < data->num; i++) {
			oly = data->olys[i];
			oly->set_overlay_info = data->flds[i].set_func;
			oly->get_overlay_info = data->flds[i].get_func;
			oly->set_overlay_info(oly, &data->info[i]);
		}

		g_data = NULL;
	}
}

void dssmgr_oly_set_client_id(struct dssmgr_oly_data *data, u32 id)
{
	BUG_ON(data == NULL);

	DMODBG("dssmgr_oly_set_client_id/%d\n", id);

	mutex_lock(&data->mtx);

	data->client_id = id;

	mutex_unlock(&data->mtx);
}

void dssmgr_oly_reset(struct dssmgr_oly_data *data)
{
	int i;

	BUG_ON(data == NULL);

	for (i = 0; i < data->num; i++)
		dssmgr_oly_reset_oly(data, i);
}

void dssmgr_oly_reset_oly(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	DMODBG("dssmgr_oly_reset_oly/%d\n", idx);

	mutex_lock(&data->mtx);

	dssmgr_oly_reset_oly_idx(data, idx);
	dssmgr_oly_set_and_apply(data, idx);

	mutex_unlock(&data->mtx);
}

void dssmgr_oly_reset_client(struct dssmgr_oly_data *data, u32 id)
{
	struct dssmgr_oly_flds *flds;
	int i;
	int dirty;

	BUG_ON(data == NULL);

	DMODBG("dssmgr_oly_reset_client/%d\n", id);

	for (i = 0; i < data->num; i++) {
		dirty = 0;
		flds = &data->flds[i];

		if (flds->disable_client_id == id) {
			flds->force_disable = 0;
			dirty = 1;
		}
		if (flds->rotation_client_id == id) {
			flds->force_rotation = DSSMGR_ROTATION_IGNORE;
			dirty = 1;
		}
		if (flds->scale_client_id == id) {
			flds->force_scale = DSSMGR_SCALE_IGNORE;
			dirty = 1;
		}
		if (flds->mirror_client_id == id) {
			flds->force_mirror_dst_idx = DSSMGR_OLY_INVALID_IDX;
			dirty = 1;
		}
		if (flds->enable_nf_client_id == id) {
			flds->force_enable_nf = 0;
			dirty = 1;
		}
		if (flds->exclusive_client_id == id) {
			flds->force_exclusive = 0;
			if (data->mirror.excl_idx == i)
				data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
			dirty = 1;
		}

		if (dirty)
			dssmgr_oly_set_and_apply(data, i);
	}
}

int dssmgr_oly_get_force_disable(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_disable;
}

int dssmgr_oly_set_force_disable(struct dssmgr_oly_data *data, uint8_t idx,
							uint8_t dis)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(dis > 1);

	/* Verify a change is occuring */
	if (data->flds[idx].force_disable != dis) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_disable = dis;
		data->flds[idx].disable_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_rotation(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_rotation;
}

int dssmgr_oly_set_force_rotation(struct dssmgr_oly_data *data, uint8_t idx,
						enum dssmgr_rotation rot)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	/* Verify a change is occuring */
	if (data->flds[idx].force_rotation != rot) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_rotation = rot;
		data->flds[idx].rotation_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_scale(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_scale;
}

int dssmgr_oly_set_force_scale(struct dssmgr_oly_data *data, uint8_t idx,
				enum dssmgr_scale scale, int percent)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	/* Verify a change is occuring */
	if (data->flds[idx].force_scale != scale
	   || data->flds[idx].force_scale_percent != (u32) percent) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_scale = scale;
		data->flds[idx].force_scale_percent = (u32) percent;
		data->flds[idx].scale_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_mirror(struct dssmgr_oly_data *data, uint8_t s_idx)
{
	BUG_ON(data == NULL);
	BUG_ON(s_idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[s_idx].force_mirror_dst_idx;
}

int dssmgr_oly_set_force_mirror(struct dssmgr_oly_data *data, uint8_t s_idx
						, uint8_t d_idx)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(s_idx >= DSSMGR_MAX_OVERLAYS);

	mutex_lock(&data->mtx);

	/* FIXME: For now only allow mirroring from the GFX plane to a
	 *        video plane.
	 */
	if (s_idx != 0) {
		rc = -EINVAL;
		goto failed;
	}

	/* FIXME: Only allow one mirror to occur at a time */
	if (d_idx != DSSMGR_OLY_INVALID_IDX &&
			data->mirror.dst_idx != DSSMGR_OLY_INVALID_IDX) {
		rc = -EBUSY;
		goto failed;
	}

	/* Verify a change is occuring */
	if (data->flds[s_idx].force_mirror_dst_idx != d_idx) {
		data->flds[s_idx].force_mirror_dst_idx = d_idx;
		data->flds[s_idx].mirror_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, s_idx);
	}

failed:
	mutex_unlock(&data->mtx);

	return rc;
}

int dssmgr_oly_get_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_enable_nf;
}

/* Force enable the overlay when the client pushes the next frame */
int dssmgr_oly_set_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(en > 1);

	/* Verify a change is occuring */
	if (data->flds[idx].force_enable_nf != en) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_enable_nf = en;
		data->flds[idx].enable_nf_client_id = data->client_id;

		/* No need to set & apply when enabled since its the next
		 * pushed frame that we will tigger off of
		 */
		if (!en)
			rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_exclusive;
}

int dssmgr_oly_set_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en)
{
	struct omap_overlay_info  info;
	struct omap_overlay      *oly;
	int rc = 0;
	int m_idx;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(en > 1);

	/* Verify a change is occuring */
	if (data->mirror.excl_idx != DSSMGR_OLY_INVALID_IDX &&
					data->mirror.excl_idx != idx) {
		rc = -EBUSY;
	} else if (data->flds[idx].force_exclusive != en) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_exclusive = en;
		data->flds[idx].enable_nf_client_id = data->client_id;

		if (en) {
			data->mirror.excl_idx = idx;

			/* Disable the mirrored overlay */
			m_idx = data->mirror.dst_idx;
			if (m_idx != DSSMGR_OLY_INVALID_IDX) {
				oly   = data->olys[m_idx];
				data->flds[m_idx].get_func(oly, &info);
				if (info.enabled != false) {
					info.enabled = false;
					data->flds[m_idx].set_func(oly, &info);
					oly->manager->apply(oly->manager);
				}
			}
		} else {
			data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
			/* The next mirror frame will re-enable the mirror */
		}

		mutex_unlock(&data->mtx);
	}

	return rc;
}

void dmmgr_oly_reconfig_for_dpy(struct dssmgr_oly_data *data
						, struct omap_dss_device *dpy)
{
	struct omap_overlay *oly;
	int i;

	BUG_ON(data == NULL);

	for (i = 0; i < data->num; i++) {
		oly = data->olys[i];
		if (oly->manager == NULL || oly->manager->device == NULL)
			continue;

		if (oly->manager->device == dpy)
			dssmgr_oly_set_and_apply(data, i);
	}
}

