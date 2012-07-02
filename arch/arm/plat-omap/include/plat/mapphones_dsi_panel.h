#ifndef __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H
#define __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H

#include "display.h"

struct mapphone_dsi_panel_data {
	const char *name;

	int reset_gpio;

	bool use_ext_te;
	int ext_te_gpio;

	bool use_esd_check;

	atomic_t state;
	void *panel_handle;

	bool te_support;
	bool manual_te_trigger;
	u32 te_scan_line;
	enum omap_dsi_te_type te_type;

	int max_backlight_level;
	int (*set_backlight)(struct omap_dss_device *dssdev, int level);
	int (*get_backlight)(struct omap_dss_device *dssdev);
};

#endif /* __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H */


