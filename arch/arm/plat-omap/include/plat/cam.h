#ifndef __OMAP_CAM_H__
#define __OMAP_CAM_H__

#if defined(CONFIG_OMAP_ISS_MEM_SIZE)
extern void omap_cam_reserve_sdram_memblock(void);
#else
static inline void omap_cam_reserve_sdram_memblock(void) { }
#endif

#endif
