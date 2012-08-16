#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <plat/i2c.h>
#include <plat/omap-pm.h>

#include <asm/mach-types.h>

// FIXME-HASH: need new sensor drivers
#include <media/ov5640.h>
// #include <media/ov772x.h>

#include "devices.h"
#include "../../../drivers/media/video/omap4iss/iss.h"

#include "control.h"
#include "mux.h"

#define MAPPHONE_GPIO_CAM1_PWRDN	37
#define MAPPHONE_GPIO_CAM2_REVERSE	47
#define MAPPHONE_GPIO_CAM2_PWRDN	171
#define MAPPHONE_GPIO_CAM_RESET		83

#define OV5640_I2C_ADDRESS   (0x3C)

/* FIXME-HASH: Disable read cam for now */
#if 0
static struct ov5640_platform_data ov5640_cam1_platform_data = {
	.reg_avdd = "vcam",	/* changed from cam2pwr */
	.reg_dovdd = NULL,	/* Hardwired on */

	.clk_xvclk = "auxclk2_ck",

	.gpio_pwdn = MAPPHONE_GPIO_CAM1_PWRDN,
	.is_gpio_pwdn_acthi = 1,
	.gpio_resetb = -1, /* Not connected */

	.pre_poweron = NULL,
	.post_poweroff = NULL,
};

static struct i2c_board_info ov5640_cam1_i2c_device = {
	I2C_BOARD_INFO("ov5640", OV5640_I2C_ADDRESS),
	.platform_data = &ov5640_cam1_platform_data,
};
#endif

#define OV772X_I2C_ADDRESS   (0x36)

static struct ov5640_platform_data ov772x_cam2_platform_data = {
	.reg_avdd = "vcam",	/* changed from cam2pwr */
	.reg_dovdd = NULL,	/* Hardwired on */

	.clk_xvclk = "auxclk2_ck",

	.gpio_pwdn = MAPPHONE_GPIO_CAM2_PWRDN,
	.is_gpio_pwdn_acthi = 1,
	.gpio_resetb = -1, /* Not connected */

	.pre_poweron = NULL,
	.post_poweroff = NULL,
};

static struct i2c_board_info ov772x_cam2_i2c_device = {
	I2C_BOARD_INFO("ov772x", OV772X_I2C_ADDRESS),
	.platform_data = &ov772x_cam2_platform_data,
};


#if 0
static struct iss_subdev_i2c_board_info ov5640_cam1_subdevs[] = {
	{
		.board_info = &ov5640_cam1_i2c_device,
		.i2c_adapter_id = 2,
	},
	{ NULL, 0, },
};
#endif

static struct iss_subdev_i2c_board_info ov772x_cam2_subdevs[] = {
	{
		.board_info = &ov772x_cam2_i2c_device,
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct iss_v4l2_subdevs_group mapphone_camera_subdevs[] = {
#if 0
	{
		.subdevs = ov5640_cam1_subdevs,
		.interface = ISS_INTERFACE_CSI2B_PHY2,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
#endif
	{
		.subdevs = ov772x_cam2_subdevs,
		.interface = ISS_INTERFACE_CSI2A_PHY1,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
	{ },
};

static void mapphone_omap4iss_set_constraints(struct iss_device *iss, bool enable)
{
	if (!iss)
		return;

	/* FIXME: Look for something more precise as a good throughtput limit */
	omap_pm_set_min_bus_tput(iss->dev, OCP_INITIATOR_AGENT,
				 enable ? 800000 : -1);
}

static struct iss_platform_data mapphone_iss_platform_data = {
	.subdevs = mapphone_camera_subdevs,
	.set_constraints = mapphone_omap4iss_set_constraints,
};

static struct omap_device_pad omap4iss_pads[] = {
	/* CSI2-A */
	{
		.name   = "csi21_dx0.csi21_dx0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy0.csi21_dy0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dx1.csi21_dx1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy1.csi21_dy1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dx2.csi21_dx2",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy2.csi21_dy2",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	/* CSI2-B */
	{
		.name   = "csi22_dx0.csi22_dx0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dy0.csi22_dy0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dx1.csi22_dx1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dy1.csi22_dy1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
};

static struct omap_board_data omap4iss_data = {
	.id	    		= 1,
	.pads	 		= omap4iss_pads,
	.pads_cnt       	= ARRAY_SIZE(omap4iss_pads),
};

static int __init mapphone_camera_init(void)
{
	omap_mux_init_gpio(MAPPHONE_GPIO_CAM1_PWRDN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(MAPPHONE_GPIO_CAM2_PWRDN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(MAPPHONE_GPIO_CAM_RESET, OMAP_PIN_OUTPUT);

	/* Init FREF_CLK1_OUT */
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);

	/* Init FREF_CLK2_OUT */
	omap_mux_init_signal("fref_clk2_out", OMAP_PIN_OUTPUT);

	/* FIXME: this should be operated in the sensor driver, not here */
	if (gpio_request_one( MAPPHONE_GPIO_CAM_RESET, GPIOF_OUT_INIT_HIGH, "CAM_RESET"))
		printk(KERN_WARNING "Cannot request GPIO %d\n", MAPPHONE_GPIO_CAM_RESET);

	return omap4_init_camera(&mapphone_iss_platform_data, &omap4iss_data);
}
late_initcall(mapphone_camera_init);
