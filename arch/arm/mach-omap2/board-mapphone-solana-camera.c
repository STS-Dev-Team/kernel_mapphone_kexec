#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <plat/i2c.h>
#include <plat/omap-pm.h>
#include <linux/pm_runtime.h>

#include <asm/mach-types.h>

#include <media/ov8820.h>

//leave 5640 in here until we make correct 7720 driver
#include <media/ov5640.h>
// #include <media/ov772x.h>

#include "devices.h"
#include "../../../drivers/media/video/omap4iss/iss.h"

#include "control.h"
#include "mux.h"

//omapconf dumped registers on stock
// only 2 values changed to output,
// assuming cam1 pwrdn and cam2 pwrdn are correct
// not sure on reset
#define MAPPHONE_GPIO_CAM1_PWRDN	171
//#define MAPPHONE_GPIO_CAM2_REVERSE	47
#define MAPPHONE_GPIO_CAM2_PWRDN	171
#define MAPPHONE_GPIO_CAM_RESET		83

//This should be right
#define OV8820_I2C_ADDRESS   (0x36)

/*
 * Have to specifically enable i2c3 module or else it idles
 * its wired directly to camera i think
 * Also need auxclk0 and some gpios, copying what happens on stock
 */

static int mapphone_solana_ov_cam1_pre_poweron(struct v4l2_subdev *subdev) {
	int ret;
	// flip on some gpios
	gpio_direction_output(48, 1); // 48 is selection for ov8820, 171 is for ov7739?
	gpio_direction_output(83,1); //  cam_reset?, has to stay on for both cams 
	
	//enable auclk0...is on with both cams
	struct clk *clk = NULL;
	clk = clk_get(NULL, "auxclk0_ck");
	clk_enable(clk);
	if(ret) {
		pr_err("%s: could not get auxclk0_ck\n", __func__);
		return -1;
	}
				
	// enable i2c3 bus
	struct i2c_adapter *adapter;
	struct device *i2c_dev;
	adapter = i2c_get_adapter(3);
	if (!adapter) {
		pr_err("%s: could not get i2c3 adapter\n", __func__);
		return -1;
	} 

	i2c_dev = adapter->dev.parent;
	i2c_detect_ext_master(adapter);
	i2c_put_adapter(adapter);
	
	ret = pm_runtime_get_sync(i2c_dev);

	ret -= ret == 1;
	if (ret)
		dev_warn(i2c_dev, "%s: failed get sync %d\n", __func__, ret);

	return ret;
}

static int mapphone_solana_ov_cam1_post_poweroff(struct v4l2_subdev *subdev) {
	//flip off GPIOS
	gpio_free(48, 0);
	gpio_free(83,0);

	//disable clk
	struct clk *clk = NULL;
	clk = clk_get(NULL, "auxclk0_ck");
	clk_disable(clk);
	clk_put(clk);

	//enable i2c3 idle
	struct i2c_adapter *adapter;
	struct device *i2c_dev;
	adapter = i2c_get_adapter(3);

	i2c_dev = adapter->dev.parent;
	i2c_detect_ext_master(adapter);
	i2c_put_adapter(adapter);
	
	ret = pm_runtime_put_sync(i2c_dev);
	return 0;
	
return ret;
}

static struct ov8820_platform_data ov8820_cam1_platform_data = {
	.reg_avdd = "vcam",	/* changed from cam2pwr */
	.reg_dovdd = "vwlan1",	// possibly hardwired, but vwlan1 turns on w/cam on stock

	.clk_xvclk = "auxclk1_ck",

	.gpio_pwdn = -1, // take care of this in pre/postpower as it requires specific stuff
	.is_gpio_pwdn_acthi = 1,
	.gpio_resetb = -1,

	//need more clocks and i2c3 bus enabled
	.pre_poweron = mapphone_solana_ov_cam1_pre_poweron,
	.post_poweroff = mapphone_solana_ov_cam1_post_poweroff,
};

static struct i2c_board_info ov8820_cam1_i2c_device = {
	I2C_BOARD_INFO("ov8820", OV8820_I2C_ADDRESS),
	.platform_data = &ov8820_cam1_platform_data,
};

//This should be right
#define OV772X_I2C_ADDRESS   (0x3c)

static struct ov5640_platform_data ov772x_cam2_platform_data = {
	.reg_avdd = "vcam",	/* changed from cam2pwr */
	.reg_dovdd = NULL,	/* Hardwired on */

	.clk_xvclk = "auxclk2_ck",

	.gpio_pwdn = MAPPHONE_GPIO_CAM2_PWRDN,
	.is_gpio_pwdn_acthi = 1,
	.gpio_resetb = MAPPHONE_GPIO_CAM_RESET,

	.pre_poweron = NULL,
	.post_poweroff = NULL,
};

static struct i2c_board_info ov772x_cam2_i2c_device = {
	I2C_BOARD_INFO("ov772x", OV772X_I2C_ADDRESS),
	.platform_data = &ov772x_cam2_platform_data,
};

//These should stay 3
static struct iss_subdev_i2c_board_info ov8820_cam1_subdevs[] = {
	{
		.board_info = &ov8820_cam1_i2c_device,
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct iss_subdev_i2c_board_info ov772x_cam2_subdevs[] = {
	{
		.board_info = &ov772x_cam2_i2c_device,
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct iss_v4l2_subdevs_group mapphone_camera_subdevs[] = {
	{
		.subdevs = ov8820_cam1_subdevs,
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

	{
		.subdevs = ov772x_cam2_subdevs,
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

//pads should be good..worthless if we cant get it to turn on though
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

	// fref_clk0_out -- turns on for both cams, might be necessary
	omap_mux_init_signal("fref_clk0_out", OMAP_PIN_OUTPUT);

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
