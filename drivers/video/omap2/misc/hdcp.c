/*
 * hdcp.c
 *
 * HDCP support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Fabrice Olivero <f-olivero@ti.com>
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <plat/display.h>
#include "../dss/hdmi.h"

#define DEBUG        /* Comment to remove debug printk */
/*#define DSS_INACTIVITY     Un-comment when DSS inactivity will be supported */

#include "hdcp.h"
#include "../dss/dss.h"

enum hdcp_states {
	HDCP_DISABLED,
	HDCP_ENABLED
};

enum hdmi_states {
	HDMI_STOPPED,
	HDMI_STARTED
};

struct hdcp_delayed_work {
	struct delayed_work work;
	int event;
};

static struct hdcp {
	void __iomem *hdmi_wp_base_addr;
	void __iomem *deshdcp_base_addr;
	struct mutex lock;
	struct mutex cb_lock;
	struct hdcp_enable_control *en_ctrl;
	dev_t dev_id;
	struct class *hdcp_class;
	enum hdmi_states hdmi_state;
	enum hdcp_states hdcp_state;
	enum hdcp_states request;
	struct delayed_work *pending_start;
	int retry_cnt;
	int auth_fail_restart;
	int auth_fail;
	int auth_done;
	struct omap_dss_device *dss;
} hdcp;

/*-----------------------------------------------------------------------------
 * Function: hdcp_3des_enc_key
 *-----------------------------------------------------------------------------
 */
static void hdcp_3des_enc_key(struct hdcp_encrypt_control *enc_ctrl,
                  uint32_t out_key[DESHDCP_KEY_SIZE])
{
	int counter = 0;

	pr_debug("Encrypting HDCP keys...");

	hdcp_request_dss();

	/* Reset encrypted key array */
	for (counter = 0; counter < DESHDCP_KEY_SIZE; counter++)
		out_key[counter] = 0;

	/* Set encryption mode in DES control register */
	WR_FIELD_32(hdcp.deshdcp_base_addr,
		DESHDCP__DHDCP_CTRL,
		DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		0x1);

	/* Write raw data and read encrypted data */
	counter = 0;

	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			enc_ctrl->in_key[counter]);
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			enc_ctrl->in_key[counter + 1]);

		/* Wait for output bit at '1' */
		while( RD_FIELD_32(hdcp.deshdcp_base_addr,
				DESHDCP__DHDCP_CTRL,
				DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
				DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L
			) != 0x1) {};

			/* Read enrypted data */
			out_key[counter] = RD_REG_32(hdcp.deshdcp_base_addr,
				DESHDCP__DHDCP_DATA_L);
			out_key[counter + 1] = RD_REG_32(hdcp.deshdcp_base_addr,
				 DESHDCP__DHDCP_DATA_H);

		counter+=2;
	}

	hdcp_release_dss();
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {

	case HDCP_DISABLE:
		pr_debug("hdcp_ioctl() - HDCP DISABLE");
		break;

	case HDCP_ENCRYPT_KEY: {
		struct hdcp_encrypt_control *ctrl;
		uint32_t *out_key;

		pr_debug("hdcp_ioctl() - ENCRYPT KEY");

		mutex_lock(&hdcp.lock);

		if (hdcp.hdcp_state == HDCP_ENABLED) {
			printk(KERN_ERR "HDCP: Cannot encrypt keys while HDCP "
				"is enabled");
			return -EFAULT;
		}

		mutex_unlock(&hdcp.lock);

		/* Encryption happens in ioctl / user context */
		ctrl = kmalloc(sizeof(struct hdcp_encrypt_control), GFP_KERNEL);

		if (ctrl == 0) {
			printk(KERN_ERR "HDCP: Cannot allocate memory for HDCP"
			" encryption control struct");
			return -EFAULT;
		}

		out_key = kmalloc(sizeof(uint32_t) * DESHDCP_KEY_SIZE, GFP_KERNEL);

		if (out_key == 0) {
			printk(KERN_ERR "HDCP: Cannot allocate memory for HDCP "
			"encryption output key");
			kfree(ctrl);
			return -EFAULT;
		}

		if (copy_from_user(ctrl, argp,
			sizeof(struct hdcp_encrypt_control))) {
			printk(KERN_ERR "HDCP: Error copying from user space"
			" - encrypt ioctl");

			kfree(ctrl);
			kfree(out_key);
			return -EFAULT;
		}

		/* README -- Must enable clocks or key provisioning will fail */

		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M | DSS_CLK_96M);

		/* Call encrypt function */
		hdcp_3des_enc_key(ctrl, out_key);

		/* Store output data to output pointer */
		if (copy_to_user(ctrl->out_key, out_key,
			sizeof(uint32_t)*DESHDCP_KEY_SIZE)) {
			printk(KERN_ERR "HDCP: Error copying to user space -"
			" encrypt ioctl");

			kfree(ctrl);
			kfree(out_key);
			return -EFAULT;
		}

		kfree(ctrl);
		kfree(out_key);
		return 0;
	}
	break;

	default:
		return -ENOTTY;
	} /* End switch */

	return 0;
}


static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hdcp_ioctl,
};

static struct cdev hdcp_cdev;


/*-----------------------------------------------------------------------------
 * Function: hdcp_init
 *-----------------------------------------------------------------------------
 */
static int __init hdcp_init(void)
{
	pr_debug("hdcp_init()");

	/* Map HDMI WP address */
	hdcp.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!hdcp.hdmi_wp_base_addr) {
		printk(KERN_ERR "HDCP: HDMI WP IOremap error\n");
		return -EFAULT;
	}

	/* Map DESHDCP in kernel address space */
	hdcp.deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp.deshdcp_base_addr) {
		printk(KERN_ERR "HDCP: DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp.lock);
	mutex_init(&hdcp.cb_lock);

	/* Get the major number for this module */
	if (alloc_chrdev_region(&hdcp.dev_id, 0, 1, "hdcp")) {
		printk(KERN_ERR "HDCP: Cound not register character device\n");
		goto err_register;
	}

	/* Initialize character device */
	cdev_init(&hdcp_cdev, &hdcp_fops);
	hdcp_cdev.owner = THIS_MODULE;
	hdcp_cdev.ops = &hdcp_fops;

	/* add char driver */
	if (cdev_add(&hdcp_cdev, hdcp.dev_id, 1)) {
		printk(KERN_ERR "HDCP: Could not add character driver\n");
		goto err_add_driver;
	}


	/* Create device */
	hdcp.hdcp_class = class_create(THIS_MODULE, "hdcp");
	if (IS_ERR(hdcp.hdcp_class)) {
		printk(KERN_ERR "HDCP: Cannot create class\n");
		goto err_add_driver;
	}

	if (IS_ERR(device_create(hdcp.hdcp_class, NULL, hdcp.dev_id, NULL,
				    "hdcp"))) {
		printk(KERN_ERR "HDCP: Cannot create device\n");
		class_destroy(hdcp.hdcp_class);
		goto err_add_driver;
	}

	mutex_lock(&hdcp.lock);

	/* Variable init */
	hdcp.en_ctrl  = 0;
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.request = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.retry_cnt = 0;
	hdcp.auth_fail_restart = 0;
	hdcp.auth_fail = 0;
	hdcp.auth_done = 0;

	mutex_unlock(&hdcp.lock);

	return 0;

err_add_driver:
	cdev_del(&hdcp_cdev);

	unregister_chrdev_region(hdcp.dev_id, 1);

err_register:
	mutex_destroy(&hdcp.lock);
	mutex_destroy(&hdcp.cb_lock);

	iounmap(hdcp.deshdcp_base_addr);

err_map_deshdcp:
	iounmap(hdcp.hdmi_wp_base_addr);

	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_exit
 *-----------------------------------------------------------------------------
 */
static void __exit hdcp_exit(void)
{
	pr_debug("hdcp_exit()");

	mutex_lock(&hdcp.lock);

	if (hdcp.en_ctrl)
		kfree(hdcp.en_ctrl);

	/* Unmap HDMI WP / DESHDCP */
	iounmap(hdcp.hdmi_wp_base_addr);
	iounmap(hdcp.deshdcp_base_addr);

	/* Unregister char device */
	device_destroy(hdcp.hdcp_class, MKDEV(hdcp.dev_id, 0));
	class_destroy(hdcp.hdcp_class);
	cdev_del(&hdcp_cdev);
	unregister_chrdev_region(hdcp.dev_id, 1);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
	mutex_destroy(&hdcp.cb_lock);
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Fabrice Olivero");

