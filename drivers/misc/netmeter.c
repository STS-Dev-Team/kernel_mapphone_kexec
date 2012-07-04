/*
 * drivers/misc/netmeter.c
 *
 * Copyright (C) 2010 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kmod.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/ip.h>
#include <linux/string.h>

#define DEVICE_NAME	"netmeter"

struct netmeter_data {
	struct miscdevice *device;
	int started;
	unsigned long long modem_rx;
	unsigned long long modem_tx;
	unsigned long long modem2_rx;
	unsigned long long modem2_tx;
	struct nf_hook_ops nfho_in;
	struct nf_hook_ops nfho_out;
	struct nf_hook_ops nfho_forward;
};

struct netmeter_data *g_dev;
static const char modem_devname_pre[] = "qmi";
static const char dun_devname[] = "ppp0";
static const char ap_devname[] = "tiap0";

static spinlock_t spin_modem_rx;
static spinlock_t spin_modem_tx;
static spinlock_t spin_modem2_rx;
static spinlock_t spin_modem2_tx;

static void set_modem_rx(unsigned long long , bool);
static void set_modem_tx(unsigned long long , bool);
static void set_modem2_rx(unsigned long long , bool);
static void set_modem2_tx(unsigned long long , bool);

/*=========================================================================*/
static unsigned int netmeter_nf_hook_in(unsigned int hooknum,
			struct sk_buff *skb,
			const struct net_device *in,
			const struct net_device *out,
			int (*okfn)(struct sk_buff *))
{
	if (0 == strncmp(modem_devname_pre, in->name,
		sizeof(modem_devname_pre)-1))
		set_modem_rx((unsigned long long)(skb->len), false);
	return NF_ACCEPT;
}

static unsigned int netmeter_nf_hook_out(unsigned int hooknum,
			struct sk_buff *skb,
			const struct net_device *in,
			const struct net_device *out,
			int (*okfn)(struct sk_buff *))
{
	if (0 == strncmp(modem_devname_pre, out->name,
		sizeof(modem_devname_pre)-1))
		set_modem_tx((unsigned long long)(skb->len), false);
	return NF_ACCEPT;
}

static unsigned int netmeter_nf_hook_forward(unsigned int hooknum,
			struct sk_buff *skb,
			const struct net_device *in,
			const struct net_device *out,
			int (*okfn)(struct sk_buff *))
{
	if ((0 == strncmp(modem_devname_pre, in->name,
		sizeof(modem_devname_pre)-1)) &&
		((0 == strncmp(dun_devname, out->name,
			sizeof(dun_devname)-1)) ||
		(0 == strncmp(ap_devname, out->name,
			sizeof(ap_devname)-1))))

		set_modem_rx((unsigned long long)(skb->len), false);

	else if (((0 == strncmp(dun_devname, in->name,
			sizeof(dun_devname)-1)) ||
		(0 == strncmp(ap_devname, in->name,
			sizeof(ap_devname)-1))) &&
		(0 == strncmp(modem_devname_pre, out->name,
			sizeof(modem_devname_pre)-1)))

		set_modem_tx((unsigned long long)(skb->len), false);
	return NF_ACCEPT;
}

static void set_modem_rx(unsigned long long data, bool reset_data)
{
	spin_lock(&spin_modem_rx);
	if (reset_data)
		g_dev->modem_rx = data;
	else
		g_dev->modem_rx += data;
	spin_unlock(&spin_modem_rx);

}

static void set_modem2_rx(unsigned long long data, bool reset_data)
{
	spin_lock(&spin_modem2_rx);
	if (reset_data)
		g_dev->modem2_rx = data;
	else
		g_dev->modem2_rx += data;
	spin_unlock(&spin_modem2_rx);

}
static void set_modem_tx(unsigned long long data, bool reset_data)
{
	spin_lock(&spin_modem_tx);
	if (reset_data)
		g_dev->modem_tx = data;
	else
		g_dev->modem_tx += data;
	spin_unlock(&spin_modem_tx);
}

static void set_modem2_tx(unsigned long long data, bool reset_data)
{
	spin_lock(&spin_modem2_tx);
	if (reset_data)
		g_dev->modem2_tx = data;
	else
		g_dev->modem2_tx += data;
	spin_unlock(&spin_modem2_tx);
}


static int update_nf_hooks(void)
{
	if (g_dev->started) {
		nf_register_hook(&(g_dev->nfho_in));
		nf_register_hook(&(g_dev->nfho_out));
		nf_register_hook(&(g_dev->nfho_forward));
	} else {
		nf_unregister_hook(&(g_dev->nfho_in));
		nf_unregister_hook(&(g_dev->nfho_out));
		nf_unregister_hook(&(g_dev->nfho_forward));
	}

	return 0;
}

/*==========================================================================*/

static ssize_t show_start(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_dev->started);
}

static ssize_t store_start(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long start;
	int r;

	r = strict_strtoul(buf, 10, &start);

	if (r < 0 || (start != 0 && start != 1))
		return -EINVAL;

	if (g_dev->started != start) {
		g_dev->started = start;
		r = update_nf_hooks();
	}
	if (r)
		return r;

	return count;
}



static ssize_t show_modem_rx(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", g_dev->modem_rx);
}

static ssize_t store_modem_rx(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long modem_rx;
	int r;

	r = strict_strtoul(buf, 10, &modem_rx);
	if (r < 0)
		return -EINVAL;
	set_modem_rx((unsigned long long)modem_rx, true);
	return count;
}


static ssize_t show_modem_tx(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", g_dev->modem_tx);
}

static ssize_t store_modem_tx(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long modem_tx;
	int r;

	r = strict_strtoul(buf, 10, &modem_tx);
	if (r < 0)
		return -EINVAL;
	set_modem_tx((unsigned long long)modem_tx, true);
	return count;
}


static ssize_t show_modem2_rx(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", g_dev->modem2_rx);
}

static ssize_t store_modem2_rx(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long modem2_rx;
	int r;

	r = strict_strtoul(buf, 10, &modem2_rx);
	if (r < 0)
		return -EINVAL;
	set_modem2_rx((unsigned long long)modem2_rx, true);
	return count;
}

static ssize_t show_modem2_tx(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", g_dev->modem2_tx);
}

static ssize_t store_modem2_tx(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long modem2_tx;
	int r;

	r = strict_strtoul(buf, 10, &modem2_tx);
	if (r < 0)
		return -EINVAL;
	set_modem2_tx((unsigned long long)modem2_tx, true);
	return count;
}


static struct device_attribute netmeter_attrs[] = {
	__ATTR(start, S_IRUGO | S_IWUGO, show_start, store_start),
	__ATTR(modem_rx, S_IRUGO | S_IWUGO, show_modem_rx, store_modem_rx),
	__ATTR(modem_tx, S_IRUGO | S_IWUGO, show_modem_tx, store_modem_tx),
	__ATTR(modem2_rx, S_IRUGO | S_IWUGO, show_modem2_rx, store_modem2_rx),
	__ATTR(modem2_tx, S_IRUGO | S_IWUGO, show_modem2_tx, store_modem2_tx),
};

int netmeter_create_sysfs(void)
{
	int r;
	int t;

	printk(KERN_DEBUG "create sysfs for netmeter\n");
	for (t = 0; t < ARRAY_SIZE(netmeter_attrs); t++) {
		r = device_create_file(g_dev->device->this_device,
				&netmeter_attrs[t]);

		if (r) {
			printk(KERN_ERR "failed to create sysfs file\n");
			return r;
		}
	}

	return 0;
}

void netmeter_remove_sysfs(void)
{
	int t;

	printk(KERN_DEBUG "remove sysfs\n");
	for (t = 0; t < ARRAY_SIZE(netmeter_attrs); t++)
		device_remove_file(g_dev->device->this_device,
				&netmeter_attrs[t]);
}


/*===========================================================================*/

static int netmeter_open(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "netmeter_open\n");
	return 0;
}

static int netmeter_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "netmeter_release\n");
	return 0;
}

static ssize_t netmeter_read(struct file *fp, char __user *buf,
						size_t count, loff_t *ppos)
{
	printk(KERN_DEBUG "netmeter_read\n");
	return 0;
}

static ssize_t netmeter_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	printk(KERN_DEBUG "netmeter_write\n");
	return 0;
}

static int netmeter_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	return 0;
}

static const struct file_operations netmeter_fops = {
	.owner = THIS_MODULE,
	.open = netmeter_open,
	.release = netmeter_release,
	.read = netmeter_read,
	.write = netmeter_write,
	.unlocked_ioctl = netmeter_ioctl,
};

static struct miscdevice netmeter_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &netmeter_fops,
};

static int __init netmeter_init(void)
{
	int rc = 0;
	printk(KERN_DEBUG "netmeter_init\n");

	g_dev = kzalloc(sizeof(struct netmeter_data), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	rc = misc_register(&netmeter_misc_device);
	if (rc) {
		printk(KERN_ERR "netmeter: misc register failed (%d)\n", rc);
		goto failed_misc;
	}
	g_dev->device = &netmeter_misc_device;

	g_dev->nfho_in.hook = netmeter_nf_hook_in;
	g_dev->nfho_in.pf = PF_INET;
	g_dev->nfho_in.hooknum = NF_INET_LOCAL_IN;
	g_dev->nfho_in.priority = NF_IP_PRI_FIRST;

	g_dev->nfho_out.hook = netmeter_nf_hook_out;
	g_dev->nfho_out.pf = PF_INET;
	g_dev->nfho_out.hooknum = NF_INET_LOCAL_OUT;
	g_dev->nfho_out.priority = NF_IP_PRI_FIRST;

	g_dev->nfho_forward.hook = netmeter_nf_hook_forward;
	g_dev->nfho_forward.pf = PF_INET;
	g_dev->nfho_forward.hooknum = NF_INET_FORWARD;
	g_dev->nfho_forward.priority = NF_IP_PRI_FIRST;

	g_dev->started = 1;

	spin_lock_init(&spin_modem_rx);
	spin_lock_init(&spin_modem_tx);
	spin_lock_init(&spin_modem2_rx);
	spin_lock_init(&spin_modem2_tx);
	update_nf_hooks();
	rc = netmeter_create_sysfs();
	if (rc)
		printk(KERN_ERR "netmeter: create sysfs failed");

	return 0;

failed_misc:
	printk(KERN_ERR "netmeter register failed (%d)\n", rc);
	kfree(g_dev);
	g_dev = NULL;
	return -ENODEV;
}

static void __exit netmeter_exit(void)
{
	printk(KERN_DEBUG "netmeter_exit\n");

	if (g_dev->started) {
		nf_unregister_hook(&(g_dev->nfho_in));
		nf_unregister_hook(&(g_dev->nfho_out));
	}

	netmeter_remove_sysfs();
	misc_deregister(&netmeter_misc_device);
	kfree(g_dev);
	g_dev = NULL;

}

module_init(netmeter_init);
module_exit(netmeter_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
