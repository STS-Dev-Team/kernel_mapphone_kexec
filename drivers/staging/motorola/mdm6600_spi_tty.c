/*
 * MDM6600 spi-tty device driver
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/hardirq.h>
#include <linux/wakelock.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/mdm6600_spi_tty.h>
#include <linux/radio_ctrl/mdm6600_ctrl.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>

#define SPI_TRANSACTION_LEN 16256
#define SPI_TTY_MINORS		1


#define SPI_TTY_BUF_SIZE (128*1024) /* size should be power of 2 */

/*
 * 2s wakelock timeout, because everything is done in kthread,
 * we give it a little bit more time
 */
#define SPI_TTY_WAKE_LOCK_TIMEOUT (2*HZ)


#define SPI_MSG_HEADER_LEN 16
#define SPI_MTU (SPI_TRANSACTION_LEN - SPI_MSG_HEADER_LEN)
#define REQUEST_BLOCK_UNIT  1792
#define SPI_IPC_DEBUG 0

#define MRDY_IRQ_DISABLE	0
#define MRDY_IRQ_ENABLE		1

#define SPI_TTY_START	0
#define SPI_TTY_FINISH	1

#if SPI_IPC_DEBUG
#include <linux/sched.h>
#include <linux/ctype.h>

#define SPI_IPC_INFO(fmt, args...) do { \
	printk(KERN_INFO"[%d] "fmt, current->pid, ## args); \
	} while (0)

#define spi_ipc_buf_dump(header, buf, len) do { \
	spi_ipc_buf_dump1(header, buf, len, 0); \
	} while (0)

#define spi_ipc_buf_dump_ascii(header, buf, len) do { \
	spi_ipc_buf_dump1(header, buf, len, 1); \
	} while (0)

#else
#define SPI_IPC_INFO(fmt, args...) do {} while (0)
#define spi_ipc_buf_dump(header, buf, len) do {} while (0)
#define spi_ipc_buf_dump_ascii(header, buf, len) do {} while (0)
#endif

struct spi_tty_msg {
	__le32 type;
	__le32 len;
	__le32 dtr;
	__le32 fcs;
	u8   data[SPI_MTU];
};

struct mdm6600_spi_tty_device {

	struct spi_device			*spi;

	struct mdm6600_spi_platform_data *pdata;
	/*
	 *  mrdy is used by spi master to:
	 *  a) As as a response for SPI slave initiated transaction, tell slave
	 *  that master is ready and will shift clock out;
	 *  b) Master is request slave to prepare a data transaction;
	 *  srdy is used by slave to:
	 *  a) Notify master that slave requests for a data transaction;
	 *  b) As as a response for SPI master initiated transaction,
	 *  tell master that slave is ready and clock can be shifted in;
	 */
	int mrdy_irq;
	u8 wait_for_mrdy;
	/*
	 * srdy is used by slave to:
	 * a) Notify master that slave requests for a data transaction;
	 * b) As as a response for SPI master initiated transaction, tell master
	 * that slave is ready and clock can be shifted in;
	 */
	void (*active_slave_srdy)(void *spi);
	void (*deactive_slave_srdy)(void *spi);

	struct tty_struct			*tty;

	struct spi_message			spi_big_msg;
	struct spi_transfer			spi_big_trans;

	u8								dtr;
	u8                                      write_buf_full;
	struct circ_buf				*write_buf;

	u8					throttle;
	int					open_count;
	/*
	 * When need read, null data packet will be created if no data to
	 * send this is for DTR change and MRDY request case
	 */
	u8					tx_null;
	/* lock for spi port operation */
	spinlock_t				port_lock;
	/* mutex for work operation */
	struct mutex				work_lock;
	struct work_struct			write_work;
	struct wake_lock			wakelock;
	wait_queue_head_t                       write_wait;
	struct workqueue_struct		*work_queue;
	struct notifier_block                   pm_notify;
	int					mrdy_irq_status;
	u8					complete_status;
};

static struct tty_driver *spi_tty_driver;
static struct dentry *mdm6600_spi_tty_debug_root;
/*
 * REVISIT: we currently only allow one instance of the device; however,
 * most code should be written as if multiple instance of the device
 * were allowed.
 */
static struct mdm6600_spi_tty_device *__mdm6600_spi_tty_device;
static int mdm6600_spi_tty_count = 0;

static unsigned long tx_size;
static unsigned long tx_time;
static unsigned long tx_count;
static unsigned long write_count;

#if SPI_IPC_DEBUG
void spi_ipc_buf_dump1(const char *header, const u8 *buf, int len, int in_ascii)
{
	int i;
	int c;

	u8 dbg_buf[256];

	if (len <= 0)
		return;

	for (i = 0, c = 0; (i < len) && (c < (256 - 3)); i++) {
		if (in_ascii && isprint(buf[i])) {
			sprintf(&dbg_buf[c], "%c ", buf[i]);
			c += 2;
		} else {
			sprintf(&dbg_buf[c], "%02x ", buf[i]);
			c += 3;
		}
	}
	dbg_buf[c] = 0;
	SPI_IPC_INFO("%s%s\n", header, dbg_buf);
}
#endif
static void spi_tty_buf_clear(struct circ_buf *cb)
{
	cb->head = 0;
	cb->tail = 0;
}

static struct circ_buf *spi_tty_buf_alloc(void)
{
	struct circ_buf *cb;

	cb = kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
	if (!cb)
		return NULL;
	cb->buf = kmalloc(SPI_TTY_BUF_SIZE, GFP_KERNEL);
	if (!cb->buf) {
		kfree(cb);
		return NULL;
	}

	spi_tty_buf_clear(cb);

	return cb;
}

static void spi_tty_buf_free(struct circ_buf *cb)
{
	kfree(cb->buf);
	kfree(cb);
}

static u32 spi_tty_msg_calc_crc(struct spi_tty_msg *msg)
{
	u32 fcs = 0;

	fcs += le32_to_cpu(msg->type);
	fcs += le32_to_cpu(msg->len);
	fcs += le32_to_cpu(msg->dtr);

	SPI_IPC_INFO("%s: type=%d, len=%d, dtr=%d, fcs=%d\n",
		__func__, msg->type, msg->len, msg->dtr, fcs);
	return fcs;
}

static int spi_tty_buf_room_avail(struct circ_buf *cb)
{
	return CIRC_SPACE(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
}

static int spi_tty_buf_data_avail(struct circ_buf *cb)
{
	return CIRC_CNT(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
}

static int spi_tty_buf_put(struct circ_buf *cb, const char *buf, int count)
{
	int c, ret = 0;

	while (1) {
		c = CIRC_SPACE_TO_END(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(cb->buf + cb->head, buf, c);
		cb->head = (cb->head + c) & (SPI_TTY_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}

static int spi_tty_buf_get(struct circ_buf *cb, char *buf, int count)
{
	int c, ret = 0;

	SPI_IPC_INFO("%s Enter\n", __func__);

	while (1) {
		c = CIRC_CNT_TO_END(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(buf, cb->buf + cb->tail, c);
		cb->tail = (cb->tail + c) & (SPI_TTY_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}

int spi_tty_write_cache(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	unsigned long flags;
	int ret;
	struct mdm6600_spi_tty_device *spi_tty = tty->driver_data;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (spi_tty->throttle == 1) {
		/* SPI is symmetrical bus, so stop write means stop receive */
		ret = 0;
	} else {
		if (!in_interrupt()
			&& spi_tty_buf_room_avail(spi_tty->write_buf) < count) {
			/* no enough room, wait */
			spin_unlock_irqrestore(&spi_tty->port_lock, flags);
			SPI_IPC_INFO("No write room, put write wait...\n");
			spi_tty->write_buf_full = 1;
			wait_event_interruptible_timeout(spi_tty->write_wait,
					spi_tty->write_buf_full == 0, 2*HZ);
			SPI_IPC_INFO("Write wait up from sleep\n");
			spin_lock_irqsave(&spi_tty->port_lock, flags);
		}

		ret = spi_tty_buf_put(spi_tty->write_buf, buffer, count);
	}

	/* wake lock to prevent suspend */
	wake_lock_timeout(&spi_tty->wakelock, SPI_TTY_WAKE_LOCK_TIMEOUT);

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	return ret;
}

static int spi_tty_open(struct tty_struct *tty, struct file *file)
{
	unsigned long flags;

	if (!__mdm6600_spi_tty_device) {
		pr_err("Cannot open spi_tty, spi slave device is not ready!\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&__mdm6600_spi_tty_device->port_lock, flags);

	/*
	 * REVISIT: we may want to setup driver_data in an .install function
	 */
	tty->driver_data = __mdm6600_spi_tty_device;
	tty->low_latency = 1;
	__mdm6600_spi_tty_device->tty = tty;
	++__mdm6600_spi_tty_device->open_count;

	spin_unlock_irqrestore(&__mdm6600_spi_tty_device->port_lock, flags);

	return 0;
}

static void spi_tty_handle_data(struct mdm6600_spi_tty_device  *mdm6600_tty)
{
	int cnt, crc, len;
	u32 msg_type, msg_len, msg_fcs;
	u8 *data;
	u8 request_num, i;

	struct spi_tty_msg *msg = mdm6600_tty->spi_big_trans.rx_buf;

	crc = spi_tty_msg_calc_crc(msg);

	msg_type = le32_to_cpu(msg->type);
	msg_len = le32_to_cpu(msg->len);
	msg_fcs = le32_to_cpu(msg->fcs);

	/* validate data */
	if (msg_type != 1 || msg_len > SPI_MTU || msg_fcs != crc) {
		pr_err("Invalid data receicved!\n");
		return;
	}

	if (mdm6600_tty->tty && mdm6600_tty->open_count > 0 && msg_len > 0) {
		len = msg_len;
		spi_ipc_buf_dump_ascii("rx data: ",
				(u8 *)msg+SPI_MSG_HEADER_LEN,
				(len > 16 ? 16 : len));
		SPI_IPC_INFO("insert data to tty\n");
		data = &msg->data[0];
		do {
			request_num = len/REQUEST_BLOCK_UNIT;
			if (request_num > 0) {
				for (i = 0; i < request_num; i++) {
					cnt = tty_buffer_request_room
						(mdm6600_tty->tty,
						 REQUEST_BLOCK_UNIT);
					if (cnt == 0)
						break;
				}
			}
			if ((len%REQUEST_BLOCK_UNIT) > 0) {
				cnt = tty_buffer_request_room(mdm6600_tty->tty,
							len%REQUEST_BLOCK_UNIT);
				if (cnt == 0)
					break;
			}
			cnt = tty_insert_flip_string(mdm6600_tty->tty,
					data, cnt);
			tty_flip_buffer_push(mdm6600_tty->tty);
			len -= cnt;
			data += cnt;
		} while (len > 0);
	}
}

static void spi_tty_write_worker(struct work_struct *work)
{
	int c;
	int crc;
	unsigned long flags;
	unsigned long start_t = 0;
	struct spi_tty_msg *msg;
	struct mdm6600_spi_tty_device *mdm6600_tty =
		container_of(work, struct mdm6600_spi_tty_device, write_work);

	start_t = jiffies;

	msg = (struct spi_tty_msg *)mdm6600_tty->spi_big_trans.tx_buf;

	mutex_lock(&mdm6600_tty->work_lock);
	spin_lock_irqsave(&mdm6600_tty->port_lock, flags);

	c = spi_tty_buf_data_avail(mdm6600_tty->write_buf);
	while (((c) || (mdm6600_tty->tx_null))
		&& (!mdm6600_tty->throttle)
		&& mdm6600_ctrl_is_bp_up()) {
		if (mdm6600_tty->tx_null)
			mdm6600_tty->tx_null = 0;

		/* initiate spi_big_trans */
		memset((u8 *)mdm6600_tty->spi_big_trans.tx_buf,
				0x0, SPI_TRANSACTION_LEN*2);
		mdm6600_tty->spi_big_msg.actual_length = 0;

		c = min(c, SPI_MTU);
		spi_tty_buf_get(mdm6600_tty->write_buf,
				(u8 *)mdm6600_tty->spi_big_trans.tx_buf
				 + SPI_MSG_HEADER_LEN,
				c);

		if (mdm6600_tty->tty && mdm6600_tty->open_count)
			tty_wakeup(mdm6600_tty->tty);
		mdm6600_tty->complete_status = SPI_TTY_START;
		spin_unlock_irqrestore(&mdm6600_tty->port_lock, flags);

		msg->type = cpu_to_le32(1);
		msg->len = cpu_to_le32(c);
		msg->dtr = cpu_to_le32(mdm6600_tty->dtr);
		crc = spi_tty_msg_calc_crc(msg);
		msg->fcs = cpu_to_le32(crc);
		mdm6600_tty->spi_big_trans.len = c + SPI_MSG_HEADER_LEN;
		spi_ipc_buf_dump("tx header: ",
				(u8 *)mdm6600_tty->spi_big_trans.tx_buf,
				SPI_MSG_HEADER_LEN);
		spi_ipc_buf_dump_ascii("tx data: ",
				(u8 *)mdm6600_tty->spi_big_trans.tx_buf
				 + SPI_MSG_HEADER_LEN,
				(c > 16 ? 16 : c));
		mdm6600_tty->spi_big_trans.len = SPI_TRANSACTION_LEN;
		mdm6600_tty->spi_big_trans.bits_per_word = 32;

		mdm6600_tty->spi_big_trans.transfer_context = mdm6600_tty;

		spi_sync(mdm6600_tty->spi, &mdm6600_tty->spi_big_msg);

		SPI_IPC_INFO("%s actual transfer length=%d\n",\
			__func__, mdm6600_tty->spi_big_msg.actual_length);
		if (mdm6600_tty->spi_big_msg.actual_length ==
				SPI_TRANSACTION_LEN) {
			tx_count++;
			tx_size += mdm6600_tty->spi_big_trans.len
				- SPI_MSG_HEADER_LEN;
			spi_tty_handle_data(mdm6600_tty);
		} else {
			dev_err(&mdm6600_tty->spi->dev,
					"spi_sync() failed to transfer data!\n"
			       );
		}

		/* wake up writes wait on queue */
		wake_up_interruptible(&mdm6600_tty->write_wait);

		spin_lock_irqsave(&mdm6600_tty->port_lock, flags);
		mdm6600_tty->complete_status = SPI_TTY_FINISH;
		c = spi_tty_buf_data_avail(mdm6600_tty->write_buf);
	}

	spin_unlock_irqrestore(&mdm6600_tty->port_lock, flags);
	mutex_unlock(&mdm6600_tty->work_lock);
	tx_time += jiffies_to_msecs(jiffies - start_t);
}
static irqreturn_t mdm6600_spi_mrdy_irq_handler(int irq, void *ptr)
{
	unsigned state;
	unsigned long flags;
	struct mdm6600_spi_tty_device *mdm6600 =
		(struct mdm6600_spi_tty_device *)ptr;
	/* check if this is just a spur */
	state = gpio_get_value(mdm6600->pdata->gpio_mrdy);
	if (state == 0) {
		spin_lock_irqsave(&mdm6600->port_lock, flags);
		if (mdm6600->mrdy_irq_status == MRDY_IRQ_ENABLE) {
			mdm6600->mrdy_irq_status = MRDY_IRQ_DISABLE;
			disable_irq_nosync(irq);
		}
		/* mrdy is active */
		if (mdm6600->wait_for_mrdy == 1) {
			/* mrdy is just a confirm of srdy, do nothing*/
			mdm6600->wait_for_mrdy = 0;
			spin_unlock_irqrestore(&mdm6600->port_lock, flags);
		} else {
		mdm6600->tx_null = 1;
		/* wake lock to prevent suspend */
		wake_lock_timeout(&mdm6600->wakelock,
			SPI_TTY_WAKE_LOCK_TIMEOUT);
		spin_unlock_irqrestore(&mdm6600->port_lock, flags);
		queue_work(mdm6600->work_queue, &mdm6600->write_work);
		}
	} else {
	  /*printk(KERN_ERR"This is a spur!\n");*/
	}
	return IRQ_HANDLED;
}

static void mdm6600_active_slave_srdy(void *ptr)
{
	int state;
	struct mdm6600_spi_tty_device	*spi = ptr;

	gpio_direction_output(spi->pdata->gpio_srdy, 0);
	state = gpio_get_value(spi->pdata->gpio_mrdy);
	SPI_IPC_INFO("%s: after setting output, MRDY state is %d\n",\
		 __func__, state);
	spi->wait_for_mrdy = !!state;

}

static void mdm6600_deactive_slave_srdy(void *ptr)
{
	struct mdm6600_spi_tty_device	*spi = ptr;
	unsigned long flags;

	gpio_direction_output(spi->pdata->gpio_srdy, 1);
	spi->wait_for_mrdy = 0;
	spin_lock_irqsave(&spi->port_lock, flags);
	if (spi->mrdy_irq_status ==  MRDY_IRQ_DISABLE) {
		spi->mrdy_irq_status = MRDY_IRQ_ENABLE;
		enable_irq(spi->mrdy_irq);
	}
	spin_unlock_irqrestore(&spi->port_lock, flags);
}


static int mdm6600_spi_config_gpio(struct mdm6600_spi_tty_device *mdm6600)
{
	int mrdy_irq, err = 0;

	if (!mdm6600->pdata) {
		dev_err(&mdm6600->spi->dev, "gpio information not provided");
		return -EINVAL;
	}

	err = gpio_request(mdm6600->pdata->gpio_srdy, "SPI IPC SRDY");
	if (err < 0) {
		dev_err(&mdm6600->spi->dev,
				"%s Failed to Acquire SRDY GPIO pin, "
				"errno = %d\n", __func__, err);
		return err;
	}

	/* set SRDY to high  by default */
	err = gpio_direction_output(mdm6600->pdata->gpio_srdy, 1);
	if (err < 0) {
		dev_err(&mdm6600->spi->dev,
				"%s Failed to initialize SRDY GPIO pin, "
				"errno = %d\n", __func__, err);
		goto free_srdy;
	}

	/* AP/BP SPI IPC - MRDY uses gpio */
	err = gpio_request(mdm6600->pdata->gpio_mrdy, "SPI IPC MRDY");
	if (err < 0) {
		dev_err(&mdm6600->spi->dev,
				"%s Failed to Acquire MRDY GPIO pin., "
				"errno = %d\n", __func__, err);
		goto free_srdy;
	}

	/* set SRDY to high  by default */
	err = gpio_direction_input(mdm6600->pdata->gpio_mrdy);
	if (err < 0) {
		dev_err(&mdm6600->spi->dev,
				"%s Failed to initialize MRDY GPIO pin., "
				"errno = %d\n", __func__, err);
		goto free_mrdy;
	}

	/* setup irq for MRDY */
	mrdy_irq = gpio_to_irq(mdm6600->pdata->gpio_mrdy);
	if (mrdy_irq < 0) {
		err = mrdy_irq;
		dev_err(&mdm6600->spi->dev,
				"%s Failed to determine MRDY IRQ#., "
				"errno = %d\n", __func__, err);
		goto free_mrdy;
	}

	/*how to specify only falling edge for interrupt?*/
	set_irq_type(mrdy_irq, IRQ_TYPE_LEVEL_LOW);
	err = request_irq(mrdy_irq, mdm6600_spi_mrdy_irq_handler,
		IRQ_TYPE_LEVEL_LOW, "MRDY GPIO IRQ", mdm6600);

	if (err < 0) {
		dev_err(&mdm6600->spi->dev,
				"%s Failed to register MRDY interrupt handler, "
				"errno = %d\n", __func__, err);
		goto free_mrdy;
	}
	mdm6600->mrdy_irq = mrdy_irq;
	return 0;

free_mrdy:
	gpio_free(mdm6600->pdata->gpio_mrdy);
free_srdy:
	gpio_free(mdm6600->pdata->gpio_srdy);

	return err;
}

static int mdm6600_spi_suspend(struct spi_device *dev, pm_message_t mesg)
{
	int ret;
	unsigned long flags;

	ret = 0;
	spin_lock_irqsave(&__mdm6600_spi_tty_device->port_lock, flags);
	if (__mdm6600_spi_tty_device->mrdy_irq_status == MRDY_IRQ_ENABLE) {
		__mdm6600_spi_tty_device->mrdy_irq_status = MRDY_IRQ_DISABLE;
		disable_irq(__mdm6600_spi_tty_device->mrdy_irq);
	}

	if (__mdm6600_spi_tty_device->complete_status == SPI_TTY_START)
		ret = -EAGAIN;

	spin_unlock_irqrestore(&__mdm6600_spi_tty_device->port_lock, flags);

	return ret;
}

int tty_pm_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
	struct mdm6600_spi_tty_device  *mdm6600_tty = container_of(
		notify_block, struct mdm6600_spi_tty_device, pm_notify);

	if (mode == PM_POST_SUSPEND)
		if (mdm6600_tty->mrdy_irq_status == MRDY_IRQ_DISABLE) {
			mdm6600_tty->mrdy_irq_status = MRDY_IRQ_ENABLE;
			enable_irq(mdm6600_tty->mrdy_irq);
		}

	return NOTIFY_OK;
}

static int __devinit mdm6600_spi_probe(struct spi_device *spi)
{
	int err;
	struct mdm6600_spi_tty_device *mdm6600_tty;
	struct device *tty_dev;

	SPI_IPC_INFO("%s, spi dev=%p\n", __func__, spi);

	if (mdm6600_spi_tty_count > 0)
		return -ENODEV;

	mdm6600_tty = kzalloc(sizeof(struct mdm6600_spi_tty_device),
			GFP_KERNEL);
	if (!mdm6600_tty) {
		dev_err(&spi->dev, "cannot allocate memory for mdm6600");
		return -ENOMEM;
	}

	mdm6600_tty->spi = spi;
	mdm6600_tty->pdata = spi->dev.platform_data;
	spi_set_drvdata(spi, mdm6600_tty);

	mdm6600_tty->mrdy_irq_status =  MRDY_IRQ_ENABLE;
	mdm6600_tty->complete_status = SPI_TTY_FINISH;

	tx_size = 0L;
	tx_time = 0L;
	tx_count = 0L;
	write_count = 0L;

	mdm6600_tty->write_buf = spi_tty_buf_alloc();
	if (!mdm6600_tty->write_buf) {
		dev_err(&spi->dev, "failed to malloc spi_tty write buf!\n");
		err = -ENOMEM;
		goto err_free;
	}


	mdm6600_tty->throttle = 0;
	mdm6600_tty->open_count = 0;
	mdm6600_tty->tx_null = 0;
	spin_lock_init(&mdm6600_tty->port_lock);
	mutex_init(&mdm6600_tty->work_lock);
	INIT_WORK(&mdm6600_tty->write_work, spi_tty_write_worker);
	wake_lock_init(&mdm6600_tty->wakelock, WAKE_LOCK_SUSPEND,
			"spi_tty_wakelock");

	init_waitqueue_head(&mdm6600_tty->write_wait);
	mdm6600_tty->write_buf_full = 0;

	mdm6600_tty->work_queue = create_singlethread_workqueue("spi_tty_wq");
	if (mdm6600_tty->work_queue == NULL) {
		dev_err(&spi->dev, "Failed to create work queue\n");
		err = -ESRCH;
		goto err_free_cb;
	}

	spi_message_init(&mdm6600_tty->spi_big_msg);

	mdm6600_tty->spi_big_trans.tx_buf =
		kmalloc(SPI_TRANSACTION_LEN*2, GFP_KERNEL);
	if (!mdm6600_tty->spi_big_trans.tx_buf) {
		pr_err("%s: Cannot malloc mem!\n", __func__);
		err = -ENOMEM;
		goto err_free_wq;
	}

	mdm6600_tty->spi_big_trans.rx_buf =
		(u8 *)mdm6600_tty->spi_big_trans.tx_buf
		+ SPI_TRANSACTION_LEN;

	mdm6600_tty->spi_big_trans.transfer_start_callback =
						mdm6600_active_slave_srdy;
	mdm6600_tty->spi_big_trans.transfer_complete_callback =
						mdm6600_deactive_slave_srdy;

	spi_message_add_tail(&mdm6600_tty->spi_big_trans,
			&mdm6600_tty->spi_big_msg);

	mdm6600_tty->pm_notify.notifier_call = tty_pm_notify;
	register_pm_notifier(&mdm6600_tty->pm_notify);

	/*
	 * spi_tty_open should be the only place using
	 * __mdm6600_spi_tty_device, so set it up here.
	 */
	__mdm6600_spi_tty_device = mdm6600_tty;

	tty_dev = tty_register_device(spi_tty_driver, mdm6600_spi_tty_count++,
			&mdm6600_tty->spi->dev);
	if (IS_ERR(tty_dev)) {
		err = PTR_ERR(tty_dev);
		goto err_free_tx_buf;
	}

	err = mdm6600_spi_config_gpio(mdm6600_tty);
	if (err < 0)
		goto err_free_tx_buf;

	return 0;

err_free_tx_buf:
	kfree(mdm6600_tty->spi_big_trans.tx_buf);
err_free_wq:
	destroy_workqueue(mdm6600_tty->work_queue);
err_free_cb:
	spi_tty_buf_free(mdm6600_tty->write_buf);
err_free:
	kfree(mdm6600_tty);

	__mdm6600_spi_tty_device = NULL;

	dev_err(&spi->dev, "probe failed with errno=%d\n", err);
	return err;
}

static int mdm6600_spi_resume(struct spi_device *dev)
{
	SPI_IPC_INFO("%s\n", __func__);
	return 0;
}

static int __devexit mdm6600_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver mdm6600_spi_driver = {
	.driver = {
		.name = "mdm6600_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = mdm6600_spi_probe,
	.suspend = mdm6600_spi_suspend,
	.resume = mdm6600_spi_resume,
	.remove = __devexit_p(mdm6600_spi_remove),
};


static int spi_tty_tiocmset(struct tty_struct *tty, struct file *file,
		unsigned int set, unsigned int clear)
{
	unsigned long flags;
	struct mdm6600_spi_tty_device *spi_tty = tty->driver_data;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (set & TIOCM_DTR) {
		SPI_IPC_INFO("set DTR\n");
		spi_tty->dtr = 1;
	}

	if (clear & TIOCM_DTR) {
		SPI_IPC_INFO("clear DTR\n");
		spi_tty->dtr = 0;
	}

	spi_tty->tx_null = 1;

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);
	queue_work(spi_tty->work_queue, &spi_tty->write_work);

	return 0;
}

static int spi_tty_write_room(struct tty_struct *tty)
{
	unsigned long flags;
	struct mdm6600_spi_tty_device *spi_tty = tty->driver_data;
	int room = 0;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (spi_tty->open_count) {
		if (spi_tty->throttle == 1)
			room = 0;
		else
			room = spi_tty_buf_room_avail(spi_tty->write_buf);
	}

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	SPI_IPC_INFO("room=%d\n", room);

	return room;
}


static int spi_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	unsigned long flags;
	struct mdm6600_spi_tty_device *spi_tty = tty->driver_data;
	int retval = 0;

	SPI_IPC_INFO("%s, count=%d\n", __func__, count);

	if (!spi_tty || !mdm6600_ctrl_is_bp_up())
		return -ENODEV;

	/*spi_slave_dump_hex(buffer, count); */
	if (count > SPI_MTU) {
		pr_err("%s: data length over SPI MTU!\n", __func__);
		return 0;
	}

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (!spi_tty->open_count) {
		pr_err("%s: device not opened!\n", __func__);
		spin_unlock_irqrestore(&spi_tty->port_lock, flags);
		return -EIO;
	}

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	retval = spi_tty_write_cache(tty, buffer, count);
	write_count++;
	queue_work(spi_tty->work_queue, &spi_tty->write_work);
	SPI_IPC_INFO("%s Exit, retval=%d\n", __func__, retval);
	return retval;
}

static int spi_tty_tiocmget(struct tty_struct *tty, struct file *file)
{
	return 0;
}

static int spi_tty_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static void do_close(struct mdm6600_spi_tty_device *spi_tty)
{
	unsigned long flags;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (!spi_tty->open_count)
		goto exit;
	--spi_tty->open_count;
	spin_unlock_irqrestore(&spi_tty->port_lock, flags);
	/* flush pending workqueue after decrease open_count
	 * and before clear tty, new work queued after this will not access
	 *  tty any more.
	 */
	flush_workqueue(spi_tty->work_queue);
	spin_lock_irqsave(&spi_tty->port_lock, flags);
	if (spi_tty->open_count <= 0)
		spi_tty->tty = NULL;
exit:
	spin_unlock_irqrestore(&spi_tty->port_lock, flags);
}
static void spi_tty_close(struct tty_struct *tty, struct file *file)
{
	struct mdm6600_spi_tty_device *spi_tty = tty->driver_data;

	if (spi_tty)
		do_close(spi_tty);
}

static const struct tty_operations serial_ops = {
	.open = spi_tty_open,
	.close = spi_tty_close,
	.write = spi_tty_write,
	.write_room = spi_tty_write_room,
	.set_termios = NULL,
	.ioctl = spi_tty_ioctl,
	.tiocmget = spi_tty_tiocmget,
	.tiocmset = spi_tty_tiocmset,
	.throttle = NULL,
	.unthrottle = NULL,
};
static int debugfs_tx_info_init(void)
{
	struct dentry *d;

	d = debugfs_create_dir("mdm6600_spi_tty", NULL);
	mdm6600_spi_tty_debug_root = d;
	if (!d)
		return -ENOMEM;
	d = debugfs_create_u32("tx_size_entries", S_IRUGO|S_IWUSR,
				mdm6600_spi_tty_debug_root,
				(u32 *)&tx_size);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32("tx_time_entries", S_IRUGO|S_IWUSR,
			mdm6600_spi_tty_debug_root,
			(u32 *)&tx_time);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32("tx_count_entries", S_IRUGO|S_IWUSR,
			mdm6600_spi_tty_debug_root,
			(u32 *)&tx_count);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32("write_count_entries", S_IRUGO|S_IWUSR,
			mdm6600_spi_tty_debug_root,
			(u32 *)&write_count);
	if (!d)
		return -ENOMEM;

	return 0;
}
static int __init mdm6600_spi_tty_init(void)
{
	int ret;

	spi_tty_driver = alloc_tty_driver(SPI_TTY_MINORS);
	if (!spi_tty_driver)
		return -ENOMEM;

	spi_tty_driver->owner = THIS_MODULE;
	spi_tty_driver->driver_name = "spi_modem";
	spi_tty_driver->name = "ttySPI";
	spi_tty_driver->major = 0;      /* dynamic */
	spi_tty_driver->minor_start = 0;
	spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	spi_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	spi_tty_driver->init_termios = tty_std_termios;
	spi_tty_driver->init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(spi_tty_driver, &serial_ops);

	ret = tty_register_driver(spi_tty_driver);
	if (ret) {
		pr_err("failed to register spi_tty tty driver");
		goto err_put;
	}

	ret = spi_register_driver(&mdm6600_spi_driver);
	if (ret)
		goto err_put;

	ret = debugfs_tx_info_init();

	if (ret)
		goto err_put;
	return 0;

err_put:
	put_tty_driver(spi_tty_driver);

	SPI_IPC_INFO("%s failed with errno=%d\n", __func__, ret);
	return ret;
}

static void mdm6600_spi_tty_shutdown(void)
{
	debugfs_remove_recursive(mdm6600_spi_tty_debug_root);
	spi_unregister_driver(&mdm6600_spi_driver);
	tty_unregister_device(spi_tty_driver, 0);
	tty_unregister_driver(spi_tty_driver);
	put_tty_driver(spi_tty_driver);
	if (__mdm6600_spi_tty_device) {
		/* close the port */
		while (__mdm6600_spi_tty_device->open_count)
			do_close(__mdm6600_spi_tty_device);

		kfree(__mdm6600_spi_tty_device->spi_big_trans.tx_buf);
		destroy_workqueue(__mdm6600_spi_tty_device->work_queue);
		spi_tty_buf_free(__mdm6600_spi_tty_device->write_buf);
		wake_lock_destroy(&__mdm6600_spi_tty_device->wakelock);

		kfree(__mdm6600_spi_tty_device);
		__mdm6600_spi_tty_device = NULL;
	}
}

subsys_initcall(mdm6600_spi_tty_init);
module_exit(mdm6600_spi_tty_shutdown);
