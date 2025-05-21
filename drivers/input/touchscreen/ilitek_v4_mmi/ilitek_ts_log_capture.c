/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: mfeng <ming_feng@ilitek.com>
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
 */
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "ilitek_v3.h"

#define LOG_FIFO_SIZE 8172
#define ILITEK_LOG_DEVICE_NAME "tp_tools"

struct ili_ts_log {
	struct kfifo        fifo;
	wait_queue_head_t   wq;
	struct miscdevice   miscdev;
};

static struct ili_ts_log ts_log_dev;

/* Save a byte to a FIFO and discard the oldest byte if FIFO is full */
void ili_put_fifo_with_discard(char *log_buf, int len)
{
	if (!kfifo_initialized(&ts_log_dev.fifo))
		return;

	if (kfifo_is_full(&ts_log_dev.fifo)) {
		kfifo_skip(&ts_log_dev.fifo);
		ILI_INFO("Save a byte to a FIFO and discard the oldest byte if FIFO is full");
	}

	kfifo_in(&ts_log_dev.fifo, log_buf, len);

	wake_up_interruptible(&ts_log_dev.wq);
}

void ili_clear_kfifo(void)
{
	if (kfifo_len(&ts_log_dev.fifo) != 0) {
		kfifo_reset(&ts_log_dev.fifo);
	}
}

static __poll_t log_file_poll(struct file *file,
                    struct poll_table_struct *pt)
{
	struct ili_ts_log *log = file->private_data;

	poll_wait(file, &log->wq, pt);
	return !kfifo_is_empty(&log->fifo) ? (POLLPRI|POLLIN) : 0;
}

static ssize_t log_file_read(struct file *file, char __user *buffer,
                size_t count, loff_t *ppos)
{
	struct ili_ts_log *log = file->private_data;
	unsigned int copied;
	int ret = 0;

	if (kfifo_is_empty(&log->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(log->wq,
			!kfifo_is_empty(&log->fifo));
		if (ret == -ERESTARTSYS)
			return -EINTR;
	}
	ret = kfifo_to_user(&log->fifo, buffer, count, &copied);
	if (ret)
		return ret;

	return copied;
}

static int ili_log_device_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ts_log_dev;
	ILI_INFO("success open log device");

	return 0;
}

static const struct file_operations log_device_fops = {
	.owner  = THIS_MODULE,
	.open    = ili_log_device_open,
	.read   = log_file_read,
	.poll   = log_file_poll,
	.llseek = noop_llseek,
};

int ili_log_capture_register_misc(void)
{
	int rc = 0;

	init_waitqueue_head(&ts_log_dev.wq);
	/* Create FIFO datastructure */
	rc = kfifo_alloc(&ts_log_dev.fifo,
		LOG_FIFO_SIZE, GFP_KERNEL);
	if (rc)
		return rc;


	ts_log_dev.miscdev.minor = MISC_DYNAMIC_MINOR;
	ts_log_dev.miscdev.name = ILITEK_LOG_DEVICE_NAME;
	ts_log_dev.miscdev.fops = &log_device_fops;
	rc = misc_register(&ts_log_dev.miscdev);
	if (rc)
		return rc;
	return 0;
}

int ili_log_capture_unregister_misc(void)
{
	if (!kfifo_initialized(&ts_log_dev.fifo))
		return -EINTR ;
	kfifo_free(&ts_log_dev.fifo);
	misc_deregister(&ts_log_dev.miscdev);
	return 0;
}

