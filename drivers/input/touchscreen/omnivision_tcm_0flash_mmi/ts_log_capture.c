/*
 * Copyright (C) 2026 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "omnivision_tcm_core.h"

#define LOG_FIFO_SIZE 8192
#define TP_LOG_DEVICE_NAME "tp_tools"

struct ts_log {
	struct kfifo        fifo;
	wait_queue_head_t   wq;
	struct miscdevice   miscdev;
};

static struct ts_log ts_log_dev;

void ts_put_fifo_with_discard(char *log_buf, int len)
{
	unsigned int reval;
	int space;
	if (!kfifo_initialized(&ts_log_dev.fifo))
		return;

	space = kfifo_avail(&ts_log_dev.fifo);
	if (space < len) {
		reval = kfifo_out(&ts_log_dev.fifo, NULL, len - space);
		if(reval != (len - space))
			OVT_ERROR("kfifo_out failed to discard requested bytes\n");
	}

	kfifo_in(&ts_log_dev.fifo, log_buf, len);

	wake_up_interruptible(&ts_log_dev.wq);
}

void ts_clear_kfifo(void)
{
	if (kfifo_len(&ts_log_dev.fifo) != 0) {
		kfifo_reset(&ts_log_dev.fifo);
	}
}

static __poll_t log_file_poll(struct file *file,
                    struct poll_table_struct *pt)
{
	if (file) {
		struct ts_log *log = file->private_data;
		poll_wait(file, &log->wq, pt);
		return !kfifo_is_empty(&log->fifo) ? (POLLPRI|POLLIN) : 0;
	}
	else {
		OVT_ERROR("filp_open failed: file is NULL\n");
    	return -EINVAL;
	}
}

static ssize_t log_file_read(struct file *file, char __user *buffer,
                size_t count, loff_t *ppos)
{
	if (file) {
		struct ts_log *log = file->private_data;
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
	else {
		OVT_ERROR("filp_open failed: file is NULL\n");
    	return -EINVAL;
	}
}

static int ts_log_device_open(struct inode *inode, struct file *filp)
{
	if (filp) {
		filp->private_data = &ts_log_dev;
		OVT_INFO("success open log device");

		return 0;
	}
	else {
		OVT_ERROR("filp_open failed: filp is NULL\n");
    	return -EINVAL;
	}
}

static const struct file_operations log_device_fops = {
	.owner  = THIS_MODULE,
	.open    = ts_log_device_open,
	.read   = log_file_read,
	.poll   = log_file_poll,
	.llseek = noop_llseek,
};

int ts_log_capture_register_misc(void)
{
	int rc = 0;
	init_waitqueue_head(&ts_log_dev.wq);
	/* Create FIFO datastructure */
	rc = kfifo_alloc(&ts_log_dev.fifo,
		LOG_FIFO_SIZE, GFP_KERNEL);
	if (rc)
		return rc;
	ts_log_dev.miscdev.minor = MISC_DYNAMIC_MINOR;
	ts_log_dev.miscdev.name = TP_LOG_DEVICE_NAME;
	ts_log_dev.miscdev.fops = &log_device_fops;
	rc = misc_register(&ts_log_dev.miscdev);
	if (rc)
		return rc;
	return 0;
}

int ts_log_capture_unregister_misc(void)
{
	if (!kfifo_initialized(&ts_log_dev.fifo))
		return -EINVAL;
	kfifo_free(&ts_log_dev.fifo);
	misc_deregister(&ts_log_dev.miscdev);
	return 0;
}

