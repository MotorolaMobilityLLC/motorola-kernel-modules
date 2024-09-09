/*
 * Copyright (C) 2023 Motorola Mobility LLC
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

#define pr_fmt(fmt) "moto_binder: " fmt

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "moto_binder.h"

#define MOTO_BINDER_PROC_DIR		"moto_binder"

#define MAX_SET (128)

static int moto_binder_status_enabled = 0;
static struct proc_dir_entry *d_moto_binder;

int moto_binder_check_uid = 0;

static ssize_t proc_binder_status_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 16, &val);
	if (err)
		return err;

	moto_binder_status_enabled = val;

	if (moto_binder_status_enabled > 0)
		moto_binder_init();
	else
		moto_binder_exit();

	return count;
}

static ssize_t proc_binder_status_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_binder_status_enabled);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_binder_status_enabled_fops = {
	.proc_write		= proc_binder_status_enabled_write,
	.proc_read		= proc_binder_status_enabled_read,
};

static void *proc_binder_status_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *proc_binder_status_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void proc_binder_status_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations proc_binder_status_op = {
	.start	= proc_binder_status_start,
	.next	= proc_binder_status_next,
	.stop	= proc_binder_status_stop,
	.show	= proc_binder_status_show
};

static int proc_binder_status_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &proc_binder_status_op);
}

static const struct proc_ops proc_binder_status_fops = {
	.proc_open	= proc_binder_status_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= seq_release,
};

static ssize_t proc_binder_check_uid_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	moto_binder_check_uid = val;

	return count;
}

static ssize_t proc_binder_check_uid_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_binder_check_uid);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_binder_check_uid_fops = {
	.proc_write		= proc_binder_check_uid_write,
	.proc_read		= proc_binder_check_uid_read,
};

static void *proc_binder_check_status_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *proc_binder_check_status_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void proc_binder_check_status_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations proc_binder_check_status_op = {
	.start	= proc_binder_check_status_start,
	.next	= proc_binder_check_status_next,
	.stop	= proc_binder_check_status_stop,
	.show	= proc_binder_check_status_show
};

static int proc_binder_check_status_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &proc_binder_check_status_op);
}

static const struct proc_ops proc_binder_check_status_fops = {
	.proc_open	= proc_binder_check_status_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= seq_release,
};

static int __init moto_binder_proc_init(void)
{
	struct proc_dir_entry *proc_node;

	d_moto_binder = proc_mkdir(MOTO_BINDER_PROC_DIR, NULL);
	if (!d_moto_binder) {
		pr_err("failed to create proc dir moto_binder\n");
		goto err_creat_d_moto_binder;
	}

	proc_node = proc_create("binder_status_enabled", 0666, d_moto_binder, &proc_binder_status_enabled_fops);
	if (!proc_node) {
		pr_err("failed to create proc node binder_status_enabled\n");
		goto err_creat_binder_status_enabled;
	}

	proc_node = proc_create("binder_status", 0444, d_moto_binder, &proc_binder_status_fops);
	if (!proc_node) {
		pr_err("failed to create proc node binder_status\n");
		goto err_create_binder_status;
	}

	proc_node = proc_create("binder_check_uid", 0666, d_moto_binder, &proc_binder_check_uid_fops);
	if (!proc_node) {
		pr_err("failed to create proc node binder_check_uid\n");
		goto err_creat_binder_check_uid;
	}

	proc_node = proc_create("binder_check_status", 0444, d_moto_binder, &proc_binder_check_status_fops);
	if (!proc_node) {
		pr_err("failed to create proc node binder_check_status\n");
		goto err_create_binder_check_status;
	}

	return 0;


err_create_binder_check_status:

	remove_proc_entry("binder_check_uid", NULL);
err_creat_binder_check_uid:

	remove_proc_entry("binder_status", NULL);
err_create_binder_status:

	remove_proc_entry("binder_status_enabled", NULL);
err_creat_binder_status_enabled:

	remove_proc_entry(MOTO_BINDER_PROC_DIR, NULL);
err_creat_d_moto_binder:
	return -ENOENT;
}

static void __exit moto_binder_proc_deinit(void)
{
	remove_proc_entry("binder_status", NULL);
	remove_proc_entry("binder_status_enabled", d_moto_binder);
	remove_proc_entry(MOTO_BINDER_PROC_DIR, NULL);
}

module_init(moto_binder_proc_init);
module_exit(moto_binder_proc_deinit);
MODULE_DESCRIPTION("Motorola binder optimization driver");
MODULE_LICENSE("GPL v2");
