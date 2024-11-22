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

#define pr_fmt(fmt) "moto_mm: " fmt

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "mm_common.h"

#define MOTO_MM_PROC_DIR		"moto_mm"

#define MAX_SET (128)

#if defined(MM_INFO_SUPPORTED)
int moto_mm_info_enabled = 0; // disabled by default because of overhead of hook and log.
int moto_alloc_warn_ms = 100; // 100ms by default
#endif // defined(MM_INFO_SUPPORTED)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
struct proc_dir_entry *d_moto_mm;
#endif

#if defined(MM_INFO_SUPPORTED)
static ssize_t proc_mm_info_enabled_write(struct file *file, const char __user *buf,
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

	moto_mm_info_enabled = val;

	if (moto_mm_info_enabled > 0)
		mm_info_init();
	else
		mm_info_exit();

	return count;
}

static ssize_t proc_mm_info_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_mm_info_enabled);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_alloc_warn_ms_write(struct file *file, const char __user *buf,
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

	moto_alloc_warn_ms = val;
	return count;
}

static ssize_t proc_alloc_warn_ms_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_alloc_warn_ms);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}
#endif // defined(MM_INFO_SUPPORTED)

#if defined(MM_INFO_SUPPORTED)
static const struct proc_ops proc_mm_info_enabled_fops = {
	.proc_write		= proc_mm_info_enabled_write,
	.proc_read		= proc_mm_info_enabled_read,
};

static const struct proc_ops proc_alloc_warn_ms_fops = {
	.proc_write		= proc_alloc_warn_ms_write,
	.proc_read		= proc_alloc_warn_ms_read,
};
#endif // defined(MM_INFO_SUPPORTED)

int moto_mm_proc_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	struct proc_dir_entry *proc_node;

	d_moto_mm = proc_mkdir(MOTO_MM_PROC_DIR, NULL);
	if (!d_moto_mm) {
		pr_err("failed to create proc dir moto_mm\n");
		goto err_creat_d_moto_mm;
	}
#endif

#if defined(MM_INFO_SUPPORTED)
	proc_node = proc_create("mm_info_enabled", 0666, d_moto_mm, &proc_mm_info_enabled_fops);
	if (!proc_node) {
		pr_err("failed to create proc node mm_info_enabled\n");
		goto err_creat_mm_info_enabled;
	}

	proc_node = proc_create("alloc_warn_ms", 0666, d_moto_mm, &proc_alloc_warn_ms_fops);
	if (!proc_node) {
		pr_err("failed to create proc node alloc_warn_ms\n");
		goto err_creat_alloc_warn_ms;
	}
#endif // defined(MM_INFO_SUPPORTED)

	return 0;

#if defined(MM_INFO_SUPPORTED)
	remove_proc_entry("alloc_warn_ms", NULL);
err_creat_alloc_warn_ms:

	remove_proc_entry("mm_info_enabled", NULL);
err_creat_mm_info_enabled:
#endif // defined(MM_INFO_SUPPORTED)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	remove_proc_entry(MOTO_MM_PROC_DIR, NULL);
err_creat_d_moto_mm:
	return -ENOENT;
#endif
}

void moto_mm_proc_deinit(void)
{
#if defined(MM_INFO_SUPPORTED)
	remove_proc_entry("alloc_warn_ms", d_moto_mm);
	remove_proc_entry("mm_info_enabled", d_moto_mm);
#endif // defined(MM_INFO_SUPPORTED)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	remove_proc_entry(MOTO_MM_PROC_DIR, NULL);
#endif
}

