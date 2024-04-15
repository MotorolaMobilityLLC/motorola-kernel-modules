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

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "mm_common.h"

#define MOTO_MM_PROC_DIR		"moto_mm"

#define MAX_SET (128)

int moto_alloc_warn_ms = -1;

static ssize_t proc_alloc_warn_ms_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[13];
	int err,val;
	static DEFINE_MUTEX(alloc_warn_ms_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
	    count = sizeof(buffer) - 1;

	if (copy_from_user (buffer, buf, count))
	    return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
	    return err;

	mutex_lock(&alloc_warn_ms_mutex);
	moto_alloc_warn_ms = val;
	if (moto_alloc_warn_ms > 0)
	    register_mem_alloc_hooks();
	else
	    unregister_mem_alloc_hooks();
	mutex_unlock(&alloc_warn_ms_mutex);
	return count;
}

static ssize_t proc_alloc_warn_ms_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_alloc_warn_ms);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_alloc_warn_ms_fops = {
	.proc_write         = proc_alloc_warn_ms_write,
	.proc_read          = proc_alloc_warn_ms_read,
};

struct proc_dir_entry *d_moto_mm;

int moto_mm_proc_init(void)
{
	struct proc_dir_entry *proc_node;

	d_moto_mm = proc_mkdir(MOTO_MM_PROC_DIR, NULL);
	if (!d_moto_mm) {
		mm_err("failed to create proc dir moto_mm\n");
		goto err_creat_d_moto_mm;
	}

	proc_node = proc_create("alloc_warn_ms", 0666, d_moto_mm, &proc_alloc_warn_ms_fops);
	if (!proc_node){
		mm_err("failed to create proc node alloc_warn_ms\n");
		goto err_create_alloc_warn_ms;
	}

	return 0;

err_create_alloc_warn_ms:
	remove_proc_entry (MOTO_MM_PROC_DIR, NULL);

err_creat_d_moto_mm:
	return -ENOENT;
}

void moto_mm_proc_deinit(void)
{
	remove_proc_entry("alloc_warn_ms", d_moto_mm);
	remove_proc_entry(MOTO_MM_PROC_DIR, NULL);
}

