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


struct proc_dir_entry *d_moto_mm;

int moto_mm_proc_init(void)
{
	d_moto_mm = proc_mkdir(MOTO_MM_PROC_DIR, NULL);
	if (!d_moto_mm) {
		mm_err("failed to create proc dir moto_mm\n");
		goto err_creat_d_moto_mm;
	}

	return 0;

err_creat_d_moto_mm:
	return -ENOENT;
}

void moto_mm_proc_deinit(void)
{
	remove_proc_entry(MOTO_MM_PROC_DIR, NULL);
}

