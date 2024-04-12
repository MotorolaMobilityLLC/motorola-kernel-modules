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

#include <linux/module.h>
#include <linux/types.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>

#include "mm_common.h"

static void tune_inactive_ratio_hook(void *data, unsigned long *inactive_ratio, int file)
{
	if (file)
		*inactive_ratio = min(2UL, *inactive_ratio);
	else
		*inactive_ratio = 1;

	return;
}

static int register_all_hooks(void)
{
	int rc;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	/* tune_inactive_ratio_hook */
	REGISTER_HOOK(tune_inactive_ratio);
#endif
	return 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	UNREGISTER_HOOK(tune_inactive_ratio);
ERROR_OUT(tune_inactive_ratio):
#endif
	return rc;
}

static void unregister_all_hooks(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	UNREGISTER_HOOK(tune_inactive_ratio);
#endif
}

static int __init moto_mm_init(void)
{
	int ret = 0;

	ret = moto_mm_proc_init();
	if (ret != 0)
		return ret;

	ret = register_all_hooks();
	if (ret != 0) {
		return ret;
	}

	pr_info("moto_mm_init succeed!\n");
	return 0;
}

static void __exit moto_mm_exit(void)
{
	unregister_all_hooks();
	moto_mm_proc_deinit();

	pr_info("moto_mm_exit succeed!\n");
	return;
}

module_init(moto_mm_init);
module_exit(moto_mm_exit);
MODULE_DESCRIPTION("Motorola mm optimizations driver");
MODULE_LICENSE("GPL v2");
