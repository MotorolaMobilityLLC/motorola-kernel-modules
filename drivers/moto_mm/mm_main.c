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

bool mem_alloc_hook_registered = false;

static void tune_inactive_ratio_hook(void *data, unsigned long *inactive_ratio, int file)
{
	if (file)
		*inactive_ratio = min(2UL, *inactive_ratio);
	else
		*inactive_ratio = 1;

	return;
}

//5.15+
static void alloc_pages_slowpath_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_start)
{
	u64 wait_ms = jiffies_to_msecs(jiffies - alloc_start);
	if (moto_alloc_warn_ms > 0 && wait_ms > moto_alloc_warn_ms){
		int fg = task_in_top_app_group(current);
		mm_debug("alloc, %s, %d:%d, order %d, wait %lld ms!\n", (fg? "fg":"bg"), current->tgid, current->pid, order, wait_ms);
	}
}

//5.10
static void alloc_pages_slowpath_begin_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long *alloc_start)
{
	*alloc_start = jiffies;
}

static void alloc_pages_slowpath_end_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_start)
{
	alloc_pages_slowpath_hook(data, gfp_mask, order, alloc_start);
}

int register_mem_alloc_hooks(void)
{
	int rc;

    if (!mem_alloc_hook_registered) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
		REGISTER_HOOK(alloc_pages_slowpath);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
        REGISTER_HOOK(alloc_pages_slowpath_begin);
		REGISTER_HOOK(alloc_pages_slowpath_end);
#endif
		mem_alloc_hook_registered = true;
	}
	return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
ERROR_OUT(alloc_pages_slowpath):
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
ERROR_OUT(alloc_pages_slowpath_begin):
ERROR_OUT(alloc_pages_slowpath_end):
#endif
	return rc;
}

void unregister_mem_alloc_hooks(void)
{
	if (mem_alloc_hook_registered) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
		UNREGISTER_HOOK(alloc_pages_slowpath);
#else
        UNREGISTER_HOOK(alloc_pages_slowpath_begin);
		UNREGISTER_HOOK(alloc_pages_slowpath_end);
#endif
		mem_alloc_hook_registered = false;
	}
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
	unregister_mem_alloc_hooks();
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
