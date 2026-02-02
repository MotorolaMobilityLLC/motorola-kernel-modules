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
#include <linux/mmzone.h>
#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
#include <linux/memory-tiers.h>
#endif

#include "mm_common.h"

bool mm_info_initialized = false;
bool mm_init_adjust_zone_wmark_initialized = false;


#if defined(MM_INFO_SUPPORTED)
// 5.15+
static void alloc_pages_slowpath_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_start)
{
	u64 wait_ms = jiffies_to_msecs(jiffies - alloc_start);
	if (moto_alloc_warn_ms > 0 && wait_ms > moto_alloc_warn_ms) {
		int fg = task_in_top_app_group(current);
		pr_info("alloc, %s, %d:%d, order %d, wait %lld ms!\n", (fg ? "fg":"bg"), current->tgid, current->pid, order, wait_ms);
	}
}

// 5.10
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
static void alloc_pages_slowpath_begin_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long *alloc_start)
{
	*alloc_start = jiffies;
}

static void alloc_pages_slowpath_end_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_start)
{
	alloc_pages_slowpath_hook(data, gfp_mask, order, alloc_start);
}
#endif
#endif // defined(MM_INFO_SUPPORTED)

int register_mm_info_hooks(void)
{
#if defined(MM_INFO_SUPPORTED)
	int rc;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	REGISTER_HOOK(alloc_pages_slowpath);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	REGISTER_HOOK(alloc_pages_slowpath_begin);
	REGISTER_HOOK(alloc_pages_slowpath_end);
#endif
	return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
ERROR_OUT(alloc_pages_slowpath):
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
ERROR_OUT(alloc_pages_slowpath_begin):
ERROR_OUT(alloc_pages_slowpath_end):
#endif
	return rc;
#else
	return -1;
#endif // defined(MM_INFO_SUPPORTED)
}

void unregister_mm_info_hooks(void)
{
#if defined(MM_INFO_SUPPORTED)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	UNREGISTER_HOOK(alloc_pages_slowpath);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	UNREGISTER_HOOK(alloc_pages_slowpath_begin);
	UNREGISTER_HOOK(alloc_pages_slowpath_end);
#endif
#endif // defined(MM_INFO_SUPPORTED)
}

int mm_info_init(void)
{
	int ret;

	if (mm_info_initialized) return 0;

	ret = register_mm_info_hooks();
	if (ret != 0)
		return ret;

	mm_info_initialized = true;
	pr_info("mm_info_init succeed!\n");

	return 0;
}

void mm_info_exit(void)
{
	if (!mm_info_initialized) return;

	unregister_mm_info_hooks();

	mm_info_initialized = false;
	pr_info("mm_info_exit succeed!\n");
}

#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
static void init_adjust_zone_wmark_hook(void *data, struct zone *zone, u64 interval)
{
	if (!zone) return;

	if (zone_idx(zone) == ZONE_NORMAL) {
		pr_info("init_adjust_zone_wmark_hook, high=%lumb. high_delta=%ldmb. \n",
			(zone->_watermark[WMARK_HIGH] * 4 / 1024), wmark_high_delta_mb);
		zone->_watermark[WMARK_HIGH] = low_wmark_pages(zone) + interval + wmark_high_delta_mb * 1024 / 4;
	}
}
#endif // defined(MM_NON_LINEAR_WMARK_SUPPORTED)

int register_mm_init_adjust_zone_wmark_hooks(void)
{
#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
	int rc;

	REGISTER_HOOK(init_adjust_zone_wmark);
	return 0;

ERROR_OUT(init_adjust_zone_wmark):
	return rc;
#else
	return -1;
#endif // defined(MM_NON_LINEAR_WMARK_SUPPORTED)
}

void unregister_mm_init_adjust_zone_wmark_hooks(void)
{
#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
	UNREGISTER_HOOK(init_adjust_zone_wmark);
#endif // defined(MM_NON_LINEAR_WMARK_SUPPORTED)
}
int mm_init_adjust_zone_wmark_init(void)
{
	int ret;

	if (mm_init_adjust_zone_wmark_initialized) return 0;

	ret = register_mm_init_adjust_zone_wmark_hooks();
	if (ret != 0)
		return ret;

	mm_init_adjust_zone_wmark_initialized = true;
	pr_info("moto_mm, mm_init_adjust_zone_wmark_init succeed!\n");

	return 0;
}

void mm_init_adjust_zone_wmark_exit(void)
{
	if (!mm_init_adjust_zone_wmark_initialized) return;

	unregister_mm_init_adjust_zone_wmark_hooks();

	mm_init_adjust_zone_wmark_initialized = false;
	pr_info("moto_mm, mm_init_adjust_zone_wmark_exit succeed!\n");
}
