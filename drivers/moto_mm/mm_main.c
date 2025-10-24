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

// 5.4 - 5.15 tune_inactive_ratio
#if defined(TUNE_INACTIVE_SUPPORTED)
static void tune_inactive_ratio_hook(void *data, unsigned long *inactive_ratio, int file)
{
	if (file)
		*inactive_ratio = min(2UL, *inactive_ratio);
	else
		*inactive_ratio = 1;

	return;
}
#endif

// 5.10 - 5.15 skip drain all pages to speed up direct reclaim
#if defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
static void drain_all_pages_bypass_hook(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_flags,
					int migratetype, unsigned long did_some_progress, bool *bypass)
{
	*bypass = true;
}
#endif

static int register_all_hooks(void)
{
#if defined(TUNE_INACTIVE_SUPPORTED) || defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
	int rc;

#if defined(TUNE_INACTIVE_SUPPORTED)
	REGISTER_HOOK(tune_inactive_ratio);
#endif // defined(TUNE_INACTIVE_SUPPORTED)
#if defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
	REGISTER_HOOK(drain_all_pages_bypass);
#endif // defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
	return 0;

#if defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
	UNREGISTER_HOOK(drain_all_pages_bypass);
ERROR_OUT(drain_all_pages_bypass):
#endif // defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
#if defined(TUNE_INACTIVE_SUPPORTED)
	UNREGISTER_HOOK(tune_inactive_ratio);
ERROR_OUT(tune_inactive_ratio):
#endif // defined(TUNE_INACTIVE_SUPPORTED)
	return rc;

#else
	return 0;
#endif // defined(TUNE_INACTIVE_SUPPORTED) || defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
}

static void unregister_all_hooks(void)
{
#if defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
	UNREGISTER_HOOK(drain_all_pages_bypass);
#endif // defined(DRAIN_ALL_PAGES_BYPASS_SUPPORTED)
#if defined(TUNE_INACTIVE_SUPPORTED)
	UNREGISTER_HOOK(tune_inactive_ratio);
#endif // defined(TUNE_INACTIVE_SUPPORTED)
}

static int __init moto_mm_init(void)
{
	int ret = 0;

	ret = moto_mm_proc_init();
	if (ret != 0)
		return ret;

	ret = register_all_hooks();
	if (ret != 0)
		return ret;

#if defined(MM_INFO_SUPPORTED)
	if (moto_mm_info_enabled > 0)
		mm_info_init();
#endif // defined(MM_INFO_SUPPORTED)

#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
	if (wmark_high_delta_mb != 0)
		mm_init_adjust_zone_wmark_init();
#endif // defined(MM_NON_LINEAR_WMARK_SUPPORTED)

	pr_info("moto_mm_init succeed!\n");
	return 0;
}

static void __exit moto_mm_exit(void)
{
#if defined(MM_NON_LINEAR_WMARK_SUPPORTED)
	mm_init_adjust_zone_wmark_exit();
#endif // defined(MM_NON_LINEAR_WMARK_SUPPORTED)
#if defined(MM_INFO_SUPPORTED)
	mm_info_exit();
#endif // defined(MM_INFO_SUPPORTED)

	unregister_all_hooks();
	moto_mm_proc_deinit();

	pr_info("moto_mm_exit succeed!\n");
	return;
}

module_init(moto_mm_init);
module_exit(moto_mm_exit);
MODULE_DESCRIPTION("Motorola mm optimizations driver");
MODULE_LICENSE("GPL v2");
