// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2025 Moto LLC
 */

#include <linux/module.h>
#include <trace/hooks/mm.h>
#include "vmscan.h"

extern void vh_rmqueue_mod(void *data, struct zone *preferred_zone,
		struct zone *zone, unsigned int order, gfp_t gfp_flags,
		unsigned int alloc_flags, int migratetype);
extern int moto_mm_sysfs(void);

extern void rvh_mapping_shrinkable(void *data, bool *shrinkable);

static int moto_stat_mm_init(void)
{
	int ret;

	ret = moto_mm_sysfs();
	if (ret)
		return ret;

	ret = register_trace_android_vh_rmqueue(vh_rmqueue_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_mm_vmscan_direct_reclaim_begin(vh_direct_reclaim_begin, NULL);
	if (ret)
		return ret;

	ret = register_trace_mm_vmscan_direct_reclaim_end(vh_direct_reclaim_end, NULL);
	if (ret)
		return ret;

	return 0;
}

module_init(moto_stat_mm_init);
MODULE_SOFTDEP("pre: moto_stat_sysfs");
MODULE_LICENSE("GPL v2");
