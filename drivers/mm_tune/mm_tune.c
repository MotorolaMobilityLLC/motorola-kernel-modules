/*
 * Copyright (C) 2026 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/cma.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/page-isolation.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>


#include <trace/hooks/mm.h>

#define ALLOC_CMA          0x80 /* allow allocations from CMA areas */

#define CMA_ENABLE_TIME_MS 4000
//pages threshed
#define FREE_THRESHOLD (800*1024*1024/PAGE_SIZE)

static int cma_bypass = 1;
module_param(cma_bypass, int, 0640);
static int mem_thresh = FREE_THRESHOLD;
module_param(mem_thresh, int, 0440);


static int bypass_cycle = 25;

static int dump_once = 0;

enum TUNE_EVENT {
	TUNE_EVENT_FLAG,
	TUNE_EVENT_CMA_FLAG,
	TUNE_EVENT_BYPASS,

	NR_TUNE_EVENTS,
};
static atomic64_t tune_event[NR_TUNE_EVENTS];


static void cma_enable_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(cma_enable_work, cma_enable_workfn);

static void cma_enable_workfn(struct work_struct *work)
{
	long num_pages_avail;

	if (!cma_bypass)
		return;
	/* Get system memory information */
	num_pages_avail = si_mem_available();
	if (( num_pages_avail  < mem_thresh ) || (!bypass_cycle))
	{
		pr_info("tune available pages %ld < %d %d \n", num_pages_avail, mem_thresh, bypass_cycle);
		if (bypass_cycle)
			show_mem();
		cma_bypass = 0;
	}else
	{
		pr_debug("tune next cma cnt %d\n",  bypass_cycle);
		bypass_cycle--;
		schedule_delayed_work(&cma_enable_work, msecs_to_jiffies(CMA_ENABLE_TIME_MS));
	}
}
static inline void record_gfp(gfp_t gfp_mask)
{
	atomic64_inc(&tune_event[TUNE_EVENT_FLAG]);
	if ( gfp_mask & __GFP_CMA )
		atomic64_inc(&tune_event[TUNE_EVENT_CMA_FLAG]);
}
#if KERNEL_VERSION(6, 12, 0) <= LINUX_VERSION_CODE
static void oem_vh_calc_alloc_flags(void *data, gfp_t gfp_mask,
						  unsigned int *alloc_flags, bool *bypass)
{
	if (cma_bypass)
	{
		*bypass = 1;
		record_gfp(gfp_mask);
	}
}
#elif KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE
static void oem_vh_alloc_flags_cma_adjust(void *data, gfp_t gfp_mask,
						  unsigned int *alloc_flags)
{
	if (cma_bypass && ( *alloc_flags & ALLOC_CMA ))
	{
		*alloc_flags &= ~ALLOC_CMA;
		record_gfp(gfp_mask);
	}
}
static void oem_vh_cma_alloc_set_max_retries(void *data, int *max_retries)
{
	 *max_retries=10;
}
#endif
static void oem_vh_cma_alloc_fail(void* data, char *name, unsigned long count, unsigned long req_count)
{
	if (cma_bypass && !dump_once ) {
		dump_once = 1;
		show_mem();
	}
}
static void oem_vh_migration_target_bypass(void *data, struct page *page, bool *bypass)
{
	int migrate_type = get_pageblock_migratetype(page);

	if (is_migrate_cma(migrate_type) || is_migrate_isolate(migrate_type)){
		*bypass = true;
		atomic64_inc(&tune_event[TUNE_EVENT_BYPASS]);
	}
}
static int tune_event_proc_show(struct seq_file *s, void *v)
{
	int i;
	for (i=0;i< NR_TUNE_EVENTS;i++)
		seq_printf(s, "%lld ", atomic64_read(&tune_event[i]));
	seq_puts(s, "\n");
	return 0;
}

static int __init mm_tune_init(void)
{
    pr_info("%s init\n", __func__);
#if KERNEL_VERSION(6, 12, 0) <= LINUX_VERSION_CODE
	register_trace_android_vh_calc_alloc_flags(oem_vh_calc_alloc_flags, NULL);
#elif KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE
	register_trace_android_vh_cma_alloc_set_max_retries(oem_vh_cma_alloc_set_max_retries, NULL);
	register_trace_android_vh_alloc_flags_cma_adjust(oem_vh_alloc_flags_cma_adjust, NULL);
#endif
	register_trace_android_vh_migration_target_bypass(oem_vh_migration_target_bypass, NULL);
	register_trace_android_vh_cma_alloc_fail(oem_vh_cma_alloc_fail, NULL);
	schedule_delayed_work(&cma_enable_work, msecs_to_jiffies(CMA_ENABLE_TIME_MS));

	proc_create_single_data("mm_tune",0640, NULL, tune_event_proc_show, NULL);
    return 0;
}

static void __exit mm_tune_exit(void)
{
	remove_proc_entry("mm_tune", NULL);
	cancel_delayed_work(&cma_enable_work);
	unregister_trace_android_vh_cma_alloc_fail(oem_vh_cma_alloc_fail, NULL);
	unregister_trace_android_vh_migration_target_bypass(oem_vh_migration_target_bypass, NULL);
#if KERNEL_VERSION(6, 12, 0) <= LINUX_VERSION_CODE
	unregister_trace_android_vh_calc_alloc_flags(oem_vh_calc_alloc_flags, NULL);
#elif KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE
	unregister_trace_android_vh_cma_alloc_set_max_retries(oem_vh_cma_alloc_set_max_retries, NULL);
	unregister_trace_android_vh_alloc_flags_cma_adjust(oem_vh_alloc_flags_cma_adjust, NULL);
#endif
}
MODULE_IMPORT_NS(MINIDUMP);
module_init(mm_tune_init);
module_exit(mm_tune_exit);
MODULE_LICENSE("GPL v2");
