/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 Motorola Mobility LLC.  All rights reserved.
 *
 */

#define pr_fmt(fmt) "uimem: " fmt

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/cgroup.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/oom.h>
#include <linux/mm_inline.h>
#include <linux/mm_types.h>
#include <linux/mmzone.h>
#include <linux/memcontrol.h>
#include <linux/pagemap.h>
#include <linux/page-flags.h>
#include <linux/pageblock-flags.h>
#include <linux/page_owner.h>
#include <linux/page_ref.h>
#include <linux/kasan.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/kmemleak.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <trace/hook_helper.h>
#include <linux/of.h>
#include <linux/stdarg.h>

#include "uimem_trace.h"
#include "uimem.h"
#include "msched_common.h"

#define SUPPLIER_THREAD_NAME "uimem_supplier"
struct pool_supplier {
    struct task_struct *supplier_task;
    wait_queue_head_t waitq;
    unsigned signal;
};

#define WM_LEVEL 2
#define NR_PAGES0 ((SZ_64M + SZ_32M) >> PAGE_SHIFT)
#define NR_PAGES1 (SZ_8M >> PAGE_SHIFT)
/*
    If the available memory is sufficient, use a gradient method
    to fill the pool.
 */
struct pool_property {
    unsigned order;
    unsigned nr_pages;
    int wm[WM_LEVEL];
};

static struct pool_property props_data[] = {
    {
        .order = 0,
        .nr_pages = NR_PAGES0,
        .wm = {NR_PAGES0 * 8 / 10, NR_PAGES0 * 6 / 10},
    },
    //Rarely request order 1.
    {
        .order = 1,
        .nr_pages = NR_PAGES1,
        .wm = {NR_PAGES1 * 8 / 10, NR_PAGES1 * 6 / 10},
    }
};
#define NUM_POOLS ARRAY_SIZE(props_data)

/* data/bss or slab? */
//static struct pool_struct pools[ARRAY_SIZE(props)];
//static struct pool_struct* pools[NUM_POOLS] = {NULL};
static struct pool_property* props = NULL;
static struct pool_struct** pools = {NULL};
static int pools_size = 0;
static struct pool_supplier supplier;
static bool sysctl_uimem_enable = false;
static int* sysctl_uimem_stats =  NULL;  //[NUM_POOLS * POOL_NR_MIGRATE_TYPES];
//static unsigned sysctl_uimem_uxtype = UX_TYPE_ANIMATOR | UX_TYPE_TOPAPP
//                    | UX_TYPE_TOPUI | UX_TYPE_LAUNCHER | UX_TYPE_INHERIT_BINDER
//                    | UX_TYPE_SF | UX_TYPE_LOW_LATENCY_BINDER
//                    | UX_TYPE_SYSUI;
static unsigned sysctl_uimem_uxtype = UX_TYPE_ANIMATOR
                    | UX_TYPE_LAUNCHER | UX_TYPE_INHERIT_BINDER
                    | UX_TYPE_SF | UX_TYPE_LOW_LATENCY_BINDER
                    | UX_TYPE_SYSUI;
static DEFINE_MUTEX(ctl_mutex);

static void trace_pools_status(const char *fmt, ...)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(6, 0, 0)
    va_list args;

    if (trace_pools_status_event_enabled()) {
        va_start(args, fmt);
        trace_pools_status_event(fmt, &args);
        va_end(args);
    }
#endif
}

#if IS_ENABLED(CONFIG_SCHED_MOTO_UNFAIR)
static inline bool is_labeled_task(struct task_struct* t)
{
    int type = task_get_ux_type(t);

    //ANIMATOR, TOP*?, LAUNCHER, *BINDER, SF, SYSUI
    //return (rt_task(t) || (type & 0x21678));
    trace_pools_status("%s: uxtype 0x%X, hit 0x%X", __func__, type,
            type & sysctl_uimem_uxtype);
    return ((t->pid != supplier.supplier_task->pid)
            && (rt_task(t) || (type & sysctl_uimem_uxtype)));
}
#else
static bool is_labeled_task(struct task_struct* t) { return false; }
#endif

static int uimem_find_index_by_order(unsigned order)
{
    int i = 0;

    for (i = 0; i < pools_size; i++) {
        if (order == pools[i]->order)
            return i;
    }

    return -1;
}

static void uimem_wakeup_supplier_thread(void)
{
    supplier.signal = true;
    wake_up_interruptible(&supplier.waitq);
}

static inline void uimem_fill_pool_ifneeded(struct pool_struct* pool, unsigned mt)
{
    if (pool->buoy[mt] < pool->wm_low[mt]) {
        uimem_wakeup_supplier_thread();
    }
}

static int check_system_watermark(void)
{
    return 0;
}

static int uimem_page_pool_add(int index, unsigned mt, struct page * page)
{
    unsigned long flags;
    struct pool_struct* pool = pools[index];

    spin_lock_irqsave(&pool->lock, flags);
    list_add_tail(&page->lru, &pool->page_list[mt]);
    //mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
    //        1 << pool->order);
    pool->buoy[mt]++;
    sysctl_uimem_stats[index * POOL_NR_MIGRATE_TYPES +  mt] = pool->buoy[mt];
    trace_pools_status("%s: index %d, mt %d, pages %d",
        __func__, index, mt, pool->buoy[mt]);
    spin_unlock_irqrestore(&pool->lock, flags);

    return pool->buoy[mt];
}

static struct page *uimem_page_pool_remove(int index, unsigned mt)
{
    unsigned long flags;
    struct page * page = NULL;
    struct pool_struct* pool = pools[index];

    spin_lock_irqsave(&pool->lock, flags);
    page = list_first_entry_or_null(&pool->page_list[mt], struct page, lru);
    if (!page) {
        mt = (mt + 1) % POOL_NR_MIGRATE_TYPES;
        page = list_first_entry_or_null(&pool->page_list[mt], struct page, lru);
    }

    if (page) {
        list_del(&page->lru);
        //mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
        //    -(1 << pool->order));
        pool->buoy[mt]--;
        sysctl_uimem_stats[index * POOL_NR_MIGRATE_TYPES + mt] = pool->buoy[mt];
    }
    trace_pools_status("%s: index %d, mt %d, mt1 %d",
        __func__, index, pool->buoy[0], pool->buoy[1]);

    spin_unlock_irqrestore(&pool->lock, flags);

    uimem_fill_pool_ifneeded(pool, mt);
    return page;
}

static int uimem_fill_pools(void)
{
    int i = 0, j = 0;
    int count = 0;
    struct page * page = NULL;
    struct pool_struct * pool;

    //FIXME: Should be check system water mark, wakeup kswapd.
    // Update wm_high by wm
    check_system_watermark();

    for (i = 0; i < pools_size; i++) {
        pool = pools[i];

        for (j = 0; j < POOL_NR_MIGRATE_TYPES; j++) {
            while (pool->buoy[j] < pool->wm_high[j]) {
                page = alloc_pages(pool->gfp_mask, pool->order);

                if (!page) {
                    trace_pools_status("%s: index %d, mt %d",
                        __func__, i, j);
                    usleep_range(1000, 2000);
                    continue;
                }
                uimem_page_pool_add(i, j, page);
                count++;
            }
        }
    }

    return count;
}

static int uimem_supplier_thread(void * data)
{
    int ret = 0;

    while (!kthread_should_stop()) {
        ret = wait_event_interruptible(supplier.waitq, supplier.signal
                    || kthread_should_stop());

        if (kthread_should_stop())
            break;

        if (ret < 0)
            continue;

        supplier.signal = false;

        uimem_fill_pools();
    }

    return ret;
}

static int uimem_supplier_create(void)
{
    struct sched_param param = { .sched_priority = MAX_RT_PRIO >> 1 };

    supplier.signal = false;
    init_waitqueue_head(&supplier.waitq);
    supplier.supplier_task = kthread_run(uimem_supplier_thread, NULL,
            SUPPLIER_THREAD_NAME);

    if (IS_ERR_OR_NULL(supplier.supplier_task)) {
        pr_err("Failed to create the supplier thread");
        return -EINVAL;
    }
    sched_setscheduler_nocheck(supplier.supplier_task, SCHED_FIFO, &param);

    uimem_wakeup_supplier_thread();
    return 0;
}

static int uimem_pools_create(void)
{
    int i = 0, j = 0;
    int ret = 0;

    pools = kmalloc_array(pools_size, sizeof(*pools), GFP_KERNEL);
    if (!pools) {
        pr_err("No memory to create pools");
        ret = -ENOMEM;
        return ret;
    }

    for (i = 0; i < pools_size; i++) {
        pools[i] = kmalloc(sizeof(struct pool_struct), GFP_KERNEL);
        if (IS_ERR_OR_NULL(pools[i])) {
            pr_err("No memory to create pool item");
            goto error;
        }

        for (j = 0; j < POOL_NR_MIGRATE_TYPES; j++) {
            pools[i]->buoy[j] = 0;
            /* Assuming the available memory is sufficient during initialization */
            pools[i]->wm_high[j] = props[i].wm[0];
            pools[i]->wm_low[j] = props[i].wm[1];
            INIT_LIST_HEAD(&pools[i]->page_list[j]);
        }

        spin_lock_init(&pools[i]->lock);
        pools[i]->gfp_mask = (GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN
                             | __GFP_NORETRY) & ~__GFP_RECLAIM;
        pools[i]->order = props[i].order;
    }

    sysctl_uimem_stats = kmalloc_array(pools_size * POOL_NR_MIGRATE_TYPES,
                            sizeof(int), GFP_KERNEL);
    if (!sysctl_uimem_stats) {
        pr_err("No memory for proc ctl\n");
        goto error;
    }
    memset(sysctl_uimem_stats, 0, pools_size * POOL_NR_MIGRATE_TYPES);

    ret = uimem_supplier_create();
    if (unlikely(ret < 0)) {
        goto error1;
    }
    if (props != props_data)
        kfree(props);

    return 0;

error1:
    kfree(sysctl_uimem_stats);

error:
    for (i = 0; i < pools_size; i++) {
        if (pools[i])
            kfree(pools[i]);
        pools[i] = NULL;
    }
    kfree(pools);

    return -ENOMEM;
}

static void uimem_pools_destory(void)
{
    int i = 0, j = 0;
    unsigned long flags;
    struct page *page, *tmp;

    kthread_stop(supplier.supplier_task);

    for (i = 0; i < pools_size; i++) {
        spin_lock_irqsave(&pools[i]->lock, flags);
        for (j = 0; j < POOL_NR_MIGRATE_TYPES; j++) {
            list_for_each_entry_safe(page, tmp, &pools[i]->page_list[j], lru) {
                list_del(&page->lru);
                __free_pages(page, pools[i]->order);
            }
        }
        spin_unlock_irqrestore(&pools[i]->lock, flags);

        if (pools[i])
            kfree(pools[i]);
        pools[i] = NULL;
    }
    kfree(pools);
}

#define COLWIDTH 4
static int uimem_init_pools_from_dt(void)
{
    struct device_node *np;
    int i = 0, j = 0, count;
    int ret = 0;
    u32* cells;

    np = of_find_compatible_node(NULL, NULL, "moto,uimem");
    if (!np) {
        pr_err("Failed to find device node\n");
        return -ENODEV;
    }

    ret = of_property_read_u32(np, "labeled-thread-mask", &j);
    if (!ret) {
        pr_info("update mask: 0x%x\n", j);
        sysctl_uimem_uxtype = j;
    }
    ret = 0;

    count = of_property_count_u32_elems(np, "moto,uimem_prop");
    if (count <= 0) {
        pr_err("Invalid element number\n");
        ret = -ENODEV;
        goto err;
    }
    if (count % COLWIDTH != 0) {
        pr_err("An unexpected length: %d\n", count);
        ret = -ENODEV;
        goto err;
    }

    cells = kmalloc_array(count, sizeof(u32), GFP_KERNEL);
    if (!cells) {
        pr_err("No memory for parse dt");
        ret = -ENOMEM;
        goto err;
    }

    ret = of_property_read_u32_array(np, "moto,uimem_prop", cells, count);
    if (ret) {
        pr_err("Error reading moto,uimem_prop: %d\n", ret);
        goto out;
    }

    pools_size = count / COLWIDTH;
    props = kmalloc_array(pools_size, sizeof(struct pool_property), GFP_KERNEL);
    if (!props) {
        pr_err("No memory for property\n");
        ret = -ENOMEM;
        goto out;
    }

    for (i = 0; i < pools_size; i++) {
        props[i].order = cells[i * COLWIDTH];
        props[i].nr_pages = cells[i * COLWIDTH + 1] >> PAGE_SHIFT;
        props[i].wm[0] = props[i].nr_pages
                            * cells[i * COLWIDTH + 2] / 100;
        props[i].wm[1] = props[i].nr_pages
                            * cells[i * COLWIDTH + 3] / 100;
        pr_info("order %d, pages %d, wm1 %d, wm2 %d",
                    props[i].order, props[i].nr_pages,
                    props[i].wm[0],
                    props[i].wm[1]);
    }

out:
   kfree(cells);

err:
    of_node_put(np);
    return ret;
}

static int uimem_init_page_pool(void)
{
    int  ret = 0;

    ret = uimem_init_pools_from_dt();
    if (ret < 0) {
        pools_size = NUM_POOLS;
        props = props_data;
    }

    ret = uimem_pools_create();
    if (ret < 0) {
        return ret;
    }

    sysctl_uimem_enable = true;

    return 0;
}

static void uimem_alloc_from_pool(void* data, gfp_t gfp_mask, int order, int alloc_flags,
	int migratetype, struct page **page)
{
    struct page *p = NULL;
    int index = -1;

    if (unlikely(!sysctl_uimem_enable)) {
        goto out;
    }

    if (!is_labeled_task(current) || (gfp_mask & __GFP_DMA32)) {
        goto out;
    }

    index = uimem_find_index_by_order(order);
    if (index < 0 || migratetype > MIGRATE_MOVABLE) {
        goto out;
    }

    /* NOTE: The page is from alloc_pages */
    p = uimem_page_pool_remove(index, migratetype);
    if (p && order && (gfp_mask & __GFP_COMP)) {
        prep_compound_page(p, order);
    }

out:
    *page = p;
}

static void si_meminfo_adjust_hook(void* data, unsigned long *total, unsigned long *free)
{
    struct pool_struct* pool;
    unsigned long flags;
    unsigned long size = 0;
    int i, j;

    if (unlikely(!sysctl_uimem_enable))
        return;

    for (i = 0; i < pools_size; i++) {
        pool = pools[i];
        spin_lock_irqsave(&pool->lock, flags);
        for (j = 0; j < POOL_NR_MIGRATE_TYPES; j++) {
            size += pool->buoy[j] << pool->order;
        }
        spin_unlock_irqrestore(&pool->lock, flags);
    }
    *free += size;
}

static int uimem_register_vendor_hook(void)
{
    int rc = 0;
    //register_trace_android_vh_alloc_pages_reclaim_bypass(uimem_alloc_from_pool, NULL);
    REGISTER_ANDROID_VH_ALIAS_HOOK(alloc_pages_reclaim_bypass, uimem_alloc_from_pool);
    REGISTER_ANDROID_VH_HOOK(si_meminfo_adjust);
    return 0;

ERROR_OUT(si_meminfo_adjust):
    UNREGISTER_ANDROID_VH_ALIAS_HOOK(alloc_pages_reclaim_bypass, uimem_alloc_from_pool);
ERROR_OUT(alloc_pages_reclaim_bypass):
    return rc;
}

static void uimem_unregister_vendor_hook(void)
{
    //unregister_trace_android_vh_alloc_pages_reclaim_bypass(uimem_alloc_from_pool, NULL);
    UNREGISTER_ANDROID_VH_HOOK(si_meminfo_adjust);
    UNREGISTER_ANDROID_VH_ALIAS_HOOK(alloc_pages_reclaim_bypass, uimem_alloc_from_pool);
}

static struct ctl_table uimem_ctl[] = {
	{
		.procname	= "stats",
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "uxtype",
		.data		= &sysctl_uimem_uxtype,
		.maxlen		= sizeof(unsigned),
		.mode		= 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_INT_MAX,
	},
};

static void uimem_register_ctl(void)
{
	struct ctl_table_header *ctl_hdr;

    uimem_ctl[0].data = sysctl_uimem_stats;
    uimem_ctl[0].maxlen = sizeof(int) * pools_size * POOL_NR_MIGRATE_TYPES;
    ctl_hdr = register_sysctl("uimem", uimem_ctl);

	kmemleak_not_leak(ctl_hdr);
}

static int __init uimem_init(void)
{
    int ret = 0;

    ret = uimem_init_page_pool();
    if (ret) {
        return ret;
    }

    ret = uimem_register_vendor_hook();
    if (ret) {
        return ret;
    }

    uimem_register_ctl();
    pr_info("Initialization successful\n");
	return 0;
}

static void __exit uimem_exit(void)
{
    sysctl_uimem_enable = false;

    uimem_unregister_vendor_hook();
    uimem_pools_destory();
}

module_init(uimem_init);
module_exit(uimem_exit);
MODULE_DESCRIPTION("Moto UI Memory Driver");
MODULE_LICENSE("GPL v2");
