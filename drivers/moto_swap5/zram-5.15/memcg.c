// SPDX-License-Identifier: GPL-2.0-only
/*
 * Extracted memcgroup force_shrink_anon logic
 * Based on 5.15 kernel
 */

#define pr_fmt(fmt) "memcg_shrink: " fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/pagemap.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>
#include <linux/proc_fs.h>
#include <linux/swap.h>
#include <linux/version.h>
#include <linux/memcontrol.h>
#include <linux/cgroup.h>
#include <linux/kernfs.h>

#include <linux/kprobes.h>
#include <linux/pagewalk.h>
#include <linux/swapops.h>
#include <linux/memcontrol.h>
#include <linux/sched/mm.h>

#include "zram_drv.h"
#include "zram_drv_internal.h"

typedef struct mem_cgroup_oem_data {
    /* Counter to report back to userspace */
    atomic64_t last_reclaim_count;    
} memcg_oem_t;

static struct kmem_cache *memcg_oem_cache;

/* Helper macro to access OEM data from memcg */
#define MEMCG_OEM_PTR(memcg) ((memcg_oem_t *)((memcg)->android_oem_data1[0]))

/* Helper struct to track progress during the walk */
struct eswapout_context {
    struct list_head *wb_list;
    unsigned long scanned_pages; /* Pages checked */
    unsigned long found_pages;   /* Pages added to writeback list */
};

/* Type definitions for the functions we need to resolve */
typedef unsigned long (*kallsyms_lookup_name_t)(const char *name);

typedef int (*swp_swapcount_t)(swp_entry_t entry);
typedef bool (*vma_is_shmem_t)(struct vm_area_struct *vma);
typedef void (*css_task_iter_start_t)(struct cgroup_subsys_state *css, 
                                      unsigned int flags, 
                                      struct css_task_iter *it);
typedef struct task_struct *(*css_task_iter_next_t)(struct css_task_iter *it);
typedef void (*css_task_iter_end_t)(struct css_task_iter *it);

/* Function Pointers */
static kallsyms_lookup_name_t ref_kallsyms_lookup_name;
static swp_swapcount_t ref_swp_swapcount;
static vma_is_shmem_t ref_vma_is_shmem;
static css_task_iter_start_t ref_css_task_iter_start;
static css_task_iter_next_t ref_css_task_iter_next;
static css_task_iter_end_t ref_css_task_iter_end;

static const struct block_device_operations *zram_disk_fops;
zram_oem_func zram_oem_fn = NULL;

/* 
 * External declaration for 5.15 kernel base.
 * In 5.15, the signature uses 'bool may_swap'.
 */
extern unsigned long try_to_free_mem_cgroup_pages(struct mem_cgroup *memcg,
						  unsigned long nr_pages,
						  gfp_t gfp_mask,
						  bool may_swap);

unsigned long __nocfi zram_oem_fn_nocfi(int cmd, void *priv, unsigned long param)
{
	return zram_oem_fn(cmd, priv, param);
}

static void mem_cgroup_alloc_hook(void *data, struct mem_cgroup *memcg)
{
    memcg_oem_t *oem;

    /* Sanity check: ensure data isn't already there */
    if (MEMCG_OEM_PTR(memcg))
        return;

    oem = kmem_cache_zalloc(memcg_oem_cache, GFP_KERNEL);
    if (!oem)
        return;

    /* Initialize counters */
    atomic64_set(&oem->last_reclaim_count, 0);

    /* Attach to the memcg using the android vendor hook field */
    /* Note: We use android_oem_data1 as the storage pointer */
    memcg->android_oem_data1[0] = (u64)oem;
}

static void mem_cgroup_free_hook(void *data, struct mem_cgroup *memcg)
{
    memcg_oem_t *oem = MEMCG_OEM_PTR(memcg);

    if (!oem)
        return;

    kmem_cache_free(memcg_oem_cache, oem);
    memcg->android_oem_data1[0] = 0;
}

/* 
 * Helper to get local page state, needed for 5.15 
 * as it might not be exported or inline.
 */
static unsigned long memcg_page_state_local(struct mem_cgroup *memcg, int idx)
{
	long x = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		x += per_cpu(memcg->vmstats_percpu->state[idx], cpu);

#ifdef CONFIG_SMP
	if (x < 0)
		x = 0;
#endif
	return x;
}

static unsigned long memcg_anon_pages(struct mem_cgroup *memcg)
{
	if (!memcg)
		return 0;

	/* Logic for 5.15 ( >= 5.10 ) */
	return (memcg_page_state_local(memcg, NR_ACTIVE_ANON) +
		memcg_page_state_local(memcg, NR_INACTIVE_ANON));
}

static unsigned long memcg_inactive_anon_pages(struct mem_cgroup *memcg)
{
	if (!memcg)
		return 0;

	/* Logic for 5.15 ( >= 5.10 ) */
	return memcg_page_state_local(memcg, NR_INACTIVE_ANON);
}

static ssize_t mem_cgroup_force_shrink_anon(struct kernfs_open_file *of,
					    char *buf, size_t nbytes, loff_t off)
{
	struct mem_cgroup *memcg;
	unsigned long nr_need_reclaim, reclaim_total, nr_reclaimed;
	int ret;

	buf = strstrip(buf);
	ret = kstrtoul(buf, 0, &reclaim_total);
	if (unlikely(ret)) {
		pr_err("reclaim_total %s value is error!\n", buf);
		return -EINVAL;
	}

	memcg = mem_cgroup_from_css(of_css(of));

	if (reclaim_total)
		nr_need_reclaim = memcg_anon_pages(memcg);
	else
		nr_need_reclaim = memcg_inactive_anon_pages(memcg);

	/* 
	 * Logic for 5.15:
	 * Uses try_to_free_mem_cgroup_pages with boolean may_swap=true 
	 */
	nr_reclaimed = try_to_free_mem_cgroup_pages(memcg, nr_need_reclaim,
						    GFP_KERNEL, true);

	pr_debug("FORCE SHRINK: to_reclaim %lu reclaimed %lu\n", 
		 nr_need_reclaim, nr_reclaimed);
	
	return nbytes;
}

static int __nocfi eswapout_writeback_pte_range(pmd_t *pmd, unsigned long addr,
                                        unsigned long end, struct mm_walk *walk)
{
    struct eswapout_context *ctx = walk->private;
    struct vm_area_struct *vma = walk->vma;
    pte_t *pte, *orig_pte;
    spinlock_t *ptl;
    swp_entry_t entry;

    if (pmd_trans_unstable(pmd))
        return 0;

    orig_pte = pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
    
    for (; addr < end; pte++, addr += PAGE_SIZE) {
        pte_t ptent = *pte;
        
        ctx->scanned_pages++;

        if (pte_present(ptent)) continue;
        if (!is_swap_pte(ptent)) continue;
        
        entry = pte_to_swp_entry(ptent);
        if (unlikely(non_swap_entry(entry))) continue;
        
        /* Use the function pointer for swapcount */
        if (ref_swp_swapcount(entry) > 1) continue; 

        if (zram_oem_fn) {
            zram_oem_fn(ZRAM_ADD_TO_WRITEBACK_LIST, ctx->wb_list, swp_offset(entry));
            ctx->found_pages++;
            
            /* Verbose debug: log specific offsets found (careful: very noisy) */
            // pr_debug("moto_swap: Found candidate swp_offset=%lu at addr=%lx\n", 
            //          swp_offset(entry), addr);
        }
    }
    
    pte_unmap_unlock(orig_pte, ptl);
    cond_resched();
    return 0;
}

static const struct mm_walk_ops eswapout_walk_ops = {
    .pmd_entry = eswapout_writeback_pte_range,
};

int __nocfi perform_force_eswapout(struct mem_cgroup *memcg)
{
    struct css_task_iter it;
    struct task_struct *task;
    struct list_head list;
    struct eswapout_context ctx;
    memcg_oem_t *oem;
    int ret = 0;
    
    /* Statistics for logging */
    int task_count = 0;
    int mm_lock_fail_count = 0;
    unsigned long total_found = 0;

    pr_info("moto_swap: perform_force_eswapout ENTER (memcg=%p)\n", memcg);

    if (!ref_css_task_iter_start) {
        pr_err("moto_swap: Error: Internal symbols not resolved\n");
        return -EINVAL;
    }

    if (!zram_oem_fn) {
        pr_err("moto_swap: Error: zram_oem_fn is NULL\n");
        return -EINVAL;
    }

    INIT_LIST_HEAD(&list);
    
    /* Setup the context for the walker */
    ctx.wb_list = &list;
    ctx.found_pages = 0;
    ctx.scanned_pages = 0;

    /* Iterate Tasks */
    ref_css_task_iter_start(&memcg->css, CSS_TASK_ITER_PROCS, &it);

    while ((task = ref_css_task_iter_next(&it))) {
        struct vm_area_struct *vma;
        struct mm_struct *mm = get_task_mm(task);
        if (!mm) {
            pr_debug("moto_swap: Skipping task pid=%d (no mm)\n", task->pid);
            continue;
        }

        task_count++;
        
        if (mmap_read_lock_killable(mm)) {
            pr_warn("moto_swap: Failed to acquire mmap_lock for pid=%d\n", task->pid);
            mm_lock_fail_count++;
            mmput(mm);
            continue;
        }

        pr_debug("moto_swap: Scanning pid=%d comm=%s\n", task->pid, task->comm);

        for (vma = mm->mmap; vma; vma = vma->vm_next) {
            
            /* Basic VMA filtering logs */
            if (vma->vm_flags & (VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_LOCKED))
                continue;

            /* Check Anonymous or Shmem */
            if (!vma_is_anonymous(vma) && !ref_vma_is_shmem(vma))
                continue;

            /* Debug: log the VMA range being walked */
            /* pr_debug("moto_swap: Walk VMA %lx-%lx (pid=%d)\n", 
                        vma->vm_start, vma->vm_end, task->pid); */

            walk_page_range(vma->vm_mm, vma->vm_start, vma->vm_end, 
                            &eswapout_walk_ops, &ctx);
        }

        mmap_read_unlock(mm);
        mmput(mm);
    }
    ref_css_task_iter_end(&it);

    total_found = ctx.found_pages;

    pr_info("moto_swap: Scan Summary: Tasks=%d (LockFail=%d), PTEsChecked=%lu, CandidatesFound=%lu\n", 
            task_count, mm_lock_fail_count, ctx.scanned_pages, total_found);

    /* Submit to ZRAM */
    if (ret == 0 && !list_empty(&list)) {
        pr_info("moto_swap: Submitting %lu pages to ZRAM writeback...\n", total_found);
        zram_oem_fn(ZRAM_WRITEBACK_LIST, &list, 0);
    } else if (list_empty(&list)) {
        total_found = 0;
        pr_info("moto_swap: No eligible pages found for writeback.\n");
    }

    /* Flush / Cleanup */
    zram_oem_fn(ZRAM_FLUSH_WRITEBACK_BUFFER, &list, 0);

    oem = MEMCG_OEM_PTR(memcg);
    if (oem) {
        atomic64_set(&oem->last_reclaim_count, total_found);
    }

    pr_info("moto_swap: perform_force_eswapout EXIT (ret=%d)\n", ret);
    return ret;
}

/*
 * Handler for force_eswapout.
 * Triggers the ZRAM driver to reclaim memory from this specific cgroup.
 */
static int mem_cgroup_force_eswapout_write(struct cgroup_subsys_state *css,
                        struct cftype *cft, s64 val)
{
    struct mem_cgroup *memcg = mem_cgroup_from_css(css);
    int ret;

    ret = perform_force_eswapout(memcg);

    return ret;
}

static u64 mem_cgroup_force_eswapout_read(struct cgroup_subsys_state *css,
                                          struct cftype *cft)
{
    struct mem_cgroup *memcg = mem_cgroup_from_css(css);
    memcg_oem_t *oem = MEMCG_OEM_PTR(memcg);

    if (!oem)
        return 0;

    return (u64)atomic64_read(&oem->last_reclaim_count);
}

static struct cftype mem_cgroup_shrink_files[] = {
	{
		.name = "force_shrink_anon",
		.write = mem_cgroup_force_shrink_anon,
	},
    /* Added force_eswapout node */
    {
        .name = "force_eswapout",
        .write_s64 = mem_cgroup_force_eswapout_write,
        .read_u64 = mem_cgroup_force_eswapout_read,
    },
	{ }, /* terminate */
};

int memcg_shrink_setfops(struct zram *zram)
{
	/* Extract the fops */
	zram_disk_fops = zram->disk->fops;
	pr_info("memcg_shrink: Captured zram fops from device %s\n", 
		zram->disk->disk_name);

    if (zram_disk_fops->android_oem_data1)
        zram_oem_fn = (zram_oem_func)zram_disk_fops->android_oem_data1;

    return 0;
}

/* 
 * Backdoor to find kallsyms_lookup_name using a dummy kprobe.
 * This works on almost all kernels because kprobe_register is exported.
 */
static int get_kallsyms_lookup_name_cb(struct kprobe *p, struct pt_regs *regs)
{
    return 0;
}

static int __nocfi lookup_kallsyms_lookup_name(void)
{
    struct kprobe kp = {
        .symbol_name = "kallsyms_lookup_name",
    };
    int ret;

    /* Otherwise, fish it out via kprobe */
    kp.pre_handler = get_kallsyms_lookup_name_cb;
    ret = register_kprobe(&kp);
    if (ret < 0) {
        pr_err("moto_swap: Failed to register kprobe for kallsyms: %d\n", ret);
        return ret;
    }
    
    ref_kallsyms_lookup_name = (kallsyms_lookup_name_t)kp.addr;
    unregister_kprobe(&kp);
    
    if (!ref_kallsyms_lookup_name) {
        pr_err("moto_swap: Failed to resolve kallsyms_lookup_name\n");
        return -EFAULT;
    }

    pr_info("moto_swap: kallsyms_lookup_name found at %p\n", ref_kallsyms_lookup_name);
    return 0;
}

/* Resolve all your required symbols */
static int __nocfi resolve_hidden_symbols(void)
{
    if (lookup_kallsyms_lookup_name() < 0)
        return -EFAULT;

    ref_swp_swapcount = (swp_swapcount_t)
                        ref_kallsyms_lookup_name("swp_swapcount");
    ref_vma_is_shmem = (vma_is_shmem_t)
                       ref_kallsyms_lookup_name("vma_is_shmem");
    ref_css_task_iter_start = (css_task_iter_start_t)
                              ref_kallsyms_lookup_name("css_task_iter_start");
    ref_css_task_iter_next = (css_task_iter_next_t)
                             ref_kallsyms_lookup_name("css_task_iter_next");
    ref_css_task_iter_end = (css_task_iter_end_t)
                            ref_kallsyms_lookup_name("css_task_iter_end");

    if (!ref_swp_swapcount || !ref_vma_is_shmem || 
        !ref_css_task_iter_start || !ref_css_task_iter_next || !ref_css_task_iter_end) {
        pr_err("moto_swap: Failed to resolve one or more symbols\n");
        return -EINVAL;
    }
    
    return 0;
}

static void unregister_hooks(void)
{
    unregister_trace_android_vh_mem_cgroup_alloc(mem_cgroup_alloc_hook, NULL);
    unregister_trace_android_vh_mem_cgroup_free(mem_cgroup_free_hook, NULL);
}

int memcg_shrink_init(struct zram *zram)
{
	int ret;

    ret = resolve_hidden_symbols();
    if (ret) {
        pr_err("moto_swap: Symbol resolution failed\n");
        return ret;
    }

    /* Create the OEM data cache */
    memcg_oem_cache = kmem_cache_create("memcg_oem_cache",
                                        sizeof(memcg_oem_t), 
                                        0, SLAB_PANIC, NULL);
    if (!memcg_oem_cache)
        return -ENOMEM;

    /* Register hooks to allocate/free data for memcgroups */
    ret = register_trace_android_vh_mem_cgroup_alloc(mem_cgroup_alloc_hook, NULL);
    if (ret) goto err_hooks;
    
    ret = register_trace_android_vh_mem_cgroup_free(mem_cgroup_free_hook, NULL);
    if (ret) {
        unregister_trace_android_vh_mem_cgroup_alloc(mem_cgroup_alloc_hook, NULL);
        goto err_hooks;
    }

	if (!zram || !zram->disk) {
		pr_err("memcg_shrink: Invalid zram instance\n");
        ret = -EINVAL;
		goto err_zram;
	}

    ret = cgroup_add_legacy_cftypes(&memory_cgrp_subsys, mem_cgroup_shrink_files);
	if (ret) {
		pr_err("add mem_cgroup_shrink_files failed\n");
		goto err_cgroup;
	}

	pr_info("memcg force_shrink_anon interface initialized\n");
	return 0;

err_cgroup:
err_zram:
    unregister_hooks();
err_hooks:
    kmem_cache_destroy(memcg_oem_cache);
    return ret;
}

/* 
 * Export the symbol so zram_drv can call it 
 */
EXPORT_SYMBOL(memcg_shrink_init);



