// SPDX-License-Identifier: GPL-2.0-only
/*
 * zram_ext force_shrink_anon logic
 * Based on 5.15 kernel
 */

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

/* Function Pointers */
static kallsyms_lookup_name_t ref_kallsyms_lookup_name;
static swp_swapcount_t ref_swp_swapcount;
static vma_is_shmem_t ref_vma_is_shmem;

static const struct block_device_operations *zram_disk_fops;
zram_oem_func zram_oem_fn = NULL;

unsigned long __nocfi zram_oem_fn_nocfi(int cmd, void *priv, unsigned long param)
{
	return zram_oem_fn(cmd, priv, param);
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

int __nocfi zram_perform_task_eswapout(struct zram* zram, struct task_struct *task)
{
    struct vm_area_struct *vma;
    struct mm_struct *mm;
    struct list_head list;
    struct eswapout_context ctx;
    unsigned long total_found = 0;
    int ret = 0;

    if (!task) {
        pr_err("moto_swap: Error: task is NULL\n");
        return -EINVAL;
    }

    if (!zram_oem_fn) {
        pr_err("moto_swap: Error: zram_oem_fn is NULL\n");
        return -EINVAL;
    }
        
    pr_info("moto_swap: Starting eswapout for task pid=%d comm=%s\n",
            task->pid, task->comm);
    
    INIT_LIST_HEAD(&list);
    
    /* Setup the context */
    ctx.wb_list = &list;
    ctx.found_pages = 0;
    ctx.scanned_pages = 0;
    
    mm = get_task_mm(task);
    if (!mm) {
        pr_warn("moto_swap: Task pid=%d has no mm\n", task->pid);
        return -ESRCH;
    }

    if (mmap_read_lock_killable(mm)) {
        pr_warn("moto_swap: Failed to acquire mmap_lock for pid=%d\n", task->pid);
        mmput(mm);
        return -EINTR;
    }
    
    for (vma = mm->mmap; vma; vma = vma->vm_next) {
        if(atomic_read(&zram->wb_pid_abort)) {
            pr_warn("moto_swap: Writeback aborted for pid=%d\n", task->pid);
            ret = -ECANCELED;
            break;
        }

        /* Basic VMA filtering */
        if (vma->vm_flags & (VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_LOCKED))
            continue;
        
        /* Check Anonymous or Shmem */
        if (!vma_is_anonymous(vma) && !ref_vma_is_shmem(vma))
            continue;
        
        /* Debug: log the VMA range being walked */
        /* pr_debug("moto_swap: Walk VMA %lx-%lx (pid=%d)\n", 
                    vma->vm_start, vma->vm_end, task->pid); */

        walk_page_range(mm, vma->vm_start, vma->vm_end,
                        &eswapout_walk_ops, &ctx);
    }
    
    mmap_read_unlock(mm);
    mmput(mm);
    
    total_found = ctx.found_pages;
    
    if (ret == 0 && !atomic_read(&zram->wb_pid_abort)) {
        if (total_found > 0 && !list_empty(&list)) {
            pr_info("moto_swap: Submitting %lu pages from pid=%d to bdev\n",
                   total_found, task->pid);
            zram_oem_fn(ZRAM_WRITEBACK_LIST, &list, 0);
        } else if (list_empty(&list))
            pr_info("moto_swap: No eligible pages found for pid=%d\n", task->pid);
    } else {
        pr_warn("moto_swap: Writeback aborted/skipped for pid=%d (collected %lu pages)\n", task->pid, total_found);
        if (ret == 0) 
            ret = -ECANCELED;
    }
 
    /* Flush / Cleanup */
    zram_oem_fn(ZRAM_FLUSH_WRITEBACK_BUFFER, &list, 0);
    
    return ret;
}

int zram_shrink_setfops(struct zram *zram)
{
	/* Extract the fops */
	zram_disk_fops = zram->disk->fops;
	pr_info("zram_shrink: Captured zram fops from device %s\n", 
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

    if (!ref_swp_swapcount || !ref_vma_is_shmem) {
        pr_err("moto_swap: Failed to resolve one or more symbols\n");
        return -EINVAL;
    }
    
    return 0;
}

int zram_shrink_init(struct zram *zram)
{
	int ret;

    ret = resolve_hidden_symbols();
    if (ret) {
        pr_err("moto_swap: Symbol resolution failed\n");
        return ret;
    }

	if (!zram || !zram->disk) {
		pr_err("zram_shrink: Invalid zram instance\n");
        ret = -EINVAL;
		return ret;
	}

	pr_info("zram force_shrink_anon interface initialized\n");
	return 0;
}

/*
 * Export the symbol so zram_drv can call it
 */
EXPORT_SYMBOL(zram_shrink_init);



