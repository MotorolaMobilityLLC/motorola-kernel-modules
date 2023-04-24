// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Motorola Mobility, Inc.
 *
 * Authors: Maxwell Bland
 * Binsheng "Sammy" Que
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Kernel module that initializes motorola's hypervisor-level runtime kernel protections.
 */

#include <linux/highmem.h>
#include <linux/kprobes.h>
#include <linux/list.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pagewalk.h>
#include <linux/types.h>
#include <soc/qcom/watchdog.h>
#include <trace/hooks/fault.h>
#include <trace/hooks/vendor_hooks.h>

#include "patch_lookup_tree.h"
#include "rkp_hvc_api.h"

/*
 * A critical note on why this is included. After our protections initialize, it
 * becomes impossible to register additional kprobes, and thus any kprobe
 * initialization for testing purposes must be done before this kernel module
 * runs. That means a separate module for testing would involve implementing
 * some (likely file-based) IPC, which is decidedly more ugly than bringing
 * in the needed test code/adding a compile-time hook during development.
 */
#ifdef ATTACK_TEST
#include "tests/attack_test.h"
#endif

/*
 * For determining the offsets of kernel code, rodata, etc.
 * kallsyms_lookup_name is no longer exported due to misuse. In this case,
 * however, we want it just to look up very specific constant name strings
 */
static struct kprobe kp_kallsyms_lookup_name = { .symbol_name = "kallsyms_lookup_name", .addr = 0 };
typedef unsigned long (*kallsyms_lookup_name_t)(const char *name);

/*
 * Used to pin pages of the jump_entry lookup table in memory, so that we
 * do not accidentally swap them out
 */
static struct kprobe kp_walk_page_range_novma = { .symbol_name = "walk_page_range_novma",
			     .addr = 0 };
typedef int (*walk_page_range_novma_t)(struct mm_struct *mm,
				       unsigned long start, unsigned long end,
				       const struct mm_walk_ops *ops,
				       pgd_t *pgd, void *private);

static int mem_ready;
module_param(mem_ready, int, 0);
MODULE_PARM_DESC(
	mem_ready,
	"Parameter for loading the module after boot is complete, indicating that the kernel has allocated needed memory/variables and is ready to be protected.");

static int pin_pmd_entry(pmd_t *pmd, unsigned long addr, unsigned long next,
			 struct mm_walk *walk)
{
	pmd_t val;
	struct page *page;

	if (!pmd)
		return -EACCES;

	val = READ_ONCE(*pmd);


	if (pmd_leaf(val)) {
		page = pmd_page(val);
		if (page->flags & PAGE_FLAGS_CHECK_AT_FREE)
			get_page(page);
	}

	return 0;
}

static int pin_pte_entry(pte_t *pte, unsigned long addr, unsigned long next,
			 struct mm_walk *walk)
{
	pte_t val;
	struct page *page;

	if (!pte)
		return -EACCES;

	val = READ_ONCE(*pte);
	page = pte_page(val);

	if (page->flags & PAGE_FLAGS_CHECK_AT_FREE)
		get_page(page);

	return 0;
}

static const struct mm_walk_ops pin_ops = { .pmd_entry = pin_pmd_entry,
					    .pte_entry = pin_pte_entry };

#define pin_range(vsaddr, veaddr, mm) \
	walk_page_range_novma_ind(mm, vsaddr, veaddr, &pin_ops, NULL, NULL)

/**
 * tlb_update_barrier - synchronizes PTE updates on Snapdragon processors
 * @info: an array of volatile ints that indicate whether a processor is
 * locked. We use volatile here: it is necessary as attempting to access other
 * resources (locks, spinlocks) will lead to a fatal page fault on the actively
 * modified kernel code page.
 *
 * Some processors (the 6450) lack a inner-sharable domain for tlb invalidation
 * operations and handle TLB updates out-of-band. Updating the PTEs/TLBs needs
 * to occur for every single processor. In order to do so, we must gate, in
 * order, each processor to preform the update. Once a processor leaves the
 * gate, it can commit an operation following break-before-make semantics. Then
 * it must return to a spinlock in order to allow the other processors to
 * commit their changes.
 *
 * The linux spinlock is not good enough here: it still tries to access page
 * tables, so we use raw arm.
 */
void tlb_update_barrier(void *info)
{
	volatile int *cpuready = (volatile int *)info;
	spinlock_t irqoff = INIT_LOCAL_LOCK("irqnever");
	unsigned long irqflags;
	int cpu = get_cpu();

	spin_lock_irqsave(&irqoff, irqflags);

	flush_cache_all();

	WRITE_ONCE(cpuready[cpu], 1);
	while (true) {
		asm volatile("sevl\n"
			     "wfe;\n"
			     "wfe;\n");
		if (!READ_ONCE(cpuready[cpu]))
			break;
	}

	/*
	 * Note we must absolutely not fail here, since if we do, the TLB will
	 * be invalid for kernel code and we will be unable to recover
	 */
	commit_hugepage();

	WRITE_ONCE(cpuready[cpu], 1);
	asm volatile("sev\n");
	while (true) {
		asm volatile("sevl\n"
			     "wfe;\n"
			     "wfe;\n");
		if (!READ_ONCE(cpuready[cpu]))
			break;
	}

	WRITE_ONCE(cpuready[cpu], 1);

	spin_unlock_irqrestore(&irqoff, irqflags);
	put_cpu();
}

/**
 * on_each_cpu_private - a duplicate of on_each_cpu_mask
 *
 * Google's GKI does not export the on_each_cpu_mask function but does export
 * the equivalent smp function, so this acts as a simple reproduction of
 * the on_each_cpu_mask function
 */
void on_each_cpu_private(const struct cpumask *mask, smp_call_func_t func,
			 void *info, bool wait)
{
	int cpu = get_cpu();
	unsigned long flags;

	smp_call_function_many(mask, func, info, wait);
	if (cpumask_test_cpu(cpu, mask)) {
		local_irq_save(flags);
		func(info);
		local_irq_restore(flags);
	}
	put_cpu();
}

/**
 * try_prepare_hugepage - splits a hugepage in the hypervisor at a vaddr
 * @vaddr: the virtual address of the 2MB page
 *
 * Used to split 2MB hugepages in the hypervisor to 4KB pages with
 * appropriate locking for snapdragon and other processors: puts all
 * the other procs in a lock, then wakes them up one by one to do
 * TLB updates
 *
 * Return: 0 on success (should always happen)
 */
int try_prepare_hugepage(uint64_t vaddr)
{
	cpumask_t cpu_mask = { 0 };
	int num_tlb_waiting = 0;
	int i = 0;
	spinlock_t irqoff = INIT_LOCAL_LOCK("irqnever");
	unsigned long irqflags = 0;
	int cpu = -1;
	volatile int *cpuready =
		kvcalloc((num_present_cpus() + 1), sizeof(int), GFP_KERNEL);
	if (!cpuready) {
		pr_err("Failed to allocate cpuready array for MotoRKP!\n");
		return -ENOMEM;
	}

	cpumask_copy(&cpu_mask, cpu_possible_mask);
	cpumask_clear_cpu(smp_processor_id(), &cpu_mask);
	on_each_cpu_private(&cpu_mask, tlb_update_barrier, (void *)cpuready, 0);

	cpu = get_cpu();
	spin_lock_irqsave(&irqoff, irqflags);

	flush_cache_all();
	while (true) {
		num_tlb_waiting = 0;
		for (i = 0; i < num_present_cpus(); i++) {
			if (READ_ONCE(cpuready[i]))
				num_tlb_waiting++;
		}
		if (num_tlb_waiting == num_present_cpus() - 1)
			break;
	}

	/*
	 * The following call in to the hypervisor and preform
	 * block demotion on the stage-2 page table if it is
	 * needed. The first call identifies the block to split
	 * and allocates a new page table, and the second
	 * performs the break-before-make to update the PTE
	 */
	if (prepare_hugepage(vaddr)) {
		pr_err("MotoRKP failed to prepare hugepage\n");
		return -EACCES;
	}
	if (commit_hugepage()) {
		pr_err("MotoRKP failed to commit hugepage\n");
		return -EACCES;
	}

	/*
	 * We are OK here since each invalidation is local, so we should keep
	 * our "good" mappings, and IRQs are disabled, so the only pages being
	 * accessed here are in the spinlock
	 */
	for (i = 0; i < num_present_cpus(); i++) {
		if (i != cpu) {
			WRITE_ONCE(cpuready[i], 0);
			asm volatile("sev\n");
			while (true) {
				asm volatile("sevl\n"
					     "wfe;\n"
					     "wfe;\n");
				if (READ_ONCE(cpuready[i]))
					break;
			}
		}
	}

	for (i = 0; i < num_present_cpus(); i++) {
		if (i != cpu) {
			WRITE_ONCE(cpuready[i], 0);
			asm volatile("sev\n");
			while (true) {
				if (READ_ONCE(cpuready[i]))
					break;
			}
		}
	}

	spin_unlock_irqrestore(&irqoff, irqflags);
	put_cpu();

	if (cpuready)
		kvfree((void *)cpuready);
	return 0;
}

/**
 * mod_init - Handles the full initialization of Motorola's RKP
 *
 * Manages the multiprocessor system, allocation of data structures, and
 * modification of memory permissions.
 */
static int __init mod_init(void)
{
	uint64_t jel_start, jel_sz, jel_end; /* jump_entry_lookup */
	kallsyms_lookup_name_t kallsyms_lookup_name_ind;
	walk_page_range_novma_t walk_page_range_novma_ind;
	uint64_t start_jump_table, stop_jump_table, stext, etext, start_rodata,
		end_rodata;
	struct mm_struct *mm;

#ifdef ATTACK_TEST
	ATTACK_KERNEL_CODE_DECLS;
#endif

	/*
	 * Ensure that this module is never accidentally insmodded before
	 * kernel memory is mapped in
	 */
	if (!mem_ready) {
		pr_err("MotoRKP waiting to insmod until kernel memory mapped\n");
		return -EACCES;
	}

	pr_info("MotoRKP module loaded!\n");

	if (register_kprobe(&kp_kallsyms_lookup_name)) {
		pr_err("MotoRKP failed to register kallsyms kprobe!\n");
		return -EACCES;
	}
	kallsyms_lookup_name_ind = (kallsyms_lookup_name_t)kp_kallsyms_lookup_name.addr;
	if (register_kprobe(&kp_walk_page_range_novma)) {
		pr_err("MotoRKP failed to register kallsyms kprobe!\n");
		return -EACCES;
	}
	walk_page_range_novma_ind = (walk_page_range_novma_t)kp_walk_page_range_novma.addr;

	start_jump_table = kallsyms_lookup_name_ind("__start___jump_table");
	stop_jump_table = kallsyms_lookup_name_ind("__stop___jump_table");
	stext = kallsyms_lookup_name_ind("_stext");
	etext = kallsyms_lookup_name_ind("_etext");
	start_rodata = kallsyms_lookup_name_ind("__start_rodata");
	end_rodata = kallsyms_lookup_name_ind("__hyp_rodata_end");
	mm = (struct mm_struct *)kallsyms_lookup_name_ind("init_mm");

#ifdef ATTACK_TEST
	if (tc_num == 1) {
		ATTACK_KERNEL_CODE;
		return 0;
	}
#endif

	/* If we unregister it later, our own protections will create an exception */
	unregister_kprobe(&kp_kallsyms_lookup_name);

	jel_start = jet_alloc((struct jump_entry *)start_jump_table,
					 (struct jump_entry *)stop_jump_table,
					 &(jel_sz));
	if (!jel_start) {
		pr_err("MotoRKP failed to allocate memory for jump table lookup\n");
		return -EACCES;
	}
	jel_end = jel_start + jel_sz;
	if (pin_range(jel_start, jel_end, mm)) {
		pr_err("MotoRKP failed to pin jump table lookup in memory\n");
		return -EACCES;
	}

	unregister_kprobe(&kp_walk_page_range_novma);
	if (add_jump_entry_lookup(jel_start, jel_end)) {
		pr_err("MotoRKP failed to add jump label lookup to hypervisor!\n");
		return -EACCES;
	}

	/*
	 * Check the edges of the protection boundary and ensure that if they
	 * do correspond to somewhere in the middle of a hugepage, we properly
	 * demote this block in order to avoid incorrect permissions on
	 * adjacent memory regions
	 */
	try_prepare_hugepage(stext);
	try_prepare_hugepage(etext);

	if (mark_range_ro_smc(stext, etext))
		return -EACCES;

	try_prepare_hugepage(start_rodata);
	try_prepare_hugepage(end_rodata);

	if (mark_range_ro_smc(start_rodata, end_rodata))
		return -EACCES;

	if (lock_rkp())
		return -EACCES;

#ifdef ATTACK_TEST
	if (tc_num == 2)
		ATTACK_KERNEL_CODE;
	else
		attack();
#endif

	return 0;
}

static void __exit mod_exit(void)
{

	pr_info("MotoRKP module exiting. Locking RKP interface just in case.\n");
	lock_rkp();
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maxwell Bland <mbland@motorola.com>");
MODULE_DESCRIPTION(
	"Initializes hypervisor-based Motorola runtime kernel protections.");
