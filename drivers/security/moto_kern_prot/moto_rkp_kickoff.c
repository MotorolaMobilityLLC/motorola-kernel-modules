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
#include <asm/pgalloc.h>
#include <asm/pgtable-hwdef.h>
#include <mm/pgalloc-track.h>
#include <trace/hooks/fault.h>
#include <trace/hooks/vendor_hooks.h>
#include <fs/erofs/compress.h>

#include "patch_lookup_tree.h"
#include "rkp_hvc_api.h"

/*
 * For determining the offsets of kernel code, rodata, etc.
 * kallsyms_lookup_name is no longer exported due to misuse. In this case,
 * however, we want it just to look up very specific constant name strings
 */
static struct kprobe kp_kallsyms_lookup_name = { .symbol_name =
							 "kallsyms_lookup_name",
						 .addr = 0 };
typedef unsigned long (*kallsyms_lookup_name_t)(const char *name);

static int mem_ready;
module_param(mem_ready, int, 0);
MODULE_PARM_DESC(
	mem_ready,
	"Parameter for loading the module after boot is complete, indicating that the kernel has allocated needed memory/variables and is ready to be protected.");

/**
 * register_contiguous_region - registers the kernel protection of_node
 * @size: output parameter giving the size of the region
 *
 * Grabs the of_node and resource for the region of memory protected
 * and used by the hypervisor to manage kernel protections.
 *
 * Return: physical address of the region on success, 0 on failure
 */
static uint64_t register_contiguous_region(uint64_t *size)
{
	struct device_node *el2_memory_node = 0;
	struct resource el2_memory_resource = { 0 };

	el2_memory_node = of_find_node_by_name(NULL, "kern_prot_region");
	if (!el2_memory_node) {
		pr_err("%s fail: of_find_node_by_name\n", __func__);
		return 0;
	}
	if (of_address_to_resource(el2_memory_node, 0, &el2_memory_resource)) {
		pr_err("%s fail: of_address_to_resource\n", __func__);
		return 0;
	}
	*size = resource_size(&el2_memory_resource) - PAGE_SIZE;

	return el2_memory_resource.start;
}

/**
 * jel_init - Initializes the EL2 jump entry table lookup
 * @start_jump_table: start of jump entry table (vaddr)
 * @stop_jump_table: end of jump entry table (vaddr)
 * @jet_end: output parameter: vaddr of end of jump entry table
 * @c_base_ptr: the physical address of the base of a physically contiguous memory region at which to allocate
 * @c_sz: the size of the physically contiguous memory region at which to allocate
 *
 * Constructs a bitwise trie for jump entry exceptions to
 * kernel code immutability. Does not signal to EL2 to
 * lock this memory.
 *
 * Return: virtual address of jump entry table on success, 0 on failure
 */
uint64_t jel_init(uint64_t start_jump_table, uint64_t stop_jump_table,
		  uint64_t *jel_end, uint64_t c_base_ptr, uint64_t c_sz)
{
	uint64_t jel_start = 0;
	uint64_t jel_sz = 0;
	uint64_t contig_vaddr = 0;

	jel_start = jet_alloc((struct jump_entry *)start_jump_table,
			              (struct jump_entry *)stop_jump_table,
                          &jel_sz);
	if (!jel_start) {
		pr_err("MotoRKP failed to allocate memory for jump table lookup\n");
		return 0;
	}
	if ((jel_sz * sizeof(union jump_tree_node)) >= c_sz) {
		pr_err("MotoRKP jump table lookup too large!\n");
		kvfree((void *)jel_start);
		return 0;
	}
	contig_vaddr = (uint64_t)ioremap(c_base_ptr, c_sz);
	memcpy((uint64_t *)contig_vaddr, (void *)jel_start,
	       jel_sz * sizeof(union jump_tree_node));

	*jel_end = jel_start + (jel_sz * sizeof(union jump_tree_node));

	return jel_start;
}

/**
 * mod_init - Handles the full initialization of Motorola's RKP
 *
 * Manages the multiprocessor system, allocation of data structures, and
 * modification of memory permissions.
 */
static int __init mod_init(void)
{
	uint64_t jel_vaddr, jel_end, jel_sz; /* jump_entry_lookup */
	kallsyms_lookup_name_t kallsyms_lookup_name_ind;
        uint64_t start_jump_table, stop_jump_table, stext, etext, stext_vaddr,
                etext_vaddr, start_rodata, end_rodata;
        uint64_t c_region_size = 0;
	uint64_t c_region_paddr = 0;

	/*
         * Ensure that this module is never accidentally insmodded before
         * kernel memory is mapped in
         */
	if (!mem_ready) {
		pr_err("MotoRKP waiting to insmod until kernel memory mapped\n");
		return -EACCES;
	}

	pr_info("MotoRKP module loaded!\n");

	/* Locate kernel symbol info through the kprobes */
	if (register_kprobe(&kp_kallsyms_lookup_name)) {
		pr_err("MotoRKP failed to register kallsyms kprobe!\n");
		return -EACCES;
	}
	kallsyms_lookup_name_ind =
		(kallsyms_lookup_name_t)kp_kallsyms_lookup_name.addr;

	/* Get the addresses of everything we need to protect */

	/* 3.0 version will need the vaddrs */
	stext_vaddr = kallsyms_lookup_name_ind("_stext");
	etext_vaddr = kallsyms_lookup_name_ind("_etext");

	/* Initialization of paddrs for direct immutability */
	start_jump_table = kallsyms_lookup_name_ind("__start___jump_table");
	stop_jump_table = kallsyms_lookup_name_ind("__stop___jump_table");
	stext = __virt_to_phys(stext_vaddr);
	etext = __virt_to_phys(etext_vaddr);
	start_rodata =
		__virt_to_phys(kallsyms_lookup_name_ind("__start_rodata"));
	end_rodata =
		__virt_to_phys(kallsyms_lookup_name_ind("__hyp_rodata_end"));

	/* If we unregister it later, our own protections will create an exception */
	unregister_kprobe(&kp_kallsyms_lookup_name);

	/* Register our contiguous memory area with the hypervisor */
	c_region_paddr = register_contiguous_region(&c_region_size);
	if (!c_region_paddr) {
		pr_err("MotoRKP failed to register contiguous vmap!\n");
		return -EINVAL;
	}

	jel_vaddr = jel_init(start_jump_table, stop_jump_table, &jel_end,
			     c_region_paddr, c_region_size);
	if (!jel_vaddr) {
		pr_err("MotoRKP failed to init the jel!\n");
		return -EACCES;
	}

	jel_sz = ((jel_end - jel_vaddr) + PAGE_SIZE) & 0xFFFFFFFFFFFFF000;
	add_jump_entry_lookup(c_region_paddr, jel_sz);
	amem_register(c_region_paddr + jel_sz, c_region_size - jel_sz);

	mark_range_ro_smc(c_region_paddr, c_region_paddr + c_region_size,
			  KERN_PROT_GENERIC);

	if (CONFIG_PGTABLE_LEVELS != 3) {
		pr_err("MotoRKP does not support EL1 P4D, PUD page table configurations!\n");
		return -EINVAL;
	}

	/* These are guaranteed to be OK at page granularity by bootloader-level
         * hugepage splitting */
	mark_range_ro_smc(stext, etext, KERN_PROT_GENERIC);
	mark_range_ro_smc(start_rodata, end_rodata, KERN_PROT_GENERIC);

	/* Lock down RKP API to prevent further abuse from guest OS */
	lock_rkp();

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
