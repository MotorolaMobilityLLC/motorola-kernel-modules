/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Motorola Mobility, Inc.
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
 * Defines the kernel-side API for runtime kernel protections as they
 * call into the hypervisor.
 */
#ifndef _RKP_HVC_API_H_
#define _RKP_HVC_API_H_

#ifdef MTK_PLATFORM
#define MOTO_RKP_SMCID 0x82000000
#else
#define MOTO_RKP_SMCID 0x43000000
#endif

enum SMC_FUNC_IDS {
	KERN_MARK_RANGE_RO_SMC_ID = 0x1,
	KERN_LOCK_RKP_SMC_ID = 0x2,
	KERN_ADD_JUMP_ENTRY_LOOKUP_SMC_ID = 0x3,
	KERN_SPLIT_BLOCK_SMC_ID = 0x6,
	KERN_COMMIT_BLOCK_SMC_ID = 0x7,
	KERN_REGISTER_AMEM_SMC_ID = 0x8,
	// KERN_MARK_PT_RO_SMC_ID = 0x9,
	KERN_COMM_EL1_PT_SMC_ID = 0xA
};

enum MEM_PROT_TYPE {
	KERN_PROT_GENERIC = 0x0,
	KERN_PROT_PAGE_TABLE = 0x1,
};

/**
 * mark_range_ro_smc - Marks a range of memory read-only using the RKP smcID
 * @start: start of the range to mark ro
 * @end: end of the range to mark ro
 * @type: type of the memory with 0 being generic, 1 being page table,
 * and others yet undefined
 *
 * Callers should ensure the inputs always resolve to a fixed value, e.g.
 * _stext. Should be used for large regions of memory spanning multiple pages
 */
void mark_range_ro_smc(uint64_t start, uint64_t end, uint64_t type);

/**
 * add_jump_entry_lookup - loads the jet table into the hypervisor
 *
 * Sends the start of the jump entry struct lookup data structure and the size
 * of it to the hypervisor. The hypervisor will use this to determine if a
 * jump entry is valid or not, and if so, allow it to patch the jump entry.
 */
void add_jump_entry_lookup(uint64_t paddr, uint64_t size);

/**
 * lock_rkp - removes all RKP-related SMC calls
 *
 * Locks down the hypervisor side of RKP so that it may not be called into
 */
void lock_rkp(void);

/**
 * register_amem - registers contiguous physical memory region as additional
 * @paddr_start: start of physical memory region
 * @size: size of physical memory region
 *
 * Registers a contiguous physical memory region as additional memory for
 * the hypervisor to use. This memory should be marked RO ahead of time and
 * not be writable by the kernel.
 */
void amem_register(uint64_t paddr_start, uint64_t size);

/**
 * communicate_el1_pt - saves the EL1 page table information for this VM at EL2
 * @pgd: physical address of the page table
 * @pgd_ind: index of the pgd entry
 * @pmd_ind: index of the pmd entry
 * @pte_ind: index of the pte entry
 * @virt_offset: offset of virtual addresses from physical ones
 *
 * Of course, assumes p4d and pud are unused. This is used to communicate
 * the EL1 page table information to the hypervisor. Then, once RO protections
 * are appplied to the page table, the hypervisor will be able to enforce
 * immutability on the page table entries, i.e. writes to the page table may
 * consist of _existing_ entries only (remappings or double mappings of
 * virtual memory) but not _new_ or mutated entries.
 */
void comm_el1_pt(uint64_t pgd);

#endif /* _RKP_HVC_API_H_ */
