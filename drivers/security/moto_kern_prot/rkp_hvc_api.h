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
#define MOTO_RKP_SMCID 0x3000000
#endif

enum SMC_FUNC_IDS {
	MARK_RANGE_RO = 0x1,
	LOCK_RKP = 0x2,
	ADD_JUMP_LABEL_LOOKUP = 0x3,
	ALLOC_IMMUTABLE_BITMAP = 0x4, // Not enabled in this release
	MARK_WORD_RO = 0x5, // Not enabled in this release
	PREPARE_BLOCK = 0x6,
	COMMIT_BLOCK = 0x7
};

/**
 * mark_range_ro_smc - Marks a range of memory read-only using the RKP smcID
 *
 * Callers should ensure the inputs always resolve to a fixed value, e.g.
 * _stext. Should be used for large regions of memory spanning multiple pages
 */
int mark_range_ro_smc(uint64_t start, uint64_t end);

/**
 * add_jump_entry_lookup - loads the jet table into the hypervisor
 *
 * Sends the start of the jump entry struct lookup data structure and the size
 * of it to the hypervisor. The hypervisor will use this to determine if a
 * jump entry is valid or not, and if so, allow it to patch the jump entry.
 */
int add_jump_entry_lookup(uint64_t vaddr, uint64_t size);

/**
 * lock_rkp - removes all RKP-related SMC calls
 *
 * Locks down the hypervisor side of RKP so that it may not be called into
 */
int lock_rkp(void);

/**
 * prepare_hugepage - sets up a split hugepage in the hypervisor
 *
 * Prepares a new page table for splitting a 2MB hugepage into 4KB pages
 * in the hypervisor
 */
int prepare_hugepage(uint64_t addr);

/**
 * commit_hugepage - updates TLB for hugepage split by prepare_hugepage
 *
 * Commits a hugepage split by prepare_hugepage into the calling CPU's
 * TLB and page table, must be called from a context where it is impossible
 * for the break-before-make semantics to affect other processor's execution
 */
int commit_hugepage(void);

#endif /* _RKP_HVC_API_H_ */
