// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Motorola Mobility, Inc.
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
 * Test module for motorola's hypervisor-level runtime kernel protections.
 */

#include <linux/highmem.h>
#include <linux/kprobes.h>
#include <linux/list.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/pagewalk.h>
#include <linux/types.h>
#include <asm/pgalloc.h>
#include <mm/pgalloc-track.h>
#include <trace/hooks/fault.h>
#include <trace/hooks/vendor_hooks.h>
#include <fs/erofs/compress.h>
#include "mrkp_test.h"


/*
 * For determining the offsets of kernel code, rodata, etc.
 * kallsyms_lookup_name is no longer exported due to misuse. In this case,
 * however, we want it just to look up very specific constant name strings
 */
static struct kprobe kp_kallsyms_lookup_name = {
				.symbol_name = "kallsyms_lookup_name",
				.addr = 0
};

typedef unsigned long (*kallsyms_lookup_name_t)(const char *name);

/**
 * mod_init - Handles the full initialization of Motorola's RKP
 *
 * Manages the multiprocessor system, allocation of data structures, and
 * modification of memory permissions.
 */
int mrkp_get_krn_region_info(uint64_t *start_addr, uint64_t *end_addr, MRKP_REGION_T region_type)
{
	kallsyms_lookup_name_t kallsyms_lookup_name_ind;

	pr_info("MotoRKP parse the kernel symbol info!\n");

	if (start_addr == NULL || end_addr == NULL) {
		pr_err("MotoRKP: Invalid input addr parameters\n");
		return -EINVAL;
	}

	/* Locate kernel symbol info through the kprobes */
	if (register_kprobe(&kp_kallsyms_lookup_name)) {
		pr_err("MotoRKP failed to register kallsyms kprobe!\n");
		return -EACCES;
	}
	kallsyms_lookup_name_ind = (kallsyms_lookup_name_t)kp_kallsyms_lookup_name.addr;
	if (region_type == KERN_REGION_JEL) {
		*start_addr = kallsyms_lookup_name_ind("__start___jump_table");
		*end_addr = kallsyms_lookup_name_ind("__stop___jump_table");
	}

	if (region_type == KERN_REGION_TEXT) {
		*start_addr = kallsyms_lookup_name_ind("_stext");
		*end_addr = kallsyms_lookup_name_ind("_etext");
	}

	if (region_type == KERN_REGION_RODATA) {
		*start_addr = kallsyms_lookup_name_ind("__start_rodata");
		*end_addr = kallsyms_lookup_name_ind("__hyp_rodata_end");
	}

	/* If we unregister it later, our own protections will create an exception. */
	unregister_kprobe(&kp_kallsyms_lookup_name);

	return 0;
}
