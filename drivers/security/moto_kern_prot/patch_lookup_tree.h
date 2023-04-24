/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2023 Motorola Mobility, Inc.
 *
 * Author: Maxwell Bland
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
 * O(64) time bitwise trie data structure for storing jump label code addresses.
 * To perform a lookup, we iterate over the bits of the jump_label from MSB to
 * LSB, and use the bit as a tree traversal.
 *
 * Ideally we would just store this in the hypervisor, but we don't have enough
 * memory.  At the time of writing there are 56404 jump labels in the kernel,
 * and HypX is restricted to a memory footprint of 32 KB.
 *
 * So instead what we do is allocate this data structure in the kernel module,
 * and then lock it down. Finally, we pass the pointer to the hypervisor, which
 * can then perform lookups whenever a DABT exception occurs.
 *
 * We choose not to use the kernel's radix tree for this because it is less
 * space efficient and much slower, and jump_label operations relate to
 * performance sensitive code (e.g. the networking stack).
 */

#ifndef JUMP_TABLE_LOOKUP_H
#define JUMP_TABLE_LOOKUP_H

#include <linux/dma-mapping.h>

/* Used to determine the branch direction */
#define JUMP_TREE_BRANCH(x) ((x >> ((sizeof(x) * 8) - 1)) & 1)

union jump_tree_node {
	uint32_t je_index[2];
	uint64_t je_ptr;
};

static union jump_tree_node *jet; /* Start of jump table lookup */
static uint64_t jet_ind; /* Current index of jump table lookup */
static union jump_tree_node *jet_end; /* End of jump table lookup */

void init_jtn(union jump_tree_node *jtn)
{
	jtn->je_ptr = 0;
}

/**
 * jet_extend - extends the jump entry lookup table
 *
 * Scales up the size of the jump entry lookup table by 2 just in case our
 * initial estimate was too small. Ensures that the new size is a multiple of
 * PAGE_SIZE.
 */
void jet_extend(void)
{
	uint64_t jet_sz = jet_end - jet;
	uint64_t new_sz = jet_sz * 2;
	union jump_tree_node *new_jet;

	if (new_sz % PAGE_SIZE)
		new_sz += PAGE_SIZE - (new_sz % PAGE_SIZE);

	if (jet_ind + 1 >= jet_sz) {
		new_jet = kvmalloc(new_sz * sizeof(union jump_tree_node),
				   GFP_KERNEL);
		memcpy(new_jet, jet, jet_sz * sizeof(union jump_tree_node));
		jet_end = new_jet + new_sz;
		kvfree(jet);
		jet = new_jet;
	}

	init_jtn(jet + jet_ind);
}

/**
 * add_branch - adds a branch to the jump entry lookup tree
 *
 * Adds a branch to the current node corresponding to whether the LSB of the
 * cur_code_ptr is 1 or 0. The branch target is the index of the next node in
 * the tree or a leaf (jump entry)
 */
int add_branch(unsigned long *cur_code_ptr, union jump_tree_node *cur_node,
		uint64_t branch_target)
{
	bool bit;

	if (!cur_code_ptr || !cur_node)
		return -EACCES;

	bit = JUMP_TREE_BRANCH(*cur_code_ptr);
	*cur_code_ptr <<= 1;

	if (bit)
		cur_node->je_index[0] = branch_target;
	else
		cur_node->je_index[1] = branch_target;

	return 0;
}

/**
 * add_trunk - adds a path of branches to the jump entry lookup tree
 *
 * Adds the series of branches to the jump entry lookup tree corresponding to
 * the bits in the cur_code_ptr value starting at LSB index i, and adds the leaf
 * node at the end.
 */
int add_trunk(union jump_tree_node *cur_node, struct jump_entry *je,
	       unsigned long cur_code_ptr, int i)
{
	if (!cur_node || !je)
		return -EACCES;

	for (; i < sizeof(cur_code_ptr) * 8; i++) {
		if (add_branch(&cur_code_ptr, cur_node, jet_ind))
			return -EACCES;
		jet_extend();
		cur_node = jet + jet_ind;
		jet_ind++;
	}
	cur_node->je_ptr = (uint64_t)je;

	return 0;
}

/**
 * jet_alloc - allocates a new jump entry tree for lookups
 * @jet_end_out: output parameter for the size of the jump entry tree (bytes)
 *
 * Allocates the jump entry tree with 2x the number of jump entries in the
 * kernel. This is because we need to store both the jump entry and, in the
 * worst case, a pointer to the next index in the tree. Ensures the total memory
 * allocated is a multiple of PAGE_SIZE, uses kvmalloc (the memory is
 * non-contiguous)
 *
 * Return: 0 on failure, or the pointer to the allocated jump entry tree
 */
uint64_t jet_alloc(struct jump_entry *je_start, struct jump_entry *je_end,
		   uint64_t *jet_size_out)
{
	union jump_tree_node *cur_node;
	unsigned long cur_code_ptr = -1;
	bool bit;
	int i = 0;

	if (!je_start || !je_end || !jet_size_out) {
		pr_err("%s fail: invalid parameters\n", __func__);
		return 0;
	}

	if (je_start >= je_end) {
		pr_err("%s fail: je_start >= je_end\n", __func__);
		return 0;
	}

	jet = kvmalloc(2 * (je_end - je_start) * sizeof(union jump_tree_node),
		       GFP_KERNEL);
	jet_ind = 1;
	jet_end = jet + 2 * (je_end - je_start);
	init_jtn(jet);
	init_jtn(jet + 1);

	while (je_start != je_end) {
		cur_code_ptr = __virt_to_phys(jump_entry_code(je_start));
		cur_node = jet;

		// iterate over the bits of the jump label
		for (i = 0; i < sizeof(cur_code_ptr) * 8; i++) {
			bit = JUMP_TREE_BRANCH(cur_code_ptr);

			if (bit) {
				if (cur_node->je_index[0]) {
					cur_node = jet + cur_node->je_index[0];
				} else {
					if (add_trunk(cur_node, je_start,
						  cur_code_ptr, i))
						return 0;
					break;
				}
			} else {
				if (cur_node->je_index[1]) {
					cur_node = jet + cur_node->je_index[1];
				} else {
					if (add_trunk(cur_node, je_start,
						  cur_code_ptr, i))
						return 0;
					break;
				}
			}

			cur_code_ptr <<= 1;
		}

		je_start++;
	}

	*jet_size_out = jet_end - jet;

	return (uint64_t)jet;
}

#endif
