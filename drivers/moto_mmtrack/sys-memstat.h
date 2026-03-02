/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 */
#ifndef _OSVELTE_SYS_MEMSTAT_H
#define _OSVELTE_SYS_MEMSTAT_H

#include <linux/vmstat.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_CONT_PTE_HUGEPAGE
#include "../../../mm/chp_ext.h"
#endif /* CONFIG_CONT_PTE_HUGEPAGE */

enum mtrack_type {
	MTRACK_ASHMEM,
	MTRACK_DMABUF,
	MTRACK_GPU,
	MTRACK_MAX
};

enum mtrack_subtype {
	MTRACK_DMABUF_SYSTEM_HEAP,
	MTRACK_DMABUF_POOL,
	MTRACK_DMABUF_BOOST_POOL,
	MTRACK_GPU_TOTAL,
	MTRACK_GPU_PROC_KERNEL,
	MTRACK_SUBTYPE_MAX
};

static const char * const mtrack_text[MTRACK_MAX] = {
	"ashmem",
	"dma_buf",
	"gpu",
};

struct mtrack_debugger {
	long (*mem_usage)(enum mtrack_subtype type);
	long (*pid_mem_usage)(enum mtrack_subtype type, pid_t pid);
	void (*dump_usage_stat)(bool verbose);
};

static inline unsigned long sys_totalram(void)
{
	return totalram_pages();
}

static inline unsigned long sys_freeram(void)
{
	return global_zone_page_state(NR_FREE_PAGES);
}

static inline unsigned long sys_inactive_file(void)
{
	return global_node_page_state(NR_ACTIVE_FILE);
}

static inline unsigned long sys_active_file(void)
{
	return global_node_page_state(NR_INACTIVE_FILE);
}

static inline unsigned long sys_file(void)
{
	return global_node_page_state(NR_FILE_PAGES);
}

static inline unsigned long sys_slab_reclaimable(void)
{
	return global_node_page_state_pages(NR_SLAB_RECLAIMABLE_B);
}

static inline unsigned long sys_slab_unreclaimable(void)
{
	return global_node_page_state_pages(NR_SLAB_UNRECLAIMABLE_B);
}

static inline unsigned long sys_vmalloc(void)
{
	return vmalloc_nr_pages();
}

static inline unsigned long sys_inactive_anon(void)
{
	return global_node_page_state(NR_INACTIVE_ANON);
}

static inline unsigned long sys_active_anon(void)
{
	return global_node_page_state(NR_ACTIVE_ANON);
}

static inline unsigned long sys_anon(void)
{
	return global_node_page_state(NR_ANON_MAPPED);
}

static inline unsigned long sys_anon_huge(void)
{
	return global_node_page_state(NR_ANON_THPS);
}

static inline unsigned long sys_page_tables(void)
{
	return global_node_page_state(NR_PAGETABLE);
}

static inline unsigned long sys_kernel_stack(void)
{
	return global_node_page_state(NR_KERNEL_STACK_KB) >> (PAGE_SHIFT - 10);
}

static inline unsigned long sys_kernel_misc_reclaimable(void)
{
	return  global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE);
}

static inline unsigned long sys_sharedram(void)
{
	return global_node_page_state(NR_SHMEM);
}

/*
#ifdef CONFIG_CONT_PTE_HUGEPAGE
static inline unsigned long sys_chp_pool_cma(void)
{
	return chp_read_info_ext(CHP_EXT_CMD_POOL_CMA_COUNT) * HPAGE_CONT_PTE_NR;
}

static inline unsigned long sys_chp_pool_buddy(void)
{
	return chp_read_info_ext(CHP_EXT_CMD_POOL_BUDDY_COUNT) * HPAGE_CONT_PTE_NR;
}
#endif
*/
static inline unsigned long sys_free_cma(void)
{
	return global_zone_page_state(NR_FREE_CMA_PAGES);
}

int register_mtrack_debugger(enum mtrack_type type,
			     struct mtrack_debugger *debugger);
void unregister_mtrack_debugger(enum mtrack_type type,
				struct mtrack_debugger *debugger);
int register_mtrack_procfs(enum mtrack_type t, const char *name, umode_t mode,
			   const struct proc_ops *proc_ops, void *data);
void unregister_mtrack_procfs(enum mtrack_type t, const char *name);

int sys_memstat_init(struct proc_dir_entry *root);
int sys_memstat_exit(void);
inline long read_mtrack_mem_usage(enum mtrack_type t, enum mtrack_subtype s);
inline long read_pid_mtrack_mem_usage(enum mtrack_type t, enum mtrack_subtype s,
				      pid_t pid);
inline void dump_mtrack_usage_stat(struct seq_file *m, enum mtrack_type t, bool verbose);

void seq_put_decimal_ull_width_dup(struct seq_file *m, const char *delimiter,
				   unsigned long long num, unsigned int width);

void create_dmabuf_procfs(struct proc_dir_entry *root);
void create_ashmem_procfs(struct proc_dir_entry *root);

#endif /* _OSVELTE_SYS_MEMSTAT_H */
