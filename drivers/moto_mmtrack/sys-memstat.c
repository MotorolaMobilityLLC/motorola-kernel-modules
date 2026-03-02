// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/fdtable.h>
#include <linux/seq_file.h>
#include <linux/hashtable.h>
#include <trace/hooks/mm.h>

#include "common.h"
#include "memstat.h"
#include "sys-memstat.h"

static struct proc_dir_entry *mtrack_procs[MTRACK_MAX];
static struct mtrack_debugger *mtrack_debugger[MTRACK_MAX];

static void show_val_kb(struct seq_file *m, const char *s, unsigned long num)
{
	seq_put_decimal_ull_width_dup(m, s, num << (PAGE_SHIFT - 10), 8);
	seq_write(m, " kB\n", 4);
}

void unregister_mtrack_debugger(enum mtrack_type type,
				struct mtrack_debugger *debugger)
{
	mtrack_debugger[type] = NULL;
}
EXPORT_SYMBOL_GPL(unregister_mtrack_debugger);

int register_mtrack_debugger(enum mtrack_type type,
			     struct mtrack_debugger *debugger)
{
	if (!debugger)
		return -EINVAL;

	if (mtrack_debugger[type])
		return -EEXIST;

	mtrack_debugger[type] = debugger;
	return 0;
}
EXPORT_SYMBOL_GPL(register_mtrack_debugger);

int register_mtrack_procfs(enum mtrack_type t, const char *name, umode_t mode,
			   const struct proc_ops *proc_ops, void *data)
{
	struct proc_dir_entry *entry;

	if (!mtrack_procs[t])
		return -EBUSY;

	entry = proc_create_data(name, mode, mtrack_procs[t], proc_ops, data);
	if (!entry)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL_GPL(register_mtrack_procfs);

void unregister_mtrack_procfs(enum mtrack_type t, const char *name)
{
	if (!unlikely(mtrack_procs[t]))
		return;

	remove_proc_subtree(name, mtrack_procs[t]);
}
EXPORT_SYMBOL_GPL(unregister_mtrack_procfs);

inline long read_mtrack_mem_usage(enum mtrack_type t, enum mtrack_subtype s)
{
	struct mtrack_debugger *d = mtrack_debugger[t];

	if (d && d->mem_usage)
		return d->mem_usage(s);
	return 0;
}

inline long read_pid_mtrack_mem_usage(enum mtrack_type t,
				      enum mtrack_subtype s, pid_t pid)
{
	struct mtrack_debugger *d = mtrack_debugger[t];

	if (d && d->pid_mem_usage)
		return d->pid_mem_usage(s, pid);
	return 0;
}

inline void dump_mtrack_usage_stat(struct seq_file *m, enum mtrack_type t, bool verbose)
{
	struct mtrack_debugger *d = mtrack_debugger[t];

	if (d && d->dump_usage_stat) {
		dbg_print(m, "======= dump_%s\n", mtrack_text[t]);
		return d->dump_usage_stat(verbose);
	}
}

static void extra_meminfo_proc_show(void *data, struct seq_file *m)
{
	show_val_kb(m, "IonTotalCache:  ",
		    read_mtrack_mem_usage(MTRACK_DMABUF, MTRACK_DMABUF_POOL));
	show_val_kb(m, "IonTotalUsed:   ",
		    read_mtrack_mem_usage(MTRACK_DMABUF,
					  MTRACK_DMABUF_SYSTEM_HEAP));
	show_val_kb(m, "GPUTotalUsed:   ",
		    read_mtrack_mem_usage(MTRACK_GPU,
					  MTRACK_GPU_TOTAL));
}

int sys_memstat_init(struct proc_dir_entry *root)
{
	struct proc_dir_entry *dir_entry;
	int i;

	if (register_trace_android_vh_meminfo_proc_show(extra_meminfo_proc_show, NULL)) {
		pr_err("register extra meminfo proc failed.\n");
		return -EINVAL;
	}

	/* create mtrack dir here */
	for (i = 0; i < MTRACK_MAX; i++) {
		mtrack_procs[i] = proc_mkdir(mtrack_text[i], root);
		if (!mtrack_procs[i]) {
			osvelte_err("proc_fs: create %s failed\n",
				    mtrack_text[i]);
		}
	}

	dir_entry = mtrack_procs[MTRACK_DMABUF];
	if (dir_entry)
		create_dmabuf_procfs(dir_entry);

	dir_entry = mtrack_procs[MTRACK_ASHMEM];
	if (dir_entry)
		create_ashmem_procfs(dir_entry);
	return 0;
}

int sys_memstat_exit(void)
{
	remove_proc_subtree(DEV_NAME, NULL);
	unregister_trace_android_vh_meminfo_proc_show(extra_meminfo_proc_show, NULL);
	return 0;
}
