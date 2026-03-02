// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2023 Oplus. All rights reserved.
 */
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/seq_file.h>
#include <trace/hooks/mm.h>
#include <linux/fdtable.h>
#include <linux/hashtable.h>

#include "common.h"
#include "sys-memstat.h"
#include "memstat.h"

#define ASHMEM_NAME_LIMIT_LEN (80)
#define DEFINE_PROC_SHOW_ATTRIBUTE_SIZE_BUF(__name, buf_sz)		\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open_size(file, __name ## _show, PDE_DATA(inode), \
				buf_sz);				\
}									\
									\
static const struct proc_ops __name ## _proc_ops = {			\
	.proc_open	= __name ## _open,				\
	.proc_read	= seq_read,					\
	.proc_lseek	= seq_lseek,					\
	.proc_release	= single_release,				\
}

struct ashmem_info {
	char name[ASHMEM_NAME_LIMIT_LEN];
	unsigned long size;
	unsigned long i_ino;
	unsigned long file_count;
	struct ashmem_area *asma;
	struct hlist_node head;
};

struct ashmem_proc {
	char name[TASK_COMM_LEN];
	pid_t pid;
	size_t size;
	struct hlist_head ashmem_bufs[1 << 10];
	struct list_head head;
};

static int get_ashmem_buf_info(const void *data, struct file *file, unsigned int n)
{
	struct ashmem_proc *ashmem_proc;
	struct ashmem_info *ashmem_info;
	struct ashmem_area *asma;

	if (!is_ashmem_file(file))
		return 0;

	asma = file->private_data;
	if (!asma || !asma->size || !asma->file)
		return 0;

	ashmem_proc = (struct ashmem_proc *)data;
	hash_for_each_possible(ashmem_proc->ashmem_bufs, ashmem_info, head,
			       (unsigned long)asma)
		if (asma == ashmem_info->asma)
			return 0;

	ashmem_info = kzalloc(sizeof(*ashmem_info), GFP_ATOMIC);
	if (!ashmem_info)
		return -ENOMEM;

	ashmem_info->asma = asma;
	if (asma->name[ASHMEM_NAME_PREFIX_LEN] != '\0') {
		strncpy(ashmem_info->name, asma->name + ASHMEM_NAME_PREFIX_LEN,
			ASHMEM_NAME_LIMIT_LEN - 1);
	} else {
		memcpy(ashmem_info->name, ASHMEM_NAME_DEF,
		       sizeof(ASHMEM_NAME_LEN));
	}
	ashmem_info->i_ino = file_inode(asma->file)->i_ino;
	ashmem_info->file_count = file_count(asma->file);
	ashmem_info->size = asma->size;

	ashmem_proc->size += asma->size;
	hash_add(ashmem_proc->ashmem_bufs, &ashmem_info->head,
		 (unsigned long)ashmem_info->asma);
	return 0;
}

static void free_ashmem_proc(struct ashmem_proc *proc)
{
	struct ashmem_info *tmp;
	struct hlist_node *n;
	int i;

	hash_for_each_safe(proc->ashmem_bufs, i, n, tmp, head) {
		hash_del(&tmp->head);
		kfree(tmp);
	}
	kfree(proc);
}

static void ashmem_proc_show(struct seq_file *s, struct ashmem_proc *proc)
{
	struct ashmem_info *tmp;
	int i;

	seq_printf(s, "\n%s (PID %d) size: %ld kB\nASHMEM Buffers:\n",
		   proc->name, proc->pid, proc->size / SZ_1K);
	seq_printf(s, "%-8s\t%-8s\t%-8s\t%-s\n",
		   "ino", "size", "count", "name");

	hash_for_each(proc->ashmem_bufs, i, tmp, head) {
		seq_printf(s, "%08lu\t%-8zu\t%-8ld\t%-s\n",
			   tmp->i_ino,
			   tmp->size / SZ_1K,
			   tmp->file_count,
			   tmp->name);
	}
}

static int ashmem_buf_procinfo_show(struct seq_file *s, void *unused)
{
	struct task_struct *task, *thread;
	struct files_struct *files;
	int ret = 0;
	struct ashmem_proc *tmp, *n;
	LIST_HEAD(plist);

	osvelte_info("%s:%d read %s, seq_buf size:%zu\n",
		     current->comm, current->tgid, __func__, s->size);

	rcu_read_lock();
	for_each_process(task) {
		struct files_struct *group_leader_files = NULL;

		tmp = kzalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp) {
			ret = -ENOMEM;
			rcu_read_unlock();
			goto mem_err;
		}
		hash_init(tmp->ashmem_bufs);
		for_each_thread(task, thread) {
			task_lock(thread);
			if (unlikely(!group_leader_files))
				group_leader_files = task->group_leader->files;
			files = thread->files;
			if (files && (group_leader_files != files ||
				      thread == task->group_leader))
				ret = iterate_fd(files, 0, get_ashmem_buf_info, tmp);
			task_unlock(thread);
		}

		if (ret || hash_empty(tmp->ashmem_bufs))
			goto skip;

		get_task_comm(tmp->name, task);
		tmp->pid = task->tgid;
		list_add(&tmp->head, &plist);
		continue;
skip:
		free_ashmem_proc(tmp);
	}
	rcu_read_unlock();

	list_for_each_entry(tmp, &plist, head)
		ashmem_proc_show(s, tmp);

	ret = 0;
mem_err:
	list_for_each_entry_safe(tmp, n, &plist, head) {
		list_del(&tmp->head);
		free_ashmem_proc(tmp);
	}
	return ret;
}
DEFINE_PROC_SHOW_ATTRIBUTE_SIZE_BUF(ashmem_buf_procinfo, SZ_256K);

void create_ashmem_procfs(struct proc_dir_entry *root)
{
	OSVELTE_STATIC_ASSERT(sizeof(struct ashmem_info) <= 128);

	proc_create("procinfo", 0444, root, &ashmem_buf_procinfo_proc_ops);
}
