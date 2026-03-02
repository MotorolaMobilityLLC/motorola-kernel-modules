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

#define EXP_NAME_LEN (32)
#define NAME_LEN (32)

struct dma_info {
	char exp_name[EXP_NAME_LEN];
	char name[NAME_LEN];
	fmode_t f_mode;
	unsigned int f_flags;
	unsigned long size;
	unsigned long i_ino;
	unsigned long file_count;
	struct dma_buf *dmabuf;
	struct hlist_node head;
};

struct dma_proc {
	char name[TASK_COMM_LEN];
	pid_t pid;
	size_t size;
	struct hlist_head dma_bufs[1 << 10];
	struct list_head head;
};

struct dma_buf_priv {
	int count;
	size_t size;
	struct seq_file *s;
};

static int dma_buf_show(const struct dma_buf *buf_obj, void *private)
{
	int ret;
	struct dma_buf_attachment *attach_obj;
	struct dma_resv *robj;
	struct dma_resv_list *fobj;
	struct dma_fence *fence;
	unsigned seq;
	int attach_count, shared_count, i;
	struct dma_buf_priv *buf = (struct dma_buf_priv *)private;
	struct seq_file *s = buf->s;

	ret = dma_resv_lock_interruptible(buf_obj->resv, NULL);

	if (ret)
		goto err;

	spin_lock((spinlock_t *)&buf_obj->name_lock);
	seq_printf(s, "%08zu\t%08x\t%08x\t%08ld\t%s\t%08lu\t%s\n",
		   buf_obj->size,
		   buf_obj->file->f_flags, buf_obj->file->f_mode,
		   file_count(buf_obj->file),
		   buf_obj->exp_name,
		   file_inode(buf_obj->file)->i_ino,
		   buf_obj->name ?: "");
	spin_unlock((spinlock_t *)&buf_obj->name_lock);

	robj = buf_obj->resv;
	while (true) {
		seq = read_seqcount_begin(&robj->seq);
		rcu_read_lock();
		fobj = rcu_dereference(robj->fence);
		shared_count = fobj ? fobj->shared_count : 0;
		fence = rcu_dereference(robj->fence_excl);
		if (!read_seqcount_retry(&robj->seq, seq))
			break;
		rcu_read_unlock();
	}

	if (fence)
		seq_printf(s, "\tExclusive fence: %s %s %ssignalled\n",
			   fence->ops->get_driver_name(fence),
			   fence->ops->get_timeline_name(fence),
			   dma_fence_is_signaled(fence) ? "" : "un");
	for (i = 0; i < shared_count; i++) {
		fence = rcu_dereference(fobj->shared[i]);
		if (!dma_fence_get_rcu(fence))
			continue;
		seq_printf(s, "\tShared fence: %s %s %ssignalled\n",
			   fence->ops->get_driver_name(fence),
			   fence->ops->get_timeline_name(fence),
			   dma_fence_is_signaled(fence) ? "" : "un");
		dma_fence_put(fence);
	}
	rcu_read_unlock();

	seq_puts(s, "\tAttached Devices:\n");
	attach_count = 0;

	list_for_each_entry(attach_obj, &buf_obj->attachments, node) {
		seq_printf(s, "\t%s\n", dev_name(attach_obj->dev));
		attach_count++;
	}
	dma_resv_unlock(buf_obj->resv);

	seq_printf(s, "Total %d devices attached\n\n", attach_count);

	buf->count++;
	buf->size += buf_obj->size;

	return 0;
err:
	return ret;
}

static int dma_buf_bufinfo_show(struct seq_file *s, void *unused)
{
	struct dma_buf_priv dma_buf_priv;

	dma_buf_priv.count = 0;
	dma_buf_priv.size = 0;
	dma_buf_priv.s = s;

	osvelte_info("%s:%d read %s, seq_buf size:%zu\n",
		     current->comm, current->tgid, __func__, s->size);

	seq_puts(s, "\nDma-buf Objects:\n");
	seq_printf(s, "%-8s\t%-8s\t%-8s\t%-8s\texp_name\t%-8s\n",
		   "size", "flags", "mode", "count", "ino");

	get_each_dmabuf(dma_buf_show, &dma_buf_priv);

	seq_printf(s, "\nTotal %d objects, %zu bytes\n",
		   dma_buf_priv.count, dma_buf_priv.size);

	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE_SIZE_BUF(dma_buf_bufinfo, SZ_512K);

static int get_dma_buf_info(const void *data, struct file *file, unsigned int n)
{
	struct dma_proc *dma_proc;
	struct dma_info *dma_info;
	struct dma_buf *dmabuf;

	if (!file || !is_dma_buf_file(file))
		return 0;

	dma_proc = (struct dma_proc *)data;
	hash_for_each_possible_rcu(dma_proc->dma_bufs, dma_info, head,
			       (unsigned long) file->private_data)
		if (file->private_data == dma_info->dmabuf)
			return 0;

	dma_info = kzalloc(sizeof(*dma_info), GFP_ATOMIC);
	if (!dma_info)
		return -ENOMEM;

	dmabuf = file->private_data;

	dma_info->dmabuf = dmabuf;
	spin_lock(&dmabuf->name_lock);
	strncpy(dma_info->exp_name, dmabuf->exp_name, EXP_NAME_LEN - 1);
	if (dmabuf->name)
		strncpy(dma_info->name, dmabuf->name, NAME_LEN - 1);
	spin_unlock(&dmabuf->name_lock);

	dma_info->i_ino = file_inode(dmabuf->file)->i_ino;
	dma_info->file_count = file_count(dmabuf->file);
	dma_info->f_flags = dmabuf->file->f_flags;
	dma_info->f_mode = dmabuf->file->f_mode;
	dma_info->size = dmabuf->size;

	dma_proc->size += dmabuf->size;
	hash_add(dma_proc->dma_bufs, &dma_info->head,
		 (unsigned long)dma_info->dmabuf);
	return 0;
}

static void free_dma_proc(struct dma_proc *proc)
{
	struct dma_info *tmp;
	struct hlist_node *n;
	int i;

	hash_for_each_safe(proc->dma_bufs, i, n, tmp, head) {
		hash_del(&tmp->head);
		kfree(tmp);
	}
	kfree(proc);
}

static void dma_proc_show(struct seq_file *s, struct dma_proc *proc)
{
	struct dma_info *tmp;
	int i;

	seq_printf(s, "\n%s (PID %d) size: %ld kB\nDMA Buffers:\n",
		   proc->name, proc->pid, proc->size / SZ_1K);
	seq_printf(s, "%-8s\t%-8s\t%-8s\t%-10s\t%-10s\t%-s\n",
		   "ino", "size", "count", "flags", "mode", "exp_name");

	hash_for_each(proc->dma_bufs, i, tmp, head) {
		seq_printf(s, "%08lu\t%-8zu\t%-8ld\t0x%08x\t0x%08x\t%-s\t%-s\n",
			   tmp->i_ino,
			   tmp->size / SZ_1K,
			   tmp->file_count,
			   tmp->f_flags, tmp->f_mode,
			   tmp->exp_name,
			   tmp->name);
	}
}

static int dma_buf_procinfo_show(struct seq_file *s, void *unused)
{
	struct task_struct *task, *thread;
	struct files_struct *files;
	int ret = 0;
	struct dma_proc *tmp, *n;
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
		hash_init(tmp->dma_bufs);
		for_each_thread(task, thread) {
			task_lock(thread);
			if (unlikely(!group_leader_files))
				group_leader_files = task->group_leader->files;
			files = thread->files;
			if (files && (group_leader_files != files ||
				      thread == task->group_leader))
				ret = iterate_fd(files, 0, get_dma_buf_info, tmp);
			task_unlock(thread);
		}

		if (ret || hash_empty(tmp->dma_bufs))
			goto skip;

		get_task_comm(tmp->name, task);
		tmp->pid = task->tgid;
		list_add(&tmp->head, &plist);
		continue;
skip:
		free_dma_proc(tmp);
	}
	rcu_read_unlock();

	list_for_each_entry(tmp, &plist, head)
		dma_proc_show(s, tmp);

	ret = 0;
mem_err:
	list_for_each_entry_safe(tmp, n, &plist, head) {
		list_del(&tmp->head);
		free_dma_proc(tmp);
	}
	return ret;
}
DEFINE_PROC_SHOW_ATTRIBUTE_SIZE_BUF(dma_buf_procinfo, SZ_256K);

void create_dmabuf_procfs(struct proc_dir_entry *root)
{
	OSVELTE_STATIC_ASSERT(sizeof(struct dma_info) <= 128);

	proc_create("procinfo", 0444, root, &dma_buf_procinfo_proc_ops);
	proc_create("bufinfo", 0444, root, &dma_buf_bufinfo_proc_ops);
}
