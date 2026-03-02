// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "osvelte: " fmt

#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#include <linux/cred.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/pagemap.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <linux/thread_info.h>

#include "common.h"
#include "memstat.h"
#include "proc-memstat.h"
#include "sys-memstat.h"
#include "logger.h"

struct hash_data_info {
	void *p;
	struct hlist_node head;
};

struct hash_data {
	struct hlist_head hash_buf[1 << 10];
	unsigned int inx;
	struct hash_data_info p_arr[0];
};

struct fd_data {
	bool print_oom;
	struct proc_ms *ms;
	struct hash_data *phash_data;
};

#define SIZE_TO_PAGES(size) ((size) >> PAGE_SHIFT)
#define MAX_FD (4096)
/* 4096 * 32 + 1024 * 8 + 8 = 106504 */
#define HASH_DATA_SIZE (sizeof(struct hash_data) + MAX_FD * sizeof(struct hash_data_info))

ssize_t proc_memstat_fd_data_size(void)
{
	return HASH_DATA_SIZE;
}

static int match_file(const void *p, struct file *file, unsigned fd)
{
	struct fd_data *pfd_data = (struct fd_data *)p;
	struct proc_ms *ms = pfd_data->ms;
	struct hash_data_info *hash_data_info;
	struct hash_data *hash_data = pfd_data->phash_data;

	if (unlikely(!file->private_data))
		goto unaccount;

	if (!is_ashmem_file(file) && !is_dma_buf_file(file))
		goto unaccount;

	if (unlikely(!pfd_data->phash_data))
		goto no_hash_data;

	hash_for_each_possible(hash_data->hash_buf, hash_data_info, head,
			       (unsigned long) file->private_data)
		if (file->private_data == hash_data_info->p)
			goto unaccount;

	if (unlikely(hash_data->inx == MAX_FD)) {
		if (!pfd_data->print_oom) {
			pr_err("%s out of (%d) fds\n", ms->comm, MAX_FD);
			pfd_data->print_oom = true;
		}
		goto no_hash_data;
	}

	hash_data_info = hash_data->p_arr + hash_data->inx;
	hash_data_info->p = file->private_data;
	hash_data->inx += 1;
	hash_add(hash_data->hash_buf, &hash_data_info->head,
		 (unsigned long)hash_data_info->p);

no_hash_data:
	if (is_dma_buf_file(file)) {
		struct dma_buf *dmabuf = file->private_data;

		if (dmabuf)
			ms->dmabuf += SIZE_TO_PAGES(dmabuf->size);
		return 0;
	}

#ifdef CONFIG_ASHMEM
	if (is_ashmem_file(file)) {
		struct ashmem_area *ashmem_data = file->private_data;

		if (ashmem_data->file && ashmem_data->size)
			ms->ashmem += SIZE_TO_PAGES(ashmem_data->size);
		return 0;
	}
#endif /* CONFIG_ASHMEM */
unaccount:
	ms->nr_fds += 1;
	return 0;
}

/*
 * this should be called without rcu, qcom use read_lock and mtk use mutex
 * which might sleep.
 */
static void __proc_mtrack_memstat(struct proc_ms *ms, pid_t pid, u32 flags)
{
	if (flags & PROC_MS_ITERATE_MTRACK) {
		/* gpu in page_size, so it cannot overflow */
		ms->gpu = (u32)read_pid_mtrack_mem_usage(MTRACK_GPU,
							 MTRACK_GPU_PROC_KERNEL,
							 pid);
	}
}

/*
 * Must be called under rcu_read_lock() & increment task_struct counter.
 */
static int __proc_memstat(struct task_struct *p, struct proc_ms *ms, u32 flags,
			  struct fd_data *pfd_data)
{
	struct mm_struct *mm = NULL;
	struct task_struct *tsk;

	if (flags & PROC_MS_UID)
		ms->uid = from_kuid(&init_user_ns, task_uid(p));

	if (flags & PROC_MS_PID)
		ms->pid = p->pid;

	if (flags & PROC_MS_OOM_SCORE_ADJ)
		ms->oom_score_adj = p->signal->oom_score_adj;

	if (flags & PROC_MS_32BIT)
		ms->is_32bit = test_ti_thread_flag(task_thread_info(p),
						   TIF_32BIT);

	if (flags & PROC_MS_COMM)
		strncpy(ms->comm, p->comm, sizeof(ms->comm));

	tsk = find_lock_task_mm_dup(p);
	if (!tsk)
		return -EEXIST;

	if (flags & PROC_MS_ITERATE_FD) {
		pfd_data->ms = ms;

		if (likely(pfd_data->phash_data)) {
			pfd_data->print_oom = false;
			memset(pfd_data->phash_data, 0, HASH_DATA_SIZE);
			hash_init(pfd_data->phash_data->hash_buf);
		}
		iterate_fd(p->files, 0, match_file, pfd_data);
	}

	mm = tsk->mm;

	if (flags & PROC_MS_VSS)
		ms->vss = mm->total_vm;

	if (flags & PROC_MS_ANON)
		ms->anon = get_mm_counter(mm, MM_ANONPAGES);

	if (flags & PROC_MS_FILE)
		ms->file = get_mm_counter(mm, MM_FILEPAGES);

	if (flags & PROC_MS_SHMEM)
		ms->shmem = get_mm_counter(mm, MM_SHMEMPAGES);

	if (flags & PROC_MS_SWAP)
		ms->swap = get_mm_counter(mm, MM_SWAPENTS);

	task_unlock(tsk);
	return 0;
}

static void init_fd_data(struct fd_data *p)
{
	struct hash_data *phash_data = OSVELTE_FEATURE_USE_HASHLIST ?
		vmalloc(HASH_DATA_SIZE) : NULL;

	p->phash_data = phash_data;
}

static void destroy_fd_data(struct fd_data *p)
{
	if (OSVELTE_FEATURE_USE_HASHLIST && likely(p->phash_data))
		vfree(p->phash_data);
}

static int proc_pid_memstat(unsigned long arg)
{
	long ret = 0;
	struct proc_pid_ms ppm;
	struct task_struct *p;
	pid_t pid;
	void __user *argp = (void __user *) arg;
	struct fd_data fd_data;
	bool iter_fd;

	if (copy_from_user(&ppm, argp, sizeof(ppm)))
		return -EFAULT;

	pid = ppm.pid;
	/* zeroed data */
	memset(&ppm.ms, 0, sizeof(ppm.ms));
	iter_fd = !!(ppm.flags & PROC_MS_ITERATE_FD);

	if (iter_fd)
		init_fd_data(&fd_data);

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		ret = -EINVAL;
		goto free_fd_data;
	}

	if ((ppm.flags & PROC_MS_PPID) && pid_alive(p))
		ppm.ms.ppid = task_pid_nr(rcu_dereference(p->real_parent));
	ret = __proc_memstat(p, &ppm.ms, ppm.flags, &fd_data);
	rcu_read_unlock();

	if (ret)
		goto free_fd_data;

	__proc_mtrack_memstat(&ppm.ms, pid, ppm.flags);
	if (copy_to_user(argp, &ppm, sizeof(ppm)))
		ret = -EFAULT;

free_fd_data:
	if (iter_fd)
		destroy_fd_data(&fd_data);
	return ret;
}

static int proc_size_memstat(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct logger_reader *reader = file->private_data;
	struct proc_size_ms psm;
	struct task_struct *p = NULL;
	int ret = 0, i, cnt = 0;
	bool iter_fd;
	struct fd_data fd_data;

	void __user *argp = (void __user *) arg;

	if (copy_from_user(&psm, argp, sizeof(psm)))
		return -EFAULT;

	if (psm.size > PROC_MS_MAX_SIZE)
		return -EINVAL;

	if (unlikely(!reader->arr_ms)) {
		reader->arr_ms = vmalloc(PROC_MS_MAX_SIZE * sizeof(struct proc_ms));

		if (!reader->arr_ms)
			return -ENOMEM;
	}
	memset(reader->arr_ms, 0, PROC_MS_MAX_SIZE * sizeof(struct proc_ms));

	iter_fd = !!(psm.flags & PROC_MS_ITERATE_FD);
	if (iter_fd)
		init_fd_data(&fd_data);

	rcu_read_lock();
	for_each_process(p) {
		struct proc_ms *ms = reader->arr_ms + cnt;

		if (cnt >= psm.size)
			break;

		if (p->flags & PF_KTHREAD)
			continue;

		if (p->pid != p->tgid)
			continue;

		if (cmd == CMD_PROC_MS_SIZE_UID) {
			/* don't need fetch uid again */
			psm.flags &= ~PROC_MS_UID;
			ms->uid = from_kuid(&init_user_ns, task_uid(p));
			if (ms->uid != psm.uid)
				continue;
		}

		if ((psm.flags & PROC_MS_PPID) && pid_alive(p))
			ms->ppid = task_pid_nr(rcu_dereference(p->real_parent));

		if (likely(!__proc_memstat(p, ms, psm.flags, &fd_data)))
			cnt++;
	}
	rcu_read_unlock();

	psm.size = cnt;
	if (copy_to_user(argp, &psm, sizeof(psm))) {
		ret = -EFAULT;
		goto err_buf;
	}

	for (i = 0; i < cnt; i++) {
		struct proc_ms *ms = reader->arr_ms + i;

		__proc_mtrack_memstat(ms, ms->pid, psm.flags);
	}

	/* if cnt is zero, copy nothin. */
	if (copy_to_user(argp + sizeof(psm), reader->arr_ms, cnt * sizeof(struct proc_ms))) {
		ret = -EFAULT;
		goto err_buf;
	}
err_buf:
	if (iter_fd)
		destroy_fd_data(&fd_data);
	return ret;
}

long proc_memstat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct logger_reader *reader = file->private_data;
	long ret = -EINVAL;

	if (cmd < CMD_PROC_MS_MIN || cmd > CMD_PROC_MS_MAX) {
		osvelte_err("cmd invalid.\n");
		return CMD_PROC_MS_INVALID;
	}

	if (!(file->f_mode & FMODE_READ))
		return -EBADF;

	mutex_lock(&reader->mutex);
	switch (cmd) {
	case CMD_PROC_MS_PID:
		ret = proc_pid_memstat(arg);
		break;
	case CMD_PROC_MS_SIZE:
		ret = proc_size_memstat(file, cmd, arg);
		break;
	case CMD_PROC_MS_SIZE_UID:
		ret = proc_size_memstat(file, cmd, arg);
		break;
	}
	mutex_unlock(&reader->mutex);

	return ret;
}
