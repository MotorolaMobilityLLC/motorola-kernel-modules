/*
 * Copyright (C) 2023 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "moto_binder: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include "moto_binder.h"


static bool moto_binder_status_initialized = false;

/**
 * binder_inner_proc_lock() - Acquire inner lock for given binder_proc
 * @proc: struct binder_proc to acquire
 *
 * Acquires proc->inner_lock. Used to protect todo lists
 */
// #define binder_inner_proc_lock(proc) _binder_inner_proc_lock(proc, __LINE__)
// static void
// _binder_inner_proc_lock(struct binder_proc *proc, int line)
// 	__acquires(&proc->inner_lock)
// {
// 	spin_lock(&proc->inner_lock);
// }

/**
 * binder_inner_proc_unlock() - Release inner lock for given binder_proc
 * @proc: struct binder_proc to acquire
 *
 * Release lock acquired via binder_inner_proc_lock()
 */
// #define binder_inner_proc_unlock(proc) _binder_inner_proc_unlock(proc, __LINE__)
// static void
// _binder_inner_proc_unlock(struct binder_proc *proc, int line)
// 	__releases(&proc->inner_lock)
// {
// 	spin_unlock(&proc->inner_lock);
// }

void binder_trans_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc,
				struct binder_thread *thread,
				struct binder_transaction_data *tr)
{
	/*report sync binder call*/
	if (!(tr->flags & TF_ONE_WAY)
		&& target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val > MIN_USERAPP_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		moto_binder_write_status(CALL_BINDER, task_uid(proc->tsk).val, task_tgid_nr(proc->tsk), task_uid(target_proc->tsk).val, task_tgid_nr(target_proc->tsk), 0, 0);
	}
}

void binder_reply_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc,
				struct binder_thread *thread,
				struct binder_transaction_data *tr)
{
	/*only sync binder call has BC_REPLY*/
	if (target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val <= MAX_SYSTEM_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		moto_binder_write_status(REPLY_BINDER, task_uid(proc->tsk).val, task_tgid_nr(proc->tsk), task_uid(target_proc->tsk).val, task_tgid_nr(target_proc->tsk), 0, 0);
	}
}

struct moto_binder_transaction_log_entry {
	int seq_id;
	int call_type;
	int caller_uid;
	int caller_pid;
	int target_uid;
	int target_pid;
	int arg1;
	int arg2;
};

struct moto_binder_transaction_log {
	atomic_t cur;
	bool full;
	struct moto_binder_transaction_log_entry entry[32];
};

static atomic_t moto_binder_last_id;
static struct moto_binder_transaction_log moto_binder_transaction_log;

static struct moto_binder_transaction_log_entry *moto_binder_transaction_log_add(
	struct moto_binder_transaction_log *log)
{
	struct moto_binder_transaction_log_entry *e;
	unsigned int cur = atomic_inc_return(&log->cur);

	if (cur >= ARRAY_SIZE(log->entry))
		log->full = true;
	e = &log->entry[cur % ARRAY_SIZE(log->entry)];
	memset(e, 0, sizeof(*e));
	return e;
}

void moto_binder_write_status(int call_type, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2)
{
	struct moto_binder_transaction_log_entry *e;
	int seq_id = atomic_inc_return(&moto_binder_last_id);

	e = moto_binder_transaction_log_add(&moto_binder_transaction_log);
	e->seq_id = seq_id;
	e->call_type = call_type;
	e->caller_uid = caller_uid;
	e->caller_pid = caller_pid;
	e->target_uid = target_uid;
	e->target_pid = target_pid;
	e->arg1 = arg1;
	e->arg2 = arg2;

	moto_binder_send_uevent(call_type, caller_uid, caller_pid, target_uid, target_pid, arg1, arg2);
}

int proc_binder_status_show(struct seq_file *m, void *unused)
{
	struct moto_binder_transaction_log *log = &moto_binder_transaction_log;
	unsigned int log_cur = atomic_read(&log->cur);
	unsigned int count;
	unsigned int cur;
	int i;

	count = log_cur + 1;
	cur = count < ARRAY_SIZE(log->entry) && !log->full ?
		0 : count % ARRAY_SIZE(log->entry);
	if (count > ARRAY_SIZE(log->entry) || log->full)
		count = ARRAY_SIZE(log->entry);
	for (i = 0; i < count; i++) {
		unsigned int index = cur++ % ARRAY_SIZE(log->entry);
		struct moto_binder_transaction_log_entry *e = &log->entry[index];

		seq_printf(m,
		   "%d: %s from %d:%d to %d:%d %d %d\n",
		   e->seq_id, (e->call_type == REPLY_BINDER) ? "reply" :
		   ((e->call_type == ASYNC_BINDER) ? "async" : "call "),
		   e->caller_uid, e->caller_pid, e->target_uid, e->target_pid,
		   e->arg1, e->arg2);

	}
	return 0;
}

int register_moto_binder_hooks(void)
{
	int rc = 0;

	rc = register_trace_android_vh_binder_trans(binder_trans_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_trans failed,  rc=%d\n", rc);
		return rc;
	}

	rc = register_trace_android_vh_binder_reply(binder_reply_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_reply failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

void unregister_moto_binder_hooks(void)
{
	unregister_trace_android_vh_binder_trans(binder_trans_handler, NULL);
	unregister_trace_android_vh_binder_reply(binder_reply_handler, NULL);
}

int moto_binder_status_init(void)
{
	int ret;

	if (moto_binder_status_initialized) return 0;

	ret = register_moto_binder_hooks();
	if (ret != 0) {
		unregister_moto_binder_hooks();
		return ret;
	}

	moto_binder_status_initialized = true;
	pr_info("moto_binder_status_init succeed!\n");

	return 0;
}

void moto_binder_status_exit(void)
{
	if (!moto_binder_status_initialized) return;

	unregister_moto_binder_hooks();

	moto_binder_status_initialized = false;
	pr_info("moto_binder_status_exit succeed!\n");
}