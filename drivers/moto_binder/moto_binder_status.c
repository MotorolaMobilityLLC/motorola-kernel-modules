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

#define ASYNC_REPORT_DISABLE 1


struct hlist_head *binder_procs = NULL;
struct mutex *binder_procs_lock = NULL;

/**
 * binder_inner_proc_lock() - Acquire inner lock for given binder_proc
 * @proc: struct binder_proc to acquire
 *
 * Acquires proc->inner_lock. Used to protect todo lists
 */
#define binder_inner_proc_lock(proc) _binder_inner_proc_lock(proc, __LINE__)
static void
_binder_inner_proc_lock(struct binder_proc *proc, int line)
	__acquires(&proc->inner_lock)
{
	spin_lock(&proc->inner_lock);
}

/**
 * binder_inner_proc_unlock() - Release inner lock for given binder_proc
 * @proc: struct binder_proc to acquire
 *
 * Release lock acquired via binder_inner_proc_lock()
 */
#define binder_inner_proc_unlock(proc) _binder_inner_proc_unlock(proc, __LINE__)
static void
_binder_inner_proc_unlock(struct binder_proc *proc, int line)
	__releases(&proc->inner_lock)
{
	spin_unlock(&proc->inner_lock);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static long __copy_from_user_nofault(void *dst, const void __user *src, size_t size)
{
	long ret = -EFAULT;
	if (access_ok(src, size)) {
		pagefault_disable();
		ret = __copy_from_user_inatomic(dst, src, size);
		pagefault_enable();
	}
	if (ret)
		return -EFAULT;
	return 0;
}
#endif

static long copy_from_user_compatible(void *dst, const void __user *src, size_t size)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return __copy_from_user_nofault(dst, src, size);
#else
	return copy_from_user(dst, src, size);
#endif
}

void binder_trans_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc,
				struct binder_thread *thread,
				struct binder_transaction_data *tr)
{
	char buf_data[INTERFACETOKEN_BUFF_SIZE];
	size_t buf_data_size;
	char buf[INTERFACETOKEN_BUFF_SIZE] = {0};
	int i = 0;
	int j = 0;

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

	if ((tr->flags & TF_ONE_WAY) /*report async binder call*/
		&& target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val > MIN_USERAPP_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		buf_data_size = tr->data_size>INTERFACETOKEN_BUFF_SIZE ?INTERFACETOKEN_BUFF_SIZE:tr->data_size;
		if (!copy_from_user_compatible(buf_data, (char*)tr->data.ptr.buffer, buf_data_size)) {
			/*1.skip first PARCEL_OFFSET bytes (useless data)
			  2.make sure the invalid address issue is not occuring(j =PARCEL_OFFSET+1, j+=2)
			  3.java layer uses 2 bytes char. And only the first bytes has the data.(p+=2)*/
			if (buf_data_size > PARCEL_OFFSET) {
				char *p = (char *)(buf_data) + PARCEL_OFFSET;
				j = PARCEL_OFFSET + 1;
				while (i < INTERFACETOKEN_BUFF_SIZE && j < buf_data_size && *p != '\0') {
					buf[i++] = *p;
					j += 2;
					p += 2;
				}
				if (i == INTERFACETOKEN_BUFF_SIZE) buf[i-1] = '\0';
			}
			moto_binder_write_status(ASYNC_BINDER, task_uid(proc->tsk).val, task_tgid_nr(proc->tsk), task_uid(target_proc->tsk).val, task_tgid_nr(target_proc->tsk), 0, tr->code);
		}
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
void binder_alloc_handler(void *data, size_t size, size_t *free_async_space, int is_async)
{
	struct task_struct *p = NULL;
	struct binder_alloc *alloc = NULL;

	alloc = container_of(free_async_space, struct binder_alloc, free_async_space);
	if (alloc == NULL) {
		return;
	}

	/*
	* async buffer free space bigger 1/3 buffer or buffer free lower 100K
	*/
	if (is_async
		&& (alloc->free_async_space < 3 * (size + sizeof(struct binder_buffer))
		|| (alloc->free_async_space < 100*1024))) {
		rcu_read_lock();
		p = find_task_by_vpid(alloc->pid);
		rcu_read_unlock();
		if (p != NULL && is_frozen_tg(p)) {
			moto_binder_write_status(ASYNC_ALLOC_FULL, task_uid(current).val, task_tgid_nr(current), task_uid(p).val, task_tgid_nr(p), 0, 0);
		}
	}
}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
void binder_alloc_handler(void *data, size_t size, struct binder_alloc *alloc, int is_async)
{
	struct task_struct *p = NULL;

	/*
	* async buffer free space bigger 1/3 buffer or buffer free lower 100K
	*/
	if (is_async
		&& (alloc->free_async_space < 3 * (size + sizeof(struct binder_buffer))
		|| (alloc->free_async_space < 100*1024))) {
		rcu_read_lock();
		p = find_task_by_vpid(alloc->pid);
		rcu_read_unlock();
		if (p != NULL && is_frozen_tg(p)) {
			moto_binder_write_status(ASYNC_ALLOC_FULL, task_uid(current).val, task_tgid_nr(current), task_uid(p).val, task_tgid_nr(p), 0, 0);
		}
	}
}
#endif

void binder_preset_handler(void *data, struct hlist_head *hhead, struct mutex *lock)
{
	if (binder_procs == NULL)
		binder_procs = hhead;

	if (binder_procs_lock == NULL)
		binder_procs_lock = lock;
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

#if ASYNC_REPORT_DISABLE
	if (call_type == ASYNC_BINDER)
		return;
#endif

	moto_binder_send_uevent(call_type, caller_uid, caller_pid, target_uid, target_pid, arg1, arg2);
}

static char* binder_type_to_string(int type)
{
	switch (type)
	{
	case CALL_BINDER:
		return "call";

	case ASYNC_BINDER:
		return "async";

	case REPLY_BINDER:
		return "reply";

	case ASYNC_ALLOC_FULL:
		return "async_buf_full";

	default:
		return "unknown";
	}
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
		   e->seq_id, binder_type_to_string(e->call_type),
		   e->caller_uid, e->caller_pid, e->target_uid, e->target_pid,
		   e->arg1, e->arg2);

	}
	return 0;
}

static void get_uid_pid(int *from_pid, int *from_uid, int *to_pid, int *to_uid, struct binder_transaction *tr)
{
	/* from pid/uid */
	*from_pid = -1;
	*from_uid = tr->sender_euid.val;
	if (tr->from != NULL && tr->from->task != NULL) {
		*from_pid = task_tgid_nr(tr->from->task);
	} else if (tr->from != NULL &&
		   tr->from->proc != NULL &&
		   tr->from->proc->tsk != NULL) {
		*from_pid = task_tgid_nr(tr->from->proc->tsk);
	} else if (tr->from != NULL && tr->from->proc != NULL) {
		*from_pid = tr->from->proc->pid;
	} else if (tr->from != NULL) {
		*from_pid = tr->from->pid;
	}

	/* to thread pid/uid */
	*to_pid = -1;
	if (tr->to_thread != NULL && tr->to_thread->task != NULL) {
		*to_pid = task_tgid_nr(tr->to_thread->task);
	} else if (tr->to_thread != NULL &&
		   tr->to_thread->proc != NULL &&
		   tr->to_thread->proc->tsk != NULL) {
		*to_pid = task_tgid_nr(tr->to_thread->proc->tsk);
	} else if (tr->to_proc != NULL && tr->to_proc->tsk != NULL) {
		*to_pid = task_tgid_nr(tr->to_proc->tsk);
	}
}

static bool binder_worklist_empty_ilocked(struct list_head *list)
{
	return list_empty(list);
}

void proc_binder_check_status_print(struct seq_file *m, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2)
{
	seq_printf(m,
		"from %d:%d to %d:%d %d %d\n",
		caller_uid, caller_pid, target_uid, target_pid,
		arg1, arg2);
}

void proc_binder_check_status_show_detail(struct seq_file *m, struct binder_proc *proc, int to_uid)
{
	struct rb_node *n = NULL;
	struct binder_thread *thread = NULL;
	int from_uid = -1;
	int from_pid = -1;
	int to_pid = -1;
	struct binder_transaction *btrans = NULL;
	bool empty = true;
	int need_reply = -1;
	struct binder_work *w = NULL;

	/*check binder_thread/transaction_stack/binder_proc ongoing transaction*/
	binder_inner_proc_lock(proc);
	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		empty = binder_worklist_empty_ilocked(&thread->todo);

		if (thread->task != NULL) {
			/*has "todo" binder thread in worklist?*/
			if (!empty) {
				/*scan thread->todo list*/
				list_for_each_entry(w, &thread->todo, entry) {
					btrans = container_of(w, struct binder_transaction, work);
					if (w != NULL && w->type == BINDER_WORK_TRANSACTION && btrans != NULL) {
						spin_lock(&btrans->lock);
						get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
						need_reply = (int)(!(btrans->flags & TF_ONE_WAY));
						if (btrans->to_thread != NULL && btrans->to_thread == thread && (!(btrans->flags & TF_ONE_WAY))) {
							spin_unlock(&btrans->lock);
							if (from_uid != to_uid) {
								binder_inner_proc_unlock(proc);
								proc_binder_check_status_print(m, from_uid, from_pid, to_uid, to_pid, 0, need_reply);
								return;
							}
						} else {
							spin_unlock(&btrans->lock);
						}
					} else if (w != NULL && w->type != BINDER_WORK_TRANSACTION_COMPLETE && w->type != BINDER_WORK_NODE) {
						/* under binder error/dead status, to_proc could be null, user "-1" instead uid/pid */
						binder_inner_proc_unlock(proc);
						proc_binder_check_status_print(m, -1, -1, to_uid, -1, 1, 1);
						return;
					}
				}
			}

			/*has transcation in transaction_stack?*/
			btrans = thread->transaction_stack;
			if (btrans) {
				spin_lock(&btrans->lock);
				if (btrans->to_thread != NULL && btrans->to_thread == thread) {
					/*only report incoming binder call*/
					need_reply = (int)(!(btrans->flags & TF_ONE_WAY));
					get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
					spin_unlock(&btrans->lock);
					if (from_uid != to_uid) {
						binder_inner_proc_unlock(proc);
						proc_binder_check_status_print(m, from_uid, from_pid, to_uid, to_pid, 2, need_reply);
						return;
					}
				} else {
					spin_unlock(&btrans->lock);
				}
			}
		}
	}

	/*has "todo" binder proc in worklist*/
	empty = binder_worklist_empty_ilocked(&proc->todo);
	if (proc->tsk != NULL && !empty) {
		list_for_each_entry(w, &proc->todo, entry) {
			btrans = container_of(w, struct binder_transaction, work);
			if (w != NULL && w->type == BINDER_WORK_TRANSACTION && btrans != NULL) {
				spin_lock(&btrans->lock);
				get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
				need_reply = (int)(!(btrans->flags & TF_ONE_WAY));
				if (btrans->to_thread != NULL && btrans->to_thread == thread && (!(btrans->flags & TF_ONE_WAY))) {
					spin_unlock(&btrans->lock);
					if (from_uid != to_uid) {
						binder_inner_proc_unlock(proc);
						proc_binder_check_status_print(m, from_uid, from_pid, to_uid, to_pid, 3, need_reply);
						return;
					}
				} else {
					spin_unlock(&btrans->lock);
				}
			} else if (w != NULL && w->type != BINDER_WORK_TRANSACTION_COMPLETE && w->type != BINDER_WORK_NODE) {
				/* under binder error/dead status, to_proc could be null, user "-1" instead uid/pid */
				binder_inner_proc_unlock(proc);
				proc_binder_check_status_print(m, -1, -1, to_uid, -1, 4, 1);
				return;
			}
		}
	}
	binder_inner_proc_unlock(proc);
}

int proc_binder_check_status_show(struct seq_file *m, void *unused)
{
	struct binder_proc *proc;

	if (moto_binder_check_uid <= MIN_USERAPP_UID)
		return 0;

	mutex_lock(binder_procs_lock);
	hlist_for_each_entry(proc, binder_procs, proc_node) {
		if (proc != NULL && (proc->tsk != NULL) && (task_uid(proc->tsk).val == moto_binder_check_uid)) {
			proc_binder_check_status_show_detail(m, proc, moto_binder_check_uid);
		}
	}
	mutex_unlock(binder_procs_lock);

	return 0;
}

int register_moto_binder_hooks(void)
{
	int rc = 0;

	rc = register_trace_android_vh_binder_preset(binder_preset_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_preset failed, rc=%d\n", rc);
		return rc;
	}

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

	rc = register_trace_android_vh_binder_alloc_new_buf_locked(binder_alloc_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_alloc_new_buf_locked failed, rc=%d\n", rc);
		return rc;
	}
	return rc;
}

void unregister_moto_binder_hooks(void)
{
	unregister_trace_android_vh_binder_preset(binder_preset_handler, NULL);
	unregister_trace_android_vh_binder_trans(binder_trans_handler, NULL);
	unregister_trace_android_vh_binder_reply(binder_reply_handler, NULL);
	unregister_trace_android_vh_binder_alloc_new_buf_locked(binder_alloc_handler, NULL);
}

int moto_binder_status_init(void)
{
	int ret;

	ret = register_moto_binder_hooks();
	if (ret != 0) {
		unregister_moto_binder_hooks();
		return ret;
	}

	pr_info("moto binder status init succeed!\n");

	return 0;
}

void moto_binder_status_exit(void)
{
	unregister_moto_binder_hooks();

	pr_info("moto binder status exit succeed!\n");
}
