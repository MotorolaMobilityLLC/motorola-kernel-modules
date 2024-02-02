// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#if defined CONFIG_KERNEL_6_1
#include <linux/sched/cputime.h>
#endif
#include <linux/sched/task.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/rwsem.h>
#include <trace/hooks/dtask.h>
#include <linux/stacktrace.h>

#include "../msched_common.h"
#include "locking_main.h"

#define ENABLE_REORDER_LIST 1

#define RWSEM_READER_OWNED	(1UL << 0)
#define RWSEM_RD_NONSPINNABLE	(1UL << 1)
#define RWSEM_WR_NONSPINNABLE	(1UL << 2)
#define RWSEM_NONSPINNABLE	(RWSEM_RD_NONSPINNABLE | RWSEM_WR_NONSPINNABLE)
#define RWSEM_OWNER_FLAGS_MASK	(RWSEM_READER_OWNED | RWSEM_NONSPINNABLE)

#define RWSEM_WRITER_LOCKED	(1UL << 0)
#define RWSEM_WRITER_MASK	RWSEM_WRITER_LOCKED

/*
 * Note:
 * The following macros must be the same as in kernel/locking/rwsem.c
 */
#define RWSEM_FLAG_WAITERS	(1UL << 1)
#define RWSEM_FLAG_HANDOFF	(1UL << 2)

#if defined CONFIG_KERNEL_6_1
enum rwsem_waiter_type {
	RWSEM_WAITING_FOR_WRITE,
	RWSEM_WAITING_FOR_READ
};

struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	enum rwsem_waiter_type type;
	unsigned long timeout;
	bool handoff_set;
};
#endif

#define rwsem_first_waiter(sem) \
	list_first_entry(&sem->wait_list, struct rwsem_waiter, list)

static inline struct task_struct *rwsem_owner(struct rw_semaphore *sem)
{
	return (struct task_struct *)
		(atomic_long_read(&sem->owner) & ~RWSEM_OWNER_FLAGS_MASK);
}

static inline bool rwsem_test_oflags(struct rw_semaphore *sem, long flags)
{
	return atomic_long_read(&sem->owner) & flags;
}

static inline bool is_rwsem_reader_owned(struct rw_semaphore *sem)
{
#if IS_ENABLED(CONFIG_DEBUG_RWSEMS)
	/*
	 * Check the count to see if it is write-locked.
	 */
	long count = atomic_long_read(&sem->count);

	if (count & RWSEM_WRITER_MASK)
		return false;
#endif
	return rwsem_test_oflags(sem, RWSEM_READER_OWNED);
}

#ifdef ENABLE_REORDER_LIST
bool rwsem_list_add(struct task_struct *tsk, struct list_head *entry, struct list_head *head, int owner_pid)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	int index = 0;
	int prio = 0;

	if (!entry || !head) {
		printk(KERN_ERR "rwsem_list_add %p %p is NULL", entry, head);
		return false;
	}
	prio = task_get_mvp_prio(current, true);

	if (prio > 0) {
		list_for_each_safe(pos, n, head) {
			waiter = list_entry(pos, struct rwsem_waiter, list);
			if (waiter && waiter->task->prio > MAX_RT_PRIO && prio > task_get_mvp_prio(waiter->task, true)) {
				cond_trace_printk(moto_sched_debug,
					"rwsem_list_add %d -> %d prio=%d(%d)index=%d\n", tsk->pid, owner_pid, prio, task_get_mvp_prio(waiter->task, true), index);
				list_add(entry, waiter->list.prev);
				return true;
			}
			index +=1;
		}

		if (pos == head) {
			list_add_tail(entry, head);
		}
		return true;
	}

	return false;
}

/* timeout is 5s */
#define WAIT_TIMEOUT		5000

inline bool test_wait_timeout(struct rw_semaphore *sem)
{
	struct rwsem_waiter *waiter;
	unsigned long timeout;
	struct task_struct *task;
	long count;
	bool ret = false;

	if (!sem)
		return false;

	count = atomic_long_read(&sem->count);
	if (!(count & RWSEM_FLAG_WAITERS))
		return false;

	waiter = rwsem_first_waiter(sem);
	if (!waiter)
		return false;

	timeout = waiter->timeout;
	task = waiter->task;
	if (!task)
		return false;

	ret = time_is_before_jiffies(timeout + msecs_to_jiffies(WAIT_TIMEOUT));
	if (ret) {
		cond_trace_printk(moto_sched_debug,
			"rwsem wait timeout [%s$%d]: task=%s, pid=%d, tgid=%d, prio=%d, ux=%d, timeout=%lu(0x%lx), t_m=%lu(0x%lx), jiffies=%lu(0x%lx)\n",
			__func__, __LINE__,
			task->comm, task->pid, task->tgid, task->prio, task_get_ux_type(task),
			timeout, timeout,
			timeout + msecs_to_jiffies(WAIT_TIMEOUT), timeout + msecs_to_jiffies(WAIT_TIMEOUT),
			jiffies, jiffies);
	}

	return ret;
}

static void android_vh_alter_rwsem_list_add_handler(void *unused, struct rwsem_waiter *waiter,
			struct rw_semaphore *sem, bool *already_on_list)
{
	struct task_struct *ts;
	bool ret = false;
	if (!waiter || !sem)
		return;

	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return;

	ts = rwsem_owner(sem);

	if (!ts || waiter->type == RWSEM_WAITING_FOR_READ)
		return;

	if (test_wait_timeout(sem))
		return;

	ret = rwsem_list_add(waiter->task, &waiter->list, &sem->wait_list, ts->pid);

	if (ret)
		*already_on_list = true;
}
#endif

static void android_vh_rwsem_wake_handler(void *unused, struct rw_semaphore *sem)
{
	struct task_struct *owner_ts = NULL;
	long owner = atomic_long_read(&sem->owner);
	bool boost = false;
#ifdef DEBUG_LOCK
	struct moto_task_struct *waiter_wts = (struct moto_task_struct *) current->android_oem_data1;
#endif

	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE) || !sem)) {
		return;
	}

	if (!current_is_important_ux() && (current->prio > 100)) {
		return;
	}

#ifdef DEBUG_LOCK
	waiter_wts->wait_start = jiffies_to_nsecs(jiffies);
	waiter_wts->wait_prio = task_get_mvp_prio(current, true);
#endif

	if (is_rwsem_reader_owned(sem)) {
		cond_trace_printk(moto_sched_debug,
			"is_rwsem_reader_owned, ignore! owner=%lx count=%lx\n", atomic_long_read(&sem->owner),
			atomic_long_read(&sem->count));
		return;
	}

	owner_ts = rwsem_owner(sem);
	if (!owner_ts) {
		cond_trace_printk(moto_sched_debug,
			"rwsem can't find owner=%lx count=%lx\n", atomic_long_read(&sem->owner),
			atomic_long_read(&sem->count));
		return;
	}

	get_task_struct(owner_ts);
	boost = lock_inherit_ux_type(owner_ts, current, "rwsem_wake");

	if (boost && (atomic_long_read(&sem->owner) != owner || is_rwsem_reader_owned(sem))) {
		cond_trace_printk(moto_sched_debug,
			"rwsem owner status has been changed owner=%lx(%lx)\n",
			atomic_long_read(&sem->owner), owner);
		lock_clear_inherited_ux_type(owner_ts, "rwsem_wake_finish");
	}
	put_task_struct(owner_ts);
}

#ifdef DEBUG_LOCK
static void android_vh_rwsem_wait_finish_handler(void *unused, struct rw_semaphore *sem)
{
	struct moto_task_struct *waiter_wts = (struct moto_task_struct *) current->android_oem_data1;
	if (unlikely(!locking_opt_enable(LK_MUTEX_ENABLE)))
		return;

	if (waiter_wts->wait_start > 0) {
		u64 sleep = (jiffies_to_nsecs(jiffies) - waiter_wts->wait_start) / 1000000U;
		if (sleep > 50) {
			cond_trace_printk(moto_sched_debug,
					"rwsem wait too long prio=%d wait=%llu\n", waiter_wts->wait_prio, sleep);
			dump_stack();
		}
		waiter_wts->wait_start = 0;
	}
}
#endif

static void android_vh_rwsem_wake_finish_handler(void *unused, struct rw_semaphore *sem)
{
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE))) {
		return;
	}
	lock_clear_inherited_ux_type(current, "rwsem_wake_finish");
}

void register_rwsem_vendor_hooks(void)
{
#ifdef ENABLE_REORDER_LIST
	register_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif
	register_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	register_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);

#ifdef DEBUG_LOCK
	register_trace_android_vh_rwsem_read_wait_finish(android_vh_rwsem_wait_finish_handler, NULL);
	register_trace_android_vh_rwsem_write_wait_finish(android_vh_rwsem_wait_finish_handler, NULL);
#endif
}

void unregister_rwsem_vendor_hooks(void)
{
#ifdef ENABLE_REORDER_LIST
	unregister_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif
	unregister_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	unregister_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
}

