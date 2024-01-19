// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/rwsem.h>
#include <trace/hooks/dtask.h>

#include "../msched_common.h"
#include "locking_main.h"

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

#define rwsem_first_waiter(sem) \
	list_first_entry(&sem->wait_list, struct rwsem_waiter, list)

static inline struct task_struct *rwsem_owner(struct rw_semaphore *sem)
{
	return (struct task_struct *)
		(atomic_long_read(&sem->owner) & ~RWSEM_OWNER_FLAGS_MASK);
}

bool rwsem_list_add(struct task_struct *tsk, struct list_head *entry, struct list_head *head, int owner_pid)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	int index=0;

	if (!entry || !head) {
		printk(KERN_ERR "rwsem_list_add %p %p is NULL", entry, head);
		return false;
	}

	if (task_is_important_ux(current)) {
		list_for_each_safe(pos, n, head) {
			waiter = list_entry(pos, struct rwsem_waiter, list);
			if (waiter && waiter->task->prio > MAX_RT_PRIO && !task_is_important_ux(waiter->task)) {
				cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
					"rwsem_list_add %d -> %d index=%d\n", tsk->pid, owner_pid, index);
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

static void android_vh_alter_rwsem_list_add_handler(void *unused, struct rwsem_waiter *waiter,
			struct rw_semaphore *sem, bool *already_on_list)
{
	struct task_struct *ts = rwsem_owner(sem);
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE) || !ts))
		return;

	*already_on_list = rwsem_list_add(waiter->task, &waiter->list, &sem->wait_list, ts->pid);
}

static void android_vh_rwsem_wake_handler(void *unused, struct rw_semaphore *sem)
{
	struct task_struct *owner_ts = NULL;

	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE) || !sem)) {
		return;
	}

	if (!current_is_important_ux() || (atomic_long_read(&sem->owner) & RWSEM_READER_OWNED)) {
		return;
	}

	owner_ts = rwsem_owner(sem);
	if (!owner_ts) {
		return;
	}

	lock_inherit_ux_type(owner_ts, current, "rwsem_wake");
}

static void android_vh_rwsem_wake_finish_handler(void *unused, struct rw_semaphore *sem)
{
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE))) {
		return;
	}

	lock_clear_inherited_ux_type(current, "rwsem_wake_finish");
}

void register_rwsem_vendor_hooks(void)
{
	register_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
	register_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	register_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);

}

void unregister_rwsem_vendor_hooks(void)
{
	unregister_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
	unregister_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	unregister_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
}

