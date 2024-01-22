// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#include <linux/mutex.h>
#include <linux/sched/task.h>
#include <linux/ww_mutex.h>
#include <linux/version.h>
#include <trace/hooks/dtask.h>

#include "../msched_common.h"
#include "locking_main.h"

/*
 * Note:
 * The following structure must be the same as it's definition in kernel/locking/mutex.h
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
struct mutex_waiter {
	struct list_head	list;
	struct task_struct	*task;
	struct ww_acquire_ctx	*ww_ctx;
#ifdef CONFIG_DEBUG_MUTEXES
	void			*magic;
#endif
};
#endif

#define MUTEX_FLAGS		0x07
static inline struct task_struct *__mutex_owner(struct mutex *lock)
{
	return (struct task_struct *)(atomic_long_read(&lock->owner) & ~MUTEX_FLAGS);
}

static void mutex_list_add_ux(struct list_head *entry, struct list_head *head)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct mutex_waiter *waiter = NULL;
	int index = 0;
	int prio = task_get_mvp_prio(current, true);

	list_for_each_safe(pos, n, head) {
		waiter = list_entry(pos, struct mutex_waiter, list);
		if (waiter && waiter->task && (waiter->task->prio > MAX_RT_PRIO) && prio > task_get_mvp_prio(waiter->task, true)) {
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
					"mutex_list_add_ux %d  prio=%d(%d)index=%d\n", current->pid, prio, task_get_mvp_prio(waiter->task, true), index);
			list_add(entry, waiter->list.prev);
			return;
		}
		index += 1;
	}
	if (pos == head)
		list_add_tail(entry, head);
}

static bool mutex_list_add(struct task_struct *task, struct list_head *entry, struct list_head *head, struct mutex *lock)
{
	bool is_ux = task_is_important_ux(task);

	if (!entry || !head || !lock)
		return false;

	if (is_ux) {
		mutex_list_add_ux(entry, head);
		return true;
	}

	return false;
}

/* implement vender hook in kernel/locking/mutex.c */
static void android_vh_alter_mutex_list_add_handler(void *unused, struct mutex *lock,
			struct mutex_waiter *waiter, struct list_head *list, bool *already_on_list)
{
	if (unlikely(!locking_opt_enable(LK_MUTEX_ENABLE)))
		return;

	*already_on_list = mutex_list_add(current, &waiter->list, list, lock);
}

static void android_vh_mutex_wait_start_handler(void *unused, struct mutex *lock)
{
	struct task_struct *owner_ts = NULL;

	if (unlikely(!locking_opt_enable(LK_MUTEX_ENABLE) || !lock)) {
		return;
	}

	if (!current_is_important_ux()) {
		return;
	}

	owner_ts = __mutex_owner(lock);
	if (!owner_ts) {
		return;
	}

	get_task_struct(owner_ts);
	lock_inherit_ux_type(owner_ts, current, "mutex_wait");

	if (__mutex_owner(lock) != owner_ts) {
		cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"mutex owner has been changed owner=%p(%p)\n", __mutex_owner(lock), owner_ts);
		lock_clear_inherited_ux_type(owner_ts, "mutex_wait");
	}
	put_task_struct(owner_ts);
}


void android_vh_mutex_unlock_slowpath_handler(void *unused, struct mutex *lock)
{
	if (unlikely(!locking_opt_enable(LK_MUTEX_ENABLE)))
		return;

	lock_clear_inherited_ux_type(current, "mutex_wait_finish");
}

void register_mutex_vendor_hooks(void)
{
	register_trace_android_vh_alter_mutex_list_add(android_vh_alter_mutex_list_add_handler, NULL);
	register_trace_android_vh_mutex_wait_start(android_vh_mutex_wait_start_handler, NULL);
	register_trace_android_vh_mutex_unlock_slowpath(android_vh_mutex_unlock_slowpath_handler, NULL);
}

void unregister_mutex_vendor_hooks(void)
{
	unregister_trace_android_vh_alter_mutex_list_add(android_vh_alter_mutex_list_add_handler, NULL);
	unregister_trace_android_vh_mutex_wait_start(android_vh_mutex_wait_start_handler, NULL);
	unregister_trace_android_vh_mutex_unlock_slowpath(android_vh_mutex_unlock_slowpath_handler, NULL);
}
