// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#include <linux/cgroup-defs.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <linux/percpu-rwsem.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/stacktrace.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#include <linux/sched/cputime.h>
#endif

#include <trace/hooks/rwsem.h>
#include <trace/hooks/dtask.h>

#include <../kernel/sched/sched.h>
#include "../msched_common.h"
#include "locking_main.h"
#include "locking_trace.h"

#define ENABLE_REORDER_LIST 1
#define ENABLE_INHERITE 1

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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
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

static DEFINE_SPINLOCK(RWSEM_SPIN_LOCK);

void reset_rwsem_owner_list(struct moto_task_struct *mts)
{
	unsigned long flags;
	if(!mts)
		return;

	spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);
	list_del_init(&mts->owner_node);
	spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

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
bool rwsem_list_add(struct task_struct *tsk, struct list_head *entry, struct list_head *head)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	struct rwsem_waiter *entry_waiter;
	int index = 0;
	int prio = 0;

	if (!entry || !head) {
		printk(KERN_ERR "rwsem_list_add %p %p is NULL", entry, head);
		return false;
	}
	prio = task_get_mvp_prio(current, true);
	entry_waiter = list_entry(entry, struct rwsem_waiter, list);
	if (prio > 0) {
		list_for_each_safe(pos, n, head) {
			waiter = list_entry(pos, struct rwsem_waiter, list);
			if (waiter && waiter->task->prio > MAX_RT_PRIO && prio > task_get_mvp_prio(waiter->task, true)) {
				cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
					"rwsem_list_add %d prio=%d(%d)index=%d\n", tsk->pid, prio, task_get_mvp_prio(waiter->task, true), index);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
				entry_waiter->handoff_set = waiter->handoff_set;
				waiter->handoff_set = false;
#endif
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
		cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
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
	bool ret = false;
	if (!waiter || !sem)
		return;

	if (unlikely(!locking_opt_enable()))
		return;

	if (waiter->type == RWSEM_WAITING_FOR_READ)
		return;

	if (test_wait_timeout(sem))
		return;

	ret = rwsem_list_add(waiter->task, &waiter->list, &sem->wait_list);

	if (ret)
		*already_on_list = true;
}
#endif

#ifdef ENABLE_INHERITE

static void android_vh_rwsem_init(void *unused, struct rw_semaphore *sem)
{
	unsigned long flags;
	struct list_head *owner_list;

	if (unlikely(!locking_opt_enable()) || !sem)
		return;

	owner_list = (struct list_head *)sem->android_oem_data1;

	spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);
	sem->android_vendor_data1 = 0;
	INIT_LIST_HEAD(owner_list);
	trace_locking_debug_trace(sem, __func__, "init", (unsigned long)owner_list->prev, (unsigned long)owner_list->next);
	spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

static void android_vh_rwsem_record_rwsem_reader_owned(void *unused, struct rw_semaphore *sem, struct list_head *wlist)
{
	unsigned long flags;
	struct list_head *owner_list;

	// trace_locking_debug_trace(sem, __func__, "record", (unsigned long)owner_list->prev, (unsigned long)owner_list->next);

	if (unlikely(!locking_opt_enable()) || !sem)
		return;

	owner_list = (struct list_head *)sem->android_oem_data1;

	spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);
	if (owner_list->prev == NULL || owner_list->next == NULL)
		INIT_LIST_HEAD(owner_list);

	if (wlist && !list_empty(wlist)) {
		struct rwsem_waiter *waiter;

		list_for_each_entry(waiter, wlist, list) {
			if (waiter->task) {
				struct moto_task_struct *vp = get_moto_task_struct(waiter->task);
				if (IS_ERR_OR_NULL(vp))
					continue;
				if (list_empty(&vp->owner_node)) {
					list_add_tail(&vp->owner_node, owner_list);
					vp->owner = (u64)sem;
					trace_locking_debug_trace(sem, __func__, "record rd owned", waiter->task->pid, 0);
				}
			}
		}
	}
	else {
		struct moto_task_struct *vp = get_moto_task_struct(current);
		if (!IS_ERR_OR_NULL(vp)) {
			if (list_empty(&vp->owner_node)) {
				list_add_tail(&vp->owner_node, owner_list);
				vp->owner = (u64)sem;
				trace_locking_debug_trace(sem, __func__, "record rd owned", current->pid, 0);
			}
			if (sem->android_vendor_data1 && (!current_is_important_ux() && (current->prio > 100))) {
				lock_inherit_ux_type(current, current, "rwsem_leak_boost");
			}
		}
	}
	spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

static void android_vh_rwsem_record_rwsem_writer_owned(void *unused, struct rw_semaphore *sem)
{
	unsigned long flags;
	struct moto_task_struct *vp = get_moto_task_struct(current);
	struct list_head *owner_list;

	// trace_locking_debug_trace(sem, __func__, "clear", (unsigned long)owner_list->prev, (unsigned long)owner_list->next);

	if (unlikely(!locking_opt_enable()) || !sem)
		return;

	if (IS_ERR_OR_NULL(vp))
		return;

	owner_list = (struct list_head *)sem->android_oem_data1;

	spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);
	if (owner_list->prev == NULL || owner_list->next == NULL)
		INIT_LIST_HEAD(owner_list);

	if (list_empty(&vp->owner_node)) {
		list_add_tail(&vp->owner_node, owner_list);
		vp->owner = (u64)sem;
		trace_locking_debug_trace(sem, __func__, "record wr owned", 0, 0);
	}

	if (sem->android_vendor_data1 && (!current_is_important_ux() && (current->prio > 100))) {
		lock_inherit_ux_type(current, current, "rwsem_leak_boost");
	}

	spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

static void android_vh_rwsem_clear_rwsem_owned(void *unused, struct rw_semaphore *sem)
{
	unsigned long flags;
	struct moto_task_struct *vp = get_moto_task_struct(current);

	if (unlikely(!locking_opt_enable()))
		return;

	if (IS_ERR_OR_NULL(vp))
		return;

	spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);

	if (!list_empty(&vp->owner_node) && vp->owner == (u64)sem) {
		list_del_init(&vp->owner_node);
		vp->owner = 0;
		trace_locking_debug_trace(sem, __func__, "clear owned", 0, 0);
		lock_clear_inherited_ux_type(current, "clear_rwsem_own");
	}

	spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

// Define a reasonable limit for how many waiters we'll boost in one go
// to avoid excessive stack usage.
#define MAX_WAITERS_TO_BOOST 16

static void rwsem_wait_start(struct rw_semaphore *sem)
{
    unsigned long flags;
    struct moto_task_struct *owner_vp = NULL;
    struct rwsem_waiter *waiter = NULL;
	struct task_struct *owner;
    struct list_head *owner_list;

    // Array to hold tasks for boosting outside the lock
    struct task_struct *tasks_to_boost[MAX_WAITERS_TO_BOOST];
    int boost_count = 0;
    int i;
	bool already_boosted = false;

	if (!sem) {
		return;
	}
	owner_list = (struct list_head *)sem->android_oem_data1;

	if (owner_list->prev == NULL || owner_list->next == NULL)
		return;

    // --- Part 1: Boost Owners (using your dedicated spinlock) ---
    spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);

	owner = rwsem_owner(sem);

    list_for_each_entry(owner_vp, owner_list, owner_node) {
		struct task_struct *owner_ts = owner_vp->task;
		if (owner_ts) {
			if (!already_boosted && owner_ts == owner) {
				already_boosted = true;
			}
			// It's safe to call this here IF it doesn't sleep. Assuming it doesn't.
			lock_inherit_ux_type(owner_ts, current, "rwsem_owner_boost");

			boost_count++;
		}
    }

	trace_locking_debug_trace(sem, __func__, "boosting owners", boost_count, 0);

	boost_count = 0;

	if (!already_boosted && owner) {
		get_task_struct(owner);
		lock_inherit_ux_type(owner, current, "rwsem_cowner_boost");
		put_task_struct(owner);
	}
	// trace_locking_debug_trace(sem, __func__, "boosting waiters", 0, 0);

    // --- Part 2: Safely collect waiters for boosting ---

    list_for_each_entry(waiter, &sem->wait_list, list) {
        if (waiter->task == current) {
            break; // Stop at ourselves
        }

        if (waiter->task && boost_count < MAX_WAITERS_TO_BOOST) {
            // Get a reference to the task and store it
            get_task_struct(waiter->task);
            tasks_to_boost[boost_count++] = waiter->task;
        }
    }

    // --- Part 3: Perform the actual boosting outside the lock ---
    if (boost_count > 0) {
        trace_locking_debug_trace(sem, __func__, "boosting waiters", boost_count, 0);
    }

    // spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);

    for (i = 0; i < boost_count; i++) {
        lock_inherit_ux_type(tasks_to_boost[i], current, "rwsem_waiter_boost");
        // Release the reference we took inside the lock
        put_task_struct(tasks_to_boost[i]);
    }

	sem->android_vendor_data1 = 1;

    spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}

static void android_vh_rwsem_wake(void *unused, struct rw_semaphore *sem)
{
    if (unlikely(!locking_opt_enable()) || !sem || (!current_is_important_ux() && (current->prio > 100))) {
        return;
    }
	rwsem_wait_start(sem);
}

static void android_vh_rwsem_wait_finish(void *unused, struct rw_semaphore *sem)
{
	unsigned long flags;
    if (unlikely(!locking_opt_enable()) || !sem || (!current_is_important_ux() && (current->prio > 100))) {
        return;
    }

    spin_lock_irqsave(&RWSEM_SPIN_LOCK, flags);

	sem->android_vendor_data1 = 0;

    spin_unlock_irqrestore(&RWSEM_SPIN_LOCK, flags);
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static void android_vh_record_pcpu_rwsem_starttime(void *unused, struct percpu_rw_semaphore *sem, unsigned long settime_jiffies)
{
	if (unlikely(!locking_opt_enable()))
		return;

	if (sem == &cgroup_threadgroup_rwsem) {
		lock_protect_update_starttime(current, settime_jiffies, "percpu_rwsem", sem);
	}
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void android_vh_record_pcpu_rwsem_time_early(void *unused, unsigned long settime_jiffies, struct percpu_rw_semaphore *sem)
{
	if (unlikely(!locking_opt_enable()))
		return;

	if (sem == &cgroup_threadgroup_rwsem) {
		lock_protect_update_starttime(current, settime_jiffies, "percpu_rwsem", sem);
	}
}
#endif


void register_rwsem_vendor_hooks(void)
{
#ifdef ENABLE_REORDER_LIST
	register_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif

#ifdef ENABLE_INHERITE
	register_trace_android_vh_rwsem_init(android_vh_rwsem_init, NULL);

	register_trace_android_vh_clear_rwsem_reader_owned(android_vh_rwsem_clear_rwsem_owned, NULL);
	register_trace_android_vh_clear_rwsem_writer_owned(android_vh_rwsem_clear_rwsem_owned, NULL);

	register_trace_android_vh_rwsem_wake(android_vh_rwsem_wake, NULL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	register_trace_android_vh_record_rwsem_writer_owned(android_vh_rwsem_record_rwsem_writer_owned, NULL);
#else
	register_trace_android_vh_rwsem_set_owner(android_vh_rwsem_record_rwsem_writer_owned, NULL);
#endif
	register_trace_android_vh_record_rwsem_reader_owned(android_vh_rwsem_record_rwsem_reader_owned, NULL);
	register_trace_android_vh_rwsem_read_wait_finish(android_vh_rwsem_wait_finish, NULL);
	register_trace_android_vh_rwsem_write_wait_finish(android_vh_rwsem_wait_finish, NULL);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	register_trace_android_vh_record_pcpu_rwsem_starttime(android_vh_record_pcpu_rwsem_starttime, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
    register_trace_android_vh_record_pcpu_rwsem_time_early(android_vh_record_pcpu_rwsem_time_early, NULL);
#endif
}

void unregister_rwsem_vendor_hooks(void)
{
#ifdef ENABLE_REORDER_LIST
	unregister_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif
#ifdef ENABLE_INHERITE
	unregister_trace_android_vh_rwsem_init(android_vh_rwsem_init, NULL);
	unregister_trace_android_vh_rwsem_wake(android_vh_rwsem_wake, NULL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	unregister_trace_android_vh_record_rwsem_writer_owned(android_vh_rwsem_record_rwsem_writer_owned, NULL);
#else
	unregister_trace_android_vh_rwsem_set_owner(android_vh_rwsem_record_rwsem_writer_owned, NULL);
#endif
	unregister_trace_android_vh_record_rwsem_reader_owned(android_vh_rwsem_record_rwsem_reader_owned, NULL);
	unregister_trace_android_vh_clear_rwsem_writer_owned(android_vh_rwsem_clear_rwsem_owned, NULL);
	unregister_trace_android_vh_clear_rwsem_reader_owned(android_vh_rwsem_clear_rwsem_owned, NULL);
	unregister_trace_android_vh_rwsem_read_wait_finish(android_vh_rwsem_wait_finish, NULL);
	unregister_trace_android_vh_rwsem_write_wait_finish(android_vh_rwsem_wait_finish, NULL);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	unregister_trace_android_vh_record_pcpu_rwsem_starttime(android_vh_record_pcpu_rwsem_starttime, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
    unregister_trace_android_vh_record_pcpu_rwsem_time_early(android_vh_record_pcpu_rwsem_time_early, NULL);
#endif
}

