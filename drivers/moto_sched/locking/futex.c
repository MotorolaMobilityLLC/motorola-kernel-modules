// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#include <include/linux/sched.h>
#include <kernel/sched/sched.h>
#include <include/linux/hrtimer.h>
#include <include/linux/futex.h>
#include <include/linux/sched/task.h>
#include <linux/pid.h>
#include <trace/hooks/futex.h>

#include "../msched_common.h"
#include "locking_main.h"

#ifdef CONFIG_MMU
# define FLAGS_SHARED		0x01
#else
# define FLAGS_SHARED		0x00
#endif


/*
 * Note:
 * Add new futex ops, which should be different from other ops defined
 * in include/uapi/linux/futex.h.
 */
#define FUTEX_WAIT_NOTIFY_WAITER 15

#define FUTEX_WAITER_TOLERATE_THRESHOLD (200000000) /* 200ms */

atomic64_t futex_inherit_set_times;
atomic64_t futex_inherit_unset_times;
atomic64_t futex_inherit_useless_times;

atomic64_t futex_low_count;
atomic64_t futex_high_count;

/*
 * Note:
 * structure futex_q should keep same as it's definition
 * in kernel/futex.c.
 */
struct futex_q {
	struct plist_node list;

	struct task_struct *task;
	spinlock_t *lock_ptr;
	union futex_key key;
	struct futex_pi_state *pi_state;
	struct rt_mutex_waiter *rt_waiter;
	union futex_key *requeue_pi_key;
	u32 bitset;
} __randomize_layout;

#define INHERIT_SET (1)
#define INHERIT_INC (2)
static int futex_set_inherit_ux_refs(struct task_struct *holder, struct task_struct *p)
{
	if (unlikely(!holder || !p))
		return 0;

	if (task_is_important_ux(p)) {
		cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"futex_set_inherit_ux %d -> %d\n", p->pid, holder->pid);
		task_add_ux_type(holder, UX_TYPE_INHERIT_FUTEX);
		return INHERIT_SET;
	}

	return 0;
}

static void futex_unset_inherit_ux_refs(struct task_struct *p, int value, int path)
{
	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
		"futex_unset_inherit_ux %d\n", p->pid);
	task_clr_ux_type(p, UX_TYPE_INHERIT_FUTEX);
}

static inline bool curr_is_inherit_futex(void)
{
	return task_get_ux_type(current) & UX_TYPE_INHERIT_FUTEX;
}

static bool boost_holder(struct task_struct *holder, struct task_struct *waiter)
{
	struct moto_task_struct *mts;

	mts = get_moto_task_struct(waiter);
	if (unlikely(IS_ERR_OR_NULL(mts)))
		return false;

	/*
	 * When futex-opt disabled, still keep futex-holder tracing, it won't affect
	 * performance/power. But stop these two functions:
	 * 1. modify ux thread's pnode-prio in android_vh_alter_futex_plist_add() hook.
	 * 2. set inherit ux thread with futex type in android_vh_futex_sleep_start() hook.
	 */
	if (unlikely(!locking_opt_enable(LK_FUTEX_ENABLE)))
		return false;

	/*
	 * If current(ux thread) upgrade it's holder to inherit ux successfully,
	 * mark ux_contrib as true(Ya, we have contributed an inherit ux thread).
	 */
	if (futex_set_inherit_ux_refs(holder, waiter)) {
		mts->lkinfo.ux_contrib = true;
		return true;
	}

	return false;
}

/**
 * match_futex - Check whether two futex keys are equal
 * @key1:	Pointer to key1
 * @key2:	Pointer to key2
 *
 * Return 1 if two futex_keys are equal, 0 otherwise.
 */
static inline int match_futex(union futex_key *key1, union futex_key *key2)
{
	return (key1 && key2
		&& key1->both.word == key2->both.word
		&& key1->both.ptr == key2->both.ptr
		&& key1->both.offset == key2->both.offset);
}

static struct task_struct *futex_find_task_by_pid(unsigned int pid)
{
	struct task_struct *p;

	if (pid <= 0 || pid > PID_MAX_DEFAULT)
		return NULL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (p)
		get_task_struct(p);
	rcu_read_unlock();

	return p;
}

static bool set_holder(struct task_struct *waiter, struct task_struct *holder)
{
	struct moto_task_struct *mts;

	if (waiter->tgid == holder->tgid) {
		mts = get_moto_task_struct(waiter);
		/* If waiter's holder is set, do put_task_struct(holder) in futex_wait_end() */
		if (likely(!IS_ERR_OR_NULL(mts))) {
			mts->lkinfo.holder = holder;
			return true;
		}
	}

	put_task_struct(holder);
	return false;
}

static void futex_notify_waiter(unsigned long pid)
{
	struct task_struct *waiter, *holder = current;
	bool is_target = false;
	bool try_to_boost;

	waiter = futex_find_task_by_pid(pid);
	if (!waiter)
		return;

	is_target = task_is_important_ux(waiter);

	if (!is_target)
		goto out_put_waiter;

	get_task_struct(holder);
	try_to_boost = set_holder(waiter, holder);

	/* TODO: should check waiter's state? it should be sleeping */
	if (try_to_boost)
		boost_holder(holder, waiter);

out_put_waiter:
	put_task_struct(waiter);
}

/* using the param flags to pass info to wait_start&wait_end.
 * flags total 32 bits long:
 * wait situation:
 *     bit0 ~ bit7 : Used by kernel.
 *     bit8 ~ bit29 : Used to pass owner tid.
 *     bit30 ~ bit31 : Used to pass lock type of JUC/ART.
 * wake situation:
 *     N/A.
 **/
#define WAKE_MSG (64)
#define NOTIFY_WAITER_SHIFT (16)
#define NOTIFY_OWNER_MASK   ((1 << NOTIFY_WAITER_SHIFT)-1)
#define FLAGS_OWNER_SHIFT		(8)
extern int thread_info_ctrl;

static void android_vh_do_futex_handler(void *unused, int cmd,
		  unsigned int *flags, u32 __user *uaddr2)
{
	unsigned long waiter_pid;
	char wake_msg[WAKE_MSG];
	struct futex_uinfo info;

	switch (cmd) {
	case FUTEX_WAIT:
		fallthrough;
	case FUTEX_WAIT_BITSET:
		if (NULL != uaddr2) {
			if (copy_from_user(&info, uaddr2, sizeof(info))) {
				cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
						"copy from user failed, uaddr2 = 0x%lx \n",
						(unsigned long)uaddr2);
				return;
			}
			memset(&info.inform_user, 0, sizeof(info.inform_user));
			/* owner_tid part:*/
			if ((info.cmd & MAGIC_MASK)  == MAGIC_NUM) {
				if (info.cmd & OWNER_BIT) {
					/* owner_tid != 0 indicate that there is an owner to be set. */
					if (info.owner_tid > 0 && info.owner_tid <= PID_MAX_LIMIT) {
						*flags += info.owner_tid << FLAGS_OWNER_SHIFT;
					} else {
						pr_err("Set owner failed, invailid tid.\n");
					}
				}
			}
		}
		break;
	case FUTEX_WAIT_NOTIFY_WAITER:
		waiter_pid = (unsigned long)uaddr2 >> NOTIFY_WAITER_SHIFT;
		futex_notify_waiter(waiter_pid);
		break;
	case FUTEX_WAKE:
		if ((uaddr2 != NULL) && !copy_from_user(wake_msg, uaddr2, WAKE_MSG - 1))
			wake_msg[WAKE_MSG-1] = '\0';
		break;
	}
}

static void android_vh_futex_wait_start_handler(void *unused, unsigned int flags,
		  u32 bitset)
{
	struct task_struct *holder;

	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"current_is_important_ux %d, set_holder %d -> %d\n", current_is_important_ux(), current->pid, (flags & ~(0x3 << LOCK_TYPE_SHIFT)) >> FLAGS_OWNER_SHIFT);

	if (!current_is_important_ux())
		return;

	holder = futex_find_task_by_pid((flags & ~(0x3 << LOCK_TYPE_SHIFT)) >> FLAGS_OWNER_SHIFT);
	if (!holder)
		return;

	set_holder(current, holder);
}

static void android_vh_futex_wait_end_handler(void *unused, unsigned int flags,
		  u32 bitset)
{
	struct moto_task_struct *mts;

	mts = get_moto_task_struct(current);
	if (IS_ERR_OR_NULL(mts))
		return;

	if (mts->lkinfo.holder) {
		if (unlikely(mts->lkinfo.ux_contrib)) {
			futex_unset_inherit_ux_refs(mts->lkinfo.holder, 1, 3);
		}

		put_task_struct(mts->lkinfo.holder);
		mts->lkinfo.holder = NULL;
	}
}

void android_vh_alter_futex_plist_add_handler(void *unused, struct plist_node *node,
		    struct plist_head *head, bool *already_on_hb)
{
	struct sched_entity *se;
	struct rq *rq;
	struct rq_flags rf;
	struct futex_q *this, *next, *cur;
	struct task_struct *first_normal_waiter = NULL;
	u64 sleep_start;
	int prio;
	struct moto_task_struct *mts;

	if (unlikely(!locking_opt_enable(LK_FUTEX_ENABLE) || *already_on_hb))
		return;

	if (!current_is_important_ux())
		return;

	mts = get_moto_task_struct(current);
	if (mts->lkinfo.holder)
		boost_holder(mts->lkinfo.holder, current);

	cur = (struct futex_q *) node;
	/*
	 * Find out the first normal thread in &hb->chain(FIFO if it's a normal node),
	 * make sure &hb->lock is held.
	 */
	plist_for_each_entry_safe(this, next, head, list) {
		struct plist_node *tmp = &this->list;

		if (match_futex(&this->key, &cur->key)) {
			if (tmp->prio == MAX_RT_PRIO) {
				first_normal_waiter = this->task;
				break;
			}
		}
	}

	if (first_normal_waiter == NULL)
		goto re_init;

	rq = task_rq_lock(first_normal_waiter, &rf);
	se = &first_normal_waiter->se;
	sleep_start = schedstat_val(se->statistics.sleep_start);

	if (sleep_start) {
		u64 delta = rq_clock(rq) - sleep_start;

		if ((s64)delta < 0)
			delta = 0;

		if (delta > FUTEX_WAITER_TOLERATE_THRESHOLD) {
			task_rq_unlock(rq, first_normal_waiter, &rf);
			return;
		}
	}
	task_rq_unlock(rq, first_normal_waiter, &rf);

re_init:
	/*
	 * &hb->chain's pnode should be one of below:
	 * RT thread: pnode's prio is p->normal_prio.
	 * UX thread: pnode's prio is MAX_RT_PRIO - 1.
	 * Normal thread: pnode's prio is MAX_RT_PRIO.
	 */
	prio = min(current->normal_prio, MAX_RT_PRIO - 1);
	plist_node_init(node, prio);
}

static void android_vh_futex_sleep_start_handler(void *unused,
		  struct task_struct *p)
{
	struct moto_task_struct *mts;

	mts = get_moto_task_struct(p);
	if (unlikely(IS_ERR_OR_NULL(mts)))
		return;
}

static void android_vh_futex_wake_traverse_plist_handler(void *unused, struct plist_head *chain,
		  int *target_nr, union futex_key key, u32 bitset)
{
	struct futex_q *this, *next;
	struct task_struct *p;
	struct moto_task_struct *mts;
	int idx = 0;

	*target_nr = 0;
	if (likely(!curr_is_inherit_futex()))
		return;

	/*
	 * If waker is an inherit-futex ux thread, see how many ux waiters in this chain,
	 * make sure &hb->lock is held.
	 */
	plist_for_each_entry_safe(this, next, chain, list) {
		if (match_futex(&this->key, &key)) {
			p = this->task;
			mts = get_moto_task_struct(p);
			if (unlikely(IS_ERR_OR_NULL(mts)))
				continue;

			if ((mts->lkinfo.holder == current) && mts->lkinfo.ux_contrib) {
				*target_nr += 1;
				mts->lkinfo.ux_contrib = false;
			}
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"idx=%d this=%-12s pid=%d tgid=%d nr=%d\n",
				++idx, p->comm, p->pid, p->tgid, *target_nr);
		}
	}
}

static void android_vh_futex_wake_this_handler(void *unused, int ret, int nr_wake, int target_nr,
		  struct task_struct *p)
{
	struct moto_task_struct *mts;

	mts = get_moto_task_struct(p);
	if (unlikely(IS_ERR_OR_NULL(mts)))
		return;

	if (mts->lkinfo.ux_contrib && mts->lkinfo.holder) {
		futex_unset_inherit_ux_refs(mts->lkinfo.holder, 1, 2);
		mts->lkinfo.ux_contrib = false;
	}

	if (likely(!curr_is_inherit_futex()))
		return;
}

static void android_vh_futex_wake_up_q_finish_handler(void *unused, int nr_wake,
		  int target_nr)
{
	if (likely(!curr_is_inherit_futex()))
		return;

	/* Owner will be changed, remove current's inherit count. */
	if (target_nr) {
		futex_unset_inherit_ux_refs(current, target_nr, 1);
	}
}

void register_futex_vendor_hooks(void)
{
	register_trace_android_vh_do_futex(
		android_vh_do_futex_handler,
		NULL);
	register_trace_android_vh_futex_wait_start(
		android_vh_futex_wait_start_handler,
		NULL);
	register_trace_android_vh_futex_wait_end(
		android_vh_futex_wait_end_handler,
		NULL);
	register_trace_android_vh_alter_futex_plist_add(
		android_vh_alter_futex_plist_add_handler,
		NULL);
	register_trace_android_vh_futex_sleep_start(
		android_vh_futex_sleep_start_handler,
		NULL);
	register_trace_android_vh_futex_wake_traverse_plist(
		android_vh_futex_wake_traverse_plist_handler,
		NULL);
	register_trace_android_vh_futex_wake_this(
		android_vh_futex_wake_this_handler,
		NULL);
	register_trace_android_vh_futex_wake_up_q_finish(
		android_vh_futex_wake_up_q_finish_handler,
		NULL);
}

void unregister_futex_vendor_hooks(void)
{
	unregister_trace_android_vh_do_futex(
		android_vh_do_futex_handler,
		NULL);
	unregister_trace_android_vh_futex_wait_start(
		android_vh_futex_wait_start_handler,
		NULL);
	unregister_trace_android_vh_futex_wait_end(
		android_vh_futex_wait_end_handler,
		NULL);
	unregister_trace_android_vh_alter_futex_plist_add(
		android_vh_alter_futex_plist_add_handler,
		NULL);
	unregister_trace_android_vh_futex_sleep_start(
		android_vh_futex_sleep_start_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_traverse_plist(
		android_vh_futex_wake_traverse_plist_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_this(
		android_vh_futex_wake_this_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_up_q_finish(
		android_vh_futex_wake_up_q_finish_handler,
		NULL);
}