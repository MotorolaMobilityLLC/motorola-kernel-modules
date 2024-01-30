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

#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#if defined CONFIG_KERNEL_6_1
#include <linux/sched/cputime.h>
#endif
#include <trace/hooks/sched.h>
#include <trace/hooks/signal.h>
#include <kernel/sched/sched.h>

#include "msched_common.h"
#include "locking/locking_main.h"

#define MS_TO_NS (1000000)
#define MAX_INHERIT_GRAN ((u64)(64 * MS_TO_NS))

// #define LOCK_DEBUG 1

#ifdef LOCK_DEBUG
atomic_t g_lock_count = ATOMIC_INIT(0);
#endif

static inline bool task_in_top_app_group(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	return (rcu_access_pointer(wts->grp) != NULL);
#else
	return get_task_cgroup_id(p) == CGROUP_TOP_APP;
#endif
}

static inline bool task_in_ux_related_group(struct task_struct *p)
{
	int ux_type = task_get_ux_type(p);

	// Boost all kernel threads with prio <= 120
	if (p->mm == NULL && p->prio <= 120) {
		return true;
	}

	if (ux_type & UX_TYPE_NATIVESERVICE && p->prio < 120 && p->prio >= 100) {
		return true;
	}

	if (moto_sched_scene & UX_SCENE_AUDIO) {
		if (ux_type & UX_TYPE_AUDIOSERVICE && p->prio <= 120 && p->prio >= 100)
			return true;
		else if (p->prio == 104 || p->prio == 101)
			return true;
	}

	if (moto_sched_scene & UX_SCENE_CAMERA) {
		if (ux_type & UX_TYPE_CAMERASERVICE && p->prio < 120 && p->prio >= 100)
			return true;
	}

	if (moto_sched_scene & UX_SCENE_TOUCH) {
		if (ux_type & UX_TYPE_GESTURE_MONITOR)
			return true;
	}

	if ((moto_sched_scene & (UX_SCENE_LAUNCH|UX_SCENE_TOUCH) || p->prio <= moto_boost_prio)
		&& task_in_top_app_group(p)) {
			return true;
	}

	if (p->tgid == global_launcher_tgid
			|| p->tgid == global_systemserver_tgid
			|| p->tgid == global_sysui_tgid
			|| p->tgid == global_sf_tgid) {
		if (moto_sched_scene & (UX_SCENE_LAUNCH|UX_SCENE_TOUCH))
			return true;
		else if (p->prio <= moto_boost_prio)
			return true;
	}

	return false;
}

int task_get_mvp_prio(struct task_struct *p, bool with_inherit)
{
	int ux_type = task_get_ux_type(p);
	int prio = UX_PRIO_INVALID;
	int inherit_prio = task_get_ux_inherit_prio(p);

	if (ux_type & UX_TYPE_PERF_DAEMON)
		prio = UX_PRIO_HIGHEST;
	else if (ux_type & UX_TYPE_AUDIO || ((ux_type & UX_TYPE_AUDIOSERVICE) && (p->prio == 101 || p->prio == 104)))
		prio = UX_PRIO_AUDIO;
	else if (ux_type & (UX_TYPE_INPUT|UX_TYPE_ANIMATOR|UX_TYPE_LOW_LATENCY_BINDER))
		prio = UX_PRIO_ANIMATOR;
	else if (ux_type & UX_TYPE_SYSTEM_LOCK)
		prio = UX_PRIO_SYSTEM;
	else if (ux_type & (UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI))
		prio = UX_PRIO_TOPAPP;
	else if (with_inherit && (ux_type & (UX_TYPE_INHERIT_BINDER)))
		prio = UX_PRIO_OTHER;
	else if (task_in_ux_related_group(p))
		prio = UX_PRIO_OTHER;
	else if (ux_type & UX_TYPE_KSWAPD)
		prio = UX_PRIO_KSWAPD;

	if (with_inherit && inherit_prio > 0) {
		prio = prio > inherit_prio ? prio : inherit_prio;
	}

	cond_trace_printk(moto_sched_debug,
		"pid=%d tgid=%d prio=%d scene=%d ux_type=%d inherit_prio=%d mvp_prio=%d\n",
		p->pid, p->tgid, p->prio, moto_sched_scene, ux_type, inherit_prio, prio);

	return prio;
}
EXPORT_SYMBOL(task_get_mvp_prio);

#define TOPAPP_MVP_LIMIT		120000000U	// 120ms
#define TOPAPP_MVP_LIMIT_BOOST	240000000U	// 240ms
#define SYSTEM_MVP_LIMIT		36000000U	// 36ms
#define SYSTEM_MVP_LIMIT_BOOST	72000000U	// 72ms
#define RTG_MVP_LIMIT			24000000U	// 24ms
#define RTG_MVP_LIMIT_BOOST		48000000U	// 48ms
#define KSWAPD_LIMIT			3000000000U	// 3000ms
#define DEF_MVP_LIMIT			12000000U	// 12ms
#define DEF_MVP_LIMIT_BOOST		24000000U	// 24ms

static inline bool task_in_top_related_group(struct task_struct *p) {
	return p->tgid == global_launcher_tgid
			|| p->tgid == global_sysui_tgid
			|| task_in_top_app_group(p);
}

unsigned int task_get_mvp_limit(struct task_struct *p, int mvp_prio) {
	bool boost = moto_sched_scene & (UX_SCENE_LAUNCH|UX_SCENE_TOUCH);

	if (mvp_prio == UX_PRIO_TOPAPP)
		return boost ? TOPAPP_MVP_LIMIT_BOOST : TOPAPP_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_SYSTEM || (p->tgid == global_systemserver_tgid && p->prio <= 120))
		return boost ? SYSTEM_MVP_LIMIT_BOOST : SYSTEM_MVP_LIMIT;
	else if (task_in_top_related_group(p) && p->prio <= 120)
		return boost ? RTG_MVP_LIMIT_BOOST : RTG_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_KSWAPD)
		return KSWAPD_LIMIT;
	else if (mvp_prio > UX_PRIO_INVALID)
		return boost ? DEF_MVP_LIMIT_BOOST : DEF_MVP_LIMIT;

	return 0;
}
EXPORT_SYMBOL(task_get_mvp_limit);

void binder_inherit_ux_type(struct task_struct *task) {
	if (current_is_important_ux()) {
		task_add_ux_type(task, UX_TYPE_INHERIT_BINDER);
	}
}
EXPORT_SYMBOL(binder_inherit_ux_type);

void binder_clear_inherited_ux_type(struct task_struct *task) {
	task_clr_ux_type(task, UX_TYPE_INHERIT_BINDER);
}
EXPORT_SYMBOL(binder_clear_inherited_ux_type);

void queue_ux_task(struct rq *rq, struct task_struct *task, int enqueue) {
	if (enqueue) {

	} else {
		struct moto_task_struct *wts = get_moto_task_struct(task);
		if (wts->inherit_mvp_prio > 0) {
			if (jiffies_to_nsecs(jiffies) - wts->inherit_start > MAX_INHERIT_GRAN) {
	#ifdef LOCK_DEBUG
				atomic_dec(&g_lock_count);
				cond_trace_printk(moto_sched_debug,
						"lock_clear_inherited_ux_type %s  %d  ux_type %d -> %d  cost=%llu total=%d\n", "dequeue task",
						task->pid, wts->ux_type, wts->inherit_mvp_prio,
						(jiffies_to_nsecs(jiffies) - wts->inherit_start) / 1000000U, atomic_read(&g_lock_count));
	#endif
				task_clr_inherit_type(task);
			}
		}
	}
}
EXPORT_SYMBOL(queue_ux_task);

void binder_ux_type_set(struct task_struct *task) {
	if (task && ((task_in_top_related_group(current) && task->group_leader->prio < MAX_RT_PRIO)
					|| (current->group_leader->prio < MAX_RT_PRIO && task_in_top_related_group(task))))
		task_add_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);
	else
		task_clr_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);

	cond_trace_printk(moto_sched_debug,
			"current (tgid=%d leader_prio=%d) task (tgid=%d leader_prio=%d) ux_type=%d\n",
			current->tgid, current->group_leader->prio, task->tgid, task->group_leader->prio, task_get_ux_type(task));
}
EXPORT_SYMBOL(binder_ux_type_set);

bool lock_inherit_ux_type(struct task_struct *owner, struct task_struct *waiter, char* lock_name) {
#ifdef LOCK_DEBUG
	struct moto_task_struct *owner_wts;
	struct moto_task_struct *waiter_wts;
#endif
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner || !waiter || owner->prio <= 100 ||
		task_get_ux_depth(waiter) >= UX_DEPTH_MAX || task_is_important_ux(owner)) {
		return false;
	}

	rq = task_rq_lock(owner, &flags);

#ifdef LOCK_DEBUG
	owner_wts = (struct moto_task_struct *) owner->android_oem_data1;
	waiter_wts = (struct moto_task_struct *) waiter->android_oem_data1;
	atomic_inc(&g_lock_count);
	cond_trace_printk(moto_sched_debug,
			"lock_inherit_ux_type %s %d -> %d   ux_type %d -> %d  depth=%d total=%d\n",
			lock_name, waiter->pid, owner->pid, waiter_wts->ux_type, owner_wts->ux_type,
			waiter_wts->inherit_depth, atomic_read(&g_lock_count));
#endif
	task_set_ux_inherit_prio(owner, task_get_mvp_prio(waiter, true),
		task_get_ux_depth(waiter) + 1);

	task_rq_unlock(rq, owner, &flags);
	return true;
}

bool lock_clear_inherited_ux_type(struct task_struct *owner, char* lock_name) {
#ifdef LOCK_DEBUG
	struct moto_task_struct *owner_wts;
#endif
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner) {
		return false;
	}
	if (task_get_ux_inherit_prio(owner) <= 0) {
		return false;
	}

	rq = task_rq_lock(owner, &flags);

#ifdef LOCK_DEBUG
	owner_wts = get_moto_task_struct(owner);
	atomic_dec(&g_lock_count);
	cond_trace_printk(moto_sched_debug,
			"lock_clear_inherited_ux_type %s  %d  ux_type %d -> %d  cost=%llu total=%d\n", lock_name,
			owner->pid, owner_wts->ux_type, owner_wts->inherit_mvp_prio,
			(jiffies_to_nsecs(jiffies) - owner_wts->inherit_start) / 1000000U, atomic_read(&g_lock_count));
#endif
	task_clr_inherit_type(owner);

	task_rq_unlock(rq, owner, &flags);
	return true;
}

static void android_vh_dup_task_struct(void *unused, struct task_struct *task, struct task_struct *orig)
{
	int ux_type = task_get_ux_type(orig);
	if (ux_type & (UX_TYPE_AUDIOSERVICE|UX_TYPE_NATIVESERVICE|UX_TYPE_CAMERASERVICE)) {
		task_add_ux_type(task, ux_type);
	}
}

void register_vendor_comm_hooks(void)
{
	register_trace_android_vh_dup_task_struct(android_vh_dup_task_struct, NULL);
}
