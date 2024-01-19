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
#include <trace/hooks/sched.h>
#include <kernel/sched/sched.h>

#include "msched_common.h"
#include "locking/locking_main.h"

// #define LOCK_DEBUG 0

#ifdef LOCK_DEBUG
atomic_t g_lock_count = ATOMIC_INIT(0);
#endif

static inline bool task_in_audio_server_group(struct task_struct *p)
{
	int ux_type = task_get_ux_type(p);
	if (ux_type & UX_TYPE_AUDIOSERVICE) {
		return p->prio == 101 || p->prio == 104;
	}
	return false;
}

static inline bool task_in_ux_related_group(struct task_struct *p, int cgroup_id)
{
	int ux_type = task_get_ux_type(p);

	if (ux_type & UX_TYPE_NATIVESERVICE) {
		return p->prio < 120 && p->prio >= 100;
	}

	if (cgroup_id == CGROUP_DEFAULT) {
		return p->prio == 100;
	}

	if (cgroup_id == CGROUP_TOP_APP) {
		if (moto_sched_scene & UX_SCENE_LAUNCH) {
			return true;
		} else if (p->prio <= moto_boost_prio) {
			return true;
		}
	}

	if (moto_sched_scene & UX_SCENE_AUDIO) {
		if (ux_type & UX_TYPE_AUDIOSERVICE) {
			return p->prio < 120 && p->prio >= 100;
		} else if (cgroup_id == CGROUP_FOREGROUND) {
			return p->prio == 104 || p->prio == 101;
		}
	}

	if (moto_sched_scene & UX_SCENE_LAUNCH) {
		if (p->tgid == global_launcher_tgid || p->tgid == global_systemserver_tgid) {
			return p->prio <= moto_boost_prio;
		}
	}

	if (moto_sched_scene & UX_SCENE_CAMERA) {
		if (ux_type & UX_TYPE_CAMERASERVICE) {
			return p->prio < 120 && p->prio >= 100;
		}
	}
	return false;
}

int task_get_mvp_prio(struct task_struct *p, bool with_inherit)
{
	int ux_type = task_get_ux_type(p);
	int cgroup_id = get_task_cgroup_id(p);
	int prio = UX_PRIO_INVALID;
	int inherit_prio = task_get_ux_inherit_prio(p);

	if (ux_type & UX_TYPE_PERF_DAEMON)
		prio = UX_PRIO_HIGHEST;
	else if (ux_type & UX_TYPE_AUDIO || task_in_audio_server_group(p))
		prio = p->prio <= 101 ? UX_PRIO_URGENT_AUDIO : UX_PRIO_AUDIO;
	else if (ux_type & UX_TYPE_INPUT)
		prio = UX_PRIO_INPUT;
	else if (ux_type & UX_TYPE_ANIMATOR)
		prio = p->prio <= 116 ? UX_PRIO_ANIMATOR : UX_PRIO_ANIMATOR_LOW;
	else if (ux_type & UX_TYPE_TOPAPP)
		prio = UX_PRIO_TOPAPP;
	else if (ux_type & UX_TYPE_TOPUI)
		prio = UX_PRIO_TOPUI;
	else if ((moto_sched_scene & UX_SCENE_LAUNCH) && (ux_type & UX_TYPE_LAUNCHER))
		prio = UX_PRIO_LAUNCHER;
	else if ((moto_sched_scene & UX_SCENE_TOUCH) && (ux_type & UX_TYPE_GESTURE_MONITOR))
		prio = UX_PRIO_GESTURE_MONITOR;
	else if (with_inherit && (ux_type & (UX_TYPE_INHERIT_BINDER)))
		prio = UX_PRIO_INHERIT;
	else if (ux_type & UX_TYPE_SYSTEM_LOCK)
		prio = UX_PRIO_SYS_LOCK;
	else if (task_in_ux_related_group(p, cgroup_id))
		prio = p->prio < 120 ? (UX_PRIO_OTHER_HIGH - (p->prio - 100)) : (UX_PRIO_OTHER_LOW - (p->prio - 120));
	else if (ux_type & UX_TYPE_KSWAPD)
		prio = UX_PRIO_KSWAPD;

	if (with_inherit && inherit_prio > 0) {
		return prio > inherit_prio ? prio : inherit_prio;
	}
	return prio;
}
EXPORT_SYMBOL(task_get_mvp_prio);

#define AUDIO_MVP_LIMIT		12000000U	// 12ms
#define TOPAPP_MVP_LIMIT	120000000U	// 120ms
#define KSWAPD_MVP_LIMIT	120000000U	// 120ms
#define DEF_MVP_LIMIT		12000000U	// 12ms
unsigned int task_get_mvp_limit(int mvp_prio) {
	if (mvp_prio == UX_PRIO_URGENT_AUDIO || mvp_prio == UX_PRIO_AUDIO)
		return AUDIO_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_TOPAPP || mvp_prio == UX_PRIO_LAUNCHER || mvp_prio == UX_PRIO_TOPUI)
		return TOPAPP_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_KSWAPD)
		return KSWAPD_MVP_LIMIT;
	else if (mvp_prio > UX_PRIO_INVALID)
		return DEF_MVP_LIMIT;

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

void binder_ux_type_set(struct task_struct *task, bool has_clear, bool clear) {
	if (has_clear) {
		if (!clear) {
			if (task && ((current->tgid == global_launcher_tgid &&
					task->group_leader->prio < MAX_RT_PRIO) ||
					(current->group_leader->prio < MAX_RT_PRIO &&
					task->tgid == global_launcher_tgid)))
				task_add_ux_type(task, UX_TYPE_ANIMATOR);
		} else {
			task_clr_ux_type(task, UX_TYPE_ANIMATOR);
		}
	} else {
		if (task && ((current->tgid == global_launcher_tgid &&
				task->group_leader->prio < MAX_RT_PRIO) ||
				(current->group_leader->prio < MAX_RT_PRIO &&
				task->tgid == global_launcher_tgid)))
			task_add_ux_type(task, UX_TYPE_ANIMATOR);
		else
			task_clr_ux_type(task, UX_TYPE_ANIMATOR);
	}
}
EXPORT_SYMBOL(binder_ux_type_set);

bool lock_inherit_ux_type(struct task_struct *owner, struct task_struct *waiter, char* lock_name) {
	struct moto_task_struct *owner_wts;
	struct moto_task_struct *waiter_wts;
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner || !waiter || owner->prio <= 100 ||
		task_get_ux_depth(waiter) >= UX_DEPTH_MAX || task_is_important_ux(owner)) {
		return false;
	}

	rq = task_rq_lock(owner, &flags);
	owner_wts = (struct moto_task_struct *) owner->android_oem_data1;
	waiter_wts = (struct moto_task_struct *) waiter->android_oem_data1;
	if (!owner_wts || !waiter_wts) {
		task_rq_unlock(rq, owner, &flags);
		printk(KERN_ERR "lock_inherit_ux_type moto_task_struct is null!");
		return false;
	}

#ifdef LOCK_DEBUG
	atomic_inc(&g_lock_count);
	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"lock_inherit_ux_type %s %d -> %d   ux_type %d -> %d  depth=%d total=%d\n", lock_name, waiter->pid,
			owner->pid, waiter_wts->ux_type, owner_wts->ux_type, waiter_wts->inherit_depth, atomic_read(&g_lock_count));
#endif
	owner_wts->inherit_mvp_prio = task_get_mvp_prio(waiter, true);

	owner_wts->inherit_start = jiffies_to_nsecs(jiffies);
	owner_wts->inherit_depth = task_get_ux_depth(waiter) + 1;

	task_rq_unlock(rq, owner, &flags);
	return true;
}

bool lock_clear_inherited_ux_type(struct task_struct *owner, char* lock_name) {
	struct moto_task_struct *owner_wts;
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner) {
		return false;
	}
	owner_wts = get_moto_task_struct(owner);
	if (owner_wts->inherit_mvp_prio <= 0) {
		return false;
	}

	rq = task_rq_lock(owner, &flags);
	owner_wts = get_moto_task_struct(owner);
#ifdef LOCK_DEBUG
	atomic_dec(&g_lock_count);
	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"lock_clear_inherited_ux_type %s  %d  ux_type %d -> %d  cost=%llu total=%d\n", lock_name,
			owner->pid, owner_wts->ux_type, owner_wts->inherit_mvp_prio,
			(jiffies_to_nsecs(jiffies) - owner_wts->inherit_start) / 1000000U, atomic_read(&g_lock_count));
#endif
	owner_wts->inherit_mvp_prio = UX_PRIO_INVALID;
	owner_wts->inherit_depth = 0;

	task_rq_unlock(rq, owner, &flags);
	return true;
}

static void android_vh_dup_task_struct(void *unused, struct task_struct *tsk, struct task_struct *orig)
{
	int ux_type = task_get_ux_type(orig);
	if (ux_type & (UX_TYPE_AUDIOSERVICE|UX_TYPE_NATIVESERVICE|UX_TYPE_CAMERASERVICE)) {
		task_add_ux_type(tsk, ux_type);
	}
}

void register_vendor_comm_hooks(void)
{
	register_trace_android_vh_dup_task_struct(android_vh_dup_task_struct, NULL);
}