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
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#include <linux/sched/cputime.h>
#endif
#include <trace/hooks/sched.h>
#include <trace/hooks/signal.h>
#include <kernel/sched/sched.h>

#include "msched_common.h"
#include "locking/locking_main.h"

#define MS_TO_NS (1000000)
#define MAX_INHERIT_GRAN ((u64)(64 * MS_TO_NS))

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

	if (is_enabled(UX_ENABLE_AUDIO) && is_scene(UX_SCENE_AUDIO)) {
		if (ux_type & UX_TYPE_AUDIOSERVICE && p->prio <= 120)
			return true;
		else if (p->prio == 104 || p->prio == 101)
			return true;
		else if (p->tgid == global_audioapp_tgid)
			return true;
	}

	if (is_enabled(UX_ENABLE_CAMERA) && is_scene(UX_SCENE_CAMERA)
		&& (p->tgid == global_camera_tgid || ux_type & UX_TYPE_CAMERASERVICE)) {
		return true;
	}

	if (is_heavy_scene()) {
		// Boost all kernel threads with prio == 100
		if (p->mm == NULL && p->prio == 100)
			return true;

		if ((ux_type & UX_TYPE_KERNEL) && is_enabled(UX_ENABLE_KERNEL))
			return true;

		if (ux_type & UX_TYPE_NATIVESERVICE && p->prio < 120)
			return true;

		if (p->tgid == global_launcher_tgid
				|| p->tgid == global_systemserver_tgid
				|| p->tgid == global_sysui_tgid
				|| p->tgid == global_sf_tgid
				|| task_in_top_app_group(p))
			return true;
	}

	// Base feature: always boost top app's high prio threads.
	if(p->prio <= moto_boost_prio && task_in_top_app_group(p))
		return true;

	return false;
}

int task_get_mvp_prio(struct task_struct *p, bool with_inherit)
{
	int ux_type = task_get_ux_type(p);
	int prio = UX_PRIO_INVALID;

	if (p->prio < 100)
		return UX_PRIO_INVALID;

	if (ux_type & UX_TYPE_PERF_DAEMON) // Base feature: perf daemon
		prio = UX_PRIO_HIGHEST;
	else if (is_enabled(UX_ENABLE_AUDIO)
	    && (ux_type & UX_TYPE_AUDIO || ((ux_type & UX_TYPE_AUDIOSERVICE) && (p->prio == 101 || p->prio == 104))))
		prio = UX_PRIO_AUDIO;
	else if (ux_type & (UX_TYPE_INPUT|UX_TYPE_ANIMATOR|UX_TYPE_LOW_LATENCY_BINDER)) // Base feature: input & animation & low latency binder
		prio = UX_PRIO_ANIMATOR;
	else if (ux_type & (UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI)) // Base feature: main & render thread of top app, launcher and top UI.
		prio = UX_PRIO_TOPAPP;
	else if (ux_type & (UX_TYPE_SYSTEM_LOCK|UX_TYPE_SERVICEMANAGER)) // Base feature: systemserver important lock
		prio = UX_PRIO_SYSTEM;
	else if (is_enabled(UX_ENABLE_CAMERA) && is_scene(UX_SCENE_CAMERA)
		&& (p->tgid == global_camera_tgid || ux_type & UX_TYPE_CAMERASERVICE)
		&& p->prio <= 120)
		prio = UX_PRIO_CAMERA;
	else if (is_enabled(UX_ENABLE_KSWAPD) && (ux_type & UX_TYPE_KSWAPD))
		prio = UX_PRIO_KSWAPD;
	else if (with_inherit && (ux_type & (UX_TYPE_INHERIT_BINDER|UX_TYPE_INHERIT_LOCK)))
		prio = UX_PRIO_OTHER;
	else if (task_in_ux_related_group(p))
		prio = UX_PRIO_OTHER;

	cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
		"pid=%d tgid=%d prio=%d scene=%d ux_type=%d mvp_prio=%d\n",
		p->pid, p->tgid, p->prio, moto_sched_scene, ux_type, prio);

	return prio;
}
EXPORT_SYMBOL(task_get_mvp_prio);

#define TOPAPP_MVP_LIMIT		120000000U	// 120ms
#define TOPAPP_MVP_LIMIT_BOOST	240000000U	// 240ms
#define SYSTEM_MVP_LIMIT		36000000U	// 36ms
#define SYSTEM_MVP_LIMIT_BOOST	120000000U	// 120ms
#define RTG_MVP_LIMIT			24000000U	// 24ms
#define RTG_MVP_LIMIT_BOOST		120000000U	// 120ms
#define KSWAPD_LIMIT			3000000000U	// 3000ms
#define CAMERA_LIMIT			3000000000U	// 3000ms
#define DEF_MVP_LIMIT			12000000U	// 12ms
#define DEF_MVP_LIMIT_BOOST		24000000U	// 24ms

static inline bool task_in_top_related_group(struct task_struct *p) {
	return p->tgid == global_launcher_tgid
			|| p->tgid == global_sysui_tgid
			|| task_in_top_app_group(p);
}

unsigned int task_get_mvp_limit(struct task_struct *p, int mvp_prio) {
	bool boost = is_heavy_scene();

	if (mvp_prio == UX_PRIO_TOPAPP)
		return boost ? TOPAPP_MVP_LIMIT_BOOST : TOPAPP_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_CAMERA)
		return CAMERA_LIMIT;
	else if (mvp_prio == UX_PRIO_SYSTEM || (p->tgid == global_systemserver_tgid))
		return boost ? SYSTEM_MVP_LIMIT_BOOST : SYSTEM_MVP_LIMIT;
	else if (task_in_top_related_group(p))
		return boost ? RTG_MVP_LIMIT_BOOST : RTG_MVP_LIMIT;
	else if (mvp_prio == UX_PRIO_KSWAPD)
		return KSWAPD_LIMIT;
	else if (mvp_prio > UX_PRIO_INVALID)
		return boost ? DEF_MVP_LIMIT_BOOST : DEF_MVP_LIMIT;

	return 0;
}
EXPORT_SYMBOL(task_get_mvp_limit);

void binder_inherit_ux_type(struct task_struct *task) {
	if (is_enabled(UX_ENABLE_BINDER) && current_is_important_ux()) {
		task_add_ux_type(task, UX_TYPE_INHERIT_BINDER);
	}
}
EXPORT_SYMBOL(binder_inherit_ux_type);

void binder_clear_inherited_ux_type(struct task_struct *task) {
	if (is_enabled(UX_ENABLE_BINDER)) {
		task_clr_ux_type(task, UX_TYPE_INHERIT_BINDER);
	}
}
EXPORT_SYMBOL(binder_clear_inherited_ux_type);

void queue_ux_task(struct rq *rq, struct task_struct *task, int enqueue) {
	if (is_enabled(UX_ENABLE_LOCK) && !enqueue){
		struct moto_task_struct *wts = get_moto_task_struct(task);
		if (task_has_ux_type(task, UX_TYPE_INHERIT_LOCK)) {
			if (jiffies_to_nsecs(jiffies) - wts->inherit_start > MAX_INHERIT_GRAN) {
				cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
						"lock_clear_inherited_ux_type %s  %d  ux_type %d  cost=%llu\n", "dequeue task",
						task->pid, wts->ux_type, (jiffies_to_nsecs(jiffies) - wts->inherit_start) / 1000000U);
				task_clr_inherit_type(task);
			}
		}
		if (task_has_ux_type(task, UX_TYPE_KERNEL)) {
			if (jiffies_to_nsecs(jiffies) - wts->boost_kernel_start > MAX_INHERIT_GRAN) {
				cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
						"lock_clear kernel boost %s  %d  ux_type %d  cost=%llu\n", "dequeue task",
						task->pid, wts->ux_type, (jiffies_to_nsecs(jiffies) - wts->inherit_start) / 1000000U);
				wts->boost_kernel_lock_depth = 0;
				wts->boost_kernel_start = -1;
				task_clr_ux_type(task, UX_TYPE_KERNEL);
			}
		}
	}
}
EXPORT_SYMBOL(queue_ux_task);

void binder_ux_type_set(struct task_struct *task) {
	// Base feature: low latency binder
	if (task && ((task_in_top_related_group(current) && task->group_leader->prio < MAX_RT_PRIO)
					|| (current->group_leader->prio < MAX_RT_PRIO && task_in_top_related_group(task))))
		task_add_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);
	else
		task_clr_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);

	cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
			"current (tgid=%d leader_prio=%d) task (tgid=%d leader_prio=%d) ux_type=%d\n",
			current->tgid, current->group_leader->prio, task->tgid, task->group_leader->prio, task_get_ux_type(task));
}
EXPORT_SYMBOL(binder_ux_type_set);

bool lock_inherit_ux_type(struct task_struct *owner, struct task_struct *waiter, char* lock_name) {
	struct moto_task_struct *owner_wts;
	struct moto_task_struct *waiter_wts;
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner || !waiter) {
		cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
			"lock_inherit_ux_type empty!! %d \n", 0);
		return false;
	}

	if (task_get_ux_depth(waiter) >= UX_DEPTH_MAX) {
		cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
			"lock_inherit_ux_type max depth reached %d->%d\n",
			waiter->pid, owner->pid);
		return false;
	}

	rq = task_rq_lock(owner, &flags);

	owner_wts = (struct moto_task_struct *) owner->android_oem_data1;
	waiter_wts = (struct moto_task_struct *) waiter->android_oem_data1;

	task_set_ux_inherit_prio(owner, task_get_ux_depth(waiter) + 1);

	cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
			"lock_inherit_ux_type %s %d -> %d   ux_type %d -> %d  depth=%d\n",
			lock_name, waiter->pid, owner->pid, waiter_wts->ux_type, owner_wts->ux_type,
			waiter_wts->inherit_depth);

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
	if (!task_has_ux_type(owner, UX_TYPE_INHERIT_LOCK)) {
		return false;
	}

	rq = task_rq_lock(owner, &flags);

	owner_wts = get_moto_task_struct(owner);
	cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
			"lock_clear_inherited_ux_type %s  %d  ux_type %d cost=%llu\n", lock_name,
			owner->pid, owner_wts->ux_type,
			(jiffies_to_nsecs(jiffies) - owner_wts->inherit_start) / 1000000U);
	task_clr_inherit_type(owner);

	task_rq_unlock(rq, owner, &flags);
	return true;
}

void lock_protect_update_starttime(struct task_struct *tsk, unsigned long settime_jiffies, char* lock_name, void *pointer) {
	struct moto_task_struct *waiter_wts = (struct moto_task_struct *) tsk->android_oem_data1;
	if (unlikely(!locking_opt_enable()))
		return;

	if (unlikely(is_debuggable(DEBUG_TYPE_LOCK))) {
		if (settime_jiffies == 0) {
			if (waiter_wts->boost_kernel_lock_depth == 0) {
				printk(KERN_ERR "LOCK_PERF(%s)kernel boost mismatch(%d)!!", lock_name, waiter_wts->boost_kernel_lock_depth);
			}
			if (task_has_ux_type(tsk,UX_TYPE_KERNEL)) {
				u64 sleep = (jiffies_to_nsecs(jiffies) - waiter_wts->boost_kernel_start) / 1000000U;
				if (sleep > 40) {
					cond_trace_printk(true,
							"(%s) too long prio=%d locked=%d cost=%llu\n", lock_name, tsk->prio, waiter_wts->boost_kernel_lock_depth, sleep);
				}
				if (sleep > 100) {
					printk(KERN_ERR "LOCK_PERF (%s) running too long prio=%d locked=%d cost=%llu", lock_name, tsk->prio, waiter_wts->boost_kernel_lock_depth, sleep);
				}
				if (sleep > 500) {
					dump_stack();
				}
			} else {
				printk(KERN_ERR "LOCK_PERF rwsem didn't boost!!!");
			}
		} else {
			if (waiter_wts->boost_kernel_lock_depth > 32) {
				cond_trace_printk(true,
					"(%s)kernel boost mismatch(%d)!!", lock_name, waiter_wts->boost_kernel_lock_depth);
			}
		}
	}

	if (settime_jiffies > 0) {
		if (waiter_wts->boost_kernel_lock_depth == 0) {
			task_add_ux_type(tsk, UX_TYPE_KERNEL);
			waiter_wts->boost_kernel_start = jiffies_to_nsecs(jiffies);
		}
		waiter_wts->boost_kernel_lock_depth++;
	} else {
		waiter_wts->boost_kernel_lock_depth--;
		if (waiter_wts->boost_kernel_lock_depth < 0) {
			cond_trace_printk(unlikely(is_debuggable(DEBUG_TYPE_BASE)),
					"(%s)kernel boost mismatch(%d)!!", lock_name, waiter_wts->boost_kernel_lock_depth);
			waiter_wts->boost_kernel_lock_depth = 0;
		}
		if (waiter_wts->boost_kernel_lock_depth == 0) {
			task_clr_ux_type(tsk, UX_TYPE_KERNEL);
		}
	}
}

static void android_vh_dup_task_struct(void *unused, struct task_struct *task, struct task_struct *orig)
{
	// Base feature: inherit task ux_type during fork for some native services.
	int ux_type = task_get_ux_type(orig);
	if (ux_type & (UX_TYPE_AUDIOSERVICE|UX_TYPE_NATIVESERVICE|UX_TYPE_CAMERASERVICE|UX_TYPE_SERVICEMANAGER)) {
		task_add_ux_type(task, ux_type);
	}
}

void register_vendor_comm_hooks(void)
{
	register_trace_android_vh_dup_task_struct(android_vh_dup_task_struct, NULL);
}
