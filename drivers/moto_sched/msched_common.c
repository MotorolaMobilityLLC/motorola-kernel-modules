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

#define pr_fmt(fmt) "moto_sched: " fmt

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
#include <trace/hooks/binder.h>
#include <kernel/sched/sched.h>

#include "msched_common.h"
#include "locking/locking_main.h"
#include "locking/locking_trace.h"
#define CREATE_TRACE_POINTS
#include "msched_trace.h"
#include "msched_uclamp.h"

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
	// don't boost camera when audio active
	} else if (is_enabled(UX_ENABLE_CAMERA) && is_scene(UX_SCENE_CAMERA)
		&& (p->tgid == global_camera_tgid || ux_type & UX_TYPE_CAMERASERVICE)
		&& p->prio < 120) {
		return true;
	}

	if (is_enabled(UX_ENABLE_KERNEL) && (ux_type & UX_TYPE_KERNEL))
		return true;

	if (is_heavy_scene()) {
		// audio client app
		if (is_enabled(UX_ENABLE_AUDIO) && is_scene(UX_SCENE_AUDIO)
			&& p->tgid == global_audioapp_tgid) {
			return true;
		}

		if (ux_type & UX_TYPE_NATIVESERVICE && p->prio <= 120)
			return true;

		// all high priority kernel threads
		if (p->mm == NULL && p->prio == 100)
			return true;

		if (p->tgid == global_launcher_tgid
				|| p->tgid == global_systemserver_tgid
				|| p->tgid == global_sysui_tgid
				|| p->tgid == global_sf_tgid
				|| task_in_top_app_group(p))
			return true;
	}

	// always boost top app's high prio threads.
	if(p->prio <= moto_boost_prio && task_in_top_app_group(p))
		return true;

	return false;
}

void task_ux_type_set(int pid, int ux_type) {
	struct task_struct *ux_task = NULL;
	static DEFINE_MUTEX(ux_mutex);

	mutex_lock(&ux_mutex);
	rcu_read_lock();
	ux_task = find_task_by_vpid(pid);
	if (ux_task)
		get_task_struct(ux_task);
	rcu_read_unlock();

	if (ux_task) {
		if (ux_type & UX_TYPE_PERF_DAEMON) {
			// perf daemon is in systemserver, so use its tgid.
			global_systemserver_tgid = ux_task->tgid;
		} else if (ux_type & UX_TYPE_LAUNCHER) {
			global_launcher_tgid = ux_task->tgid;
		} else if (ux_type & UX_TYPE_SYSUI) {
			global_sysui_tgid = ux_task->tgid;
		} else if (ux_type & UX_TYPE_SF) {
			global_sf_tgid = ux_task->tgid;
		} else if (ux_type & UX_TYPE_AUDIOAPP) {
			global_audioapp_tgid = ux_task->tgid;
		} else if (ux_type & UX_TYPE_CAMERAAPP) {
			global_camera_tgid = ux_task->tgid;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		} else if (ux_type & UX_TYPE_IO_PRIO_1) {
			set_task_ioprio(ux_task, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_RT, IOPRIO_NORM)); // use rt-4 for UX_TYPE_IO_PRIO_1
		} else if (ux_type & UX_TYPE_IO_PRIO_2) {
			set_task_ioprio(ux_task, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_BE, 0)); // use be-0 for UX_TYPE_IO_PRIO_2
#endif
		}
		task_add_ux_type(ux_task, ux_type);
		put_task_struct(ux_task);

		cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE) && ux_type != UX_TYPE_SYSTEM_LOCK),
				"set ux_type %d to %d\n", ux_type, ux_task->pid);
	}
	mutex_unlock(&ux_mutex);
}

void task_ux_type_clear(int pid, int ux_type) {
	struct task_struct *ux_task = NULL;
	static DEFINE_MUTEX(ux_mutex);

	mutex_lock(&ux_mutex);
	rcu_read_lock();
	ux_task = find_task_by_vpid(pid);
	if (ux_task)
		get_task_struct(ux_task);
	rcu_read_unlock();

	if (ux_task) {
		if (ux_type & UX_TYPE_AUDIOAPP && global_audioapp_tgid == ux_task->tgid) {
			global_audioapp_tgid = -1;
		} else if (ux_type & UX_TYPE_CAMERAAPP) {
			global_camera_tgid = -1;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		} else if (ux_type & (UX_TYPE_IO_PRIO_1|UX_TYPE_IO_PRIO_2)) {
			set_task_ioprio(ux_task, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_BE, IOPRIO_BE_NORM));
#endif
		}
		task_clr_ux_type(ux_task, ux_type);
		put_task_struct(ux_task);

		cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE) && ux_type != UX_TYPE_SYSTEM_LOCK),
				"clr ux_type %d from %d\n", ux_type, ux_task->pid);
	}
	mutex_unlock(&ux_mutex);
}

int task_get_mvp_prio(struct task_struct *p, bool with_inherit)
{
	int ux_type = task_get_ux_type(p);
	int prio = UX_PRIO_INVALID;

	if (p->prio < 100)
		return UX_PRIO_INVALID;

	// perf daemon
	if (ux_type & UX_TYPE_PERF_DAEMON)
		prio = UX_PRIO_HIGHEST;
	// high priority audio
	else if (is_enabled(UX_ENABLE_AUDIO) && (ux_type & UX_TYPE_AUDIO
				|| ((ux_type & UX_TYPE_AUDIOSERVICE) && ((p->prio == 101 || p->prio == 104)
					|| (is_heavy_scene() && is_scene(UX_SCENE_AUDIO) && p->prio <=120)))))
		prio = UX_PRIO_AUDIO;
	// input & animation & low latency binder
	else if (ux_type & (UX_TYPE_INPUT|UX_TYPE_ANIMATOR|UX_TYPE_LOW_LATENCY_BINDER|UX_TYPE_GESTURE_MONITOR))
		prio = UX_PRIO_ANIMATOR;
	// main & render thread of top app, launcher and top UI.
	else if (ux_type & (UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI) || p->pid == atomic_read(&global_boost_pid))
		prio = UX_PRIO_TOPAPP;
	else if (is_enabled(UX_ENABLE_KSWAPD) && (ux_type & UX_TYPE_KSWAPD))
		prio = UX_PRIO_KSWAPD;
	else if (ux_type & (UX_TYPE_MDPF))
		prio = UX_PRIO_MDPF;
	// system lock & service mgr
	else if ((ux_type & (UX_TYPE_SYSTEM_LOCK|UX_TYPE_SERVICEMANAGER)) || (p->tgid == global_systemserver_tgid && p->prio == 105) )
		prio = UX_PRIO_SYSTEM;
	// inherit lock & binder
	else if (with_inherit && (ux_type & (UX_TYPE_INHERIT_BINDER|UX_TYPE_INHERIT_LOCK)))
		prio = UX_PRIO_OTHER;
	// others high & others low but small tasks.
	else if (task_in_ux_related_group(p) && (p->prio <= moto_boost_prio || moto_task_util(p) < moto_boost_task_util))
		prio = UX_PRIO_OTHER;

	cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
		"pid=%d tgid=%d prio=%d scene=%d ux_type=%d task_util=%lu mvp_prio=%d\n",
		p->pid, p->tgid, p->prio, moto_sched_scene, ux_type, moto_task_util(p), prio);

        if (trace_msched_task_get_mvp_prio_enabled()) {
	    trace_msched_task_get_mvp_prio(p, ux_type, prio, moto_task_util(p), moto_sched_scene);
	}
	return prio;
}
EXPORT_SYMBOL(task_get_mvp_prio);

#define TOPAPP_MVP_LIMIT		120000000U	// 120ms
#define TOPAPP_MVP_LIMIT_BOOST	3000000000U	// 3000ms
#define SYSTEM_MVP_LIMIT		36000000U	// 36ms
#define SYSTEM_MVP_LIMIT_BOOST	3000000000U	// 3000ms
#define RTG_MVP_LIMIT			24000000U	// 24ms
#define RTG_MVP_LIMIT_BOOST		3000000000U	// 3000ms
#define KSWAPD_LIMIT			3000000000U	// 3000ms
#define CAMERA_LIMIT			3000000000U	// 3000ms
#define DEF_MVP_LIMIT			12000000U	// 12ms
#define DEF_MVP_LIMIT_BOOST		3000000000U	// 3000ms

static inline bool task_in_top_related_group(struct task_struct *p) {
	return p->tgid == global_launcher_tgid
			|| p->tgid == global_sysui_tgid
			|| p->tgid == global_systemserver_tgid
			|| p->tgid == atomic_read(&global_boost_pid)
			|| task_in_top_app_group(p);
}

static inline bool task_is_animator(struct task_struct *p) {
	return task_has_ux_type(p, UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI|UX_TYPE_ANIMATOR);
}

unsigned int task_get_mvp_limit(struct task_struct *p, int mvp_prio) {
	bool boost = is_scene(UX_SCENE_LAUNCH)
			|| (is_enabled(UX_ENABLE_BOOST) && is_scene(UX_SCENE_BOOST));

	if (mvp_prio == UX_PRIO_TOPAPP|| mvp_prio == UX_PRIO_MDPF)
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
	msched_uclamp_binder_set_priority_hook(task);
}
EXPORT_SYMBOL(binder_inherit_ux_type);

void binder_clear_inherited_ux_type(struct task_struct *task) {
	if (is_enabled(UX_ENABLE_BINDER)) {
		task_clr_ux_type(task, UX_TYPE_INHERIT_BINDER);
	}
	msched_uclamp_binder_restore_priority_hook(task);
}
EXPORT_SYMBOL(binder_clear_inherited_ux_type);

void queue_ux_task(struct rq *rq, struct task_struct *task, int enqueue) {
	if (!task) {
		return;
	}
	if (is_enabled(UX_ENABLE_LOCK) && !enqueue){
		struct moto_task_struct *mts = get_moto_task_struct(task);
		if (IS_ERR_OR_NULL(mts))
			return;
		if (task_has_ux_type(task, UX_TYPE_INHERIT_LOCK)) {
			if (jiffies_to_nsecs(jiffies) - mts->inherit_start > MAX_INHERIT_GRAN) {
				cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
						"lock_clear_inherited_ux_type %s  %d  ux_type %d  cost=%llu\n", "dequeue task",
						task->pid, mts->ux_type, (jiffies_to_nsecs(jiffies) - mts->inherit_start) / 1000000U);
				task_clr_inherit_type(task);
			}
		}
		if (task_has_ux_type(task, UX_TYPE_KERNEL)) {
			if (jiffies_to_nsecs(jiffies) - mts->boost_kernel_start > MAX_INHERIT_GRAN) {
				cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
						"lock_clear kernel boost %s  %d  ux_type %d  cost=%llu\n", "dequeue task",
						task->pid, mts->ux_type, (jiffies_to_nsecs(jiffies) - mts->inherit_start) / 1000000U);
				mts->boost_kernel_lock_depth = 0;
				mts->boost_kernel_start = -1;
				task_clr_ux_type(task, UX_TYPE_KERNEL);
			}
		}
	}
}
EXPORT_SYMBOL(queue_ux_task);

void binder_ux_type_set(struct task_struct *task) {
	if (!task) {
		return;
	}
	// Base feature: low latency binder
	if (task && ((task_in_top_related_group(current) && task->group_leader->prio < MAX_RT_PRIO)
					|| (current->group_leader->prio < MAX_RT_PRIO && task_in_top_related_group(task))
					|| (current->group_leader->prio < MAX_RT_PRIO && task->group_leader->prio < MAX_RT_PRIO)
					|| (task_in_top_related_group(current) && task_in_top_related_group(task))))
		task_add_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);
	else
		task_clr_ux_type(task, UX_TYPE_LOW_LATENCY_BINDER);

	cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
			"current (tgid=%d leader_prio=%d) task (tgid=%d leader_prio=%d) ux_type=%d\n",
			current->tgid, current->group_leader->prio, task->tgid, task->group_leader->prio, task_get_ux_type(task));
}
EXPORT_SYMBOL(binder_ux_type_set);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void android_rvh_set_user_nice(void *ignore, struct task_struct *p, long *nice)
{
	struct moto_task_struct *mts = get_moto_task_struct(p);
	if (IS_ERR_OR_NULL(mts) || !nice || !p)
		return;

	if ((*nice < MIN_NICE || *nice > MAX_NICE) && !(*nice == 0xbeef || *nice == 0xbeee)) {
		/* ADDED: Trace the disallowed request before returning */
		trace_set_user_nice(p, *nice, false);
		return;
	}

	if (!(*nice == 0xbeef || *nice == 0xbeee))
		mts->nice_backup = *nice;

	if (task_has_ux_type(p, UX_TYPE_INHERIT_LOCK) && (*nice != 0xbeee)) {
		unsigned long rlimit = task_rlimit(p, RLIMIT_NICE);
		if (rlimit != 0) {
			*nice = rlimit_to_nice(rlimit);
			if (unlikely(*nice > MAX_NICE)) {
				pr_warn("%s: pid=%d RLIMIT_NICE=%ld is not set\n", "moto_sched", p->pid, *nice);
				*nice = mts->nice_backup;
			}
		} else
			*nice = mts->nice_backup;
	} else
		*nice = mts->nice_backup;

	/* ADDED: Trace the final values before the function exits */
	trace_set_user_nice(p, *nice, true);
}
#else
static void android_rvh_set_user_nice(void *ignore, struct task_struct *p, long *nice, bool *allowed)
{
	struct moto_task_struct *mts = get_moto_task_struct(p);
	if (IS_ERR_OR_NULL(mts) || !nice || !p || !allowed)
		return;

	if ((*nice < MIN_NICE || *nice > MAX_NICE) && !(*nice == 0xbeef || *nice == 0xbeee)) {
		*allowed = false;
		/* ADDED: Trace the disallowed request before returning */
		trace_set_user_nice(p, *nice, *allowed);
		return;
	} else
		*allowed = true;

	if (!(*nice == 0xbeef || *nice == 0xbeee))
		mts->nice_backup = *nice;

	if (task_has_ux_type(p, UX_TYPE_INHERIT_LOCK) && (*nice != 0xbeee)) {
		unsigned long rlimit = task_rlimit(p, RLIMIT_NICE);
		if (rlimit != 0) {
			*nice = rlimit_to_nice(rlimit);
			if (unlikely(*nice > MAX_NICE)) {
				pr_warn("%s: pid=%d RLIMIT_NICE=%ld is not set\n", "moto_sched", p->pid, *nice);
				*nice = mts->nice_backup;
			}
		} else
			*nice = mts->nice_backup;
	} else
		*nice = mts->nice_backup;

	/* ADDED: Trace the final values before the function exits */
	trace_set_user_nice(p, *nice, *allowed);
}
#endif

bool lock_inherit_ux_type(struct task_struct *owner, struct task_struct *waiter, char* lock_name) {
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner || !waiter) {
		/* UPDATED: Replaced cond_trace_printk with the new generic trace event */
		trace_locking_debug_trace(NULL, __func__, "empty_owner_or_waiter", 0, 0);
		return false;
	}

	if (task_get_ux_depth(waiter) >= UX_DEPTH_MAX) {
		/* UPDATED: Replaced cond_trace_printk */
		trace_locking_debug_trace(NULL, __func__, "max_depth_reached", waiter->pid, owner->pid);
		return false;
	}

	/* ADDED: Trace event for lock inheritance start */
	trace_lock_pi_start(owner, waiter, lock_name);

	rq = task_rq_lock(owner, &flags);

	task_set_ux_inherit_prio(owner, task_get_ux_depth(waiter) + 1);
	/*
	 * UPDATED: Replaced the main debug printk with the trace event.
	 * We log the waiter's and owner's original ux_type for context.
	 */
	// trace_locking_debug_trace(NULL, __func__, lock_name,
	//	waiter_wts->ux_type, owner_wts->ux_type);
	task_rq_unlock(rq, owner, &flags);

	if (fair_policy(owner->policy)) {
		set_user_nice(owner, 0xbeef); // trigger requeue even if task is already in queue
	}

	return true;
}

bool lock_clear_inherited_ux_type(struct task_struct *owner, char* lock_name) {
	struct moto_task_struct *owner_mts;
	struct rq *rq = NULL;
	struct rq_flags flags;

	if (!owner) {
		return false;
	}
	owner_mts = get_moto_task_struct(owner);
	if (IS_ERR_OR_NULL(owner_mts))
		return false;

	if (!task_has_ux_type(owner, UX_TYPE_INHERIT_LOCK)) {
		return false;
	}

	/* ADDED: Trace event for lock inheritance clear */
	trace_lock_pi_finish(owner, lock_name);

	rq = task_rq_lock(owner, &flags);
	cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
			"lock_clear_inherited_ux_type %s  %d  ux_type %d cost=%llu\n", lock_name,
			owner->pid, owner_mts->ux_type,
			(jiffies_to_nsecs(jiffies) - owner_mts->inherit_start) / 1000000U);
	task_clr_inherit_type(owner);

	task_rq_unlock(rq, owner, &flags);

	if (fair_policy(owner->policy)) {
		set_user_nice(owner, 0xbeee); // trigger requeue even if task is already in queue
	}

	return true;
}

void lock_protect_update_starttime(struct task_struct *tsk, unsigned long settime_jiffies, char* lock_name, void *pointer) {
	struct moto_task_struct *waiter_mts = get_moto_task_struct(tsk);
	if (unlikely(!locking_opt_enable()) || IS_ERR_OR_NULL(waiter_mts) || !tsk)
		return;

	if (unlikely(is_debuggable(DEBUG_LOCK))) {
		if (settime_jiffies == 0) {
			if (waiter_mts->boost_kernel_lock_depth == 0) {
				printk(KERN_ERR "LOCK_PERF(%s)kernel boost mismatch(%d)!!", lock_name, waiter_mts->boost_kernel_lock_depth);
			}
			if (task_has_ux_type(tsk,UX_TYPE_KERNEL)) {
				u64 sleep = (jiffies_to_nsecs(jiffies) - waiter_mts->boost_kernel_start) / 1000000U;
				if (sleep > 40) {
					cond_trace_printk(true,
							"(%s) too long prio=%d locked=%d cost=%llu\n", lock_name, tsk->prio, waiter_mts->boost_kernel_lock_depth, sleep);
				}
				if (sleep > 100) {
					printk(KERN_ERR "LOCK_PERF (%s) running too long prio=%d locked=%d cost=%llu", lock_name, tsk->prio, waiter_mts->boost_kernel_lock_depth, sleep);
				}
				if (sleep > 500) {
					dump_stack();
				}
			} else {
				printk(KERN_ERR "LOCK_PERF rwsem didn't boost!!!");
			}
		} else {
			if (waiter_mts->boost_kernel_lock_depth > 32) {
				cond_trace_printk(true,
					"(%s)kernel boost mismatch(%d)!!", lock_name, waiter_mts->boost_kernel_lock_depth);
			}
		}
	}

	if (settime_jiffies > 0) {
		if (waiter_mts->boost_kernel_lock_depth == 0) {
			task_add_ux_type(tsk, UX_TYPE_KERNEL);
			waiter_mts->boost_kernel_start = jiffies_to_nsecs(jiffies);
		}
		waiter_mts->boost_kernel_lock_depth++;
	} else {
		waiter_mts->boost_kernel_lock_depth--;
		if (waiter_mts->boost_kernel_lock_depth < 0) {
			cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
					"(%s)kernel boost mismatch(%d)!!", lock_name, waiter_mts->boost_kernel_lock_depth);
			waiter_mts->boost_kernel_lock_depth = 0;
		}
		if (waiter_mts->boost_kernel_lock_depth == 0) {
			task_clr_ux_type(tsk, UX_TYPE_KERNEL);
		}
	}
}

#if (LINUX_VERSION_CODE == KERNEL_VERSION(5, 10, 0))
static void probe_android_vh_binder_priority_skip(void *ignore, struct task_struct *task,
							bool *skip)
{
	int policy = task->policy;
	if (policy == SCHED_FIFO || policy == SCHED_RR) {
	    if (task->pid == global_sf_tgid) {
		*skip = true;
	    }
	}
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static void android_vh_binder_proc_transaction_finish(void *unused, struct binder_proc *proc,
		struct binder_transaction *t, struct task_struct *task, bool pending_async, bool sync)
{
	if (current == task)
		return;

	if (!pending_async && task) {
		binder_ux_type_set(task);
	}
}
#endif

void register_vendor_comm_hooks(void)
{
	register_trace_android_rvh_set_user_nice(android_rvh_set_user_nice, NULL);

#if (LINUX_VERSION_CODE == KERNEL_VERSION(5, 10, 0))
	register_trace_android_vh_binder_priority_skip(probe_android_vh_binder_priority_skip, NULL);
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	register_trace_android_vh_binder_proc_transaction_finish(
		android_vh_binder_proc_transaction_finish, NULL);
#endif

	msched_uclamp_register_vendor_comm_hooks();
}
