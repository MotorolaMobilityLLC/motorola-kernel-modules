/*
 * Copyright (c) 2025 Motorola Inc.
 */

#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#include <linux/sched/cputime.h>
#endif
#include <trace/hooks/sched.h>
#include <trace/hooks/binder.h>
#include <kernel/sched/sched.h>
#include <kernel/sched/autogroup.h>
#include <drivers/android/binder_internal.h>

#include "msched_uclamp.h"
#include "msched_common.h"
#include "msched_sysfs.h"
#include "msched_trace.h"

static inline struct uclamp_se
uclamp_tg_restrict_moto(struct task_struct *p, enum uclamp_id clamp_id) {
	struct uclamp_se uc_req = p->uclamp_req[clamp_id];
	struct moto_task_struct *vp = get_moto_task_struct(p);
	struct moto_task_struct *vbinder = get_moto_task_struct(p);

	if (IS_ERR_OR_NULL(vp) || IS_ERR_OR_NULL(vbinder))
		return uc_req;

#if IS_ENABLED(CONFIG_UCLAMP_TASK_GROUP)
	unsigned int tg_min, tg_max, value;

	// Task group restriction
	/*
	 * Tasks in autogroups or root task group should have uclamp_min is 0.
	 * uclamp_min is defined as a LIMIT rather than a PROTECTION at that
	 * level.
	 */
	if (task_group_is_autogroup(task_group(p)))
		tg_min = 0;
	else if (task_group(p) == &root_task_group)
		tg_min = 0;
	else
		tg_min = task_group(p)->uclamp[UCLAMP_MIN].value;
	tg_max = task_group(p)->uclamp[UCLAMP_MAX].value;

	value = uc_req.value;
	value = clamp(value, tg_min, tg_max);

	// RT_mutex inherited uclamp restriction
	value = clamp_t(unsigned int, value, vp->uclamp_pi[UCLAMP_MIN], vp->uclamp_pi[UCLAMP_MAX]);

	// Inherited uclamp restriction
	if (vbinder->uclamp_active) {
		value = clamp_t(unsigned int, value, vbinder->uclamp[UCLAMP_MIN], vbinder->uclamp[UCLAMP_MAX]);
	}

	uc_req.value = value;
	uc_req.bucket_id = get_bucket_id(value);
#endif

	return uc_req;
}

void rvh_uclamp_eff_get(void *data, struct task_struct *p, enum uclamp_id clamp_id,
				  struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
				  int *ret) {
	struct uclamp_se uc_req;
	struct moto_task_struct *mts = get_moto_task_struct(p);
	if (IS_ERR_OR_NULL(mts))
			return;

	if(is_enabled(UX_ENABLE_MDPF)) {
		*ret = 1;

		uc_req = uclamp_tg_restrict_moto(p, clamp_id);

		/* System default restrictions always apply */
		if (unlikely(uc_req.value > uclamp_max->value)) {
			*uclamp_eff = *uclamp_max;

			if (trace_msched_uclamp_restriction_result_enabled()) {
				trace_msched_uclamp_restriction_result(
					p->pid, clamp_id,
					p->uclamp_req[clamp_id].value,
					uclamp_eff->value,
					uclamp_max->value,
					(task_group_is_autogroup(task_group(p)) || task_group(p) == &root_task_group) ? 0 :
						task_group(p)->uclamp[UCLAMP_MIN].value,
					task_group(p)->uclamp[UCLAMP_MAX].value,
					mts->uclamp_pi[UCLAMP_MIN],
					mts->uclamp_pi[UCLAMP_MAX],
					mts->uclamp[UCLAMP_MIN],
					mts->uclamp[UCLAMP_MAX]);
			}

			return;
		}

		*uclamp_eff = uc_req;

		if (trace_msched_uclamp_restriction_result_enabled()) {
			trace_msched_uclamp_restriction_result(
				p->pid, clamp_id,
				p->uclamp_req[clamp_id].value,
				uclamp_eff->value,
				uclamp_max->value,
				(task_group_is_autogroup(task_group(p)) || task_group(p) == &root_task_group) ? 0 :
					task_group(p)->uclamp[UCLAMP_MIN].value,
				task_group(p)->uclamp[UCLAMP_MAX].value,
				mts->uclamp_pi[UCLAMP_MIN],
				mts->uclamp_pi[UCLAMP_MAX],
				mts->uclamp[UCLAMP_MIN],
				mts->uclamp[UCLAMP_MAX]);
		}

	}
	*ret = 0;
}

static inline unsigned int
uclamp_eff_value_moto(struct task_struct *p, enum uclamp_id clamp_id) {
	struct uclamp_se uc_max = {};
	struct uclamp_se uc_eff;
	int ret;

	uc_max.value = uclamp_none(UCLAMP_MAX);
	uc_max.bucket_id = get_bucket_id(uc_max.value);
	uc_max.user_defined = false;

	/* Task currently refcounted: use back-annotated (effective) value */
	if (p->uclamp[clamp_id].active)
		return (unsigned int)p->uclamp[clamp_id].value;

	// This function will always return uc_eff
	rvh_uclamp_eff_get(NULL, p, clamp_id, &uc_max, &uc_eff, &ret);

	return (unsigned int)uc_eff.value;
}

void set_uclamp_inheritance(struct task_struct *p, struct task_struct *pi_task,
	u16 *uclamp_i, unsigned int type) {
	struct rq_flags rf;
	struct rq *rq;
	unsigned long p_util, pi_util;
	u16 p_uclamp_min, p_uclamp_max;
	u16 pi_uclamp_min, pi_uclamp_max;

	if(type == VENDOR_INHERITANCE_RTMUTEX) {
		rq = __task_rq_lock(p, & rf);
		lockdep_assert_held(&p->pi_lock);
	} else
		rq = task_rq_lock(p, &rf);
	p_util = moto_task_util(p);
	p_uclamp_min = uclamp_eff_value_moto(p, UCLAMP_MIN);
	p_uclamp_max = uclamp_eff_value_moto(p, UCLAMP_MAX);
	if (pi_task) {
		pi_util = moto_task_util(pi_task);
		pi_uclamp_min = uclamp_eff_value_moto(pi_task, UCLAMP_MIN);
		pi_uclamp_max = uclamp_eff_value_moto(pi_task, UCLAMP_MAX);

		/*
		 * Take task's util into consideration first to do full
		 * performance inheritance.
		 *
		 * If pi_uclamp_min = 612 but pi_util is 812, then setting
		 * p_uclamp_min to 612 is not enough as the task will still run
		 * slower.
		 *
		 * Or if pi_uclamp_min is 0 but pi_util is 800 while p_util is
		 * 100, then pi_task could wait for longer to acquire the lock
		 * because the performance of p is too low.
		 */
		p_uclamp_min = clamp_t(unsigned long, p_util, p_uclamp_min, p_uclamp_max);
		pi_uclamp_min = clamp_t(unsigned long, pi_util, pi_uclamp_min, pi_uclamp_max);

		/* Inherit unclamp_min/max if they're inverted */

		if (p_uclamp_min < pi_uclamp_min)
			uclamp_i[UCLAMP_MIN] = pi_uclamp_min;

		if (p_uclamp_max < pi_uclamp_max || pi_uclamp_min > p_uclamp_max)
			uclamp_i[UCLAMP_MAX] = pi_uclamp_max;

		if (trace_msched_uclamp_inheritance_result_enabled()) {
			trace_msched_uclamp_inheritance_result(
				p->pid, pi_task->pid,
				p_util, p_uclamp_min, p_uclamp_max,
				pi_util, pi_uclamp_min, pi_uclamp_max,
				uclamp_i[UCLAMP_MIN], uclamp_i[UCLAMP_MAX], type);
		}
	} else {
		uclamp_i[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
		uclamp_i[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
	}
	if(type == VENDOR_INHERITANCE_RTMUTEX)
		__task_rq_unlock(rq, &rf);
	else
		task_rq_unlock(rq, p, &rf);
}

void msched_uclamp_vh_dup_task_struct(void *unused, struct task_struct *task, struct task_struct *orig) {
	enum uclamp_id clamp_id;

	if (is_enabled(UX_ENABLE_MDPF)) {
		int ux_type = task_get_ux_type(orig);
		if(ux_type & UX_TYPE_MDPF) {
			/* reset uclamp min value of new forked task */
			cond_trace_printk(unlikely(is_debuggable(DEBUG_MDPF)),
				"reset uclamp min[%d,%d] of task(tgid-%d pid-%d util-%lu) which is forked from MDPF task(tgid-%d pid-%d util-%lu)!\n",
				task->uclamp[UCLAMP_MIN].value, task->uclamp_req[UCLAMP_MIN].value,
				task->tgid, task->pid, moto_task_util(task),
				orig->tgid, orig->pid, moto_task_util(orig));

			for_each_clamp_id(clamp_id) {
				uclamp_se_set(&task->uclamp_req[clamp_id],
						uclamp_none(clamp_id), false);
			}
		}
	}
}

void msched_uclamp_binder_set_priority_hook(struct task_struct *task) {
	if (current == task)
		return;

	if (is_enabled(UX_ENABLE_MDPF)) {
		struct moto_task_struct *mts = get_moto_task_struct(task);
		if (IS_ERR_OR_NULL(mts))
			return;
		if(!mts->uclamp_active) {
			mts->uclamp_active = true;
			/* inherit uclamp */
			set_uclamp_inheritance(task, current, mts->uclamp, VENDOR_INHERITANCE_BINDER);
		}
	}
}

void msched_uclamp_binder_restore_priority_hook(struct task_struct *task) {
	if (current == task)
		return;

	if (is_enabled(UX_ENABLE_MDPF)) {
		struct moto_task_struct *mts = get_moto_task_struct(task);
		if (IS_ERR_OR_NULL(mts))
			return;
		if (mts->uclamp_active) {
			set_uclamp_inheritance(task, NULL, mts->uclamp, VENDOR_INHERITANCE_BINDER);
			mts->uclamp_active = false;
		}
	}
}

static void rvh_rtmutex_prepare_setprio(void *data, struct task_struct *p, struct task_struct *pi_task) {
	if(is_enabled(UX_ENABLE_MDPF)) {
		struct moto_task_struct *mts = get_moto_task_struct(p);
		if (IS_ERR_OR_NULL(mts))
			return;
		set_uclamp_inheritance(p, pi_task, mts->uclamp_pi, VENDOR_INHERITANCE_RTMUTEX);
	}
}

static void rvh_sched_fork_init(void *unused, struct task_struct *p) {
	struct moto_task_struct *mts = get_moto_task_struct(p);
	if (IS_ERR_OR_NULL(mts))
			return;
	mts->uclamp[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
	mts->uclamp[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
	mts->uclamp_pi[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
	mts->uclamp_pi[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
	mts->uclamp_active = false;
}

int msched_uclamp_register_vendor_comm_hooks(void)
{
	register_trace_android_rvh_rtmutex_prepare_setprio(rvh_rtmutex_prepare_setprio, NULL);
	register_trace_android_rvh_uclamp_eff_get(rvh_uclamp_eff_get, NULL);
	register_trace_android_rvh_sched_fork_init(rvh_sched_fork_init, NULL);

	return 0;
}

