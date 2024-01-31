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


#ifndef _MOTO_SCHED_COMMON_H_
#define _MOTO_SCHED_COMMON_H_

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/cgroup.h>
#include <linux/hashtable.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <linux/sched/walt.h>
#endif

#define VERION 1003

#define cond_trace_printk(cond, fmt, ...)	\
do {										\
	if (cond)								\
		trace_printk(fmt, ##__VA_ARGS__);	\
} while (0)

#define sched_err(fmt, ...) \
		pr_err("[moto_sched][%s]"fmt, __func__, ##__VA_ARGS__)
#define sched_warn(fmt, ...) \
		pr_warn("[moto_sched][%s]"fmt, __func__, ##__VA_ARGS__)
#define sched_debug(fmt, ...) \
		pr_info("[moto_sched][%s]"fmt, __func__, ##__VA_ARGS__)


/* define for UX thread type, keep same as the define in java file */
#define UX_TYPE_PERF_DAEMON			(1 << 0)
#define UX_TYPE_AUDIO				(1 << 1)
#define UX_TYPE_INPUT				(1 << 2)
#define UX_TYPE_ANIMATOR			(1 << 3)
#define UX_TYPE_TOPAPP				(1 << 4)
#define UX_TYPE_TOPUI				(1 << 5)
#define UX_TYPE_LAUNCHER			(1 << 6)
#define UX_TYPE_KSWAPD				(1 << 7)
#define UX_TYPE_SYSTEM_LOCK			(1 << 8)
#define UX_TYPE_INHERIT_BINDER		(1 << 9)
#define UX_TYPE_LOW_LATENCY_BINDER	(1 << 10)
#define UX_TYPE_GESTURE_MONITOR		(1 << 11)
#define UX_TYPE_SF					(1 << 12)
#define UX_TYPE_AUDIOSERVICE		(1 << 13)
#define UX_TYPE_AUDIOAPP			(1 << 14)
#define UX_TYPE_NATIVESERVICE		(1 << 15)
#define UX_TYPE_CAMERASERVICE		(1 << 16)
#define UX_TYPE_SYSUI				(1 << 17)
#define UX_TYPE_SERVICEMANAGER		(1 << 18)

/* define for UX scene type, keep same as the define in java file */
#define UX_SCENE_LAUNCH				(1 << 0)
#define UX_SCENE_TOUCH				(1 << 1)
#define UX_SCENE_AUDIO				(1 << 2)
#define UX_SCENE_CAMERA				(1 << 3)
#define UX_SCENE_INVALID			-1

/* define for MVP priority, the higher the better, should be in the range (11~100) */
#define UX_PRIO_HIGHEST		100

#define UX_PRIO_AUDIO		80
#define UX_PRIO_ANIMATOR	79
#define UX_PRIO_SYSTEM		78
#define UX_PRIO_TOPAPP		70 // must be aligned with walt.h!
#define UX_PRIO_KSWAPD		65 // must be aligned with walt.h!
#define UX_PRIO_OTHER		60

#define UX_PRIO_LOWEST		11
#define UX_PRIO_INVALID		-1

#define UX_DEPTH_MAX		5

enum {
	CGROUP_RESV = 0,
	CGROUP_DEFAULT = 1,         /* sys */
	CGROUP_FOREGROUND,
	CGROUP_BACKGROUND,
	CGROUP_TOP_APP,

	CGROUP_NRS,
};

#ifdef CONFIG_MOTO_FUTEX_INHERIT
struct locking_info {
	struct task_struct *holder;
	bool ux_contrib;
};
#endif

/* Moto task struct */
struct moto_task_struct {
	int				ux_type;

#ifdef CONFIG_MOTO_FUTEX_INHERIT
	struct locking_info lkinfo;
#endif

	int				inherit_depth;
	int             inherit_mvp_prio;
	u64				inherit_start;
};

/* global vars and functions */
extern int __read_mostly moto_sched_enabled;
extern int __read_mostly moto_sched_debug;
extern int __read_mostly moto_sched_scene;
extern int __read_mostly moto_boost_prio;
extern pid_t __read_mostly global_systemserver_tgid;
extern pid_t __read_mostly global_launcher_tgid;
extern pid_t __read_mostly global_sysui_tgid;
extern pid_t __read_mostly global_sf_tgid;

extern int task_get_origin_mvp_prio(struct task_struct *p, bool with_inherit);
extern int task_get_mvp_prio(struct task_struct *p, bool with_inherit);
extern unsigned int task_get_mvp_limit(struct task_struct *p, int mvp_prio);
extern void binder_inherit_ux_type(struct task_struct *task);
extern void binder_clear_inherited_ux_type(struct task_struct *task);
extern void binder_ux_type_set(struct task_struct *task);
extern void queue_ux_task(struct rq *rq, struct task_struct *task, int enqueue);
extern bool lock_inherit_ux_type(struct task_struct *owner, struct task_struct *waiter, char* lock_name);
extern bool lock_clear_inherited_ux_type(struct task_struct *waiter, char* lock_name);
extern void register_vendor_comm_hooks(void);

static inline struct moto_task_struct *get_moto_task_struct(struct task_struct *p)
{
	return (struct moto_task_struct *) p->android_oem_data1;
}

static inline int task_get_ux_type(struct task_struct *p)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	return wts->ux_type;
}

static inline void task_add_ux_type(struct task_struct *p, int type)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	wts->ux_type |= type;
}

static inline void task_clr_ux_type(struct task_struct *p, int type)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	wts->ux_type &= ~type;
}

static inline int get_task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);
	return css ? css->id : -1;
}

static inline bool task_is_important_ux(struct task_struct *p)
{
	return task_get_mvp_prio(p, true) >= UX_PRIO_OTHER;
}

static inline bool current_is_important_ux(void)
{
	return task_get_mvp_prio(current, true) >= UX_PRIO_OTHER;
}

static inline void task_set_ux_inherit_prio(struct task_struct *p, int prio, int depth)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	wts->inherit_mvp_prio = prio;
	wts->inherit_start = jiffies_to_nsecs(jiffies);
	wts->inherit_depth = depth;
}

static inline int task_get_ux_inherit_prio(struct task_struct *p)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	return wts->inherit_mvp_prio;
}

static inline int task_get_ux_depth(struct task_struct *t)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) t->android_oem_data1;

	return wts->inherit_depth;
}

static inline void task_clr_inherit_type(struct task_struct *p)
{
	struct moto_task_struct *wts = (struct moto_task_struct *) p->android_oem_data1;
	wts->inherit_mvp_prio = UX_PRIO_INVALID;
	wts->inherit_depth = 0;
}

#endif /* _MOTO_SCHED_COMMON_H_ */
