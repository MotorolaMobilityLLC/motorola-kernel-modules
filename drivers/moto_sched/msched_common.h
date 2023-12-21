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
#include <linux/hashtable.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <linux/sched/walt.h>
#endif


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
#define UX_TYPE_ONCE				(1 << 8) /* clear ux type when dequeue */
#define UX_TYPE_INHERIT_HIGH		(1 << 9)
#define UX_TYPE_INHERIT_LOW			(1 << 10)
#define UX_TYPE_GESTURE_MONITOR		(1 << 11)
#define UX_TYPE_SF					(1 << 12)

/* define for UX scene type, keep same as the define in java file */
#define UX_SCENE_LAUNCH				(1 << 0)
#define UX_SCENE_SCROLL				(1 << 1)
#define UX_SCENE_RINGTONE			(1 << 2)

/* define for MVP priority, the higher the better, should be in the range (11~100) */
#define UX_PRIO_HIGHEST		100

#define UX_PRIO_URGENT_AUDIO	30
#define UX_PRIO_INPUT		29
#define UX_PRIO_ANIMATOR	28
#define UX_PRIO_AUDIO		27
#define UX_PRIO_KSWAPD		26

#define UX_PRIO_TOPAPP		20 // fixed value 20, aligned with walt.h, must not be changed!
#define UX_PRIO_TOPUI		19
#define UX_PRIO_LAUNCHER	18
#define UX_PRIO_TOPAPP_HIGH	17
#define UX_PRIO_SYSTEM_HIGH	16
#define UX_PRIO_TOPAPP_LOW	15
#define UX_PRIO_SYSTEM_LOW	14

#define UX_PRIO_LOWEST		11
#define UX_PRIO_INVALID		-1

#define SYSTEM_UID 1000
#define PHONE_UID 1001
#define MEDIA_UID 1013
#define MEDIA_EX_UID 1040
#define AUDIOSERVER_UID 1041
#define MEDIA_CODEC_UID 1046
#define CAMERASERVER_UID 1047

/* global vars and functions */
extern int moto_sched_enabled;
extern int moto_sched_scene;
extern pid_t global_systemserver_tgid;
extern pid_t global_surfaceflinger_tgid;
extern pid_t global_boost_uid;

extern int task_get_mvp_prio(struct task_struct *p, bool with_inherit);
extern unsigned int task_get_mvp_limit(int mvp_prio);
extern void binder_inherit_ux_type(struct task_struct *task);
extern void binder_clear_inherited_ux_type(struct task_struct *task);


#endif /* _MOTO_SCHED_COMMON_H_ */
