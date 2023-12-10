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
#define UX_TYPE_INHERIT				(1 << 9)


/* define for UX scene type, keep same as the define in java file */
#define UX_SCENE_LAUNCH				(1 << 0)
#define UX_SCENE_SCROLL				(1 << 1)


extern int moto_sched_enabled;
extern int moto_sched_scene;

#endif /* _MOTO_SCHED_COMMON_H_ */
