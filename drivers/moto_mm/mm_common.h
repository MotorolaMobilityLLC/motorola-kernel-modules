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

#ifndef _MOTO_MM_COMMON_H_
#define _MOTO_MM_COMMON_H_

#include <linux/list.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/cgroup.h>
#include <linux/version.h>


#define cond_trace_printk(cond, fmt, ...)	\
do {										\
	if (cond)								\
		trace_printk(fmt, ##__VA_ARGS__);	\
} while (0)

#define mm_err(fmt, ...) \
		pr_err("[moto_mm][%s]"fmt, __func__, ##__VA_ARGS__)
#define mm_warn(fmt, ...) \
		pr_warn("[moto_mm][%s]"fmt, __func__, ##__VA_ARGS__)
#define mm_debug(fmt, ...) \
		pr_info("[moto_mm][%s]"fmt, __func__, ##__VA_ARGS__)

#define REGISTER_HOOK(name) do {\
	rc = register_trace_android_vh_##name(name##_hook, NULL);\
	if (rc) {\
		pr_err("register hook %s failed", #name);\
		goto err_out_##name;\
	}\
} while (0)

#define UNREGISTER_HOOK(name) do {\
	unregister_trace_android_vh_##name(name##_hook, NULL);\
} while (0)

#define ERROR_OUT(name) err_out_##name

enum {
	CGROUP_RESV = 0,
	CGROUP_DEFAULT = 1,         /* sys */
	CGROUP_FOREGROUND,
	CGROUP_BACKGROUND,
	CGROUP_TOP_APP,

	CGROUP_NRS,
};

/* global vars and functions */

extern int moto_mm_proc_init(void);
extern void moto_mm_proc_deinit(void);

static inline int get_task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);
	return css ? css->id : -1;
}

static inline bool task_in_top_app_group(struct task_struct *p)
{
	return get_task_cgroup_id(p) == CGROUP_TOP_APP;
}


#endif /* _MOTO_MM_COMMON_H_ */
