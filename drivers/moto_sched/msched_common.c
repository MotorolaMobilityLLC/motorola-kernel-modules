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

#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "msched_common.h"

static inline bool task_in_audio_group(struct task_struct *p)
{
	int gl_ux_type = task_get_ux_type(p->group_leader);
	if (gl_ux_type & UX_TYPE_AUDIOSERVICE) {
		return p->prio <= 110;
	} else if ((moto_sched_scene & UX_SCENE_AUDIO) && (gl_ux_type & UX_TYPE_AUDIOAPP)
		&& (p->prio == 104 || p->prio == 101)) {
		return true;
	}
	return false;
}

static inline bool task_in_top_app_group(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	return (rcu_access_pointer(wts->grp) != NULL);
#else
	return false; // todo: implement it on mtk platform.
#endif
}

int task_get_mvp_prio(struct task_struct *p, bool with_inherit)
{
	int ux_type = task_get_ux_type(p);

	if (ux_type & UX_TYPE_PERF_DAEMON)
		return UX_PRIO_HIGHEST;
	else if (ux_type & UX_TYPE_AUDIO || task_in_audio_group(p))
		return p->prio <= 101 ? UX_PRIO_URGENT_AUDIO : UX_PRIO_AUDIO;
	else if (ux_type & UX_TYPE_INPUT)
		return UX_PRIO_INPUT;
	else if (ux_type & UX_TYPE_ANIMATOR)
		return p->prio <= 116 ? UX_PRIO_ANIMATOR : UX_PRIO_ANIMATOR_LOW;
	else if (ux_type & UX_TYPE_TOPAPP)
		return UX_PRIO_TOPAPP;
	else if (ux_type & UX_TYPE_TOPUI)
		return UX_PRIO_TOPUI;
	else if ((moto_sched_scene & UX_SCENE_TOUCH) && (ux_type & UX_TYPE_LAUNCHER))
		return UX_PRIO_LAUNCHER;
	else if ((moto_sched_scene & UX_SCENE_TOUCH) && (ux_type & UX_TYPE_GESTURE_MONITOR))
		return UX_PRIO_GESTURE_MONITOR;
	else if (with_inherit && (ux_type & UX_TYPE_INHERIT_HIGH))
		return UX_PRIO_OTHER_HIGH;
	else if ((ux_type & UX_TYPE_SYSTEM || task_in_top_app_group(p)) && p->prio <= moto_boost_prio)
		return p->prio < 120 ? (UX_PRIO_OTHER_HIGH - (p->prio - 100)) : (UX_PRIO_OTHER_LOW - (p->prio - 120));
	else if (ux_type & UX_TYPE_KSWAPD)
		return UX_PRIO_KSWAPD;
	else if (with_inherit && (ux_type & UX_TYPE_INHERIT_LOW))
		return UX_PRIO_OTHER_LOW;

	return UX_PRIO_INVALID;
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
	int mvp_prio = task_get_mvp_prio(current, false);
	if (mvp_prio >= UX_PRIO_GESTURE_MONITOR) {
		task_add_ux_type(task, UX_TYPE_INHERIT_HIGH);
	}
}
EXPORT_SYMBOL(binder_inherit_ux_type);

void binder_clear_inherited_ux_type(struct task_struct *task) {
	task_clr_ux_type(task, UX_TYPE_INHERIT_HIGH|UX_TYPE_INHERIT_LOW);
}
EXPORT_SYMBOL(binder_clear_inherited_ux_type);

void binder_ux_type_set(struct task_struct *task, bool has_clear, bool clear) {

}
EXPORT_SYMBOL(binder_ux_type_set);
