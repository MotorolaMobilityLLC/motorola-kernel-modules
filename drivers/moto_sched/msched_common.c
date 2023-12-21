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


static inline int task_get_ux_type(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	return wts->ux_type;
#else
	return 0; // todo: implement it on mtk platform.
#endif
}

static inline void task_add_ux_type(struct task_struct *p, int type)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	wts->ux_type |= type;
#else
	// todo: implement it on mtk platform.
#endif
}

static inline void task_clr_ux_type(struct task_struct *p, int type)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	wts->ux_type &= ~type;
#else
	// todo: implement it on mtk platform.
#endif
}

static inline bool task_in_boost_uid(struct task_struct *p)
{
	return (moto_sched_scene & UX_SCENE_LAUNCH)
			&& global_boost_uid > 0 && task_uid(p).val == global_boost_uid;
}

static inline bool task_in_audio_uid(struct task_struct *p)
{
	if (moto_sched_scene & UX_SCENE_RINGTONE) {
		int uid = task_uid(p).val;
		return (uid == MEDIA_CODEC_UID)
			|| (uid == MEDIA_UID)
			|| (uid == MEDIA_EX_UID)
			|| (uid == AUDIOSERVER_UID);
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
	else if (ux_type & UX_TYPE_AUDIO || task_in_audio_uid(p))
		return p->prio <= 101 ? UX_PRIO_URGENT_AUDIO : UX_PRIO_AUDIO;
	else if (ux_type & UX_TYPE_INPUT)
		return UX_PRIO_INPUT;
	else if (ux_type & UX_TYPE_ANIMATOR)
		return UX_PRIO_ANIMATOR;
	else if (ux_type & UX_TYPE_TOPAPP)
		return UX_PRIO_TOPAPP;
	else if (with_inherit && (ux_type & UX_TYPE_INHERIT_HIGH))
		return UX_PRIO_TOPAPP;
	else if (ux_type & UX_TYPE_TOPUI)
		return UX_PRIO_TOPUI;
	else if (ux_type & (UX_TYPE_LAUNCHER|UX_TYPE_GESTURE_MONITOR))
		return UX_PRIO_LAUNCHER;
	else if (ux_type & UX_TYPE_KSWAPD)
		return UX_PRIO_KSWAPD;
	else if (task_in_top_app_group(p) || task_in_boost_uid(p))
		return p->prio < 120 ? UX_PRIO_TOPAPP_HIGH : UX_PRIO_TOPAPP_LOW;
	else if (p->tgid == global_systemserver_tgid || p->tgid == global_surfaceflinger_tgid)
		return p->prio < 120 ? UX_PRIO_SYSTEM_HIGH : UX_PRIO_SYSTEM_LOW;
	else if (with_inherit && (ux_type & UX_TYPE_INHERIT_LOW))
		return UX_PRIO_SYSTEM_LOW;
	else if (ux_type > 0) // in case we have some ux_type not handled, use UX_PRIO_SYSTEM_LOW.
		return UX_PRIO_SYSTEM_LOW;

	return UX_PRIO_INVALID;
}
EXPORT_SYMBOL(task_get_mvp_prio);

unsigned int task_get_mvp_limit(int mvp_prio) {
	// 100ms for top app
	if (mvp_prio == UX_PRIO_TOPAPP || mvp_prio == UX_PRIO_TOPAPP_HIGH || mvp_prio == UX_PRIO_TOPAPP_LOW)
		return 100000000U;

	// 100ms for kswapd
	if (mvp_prio == UX_PRIO_KSWAPD)
		return 100000000U;

	return -1;
}
EXPORT_SYMBOL(task_get_mvp_limit);

void binder_inherit_ux_type(struct task_struct *task) {
	int mvp_prio = task_get_mvp_prio(current, false);
	if (mvp_prio >= UX_PRIO_TOPUI) {
		task_add_ux_type(task, UX_TYPE_INHERIT_HIGH);
	} else {
		task_add_ux_type(task, UX_TYPE_INHERIT_LOW);
	}
}
EXPORT_SYMBOL(binder_inherit_ux_type);

void binder_clear_inherited_ux_type(struct task_struct *task) {
	task_clr_ux_type(task, UX_TYPE_INHERIT_HIGH|UX_TYPE_INHERIT_LOW);
}
EXPORT_SYMBOL(binder_clear_inherited_ux_type);
