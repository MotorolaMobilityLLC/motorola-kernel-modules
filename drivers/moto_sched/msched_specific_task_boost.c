// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Motorola Mobility LLC
 *
 * Boost specific tasks by matching names inherited at creation or assigned
 * by an explicit rename.
 */

#define pr_fmt(fmt) "moto_sched: " fmt

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/version.h>
#include <trace/hooks/sched.h>

#include "msched_common.h"
#include "msched_specific_task_boost.h"
#include "msched_trace.h"

/*
 * Names inherited from the creating task without an explicit rename.
 * Add such tasks here for animator boost at creation.
 */
#ifdef CONFIG_MOTO_BOOST_SPECIFIC_TASK_ON_CREATE
static const char *const new_task_animator_boost_names[] = {
	"surfaceflinger",
};
#endif

/*
 * Names assigned explicitly through exec, prctl, or pthread naming APIs.
 * Add such tasks here for foreground UI boost after rename.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static const char *const renamed_task_fg_ui_boost_names[] = {
	"UnityMain",
};
#endif

static void boost_specific_task(struct task_struct *p,
		const char *const *task_names, size_t nr_task_names,
		int ux_type)
{
	size_t i;

	if (!p || !task_names)
		return;

	for (i = 0; i < nr_task_names; i++) {
		if (!task_names[i] || strcmp(p->comm, task_names[i]))
			continue;

		if (!task_has_ux_type(p, ux_type))
			task_add_ux_type(p, ux_type);
		break;
	}
}

#ifdef CONFIG_MOTO_BOOST_SPECIFIC_TASK_ON_CREATE
static bool specific_task_name_matches(struct task_struct *p,
		const char *const *task_names, size_t nr_task_names)
{
	size_t i;

	if (!p || !task_names)
		return false;

	for (i = 0; i < nr_task_names; i++) {
		if (task_names[i] && !strcmp(p->comm, task_names[i]))
			return true;
	}

	return false;
}

static void clear_specific_animator_boost_after_rename(
		struct task_struct *p)
{
	if (!task_has_ux_type(p, UX_TYPE_SPECIFIC_ANIMATOR))
		return;

	if (!specific_task_name_matches(p, new_task_animator_boost_names,
			ARRAY_SIZE(new_task_animator_boost_names)))
		task_clr_ux_type(p, UX_TYPE_SPECIFIC_ANIMATOR);
}

/*
 * Boost newly created tasks before their first enqueue. This catches target
 * names inherited from the creating task when no explicit rename follows.
 */
static void android_rvh_wake_up_new_task(void *unused, struct task_struct *p)
{
	(void)unused;

	if (!p)
		return;

	trace_msched_specific_task_new(p);
	task_clr_ux_type(p, UX_TYPE_SPECIFIC_ANIMATOR);
	boost_specific_task(p, new_task_animator_boost_names,
			ARRAY_SIZE(new_task_animator_boost_names),
			UX_TYPE_SPECIFIC_ANIMATOR);
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0))
/*
 * Boost tasks after an explicit comm change. This catches final names
 * assigned by exec, prctl, or pthread naming APIs.
 */
static void android_vh_set_task_comm(void *unused, struct task_struct *p)
{
	(void)unused;

	if (!p)
		return;

	trace_msched_specific_task_rename(p);
#ifdef CONFIG_MOTO_BOOST_SPECIFIC_TASK_ON_CREATE
	clear_specific_animator_boost_after_rename(p);
#endif
	boost_specific_task(p, renamed_task_fg_ui_boost_names,
			ARRAY_SIZE(renamed_task_fg_ui_boost_names),
			UX_TYPE_FG_UI);
}
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 18, 0))
/*
 * Boost tasks after an explicit comm change. This catches final names
 * assigned by exec, prctl, or pthread naming APIs.
 */
static void android_rvh_set_task_comm(void *unused, struct task_struct *p,
		bool exec)
{
	(void)unused;
	(void)exec;

	if (!p)
		return;

	trace_msched_specific_task_rename(p);
#ifdef CONFIG_MOTO_BOOST_SPECIFIC_TASK_ON_CREATE
	clear_specific_animator_boost_after_rename(p);
#endif
	boost_specific_task(p, renamed_task_fg_ui_boost_names,
			ARRAY_SIZE(renamed_task_fg_ui_boost_names),
			UX_TYPE_FG_UI);
}
#endif

#endif /* LINUX_VERSION_CODE >= 5.15 */

void msched_specific_task_boost_init(void)
{
	int ret;
#ifdef CONFIG_MOTO_BOOST_SPECIFIC_TASK_ON_CREATE
	ret = register_trace_android_rvh_wake_up_new_task(
			android_rvh_wake_up_new_task, NULL);
	if (ret)
		pr_err("failed to register wake_up_new_task hook: %d\n", ret);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0))
	ret = register_trace_android_vh_set_task_comm(
			android_vh_set_task_comm, NULL);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 18, 0))
	ret = register_trace_android_rvh_set_task_comm(
			android_rvh_set_task_comm, NULL);
#endif
	if (ret)
		pr_err("failed to register set_task_comm hook: %d\n", ret);
#endif
}
