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

#ifndef _MOTO_BINDER_H
#define _MOTO_BINDER_H

#include <linux/freezer.h>
#include <linux/cgroup.h>
#include <uapi/linux/android/binder.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/signal.h>
#include <linux/version.h>
#include <linux/sched/cputime.h>
#include "../../kernel/sched/sched.h"
#include "../../drivers/android/binder_internal.h"
#include "../../drivers/android/binder_alloc.h"


#define MIN_USERAPP_UID (10000)
#define MAX_SYSTEM_UID  (2000)

/* type */
#define CALL_BINDER 0
#define ASYNC_BINDER 1
#define REPLY_BINDER 2

static inline bool is_jobctl_frozen(struct task_struct *task)
{
	return ((task->jobctl & JOBCTL_TRAP_FREEZE) != 0);
}

static inline bool is_frozen_state_compatible(struct task_struct *task)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return READ_ONCE(task->__state) & TASK_FROZEN;
#else
	return frozen(task);
#endif
}

/* Check if the thread group is frozen */
static inline bool is_frozen_tg(struct task_struct *task)
{
	return ((cgroup_task_frozen(task) && is_jobctl_frozen(task)) || is_frozen_state_compatible(task->group_leader) || freezing(task->group_leader));
}

extern int moto_binder_status_enabled;

extern int moto_binder_proc_init(void);
extern void moto_binder_proc_deinit(void);
extern int moto_binder_status_init(void);
extern void moto_binder_status_exit(void);
extern int proc_binder_status_show(struct seq_file *m, void *unused);
extern void moto_binder_write_status(int call_type, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2);
extern void moto_binder_send_uevent(int call_type, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2);

#endif  /*_MOTO_BINDER_H*/
