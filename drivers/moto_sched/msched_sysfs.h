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

#ifndef _MOTO_SCHED_SYSFS_H_
#define _MOTO_SCHED_SYSFS_H_

extern int moto_sched_enabled;
extern int moto_sched_scene;

int moto_sched_proc_init(void);
void moto_sched_proc_deinit(void);

extern struct task_struct *find_task_by_vpid(pid_t vnr);

#endif /* _MOTO_SCHED_SYSFS_H_ */
