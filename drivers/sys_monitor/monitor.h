/*
 * Copyright (C) 2025 Motorola Mobility LLC
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

#ifndef __MONITOR_H__
#define __MONITOR_H__

#define sys_monitor_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr = {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show = _name##_show,			\
	.store = _name##_store,			\
}

#define sys_monitor_attr_ro(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr = {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show = _name##_show,			\
}

ssize_t sleep_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

sys_monitor_attr_ro(sleep_state);

int monitor_sleep_init(void);
void monitor_sleep_exit(void);

int monitor_misc_init(struct kobject *parent_kobj);
void monitor_misc_exit(struct kobject *parent_kobj);

int wakelock_profile_init(struct kobject *parent_kobj);
void wakelock_profile_exit(struct kobject *parent_kobj);

int monitor_net_stats_init(struct kobject *parent_kobj);
void monitor_net_stats_exit(struct kobject *parent_kobj);
#endif //__MONITOR_H__
