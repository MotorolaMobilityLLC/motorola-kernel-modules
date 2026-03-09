/*
 * Copyright (C) 2026 Motorola Mobility LLC
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

#define pr_fmt(fmt) "sys_monitor: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include "monitor.h"

static char screen_state[4];
static char screen0_state[4];
static char screen1_state[4];
static char refresh_rate[128];
static char refresh_by_region[128];
static char refresh_by_area[128];
static char hotspot[12];

ssize_t generic_str_show(char *dest, size_t dest_size, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", dest);
}

ssize_t generic_str_store(char *dest, size_t dest_size, const char *buf, size_t n)
{
	strscpy(dest, buf, dest_size);
	return n;
}

#define DEFINE_MONITOR_ATTR(_name) \
ssize_t _name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
{ \
	return generic_str_show(_name, sizeof(_name), buf); \
} \
ssize_t _name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n) \
{ \
	return generic_str_store(_name, sizeof(_name), buf, n); \
} \
sys_monitor_attr(_name)

DEFINE_MONITOR_ATTR(screen_state);
DEFINE_MONITOR_ATTR(screen0_state);
DEFINE_MONITOR_ATTR(screen1_state);
DEFINE_MONITOR_ATTR(refresh_rate);
DEFINE_MONITOR_ATTR(refresh_by_region);
DEFINE_MONITOR_ATTR(refresh_by_area);
DEFINE_MONITOR_ATTR(hotspot);

static struct attribute * misc_attrs[] = {
	&screen_state_attr.attr,
	&screen0_state_attr.attr,
	&screen1_state_attr.attr,
	&refresh_rate_attr.attr,
	&refresh_by_region_attr.attr,
	&refresh_by_area_attr.attr,
	&hotspot_attr.attr,
	NULL,
};

static const struct attribute_group misc_attr_group = {
	.attrs = misc_attrs,
};

int monitor_misc_init(struct kobject *parent_kobj)
{
	int ret = 0;
	ret = sysfs_create_group(parent_kobj, &misc_attr_group);
	if (ret) {
		pr_err("%s: misc node create failed\n", __func__);
		ret = -ENOMEM;
	}
	return ret;
}

void monitor_misc_exit(struct kobject *parent_kobj)
{
	sysfs_remove_group(parent_kobj, &misc_attr_group);
}
