// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2025 Moto LLC
 */

#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "cma.h"
#include "vmscan.h"
#include "compaction.h"
#include "slowpath.h"

DEFINE_PER_CPU(unsigned long, pgalloc_costly_order);
DEFINE_PER_CPU(unsigned long, pgcache_miss);
DEFINE_PER_CPU(unsigned long, pgcache_hit);

static ssize_t vmstat_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int cpu;
	unsigned long pages = 0;
	unsigned long miss_count = 0;
	unsigned long hit_count = 0;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		pages += per_cpu(pgalloc_costly_order, cpu);
		miss_count += per_cpu(pgcache_miss, cpu);
		hit_count += per_cpu(pgcache_hit, cpu);
	}
	cpus_read_unlock();

	return sprintf(buf, "%s %lu\n%s %lu\n%s %lu\n",
			"pgalloc_costly_order", pages,
			"pgcache_miss", miss_count,
			"pgcache_hit", hit_count);
}

static struct kobj_attribute vmstat_attribute = __ATTR_RO(vmstat);

extern struct kobject *moto_stat_kobj;
static struct kobject *moto_stat_mm_kobj;
static struct attribute *attrs[] = {
	&vmstat_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

struct kobject *moto_stat_kobj;
EXPORT_SYMBOL_GPL(moto_stat_kobj);

int moto_mm_sysfs(void)
{
	int ret;

	moto_stat_kobj = kobject_create_and_add("moto_stat", kernel_kobj);

        if (!moto_stat_kobj)
                return -ENOMEM;

	if (moto_stat_kobj)
		moto_stat_mm_kobj = kobject_create_and_add("mm", moto_stat_kobj);

	if (!moto_stat_mm_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(moto_stat_mm_kobj, &attr_group);
	if (ret)
		goto put_mm_kobj;

	ret = create_vmscan_sysfs(moto_stat_mm_kobj);
	if (ret)
		goto remove_stat_sysfs;

	ret = create_cma_sysfs(moto_stat_mm_kobj);
	if (ret)
		goto remove_vmscan_sysfs;

	ret = slowpath_sysfs(moto_stat_mm_kobj);
	if (ret)
		goto remove_cma_sysfs;

	ret = compaction_sysfs(moto_stat_mm_kobj);
	if (ret)
		goto remove_slowpath_sysfs;

	return ret;

remove_slowpath_sysfs:
	remove_slowpath_sysfs();
remove_cma_sysfs:
	remove_cma_sysfs();
remove_vmscan_sysfs:
	remove_vmscan_sysfs();
remove_stat_sysfs:
	sysfs_remove_group(moto_stat_mm_kobj, &attr_group);
put_mm_kobj:
	kobject_put(moto_stat_mm_kobj);

	return ret;
}
