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

#define pr_fmt(fmt) "sys_monitor: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/sort.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/cpufreq.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/kernel_stat.h>
#include <trace/hooks/cpufreq.h>
#include <clocksource/arm_arch_timer.h>
#include "monitor.h"

#define MAX_TOP_RECORD 20

#define MAX_SYS_RECORD 100
#define MAX_APP_RECORD 100
#define MAX_OTHERS_RECORD 100

#define MAX_OTHERS_TOP 10

#define STATE_MAX 90

struct uid_record {
	unsigned int uid;
	unsigned int type;
	unsigned int max_state;
	unsigned int __pad;
	char name[TASK_COMM_LEN];
	u64 total_power;
	u64 time_in_state[STATE_MAX];
} __attribute__((aligned(64)));

struct cpufreqs_mmap_data {
	int app_count;
	int sys_count;
	int others_count;
	struct uid_record app_records[MAX_APP_RECORD];
	struct uid_record sys_records[MAX_SYS_RECORD];
	struct uid_record others_records[MAX_OTHERS_RECORD];
};

static struct cpufreqs_mmap_data cpufreqs_usage = {0};

static DEFINE_SPINLOCK(sys_monitor_lock);
static struct uid_record top_record[MAX_TOP_RECORD];
static struct uid_record backup_record[MAX_SYS_RECORD];
static struct kobject *sys_monitor_obj;
static int cpufreq_count = 0;
static int top_app_count = 10;
static int core_num = 0;
static char core_freq_count[3];

static void clean_records(struct uid_record *records, int count)
{
        if(records != NULL)
	    memset(records, 0, sizeof(struct uid_record) * count);
}

static int compare_total_power(const void *a, const void *b) {
	const struct uid_record *ra = a;
	const struct uid_record *rb = b;
	return (ra->total_power < rb->total_power) ? 1:
		(ra->total_power > rb->total_power) ? -1 : 0;
}

void select_top_records(struct uid_record *records,
                          struct uid_record *top, int count, int size) {
	int i;
	int heap_size = size;
	
	if ((top == NULL) || (records == NULL)) {
	        pr_err("%s: select_top_records failed\n", __func__);
	        return;
	}

	if (count <= heap_size) {
		memcpy(top, records, sizeof(struct uid_record) * count);
		sort(top, count, sizeof(struct uid_record), compare_total_power, NULL);
		return;
	}

	memcpy(top, records, sizeof(struct uid_record) * heap_size);
	sort(top, heap_size, sizeof(struct uid_record), compare_total_power, NULL);
	for (i = heap_size; i < count; i++) {
		if (records[i].total_power > top[heap_size - 1].total_power) {
			memcpy(&top[heap_size - 1], &records[i], sizeof(struct uid_record) * 1);
			sort(top, heap_size, sizeof(struct uid_record), compare_total_power, NULL);
		}
	}

}

static int show_top_records(struct uid_record *records, char *buf, int use_count, int max_count)
{
	int count;
	int i, j, len;

	select_top_records(records, top_record, use_count, MAX_TOP_RECORD);
	clean_records(records, use_count);

	if (use_count > max_count)
		count = max_count;
	else
		count = use_count;

	len = 0;
	for (i = 0; i < count; i++ ) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d:%d:%s", top_record[i].uid, use_count, top_record[i].name);

		for (j = 0; j < top_record[i].max_state; j++) {
			if (top_record[i].time_in_state[j] != 0) {
				len += scnprintf(buf + len, PAGE_SIZE - len, "|%d:%llu", j, top_record[i].time_in_state[j]);
			}
		}
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		if (PAGE_SIZE - len < 256)
			break;
	}

	return len;
}

static ssize_t others_top_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	int use_count;

	spin_lock_irqsave(&sys_monitor_lock, flags);
	use_count = cpufreqs_usage.others_count;
	memcpy(backup_record, cpufreqs_usage.others_records, sizeof(struct uid_record) * use_count);
	cpufreqs_usage.others_count = 0;
	clean_records(cpufreqs_usage.others_records, use_count);
	spin_unlock_irqrestore(&sys_monitor_lock, flags);

	return show_top_records(backup_record, buf, use_count, MAX_OTHERS_TOP);
}

sys_monitor_attr_ro(others_top);

static ssize_t sys_top_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	int use_count;

	spin_lock_irqsave(&sys_monitor_lock, flags);
	use_count = cpufreqs_usage.sys_count;
	memcpy(backup_record, cpufreqs_usage.sys_records, sizeof(struct uid_record) * use_count);
	cpufreqs_usage.sys_count = 0;
	clean_records(cpufreqs_usage.sys_records, use_count);
	spin_unlock_irqrestore(&sys_monitor_lock, flags);

	return show_top_records(backup_record, buf, use_count, MAX_OTHERS_TOP);
}

sys_monitor_attr_ro(sys_top);

static ssize_t app_top_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	int use_count;

	spin_lock_irqsave(&sys_monitor_lock, flags);
	use_count = cpufreqs_usage.app_count;
	memcpy(backup_record, cpufreqs_usage.app_records, sizeof(struct uid_record) * use_count);
	cpufreqs_usage.app_count = 0;
	clean_records(cpufreqs_usage.app_records, use_count);
	spin_unlock_irqrestore(&sys_monitor_lock, flags);

	return show_top_records(backup_record, buf, use_count, top_app_count);
}

static ssize_t app_top_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned int val;
	if (kstrtoint(buf, 0, &val)) {
		return -EINVAL;
	}
	if (val > MAX_TOP_RECORD)
		top_app_count = MAX_TOP_RECORD;
	else
		top_app_count = val;
	return n;
}
sys_monitor_attr(app_top);

static ssize_t cpufreq_count_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i, len;

	len = 0;
	for (i = 0; i < core_num; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", core_freq_count[i]);
	}

	len += scnprintf(buf + len, PAGE_SIZE - len, "%d\n", cpufreq_count);
	return len;
}

static ssize_t cpufreq_count_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	char *str = kmemdup(buf, n + 1, GFP_KERNEL);
	char *token;
	int idx = 0;
	int count = 0;
	int core_count = 0;
	int val;
	int num[5] = {0};
	int i;

	if (!str)
		return n;

	if (cpufreq_count!= 0)
		goto _exit;

	idx = 0;

	while ((token = strsep(&str, " ")) != NULL && idx < ARRAY_SIZE(num)) {
		if (*token == '\0')
			continue;

		if (kstrtoint(token, 10, &val)) {
			continue;
		}

		num[idx++] = val;
	}

	if (idx > 2) {
		core_count = idx -1;
		if (core_count > 3) {
			pr_err("core count input error\n");
			goto _exit;
		}

		for (i = 0; i < core_count; i++) {
			count += num[i];
		}

		if (count == num[core_count]) {
			core_num = core_count;
			for (i = 0; i < core_count; i++) {
				core_freq_count[i] = num[i];
			}
			cpufreq_count = num[core_count];
		}
	}

_exit:
	kfree(str);

	return n;
}
sys_monitor_attr(cpufreq_count);

static struct attribute * sys_monitor[] = {
	&app_top_attr.attr,
	&sys_top_attr.attr,
	&others_top_attr.attr,
	&cpufreq_count_attr.attr,
	&sleep_state_attr.attr,
	NULL,
};

static const struct attribute_group sys_monitor_attr_group = {
	.attrs = sys_monitor,
};

uint64_t get_weights(int index)
{
	int little = core_freq_count[0];
	int big = core_freq_count[1];
	int super = core_freq_count[2];
	int base = 100;

	if (index < little && little > 1) {
		int reversed_index = little - 1 - index;
		return (base + (int)(reversed_index * ((base * 2 - base) / (little - 1))));
	} else if (index < (big + little) && big > 1) {
		int local_index = index - little;
		int reversed_index = big - 1 - local_index;
		return (base + (int)(reversed_index * ((base * 6 - base) / (big - 1))));
	} else if (index < (big + little + super) && super > 1) {
		int local_index = index - little - big;
		int reversed_index = super - 1 - local_index;
		return (base + (int)(reversed_index * ((base * 6 - base) / (super - 1))));
	}
	return 1;
}

void record_task_cpufreq_times(void *data, u64 cputime, struct task_struct *p,
					unsigned int state) {
	struct cpufreqs_mmap_data *ptr = &cpufreqs_usage;
	struct uid_record *records;
	int max_count;
	int *count;
	unsigned long flags;
	int i;
	uid_t uid = from_kuid_munged(current_user_ns(), task_uid(p));
	unsigned int pid = (unsigned int )task_pid_nr(p);
	unsigned int tgid = (unsigned int )task_tgid_nr(p);

	if ((cpufreq_count == 0) || (uid == (uid_t) -1))
		return;

	spin_lock_irqsave(&sys_monitor_lock, flags);

	if (uid == 1000) {
		uid = tgid;
		max_count = MAX_SYS_RECORD;
		records = ptr->sys_records;
		count = &ptr->sys_count;
	} else if (uid == 1001 || uid == 0) {
		uid = tgid;
		max_count = MAX_OTHERS_RECORD;
		records = ptr->others_records;
		count = &ptr->others_count;
	} else {
		max_count = MAX_APP_RECORD;
		records = ptr->app_records;
		count = &ptr->app_count;
	}

	for (i = 0; i < *count; i++) {
		if (records[i].uid == uid) {
			if (state < STATE_MAX)	{
				/* the unit of time_in_state is ms */
				records[i].time_in_state[state] += DIV_ROUND_CLOSEST(cputime, NSEC_PER_MSEC);
				if (records[i].max_state < state)
					records[i].max_state = state + 1;
			}
			if (pid == tgid && likely(p != NULL)) {
				if (likely(p->comm)) {
					memcpy(records[i].name, p->comm, TASK_COMM_LEN);
					records[i].name[TASK_COMM_LEN - 1] = '\0';
				}
			}
			records[i].total_power += (DIV_ROUND_CLOSEST(cputime, NSEC_PER_MSEC)  * get_weights(state));
			goto _exit;
		}
	}

	if (i >= max_count || unlikely(p == NULL)) {
		goto _exit;
	}

	records[i].uid = uid;
	/* the unit of time_in_state is ms */
	if (likely(p->comm)) {
		memcpy(records[i].name, p->comm, TASK_COMM_LEN);
		records[i].name[TASK_COMM_LEN - 1] = '\0';
	}
	if (state < STATE_MAX) {
		records[i].time_in_state[state] = DIV_ROUND_CLOSEST(cputime, NSEC_PER_MSEC);
		records[i].max_state = state + 1;
	}
	records[i].total_power = DIV_ROUND_CLOSEST(cputime, NSEC_PER_MSEC) * get_weights(state);

	(*count)++;

_exit:
	spin_unlock_irqrestore(&sys_monitor_lock, flags);
}

static int __init sys_monitor_init(void)
{
	int ret = 0;

	ret = register_trace_android_vh_cpufreq_acct_update_power(record_task_cpufreq_times, NULL);
	if (ret < 0) {
		pr_err("register trace cpufreq update failed! ret=%d\n", ret);
		return ret;
	}

	sys_monitor_obj = kobject_create_and_add("sys_monitor", kernel_kobj);
	if (!sys_monitor_obj) {
		pr_err("%s: sysfs create and add failed\n", __func__);
		ret = -ENOMEM;
		goto error_kobj_register;
	}

	ret = sysfs_create_group(sys_monitor_obj, &sys_monitor_attr_group);
	if (ret) {
		pr_err("%s: sysfs create failed\n", __func__);
		ret = -ENOMEM;
		goto error_sysfs_create;
	}

	monitor_sleep_init();

	return 0;

error_sysfs_create:
	kobject_del(sys_monitor_obj);
	sys_monitor_obj= NULL;
error_kobj_register:
	unregister_trace_android_vh_cpufreq_acct_update_power(record_task_cpufreq_times, NULL);

	return ret;
}

void __exit sys_monitor_exit(void)
{
	unregister_trace_android_vh_cpufreq_acct_update_power(record_task_cpufreq_times, NULL);
	sysfs_remove_group(sys_monitor_obj, &sys_monitor_attr_group);
	kobject_del(sys_monitor_obj);
	monitor_sleep_exit();
	sys_monitor_obj= NULL;
}

module_init(sys_monitor_init);
module_exit(sys_monitor_exit);
MODULE_DESCRIPTION("Motorola sys monitor driver");
MODULE_LICENSE("GPL v2");
