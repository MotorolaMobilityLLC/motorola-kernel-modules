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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/spinlock.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/list.h>
#include "monitor.h"

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */
#define STATICS_NUMBER      5      /* Number of top wakelocks to display */
#define NAME_SIZE           48      /* Max length of wakelock name */
#define MAX_TRACKING_LIMIT  256     /* Max unique locks to track (Memory Safety) */

/* --------------------------------------------------------------------------
 * Data Structures
 * -------------------------------------------------------------------------- */

struct prev_read_entry {
	struct list_head node;
	struct wakeup_source *ws;
	char name[NAME_SIZE];
	ktime_t last_total_time;
};

static LIST_HEAD(prev_read_list);
static DEFINE_MUTEX(stats_lock);
static ktime_t module_load_time;

/* Helper: Get current count of a list */
static int get_list_count(struct list_head *head)
{
	struct list_head *pos;
	int count = 0;
	list_for_each(pos, head) {
		count++;
	}
	return count;
}

/*
 * Core Logic: Calculate Delta
 *
 * Strategy:
 * 1. If lock exists in list: Delta = Current - Last_Read. Update Last_Read.
 * 2. If lock is NEW (not in list):
 *    - If list not full: Add to list, set Last_Read = Current. Delta = 0.
 *    - If list full: Ignore (Delta = 0).
 *
 * Note: If a lock disappears and reappears, it is treated as NEW (Delta=0 for that interval).
 * This ensures we never report huge historical spikes, only recent activity.
 */
static ktime_t calculate_delta_and_update(struct wakeup_source *ws, ktime_t current_total)
{
	struct prev_read_entry *pr_entry;
	ktime_t baseline = 0;
	bool found = false;

	mutex_lock(&stats_lock);

	// 1. Search in Cache
	list_for_each_entry(pr_entry, &prev_read_list, node) {
		if (pr_entry->ws == ws && strcmp(pr_entry->name, ws->name) == 0) {
			baseline = pr_entry->last_total_time;
			found = true;

			pr_entry->last_total_time = current_total;
			break;
		}
	}

	if (!found) {
		int current_count = get_list_count(&prev_read_list);

		if (current_count >= MAX_TRACKING_LIMIT) {
			// Limit reached: Ignore this lock to save memory.
			// Baseline = Current, so Delta = 0.
			baseline = current_total;
			pr_info_once("wakelock_profiler: Tracking limit (%d) reached. Ignoring new lock: %s\n",
					     MAX_TRACKING_LIMIT, ws->name);
		} else {
			pr_entry = kmalloc(sizeof(*pr_entry), GFP_ATOMIC);
			if (pr_entry) {
				pr_entry->ws = ws;
				strscpy(pr_entry->name, ws->name, NAME_SIZE);
				pr_entry->last_total_time = current_total;
				list_add(&pr_entry->node, &prev_read_list);

				baseline = current_total;
			} else {
				baseline = current_total;
			}
		}
	}

	mutex_unlock(&stats_lock);

	ktime_t delta = ktime_sub(current_total, baseline);
	return (delta < 0) ? 0 : delta;
}

/* Helper: Clear all tracking lists (Used on Exit) */
static void clear_all_stats(void)
{
	struct prev_read_entry *pr, *pr_tmp;

	mutex_lock(&stats_lock);
	list_for_each_entry_safe(pr, pr_tmp, &prev_read_list, node) {
		list_del(&pr->node);
		kfree(pr);
	}
	mutex_unlock(&stats_lock);
}

/*
 * Initial Capture: Run once at module load
 * Sets the baseline for all existing wakelocks so first 'cat' shows 0 delta.
 */
static void initial_capture(void)
{
	int srcuidx;
	unsigned long flags;
	struct wakeup_source *ws;
	int count = 0;

	module_load_time = ktime_get();

	mutex_lock(&stats_lock);
	srcuidx = wakeup_sources_read_lock();

	for (ws = wakeup_sources_walk_start(); ws; ws = wakeup_sources_walk_next(ws)) {
		if (count >= MAX_TRACKING_LIMIT) {
			pr_info("wakelock_profiler: Too many wakelocks at init. Truncating to %d.\n", MAX_TRACKING_LIMIT);
			break;
		}

		spin_lock_irqsave(&ws->lock, flags);
		ktime_t total = ws->total_time;
		if (ws->active) {
			ktime_t active_time = ktime_sub(ktime_get(), ws->last_time);
			total = ktime_add(total, active_time);
		}
		spin_unlock_irqrestore(&ws->lock, flags);

		// Add to list with current value as baseline
		struct prev_read_entry *pr = kmalloc(sizeof(*pr), GFP_ATOMIC);
		if (pr) {
			pr->ws = ws;
			strscpy(pr->name, ws->name, NAME_SIZE);
			pr->last_total_time = total;
			list_add(&pr->node, &prev_read_list);
			count++;
		}
	}
	wakeup_sources_read_unlock(srcuidx);
	mutex_unlock(&stats_lock);

	pr_info("wakelock_profiler: Initialized. Tracking %d locks.\n", count);
}

/* --------------------------------------------------------------------------
 * Sysfs Logic
 * -------------------------------------------------------------------------- */

struct wakelock_desc {
	struct wakeup_source *ws;
	ktime_t delta_time;
	char name[NAME_SIZE];
};

struct wakelock_name_delta {
	char name[NAME_SIZE];
	ktime_t delta_time;
};

static struct wakelock_desc max_wakelock_list[STATICS_NUMBER];
static DEFINE_MUTEX(list_lock);

/*
 * SHOW Command
 * Calculates deltas since last read, sorts top N, and prints.
 */
static ssize_t active_wakelock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int srcuidx, i, j, k;
	int buf_offset = 0;
	unsigned long flags;
	struct wakeup_source *ws;
	struct wakelock_name_delta *agg_list;
	int agg_count = 0;

	mutex_lock(&list_lock);
	memset(max_wakelock_list, 0, sizeof(max_wakelock_list));
	agg_list = kcalloc(MAX_TRACKING_LIMIT, sizeof(*agg_list), GFP_KERNEL);
	if (!agg_list) {
		mutex_unlock(&list_lock);
		return -ENOMEM;
	}

	srcuidx = wakeup_sources_read_lock();

	for (ws = wakeup_sources_walk_start(); ws; ws = wakeup_sources_walk_next(ws)) {
		ktime_t current_total = ws->total_time;
		ktime_t delta;
		int found_idx = -1;

		// Include currently active time
		spin_lock_irqsave(&ws->lock, flags);
		if (ws->active) {
			ktime_t active_time = ktime_sub(ktime_get(), ws->last_time);
			current_total = ktime_add(current_total, active_time);
		}
		spin_unlock_irqrestore(&ws->lock, flags);

		// Calculate Delta (Pure Mode)
		delta = calculate_delta_and_update(ws, current_total);
		if (delta <= 0)
			continue;

		for (i = 0; i < agg_count; i++) {
			if (strcmp(agg_list[i].name, ws->name) == 0) {
				found_idx = i;
				break;
			}
		}

		if (found_idx >= 0) {
			agg_list[found_idx].delta_time =
				ktime_add(agg_list[found_idx].delta_time, delta);
		} else if (agg_count < MAX_TRACKING_LIMIT) {
			strscpy(agg_list[agg_count].name, ws->name, NAME_SIZE);
			agg_list[agg_count].delta_time = delta;
			agg_count++;
		}
	}
	wakeup_sources_read_unlock(srcuidx);

	for (i = 0; i < agg_count; i++) {
		for (j = 0; j < STATICS_NUMBER; j++) {
			if (max_wakelock_list[j].name[0] == 0 ||
			    ktime_compare(agg_list[i].delta_time, max_wakelock_list[j].delta_time) > 0) {
				for (k = STATICS_NUMBER - 1; k >= j + 1; k--) {
					max_wakelock_list[k].ws = max_wakelock_list[k-1].ws;
					max_wakelock_list[k].delta_time = max_wakelock_list[k-1].delta_time;
					strscpy(max_wakelock_list[k].name, max_wakelock_list[k-1].name, NAME_SIZE);
				}
				max_wakelock_list[j].ws = NULL;
				max_wakelock_list[j].delta_time = agg_list[i].delta_time;
				strscpy(max_wakelock_list[j].name, agg_list[i].name, NAME_SIZE);
				break;
			}
		}
	}

	for (i = 0; i < STATICS_NUMBER; i++) {
		if (max_wakelock_list[i].name[0] != 0 && max_wakelock_list[i].delta_time > 0) {
			size_t remaining;
			int written;

			if (buf_offset >= PAGE_SIZE)
				break;

			remaining = PAGE_SIZE - buf_offset;
			written = scnprintf(buf + buf_offset, remaining,
				"%s %lld\n",
				max_wakelock_list[i].name,
				ktime_to_ms(max_wakelock_list[i].delta_time));
			if (written <= 0)
				break;

			buf_offset += written;
		}
	}

	kfree(agg_list);

	mutex_unlock(&list_lock);
	return buf_offset;
}

sys_monitor_attr_ro(active_wakelock);

static struct attribute *wakelock_attrs[] = {
	&active_wakelock_attr.attr,
	NULL,
};

static struct attribute_group wakelock_attr_group = {
	.attrs = wakelock_attrs,
};

/* --------------------------------------------------------------------------
 * Module Init / Exit
 * -------------------------------------------------------------------------- */
int wakelock_profile_init(struct kobject *parent_kobj)
{
	int retval;

	if (!parent_kobj) {
		pr_err("wakelock_profiler: parent kobject is NULL!\n");
		return -EINVAL;
	}

	pr_info("wakelock_profiler: Initializing (Pure Delta Mode)...\n");
	pr_info("wakelock_profiler: Max tracked: %d, Display top: %d\n", MAX_TRACKING_LIMIT, STATICS_NUMBER);

	retval = sysfs_create_group(parent_kobj, &wakelock_attr_group);
	if (retval) {
		pr_err("wakelock_profiler: failed to create sysfs group (%d)\n", retval);
		return retval;
	}

	// Capture initial state immediately
	initial_capture();

	pr_info("wakelock_profiler: Ready\n");
	return 0;
}

void wakelock_profile_exit(struct kobject *parent_kobj)
{
	sysfs_remove_group(parent_kobj, &wakelock_attr_group);
	clear_all_stats();
	pr_info("wakelock_profiler: Removed.\n");
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Pure Delta Wakelock Profiler (No Reset Needed)");
MODULE_AUTHOR("Optimized Version");
