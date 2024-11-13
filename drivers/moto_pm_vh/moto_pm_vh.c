/*
 * Simple PM android vendor hook module
 *
 * Copyright (C) 2024 Motorola Mobility LLC
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

#define pr_fmt(fmt) "moto_pm_vh: " fmt

#include <linux/module.h>
#include <linux/types.h>

#include <linux/interrupt.h>
#include <linux/oom.h>
#include <linux/suspend.h>
#include <linux/sched/debug.h>
#include <linux/sched/task.h>
#include <linux/syscalls.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kmod.h>
#include <trace/events/power.h>
#include <linux/cpuset.h>

#include <trace/hooks/power.h>

#include <linux/proc_fs.h>

#define MAX_FREE_MODE_LEN 16

static char freeze_mode_stat[MAX_FREE_MODE_LEN];
static struct proc_dir_entry *procfs_file;

static bool frozen_moto(struct task_struct *p)
{
	return READ_ONCE(p->__state)& TASK_FROZEN;
}

static int moto_pm_vh_seq_show(struct seq_file *f, void *ptr)
{
	seq_printf(f, "%s", freeze_mode_stat);

	return 0;
}

static int moto_pm_vh_open(struct inode *inode, struct file *file)
{
	return single_open(file, moto_pm_vh_seq_show, inode->i_private);
}

static ssize_t moto_pm_vh_write(struct file *file, const char __user *buf,
				size_t count, loff_t *offset)
{
	char buffer[MAX_FREE_MODE_LEN];
	const size_t maxlen = MAX_FREE_MODE_LEN - 1;

	memset(buffer, 0x0, MAX_FREE_MODE_LEN);
	if (copy_from_user(buffer, buf, count > maxlen ? maxlen : count))
		return -EFAULT;

	if (!memcmp(buffer, "panic", 5)) {
		memset(freeze_mode_stat, 0x0, MAX_FREE_MODE_LEN);
		memcpy(freeze_mode_stat, "panic", 5);
	}

	if(!memcmp(buffer, "normal", 6)) {
		memset(freeze_mode_stat, 0x0, MAX_FREE_MODE_LEN);
		memcpy(freeze_mode_stat, "normal", 6);
	}
	pr_info("%s: freeze_mode_stat [%s]\n", __func__, freeze_mode_stat);

	return count;
}

//from process.c: trace_android_vh_try_to_freeze_todo(todo, elapsed_msecs, wq_busy);
static void try_to_freeze_todo_hook(void *unused, unsigned int todo, unsigned int elapsed_msecs, bool wq_bus)
{
	struct task_struct *g, *p;

	pr_err("moto_pm_vh: Freezing failed after %d.%03d seconds,"
			"tasks refusing to freeze [%d] wq_bus [%d].\n",
			elapsed_msecs / 1000, elapsed_msecs % 1000,
			todo - wq_bus, wq_bus);
	read_lock(&tasklist_lock);
	for_each_process_thread(g, p) {
		//if (p != current && freezing(p) && !frozen(p)) {
		if (p != current && freezing(p) && !frozen_moto(p)) {
			sched_show_task(p);
		}
	}
	read_unlock(&tasklist_lock);

	pr_info("%s: freeze_mode_stat [%s]\n", __func__, freeze_mode_stat);
	if (!memcmp(freeze_mode_stat, "panic", 5)) {
		/* Trigger a real panic on debug setting */
		BUG();
	}

	pr_err("moto_pm_vh: try_to_freeze_todo exit");
	return;
}


#define REGISTER_HOOK(name) do {\
	rc = register_trace_android_vh_##name(name##_hook, NULL);\
	if (rc) {\
		pr_err("register hook %s failed", #name);\
		goto err_out_##name;\
	}\
} while (0)

#define UNREGISTER_HOOK(name) do {\
	unregister_trace_android_vh_##name(name##_hook, NULL);\
} while (0)

#define ERROR_OUT(name) err_out_##name

static int register_all_hooks(void)
{
	int rc;

	/* try_to_freeze_todo_hook */
	REGISTER_HOOK(try_to_freeze_todo);
	return 0;

ERROR_OUT(try_to_freeze_todo):
	return rc;
}

static void unregister_all_hook(void)
{
	UNREGISTER_HOOK(try_to_freeze_todo);
}


static const struct proc_ops  moto_pm_vh_op= {
	.proc_open		= moto_pm_vh_open,
	.proc_read		= seq_read,
	.proc_write		= moto_pm_vh_write,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int __init moto_pm_vh_init(void)
{
	int ret = 0;

	ret = register_all_hooks();
	if (ret != 0) {
		pr_err("register_all_hooks failed, ret = [%d]\n", ret);
		return ret;
	}

	memset(freeze_mode_stat, 0x0, MAX_FREE_MODE_LEN);
	memcpy(freeze_mode_stat, "normal", 6);

	procfs_file = proc_create("driver/moto_pm_vh", 0444, NULL, &moto_pm_vh_op);

	pr_info("moto_pm_vh_init succeed!\n");
	return 0;
}

static void __exit moto_pm_vh_exit(void)
{
	unregister_all_hook();

	if (procfs_file)
		remove_proc_entry("driver/moto_pm_vh", NULL);

	pr_info("moto_pm_vh_exit succeed!\n");
	return;
}

module_init(moto_pm_vh_init);
module_exit(moto_pm_vh_exit);
MODULE_DESCRIPTION("moto power vendor hook Driver");
MODULE_LICENSE("GPL v2");
