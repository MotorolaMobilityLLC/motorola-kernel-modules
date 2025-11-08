/*
 * Simple PM android vendor hook module
 *
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

#define pr_fmt(fmt) "moto_thread_vh: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <trace/hooks/sched.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>

static int num_threads;
static int nr_thread_max_level;
static int max_signal_threads;
static struct proc_dir_entry *procfs_file;

static int moto_thread_vh_seq_show(struct seq_file *f, void *ptr)
{
	seq_printf(f, "%d", num_threads);

	return 0;
}

static int moto_thread_vh_open(struct inode *inode, struct file *file)
{
	return single_open(file, moto_thread_vh_seq_show, inode->i_private);
}

//from fork.c: trace_android_vh_copy_process(current, nr_threads);
static void copy_process_hook(void *unused, struct task_struct *p, int nr_threads)
{
	num_threads = nr_threads;

	if (p->signal && (p->signal->nr_threads >= max_signal_threads))
	{
		pr_info("p->comm [%s] tgid[%d] pid[%d] nr_threads %d excends max_signal_threads %d",
				p->comm, task_tgid_nr(p), task_pid_nr(p), p->signal->nr_threads, max_signal_threads);
		max_signal_threads += 200;
		if (p->signal->nr_threads >= 5000)
		{
			pr_err("p->comm [%s] tgid[%d] pid[%d] p->nr_threads %d excends 5000, need panic",
					p->comm, task_tgid_nr(p), task_pid_nr(p), p->signal->nr_threads);
			BUG();
		}
	}

	if (nr_threads >= nr_thread_max_level) {
		pr_info("current processes [%s] tgid [%d] pid[%d] nr_threads %d count exceeds current max level %d",
				p->comm, task_tgid_nr(p), task_pid_nr(p), nr_threads, nr_thread_max_level);
		nr_thread_max_level += 1000;
		if (nr_threads >= 32000) {
			pr_info("current processes [%s] tgid [%d] pid[%d] count exceeds current max level 32000, need panic",
				p->comm, task_tgid_nr(p), task_pid_nr(p));
			BUG();
		}
	}

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
	REGISTER_HOOK(copy_process);
	return 0;

ERROR_OUT(copy_process):
	return rc;
}

static void unregister_all_hook(void)
{
	UNREGISTER_HOOK(copy_process);
}


static const struct proc_ops  moto_thread_vh= {
	.proc_open		= moto_thread_vh_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int __init moto_thread_vh_init(void)
{
	int ret = 0;

	nr_thread_max_level = 5000;
	max_signal_threads = 200;

	ret = register_all_hooks();
	if (ret != 0) {
		pr_err("register_all_hooks failed, ret = [%d]", ret);
		return ret;
	}

	procfs_file = proc_create("driver/moto_thread_vh", 0444, NULL, &moto_thread_vh);

	pr_info("moto_thread_vh_init succeed!");
	return 0;
}

static void __exit moto_thread_vh_exit(void)
{
	unregister_all_hook();

	if (procfs_file)
		remove_proc_entry("driver/moto_thread_vh", NULL);

	pr_info("moto_thread_vh_exit succeed!");
	return;
}

module_init(moto_thread_vh_init);
module_exit(moto_thread_vh_exit);
MODULE_DESCRIPTION("moto thread hook Driver");
MODULE_LICENSE("GPL v2");
