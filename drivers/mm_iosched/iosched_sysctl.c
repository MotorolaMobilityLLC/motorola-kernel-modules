// SPDX-License-Identifier: GPL-2.0-only

/*
 */

#include <linux/init.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/slab.h>

#include <trace/events/sched.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/cgroup.h>

#include "mot-io-trace.h"
#include "mio.h"


#define UX_TYPE_MASK ( UX_TYPE_INHERIT_BINDER | UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI |UX_TYPE_INHERIT_LOCK | UX_TYPE_SYSTEM_LOCK | UX_TYPE_PERF_DAEMON | UX_TYPE_AUDIO |UX_TYPE_AUDIOSERVICE |UX_TYPE_LOW_LATENCY_BINDER)

static struct ctl_table_header *ctl_table_hdr;
int enable_boost = 0;
static pid_t sys_pid[2] = {0,0};

int enable_log = 0;
static int enable_hook = 0;

#define system_pid  sys_pid[0]
static pid_t srv_pid = 0;




struct ctl_table iosched_table[] = {
    {
        .procname   = "enable_io_boost",
        .data       = &enable_boost,
        .maxlen     = sizeof(int),
        .mode       = 0644,
        .proc_handler   = proc_dointvec_minmax,
    },
	{
        .procname   = "enable_log",
        .data       = &enable_log,
        .maxlen     = sizeof(int),
        .mode       = 0644,
        .proc_handler   = proc_dointvec_minmax,
    },
    {
        .procname   = "sys_pid",
        .data       = &sys_pid,
        .maxlen     = 2*sizeof(pid_t),
        .mode       = 0644,
        .proc_handler   = proc_dointvec_minmax,
    },
    { }
};

struct ctl_table iosched_base_table[] = {
    {
        .procname   = "mio",
        .mode       = 0555,
        .child      = iosched_table,
    },
    { }
};


bool is_enabled_boost(void)
{
	return enable_boost;
}
static inline bool main_task(struct task_struct *tsk)
{
	return ( tsk->pid == system_pid);
}

void enable_mdd(void)
{
	enable_hook++;
}
void disable_mdd(void)
{
	enable_hook--;
}

static inline bool is_android_app(struct task_struct *tsk)
{
	return ( tsk->parent &&  tsk->parent->pid == system_pid);
}

static inline bool request_worker(struct task_struct *tsk, struct request *rq)
{
	if (unlikely(tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER)) /* && (rq_data_dir(rq) == READ ) */
		/*&& (tsk->prio <= 100)
		&& (ui_iowait > jiffies)*/)
	{
		return true;
	}
	else
		return false;
}

bool request_boost(struct mdd_data *dd, struct task_struct *tsk, struct request *rq, struct mio_rq_info *rqi)
{
	bool isboost = false;
	struct moto_task_struct *oem_data;
	bool is_top = false;
	/* only do synchronous now*/
	oem_data  = get_moto_task_struct(tsk);
	if ((system_pid <= 0)  && ( tsk->pid > 1000 ))
	{
		if ((tsk->parent) && (!strcmp(tsk->comm, "system_server"))) /* || (!strcmp(tsk->parent->comm, "main")))) */
		{
			system_pid = tsk->parent->pid;
			printk("%s system_pid %d %d\n", __func__, system_pid, tsk->pid);
			srv_pid = tsk->pid;
		}
	}
	if (unlikely(!enable_boost))
		goto output;

	if (system_pid <=0)
		goto output;

	if (!rq_is_sync(rq))
	{
		goto output;
	}
	is_top = task_in_top_app_group(tsk);
	/*
	if ((rq_data_dir(rq) == WRITE) && ( tsk->pid != tsk->tgid))
	{
		goto output;
	}
	*/
	isboost = (is_top && is_android_app(tsk));
	if (isboost)
		goto output;

	isboost = request_worker(tsk, rq);
	if (isboost)
		goto output;
	if (!isboost) {
		// step = 2;
		if ((tsk->pid == srv_pid )
			|| (oem_data->ux_type & UX_TYPE_MASK ))
		{
			isboost = true;
		}
	}
output:
	//mio_log(" boost %d ppid %d ux type 0x%x w:%d top:%d u:%d adj:%d\n", isboost, tsk->tgid, oem_data->ux_type, rq_data_dir(rq), is_top,tsk->cred->uid.val, tsk->signal->oom_score_adj);
	return isboost;
}
void request_finish(struct request *rq, u64 now,  struct mio_rq_info *rqi)
{
	if (rqi->start_time)
	{
		mio_log("ppid %d  sc %u ic %lld \n", rqi->pid,
			jiffies_to_usecs(jiffies - rqi->start_time),  (now - rq->io_start_time_ns));
	}else
		mio_log("ppid %d w:%d sz 0x%x ic %lld \n", rqi->pid, rq_data_dir(rq), rqi->data_size, (now - rq->io_start_time_ns));
}

static void oem_android_vh_free_task_handler(void *unused, struct task_struct *tsk)
{
	if ( tsk && (tsk->pid == system_pid ))
	{
		system_pid = 0;
	}
}


// static void oem_android_rvh_cpu_cgroup_attach(void *unused,
//                         struct cgroup_taskset *tset)
// {
// 	struct task_struct *task;
//     struct cgroup_subsys_state *css;

//     cgroup_taskset_for_each(task, css, tset)
//         mio_log("tsk %d tgid %d, cgroup %d\n", task->pid, task->tgid, css->id);
// }

void iosched_ctl_init(void)
{
	ctl_table_hdr = register_sysctl_table(iosched_base_table);

    register_trace_android_vh_free_task(oem_android_vh_free_task_handler, NULL);
	// register_trace_android_rvh_cpu_cgroup_attach(oem_android_rvh_cpu_cgroup_attach, NULL);
}

void iosched_ctl_deinit(void)
{
	unregister_trace_android_vh_free_task(oem_android_vh_free_task_handler, NULL);
	// unregister_trace_android_rvh_cpu_cgroup_attach(oem_android_rvh_cpu_cgroup_attach, NULL);
	if (ctl_table_hdr)
		unregister_sysctl_table(ctl_table_hdr);
}

