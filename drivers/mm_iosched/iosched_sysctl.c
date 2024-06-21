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


#define UX_TYPE_MASK ( UX_TYPE_INHERIT_BINDER | UX_TYPE_TOPAPP|UX_TYPE_LAUNCHER|UX_TYPE_TOPUI |UX_TYPE_INHERIT_LOCK | UX_TYPE_SYSTEM_LOCK \
	 | UX_TYPE_PERF_DAEMON | UX_TYPE_AUDIO |UX_TYPE_AUDIOSERVICE |UX_TYPE_LOW_LATENCY_BINDER | UX_TYPE_NATIVESERVICE | UX_TYPE_KSWAPD )

static struct ctl_table_header *ctl_table_hdr;
int enable_boost = 0;
static pid_t sys_pid[2] = {0,0};

int enable_log = 0;
static int enable_hook = 0;

#define system_pid  sys_pid[0]
static pid_t srv_pid = 0;

static uid_t top_uid = 0;
static pid_t top_pid = 0;

// #define MAX_TOP 3
// struct top_entry
// {
// 	uid_t uid;
// 	pid_t pid;
// };

// static struct top_entry  top_list[MAX_TOP];


struct ctl_table iosched_table[] = {
    {
        .procname   = "enable_io_boost",
        .data       = &enable_boost,
        .maxlen     = sizeof(int),
        .mode       = 0600,
        .proc_handler   = proc_dointvec_minmax,
    },
	{
        .procname   = "enable_log",
        .data       = &enable_log,
        .maxlen     = sizeof(int),
        .mode       = 0600,
        .proc_handler   = proc_dointvec_minmax,
    },
    {
        .procname   = "sys_pid",
        .data       = &sys_pid,
        .maxlen     = 2*sizeof(pid_t),
        .mode       = 0600,
        .proc_handler   = proc_dointvec_minmax,
    },
    {
        .procname   = "top_uid",
        .data       = &top_uid,
        .maxlen     = sizeof(uid_t),
        .mode       = 0600,
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

static inline bool request_worker(struct task_struct *tsk, struct moto_task_struct *oem_data)
{
	if (( tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER )) \
		|| ((tsk->flags & PF_KTHREAD ) && ( tsk->prio < DEFAULT_PRIO )))
	{
		return true;
	}else if  ((tsk->flags & PF_KTHREAD )&& (strstr(tsk->comm, "f2fs_ckpt-")))
	{
		oem_data->ux_type |= UX_TYPE_NATIVESERVICE;
		return true;
	}
	else
		return false;
}

bool request_boost(struct mdd_data *dd, struct task_struct *tsk, bool is_sync, int data_dir)
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
			// pr_info("system_pid %d %d\n",  system_pid, tsk->pid);
			srv_pid = tsk->pid;
		}
	}
	if (unlikely(!enable_boost))
		return false;

	if ((system_pid <=0) && (tsk->cred->uid.val > 10000)  && (tsk->parent->pid != 1))
	{
		system_pid = tsk->parent->pid;
		// pr_info("system_pid %d %d\n", system_pid, tsk->pid);
	}

	if (!is_sync)
	{
		goto output;
	}

	if ((tsk->pid == srv_pid ) \
			|| (oem_data->ux_type & UX_TYPE_MASK ))
	{
			isboost = true;
	}

	// is_top = task_in_tf_app_group(tsk);
	is_top =  (oem_data->ux_type & UX_TYPE_TOPAPP) || task_in_top_app_group(tsk);
	if (isboost && (!top_uid) && is_top  && (tsk->cred->uid.val >= 10000 ))
	{
		/* check top app */
		if ((tsk->pid == tsk->tgid) \
				&& (!(oem_data->ux_type & ( UX_TYPE_SYSUI | UX_TYPE_LAUNCHER ))))
		{
			top_uid = tsk->cred->uid.val;
			top_pid = tsk->tgid;
		}
	}

	isboost  = ( isboost || is_top || ( top_uid  && (top_uid == tsk->cred->uid.val)));
	if (isboost)
		goto output;

	isboost = request_worker(tsk, oem_data);

output:
	// if (isboost)
	// mio_log(" boost %d ppid %d ux type 0x%x w:%d top:%d u:%d %d\n", isboost, tsk->tgid, oem_data->ux_type, rq_data_dir(rq), is_top,tsk->cred->uid.val,top_uid);
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
	if (!tsk)
		return;
	if (tsk->pid == system_pid)
	{
		system_pid = 0;
	}
	else if(tsk->pid == top_pid)
	{
		top_uid = 0;
		top_pid = 0;
	}
}


static void oem_android_rvh_cpu_cgroup_attach(void *unused,
                        struct cgroup_taskset *tset)
{
	struct task_struct *task;
    struct cgroup_subsys_state *css;
	struct moto_task_struct *oem_data;
	// pid_t pid;
	if (unlikely(!enable_boost))
		return;


    cgroup_taskset_first(tset, &css);
    if (!css)
        return;

	//cgroup_taskset_for_each_leader(task, css, tset)
	cgroup_taskset_for_each(task, css, tset)
	{
		oem_data  = get_moto_task_struct(task);
		oem_data->cgr_type = css->id;
		// mio_log("task %d tgid %d, %d, top:%d ux:%x\n", task->pid, task->tgid, task->cred->uid.val, css->id, oem_data->ux_type);//, cgrptg->colocate,  cgrptg->groupid);
		if ( task != task->group_leader)
			continue;
		if ( task->cred->uid.val < 10000 )
			continue;
		if ( oem_data->ux_type & ( UX_TYPE_SYSUI | UX_TYPE_LAUNCHER ) )
			continue;
		if ( oem_data->ux_type & UX_TYPE_TOPAPP )
		{
			top_uid = task->cred->uid.val;
			top_pid = task->tgid;
			continue;
		}

		// // pid = is_top_uid(task->cred->uid.val);
		// // if ((CGROUP_BACKGROUND == css->id) && top_pid) ||
		if ((top_pid == task->tgid) &&  (!( oem_data->ux_type & UX_TYPE_TOPAPP )))
		// if (((CGROUP_BACKGROUND == css->id) && top_pid ) || ((pid == task->tgid) && ( css->id != CGROUP_TOP_APP )))
		{
			top_uid = 0;
			top_pid = 0;
			continue;
		}
		//TODO save top group info
	}
	// cgroup_taskset_for_each(task, css, tset) {
	// 	oem_data  = get_moto_task_struct(task);
	// 	oem_data->cgr_type = get_task_cgroup_id(task);
    // }

}

static void oem_android_rvh_wake_up_new_task(void *unused, struct task_struct *task)
{
	struct moto_task_struct *oem_data;
	if (unlikely(!enable_boost))
		return;
	// if (system_pid <=0)
	// 	return;
	// if (task->parent && (system_pid == task->parent->pid))
	{
		oem_data  = get_moto_task_struct(task);
		// oem_data->cgr_type = CGROUP_TOP_APP;
		oem_data->cgr_type = get_task_cgroup_id(task);
	}
}

void iosched_ctl_init(void)
{
	ctl_table_hdr = register_sysctl_table(iosched_base_table);

	register_trace_android_vh_free_task(oem_android_vh_free_task_handler, NULL);
	register_trace_android_rvh_cpu_cgroup_attach(oem_android_rvh_cpu_cgroup_attach, NULL);
	register_trace_android_rvh_wake_up_new_task(oem_android_rvh_wake_up_new_task, NULL);

}

void iosched_ctl_deinit(void)
{
	unregister_trace_android_vh_free_task(oem_android_vh_free_task_handler, NULL);
	// unregister_trace_android_rvh_cpu_cgroup_attach(oem_android_rvh_cpu_cgroup_attach, NULL);
	if (ctl_table_hdr)
		unregister_sysctl_table(ctl_table_hdr);
}

