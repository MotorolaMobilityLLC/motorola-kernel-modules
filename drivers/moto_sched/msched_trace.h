/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Moto. All rights reserved.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM msched

#if !defined(_TRACE_MSCHED_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MSCHED_H

#include <linux/tracepoint.h>
#include <linux/sched.h>

#include <linux/mm_types.h>

TRACE_EVENT(msched_pr_set_vma_name_bypass,
	TP_PROTO(struct task_struct *task, unsigned long addr, unsigned long size),

	TP_ARGS(task, addr, size),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__array(char, comm, TASK_COMM_LEN)
		__field(unsigned long, addr)
		__field(unsigned long, size)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->tgid = task->tgid;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->addr = addr;
		__entry->size = size;
	),

	TP_printk("msched_pr_set_vma_name_bypass: task=%s pid=%d tgid=%d addr=0x%lx size=%lu",
		__entry->comm, __entry->pid, __entry->tgid, __entry->addr, __entry->size)
);

TRACE_EVENT(percpu_rwsem_down_read_preempt,
	TP_PROTO(struct percpu_rw_semaphore *sem,
		bool try, bool ret),

	TP_ARGS(sem, try, ret),

	TP_STRUCT__entry(
		__field(const void *, sem_addr)
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__array(char, comm, TASK_COMM_LEN)
		__array(char, tg_comm, TASK_COMM_LEN)
		__field(bool, try)
		__field(bool, ret)
		__field(int, block)
	),

	TP_fast_assign(
		__entry->sem_addr = sem;
		__entry->pid = current->pid;
		__entry->tgid = current->tgid;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__entry->tg_comm,
			current->group_leader->comm,
			TASK_COMM_LEN);
		__entry->try = try;
		__entry->ret = ret;
		__entry->block = sem ? atomic_read(&sem->block) : -1;
	),

	TP_printk("sem=%p task=%d/%d (%s/%s) try=%d hook_ret=%d block=%d",
		__entry->sem_addr,
		__entry->pid,
		__entry->tgid,
		__entry->comm,
		__entry->tg_comm,
		__entry->try,
		__entry->ret,
		__entry->block
	)
);

TRACE_EVENT(percpu_rwsem_wait_complete,
	TP_PROTO(struct percpu_rw_semaphore *sem,
		int state, bool complete),

	TP_ARGS(sem, state, complete),

	TP_STRUCT__entry(
        	__field(const void *, sem_addr)
        	__field(pid_t, pid)
		__array(char, comm, TASK_COMM_LEN)
		__field(int, state)
		__field(bool, complete)
		__field(int, block)
	),

	TP_fast_assign(
        	__entry->sem_addr = sem;
		__entry->pid = current->pid;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		__entry->state = state;
		__entry->complete = complete;
		__entry->block = sem ? atomic_read(&sem->block) : -1;
	),

	TP_printk("sem=%p task=%d(%s) state=%d complete=%d block=%d",
		__entry->sem_addr,
		__entry->pid,
		__entry->comm,
		__entry->state,
		__entry->complete,
		__entry->block
	)
);

TRACE_EVENT(percpu_rwsem_up_write,
		TP_PROTO(struct percpu_rw_semaphore *sem),

		TP_ARGS(sem),

		TP_STRUCT__entry(
			__field(const void *, sem_addr)
			__field(pid_t, pid)
			__array(char, comm, TASK_COMM_LEN)
			__field(int, block_value_before)
		),

		TP_fast_assign(
			__entry->sem_addr = sem;
			__entry->pid = current->pid;
			memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
			__entry->block_value_before = sem ? atomic_read(&sem->block) : -1;
		),

		TP_printk("sem=%p task=%d(%s) block_before=%d",
			__entry->sem_addr,
			__entry->pid,
			__entry->comm,
			__entry->block_value_before
		)
);

TRACE_EVENT(msched_uclamp_restriction_result,
	TP_PROTO(pid_t pid, enum uclamp_id id,
	         unsigned int req, unsigned int eff,
	         unsigned int max, unsigned int tg_min, unsigned int tg_max,
	         unsigned int pi_min, unsigned int pi_max,
	         unsigned int binder_min, unsigned int binder_max),
	TP_ARGS(pid, id, req, eff, max, tg_min, tg_max, pi_min, pi_max, binder_min, binder_max),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(enum uclamp_id, id)
		__field(unsigned int, req)
		__field(unsigned int, eff)
		__field(unsigned int, max)
		__field(unsigned int, tg_min)
		__field(unsigned int, tg_max)
		__field(unsigned int, pi_min)
		__field(unsigned int, pi_max)
		__field(unsigned int, binder_min)
		__field(unsigned int, binder_max)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->id = id;
		__entry->req = req;
		__entry->eff = eff;
		__entry->max = max;
		__entry->tg_min = tg_min;
		__entry->tg_max = tg_max;
		__entry->pi_min = pi_min;
		__entry->pi_max = pi_max;
		__entry->binder_min = binder_min;
		__entry->binder_max = binder_max;
	),
	TP_printk("pid=%d clamp_id=%d req=%u eff=%u max=%u tg=[%u,%u] pi=[%u,%u] binder=[%u,%u]",
		__entry->pid, __entry->id, __entry->req, __entry->eff, __entry->max,
		__entry->tg_min, __entry->tg_max, __entry->pi_min, __entry->pi_max,
		__entry->binder_min, __entry->binder_max)
);

TRACE_EVENT(msched_uclamp_inheritance_result,
	TP_PROTO(pid_t pid, pid_t pi_pid,
	         unsigned long p_util, u16 p_min, u16 p_max,
	         unsigned long pi_util, u16 pi_min, u16 pi_max,
	         u16 inherited_min, u16 inherited_max, int type),
	TP_ARGS(pid, pi_pid, p_util, p_min, p_max, pi_util, pi_min, pi_max, inherited_min, inherited_max, type),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, pi_pid)
		__field(unsigned long, p_util)
		__field(u16, p_min)
		__field(u16, p_max)
		__field(unsigned long, pi_util)
		__field(u16, pi_min)
		__field(u16, pi_max)
		__field(u16, inherited_min)
		__field(u16, inherited_max)
		__field(int, type)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->pi_pid = pi_pid;
		__entry->p_util = p_util;
		__entry->p_min = p_min;
		__entry->p_max = p_max;
		__entry->pi_util = pi_util;
		__entry->pi_min = pi_min;
		__entry->pi_max = pi_max;
		__entry->inherited_min = inherited_min;
		__entry->inherited_max = inherited_max;
		__entry->type = type;
	),
	TP_printk("p=%d pi=%d p_util=%lu [%u,%u] pi_util=%lu [%u,%u] inherit_min=%u inherit_max=%u type=%d",
		__entry->pid, __entry->pi_pid,
		__entry->p_util, __entry->p_min, __entry->p_max,
		__entry->pi_util, __entry->pi_min, __entry->pi_max,
		__entry->inherited_min, __entry->inherited_max, __entry->type)
);

TRACE_EVENT(msched_task_get_mvp_prio,
        TP_PROTO(struct task_struct *p, int ux_type, int prio_val, unsigned long util, int scene),

        TP_ARGS(p, ux_type, prio_val, util, scene),

        TP_STRUCT__entry(
                __field(pid_t, pid)
                __field(pid_t, tgid)
                __field(int, prio)
                __field(int, ux_type)
                __field(unsigned long, util)
                __field(int, mvp_prio)
                __field(int, scene)
        ),

        TP_fast_assign(
                __entry->pid = p->pid;
                __entry->tgid = p->tgid;
                __entry->prio = p->prio;
                __entry->ux_type = ux_type;
                __entry->util = util;
                __entry->mvp_prio = prio_val;
                __entry->scene = scene;
        ),

        TP_printk("pid=%d tgid=%d prio=%d scene=%d ux_type=%d task_util=%lu mvp_prio=%d",
                __entry->pid, __entry->tgid, __entry->prio, __entry->scene,
                __entry->ux_type, __entry->util, __entry->mvp_prio)
);

TRACE_EVENT(sched_wake_by_irq_kth,

        TP_PROTO(struct task_struct *p, int ux_prio),

        TP_ARGS(p, ux_prio),

        TP_STRUCT__entry(
                __field(pid_t, pid)
                __field(pid_t, tgid)
                __field(int,   prio)
                __array(char,  comm,        TASK_COMM_LEN)
                __field(int,   ux_prio)
        ),

        TP_fast_assign(
                __entry->pid  = p->pid;
                __entry->tgid = p->tgid;
                __entry->prio = p->prio;
                __entry->ux_prio = ux_prio;
                memcpy(__entry->comm,       p->comm,       TASK_COMM_LEN);
        ),

        TP_printk("pid=%d tgid=%d prio=%d comm=%s ux_prio=%d",
                  __entry->pid,
                  __entry->tgid,
                  __entry->prio,
                  __entry->comm,
                  __entry->ux_prio)
);

TRACE_EVENT(sched_boost_ux_kworker,

	TP_PROTO(struct task_struct *p,
		int waker_prio,
		bool is_launcher_wake,
		bool is_top_task,
		int ux_type),

	TP_ARGS(p, waker_prio, is_launcher_wake, is_top_task, ux_type),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__field(int, prio)
		__array(char, comm, TASK_COMM_LEN)

		__array(char, waker_comm, TASK_COMM_LEN)
		__field(int, waker_prio)

		__field(bool, is_launcher_wake)
		__field(bool, is_top_task)
		__field(int, ux_type)
	),

	TP_fast_assign(
		__entry->pid  = p->pid;
		__entry->tgid = p->tgid;
		__entry->prio = p->prio;

		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		memcpy(__entry->waker_comm, current->comm, TASK_COMM_LEN);

		__entry->waker_prio       = waker_prio;
		__entry->is_launcher_wake = is_launcher_wake;
		__entry->is_top_task      = is_top_task;
		__entry->ux_type          = ux_type;
	),

	TP_printk("pid=%d tgid=%d prio=%d comm=%s "
		"waker=%s waker_prio=%d launcher=%d top=%d ux_type=%d",
		__entry->pid,
		__entry->tgid,
		__entry->prio,
		__entry->comm,
		__entry->waker_comm,
		__entry->waker_prio,
		__entry->is_launcher_wake,
		__entry->is_top_task,
		__entry->ux_type)
);

TRACE_EVENT(sched_percpu_rwsem_hold_time,

        TP_PROTO(struct task_struct *p, unsigned long hold_time),

        TP_ARGS(p, hold_time),

        TP_STRUCT__entry(
                __field(pid_t, pid)
                __field(pid_t, tgid)
                __field(int,   prio)
                __array(char,  comm,        TASK_COMM_LEN)
                __field(unsigned long,   hold_time)
        ),

        TP_fast_assign(
                __entry->pid  = p->pid;
                __entry->tgid = p->tgid;
                __entry->prio = p->prio;
                __entry->hold_time = hold_time;

                memcpy(__entry->comm,       p->comm,       TASK_COMM_LEN);
        ),

        TP_printk("pid=%d tgid=%d prio=%d comm=%s hold_time=%lu",
                  __entry->pid,
                  __entry->tgid,
                  __entry->prio,
                  __entry->comm,
                  __entry->hold_time)
);

TRACE_EVENT(sched_percpu_rwsem_starttime,

        TP_PROTO(struct task_struct *p, unsigned char depth, bool acquire, int ux_type, int mvp_prio),

        TP_ARGS(p, depth, acquire, ux_type, mvp_prio),

        TP_STRUCT__entry(
                __field(pid_t, pid)
                __field(int,   prio)
                __array(char,  comm,        TASK_COMM_LEN)
                __field(unsigned char,   depth)
                __field(bool,   acquire)
                __field(int,   ux_type)
                __field(int,   mvp_prio)

        ),

        TP_fast_assign(
                __entry->pid  = p->pid;
                __entry->prio = p->prio;
                memcpy(__entry->comm,       p->comm,       TASK_COMM_LEN);
                __entry->depth = depth;
                __entry->acquire = acquire;
                __entry->ux_type = ux_type;
                __entry->mvp_prio = mvp_prio;
        ),

        TP_printk("pid=%d prio=%d comm=%s depth=%d acquire=%d ux_type=%d mvp=%d",
                  __entry->pid,
                  __entry->prio,
                  __entry->comm,
                  __entry->depth,
                  __entry->acquire,
                  __entry->ux_type,
                  __entry->mvp_prio)
);

TRACE_EVENT(binder_inherit_ux_type,

	TP_PROTO(struct task_struct *task, int ux_type, bool set),

	TP_ARGS(task, ux_type, set),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__field(int, prio)
		__array(char, comm, TASK_COMM_LEN)
		__field(int, ux_type)
		__field(bool, set)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->tgid = task->tgid;
		__entry->prio = task->prio;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->ux_type = ux_type;
		__entry->set = set;
	),

	TP_printk("binder_ux: pid=%d tgid=%d prio=%d comm=%s ux_type=%d set=%d",
		__entry->pid, __entry->tgid, __entry->prio,
		__entry->comm, __entry->ux_type, __entry->set)
);

TRACE_EVENT(binder_pick_best_thread,
	TP_PROTO(int proc_pid, struct task_struct *best_task, int mvp_prio,
				int found_idle, int best_score),
	TP_ARGS(proc_pid, best_task, mvp_prio, found_idle, best_score),
	TP_STRUCT__entry(
		__field(int, proc_pid)
		__field(int, best_pid)
		__field(int, best_prio)
		__field(int, best_mvpprio)
		__array(char, best_comm, TASK_COMM_LEN)
		__field(int, found_idle)
		__field(int, best_score)
	),
	TP_fast_assign(
		__entry->proc_pid = proc_pid;
		if (best_task){
			__entry->best_pid = best_task->pid;
			memcpy(__entry->best_comm, best_task->comm, TASK_COMM_LEN);
			__entry->best_prio = best_task->prio;
		} else {
			__entry->best_pid = -1;
			strscpy(__entry->best_comm, "(null)", TASK_COMM_LEN);
			__entry->best_prio = -1;
		}
		__entry->best_mvpprio = mvp_prio;
		__entry->found_idle = found_idle;
		__entry->best_score = best_score;
	),
	TP_printk("proc=%d best_pid=%d best_comm=%s prio=%d mvpprio=%d idle=%d score=%d",
				__entry->proc_pid,  __entry->best_pid, __entry->best_comm,
				__entry->best_prio, __entry->best_mvpprio,
				__entry->found_idle, __entry->best_score)
);

TRACE_EVENT(binder_nothread_be_select,

	TP_PROTO(struct task_struct *task, int proc, bool epoll),

	TP_ARGS(task, proc, epoll),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__field(int, prio)
		__array(char, comm, TASK_COMM_LEN)
		__field(int, proc)
		__field(bool, epoll)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->tgid = task->tgid;
		__entry->prio = task->prio;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->proc = proc;
		__entry->epoll = epoll;
	),

	TP_printk("pid=%d tgid=%d prio=%d comm=%s proc=%d epoll=%d",
		__entry->pid, __entry->tgid, __entry->prio,
		__entry->comm, __entry->proc, __entry->epoll)
);

TRACE_EVENT(binder_inherit_rt_prio,

	TP_PROTO(struct task_struct *task, struct task_struct *call_task),

	TP_ARGS(task, call_task),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__field(int, prio)
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, call_pid)
		__field(pid_t, call_tgid)
		__field(int, call_prio)
		__array(char, call_comm, TASK_COMM_LEN)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->tgid = task->tgid;
		__entry->prio = task->prio;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->call_pid = call_task->pid;
		__entry->call_tgid = call_task->tgid;
		__entry->call_prio = call_task->prio;
		memcpy(__entry->call_comm, call_task->comm, TASK_COMM_LEN);
	),

	TP_printk("pid=%d tgid=%d prio=%d comm=%s call_pid=%d call_tgid=%d call_prio=%d call_comm=%s",
		__entry->pid, __entry->tgid, __entry->prio, __entry->comm,
		__entry->call_pid, __entry->call_tgid, __entry->call_prio, __entry->call_comm)
);

TRACE_EVENT(binder_inherit_rt_check,

	TP_PROTO(struct task_struct *task, unsigned int policy, int prio, pid_t pid, const char * comm, unsigned long to_thread),

	TP_ARGS(task, policy, prio, pid, comm, to_thread),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, tgid)
		__field(unsigned int, policy)
		__field(int, prio)
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, call_pid)
		__field(unsigned int, call_policy)
		__field(int, call_prio)
		__array(char, call_comm, TASK_COMM_LEN)
		__field(unsigned long, to_thread)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->tgid = task->tgid;
		__entry->policy = task->policy;
		__entry->prio = task->prio;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->call_pid = pid;
		__entry->call_policy = policy;
		__entry->call_prio = prio;
		memcpy(__entry->call_comm, comm, TASK_COMM_LEN);
		__entry->to_thread = to_thread;
	),

	TP_printk("pid=%d tgid=%d policy=%d prio=%d comm=%s call_pid=%d call_policy=%d call_prio=%d call_comm=%s to_thread=%lu",
		__entry->pid, __entry->tgid, __entry->policy, __entry->prio, __entry->comm,
		__entry->call_pid, __entry->call_policy, __entry->call_prio, __entry->call_comm,
		__entry->to_thread)
);
#endif /* _TRACE_MSCHED_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE msched_trace

#include <trace/define_trace.h>
