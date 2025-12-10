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
                __field(int,   prio)
                __array(char,  comm,        TASK_COMM_LEN)

                __array(char,  waker_comm,  TASK_COMM_LEN)
                __field(int,   waker_prio)

                __field(bool,  is_launcher_wake)
                __field(bool,  is_top_task)
                __field(int,   ux_type)
        ),

        TP_fast_assign(
                __entry->pid  = p->pid;
                __entry->tgid = p->tgid;
                __entry->prio = p->prio;

                memcpy(__entry->comm,       p->comm,       TASK_COMM_LEN);
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

TRACE_EVENT(sched_wake_by_irq_kth,

        TP_PROTO(struct task_struct *p),

        TP_ARGS(p),

        TP_STRUCT__entry(
                __field(pid_t, pid)
                __field(pid_t, tgid)
                __field(int,   prio)
                __array(char,  comm,        TASK_COMM_LEN)
        ),

        TP_fast_assign(
                __entry->pid  = p->pid;
                __entry->tgid = p->tgid;
                __entry->prio = p->prio;

                memcpy(__entry->comm,       p->comm,       TASK_COMM_LEN);
        ),

        TP_printk("pid=%d tgid=%d prio=%d comm=%s",
                  __entry->pid,
                  __entry->tgid,
                  __entry->prio,
                  __entry->comm)
);
#endif /* _TRACE_MSCHED_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE msched_trace

#include <trace/define_trace.h>
