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

#endif /* _TRACE_MSCHED_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE msched_trace

#include <trace/define_trace.h>
