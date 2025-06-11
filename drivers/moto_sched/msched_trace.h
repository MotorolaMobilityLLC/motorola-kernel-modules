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

#endif /* _TRACE_MSCHED_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE msched_trace

#include <trace/define_trace.h>
