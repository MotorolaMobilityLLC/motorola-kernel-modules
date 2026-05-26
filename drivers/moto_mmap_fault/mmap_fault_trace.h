/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 Motorola Mobility LLC. All rights reserved.
 * Motorola Mobility Confidential Restricted.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mmap_fault

#if !defined(_TRACE_MMAP_FAULT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MMAP_FAULT_H

#include <linux/tracepoint.h>
#include <linux/sched.h>

TRACE_EVENT(mmap_fault_throttle_bypass,
	TP_PROTO(struct task_struct *p, bool bypass),

	TP_ARGS(p, bypass),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__array(char, comm, TASK_COMM_LEN)
		__field(bool, bypass)
	),

	TP_fast_assign(
		__entry->pid = p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->bypass = bypass;
	),

	TP_printk("pid=%d comm=%s bypass=%d", __entry->pid, __entry->comm, __entry->bypass)
);

#endif /* _TRACE_MMAP_FAULT_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../motorola/kernel/modules/drivers/moto_mmap_fault
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE mmap_fault_trace

#include <trace/define_trace.h>
