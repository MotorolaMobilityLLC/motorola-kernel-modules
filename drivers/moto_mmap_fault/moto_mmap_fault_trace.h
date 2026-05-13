#undef TRACE_SYSTEM
#define TRACE_SYSTEM moto_mmap_fault

#if !defined(_TRACE_MOTO_MMAP_FAULT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MOTO_MMAP_FAULT_H

#include <linux/tracepoint.h>
#include <linux/sched.h>

TRACE_EVENT(moto_disable_fault_around,

	TP_PROTO(unsigned long address, int prio),

	TP_ARGS(address, prio),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__field(int, prio)
		__field(unsigned long, address)
	),

	TP_fast_assign(
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		__entry->pid = current->pid;
		__entry->prio = prio;
		__entry->address = address;
	),

	TP_printk("disable_fault_around comm=%s pid=%d prio=%d address=%#lx",
		__entry->comm, __entry->pid, __entry->prio, __entry->address)
);

#endif /* _TRACE_MOTO_MMAP_FAULT_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../motorola/kernel/modules/drivers/moto_mmap_fault
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE moto_mmap_fault_trace

#include <trace/define_trace.h>
