#undef TRACE_SYSTEM
#define TRACE_SYSTEM locking_trace

#if !defined(_LOCKING_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _LOCKING_TRACE_H

#include <linux/tracepoint.h>
#include <linux/version.h>

/*
 * Trace event for rwsem priority/UX inheritance start.
 * Logs the attempt to boost the lock owner's priority.
 */
TRACE_EVENT(rwsem_pi_start,
	TP_PROTO(struct rw_semaphore *sem, struct task_struct *waiter,
		 struct task_struct *owner, bool boosted),
	TP_ARGS(sem, waiter, owner, boosted),

	TP_STRUCT__entry(
		__field(const void *, sem)
		__field(pid_t, waiter_pid)
		__array(char, waiter_comm, TASK_COMM_LEN)
		__field(pid_t, owner_pid)
		__array(char, owner_comm, TASK_COMM_LEN)
		__field(bool, boosted)
	),

	TP_fast_assign(
		__entry->sem = sem;
		__entry->waiter_pid = waiter->pid;
		memcpy(__entry->waiter_comm, waiter->comm, TASK_COMM_LEN);
		__entry->owner_pid = owner ? owner->pid : -1;
		if (owner)
			memcpy(__entry->owner_comm, owner->comm, TASK_COMM_LEN);
		else
			__entry->owner_comm[0] = '\0';
		__entry->boosted = boosted;
	),

	TP_printk("sem=%p waiter=%d(%s) owner=%d(%s) boosted=%d",
		__entry->sem,
		__entry->waiter_pid, __entry->waiter_comm,
		__entry->owner_pid, __entry->owner_comm,
		(int)__entry->boosted
	)
);

/*
 * Trace event for rwsem priority/UX inheritance finish.
 * Logs the clearing of the boost when the waiter acquires the lock.
 */
TRACE_EVENT(rwsem_pi_finish,
	TP_PROTO(struct rw_semaphore *sem, struct task_struct *waiter,
		 struct task_struct *owner),
	TP_ARGS(sem, waiter, owner),

	TP_STRUCT__entry(
		__field(const void *, sem)
		__field(pid_t, waiter_pid)
		__array(char, waiter_comm, TASK_COMM_LEN)
		__field(pid_t, owner_pid)
		__array(char, owner_comm, TASK_COMM_LEN)
	),

	TP_fast_assign(
		__entry->sem = sem;
		__entry->waiter_pid = waiter->pid;
		memcpy(__entry->waiter_comm, waiter->comm, TASK_COMM_LEN);
		__entry->owner_pid = owner ? owner->pid : -1;
		if (owner)
			memcpy(__entry->owner_comm, owner->comm, TASK_COMM_LEN);
		else
			__entry->owner_comm[0] = '\0';
	),

	TP_printk("sem=%p waiter=%d(%s) owner=%d(%s) cleanup",
		__entry->sem,
		__entry->waiter_pid, __entry->waiter_comm,
		__entry->owner_pid, __entry->owner_comm
	)
);

/*
 * NEW: Trace event for generic lock priority/UX inheritance start.
 */
TRACE_EVENT(lock_pi_start,
	TP_PROTO(struct task_struct *owner, struct task_struct *waiter, const char *lock_name),
	TP_ARGS(owner, waiter, lock_name),
	TP_STRUCT__entry(
		__field(pid_t, waiter_pid)
		__array(char, waiter_comm, TASK_COMM_LEN)
		__field(pid_t, owner_pid)
		__array(char, owner_comm, TASK_COMM_LEN)
		__string(lock_name, lock_name)
	),
	TP_fast_assign(
		__entry->waiter_pid = waiter->pid;
		memcpy(__entry->waiter_comm, waiter->comm, TASK_COMM_LEN);
		__entry->owner_pid = owner->pid;
		memcpy(__entry->owner_comm, owner->comm, TASK_COMM_LEN);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
		__assign_str(lock_name);
#else
		__assign_str(lock_name, lock_name);
#endif
	),
	TP_printk("lock=%s waiter=%d(%s) boosted owner=%d(%s)",
		__get_str(lock_name),
		__entry->waiter_pid, __entry->waiter_comm,
		__entry->owner_pid, __entry->owner_comm
	)
);

/*
 * NEW: Trace event for generic lock priority/UX inheritance clear.
 */
TRACE_EVENT(lock_pi_finish,
	TP_PROTO(struct task_struct *owner, const char *lock_name),
	TP_ARGS(owner, lock_name),
	TP_STRUCT__entry(
		__field(pid_t, owner_pid)
		__array(char, owner_comm, TASK_COMM_LEN)
		__string(lock_name, lock_name)
	),
	TP_fast_assign(
		__entry->owner_pid = owner->pid;
		memcpy(__entry->owner_comm, owner->comm, TASK_COMM_LEN);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
		__assign_str(lock_name);
#else
		__assign_str(lock_name, lock_name);
#endif
	),
	TP_printk("lock=%s owner=%d(%s) cleared",
		__get_str(lock_name),
		__entry->owner_pid, __entry->owner_comm
	)
);

TRACE_EVENT(set_user_nice,
    TP_PROTO(struct task_struct *p, long nice, bool allowed),
    TP_ARGS(p, nice, allowed),

    TP_STRUCT__entry(
        __field(pid_t, pid)
        __array(char, comm, TASK_COMM_LEN)
        __field(long, nice)
        __field(bool, allowed)
    ),

    TP_fast_assign(
        __entry->pid = p->pid;
        memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
        __entry->nice = nice;
        __entry->allowed = allowed;
    ),

    TP_printk("pid=%d comm=%s nice=%ld allowed=%d",
        __entry->pid, __entry->comm, __entry->nice, (int)__entry->allowed
    )
);

TRACE_EVENT(locking_debug_trace,
    TP_PROTO(const void *lock, const char *func, const char *reason,
         unsigned long val1, unsigned long val2),
    TP_ARGS(lock, func, reason, val1, val2),

    TP_STRUCT__entry(
        __field(const void *, lock)
        __string(func, func)
        __string(reason, reason)
        __field(unsigned long, val1)
        __field(unsigned long, val2)
    ),

    TP_fast_assign(
        __entry->lock = lock;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
		__assign_str(func);
        __assign_str(reason);
#else
        __assign_str(func, func);
        __assign_str(reason, reason);
#endif
        __entry->val1 = val1;
        __entry->val2 = val2;
    ),

    TP_printk("lock=%p func=%s reason=\"%s\" val1=0x%lx val2=0x%lx",
        __entry->lock, __get_str(func), __get_str(reason),
        __entry->val1, __entry->val2
    )
);

#endif /* _LOCKING_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../motorola/kernel/modules/drivers/moto_sched/locking

/* This part must be outside the header guard */
#include <trace/define_trace.h>



