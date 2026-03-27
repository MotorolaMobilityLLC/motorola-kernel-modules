#undef TRACE_SYSTEM
#define TRACE_SYSTEM uimem

#if !defined(_UIMEM_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _UIMEM_TRACE_H

#include <linux/tracepoint.h>
#include <linux/stdarg.h>

TRACE_EVENT(pools_status_event,

	/* Accept the format string and a pointer to the va_list */
	TP_PROTO(const char *fmt, va_list *args),

	TP_ARGS(fmt, args),

	TP_STRUCT__entry(
		/*
		 * Dynamically sizes the string buffer based on the formatted
		 * output of 'fmt' and 'args'
		 */
		__vstring(msg, fmt, args)
	),

	TP_fast_assign(
		/* Formats the string and stores it into the ring buffer */
		__assign_vstr(msg, fmt, args);
	),

	TP_printk("%s", __get_str(msg))
);

#endif /* _UIMEM_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE uimem_trace

#include <trace/define_trace.h>
