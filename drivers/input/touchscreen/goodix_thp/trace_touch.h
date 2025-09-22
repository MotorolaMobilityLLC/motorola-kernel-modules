/*
 * Copyright (C) 2025 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM trace_touch

#if !defined(_TRACE_TOUCH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TOUCH_H

#include <linux/tracepoint.h>
#include <linux/trace_events.h>

/* common tracepoints */
TRACE_EVENT(touch_coord,

	TP_PROTO(int x, int y),

	TP_ARGS(x, y),

	TP_STRUCT__entry(
		__field(int, x)
		__field(int, y)
	),

	TP_fast_assign(
		__entry->x = x;
		__entry->y = y;
	),

	TP_printk("touch coord x=%d, y=%d", __entry->x, __entry->y)
);

#endif /* _TRACE_TOUCH_H */
/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../motorola/kernel/modules/drivers/input/touchscreen/goodix_thp
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_touch
#include <trace/define_trace.h>
