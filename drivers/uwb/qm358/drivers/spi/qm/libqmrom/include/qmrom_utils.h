/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#ifndef __QMROM_UTILS_H__
#define __QMROM_UTILS_H__

#if defined(__KERNEL__)
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/slab.h>
#else /* __KERNEL__ */
#include <stdlib.h>
#include <unistd.h>
#endif /* __KERNEL__ */

/******************/
/* dynamic memory */
/******************/

#if defined(__KERNEL__)

#define qmrom_alloc(ptr, size)                               \
	do {                                                 \
		ptr = kzalloc((size), GFP_KERNEL | GFP_DMA); \
	} while (0)
#define qmrom_free kfree
#define qmrom_data_dma_able(d) (!is_vmalloc_addr(d))

#else /* __KERNEL__ */

#define qmrom_alloc(ptr, size)         \
	do {                           \
		ptr = calloc(1, size); \
	} while (0)
#define qmrom_free free
#define qmrom_data_dma_able(d) true

#endif /* __KERNEL__ */

/*********/
/* sleep */
/*********/

#if defined(__KERNEL__)

#define qmrom_usleep(us) usleep_range(us, (us)*11 / 10)
#define qmrom_msleep(ms) usleep_range((ms)*1000, (ms)*1100)

#else /* __KERNEL__ */

#define qmrom_usleep(us) usleep(us)
#define qmrom_msleep(ms) usleep((ms)*1000)

#endif /* __KERNEL__ */

/********/
/* time */
/********/

#if defined(__KERNEL__)

#define qmrom_time ktime_t
#define qmrom_time_get ktime_get
#define qmrom_time_sub ktime_sub
#define qmrom_time_div ktime_divns
#define qmrom_time_to_ns ktime_to_ns

#else /* __KERNEL__ */

#define qmrom_time uint64_t
#define qmrom_time_sub(a, b) ((a) - (b))
#define qmrom_time_div(a, b) ((a) / (b))

/* XCode 8 implements clock_gettime() and defines CLOCK_MONOTONIC_RAW,
 * even if it does not define _POSIX_TIMERS. */
#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0 || defined(__APPLE__)

#include <time.h>

static inline qmrom_time qmrom_time_get()
{
	struct timespec time;
	clock_gettime(CLOCK_MONOTONIC_RAW, &time);
	return time.tv_sec * 1e9 + time.tv_nsec;
}
#define qmrom_time_to_ns(t) (t)

#elif defined(_WIN32) /* _POSIX_TIMERS */

#include <sysinfoapi.h>

static inline qmrom_time qmrom_time_get()
{
	FILETIME time;
	GetSystemTimeAsFileTime(&time);
	return ((qmrom_time)time.dwHighDateTime << 32) + time.dwLowDateTime;
}
#define qmrom_time_to_ns(t) ((t)*100)

#else /* _POSIX_TIMERS */
#error "_POSIX_TIMERS not defined on this platform."
#endif /* _POSIX_TIMERS */

#endif /* __KERNEL__ */

/*******/
/* min */
/*******/

#if !defined(min)

#define min(a, b)                       \
	({                              \
		__typeof__(a) _a = (a); \
		__typeof__(b) _b = (b); \
		_a < _b ? _a : _b;      \
	})

#endif

#endif /* __QMROM_UTILS_H__ */
