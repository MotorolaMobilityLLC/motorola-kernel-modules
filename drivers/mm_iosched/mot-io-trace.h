// SPDX-License-Identifier: GPL-2.0

#undef TRACE_SYSTEM
#define TRACE_SYSTEM motio

#if !defined(_TRACE_IO_MOT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_IO_MOT_H
#include <linux/version.h>
#include <linux/tracepoint.h>
#include "iosched_util.h"

DECLARE_EVENT_CLASS(rq_sched,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq),

    TP_STRUCT__entry(
        __field(  dev_t,    dev         )
        __field(  sector_t, sector          )
        __field(  unsigned int, nr_sector       )
        __field(  unsigned int, bytes           )
        __field(  int, tag           )
        __array(  char,     rwbs,   RWBS_LEN    )
        __array(  char,         comm,   TASK_COMM_LEN   )
        __dynamic_array( char,  cmd,    1       )
    ),

    TP_fast_assign(
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
        __entry->dev       = rq->q->rq_disk ? disk_devt(rq->q->rq_disk) : 0;
        blk_fill_rwbs_op(__entry->rwbs, rq->cmd_flags);
#else
         __entry->dev       = rq->q->disk ? disk_devt(rq->q->disk) : 0;
         blk_fill_rwbs(__entry->rwbs, rq->cmd_flags);
#endif
        __entry->sector    = blk_rq_trace_sector(rq);
        __entry->nr_sector = blk_rq_trace_nr_sectors(rq);
        __entry->bytes     = blk_rq_bytes(rq);
        __entry->tag	   = rq->internal_tag;

        __get_str(cmd)[0] = '\0';
        memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
    ),

    TP_printk("%d,%d %s %u (%s) %llu + %u [%s] %d",
          MAJOR(__entry->dev), MINOR(__entry->dev),
          __entry->rwbs, __entry->bytes, __get_str(cmd),
          (unsigned long long)__entry->sector,
          __entry->nr_sector, __entry->comm, __entry->tag)
);

DEFINE_EVENT(rq_sched, rq_sched_insert,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);


DEFINE_EVENT(rq_sched, rq_sched_dispatch,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);


DEFINE_EVENT(rq_sched, rq_sched_next,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);

DEFINE_EVENT(rq_sched, rq_sched_complete,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);

DEFINE_EVENT(rq_sched, rq_sched_remove,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);
DEFINE_EVENT(rq_sched, rq_sched_prepare,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);

DEFINE_EVENT(rq_sched, rq_sched_merge,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);

DEFINE_EVENT(rq_sched, rq_sched_finish,

    TP_PROTO(struct request *rq),

    TP_ARGS(rq)
);


TRACE_EVENT(rq_sched_log,

    TP_PROTO(struct request *rq, u8 in_queue, u8 prio, u8 data_dir, pid_t tid),

    TP_ARGS(rq, in_queue, prio, data_dir, tid),

    TP_STRUCT__entry(
        __field(  dev_t,    dev         )
        __field(  sector_t, sector          )
        __field(  u8, in_queue       )
        __field(  u8, prio       )
        __field(  u8, data_dir       )
        __field(  pid_t, tid       )
        __field(  int, tag           )
    ),

    TP_fast_assign(
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
        __entry->dev       = rq->rq_disk ? disk_devt(rq->rq_disk) : 0;
#else
         __entry->dev       = rq->q->disk ? disk_devt(rq->q->disk) : 0;
#endif
        __entry->sector    = blk_rq_pos(rq);
		__entry->in_queue 	   = in_queue;
		__entry->prio 	   = prio;
		__entry->data_dir  = data_dir;
        __entry->tid  = tid;
        __entry->tag	   = rq->internal_tag;

    ),

    TP_printk("%d,%d %llu IN:%u P:%u W:%u [%d] %d",
          MAJOR(__entry->dev), MINOR(__entry->dev),
          (unsigned long long)__entry->sector,
		 __entry->in_queue,
		 __entry->prio,
         __entry->data_dir,
         __entry->tid,  __entry->tag)
);

#endif  /* _TRACE_IO_MOT_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../motorola/kernel/modules/drivers/mm_iosched
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE mot-io-trace


/* This part must be outside protection */
#include <trace/define_trace.h>

