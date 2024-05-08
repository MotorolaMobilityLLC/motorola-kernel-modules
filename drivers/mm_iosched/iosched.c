 // SPDX-License-Identifier: GPL-2.0
/*
 *
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
#include <linux/elevator.h>
#else
#include <block/elevator.h>
#endif
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/sbitmap.h>

#include <trace/events/block.h>
#define CREATE_TRACE_POINTS
#include "mot-io-trace.h"
#undef CREATE_TRACE_POINTS


#include "mio.h"


static const int read_expire = HZ / 2;  /* max time before a read is submitted. */
static const int write_expire = 5 * HZ; /* ditto for writes, these limits are SOFT! */
/*
 * Time after which to dispatch lower priority requests even if higher
 * priority requests are pending.
 */
static const int prio_aging_expire = 4 * HZ;
static const int writes_starved = 2;    /* max times reads can starve a write */
static const int fifo_batch = 16;       /* # of sequential requests treated as one
				     by the above parameters. For throughput. */

#define IO_TYPES (REQ_OP_DISCARD + 1)

static const char * const op_labels[] = {
    [REQ_OP_READ] = "read",
    [REQ_OP_WRITE] = "write",
};


static unsigned int byte_table[] = {
    4096, // 4KB
    32768, // 32KB
    65536, // 64KB
    131072, // 128KB
    524288, // 512KB
    1024*1024*2, // 2MB
    UINT_MAX // should be last in this array
};
#define BYTE_TABLE_SIZE (sizeof(byte_table)/sizeof(unsigned int))

static u64 nsec_table[] = {
    500000, // 0.5ms
    1000000, // 1ms
    2000000, // 2ms
    3000000, // 3ms
    4000000, // 4ms
    5000000, // 5ms
    10000000, // 10ms
    20000000, // 20ms

    ULLONG_MAX // should be last in this array
};



#define NSEC_TABLE_SIZE (sizeof(nsec_table)/sizeof(u64))

struct mio_latency {
    u64 io_latency_cnt[DD_DIR_COUNT][BYTE_TABLE_SIZE][NSEC_TABLE_SIZE];
};

struct mio_request_info {
	u16 data;
	u8 io_class;
	u8 reserved;
	int data_size;
	struct task_struct* tsk;
};


/* Maps an I/O priority class to a deadline scheduler priority. */
#define IOPRIO_CLASS_TB IOPRIO_CLASS_IDLE+1
static const enum mdd_prio ioprio_class_to_prio[] = {
	[IOPRIO_CLASS_NONE]	= DD_BE_PRIO,
	[IOPRIO_CLASS_RT]	= DD_RT_PRIO,
	[IOPRIO_CLASS_TB]	= DD_TB_PRIO,
	[IOPRIO_CLASS_BE]	= DD_BE_PRIO,
	[IOPRIO_CLASS_IDLE]	= DD_IDLE_PRIO,
};

static unsigned int byte_to_index(unsigned int byte)
{
    unsigned int idx;

    for (idx = 0; idx < BYTE_TABLE_SIZE; idx++)
        if (byte <= byte_table[idx])
            return idx;

    return BYTE_TABLE_SIZE - 1;
}

static unsigned int nsec_to_index(u64 nsec)
{
    unsigned int idx;

    for (idx = 0; idx < NSEC_TABLE_SIZE; idx++)
        if (nsec <= nsec_table[idx])
            return idx;

    return NSEC_TABLE_SIZE - 1;
}

static void update_io_latency(struct mdd_data *dd, struct request *rq,
        unsigned int data_size, u64 now, struct mio_rq_info *rqi)
{
    struct mio_latency *lats = this_cpu_ptr(dd->io_latency);
    int type, byte_idx, ns_idx;

	const enum mdd_data_dir data_dir = rq_data_dir(rq);

    if (rq->io_start_time_ns > now)
        return;
    type = data_dir;
    byte_idx = byte_to_index(data_size);
    ns_idx = nsec_to_index(now - rq->io_start_time_ns);
    lats->io_latency_cnt[type][byte_idx][ns_idx]++;
	if ( ns_idx >= (NSEC_TABLE_SIZE-2 ) && type == DD_READ)
		mio_log("bytes %x ns %d idx %d flags 0x%x  p:%d dur %lld, %d\n", data_size, ns_idx, byte_idx, rq->cmd_flags, rqi->pid, (now - rq->io_start_time_ns), rq->internal_tag);
}

static inline struct mio_request_info *get_mio_request_info(struct request *rq)
{
	return (struct mio_request_info*)rq->elv.priv;
}

static u32 mdd_owned_by_driver(struct mdd_data *dd, enum mdd_prio prio);

static struct mio_rq_info *get_mio_rq_info(struct mdd_data *dd, struct request *rq)
{
	if (dd->nr_requests > rq->internal_tag)
		return &dd->rqs[rq->internal_tag];
	else {
		//mio_log(" internal_tag %d\n", rq->internal_tag);
		return NULL;
	}
}

static int is_task_nr_rq_more(struct mdd_data *dd, struct task_struct *tsk, int max)
{
	int i;
	int nr = 0;

	for ( i=0; i<dd->nr_requests; i++ )
	{
		if (!dd->rqs[i].pid)
		{
			continue;
		}
		if ( tsk->tgid == dd->rqs[i].pid)
		{
			if (++nr > max)
				break;
		}
	}

	return (nr > max);
}

static inline struct rb_root *
__mdd_rb_root(struct mdd_per_prio *per_prio, struct request *rq)
{
	return &per_prio->sort_list[rq_data_dir(rq)];
}

/*
 * Returns the I/O priority class (IOPRIO_CLASS_*) that has been assigned to a
 * request.
 */
static u8 mdd_rq_ioclass(struct mdd_data *dd , struct request *rq)
{
	struct mio_rq_info *mrq =  get_mio_rq_info(dd, rq);

	return mrq?mrq->io_class:(uintptr_t)(rq->elv.priv[1]);
}

/*
 * get the request before `rq' in sector-sorted order
 */
static inline struct request *
__mdd_earlier_request(struct request *rq)
{
	struct rb_node *node = rb_prev(&rq->rb_node);

	if (node)
		return rb_entry_rq(node);

	return NULL;
}

/*
 * get the request after `rq' in sector-sorted order
 */
static inline struct request *
__mdd_latter_request(struct request *rq)
{
	struct rb_node *node = rb_next(&rq->rb_node);

	if (node)
		return rb_entry_rq(node);

	return NULL;
}

static void
__mdd_add_rq_rb(struct mdd_per_prio *per_prio, struct request *rq)
{
	struct rb_root *root = __mdd_rb_root(per_prio, rq);

	elv_rb_add(root, rq);
}

static inline void
__mdd_del_rq_rb(struct mdd_per_prio *per_prio, struct request *rq)
{
	const enum mdd_data_dir data_dir = rq_data_dir(rq);

	if (per_prio->next_rq[data_dir] == rq)
		per_prio->next_rq[data_dir] = __mdd_latter_request(rq);

	elv_rb_del(__mdd_rb_root(per_prio, rq), rq);
}

/*
 * remove rq from rbtree and fifo.
 */
static void __mdd_remove_request(struct request_queue *q,
				    struct mdd_per_prio *per_prio,
				    struct request *rq)
{
	list_del_init(&rq->queuelist);

	/*
	 * We might not be on the rbtree, if we are doing an insert merge
	 */
	if (!RB_EMPTY_NODE(&rq->rb_node))
		__mdd_del_rq_rb(per_prio, rq);

	elv_rqhash_del(q, rq);
	if (q->last_merge == rq)
		q->last_merge = NULL;
}

static void mdd_request_merged(struct request_queue *q, struct request *req,
			      enum elv_merge type)
{
	struct mdd_data *dd = q->elevator->elevator_data;
	const u8 ioprio_class = mdd_rq_ioclass(dd, req);
	const enum mdd_prio prio = ioprio_class_to_prio[ioprio_class];
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];

	/*
	 * if the merge was a front merge, we need to reposition request
	 */
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(__mdd_rb_root(per_prio, req), req);
		__mdd_add_rq_rb(per_prio, req);
	}
}

/*
 * Callback function that is invoked after @next has been merged into @req.
 */
static void mdd_merged_requests(struct request_queue *q, struct request *req,
			       struct request *next)
{
	struct mdd_data *dd = q->elevator->elevator_data;
	const u8 ioprio_class = mdd_rq_ioclass(dd, next);
	const enum mdd_prio prio = ioprio_class_to_prio[ioprio_class];

	lockdep_assert_held(&dd->lock);

	dd->per_prio[prio].stats.merged++;

	/*
	 * if next expires before rq, assign its expire time to rq
	 * and move into next position (next will be deleted) in fifo
	 */
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before((unsigned long)next->fifo_time,
				(unsigned long)req->fifo_time)) {
			list_move(&req->queuelist, &next->queuelist);
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	 * kill knowledge of next, this one is a goner
	 */
	__mdd_remove_request(q, &dd->per_prio[prio], next);
}

/*
 * move an entry to dispatch queue
 */
static void
__mdd_move_request(struct mdd_data *dd, struct mdd_per_prio *per_prio,
		      struct request *rq)
{
	const enum mdd_data_dir data_dir = rq_data_dir(rq);

	per_prio->next_rq[data_dir] = __mdd_latter_request(rq);

	/*
	 * take it off the sort and fifo list
	 */
	__mdd_remove_request(rq->q, per_prio, rq);
	//trace_rq_sched_remove(rq);
}

/* Number of requests queued for a given priority level. */
static u32 mdd_queued(struct mdd_data *dd, enum mdd_prio prio)
{
	const struct io_stats_per_prio *stats = &dd->per_prio[prio].stats;

	lockdep_assert_held(&dd->lock);

	return stats->inserted - atomic_read(&stats->completed);
}

/*
 * __mdd_check_fifo returns 0 if there are no expired requests on the fifo,
 * 1 otherwise. Requires !list_empty(&dd->fifo_list[data_dir])
 */
static inline int __mdd_check_fifo(struct mdd_per_prio *per_prio,
				      enum mdd_data_dir data_dir)
{
	struct request *rq = rq_entry_fifo(per_prio->fifo_list[data_dir].next);

	/*
	 * rq is expired!
	 */
	if (time_after_eq(jiffies, (unsigned long)rq->fifo_time))
		return 1;

	return 0;
}

/*
 * Check if rq has a sequential request preceding it.
 */
static bool __mdd_is_seq_write(struct mdd_data *dd, struct request *rq)
{
	struct request *prev = __mdd_earlier_request(rq);

	if (!prev)
		return false;

	return blk_rq_pos(prev) + blk_rq_sectors(prev) == blk_rq_pos(rq);
}

/*
 * Skip all write requests that are sequential from @rq, even if we cross
 * a zone boundary.
 */
static struct request *__mdd_skip_seq_writes(struct mdd_data *dd,
						struct request *rq)
{
	sector_t pos = blk_rq_pos(rq);
	sector_t skipped_sectors = 0;

	while (rq) {
		if (blk_rq_pos(rq) != pos + skipped_sectors)
			break;
		skipped_sectors += blk_rq_sectors(rq);
		rq = __mdd_latter_request(rq);
	}

	return rq;
}

/*
 * For the specified data direction, return the next request to
 * dispatch using arrival ordered lists.
 */
static struct request *
__mdd_fifo_request(struct mdd_data *dd, struct mdd_per_prio *per_prio,
		      enum mdd_data_dir data_dir)
{
	struct request *rq;
	unsigned long flags;

	if (list_empty(&per_prio->fifo_list[data_dir]))
		return NULL;

	rq = rq_entry_fifo(per_prio->fifo_list[data_dir].next);
	if (data_dir == DD_READ || !blk_queue_is_zoned(rq->q))
		return rq;

	/*
	 * Look for a write request that can be dispatched, that is one with
	 * an unlocked target zone. For some HDDs, breaking a sequential
	 * write stream can lead to lower throughput, so make sure to preserve
	 * sequential write streams, even if that stream crosses into the next
	 * zones and these zones are unlocked.
	 */
	spin_lock_irqsave(&dd->zone_lock, flags);
	list_for_each_entry(rq, &per_prio->fifo_list[DD_WRITE], queuelist) {
		if (blk_req_can_dispatch_to_zone(rq) &&
		    (blk_queue_nonrot(rq->q) ||
		     !__mdd_is_seq_write(dd, rq)))
			goto out;
	}
	rq = NULL;
out:
	spin_unlock_irqrestore(&dd->zone_lock, flags);

	return rq;
}

/*
 * For the specified data direction, return the next request to
 * dispatch using sector position sorted lists.
 */
static struct request *
__mdd_next_request(struct mdd_data *dd, struct mdd_per_prio *per_prio,
		      enum mdd_data_dir data_dir)
{
	struct request *rq;
	unsigned long flags;

	rq = per_prio->next_rq[data_dir];
	if (!rq)
		return NULL;
	if (data_dir == DD_READ || !blk_queue_is_zoned(rq->q))
		return rq;

	/*
	 * Look for a write request that can be dispatched, that is one with
	 * an unlocked target zone. For some HDDs, breaking a sequential
	 * write stream can lead to lower throughput, so make sure to preserve
	 * sequential write streams, even if that stream crosses into the next
	 * zones and these zones are unlocked.
	 */
	spin_lock_irqsave(&dd->zone_lock, flags);
	while (rq) {
		if (blk_req_can_dispatch_to_zone(rq))
			break;
		if (blk_queue_nonrot(rq->q))
			rq = __mdd_latter_request(rq);
		else
			rq = __mdd_skip_seq_writes(dd, rq);
	}
	spin_unlock_irqrestore(&dd->zone_lock, flags);

	return rq;
}

/*
 * Returns true if and only if @rq started after @latest_start where
 * @latest_start is in jiffies.
 */
static bool started_after(struct mdd_data *dd, struct request *rq,
			  unsigned long latest_start)
{
	unsigned long start_time = (unsigned long)rq->fifo_time;

	start_time -= dd->fifo_expire[rq_data_dir(rq)];

	return time_after(start_time, latest_start);
}

/*
 * __mdd_dispatch_requests selects the best request according to
 * read/write expire, fifo_batch, etc and with a start time <= @latest_start.
 */
static struct request *__mdd_dispatch_request(struct mdd_data *dd,
					     struct mdd_per_prio *per_prio,
					     unsigned long latest_start)
{
	struct request *rq, *next_rq;
	enum mdd_data_dir data_dir = 0;
	enum mdd_prio prio;
	u8 ioprio_class;

	lockdep_assert_held(&dd->lock);

	if (!list_empty(&per_prio->dispatch)) {
		rq = list_first_entry(&per_prio->dispatch, struct request,
				      queuelist);
		if (started_after(dd, rq, latest_start))
			return NULL;
		list_del_init(&rq->queuelist);
		goto done;
	}

	/*
	 * batches are currently reads XOR writes
	 */
	if (per_prio->fifo_only )
	{
		rq = __mdd_fifo_request(dd, per_prio, dd->last_dir);
	}else
	{
		rq = __mdd_next_request(dd, per_prio, dd->last_dir);
	}
	if (rq && dd->batching < dd->fifo_batch) {
		/* we have a next request are still entitled to batch */
		goto dispatch_request;
	}

	/*
	 * at this point we are not running a batch. select the appropriate
	 * data direction (read / write)
	 */

	if (!list_empty(&per_prio->fifo_list[DD_READ])) {
		BUG_ON(RB_EMPTY_ROOT(&per_prio->sort_list[DD_READ]));

		if (__mdd_fifo_request(dd, per_prio, DD_WRITE) &&
		    (dd->starved++ >= dd->writes_starved))
			goto dispatch_writes;

		data_dir = DD_READ;
		goto dispatch_find_request;
	}

	/*
	 * there are either no reads or writes have been starved
	 */

	if (!list_empty(&per_prio->fifo_list[DD_WRITE])) {
dispatch_writes:
		BUG_ON(RB_EMPTY_ROOT(&per_prio->sort_list[DD_WRITE]));

		dd->starved = 0;

		data_dir = DD_WRITE;
		goto dispatch_find_request;
	}

	return NULL;

dispatch_find_request:
	/*
	 * we are not running a batch, find best request for selected data_dir
	 */
	if (per_prio->fifo_only )
	{
		rq = __mdd_fifo_request(dd, per_prio, data_dir);
	}else
	{
		next_rq = __mdd_next_request(dd, per_prio, data_dir);
		if (__mdd_check_fifo(per_prio, data_dir) || !next_rq) {
			/*
			* A deadline has expired, the last request was in the other
			* direction, or we have run out of higher-sectored requests.
			* Start again from the request with the earliest expiry time.
			*/
			rq = __mdd_fifo_request(dd, per_prio, data_dir);
		} else {
			/*
			* The last req was the same dir and we have a next request in
			* sort order. No expired requests so continue on from here.
			*/
			rq = next_rq;
		}
	}
	/*
	 * For a zoned block device, if we only have writes queued and none of
	 * them can be dispatched, rq will be NULL.
	 */
	if (!rq)
		return NULL;

	dd->last_dir = data_dir;
	dd->batching = 0;

dispatch_request:
	if (started_after(dd, rq, latest_start))
		return NULL;

	/*
	 * rq is the selected appropriate request.
	 */
	dd->batching++;
	__mdd_move_request(dd, per_prio, rq);
done:
	ioprio_class = mdd_rq_ioclass(dd, rq);
	prio = ioprio_class_to_prio[ioprio_class];
	dd->per_prio[prio].stats.dispatched++;
	if (prio != DD_RT_PRIO ) {
		if ( dd->last_prio == prio )
			dd->per_prio[prio].stats.dispatching++;
		else {
			dd->per_prio[prio].stats.dispatching = 1;
			dd->per_prio[dd->last_prio].stats.dispatching = 0;
			dd->last_prio = prio;
		}
	}
	/*
	 * If the request needs its target zone locked, do it.
	 */
	blk_req_zone_write_lock(rq);
	rq->rq_flags |= RQF_STARTED;
	return rq;
}

/*
 * Check whether there are any requests with priority other than DD_RT_PRIO
 * that were inserted more than prio_aging_expire jiffies ago.
 */
static struct request *mdd_dispatch_prio_aged_requests(struct mdd_data *dd,
						      unsigned long now)
{
	struct request *rq;
	enum mdd_prio prio;
	int prio_cnt;

	lockdep_assert_held(&dd->lock);

	prio_cnt = !!mdd_queued(dd, DD_RT_PRIO) + !!mdd_queued(dd, DD_BE_PRIO) + !!mdd_queued(dd, DD_TB_PRIO) +
		   !!mdd_queued(dd, DD_IDLE_PRIO);
	if (prio_cnt < 2)
		return NULL;

	for (prio = DD_BE_PRIO; prio <= DD_PRIO_MAX; prio++) {
		if ( dd->per_prio[DD_TB_PRIO].stats.dispatching >= 64 )
		{
			rq = __mdd_dispatch_request(dd, &dd->per_prio[prio],
						now - (HZ>>1));
		}else
		rq = __mdd_dispatch_request(dd, &dd->per_prio[prio],
					   now - dd->prio_aging_expire);

		if (rq) {
			return rq;
		}
	}

	return NULL;
}

/*
 * Called from blk_mq_run_hw_queue() -> __blk_mq_sched_dispatch_requests().
 *
 * One confusing aspect here is that we get called for a specific
 * hardware queue, but we may return a request that is for a
 * different hardware queue. This is because mq-deadline has shared
 * state for all hardware queues, in terms of sorting, FIFOs, etc.
 */
static struct request *mdd_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
	struct mdd_data *dd = hctx->queue->elevator->elevator_data;
	const unsigned long now = jiffies;
	struct request *rq;
	enum mdd_prio prio;

	spin_lock(&dd->lock);
	rq = mdd_dispatch_prio_aged_requests(dd, now);
	if (rq)
		goto unlock;

	/*
	 * Next, dispatch requests in priority order. Ignore lower priority
	 * requests if any higher priority requests are pending.
	 */
	for (prio = 0; prio <= DD_PRIO_MAX; prio++) {
		rq = __mdd_dispatch_request(dd, &dd->per_prio[prio], now);
		if (rq || mdd_queued(dd, prio)) {
			break;
		}
	}

unlock:
	spin_unlock(&dd->lock);
	if (rq) {
		struct mio_rq_info *mrq;
		mrq = get_mio_rq_info(dd, rq);
		if (likely(mrq))
			mrq->data_size = blk_rq_bytes(rq);

		trace_rq_sched_dispatch(rq);
	}
	// mio_log(" in_queue %d  rq %p\n", atomic_read(&dd->in_queue_rqs) , rq);
	return rq;
}

static void mdd_completed_request(struct request *rq, u64 now)
{
	struct mdd_data *dd = rq->q->elevator->elevator_data;
	struct mio_rq_info *mrq;
	mrq = get_mio_rq_info(dd, rq);

	if (!dd->latency )
	{
		return;
	}
	trace_rq_sched_complete(rq);
	if (( rq->rq_flags & RQF_FLUSH_SEQ) || (blk_rq_pos(rq) == -1 ))
	{
		return;
	}

	//valid date size
	if (likely(mrq) && mrq->data_size)
	{
		update_io_latency(dd, rq, mrq->data_size, now, mrq);
		request_finish(rq, now, mrq);
		mrq->data_size = 0;
	}
}

/*
 * Called by __blk_mq_alloc_request(). The shallow_depth value set by this
 * function is used by __blk_mq_get_tag().
 */
static void mdd_limit_depth(blk_opf_t opf, struct blk_mq_alloc_data *data)
{
	struct mdd_data *dd = data->q->elevator->elevator_data;

	/* Do not throttle synchronous reads. */
	if (op_is_sync(opf) && !op_is_write(opf)){

		if (!is_enabled_boost())
			return;
		if (task_in_top_app_group(current)) {
			return;
		}
		if (atomic_read(&dd->in_queue_rqs ) > (dd->nr_threshold_rqs))
		{
			if (is_task_nr_rq_more(dd, current, (dd->nr_requests >>1 )))
			{
				data->shallow_depth = max(dd->async_depth,  mio_blkcg_shallow_depth(data->q)>>1);
			}
		}
		// if (data->shallow_depth)
			// mio_log(" non top pid %d depth %d\n", current->tgid, data->shallow_depth);
		return;
	}

	/*
	 * Throttle asynchronous requests and writes such that these requests
	 * do not block the allocation of synchronous requests.
	 */
	data->shallow_depth = max(dd->async_depth, mio_blkcg_shallow_depth(data->q));
	// mio_log("limit_depth pid %d depth %d\n", current->tgid, data->shallow_depth);
}

/* Called by blk_mq_update_nr_requests(). */
static void mdd_depth_updated(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	struct mdd_data *dd = q->elevator->elevator_data;
	struct blk_mq_tags *tags = hctx->sched_tags;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
	unsigned int shift = tags->bitmap_tags->sb.shift;
	int depth = tags->bitmap_tags->sb.depth;
#else
	unsigned int shift = tags->bitmap_tags.sb.shift;
	int depth = tags->bitmap_tags.sb.depth;
#endif
	if (dd->nr_requests <  depth)
	{
		dd->rqs = krealloc_array(dd->rqs,  depth,
			sizeof(struct mio_rq_info),
			GFP_KERNEL | __GFP_ZERO);
		dd->nr_requests = depth;
		dd->nr_threshold_rqs = dd->nr_requests * 3 /4;
	}
	dd->async_depth = max(1U,  (1U << shift)  / 4);
	mio_blkcg_depth_updated(hctx);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
	sbitmap_queue_min_shallow_depth(tags->bitmap_tags, dd->async_depth);
#else
 	sbitmap_queue_min_shallow_depth(&tags->bitmap_tags, dd->async_depth);
#endif
}

/* Called by blk_mq_init_hctx() and blk_mq_init_sched(). */
static int mdd_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	mdd_depth_updated(hctx);
	return 0;
}

static void mdd_exit_sched(struct elevator_queue *e)
{
	struct mdd_data *dd = e->elevator_data;
	enum mdd_prio prio;

	mio_blkcg_deactivate(dd->queue);
	for (prio = 0; prio <= DD_PRIO_MAX; prio++) {
		struct mdd_per_prio *per_prio = &dd->per_prio[prio];
		const struct io_stats_per_prio *stats = &per_prio->stats;
		uint32_t queued;

		WARN_ON_ONCE(!list_empty(&per_prio->fifo_list[DD_READ]));
		WARN_ON_ONCE(!list_empty(&per_prio->fifo_list[DD_WRITE]));

		spin_lock(&dd->lock);
		queued = mdd_queued(dd, prio);
		spin_unlock(&dd->lock);

		WARN_ONCE(queued != 0,
			"statistics for priority %d: i %u m %u d %u c %u b %u\n",
			  prio, stats->inserted, stats->merged,
			  stats->dispatched, atomic_read(&stats->completed), dd->boosted);
	}
	printk(" %s boost %d \n", __func__, dd->boosted);
    free_percpu(dd->io_latency);
	kfree(dd->rqs);
	kfree(dd);
	disable_mdd();
}

/*
 * initialize elevator private data (mdd_data).
 */
static int mdd_init_sched(struct request_queue *q, struct elevator_type *e)
{
	struct mdd_data *dd;
	struct elevator_queue *eq;
	enum mdd_prio prio;
	//struct blk_mq_hw_ctx *hctx;
	int ret = -ENOMEM;

	eq = elevator_alloc(q, e);
	if (!eq)
		return ret;

	dd = kzalloc_node(sizeof(*dd), GFP_KERNEL, q->node);
	if (!dd)
		goto put_eq;

	eq->elevator_data = dd;

	for (prio = 0; prio <= DD_PRIO_MAX; prio++) {
		struct mdd_per_prio *per_prio = &dd->per_prio[prio];

		INIT_LIST_HEAD(&per_prio->dispatch);
		INIT_LIST_HEAD(&per_prio->fifo_list[DD_READ]);
		INIT_LIST_HEAD(&per_prio->fifo_list[DD_WRITE]);
		per_prio->sort_list[DD_READ] = RB_ROOT;
		per_prio->sort_list[DD_WRITE] = RB_ROOT;
		per_prio->prio = prio;
		per_prio->fifo_only = 0;
	}
	dd->per_prio[DD_TB_PRIO].fifo_only = 1;
	dd->fifo_expire[DD_READ] = read_expire;
	dd->fifo_expire[DD_WRITE] = write_expire;
	dd->writes_starved = writes_starved;
	dd->front_merges = 1;
	dd->last_dir = DD_WRITE;
	dd->fifo_batch = fifo_batch;
	dd->prio_aging_expire = prio_aging_expire;
	dd->queue = q;
	spin_lock_init(&dd->lock);
	spin_lock_init(&dd->zone_lock);
	atomic_set(&dd->in_queue_rqs, 0);

	dd->io_latency = alloc_percpu_gfp(struct mio_latency,
            GFP_KERNEL | __GFP_ZERO);
	if ( !dd->io_latency) {
		kfree(dd);
		goto put_eq;
	}
	dd->nr_requests = q->nr_requests;
	dd->nr_threshold_rqs = dd->nr_requests * 4 / 5 ;
	dd->last_prio = 0;
	dd->dispatch_fifo = 0;
	dd->latency = 0;

	//hctx = ((struct blk_mq_hw_ctx*)xa_load(&q->hctx_table, 0));
	dd->rqs = kmalloc_array(q->nr_requests,
			sizeof(struct mio_rq_info),
			GFP_KERNEL | __GFP_ZERO);
	if (!(dd->rqs)) {
		printk("alloc rq info failed!\n");
		free_percpu(dd->io_latency);
		kfree(dd);
		goto put_eq;
	}

	/* We dispatch from request queue wide instead of hw queue */
	blk_queue_flag_set(QUEUE_FLAG_SQ_SCHED, q);

	q->elevator = eq;
	mio_blkcg_activate(q);
	enable_mdd();
	return 0;

put_eq:
	kobject_put(&eq->kobj);
	return ret;
}

/*
 * Try to merge @bio into an existing request. If @bio has been merged into
 * an existing request, store the pointer to that request into *@rq.
 */
static int mdd_request_merge(struct request_queue *q, struct request **rq,
			    struct bio *bio)
{
	struct mdd_data *dd = q->elevator->elevator_data;
	const u8 ioprio_class = IOPRIO_PRIO_CLASS(bio->bi_ioprio);
	const enum mdd_prio prio = ioprio_class_to_prio[ioprio_class];
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];
	sector_t sector = bio_end_sector(bio);
	struct request *__rq;

	if (!dd->front_merges)
		return ELEVATOR_NO_MERGE;

	__rq = elv_rb_find(&per_prio->sort_list[bio_data_dir(bio)], sector);
	if (__rq) {
		BUG_ON(sector != blk_rq_pos(__rq));

		if (elv_bio_merge_ok(__rq, bio)) {
			*rq = __rq;
			if (blk_discard_mergable(__rq))
				return ELEVATOR_DISCARD_MERGE;
			return ELEVATOR_FRONT_MERGE;
		}
	}

	return ELEVATOR_NO_MERGE;
}

/*
 * Attempt to merge a bio into an existing request. This function is called
 * before @bio is associated with a request.
 */
static bool mdd_bio_merge(struct request_queue *q, struct bio *bio,
		unsigned int nr_segs)
{
	struct mdd_data *dd = q->elevator->elevator_data;
	struct request *free = NULL;
	bool ret;

	spin_lock(&dd->lock);
	ret = blk_mq_sched_try_merge(q, bio, nr_segs, &free);
	spin_unlock(&dd->lock);

	if (free) {
		blk_mq_free_request(free);
	}
	return ret;
}

/*
 * add rq to rbtree and fifo
 */
static void mdd_insert_request(struct blk_mq_hw_ctx *hctx, struct request *rq,
			      bool at_head)
{
	struct request_queue *q = hctx->queue;
	struct mdd_data *dd = q->elevator->elevator_data;
	const enum mdd_data_dir data_dir = rq_data_dir(rq);
	u16 ioprio = req_get_ioprio(rq);
	u8 ioprio_class = IOPRIO_PRIO_CLASS(ioprio);
	struct mdd_per_prio *per_prio;
	enum mdd_prio prio;
	struct mio_rq_info *mrq;
	LIST_HEAD(free);


	ioprio = get_current_ioprio();
	if ((ioprio != IOPRIO_CLASS_RT) && ( IOPRIO_CLASS_RT == task_nice_ioclass(current) ))
	{
		ioprio = IOPRIO_CLASS_RT;
	}
	ioprio_class = IOPRIO_PRIO_CLASS(ioprio);

	prio = ioprio_class_to_prio[ioprio_class];
	mrq = get_mio_rq_info(dd, rq);

	if ((prio > DD_TB_PRIO) && request_boost(dd,current, rq, mrq))
	{
		ioprio_class =  IOPRIO_CLASS_TB;
		if (likely(mrq))
			mrq->io_class = ioprio_class;
		dd->boosted++;
		mio_log("%d:%d (%d %d) in be when boost\n", mdd_queued(dd, DD_TB_PRIO), mdd_queued(dd, DD_BE_PRIO), mdd_owned_by_driver(dd, DD_TB_PRIO),mdd_owned_by_driver(dd, DD_BE_PRIO));
	}
	rq->elv.priv[1] = (void *)(uintptr_t)ioprio_class;
	lockdep_assert_held(&dd->lock);
	/*
	 * This may be a requeue of a write request that has locked its
	 * target zone. If it is the case, this releases the zone lock.
	 */
	blk_req_zone_write_unlock(rq);

	prio = ioprio_class_to_prio[ioprio_class];
	per_prio = &dd->per_prio[prio];
	if (!rq->elv.priv[0]) {
		per_prio->stats.inserted++;
		rq->elv.priv[0] = (void *)1;
		atomic_inc(&dd->in_queue_rqs);
		if (likely(mrq)) {
			mrq->pid = current->tgid;
			mrq->tid = current->pid;
			mrq->task_i=0;
			mrq->io_class = ioprio_class;
			mrq->m_prio = prio;
		}
	}

	if (blk_mq_sched_try_insert_merge(q, rq, &free)) {
		blk_mq_free_requests(&free);
		return;
	}
	trace_block_rq_insert(rq);

	if (at_head || blk_rq_is_passthrough(rq)) {
		if (at_head)
			list_add(&rq->queuelist, &per_prio->dispatch);
		else
			list_add_tail(&rq->queuelist, &per_prio->dispatch);

		rq->fifo_time = jiffies;
	} else {
		__mdd_add_rq_rb(per_prio, rq);

		if (rq_mergeable(rq)) {
			elv_rqhash_add(q, rq);
			if (!q->last_merge)
				q->last_merge = rq;
		}

		/*
		 * set expire time and add to fifo list
		 */
		rq->fifo_time = jiffies + dd->fifo_expire[data_dir];
		list_add_tail(&rq->queuelist, &per_prio->fifo_list[data_dir]);
	}
	trace_rq_sched_log(rq, at_head, prio, data_dir, current->tgid);
	//trace_rq_sched_insert(rq);
}

/*
 * Called from blk_mq_sched_insert_request() or blk_mq_sched_insert_requests().
 */
static void mdd_insert_requests(struct blk_mq_hw_ctx *hctx,
			       struct list_head *list, bool at_head)
{
	struct request_queue *q = hctx->queue;
	struct mdd_data *dd = q->elevator->elevator_data;

	spin_lock(&dd->lock);
	while (!list_empty(list)) {
		struct request *rq;

		rq = list_first_entry(list, struct request, queuelist);
		list_del_init(&rq->queuelist);
		mdd_insert_request(hctx, rq, at_head);
	}
	spin_unlock(&dd->lock);
}

/* Callback from inside blk_mq_rq_ctx_init(). */
static void mdd_prepare_request(struct request *rq)
{
	//trace_rq_sched_prepare(rq);
	rq->elv.priv[0] = NULL;
	rq->elv.priv[1] = NULL;
}

static bool mdd_has_write_work(struct blk_mq_hw_ctx *hctx)
{
	struct mdd_data *dd = hctx->queue->elevator->elevator_data;
	enum mdd_prio p;

	for (p = 0; p <= DD_PRIO_MAX; p++)
		if (!list_empty_careful(&dd->per_prio[p].fifo_list[DD_WRITE]))
			return true;

	return false;
}

/*
 * Callback from inside blk_mq_free_request().
 *
 * For zoned block devices, write unlock the target zone of
 * completed write requests. Do this while holding the zone lock
 * spinlock so that the zone is never unlocked while __mdd_fifo_request()
 * or __mdd_next_request() are executing. This function is called for
 * all requests, whether or not these requests complete successfully.
 *
 * For a zoned block device, __mdd_dispatch_request() may have stopped
 * dispatching requests if all the queued requests are write requests directed
 * at zones that are already locked due to on-going write requests. To ensure
 * write request dispatch progress in this case, mark the queue as needing a
 * restart to ensure that the queue is run again after completion of the
 * request and zones being unlocked.
 */
static void mdd_finish_request(struct request *rq)
{
	struct request_queue *q = rq->q;
	struct mdd_data *dd = q->elevator->elevator_data;
	const u8 ioprio_class = mdd_rq_ioclass(dd, rq);
	const enum mdd_prio prio = ioprio_class_to_prio[ioprio_class];
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];
	//struct mio_request_info *trk = get_mio_request_info(rq);
	struct mio_rq_info *mrq;

	trace_rq_sched_finish(rq);

	/*
	 * The block layer core may call mdd_finish_request() without having
	 * called mdd_insert_requests(). Skip requests that bypassed I/O
	 * scheduling. See also blk_mq_request_bypass_insert().
	 */
	if (!rq->elv.priv[0])
		return;

	atomic_inc(&per_prio->stats.completed);
	atomic_dec(&dd->in_queue_rqs);
	mrq = get_mio_rq_info(dd, rq);
	if (likely(mrq))
	{
		mrq->pid = 0;
		mrq->tid = 0;
		mrq->start_time = 0;
	}

	if (blk_queue_is_zoned(q)) {
		unsigned long flags;

		spin_lock_irqsave(&dd->zone_lock, flags);
		blk_req_zone_write_unlock(rq);
		spin_unlock_irqrestore(&dd->zone_lock, flags);

		if (mdd_has_write_work(rq->mq_hctx))
			blk_mq_sched_mark_restart_hctx(rq->mq_hctx);
	}
}

static bool mdd_has_work_for_prio(struct mdd_per_prio *per_prio)
{
	return !list_empty_careful(&per_prio->dispatch) ||
		!list_empty_careful(&per_prio->fifo_list[DD_READ]) ||
		!list_empty_careful(&per_prio->fifo_list[DD_WRITE]);
}

static bool mdd_has_work(struct blk_mq_hw_ctx *hctx)
{
	struct mdd_data *dd = hctx->queue->elevator->elevator_data;
	enum mdd_prio prio;

	for (prio = 0; prio <= DD_PRIO_MAX; prio++)
		if (mdd_has_work_for_prio(&dd->per_prio[prio]))
			return true;

	return false;
}

/*
 * sysfs parts below
 */
#define SHOW_INT(__FUNC, __VAR)						\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct mdd_data *dd = e->elevator_data;			\
	(void)dd;								\
											\
	return sysfs_emit(page, "%d\n", __VAR);				\
}
#define SHOW_JIFFIES(__FUNC, __VAR) SHOW_INT(__FUNC, jiffies_to_msecs(__VAR))
SHOW_JIFFIES(__mdd_read_expire_show, dd->fifo_expire[DD_READ]);
SHOW_JIFFIES(__mdd_write_expire_show, dd->fifo_expire[DD_WRITE]);
SHOW_JIFFIES(__mdd_prio_aging_expire_show, dd->prio_aging_expire);
SHOW_INT(__mdd_writes_starved_show, dd->writes_starved);
SHOW_INT(__mdd_front_merges_show, dd->front_merges);
SHOW_INT(__mdd_async_depth_show, dd->async_depth);
SHOW_INT(__mdd_fifo_batch_show, dd->fifo_batch);
SHOW_INT(__mdd_dispatch_fifo_show, dd->dispatch_fifo);
SHOW_INT(__mdd_latency_show, dd->latency);
SHOW_INT(__mdd_boost_show, enable_boost);
#undef SHOW_INT
#undef SHOW_JIFFIES

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct mdd_data *dd = e->elevator_data;			\
	int __data, __ret;						\
	(void)dd;								\
											\
	__ret = kstrtoint(page, 0, &__data);				\
	if (__ret < 0)							\
		return __ret;						\
	if (__data < (MIN))						\
		__data = (MIN);						\
	else if (__data > (MAX))					\
		__data = (MAX);						\
	*(__PTR) = __CONV(__data);					\
	return count;							\
}
#define STORE_INT(__FUNC, __PTR, MIN, MAX)				\
	STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, )
#define STORE_JIFFIES(__FUNC, __PTR, MIN, MAX)				\
	STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, msecs_to_jiffies)
STORE_JIFFIES(__mdd_read_expire_store, &dd->fifo_expire[DD_READ], 0, INT_MAX);
STORE_JIFFIES(__mdd_write_expire_store, &dd->fifo_expire[DD_WRITE], 0, INT_MAX);
STORE_JIFFIES(__mdd_prio_aging_expire_store, &dd->prio_aging_expire, 0, INT_MAX);
STORE_INT(__mdd_writes_starved_store, &dd->writes_starved, INT_MIN, INT_MAX);
STORE_INT(__mdd_front_merges_store, &dd->front_merges, 0, 1);
STORE_INT(__mdd_async_depth_store, &dd->async_depth, 1, INT_MAX);
STORE_INT(__mdd_fifo_batch_store, &dd->fifo_batch, 0, INT_MAX);
STORE_INT(__mdd_dispatch_fifo_store, &dd->dispatch_fifo, 0, INT_MAX);
STORE_INT(__mdd_latency_store, &dd->latency, 0, INT_MAX);
STORE_INT(__mdd_boost_store, &enable_boost, 0, INT_MAX);
#undef STORE_FUNCTION
#undef STORE_INT
#undef STORE_JIFFIES

#define DD_ATTR(name) \
	__ATTR(name, 0644, __mdd_##name##_show, __mdd_##name##_store)


static struct elv_fs_entry __mdd_attrs[] = {
	DD_ATTR(read_expire),
	DD_ATTR(write_expire),
	DD_ATTR(writes_starved),
	DD_ATTR(front_merges),
	DD_ATTR(async_depth),
	DD_ATTR(fifo_batch),
	DD_ATTR(dispatch_fifo),
	DD_ATTR(latency),
	DD_ATTR(boost),
	DD_ATTR(prio_aging_expire),
	__ATTR_NULL
};

#ifdef CONFIG_BLK_DEBUG_FS

#define DEADLINE_DEBUGFS_DDIR_ATTRS(prio, data_dir, name)		\
static void *__mdd_##name##_fifo_start(struct seq_file *m,		\
					  loff_t *pos)			\
	__acquires(&dd->lock)						\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];		\
									\
	spin_lock(&dd->lock);						\
	return seq_list_start(&per_prio->fifo_list[data_dir], *pos);	\
}									\
									\
static void *__mdd_##name##_fifo_next(struct seq_file *m, void *v,	\
					 loff_t *pos)			\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];		\
									\
	return seq_list_next(v, &per_prio->fifo_list[data_dir], pos);	\
}									\
									\
static void __mdd_##name##_fifo_stop(struct seq_file *m, void *v)	\
	__releases(&dd->lock)						\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
									\
	spin_unlock(&dd->lock);						\
}									\
									\
static const struct seq_operations __mdd_##name##_fifo_seq_ops = {	\
	.start	= __mdd_##name##_fifo_start,				\
	.next	= __mdd_##name##_fifo_next,				\
	.stop	= __mdd_##name##_fifo_stop,				\
	.show	= blk_mq_debugfs_rq_show,				\
};									\
									\
static int __mdd_##name##_next_rq_show(void *data,			\
					  struct seq_file *m)		\
{									\
	struct request_queue *q = data;					\
	struct mdd_data *dd = q->elevator->elevator_data;		\
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];		\
	struct request *rq = per_prio->next_rq[data_dir];		\
									\
	if (rq)								\
		__blk_mq_debugfs_rq_show(m, rq);		\
	return 0;							\
}

DEADLINE_DEBUGFS_DDIR_ATTRS(DD_RT_PRIO, DD_READ, read0);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_RT_PRIO, DD_WRITE, write0);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_TB_PRIO, DD_READ, read1);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_TB_PRIO, DD_WRITE, write1);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_BE_PRIO, DD_READ, read2);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_BE_PRIO, DD_WRITE, write2);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_IDLE_PRIO, DD_READ, read3);
DEADLINE_DEBUGFS_DDIR_ATTRS(DD_IDLE_PRIO, DD_WRITE, write3);
#undef DEADLINE_DEBUGFS_DDIR_ATTRS

static int __mdd_batching_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;

	seq_printf(m, "%u\n", dd->batching);
	return 0;
}

static int __mdd_starved_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;

	seq_printf(m, "%u\n", dd->starved);
	return 0;
}

static int mdd_async_depth_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;

	seq_printf(m, "%u\n", dd->async_depth);
	return 0;
}

static int mdd_queued_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;
	u32 rt, be, idle, tb;

	spin_lock(&dd->lock);
	rt = mdd_queued(dd, DD_RT_PRIO);
	tb = mdd_queued(dd, DD_TB_PRIO);
	be = mdd_queued(dd, DD_BE_PRIO);
	idle = mdd_queued(dd, DD_IDLE_PRIO);
	spin_unlock(&dd->lock);

	seq_printf(m, "%u %u %u %u\n", rt, tb, be, idle);

	return 0;
}

static int io_latency_show(struct mio_latency __percpu *lats, int io_type, struct seq_file *m)
{
    u64 sum[BYTE_TABLE_SIZE][NSEC_TABLE_SIZE] = { 0, };
    int cpu;
    int byte_idx, ns_idx;

	seq_printf(m, "\nLAT(ms) %s:\t", op_labels[io_type]);
	for (ns_idx = 0; ns_idx < NSEC_TABLE_SIZE; ns_idx++)
		seq_printf(m, " %lld", nsec_table[ns_idx]/NSEC_PER_MSEC);
	seq_printf(m, "\n");
    for_each_possible_cpu(cpu) {
        struct mio_latency *s = per_cpu_ptr(lats, cpu);

        for (byte_idx = 0; byte_idx < BYTE_TABLE_SIZE; byte_idx++)
            for (ns_idx = 0; ns_idx < NSEC_TABLE_SIZE; ns_idx++)
                sum[byte_idx][ns_idx] +=
                    s->io_latency_cnt[io_type][byte_idx][ns_idx];
    }

    for (byte_idx = 0; byte_idx < BYTE_TABLE_SIZE; byte_idx++) {
        seq_printf(m,  "%u KB:\t",
                byte_table[byte_idx] / 1024);
        for (ns_idx = 0; ns_idx < NSEC_TABLE_SIZE; ns_idx++)
            seq_printf(m, " %llu",
                    sum[byte_idx][ns_idx]);
        seq_printf(m,  "\n");
    }

    return 0;
}


static int mdd_latency_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;

	struct mio_latency *s;
	int io_type, cpu;

	for (io_type=0;io_type < DD_DIR_COUNT; io_type++)
		io_latency_show(dd->io_latency, io_type, m);

    for_each_possible_cpu(cpu) {
        s = per_cpu_ptr(dd->io_latency, cpu);
		memset(s, 0, sizeof(struct mio_latency));
	}

	return 0;
}


/* Number of requests owned by the block driver for a given priority. */
static u32 mdd_owned_by_driver(struct mdd_data *dd, enum mdd_prio prio)
{
	const struct io_stats_per_prio *stats = &dd->per_prio[prio].stats;

	lockdep_assert_held(&dd->lock);

	return stats->dispatched + stats->merged -
		atomic_read(&stats->completed);
}

static int mdd_owned_by_driver_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct mdd_data *dd = q->elevator->elevator_data;
	u32 rt, be, idle, tb;

	spin_lock(&dd->lock);
	rt = mdd_owned_by_driver(dd, DD_RT_PRIO);
	tb = mdd_owned_by_driver(dd, DD_TB_PRIO);
	be = mdd_owned_by_driver(dd, DD_BE_PRIO);
	idle = mdd_owned_by_driver(dd, DD_IDLE_PRIO);
	spin_unlock(&dd->lock);

	seq_printf(m, "%u %u %u %u\n", rt, tb, be, idle);

	return 0;
}

#define DEADLINE_DISPATCH_ATTR(prio)					\
static void *__mdd_dispatch##prio##_start(struct seq_file *m,	\
					     loff_t *pos)		\
	__acquires(&dd->lock)						\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];		\
									\
	spin_lock(&dd->lock);						\
	return seq_list_start(&per_prio->dispatch, *pos);		\
}									\
									\
static void *__mdd_dispatch##prio##_next(struct seq_file *m,		\
					    void *v, loff_t *pos)	\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
	struct mdd_per_prio *per_prio = &dd->per_prio[prio];		\
									\
	return seq_list_next(v, &per_prio->dispatch, pos);		\
}									\
									\
static void __mdd_dispatch##prio##_stop(struct seq_file *m, void *v)	\
	__releases(&dd->lock)						\
{									\
	struct request_queue *q = m->private;				\
	struct mdd_data *dd = q->elevator->elevator_data;		\
									\
	spin_unlock(&dd->lock);						\
}									\
									\
static const struct seq_operations __mdd_dispatch##prio##_seq_ops = { \
	.start	= __mdd_dispatch##prio##_start,			\
	.next	= __mdd_dispatch##prio##_next,			\
	.stop	= __mdd_dispatch##prio##_stop,			\
	.show	= blk_mq_debugfs_rq_show,				\
}

DEADLINE_DISPATCH_ATTR(0);
DEADLINE_DISPATCH_ATTR(1);
DEADLINE_DISPATCH_ATTR(2);
DEADLINE_DISPATCH_ATTR(3);
#undef DEADLINE_DISPATCH_ATTR

#define DEADLINE_QUEUE_DDIR_ATTRS(name)					\
	{#name "_fifo_list", 0400,					\
			.seq_ops = &__mdd_##name##_fifo_seq_ops}
#define DEADLINE_NEXT_RQ_ATTR(name)					\
	{#name "_next_rq", 0400, __mdd_##name##_next_rq_show}
static const struct blk_mq_debugfs_attr __mdd_queue_debugfs_attrs[] = {
	DEADLINE_QUEUE_DDIR_ATTRS(read0),
	DEADLINE_QUEUE_DDIR_ATTRS(write0),
	DEADLINE_QUEUE_DDIR_ATTRS(read1),
	DEADLINE_QUEUE_DDIR_ATTRS(write1),
	DEADLINE_QUEUE_DDIR_ATTRS(read2),
	DEADLINE_QUEUE_DDIR_ATTRS(write2),
	DEADLINE_QUEUE_DDIR_ATTRS(read3),
	DEADLINE_QUEUE_DDIR_ATTRS(write3),
	DEADLINE_NEXT_RQ_ATTR(read0),
	DEADLINE_NEXT_RQ_ATTR(write0),
	DEADLINE_NEXT_RQ_ATTR(read1),
	DEADLINE_NEXT_RQ_ATTR(write1),
	DEADLINE_NEXT_RQ_ATTR(read2),
	DEADLINE_NEXT_RQ_ATTR(write2),
	DEADLINE_NEXT_RQ_ATTR(read3),
	DEADLINE_NEXT_RQ_ATTR(write3),
	{"batching", 0400, __mdd_batching_show},
	{"starved", 0400, __mdd_starved_show},
	{"async_depth", 0400, mdd_async_depth_show},
	{"dispatch0", 0400, .seq_ops = &__mdd_dispatch0_seq_ops},
	{"dispatch1", 0400, .seq_ops = &__mdd_dispatch1_seq_ops},
	{"dispatch2", 0400, .seq_ops = &__mdd_dispatch2_seq_ops},
	{"dispatch3", 0400, .seq_ops = &__mdd_dispatch3_seq_ops},
	{"owned_by_driver", 0400, mdd_owned_by_driver_show},
	{"queued", 0400, mdd_queued_show},
	{"latency", 0400, mdd_latency_show},
	{},
};
#undef DEADLINE_QUEUE_DDIR_ATTRS
#endif

static struct elevator_type mdd_iosched = {
	.ops = {
		.depth_updated		= mdd_depth_updated,
		.limit_depth		= mdd_limit_depth,
		.insert_requests	= mdd_insert_requests,
		.dispatch_request	= mdd_dispatch_request,
		.prepare_request	= mdd_prepare_request,
		.finish_request		= mdd_finish_request,
		.completed_request =  mdd_completed_request,
		.next_request		= elv_rb_latter_request,
		.former_request		= elv_rb_former_request,
		.bio_merge		= mdd_bio_merge,
		.request_merge		= mdd_request_merge,
		.requests_merged	= mdd_merged_requests,
		.request_merged		= mdd_request_merged,
		.has_work		= mdd_has_work,
		.init_sched		= mdd_init_sched,
		.exit_sched		= mdd_exit_sched,
		.init_hctx		= mdd_init_hctx,
	},

#ifdef CONFIG_BLK_DEBUG_FS
	.queue_debugfs_attrs = __mdd_queue_debugfs_attrs,
#endif
	.elevator_attrs = __mdd_attrs,
	.elevator_name = "mdd",
	.elevator_alias = "mdd",
	.elevator_features = ELEVATOR_F_ZBD_SEQ_WRITE,
	.elevator_owner = THIS_MODULE,
};
MODULE_ALIAS("mdd-iosched");

static int __init __mdd_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct mio_request_info) > (sizeof(void*)*ARRAY_SIZE(((struct request *)0)->elv.priv)));

	ret = elv_register(&mdd_iosched);
	if (ret)
		return ret;

	ret = mio_blkcg_init();
	if (ret) {
		elv_unregister(&mdd_iosched);
	}
	iosched_ctl_init();
	return ret;
}

/*
static void __exit __mdd_exit(void)
{

	iosched_ctl_deinit();
	mio_blkcg_exit();
	elv_unregister(&mdd_iosched);
}
*/

module_init(__mdd_init);
//module_exit(__mdd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mdd IO scheduler");
