// SPDX-License-Identifier: GPL-2.0-only

#ifndef __MIO_H
#define __MIO_H
#include <linux/blk-cgroup.h>
#include <linux/version.h>
#include <block/blk-cgroup.h>
#include <block/blk.h>
#include <block/blk-mq.h>
#include <block/blk-mq-debugfs.h>
#include <block/blk-mq-tag.h>
#include <block/blk-mq-sched.h>


#include "msched_common.h"




extern int enable_log;
extern int enable_boost;

enum mdd_data_dir {
	DD_READ		= READ,
	DD_WRITE	= WRITE,
};

enum { DD_DIR_COUNT = 2 };

enum mdd_prio {
	DD_RT_PRIO	= 0,
	DD_TB_PRIO	= 1,
	DD_BE_PRIO	= 2,
	DD_IDLE_PRIO	= 3,
	DD_PRIO_MAX	= 3,
};

enum { DD_PRIO_COUNT = 	DD_PRIO_MAX+1 , };

struct mio_blkcg {
	struct blkcg_policy_data cpd; /* must be the first member */

	int weight;
};

struct mio_blkg {
	struct blkg_policy_data pd; /* must be the first member */

	u32 shallow_depth;
	u32 async_shallow_depth;
};

struct mio_rq_info {
	pid_t pid;
	uid_t uid;
	pid_t tid;
	u64 	start_time;
	u8		io_class;
	u8 		m_prio;
	int data_size;
};


/*
 * I/O statistics per I/O priority. It is fine if these counters overflow.
 * What matters is that these counters are at least as wide as
 * log2(max_outstanding_requests).
 */
struct io_stats_per_prio {
	uint32_t inserted;
	uint32_t merged;
	uint32_t dispatched;
	uint32_t dispatching;
	atomic_t completed;
};

/*
 * Deadline scheduler data per I/O priority (enum mdd_prio). Requests are
 * present on both sort_list[] and fifo_list[].
 */
struct mdd_per_prio {
	struct list_head dispatch;
	struct rb_root sort_list[DD_DIR_COUNT];
	struct list_head fifo_list[DD_DIR_COUNT];
	/* Next request in FIFO order. Read, write or both are NULL. */
	struct request *next_rq[DD_DIR_COUNT];
	struct io_stats_per_prio stats;
	enum mdd_prio prio;
	int fifo_only;
};


struct mdd_data {

	struct request_queue *queue;
	/*
	 * run time data
	 */

	struct mdd_per_prio per_prio[DD_PRIO_COUNT];

	/* Data direction of latest dispatched request. */
	enum mdd_data_dir last_dir;
	unsigned int batching;		/* number of sequential requests made */
	unsigned int starved;		/* times reads have starved writes */

	/*
	 * settings that change how the i/o scheduler behaves
	 */
	int fifo_expire[DD_DIR_COUNT];
	int fifo_batch;
	int writes_starved;
	int front_merges;
	u32 async_depth;
	int prio_aging_expire;
	struct mio_latency __percpu *io_latency;

	spinlock_t lock;
	spinlock_t zone_lock;

	int nr_requests;
	int nr_threshold_rqs;
	struct mio_rq_info* rqs;
	atomic_t in_queue_rqs;

	enum mdd_prio last_prio;
	int latency;
	int max_prio_request;
	int min_prio_request;
};

struct bio_oem {
	u8 ioprio_class;
};



#define mio_log(fmt, args...)	do { \
	if (unlikely(enable_log)) { \
		char buf[80];	\
		snprintf(buf, 80, fmt, ##args); \
		trace_printk("%s",  buf); \
	} \
} while (0)

static inline  struct bio_oem* get_bio_oem(struct bio *bio)
{
	return (struct bio_oem*)&bio->android_oem_data1;
}
static inline bool task_in_top_app_group(struct task_struct *p)
{
	struct moto_task_struct *oem_data;
	oem_data  = get_moto_task_struct(p);
	// mio_log(" top %d:%d\n", oem_data->cgr_type, get_task_cgroup_id(p));
	return ( oem_data->cgr_type == CGROUP_TOP_APP );
}

static inline bool task_in_tf_app_group(struct task_struct *p)
{
	int cgrp_id = get_task_cgroup_id(p);
	return (cgrp_id == CGROUP_TOP_APP ) || ( cgrp_id == CGROUP_FOREGROUND) ;
}

static inline bool is_launch(void)
{
	return (moto_sched_scene & UX_SCENE_LAUNCH);
}

void iosched_ctl_init(void);
void iosched_ctl_deinit(void);
u32 mio_blkcg_shallow_depth(struct request_queue *q, bool is_sync , int *weight);

int mio_blkcg_init(void);
void mio_blkcg_exit(void);
int mio_blkcg_activate(struct request_queue *q);
void mio_blkcg_deactivate(struct request_queue *q);
void mio_blkcg_depth_updated(struct blk_mq_hw_ctx *hctx);

bool request_boost(struct mdd_data *dd, struct task_struct *tsk, bool sync, int data_dir);
void request_finish(struct request *rq, u64 now, struct mio_rq_info *rqi);
bool is_enabled_boost(void);
void enable_mdd(void);
void disable_mdd(void);
#endif /* __MIO_H */
