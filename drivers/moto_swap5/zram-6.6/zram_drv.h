/*
 * Compressed RAM block device
 *
 * Copyright (C) 2008, 2009, 2010  Nitin Gupta
 *               2012, 2013 Minchan Kim
 *
 * This code is released using a dual license strategy: BSD/GPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 *
 */

#ifndef _ZRAM_DRV_H_
#define _ZRAM_DRV_H_

#include <linux/rwsem.h>
#include <linux/zsmalloc.h>
#include <linux/crypto.h>

#include "zcomp.h"
#include "zram_ext.h"
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/printk.h>

#define ZRAM_CTX(fmt, ...)                                              \
    do {                                                                \
        pr_debug("[zram-ctx] " fmt                                      \
                " | in_atomic=%d in_irq=%d in_softirq=%d "              \
                " irqs_disabled=%d preempt=%x\n",                       \
                ##__VA_ARGS__,                                          \
                (int)in_atomic(), (int)in_irq(), (int)in_softirq(),                    \
                (int)irqs_disabled(), preempt_count());                      \
    } while (0)

#define ZRAM_MIGHT_SLEEP()                                              \
    do {                                                                \
        ZRAM_CTX("might_sleep");                                        \
        might_sleep();                                                  \
    } while (0)

#define ZRAM_MIGHT_SLEEP_IF(cond)                                       \
    do {                                                                \
        if (cond) {                                                     \
            ZRAM_CTX("might_sleep_if(cond)");                           \
            might_sleep();                                              \
        }                                                               \
    } while (0)

#define ZRAM_WARN_IF_ATOMIC_WAIT()                                      \
    WARN_ON_ONCE(in_atomic() || in_interrupt() || irqs_disabled())

#define SECTORS_PER_PAGE_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define SECTORS_PER_PAGE	(1 << SECTORS_PER_PAGE_SHIFT)
#define ZRAM_LOGICAL_BLOCK_SHIFT 12
#define ZRAM_LOGICAL_BLOCK_SIZE	(1 << ZRAM_LOGICAL_BLOCK_SHIFT)
#define ZRAM_SECTOR_PER_LOGICAL_BLOCK	\
	(1 << (ZRAM_LOGICAL_BLOCK_SHIFT - SECTOR_SHIFT))

#define print_hex_dump_fmt(src, size) \
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1, src, size, 1)

/*
 * ZRAM is mainly used for memory efficiency so we want to keep memory
 * footprint small and thus squeeze size and zram pageflags into a flags
 * member. The lower ZRAM_FLAG_SHIFT bits is for object size (excluding
 * header), which cannot be larger than PAGE_SIZE (requiring PAGE_SHIFT
 * bits), the higher bits are for zram_pageflags.
 *
 * We use BUILD_BUG_ON() to make sure that zram pageflags don't overflow.
 */
#define ZRAM_FLAG_SHIFT (PAGE_SHIFT + 1)

/* Only 2 bits are allowed for comp priority index */
#define ZRAM_COMP_PRIORITY_MASK	0x3

/* Flags for zram pages (table[page_no].flags) */
enum zram_pageflags {
	/* zram slot is locked */
	ZRAM_LOCK = ZRAM_FLAG_SHIFT,
	ZRAM_SAME,	/* Page consists the same element */
	ZRAM_WB,	/* page is stored on backing_device */
	ZRAM_UNDER_WB,	/* page is under writeback */
	ZRAM_HUGE,	/* Incompressible page */
	ZRAM_IDLE,	/* not accessed page since last idle marking */
	ZRAM_INCOMPRESSIBLE, /* none of the algorithms could compress it */

	ZRAM_COMP_PRIORITY_BIT1, /* First bit of comp priority index */
	ZRAM_COMP_PRIORITY_BIT2, /* Second bit of comp priority index */
#ifdef CONFIG_HYBRIDSWAP_CORE
	ZRAM_BATCHING_OUT,
	ZRAM_FROM_HYBRIDSWAP,
	ZRAM_MCGID_CLEAR,
	ZRAM_IN_BD, /* zram stored in back device */
#endif
	__NR_ZRAM_PAGEFLAGS,
};

/*-- Data structures */

/* Allocated for each disk page */
struct zram_table_entry {
	union {
		unsigned long handle;
		unsigned long element;
	};
	unsigned long flags;
#ifdef CONFIG_HYBRIDSWAP_ZRAM_MEMORY_TRACKING
	ktime_t ac_time;
#endif
};

#ifdef CONFIG_ZRAM_EXT
enum zram_error_types {
	ERR_TYPE1,
	ERR_TYPE2,

	NR_ERR_TYPES,
};
#endif

struct zram_stats {
	atomic64_t compr_data_size;	/* compressed size of pages stored */
	atomic64_t failed_reads;	/* can happen when memory is too low */
	atomic64_t failed_writes;	/* can happen when memory is too low */
	atomic64_t notify_free;	/* no. of swap slot free notifications */
	atomic64_t same_pages;		/* no. of same element filled pages */
	atomic64_t huge_pages;		/* no. of huge pages */
	atomic64_t huge_pages_since;	/* no. of huge pages since zram set up */
	atomic64_t pages_stored;	/* no. of pages currently stored */
	atomic_long_t max_used_pages;	/* no. of maximum pages stored */
	atomic64_t writestall;		/* no. of write slow paths */
	atomic64_t miss_free;		/* no. of missed free */
//#ifdef	CONFIG_VENDOR_ZRAM_WRITEBACK
	atomic64_t bd_count;		/* no. of pages in backing device */
	atomic64_t bd_reads;		/* no. of reads from backing device */
	atomic64_t bd_writes;		/* no. of writes from backing device */
//#endif
#ifdef CONFIG_ZRAM_EXT
	atomic64_t bd_objcnt;
	atomic64_t bd_size;
	atomic64_t bd_max_count;
	atomic64_t bd_max_size;
	atomic64_t bd_objreads;
	atomic64_t bd_objwrites;
	atomic64_t error_count[NR_ERR_TYPES];
#endif
};

#ifdef CONFIG_ZRAM_MULTI_COMP
#define ZRAM_PRIMARY_COMP	0U
#define ZRAM_SECONDARY_COMP	1U
#define ZRAM_MAX_COMPS	4U
#else
#define ZRAM_PRIMARY_COMP	0U
#define ZRAM_SECONDARY_COMP	0U
#define ZRAM_MAX_COMPS	1U
#endif

struct zram {
	struct zram_table_entry *table;
	struct zs_pool *mem_pool;
	struct zcomp *comps[ZRAM_MAX_COMPS];
	struct gendisk *disk;
	/* Prevent concurrent execution of device init */
	struct rw_semaphore init_lock;
	/*
	 * the number of pages zram can consume for storing compressed data
	 */
	unsigned long limit_pages;

	struct zram_stats stats;
	/*
	 * This is the limit on amount of *uncompressed* worth of data
	 * we can store in a disk.
	 */
	u64 disksize;	/* bytes */
	const char *comp_algs[ZRAM_MAX_COMPS];
	s8 num_active_comps;
	/*
	 * zram is claimed so open request will be failed
	 */
	bool claim; /* Protected by disk->open_mutex */
	struct file *backing_dev;
#ifdef CONFIG_VENDOR_ZRAM_WRITEBACK
	spinlock_t wb_limit_lock;
	bool wb_limit_enable;
	u64 bd_wb_limit;
	struct block_device *bdev;
	unsigned long *bitmap;
	unsigned long nr_pages;
#endif
#ifdef CONFIG_HYBRIDSWAP_ZRAM_MEMORY_TRACKING
	struct dentry *debugfs_dir;
#endif
#if (defined CONFIG_HYBRIDSWAP_CORE)
	struct block_device *bdev;
	unsigned int old_block_size;
	unsigned long nr_pages;
	unsigned long increase_nr_pages;
#endif
#ifdef CONFIG_HYBRIDSWAP_CORE
	struct hyb_info *infos;
#endif
#ifdef CONFIG_ZRAM_EXT
	struct task_struct *prefetchd;
	struct list_head prefetch_list;
	struct mutex falloc_lock;
	struct zram_wb_work **read_work;
	spinlock_t bitmap_lock;
	spinlock_t prefetch_lock;
	spinlock_t read_work_lock;
	spinlock_t refcount_lock;
	wait_queue_head_t prefetch_wait;
	unsigned long *chunk_bitmap;
	unsigned long *falloc_bitmap;
	unsigned long *read_bitmap;
	u16 *refcount_table;
	atomic_t nr_prefetch;
#endif
};
#endif
