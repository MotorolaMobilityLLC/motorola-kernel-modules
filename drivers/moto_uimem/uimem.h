#ifndef __UIMEM_H__
#define __UIMEM_H__

#include <linux/types.h>
#include <linux/spinlock.h>

enum pool_migratetype {
    POOL_MIGRATE_UNMOVABLE,
    POOL_MIGRATE_MOVABLE,
    POOL_NR_MIGRATE_TYPES
};

struct pool_struct {
    int wm_low[POOL_NR_MIGRATE_TYPES];
    int wm_high[POOL_NR_MIGRATE_TYPES];
    int buoy[POOL_NR_MIGRATE_TYPES];
    struct list_head page_list[POOL_NR_MIGRATE_TYPES];
    spinlock_t lock;
    gfp_t    gfp_mask; /* pool gfp mask */
    unsigned order;
};

#endif
