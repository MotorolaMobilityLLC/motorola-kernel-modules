#ifndef _ZRAM_DRV_INTERNAL_H_
#define _ZRAM_DRV_INTERNAL_H_
#ifdef BIT
#undef BIT
#define BIT(nr)		(1lu << (nr))
#endif

#define zram_slot_lock(zram, index) (bit_spin_lock(ZRAM_LOCK, &zram->table[index].flags))

#define zram_slot_unlock(zram, index) (bit_spin_unlock(ZRAM_LOCK, &zram->table[index].flags))

#define init_done(zram)  (zram->disksize)

#define dev_to_zram(dev) ((struct zram *)dev_to_disk(dev)->private_data)

#define zram_get_handle(zram, index) (zram->table[index].handle)

#define zram_set_handle(zram, index, handle_val) (zram->table[index].handle = handle_val)

#define zram_test_flag(zram, index,  flag) (zram->table[index].flags & BIT(flag))

#define zram_set_flag(zram, index, flag) (zram->table[index].flags |= BIT(flag))

#define zram_clear_flag(zram, index, flag) (zram->table[index].flags &= ~BIT(flag))

//#define zram_set_element(zram, index, element) (zram->table[index].element = element)

#define zram_get_obj_size(zram, index) (zram->table[index].flags & (BIT(ZRAM_FLAG_SHIFT) - 1))

#define zram_set_obj_size(zram, index, size) do {\
	unsigned long flags = zram->table[index].flags >> ZRAM_FLAG_SHIFT; \
	zram->table[index].flags = (flags << ZRAM_FLAG_SHIFT) | size; \
} while(0)

void zram_free_page(struct zram *zram, size_t index);

static inline void zram_set_element(struct zram *zram, u32 index,
			unsigned long element)
{
	zram->table[index].element = element;
}
static inline int zram_slot_trylock(struct zram *zram, u32 index)
{
	return bit_spin_trylock(ZRAM_LOCK, &zram->table[index].flags);
}



static inline unsigned long zram_get_element(struct zram *zram, u32 index)
{
	return zram->table[index].element;
}


static inline bool zram_allocated(struct zram *zram, u32 index)
{
	return zram_get_obj_size(zram, index) ||
			zram_test_flag(zram, index, ZRAM_SAME) ||
			zram_test_flag(zram, index, ZRAM_WB);
}

static inline void zram_set_priority(struct zram *zram, u32 index, u32 prio)
{
	prio &= ZRAM_COMP_PRIORITY_MASK;
	/*
	 * Clear previous priority value first, in case if we recompress
	 * further an already recompressed page
	 */
	zram->table[index].flags &= ~(ZRAM_COMP_PRIORITY_MASK <<
				      ZRAM_COMP_PRIORITY_BIT1);
	zram->table[index].flags |= (prio << ZRAM_COMP_PRIORITY_BIT1);
}

static inline u32 zram_get_priority(struct zram *zram, u32 index)
{
	u32 prio = zram->table[index].flags >> ZRAM_COMP_PRIORITY_BIT1;

	return prio & ZRAM_COMP_PRIORITY_MASK;
}

static inline void update_used_max(struct zram *zram,
					const unsigned long pages)
{
	unsigned long cur_max = atomic_long_read(&zram->stats.max_used_pages);

	do {
		if (cur_max >= pages)
			return;
	} while (!atomic_long_try_cmpxchg(&zram->stats.max_used_pages,
					  &cur_max, pages));
}
#endif
