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

static inline void zram_set_element(struct zram *zram, u32 index,
						unsigned long element)
{
	zram->table[index].element = element;
}
#define zram_get_obj_size(zram, index) (zram->table[index].flags & (BIT(ZRAM_FLAG_SHIFT) - 1))

#define zram_set_obj_size(zram, index, size) do {\
	unsigned long flags = zram->table[index].flags >> ZRAM_FLAG_SHIFT; \
	zram->table[index].flags = (flags << ZRAM_FLAG_SHIFT) | size; \
} while(0)

enum zram_oem_funcs_cmds {
	ZRAM_APP_LAUNCH_NOTIFY,
	ZRAM_ADD_TO_WRITEBACK_LIST,
	ZRAM_WRITEBACK_LIST,
	ZRAM_FLUSH_WRITEBACK_BUFFER,
	ZRAM_GET_ENTRY_TYPE,
	ZRAM_MARK_ENTRY_NON_LRU,
	ZRAM_PREFETCH_ENTRY,
};
enum zram_entry_type {
	ZRAM_WB_TYPE = 1,
	ZRAM_WB_HUGE_TYPE,
	ZRAM_SAME_TYPE,
	ZRAM_HUGE_TYPE,
};

typedef unsigned long (*zram_oem_func)(int, void *, unsigned long);
extern zram_oem_func zram_oem_fn;
extern unsigned long zram_oem_fn_nocfi(int cmd, void *priv, unsigned long param);

#endif
