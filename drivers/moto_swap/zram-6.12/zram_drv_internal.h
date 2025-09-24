#ifndef _ZRAM_DRV_INTERNAL_H_
#define _ZRAM_DRV_INTERNAL_H_

#define ZRAM_BIT(nr)		(1UL << (nr))

#define zram_slot_lock(zram, index) (spin_lock(&zram->table[index].lock))

#define zram_slot_unlock(zram, index) (spin_unlock(&zram->table[index].lock))

#define init_done(zram)  (zram->disksize)

#define dev_to_zram(dev) ((struct zram *)dev_to_disk(dev)->private_data)

#define zram_get_handle(zram, index) (zram->table[index].handle)

#define zram_set_handle(zram, index, handle_val) (zram->table[index].handle = handle_val)

#define zram_test_flag(zram, index,  flag) (zram->table[index].flags & ZRAM_BIT(flag))

#define zram_set_flag(zram, index, flag) (zram->table[index].flags |= ZRAM_BIT(flag))

#define zram_clear_flag(zram, index, flag) (zram->table[index].flags &= ~ZRAM_BIT(flag))

#define zram_set_element(zram, index, element) (zram->table[index].element = element)

#define zram_get_obj_size(zram, index) (zram->table[index].flags & (ZRAM_BIT(ZRAM_FLAG_SHIFT) - 1))

#define zram_set_obj_size(zram, index, size) do {\
	unsigned long flags = zram->table[index].flags >> ZRAM_FLAG_SHIFT; \
	zram->table[index].flags = (flags << ZRAM_FLAG_SHIFT) | size; \
} while(0)

#endif
