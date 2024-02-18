/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#ifndef _MOTO_LOCKING_MAIN_H_
#define _MOTO_LOCKING_MAIN_H_


#define MAGIC_NUM       (0xdead0000)
#define MAGIC_MASK      (0xffff0000)
#define MAGIC_SHIFT     (16)
#define OWNER_BIT       (1 << 0)
#define THREAD_INFO_BIT (1 << 1)
#define TYPE_BIT        (1 << 2)

#define UX_FLAG_BIT       (1<<0)
#define SS_FLAG_BIT       (1<<1)
#define GRP_SHIFT         (2)
#define GRP_FLAG_MASK     (7 << GRP_SHIFT)
#define U_GRP_OTHER       (1 << GRP_SHIFT)
#define U_GRP_BACKGROUND  (2 << GRP_SHIFT)
#define U_GRP_FRONDGROUD  (3 << GRP_SHIFT)
#define U_GRP_TOP_APP     (4 << GRP_SHIFT)

#define LOCK_TYPE_SHIFT (30)
#define INVALID_TYPE    (0)
#define LOCK_ART        (1)
#define LOCK_JUC        (2)

struct futex_uinfo {
	u32 cmd;
	u32 owner_tid;
	u32 type;
	u64 inform_user;
};

#define LK_MUTEX_ENABLE (1 << 0)
#define LK_RWSEM_ENABLE (1 << 1)
#define LK_FUTEX_ENABLE (1 << 2)

extern unsigned int g_opt_enable;
extern int __read_mostly moto_sched_enabled;

extern atomic64_t futex_inherit_set_times;
extern atomic64_t futex_inherit_unset_times;
extern atomic64_t futex_inherit_useless_times;
extern atomic64_t futex_low_count;
extern atomic64_t futex_high_count;

static inline bool locking_opt_enable(unsigned int enable)
{
	return moto_sched_enabled && (g_opt_enable & enable);
}

#ifdef CONFIG_MOTO_FUTEX_INHERIT
void register_futex_vendor_hooks(void);
void unregister_futex_vendor_hooks(void);
#endif

#ifdef CONFIG_MOTO_RWSEM_INHERIT
void register_rwsem_vendor_hooks(void);
void unregister_rwsem_vendor_hooks(void);
#endif

#ifdef CONFIG_MOTO_MUTEX_INHERIT
void register_mutex_vendor_hooks(void);
void unregister_mutex_vendor_hooks(void);
#endif

#endif /* _MOTO_LOCKING_MAIN_H_ */
