// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#define pr_fmt(fmt) "moto_locking_boost:" fmt

#include <linux/sched.h>
#include <linux/module.h>

#include "locking_main.h"

int locking_opt_init(void)
{
	int ret = 0;

#ifdef CONFIG_MOTO_MUTEX_INHERIT
	register_mutex_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_RWSEM_INHERIT
	register_rwsem_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_FUTEX_INHERIT
	register_futex_vendor_hooks();
#endif

	return ret;
}

void locking_opt_exit(void)
{
#ifdef CONFIG_MOTO_FUTEX_INHERIT
	unregister_futex_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_RWSEM_INHERIT
	unregister_rwsem_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_MUTEX_INHERIT
	unregister_mutex_vendor_hooks();
#endif

}
