// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Moto. All rights reserved.
 */

#define pr_fmt(fmt) "moto_locking_boost:" fmt

#include <linux/sched.h>
#include <linux/module.h>

#include "locking_main.h"

unsigned int g_opt_enable;
unsigned int g_opt_debug;

int locking_opt_init(void)
{
	int ret = 0;

	// g_opt_debug |= LK_DEBUG_FTRACE;

#ifdef CONFIG_MOTO_MUTEX_INHERIT
	g_opt_enable |= LK_MUTEX_ENABLE;
	register_mutex_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_RWSEM_INHERIT
	g_opt_enable |= LK_RWSEM_ENABLE;
	register_rwsem_vendor_hooks();
#endif

#ifdef CONFIG_MOTO_FUTEX_INHERIT
	g_opt_enable |= LK_FUTEX_ENABLE;
	register_futex_vendor_hooks();
#endif

	return ret;
}

void locking_opt_exit(void)
{
	g_opt_enable = 0;

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
