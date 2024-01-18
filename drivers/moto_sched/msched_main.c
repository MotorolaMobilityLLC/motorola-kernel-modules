/*
 * Copyright (C) 2023 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "moto_sched: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <kernel/sched/sched.h>

#include "msched_sysfs.h"
#include "msched_common.h"

#define MOTO_OEM_DATA_SIZE_TEST(wstruct, kstruct)		\
	BUILD_BUG_ON(sizeof(wstruct) > (sizeof(u64) *		\
		ARRAY_SIZE(((kstruct *)0)->android_oem_data1)))


extern int locking_opt_init(void);

static int __init moto_sched_init(void)
{
	int ret = 0;

	/* compile time checks for oem data size */
	MOTO_OEM_DATA_SIZE_TEST(struct moto_task_struct, struct task_struct);

	ret = moto_sched_proc_init();
	if (ret != 0)
		return ret;

	register_vendor_comm_hooks();
	locking_opt_init();

	pr_info("moto_sched_init succeed!\n");
	return 0;
}

module_init(moto_sched_init);
MODULE_DESCRIPTION("Motorola CPU sched optimizations driver");
MODULE_LICENSE("GPL v2");
