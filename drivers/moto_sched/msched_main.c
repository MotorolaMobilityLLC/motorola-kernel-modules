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
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#include <linux/sched/cputime.h>
#endif
#include <kernel/sched/sched.h>

#include "msched_sysfs.h"
#include "msched_common.h"
#include "mdpf/mdpf_sysfs.h"

#ifdef CONFIG_MOTO_LOCKING_2
#include "msched_oemdata.h"
#else
#define MOTO_OEM_DATA_SIZE_TEST(wstruct, kstruct)		\
	BUILD_BUG_ON(sizeof(wstruct) > (sizeof(u64) *		\
		ARRAY_SIZE(((kstruct *)0)->android_oem_data1)))
#endif

extern int locking_opt_init(void);

static int __init moto_sched_init(void)
{
	int ret = 0;

#ifdef CONFIG_MOTO_LOCKING_2
	ret = msched_oemdata_init();
	if (ret != 0)
		return ret;
#else
	/* compile time checks for oem data size */
	MOTO_OEM_DATA_SIZE_TEST(struct moto_task_struct, struct task_struct);
#endif

	ret = moto_sched_proc_init();
	if (ret != 0)
		return ret;
	ret = mdpf_proc_init();
	if (ret != 0) {
		pr_err("mdpf_proc_init failed!\n");
		moto_sched_proc_deinit();
		return ret;
	}

	register_vendor_comm_hooks();
	locking_opt_init();

	pr_info("moto_sched_init succeed!\n");
	return 0;
}

module_init(moto_sched_init);
MODULE_DESCRIPTION("Motorola CPU sched optimizations driver");
MODULE_LICENSE("GPL v2");
