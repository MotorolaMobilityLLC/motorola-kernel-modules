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

#define pr_fmt(fmt) "moto_mm: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <trace/hooks/vmscan.h>
#include <linux/swap.h>


static void tune_inactive_ratio_hook(void *data, unsigned long *inactive_ratio, int file)
{
	if (file)
		*inactive_ratio = min(2UL, *inactive_ratio);
	else
		*inactive_ratio = 1;

	return;
}


#define REGISTER_HOOK(name) do {\
	rc = register_trace_android_vh_##name(name##_hook, NULL);\
	if (rc) {\
		pr_err("register hook %s failed", #name);\
		goto err_out_##name;\
	}\
} while (0)

#define UNREGISTER_HOOK(name) do {\
	unregister_trace_android_vh_##name(name##_hook, NULL);\
} while (0)

#define ERROR_OUT(name) err_out_##name

static int register_all_hooks(void)
{
	int rc;

	/* tune_inactive_ratio_hook */
	REGISTER_HOOK(tune_inactive_ratio);
	return 0;

	UNREGISTER_HOOK(tune_inactive_ratio);
ERROR_OUT(tune_inactive_ratio):
	return rc;
}

static void unregister_all_hook(void)
{
	UNREGISTER_HOOK(tune_inactive_ratio);
}

static int __init moto_mm_init(void)
{
	int ret = 0;

	ret = register_all_hooks();
	if (ret != 0) {
		return ret;
	}

	pr_info("moto_mm_init succeed!\n");
	return 0;
}

static void __exit moto_mm_exit(void)
{
	unregister_all_hook();

	pr_info("moto_mm_exit succeed!\n");
	return;
}

module_init(moto_mm_init);
module_exit(moto_mm_exit);
MODULE_DESCRIPTION("Motorola mm optimizations driver");
MODULE_LICENSE("GPL v2");
