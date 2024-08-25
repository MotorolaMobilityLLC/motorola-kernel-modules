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

#define pr_fmt(fmt) "moto_binder: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/miscdevice.h>   /* for misc_register, and SYNTH_MINOR */
#include "moto_binder.h"


static struct miscdevice moto_binder_object;

static int moto_binder_uevent_init(void)
{
	int ret = 0;

	moto_binder_object.name = "moto_binder";
	moto_binder_object.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&moto_binder_object);
	if (ret) {
		pr_info("misc_register error:%d\n", ret);
		return ret;
	}

	ret = kobject_uevent(
			&moto_binder_object.this_device->kobj, KOBJ_ADD);

	if (ret) {
		misc_deregister(&moto_binder_object);
		pr_info("uevent creat fail:%d\n", ret);
		return ret;
	}

	return ret;
}

static void moto_binder_uevent_exit(void)
{
	misc_deregister(&moto_binder_object);
}

#define ENV_KV_SIZE 30
void moto_binder_send_uevent(int type, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2)
{
	int ret;
	char name[7][ENV_KV_SIZE];
	char *envp[8];

	snprintf(name[0], ENV_KV_SIZE, "type=%d", type);
	snprintf(name[1], ENV_KV_SIZE, "caller_uid=%d", caller_uid);
	snprintf(name[2], ENV_KV_SIZE, "caller_pid=%d", caller_pid);
	snprintf(name[3], ENV_KV_SIZE, "target_uid=%d", target_uid);
	snprintf(name[4], ENV_KV_SIZE, "target_pid=%d", target_pid);
	snprintf(name[5], ENV_KV_SIZE, "arg1=%d", arg1);
	snprintf(name[6], ENV_KV_SIZE, "arg2=%d", arg2);

	envp[0] = name[0];
	envp[1] = name[1];
	envp[2] = name[2];
	envp[3] = name[3];
	envp[4] = name[4];
	envp[5] = name[5];
	envp[6] = name[6];
	envp[7] = NULL;

	ret = kobject_uevent_env(&moto_binder_object.this_device->kobj, KOBJ_CHANGE, envp);
	if (ret != 0) {
		pr_info("send uevent failed");
		return;
	}
}

static int __init moto_binder_init(void)
{
	int ret = 0;

	ret = moto_binder_proc_init();
	if (ret != 0)
		return ret;

	ret = moto_binder_uevent_init();
	if (ret != 0)
		return ret;

	if (moto_binder_status_enabled > 0) {
		ret = moto_binder_status_init();
		if (ret != 0)
			return ret;
	}

	pr_info("moto_binder_init succeed!\n");
	return ret;

}

static void __exit moto_binder_exit(void)
{
	moto_binder_status_exit();
	moto_binder_uevent_exit();
	moto_binder_proc_deinit();
	pr_info("moto_binder_exit succeed!\n");
}

module_init(moto_binder_init);
module_exit(moto_binder_exit);
MODULE_DESCRIPTION("Motorola binder optimization driver");
MODULE_LICENSE("GPL v2");