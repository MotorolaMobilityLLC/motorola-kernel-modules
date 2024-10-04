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
#include <linux/task_work.h>
#include <linux/mutex.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/miscdevice.h>   /* for misc_register, and SYNTH_MINOR */
#include "moto_binder.h"

#define MOTO_BINDER_DRV_NAME "moto_binder"

typedef struct {
	int type;
	int caller_uid;
	int caller_pid;
	int target_uid;
	int target_pid;
	int arg1;
	int arg2;
	struct work_struct work;
} event_ctx_t;

typedef struct {
	struct mutex lock;
	atomic_t work_cnt;
	struct miscdevice miscdev;
	struct workqueue_struct *work_wq;
} moto_binder_context_t;

static bool moto_binder_initialized = false;
static moto_binder_context_t *g_context = NULL;
static void moto_binder_report_event(struct work_struct *ws);

void moto_binder_send_uevent(int type, int caller_uid, int caller_pid, int target_uid, int target_pid, int arg1, int arg2)
{
	event_ctx_t *ctx;

	if (atomic_read(&g_context->work_cnt) > 100) {
		pr_err("alloc work cnt more than 100, skip reporting\n");
		return;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		pr_err("no memory\n");
		return;
	}

	ctx->type = type;
	ctx->caller_uid = caller_uid;
	ctx->caller_pid = caller_pid;
	ctx->target_uid = target_uid;
	ctx->target_pid = target_pid;
	ctx->arg1 = arg1;
	ctx->arg2 = arg2;

	INIT_WORK(&ctx->work, moto_binder_report_event);
	atomic_inc(&g_context->work_cnt);

	if (g_context)
		queue_work(g_context->work_wq, &ctx->work);
	else
		pr_err("Driver not initialized\n");

}

static void moto_binder_report_event(struct work_struct *ws)
{
	event_ctx_t *ctx = container_of(ws, event_ctx_t, work);
	struct kobj_uevent_env *env;

	env = kzalloc(sizeof(*env), GFP_KERNEL);
	if (!env)
		return;
	if (add_uevent_var(env, "type=%d", ctx->type))
		goto _err;
	if (add_uevent_var(env, "caller_uid=%d", ctx->caller_uid))
		goto _err;
	if (add_uevent_var(env, "caller_pid=%d", ctx->caller_pid))
		goto _err;
	if (add_uevent_var(env, "target_uid=%d", ctx->target_uid))
		goto _err;
	if (add_uevent_var(env, "target_pid=%d", ctx->target_pid))
		goto _err;
	if (add_uevent_var(env, "arg1=%d", ctx->arg1))
		goto _err;
	if (add_uevent_var(env, "arg2=%d", ctx->arg2))
		goto _err;

	kobject_uevent_env(&g_context->miscdev.this_device->kobj, KOBJ_CHANGE, env->envp);
	kfree(env);
	kfree(ctx);
	atomic_dec(&g_context->work_cnt);

	return;
_err:
	kfree(env);
	kfree(ctx);
	atomic_dec(&g_context->work_cnt);
	pr_info("send uevent failed");
}

static moto_binder_context_t moto_binder_context = {
	.miscdev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = MOTO_BINDER_DRV_NAME,
	}
};

static void moto_binder_uevent_exit(void)
{
	if (g_context) {
		destroy_workqueue(g_context->work_wq);
		kobject_uevent(&g_context->miscdev.this_device->kobj, KOBJ_REMOVE);
		misc_deregister(&g_context->miscdev);
		g_context = NULL;
	}
}

static int moto_binder_uevent_init(void)
{
	int ret = 0;

	ret = misc_register(&moto_binder_context.miscdev);
	if (ret) {
		pr_info("misc_register error:%d\n", ret);
		return ret;
	}

	ret = kobject_uevent(
			&moto_binder_context.miscdev.this_device->kobj, KOBJ_ADD);

	if (ret) {
		misc_deregister(&moto_binder_context.miscdev);
		pr_info("uevent creat fail:%d\n", ret);
		goto __err_add;
	}

	/* Create workqueue for the write work */
	moto_binder_context.work_wq = alloc_workqueue("moto_binder", WQ_UNBOUND, 1);
	if (!moto_binder_context.work_wq) {
		pr_err("Could not create workqueue\n");
		goto __err_wq;
	}

	atomic_set(&moto_binder_context.work_cnt, 0);
	g_context = &moto_binder_context;

	return ret;

__err_wq:
	kobject_uevent(&moto_binder_context.miscdev.this_device->kobj, KOBJ_REMOVE);

__err_add:
	misc_deregister(&moto_binder_context.miscdev);
	return -1;
}

int moto_binder_init(void)
{
	int ret = 0;

	if (moto_binder_initialized) return 0;

	ret = moto_binder_status_init();
	if (ret != 0)
		goto __status_err;

	ret = moto_binder_uevent_init();
	if (ret != 0)
		goto __uevent_err;

	ret = moto_netfilter_init();
	if (ret != 0)
		goto __netfilter_err;

	pr_info("moto binder init succeed!\n");
	moto_binder_initialized = true;

	return ret;

__netfilter_err:
	moto_binder_uevent_exit();
__uevent_err:
	moto_binder_status_exit();
__status_err:
	pr_info("moto binder init failed!\n");

	return ret;

}

void moto_binder_exit(void)
{
	if (!moto_binder_initialized) return;

	moto_binder_status_exit();
	moto_binder_uevent_exit();
	moto_netfilter_deinit();

	moto_binder_initialized = false;
	pr_info("moto binder exit succeed!\n");
}
