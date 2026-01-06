/*
 * Copyright (C) 2025 Motorola Mobility LLC
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

#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/sched/cputime.h>
#include <linux/minmax.h>
#include <asm/cache.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <asm/barrier.h>
#include <uapi/linux/sched/types.h>
#include <kernel/sched/sched.h>

#include <trace/hooks/sched.h>

#include "msched_oemdata.h"
#include "msched_common.h"
#include "msched_uclamp.h"

struct kmem_cache *msched_task_struct_cachep;
EXPORT_SYMBOL(msched_task_struct_cachep);

extern void reset_rwsem_owner_list(struct moto_task_struct *mts);

static void init_moto_task_struct(void *ptr)
{
	struct moto_task_struct *mts = ptr;

	memset(mts, 0, sizeof(struct moto_task_struct));

	INIT_LIST_HEAD(&mts->owner_node);
}

static inline void free_moto_task_struct(struct moto_task_struct *mts)
{
	if (!mts)
		return;

	kmem_cache_free(msched_task_struct_cachep, mts);
}

// don't dup ux_type for UX_TYPE_SERVICEMANAGER as init was labeled.
#define UX_TYPE_TO_DUP (UX_TYPE_AUDIOSERVICE|UX_TYPE_NATIVESERVICE|UX_TYPE_CAMERASERVICE|UX_TYPE_ANIMATOR)
void android_vh_dup_task_struct_handler(void *unused,
		struct task_struct *tsk, struct task_struct *orig)
{
	struct moto_task_struct *mts = NULL;
    // Base feature: inherit task ux_type during fork for some native services.
	int ux_type = task_get_ux_type(orig) & UX_TYPE_TO_DUP;

	if (!tsk || !orig)
		return;

	if (!IS_ERR_OR_NULL((void *)tsk->android_oem_data1[0]))
		return;

	mts = kmem_cache_alloc(msched_task_struct_cachep, GFP_ATOMIC);
	if (IS_ERR_OR_NULL(mts))
		return;

	init_moto_task_struct(mts);
	mts->task = tsk;
	smp_mb();

	WRITE_ONCE(tsk->android_oem_data1[0], (u64) mts);

    if (ux_type != 0) {
        task_add_ux_type(tsk, ux_type);
		cond_trace_printk(unlikely(is_debuggable(DEBUG_BASE)),
			"copy ux_type %d from %d to %d\n", ux_type, orig->pid, tsk->pid);
	}
        msched_uclamp_vh_dup_task_struct(unused, tsk, orig);
}

void android_vh_free_task_handler(void *unused, struct task_struct *tsk)
{
	struct moto_task_struct *mts = NULL;

	if (!tsk)
		return;

	mts = (struct moto_task_struct *) READ_ONCE(tsk->android_oem_data1[0]);
	if (IS_ERR_OR_NULL(mts))
		return;

	WRITE_ONCE(tsk->android_oem_data1[0], 0);
	barrier();

	reset_rwsem_owner_list(mts);
	mts->task = NULL;

	free_moto_task_struct(mts);
}

static int register_oemdata_hooks(void)
{
	int ret = 0;

    register_trace_android_vh_dup_task_struct(android_vh_dup_task_struct_handler, NULL);
    register_trace_android_vh_free_task(android_vh_free_task_handler, NULL);

	return ret;
}

static void unregister_oemdata_hooks(void)
{
    unregister_trace_android_vh_dup_task_struct(android_vh_dup_task_struct_handler, NULL);
    unregister_trace_android_vh_free_task(android_vh_free_task_handler, NULL);
}

static int init_moto_struct_node_in_vendor_task_data(void *data)
{
	struct task_struct *p, *g;
	u32 iter_cpu;

	read_lock(&tasklist_lock);

	for_each_process_thread(g, p) {
		struct moto_task_struct *mts = NULL;

		mts = (struct moto_task_struct *) READ_ONCE(p->android_oem_data1[0]);
		if (IS_ERR_OR_NULL(mts)) {
			mts = kmem_cache_alloc(msched_task_struct_cachep, GFP_ATOMIC);

			if (!IS_ERR_OR_NULL(mts)) {
				mts->task = p;
				smp_mb();

				WRITE_ONCE(p->android_oem_data1[0], (u64) mts);
			}
		}
	}
	for_each_possible_cpu(iter_cpu) {
		struct moto_task_struct *mts = NULL;
		struct rq *rq = cpu_rq(iter_cpu);
		if (rq) {
			p = rq->idle;
			mts = (struct moto_task_struct *) READ_ONCE(p->android_oem_data1[0]);
			if (IS_ERR_OR_NULL(mts)) {
				mts = kmem_cache_alloc(msched_task_struct_cachep, GFP_ATOMIC);
				if (!IS_ERR_OR_NULL(mts)) {
					mts->task = p;
					smp_mb();

					WRITE_ONCE(p->android_oem_data1[0], (u64) mts);
				}
			}
		}
	}
	read_unlock(&tasklist_lock);
	return 0;
}

int msched_oemdata_init(void)
{
	msched_task_struct_cachep = kmem_cache_create("moto_task_struct",
			sizeof(struct moto_task_struct), 0,
			SLAB_PANIC|SLAB_ACCOUNT, init_moto_task_struct);

	if (!msched_task_struct_cachep)
		return -ENOMEM;

	register_oemdata_hooks();

    stop_machine(init_moto_struct_node_in_vendor_task_data, NULL, cpumask_of(raw_smp_processor_id()));

	return 0;
}

void __maybe_unused msched_oemdata_deinit(void)
{
	unregister_oemdata_hooks();
	kmem_cache_destroy(msched_task_struct_cachep);
}

