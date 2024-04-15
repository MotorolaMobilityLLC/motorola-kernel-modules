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
#include <linux/oom.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>
#include <linux/printk.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>

#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/swap.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/vmstat.h>
#include <linux/mm_inline.h>
#include <linux/rmap.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/memcontrol.h>
#include <linux/psi.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <linux/vmstat.h>

#include "../../../mm/internal.h"
#include "mm_common.h"

#if defined(LRU_SHRINKER_SUPPORTED)

#include <trace/hooks/signal.h>

#define LRU_SHRINKER_PAGES_HIGH (0x1000)  //16Mbytes

#define PG_nolockdelay (__NR_PAGEFLAGS + 2)
#define SetPageNoLockDelay(page) set_bit(PG_nolockdelay, &(page)->flags)
#define TestPageNoLockDelay(page) test_bit(PG_nolockdelay, &(page)->flags)
#define TestClearPageNoLockDelay(page) test_and_clear_bit(PG_nolockdelay, &(page)->flags)
#define ClearPageNoLockDelay(page) clear_bit(PG_nolockdelay, &(page)->flags)

#define PG_skiped_lock (__NR_PAGEFLAGS + 3)
#define SetPageSkipedLock(page) set_bit(PG_skiped_lock, &(page)->flags)
#define ClearPageSkipedLock(page) clear_bit(PG_skiped_lock, &(page)->flags)
#define PageSkipedLock(page) test_bit(PG_skiped_lock, &(page)->flags)
#define TestClearPageSkipedLock(page) test_and_clear_bit(PG_skiped_lock, &(page)->flags)

extern unsigned long reclaim_pages(struct list_head *page_list);

bool lru_shrinker_initialized = false;

static struct task_struct *lru_shrinker_tsk = NULL;
static int lru_shrinker_pid = -1;

static atomic_t lru_shrinker_runnable = ATOMIC_INIT(0);
unsigned long lru_shrinker_pages = 0;
unsigned long lru_shrinker_pages_max = 0;
unsigned long lru_shrinker_handled_pages = 0;

wait_queue_head_t wq_lru_shrinker;
spinlock_t lru_inactive_lock;
LIST_HEAD(lru_inactive);

static inline bool process_is_lru_shrinker(struct task_struct *tsk)
{
	return (lru_shrinker_pid == tsk->pid);
}

static void add_to_lru_inactive(struct page *page)
{
	list_move(&page->lru, &lru_inactive);

	/* account how much pages in lru_inactive */
	lru_shrinker_pages += thp_nr_pages(page);
	if (lru_shrinker_pages > lru_shrinker_pages_max)
		lru_shrinker_pages_max = lru_shrinker_pages;
}

static void handle_failed_page_trylock_hook(void *data, struct list_head *page_list)
{
	struct page *page, *next;
	bool lru_shrinker_is_full = false;
	bool pages_should_be_reclaim = false;
	LIST_HEAD(tmp_lru_inactive);

	if (unlikely(!lru_shrinker_initialized))
		return;

	if (list_empty(page_list))
		return;

	if (unlikely(lru_shrinker_pages > LRU_SHRINKER_PAGES_HIGH))
		lru_shrinker_is_full = true;

	list_for_each_entry_safe(page, next, page_list, lru) {
		ClearPageNoLockDelay(page);
		if (unlikely(TestClearPageSkipedLock(page))) {
			/* trylock failed and been skiped  */
			ClearPageActive(page);
			if (!lru_shrinker_is_full)
				list_move(&page->lru, &tmp_lru_inactive);
		}
	}

	if (unlikely(!list_empty(&tmp_lru_inactive))) {
		spin_lock_irq(&lru_inactive_lock);
		list_for_each_entry_safe(page, next, &tmp_lru_inactive, lru) {
			if (likely(!lru_shrinker_is_full)) {
				pages_should_be_reclaim = true;
				add_to_lru_inactive(page);
			}
		}
		spin_unlock_irq(&lru_inactive_lock);
	}

	if (lru_shrinker_is_full || !pages_should_be_reclaim)
		return;

	if (atomic_read(&lru_shrinker_runnable) == 1)
		return;

	atomic_set(&lru_shrinker_runnable, 1);
	wake_up_interruptible(&wq_lru_shrinker);
}

void set_lru_shrinkerd_cpus(void)
{
	struct cpumask mask;
	struct cpumask *cpumask = &mask;
	pg_data_t *pgdat = NODE_DATA(0);
	unsigned int cpu = 0, cpufreq_max_tmp = 0;
	struct cpufreq_policy *policy_max;
	static bool set_cpus_success = false;

	if (unlikely(!lru_shrinker_initialized))
		return;

	if (likely(set_cpus_success))
		return;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (policy == NULL)
			continue;

		if (policy->cpuinfo.max_freq >= cpufreq_max_tmp) {
			cpufreq_max_tmp = policy->cpuinfo.max_freq;
			policy_max = policy;
		}
	}

	cpumask_copy(cpumask, cpumask_of_node(pgdat->node_id));
	cpumask_andnot(cpumask, cpumask, policy_max->related_cpus);

	if (!cpumask_empty(cpumask)) {
		set_cpus_allowed_ptr(lru_shrinker_tsk, cpumask);
		set_cpus_success = true;
	}
}

static int lru_shrinker(void *p)
{
	//pg_data_t *pgdat;
	LIST_HEAD(tmp_lru_inactive);
	struct page *page, *next;
	struct list_head;

	/*
	 * Tell the memory management that we're a "memory allocator",
	 * and that if we need more memory we should get access to it
	 * regardless (see "__alloc_pages()"). "kswapd" should
	 * never get caught in the normal page freeing logic.
	 *
	 * (Kswapd normally doesn't need memory anyway, but sometimes
	 * you need a small amount of memory in order to be able to
	 * page out something else, and this flag essentially protects
	 * us from recursively trying to free more memory as we're
	 * trying to free the first piece of memory in the first place).
	 */
	//pgdat = (pg_data_t *)p;

	current->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD;
	set_freezable();

	while (!kthread_should_stop()) {
		wait_event_freezable(wq_lru_shrinker,
			(atomic_read(&lru_shrinker_runnable) == 1));

		// exit on kthread_stop
		if (unlikely(kthread_should_stop())) {
			atomic_set(&lru_shrinker_runnable, 0);
			break;
		}

		set_lru_shrinkerd_cpus();
retry_reclaim:

		spin_lock_irq(&lru_inactive_lock);
		if (list_empty(&lru_inactive)) {
			spin_unlock_irq(&lru_inactive_lock);
			atomic_set(&lru_shrinker_runnable, 0);
			continue;
		}
		list_for_each_entry_safe(page, next, &lru_inactive, lru) {
			list_move(&page->lru, &tmp_lru_inactive);
			lru_shrinker_pages -= thp_nr_pages(page);
			lru_shrinker_handled_pages += thp_nr_pages(page);
		}
		spin_unlock_irq(&lru_inactive_lock);

		reclaim_pages(&tmp_lru_inactive);
		goto retry_reclaim;
	}
	current->flags &= ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD);

	return 0;
}

static void page_trylock_set_hook(void *data, struct page *page)
{
	if (unlikely(!lru_shrinker_initialized))
		return;

	ClearPageSkipedLock(page);

	if (unlikely(process_is_lru_shrinker(current))) {
		ClearPageNoLockDelay(page);
		return;
	}

	SetPageNoLockDelay(page);
}

static void page_trylock_clear_hook(void *data, struct page *page)
{
	ClearPageNoLockDelay(page);
	ClearPageSkipedLock(page);
}

static void page_trylock_get_result_hook(void *data, struct page *page, bool *trylock_fail)
{
	ClearPageNoLockDelay(page);

	if (unlikely(!lru_shrinker_initialized) ||
			unlikely(process_is_lru_shrinker(current))) {
		*trylock_fail = false;
		return;
	}

	if (PageSkipedLock(page))
		*trylock_fail = true; /*page trylock failed and been skipped*/
}

static void do_page_trylock_hook(void *data, struct page *page, struct rw_semaphore *sem,
		bool *got_lock, bool *success)
{
	*success = false;
	if (unlikely(!lru_shrinker_initialized))
		return;

	if (TestClearPageNoLockDelay(page)) {
		*success = true;

		if (sem == NULL)
			return;

		if (down_read_trylock(sem)) {   /* return 1 successful */
			*got_lock = true;

		} else {
			SetPageSkipedLock(page);  /* trylock failed and skipped */
			*got_lock = false;
		}
	}
}

ssize_t proc_lru_shrinker_status_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "lru_shrinker=%s pages=%lu handled_pages=%lu pages_max=%lu\n",
		lru_shrinker_initialized ? "enabled" : "disabled",
		lru_shrinker_pages, lru_shrinker_handled_pages, lru_shrinker_pages_max);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

int register_lru_shrinker_hooks(void)
{
	int rc;

	REGISTER_HOOK(handle_failed_page_trylock);
	REGISTER_HOOK(page_trylock_set);
	REGISTER_HOOK(page_trylock_clear);
	REGISTER_HOOK(page_trylock_get_result);
	REGISTER_HOOK(do_page_trylock);
	return 0;

	UNREGISTER_HOOK(do_page_trylock);
ERROR_OUT(do_page_trylock):
	UNREGISTER_HOOK(page_trylock_get_result);
ERROR_OUT(page_trylock_get_result):
	UNREGISTER_HOOK(page_trylock_clear);
ERROR_OUT(page_trylock_clear):
	UNREGISTER_HOOK(page_trylock_set);
ERROR_OUT(page_trylock_set):
	UNREGISTER_HOOK(handle_failed_page_trylock);
ERROR_OUT(handle_failed_page_trylock):

	return rc;
}

void unregister_lru_shrinker_hooks(void)
{
	UNREGISTER_HOOK(do_page_trylock);
	UNREGISTER_HOOK(page_trylock_get_result);
	UNREGISTER_HOOK(page_trylock_clear);
	UNREGISTER_HOOK(page_trylock_set);
	UNREGISTER_HOOK(handle_failed_page_trylock);
}

int mm_lru_shrinker_init(void)
{
	int ret;
	pg_data_t *pgdat = NODE_DATA(0);

	if (lru_shrinker_initialized) return 0;

	ret = register_lru_shrinker_hooks();
	if (ret != 0)
		return ret;

	init_waitqueue_head(&wq_lru_shrinker);
	spin_lock_init(&lru_inactive_lock);

	lru_shrinker_tsk = kthread_run(lru_shrinker, pgdat, "klrushrinkerd");
	if (IS_ERR_OR_NULL(lru_shrinker_tsk)) {
		pr_err("Failed to start lru_shrinker on node 0\n");
		ret = PTR_ERR(lru_shrinker_tsk);
		lru_shrinker_tsk = NULL;
		unregister_lru_shrinker_hooks();
		return ret;
	}

	lru_shrinker_pid = lru_shrinker_tsk->pid;

	lru_shrinker_initialized = true;

	pr_info("mm_lru_shrinker_init succeed!\n");
	return 0;
}

void mm_lru_shrinker_exit(void)
{
	if (!lru_shrinker_initialized) return;

	unregister_lru_shrinker_hooks();

	if (lru_shrinker_tsk) {
		atomic_set(&lru_shrinker_runnable, 1);
		kthread_stop(lru_shrinker_tsk);
		lru_shrinker_tsk = NULL;
		lru_shrinker_pid = -1;
	}

	lru_shrinker_initialized = false;

	pr_info("mm_lru_shrinker_exit succeed!\n");
}

#endif // defined(LRU_SHRINKER_SUPPORTED)