// SPDX-License-Identifier: GPL-2.0


#include <linux/blkdev.h>
#include <linux/blk-mq.h>

#include "mio.h"


static struct blkcg_policy mio_blkcg_policy;


#define CPD_TO_MIO_BLKCG(_cpd) \
	container_of_safe((_cpd), struct mio_blkcg, cpd)
#define BLKCG_TO_MIO_BLKCG(_blkcg) \
	CPD_TO_MIO_BLKCG(blkcg_to_cpd((_blkcg), &mio_blkcg_policy))

#define PD_TO_MIO_BLKG(_pd) \
	container_of_safe((_pd), struct mio_blkg, pd)
#define BLKG_TO_MIO_BLKG(_blkg) \
	PD_TO_MIO_BLKG(blkg_to_pd((_blkg), &mio_blkcg_policy))

#define CSS_TO_MIO_BLKCG(css) BLKCG_TO_MIO_BLKCG(css_to_blkcg(css))



static struct blkcg_policy_data *mio_blkcg_cpd_alloc(gfp_t gfp)
{
	struct mio_blkcg *mio_blkcg;

	mio_blkcg = kzalloc(sizeof(struct mio_blkcg), gfp);
	if (ZERO_OR_NULL_PTR(mio_blkcg))
		return NULL;

	return &mio_blkcg->cpd;
}

static void mio_blkcg_cpd_init(struct blkcg_policy_data *cpd)
{
	struct mio_blkcg *mio_blkcg = CPD_TO_MIO_BLKCG(cpd);

	if (IS_ERR_OR_NULL(mio_blkcg))
		return;

	mio_blkcg->weight = 100;
}

static void mio_blkcg_cpd_free(struct blkcg_policy_data *cpd)
{
	struct mio_blkcg *mio_blkcg = CPD_TO_MIO_BLKCG(cpd);

	if (IS_ERR_OR_NULL(mio_blkcg))
		return;

	kfree(mio_blkcg);
}

static void mio_blkcg_set_shallow_depth(struct mio_blkcg *mio_blkcg,
		struct mio_blkg *mio_blkg, struct blk_mq_tags *tags)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
	unsigned int depth = tags->bitmap_tags->sb.depth;
	unsigned int map_nr = tags->bitmap_tags->sb.map_nr;
#else
	unsigned int depth = tags->bitmap_tags.sb.depth;
	unsigned int map_nr = tags->bitmap_tags.sb.map_nr;
#endif


	mio_blkg->shallow_depth =
		max_t(unsigned int, 1, (depth * max(25, mio_blkcg->weight) / 100U) / map_nr);
	mio_blkg->async_shallow_depth = mio_blkg->shallow_depth * 3 /4;
	//printk("%s shallow_depth %d  map_nr %d depth %d\n", __func__, mio_blkg->shallow_depth, map_nr, depth);
}

static struct blkg_policy_data *mio_blkcg_pd_alloc(gfp_t gfp,
		struct request_queue *q, struct blkcg *blkcg)
{
	struct mio_blkg *mio_blkg;

	mio_blkg = kzalloc_node(sizeof(struct mio_blkg), gfp, q->node);
	if (ZERO_OR_NULL_PTR(mio_blkg))
		return NULL;

	return &mio_blkg->pd;
}

static void mio_blkcg_pd_init(struct blkg_policy_data *pd)
{
	struct mio_blkg *mio_blkg;
	struct mio_blkcg *mio_blkcg;
	struct blk_mq_hw_ctx *hctx;
    unsigned long i;

	mio_blkg = PD_TO_MIO_BLKG(pd);
	if (IS_ERR_OR_NULL(mio_blkg))
		return;

	mio_blkcg = BLKCG_TO_MIO_BLKCG(pd->blkg->blkcg);
	if (IS_ERR_OR_NULL(mio_blkcg))
		return;

	queue_for_each_hw_ctx(pd->blkg->q, hctx, i)
        	mio_blkcg_set_shallow_depth(mio_blkcg,mio_blkg, hctx->sched_tags);

}

static void mio_blkcg_pd_free(struct blkg_policy_data *pd)
{
	struct mio_blkg *mio_blkg = PD_TO_MIO_BLKG(pd);

	if (IS_ERR_OR_NULL(mio_blkg))
		return;

	kfree(mio_blkg);
}

 #if LINUX_VERSION_CODE > KERNEL_VERSION(6, 0, 0)
#if 0
struct kthread {
	unsigned long flags;
	unsigned int cpu;
	int result;
	int (*threadfn)(void *);
	void *data;
	struct completion parked;
	struct completion exited;
#ifdef CONFIG_BLK_CGROUP
	struct cgroup_subsys_state *blkcg_css;
#endif
	/* To store the full name if task comm is truncated. */
	char *full_name;
};


static inline struct kthread *to_kthread(struct task_struct *k)
{
    WARN_ON(!(k->flags & PF_KTHREAD));
    return k->worker_private;


#endif

static struct cgroup_subsys_state *blkcg_css(void)
{
#if 0
	struct kthread *kt;

    if (current->flags & PF_KTHREAD) {
        kt = to_kthread(current);
        if (kt && kt->blkcg_css)
            return kt->blkcg_css;
    }
#endif
	return task_css(current, io_cgrp_id);
}
#endif

u32 mio_blkcg_shallow_depth(struct request_queue *q, bool is_sync, int *weight)
{
	struct blkcg_gq *blkg;
	struct mio_blkg *mio_blkg;
	struct mio_blkcg *mio_blkcg;

	rcu_read_lock();
	blkg = blkg_lookup(css_to_blkcg(blkcg_css()), q);
	mio_blkg = BLKG_TO_MIO_BLKG(blkg);
	mio_blkcg = BLKCG_TO_MIO_BLKCG(blkg->blkcg);
	rcu_read_unlock();

	if (IS_ERR_OR_NULL(mio_blkg))
		return 0;

	if (IS_ERR_OR_NULL(mio_blkcg))
		return 0;
	*weight = mio_blkcg->weight;

	return is_sync?mio_blkg->shallow_depth:mio_blkg->async_shallow_depth;
}

void mio_blkcg_depth_updated(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	struct cgroup_subsys_state *pos_css;
	struct blkcg_gq *blkg;
	struct mio_blkg *mio_blkg;
	struct mio_blkcg *mio_blkcg;

	rcu_read_lock();
	blkg_for_each_descendant_pre(blkg, pos_css, q->root_blkg) {
		mio_blkg = BLKG_TO_MIO_BLKG(blkg);
		if (IS_ERR_OR_NULL(mio_blkg))
			continue;

		mio_blkcg = BLKCG_TO_MIO_BLKCG(blkg->blkcg);
		if (IS_ERR_OR_NULL(mio_blkcg))
			continue;
		mio_blkcg_set_shallow_depth(mio_blkcg, mio_blkg, hctx->sched_tags);
	}
	rcu_read_unlock();
}



static int mio_blkcg_show_weight(struct seq_file *sf, void *v)
{
	struct mio_blkcg *mio_blkcg = CSS_TO_MIO_BLKCG(seq_css(sf));

	if (IS_ERR_OR_NULL(mio_blkcg))
		return -EINVAL;

	seq_printf(sf, "%d\n", mio_blkcg->weight);

	return 0;
}

static int mio_blkcg_set_weight(struct cgroup_subsys_state *css,
		struct cftype *cftype, u64 weight)
{
	struct blkcg *blkcg = css_to_blkcg(css);
	struct mio_blkcg *mio_blkcg = CSS_TO_MIO_BLKCG(css);
	struct blkcg_gq *blkg;
	struct mio_blkg *mio_blkg;
    struct blk_mq_hw_ctx *hctx;
    unsigned long i;



	if (IS_ERR_OR_NULL(mio_blkcg))
		return -EINVAL;

	if (weight > 100)
		return -EINVAL;

	// printk("%s weight %lld\n", __func__, weight);
	spin_lock_irq(&blkcg->lock);
	mio_blkcg->weight = weight;
	hlist_for_each_entry(blkg, &blkcg->blkg_list, blkcg_node) {
		mio_blkg = BLKG_TO_MIO_BLKG(blkg);
		if (mio_blkg) {
			queue_for_each_hw_ctx(blkg->q, hctx, i)
				mio_blkcg_set_shallow_depth(mio_blkcg,mio_blkg, hctx->sched_tags);
		}

	}
	spin_unlock_irq(&blkcg->lock);

	return 0;
}

struct cftype mio_blkg_files[] = {
	{
		.name = "mio.weight",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = mio_blkcg_show_weight,
		.write_u64 = mio_blkcg_set_weight,
	},

	{} /* terminate */
};

static struct blkcg_policy mio_blkcg_policy = {
	.legacy_cftypes = mio_blkg_files,

	.cpd_alloc_fn = mio_blkcg_cpd_alloc,
	.cpd_init_fn = mio_blkcg_cpd_init,
	.cpd_free_fn = mio_blkcg_cpd_free,

	.pd_alloc_fn = mio_blkcg_pd_alloc,
	.pd_init_fn = mio_blkcg_pd_init,
	.pd_free_fn = mio_blkcg_pd_free,
};

int mio_blkcg_activate(struct request_queue *q)
{
	return blkcg_activate_policy(q, &mio_blkcg_policy);
}

void mio_blkcg_deactivate(struct request_queue *q)
{
	blkcg_deactivate_policy(q, &mio_blkcg_policy);
}

int mio_blkcg_init(void)
{
	return blkcg_policy_register(&mio_blkcg_policy);
}

void mio_blkcg_exit(void)
{
	blkcg_policy_unregister(&mio_blkcg_policy);
}
