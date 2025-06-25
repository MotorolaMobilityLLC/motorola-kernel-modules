/*
 * Copyright (c) 2025 Motorola Inc.
 */

#define get_bucket_id(__val)								      \
		min_t(unsigned int,							      \
		      __val / DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS),	      \
		      UCLAMP_BUCKETS - 1)

#define for_each_clamp_id(clamp_id) \
	for ((clamp_id) = 0; (clamp_id) < UCLAMP_CNT; (clamp_id)++)

#define VENDOR_INHERITANCE_BINDER 0
#define VENDOR_INHERITANCE_RTMUTEX 1

static inline unsigned int uclamp_none(enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN)
		return 0;
	return SCHED_CAPACITY_SCALE;
}

static inline void uclamp_se_set(struct uclamp_se *uc_se,
				 unsigned int value, bool user_defined)
{
	uc_se->value = value;
	uc_se->bucket_id = get_bucket_id(value);
	uc_se->user_defined = user_defined;
}

extern int msched_uclamp_init(void);
extern int msched_uclamp_register_vendor_comm_hooks(void);
extern void msched_uclamp_binder_set_priority_hook(struct task_struct *task);
extern void msched_uclamp_binder_restore_priority_hook(struct task_struct *task);
extern void msched_uclamp_vh_dup_task_struct(void *unused, struct task_struct *task, struct task_struct *orig);

