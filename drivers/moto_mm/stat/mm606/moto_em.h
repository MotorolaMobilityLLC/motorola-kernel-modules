/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hardware Energy Model (EM).
 *
 * Copyright (C) 2025 Moto, Inc.
 */

#ifndef __MOTO_EM_H__
#define __MOTO_EM_H__

#if IS_ENABLED(CONFIG_MOTO_EM)

struct moto_em_opp {
  unsigned int freq;
  unsigned int capacity;
  unsigned int power;
  unsigned long cost;
  bool inefficient;
};

struct moto_em_idle_opp {
  unsigned int freq;
  unsigned int energy;
};

struct moto_em_cluster {
  cpumask_t cpus;
  int num_opps;
  union {
    struct moto_em_opp *opps;
    struct moto_em_idle_opp *idle_opps;
  };
};

struct moto_em_profile {
  struct list_head list;
  struct profile_sysfs_helper *sysfs_helper;
  const char *name;
  int num_clusters;
  struct moto_em_cluster *clusters;
  struct moto_em_cluster **cpu_to_cluster; // Maps CPU index to a cluster pointer
};

struct moto_idle_em {
  int num_clusters;
  struct moto_em_cluster *clusters;
  struct moto_em_cluster **cpu_to_cluster;
};

#if IS_ENABLED(CONFIG_VH_SCHED)
extern struct moto_em_profile **vendor_sched_moto_em_profile;
extern struct moto_idle_em *vendor_sched_moto_idle_em;
#endif

#endif /* CONFIG_MOTO_EM */

#endif /* __MOTO_EM_H__ */
