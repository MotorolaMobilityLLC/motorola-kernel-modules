/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_MOTO_SLOWPATH_H_
#define __MM_MOTO_SLOWPATH_H_

#include <trace/hooks/mm.h>

struct slowpath_control;

int slowpath_sysfs(struct kobject *parent);
void remove_slowpath_sysfs(void);

void vh_slowpath_begin(void *, u64 *ts);
void vh_slowpath_end(void *,  gfp_t *, unsigned int,  unsigned long, u64, unsigned long,  unsigned long, int );

#endif
