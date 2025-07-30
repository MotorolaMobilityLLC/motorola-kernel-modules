/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_MOTO_CMA_H__
#define __MM_MOTO_CMA_H__

#include <trace/events/cma.h>

struct kobject;

int create_cma_sysfs(struct kobject *mm_kobj);
void remove_cma_sysfs(void);
void vh_cma_alloc_start(void *data, const char *name, unsigned long count,
			unsigned int align);
void vh_cma_alloc_finish(void *data, const char *name, unsigned long pfn,
			 const struct page *page, unsigned long count,
			 unsigned int align, int errorno);
#endif
