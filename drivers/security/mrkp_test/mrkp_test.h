// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Test module for motorola's hypervisor-level runtime kernel protections.
 */

#ifndef _MOTO_RKP_TEST_H_
#define _MOTO_RKP_TEST_H_

#include <linux/compiler.h>
#include <linux/dcache.h>
#include <linux/magic.h>
#include <linux/types.h>
#include <linux/rcupdate.h>
#include <linux/refcount.h>
#include <linux/workqueue.h>


typedef enum {
	KERN_REGION_JEL = 0x1,
	KERN_REGION_TEXT = 0x2,
	KERN_REGION_RODATA = 0x3,
} MRKP_REGION_T;

int mrkp_get_krn_region_info(uint64_t *start_addr, uint64_t *end_addr, MRKP_REGION_T region_type);


#endif /* _MOTO_MKP_TEST_H_ */
