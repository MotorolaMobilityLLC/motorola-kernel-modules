// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Motorola Mobility, Inc.
 *
 * Author: Maxwell Bland
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
 * Implements the Motorola Kernel Integrity Protection Hypervisor API.
 */
#include <linux/memory.h>
#include <linux/arm-smccc.h>

#include "rkp_hvc_api.h"

/**
 * mrkp_smc - raw arm SMC call for mrkp
 * @id: SMC call ID
 * @arg0: argument in x2 register
 * @arg1: argument in x3 register
 *
 * Unfortunately using the QCOM API leads to a very infrequent
 * crash as it attempts to acquire a mutex while we may be
 * performing a cross-CPU spinlock. arg0 and arg1 are at the
 * intentionally wrong places here as the QCOM API misplaced
 * them into x2/x3 rather than x1, and for historical reasons
 * we adapted.
 */
static void mrkp_smc(uint64_t id, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
  struct arm_smccc_res res;
  struct arm_smccc_quirk quirk = { .id = ARM_SMCCC_QUIRK_QCOM_A6 };
  quirk.state.a6 = 0;
  do {
    arm_smccc_smc_quirk(MOTO_RKP_SMCID | id, 0, arg0, arg1, arg2, 0,
                        quirk.state.a6, 0, &res, &quirk);
  } while (res.a0);
}

void mark_range_ro_smc(uint64_t start, uint64_t end, uint64_t type)
{
	mrkp_smc(KERN_MARK_RANGE_RO_SMC_ID, start, end, 0);
}

void add_jump_entry_lookup(uint64_t paddr, uint64_t size)
{
	mrkp_smc(KERN_ADD_JUMP_ENTRY_LOOKUP_SMC_ID, paddr, size, 0);
}

void lock_rkp(void)
{
	mrkp_smc(KERN_LOCK_RKP_SMC_ID, 0, 0, 0);
}

void amem_register(uint64_t paddr, uint64_t size)
{
	mrkp_smc(KERN_REGISTER_AMEM_SMC_ID, paddr, size, 0);
}
