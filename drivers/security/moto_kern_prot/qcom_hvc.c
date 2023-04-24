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
#include <linux/qcom_scm.h>
#include <soc/qcom/qseecom_scm.h>
#include <soc/qcom/qseecomi.h>

#include "rkp_hvc_api.h"

int mark_range_ro_smc(uint64_t start, uint64_t end)
{
	struct qseecom_scm_desc desc = { 0 };

	desc.arginfo = TZ_SYSCALL_CREATE_PARAM_ID_8(0, 0, 0, 0, 0, 0, 0, 0);
	desc.args[0] = __virt_to_phys(start);
	desc.args[1] = __virt_to_phys(end);
	return qcom_scm_qseecom_call(MOTO_RKP_SMCID | MARK_RANGE_RO, &desc, 1);
}

int add_jump_entry_lookup(uint64_t vaddr, uint64_t size)
{
	struct qseecom_scm_desc desc = { 0 };

	desc.arginfo = TZ_SYSCALL_CREATE_PARAM_ID_8(0, 0, 0, 0, 0, 0, 0, 0);
	desc.args[0] = vaddr;
	desc.args[1] = size;
	return qcom_scm_qseecom_call(MOTO_RKP_SMCID | ADD_JUMP_LABEL_LOOKUP,
				     &desc, 1);
}

int lock_rkp(void)
{
	struct qseecom_scm_desc desc = { 0 };

	desc.arginfo = TZ_SYSCALL_CREATE_PARAM_ID_8(0, 0, 0, 0, 0, 0, 0, 0);
	return qcom_scm_qseecom_call(MOTO_RKP_SMCID | LOCK_RKP, &desc, 1);
}

int prepare_hugepage(uint64_t addr)
{
	struct qseecom_scm_desc desc = { 0 };

	desc.arginfo = TZ_SYSCALL_CREATE_PARAM_ID_8(0, 0, 0, 0, 0, 0, 0, 0);
	desc.args[0] = __virt_to_phys(addr);
	return qcom_scm_qseecom_call(MOTO_RKP_SMCID | PREPARE_BLOCK, &desc, 1);
}

int commit_hugepage(void)
{
	struct qseecom_scm_desc desc = { 0 };

	desc.arginfo = TZ_SYSCALL_CREATE_PARAM_ID_8(0, 0, 0, 0, 0, 0, 0, 0);
	return qcom_scm_qseecom_call(MOTO_RKP_SMCID | COMMIT_BLOCK, &desc, 1);
}
