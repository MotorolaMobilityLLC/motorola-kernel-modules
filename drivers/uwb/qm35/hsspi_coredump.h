/* SPDX-License-Identifier: GPL-2.0 */

/*
 * This file is part of the QM35 UCI stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 *
 * QM35 COREDUMP layer HSSPI Protocol
 */

#ifndef __HSSPI_COREDUMP_H__
#define __HSSPI_COREDUMP_H__

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/sched.h>

#include "hsspi.h"

struct coredump_packet {
	struct hsspi_block blk;
};

struct coredump_layer {
	struct hsspi_layer hlayer;
	void *coredump_data;
	uint32_t coredump_data_wr_idx;
	uint32_t coredump_size;
	uint16_t coredump_crc;
	uint8_t coredump_status;
	struct timer_list timer;
	wait_queue_head_t wq;
};

void coredump_layer_init(struct coredump_layer *coredump);
void coredump_layer_deinit(struct coredump_layer *coredump);

/**
 * coredump_layer_force_coredump() - force coredump
 * @coredump: pointer to &struct coredump_layer
 *
 * Return: true if ok, false otherwise.
 */
bool coredump_layer_force_coredump(struct coredump_layer *coredump);

/**
 * coredump_layer_new_coredump_available() - checks if the layer has 
 * ack coredump data to be read
 * @coredump: pointer to &struct coredump_layer
 *
 * Return: true if coredump is available, false otherwise.
 */
bool coredump_layer_new_coredump_available(struct coredump_layer *coredump);

/**
 * coredump_layer_get_coredump() - get coredump
 * @coredump: pointer to &struct coredump_layer
 *
 * Return: a &uint8_t pointing to the coredump data if succeed, NULL otherwise
 */
uint8_t *coredump_layer_get_coredump(struct coredump_layer *coredump);

/**
 * coredump_layer_get_coredump_size() - get coredump size
 * @coredump: pointer to &struct coredump_layer
 *
 * Return: a uint32_t holding the size of the current coredump
 */
uint32_t coredump_layer_get_coredump_size(struct coredump_layer *coredump);

/**
 * coredump_layer_reset_status() - resets coredump status
 * @coredump: pointer to &struct coredump_layer
 *
 * Return: void,
 */
void coredump_layer_reset_status(struct coredump_layer *coredump);

#endif // __HSSPI_COREDUMP_H__
