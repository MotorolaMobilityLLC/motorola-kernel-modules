/*
 * Copyright (C) 2025 Motorola Mobility LLC
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

#ifndef _MSCHED_OEMDATA_H_
#define _MSCHED_OEMDATA_H_

#ifdef CONFIG_MOTO_LOCKING_2
int msched_oemdata_init(void);
void msched_oemdata_deinit(void);
#endif

#endif
