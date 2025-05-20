/*
 * Copyright (C) 2020 Motorola Mobility LLC
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

#ifndef __MMI_GLINK_CHARGER_API_H__
#define __MMI_GLINK_CHARGER_API_H__

int mmi_vote_charging_disable(const char *voter, bool enable);
int mmi_vote_charger_suspend(const char *voter, bool enable);
#endif
