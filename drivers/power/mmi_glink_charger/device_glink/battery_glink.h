/*
 * Copyright (C) 2024 Motorola Mobility LLC
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
#ifndef __BATTERY_GLINK_H__
#define __BATTERY_GLINK_H__
#include "battery_host.h"
#include "device_class.h"

typedef enum batt_role {
	BATT_MAIN,
	BATT_FLIP,
	BATT_NUM,
} BATT_ROLE;

struct battery_glink_dev {
	char			*name;
	struct device		dev;
	BATT_ROLE batt_role;
	int batt_id;
	struct glink_device *glink_dev;
	struct power_supply	*batt_dev_psy;
	struct battery_info 	batt_dev_info;
	struct battery_info 	batt_prop_info;
	int			state_of_health;
	int			manufacturing_date;
	int			first_usage_date;
	char batt_sn[MMI_BATT_SN_LEN];
	struct notifier_block	batt_nb;

	struct timespec64 glink_access_time;
	uint32_t elapsed_ms;
};

struct glink_device *battery_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void battery_glink_device_unregister(void);
#endif
