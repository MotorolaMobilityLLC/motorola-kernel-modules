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
#ifndef __BALANCE_GLINK_H__
#define __BALANCE_GLINK_H__
#include "device_class.h"

struct balance_dev_info
{
	int chg_role;
	int work_mode;
	int int_stat;

	int ibat_ma;
	int ibat_limit;

	int vchg_mv;
	int vbat_mv;

	int batt_temp;
	int die_temp;

	int ls_off;
	int auto_bsm_dis;
	int lpm_mode;
	int extmos_en;
};

struct balance_glink_dev {
	char			*name;
	struct device		dev;
	DEV_ROLE dev_role;
	int batt_id;
	struct glink_device *glink_dev;
	struct power_supply	*balance_dev_psy;
	struct balance_dev_info 	balance_dev_info;
	struct notifier_block	balance_nb;

	struct timespec64 glink_access_time;
	uint32_t elapsed_ms;
};

struct glink_device *balance_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void balance_glink_device_unregister(void);
#endif

