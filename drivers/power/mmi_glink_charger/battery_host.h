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

#ifndef __MMI_BATT_HOST_H__
#define __MMI_BATT_HOST_H__
#include <linux/thermal.h>
#include "mmi_glink_core.h"

#define MMI_BATT_SN_LEN 16

struct battery_info {
	bool present;
	int batt_uv;
	int batt_ua;
	int batt_soc; /* 0 ~ 10000 indicating 0% to 100% */
	int batt_temp; /* hundredth degree */
	int batt_status;
	int batt_soh; /*state of health*/
	int batt_cycle;
	int batt_full_uah;
	int batt_design_uah;
	int batt_chg_counter;
	int batt_fv_uv;
	int batt_fcc_ua;
	int batt_cutoff_mv;
};

struct battery_host {
	struct device		*dev;
	struct power_supply	*batt_psy;
	int			batt_dev_num;
	int			init_cycles;
	int			age;
	int			state_of_health;
	int			manufacturing_date;
	int			first_usage_date;
	int 			max_fcc_ua;
	int 			demo_fv_mv;
	int 			chrg_iterm_ma;
//	char			batt_sn[MMI_BATT_SN_LEN];

	u32 *thermal_primary_levels;
	u32 thermal_primary_fcc_ua;
	int curr_thermal_primary_level;
	int num_thermal_primary_levels;
	struct thermal_cooling_device *primary_tcd;

	u32 *thermal_secondary_levels;
	u32 thermal_secondary_fcc_ua;
	int android_auto_mode;
	int curr_thermal_secondary_level;
	int num_thermal_secondary_levels;
	struct thermal_cooling_device *secondary_tcd;

};

void battery_supply_init(struct battery_host *batt_host);
void battery_supply_deinit(struct battery_host *batt_host);
void battery_glink_host_deinit(void);
#endif
