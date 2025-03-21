/*
 * Copyright (C) 2022 Motorola Mobility LLC
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
#include "goodix_thp_config.h"

struct goodix_ic_report_rate_config report_rate_config_info = {
#if 0
	.rate_config_count = 3,
	.refresh_rate_ctrl = 1,
	.interpolation_ctrl = 1,
	{
		{
			.interpolation_flag = 0,
			.refresh_rate = {1, 120},
			.report_rate = 130,
			.command = 0x9D03,
		},
		{
			.interpolation_flag = 0,
			.refresh_rate = {144, 165},
			.report_rate = 220,
			.command = 0x9D01,
		},
		{
			.interpolation_flag = 1,
			.refresh_rate = {1, 165},
			.report_rate = 300,
			.command = 0x9D02,
		},
	}
#endif

	.rate_config_count = 2,
	.refresh_rate_ctrl = 0,
	.interpolation_ctrl = 1,
	{
		{
			.interpolation_flag = 0,
			.report_rate = 130,
			.command = 0x9D03,
		},
		{
			.interpolation_flag = 1,
			.report_rate = 300,
			.command = 0x9D02,
		},
	}
};

int goodix_thp_mmi_get_report_rate(struct goodix_thp_core *core_data)
{
	int refresh_rate_ctrl = 0;
	int interpolation_ctrl = 0;
	int interpolation_flag = 0;
	int refresh_rate = 0;
	int i = 0;

	refresh_rate_ctrl = core_data->ts_dev->board_data.report_rate_ctrl;
	interpolation_ctrl = core_data->ts_dev->board_data.interpolation_ctrl;

	interpolation_flag = core_data->get_mode.interpolation;
	refresh_rate = core_data->refresh_rate;

	ts_debug("refresh_rate_ctrl: %d, interpolation_ctrl: %d, interpolation_flag: %d, refresh_rate: %d",
		refresh_rate_ctrl, interpolation_ctrl, interpolation_flag, refresh_rate);

	if (refresh_rate_ctrl == 0 && interpolation_ctrl == 1) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if (interpolation_flag == report_rate_config_info.report_rate_info[i].interpolation_flag) {
				break;
			}
		}
	} else if (refresh_rate_ctrl == 1 && interpolation_ctrl == 1) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if ((interpolation_flag == report_rate_config_info.report_rate_info[i].interpolation_flag) &&
				((refresh_rate >= report_rate_config_info.report_rate_info[i].refresh_rate[0]) &&
				(refresh_rate <= report_rate_config_info.report_rate_info[i].refresh_rate[1]))) {
				break;
			}
		}
	} else if (refresh_rate_ctrl == 1 && interpolation_ctrl == 0) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if ((refresh_rate >= report_rate_config_info.report_rate_info[i].refresh_rate[0]) &&
				(refresh_rate <= report_rate_config_info.report_rate_info[i].refresh_rate[1])) {
				break;
			}
		}
	} else {
		//refresh_rate_ctrl = 0, interpolation_ctrl = 0
		i = 0;
	}

	if (i == report_rate_config_info.rate_config_count) {
		ts_err("Get config report rate fail");
		return -1;
	} else {
		ts_debug("Get config report rate %dHZ, command : 0x%02x ",
			report_rate_config_info.report_rate_info[i].report_rate,
			report_rate_config_info.report_rate_info[i].command);
		return report_rate_config_info.report_rate_info[i].command;
	}
}

