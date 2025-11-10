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
#ifndef __GOODIX_THP_CONFIG_H__
#define __GOODIX_THP_CONFIG_H__

#include <linux/touchscreen_u_mmi.h>
#include "goodix_thp.h"

#define MAX_REPORT_RATE_CONFIG 12
#define REPORT_RATE_CMD_180HZ 0x9D00
#define REPORT_RATE_CMD_240HZ 0x9D01
#define REPORT_RATE_CMD_360HZ 0x9D02
#define REPORT_RATE_CMD_120HZ 0x9D03
#define REPORT_RATE_CMD_LOW 0x9D
#define REPORT_RATE_CMD_480HZ 0xC102
#define REPORT_RATE_CMD_576HZ 0xC103
#define REPORT_RATE_CMD_720HZ 0xC101
#define REPORT_RATE_CMD_HIGH 0xC1

struct report_rate_config {
	u8 interpolation_flag; //if enable interpolation report rate
	u16 refresh_rate[2]; //display refresh rate
	u16 report_rate; //touch report rate
	u16 command; //report rate switch command
};

struct goodix_ic_report_rate_config {
	u8 rate_config_count; //the count of report rate combination
	bool refresh_rate_ctrl; //if support report rate change according to refresh rate
	bool interpolation_ctrl; //if support interpolation report rate
	struct report_rate_config report_rate_info[MAX_REPORT_RATE_CONFIG];
};

/* Stylus reporting Rate configuration structure */
struct stylus_report_rate_config {
	u16 report_rate;  // Reporting rate value
	u16 command;      // Switch commands
};

extern struct goodix_ic_report_rate_config report_rate_config_info;
extern struct stylus_report_rate_config *rate_configs;

int goodix_thp_mmi_get_report_rate(struct goodix_thp_core *core_data);
int parse_report_rate_config(struct device *dev);
struct stylus_report_rate_config *parse_stylus_report_rate_config(
	struct device *dev, u8 *config_count);
void free_stylus_report_rate_config(struct stylus_report_rate_config *configs);

#endif
