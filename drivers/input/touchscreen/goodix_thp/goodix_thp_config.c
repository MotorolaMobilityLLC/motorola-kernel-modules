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
#include "goodix_thp_config.h"

struct goodix_ic_report_rate_config report_rate_config_info;

int parse_report_rate_config(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *rate_node;
	int ret, i;
	u8 raw_data[9];
	struct goodix_ic_report_rate_config *config = &report_rate_config_info;
	int arry_size;

	rate_node = of_get_child_by_name(np, "goodix,report-rate-config");
	if (!rate_node) {
		ts_err(dev, "No report rate config found");
		return -ENODATA;
	}

	/* read the basic config */
	ret = of_property_read_u8(rate_node, "goodix,rate-config-count",
                            &config->rate_config_count);
	if (ret) {
		ts_err(dev, "Can't read rate config count");
		goto out;
	}

	config->refresh_rate_ctrl = of_property_read_bool(rate_node,
                                    "goodix,refresh-rate-ctrl");
	config->interpolation_ctrl = of_property_read_bool(rate_node,
                                    "goodix,interpolation-ctrl");

	/* limit the max rate config count */
	if (config->rate_config_count > MAX_REPORT_RATE_CONFIG) {
		ts_debug(dev, "Rate config count exceeds max, truncating to %d", MAX_REPORT_RATE_CONFIG);
		config->rate_config_count = MAX_REPORT_RATE_CONFIG;
	}

	if (config->refresh_rate_ctrl) {
		arry_size = 9;
	} else {
		arry_size = 5;
	}

	/* parse the every config item */
	for (i = 0; i < config->rate_config_count; i++) {
		char prop_name[32];
		struct report_rate_config *rate_info = &config->report_rate_info[i];

		snprintf(prop_name, sizeof(prop_name),
                "goodix,report-rate-config-%d", i);

		ret = of_property_read_u8_array(rate_node, prop_name, raw_data, arry_size);
		if (ret) {
			ts_err(dev, "Can't read config %d: %d", i, ret);
			goto out;
		}

		/* fill the rate info struct */
		rate_info->interpolation_flag = raw_data[0];
		if (config->refresh_rate_ctrl) {
			rate_info->refresh_rate[0] = (raw_data[1] << 8) | raw_data[2];
			rate_info->refresh_rate[1] = (raw_data[3] << 8) | raw_data[4];
			rate_info->report_rate = (raw_data[5] << 8) | raw_data[6];
			rate_info->command = (raw_data[7] << 8) | raw_data[8];
		} else {
			rate_info->report_rate = (raw_data[1] << 8) | raw_data[2];
			rate_info->command = (raw_data[3] << 8) | raw_data[4];
		}
	}

out:
	of_node_put(rate_node);
	return ret;
}

int goodix_thp_mmi_get_report_rate(struct goodix_thp_core *core_data)
{
	int refresh_rate_ctrl = 0;
	int interpolation_ctrl = 0;
	int interpolation_flag = 0;
	int refresh_rate = 0;
	int i = 0;
	struct device *tdev = core_data->ts_dev->dev;

	refresh_rate_ctrl = core_data->ts_dev->board_data.report_rate_ctrl;
	interpolation_ctrl = core_data->ts_dev->board_data.interpolation_ctrl;

	interpolation_flag = core_data->get_mode.interpolation;
	refresh_rate = core_data->refresh_rate;

	ts_debug(tdev, "refresh_rate_ctrl: %d, interpolation_ctrl: %d, interpolation_flag: %d, refresh_rate: %d",
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
		ts_err(tdev, "Get config report rate fail");
		return -1;
	} else {
		ts_debug(tdev, "Get config report rate %dHZ, command : 0x%02x ",
			report_rate_config_info.report_rate_info[i].report_rate,
			report_rate_config_info.report_rate_info[i].command);
		return report_rate_config_info.report_rate_info[i].command;
	}
}

