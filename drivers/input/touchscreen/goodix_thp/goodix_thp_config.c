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

/**
* Analyze the configuration of the stylus point reporting rate
* @dev: Device pointer
* @config_count: output parameter, the number of configurations parsed
* Return: Configure the array pointer. NULL indicates that the parsing failed
*/
struct stylus_report_rate_config *parse_stylus_report_rate_config(
	struct device *dev, u8 *config_count)
{
	struct device_node *np = dev->of_node;
	struct device_node *config_np;
	const char *prop_name;
	u8 count = 0;
	int ret = 0;
	int i;
	struct stylus_report_rate_config *configs = NULL;
	u8 raw_data[4];  // [report_rate_high, report_rate_low, command_high, command_low]

	if (!np || !config_count) {
		ts_err(dev, "Invalid parameters");
		return NULL;
	}

	config_np = of_get_child_by_name(np, "goodix,stylus-report-rate-config");
	if (!config_np) {
		ts_err(dev, "No stylus report rate config found");
		return NULL;
	}

	ret = of_property_read_u8(config_np, "goodix,rate-config-count", &count);
	if (ret) {
		ts_err(dev, "Failed to read rate-config-count: %d", ret);
		goto out_put_node;
	}

	if (count == 0) {
		ts_err(dev, "Invalid config count: 0");
		goto out_put_node;
	}

	configs = kzalloc(sizeof(struct stylus_report_rate_config) * count, GFP_KERNEL);
	if (!configs) {
		ts_err(dev, "Failed to allocate memory for configs");
		goto out_put_node;
	}

	for (i = 0; i < count; i++) {
		prop_name = kasprintf(GFP_KERNEL, "goodix,stylus-report-rate-config-%d", i);
		if (!prop_name) {
			ts_err(dev, "Failed to allocate property name for config %d", i);
			goto out_cleanup;
		}

		ret = of_property_read_u8_array(config_np, prop_name, raw_data, 4);
		if (ret) {
			ts_err(dev, "Can't read stylus config %s: %d", prop_name, ret);
			kfree(prop_name);
			goto out_cleanup;
		}

		kfree(prop_name);

		configs[i].report_rate = (raw_data[0] << 8) | raw_data[1];
		configs[i].command = (raw_data[2] << 8) | raw_data[3];

		ts_info(dev, "Parsed config %d: report_rate=0x%04x (%dHz), command=0x%04x",
			i, configs[i].report_rate, configs[i].report_rate,
			configs[i].command);
	}

	*config_count = count;
	ts_info(dev, "Successfully parsed %d stylus report rate configs", count);

out_put_node:
	of_node_put(config_np);
	return configs;

out_cleanup:
	kfree(configs);
	configs = NULL;
	goto out_put_node;
}

/**
* Release the memory of the configuration array
* @configs: Configure array Pointers
*/
void free_stylus_report_rate_config(struct stylus_report_rate_config *configs)
{
	kfree(configs);
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

