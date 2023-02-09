/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include "smart_battery_core.h"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for MMI DISCRETE CHARGER driver");

static struct mmi_smart_battery *this_chip = NULL;

static int mmi_get_batt_capacity_level(struct mmi_smart_battery *chip)
{
	int uisoc = chip->uisoc;

	if (uisoc >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (uisoc >= 80 && uisoc < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (uisoc >= 20 && uisoc < 80)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (uisoc > 0 && uisoc < 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (uisoc == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int batt_get_prop(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct mmi_smart_battery *chip = power_supply_get_drvdata(psy);
	int ret = 0, temp = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mmi_charger_update_batt_status();
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = gauge_dev_get_voltage_now(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->voltage_now = temp;
		val->intval = chip->voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = gauge_dev_get_current_now(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->current_now = temp;
		val->intval = chip->current_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->fake_soc > 0) {
			val->intval = chip->fake_soc;
			break;
		}

		ret = gauge_dev_get_capacity(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->uisoc = temp;
		val->intval = chip->uisoc;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = mmi_get_batt_capacity_level(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (chip->fake_temp != -EINVAL) {
			val->intval = chip->fake_temp;
			break;
		}

		ret = gauge_dev_get_temperature(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->batt_temp = temp;
		val->intval = chip->batt_temp;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = gauge_dev_get_tte(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->batt_tte = temp;
		val->intval = chip->batt_tte;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = gauge_dev_get_charge_full(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->charge_full = temp;
		val->intval = chip->charge_full * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = gauge_dev_get_charge_full_design(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->charge_full_design = temp;
		val->intval = chip->charge_full_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = gauge_dev_get_cycle_count(chip->gauge_dev, &temp);
		if (ret >= 0)
			chip->cycle_count = temp;
		val->intval = chip->cycle_count;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mmi_batt_health_check();
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int batt_set_prop(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct mmi_smart_battery *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		chip->fake_temp = val->intval;
		gauge_dev_set_temperature(chip->gauge_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_soc = val->intval;
		gauge_dev_set_capacity(chip->gauge_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		gauge_dev_set_charge_type(chip->gauge_dev, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int batt_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TYPE:
		return 1;
	default:
		break;
	}
	return 0;
}

static const struct power_supply_desc batt_psy_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= batt_get_prop,
	.set_property	= batt_set_prop,
	//.external_power_changed = batt_external_power_changed,
	.property_is_writeable = batt_prop_is_writeable,
	.properties	= batt_props,
	.num_properties	= ARRAY_SIZE(batt_props),
};

static int  tcmd_get_bat_temp(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->batt_temp / 10;

	return ret;
}

static int  tcmd_get_bat_voltage(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->voltage_now * 1000;

	return ret;
}

static int  tcmd_get_bat_ocv(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->voltage_now;

	return ret;
}

static int battery_tcmd_register(struct mmi_smart_battery *chip)
{
	int ret = 0;

	chip->batt_tcmd_client.data = chip;
	chip->batt_tcmd_client.client_id = MOTO_CHG_TCMD_CLIENT_BAT;

	chip->batt_tcmd_client.get_bat_temp = tcmd_get_bat_temp;
	chip->batt_tcmd_client.get_bat_voltage = tcmd_get_bat_voltage;
	chip->batt_tcmd_client.get_bat_ocv= tcmd_get_bat_ocv;

	ret = moto_chg_tcmd_register(&chip->batt_tcmd_client);

	return ret;
}

static int smart_battery_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mmi_smart_battery *chip;
	struct power_supply_config psy_cfg = {};

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;
	platform_set_drvdata(pdev, chip);
	this_chip = chip;
	device_init_wakeup(chip->dev, true);

	chip->fake_soc	= -EINVAL;
	chip->fake_temp	= -EINVAL;

	chip->gauge_dev = get_gauge_by_name("bms");
	if (chip->gauge_dev) {
		mmi_info(chip, "Found bms\n");
		gauge_dev_set_drvdata(chip->gauge_dev, chip);
	} else {
		mmi_err(chip, "*** Error : can't find bms ***\n");
		return -EPROBE_DEFER;
	}

	chip->debug_enabled = &debug_enabled;

	chip->batt_psy = devm_power_supply_register(chip->dev,
						    &batt_psy_desc,
						    &psy_cfg);
	if (IS_ERR(chip->batt_psy)) {
		mmi_err(chip,
			"Failed: batt power supply register\n");
		rc = PTR_ERR(chip->batt_psy);
		goto cleanup;
	}

	battery_tcmd_register(chip);

	return rc;

cleanup:
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int smart_battery_remove(struct platform_device *pdev)
{
	//struct mmi_smart_battery *chip = platform_get_drvdata(pdev);


	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,smart-battery", },
	{ },
};

static struct platform_driver smart_battery_driver = {
	.driver		= {
		.name		= "mmi,smart-battery",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= smart_battery_probe,
	.remove		= smart_battery_remove,
};
module_platform_driver(smart_battery_driver);

MODULE_DESCRIPTION("MMI Smart Battery Driver");
MODULE_LICENSE("GPL v2");