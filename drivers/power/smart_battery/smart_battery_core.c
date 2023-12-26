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
#include <linux/list.h>
#include "smart_battery_core.h"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for MMI DISCRETE CHARGER driver");

static struct mmi_smart_battery *this_chip = NULL;

#ifndef MAX
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#endif
#ifndef MIN
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

static int smart_batt_set_temperature(struct mmi_smart_battery *chip, int intval)
{
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_set_temperature(battery->gauge_dev, intval);
	}

	return 0;
}

static int smart_batt_set_shutdown_threshold(struct mmi_smart_battery *chip, int shutdown_threshold)
{
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_set_shutdown_threshold(battery->gauge_dev, shutdown_threshold);
	}

	return 0;
}

static int smart_batt_set_charge_type(struct mmi_smart_battery *chip, int intval)
{
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_set_charge_type(battery->gauge_dev, intval);
	}

	return 0;
}

static int smart_batt_get_charge_full(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int charge_full_total = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_charge_full(battery->gauge_dev, &battery->charge_full);
		charge_full_total +=  battery->charge_full;
	}
	chip->combo_charge_full= charge_full_total;

	return chip->combo_charge_full;
}

static int smart_batt_get_charge_full_design(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int design_total = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_charge_full_design(battery->gauge_dev, &battery->charge_full_design);
		design_total +=  battery->charge_full_design;
	}
	chip->combo_charge_full_design= design_total;

	return chip->combo_charge_full_design;
}

static int smart_batt_get_charge_counter(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int counter = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_charge_counter(battery->gauge_dev, &battery->charge_counter);
		counter +=  battery->charge_counter;
	}
	chip->combo_charge_counter= counter;

	return chip->combo_charge_counter;
}

static int smart_batt_get_cycle_count(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int counter = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_cycle_count(battery->gauge_dev, &battery->cycle_count);
		counter = MAX(counter, battery->cycle_count);
	}
	chip->combo_cycle_count = counter;

	return chip->combo_cycle_count;
}

static int smart_batt_get_temperature(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_temperature(battery->gauge_dev, &battery->batt_temp);
		if (strcmp(battery->gauge_dev->dev.kobj.name, "bms") == 0 ||
			strcmp(battery->gauge_dev->dev.kobj.name, "main_battery") == 0) {
			chip->combo_batt_temp = battery->batt_temp;
		}
	}

	return chip->combo_batt_temp;
}

static int smart_batt_get_soh(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int batt_soh = 100;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_soh(battery->gauge_dev, &battery->soh);
		batt_soh = MIN(batt_soh, battery->soh);
	}
	chip->combo_soh = batt_soh;
	//chip->combo_soh = chip->combo_charge_full * 100 / chip->combo_charge_full_design;

	return chip->combo_soh;
}

static int smart_batt_get_current_now(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int current_total = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_current_now(battery->gauge_dev, &battery->current_now);
		current_total +=  battery->current_now;
	}
	chip->combo_current_now = current_total * 1000;

	return chip->combo_current_now;
}

static int smart_batt_get_voltage_now(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int voltage_total = 0;

	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_voltage_now(battery->gauge_dev, &battery->voltage_now);
		if (strcmp(battery->gauge_dev->dev.kobj.name, "bms") == 0 ||
			strcmp(battery->gauge_dev->dev.kobj.name, "main_battery") == 0) {
			voltage_total +=  battery->voltage_now;
		}
	}
	chip->combo_voltage_now = voltage_total * 1000;

	return chip->combo_voltage_now;
}

static int smart_batt_get_capacity(struct mmi_smart_battery *chip)
{
	struct mmi_battery_pack *battery = NULL;
	int capacity_total = 0;
	int rsoc = 0;
	list_for_each_entry(battery, &chip->battery_list, list) {
		gauge_dev_get_capacity(battery->gauge_dev, &battery->soc);
		capacity_total += battery->soc * battery->charge_full;
	}
	rsoc = capacity_total / chip->combo_charge_full;

	return rsoc;
}

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
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
};

static int batt_get_prop(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct mmi_smart_battery *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mmi_charger_update_batt_status();
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (chip->combo_voltage_now == -EINVAL || (mmi_charger_update_batt_status() == POWER_SUPPLY_STATUS_CHARGING)) {
			smart_batt_get_voltage_now(chip);
		}
		val->intval = chip->combo_voltage_now;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->combo_current_now == -EINVAL || (mmi_charger_update_batt_status() == POWER_SUPPLY_STATUS_CHARGING)) {
			smart_batt_get_current_now(chip);
		}
		val->intval = chip->combo_current_now;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->fake_soc > 0) {
			val->intval = chip->fake_soc;
			break;
		}
		if(chip->uisoc == -EINVAL)
			val->intval = smart_batt_get_capacity(chip);
		else
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
		if (chip->combo_batt_temp == -EINVAL) {
			smart_batt_get_temperature(chip);
		}
		val->intval = chip->combo_batt_temp;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = chip->batt_tte;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = chip->combo_charge_full * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->combo_charge_full_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = chip->combo_charge_counter* 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->combo_cycle_count;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mmi_batt_health_check();
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = chip->combo_soh;
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
		smart_batt_set_temperature(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		smart_batt_set_charge_type(chip, val->intval);
		chip->is_ffc_charge = val->intval;
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

#define CAP(min, max, value)			\
		((min > value) ? min : ((value > max) ? max : value))

static int smart_batt_monotonic_soc(struct mmi_smart_battery *chip, int rsoc)
{
	int uisoc = rsoc;

	mmi_info(chip, "rsoc = %d, chip->uisoc = %d\n", rsoc, chip->uisoc);
	if (chip->uisoc == -EINVAL)
		return uisoc;

	if (mmi_charger_update_batt_status() == POWER_SUPPLY_STATUS_FULL)
		return 100;

	if (rsoc > chip->uisoc) {
		/* SOC increased */
		if (chip->combo_current_now > 0) {
			uisoc = chip->uisoc + 1;
		} else
			uisoc = chip->uisoc;
	} else if (rsoc < chip->uisoc) {
		/* SOC dropped */
		if (chip->combo_current_now< 0) {
			uisoc = chip->uisoc - 1;
		} else
			uisoc = chip->uisoc;
	}
	uisoc = CAP(0, 100, uisoc);

	return uisoc;
}

#define TAPER_COUNT 5
static int smart_batt_soc100_forward(struct mmi_smart_battery *chip, int rsoc)
{
	int logic_soc;

	logic_soc = (rsoc * 100 * 100 / chip->ui_full_soc + 50) / 100;
	if (chip->soc100_curr_threshod) {
		if ((mmi_charger_update_batt_status() == POWER_SUPPLY_STATUS_CHARGING) &&
			(chip->is_ffc_charge) && (logic_soc >= 100) && (chip->uisoc != 100)) {

			if (chip->combo_current_now <= chip->soc100_curr_threshod)  {
				if (chip->taper_count >= TAPER_COUNT)
					chip->taper_count = 0;
				else
					chip->taper_count ++;
			} else {
				chip->taper_count = 0;
			}

			if ( chip->taper_count < TAPER_COUNT) {
				logic_soc = 99;
				mmi_info(chip, "the current is greater than requested,force soc to 99");
			}
		}
	}

	mmi_info(chip, "original_soc=%d, logic_soc=%d", rsoc, logic_soc);

	if (logic_soc > 100)
		logic_soc = 100;

	return logic_soc;
}

static void smart_batt_update_thread(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct mmi_smart_battery *chip;
	int rsoc;

	delay_work = container_of(work, struct delayed_work, work);
	chip = container_of(delay_work, struct mmi_smart_battery, battery_delay_work);

	smart_batt_get_voltage_now(chip);
	smart_batt_get_current_now(chip);
	smart_batt_get_temperature(chip);
	smart_batt_get_soh(chip);
	smart_batt_get_charge_full_design(chip);
	smart_batt_get_charge_full(chip);
	smart_batt_get_cycle_count(chip);
	rsoc = smart_batt_get_capacity(chip);
	smart_batt_get_charge_counter(chip);
	rsoc = smart_batt_soc100_forward(chip, rsoc);
	rsoc = smart_batt_monotonic_soc(chip, rsoc);
	if (chip->batt_psy) {
		if (rsoc != chip->uisoc) {
			chip->uisoc = rsoc;
			power_supply_changed(chip->batt_psy);
		}
	}
	if (chip->sync_boardtemp_to_fg)
		smart_batt_set_temperature(chip, chip->combo_batt_temp);

	mmi_info(chip, "UISOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		chip->uisoc, chip->combo_voltage_now, chip->combo_current_now, chip->combo_batt_temp);

	queue_delayed_work(chip->fg_workqueue, &chip->battery_delay_work, msecs_to_jiffies(QUEUS_DELAYED_WORK_TIME));

}

static int  tcmd_get_bat_temp(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->combo_batt_temp / 10;

	return ret;
}

static int  tcmd_get_bat_voltage(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->combo_voltage_now * 1000;

	return ret;
}

static int  tcmd_get_bat_ocv(void *input, int* val)
{
	int ret = 0;
	struct mmi_smart_battery *chip = (struct mmi_smart_battery *)input;

	*val = chip->combo_voltage_now;

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

static int smart_battery_suspend(struct device *dev)
{
	struct mmi_smart_battery *chip = this_chip;

	cancel_delayed_work(&chip->battery_delay_work);
	chip->resume_completed = false;

	return 0;
}


static int smart_battery_resume(struct device *dev)
{
	struct mmi_smart_battery *chip = this_chip;

	chip->resume_completed = true;
	queue_delayed_work(chip->fg_workqueue, &chip->battery_delay_work, msecs_to_jiffies(1));

	return 0;
}

static int smart_battery_parse_dt(struct mmi_smart_battery *chip)
{
	struct device_node *np = chip->dev->of_node;
	int i, rc;
	chip->sync_boardtemp_to_fg = of_property_read_bool(np , "mmi,sync_boardtemp_to_fg");

	if (of_property_read_u32(np, "mmi,ui_full_soc", &chip ->ui_full_soc) < 0) {
		chip ->ui_full_soc = 100;
	}

	of_property_read_u32(np , "mmi,shutdown_vol_threshold", &chip->shutdown_threshold);

	if (chip ->ui_full_soc != 100) {
		of_property_read_u32(np , "mmi,soc100_curr_threshod", &chip->soc100_curr_threshod);
	}

	chip->gauge_count = of_property_count_strings(np, "mmi,gauge_names");
	if (chip->gauge_count < 0) {
		chip->gauge_count = 1;
	}

	mmi_info(chip, "gauge_cnt=%d", chip->gauge_count);
	chip->gauge_name_arry = devm_kzalloc(chip->dev, chip->gauge_count  * sizeof(char *), GFP_KERNEL);
	if (chip->gauge_name_arry) {
		for (i = 0; i < chip->gauge_count; i++) {
			rc = of_property_read_string_index(np, "mmi,gauge_names", i,
							    &chip->gauge_name_arry[i]);
			if (rc < 0) {
				chip->gauge_name_arry[i] = "bms";
				mmi_info(chip, "use default gauges_name bms\n");
			}
			mmi_info(chip, "support gauges_name[%d](%s)\n", i, chip->gauge_name_arry[i]);
		}
	}

	return 0;
}
static void smart_battery_init_data(struct mmi_smart_battery *chip)
{
	smart_batt_get_voltage_now(chip);
	smart_batt_get_current_now(chip);
	smart_batt_get_temperature(chip);
	smart_batt_get_soh(chip);
	smart_batt_get_charge_full_design(chip);
	smart_batt_get_charge_full(chip);
	smart_batt_get_cycle_count(chip);
	chip->uisoc = smart_batt_get_capacity(chip);
	smart_batt_get_charge_counter(chip);
}

static int smart_battery_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mmi_smart_battery *chip;
	struct power_supply_config psy_cfg = {};
	int default_gauge_count = 0;
	int i;

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
	chip->resume_completed = true;
	chip->uisoc = -EINVAL;
	chip->combo_soh= 100;
	chip->combo_voltage_now = -EINVAL;
	chip->combo_current_now = -EINVAL;
	chip->combo_batt_temp = -EINVAL;
	chip->shutdown_threshold = -EINVAL;
	chip->soc100_curr_threshod = 0;
	chip->taper_count = 0;
	chip->gauge_count = -ENODATA;
	chip->battery = NULL;
	smart_battery_parse_dt(chip);
	INIT_LIST_HEAD(&chip->battery_list);

	chip->battery = (struct mmi_battery_pack *)devm_kzalloc(chip->dev,  sizeof(struct mmi_battery_pack ) * (chip->gauge_count) , GFP_KERNEL);
	if (!chip->battery) {
		mmi_info(chip, "devm_kzalloc mmi_battery_pack error\n");
		goto cleanup;
	}
	default_gauge_count = chip->gauge_count;
	for (i = 0; i < default_gauge_count; i++) {
		chip->battery[i].gauge_dev = get_gauge_by_name(chip->gauge_name_arry[i]);
		if (chip->battery[i].gauge_dev) {
			mmi_info(chip, "Found gauge_name=%s\n", chip->gauge_name_arry[i]);
			gauge_dev_set_drvdata(chip->battery[i].gauge_dev, chip);
			list_add_tail(&chip->battery[i].list, &chip->battery_list);
		} else {
			mmi_err(chip, "*** Error : can't find gauge_name of %s ***\n", chip->gauge_name_arry[i]);
			if (chip->gauge_count > 1)
				chip->gauge_count--;
			else
				goto cleanup;
		}
	}

	smart_battery_init_data(chip);

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

	if (chip->shutdown_threshold != -EINVAL)
		smart_batt_set_shutdown_threshold(chip, chip->shutdown_threshold);

	chip->fg_workqueue = create_singlethread_workqueue("smart_battery");
	INIT_DELAYED_WORK(&chip->battery_delay_work, smart_batt_update_thread);
	queue_delayed_work(chip->fg_workqueue, &chip->battery_delay_work , msecs_to_jiffies(QUEUE_START_WORK_TIME));

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

static const struct dev_pm_ops smart_battery_pm_ops = {
	.resume	= smart_battery_resume,
	.suspend	= smart_battery_suspend,
};

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,smart-battery", },
	{ },
};

static struct platform_driver smart_battery_driver = {
	.driver		= {
		.name		= "mmi,smart-battery",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
		.pm     = &smart_battery_pm_ops,
	},
	.probe		= smart_battery_probe,
	.remove		= smart_battery_remove,
};
module_platform_driver(smart_battery_driver);

MODULE_DESCRIPTION("MMI Smart Battery Driver");
MODULE_LICENSE("GPL v2");
