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

#define pr_fmt(fmt)     "GLINK_CHG:BATT: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/power/bm_adsp_ulog.h>

#include "glink_device.h"

#define BATT_DEFAULT_ID 107000

struct glink_batt_data {
	bool cfg_done;
	bool init_done;
	int profile_id;
	const char *batt_sn;
	int state_of_health;
        int manufacturing_date;
        int first_usage_date;
	int charge_current_max;
	int discharge_current_max;
	struct battery_info info;
	struct power_supply *psy;
	struct power_supply_desc *psy_desc;
};

static int find_batt_profile_id(struct glink_dev *dev)
{
	int i;
	int rc;
	int count;
	int profile_id = -EINVAL;
	struct profile_sn_map {
		const char *id;
		const char *sn;
	} *map_table;
	struct glink_batt_data *data = dev->devdata;

	if (!dev->node) {
		pr_err("Invalid node\n");
		return -EINVAL;
	}

	if (!data || !data->batt_sn) {
		pr_err("%s: invalid data or batt_sn\n", dev->name);
		return -EINVAL;
	}

	count = of_property_count_strings(dev->node, "profile-ids-map");
	if (count <= 0 || (count % 2)) {
		pr_err("%s: invalid profile-ids-map, count=%d\n",
					dev->name, count);
		return -EINVAL;
	}

	map_table = devm_kmalloc_array(dev->dev, count / 2,
					sizeof(struct profile_sn_map),
					GFP_KERNEL);
	if (!map_table)
		return -ENOMEM;

	rc = of_property_read_string_array(dev->node, "profile-ids-map",
					(const char **)map_table,
					count);
	if (rc < 0) {
		pr_err("%s: failed to get profile-ids-map, rc=%d\n",
					dev->name, rc);
		profile_id = rc;
		goto free_map;
	}

	for (i = 0; i < count / 2 && map_table[i].sn; i++) {
		pr_info("profile_ids_map[%d]: id=%s, sn=%s\n", i,
					map_table[i].id, map_table[i].sn);
		if (!strcmp(map_table[i].sn, data->batt_sn))
			profile_id = i;
	}

	if (profile_id >= 0 && profile_id < count / 2) {
		i = profile_id;
		profile_id = 0;
		rc = kstrtou32(map_table[i].id, 0, &profile_id);
		if (rc) {
			pr_err("%s: invalid id: %s, sn: %s\n",
						dev->name,
						map_table[i].id,
						map_table[i].sn);
			profile_id = rc;
		} else {
			pr_info("%s: profile id: %s(%d), sn: %s\n",
						dev->name,
						map_table[i].id,
						profile_id,
						map_table[i].sn);
		}
	} else {
		pr_warn("%s: no matched battery profile id\n", dev->name);
	}

free_map:
	devm_kfree(dev->dev, map_table);

	return profile_id;
}

static ssize_t state_of_health_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	struct glink_dev *batt_dev;
	struct glink_batt_data *data;
	struct battery_info info = {0};
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no battery psy available\n");
		return -ENODEV;
	}

	batt_dev = power_supply_get_drvdata(psy);
	if (!batt_dev || !batt_dev->devdata) {
		pr_err("no battery device available\n");
		return -ENODEV;
	}

	data = batt_dev->devdata;
	property = GLINK_PROP_BATT_INFO;
	property |= batt_dev->id << DEVICE_ID_SHIFT;
        rc = batt_dev->ops.get_property(batt_dev,
				property, &info,
				sizeof(struct battery_info));
        if (rc) {
		pr_err("glink device %s read fail, rc=%d\n",
				batt_dev->name, rc);
                return rc;
	}

	data->state_of_health = info.batt_soh;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				data->state_of_health);
}

static DEVICE_ATTR(state_of_health, S_IRUGO,
				state_of_health_show, NULL);

static ssize_t first_usage_date_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct glink_dev *batt_dev;
	struct glink_batt_data *data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no battery psy available\n");
		return -ENODEV;
	}

	batt_dev = power_supply_get_drvdata(psy);
	if (!batt_dev || !batt_dev->devdata) {
		pr_err("no battery device available\n");
		return -ENODEV;
	}

	data = batt_dev->devdata;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				data->first_usage_date);
}

static ssize_t first_usage_date_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned long first_usage_date;
	struct glink_dev *batt_dev;
	struct glink_batt_data *data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no battery psy available\n");
		return -ENODEV;
	}

	batt_dev = power_supply_get_drvdata(psy);
	if (!batt_dev || !batt_dev->devdata) {
		pr_err("no battery device available\n");
		return -ENODEV;
	}

	rc = kstrtoul(buf, 0, &first_usage_date);
	if (rc) {
		pr_err("Invalid first_usage_date value = %lu\n",
				first_usage_date);
		return -EINVAL;
	}

	data = batt_dev->devdata;
	data->first_usage_date = first_usage_date;

	return rc ? rc : count;
}
static DEVICE_ATTR(first_usage_date, 0644,
				first_usage_date_show,
				first_usage_date_store);

static ssize_t manufacturing_date_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct glink_dev *batt_dev;
	struct glink_batt_data *data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no battery psy available\n");
		return -ENODEV;
	}

	batt_dev = power_supply_get_drvdata(psy);
	if (!batt_dev || !batt_dev->devdata) {
		pr_err("no battery device available\n");
		return -ENODEV;
	}

	data = batt_dev->devdata;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				data->manufacturing_date);
}

static ssize_t manufacturing_date_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned long manufacturing_date;
	struct glink_dev *batt_dev;
	struct glink_batt_data *data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no battery psy available\n");
		return -ENODEV;
	}

	batt_dev = power_supply_get_drvdata(psy);
	if (!batt_dev || !batt_dev->devdata) {
		pr_err("no battery device available\n");
		return -ENODEV;
	}

	rc = kstrtoul(buf, 0, &manufacturing_date);
	if (rc) {
		pr_err("Invalid manufacturing_date value = %lu\n",
				manufacturing_date);
		return -EINVAL;
	}

	data = batt_dev->devdata;
	data->manufacturing_date = manufacturing_date;

	return rc ? rc : count;
}
static DEVICE_ATTR(manufacturing_date, 0644,
				manufacturing_date_show,
				manufacturing_date_store);

/* Battery presence detection threshold on battery temperature */
#define BPD_TEMP_THRE -3000
static int get_batt_psy_prop(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *pval)
{
	int rc;
	u32 property;
	struct battery_info info = {0};
	struct glink_batt_data *data;
	struct glink_dev *dev = power_supply_get_drvdata(psy);

	if (!dev || !dev->devdata)
		return -ENODATA;

	pval->intval = -ENODATA;
	data = dev->devdata;
	property = GLINK_PROP_BATT_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
				sizeof(struct battery_info));
	if (rc) {
		pr_err("failed to get batt info for %s\n", dev->name);
		return rc;
	}

	data->info = info;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = info.batt_status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = info.batt_temp > BPD_TEMP_THRE? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pval->intval = info.batt_cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = info.batt_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = info.batt_ua;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = info.batt_soc / 100;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = info.batt_temp / 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		pval->intval = info.batt_full_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		pval->intval = info.batt_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		pval->intval = info.batt_chg_counter;
		break;
	default:
		break;
	}

	return rc;
}

static int set_batt_psy_prop(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *pval)
{
	struct glink_dev *dev = power_supply_get_drvdata(psy);

	switch (prop) {
	default:
		pr_err("%s not supported property: %d\n", dev->name, prop);
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

const char *glink_battery_get_serial_number(struct glink_dev *dev)
{
	int rc = 0;
	const char *df_sn = NULL, *dev_sn = NULL;
	struct device_node *np = of_find_node_by_path("/chosen");

	if (np) {
		const char *utags = NULL;
		rc = of_property_read_string(dev->node,
				"battid-utags", &utags);
		if (rc)
			utags = "mmi,battid";
		rc = of_property_read_string(np, utags, &dev_sn);
		if ((rc == -EINVAL) || !dev_sn) {
			pr_warn("Battsn utag is unused\n");
		} else {
			pr_info("Battsn = %s\n", dev_sn);
		}
		of_node_put(np);
        }

	if (!dev_sn) {
		rc = of_property_read_string(dev->node,
					"df-serialnum",
					&df_sn);
		if (!rc && df_sn) {
			pr_info("Default Serial Number %s\n", df_sn);
		} else {
			pr_err("No Default Serial Number defined\n");
			df_sn = "unknown-sn";
		}
		return df_sn;
	}
	return dev_sn;
}
EXPORT_SYMBOL(glink_battery_get_serial_number);

static void glink_battery_status_update(struct glink_dev *dev)
{
	int rc;
	u32 property;
	struct battery_info info = {0};
	struct glink_batt_data *data;

	if (!dev->devdata)
		return;

	data = dev->devdata;
	property = GLINK_PROP_BATT_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
				sizeof(struct battery_info));
	if (rc) {
		pr_err("failed to get batt info for %s\n", dev->name);
		return;
	}

	data->info = info;
	pr_info("%s: volt:%d, curr:%d, soc:%d, temp:%d, st:%d, soh:%d, "
		"f-mah:%d, d-mah:%d, fv:%d, fcc:%d, chg-cnt:%d, cycle:%d\n",
		dev->name,
		info.batt_uv/1000, info.batt_ua/1000,
		info.batt_soc/100, info.batt_temp/100,
		info.batt_status, info.batt_soh,
		info.batt_full_uah/1000, info.batt_design_uah/1000,
		info.batt_fv_uv/1000, info.batt_fcc_ua/1000,
		info.batt_chg_counter, info.batt_cycle_count);
}

static int glink_battery_cfg(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	u32 property;
	struct glink_batt_data *data = dev->devdata;

	property = GLINK_PROP_BATT_PROFILE_ID;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.set_property(dev,
				property,
				&data->profile_id,
				sizeof(data->profile_id));
	if (rc) {
		pr_err("failed to set profile id = %d", data->profile_id);
		return rc;
	}
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_battery_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *psy_desc = NULL;
	struct glink_batt_data *data = dev->devdata;

	if (data->init_done)
		return 0;

	rc = glink_battery_cfg(dev, cfg);
	if (rc) {
		pr_err("failed to cfg battery %s\n", dev->name);
		return rc;
	}
	data->cfg_done = true;

	if (of_property_read_bool(dev->node, "battery-psy-enabled")) {
		psy_desc = devm_kzalloc(dev->dev,
				sizeof(*psy_desc), GFP_KERNEL);
		if (!psy_desc) {
			return -ENOMEM;
		}
		psy_desc->name			= dev->name;
		psy_desc->type			= POWER_SUPPLY_TYPE_MAINS;
		psy_desc->properties		= batt_props;
		psy_desc->num_properties	= ARRAY_SIZE(batt_props);
		psy_desc->get_property		= get_batt_psy_prop;
		psy_desc->set_property		= set_batt_psy_prop;
	} else {
		pr_info("glink %s init done\n", dev->name);
		data->init_done = true;
		return 0;
	}

	psy_cfg.drv_data = dev;
	psy_cfg.of_node = dev->node;
	data->psy = devm_power_supply_register(dev->dev,
				(const struct power_supply_desc *)psy_desc,
				&psy_cfg);
	if (IS_ERR(data->psy)) {
		rc = PTR_ERR(data->psy);
		data->psy = NULL;
		devm_kfree(dev->dev, psy_desc);
		pr_err("Failed to register %s psy, rc=%d\n", dev->name, rc);
		return rc;
	}
	data->psy_desc = psy_desc;
	data->init_done = true;

	rc = device_create_file(&data->psy->dev,
				&dev_attr_state_of_health);
	if (rc)
		pr_err("Failed to create state_of_health, rc=%d\n", rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_manufacturing_date);
	if (rc)
		pr_err("Failed to create manufacturing_date, rc=%d\n", rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_first_usage_date);
	if (rc)
		pr_err("Failed to create first_usage_date, rc=%d\n", rc);

	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_battery_deinit(struct glink_dev *dev)
{
	struct glink_batt_data *data = dev->devdata;

	device_remove_file(&data->psy->dev, &dev_attr_state_of_health);
	device_remove_file(&data->psy->dev, &dev_attr_manufacturing_date);
	device_remove_file(&data->psy->dev, &dev_attr_first_usage_date);

	if (data && data->psy) {
		power_supply_unregister(data->psy);
		data->psy = NULL;
	}
	if (data && data->psy_desc) {
		devm_kfree(dev->dev, data->psy_desc);
		data->psy_desc = NULL;
	}
	data->init_done = false;

	return 0;
}

static int glink_battery_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct glink_batt_data *data = dev->devdata;

	if (!data) {
		pr_err("glink %s is not ready\n", dev->name);
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (data->init_done)
				glink_battery_cfg(dev, &dev->cfg);
			break;
		case MMI_GLINK_STATE_DOWN:
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_PSY_USR:
		switch (notification) {
		case MMI_PSY_CHANGE_BATT:
			glink_battery_status_update(dev);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		glink_battery_status_update(dev);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_battery_setup(struct glink_dev *dev)
{
	struct glink_batt_data *data = dev->devdata;

	if (data) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	if (dev->id >= GLINK_DEV_ID_BATT_NUM) {
		pr_warn("glink %s has invalid id=%d\n", dev->name, dev->id);
		return -EINVAL;
	}

	data = devm_kzalloc(dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev->devdata = data;
	dev->ops.init = glink_battery_init;
	dev->ops.notify	= glink_battery_notify;
	dev->ops.deinit = glink_battery_deinit;

	data->batt_sn = glink_battery_get_serial_number(dev);
	if (!data->batt_sn)
		data->batt_sn = "unknown-sn";
	data->profile_id = find_batt_profile_id(dev);
	if (data->profile_id < 0)
		data->profile_id = BATT_DEFAULT_ID;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
