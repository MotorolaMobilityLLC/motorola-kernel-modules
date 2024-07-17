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

#define pr_fmt(fmt)     "GLINK_CHG:PUMP: %s: " fmt, __func__

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

struct glink_pump_data {
	bool cfg_done;
	bool init_done;
	struct pump_info info;
	struct power_supply *psy;
	struct power_supply_desc *psy_desc;
};

static ssize_t charge_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 enable;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &enable);
	if (rc) {
		pr_err("Invalid pump enable state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_PUMP_ENABLE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.set_property(pump_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s set enable=%d fail, rc=%d\n",
				pump_dev->name, enable, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t charge_enable_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 enable = 0;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_PUMP_ENABLE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.get_property(pump_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s get enable state fail, rc=%d\n",
				pump_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", enable);
}
static DEVICE_ATTR(charge_enable, 0664,
				charge_enable_show, charge_enable_store);

static ssize_t work_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 mode;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &mode);
	if (rc) {
		pr_err("Invalid pump work mode state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_PUMP_WORK_MODE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.set_property(pump_dev,
				property, &mode,
				sizeof(mode));
        if (rc) {
		pr_err("glink device %s set work_mode=%d fail, rc=%d\n",
				pump_dev->name, mode, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t work_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 mode = 0;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_PUMP_WORK_MODE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.get_property(pump_dev,
				property, &mode,
				sizeof(mode));
        if (rc) {
		pr_err("glink device %s get work_mode fail, rc=%d\n",
				pump_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", mode);
}
static DEVICE_ATTR(work_mode, 0664,
				work_mode_show, work_mode_store);

static ssize_t ovp_gate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 gate;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &gate);
	if (rc) {
		pr_err("Invalid pump work mode state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_PUMP_OVP_GATE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.set_property(pump_dev,
				property, &gate,
				sizeof(gate));
        if (rc) {
		pr_err("glink device %s set ovp_gate=%d fail, rc=%d\n",
				pump_dev->name, gate, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t ovp_gate_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 gate = 0;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_PUMP_OVP_GATE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.get_property(pump_dev,
				property, &gate,
				sizeof(gate));
        if (rc) {
		pr_err("glink device %s get ovp_gate fail, rc=%d\n",
				pump_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", gate);
}
static DEVICE_ATTR(ovp_gate, 0664,
				ovp_gate_show, ovp_gate_store);

static ssize_t manual_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 manual;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &manual);
	if (rc) {
		pr_err("Invalid pump work mode state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_PUMP_MANUAL_MODE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.set_property(pump_dev,
				property, &manual,
				sizeof(manual));
        if (rc) {
		pr_err("glink device %s set manual_mode=%d fail, rc=%d\n",
				pump_dev->name, manual, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t manual_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 manual = 0;
	u32 property;
	struct glink_dev *pump_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available pump psy\n");
		return -ENODEV;
	}

	pump_dev = power_supply_get_drvdata(psy);
	if (!pump_dev) {
		pr_err("no available pump device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_PUMP_MANUAL_MODE;
	property |= pump_dev->id << DEVICE_ID_SHIFT;
        rc = pump_dev->ops.get_property(pump_dev,
				property, &manual,
				sizeof(manual));
        if (rc) {
		pr_err("glink device %s get manual_mode fail, rc=%d\n",
				pump_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", manual);
}
static DEVICE_ATTR(manual_mode, 0664,
				manual_mode_show, manual_mode_store);

static int get_pump_psy_prop(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *pval)
{
	int rc;
	u32 property;
	struct pump_info info = {0};
	struct glink_pump_data *data;
	struct glink_dev *dev = power_supply_get_drvdata(psy);

	if (!dev || !dev->devdata)
		return -ENODATA;

	pval->intval = -ENODATA;
	data = dev->devdata;
	property = GLINK_PROP_CHG_PUMP_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
				sizeof(struct pump_info));
	if (rc) {
		pr_err("failed to get pump info for %s\n", dev->name);
		return rc;
	}

	data->info = info;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (data->info.chg_en)
			pval->intval = 1;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (data->info.chg_en)
			pval->intval = data->info.vbus_mv * 1000;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (data->info.chg_en)
			pval->intval = data->info.ibus_ma * 1000;
		else
			pval->intval = 0;
		break;
	default:
		break;
	}

	return rc;
}

static int set_pump_psy_prop(struct power_supply *psy,
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

static enum power_supply_property pump_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int glink_pump_cfg(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_pump_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *psy_desc = NULL;
	struct glink_pump_data *data = dev->devdata;

	if (data->init_done)
		return 0;

	rc = glink_pump_cfg(dev, cfg);
	if (rc) {
		pr_err("failed to cfg cp %s\n", dev->name);
		return rc;
	}
	data->cfg_done = true;

	if (of_property_read_bool(dev->node, "pump-psy-enabled")) {
		psy_desc = devm_kzalloc(dev->dev,
				sizeof(*psy_desc), GFP_KERNEL);
		if (!psy_desc) {
			return -ENOMEM;
		}
		psy_desc->name			= dev->name;
		psy_desc->type			= POWER_SUPPLY_TYPE_UNKNOWN;
		psy_desc->properties		= pump_props;
		psy_desc->num_properties	= ARRAY_SIZE(pump_props);
		psy_desc->get_property		= get_pump_psy_prop;
		psy_desc->set_property		= set_pump_psy_prop;
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
				&dev_attr_charge_enable);
	if (rc)
		pr_err("%s: Failed to create charge_enable, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_work_mode);
	if (rc)
		pr_err("%s: Failed to create work_mode, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_ovp_gate);
	if (rc)
		pr_err("%s: Failed to create ovp_gate, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_manual_mode);
	if (rc)
		pr_err("%s: Failed to create manual_mode, rc=%d\n",
				dev->name, rc);

	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_pump_deinit(struct glink_dev *dev)
{
	struct glink_pump_data *data = dev->devdata;

	if (data && data->psy) {
		device_remove_file(&data->psy->dev, &dev_attr_manual_mode);
		device_remove_file(&data->psy->dev, &dev_attr_ovp_gate);
		device_remove_file(&data->psy->dev, &dev_attr_work_mode);
		device_remove_file(&data->psy->dev, &dev_attr_charge_enable);
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

static void glink_pump_status_update(struct glink_dev *dev)
{
	int rc;
	u32 property;
	struct pump_info info = {0};
	struct glink_pump_data *data;

	if (!dev->devdata)
		return;

	data = dev->devdata;
	property = GLINK_PROP_CHG_PUMP_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
			sizeof(struct pump_info));
	if (rc) {
		pr_err("failed to get pump info for %s\n", dev->name);
		return;
	}
	data->info = info;

	pr_info("%s: chip_id:0x%x, enabled:%d, ovpgate:%d, manual:%d, "
		"otg:%d, work_mode:%d, interrupt:%d, "
		"ibus:%d, vbus:%d, vout:%d, vac:%d, vusb:%d, "
		"vwpc:%d, ibat:%d, vbat:%d, die_temp:%d\n",
		dev->name, info.chip_id,
		info.chg_en, info.ovpgate, info.manual,
		info.otg_en, info.work_mode, info.int_stat,
		info.ibus_ma, info.vbus_mv, info.vout_mv, info.vac_mv,
		info.vusb_mv, info.vwpc_mv, info.ibat_ma, info.vbat_mv,
		info.die_temp);
}

static int glink_pump_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct glink_pump_data *data = dev->devdata;

	if (!data) {
		pr_err("glink %s is not ready\n", dev->name);
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (data->init_done)
				glink_pump_cfg(dev, &dev->cfg);
			break;
		case MMI_GLINK_STATE_DOWN:
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_PSY_USR:
		switch (notification) {
		case MMI_PSY_CHANGE_USB:
		case MMI_PSY_CHANGE_WLS:
			glink_pump_status_update(dev);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		glink_pump_status_update(dev);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_pump_setup(struct glink_dev *dev)
{
	struct glink_pump_data *data = dev->devdata;

	if (data) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	if (dev->id >= GLINK_DEV_ID_PUMP_NUM) {
		pr_warn("glink %s has invalid id=%d\n", dev->name, dev->id);
		return -EINVAL;
	}

	data = devm_kzalloc(dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev->devdata = data;
	dev->ops.init = glink_pump_init;
	dev->ops.notify	= glink_pump_notify;
	dev->ops.deinit = glink_pump_deinit;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
