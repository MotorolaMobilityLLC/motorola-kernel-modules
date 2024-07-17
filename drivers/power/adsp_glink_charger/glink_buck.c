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

#define pr_fmt(fmt)     "GLINK_CHG:BUCK: %s: " fmt, __func__

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

struct glink_buck_data {
	bool cfg_done;
	bool init_done;
	struct buck_info info;
	struct power_supply *psy;
	struct power_supply_desc *psy_desc;
};

static ssize_t chip_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 enable;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &enable);
	if (rc) {
		pr_err("Invalid buck chip enable state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_ENABLE;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s set chip enable=%d fail, rc=%d\n",
				buck_dev->name, enable, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t chip_enable_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 enable = 0;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_ENABLE;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s get chip enable state fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", enable);
}
static DEVICE_ATTR(chip_enable, 0664,
				chip_enable_show, chip_enable_store);

static ssize_t charge_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 enable;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &enable);
	if (rc) {
		pr_err("Invalid buck enable state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_ENABLE_CHG;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s set enable=%d fail, rc=%d\n",
				buck_dev->name, enable, rc);
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
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_ENABLE_CHG;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s get enable state fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", enable);
}
static DEVICE_ATTR(charge_enable, 0664,
				charge_enable_show, charge_enable_store);

static ssize_t input_suspend_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 suspend;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &suspend);
	if (rc) {
		pr_err("Invalid buck input suspend state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_SUSPEND;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &suspend,
				sizeof(suspend));
        if (rc) {
		pr_err("glink device %s set input suspend=%d fail, rc=%d\n",
				buck_dev->name, suspend, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t input_suspend_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 suspend = 0;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_SUSPEND;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &suspend,
				sizeof(suspend));
        if (rc) {
		pr_err("glink device %s get input_suspend fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", suspend);
}
static DEVICE_ATTR(input_suspend, 0664,
				input_suspend_show, input_suspend_store);

static ssize_t icl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 icl;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &icl);
	if (rc) {
		pr_err("Invalid buck icl state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_ICL;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &icl,
				sizeof(icl));
        if (rc) {
		pr_err("glink device %s set icl=%d fail, rc=%d\n",
				buck_dev->name, icl, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t icl_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 icl = 0;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_ICL;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &icl,
				sizeof(icl));
        if (rc) {
		pr_err("glink device %s get icl fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", icl);
}
static DEVICE_ATTR(icl, 0664, icl_show, icl_store);

static ssize_t fcc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 fcc;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &fcc);
	if (rc) {
		pr_err("Invalid buck fcc state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_FCC;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &fcc,
				sizeof(fcc));
        if (rc) {
		pr_err("glink device %s set fcc=%d fail, rc=%d\n",
				buck_dev->name, fcc, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t fcc_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 fcc = 0;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_FCC;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &fcc,
				sizeof(fcc));
        if (rc) {
		pr_err("glink device %s get fcc fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", fcc);
}
static DEVICE_ATTR(fcc, 0664, fcc_show, fcc_store);

static ssize_t fv_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 fv;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &fv);
	if (rc) {
		pr_err("Invalid buck fv state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BUCK_FV;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.set_property(buck_dev,
				property, &fv,
				sizeof(fv));
        if (rc) {
		pr_err("glink device %s set fv=%d fail, rc=%d\n",
				buck_dev->name, fv, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t fv_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 fv = 0;
	u32 property;
	struct glink_dev *buck_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available buck psy\n");
		return -ENODEV;
	}

	buck_dev = power_supply_get_drvdata(psy);
	if (!buck_dev) {
		pr_err("no available buck device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BUCK_FV;
	property |= buck_dev->id << DEVICE_ID_SHIFT;
        rc = buck_dev->ops.get_property(buck_dev,
				property, &fv,
				sizeof(fv));
        if (rc) {
		pr_err("glink device %s get fv fail, rc=%d\n",
				buck_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", fv);
}
static DEVICE_ATTR(fv, 0664, fv_show, fv_store);

static int get_buck_psy_prop(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *pval)
{
	int rc;
	u32 property;
	struct buck_info info = {0};
	struct glink_buck_data *data;
	struct glink_dev *dev = power_supply_get_drvdata(psy);

	if (!dev || !dev->devdata)
		return -ENODATA;

	pval->intval = -ENODATA;
	data = dev->devdata;
	property = GLINK_PROP_CHG_BUCK_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
				sizeof(struct buck_info));
	if (rc) {
		pr_err("failed to get buck info for %s\n", dev->name);
		return rc;
	}

	data->info = info;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		if (info.input_ready)
			pval->intval = info.input_vlimit * 1000;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (!info.input_suspend)
			pval->intval = info.input_ilimit * 1000;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (info.chg_en)
			pval->intval = info.batt_fv * 1000;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (info.chg_en)
			pval->intval = info.batt_fcc * 1000;
		else
			pval->intval = 0;
		break;
	default:
		break;
	}

	return rc;
}

static int set_buck_psy_prop(struct power_supply *psy,
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

static enum power_supply_property buck_props[] = {
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static int glink_buck_cfg(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_buck_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *psy_desc = NULL;
	struct glink_buck_data *data = dev->devdata;

	if (data->init_done)
		return 0;

	rc = glink_buck_cfg(dev, cfg);
	if (rc) {
		pr_err("failed to cfg cp %s\n", dev->name);
		return rc;
	}
	data->cfg_done = true;

	if (of_property_read_bool(dev->node, "buck-psy-enabled")) {
		psy_desc = devm_kzalloc(dev->dev,
				sizeof(*psy_desc), GFP_KERNEL);
		if (!psy_desc) {
			return -ENOMEM;
		}
		psy_desc->name			= dev->name;
		psy_desc->type			= POWER_SUPPLY_TYPE_UNKNOWN;
		psy_desc->properties		= buck_props;
		psy_desc->num_properties	= ARRAY_SIZE(buck_props);
		psy_desc->get_property		= get_buck_psy_prop;
		psy_desc->set_property		= set_buck_psy_prop;
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
				&dev_attr_chip_enable);
	if (rc)
		pr_err("%s: Failed to create chip_enable, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_charge_enable);
	if (rc)
		pr_err("%s: Failed to create charge_enable, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_input_suspend);
	if (rc)
		pr_err("%s: Failed to create input suspend, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_icl);
	if (rc)
		pr_err("%s: Failed to create icl, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_fcc);
	if (rc)
		pr_err("%s: Failed to create fcc, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_fv);
	if (rc)
		pr_err("%s: Failed to create fv, rc=%d\n",
				dev->name, rc);

	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_buck_deinit(struct glink_dev *dev)
{
	struct glink_buck_data *data = dev->devdata;

	if (data && data->psy) {
		device_remove_file(&data->psy->dev, &dev_attr_fv);
		device_remove_file(&data->psy->dev, &dev_attr_fcc);
		device_remove_file(&data->psy->dev, &dev_attr_icl);
		device_remove_file(&data->psy->dev, &dev_attr_input_suspend);
		device_remove_file(&data->psy->dev, &dev_attr_charge_enable);
		device_remove_file(&data->psy->dev, &dev_attr_chip_enable);
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

static void glink_buck_status_update(struct glink_dev *dev)
{
	int rc;
	u32 property;
	struct buck_info info = {0};
	struct glink_buck_data *data;

	if (!dev->devdata)
		return;

	data = dev->devdata;
	property = GLINK_PROP_CHG_BUCK_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
			sizeof(struct buck_info));
	if (rc) {
		pr_err("failed to get buck info for %s\n", dev->name);
		return;
	}
	data->info = info;

	pr_info("%s: chip_en:%d, chg_en:%d, input_ready:%d, input_suspend:%d, "
		"input_ilimit:%d, input_vlimit:%d, fcc:%d, fv:%d, "
		"chg_ctrl:0x%08x, chg_stat:0x%08x\n",
		dev->name,
		info.chip_en, info.chg_en, info.input_ready,
		info.input_suspend, info.input_ilimit, info.input_vlimit,
		info.batt_fcc, info.batt_fv,
		info.chg_ctrl, info.chg_stat);
}

static int glink_buck_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct glink_buck_data *data = dev->devdata;

	if (!data) {
		pr_err("glink %s is not ready\n", dev->name);
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (data->init_done)
				glink_buck_cfg(dev, &dev->cfg);
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
			glink_buck_status_update(dev);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		glink_buck_status_update(dev);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_buck_setup(struct glink_dev *dev)
{
	struct glink_buck_data *data = dev->devdata;

	if (data) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	if (dev->id >= GLINK_DEV_ID_BUCK_NUM) {
		pr_warn("glink %s has invalid id=%d\n", dev->name, dev->id);
		return -EINVAL;
	}

	data = devm_kzalloc(dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev->devdata = data;
	dev->ops.init = glink_buck_init;
	dev->ops.notify	= glink_buck_notify;
	dev->ops.deinit = glink_buck_deinit;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
