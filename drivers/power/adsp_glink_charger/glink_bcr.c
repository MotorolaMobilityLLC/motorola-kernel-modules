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

#define pr_fmt(fmt)     "GLINK_CHG:BCR: %s: " fmt, __func__

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

struct glink_bcr_data {
	bool cfg_done;
	bool init_done;
	struct bcr_info info;
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
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &enable);
	if (rc) {
		pr_err("Invalid bcr enable state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BCR_ENABLE;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.set_property(bcr_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s set enable=%d fail, rc=%d\n",
				bcr_dev->name, enable, rc);
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
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BCR_ENABLE;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.get_property(bcr_dev,
				property, &enable,
				sizeof(enable));
        if (rc) {
		pr_err("glink device %s get enable state fail, rc=%d\n",
				bcr_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", enable);
}
static DEVICE_ATTR(charge_enable, 0664,
				charge_enable_show, charge_enable_store);

static ssize_t auto_bsm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 bsm;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &bsm);
	if (rc) {
		pr_err("Invalid bcr auto bsm state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BCR_AUTO_BSM;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.set_property(bcr_dev,
				property, &bsm,
				sizeof(bsm));
        if (rc) {
		pr_err("glink device %s set auto_bsm=%d fail, rc=%d\n",
				bcr_dev->name, bsm, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t auto_bsm_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 bsm = 0;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BCR_AUTO_BSM;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.get_property(bcr_dev,
				property, &bsm,
				sizeof(bsm));
        if (rc) {
		pr_err("glink device %s get auto_bsm fail, rc=%d\n",
				bcr_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", bsm);
}
static DEVICE_ATTR(auto_bsm, 0664,
				auto_bsm_show, auto_bsm_store);

static ssize_t ext_sw_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 sw;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &sw);
	if (rc) {
		pr_err("Invalid bcr ext sw state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BCR_EXT_SW_ENABLE;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.set_property(bcr_dev,
				property, &sw,
				sizeof(sw));
        if (rc) {
		pr_err("glink device %s set ext_sw_en=%d fail, rc=%d\n",
				bcr_dev->name, sw, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t ext_sw_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 sw = 0;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BCR_EXT_SW_ENABLE;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.get_property(bcr_dev,
				property, &sw,
				sizeof(sw));
        if (rc) {
		pr_err("glink device %s get ext_sw_en fail, rc=%d\n",
				bcr_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", sw);
}
static DEVICE_ATTR(ext_sw_en, 0664,
				ext_sw_en_show, ext_sw_en_store);

static ssize_t chg_imax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 imax;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &imax);
	if (rc) {
		pr_err("Invalid bcr chg_imax, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BCR_CHG_CURR_MAX;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.set_property(bcr_dev,
				property, &imax,
				sizeof(imax));
        if (rc) {
		pr_err("glink device %s set chg_imax=%d fail, rc=%d\n",
				bcr_dev->name, imax, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t chg_imax_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 imax = 0;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BCR_CHG_CURR_MAX;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.get_property(bcr_dev,
				property, &imax,
				sizeof(imax));
        if (rc) {
		pr_err("glink device %s get chg_imax fail, rc=%d\n",
				bcr_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", imax);
}
static DEVICE_ATTR(chg_imax, 0664,
				chg_imax_show, chg_imax_store);

static ssize_t dischg_imax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 imax;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &imax);
	if (rc) {
		pr_err("Invalid bcr dischg_imax state, rc=%d\n", rc);
		return -EINVAL;
	}

	property = GLINK_PROP_CHG_BCR_DISCHG_CURR_MAX;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.set_property(bcr_dev,
				property, &imax,
				sizeof(imax));
        if (rc) {
		pr_err("glink device %s set dischg_imax=%d fail, rc=%d\n",
				bcr_dev->name, imax, rc);
                return rc;
	}

	return rc ? rc : count;
}

static ssize_t dischg_imax_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 imax = 0;
	u32 property;
	struct glink_dev *bcr_dev;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!psy) {
		pr_err("no available bcr psy\n");
		return -ENODEV;
	}

	bcr_dev = power_supply_get_drvdata(psy);
	if (!bcr_dev) {
		pr_err("no available bcr device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_CHG_BCR_DISCHG_CURR_MAX;
	property |= bcr_dev->id << DEVICE_ID_SHIFT;
        rc = bcr_dev->ops.get_property(bcr_dev,
				property, &imax,
				sizeof(imax));
        if (rc) {
		pr_err("glink device %s get dischg_imax fail, rc=%d\n",
				bcr_dev->name, rc);
                return rc;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", imax);
}
static DEVICE_ATTR(dischg_imax, 0664,
				dischg_imax_show, dischg_imax_store);

static int get_bcr_psy_prop(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *pval)
{
	int rc;
	u32 property;
	struct bcr_info info = {0};
	struct glink_bcr_data *data;
	struct glink_dev *dev = power_supply_get_drvdata(psy);

	if (!dev || !dev->devdata)
		return -ENODATA;

	pval->intval = -ENODATA;
	data = dev->devdata;
	property = GLINK_PROP_CHG_BCR_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
				sizeof(struct bcr_info));
	if (rc) {
		pr_err("failed to get bcr info for %s\n", dev->name);
		return rc;
	}

	data->info = info;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = info.die_temp * 10;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = info.vbat_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = info.ibat_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		pval->intval = info.vchg_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (info.chg_en)
			pval->intval = info.chg_ilimit * 1000;
		else
			pval->intval = 0;
		break;
	default:
		break;
	}

	return rc;
}

static int set_bcr_psy_prop(struct power_supply *psy,
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

static enum power_supply_property bcr_props[] = {
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static int glink_bcr_cfg(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_bcr_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *psy_desc = NULL;
	struct glink_bcr_data *data = dev->devdata;

	if (data->init_done)
		return 0;

	rc = glink_bcr_cfg(dev, cfg);
	if (rc) {
		pr_err("failed to cfg cp %s\n", dev->name);
		return rc;
	}
	data->cfg_done = true;

	if (of_property_read_bool(dev->node, "bcr-psy-enabled")) {
		psy_desc = devm_kzalloc(dev->dev,
				sizeof(*psy_desc), GFP_KERNEL);
		if (!psy_desc) {
			return -ENOMEM;
		}
		psy_desc->name			= dev->name;
		psy_desc->type			= POWER_SUPPLY_TYPE_UNKNOWN;
		psy_desc->properties		= bcr_props;
		psy_desc->num_properties	= ARRAY_SIZE(bcr_props);
		psy_desc->get_property		= get_bcr_psy_prop;
		psy_desc->set_property		= set_bcr_psy_prop;
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
				&dev_attr_auto_bsm);
	if (rc)
		pr_err("%s: Failed to create auto_bsm, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_ext_sw_en);
	if (rc)
		pr_err("%s: Failed to create ext_sw_en, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_chg_imax);
	if (rc)
		pr_err("%s: Failed to create chg_imax, rc=%d\n",
				dev->name, rc);

	rc = device_create_file(&data->psy->dev,
				&dev_attr_dischg_imax);
	if (rc)
		pr_err("%s: Failed to create dischg_imax, rc=%d\n",
				dev->name, rc);

	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_bcr_deinit(struct glink_dev *dev)
{
	struct glink_bcr_data *data = dev->devdata;

	if (data && data->psy) {
		device_remove_file(&data->psy->dev, &dev_attr_dischg_imax);
		device_remove_file(&data->psy->dev, &dev_attr_chg_imax);
		device_remove_file(&data->psy->dev, &dev_attr_ext_sw_en);
		device_remove_file(&data->psy->dev, &dev_attr_auto_bsm);
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

static void glink_bcr_status_update(struct glink_dev *dev)
{
	int rc;
	u32 property;
	struct bcr_info info = {0};
	struct glink_bcr_data *data;

	if (!dev->devdata)
		return;

	data = dev->devdata;
	property = GLINK_PROP_CHG_BCR_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &info,
			sizeof(struct bcr_info));
	if (rc) {
		pr_err("failed to get bcr info for %s\n", dev->name);
		return;
	}
	data->info = info;

	pr_info("%s: chg_en:%d, ext_sw_st:%d, work_mode:%d, "
		"ibat_ma:%d, vbat_mv:%d, vchg_mv:%d, "
		"chg_ilimit:%d, dischg_ilimit:%d, "
		"batt_temp:%d, die_temp:%d, "
		"chg_ctrl:0x%08x, chg_stat:0x%08x\n",
		dev->name,
		info.chg_en, info.ext_sw_st, info.work_mode,
		info.ibat_ma, info.vbat_mv, info.vchg_mv,
		info.chg_ilimit, info.dischg_ilimit,
		info.batt_temp, info.die_temp,
		info.chg_ctrl, info.chg_stat);
}

static int glink_bcr_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct glink_bcr_data *data = dev->devdata;

	if (!data) {
		pr_err("glink %s is not ready\n", dev->name);
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (data->init_done)
				glink_bcr_cfg(dev, &dev->cfg);
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
			glink_bcr_status_update(dev);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		glink_bcr_status_update(dev);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_bcr_setup(struct glink_dev *dev)
{
	struct glink_bcr_data *data = dev->devdata;

	if (data) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	if (dev->id >= GLINK_DEV_ID_BCR_NUM) {
		pr_warn("glink %s has invalid id=%d\n", dev->name, dev->id);
		return -EINVAL;
	}

	data = devm_kzalloc(dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev->devdata = data;
	dev->ops.init = glink_bcr_init;
	dev->ops.notify	= glink_bcr_notify;
	dev->ops.deinit = glink_bcr_deinit;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
