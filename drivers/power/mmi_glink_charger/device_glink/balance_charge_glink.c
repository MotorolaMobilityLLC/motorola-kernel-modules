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
 #include <linux/power_supply.h>
#include <linux/of.h>
#include "mmi_glink_core.h"
#include "qti_glink_charger_v2.h"
#include "device_class.h"
#include "balance_charge_glink.h"

static struct mmi_glink_chip *this_root_chip =  NULL;
static struct balance_glink_dev *this_balance_chip[DEV_NUM] = {NULL};

static ssize_t balance_chip_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 chip_en = 0, property = 0;

	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_CHIP_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_CHIP_EN;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&chip_en,
				sizeof(chip_en));

	mmi_dbg(this_root_chip, "Get balance_chip_en[%d]: %d", dev_role, chip_en);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chip_en);
}

static ssize_t balance_chip_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long chip_en;

	int dev_role = 0;
	u32 property = 0;


	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chip_en);
	if (r) {
		pr_err("Invalid Balance chip_en = %lu\n", chip_en);
		return -EINVAL;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_CHIP_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_CHIP_EN;
	else
		return -EINVAL;


	r = qti_charger_set_property(property,
				&chip_en,
				sizeof(chip_en));
	mmi_info(this_root_chip, "Set balance_chip_en[%d]: %ld", dev_role, chip_en);
	return r ? r : count;
}
static DEVICE_ATTR(balance_chip_en, S_IRUGO|S_IWUSR, balance_chip_en_show, balance_chip_en_store);

static ssize_t balance_chrg_dis_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 chrg_en = 0, property = 0;

	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_CHRG_DIS;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_CHRG_DIS;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&chrg_en,
				sizeof(chrg_en));

	mmi_dbg(this_root_chip, "Get balance_chrg_dis[%d]: %d", dev_role, chrg_en);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chrg_en);
}

static ssize_t balance_chrg_dis_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long chrg_en;

	int dev_role = 0;
	u32 property = 0;

	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chrg_en);
	if (r) {
		pr_err("Invalid Balance chrg_en = %lu\n", chrg_en);
		return -EINVAL;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_CHRG_DIS;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_CHRG_DIS;
	else
		return -EINVAL;

	r = qti_charger_set_property(property,
				&chrg_en,
				sizeof(chrg_en));

	mmi_info(this_root_chip, "Set balance_chrg_dis[%d]: %ld", dev_role, chrg_en);
	return r ? r : count;
}
static DEVICE_ATTR(balance_chrg_dis, S_IRUGO|S_IWUSR, balance_chrg_dis_show, balance_chrg_dis_store);

static ssize_t balance_extmos_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 extmos_en = 0, property = 0;

	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_EXTMOS_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_EXTMOS_EN;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&extmos_en,
				sizeof(extmos_en));

	mmi_dbg(this_root_chip, "Get balance_extmos_en[%d]: %d", dev_role, extmos_en);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", extmos_en);
}

static ssize_t balance_extmos_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long extmos_en;

	int dev_role = 0;
	u32 property = 0;

	if (!balance_chip) {
		pr_err("Balance: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &extmos_en);
	if (r) {
		pr_err("Invalid Balance extmos en = %lu\n", extmos_en);
		return -EINVAL;
	}

	dev_role = balance_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MBC_MASTER_EXTMOS_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MBC_SLAVE_EXTMOS_EN;
	else
		return -EINVAL;

	r = qti_charger_set_property(property,
				&extmos_en,
				sizeof(extmos_en));

	mmi_info(this_root_chip, "Set balance_extmos_en[%d]: %ld", dev_role, extmos_en);
	return r ? r : count;
}
static DEVICE_ATTR(balance_extmos_en, S_IRUGO|S_IWUSR, balance_extmos_en_show, balance_extmos_en_store);

static enum power_supply_property balance_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

#define BPD_TEMP_THRE -3000
static int balance_psy_get_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 union power_supply_propval *pval)
{
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);
	struct timespec64 glink_access_time_now;
	int rc = 0;

	pval->intval = -ENODATA;

	switch (balance_chip->dev_role) {
	case DEV_MASTER:
		ktime_get_real_ts64(&glink_access_time_now);
		balance_chip->elapsed_ms = (glink_access_time_now.tv_sec - balance_chip->glink_access_time.tv_sec) * 1000;
		balance_chip->elapsed_ms += (glink_access_time_now.tv_nsec - balance_chip->glink_access_time.tv_nsec) / 1000000;
		if (balance_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&balance_chip->glink_access_time);
			balance_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_MBC_MASTER_DEV_INFO,
				&balance_chip->balance_dev_info, sizeof(struct balance_dev_info));
			mmi_dbg(this_root_chip, "balance_get_prop[%d], DEV_INFO", balance_chip->dev_role);
		}
		break;
	case DEV_SLAVE:
		ktime_get_real_ts64(&glink_access_time_now);
		balance_chip->elapsed_ms = (glink_access_time_now.tv_sec - balance_chip->glink_access_time.tv_sec) * 1000;
		balance_chip->elapsed_ms += (glink_access_time_now.tv_nsec - balance_chip->glink_access_time.tv_nsec) / 1000000;
		if (balance_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&balance_chip->glink_access_time);
			balance_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_MBC_SLAVE_DEV_INFO,
				&balance_chip->balance_dev_info, sizeof(struct balance_dev_info));
			mmi_dbg(this_root_chip, "balance_get_prop[%d], DEV_INFO", balance_chip->dev_role);
		}
		break;
	default:
		mmi_err(this_root_chip, "balance_get_prop, Can not find correct balance role %d", balance_chip->dev_role);
		return rc;
		break;
	}

	if (rc) {
		mmi_err(this_root_chip, "balance_role %d, Get BALANCE_INFO property failed", balance_chip->dev_role);
		return rc;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = balance_chip->balance_dev_info.work_mode;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = balance_chip->balance_dev_info.ls_off;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = balance_chip->balance_dev_info.vbat_mv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = balance_chip->balance_dev_info.vchg_mv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = balance_chip->balance_dev_info.ibat_ma;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		pval->intval = balance_chip->balance_dev_info.ibat_limit;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = balance_chip->balance_dev_info.die_temp;
		break;
	default:
		break;
	}

	return rc;
}

static int balance_psy_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	int rc = 0, prop_cmd = 0, value = 0;
	struct balance_glink_dev *balance_chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (balance_chip->dev_role == DEV_MASTER) {
			prop_cmd = OEM_PROP_MBC_MASTER_CURR_MAX;
			mmi_info(this_root_chip, "Set DEV_MASTER, property %d", prop);
		} else if (balance_chip->dev_role == DEV_SLAVE) {
			prop_cmd = OEM_PROP_MBC_SLAVE_CURR_MAX;
			mmi_info(this_root_chip, "Set DEV_SLAVE, property %d", prop);
		}
		value = val->intval;
		qti_charger_set_property(prop_cmd,
				&value,
				sizeof(value));
		break;
	default:
		mmi_err(this_root_chip, "Not supported property: %d\n", prop);
		return -EINVAL;
		break;
	}
	return rc;
}

static int balance_psy_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc balance_psy_desc_master = {
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= balance_psy_get_prop,
	.set_property	= balance_psy_set_prop,
	.property_is_writeable = balance_psy_prop_is_writeable,
	.properties	= balance_psy_props,
	.num_properties	= ARRAY_SIZE(balance_psy_props),
};

static struct power_supply_desc balance_psy_desc_slave = {
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= balance_psy_get_prop,
	.set_property	= balance_psy_set_prop,
	.property_is_writeable = balance_psy_prop_is_writeable,
	.properties	= balance_psy_props,
	.num_properties	= ARRAY_SIZE(balance_psy_props),
};

static int balance_notify_handler(struct notifier_block *nb, unsigned long event, void *data)
{
	int rc = -1;
	struct balance_dev_info balance_info;
	struct balance_glink_dev *balance_chip =
			container_of(nb, struct balance_glink_dev, balance_nb);
//	struct struct mmi_glink_chip *chip = data;

	mmi_dbg(this_root_chip, "balance_role %d, notify-dev %ld", balance_chip->dev_role, event);
	if (event == DEV_BALANCE_CHG|| event == DEV_ALL) {

		if (balance_chip->dev_role == DEV_MASTER) {
			rc = qti_charger_get_property(OEM_PROP_MBC_MASTER_DEV_INFO,
					&balance_info, sizeof(balance_info));
		} else if (balance_chip->dev_role == DEV_SLAVE) {
			rc = qti_charger_get_property(OEM_PROP_MBC_SLAVE_DEV_INFO,
					&balance_info, sizeof(balance_info));
		} else {
			mmi_err(this_root_chip, "balance_get_prop, Can not find correct balance role %d", balance_chip->dev_role);
			return rc;
		}
		mmi_info(this_root_chip, "Balance_dev[%d]: work_mode %d, ibat_ma %d, ibat_limit %d, "
							"vchg_mv %d, vbat_mv %d, die_temp %d, ls_off %d, auto_bsm_dis %d, extmos_en %d, lpm_mode %d",
							balance_chip->dev_role, balance_info.work_mode, balance_info.ibat_ma, balance_info.ibat_limit,
							balance_info.vchg_mv, balance_info.vbat_mv, balance_info.die_temp, balance_info.ls_off, balance_info.auto_bsm_dis,
							balance_info.extmos_en, balance_info.lpm_mode);
	}

	return NOTIFY_DONE;
}

struct glink_device *balance_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{

	struct balance_glink_dev *balance_chip = NULL;
	struct glink_device * glink_dev = NULL;
	struct power_supply_config psy_cfg = {};
	int rc = 0;

	if (!chip)
		goto exit;

	balance_chip = kzalloc(sizeof(struct balance_glink_dev),GFP_KERNEL);
	if (!balance_chip)
		goto exit;

	balance_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	balance_chip->dev_role = dev_dts->dev_role;
	psy_cfg.drv_data = balance_chip;

	if (balance_chip->dev_role == DEV_MASTER) {
		mmi_err(chip, "balance[%d] glink device psy", balance_chip->dev_role);
		balance_psy_desc_master.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		balance_chip->balance_dev_psy = devm_power_supply_register(chip->dev,
							  &balance_psy_desc_master,
							  &psy_cfg);
		if (IS_ERR(balance_chip->balance_dev_psy)) {
			mmi_err(chip, "Couldn't register balance dev power supply");
			goto exit;
		}
	} else if (balance_chip->dev_role == DEV_SLAVE) {
		mmi_err(chip, "balance[%d] glink device psy", balance_chip->dev_role);
		balance_psy_desc_slave.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		balance_chip->balance_dev_psy = devm_power_supply_register(chip->dev,
							  &balance_psy_desc_slave,
							  &psy_cfg);
		if (IS_ERR(balance_chip->balance_dev_psy)) {
			mmi_err(chip, "Couldn't register balance dev power supply");
			goto exit;
		}
	} else {
		mmi_err(chip, "Couldn't register balance role %d, Failed register balance glink", balance_chip->dev_role);
		goto exit;
	}

	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_BALANCE_CHG, balance_chip);
	if (!glink_dev)
		goto exit;

	balance_chip->glink_dev = glink_dev;
	balance_chip->balance_nb.notifier_call = balance_notify_handler;
	mmi_glink_register_notifier(&balance_chip->balance_nb);


	rc = device_create_file(&balance_chip->balance_dev_psy->dev,
				&dev_attr_balance_chip_en);
        if (rc)
		pr_err("couldn't create balance chip en\n");

	rc = device_create_file(&balance_chip->balance_dev_psy->dev,
				&dev_attr_balance_chrg_dis);
        if (rc)
		pr_err("couldn't create balance chrg en\n");

	rc = device_create_file(&balance_chip->balance_dev_psy->dev,
				&dev_attr_balance_extmos_en);
        if (rc)
		pr_err("couldn't create balance extmos en\n");

	this_root_chip = chip;
	this_balance_chip[balance_chip->dev_role] = balance_chip;
	mmi_err(chip, "balance glink device %s register successfully", dev_dts->glink_dev_name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void balance_glink_device_unregister(void)
{
	int i = 0;
	for (i = 0; i < DEV_NUM; i++) {
		if (!this_balance_chip[i])
			return;
		device_remove_file(&this_balance_chip[i]->balance_dev_psy->dev, &dev_attr_balance_chip_en);
		device_remove_file(&this_balance_chip[i]->balance_dev_psy->dev, &dev_attr_balance_chrg_dis);
		device_remove_file(&this_balance_chip[i]->balance_dev_psy->dev, &dev_attr_balance_extmos_en);

		mmi_glink_unregister_notifier(&this_balance_chip[i]->balance_nb);
		power_supply_unregister(this_balance_chip[i]->balance_dev_psy);
		glink_device_unregister(this_balance_chip[i]->glink_dev);
		kfree(this_balance_chip[i]);
		mmi_err(this_root_chip, "balance_glink_device_unregister[%d] success", i);
	}
	mmi_err(this_root_chip, "balance_glink_device_unregister success");
}

