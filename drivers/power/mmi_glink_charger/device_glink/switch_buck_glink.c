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
#include "switch_buck_glink.h"

static struct mmi_glink_chip *this_root_chip = NULL;
static struct buck_glink_dev *this_buck_chip = NULL;

 static int buck_notify_handler(struct notifier_block *nb, unsigned long event, void *data)
{
	int rc = -1;
	struct buck_dev_info buck_info;

	mmi_dbg(this_root_chip, "buck: notify-dev %ld", event);
	if (event == DEV_SWITCH_BUCK || event == DEV_ALL) {

		rc = qti_charger_get_property(OEM_PROP_MSB_DEV_INFO,
					&buck_info,
					sizeof(buck_info));

		if (rc) {
			mmi_err(this_root_chip, "qti_glink: Get MSB_DUM_INFO property failed");
			return rc;
		}

		mmi_info(this_root_chip, "msb dev info : usb_iin: %dma, usb_vout: %dmv, batt_fcc: %dma, batt_fv: %dmv, jeita_en %d",
			buck_info.usb_iin,
			buck_info.usb_vout,
			buck_info.batt_fcc,
			buck_info.batt_fv,
			buck_info.jeita_en);

		 mmi_info(this_root_chip, "msb dev info : chg_en: %d, chg_fault: 0x%x, chg_st: 0x%x, vbus_gd %d, input_det %d,"
			"usb_suspend %d, vbus_ctrl %d, ce_en %d, battfet_dis %d",
			buck_info.chg_en,
			buck_info.chg_fault,
			buck_info.chg_st,
			buck_info.vbus_gd,
			buck_info.input_det,
			buck_info.usb_suspend,
			buck_info.vbus_ctrl,
			buck_info.ce_en,
			buck_info.battfet_dis);
	}
	return NOTIFY_DONE;
}

static ssize_t buck_chg_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct buck_glink_dev *chg = power_supply_get_drvdata(psy);
	u32 chg_en = 0;
	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}
	qti_charger_get_property(OEM_PROP_MSB_MASTER_CHG_EN,
				&chg_en,
				sizeof(chg_en));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg_en);
}

static ssize_t buck_chg_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct buck_glink_dev *chg = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long chg_en;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chg_en);
	if (r) {
		pr_err("Invalid chg_en = %lu\n", chg_en);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_MSB_MASTER_CHG_EN,
				&chg_en,
				sizeof(chg_en));

	mmi_info(this_root_chip, "Set switch_buck_chg_en = %lu", chg_en);
	return r ? r : count;
}

static DEVICE_ATTR(chg_en, S_IRUGO|S_IWUSR, buck_chg_en_show, buck_chg_en_store);

static ssize_t buck_usb_suspend_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct buck_glink_dev *chg = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long usb_suspend;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &usb_suspend);
	if (r) {
		pr_err("Invalid usb_suspend = %lu\n", usb_suspend);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_MSB_MASTER_USB_SUSPEND,
				&usb_suspend,
				sizeof(usb_suspend));

	mmi_info(this_root_chip, "Set switch_buck_usb_suspend = %lu", usb_suspend);
	return r ? r : count;
}

static ssize_t buck_usb_suspend_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct buck_glink_dev *chg = power_supply_get_drvdata(psy);
	u32 usb_suspend = 0;
	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}
	qti_charger_get_property(OEM_PROP_MSB_MASTER_USB_SUSPEND,
				&usb_suspend,
				sizeof(usb_suspend));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", usb_suspend);
}
static DEVICE_ATTR(usb_suspend, S_IRUGO|S_IWUSR, buck_usb_suspend_show, buck_usb_suspend_store);

static enum power_supply_property buck_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int buck_psy_get_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 union power_supply_propval *pval)
{
	struct buck_glink_dev *buck_chip = power_supply_get_drvdata(psy);
	struct timespec64 glink_access_time_now;
	int rc = 0;

	pval->intval = -ENODATA;

	ktime_get_real_ts64(&glink_access_time_now);
	buck_chip->elapsed_ms = (glink_access_time_now.tv_sec - buck_chip->glink_access_time.tv_sec) * 1000;
	buck_chip->elapsed_ms += (glink_access_time_now.tv_nsec - buck_chip->glink_access_time.tv_nsec) / 1000000;
	if (buck_chip->elapsed_ms > 1000) {
		ktime_get_real_ts64(&buck_chip->glink_access_time);
		buck_chip->elapsed_ms = 0;
		rc = qti_charger_get_property(OEM_PROP_MSB_DEV_INFO,
			&buck_chip->buck_info, sizeof(struct buck_dev_info));
		mmi_dbg(this_root_chip, "charge_pump_get_prop, DEV_INFO");
	}

	if (rc) {
		mmi_err(this_root_chip, "Get BUCK_INFO property failed");
		return rc;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = buck_chip->buck_info.chg_st;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = buck_chip->buck_info.chg_en;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = buck_chip->buck_info.batt_fv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pval->intval = buck_chip->buck_info.batt_fcc;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		pval->intval = buck_chip->buck_info.usb_iin;
		break;
	default:
		break;
	}

	return rc;
}

static int buck_psy_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	int rc = 0, prop_cmd = 0, value = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		prop_cmd = OEM_PROP_MSB_MASTER_USB_IIN;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		prop_cmd = OEM_PROP_MSB_MASTER_BATT_FCC;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		prop_cmd = OEM_PROP_MSB_MASTER_BATT_FV;
		break;
	default:
		mmi_err(this_root_chip, "Not supported property: %d\n", prop);
		return -EINVAL;
		break;
	}

	value = val->intval;
	rc = qti_charger_set_property(prop_cmd,
			&value,
			sizeof(value));
	if (rc)
		mmi_err(this_root_chip, "Failed to set property %d, rc=%d\n", prop_cmd, rc);
	else
		mmi_info(this_root_chip, "Set buck_charge, property %d, value %d", prop_cmd, value);

	return rc;
}

static int buck_psy_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}


static struct power_supply_desc buck_psy_desc = {
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= buck_psy_get_prop,
	.set_property	= buck_psy_set_prop,
	.property_is_writeable = buck_psy_prop_is_writeable,
	.properties	= buck_psy_props,
	.num_properties	= ARRAY_SIZE(buck_psy_props),
};

struct glink_device *switch_buck_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{

	struct buck_glink_dev *buck_chip = NULL;
	struct glink_device * glink_dev = NULL;
	struct power_supply_config psy_cfg = {};
	int rc = 0;

	if (!chip)
		goto exit;

	this_root_chip = chip;

	buck_chip = kzalloc(sizeof(struct buck_glink_dev), GFP_KERNEL);
	if (!buck_chip)
		goto exit;

	buck_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	psy_cfg.drv_data = buck_chip;

	buck_psy_desc.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
	buck_chip->buck_dev_psy = devm_power_supply_register(chip->dev,
						  &buck_psy_desc,
						  &psy_cfg);
	if (IS_ERR(buck_chip->buck_dev_psy)) {
		mmi_err(chip, "Couldn't register buck dev power supply");
		goto exit;
	}

	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_SWITCH_BUCK, buck_chip);
	if (!glink_dev)
		goto exit;

	buck_chip->glink_dev = glink_dev;
	buck_chip->buck_nb.notifier_call = buck_notify_handler;
	mmi_glink_register_notifier(&buck_chip->buck_nb);


	rc = device_create_file(&buck_chip->buck_dev_psy->dev,
				&dev_attr_chg_en);
        if (rc)
		pr_err("couldn't create switch buck chg_en\n");

	rc = device_create_file(&buck_chip->buck_dev_psy->dev,
				&dev_attr_usb_suspend);
        if (rc)
		pr_err("couldn't create switch buck chg_suspend\n");

	this_buck_chip = buck_chip;
	mmi_err(chip, "buck glink device %s register successfully", dev_dts->glink_dev_name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void switch_buck_device_unregister(void)
{
	if (!this_buck_chip)
		return;
	device_remove_file(&this_buck_chip->buck_dev_psy->dev, &dev_attr_chg_en);
	device_remove_file(&this_buck_chip->buck_dev_psy->dev, &dev_attr_usb_suspend);

	mmi_glink_unregister_notifier(&this_buck_chip->buck_nb);
	glink_device_unregister(this_buck_chip->glink_dev);
	kfree(this_buck_chip);

	mmi_err(this_root_chip, "switch_buck_device_unregister success");
}


