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
#include "../mmi_glink_core.h"
#include <linux/power/qti_glink_charger_v2.h>
#include "charge_pump_glink.h"

static struct mmi_glink_chip *this_root_chip =  NULL;
static struct charge_pump_glink_dev *this_charge_pump_chip[DEV_NUM] = {NULL};

static ssize_t charge_pump_chg_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct charge_pump_glink_dev *charge_pump_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 chg_en = 0, property = 0;

	if (!charge_pump_chip) {
		pr_err("Charge_pump: chip not valid\n");
		return -ENODEV;
	}

	dev_role = charge_pump_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MSC_MASTER_CHG_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MSC_SLAVE_CHG_EN;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&chg_en,
				sizeof(chg_en));

	mmi_dbg(this_root_chip, "Get charge_pump_chip_en[%d]=%d", dev_role, chg_en);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg_en);
}

static ssize_t charge_pump_chg_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct charge_pump_glink_dev *charge_pump_chip = power_supply_get_drvdata(psy);
	unsigned long r;
	unsigned long chg_en;

	int dev_role = 0;
	u32 property = 0;


	if (!charge_pump_chip) {
		pr_err("Charge_pump: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chg_en);
	if (r) {
		pr_err("Invalid charge_pump_chip_en = %lu\n", chg_en);
		return -EINVAL;
	}

	dev_role = charge_pump_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MSC_MASTER_CHG_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MSC_SLAVE_CHG_EN;
	else
		return -EINVAL;

	r = qti_charger_set_property(property,
				&chg_en,
				sizeof(chg_en));
	mmi_info(this_root_chip, "Set charge_pump_chg_en[%d] = %lu", dev_role, chg_en);
	return r ? r : count;
}
static DEVICE_ATTR(charge_pump_chg_en, S_IRUGO|S_IWUSR, charge_pump_chg_en_show, charge_pump_chg_en_store);

static ssize_t charge_pump_manual_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct charge_pump_glink_dev *charge_pump_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 manual_mode = 0, property = 0;

	if (!charge_pump_chip) {
		pr_err("Charge_pump: chip not valid\n");
		return -ENODEV;
	}

	dev_role = charge_pump_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MSC_MASTER_MANUAL_MODE;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MSC_SLAVE_MANUAL_MODE;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&manual_mode,
				sizeof(manual_mode));

	mmi_dbg(this_root_chip, "Get charge_pump_manual_mode[%d] = %d", dev_role, manual_mode);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", manual_mode);
}

static DEVICE_ATTR(charge_pump_manual_mode, S_IRUGO, charge_pump_manual_mode_show, NULL);

static ssize_t charge_pump_ovpgate_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct charge_pump_glink_dev *charge_pump_chip = power_supply_get_drvdata(psy);
	int dev_role = 0;
	u32 ovpgate_en = 0, property = 0;

	if (!charge_pump_chip) {
		pr_err("Charge_pump: chip not valid\n");
		return -ENODEV;
	}

	dev_role = charge_pump_chip->dev_role;

	if (dev_role == DEV_MASTER)
		property = OEM_PROP_MSC_MASTER_OVPGATE_EN;
	else if (dev_role == DEV_SLAVE)
		property = OEM_PROP_MSC_SLAVE_OVPGATE_EN;
	else
		return -EINVAL;

	qti_charger_get_property(property,
				&ovpgate_en,
				sizeof(ovpgate_en));

	mmi_dbg(this_root_chip, "Get charge_pump_ovpgate_en[%d]=%d", dev_role, ovpgate_en);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", ovpgate_en);
}

static DEVICE_ATTR(charge_pump_ovpgate_en, S_IRUGO, charge_pump_ovpgate_en_show, NULL);

static enum power_supply_property charge_pump_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int charge_pump_psy_get_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 union power_supply_propval *pval)
{
	if (!psy) {
		mmi_err(this_root_chip, "charge_pump_psy_get_prop, psy is null");
		return -ENODEV;
	}

	struct charge_pump_glink_dev *charge_pump_chip = power_supply_get_drvdata(psy);
	struct timespec64 glink_access_time_now;
	int rc = 0;
	bool msc_is_bq_dev = false;
	bool msc_is_cps_dev = false;

	if (!charge_pump_chip)
		return -ENODEV;

	pval->intval = -ENODATA;

	switch (charge_pump_chip->dev_role) {
	case DEV_MASTER:
		ktime_get_real_ts64(&glink_access_time_now);
		charge_pump_chip->elapsed_ms = (glink_access_time_now.tv_sec - charge_pump_chip->glink_access_time.tv_sec) * 1000;
		charge_pump_chip->elapsed_ms += (glink_access_time_now.tv_nsec - charge_pump_chip->glink_access_time.tv_nsec) / 1000000;
		if (charge_pump_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&charge_pump_chip->glink_access_time);
			charge_pump_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_MASTER_SWITCHEDCAP_INFO,
				&charge_pump_chip->charge_pump_dev_info, sizeof(struct charge_pump_dev_info));
			mmi_dbg(this_root_chip, "charge_pump_get_prop[%d], DEV_INFO", charge_pump_chip->dev_role);
		}
		break;
	case DEV_SLAVE:
		ktime_get_real_ts64(&glink_access_time_now);
		charge_pump_chip->elapsed_ms = (glink_access_time_now.tv_sec - charge_pump_chip->glink_access_time.tv_sec) * 1000;
		charge_pump_chip->elapsed_ms += (glink_access_time_now.tv_nsec - charge_pump_chip->glink_access_time.tv_nsec) / 1000000;
		if (charge_pump_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&charge_pump_chip->glink_access_time);
			charge_pump_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_SLAVE_SWITCHEDCAP_INFO,
				&charge_pump_chip->charge_pump_dev_info, sizeof(struct charge_pump_dev_info));
			mmi_dbg(this_root_chip, "charge_pump_get_prop[%d], DEV_INFO", charge_pump_chip->dev_role);
		}
		break;
	default:
		mmi_err(this_root_chip, "charge_pump_get_prop, Can not find correct charger_pump role %d", charge_pump_chip->dev_role);
		return rc;
		break;
	}

	if (rc) {
		mmi_err(this_root_chip, "charge_pump_role %d, Get CHARGE_PUMP property failed", charge_pump_chip->dev_role);
		return rc;
	}

	if (charge_pump_chip->charge_pump_dev_info.vendor_id == MSC_BQ)
		msc_is_bq_dev = true;
	else if (charge_pump_chip->charge_pump_dev_info.vendor_id == MSC_CPS)
		msc_is_cps_dev = true;

	switch (prop) {
	case  POWER_SUPPLY_PROP_STATUS:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.ce;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.chg_en;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.chg_en;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.online;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.otg_en;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.otg_en;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.vac1_mv;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.vbus_mv;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.vbus_mv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.vbat_mv;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.vout_mv;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.vout_mv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.ibus_ma;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.ibus_ma;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.ibus_ma;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (msc_is_bq_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_bq_info.tdie;
		else if (msc_is_cps_dev)
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_cps_info.die_temp;
		else
		    pval->intval = charge_pump_chip->charge_pump_dev_info.msc_info.msc_sc_info.die_temp;
		break;
	default:
		break;
	}

	return rc;
}

static struct power_supply_desc charge_pump_psy_desc_master = {
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= charge_pump_psy_get_prop,
	.set_property	= NULL,
	.properties	= charge_pump_psy_props,
	.num_properties	= ARRAY_SIZE(charge_pump_psy_props),
};

static struct power_supply_desc charge_pump_psy_desc_slave = {
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= charge_pump_psy_get_prop,
	.set_property	= NULL,
	.properties	= charge_pump_psy_props,
	.num_properties	= ARRAY_SIZE(charge_pump_psy_props),
};

static int charge_pump_notify_handler(struct notifier_block *nb, unsigned long event, void *data)
{
	int rc = -1;
	struct charge_pump_dev_info charge_pump_info;
	struct charge_pump_glink_dev *charge_pump_chip =
			container_of(nb, struct charge_pump_glink_dev, charge_pump_nb);

	mmi_dbg(this_root_chip, "charge_pump_role %d, notify-dev %ld", charge_pump_chip->dev_role, event);
	if (event == DEV_CHARGE_PUMP || event == DEV_ALL) {
		if (charge_pump_chip->dev_role == DEV_MASTER) {
			rc = qti_charger_get_property(OEM_PROP_MASTER_SWITCHEDCAP_INFO,
					&charge_pump_info, sizeof(charge_pump_info));
		} else if (charge_pump_chip->dev_role == DEV_SLAVE) {
			rc = qti_charger_get_property(OEM_PROP_SLAVE_SWITCHEDCAP_INFO,
					&charge_pump_info, sizeof(charge_pump_info));
		} else {
			mmi_err(this_root_chip, "charge_pump_get_prop, Can not find correct charge_pump role %d", charge_pump_chip->dev_role);
			return rc;
		}

		switch (charge_pump_info.vendor_id) {
		case MSC_BQ:
			mmi_info(this_root_chip, "charge_pump_dev[bq%04x]-[%d]:  vbat_mv %dmV, ibat_ma %dmA, ibus_ma %dmA,  vac1_adc %dmv, "
									"tsbat %d, tdie %d",
									charge_pump_info.msc_info.msc_bq_info.chip_id,
									charge_pump_chip->dev_role, charge_pump_info.msc_info.msc_bq_info.vbat_mv,
									charge_pump_info.msc_info.msc_bq_info.ibat_ma, charge_pump_info.msc_info.msc_bq_info.ibus_ma,
									charge_pump_info.msc_info.msc_bq_info.vac1_mv, charge_pump_info.msc_info.msc_bq_info.tsbat,
									charge_pump_info.msc_info.msc_bq_info.tdie);

			mmi_info(this_root_chip, "charge_pump_dev[bq%04x]-[%d]:  dischg %d, ovp %d, ocp %d, tflt %d,  wdt %d, ext_reverse %d, online %d, pin_short_cn %d",
									charge_pump_info.msc_info.msc_bq_info.chip_id,
									charge_pump_chip->dev_role, charge_pump_info.msc_info.msc_bq_info.dischg,
									charge_pump_info.msc_info.msc_bq_info.ovp, charge_pump_info.msc_info.msc_bq_info.ocp,
									charge_pump_info.msc_info.msc_bq_info.tflt, charge_pump_info.msc_info.msc_bq_info.wdt,
									charge_pump_info.msc_info.msc_bq_info.ext_reverse_en,
									charge_pump_info.msc_info.msc_bq_info.online,
									charge_pump_info.msc_info.msc_bq_info.pin_short_cn);

			mmi_info(this_root_chip, "charge_pump_dev[bq%04x]-[%d]:  ce %d, hiz %d, acdrv1 %d, acdrv2 %d, reverse %d, bypass %d, sc41 %d,  sc21 %d",
									charge_pump_info.msc_info.msc_bq_info.chip_id,
									charge_pump_chip->dev_role, charge_pump_info.msc_info.msc_bq_info.ce,
									charge_pump_info.msc_info.msc_bq_info.hiz, charge_pump_info.msc_info.msc_bq_info.acdrv1_en,
									charge_pump_info.msc_info.msc_bq_info.acdrv2_en, charge_pump_info.msc_info.msc_bq_info.reverse_en,
									charge_pump_info.msc_info.msc_bq_info.bypass,
									charge_pump_info.msc_info.msc_bq_info.sc41, charge_pump_info.msc_info.msc_bq_info.sc21);
#ifdef CONFIG_MSC_BQ_PIN_SHORT_PANIC
			BUG_ON(charge_pump_info.msc_info.msc_bq_info.pin_short_cn > 8);
#endif
		break;
		case MSC_CPS:
			mmi_info(this_root_chip, "charge_pump_dev[cps0x%04x]-[%d]: chg_en %d, work_mode %d, ovpgate %d, manual_mode %d, otg_en %d, "
									"int_stat %d, ibus_ma %d, ibat_ma %d",
									charge_pump_info.msc_info.msc_cps_info.chip_id, charge_pump_chip->dev_role, charge_pump_info.msc_info.msc_cps_info.chg_en, charge_pump_info.msc_info.msc_cps_info.work_mode,
									charge_pump_info.msc_info.msc_cps_info.ovpgate, charge_pump_info.msc_info.msc_cps_info.manual, charge_pump_info.msc_info.msc_cps_info.otg_en, charge_pump_info.msc_info.msc_cps_info.int_stat,
									charge_pump_info.msc_info.msc_cps_info.ibus_ma, charge_pump_info.msc_info.msc_cps_info.ibat_ma);
			mmi_info(this_root_chip, "charge_pump_dev[cps0x%04x]-[%d]: vbus_mv %d, vout_mv %d, vac_mv %d, vbat_mv %d, vusb_mv %d, die_temp %d",
									charge_pump_info.msc_info.msc_cps_info.chip_id, charge_pump_chip->dev_role,charge_pump_info.msc_info.msc_cps_info.vbus_mv, charge_pump_info.msc_info.msc_cps_info.vout_mv, charge_pump_info.msc_info.msc_cps_info.vac_mv,
									charge_pump_info.msc_info.msc_cps_info.vbat_mv, charge_pump_info.msc_info.msc_cps_info.vusb_mv, charge_pump_info.msc_info.msc_cps_info.die_temp);
		break;
		default:
			mmi_info(this_root_chip, "charge_pump_dev[0x%04x]-[%d]: chg_en %d, work_mode %d, ovpgate %d, manual_mode %d, otg_en %d, "
									"int_stat %d, ibus_ma %d, ibat_ma %d",
									charge_pump_info.msc_info.msc_sc_info.chip_id, charge_pump_chip->dev_role, charge_pump_info.msc_info.msc_sc_info.chg_en, charge_pump_info.msc_info.msc_sc_info.work_mode,
									charge_pump_info.msc_info.msc_sc_info.ovpgate, charge_pump_info.msc_info.msc_sc_info.manual, charge_pump_info.msc_info.msc_sc_info.otg_en, charge_pump_info.msc_info.msc_sc_info.int_stat,
									charge_pump_info.msc_info.msc_sc_info.ibus_ma, charge_pump_info.msc_info.msc_sc_info.ibat_ma);
			mmi_info(this_root_chip, "charge_pump_dev[0x%04x]-[%d]: vbus_mv %d, vout_mv %d, vac_mv %d, vbat_mv %d, vusb_mv %d, vwpc_mv %d, die_temp %d",
									charge_pump_info.msc_info.msc_sc_info.chip_id, charge_pump_chip->dev_role,charge_pump_info.msc_info.msc_sc_info.vbus_mv, charge_pump_info.msc_info.msc_sc_info.vout_mv, charge_pump_info.msc_info.msc_sc_info.vac_mv,
									charge_pump_info.msc_info.msc_sc_info.vbat_mv, charge_pump_info.msc_info.msc_sc_info.vusb_mv, charge_pump_info.msc_info.msc_sc_info.vwpc_mv, charge_pump_info.msc_info.msc_sc_info.die_temp);
		break;
		}
	}

	return NOTIFY_DONE;
}

struct glink_device *charge_pump_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{
	struct charge_pump_glink_dev *charge_pump_chip = NULL;
	struct glink_device * glink_dev = NULL;
	struct power_supply_config psy_cfg = {};
	int rc = 0;

	if (!chip)
		goto exit;

	this_root_chip = chip;

	charge_pump_chip = kzalloc(sizeof(struct charge_pump_glink_dev), GFP_KERNEL);
	if (!charge_pump_chip)
		goto exit;

	charge_pump_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	charge_pump_chip->dev_role = dev_dts->dev_role;
	psy_cfg.drv_data = charge_pump_chip;

	mmi_info(chip, "charge_pump [%d] glink devices psy", charge_pump_chip->dev_role);
	if (charge_pump_chip->dev_role == DEV_MASTER) {
		charge_pump_psy_desc_master.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		charge_pump_chip->charge_pump_dev_psy = devm_power_supply_register(chip->dev,
							  &charge_pump_psy_desc_master,
							  &psy_cfg);
		if (IS_ERR(charge_pump_chip->charge_pump_dev_psy)) {
			mmi_err(chip, "Couldn't register charge_pump dev power supply");
			goto exit;
		}
	} else if (charge_pump_chip->dev_role == DEV_SLAVE) {
		charge_pump_psy_desc_slave.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		charge_pump_chip->charge_pump_dev_psy = devm_power_supply_register(chip->dev,
							  &charge_pump_psy_desc_slave,
							  &psy_cfg);
		if (IS_ERR(charge_pump_chip->charge_pump_dev_psy)) {
			mmi_err(chip, "Couldn't register charge_pump dev power supply");
			goto exit;
		}
	} else {
		mmi_err(chip, "Couldn't register charge_pump role %d, Failed register charge_pump glink", charge_pump_chip->dev_role);
		goto exit;
	}

	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_CHARGE_PUMP, charge_pump_chip);
	if (!glink_dev)
		goto exit;

	charge_pump_chip->glink_dev = glink_dev;
	charge_pump_chip->charge_pump_nb.notifier_call = charge_pump_notify_handler;
	mmi_glink_register_notifier(&charge_pump_chip->charge_pump_nb);

	rc = device_create_file(&charge_pump_chip->charge_pump_dev_psy->dev,
				&dev_attr_charge_pump_chg_en);
        if (rc)
		pr_err("couldn't create charge_pump chg en\n");

	rc = device_create_file(&charge_pump_chip->charge_pump_dev_psy->dev,
				&dev_attr_charge_pump_manual_mode);
        if (rc)
		pr_err("couldn't create charge_pump manual mode\n");

	rc = device_create_file(&charge_pump_chip->charge_pump_dev_psy->dev,
				&dev_attr_charge_pump_ovpgate_en);
        if (rc)
		pr_err("couldn't create charge_pump ovpgate en\n");

	this_charge_pump_chip[charge_pump_chip->dev_role] = charge_pump_chip;

	mmi_err(chip, "charge_pump glink device %s register successfully", dev_dts->glink_dev_name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void charge_pump_glink_device_unregister(void)
{
	int i = 0;
	for (i = 0; i < DEV_NUM; i++) {
		if(!this_charge_pump_chip[i])
			return;
		device_remove_file(&this_charge_pump_chip[i]->charge_pump_dev_psy->dev, &dev_attr_charge_pump_chg_en);
		device_remove_file(&this_charge_pump_chip[i]->charge_pump_dev_psy->dev, &dev_attr_charge_pump_manual_mode);
		device_remove_file(&this_charge_pump_chip[i]->charge_pump_dev_psy->dev, &dev_attr_charge_pump_ovpgate_en);

		mmi_glink_unregister_notifier(&this_charge_pump_chip[i]->charge_pump_nb);
		power_supply_unregister(this_charge_pump_chip[i]->charge_pump_dev_psy);
		glink_device_unregister(this_charge_pump_chip[i]->glink_dev);
		kfree(this_charge_pump_chip[i]);
		mmi_err(this_root_chip, "charge_pump_glink_device[%d]_unregister", i);
	}
	mmi_err(this_root_chip, "charge_pump_glink_device_unregister");
}
