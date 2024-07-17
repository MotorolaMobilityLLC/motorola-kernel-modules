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

#define pr_fmt(fmt)     "GLINK_CHG:CHG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/power/bm_adsp_ulog.h>
#include <linux/thermal.h>

#include "mmi_charger.h"
#include "glink_device.h"
#include "trusted_shash_lib.h"

enum {
	TCD_TYPE_PRIMARY,
	TCD_TYPE_SECONDARY,
	TCD_TYPE_NUM
};

struct chg_tcd {
	int				type;
	u32				*thermal_levels;
	u32				thermal_fcc_ua;
	int				curr_thermal_level;
	int				num_thermal_levels;
	struct thermal_cooling_device	*tcd;
	struct glink_dev		*dev;
};
static struct chg_tcd			chg_tcd[TCD_TYPE_NUM];

struct encrypted_data {
	u32 random_num[4];
	u32 hmac_data[4];
	u32 sha1_data[4];
};

struct glink_charger {
	struct glink_dev		*dev;
	struct glink_dev		*bdev;
	struct notifier_block		glink_nb;
	bool				cfg_done;
	bool				init_done;

	bool				mmi_charger_disabled;
	struct power_supply		*partner_charger;
	u32				partner_charger_icl;
	u32				partner_charger_soc;
	const char			*name;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_info		chg_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_constraint	constraint;
	struct mmi_charger_driver	*driver;
	u32				chg_taper_cnt;
	struct chg_tcd			*tcd;
	bool				encrypted_chg_enabled;
};

static void glink_encrypt_authentication(struct glink_charger *chg)
{
	int rc;
	int i;
	TRUSTED_SHASH_RESULT result;
	struct encrypted_data send_data;
	u8 random_num[4] = {0};
	u32 property;
	struct glink_dev *dev = chg->dev;

	if (!dev) {
		pr_err("Invalid glink charger\n");
		return;
	}

	if (!chg->encrypted_chg_enabled) {
		pr_err("encrypted charger is not enabled\n");
		return;
	}

	memset(&send_data, 0, sizeof(send_data));
	for (i = 0; i < 4; i++) {
		get_random_bytes(&random_num[i], sizeof(random_num[0]));
		result.random_num[i] = random_num[i] % 26 + 'a';
		send_data.random_num[i] = result.random_num[i];
		pr_debug("encrypt random_num[%d]=0x%08x, result_num=0x%08x\n",
				i,
				random_num[i],
				send_data.random_num[i]);
	}

	rc = trusted_sha1(result.random_num, 4, result.sha1);
	if (rc) {
		pr_err("trusted_sha1 failed, rc=%d\n", rc);
		return;
	}

	rc = trusted_hmac(result.random_num, 4, result.hmac_sha256);
	if (rc) {
		pr_err("trusted_hmac failed, rc=%d\n", rc);
		return;
	}

	for (i = 0; i < 4; i++) {
		send_data.hmac_data[i] =
				(result.hmac_sha256[3 + (4 * i)] << 0) +
				(result.hmac_sha256[2 + (4 * i)] << 8) +
				(result.hmac_sha256[1 + (4 * i)] << 16) +
				(result.hmac_sha256[0 + (4 * i)] << 24);
		pr_debug("encrypt hmac_sha256[%d]=0x%08x\n", i,
				send_data.hmac_data[i]);
	}

	for (i = 0; i < 4; i++) {
		send_data.sha1_data[i] =
				(result.sha1[3 + (4 * i)] << 0) +
				(result.sha1[2 + (4 * i)] << 8) +
				(result.sha1[1 + (4 * i)] << 16) +
				(result.sha1[0 + (4 * i)] << 24);
		pr_debug("encrypt hmac_sha1[%d]=0x%08x\n", i,
				send_data.sha1_data[i]);
	}

	property = GLINK_PROP_ENCRYT_DATA;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.set_property(dev,
				property,
				&send_data,
				sizeof(send_data));
	if (rc) {
		pr_err("Failed to set encrypted data, rc=%d\n", rc);
	}
}

static int glink_charger_status_update(struct glink_charger *chg)
{
	int rc;
	u32 property;
	struct battery_info battinfo = {0};
	struct charger_info chginfo = {0};
	struct glink_dev *dev = chg->dev;

	if (!dev) {
		pr_err("Invalid glink charger\n");
		return -ENODEV;
	}

	if (!chg->bdev) {
		pr_err("no available battery device\n");
		return -ENODEV;
	}

	property = GLINK_PROP_BATT_INFO;
	property |= chg->bdev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(chg->bdev,
				property,
				&battinfo,
				sizeof(struct battery_info));
	if (rc) {
		pr_err("Failed to read batt info, rc=%d\n", rc);
		return rc;
	}

	chg->batt_info.batt_ma = battinfo.batt_ua / 1000;
	chg->batt_info.batt_mv = battinfo.batt_uv / 1000;
	chg->batt_info.batt_soc = battinfo.batt_soc / 100;
	chg->batt_info.batt_temp = battinfo.batt_temp / 100;
	chg->batt_info.batt_status = battinfo.batt_status;
	chg->batt_info.batt_full_uah = battinfo.batt_full_uah;
	chg->batt_info.batt_design_uah = battinfo.batt_design_uah;
	chg->batt_info.batt_chg_counter = battinfo.batt_chg_counter;
	chg->batt_info.batt_fv_mv = battinfo.batt_fv_uv / 1000;
	chg->batt_info.batt_fcc_ma = battinfo.batt_fcc_ua / 1000;
	chg->batt_info.batt_soh = battinfo.batt_soh;

	property = GLINK_PROP_CHG_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev,
				property,
				&chginfo,
				sizeof(struct charger_info));
	if (rc) {
		pr_err("Failed to read chg info, rc=%d\n", rc);
		return rc;
	}

	if (chg->encrypted_chg_enabled &&
	    chg->chg_info.chrg_present != chginfo.chrg_present) {
		glink_encrypt_authentication(chg);
	}

	chg->chg_info.chrg_mv = chginfo.chrg_uv / 1000;
	chg->chg_info.chrg_ma = chginfo.chrg_ua / 1000;
	chg->chg_info.chrg_type = chginfo.chrg_type;
	chg->chg_info.chrg_pmax_mw = chginfo.chrg_pmax_mw;
	chg->chg_info.chrg_present = chginfo.chrg_present;
	if (!chginfo.chrg_present && chginfo.chrg_type != 0)
		chg->chg_info.chrg_present = 1;

	if (!!chginfo.chrg_present && !!chginfo.chrg_wired)
		chg->chg_info.vbus_present = 1;
	else
		chg->chg_info.vbus_present = 0;
	chg->chg_info.chrg_otg_enabled = chginfo.chrg_otg_enabled;
	pr_info("%s: chrg_present:%d, chrg_type:%d, chrg_mv:%d, chrg_ma:%d, "
	        "chrg_pmax_mw:%d, chrg_otg:%d, chrg_wired=%d, chrg_sm_st=%d\n",
		chg->dev->name,
		chginfo.chrg_present, chginfo.chrg_type,
		chginfo.chrg_uv / 1000, chginfo.chrg_ua / 1000,
		chginfo.chrg_pmax_mw, chginfo.chrg_otg_enabled,
		chginfo.chrg_wired, chginfo.chrg_sm_st);

	if (chg->tcd) {
		pr_info("therm_primary: limit_level=%d, fcc_ma=%d, "
			"therm_secondary: limit_level=%d, fcc_ma=%d\n",
			chg->tcd[TCD_TYPE_PRIMARY].curr_thermal_level,
			chg->tcd[TCD_TYPE_PRIMARY].thermal_fcc_ua,
			chg->tcd[TCD_TYPE_SECONDARY].curr_thermal_level,
			chg->tcd[TCD_TYPE_SECONDARY].thermal_fcc_ua);
	}

	return 0;
}

static int glink_charger_get_batt_info(void *data,
				struct mmi_battery_info *batt_info)
{
	int rc;
	struct glink_charger *chg = data;

	rc = glink_charger_status_update(chg);
	if (rc) {
		pr_err("Failed to update charging status rc=%d\n", rc);
		return rc;
	}

	memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));
	return 0;
}

static int glink_charger_get_chg_info(void *data,
				struct mmi_charger_info *chg_info)
{
	struct glink_charger *chg = data;

	memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));
	return 0;
}

static int glink_charger_config_charge(void *data,
				struct mmi_charger_cfg *config)
{
	int rc;
	u32 value;
	u32 property;
	struct glink_charger *chg = data;
	struct glink_dev *dev = chg->dev;

	if (!dev) {
		pr_err("Invalid charger glink device\n");
		return -ENODEV;
	}

	/* configure the charger if changed */
	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		value = config->charger_suspend;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_SUSPEND;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charger_suspend = !!value;
	}
	if (config->charging_disable != chg->chg_cfg.charging_disable) {
		value = config->charging_disable;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_DISABLE;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charging_disable = !!value;
	}

	if (config->taper_kickoff != chg->chg_cfg.taper_kickoff) {
		chg->chg_cfg.taper_kickoff = config->taper_kickoff;
		chg->chg_taper_cnt = 0;
	}

	if (config->full_charged != chg->chg_cfg.full_charged) {
		chg->chg_cfg.full_charged = config->full_charged;
	}

	if (config->charging_reset != chg->chg_cfg.charging_reset) {
		if (config->charging_reset) {
			value = 1;
			property = dev->id << DEVICE_ID_SHIFT;
			property |= GLINK_PROP_CHG_DISABLE;
			rc = dev->ops.set_property(dev,
						property,
						&value,
						sizeof(value));
			msleep(200);
			value = 0;
			rc = dev->ops.set_property(dev,
						property,
						&value,
						sizeof(value));
		}
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	if (chg->partner_charger) {
		union power_supply_propval propval = {0};
		rc = power_supply_get_property(chg->partner_charger,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
				&propval);
		value = propval.intval;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_PARTNER_ICL;
		if (!rc && chg->partner_charger_icl != value) {
			rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		}
		if (!rc)
			chg->partner_charger_icl = value;

		rc = power_supply_get_property(chg->partner_charger,
				POWER_SUPPLY_PROP_CAPACITY,
				&propval);
		value = propval.intval;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_PARTNER_SOC;
		if (!rc && chg->partner_charger_icl != value) {
			rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		}
		if (!rc)
			chg->partner_charger_soc = value;
	}

	return 0;
}

#define TAPER_COUNT 2
static bool glink_charger_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct glink_charger *chg = data;

	if (abs(chg->batt_info.batt_ma) <= tapered_ma) {
		if (chg->chg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chg->chg_taper_cnt = 0;
		} else
			chg->chg_taper_cnt++;
	} else
		chg->chg_taper_cnt = 0;

	return is_tapered;
}

static bool glink_charger_is_charge_halt(void *data)
{
	struct glink_charger *chg = data;

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		return true;

	return false;
}

static void glink_charger_set_constraint(void *data,
				struct mmi_charger_constraint *constraint)
{
	int rc;
	u32 value;
	u32 property;
	struct glink_charger *chg = data;
	struct glink_dev *dev = chg->dev;

	if (!dev) {
		pr_err("Invalid glink charger device\n");
		return;
	}

	if (constraint->demo_mode != chg->constraint.demo_mode) {
		value = constraint->demo_mode;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_DEMO_MODE;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.demo_mode = !!value;
	}

	if (constraint->factory_version != chg->constraint.factory_version) {
		value = constraint->factory_version;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_FACTORY_VERSION;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_version = !!value;
	}

	if (constraint->factory_mode != chg->constraint.factory_mode) {
		value = constraint->factory_mode;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_FACTORY_MODE;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_mode = !!value;
	}

	if (constraint->dcp_pmax != chg->constraint.dcp_pmax) {
		value = constraint->dcp_pmax;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_BC_PMAX;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.dcp_pmax = constraint->dcp_pmax;
	}

	if (constraint->hvdcp_pmax != chg->constraint.hvdcp_pmax) {
		value = constraint->hvdcp_pmax;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_QC_PMAX;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.hvdcp_pmax = constraint->hvdcp_pmax;
	}

	if (constraint->pd_pmax != chg->constraint.pd_pmax) {
		value = constraint->pd_pmax;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_PD_PMAX;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.pd_pmax = constraint->pd_pmax;
	}

	if (constraint->wls_pmax != chg->constraint.wls_pmax) {
		value = constraint->wls_pmax;
		property = dev->id << DEVICE_ID_SHIFT;
		property |= GLINK_PROP_CHG_WLS_PMAX;
		rc = dev->ops.set_property(dev,
					property,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.wls_pmax = constraint->wls_pmax;
	}
}

static int thermal_charge_control_limit_set(struct chg_tcd *tcd, int level)
{
	int rc;
	u32 property;

	if (tcd->type < 0 || tcd->type > TCD_TYPE_NUM) {
		pr_err("tcd type = %d out of range\n", tcd->type);
		return -EINVAL;
	}

	if (tcd->num_thermal_levels <= 0) {
		pr_err("tcd-%d: invalid levels=%d\n", tcd->type,
				tcd->num_thermal_levels);
		return -EINVAL;
	}

	if (level < 0 || level > tcd->num_thermal_levels) {
		pr_err("tcd-%d: level=%d out of range\n", tcd->type, level);
		return -EINVAL;
	}

	tcd->thermal_fcc_ua = tcd->thermal_levels[level];
	tcd->curr_thermal_level = level;
	pr_info("tcd-%d: level=%d, fcc_ma = %d\n", tcd->type,
				level, tcd->thermal_fcc_ua);

	property = tcd->dev->id << DEVICE_ID_SHIFT;
	if (tcd->type == TCD_TYPE_PRIMARY)
		property |= GLINK_PROP_THERM_PRIMARY_CHG_CONTROL;
	else
		property |= GLINK_PROP_THERM_SECONDARY_CHG_CONTROL;
	if (tcd->dev && tcd->dev->ops.set_property) {
		rc = tcd->dev->ops.set_property(tcd->dev,
				property,
				&tcd->thermal_fcc_ua,
				sizeof(tcd->thermal_fcc_ua));
		if (rc) {
			pr_err("tcd-%d: set charge ctrl failed\n", tcd->type);
			return rc;
		}
	} else {
		pr_err("tcd-%d: no available glink device\n", tcd->type);
		return -ENODEV;
	}

	return rc;
}

static ssize_t thermal_primary_charge_control_limit_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 level;

	rc = kstrtouint(buf, 0, &level);
	if (rc) {
		pr_err("Invalid primary_limit_level = %u\n", level);
		return -EINVAL;
	}

	rc = thermal_charge_control_limit_set(
				&chg_tcd[TCD_TYPE_PRIMARY],
				(int)level);
	if (rc)
		pr_err("set primary_limit_level = %u failed\n", level);

	return rc ? rc : count;
}

static ssize_t thermal_primary_charge_control_limit_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				chg_tcd[TCD_TYPE_PRIMARY].curr_thermal_level);
}
static DEVICE_ATTR(thermal_primary_charge_control_limit, S_IRUGO|S_IWUSR,
				thermal_primary_charge_control_limit_show,
				thermal_primary_charge_control_limit_store);

static ssize_t thermal_primary_charge_control_limit_max_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				chg_tcd[TCD_TYPE_PRIMARY].num_thermal_levels);
}
static DEVICE_ATTR(thermal_primary_charge_control_limit_max, S_IRUGO,
				thermal_primary_charge_control_limit_max_show,
				NULL);

static ssize_t thermal_secondary_charge_control_limit_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 level;

	rc = kstrtouint(buf, 0, &level);
	if (rc) {
		pr_err("Invalid secondary_limit_level = %u\n", level);
		return -EINVAL;
	}

	rc = thermal_charge_control_limit_set(
				&chg_tcd[TCD_TYPE_SECONDARY],
				(int)level);
	if (rc)
		pr_err("set secondary_limit_level = %u failed\n", level);

	return rc ? rc : count;
}

static ssize_t thermal_secondary_charge_control_limit_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
		chg_tcd[TCD_TYPE_SECONDARY].curr_thermal_level);
}
static DEVICE_ATTR(thermal_secondary_charge_control_limit, S_IRUGO|S_IWUSR,
				thermal_secondary_charge_control_limit_show,
				thermal_secondary_charge_control_limit_store);

static ssize_t thermal_secondary_charge_control_limit_max_show(
				struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
		chg_tcd[TCD_TYPE_SECONDARY].num_thermal_levels);
}
static DEVICE_ATTR(thermal_secondary_charge_control_limit_max, S_IRUGO,
				thermal_secondary_charge_control_limit_max_show,
				NULL);

static inline int primary_get_max_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	*state = chg->tcd[TCD_TYPE_PRIMARY].num_thermal_levels;
	return 0;
}

static inline int primary_get_cur_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	*state = chg->tcd[TCD_TYPE_PRIMARY].curr_thermal_level;
	return 0;
}

static int primary_set_cur_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long state)
{
	int rc;
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	rc = thermal_charge_control_limit_set(
				&chg->tcd[TCD_TYPE_PRIMARY],
				(int)state);
	if (rc)
		pr_err("set primary_limit_level = %lu failed\n", state);

	return 0;
}

static const struct thermal_cooling_device_ops primary_charge_ops = {
	.get_max_state = primary_get_max_charge_cntl_limit,
	.get_cur_state = primary_get_cur_charge_cntl_limit,
	.set_cur_state = primary_set_cur_charge_cntl_limit,
};

static inline int secondary_get_max_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	*state = chg->tcd[TCD_TYPE_SECONDARY].num_thermal_levels;
	return 0;
}

static inline int secondary_get_cur_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long *state)
{
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	*state = chg->tcd[TCD_TYPE_SECONDARY].curr_thermal_level;
	return 0;
}

static int secondary_set_cur_charge_cntl_limit(
				struct thermal_cooling_device *tcd,
				unsigned long state)
{
	int rc;
	struct glink_charger *chg = tcd->devdata;

	if (!chg || !chg->tcd) {
		pr_err("Invalid chg tcd\n");
		return -ENODEV;
	}

	rc = thermal_charge_control_limit_set(
				&chg->tcd[TCD_TYPE_SECONDARY],
				(int)state);
	if (rc)
		pr_err("set secondary_limit_level = %lu failed\n", state);

	return 0;
}

static const struct thermal_cooling_device_ops secondary_charge_ops = {
	.get_max_state = secondary_get_max_charge_cntl_limit,
	.get_cur_state = secondary_get_cur_charge_cntl_limit,
	.set_cur_state = secondary_set_cur_charge_cntl_limit,
};

static void thermal_charge_control_init(struct glink_charger *chg)
{
	int rc;
	struct power_supply *batt_psy;
	struct chg_tcd *tcd = chg->tcd;
	struct thermal_cooling_device *ptcd = NULL;

	if (!chg) {
		pr_err("chip not valid\n");
		return;
	}

	if (!tcd) {
		pr_err("No tcd in charger %s\n", chg->name);
		return;
	}

	if (tcd[TCD_TYPE_PRIMARY].thermal_levels) {
		ptcd = thermal_cooling_device_register("primary_charge",
				chg, &primary_charge_ops);
		if (IS_ERR_OR_NULL(ptcd)) {
			rc = PTR_ERR_OR_ZERO(ptcd);
			ptcd = NULL;
			pr_err("create primary cooling device err=%d\n", rc);
		}
	}
	tcd[TCD_TYPE_PRIMARY].tcd = ptcd;

	ptcd = NULL;
	if (tcd[TCD_TYPE_SECONDARY].thermal_levels) {
		ptcd = thermal_cooling_device_register("secondary_charge",
				chg, &secondary_charge_ops);
		if (IS_ERR_OR_NULL(ptcd)) {
			rc = PTR_ERR_OR_ZERO(ptcd);
			ptcd = NULL;
			pr_err("create secondary cooling device err=%d\n", rc);
		}
	}
	tcd[TCD_TYPE_SECONDARY].tcd = ptcd;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("No battery power supply found\n");
		return;
	}

	rc = device_create_file(&batt_psy->dev,
			&dev_attr_thermal_primary_charge_control_limit);
	if (rc) {
		pr_err("failed to create primary_control_limit\n");
	}

	rc = device_create_file(&batt_psy->dev,
			&dev_attr_thermal_primary_charge_control_limit_max);
	if (rc) {
		pr_err("failed to create primary_control_limit_max\n");
	}

	rc = device_create_file(&batt_psy->dev,
			&dev_attr_thermal_secondary_charge_control_limit);
	if (rc) {
		pr_err("failed to create secondary_control_limit\n");
	}

	rc = device_create_file(&batt_psy->dev,
			&dev_attr_thermal_secondary_charge_control_limit_max);
	if (rc) {
		pr_err("failed to create secondary_control_limit_max\n");
	}
	power_supply_put(batt_psy);
}

static void thermal_charge_control_deinit(struct glink_charger *chg)
{
	struct power_supply *batt_psy;
	struct chg_tcd *tcd = chg->tcd;

	if (!chg) {
		pr_err("chip not valid\n");
		return;
	}

	if (!tcd) {
		pr_err("No tcd in charger %s\n", chg->name);
		return;
	}

	thermal_cooling_device_unregister(tcd[TCD_TYPE_PRIMARY].tcd);
	thermal_cooling_device_unregister(tcd[TCD_TYPE_SECONDARY].tcd);

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("No battery power supply found\n");
		goto free_tcd;
	}

	device_remove_file(&batt_psy->dev,
			&dev_attr_thermal_primary_charge_control_limit);

	device_remove_file(&batt_psy->dev,
			&dev_attr_thermal_primary_charge_control_limit_max);

	device_remove_file(&batt_psy->dev,
			&dev_attr_thermal_secondary_charge_control_limit);

	device_remove_file(&batt_psy->dev,
			&dev_attr_thermal_secondary_charge_control_limit_max);
	power_supply_put(batt_psy);

free_tcd:
	devm_kfree(chg->dev->dev, tcd[TCD_TYPE_PRIMARY].thermal_levels);
	devm_kfree(chg->dev->dev, tcd[TCD_TYPE_SECONDARY].thermal_levels);
	memset(tcd, 0, sizeof(*tcd) * TCD_TYPE_NUM);
	chg->tcd = NULL;
}

static int glink_charger_cfg(struct glink_charger *chg,
				struct glink_dev_cfg *cfg)
{
	int rc;
	u32 property;
	const char *batt_sn;
	struct glink_dev *dev = chg->dev;

	if (!chg->bdev) {
		chg->bdev = glink_device_get_by_phandle(dev->node,
				"battery", 0);
	}
	if (!chg->bdev) {
		pr_err("failed to get battery device in %s\n", dev->name);
		return -ENODATA;
	}

	chg->constraint.factory_mode = cfg->factory_mode;
	chg->constraint.factory_version = cfg->factory_version;

	batt_sn = glink_battery_get_serial_number(chg->bdev);
	if (batt_sn)
		strcpy(chg->batt_info.batt_sn, batt_sn);
	else
		strcpy(chg->batt_info.batt_sn, "unknown-sn");

	property = dev->id << DEVICE_ID_SHIFT;
	property |= GLINK_PROP_HW_REVISION;
	rc = dev->ops.set_property(dev,
				property,
				&cfg->hw_rev,
				sizeof(cfg->hw_rev));
	if (rc) {
		pr_err("failed to set HW version = %d", cfg->hw_rev);
		return rc;
	}

	property = dev->id << DEVICE_ID_SHIFT;
	property |= GLINK_PROP_FACTORY_MODE;
	rc = dev->ops.set_property(dev,
				property,
				&cfg->factory_mode,
				sizeof(cfg->factory_mode));
	if (rc) {
		pr_err("failed to set factory mode = %d",
					cfg->factory_mode);
		return rc;
	}

	property = dev->id << DEVICE_ID_SHIFT;
	property |= GLINK_PROP_FACTORY_VERSION;
	rc = dev->ops.set_property(dev,
				property,
				&cfg->factory_version,
				sizeof(cfg->factory_version));
	if (rc) {
		pr_err("failed to set factory version = %d",
					cfg->factory_version);
		return rc;
	}

	property = dev->id << DEVICE_ID_SHIFT;
	property |= GLINK_PROP_SKU_TYPE;
	rc = dev->ops.set_property(dev,
				property,
				&cfg->sku_type,
				sizeof(cfg->sku_type));
	if (rc) {
		pr_err("failed to set SKU type = %d", cfg->sku_type);
		return rc;
	}

	if (chg->encrypted_chg_enabled) {
		glink_encrypt_authentication(chg);
	}

	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_charger_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct glink_charger *chg = dev->devdata;
	struct mmi_charger_driver *driver;

	if (!chg) {
		pr_err("glink charger is not ready\n");
		return -ENODEV;
	}

	if (chg->init_done)
		return 0;

	if (chg->encrypted_chg_enabled) {
		trusted_shash_alloc();
	}

	rc = glink_charger_cfg(chg, cfg);
	if (rc) {
		pr_err("failed to cfg glink devices\n");
		return rc;
	}
	chg->cfg_done = true;
	thermal_charge_control_init(chg);

	if (chg->mmi_charger_disabled) {
		pr_warn("Skip to register to mmi_charger\n");
		chg->init_done = true;
		return 0;
	}

	if (chg->driver) {
		pr_warn("glink charger has already inited\n");
		chg->init_done = true;
		return 0;
	}

	driver = devm_kzalloc(chg->dev->dev,
			sizeof(struct mmi_charger_driver),
			GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = chg->name;
	driver->dev = chg->dev->dev;
	driver->data = chg;
	driver->get_batt_info = glink_charger_get_batt_info;
	driver->get_chg_info = glink_charger_get_chg_info;
	driver->config_charge = glink_charger_config_charge;
	driver->is_charge_tapered = glink_charger_is_charge_tapered;
	driver->is_charge_halt = glink_charger_is_charge_halt;
	driver->set_constraint = glink_charger_set_constraint;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		chg->driver = NULL;
		pr_err("glink charger init failed, rc=%d\n", rc);
	} else {
		pr_info("glink charger init successfully\n");
	}
	chg->init_done = true;

	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_charger_deinit(struct glink_dev *dev)
{
	int rc;
	struct glink_charger *chg = dev->devdata;

	if (!chg) {
		pr_err("glink charger is not ready\n");
		return -ENODEV;
	}

	thermal_charge_control_deinit(chg);
	if (chg->bdev)
		glink_device_put(chg->bdev);

	if (!chg->driver) {
		pr_info("glink charger has not inited yet\n");
		return -ENODEV;
	}

	/* unregister driver from mmi charger */
	rc = mmi_unregister_charger_driver(chg->driver);
	if (rc) {
		pr_err("glink charger deinit failed, rc=%d\n", rc);
	} else {
		devm_kfree(chg->dev->dev, chg->driver);
		chg->driver = NULL;
	}

	if (chg->encrypted_chg_enabled) {
		trusted_shash_release();
	}

	return 0;
}

static int glink_charger_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct glink_charger *chg = dev->devdata;

	if (!chg) {
		pr_err("glink charger is not ready\n");
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (chg->init_done)
				glink_charger_cfg(chg, &dev->cfg);
			break;
		case MMI_GLINK_STATE_DOWN:
			memset(&chg->chg_cfg, 0, sizeof(chg->chg_cfg));
			memset(&chg->constraint, 0, sizeof(chg->constraint));
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_PSY_USR:
		switch (notification) {
		case MMI_PSY_CHANGE_USB:
		case MMI_PSY_CHANGE_WLS:
			glink_charger_status_update(chg);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		glink_charger_status_update(chg);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

static int glink_charger_parse_dt(struct glink_charger *chg)
{
	int rc, i, len;
	u32 prev, val;
	u32 *thermal_levels;
	u32 max_chg_current_ua;
	const char *partner_name = NULL;
	struct device_node *node = chg->dev->node;

	chg->mmi_charger_disabled = of_property_read_bool(node,
				"mmi-charger-disabled");

	chg->encrypted_chg_enabled = of_property_read_bool(node,
				"encrypted-chg-enabled");

	rc = of_property_read_u32(node, "max-fcc-ma", &max_chg_current_ua);
	if (rc)
		max_chg_current_ua = 4000;
	max_chg_current_ua *= 1000;

	rc = of_property_read_string(node,
				"partner_psy_name",
				&partner_name);
	if (rc) {
		pr_warn("no available partner psy name\n");
	} else {
		chg->partner_charger = power_supply_get_by_name(partner_name);
		if (!chg->partner_charger)
			pr_err("no partner psy: %s\n", partner_name);
	}

	if (chg_tcd[TCD_TYPE_PRIMARY].thermal_levels ||
	    chg_tcd[TCD_TYPE_SECONDARY].thermal_levels) {
		return 0;
	}

	chg_tcd[TCD_TYPE_PRIMARY].type = -EINVAL;
	chg_tcd[TCD_TYPE_PRIMARY].num_thermal_levels = -EINVAL;
	chg_tcd[TCD_TYPE_SECONDARY].type = -EINVAL;
	chg_tcd[TCD_TYPE_SECONDARY].num_thermal_levels = -EINVAL;

	len = of_property_count_elems_of_size(node,
				"thermal-primary-mitigation",
				sizeof(u32));
	if (len <= 0) {
		pr_err("invalid primary thermal mitigation table\n");
		return 0;
	}

	prev = max_chg_current_ua;
	for (i = 0; i < len; i++) {
		rc = of_property_read_u32_index(node,
				"thermal-primary-mitigation",
				i, &val);
		if (rc < 0) {
			pr_err("invalid primary-mitigation[%d]\n", i);
			return rc;
		}
		pr_info("primary-mitigation[%d], val=%d, prev=%d\n",
				i, val, prev);
		if (val > prev) {
			pr_err("invalid primary levels order\n");
			return 0;
		}
		prev = val;
	}

	thermal_levels = devm_kzalloc(chg->dev->dev,
				sizeof(u32) * (len + 1), GFP_KERNEL);
	if (!thermal_levels)
		return -ENOMEM;

	rc = of_property_read_u32_array(node,
				"thermal-primary-mitigation",
				&thermal_levels[1], len);
	if (rc < 0) {
		devm_kfree(chg->dev->dev, thermal_levels);
		pr_err("read primary-mitigation, err=%d\n", rc);
		return rc;
	}

	chg->tcd = chg_tcd;
	thermal_levels[0] = thermal_levels[1];
	chg_tcd[TCD_TYPE_PRIMARY].type = TCD_TYPE_PRIMARY;
	chg_tcd[TCD_TYPE_PRIMARY].dev = chg->dev;
	chg_tcd[TCD_TYPE_PRIMARY].num_thermal_levels = len;
	chg_tcd[TCD_TYPE_PRIMARY].thermal_fcc_ua = max_chg_current_ua;
	chg_tcd[TCD_TYPE_PRIMARY].thermal_levels = thermal_levels;
	pr_info("thermal primary-limit parse done, levels=%d\n", len);

	len = of_property_count_elems_of_size(node,
				"thermal-secondary-mitigation",
				sizeof(u32));
	if (len <= 0) {
		pr_err("invalid secondary thermal mitigation table\n");
		return 0;
	}

	prev = max_chg_current_ua;
	for (i = 0; i < len; i++) {
		rc = of_property_read_u32_index(node,
				"thermal-secondary-mitigation",
				i, &val);
		if (rc < 0) {
			pr_err("invalid secondary-mitigation[%d]\n", i);
			return rc;
		}
		pr_info("secondary-mitigation[%d], val=%d, prev=%d\n",
				i, val, prev);
		if (val > prev) {
			pr_err("invalid secondary levels order\n");
			return 0;
		}
		prev = val;
	}

	thermal_levels = devm_kzalloc(chg->dev->dev,
				sizeof(u32) * (len + 1), GFP_KERNEL);
	if (!thermal_levels)
		return -ENOMEM;

	rc = of_property_read_u32_array(node,
				"thermal-secondary-mitigation",
				&thermal_levels[1], len);
	if (rc < 0) {
		devm_kfree(chg->dev->dev, thermal_levels);
		pr_err("read secondary-mitigation, err=%d\n", rc);
		return rc;
	}

	thermal_levels[0] = thermal_levels[1];
	chg_tcd[TCD_TYPE_SECONDARY].type = TCD_TYPE_SECONDARY;
	chg_tcd[TCD_TYPE_SECONDARY].dev = chg->dev;
	chg_tcd[TCD_TYPE_SECONDARY].num_thermal_levels = len;
	chg_tcd[TCD_TYPE_SECONDARY].thermal_fcc_ua = max_chg_current_ua;
	chg_tcd[TCD_TYPE_SECONDARY].thermal_levels = thermal_levels;
	pr_info("thermal secondary-limit parse done, levels=%d\n", len);

	return 0;
}

int glink_device_charger_setup(struct glink_dev *dev)
{
	int rc;
	struct glink_charger *chg = dev->devdata;

	if (chg) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	chg = devm_kzalloc(dev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->name = dev->name;

	rc = glink_charger_parse_dt(chg);
	if (rc) {
		pr_err("dt paser failed, rc=%d\n", rc);
		return rc;
	}

	dev->ops.init = glink_charger_init;
	dev->ops.notify = glink_charger_notify;
	dev->ops.deinit = glink_charger_deinit;
	dev->devdata = chg;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
