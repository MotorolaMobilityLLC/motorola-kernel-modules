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
#include "battery_glink.h"

static struct mmi_glink_chip *this_root_chip =  NULL;
static struct battery_glink_dev *this_batt_chip[BATT_NUM] = {NULL};

static enum power_supply_property batt_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

enum {
	NOTIFY_EVENT_TYPE_FLIP_CAPACITY = 0,
	NOTIFY_EVENT_TYPE_FLIP_VOLTAGE_NOW,
	NOTIFY_EVENT_TYPE_FLIP_TEMP,
	NOTIFY_EVENT_TYPE_FLIP_CYCLE_COUNT,
	NOTIFY_EVENT_TYPE_FLIP_CHARGE_FULL,
	NOTIFY_EVENT_TYPE_FLIP_SOH,
};

#define BPD_TEMP_THRE -3000
static int batt_psy_get_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 union power_supply_propval *pval)
{
	struct battery_glink_dev *batt_chip = power_supply_get_drvdata(psy);
	struct timespec64 glink_access_time_now;
	int rc = 0;

	if (!this_root_chip || !batt_chip)
		return -ENODEV;

	pval->intval = -ENODATA;

	switch (batt_chip->batt_role ) {
	case BATT_MAIN:
		ktime_get_real_ts64(&glink_access_time_now);
		batt_chip->elapsed_ms = (glink_access_time_now.tv_sec - batt_chip->glink_access_time.tv_sec) * 1000;
		batt_chip->elapsed_ms += (glink_access_time_now.tv_nsec - batt_chip->glink_access_time.tv_nsec) / 1000000;
		if (batt_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&batt_chip->glink_access_time);
			batt_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_MAIN_BATT_INFO,
				&batt_chip->batt_prop_info, sizeof(struct battery_info));
			mmi_dbg(this_root_chip, "battery_get_prop[%d], DEV_INFO", batt_chip->batt_role);
		}
		break;
	case BATT_FLIP:
		ktime_get_real_ts64(&glink_access_time_now);
		batt_chip->elapsed_ms = (glink_access_time_now.tv_sec - batt_chip->glink_access_time.tv_sec) * 1000;
		batt_chip->elapsed_ms += (glink_access_time_now.tv_nsec - batt_chip->glink_access_time.tv_nsec) / 1000000;
		if (batt_chip->elapsed_ms > 1000) {
			ktime_get_real_ts64(&batt_chip->glink_access_time);
			batt_chip->elapsed_ms = 0;
			rc = qti_charger_get_property(OEM_PROP_FLIP_BATT_INFO,
				&batt_chip->batt_prop_info, sizeof(struct battery_info));
			mmi_dbg(this_root_chip, "battery_get_prop[%d], DEV_INFO", batt_chip->batt_role);
		}
		break;
	default:
		mmi_err(this_root_chip, "battery_get_prop, Can not find correct balance role %d", batt_chip->batt_role);
		return rc;
		break;
	}

	if (rc) {
		mmi_err(this_root_chip, "batt_role %d, Get BATT_INFO property failed", batt_chip->batt_role);
		return rc;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = batt_chip->batt_prop_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = batt_chip->batt_prop_info.present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = batt_chip->batt_prop_info.batt_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = batt_chip->batt_prop_info.batt_ua;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = batt_chip->batt_prop_info.batt_soc / 100;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		pval->intval= POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = batt_chip->batt_prop_info.batt_temp / 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		pval->intval = batt_chip->batt_prop_info.batt_full_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		pval->intval = batt_chip->batt_prop_info.batt_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		pval->intval = batt_chip->batt_prop_info.batt_chg_counter;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pval->intval = batt_chip->batt_prop_info.batt_cycle;
		break;
	default:
		break;
	}

	return rc;
}

static int batt_psy_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	struct battery_glink_dev *batt_chip = power_supply_get_drvdata(psy);
	int rc = 0;

	if (!this_root_chip || !batt_chip)
		return -ENODEV;

	if (batt_chip->batt_role == BATT_MAIN) {
		mmi_err(this_root_chip, "Set BATT_MAIN, property %d", prop);
	} else if (batt_chip->batt_role == BATT_FLIP) {
		mmi_err(this_root_chip, "Set BATT_FLIP, property %d", prop);
	} else {
		mmi_err(this_root_chip, "batt_set_prop, Can not find correct batt role %d", batt_chip->batt_role);
		return rc;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		break;
	default:
		break;
	}
	return rc;
}

static int batt_psy_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc batt_psy_desc_main = {
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= batt_psy_get_prop,
	.set_property	= batt_psy_set_prop,
	.property_is_writeable = batt_psy_prop_is_writeable,
	.properties	= batt_psy_props,
	.num_properties	= ARRAY_SIZE(batt_psy_props),
};

static struct power_supply_desc batt_psy_desc_flip = {
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= batt_psy_get_prop,
	.set_property	= batt_psy_set_prop,
	.property_is_writeable = batt_psy_prop_is_writeable,
	.properties	= batt_psy_props,
	.num_properties	= ARRAY_SIZE(batt_psy_props),
};
const char *mmi_get_battery_serialnumber(BATT_ROLE batt_role)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *battsn_buf;
	int retval;

	battsn_buf = NULL;

	if (np) {

		if (batt_role == BATT_MAIN) {
			retval = of_property_read_string(np, "mmi,battid",
						&battsn_buf);
		} else {
			retval = of_property_read_string(np, "mmi,flip_battid",
						&battsn_buf);
		}

	} else
                return NULL;

        if ((retval == -EINVAL) || !battsn_buf) {
                pr_err("Battsn unused\n");
                of_node_put(np);
                return NULL;

        } else
                pr_err("Battsn = %s\n", battsn_buf);

        of_node_put(np);

        return battsn_buf;
}

#define BATT_DEFAULT_ID 107000
#define BATT_SN_UNKNOWN "unknown-sn"
static int battery_device_parse_dt(struct mmi_glink_chip *chip, struct battery_glink_dev *batt_chip)
{
	int rc, count, i;
	int profile_id = -EINVAL;
	const char *df_sn = NULL, *dev_sn = NULL;
	struct device_node *node;
	struct profile_sn_map {
		const char *id;
		const char *sn;
	} *map_table;

	const char *batt_df_serialnum[] = {
		"mmi,main-df-serialnum",
		"mmi,flip-df-serialnum"
	};

	const char *batt_ids_map[] = {
		"main-ids-map",
		"flip-ids-map"
	};

	node = chip->dev->of_node;
	dev_sn = mmi_get_battery_serialnumber(batt_chip->batt_role);

	if (!dev_sn) {

			rc = of_property_read_string(node, batt_df_serialnum[batt_chip->batt_role],
							&df_sn);
			if (!rc && df_sn) {
				mmi_info(chip, "Default Main Serial Number %s\n", df_sn);
			} else {
				mmi_err(chip, "No Default Serial Number defined\n");
				df_sn = BATT_SN_UNKNOWN;
			}
			strcpy(batt_chip->batt_sn, df_sn);

	} else {
			strcpy(batt_chip->batt_sn, dev_sn);
		}


	mmi_err(chip, "battery sn: %s, ids-map %s", batt_chip->batt_sn, batt_ids_map[batt_chip->batt_role]);

	count = of_property_count_strings(node, batt_ids_map[batt_chip->batt_role]);
	if (count <= 0 || (count % 2)) {
		mmi_err(chip, "Invalid profile-ids-map in DT, rc=%d\n", count);
		return -EINVAL;
	}

	map_table = devm_kmalloc_array(chip->dev, count / 2,
					sizeof(struct profile_sn_map),
					GFP_KERNEL);
	if (!map_table)
		return -ENOMEM;

	rc = of_property_read_string_array(node,  batt_ids_map[batt_chip->batt_role],
					(const char **)map_table,
					count);
	if (rc < 0) {
		mmi_err(chip, "Failed to get profile-ids-map, rc=%d\n", rc);
		batt_chip->batt_id = 0;
		goto free_map;
	}

	for (i = 0; i < count / 2 && map_table[i].sn; i++) {
		mmi_info(chip, "profile_ids_map[%d]: id=%s, sn=%s\n", i,
					map_table[i].id, map_table[i].sn);
		if (!strcmp(map_table[i].sn, batt_chip->batt_sn))
			profile_id = i;
	}

	if (profile_id >= 0 && profile_id < count / 2) {
		i = profile_id;
		profile_id = 0;
		rc = kstrtou32(map_table[i].id, 0, &profile_id);
		if (rc) {
			mmi_err(chip, "Invalid id: %s, sn: %s\n",
						map_table[i].id,
						map_table[i].sn);
		} else {
			mmi_info(chip, "profile id: %s(%d), sn: %s\n",
						map_table[i].id,
						profile_id,
						map_table[i].sn);
		}
	} else {
		mmi_warn(chip, "No matched profile id in profile-ids-map\n");
	}

	batt_chip->batt_id = profile_id;

	switch(batt_chip->batt_role) {
	case BATT_MAIN:
//		rc = qti_charger_set_property(OEM_PROP_MAIN_BATT_ID,
//					&batt_chip->batt_id, sizeof(batt_chip->batt_id));
		if (rc) {
			mmi_err(chip, "Failed to write main batt_id, rc=%d\n", rc);
			return rc;
		}
		mmi_info(chip, "Main battery, batt sn: %s, batt id %d\n",
						batt_chip->batt_sn,
						batt_chip->batt_id);
		break;
	case BATT_FLIP:
//		rc = qti_charger_set_property(OEM_PROP_FLIP_BATT_ID,
//					&batt_chip->batt_id, sizeof(batt_chip->batt_id));
		if (rc) {
			mmi_err(chip, "Failed to write flip batt_id, rc=%d\n", rc);
			return rc;
		}
		mmi_info(chip, "Flip battery, batt sn: %s, batt id %d\n",
						batt_chip->batt_sn,
						batt_chip->batt_id);
		break;
	default:
		break;
	}

free_map:
	devm_kfree(chip->dev, map_table);

	return rc;
}

typedef struct {
	const char *name;
	int value;
} uEnvpVar;

static void battery_notify_flip_uevent(struct battery_info *batt_info)
{
	int num_vars = 0, i = 0;
	char **uenvp_ext = NULL;
	char *uenvp_strings = NULL;
	struct battery_host *batt_host = NULL;
	struct power_supply *batt_psy = NULL;
	struct battery_glink_dev *batt_chip = NULL;

	if (!this_root_chip || !batt_info) {
		mmi_err(this_root_chip, "The data is illegal\n");
		return;
	}

	batt_chip = this_batt_chip[BATT_FLIP];
	batt_host = this_root_chip->batt_host;
	if (batt_host == NULL || batt_chip == NULL)
		return;

	batt_psy = batt_host->batt_psy;
	if (!batt_psy) {
		mmi_err(this_root_chip, "No battery supply found\n");
		return;
	}

	uEnvpVar uenvp_vars[] = {
		{"POWER_SUPPLY_FLIP_BATT_SOC", batt_info->batt_soc / 100},
		{"POWER_SUPPLY_FLIP_VOLTAGE_NOW", batt_info->batt_uv},
		{"POWER_SUPPLY_FLIP_TEMP", batt_info->batt_temp / 10},
		{"POWER_SUPPLY_FLIP_CYCLE_COUNT", batt_info->batt_cycle},
		{"POWER_SUPPLY_FLIP_CHARGE_FULL", batt_info->batt_full_uah},
		{"POWER_SUPPLY_FLIP_STATE_OF_HEALTH", batt_info->batt_soh}
	};

	num_vars = sizeof(uenvp_vars) / sizeof(uenvp_vars[0]);

	uenvp_ext = kmalloc((num_vars + 1) * sizeof(char *), GFP_KERNEL);
	if (!uenvp_ext) {
		mmi_err(this_root_chip, "Failed to kmalloc the uenvp_ext");
		return;
	}

	uenvp_strings = kmalloc(CHG_SHOW_MAX_SIZE * num_vars, GFP_KERNEL);
	if (!uenvp_strings) {
		mmi_err(this_root_chip, "Failed to kmalloc the event_string");
		kfree(uenvp_ext);
		return;
	}

	for (i = 0; i < num_vars; i++) {
		uenvp_ext[i] = uenvp_strings + i * CHG_SHOW_MAX_SIZE;
		scnprintf(uenvp_ext[i], CHG_SHOW_MAX_SIZE, "%s=%d", uenvp_vars[i].name, uenvp_vars[i].value);
	}
	uenvp_ext[num_vars] = NULL;
	kobject_uevent_env(&batt_psy->dev.kobj, KOBJ_CHANGE, uenvp_ext);

	if (uenvp_strings) {
		kfree(uenvp_strings);
	}

	if (uenvp_ext) {
		kfree(uenvp_ext);
	}

	return;
}

static void battery_notify_charger_uevent(BATT_ROLE batt_role, struct battery_info *batt_info)
{
      if (!this_batt_chip[batt_role])
		return;

	struct battery_info batt_info_save = this_batt_chip[batt_role]->batt_dev_info;

	if (batt_role == BATT_MAIN)
		return;

	if (batt_info->batt_soc != batt_info_save.batt_soc) {
		battery_notify_flip_uevent(batt_info);
	}

	return;
}

static int battery_notify_handler(struct notifier_block *nb, unsigned long event, void *data)
{
	int rc = -1;
	struct battery_info batt_info;
	struct battery_glink_dev *batt_chip =
			container_of(nb, struct battery_glink_dev, batt_nb);

	static int battid_set_flag = 0;

	if (!this_root_chip || !batt_chip)
		return -ENODEV;

	if ((battid_set_flag == 0) && (batt_chip->batt_role == BATT_MAIN))
	{
		rc = qti_charger_set_property(OEM_PROP_MAIN_BATT_ID,
					&batt_chip->batt_id, sizeof(batt_chip->batt_id));

		if (rc) {
			mmi_err(this_root_chip, "Failed to write main batt_id, rc=%d\n", rc);
		} else {
			battid_set_flag = 1;
			mmi_dbg(this_root_chip, "Main battery, batt sn: %s, batt id %d\n",
						batt_chip->batt_sn,
						batt_chip->batt_id);
		}
	}

	mmi_dbg(this_root_chip, "batt_role %d, notify-dev %ld", batt_chip->batt_role, event);
	if (event == DEV_BATT || event == DEV_ALL) {

		if (batt_chip->batt_role == BATT_MAIN) {
			rc = qti_charger_get_property(OEM_PROP_MAIN_BATT_INFO,
					&batt_info, sizeof(batt_info));
			if (rc) {
				mmi_err(this_root_chip, "Failed to read main_batt info, rc=%d\n", rc);
				return rc;
			}
			memcpy(&this_batt_chip[BATT_MAIN]->batt_dev_info, &batt_info, sizeof(batt_info));
		} else if (batt_chip->batt_role == BATT_FLIP) {
			rc = qti_charger_get_property(OEM_PROP_FLIP_BATT_INFO,
					&batt_info, sizeof(batt_info));
			if (rc) {
				mmi_err(this_root_chip, "Failed to read flip_batt info, rc=%d\n", rc);
				return rc;
			}
			battery_notify_charger_uevent(BATT_FLIP, &batt_info);
			memcpy(&this_batt_chip[BATT_FLIP]->batt_dev_info, &batt_info, sizeof(batt_info));
		} else {
			mmi_err(this_root_chip, "batt_get_prop, Can not find correct batt role %d", batt_chip->batt_role);
			return rc;
		}

		mmi_info(this_root_chip, "Batt_dev[%d]: batt_uv %d, batt_ua %d, "
							"batt_soc %d, batt_temp %d, batt_status %d, batt_soh %d",
							batt_chip->batt_role, batt_info.batt_uv, batt_info.batt_ua, batt_info.batt_soc,
							batt_info.batt_temp, batt_info.batt_status, batt_info.batt_soh);

		mmi_info(this_root_chip, "Batt_dev[%d]: batt_cycle %d, batt_full_uah %d, "
							"batt_design_uah %d, batt_chg_counter %d, batt_fv_uv %d, batt_fcc_ua %d",
							batt_chip->batt_role, batt_info.batt_cycle, batt_info.batt_full_uah, batt_info.batt_design_uah,
							batt_info.batt_chg_counter, batt_info.batt_fv_uv, batt_info.batt_fcc_ua);
	}

	return NOTIFY_DONE;
}

struct glink_device *battery_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{

	struct battery_glink_dev *batt_chip = NULL;
	struct glink_device * glink_dev = NULL;
	struct power_supply_config psy_cfg = {};
	int rc = 0;

	if (!chip)
		goto exit;
	this_root_chip = chip;

	batt_chip = kzalloc(sizeof(struct battery_glink_dev),GFP_KERNEL);
	if (!batt_chip)
		goto exit;

	batt_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	batt_chip->batt_role = dev_dts->dev_role;
	psy_cfg.drv_data = batt_chip;

	if (batt_chip->batt_role == BATT_MAIN) {
		mmi_err(chip, "battery[%d] glink device psy", batt_chip->batt_role);
		batt_psy_desc_main.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		batt_chip->batt_dev_psy = devm_power_supply_register(chip->dev,
							  &batt_psy_desc_main,
							  &psy_cfg);
		if (IS_ERR(batt_chip->batt_dev_psy)) {
			mmi_err(chip, "Couldn't register batt dev power supply");
			goto exit;
		}
	} else if (batt_chip->batt_role == BATT_FLIP) {
		mmi_err(chip, "battery[%d] glink device psy", batt_chip->batt_role);
		batt_psy_desc_flip.name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->psy_name);
		batt_chip->batt_dev_psy = devm_power_supply_register(chip->dev,
							  &batt_psy_desc_flip,
							  &psy_cfg);
		if (IS_ERR(batt_chip->batt_dev_psy)) {
			mmi_err(chip, "Couldn't register batt dev power supply");
			goto exit;
		}
	} else {
		mmi_err(chip, "Couldn't register batt role %d, Failed register battery glink", batt_chip->batt_role);
		goto exit;
	}

	rc = battery_device_parse_dt(chip, batt_chip);
	if (rc) {
		mmi_err(chip, "Parse battery %d device dts failed", batt_chip->batt_role);
	}

	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_BATT, batt_chip);
	if (!glink_dev)
		goto exit;
	batt_chip->glink_dev = glink_dev;
	batt_chip->batt_nb.notifier_call = battery_notify_handler;
	mmi_glink_register_notifier(&batt_chip->batt_nb);

	this_batt_chip[batt_chip->batt_role] = batt_chip;
	mmi_err(chip, "battery glink device %s register successfully", dev_dts->glink_dev_name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void battery_glink_device_unregister(void)
{
	int i = 0;
	for (i = 0; i < BATT_NUM; i++) {
		if (!this_batt_chip[i])
			return;
		mmi_glink_unregister_notifier(&this_batt_chip[i]->batt_nb);
		power_supply_unregister(this_batt_chip[i]->batt_dev_psy);
		glink_device_unregister(this_batt_chip[i]->glink_dev);
		kfree(this_batt_chip[i]);
		mmi_err(this_root_chip, "battery_glink_device_unregister[%d] success", i);
	}
	mmi_err(this_root_chip, "battery_glink_device_unregister success");
}
