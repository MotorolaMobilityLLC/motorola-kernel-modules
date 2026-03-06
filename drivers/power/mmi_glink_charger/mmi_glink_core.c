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

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/mmi_wake_lock.h>
#include <soc/qcom/mmi_boot_info.h>
#include <linux/time64.h>
#include <linux/power/bm_adsp_ulog.h>

#include <linux/power/qti_glink_charger_v2.h>
#include "mmi_glink_core.h"
#include "device_class.h"
#include "battery_host.h"
#include "./device_glink/battery_glink.h"
#include "./device_glink/balance_charge_glink.h"
#include "./device_glink/wireless_charge_glink.h"
#include "./device_glink/switch_buck_glink.h"
#include "./device_glink/charge_pump_glink.h"
#include "./device_glink/usb_glink.h"
#include "trusted_shash_lib.h"
#define HYST_STEP_MV 50
#define DEMO_MODE_HYS_SOC 5
#define WARM_TEMP 45
#define COOL_TEMP 0

#define RADIO_MAX_LEN 33
#define BATT_PAIR_ID_BITS 16
#define BATT_PAIR_ID_MASK ((1 << BATT_PAIR_ID_BITS) - 1)

#define HEARTBEAT_DELAY_MS 60000
#define HEARTBEAT_FACTORY_MS 5000
#define HEARTBEAT_DISCHARGE_MS 100000
#define HEARTBEAT_WAKEUP_INTRVAL_NS 70000000000
#define OEM_BM_ULOG_SIZE		4096
#define ULOG_DURATION_MS		60000

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for mmi glink charger driver");

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
static int suspend_wakeups;
module_param(suspend_wakeups, int, 0644);

static bool shutdown_triggered = false;

static struct mmi_glink_chip *this_chip = NULL;
static struct mmi_glink_dev_dts_info *glink_dev_dts_list = NULL;

void moto_reboot_call_notifier(char *call_name);

enum {
	MMI_POWER_SUPPLY_CHARGE_RATE_NONE = 0,
	MMI_POWER_SUPPLY_CHARGE_RATE_NORMAL,
	MMI_POWER_SUPPLY_CHARGE_RATE_WEAK,
	MMI_POWER_SUPPLY_CHARGE_RATE_TURBO,
	MMI_POWER_SUPPLY_CHARGE_RATE_TURBO_30W,
	MMI_POWER_SUPPLY_CHARGE_RATE_HYPER,
};

enum {
	NOTIFY_EVENT_TYPE_CHG_RATE = 0,
	NOTIFY_EVENT_TYPE_VBUS_PRESENT,
	NOTIFY_EVENT_TYPE_POWER_WATT,
	NOTIFY_EVENT_TYPE_POWER_WATT_DESIGN,
	NOTIFY_EVENT_TYPE_CHG_REAL_TYPE,
	NOTIFY_EVENT_TYPE_BATTERY_SOH,
};

static char *charge_rate[] = {
	"None", "Normal", "Weak", "Turbo", "Turbo_30W", "Hyper"
};

#define IS_SUSPENDED BIT(0)
#define WAS_SUSPENDED BIT(1)

static int mmi_vote(struct mmi_vote *vote, const char *voter,
				bool enabled, int value)
{
	int i;
	int empty = -EINVAL;
	struct mmi_glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	if (!voter) {
		mmi_err(chip, "%s: Invalid voter\n", vote->name);
		return -EINVAL;
	}

	for (i = 0; i < MMI_VOTE_NUM_MAX; i++) {
		if (vote->voters[i] &&
		    !strcmp(vote->voters[i], voter)) {
			break;
		} else if (empty < 0 && !vote->voters[i]) {
			empty = i;
		}
	}

	if (i < MMI_VOTE_NUM_MAX) {
		if (enabled) {
			vote->voters[i] = voter;
			vote->votes[i] = value;
		} else {
			vote->voters[i] = NULL;
			vote->votes[i] = 0;
		}
	} else {
		if (!enabled) {
			return 0;
		} else if (empty < 0) {
			mmi_err(this_chip, "No entry found\n");
			return -ENOENT;
		} else {
			vote->voters[empty] = voter;
			vote->votes[empty] = value;
		}
	}

	mmi_info(chip, "%s:%s voter: en:%d, val:%d\n",
			vote->name, voter, enabled, value);

	return 0;
}

static int mmi_get_effective_voter(struct mmi_vote *vote)
{
	int i;
	int win_voter = -EINVAL;
	int win_vote = INT_MAX;

	for (i = 0; i < MMI_VOTE_NUM_MAX; i++) {
		if (vote->voters[i] &&
		    vote->votes[i] < win_vote) {
			win_voter = i;
			win_vote = vote->votes[i];
		}
	}

	return win_voter;
}

static void mmi_notify_charger_event(struct mmi_glink_chip *chip, int type);
static ssize_t state_sync_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(this_chip, "Invalid state_sync value = %lu\n", mode);
		return -EINVAL;
	}

	if (mode) {
		mutex_lock(&this_chip->charger_lock);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_CHG_RATE);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_VBUS_PRESENT);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_POWER_WATT);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_POWER_WATT_DESIGN);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_CHG_REAL_TYPE);
		mmi_glink_notifier(DEV_USB, this_chip);
		mutex_unlock(&this_chip->charger_lock);
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
					msecs_to_jiffies(0));
		mmi_info(this_chip, "charger state sync received\n");
	}

	return count;
}
static DEVICE_ATTR(state_sync, 0200, NULL, state_sync_store);

#define CHARGER_POWER_5W 5000
#define CHARGER_POWER_7P5W 7500
#define CHARGER_POWER_10W 10000
#define CHARGER_POWER_15W 15000
#define CHARGER_POWER_18W 18000
#define CHARGER_POWER_20W 20000
#define CHARGER_POWER_MAX 100000
static ssize_t dcp_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid dcp pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->dcp_pmax != pmax &&
	    (pmax >= CHARGER_POWER_5W && pmax <= CHARGER_POWER_10W)) {
		this_chip->dcp_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t dcp_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->dcp_pmax);
}

static DEVICE_ATTR(dcp_pmax, 0644,
		dcp_pmax_show,
		dcp_pmax_store);

static ssize_t hvdcp_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid hvdcp pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->hvdcp_pmax != pmax &&
	    (pmax >= CHARGER_POWER_7P5W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->hvdcp_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t hvdcp_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->hvdcp_pmax);
}

static DEVICE_ATTR(hvdcp_pmax, 0644,
		hvdcp_pmax_show,
		hvdcp_pmax_store);

static ssize_t pd_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid pd pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->pd_pmax != pmax &&
	    (pmax >= CHARGER_POWER_15W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->pd_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t pd_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->pd_pmax);
}

static DEVICE_ATTR(pd_pmax, 0644,
		pd_pmax_show,
		pd_pmax_store);

static ssize_t wls_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid wireless pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->wls_pmax != pmax &&
	    (pmax >= CHARGER_POWER_5W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->wls_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t wls_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->wls_pmax);
}

static DEVICE_ATTR(wls_pmax, 0644,
		wls_pmax_show,
		wls_pmax_store);

static void qti_encrypt_authentication(struct mmi_glink_chip *chip)
{
	int i;
	TRUSTED_SHASH_RESULT trusted_result;
	struct encrypted_data send_data;
	u8 random_num[4] = {0};

	memset(&send_data, 0, sizeof(send_data));
	for (i = 0;i < 4;i++) {
		get_random_bytes(&(random_num[i]),sizeof(*random_num));
		trusted_result.random_num[i] = random_num[i] % 26 + 'a';
		send_data.random_num[i] = trusted_result.random_num[i];
		mmi_info(chip, "encrypt random_num1[%d]: %d, 0x%x \n", i, send_data.random_num[i], send_data.random_num[i]);
	}

	trusted_sha1(trusted_result.random_num, 4, trusted_result.sha1);
	trusted_hmac(trusted_result.random_num, 4, trusted_result.hmac_sha256);

	for (i = 0;i < 4;i++) {
	    send_data.hmac_data[i] = trusted_result.hmac_sha256[3 + (4 * i)] + (trusted_result.hmac_sha256[2 + (4 * i)] << 8) +
	            (trusted_result.hmac_sha256[1 + (4 * i)] << 16) + (trusted_result.hmac_sha256[0 + (4 * i)] << 24);
	    mmi_info(chip, "encrypt hmac_sha256[%d]: 0x%x \n", i, send_data.hmac_data[i]);

	}

	for (i = 0;i < 4;i++) {
	    send_data.sha1_data[i] = trusted_result.sha1[3 + (4 * i)] + (trusted_result.sha1[2 + (4 * i)] << 8) +
	            (trusted_result.sha1[1 + (4 * i)] << 16) + (trusted_result.sha1[0 + (4 * i)] << 24);
	    mmi_info(chip, "encrypt hmac_sha1[%d]: 0x%x \n", i, send_data.sha1_data[i]);
	}

	qti_charger_set_property(OEM_PROP_ENCRYT_DATA,
				&send_data,
				sizeof(struct encrypted_data));

}

static int mmi_get_cur_thermal_level(struct mmi_glink_chip *chip, int *val)
{
	struct battery_host *batt_host = chip->batt_host;
	union power_supply_propval prop;
	int ret;
	if (!batt_host->batt_psy) {
		mmi_err(chip, "No battery supply found\n");
		return -ENODEV;
	}

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &prop);
	if (!ret)
		*val = prop.intval;

	return ret;
}

static void mmi_notify_charger_event(struct mmi_glink_chip *chip, int type)
{
	char *event_string = NULL;
	struct battery_host *batt_host = chip->batt_host;

	if (!batt_host->batt_psy) {
		battery_supply_init(batt_host);
		if (!batt_host->batt_psy)
			return;
	}

	event_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (event_string) {
		switch (type) {
		case NOTIFY_EVENT_TYPE_CHG_RATE:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_CHARGE_RATE=%s",
				charge_rate[chip->max_charger_rate]);
			break;
		case NOTIFY_EVENT_TYPE_VBUS_PRESENT:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_VBUS_PRESENT=%s",
				chip->vbus_present? "true" : "false");
			break;
		case NOTIFY_EVENT_TYPE_POWER_WATT:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_POWER_WATT=%d",
				chip->power_watt / 1000);
			break;
		case NOTIFY_EVENT_TYPE_POWER_WATT_DESIGN:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_POWER_WATT_DESIGN=%d",
				chip->pd_pmax / 1000);
			break;
		case NOTIFY_EVENT_TYPE_CHG_REAL_TYPE:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_CHARGE_REAL_TYPE=%d",
				chip->real_charger_type);
			sysfs_notify(&batt_host->batt_psy->dev.kobj, NULL, "charge_real_type");
			break;
		case NOTIFY_EVENT_TYPE_BATTERY_SOH:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_STATE_OF_HEALTH=%d",
				chip->state_of_health);
			break;
		default:
			mmi_err(chip, "Invalid notify event type %d\n", type);
			kfree(event_string);
			return;
		}

		if (chip->batt_uenvp[0])
			kfree(chip->batt_uenvp[0]);
		chip->batt_uenvp[0] = event_string;
		chip->batt_uenvp[1] = NULL;
		kobject_uevent_env(&batt_host->batt_psy->dev.kobj,
					KOBJ_CHANGE,
					chip->batt_uenvp);
	}
}

#define WEAK_CHRG_THRSH_MW 2500
#define TURBO_CHRG_THRSH_MW 12500
#define TURBO_CHRG_30W_THRSH_MW 25000
#define HYPER_CHRG_THRSH_MW 40000
int mmi_get_battery_charger_rate(struct mmi_glink_chip *charger)
{
	int rate;
	struct mmi_charger_info *info;

	info = &charger->charger_info;
	if (!info->chrg_present)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	else if (info->chrg_pmax_mw < WEAK_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_WEAK;
	else if (info->chrg_pmax_mw < TURBO_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_NORMAL;
	else if (info->chrg_pmax_mw < TURBO_CHRG_30W_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_TURBO;
	else if (info->chrg_pmax_mw < HYPER_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_TURBO_30W;
	else
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_HYPER;

	return rate;
}

static void mmi_update_charger_event(struct mmi_glink_chip *chip)
{
	bool mmi_changed = false;
	int charger_rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	static int max_charger_rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	//int charger_type = 0;
	int real_charger_type = 0;
	bool vbus_present = false;
	int power_watt = 0;

	charger_rate = mmi_get_battery_charger_rate(chip);
	power_watt = chip->charger_info.chrg_pmax_mw;
	real_charger_type =  chip->charger_info.chrg_type;
	vbus_present = chip->charger_info.usb_online;

	if (max_charger_rate < charger_rate || charger_rate == MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		max_charger_rate = charger_rate;

	if (chip->max_charger_rate != max_charger_rate) {
		mmi_changed = true;
		chip->max_charger_rate = max_charger_rate;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_CHG_RATE);
		mmi_err(chip, "%s charger is detected\n",
			charge_rate[chip->max_charger_rate]);
	}

	if (chip->vbus_present != vbus_present) {
		mmi_changed = true;
		chip->vbus_present = vbus_present;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_VBUS_PRESENT);
		mmi_info(chip, "vbus is %s\n",
			vbus_present? "present" : "absent");
	}

	if ((chip->power_watt / 1000) != (power_watt / 1000)) {
		mmi_changed = true;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_POWER_WATT_DESIGN);
		chip->power_watt = power_watt;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_POWER_WATT);
		mmi_info(chip, "charger power is %d mW\n", power_watt);
	}

	if ((chip->real_charger_type) != real_charger_type) {
		mmi_changed = true;
		chip->real_charger_type = real_charger_type;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_CHG_REAL_TYPE);
		mmi_info(chip, "charger real type is %d\n", real_charger_type);
	}

	if (mmi_changed) {
		mmi_info(chip, "charger_rate %s, vbus %d, power_watt %d, charger real type %d\n",
			charge_rate[chip->max_charger_rate], chip->vbus_present, chip->power_watt, chip->real_charger_type);
	}
}

#define VBUS_MIN_MV			4000
static void mmi_update_battery_status(struct mmi_glink_chip *chip)
{
	struct battery_host *batt_host = chip->batt_host;
	struct battery_info *batt_info = &chip->battery_info;
	struct battery_info qti_batt_info;
	struct mmi_charger_status *chrg_status = &chip->charger_status;
	union power_supply_propval prop;
	int ret = 0;
	if (!batt_host->batt_psy) {
		mmi_err(chip, "No battery supply found\n");
		return;
	}

	qti_charger_get_property(OEM_PROP_BATT_INFO,
				&qti_batt_info,
				sizeof(struct battery_info));

	batt_info->batt_soh = qti_batt_info.batt_soh;
	batt_host->state_of_health = qti_batt_info.batt_soh;
	if (chip->state_of_health != qti_batt_info.batt_soh) {
		chip->state_of_health = qti_batt_info.batt_soh;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_BATTERY_SOH);
		mmi_info(chip, "state_of_health %d\n", chip->state_of_health);
	}

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_STATUS, &prop);
	if (!ret)
		batt_info->batt_status = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!ret)
		batt_info->batt_uv = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!ret)
		batt_info->batt_ua = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (!ret)
		batt_info->batt_soc = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_TEMP, &prop);
	if (!ret)
		batt_info->batt_temp = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL, &prop);
	if (!ret)
		batt_info->batt_full_uah = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
	if (!ret)
		batt_info->batt_design_uah = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_COUNTER, &prop);
	if (!ret)
		batt_info->batt_chg_counter = prop.intval;

	ret = power_supply_get_property(batt_host->batt_psy,
		POWER_SUPPLY_PROP_CYCLE_COUNT, &prop);
	if (!ret)
		batt_info->batt_cycle = prop.intval;

	if (batt_info->batt_soh > 0) {
		batt_host->age = batt_info->batt_soh;
	} else if (batt_info->batt_full_uah > 0 && batt_info->batt_design_uah > 0) {
		batt_host->age = batt_info->batt_full_uah * 100 / batt_info->batt_design_uah;
	} else if (batt_host->age == 0) {
		batt_host->age = -1;
	}

	if (chip->max_chrg_temp > 0 && batt_info->batt_temp >= (chip->max_chrg_temp * 10))
		chrg_status->pres_temp_zone = ZONE_HOT;
	else
		chrg_status->pres_temp_zone = ZONE_FIRST;

	mmi_info(chip, "batt_mv %d, batt_ma %d, batt_soc %d, batt_temp %d, batt_status %d, batt_soh %d "
		"batt_full_mah %d, batt_design_mah %d, batt_chg_counter %d, batt_cycle %d, init_cycles %d, batt_num %d, max_chrg_temp %d",
		batt_info->batt_uv / 1000, batt_info->batt_ua / 1000, batt_info->batt_soc, batt_info->batt_temp, batt_info->batt_status, batt_info->batt_soh,
		batt_info->batt_full_uah / 1000, batt_info->batt_design_uah / 1000, batt_info->batt_chg_counter,
		batt_info->batt_cycle, batt_host->init_cycles, batt_host->batt_dev_num, chip->max_chrg_temp);

	return;
}

static void mmi_get_charger_info(struct mmi_glink_chip *chip)
{
	int rc;
	struct mmi_charger_info *charger_info = NULL;
	struct mmi_charger_info charger_info_update;
	struct mmi_pmic_info *pmic_info = NULL;
	struct mmi_pmic_info pmic_info_update;
	struct battery_host *batt_host;
	int thermal_level = 0;

	if (!chip)
		return;

	charger_info = &chip->charger_info;
	pmic_info = &chip->pmic_info;
	batt_host = chip->batt_host;

	mmi_get_cur_thermal_level(chip, &thermal_level);

	rc = qti_charger_get_property(OEM_PROP_CHG_INFO,
				&charger_info_update,
				sizeof(struct mmi_charger_info));
	if (rc)
		return;

	charger_info->chrg_uv = charger_info_update.chrg_uv;
	charger_info->chrg_ua = charger_info_update.chrg_ua;
	charger_info->chrg_type = charger_info_update.chrg_type;
	charger_info->chrg_pmax_mw = charger_info_update.chrg_pmax_mw;
	charger_info->usb_online = charger_info_update.usb_online;
	charger_info->wls_online = charger_info_update.wls_online;
	charger_info->wls_tx_enabled = charger_info_update.wls_tx_enabled;
	charger_info->icm_sm_st = charger_info_update.icm_sm_st;
	charger_info->chrg_otg_enabled = charger_info_update.chrg_otg_enabled;
	charger_info->chrg_stat = charger_info_update.chrg_stat;

	if (charger_info->chrg_present != charger_info_update.chrg_present && !charger_info_update.chrg_present) {
		qti_encrypt_authentication(chip);
	}

	if (chip->charger_present_dynamic_control_bm_ulog && !bm_ulog_is_enabled_by_cmd()) {
		if ((chip->charger_info.usb_online || chip->charger_info.wls_online) &&
		    !chip->bm_ulog_enabled) {
			// bm_ulog_enable_log(true, 1000);
			bm_ulog_set_dynamic_debug_enabled(true);
			chip->bm_ulog_enabled = true;
			//mmi_info(chip, "enable adsp log during chg present!\n");
		}else if(!chip->charger_info.usb_online && !chip->charger_info.wls_online && chip->bm_ulog_enabled) {
			// bm_ulog_enable_log(false, 0);
			bm_ulog_set_dynamic_debug_enabled(false);
			chip->bm_ulog_enabled = false;
			//mmi_info(chip, "disable adsp log during chg not present!\n");
		}
	} else if (charger_info_update.chrg_present && !bm_ulog_is_enabled_by_cmd()) {
		if (charger_info->chrg_present != charger_info_update.chrg_present) {
			bm_ulog_enable_log(true, ULOG_DURATION_MS);
		}
	}

	charger_info->chrg_present = charger_info_update.chrg_present;
	if (!charger_info_update.chrg_present && charger_info_update.chrg_type != 0)
		charger_info->chrg_present = 1;

	if (batt_host->num_thermal_primary_levels > 0) {
		mmi_info(chip, "Thermal: primary_limit_level = %d, primary_fcc_ma = %d, secondary_limit_level = %d, thermal_secondary_fcc_ma = %d",
					batt_host->curr_thermal_primary_level, batt_host->thermal_primary_fcc_ua,
					batt_host->curr_thermal_secondary_level, batt_host->thermal_secondary_fcc_ua);
	}

	mmi_info(chip, "chrg_present %d, chrg_type %d, chrg_pmax_mw %d,"
		" chrg_uv %d, chrg_ua %d, usb_in %d, wls_in %d, wls_tx %d, chrg_otg_enabled %d, thermal_level %d, icm_sm_st %d, chrg_stat %d\n",
		charger_info->chrg_present,
		charger_info->chrg_type,
		charger_info->chrg_pmax_mw,
		charger_info->chrg_uv,
		charger_info->chrg_ua,
		charger_info->usb_online,
		charger_info->wls_online,
		charger_info->wls_tx_enabled,
		charger_info->chrg_otg_enabled,
		thermal_level,
		charger_info->icm_sm_st,
		charger_info->chrg_stat);

	rc = qti_charger_get_property(OEM_PROP_PMIC_INFO,
				&pmic_info_update,
				sizeof(struct mmi_pmic_info));
	if (rc)
		return;

	pmic_info->pmic_vbatt_uv = pmic_info_update.pmic_vbatt_uv;
	pmic_info->pmic_ibatt_ua = pmic_info_update.pmic_ibatt_ua;
	pmic_info->aicl_result_ma = pmic_info_update.aicl_result_ma;
	pmic_info->icl_result_ma = pmic_info_update.icl_result_ma;
	pmic_info->vfloat_mv = pmic_info_update.vfloat_mv;
	pmic_info->pmic_suspend_st = pmic_info_update.pmic_suspend_st;
	pmic_info->pmic_vph_uv = pmic_info_update.pmic_vph_uv;

	mmi_info(chip, "pmic_vph_mv %d, pmic_vbatt_mv %d, pmic_ibatt_ma %d, pmic_suspend_st %d, vfloat_mv %d, aicl_result_ma %d, icl_result_ma %d\n",
		pmic_info->pmic_vph_uv / 1000,
		pmic_info->pmic_vbatt_uv / 1000,
		pmic_info->pmic_ibatt_ua / 1000,
		pmic_info->pmic_suspend_st,
		pmic_info->vfloat_mv,
		pmic_info->aicl_result_ma,
		pmic_info->icl_result_ma);
}

#define TAPER_COUNT 2
static bool mmi_charger_is_charge_tapered(struct mmi_glink_chip *chip, int tapered_ma)
{
	bool is_tapered = false;
	struct battery_info *batt_info = &chip->battery_info;
	static int chrg_taper_cnt = 0;

	if (abs(batt_info->batt_ua / 1000) <= tapered_ma) {
		if (chrg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chrg_taper_cnt = 0;
		} else
			chrg_taper_cnt++;
	} else
		chrg_taper_cnt = 0;

	return is_tapered;
}

static void mmi_update_charger_status(struct mmi_glink_chip *chip)
{
	enum charging_limit_modes charging_limit_modes;
	struct mmi_charger_status *status = &chip->charger_status;
	struct mmi_charger_info *charger_info = &chip->charger_info;
	struct battery_info *batt_info = &chip->battery_info;
	struct battery_host *batt_host = chip->batt_host;
	bool voltage_full;
	int demo_fv_mv = 0, batt_mv = 0, chrg_iterm_ma = 0;

	demo_fv_mv = batt_host->demo_fv_mv;
	chrg_iterm_ma = batt_host->chrg_iterm_ma;
	batt_mv = batt_info->batt_uv / 1000;

	if (chip->enable_charging_limit && chip->factory_version) {
		charging_limit_modes = status->charging_limit_modes;
		if ((charging_limit_modes != CHARGING_LIMIT_RUN)
		    && (batt_info->batt_soc >= chip->upper_limit_capacity)
		    && (batt_mv >= (chip->upper_limit_en_mv)))
			charging_limit_modes = CHARGING_LIMIT_RUN;
		else if ((charging_limit_modes != CHARGING_LIMIT_OFF)
			   && (batt_info->batt_soc <= chip->lower_limit_capacity))
			charging_limit_modes = CHARGING_LIMIT_OFF;

		if (charging_limit_modes != status->charging_limit_modes) {
			status->charging_limit_modes = charging_limit_modes;
			if (status->charging_limit_modes == CHARGING_LIMIT_RUN)
				mmi_warn(chip, "Factory Mode/Image so Limiting Charging!!!\n");
		}
	}

	if (!charger_info->chrg_present) {
		status->pres_chrg_step = STEP_NONE;
	} else if (batt_info->batt_status == POWER_SUPPLY_STATUS_FULL) {
		status->pres_chrg_step = STEP_FULL;
	} else if (status->charging_limit_modes == CHARGING_LIMIT_RUN ||
			status->pres_temp_zone == ZONE_HOT ||
			status->pres_temp_zone == ZONE_COLD) {
		status->pres_chrg_step = STEP_STOP;

	} else if (chip->demo_mode) { /* Demo Mode */
		status->pres_chrg_step = STEP_DEMO;
		voltage_full = ((status->demo_chrg_suspend == false) &&
		    ((batt_mv + HYST_STEP_MV) >= demo_fv_mv) &&
			mmi_charger_is_charge_tapered(chip, chrg_iterm_ma));

		if ((status->demo_chrg_suspend == false) &&
		    ((batt_info->batt_soc >= chip->demo_mode) || voltage_full)) {
			status->demo_full_soc = batt_info->batt_soc;
			status->demo_chrg_suspend = true;
		} else if (status->demo_chrg_suspend == true &&
		    (batt_info->batt_soc <= (status->demo_full_soc - DEMO_MODE_HYS_SOC))) {
			status->demo_chrg_suspend = false;
		}
		mmi_info(chip, "voltage_full %d, batt_mv %d, demo_mode %d, demo_full_soc %d, batt_soc %d",
			voltage_full, batt_mv, chip->demo_mode, status->demo_full_soc, batt_info->batt_soc);
	} else {
		status->pres_chrg_step = STEP_NORM;
	}

	mmi_info(chip, "StepChg: %s, TempZone: %d, LimitMode: %d, DemoSuspend: %d\n",
		stepchg_str[(int)status->pres_chrg_step],
		status->pres_temp_zone,
		status->charging_limit_modes,
		status->demo_chrg_suspend);

}

static void mmi_charger_set_constraint(struct mmi_glink_chip *chip)
{
	int rc;
	u32 value;

	if (chip->demo_mode != chip->charger_constraint.demo_mode) {
		value = chip->demo_mode;
		rc = qti_charger_set_property(OEM_PROP_DEMO_MODE,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.demo_mode = chip->demo_mode;
	}

	if (chip->dcp_pmax != chip->charger_constraint.dcp_pmax) {
		value = chip->dcp_pmax;
		rc = qti_charger_set_property(OEM_PROP_CHG_BC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.dcp_pmax = chip->dcp_pmax;
	}

	if (chip->hvdcp_pmax != chip->charger_constraint.hvdcp_pmax) {
		value = chip->hvdcp_pmax;
		rc = qti_charger_set_property(OEM_PROP_CHG_QC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.hvdcp_pmax = chip->hvdcp_pmax;
	}

	if (chip->pd_pmax != chip->charger_constraint.pd_pmax) {
		value = chip->pd_pmax;
		rc = qti_charger_set_property(OEM_PROP_CHG_PD_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.pd_pmax = chip->pd_pmax;
	}

	if (chip->wls_pmax != chip->charger_constraint.wls_pmax) {
		value = chip->wls_pmax;
		qti_charger_set_property(OEM_PROP_CHG_WLS_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.wls_pmax = chip->wls_pmax;
	}

	if (chip->factory_version != chip->charger_constraint.factory_version) {
		value = chip->factory_version;
		qti_charger_set_property(OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
		if (!rc)
			chip->charger_constraint.factory_version = chip->factory_version;
	}
}

static void mmi_configure_charger(struct mmi_glink_chip *chip)
{
	struct mmi_charger_status *status = &chip->charger_status;
	bool charging_disable = false, charger_suspend = false, charging_full = false;
	bool pre_charging_disable = false, pre_charger_suspend = false;
	u32 value;

	pre_charging_disable = chip->charging_disable;
	pre_charger_suspend = chip->charger_suspend;

	switch (status->pres_chrg_step) {
	case STEP_NONE:
		break;
	case STEP_NORM:
		break;
	case STEP_FULL:
		charging_full = true;
		break;
	case STEP_DEMO:
		if (status->demo_chrg_suspend)
			charger_suspend = true;
		break;
	case STEP_STOP:
		charging_disable =  true;
		break;
	default:
		break;
	}

	if (charging_disable ||
		mmi_get_effective_voter(&chip->disable_charging_vote) >= 0)
		chip->charging_disable = true;
	else
		chip->charging_disable = false;

	if (chip->factory_mode) {
		chip->charging_disable = true;
	}

	if (chip->force_charger_disabled ||
	    charger_suspend ||
	    mmi_get_effective_voter(&chip->suspend_charger_vote) >= 0)
		chip->charger_suspend = true;
	else
		chip->charger_suspend = false;

	if (chip->force_charging_enabled) {
		chip->charging_disable = false;
	}

	if (pre_charger_suspend != chip->charger_suspend) {
		value = chip->charger_suspend;
		if(qti_charger_set_property(OEM_PROP_CHG_SUSPEND,
					&value,
					sizeof(value)) < 0) {
		    chip->charger_suspend = pre_charger_suspend;
        }
	}

	if (pre_charging_disable!= chip->charging_disable) {
		value = chip->charging_disable;
		if(qti_charger_set_property(OEM_PROP_CHG_DISABLE,
					&value,
					sizeof(value)) < 0) {
		    chip->charging_disable = pre_charging_disable;
        }
	}

	mmi_info(chip, "CDIS=%d, CSUS=%d, CFULL=%d\n",
		chip->charging_disable,
		chip->charger_suspend,
		charging_full);

	return;
}

static void mmi_charger_heartbeat_work(struct work_struct *work)
{
	int hb_resch_time;
	struct mmi_glink_chip *chip = container_of(work,
						struct mmi_glink_chip,
						heartbeat_work.work);
	struct timespec64 now;
	static struct timespec64 start;
	uint32_t elapsed_ms;

	if (!chip) {
		pr_err("heartbeat: called before chip valid!\n");
		return;
	}
	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended & IS_SUSPENDED) {
		mmi_err(chip, "HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	mmi_info(chip, "Heartbeat!\n");

	pm_stay_awake(chip->dev);
	alarm_cancel(&chip->heartbeat_alarm);

	mutex_lock(&chip->charger_lock);
	mmi_update_battery_status(chip);
	mmi_get_charger_info(chip);
	mmi_update_charger_status(chip);
	mmi_configure_charger(chip);
	mmi_charger_set_constraint(chip);
	mmi_update_charger_event(chip);
	mutex_unlock(&chip->charger_lock);

	mmi_glink_notifier(DEV_ALL, chip);

	mmi_info(chip, "DemoMode:%d, FactoryVersion:%d, FactoryMode:%d,"
		" dcp_pmax:%d, hvdcp_pmax:%d, pd_pmax:%d, wls_pmax:%d\n",
		chip->demo_mode,
		chip->factory_version,
		chip->factory_mode,
		chip->dcp_pmax,
		chip->hvdcp_pmax,
		chip->pd_pmax,
		chip->wls_pmax);

	if (chip->factory_mode ||
	    (chip->factory_version && chip->enable_factory_poweroff)) {
		if (chip->max_charger_rate > MMI_POWER_SUPPLY_CHARGE_RATE_NONE) {
			mmi_dbg(chip, "Factory Kill Armed\n");
			chip->factory_kill_armed = true;
			ktime_get_real_ts64(&start);
		} else if (chip->factory_kill_armed && !factory_kill_disable) {
			ktime_get_real_ts64(&now);
			elapsed_ms = (now.tv_sec - start.tv_sec) * 1000;
			elapsed_ms += (now.tv_nsec - start.tv_nsec) / 1000000;
			if (elapsed_ms < chip->factory_kill_debounce_ms) {
				mmi_err(chip, "Factory kill debounce elapsed_ms:%d\n",
					elapsed_ms);
			} else if(!shutdown_triggered) {
				mmi_err(chip, "Factory kill power off\n");
				shutdown_triggered = true;
				moto_reboot_call_notifier("mmi_charger_heartbeat_work1");
#if (KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE) || defined(MMI_GKI_API_ALLOWANCE)
				orderly_poweroff(true);
#else
				kernel_power_off();
#endif
			}
		} else {
			chip->factory_kill_armed = false;
		}
	}

	if (chip->empty_vbat_shutdown_triggered && !shutdown_triggered) {
		mmi_err(chip, "shutdown for empty battery voltage\n");
		shutdown_triggered = true;
		moto_reboot_call_notifier("mmi_charger_heartbeat_work2");
#if (KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE) || defined(MMI_GKI_API_ALLOWANCE)
		orderly_poweroff(true);
#else
		kernel_power_off();
#endif
	}

	chip->suspended = 0;

	if (chip->factory_mode)
		hb_resch_time = chip->heartbeat_factory_interval;
	else if (chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		hb_resch_time = chip->heartbeat_interval;
	else
		hb_resch_time = chip->heartbeat_dischg_ms;


	mmi_info(chip, "ReSchedule Heartbeat!in %d\n", hb_resch_time);

	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));
	if (suspend_wakeups ||
	    chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		alarm_start_relative(&chip->heartbeat_alarm,
				     ns_to_ktime(HEARTBEAT_WAKEUP_INTRVAL_NS));

	if (chip->max_charger_rate == MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		pm_relax(chip->dev);

	PM_RELAX(chip->mmi_hb_wake_source);
}

static char *bootargs_str = NULL;
static size_t bootargs_str_len = 0;
static int mmi_get_bootarg_dt(char *key, char **value, char *prop, char *spl_flag)
{
	const char *bootargs_tmp = NULL;
	char *idx = NULL;
	char *kvpair = NULL;
	int err = 1;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_tmp_len = 0;

	if (n == NULL)
		goto err;

	if (of_property_read_string(n, prop, &bootargs_tmp) != 0)
		goto putnode;

	bootargs_tmp_len = strlen(bootargs_tmp);
	if (bootargs_tmp_len >= bootargs_str_len) {
		if (bootargs_str)
			kfree(bootargs_str);
		bootargs_str = kzalloc(bootargs_tmp_len + 1, GFP_KERNEL);
		if (!bootargs_str) {
			err = -ENOMEM;
			goto putnode;
		}
		bootargs_str_len = bootargs_tmp_len + 1;
	} else {
		memset(bootargs_str, '\0', bootargs_str_len);
	}

	strscpy(bootargs_str, bootargs_tmp, bootargs_tmp_len + 1);

	idx = strnstr(bootargs_str, key, strlen(bootargs_str));
	if (idx) {
		kvpair = strsep(&idx, " ");
		if (kvpair)
			if (strsep(&kvpair, "=")) {
				*value = strsep(&kvpair, spl_flag);
				if (*value)
					err = 0;
			}
	}

putnode:
	of_node_put(n);
err:
	return err;
}

static int mmi_get_bootarg(char *key, char **value)
{
#ifdef CONFIG_BOOT_CONFIG
	return mmi_get_bootarg_dt(key, value, "mmi,bootconfig", "\n");
#else
	return mmi_get_bootarg_dt(key, value, "bootargs", " ");
#endif
}

static int mmi_get_sku_type(struct mmi_glink_chip *chip, u8 *sku_type)
{
	char *s = NULL;
	char androidboot_radio_str[RADIO_MAX_LEN];

	if (mmi_get_bootarg("androidboot.radio=", &s) == 0) {
		if (s != NULL) {
			strscpy(androidboot_radio_str, s, RADIO_MAX_LEN);
			if (!strncmp("PRC", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_PRC;
			} else if (!strncmp("ROW", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_ROW;
			} else if (!strncmp("NA", androidboot_radio_str, 2)) {
				*sku_type = MMI_CHARGER_SKU_NA;
			} else if (!strncmp("VZW", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_VZW;
			} else if (!strncmp("JPN", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_JPN;
			} else if (!strncmp("ITA", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_ITA;
			} else if (!strncmp("NAE", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_NAE;
			} else if (!strncmp("SUPERSET", androidboot_radio_str, 8)) {
				*sku_type = MMI_CHARGER_SKU_SUPERSET;
			} else {
				*sku_type = 0;
			}
			mmi_info(chip, "SKU type: %s, 0x%02x\n", androidboot_radio_str, *sku_type);
			return 0;
		} else {
			mmi_err(chip, "Could not get SKU type\n");
			return -1;
		}
	} else {
		mmi_err(chip, "Could not get radio bootarg\n");
		return -1;
	}
}

static bool mmi_is_factory_mode(void)
{
	char *mode = NULL;

	if ((this_chip && this_chip->factory_mode) ||
	    !strncmp(bi_bootmode(), "mot-factory", 11))
		return true;

	if (mmi_get_bootarg("androidboot.mode=", &mode) ||
		!mode || strncmp("mot-factory", mode, 11)) {
		return false;
	}
	return true;
}

static bool mmi_is_softbank_sku(void)
{
	char *carrier = NULL;

	if (mmi_get_bootarg("androidboot.carrier=", &carrier)
	   || !carrier || strncmp("softbank", carrier, 8)) {
		return false;
	}
	return true;
}

static bool mmi_is_factory_version(void)
{
	char *bootloader = NULL;

	if (this_chip && this_chip->factory_version)
		return true;

	if (mmi_get_bootarg("androidboot.bootloader=", &bootloader) ||
		!bootloader || !strnstr(bootloader, "factory", strlen(bootloader))) {
		return false;
	}
	return true;
}

int mmi_vote_charging_disable(const char *voter, bool enable)
{
	struct mmi_glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -EINVAL;
	}

	return mmi_vote(&chip->disable_charging_vote, voter, enable, 0);
}
EXPORT_SYMBOL(mmi_vote_charging_disable);

int mmi_vote_charger_suspend(const char *voter, bool enable)
{
	struct mmi_glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -EINVAL;
	}

	return mmi_vote(&chip->suspend_charger_vote, voter, enable, 0);
}
EXPORT_SYMBOL(mmi_vote_charger_suspend);

static enum alarmtimer_restart mmi_heartbeat_alarm_cb(struct alarm *alarm,
						      ktime_t now)
{
	struct mmi_glink_chip *chip = container_of(alarm,
						    struct mmi_glink_chip,
						    heartbeat_alarm);

	mmi_info(chip, "HB alarm fired\n");

	PM_STAY_AWAKE(chip->mmi_hb_wake_source);
	cancel_delayed_work(&chip->heartbeat_work);
	/* Delay by 500 ms to allow devices to resume. */
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(500));

	return ALARMTIMER_NORESTART;
}

static int mmi_psy_notifier_call(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	struct mmi_glink_chip *chip = container_of(nb,
				struct mmi_glink_chip, mmi_psy_notifier);
	struct power_supply *psy = v;
	struct battery_host *batt_host;

	if (!chip || !chip->batt_host) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	batt_host = chip->batt_host;

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy && !batt_host->batt_psy &&
	    (strcmp(psy->desc->name, "battery") == 0)) {
		battery_supply_init(batt_host);
	}

	if (psy &&
	    ((strcmp(psy->desc->name, "battery") == 0) ||
	    (strcmp(psy->desc->name, "usb") == 0) ||
	    (strcmp(psy->desc->name, "wireless") == 0))) {
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

static int mmi_charger_reboot(struct notifier_block *nb,
			 unsigned long event, void *unused)
{
	struct mmi_glink_chip *chip = container_of(nb, struct mmi_glink_chip,
						mmi_reboot);

	if (!chip) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (!chip->factory_mode)
		return NOTIFY_DONE;

	pr_info("Reboot notifier call mmi charger reboot!\n");
	switch (event) {
	case SYS_POWER_OFF:
		factory_kill_disable = true;
		chip->force_charger_disabled = true;
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work, msecs_to_jiffies(0));
		while (chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE &&
			(shutdown_triggered || chip->factory_syspoweroff_wait) && !chip->empty_vbat_shutdown_triggered) {
			mmi_info(chip, "Wait for charger removal\n");
			msleep(100);
		}
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

int mmi_glink_dev_init(struct mmi_glink_chip *chip,
					struct mmi_glink_dev_dts_info *dev_dts,
					int dev_cnt)
{
	int i = 0;
	struct glink_device *glink_dev = NULL;

	if (!chip ||!dev_dts) {
		mmi_err(chip, "invalid input param!\n");
		return -EINVAL;
	}

	if (dev_cnt != chip->glink_dev_num) {
		mmi_err(chip, "invalid input param!, dev_cnt %d, "
						"glink_dev_num %d\n",
			dev_cnt, chip->glink_dev_num);
		return -EINVAL;
	}

	mmi_info(chip, "glink_dev_num %d\n", dev_cnt);

	for (i = 0; i < dev_cnt; i++) {

		switch (dev_dts[i].dev_type) {
		case DEV_BATT:
			mmi_info(chip, "battery[%d] glink device register", dev_dts[i].dev_role);
			glink_dev = battery_glink_device_register(chip, &dev_dts[i]);
			break;
		case DEV_SWITCH_BUCK:
			mmi_info(chip, "switch_buck glink device register");
			glink_dev = switch_buck_device_register(chip, &dev_dts[i]);
			break;
		case DEV_CHARGE_PUMP:
			mmi_info(chip, "charge_pump[%d] glink device register", dev_dts[i].dev_role);
			glink_dev = charge_pump_glink_device_register(chip, &dev_dts[i]);
			break;
		case DEV_BALANCE_CHG:
			mmi_info(chip, "balance[%d] glink device register", dev_dts[i].dev_role);
			glink_dev = balance_glink_device_register(chip, &dev_dts[i]);
			break;
		case DEV_WLS:
			mmi_info(chip, "wireless glink device register");
			glink_dev = wireless_glink_device_register(chip, &dev_dts[i]);
			break;
		case DEV_USB:
			mmi_info(chip, "usb glink device register");
			glink_dev = usb_glink_device_register(chip, &dev_dts[i]);
			break;
		default:
			mmi_err(chip,"No glink_dev found , dev_idx %d, dev type %d !\n", i, dev_dts[i].dev_type);
			break;
		}

		if (!glink_dev) {
			chip->glink_dev_list[i] = glink_dev;
			mmi_info(chip, "register glink dev %d, type %d successfully !", i, dev_dts[i].dev_type);
		}

	}

	mmi_info(chip, "glink dev init successfully !");
	return 0;
}

static int mmi_parse_dt(struct mmi_glink_chip *chip)
{
	int rc, byte_len, i, chrg_idx = 0;
	int limit_capcity_by_sku = 0;
	int sku_upper_limit_capacity = 0;
	int sku_lower_limit_capacity = 0;
	struct device_node *node = chip->dev->of_node, *child;

	chip->enable_charging_limit =
			of_property_read_bool(node, "mmi,enable-charging-limit");

	chip->enable_factory_poweroff =
			of_property_read_bool(node, "mmi,enable-factory-poweroff");

	chip->factory_syspoweroff_wait =
			of_property_read_bool(node, "mmi,factory-syspoweroff-wait");

	chip->start_factory_kill_disabled =
			of_property_read_bool(node, "mmi,start-factory-kill-disabled");

	chip->enable_direct_power_supply =
			of_property_read_bool(node, "mmi,direct-power-supply");

	rc = of_property_read_u32(node, "mmi,factory-kill-debounce-ms",
				  &chip->factory_kill_debounce_ms);
	if (rc)
		chip->factory_kill_debounce_ms = 0;

	rc = of_property_read_u32(node, "mmi,upper-limit-en-voltage",
				  &chip->upper_limit_en_mv);
	if (rc)
		chip->upper_limit_en_mv = 4000;

	rc = of_property_read_u32(node, "mmi,upper-limit-capacity",
				  &chip->upper_limit_capacity);
	if (rc)
		chip->upper_limit_capacity = 100;

	rc = of_property_read_u32(node, "mmi,lower-limit-capacity",
				  &chip->lower_limit_capacity);
	if (rc)
		chip->lower_limit_capacity = 0;

	rc = of_property_read_u32(node, "mmi,limit-capacity-by-sku",
			&limit_capcity_by_sku);
	if (!rc && limit_capcity_by_sku > 0
			&& limit_capcity_by_sku == chip->sku) {
		rc = of_property_read_u32(node, "mmi,sku-upper-limit-capacity",
				&sku_upper_limit_capacity);
		if (!rc && sku_upper_limit_capacity > 0) {
			mmi_warn(chip, "sku %d factory upper limit change to %d\n",
					chip->sku, sku_upper_limit_capacity);
			chip->upper_limit_capacity = sku_upper_limit_capacity;
		}

		rc = of_property_read_u32(node, "mmi,sku-lower-limit-capacity",
				&sku_lower_limit_capacity);
		if (!rc && sku_lower_limit_capacity > 0) {
			mmi_warn(chip, "sku %d factory lower limit change to %d\n",
					chip->sku, sku_lower_limit_capacity);
			chip->lower_limit_capacity = sku_lower_limit_capacity;
		}
	}

	rc = of_property_read_u32(node, "mmi,heartbeat-interval",
				  &chip->heartbeat_interval);
	if (rc)
		chip->heartbeat_interval = HEARTBEAT_DELAY_MS;

	rc = of_property_read_u32(node, "mmi,heartbeat-factory-interval",
				  &chip->heartbeat_factory_interval);
	if (rc)
		chip->heartbeat_factory_interval = HEARTBEAT_FACTORY_MS;

	rc = of_property_read_u32(node, "mmi,dcp-power-max",
				  &chip->dcp_pmax);
	if (rc)
		chip->dcp_pmax = CHARGER_POWER_7P5W;

	rc = of_property_read_u32(node, "mmi,hvdcp-power-max",
				  &chip->hvdcp_pmax);
	if (rc)
		chip->hvdcp_pmax = CHARGER_POWER_15W;

	rc = of_property_read_u32(node, "mmi,pd-power-max",
				  &chip->pd_pmax);
	if (rc)
		chip->pd_pmax = CHARGER_POWER_18W;

	rc = of_property_read_u32(node, "mmi,wls-power-max",
				  &chip->wls_pmax);
	if (rc)
		chip->wls_pmax = CHARGER_POWER_10W;

	rc = of_property_read_u32(node, "mmi,heartbeat-discharger-ms",
				  &chip->heartbeat_dischg_ms);
	if (rc)
		chip->heartbeat_dischg_ms = HEARTBEAT_DISCHARGE_MS;

	mmi_warn(chip, "mmi,heartbeat dischg ms %d\n", chip->heartbeat_dischg_ms);

	rc = of_property_read_u32(node, "mmi,ibat-calc-alignment-time",
				  &chip->ibat_calc_alignment_time);
	if (rc)
		chip->ibat_calc_alignment_time = UINT_MAX;

	chip->charger_present_dynamic_control_bm_ulog =
			of_property_read_bool(node, "mmi,charger_control_bm_ulog");
	chip->bm_ulog_enabled = false;

	for_each_child_of_node(node, child)
		chip->glink_dev_num++;

	if (!chip->glink_dev_num) {
		mmi_err(chip,"No glink_dev_num  list !\n");
		return -ENODEV;
	}

	glink_dev_dts_list = (struct mmi_glink_dev_dts_info *)devm_kzalloc(chip->dev,
					sizeof(struct mmi_glink_dev_dts_info) *
					chip->glink_dev_num, GFP_KERNEL);
	if (!glink_dev_dts_list) {
		mmi_err(chip,"No memory for mmi glink device dts list !\n");
		goto cleanup;
	}

	chip->glink_dev_list = (struct glink_device **)devm_kzalloc(chip->dev,
					sizeof(struct glink_device *) *
					chip->glink_dev_num, GFP_KERNEL);
	if (!chip->glink_dev_list) {
		mmi_err(chip,"No memory for mmi glink device list !\n");
		goto cleanup;
	}

	for_each_child_of_node(node, child) {
		byte_len = of_property_count_strings(child, "dev-name");
		if (byte_len <= 0) {
			mmi_err(chip, "Cannot parse dev-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "dev-name",
					i, &glink_dev_dts_list[chrg_idx].glink_dev_name);
			if (rc < 0) {
				mmi_err(chip, "Cannot parse chrg-name\n");
				goto cleanup;
			}
		}

		byte_len = of_property_count_strings(child, "psy-name");
		if (byte_len <= 0) {
			mmi_err(chip, "Cannot parse psy-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "psy-name",
					i, &glink_dev_dts_list[chrg_idx].psy_name);
			if (rc < 0) {
				mmi_err(chip, "Cannot parse psy-name\n");
				goto cleanup;
			}
		}

		of_property_read_u32(child, "dev-type",
			&glink_dev_dts_list[chrg_idx].dev_type);

		if (glink_dev_dts_list[chrg_idx].dev_type == DEV_BATT ||
			glink_dev_dts_list[chrg_idx].dev_type == DEV_BALANCE_CHG ||
			glink_dev_dts_list[chrg_idx].dev_type == DEV_CHARGE_PUMP) {
			of_property_read_u32(child, "dev-role",
				&glink_dev_dts_list[chrg_idx].dev_role);

			mmi_info(chip, "dev type %d, dev-role %d\n", glink_dev_dts_list[chrg_idx].dev_type, glink_dev_dts_list[chrg_idx].dev_role);
		}
		mmi_info(chip, "mmi,chrg-name: %s, psy-name: %s, dev-type %d\n",
					glink_dev_dts_list[chrg_idx].glink_dev_name,
					glink_dev_dts_list[chrg_idx].psy_name,
					glink_dev_dts_list[chrg_idx].dev_type);
		chrg_idx++;
	}

	rc = mmi_glink_dev_init(chip, glink_dev_dts_list, chip->glink_dev_num);
	if (rc < 0) {
		mmi_err(chip, "glink dev init failed\n");
		goto cleanup;
	}

	mmi_err(chip, "mmi parse device-tree successfully !");
	return 0;
cleanup:
	chip->glink_dev_num = 0;
	devm_kfree(chip->dev, chip->glink_dev_list);
	devm_kfree(chip->dev, glink_dev_dts_list);
	return 0;
}

static int mmi_glink_charger_notify_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct qti_charger_notify_data *notify_data = data;
	struct mmi_glink_chip *chip = container_of(nb, struct mmi_glink_chip, mmi_glink_nb);

	if (notify_data->receiver != OEM_NOTIFY_RECEIVER_MMI_CHG) {
		mmi_err(chip, "Skip mis-matched receiver: %#x\n", notify_data->receiver);
		return 0;
	}

	switch (event) {
	case NOTIFY_EVENT_MMI_GLINK_STATE_UP:
		memset(&chip->charger_constraint, 0, sizeof(chip->charger_constraint));
		mmi_info(chip, "receive GLINK_STATE_UP, clear mmi_charger constraint\n");
		break;
	default:
		mmi_err(chip, "Unknown mmi_glink event: %#lx\n", event);
		break;
	}

	return 0;
}

static int mmi_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mmi_glink_chip *chip;
	struct battery_host *batt_host;
	u8 sku_type = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->name = "mmi_charger";
	chip->dev = &pdev->dev;
	chip->suspended = 0;
	chip->factory_version = mmi_is_factory_version();
	chip->factory_mode = mmi_is_factory_mode();
	chip->is_softbank = mmi_is_softbank_sku();
	chip->disable_charging_vote.name = "disable_charging";
	chip->suspend_charger_vote.name = "suspend_charger";
	chip->charger_status.charging_limit_modes = CHARGING_LIMIT_UNKNOWN;
	platform_set_drvdata(pdev, chip);
	device_init_wakeup(chip->dev, true);

	chip->debug_enabled = &debug_enabled;
	chip->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chip->ipc_log)
		mmi_err(chip, "Failed to create mmi charger IPC log\n");
	else
		mmi_info(chip, "IPC logging is enabled for mmi charger\n");

	rc = mmi_glink_class_init();
	if (rc) {
		mmi_err(chip, "Failed to init glink class\n");
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = mmi_get_sku_type(chip, &sku_type)) == 0) {
		chip->sku = sku_type;
	} else {
		chip->sku = 0;
		mmi_err(chip, "Fail to get sku type\n");
	}

	rc = mmi_parse_dt(chip);
	if (rc) {
		mmi_err(chip, "Failed to parse device tree\n");
		rc = -EINVAL;
		goto exit;
	}

	this_chip = chip;
	mutex_init(&chip->charger_lock);
	mutex_init(&chip->battery_lock);
	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_charger_heartbeat_work);
	PM_WAKEUP_REGISTER(chip->dev, chip->mmi_hb_wake_source, "mmi_hb_wake");
	alarm_init(&chip->heartbeat_alarm, ALARM_BOOTTIME,
		   mmi_heartbeat_alarm_cb);

	trusted_shash_alloc();
	batt_host = battery_glink_host_init(chip);
	if (!batt_host) {
		mmi_err(chip, "Failed to init battery glink host\n");
		goto exit;
	}
	chip->batt_host = batt_host;
	battery_supply_init(chip->batt_host);

	rc = device_create_file(chip->dev,
				&dev_attr_state_sync);
	if (rc) {
		mmi_err(chip, "couldn't create state_sync\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_dcp_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create dcp_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_hvdcp_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create hvdcp_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_pd_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create pd_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_wls_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create wls_pmax\n");
	}

	chip->mmi_glink_nb.notifier_call = mmi_glink_charger_notify_callback;
	rc = qti_charger_register_notifier(&chip->mmi_glink_nb);
	if (rc)
		pr_err("Failed to register mmi_glink notifier, rc=%d\n", rc);

	/* Register the notifier for the psy updates*/
	chip->mmi_psy_notifier.notifier_call = mmi_psy_notifier_call;
	rc = power_supply_reg_notifier(&chip->mmi_psy_notifier);
	if (rc)
		mmi_err(chip, "Failed to reg notifier: %d\n", rc);

	if (chip->factory_mode) {
		mmi_info(chip, "Entering Factory Mode!\n");
		chip->mmi_reboot.notifier_call = mmi_charger_reboot;
		chip->mmi_reboot.next = NULL;
		chip->mmi_reboot.priority = 1;
		rc = register_reboot_notifier(&chip->mmi_reboot);
		if (rc)
			mmi_err(chip, "Register for reboot failed\n");
	}

	if (chip->start_factory_kill_disabled)
		factory_kill_disable = 1;

	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(3000));

	mmi_info(chip, "MMI glink charger probed successfully!\n");
	return 0;
exit:
	ipc_log_context_destroy(chip->ipc_log);

	return rc;
}

static void mmi_charger_remove(struct platform_device *pdev)
{
	struct mmi_glink_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->heartbeat_work);

	if (chip->factory_mode)
		unregister_reboot_notifier(&chip->mmi_reboot);
	power_supply_unreg_notifier(&chip->mmi_psy_notifier);
	device_remove_file(chip->dev, &dev_attr_state_sync);
	device_remove_file(chip->dev, &dev_attr_dcp_pmax);
	device_remove_file(chip->dev, &dev_attr_hvdcp_pmax);
	device_remove_file(chip->dev, &dev_attr_pd_pmax);
	device_remove_file(chip->dev, &dev_attr_wls_pmax);
	trusted_shash_release();
	battery_supply_deinit(chip->batt_host);
	battery_glink_host_deinit();
	battery_glink_device_unregister();
	balance_glink_device_unregister();
	wireless_glink_device_unregister();
	usb_glink_device_unregister();
	PM_WAKEUP_UNREGISTER(chip->mmi_hb_wake_source);
	ipc_log_context_destroy(chip->ipc_log);
	mmi_glink_class_exit();
	if (bootargs_str) {
		kfree(bootargs_str);
		bootargs_str = NULL;
		bootargs_str_len = 0;
	}
	return;
}

static void mmi_charger_shutdown(struct platform_device *pdev)
{
	struct mmi_glink_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->heartbeat_work);
	mmi_info(chip, "MMI charger shutdown\n");

	return;
}

#ifdef CONFIG_PM_SLEEP
static int mmi_charger_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_glink_chip *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~WAS_SUSPENDED;
	chip->suspended |= IS_SUSPENDED;

	return 0;
}

static int mmi_charger_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_glink_chip *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~IS_SUSPENDED;
	chip->suspended |= WAS_SUSPENDED;

	return 0;
}
#else
#define mmi_charger_suspend NULL
#define mmi_charger_resume NULL
#endif

static const struct dev_pm_ops mmi_charger_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmi_charger_suspend, mmi_charger_resume)
};

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,mmi-charger", },
	{ },
};

static struct platform_driver mmi_charger_driver = {
	.driver		= {
		.name		= "mmi,mmi-charger",
		.owner		= THIS_MODULE,
		.pm		= &mmi_charger_dev_pm_ops,
		.of_match_table	= match_table,
	},
	.probe		= mmi_charger_probe,
	.remove		= mmi_charger_remove,
	.shutdown	= mmi_charger_shutdown,
};
module_platform_driver(mmi_charger_driver);

MODULE_DESCRIPTION("MMI Glink Charger Driver");
MODULE_LICENSE("GPL v2");
