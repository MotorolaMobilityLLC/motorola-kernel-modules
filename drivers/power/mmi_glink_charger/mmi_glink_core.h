/*
 * Copyright (C) 2020 Motorola Mobility LLC
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

#ifndef __MMI_GLINK_CHARGER_H__
#define __MMI_GLINK_CHARGER_H__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/time64.h>
#include <linux/ipc_logging.h>
#include <linux/debugfs.h>
#include <linux/power/mmi_glink_api.h>

#include "device_class.h"
#include "battery_host.h"

#define mmi_err(chg, fmt, ...)			\
	do {						\
		pr_err("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"E %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_warn(chg, fmt, ...)			\
	do {						\
		pr_warn("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"W %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_info(chg, fmt, ...)			\
	do {						\
		pr_info("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"I %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_dbg(chg, fmt, ...)			\
	do {							\
		if (*chg->debug_enabled)		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
			"D %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define MMI_LOG_PAGES (50)
#define MMI_LOG_DIR "mmi_glink_charger"
#define MMI_BATT_SN_LEN 16
#define CHG_SHOW_MAX_SIZE 50

struct mmi_glink_dev_dts_info {
	const char *glink_dev_name;
	const char *psy_name;
	int dev_type;
	int dev_role;
};

struct mmi_charger_info {
	int chrg_uv;
	int chrg_ua;
	int chrg_type;
	int chrg_pmax_mw;
	int chrg_present;
	bool chrg_otg_enabled;
	int usb_online;
	int wls_online;
	int wls_tx_enabled;
	int icm_sm_st;
	int chrg_stat;
};

struct mmi_pmic_info {
	int pmic_vph_uv;
	int pmic_vbatt_uv;
	int pmic_ibatt_ua;
	int aicl_result_ma;
	int icl_result_ma;
	int vfloat_mv;
	int pmic_suspend_st;
};

#define MMI_VOTE_NUM_MAX 32
struct mmi_vote {
	const char *name;
	int votes[MMI_VOTE_NUM_MAX];
	const char *voters[MMI_VOTE_NUM_MAX];
};

enum mmi_chrg_step {
	STEP_NONE,
	STEP_NORM,
	STEP_FULL,
	STEP_DEMO,
	STEP_STOP,
};

#define MAX_NUM_TEMP_ZONE 10
enum mmi_temp_zones {
	ZONE_FIRST = 0,
	/* states 0-9 are reserved for zones */
	ZONE_LAST = MAX_NUM_TEMP_ZONE + ZONE_FIRST - 1,
	ZONE_HOT,
	ZONE_COLD,
	ZONE_NONE,
};

static char *stepchg_str[] = {
	[STEP_NONE]		= "NONE",
	[STEP_NORM]		= "NORMAL",
	[STEP_FULL]		= "FULL",
	[STEP_DEMO]		= "DEMO",
	[STEP_STOP]		= "STOP",
};

enum charging_limit_modes {
	CHARGING_LIMIT_OFF,
	CHARGING_LIMIT_RUN,
	CHARGING_LIMIT_UNKNOWN,
};

enum mmi_charger_notify_event {
  NOTIFY_EVENT_MMI_GLINK_STATE_DOWN,
  NOTIFY_EVENT_MMI_GLINK_STATE_UP,
};

struct mmi_charger_status {
	int demo_full_soc;
	bool demo_chrg_suspend;
	enum mmi_temp_zones pres_temp_zone;
	enum mmi_chrg_step pres_chrg_step;
	enum charging_limit_modes charging_limit_modes;
};

struct mmi_charger_constraint {
	int demo_mode;
	bool factory_mode;
	bool factory_version;
	bool is_softbank;
	int dcp_pmax;
	int hvdcp_pmax;
	int pd_pmax;
	int wls_pmax;
};

struct mmi_glink_chip {
	char			*name;
	struct device		*dev;

	struct mutex		charger_lock;
	struct mutex		battery_lock;
	char			*batt_uenvp[2];

	struct glink_device 	**glink_dev_list;	/*glink device list*/
	int 			glink_dev_num;

	struct battery_host *batt_host;

	struct mmi_charger_info charger_info;
	struct mmi_pmic_info pmic_info;
	struct mmi_charger_status charger_status;
	struct mmi_charger_constraint charger_constraint;
	struct battery_info battery_info;

	int			max_charger_rate;
	int			real_charger_type;
	bool			usb_present;
	bool			wls_present;
	bool			vbus_present;
	int			power_watt;

	int			suspended;
	int			demo_mode;
	bool			factory_mode;
	bool			factory_version;
	bool			factory_kill_armed;
	bool			force_charger_disabled;
	bool			force_charging_enabled;
	bool			direct_power_supply;
	bool			is_softbank;
	u8			sku;

	bool			charging_disable;
	bool			charger_suspend;

	int			dcp_pmax;
	int			hvdcp_pmax;
	int			pd_pmax;
	int			wls_pmax;

	int			state_of_health;
	int			max_chrg_temp;
	bool			enable_charging_limit;
	bool			enable_factory_poweroff;
	bool			factory_syspoweroff_wait;
	bool			start_factory_kill_disabled;
	bool			enable_direct_power_supply;
	int			upper_limit_en_mv;
	int			upper_limit_capacity;
	int			lower_limit_capacity;

	struct wakeup_source	*mmi_hb_wake_source;
	struct alarm		heartbeat_alarm;
	int			heartbeat_interval;
	int			heartbeat_factory_interval;
	struct notifier_block	mmi_reboot;
	struct notifier_block	mmi_psy_notifier;
	struct notifier_block	mmi_glink_nb;
	struct delayed_work	heartbeat_work;

	bool			*debug_enabled;
	void			*ipc_log;

	struct mmi_vote		suspend_charger_vote;
	struct mmi_vote		disable_charging_vote;
	uint32_t		factory_kill_debounce_ms;

	bool			empty_vbat_shutdown_triggered;

	int			heartbeat_dischg_ms;
	uint32_t		ibat_calc_alignment_time;
	bool			charger_present_dynamic_control_bm_ulog;
	bool			bm_ulog_enabled;
};

struct encrypted_data {
	u32 random_num[4];
	u32 hmac_data[4];
	u32 sha1_data[4];
};

struct battery_host *battery_glink_host_init(struct mmi_glink_chip *chip);
#endif
