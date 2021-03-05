/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MMI_CHRG_CORE_H_
#define __MMI_CHRG_CORE_H_

#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/alarmtimer.h>
#include <linux/notifier.h>
#include "mmi_charger_class.h"

#define mmi_chrg_err(chip, fmt, ...)		\
	pr_err("%s: %s: " fmt, chip->name,	\
		__func__, ##__VA_ARGS__)	\

#define mmi_chrg_info(chip, fmt, ...)		\
	pr_info("%s: %s: " fmt, chip->name,	\
		__func__, ##__VA_ARGS__)	\

#define mmi_chrg_dbg(chip, reason, fmt, ...)			\
	do {							\
		if (*chip->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

enum print_reason {
	PR_INTERRUPT    = BIT(0),
	PR_MISC         = BIT(2),
	PR_MOTO         = BIT(7),
};

#define MAX_NUM_STEPS 10
enum mmi_chrg_temp_zones {
	ZONE_FIRST = 0,
	/* states 0-9 are reserved for zones */
	ZONE_LAST = MAX_NUM_STEPS + ZONE_FIRST - 1,
	ZONE_HOT,
	ZONE_COLD,
	ZONE_NONE = 0xFF,
};

enum mmi_chrg_steps {
	STEP_FIRST = 0,
	STEP_LAST = MAX_NUM_STEPS + STEP_FIRST - 1,
	STEP_NONE = 0xFF,
};

enum mmi_pd_pps_result {
	NO_ERROR,
	BLANCE_POWER,
	RESET_POWER,
};

struct mmi_chrg_step_power {
	u32 chrg_step_volt;
	u32 chrg_step_curr;
};

struct mmi_chrg_temp_zone {
	int	temp_c;	/*temperature*/
	struct	mmi_chrg_step_power *chrg_step_power;
};

struct mmi_chrg_step_info {
	enum mmi_chrg_steps	pres_chrg_step;
	int	temp_c;
	int	chrg_step_cc_curr;
	int	chrg_step_cv_volt;
	int	chrg_step_cv_tapper_curr;
	bool	last_step;
};

struct mmi_chrg_dev_ops {
	const char *dev_name;
	struct mmi_charger_ops	*ops;
};

struct mmi_chrg_dts_info {
	const char *chrg_name;
	const char *psy_name;
	int charging_curr_limited;
	int charging_curr_min;
};

#define PPS_RET_HISTORY_SIZE	8
#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3
#define STEP_FIREST_CURR_COMP 		300000
#define TYPEC_HIGH_CURRENT_UA		3000000
#define TYPEC_MIDDLE_CURRENT_UA		2000000
#define SWITCH_CHARGER_PPS_VOLT		5000000
#define PUMP_CHARGER_PPS_MIN_VOLT	8000000
#define COOLING_HYSTERISIS_DEGC 2
struct mmi_charger_manager {
	const char	*name;
	struct device	*dev;
	struct power_supply	*batt_psy;
	struct power_supply	*qcom_psy;
	struct power_supply	*extrn_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*mmi_chrg_mgr_psy;
	struct usbpd	*pd_handle;
	struct usbpd_pdo_info	mmi_pdo_info[PD_MAX_PDO_NUM];
	struct notifier_block	psy_nb;

	struct wakeup_source	mmi_hb_wake_source;
	struct alarm		heartbeat_alarm;
	bool			factory_mode;
	bool			suspended;
	bool			awake;
	bool			cp_disable;

	int *debug_mask;
	int mmi_pd_pdo_idx;	/*request the pdo idx of PD*/
	int pps_volt_steps;	/*PPS voltage, programming step size*/
	int pps_curr_steps;	/*pps current, programming step size*/

	int pd_volt_max;	/*the Maximum request PD Voltage*/
	int pd_curr_max;	/*the Maximum request PD current*/
	int batt_ovp_lmt;	/*the battery over current limitation*/
	int pl_chrg_vbatt_min;	/*the minimum battery voltage to enable parallel charging*/

	int typec_middle_current;
	int step_first_curr_comp;
	int pps_volt_comp;
	int pd_request_volt;
	int pd_request_curr;
	/*the request PD power*/
	int pd_request_volt_prev;
	int pd_request_curr_prev;
	/*the previous request PD power*/
	int pd_sys_therm_volt;
	int pd_sys_therm_curr;
	/*the thermal PD power*/

	int pd_batt_therm_volt;
	int pd_batt_therm_curr;

	int pd_target_volt;
	int pd_target_curr;
	/*the final commited request PD power*/

	int pps_result;
	int pps_result_history[PPS_RET_HISTORY_SIZE];
	int pps_result_history_idx;
	/*save the result of the return from PD request*/

	int 	charger_rate;
	bool vbus_present;
	bool pd_pps_support;
	bool pd_pps_balance;
	bool freeze_pd_power;
	bool extrn_fg;
	const char	*extrn_fg_name;
	bool extrn_sense;
	bool recovery_pmic_chrg;
	bool dont_rerun_aicl;

	bool sys_therm_cooling;
	bool sys_therm_force_pmic_chrg;
	bool batt_therm_cooling;
	int batt_therm_cooling_cnt;

	bool use_batt_age;
	int cycles;
	int batt_cap_delta;
	int batt_soc_delta;
	int soc_cycles_start;
	int pres_batt_status;
	int prev_batt_status;

	struct delayed_work	mmi_chrg_sm_work;	/*mmi charger state machine work*/
	struct delayed_work	heartbeat_work;	/*cycle trig heartbeat work*/
	struct completion	sm_completion;
	struct work_struct	psy_changed_work;	/*the change notification of power supply*/
	bool sm_work_running;
	int num_temp_zones;
	struct mmi_chrg_temp_zone		*temp_zones;	/*the temperature zone of charging*/
	enum mmi_chrg_temp_zones	pres_temp_zone;	/*the present zone idx*/

	int	*thermal_mitigation;	/*thermal mitigation array*/
	int	thermal_levels;
	int	system_thermal_level;	/*thermal level setting*/

	int	chrg_step_nums;
	struct mmi_chrg_step_info chrg_step;	/*step charger info*/

	int mmi_chrg_dev_num;
	struct mmi_charger_device **chrg_list;	/*charger device list*/
};

extern bool mmi_get_pps_result_history(struct mmi_charger_manager *chip);
extern void mmi_set_pps_result_history(struct mmi_charger_manager *chip, int pps_result);
extern void mmi_clear_pps_result_history(struct mmi_charger_manager *chip);
extern int mmi_calculate_delta_volt(int pps_voltage, int pps_current, int delta_curr);
extern bool mmi_find_chrg_step(struct mmi_charger_manager *chip, int temp_zone, int vbatt_volt);
extern bool mmi_find_temp_zone(struct mmi_charger_manager *chip, int temp_c, bool ignore_hysteresis_degc);
extern void clear_chg_manager(struct mmi_charger_manager *chip);
extern void mmi_update_all_charger_status(struct mmi_charger_manager *chip);
extern void mmi_update_all_charger_error(struct mmi_charger_manager *chip);
extern void mmi_dump_charger_error(struct mmi_charger_manager *chip,
									struct mmi_charger_device *chrg_dev);
extern ssize_t mmi_get_factory_image_mode(void);
extern ssize_t mmi_set_factory_image_mode(int mode);
extern ssize_t mmi_get_factory_charge_upper(void);
extern ssize_t mmi_get_demo_mode(void);
extern ssize_t mmi_set_demo_mode(int mode);
extern ssize_t mmi_get_max_chrg_temp(void);
extern ssize_t mmi_set_max_chrg_temp(int value);
#endif