#include <linux/init.h> /* For init/exit macros */
#include <linux/module.h> /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include "mtk_charger.h"
#include "moto_wlc2.h"
#include "mtk_charger_algorithm_class.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>

static int wlc_dbg_level = WLC_DEBUG_LEVEL;

int wlc_get_debug_level(void)
{
	return wlc_dbg_level;
}

void moto_wlc_control_gpio(struct chg_alg_device *alg, bool enable)
{
	struct moto_wlc *wlc;

	wlc = dev_get_drvdata(&alg->dev);
	if (gpio_is_valid(wlc->wls_control_en)) {
		gpio_set_value(wlc->wls_control_en, enable);
		pr_info("%s inhibit:%d wireless %s\n", __func__,
			gpio_get_value(wlc->wls_control_en), enable ? "disable" : "enable");
	}
}

static int wlc_plugout_reset(struct chg_alg_device *alg)
{
	int ret = 0, cnt = 0;
	struct moto_wlc *wlc;

	wlc_dbg("%s\n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	switch (wlc->state) {
	case WLC_HW_UNINIT:
	case WLC_HW_FAIL:
	case WLC_HW_READY:
		break;
	case WLC_TA_NOT_SUPPORT:
		wlc->state = WLC_HW_READY;
		break;
	case WLC_RUN:
		/* set flag to end WLC thread asap */
		mutex_lock(&wlc->cable_out_lock);
		wlc->is_cable_out_occur = true;
		//moto_wlc_control_gpio(alg, true);
		mutex_unlock(&wlc->cable_out_lock);
		while (mutex_trylock(&wlc->access_lock) == 0) {
			wlc_err("%s:wlc is running state:%d cnt:%d\n", __func__,
				wlc->state, cnt);
			cnt++;
			msleep(100);
		}
		wlc_hal_vbat_mon_en(alg, CHG1, false);
		wlc->old_cv = 0;
		wlc->is_cable_out_occur = false;
		wlc->stop_6pin_re_en = 0;

		/* Enable OVP */
		//ret = wlc_hal_enable_vbus_ovp(alg, true);
		//if (ret < 0)
			//wlc_err("%s:enable vbus ovp fail, ret:%d\n", __func__,
				//ret);

		/* Set MIVR for vbus 5V */
		ret = wlc_hal_set_mivr(alg, CHG1,
				       wlc->min_charger_voltage); /* uV */
		if (ret < 0)
			wlc_err("%s:set mivr fail, ret:%d\n", __func__, ret);

		wlc_dbg("%s: OK\n", __func__);
		wlc->state = WLC_HW_READY;
		mutex_unlock(&wlc->access_lock);
		break;
	default:
		wlc_dbg("%s: error state:%d\n", __func__, wlc->state);
		break;
	}
	return 0;
}

int wlc_reset_ta_vchr(struct chg_alg_device *alg)
{
	int ret;
	struct moto_wlc *wlc;

	wlc_dbg("%s: starts\n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	ret = wlc_hal_set_mivr(alg, CHG1, wlc->min_charger_voltage);
	if (ret < 0)
		wlc_err("%s:set mivr fail, ret:%d\n", __func__, ret);

	wlc_hal_vbat_mon_en(alg, CHG1, false);
	wlc->old_cv = 0;
	wlc->stop_6pin_re_en = 0;

	wlc_dbg("%s: OK\n", __func__);

	return ret;
}

static int wlc_leave(struct chg_alg_device *alg)
{
	int ret = 0;
	struct moto_wlc *wlc;

	wlc_dbg("%s: starts\n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	ret = wlc_reset_ta_vchr(alg);
	if (ret != 0) {
		wlc_err("%s: failed, state = %d, ret = %d\n", __func__,
			wlc->state, ret);
	}

	//ret = wlc_hal_enable_vbus_ovp(alg, true);
	//if (ret != 0) {
	//	wlc_err("%s: enable vbus ovp fai,ret:%d\n", __func__, ret);
	//}

	wlc_hal_set_mivr(alg, CHG1, wlc->min_charger_voltage);
	if (ret != 0) {
		wlc_err("%s:set mivr fail,ret:%d\n", __func__, ret);
	}

	wlc_dbg("%s: OK\n", __func__);
	return ret;
}

static int __wlc_check_charger(struct chg_alg_device *alg)
{
	int ret = 0, ret_value = 0;
	struct moto_wlc *wlc;

	wlc = dev_get_drvdata(&alg->dev);

	wlc_dbg("%s type:%d", __func__, wlc_hal_get_charger_type(alg));

	if (wlc_hal_get_charger_type(alg) != POWER_SUPPLY_TYPE_WIRELESS) {
		ret_value = ALG_TA_NOT_SUPPORT;
		goto out;
	}

	if (wlc->is_cable_out_occur)
		goto out;

	ret = wlc_reset_ta_vchr(alg);
	if (ret != 0)
		goto out;

	if (wlc->is_cable_out_occur)
		goto out;

	wlc_dbg("%s: OK, state = %d\n", __func__, wlc->state);

	return ret;
out:
	if (ret_value == 0)
		ret_value = ALG_TA_NOT_SUPPORT;
	wlc_dbg("%s,state:%d,plugout:%d \n", __func__,
		wlc->state, wlc->is_cable_out_occur);
	return ret_value;
}

static int _wlc_init_algo(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;
	int log_level;

	wlc = dev_get_drvdata(&alg->dev);
	mutex_lock(&wlc->access_lock);
	if (wlc_hal_init_hardware(alg) != 0) {
		wlc->state = WLC_HW_FAIL;
		wlc_err("%s:init hw fail\n", __func__);
	} else
		wlc->state = WLC_HW_READY;

	wlc_hal_vbat_mon_en(alg, CHG1, false);
	wlc->old_cv = 0;
	wlc->stop_6pin_re_en = 0;

	alg->config = SINGLE_CHARGER;

	log_level = wlc_hal_get_log_level(alg);
	pr_notice("%s: log_level=%d", __func__, log_level);
	if (log_level > 0)
		wlc_dbg_level = log_level;

	mutex_unlock(&wlc->access_lock);
	wlc_dbg("%s config:%d\n", __func__, alg->config);
	return 0;
}

static char *wlc_state_to_str(int state)
{
	switch (state) {
	case WLC_HW_UNINIT:
		return "WLC_HW_UNINIT";
	case WLC_HW_FAIL:
		return "WLC_HW_FAIL";
	case WLC_HW_READY:
		return "WLC_HW_READY";
	case WLC_TA_NOT_SUPPORT:
		return "WLC_TA_NOT_SUPPORT";
	case WLC_RUN:
		return "WLC_RUN";
	case WLC_DONE:
		return "WLC_DONE";
	default:
		break;
	}
	wlc_err("%s unknown state:%d\n", __func__, state);
	return "WLC_UNKNOWN";
}

static int _wlc_is_algo_ready(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;
	int ret_value;

	wlc = dev_get_drvdata(&alg->dev);

	mutex_lock(&wlc->access_lock);
	__pm_stay_awake(wlc->suspend_lock);
	wlc_dbg("%s state:%s\n", __func__, wlc_state_to_str(wlc->state));

	switch (wlc->state) {
	case WLC_HW_UNINIT:
	case WLC_HW_FAIL:
		ret_value = ALG_INIT_FAIL;
		break;
	case WLC_HW_READY:
		if (wlc_hal_get_charger_type(alg) != POWER_SUPPLY_TYPE_WIRELESS)
			ret_value = ALG_TA_NOT_SUPPORT;
		else
			ret_value = ALG_READY;
		break;
	case WLC_TA_NOT_SUPPORT:
		ret_value = ALG_TA_NOT_SUPPORT;
		break;
	case WLC_RUN:
		ret_value = ALG_RUNNING;
		break;
	default:
		ret_value = ALG_INIT_FAIL;
		break;
	}
	__pm_relax(wlc->suspend_lock);
	mutex_unlock(&wlc->access_lock);
	wlc_dbg("%s ret_value:%d\n", __func__, ret_value);
	return ret_value;
}

#define VBUS_DEFAULT_MV 5000
static int wlc_sc_set_charger(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;
	int ichg1_min = -1, aicr1_min = -1;
	int ret;
	int charging_current, input_current, vbus, input_thermal_limit;
	bool cable_ready = false;
	int temp = 0, batt_cv = 0;
	bool cv_auto = true;
	wlc_dbg("%s \n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	if (wlc->input_current_limit1 == 0 ||
	    wlc->charging_current_limit1 == 0) {
		pr_notice("input/charging current is 0, end WLC\n");
		return -1;
	}
	mutex_lock(&wlc->data_lock);

	input_current = wlc->wireless_charger_max_input_current;
	vbus = VBUS_DEFAULT_MV;
	charging_current = wlc->wireless_charger_max_current;

	cable_ready = wls_chg_current_select(wlc, &input_current, &vbus) >= 0 ? true : false;
	wlc->cable_ready = cable_ready;
	if (!cable_ready) {
		mutex_unlock(&wlc->data_lock);
		wlc_hal_set_charging_current(alg, CHG1, 3150000);
		temp = wlc_hal_get_batt_temp(alg);
		if (temp > 15 && temp <= 45)
			cv_auto = false;
	} else {
		cv_auto = true;
		if (wlc->charging_current_limit1 != -1) {
			if (wlc->charging_current_limit1 < charging_current)
				wlc->charging_current1 =
					wlc->charging_current_limit1;
			ret = wlc_hal_get_min_charging_current(alg, CHG1,
							       &ichg1_min);
			if (ret != -EOPNOTSUPP &&
			    wlc->charging_current_limit1 < ichg1_min)
				wlc->charging_current1 = 0;

			input_thermal_limit =
				wlc->charging_current_limit1 / 1000;
			input_thermal_limit *= VBUS_DEFAULT_MV;
			input_thermal_limit /= vbus;
			input_thermal_limit *= 1000;
		} else {
			wlc->charging_current1 = charging_current;
			input_thermal_limit = -1;
		}
		wlc->charging_current1 = wlc->charging_current1 < wlc->mmi_fcc ?
						       wlc->charging_current1 :
						       wlc->mmi_fcc;

		if (input_thermal_limit != -1 &&
		    input_thermal_limit < input_current) {
			wlc->input_current1 = input_thermal_limit;
			ret = wlc_hal_get_min_input_current(alg, CHG1,
							    &aicr1_min);
			if (ret != -EOPNOTSUPP &&
			    input_thermal_limit < aicr1_min)
				wlc->input_current1 = 0;
			wlc_info("%s input_thermal_limit:%d aicr1_min:%d\n",
					__func__, input_thermal_limit, aicr1_min);
		} else
			wlc->input_current1 = input_current;

		wlc_info("%s input current = %d:%d:%d:%d, vout = %d\n",
			 __func__, wlc->input_current1, input_current,
			 input_thermal_limit, wlc->charging_current_limit1,
			 vbus);
		mutex_unlock(&wlc->data_lock);

		if (wlc->input_current1 == 0 || wlc->charging_current1 == 0) {
			wlc_err("current is zero %d %d\n", wlc->input_current1,
				wlc->charging_current1);
			return -1;
		}

		wlc_hal_set_charging_current(alg, CHG1, wlc->charging_current1);
		wlc_hal_set_input_current(alg, CHG1, wlc->input_current1);
	}
	if (wlc->old_cv == 0 || (wlc->old_cv != wlc->cv) ||
	    wlc->wlc_6pin_en == 0) {
		wlc_hal_vbat_mon_en(alg, CHG1, false);
		batt_cv = wlc_hal_get_batt_cv(alg);
		if(!cv_auto)
			wlc_hal_set_cv(alg, CHG1, batt_cv);
		else
			wlc_hal_set_cv(alg, CHG1, wlc->cv);
		if (wlc->wlc_6pin_en && wlc->stop_6pin_re_en != 1)
			wlc_hal_vbat_mon_en(alg, CHG1, true);

		wlc_dbg("%s old_cv=%d, new_cv=%d, wlc_6pin_en=%d\n", __func__,
			wlc->old_cv, wlc->cv, wlc->wlc_6pin_en);

		wlc->old_cv = wlc->cv;
	} else {
		if (wlc->wlc_6pin_en && wlc->stop_6pin_re_en != 1) {
			wlc->stop_6pin_re_en = 1;
			wlc_hal_vbat_mon_en(alg, CHG1, true);
		}
	}

	wlc_dbg("%s m:%d s:%d cv:%d chg1:%d,%d,%d, min:%d:%d, 6pin_en:%d, 6pin_re_en=%d\n",
		__func__, alg->config, wlc->state, wlc->cv, wlc->input_current1,
		wlc->charging_current1, wlc->mmi_fcc, ichg1_min, aicr1_min,
		wlc->wlc_6pin_en, wlc->stop_6pin_re_en);

	return 0;
}

static int wlc_tcd_set_cur_state(struct thermal_cooling_device *tcd,
	unsigned long state);

static void mmi_thermal_check_status(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc = dev_get_drvdata(&alg->dev);
	int boot_mode = wlc_hal_get_boot_mode(alg);
	int batt_temp = 0;
	int i = 0;
	int thermal_level = 0;

	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if ((boot_mode != 8 && boot_mode != 9)
		|| IS_ERR_OR_NULL(wlc->wlc_thermal_com))
		return;

	batt_temp = wlc_hal_get_batt_temp(alg);
	if (batt_temp == -EINVAL) {
		wlc_err("[%s] get batt temp error\n",__func__);
		return;
	}

	batt_temp = batt_temp * 10;
	for (i = 0; i < wlc->num_wlc_thermal_com; i++) {
		if (batt_temp >= wlc->wlc_thermal_com[i].temp_c)
			thermal_level = wlc->wlc_thermal_com[i].level;
		else
			break;
	}

	wlc_tcd_set_cur_state(wlc->tcd, thermal_level);

	pr_info("[%s]batt temp %d, level %d\n", __func__, batt_temp, thermal_level);
}

static int __wlc_run(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;
	int ret = 0, ret_value = 0, uisoc = 0;
	wlc_dbg("%s \n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	if (wlc_hal_get_charger_type(alg) != POWER_SUPPLY_TYPE_WIRELESS) {
		ret_value = ALG_TA_NOT_SUPPORT;
		goto out;
	}

	mmi_thermal_check_status(alg);

	uisoc = wlc_hal_get_uisoc(alg);
	if (uisoc != wlc->data.uisoc) {
		wls_device_uisoc_change(wlc, uisoc);
		wlc->data.uisoc = uisoc;
	}

	if (wlc_sc_set_charger(alg) != 0) {
		ret = wlc_leave(alg);
		ret_value = ALG_DONE;
		goto out;
	}

	wlc_dbg("%s cv:%d chg1:%d,%d uisoc %d\n", __func__, wlc->cv, wlc->input_current1,
		wlc->charging_current1, uisoc);

out:
	wlc_dbg("%s: plugout=%d\n", __func__, wlc->is_cable_out_occur);

	wlc_dbg("%s: state = %s, ret = %d:%d\n", __func__,
		wlc_state_to_str(wlc->state),ret, ret_value);
	return ret_value;
}

static int _wlc_start_algo(struct chg_alg_device *alg)
{
	int ret = 0, ret_value = 0;
	struct moto_wlc *wlc;
	bool again;
	wlc_dbg("%s \n", __func__);
	wlc = dev_get_drvdata(&alg->dev);
	mutex_lock(&wlc->access_lock);
	__pm_stay_awake(wlc->suspend_lock);

	do {
		wlc_info("%s state:%d %s %d\n", __func__, wlc->state,
			 wlc_state_to_str(wlc->state), again);

		again = false;
		switch (wlc->state) {
		case WLC_HW_UNINIT:
		case WLC_HW_FAIL:
			ret_value = ALG_INIT_FAIL;
			break;
		case WLC_HW_READY:
			ret = __wlc_check_charger(alg);
			if (ret == 0) {
				wlc->state = WLC_RUN;
				ret_value = ALG_READY;
				again = true;
			} else {
				wlc->state = WLC_TA_NOT_SUPPORT;
				ret_value = ALG_TA_NOT_SUPPORT;
			}
			break;
		case WLC_TA_NOT_SUPPORT:
			ret_value = ALG_TA_NOT_SUPPORT;
			break;
		case WLC_RUN:
			ret = __wlc_run(alg);
			if (ret == ALG_TA_NOT_SUPPORT)
				wlc->state = WLC_TA_NOT_SUPPORT;
			else if (ret == ALG_DONE)
				wlc->state = WLC_HW_READY;
			ret_value = ret;
			break;
		default:
			wlc_err("WLC unknown state:%d\n", wlc->state);
			ret_value = ALG_INIT_FAIL;
			break;
		}
	} while (again == true);
	__pm_relax(wlc->suspend_lock);
	mutex_unlock(&wlc->access_lock);

	return ret_value;
}

static bool _wlc_is_algo_running(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;

	wlc_dbg("%s\n", __func__);
	wlc = dev_get_drvdata(&alg->dev);

	if (wlc->state == WLC_RUN)
		return true;

	return false;
}

static int _wlc_stop_algo(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;

	wlc = dev_get_drvdata(&alg->dev);

	wlc_dbg("%s %d\n", __func__, wlc->state);
	if (wlc->state == WLC_RUN) {
		wlc_reset_ta_vchr(alg);
		wlc->state = WLC_HW_READY;
	}

	return 0;
}

static int wlc_full_event(struct chg_alg_device *alg)
{
	struct moto_wlc *wlc;
	int ret_value = 0;
	wlc_dbg("%s \n", __func__);
	wlc = dev_get_drvdata(&alg->dev);
	switch (wlc->state) {
	case WLC_HW_UNINIT:
	case WLC_HW_FAIL:
	case WLC_HW_READY:
	case WLC_TA_NOT_SUPPORT:
		break;
	case WLC_RUN:
		if (wlc->state == WLC_RUN) {
			wlc_err("%s evt full\n", __func__);
			wlc_leave(alg);
			wlc->state = WLC_DONE;
		}
		break;
	default:
		wlc_dbg("%s: error state:%d\n", __func__, wlc->state);
		break;
	}

	return ret_value;
}

static int _wlc_notifier_call(struct chg_alg_device *alg,
			      struct chg_alg_notify *notify)
{
	struct moto_wlc *wlc;
	int ret_value = 0;

	wlc = dev_get_drvdata(&alg->dev);
	wlc_err("%s evt:%d state:%s\n", __func__, notify->evt,
		wlc_state_to_str(wlc->state));

	switch (notify->evt) {
	case EVT_PLUG_OUT:
		wlc->stop_6pin_re_en = 0;
		ret_value = wlc_plugout_reset(alg);
		break;
	case EVT_FULL:
		wlc->stop_6pin_re_en = 1;
		ret_value = wlc_full_event(alg);
		break;
	case EVT_BATPRO_DONE:
		wlc->wlc_6pin_en = 0;
		ret_value = 0;
		break;
	case EVT_RECHARGE:
		if (wlc->state == WLC_DONE) {
			wlc_err("%s evt recharge\n", __func__);
			moto_wlc_control_gpio(alg, false);
			wlc->state = WLC_HW_READY;
		}
		break;
	default:
		ret_value = -EINVAL;
	}
	return ret_value;
}

int _wlc_get_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int *value)
{

	pr_notice("%s\n", __func__);
	if (s == ALG_MAX_VBUS)
		*value = 12000;
	else
		pr_notice("%s does not support prop:%d\n", __func__, s);
	return 0;
}

int _wlc_set_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int value)
{
	struct moto_wlc *wlc;

	pr_notice("%s %d %d\n", __func__, s, value);

	wlc = dev_get_drvdata(&alg->dev);

	switch (s) {
	case ALG_LOG_LEVEL:
		wlc_dbg_level = value;
		break;
	case ALG_REF_VBAT:
		wlc->ref_vbat = value;
		break;
	case ALG_WLC_STATE:
		moto_wlc_control_gpio(alg, !value);
		if (!value) {
			wlc_plugout_reset(alg);
		}
		break;
	case ALG_WLC_TX_MODE:
		break;
	case ALG_NOTIFY_OTG_PLUGIN:
		wls_chg_notify_otg_plugin(wlc, !!value);
		break;
	default:
		break;
	}

	return 0;
}

int _wlc_set_setting(struct chg_alg_device *alg_dev,
		     struct chg_limit_setting *setting)
{
	struct moto_wlc *wlc;

	wlc = dev_get_drvdata(&alg_dev->dev);

	wlc_dbg("%s cv:%d icl:%d,%d cc:%d,%d, mmi_fcc:%d, 6pin_en:%d\n",
		__func__, setting->cv, setting->input_current_limit1,
		setting->input_current_limit2, setting->charging_current_limit1,
		setting->charging_current_limit2, setting->mmi_fcc_limit,
		setting->vbat_mon_en);

	mutex_lock(&wlc->access_lock);
	__pm_stay_awake(wlc->suspend_lock);
	wlc->cv = setting->cv;
	wlc->wlc_6pin_en = setting->vbat_mon_en;
	wlc->input_current_limit1 = setting->input_current_limit1;
	wlc->input_current_limit2 = setting->input_current_limit2;
	wlc->charging_current_limit2 = setting->charging_current_limit2;
	wlc->mmi_fcc = setting->mmi_fcc_limit;

	__pm_relax(wlc->suspend_lock);
	mutex_unlock(&wlc->access_lock);

	return 0;
}

static struct chg_alg_ops wlc_alg_ops = {
	.init_algo = _wlc_init_algo,
	.is_algo_ready = _wlc_is_algo_ready,
	.start_algo = _wlc_start_algo,
	.is_algo_running = _wlc_is_algo_running,
	.stop_algo = _wlc_stop_algo,
	.notifier_call = _wlc_notifier_call,
	.get_prop = _wlc_get_prop,
	.set_prop = _wlc_set_prop,
	.set_current_limit = _wlc_set_setting,
};

static int wlc_tcd_get_max_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	struct moto_wlc *wlc = tcd->devdata;

	*state = wlc->max_state;

	return 0;
}

static int wlc_tcd_get_cur_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	struct moto_wlc *wlc = tcd->devdata;

	*state = wlc->cur_state;

	return 0;
}

static int wlc_tcd_set_cur_state(struct thermal_cooling_device *tcd,
	unsigned long state)
{
	struct moto_wlc *wlc = tcd->devdata;

	if (state > wlc->max_state)
		return -EINVAL;

	if (wlc->cur_state == state)
		return 0;

	if (!wlc->config.wls_cert_mode)
		wlc->charging_current_limit1 = wlc_state_to_current_limit[state];
	wlc->cur_state = state;

	wlc_info("%s cur state = %d, config state = %ld, cur limt = %d, wls_cert_mode = %d\n",
		__func__, wlc->cur_state, state, wlc->charging_current_limit1, wlc->config.wls_cert_mode);

	return 0;
}

static const struct thermal_cooling_device_ops wlc_tcd_ops = {
	.get_max_state = wlc_tcd_get_max_state,
	.get_cur_state = wlc_tcd_get_cur_state,
	.set_cur_state = wlc_tcd_set_cur_state,
};

#ifdef CONFIG_MOTO_PHONE_CASE_SUPPORT
static int phone_case_detection_notifier_call(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct moto_wlc *wlc =
		container_of(nb, struct moto_wlc, hall_nb);

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return NOTIFY_BAD;
	}

	switch (event) {
		case PHONE_CASE_DETECTION_MOUNTED:
			wlc->ctl.mc_status = true;
			break;
		case PHONE_CASE_DETECTION_UNMOUNTED:
			wlc->ctl.mc_status = false;
			break;
		default:
			break;
	}

	wls_rx_set_mc_det(wlc->wls_dev, !wlc->ctl.mc_status);
	wlc_info("%s mc_status=%d event=%ld\n", __func__, wlc->ctl.mc_status, event);

	return NOTIFY_OK;
}
#endif

static int moto_wlc_probe(struct platform_device *pdev)
{
	struct moto_wlc *wlc = NULL;
	int rc;

	pr_notice("%s: starts\n", __func__);

	wlc = devm_kzalloc(&pdev->dev, sizeof(*wlc), GFP_KERNEL);
	if (!wlc)
		return -ENOMEM;
	platform_set_drvdata(pdev, wlc);
	wlc->pdev = pdev;

	wlc->suspend_lock =
		wakeup_source_register(NULL, "WLC suspend wakelock");
	wlc->fw_update_wake_lock =
		wakeup_source_register(NULL, "WLC FW update wakelock");

	mutex_init(&wlc->access_lock);
	mutex_init(&wlc->cable_out_lock);
	mutex_init(&wlc->data_lock);

	wlc->chg1_dev = get_charger_by_name("primary_chg");
	if (IS_ERR_OR_NULL(wlc->chg1_dev)) {
		wlc_err("%s: Error : can't find primary charger\n", __func__);
	}

	wlc->state = WLC_HW_UNINIT;
	wls_config_parse_dts(wlc, &pdev->dev);

	if (wls_config_is_charge_only_mode(wlc))
		wls_device_init_light_fan(wlc);

	if(gpio_is_valid(wlc->wls_control_en)) {
		rc  = devm_gpio_request_one(&wlc->pdev->dev, wlc->wls_control_en,
				  GPIOF_OUT_INIT_LOW, "wls_control_en");
		if (rc  < 0)
			pr_err(" [%s] Failed to request wls_control_en gpio, ret:%d", __func__, rc);
	}

	wlc->alg = chg_alg_device_register("wlc", &pdev->dev, wlc, &wlc_alg_ops,
					   NULL);

	/* Register thermal zone cooling device */
	wlc->charging_current_limit1 = -1;
	wlc->max_state = CHARGER_STATE_NUM - 1;
	wlc->tcd = thermal_of_cooling_device_register(dev_of_node(&pdev->dev),
		"moto_wlc", wlc, &wlc_tcd_ops);

	wls_chg_register_psy(wlc);

	wls_device_tcmd_register(wlc);
	wls_device_node_create(&(wlc->pdev->dev));
	wls_auth_init(wlc);

	wlc->wls_wq = create_singlethread_workqueue("wls_workqueue");
	INIT_DELAYED_WORK(&wlc->fw_update_work, wls_device_fw_update_work);
	INIT_DELAYED_WORK(&wlc->bpp_icl_work, wlc_chg_bpp_mode_icl_work);
	INIT_DELAYED_WORK(&wlc->mc_icl_work, wlc_chg_mc_icl_work);
	INIT_DELAYED_WORK(&wlc->light_fan_work, wls_device_light_fan_work);
	INIT_DELAYED_WORK(&wlc->mode_switch_work, wls_device_mode_switch_work);
	if (wlc->config.enable_rx_offset_detect) {
		wlc->ctl.enable_rod = true;
		INIT_DELAYED_WORK(&wlc->offset_detect_work, wls_device_offset_detect_work);
	}
	INIT_DELAYED_WORK(&wlc->dump_info_work, wlc_chg_dump_info_work);

#ifdef CONFIG_MOTO_PHONE_CASE_SUPPORT
	rc = phone_case_detection_get_hall_state();
	if (rc == PHONE_CASE_DETECTION_MOUNTED) {
		wlc->ctl.mc_status = true;
	} else if (rc == PHONE_CASE_DETECTION_UNMOUNTED) {
		wlc->ctl.mc_status = false;
	} else {
		wlc_err("%s hall not enabled rc=%d\n", __func__, rc);
	}
	wlc->hall_nb.notifier_call = phone_case_detection_notifier_call;
	rc = phone_case_detection_register_client(&wlc->hall_nb);
	wlc_info("%s phone_case_detection_register_client rc=%d\n", __func__, rc);
#endif

	return 0;
}

static int moto_wlc_remove(struct platform_device *dev)
{
	struct moto_wlc *wlc;

	wlc = (struct moto_wlc  *)platform_get_drvdata(dev);
	if(wlc->tcd)
		thermal_cooling_device_unregister(wlc->tcd);

	return 0;
}

static void moto_wlc_shutdown(struct platform_device *dev)
{
}

static const struct of_device_id moto_wlc_of_match[] = {
	{
		.compatible = "moto,charger,wlc",
	},
	{},
};

MODULE_DEVICE_TABLE(of, moto_wlc_of_match);

struct platform_device wlc_device = {
	.name = "wlc",
	.id = -1,
};

static struct platform_driver wlc_driver = {
	.probe = moto_wlc_probe,
	.remove = moto_wlc_remove,
	.shutdown = moto_wlc_shutdown,
	.driver = {
		   .name = "wlc",
		   .of_match_table = moto_wlc_of_match,
	},
};

static int __init moto_wlc_init(void)
{
	return platform_driver_register(&wlc_driver);
}
module_init(moto_wlc_init);

static void __exit moto_wlc_exit(void)
{
	platform_driver_unregister(&wlc_driver);
}
module_exit(moto_wlc_exit);

MODULE_DESCRIPTION("moto wlc algorithm Driver");
MODULE_LICENSE("GPL");
