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
#include <mtk_charger.h>
#include <mtk_charger_algorithm_class.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <linux/mmi_wireless_class.h>
#include "moto_wlc2.h"

static int factory_test_wls_en(void *input, bool en);

void wls_device_pm_set_awake(struct moto_wlc *wlc, bool awake)
{
	if (IS_ERR_OR_NULL(wlc->fw_update_wake_lock))
		return;

	pr_info("%s %d\n", __func__, awake);
	if (!wlc->fw_update_wake_lock->active && awake) {
		__pm_stay_awake(wlc->fw_update_wake_lock);
	} else if(wlc->fw_update_wake_lock->active && !awake) {
		__pm_relax(wlc->fw_update_wake_lock);
	}
}

bool wls_device_fw_set_boost(struct moto_wlc *wlc, bool en)
{
	int ret = 0;
	int vbus = 0;
	struct charger_device *chg_psy = NULL;

	wlc_info("%s en=%d\n", __func__, en);
	if (en) {
		wlc->ctl.otg_boost_on = true;
		wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_FW_UPDATE, true);
	}

	if (!wlc->config.wls_boost_support) {
		chg_psy = get_charger_by_name("primary_chg");
		if (IS_ERR_OR_NULL(chg_psy)) {
			wlc_err("%s Couldn't get chg_psy\n", __func__);
			goto boost_err_out;
		}
		ret = charger_dev_enable_otg(chg_psy, en);
		if(ret < 0){
			wlc_err("%s set otg fail\n", __func__);
			goto boost_err_out;
		}
		msleep(100);
		vbus = wlc_hal_get_vbus(wlc->alg) / 1000;
		wlc_info("%s vbus:%dmV\n", __func__, vbus);
	}

	if (!en) {
		wlc->ctl.otg_boost_on = false;
		wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_FW_UPDATE, false);
	}

	return true;
boost_err_out:
	wlc->ctl.otg_boost_on = false;
	return false;
}

void wls_device_fw_update_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, fw_update_work);
	union power_supply_propval prop = {0x00};
	int chip_id = 0x00;
	int rt = 0;
	int vbus = 0;
	int soc = 0;
	bool boost_flag = false;
	int status = WLS_FW_UPDATE_IDLE;
	bool otg_en = false;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("can't get wlc\n");
		return;
	}

	if (IS_ERR_OR_NULL(wlc->wls_dev)) {
		wlc_err("can't get wls_dev\n");
		return;
	}

	if (wlc_hal_get_bat_property(wlc->alg, POWER_SUPPLY_PROP_CAPACITY, &prop)) {
		wlc_err("can't get battery capacity\n");
		return;
	}

	soc = prop.intval;
	vbus = wlc_hal_get_vbus(wlc->alg) / 1000;

	if (wlc->config.secure_hardware) {
		wlc_info("%s Skip FW update when secure phone\n", __func__);
		return;
	} else if ((vbus < VBUS_VALID_MV/2) &&
			soc < wlc->config.fw_update_soc_limit &&
			!wlc->ctl.fw_update_force) {
		wlc_info("%s Wireless fw update failed. Battery SOC should be at least %d%%\n",
				__func__, wlc->config.fw_update_soc_limit);
		return;
	} else {
		if (!IS_ERR_OR_NULL(wlc->chg1_dev)) {
			rt = charger_dev_is_otg_enabled(wlc->chg1_dev, &otg_en);
			if (rt) {
				wlc_err("%s get otg status failed, rt:%d\n", __func__, rt);
				return;
			}
			if (otg_en) {
				wlc_info("%s Skip FW update when otg plug-in\n", __func__);
				return;
			}
		}
	}

	//read chip
	rt = wls_rx_get_chip_id(wlc->wls_dev, &chip_id);
	wlc_info("%s chip_id=%d, rt=%d\n", __func__, chip_id, rt);

	if (rt) {
		if (vbus >= VBUS_VALID_MV/2) {
			wlc_err("%s vbus exists but iic err, Skip\n", __func__);
			return;
		} else
			boost_flag = true; //no vbus and iic err, need boost
	}

	wlc->ctl.fw_uploading = true;
	wls_device_pm_set_awake(wlc, true);
	if (boost_flag) {
		wls_device_fw_set_boost(wlc, true);
	}

	rt = wls_rx_get_chip_id(wlc->wls_dev, &chip_id);// recheck iic
	wlc_info("%s chip_id=%d rt=%d\n", __func__, chip_id, rt);
	if (rt) {
		wlc_err("%s Error: boost failed or usb plug-out, rt=%d\n", __func__, rt);
		goto fw_update_exit;
	}
	wls_rx_set_fw_update(wlc->wls_dev, wlc->ctl.fw_update_force);

fw_update_exit:
	if (boost_flag) {
		wls_device_fw_set_boost(wlc, false);
	}

	wlc->ctl.fw_uploading = false;
	wls_device_pm_set_awake(wlc, false);

	wls_rx_get_fw_update_status(wlc->wls_dev,&status);
	if (status != WLS_FW_UPDATE_IDLE &&
		status != WLS_FW_UPDATE_SKIP &&
		status != WLS_FW_UPDATE_SUCCESS &&
		wlc->ctl.fw_update_retry_cnt < 2) {
		wlc->ctl.fw_update_retry_cnt ++;
		wlc_info("%s fw_update_retry_cnt=%d\n", __func__, wlc->ctl.fw_update_retry_cnt);
		queue_delayed_work(wlc->wls_wq,
				&wlc->fw_update_work, msecs_to_jiffies(1000));
	}
}

int wls_device_uisoc_change(struct moto_wlc *wlc, int uisoc)
{
	if (uisoc == 100 || wlc->data.uisoc == 100) {
		queue_delayed_work(wlc->wls_wq, &wlc->light_fan_work, msecs_to_jiffies(0));
	}

	return 0;
}

int wls_device_init_light_fan(struct moto_wlc *wlc)
{
	wlc->ctl.light_level = MMI_DOCK_LIGHT_ON;
	wlc->ctl.fan_speed = MMI_DOCK_FAN_DEFAULT;
	return 0;
}

int wls_device_update_light_fan(struct moto_wlc *wlc)
{
	int status = 0;
	uint8_t data[4] = {0x38, 0x05, 0x40, 0x28};
	int retry = 2;

	if (100 == wlc_hal_get_uisoc(wlc->alg)) {
		if (wlc->ctl.light_level == 0)
			data[2] = MMI_DOCK_LIGHT_OFF;
		else
			data[2] = MMI_DOCK_LIGHT_ON;
		data[3] = MMI_DOCK_FAN_SPEED_OFF;
	} else {
		if (wlc->ctl.fan_speed == 0)
			data[3] = MMI_DOCK_FAN_SPEED_LOW;
		else
			data[3] = MMI_DOCK_FAN_SPEED_HIGH;

		if (wlc->ctl.light_level == 0)
			data[2] = MMI_DOCK_LIGHT_OFF;
		else
			data[2] = MMI_DOCK_LIGHT_DEFAULT;
	}

	do {
		status = wls_rx_send_ask_packet(wlc->wls_dev, data, sizeof(data));
		wlc_info("%s: QI set fan/light, FAN_SPEED %d, LIGHT %d",
					__func__, wlc->ctl.fan_speed, wlc->ctl.light_level);
		wlc_info("%s: QI set fan/light, ight 0x%x, fan 0x%x", __func__, data[2], data[3]);
		msleep(200);
		retry--;
	} while (retry);

	return status;
}

int wls_device_notify_tx_chgfull(struct moto_wlc *wlc)
{
	int status = 0;
	uint8_t data[2] = {0x5, 0x64};
	int retry = 3;

	do {
		status = wls_rx_send_ask_packet(wlc->wls_dev, data, sizeof(data));
		wlc_info("%s: QI notify TX battery full , head 0x%x, cmd 0x%x, status %d\n",
				__func__, data[0], data[1], status);
		msleep(200);
		retry--;
	} while(retry);

	return status;
}

void wls_device_light_fan_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, light_fan_work);

	pr_info("%s hs_st:%d auth_done:%d\n", __func__, wlc->auth.hs_st, wlc->auth.auth_done);
	if (wlc->auth.hs_st == AUTH_HS_OK) {
		wls_device_update_light_fan(wlc);
	} else if (100 == wlc_hal_get_uisoc(wlc->alg)) {
		wls_device_notify_tx_chgfull(wlc);
	}
}

bool wls_device_check_iout(struct moto_wlc *wlc, int target_current, int current_now)
{
	bool skip_rod = false;
	bool chg_en = true;
	bool powerpath_en = true;
	int wls_icl = 0;

	if (!IS_ERR_OR_NULL(wlc->chg1_dev)) {
		charger_dev_is_enabled(wlc->chg1_dev, &chg_en);
		charger_dev_is_powerpath_enabled(wlc->chg1_dev, &powerpath_en);
		charger_dev_get_input_current(wlc->chg1_dev, &wls_icl);
		wls_icl = wls_icl / 1000;// uA-> mA
	}

	if (!chg_en || !powerpath_en || wls_icl <= target_current) {
		skip_rod = true;
		pr_info("%s chg_en:%d, powerpath_en:%d wls_icl:%d\n",
				__func__, chg_en, powerpath_en, wls_icl);
	} else if (wlc->cur_state > 0) {
		skip_rod = true;
	} else if (wlc->config.rod_stop_battery_soc > 0) {
		skip_rod = (wlc_hal_get_uisoc(wlc->alg) > wlc->config.rod_stop_battery_soc) ? true : false;
	} else {
		skip_rod = false;
	}

	wlc_info("[%s] skip:%d now:%dmA target:%dmA cur_state:%d\n",
				__func__, skip_rod, current_now, target_current, wlc->cur_state);

	return !skip_rod && (current_now < target_current);
}

void wlc_chg_rx_online_check(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, rx_online_check);
	int chip_id = 0;
	int sys_mode = 0;
	int vbus = 0;
	bool rx_power = false;
	static int rx_power_cnt = 0;

	if (IS_ERR_OR_NULL(wlc))
		return;

	vbus = wlc_hal_get_vbus(wlc->alg) / 1000;
	wlc_info("%s vbus:%dmV\n", __func__, vbus);
	if(vbus > VBUS_VALID_MV) {
		wlc_info("%s: Rx power on, LDO on\n", __func__);
		return;
	}

	wls_rx_get_chip_id(wlc->wls_dev, &chip_id);
	wls_rx_get_sys_mode(wlc->wls_dev, &sys_mode);
	wlc_info("%s: sys_mode = %d, chip_id = %d\n", __func__, sys_mode, chip_id);
	if (chip_id == wlc->config.chip_id && sys_mode == SYS_MODE_RX) {
		rx_power = true;
		rx_power_cnt = 0;
	} else {
		rx_power = false;
		rx_power_cnt++;
	}

	if (rx_power) {
		wlc_info("%s: Rx power on, Re-check after 500ms\n", __func__);
		queue_delayed_work(wlc->wls_wq, &wlc->rx_online_check, msecs_to_jiffies(RX_ONLINE_CHECK_MS));
		return;
	} else {
		if (rx_power_cnt > 1) {
			wlc_info("%s: Rx power off\n", __func__);
			rx_power_cnt = 0;
			wlc->ctl.bpp_icl_done = false;
			wlc->ctl.mc_icl_state = MC_ICL_IDLE;
			wlc->ctl.ce_det_count = 0;
			wlc->data.mcode = 0x00;
			wls_chg_power_off(wlc);
			wls_chg_notify_st_changed(wlc, WLC_DISCONNECTED);

			return ;
		} else {
			queue_delayed_work(wlc->wls_wq, &wlc->rx_online_check, msecs_to_jiffies(RX_ONLINE_CHECK_MS));
			wlc_info("%s: Re-check after 500ms\n", __func__);
			return;
		}
	}
}

#define OFFSET_DETECT_TIME_DELAY_DEFAULT_MS 1000

void wls_device_offset_detect_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, offset_detect_work);
	int rt = 0;
	int chip_id = 0;

	int sys_mode = 0;
	int op_mode = 0;
	static int work_timedelay = 1000;
	static int wls_current = 0; //uA

	if (IS_ERR_OR_NULL(wlc))
		return;

	if (IS_ERR_OR_NULL(wlc->chg1_dev)) {
		wlc->chg1_dev = get_charger_by_name("primary_chg");
		if (wlc->chg1_dev)
			wlc_info("%s: Found primary charger\n", __func__);
		else {
			wlc_err("%s: Error : can't find primary charger\n",__func__);
		}
	}

	rt = wls_rx_get_chip_id(wlc->wls_dev, &chip_id);
	if (chip_id != wlc->config.chip_id) { //chip not power on
		wlc_info("%s: chip not power on\n", __func__);
		return;
	}

	if (!wlc->ctl.rx_ldo_on) {
		rt = wls_rx_get_sys_mode(wlc->wls_dev, &sys_mode);
		if (sys_mode == SYS_MODE_RX) {
			wlc->ctl.rx_ldo_detect_count ++;
			if (wlc->ctl.rx_ldo_detect_count >= 3 &&
				!wlc->ctl.rx_offset) {
				wlc->ctl.rx_offset = true;
				wls_chg_notify_st_changed(wlc, WLC_ERR_LOWER_EFFICIENCY);
				pr_info("%s wlc->ctl.rx_offset=%d\n", __func__, wlc->ctl.rx_offset);
			}
			if (ktime_get_boottime() - wlc->ctl.rx_start_ktime >= WLS_ROD_STOP_TIME) {
				wlc_info("[%s] rx_ldo_on detect stop,wls status:%d\n",
					__func__, wlc->data.wlc_status);
				wlc->ctl.rod_stop = true;
				wlc->ctl.rx_offset = false;
			} else
				queue_delayed_work(wlc->wls_wq, &wlc->offset_detect_work, msecs_to_jiffies(1000));
			return;
		} else { //Disconnected
			wlc->ctl.rx_offset_detect_count = 0;
			wlc->ctl.rx_offset = false;
			wlc_info("[%s] rx_ldo_on:%d,rod_stop:%d",
				__func__, wlc->ctl.rx_ldo_on, wlc->ctl.rod_stop);
			return;
		}
	}

	if (ktime_get_boottime() - wlc->ctl.rx_start_ktime >= WLS_ROD_STOP_TIME) {
		wlc_info("[%s] rx offset detect stop,wls status:%d\n", __func__, wlc->data.wlc_status);
		wlc->ctl.rod_stop = true;
		wlc->ctl.rx_offset_detect_count = 0;
		wlc->ctl.rx_offset = false;
		goto detect_exit;
	}

	rt = wls_rx_get_rx_neg_power(wlc->wls_dev, &wlc->data.rx_neg_power);
	if (rt) {
		wlc_info("[%s] err rt:%d\n", __func__, rt);
		return;
	}
	rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
	wlc_info("[%s] op_mode:%d rx_neg_power:%d\n", __func__, op_mode, wlc->data.rx_neg_power);

	if ((op_mode == Sys_Op_Mode_BPP && wlc->ctl.bpp_icl_done) ||
		(op_mode == Sys_Op_Mode_EPP && wlc->data.rx_neg_power == WLS_RX_CAP_5W)) {

		rt = wls_rx_get_rx_irect(wlc->wls_dev, &wlc->data.rx_irect);
		if (!wlc->ctl.rx_offset) {
			if (wlc->cur_state > 0) {
				wlc->ctl.rod_stop = true;
				wlc->ctl.rx_offset_detect_count = 0;
				goto detect_exit;
			}
			if (wls_device_check_iout(wlc, WLS_BPP_ROD_THRESHOLD_CURRENT_MIN, wlc->data.rx_irect)) {
				wlc->ctl.rx_offset_detect_count ++;
			} else {
				wlc->ctl.rx_offset_detect_count = 0;
			}
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS;
			if (wlc->ctl.rx_offset_detect_count >= WLS_BPP_ROD_DETECT_COUNT_MAX) {
				wlc->ctl.rx_offset = true;
				wlc->ctl.rx_offset_detect_count = 0;
				wlc_info("[%s] offset true\n", __func__);
				if (wlc->data.wlc_status != WLC_DISCONNECTED)
					wls_chg_notify_st_changed(wlc, WLC_ERR_LOWER_EFFICIENCY);
				if (wlc->chg1_dev) {
					wls_current = wlc->data.rx_irect * 1000 / wlc->config.bpp_icl_step_uA
									* wlc->config.bpp_icl_step_uA;
					if (wls_current < wlc->config.bpp_icl_min_uA)
						wls_current = wlc->config.bpp_icl_min_uA;
					charger_dev_set_input_current(wlc->chg1_dev, wls_current);
					work_timedelay = WLS_ICL_INCREASE_DELAY;
				}
			}
		} else {
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS / 2;
			if (wls_current < wlc->config.bpp_icl_max_uA) {
				if (wlc->chg1_dev) {
					wlc_info("[%s] wls_current = %dmA rx_irect=%dmA\n",
							__func__, wls_current, wlc->data.rx_irect);
					wls_current = wls_current + wlc->config.bpp_icl_step_uA;
					charger_dev_set_input_current(wlc->chg1_dev, wls_current * 1000);
					work_timedelay = WLS_ICL_INCREASE_DELAY;
				}
			} else {
				if (!wls_device_check_iout(wlc, WLS_BPP_ROD_THRESHOLD_CURRENT_MAX, wlc->data.rx_irect)) {
					wlc->ctl.rx_offset_detect_count ++;
				} else {
					wlc->ctl.rx_offset_detect_count = 0;
				}
				if (wlc->ctl.rx_offset_detect_count >= WLS_BPP_ROD_DETECT_COUNT_MAX) {
					wlc->ctl.rx_offset = false;
					wlc_info("[%s] offset exit\n", __func__);
					wlc->ctl.rx_offset_detect_count = 0;
					if (wlc->data.wlc_status != WLC_DISCONNECTED)
						wls_chg_notify_st_changed(wlc, WLC_CHGING);
					wlc->ctl.rod_stop = true;
				}
			}
		}
	} else if(op_mode == Sys_Op_Mode_EPP) {
		/*For EPP*/
		rt = wls_rx_get_rx_vout(wlc->wls_dev, &wlc->data.rx_vout);
		rt = wls_rx_get_rx_vout_setting(wlc->wls_dev, &wlc->data.rx_vout_set);

		if (wlc->data.rx_vout_set == 12000)
			wlc->data.rx_vout_threshold = WLS_EPP_ROD_THRESHOLD_12V;
		else if (wlc->data.rx_vout_set == 10000)
			wlc->data.rx_vout_threshold = WLS_EPP_ROD_THRESHOLD_10V;
		else if (wlc->data.rx_vout_set == 9000)
			wlc->data.rx_vout_threshold = WLS_EPP_ROD_THRESHOLD_9V;
		else
			wlc->data.rx_vout_threshold = wlc->data.vbus_select * 80 / 100;

		wlc_info("[%s] vout:%dmV vout_set:%dmV vout_threshold:%dmV vbus_select:%dmV\n", __func__,
				wlc->data.rx_vout, wlc->data.rx_vout_set, wlc->data.rx_vout_threshold, wlc->data.vbus_select);
		if (!wlc->ctl.rx_offset) {
			if (wlc->data.rx_vout < wlc->data.rx_vout_threshold) {
				wlc->ctl.rx_offset_detect_count ++;
			} else {
				wlc->ctl.rx_offset_detect_count = 0;
			}
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS;
			if (wlc->ctl.rx_offset_detect_count >= WLS_EPP_ROD_DETECT_COUNT_MAX) {
				wlc->ctl.rx_offset = true;
				wlc->ctl.rx_offset_detect_count = 0;
				wlc_info("[%s] EPP offset true\n", __func__);
				if (wlc->data.wlc_status != WLC_DISCONNECTED)
					wls_chg_notify_st_changed(wlc, WLC_ERR_LOWER_EFFICIENCY);
				work_timedelay = WLS_ICL_INCREASE_DELAY;
			}
		} else {
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS / 2;
			if (wlc->data.rx_vout > wlc->data.rx_vout_threshold + 500) {
				wlc->ctl.rx_offset_detect_count ++;
			} else {
				wlc->ctl.rx_offset_detect_count = 0;
			}
			if (wlc->ctl.rx_offset_detect_count >= WLS_EPP_ROD_DETECT_COUNT_MAX) {
				wlc->ctl.rx_offset = false;
				wlc_info("[%s] EPP offset exit\n", __func__);
				wlc->ctl.rx_offset_detect_count = 0;
				if (wlc->data.wlc_status != WLC_DISCONNECTED)
					wls_chg_notify_st_changed(wlc, WLC_CHGING);
				wlc->ctl.rod_stop = true;
			}
		}
	}

	wlc_info("[%s] rx_offset_detect_count:%d rod_stop:%d\n",
			__func__, wlc->ctl.rx_offset_detect_count, wlc->ctl.rod_stop);

	if (!wlc->ctl.rod_stop) {
		queue_delayed_work(wlc->wls_wq, &wlc->offset_detect_work, msecs_to_jiffies(work_timedelay));
		return;
	} else if (wlc->ctl.rx_offset) {
		wlc->ctl.rx_offset_detect_count = 0;
		wlc->ctl.rx_offset = false;
	}

detect_exit:
	return;
}

int wls_device_set_mode_select(struct moto_wlc *wlc, char *str, bool mode)
{
	int rt = 0;

	if (wlc->config.wls_cert_mode && !mode) {
		wlc_info("%s: %s wls_cert_mode can't set mode_sel low\n", __func__, str);
	} else {
		rt = wls_rx_set_mode_select(wlc->wls_dev, mode);
		wlc_info("%s: %s mode:%d rt:%d\n", __func__, str, mode, rt);
	}
	return rt;
}

void wls_device_mode_switch_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, mode_switch_work);
	static int pre_mode = 0;
	int rt = 0;
	int switch_time = 2000;
	int op_mode = 0;
	int mode_sel = -1;

	rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);

	switch (wlc->ctl.mode_switch)
	{
	case WLC_SWITCH_TO_BPP:
		if (rt == 0 && op_mode == Sys_Op_Mode_EPP) {
			switch_time = wlc->config.bpp_switch_time_ms;
			pre_mode = op_mode;
			wlc->ctl.mode_switch = WLC_SWITCH_RUN;
			rt = wls_device_set_mode_select(wlc, "switch to BPP", 0);
		}
		break;
	case WLC_SWITCH_TO_EPP:
		if (rt == 0 && op_mode == Sys_Op_Mode_BPP) {
			switch_time = wlc->config.epp_switch_time_ms;
			pre_mode = op_mode;
			wlc->ctl.mode_switch = WLC_SWITCH_RUN;
			rt = wls_device_set_mode_select(wlc, "switch to EPP", 1);
		}
		break;
	case WLC_SWITCH_RUN:
		if (rt < 0) {
			wlc->ctl.mode_switch = WLC_SWITCH_TIME_OUT;
			rt = wls_rx_get_mode_select(wlc->wls_dev, &mode_sel);
			if (rt == 0 && mode_sel == 0) {
				rt = wls_device_set_mode_select(wlc, "Timeout reset to EPP", 1);
			}
		} else if (op_mode == pre_mode) {
			wlc->ctl.mode_switch = WLC_SWITCH_FAIL;
		} else {
			wlc->ctl.mode_switch = WLC_SWITCH_DONE;
		}

		pr_info("%s op_mode:%d mode_switch:%d rt:%d\n",
				__func__, op_mode, wlc->ctl.mode_switch, rt);

		break;
	default:
		break;
	}

	pr_info("%s op_mode:%d mode_switch:%d rt:%d\n",
			__func__, op_mode, wlc->ctl.mode_switch, rt);

	if (wlc->ctl.mode_switch == WLC_SWITCH_RUN) {
		queue_delayed_work(wlc->wls_wq, &wlc->mode_switch_work, msecs_to_jiffies(switch_time));
	} else {
		wlc->ctl.mode_switch = WLC_SWITCH_IDLE;
	}
}

int wls_device_start_mode_switch(struct moto_wlc *wlc, char *str, int op_mode)
{
	int mode_sel = -1;
	int rt = 0;

	rt = wls_rx_get_mode_select(wlc->wls_dev, &mode_sel);
	wlc_info("%s: %s op_mode:%d mode_sel:%d rt:%d\n",
			__func__, str, op_mode, mode_sel, rt);
	if (rt < 0) {
		wlc_err("%s: mode_sel is invalid\n", __func__);
	} else if ((mode_sel == 0 && op_mode == Sys_Op_Mode_BPP) ||
				(mode_sel == 1 && op_mode == Sys_Op_Mode_EPP)) {
		wlc_info("%s: skip change mode_sel\n", __func__);
	} else if (wlc->ctl.mode_switch == WLC_SWITCH_IDLE) {
		if (op_mode == Sys_Op_Mode_BPP) {
			wlc->ctl.mode_switch = WLC_SWITCH_TO_BPP;
		} else if (op_mode == Sys_Op_Mode_EPP) {
			wlc->ctl.mode_switch = WLC_SWITCH_TO_EPP;
		} else {
			wlc_err("%s: op_mode:%d is invalid\n", __func__, op_mode);
			return rt;
		}
		wlc_info("%s: wlc->mode_switch_work=%p\n", __func__, &wlc->mode_switch_work);
		queue_delayed_work(wlc->wls_wq, &wlc->mode_switch_work, msecs_to_jiffies(10));
	}

	return rt;
}

static ssize_t wireless_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int fw_version = 0x00;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	if (pWlc->data.wls_fw_version != 0) {
		fw_version = pWlc->data.wls_fw_version;
	} else {
		wls_rx_get_fw_version(pWlc->wls_dev, &fw_version);
		pWlc->data.wls_fw_version = fw_version;
	}

	return sprintf(buf, "%08x\n", fw_version);
}
static DEVICE_ATTR(wireless_fw_version, 0444, wireless_fw_version_show, NULL);

static ssize_t wireless_fw_update_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int update = 0;
	int sys_mode = 0;
	int chip_id = 0x00;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	if (kstrtoint(buf, 0, &update) ||
		(update != NORMAL_FW_UPDATE && update != FORCE_FW_UPDATE)) {
		wlc_err("%s: invalid FW update mode %d\n", __func__, update);
		return -EINVAL;
	}

	pWlc->ctl.fw_update_retry_cnt = 0;
	wls_rx_get_sys_mode(pWlc->wls_dev, &sys_mode);
	wls_rx_get_chip_id(pWlc->wls_dev, &chip_id);
	if (sys_mode == SYS_MODE_RX && chip_id == pWlc->config.chip_id) {
		wlc_info("wireless_fw_update wls online,forbid fw update\n");
	} else if (!pWlc->ctl.fw_uploading) {
		pWlc->ctl.fw_update_force = (update == FORCE_FW_UPDATE ? true : false);
		wlc_info("%s pWlc->ctl.fw_update_force=%d\n", __func__, pWlc->ctl.fw_update_force);
		queue_delayed_work(pWlc->wls_wq,
				&pWlc->fw_update_work, msecs_to_jiffies(2000));
	}

	return count;
}
static DEVICE_ATTR(wireless_fw_update, 0220, NULL, wireless_fw_update_store);

static ssize_t mode_select_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	bool val = 0;
	int ret = 0;
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	if (kstrtobool(buf, &val)) {
		pWlc->ctl.mode_select_force = false;
		wlc_info("mode_select_store force exit\n");
		return -EINVAL;
	}

	pWlc->ctl.mode_select_force = val;
	ret = wls_device_set_mode_select(pWlc, "mode_select_store", val);

	wlc_info("mode_select_store %d, wls_online:%d force:%d ret:%d\n",
			val , pWlc->wls_online, pWlc->ctl.mode_select_force, ret);

	return count;
}
static DEVICE_ATTR(mode_select, 0220, NULL, mode_select_store);

static ssize_t mode_switch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	bool val = 0;
	int ret = 0;
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip is invalid\n", __func__);
		return -ENODEV;
	}
	if (kstrtobool(buf, &val)) {
		wlc_info("%s: val is invalid\n", __func__);
		return -EINVAL;
	}

	ret = wls_device_start_mode_switch(pWlc,
			"mode_switch_store", val? Sys_Op_Mode_EPP : Sys_Op_Mode_BPP);

	wlc_info("%s switch to %s ret:%d\n", __func__, val? "EPP" : "BPP" , ret);

	return count;
}
static DEVICE_ATTR(mode_switch, 0220, NULL, mode_switch_store);

static ssize_t show_rx_irect(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	int rx_irect = 0;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	wls_rx_get_rx_irect(pWlc->wls_dev, &rx_irect);

	return sprintf(buf, "%d\n", rx_irect);
}
static DEVICE_ATTR(get_rx_irect, 0444, show_rx_irect, NULL);

static ssize_t show_rx_vrect(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	int rx_vrect = 0;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	wls_rx_get_rx_vrect(pWlc->wls_dev, &rx_vrect);

	return sprintf(buf, "%d\n", rx_vrect);
}
static DEVICE_ATTR(get_rx_vrect, 0444, show_rx_vrect, NULL);

static ssize_t show_rx_vout(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	int rx_vout = 0;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	wls_rx_get_rx_vout(pWlc->wls_dev, &rx_vout);

	return sprintf(buf, "%d\n", rx_vout);
}
static DEVICE_ATTR(get_rx_vout, 0444, show_rx_vout, NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_input_curr_max;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_input_curr_max);
	if (r) {
		wlc_err("Invalid current = %lu\n", wls_input_curr_max);
		return -EINVAL;
	}

	wlc_info("wls input_current = %lumA\n", wls_input_curr_max);
	if (wls_input_curr_max > 0) {
		wlc_hal_set_input_current(pWlc->alg, CHG1, wls_input_curr_max * 1000);
		pWlc->ctl.input_current_max = wls_input_curr_max;
	}

	return r ? r : count;
}

static ssize_t wls_input_current_limit_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->ctl.input_current_max);
}
static DEVICE_ATTR(wls_input_current_limit, S_IRUGO|S_IWUSR, wls_input_current_limit_show, wls_input_current_limit_store);


static ssize_t show_wlc_fan_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	return sprintf(buf, " %d\n", pWlc->ctl.fan_speed);
}

static ssize_t store_wlc_fan_speed(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	if (!pWlc->data.moto_stand) {
		wlc_err("[%s] not moto 50w dock %d, skip\n", __func__ , pWlc->data.mode_type);
		return count;
	}

	pWlc->ctl.fan_speed = simple_strtoul(buf, NULL, 0);
	wlc_info("[%s] fan_speed %d\n", __func__ , pWlc->ctl.fan_speed);
	queue_delayed_work(pWlc->wls_wq, &pWlc->light_fan_work, msecs_to_jiffies(0));

	return count;
}
static DEVICE_ATTR(wlc_fan_speed, S_IRUGO|S_IWUSR, show_wlc_fan_speed, store_wlc_fan_speed);

static ssize_t show_wlc_light_ctl(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	return sprintf(buf, " %d\n", pWlc->ctl.light_level);
}

static ssize_t store_wlc_light_ctl(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	if (!pWlc->data.moto_stand) {
		wlc_err("[%s] not moto dock %d, skip\n",
				__func__ , pWlc->data.mode_type);
		return count;
	}

	pWlc->ctl.light_level = simple_strtoul(buf, NULL, 0);
	wlc_info("[%s] light_level %d\n", __func__ , pWlc->ctl.light_level);
	queue_delayed_work(pWlc->wls_wq, &pWlc->light_fan_work, msecs_to_jiffies(0));

	return count;
}
static DEVICE_ATTR(wlc_light_ctl, S_IRUGO|S_IWUSR, show_wlc_light_ctl, store_wlc_light_ctl);

static ssize_t show_wlc_tx_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	int power = 0;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	wls_rx_get_rx_neg_power(pWlc->wls_dev, &power);

	return sprintf(buf, "%d\n", power);
}
static DEVICE_ATTR(wlc_tx_power, 0444, show_wlc_tx_power, NULL);

static ssize_t show_wlc_tx_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	if (pWlc->data.moto_stand)
		return sprintf(buf, "%d\n", WLC_MOTO);
	else
		return sprintf(buf, "%d\n", pWlc->data.mode_type);
}

static DEVICE_ATTR(wlc_tx_type, 0444, show_wlc_tx_type, NULL);

static ssize_t show_wlc_st_changed(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	wlc_err("%s status:%d\n", __func__, pWlc->data.wlc_status);
	return sprintf(buf, "%d\n", pWlc->data.wlc_status);
}

static DEVICE_ATTR(wlc_st_changed, S_IRUGO, show_wlc_st_changed, NULL);

static ssize_t show_wlc_tx_capability(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->auth.wlc_tx_capability);
}

static DEVICE_ATTR(wlc_tx_capability, 0444, show_wlc_tx_capability, NULL);

static ssize_t show_wlc_tx_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->auth.wlc_tx_id);
}

static DEVICE_ATTR(wlc_tx_id, 0444, show_wlc_tx_id, NULL);

static ssize_t show_wlc_tx_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->auth.wlc_tx_sn);
}
static DEVICE_ATTR(wlc_tx_sn, 0444, show_wlc_tx_sn, NULL);


static ssize_t factory_wireless_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long en;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &en);
	if (r) {
		wlc_err("Invalid factory_wls_en = %lu\n", en);
		return -EINVAL;
	}

	factory_test_wls_en(pWlc, en);

	return r ? r : count;
}

static ssize_t factory_wireless_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	if (IS_ERR_OR_NULL(pWlc) || IS_ERR_OR_NULL(pWlc->wls_dev)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->ctl.factory_wls_en);
}
static DEVICE_ATTR(factory_wireless_en, S_IRUGO|S_IWUSR, factory_wireless_en_show, factory_wireless_en_store);

static ssize_t inhibit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int en = 0;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	if (buf[0] == '0') {
		en = 0;
	} else if (buf[0] == '1') {
		en = 1;
	} else {
		wlc_err("Invalid inhibit = %d\n", en);
		return -EINVAL;
	}
	pWlc->ctl.inhibit_hook = en;

	if (gpio_is_valid(pWlc->wls_control_en)) {
		gpio_set_value(pWlc->wls_control_en, en);
		wlc_info("%s en:%d now inhibit:%d\n",
			__func__, en, gpio_get_value(pWlc->wls_control_en));
	}

	return count;
}

static ssize_t inhibit_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;
	int rt = -EINVAL;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}
	if (gpio_is_valid(pWlc->wls_control_en)) {
		rt = gpio_get_value(pWlc->wls_control_en);
	}

	return sprintf(buf, "%d\n", rt);
}
static DEVICE_ATTR(inhibit, S_IRUGO|S_IWUSR, inhibit_show, inhibit_store);

static ssize_t show_offset_detect_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->ctl.enable_rod);
}

static ssize_t store_offset_detect_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	if (pWlc->config.enable_rx_offset_detect) {
		pWlc->ctl.enable_rod = (tmp == 0? false: true);
	}

	return count;
}
static DEVICE_ATTR(offset_detect_enable, 0664, show_offset_detect_enable, store_offset_detect_enable);

static ssize_t show_mc_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	if (!pWlc->config.mc_support) {
		wlc_err("%s: Magnetic cover not support\n", __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", pWlc->ctl.mc_status);
}

static ssize_t store_mc_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	struct moto_wlc *pWlc = dev->driver_data;

	if (IS_ERR_OR_NULL(pWlc)) {
		wlc_err("%s: chip not valid\n", __func__);
		return -ENODEV;
	}

	if (!pWlc->config.mc_support) {
		wlc_err("%s: Magnetic cover not support\n", __func__);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	pWlc->ctl.mc_status = (tmp == 0? false: true);

	//have Magnetic cover mc_status:true -> md_det:0
	//have no Magnetic cover mc_status:false -> md_det:1
	if (0 != wls_rx_set_mc_det(pWlc->wls_dev, !pWlc->ctl.mc_status)) {
		wlc_err("%s: mc_det set failed\n", __func__);
		return -ENODEV;
	}

	return count;
}
static DEVICE_ATTR(mc_status, 0664, show_mc_status, store_mc_status);

int wls_device_node_create(struct device *dev)
{
	if (IS_ERR_OR_NULL(dev)) {
		wlc_err("%s dev is ERR or NULL\n", __func__);
		return -ENODEV;
	}
	wlc_info("%s\n", __func__);

//-----------------------program---------------------
	device_create_file(dev, &dev_attr_wireless_fw_version);
	device_create_file(dev, &dev_attr_wireless_fw_update);

//-----------------------factory wireless test-------
	device_create_file(dev, &dev_attr_factory_wireless_en);
	device_create_file(dev, &dev_attr_inhibit);

//-----------------------RX--------------------------
	device_create_file(dev, &dev_attr_get_rx_irect);
	device_create_file(dev, &dev_attr_get_rx_vrect);
	device_create_file(dev, &dev_attr_get_rx_vout);
	device_create_file(dev, &dev_attr_mode_select);
	device_create_file(dev, &dev_attr_mode_switch);
	device_create_file(dev, &dev_attr_wls_input_current_limit);

	device_create_file(dev, &dev_attr_wlc_fan_speed);
	device_create_file(dev, &dev_attr_wlc_light_ctl);
	device_create_file(dev, &dev_attr_wlc_tx_power);
	device_create_file(dev, &dev_attr_wlc_tx_type);
	device_create_file(dev, &dev_attr_wlc_st_changed);
	device_create_file(dev, &dev_attr_offset_detect_enable);
	device_create_file(dev, &dev_attr_mc_status);

//-----------------------MOTO WLC2.0-------------------
	device_create_file(dev, &dev_attr_wlc_tx_capability);
	device_create_file(dev, &dev_attr_wlc_tx_id);
	device_create_file(dev, &dev_attr_wlc_tx_sn);

	return 0;
}

static int wireless_get_chip_id(void *input)
{
	struct moto_wlc *wlc = NULL;
	int rt = 0;
	int retry = 0;

	wlc_info("%s input=%p\n", __func__, input);
	wlc = (struct moto_wlc *) input;
	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return -1;
	}

	if (wlc->data.chip_id == wlc->config.chip_id) {
		return wlc->config.chip_id;
	}

	rt = wls_rx_get_chip_id(wlc->wls_dev, &wlc->data.chip_id);
	wlc_info("%s chip_id=%d rt=%d\n", __func__, wlc->data.chip_id, rt);
	if (!rt) { //IIC ok
		return wlc->data.chip_id;
	}

	if ((wlc_hal_get_vbus(wlc->alg) / 1000) < (VBUS_VALID_MV / 2)) {
		wls_device_fw_set_boost(wlc, true);
		while (retry < 3 && (wlc->data.chip_id != wlc->config.chip_id)) {
			msleep(50);
			rt = wls_rx_get_chip_id(wlc->wls_dev, &wlc->data.chip_id);
			retry ++;
		}
		wls_device_fw_set_boost(wlc, false);
	}

	wlc_info("%s chip_id=0x%X, retry=%d\n", __func__, wlc->data.chip_id, retry);

	return wlc->data.chip_id;
}

static int factory_test_wls_en(void *input, bool en)
{
	struct moto_wlc *wlc = NULL;
	int ret = 0;
	int wait = 0;
	int mux_channel = 0;

	wlc = (struct moto_wlc *) input;
	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return -1;
	}

	if (en) {
		if (wls_chg_get_mux_channel(&mux_channel)) {
			wlc_err("%s get mux_channel error\n", __func__);
			return -1;
		}
		if (mux_channel != MMI_MUX_CHANNEL_TYPEC_CHG) {
			wlc_err("%s mux_channel(%d) is not TYPEC_CHG\n", __func__, mux_channel);
			return -1;
		}
		if (wlc->ctl.factory_wls_en == false) {
			wlc->ctl.factory_wls_en = true;
			ret = wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, true);
		}
	} else {
		if (wlc->ctl.factory_wls_en == true) {
			if (gpio_is_valid(wlc->wls_control_en)) { //inhibit gpio can disable wireless charging
				ret = wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, false);
			} else {
				wait = 50;
				while (wait > 0 && wlc->auth.hs_st == AUTH_HS_UNKONWN) {
					msleep(100);
					wait --;
				}
				ret |= wls_device_set_mode_select(wlc, "factory_wls_en", false);
				wait = 50;
				while (wait > 0 && (wlc_hal_get_vbus(wlc->alg) > 5500000)) {
					msleep(10);
					wait --;
				}
				ret |= wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, false);
				ret |= wls_device_set_mode_select(wlc, "factory_wls_en", true);
				wlc->ctl.factory_wls_en = false;
			}
		}
	}

	wlc_info("wls: factory wls_en %d, ret=%d\n", en, ret);
	return ret;
}


int wls_device_tcmd_register(struct moto_wlc *wlc)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	wlc->wls_tcmd_client.data = wlc;
	wlc->wls_tcmd_client.client_id = MOTO_CHG_TCMD_CLIENT_WLS;

	wlc->wls_tcmd_client.get_chip_id = wireless_get_chip_id;
	wlc->wls_tcmd_client.wls_en = factory_test_wls_en;

	ret = moto_chg_tcmd_register(&wlc->wls_tcmd_client);

	return ret;
}
