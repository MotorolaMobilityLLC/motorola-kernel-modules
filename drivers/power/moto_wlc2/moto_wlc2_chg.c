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

int wls_chg_mmi_mux_chan_set(enum mmi_mux_channel channel, bool on)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		pr_err("%s mmi_mux Couldn't get chg_psy\n", __func__);
		return -1;
	}

	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("%s mmi_mux Couldn't get chg_psy\n", __func__);
		return -1;
	}

	if (info->algo.do_mux) {
		pr_info("mmi_mux open wls chg chan = %d on = %d\n", channel, on);
		info->algo.do_mux(info, channel, on);
	} else
		pr_err("mmi_mux get info->algo.do_mux fail\n");

	return 0;
}

int wls_chg_get_mux_channel(int *mux_channel)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		pr_err("%s Couldn't get chg_psy\n", __func__);
		return -1;
	}

	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("%s Couldn't get mtk_charger info\n", __func__);
		return -1;
	}

	*mux_channel = info->mmi.mux_channel.chan;

	return 0;
}

int wls_chg_power_on(struct moto_wlc *wlc)
{
	int sys_mode = 0;
	int rt = -1;

	wlc->ctl.rx_power_on = true;

	if (wlc->ctl.factory_wls_en)
		return 0;

	wlc->ctl.rx_start_ktime = ktime_get_boottime();
	wls_rx_get_sys_mode(wlc->wls_dev, &sys_mode);
	if (sys_mode == SYS_MODE_RX) {
		rt = wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_CHG, true);
		if (rt == 0) {
			wlc->wls_online = true;
			power_supply_changed(wlc->wls_psy);
		}
	}

	if (wlc->config.mc_support && wlc->ctl.mc_status) {
		wlc_info("%s skip offset_detect_work\n", __func__);
	} else if (wlc->ctl.enable_rod && !wlc->config.wls_cert_mode) {
		wlc->ctl.rod_stop = false;
		wlc->ctl.rx_ldo_detect_count = 0;
		pr_info("%s start offset_detect_work\n", __func__);
		queue_delayed_work(wlc->wls_wq, &wlc->offset_detect_work, msecs_to_jiffies(4000));
	}

	return rt;
}

int wls_chg_power_off(struct moto_wlc *wlc)
{
	int rt = -1;

	wlc->ctl.rx_power_on = false;
	wlc->ctl.rx_ldo_on = false;

	if (wlc->ctl.factory_wls_en) {
		if (gpio_is_valid(wlc->wls_control_en)) {
			wlc->ctl.factory_wls_en = false;
		}
		pr_info("%s Exit factory wireless charging test\n", __func__);
		return 0;
	}

	rt = wls_chg_mmi_mux_chan_set(MMI_MUX_CHANNEL_WLC_CHG, false);
	if (rt == 0) {
		wlc->wls_online = false;
		power_supply_changed(wlc->wls_psy);
	}

	return rt;
}

int wls_chg_current_select(struct moto_wlc *wlc, int *icl, int *vbus)
{
	int wls_power = 0;

	if (IS_ERR_OR_NULL(wlc))
		return -1;

	if (IS_ERR_OR_NULL(wlc->wls_dev))
		return -1;

	wls_rx_get_op_mode(wlc->wls_dev, &wlc->data.mode_type);
	wlc_info("%s start icl=%d vbus=%d mode_type:%d\n",
			__func__, *icl, *vbus, wlc->data.mode_type);

	if (wlc->config.mc_support && wlc->ctl.mc_status) {
		wls_rx_get_rx_neg_power(wlc->wls_dev, &wls_power);
		if (wlc->data.moto_stand || wlc->data.mcode == MOTO_TX_MCODE) {
			if (wls_power == WLS_RX_CAP_15W) {
				*icl = wlc->config.epp_icl_max_uA;
				*vbus = 12000;
			} else if (wls_power == WLS_RX_CAP_10W) {
				*icl = 850000;
				*vbus = 12000;
			} else if (wls_power == WLS_RX_CAP_5W) {
				*icl = 1000000;
				*vbus = 5000;
			}
		} else if (wlc->data.mode_type == Sys_Op_Mode_BPP) {
			*vbus = 5000;
			if (wlc->ctl.mc_icl_state == MC_ICL_RUN) {
				return -1;
			} else {
				*icl = wlc->ctl.mc_icl_max_uA;
			}
		} else if (wlc->data.mode_type == Sys_Op_Mode_EPP) {
			if (wls_power == WLS_RX_CAP_15W) {
				*vbus = 12000;
				if (wlc->ctl.mc_icl_state == MC_ICL_RUN) {
					return -1;
				} else {
					*icl = wlc->ctl.mc_icl_max_uA;
				}
			} else if (wls_power >= WLS_RX_CAP_10W) {
				*icl = 900000;
				*vbus = 9000;
			} else if (wls_power >= WLS_RX_CAP_7W) {
				*icl = 700000;
				*vbus = 9000;
			} else if (wls_power >= WLS_RX_CAP_5W) {
				*icl = 1000000;
				*vbus = 5000;
			} else {
				*icl = 1000000;
				*vbus = 5000;
			}
		}
		wlc->data.vbus_select = *vbus;
	} else {
		if (wlc->data.mode_type == Sys_Op_Mode_BPP) {
			*icl = wlc->config.bpp_icl_max_uA;
			*vbus = 5000;
			wlc->data.vbus_select = *vbus;
			if (!wlc->ctl.bpp_icl_done) {
				return -1;
			}
		} else if (wlc->data.mode_type == Sys_Op_Mode_EPP) {
			wls_rx_get_rx_neg_power(wlc->wls_dev, &wls_power);
			if (wls_power >= WLS_RX_CAP_15W) {
				*icl = wlc->config.epp_icl_max_uA;
				*vbus = 12000;
			} else if (wls_power >= WLS_RX_CAP_10W) {
				*icl = 1000000;
				*vbus = 9000;
			} else if (wls_power >= WLS_RX_CAP_7W) {
				*icl = 850000;
				*vbus = 9000;
			} else if (wls_power >= WLS_RX_CAP_5W) {
				*icl = 1000000;
				*vbus = 5000;
			} else {
				*icl = 1000000;
				*vbus = 5000;
			}
			wlc->data.vbus_select = *vbus;
		}
	}

	if (wlc->ctl.input_current_max != 0 &&
			wlc->ctl.input_current_max * 1000 < *icl)
		*icl = wlc->ctl.input_current_max * 1000;

	wlc_info("%s icl=%d vbus=%d epp_icl_max_uA=%d input_current_max=%d\n",
			__func__, *icl, *vbus, wlc->config.epp_icl_max_uA,wlc->ctl.input_current_max);
	return 0;
}

int wlc_chg_start_mc_icl_work(struct moto_wlc *wlc, int op_mode)
{
	int rt = 0;

	if (op_mode == Sys_Op_Mode_EPP) {
		wlc->ctl.mc_icl_max_uA = WLS_MC_EPP_ICL_DEFAULT;
		wlc_hal_set_input_current(wlc->alg, CHG1, wlc->ctl.mc_icl_max_uA);
		wlc_hal_set_charging_current(wlc->alg, CHG1, wlc->wireless_charger_max_current);
	}
	wlc->ctl.ce_det_count = 0;
	wlc->ctl.mc_icl_state = MC_ICL_RUN;
	rt = queue_delayed_work(wlc->wls_wq, &wlc->mc_icl_work, msecs_to_jiffies(0));

	return rt;
}

void wlc_chg_bpp_mode_icl_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, bpp_icl_work);
	int wls_icl = 0;
	int wls_icl_max = 0;
	int wls_current_now = 0;
	int wls_voltage_now = 0;
	int retry = 2;
	int i = 0;
	int op_mode = 0;
	int rt = 0;
	int step_delay_ms = 0;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return;
	}

	if (wlc->config.mc_support && wlc->ctl.mc_status) {
		step_delay_ms = WLS_MC_BPP_ICL_STEP_DELAY;
	} else {
		step_delay_ms = wlc->config.bpp_step_delay_ms;
	}

	wlc_hal_set_input_current(wlc->alg, CHG1, wlc->config.bpp_icl_min_uA);
	wlc_hal_set_charging_current(wlc->alg, CHG1, wlc->wireless_charger_max_current);

	wls_icl_max = wlc->config.bpp_icl_max_uA / 1000;

	if (wlc->ctl.input_current_max > 0 &&
		wls_icl_max > wlc->ctl.input_current_max) {
		wls_icl_max = wlc->ctl.input_current_max;
	}
	wlc_info("%s wls_icl_max=%dmA step=%dmA delay_ms=%dms\n",
		__func__, wls_icl_max, wlc->config.bpp_icl_step_uA / 1000, step_delay_ms);

	for (i = 0; i < retry; i++) {
		if (wls_icl < wlc->config.bpp_icl_min_uA / 1000) {
			wls_icl = wlc->config.bpp_icl_min_uA / 1000;
		}

		while (wls_icl <= wls_icl_max) {
			rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
			wlc_info("%s wls_icl=%dmA op_mode=%d rt=%d\n", __func__, wls_icl, op_mode, rt);
			if (rt < 0 || op_mode != Sys_Op_Mode_BPP)
				break;
			wlc_hal_set_input_current(wlc->alg, CHG1, wls_icl * 1000);
			msleep(step_delay_ms + i * 100);

			wls_rx_get_rx_iout(wlc->wls_dev, &wls_current_now);
			wls_rx_get_rx_vout(wlc->wls_dev, &wls_voltage_now);
			wlc_info("%s set icl=%dmA I=%dmA V=%dmV\n",
					__func__, wls_icl, wls_current_now, wls_voltage_now);
			wls_icl = wls_icl + (wlc->config.bpp_icl_step_uA / 1000);
		}

		if (wls_current_now < WLS_BPP_ROD_THRESHOLD_CURRENT_MAX) {
			wls_icl = wls_current_now / WLS_ICL_INCREASE_STEP_MA * WLS_ICL_INCREASE_STEP_MA;
		} else {
			break;
		}
	}

	wlc->ctl.mc_icl_max_uA = wlc->config.bpp_icl_max_uA;

	if (wlc->config.mc_support && wlc->ctl.mc_status) {
		rt = wls_rx_get_rx_iout(wlc->wls_dev, &wls_current_now);
		wlc_info("%s wls_current_now:%d rt:%d\n", __func__, wls_current_now, rt);
		if (rt == 0 && wls_current_now <= WLS_MC_BPP_ICL_THRESHOLD) {
			wlc_info("%s moto_stand:%d\n", __func__, wlc->data.moto_stand);
			if (!wlc->data.moto_stand) //BPP can't get mcode
				wlc_chg_start_mc_icl_work(wlc, Sys_Op_Mode_BPP);
		}
	}

	wlc->ctl.bpp_icl_done = true;
}

void wlc_chg_mc_icl_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, mc_icl_work);
	int ce = 0;
	int rt = 0;
	int rx_irect = 0;
	int op_mode = 0;
	static int det_count = 0;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return;
	}

	if (wlc->ctl.ce_det_count == 0) {
		det_count = 0;
	}

	if (det_count >= WLS_MC_DET_CNT_MAX) {
		wlc_info("%s timeout, exit\n", __func__);
		goto mc_icl_exit;
	}

	rt = wls_rx_get_ce(wlc->wls_dev, &ce);
	if (rt < 0) {
		wlc_err("%s get ce err, exit\n", __func__);
		goto mc_icl_exit;
	}

	det_count ++;
	if (ce <= 2) {
		wlc->ctl.ce_det_count ++ ;
	}

	rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
	wlc_info("[%s] det_count:%d op_mode:%d ce:%d ce_det_count:%d rt:%d\n",
			__func__, det_count, op_mode, ce, wlc->ctl.ce_det_count, rt);
	if (rt < 0) {
		wlc_err("%s get op_mode err, exit\n", __func__);
		goto mc_icl_exit;
	} else if (op_mode == Sys_Op_Mode_BPP) {
		if (wlc->ctl.ce_det_count >= 50) {
			rt = wls_rx_get_rx_irect(wlc->wls_dev, &rx_irect);
			wlc_info("%s rx_irect:%dmA rt:%d\n", __func__, rx_irect, rt);
			if (rt < 0) {
				wlc_err("%s get rx_irect err, exit\n", __func__);
				goto mc_icl_exit;
			}
			rx_irect = rx_irect / 100 * 100;
			if (rx_irect >= WLS_BPP_ICL_MIN_MA + 200) {
				wlc->ctl.mc_icl_max_uA = (rx_irect - 200) * 1000;
			} else {
				wlc->ctl.mc_icl_max_uA = WLS_BPP_ICL_MIN_MA * 1000; //300 mA
			}
			wlc_info("%s rx_irect:%d mc_icl_max_uA:%d\n",
						__func__, rx_irect, wlc->ctl.mc_icl_max_uA);
			while (wlc->ctl.mc_icl_max_uA < wlc->config.bpp_icl_max_uA) {
				rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
				if (rt < 0) {
					wlc_err("%s get op_mode failed, rt:%d\n", __func__, rt);
					goto mc_icl_exit;
				}
				wlc->ctl.mc_icl_max_uA = wlc->ctl.mc_icl_max_uA + WLS_MC_ICL_STEP;
				wlc_hal_set_input_current(wlc->alg, CHG1, wlc->ctl.mc_icl_max_uA);
				msleep(WLS_MC_BPP_ICL_STEP_DELAY);
				wlc_info("%s BPP set mc_icl_max_uA:%d\n", __func__, wlc->ctl.mc_icl_max_uA);
			}
			goto mc_icl_exit;
		}
	} else if (op_mode == Sys_Op_Mode_EPP) {
		if (det_count == (WLS_MC_DET_CNT_MAX / 2) || det_count == WLS_MC_DET_CNT_MAX) {
			if (wlc->ctl.ce_det_count >= 25) {
				wlc->ctl.mc_icl_max_uA = wlc->ctl.mc_icl_max_uA + WLS_MC_ICL_STEP;
				wlc_hal_set_input_current(wlc->alg, CHG1, wlc->ctl.mc_icl_max_uA);
				wlc_info("%s EPP set mc_icl_max_uA:%d\n",
							__func__, wlc->ctl.mc_icl_max_uA);
			}
			wlc->ctl.ce_det_count = 1;
			if (det_count == WLS_MC_DET_CNT_MAX)
				goto mc_icl_exit;
		}
	}

	if (wlc->ctl.mc_icl_state == MC_ICL_RUN) {
		queue_delayed_work(wlc->wls_wq, &wlc->mc_icl_work, msecs_to_jiffies(1000));
	}

	return;

mc_icl_exit:
	wlc->ctl.mc_icl_state = MC_ICL_DONE;
	wlc->ctl.ce_det_count = 0;
	det_count = 0;
}


int wls_chg_event_handler(struct wireless_device* wls_dev, struct wls_event_msg *msg)
{
	struct moto_wlc *wlc = NULL;
	int op_mode = 0;
	int rt = 0;
	bool otg_en = false;

	wlc = (struct moto_wlc *)wls_dev->driver_data;

	if (IS_ERR_OR_NULL(wlc))
		return -1;

	if (msg == NULL)
		return -1;

	pr_info("%s event:%d len=%d\n", __func__ , msg->event, msg->len);
	switch(msg->event) {
		case WLS_EVENT_TX_DETECTED:
			if (msg->len == 1) {
				if (msg->data[0] == 0 && wlc->ctl.rx_power_on) {
					wlc->ctl.bpp_icl_done = false;
					wlc->ctl.mc_icl_state = MC_ICL_IDLE;
					wlc->ctl.ce_det_count = 0;
					wlc->data.mcode = 0x00;
					wls_chg_power_off(wlc);
					wls_auth_disconnect(wlc);
					wls_chg_notify_st_changed(wlc, WLC_DISCONNECTED);
					if (wlc->ctl.mode_switch != WLC_SWITCH_RUN &&
						!wlc->ctl.mode_select_force &&
						!wlc->ctl.factory_wls_en)
						wls_device_set_mode_select(wlc, "wls_det_irq_handler", 1);
				}
			}
			break;
		case WLS_EVENT_RX_POWER_ON:
			if (!wlc->config.secure_hardware)
				queue_delayed_work(wlc->wls_wq, &wlc->dump_info_work, msecs_to_jiffies(10));
			if (!wlc->ctl.rx_power_on)
				wls_chg_power_on(wlc);
			break;
		case WLS_EVENT_RX_NEGO_POWER_READY:
			rt = wls_rx_get_rx_neg_power(wlc->wls_dev, &wlc->data.wlc_power);
			rt |= wls_rx_get_mcode(wlc->wls_dev, &wlc->data.mcode);
			wlc_info("%s power:%d mcode:0x%04X rt:%d\n",
						__func__, wlc->data.wlc_power, wlc->data.mcode, rt);
			if (rt == 0 && wlc->config.mc_support &&
				wlc->ctl.mc_status && wlc->data.mcode != MOTO_TX_MCODE &&
				wlc->data.wlc_power == WLS_RX_CAP_15W) {
				wlc_chg_start_mc_icl_work(wlc, Sys_Op_Mode_EPP);
			}
			if (!IS_ERR_OR_NULL(wlc))
				wls_chg_notify_st_changed(wlc, WLC_TX_POWER_CHANGED);
			break;
		case WLS_EVENT_RX_LDO_ON:
			wlc->ctl.rx_ldo_on = true;
			wlc->ctl.bpp_icl_done = false;
			wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
			wls_rx_get_rx_neg_power(wlc->wls_dev, &wlc->data.wlc_power);
			pr_info("%s op_mode=%d, wlc->data.wlc_power=%d\n", __func__, op_mode, wlc->data.wlc_power);
			if (op_mode == Sys_Op_Mode_BPP) {
				wlc->data.mode_type = op_mode;
				queue_delayed_work(wlc->wls_wq, &wlc->bpp_icl_work, msecs_to_jiffies(0));
			}
			wls_chg_notify_st_changed(wlc, WLC_CONNECTED);
			break;
		case WLS_EVENT_HS_OK:
			if (!wlc->config.wls_cert_mode) {
				wlc->data.moto_stand = true;
				wlc->auth.hs_st = AUTH_HS_OK;
				wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
				wls_auth_hs_ok_handler(wlc, op_mode);
			} else {
				wlc->data.moto_stand = false;
				wlc->auth.hs_st = AUTH_HS_FAIL;
			}
			break;
		case WLS_EVENT_HS_FAIL:
			wlc->data.moto_stand = false;
			wlc->auth.hs_st = AUTH_HS_FAIL;
			queue_delayed_work(wlc->wls_wq, &wlc->light_fan_work, msecs_to_jiffies(0));
			break;
		case WLS_EVENT_RX_FSK_PKT:
			wls_auth_decode_fsk_packet(wlc, msg->data, msg->len);
			break;
		case WLS_EVENT_RX_BACKPOWER_MODE:
			if (wlc->ctl.tx_mode) {
				wlc_info("%s Skip when tx_mode\n", __func__);
			} else if(wlc->ctl.fw_uploading) {
				wlc_info("%s Skip when fw_uploading\n", __func__);
			} else if (wlc->ctl.otg_boost_on) {
				wlc_info("%s Skip when otg or boost on\n", __func__);
			} else {
				if (!IS_ERR_OR_NULL(wlc->chg1_dev)) {
					rt = charger_dev_is_otg_enabled(wlc->chg1_dev, &otg_en);
					if (rt) {
						wlc_err("%s get otg status failed, rt:%d\n", __func__, rt);
					}
				}
				wlc_info("%s otg_en:%d ret:%d\n", __func__, otg_en, rt);
				if (otg_en) {
					rt = wls_rx_set_lowpower_mode(wlc->wls_dev, true);
					wlc_info("%s set_lowpower_mode ret:%d\n", __func__, rt);
				}
			}
			break;
		default:
			break;
	}
	power_supply_changed(wlc->wls_psy);

	return 0;
}


struct wireless_device * wls_get_wireless_device(struct moto_wlc *wlc)
{
	//pr_info("%s\n", __func__);

	if (IS_ERR_OR_NULL(wlc))
		return NULL;

	if (!IS_ERR_OR_NULL(wlc->wls_dev))
		return wlc->wls_dev;
	else {
		wlc->wls_dev = get_wireless_by_name("moto_wlc2");
		if (!IS_ERR_OR_NULL(wlc->wls_dev)) {
			wlc->callback_ops.event_handler = wls_chg_event_handler;
			wireless_dev_set_drvdata(wlc->wls_dev, (void *)wlc);
			wireless_dev_set_callback(wlc->wls_dev, (void *)&wlc->callback_ops);
			if (wlc->config.mc_support) {
				wls_rx_set_mc_det(wlc->wls_dev, !wlc->ctl.mc_status);
			}
			wls_device_set_mode_select(wlc, "wlc init", 1);
			wls_rx_set_irq_enable(wlc->wls_dev, true);
			pr_info("%s wlc->wls_dev=%p\n", __func__, wlc->wls_dev);
			return wlc->wls_dev;
		} else {
			pr_info("%s get wls dev failed\n", __func__);
		}
	}

	return NULL;
}

enum power_supply_property wls_chg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

int wls_chg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct moto_wlc *wlc = power_supply_get_drvdata(psy);
	int ret = 0;
	int sys_mode = 0;

	if (IS_ERR_OR_NULL(wlc)) {
		return -EINVAL;
	}
	wls_get_wireless_device(wlc);
	if (IS_ERR_OR_NULL(wlc->wls_dev)) {
		wlc_err("%s wlc->wls_dev is err or null\n", __func__);
		return -EINVAL;
	}

	if (wlc->ctl.mode_switch == WLC_SWITCH_RUN &&
		(psp == POWER_SUPPLY_PROP_PRESENT || psp == POWER_SUPPLY_PROP_ONLINE)) {
		pr_info("%s mode switch hook online:1\n", __func__);
		val->intval = 1;
		return 0;
	}
	val->intval = 0;
	if (wlc->ctl.fw_uploading) {
		wlc_dbg("%s Unable to read registers when fw_uploading\n", __func__);
		return 0;
	} else if (!(wlc->ctl.rx_power_on || wlc->ctl.rx_ldo_on || wlc->ctl.tx_mode)) {
		wlc_dbg("%s Unable to read registers when chip no power\n", __func__);
		return 0;
	}

	switch(psp){
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_ONLINE:
			if (gpio_is_valid(wlc->wls_control_en) &&
				gpio_get_value(wlc->wls_control_en)) {
				val->intval = 0; //if inhibit is high, set online false
				pr_info("%s inhibit:1, online:0\n", __func__);
			} else {
				ret = wls_rx_get_sys_mode(wlc->wls_dev, &sys_mode);
				pr_info("%s online:%d ret:%d\n", __func__, sys_mode == SYS_MODE_RX, ret);
				if (sys_mode == SYS_MODE_RX)
					val->intval = 1;
				else {
					val->intval = 0;
				}
			}
			break;

		case POWER_SUPPLY_PROP_TYPE:
			val->intval = wlc->wls_psd.type;
			break;

		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			if (wlc->data.mode_type == Sys_Op_Mode_BPP)
				val->intval = POWER_SUPPLY_USB_TYPE_WLC_BPP;
			else if (wlc->data.mode_type == Sys_Op_Mode_EPP)
				val->intval = POWER_SUPPLY_USB_TYPE_WLC_EPP;
			else
				val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 12000000;//uV
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = -1;
			if (wlc->ctl.rx_ldo_on) {
				ret = wls_rx_get_rx_vout(wlc->wls_dev, &val->intval);
				if (ret == 0 && val->intval > 0) {
					val->intval = val->intval * 1000;
				}
				pr_info("%s rx_vout:%d ret:%d\n", __func__, val->intval, ret);
			}
			break;

		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 1250000;//uA
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = -1;
			if (wlc->ctl.rx_ldo_on) {
				ret = wls_rx_get_rx_iout(wlc->wls_dev, &val->intval);
				if (ret == 0 && val->intval > 0) {
					val->intval = val->intval * 1000;
				}
				pr_info("%s rx_iout:%d ret:%d\n", __func__, val->intval, ret);
			}
			break;

		case POWER_SUPPLY_PROP_POWER_NOW:
			wls_rx_get_rx_neg_power(wlc->wls_dev, &val->intval);
			if (val->intval > 0) {
				pr_info("%s power_now:%d\n", __func__, val->intval);
			} else
				val->intval = -1;
			break;

		default:
			ret = -EINVAL;
			break;
	}
	wlc_info("[%s] psp = %d val = %d.\n", __func__, psp,val->intval);
	return ret;
}

void wls_charger_external_power_changed(struct power_supply *psy)
{
}

static char *wls_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

int wls_chg_notify_st_changed(struct moto_wlc *wlc, int st)
{
	if (wlc->data.wlc_status != st) {
		wlc_info("%s st change	%d -> %d\n",
				__func__, wlc->data.wlc_status, st);
		wlc->data.wlc_status = st;
		sysfs_notify(&wlc->wls_psy->dev.parent->kobj, NULL, "wlc_st_changed");
	}

	return 0;
}

int wls_chg_register_psy(struct moto_wlc *wlc)
{
	struct power_supply_config wls_psy_cfg = {};

	wlc->wls_psd.name = "wireless";
	wlc->wls_psd.type = POWER_SUPPLY_TYPE_WIRELESS;
	wlc->wls_psd.properties = wls_chg_props;
	wlc->wls_psd.num_properties = ARRAY_SIZE(wls_chg_props);
	wlc->wls_psd.get_property = wls_chg_get_property;
	wlc->wls_psd.external_power_changed = wls_charger_external_power_changed;

	wls_psy_cfg.drv_data = wlc;
	wls_psy_cfg.of_node = wlc->pdev->dev.of_node;
	wls_psy_cfg.supplied_to = wls_psy_supplied_to;
	wls_psy_cfg.num_supplicants = ARRAY_SIZE(wls_psy_supplied_to);
	wlc->wls_psy = power_supply_register(&wlc->pdev->dev,
								&wlc->wls_psd,
								&wls_psy_cfg);
	if (IS_ERR(wlc->wls_psy)) {
		return PTR_ERR(wlc->wls_psy);
	}

	return 0;
}

int wls_chg_notify_otg_plugin(struct moto_wlc *wlc, bool on)
{
	int wait = 0;
	int rt = 0;
	int op_mode = Sys_Op_Mode_INVALID;
	int sys_mode = 0;

	pr_info("%s on:%d\n", __func__, on);
	if (on) {
		rt = wls_rx_get_sys_mode(wlc->wls_dev, &sys_mode);
		pr_info("%s sys_mode:%d rt:%d\n", __func__, sys_mode, rt);
		if (rt != 0 || sys_mode != SYS_MODE_RX) {
			return 0;
		}

		rt = wls_rx_get_op_mode(wlc->wls_dev, &op_mode);
		pr_info("%s op_mode:%d rt:%d\n", __func__, op_mode, rt);
		if (rt == 0 &&
			(op_mode == Sys_Op_Mode_EPP || wlc_hal_get_vbus(wlc->alg) > 5500000)) {
			if (gpio_is_valid(wlc->wls_control_en)) {
				gpio_set_value(wlc->wls_control_en, 1); //1 disable wls, 0 enable wls
				pr_info("%s disable wls, set inhibit:%d\n",
					__func__, gpio_get_value(wlc->wls_control_en));
			} else {
				wait = 50;
				while (wait > 0 && wlc->auth.hs_st == AUTH_HS_UNKONWN) {
					msleep(100);
					wait --;
				}
				wlc->ctl.mode_select_force = true;
				rt = wls_device_set_mode_select(wlc, "otg_plugin", 0);
			}
			wait = 50;
			while (wait > 0 && (wlc_hal_get_vbus(wlc->alg) > 5500000)) {
				msleep(10);
				wait --;
			}
			if (wlc->ctl.mode_select_force)
				wlc->ctl.mode_select_force = false;
			pr_info("%s vbus:%d\n", __func__, wlc_hal_get_vbus(wlc->alg));
		}
	}

	return rt;
}

void wlc_chg_dump_info_work(struct work_struct *work)
{
	struct moto_wlc *wlc =
		container_of((struct delayed_work*)work, struct moto_wlc, dump_info_work);
	int delay_ms = 0;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return;
	}

	if (wls_rx_check_dump_info(wlc->wls_dev)) {
		delay_ms = wlc->ctl.rx_ldo_on ? 3000 : 1000;
		queue_delayed_work(wlc->wls_wq, &wlc->dump_info_work, msecs_to_jiffies(delay_ms));
	}
}
