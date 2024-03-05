// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Motorola Inc.
 */

#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <mtk_charger_algorithm_class.h>
#include "mmi_pehv.h"

static int log_level = PEHV_INFO_LEVEL;
module_param(log_level, int, 0644);

static bool algo_waiver_test;
module_param(algo_waiver_test, bool, 0644);

int pehv_get_log_level(void)
{
	return log_level;
}

#define MS_TO_NS(msec)		((msec) * (NSEC_PER_MSEC))

/* Parameters */
#define PEHV_VTA_INIT		5500	/* mV */
#define PEHV_ITA_INIT		3000	/* mA */
#define PEHV_TA_WDT_MIN		10000	/* ms */
#define PEHV_VTA_GAP_MIN	200	/* mV */
#define PEHV_VTA_VAR_MIN	103	/* % */
#define PEHV_ITA_TRACKING_GAP	150	/* mA */
#define PEHV_DVCHG_VBUSALM_GAP	100	/* mV */
#define PEHV_DVCHG_STARTUP_CONVERT_RATIO	210	/* % */
#define PEHV_DVCHG_CHARGING_CONVERT_RATIO	202	/* % */
#define PEHV_VBUSOVP_RATIO	110
#define PEHV_IBUSOCP_RATIO	110
#define PEHV_VBATOVP_RATIO	110
#define PEHV_IBATOCP_RATIO	110
#define PEHV_ITAOCP_RATIO	110
#define PEHV_IBUSUCPF_RECHECK		250	/* mA */
#define PEHV_VBUS_CALI_THRESHOLD	150	/* mV */
#define PEHV_CV_LOWER_BOUND_GAP		50	/* mV */
#define PEHV_INIT_POLLING_INTERVAL	0	/* ms */
#define PEHV_CV_POLLING_INTERVAL	500	/* ms */
#define PEHV_INIT_RETRY_MAX		3
#define PEHV_MEASURE_R_RETRY_MAX	3
#define PEHV_MEASURE_R_AVG_TIMES	10
#define PEHV_VSYS_UPPER_BOUND            4700    /* mV */
#define PEHV_VSYS_UPPER_BOUND_GAP        40      /* mV */
#define PEHV_START_SOC_MAX_GAP		4	/* % */
#define PEHV_WHILE_LOOP_ITERATION_MAX	50
#define MMI_IBAT_GAP_MA 50 	/* mA */


#define PEHV_HWERR_NOTIFY \
	(BIT(EVT_VBUSOVP) | BIT(EVT_IBUSOCP) | BIT(EVT_VBATOVP) | \
	 BIT(EVT_IBATOCP) | BIT(EVT_VOUTOVP) | BIT(EVT_VDROVP) | \
	 BIT(EVT_IBUSUCP_FALL))

#define PEHV_RESET_NOTIFY \
	(BIT(EVT_DETACH) | BIT(EVT_HARDRESET))

static const char *const pehv_dvchg_role_name[PEHV_DVCHG_MAX] = {
	"master", "slave",
};

static const char *const pehv_algo_state_name[PEHV_ALGO_STATE_MAX] = {
	"INIT", "MEASURE_R", "SS_SWCHG", "SS_DVCHG", "CC_CV", "STOP",
};

/* If there's no property in dts, these values will be applied */
static const struct pehv_algo_desc algo_desc_defval = {
	.polling_interval = 500,
	.vbat_cv = 4350,
	.start_soc_min = 5,
	.start_soc_max = 80,
	.stop_soc_max = 99,
	.vbat_max_gap = 30,
	.idvchg_term = 500,
	.idvchg_step = 50,
	.idvchg_ss_init = 1000,
	.idvchg_ss_step = 250,
	.idvchg_ss_step1 = 100,
	.idvchg_ss_step2 = 50,
	.idvchg_ss_step1_vbat = 4000,
	.idvchg_ss_step2_vbat = 4200,
	.ta_blanking = 285,
	.swchg_aicr = 0,
	.swchg_ichg = 0,
	.swchg_aicr_ss_init = 400,
	.swchg_aicr_ss_step = 200,
	.swchg_off_vbat = 4250,
	.force_ta_cv_vbat = 4250,
	.chg_time_max = 5400,
	.ifod_threshold = 200,
	.rsw_min = 20,
	.ircmp_rbat = 40,
	.ircmp_vclamp = 0,
	.vta_cap_min = 6800,
	.vta_cap_max = 11000,
	.ita_cap_min = 1000,
};

/*
 * @reset_ta: set output voltage/current of TA to 5V/3A and disable
 *            direct charge
 * @hardreset: send hardreset to port partner
 * Note: hardreset's priority is higher than reset_ta
 */
struct pehv_stop_info {
	bool hardreset_ta;
	bool reset_ta;
};

static inline enum chg_idx to_chgidx(enum pehv_dvchg_role role)
{
	if (role == PEHV_DVCHG_MASTER)
		return DVCHG1;
	return DVCHG2;
}

/* Check if there is error notification coming from H/W */
static bool pehv_is_hwerr_notified(struct pehv_algo_info *info)
{
	struct pehv_algo_data *data = info->data;
	bool err = false;
	u32 hwerr = PEHV_HWERR_NOTIFY;

	mutex_lock(&data->notify_lock);
	if (data->ignore_ibusucpf)
		hwerr &= ~BIT(EVT_IBUSUCP_FALL);
	err = !!(data->notify & hwerr);
	if (err)
		PEHV_ERR("H/W error(0x%08X)", hwerr);
	mutex_unlock(&data->notify_lock);
	return err;
}

/*
 * Get ADC value from divider charger
 * Note: ibus will sum up value from all enabled chargers
 * (master dvchg, slave dvchg and swchg)
 */
static int pehv_stop(struct pehv_algo_info *info, struct pehv_stop_info *sinfo);
static int pehv_get_adc(struct pehv_algo_info *info, enum pehv_adc_channel chan,
			int *val)
{
	struct pehv_algo_data *data = info->data;
	int ret, i, ibus;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (atomic_read(&data->stop_algo)) {
		PEHV_INFO("stop algo\n");
		goto stop;
	}
	*val = 0;
	if (chan == PEHV_ADCCHAN_IBUS) {
		for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
			if (!data->is_dvchg_en[i])
				continue;
			ret = pehv_hal_get_adc(info->alg, to_chgidx(i),
					       PEHV_ADCCHAN_IBUS, &ibus);
			if (ret < 0) {
				PEHV_ERR("get dvchg ibus fail(%d)\n", ret);
				return ret;
			}
			*val += ibus;
		}
		if (data->is_swchg_en) {
			ret = pehv_hal_get_adc(info->alg, CHG1,
					       PEHV_ADCCHAN_IBUS, &ibus);
			if (ret < 0) {
				PEHV_ERR("get swchg ibus fail(%d)\n", ret);
				return ret;
			}
			*val += ibus;
		}
		return 0;
	} else if (chan == PEHV_ADCCHAN_VSYS)
		return pehv_hal_get_adc(info->alg, CHG1,
					PEHV_ADCCHAN_VSYS, val);
	return pehv_hal_get_adc(info->alg, DVCHG1, chan, val);
stop:
	pehv_stop(info, &sinfo);
	return -EIO;
}

/*
 * Calculate VBUS for divider charger
 * If divider charger is charging, the VBUS only needs to be 2 times of VOUT.
 */
static inline u32 pehv_vout2vbus(struct pehv_algo_info *info, u32 vout)
{
	struct pehv_algo_data *data = info->data;
	u32 ratio = data->is_dvchg_en[PEHV_DVCHG_MASTER] ?
		PEHV_DVCHG_CHARGING_CONVERT_RATIO :
		PEHV_DVCHG_STARTUP_CONVERT_RATIO;

	return percent(vout, ratio);
}

/*
 * Get output current and voltage measured by TA
 * and updates measured data
 * If ta does not support measure capability, dvchg's ADC is used instead
 */
static inline int pehv_get_ta_cap_by_supportive(struct pehv_algo_info *info,
						 int *vta, int *ita)
{
	int ret;

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBUS, vta);
	if (ret < 0) {
		PEHV_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	return pehv_get_adc(info, PEHV_ADCCHAN_IBUS, ita);
}

/* Calculate ibat from ita */
static inline u32 pehv_cal_ibat(struct pehv_algo_info *info, u32 ita)
{
	struct pehv_algo_data *data = info->data;

	return 2 * (data->is_swchg_en ?
		    (ita - percent(data->aicr_setting, 10)) : ita);
}

static inline u32 pehv_get_ita_tracking_max(u32 ita)
{
	return min(percent(ita, PEHV_ITAOCP_RATIO),
		   (u32)(ita + PEHV_ITA_TRACKING_GAP));
}

static inline void pehv_update_ita_gap(struct pehv_algo_info *info, u32 ita_gap)
{
	unsigned int i;
	u32 val = 0, avg_cnt = PEHV_ITA_GAP_WINDOW_SIZE;
	struct pehv_algo_data *data = info->data;

	if (ita_gap < data->ita_gap_per_vstep)
		return;
	data->ita_gap_window_idx = (data->ita_gap_window_idx + 1) %
				   PEHV_ITA_GAP_WINDOW_SIZE;
	data->ita_gaps[data->ita_gap_window_idx] = ita_gap;

	for (i = 0; i < PEHV_ITA_GAP_WINDOW_SIZE; i++) {
		if (data->ita_gaps[i] == 0)
			avg_cnt--;
		else
			val += data->ita_gaps[i];
	}
	data->ita_gap_per_vstep = avg_cnt != 0 ? precise_div(val, avg_cnt) : 0;
}

static inline int pehv_set_ta_cap_cv(struct pehv_algo_info *info, u32 vta,
				     u32 ita)
{
	int ret, ita_meas_pre, ita_meas_post, vta_meas_pre, vta_meas_post;
	int cnt = PEHV_WHILE_LOOP_ITERATION_MAX;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 vstep_cnt, ita_gap, vta_gap, ita_gap_per_vstep
						= data->ita_gap_per_vstep > 0 ?
						  data->ita_gap_per_vstep : 200;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->vta_setting == vta && data->ita_setting == ita)
		return 0;
	while (cnt-- > 0) {
		if (pehv_is_hwerr_notified(info)) {
			PEHV_ERR("H/W error notified\n");
			goto stop;
		}
		if (atomic_read(&data->stop_algo)) {
			PEHV_INFO("stop algo\n");
			goto stop;
		}
		if (vta > desc->vta_cap_max) {
			PEHV_ERR("vta(%d) over capability(%d)\n", vta,
				 desc->vta_cap_max);
			goto stop;
		}
		if (ita < desc->ita_cap_min) {
			PEHV_INFO("ita(%d) under capability(%d)\n", ita,
				  desc->ita_cap_min);
			ita = desc->ita_cap_min;
		}
		vta_gap = abs(data->vta_setting - vta);

		/* Get ta cap before setting */
		ret = pehv_get_ta_cap_by_supportive(info, &data->vta_measure,
						    &data->ita_measure);
		if (ret < 0) {
			PEHV_ERR("get ta cap by supportive fail(%d)\n", ret);
			return ret;
		}
		vta_meas_pre = data->vta_measure;
		ita_meas_pre = data->ita_measure;

		/* Not to increase vta if it exceeds pwr_lmt */
		data->ita_pwr_lmt = data->ita_lmt;
		if (data->is_dvchg_en[PEHV_DVCHG_MASTER] &&
		    vta > data->vta_setting) {
			vstep_cnt = precise_div(vta_gap, desc->vta_step);
			if (ita_meas_pre + ita_gap_per_vstep * vstep_cnt >
			    data->ita_pwr_lmt) {
				PEHV_INFO("%d + %d * %d > %d\n",
					  ita_meas_pre, ita_gap_per_vstep,
					  vstep_cnt, data->ita_pwr_lmt);
				return 0;
			}
		}
		if (ita > data->ita_pwr_lmt) {
			PEHV_INFO("ita(%d) > pwr_lmt(%d)\n",
				  ita, data->ita_pwr_lmt);
			ita = data->ita_pwr_lmt;
		}

		/* Set ta vbus */
		ret = pehv_hal_set_ta_vbus(info->alg, vta);
		if (ret < 0) {
			PEHV_ERR("set ta cap fail(%d)\n", ret);
			return ret;
		}
		data->vta_setting = vta;
		data->ita_setting = ita;
		msleep(desc->ta_blanking);

		/* Get ta cap after setting */
		ret = pehv_get_ta_cap_by_supportive(info, &data->vta_measure,
						    &data->ita_measure);
		if (ret < 0) {
			PEHV_ERR("get ta cap by supportive fail(%d)\n", ret);
			return ret;
		}
		vta_meas_post = data->vta_measure;
		ita_meas_post = data->ita_measure;

		if (data->is_dvchg_en[PEHV_DVCHG_MASTER] &&
		    (ita_meas_post > ita_meas_pre) &&
		    (vta_meas_post > vta_meas_pre)) {
			vstep_cnt = precise_div(vta_meas_post - vta_meas_pre,
						desc->vta_step);
			ita_gap = precise_div(ita_meas_post - ita_meas_pre,
					      vstep_cnt);
			pehv_update_ita_gap(info, ita_gap);
			PEHV_INFO("ita gap(now,updated)=(%d,%d)\n",
				  ita_gap, data->ita_gap_per_vstep);
		}
		if (ita_meas_post <= pehv_get_ita_tracking_max(ita))
			break;
		ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
				    data->ita_gap_per_vstep : 200;
		vstep_cnt = precise_div(ita_meas_post -
					pehv_get_ita_tracking_max(ita),
					2 * ita_gap_per_vstep);
		vta -= desc->vta_step * (vstep_cnt + 1);
		PEHV_INFO("ita_meas %dmA over setting %dmA, keep tracking...\n",
			  ita_meas_post, ita);
	}

	PEHV_INFO("vta(set,meas):(%d,%d),ita(set,meas):(%d,%d)\n",
		 data->vta_setting, data->vta_measure, data->ita_setting,
		 data->ita_measure);
	return cnt >= 0 ? 0 : -EAGAIN;
stop:
	pehv_stop(info, &sinfo);
	return -EIO;
}

static inline void pehv_calculate_vbat_ircmp(struct pehv_algo_info *info)
{
	int ret, ibat;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 ircmp;

	if (!data->is_dvchg_en[PEHV_DVCHG_MASTER]) {
		data->vbat_ircmp = 0;
		return;
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		return;
	}
	ircmp = ibat <= 0 ? 0 : div1000(ibat * data->r_bat);
	/*
	 * For safety,
	 * if state is CC_CV, ircmp can only be smaller than previous one
	 */
	if (data->state == PEHV_ALGO_CC_CV)
		ircmp = min(data->vbat_ircmp, ircmp);
	data->vbat_ircmp = min(desc->ircmp_vclamp, ircmp);
	PEHV_INFO("vbat_ircmp(vclamp,ibat,rbat)=%d(%d,%d,%d)\n",
		 data->vbat_ircmp, desc->ircmp_vclamp, ibat, data->r_bat);
}

static inline void pehv_select_vbat_cv(struct pehv_algo_info *info)
{
	int cv_limit = 0;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 cv = 0;

	mutex_lock(&data->ext_lock);
	cv_limit = data->cv_limit;
	mutex_unlock(&data->ext_lock);
	data->vbat_cv_no_ircmp = desc->vbat_cv;
	if (cv_limit > 0)
		data->vbat_cv_no_ircmp = min(data->vbat_cv_no_ircmp,
					     (u32)cv_limit);

	cv = data->vbat_cv_no_ircmp + data->vbat_ircmp;
	if (cv == data->vbat_cv)
		goto out;

	data->vbat_cv = cv;
	data->cv_lower_bound = data->vbat_cv - PEHV_CV_LOWER_BOUND_GAP;
out:
	PEHV_INFO("vbat_cv(org,limit,no_ircmp,low_bound)=%d(%d,%d,%d,%d)\n",
		  data->vbat_cv, desc->vbat_cv, cv_limit,
		  data->vbat_cv_no_ircmp, data->cv_lower_bound);
}

/*
 * Select current limit according to severial status
 * If switching charger is charging, add AICR setting to ita
 * For now, the following features are taken into consider
 * 1. Resistence
 * 2. Phone's thermal throttling
 */
static inline int pehv_get_ita_lmt(struct pehv_algo_info *info)
{
	struct pehv_algo_data *data = info->data;
	const int ita_lmt = data->ita_lmt;
	int ita = data->ita_lmt;

	mutex_lock(&data->ext_lock);
	if (data->input_current_limit >= 0)
		ita = min(ita, data->input_current_limit);

	PEHV_INFO("ita(org,throt)=%d(%d,%d)\n",
		 ita, ita_lmt, data->input_current_limit);
	mutex_unlock(&data->ext_lock);
	return ita < 0 ? 0 : ita;
}

static inline int pehv_get_idvchg_lmt(struct pehv_algo_info *info)
{
	u32 ita_lmt, idvchg_lmt;
	struct pehv_algo_data *data = info->data;

	ita_lmt = pehv_get_ita_lmt(info);
	idvchg_lmt = min(data->idvchg_cc, ita_lmt);
	PEHV_INFO("idvchg_lmt(ita_lmt,idvchg_cc)=%d(%d,%d)\n", idvchg_lmt,
		 ita_lmt, data->idvchg_cc);
	return idvchg_lmt;
}

/* Calculate VBUSOV S/W level */
static u32 pehv_get_dvchg_vbusovp(struct pehv_algo_info *info, u32 ita)
{
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 vout = data->vbat_cv +
		   div1000(pehv_cal_ibat(info, ita) * data->r_sw);
	return min(percent(pehv_vout2vbus(info, vout), PEHV_VBUSOVP_RATIO),
		   desc->vta_cap_max);
}

/* Calculate IBUSOC S/W level */
static u32 pehv_get_dvchg_ibusocp(struct pehv_algo_info *info, u32 ita)
{
	u32 ibus, ratio = PEHV_IBUSOCP_RATIO;

	ibus = pehv_get_idvchg_lmt(info);

	return percent(ibus, ratio);
}

/* Calculate VBATOV S/W level */
static u32 pehv_get_vbatovp(struct pehv_algo_info *info)
{
	struct pehv_algo_data *data = info->data;

	return percent(data->vbat_cv, PEHV_VBATOVP_RATIO);
}

/* Calculate IBATOC S/W level */
static u32 pehv_get_ibatocp(struct pehv_algo_info *info, u32 ita)
{
	struct pehv_algo_data *data = info->data;
	u32 ibat;

	ibat = data->mmi_max_ibat;
	return percent(ibat, PEHV_IBATOCP_RATIO);
}

static int pehv_set_dvchg_protection(struct pehv_algo_info *info)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 ita_lmt, vbusovp, ibusocp, vbatovp, ibatocp;
	u32 idvchg_lmt;

	/* VBATOVP ALARM */
	ret = pehv_hal_set_vbatovp_alarm(info->alg, DVCHG1,
		data->is_swchg_en ? desc->swchg_off_vbat : data->vbat_cv);
	if (ret < 0) {
		PEHV_ERR("set vbatovp alarm fail(%d)\n", ret);
		return ret;
	}

	/* VBUSOVP */
	ita_lmt = pehv_get_ita_lmt(info);
	vbusovp = pehv_get_dvchg_vbusovp(info, ita_lmt);
	ret = pehv_hal_set_vbusovp(info->alg, DVCHG1, vbusovp);
	if (ret < 0) {
		PEHV_ERR("set vbusovp fail(%d)\n", ret);
		return ret;
	}
	if (data->is_dvchg_exist[PEHV_DVCHG_SLAVE]) {
		ret = pehv_hal_set_vbusovp(info->alg, DVCHG2, vbusovp);
		if (ret < 0) {
			PEHV_ERR("set slave vbusovp fail(%d)\n", ret);
			return ret;
		}
	}

	/* IBUSOCP */
	idvchg_lmt = data->idvchg_cc;
	ibusocp = percent(idvchg_lmt, PEHV_IBUSOCP_RATIO);
	ret = pehv_hal_set_ibusocp(info->alg, DVCHG1, ibusocp);
	if (ret < 0) {
		PEHV_ERR("set ibusocp fail(%d)\n", ret);
		return ret;
	}
	if (data->is_dvchg_exist[PEHV_DVCHG_SLAVE]) {
		/* Add 10% for unbalance tolerance */
		ibusocp = percent(precise_div(idvchg_lmt, 2),
				  PEHV_IBUSOCP_RATIO + 10);
		ret = pehv_hal_set_ibusocp(info->alg, DVCHG2, ibusocp);
		if (ret < 0) {
			PEHV_ERR("set slave ibusocp fail(%d)\n", ret);
			return ret;
		}
	}

	/* VBATOVP */
	vbatovp = pehv_get_vbatovp(info);
	ret = pehv_hal_set_vbatovp(info->alg, DVCHG1, vbatovp);
	if (ret < 0) {
		PEHV_ERR("set vbatovp fail(%d)\n", ret);
		return ret;
	}

	/* IBATOCP */
	ibatocp = pehv_get_ibatocp(info, ita_lmt);
	ret = pehv_hal_set_ibatocp(info->alg, DVCHG1, ibatocp);
	if (ret < 0) {
		PEHV_ERR("set ibatocp fail(%d)\n", ret);
		return ret;
	}
	PEHV_INFO("vbusovp,ibusocp,vbatovp,ibatocp = (%d,%d,%d,%d)\n",
		 vbusovp, ibusocp, vbatovp, ibatocp);
	return 0;
}

/*
 * Enable/Disable divider charger
 *
 * @en: enable/disable
 */
static int pehv_enable_dvchg_charging(struct pehv_algo_info *info,
				      enum pehv_dvchg_role role, bool en)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	if (!data->is_dvchg_exist[role])
		return -ENODEV;
	if (data->is_dvchg_en[role] == en)
		return 0;
	PEHV_INFO("en[%s] = %d\n", pehv_dvchg_role_name[role], en);
	ret = pehv_hal_enable_charging(info->alg, to_chgidx(role), en);
	if (ret < 0) {
		PEHV_ERR("en chg fail(%d)\n", ret);
		return ret;
	}
	data->is_dvchg_en[role] = en;
	msleep(desc->ta_blanking);
	return 0;
}

/*
 * Set protection parameters, disable swchg and  enable divider charger
 *
 * @en: enable/disable
 */
static int pehv_set_dvchg_charging(struct pehv_algo_info *info, bool en)
{
	int ret;
	struct pehv_algo_data *data = info->data;

	if (!data->is_dvchg_exist[PEHV_DVCHG_MASTER])
		return -ENODEV;

	PEHV_INFO("en = %d\n", en);

	if (en) {
		ret = pehv_hal_enable_hz(info->alg, CHG1, true);
		if (ret < 0) {
			PEHV_ERR("en swchg hz fail(%d)\n", ret);
			return ret;
		}
	}
	ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_MASTER, en);
	if (ret < 0)
		return ret;
	if (!en) {
		ret = pehv_hal_enable_hz(info->alg, CHG1, false);
		if (ret < 0) {
			PEHV_ERR("disable swchg hz fail(%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

/*
 * Enable charging of switching charger
 * For divide by two algorithm, according to swchg_ichg to decide enable or not
 *
 * @en: enable/disable
 */
static int pehv_enable_swchg_charging(struct pehv_algo_info *info, bool en)
{
	int ret = 0;
	struct pehv_algo_data *data = info->data;

	PEHV_INFO("en = %d\n", en);
	if (en) {
		ret = pehv_hal_enable_charging(info->alg, CHG1, true);
		if (ret < 0) {
			PEHV_ERR("en swchg fail(%d)\n", ret);
			return ret;
		}
		ret = pehv_hal_enable_hz(info->alg, CHG1, false);
		if (ret < 0) {
			PEHV_ERR("disable swchg hz fail(%d)\n", ret);
			return ret;
		}
	} else {
		ret = pehv_hal_enable_hz(info->alg, CHG1, true);
		if (ret < 0) {
			PEHV_ERR("en swchg hz fail(%d)\n", ret);
			return ret;
		}
		ret = pehv_hal_enable_charging(info->alg, CHG1, false);
		if (ret < 0) {
			PEHV_ERR("disable swchg fail(%d)\n", ret);
			return ret;
		}
	}
	data->is_swchg_en = en;
	return 0;
}

/*
 * Set TA vbus by algo
 *
 * @mV: requested output voltage
 */
static int pehv_set_ta_vbus(struct pehv_algo_info *info, int mV, int mA)
{
	int ret;
	struct pehv_algo_data *data = info->data;

	PEHV_INFO("set vbus = %d\n", mV);

	ret = pehv_hal_set_ta_vbus(info->alg, mV);
	if (ret < 0) {
		PEHV_ERR("set ta vbus fail(%d)\n", ret);
		return ret;
	}

	data->vta_setting = mV;
	data->ita_setting = mA;
	return ret;
}

static int pehv_send_notification(struct pehv_algo_info *info,
				  unsigned long val,
				  struct chg_alg_notify *notify)
{
	return srcu_notifier_call_chain(&info->alg->evt_nh, val, notify);
}

/* Stop PEHV charging and reset parameter */
static int pehv_stop(struct pehv_algo_info *info, struct pehv_stop_info *sinfo)
{
	struct pehv_algo_data *data = info->data;
	struct chg_alg_notify notify = {
		.evt = EVT_ALGO_STOP,
	};
	bool do_reset = false;

	if (data->state == PEHV_ALGO_STOP) {
		/*
		 * Always clear stop_algo,
		 * in case it is called from pehv_stop_algo
		 */
		atomic_set(&data->stop_algo, 0);
		PEHV_DBG("already stop\n");
		return 0;
	}

	PEHV_INFO("reset ta(%d), hardreset ta(%d)\n", sinfo->reset_ta,
		 sinfo->hardreset_ta);
	data->state = PEHV_ALGO_STOP;
	atomic_set(&data->stop_algo, 0);
	alarm_cancel(&data->timer);

	pehv_enable_dvchg_charging(info, PEHV_DVCHG_SLAVE, false);
	pehv_set_dvchg_charging(info, false);

	mutex_lock(&data->notify_lock);
	do_reset = !(data->notify & PEHV_RESET_NOTIFY);
	mutex_unlock(&data->notify_lock);
	if (do_reset) {
		if (sinfo->reset_ta)
			pehv_set_ta_vbus(info, PEHV_VTA_INIT,
				      PEHV_ITA_INIT);
	}

	pehv_enable_swchg_charging(info, true);
	pehv_hal_enable_sw_vbusovp(info->alg, true);
	pehv_send_notification(info, EVT_ALGO_STOP, &notify);
	return 0;
}

static inline void pehv_init_algo_data(struct pehv_algo_info *info)
{
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	data->ita_lmt =  desc->idvchg_cc;
	data->idvchg_ss_init = max_t(u32, desc->idvchg_ss_init,
				     desc->ita_cap_min);
	data->idvchg_ss_init = min(data->idvchg_ss_init, data->ita_lmt);
	data->ita_pwr_lmt = 0;
	data->tried_dual_dvchg = false;
	data->idvchg_cc = desc->idvchg_cc - desc->swchg_aicr;
	data->idvchg_term = desc->idvchg_term;
	data->err_retry_cnt = 0;
	data->is_swchg_en = false;
	data->is_dvchg_en[PEHV_DVCHG_MASTER] = false;
	data->is_dvchg_en[PEHV_DVCHG_SLAVE] = false;
	data->suspect_ta_cc = false;
	data->aicr_setting = 0;
	data->ichg_setting = 0;
	data->vta_setting = PEHV_VTA_INIT;
	data->ita_setting = PEHV_ITA_INIT;
	data->ita_gap_per_vstep = 0;
	data->ita_gap_window_idx = 0;
	memset(data->ita_gaps, 0, sizeof(data->ita_gaps));
	data->is_vbat_over_cv = false;
	data->ignore_ibusucpf = false;
	data->force_ta_cv = false;
	data->vbat_cv = desc->vbat_cv;
	data->vbat_cv_no_ircmp = desc->vbat_cv;
	data->cv_lower_bound = desc->vbat_cv - PEHV_CV_LOWER_BOUND_GAP;
	data->vta_comp = 0;
	data->zcv = 0;
	data->r_bat = desc->ircmp_rbat;
	data->r_sw = desc->rsw_min;
	data->state = PEHV_ALGO_INIT;
	mutex_lock(&data->notify_lock);
	data->notify = 0;
	mutex_unlock(&data->notify_lock);
	data->stime = ktime_get_boottime();
}

static int pehv_earily_restart(struct pehv_algo_info *info)
{
	int ret;

	ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_SLAVE, false);
	if (ret < 0) {
		PEHV_ERR("disable slave dvchg fail(%d)\n", ret);
		return ret;
	}
	ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_MASTER, false);
	if (ret < 0) {
		PEHV_ERR("disable master dvchg fail(%d)\n", ret);
		return ret;
	}

	ret = pehv_set_ta_vbus(info, PEHV_VTA_INIT,
				      PEHV_ITA_INIT);
	if (ret < 0) {
		PEHV_ERR("disable ta charging fail(%d)\n", ret);
		return ret;
	}
	pehv_init_algo_data(info);
	return 0;
}

/*
 * Start pehv timer and run algo
 * It cannot start algo again if algo has been started once before
 * Run once flag will be reset after plugging out TA
 */
static inline int pehv_start(struct pehv_algo_info *info)
{
	int ret, i;
	struct pehv_algo_data *data = info->data;
	ktime_t ktime = ktime_set(0, MS_TO_NS(PEHV_INIT_POLLING_INTERVAL));

	PEHV_DBG("++\n");

	/* disable charger */
	ret = pehv_hal_enable_charging(info->alg, CHG1, false);
	if (ret < 0) {
		PEHV_ERR("disable swchg fail(%d)\n", ret);
		return ret;
	}

	/* Check DVCHG registers stat first */
	for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
		if (!data->is_dvchg_exist[i])
			continue;
		ret = pehv_hal_init_chip(info->alg, to_chgidx(i));
		if (ret < 0) {
			PEHV_ERR("(%s) init chip fail(%d)\n",
				pehv_dvchg_role_name[i], ret);
			return ret;
		}
	}

	pehv_init_algo_data(info);
	alarm_start_relative(&data->timer, ktime);
	return 0;
}

/* =================================================================== */
/* PEHV Algo State Machine                                              */
/* =================================================================== */

static int pehv_adjust_vta_with_ta_cv(struct pehv_algo_info *info)
{
	int ret, cnt = PEHV_WHILE_LOOP_ITERATION_MAX;
	bool vbuslow = false, vbushigh = false;
	u32 idvchg_lmt, vta, ita, ita_gap_per_vstep;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	while (cnt-- > 0) {
		ret = pehv_hal_is_vbuslowerr(info->alg, DVCHG1, &vbuslow);
		if (ret < 0) {
			PEHV_ERR("get vbuslowerr fail(%d)\n", ret);
			return ret;
		}

		ret = pehv_hal_is_vbushigherr(info->alg, DVCHG1, &vbushigh);
		if (ret < 0) {
			PEHV_ERR("get vbushigherr fail(%d)\n", ret);
			return ret;
		}

		if (!vbuslow && !vbushigh)
			break;

		ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
				    data->ita_gap_per_vstep : 200;
		idvchg_lmt = pehv_get_idvchg_lmt(info);

		if (vbuslow) {
			vta = data->vta_setting + desc->vta_step;
			ita = data->ita_setting + ita_gap_per_vstep;
		} else if (vbushigh) {
			vta = data->vta_setting - desc->vta_step;
			ita = data->ita_setting - ita_gap_per_vstep;
		}

		vta = min_t(u32, vta, desc->vta_cap_max);
		ita = min(ita, idvchg_lmt);
		ret = pehv_set_ta_cap_cv(info, vta, ita);
		if (ret < 0) {
			PEHV_ERR("set ta cap fail(%d)\n", ret);
			return ret;
		}
	}
	return cnt >= 0 ? 0 : -EAGAIN;
}

static int pehv_algo_init_with_ta_cv(struct pehv_algo_info *info)
{
	int ret, i, vbus, vbat, vout;
	int vbat_avg = 0;
	const int avg_times = 10;
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_DBG("++\n");

	/* Change charging policy first */
	ret = pehv_set_ta_vbus(info, PEHV_VTA_INIT, PEHV_ITA_INIT);
	if (ret < 0) {
		PEHV_ERR("set ta vbus fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}

	ret = pehv_hal_enable_hz(info->alg, CHG1, false);
	if (ret < 0) {
		PEHV_ERR("disable swchg hz fail(%d)\n", ret);
		goto err;
	}

	/* Check VBAT after disabling CHG_EN and before enabling HZ */
	for (i = 0; i < avg_times; i++) {
		ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
		if (ret < 0) {
			PEHV_ERR("get vbat fail(%d)\n", ret);
			goto err;
		}
		vbat_avg += vbat;
	}
	vbat_avg = precise_div(vbat_avg, avg_times);
	data->zcv = vbat_avg;
	PEHV_INFO("avg(vbat):(%d)\n", vbat_avg);

	ret = pehv_enable_swchg_charging(info, false);
	if (ret < 0) {
		PEHV_ERR("disable swchg fail(%d)\n", ret);
		return ret;
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBUS, &vbus);
	if (ret < 0) {
		PEHV_ERR("get vbus fail(%d)\n", ret);
		goto err;
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VOUT, &vout);
	if (ret < 0) {
		PEHV_ERR("get vout fail(%d)\n", ret);
		goto err;
	}

	/* Adjust VBUS to make sure DVCHG can be turned on */
	ret = pehv_set_ta_cap_cv(info, pehv_vout2vbus(info, vout),
				 data->idvchg_ss_init);
	if (ret < 0) {
		PEHV_ERR("set ta cap fail(%d)\n", ret);
		goto err;
	}
	ret = pehv_adjust_vta_with_ta_cv(info);
	if (ret < 0) {
		PEHV_ERR("adjust vta fail(%d)\n", ret);
		goto err;
	}

	ret = pehv_set_dvchg_charging(info, true);
	if (ret < 0) {
		PEHV_ERR("en dvchg fail\n");
		goto err;
	}

	data->err_retry_cnt = 0;
	data->state = PEHV_ALGO_MEASURE_R;
	return 0;
err:
	if (data->err_retry_cnt < PEHV_INIT_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}
	return pehv_stop(info, &sinfo);
}

/*
 * PEHV algorithm initial state
 * It does Foreign Object Detection(FOD)
 */
static int pehv_algo_init(struct pehv_algo_info *info)
{

	return pehv_algo_init_with_ta_cv(info);
}

struct meas_r_info {
	u32 vbus;
	u32 ibus;
	u32 vbat;
	u32 ibat;
	u32 vout;
	u32 vta;
	u32 ita;
	u32 r_cable;
	u32 r_bat;
	u32 r_sw;
};

static int pehv_algo_get_r_info(struct pehv_algo_info *info,
				struct meas_r_info *r_info,
				struct pehv_stop_info *sinfo)
{
	int ret;

	memset(r_info, 0, sizeof(struct meas_r_info));

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBUS, &r_info->vbus);
	if (ret < 0) {
		PEHV_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBUS, &r_info->ibus);
	if (ret < 0) {
		PEHV_ERR("get ibus fail(%d)\n", ret);
		return ret;
	}
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VOUT, &r_info->vout);
	if (ret < 0) {
		PEHV_ERR("get vout fail(%d)\n", ret);
		return ret;
	}
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &r_info->vbat);
	if (ret < 0) {
		PEHV_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}
	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &r_info->ibat);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		return ret;
	}
	PEHV_DBG("vta:%d,ita:%d,vbus:%d,ibus:%d,vout:%d,vbat:%d,ibat:%d\n",
		 r_info->vta, r_info->ita, r_info->vbus, r_info->ibus,
		 r_info->vout, r_info->vbat, r_info->ibat);
	return 0;
}

static int pehv_algo_cal_r_info_with_ta_cap(struct pehv_algo_info *info,
					    struct pehv_stop_info *sinfo)
{
	int ret, i;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	struct meas_r_info r_info, max_r_info, min_r_info;
	struct pehv_stop_info _sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	memset(&max_r_info, 0, sizeof(struct meas_r_info));
	memset(&min_r_info, 0, sizeof(struct meas_r_info));
	data->r_bat = data->r_sw = data->r_cable = 0;
	for (i = 0; i < PEHV_MEASURE_R_AVG_TIMES + 2; i++) {
		if (atomic_read(&data->stop_algo)) {
			PEHV_INFO("stop algo\n");
			goto stop;
		}
		ret = pehv_algo_get_r_info(info, &r_info, sinfo);
		if (ret < 0) {
			PEHV_ERR("get r info fail(%d)\n", ret);
			return ret;
		}
		if (r_info.ibat == 0) {
			PEHV_ERR("ibat == 0 fail\n");
			return -EINVAL;
		}

		/* Use absolute instead of relative calculation */
		r_info.r_bat = precise_div(abs(r_info.vbat - data->zcv) * 1000,
					   abs(r_info.ibat));
		if (r_info.r_bat > desc->ircmp_rbat)
			r_info.r_bat = desc->ircmp_rbat;

		r_info.r_sw = precise_div(abs(r_info.vout - r_info.vbat) * 1000,
					  abs(r_info.ibat));
		if (r_info.r_sw < desc->rsw_min)
			r_info.r_sw = desc->rsw_min;

		PEHV_INFO("r_sw:%d, r_bat:%d, r_cable:%d\n", r_info.r_sw,
			  r_info.r_bat, r_info.r_cable);

		if (i == 0) {
			memcpy(&max_r_info, &r_info,
			       sizeof(struct meas_r_info));
			memcpy(&min_r_info, &r_info,
			       sizeof(struct meas_r_info));
		} else {
			max_r_info.r_bat = max(max_r_info.r_bat, r_info.r_bat);
			max_r_info.r_sw = max(max_r_info.r_sw, r_info.r_sw);

			min_r_info.r_bat = min(min_r_info.r_bat, r_info.r_bat);
			min_r_info.r_sw = min(min_r_info.r_sw, r_info.r_sw);
		}
		data->r_bat += r_info.r_bat;
		data->r_sw += r_info.r_sw;
	}
	data->r_bat -= (max_r_info.r_bat + min_r_info.r_bat);
	data->r_sw -= (max_r_info.r_sw + min_r_info.r_sw);
	data->r_bat = precise_div(data->r_bat, PEHV_MEASURE_R_AVG_TIMES);
	data->r_sw = precise_div(data->r_sw, PEHV_MEASURE_R_AVG_TIMES);

	data->r_total = data->r_bat + data->r_sw;
	return 0;
stop:
	pehv_stop(info, &_sinfo);
	return -EIO;
}

static int pehv_algo_measure_r_with_ta_cv(struct pehv_algo_info *info)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_DBG("++\n");

	ret = pehv_algo_cal_r_info_with_ta_cap(info, &sinfo);
	if (ret < 0) {
		PEHV_ERR("get r info fail(%d)\n", ret);
		goto err;
	}

	PEHV_ERR("avg_r(sw,bat,cable):(%d,%d,%d), r_total:%d\n",
		 data->r_sw, data->r_bat, data->r_cable, data->r_total);

	data->err_retry_cnt = 0;
	data->state = PEHV_ALGO_SS_DVCHG;
	return 0;
err:
	if (data->err_retry_cnt < PEHV_MEASURE_R_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}

	return pehv_stop(info, &sinfo);
}

/* Measure resistance of cable/battery/sw and get corressponding ita limit */
static int pehv_algo_measure_r(struct pehv_algo_info *info)
{

	return pehv_algo_measure_r_with_ta_cv(info);
}

static int pehv_check_slave_dvchg_off(struct pehv_algo_info *info)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	data->idvchg_term = desc->idvchg_term;
	data->idvchg_cc = desc->idvchg_cc / 2 - desc->swchg_aicr;
	ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_SLAVE, false);
	if (ret < 0) {
		PEHV_ERR("disable slave dvchg fail(%d)\n", ret);
		return ret;
	}

	return 0;
}

static int pehv_algo_ss_dvchg_with_ta_cv(struct pehv_algo_info *info)
{
	int ret, vbat, ibat, fcc_min;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 idvchg_lmt, vta, ita, ita_gap_per_vstep, vstep_cnt;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_DBG("++\n");
	ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
			    data->ita_gap_per_vstep : 200;

	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		goto out;
	}

	ret = pehv_get_ta_cap_by_supportive(info, &data->vta_measure,
					    &data->ita_measure);
	if (ret < 0) {
		PEHV_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}

	/* Turn on slave dvchg if ita_measure >= idvchg_ss_init */
	if (data->is_dvchg_exist[PEHV_DVCHG_SLAVE] && !data->tried_dual_dvchg &&
	    data->ita_measure >= desc->idvchg_ss_init &&
	    data->idvchg_cc <= pehv_get_ita_lmt(info)) {
		PEHV_INFO("try dual dvchg\n");
		data->tried_dual_dvchg = true;
		data->idvchg_term = 2 * desc->idvchg_term;
		data->idvchg_cc = desc->idvchg_cc - desc->swchg_aicr;

		ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_MASTER,
						 false);
		if (ret < 0) {
			PEHV_ERR("disable master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		data->ignore_ibusucpf = true;
		ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_MASTER, true);
		if (ret < 0) {
			PEHV_ERR("en master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ret = pehv_adjust_vta_with_ta_cv(info);
		if (ret < 0) {
			PEHV_ERR("adjust vta fail(%d)\n", ret);
			goto single_dvchg_restart;
		}

		msleep(200);
		ret = pehv_enable_dvchg_charging(info, PEHV_DVCHG_SLAVE, true);
		if (ret < 0) {
			PEHV_ERR("en slave dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		return 0;
single_dvchg_restart:
		ret = pehv_earily_restart(info);
		if (ret < 0) {
			PEHV_ERR("earily restart fail(%d)\n", ret);
			goto out;
		}
		return 0;
	}

	vta = data->vta_setting;
	ita = data->ita_setting;

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PEHV_ERR("get vbat fail(%d)\n", ret);
		goto out;
	}
	/* VBAT reaches CV level */
	PEHV_INFO("vbat(%d), ita(%d)\n", vbat, data->ita_measure);
	if (vbat >= data->vbat_cv) {
		if (data->ita_measure < data->idvchg_term &&
		    data->is_dvchg_en[PEHV_DVCHG_SLAVE]) {
			ret = pehv_check_slave_dvchg_off(info);
			if (ret < 0) {
				PEHV_INFO("slave off fail(%d)\n", ret);
				goto out;
			}
		}
		vta -= desc->vta_step;
		ita -= ita_gap_per_vstep;
		data->state = PEHV_ALGO_CC_CV;
		goto out_set_cap;
	}

	idvchg_lmt = pehv_get_idvchg_lmt(info);
	/* ITA reaches CC level */
	if (data->mmi_therm_fcc_limit > 0 &&
		data->mmi_therm_fcc_limit < data->mmi_fcc_limit)
		fcc_min = data->mmi_therm_fcc_limit;
	else
		fcc_min = data->mmi_fcc_limit;
	PEHV_INFO(" ss_dvchg fcc_result(%d), mmi_fcc(%d), mmi_therm_fcc(%d)\n",
				  fcc_min, data->mmi_fcc_limit, data->mmi_therm_fcc_limit);

	if (data->ita_measure + ita_gap_per_vstep > idvchg_lmt ||
	    vta == desc->vta_cap_max ||
	    ibat + 2 * MMI_IBAT_GAP_MA > fcc_min)
		data->state = PEHV_ALGO_CC_CV;
	else {
		vstep_cnt = precise_div(idvchg_lmt - data->ita_measure,
					3 * ita_gap_per_vstep);
		vta += desc->vta_step * (vstep_cnt + 1);
		vta = min(vta, desc->vta_cap_max);
		ita += ita_gap_per_vstep * (vstep_cnt + 1);
		ita = min(ita, idvchg_lmt);
	}

out_set_cap:
	ret = pehv_set_ta_cap_cv(info, vta, ita);
	if (ret < 0) {
		PEHV_ERR("set ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}
	return 0;
out:
	return pehv_stop(info, &sinfo);
}

/* Soft start of divider charger */
static int pehv_algo_ss_dvchg(struct pehv_algo_info *info)
{

	return pehv_algo_ss_dvchg_with_ta_cv(info);
}

static int mmi_thermal_ratio(struct pehv_algo_info *info, int ibat, int vbat)
{
	struct pehv_algo_data *data = info->data;
	int target_fcc = 0;
	int ratio;

	if (data->mmi_therm_fcc_limit > 0) {
		target_fcc = min((u32)data->mmi_therm_fcc_limit, (u32)data->mmi_fcc_limit);
		if (ibat - target_fcc > data->mmi_therm_cur_thres) {
			ratio = data->mmi_therm_step;
			PEHV_INFO("--current for thermal,ratio=%d, target_ibat = %d, now_ibat = %d\n",
				ratio, target_fcc, ibat);
		} else if(target_fcc - ibat > data->mmi_therm_cur_thres &&
			data->vbat_cv - vbat > data->mmi_therm_vol_thres) {
			ratio = data->mmi_therm_step;
			PEHV_INFO("++current for thermal,ratio=%d, target_ibat = %d, now_ibat = %d, cv = %d, now_vbat= %d\n",
				ratio,target_fcc, ibat, data->vbat_cv, vbat);
		} else {
			ratio = 1;
			PEHV_INFO("keep current for thermal,ratio=%d, target_ibat = %d, now_ibat = %d, cv = %d, now_vbat= %d\n",
				ratio,target_fcc, ibat, data->vbat_cv, vbat);
		}
	} else
		ratio = 1;

	return ratio;
}

static int pehv_algo_cc_cv_with_ta_cv(struct pehv_algo_info *info)
{
	int ret, vbat, ibat, vsys, fcc_min, ratio;
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	u32 idvchg_lmt, vta = data->vta_setting, ita = data->ita_setting;
	u32 ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
				data->ita_gap_per_vstep : 200;
	u32 vta_measure, ita_measure;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_DBG("++\n");

	ret = pehv_get_ta_cap_by_supportive(info, &data->vta_measure,
					    &data->ita_measure);
	if (ret < 0) {
		PEHV_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}
	if (data->ita_measure < data->idvchg_term &&
	    data->is_dvchg_en[PEHV_DVCHG_SLAVE]) {
		ret = pehv_check_slave_dvchg_off(info);
		if (ret < 0) {
			PEHV_INFO("slave off fail(%d)\n", ret);
			goto out;
		}
		ret = pehv_get_ta_cap_by_supportive(info,
						    &data->vta_measure,
						    &data->ita_measure);
		if (ret < 0) {
			PEHV_ERR("get ta cap fail(%d)\n", ret);
			sinfo.hardreset_ta = true;
			goto out;
		}
	}

	idvchg_lmt = pehv_get_idvchg_lmt(info);

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PEHV_ERR("get vbat fail(%d)\n", ret);
		goto out;
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VSYS, &vsys);
	if (ret < 0) {
		PEHV_ERR("get vsys fail(%d)\n", ret);
		goto out;
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		goto out;
	}

	if (data->mmi_therm_fcc_limit > 0 &&
		data->mmi_therm_fcc_limit < data->mmi_fcc_limit)
		fcc_min = data->mmi_therm_fcc_limit;
	else
		fcc_min = data->mmi_fcc_limit;
	PEHV_INFO(" cc_cv fcc_result(%d), mmi_fcc(%d), mmi_therm_fcc(%d)\n",
				  fcc_min, data->mmi_fcc_limit, data->mmi_therm_fcc_limit);

	if (vbat >= data->vbat_cv) {
		PEHV_INFO("--vbat >= vbat_cv, %d > %d\n", vbat, data->vbat_cv);
		vta -= desc->vta_step;
		ita -= ita_gap_per_vstep;
		data->is_vbat_over_cv = true;
	} else if (data->ita_measure > idvchg_lmt  || vsys >= PEHV_VSYS_UPPER_BOUND ||
		   idvchg_lmt < ita_gap_per_vstep) {
		vta -= desc->vta_step;
		ita -= ita_gap_per_vstep;
		PEHV_INFO("--vta, ita(meas,lmt)=(%d,%d)\n", data->ita_measure,
			  idvchg_lmt);
	} else if (ibat > fcc_min) {
		ratio = mmi_thermal_ratio(info, ibat, vbat);
		vta -= ratio *desc->vta_step;
		ita -= ratio *ita_gap_per_vstep;
		ita = max(ita, idvchg_lmt);
		PEHV_INFO("--vta, ibat(meas,lmt)=(%d,%d), ratio = %d\n", ibat, fcc_min, ratio);
	} else if (!data->is_vbat_over_cv && vbat <= data->cv_lower_bound &&
		   data->ita_measure <= (idvchg_lmt - ita_gap_per_vstep) &&  ibat <=  (fcc_min - MMI_IBAT_GAP_MA) &&
		   vta < desc->vta_cap_max &&
		   vsys < (PEHV_VSYS_UPPER_BOUND - PEHV_VSYS_UPPER_BOUND_GAP)) {
		ratio = mmi_thermal_ratio(info, ibat, vbat);
		vta += ratio * desc->vta_step;
		vta = min(vta, desc->vta_cap_max);
		ita += ratio * ita_gap_per_vstep;
		ita = min(ita, idvchg_lmt);
		PEHV_INFO("++vta, ita(meas,lmt)=(%d,%d), mmi_fcc = %d, ratio = %d\n", data->ita_measure,
			  idvchg_lmt, fcc_min, ratio);
	} else if (data->is_vbat_over_cv)
		data->is_vbat_over_cv = false;

	ret = pehv_set_ta_cap_cv(info, vta, ita);
	if (ret < 0) {
		PEHV_ERR("set_ta_cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}
	ret = pehv_get_ta_cap_by_supportive(info, &vta_measure, &ita_measure);
	if (ret < 0) {
		PEHV_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}

	return 0;
out:
	return pehv_stop(info, &sinfo);
}

static int pehv_algo_cc_cv(struct pehv_algo_info *info)
{

	return pehv_algo_cc_cv_with_ta_cv(info);
}

/*
 * Check charging time of pehv algorithm
 * return false if timeout otherwise return true
 */
static bool pehv_check_charging_time(struct pehv_algo_info *info,
				     struct pehv_stop_info *sinfo)
{
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	ktime_t etime, time_diff;
	struct timespec64 dtime;

	etime = ktime_get_boottime();
	time_diff = ktime_sub(etime, data->stime);
	dtime = ktime_to_timespec64(time_diff);
	if (dtime.tv_sec >= desc->chg_time_max) {
		PEHV_ERR("PEHV algo timeout(%d, %d)\n", (int)dtime.tv_sec,
			 desc->chg_time_max);
		return false;
	}
	return true;
}

/*
 * Check EOC of pehv algorithm
 * return false if EOC otherwise return true
 */
static bool pehv_check_eoc(struct pehv_algo_info *info,
			   struct pehv_stop_info *sinfo)
{
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;
	int ret = 0, vbat = 0, ibat = 0;
	u32 soc = 0, ita_lmt = 0;
	bool algo_running = !(data->state == PEHV_ALGO_STOP);

	ret = pehv_hal_get_soc(info->alg, &soc);
	if (ret < 0)
		PEHV_ERR("get SOC fail(%d)\n", ret);
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
	if (ret < 0)
		PEHV_ERR("get vbat fail(%d)\n", ret);
	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0)
		PEHV_ERR("get ibat fail(%d)\n", ret);

	if (soc >= desc->stop_soc_max &&
	    vbat > (data->vbat_cv - desc->vbat_max_gap) &&
	    ibat < (data->idvchg_term * 2)) {
		if (algo_running)
			data->start_soc_max = desc->start_soc_max -
					      PEHV_START_SOC_MAX_GAP;
		return false;
	}

	ita_lmt = pehv_get_ita_lmt(info);
	/* Consider AICR is decreased */
	ita_lmt = min(ita_lmt, data->is_swchg_en ?
		      (data->idvchg_cc + data->aicr_setting) : data->idvchg_cc);
	if (ita_lmt < data->idvchg_term) {
		PEHV_INFO("ita_lmt(%d) < idvchg_term(%d)\n", ita_lmt,
			 data->idvchg_term);
		return false;
	}

	return true;
}

static bool pehv_check_dvchg_ibusocp(struct pehv_algo_info *info,
				     struct pehv_stop_info *sinfo)
{
	int ret, i, ibus, acc = 0;
	struct pehv_algo_data *data = info->data;
	u32 ibusocp;

	if (!data->is_dvchg_en[PEHV_DVCHG_MASTER])
		return true;
	ibusocp = pehv_get_dvchg_ibusocp(info, data->ita_setting);
	for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		pehv_hal_get_adc_accuracy(info->alg, to_chgidx(i),
					  PEHV_ADCCHAN_IBUS, &acc);
		ret = pehv_hal_get_adc(info->alg, to_chgidx(i),
				       PEHV_ADCCHAN_IBUS, &ibus);
		if (ret < 0) {
			PEHV_ERR("get ibus fail(%d)\n", ret);
			return false;
		}
		PEHV_INFO("(%s)ibus(%d+-%dmA), ibusocp(%dmA)\n",
			 pehv_dvchg_role_name[i], ibus, acc, ibusocp);
		if (ibus > acc)
			ibus -= acc;
		if (ibus > ibusocp) {
			PEHV_ERR("(%s)ibus(%dmA) > ibusocp(%dmA)\n",
				 pehv_dvchg_role_name[i], ibus, ibusocp);
			return false;
		}
	}
	return true;
}

/*
 * Check VBUS voltage of divider charger
 * return false if VBUS is over voltage otherwise return true
 */
static bool pehv_check_dvchg_vbusovp(struct pehv_algo_info *info,
				     struct pehv_stop_info *sinfo)
{
	int ret, vbus, i;
	struct pehv_algo_data *data = info->data;
	u32 vbusovp;

	vbusovp = pehv_get_dvchg_vbusovp(info, data->ita_setting);
	for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pehv_hal_get_adc(info->alg, to_chgidx(i),
				       PEHV_ADCCHAN_VBUS, &vbus);
		if (ret < 0) {
			PEHV_ERR("get vbus fail(%d)\n", ret);
			return false;
		}
		PEHV_INFO("(%s)vbus(%dmV), vbusovp(%dmV)\n",
			  pehv_dvchg_role_name[i], vbus, vbusovp);
		if (vbus > vbusovp) {
			PEHV_ERR("(%s)vbus(%dmV) > vbusovp(%dmV)\n",
				 pehv_dvchg_role_name[i], vbus, vbusovp);
			return false;
		}
	}
	return true;
}

static bool pehv_check_vbatovp(struct pehv_algo_info *info,
			       struct pehv_stop_info *sinfo)
{
	int ret, vbat;
	u32 vbatovp;

	vbatovp = pehv_get_vbatovp(info);
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PEHV_ERR("get vbat fail(%d)\n", ret);
		return false;
	}
	PEHV_INFO("vbat(%dmV), vbatovp(%dmV)\n", vbat, vbatovp);
	if (vbat > vbatovp) {
		PEHV_ERR("vbat(%dmV) > vbatovp(%dmV)\n", vbat, vbatovp);
		return false;
	}
	return true;
}

static bool pehv_check_ibatocp(struct pehv_algo_info *info,
			       struct pehv_stop_info *sinfo)
{
	int ret, ibat;
	struct pehv_algo_data *data = info->data;
	u32 ibatocp;

	if (!data->is_dvchg_en[PEHV_DVCHG_MASTER])
		return true;
	ibatocp = pehv_get_ibatocp(info, data->ita_setting);
	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		return false;
	}
	PEHV_INFO("ibat(%dmA), ibatocp(%dmA)\n", ibat, ibatocp);
	if (ibat < 0) {
		PEHV_ERR("ibat(%dmA) < 0)\n", ibat);
		return true;
	}
	if (ibat > ibatocp) {
		PEHV_ERR("ibat(%dmA) > ibatocp(%dmA)\n", ibat, ibatocp);
		return false;
	}
	return true;
}

struct pehv_safety_check_fn_desc {
	bool (*fn)(struct pehv_algo_info *info,
		   struct pehv_stop_info *sinfo);
	bool check_during_running;
};

static struct pehv_safety_check_fn_desc fn_descs[] = {
	{pehv_check_charging_time, true},
	{pehv_check_dvchg_vbusovp, true},
	{pehv_check_dvchg_ibusocp, true},
	{pehv_check_vbatovp, true},
	{pehv_check_ibatocp, true},
	{pehv_check_eoc, false},
};

static bool pehv_algo_safety_check(struct pehv_algo_info *info)
{
	unsigned int i;
	struct pehv_algo_data *data = info->data;
	bool algo_running = !(data->state == PEHV_ALGO_STOP);
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_DBG("++\n");
	for (i = 0; i < ARRAY_SIZE(fn_descs); i++) {
		if (!algo_running && fn_descs[i].check_during_running)
			continue;
		if (!fn_descs[i].fn(info, &sinfo))
			goto err;
	}
	return true;
err:
	pehv_stop(info, &sinfo);
	return false;
}

static inline void pehv_wakeup_algo_thread(struct pehv_algo_data *data)
{
	PEHV_DBG("++\n");
	atomic_set(&data->wakeup_thread, 1);
	wake_up_interruptible(&data->wq);
}

static enum alarmtimer_restart
pehv_algo_timer_cb(struct alarm *alarm, ktime_t now)
{
	struct pehv_algo_data *data =
		container_of(alarm, struct pehv_algo_data, timer);

	PEHV_DBG("++\n");
	pehv_wakeup_algo_thread(data);
	return ALARMTIMER_NORESTART;
}

static void pehv_algo_data_partial_reset(struct pehv_algo_info *info)
{
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	data->idvchg_cc = desc->idvchg_cc;
	data->idvchg_term = desc->idvchg_term;
	data->ita_lmt = desc->idvchg_cc;
	data->ita_pwr_lmt = 0;
	mutex_lock(&data->ext_lock);
	data->input_current_limit = -1;
	data->cv_limit = -1;
	mutex_unlock(&data->ext_lock);
	data->start_soc_max = desc->start_soc_max;
}

static inline int __pehv_plugout_reset(struct pehv_algo_info *info,
				       struct pehv_stop_info *sinfo)
{
	struct pehv_algo_data *data = info->data;

	PEHV_DBG("++\n");
	data->ta_ready = false;

	pehv_algo_data_partial_reset(info);
	return pehv_stop(info, sinfo);
}

static int pehv_notify_hardreset_hdlr(struct pehv_algo_info *info)
{
	struct pehv_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	PEHV_INFO("++\n");
	return __pehv_plugout_reset(info, &sinfo);
}

static int pehv_notify_softreset_hdlr(struct pehv_algo_info *info)
{
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	return __pehv_plugout_reset(info, &sinfo);
}

static int pehv_notify_detach_hdlr(struct pehv_algo_info *info)
{
	struct pehv_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	PEHV_INFO("++\n");
	return __pehv_plugout_reset(info, &sinfo);
}

static int pehv_notify_hwerr_hdlr(struct pehv_algo_info *info)
{
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PEHV_INFO("++\n");
	return pehv_stop(info, &sinfo);
}

static int pehv_notify_ibusucpf_hdlr(struct pehv_algo_info *info)
{
	int ret, ibus;
	struct pehv_algo_data *data = info->data;

	if (data->ignore_ibusucpf) {
		PEHV_INFO("ignore ibusucpf\n");
		data->ignore_ibusucpf = false;
		return 0;
	}
	if (!data->is_dvchg_en[PEHV_DVCHG_MASTER]) {
		PEHV_INFO("master dvchg is off\n");
		return 0;
	}
	/* Last chance */
	ret = pehv_hal_get_adc(info->alg, DVCHG1, PEHV_ADCCHAN_IBUS, &ibus);
	if (ret < 0) {
		PEHV_ERR("get dvchg ibus fail(%d)\n", ret);
		goto out;
	}
	if (ibus < PEHV_IBUSUCPF_RECHECK) {
		PEHV_ERR("ibus(%d) < recheck(%d)\n", ibus,
			 PEHV_IBUSUCPF_RECHECK);
		goto out;
	}
	PEHV_INFO("recheck ibus and it is not ucp\n");
	return 0;
out:
	return pehv_notify_hwerr_hdlr(info);
}

static int pehv_notify_vbatovp_alarm_hdlr(struct pehv_algo_info *info)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->state == PEHV_ALGO_STOP)
		return 0;
	PEHV_INFO("++\n");
	ret = pehv_hal_reset_vbatovp_alarm(info->alg, DVCHG1);
	if (ret < 0) {
		PEHV_ERR("reset vbatovp alarm fail(%d)\n", ret);
		return pehv_stop(info, &sinfo);
	}
	return 0;
}

static int pehv_notify_vbusovp_alarm_hdlr(struct pehv_algo_info *info)
{
	int ret;
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->state == PEHV_ALGO_STOP)
		return 0;
	PEHV_INFO("++\n");
	ret = pehv_hal_reset_vbusovp_alarm(info->alg, DVCHG1);
	if (ret < 0) {
		PEHV_ERR("reset vbusovp alarm fail(%d)\n", ret);
		return pehv_stop(info, &sinfo);
	}
	return 0;
}

static int
(*pehv_notify_pre_hdlr[EVT_MAX])(struct pehv_algo_info *info) = {
	[EVT_DETACH] = pehv_notify_detach_hdlr,
	[EVT_HARDRESET] = pehv_notify_hardreset_hdlr,
	[EVT_SOFTRESET] = pehv_notify_softreset_hdlr,
	[EVT_VBUSOVP] = pehv_notify_hwerr_hdlr,
	[EVT_IBUSOCP] = pehv_notify_hwerr_hdlr,
	[EVT_IBUSUCP_FALL] = pehv_notify_ibusucpf_hdlr,
	[EVT_VBATOVP] = pehv_notify_hwerr_hdlr,
	[EVT_IBATOCP] = pehv_notify_hwerr_hdlr,
	[EVT_VOUTOVP] = pehv_notify_hwerr_hdlr,
	[EVT_VDROVP] = pehv_notify_hwerr_hdlr,
	[EVT_VBATOVP_ALARM] = pehv_notify_vbatovp_alarm_hdlr,
};

static int
(*pehv_notify_post_hdlr[EVT_MAX])(struct pehv_algo_info *info) = {
	[EVT_DETACH] = pehv_notify_detach_hdlr,
	[EVT_HARDRESET] = pehv_notify_hardreset_hdlr,
	[EVT_SOFTRESET] = pehv_notify_softreset_hdlr,
	[EVT_VBUSOVP] = pehv_notify_hwerr_hdlr,
	[EVT_IBUSOCP] = pehv_notify_hwerr_hdlr,
	[EVT_IBUSUCP_FALL] = pehv_notify_ibusucpf_hdlr,
	[EVT_VBATOVP] = pehv_notify_hwerr_hdlr,
	[EVT_IBATOCP] = pehv_notify_hwerr_hdlr,
	[EVT_VOUTOVP] = pehv_notify_hwerr_hdlr,
	[EVT_VDROVP] = pehv_notify_hwerr_hdlr,
	[EVT_VBATOVP_ALARM] = pehv_notify_vbatovp_alarm_hdlr,
	[EVT_VBUSOVP_ALARM] = pehv_notify_vbusovp_alarm_hdlr,
};

static int pehv_handle_notify_evt(struct pehv_algo_info *info,
				  int (**hdlrs)(struct pehv_algo_info *))
{
	unsigned int i;
	struct pehv_algo_data *data = info->data;

	mutex_lock(&data->notify_lock);
	PEHV_DBG("0x%08X\n", data->notify);
	for (i = 0; i < EVT_MAX; i++) {
		if ((data->notify & BIT(i)) && hdlrs[i]) {
			data->notify &= ~BIT(i);
			mutex_unlock(&data->notify_lock);
			hdlrs[i](info);
			mutex_lock(&data->notify_lock);
		}
	}
	mutex_unlock(&data->notify_lock);
	return 0;
}

static int pehv_dump_charging_info(struct pehv_algo_info *info)
{
	int ret = 0, i = 0;
	int vbus = 0, ibus[PEHV_DVCHG_MAX] = {0}, ibus_swchg = 0, vbat = 0,
	    ibat = 0, vout[PEHV_DVCHG_MAX] = {0};
	int ibus_total = 0, vsys = 0, tbat = 0;
	u32 soc = 0;
	struct pehv_algo_data *data = info->data;

	/* vbus */
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBUS, &vbus);
	if (ret < 0)
		PEHV_ERR("get vbus fail(%d)\n", ret);
	/* ibus */
	for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pehv_hal_get_adc(info->alg, to_chgidx(i),
				       PEHV_ADCCHAN_IBUS, &ibus[i]);
		if (ret < 0) {
			PEHV_ERR("get %s ibus fail\n", pehv_dvchg_role_name[i]);
			continue;
		}
	}

	ibus_total = ibus[PEHV_DVCHG_MASTER] + ibus[PEHV_DVCHG_SLAVE];

	if (data->is_swchg_en) {
		ret = pehv_hal_get_adc(info->alg, CHG1, PEHV_ADCCHAN_IBUS,
				       &ibus_swchg);
		if (ret < 0)
			PEHV_ERR("get swchg ibus fail\n");
	}
	/* vbat */
	ret = pehv_get_adc(info, PEHV_ADCCHAN_VBAT, &vbat);
	if (ret < 0)
		PEHV_ERR("get vbat fail\n");
	/* ibat */
	ret = pehv_get_adc(info, PEHV_ADCCHAN_IBAT, &ibat);
	if (ret < 0)
		PEHV_ERR("get ibat fail\n");

	ret = pehv_get_ta_cap_by_supportive(info, &data->vta_measure,
					     &data->ita_measure);
	if (ret < 0)
		PEHV_ERR("get ta measure cap fail(%d)\n", ret);

	/* vout */
	for (i = PEHV_DVCHG_MASTER; i < PEHV_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pehv_hal_get_adc(info->alg, to_chgidx(i), PEHV_ADCCHAN_VOUT,
					   &vout[i]);
		if (ret < 0) {
			PEHV_ERR("get %s ibus fail\n", pehv_dvchg_role_name[i]);
			continue;
		}
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_VSYS, &vsys);
	if (ret < 0) {
		PEHV_ERR("get vsys from swchg fail\n");
	}

	ret = pehv_get_adc(info, PEHV_ADCCHAN_TBAT, &tbat);

	ret = pehv_hal_get_soc(info->alg, &soc);
	if (ret < 0)
		PEHV_ERR("get SOC fail\n");

	PEHV_INFO("vbus,ibus(master,slave,sw),vbat,ibat=%d,(%d,%d,%d),%d,%d\n",
		 vbus, ibus[PEHV_DVCHG_MASTER], ibus[PEHV_DVCHG_SLAVE],
		 ibus_swchg, vbat, ibat);
	PEHV_INFO("vta,ita(set,meas)=(%d,%d),(%d,%d),force_cv=%d\n",
		 data->vta_setting, data->vta_measure, data->ita_setting,
		 data->ita_measure, data->force_ta_cv);
	PEHV_INFO("vout(master,slave)=(%d,%d)\n",
		 vout[PEHV_DVCHG_MASTER], vout[PEHV_DVCHG_SLAVE]);
	PEHV_INFO("[PEHV] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 vbus, ibus[PEHV_DVCHG_MASTER], ibus[PEHV_DVCHG_SLAVE], ibus_total, vbat, ibat,
		 tbat, vsys, soc,
		 vout[PEHV_DVCHG_MASTER], vout[PEHV_DVCHG_SLAVE]);

	return 0;
}

static int pehv_algo_threadfn(void *param)
{
	struct pehv_algo_info *info = param;
	struct pehv_algo_data *data = info->data;
	u32 sec, ms, polling_interval;
	ktime_t ktime;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	while (!kthread_should_stop()) {
		wait_event_interruptible(data->wq,
					 atomic_read(&data->wakeup_thread));
		if (atomic_read(&data->stop_thread))
			break;
		pm_stay_awake(info->dev);
		atomic_set(&data->wakeup_thread, 0);
		mutex_lock(&data->lock);
		PEHV_INFO("state = %s\n", pehv_algo_state_name[data->state]);
		if (atomic_read(&data->stop_algo))
			pehv_stop(info, &sinfo);
		pehv_handle_notify_evt(info, pehv_notify_pre_hdlr);
		if (data->state != PEHV_ALGO_STOP) {
			pehv_calculate_vbat_ircmp(info);
			pehv_select_vbat_cv(info);
			pehv_dump_charging_info(info);
			if (pehv_algo_safety_check(info))
				pehv_set_dvchg_protection(info);
		}
		pehv_handle_notify_evt(info, pehv_notify_pre_hdlr);
		switch (data->state) {
		case PEHV_ALGO_INIT:
			pehv_algo_init(info);
			break;
		case PEHV_ALGO_MEASURE_R:
			pehv_algo_measure_r(info);
			break;
		case PEHV_ALGO_SS_SWCHG:
			//do nothing;
			break;
		case PEHV_ALGO_SS_DVCHG:
			pehv_algo_ss_dvchg(info);
			break;
		case PEHV_ALGO_CC_CV:
			pehv_algo_cc_cv(info);
			break;
		case PEHV_ALGO_STOP:
			PEHV_INFO("PEHV STOP\n");
			break;
		default:
			PEHV_ERR("NO SUCH STATE\n");
			break;
		}
		pehv_handle_notify_evt(info, pehv_notify_post_hdlr);
		if (data->state != PEHV_ALGO_STOP) {
			if (data->state == PEHV_ALGO_CC_CV) {
				polling_interval = PEHV_CV_POLLING_INTERVAL;
			} else {
				polling_interval = PEHV_INIT_POLLING_INTERVAL;
			}
			sec = polling_interval / 1000;
			ms = polling_interval % 1000;
			ktime = ktime_set(sec, MS_TO_NS(ms));
			alarm_start_relative(&data->timer, ktime);
		}
		mutex_unlock(&data->lock);
		pm_relax(info->dev);
	}
	return 0;
}

/* =================================================================== */
/* PEHV Algo OPS                                                        */
/* =================================================================== */

static int pehv_init_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;

	mutex_lock(&data->lock);
	PEHV_DBG("++\n");

	if (data->inited) {
		PEHV_INFO("already inited\n");
		goto out;
	}
	ret = pehv_hal_init_hardware(info->alg);
	if (ret) {
		PEHV_ERR("%s: init hw fail\n", __func__);
		goto out;
	}
	data->inited = true;
	PEHV_INFO("successfully\n");
out:
	mutex_unlock(&data->lock);
	return ret;
}

static bool pehv_is_algo_running(struct chg_alg_device *alg)
{
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	bool running = true;

	mutex_lock(&data->lock);

	if (!data->inited) {
		running = false;
		goto out_unlock;
	}
	running = !(data->state == PEHV_ALGO_STOP);
	PEHV_DBG("running = %d\n", running);
out_unlock:
	mutex_unlock(&data->lock);

	return running;
}

static int pehv_is_algo_ready(struct chg_alg_device *alg)
{
	int ret = 0;
	int soc = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	struct pehv_algo_desc *desc = info->desc;

	if (algo_waiver_test)
		return ALG_WAIVER;

	if (pehv_is_algo_running(info->alg))
		return ALG_RUNNING;

	mutex_lock(&data->lock);
	PEHV_DBG("++\n");
	if (!data->inited) {
		ret = ALG_INIT_FAIL;
		goto out;
	}

	mutex_lock(&data->notify_lock);
	if (data->notify & PEHV_RESET_NOTIFY) {
		PEHV_INFO("detach/hardreset happened\n");
		data->notify &= ~PEHV_RESET_NOTIFY;
		data->ta_ready = false;
	}
	mutex_unlock(&data->notify_lock);

	ret = pehv_hal_get_soc(info->alg, &soc);
	if (ret < 0) {
		PEHV_ERR("get SOC fail(%d)\n", ret);
		ret = ALG_INIT_FAIL;
		goto out;
	}
	if (soc < desc->start_soc_min || soc > data->start_soc_max) {
		if (soc > 0) {
			PEHV_INFO("soc(%d) not in range(%d~%d)\n", soc,
				  desc->start_soc_min, data->start_soc_max);
			ret = ALG_WAIVER;
			goto out;
		}
		if (soc == -1 && data->ref_vbat > data->vbat_threshold) {
			PEHV_INFO("vbat(%d) > %d\n", data->ref_vbat,
				  data->vbat_threshold);
			ret = ALG_WAIVER;
			goto out;
		}
	}

	if (!pehv_algo_safety_check(info)) {
		ret = ALG_NOT_READY;
		goto out;
	}

	if (!data->ta_ready) {
		ret = pehv_hal_is_hv_adapter_ready(alg);
		if (ret == ALG_READY)
			data->ta_ready = true;
		goto out;
	}
	ret = ALG_READY;
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pehv_start_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;

	if (pehv_is_algo_running(alg))
		return ALG_RUNNING;
	mutex_lock(&data->lock);
	PEHV_DBG("++\n");
	if (!data->inited || !data->ta_ready) {
		ret = ALG_INIT_FAIL;
		goto out;
	}
	pehv_hal_enable_sw_vbusovp(alg, false);
	ret = pehv_start(info);
	if (ret < 0) {
		pehv_hal_enable_sw_vbusovp(alg, true);
		PEHV_ERR("start PEHV algo fail\n");
		ret = ALG_INIT_FAIL;
	}
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pehv_plugout_reset(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	mutex_lock(&data->lock);
	PEHV_DBG("++\n");
	if (!data->inited)
		goto out;
	ret = __pehv_plugout_reset(info, &sinfo);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pehv_stop_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	struct pehv_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	atomic_set(&data->stop_algo, 1);
	mutex_lock(&data->lock);
	if (!data->inited)
		goto out;
	data->ta_ready = false;
	data->run_once = false;
	ret = pehv_stop(info, &sinfo);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pehv_notifier_call(struct chg_alg_device *alg,
			      struct chg_alg_notify *notify)
{
	int ret = 0;
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;

	mutex_lock(&data->notify_lock);
	if (data->state == PEHV_ALGO_STOP) {
		if (notify->evt == EVT_DETACH || notify->evt == EVT_HARDRESET) {
			PEHV_INFO("detach/hardreset after stop\n");
			data->notify |= BIT(notify->evt);
		}
		goto out;
	}
	PEHV_INFO("%s\n", chg_alg_notify_evt_tostring(notify->evt));
	switch (notify->evt) {
	case EVT_DETACH:
	case EVT_HARDRESET:
	case EVT_SOFTRESET:
	case EVT_VBUSOVP:
	case EVT_IBUSOCP:
	case EVT_IBUSUCP_FALL:
	case EVT_VBATOVP:
	case EVT_IBATOCP:
	case EVT_VOUTOVP:
	case EVT_VDROVP:
	case EVT_VBATOVP_ALARM:
	case EVT_VBUSOVP_ALARM:
		data->notify |= BIT(notify->evt);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
	pehv_wakeup_algo_thread(data);
out:
	mutex_unlock(&data->notify_lock);
	return ret;
}

static int pehv_set_current_limit(struct chg_alg_device *alg,
				  struct chg_limit_setting *setting)
{
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	int cv = micro_to_milli(setting->cv);
	int ic = micro_to_milli(setting->input_current_limit_dvchg1);
	int fcc = micro_to_milli(setting->mmi_fcc_limit);
	int therm_fcc = micro_to_milli(setting->mmi_current_limit_dvchg1);

	mutex_lock(&data->ext_lock);
	if (data->cv_limit != cv || data->input_current_limit != ic
		|| data->mmi_fcc_limit != fcc
		|| data->mmi_therm_fcc_limit != therm_fcc) {
		data->cv_limit = cv;
		data->input_current_limit = ic;
		data->mmi_fcc_limit = fcc;
		data->mmi_therm_fcc_limit = therm_fcc;
		PEHV_INFO("ic = %d, cv = %d, mmi_fcc = %d, therm_fcc = %d\n", ic, cv, fcc, therm_fcc);
		pehv_wakeup_algo_thread(data);
	}
	mutex_unlock(&data->ext_lock);
	return 0;
}

static int pehv_get_prop(struct chg_alg_device *alg,
			 enum chg_alg_props s, int *value)
{
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_desc *desc = info->desc;

	pr_notice("%s %d\n", __func__, s);

	switch (s) {
	case ALG_MAX_VBUS:
		*value = desc->vta_cap_max;
		break;
	default:
		break;
	}

	return 0;
}

static int pehv_set_prop(struct chg_alg_device *alg,
			 enum chg_alg_props s, int value)
{
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;

	pr_notice("%s %d %d\n", __func__, s, value);

	switch (s) {
	case ALG_LOG_LEVEL:
		log_level = value;
		break;
	case ALG_REF_VBAT:
		data->ref_vbat = value;
		break;
	default:
		break;
	}

	return 0;
}

static struct chg_alg_ops pehv_ops = {
	.init_algo = pehv_init_algo,
	.is_algo_ready = pehv_is_algo_ready,
	.start_algo = pehv_start_algo,
	.is_algo_running = pehv_is_algo_running,
	.plugout_reset = pehv_plugout_reset,
	.stop_algo = pehv_stop_algo,
	.notifier_call = pehv_notifier_call,
	.set_current_limit = pehv_set_current_limit,
	.get_prop = pehv_get_prop,
	.set_prop = pehv_set_prop,
};

#define PEHV_DT_VALPROP_ARR(name, var_name, sz) \
	{name, offsetof(struct pehv_algo_desc, var_name), sz}

#define PEHV_DT_VALPROP(name, var_name) \
	PEHV_DT_VALPROP_ARR(name, var_name, 1)

struct pehv_dtprop {
	const char *name;
	size_t offset;
	size_t sz;
};

static inline void pehv_parse_dt_u32(struct device_node *np, void *desc,
				     const struct pehv_dtprop *props,
				     int prop_cnt)
{
	int i;

	for (i = 0; i < prop_cnt; i++) {
		if (unlikely(!props[i].name))
			continue;
		of_property_read_u32(np, props[i].name, desc + props[i].offset);
	}
}

static const struct pehv_dtprop pehv_dtprops_u32[] = {
	PEHV_DT_VALPROP("polling-interval", polling_interval),
	PEHV_DT_VALPROP("vbat-cv", vbat_cv),
	PEHV_DT_VALPROP("start-soc-min", start_soc_min),
	PEHV_DT_VALPROP("start-soc-max", start_soc_max),
	PEHV_DT_VALPROP("stop-soc-max", stop_soc_max),
	PEHV_DT_VALPROP("vbat-max-gap", vbat_max_gap),
	PEHV_DT_VALPROP("idvchg-cc", idvchg_cc),
	PEHV_DT_VALPROP("idvchg-term", idvchg_term),
	PEHV_DT_VALPROP("idvchg-step", idvchg_step),
	PEHV_DT_VALPROP("idvchg-ss-init", idvchg_ss_init),
	PEHV_DT_VALPROP("idvchg-ss-step", idvchg_ss_step),
	PEHV_DT_VALPROP("idvchg-ss-step1", idvchg_ss_step1),
	PEHV_DT_VALPROP("idvchg-ss-step2", idvchg_ss_step2),
	PEHV_DT_VALPROP("idvchg-ss-step1-vbat", idvchg_ss_step1_vbat),
	PEHV_DT_VALPROP("idvchg-ss-step2-vbat", idvchg_ss_step2_vbat),
	PEHV_DT_VALPROP("ta-blanking", ta_blanking),
	PEHV_DT_VALPROP("swchg-aicr", swchg_aicr),
	PEHV_DT_VALPROP("swchg-ichg", swchg_ichg),
	PEHV_DT_VALPROP("swchg-aicr-ss-init", swchg_aicr_ss_init),
	PEHV_DT_VALPROP("swchg-aicr-ss-step", swchg_aicr_ss_step),
	PEHV_DT_VALPROP("swchg-off-vbat", swchg_off_vbat),
	PEHV_DT_VALPROP("force-ta-cv-vbat", force_ta_cv_vbat),
	PEHV_DT_VALPROP("chg-time-max", chg_time_max),
	PEHV_DT_VALPROP("ifod-threshold", ifod_threshold),
	PEHV_DT_VALPROP("rsw-min", rsw_min),
	PEHV_DT_VALPROP("ircmp-rbat", ircmp_rbat),
	PEHV_DT_VALPROP("ircmp-vclamp", ircmp_vclamp),
	PEHV_DT_VALPROP("vta-cap-min", vta_cap_min),
	PEHV_DT_VALPROP("vta-cap-max", vta_cap_max),
	PEHV_DT_VALPROP("vta-step", vta_step),
	PEHV_DT_VALPROP("ita-cap-min", ita_cap_min),
};

static int pehv_parse_dt(struct pehv_algo_info *info)
{
	struct pehv_algo_desc *desc;
	struct pehv_algo_data *data;
	struct device_node *np = info->dev->of_node;
	u32 val;

	desc = devm_kzalloc(info->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	info->desc = desc;
	data = info->data;
	memcpy(desc, &algo_desc_defval, sizeof(*desc));

	pehv_parse_dt_u32(np, (void *)desc, pehv_dtprops_u32,
			  ARRAY_SIZE(pehv_dtprops_u32));

	if (desc->swchg_aicr == 0 || desc->swchg_ichg == 0) {
		desc->swchg_aicr = 0;
		desc->swchg_ichg = 0;
	}

	if (of_property_read_u32(np, "vbat_threshold", &val) >= 0)
		data->vbat_threshold = val;
	else if (of_property_read_u32(np, "vbat-threshold", &val) >= 0)
		data->vbat_threshold = val;
	else {
		pr_notice("turn off vbat_threshold checking:%d\n",
			DISABLE_VBAT_THRESHOLD);
		data->vbat_threshold = DISABLE_VBAT_THRESHOLD;
	}

	if (of_property_read_u32(np, "mmi_therm_cur_thres", &val) >= 0)
		data->mmi_therm_cur_thres = val;
	else {
		pr_notice("mmi therm current thres using default:%d\n",
			MMI_THERMAL_CURRENT_THRESHOLD);
		data->mmi_therm_cur_thres = MMI_THERMAL_CURRENT_THRESHOLD;
	}
	if (of_property_read_u32(np, "mmi_therm_vol_thres", &val) >= 0)
		data->mmi_therm_vol_thres = val;
	else {
		pr_notice("mmi therm voltage thres using default:%d\n",
			MMI_THERMAL_VOL_THRESHOLD);
		data->mmi_therm_vol_thres = MMI_THERMAL_VOL_THRESHOLD;
	}
	if (of_property_read_u32(np, "mmi_therm_step", &val) >= 0)
		data->mmi_therm_step = val;
	else {
		pr_notice("mmi therm step using default:%d\n",
			MMI_THERMAL_STEP);
		data->mmi_therm_step = MMI_THERMAL_STEP;
	}
	PEHV_INFO("mmi thermal dts= %d,%d,%d\n",
		data->mmi_therm_cur_thres, data->mmi_therm_vol_thres, data->mmi_therm_step);

	if (of_property_read_u32(np, "mmi_max_ibat", &val) >= 0)
		data->mmi_max_ibat = val;
	else {
		pr_notice("mmi_max_ibat using default:%d\n",
			MMI_MAX_IBAT);
		data->mmi_max_ibat = MMI_MAX_IBAT;
	}

	return 0;
}

static int pehv_probe(struct platform_device *pdev)
{
	int ret;
	struct pehv_algo_info *info;
	struct pehv_algo_data *data;

	dev_info(&pdev->dev, "%s\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	info->data = data;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	ret = pehv_parse_dt(info);
	if (ret < 0) {
		PEHV_ERR("%s parse dt fail(%d)\n", __func__, ret);
		return ret;
	}

	mutex_init(&data->notify_lock);
	mutex_init(&data->lock);
	mutex_init(&data->ext_lock);
	init_waitqueue_head(&data->wq);
	atomic_set(&data->wakeup_thread, 0);
	atomic_set(&data->stop_thread, 0);
	data->state = PEHV_ALGO_STOP;
	alarm_init(&data->timer, ALARM_REALTIME, pehv_algo_timer_cb);
	data->task = kthread_run(pehv_algo_threadfn, info, "pehv_algo_task");
	if (IS_ERR(data->task)) {
		ret = PTR_ERR(data->task);
		PEHV_ERR("%s run task fail(%d)\n", __func__, ret);
		goto err;
	}
	pehv_algo_data_partial_reset(info);
	device_init_wakeup(info->dev, true);

	info->alg = chg_alg_device_register("pehv", info->dev, info, &pehv_ops,
					    NULL);
	if (IS_ERR_OR_NULL(info->alg)) {
		PEHV_ERR("%s reg pehv algo fail(%d)\n", __func__, ret);
		ret = PTR_ERR(info->alg);
		goto err;
	}
	chg_alg_dev_set_drvdata(info->alg, info);

	dev_info(info->dev, "%s successfully\n", __func__);
	return 0;
err:
	mutex_destroy(&data->ext_lock);
	mutex_destroy(&data->lock);
	mutex_destroy(&data->notify_lock);
	chg_alg_device_unregister(info->alg);
	return ret;
}

static int pehv_remove(struct platform_device *pdev)
{
	struct pehv_algo_info *info = platform_get_drvdata(pdev);
	struct pehv_algo_data *data;

	if (info) {
		data = info->data;
		atomic_set(&data->stop_thread, 1);
		pehv_wakeup_algo_thread(data);
		kthread_stop(data->task);
		mutex_destroy(&data->ext_lock);
		mutex_destroy(&data->lock);
		mutex_destroy(&data->notify_lock);
		chg_alg_device_unregister(info->alg);
	}

	return 0;
}

static int __maybe_unused pehv_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pehv_algo_info *info = platform_get_drvdata(pdev);
	struct pehv_algo_data *data = info->data;

	dev_info(dev, "%s\n", __func__);
	mutex_lock(&data->lock);
	return 0;
}

static int __maybe_unused pehv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pehv_algo_info *info = platform_get_drvdata(pdev);
	struct pehv_algo_data *data = info->data;

	dev_info(dev, "%s\n", __func__);
	mutex_unlock(&data->lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(pehv_pm_ops, pehv_suspend, pehv_resume);

static const struct of_device_id mmi_pehv_of_match[] = {
	{ .compatible = "mmi,pehv", },
	{},
};
MODULE_DEVICE_TABLE(of, mmi_pehv_of_match);

static struct platform_driver pehv_platdrv = {
	.probe = pehv_probe,
	.remove = pehv_remove,
	.driver = {
		.name = "pehv",
		.owner = THIS_MODULE,
		.pm = &pehv_pm_ops,
		.of_match_table = mmi_pehv_of_match,
	},
};

static int __init pehv_init(void)
{
	return platform_driver_register(&pehv_platdrv);
}

static void __exit pehv_exit(void)
{
	platform_driver_unregister(&pehv_platdrv);
}
late_initcall(pehv_init);
module_exit(pehv_exit);

MODULE_AUTHOR("MaHaijian <mahj8@motorola.com>");
MODULE_DESCRIPTION("MMI Pump Express HV Algorithm");
MODULE_LICENSE("GPL");
