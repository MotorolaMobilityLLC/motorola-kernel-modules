/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 MOTOROLA Inc.
 */

#ifndef __MMI_PEHV_H
#define __MMI_PEHV_H

#include <mtk_charger_algorithm_class.h>

#define PEHV_ERR_LEVEL	1
#define PEHV_INFO_LEVEL	2
#define PEHV_DBG_LEVEL	3
#define PEHV_ITA_GAP_WINDOW_SIZE	50
#define PRECISION_ENHANCE	5

#define DISABLE_VBAT_THRESHOLD -1

#define MMI_THERMAL_CURRENT_THRESHOLD	500
#define MMI_THERMAL_VOL_THRESHOLD	80
#define MMI_THERMAL_STEP	3
#define MMI_MAX_IBAT	6000
#define MMI_MAX_HRST_CNT 100

extern int pehv_get_log_level(void);
#define PEHV_DBG(fmt, ...) \
	do { \
		if (pehv_get_log_level() >= PEHV_DBG_LEVEL) \
			pr_info("[PEHV]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define PEHV_INFO(fmt, ...) \
	do { \
		if (pehv_get_log_level() >= PEHV_INFO_LEVEL) \
			pr_info("[PEHV]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define PEHV_ERR(fmt, ...) \
	do { \
		if (pehv_get_log_level() >= PEHV_ERR_LEVEL) \
			pr_info("[PEHV]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

enum pehv_adc_channel {
	PEHV_ADCCHAN_VBUS = 0,
	PEHV_ADCCHAN_IBUS,
	PEHV_ADCCHAN_VBAT,
	PEHV_ADCCHAN_IBAT,
	PEHV_ADCCHAN_TBAT,
	PEHV_ADCCHAN_TCHG,
	PEHV_ADCCHAN_VOUT,
	PEHV_ADCCHAN_VSYS,
	PEHV_ADCCHAN_MAX,
};

enum pehv_algo_state {
	PEHV_ALGO_INIT = 0,
	PEHV_ALGO_MEASURE_R,
	PEHV_ALGO_SS_SWCHG,
	PEHV_ALGO_SS_DVCHG,
	PEHV_ALGO_CC_CV,
	PEHV_ALGO_STOP,
	PEHV_ALGO_STATE_MAX,
};

enum pehv_thermal_level {
	PEHV_THERMAL_VERY_COLD = 0,
	PEHV_THERMAL_COLD,
	PEHV_THERMAL_VERY_COOL,
	PEHV_THERMAL_COOL,
	PEHV_THERMAL_NORMAL,
	PEHV_THERMAL_WARM,
	PEHV_THERMAL_VERY_WARM,
	PEHV_THERMAL_HOT,
	PEHV_THERMAL_VERY_HOT,
	PEHV_THERMAL_MAX,
};

enum pehv_dvchg_role {
	PEHV_DVCHG_MASTER = 0,
	PEHV_DVCHG_SLAVE,
	PEHV_DVCHG_MAX,
};

struct pehv_ta_status {
	int temperature;
	bool ocp;
	bool otp;
	bool ovp;
};

struct pehv_algo_data {
	bool is_dvchg_exist[PEHV_DVCHG_MAX];

	/* Thread & Timer */
	struct alarm timer;
	struct task_struct *task;
	struct mutex lock;
	struct mutex ext_lock;
	wait_queue_head_t wq;
	atomic_t wakeup_thread;
	atomic_t stop_thread;
	atomic_t stop_algo;

	/* Notify */
	struct mutex notify_lock;
	u32 notify;

	/* Algorithm */
	bool inited;
	bool ta_ready;
	bool run_once;
	bool is_swchg_en;
	bool is_dvchg_en[PEHV_DVCHG_MAX];
	bool ignore_ibusucpf;
	bool force_ta_cv;
	bool tried_dual_dvchg;
	bool suspect_ta_cc;
	u32 vta_setting;
	u32 ita_setting;
	u32 vta_measure;
	u32 ita_measure;
	u32 ita_gap_per_vstep;
	u32 ita_gap_window_idx;
	u32 ita_gaps[PEHV_ITA_GAP_WINDOW_SIZE];
	u32 ichg_setting;
	u32 aicr_setting;
	u32 aicr_lmt;
	u32 aicr_init_lmt;
	u32 idvchg_cc;
	u32 idvchg_ss_init;
	u32 idvchg_term;
	int vbus_cali;
	u32 r_sw;
	u32 r_cable;
	u32 r_cable_by_swchg;
	u32 r_bat;
	u32 r_total;
	u32 ita_lmt;
	u32 ita_pwr_lmt;
	u32 cv_lower_bound;
	u32 err_retry_cnt;
	u32 zcv;
	u32 vbat_cv_no_ircmp;
	u32 vbat_cv;
	u32 vbat_ircmp;
	int vta_comp;
	int vbat_threshold; /* For checking Ready */
	int ref_vbat; /* Vbat with cable in */
	bool is_vbat_over_cv;
	ktime_t stime;
	enum pehv_algo_state state;
	int input_current_limit;
	int cv_limit;
	u32 start_soc_max;		/* algo start soc upper bound */
	int mmi_fcc_limit;
	int mmi_therm_fcc_limit;
	int mmi_therm_cur_thres;
	int mmi_therm_vol_thres;
	int mmi_therm_step;
	int mmi_max_ibat;
};

/* Setting from dtsi */
struct pehv_algo_desc {
	u32 polling_interval;		/* polling interval */
	u32 vbat_cv;			/* vbat constant voltage */
	u32 start_soc_min;		/* algo start soc low bound */
	u32 start_soc_max;		/* algo start soc upper bound */
	u32 stop_soc_max;		/* algo stop soc upper bound */
	u32 vbat_max_gap;		/* algo vbat upper bound for eoc */
	u32 idvchg_cc;		/* max input current */
	u32 idvchg_term;		/* terminated current */
	u32 idvchg_step;		/* input current step */

	u32 idvchg_ss_init;		/* SS state init input current */
	u32 idvchg_ss_step;		/* SS state input current step */
	u32 idvchg_ss_step1;		/* SS state input current step2 */
	u32 idvchg_ss_step2;		/* SS state input current step3 */
	u32 idvchg_ss_step1_vbat;	/* vbat threshold for ic_ss_step2 */
	u32 idvchg_ss_step2_vbat;	/* vbat threshold for ic_ss_step3 */
	u32 ta_blanking;		/* wait TA stable */
	u32 swchg_aicr;			/* CC state swchg input current */
	u32 swchg_ichg;			/* CC state swchg charging current */
	u32 swchg_aicr_ss_init;		/* SWCHG_SS state init input current */
	u32 swchg_aicr_ss_step;		/* SWCHG_SS state input current step */
	u32 swchg_off_vbat;		/* VBAT to turn off SWCHG */
	u32 force_ta_cv_vbat;		/* Force TA using CV mode */
	u32 chg_time_max;		/* max charging time */
	u32 ifod_threshold;		/* FOD current threshold */
	u32 rsw_min;			/* min rsw */
	u32 ircmp_rbat;			/* IR compensation's rbat */
	u32 ircmp_vclamp;		/* IR compensation's vclamp */
	u32 vta_cap_min;		/* min ta voltage capability */
	u32 vta_cap_max;		/* max ta voltage capability */
	u32 vta_step;		/* input voltage step */
	u32 ita_cap_min;		/* min ta current capability */
	const char **support_ta;	/* supported ta name */
	u32 support_ta_cnt;		/* supported ta count */
};

struct pehv_algo_info {
	struct device *dev;
	struct chg_alg_device *alg;
	struct pehv_algo_desc *desc;
	struct pehv_algo_data *data;
};

static inline u32 precise_div(u64 dividend, u64 divisor)
{
	u64 _val = div64_u64(dividend << PRECISION_ENHANCE, divisor);

	return (u32)((_val + (1 << (PRECISION_ENHANCE - 1))) >>
		PRECISION_ENHANCE);
}

static inline u32 percent(u32 val, u32 percent)
{
	return precise_div((u64)val * percent, 100);
}

static inline u32 div1000(u32 val)
{
	return precise_div(val, 1000);
}

static inline u32 milli_to_micro(u32 val)
{
	return val * 1000;
}

static inline int micro_to_milli(int val)
{
	return (val < 0) ? -1 : div1000(val);
}

extern int pehv_hal_set_ta_vbus(struct chg_alg_device *alg, int target_mV);

extern int pehv_hal_init_hardware(struct chg_alg_device *alg);
extern int pehv_hal_enable_sw_vbusovp(struct chg_alg_device *alg, bool en);
extern int pehv_hal_enable_charging(struct chg_alg_device *alg,
				    enum chg_idx chgidx, bool en);
extern int pehv_hal_enable_chip(struct chg_alg_device *alg, enum chg_idx chgidx,
				bool en);
extern int pehv_hal_enable_hz(struct chg_alg_device *alg, enum chg_idx chgidx,
			      bool en);
extern int pehv_hal_set_vbusovp(struct chg_alg_device *alg, enum chg_idx chgidx,
				u32 mV);
extern int pehv_hal_set_ibusocp(struct chg_alg_device *alg, enum chg_idx chgidx,
				u32 mA);
extern int pehv_hal_set_vbatovp(struct chg_alg_device *alg, enum chg_idx chgidx,
				u32 mV);
extern int pehv_hal_set_ibatocp(struct chg_alg_device *alg, enum chg_idx chgidx,
				u32 mA);
extern int pehv_hal_set_vbatovp_alarm(struct chg_alg_device *alg,
				      enum chg_idx chgidx, u32 mV);
extern int pehv_hal_reset_vbatovp_alarm(struct chg_alg_device *alg,
					enum chg_idx chgidx);
extern int pehv_hal_set_vbusovp_alarm(struct chg_alg_device *alg,
				      enum chg_idx chgidx, u32 mV);
extern int pehv_hal_reset_vbusovp_alarm(struct chg_alg_device *alg,
					enum chg_idx chgidx);
extern int pehv_hal_get_adc(struct chg_alg_device *alg, enum chg_idx chgidx,
			    enum pehv_adc_channel chan, int *val);
extern int pehv_hal_get_soc(struct chg_alg_device *alg, u32 *soc);
extern int pehv_hal_is_hv_adapter_ready(struct chg_alg_device *alg);
extern int pehv_hal_set_ichg(struct chg_alg_device *alg, enum chg_idx chgidx,
			     u32 mA);
extern int pehv_hal_set_aicr(struct chg_alg_device *alg, enum chg_idx chgidx,
			     u32 mA);
extern int pehv_hal_get_ichg(struct chg_alg_device *alg, enum chg_idx chgidx,
			     u32 *mA);
extern int pehv_hal_get_aicr(struct chg_alg_device *alg, enum chg_idx chgidx,
			     u32 *mA);
extern int pehv_hal_is_vbuslowerr(struct chg_alg_device *alg,
				  enum chg_idx chgidx, bool *err);
extern int pehv_hal_is_vbushigherr(struct chg_alg_device *alg,
				  enum chg_idx chgidx, bool *err);
extern int pehv_hal_get_adc_accuracy(struct chg_alg_device *alg,
				     enum chg_idx chgidx,
				     enum pehv_adc_channel chan, int *val);
extern int pehv_hal_init_chip(struct chg_alg_device *alg, enum chg_idx chgidx);
#endif /* __MMI_PEHV_H */
