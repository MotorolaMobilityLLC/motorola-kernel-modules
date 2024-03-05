// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Motorola Inc.
 */

#include <charger_class.h>
#include <mtk_charger.h>
#include "mmi_pehv.h"

enum mmi_chg_type {
	MMI_CHGTYP_SWCHG = 0,
	MMI_CHGTYP_DVCHG,
	MMI_CHGTYP_DVCHG_SLAVE,
	MMI_CHGTYP_MAX,
};

struct mmi_chgdev_desc {
	enum mmi_chg_type type;
	const char *name;
	bool must_exist;
};

static struct mmi_chgdev_desc mmi_chgdev_desc_tbl[MMI_CHGTYP_MAX] = {
	{
		.type = MMI_CHGTYP_SWCHG,
		.name = "primary_chg",
		.must_exist = true,
	},
	{
		.type = MMI_CHGTYP_DVCHG,
		.name = "primary_dvchg",
		.must_exist = true,
	},
	{
		.type = MMI_CHGTYP_DVCHG_SLAVE,
		.name = "secondary_dvchg",
		.must_exist = false,
	},
};

struct pehv_hal {
	struct device *dev;
	struct charger_device *chgdevs[MMI_CHGTYP_MAX];
	struct power_supply *bat_psy;
};

static inline int to_chgtyp(enum chg_idx idx)
{
	switch (idx) {
	case CHG1:
		return MMI_CHGTYP_SWCHG;
	case DVCHG1:
		return MMI_CHGTYP_DVCHG;
	case DVCHG2:
		return MMI_CHGTYP_DVCHG_SLAVE;
	default:
		return -EOPNOTSUPP;
	}
}

static inline int to_chgclass_adc(enum pehv_adc_channel chan)
{
	switch (chan) {
	case PEHV_ADCCHAN_VBUS:
		return ADC_CHANNEL_VBUS;
	case PEHV_ADCCHAN_IBUS:
		return ADC_CHANNEL_IBUS;
	case PEHV_ADCCHAN_VBAT:
		return ADC_CHANNEL_VBAT;
	case PEHV_ADCCHAN_IBAT:
		return ADC_CHANNEL_IBAT;
	case PEHV_ADCCHAN_TBAT:
		return ADC_CHANNEL_TBAT;
	case PEHV_ADCCHAN_TCHG:
		return ADC_CHANNEL_TEMP_JC;
	case PEHV_ADCCHAN_VOUT:
		return ADC_CHANNEL_VOUT;
	case PEHV_ADCCHAN_VSYS:
		return ADC_CHANNEL_VSYS;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

#define VBUS_STEP 20
#define PEHV_PULSE_COUNT_MAX    ((11000 - 5000) / VBUS_STEP)
int pehv_hal_set_ta_vbus(struct chg_alg_device *alg, int target_mV)
{
	int ret = 0;
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);
	int pulse_cnt, curr_vbus_mv, step, i, val;

	ret = charger_dev_get_dp_dm(hal->chgdevs[MMI_CHGTYP_SWCHG], &pulse_cnt);
	if (ret < 0) {
		PEHV_ERR("Couldn't read dpdm pulse count rc=%d\n", ret);
		return ret;
	} else {
		PEHV_INFO("DP DM pulse count = %d\n", pulse_cnt);
	}

	ret = pehv_hal_get_adc(alg, DVCHG1, PEHV_ADCCHAN_VBUS, &curr_vbus_mv);
	if (ret < 0) {
		PEHV_ERR("Unable to read vbus voltage: ret=%d\n", ret);
		return ret;
	}

	if(target_mV < curr_vbus_mv) {
		step = (curr_vbus_mv - target_mV) / VBUS_STEP;
		val = DP_DM_DM_PULSE;
		if (pulse_cnt <= step)
			step = pulse_cnt;
	} else {
		step = (target_mV - curr_vbus_mv) / VBUS_STEP;
		val = DP_DM_DP_PULSE;
		if (step + pulse_cnt > PEHV_PULSE_COUNT_MAX)
			step = PEHV_PULSE_COUNT_MAX - pulse_cnt;
	}
	PEHV_INFO("curr_vbus= %d, target_vbus = %d, step = %d\n",curr_vbus_mv, target_mV, step);
	for (i = 0; i < step; i++) {
		ret = charger_dev_set_dp_dm(hal->chgdevs[MMI_CHGTYP_SWCHG], val);
		if (ret < 0) {
			PEHV_ERR("Couldn't set dpdm pulse ret=%d\n", ret);
			break;
		}

		if (i < step - 1)
			udelay(5000);
	}

	ret = pehv_hal_get_adc(alg, DVCHG1, PEHV_ADCCHAN_VBUS, &curr_vbus_mv);
	if (ret < 0) {
		PEHV_ERR("Unable to read VBUS: %d\n", ret);
		return ret;
	}
	PEHV_INFO("result_vbus= %d\n",curr_vbus_mv);

	return ret;
}

int pehv_hal_init_hardware(struct chg_alg_device *alg)
{
	struct pehv_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pehv_algo_data *data = info->data;
	struct pehv_hal *hal;
	int i, ret;

	hal = devm_kzalloc(info->dev, sizeof(*hal), GFP_KERNEL);
	if (!hal)
		return -ENOMEM;

	/* get charger device */
	for (i = 0; i < MMI_CHGTYP_MAX; i++) {
		hal->chgdevs[i] =
			get_charger_by_name(mmi_chgdev_desc_tbl[i].name);
		if (!hal->chgdevs[i]) {
			PEHV_ERR("get %s fail\n", mmi_chgdev_desc_tbl[i].name);
			if (mmi_chgdev_desc_tbl[i].must_exist) {
				ret = -ENODEV;
				goto err_free_mem1;
			}
		}
	}
	data->is_dvchg_exist[PEHV_DVCHG_MASTER] = true;
	if (hal->chgdevs[MMI_CHGTYP_DVCHG_SLAVE])
		data->is_dvchg_exist[PEHV_DVCHG_SLAVE] = true;
	chg_alg_dev_set_drv_hal_data(alg, hal);
	hal->dev = info->dev;
	hal->bat_psy = devm_power_supply_get_by_phandle(hal->dev, "gauge");
	if (IS_ERR_OR_NULL(hal->bat_psy)) {
		ret = IS_ERR(hal->bat_psy) ? PTR_ERR(hal->bat_psy) : -ENODEV;
		PEHV_ERR("get bat_psy fail(%d)\n", ret);
	}
	PEHV_INFO("successfully\n");
	return 0;

err_free_mem1:
	devm_kfree(info->dev, hal);
	return ret;
}

int pehv_hal_enable_sw_vbusovp(struct chg_alg_device *alg, bool en)
{
	mtk_chg_enable_vbus_ovp(en);
	return 0;
}

int pehv_hal_enable_charging(struct chg_alg_device *alg, enum chg_idx chgidx,
			     bool en)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_enable(hal->chgdevs[chgtyp], en);
}

int pehv_hal_enable_hz(struct chg_alg_device *alg, enum chg_idx chgidx, bool en)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_enable_hz(hal->chgdevs[chgtyp], en);
}

int pehv_hal_set_vbusovp(struct chg_alg_device *alg, enum chg_idx chgidx,
			 u32 mV)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_vbusovp(hal->chgdevs[chgtyp],
				       milli_to_micro(mV));
}

int pehv_hal_set_ibusocp(struct chg_alg_device *alg, enum chg_idx chgidx,
			 u32 mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_ibusocp(hal->chgdevs[chgtyp],
				       milli_to_micro(mA));
}

int pehv_hal_set_vbatovp(struct chg_alg_device *alg, enum chg_idx chgidx,
			 u32 mV)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_vbatovp(hal->chgdevs[chgtyp],
				       milli_to_micro(mV));
}

int pehv_hal_set_ibatocp(struct chg_alg_device *alg, enum chg_idx chgidx,
			 u32 mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_ibatocp(hal->chgdevs[chgtyp],
				       milli_to_micro(mA));
}

int pehv_hal_set_vbatovp_alarm(struct chg_alg_device *alg, enum chg_idx chgidx,
			       u32 mV)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_vbatovp_alarm(hal->chgdevs[chgtyp],
					     milli_to_micro(mV));
}

int pehv_hal_reset_vbatovp_alarm(struct chg_alg_device *alg,
				 enum chg_idx chgidx)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_reset_vbatovp_alarm(hal->chgdevs[chgtyp]);
}

int pehv_hal_set_vbusovp_alarm(struct chg_alg_device *alg, enum chg_idx chgidx,
			       u32 mV)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_vbusovp_alarm(hal->chgdevs[chgtyp],
					     milli_to_micro(mV));
}

int pehv_hal_reset_vbusovp_alarm(struct chg_alg_device *alg,
				 enum chg_idx chgidx)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_reset_vbusovp_alarm(hal->chgdevs[chgtyp]);
}

static int pehv_get_tbat(struct pehv_hal *hal)
{
	int ret = 27;
	union power_supply_propval val = {0,};

	if (IS_ERR_OR_NULL(hal->bat_psy))
		goto out;

	ret = power_supply_get_property(hal->bat_psy, POWER_SUPPLY_PROP_TEMP,
					&val);
	if (ret < 0) {
		PEHV_ERR("get tbat fail(%d)\n", ret);
		ret = 27;
		goto out;
	}
	ret = val.intval / 10;
out:
	PEHV_DBG("%d\n", ret);
	return ret;
}

static int pehv_get_ibat(struct pehv_hal *hal)
{
	int ret = 0;
	union power_supply_propval val = {0,};

	if (IS_ERR_OR_NULL(hal->bat_psy))
		goto out;

	ret = power_supply_get_property(hal->bat_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (ret < 0) {
		PEHV_ERR("get ibat fail(%d)\n", ret);
		ret = 0;
		goto out;
	}
	ret = val.intval / 1000;
out:
	PEHV_DBG("%d\n", ret);
	return ret;
}

static int pehv_get_vbat(struct pehv_hal *hal)
{
	int ret = 0;
	union power_supply_propval val = {0,};

	if (IS_ERR_OR_NULL(hal->bat_psy))
		goto out;

	ret = power_supply_get_property(hal->bat_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		PEHV_ERR("get vbat fail(%d)\n", ret);
		ret = 0;
		goto out;
	}
	ret = val.intval / 1000;
out:
	PEHV_DBG("%d\n", ret);
	return ret;
}

int pehv_hal_get_adc(struct chg_alg_device *alg, enum chg_idx chgidx,
		     enum pehv_adc_channel chan, int *val)
{
	int ret;
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);
	int _chan = to_chgclass_adc(chan);

	if (chgtyp < 0)
		return chgtyp;
	if (_chan < 0)
		return _chan;

	if (_chan == ADC_CHANNEL_TBAT) {
		*val = pehv_get_tbat(hal);
		return 0;
	} else if (_chan == ADC_CHANNEL_IBAT) {
		*val = pehv_get_ibat(hal);
		return 0;
	} else if (_chan == ADC_CHANNEL_VBAT) {
		*val = pehv_get_vbat(hal);
		return 0;
	}

	ret = charger_dev_get_adc(hal->chgdevs[chgtyp], _chan, val, val);
	if (ret < 0)
		return ret;
	if (_chan == ADC_CHANNEL_VBAT || _chan == ADC_CHANNEL_IBAT ||
	    _chan == ADC_CHANNEL_VBUS || _chan == ADC_CHANNEL_IBUS ||
	    _chan == ADC_CHANNEL_VOUT || _chan == ADC_CHANNEL_VSYS)
		*val = micro_to_milli(*val);
	return 0;
}

int pehv_hal_get_soc(struct chg_alg_device *alg, u32 *soc)
{
	int ret = -EOPNOTSUPP;
	union power_supply_propval val = {0,};
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (IS_ERR_OR_NULL(hal->bat_psy)) {
	    hal->bat_psy = devm_power_supply_get_by_phandle(hal->dev, "gauge");
	}

	if (IS_ERR_OR_NULL(hal->bat_psy))
		goto out;

	ret = power_supply_get_property(hal->bat_psy,
					POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret < 0) {
		PEHV_ERR("get soc fail(%d)\n", ret);
		goto out;
	}
	ret = *soc = val.intval;
out:
	PEHV_DBG("%d\n", ret);
	return ret;
}

int pehv_hal_is_hv_adapter_ready(struct chg_alg_device *alg)
{
	struct pehv_hal *hal;
	int type = 0;

	if (alg == NULL) {
		pr_notice("%s: alg is null\n", __func__);
		return -EINVAL;
	}

	hal = chg_alg_dev_get_drv_hal_data(alg);

	charger_dev_get_protocol(hal->chgdevs[MMI_CHGTYP_SWCHG], &type);
	pr_notice("%s type:%d\n", __func__, type);

	if (type == USB_TYPE_QC3P_18 || type == USB_TYPE_QC3P_27)
		return ALG_READY;
	else if (type == USB_TYPE_UNKNOWN)
		return ALG_TA_CHECKING;
	return ALG_TA_NOT_SUPPORT;
}

int pehv_hal_set_ichg(struct chg_alg_device *alg, enum chg_idx chgidx, u32 mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_charging_current(hal->chgdevs[chgtyp],
						milli_to_micro(mA));
}

int pehv_hal_set_aicr(struct chg_alg_device *alg, enum chg_idx chgidx, u32 mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_set_input_current(hal->chgdevs[chgtyp],
					     milli_to_micro(mA));
}

int pehv_hal_get_ichg(struct chg_alg_device *alg, enum chg_idx chgidx, u32 *mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);
	int ret, uA = 0;

	if (chgtyp < 0)
		return chgtyp;

	ret = charger_dev_get_charging_current(hal->chgdevs[chgtyp], &uA);
	*mA = micro_to_milli(uA);

	return ret;
}

int pehv_hal_get_aicr(struct chg_alg_device *alg, enum chg_idx chgidx, u32 *mA)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);
	int ret, uA = 0;

	if (chgtyp < 0)
		return chgtyp;

	ret = charger_dev_get_input_current(hal->chgdevs[chgtyp], &uA);
	*mA = micro_to_milli(uA);

	return ret;
}

int pehv_hal_is_vbuslowerr(struct chg_alg_device *alg, enum chg_idx chgidx,
			   bool *err)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_is_vbuslowerr(hal->chgdevs[chgtyp], err);
}

int pehv_hal_is_vbushigherr(struct chg_alg_device *alg, enum chg_idx chgidx,
			   bool *err)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_is_vbushigherr(hal->chgdevs[chgtyp], err);
}

int pehv_hal_get_adc_accuracy(struct chg_alg_device *alg, enum chg_idx chgidx,
			      enum pehv_adc_channel chan, int *val)
{
	int ret;
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);
	int _chan = to_chgclass_adc(chan);

	if (chgtyp < 0)
		return chgtyp;
	if (_chan < 0)
		return _chan;

	ret = charger_dev_get_adc_accuracy(hal->chgdevs[chgtyp], _chan, val,
					   val);
	if (ret < 0)
		return ret;
	if (_chan == ADC_CHANNEL_VBAT || _chan == ADC_CHANNEL_IBAT ||
	    _chan == ADC_CHANNEL_VBUS || _chan == ADC_CHANNEL_IBUS ||
	    _chan == ADC_CHANNEL_VOUT)
		*val = micro_to_milli(*val);
	return 0;
}

int pehv_hal_init_chip(struct chg_alg_device *alg, enum chg_idx chgidx)
{
	int chgtyp = to_chgtyp(chgidx);
	struct pehv_hal *hal = chg_alg_dev_get_drv_hal_data(alg);

	if (chgtyp < 0)
		return chgtyp;
	return charger_dev_init_chip(hal->chgdevs[chgtyp]);
}
