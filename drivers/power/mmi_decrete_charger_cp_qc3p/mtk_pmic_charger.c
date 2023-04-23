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

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include "mmi_charger_class.h"
#include <linux/power_supply.h>
#include "mmi_charger_core.h"

static int mtk_pmic_enable_charging(struct mmi_charger_device *chrg, bool en)
{
	int rc;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI SW chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_enable(chrg->chg_dev, en);
	if (rc<0) {
		chrg_dev_info(chrg, "MMI chrg: sw charger_enabled failed, set to false\n");
		//chrg->charger_enabled  = false;
	} else {
		chrg->charger_enabled = !!en;
		chrg_dev_info(chrg, "MMI chrg: set sw charger_enabled = %d\n",chrg->charger_enabled);
	}

	return rc;
}

static int mtk_pmic_is_charging_enabled(struct mmi_charger_device *chrg, bool *en)
{
	int rc;
	bool val;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_is_enabled(chrg->chg_dev, &val);
	if (rc>=0)
		chrg->charger_enabled = !!val;

	*en = chrg->charger_enabled;

	return rc;
}

static int mtk_pmic_get_input_voltage_settled(struct mmi_charger_device *chrg, u32 *vbus)
{
	int rc, vbus_voltage;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBUS, &vbus_voltage, &vbus_voltage);
	if (rc>=0)
		chrg->charger_data.vbus_volt = vbus_voltage;

	*vbus = chrg->charger_data.vbus_volt;

	return rc;
}

static int mtk_pmic_get_input_current(struct mmi_charger_device *chrg, u32 *uA)
{
	int rc, ibus;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBUS, &ibus, &ibus);
	if (rc>=0)
		chrg->charger_data.ibus_curr = ibus;

	*uA = chrg->charger_data.ibus_curr;

	return rc;
}

static int mtk_pmic_get_charging_current(struct mmi_charger_device *chrg, u32 *uA)
{
	int rc, ibat;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBAT, &ibat, &ibat);
	if (rc>=0)
		chrg->charger_data.ibatt_curr = ibat;

	*uA = chrg->charger_data.ibatt_curr;

	return rc;
}

static int mtk_pmic_set_charging_current(struct mmi_charger_device *chrg, u32 uA)
{
	int rc;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_set_charging_current(chrg->chg_dev, uA);
	if (rc<0)
		chrg_dev_info(chrg, "MMI CP chrg: pmic set charging current fail \n");

	return rc;
}

static int mtk_pmic_set_charging_current_limit(struct mmi_charger_device *chrg, bool en, u32 uA)
{
	int rc;
	union power_supply_propval prop = {0,};
	struct power_supply	*chg_psy;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;	}

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (!chg_psy)
		return -ENODEV;

	if (en)
		prop.intval = uA;
	else
		prop.intval = -1;

	rc = charger_dev_set_charging_current(chrg->chg_dev, uA);
	if (rc<0)
		chrg_dev_info(chrg, "MMI CP chrg: pmic set charging current fail \n");

	rc = power_supply_set_property(chg_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &prop);

	return rc;
}

static int mtk_pmic_update_charger_status(struct mmi_charger_device *chrg)
{
	int rc,vbat,vbus,ibus,ibat;
	bool enable;
	union power_supply_propval prop = {0,};

	chrg_dev_info(chrg, "mtk_pmic_update_charger_status enter");

	if (!chrg->chrg_psy)
		return -ENODEV;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	chrg_dev_info(chrg, "mtk_pmic_update_charger_status enter 1");

	// vbat
	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBAT, &vbat, &vbat);
	if (rc>=0)
		chrg->charger_data.vbatt_volt = vbat;

	// ibat
	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBAT, &ibat, &ibat);
	if (rc>=0)
		chrg->charger_data.ibatt_curr = ibat; //*-1

	// vbus
	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBUS, &vbus, &vbus);
	if (rc>=0)
		chrg->charger_data.vbus_volt = vbus; //*-1

	// ibus
	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBUS, &ibus, &ibus);
	if (rc>=0)
		chrg->charger_data.ibus_curr = ibus; //*-1

	chrg->charger_data.batt_temp = 30;  //Force return 30 temp from main charger.

	// vbus online state
	rc = power_supply_get_property(chrg->chrg_psy,
						POWER_SUPPLY_PROP_ONLINE, &prop);
	if (!rc)
		chrg->charger_data.vbus_pres = !!prop.intval;

	// is charging enable
	rc = charger_dev_is_enabled(chrg->chg_dev, &enable);
	if (rc>=0)
		chrg->charger_enabled = !!enable;

	chrg_dev_info(chrg, "mtk SW chrg: status update: --- info---1");
	chrg_dev_info(chrg, "vbatt %d\n", chrg->charger_data.vbatt_volt);
	chrg_dev_info(chrg, "ibatt %d\n", chrg->charger_data.ibatt_curr);
	chrg_dev_info(chrg, "batt temp %d\n", chrg->charger_data.batt_temp);
	chrg_dev_info(chrg, "vbus %d\n", chrg->charger_data.vbus_volt);
	chrg_dev_info(chrg, "ibus %d\n", chrg->charger_data.ibus_curr);
	chrg_dev_info(chrg, "vbus pres %d\n", chrg->charger_data.vbus_pres);
	chrg_dev_info(chrg, "charger_enabled %d\n", chrg->charger_enabled);
	chrg_dev_info(chrg, "charger_limited %d\n", chrg->charger_limited);
	return rc;
}

struct mmi_charger_ops mtk_pmic_charger_ops = {
	.enable = mtk_pmic_enable_charging,
	.is_enabled = mtk_pmic_is_charging_enabled,
	.get_input_voltage_settled = mtk_pmic_get_input_voltage_settled,
	.get_input_current = mtk_pmic_get_input_current,
	.get_charging_current = mtk_pmic_get_charging_current,
	.set_charging_current = mtk_pmic_set_charging_current,
	.set_charging_current_limit = mtk_pmic_set_charging_current_limit,
	.update_charger_status = mtk_pmic_update_charger_status,
};
