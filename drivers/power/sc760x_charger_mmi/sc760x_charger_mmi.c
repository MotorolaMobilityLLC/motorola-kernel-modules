/*
 * Copyright (c) 2022 Motorola Mobility, LLC.
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
#define pr_fmt(fmt)     "sc760x-charger: %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regmap.h>

#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/time64.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#include "sc760x_charger_mmi.h"


__maybe_unused static const char* sc760x_irq_name[] = {
    [SC760X_MASTER] = "sc760x_master_irq",
    [SC760X_SLAVE] = "sc760x_slave_irq",
};

static int sc760x_role_data[] = {
    [SC760X_MASTER] = SC760X_MASTER,
    [SC760X_SLAVE] = SC760X_SLAVE,
};

static struct sc760x_chg_platform_data default_cfg = {
    .bat_chg_lim_disable = 0,
    .bat_chg_lim = 39,
    .pow_lim_disable = 0,
    .ilim_disable = 0,
    .load_switch_disable = 0,
    .lp_mode_enable = 0,
    .itrichg = 3,
    .iprechg = 2,
    .vfc_chg = 2,
    .chg_ovp_disable = 0,
    .chg_ovp = 0,
    .bat_ovp_disable = 0,
    .bat_ovp = 10,
    .chg_ocp_disable = 0,
    .chg_ocp = 2,
    .dsg_ocp_disable = 0,
    .dsg_ocp = 2,
    .tdie_flt_disable = 0,
    .tdie_alm_disable = 0,
    .tdie_alm = 9,
};

static const int sc760x_adc_m[] =
    {3125, 125, 125, 3125, 5};

static const int sc760x_adc_l[] =
    {1000, 100, 100, 10000, 10};

//REGISTER
static const struct reg_field sc760x_reg_fields[] = {
    /*reg00*/
    [DEVICE_REV] = REG_FIELD(0x00, 4, 7),
    [DEVICE_ID] = REG_FIELD(0x00, 0, 3),
    /*reg01*/
    [IBAT_CHG_LIM] = REG_FIELD(0x01, 0, 7),
    /*reg02*/
    [POW_LIM_DIS] = REG_FIELD(0x02, 7, 7),
    [VBALANCE_DIS] = REG_FIELD(0x02, 5, 5),
    [POW_LIM] = REG_FIELD(0x02, 0, 3),
    /*reg03*/
    [ILIM_DIS] = REG_FIELD(0x03, 7, 7),
    [IBAT_CHG_LIM_DIS] = REG_FIELD(0x03, 6, 6),
    [BAT_DET_DIS] = REG_FIELD(0x03, 5, 5),
    [VDIFF_CHECK_DIS] = REG_FIELD(0x03, 4, 4),
    [LS_OFF] = REG_FIELD(0x03, 3, 3),
    [SHIP_EN] = REG_FIELD(0x03, 0, 2),
    /*reg04*/
    [REG_RST] = REG_FIELD(0x04, 7, 7),
    [EN_LOWPOWER] = REG_FIELD(0x04, 6, 6),
    [VDIFF_OPEN_TH] = REG_FIELD(0x04, 4, 5),
    [AUTO_BSM_DIS] = REG_FIELD(0x04, 3, 3),
    [AUTO_BSM_TH] = REG_FIELD(0x04, 2, 2),
    [SHIP_WT] = REG_FIELD(0x04, 0, 0),
    /*reg05*/
    [ITRICHG] = REG_FIELD(0x05, 5, 7),
    [VPRE_CHG] = REG_FIELD(0x05, 0, 2),
    /*reg06*/
    [IPRECHG] = REG_FIELD(0x06, 4, 7),
    [VFC_CHG] = REG_FIELD(0x06, 0, 3),
    /*reg07*/
    [CHG_OVP_DIS] = REG_FIELD(0x07, 7, 7),
    [CHG_OVP] = REG_FIELD(0x07, 6, 6),
    /*reg08*/
    [BAT_OVP_DIS] = REG_FIELD(0x08, 7, 7),
    [BAT_OVP] = REG_FIELD(0x08, 2, 6),
    /*reg09*/
    [CHG_OCP_DIS] = REG_FIELD(0x09, 7, 7),
    [CHG_OCP] = REG_FIELD(0x09, 4, 6),
    /*reg0A*/
    [DSG_OCP_DIS] = REG_FIELD(0x0A, 7, 7),
    [DSG_OCP] = REG_FIELD(0x0A, 4, 6),
    /*reg0B*/
    [TDIE_FLT_DIS] = REG_FIELD(0x0B, 7, 7),
    [TDIE_FLT] = REG_FIELD(0x0B, 0, 3),
    /*reg0C*/
    [TDIE_ALRM_DIS] = REG_FIELD(0x0C, 7, 7),
    [TDIE_ALRM] = REG_FIELD(0x0C, 0, 3),
    /*reg0D*/
    [CHG_OVP_DEG] = REG_FIELD(0x0D, 6, 7),
    [BAT_OVP_DEG] = REG_FIELD(0x0D, 4, 5),
    [CHG_OCP_DEG] = REG_FIELD(0x0D, 2, 3),
    [DSG_OCP_DEG] = REG_FIELD(0x0D, 0, 1),
    /*reg0E*/
    [AUTO_BSM_DEG] = REG_FIELD(0x0E, 6, 7),
    /*reg0F*/
    [WORK_MODE] = REG_FIELD(0x0F, 5, 7),
    /*reg15*/
    [ADC_EN] = REG_FIELD(0x15, 7, 7),
    [ADC_RATE] = REG_FIELD(0x15, 6, 6),
    [ADC_FREEZE] = REG_FIELD(0x15, 5, 5),
    [ADC_DONE_MASK] = REG_FIELD(0x15, 0, 0),
};

static const struct regmap_config sc760x_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
};

#ifndef MIN_VAL
   #define  MIN_VAL( x, y ) ( ((x) < (y)) ? (x) : (y) )
#endif

__attribute__((unused)) static int sc760x_set_ibat_dis(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_load_switch(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_lowpower_mode(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_itrickle(struct sc760x_chip *sc, int curr);
__attribute__((unused)) static int sc760x_set_iprechg(struct sc760x_chip *sc, int curr);
__attribute__((unused)) static int sc760x_set_vfcchg(struct sc760x_chip *sc, int volt);
__attribute__((unused)) static int sc760x_set_batovp(struct sc760x_chip *sc, int volt);
__attribute__((unused)) static int sc760x_set_adc_enable(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_power_limit_dis(struct sc760x_chip *sc, bool en);

static int sc760x_get_state(struct sc760x_chip *sc,
			     struct sc760x_state *state);
static int sc760x_init_device(struct sc760x_chip *sc);
static struct power_supply_desc sc760x_power_supply_desc;

/*********************************************************/

static int sc760x_enable_chip(struct sc760x_chip *sc, bool en)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;
	const char *pinctrl_name;
	int ret;

	pinctrl = pinctrl_get(sc->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(sc->dev, "Failed to get pinctrl\n");
		return -EINVAL;
	}

	if (en)
		pinctrl_name = "sc760x_enable";
	else
		pinctrl_name = "sc760x_disable";


	state = pinctrl_lookup_state(pinctrl, pinctrl_name);
	if (IS_ERR_OR_NULL(state)) {
		dev_err(sc->dev, "fail to get %s state\n", pinctrl_name);
		ret = -ENODEV;
		goto put_pinctrl;
	}

	ret = pinctrl_select_state(pinctrl, state);
	if (ret) {
		dev_err(sc->dev, "fail to set %s state\n", pinctrl_name);
		ret = -EINVAL;
		goto put_pinctrl;
	}

	if (en)
	    msleep(2000);
	sc->sc760x_enable = en;

	if (sc->sc760x_enable) {

		if (!sc->irq_enabled) {
			//enable_irq_wake(sc->irq);
			//enable_irq(sc->irq);
			sc->irq_enabled = true;
		}
		ret = sc760x_init_device(sc);
		if (ret < 0) {
		    pr_info("init device failed(%d)\n", ret);
		}
	} else {
		if (sc->irq_enabled) {
			//disable_irq_wake(sc->irq);
			//disable_irq(sc->irq);
			sc->irq_enabled = false;
		}
	}
	dev_err(sc->dev, "success to set %s state, sleep 2s\n", pinctrl_name);

	ret = 0;
put_pinctrl:
	pinctrl_put(pinctrl);

	return ret;
}

static int sc760x_field_read(struct sc760x_chip *sc,
                enum sc760x_fields field_id, int *val)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc760x_field_write(struct sc760x_chip *sc,
                enum sc760x_fields field_id, int val)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc760x_read_block(struct sc760x_chip *sc,
                int reg, uint8_t *val, int len)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read %02x block failed %d\n", reg, ret);
    }

    return ret;
}

/*******************************************************/
static int sc760x_reg_reset(struct sc760x_chip *sc)
{
    return sc760x_field_write(sc, REG_RST, 1);
}

static int sc760x_set_auto_bsm_dis(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, AUTO_BSM_DIS, !!en);
}

static int sc760x_set_ibat_limit(struct sc760x_chip *sc, int curr)
{
    int user_val = INT_MAX;

    if (sc->user_ichg >= 0) {
        user_val = sc->user_ichg;
    }

    if (sc->user_ilim >= 0) {
        user_val = MIN_VAL(user_val, sc->user_ilim);
    }

    if (user_val != INT_MAX) {
        curr = user_val/1000;
        dev_info(sc->dev, "%s : User request overide ibat = %dmA\n", __func__, curr);
    }

    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, IBAT_CHG_LIM,
            (curr - IBAT_CHG_LIM_BASE) / IBAT_CHG_LIM_LSB);
}

static int sc760x_set_ibat_dis(struct sc760x_chip *sc, bool en)
{
    dev_info(sc->dev, "%s : ibat_chrg %d\n", __func__, en);
    return sc760x_field_write(sc, IBAT_CHG_LIM_DIS, !!en);
}

static int sc760x_get_ibat_limit(struct sc760x_chip *sc, int *curr)
{
    int data = 0, ret = 0;
    ret = sc760x_field_read(sc, IBAT_CHG_LIM, &data);
    *curr = data *50 + 50;
    return ret;
}

static int sc760x_set_power_limit_dis(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, POW_LIM_DIS, !!en);
}

static int sc760x_set_load_switch(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, LS_OFF, !!en);
}

static int sc760x_get_load_switch(struct sc760x_chip *sc, int *ls_en)
{
    int ret = 0;
    ret = sc760x_field_read(sc, LS_OFF, ls_en);
    return ret;
}

static int sc760x_set_lowpower_mode(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, EN_LOWPOWER, !!en);
}

static int sc760x_set_itrickle(struct sc760x_chip *sc, int curr)
{
    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, ITRICHG,
            (curr * 1000 - ITRICHG_BASE) / ITRICHG_LSB);
}

static int sc760x_set_iprechg(struct sc760x_chip *sc, int curr)
{
    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, IPRECHG,
            (curr - IPRECHG_BASE) / IPRECHG_LSB);
}

static int sc760x_set_vfcchg(struct sc760x_chip *sc, int volt)
{
    dev_info(sc->dev, "%s : %dmV\n", __func__, volt);

    return sc760x_field_write(sc, VFC_CHG,
            (volt - VFC_CHG_BASE) / VFC_CHG_LSB);
}

static int sc760x_set_batovp(struct sc760x_chip *sc, int volt)
{
    dev_info(sc->dev, "%s : %dmV\n", __func__, volt);

    return sc760x_field_write(sc, BAT_OVP,
            (volt - BAT_OVP_BASE) / BAT_OVP_LSB);
}

static int sc760x_get_work_mode(struct sc760x_chip *sc, int *mode)
{
    return sc760x_field_read(sc, WORK_MODE, mode);
}

static int sc760x_set_adc_enable(struct sc760x_chip *sc, bool en)
{
    dev_err(sc->dev, "%s set adc %d\n", __func__, en);
    return sc760x_field_write(sc, ADC_EN, !!en);
}

static int sc760x_set_adc_done_mask(struct sc760x_chip *sc, bool en)
{
    dev_info(sc->dev, "%s : set adc done mask %d\n", __func__, en);
    return sc760x_field_write(sc, ADC_DONE_MASK, !!en);
}

static int sc760x_get_adc(struct sc760x_chip *sc,
            ADC_CH channel, int *result)
{
    int reg = 0x17 + channel * 2;
    u8 val[2] = {0};
    int ret;

    ret = sc760x_read_block(sc, reg, val, 2);
    if (ret) {
        return ret;
    }

    *result = (val[1] | (val[0] << 8)) *
                sc760x_adc_m[channel] / sc760x_adc_l[channel];

    return ret;
}

static int sc760x_dump_reg(struct sc760x_chip *sc)
{
    int ret = 0, i = 0, val = 0, desc = 0;
    char buf[1024];
    if (!sc->sc760x_enable)
        return -1;

    for (i = 0; i <= 0x15; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        if (!ret) {
            desc +=
                sprintf(buf + desc, "[0x%02x]:0x%02x, ", i, val);
        }
    }

    dev_err(sc->dev, "Reg %s\n ", buf);
    return ret;
}

__maybe_unused static irqreturn_t sc760x_irq_handler(int irq, void *data)
{
    struct sc760x_chip *sc = data;

    dev_dbg(sc->dev, "%s\n", __func__);
    //sc760x_dump_reg(sc);
    return IRQ_HANDLED;
}

static ssize_t sc760x_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc7603");
    for (addr = 0x0; addr <= 0x20; addr++) {
        ret = regmap_read(sc->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc760x_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x20)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc760x_show_registers, sc760x_store_register);

static ssize_t sc760x_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);


	r = kstrtoul(buf, 0, &enable);
	if (r) {
		pr_err("Invalid tx_mode = %lu\n", enable);
		return -EINVAL;
	}

	if (enable < 0) {
		sc->user_gpio_en = -EINVAL;
		pr_info("Clear user gpio en setting\n");
	} else {
		sc->user_gpio_en = !!enable;
		pr_info("Set user gpio en: %d\n", sc->user_gpio_en);
		sc760x_enable_chip(sc, enable);
		sc->sc760x_enable = enable;
	}

	return r ? r : count;
}

static ssize_t sc760x_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);

	return scnprintf(buf, 50, "%d\n", sc->sc760x_enable);
}

static DEVICE_ATTR(sc760x_enable, S_IRUGO|S_IWUSR, sc760x_enable_show, sc760x_enable_store);

static ssize_t charger_suspend_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int chg_suspend;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &chg_suspend);
	if (ret) {
		pr_err("Invalid chg_suspend value, ret = %d\n", ret);
		return ret;
	}

	if (chg_suspend < 0) {
		sc->user_chg_susp = -EINVAL;
		pr_info("Clear user suspend charger setting\n");
	} else {
		sc->user_chg_susp = !!chg_suspend;
		pr_info("Set user suspend charger: %d\n", sc->user_chg_susp);
		sc760x_set_load_switch(sc, sc->user_chg_susp);
	}
	cancel_delayed_work(&sc->charge_monitor_work);
	schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t charger_suspend_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sc->user_chg_susp);
}
DEVICE_ATTR(charger_suspend, S_IRUGO|S_IWUSR, charger_suspend_show, charger_suspend_store);

static bool sc760x_is_enabled_charging(struct sc760x_chip *sc)
{
	int ret = 0, ls_off = 0;
	bool enabled = false;

	ret = sc760x_get_load_switch(sc, &ls_off);
	if(ret)
		return false;

	enabled = ls_off ? false : true;
	return enabled;
}

int sc760x_enable_charger(struct sc760x_chip *sc)
{
	int ret;

	if (sc->user_chg_en == 0) {
		pr_info("Skip enable charging for user request override\n");
		return 0;
	}

	if (sc->user_chg_susp > 0) {
		pr_info("Skip enable charging for user chg suspend\n");
		return 0;
	}

	ret = sc760x_set_load_switch(sc, false);
	pr_info("sc760x_enable_charger\n");
	return ret;
}

int sc760x_disable_charger(struct sc760x_chip *sc)
{
	int ret;

	if (sc->user_chg_en > 0) {
		pr_info("Skip disable charging for user request override\n");
		return 0;
	}

	ret = sc760x_set_load_switch(sc, true);

	pr_info("sc760x_disable_charger\n");
	return ret;
}

static ssize_t chg_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &enable);
	if (ret) {
		pr_err("Invalid chg_en value, ret = %d\n", ret);
		return ret;
	}

	if (enable < 0) {
		sc->user_chg_en = -EINVAL;
		pr_info("Clear user enable charging setting\n");
	} else {
		sc->user_chg_en = !!enable;
		pr_info("Set user enable charging: %d\n", sc->user_chg_en);
		if (sc->user_chg_en)
			sc760x_enable_charger(sc);
		else
			sc760x_disable_charger(sc);
	}
	cancel_delayed_work(&sc->charge_monitor_work);
	schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t chg_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	bool enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	enable = sc760x_is_enabled_charging(sc);

	return sprintf(buf, "%d\n", enable);
}
DEVICE_ATTR(chg_en , S_IRUGO|S_IWUSR, chg_en_show, chg_en_store);

static void sc760x_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
    device_create_file(dev, &dev_attr_sc760x_enable);
    device_create_file(dev, &dev_attr_charger_suspend);
    device_create_file(dev, &dev_attr_chg_en);
}

static void sc760x_remove_device_node(struct device *dev)
{
    device_remove_file(dev, &dev_attr_registers);
    device_remove_file(dev, &dev_attr_sc760x_enable);
    device_remove_file(dev, &dev_attr_charger_suspend);
    device_remove_file(dev, &dev_attr_chg_en);
}

static enum power_supply_property sc760x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT
};


static enum power_supply_usb_type sc760x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,		/* Standard Downstream Port */
	POWER_SUPPLY_USB_TYPE_DCP,		/* Dedicated Charging Port */
	POWER_SUPPLY_USB_TYPE_CDP,		/* Charging Downstream Port */
	POWER_SUPPLY_USB_TYPE_ACA,		/* Accessory Charger Adapters */
	POWER_SUPPLY_USB_TYPE_C,		/* Type C Port */
	POWER_SUPPLY_USB_TYPE_PD,		/* Power Delivery Port */
	POWER_SUPPLY_USB_TYPE_PD_DRP,		/* PD Dual Role Port */
	POWER_SUPPLY_USB_TYPE_PD_PPS,		/* PD Programmable Power Supply */
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,	/* Apple Charging Method */
};

static char *sc760x_charger_supplied_to[] = {
	"battery",
};

static int sc760x_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return true;
	default:
		return false;
	}
}

static int sc760x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sc760x_chip *sc = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0) {
			sc->user_ilim = -EINVAL;
			ret = 0;
			pr_info("Clear user input limit\n");
		} else {
			sc->user_ilim = val->intval;
			ret = sc760x_set_ibat_limit(sc, val->intval / 1000);
			pr_info("Set user input limit: %duA\n", sc->user_ilim);
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < 0) {
			sc->user_ichg = -EINVAL;
			ret = 0;
			pr_info("Clear user charging current limit\n");
		} else {
			sc->user_ichg = val->intval;
			ret = sc760x_set_ibat_limit(sc, val->intval / 1000);
			pr_info("Set user charging current limit: %duA\n", sc->user_ichg);
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int sc760x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sc760x_chip *sc = power_supply_get_drvdata(psy);
	struct sc760x_state state = sc->state;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (state.work_mode) {
		case WORK_TRICKLE_CHARGE:
		case WORK_PRE_CHARGE:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case WORK_FULLY_ON:
		case WORK_CURRENT_REGULATION:
		case WORK_POWER_REGULATION:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case WORK_FULLY_OFF:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SC760X_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SC760X_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sc760x_is_enabled_charging(sc);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sc->sc760x_enable;
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sc760x_power_supply_desc.type;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbat_adc;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = state.vchg_adc;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = state.tdie_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibat_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc760x_get_ibat_limit(sc, &val->intval);
		val->intval = val->intval * 1000;
		if (ret < 0)
			return ret;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc760x_get_ibat_limit(sc, &val->intval);
		val->intval = val->intval * 1000;
		if (ret < 0)
			return ret;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct power_supply_desc sc760x_power_supply_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.usb_types = sc760x_usb_type,
	.num_usb_types = ARRAY_SIZE(sc760x_usb_type),
	.properties = sc760x_power_supply_props,
	.num_properties = ARRAY_SIZE(sc760x_power_supply_props),
	.get_property = sc760x_charger_get_property,
	.set_property = sc760x_charger_set_property,
	.property_is_writeable = sc760x_property_is_writeable,
};

static int sc760x_power_supply_init(struct sc760x_chip *sc,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sc,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sc760x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sc760x_charger_supplied_to);

        dev_err(dev, "regiser master power supply\n");
	sc->charger_psy = devm_power_supply_register(sc->dev,
						 &sc760x_power_supply_desc,
						 &psy_cfg);

	if (IS_ERR(sc->charger_psy))
		return PTR_ERR(sc->charger_psy);

	return 0;
}

static int sc760x_parse_dt(struct sc760x_chip *sc, struct device *dev)
{
    int i, ret;

    struct device_node *node = dev->of_node;

    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc760x,bat-chg-lim-disable", &(sc->pdata->bat_chg_lim_disable)},
        {"sc,sc760x,bat-chg-lim", &(sc->pdata->bat_chg_lim)},
        {"sc,sc760x,pow-lim-disable", &(sc->pdata->pow_lim_disable)},
        {"sc,sc760x,ilim-disable", &(sc->pdata->ilim_disable)},
        {"sc,sc760x,load-switch-disable", &(sc->pdata->load_switch_disable)},
        {"sc,sc760x,low-power-mode-enable", &(sc->pdata->lp_mode_enable)},
        {"sc,sc760x,itrichg", &(sc->pdata->itrichg)},
        {"sc,sc760x,iprechg", &(sc->pdata->iprechg)},
        {"sc,sc760x,vfc-chg", &(sc->pdata->vfc_chg)},
        {"sc,sc760x,chg-ovp-disable", &(sc->pdata->chg_ovp_disable)},
        {"sc,sc760x,chg-ovp", &(sc->pdata->chg_ovp)},
        {"sc,sc760x,bat-ovp-disable", &(sc->pdata->bat_ovp_disable)},
        {"sc,sc760x,bat-ovp", &(sc->pdata->bat_ovp)},
        {"sc,sc760x,chg-ocp-disable", &(sc->pdata->chg_ocp_disable)},
        {"sc,sc760x,chg-ocp", &(sc->pdata->chg_ocp)},
        {"sc,sc760x,dsg-ocp-disable", &(sc->pdata->dsg_ocp_disable)},
        {"sc,sc760x,dsg-ocp", &(sc->pdata->dsg_ocp)},
        {"sc,sc760x,tdie-flt-disable", &(sc->pdata->tdie_flt_disable)},
        {"sc,sc760x,tdie-alm-disable", &(sc->pdata->tdie_alm_disable)},
        {"sc,sc760x,tdie-alm", &(sc->pdata->tdie_alm)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(node, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            continue;
        }
    }

	/*chgdev name */
	if (of_property_read_string(node, "chg_name", &sc->pdata->chg_name))
		dev_err(sc->dev, "failed to get chg_name\n");

	dev->platform_data = sc->pdata;

	sc->init_data.charger_disabled = of_property_read_bool(node,
					"init-charger-disabled");

	ret = device_property_read_u32(sc->dev,
				       "iterm-microamp",
				       &sc->init_data.iterm);
	if (ret)
		sc->init_data.iterm = SC760x_TERMCHRG_I_DEF_uA;

	ret = device_property_read_u32(sc->dev,
				       "ichg-max-microamp",
				       &sc->init_data.max_ichg);
	if (ret)
		sc->init_data.max_ichg = SC760x_ICHRG_I_MAX_uA;

	ret = device_property_read_u32(sc->dev,
				       "vchg-max-microvolt",
				       &sc->init_data.max_vreg);
	if (ret)
		sc->init_data.max_vreg = SC760x_VREG_V_MAX_uV;

	ret = device_property_read_u32(sc->dev,
				       "ichg-microamp",
				       &sc->init_data.ichg);
	if (ret)
		sc->init_data.ichg = SC760x_ICHRG_I_DEF_uA;

	return 0;
}

static int sc760x_init_device(struct sc760x_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc760x_fields field_id;
        int conv_data;
    } props[] = {
        {IBAT_CHG_LIM_DIS, sc->pdata->bat_chg_lim_disable},
        {IBAT_CHG_LIM, sc->pdata->bat_chg_lim},
        {POW_LIM_DIS, sc->pdata->pow_lim_disable},
        {ILIM_DIS, sc->pdata->ilim_disable},
        {LS_OFF, sc->pdata->load_switch_disable},
        {EN_LOWPOWER, sc->pdata->lp_mode_enable},
        {ITRICHG, sc->pdata->itrichg},
        {IPRECHG, sc->pdata->iprechg},
        {VFC_CHG, sc->pdata->vfc_chg},
        {CHG_OVP_DIS, sc->pdata->chg_ovp_disable},
        {CHG_OVP, sc->pdata->chg_ovp},
        {BAT_OVP_DIS, sc->pdata->bat_ovp_disable},
        {BAT_OVP, sc->pdata->bat_ovp},
        {CHG_OCP_DIS, sc->pdata->chg_ocp_disable},
        {CHG_OCP, sc->pdata->chg_ocp},
        {DSG_OCP_DIS, sc->pdata->dsg_ocp_disable},
        {DSG_OCP, sc->pdata->dsg_ocp},
        {TDIE_FLT_DIS, sc->pdata->tdie_flt_disable},
        {TDIE_ALRM_DIS, sc->pdata->tdie_alm_disable},
        {TDIE_ALRM, sc->pdata->tdie_alm},
    };

    ret = sc760x_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc760x_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    ret = sc760x_set_adc_enable(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
    }

    ret = sc760x_set_adc_done_mask(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to set adc mask (%d)\n", __func__, ret);
    }

    return sc760x_dump_reg(sc);
}

static int sc760x_register_interrupt(struct sc760x_chip *sc, struct i2c_client *client)
{
    int ret = 0;
//<MMI_STOPSHIP>: PMIC: disable balance ic irq
//    ret = devm_request_threaded_irq(sc->dev, client->irq, NULL,
//                    sc760x_irq_handler,
//                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
//                    sc760x_irq_name[sc->role], sc);
    if (ret < 0) {
        dev_err(sc->dev, "request thread irq failed:%d\n", ret);
        return ret;
    }

    sc->irq_enabled = false;
    sc->irq = client->irq;
    dev_err(sc->dev, "request thread irq success\n");
    return 0;
}

static int sc760x_get_state(struct sc760x_chip *sc,
			     struct sc760x_state *state)
{
	sc760x_get_work_mode(sc, (int *)&state->work_mode);

	sc760x_get_adc(sc, ADC_IBAT, &state->ibat_adc);
	sc760x_get_adc(sc, ADC_IBAT, &state->ibat_adc);
	sc760x_get_adc(sc, ADC_VBAT, &state->vbat_adc);
	sc760x_get_adc(sc, ADC_VCHG, &state->vchg_adc);
	sc760x_get_adc(sc, ADC_TBAT, &state->tbat_adc);
	sc760x_get_adc(sc, ADC_TDIE, &state->tdie_adc);
	state->ibat_adc = state->ibat_adc * 1000;
	state->vbat_adc = state->vbat_adc * 1000;
	state->vchg_adc = state->vchg_adc * 1000;

	return 0;
}

static void charger_monitor_work_func(struct work_struct *work)
{
	struct sc760x_chip *sc = container_of(work,
					struct sc760x_chip,
					charge_monitor_work.work);

	sc760x_get_state(sc, &sc->state);

	sc760x_dump_reg(sc);

	schedule_delayed_work(&sc->charge_monitor_work, 10*HZ);
}

static int sc760x_plug_in(struct charger_device *chgdev)
{
	struct sc760x_chip *sc = charger_get_data(chgdev);

	sc760x_set_auto_bsm_dis(sc, true);
	cancel_delayed_work(&sc->charge_monitor_work);
	schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

	return 0;
}

static int sc760x_plug_out(struct charger_device *chgdev)
{
	struct sc760x_chip *sc = charger_get_data(chgdev);
	
	sc760x_disable_charger(sc);
	sc760x_set_auto_bsm_dis(sc, false);

	return 0;
}

static const struct charger_ops sc760x_chg_ops = {
	/* cable plug in/out */
	.plug_in = sc760x_plug_in,
	.plug_out = sc760x_plug_out,
};

static const struct charger_properties sc760x_chg_props = {
	.alias_name = "sc760x_chg",
};

static int sc760x_chg_init_chgdev(struct sc760x_chip *sc)
{
	struct sc760x_chg_platform_data *pdata = dev_get_platdata(sc->dev);

	pr_debug("%s\n", __func__);
	sc->chgdev = charger_device_register(pdata->chg_name, sc->dev,
						sc, &sc760x_chg_ops,
						&sc760x_chg_props);
	return IS_ERR(sc->chgdev) ? PTR_ERR(sc->chgdev) : 0;
}

static struct of_device_id sc760x_charger_match_table[] = {
    {   .compatible = "southchip,sc7603_master",
        .data = &sc760x_role_data[SC760X_MASTER], },
    {   .compatible = "southchip,sc7603_slave",
        .data = &sc760x_role_data[SC760X_SLAVE], },
    { },
};
MODULE_DEVICE_TABLE(of, sc760x_charger_match_table);

static int sc760x_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc760x_chip *sc;
    struct device *dev = &client->dev;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    char *name = NULL;
    int ret = 0, i =0;

    pr_err("%s (%s)\n", __func__, SC760X_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc760x_chip), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->dev = dev;
    sc->client = client;
    mutex_init(&sc->lock);

    sc->user_ilim = -EINVAL;
    sc->user_ichg = -EINVAL;
    sc->user_chg_en = -EINVAL;
    sc->user_chg_susp = -EINVAL;
    sc->user_gpio_en = -EINVAL;
    sc->sc760x_enable = 0;


    sc->regmap = devm_regmap_init_i2c(client,
                            &sc760x_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        return -EINVAL;
    }

    for (i = 0; i < ARRAY_SIZE(sc760x_reg_fields); i++) {
        const struct reg_field *reg_fields = sc760x_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            return PTR_ERR(sc->rmap_fields[i]);
        }
    }

    i2c_set_clientdata(client, sc);
    sc760x_create_device_node(&(client->dev));

    match = of_match_node(sc760x_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        goto err_get_match;
    }

    sc->role = *(int *)match->data;

    dev_err(sc->dev, "sc760x[%s] probe running!!!\n",
            sc->role == SC760X_MASTER ? "master" : "slave");

    sc->pdata = &default_cfg;
    ret = sc760x_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc760x_chg_init_chgdev(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init chg dev fail(%d)\n",
                    __func__, ret);
        goto err_init_chgdev;
    }

    ret = sc760x_register_interrupt(sc, client);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

    name = devm_kasprintf(sc->dev, GFP_KERNEL, "%s",
		"sc760x suspend wakelock");
    sc->charger_wakelock =
		wakeup_source_register(NULL, name);

    ret = sc760x_power_supply_init(sc, dev);
    if (ret) {
        dev_err(dev, "Failed to register power supply, ret=%d\n", ret);
        goto err_init_device;
    }

    ret = sc760x_enable_chip(sc, true);
    if (ret) {
        dev_err(dev, "Failed to enable sc760x chip, ret=%d\n", ret);
        goto err_init_device;
    }

    INIT_DELAYED_WORK(&sc->charge_monitor_work, charger_monitor_work_func);

    schedule_delayed_work(&sc->charge_monitor_work,
						msecs_to_jiffies(0));

    dev_err(sc->dev, "sc760x[%s] probe successfully!!!\n",
            sc->role == SC760X_MASTER ? "master" : "slave");
    return 0;

err_register_irq:
err_init_device:
err_init_chgdev:
err_parse_dt:
err_get_match:
    dev_err(sc->dev, "sc760x probe failed!\n");
    devm_kfree(sc->dev, sc);
    return ret;
}


static void sc760x_charger_remove(struct i2c_client *client)
{
    struct sc760x_chip *sc = i2c_get_clientdata(client);
    sc760x_remove_device_node(&(client->dev));

    cancel_delayed_work_sync(&sc->charge_monitor_work);

    power_supply_unregister(sc->charger_psy);

    sc760x_enable_chip(sc, false);

    mutex_destroy(&sc->lock);
    return;
}

#ifdef CONFIG_PM_SLEEP
static int sc760x_suspend(struct device *dev)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret = 0;
    dev_info(sc->dev, "Suspend successfully!");
//    if (device_may_wakeup(dev))
//        enable_irq_wake(sc->irq);
//    disable_irq(sc->irq);
    ret = sc760x_set_adc_enable(sc, false);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to disable adc(%d)\n", __func__, ret);
    }

    ret = sc760x_set_lowpower_mode(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enter lowpower(%d)\n", __func__, ret);
    }

    cancel_delayed_work_sync(&sc->charge_monitor_work);

    return 0;
}
static int sc760x_resume(struct device *dev)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret = 0;
    dev_info(sc->dev, "Resume successfully!");
//    if (device_may_wakeup(dev))
//        disable_irq_wake(sc->irq);
//    enable_irq(sc->irq);

    ret = sc760x_set_lowpower_mode(sc, false);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to exit lowpower(%d)\n", __func__, ret);
    }

    ret = sc760x_set_adc_enable(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
    }

    cancel_delayed_work_sync(&sc->charge_monitor_work);
    schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

    return 0;
}

static const struct dev_pm_ops sc760x_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc760x_suspend, sc760x_resume)
};
#endif

static void sc760x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sc760x_chip *sc = i2c_get_clientdata(client);
	ret = sc760x_set_adc_enable(sc, false);
	if (ret < 0) {
		dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
	}

	ret = sc760x_enable_chip(sc, false);
	if (ret) {
		pr_err("Failed to disable sc760x chip, ret = %d\n", ret);
	}
	pr_info("sc760x_charger_shutdown\n");
}

static const struct i2c_device_id sc760x_i2c_ids[] = {
	{ "sc7603_master", SC760X_MASTER },
	{ "sc7603_slave", SC760X_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc760x_i2c_ids);

static struct i2c_driver sc760x_charger_driver = {
    .driver     = {
        .name   = "sc760x-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc760x_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc760x_pm,
#endif
    },
    .probe      = sc760x_charger_probe,
    .remove     = sc760x_charger_remove,
    .shutdown = sc760x_charger_shutdown,
    .id_table = sc760x_i2c_ids,
};

module_i2c_driver(sc760x_charger_driver);

MODULE_DESCRIPTION("SC SC760X Driver");
MODULE_LICENSE("GPL v2");

