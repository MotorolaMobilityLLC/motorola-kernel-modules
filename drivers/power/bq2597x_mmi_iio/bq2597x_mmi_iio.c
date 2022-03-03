/*
 * BQ2570x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[bq2597x] %s: " fmt, __func__

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
#include "bq25970_reg.h"
#include "bq2597x_charger_iio.h"
#include <linux/gpio.h>
#include <linux/iio/consumer.h>

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err

#endif

enum {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

enum {
	BQ25970 = 0x00,
};

#define BQ2597X_PART_NO	0x10
#define SC8551_PART_NO	0x00
#define SC8551A_PART_NO	0x51
#define NU2105_PART_NO	0xC0
#define NU2105_MAX_SHOW_REG_ADDR 		0x2F
#define SC8551_MAX_SHOW_REG_ADDR 		0x36
#define BQ25970_MAX_SHOW_REG_ADDR 		0x2A
#define	BAT_OVP_ALARM		BIT(7)
#define BAT_OCP_ALARM		BIT(6)
#define	BUS_OVP_ALARM		BIT(5)
#define	BUS_OCP_ALARM		BIT(4)
#define	BAT_UCP_ALARM		BIT(3)
#define	VBUS_INSERT		BIT(2)
#define VBAT_INSERT		BIT(1)
#define	ADC_DONE		BIT(0)

#define BAT_OVP_FAULT		BIT(7)
#define BAT_OCP_FAULT		BIT(6)
#define BUS_OVP_FAULT		BIT(5)
#define BUS_OCP_FAULT		BIT(4)
#define TBUS_TBAT_ALARM		BIT(3)
#define TS_BAT_FAULT		BIT(2)
#define	TS_BUS_FAULT		BIT(1)
#define	TS_DIE_FAULT		BIT(0)

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	BAT_THERM_FAULT_SHIFT			12
#define	BUS_THERM_FAULT_SHIFT			13
#define	DIE_THERM_FAULT_SHIFT			14

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
/*end*/

#define AUTO_ENABLE_SHOW_MAX_SIZE 50

struct bq2597x_cfg {
	bool bat_ovp_disable;
	bool bat_ocp_disable;
	bool bat_ovp_alm_disable;
	bool bat_ocp_alm_disable;

	int bat_ovp_th;
	int bat_ovp_alm_th;
	int bat_ocp_th;
	int bat_ocp_alm_th;

	bool bus_ovp_alm_disable;
	bool bus_ocp_disable;
	bool bus_ocp_alm_disable;

	int bus_ovp_th;
	int bus_ovp_alm_th;
	int bus_ocp_th;
	int bus_ocp_alm_th;

	bool bat_ucp_alm_disable;

	int bat_ucp_alm_th;
	int ac_ovp_th;

	bool bat_therm_disable;
	bool bus_therm_disable;
	bool die_therm_disable;

	int bat_therm_th; /*in %*/
	int bus_therm_th; /*in %*/
	int die_therm_th; /*in degC*/

	int sense_r_mohm;
};

struct bq2597x {
	struct device *dev;
	struct i2c_client *client;

	int device_id;
	int part_no;
	int revision;
	struct iio_dev		*indio_dev;
	struct iio_chan_spec	*iio_chan;
	struct iio_channel	*int_iio_chans;
	struct iio_channel	**ext_iio_chans;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;
	struct regmap		*regmap;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int bat_temp;
	int bus_temp;
	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int  prev_alarm;
	int  prev_fault;

	int chg_ma;
	int chg_mv;

	int charge_state;

	struct bq2597x_cfg *cfg;

	int skip_writes;
	int skip_reads;

	struct bq2597x_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct power_supply *fc2_psy;

	//struct regulator	*vdd_i2c_vreg;
	//struct pinctrl		*irq_pinctrl;
	struct gpio		irq_gpio;
};

/************************************************************************/

struct bq2597x *g_chip;

static int __bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __bq2597x_write_byte(struct bq2597x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int __bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_write_byte(struct bq2597x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_update_bits(struct bq2597x *bq, u8 reg,
				    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2597x_write_byte(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

static int bq2597x_enable_charge(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_CHG_ENABLE;
	else
		val = BQ2597X_CHG_DISABLE;

	val <<= BQ2597X_CHG_EN_SHIFT;

	pr_err("bq2597x  enable charge   %d\n", enable);

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_CHG_EN_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_charge);

static int bq2597x_check_charge_enabled(struct bq2597x *bq, bool *enabled)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0C, &val);
	if (!ret)
		*enabled = !!(val & BQ2597X_CHG_EN_MASK);
	return ret;
}

static int bq2597x_enable_wdt(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_WATCHDOG_ENABLE;
	else
		val = BQ2597X_WATCHDOG_DISABLE;

	val <<= BQ2597X_WATCHDOG_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,
				BQ2597X_WATCHDOG_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_wdt);

static int bq2597x_set_wdt(struct bq2597x *bq, int ms)
{
	int ret;
	u8 val;

	if (ms == 500)
		val = BQ2597X_WATCHDOG_0P5S;
	else if (ms == 1000)
		val = BQ2597X_WATCHDOG_1S;
	else if (ms == 5000)
		val = BQ2597X_WATCHDOG_5S;
	else if (ms == 30000)
		val = BQ2597X_WATCHDOG_30S;
	else
		val = BQ2597X_WATCHDOG_30S;

	val <<= BQ2597X_WATCHDOG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,
				BQ2597X_WATCHDOG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_wdt);

static int bq2597x_enable_batovp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ENABLE;
	else
		val = BQ2597X_BAT_OVP_DISABLE;

	val <<= BQ2597X_BAT_OVP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,
				BQ2597X_BAT_OVP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp);

static int bq2597x_set_batovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	pr_err("bq2597x  bq2597x_set_batovp_th   %d\n", threshold);
	if (threshold < BQ2597X_BAT_OVP_BASE)
		threshold = BQ2597X_BAT_OVP_BASE;

	val = (threshold - BQ2597X_BAT_OVP_BASE) / BQ2597X_BAT_OVP_LSB;

	val <<= BQ2597X_BAT_OVP_SHIFT;
	pr_err("bq2597x  bq2597x_set_batovp_th   0X%x\n", val);
	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,
				BQ2597X_BAT_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_th);

static int bq2597x_enable_batovp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OVP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
				BQ2597X_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp_alarm);

static int bq2597x_set_batovp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OVP_ALM_BASE)
		threshold = BQ2597X_BAT_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OVP_ALM_BASE) / BQ2597X_BAT_OVP_ALM_LSB;

	val <<= BQ2597X_BAT_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
				BQ2597X_BAT_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_alarm_th);

static int bq2597x_enable_batocp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ENABLE;
	else
		val = BQ2597X_BAT_OCP_DISABLE;

	val <<= BQ2597X_BAT_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
				BQ2597X_BAT_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp);

static int bq2597x_set_batocp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_BASE)
		threshold = BQ2597X_BAT_OCP_BASE;

	val = (threshold - BQ2597X_BAT_OCP_BASE) / BQ2597X_BAT_OCP_LSB;

	val <<= BQ2597X_BAT_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
				BQ2597X_BAT_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_th);

static int bq2597x_enable_batocp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
				BQ2597X_BAT_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp_alarm);

static int bq2597x_set_batocp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_ALM_BASE)
		threshold = BQ2597X_BAT_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OCP_ALM_BASE) / BQ2597X_BAT_OCP_ALM_LSB;

	val <<= BQ2597X_BAT_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
				BQ2597X_BAT_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_alarm_th);

static int bq2597x_set_busovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_BASE)
		threshold = BQ2597X_BUS_OVP_BASE;

	val = (threshold - BQ2597X_BUS_OVP_BASE) / BQ2597X_BUS_OVP_LSB;

	val <<= BQ2597X_BUS_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_06,
				BQ2597X_BUS_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_th);

static int bq2597x_enable_busovp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OVP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
				BQ2597X_BUS_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busovp_alarm);

static int bq2597x_set_busovp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_ALM_BASE)
		threshold = BQ2597X_BUS_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OVP_ALM_BASE) / BQ2597X_BUS_OVP_ALM_LSB;

	val <<= BQ2597X_BUS_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
				BQ2597X_BUS_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_alarm_th);

static int bq2597x_enable_busocp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ENABLE;
	else
		val = BQ2597X_BUS_OCP_DISABLE;

	val <<= BQ2597X_BUS_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
				BQ2597X_BUS_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp);

static int bq2597x_set_busocp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_BASE)
		threshold = BQ2597X_BUS_OCP_BASE;

	val = (threshold - BQ2597X_BUS_OCP_BASE) / BQ2597X_BUS_OCP_LSB;

	val <<= BQ2597X_BUS_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
				BQ2597X_BUS_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_th);

static int bq2597x_enable_busocp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OCP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
				BQ2597X_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp_alarm);

static int bq2597x_set_busocp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_ALM_BASE)
		threshold = BQ2597X_BUS_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OCP_ALM_BASE) / BQ2597X_BUS_OCP_ALM_LSB;

	val <<= BQ2597X_BUS_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
				BQ2597X_BUS_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_alarm_th);

static int bq2597x_enable_batucp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_UCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_UCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_UCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
				BQ2597X_BAT_UCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batucp_alarm);

static int bq2597x_set_batucp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_UCP_ALM_BASE)
		threshold = BQ2597X_BAT_UCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_UCP_ALM_BASE) / BQ2597X_BAT_UCP_ALM_LSB;

	val <<= BQ2597X_BAT_UCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
				BQ2597X_BAT_UCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batucp_alarm_th);

static int bq2597x_set_acovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_AC_OVP_BASE)
		threshold = BQ2597X_AC_OVP_BASE;

	if (threshold == BQ2597X_AC_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - BQ2597X_AC_OVP_BASE) /  BQ2597X_AC_OVP_LSB;

	val <<= BQ2597X_AC_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
				BQ2597X_AC_OVP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2597x_set_acovp_th);

static int bq2597x_set_vdrop_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold == 300)
		val = BQ2597X_VDROP_THRESHOLD_300MV;
	else
		val = BQ2597X_VDROP_THRESHOLD_400MV;

	val <<= BQ2597X_VDROP_THRESHOLD_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
				BQ2597X_VDROP_THRESHOLD_SET_MASK,
				val);

	return ret;
}

static int bq2597x_set_vdrop_deglitch(struct bq2597x *bq, int us)
{
	int ret;
	u8 val;

	if (us == 8)
		val = BQ2597X_VDROP_DEGLITCH_8US;
	else
		val = BQ2597X_VDROP_DEGLITCH_5MS;

	val <<= BQ2597X_VDROP_DEGLITCH_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
				BQ2597X_VDROP_DEGLITCH_SET_MASK,
				val);
	return ret;
}

static int bq2597x_enable_bat_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBAT_ENABLE;
	else
		val = BQ2597X_TSBAT_DISABLE;

	val <<= BQ2597X_TSBAT_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_TSBAT_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bat_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bat_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_29, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bat_therm_th);

static int bq2597x_enable_bus_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBUS_ENABLE;
	else
		val = BQ2597X_TSBUS_DISABLE;

	val <<= BQ2597X_TSBUS_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_TSBUS_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bus_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bus_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_28, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bus_therm_th);

static int bq2597x_enable_die_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TDIE_ENABLE;
	else
		val = BQ2597X_TDIE_DISABLE;

	val <<= BQ2597X_TDIE_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_TDIE_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_die_therm);

/*
 * please be noted that the unit here is degC
 */
static int bq2597x_set_die_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;
	u8 val;

	/*BE careful, LSB is here is 1/LSB, so we use multiply here*/
	val = (threshold - BQ2597X_TDIE_ALM_BASE) * BQ2597X_TDIE_ALM_LSB;
	val <<= BQ2597X_TDIE_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2A,
				BQ2597X_TDIE_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_die_therm_th);

static int bq2597x_enable_adc(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_ADC_ENABLE;
	else
		val = BQ2597X_ADC_DISABLE;

	val <<= BQ2597X_ADC_EN_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_adc);

static int bq2597x_set_adc_average(struct bq2597x *bq, bool avg)
{
	int ret;
	u8 val;

	if (avg)
		val = BQ2597X_ADC_AVG_ENABLE;
	else
		val = BQ2597X_ADC_AVG_DISABLE;

	val <<= BQ2597X_ADC_AVG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_AVG_MASK, val);
	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_average);

static int bq2597x_set_adc_scanrate(struct bq2597x *bq, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = BQ2597X_ADC_RATE_ONESHOT;
	else
		val = BQ2597X_ADC_RATE_CONTINOUS;

	val <<= BQ2597X_ADC_RATE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_scanrate);

static int bq2597x_set_adc_bits(struct bq2597x *bq, int bits)
{
	int ret;
	u8 val;

	if (bits > 15)
		bits = 15;
	if (bits < 12)
		bits = 12;
	val = 15 - bits;

	val <<= BQ2597X_ADC_SAMPLE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_SAMPLE_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_bits);

#define ADC_REG_BASE 0x16
static int bq2597x_get_adc_data(struct bq2597x *bq, int channel,  int *result)
{
	int ret;
	u16 val;
	s16 t;
	int tmp;
	u8 val_h, val_l;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (bq->part_no == NU2105_PART_NO) {
		ret = bq2597x_read_byte(bq, ADC_REG_BASE + (channel << 1), &val_h);
		ret |= bq2597x_read_byte(bq, ADC_REG_BASE + (channel << 1) + 1, &val_l);
		if (ret < 0) {
			pr_err(" nu2105 i2c read failed!\n");
			return ret;
                }
		t = (u16)val_l + ((u16)val_h << 8);
	} else {

		ret = bq2597x_read_word(bq, ADC_REG_BASE + (channel << 1), &val);
		if (ret < 0) {
			pr_err(" bq2597x i2c read failed!\n");
			return ret;
		}
		t = val & 0xFF;
		t <<= 8;
		t |= (val >> 8) & 0xFF;
	}

	if (bq->part_no == SC8551_PART_NO
		||bq->part_no == SC8551A_PART_NO) {
		if (channel == ADC_IBUS) {
			tmp = t * SC8551_IBUS_ADC_LSB_MUL;
			t = tmp /SC8551_IBUS_ADC_LSB_DIV;
		} else if (channel == ADC_VBUS) {
			tmp = t * SC8551_VBUS_ADC_LSB_MUL;
			t = tmp /SC8551_VBUS_ADC_LSB_DIV;
		} else if (channel == ADC_VAC)
			t *= SC8551_VAC_ADC_LSB;
		else if (channel == ADC_VOUT) {
			tmp = t * SC8551_VOUT_ADC_LSB_MUL;
			t = tmp /SC8551_VOUT_ADC_LSB_DIV;
		} else if (channel == ADC_VBAT) {
			tmp = t * SC8551_VBAT_ADC_LSB_MUL;
			t = tmp /SC8551_VBAT_ADC_LSB_DIV;
		} else if (channel == ADC_IBAT) {
			tmp = t * SC8551_IBAT_ADC_LSB_MUL;
			t = tmp /SC8551_IBAT_ADC_LSB_DIV;
		}
	}

	*result = t;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_get_adc_data);

static int bq2597x_set_adc_scan(struct bq2597x *bq, int channel, bool enable)
{
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = BQ2597X_REG_14;
		shift = BQ2597X_IBUS_ADC_DIS_SHIFT;
		mask = BQ2597X_IBUS_ADC_DIS_MASK;
	} else {
		reg = BQ2597X_REG_15;
		shift = 8 - channel;
		mask = 1 << shift;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = bq2597x_update_bits(bq, reg, mask, val);

	return ret;
}

static int bq2597x_set_alarm_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_alarm_int_mask);

static int bq2597x_clear_alarm_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_alarm_int_mask);

static int bq2597x_set_fault_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_fault_int_mask);

static int bq2597x_clear_fault_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_fault_int_mask);

static int bq2597x_set_sense_resistor(struct bq2597x *bq, int r_mohm)
{
	int ret;
	u8 val;

	if (r_mohm == 2)
		val = BQ2597X_SET_IBAT_SNS_RES_2MHM;
	else if (r_mohm == 5)
		val = BQ2597X_SET_IBAT_SNS_RES_5MHM;
	else
		return -EINVAL;

	val <<= BQ2597X_SET_IBAT_SNS_RES_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
				BQ2597X_SET_IBAT_SNS_RES_MASK,
				val);
	return ret;
}

static int bq2597x_set_ucp_threshold(struct bq2597x *bq, int curr_ma)
{
	int ret;
	u8 val;

	if (curr_ma == 300)
		val = BQ2597X_IBUS_UCP_RISE_300MA;
	else if (curr_ma == 500)
		val = BQ2597X_IBUS_UCP_RISE_500MA;
	else
		val = BQ2597X_IBUS_UCP_RISE_300MA;

	val <<= BQ2597X_IBUS_UCP_RISE_TH_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
				BQ2597X_IBUS_UCP_RISE_TH_MASK,
				val);
	return ret;
}

static int bq2597x_enable_regulation(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_EN_REGULATION_ENABLE;
	else
		val = BQ2597X_EN_REGULATION_DISABLE;

	val <<= BQ2597X_EN_REGULATION_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
				BQ2597X_EN_REGULATION_MASK,
				val);

	return ret;

}

static int bq2597x_set_ss_timeout(struct bq2597x *bq, int timeout)
{
	int ret;
	u8 val;

	switch (timeout) {
	case 0:
		val = BQ2597X_SS_TIMEOUT_DISABLE;
		break;
	case 12:
		val = BQ2597X_SS_TIMEOUT_SET_12P5MS;
		break;
	case 25:
		val = BQ2597X_SS_TIMEOUT_SET_25MS;
		break;
	case 50:
		val = BQ2597X_SS_TIMEOUT_SET_50MS;
		break;
	case 100:
		val = BQ2597X_SS_TIMEOUT_SET_100MS;
		break;
	case 400:
		val = BQ2597X_SS_TIMEOUT_SET_400MS;
		break;
	case 1500:
		val = BQ2597X_SS_TIMEOUT_SET_1500MS;
		break;
	case 100000:
		val = BQ2597X_SS_TIMEOUT_SET_100000MS;
		break;
	default:
		val = BQ2597X_SS_TIMEOUT_DISABLE;
		break;
	}

	val <<= BQ2597X_SS_TIMEOUT_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
				BQ2597X_SS_TIMEOUT_SET_MASK,
				val);

	return ret;
}

static int bq2597x_set_ibat_reg_th(struct bq2597x *bq, int th_ma)
{
	int ret;
	u8 val;

	if (th_ma == 200)
		val = BQ2597X_IBAT_REG_200MA;
	else if (th_ma == 300)
		val = BQ2597X_IBAT_REG_300MA;
	else if (th_ma == 400)
		val = BQ2597X_IBAT_REG_400MA;
	else if (th_ma == 500)
		val = BQ2597X_IBAT_REG_500MA;
	else
		val = BQ2597X_IBAT_REG_500MA;

	val <<= BQ2597X_IBAT_REG_SHIFT;
	ret = bq2597x_update_bits(bq, BQ2597X_REG_2C,
				BQ2597X_IBAT_REG_MASK,
				val);

	return ret;
}

static int bq2597x_set_vbat_reg_th(struct bq2597x *bq, int th_mv)
{
	int ret;
	u8 val;

	if (th_mv == 50)
		val = BQ2597X_VBAT_REG_50MV;
	else if (th_mv == 100)
		val = BQ2597X_VBAT_REG_100MV;
	else if (th_mv == 150)
		val = BQ2597X_VBAT_REG_150MV;
	else
		val = BQ2597X_VBAT_REG_200MV;

	val <<= BQ2597X_VBAT_REG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2C,
				BQ2597X_VBAT_REG_MASK,
				val);

	return ret;
}

static void bq2597x_dump_reg(struct bq2597x *bq)
{
	int ret;
	u8 val;
	u8 addr;

	for (addr = 0x00; addr <= 0x2E; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}
}

void bq2597x_dump_reg_g(void)
{
	int ret;
	u8 val;
	u8 addr;

	for (addr = 0x00; addr <= 0x2E; addr++) {
		ret = bq2597x_read_byte(g_chip, addr, &val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}
}

EXPORT_SYMBOL(bq2597x_dump_reg_g);

#if 0
static int bq2597x_check_reg_status(struct bq2597x *bq)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_2C, &val);
	if (!ret) {
		bq->vbat_reg = !!(val & BQ2597X_VBAT_REG_ACTIVE_STAT_MASK);
		bq->ibat_reg = !!(val & BQ2597X_IBAT_REG_ACTIVE_STAT_MASK);
	}

	return ret;
}
#endif

static int bq2597x_detect_device(struct bq2597x *bq)
{
	int ret;
	u8 data;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_13, &data);
	if (ret == 0) {
		bq->part_no = data;
		bq->device_id = (data & BQ2597X_DEV_ID_MASK);
		bq->device_id >>= BQ2597X_DEV_ID_SHIFT;
		bq->revision = (data & BQ2597X_DEV_REV_MASK);
		bq->revision >>= BQ2597X_DEV_REV_SHIFT;

		if (bq->part_no == NU2105_PART_NO) {
			pr_info("detect device PN:%x, ID:%x, REV:%x chip_name:NU2105\n!",
				bq->part_no, bq->device_id, bq->revision);
		} else if ((bq->part_no == SC8551_PART_NO) || (bq->part_no == SC8551A_PART_NO)) {
			pr_info("detect device PN:%x, ID:%x, REV:%x chip_name:SC8551\n!",
				bq->part_no, bq->device_id, bq->revision);
		} else if (bq->part_no == BQ2597X_PART_NO) {
			pr_info("detect device PN:%x, ID:%x, REV:%x chip_name:BQ2597X\n!",
				bq->part_no, bq->device_id, bq->revision);
		} else {
			pr_info("detect device PN:%x, ID:%x, REV:%x chip_name:unknow\n!",
				bq->part_no, bq->device_id, bq->revision);
		}
	}

	return ret;
}

static int bq2597x_parse_dt(struct bq2597x *bq, struct device *dev)
{
	int ret;
	int irqn = 0;
	struct device_node *np = dev->of_node;

	bq->cfg = devm_kzalloc(dev, sizeof(struct bq2597x_cfg),
					GFP_KERNEL);

	if (!bq->cfg) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

#if 0
	if (of_find_property(np, "vdd-i2c-supply", NULL)) {
		bq->vdd_i2c_vreg = devm_regulator_get(bq->dev, "vdd-i2c");
		if (IS_ERR(bq->vdd_i2c_vreg)) {
			pr_err("bq2597x get regulator failed\n");
			return PTR_ERR(bq->vdd_i2c_vreg);
		}
	}
#endif

	bq->cfg->bat_ovp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-disable");
	bq->cfg->bat_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-disable");
	bq->cfg->bat_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-alarm-disable");
	bq->cfg->bat_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-alarm-disable");
	bq->cfg->bus_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-disable");
	bq->cfg->bus_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ovp-alarm-disable");
	bq->cfg->bus_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-alarm-disable");
	bq->cfg->bat_ucp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ucp-alarm-disable");
	bq->cfg->bat_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-therm-disable");
	bq->cfg->bus_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-therm-disable");
	bq->cfg->die_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,die-therm-disable");

	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-threshold",
			&bq->cfg->bat_ovp_th);
	if (ret) {
		pr_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-alarm-threshold",
			&bq->cfg->bat_ovp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-threshold",
			&bq->cfg->bat_ocp_th);
	if (ret) {
		pr_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-alarm-threshold",
			&bq->cfg->bat_ocp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-threshold",
			&bq->cfg->bus_ovp_th);
	if (ret) {
		pr_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-alarm-threshold",
			&bq->cfg->bus_ovp_alm_th);
	if (ret) {
		pr_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-threshold",
			&bq->cfg->bus_ocp_th);
	if (ret) {
		pr_err("failed to read bus-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-alarm-threshold",
			&bq->cfg->bus_ocp_alm_th);
	if (ret) {
		pr_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ucp-alarm-threshold",
			&bq->cfg->bat_ucp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ucp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-therm-threshold",
			&bq->cfg->bat_therm_th);
	if (ret) {
		pr_err("failed to read bat-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-therm-threshold",
			&bq->cfg->bus_therm_th);
	if (ret) {
		pr_err("failed to read bus-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,die-therm-threshold",
			&bq->cfg->die_therm_th);
	if (ret) {
		pr_err("failed to read die-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2597x,ac-ovp-threshold",
			&bq->cfg->ac_ovp_th);
	if (ret) {
		pr_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	bq->irq_gpio.gpio = of_get_named_gpio(bq->dev->of_node, "bq2597x,irq-gpio", 0);

	if (!gpio_is_valid(bq->irq_gpio.gpio)) {

		pr_err("Invalid gpio irq int=%d\n", bq->irq_gpio.gpio);
		return -EINVAL;
	}

	ret = gpio_request(bq->irq_gpio.gpio, "bq2597x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, bq->irq_gpio.gpio);
		return ret;
	}
	gpio_direction_input(bq->irq_gpio.gpio);
	irqn = gpio_to_irq(bq->irq_gpio.gpio);

	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	bq->client->irq = irqn;

	ret = of_property_read_u32(np, "ti,bq2597x,sense-resistor-mohm",
			&bq->cfg->sense_r_mohm);
	if (ret) {
		pr_err("failed to read sense-resistor-mohm\n");
		return ret;
	}

	return 0;
}

static int bq2597x_init_protection(struct bq2597x *bq)
{
	int ret;

	ret = bq2597x_enable_batovp(bq, !bq->cfg->bat_ovp_disable);
	pr_info("%s bat ovp %s\n",
		bq->cfg->bat_ovp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp(bq, !bq->cfg->bat_ocp_disable);
	pr_info("%s bat ocp %s\n",
		bq->cfg->bat_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batovp_alarm(bq, !bq->cfg->bat_ovp_alm_disable);
	pr_info("%s bat ovp alarm %s\n",
		bq->cfg->bat_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp_alarm(bq, !bq->cfg->bat_ocp_alm_disable);
	pr_info("%s bat ocp alarm %s\n",
		bq->cfg->bat_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batucp_alarm(bq, !bq->cfg->bat_ucp_alm_disable);
	pr_info("%s bat ocp alarm %s\n",
		bq->cfg->bat_ucp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busovp_alarm(bq, !bq->cfg->bus_ovp_alm_disable);
	pr_info("%s bus ovp alarm %s\n",
		bq->cfg->bus_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp(bq, !bq->cfg->bus_ocp_disable);
	pr_info("%s bus ocp %s\n",
		bq->cfg->bus_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp_alarm(bq, !bq->cfg->bus_ocp_alm_disable);
	pr_info("%s bus ocp alarm %s\n",
		bq->cfg->bus_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bat_therm(bq, !bq->cfg->bat_therm_disable);
	pr_info("%s bat therm %s\n",
		bq->cfg->bat_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bus_therm(bq, !bq->cfg->bus_therm_disable);
	pr_info("%s bus therm %s\n",
		bq->cfg->bus_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_die_therm(bq, !bq->cfg->die_therm_disable);
	pr_info("%s die therm %s\n",
		bq->cfg->die_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_set_batovp_th(bq, bq->cfg->bat_ovp_th);
	pr_info("set bat ovp th %d %s\n", bq->cfg->bat_ovp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batovp_alarm_th(bq, bq->cfg->bat_ovp_alm_th);
	pr_info("set bat ovp alarm threshold %d %s\n", bq->cfg->bat_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_th(bq, bq->cfg->bat_ocp_th);
	pr_info("set bat ocp threshold %d %s\n", bq->cfg->bat_ocp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_alarm_th(bq, bq->cfg->bat_ocp_alm_th);
	pr_info("set bat ocp alarm threshold %d %s\n", bq->cfg->bat_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_th(bq, bq->cfg->bus_ovp_th);
	pr_info("set bus ovp threshold %d %s\n", bq->cfg->bus_ovp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_alarm_th(bq, bq->cfg->bus_ovp_alm_th);
	pr_info("set bus ovp alarm threshold %d %s\n", bq->cfg->bus_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_th(bq, bq->cfg->bus_ocp_th);
	pr_info("set bus ocp threshold %d %s\n", bq->cfg->bus_ocp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_alarm_th(bq, bq->cfg->bus_ocp_alm_th);
	pr_info("set bus ocp alarm th %d %s\n", bq->cfg->bus_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batucp_alarm_th(bq, bq->cfg->bat_ucp_alm_th);
	pr_info("set bat ucp threshold %d %s\n", bq->cfg->bat_ucp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_bat_therm_th(bq, bq->cfg->bat_therm_th);
	pr_info("set die therm threshold %d %s\n", bq->cfg->bat_therm_th,
		!ret ? "successfully" : "failed");
	ret = bq2597x_set_bus_therm_th(bq, bq->cfg->bus_therm_th);
	pr_info("set bus therm threshold %d %s\n", bq->cfg->bus_therm_th,
		!ret ? "successfully" : "failed");
	ret = bq2597x_set_die_therm_th(bq, bq->cfg->die_therm_th);
	pr_info("set die therm threshold %d %s\n", bq->cfg->die_therm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_acovp_th(bq, bq->cfg->ac_ovp_th);
	pr_info("set ac ovp threshold %d %s\n", bq->cfg->ac_ovp_th,
		!ret ? "successfully" : "failed");

	return 0;
}

static int bq2597x_init_adc(struct bq2597x *bq)
{

	bq2597x_set_adc_scanrate(bq, false);
	bq2597x_set_adc_bits(bq, 13);
	bq2597x_set_adc_average(bq, true);
	bq2597x_set_adc_scan(bq, ADC_IBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VOUT, false);
	bq2597x_set_adc_scan(bq, ADC_VBAT, true);
	bq2597x_set_adc_scan(bq, ADC_IBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TBUS, true);
	bq2597x_set_adc_scan(bq, ADC_TBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TDIE, true);
	bq2597x_set_adc_scan(bq, ADC_VAC, true);

	bq2597x_enable_adc(bq, true);
	return 0;
}

static int bq2597x_init_int_src(struct bq2597x *bq)
{
	int ret;
	/*TODO:be careful ts bus and ts bat alarm bit mask is in
	 *	fault mask register, so you need call
	 *	bq2597x_set_fault_int_mask for tsbus and tsbat alarm*/
	ret = bq2597x_set_alarm_int_mask(bq, ADC_DONE
		/*			| BAT_UCP_ALARM*/
					| BAT_OVP_ALARM);
	if (ret) {
		pr_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
#if 0
	ret = bq2597x_set_fault_int_mask(bq, TS_BUS_FAULT);
	if (ret) {
		pr_err("failed to set fault mask:%d\n", ret);
		return ret;
	}
#endif
	return ret;
}

static int sc8551_init_adc_trim(struct bq2597x *bq)
{
	int ret;

	ret = bq2597x_update_bits(bq, SC8551_REG_34,
				SC8551_ADC_TRIM_MASK, SC8551_ADC_TRIM_VAL);
	return ret;
}

static int sc8551_set_ibus_low_dg(struct bq2597x *bq, int low_dg)
{
	int ret;
	u8 val;

	if (low_dg == 10)
		val = BQ2597X_IBUS_LOW_DG_10US;
	else
		val = BQ2597X_IBUS_LOW_DG_5MS;

	val <<= BQ2597X_IBUS_LOW_DG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2E,
				BQ2597X_IBUS_LOW_DG_MASK,
				val);

	return ret;
}

static int bq2597x_init_regulation(struct bq2597x *bq)
{
	bq2597x_set_ibat_reg_th(bq, 300);
	bq2597x_set_vbat_reg_th(bq, 100);

	bq2597x_set_vdrop_deglitch(bq, 5000);
	bq2597x_set_vdrop_th(bq, 400);

	if (bq->part_no == NU2105_PART_NO) {
		bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_FREQ_SHIFT_MASK, (0x02 << BQ2597X_FREQ_SHIFT_SHIFT));
	}
	bq2597x_enable_regulation(bq, false);

	return 0;
}

static int bq2597x_enable_vout(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_VOUT_OVP_ENABLE;
	else
		val = BQ2597X_VOUT_OVP_DISABLE;

	val <<= BQ2597X_VOUT_OVP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
				BQ2597X_VOUT_OVP_DIS_MASK,
				val);

	return ret;

}

static int bq2597x_init_device(struct bq2597x *bq)
{
	bq2597x_enable_wdt(bq, false);


	bq2597x_set_ss_timeout(bq, 0);
	bq2597x_set_sense_resistor(bq, bq->cfg->sense_r_mohm);
	bq2597x_set_ucp_threshold(bq, 300);

	bq2597x_init_protection(bq);
	bq2597x_init_adc(bq);
	bq2597x_init_int_src(bq);

	bq2597x_init_regulation(bq);

	bq2597x_enable_vout(bq, false);

	if (bq->part_no == SC8551_PART_NO
		||bq->part_no == SC8551A_PART_NO) {
		sc8551_init_adc_trim(bq);
		sc8551_set_ibus_low_dg(bq, 5000);
	}
	return 0;
}

static int bq2597x_set_present(struct bq2597x *bq, bool present)
{
	bq->usb_present = present;

	if (present)
		bq2597x_init_device(bq);
	return 0;
}

static ssize_t bq2597x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2597x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;
	u8 max_addr = 0;

	if (bq->part_no == NU2105_PART_NO) {
		max_addr = NU2105_MAX_SHOW_REG_ADDR;
		idx = snprintf(buf, PAGE_SIZE, "%s:\n", "nu2105");
	} else if ((bq->part_no == SC8551A_PART_NO) || (bq->part_no == SC8551_PART_NO)) {
		max_addr = SC8551_MAX_SHOW_REG_ADDR;
		idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8551");
	} else {
		max_addr = BQ25970_MAX_SHOW_REG_ADDR;
		idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25970");
	}
	for (addr = 0x0; addr <= 0x2E; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2597x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2597x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x2E)
		bq2597x_write_byte(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, bq2597x_show_registers, bq2597x_store_register);

static struct attribute *bq2597x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2597x_attr_group = {
	.attrs = bq2597x_attributes,
};

static enum power_supply_property bq2597x_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static void bq2597x_check_alarm_status(struct bq2597x *bq);
static void bq2597x_check_fault_status(struct bq2597x *bq);

static int bq2597x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2597x *bq  = power_supply_get_drvdata(psy);
	int ret;
	int result;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->vbus_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq2597x_get_adc_data(bq, ADC_VBAT, &result);
		if (!ret)
			bq->vbat_volt = result;

		val->intval = bq->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq2597x_get_adc_data(bq, ADC_IBAT, &result);
		if (!ret)
			bq->ibat_curr = result;

		val->intval = bq->ibat_curr;
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static int bq2597x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2597x *bq  = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		bq2597x_set_present(bq, !!val->intval);
		pr_info("set present :%d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2597x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	default:
		ret = 0;
		break;
	}
	return ret;
}

static const struct power_supply_desc bq2597x_psy_desc = {
	.name = "cp-standalone",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = bq2597x_charger_props,
	.num_properties = ARRAY_SIZE(bq2597x_charger_props),
	.get_property = bq2597x_charger_get_property,
	.set_property = bq2597x_charger_set_property,
	.property_is_writeable = bq2597x_charger_is_writeable,
};

static int bq2597x_psy_register(struct bq2597x *bq)
{
	struct power_supply_config bq2597x_cfg = {};

	bq2597x_cfg.drv_data = bq;
	bq2597x_cfg.of_node = bq->dev->of_node;
	bq->fc2_psy = power_supply_register(bq->dev, &bq2597x_psy_desc, &bq2597x_cfg);
	if (IS_ERR(bq->fc2_psy)) {
		pr_err("Couldn't register bq2597x power supply\n");
		return PTR_ERR(bq->fc2_psy);
	}

	pr_info("power supply register bq2597x successfully\n");
	return 0;
}

EXPORT_SYMBOL_GPL(bq2597x_dump_reg);


static int bq2597x_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct bq2597x *bq = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		bq2597x_enable_charge(bq, val1);
		bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
		pr_info("PSY_IIO_CP_ENABLE: %s\n",
				val1 ? "enable" : "disable");
		break;
	case PSY_IIO_ONLINE:
		bq2597x_set_present(bq, !!val1);
		pr_info("set present :%d\n", val1);
		break;
	case PSY_IIO_CP_CLEAR_ERROR:
		bq->bat_ovp_fault = false;
		bq->bat_ocp_fault = false;
		bq->bus_ovp_fault = false;
		bq->bus_ocp_fault = false;
		break;
	default:
		pr_err("Unsupported bq2597x IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int bq2597x_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct bq2597x *bq = iio_priv(indio_dev);
	int rc = 0;
	int result;
	*val1 = 0;

	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
		*val1 = bq->charge_enabled;
		pr_info("read bq2597x enable:%d\n",*val1);
		break;
	case PSY_IIO_ONLINE:
		*val1 = bq->vbus_present;
		break;
	case PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW:
		rc = bq2597x_get_adc_data(bq, ADC_VBUS, &result);
		if (!rc)
			bq->vbus_volt = result;

		*val1 = bq->vbus_volt;
		break;
	case PSY_IIO_MMI_CP_INPUT_CURRENT_NOW:
		rc = bq2597x_get_adc_data(bq, ADC_IBUS, &result);
		if (!rc)
			bq->ibus_curr = result;

		*val1 = bq->ibus_curr;
		break;
	case PSY_IIO_CP_STATUS1:
		bq2597x_check_alarm_status(bq);
		bq2597x_check_fault_status(bq);
		*val1 = ((bq->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
			| (bq->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
			| (bq->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
			| (bq->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
			| (bq->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
			| (bq->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
			| (bq->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
			| (bq->die_therm_alarm << DIE_THERM_ALARM_SHIFT)
			| (bq->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
			| (bq->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
			| (bq->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
			| (bq->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
			| (bq->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
			| (bq->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
			| (bq->die_therm_fault << DIE_THERM_FAULT_SHIFT));
		break;

	default:
		pr_err("Unsupported bq2597x IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int bq2597x_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct bq2597x *bq = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = bq->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(bq2597x_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info bq2597x_iio_info = {
	.read_raw	= bq2597x_iio_read_raw,
	.write_raw	= bq2597x_iio_write_raw,
	.of_xlate	= bq2597x_iio_of_xlate,
};

static int bq2597x_init_iio_psy(struct bq2597x *chip)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int bq2597x_num_iio_channels = ARRAY_SIZE(bq2597x_iio_psy_channels);
	int rc, i;

	chip->iio_chan = devm_kcalloc(chip->dev, bq2597x_num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				bq2597x_num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &bq2597x_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->name = "bq2597x-charger";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = bq2597x_num_iio_channels;

	for (i = 0; i < bq2597x_num_iio_channels; i++) {
		chip->int_iio_chans[i].indio_dev = indio_dev;
		chan = &chip->iio_chan[i];
		chip->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = bq2597x_iio_psy_channels[i].channel_num;
		chan->type = bq2597x_iio_psy_channels[i].type;
		chan->datasheet_name =
			bq2597x_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			bq2597x_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			bq2597x_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("Failed to register bq2597x IIO device, rc=%d\n", rc);

	return rc;
}

static void bq2597x_check_alarm_status(struct bq2597x *bq)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	bool changed = false;

	mutex_lock(&bq->data_lock);
	ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &flag);
	if (!ret && (flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK))
		pr_debug("UCP_FLAG =0x%02X\n", !!(flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK));


	ret = bq2597x_read_byte(bq, BQ2597X_REG_2D, &flag);
	if (!ret && (flag & BQ2597X_VDROP_OVP_FLAG_MASK))
		pr_debug("VDROP_OVP_FLAG =0x%02X\n", !!(flag & BQ2597X_VDROP_OVP_FLAG_MASK));
	/*read to clear alarm flag*/
	ret = bq2597x_read_byte(bq, BQ2597X_REG_0E, &flag);
	if (!ret && flag)
		pr_debug("INT_FLAG =0x%02X\n", flag);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &stat);
	if (!ret && stat != bq->prev_alarm) {
		pr_debug("INT_STAT = 0X%02x\n", stat);
		changed = true;
		bq->prev_alarm = stat;
		bq->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
		bq->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
		bq->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
		bq->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
		bq->batt_present  = !!(stat & VBAT_INSERT);
		bq->vbus_present  = !!(stat & VBUS_INSERT);
		bq->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
	}

	ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &stat);
	if (!ret && (stat & 0x50))
		pr_err("Reg[05]BUS_UCPOVP = 0x%02X\n", stat);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0A, &stat);
	if (!ret && (stat & 0x02))
		pr_err("Reg[0A]CONV_OCP = 0x%02X\n", stat);

	mutex_unlock(&bq->data_lock);

	if (changed)
		power_supply_changed(bq->fc2_psy);

}

static void bq2597x_check_fault_status(struct bq2597x *bq)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	bool changed = false;

	mutex_lock(&bq->data_lock);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_10, &stat);
	if (!ret && stat)
		pr_err("FAULT_STAT = 0x%02X\n", stat);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_11, &flag);
	if (!ret && flag)
		pr_err("FAULT_FLAG = 0x%02X\n", flag);

	if (!ret && flag != bq->prev_fault) {
		changed = true;
		bq->prev_fault = flag;
		bq->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		bq->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		bq->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		bq->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		bq->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		bq->bus_therm_fault = !!(flag & TS_BUS_FAULT);

		bq->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		bq->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

	mutex_unlock(&bq->data_lock);

	if (changed)
		power_supply_changed(bq->fc2_psy);

}

static irqreturn_t bq2597x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2597x *bq = dev_id;

	pr_debug("interrupt enter -\n");
	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	/* TODO */
	bq2597x_check_alarm_status(bq);
	bq2597x_check_fault_status(bq);

	bq2597x_dump_reg(bq);
	mutex_unlock(&bq->irq_complete);

	pr_err("stat=0x%02X, fault flag =0x%02X\n", bq->prev_alarm, bq->prev_fault);

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq2597x *bq)
{
	bq2597x_charger_interrupt(bq->client->irq, bq);
}

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2597x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x2B; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			seq_printf(m, "Reg-[%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2597x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2597x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2597x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {
		debugfs_create_file("registers",
					S_IFREG | S_IRUGO,
					bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("skip_reads",
					S_IFREG | S_IWUSR | S_IRUGO,
					bq->debug_root,
					&(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					S_IFREG | S_IWUSR | S_IRUGO,
					bq->debug_root,
					&(bq->skip_writes));
	}
}

static const struct regmap_config bq2597x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static ssize_t force_chg_auto_enable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long  r;
	bool mode;

	struct bq2597x *bq = dev_get_drvdata(dev->parent);
	if (!bq) {
		pr_err("bq2597x_mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtobool(buf, &mode);
	if (r) {
		pr_err("bq2597x_mmi_charger: Invalid chrg enable value = %d\n", mode);
		return -EINVAL;
	}

	r = bq2597x_enable_charge(bq, mode);
	if (r < 0) {
		pr_err("bq2597x_mmi_charger Couldn't %s charging rc=%d\n",
			   mode ? "enable" : "disable", (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_auto_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	int state = 0;

	struct bq2597x *bq = dev_get_drvdata(dev->parent);

	if (!bq) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = bq2597x_check_charge_enabled(bq, &bq->charge_enabled);

	state = bq->charge_enabled;

end:
	return scnprintf(buf, AUTO_ENABLE_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_auto_enable, 0664,
		force_chg_auto_enable_show,
		force_chg_auto_enable_store);

static struct attribute *bq2597x_force_attributes[] = {
	&dev_attr_force_chg_auto_enable.attr,
	NULL,
};

static const struct attribute_group bq2597x_attribute_group = {
	.attrs = bq2597x_force_attributes,
};

static int bq2597x_create_sys(struct device *dev, const struct attribute_group * grp)
{
	int err = 0;
	struct power_supply *cp_ply;

	pr_err("enter bq2597x create sys \n");
	cp_ply = power_supply_get_by_name("cp-standalone");

	if(NULL == dev){
		pr_err("[BATT]: failed to register battery\n");
		return -EINVAL;
	}

	if(cp_ply)
	{
		err = sysfs_create_group(&cp_ply->dev.kobj, grp);

		if (!err)
		{
			pr_info("creat BMT sysfs group ok\n");
		}
		else
		{
			pr_err("creat BMT sysfs group fail\n");
			err =  -EIO;
		}
		power_supply_put(cp_ply);
	}
	else
	{
		pr_err("don't have /sys/class/power_supply/cp-standalone dir\n");
	}

	return err;
}

static int bq2597x_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq2597x *bq;
	int ret;
	struct iio_dev *indio_dev;
	//const struct of_device_id *match;
	//struct device_node *node = client->dev.of_node;
	pr_err("bq2597x_charger_probe start\n");
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bq));


	if (!indio_dev)
		return -ENOMEM;

	bq = iio_priv(indio_dev);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}
	bq->indio_dev = indio_dev;

	bq->dev = &client->dev;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	bq->regmap = regmap_init_i2c(client, &bq2597x_regmap_config);
	if (IS_ERR(bq->regmap)) {
		pr_err("Couldn't initialize register regmap rc = %ld\n",
				PTR_ERR(bq->regmap));
		ret = PTR_ERR(bq->regmap);
		goto free_mem;
	}


	ret = bq2597x_init_iio_psy(bq);
	if (ret < 0) {
		pr_err("Failed to initialize bq2597x IIO PSY, ret = %d\n", ret);
		return ret;
	}

	if (client->dev.of_node) {
		ret = bq2597x_parse_dt(bq, &client->dev);
		if (ret)
			goto free_mem;
	}

	ret = bq2597x_detect_device(bq);
	if (ret) {
		pr_err("No bq2597x device found!\n");
		goto free_mem;
	}

	ret = bq2597x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		goto free_mem;
	}

	ret = bq2597x_psy_register(bq);
	if (ret) {
		pr_err("Failed to register psy\n");
		goto free_mem;
	}
/*
	bq->irq_pinctrl =
		pinctrl_get_select(bq->dev, "bq_int_default");
	if (!bq->irq_pinctrl) {
		pr_err("Couldn't set pinctrl bq_int_default\n");
		goto free_psy;
	}
*/
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2597x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2597x charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n",
							client->irq, ret);
			goto free_gpio;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);
	create_debugfs_entry(bq);
	ret = sysfs_create_group(&bq->dev->kobj, &bq2597x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto free_irq;
	}

	ret = bq2597x_create_sys(&(client->dev), &bq2597x_attribute_group);
	if(ret){
		pr_err("[BATT]: Err failed to creat BMT attributes\n");
		goto free_irq;
	}

	determine_initial_status(bq);

	pr_info("bq2597x probe successfully-, Part Num:%d\n!",
				bq->part_no);
	g_chip = bq;
	return 0;

free_irq:
	devm_free_irq(&client->dev,client->irq,bq);
	power_supply_unregister(bq->fc2_psy);
free_gpio:
	gpio_free(bq->irq_gpio.gpio);
	power_supply_unregister(bq->fc2_psy);
#if 0
free_psy:
	power_supply_unregister(bq->fc2_psy);

disable_regator:
	regulator_disable(bq->vdd_i2c_vreg);
free_regulator:
	devm_regulator_put(bq->vdd_i2c_vreg);
#endif
free_mem:
	devm_kfree(bq->dev, bq);
	return ret;
}

static inline bool is_device_suspended(struct bq2597x *bq)
{
	return !bq->resume_completed;
}

static int bq2597x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	pr_err("Suspend successfully!");

	return 0;
}

static int bq2597x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2597x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);

	pr_err("Resume begin");
	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2597x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->fc2_psy);
	pr_err("Resume successfully!");

	return 0;
}
static int bq2597x_charger_remove(struct i2c_client *client)
{
	struct bq2597x *bq = i2c_get_clientdata(client);

	bq2597x_enable_adc(bq, false);

	//devm_regulator_put(bq->vdd_i2c_vreg);
	devm_free_irq(&client->dev,client->irq,bq);
	gpio_free(bq->irq_gpio.gpio);
	power_supply_unregister(bq->fc2_psy);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	debugfs_remove_recursive(bq->debug_root);

	sysfs_remove_group(&bq->dev->kobj, &bq2597x_attr_group);
	sysfs_remove_group(&(bq->client->dev.kobj), &bq2597x_attribute_group);

	return 0;
}

static void bq2597x_charger_shutdown(struct i2c_client *client)
{
	struct bq2597x *bq = i2c_get_clientdata(client);
	bq2597x_enable_adc(bq, false);
	pr_info("Shutdown Successfully\n");
}

static struct of_device_id bq2597x_charger_match_table[] = {
	{.compatible = "ti,bq25970-charger",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2597x_charger_match_table);

static const struct i2c_device_id bq2597x_charger_id[] = {
	{ "bq25970-charger", BQ25970 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2597x_charger_id);

static const struct dev_pm_ops bq2597x_pm_ops = {
	.resume		= bq2597x_resume,
	.suspend_noirq = bq2597x_suspend_noirq,
	.suspend	= bq2597x_suspend,
};
static struct i2c_driver bq2597x_charger_driver = {
	.driver		= {
		.name	= "bq2597x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2597x_charger_match_table,
		.pm	= &bq2597x_pm_ops,
	},
	.id_table	= bq2597x_charger_id,

	.probe		= bq2597x_charger_probe,
	.remove		= bq2597x_charger_remove,
	.shutdown	= bq2597x_charger_shutdown,
};

module_i2c_driver(bq2597x_charger_driver);

MODULE_DESCRIPTION("TI BQ2597x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
