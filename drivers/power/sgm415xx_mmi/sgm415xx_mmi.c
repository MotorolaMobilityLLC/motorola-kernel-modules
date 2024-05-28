// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/iio/consumer.h>

#include "sgm415xx.h"
#include "charger_class.h"
#include "mtk_charger.h"
#if IS_ENABLED(CONFIG_OEM_DEVINFO)
#include <dev_info.h>
#endif
#include <linux/iio/consumer.h>
#include <linux/phy/phy.h>

/**********************************************************
 *
 *   [I2C Slave Setting]
 *
 *********************************************************/
#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

#define SGM4154x_REG_NUM	(0xF)
#define SINGLE_DUMP_LEN		22
#define TOTAL_DUMP_LEN		(SINGLE_DUMP_LEN * (SGM4154x_REG_NUM))

#define R_VBUS_CHARGER_1   330
#define R_VBUS_CHARGER_2   39

#if IS_ENABLED(CONFIG_FACTORY_BUILD)
extern int factory_charging_limit;
#endif

static struct proc_dir_entry *entry;
static bool dump_reg_enable;
static bool allow_set_dp_dm_vol = false;

#if IS_ENABLED(CONFIG_OEM_TURBO_CHARGER)
extern bool turbo_charger_active;
extern bool ffc_batt_full;
#endif

enum vindpm_track {
	SGM4154x_TRACK_DIS,
	SGM4154x_TRACK_200,
	SGM4154x_TRACK_250,
	SGM4154x_TRACK_300,
};

/* SGM4154x REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000
};

/* SGM4154x REG02 BOOST_LIM[7:7], uA */
#if (defined(__SGM41542_CHIP_ID__) || defined(__SGM41541_CHIP_ID__)|| defined(__SGM41543_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))

static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};

static const struct charger_properties sgm4154x_chg_props = {
	.alias_name = SGM4154x_NAME,
};

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
static struct power_supply_desc sgm4154x_power_supply_desc;
static struct charger_device *s_chg_dev_otg;

/**********************************************************
 *
 *   [I2C Function For Read/Write sgm4154x]
 *
 *********************************************************/
static int __sgm4154x_read_byte(struct sgm4154x_device *sgm, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(sgm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;
}

static int __sgm4154x_write_byte(struct sgm4154x_device *sgm, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int sgm4154x_read_reg(struct sgm4154x_device *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

__maybe_unused static int sgm4154x_write_reg(struct sgm4154x_device *sgm, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_write_byte(sgm, reg, val);
	mutex_unlock(&sgm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int sgm4154x_update_bits(struct sgm4154x_device *sgm, u8 reg,
	u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_byte(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	ret = __sgm4154x_write_byte(sgm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
 int Charger_Detect_Init(struct sgm4154x_device *sgm)
{
	struct phy *phy;
	int ret;
	phy = phy_get(sgm->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_err(sgm->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}
	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_BC11_SET);
	dev_err(sgm->dev, "%s\n", __func__);
	if (ret)
		dev_err(sgm->dev, "failed to set phy ext mode\n");
	phy_put(sgm->dev, phy);
	return ret;
}

int Charger_Detect_Release(struct sgm4154x_device *sgm)
{
	struct phy *phy;
	int ret;
	phy = phy_get(sgm->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_err(sgm->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}
	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_BC11_CLR);
	dev_err(sgm->dev, "%s\n", __func__);
	if (ret)
		dev_err(sgm->dev, "failed to set phy ext mode\n");
	phy_put(sgm->dev, phy);
	return ret;
}

static int sgm4154x_set_watchdog_timer(struct sgm4154x_device *sgm, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = SGM4154x_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = SGM4154x_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = SGM4154x_WDT_TIMER_80S;
	else
		reg_val = SGM4154x_WDT_TIMER_160S;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
			SGM4154x_WDT_TIMER_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_vindpm_track(struct sgm4154x_device *sgm, enum vindpm_track track)
{
	int ret;
	dev_err(sgm->dev, "start vindpm track\n");
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7,
			SGM4154x_VINDPM_TRACK, track);

	return ret;
}

static int sgm4154x_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	int ret, value;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = iio_read_channel_processed(sgm->vbus, &value);
	if (ret < 0) {
		dev_err(sgm->dev, "get vbus voltage failed");
		return -EINVAL;
	}
	*vbus = value + R_VBUS_CHARGER_1 * value / R_VBUS_CHARGER_2;
 	*vbus = *vbus * 1000;
	dev_info(sgm->dev, "vbus voltage: %d", *vbus);
	return ret;
}

static int sgm4154x_set_tmr2x(struct sgm4154x_device *sgm, bool enable)
{
	int ret;
	int reg_val = enable ? 1 : 0;
	dev_err(sgm->dev, "start set tmr2x\n");

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7,
			SGM4154x_SAFETY_TIMER_RM2X, reg_val);

	return ret;
}

static int sgm4154x_set_dpm_mask(struct sgm4154x_device *sgm)
{
	int ret;
	dev_err(sgm->dev, "start dpm mask\n");
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_a,
			SGM4154x_DPM_MASK, SGM4154x_DPM_MASK);

	return ret;
}

__maybe_unused static int sgm4154x_get_term_curr(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM4154x_TERMCHRG_I_MIN_uA;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val &= SGM4154x_TERMCHRG_CUR_MASK;
	curr = reg_val * SGM4154x_TERMCHRG_CURRENT_STEP_uA + offset;
	return curr;
}

__maybe_unused static int sgm4154x_get_prechrg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM4154x_PRECHRG_I_MIN_uA;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val & SGM4154x_PRECHRG_CUR_MASK) >> 4;
	curr = reg_val * SGM4154x_PRECHRG_CURRENT_STEP_uA + offset;

	return curr;
}

static int sgm4154x_set_chg_term(struct sgm4154x_device *sgm, bool en)
{
	int reg_val = -1;

	reg_val = en <<  7;
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
					SGM4154x_TERM_EN, reg_val);
}

static int sgm4154x_enable_terminate(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	ret = sgm4154x_set_chg_term(sgm, en);
	if (ret < 0)
		dev_err(sgm->dev, "%s failed ret(%d)\n", __func__, ret);

	return ret;
}

static int sgm4154x_set_term_curr(struct charger_device *chg_dev, u32 uA)
{
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for (reg_val = 1; reg_val < 16 && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_TERMCHRG_I_MIN_uA)
		uA = SGM4154x_TERMCHRG_I_MIN_uA;
	else if (uA > SGM4154x_TERMCHRG_I_MAX_uA)
		uA = SGM4154x_TERMCHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_TERMCHRG_I_MIN_uA) / SGM4154x_TERMCHRG_CURRENT_STEP_uA;
#endif
	dev_info(sgm->dev, "%s: iterm curr = %d uA\n", __func__, uA);
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_3,
			SGM4154x_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_prechrg_curr(struct sgm4154x_device *sgm, int uA)
{
	u8 reg_val;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for(reg_val = 1; reg_val < 16 && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_PRECHRG_I_MIN_uA)
		uA = SGM4154x_PRECHRG_I_MIN_uA;
	else if (uA > SGM4154x_PRECHRG_I_MAX_uA)
		uA = SGM4154x_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_PRECHRG_I_MIN_uA) / SGM4154x_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << 4;
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_3,
			SGM4154x_PRECHRG_CUR_MASK, reg_val);
}

static int sgm4154x_get_ichg_curr(struct charger_device *chg_dev, u32 *uA)
{
	int ret;
	u8 ichg;
	u32 curr;

	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;

	ichg &= SGM4154x_ICHRG_I_MASK;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (ichg <= 0x8)
		curr = ichg * 5000;
	else if (ichg <= 0xF)
		curr = 40000 + (ichg - 0x8) * 10000;
	else if (ichg <= 0x17)
		curr = 110000 + (ichg - 0xF) * 20000;
	else if (ichg <= 0x20)
		curr = 270000 + (ichg - 0x17) * 30000;
	else if (ichg <= 0x30)
		curr = 540000 + (ichg - 0x20) * 60000;
	else if (ichg <= 0x3C)
		curr = 1500000 + (ichg - 0x30) * 120000;
	else
		curr = 3000000;
#else
	curr = ichg * SGM4154x_ICHRG_I_STEP_uA;
#endif
	*uA = curr;

	return 0;
}

static int sgm4154x_get_minichg_curr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = SGM4154x_ICHRG_I_MIN_uA;
	return 0;
}

static int sgm4154x_set_ichrg_curr(struct charger_device *chg_dev, unsigned int uA)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (uA < SGM4154x_ICHRG_I_MIN_uA)
		uA = SGM4154x_ICHRG_I_MIN_uA;
	else if ( uA > sgm->init_data.max_ichg)
		uA = sgm->init_data.max_ichg;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (uA <= 40000)
		reg_val = uA / 5000;
	else if (uA <= 110000)
		reg_val = 0x08 + (uA -40000) / 10000;
	else if (uA <= 270000)
		reg_val = 0x0F + (uA -110000) / 20000;
	else if (uA <= 540000)
		reg_val = 0x17 + (uA -270000) / 30000;
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA -540000) / 60000;
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA -1500000) / 120000;
	else
		reg_val = 0x3d;
#else
	reg_val = uA / SGM4154x_ICHRG_I_STEP_uA;
#endif
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2,
			SGM4154x_ICHRG_I_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_chrg_volt(struct charger_device *chg_dev, u32 chrg_volt)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (chrg_volt < SGM4154x_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;

	reg_val = (chrg_volt - SGM4154x_VREG_V_MIN_uV) / SGM4154x_VREG_V_STEP_uV;
	reg_val = reg_val << 3;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_4,
			SGM4154x_VREG_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_get_chrg_volt(struct charger_device *chg_dev,unsigned int *volt)
{
	int ret;
	u8 vreg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & SGM4154x_VREG_V_MASK) >> 3;

	if (15 == vreg_val)
		*volt = 4352000; //default
	else if (vreg_val < 25)
		*volt = vreg_val * SGM4154x_VREG_V_STEP_uV + SGM4154x_VREG_V_MIN_uV;

	return 0;
}

static int sgm4154x_get_vindpm_offset_os(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;

	reg_val = reg_val & SGM4154x_VINDPM_OS_MASK;

	return reg_val;
}

static int sgm4154x_set_vindpm_offset_os(struct sgm4154x_device *sgm,u8 offset_os)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_f,
			SGM4154x_VINDPM_OS_MASK, offset_os);

	if (ret) {
		pr_err("%s fail\n",__func__);
		return ret;
	}

	return ret;
}

static int sgm4154x_set_input_volt_lim(struct charger_device *chg_dev, unsigned int vindpm)
{
	int ret;
	unsigned int offset;
	u8 reg_val;
	u8 os_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (vindpm < SGM4154x_VINDPM_V_MIN_uV ||
		vindpm > SGM4154x_VINDPM_V_MAX_uV)
 		return -EINVAL;

	if (vindpm < 5900000) {
		os_val = 0;
		offset = 3900000;
	} else if (vindpm >= 5900000 && vindpm < 7500000) {
		os_val = 1;
		offset = 5900000; //uv
	} else if (vindpm >= 7500000 && vindpm < 10500000) {
		os_val = 2;
		offset = 7500000; //uv
	} else {
		os_val = 3;
		offset = 10500000; //uv
	}

	sgm4154x_set_vindpm_offset_os(sgm,os_val);
	reg_val = (vindpm - offset) / SGM4154x_VINDPM_STEP_uV;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_6,
			SGM4154x_VINDPM_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_get_input_volt_lim(struct charger_device *chg_dev, u32 *uV)
{
	int ret;
	int offset;
	u8 vlim;
	int temp;

	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;

	temp = sgm4154x_get_vindpm_offset_os(sgm);
	if (0 == temp)
		offset = 3900000; //uv
	else if (1 == temp)
		offset = 5900000;
	else if (2 == temp)
		offset = 7500000;
	else if (3 == temp)
		offset = 10500000;
	else
		return temp;

	*uV = offset + (vlim & 0x0F) * SGM4154x_VINDPM_STEP_uV;

	return 0;
}

__maybe_unused static int sgm4154x_get_input_minvolt_lim(struct charger_device *chg_dev, u32 *uV)
{
	*uV = SGM4154x_VINDPM_V_MIN_uV;

	return 0;
}

static int sgm4154x_set_input_curr_lim(struct charger_device *chg_dev, unsigned int iindpm)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (iindpm < SGM4154x_IINDPM_I_MIN_uA ||
		iindpm > SGM4154x_IINDPM_I_MAX_uA)
		return -EINVAL;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	reg_val = (iindpm - SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
#else
	if (iindpm >= SGM4154x_IINDPM_I_MIN_uA && iindpm <= 3100000) //default
		reg_val = (iindpm - SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = SGM4154x_IINDPM_I_MASK;
#endif
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_0,
			SGM4154x_IINDPM_I_MASK, reg_val);

	return ret;
}

static int sgm4154x_get_input_curr_lim(struct charger_device *chg_dev,unsigned int *ilim)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;

	if (SGM4154x_IINDPM_I_MASK == (reg_val & SGM4154x_IINDPM_I_MASK))
		*ilim =  SGM4154x_IINDPM_I_MAX_uA;
	else
		*ilim = (reg_val & SGM4154x_IINDPM_I_MASK) * SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;

	return 0;
}

static int sgm4154x_get_input_mincurr_lim(struct charger_device *chg_dev,u32 *ilim)
{
	*ilim = SGM4154x_IINDPM_I_MIN_uA;

	return 0;
}

static int sgm4154x_get_state(struct sgm4154x_device *sgm, struct sgm4154x_state *state)
{
	u8 chrg_stat;
	u8 fault;
	u8 chrg_param_0, chrg_param_1, chrg_param_2;
	int ret;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret) {
		ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret) {
			pr_err("%s read SGM4154x_CHRG_STAT fail\n", __func__);
			return ret;
		}
	}

	state->chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM4154x_PG_STAT);
	state->therm_stat = !!(chrg_stat & SGM4154x_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_VSYS_STAT);

	pr_err("%s chrg_type:0x%x, chrg_stat:0x%x online:%d\n", __func__,
		state->chrg_type, state->chrg_stat, state->online);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_FAULT, &fault);
	if (ret) {
		pr_err("%s read SGM4154x_CHRG_FAULT fail\n", __func__);
		return ret;
	}

	state->chrg_fault = fault;
	state->ntc_fault = fault & SGM4154x_TEMP_MASK;
	state->health = state->ntc_fault;
	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_0, &chrg_param_0);
	if (ret) {
		pr_err("%s read SGM4154x_CHRG_CTRL_0 fail\n", __func__);
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_HIZ_EN);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_5, &chrg_param_1);
	if (ret) {
		pr_err("%s read SGM4154x_CHRG_CTRL_5 fail\n", __func__);
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_TERM_EN);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_a, &chrg_param_2);
	if (ret) {
		pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n", __func__);
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_VBUS_GOOD);

	return 0;
}

__maybe_unused static int sgm4154x_get_charge_stat(struct sgm4154x_device *sgm)
{
	u8 chrg_stat;
	int ret;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret) {
		ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret) {
			pr_err("%s read SGM4154x_CHRG_STAT fail\n", __func__);
			return status;
		}
	}

	mutex_lock(&sgm->lock);
	sgm->state.chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
	sgm->state.chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	mutex_unlock(&sgm->lock);

	dev_info(sgm->dev, "%s: chrg_type:0x%x, chrg_stat:0x%x\n",
		__func__, sgm->state.chrg_type, sgm->state.chrg_stat);

	if (!sgm->state.chrg_type || sgm->state.chrg_type == SGM4154x_OTG_MODE) {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		switch (sgm->state.chrg_stat) {
		case SGM4154x_NOT_CHRGING:
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case SGM4154x_PRECHRG:
		case SGM4154x_FAST_CHRG:
			status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case SGM4154x_TERM_CHRG:
			status = POWER_SUPPLY_STATUS_FULL;
			break;
		}
	}
	return status;
}

__maybe_unused static int sgm4154x_set_hiz_en(struct charger_device *chg_dev, bool hiz_en)
{
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM4154x_HIZ_EN : 0;

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_0,
			SGM4154x_HIZ_EN, reg_val);
}

static int sgm4154x_enable_charger(struct sgm4154x_device *sgm)
{
	int ret;
#if IS_ENABLED(CONFIG_FACTORY_BUILD)
	int uisoc = -1;
	int bat_vol = 3450;
	struct power_supply *bat_psy = NULL;
	union power_supply_propval prop;

	if (bat_psy == NULL) {
		bat_psy = power_supply_get_by_name("battery");
		if (bat_psy == NULL) {
			dev_err(sgm->dev, "[%s]psy is not rdy\n", __func__);
			uisoc = -1;
			bat_vol = 4001;
		}
	}

	if (bat_psy) {
		ret = power_supply_get_property(bat_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		bat_vol = prop.intval / 1000;

		ret = power_supply_get_property(bat_psy,
				POWER_SUPPLY_PROP_CAPACITY, &prop);
		uisoc = prop.intval;
	}

	if ((uisoc >= 70 || (uisoc == -1 && bat_vol > 4000)) && (factory_charging_limit == 1))
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
				SGM4154x_CHRG_EN, 0);
	else
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
				SGM4154x_CHRG_EN, SGM4154x_CHRG_EN);
	dev_info(sgm->dev, "[Factory Test][%s] uisoc:%d battery_voltage:%d\n",
			__func__, uisoc, bat_vol);
#else
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_CHRG_EN, SGM4154x_CHRG_EN);
#endif

	return ret;
}

static int sgm4154x_disable_charger(struct sgm4154x_device *sgm)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_CHRG_EN, 0);

	return ret;
}

static int sgm4154x_disable_pfm(struct sgm4154x_device *sgm)
{
	int ret;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_PFM_EN, SGM4154x_PFM_EN);


	pr_err("[%s] pfm mode disable sucessful\n", __func__);
	return ret;
}
static int sgm4154x_is_charging(struct charger_device *chg_dev,bool *en)
{
	int ret;
	u8 val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_1, &val);
	if (ret) {
		pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n", __func__);
		return ret;
	}
	*en = (val & SGM4154x_CHRG_EN) ? 1 : 0;

	return ret;
}

static int sgm4154x_charging_switch(struct charger_device *chg_dev,bool enable)
{
	int ret;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (enable)
		ret = sgm4154x_enable_charger(sgm);
	else
		ret = sgm4154x_disable_charger(sgm);

	return ret;
}

static int sgm4154x_set_recharge_volt(struct sgm4154x_device *sgm, int mV)
{
	u8 reg_val;

	reg_val = (mV - SGM4154x_VRECHRG_OFFSET_mV) / SGM4154x_VRECHRG_STEP_mV;

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_4,
			SGM4154x_VRECHARGE, reg_val);
}

static int sgm4154x_set_wdt_rst(struct sgm4154x_device *sgm, bool is_rst)
{
	u8 val;

	if (is_rst)
		val = SGM4154x_WDT_RST_MASK;
	else
		val = 0;

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_WDT_RST_MASK, val);
}

static int sgm4154x_set_dpdm_hiz(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val = 0;

	/*set dp in Hiz mode*/
	reg_val = 0 << 3;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_DP_VSEL_MASK, reg_val);
	if (ret < 0) {
		dev_err(sgm->dev, "%s set dp hiz failed ret(%d)\n", __func__, ret);
		return ret;
	}

	/*set dm in Hiz mode*/
	reg_val = 0 << 1;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_DM_VSEL_MASK, reg_val);
	if (ret < 0) {
		dev_err(sgm->dev, "%s set dm hiz failed ret(%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_dump_register(struct charger_device *chg_dev)
{

	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char sgm4154x_reg[SGM4154x_REG_NUM + 1] = { 0 };
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	char reg_buff[TOTAL_DUMP_LEN] = {0};
	char temp_buff[SINGLE_DUMP_LEN] = {0};

	if (dump_reg_enable) {
		for (i = 0; i < SGM4154x_REG_NUM + 1; i++) {
			ret = sgm4154x_read_reg(sgm, i, &sgm4154x_reg[i]);
			if (ret != 0) {
				pr_info("%s, [sgm4154x] i2c transfor error\n", __func__);
				return ret;
			}
			snprintf(temp_buff, SINGLE_DUMP_LEN, "reg[0x%02x] = 0x%02x, ", i, sgm4154x_reg[i]);
			strcat(reg_buff, temp_buff);
		}
		pr_info("%s: %s", __func__, reg_buff);
	} else {
		pr_err("%s, dump register has been disabled\n", __func__);
	}

	return ret;
}

static int sgm4154x_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	struct sgm4154x_state state;

	pr_info("%s: enter, enable charging\n", __func__);
	sgm4154x_disable_pfm(sgm);
	/* Enable charging */

	ret = sgm4154x_enable_charger(sgm);
	if (ret) {
		pr_err("%s: Failed to enable charging:%d\n", __func__, ret);
	}
	sgm4154x_dump_register(sgm->chg_dev);
	ret = sgm4154x_get_state(sgm, &state);
	if (ret) {
		pr_err("%s: Failed to get state:%d\n", __func__, ret);
	}

	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	return ret;
}

static int sgm4154x_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_info("%s: enter, disable charging\n", __func__);

	ret = sgm4154x_disable_charger(sgm);
	if (ret) {
		pr_err("%s: Failed to disable charging:%d\n", __func__, ret);
	}

	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_reset_registers(struct sgm4154x_device *sgm)
{
	int ret = 0;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_b,
					SGM4151x_REG_RST, SGM4151x_REG_RST);
	if (ret < 0) {
		pr_info("[%s] reset fail\n", __func__);
		return ret;
	}

	return ret;
}

static int sgm4154x_hw_chipid_detect(struct sgm4154x_device *sgm)
{
	int ret = 0;
	u8 val = 0;

	ret = sgm4154x_read_reg(sgm,SGM4154x_CHRG_CTRL_b, &val);
	if (ret < 0) {
		pr_info("[%s] read SGM4154x_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}

	val = val & SGM4154x_PN_MASK;
	pr_info("[%s] Reg[0x0B]=0x%x\n", __func__, val);

	return val;
}

static int sgm4154x_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	int ret;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	pr_info("[%s] charging_reset_watch_dog_timer\n", __func__);

	ret = sgm4154x_set_wdt_rst(sgm, 0x1);	/* RST watchdog */

	return ret;
}

static int sgm4154x_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	//struct sgm4154x_state state;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	//sgm4154x_get_state(sgm, &state);

	if (sgm->state.chrg_stat == SGM4154x_TERM_CHRG)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}

static int sgm4154x_set_en_timer(struct sgm4154x_device *sgm)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
			SGM4154x_SAFETY_TIMER_EN, SGM4154x_SAFETY_TIMER_EN);

	return ret;
}

static int sgm4154x_set_disable_timer(struct sgm4154x_device *sgm)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
			SGM4154x_SAFETY_TIMER_EN, 0);

	return ret;
}

static int sgm4154x_enable_safetytimer(struct charger_device *chg_dev, bool en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int ret = 0;

	if (en)
		ret = sgm4154x_set_en_timer(sgm);
	else
		ret = sgm4154x_set_disable_timer(sgm);

	return ret;
}

static int sgm4154x_get_is_safetytimer_enable(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	u8 val = 0;

	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm,SGM4154x_CHRG_CTRL_5, &val);
	if (ret < 0) {
		pr_info("[%s] read SGM4154x_CHRG_CTRL_5 fail\n", __func__);
		return ret;
	}

	*en = !!(val & SGM4154x_SAFETY_TIMER_EN);

	return 0;
}

#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static int sgm4154x_en_pe_current_partern(struct charger_device *chg_dev, bool is_up)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
			SGM4154x_EN_PUMPX, SGM4154x_EN_PUMPX);
	if (ret < 0) {
		pr_info("[%s] read SGM4154x_CHRG_CTRL_d fail\n", __func__);
		return ret;
	}

	if (is_up)
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_PUMPX_UP, SGM4154x_PUMPX_UP);
	else
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_PUMPX_DN, SGM4154x_PUMPX_DN);
	return ret;
}
#endif

static enum power_supply_property sgm4154x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TYPE,
	//POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PRESENT
};

static int sgm4154x_property_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	//case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_ONLINE:
		return true;
	default:
		return false;
	}
}

static int sgm4154x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	int ret = 0;

	if (IS_ERR_OR_NULL(sgm)) {
		pr_err("%s: get sgm device failed\n", __func__);
		return -ENODEV;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 2) {
			dev_info(sgm->dev, "%s: %d, start charger detection\n", __func__, val->intval);
			schedule_delayed_work(&sgm->charge_detect_delayed_work, msecs_to_jiffies(600));
		} else if (val->intval == 0) {
			dev_info(sgm->dev, "%s: %d, vbus not online \n", __func__, val->intval);
			sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
			cancel_delayed_work(&sgm->charge_detect_delayed_work);
			power_supply_changed(sgm->charger);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_set_input_curr_lim(s_chg_dev_otg, val->intval);
		break;
/*	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sgm4154x_charging_switch(s_chg_dev_otg,val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_set_input_volt_lim(s_chg_dev_otg, val->intval);
		break;*/
	default:
		return -EINVAL;
	}

	return ret;
}

static int sgm4154x_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	struct sgm4154x_state state;
	int ret = 0;

	mutex_lock(&sgm->lock);
	state = sgm->state;
	mutex_unlock(&sgm->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sgm4154x_get_charge_stat(sgm);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_stat) {
		case SGM4154x_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case SGM4154x_TERM_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM4154x_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SGM4154x_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.vbus_gd;
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sgm4154x_power_supply_desc.type;
		break;

	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = sgm->psy_usb_type;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.chrg_fault & 0xF8)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case SGM4154x_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		case SGM4154x_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case SGM4154x_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case SGM4154x_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		sgm4154x_get_vbus(sgm->chg_dev, &(val->intval));
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval = state.ibus_adc;
		break;

/*	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_get_input_volt_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;*/

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		break;
#if 0
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !state.hiz_en;
		break;
#endif
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = sgm->batt_vol * 1000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

__maybe_unused static bool sgm4154x_state_changed(struct sgm4154x_device *sgm,
		struct sgm4154x_state *new_state)
{
	struct sgm4154x_state old_state;

	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_type != new_state->chrg_type ||
		old_state.chrg_stat != new_state->chrg_stat ||
		old_state.online != new_state->online ||
		old_state.therm_stat != new_state->therm_stat ||
		old_state.vsys_stat != new_state->vsys_stat ||
		old_state.chrg_fault != new_state->chrg_fault
		);
}

#if 0
static int update_battery_info_from_gauge(struct sgm4154x_device *sgm)
{
	int ret = 0;
	union power_supply_propval info;

	if (IS_ERR_OR_NULL(sgm->battery)) {
		sgm->battery = power_supply_get_by_name("battery");
		if (IS_ERR_OR_NULL(sgm->battery)) {
			dev_err(sgm->dev, "%s failed to get battery supply\n", __func__);
		}
		return -EINVAL;
	}

	/*get Vbat from gauge*/
	ret = power_supply_get_property(sgm->battery,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &info);
	sgm->batt_vol = info.intval / 1000;
	/*get Ibat from gauge*/
	ret = power_supply_get_property(sgm->battery,
			POWER_SUPPLY_PROP_CURRENT_NOW, &info);
	sgm->batt_curr = info.intval / 1000;

	dev_info(sgm->dev, "%s: Vbat = %d mV, Ibat = %d mA\n",
			__func__, sgm->batt_vol, sgm->batt_curr);

	return ret;
}

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	struct sgm4154x_device *sgm = NULL;
	//static u8 last_chg_method = 0;
	struct sgm4154x_state state;

	sgm = container_of(work, struct sgm4154x_device, charge_monitor_work.work);
	if (sgm == NULL) {
		pr_err("%s: Cann't get sgm \n", __func__);
		return;
	}

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	ret = update_battery_info_from_gauge(sgm);
	if (ret) {
		dev_err(sgm->dev, "%s: failed to get batt vol and curr\n", __func__);
	}

	if (!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "%s: Vbus not present, disable charge\n", __func__);
		sgm4154x_disable_charger(sgm);
		goto out;
	}

	if (!state.online) {
		dev_err(sgm->dev, "%s: Vbus not online\n", __func__);
		goto out;
	}

	sgm4154x_dump_register(sgm->chg_dev);
	pr_err("%s\n", __func__);
out:
	schedule_delayed_work(&sgm->charge_monitor_work, 10 * HZ);
}
#endif

static int sgm4154x_force_dpdm(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;

	Charger_Detect_Init(sgm);
	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_7, &reg_val);
	if (ret) {
	    pr_err("%s: read reg failed(%d)\n", __func__, ret);
	}

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7,
	        SGM4154x_FORCE_DPDM, reg_val | 0x80);
}

static void retry_charger_detect_work_func(struct work_struct *work)
{
	struct sgm4154x_device *sgm = NULL;
	int ret;
	sgm = container_of(work, struct sgm4154x_device, retry_charger_detect_work.work);
	if (sgm == NULL) {
		pr_err("%s: Cann't get sgm4154x_device\n", __func__);
		return;
	}

	ret = sgm4154x_force_dpdm(sgm);
	if (ret < 0) {
		pr_err("%s: Cann't force dpdm\n", __func__);
		return;
	}

	sgm->force_detect_count++;
	schedule_delayed_work(&sgm->charge_detect_delayed_work, msecs_to_jiffies(300));

	return;
}

static void charger_detect_work_func(struct work_struct *work)
{
	struct sgm4154x_device *sgm = NULL;
	//static int charge_type_old = 0;
	struct sgm4154x_state state;
	int ret;

	sgm = container_of(work, struct sgm4154x_device, charge_detect_delayed_work.work);
	if (sgm == NULL) {
		pr_err("%s: Cann't get sgm4154x_device\n", __func__);
		return;
	}

	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);
	ret = sgm4154x_set_vindpm_track(sgm, SGM4154x_TRACK_300);
	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);
	pr_err("%s:charger_detect_work_func %d\n", __func__, ret);
#if 0
	if (!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "%s: Vbus not present, disable charge\n", __func__);
		sgm4154x_disable_charger(sgm);
		sgm4154x_set_dpdm_hiz(sgm);
		sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		allow_set_dp_dm_vol = false;
		goto err;
	} else {
		allow_set_dp_dm_vol = true;
	}

	if (!state.online) {
		dev_err(sgm->dev, "%s: Vbus not online\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		goto err;
	}
#endif
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	switch(sgm->state.chrg_type) {
	case SGM4154x_USB_SDP:
		pr_info("[%s] SGM4154x charger type: SDP\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_USB;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
		if (sgm->force_detect_count < 3) {
			pr_info("[%s] SGM4154x charger type: SDP, retry bc12 count:%d\n", __func__, sgm->force_detect_count);
			schedule_delayed_work(&sgm->retry_charger_detect_work, 100);
		}
		break;

	case SGM4154x_USB_CDP:
		pr_info("[%s] SGM4154x charger type: CDP\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;

	case SGM4154x_USB_DCP:
		pr_info("[%s] SGM4154x charger type: DCP\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;

	case SGM4154x_UNKNOWN:
		pr_info("[%s] SGM4154x charger type: UNKNOWN\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
		if (sgm->force_detect_count < 10) {
			pr_info("[%s] SGM4154x charger type: UNKNOWN, retry bc12 count:%d\n", __func__, sgm->force_detect_count);
			schedule_delayed_work(&sgm->retry_charger_detect_work, 100);
		}
		break;

	case SGM4154x_NON_STANDARD:
		pr_info("[%s] SGM4154x charger type: NON STANDARD\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
		if (sgm->force_detect_count < 10) {
			pr_info("[%s] SGM4154x charger type: NON STANDARD, retry bc12 count:%d\n", __func__, sgm->force_detect_count);
			schedule_delayed_work(&sgm->retry_charger_detect_work, 100);
		}
		break;

	default:
		pr_info("[%s] SGM4154x charger type: default\n", __func__);
		sgm->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
		if (sgm->force_detect_count < 10) {
			pr_info("[%s] SGM4154x charger type: Default, retry bc12 count:%d\n", __func__, sgm->force_detect_count);
			schedule_delayed_work(&sgm->retry_charger_detect_work, 100);
		}
		__pm_relax(sgm->charger_wakelock);
		//break;
		return;
	}

	if (sgm->state.chrg_type == SGM4154x_USB_SDP || sgm->state.chrg_type == SGM4154x_USB_CDP) {
		Charger_Detect_Release(sgm);
	}

	dev_info(sgm->dev, "%s: Update: chg_type = %d, psy_usb_type = %d\n",
				__func__, sgm->chg_type, sgm->psy_usb_type);
#endif
	//sgm4154x_enable_charger(sgm);
	sgm4154x_dump_register(sgm->chg_dev);
#if 0
err:
#endif
	//release wakelock
	power_supply_changed(sgm->charger);
	dev_err(sgm->dev, "Relax wakelock\n");
	__pm_relax(sgm->charger_wakelock);

	return;
}

static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_device *sgm = private;
	struct sgm4154x_state state;
	bool prev_vbus_gd;
	int ret = 0;

	//lock wakelock
	pr_info("[%s] entry\n", __func__);

	ret = sgm4154x_get_state(sgm, &state);
	if (ret) {
		pr_err("%s: Failed to get state:%d\n", __func__, ret);
		return IRQ_HANDLED;
	}

	mutex_lock(&sgm->lock);
	prev_vbus_gd = sgm->state.vbus_gd;
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if (!prev_vbus_gd && sgm->state.vbus_gd) {
		Charger_Detect_Init(sgm);
		sgm->force_detect_count = 0;
		allow_set_dp_dm_vol = true;
		dev_info(sgm->dev, "%s: adapter/usb inserted\n", __func__);
#if IS_ENABLED(CONFIG_FACTORY_BUILD)
		sgm4154x_enable_charger(sgm);
#endif
	} else if (prev_vbus_gd && !sgm->state.vbus_gd) {
		dev_info(sgm->dev, "%s: adapter/usb removed\n", __func__);
		Charger_Detect_Release(sgm);
		sgm4154x_set_dpdm_hiz(sgm);
		allow_set_dp_dm_vol = false;
	}

	//power_supply_changed(sgm->charger);

	return IRQ_HANDLED;
}

static char *sgm4154x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static struct power_supply_desc sgm4154x_power_supply_desc = {
	.name = "primary_chg",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = sgm4154x_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm4154x_usb_type),
	.properties = sgm4154x_power_supply_props,
	.num_properties = ARRAY_SIZE(sgm4154x_power_supply_props),
	.get_property = sgm4154x_charger_get_property,
	.set_property = sgm4154x_charger_set_property,
	.property_is_writeable = sgm4154x_property_is_writeable,
};

static int sgm4154x_power_supply_init(struct sgm4154x_device *sgm, struct device *dev)
{
	struct power_supply_config psy_cfg = {
		.drv_data = sgm,
		.of_node = dev->of_node,
	};

	psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);

	sgm->charger = devm_power_supply_register(sgm->dev,
			 &sgm4154x_power_supply_desc,
			 &psy_cfg);

	if (IS_ERR(sgm->charger))
		return -EINVAL;

	return 0;
}

static int sgm4154x_hw_init(struct sgm4154x_device *sgm)
{
	int ret = 0;
	struct power_supply_battery_info bat_info = { };

	bat_info.constant_charge_current_max_ua =
			SGM4154x_ICHRG_I_DEF_uA;

	bat_info.constant_charge_voltage_max_uv =
			SGM4154x_VREG_V_DEF_uV;

	bat_info.precharge_current_ua =
			SGM4154x_PRECHRG_I_DEF_uA;

	bat_info.charge_term_current_ua =
			SGM4154x_TERMCHRG_I_DEF_uA;

	sgm->init_data.max_ichg =
			SGM4154x_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
			SGM4154x_VREG_V_MAX_uV;

	sgm4154x_set_watchdog_timer(sgm, 0);
	sgm4154x_set_dpm_mask(sgm);


#if IS_ENABLED(CONFIG_FACTORY_BUILD)
	dev_info(sgm->dev, "%s disable charging for factory version\n", __func__);
	ret = sgm4154x_disable_charger(sgm);
	if (ret)
		dev_err(sgm->dev, "%s disable charging failed\n", __func__);
#endif
	sgm4154x_set_tmr2x(sgm, false);

	sgm4154x_dump_register(sgm->chg_dev);
	ret = sgm4154x_set_ichrg_curr(s_chg_dev_otg,
			bat_info.constant_charge_current_max_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_prechrg_curr(sgm, bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_chrg_volt(s_chg_dev_otg,
			bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_term_curr(s_chg_dev_otg, bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;

	/*ret = sgm4154x_set_input_volt_lim(sgm, sgm->init_data.vlim);
	if (ret)
		goto err_out;*/

	ret = sgm4154x_set_input_curr_lim(s_chg_dev_otg, sgm->init_data.ilim);
	if (ret)
		goto err_out;
#if 0
	ret = sgm4154x_set_vac_ovp(sgm); //14V
	if (ret)
		goto err_out;
#endif
	ret = sgm4154x_set_recharge_volt(sgm, 200); //100~200mv
	if (ret)
		goto err_out;

	dev_notice(sgm->dev, "ichrg_curr:%d prechrg_curr:%d chrg_vol:%d term_curr:%d input_curr_lim:%d",
		bat_info.constant_charge_current_max_ua,
		bat_info.precharge_current_ua,
		bat_info.constant_charge_voltage_max_uv,
		bat_info.charge_term_current_ua,
		sgm->init_data.ilim);

	return 0;
err_out:
	return ret;
}

static int sgm4154x_parse_dt(struct sgm4154x_device *sgm)
{
	int ret;
	int irq_gpio = 0, irqn = 0;
	int chg_en_gpio = 0;

	ret = device_property_read_u32(sgm->dev,
			"input-voltage-limit-microvolt", &sgm->init_data.vlim);
	if (ret) {
		sgm->init_data.vlim = SGM4154x_VINDPM_DEF_uV;
	}

	if (sgm->init_data.vlim > SGM4154x_VINDPM_V_MAX_uV ||
		sgm->init_data.vlim < SGM4154x_VINDPM_V_MIN_uV) {
		dev_err(sgm->dev, "%s: VIN DPM out of range\n", __func__);
		return -EINVAL;
	}

	ret = device_property_read_u32(sgm->dev,
			"input-current-limit-microamp", &sgm->init_data.ilim);
	if (ret) {
		sgm->init_data.ilim = SGM4154x_IINDPM_DEF_uA;
	}

	if (sgm->init_data.ilim > SGM4154x_IINDPM_I_MAX_uA ||
		sgm->init_data.ilim < SGM4154x_IINDPM_I_MIN_uA) {
		dev_err(sgm->dev, "%s: IIN DPM out of range\n", __func__);
		return -EINVAL;
	}

	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}

	ret = gpio_request(irq_gpio, "sgm4154x irq pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}

	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}

	sgm->client->irq = irqn;

	chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (!gpio_is_valid(chg_en_gpio)) {
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, chg_en_gpio);
		return -EINVAL;
	}

	ret = gpio_request(chg_en_gpio, "sgm chg en pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, chg_en_gpio);
		return ret;
	}

	gpio_direction_output(chg_en_gpio, 0); //default enable charge

	return 0;
}

static int sgm4154x_enable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(s_chg_dev_otg);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_OTG_EN, SGM4154x_OTG_EN);

	return ret;
}

static int sgm4154x_disable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(s_chg_dev_otg);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_OTG_EN, 0);

	return ret;
}

static int sgm4154x_is_enabled_vbus(struct regulator_dev *rdev)
{
	u8 temp = 0;
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(s_chg_dev_otg);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_1, &temp);
	return (temp&SGM4154x_OTG_EN) ? 1 : 0;
	pr_err("%s:sgm4154x_is_enabled_vbus %d\n", __func__, ret);
}

#if 0
static int sgm4154x_set_volt_to_reg(u32 volt)
{
	int reg_val = 0;
	if (volt == 0)
		reg_val = 0x1;
	else if (volt == 3300000)
		reg_val = 0x3;
	else if (volt == 600000)
		reg_val = 0x2;
	else
		reg_val = 0x0;

	return reg_val;
}


static int sgm4154x_set_dp(struct charger_device *chg_dev, u32 volt)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int reg_val = 0;

	if (false == allow_set_dp_dm_vol) {
		dev_info(sgm->dev, "%s: not allow set dp voltage\n", __func__);
		return -EINVAL;
	}

	reg_val = sgm4154x_set_volt_to_reg(volt);

	reg_val = reg_val << 3;
	dev_info(sgm->dev, "%s: set_dp = %duV\n", __func__, volt);
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
			SGM4154x_DP_VSEL_MASK, reg_val);
}

static int sgm4154x_set_dm(struct charger_device *chg_dev, u32 volt)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int reg_val = 0;

	if (false == allow_set_dp_dm_vol) {
		dev_info(sgm->dev, "%s: not allow set dp voltage\n", __func__);
		return -EINVAL;
	}

	reg_val = sgm4154x_set_volt_to_reg(volt);

	reg_val = reg_val << 1;
	dev_info(sgm->dev, "%s: set_dm = %duV\n", __func__, volt);
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
			SGM4154x_DM_VSEL_MASK, reg_val);
}


static int sgm4154x_enable_dpdm_hiz(struct charger_device *chg_dev)
{
	int ret;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	ret = sgm4154x_set_dpdm_hiz(sgm);
	if (ret < 0)
		dev_err(sgm->dev, "%s set dpdm hiz failed ret(%d)\n", __func__, ret);

	return ret;
}
#endif

static int sgm4154x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	dev_info(sgm->dev, "%s event:%d\n", __func__, event);

	switch (event) {
	case EVENT_FULL:
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		power_supply_changed(sgm->charger);
		break;
	default:
		break;
	}
	return 0;
}

static int sgm4154x_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	pr_info("%s en = %d\n", __func__, en);
	if (en) {
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_OTG_EN, SGM4154x_OTG_EN);
	} else {
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
			SGM4154x_OTG_EN, 0);
	}

	return ret;
}

__maybe_unused static int sgm4154x_set_boost_voltage_limit(
		struct charger_device *chg_dev, u32 uV)
{
	int ret = 0;
	char reg_val = -1;
	int i = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	while (i < 4) {
		if (uV == BOOST_VOLT_LIMIT[i]) {
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;

	reg_val = reg_val << 4;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_6,
			SGM4154x_BOOSTV, reg_val);

	return ret;
}

static int sgm4154x_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (uA == BOOST_CURRENT_LIMIT[0]) {
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2,
				SGM4154x_BOOST_LIM, 0);
	} else if (uA == BOOST_CURRENT_LIMIT[1]) {
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2,
				SGM4154x_BOOST_LIM, BIT(7));
	}

	return ret;
}

static struct regulator_ops sgm4154x_vbus_ops = {
	.enable = sgm4154x_enable_vbus,
	.disable = sgm4154x_disable_vbus,
	.is_enabled = sgm4154x_is_enabled_vbus,
};

static const struct regulator_desc sgm4154x_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

__maybe_unused static int sgm4154x_vbus_regulator_register(struct sgm4154x_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
				&sgm4154x_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
	}

	return ret;
}

static struct charger_ops sgm4154x_chg_ops = {
	.dump_registers = sgm4154x_dump_register,
	/* cable plug in/out */
	.plug_in = sgm4154x_plug_in,
	.plug_out = sgm4154x_plug_out,
	/* enable */
	.enable = sgm4154x_charging_switch,
	.is_enabled = sgm4154x_is_charging,
	/* charging current */
	.set_charging_current = sgm4154x_set_ichrg_curr,
	.get_charging_current = sgm4154x_get_ichg_curr,
	.get_min_charging_current = sgm4154x_get_minichg_curr,
	/* charging voltage */
	.set_constant_voltage = sgm4154x_set_chrg_volt,
	.get_constant_voltage = sgm4154x_get_chrg_volt,
	/* input current limit */
	.set_input_current = sgm4154x_set_input_curr_lim,
	.get_input_current = sgm4154x_get_input_curr_lim,
	.get_min_input_current = sgm4154x_get_input_mincurr_lim,
	/* MIVR */
	.set_mivr = sgm4154x_set_input_volt_lim,
	.get_mivr = sgm4154x_get_input_volt_lim,
	//.get_mivr_state = sgm4154x_get_input_minvolt_lim,
	/* ADC */
	.get_vbus_adc = sgm4154x_get_vbus,
	//.get_adc = sgm4154x_get_adc,
	//.get_adc = mt6375_get_adc,
	//.get_vbus_adc = mt6375_get_vbus,
	//.get_ibus_adc = mt6375_get_ibus,
	//.get_ibat_adc = mt6375_get_ibat,
	//.get_tchg_adc = mt6375_get_tchg,
	//.get_zcv = mt6375_get_zcv,
	/* charing termination */
	.set_eoc_current = sgm4154x_set_term_curr,
	.enable_termination = sgm4154x_enable_terminate,
	//.reset_eoc_state = mt6375_reset_eoc_state,
	//.safety_check = mt6375_sw_check_eoc,
	.is_charging_done = sgm4154x_get_charging_status,
	/* power path */
	//.enable_powerpath = mt6375_enable_buck,
	//.is_powerpath_enabled = mt6375_is_buck_enabled,
	/* timer */
	.enable_safety_timer = sgm4154x_enable_safetytimer,
	.is_safety_timer_enabled = sgm4154x_get_is_safetytimer_enable,
	.kick_wdt = sgm4154x_reset_watch_dog_timer,
	/* AICL */
	//.run_aicl = mt6375_run_aicc,
	/* PE+/PE+20 */
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	.send_ta_current_pattern = sgm4154x_en_pe_current_partern,
#else
	.send_ta_current_pattern = NULL,
#endif
	//.set_pe20_efficiency_table = mt6375_set_pe20_efficiency_table,
	//.send_ta20_current_pattern = mt6375_set_pe20_current_pattern,
	//.reset_ta = mt6375_reset_pe_ta,
	//.enable_cable_drop_comp = mt6,
	/* OTG */
	.enable_otg = sgm4154x_enable_otg,
	.set_boost_current_limit = sgm4154x_set_boost_current_limit,
	.enable_hz = sgm4154x_set_hiz_en,
	/* DPDM */
	//.set_dp = sgm4154x_set_dp,
	//.set_dm = sgm4154x_set_dm,
#if 0
	.enable_dpdm_hz = sgm4154x_enable_dpdm_hiz,
#endif
	.event = sgm4154x_do_event,

};

static ssize_t dump_reg_ctrl_write(struct file *filp,
	const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[8] = {0};
	long val = 0;
	int ret = 0;

	if (cnt >= sizeof(buf)) {
		pr_err( "%s cnt is invalid\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&buf, ubuf, cnt)) {
		pr_err("%s cnt is invalid\n", __func__);
		return -EFAULT;
	}

	buf[cnt] = 0;
	ret = kstrtoul(buf, 10, (unsigned long *)&val);
	if (ret < 0) {
		pr_err("%s cnt is invalid\n", __func__);
		return ret;
	}

	dump_reg_enable = val;
	pr_info("%s dump_reg_enable is %s\n", __func__, dump_reg_enable ? "enable" : "disable");

	return cnt;
}

static int dump_reg_ctrl_show(struct seq_file *m, void *v)
{
	seq_printf(m, "dump reg enable is %s\n", dump_reg_enable ? "enable" : "disable");

	return 0;
}

static int dump_reg_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_reg_ctrl_show, inode->i_private);
}

static const struct proc_ops dump_reg_ctrl_fops = {
	.proc_open = dump_reg_ctrl_open,
	.proc_write = dump_reg_ctrl_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static ssize_t sgm4154x_show_registers(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);
	uint8_t addr;
	uint8_t val;
	uint8_t tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sgm41542");

	for (addr = 0; addr < SGM4154x_REG_NUM + 1; addr++) {
		ret = sgm4154x_read_reg(sgm, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sgm4154x_store_register(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	unsigned int reg;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= SGM4154x_REG_NUM)
		sgm4154x_write_reg(sgm, reg, val);

	return count;
}

static DEVICE_ATTR(registers, 0660, sgm4154x_show_registers, sgm4154x_store_register);

static int sgm4154x_create_device_node(struct device *dev)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_registers);
	if (ret < 0) {
		pr_err("[%s] failed to create register attr\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void sgm4154x_destory_device_node(struct device *dev)
{
	device_remove_file(dev, &dev_attr_registers);
}

static int sgm4154x_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = &client->dev;
	struct sgm4154x_device *sgm;

	char *name = NULL;

	pr_info("[%s]\n", __func__);

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm) {
		pr_err("[%s] alloc memory failed\n", __func__);
		return -ENOMEM;
	}

	sgm->client = client;
	sgm->dev = dev;

	mutex_init(&sgm->lock);
	mutex_init(&sgm->i2c_rw_lock);

	i2c_set_clientdata(client, sgm);

	sgm4154x_disable_pfm(sgm);
	if (ret) {
		pr_err("[%s] pfm mode disable failed\n", __func__);
	}

	sgm->vbus = devm_iio_channel_get(sgm->dev, "pmic_vbus");
	if (IS_ERR_OR_NULL(sgm->vbus)) {
		dev_err(sgm->dev, "sgm41542 get vbus failed\n");
		return -EPROBE_DEFER;
	}

	ret = sgm4154x_hw_chipid_detect(sgm);
	if (ret != SGM4154x_PN_ID) {
		pr_info("[%s] device not found !!!\n", __func__);
		return ret;
	}

	ret = sgm4154x_parse_dt(sgm);
	if (ret) {
		pr_err("[%s] parse dts resource failed\n", __func__);
		return ret;
	}

	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s","sgm4154x suspend wakelock");
	sgm->charger_wakelock =	wakeup_source_register(sgm->dev, name);

	/* Register charger device */
	sgm->chg_dev = charger_device_register("primary_chg",
				&client->dev, sgm,
				&sgm4154x_chg_ops,
				&sgm4154x_chg_props);

	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}

	/* otg regulator */
	s_chg_dev_otg = sgm->chg_dev;

	INIT_DELAYED_WORK(&sgm->charge_detect_delayed_work, charger_detect_work_func);
	//INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	INIT_DELAYED_WORK(&sgm->retry_charger_detect_work, retry_charger_detect_work_func);
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
				sgm4154x_irq_handler_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				dev_name(&client->dev), sgm);
		if (ret) {
			pr_err("[%s] request irq failed\n", __func__);
			return ret;
		}
		enable_irq_wake(client->irq);
	}

	ret = sgm4154x_power_supply_init(sgm, dev);
	if (ret) {
		pr_err("[%s] Failed to register power supply\n", __func__);
		return ret;
	}
	ret = sgm4154x_hw_init(sgm);

	dump_reg_enable = true;
	entry = proc_create("dump_reg_ctrl", 0664, NULL, &dump_reg_ctrl_fops);
	if (!entry) {
		pr_err("%s create proc directory failed\n", __func__);
	}
	ret = sgm4154x_disable_pfm(sgm);
	sgm4154x_dump_register(sgm->chg_dev);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}

	ret = sgm4154x_create_device_node(&(client->dev));

	//OTG setting
	//sgm4154x_set_otg_voltage(s_chg_dev_otg, 5000000); //5V
	//sgm4154x_set_otg_current(s_chg_dev_otg, 1200000); //1.2A

	//ret = sgm4154x_vbus_regulator_register(sgm);

	//schedule_delayed_work(&sgm->charge_monitor_work, msecs_to_jiffies(100));

#if IS_ENABLED(CONFIG_OEM_DEVINFO)
	FULL_PRODUCT_DEVICE_INFO(ID_SWITCH_CHARGER, "SGM41542");
#endif
	sgm4154x_irq_handler_thread(client->irq, (void *)sgm);
	pr_info("%s successfully\n", __func__);
	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

	//cancel_delayed_work_sync(&sgm->charge_monitor_work);

	//regulator_unregister(sgm->otg_rdev);

	power_supply_unregister(sgm->charger);

	sgm4154x_destory_device_node(sgm->dev);
	mutex_destroy(&sgm->lock);
	mutex_destroy(&sgm->i2c_rw_lock);

	return 0;
}

static void sgm4154x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

	ret = sgm4154x_disable_charger(sgm);
	if (ret) {
		pr_err("[%s] Failed to disable charger, ret = %d\n", __func__, ret);
	}

	ret = sgm4154x_reset_registers(sgm);
	if (ret) {
		pr_err("[%s] Failed to reset registers, ret = %d\n", __func__, ret);
	}

	pr_info("[%s] sgm4154x_charger_shutdown\n", __func__);
}

static const struct i2c_device_id sgm4154x_i2c_ids[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 1 },
	{ "sgm41543", 2 },
	{ "sgm41543D", 3 },
	{ "sgm41513", 4 },
	{ "sgm41513A", 5 },
	{ "sgm41513D", 6 },
	{ "sgm41516", 7 },
	{ "sgm41516D", 8 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4154x_i2c_ids);

static const struct of_device_id sgm4154x_of_match[] = {
	{ .compatible = "sgm,sgm41541", },
	{ .compatible = "sgm,sgm41542", },
	{ .compatible = "sgm,sgm41543", },
	{ .compatible = "sgm,sgm41543D", },
	{ .compatible = "sgm,sgm41513", },
	{ .compatible = "sgm,sgm41513A", },
	{ .compatible = "sgm,sgm41513D", },
	{ .compatible = "sgm,sgm41516", },
	{ .compatible = "sgm,sgm41516D", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_of_match);

static int sgm4154x_suspend(struct device *dev)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(sgm->client->irq);
	disable_irq(sgm->client->irq);

	return 0;
}

static int sgm4154x_resume(struct device *dev)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(sgm->client->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(sgm->client->irq);

	return 0;
}

static const struct dev_pm_ops sgm4154x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sgm4154x_suspend, sgm4154x_resume)
};

static struct i2c_driver sgm4154x_driver = {
	.driver = {
		.name = "primary_chg",
		.of_match_table = sgm4154x_of_match,
		.pm = &sgm4154x_pm_ops,
	},
	.probe = sgm4154x_driver_probe,
	.remove = sgm4154x_charger_remove,
	.shutdown = sgm4154x_charger_shutdown,
	.id_table = sgm4154x_i2c_ids,
};
module_i2c_driver(sgm4154x_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4154x charger driver");
MODULE_LICENSE("GPL v2");
