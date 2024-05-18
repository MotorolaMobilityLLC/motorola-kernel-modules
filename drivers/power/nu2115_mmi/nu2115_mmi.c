// SPDX-License-Identifier: GPL-2.0
// BQ25980 Battery Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


#include "nu2115_reg.h"
#include <charger_class.h>
struct nu2115_state {
	bool dischg;
	bool ovp;
	bool ocp;
	bool vac_ovp;
	bool bat_ovp;
	bool vout_ovp;
	bool vbus_ovp;
	bool bat_ocp;
	bool bus_ocp;
	bool bus_ucp;
	bool wdt;
	bool tflt;
	bool online;
	bool ce;
	bool bypass;

	u32 vbat_adc;
	u32 vsys_adc;
	u32 ibat_adc;
	u32 fault_status;
};

enum nu_work_mode {
	NU_STANDALONE,
	NU_SLAVE,
	NU_MASTER,
};

#define NU2115_PART_NO 0x90
#define NU2115A_PART_NO 0x40
#define VAC1_STAT_MASK  0x80
#define VAC1_STAT_SHIFT  7

enum nu_device_id {
	NU2115 = 0,
};

enum nu_compatible_id {
	NU2115_STANDALONE,
	NU2115_SLAVE,
	NU2115_MASTER,
};

struct nu2115_chip_info {

	int model_id;

	const struct regmap_config *regmap_config;

	const struct reg_default *reg_init_values;

	int busocp_sc_def;
	int busocp_byp_def;
	int busocp_sc_max;
	int busocp_byp_max;
	int busocp_sc_min;
	int busocp_byp_min;
	int busocp_step;
	int busocp_offset;

	int busovp_sc_def;
	int busovp_byp_def;
	int busovp_sc_step;

	int busovp_sc_offset;
	int busovp_byp_step;
	int busovp_byp_offset;
	int busovp_sc_min;
	int busovp_sc_max;
	int busovp_byp_min;
	int busovp_byp_max;

	int batovp_def;
	int batovp_max;
	int batovp_min;
	int batovp_step;
	int batovp_offset;

	int batocp_def;
	int batocp_max;

	int vac_sc_ovp;
	int vac_byp_ovp;

	int adc_curr_step;
	int adc_vbat_volt_step;
	int adc_vbus_volt_step;
	int adc_vbus_volt_offset;
	int adc_vout_volt_step;
	int adc_vout_volt_offset;
};

struct nu2115_init_data {
	u32 ichg;
	u32 bypass_ilim;
	u32 sc_ilim;
	u32 vreg;
	u32 iterm;
	u32 iprechg;
	u32 bypass_vlim;
	u32 sc_vlim;
	u32 ichg_max;
	u32 vreg_max;
};

struct nu2115_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];

	struct nu2115_init_data init_data;
	const struct nu2115_chip_info *chip_info;
	struct nu2115_state state;
	int watchdog_timer;
	int mode;
	int device_id;
	int part_no;
	int nu2115_addr;
	struct power_supply_desc psy_desc;
	int reg_addr;
	int reg_data;
	struct charger_device *chg_dev;
	struct charger_properties chg_prop;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	struct mutex irq_complete;
	struct delayed_work dump_register_work;
};


static struct reg_default nu2115_reg_init_val[] = {
	{NU2115_BATOVP,	    0x80},//0x47:4550mV 0x45:4580mv, 0x80:disable
	{NU2115_BATOVP_ALM,	0x80},//0x3f:4470mV 0x42:4500mv, 0x80:disable
	{NU2115_BATOCP,	    0x80},//0xDA:disable for dual  11A, 0x46:7000mA for standalone, 0x80:disable
	{NU2115_BATOCP_ALM,	0x80},//0x6B:12700mA, 0x80:disable
	{NU2115_BATUCP_ALM,	0x80},//0x28:default 2a, 0x80:disable
	{NU2115_AC1PROT,	0x06},//default
	{NU2115_AC2PROT,	0x07},//default
	{NU2115_BUSOVP,	    0x2D},//0x3C:12000mv,0x2D:10500mv
	{NU2115_BUSOVP_ALM,	0x80},//0X32:11000mV, 0x80:disable
	{NU2115_BUSOCP,	    0x06},//0X06:4000mA
	{NU2115_BUSOCP_ALM,	0x8C},//0X8C:4000mA disable
	//{NU2115_CON_STAT,	0x00},
	{NU2115_CTRL_REG,	0x36},//0x36:watchdog disable 5s,600kHz
	{NU2115_CHGCTRL,	0x07},//default
	{NU2115_INT_STAT,	0x00},//default mean {NU2115_STAT1, 0x0}
	{NU2115_INT_FLAG,	0x00},//default
	{NU2115_INT_MASK,	0x00},//default
	{NU2115_FLT_MASK,	0x00},//default
	{NU2115_ADC_CTRL,	0x80},//default mean {NU2115_ADC_CONTROL1,	0x00}, 0x80:enable adc
	//{NU2115_ADC_FN_DIS,	0x07},//0x06:TSBUS TSBAT mean {NU2115_ADC_CONTROL2,	0x06}
	//{NU2115_TSBUS_FLT,	0x15},
	//{NU2115_TSBAT_FLG,	0x15},
	//{NU2115_TDIE_ALM,	0x48},//0x48:60C
	{NU2115_IBUS_UCP,	0xE8},
	//{NU2115_VAC12PRET,	0x01},
	//{NU2115_ACDRV12_CTRL,   0x80},
	{NU2115_P2VOUT_UOVP,    0x70},
	{NU2115_DEGLITC_REG,    0x0D},
	{NU2115_CP_OPTION,      0x00},
	{NU2115_CP_OPTION1,     0x00},
	//{NU2115_CP_OPTION2,     0x27},
};

static struct reg_default nu2115_reg_defs[] = {
	{NU2115_BATOVP,        0x37},
	{NU2115_BATOVP_ALM,    0x2F},
	{NU2115_BATOCP,        0x41},
	{NU2115_BATOCP_ALM,    0x3E},
	{NU2115_BATUCP_ALM,    0x80},
	{NU2115_AC1PROT,       0x06},
	{NU2115_AC2PROT,       0x07},
	{NU2115_BUSOVP,        0x1E},
	{NU2115_BUSOVP_ALM,    0x1D},
	{NU2115_BUSOCP,        0x01},
	{NU2115_BUSOCP_ALM,    0x00},
	{NU2115_VOUTOVP,       0x00},
	{NU2115_CON_STAT,      0x00},
	{NU2115_CTRL_REG,      0x20},
	{NU2115_CHGCTRL,       0x07},
	{NU2115_INT_STAT,      0x00},
	{NU2115_INT_FLAG,      0x00},
	{NU2115_INT_MASK,      0x00},
	{NU2115_FLT_STAT,      0x00},
	{NU2115_FLT_FLAG,      0x00},
	{NU2115_FLT_MASK,      0x00},
	{NU2115_ADC_CTRL,      0x00},
	{NU2115_ADC_FN_DIS,    0x07},
	{NU2115_IBUS_ADC_MSB,  0x00},
	{NU2115_IBUS_ADC_LSB,  0x00},
	{NU2115_VBUS_ADC_MSB,  0x00},
	{NU2115_VBUS_ADC_LSB,  0x00},
	{NU2115_VAC1_ADC_MSB,  0x00},
	{NU2115_VAC1_ADC_LSB,  0x00},
	{NU2115_VAC2_ADC_MSB,  0x00},
	{NU2115_VAC2_ADC_LSB,  0x00},
	{NU2115_VOUT_ADC_MSB,  0x00},
	{NU2115_VOUT_ADC_LSB,  0x00},
	{NU2115_VBAT_ADC_MSB,  0x00},
	{NU2115_VBAT_ADC_LSB,  0x00},
	{NU2115_IBAT_ADC_MSB,  0x00},
	{NU2115_IBAT_ADC_LSB,  0x00},
	{NU2115_TSBUS_ADC_MSB, 0x00},
	{NU2115_TSBUS_ADC_LSB, 0x00},
	{NU2115_TSBAT_ADC_MSB, 0x00},
	{NU2115_TSBAT_ADC_LSB, 0x00},
	{NU2115_TDIE_ADC_MSB,  0x00},
	{NU2115_TDIE_ADC_LSB,  0x00},
	{NU2115_TSBUS_FLT,     0x15},
	{NU2115_TSBAT_FLG,     0x15},
	{NU2115_TDIE_ALM,      0xC3},
	{NU2115_IBUS_UCP,      0xE0},
	{NU2115_VAC12PRET,     0x01},
	{NU2115_ACDRV12_CTRL,  0x80},
	{NU2115_DEV_INFO,      0x90},
	{NU2115_P2VOUT_UOVP,   0x50},
	{NU2115_DEGLITC_REG,   0x0D},
	{NU2115_CP_OPTION,     0x08},
	{NU2115_CP_OPTION1,    0x00},
	{NU2115_CP_OPTION2,    0x27},
};


static void dump_all_reg(struct nu2115_device *bq)
{
	int ret;
	unsigned int val;
	int addr;

	for (addr = 0x00; addr <= 0x37; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_err(bq->dev, "%s_dump_registe:Reg[%02X] = 0x%02X\n", bq->model_name, addr, val);
	}
}

static int nu2115_set_adc_enable(struct nu2115_device *bq, bool enable)
{
	int ret;

	dev_notice(bq->dev, "%s-%s-%d", __FUNCTION__, bq->model_name, enable);

	if (enable)
		ret = regmap_update_bits(bq->regmap, NU2115_ADC_CTRL,
				NU2115_ADC_EN, NU2115_ADC_EN);
	else
		ret = regmap_update_bits(bq->regmap, NU2115_ADC_CTRL,
				NU2115_ADC_EN, 0);


	return ret;
}

static int nu2115_get_part_no(struct nu2115_device *bq)
{
	struct i2c_client client;
	int ret;
	int len;
	const char *nu2115_name;

	ret = device_property_read_u32(bq->dev, "nu2115-addr",
				       &bq->nu2115_addr);
	if (ret)
		return -ENXIO;

	client = *(bq->client);
	pr_info("nu2115_get_part_no: orig addr = %d, nu2115 addr =%d\n ",
		client.addr, bq->nu2115_addr);
	client.addr =  bq->nu2115_addr;
	ret = i2c_smbus_read_byte_data(&client, NU2115_DEV_INFO);

	if(ret == NU2115_PART_NO || ret == NU2115A_PART_NO) {
		memset((void*)bq->model_name, 0x00, sizeof(bq->model_name));
		if(!device_property_read_string(bq->dev, "nu2115-name", &nu2115_name)){
			len = strlen(nu2115_name);
			strncpy(bq->model_name, nu2115_name, min(I2C_NAME_SIZE,len) );
		} else
			strncpy(bq->model_name, "nu2115-standalone", I2C_NAME_SIZE);

		pr_err("[%s] model_name=%s\n", __func__ , bq->model_name);
	}

	return ret;
}

static int nu2115_get_const_charge_curr(struct nu2115_device *bq)
{
	unsigned int batocp_reg_code;
	unsigned int curr_value;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_BATOCP, &batocp_reg_code);
	if (ret)
		return ret;

	curr_value = (batocp_reg_code & NU2115_BATOCP_MASK) *
							NU2115_BATOCP_STEP_uA + NU2115_BATOCP_OFFSET_uA;

	return curr_value;
}

static int nu2115_get_const_charge_volt(struct nu2115_device *bq)
{
	unsigned int batovp_reg_code;
	unsigned int volt_value;
	int ret;
	ret = regmap_read(bq->regmap, NU2115_BATOVP, &batovp_reg_code);
	if (ret)
		return ret;

	volt_value = ((batovp_reg_code * bq->chip_info->batovp_step) +
			bq->chip_info->batovp_offset);


	return volt_value;
}

/*(PMID/n-VOUT)/VOUT
is used to protect output short during switching :
n = 2 at 2:1 mode;
n = 1 at 1:1 mode;
00: 5%
01: 7.5% (default)
10: 10%
11: 12.5%*/
static int nu2115_set_pmid2vout_ovp(struct nu2115_device *bq, int val)
{
	int ret;

	ret = regmap_update_bits(bq->regmap, NU2115_P2VOUT_UOVP,
				0x30, val << 4);
	if (ret) {
		dev_err(bq->dev, "%s write NU2115_P2VOUT_UOVP fail, ret = %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static int nu2115_set_chg_en(struct nu2115_device *bq, bool en_chg)
{
	int ret;

	if (en_chg) {
		ret = nu2115_set_pmid2vout_ovp(bq, 3);
		if (ret)
			return ret;

		ret = regmap_update_bits(bq->regmap, NU2115_CHGCTRL,
					NU2115_CHG_EN, NU2115_CHG_EN);
		if (ret)
			return ret;
		mdelay(30);
		ret = nu2115_set_pmid2vout_ovp(bq, 1);
	} else
		ret = regmap_update_bits(bq->regmap, NU2115_CHGCTRL,
				NU2115_CHG_EN, en_chg);
	if (ret)
		return ret;


	bq->state.ce = en_chg;

	return 0;
}


static int nu2115_is_chg_en(struct nu2115_device *bq, bool *en_chg)
{
	unsigned int stat5;
	unsigned int chg_ctrl_2;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_CHGCTRL, &chg_ctrl_2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CON_STAT, &stat5);
	if (ret)
		return ret;

	*en_chg = (!!(chg_ctrl_2 & NU2115_CHG_EN) &
		 !!(stat5 & NU2115_SWITCHING_STAT));


	return 0;
}

static int nu2115_get_adc_ibus(struct nu2115_device *bq)
{
	int ibus_adc_lsb, ibus_adc_msb;
	u16 ibus_adc;
	int ret;


	ret = regmap_read(bq->regmap, NU2115_IBUS_ADC_MSB, &ibus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_IBUS_ADC_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

	if (ibus_adc_msb & NU2115_ADC_POLARITY_BIT)
		ibus_adc = ((ibus_adc ^ 0xffff) + 1);//mA
	return ibus_adc * 1000;
}

static int nu2115_get_adc_vbus(struct nu2115_device *bq)
{
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;
	int ret;


	ret = regmap_read(bq->regmap, NU2115_VBUS_ADC_MSB, &vbus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_VBUS_ADC_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

	if (vbus_adc_msb & NU2115_ADC_POLARITY_BIT)
		vbus_adc = ((vbus_adc ^ 0xffff) + 1);//mA
	return vbus_adc * 1000;
}

static int nu2115_get_adc_vout(struct nu2115_device *bq)
{
	int vout_adc_lsb, vout_adc_msb;
	u16 vout_adc;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_VOUT_ADC_MSB, &vout_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read NU2115_VOUT_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq->regmap, NU2115_VOUT_ADC_LSB, &vout_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read NU2115_VOUT_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	vout_adc = (vout_adc_msb << 8) | vout_adc_lsb;

	return vout_adc * 1000;
}

static int nu2115_get_adc_vbat(struct nu2115_device *bq)
{
	int vsys_adc_lsb, vsys_adc_msb;
	u16 vsys_adc;
	int ret;


	ret = regmap_read(bq->regmap, NU2115_VBAT_ADC_MSB, &vsys_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_VBAT_ADC_LSB, &vsys_adc_lsb);
	if (ret)
		return ret;

	vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

	if (vsys_adc_msb & NU2115_ADC_POLARITY_BIT)
		vsys_adc = ((vsys_adc ^ 0xffff) + 1);//mA

	return vsys_adc * 1000;
}

#ifdef CONFIG_MOTO_CHANNEL_SWITCH
static int nu2115_get_adc_vac1(struct nu2115_device *bq)
{
	int vac1_stat;
	int ret = 0;

	ret = regmap_read(bq->regmap, NU2115_VAC12PRET, &vac1_stat);
	dev_err(bq->dev,"the vac1 stat = :%x",vac1_stat);
	if (ret) {
		return 0;
	}

	if((vac1_stat& VAC1_STAT_MASK) >> VAC1_STAT_SHIFT){
		ret = 1;//VAC_ONLINE
        } else {
		ret = 0;//VAC_NOT_ONLINE
        }
	pr_err("[%s,%d] vac1_stat = %d, online = %d", __func__, __LINE__, vac1_stat, ret);

	return ret;
}

static int nu2115_get_adc_vac2(struct nu2115_device *bq)
{
	int vac2_adc_lsb, vac2_adc_msb;
	u16 vac2_adc;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_VAC2_ADC_MSB, &vac2_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read NU2115_VAC2_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq->regmap, NU2115_VAC2_ADC_LSB, &vac2_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read NU2115_VAC2_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	vac2_adc = (vac2_adc_msb << 8) | vac2_adc_lsb;

	pr_err("[%s,%d] vac2_adc = %d", __func__, __LINE__, vac2_adc);

	return vac2_adc;
}
#endif

static int nu2115_notify_state(struct nu2115_device *bq,
				struct nu2115_state *state)
{

	if (state->vbus_ovp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_VBUS_OVP);
	else if  (state->bat_ovp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_BAT_OVP);
	else if  (state->vout_ovp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_VOUTOVP);
	else if  (state->bus_ocp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_IBUSOCP);
	else if  (state->bat_ocp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_IBATOCP);
	else if  (state->bus_ucp)
		charger_dev_notify(bq->chg_dev,
			CHARGER_DEV_NOTIFY_IBUSUCP_FALL);

	return 0;
}

static int nu2115_get_state(struct nu2115_device *bq,
				struct nu2115_state *state)
{
	unsigned int chg_ctrl;
	unsigned int ac1_ovp;
	unsigned int ac2_ovp;
	unsigned int vout_ovp;
	unsigned int bus_ucp;
	unsigned int wdt_flag;
	unsigned int flt_stat;
	unsigned int ac_online;
	unsigned int bus_online;
	unsigned int ibat_adc_msb;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_AC1PROT, &ac1_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_AC2PROT, &ac2_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_FLT_STAT, &flt_stat);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_VOUTOVP, &vout_ovp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_BUSOCP_ALM, &bus_ucp);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CTRL_REG, &wdt_flag);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_VAC12PRET, &ac_online);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CP_OPTION2, &bus_online);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_CHGCTRL, &chg_ctrl);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	state->dischg = ibat_adc_msb & NU2115_ADC_POLARITY_BIT;
	state->vac_ovp = (ac1_ovp & NU2115_AC_OVP_MASK) | (ac2_ovp & NU2115_AC_OVP_MASK);
	state->bat_ovp = flt_stat & NU2115_BAT_OVP_MASK;
	state->vout_ovp = vout_ovp & NU2115_OUT_OVP_MASK;
	state->vbus_ovp = flt_stat & NU2115_BUS_OVP_MASK;
	state->bus_ocp = flt_stat & NU2115_BUS_OCP_MASK;
	state->bat_ocp = flt_stat & NU2115_BAT_OCP_MASK;
	state->bus_ucp = bus_ucp & NU2115_BUS_UCP_MASK;
	state->tflt = flt_stat & NU2115_TFLT_MASK;
	state->wdt = wdt_flag & NU2115_WDT_FLAG_MASK;
	state->online = (ac_online & NU2115_AC_PRESENT_MASK) | (bus_online & NU2115_BUS_PRESENT_MASK);
	state->ce = chg_ctrl & NU2115_CHG_EN;
	//state->hiz = chg_ctrl_2 & BQ25980_EN_HIZ;
	state->bypass = chg_ctrl & NU2115_EN_BYPASS;

	dev_info(bq->dev, "dc=%d,ovp=%d,%d,%d,%d,ocp=%d,%d,ucp=%d,t=%d,wdt=%d,online=%d,ce=%d,bypass=%d\n",
			state->dischg, state->vac_ovp, state->bat_ovp,
			state->vout_ovp, state->vbus_ovp,  state->bus_ocp,
			state->bat_ocp, state->bus_ucp, state->tflt, state->wdt,
			state->online, state->ce, state->bypass);

	return 0;
}

static int nu2115_set_charger_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	switch (prop) {

	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_get_charger_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct nu2115_device *bq = power_supply_get_drvdata(psy);
	struct nu2115_state state;
	int ret = 0;
	unsigned int ac1_ovp;
	unsigned int ac2_ovp;
	unsigned int vout_ovp;
	unsigned int flt_stat;
	unsigned int ibat_adc_msb;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = NU2115_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

		ret = regmap_read(bq->regmap, NU2115_AC1PROT, &ac1_ovp);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_AC2PROT, &ac2_ovp);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_FLT_STAT, &flt_stat);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_VOUTOVP, &vout_ovp);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, NU2115_IBAT_ADC_MSB, &ibat_adc_msb);
		if (ret)
			return ret;

		state.ovp = (ac1_ovp & NU2115_AC_OVP_MASK) | (ac2_ovp & NU2115_AC_OVP_MASK)
	            | (flt_stat & NU2115_BUS_OVP_MASK) | (flt_stat & NU2115_BAT_OVP_MASK) | (vout_ovp & NU2115_OUT_OVP_MASK);
		state.ocp = (flt_stat & NU2115_BUS_OCP_MASK) | (flt_stat & NU2115_BAT_OCP_MASK);
		state.tflt = flt_stat & NU2115_TFLT_MASK;


		if (state.tflt)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (state.ovp)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.ocp)
			val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;

		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nu2115_get_adc_ibus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nu2115_get_adc_vbat(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = nu2115_get_const_charge_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = nu2115_get_const_charge_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool nu2115_state_changed(struct nu2115_device *bq,
				  struct nu2115_state *new_state)
{
	struct nu2115_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.dischg != new_state->dischg ||
		old_state.vac_ovp != new_state->vac_ovp ||
		old_state.bat_ovp != new_state->bat_ovp ||
		old_state.vout_ovp != new_state->vout_ovp ||
		old_state.vbus_ovp != new_state->vbus_ovp ||
		old_state.bat_ocp != new_state->bat_ocp ||
		old_state.bus_ocp != new_state->bus_ocp ||
		old_state.bus_ucp != new_state->bus_ucp ||
		old_state.online != new_state->online ||
		old_state.wdt != new_state->wdt ||
		old_state.tflt != new_state->tflt ||
		old_state.ce != new_state->ce ||
		old_state.bypass != new_state->bypass);
}

static irqreturn_t nu2115_irq_handler_thread(int irq, void *private)
{
	struct nu2115_device *bq = private;
	struct nu2115_state state;
	int ret;

	dev_err(bq->dev,"[%s]%s enter\n",bq->model_name, __func__);
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

	ret = nu2115_get_state(bq, &state);
	if (ret < 0) {
		mutex_unlock(&bq->irq_complete);
		goto irq_out;
	}

	if (!nu2115_state_changed(bq, &state)) {
		mutex_unlock(&bq->irq_complete);
		goto irq_out;
	}

	dump_all_reg(bq);
	mutex_unlock(&bq->irq_complete);

	mutex_lock(&bq->lock);
	bq->state = state;
	nu2115_notify_state(bq, &state);
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static void nu2115_dump_register_work(struct work_struct *work)
{
	struct nu2115_device *bq = container_of(work, struct nu2115_device, dump_register_work.work);

	dump_all_reg(bq);

	//schedule_delayed_work(&bq->dump_register_work, 5 * HZ);
}

static enum power_supply_property nu2115_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int nu2115_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {

	default:
		return false;
	}
}



static bool nu2115_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NU2115_AC1PROT:
	case NU2115_AC2PROT...NU2115_FLT_FLAG:
	case NU2115_ADC_CTRL...NU2115_TDIE_ADC_LSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config nu2115_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = NU2115_CTRL6_REG,
	.reg_defaults	= nu2115_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(nu2115_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = nu2115_is_volatile_reg,
};

static const struct nu2115_chip_info nu2115_chip_info_tbl[] = {
	[NU2115] = {
		.model_id = NU2115,
		.regmap_config = &nu2115_regmap_config,
		.reg_init_values = nu2115_reg_init_val,

		.busocp_sc_def = NU2115_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = NU2115_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = NU2115_BUSOCP_MIN_uA,
		.busocp_sc_max = NU2115_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = NU2115_BUSOCP_MIN_uA,
		.busocp_byp_max = NU2115_BUSOCP_BYP_MAX_uA,
		.busocp_step = NU2115_BUSOCP_STEP_uA,
		.busocp_offset = NU2115_BUSOCP_OFFSET_uA,

		.busovp_sc_def = NU2115_BUSOVP_DFLT_uV,
		.busovp_byp_def = NU2115_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = NU2115_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = NU2115_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = NU2115_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = NU2115_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = NU2115_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = NU2115_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = NU2115_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = NU2115_BUSOVP_BYP_MAX_uV,

		.batovp_def = NU2115_BATOVP_DFLT_uV,
		.batovp_max = NU2115_BATOVP_MAX_uV,
		.batovp_min = NU2115_BATOVP_MIN_uV,
		.batovp_step = NU2115_BATOVP_STEP_uV,
		.batovp_offset = NU2115_BATOVP_OFFSET_uV,

		.batocp_def = NU2115_BATOCP_DFLT_uA,
		.batocp_max = NU2115_BATOCP_MAX_uA,

		.adc_curr_step = NU2115_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = NU2115_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = NU2115_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = 0,
		.adc_vout_volt_step = NU2115_ADC_VOLT_STEP_VOUT_deciuV,
		.adc_vout_volt_offset = 0,
	},
};

static int nu2115_power_supply_init(struct nu2115_device *bq,
							struct device *dev,
							int driver_data)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
						.of_node = dev->of_node, };

	switch (driver_data) {
	case NU2115_MASTER:
		bq->psy_desc.name = "cp-master";
		break;
	case NU2115_SLAVE:
		bq->psy_desc.name = "cp-slave";
		break;
	case NU2115_STANDALONE:
		bq->psy_desc.name = "cp-standalone";
		break;
	default:
		return -EINVAL;
	}

	bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS,
	bq->psy_desc.properties = nu2115_power_supply_props,
	bq->psy_desc.num_properties = ARRAY_SIZE(nu2115_power_supply_props),
	bq->psy_desc.get_property = nu2115_get_charger_property,
	bq->psy_desc.set_property = nu2115_set_charger_property,
	bq->psy_desc.property_is_writeable = nu2115_property_is_writeable,

	bq->charger = devm_power_supply_register(bq->dev,
						 &bq->psy_desc,
						 &psy_cfg);
	if (IS_ERR(bq->charger)) {
		dev_err(bq->dev, "bq register power supply fail");
		return -EINVAL;
	}

	return 0;
}

static int nu2115_reg_init(struct nu2115_device *bq)
{
	int i, ret;

	/*reg reset*/
//	ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
//		BQ25980_REG_RESET, BQ25980_REG_RESET);

	for (i = 0; i < ARRAY_SIZE(nu2115_reg_init_val); i++) {
		ret = regmap_update_bits(bq->regmap, bq->chip_info->reg_init_values[i].reg,
			0xFF, bq->chip_info->reg_init_values[i].def);
		dev_notice(bq->dev, "init Reg[%02X] = 0x%02X\n",
			bq->chip_info->reg_init_values[i].reg,
			bq->chip_info->reg_init_values[i].def);
		if (ret)
		{
			dev_err(bq->dev, "Reg init fail ret=%d", ret);
			return ret;
		}
	}
	return 0;
}
#if 0
static int nu2115_parse_dt(struct nu2115_device *bq)
{
	int ret;

	ret = device_property_read_u32(bq->dev, "ti,watchdog-timeout-ms",
				       &bq->watchdog_timer);
	if (ret)
		bq->watchdog_timer = BQ25980_WATCHDOG_MIN;

	if (bq->watchdog_timer > BQ25980_WATCHDOG_MAX ||
	    bq->watchdog_timer < BQ25980_WATCHDOG_MIN)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ovp-limit-microvolt",
				       &bq->init_data.sc_vlim);
	if (ret)
		bq->init_data.sc_vlim = bq->chip_info->busovp_sc_def;

	if (bq->init_data.sc_vlim > bq->chip_info->busovp_sc_max ||
	    bq->init_data.sc_vlim < bq->chip_info->busovp_sc_min) {
		dev_err(bq->dev, "SC ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ocp-limit-microamp",
				       &bq->init_data.sc_ilim);
	if (ret)
		bq->init_data.sc_ilim = bq->chip_info->busocp_sc_def;

	if (bq->init_data.sc_ilim > bq->chip_info->busocp_sc_max ||
	    bq->init_data.sc_ilim < bq->chip_info->busocp_sc_min) {
		dev_err(bq->dev, "SC ocp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ovp-limit-microvolt",
				       &bq->init_data.bypass_vlim);
	if (ret)
		bq->init_data.bypass_vlim = bq->chip_info->busovp_byp_def;

	if (bq->init_data.bypass_vlim > bq->chip_info->busovp_byp_max ||
	    bq->init_data.bypass_vlim < bq->chip_info->busovp_byp_min) {
		dev_err(bq->dev, "Bypass ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ocp-limit-microamp",
				       &bq->init_data.bypass_ilim);
	if (ret)
		bq->init_data.bypass_ilim = bq->chip_info->busocp_byp_def;

	if (bq->init_data.bypass_ilim > bq->chip_info->busocp_byp_max ||
	    bq->init_data.bypass_ilim < bq->chip_info->busocp_byp_min) {
		dev_err(bq->dev, "Bypass ocp limit is out of range\n");
		return -EINVAL;
	}


	bq->state.bypass = device_property_read_bool(bq->dev,
						      "ti,bypass-enable");
	return 0;
}
#endif
static int nu2115_check_work_mode(struct nu2115_device *bq)
{
	int ret;
	int val;

	ret = regmap_read(bq->regmap, NU2115_IBUS_UCP, &val);
	if (ret) {
		dev_err(bq->dev, "Failed to read operation mode register\n");
		return ret;
	}

	val = ((val & NU2115_MS_MASK)>>3);

	if (bq->mode != val) {
		dev_err(bq->dev, "dts mode %d mismatch with hardware mode %d\n", bq->mode, val);
		return -EINVAL;
	}

	dev_info(bq->dev, "work mode:%s\n", bq->mode == NU_STANDALONE ? "Standalone" :
			(bq->mode == NU_SLAVE ? "Slave" : "Master"));
	return 0;
}


static int nu2115_parse_dt_id(struct nu2115_device *bq, int driver_data)
{
	switch (driver_data) {
	case NU2115_STANDALONE:
		bq->device_id = NU2115;
		bq->mode = NU_STANDALONE;
		break;
	case NU2115_SLAVE:
		bq->device_id = NU2115;
		bq->mode = NU_SLAVE;
		break;
	case NU2115_MASTER:
		bq->device_id = NU2115;
		bq->mode = NU_MASTER;
		break;
	default:
		dev_err(bq->dev, "dts compatible id %d is unknown", driver_data);
		return -EINVAL;
		break;
	}

	return 0;
}

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", bq->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);


static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", bq->reg_addr, bq->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_data = tmp;
	regmap_write(bq->regmap, bq->reg_addr, bq->reg_data);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	bool enable;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = nu2115_is_chg_en(bq, &enable);
	if (ret < 0) {
		pr_err("nu2115: nu2115_is_chg_en not valid\n");
		state = -ENODEV;
		goto end;
	}
	state = enable;
end:
	return sprintf(buf, "%d\n", state);
}

static ssize_t store_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	bool enable;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = nu2115_set_chg_en(bq, enable);
	if (ret) {
		pr_err("nu2115 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("nu2115  %s charging \n",
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static ssize_t show_reg_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	int addr;
	ssize_t size = 0;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	for (addr = 0; addr <= 0x3a; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_err(bq->dev, "Reg[%02X] = 0x%02X\n", addr, val);
		size += snprintf(buf + size, PAGE_SIZE - size,
				"reg addr 0x%08x -> 0x%08x\n", addr,val);
	}

	return size;
}
static DEVICE_ATTR(reg_dump, 0444, show_reg_dump, NULL);

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct nu2115_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}
	vbus = nu2115_get_adc_vbus(bq);

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static void nu2115_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_reg_dump);
    device_create_file(dev, &dev_attr_vbus);
}

static int nu2115_enable_chg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct nu2115_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = nu2115_set_chg_en(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	pr_err("[%s,%d]enable chg dump register -----start-----", __func__, __LINE__);
	dump_all_reg(bq);
	pr_err("[%s,%d]enable chg dump register -----end-----", __func__, __LINE__);
	return 0;
}

static int nu2115_enable_adc(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct nu2115_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = nu2115_set_adc_enable(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}

static int nu2115_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret;
	struct nu2115_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	ret = nu2115_is_chg_en(bq, en);
	if (ret < 0) {
		dev_err(bq->dev, "%s get chg en fail %d\n", __func__, *en);
		return ret;
	}
	dev_info(bq->dev, "%s %d\n", __func__, *en);

	return 0;
}

static int nu2115_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	int tmp;
	struct nu2115_device *bq  = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}


	switch (chan) {
		case ADC_CHANNEL_VBUS:
			tmp = nu2115_get_adc_vbus(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_IBUS:
			tmp = nu2115_get_adc_ibus(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_VBAT:
			tmp = nu2115_get_adc_vbat(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_TEMP_JC:
			/*cp die temp*/
			*max = 25;
			break;
		case ADC_CHANNEL_VOUT:
			tmp = nu2115_get_adc_vout(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		default:
			return -ENOTSUPP;
			break;
	}
	*min = *max;

	return 0;
}

static int nu2115_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	unsigned int val;
	int tmp;
	struct nu2115_device *bq  = charger_get_data(chg_dev);

	tmp = regmap_read(bq->regmap, NU2115_CP_OPTION1, &val);
	if (tmp)
		return tmp;

	*err = !!(val & NU2115_VBUS_ERRPRLO_STAT);

	return 0;
}

static int nu2115_is_vbushigherr(struct charger_device *chg_dev, bool *err)
{
	unsigned int val;
	int tmp;
	struct nu2115_device *bq  = charger_get_data(chg_dev);

	tmp = regmap_read(bq->regmap, NU2115_CP_OPTION1, &val);
	if (tmp)
		return tmp;

	*err = !!(val & NU2115_VBUS_ERRPRHI_STAT);

	return 0;
}

static int nu2115_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
	*min = *max = 0;

	return 0;
}

static int nu2115_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int nu2115_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int nu2115_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int nu2115_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int nu2115_init_chip(struct charger_device *chg_dev)
{
	return 0;
}

static int nu2115_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int nu2115_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int nu2115_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int nu2115_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static bool nu2115_is_vbusovp_en(struct nu2115_device *bq)
{
	unsigned int state;
	int ret;

	ret = regmap_read(bq->regmap, NU2115_BUSOVP, &state);
	if (ret)
		return ret;

	ret = !!(state & NU2115_DIS_VBUSOVP);

	return ret;
}

static int nu2115_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	unsigned int val;
	struct nu2115_device *bq  = charger_get_data(chg_dev);

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
		ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
			NU2115_EN_OTG, 0);
		if (ret) {
			dev_err(bq->dev, "%s:mmi_mux close en otg fail ret=%d", __func__, ret);
			return ret;
		}
        }

        if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }
        if (wls_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_WLC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }

        if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, NU2115_ENABLE_TYPEC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
                return ret;
            }

            if (nu2115_is_vbusovp_en(bq)) {
                ret = regmap_update_bits(bq->regmap, NU2115_BUSOVP,NU2115_DIS_VBUSOVP,0);
                dev_err(bq->dev, "%s need open vbus ovp function\n", __func__);
            }

        } else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
            ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
                NU2115_EN_OTG, NU2115_EN_OTG);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
                return ret;
            }

            udelay(100);

            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, NU2115_ENABLE_TYPEC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
        } else if (typec_mos == MMI_DVCHG_MUX_OTG_WLC_OPEN) {
            ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
                NU2115_EN_OTG, NU2115_EN_OTG);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
                return ret;
            }

            udelay(100);

            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
#endif
        }

        if (wls_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_WLC_MOS, NU2115_ENABLE_WLC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  open wls mux fail ret=%d", __func__, ret);
                return ret;
            }
        } else if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = regmap_update_bits(bq->regmap, NU2115_VAC12PRET,
                    NU2115_EN_OTG, NU2115_EN_OTG);
	    ret = regmap_read(bq->regmap, NU2115_VAC12PRET, &val);
        	if (!ret)
                    dev_err(bq->dev, "%s:NU2115_VAC12PRET = 0x%02X\n", __func__,val);

            ret = regmap_update_bits(bq->regmap, NU2115_BUSOVP,
                    NU2115_DIS_VBUSOVP, NU2115_DIS_VBUSOVP);

            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux menu close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            mdelay(200);
            ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_WLC_MOS, NU2115_ENABLE_WLC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        }

	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
	      ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}
	if (wls_mos == MMI_DVCHG_MUX_DISABLE) {
	     ret = regmap_update_bits(bq->regmap, NU2115_ACDRV12_CTRL,
                    NU2115_ENABLE_WLC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}

        ret = regmap_read(bq->regmap, NU2115_VAC12PRET, &val);
        if (!ret)
                dev_err(bq->dev, "%s:mmi_mux Reg NU2115_VAC12PRET] = 0x%02X\n", __func__,val);
        ret = regmap_read(bq->regmap, NU2115_ACDRV12_CTRL, &val);
        if (!ret)
                dev_err(bq->dev, "%s:mmi_mux Reg NU2115_ACDRV12_CTRL] = 0x%02X\n", __func__, val);

	return 0;
}

#ifdef CONFIG_MOTO_CHANNEL_SWITCH
static int nu2115_get_vmos_chg(struct charger_device *chg_dev, bool type, int *mos_vbus)
{
	struct nu2115_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}

	if (type) {
		*mos_vbus = nu2115_get_adc_vac1(bq);
		pr_err("vac1 val = %d\n", *mos_vbus);
	} else {
		*mos_vbus = nu2115_get_adc_vac2(bq);
		pr_err("vac2 val = %d\n", *mos_vbus);
	}
	return 0;
}
#endif

static const struct charger_ops nu2115_chg_ops = {
	.enable = nu2115_enable_chg,
	.is_enabled = nu2115_is_chg_enabled,
	.get_adc = nu2115_get_adc,
	.set_vbusovp = nu2115_set_vbusovp,
	.set_ibusocp = nu2115_set_ibusocp,
	.set_vbatovp = nu2115_set_vbatovp,
	.set_ibatocp = nu2115_set_ibatocp,
	.init_chip = nu2115_init_chip,
	.set_vbatovp_alarm = nu2115_set_vbatovp_alarm,
	.reset_vbatovp_alarm = nu2115_reset_vbatovp_alarm,
	.set_vbusovp_alarm = nu2115_set_vbusovp_alarm,
	.reset_vbusovp_alarm = nu2115_reset_vbusovp_alarm,
	.is_vbuslowerr = nu2115_is_vbuslowerr,
	.is_vbushigherr = nu2115_is_vbushigherr,
	.get_adc_accuracy = nu2115_get_adc_accuracy,
	.config_mux = nu2115_config_mux,
	.enable_adc = nu2115_enable_adc,
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	.get_vmos_chg = nu2115_get_vmos_chg,
#endif
};

static int nu2115_register_chgdev(struct nu2115_device *bq)
{
	bq->chg_prop.alias_name = bq->psy_desc.name;

	if(bq->mode == NU_SLAVE)
		bq->chg_dev = charger_device_register("secondary_dvchg", bq->dev,
						bq, &nu2115_chg_ops,
						&bq->chg_prop);
	else
		bq->chg_dev = charger_device_register("primary_dvchg", bq->dev,
						bq, &nu2115_chg_ops,
						&bq->chg_prop);
	return bq->chg_dev ? 0 : -EINVAL;
}

static int nu2115_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct nu2115_device *bq;
	int ret;//, irq_gpio, irqn;

	printk("-------nu2115 driver probe--------\n");
	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(dev, "Out of memory\n");
		return -ENOMEM;
	}

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	strncpy(bq->model_name, id->name, I2C_NAME_SIZE);

	ret = nu2115_parse_dt_id(bq, id->driver_data);
	if (ret)
		goto free_mem;

	bq->part_no = nu2115_get_part_no(bq);
	pr_info("nu2115 part no = %d, %x \n",bq->part_no,bq->client->addr);

	bq->chip_info = &nu2115_chip_info_tbl[bq->device_id];

	bq->regmap = devm_regmap_init_i2c(client,
					  bq->chip_info->regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		devm_kfree(bq->dev, bq);
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	ret = nu2115_check_work_mode(bq);
	if (ret)
		goto free_mem;
#if 0
	ret = nu2115_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		goto free_mem;
	}
#endif
#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "ti,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(bq->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		goto free_mem;
	}
	ret = gpio_request(irq_gpio, "nu2115 irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		goto free_mem;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		goto free_mem;
	}
	client->irq = irqn;
#endif

	ret = nu2115_power_supply_init(bq, dev, id->driver_data);
	if (ret)
		goto free_mem;

	ret = nu2115_reg_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto free_psy;
	}

	ret = nu2115_register_chgdev(bq);
	if (ret < 0) {
		dev_err(dev, "%s reg chgdev fail(%d)\n", __func__, ret);
		goto free_psy;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						nu2115_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret < 0) {
			dev_err(bq->dev,"request irq for irq=%d failed, ret =%d\n",
				   client->irq, ret);
			goto free_chgdev;
		}
		enable_irq_wake(client->irq);
	}

	nu2115_create_device_node(bq->dev);
	dump_all_reg(bq);

	INIT_DELAYED_WORK(&bq->dump_register_work, nu2115_dump_register_work);
	schedule_delayed_work(&bq->dump_register_work, 0);
	printk("-------nu2115 driver probe success--------%s\n",dev_name(&client->dev));
	return 0;

free_chgdev:
	charger_device_unregister(bq->chg_dev);

free_psy:
	power_supply_unregister(bq->charger);

free_mem:
	devm_kfree(bq->dev, bq);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void nu2115_charger_remove(struct i2c_client *client)
{
	struct nu2115_device *bq = i2c_get_clientdata(client);

	nu2115_set_adc_enable(bq, false);

	power_supply_unregister(bq->charger);

	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->irq_complete);
	dev_err(bq->dev,"remove Successfully\n");
}

#else
static int nu2115_charger_remove(struct i2c_client *client)
{
	struct nu2115_device *bq = i2c_get_clientdata(client);

	nu2115_set_adc_enable(bq, false);

	power_supply_unregister(bq->charger);

	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->irq_complete);
	dev_err(bq->dev,"remove Successfully\n");
	return 0;
}
#endif

static void nu2115_charger_shutdown(struct i2c_client *client)
{
	struct nu2115_device *bq = i2c_get_clientdata(client);

	/*reg reset*/
	regmap_update_bits(bq->regmap, NU2115_CTRL_REG,
		NU2115_REG_RESET, NU2115_REG_RESET);
	nu2115_set_adc_enable(bq, false);

	regmap_write(bq->regmap, NU2115_CTRL_REG, 0);

	dev_err(bq->dev,"Shutdown Successfully\n");
}

static int nu2115_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nu2115_device *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	dev_err(bq->dev, "Suspend successfully!");

	return 0;
}

static int nu2115_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nu2115_device *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
				pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int nu2115_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nu2115_device *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		nu2115_irq_handler_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->charger);
	dev_err(bq->dev,"Resume successfully!");

	return 0;
}

static const struct dev_pm_ops nu2115_pm_ops = {
	.resume		= nu2115_resume,
	.suspend_noirq = nu2115_suspend_noirq,
	.suspend	= nu2115_suspend,
};

static const struct i2c_device_id nu2115_i2c_ids[] = {
	{ "nu2115-standalone", NU2115_STANDALONE },
	{ "nu2115-master", NU2115_MASTER },
	{ "nu2115-slave", NU2115_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, nu2115_i2c_ids);

static const struct of_device_id nu2115_of_match[] = {
	{ .compatible = "ti,nu2115-standalone", .data = (void *)NU2115_STANDALONE},
	{ .compatible = "ti,nu2115-master", .data = (void *)NU2115_MASTER},
	{ .compatible = "ti,nu2115-slave", .data = (void *)NU2115_SLAVE},
	{ },
};
MODULE_DEVICE_TABLE(of, nu2115_of_match);

static struct i2c_driver nu2115_driver = {
	.driver = {
		.name = "nu2115-charger",
		.of_match_table = nu2115_of_match,
		.pm	= &nu2115_pm_ops,
	},
	.probe = nu2115_probe,
	.id_table = nu2115_i2c_ids,
	.remove		= nu2115_charger_remove,
	.shutdown	= nu2115_charger_shutdown,
};
module_i2c_driver(nu2115_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_AUTHOR("Ricardo Rivera-Matos <r-rivera-matos@ti.com>");
MODULE_DESCRIPTION("nu2115 charger driver");
MODULE_LICENSE("GPL v2");
