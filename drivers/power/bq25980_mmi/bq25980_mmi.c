// SPDX-License-Identifier: GPL-2.0
// BQ25980 Battery Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
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

#include "bq25980_charger.h"
#include <charger_class.h>

struct bq25980_state {
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
	bool hiz;
	bool bypass;

	u32 vbat_adc;
	u32 vsys_adc;
	u32 ibat_adc;
	u32 fault_status;
};

enum bq_work_mode {
	BQ_STANDALONE,
	BQ_SLAVE,
	BQ_MASTER,
};

#define BQ_MODE_COUNT 3
#define SC8541_PART_NO 0x41

enum bq_device_id {
	BQ25980 = 0,
	BQ25960 = 8,
	BQ25975,
};

enum bq_compatible_id {
	BQ25960_STANDALONE,
	BQ25960_SLAVE,
	BQ25960_MASTER,

	BQ25980_STANDALONE,
	BQ25980_SLAVE,
	BQ25980_MASTER,

	BQ25975_STANDALONE,
};

struct bq25980_chip_info {

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

struct bq25980_init_data {
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

struct bq25980_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];

	struct bq25980_init_data init_data;
	const struct bq25980_chip_info *chip_info;
	struct bq25980_state state;
	int watchdog_timer;
	int mode;
	int device_id;
	int part_no;
	int sc8541_addr;
	struct power_supply_desc psy_desc;
	int reg_addr;
	int reg_data;
	struct charger_device *chg_dev;
	struct charger_properties chg_prop;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	struct mutex irq_complete;
};

static struct reg_default bq25980_reg_init_val[] = {
	{BQ25980_BATOVP,	0x69},//0x69:9100mV
	{BQ25980_BATOVP_ALM,	0x61},//0x61:8940mV
	{BQ25980_BATOCP,	0xEE},//0xEE:disable for dual //0x64:11000mA for standalone
	{BQ25980_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{BQ25980_CHRGR_CFG_1,	0xA8},
	{BQ25980_CHRGR_CTRL_1,	0x4A},
	{BQ25980_BUSOVP,	0x3C},//0X3c:20000mV
	{BQ25980_BUSOVP_ALM,	0x32},//0X32:19000mV
	{BQ25980_BUSOCP,	0x0C},//0X0c:4200mA
	{BQ25980_REG_09,	0x8C},
	{BQ25980_TEMP_CONTROL,	0x2C},
	{BQ25980_TDIE_ALM,	0x78},//0x78:85C
	{BQ25980_TSBUS_FLT,	0x15},
	{BQ25980_TSBAT_FLG,	0x15},
	{BQ25980_VAC_CONTROL,	0xD8},//0xD8:22V
	{BQ25980_CHRGR_CTRL_2,	0x00},
	{BQ25980_CHRGR_CTRL_3,	0x74},//0x74:watchdog disable 5s,275kHz
	{BQ25980_CHRGR_CTRL_4,	0x01},
	{BQ25980_CHRGR_CTRL_5,	0x00},

	{BQ25980_MASK1,		0x00},
	{BQ25980_MASK2,		0x00},
	{BQ25980_MASK3,		0x00},
	{BQ25980_MASK4,		0x00},
	{BQ25980_MASK5,		0x80},

	{BQ25980_ADC_CONTROL1,	0x04},//sample 14 bit
	{BQ25980_ADC_CONTROL2,	0xE6},
};

static struct reg_default sc8541_reg_init_val[] = {
	{BQ25980_BATOVP,	0x4a},//0x47:4550mV 0x4a:4580mv
	{BQ25980_BATOVP_ALM,	0x42},//0x3f:4470mV 0x42:4500mv
	{BQ25980_BATOCP,	0xEE},//0xEE:disable for dual //0x46:7000mA for standalone
	{BQ25980_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{BQ25980_CHRGR_CFG_1,	0x00},
	{BQ25980_CHRGR_CTRL_1,	0x49},
	{BQ25980_BUSOVP,	0x64},//0X50:11000mV 0x64:12000mv
	{BQ25980_BUSOVP_ALM,	0x50},//0X46:10500mV 0X50:11000mV
	{BQ25980_BUSOCP,	0x0C},//0X0c:4000mA
	{BQ25980_REG_09,	0x00},
	{BQ25980_TEMP_CONTROL,	0x2C},
	{BQ25980_TDIE_ALM,	0x78},//0x78:85C
	{BQ25980_TSBUS_FLT,	0x15},
	{BQ25980_TSBAT_FLG,	0x15},
	{BQ25980_VAC_CONTROL,	0x6c},//0xb4:14v*2 for vacovp
	{BQ25980_CHRGR_CTRL_2,	0x00},
	{BQ25980_CHRGR_CTRL_3,	0x94},//0x94:watchdog disable 5s,500kHz
	{BQ25980_CHRGR_CTRL_4,	0xf1},//5m oum battery sense resister & ss_timeout is 10s
	{BQ25980_CHRGR_CTRL_5,	0x60},

	{BQ25980_MASK1,		0x00},
	{BQ25980_MASK2,		0x00},
	{BQ25980_MASK3,		0x00},
	{BQ25980_MASK4,		0x00},
	{BQ25980_MASK5,		0x00},

	{BQ25980_ADC_CONTROL1,	0x00},
	{BQ25980_ADC_CONTROL2,	0x06}, //0x26: enable vac1 vac2 adc and vout adc

};

static struct reg_default bq25960_reg_init_val[] = {
	{BQ25980_BATOVP,	0x6c},//0x69:4550mV 0x6c:4580mv
	{BQ25980_BATOVP_ALM,	0x64},//0x61:4470mV 0x64:4500mv
	{BQ25980_BATOCP,	0xEE},//0xEE:disable for dual //0x46:7000mA for standalone
	{BQ25980_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{BQ25980_CHRGR_CFG_1,	0xA8},
	{BQ25980_CHRGR_CTRL_1,	0x49},
	{BQ25980_BUSOVP,	0x64},//0X50:11000mV 0x64:12000mv
	{BQ25980_BUSOVP_ALM,	0x50},//0X46:10500mV 0X50:11000mV
	{BQ25980_BUSOCP,	0x0C},//0X0c:4000mA
	{BQ25980_REG_09,	0x8C},
	{BQ25980_TEMP_CONTROL,	0x2C},
	{BQ25980_TDIE_ALM,	0x78},//0x78:85C
	{BQ25980_TSBUS_FLT,	0x15},
	{BQ25980_TSBAT_FLG,	0x15},
	{BQ25980_VAC_CONTROL,	0x6c},//0xb4:14v*2 for vacovp
	{BQ25980_CHRGR_CTRL_2,	0x00},
	{BQ25980_CHRGR_CTRL_3,	0x94},//0x94:watchdog disable 5s,500kHz
	{BQ25980_CHRGR_CTRL_4,	0xf1},//5m oum battery sense resister & ss_timeout is 10s
	{BQ25980_CHRGR_CTRL_5,	0x60},

	{BQ25980_MASK1,		0x00},
	{BQ25980_MASK2,		0x00},
	{BQ25980_MASK3,		0x00},
	{BQ25980_MASK4,		0x00},
	{BQ25980_MASK5,		0x80},

	{BQ25980_ADC_CONTROL1,	0x04},//sample 14 bit
	{BQ25980_ADC_CONTROL2,	0x06}, //0x26: enable vac1 vac2 adc and vout adc

};

static struct reg_default bq25980_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static struct reg_default bq25975_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static struct reg_default bq25960_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static struct reg_default sc8541_reg_defs[] = {
	{BQ25980_BATOVP, 0x4a},
	{BQ25980_BATOVP_ALM, 0x42},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x0},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0x0},
	{BQ25980_TEMP_CONTROL, 0x00},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x41},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static void dump_all_reg(struct bq25980_device *bq)
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

static int bq25980_set_adc_enable(struct bq25980_device *bq, bool enable)
{
	int ret;

	dev_notice(bq->dev, "%s-%s-%d", __FUNCTION__, bq->model_name, enable);

	if (enable)
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, BQ25980_ADC_EN);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, 0);

	return ret;
}

static int bq25890_get_part_no(struct bq25980_device *bq)
{
	struct i2c_client client;
	int ret;

	ret = device_property_read_u32(bq->dev, "sc8541-addr",
				       &bq->sc8541_addr);
	if (ret)
		return -ENXIO;

	client = *(bq->client);
	pr_info("bq25890_get_part_no: orig addr = %d, sc8541 addr =%d\n ",
		client.addr, bq->sc8541_addr);
	client.addr =  bq->sc8541_addr;
	ret = i2c_smbus_read_byte_data(&client, BQ25980_DEVICE_INFO);

	return ret;
}

static int bq25980_get_const_charge_curr(struct bq25980_device *bq)
{
	unsigned int batocp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BATOCP, &batocp_reg_code);
	if (ret)
		return ret;

	return (batocp_reg_code & BQ25980_BATOCP_MASK) *
						BQ25980_BATOCP_STEP_uA;
}

static int bq25980_get_const_charge_volt(struct bq25980_device *bq)
{
	unsigned int batovp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BATOVP, &batovp_reg_code);
	if (ret)
		return ret;

	return ((batovp_reg_code * bq->chip_info->batovp_step) +
			bq->chip_info->batovp_offset);
}

static int bq25980_set_chg_en(struct bq25980_device *bq, bool en_chg)
{
	int ret;

	if (en_chg)
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_CHG_EN, BQ25980_CHG_EN);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_CHG_EN, en_chg);
	if (ret)
		return ret;

	bq->state.ce = en_chg;

	return 0;
}


static int bq25980_is_chg_en(struct bq25980_device *bq, bool *en_chg)
{
	unsigned int stat5;
	unsigned int chg_ctrl_2;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT5, &stat5);
	if (ret)
		return ret;

	*en_chg = (!!(chg_ctrl_2 & BQ25980_CHG_EN) &
            !!(stat5 & BQ25980_SWITCHING_STAT));

	return 0;
}

static int bq25980_get_adc_ibus(struct bq25980_device *bq)
{
	int ibus_adc_lsb, ibus_adc_msb;
	u16 ibus_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_IBUS_ADC_MSB, &ibus_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_IBUS_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq->regmap, BQ25980_IBUS_ADC_LSB, &ibus_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_IBUS_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

	if (ibus_adc_msb & BQ25980_ADC_POLARITY_BIT)
		return ((ibus_adc ^ 0xffff) + 1) * bq->chip_info->adc_curr_step;

	return ibus_adc * bq->chip_info->adc_curr_step;
}

static int bq25980_get_adc_vbus(struct bq25980_device *bq)
{
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_MSB, &vbus_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VBUS_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}
	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_LSB, &vbus_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VBUS_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

	return bq->chip_info->adc_vbus_volt_offset + vbus_adc * bq->chip_info->adc_vbus_volt_step /10;
}

static int bq25980_get_adc_vout(struct bq25980_device *bq)
{
	int vout_adc_lsb, vout_adc_msb;
	u16 vout_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VOUT_ADC_MSB, &vout_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VOUT_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq->regmap, BQ25980_VOUT_ADC_LSB, &vout_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VOUT_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	vout_adc = (vout_adc_msb << 8) | vout_adc_lsb;

	return bq->chip_info->adc_vout_volt_offset + vout_adc * bq->chip_info->adc_vout_volt_step /10;
}

static int bq25980_get_adc_vbat(struct bq25980_device *bq)
{
	int vsys_adc_lsb, vsys_adc_msb;
	u16 vsys_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_MSB, &vsys_adc_msb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VBAT_ADC_MSB fail ret = %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_LSB, &vsys_adc_lsb);
	if (ret) {
		dev_err(bq->dev, "read BQ25980_VBAT_ADC_LSB fail ret = %d\n", ret);
		return ret;
	}

	vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

	return vsys_adc * bq->chip_info->adc_vbat_volt_step / 10;
}

static int bq25980_notify_state(struct bq25980_device *bq,
				struct bq25980_state *state)
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

static int bq25980_get_state(struct bq25980_device *bq,
				struct bq25980_state *state)
{
	unsigned int chg_ctrl_2;
	unsigned int stat1;
	unsigned int stat2;
	unsigned int stat3;
	unsigned int stat4;
	unsigned int ibat_adc_msb;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT1, &stat1);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT2, &stat2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT4, &stat4);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	state->dischg = ibat_adc_msb & BQ25980_ADC_POLARITY_BIT;
	state->vac_ovp = stat3 & BQ25980_STAT3_OVP_MASK;
	state->bat_ovp = stat1 & BQ25980_STAT1_BATOVP_MASK;
	state->vout_ovp = stat1 & BQ25980_STAT1_VOUTOVP_MASK;
	state->vbus_ovp = stat1 & BQ25980_STAT1_VBUSOVP_MASK;
	state->bus_ocp = stat2 & BQ25980_STAT2_OCP_MASK;
	state->bat_ocp = stat1 & BQ25980_STAT1_OCP_MASK;
	state->bus_ucp = stat2 & BQ25980_STAT2_BUSUCP_MASK;
	state->tflt = stat4 & BQ25980_STAT4_TFLT_MASK;
	state->wdt = stat4 & BQ25980_WD_STAT;
	state->online = stat3 & BQ25980_PRESENT_MASK;
	state->ce = chg_ctrl_2 & BQ25980_CHG_EN;
	state->hiz = chg_ctrl_2 & BQ25980_EN_HIZ;
	state->bypass = chg_ctrl_2 & BQ25980_EN_BYPASS;

	dev_info(bq->dev, "dc=%d,ovp=%d,%d,%d,%d,ocp=%d,%d,ucp=%d,t=%d,wdt=%d,online=%d,ce=%d,hiz=%d,bypass=%d\n",
			state->dischg, state->vac_ovp, state->bat_ovp,
			state->vout_ovp, state->vbus_ovp,  state->bus_ocp,
			state->bat_ocp, state->bus_ucp, state->tflt, state->wdt,
			state->online, state->ce, state->hiz, state->bypass);

	return 0;
}

static int bq25980_set_charger_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25980_get_charger_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	struct bq25980_state state;
	int ret = 0;
	unsigned int stat1 = 0;
	unsigned int stat2 = 0;
	unsigned int stat3 = 0;
	unsigned int stat4 = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25980_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

		ret = regmap_read(bq->regmap, BQ25980_STAT1, &stat1);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT2, &stat2);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT4, &stat4);
		if (ret)
			return ret;

		state.ovp = (stat1 & BQ25980_STAT1_OVP_MASK) |
			(stat3 & BQ25980_STAT3_OVP_MASK);
		state.ocp = (stat1 & BQ25980_STAT1_OCP_MASK) |
			(stat2 & BQ25980_STAT2_OCP_MASK);
		state.tflt = stat4 & BQ25980_STAT4_TFLT_MASK;

		if (state.tflt)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (state.ovp)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.ocp)
			val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;

		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq25980_get_adc_ibus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq25980_get_adc_vbat(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25980_get_const_charge_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25980_get_const_charge_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool bq25980_state_changed(struct bq25980_device *bq,
				  struct bq25980_state *new_state)
{
	struct bq25980_state old_state;

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
		old_state.hiz != new_state->hiz ||
		old_state.bypass != new_state->bypass);
}

static irqreturn_t bq25980_irq_handler_thread(int irq, void *private)
{
	struct bq25980_device *bq = private;
	struct bq25980_state state;
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

	dump_all_reg(bq);
	mutex_unlock(&bq->irq_complete);

	ret = bq25980_get_state(bq, &state);
	if (ret < 0)
		goto irq_out;

	if (!bq25980_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	bq25980_notify_state(bq, &state);
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static enum power_supply_property bq25980_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int bq25980_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return true;
	default:
		return false;
	}
}



static bool bq25980_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BQ25980_CHRGR_CTRL_2:
	case BQ25980_STAT1...BQ25980_FLAG5:
	case BQ25980_ADC_CONTROL1...BQ25980_TDIE_ADC_LSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config bq25980_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25980_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25980_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25975_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25975_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25975_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25960_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25960_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25960_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config sc8541_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SC8541_CTRL6_REG,
	.reg_defaults	= sc8541_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(sc8541_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct bq25980_chip_info sc8541_chip_info_tbl[] = {
	[BQ25960] = {
		.model_id = BQ25960,
		.regmap_config = &sc8541_regmap_config,
		.reg_init_values = sc8541_reg_init_val,

		.busocp_sc_def = BQ25960_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = BQ25960_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25960_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_byp_max = BQ25960_BUSOCP_BYP_MAX_uA,
		.busocp_step = BQ25960_BUSOCP_STEP_uA,
		.busocp_offset = BQ25960_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25975_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25975_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25960_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25960_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25960_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25960_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25960_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25960_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25960_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25960_BUSOVP_BYP_MAX_uV,

		.batovp_def = SC8545_BATOVP_DFLT_uV,
		.batovp_max = SC8545_BATOVP_MAX_uV,
		.batovp_min = SC8545_BATOVP_MIN_uV,
		.batovp_step = SC8545_BATOVP_STEP_uV,
		.batovp_offset = SC8545_BATOVP_OFFSET_uV,

		.batocp_def = SC8545_BATOCP_DFLT_uA,
		.batocp_max = SC8545_BATOCP_MAX_uA,

		.adc_curr_step = SC8541_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = SC8541_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = SC8541_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = 0,
		.adc_vout_volt_step = SC8541_ADC_VOLT_STEP_VOUT_deciuV,
		.adc_vout_volt_offset = 0,
	},
};

static const struct bq25980_chip_info bq25980_chip_info_tbl[] = {
	[BQ25980] = {
		.model_id = BQ25980,
		.regmap_config = &bq25980_regmap_config,
		.reg_init_values = bq25980_reg_init_val,

		.busocp_sc_def = BQ25980_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = BQ25980_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = BQ25980_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25980_BUSOCP_SC_MAX_uA,
		.busocp_byp_max = BQ25980_BUSOCP_BYP_MAX_uA,
		.busocp_byp_min = BQ25980_BUSOCP_MIN_uA,
		.busocp_step = BQ25980_BUSOCP_STEP_uA,
		.busocp_offset = BQ25980_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25980_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25980_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25980_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25980_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25980_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25980_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25980_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25980_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25980_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25980_BUSOVP_BYP_MAX_uV,

		.batovp_def = BQ25980_BATOVP_DFLT_uV,
		.batovp_max = BQ25980_BATOVP_MAX_uV,
		.batovp_min = BQ25980_BATOVP_MIN_uV,
		.batovp_step = BQ25980_BATOVP_STEP_uV,
		.batovp_offset = BQ25980_BATOVP_OFFSET_uV,

		.batocp_def = BQ25980_BATOCP_DFLT_uA,
		.batocp_max = BQ25980_BATOCP_MAX_uA,

		.adc_curr_step = BQ25980_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = BQ25980_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = BQ25980_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = BQ25980_ADC_VOLT_OFFSET_VBUS,
	},

	[BQ25960] = {
		.model_id = BQ25960,
		.regmap_config = &bq25960_regmap_config,
		.reg_init_values = bq25960_reg_init_val,

		.busocp_sc_def = BQ25960_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = BQ25960_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25960_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_byp_max = BQ25960_BUSOCP_BYP_MAX_uA,
		.busocp_step = BQ25960_BUSOCP_STEP_uA,
		.busocp_offset = BQ25960_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25975_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25975_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25960_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25960_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25960_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25960_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25960_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25960_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25960_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25960_BUSOVP_BYP_MAX_uV,

		.batovp_def = BQ25960_BATOVP_DFLT_uV,
		.batovp_max = BQ25960_BATOVP_MAX_uV,
		.batovp_min = BQ25960_BATOVP_MIN_uV,
		.batovp_step = BQ25960_BATOVP_STEP_uV,
		.batovp_offset = BQ25960_BATOVP_OFFSET_uV,

		.batocp_def = BQ25960_BATOCP_DFLT_uA,
		.batocp_max = BQ25960_BATOCP_MAX_uA,

		.adc_curr_step = BQ25960_ADC_CURR_STEP_uA,
		.adc_vbat_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_offset = 0,
		.adc_vout_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vout_volt_offset = 0,
	},

	[BQ25975] = {
		.model_id = BQ25975,
		.regmap_config = &bq25975_regmap_config,

		.busocp_sc_def = BQ25975_BUSOCP_DFLT_uA,
		.busocp_byp_def = BQ25975_BUSOCP_DFLT_uA,
		.busocp_sc_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25975_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_byp_max = BQ25975_BUSOCP_BYP_MAX_uA,
		.busocp_step = BQ25960_BUSOCP_STEP_uA,
		.busocp_offset = BQ25960_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25975_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25975_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25975_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25975_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25975_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25975_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25975_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25975_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25975_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25975_BUSOVP_BYP_MAX_uV,

		.batovp_def = BQ25975_BATOVP_DFLT_uV,
		.batovp_max = BQ25975_BATOVP_MAX_uV,
		.batovp_min = BQ25975_BATOVP_MIN_uV,
		.batovp_step = BQ25975_BATOVP_STEP_uV,
		.batovp_offset = BQ25975_BATOVP_OFFSET_uV,

		.batocp_def = BQ25980_BATOCP_DFLT_uA,
		.batocp_max = BQ25980_BATOCP_MAX_uA,

		.adc_curr_step = BQ25960_ADC_CURR_STEP_uA,
		.adc_vbat_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_offset = 0,
	},

};

static int bq25980_power_supply_init(struct bq25980_device *bq,
							struct device *dev,
							int driver_data)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
						.of_node = dev->of_node, };

	switch (driver_data) {
	case BQ25980_MASTER:
		bq->psy_desc.name = "cp-master";
		break;
	case BQ25980_SLAVE:
		bq->psy_desc.name = "cp-slave";
		break;
	case BQ25980_STANDALONE:
		bq->psy_desc.name = "cp-standalone";
		break;
	case BQ25960_MASTER:
		bq->psy_desc.name = "cp-master";
		break;
	case BQ25960_SLAVE:
		bq->psy_desc.name = "cp-slave";
		break;
	case BQ25960_STANDALONE:
		bq->psy_desc.name = "cp-standalone";
		break;
	default:
		return -EINVAL;
	}

	bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS,
	bq->psy_desc.properties = bq25980_power_supply_props,
	bq->psy_desc.num_properties = ARRAY_SIZE(bq25980_power_supply_props),
	bq->psy_desc.get_property = bq25980_get_charger_property,
	bq->psy_desc.set_property = bq25980_set_charger_property,
	bq->psy_desc.property_is_writeable = bq25980_property_is_writeable,

	bq->charger = devm_power_supply_register(bq->dev,
						 &bq->psy_desc,
						 &psy_cfg);
	if (IS_ERR(bq->charger)) {
		dev_err(bq->dev, "bq register power supply fail");
		return -EINVAL;
	}

	return 0;
}

static int bq25980_reg_init(struct bq25980_device *bq)
{
	int i, ret;

	/*reg reset*/
	ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
		BQ25980_REG_RESET, BQ25980_REG_RESET);

	for (i = 0; i < ARRAY_SIZE(bq25980_reg_init_val); i++) {
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

static int bq25980_parse_dt(struct bq25980_device *bq)
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

static int bq25980_check_work_mode(struct bq25980_device *bq)
{
	int ret;
	int val;

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_5, &val);
	if (ret) {
		dev_err(bq->dev, "Failed to read operation mode register\n");
		return ret;
	}

	val = (val & BQ25980_MS_MASK);
	if (bq->mode != val) {
		dev_err(bq->dev, "dts mode %d mismatch with hardware mode %d\n", bq->mode, val);
		return -EINVAL;
	}

	dev_info(bq->dev, "work mode:%s\n", bq->mode == BQ_STANDALONE ? "Standalone" :
			(bq->mode == BQ_SLAVE ? "Slave" : "Master"));
	return 0;
}


static int bq25980_parse_dt_id(struct bq25980_device *bq, int driver_data)
{
	switch (driver_data) {
	case BQ25960_STANDALONE:
		bq->device_id = BQ25960;
		bq->mode = BQ_STANDALONE;
		break;
	case BQ25960_SLAVE:
		bq->device_id = BQ25960;
		bq->mode = BQ_SLAVE;
		break;
	case BQ25960_MASTER:
		bq->device_id = BQ25960;
		bq->mode = BQ_MASTER;
		break;
	case BQ25980_STANDALONE:
		bq->device_id = BQ25980;
		bq->mode = BQ_STANDALONE;
		break;
	case BQ25980_SLAVE:
		bq->device_id = BQ25980;
		bq->mode = BQ_SLAVE;
		break;
	case BQ25980_MASTER:
		bq->device_id = BQ25980;
		bq->mode = BQ_MASTER;
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
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", bq->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);


static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	ret = regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);;
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", bq->reg_addr, bq->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
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
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = bq25980_is_chg_en(bq, &enable);
	if (ret < 0) {
		pr_err("bq25980: bq25980_is_chg_en not valid\n");
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
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = bq25980_set_chg_en(bq, enable);
	if (ret) {
		pr_err("bq25980 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("bq25980  %s charging \n",
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
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
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
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}
	vbus = bq25980_get_adc_vbus(bq);

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static void bq25980_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_reg_dump);
    device_create_file(dev, &dev_attr_vbus);
}

static int bq25980_enable_chg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct bq25980_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = bq25980_set_chg_en(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}

static int bq25980_enable_adc(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct bq25980_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = bq25980_set_adc_enable(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}

static int bq25980_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret;
	struct bq25980_device *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}

	ret = bq25980_is_chg_en(bq, en);
	if (ret < 0) {
		dev_err(bq->dev, "%s get chg en fail %d\n", __func__, *en);
		return ret;
	}
	dev_info(bq->dev, "%s %d\n", __func__, *en);

	return 0;
}

static int bq25980_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	int tmp;
	struct bq25980_device *bq  = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("bq25980 chip not valid\n");
		return -ENODEV;
	}


	switch (chan) {
		case ADC_CHANNEL_VBUS:
			tmp = bq25980_get_adc_vbus(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_IBUS:
			tmp = bq25980_get_adc_ibus(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_VBAT:
			tmp = bq25980_get_adc_vbat(bq);
			if (tmp < 0)
				return tmp;
			*max = tmp;
			break;
		case ADC_CHANNEL_TEMP_JC:
			/*cp die temp*/
			*max = 25;
			break;
		case ADC_CHANNEL_VOUT:
			tmp = bq25980_get_adc_vout(bq);
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

static int bq25980_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	unsigned int val;
	int tmp;
	struct bq25980_device *bq  = charger_get_data(chg_dev);

	if (bq->part_no == SC8541_PART_NO) {
		tmp = regmap_read(bq->regmap, BQ25980_STAT5, &val);
		if (tmp)
			return tmp;

		*err = !!(val & SC8541_VBUS_ERRPRLO_STAT);
	} else {
		*err = 0;
	}
	return 0;
}

static int bq25980_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
	*min = *max = 0;

	return 0;
}

static int bq25980_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int bq25980_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int bq25980_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int bq25980_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int bq25980_init_chip(struct charger_device *chg_dev)
{
	return 0;
}

static int bq25980_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int bq25980_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int bq25980_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int bq25980_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int sc8541_config_mux(struct bq25980_device *bq,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	unsigned int val;

	 if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
            ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
                    BQ25980_EN_OTG, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close en otg fail ret=%d", __func__, ret);
                return ret;
            }
        }
	 if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN
		&& typec_mos != MMI_DVCHG_MUX_DISABLE && wls_mos != MMI_DVCHG_MUX_DISABLE
		 && wls_mos != MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ACDRV_MANUAL_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux dis mos both fail ret=%d", __func__, ret);
                return ret;
            }
        }

        if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }
        if (wls_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_WLC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }

        if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_TYPEC_MOS, SC8541_ENABLE_TYPEC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        } else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
            ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
                BQ25980_EN_OTG, BQ25980_EN_OTG);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
                return ret;
            }

            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ACDRV_MANUAL_EN, SC8541_ACDRV_MANUAL_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);

            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_TYPEC_MOS, SC8541_ENABLE_TYPEC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        }

        if (wls_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_WLC_MOS, SC8541_ENABLE_WLC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  open wls mux fail ret=%d", __func__, ret);
                return ret;
            }
        } else if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ACDRV_MANUAL_EN, SC8541_ACDRV_MANUAL_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux menu close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            mdelay(50);

            ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_WLC_MOS, SC8541_ENABLE_WLC_MOS);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        }


	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ACDRV_MANUAL_EN, SC8541_ACDRV_MANUAL_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
	      ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_TYPEC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}
	if (wls_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ACDRV_MANUAL_EN, SC8541_ACDRV_MANUAL_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
	     ret = regmap_update_bits(bq->regmap, SC8541_CTRL6_REG,
                    SC8541_ENABLE_WLC_MOS, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}

        ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &val);
        if (!ret)
                dev_err(bq->dev, "%s:mmi_mux Reg SC8541_CHRGR_CTRL_2] = 0x%02X\n", __func__,val);
        ret = regmap_read(bq->regmap, SC8541_CTRL6_REG, &val);
        if (!ret)
                dev_err(bq->dev, "%s:mmi_mux Reg SC8541_CTRL6_REG] = 0x%02X\n", __func__, val);

	return 0;
}

static int bq25980_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	unsigned int val;
	struct bq25980_device *bq  = charger_get_data(chg_dev);

	if (bq->part_no == SC8541_PART_NO)
		return sc8541_config_mux(bq, typec_mos, wls_mos);

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_EN_OTG, 0);
		if (ret) {
			dev_err(bq->dev, "mmi_mux close en otg fail ret=%d", ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_DIS_MOS_BOTH | BQ25980_ENABLE_TYPEC_MOS, 0);
		if (ret) {
			dev_err(bq->dev, "mmi_mux close typec mos fail ret=%d", ret);
			return ret;
		}
		udelay(1000);
	}
	if (wls_mos == MMI_DVCHG_MUX_CLOSE) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_DIS_MOS_BOTH | BQ25980_ENABLE_WLC_MOS, 0);
		if (ret) {
			dev_err(bq->dev, "mmi_mux close wls mos fail ret=%d", ret);
			return ret;
		}
		udelay(1000);
	}

	if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_DIS_MOS_BOTH, 0);
		if (ret) {
			dev_err(bq->dev, "mmi_mux open mos both fail ret=%d", ret);
			return ret;
		}
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_ENABLE_TYPEC_MOS, BQ25980_ENABLE_TYPEC_MOS);
		if (ret) {
			dev_err(bq->dev, "mmi_mux open typec mos fail ret=%d", ret);
			return ret;
		}
	} else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
		ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &val);
		if (ret) {
			dev_err(bq->dev, "mmi_mux Reg BQ25980_CHRGR_CTRL_2] fail ret=%d", ret);
			return ret;
		}
		dev_err(bq->dev, "mmi_mux typec mos 0xf reg=%d", val);
		if ((val & (BQ25980_ENABLE_TYPEC_MOS | BQ25980_EN_OTG))
			!= (BQ25980_ENABLE_TYPEC_MOS | BQ25980_EN_OTG))	 {
			ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_EN_OTG, BQ25980_EN_OTG);
			if (ret) {
				dev_err(bq->dev, "mmi_mux  en otg fail ret=%d", ret);
				return ret;
			}
			ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_DIS_MOS_BOTH, BQ25980_DIS_MOS_BOTH);
			if (ret) {
				dev_err(bq->dev, "mmi_mux dis mos both fail ret=%d", ret);
				return ret;
			}
			udelay(1000);
			ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_DIS_MOS_BOTH, 0);
			if (ret) {
				dev_err(bq->dev, "mmi_mux enable mos both fail ret=%d", ret);
				return ret;
			}
			ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_ENABLE_TYPEC_MOS, BQ25980_ENABLE_TYPEC_MOS);
			if (ret) {
				dev_err(bq->dev, "mmi_mux enable otg typec mos fail ret=%d", ret);
				return ret;
			}
		}
	}

	if (wls_mos == MMI_DVCHG_MUX_CHG_OPEN
		||wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_DIS_MOS_BOTH, 0);
		if (ret) {
			dev_err(bq->dev, "mmi_mux wls open mos both fail ret=%d", ret);
			return ret;
		}
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_ENABLE_WLC_MOS, BQ25980_ENABLE_WLC_MOS);
		if (ret) {
			dev_err(bq->dev, "mmi_mux  open wls mux fail ret=%d", ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_DISABLE || wls_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
				BQ25980_DIS_MOS_BOTH, BQ25980_DIS_MOS_BOTH);
		if (ret) {
			dev_err(bq->dev, "mmi_mux dis mos both err ret=%d", ret);
			return ret;
		}
		udelay(1000);
	}

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &val);
	if (!ret)
			dev_err(bq->dev, "mmi_mux Reg BQ25980_CHRGR_CTRL_2] = 0x%02X\n", val);
	return 0;
}

static const struct charger_ops bq25980_chg_ops = {
	.enable = bq25980_enable_chg,
	.is_enabled = bq25980_is_chg_enabled,
	.get_adc = bq25980_get_adc,
	.set_vbusovp = bq25980_set_vbusovp,
	.set_ibusocp = bq25980_set_ibusocp,
	.set_vbatovp = bq25980_set_vbatovp,
	.set_ibatocp = bq25980_set_ibatocp,
	.init_chip = bq25980_init_chip,
	.set_vbatovp_alarm = bq25980_set_vbatovp_alarm,
	.reset_vbatovp_alarm = bq25980_reset_vbatovp_alarm,
	.set_vbusovp_alarm = bq25980_set_vbusovp_alarm,
	.reset_vbusovp_alarm = bq25980_reset_vbusovp_alarm,
	.is_vbuslowerr = bq25980_is_vbuslowerr,
	.get_adc_accuracy = bq25980_get_adc_accuracy,
	.config_mux = bq25980_config_mux,
	.enable_adc = bq25980_enable_adc,
};

static int bq25980_register_chgdev(struct bq25980_device *bq)
{
	bq->chg_prop.alias_name = bq->psy_desc.name;

	if(bq->mode == BQ_SLAVE)
		bq->chg_dev = charger_device_register("secondary_dvchg", bq->dev,
						bq, &bq25980_chg_ops,
						&bq->chg_prop);
	else
		bq->chg_dev = charger_device_register("primary_dvchg", bq->dev,
						bq, &bq25980_chg_ops,
						&bq->chg_prop);
	return bq->chg_dev ? 0 : -EINVAL;
}

static int bq25980_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq25980_device *bq;
	const char *sc8541_name;
	int len;
	int ret;//, irq_gpio, irqn;

	printk("-------bq25980 driver probe--------\n");
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

	ret = bq25980_parse_dt_id(bq, id->driver_data);
	if (ret)
		goto free_mem;

	bq->part_no = bq25890_get_part_no(bq);
	pr_info("bq25980 part no = %d, %x \n",bq->part_no,bq->client->addr);

	if (bq->part_no == SC8541_PART_NO) {
		bq->client->addr =  bq->sc8541_addr;
		bq->chip_info = &sc8541_chip_info_tbl[bq->device_id];

		memset((void*)bq->model_name, 0x00, sizeof(bq->model_name));
		if(!device_property_read_string(bq->dev, "sc8541-name", &sc8541_name)) {
			len = strlen(sc8541_name);
			strncpy(bq->model_name, sc8541_name, min(I2C_NAME_SIZE,len) );
		} else
			strncpy(bq->model_name, "sc8541-standalone", I2C_NAME_SIZE);

		pr_err("[%s] model_name=%s\n", __func__ , bq->model_name);
	} else {
		bq->chip_info = &bq25980_chip_info_tbl[bq->device_id];
	}

	bq->regmap = devm_regmap_init_i2c(client,
					  bq->chip_info->regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		devm_kfree(bq->dev, bq);
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	ret = bq25980_check_work_mode(bq);
	if (ret)
		goto free_mem;

	ret = bq25980_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		goto free_mem;
	}

#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "ti,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(bq->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		goto free_mem;
	}
	ret = gpio_request(irq_gpio, "bq25980 irq pin");
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

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						bq25980_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret < 0) {
			dev_err(bq->dev,"request irq for irq=%d failed, ret =%d\n",
				   client->irq, ret);
			goto free_mem;
		}
		enable_irq_wake(client->irq);
	}

	ret = bq25980_power_supply_init(bq, dev, id->driver_data);
	if (ret)
		goto free_mem;

	ret = bq25980_reg_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto free_psy;
	}
	ret = bq25980_register_chgdev(bq);
	if (ret < 0) {
		dev_err(dev, "%s reg chgdev fail(%d)\n", __func__, ret);
		goto free_psy;
	}

	bq25980_create_device_node(bq->dev);
	dump_all_reg(bq);
	printk("-------bq25980 driver probe success--------%s\n",dev_name(&client->dev));
	return 0;
free_psy:
	power_supply_unregister(bq->charger);

free_mem:
	devm_kfree(bq->dev, bq);
	return ret;
}
static int bq25980_charger_remove(struct i2c_client *client)
{
	struct bq25980_device *bq = i2c_get_clientdata(client);

	bq25980_set_adc_enable(bq, false);

	power_supply_unregister(bq->charger);

	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->irq_complete);
	dev_err(bq->dev,"remove Successfully\n");
	return 0;
}


static void bq25980_charger_shutdown(struct i2c_client *client)
{
	struct bq25980_device *bq = i2c_get_clientdata(client);

	/*reg reset*/
	regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
		BQ25980_REG_RESET, BQ25980_REG_RESET);
	bq25980_set_adc_enable(bq, false);

	regmap_write(bq->regmap, BQ25980_CHRGR_CTRL_2, 0);

	dev_err(bq->dev,"Shutdown Successfully\n");
}

static int bq25980_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25980_device *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	dev_err(bq->dev, "Suspend successfully!");

	return 0;
}

static int bq25980_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25980_device *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
				pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq25980_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25980_device *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq25980_irq_handler_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->charger);
	dev_err(bq->dev,"Resume successfully!");

	return 0;
}

static const struct dev_pm_ops bq25980_pm_ops = {
	.resume		= bq25980_resume,
	.suspend_noirq = bq25980_suspend_noirq,
	.suspend	= bq25980_suspend,
};

static const struct i2c_device_id bq25980_i2c_ids[] = {
	{ "bq25980-standalone", BQ25980_STANDALONE },
	{ "bq25980-master", BQ25980_MASTER },
	{ "bq25980-slave", BQ25980_SLAVE },
	{ "bq25960-standalone", BQ25960_STANDALONE },
	{ "bq25960-master", BQ25960_MASTER },
	{ "bq25960-slave", BQ25960_SLAVE },
	{ "bq25975", BQ25975 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25980_i2c_ids);

static const struct of_device_id bq25980_of_match[] = {
	{ .compatible = "ti,bq25980-standalone", .data = (void *)BQ25980_STANDALONE},
	{ .compatible = "ti,bq25980-master", .data = (void *)BQ25980_MASTER},
	{ .compatible = "ti,bq25980-slave", .data = (void *)BQ25980_SLAVE},
	{ .compatible = "ti,bq25960-standalone", .data = (void *)BQ25960_STANDALONE},
	{ .compatible = "ti,bq25960-master", .data = (void *)BQ25960_MASTER},
	{ .compatible = "ti,bq25960-slave", .data = (void *)BQ25960_SLAVE},
	{ .compatible = "ti,bq25975", .data = (void *)BQ25975 },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25980_of_match);

static struct i2c_driver bq25980_driver = {
	.driver = {
		.name = "bq25980-charger",
		.of_match_table = bq25980_of_match,
		.pm	= &bq25980_pm_ops,
	},
	.probe = bq25980_probe,
	.id_table = bq25980_i2c_ids,
	.remove		= bq25980_charger_remove,
	.shutdown	= bq25980_charger_shutdown,
};
module_i2c_driver(bq25980_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_AUTHOR("Ricardo Rivera-Matos <r-rivera-matos@ti.com>");
MODULE_DESCRIPTION("bq25980 charger driver");
MODULE_LICENSE("GPL v2");
