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
#include <linux/of_device.h>
#include <linux/iio/consumer.h>

#include "bq25980_reg.h"
#include "bq25980_mmi_iio.h"

//undef because of pinctrl
#define CONFIG_INTERRUPT_AS_GPIO

/*
Pay attention to the order of the macro definitions
for alarm_status and fault_status.You can't modify
 bq25980_alarm_status and bq25980_fault_status.

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	BAT_THERM_FAULT_SHIFT			12
#define	BUS_THERM_FAULT_SHIFT			13
#define	DIE_THERM_FAULT_SHIFT			14
*/

union bq25980_alarm_status {
	struct {
	unsigned char bat_ovp_alarm:1;
	unsigned char bat_ocp_alarm:1;
	unsigned char bus_ovp_alarm:1;
	unsigned char bus_ocp_alarm:1;
	unsigned char bat_therm_alarm:1;
	unsigned char bus_therm_alarm:1;
	unsigned char die_therm_alarm:1;
	unsigned char bat_ucp_alarm:1;
	}bits;
	unsigned char status;
};

union bq25980_fault_status {
	struct {
	unsigned char bat_ovp_fault:1;
	unsigned char bat_ocp_fault:1;
	unsigned char bus_ovp_fault:1;
	unsigned char bus_ocp_fault:1;
	unsigned char bat_therm_fault:1;
	unsigned char bus_therm_fault:1;
	unsigned char die_therm_fault:1;
	unsigned char :1;
	}bits;
	unsigned char status;
};

struct bq25980_state {
	bool dischg;
	bool ovp;
	bool ocp;
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
#define BQ25960_PART_NO 0x00
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
	struct power_supply_desc psy_desc;
	struct pinctrl *irq_pinctrl;
	bool usb_present;
	int irq_counts;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	struct mutex irq_complete;

	union bq25980_alarm_status alarm_status;
	union bq25980_fault_status fault_status;
	struct iio_dev *indio_dev;
	struct iio_chan_spec *iio_chan;
	struct iio_channel *int_iio_chans;
	struct iio_channel **ext_iio_chans;

	int reg_addr;
	int reg_data;
	int part_no;
	int sc8541_addr;
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
	{BQ25980_VAC_CONTROL,	0x48},//0x48:12V*2
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
	{BQ25980_ADC_CONTROL2,	0xE6},

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

static int bq25980_reg_init(struct bq25980_device *bq);
//static int bq25980_watchdog_time[BQ25980_NUM_WD_VAL] = {5000, 10000, 50000,
//							300000};

/*static void dump_reg(struct bq25980_device *bq, int start, int end)
{
	int ret;
	unsigned int val;
	int addr;

	for (addr = start; addr <= end; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_err(bq->dev, "[%s] Reg[%02X] = 0x%02X\n", bq->model_name, addr, val);
	}
}*/

static void dump_all_reg(struct bq25980_device *bq)
{
	int ret;
	unsigned int val;
	int addr;

	for (addr = 0x00; addr <= 0x37; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_err(bq->dev, "[%s] Reg[%02X] = 0x%02X\n", bq->model_name, addr, val);
	}
}
/*
static int bq25980_get_adc_enable(struct bq25980_device *bq)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_ADC_CONTROL1, &reg_code);
	if (ret)
		return ret;

	return !!(reg_code & BQ25980_ADC_EN);
}
*/
static int bq25980_set_adc_enable(struct bq25980_device *bq, bool enable)
{
	int ret;

	dev_notice(bq->dev, "%s %d", __FUNCTION__, enable);

	if (enable)
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, BQ25980_ADC_EN);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, 0);

	return ret;
}

/*
static int bq25980_get_input_curr_lim(struct bq25980_device *bq)
{
	unsigned int busocp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BUSOCP, &busocp_reg_code);
	if (ret)
		return ret;

	return (busocp_reg_code * bq->chip_info->busocp_step) + bq->chip_info->busocp_offset;
}

static int bq25980_set_hiz(struct bq25980_device *bq, int setting)
{
	return regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
			BQ25980_EN_HIZ, setting);
}*/
/*
static int bq25980_set_input_curr_lim(struct bq25980_device *bq, int busocp)
{
	unsigned int busocp_reg_code;

	if (!busocp)
		return bq25980_set_hiz(bq, BQ25980_ENABLE_HIZ);

	bq25980_set_hiz(bq, BQ25980_DISABLE_HIZ);

	if (bq->state.bypass) {
		busocp = max(busocp, bq->chip_info->busocp_byp_min);
		busocp = min(busocp, bq->chip_info->busocp_byp_max);
	} else {
		busocp = max(busocp, bq->chip_info->busocp_sc_min);
		busocp = min(busocp, bq->chip_info->busocp_sc_max);
	}

	busocp_reg_code = (busocp - bq->chip_info->busocp_offset)
						/ bq->chip_info->busocp_step;

	return regmap_write(bq->regmap, BQ25980_BUSOCP, busocp_reg_code);
}

static int bq25980_get_input_volt_lim(struct bq25980_device *bq)
{
	unsigned int busovp_reg_code;
	unsigned int busovp_offset;
	unsigned int busovp_step;
	int ret;

	if (bq->state.bypass) {
		busovp_step = bq->chip_info->busovp_byp_step;
		busovp_offset = bq->chip_info->busovp_byp_offset;
	} else {
		busovp_step = bq->chip_info->busovp_sc_step;
		busovp_offset = bq->chip_info->busovp_sc_offset;
	}

	ret = regmap_read(bq->regmap, BQ25980_BUSOVP, &busovp_reg_code);
	if (ret)
		return ret;

	return (busovp_reg_code * busovp_step) + busovp_offset;
}

static int bq25980_set_input_volt_lim(struct bq25980_device *bq, int busovp)
{
	unsigned int busovp_reg_code;
	unsigned int busovp_step;
	unsigned int busovp_offset;
	int ret;

	if (bq->state.bypass) {
		busovp_step = bq->chip_info->busovp_byp_step;
		busovp_offset = bq->chip_info->busovp_byp_offset;
		if (busovp > bq->chip_info->busovp_byp_max)
			busovp = bq->chip_info->busovp_byp_max;
		else if (busovp < bq->chip_info->busovp_byp_min)
			busovp = bq->chip_info->busovp_byp_min;
	} else {
		busovp_step = bq->chip_info->busovp_sc_step;
		busovp_offset = bq->chip_info->busovp_sc_offset;
		if (busovp > bq->chip_info->busovp_sc_max)
			busovp = bq->chip_info->busovp_sc_max;
		else if (busovp < bq->chip_info->busovp_sc_min)
			busovp = bq->chip_info->busovp_sc_min;
	}

	busovp_reg_code = (busovp - busovp_offset) / busovp_step;

	ret = regmap_write(bq->regmap, BQ25980_BUSOVP, busovp_reg_code);
	if (ret)
		return ret;

	return regmap_write(bq->regmap, BQ25980_BUSOVP_ALM, busovp_reg_code);
}
*/
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
/*
static int bq25980_set_const_charge_curr(struct bq25980_device *bq, int batocp)
{
	unsigned int batocp_reg_code;
	int ret;

	batocp = max(batocp, BQ25980_BATOCP_MIN_uA);
	batocp = min(batocp, bq->chip_info->batocp_max);

	batocp_reg_code = batocp / BQ25980_BATOCP_STEP_uA;

	ret = regmap_update_bits(bq->regmap, BQ25980_BATOCP,
				BQ25980_BATOCP_MASK, batocp_reg_code);
	if (ret)
		return ret;

	return regmap_update_bits(bq->regmap, BQ25980_BATOCP_ALM,
				BQ25980_BATOCP_MASK, batocp_reg_code);
}
*/
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
/*
static int bq25980_set_const_charge_volt(struct bq25980_device *bq, int batovp)
{
	unsigned int batovp_reg_code;
	int ret;

	if (batovp < bq->chip_info->batovp_min)
		batovp = bq->chip_info->batovp_min;

	if (batovp > bq->chip_info->batovp_max)
		batovp = bq->chip_info->batovp_max;

	batovp_reg_code = (batovp - bq->chip_info->batovp_offset) /
						bq->chip_info->batovp_step;

	ret = regmap_write(bq->regmap, BQ25980_BATOVP, batovp_reg_code);
	if (ret)
		return ret;

	return regmap_write(bq->regmap, BQ25980_BATOVP_ALM, batovp_reg_code);
}*/
/*
static int bq25980_set_bypass(struct bq25980_device *bq, bool en_bypass)
{
	int ret;

	if (en_bypass)
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, BQ25980_EN_BYPASS);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, en_bypass);
	if (ret)
		return ret;

	bq->state.bypass = en_bypass;

	return 0;
}
*/

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
	int ret;
	unsigned int chg_ctrl_2;
	unsigned int stat5;

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
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBUS_ADC_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

	if (ibus_adc_msb & BQ25980_ADC_POLARITY_BIT)
		return (((ibus_adc ^ 0xffff) + 1) * bq->chip_info->adc_curr_step) /1000;//mA

	return (ibus_adc * bq->chip_info->adc_curr_step) /1000; //mA
}

static int bq25980_get_adc_vbus(struct bq25980_device *bq)
{
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_MSB, &vbus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

	return (bq->chip_info->adc_vbus_volt_offset + vbus_adc * bq->chip_info->adc_vbus_volt_step /10) /1000;//mV
}

static int bq25980_get_ibat_adc(struct bq25980_device *bq)
{
	int ret;
	int ibat_adc_lsb, ibat_adc_msb;
	int ibat_adc;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_LSB, &ibat_adc_lsb);
	if (ret)
		return ret;

	ibat_adc = (ibat_adc_msb << 8) | ibat_adc_lsb;

	if (ibat_adc_msb & BQ25980_ADC_POLARITY_BIT)
		return (((ibat_adc ^ 0xffff) + 1) * BQ25960_ADC_CURR_STEP_uA) /1000;//mA

	return (ibat_adc * BQ25960_ADC_CURR_STEP_uA) /1000; //mA
}

static int bq25980_get_adc_vbat(struct bq25980_device *bq)
{
	int vsys_adc_lsb, vsys_adc_msb;
	u16 vsys_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_MSB, &vsys_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_LSB, &vsys_adc_lsb);
	if (ret)
		return ret;

	vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

	return (vsys_adc * bq->chip_info->adc_vbat_volt_step / 10) /1000;//mV
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
	state->ovp = (stat1 & BQ25980_STAT1_OVP_MASK) |
		(stat3 & BQ25980_STAT3_OVP_MASK);
	state->ocp = (stat1 & BQ25980_STAT1_OCP_MASK) |
		(stat2 & BQ25980_STAT2_OCP_MASK);
	state->tflt = stat4 & BQ25980_STAT4_TFLT_MASK;
	state->wdt = stat4 & BQ25980_WD_STAT;
	state->online = stat3 & BQ25980_PRESENT_MASK;
	state->ce = chg_ctrl_2 & BQ25980_CHG_EN;
	state->hiz = chg_ctrl_2 & BQ25980_EN_HIZ;
	state->bypass = chg_ctrl_2 & BQ25980_EN_BYPASS;

	bq->alarm_status.bits.bat_ovp_alarm = stat1 & BQ25980_STAT1_BAT_OVP_ALM_MASK;
	bq->alarm_status.bits.bat_ocp_alarm = stat1 & BQ25980_STAT1_BAT_OCP_ALM_MASK;
	bq->alarm_status.bits.bus_ovp_alarm = stat1 & BQ25980_STAT1_BUS_OVP_ALM_MASK;
	bq->alarm_status.bits.bus_ocp_alarm = stat2 & BQ25980_STAT2_BUS_OCP_ALM_MASK;
	bq->alarm_status.bits.bat_therm_alarm = stat4 & BQ25980_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.bus_therm_alarm = stat4 & BQ25980_STAT4_TSBUS_TSBAT_ALM_MASK;
	bq->alarm_status.bits.die_therm_alarm = stat4 & BQ25980_STAT4_TDIE_ALM_MASK;
	bq->alarm_status.bits.bat_ucp_alarm = stat1 & BQ25980_STAT1_BAT_UCP_ALM_MASK;

	bq->fault_status.bits.bat_ovp_fault = stat1 & BQ25980_STAT1_BAT_OVP_MASK;
	bq->fault_status.bits.bat_ocp_fault = stat1 & BQ25980_STAT1_BAT_OCP_MASK;
	bq->fault_status.bits.bus_ovp_fault = stat1 & BQ25980_STAT1_BUS_OVP_MASK;
	bq->fault_status.bits.bus_ocp_fault = stat2 & BQ25980_STAT2_BUS_OCP_MASK;
	bq->fault_status.bits.bat_therm_fault = stat4 & BQ25980_STAT4_TSBAT_FLT_MASK;
	bq->fault_status.bits.bus_therm_fault = stat4 & BQ25980_STAT4_TSBUS_FLT_MASK;
	bq->fault_status.bits.die_therm_fault = stat4 & BQ25980_STAT4_TDIE_FLT_MASK;

	return 0;
}
#if 0
static int bq25980_set_battery_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25980_set_const_charge_curr(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25980_set_const_charge_volt(bq, val->intval);
		if (ret)
			return ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25980_get_battery_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->init_data.ichg_max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->init_data.vreg_max;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq25980_get_ibat_adc(bq);
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq25980_get_adc_vbat(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}
#endif
static int bq25980_set_present(struct bq25980_device *bq, bool present)
{
	bq->usb_present = present;

	if (present)
		bq25980_reg_init(bq);
	return 0;
}

static int bq25980_set_charger_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
/*
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25980_set_input_curr_lim(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25980_set_input_volt_lim(bq, val->intval);
		if (ret)
			return ret;
		break;
*/
/*	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		dev_err(bq->dev,"%s,POWER_SUPPLY_PROP_CHARGING_ENABLED prop:%d,value:%d\n",__func__,prop,val->intval);
		ret = bq25980_set_chg_en(bq, val->intval);
		if (ret)
			return ret;
		break;
*/
/*
	case POWER_SUPPLY_PROP_TI_ADC:
		ret = bq25980_set_adc_enable(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TI_BYPASS:
		ret = bq25980_set_bypass(bq, val->intval);
		if (ret)
			return ret;
		if (val->intval)
			ret = bq25980_set_input_curr_lim(bq, bq->chip_info->busocp_byp_def);
		else
			ret = bq25980_set_input_curr_lim(bq, bq->chip_info->busocp_sc_def);
		if (ret)
			return ret;
		break;
*/
	case POWER_SUPPLY_PROP_PRESENT:
		ret = bq25980_set_present(bq, !!val->intval);
		pr_info("[%s] set present :%d ret=%d\n", bq->model_name, val->intval,ret);
		//dev_err(bq->dev,"%s,POWER_SUPPLY_PROP_PRESENT prop:%d,value:%d\n",__func__,prop,val->intval);
		break;
/*	case POWER_SUPPLY_PROP_UPDATE_NOW:
		dev_err(bq->dev,"%s,POWER_SUPPLY_PROP_UPDATE_NOW prop:%d,value:%d\n",__func__,prop,val->intval);
		break;
*/
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
	//unsigned int chg_ctrl_2 = 0;
	unsigned int stat1 = 0;
	unsigned int stat2 = 0;
	unsigned int stat3 = 0;
	unsigned int stat4 = 0;

	switch (psp) {
	//case POWER_SUPPLY_PROP_CHIP_VERSION:
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25980_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
/*	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
		if (ret)
			return ret;

		state.online = stat3 & BQ25980_PRESENT_MASK;
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25980_get_input_volt_lim(bq);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25980_get_input_curr_lim(bq);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
*/
/*	case POWER_SUPPLY_PROP_CP_STATUS1:
		val->intval = 0;
			((sc->bat_ovp_fault << SC8549_VBAT_OVP_FLAG_SHIFT)
					   | (sc->bat_ocp_fault << SC8549_IBAT_OCP_FLAG_SHIFT)
					   | (sc->bus_ovp_fault << SC8549_VBUS_OVP_FLAG_SHIFT)
					   | (sc->bus_ocp_fault << SC8549_IBUS_OCP_FLAG_SHIFT));
		break;
*/
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
		ret = bq25980_get_ibat_adc(bq);
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
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25980_get_adc_vbus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25980_get_adc_ibus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		//dump_all_reg(bq);
		break;

/*	case POWER_SUPPLY_PROP_CHARGING_ENABLED://undeclared identifier
		ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
		if (ret)
			return ret;

		state.ce = chg_ctrl_2 & BQ25980_CHG_EN;
		val->intval = state.ce;
		break;
*/
/*
	case POWER_SUPPLY_PROP_TI_ADC:
		ret = bq25980_get_adc_enable(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_TI_BYPASS:
		ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
		if (ret)
			return ret;

		state.bypass = chg_ctrl_2 & BQ25980_EN_BYPASS;
		val->intval = state.bypass;
		break;
*/
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->state.online;
		break;
//	case POWER_SUPPLY_PROP_UPDATE_NOW:
//		break;
/*	case POWER_SUPPLY_PROP_CP_IRQ_STATUS:
		val->intval = bq->irq_counts;
		break;
*/
	default:
		return -EINVAL;
	}
//	dev_err(bq->dev,"%s,psp:%d,value:%d\n",__func__,psp,val->intval);
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
		old_state.ovp != new_state->ovp ||
		old_state.ocp != new_state->ocp ||
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

	dev_err(bq->dev,"[%s]%s enter\n",bq->model_name,__func__);
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

	if(bq->irq_counts > INT_MAX -1)
		bq->irq_counts = 0;
	else
		bq->irq_counts++;


	mutex_unlock(&bq->irq_complete);

	ret = bq25980_get_state(bq, &state);
	if (ret < 0)
		goto irq_out;

	if(bq->alarm_status.status > 0 ||
		bq->fault_status.status > 0)
		dump_all_reg(bq);
	if (!bq25980_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static enum power_supply_property bq25980_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	//POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
//	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
//	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CHARGING_ENABLED,
/*
	POWER_SUPPLY_PROP_TI_ADC,
	POWER_SUPPLY_PROP_TI_BYPASS,
*/

	//POWER_SUPPLY_PROP_CP_STATUS1,//
	POWER_SUPPLY_PROP_PRESENT,
//	POWER_SUPPLY_PROP_CHARGING_ENABLED,//undeclared identifier
//	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	//POWER_SUPPLY_PROP_CP_IRQ_STATUS,//undeclared identifier
	//POWER_SUPPLY_PROP_CHIP_VERSION,
};
#if 0
static enum power_supply_property bq25980_battery_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};
#endif
static char *bq25980_charger_supplied_to[] = {
	"main-battery",
};

static int bq25980_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
//	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
//	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
//	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_PRESENT:
//	case POWER_SUPPLY_PROP_UPDATE_NOW:
//	case POWER_SUPPLY_PROP_TI_ADC:
//	case POWER_SUPPLY_PROP_TI_BYPASS:
		return true;
	default:
		return false;
	}
}

#if 0
static struct power_supply_desc bq25980_battery_desc = {
	.name			= "bq25980-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= bq25980_get_battry_property,
	.set_property		= bq25980_set_battery_property,
	.properties		= bq25980_battery_props,
	.num_properties		= ARRAY_SIZE(bq25980_battery_props),
	.property_is_writeable	= bq25980_property_is_writeable,
};
#endif

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
	.reg_defaults = bq25980_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25980_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25975_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults = bq25975_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25975_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25960_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults = bq25960_reg_defs,
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

	psy_cfg.supplied_to = bq25980_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25980_charger_supplied_to);

	switch (driver_data) {
	case BQ25980_MASTER:
		bq->psy_desc.name = "bq25980-master";
		break;
	case BQ25980_SLAVE:
		bq->psy_desc.name = "bq25980-slave";
		break;
	case BQ25980_STANDALONE:
		bq->psy_desc.name = "bq25980-standalone";
		break;
	case BQ25960_MASTER:
		bq->psy_desc.name = "bq25960-master";
		break;
	case BQ25960_SLAVE:
		bq->psy_desc.name = "bq25960-slave";
		break;
	case BQ25960_STANDALONE:
		bq->psy_desc.name = "bq25960-standalone";
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
/*
	if (bq->mode != BQ_SLAVE) {
		bq->battery = devm_power_supply_register(bq->dev,
						      &bq25980_battery_desc,
						      &psy_cfg);
		if (IS_ERR(bq->battery)) {
			dev_err(bq->dev, "battery register power supply fail");
			return -EINVAL;
		}
	}
*/
	return 0;
}

static int bq25980_reg_init(struct bq25980_device *bq)
{
	int i, ret;

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
#if 0
static int bq25980_hw_init(struct bq25980_device *bq)
{
	struct power_supply_battery_info bat_info = { };
	int wd_reg_val;
	int ret = 0;
	int curr_val;
	int volt_val;
	int i;

	if (!bq->watchdog_timer) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_3,
					 BQ25980_WATCHDOG_DIS,
					 BQ25980_WATCHDOG_DIS);
	} else {
		for (i = 0; i < BQ25980_NUM_WD_VAL; i++) {
			if (bq->watchdog_timer > bq25980_watchdog_time[i] &&
			    bq->watchdog_timer < bq25980_watchdog_time[i + 1]) {
				wd_reg_val = i;
				break;
			}
		}

		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_3,
					BQ25980_WATCHDOG_MASK, wd_reg_val);
	}
	if (ret)
		return ret;

	ret = power_supply_get_battery_info(bq->charger, &bat_info);
	if (ret) {
		dev_warn(bq->dev, "battery info missing\n");
		return -EINVAL;
	}

	bq->init_data.ichg_max = bat_info.constant_charge_current_max_ua;
	bq->init_data.vreg_max = bat_info.constant_charge_voltage_max_uv;

	if (bq->state.bypass) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, BQ25980_EN_BYPASS);
		if (ret)
			return ret;

		curr_val = bq->init_data.bypass_ilim;
		volt_val = bq->init_data.bypass_vlim;
	} else {
		curr_val = bq->init_data.sc_ilim;
		volt_val = bq->init_data.sc_vlim;
	}

	ret = bq25980_set_input_curr_lim(bq, curr_val);
	if (ret)
		return ret;

	ret = bq25980_set_input_volt_lim(bq, volt_val);
	if (ret)
		return ret;

	return regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				 BQ25980_ADC_EN, BQ25980_ADC_EN);
}
#endif
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
static int bq25890_get_part_no(struct bq25980_device *bq)
{
	struct i2c_client client;
	int ret;
	int len;
	const char *sc8541_name;

	bq->part_no = 0;
	ret = device_property_read_u32(bq->dev, "sc8541-addr",
				       &bq->sc8541_addr);
	if (ret)
		return BQ25960_PART_NO;
	client = *(bq->client);
	ret = i2c_smbus_read_byte_data(&client, BQ25980_DEVICE_INFO);
	if(ret == BQ25960_PART_NO ) {
		return BQ25960_PART_NO; //read success
	}else {
		pr_info("bq25890_get_part_no: orig addr = %d, sc8541 addr =%d\n ",
			client.addr, bq->sc8541_addr);
		client.addr = bq->sc8541_addr;
		ret = i2c_smbus_read_byte_data(&client, BQ25980_DEVICE_INFO);
		if(ret == SC8541_PART_NO) {
			memset((void*)bq->model_name, 0x00, sizeof(bq->model_name));
			device_property_read_string(bq->dev, "sc8541-name", &sc8541_name);
			len = strlen(sc8541_name);
			strncpy(bq->model_name, sc8541_name, min(I2C_NAME_SIZE,len) );
			pr_err("[%s] model_name=%s\n", __func__ , bq->model_name);
		}
	}

	return ret;
}

static int bq25980_check_device_id(struct bq25980_device *bq)
{
#if 0 //IC device id not confirmed
	int ret;
	int val;

	ret = regmap_read(bq->regmap, BQ25980_DEVICE_INFO, &val);
	if (ret) {
		dev_err(bq->dev, "Failed to read device id\n");
		return ret;
	}

	val = (val & BQ25980_DEVICE_ID_MASK);
	if (bq->device_id != val) {
		dev_err(bq->dev, "dts id %d mismatch with hardware id %d\n", bq->device_id, val);
		return -EINVAL;
	}
#endif
	return 0;
}

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}
	return sprintf(buf, "reg addr 0x%02x\n", bq->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
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
	ssize_t size = 0;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}
	if( ( bq->reg_addr >= 0 ) && (bq->reg_addr <= 0x37) )
	{
		ret = regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);
		size = sprintf(buf, "reg[%02X]=0x%02X\n", bq->reg_addr, bq->reg_data);
	}else
	{
		size = sprintf(buf, "reg addr error\n");
	}
	return size;
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_data = tmp;

	if( (bq->reg_addr >= 0) && (bq->reg_addr <= 0x37) ) {
		if( (bq->reg_data >= 0) && (bq->reg_data <= 0xFF) ) {
			regmap_write(bq->regmap, bq->reg_addr, bq->reg_data);
		}else
			pr_err("reg data error : data=0x%X\n", bq->reg_data);
	}else
		pr_err("reg addr error : addr=0x%X\n", bq->reg_addr);

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
		pr_err("[%s] chip not valid\n", bq->model_name);
		state = -ENODEV;
	}else {
		ret = bq25980_is_chg_en(bq, &enable);
		if (ret < 0) {
			pr_err("[%s] bq25980_is_chg_en not valid\n", bq->model_name);
			state = -ENODEV;
		}else
			state = enable;
	}
	return sprintf(buf, "%d\n", state);
}

static ssize_t store_force_chg_auto_enable(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	int ret;
	bool enable;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = bq25980_set_chg_en(bq, enable);
	if (ret) {
		pr_err("[%s] Couldn't %s charging rc=%d\n", bq->model_name,
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("[%s] %s charging \n", bq->model_name,
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
		pr_err("[%s] chip not valid\n", bq->model_name);
		return -ENODEV;
	}

	for (addr = 0; addr <= 0x3a; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret) {
			pr_err("%s_dump_register Reg[%02X]=0x%02X\n", bq->model_name, addr, val);
			size += snprintf(buf + size, PAGE_SIZE - size,
				"reg[%02X]=[0x%02X]\n", addr,val);
		}else {
			size += snprintf(buf + size, PAGE_SIZE - size,
				"reg[%02X]=[failed]\n", addr);
		}
	}

	return size;
}
static DEVICE_ATTR(reg_dump, 0444, show_reg_dump, NULL);

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct bq25980_device *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("[%s] chip not valid\n", bq->model_name);
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

static int bq25980_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct bq25980_device *bq = iio_priv(indio_dev);
	int rc = 0;

	pr_err("[%s] iio_write_raw ch=%d val1=%d val2=%d\n",
			bq->model_name, chan->channel, val1, val2);
	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		bq25980_set_chg_en(bq, val1);
		pr_info("[%s] set_chg_en: %s\n", bq->model_name,
				val1 ? "enable" : "disable");
		break;
	case PSY_IIO_ONLINE:
		bq25980_set_present(bq, !!val1);
		pr_info("[%s] set_present :%d\n", bq->model_name, val1);
		break;
	case PSY_IIO_CP_CLEAR_ERROR:
		bq->fault_status.status = 0;
		bq->alarm_status.status = 0;
		break;
	case PSY_IIO_CP_STATUS1:
		if (val1 == MMI_DISABLE_ADC)
			bq25980_set_adc_enable(bq, false);
		else if (val1 == MMI_ENABLE_ADC)
			bq25980_set_adc_enable(bq, true);
		break;
	default:
		pr_err("Unsupported [%s] IIO chan %d\n",
				bq->model_name, chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("[%s] Couldn't write IIO channel %d, rc = %d\n",
			bq->model_name,
			chan->channel, rc);

	return rc;
}

static int bq25980_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct bq25980_device *bq = iio_priv(indio_dev);
	int rc = 0;
	int result;
	struct bq25980_state state;

	*val1 = 0;
	pr_err("[%s] iio_read_raw ch=%d \n",bq->model_name,chan->channel);
	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		rc = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &result);
		if (!rc) {
			bq->state.ce = result & BQ25980_CHG_EN;
			*val1 = bq->state.ce;
			pr_info("[%s] read charge enable:%d\n", bq->model_name, *val1);
		}else
			pr_err("[%s] read charge enable err\n", bq->model_name);
		break;
	case PSY_IIO_ONLINE:
		rc = regmap_read(bq->regmap, BQ25980_STAT3, &result);
		if (!rc) {
			bq->state.online = result & BQ25980_PRESENT_MASK;
			*val1 = bq->state.online;
			pr_info("[%s] read online:%d\n", bq->model_name, *val1);
		}else
			pr_err("[%s] read online err\n", bq->model_name);
		break;
	case PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW:
		rc = bq25980_get_adc_vbus(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_vbus err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_vbus %duV\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_MMI_CP_INPUT_CURRENT_NOW:
		rc = bq25980_get_adc_ibus(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_ibus err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_ibus %duA\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_CP_STATUS1:
		rc = bq25980_get_state(bq,&state);
		if (rc) {
			pr_err("[%s] get_state err\n", bq->model_name);
			return rc;
		}else {
			if (!bq25980_state_changed(bq, &state)){
				mutex_lock(&bq->lock);
				bq->state = state;
				mutex_unlock(&bq->lock);
				power_supply_changed(bq->charger);
			}
			*val1 = bq->alarm_status.status | (bq->fault_status.status << 8);
			pr_err("[%s] get_state fault:0x%02X , alarm:0x%02X\n",
					bq->model_name,
					bq->fault_status.status,
					bq->alarm_status.status);
		}
		break;
	case PSY_IIO_CURRENT_NOW:
		rc = bq25980_get_ibat_adc(bq);
		if ( rc < 0) {
			pr_err("[%s] get_ibat_adc err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_ibat_adc %duA\n", bq->model_name, *val1);
		}
		break;
	case PSY_IIO_VOLTAGE_NOW:
		rc = bq25980_get_adc_vbat(bq);
		if ( rc < 0 ) {
			pr_err("[%s] get_adc_vbat err %d\n", bq->model_name, rc);
		}else {
			*val1 = rc;
			pr_info("[%s] get_adc_vbat %duV\n", bq->model_name, *val1);
		}
		break;
	default:
		pr_err("[%s] Unsupported bq25980 IIO chan %d\n", bq->model_name, chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("[%s] Couldn't read IIO channel %d, rc = %d\n", bq->model_name,
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int bq25980_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct bq25980_device *bq = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = bq->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(bq25980_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info bq25980_iio_info = {
	.read_raw	= bq25980_iio_read_raw,
	.write_raw	= bq25980_iio_write_raw,
	.of_xlate	= bq25980_iio_of_xlate,
};

static int bq25980_init_iio_psy(struct bq25980_device *bq)
{
	struct iio_dev *indio_dev = bq->indio_dev;
	struct iio_chan_spec *chan;
	int bq25980_num_iio_channels = ARRAY_SIZE(bq25980_iio_psy_channels);
	int rc, i;

	bq->iio_chan = devm_kcalloc(bq->dev, bq25980_num_iio_channels,
				sizeof(*bq->iio_chan), GFP_KERNEL);
	if (!bq->iio_chan)
		return -ENOMEM;

	bq->int_iio_chans = devm_kcalloc(bq->dev,
				bq25980_num_iio_channels,
				sizeof(*bq->int_iio_chans),
				GFP_KERNEL);
	if (!bq->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &bq25980_iio_info;
	indio_dev->dev.parent = bq->dev;
	indio_dev->dev.of_node = bq->dev->of_node;
	indio_dev->name = bq->model_name;//"bq25980-charger";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = bq->iio_chan;
	indio_dev->num_channels = bq25980_num_iio_channels;

	for (i = 0; i < bq25980_num_iio_channels; i++) {
		bq->int_iio_chans[i].indio_dev = indio_dev;
		chan = &bq->iio_chan[i];
		bq->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = bq25980_iio_psy_channels[i].channel_num;
		chan->type = bq25980_iio_psy_channels[i].type;
		chan->datasheet_name =
			bq25980_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			bq25980_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			bq25980_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(bq->dev, indio_dev);
	if (rc)
		pr_err("Failed to register [%s] IIO device, rc=%d\n", indio_dev->name,rc);
	else
		pr_err("Success to register [%s] IIO device\n", indio_dev->name);

	return rc;
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

static int bq25980_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq25980_device *bq;
	int ret, irq_gpio, irqn;
	struct iio_dev *indio_dev;

	printk("-------[%s] driver probe--------\n",id->name);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bq));

	if (!indio_dev)
		return -ENOMEM;

	bq = iio_priv(indio_dev);
	if (!bq) {
		dev_err(dev, "Out of memory\n");
		return -ENOMEM;
	}

	bq->indio_dev = indio_dev;
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

	if ( bq->part_no == SC8541_PART_NO ) {
		bq->client->addr = bq->sc8541_addr;
		bq->chip_info = &sc8541_chip_info_tbl[bq->device_id];
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

	ret = bq25980_check_device_id(bq);
	if (ret)
		goto free_mem;

	ret = bq25980_check_work_mode(bq);
	if (ret)
		goto free_mem;

	ret = bq25980_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		goto free_mem;
	}

#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpio", 0);
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
#else

	bq->irq_pinctrl =
		pinctrl_get_select(bq->dev, "bq25960_int_default");
	if (!bq->irq_pinctrl) {
		dev_err(bq->dev,"Couldn't set pinctrl bq25960_int_default\n");
		goto free_mem;
	}
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
		//enable_irq_wake(client->irq);
	}

	ret = bq25980_power_supply_init(bq, dev, id->driver_data);
	if (ret)
		goto free_mem;

	ret = bq25980_reg_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto free_psy;
	}

	bq25980_init_iio_psy(bq);

	bq25980_create_device_node(bq->dev);

	//dump_reg(bq,0x00,0x37);

	printk("-------[%s] driver probe success--------\n",bq->model_name);
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
