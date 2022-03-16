/*
 * Fuelgauge battery driver
 *
 * Copyright (C) 2018 Siliconmitus
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#define pr_fmt(fmt)	"[sm5602] %s(%d): " fmt, __func__, __LINE__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include "sm5602_fg.h"

#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err
#undef dev_dbg
#define dev_dbg dev_err

#define	INVALID_REG_ADDR	0xFF
#define   RET_ERR -1
#define queue_delayed_work_time  8000//8000
#define queue_start_work_time    50
#define SM_CUR_UNIT              1000
#define ENABLE_MAP_SOC

#ifdef ENABLE_MAP_SOC
#define MAP_MAX_SOC		97
#define MAP_RATE_SOC	975
#define MAP_MIN_SOC		4
#endif

enum sm_fg_reg_idx {
	SM_FG_REG_DEVICE_ID = 0,
	SM_FG_REG_CNTL,
	SM_FG_REG_INT,
	SM_FG_REG_INT_MASK,
	SM_FG_REG_STATUS,
	SM_FG_REG_SOC,
	SM_FG_REG_OCV,
	SM_FG_REG_VOLTAGE,
	SM_FG_REG_CURRENT,
	SM_FG_REG_TEMPERATURE_IN,
	SM_FG_REG_TEMPERATURE_EX,
	SM_FG_REG_V_L_ALARM,
	SM_FG_REG_V_H_ALARM,
	SM_FG_REG_A_H_ALARM,
	SM_FG_REG_T_IN_H_ALARM,
	SM_FG_REG_SOC_L_ALARM,
	SM_FG_REG_FG_OP_STATUS,
	SM_FG_REG_TOPOFFSOC,
	SM_FG_REG_PARAM_CTRL,
	SM_FG_REG_SHUTDOWN,
	SM_FG_REG_VIT_PERIOD,
	SM_FG_REG_CURRENT_RATE,
	SM_FG_REG_BAT_CAP,
	SM_FG_REG_CURR_OFFSET,
	SM_FG_REG_CURR_SLOPE,
	SM_FG_REG_MISC,
	SM_FG_REG_RESET,
	SM_FG_REG_RSNS_SEL,
	SM_FG_REG_VOL_COMP,
	NUM_REGS,
};

static u8 sm5602_regs[NUM_REGS] = {
	0x00, /* DEVICE_ID */
	0x01, /* CNTL */
	0x02, /* INT */
	0x03, /* INT_MASK */
	0x04, /* STATUS */
	0x05, /* SOC */
	0x06, /* OCV */
	0x07, /* VOLTAGE */
	0x08, /* CURRENT */
	0x09, /* TEMPERATURE_IN */
	0x0A, /* TEMPERATURE_EX */
	0x0C, /* V_L_ALARM */
	0x0D, /* V_H_ALARM */
	0x0E, /* A_H_ALARM */
	0x0F, /* T_IN_H_ALARM */
	0x10, /* SOC_L_ALARM */
	0x11, /* FG_OP_STATUS */
	0x12, /* TOPOFFSOC */
	0x13, /* PARAM_CTRL */
	0x14, /* SHUTDOWN */
	0x1A, /* VIT_PERIOD */
	0x1B, /* CURRENT_RATE */
	0x62, /* BAT_CAP */
	0x73, /* CURR_OFFSET */
	0x74, /* CURR_SLOPE */
	0x90, /* MISC */
	0x91, /* RESET */
	0x95, /* RSNS_SEL */
	0x96, /* VOL_COMP */
};

enum sm_fg_device {
	SM5602,
};

enum sm_fg_temperature_type {
	TEMPERATURE_IN = 0,
	TEMPERATURE_EX,
};

const unsigned char *device2str[] = {
	"sm5602",
};

enum battery_table_type {
	BATTERY_TABLE0 = 0,
	BATTERY_TABLE1,
	BATTERY_TABLE2,
	BATTERY_TABLE_MAX,
};

//struct sm_fg_chip;

struct sm_fg_chip {
	struct device *dev;
	struct i2c_client *client;
	struct mutex i2c_rw_lock; /* I2C Read/Write Lock */
	struct mutex data_lock; /* Data Lock */
	u8 chip;
	u8 regs[NUM_REGS];
	int batt_id;
	//int gpio_int;
	/* Status Tracking */
	bool batt_present;
	bool batt_fc;	/* Battery Full Condition */
	bool batt_ot;	/* Battery Over Temperature */
	bool batt_ut;	/* Battery Under Temperature */
	bool batt_soc1;	/* SOC Low */
	bool batt_socp;	/* SOC Poor */
	bool batt_dsg;	/* Discharge Condition*/
	int  batt_soc;
	int batt_ocv;
	int batt_fcc;	/* Full charge capacity */
	int batt_fcc_design;	/* Design Full charge capacity */
	int batt_volt;
	int aver_batt_volt;
	int batt_temp;
	int batt_curr;
	int is_charging;	/* Charging informaion from charger IC */
	int batt_soc_cycle; /* Battery SOC cycle */
 	int topoff_soc;
	int top_off;
	int iocv_error_count;

	/* previous battery voltage current*/
	int p_batt_voltage;
	int p_batt_current;

	/* DT */
	bool en_temp_ex;
	bool en_temp_in;
	bool en_batt_det;
	bool iocv_man_mode;
	int aging_ctrl;
	int batt_rsns;	/* Sensing resistor value */
	int ntc_exist;
	bool factory_mode;
	int cycle_cfg;
	int fg_irq_set;
	int low_soc1;
	int low_soc2;
	int v_l_alarm;
	int v_h_alarm;
	int battery_table_num;
	int misc;
	int batt_v_max;
	int min_cap;
	u32 common_param_version;
	int t_l_alarm_in;
	int t_h_alarm_in;
	u32 t_l_alarm_ex;
	u32 t_h_alarm_ex;

	/* Battery Data */
	int battery_table[BATTERY_TABLE_MAX][FG_TABLE_LEN];
	signed short battery_temp_table[FG_TEMP_TABLE_CNT_MAX]; /* -20~80 Degree */
	int alpha;
	int beta;
	int rs;
	int rs_value[4];
	int vit_period;
	int mix_value;
	const char *battery_type;
	int volt_cal;
	int curr_offset;
	int curr_slope;
	int cap;
	int n_tem_poff;
	int n_tem_poff_offset;
	int batt_max_voltage_uv;
	int temp_std;
	int en_high_fg_temp_offset;
	int high_fg_temp_offset_denom;
	int high_fg_temp_offset_fact;
	int en_low_fg_temp_offset;
	int low_fg_temp_offset_denom;
	int low_fg_temp_offset_fact;
	int en_high_fg_temp_cal;
	int high_fg_temp_p_cal_denom;
	int high_fg_temp_p_cal_fact;
	int high_fg_temp_n_cal_denom;
	int high_fg_temp_n_cal_fact;
	int en_low_fg_temp_cal;
	int low_fg_temp_p_cal_denom;
	int low_fg_temp_p_cal_fact;
	int low_fg_temp_n_cal_denom;
	int low_fg_temp_n_cal_fact;
	int en_high_temp_cal;
	int high_temp_p_cal_denom;
	int high_temp_p_cal_fact;
	int high_temp_n_cal_denom;
	int high_temp_n_cal_fact;
	int en_low_temp_cal;
	int low_temp_p_cal_denom;
	int low_temp_p_cal_fact;
	int low_temp_n_cal_denom;
	int low_temp_n_cal_fact;
	u32 battery_param_version;

	struct delayed_work monitor_work;
	struct workqueue_struct *smfg_workqueue;
//	unsigned long last_update;

	/* Debug */
	int skip_reads;
	int skip_writes;
	int fake_soc;
	int fake_temp;
	struct dentry *debug_root;
	struct power_supply *batt_psy;
	struct power_supply *bms_psy;
	//struct power_supply fg_psy;
};

static int show_registers(struct seq_file *m, void *data);
static bool fg_init(struct i2c_client *client);

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		pr_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int fg_read_word(struct sm_fg_chip *sm, u8 reg, u16 *val)
{
	int ret;

	if (sm->skip_reads) {
		*val = 0;
		return 0;
	}
	/* TODO:check little endian */
	mutex_lock(&sm->i2c_rw_lock);
	ret = __fg_read_word(sm->client, reg, val);
	mutex_unlock(&sm->i2c_rw_lock);

	return ret;
}

static int fg_write_word(struct sm_fg_chip *sm, u8 reg, u16 val)
{
	int ret;

	if (sm->skip_writes)
		return 0;

	/* TODO:check little endian */
	mutex_lock(&sm->i2c_rw_lock);
	ret = __fg_write_word(sm->client, reg, val);
	mutex_unlock(&sm->i2c_rw_lock);

	return ret;
}

#define	FG_STATUS_SLEEP				BIT(10)
#define	FG_STATUS_BATT_PRESENT		BIT(9)
#define	FG_STATUS_SOC_UPDATE		BIT(8)
#define	FG_STATUS_TOPOFF			BIT(7)
#define	FG_STATUS_LOW_SOC2			BIT(6)
#define	FG_STATUS_LOW_SOC1			BIT(5)
#define	FG_STATUS_HIGH_CURRENT		BIT(4)
#define	FG_STATUS_HIGH_TEMPERATURE	BIT(3)
#define	FG_STATUS_LOW_TEMPERATURE	BIT(2)
#define	FG_STATUS_HIGH_VOLTAGE		BIT(1)
#define	FG_STATUS_LOW_VOLTAGE		BIT(0)

#define	FG_OP_STATUS_CHG_DISCHG		BIT(15) //if can use the charger information, plz use the charger information for CHG/DISCHG condition.

static int fg_read_status(struct sm_fg_chip *sm)
{
	int ret;
	u16 flags1, flags2;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_STATUS], &flags1);
	if (ret < 0)
		return ret;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &flags2);
		if (ret < 0)
			return ret;

	mutex_lock(&sm->data_lock);
	if(sm->factory_mode && !sm->ntc_exist)
		sm->batt_present = true;
	else
		sm->batt_present	= !!(flags1 & FG_STATUS_BATT_PRESENT);
	sm->batt_ot			= !!(flags1 & FG_STATUS_HIGH_TEMPERATURE);
	sm->batt_ut			= !!(flags1 & FG_STATUS_LOW_TEMPERATURE);
	sm->batt_fc			= !!(flags1 & FG_STATUS_TOPOFF);
	sm->batt_soc1		= !!(flags1 & FG_STATUS_LOW_SOC2);
	sm->batt_socp		= !!(flags1 & FG_STATUS_LOW_SOC1);
	sm->batt_dsg		= !!!(flags2 & FG_OP_STATUS_CHG_DISCHG);
	mutex_unlock(&sm->data_lock);

	return 0;
}
#if 0
static int fg_status_changed(struct sm_fg_chip *sm)
{
	//cancel_delayed_work(&sm->monitor_work);
	//schedule_delayed_work(&sm->monitor_work, 0);
	//power_supply_changed(sm->batt_psy);
	printk("wangtao fg_status_changed\n");
	return IRQ_HANDLED;
}

static irqreturn_t fg_irq_thread(int irq, void *dev_id)
{
	struct sm_fg_chip *sm = dev_id;
	int ret;
	u16 data_int, data_int_mask;

	/* Read INT */
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_INT_MASK], &data_int_mask);
	if (ret < 0){
		pr_err("Failed to read INT_MASK, ret = %d\n", ret);
		return ret;
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK], 0x8000 | data_int_mask);
    if (ret < 0) {
		pr_err("Failed to write 0x8000 | INIT_MARK, ret = %d\n", ret);
		return ret;
	}

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_INT], &data_int);
	if (ret < 0) {
		pr_err("Failed to write REG_INT, ret = %d\n", ret);
		return ret;
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK], 0x03FF & data_int_mask);
    if (ret < 0) {
		pr_err("Failed to write INIT_MARK, ret = %d\n", ret);
		return ret;
	}

	fg_status_changed(sm);

	pr_info("fg_read_int = 0x%x\n", data_int);

	return 0;
}
#endif

static int fg_read_soc(struct sm_fg_chip *sm)
{
	int ret;
	unsigned int soc = 0;
	u16 data = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC], &data);
	if (ret < 0) {
		pr_err("could not read SOC, ret = %d\n", ret);
		return ret;
	} else {
		/*integer bit;*/
		soc = ((data&0x7f00)>>8) * 10;
		/* integer + fractional bit*/
		soc = soc + (((data&0x00ff)*10)/256);

		if (data & 0x8000) {
			soc *= -1;
		}
	}

	 pr_info("fg_read_soc soc=%d\n",soc);
	return soc;
}

static unsigned int fg_read_ocv(struct sm_fg_chip *sm)
{
	int ret;
	u16 data = 0;
	unsigned int ocv;// = 3500; /*3500 means 3500mV*/

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_OCV], &data);
	if (ret<0) {
		pr_err("could not read OCV, ret = %d\n", ret);
		ocv = 4000;
	} else {
		ocv = (((data&0x0fff)*1000)/2048) + (((data&0xf000)>>11)*1000);
	}

	return ocv; //mV
}

bool is_factory_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory_mode = false;
	const char *bootargs = NULL;
	char *bootmode = NULL;
	char *end = NULL;

	if (!np)
		return factory_mode;

	if (!of_property_read_string(np, "bootargs", &bootargs)) {
		bootmode = strstr(bootargs, "androidboot.mode=");
		if (bootmode) {
			end = strpbrk(bootmode, " ");
			bootmode = strpbrk(bootmode, "=");
		}
		if (bootmode &&
		    end > bootmode &&
		    strnstr(bootmode, "mot-factory", end - bootmode)) {
				factory_mode = true;
		}
	}
	of_node_put(np);

	return factory_mode;
}


static int _calculate_battery_temp_ex(struct sm_fg_chip *sm, u16 uval)
{
	int i = 0, temp = 0;
	signed short val = 0;

	if ((uval >= 0x8001) && (uval <= 0x823B)) {
		pr_info("sp_range uval = 0x%x\n",uval);
		uval = 0x0000;
	}

	val = uval;

	if (val >= sm->battery_temp_table[0]) {
		temp = -20; //Min : -20
	} else if (val <= sm->battery_temp_table[FG_TEMP_TABLE_CNT_MAX-1]) {
		temp = 80; //Max : 80
	} else {
		for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
			if  (val >= sm->battery_temp_table[i]) {
				temp = -20 + i; 									  //[ex] ~-20 : -20(skip), -19.9~-19.0 : 19, -18.9~-18 : 18, .., 0.9~0 : 0 				
				if ((temp >= 1) && (val != sm->battery_temp_table[i]))//+ range 0~79 degree. In same value case, no needed (temp-1)
					temp = temp -1; 								  //[ex] 0.1~0.9 : 0, 1.1~1.9 : 1, .., 79.1~79.9 : 79  
				break;
			}
		}
	}

	pr_info("uval = 0x%x, val = 0x%x, temp = %d\n",uval, val, temp);

	if(sm->factory_mode && !sm->ntc_exist && ((-20 == temp) || (80 == temp)))
		temp = 25;
	return temp;
}

static int fg_read_temperature(struct sm_fg_chip *sm, enum sm_fg_temperature_type temperature_type)
{
	int ret, temp = 0;
	u16 data = 0;

	switch (temperature_type) {
	case TEMPERATURE_IN:
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_IN], &data);
		if (ret < 0) {
			pr_err("could not read temperature in , ret = %d\n", ret);
			return ret;
		} else {
			/*integer bit*/
			temp = ((data & 0x00FF));
			if (data & 0x8000)
				temp *= -1;
		}
		pr_info("fg_read_temperature_in temp_in=%d\n", temp);
		break;
	case TEMPERATURE_EX:
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_EX], &data);
		if (ret < 0) {
			pr_err("could not read temperature ex , ret = %d\n", ret);
			return ret;
		} else {
			temp = _calculate_battery_temp_ex(sm, data);
		}
		temp = temp * 10;
		pr_info("fg_read_temperature_ex temp_ex=%d\n", temp);
		break;

	default:
		return -EINVAL;
	}

	return temp;
}

/*
 *	Return : mV
 */
static int fg_read_volt(struct sm_fg_chip *sm)
{
	int ret = 0;
	int volt = 0;
	u16 data = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_VOLTAGE], &data);
	if (ret < 0) {
		pr_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}  else {
		/* */
		volt = 1800 * (data & 0x7FFF) / 19622;
		if (data&0x8000)
			volt *= -1;
		volt += 2700;
	}

	/*cal avgvoltage*/
	sm->aver_batt_volt = (((sm->aver_batt_volt)*4) + volt)/5;
	volt = volt * 1000;
	return volt;
}

static int fg_get_cycle(struct sm_fg_chip *sm)
{
	int ret;
	int cycle;
	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_SOC_CYCLE, &data);
	if (ret<0) {
		pr_err("read cycle reg fail ret = %d\n", ret);
		cycle = 0;
	} else {
		cycle = data&0x01FF;
	}

	return cycle;
}

static int fg_read_current(struct sm_fg_chip *sm)
{
	int ret = 0, rsns = 0;
	u16 data = 0;
	int curr = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CURRENT], &data);
	if (ret < 0) {
		pr_err("could not read current, ret = %d\n", ret);
		return ret;
	} else {
		if (sm->batt_rsns == -EINVAL) {
			pr_err("could not read sm->batt_rsns, rsns = 10mohm\n");
			rsns = 10;
		} else {
			sm->batt_rsns == 0 ? rsns = 5 : (rsns = sm->batt_rsns*10);
		}

		curr = ((data & 0x7FFF) * 1250 / 511 / rsns );

		if (data & 0x8000)
			curr *= -1;
	}

	curr *= SM_CUR_UNIT * (-1);
	pr_err("curr = %d,data=%d\n",curr,data);

	return curr;
}

static int fg_read_fcc(struct sm_fg_chip *sm)
{
	int ret = 0;
	int fcc = 0;
	u16 data = 0;
	int64_t temp = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_BAT_CAP], &data);
	if (ret < 0) {
		pr_err("could not read FCC, ret=%d\n", ret);
		return ret;
	} else {
		/* */
		temp = div_s64((data & 0x7FFF) * 1000, 2048);
		fcc = temp;
	}

	return fcc;
}

#define FG_SOFT_RESET	0xA6
static int fg_reset(struct sm_fg_chip *sm)
{
    int ret;

    ret = fg_write_word(sm, sm->regs[SM_FG_REG_RESET], FG_SOFT_RESET);
	if (ret < 0) {
		pr_err("could not reset, ret=%d\n", ret);
		return ret;
	}

	msleep(600);

    return 0;
}

static int get_battery_status(struct sm_fg_chip *sm)
{
	union power_supply_propval ret = {0,};
	int rc;

	if (sm->batt_psy == NULL)
		sm->batt_psy = power_supply_get_by_name("battery");
	if (sm->batt_psy) {
		/* if battery has been registered, use the status property */
		rc = power_supply_get_property(sm->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Battery does not export status: %d\n", rc);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool is_battery_charging(struct sm_fg_chip *sm)
{
	return get_battery_status(sm) == POWER_SUPPLY_STATUS_CHARGING;
}

static void fg_vbatocv_check(struct sm_fg_chip *sm)
{
	pr_info("%s: sm->batt_curr (%d), sm->is_charging (%d), sm->top_off (%d), sm->batt_soc (%d)\n",
		__func__, sm->batt_curr, sm->is_charging, sm->top_off, sm->batt_soc);

	if((abs(sm->batt_curr)<50) ||
	   ((sm->is_charging) && (sm->batt_curr<(sm->top_off)) &&
	   (sm->batt_curr>(sm->top_off/3)) && (sm->batt_soc>=900)))
	{
		if(abs(sm->batt_ocv-sm->batt_volt)>30)
		{
			sm->iocv_error_count ++;
		}

		pr_info("%s: sm5602 FG iocv_error_count (%d)\n", __func__, sm->iocv_error_count);

		if(sm->iocv_error_count > 5)
			sm->iocv_error_count = 6;
	}
	else
	{
		sm->iocv_error_count = 0;
	}

	if(sm->iocv_error_count > 5)
	{
		pr_info("%s: p_v - v = (%d)\n", __func__, sm->p_batt_voltage - sm->batt_volt);
		if(abs(sm->p_batt_voltage - sm->batt_volt)>15)
		{
			sm->iocv_error_count = 0;
		}
		else
		{
			pr_info("%s: mode change to RS m mode\n", __func__);
			fg_write_word(sm, FG_REG_RS_2, sm->rs_value[0]);
		}
	}
	else
	{
		if((sm->p_batt_voltage < sm->n_tem_poff) &&
			(sm->batt_volt < sm->n_tem_poff) && (!sm->is_charging))
		{
			pr_info("%s: mode change to normal tem RS m mode\n", __func__);
			if((sm->p_batt_voltage <
				(sm->n_tem_poff - sm->n_tem_poff_offset)) &&
				(sm->batt_volt <
				(sm->n_tem_poff - sm->n_tem_poff_offset)))
			{
				fg_write_word(sm, FG_REG_RS_2, sm->rs_value[0]>>1);
			}
			else
			{
				fg_write_word(sm, FG_REG_RS_2, sm->rs_value[0]);
			}
		}
		else
		{
			pr_info("%s: mode change to RS a mode\n", __func__);

			fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
		}
	}
	sm->p_batt_voltage = sm->batt_volt;
	sm->p_batt_current = sm->batt_curr;
	// iocv error case cover end
}


static int fg_cal_carc (struct sm_fg_chip *sm)
{
	int curr_cal = 0, p_curr_cal=0, n_curr_cal=0, p_delta_cal=0, n_delta_cal=0, p_fg_delta_cal=0, n_fg_delta_cal=0, temp_curr_offset=0;
	int temp_gap, fg_temp_gap = 0;
	int ret = 0;
	u16 data[3] = {0,};

	fg_vbatocv_check(sm);

	sm->is_charging = is_battery_charging(sm); //From Charger Driver

	//fg_temp_gap = (sm->batt_temp/10) - sm->temp_std;
	fg_temp_gap = sm->batt_temp - sm->temp_std;

	temp_curr_offset = sm->curr_offset;
	if(sm->en_high_fg_temp_offset && (fg_temp_gap > 0))
	{
		if(temp_curr_offset & 0x0080)
		{
			temp_curr_offset = -(temp_curr_offset & 0x007F);
		}
		temp_curr_offset = temp_curr_offset + (fg_temp_gap / sm->high_fg_temp_offset_denom)*sm->high_fg_temp_offset_fact;
		if(temp_curr_offset < 0)
		{
			temp_curr_offset = -temp_curr_offset;
			temp_curr_offset = temp_curr_offset|0x0080;
		}
	}
	else if (sm->en_low_fg_temp_offset && (fg_temp_gap < 0))
	{
		if(temp_curr_offset & 0x0080)
		{
			temp_curr_offset = -(temp_curr_offset & 0x007F);
		}
		temp_curr_offset = temp_curr_offset + ((-fg_temp_gap) / sm->low_fg_temp_offset_denom)*sm->low_fg_temp_offset_fact;
		if(temp_curr_offset < 0)
		{
			temp_curr_offset = -temp_curr_offset;
			temp_curr_offset = temp_curr_offset|0x0080;
		}
	}
    temp_curr_offset = temp_curr_offset | (temp_curr_offset<<8);
	ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, temp_curr_offset);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("CURR_IN_OFFSET [0x%x] = 0x%x\n", FG_REG_CURR_IN_OFFSET, temp_curr_offset);
	}

	n_curr_cal = (sm->curr_slope & 0xFF00)>>8;
	p_curr_cal = (sm->curr_slope & 0x00FF);

	if (sm->en_high_fg_temp_cal && (fg_temp_gap > 0))
	{
		p_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_p_cal_denom)*sm->high_fg_temp_p_cal_fact;
		n_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_n_cal_denom)*sm->high_fg_temp_n_cal_fact;
	}
	else if (sm->en_low_fg_temp_cal && (fg_temp_gap < 0))
	{
		fg_temp_gap = -fg_temp_gap;
		p_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_p_cal_denom)*sm->low_fg_temp_p_cal_fact;
		n_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_n_cal_denom)*sm->low_fg_temp_n_cal_fact;
	}
	p_curr_cal = p_curr_cal + (p_fg_delta_cal);
	n_curr_cal = n_curr_cal + (n_fg_delta_cal);

	pr_info("%s: <%d %d %d %d %d %d %d %d %d %d>, temp_fg = %d ,p_curr_cal = 0x%x, n_curr_cal = 0x%x, "
		"batt_temp = %d\n",
		__func__,
		sm->en_high_fg_temp_cal,
		sm->high_fg_temp_p_cal_denom, sm->high_fg_temp_p_cal_fact,
		sm->high_fg_temp_n_cal_denom, sm->high_fg_temp_n_cal_fact,
		sm->en_low_fg_temp_cal,
		sm->low_fg_temp_p_cal_denom, sm->low_fg_temp_p_cal_fact,
		sm->low_fg_temp_n_cal_denom, sm->low_fg_temp_n_cal_fact,
		sm->batt_temp, p_curr_cal, n_curr_cal, sm->batt_temp);

	//temp_gap = (sm->batt_temp/10) - sm->temp_std;
	temp_gap = sm->batt_temp - sm->temp_std;
	if (sm->en_high_temp_cal && (temp_gap > 0))
	{
		p_delta_cal = (temp_gap / sm->high_temp_p_cal_denom)*sm->high_temp_p_cal_fact;
		n_delta_cal = (temp_gap / sm->high_temp_n_cal_denom)*sm->high_temp_n_cal_fact;
	}
	else if (sm->en_low_temp_cal && (temp_gap < 0))
	{
		temp_gap = -temp_gap;
		p_delta_cal = (temp_gap / sm->low_temp_p_cal_denom)*sm->low_temp_p_cal_fact;
		n_delta_cal = (temp_gap / sm->low_temp_n_cal_denom)*sm->low_temp_n_cal_fact;
	}
	p_curr_cal = p_curr_cal + (p_delta_cal);
	n_curr_cal = n_curr_cal + (n_delta_cal);

    curr_cal = (n_curr_cal << 8) | p_curr_cal;

	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, curr_cal);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("write CURR_IN_SLOPE [0x%x] = 0x%x\n", FG_REG_CURR_IN_SLOPE, curr_cal);
	}

	pr_info("%s: <%d %d %d %d %d %d %d %d %d %d>, "
		"p_curr_cal = 0x%x, n_curr_cal = 0x%x, curr_cal = 0x%x\n",
		__func__,
		sm->en_high_temp_cal,
		sm->high_temp_p_cal_denom, sm->high_temp_p_cal_fact,
		sm->high_temp_n_cal_denom, sm->high_temp_n_cal_fact,
		sm->en_low_temp_cal,
		sm->low_temp_p_cal_denom, sm->low_temp_p_cal_fact,
		sm->low_temp_n_cal_denom, sm->low_temp_n_cal_fact,
		p_curr_cal, n_curr_cal, curr_cal);

	ret = fg_read_word(sm, 0x28, &data[0]);
	ret |= fg_read_word(sm, 0x82, &data[1]);
	ret |= fg_read_word(sm, 0x83, &data[2]);
	if (ret < 0) {
		pr_err("could not read , ret = %d\n", ret);
		return ret;
	} else
		pr_info("0x28=0x%x, 0x82=0x%x, 0x83=0x%x\n", data[0],data[1],data[2]);

	return 1;
}

#if 0
static int fg_get_batt_status(struct sm_fg_chip *sm)
{
	if (!sm->batt_present)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	//else if (sm->batt_fc)
	//	return POWER_SUPPLY_STATUS_FULL;
	else if (sm->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (sm->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

}
#endif

static int fg_get_batt_capacity_level(struct sm_fg_chip *sm)
{
	if (!sm->batt_present)
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	else if (sm->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (sm->batt_soc1)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (sm->batt_socp)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

}

static int fg_get_charge_counter(struct sm_fg_chip *sm)
{
	int charge_counter;
	int full_capacity;
	int ui_soc;

	full_capacity = fg_read_fcc(sm) * 1000;
#ifdef ENABLE_MAP_SOC
	ui_soc = (((100*(sm->batt_soc*10+MAP_MAX_SOC))/MAP_RATE_SOC)-MAP_MIN_SOC)/10;
#else
	ui_soc = sm->batt_soc/10;
#endif
	charge_counter = div_s64(full_capacity * ui_soc, 100);

	return charge_counter;
}

static int fg_get_batt_health(struct sm_fg_chip *sm)
{
	if (!sm->batt_present)
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (sm->batt_ot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (sm->batt_ut)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static void fg_monitor_workfunc(struct work_struct *work);

static int fg_get_property(struct power_supply *psy, enum power_supply_property psp,
					union power_supply_propval *val)
{

	struct sm_fg_chip *sm = power_supply_get_drvdata(psy);
	int ret;
	/* pr_debug("fg_get_property\n"); */
	switch (psp) {
#if 0
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(sm);
/*		pr_info("fg POWER_SUPPLY_PROP_STATUS:%d\n", val->intval); */
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg_read_volt(sm);
		mutex_lock(&sm->data_lock);
		if (ret >= 0)
			sm->batt_volt = ret;
		val->intval = sm->batt_volt;
		mutex_unlock(&sm->data_lock);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sm->batt_present;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&sm->data_lock);
		sm->batt_curr = fg_read_current(sm);
		val->intval = sm->batt_curr;
		mutex_unlock(&sm->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (sm->fake_soc >= 0) {
			val->intval = sm->fake_soc;
			break;
		}
		ret = fg_read_soc(sm);
		mutex_lock(&sm->data_lock);
		if (ret >= 0)
			sm->batt_soc = ret;
		//val->intval = sm->batt_soc;
#ifdef ENABLE_MAP_SOC
		val->intval = (((100*(sm->batt_soc*10+MAP_MAX_SOC))/MAP_RATE_SOC)-MAP_MIN_SOC)/10;
#else
		val->intval = sm->batt_soc/10;
#endif
		pr_info("fg POWER_SUPPLY_PROP_STATUS:%d\n", val->intval);
		mutex_unlock(&sm->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(sm);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if (sm->fake_temp != -EINVAL) {
			val->intval = sm->fake_temp;
			break;
		}
		if (sm->en_temp_in)
			ret = fg_read_temperature(sm, TEMPERATURE_IN);
		else if (sm->en_temp_ex)
			ret = fg_read_temperature(sm, TEMPERATURE_EX);
		else
			ret = -ENODATA;
		mutex_lock(&sm->data_lock);
		if (ret > 0)
			sm->batt_temp = ret;
		val->intval = sm->batt_temp;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(sm);
		mutex_lock(&sm->data_lock);
		if (ret > 0)
			sm->batt_fcc = ret;
		val->intval = sm->batt_fcc * 1000;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = fg_get_batt_health(sm);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = fg_get_cycle(sm);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = sm->batt_fcc_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = fg_get_charge_counter(sm);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int fg_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int ret = 0;
	struct sm_fg_chip *sm;
	sm = power_supply_get_drvdata(psy);

	switch(prop) {
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
static int fg_prop_is_writeable(struct power_supply *psy,enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}


static void fg_external_power_changed(struct power_supply *psy)
{
	struct sm_fg_chip *sm = container_of(psy, struct sm_fg_chip, fg_psy);

	cancel_delayed_work(&sm->monitor_work);
	schedule_delayed_work(&sm->monitor_work, 0);
}
*/

static const u8 fg_dump_regs[] = {
	0x00, 0x01, 0x03, 0x04,
	0x05, 0x06, 0x07, 0x08,
	0x09, 0x0A, 0x0C, 0x0D,
	0x0E, 0x0F, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x1A,
	0x1B, 0x1C, 0x62, 0x73,
	0x74, 0x90, 0x91, 0x95,
	0x96
};

static int fg_dump_debug(struct sm_fg_chip *sm)
{
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		ret = fg_read_word(sm, fg_dump_regs[i], &val);
		if (!ret)
			pr_info("Reg[0x%02X] = 0x%02X\n",
						fg_dump_regs[i], val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sm_fg_chip *sm = inode->i_private;

	return single_open(file, show_registers, sm);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct sm_fg_chip *sm)
{
	sm->debug_root = debugfs_create_dir("sm_fg", NULL);
	if (!sm->debug_root)
		pr_err("Failed to create debug dir\n");

	if (sm->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						sm->debug_root, sm, &reg_debugfs_ops);

		debugfs_create_x32("fake_soc",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->fake_soc));

		debugfs_create_x32("fake_temp",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->fake_temp));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->skip_writes));
	}
}

static int show_registers(struct seq_file *m, void *data)
{
	struct sm_fg_chip *sm = m->private;
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		ret = fg_read_word(sm, fg_dump_regs[i], &val);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n",
						fg_dump_regs[i], val);
	}
	return 0;
}
#if 0
static void fg_refresh_status(struct sm_fg_chip *sm)
{
	bool last_batt_inserted;
	bool last_batt_fc;
	bool last_batt_ot;
	bool last_batt_ut;

	last_batt_inserted	= sm->batt_present;
	last_batt_fc		= sm->batt_fc;
	last_batt_ot		= sm->batt_ot;
	last_batt_ut		= sm->batt_ut;

	fg_read_status(sm);

	pr_info("batt_present=%d", sm->batt_present);

	if (!last_batt_inserted && sm->batt_present) {/* battery inserted */
		pr_info("Battery inserted\n");
	} else if (last_batt_inserted && !sm->batt_present) {/* battery removed */
		pr_info("Battery removed\n");
		sm->batt_soc	= -ENODATA;
		sm->batt_fcc	= -ENODATA;
		sm->batt_volt	= -ENODATA;
		sm->batt_curr	= -ENODATA;
		sm->batt_temp	= -ENODATA;
	}

	if ((last_batt_inserted != sm->batt_present)
		|| (last_batt_fc != sm->batt_fc)
		|| (last_batt_ot != sm->batt_ot)
		|| (last_batt_ut != sm->batt_ut))
		power_supply_changed(sm->batt_psy);

	if (sm->batt_present) {
		sm->batt_soc = fg_read_soc(sm);
		sm->batt_ocv = fg_read_ocv(sm);
		sm->batt_volt = fg_read_volt(sm);
		sm->batt_curr = fg_read_current(sm);
		sm->batt_soc_cycle = fg_get_cycle(sm);
		if (sm->en_temp_in)
			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_IN);
		else if (sm->en_temp_ex)
			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_EX);
		else
			sm->batt_temp = -ENODATA;
		fg_cal_carc(sm);

		pr_info("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
			sm->batt_soc, sm->batt_volt, sm->batt_curr, sm->batt_temp);
	}

//	sm->last_update = jiffies;
}
#endif

static int sm_update_data(struct sm_fg_chip *sm)
{
	int ret = 0;
	bool last_batt_inserted;
	bool last_batt_fc;
	bool last_batt_ot;
	bool last_batt_ut;

	last_batt_inserted	= sm->batt_present;
	last_batt_fc		= sm->batt_fc;
	last_batt_ot		= sm->batt_ot;
	last_batt_ut		= sm->batt_ut;

	fg_read_status(sm);

	pr_info("batt_present=%d", sm->batt_present);

	if (!last_batt_inserted && sm->batt_present) {/* battery inserted */
		pr_info("Battery inserted\n");
	} else if (last_batt_inserted && !sm->batt_present) {/* battery removed */
		pr_info("Battery removed\n");
		sm->batt_soc	= -ENODATA;
		sm->batt_fcc	= -ENODATA;
		sm->batt_volt	= -ENODATA;
		sm->batt_curr	= -ENODATA;
		sm->batt_temp	= -ENODATA;
	}

	if ((last_batt_inserted != sm->batt_present)
		|| (last_batt_fc != sm->batt_fc)
		|| (last_batt_ot != sm->batt_ot)
		|| (last_batt_ut != sm->batt_ut)) {
		if (sm->batt_psy)
			power_supply_changed(sm->batt_psy);
	}

	if (sm->batt_present) {
		sm->batt_soc = fg_read_soc(sm);
		sm->batt_ocv = fg_read_ocv(sm);
		sm->batt_volt = fg_read_volt(sm);
		sm->batt_curr = fg_read_current(sm);
		sm->batt_soc_cycle = fg_get_cycle(sm);
		if (sm->en_temp_in)

			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_IN);

		else if (sm->en_temp_ex)

			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_EX);

		else
			sm->batt_temp = -ENODATA;

		fg_cal_carc(sm);

          	pr_info("RSOC:%d, Volt:%d, Current:%d, Temperature:%d, OCV:%d\n",
			sm->batt_soc, sm->batt_volt, sm->batt_curr, sm->batt_temp, sm->batt_ocv);
	}

	return ret;
}

//static unsigned int poll_interval = 60;
static void fg_monitor_workfunc(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct sm_fg_chip *sm_bat;
	int ret;

	delay_work = container_of(work, struct delayed_work, work);
	sm_bat = container_of(delay_work, struct sm_fg_chip, monitor_work);

	/* get battery power supply */
	if (!sm_bat->batt_psy) {
		sm_bat->batt_psy = power_supply_get_by_name("battery");
		if (!sm_bat->batt_psy)
			printk("%s: get batt_psy fail\n", __func__);
	}

	ret = sm_update_data(sm_bat);
	if (ret < 0)
		printk(KERN_ERR "iic read error when update data");

	if (sm_bat->batt_psy) {
		power_supply_changed(sm_bat->batt_psy);
	}

	queue_delayed_work(sm_bat->smfg_workqueue, &sm_bat->monitor_work, msecs_to_jiffies(queue_delayed_work_time));

}

#define COMMON_PARAM_MASK		0xFF00
#define COMMON_PARAM_SHIFT		8
#define BATTERY_PARAM_MASK		0x00FF
static bool fg_check_reg_init_need(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int ret = 0;
	u16 data = 0;
	u16 param_ver = 0;
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
	if (ret < 0) {
			pr_err("Failed to read param_ctrl unlock, ret = %d\n", ret);
			return ret;
	} else {
		pr_info("FG_OP_STATUS = 0x%x\n", data);

		ret = fg_read_word(sm, FG_PARAM_VERION, &param_ver);
		if (ret < 0) {
				pr_err("Failed to read FG_PARAM_VERION, ret = %d\n", ret);
				return ret;
		}

		pr_info("param_ver = 0x%x, common_param_version = 0x%x, battery_param_version = 0x%x\n", param_ver, sm->common_param_version, sm->battery_param_version);

		if(((data & INIT_CHECK_MASK) == DISABLE_RE_INIT)
			&& (((param_ver & COMMON_PARAM_MASK) >> COMMON_PARAM_SHIFT) >= sm->common_param_version)
			&& ((param_ver & BATTERY_PARAM_MASK) >= sm->battery_param_version))
		{
			pr_info("%s: SM_FG_REG_FG_OP_STATUS : 0x%x , return FALSE NO init need\n", __func__, data);
			return 0;
		}
		else
		{
			pr_info("%s: SM_FG_REG_FG_OP_STATUS : 0x%x , return TRUE init need!!!!\n", __func__, data);
			return 1;
		}
	}
}

#define MINVAL(a, b) ((a <= b) ? a : b)
#define MAXVAL(a, b) ((a > b) ? a : b)
static int fg_calculate_iocv(struct sm_fg_chip *sm)
{
	bool only_lb=false, sign_i_offset=0; //valid_cb=false,
	int roop_start=0, roop_max=0, i=0, cb_last_index = 0, cb_pre_last_index =0;
	int lb_v_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int lb_i_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int cb_v_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int cb_i_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int i_offset_margin = 0x14, i_vset_margin = 0x67;
	int v_max=0, v_min=0, v_sum=0, lb_v_avg=0, cb_v_avg=0, lb_v_set=0, lb_i_set=0, i_offset=0;
	int i_max=0, i_min=0, i_sum=0, lb_i_avg=0, cb_i_avg=0, cb_v_set=0, cb_i_set=0;
	int lb_i_p_v_min=0, lb_i_n_v_max=0, cb_i_p_v_min=0, cb_i_n_v_max=0;

	u16 v_ret, i_ret = 0;
	int ret=0;

	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_END_V_IDX, &data);
	if (ret < 0) {
			pr_err("Failed to read FG_REG_END_V_IDX, ret = %d\n", ret);
			return ret;
	} else {
		pr_info("iocv_status_read = addr : 0x%x , data : 0x%x\n", FG_REG_END_V_IDX, data);
	}

	if((data & 0x0010) == 0x0000)
	{
		only_lb = true;
	}

    	roop_max = (data & 0x000F);
	if(roop_max > FG_INIT_B_LEN)
		roop_max = FG_INIT_B_LEN;

	roop_start = FG_REG_START_LB_V;
	for (i = roop_start; i < roop_start + roop_max; i++)
	{
		ret = fg_read_word(sm, i, &v_ret);
		if (ret < 0) {
			pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
			return ret;
		}
		ret = fg_read_word(sm, i+0x20, &i_ret);
		if (ret < 0) {
			pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
			return ret;
		}

		if((i_ret&0x4000) == 0x4000)
		{
			i_ret = -(i_ret&0x3FFF);
		}

		lb_v_buffer[i-roop_start] = v_ret;
		lb_i_buffer[i-roop_start] = i_ret;

		if (i == roop_start)
		{
			v_max = v_ret;
			v_min = v_ret;
			v_sum = v_ret;
			i_max = i_ret;
			i_min = i_ret;
			i_sum = i_ret;
		}
		else
		{
			if(v_ret > v_max)
				v_max = v_ret;
			else if(v_ret < v_min)
				v_min = v_ret;
			v_sum = v_sum + v_ret;

			if(i_ret > i_max)
				i_max = i_ret;
			else if(i_ret < i_min)
				i_min = i_ret;
			i_sum = i_sum + i_ret;
		}

		if(abs(i_ret) > i_vset_margin)
		{
			if(i_ret > 0)
			{
				if(lb_i_p_v_min == 0)
				{
					lb_i_p_v_min = v_ret;
				}
				else
				{
					if(v_ret < lb_i_p_v_min)
						lb_i_p_v_min = v_ret;
				}
			}
			else
			{
				if(lb_i_n_v_max == 0)
				{
					lb_i_n_v_max = v_ret;
				}
				else
				{
					if(v_ret > lb_i_n_v_max)
						lb_i_n_v_max = v_ret;
				}
			}
		}
	}
	v_sum = v_sum - v_max - v_min;
	i_sum = i_sum - i_max - i_min;

	lb_v_avg = v_sum / (roop_max-2);
	lb_i_avg = i_sum / (roop_max-2);

	if(abs(lb_i_buffer[roop_max-1]) < i_vset_margin)
	{
		if(abs(lb_i_buffer[roop_max-2]) < i_vset_margin)
		{
			lb_v_set = MAXVAL(lb_v_buffer[roop_max-2], lb_v_buffer[roop_max-1]);
			if(abs(lb_i_buffer[roop_max-3]) < i_vset_margin)
			{
				lb_v_set = MAXVAL(lb_v_buffer[roop_max-3], lb_v_set);
			}
		}
		else
		{
			lb_v_set = lb_v_buffer[roop_max-1];
		}
	}
	else
	{
		lb_v_set = lb_v_avg;
	}

	if(lb_i_n_v_max > 0)
	{
		lb_v_set = MAXVAL(lb_i_n_v_max, lb_v_set);
	}

	if(roop_max > 3)
	{
		lb_i_set = (lb_i_buffer[2] + lb_i_buffer[3]) / 2;
	}

	if((abs(lb_i_buffer[roop_max-1]) < i_offset_margin) && (abs(lb_i_set) < i_offset_margin))
	{
		lb_i_set = MAXVAL(lb_i_buffer[roop_max-1], lb_i_set);
	}
	else if(abs(lb_i_buffer[roop_max-1]) < i_offset_margin)
	{
		lb_i_set = lb_i_buffer[roop_max-1];
	}
	else if(abs(lb_i_set) < i_offset_margin)
	{
		//lb_i_set = lb_i_set;
	}
	else
	{
		lb_i_set = 0;
	}

	i_offset = lb_i_set;

	i_offset = i_offset + 4;

	if(i_offset <= 0)
	{
		sign_i_offset = 1;
#ifdef IGNORE_N_I_OFFSET
		i_offset = 0;
#else
		i_offset = -i_offset;
#endif
	}

	i_offset = i_offset>>1;

	if(sign_i_offset == 0)
	{
		i_offset = i_offset|0x0080;
	}
    i_offset = i_offset | i_offset<<8;

	pr_info("%s: iocv_l_max=0x%x, iocv_l_min=0x%x, iocv_l_avg=0x%x, lb_v_set=0x%x, roop_max=%d \n",
			__func__, v_max, v_min, lb_v_avg, lb_v_set, roop_max);
	pr_info("%s: ioci_l_max=0x%x, ioci_l_min=0x%x, ioci_l_avg=0x%x, lb_i_set=0x%x, i_offset=0x%x, sign_i_offset=%d\n",
			__func__, i_max, i_min, lb_i_avg, lb_i_set, i_offset, sign_i_offset);

	if(!only_lb)
	{
		roop_start = FG_REG_START_CB_V;
		roop_max = 6;
		for (i = roop_start; i < roop_start + roop_max; i++)
		{
			ret = fg_read_word(sm, i, &v_ret);
			if (ret < 0) {
				pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
				return ret;
			}
			ret = fg_read_word(sm, i+0x20, &i_ret);
			if (ret < 0) {
				pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
				return ret;
			}

			if((i_ret&0x4000) == 0x4000)
			{
				i_ret = -(i_ret&0x3FFF);
			}

			cb_v_buffer[i-roop_start] = v_ret;
			cb_i_buffer[i-roop_start] = i_ret;

			if (i == roop_start)
			{
				v_max = v_ret;
				v_min = v_ret;
				v_sum = v_ret;
				i_max = i_ret;
				i_min = i_ret;
				i_sum = i_ret;
			}
			else
			{
				if(v_ret > v_max)
					v_max = v_ret;
				else if(v_ret < v_min)
					v_min = v_ret;
				v_sum = v_sum + v_ret;

				if(i_ret > i_max)
					i_max = i_ret;
				else if(i_ret < i_min)
					i_min = i_ret;
				i_sum = i_sum + i_ret;
			}

			if(abs(i_ret) > i_vset_margin)
			{
				if(i_ret > 0)
				{
					if(cb_i_p_v_min == 0)
					{
						cb_i_p_v_min = v_ret;
					}
					else
					{
						if(v_ret < cb_i_p_v_min)
							cb_i_p_v_min = v_ret;
					}
				}
				else
				{
					if(cb_i_n_v_max == 0)
					{
						cb_i_n_v_max = v_ret;
					}
					else
					{
						if(v_ret > cb_i_n_v_max)
							cb_i_n_v_max = v_ret;
					}
				}
			}
		}
		v_sum = v_sum - v_max - v_min;
		i_sum = i_sum - i_max - i_min;

		cb_v_avg = v_sum / (roop_max-2);
		cb_i_avg = i_sum / (roop_max-2);

		cb_last_index = (data & 0x000F)-7; //-6-1
		if(cb_last_index < 0)
		{
			cb_last_index = 5;
		}

		for (i = roop_max; i > 0; i--)
		{
			if(abs(cb_i_buffer[cb_last_index]) < i_vset_margin)
			{
				cb_v_set = cb_v_buffer[cb_last_index];
				if(abs(cb_i_buffer[cb_last_index]) < i_offset_margin)
				{
					cb_i_set = cb_i_buffer[cb_last_index];
				}

				cb_pre_last_index = cb_last_index - 1;
				if(cb_pre_last_index < 0)
				{
					cb_pre_last_index = 5;
				}

				if(abs(cb_i_buffer[cb_pre_last_index]) < i_vset_margin)
				{
					cb_v_set = MAXVAL(cb_v_buffer[cb_pre_last_index], cb_v_set);
					if(abs(cb_i_buffer[cb_pre_last_index]) < i_offset_margin)
					{
						cb_i_set = MAXVAL(cb_i_buffer[cb_pre_last_index], cb_i_set);
					}
				}
			}
			else
			{
				cb_last_index--;
				if(cb_last_index < 0)
				{
					cb_last_index = 5;
				}
			}
		}

		if(cb_v_set == 0)
		{
			cb_v_set = cb_v_avg;
			if(cb_i_set == 0)
			{
				cb_i_set = cb_i_avg;
			}
		}

		if(cb_i_n_v_max > 0)
		{
			cb_v_set = MAXVAL(cb_i_n_v_max, cb_v_set);
		}

		if(abs(cb_i_set) < i_offset_margin)
		{
			if(cb_i_set > lb_i_set)
			{
				i_offset = cb_i_set;
				i_offset = i_offset + 4;

				if(i_offset <= 0)
				{
					sign_i_offset = 1;
#ifdef IGNORE_N_I_OFFSET
					i_offset = 0;
#else
					i_offset = -i_offset;
#endif
				}

				i_offset = i_offset>>1;

				if(sign_i_offset == 0)
				{
					i_offset = i_offset|0x0080;
				}
                i_offset = i_offset | i_offset<<8;

			}
		}

		pr_info("%s: iocv_c_max=0x%x, iocv_c_min=0x%x, iocv_c_avg=0x%x, cb_v_set=0x%x, cb_last_index=%d\n",
				__func__, v_max, v_min, cb_v_avg, cb_v_set, cb_last_index);
		pr_info("%s: ioci_c_max=0x%x, ioci_c_min=0x%x, ioci_c_avg=0x%x, cb_i_set=0x%x, i_offset=0x%x, sign_i_offset=%d\n",
				__func__, i_max, i_min, cb_i_avg, cb_i_set, i_offset, sign_i_offset);

	}

	if((abs(cb_i_set) > i_vset_margin) || only_lb)
	{
		ret = MAXVAL(lb_v_set, cb_i_n_v_max);
	}
	else
	{
		ret = cb_v_set;
	}

    if(ret > sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1])
    {
        pr_info("iocv ret change 0x%x -> 0x%x \n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
        ret = sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1];
    }
    else if(ret < sm->battery_table[BATTERY_TABLE0][0])
    {
        pr_info("iocv ret change 0x%x -> 0x%x \n", ret, (sm->battery_table[BATTERY_TABLE0][0] + 0x10));
        ret = sm->battery_table[BATTERY_TABLE0][0] + 0x10;
    }

	return ret;
}

static bool fg_reg_init(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int i, j, value, ret, cnt = 0;
	uint8_t table_reg;
	u16 data, data_int_mask = 0;

	pr_info("sm5602_fg_reg_init START!!\n");

	/* Init mark */
	if (sm->fg_irq_set == -EINVAL) {
		pr_err("sm->fg_irq_set is invalid");
	} else {
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_INT_MASK], &data_int_mask);
		if (ret < 0){
			pr_err("Failed to read INT_MASK, ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK], 0x4000 | (data_int_mask | sm->fg_irq_set));
	    if (ret < 0) {
			pr_err("Failed to write 0x4000 | INIT_MASK, ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK], 0x07FF & (data_int_mask | sm->fg_irq_set));
	    if (ret < 0) {
			pr_err("Failed to write INIT_MASK, ret = %d\n", ret);
			return ret;
		}
	}

	/* Low SOC1  */
	if (sm->low_soc1 == -EINVAL) {
		pr_err("sm->low_soc1 is invalid");
	} else {
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM], &data);
		if (ret < 0){
			pr_err("Failed to read SOC_L_ALARM (LOW_SOC1), ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM], ((data & 0xFFE0) | sm->low_soc1));
	    if (ret < 0) {
			pr_err("Failed to write SOC_L_ALARM (LOW_SOC1), ret = %d\n", ret);
			return ret;
		}
	}

	/* Low SOC2  */
	if (sm->low_soc2 == -EINVAL) {
		pr_err("sm->low_soc2 is invalid");
	} else {
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM], &data);
		if (ret < 0){
			pr_err("Failed to read SOC_L_ALARM (LOW_SOC2), ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_SOC_L_ALARM],((data & 0xE0FF) | (sm->low_soc2 << 8)));
	    if (ret < 0) {
			pr_err("Failed to write LOW_SOC2, ret = %d\n", ret);
			return ret;
		}
	}

	/* V L ALARM  */
	if (sm->v_l_alarm == -EINVAL) {
		pr_err("sm->v_l_alarm is invalid");
	} else {
		if (sm->v_l_alarm >= 2000 && sm->v_l_alarm < 3000)
			data = (0xFEFF & (sm->v_l_alarm/10 * 256));
		else if (sm->v_l_alarm >= 3000 && sm->v_l_alarm < 4000)
			data = (0x0100 | (sm->v_l_alarm/10 * 256));
		else {
			ret = -EINVAL;
			pr_err("Failed to calculate V_L_ALARM, ret = %d\n", ret);
			return ret;
		}

		ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_L_ALARM], data);
		if (ret < 0) {
			pr_err("Failed to write V_L_ALARM, ret = %d\n", ret);
			return ret;
		}
	}

	/* V H ALARM  */
	if (sm->v_h_alarm == -EINVAL) {
		pr_err("sm->v_h_alarm is invalid");
	} else {
		if (sm->v_h_alarm >= 3000 && sm->v_h_alarm < 4000)
			data = (0xFEFF & (sm->v_h_alarm/10 * 256));
		else if (sm->v_h_alarm >= 4000 && sm->v_h_alarm < 5000)
			data = (0x0100 | (sm->v_h_alarm/10 * 256));
		else {
			ret = -EINVAL;
			pr_err("Failed to calculate V_H_ALARM, ret = %d\n", ret);
			return ret;
		}

		ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_H_ALARM], data);
		if (ret < 0) {
			pr_err("Failed to write V_H_ALARM, ret = %d\n", ret);
			return ret;
		}
	}

	/* T IN H/L ALARM  */
	if (sm->t_h_alarm_in == -EINVAL
		|| sm->t_l_alarm_in == -EINVAL) {
		pr_err("sm->t_h_alarm_in || sm->t_l_alarm_in is invalid");
	} else {
		data = 0; //clear value
		//T IN H ALARM
		if (sm->t_h_alarm_in < 0) {
			data |= 0x8000;
			data |= ((((-1)*sm->t_h_alarm_in) & 0x7F) << 8);
		} else {
			data |= (((sm->t_h_alarm_in) & 0x7F) << 8);
		}
		//T IN L ALARM
		if (sm->t_l_alarm_in < 0) {
			data |= 0x0080;
			data |= ((((-1)*sm->t_l_alarm_in) & 0x7F));
		} else {
			data |= (((sm->t_l_alarm_in) & 0x7F));
		}

		ret = fg_write_word(sm, sm->regs[SM_FG_REG_T_IN_H_ALARM], data);
		if (ret < 0) {
			pr_err("Failed to write SM_FG_REG_T_IN_H_ALARM, ret = %d\n", ret);
			return ret;
		}
	}

	do {
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], (FG_PARAM_UNLOCK_CODE | ((sm->battery_table_num & 0x0003) << 6) | (FG_TABLE_LEN-1)));
		if (ret < 0) {
			pr_err("Failed to write param_ctrl unlock, ret = %d\n", ret);
			return ret;
		} else {
			pr_info("Param Unlock\n");
		}
		//msleep(3);
		msleep(60);
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
		if (ret < 0){
			pr_err("Failed to read FG_OP_STATUS, ret = %d\n", ret);
		} else {
			pr_info(" FG_OP_STATUS = 0x%x\n", data);
		}
		cnt++;

	} while(((data & 0x03)!=0x03) && cnt <= 3);

	/* VIT_PERIOD write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
	if (ret < 0) {
		pr_err("Failed to write VIT PERIOD, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write VIT_PERIOD = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
	}

	/* Aging ctrl write */
	ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_AGING_CTRL, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write FG_REG_AGING_CTRL = 0x%x : 0x%x\n", FG_REG_AGING_CTRL, sm->aging_ctrl);
	}

	/* SOC Cycle ctrl write */
	ret = fg_write_word(sm, FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_SOC_CYCLE_CFG, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write FG_REG_SOC_CYCLE_CFG = 0x%x : 0x%x\n", FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
	}

	/*RSNS write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_RSNS_SEL, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write SM_FG_REG_RSNS_SEL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);
	}

	/* Battery_Table write */
	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		table_reg = 0xA0 + (i*FG_TABLE_LEN);
		for (j = 0; j < FG_TABLE_LEN; j++) {
			ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
			if (ret < 0) {
				pr_err("Failed to write Battery Table, ret = %d\n", ret);
				return ret;
			} else {
				pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
					i, j, (table_reg + j), sm->battery_table[i][j]);
			}
		}
	}

	for(j=0; j < FG_ADD_TABLE_LEN; j++)
	{
		table_reg = 0xD0 + j;
		ret = fg_write_word(sm, table_reg, sm->battery_table[i][j]);
		if (ret < 0) {
			pr_err("Failed to write Battery Table, ret = %d\n", ret);
			return ret;
		} else {
			pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
				i, j, table_reg, sm->battery_table[i][j]);
		}
	}

	/*  RS write */
	ret = fg_write_word(sm, FG_REG_RS, sm->rs);
	if (ret < 0) {
		pr_err("Failed to write RS, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS = 0x%x : 0x%x\n",FG_REG_RS, sm->rs);
	}

	/*  alpha write */
	ret = fg_write_word(sm, FG_REG_ALPHA, sm->alpha);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_ALPHA, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("ALPHA = 0x%x : 0x%x\n",FG_REG_ALPHA, sm->alpha);
	}

	/*  beta write */
	ret = fg_write_word(sm, FG_REG_BETA, sm->beta);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_BETA, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("BETA = 0x%x : 0x%x\n",FG_REG_BETA, sm->beta);
	}

	/*  RS write */
	ret = fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
	if (ret < 0) {
		pr_err("Failed to write RS_0, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS = 0x%x : 0x%x\n",FG_REG_RS_0, sm->rs_value[0]);
	}

	ret = fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
	if (ret < 0) {
		pr_err("Failed to write RS_1, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_1 = 0x%x : 0x%x\n", FG_REG_RS_1, sm->rs_value[1]);
	}

	ret = fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
	if (ret < 0) {
		pr_err("Failed to write RS_2, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_2 = 0x%x : 0x%x\n", FG_REG_RS_2, sm->rs_value[2]);
	}

	ret = fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
	if (ret < 0) {
		pr_err("Failed to write RS_3, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_3 = 0x%x : 0x%x\n", FG_REG_RS_3, sm->rs_value[3]);
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
	if (ret < 0) {
		pr_err("Failed to write CURRENT_RATE, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURRENT_RATE = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
	}

	pr_info("RS_0 = 0x%x, RS_1 = 0x%x, RS_2 = 0x%x, RS_3 = 0x%x, CURRENT_RATE = 0x%x\n",
		sm->rs_value[0], sm->rs_value[1], sm->rs_value[2], sm->rs_value[3], sm->mix_value);

	/* VOLT_CAL write*/
	ret = fg_write_word(sm, FG_REG_VOLT_CAL, sm->volt_cal);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_VOLT_CAL, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("FG_REG_VOLT_CAL = 0x%x : 0x%x\n", FG_REG_VOLT_CAL, sm->volt_cal);
	}

	/* CAL write*/
	ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, sm->curr_offset);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURR_IN_OFFSET = 0x%x : 0x%x\n", FG_REG_CURR_IN_OFFSET, sm->curr_offset);
	}
	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, sm->curr_slope);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURR_IN_SLOPE = 0x%x : 0x%x\n", FG_REG_CURR_IN_SLOPE, sm->curr_slope);
	}

	/* BAT CAP write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
	if (ret < 0) {
		pr_err("Failed to write BAT_CAP, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("BAT_CAP = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
	}

	/* MISC write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_MISC], sm->misc);
	if (ret < 0) {
		pr_err("Failed to write REG_MISC, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("REG_MISC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_MISC], sm->misc);
	}

	/* TOPOFF SOC */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_TOPOFFSOC, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("SM_REG_TOPOFFSOC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);
	}

	/*INIT_last -  control register set*/
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
	if (ret < 0) {
			pr_err("Failed to read CNTL, ret = %d\n", ret);
			return ret;
	}

	if (sm->en_temp_in)
		data |= ENABLE_EN_TEMP_IN;
	if (sm->en_temp_ex)
		data |= ENABLE_EN_TEMP_EX;
	if (sm->en_batt_det)
		data |= ENABLE_EN_BATT_DET;
	if (sm->iocv_man_mode)
		data |= ENABLE_IOCV_MAN_MODE;

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
	if (ret < 0) {
		pr_err("Failed to write CNTL, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
	}

	/* Parameter Version [COMMON(0~255) | BATTERY(0~255)] */
	ret = fg_write_word(sm, FG_PARAM_VERION, ((sm->common_param_version << 8) | sm->battery_param_version));
	if (ret < 0) {
		pr_err("Failed to write FG_PARAM_VERION, ret = %d\n", ret);
		return ret;
	}

	/* T EX L ALARM  */
	if (sm->t_l_alarm_ex == -EINVAL) {
		pr_err("sm->t_l_alarm_ex is invalid");
	} else {
		data = (sm->t_l_alarm_ex) >> 1; //NTC Value/2

		ret = fg_write_word(sm, FG_REG_SWADDR, 0x6A);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, data);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}

		pr_info("write to T_EX_H_ALARM = 0x%x\n", data);
	}

	/* T EX H ALARM  */
	if (sm->t_h_alarm_ex == -EINVAL) {
		pr_err("sm->t_h_alarm_ex is invalid");
	} else {
		data = (sm->t_h_alarm_ex) >> 1; //NTC Value/2

		ret = fg_write_word(sm, FG_REG_SWADDR, 0x6B);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, data);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}

		pr_info("write to T_EX_L_ALARM = 0x%x\n", data);
	}

	if (sm->iocv_man_mode) {
		value = fg_calculate_iocv(sm);

	    msleep(10);
		ret = fg_write_word(sm, FG_REG_SWADDR, 0x75);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, value);
		if (ret < 0) {
			pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
			return ret;
		}
		pr_info("IOCV_MAN : 0x%x\n", value);
	}

	msleep(20);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], ((FG_PARAM_LOCK_CODE | (sm->battery_table_num & 0x0003) << 6) | (FG_TABLE_LEN-1)));
	if (ret < 0) {
		pr_err("Failed to write param_ctrl lock, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("Param Lock\n");
	}

	msleep(160);

	return 1;
}

static unsigned int fg_get_device_id(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int ret;
	u16 data;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_DEVICE_ID], &data);
	if (ret < 0) {
		pr_err("Failed to read DEVICE_ID, ret = %d\n", ret);
		return ret;
	}

	pr_info("revision_id = 0x%x\n",(data & 0x000f));
	pr_info("device_id = 0x%x\n",(data & 0x00f0)>>4);

	return ret;
}
static bool fg_init(struct i2c_client *client)
{
	int ret;
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	/*sm5602 i2c read check*/

	if (fg_check_reg_init_need(client)) {
		ret = fg_reset(sm);
		if (ret < 0) {
			pr_err("%s: fail to do reset(%d)\n", __func__, ret);
			return false;
		}
		fg_reg_init(client);
	}

	//sm->is_charging = (sm->batt_current > 9) ? true : false;
	//pr_err("is_charging = %dd\n",sm->is_charging);

	return true;
}

#define PROPERTY_NAME_SIZE 128
static int fg_common_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *np = dev->of_node;
	int rc;

	BUG_ON(dev == 0);
	BUG_ON(np == 0);

	/*
	sm->gpio_int = of_get_named_gpio(np, "qcom,irq-gpio", 0);
	pr_info("gpio_int=%d\n", sm->gpio_int);

	if (!gpio_is_valid(sm->gpio_int)) {
		pr_info("gpio_int is not valid\n");
		sm->gpio_int = -EINVAL;
	}
	*/
	rc = of_property_read_u32(np, "sm,fcc_design",
					&sm->batt_fcc_design);
	if (rc < 0)
		sm->batt_fcc_design = 5000;
	pr_info("sm,fcc_design = %d\n", sm->batt_fcc_design);

	/* EN TEMP EX/IN */
	if (of_property_read_bool(np, "sm,en_temp_ex"))
		sm->en_temp_ex = true;
	else
		sm->en_temp_ex = 0;
	pr_info("Temperature EX enabled = %d\n", sm->en_temp_ex);

	if (of_property_read_bool(np, "sm,en_temp_in"))
		sm->en_temp_in = true;
	else
		sm->en_temp_in = 0;
	pr_info("Temperature IN enabled = %d\n", sm->en_temp_in);

	/* EN BATT DET  */
	if (of_property_read_bool(np, "sm,en_batt_det"))
		sm->en_batt_det = true;
	else
		sm->en_batt_det = 0;
	pr_info("Batt Det enabled = %d\n", sm->en_batt_det);

    /* MISC */
	rc = of_property_read_u32(np, "sm,misc",
                        &sm->misc);
	if (rc < 0)
		sm->misc = 0x0800;

	/* IOCV MAN MODE */
	if (of_property_read_bool(np, "sm,iocv_man_mode"))
		sm->iocv_man_mode = true;
	else
		sm->iocv_man_mode = 0;
	pr_info("IOCV_MAN_MODE = %d\n", sm->iocv_man_mode);

	/* Aging */
	rc = of_property_read_u32(np, "sm,aging_ctrl",&sm->aging_ctrl);
	if (rc < 0)
		sm->aging_ctrl = -EINVAL;

	/* SOC Cycle cfg */
	rc = of_property_read_u32(np, "sm,cycle_cfg",
                        &sm->cycle_cfg);
	if (rc < 0)
		sm->cycle_cfg = -EINVAL;

	/* RSNS */
	rc = of_property_read_u32(np, "sm,rsns",
                        &sm->batt_rsns);
	if (rc < 0)
		sm->batt_rsns = -EINVAL;

	rc = of_property_read_u32(np, "factory_mode_ntc_exist",
                        &sm->ntc_exist);
	if (rc < 0)
		sm->ntc_exist = true;

	/* IRQ Mask */
	rc = of_property_read_u32(np, "sm,fg_irq_set",
                        &sm->fg_irq_set);
	if (rc < 0)
		sm->fg_irq_set = -EINVAL;

	/* LOW SOC1/2 */
	rc = of_property_read_u32(np, "sm,low_soc1",
    					&sm->low_soc1);
	if (rc < 0)
		sm->low_soc1 = -EINVAL;
	pr_info("low_soc1 = %d\n", sm->low_soc1);

	rc = of_property_read_u32(np, "sm,low_soc2",
					&sm->low_soc2);
	if (rc < 0)
		sm->low_soc2 = -EINVAL;
	pr_info("low_soc2 = %d\n", sm->low_soc2);

	/* V_L/H_ALARM */
	rc = of_property_read_u32(np, "sm,v_l_alarm",
    					&sm->v_l_alarm);
	if (rc < 0)
		sm->v_l_alarm = -EINVAL;
	pr_info("v_l_alarm = %d\n", sm->v_l_alarm);

	rc = of_property_read_u32(np, "sm,v_h_alarm",
					&sm->v_h_alarm);
	if (rc < 0)
		sm->v_h_alarm = -EINVAL;
	pr_info("v_h_alarm = %d\n", sm->v_h_alarm);

	/* T_IN_H/L_ALARM */
	rc = of_property_read_u32(np, "sm,t_l_alarm_in",
    					&sm->t_l_alarm_in);
	if (rc < 0)
		sm->t_l_alarm_in = -EINVAL;
	pr_info("t_l_alarm_in = %d\n", sm->t_l_alarm_in);

	rc = of_property_read_u32(np, "sm,t_h_alarm_in",
					&sm->t_h_alarm_in);
	if (rc < 0)
		sm->t_h_alarm_in = -EINVAL;
	pr_info("t_h_alarm_in = %d\n", sm->t_h_alarm_in);

	/* T_EX_H/L_ALARM */
	rc = of_property_read_u32(np, "sm,t_l_alarm_ex",
    					&sm->t_l_alarm_ex);
	if (rc < 0)
		sm->t_l_alarm_ex = -EINVAL;
	pr_info("t_l_alarm_ex = %d\n", sm->t_l_alarm_ex);

	rc = of_property_read_u32(np, "sm,t_h_alarm_ex",
					&sm->t_h_alarm_ex);
	if (rc < 0)
		sm->t_h_alarm_ex = -EINVAL;
	pr_info("t_h_alarm_ex = %d\n", sm->t_h_alarm_ex);

	/* Battery Table Number */
	rc = of_property_read_u32(np, "sm,battery_table_num",
                        &sm->battery_table_num);
	if (rc < 0)
		sm->battery_table_num = -EINVAL;

	/* Paramater Number */
	rc = of_property_read_u32(np, "sm,param_version",
                        &sm->common_param_version);
	if (rc < 0)
		sm->common_param_version = -EINVAL;
	pr_info("common_param_version = %d\n", sm->common_param_version);

	return 0;
}

static const char *get_battery_serialnumber(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *battsn_buf;
	int retval;

	battsn_buf = NULL;

	if (np)
		retval = of_property_read_string(np, "mmi,battid",
						 &battsn_buf);
	else
		return NULL;

	if ((retval == -EINVAL) || !battsn_buf) {
		pr_info(" Battsn unused\n");
		of_node_put(np);
		return NULL;

	} else
		pr_info("Battsn = %s\n", battsn_buf);

	of_node_put(np);

	return battsn_buf;
}

static struct device_node *get_profile_by_serialnumber(
		const struct device_node *np)
{
	struct device_node *node, *df_node, *sn_node;
	const char *sn_buf, *df_sn, *dev_sn;
	int rc;

	if (!np)
		return NULL;

	dev_sn = NULL;
	df_sn = NULL;
	sn_buf = NULL;
	df_node = NULL;
	sn_node = NULL;

	dev_sn = get_battery_serialnumber();

	rc = of_property_read_string(np, "df-serialnum",
				     &df_sn);
	if (rc)
		pr_info("No Default Serial Number defined\n");
	else if (df_sn)
		pr_info("Default Serial Number %s\n", df_sn);

	for_each_child_of_node(np, node) {
		rc = of_property_read_string(node, "serialnum",
					     &sn_buf);
		if (!rc && sn_buf) {
			if (dev_sn)
				if (strnstr(dev_sn, sn_buf, 32))
					sn_node = node;
			if (df_sn)
				if (strnstr(df_sn, sn_buf, 32))
					df_node = node;
		}
	}

	if (sn_node) {
		node = sn_node;
		df_node = NULL;
		pr_info("Battery Match Found using %s\n", sn_node->name);
	} else if (df_node) {
		node = df_node;
		sn_node = NULL;
		pr_info("Battery Match Found using default %s\n",
				df_node->name);
	} else {
		pr_info("No Battery Match Found!\n");
		return NULL;
	}

	return node;
}

static int fg_battery_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *np = dev->of_node;
	struct device_node *batt_profile_node = NULL;
	char prop_name[PROPERTY_NAME_SIZE];
	int battery_temp_table[FG_TEMP_TABLE_CNT_MAX];
	int table[FG_TABLE_LEN];
	int rs_value[4];
	int topoff_soc[3];
	int temp_offset[6];
	int temp_cal[10];
	int ext_temp_cal[10];
	int battery_type[3];
	int set_temp_poff[4];
	int ret;
	int i, j;

	BUG_ON(dev == 0);
	BUG_ON(np == 0);
	batt_profile_node = get_profile_by_serialnumber(np);
	if (!batt_profile_node) {
		pr_err("sm5602 %s get battery parameter dts fail.\n", __func__);
		return -1;
	}
	/*  battery_table*/
	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE,
			 "battery,%s%d", "battery_table", i);

		ret = of_property_read_u32_array(batt_profile_node, prop_name, table, FG_TABLE_LEN);
		if (ret < 0)
			pr_info("Can get prop %s (%d)\n", prop_name, ret);
		for (j = 0; j < FG_TABLE_LEN; j++) {
			sm->battery_table[i][j] = table[j];
			pr_info("%s = <table[%d][%d] 0x%x>\n",
				prop_name, i, j, table[j]);
		}
	}

	i = BATTERY_TABLE2;
	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery,%s%d", "battery_table", i);
	ret = of_property_read_u32_array(batt_profile_node, prop_name, table, FG_ADD_TABLE_LEN);
	if (ret < 0)
		pr_info("Can get prop %s (%d)\n", prop_name, ret);
	else {
		for(j=0; j < FG_ADD_TABLE_LEN; j++)
		{
			sm->battery_table[i][j] = table[j];
			pr_info("%s = <table[%d][%d] 0x%x>\n",
				prop_name, i, j, table[j]);
		}
	}

    /* rs */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "rs");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->rs, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->rs);

    /* alpha */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "alpha");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->alpha, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->alpha);

    /* beta */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "beta");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->beta, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->beta);

	/* rs_value*/
	for (i = 0; i < 4; i++) {
		snprintf(prop_name,
			PROPERTY_NAME_SIZE, "battery,%s", "rs_value");
		ret = of_property_read_u32_array(batt_profile_node, prop_name, rs_value, 4);
		if (ret < 0)
			pr_err("Can get prop %s (%d)\n", prop_name, ret);
		sm->rs_value[i] = rs_value[i];
	}
	pr_info("%s = <0x%x 0x%x 0x%x 0x%x>\n",
		prop_name, rs_value[0], rs_value[1], rs_value[2], rs_value[3]);

	/* vit_period*/
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "vit_period");
	ret = of_property_read_u32_array(batt_profile_node,
		prop_name, &sm->vit_period, 1);
	if (ret < 0)
		pr_info("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->vit_period);

    /* battery_type*/
    snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "battery_type");
    ret = of_property_read_u32_array(batt_profile_node, prop_name, battery_type, 3);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    sm->batt_v_max = battery_type[0];
    sm->min_cap = battery_type[1];
    sm->cap = battery_type[2];

    pr_info("%s = <%d %d %d>\n", prop_name,
        sm->batt_v_max, sm->min_cap, sm->cap);

	/* tem poff level */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "tem_poff");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, set_temp_poff, 2);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->n_tem_poff = set_temp_poff[0];
	sm->n_tem_poff_offset = set_temp_poff[1];

	pr_info("%s = <%d, %d>\n",
		prop_name,
		sm->n_tem_poff, sm->n_tem_poff_offset);

    /* max-voltage -mv*/
    snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "max_voltage_uv");
	ret = of_property_read_u32(batt_profile_node, prop_name,
				        &sm->batt_max_voltage_uv);
	if (ret < 0)
	    pr_err("couldn't find battery max voltage\n");

    // TOPOFF SOC
    snprintf(prop_name,
    	PROPERTY_NAME_SIZE, "battery,%s", "topoff_soc");
    ret = of_property_read_u32_array(batt_profile_node, prop_name, topoff_soc, 2);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    sm->topoff_soc = topoff_soc[0];
    sm->top_off = topoff_soc[1];

    pr_info("%s = <%d %d>\n", prop_name,
        sm->topoff_soc, sm->top_off);

    // Mix
    snprintf(prop_name,
    	PROPERTY_NAME_SIZE, "battery,%s", "mix_value");
    ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->mix_value, 1);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);

    pr_info("%s = <%d>\n", prop_name,
        sm->mix_value);

    /* VOLT CAL */
    snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "volt_cal");
    ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->volt_cal, 1);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    pr_info("%s = <0x%x>\n", prop_name, sm->volt_cal);

	/* CURR OFFSET */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "curr_offset");
	ret = of_property_read_u32_array(batt_profile_node,
		prop_name, &sm->curr_offset, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->curr_offset);

	/* CURR SLOPE */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "curr_slope");
	ret = of_property_read_u32_array(batt_profile_node,
		prop_name, &sm->curr_slope, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->curr_slope);

	/* temp_std */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "temp_std");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->temp_std, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <%d>\n", prop_name, sm->temp_std);

	/* temp_offset */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "temp_offset");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, temp_offset, 6);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_fg_temp_offset = temp_offset[0];
	sm->high_fg_temp_offset_denom = temp_offset[1];
	sm->high_fg_temp_offset_fact = temp_offset[2];
	sm->en_low_fg_temp_offset = temp_offset[3];
	sm->low_fg_temp_offset_denom = temp_offset[4];
	sm->low_fg_temp_offset_fact = temp_offset[5];
	pr_info("%s = <%d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_fg_temp_offset,
		sm->high_fg_temp_offset_denom, sm->high_fg_temp_offset_fact,
		sm->en_low_fg_temp_offset,
		sm->low_fg_temp_offset_denom, sm->low_fg_temp_offset_fact);

	/* temp_calc */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "temp_cal");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, temp_cal, 10);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_fg_temp_cal = temp_cal[0];
	sm->high_fg_temp_p_cal_denom = temp_cal[1];
	sm->high_fg_temp_p_cal_fact = temp_cal[2];
	sm->high_fg_temp_n_cal_denom = temp_cal[3];
	sm->high_fg_temp_n_cal_fact = temp_cal[4];
	sm->en_low_fg_temp_cal = temp_cal[5];
	sm->low_fg_temp_p_cal_denom = temp_cal[6];
	sm->low_fg_temp_p_cal_fact = temp_cal[7];
	sm->low_fg_temp_n_cal_denom = temp_cal[8];
	sm->low_fg_temp_n_cal_fact = temp_cal[9];
	pr_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_fg_temp_cal,
		sm->high_fg_temp_p_cal_denom, sm->high_fg_temp_p_cal_fact,
		sm->high_fg_temp_n_cal_denom, sm->high_fg_temp_n_cal_fact,
		sm->en_low_fg_temp_cal,
		sm->low_fg_temp_p_cal_denom, sm->low_fg_temp_p_cal_fact,
		sm->low_fg_temp_n_cal_denom, sm->low_fg_temp_n_cal_fact);

	/* ext_temp_calc */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery,%s", "ext_temp_cal");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, ext_temp_cal, 10);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_temp_cal = ext_temp_cal[0];
	sm->high_temp_p_cal_denom = ext_temp_cal[1];
	sm->high_temp_p_cal_fact = ext_temp_cal[2];
	sm->high_temp_n_cal_denom = ext_temp_cal[3];
	sm->high_temp_n_cal_fact = ext_temp_cal[4];
	sm->en_low_temp_cal = ext_temp_cal[5];
	sm->low_temp_p_cal_denom = ext_temp_cal[6];
	sm->low_temp_p_cal_fact = ext_temp_cal[7];
	sm->low_temp_n_cal_denom = ext_temp_cal[8];
	sm->low_temp_n_cal_fact = ext_temp_cal[9];
	pr_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_temp_cal,
		sm->high_temp_p_cal_denom, sm->high_temp_p_cal_fact,
		sm->high_temp_n_cal_denom, sm->high_temp_n_cal_fact,
		sm->en_low_temp_cal,
		sm->low_temp_p_cal_denom, sm->low_temp_p_cal_fact,
		sm->low_temp_n_cal_denom, sm->low_temp_n_cal_fact);

	/* get battery_temp_table*/
	 snprintf(prop_name, PROPERTY_NAME_SIZE,
		  "battery,%s", "thermal_table");

	 ret = of_property_read_u32_array(batt_profile_node, prop_name, battery_temp_table, FG_TEMP_TABLE_CNT_MAX);
	 if (ret < 0)
		 pr_err("Can get prop %s (%d)\n", prop_name, ret);
	 for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
		 sm->battery_temp_table[i] = battery_temp_table[i];
		 pr_err("%s = <battery_temp_table[%d] 0x%x>\n",
			 prop_name, i,	battery_temp_table[i]);
	 }

    /* Battery Paramter */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery,%s", "param_version");
	ret = of_property_read_u32_array(batt_profile_node, prop_name, &sm->battery_param_version, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->battery_param_version);

	return 0;
}

bool hal_fg_init(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int ret = 0;

	pr_info("sm5602 hal_fg_init...\n");

	ret = fg_get_device_id(client);
	if (ret < 0) {
		pr_err("%s: fail to do i2c read(%d)\n", __func__, ret);
		return false;
	}

	mutex_lock(&sm->data_lock);
	if (client->dev.of_node) {
		/* Load common data from DTS*/
		fg_common_parse_dt(sm);
		/* Load battery data from DTS*/
		ret = fg_battery_parse_dt(sm);
		if (ret < 0) {
			pr_err("%s: Load battery data fail,(%d)\n", __func__, ret);
			return false;
		}
	}

	if(!fg_init(client))
        return false;
	//sm->batt_temp = 250;

	mutex_unlock(&sm->data_lock);
	pr_info("hal fg init OK\n");
	return true;
}

static int sm_fg_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{

	int ret = 0;
	struct sm_fg_chip *sm;
	u8 *regs;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};

	pr_info("enter\n");
	sm = devm_kzalloc(&client->dev, sizeof(*sm), GFP_KERNEL);

	if (!sm)
		return -ENOMEM;

	sm->dev = &client->dev;
	sm->client = client;
	sm->chip = id->driver_data;

	sm->batt_soc	= -ENODATA;
	sm->batt_fcc	= -ENODATA;
	//sm->batt_dc		= -ENODATA;
	sm->batt_volt	= -ENODATA;
	sm->batt_temp	= -ENODATA;
	sm->batt_curr	= -ENODATA;
	sm->fake_soc	= -EINVAL;
	sm->fake_temp	= -EINVAL;

	if (sm->chip == SM5602) {
		regs = sm5602_regs;
	} else {
		pr_err("unexpected fuel gauge: %d\n", sm->chip);
		regs = sm5602_regs;
	}

	memcpy(sm->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, sm);

	mutex_init(&sm->i2c_rw_lock);
	mutex_init(&sm->data_lock);

	if (!hal_fg_init(client)) {
		pr_err("Failed to Initialize Fuelgauge\n");
		ret = -EIO;
		goto err_0;
	}

	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_cfg.drv_data = sm;
	psy_desc->name = "bms";
	psy_desc->type = POWER_SUPPLY_TYPE_MAINS;
	psy_desc->properties = fg_props;
	psy_desc->num_properties = ARRAY_SIZE(fg_props);
	psy_desc->get_property = fg_get_property;
	psy_desc->set_property = fg_set_property;
	sm->bms_psy = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if (IS_ERR(sm->bms_psy)) {
		ret = PTR_ERR(sm->bms_psy);
		printk(KERN_ERR"failed to register battery: %d\n", ret);
		return ret;
	}

	sm->smfg_workqueue = create_singlethread_workqueue("smfg_gauge");
	INIT_DELAYED_WORK(&sm->monitor_work, fg_monitor_workfunc);
	queue_delayed_work(sm->smfg_workqueue, &sm->monitor_work, msecs_to_jiffies(queue_start_work_time));

	/*
	if (sm->gpio_int != -EINVAL)
		client->irq = sm->gpio_int;
	else {
		pr_err("Failed to registe gpio interrupt\n");
		goto err_0;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			fg_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			"sm fuel gauge irq", sm);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			goto err_0;
		}
	}
	*/
	//fg_irq_thread(client->irq, sm); // if IRQF_TRIGGER_FALLING or IRQF_TRIGGER_RISING is needed, enable initial irq.

	create_debugfs_entry(sm);

	fg_dump_debug(sm);

	if(is_factory_mode())
		sm->factory_mode = true;

	//schedule_delayed_work(&sm->monitor_work, 10 * HZ);
	pr_info("sm fuel gauge probe successfully, %s\n",device2str[sm->chip]);

	return 0;

err_0:
	return ret;
}


static int sm_fg_remove(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sm->monitor_work);

	mutex_destroy(&sm->data_lock);
	mutex_destroy(&sm->i2c_rw_lock);

	debugfs_remove_recursive(sm->debug_root);

	return 0;

}

#ifdef CONFIG_PM
static int sm_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm_bat = i2c_get_clientdata(client);

	cancel_delayed_work(&sm_bat->monitor_work);
	return 0;
}

static int sm_bat_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm_bat = i2c_get_clientdata(client);

	queue_delayed_work(sm_bat->smfg_workqueue, &sm_bat->monitor_work, msecs_to_jiffies(20));
	return 0;
}

static const struct dev_pm_ops sm_bat_pm_ops = {
	.suspend  = sm_bat_suspend,
	.resume   = sm_bat_resume,
};
#endif

static void sm_fg_shutdown(struct i2c_client *client)
{
	pr_info("sm fuel gauge driver shutdown!\n");
}

static const struct of_device_id sm_fg_match_table[] = {
	{.compatible = "sm,sm5602",},
	{},
};
MODULE_DEVICE_TABLE(of, sm_fg_match_table);

static const struct i2c_device_id sm_fg_id[] = {
	{ "sm5602", SM5602 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sm_fg_id);

static struct i2c_driver sm_fg_driver = {
	.driver		= {
		.name		= "sm5602",
#ifdef CONFIG_PM
		.pm = &sm_bat_pm_ops,
#endif
		.owner		= THIS_MODULE,
		.of_match_table	= sm_fg_match_table,
	},
	.id_table   = sm_fg_id,
	.probe		= sm_fg_probe,
	.remove		= sm_fg_remove,
	.shutdown     = sm_fg_shutdown,
};


module_i2c_driver(sm_fg_driver);

MODULE_DESCRIPTION("SM SM5602 Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Siliconmitus");

