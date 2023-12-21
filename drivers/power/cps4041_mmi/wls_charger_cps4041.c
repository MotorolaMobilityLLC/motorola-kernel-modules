/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * version:1.3
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#include <mtk_charger.h>
#include <moto_wlc.h>
#include "wls_charger_cps4041.h"
#include <linux/alarmtimer.h>
#include <mtk_charger_algorithm_class.h>
#include <linux/power/moto_chg_tcmd.h>


MOTO_WLS_AUTH_T motoauth;

#define CPS_REG_DEBUG 0 //only for dump reg log

#define CPS_CHIP_ID 0x4041

#define CPS_WLS_CHRG_DRV_NAME "cps-wls-charger"

#define CPS_WLS_CHRG_PSY_NAME "wireless"

#define KERNEL_POWER_OFF_CHARGING_BOOT 8
#define LOW_POWER_OFF_CHARGING_BOOT 9

#define VBUS_VALID_MV 4600 //If vbus >= 4.6V,the vbus is valid.

struct cps_wls_chrg_chip *chip = NULL;
static bool CPS_RX_MODE_ERR = false;
static bool CPS_TX_MODE = false;
static bool CPS_TX_IRQ = false;
static bool CPS_RX_CHRG_FULL = false;

enum {
	TX_MODE_EPT_ERR = -2,
	TX_MODE_OVERHEAT = -1,
	TX_MODE_NOT_CONNECTED = 0,
	TX_MODE_POWER_SHARE = 2,
};

struct tags_bootmode {
	uint32_t size;
	uint32_t tag;
	uint32_t bootmode;
	uint32_t boottype;
};

/*define cps rx reg enum*/
typedef enum
{
	CPS_RX_REG_FC_VPA_VOLTAGE,
	CPS_RX_REG_FC_MLDO_VOLTAGE,
	CPS_RX_REG_FC_BOOST_MODE,
	CPS_RX_REG_EPT_VAL,
	CPS_RX_REG_POWER_SET,
	CPS_RX_REG_VOUT_SET,
	CPS_RX_REG_OCP_TH,
	CPS_RX_REG_OVP_TH,
	CPS_RX_REG_DUMY_LOAD_MOD,
	CPS_RX_REG_DUMY_LOAD_NO_MOD,
	CPS_RX_REG_DROP_MIN,
	CPS_RX_REG_DROP_MAX,
	CPS_RX_REG_DROP_MIN_CUR,
	CPS_RX_REG_DROP_MAX_CUR,
	CPS_RX_REG_SS_VAL,
	CPS_RX_REG_CE_VAL,
	CPS_RX_REG_RP_VAL,
	CPS_RX_REG_FOP_VAL,
	CPS_RX_REG_ADC_VRECT,
	CPS_RX_REG_ADC_MLDO_DROP,
	CPS_RX_REG_ADC_IRECT,//CPS_RX_REG_ADC_IOUT
	CPS_RX_REG_ADC_VOUT,
	CPS_RX_REG_ADC_DIE_TMP,
	CPS_RX_REG_PPP_HEADER,
	CPS_RX_REG_PPP_COMMAND,
	CPS_RX_REG_PPP_DATA0,
	CPS_RX_REG_PPP_DATA1,
	CPS_RX_REG_PPP_DATA2,
	CPS_RX_REG_PPP_DATA3,
	CPS_RX_REG_PPP_DATA4,
	CPS_RX_REG_PPP_DATA5,
	CPS_RX_REG_PPP_DATA6,
	CPS_RX_REG_BC_HEADER,
	CPS_RX_REG_BC_COMMAND,
	CPS_RX_REG_BC_DATA0,
	CPS_RX_REG_BC_DATA1,
	CPS_RX_REG_BC_DATA2,
	CPS_RX_REG_BC_DATA3,
	CPS_RX_REG_BC_DATA4,
	CPS_RX_REG_BC_DATA5,
	CPS_RX_REG_BC_DATA6,
	CPS_RX_REG_OP_MODE,
	CPS_RX_REG_NEGO_POWER,
	CPS_RX_REG_MAX
}cps_rx_reg_e;

/*define cps tx reg enum*/
typedef enum
{
	CPS_TX_REG_PPP_HEADER,
	CPS_TX_REG_PPP_COMMAND,
	CPS_TX_REG_PPP_DATA0,
	CPS_TX_REG_PPP_DATA1,
	CPS_TX_REG_PPP_DATA2,
	CPS_TX_REG_PPP_DATA3,
	CPS_TX_REG_PPP_DATA4,
	CPS_TX_REG_PPP_DATA5,
	CPS_TX_REG_PPP_DATA6,
	CPS_TX_REG_BC_HEADER,
	CPS_TX_REG_BC_COMMAND,
	CPS_TX_REG_BC_DATA0,
	CPS_TX_REG_BC_DATA1,
	CPS_TX_REG_BC_DATA2,
	CPS_TX_REG_BC_DATA3,
	CPS_TX_REG_BC_DATA4,
	CPS_TX_REG_BC_DATA5,
	CPS_TX_REG_BC_DATA6,
	CPS_TX_REG_OCP_TH,
	//CPS_TX_REG_UVP_TH,
	CPS_TX_REG_OVP_TH,
	CPS_TX_REG_FOP_MIN,
	CPS_TX_REG_FOP_MAX,
	CPS_TX_REG_PING_FREQ,
	CPS_TX_REG_PING_DUTY,
	CPS_TX_REG_PING_OCP_TH,
	CPS_TX_REG_PING_TIME,
	CPS_TX_REG_PING_INTERVAL,
	CPS_TX_REG_FOD0_TH,
	CPS_TX_REG_FUNC_EN,
	CPS_TX_REG_FOP_VAL,
	CPS_TX_REG_ADC_VIN,
	CPS_TX_REG_ADC_VRECT,
	//CPS_TX_REG_ADC_I_IN,
	CPS_TX_REG_ADC_IPA,
	CPS_TX_REG_CE_VAL,
	CPS_TX_REG_RP_VAL,
	CPS_TX_REG_EPT_RSN,
	CPS_TX_REG_ADC_DIE_TEMP,
	CPS_TX_REG_EPT_CODE,
	CPS_TX_REG_RX_POWER,
	CPS_TX_REG_TX_POWER,
	CPS_TX_REG_POWER_LOSS,
	CPS_TX_REG_MAX
} cps_tx_reg_e;

typedef enum
{
	CPS_COMM_REG_CHIP_ID,
	CPS_COMM_REG_FW_VER,
	CPS_COMM_REG_SYS_MODE,
	CPS_COMM_REG_INT_EN,
	CPS_COMM_REG_INT_FLAG,
	CPS_COMM_REG_INT_CLR,
	CPS_COMM_REG_CMD,
	CPS_COMM_REG_MAX
}cps_comm_reg_e;

#define RX_REG_FOD_CUR_0		0x00C0
#define RX_REG_FOD_CUR_1		0x00C1
#define RX_REG_FOD_CUR_2		0x00C2
#define RX_REG_FOD_CUR_3		0x00C3
#define RX_REG_FOD_CUR_4		0x00C4
#define RX_REG_FOD_CUR_5		0x00C5
#define RX_REG_FOD_CUR_6		0x00C6
#define RX_REG_FOD_C0_GAIN		0x00C7
#define RX_REG_FOD_C0_OFFSET	0x00C8
#define RX_REG_FOD_C1_GAIN		0x00C9
#define RX_REG_FOD_C1_OFFSET	0x00CA
#define RX_REG_FOD_C2_GAIN		0x00CB
#define RX_REG_FOD_C2_OFFSET	0x00CC
#define RX_REG_FOD_C3_GAIN		0x00CD
#define RX_REG_FOD_C3_OFFSET	0x00CE
#define RX_REG_FOD_C4_GAIN		0x00CF
#define RX_REG_FOD_C4_OFFSET	0x00D0
#define RX_REG_FOD_C5_GAIN		0x00D1
#define RX_REG_FOD_C5_OFFSET	0x00D2
#define RX_REG_FOD_C6_GAIN		0x00D3
#define RX_REG_FOD_C6_OFFSET	0x00D4
#define RX_REG_FOD_C7_GAIN		0x00D5
#define RX_REG_FOD_C7_OFFSET	0x00D6

typedef enum {
	MMI_DOCK_LIGHT_OFF = 0x10,
	MMI_DOCK_LIGHT_ON = 0x20,
	MMI_DOCK_LIGHT_BREATH_2S = 0x30,
	MMI_DOCK_LIGHT_BREATH_4S = 0x40,
	MMI_DOCK_LIGHT_DEFAULT = MMI_DOCK_LIGHT_BREATH_4S,
} mmi_dock_light_ctrl;

/* value = fan speed / 100 */
typedef enum {
	MMI_DOCK_FAN_SPEED_OFF= 0,
	MMI_DOCK_FAN_SPEED_LOW = 20,//2000
	MMI_DOCK_FAN_SPEED_HIGH = 40,//4000
	MMI_DOCK_FAN_DEFAULT = MMI_DOCK_FAN_SPEED_HIGH,
} mmi_dock_fan_speed;

typedef struct
{
	uint16_t reg_name;
	uint16_t reg_bytes_len;
	uint32_t reg_addr;
}cps_reg_s;


cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
	//reg name                 bytes number     reg address  /
	{CPS_COMM_REG_CHIP_ID,      2,              0x0000},
	{CPS_COMM_REG_FW_VER,       2,              0x0008},
	{CPS_COMM_REG_SYS_MODE,     1,              0x000A},
	{CPS_COMM_REG_INT_EN,       4,              0x0020},
	{CPS_COMM_REG_INT_FLAG,     4,              0x0024},
	{CPS_COMM_REG_INT_CLR,      4,              0x0028},
	{CPS_COMM_REG_CMD,          4,              0x002C},
};

cps_reg_s cps_rx_reg[CPS_RX_REG_MAX] = {
	// reg name            bytes number      reg address          /
	{CPS_RX_REG_FC_VPA_VOLTAGE,       2,               0x0092},
	{CPS_RX_REG_FC_MLDO_VOLTAGE,      2,               0x0094},
	{CPS_RX_REG_FC_BOOST_MODE,        1,               0x0096},
	{CPS_RX_REG_EPT_VAL,              1,               0x00BE},
	{CPS_RX_REG_POWER_SET,            1,               0x00BF},
	{CPS_RX_REG_VOUT_SET,             2,               0x0090},
	{CPS_RX_REG_OCP_TH,               2,               0x00B6},
	{CPS_RX_REG_OVP_TH,               2,               0x00BC},
	{CPS_RX_REG_DUMY_LOAD_MOD,        1,               0x00A9},
	{CPS_RX_REG_DUMY_LOAD_NO_MOD,     1,               0x00A8},
	{CPS_RX_REG_DROP_MIN,             2,               0x009C},
	{CPS_RX_REG_DROP_MAX,             2,               0x009E},
	{CPS_RX_REG_DROP_MIN_CUR,         2,               0x00A0},
	{CPS_RX_REG_DROP_MAX_CUR,         2,               0x00A2},
	{CPS_RX_REG_SS_VAL,               2,               0x0114},
	{CPS_RX_REG_CE_VAL,               2,               0x0116},
	{CPS_RX_REG_RP_VAL,               2,               0x0118},
	{CPS_RX_REG_FOP_VAL,              2,               0x010E},
	{CPS_RX_REG_ADC_VRECT,            2,               0x0104},
	{CPS_RX_REG_ADC_MLDO_DROP,        2,               0x0106},
	{CPS_RX_REG_ADC_IRECT,            2,               0x0108},
	{CPS_RX_REG_ADC_VOUT,             2,               0x010A},
	{CPS_RX_REG_ADC_DIE_TMP,          2,               0x010C},
	{CPS_RX_REG_PPP_HEADER,           1,               0x0050},
	{CPS_RX_REG_PPP_COMMAND,          1,               0x0051},
	{CPS_RX_REG_PPP_DATA0,            1,               0x0052},
	{CPS_RX_REG_PPP_DATA1,            1,               0x0053},
	{CPS_RX_REG_PPP_DATA2,            1,               0x0054},
	{CPS_RX_REG_PPP_DATA3,            1,               0x0055},
	{CPS_RX_REG_PPP_DATA4,            1,               0x0056},
	{CPS_RX_REG_PPP_DATA5,            1,               0x0057},
	{CPS_RX_REG_PPP_DATA6,            1,               0x0058},
	{CPS_RX_REG_BC_HEADER,            1,               0x0070},
	{CPS_RX_REG_BC_COMMAND,           1,               0x0071},
	{CPS_RX_REG_BC_DATA0,             1,               0x0072},
	{CPS_RX_REG_BC_DATA1,             1,               0x0073},
	{CPS_RX_REG_BC_DATA2,             1,               0x0074},
	{CPS_RX_REG_BC_DATA3,             1,               0x0075},
	{CPS_RX_REG_BC_DATA4,             1,               0x0076},
	{CPS_RX_REG_BC_DATA5,             1,               0x0077},
	{CPS_RX_REG_BC_DATA6,             1,               0x0078},
	{CPS_RX_REG_OP_MODE,              1,               0x011D},
	{CPS_RX_REG_NEGO_POWER,           1,               0x011E},
};

cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
	// reg name            bytes number      reg address          /
	{CPS_TX_REG_PPP_HEADER,      1,               0x0070},
	{CPS_TX_REG_PPP_COMMAND,     1,               0x0071},
	{CPS_TX_REG_PPP_DATA0,       1,               0x0072},
	{CPS_TX_REG_PPP_DATA1,       1,               0x0073},
	{CPS_TX_REG_PPP_DATA2,       1,               0x0074},
	{CPS_TX_REG_PPP_DATA3,       1,               0x0075},
	{CPS_TX_REG_PPP_DATA4,       1,               0x0076},
	{CPS_TX_REG_PPP_DATA5,       1,               0x0077},
	{CPS_TX_REG_PPP_DATA6,       1,               0x0078},
	{CPS_TX_REG_BC_HEADER,       1,               0x0050},
	{CPS_TX_REG_BC_COMMAND,      1,               0x0051},
	{CPS_TX_REG_BC_DATA0,        1,               0x0052},
	{CPS_TX_REG_BC_DATA1,        1,               0x0053},
	{CPS_TX_REG_BC_DATA2,        1,               0x0054},
	{CPS_TX_REG_BC_DATA3,        1,               0x0055},
	{CPS_TX_REG_BC_DATA4,        1,               0x0056},
	{CPS_TX_REG_BC_DATA5,        1,               0x0057},
	{CPS_TX_REG_BC_DATA6,        1,               0x0058},
	{CPS_TX_REG_OCP_TH,          2,               0x0090},
	{CPS_TX_REG_OVP_TH,          2,               0x0094},
	{CPS_TX_REG_FOP_MIN,         1,               0x009E},
	{CPS_TX_REG_FOP_MAX,         1,               0x009F},
	{CPS_TX_REG_PING_FREQ,       1,               0x00A0},
	{CPS_TX_REG_PING_DUTY,       1,               0x00A1},
	{CPS_TX_REG_PING_OCP_TH,     2,               0x0096},
	{CPS_TX_REG_PING_TIME,       2,               0x00A2},
	{CPS_TX_REG_PING_INTERVAL,   2,               0x00A4},
	{CPS_TX_REG_FOD0_TH,         2,               0x00A8},
	{CPS_TX_REG_FUNC_EN,         1,               0x0030},
	{CPS_TX_REG_FOP_VAL,         2,               0x0108},
	{CPS_TX_REG_ADC_VIN,         2,               0x0100},
	{CPS_TX_REG_ADC_VRECT,       2,               0x0102},
	{CPS_TX_REG_ADC_IPA,         2,               0x0104},
	{CPS_TX_REG_CE_VAL,          1,               0x010D},
	{CPS_TX_REG_RP_VAL,          2,               0x010E},
	{CPS_TX_REG_EPT_RSN,         4,               0x0110},
	{CPS_TX_REG_ADC_DIE_TEMP,    2,               0x0106},
	{CPS_TX_REG_EPT_CODE,        1,               0x010B},
	{CPS_TX_REG_RX_POWER,        2,               0x0116},
	{CPS_TX_REG_TX_POWER,        2,               0x0118},
	{CPS_TX_REG_POWER_LOSS,      2,               0x011A},
};

//-------------------I2C APT start--------------------

static const struct regmap_config cps4041_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct regmap_config cps4041_regmap32_config = {
	.reg_bits = 32,
	.val_bits = 8,
};


static void wls_rx_start_timer(struct cps_wls_chrg_chip *info);
static void cps_rx_online_check(struct cps_wls_chrg_chip *chg);
static int cps_wls_set_status(int status);
static void cps_wls_current_select(int *icl, int *vbus, bool *cable_ready);
static void cps_init_charge_hardware(void);
static void cps_epp_current_select(int *icl, int *vbus);
static void cps_wls_tx_enable(bool en);
static bool cps_stop_epp_timeout(long ms);
static void cps_wls_notify_tx_chrgfull(void);
static int cps_get_bat_info(enum power_supply_property property);

int cps_wls_get_ldo_on(void);
int cps_wls_sysfs_notify(const char *attr);
static int cps_get_vbus(void);
static int factory_test_wls_en(void *input, bool en);

int cps_wls_reg_check(void)
{
	cps_reg_s *cps_reg = NULL;
	int ret = 0;
	int i = 0;

	for (i = 0; i < CPS_COMM_REG_MAX; i++) {
		cps_reg = (cps_reg_s*)(&cps_comm_reg[i]);
		if ((int)cps_reg->reg_name != i) {
			cps_wls_log(CPS_LOG_ERR, "cps_comm_reg Error: index=%d name=%d addr=0x%04x\n",
				i, (int)cps_reg->reg_name, (int)cps_reg->reg_addr);
			ret = -1;
		}
	}

	for (i = 0; i < CPS_RX_REG_MAX; i++) {
		cps_reg = (cps_reg_s*)(&cps_rx_reg[i]);
		if ((int)cps_reg->reg_name != i) {
			cps_wls_log(CPS_LOG_ERR, "cps_rx_reg Error: index=%d name=%d addr=0x%04x\n",
				i, (int)cps_reg->reg_name, (int)cps_reg->reg_addr);
			ret = -1;
		}
	}

	for (i = 0; i < CPS_TX_REG_MAX; i++) {
		cps_reg = (cps_reg_s*)(&cps_tx_reg[i]);
		if ((int)cps_reg->reg_name != i) {
			cps_wls_log(CPS_LOG_ERR, "cps_tx_reg Error: index=%d name=%d addr=0x%04x\n",
				i, (int)cps_reg->reg_name, (int)cps_reg->reg_addr);
			ret = -1;
		}
	}

	return ret;
}


static int cps_wls_l_write_reg(int reg, int value)
{
	int ret = -1;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_write(chip->regmap, reg, value);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_l_read_reg(int reg)
{
	int ret = -1;
	int value = 0;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, reg, &value);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
		return CPS_WLS_FAIL;
	}

	return value;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_write_reg(int reg, int value, int byte_len)
{
	int i = 0;
	int tmp = 0;

	for (i = 0; i < byte_len; i++) {
		tmp = (value >> (i*8)) & 0xff;
		if (cps_wls_l_write_reg((reg & 0xffff) + i, tmp) == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read_reg(int reg, int byte_len)
{
	int i = 0;
	int tmp = 0;
	int read_data = 0;

	for (i = 0; i < byte_len; i++) {
		tmp = cps_wls_l_read_reg((reg&0xffff) + i);
		if(tmp == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
		read_data |= (tmp << (8*i));
	}
#if CPS_REG_DEBUG
	cps_wls_log(CPS_LOG_ERR, "[%s] reg[0x%04X]=0x%X\n", __func__, reg, read_data);
#endif
	return read_data;
}

//*****************************for program************************

static int cps_wls_write_word(int addr, int value)
{
	int ret;
	u8 write_data[4];

	write_data[3] = (value >> 24) & 0xff;
	write_data[2] = (value >> 16) & 0xff;
	write_data[1] = (value >> 8) & 0xff;
	write_data[0] = value & 0xff;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap32, addr, write_data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c[%04X] write error!\n", __func__, addr);
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

static int cps_wls_read_word(int addr)
{
	int ret;
	u8 read_data[4] = {0x00};

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_read(chip->regmap32, addr, read_data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0){
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c[%04X] read error!\n", __func__, addr);
		return CPS_WLS_FAIL;
	}

	return *(int *)read_data;
}

static int cps_wls_program_sram(int addr, const unsigned char *data, int len)
{
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap32, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0){
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c[%04X] write error!\n", __func__, addr);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_program_cmd_send(int cmd)
{
	return cps_wls_write_word(ADDR_CMD, cmd);
}

static int cps_wls_program_wait_cmd_done(void)
{
	int res = 0;
	int cnt = 0;//ms

	while (1) {
		res = cps_wls_read_word(ADDR_FLAG);
		if (res == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;

		msleep(1);

		switch(res)
		{
			case RUNNING:
				break;
			case PASS:
				break;
			case FAIL:
				cps_wls_log(CPS_LOG_ERR, "---> FAIL : %x\n", res);
				return res;
				break;
			case ILLEGAL:
				cps_wls_log(CPS_LOG_ERR, "---> ILLEGAL : %x\n", res);
				return res;
				break;
			default :
				cps_wls_log(CPS_LOG_ERR, "---> ERROR-CODE : %x\n", res);
				return res;
				break;
		}

		if (res == PASS) {
			break;
		}

		/*3s over time*/
		if((cnt++) == 3000) {
			cps_wls_log(CPS_LOG_ERR, "--->[%s] CMD-OVERTIME\n", __func__);
			break;
		}
	}
	return res;
}

//get crc
uint16_t get_crc(u8 *buf, int len)
{
	int i,j;
	uint16_t crc_in = 0x0000;
	uint16_t crc_poly = 0x1021;

	for (i = 0; i < len; i++) {
		crc_in ^= (buf[i] << 8);
		for (j = 0; j < 8; j++) {
			if(crc_in & 0x8000)
				crc_in = (crc_in << 1) ^ crc_poly;
			else
				crc_in = crc_in << 1;
		}
	}

	return crc_in;
}

//****************************************************************
//-------------------I2C APT end--------------------

//-------------------CPS4041 system interface-------------------
static int cps_wls_comm_get(cps_comm_reg_e index)
{
	cps_reg_s *cps_reg = NULL;

	if (index < 0 || index >= CPS_COMM_REG_MAX) {
		return -1;
	}
	cps_reg = (cps_reg_s*)(&cps_comm_reg[index]);

	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_rx_get(cps_rx_reg_e index)
{
	int ret = -1;
	cps_reg_s *cps_reg = NULL;
	if (index < 0 || index >= CPS_RX_REG_MAX) {
		return -1;
	}
	cps_reg = (cps_reg_s*)(&cps_rx_reg[index]);
	ret = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);

	return ret;
}

static int cps_wls_tx_get(cps_tx_reg_e index)
{
	cps_reg_s *cps_reg = NULL;
	if (index < 0 || index >= CPS_TX_REG_MAX) {
		return -1;
	}
	cps_reg = (cps_reg_s*)(&cps_tx_reg[index]);

	return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

/*
static int cps_wls_rx_set(cps_rx_reg_e index, int value)
{
	cps_reg_s *cps_reg;
	if (index < 0 || index >= CPS_RX_REG_MAX) {
		return -1;
	}
	cps_reg = (cps_reg_s*)(&cps_rx_reg[index]);
	return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}*/

static int cps_wls_set_cmd(int value)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CMD]);
	return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static uint16_t cps_wls_get_cmd(void)
{
	return (uint16_t) cps_wls_comm_get(CPS_COMM_REG_CMD);
}

static int cps_wls_get_int_flag(void)
{
	return cps_wls_comm_get(CPS_COMM_REG_INT_FLAG);
}

static int cps_wls_set_int_clr(int value)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_CLR]);
	return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}
static int cps_wls_get_chip_id(void)
{
	return cps_wls_comm_get(CPS_COMM_REG_CHIP_ID);
}

static int cps_wls_get_sys_fw_version(void)
{
	return cps_wls_comm_get(CPS_COMM_REG_FW_VER);
}

static int cps_wls_get_sys_mode(void)
{
	return cps_wls_comm_get(CPS_COMM_REG_SYS_MODE);
}



//-------------------CPS4041 RX interface-------------------
#if 0
static int cps_wls_get_rx_ss_pkt_value(void)
{
	return cps_wls_rx_get(CPS_RX_REG_SS_VAL);
}
#endif

static int cps_wls_get_rx_ce_pkt_value(void)
{
	return cps_wls_rx_get(CPS_RX_REG_CE_VAL);
}

#if 0
static int cps_wls_get_rx_rp_pkt_value(void)
{
	return cps_wls_rx_get(CPS_RX_REG_RP_VAL);
}
#endif

static int cps_wls_get_rx_fop_value(void)
{
	return cps_wls_rx_get(CPS_RX_REG_FOP_VAL);
}

static int cps_wls_get_rx_ept_code(void)
{
	return cps_wls_rx_get(CPS_RX_REG_EPT_VAL);
}

static int cps_wls_get_rx_neg_power(void)
{
	return cps_wls_rx_get(CPS_RX_REG_NEGO_POWER);
}

static int cps_wls_get_rx_vrect(void)
{
	return cps_wls_rx_get(CPS_RX_REG_ADC_VRECT);
}

static int cps_wls_get_rx_irect(void)
{
	return cps_wls_rx_get(CPS_RX_REG_ADC_IRECT);
}

static int cps_wls_get_rx_iout(void)
{
	return cps_wls_rx_get(CPS_RX_REG_ADC_IRECT);
}

static int cps_wls_get_rx_vout(void)
{
	return cps_wls_rx_get(CPS_RX_REG_ADC_VOUT);
}

static int cps_wls_set_rx_fod_array_gain(uint32_t *fod_array, uint32_t fod_array_len)
{
	int status = CPS_WLS_SUCCESS;
	uint8_t i;
	if (fod_array_len != RX_FOD_GAIN_LEN)
		return -1;

	for(i = 0; i < RX_FOD_GAIN_LEN; i++)
	{
		status = cps_wls_write_reg((int)(RX_REG_FOD_C0_GAIN + i), fod_array[i], 1);
	}

	return status;
}

static int cps_get_sys_op_mode(Sys_Op_Mode *sys_mode_type)
{
	int status = CPS_WLS_SUCCESS;
	uint32_t temp = 0;
	uint8_t retries = 3;

	do {
		//msleep(10);//10ms
		retries--;
		temp = cps_wls_rx_get(CPS_RX_REG_OP_MODE);

		*sys_mode_type = (temp & CPS_MASK(7, 0));
		cps_wls_log(CPS_LOG_DEBG, "CPS_REG_Sys_Op_Mode = 0x%02x, 0x%02x", temp, *sys_mode_type);
		if(*sys_mode_type == Sys_Op_Mode_AC_Missing
				|| *sys_mode_type == Sys_Op_Mode_BPP
				|| *sys_mode_type == Sys_Op_Mode_EPP
				|| *sys_mode_type == Sys_Op_Mode_MOTO_WLC) {
			if (*sys_mode_type != Sys_Op_Mode_MOTO_WLC) {
				chip->qi_mode_type = *sys_mode_type;
			}
			break;
		}
		msleep(10);//10ms
	} while(retries);

	if (chip->folio_mode) {
		if (*sys_mode_type == Sys_Op_Mode_BPP) {
			cps_wls_set_rx_fod_array_gain(bpp_fod_array_w_folio, RX_FOD_GAIN_LEN);
		} else if (*sys_mode_type == Sys_Op_Mode_EPP) {
			cps_wls_set_rx_fod_array_gain(epp_fod_array_w_folio, RX_FOD_GAIN_LEN);
		}
	}

	return status;
}

static int cps_wls_set_fod_para(void)
{
	uint8_t i;

	const uint8_t FOP_CUR[7] =
	{
		// unit 10mA
		25 * 1,	// C0
		25 * 2,	// C1
		25 * 3,	// C2
		25 * 4,	// C3
		25 * 5,	// C4
		25 * 6,	// C5
		25 * 7,	// C6
	};

	const uint8_t FOP_GAIN_OFFSET[8 * 2] =
	{
		// gain(0.01)	 offset(40mW)
		// 5V
		54,				 6,			 // C0
		54,				 6,			 // C1
		54,				 6,			 // C2
		54,				 6,			 // C3
		54,				 6,			 // C4
		54,				 6,			 // C5
		54,				 6,			 // C6
		54,				 6,			 // C7
	};

	for(i = 0; i < 7; i++) {
		if(cps_wls_write_reg((int)(RX_REG_FOD_CUR_0 + i), FOP_CUR[i], 1) == CPS_WLS_FAIL)
		{
			return CPS_WLS_FAIL;
		}
	}

	for(i = 0; i < 16; i++) {
		if(cps_wls_write_reg((int)(RX_REG_FOD_C0_GAIN + i), FOP_GAIN_OFFSET[i], 1) == CPS_WLS_FAIL)
		{
			return CPS_WLS_FAIL;
		}
	}

	return CPS_WLS_SUCCESS;
}

static int cps_wls_get_rx_die_tmp(void)
{
	return cps_wls_rx_get(CPS_RX_REG_ADC_DIE_TMP);
}

static int cps_wls_get_rx_vout_set(void)
{
	return cps_wls_rx_get(CPS_RX_REG_VOUT_SET);
}

//-------------------CPS4041 TX interface-------------------
static int cps_wls_get_tx_i_in(void)
{
	return cps_wls_tx_get(CPS_TX_REG_ADC_IPA);//CPS_TX_REG_ADC_I_IN
}

static int cps_wls_get_tx_vin(void)
{
	return cps_wls_tx_get(CPS_TX_REG_ADC_VIN);
}

static int cps_wls_get_tx_vrect(void)
{
	return cps_wls_tx_get(CPS_TX_REG_ADC_VRECT);
}

/**
 * @brief Rp power smaller than RP0 threshold(0x0252),FOD ploss trigger threshold setting
 * @note
 * @param None
 * @retval
 */
static int cps_wls_set_tx_fod0_thresh(int value)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD0_TH]);
	return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_enable_tx_mode(void)
{
	uint16_t cmd;
	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_ENTER_TX_MODE;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_disable_tx_mode(void)
{
	uint16_t cmd;
	cmd = cps_wls_get_cmd();
	//cmd |= TX_CMD_EXIT_TX_MODE;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_send_fsk_packet(uint8_t *data, uint8_t data_len)
{
	uint16_t cmd;
	uint8_t i;
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PPP_HEADER]);

	for (i = 0; i < data_len; i++) {
		if (cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1) == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
	}

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_SEND_FSK;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
	int status = CPS_WLS_SUCCESS;
	uint32_t cmd = 0, i = 0;
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_PPP_HEADER]);

	moto_auth_printf_data("send_fsk_packet", data, data_len);
	for (i = 0; i < data_len; i++) {
		status = cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1);
		if (CPS_WLS_SUCCESS != status) {
			cps_wls_log(CPS_LOG_ERR, " cps wls write failed, addr 0x%02x, data 0x%02x", cps_reg->reg_addr + i, *(data + i));
			return status;
		}
	}

	cmd = cps_wls_get_cmd();

	cps_wls_log(CPS_LOG_ERR, " cps_wls_get_cmd %x\n",cmd);

	cmd |= RX_CMD_SEND_ASK;

	status = cps_wls_set_cmd(cmd);

	if(CPS_WLS_SUCCESS != status) {
		cps_wls_log(CPS_LOG_ERR, " cps TX_CMD_SEND_FSK failed");
	}

	return status;
}

uint8_t cps_wls_get_message_size(uint8_t header)
{
	uint8_t size = 0;

	if (header < 0x20) {
		size = 1;
	} else if (header < 0x80) {
		size = header / 16;
	} else if (header < 0xE0) {
		size = header / 8 - 8;
	} else {
		size = header / 4 - 36;
	}

	return size;
}

static int cps_wls_get_fsk_packet(uint8_t *data)
{
	int temp;
	uint8_t i;
	uint8_t data_len;
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_BC_HEADER]);

	/*get header*/
	temp = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if (temp != CPS_WLS_FAIL) {
		*data = temp;
		data_len = cps_wls_get_message_size(*data);
	} else {
		return CPS_WLS_FAIL;
	}

	cps_wls_log(CPS_LOG_ERR,"%s reg_addr=0x%04X data_len=%d\n",
			__func__, cps_reg->reg_addr, data_len);
	for (i = 0; i < data_len; i++) {
		temp = cps_wls_read_reg((int)(cps_reg->reg_addr + 1 + i), (int)cps_reg->reg_bytes_len);
		if(temp != CPS_WLS_FAIL) {
			*(data + 1 + i) = temp;
		} else {
			return CPS_WLS_FAIL;
		}
	}

	moto_auth_printf_data("get_fsk_packet", data+1, data_len);
	if (motoauth.wls_get_fsk_packet && chip && chip->moto_stand) {
		motoauth.wls_get_fsk_packet(data+1, data_len);
	}

	return CPS_WLS_SUCCESS;
}
#if 0
static int cps_wls_get_ask_packet(uint8_t *data)
{
	int temp;
	uint8_t i;
	uint8_t data_len;
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_BC_HEADER]);

	/*get header*/
	temp = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if(temp != CPS_WLS_FAIL)
	{
		*data = temp;
		data_len = cps_wls_get_message_size(*data);
	}
	else
	{
		return CPS_WLS_FAIL;
	}

	for(i = 0; i < data_len; i++)
	{
		temp = cps_wls_read_reg((int)(cps_reg->reg_addr + 1 + i), (int)cps_reg->reg_bytes_len);
		if(temp != CPS_WLS_FAIL)
		{
			*(data + 1 + i) = temp;
		}
		else
		{
			return CPS_WLS_FAIL;
		}
	}

	return CPS_WLS_SUCCESS;
}
#endif

static int cps_wls_get_tx_ept_rsn(void)
{
	return cps_wls_tx_get(CPS_TX_REG_EPT_RSN);
}

static int mmi_mux_wls_chg_chan(enum mmi_mux_channel channel, bool on)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s mmi_mux Couldn't get chg_psy\n", __func__);
		return CPS_WLS_FAIL;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	}

	cps_wls_log(CPS_LOG_DEBG,"mmi_mux open wls chg chan = %d on = %d\n", channel, on);

	if (info->algo.do_mux)
		info->algo.do_mux(info, channel, on);
	else
		cps_wls_log(CPS_LOG_ERR,"mmi_mux get info->algo.do_mux fail\n");

	return CPS_WLS_SUCCESS;
}
//------------------------------IRQ Handler-----------------------------------
static int cps_wls_set_int_enable(void)
{
	uint32_t int_en;
	cps_reg_s *cps_reg;

	int_en = 0xFFFFFFFF;
	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_EN]);

	if(CPS_WLS_FAIL == cps_wls_write_reg((int)cps_reg->reg_addr, int_en, (int)cps_reg->reg_bytes_len))	goto set_int_fail;
	return CPS_WLS_SUCCESS;

set_int_fail:
	return CPS_WLS_FAIL;
}

static void cps_bpp_icl_on(void)
{
	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;
	pr_info("%s\n", __func__);

	if (chip->factory_wls_en)
		return;
	cps_get_sys_op_mode(&mode_type);
	if (chip->rx_ldo_on && mode_type == Sys_Op_Mode_BPP)
	{
		chip->mode_type = mode_type;
		chip->wlc_tx_power = cps_wls_get_rx_neg_power() / 2;
		queue_delayed_work(chip->wls_wq, &chip->bpp_icl_work, msecs_to_jiffies(0));
	}
}

static void cps_epp_icl_on(void)
{
	int icl, vbus;
	int wls_icl = 0;

	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;
	pr_info("%s\n", __func__);

	if (chip->factory_wls_en)
		return;
	cps_get_sys_op_mode(&mode_type);
	if (mode_type == Sys_Op_Mode_EPP)
	{
		chip->mode_type = mode_type;
		chip->wlc_tx_power = cps_wls_get_rx_neg_power() / 2;
		cps_epp_current_select(&icl, &vbus);
		if (chip->thermal_state &&
			chip->thermal_wls_ccl > 0) {
			wls_icl = chip->thermal_wls_ccl / 1000;
			wls_icl *= 5000; /*follow moto_wlc*/
			wls_icl /= vbus;
			wls_icl *= 1000;
			if (icl > wls_icl) {
				icl = wls_icl;
				cps_wls_log(CPS_LOG_DEBG, "%s set icl %duA\n",
					__func__, wls_icl);
			}
		}
		if (chip->chg1_dev) {
			charger_dev_set_charging_current(chip->chg1_dev, 3150000);
			charger_dev_set_input_current(chip->chg1_dev, icl);
		} else {
			cps_init_charge_hardware();
			charger_dev_set_charging_current(chip->chg1_dev, 3150000);
			charger_dev_set_input_current(chip->chg1_dev, icl);
		}
		if (chip->enable_rod) {
			chip->rx_start_ktime = ktime_get_boottime();
			chip->rod_stop = false;
			queue_delayed_work(chip->wls_wq, &chip->offset_detect_work, msecs_to_jiffies(3000));
		}
	}
}

static int cps_wls_rx_irq_handler(int int_flag)
{
	int rc = 0;
	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;
	uint8_t data[8] = {0};
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	Sys_Op_Mode sys_mode_type;
	uint32_t temp = 0;
	cps_reg_s *cps_reg;
#endif
	if (int_flag & RX_INT_POWER_ON) {
		CPS_RX_MODE_ERR = false;
		CPS_RX_CHRG_FULL = false;
		chip->cable_ready_wait_count = 0;
		cps_rx_online_check(chip);
		/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
		/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
		if(chip->bootmode == 8 || chip->bootmode == 9)
			cps_rx_online_check(chip);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_POWER_ON");
	}
	if(int_flag & RX_INT_LDO_OFF) {
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_LDO_OFF");
	}
	if (int_flag & RX_INT_LDO_ON) {
		chip->rx_ldo_on = true;
		chip->rx_start_ktime = ktime_get_boottime();
		if (chip->wlc_status == WLC_DISCONNECTED) {
			cps_wls_set_status(WLC_CONNECTED);
		}
		cps_bpp_icl_on();
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_LDO_ON");
		queue_delayed_work(chip->wls_wq, &chip->dump_info_work, msecs_to_jiffies(2000));
	}
	if(int_flag & RX_INT_READY){
		data[0] = 0x38;
		data[1] = 0x3B;
		data[2] = 0x88;
		data[3] = 0x66;
		//data[0] = 0x38;
		// cps_wls_send_handshake_packet(data,4);
		chip->rx_int_ready = true;
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_READY");
	}
	//if(int_flag & RX_INT_FSK_ACK){}
	if(int_flag & RX_INT_FSK_TIMEOUT){}
	if(int_flag & RX_INT_FSK_PKT){
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_FSK_PKT");
		cps_wls_get_fsk_packet(data);
	}
	//if (int_flag & RX_INT_OVP)
	//{
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_OVP");
	//}
	if(int_flag & RX_INT_AC_LOSS){}
	//if(int_flag & RX_INT_OVP_TO){}
	//if(int_flag & RX_INT_AC_SHORT){}
	//if (int_flag & RX_INT_OTP)
	//{
	//	//int_type |= INT_OTP;
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_OTP");
	//}
	if(int_flag & RX_INT_SR_OCP){}
	//if (int_flag & RX_INT_OCP)
	//{
	//	CPS_RX_MODE_ERR = true;
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_OCP");
	//}
	//if (int_flag & RX_INT_HOCP) cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_HOCP");
	//if (int_flag & RX_INT_SCP) cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_SCP");
	//if(int_flag & RX_INT_SR_SW_R){}
	//if(int_flag & RX_INT_SR_SW_F){}
	//if(int_flag & RX_INT_HTP){}

	if (int_flag & RX_INT_NEGO_POWER_READY)
	{
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_NEGO_POWER_READY");
		cps_epp_icl_on();
	}

	//if (int_flag & RX_INT_PT) {
	//	chip->rx_vout_set = cps_wls_get_rx_vout_set();
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_PT rx_vout_set %dmV",
	//		chip->rx_vout_set);
	//}
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_OP_MODE]);
	temp = cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	sys_mode_type = (temp & CPS_MASK(7, 0));
	cps_wls_log(CPS_LOG_DEBG, "CPS_REG_Sys_Op_Mode = 0x%02x, 0x%02x", temp, sys_mode_type);
#endif
	if (int_flag & RX_INT_HS_OK) {
		chip->hs_st = HS_OK;
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_HS_OK");
		cps_get_sys_op_mode(&mode_type);
		if (mode_type == Sys_Op_Mode_MOTO_WLC) {
			chip->moto_stand = true;
			cps_wls_set_status(WLC_TX_TYPE_CHANGED);
		}
		//check_factory_mode(&factory_mode);
		if (chip->bootmode == KERNEL_POWER_OFF_CHARGING_BOOT ||
			chip->bootmode == LOW_POWER_OFF_CHARGING_BOOT) {
			queue_delayed_work(chip->wls_wq, &chip->light_fan_work, msecs_to_jiffies(0));
		}
		else
			motoauth_hs_ok_handler(mode_type);
	}
	if (int_flag & RX_INT_HS_FAIL) {
		chip->moto_stand = false;
		chip->hs_st = HS_FAIL;
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_HS_FAIL");
		if (chip->bootmode == KERNEL_POWER_OFF_CHARGING_BOOT ||
			chip->bootmode == LOW_POWER_OFF_CHARGING_BOOT)
			queue_delayed_work(chip->wls_wq, &chip->light_fan_work, msecs_to_jiffies(0));
	}
	if (int_flag & RX_INT_FC_FAIL) {
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	RX_INT_FC_FAIL");
//		WLC_FC_IRQ = WLS_IRQ_ACK_FAIL;
//		cps_wls_wlc_update_light_fan();
//		WLC_VOUT_BOOSTING = FALSE;
//		motoauth_event_notify(MOTOAUTH_EVENT_TX_SN);
	}
	return rc;
}

static int cps_wls_tx_irq_handler(int int_flag)
{
	int rc = 0;

	if (int_flag & TX_INT_PING) {
		if (chip->rx_connected) {
			chip->rx_connected = false;
			sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
		}
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_PING");
	}
	if (int_flag & TX_INT_SSP) {
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_SSP");
	}
	if (int_flag & TX_INT_IDP) {
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_IDP");
	}
	if (int_flag & TX_INT_CFGP) {
		if (!chip->rx_connected) {
			chip->rx_connected = true;
			sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
		}
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_CFGP");
	}
	if (int_flag & TX_INT_EPT) {
		if (chip->fw_uploading) {
			cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_EPT fw_uploading");
		} else {
			rc = cps_wls_get_tx_ept_rsn();
			cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_EPT RSN:0x%04X", rc);
			chip->tx_ept_flag = true;
			if (true == CPS_TX_MODE) {
				cps_wls_tx_enable(false);
			} else {
				chip->rx_connected = false;
				sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
				chip->tx_mode = false;
				sysfs_notify(&chip->dev->kobj, NULL, "tx_mode");
			}
		}
	}
	//if(int_flag & TX_INT_RPP_TO){
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_RPP_TO");
	//}
	//if(int_flag & TX_INT_CEP_TO){
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_CEP_TO");
	//}
	if(int_flag & TX_INT_AC_DET){
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_AC_DET");
	}
	//if(int_flag & TX_INT_INIT){
	//	cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_INIT");
	//	CPS_TX_IRQ = true;
	//}
	if(int_flag & TX_INT_ASK_PKT)
	{
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:	TX_INT_ASK_PKT");
#ifdef SMART_PEN_SUPPORT
		rc = cps_wls_get_ask_packet();
#endif
	}

	return rc;
}

static int cps_wls_rx_power_on(void);
#if 0
void cps_wls_rx_work_func_t(struct work_struct *work)
{
	struct cps_wls_chrg_chip *info = chip;
	info->rx_polling_ns = 500 * 1000 * 1000;
	cps_wls_log(CPS_LOG_DEBG, "[%s] into", __func__);
	wls_rx_start_timer(info);
	cps_wls_log(CPS_LOG_DEBG, "[%s] out", __func__);
}
#endif
static void cps_rx_online_check(struct cps_wls_chrg_chip *chg)
{
	bool wls_online = false;
	struct cps_wls_chrg_chip *chip = chg;

	wls_online = cps_wls_rx_power_on();
	if(!chip->wls_online && wls_online) {
		cps_wls_log(CPS_LOG_DEBG, "%s wls_online 1\n" , __func__);
		chip->wls_online = true;
		mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, true);
		power_supply_changed(chip->wl_psy);
	}
	if(chip->wls_online && !wls_online){
		cps_wls_log(CPS_LOG_DEBG, "%s wls_online 0\n" , __func__);
		chip->wls_online = false;
		mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, false);
		power_supply_changed(chip->wl_psy);
	}
}

#define WLS_ICL_INCREASE_STEP_mA 100 /*100mA*/
#define WLS_ICL_INCREASE_DELAY 200 /*200ms*/
#define WLS_BPP_ICL_MAX_mA 1000
#define WLS_BPP_ROD_THRESHOLD_CURRENT_MAX 800 /*mA*/
#define WLS_BPP_ROD_THRESHOLD_CURRENT_MIN 700 /*mA*/
#define WLS_BPP_ROD_DETECT_COUNT_MAX 3
#define WLS_ROD_STOP_BATERY_SOC 90
#define WLS_ROD_STOP_TIME (60*1000*1000*1000uL) /*60s*/

#define WLS_EPP_ROD_DETECT_COUNT_MAX 3
#define WLS_EPP_ROD_THRESHOLD_12V 10500 /*mV*/
#define WLS_EPP_ROD_THRESHOLD_10V 9000 /*mV*/

static void cps_bpp_mode_icl_work(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chg = chip;
	int wls_icl = 0;
	int wls_icl_max = WLS_BPP_ICL_MAX_mA;
	int wls_current_now = 0;
	int wls_voltage_now = 0;
	int retry = 2;
	int i = 0;

	wls_icl = WLS_ICL_INCREASE_STEP_mA;
	chg->bpp_icl_done = false;
	if (chip && chip->wls_input_curr_max > 0) {
		wls_icl_max = chip->wls_input_curr_max;
		if (wls_icl_max > WLS_BPP_ICL_MAX_mA) {
			wls_icl_max = WLS_BPP_ICL_MAX_mA;
		}
	}

	if (chip && chip->thermal_state &&
		chip->thermal_wls_ccl > 0 &&
		wls_icl_max > chip->thermal_wls_ccl / 1000) {
		wls_icl_max = chip->thermal_wls_ccl / 1000;
		cps_wls_log(CPS_LOG_DEBG, "%s set wls_icl_max=thermal_wls_ccl %duA\n",
					__func__, chip->thermal_wls_ccl);
	}
	for (i = 0; i < retry; i++) {
		while (wls_icl <= wls_icl_max) {
			if (!chg->wls_online)
				break;
			if (!chip->chg1_dev)
				cps_init_charge_hardware();
			if (chip->chg1_dev)
				charger_dev_set_input_current(chip->chg1_dev, wls_icl * 1000);
			msleep(WLS_ICL_INCREASE_DELAY);
			wls_current_now = cps_wls_get_rx_iout();
			wls_voltage_now = cps_wls_get_rx_vout();

			cps_wls_log(CPS_LOG_DEBG, "%s set icl=%dmA I=%dmA V=%dmV\n",
					__func__, wls_icl, wls_current_now, wls_voltage_now);
			wls_icl += WLS_ICL_INCREASE_STEP_mA;
		}
		if (wls_current_now < WLS_BPP_ROD_THRESHOLD_CURRENT_MAX) {
			wls_icl = wls_current_now / WLS_ICL_INCREASE_STEP_mA * WLS_ICL_INCREASE_STEP_mA;
		} else {
			break;
		}
	}

	chg->bpp_icl_done = true;

	if (chip->enable_rod) {
		chip->rod_stop = false;
		queue_delayed_work(chip->wls_wq,
			&chip->offset_detect_work,
			msecs_to_jiffies(wls_current_now >= WLS_BPP_ROD_THRESHOLD_CURRENT_MAX ? 5000 : 0));
	}
}

static void cps_wls_notify_thermal_input_current_limit(int thermal_icl)
{
	if (!chip)
		return;
	if (thermal_icl != -1) {
		chip->thermal_icl = thermal_icl / 1000;
		cps_wls_log(CPS_LOG_DEBG, "%s thermal_icl %d ma\n", __func__, chip->thermal_icl);
	} else {
		chip->thermal_icl = -1;
	}
}

static void cps_wls_notify_cur_state(int state, int wls_ccl)
{
	if (!chip)
		return;
	chip->thermal_state = state;
	chip->thermal_wls_ccl = wls_ccl;
	cps_wls_log(CPS_LOG_DEBG, "%s state:%d ccl:%duA\n",
		__func__, chip->thermal_state, chip->thermal_wls_ccl);
}

static bool cps_wls_check_iout(int target_current, int current_now)
{
	bool rt = false;
	int batt_soc = 0;
	bool skip_rod = false;

	if (!chip)
		return rt;

	if (chip->thermal_icl != -1 &&
		chip->thermal_icl < chip->MaxI) {
		skip_rod = true;
	} else if (chip->rod_stop_battery_soc > 0 &&
			chip->rod_stop_battery_soc < 100) {
		batt_soc = cps_get_bat_info(POWER_SUPPLY_PROP_CAPACITY);
		skip_rod = (batt_soc > chip->rod_stop_battery_soc) ? true : false;
	} else {
		skip_rod = false;
	}

	cps_wls_log(CPS_LOG_DEBG, "[%s] skip:%d now:%dmA target:%dmA\n",
				__func__, skip_rod, current_now, target_current);

	return !skip_rod && (current_now < target_current);
}

#define OFFSET_DETECT_TIME_DELAY_DEFAULT_MS 1000
static void cps_offset_detect_work(struct work_struct *work)
{
	Sys_Op_Mode wls_mode = Sys_Op_Mode_INVALID;
	static int work_timedelay = 1000;
	static int wls_current = 0;
	int current_now = 0;

	if (!chip)
		return;

	if (!chip->rx_ldo_on || chip->rod_stop) {
		chip->rx_offset_detect_count = 0;
		chip->rx_offset = false;
		cps_wls_log(CPS_LOG_DEBG, "[%s] rx_ldo_on:%d,rod_stop:%d",
			__func__, chip->rx_ldo_on, chip->rod_stop);
		return;
	}

	if (chip->mode_type != Sys_Op_Mode_MOTO_WLC) {
		wls_mode = chip->mode_type;
	} else {
		wls_mode = chip->qi_mode_type;
	}

	if ((ktime_get_boottime() - chip->rx_start_ktime >= WLS_ROD_STOP_TIME) &&
		(chip->wlc_status != WLC_ERR_LOWER_EFFICIENCY)) {
		cps_wls_log(CPS_LOG_DEBG, "[%s] rx offset detect stop,wls status:%d\n", __func__, chip->wlc_status);
		chip->rod_stop = true;
		chip->rx_offset_detect_count = 0;
		chip->rx_offset = false;
		return;
	}

	current_now = cps_wls_get_rx_iout();
	if (wls_mode == Sys_Op_Mode_BPP) {
		if (!chip->rx_offset) {
			if (chip->thermal_state != 0 &&
				chip->thermal_wls_ccl < chip->MaxI) {
				chip->rod_stop = true;
				chip->rx_offset_detect_count = 0;
				return;
			}
			if (cps_wls_check_iout(WLS_BPP_ROD_THRESHOLD_CURRENT_MIN, current_now)) {
				chip->rx_offset_detect_count ++;
			} else {
				chip->rx_offset_detect_count = 0;
			}
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS;
			if (chip->rx_offset_detect_count >= WLS_BPP_ROD_DETECT_COUNT_MAX) {
				chip->rx_offset = true;
				chip->rx_offset_detect_count = 0;
				cps_wls_log(CPS_LOG_DEBG, "[%s] offset true\n", __func__);
				if (chip->wlc_status != WLC_DISCONNECTED)
					cps_wls_set_status(WLC_ERR_LOWER_EFFICIENCY);
				if (chip->chg1_dev) {
					wls_current = current_now / WLS_ICL_INCREASE_STEP_mA * WLS_ICL_INCREASE_STEP_mA;
					if (wls_current < WLS_ICL_INCREASE_STEP_mA)
						wls_current = WLS_ICL_INCREASE_STEP_mA;
					charger_dev_set_input_current(chip->chg1_dev, wls_current * 1000);
					work_timedelay = WLS_ICL_INCREASE_DELAY;
				}
			}
		} else {
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS / 2;
			if (wls_current < WLS_BPP_ICL_MAX_mA) {
				if (chip->chg1_dev) {
					cps_wls_log(CPS_LOG_DEBG, "[%s] wls_current = %dmA current_now=%dmA\n",
						 __func__, wls_current, current_now);
					wls_current = wls_current + WLS_ICL_INCREASE_STEP_mA;
					charger_dev_set_input_current(chip->chg1_dev, wls_current * 1000);
					work_timedelay = WLS_ICL_INCREASE_DELAY;
				}
			} else {
				if (!cps_wls_check_iout(WLS_BPP_ROD_THRESHOLD_CURRENT_MAX, current_now)) {
					chip->rx_offset_detect_count ++;
				} else {
					chip->rx_offset_detect_count = 0;
				}
				if (chip->rx_offset_detect_count >= WLS_BPP_ROD_DETECT_COUNT_MAX) {
					chip->rx_offset = false;
					cps_wls_log(CPS_LOG_DEBG, "[%s] offset exit\n", __func__);
					chip->rx_offset_detect_count = 0;
					if (chip->wlc_status != WLC_DISCONNECTED)
						cps_wls_set_status(WLC_CHGING);
					chip->rod_stop = true;
				}
			}
		}
	} else if(wls_mode == Sys_Op_Mode_EPP) {
		/*For EPP*/
		chip->rx_vout = cps_wls_get_rx_vout();
		chip->rx_vout_set = cps_wls_get_rx_vout_set();
		if (chip->rx_vout_set == 12000)
			chip->rx_vout_threshold = WLS_EPP_ROD_THRESHOLD_12V;
		else if (chip->rx_vout_set == 10000)
			chip->rx_vout_threshold = WLS_EPP_ROD_THRESHOLD_10V;
		else
			chip->rx_vout_threshold = chip->MaxV * 80 / 100;

		cps_wls_log(CPS_LOG_DEBG, "[%s] vout:%dmV iout:%dmA vout_threshold:%dmV MaxI:%dmA\n", __func__,
				chip->rx_vout, current_now, chip->rx_vout_threshold, chip->MaxI);
		if (!chip->rx_offset) {
			if (chip->rx_vout < chip->rx_vout_threshold) {
				chip->rx_offset_detect_count ++;
			} else {
				chip->rx_offset_detect_count = 0;
			}
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS;
			if (chip->rx_offset_detect_count >= WLS_EPP_ROD_DETECT_COUNT_MAX) {
				chip->rx_offset = true;
				chip->rx_offset_detect_count = 0;
				cps_wls_log(CPS_LOG_DEBG, "[%s] EPP offset true\n", __func__);
				if (chip->wlc_status != WLC_DISCONNECTED)
					cps_wls_set_status(WLC_ERR_LOWER_EFFICIENCY);
				work_timedelay = WLS_ICL_INCREASE_DELAY;
			}
		} else {
			work_timedelay = OFFSET_DETECT_TIME_DELAY_DEFAULT_MS / 2;
			if (chip->rx_vout > chip->rx_vout_threshold + 500) {
				chip->rx_offset_detect_count ++;
			} else {
				chip->rx_offset_detect_count = 0;
			}
			if (chip->rx_offset_detect_count >= WLS_EPP_ROD_DETECT_COUNT_MAX) {
				chip->rx_offset = false;
				cps_wls_log(CPS_LOG_DEBG, "[%s] EPP offset exit\n", __func__);
				chip->rx_offset_detect_count = 0;
				if (chip->wlc_status != WLC_DISCONNECTED)
					cps_wls_set_status(WLC_CHGING);
				chip->rod_stop = true;
			}
		}
	}

	if (chip->enable_rod) {
		queue_delayed_work(chip->wls_wq, &chip->offset_detect_work, msecs_to_jiffies(work_timedelay));
	} else if (chip->rx_offset) {
		chip->rx_offset_detect_count = 0;
		chip->rx_offset = false;
	}
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
	int int_flag = 0;
	int int_clr = 0;
	cps_wls_log(CPS_LOG_DEBG, "[%s] IRQ triggered\n", __func__);
	mutex_lock(&chip->irq_lock);
	cps_wls_set_int_enable();
	chip->chip_state = true;

	int_flag = cps_wls_get_int_flag();
	cps_wls_log(CPS_LOG_DEBG, ">>>>>int_flag = %x\n", int_flag);
	if(int_flag == CPS_WLS_FAIL)
	{
		cps_wls_log(CPS_LOG_ERR, "[%s] read wls irq reg failed\n", __func__);
		mutex_unlock(&chip->irq_lock);
		return IRQ_HANDLED;
	}

	int_clr = int_flag;
	cps_wls_set_int_clr(int_flag);
	mutex_unlock(&chip->irq_lock);
	if (cps_wls_get_sys_mode() == SYS_MODE_RX) {
		cps_wls_rx_irq_handler(int_flag);
	} else {
		cps_wls_tx_irq_handler(int_flag);
	}

	return IRQ_HANDLED;
}

static int cps_wls_mode_select(char *str, bool mode)
{
	struct cps_wls_chrg_chip *chg = chip;
	int rt = CPS_WLS_FAIL;

	if (chg && gpio_is_valid(chg->wls_mode_select)) {
		gpio_set_value(chg->wls_mode_select, mode);
		rt = CPS_WLS_SUCCESS;
	}

	cps_wls_log(CPS_LOG_DEBG, "[%s] cps_wls_mode_select mode:%d rt:%d\n", str, mode, rt);

	return rt;
}

static irqreturn_t wls_det_irq_handler(int irq, void *dev_id)
{
	struct cps_wls_chrg_chip *chip = dev_id;
	int tx_detected = gpio_get_value(chip->wls_det_int);

	if (tx_detected) {
		cps_wls_log(CPS_LOG_DEBG, "Detected an attach event.\n");
		if (chip->stop_epp_flag) {
			chip->stop_epp_flag = false;
			cps_wls_log(CPS_LOG_DEBG, "wls_det_irq_handler stop_epp_flag false.\n");
		}
	} else {
		cps_wls_log(CPS_LOG_DEBG, "mmi_mux Detected a detach event.\n");
		chip->rx_int_ready = false;
		chip->bpp_icl_done = false;

		if (!chip->stop_epp_flag && !chip->mode_select_force && !chip->factory_wls_en)
			cps_wls_mode_select("wls_det_irq_handler", true);

		if (chip->rx_ldo_on) {
			//chip->wls_online = false;
			cps_wls_set_status(WLC_DISCONNECTED);
			cps_rx_online_check(chip);
			chip->rx_ldo_on = false;
			chip->rx_offset_detect_count = 0;
			chip->rx_offset = false;
			chip->rx_vout_set = 0;
			chip->hs_st = HS_UNKONWN;
			//if (chip->factory_wls_en == true) {
			//	chip->factory_wls_en = false;
			//	mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, false);
			//}
			motoauth_disconnect(&motoauth);
			//mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, false);
			//power_supply_changed(chip->wl_psy);
			//mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, false);
		}
	}

	return IRQ_HANDLED;
}
static enum power_supply_property cps_wls_chrg_props[] = {
	// POWER_SUPPLY_PROP_CURRENT_MAX,
	//POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	//POWER_SUPPLY_PROP_VOUT_NOW,
	//POWER_SUPPLY_PROP_VRECT,
	//POWER_SUPPLY_PROP_IRECT,
	//POWER_SUPPLY_PROP_PROTOCOL,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static int cps_wls_chrg_property_is_writeable(struct power_supply *psy,
												enum power_supply_property psp)
{
	return 0;
}

static int cps_wls_chrg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch(psp){
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->wls_online;
			cps_wls_log(CPS_LOG_DEBG, "%s wls_online=%d\n", __func__, chip->wls_online);
			if (!chip->chip_state) {
				val->intval = chip->chip_state;
			}
			if (chip->stop_epp_flag && !cps_stop_epp_timeout(5000)) {
				val->intval = true;
			}
			break;

		case POWER_SUPPLY_PROP_TYPE:
			val->intval = chip->wl_psd.type;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = chip->MaxV * 1000;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			if (chip->rx_ldo_on || CPS_TX_MODE)
				val->intval = cps_wls_get_rx_vout() * 1000;
			else
				val->intval = -1;
			break;

		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = chip->MaxI * 1000;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (chip->rx_ldo_on || CPS_TX_MODE)
				val->intval = cps_wls_get_rx_iout() * 1000;
			else
				val->intval = -1;
			break;
		 case POWER_SUPPLY_PROP_POWER_NOW:
			if (chip->rx_ldo_on || CPS_TX_MODE)
				val->intval = cps_wls_get_rx_neg_power() / 2;
			else
				val->intval = -1;
			break;
		default:
			return -EINVAL;
			break;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s] psp = %d val = %d.\n", __func__, psp,val->intval);
	return 0;
}

static int cps_wls_chrg_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	switch(psp){
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
			chip->chip_state = val->intval;
			power_supply_changed(chip->wl_psy);
			break;
		default:
			return -EINVAL;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s] psp = %d val = %d.\n", __func__, psp,val->intval);
	return 0;
}

static void cps_wls_charger_external_power_changed(struct power_supply *psy)
{
	;
}

static int cps_get_fw_revision(uint32_t* fw_revision)
{
	uint32_t fw_major_revision = 0;
	uint32_t fw_minor_revision = 0;
	int fw_version = 0;

	fw_version = cps_wls_get_sys_fw_version();

	if (CPS_WLS_FAIL != fw_version) {
		fw_minor_revision = (fw_version) & 0xFF;
		fw_major_revision = ((fw_version) >> 8) & 0xFF;
		fw_version = (fw_major_revision << 16) | (fw_minor_revision);
		*fw_revision = fw_version;
		chip->wls_fw_version = fw_version;
		cps_wls_log(CPS_LOG_ERR, "%s 0x%x, minor 0x%x, major 0x%x", __func__,
				*fw_revision, fw_minor_revision, fw_major_revision);
		return CPS_WLS_SUCCESS;
	} else {
		*fw_revision = 0;
		cps_wls_log(CPS_LOG_ERR, "%s failed, fw_version=0x%X", __func__, fw_version);
	}

	return CPS_WLS_FAIL;
}

static void cps_wls_pm_set_awake(int awake)
{
	if(!chip->cps_wls_wake_lock->active && awake) {
		__pm_stay_awake(chip->cps_wls_wake_lock);
	} else if(chip->cps_wls_wake_lock->active && !awake) {
		__pm_relax(chip->cps_wls_wake_lock);
	}
}

static void cps_wls_set_boost(bool val)
{
	/* Assume if we turned the boost on we want to stay awake */
	if(val) {
		cps_wls_pm_set_awake(1);
	} else {
		cps_wls_pm_set_awake(0);
	}
}

#ifdef CONFIG_MOTO_CHANNEL_SWITCH

static bool usb_online(void)
{
	union power_supply_propval prop;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n", __func__);
		prop.intval = 0;
	} else {
		power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		cps_wls_log(CPS_LOG_ERR,"%s online:%d\n", __func__, prop.intval);
	}

	return prop.intval;
}

static bool cps_wls_query_typec_attached_state(void)
{
	struct tcpc_device *tcpc_dev;
	uint8_t attached_type = 0;

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		dev_err(chip->dev, "get tcpc device fail\n");
		return false;
	}

	attached_type=tcpc_dev->typec_attach_new;
	dev_info(chip->dev, "get typec attached type %x !\n", attached_type);
	if(attached_type == TYPEC_ATTACHED_SRC)
		return true;
	else
		return false;
}
#endif /* CONFIG_MOTO_CHANNEL_SWITCH */

static bool cps_wls_fw_set_boost(bool val)
{
	int ret = 0;
	int vbus = 0;
	struct charger_device *chg_psy = NULL;

	chg_psy = get_charger_by_name("primary_chg");
	if (chg_psy) {
			cps_wls_log(CPS_LOG_ERR, "%s get chg_psy\n", __func__);
	} else {
		cps_wls_log(CPS_LOG_ERR, "%s Couldn't get chg_psy\n", __func__);
		return false;
	}

	mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FW_UPDATE, !!val);//MMI_MUX_CHANNEL_WLC_OTG

	ret = charger_dev_enable_otg(chg_psy, !!val);
	if(ret < 0){
		cps_wls_log(CPS_LOG_ERR, "%s set otg fail\n", __func__);
		return false;
	}

	msleep(50);
	vbus = cps_get_vbus();
	if (val && vbus < VBUS_VALID_MV) {
		cps_wls_log(CPS_LOG_ERR, "%s enable otg fail\n", __func__);
		return false;
	} else if(!val && vbus >= VBUS_VALID_MV) {
		cps_wls_log(CPS_LOG_ERR, "%s disable otg fail\n", __func__);
	}

	if(val) {
		cps_wls_pm_set_awake(1);
	} else {
		cps_wls_pm_set_awake(0);
	}

	return true;
}

static int cps_get_bat_info(enum power_supply_property property)
{
	union power_supply_propval prop;
	int ret = CPS_WLS_FAIL;

	if (!chip)
		return ret;

	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy == NULL || IS_ERR(chip->batt_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chip->batt_psy\n", __func__);
	} else {
		power_supply_get_property(chip->batt_psy, property, &prop);
		cps_wls_log(CPS_LOG_DEBG,"%s battery prop:%d value:%d\n",
				__func__, property, prop.intval);
		ret = prop.intval;
	}
	return ret;
}

#if 0
static int cps_get_bat_soc(void)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s mmi_mux Couldn't get chg_psy\n", __func__);
		return CPS_WLS_FAIL;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	}
	return get_uisoc(info);
}

static bool usb_online(void)
{
	union power_supply_propval prop;
	struct power_supply *chg_psy = NULL;
	int ret = 0;

	chg_psy = devm_power_supply_get_by_phandle(chip->dev, "charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n", __func__);
		prop.intval = 0;
	} else {
		power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		cps_wls_log(CPS_LOG_ERR,"%s online:%d\n", __func__, prop.intval);
		return prop.intval;
	}
	return ret;
}
static void wireless_chip_reset(void)
{
	struct chg_alg_device *alg;

	alg = get_chg_alg_by_name("wlc");
	if (NULL != alg) {
		chg_alg_set_prop(alg, ALG_WLC_STATE, false);
		msleep(100);
		chg_alg_set_prop(alg, ALG_WLC_STATE, true);
	}
	return;
}
#endif

static void wake_up_rx_check_thread(struct cps_wls_chrg_chip *info);
#define CPS_FW_MAJOR_VER_OFFSET 0xc4
#define CPS_FW_MINOR_VER_OFFSET 0xc5
static int wireless_fw_update(bool force)
{
	int buf0_flag = 0, buf1_flag = 0;
	u32 version = 0;
	u16 maj_ver = 0;
	u16 min_ver = 0;
	u8 *firmware_buf = NULL;
	int result;
	int rc,i;
	u32 fw_revision = 0;
	const struct firmware *fw = NULL;
	int cfg_buf_size = 0;
	int addr,ret = CPS_WLS_SUCCESS;
	bool boost_enable = false;
	int sys_mode = 0x00;

	if (cps_get_bat_info(POWER_SUPPLY_PROP_CAPACITY) < 10 && !force) {
		cps_wls_log(CPS_LOG_ERR,
			"Wireless fw update failed. Battery SOC should be at least 10%%\n");
		return CPS_WLS_FAIL;
	}

	sys_mode = cps_wls_get_sys_mode();
	if (chip->rx_ldo_on || sys_mode == SYS_MODE_RX) {
		cps_wls_log(CPS_LOG_ERR,"%s skip fw update when in wireles charging\n", __func__);
		return CPS_WLS_FAIL;
	}

	maj_ver = 0;
	min_ver = 0;
	fw = NULL;

	firmware_buf = kzalloc(FIRMWARE_SIZE_MAX, GFP_KERNEL); // 24K buffer
	if (firmware_buf == NULL) {
		cps_wls_log(CPS_LOG_ERR,"%s kzalloc firmware_buf failed\n", __func__);
		return CPS_WLS_FAIL;
	}

	rc = firmware_request_nowarn(&fw, chip->wls_fw_name, chip->dev);
	if (rc) {
		cps_wls_log(CPS_LOG_ERR,"Couldn't get firmware rc=%d\n", rc);
		ret = CPS_WLS_FAIL;
		goto update_exit;
	}

	maj_ver = be16_to_cpu(*(__le16 *)(fw->data + CPS_FW_MAJOR_VER_OFFSET));
	maj_ver = maj_ver >> 8;
	min_ver = be16_to_cpu(*(__le16 *)(fw->data + CPS_FW_MINOR_VER_OFFSET));
	min_ver = min_ver >> 8;
	cps_wls_log(CPS_LOG_DEBG,"maj_var %#x, min_ver %#x\n", maj_ver, min_ver);
	version = maj_ver << 16 | min_ver;

	cps_wls_log(CPS_LOG_DEBG,"FW size: %zu version: %#x force update: %d\n", fw->size, version, force);

	cps_wls_log(CPS_LOG_ERR,"%s cps_get_vbus = %d\n", __func__, cps_get_vbus());

	chip->chip_id = cps_wls_get_chip_id();
	if (chip->chip_id != CPS_WLS_FAIL) {
		boost_enable = false; //Wired charger plugin or OTG output.
	} else {
		boost_enable = true;
	}

	CPS_TX_MODE = true;
	chip->fw_uploading = true;

	if (boost_enable) {
		cps_wls_fw_set_boost(true);
		msleep(100);//100ms
	}

	chip->chip_id = cps_wls_get_chip_id();

	if (cps_get_vbus() < VBUS_VALID_MV || chip->chip_id == CPS_WLS_FAIL) {
		cps_wls_log(CPS_LOG_ERR,
			"Wireless fw update failed. Can't boost for CHIP,chip_id=0x%X\n", chip->chip_id);
		ret = CPS_WLS_FAIL;
		goto update_exit;
	}

	//recheck fw version
	result = cps_get_fw_revision(&fw_revision);
	if (!force && version == fw_revision) {
		cps_wls_log(CPS_LOG_DEBG,"%s bin version %x same as fw version %x,not need update fw\n", __func__,version,fw_revision);
		ret = CPS_WLS_SUCCESS;
		goto update_exit;
	}

	//CPS4041_BL
	//bootloader_buf = CPS4041_BOOTLOADER;
	if(CPS_WLS_FAIL == cps_wls_write_word(0xFFFFFF00, 0x0000000E)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;/*enable 32bit i2c*/
	}
	if(CPS_WLS_FAIL == cps_wls_write_word(0x4000E75C, 0x00001250)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;//*write password*/
	}
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40040010, 0x00000006)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;/*reset and halt mcu*/
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM HEX!\n", __func__);
	if(CPS_WLS_FAIL == cps_wls_program_sram(0x20000000, CPS4041_BOOTLOADER, BOOTLOADER_SIZE_MAX)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;//program sram
	}
	if(CPS_WLS_FAIL == cps_wls_write_word(0x400400A0, 0x000000FF)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;//remap enable
	}
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40040010, 0x00008003)) {
		ret = CPS_WLS_FAIL;
		goto update_exit;/*triming load function is disabled and run mcu*/
	}
	msleep(10);

	if(CPS_WLS_FAIL == cps_wls_write_word(0xFFFFFF00, 0x0000000E)) {
		ret = CPS_WLS_FAIL;
		goto update_exit; /*enable 32bit i2c*/
	}
	msleep(10);

	// cali bootloader code
	cps_wls_program_cmd_send(CACL_CRC_TEST);
	result = cps_wls_program_wait_cmd_done();

	if(result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader crc fail\n", __func__);
		ret = CPS_WLS_FAIL;
		goto update_exit;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load bootloader successful\n", __func__);

	// LOAD firmware to MTP
	memset(firmware_buf, 0, FIRMWARE_SIZE_MAX);
	memcpy(firmware_buf,fw->data,fw->size);
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load fw size %x\n", __func__,(int)fw->size);
	buf0_flag = 0;
	buf1_flag = 0;
	cfg_buf_size = 256;
	addr = 0;
	ret = CPS_WLS_FAIL;
	cps_wls_write_word(ADDR_BUF_SIZE, cfg_buf_size);

	result = cps_wls_program_wait_cmd_done();
	if(result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]	---> ERASE FAIL\n", __func__);
		goto update_exit;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s]	---> ERASE SUCCESS\n", __func__);

	for(i = 0; i < FIRMWARE_SIZE_MAX / 4 / cfg_buf_size; i++)
	{

		if (buf0_flag == 0) {
			cps_wls_program_sram(ADDR_BUFFER0, firmware_buf + addr, cfg_buf_size * 4);
			addr = addr + cfg_buf_size * 4;

			if(buf1_flag == 1) {
				result = cps_wls_program_wait_cmd_done();
				if(result != PASS) {
					pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
					goto update_exit;
				}
				buf1_flag = 0;
			}
			cps_wls_program_cmd_send(PGM_BUFFER0);
			buf0_flag = 1;
			continue;
		}

		if (buf1_flag == 0) {
			cps_wls_program_sram(ADDR_BUFFER1, firmware_buf + addr, cfg_buf_size * 4);
			addr = addr + cfg_buf_size * 4;

			if (buf0_flag == 1) {
				result = cps_wls_program_wait_cmd_done();
				if(result != PASS) {
					pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
					goto update_exit;
				}
				buf0_flag = 0;
			}
			cps_wls_program_cmd_send(PGM_BUFFER1);
			buf1_flag = 1;
			continue;
		}
	}

	if (buf0_flag == 1) {
		result = cps_wls_program_wait_cmd_done();
		if (result != PASS) {
			pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
			goto update_exit;
		}
		buf0_flag = 0;
	}

	if (buf1_flag == 1) {
		result = cps_wls_program_wait_cmd_done();
		if (result != PASS) {
			pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
			goto update_exit;
		}
		buf1_flag = 0;
	}
	pr_info("%s: ---> LOAD APP HEX SUCCESSFUL\n", __func__);

	//Step4, check app CRC
	cps_wls_program_cmd_send(CACL_CRC_APP);
	result = cps_wls_program_wait_cmd_done();

	if (result != PASS) {
		pr_err("%s: ---> APP CRC FAIL\n", __func__);
		goto update_exit;
	}
	pr_info("%s: ---> CHERK APP CRC SUCCESSFUL\n", __func__);

	//Step5, write mcu start flag
	//cps_wls_program_cmd_send(PGM_WR_FLAG);
	//result = cps_wls_program_wait_cmd_done();
	//if (result != PASS) {
	//	pr_err("%s:---> WRITE MCU START FLAG FAIL\n", __func__);
	//	goto update_fail;
	//}

	pr_info("%s: ---> WRITE MCU START FLAG SUCCESSFUL\n", __func__);
	cps_wls_write_word(0x40040010, 0x00000008); /*reset all system*/
	msleep(200);
	cps_wls_write_word(0xFFFFFF00, 0x00000000); /*i2c 32bit mode disable*/

	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- Program successful reg[0x0000]=0x%X\n", __func__ , cps_wls_read_word(0x0000));

	result = cps_get_fw_revision(&fw_revision);

	if (version == fw_revision) {
		cps_wls_log(CPS_LOG_DEBG, "%s update fw 0x%X successful \n", __func__, version);
		ret = CPS_WLS_SUCCESS;
	} else {
		cps_wls_log(CPS_LOG_DEBG, "%s update fw 0x%X failed,fw_revision 0x%X\n", __func__, version, fw_revision);
		ret = CPS_WLS_FAIL;
	}

update_exit:
	//release resources
	if (boost_enable)
		cps_wls_fw_set_boost(false);//disable power, after FW updating, need a power reset
	msleep(20);//20ms
	if (firmware_buf != NULL)
		kfree(firmware_buf);
	if (fw != NULL)
		release_firmware(fw);

	CPS_TX_MODE = false;
	chip->fw_uploading = false;

	if (ret == CPS_WLS_FAIL)
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);

	return ret;
}

static void cps_firmware_update_work(struct work_struct *work)
{
	cps_wls_log(CPS_LOG_ERR, "[%s]\n", __func__);
	wireless_fw_update(false);
}

//-----------------------------reg addr----------------------------------
static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "reg addr 0x%08x\n", chip->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	chip->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

//-----------------------------reg data----------------------------------
static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	chip->reg_data = cps_wls_read_reg(chip->reg_addr, 4);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", chip->reg_addr, chip->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	chip->reg_data = tmp;
	cps_wls_write_reg(chip->reg_addr, chip->reg_data, 4);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t wireless_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t fw_version = 0x00;

	if (chip && chip->wls_fw_version) {
		fw_version = chip->wls_fw_version;
	}

	return sprintf(buf, "%08x\n", fw_version);
}
static DEVICE_ATTR(wireless_fw_version, 0444, wireless_fw_version_show, NULL);

static ssize_t wireless_fw_force_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(true);
	if (rc < 0)
		return rc;

	return count;
}
static DEVICE_ATTR(wireless_fw_force_update, 0220, NULL, wireless_fw_force_update_store);

static ssize_t wireless_fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	if(!chip->wls_online) {
		queue_delayed_work(chip->wls_wq,
				&chip->fw_update_work, msecs_to_jiffies(2000));
	} else {
		cps_wls_log(CPS_LOG_DEBG, "wireless_fw_update wls online,forbid fw update\n");
	}

	return count;
}
static DEVICE_ATTR(wireless_fw_update, 0220, NULL, wireless_fw_update_store);

static ssize_t mode_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;

	if (kstrtobool(buf, &val)) {
		chip->mode_select_force = false;
		cps_wls_log(CPS_LOG_DEBG, "mode_select_store force exit\n");
		return -EINVAL;
	}

	chip->mode_select_force = true;
	cps_wls_log(CPS_LOG_DEBG, "mode_select_store %d, wls_online:%d force:%d\n",
			val , chip->wls_online, chip->mode_select_force);
	if (chip->wls_online) {
		cps_wls_mode_select("mode_select_store", val);
	} else {
		cps_wls_log(CPS_LOG_ERR, "wls not online, can't ctl mode_sel\n");
	}

	return count;
}
static DEVICE_ATTR(mode_select, 0220, NULL, mode_select_store);

static ssize_t show_rx_irect(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "rx irect = %dmA\n", cps_wls_get_rx_irect());
}
static DEVICE_ATTR(get_rx_irect, 0444, show_rx_irect, NULL);

static ssize_t show_rx_vrect(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "rx vrect = %dmV\n", cps_wls_get_rx_vrect());
}
static DEVICE_ATTR(get_rx_vrect, 0444, show_rx_vrect, NULL);

static ssize_t show_rx_vout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "rx vout = %dmV\n", cps_wls_get_rx_vout());
}
static DEVICE_ATTR(get_rx_vout, 0444, show_rx_vout, NULL);

static ssize_t show_tx_vin(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "tx vin : %d\n", cps_wls_get_tx_vin());
}
static DEVICE_ATTR(get_tx_vin, 0444, show_tx_vin, NULL);

static ssize_t show_tx_iin(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "tx iin : %d\n", cps_wls_get_tx_i_in());
}
static DEVICE_ATTR(get_tx_iin, 0444, show_tx_iin, NULL);

static ssize_t show_tx_vrect(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "tx vrect : %d\n", cps_wls_get_tx_vrect());
}
static DEVICE_ATTR(get_tx_vrect, 0444, show_tx_vrect, NULL);

static ssize_t show_tx_mode_vout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cps_wls_get_tx_vrect());
}
static DEVICE_ATTR(tx_mode_vout, 0444, show_tx_mode_vout, NULL);

static void cps_wls_tx_mode(bool en)
{
	int retry = 0;
	CPS_TX_IRQ = false;
	cps_wls_set_boost(en);
	if ((true == en) && (false == CPS_TX_MODE)) {
		cps_wls_log(CPS_LOG_ERR,"cps mmi_mux wls tx start\n");
		if(chip->ntc_thermal) {
			cps_wls_log(CPS_LOG_ERR, "mmi_mux device too hot to enable tx mode\n");
			return;
		}
		mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_OTG, true);
		/* bootst voltage need 10ms to stable and cps need 30ms to stable*/
		msleep(100);
		cps_wls_enable_tx_mode();
#if 0
		if (chip->folio_mode) {
			cps_wls_set_tx_fod_thresh_I(fod_i_th_w_folio);
			cps_wls_set_tx_fod_thresh_II(fod_ii_th_w_folio);
		} else {
			cps_wls_set_tx_fod_thresh_I(fod_i_th_o_folio);
			cps_wls_set_tx_fod_thresh_II(fod_ii_th_o_folio);
		}
		cps_wls_dump_FW_info();
#endif
		while (retry < 100 && CPS_TX_IRQ == false) {
			msleep(1);
			retry ++;
		}
		cps_wls_log(CPS_LOG_DEBG,"cps wait tx_mode %dms\n", retry);

		CPS_TX_MODE = true;
		chip->tx_mode = true;
		sysfs_notify(&chip->dev->kobj, NULL, "tx_mode");
	} else if((false == en) && (true == CPS_TX_MODE)){
		cps_wls_log(CPS_LOG_ERR,"cps mmi_mux wls tx end\n");
		cps_wls_disable_tx_mode();
		//cps_wls_dump_FW_info();
		mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_OTG, false);
		 CPS_TX_MODE = false;
		chip->rx_connected = false;
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
		chip->tx_mode = false;
		sysfs_notify(&chip->dev->kobj, NULL, "tx_mode");
	} else {
		return;
	}

	cps_wls_log(CPS_LOG_DEBG,"cps mmi_mux wls tx mode %d\n",chip->tx_mode);
}

static void cps_wls_tx_enable(bool en)
{
	cps_wls_tx_mode(en);
}

static ssize_t tx_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long tx_mode;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &tx_mode);
	if (r) {
		cps_wls_log(CPS_LOG_ERR,"Invalid tx_mode = %lu\n", tx_mode);
		return -EINVAL;
	}

	cps_wls_tx_enable(!!tx_mode);

	return r ? r : count;
}

static ssize_t tx_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"PEN: chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", chip->tx_mode);
}
static DEVICE_ATTR(tx_mode, S_IRUGO|S_IWUSR, tx_mode_show, tx_mode_store);

static ssize_t rx_connected_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int rx_connected;
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	if (chip->ntc_thermal) {
		rx_connected = TX_MODE_OVERHEAT;
		return sprintf(buf, "%d\n", rx_connected);
	} else if (chip->tx_ept_flag) {
		rx_connected = TX_MODE_EPT_ERR;
		chip->tx_ept_flag = false;
		return sprintf(buf, "%d\n", rx_connected);
	}

	if (chip->rx_connected)
		rx_connected = TX_MODE_POWER_SHARE;
	else
		rx_connected = TX_MODE_NOT_CONNECTED;

	return sprintf(buf, "%d\n", rx_connected);

}

static DEVICE_ATTR(rx_connected, S_IRUGO,
		rx_connected_show,
		NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_input_curr_max;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_input_curr_max);
	if (r) {
		cps_wls_log(CPS_LOG_ERR, "Invalid TCMD = %lu\n", wls_input_curr_max);
		return -EINVAL;
	}

	if (chip->rx_ldo_on && wls_input_curr_max > 0) {
		if (wls_input_curr_max > chip->MaxI) {
			wls_input_curr_max = chip->MaxI;
		}
		if (!chip->chg1_dev)
			cps_init_charge_hardware();
		if (chip->chg1_dev)
			charger_dev_set_input_current(chip->chg1_dev, wls_input_curr_max * 1000);
	}

	cps_wls_log(CPS_LOG_DEBG,"wls input_current = %lu chip->MaxI = %dmA\n",
		wls_input_curr_max, chip->MaxI);
	chip->wls_input_curr_max = wls_input_curr_max;
	return r ? r : count;
}

static ssize_t wls_input_current_limit_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", chip->wls_input_curr_max);
}
static DEVICE_ATTR(wls_input_current_limit, S_IRUGO|S_IWUSR, wls_input_current_limit_show, wls_input_current_limit_store);

static ssize_t folio_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long folio_mode;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &folio_mode);
	if (r) {
		cps_wls_log(CPS_LOG_ERR,"Invalid folio_mode = %lu\n", folio_mode);
		return -EINVAL;
	}

	chip->folio_mode = folio_mode;

	return r ? r : count;
}

static ssize_t folio_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"PEN: chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", chip->folio_mode);
}
static DEVICE_ATTR(folio_mode, S_IRUGO|S_IWUSR, folio_mode_show, folio_mode_store);

static ssize_t show_wlc_fan_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, " %d\n", chip->fan_speed);
}
static int cps_wls_wlc_update_light_fan(void);
static ssize_t store_wlc_fan_speed(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	if (!chip->moto_stand) {
		cps_wls_log(CPS_LOG_ERR, "[%s] not moto 50w dock %d, skip\n", __func__ , chip->mode_type);
		return count;
	}

	chip->fan_speed = simple_strtoul(buf, NULL, 0);
	cps_wls_wlc_update_light_fan();

	return count;
}
static DEVICE_ATTR(wlc_fan_speed, S_IRUGO|S_IWUSR, show_wlc_fan_speed, store_wlc_fan_speed);

static ssize_t show_wlc_light_ctl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, " %d\n", chip->light_level);
}

static ssize_t store_wlc_light_ctl(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	if (!chip->moto_stand) {
		cps_wls_log(CPS_LOG_ERR, "[%s] not moto 50w dock %d, skip\n", __func__ , chip->mode_type);
		return count;
	}

	chip->light_level = simple_strtoul(buf, NULL, 0);
	cps_wls_wlc_update_light_fan();

	return count;
}
static DEVICE_ATTR(wlc_light_ctl, S_IRUGO|S_IWUSR, show_wlc_light_ctl, store_wlc_light_ctl);

static ssize_t show_wlc_tx_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", chip->wlc_tx_power);
}
static DEVICE_ATTR(wlc_tx_power, 0444, show_wlc_tx_power, NULL);

static ssize_t show_wlc_tx_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (chip->moto_stand)
		return sprintf(buf, "%d\n", WLC_MOTO);//Sys_Op_Mode_MOTO_WLC);
	else
		return sprintf(buf, "%d\n", chip->mode_type);
}

static DEVICE_ATTR(wlc_tx_type, 0444, show_wlc_tx_type, NULL);

static ssize_t show_wlc_st_changed(struct device *dev, struct device_attribute *attr, char *buf)
{
	int moto_wlc_status = WLC_DISCONNECTED;
	if (chip) {
		moto_wlc_status = chip->wlc_status;
	}
	return sprintf(buf, "%d\n", moto_wlc_status);
}

static DEVICE_ATTR(wlc_st_changed, S_IRUGO, show_wlc_st_changed, NULL);

static ssize_t show_wlc_tx_capability(struct device *dev, struct device_attribute *attr, char *buf)
{
	int wlc_tx_capability = 0;
	if (chip && chip->moto_stand) {
		wlc_tx_capability = motoauth.WLS_WLC_CAPABILITY;
	}
	return sprintf(buf, "%d\n", wlc_tx_capability);
}

static DEVICE_ATTR(wlc_tx_capability, 0444, show_wlc_tx_capability, NULL);

static ssize_t show_wlc_tx_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	int wlc_tx_id = 0;
	if (chip && chip->moto_stand) {
		wlc_tx_id = motoauth.WLS_WLC_ID;
	}

	return sprintf(buf, "%d\n", wlc_tx_id);
}

static DEVICE_ATTR(wlc_tx_id, 0444, show_wlc_tx_id, NULL);

static ssize_t show_wlc_tx_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
	int wlc_tx_sn = 0;
	if(chip && chip->moto_stand) {
		wlc_tx_sn = motoauth.WLS_WLC_SN;
	}
	return sprintf(buf, "%d\n", wlc_tx_sn);
}

static DEVICE_ATTR(wlc_tx_sn, 0444, show_wlc_tx_sn, NULL);

static ssize_t show_offset_detect_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable = -1;

	if (chip) {
		enable = chip->enable_rod;
	}

	return sprintf(buf, "%d\n", enable);
}

static ssize_t store_offset_detect_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtoul(buf, NULL, 0);
	if (chip && chip->enable_rx_offset_detect && chip->enable_rod != tmp) {
		if (chip->rx_ldo_on) {
			if (chip->rx_offset &&
				chip->wlc_status == WLC_ERR_LOWER_EFFICIENCY &&
				!tmp) {
				cps_wls_set_status(WLC_CHGING);
			} else if (tmp) {
				queue_delayed_work(chip->wls_wq, &chip->offset_detect_work, msecs_to_jiffies(1000));
			}
		}
		chip->rx_offset = false;
		chip->rx_offset_detect_count = 0;
		chip->enable_rod = tmp;
	}

	return count;
}
static DEVICE_ATTR(offset_detect_enable, 0664, show_offset_detect_enable, store_offset_detect_enable);

static ssize_t factory_wireless_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long en;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &en);
	if (r) {
		cps_wls_log(CPS_LOG_ERR,"Invalid factory_wls_en = %lu\n", en);
		return -EINVAL;
	}
	factory_test_wls_en(NULL, !!en);

	return r ? r : count;
}

static ssize_t factory_wireless_en_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"WLS: chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", chip->factory_wls_en);
}
static DEVICE_ATTR(factory_wireless_en, S_IRUGO|S_IWUSR, factory_wireless_en_show, factory_wireless_en_store);

static void cps_wls_create_device_node(struct device *dev)
{
	pr_info("%s\n", __func__);
	device_create_file(dev, &dev_attr_reg_addr);
	device_create_file(dev, &dev_attr_reg_data);
//-----------------------program---------------------
	device_create_file(dev, &dev_attr_wireless_fw_version);
	device_create_file(dev, &dev_attr_wireless_fw_update);
	device_create_file(dev, &dev_attr_wireless_fw_force_update);
//-----------------------write password--------------
	//device_create_file(dev, &dev_attr_write_password);

	device_create_file(dev, &dev_attr_factory_wireless_en);

//-----------------------RX--------------------------
	device_create_file(dev, &dev_attr_get_rx_irect);
	device_create_file(dev, &dev_attr_get_rx_vrect);
	device_create_file(dev, &dev_attr_get_rx_vout);
	device_create_file(dev, &dev_attr_mode_select);
//-----------------------TX--------------------------
	device_create_file(dev, &dev_attr_get_tx_vin);
	device_create_file(dev, &dev_attr_get_tx_iin);
	device_create_file(dev, &dev_attr_get_tx_vrect);
	device_create_file(dev, &dev_attr_tx_mode_vout);

	device_create_file(dev, &dev_attr_tx_mode);
	device_create_file(dev, &dev_attr_rx_connected);
	device_create_file(dev, &dev_attr_wls_input_current_limit);
	device_create_file(dev, &dev_attr_folio_mode);

	device_create_file(dev, &dev_attr_wlc_fan_speed);
	device_create_file(dev, &dev_attr_wlc_light_ctl);
	device_create_file(dev, &dev_attr_wlc_tx_power);
	device_create_file(dev, &dev_attr_wlc_tx_type);
	device_create_file(dev, &dev_attr_wlc_st_changed);

//-----------------------MOTO WLC2.0-------------------
	device_create_file(dev, &dev_attr_wlc_tx_capability);
	device_create_file(dev, &dev_attr_wlc_tx_id);
	device_create_file(dev, &dev_attr_wlc_tx_sn);
//-----------------------MOTO RX Offset Detect---------
	device_create_file(dev, &dev_attr_offset_detect_enable);
}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *boot_node = NULL;
	struct tags_bootmode *tag = NULL;

	pr_info("%s\n", __func__);
	boot_node = of_parse_phandle(node, "bootmode", 0);
	if (!boot_node)
		cps_wls_log(CPS_LOG_ERR, "%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tags_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			cps_wls_log(CPS_LOG_ERR, "%s: failed to get atag,boot\n", __func__);
		else {
			cps_wls_log(CPS_LOG_ERR, "%s: size:0x%x tag:0x%x bootmode:0x%x\n",
				__func__, tag->size, tag->tag, tag->bootmode);
			chip->bootmode = tag->bootmode;
		}
	}

	if(!node){
		cps_wls_log(CPS_LOG_ERR, "devices tree node missing \n");
		return -EINVAL;
	}

	chip->wls_charge_int = of_get_named_gpio(node, "cps_wls_int", 0);
	if(!gpio_is_valid(chip->wls_charge_int))
		return -EINVAL;

	chip->wls_det_int = of_get_named_gpio(node, "cps_det_int", 0);
	if(!gpio_is_valid(chip->wls_det_int))
		return -EINVAL;

/*
	chip->otg_switch_en = of_get_named_gpio(node, "mmi,mux_otg_switch_en", 0);
	if (!gpio_is_valid(chip->otg_switch_en))
		cps_wls_log(CPS_LOG_ERR,"mmi otg_switch_en gpio is invalid\n");
	else {
		if (gpio_request(chip->otg_switch_en, "mmi_otg_en_pin") < 0)
			cps_wls_log(CPS_LOG_ERR, " [%s] Failed to request otg_switch_en gpio\n", __func__);
		else if (gpio_direction_output(chip->otg_switch_en, 0) < 0) {
			cps_wls_log(CPS_LOG_ERR, "[%s] set otg_switch_en GPIO to Low failed\n", __func__);
		}
	}
*/

	chip->wls_mode_select = of_get_named_gpio(node, "wls_mode_select", 0);
	if(!gpio_is_valid(chip->wls_mode_select))
		cps_wls_log(CPS_LOG_ERR,"wls_mode_select is %d invalid\n", chip->wls_mode_select );

	of_property_read_string(node, "wireless-fw-name", &chip->wls_fw_name);
	cps_wls_log(CPS_LOG_ERR, "[%s]	wls_charge_int %d wls_det_int %d wls_fw_name: %s\n",
			__func__, chip->wls_charge_int, chip->wls_det_int, chip->wls_fw_name);

	chip->enable_stop_epp = 0x00;
	of_property_read_u32(node, "enable_stop_epp", &chip->enable_stop_epp);
	cps_wls_log(CPS_LOG_ERR,"[%s] enable_stop_epp %d\n", __func__, chip->enable_stop_epp);

	chip->enable_rx_offset_detect = 0x00;
	of_property_read_u32(node, "enable_rx_offset_detect", &chip->enable_rx_offset_detect);
	cps_wls_log(CPS_LOG_DEBG,"[%s] enable_rx_offset_detect %d\n", __func__, chip->enable_rx_offset_detect);

	chip->rod_stop_battery_soc = WLS_ROD_STOP_BATERY_SOC;
	of_property_read_u32(node, "rod_stop_battery_soc", &chip->rod_stop_battery_soc);
	cps_wls_log(CPS_LOG_DEBG,"[%s] rod_stop_battery_soc %d\n", __func__, chip->rod_stop_battery_soc);

	return 0;
}

static int cps_wls_gpio_request(struct cps_wls_chrg_chip *chip)
{
	int ret =0;
	pr_info("%s\n", __func__);
	ret = devm_gpio_request_one(chip->dev, chip->wls_charge_int,
					GPIOF_IN, "cps4041_ap_int");
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"Failed to request int gpio, ret:%d", ret);
		return ret;
	}
	chip->cps_wls_irq = gpio_to_irq(chip->wls_charge_int);
	if (chip->cps_wls_irq < 0) {
		cps_wls_log(CPS_LOG_ERR,"failed get irq num %d", chip->cps_wls_irq);
		return -EINVAL;
	}

	ret = devm_gpio_request_one(chip->dev, chip->wls_det_int,
					GPIOF_IN, "cps4041_wls_det_int");
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"Failed to request det_int gpio, ret:%d", ret);
		return ret;
	}
	chip->wls_det_irq = gpio_to_irq(chip->wls_det_int);
	if (chip->wls_det_irq < 0) {
		cps_wls_log(CPS_LOG_ERR,"failed get det irq num %d", chip->wls_det_irq);
		return -EINVAL;
	}

	if(gpio_is_valid(chip->wls_mode_select)) {
		ret = devm_gpio_request_one(chip->dev, chip->wls_mode_select,
					GPIOF_OUT_INIT_LOW, "wls_mode_select");
		if (ret < 0)
			cps_wls_log(CPS_LOG_ERR," [%s] Failed to request wls_mode_select gpio, ret:%d", __func__, ret);
	}


	return ret;
}


static void cps_wls_lock_work_init(struct cps_wls_chrg_chip *chip)
{
	char *name = NULL;
	pr_info("%s\n", __func__);
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->i2c_lock);
	name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", "cps_wls_wake_lock");
	chip->cps_wls_wake_lock = wakeup_source_register(NULL, name);
}


static void cps_wls_lock_destroy(struct cps_wls_chrg_chip *chip)
{
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->i2c_lock);
//	wake_lock_destroy(&chip->cps_wls_wake_lock);
	//cancel_delayed_work_sync(&chip->cps_wls_monitor_work);
}

static char *wl_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static int cps_wls_register_psy(struct cps_wls_chrg_chip *chip)
{
	struct power_supply_config cps_wl_psy_cfg = {};

	chip->wl_psd.name = CPS_WLS_CHRG_PSY_NAME;
	chip->wl_psd.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wl_psd.properties = cps_wls_chrg_props;
	chip->wl_psd.num_properties = ARRAY_SIZE(cps_wls_chrg_props);
	chip->wl_psd.get_property = cps_wls_chrg_get_property;
	chip->wl_psd.set_property = cps_wls_chrg_set_property;
	chip->wl_psd.property_is_writeable= cps_wls_chrg_property_is_writeable;
	chip->wl_psd.external_power_changed = cps_wls_charger_external_power_changed;

	cps_wl_psy_cfg.drv_data = chip;
	cps_wl_psy_cfg.of_node = chip->dev->of_node;
	cps_wl_psy_cfg.supplied_to = wl_psy_supplied_to,
	cps_wl_psy_cfg.num_supplicants = ARRAY_SIZE(wl_psy_supplied_to),
	chip->wl_psy = power_supply_register(chip->dev,
								&chip->wl_psd,
								&cps_wl_psy_cfg);
	if(IS_ERR(chip->wl_psy)){
		return PTR_ERR(chip->wl_psy);
	}
	return CPS_WLS_SUCCESS;
}

static int factory_test_wls_en(void *input, bool en)
{
	int ret = 0;
	int wait = 0;
	struct chg_alg_device *alg;

	alg = get_chg_alg_by_name("wlc");
	if (!alg) {
		cps_wls_log(CPS_LOG_ERR,"wls: not found wlc\n");
		return -1;
	}

	if (en) {
		if (chip->factory_wls_en == false) {
			chip->factory_wls_en = true;
			ret = mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, true);
		}
	} else {
		if (chip->factory_wls_en == true) {
			wait = 50;
			while (wait > 0 && chip->hs_st == HS_UNKONWN) {
				msleep(100);
				wait --;
			}
			cps_wls_mode_select("factory_test_stop_epp", false);
			wait = 50;
			while (wait > 0 && (cps_get_vbus() > 6000)) {
				msleep(10);
				wait --;
			}
			ret = mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, false);
			chip->factory_wls_en = false;
			cps_wls_mode_select("factory_test_start_epp", true);
		}
	}

	cps_wls_log(CPS_LOG_ERR,"wls: factory wls_en %d, ret=%d\n", en, ret);
	return ret;
}

static int wireless_get_chip_id(void *input)
{
	int value = chip->chip_id;

	if(0 != value)
		return value;
	if (chip->rx_ldo_on) {
		value = cps_wls_get_chip_id();
		if (value == CPS_CHIP_ID)
			chip->chip_id = value;
	} else {
		cps_wls_tx_enable(true);
		value = cps_wls_get_chip_id();
		chip->chip_id = value;
		cps_wls_tx_enable(false);
	}
	return value;
}

int cps_wls_get_ldo_on(void)
{
	if (!chip)
		return 0;
	return chip->rx_ldo_on;
}

static int wls_tcmd_register(struct cps_wls_chrg_chip *cm)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	cm->wls_tcmd_client.data = cm;
	cm->wls_tcmd_client.client_id = MOTO_CHG_TCMD_CLIENT_WLS;

	cm->wls_tcmd_client.get_chip_id = wireless_get_chip_id;
	cm->wls_tcmd_client.wls_en = factory_test_wls_en;

	ret = moto_chg_tcmd_register(&cm->wls_tcmd_client);

	return ret;
}

static void cps_wls_current_select(int *icl, int *vbus, bool *cable_ready)
{
	struct cps_wls_chrg_chip *chg = chip;
	uint32_t wls_power = 0;
	int wls_voltage = 0;

	if (chip->cable_ready_wait_count < 3 && !chip->moto_stand)
	{
		*cable_ready = false;
		chip->cable_ready_wait_count++;
		return;
	}
	*cable_ready = true;
	*icl = 400000;
	*vbus = 5000;

	if (chg->mode_type == Sys_Op_Mode_BPP)
	{
		if (!chg->bpp_icl_done){
			chg->MaxV = 5000;
			chg->MaxI = 1000;
			*icl = 100000;
			*vbus = 5000;
			return;
		}
		chg->MaxV = 5000;
		chg->MaxI = 1000;
		*icl = 1000000;
		*vbus = 5000;
	}
	else if (chg->mode_type == Sys_Op_Mode_EPP)
	{
		wls_power = cps_wls_get_rx_neg_power() / 2;
		wls_voltage = cps_wls_get_rx_vout();
		cps_wls_log(CPS_LOG_DEBG, "%s cps4041 power:%dW vout:%dmV",
						__func__, wls_power, wls_voltage);
		if (wls_power >= WLS_RX_CAP_15W)
		{
			chg->MaxV = 12000;
			chg->MaxI = 1150;
			*icl = 1150000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_10W)
		{
			chg->MaxV = 12000;
			chg->MaxI = 800;
			*icl = 800000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_8W)
		{
			chg->MaxV = 12000;
			chg->MaxI = 650;
			*icl = 650000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_5W)
		{
			if (wls_voltage < 5500) {
				chg->MaxV = 5000;
				chg->MaxI = 1000;
			} else {
				chg->MaxV = 12000;
				chg->MaxI = 400;
			}
			*icl = chg->MaxI * 1000;
			*vbus = chg->MaxV;
		}
		else
		{
			chg->MaxV = 5000;
			chg->MaxI = 1000;
			*icl = 1000000;
			*vbus = 5000;
		}
	}
	if (chip->wls_input_curr_max != 0 && chip->wls_input_curr_max < chg->MaxI)
		*icl = chip->wls_input_curr_max * 1000;
}

static void cps_epp_current_select(int *icl, int *vbus)
{
	struct cps_wls_chrg_chip *chg = chip;
	uint32_t wls_power = 0;
	int wls_voltage = 0;
	pr_info("%s\n", __func__);

	*icl = 400000;
	*vbus = 5000;
	if (chg->mode_type == Sys_Op_Mode_EPP) {
		wls_power = cps_wls_get_rx_neg_power() / 2;
		wls_voltage = cps_wls_get_rx_vout();
		cps_wls_log(CPS_LOG_DEBG, "%s cps4041 power:%dW vout:%dmV",
						__func__, wls_power, wls_voltage);
		if (wls_power >= WLS_RX_CAP_15W) {
			chg->MaxV = 12000;
			chg->MaxI = 1150;
			*icl = 1150000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_10W) {
			chg->MaxV = 12000;
			chg->MaxI = 800;
			*icl = 800000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_8W) {
			chg->MaxV = 12000;
			chg->MaxI = 650;
			*icl = 650000;
			*vbus = 12000;
		}
		else if (wls_power >= WLS_RX_CAP_5W) {
			if (wls_voltage < 5500) {
				chg->MaxV = 5000;
				chg->MaxI = 1000;
			} else {
				chg->MaxV = 12000;
				chg->MaxI = 400;
			}
			*icl = chg->MaxI * 1000;
			*vbus = chg->MaxV;
		} else {
			chg->MaxV = 5000;
			chg->MaxI = 1000;
			*icl = 1000000;
			*vbus = 5000;
		}
	}
	if (chip->wls_input_curr_max != 0 && chip->wls_input_curr_max < chg->MaxI)
		*icl = chip->wls_input_curr_max * 1000;
	cps_wls_log(CPS_LOG_DEBG, "%s icl=%duA vbus=%dmV wls_power=%d", __func__, *icl, *vbus, wls_power);
}

static void cps_wls_dump_info_work(struct work_struct *work)
{
	if (!chip)
		return;

	if (!chip->rx_ldo_on)
		return;

	chip->rx_fop = cps_wls_get_rx_fop_value();
	chip->rx_ept = cps_wls_get_rx_ept_code();
	chip->rx_ce = cps_wls_get_rx_ce_pkt_value();
	chip->rx_dietmp = cps_wls_get_rx_die_tmp();
	cps_wls_log(CPS_LOG_DEBG, "cps_dump_info fop %dkHz,ept 0x%04X, ce %d, dietmp %d\n",
		chip->rx_fop, chip->rx_ept, chip->rx_ce, chip->rx_dietmp);

	queue_delayed_work(chip->wls_wq, &chip->dump_info_work, msecs_to_jiffies(3000));
}

static void cps_wls_light_fan_work(struct work_struct *work)
{
	int uisoc = 0;

	if (!chip)
		return;

	if (!chip->rx_ldo_on)
		return;

	uisoc = cps_get_bat_info(POWER_SUPPLY_PROP_CAPACITY);
	cps_wls_log(CPS_LOG_DEBG, "%s moto:%d boot:%d uisoc:%d\n",
			__func__, chip->moto_stand, chip->bootmode, uisoc);

	if (uisoc == 100)
		CPS_RX_CHRG_FULL = true;

	if (chip->moto_stand) {
		cps_wls_wlc_update_light_fan();
	} else if(CPS_RX_CHRG_FULL) {
		cps_wls_notify_tx_chrgfull();
	}
}

static int cps_wls_wlc_update_light_fan(void)
{
	int status = CPS_WLS_SUCCESS;
	uint8_t data[4] = {0x38, 0x05, 0x40, 0x28};
	int retry = 2;

	if (CPS_RX_CHRG_FULL) {
		if (chip->light_level == 0)
			data[2] = MMI_DOCK_LIGHT_OFF;
		else
			data[2] = MMI_DOCK_LIGHT_ON;
		data[3] = MMI_DOCK_FAN_SPEED_OFF;
	} else {
		if (chip->fan_speed == 0)
			data[3] = MMI_DOCK_FAN_SPEED_LOW;
		else
			data[3] = MMI_DOCK_FAN_SPEED_HIGH;

		if (chip->light_level == 0)
			data[2] = MMI_DOCK_LIGHT_OFF;
		else
			data[2] = MMI_DOCK_LIGHT_DEFAULT;
	}

	do {
		//status = cps_wls_send_fsk_packet(data, 4);
		status = cps_wls_send_ask_packet(data, 4);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS: QI set fan/light, WLS_WLC_FAN_SPEED %d, CPS_RX_CHRG_FULL %d, WLS_WLC_LIGHT %d",
					chip->fan_speed, CPS_RX_CHRG_FULL, chip->light_level);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS: QI set fan/light, ight 0x%x, fan 0x%x", data[2], data[3]);
		msleep(200);
		retry--;
	} while (retry);

	return 0;
}

static void cps_wls_notify_tx_chrgfull(void)
{
	int status = CPS_WLS_SUCCESS;
	uint8_t data[2] = {0x5, 0x64};
	int retry = 5;

	do {
		//status = cps_wls_send_fsk_packet(data, 2);
		status = cps_wls_send_ask_packet(data, 2);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS: QI notify TX battery full , head 0x%x, cmd 0x%x, status %d\n", 0x5, 0x64,status);
		msleep(200);
		retry--;
	} while(retry);

	return;
}

static int cps_wls_notify_st_changed(void)
{
	static int pre_status = -1;

	if (pre_status != chip->wlc_status) {
		cps_wls_log(CPS_LOG_DEBG,"%s st change	%d -> %d\n",
				__func__, pre_status, chip->wlc_status);
		pre_status = chip->wlc_status;
		sysfs_notify(&chip->wl_psy->dev.parent->kobj, NULL, "wlc_st_changed");
	}

	return 0;
}

int cps_wls_sysfs_notify(const char *attr)
{
	cps_wls_log(CPS_LOG_DEBG,"%s %s\n", __func__, attr);
	if(chip && chip->wl_psy) {
		sysfs_notify(&chip->wl_psy->dev.parent->kobj, NULL, attr);
		//pr_info("wireless charger notify, event %lu\n", event);
		power_supply_changed(chip->wl_psy);
	}
	return 0;
}

static int cps_wls_set_status(int status)
{
	chip->wlc_status = status;
	if (status == WLC_DISCONNECTED)
		chip->moto_stand = false;
	cps_wls_notify_st_changed();

	return 0;
}

static void cps_wls_set_battery_soc(int uisoc)
{
	struct cps_wls_chrg_chip *chg = chip;
	int soc = uisoc;
	cps_wls_log(CPS_LOG_DEBG, "cps_wls_set_battery_soc");
	if (!CPS_RX_CHRG_FULL && chg->wls_online && soc == 100) {
		cps_wls_log(CPS_LOG_DEBG, "cps Notify TX, battery has been charged full !");
		CPS_RX_CHRG_FULL = true;
		if (chip->moto_stand) {
			cps_wls_wlc_update_light_fan();
		} else
			cps_wls_notify_tx_chrgfull();
	}
}

static bool cps_stop_epp_timeout(long ms)
{
	ktime_t ktime_now;

	if (!chip)
		return true;

	ktime_now = ktime_get_boottime();
	if (ktime_now >= chip->stop_epp_ktime + ms *1000000) {
		cps_wls_log(CPS_LOG_DEBG, "cps_stop_epp_timeout start:%ld\n",
					(long int)chip->stop_epp_ktime);
		cps_wls_log(CPS_LOG_DEBG, "cps_stop_epp_timeout now:%ld\n",
					(long int)ktime_now);
		chip->stop_epp_ktime = 0;
		chip->stop_epp_flag = false;
		cps_wls_mode_select("cps_stop_epp_timeout", true);
		return true;
	} else {
		return false;
	}
}

static void cps_wls_stop_epp(void)
{
	struct cps_wls_chrg_chip *chg = chip;

	if (!chg) {
		cps_wls_log(CPS_LOG_DEBG, "%s chg=NULL\n", __func__);
		return;
	}

	if (!chg->wls_online) {
		cps_wls_log(CPS_LOG_DEBG, "%s wls_online=0\n", __func__);
		return;
	}

	cps_wls_log(CPS_LOG_DEBG, "%s mode_type:%d stop_epp_flag:%d\n",
					__func__, chg->mode_type, chg->stop_epp_flag);

	if (chip->enable_stop_epp &&
		(chip->bootmode == KERNEL_POWER_OFF_CHARGING_BOOT ||
		chip->bootmode == LOW_POWER_OFF_CHARGING_BOOT)) {
		if (chip->moto_stand) {
			chip->light_level = 0;
			cps_wls_log(CPS_LOG_DEBG, "%s moto stand set light level 0\n", __func__);
			if (chg->mode_type == Sys_Op_Mode_BPP) {
				cps_wls_wlc_update_light_fan();
			}
		}
		if (chg->mode_type == Sys_Op_Mode_EPP) {
			if (CPS_WLS_SUCCESS == cps_wls_mode_select("cps_wls_stop", false)) {
				chg->stop_epp_flag = true;
				chg->stop_epp_ktime = ktime_get_boottime();
				cps_wls_log(CPS_LOG_DEBG, "%s start\n", __func__);
			} else {
				cps_wls_log(CPS_LOG_ERR, "%s failed\n", __func__);
			}
		}
	}
}

static int wls_chg_ops_register(struct cps_wls_chrg_chip *cm)
{
	int ret = -1;

	pr_info("%s\n", __func__);
	cm->wls_chg_ops.wls_current_select = cps_wls_current_select;
	cm->wls_chg_ops.wls_set_battery_soc = cps_wls_set_battery_soc;
	cm->wls_chg_ops.wls_stop_epp = cps_wls_stop_epp;
	cm->wls_chg_ops.wls_notify_thermal_icl = cps_wls_notify_thermal_input_current_limit;
	cm->wls_chg_ops.wls_notify_cur_state = cps_wls_notify_cur_state;

	ret = moto_wireless_chg_ops_register(&cm->wls_chg_ops);

	return ret;
}

static int cps_wls_rx_power_on(void)
{
	struct cps_wls_chrg_chip *info = chip;
	uint32_t chip_id = 0, sys_mode = 0;
	bool rx_power = false;
	static int rx_power_cnt = 0;
#ifdef SMART_PEN_SUPPORT
	if (CPS_RX_MODE_ERR || CPS_TX_MODE || chip->pen_power_on) {
		cps_wls_log(CPS_LOG_ERR, "CPS_RX_MODE_ERR %d, CPS_TX_MODE %d, PEN_POWER_ON%d",
			CPS_RX_MODE_ERR, CPS_TX_MODE, chip->pen_power_on);
		cps_wls_log(CPS_LOG_ERR, "Stop wls timer, and report wls rx offline");
		alarm_try_to_cancel(&info->wls_rx_timer);
		return false;
	}
#else
	if (CPS_RX_MODE_ERR || CPS_TX_MODE) {
		cps_wls_log(CPS_LOG_ERR, "CPS_RX_MODE_ERR %d, CPS_TX_MODE %d",
			CPS_RX_MODE_ERR, CPS_TX_MODE);
		cps_wls_log(CPS_LOG_ERR, "Stop wls timer, and report wls rx offline");
		alarm_try_to_cancel(&info->wls_rx_timer);
		return false;
	}
#endif

	sys_mode = cps_wls_get_sys_mode();
	cps_wls_log(CPS_LOG_DEBG, "CPS_TX_MODE %d, sys_mode RX/TX, 0 mode %d", CPS_TX_MODE, sys_mode);

	chip_id = cps_wls_get_chip_id();
	if(chip_id == CPS_CHIP_ID && sys_mode == SYS_MODE_RX) {
		rx_power = true;
		rx_power_cnt = 0;
		info->chip_id = chip_id;
	} else {
		rx_power = false;
		rx_power_cnt++;
	}

	if (rx_power) {
		cps_wls_log(CPS_LOG_DEBG, "Rx power on, Re-check after 500ms");
		info->rx_polling_ns = 500 * 1000 * 1000;
		wls_rx_start_timer(info);
		return true;
	} else {

		if (rx_power_cnt > 1) {
			cps_wls_log(CPS_LOG_DEBG, "Rx power off");
			rx_power_cnt = 0;
			return false;
		} else {
			info->rx_polling_ns = 500 * 1000 * 1000;
			wls_rx_start_timer(info);
			cps_wls_log(CPS_LOG_DEBG, "Re-check after 500ms");
			//dc_get_power_supply_properties(DC_POWER_SUPPLY_ONLINE, &value);
			return (info->wls_online ? true : false);
		}
	}
}

static int get_pmic_vbus(struct mtk_charger *info, int *vchr)
{
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	int ret;

	if (chg_psy == NULL)
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	}
	*vchr = prop.intval;

	chr_debug("%s vbus:%d\n", __func__,
		prop.intval);
	return ret;
}

static int cps_get_vbus(void)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;
	int vbus = 0;
	int ret = -1;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s mmi_mux Couldn't get chg_psy\n", __func__);
		return CPS_WLS_FAIL;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	}
	//vbus = get_vbus(info);
	if (info == NULL)
		return 0;
	ret = charger_dev_get_vbus(info->chg1_dev, &vbus);
	if (ret < 0) {
		ret = get_pmic_vbus(info, &vbus);
		if (ret < 0)
			chr_err("%s: get vbus failed: %d\n", __func__, ret);
	} else
		vbus /= 1000;
	cps_wls_log(CPS_LOG_ERR, "%s: vbus:%d\n", __func__,vbus);
	return vbus;
}

static int cps_rx_check_events_thread(void *arg)
{
	int status = CPS_WLS_SUCCESS;
	struct cps_wls_chrg_chip *info = arg;

	while (1) {
		status = wait_event_interruptible(info->wait_que,
			(info->wls_rx_check_thread_timeout == true));
		if (status < 0) {
			cps_wls_log(CPS_LOG_ERR, "%s: wait event been interrupted\n", __func__);
			continue;
		}
		info->wls_rx_check_thread_timeout = false;
		if (cps_get_vbus() < 5000 || !chip->rx_int_ready)
			cps_rx_online_check(info);
		__pm_relax(info->rx_check_wakelock);
	}
	return 0;
}

static void wake_up_rx_check_thread(struct cps_wls_chrg_chip *info)
{
	if (info == NULL)
		return;

	if (!info->rx_check_wakelock->active)
		__pm_stay_awake(info->rx_check_wakelock);
	info->wls_rx_check_thread_timeout = true;
	wake_up_interruptible(&info->wait_que);
}

static enum alarmtimer_restart
	wls_rx_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct cps_wls_chrg_chip *info = chip;

	wake_up_rx_check_thread(info);

	return ALARMTIMER_NORESTART;
}

static void wls_rx_start_timer(struct cps_wls_chrg_chip *info)
{
	struct timespec64 endtime, time_now;
	ktime_t ktime, ktime_now;
	int ret = 0;

	/* If the timer was already set, cancel it */
	ret = alarm_try_to_cancel(&info->wls_rx_timer);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"%s: callback was running, skip timer\n", __func__);
		return;
	}

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	endtime.tv_sec = time_now.tv_sec + 0;
	endtime.tv_nsec = time_now.tv_nsec + info->rx_polling_ns;
	info->end_time = endtime;
	ktime = ktime_set(info->end_time.tv_sec, info->end_time.tv_nsec);

	cps_wls_log(CPS_LOG_DEBG,"%s: alarm timer start:%d, %lld %ld\n", __func__, ret,
		info->end_time.tv_sec, info->end_time.tv_nsec);
	alarm_start(&info->wls_rx_timer, ktime);
}

static void wls_rx_init_timer(struct cps_wls_chrg_chip *info)
{
	alarm_init(&info->wls_rx_timer, ALARM_BOOTTIME,
			wls_rx_alarm_timer_func);

}

static int cps_tcd_get_max_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	*state = 1;

	return 0;
}

static int cps_tcd_get_cur_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	struct cps_wls_chrg_chip *chip = tcd->devdata;

	*state = chip->ntc_thermal;

	return 0;
}

static int cps_tcd_set_cur_state(struct thermal_cooling_device *tcd,
	unsigned long state)
{
	struct cps_wls_chrg_chip *chip = tcd->devdata;

	if (chip->fw_uploading) {
		cps_wls_log(CPS_LOG_DEBG, "cps fw uploading,ignore thermal event\n");
		return 0;
	}
	if (state && !chip->ntc_thermal) {
		cps_wls_log(CPS_LOG_DEBG, "cps Wireless charger overtemp\n");
		chip->ntc_thermal = true;
		cps_wls_tx_enable(false);
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
	} else if (!state && chip->ntc_thermal) {
		cps_wls_log(CPS_LOG_DEBG, "cps Wireless charger temp OK\n");
		chip->ntc_thermal = false;
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
	}

	return 0;
}

static const struct thermal_cooling_device_ops cps_tcd_ops = {
	.get_max_state = cps_tcd_get_max_state,
	.get_cur_state = cps_tcd_get_cur_state,
	.set_cur_state = cps_tcd_set_cur_state,
};

static void cps_init_charge_hardware(void)
{
	struct cps_wls_chrg_chip *chg = chip;
	pr_info("%s\n", __func__);
	chg->chg1_dev = get_charger_by_name("primary_chg");
	if (chg->chg1_dev)
		cps_wls_log(CPS_LOG_DEBG, "%s: Found primary charger\n", __func__);
	else {
		cps_wls_log(CPS_LOG_ERR, "%s: Error : can't find primary charger\n",
			__func__);
	}
}

static int cps_wls_chrg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret=0;
	char *name = NULL;
	
	cps_wls_log(CPS_LOG_ERR, "[%s] ---->start\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"[%s] cps_debug: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "cps_wls";
	chip->regmap = devm_regmap_init_i2c(client, &cps4041_regmap_config);
	if (IS_ERR(chip->regmap)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap);
	}
	chip->regmap32 = devm_regmap_init_i2c(client, &cps4041_regmap32_config);
	if (IS_ERR(chip->regmap)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap);
	}

	i2c_set_clientdata(client, chip);
	dev_set_drvdata(&(client->dev), chip);

	name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", "cps suspend wakelock");
	chip->rx_check_wakelock = wakeup_source_register(NULL, name);

	ret = cps_wls_parse_dt(chip);
	if(ret < 0){
		cps_wls_log(CPS_LOG_ERR, "[%s] Couldn't parse DT nodes ret = %d\n", __func__, ret);
		goto free_source;
	}

	ret = cps_wls_gpio_request(chip);
	if(ret < 0){
		cps_wls_log(CPS_LOG_ERR, "[%s] gpio request failed ret = %d\n", __func__, ret);
		goto free_source;
	}

	if(chip->cps_wls_irq){
		ret = devm_request_threaded_irq(&client->dev, chip->cps_wls_irq, NULL,
			cps_wls_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "cps_wls_irq", chip);
		if(ret){
			cps_wls_log(CPS_LOG_ERR, "[%s] request cps_wls_int irq failed ret = %d\n", __func__, ret);
			goto free_source;
		}
		enable_irq_wake(chip->cps_wls_irq);
	}

	if(chip->wls_det_irq){
		ret = devm_request_threaded_irq(&client->dev, chip->wls_det_irq, NULL,
			wls_det_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "wls_det_irq", chip);
		if(ret){
			cps_wls_log(CPS_LOG_ERR, "[%s] request wls_det_irq irq failed ret = %d\n", __func__, ret);
			goto free_source;
		}
		enable_irq_wake(chip->wls_det_irq);
	}

	//Enable IC EPP mode as default
	cps_wls_mode_select("cps_wls_chrg_probe", true);

	cps_wls_lock_work_init(chip);

	cps_wls_create_device_node(&(client->dev));

	ret = cps_wls_register_psy(chip);
	if(IS_ERR(chip->wl_psy)){
		cps_wls_log(CPS_LOG_ERR, "[%s] power_supply_register wireless failed , ret = %d\n", __func__, ret);
		goto free_source;
	}

	// wake_lock(&chip->cps_wls_wake_lock);
	wls_tcmd_register(chip);
	chip->rx_connected = false;
	chip->wls_online = false;
	wls_chg_ops_register(chip);
	chip->wls_input_curr_max = 0;
	chip->MaxV = 12000;
	chip->MaxI = 1250;
	chip->chip_id = 0;
	chip->factory_wls_en = false;
	chip->mode_type = Sys_Op_Mode_INVALID;
	chip->fan_speed = 1;
	chip->light_level = 1;
	chip->moto_stand = false;

	init_waitqueue_head(&chip->wait_que);
	wls_rx_init_timer(chip);
	chip->wls_wq = create_singlethread_workqueue("wls_workqueue");
	INIT_DELAYED_WORK(&chip->fw_update_work,
				cps_firmware_update_work);
	INIT_DELAYED_WORK(&chip->bpp_icl_work,
				cps_bpp_mode_icl_work);
	INIT_DELAYED_WORK(&chip->light_fan_work,
				cps_wls_light_fan_work);

	if (chip->enable_rx_offset_detect) {
		chip->enable_rod = true;
		INIT_DELAYED_WORK(&chip->offset_detect_work, cps_offset_detect_work);
	}

	INIT_DELAYED_WORK(&chip->dump_info_work,
				cps_wls_dump_info_work);

	/* Register thermal zone cooling device */
	chip->tcd = thermal_of_cooling_device_register(dev_of_node(chip->dev),
		"cps_wls_charger_l", chip, &cps_tcd_ops);
	cps_init_charge_hardware();

	kthread_run(cps_rx_check_events_thread, chip, "cps_rx_check_thread");

	cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger addr low probe successful!\n", __func__);


	motoauth.wls_get_ldo_on = cps_wls_get_ldo_on;
	motoauth.wls_send_ask_packet = cps_wls_send_ask_packet;
	motoauth.wls_sysfs_notify = cps_wls_sysfs_notify;
	motoauth.wls_set_status = cps_wls_set_status;
	if (0 == moto_wls_auth_init(&motoauth)){
		cps_wls_log(CPS_LOG_DEBG, "[%s] moto_wls_auth_init successful!\n", __func__);
	} else {
		cps_wls_log(CPS_LOG_ERR, "[%s] moto_wls_auth_init failed!\n", __func__);
	}

	cps_wls_log(CPS_LOG_DEBG, "cps_wls_get_sys_mode %d\n", cps_wls_get_sys_mode());
	cps_wls_log(CPS_LOG_DEBG, "cps_wls_get_chip_id 0x%X\n", cps_wls_get_chip_id());
	cps_wls_log(CPS_LOG_DEBG, "cps_wls_get_sys_fw_version 0x%X\n", cps_wls_get_sys_fw_version());

	//queue_delayed_work(chip->wls_wq,&chip->fw_update_work, msecs_to_jiffies(5000));
	cps_wls_reg_check();
	return ret;

free_source:
	cps_wls_lock_destroy(chip);
	cps_wls_log(CPS_LOG_ERR, "[%s] error: free resource.\n", __func__);

	return ret;
}

static void not_called_api(void)
{
	/*int rc;
	rc = cps_wls_get_rx_ss_pkt_value();
	rc = cps_wls_get_rx_ce_pkt_value();
	rc = cps_wls_get_rx_rp_pkt_value();
	rc = cps_wls_get_rx_fop_value();
	rc = cps_wls_get_rx_ept_code();
	rc = cps_wls_get_rx_neg_power();
	rc = cps_wls_get_rx_neg_pro();
	rc = cps_wls_get_rx_vrect();
	rc = cps_wls_get_rx_irect();
	rc = cps_wls_get_rx_vout();
	rc = cps_wls_get_rx_die_tmp();
	rc = cps_wls_set_rx_vout_target(5000);
	rc = cps_wls_set_rx_neg_power(20);
	rc = cps_wls_get_tx_ce_value();
	rc = cps_wls_get_tx_rp_value();*/
	int rc;
	uint8_t data[2] = {0x1F, 0xAC};
	rc = cps_wls_set_tx_fod0_thresh(3000);
	//rc = cps_wls_set_tx_fod1_thresh(3000);
	//rc = cps_wls_set_tx_fod2_thresh(3000);
	//rc = cps_wls_set_tx_fod3_thresh(3000);
	//rc = cps_wls_set_tx_fod4_thresh(3000);
	//rc = cps_wls_set_tx_fod5_thresh(3000);
	//rc = cps_wls_set_tx_fod6_thresh(3000);
	//rc = cps_wls_set_tx_fod7_thresh(3000);
	//rc = cps_wls_set_tx_fod_rp0_thresh(20);
	//rc = cps_wls_set_tx_fod_rp1_thresh(40);
	//rc = cps_wls_set_tx_fod_rp2_thresh(60);
	//rc = cps_wls_set_tx_fod_rp3_thresh(80);
	//rc = cps_wls_set_tx_fod_rp4_thresh(100);
	//rc = cps_wls_set_tx_fod_rp5_thresh(120);
	//rc = cps_wls_set_tx_fod_rp6_thresh(140);
	//rc = cps_wls_set_tx_fod_rp7_thresh(160);
	//rc = cps_wls_enable_tx_mode();
	rc = cps_wls_disable_tx_mode();
	rc = cps_wls_send_fsk_packet(data, 2);
	rc = cps_wls_set_fod_para();
	return;
}

static void cps_wls_chrg_remove(struct i2c_client *client)
{
	not_called_api();
	//cps_wls_lock_destroy(chip);
	kfree(chip);
	//return 0;
}

static const struct i2c_device_id cps_wls_charger_id[] = {
	{"cps-wls-charger", 0},
	{},
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
	{ .compatible = "cps,wls-charger-cps4041", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps_wls_charger_driver = {
	.driver = {
		.name	= CPS_WLS_CHRG_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = cps_wls_chrg_of_tbl,
	},
	.probe		= cps_wls_chrg_probe,
	.remove		= cps_wls_chrg_remove,
	.id_table	= cps_wls_charger_id,
};

static int __init cps_wls_driver_init(void)
{
	pr_info("%s\n", __func__);
	return (i2c_add_driver(&cps_wls_charger_driver));
}

late_initcall(cps_wls_driver_init);

static void __exit cps_wls_driver_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&cps_wls_charger_driver);

}

module_exit(cps_wls_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("jian.deng@convenientpower.com");
MODULE_DESCRIPTION("cps_wls_charger driver");
MODULE_ALIAS("i2c:cps_wls_charger");

