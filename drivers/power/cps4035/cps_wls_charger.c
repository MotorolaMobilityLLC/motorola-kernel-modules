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

#include <cps_wls_charger.h>
#define BOOTLOADER_FILE_NAME "/data/misc/cps/bootloader.hex"
#define FIRMWARE_FILE_NAME   "/data/misc/cps/firmware.hex"

#define CPS_WLS_L_CHRG_DRV_NAME "cps-wls-charger-L"
#define CPS_WLS_H_CHRG_DRV_NAME "cps-wls-charger-h"

#define CPS_WLS_CHRG_PSY_NAME "wireless"
struct cps_wls_chrg_chip *chip = NULL;
struct cps_wls_chrg_chip *chip_h = NULL;

/*define cps rx reg enum*/
typedef enum
{
    CPS_RX_REG_EPT_VAL,
    CPS_RX_REG_POWER_SET,
    CPS_RX_REG_VOUT_SET,
    CPS_RX_REG_OCP_TH,
    CPS_RX_REG_OVP_TH,
    CPS_RX_REG_HTP_TH,
    CPS_RX_REG_DUMY_LOAD,
    CPS_RX_REG_DROP_MIN,
    CPS_RX_REG_DROP_MAX,
    CPS_RX_REG_DROP_MIN_CUR,
    CPS_RX_REG_DROP_MAX_CUR,
    CPS_RX_REG_SS_VAL,
    CPS_RX_REG_CE_VAL,
    CPS_RX_REG_RP_VAL,
    CPS_RX_REG_FOP_VAL,
    CPS_RX_REG_NEGO_POWER,
    CPS_RX_REG_NEGO_PRO,
    CPS_RX_REG_ADC_VRECT,
    CPS_RX_REG_ADC_IOUT,
    CPS_RX_REG_ADC_VOUT,
    CPS_RX_REG_ADC_DIE_TMP,
    CPS_RX_REG_MAX
}cps_rx_reg_e;

/*define cps tx reg enum*/
typedef enum
{
    CPS_TX_REG_OCP_TH,
    CPS_TX_REG_UVP_TH,
    CPS_TX_REG_OVP_TH,
    CPS_TX_REG_FOP_MIN,
    CPS_TX_REG_FOP_MAX,
    CPS_TX_REG_PING_FREQ,
    CPS_TX_REG_PING_DUTY,
    CPS_TX_REG_PING_OCP_TH,
    CPS_TX_REG_PING_TIME,
    CPS_TX_REG_PING_INTERVAL,
    CPS_TX_REG_FOD_I_TH,
    CPS_TX_REG_FOD_II_TH,
    CPS_TX_REG_FOD_RP_TH,
    CPS_TX_REG_FUNC_EN,
    CPS_TX_REG_MAX_CUR,
    CPS_TX_REG_FOP_VAL,
    CPS_TX_REG_ADC_VIN,
    CPS_TX_REG_ADC_VRECT,
    CPS_TX_REG_ADC_I_IN,
    CPS_TX_REG_CE_VAL,
    CPS_TX_REG_RP_VAL,
    CPS_TX_REG_EPT_RSN,
    CPS_TX_REG_ADC_DIE_TEMP,
    CPS_TX_REG_EPT_CODE,
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
    CPS_TX_REG_MAX
}cps_tx_reg_e;

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

#define RX_REG_FOD_CUR_0        0x1F06
#define RX_REG_FOD_CUR_1        0x1F07
#define RX_REG_FOD_CUR_2        0x1F08
#define RX_REG_FOD_CUR_3        0x1F09
#define RX_REG_FOD_CUR_4        0x1F0A
#define RX_REG_FOD_CUR_5        0x1F0B
#define RX_REG_FOD_CUR_6        0x1F0C
#define RX_REG_FOD_C0_GAIN      0x1F0D
#define RX_REG_FOD_C0_OFFSET    0x1F0E
#define RX_REG_FOD_C1_GAIN      0x1F0F
#define RX_REG_FOD_C1_OFFSET    0x1F10
#define RX_REG_FOD_C2_GAIN      0x1F11
#define RX_REG_FOD_C2_OFFSET    0x1F12
#define RX_REG_FOD_C3_GAIN      0x1F13
#define RX_REG_FOD_C3_OFFSET    0x1F14
#define RX_REG_FOD_C4_GAIN      0x1F15
#define RX_REG_FOD_C4_OFFSET    0x1F16
#define RX_REG_FOD_C5_GAIN      0x1F17
#define RX_REG_FOD_C5_OFFSET    0x1F18
#define RX_REG_FOD_C6_GAIN      0x1F19
#define RX_REG_FOD_C6_OFFSET    0x1F1A
#define RX_REG_FOD_C7_GAIN      0x1F1B
#define RX_REG_FOD_C7_OFFSET    0x1F1C

typedef struct
{
    uint16_t     reg_name;
    uint16_t     reg_bytes_len;
    uint32_t     reg_addr;
}cps_reg_s;

cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
    /* reg name               bytes number      reg address          */
    {CPS_COMM_REG_CHIP_ID,         4,              0x1D00},
    {CPS_COMM_REG_FW_VER,          4,              0x1D10},
    {CPS_COMM_REG_SYS_MODE,        1,              0x1D14},
    {CPS_COMM_REG_INT_EN,          2,              0x1D40},
    {CPS_COMM_REG_INT_FLAG,        2,              0x1D42},
    {CPS_COMM_REG_INT_CLR,         2,              0x1D44},
    {CPS_COMM_REG_CMD,             2,              0x1D46},
};

cps_reg_s cps_rx_reg[CPS_RX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_RX_REG_EPT_VAL,         1,              0x1F00},
    {CPS_RX_REG_POWER_SET,       1,              0x1F02},
    {CPS_RX_REG_VOUT_SET,        2,              0x1F40},
    {CPS_RX_REG_OCP_TH,          2,              0x1F46},
    {CPS_RX_REG_OVP_TH,          1,              0x1F48},
    {CPS_RX_REG_HTP_TH,          1,              0x1F49},
    {CPS_RX_REG_DUMY_LOAD,       1,              0x1F54},
    {CPS_RX_REG_DROP_MIN,        2,              0x1F58},
    {CPS_RX_REG_DROP_MAX,        2,              0x1F5A},
    {CPS_RX_REG_DROP_MIN_CUR,    2,              0x1F5C},
    {CPS_RX_REG_DROP_MAX_CUR,    2,              0x1F5E},
    {CPS_RX_REG_SS_VAL,          1,              0x1F84},
    {CPS_RX_REG_CE_VAL,          1,              0x1F86},
    {CPS_RX_REG_RP_VAL,          2,              0x1F88},
    {CPS_RX_REG_FOP_VAL,         2,              0x1F8A},
    {CPS_RX_REG_NEGO_POWER,      1,              0x1F90},
    {CPS_RX_REG_NEGO_PRO,        1,              0x1F91},
    {CPS_RX_REG_ADC_VRECT,       2,              0x1F94},
    {CPS_RX_REG_ADC_IOUT,        2,              0x1F96},
    {CPS_RX_REG_ADC_VOUT,        2,              0x1F98},
    {CPS_RX_REG_ADC_DIE_TMP,     2,              0x1F9A},
};

cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_TX_REG_PPP_HEADER,      1,              0x1D82},
    {CPS_TX_REG_PPP_COMMAND,     1,              0x1D83},
    {CPS_TX_REG_PPP_DATA0,       1,              0x1D84},
    {CPS_TX_REG_PPP_DATA1,       1,              0x1D85},
    {CPS_TX_REG_PPP_DATA2,       1,              0x1D86},
    {CPS_TX_REG_PPP_DATA3,       1,              0x1D87},
    {CPS_TX_REG_PPP_DATA4,       1,              0x1D88},
    {CPS_TX_REG_PPP_DATA5,       1,              0x1D89},
    {CPS_TX_REG_PPP_DATA6,       1,              0x1D8A},
    {CPS_TX_REG_BC_HEADER,       1,              0x1DC2},
    {CPS_TX_REG_BC_COMMAND,      1,              0x1DC3},
    {CPS_TX_REG_BC_DATA0,        1,              0x1DC4},
    {CPS_TX_REG_BC_DATA1,        1,              0x1DC5},
    {CPS_TX_REG_BC_DATA2,        1,              0x1DC6},
    {CPS_TX_REG_BC_DATA3,        1,              0x1DC7},
    {CPS_TX_REG_BC_DATA4,        1,              0x1DC8},
    {CPS_TX_REG_BC_DATA5,        1,              0x1DC9},
    {CPS_TX_REG_BC_DATA6,        1,              0x1DCA},
    {CPS_TX_REG_OCP_TH,          2,              0x1E42},
    {CPS_TX_REG_UVP_TH,          2,              0x1E44},
    {CPS_TX_REG_OVP_TH,          2,              0x1E46},
    {CPS_TX_REG_FOP_MIN,         1,              0x1E48},
    {CPS_TX_REG_FOP_MAX,         1,              0x1E49},
    {CPS_TX_REG_PING_FREQ,       1,              0x1E4A},
    {CPS_TX_REG_PING_DUTY,       1,              0x1E4B},
    {CPS_TX_REG_PING_OCP_TH,     2,              0x1E4E},
    {CPS_TX_REG_PING_TIME,       2,              0x1E52},
    {CPS_TX_REG_PING_INTERVAL,   2,              0x1E54},
    {CPS_TX_REG_FOD_I_TH,        2,              0x1E58},
    {CPS_TX_REG_FOD_II_TH,       2,              0x1E5A},
    {CPS_TX_REG_FOD_RP_TH,       2,              0x1E5C},
    {CPS_TX_REG_FUNC_EN,         1,              0x1E5E},
    {CPS_TX_REG_MAX_CUR,         2,              0x1E60},
    {CPS_TX_REG_FOP_VAL,         2,              0x1E84},
    {CPS_TX_REG_ADC_VIN,         2,              0x1E86},
    {CPS_TX_REG_ADC_VRECT,       2,              0x1E88},
    {CPS_TX_REG_ADC_I_IN,        2,              0x1E8A},
    {CPS_TX_REG_CE_VAL,          1,              0x1E8D},
    {CPS_TX_REG_RP_VAL,          1,              0x1E8E},
    {CPS_TX_REG_EPT_RSN,         2,              0x1E90},
    {CPS_TX_REG_ADC_DIE_TEMP,    2,              0x1E94},
    {CPS_TX_REG_EPT_CODE,        1,              0x1E96},
};

//-------------------I2C APT start--------------------
static const struct regmap_config cps4035H_regmap_config = {
    .reg_bits = 16,
    .val_bits = 16,
};

static const struct regmap_config cps4035L_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static u8 CPS4035_BOOTLOADER[0x800] = {
	// CPS4035_BL_V1.2_CRC_6923
	0x70, 0x1E, 0x00, 0x20, 0xD9, 0x01, 0x00, 0x20,
	0x51, 0x01, 0x00, 0x20, 0xE9, 0x01, 0x00, 0x20,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x55, 0x01, 0x00, 0x20,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x57, 0x01, 0x00, 0x20, 0x59, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x5B, 0x01, 0x00, 0x20, 0x5B, 0x01, 0x00, 0x20,
	0x1F, 0xB5, 0x1F, 0xBD, 0x01, 0x02, 0x00, 0x00,
	0xC4, 0x04, 0x00, 0x00, 0x00, 0xF0, 0x02, 0xF8,
	0x00, 0xF0, 0x2E, 0xF8, 0x0C, 0xA0, 0x30, 0xC8,
	0x08, 0x38, 0x24, 0x18, 0x2D, 0x18, 0xA2, 0x46,
	0x67, 0x1E, 0xAB, 0x46, 0x54, 0x46, 0x5D, 0x46,
	0xAC, 0x42, 0x01, 0xD1, 0x00, 0xF0, 0x20, 0xF8,
	0x7E, 0x46, 0x0F, 0x3E, 0x0F, 0xCC, 0xB6, 0x46,
	0x01, 0x26, 0x33, 0x42, 0x00, 0xD0, 0xFB, 0x1A,
	0xA2, 0x46, 0xAB, 0x46, 0x33, 0x43, 0x18, 0x47,
	0xA8, 0x03, 0x00, 0x00, 0xB8, 0x03, 0x00, 0x00,
	0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26,
	0x10, 0x3A, 0x01, 0xD3, 0x78, 0xC1, 0xFB, 0xD8,
	0x52, 0x07, 0x00, 0xD3, 0x30, 0xC1, 0x00, 0xD5,
	0x0B, 0x60, 0x70, 0x47, 0x10, 0xB5, 0x10, 0xBD,
	0x00, 0xF0, 0x26, 0xF8, 0x11, 0x46, 0xFF, 0xF7,
	0xC3, 0xFF, 0x00, 0xF0, 0x25, 0xF9, 0x00, 0xF0,
	0x3E, 0xF8, 0x03, 0xB4, 0xFF, 0xF7, 0xF2, 0xFF,
	0x03, 0xBC, 0x00, 0xF0, 0x02, 0xF9, 0x00, 0x00,
	0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7,
	0xFE, 0xE7, 0xFE, 0xE7, 0x02, 0x48, 0x03, 0x49,
	0x03, 0x4A, 0x04, 0x4B, 0x70, 0x47, 0x00, 0x00,
	0x70, 0x19, 0x00, 0x20, 0x70, 0x1E, 0x00, 0x20,
	0x70, 0x1A, 0x00, 0x20, 0x70, 0x1A, 0x00, 0x20,
	0x70, 0x47, 0x70, 0x47, 0x70, 0x47, 0x70, 0x47,
	0x75, 0x46, 0x00, 0xF0, 0x25, 0xF8, 0xAE, 0x46,
	0x05, 0x00, 0x69, 0x46, 0x53, 0x46, 0xC0, 0x08,
	0xC0, 0x00, 0x85, 0x46, 0x18, 0xB0, 0x20, 0xB5,
	0xFF, 0xF7, 0xE0, 0xFF, 0x60, 0xBC, 0x00, 0x27,
	0x49, 0x08, 0xB6, 0x46, 0x00, 0x26, 0xC0, 0xC5,
	0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5,
	0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5, 0x40, 0x3D,
	0x49, 0x00, 0x8D, 0x46, 0x70, 0x47, 0x10, 0xB5,
	0x04, 0x46, 0xC0, 0x46, 0xC0, 0x46, 0x20, 0x46,
	0xFF, 0xF7, 0xBB, 0xFF, 0x10, 0xBD, 0x00, 0x00,
	0x00, 0x48, 0x70, 0x47, 0x10, 0x19, 0x00, 0x20,
	0x01, 0x48, 0x80, 0x47, 0x01, 0x48, 0x00, 0x47,
	0x51, 0x03, 0x00, 0x20, 0xCD, 0x00, 0x00, 0x20,
	0x02, 0x48, 0x04, 0x22, 0x41, 0x68, 0x51, 0x40,
	0x41, 0x60, 0xFB, 0xE7, 0x00, 0x00, 0x01, 0x40,
	0x4B, 0x4E, 0x00, 0x25, 0x4B, 0x48, 0x01, 0x68,
	0x00, 0x29, 0xFB, 0xD0, 0x00, 0x22, 0x02, 0x60,
	0x49, 0x48, 0x14, 0x46, 0x01, 0x23, 0x02, 0x68,
	0x5B, 0x02, 0x9A, 0x42, 0x01, 0xDD, 0x40, 0x20,
	0x84, 0xE0, 0x66, 0x22, 0x32, 0x60, 0x90, 0x29,
	0x24, 0xD0, 0x06, 0xDC, 0x10, 0x29, 0x47, 0xD0,
	0x20, 0x29, 0x5B, 0xD0, 0x80, 0x29, 0x04, 0xD1,
	0x33, 0xE0, 0xB0, 0x29, 0x04, 0xD0, 0xC0, 0x29,
	0x71, 0xD0, 0x40, 0x20, 0x30, 0x60, 0xEA, 0xE7,
	0x3C, 0x48, 0x84, 0x68, 0x80, 0x14, 0x84, 0x42,
	0x02, 0xDD, 0x3B, 0x48, 0x04, 0x60, 0x63, 0xE0,
	0x01, 0x20, 0x21, 0x1F, 0x40, 0x07, 0x00, 0xF0,
	0x7D, 0xF8, 0x37, 0x49, 0x37, 0x4A, 0x08, 0x60,
	0xA2, 0x18, 0xD2, 0x6F, 0x4A, 0x60, 0x90, 0x42,
	0x56, 0xD1, 0x5A, 0xE0, 0xC0, 0x20, 0x84, 0x68,
	0x00, 0x2C, 0x51, 0xDB, 0x09, 0x20, 0xC0, 0x02,
	0x84, 0x42, 0x4D, 0xDC, 0xC8, 0x2C, 0x4B, 0xDB,
	0x21, 0x1F, 0x00, 0x20, 0x00, 0xF0, 0x66, 0xF8,
	0x01, 0x46, 0x2B, 0x48, 0x80, 0x3C, 0x01, 0x60,
	0xE2, 0x6F, 0x42, 0x60, 0x91, 0x42, 0x3F, 0xD1,
	0x43, 0xE0, 0x00, 0x22, 0x28, 0x49, 0x29, 0x48,
	0x00, 0xF0, 0xAE, 0xF8, 0x07, 0x46, 0x27, 0x48,
	0x00, 0x22, 0x27, 0x49, 0x00, 0x1D, 0x00, 0xF0,
	0xA7, 0xF8, 0x38, 0x43, 0x04, 0x46, 0x2B, 0xE0,
	0x09, 0x21, 0xC9, 0x02, 0x8D, 0x42, 0x2B, 0xDA,
	0x00, 0x27, 0x09, 0xE0, 0x1C, 0x48, 0xB9, 0x00,
	0x41, 0x58, 0x01, 0x22, 0x28, 0x46, 0x00, 0xF0,
	0x97, 0xF8, 0x04, 0x43, 0x2D, 0x1D, 0x7F, 0x1C,
	0x15, 0x48, 0x00, 0x68, 0x87, 0x42, 0xF1, 0xDB,
	0x15, 0x48, 0x14, 0xE0, 0x09, 0x21, 0xC9, 0x02,
	0x8D, 0x42, 0x15, 0xDA, 0x00, 0x27, 0x09, 0xE0,
	0x16, 0x48, 0xB9, 0x00, 0x41, 0x58, 0x01, 0x22,
	0x28, 0x46, 0x00, 0xF0, 0x81, 0xF8, 0x04, 0x43,
	0x2D, 0x1D, 0x7F, 0x1C, 0x0A, 0x48, 0x00, 0x68,
	0x87, 0x42, 0xF1, 0xDB, 0x0F, 0x48, 0x05, 0x60,
	0x60, 0x1C, 0x80, 0xD0, 0x00, 0x2C, 0x04, 0xD0,
	0xAA, 0x20, 0x03, 0xE0, 0xFF, 0xE7, 0x06, 0x48,
	0x05, 0x68, 0x55, 0x20, 0x30, 0x60, 0x69, 0xE7,
	0x04, 0x18, 0x00, 0x20, 0x00, 0x18, 0x00, 0x20,
	0x08, 0x18, 0x00, 0x20, 0xC0, 0x00, 0x00, 0x20,
	0x00, 0x08, 0x00, 0x20, 0x80, 0xFF, 0xFF, 0x1F,
	0x85, 0xE1, 0x24, 0x57, 0xF8, 0x47, 0x00, 0x00,
	0x4E, 0x87, 0x55, 0x74, 0x00, 0x10, 0x00, 0x20,
	0x70, 0x47, 0xFE, 0xE7, 0x70, 0xB5, 0x04, 0x46,
	0x00, 0x20, 0x0A, 0x4D, 0x02, 0x46, 0x0E, 0xE0,
	0xA3, 0x5C, 0x1B, 0x02, 0x58, 0x40, 0x00, 0x23,
	0x06, 0x04, 0x02, 0xD5, 0x40, 0x00, 0x68, 0x40,
	0x00, 0xE0, 0x40, 0x00, 0x5B, 0x1C, 0x80, 0xB2,
	0x08, 0x2B, 0xF5, 0xDB, 0x52, 0x1C, 0x8A, 0x42,
	0xEE, 0xDB, 0x70, 0xBD, 0x21, 0x10, 0x00, 0x00,
	0x14, 0x49, 0x13, 0x48, 0x08, 0x60, 0x14, 0x49,
	0x40, 0x1E, 0x08, 0x60, 0x13, 0x48, 0x00, 0x24,
	0x04, 0x60, 0x13, 0x48, 0x04, 0x60, 0x00, 0xF0,
	0x6F, 0xF8, 0x12, 0x48, 0x04, 0x60, 0x12, 0x48,
	0x04, 0x83, 0x10, 0x48, 0x80, 0x30, 0x04, 0x62,
	0x10, 0x49, 0x04, 0x20, 0x48, 0x60, 0x08, 0x61,
	0xC8, 0x61, 0x47, 0x21, 0x09, 0x02, 0x0E, 0x4B,
	0x00, 0x20, 0x82, 0x00, 0x8C, 0x58, 0x40, 0x1C,
	0x9C, 0x50, 0x40, 0x28, 0xF9, 0xDB, 0x0B, 0x48,
	0x80, 0x69, 0xFF, 0xF7, 0x11, 0xFF, 0x00, 0x00,
	0x01, 0x02, 0x00, 0x00, 0x0C, 0x18, 0x00, 0x20,
	0x08, 0x18, 0x00, 0x20, 0x00, 0x18, 0x00, 0x20,
	0x04, 0x18, 0x00, 0x20, 0x00, 0x00, 0x04, 0x40,
	0xC0, 0x21, 0x01, 0x40, 0x00, 0x00, 0x01, 0x40,
	0x10, 0x18, 0x00, 0x20, 0x00, 0x20, 0x01, 0x40,
	0x18, 0xB5, 0x09, 0x24, 0x1B, 0x4B, 0xE4, 0x02,
	0xA0, 0x42, 0x29, 0xDA, 0x01, 0x2A, 0x0C, 0xD1,
	0x47, 0x22, 0x12, 0x02, 0x90, 0x42, 0x08, 0xDB,
	0x81, 0x1A, 0xCA, 0x17, 0x92, 0x0F, 0x51, 0x18,
	0x89, 0x08, 0x15, 0x4A, 0x89, 0x00, 0x89, 0x18,
	0x09, 0x68, 0x03, 0xC3, 0x0A, 0x20, 0x08, 0x3B,
	0x00, 0x90, 0x41, 0x1E, 0x00, 0x91, 0x01, 0xD3,
	0x08, 0x46, 0xFA, 0xE7, 0x08, 0x22, 0x32, 0x21,
	0x1A, 0x61, 0x00, 0x91, 0x00, 0x98, 0x44, 0x1E,
	0x00, 0x94, 0xFB, 0xD2, 0x0B, 0x48, 0x5A, 0x61,
	0x00, 0x90, 0xD8, 0x68, 0x82, 0x06, 0x05, 0xD5,
	0x00, 0x98, 0x42, 0x1E, 0x00, 0x92, 0xF8, 0xD2,
	0x01, 0x20, 0x18, 0xBD, 0x40, 0x06, 0xC0, 0x0F,
	0x00, 0x91, 0x4A, 0x1E, 0x00, 0x92, 0xF8, 0xD3,
	0x11, 0x46, 0xFA, 0xE7, 0x00, 0x20, 0x01, 0x40,
	0x10, 0x18, 0x00, 0x20, 0x50, 0xC3, 0x00, 0x00,
	0x30, 0xB5, 0x0A, 0x49, 0x8F, 0x23, 0x07, 0x4A,
	0x07, 0x48, 0xDB, 0x01, 0xC8, 0x61, 0x00, 0x20,
	0x84, 0x00, 0x1D, 0x59, 0x40, 0x1C, 0x15, 0x51,
	0x0E, 0x28, 0xF9, 0xDB, 0x00, 0x20, 0xC8, 0x61,
	0x30, 0xBD, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x40,
	0x50, 0x12, 0x00, 0x00, 0x40, 0xE8, 0x00, 0x40,
	0xC0, 0x04, 0x00, 0x20, 0x00, 0x08, 0x00, 0x20,
	0x70, 0x16, 0x00, 0x00, 0x10, 0x01, 0x00, 0x20,
	0x23, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*
return -1 means fail, 0 means success
*/
static int cps_wls_h_write_reg(int reg, int value)
{
    int ret;
    int tmp;

    tmp = ((value&0xff) << 8) | ((value>>8) & 0xff);
    mutex_lock(&chip_h->i2c_lock);
    ret = regmap_write(chip_h->regmap, reg, tmp);
    mutex_unlock(&chip_h->i2c_lock);

    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }

    return CPS_WLS_SUCCESS;
}

static int cps_wls_l_write_reg(int reg, int value)
{
    int ret;
    
    mutex_lock(&chip->i2c_lock);
    ret = regmap_write(chip->regmap, reg, value);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0){   
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
    int ret;
    int value;
    
    mutex_lock(&chip->i2c_lock);
    ret = regmap_read(chip->regmap, reg, &value);
    mutex_unlock(&chip->i2c_lock);
    
    if (ret < 0){   
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
    int i=0, tmp=0;
    for(i = 0; i < byte_len; i++)
    {
        tmp = (value >> (i*8))&0xff;
        if(cps_wls_l_write_reg((reg&0xffff) + i, tmp) == CPS_WLS_FAIL)     goto write_fail;
    }
    return CPS_WLS_SUCCESS;

write_fail:
    return CPS_WLS_FAIL;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read_reg(int reg, int byte_len)
{
    int i=0,tmp=0,read_date=0;
    for(i = 0; i < byte_len; i++)
    {
        tmp = cps_wls_l_read_reg((reg&0xffff) + i);
        if(tmp == CPS_WLS_FAIL)    goto read_fail;
        read_date |= (tmp << (8*i)); 
    }
    return read_date;
    
    
read_fail:
    return CPS_WLS_FAIL;
}

//*****************************for program************************

static int cps_wls_program_sram(int addr, u8 *date, int len)
{
    int ret;
    cps_wls_h_write_reg(REG_WRITE_MODE, PROGRAM_WRITE_MODE);//set wirte mode :byte write mode
    cps_wls_h_write_reg(REG_HIGH_ADDR,  (addr >> 16)&0xffff);//set high 16 bit addr

    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap, addr & 0xffff, date, len);
    mutex_unlock(&chip->i2c_lock);
    
    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return CPS_WLS_SUCCESS;
}

static int cps_wls_write_word(int addr, int value)
{
    int ret;
    u8 write_date[4];

    write_date[0] = value & 0xff;
    write_date[1] = (value >> 8) & 0xff;
    write_date[2] = (value >> 16) & 0xff;
    write_date[3] = (value >> 24) & 0xff;

    cps_wls_h_write_reg(REG_HIGH_ADDR,  (addr >> 16)&0xffff);//set high 16 bit addr
    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap, addr & 0xffff, write_date, 4);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return CPS_WLS_SUCCESS;
}

static int cps_wls_read_word(int addr)
{
    int ret;
    u8 read_date[4];
    mutex_lock(&chip->i2c_lock);
    cps_wls_h_write_reg(REG_HIGH_ADDR,  (addr >> 16)&0xffff);//set high 16 bit addr
    
    ret = regmap_raw_read(chip->regmap, addr & 0xffff, read_date, 4);
    mutex_unlock(&chip->i2c_lock);
    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return *(int *)read_date;
}

static int cps_wls_program_cmd_send(int cmd)
{
    return cps_wls_write_word(ADDR_CMD, cmd);
}

static int cps_wls_program_wait_cmd_done(void)
{
    int ret;
    int wait_time_out = 50;//ms
    while(1)
    {
        ret = cps_wls_read_word(ADDR_FLAG);
        if(ret == CPS_WLS_FAIL)  return CPS_WLS_FAIL;
        wait_time_out--;
        msleep(1);
        if((ret & 0xff) == PASS) 
        {
            break;
        }
        if(wait_time_out < 0)  return CPS_WLS_FAIL;
    }

    return CPS_WLS_SUCCESS;
}

//get crc
uint16_t get_crc(u8 *buf, int len){
    int i,j;

    uint16_t crc_in = 0x0000;
    uint16_t crc_poly = 0x1021;

    for(i=0;i<len;i++)
    {
        crc_in ^= (buf[i]  << 8);
        for(j=0;j<8;j++)
        {
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

//-------------------CPS4035 system interface-------------------
static void cps_wls_write_password()
{
    cps_wls_log(CPS_LOG_DEBG, "[%s] -------write password\n", __func__);
    cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);
    cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
    cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);
}

static int cps_wls_set_cmd(int value)
{    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CMD]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static uint16_t cps_wls_get_cmd(void)
{    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CMD]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_int_flag(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_FLAG]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_int_clr(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_CLR]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_chip_id(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CHIP_ID]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
#if 0
static int cps_wls_get_sys_fw_version(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_VER]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
#endif
static int cps_wls_get_sys_mode(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_SYS_MODE]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

#if 0

//-------------------CPS4035 RX interface-------------------
static int cps_wls_get_rx_ss_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_SS_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_ce_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_CE_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_rp_pkt_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_RP_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_fop_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_FOP_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_ept_code(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_EPT_VAL]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_neg_power(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_NEGO_POWER]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
#endif

#if 0
static int cps_wls_get_rx_neg_pro(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_NEGO_PRO]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
#endif
static int cps_wls_get_rx_vrect(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_VRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_irect(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_IOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_iout(void)
{
    cps_reg_s *cps_reg;
    cps_wls_write_password();
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_IOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_vout(void)
{
    cps_reg_s *cps_reg;
    cps_wls_write_password();
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_VOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_fod_para(void)
{
    uint8_t i;

    const uint8_t FOP_CUR[7] =
    {
        // unit 10mA
        25 * 1,   // C0
        25 * 2,   // C1
        25 * 3,   // C2
        25 * 4,   // C3
        25 * 5,   // C4
        25 * 6,   // C5
        25 * 7,   // C6
    };

    const uint8_t FOP_GAIN_OFFSET[8 * 2] =
    {
        // gain(0.01)     offset(40mW)
        // 5V
        56,                 3,             // C0
        56,                 3,             // C1
        56,                 3,             // C2
        56,                 3,             // C3
        56,                 3,             // C4
        56,                 3,             // C5
        56,                 3,             // C6
        56,                 3,             // C7
    };

    for(i = 0; i < 7; i++)
    {
        if(cps_wls_write_reg((int)(RX_REG_FOD_CUR_0 + i), FOP_CUR[i], 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }
    
    for(i = 0; i < 16; i++)
    {
        if(cps_wls_write_reg((int)(RX_REG_FOD_C0_GAIN + i), FOP_GAIN_OFFSET[i], 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    return CPS_WLS_SUCCESS;
}
#if 0
static int cps_wls_get_rx_die_tmp(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_DIE_TMP]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_vout_target(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_VOUT_SET]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_ocp_threshold(int value)
{
    if(value < 0 || value > 2800) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_OCP_TH]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_ovp_threshold(int value)
{
    if(value < 0 || value > 15) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_OVP_TH]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_htp_protect(int enable, int value)
{
    if(value < 0 || value > 4) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_HTP_TH]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_send_command(uint16_t ap_command)
{
    chip->command_flag = 0;
    if(cps_wls_set_cmd(ap_command)!= CPS_WLS_SUCCESS)
    {
        return CPS_WLS_FAIL;
    }

    msleep(10);
    return CPS_WLS_SUCCESS;
}

static int cps_wls_rx_send_ept_packet(ept_reason_e value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_EPT_VAL]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);

    if(CPS_WLS_SUCCESS == cps_wls_write_reg((int)cps_reg.reg_addr, value, (int)cps_reg.reg_bytes_len))
    {
        if(CPS_WLS_SUCCESS == cps_wls_send_command(RX_CMD_SEND_EPT));
        {
            return CPS_WLS_SUCCESS;
        }
    }
    return CPS_WLS_FAIL;
}

static int cps_wls_set_rx_neg_power(int value)
{
    if(value < 0 || value > 30)  return CPS_WLS_FAIL;    
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_POWER_SET]);
    return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}
#endif
//-------------------CPS4035 TX interface-------------------
static int cps_wls_get_tx_i_in(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_I_IN]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_vin(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_VIN]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_vrect(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_VRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power smaller than RP threshold(0x1E5C),FOD ploss trigger threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_thresh_I(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_I_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP threshold(0x1E5C), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_thresh_II(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_II_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP_TH]);
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
    cmd |= TX_CMD_EXIT_TX_MODE;
    return cps_wls_set_cmd(cmd);
}

static int cps_wls_send_fsk_packet(uint8_t *data, uint8_t data_len)
{
    uint16_t cmd;
    uint8_t i;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PPP_HEADER]);


    for(i = 0; i < data_len; i++)
    {
        if(cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    cmd = cps_wls_get_cmd();
    cmd |= TX_CMD_SEND_FSK;
    return cps_wls_set_cmd(cmd);
}

uint8_t cps_wls_get_message_size(uint8_t header)
{
    if(header < 0x20)
    {
        return 1;
    }
    else if(header < 0x80)
    {
        return header / 16;
    }
    else if(header < 0xE0)
    {
        return header / 8 - 8;
    }
    else
    {
        return header / 4 - 36;
    }
}

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

#if 0
static int cps_wls_get_tx_die_tmp(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_DIE_TEMP]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_freq(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ept_code(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_EPT_CODE]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ce_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_CE_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_rp_value(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_RP_VAL]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ocp_threshold(int value)
{
    if(value < 0 || value > 3000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OCP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_uvp_threshold(int value)
{
    if(value < 0 || value > 5000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_UVP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ovp_threshold(int value)
{
    if(value < 3000 || value > 20000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OVP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_fop_min(int value)
{
    if(value < 1050 || value > 1470) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_MIN]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_fop_max(int value)
{
    if(value < 1050 || value > 1470) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOP_MAX]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ping_frequency(int value)
{
    if(value < 1050 || value > 1470) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PING_FREQ]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_huge_metal_threshold(int value)
{
    if(value < 0 || value > 1000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PING_OCP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}
#endif

//------------------------------IRQ Handler-----------------------------------
static int cps_wls_set_int_enable(void)
{
    uint16_t int_en;
    cps_reg_s *cps_reg;
    
    int_en = 0xFFFF;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_EN]);
    
    if(CPS_WLS_FAIL == cps_wls_write_reg((int)cps_reg->reg_addr, int_en, (int)cps_reg->reg_bytes_len))  goto set_int_fail;
    return CPS_WLS_SUCCESS;

set_int_fail:
    return CPS_WLS_FAIL;
}

static int cps_wls_rx_irq_handler(int int_flag)
{
    int rc = 0;

    if (int_flag & RX_INT_POWER_ON)
    {
        //CPS_RX_MODE_ERR = FALSE;
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_POWER_ON");
    }
    if(int_flag & RX_INT_LDO_OFF)
    {
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_LDO_OFF");
    }
    if(int_flag & RX_INT_LDO_ON){
         chip->rx_ldo_on = 1;
         power_supply_changed(chip->wl_psy);
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_LDO_ON");
    }
    if(int_flag & RX_INT_READY){
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_READY");
    }
    if(int_flag & RX_INT_OVP){
          cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OVP");
    }
    if(int_flag & RX_INT_OTP){
          //int_type |= INT_OTP;
          cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OTP");
    }
    if(int_flag & RX_INT_OCP){
          //wls_event_notify(WLS_EVENT_RX_OCP);
          cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OCP");
    }
    if(int_flag & RX_INT_HOCP){
          cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_HOCP");
    }
    if(int_flag & RX_INT_SCP){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_SCP");
    }
    if(int_flag & RX_INT_INHIBIT_HIGH){
          cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_INHIBIT_HIGH");
    }

    return rc;
}

static int cps_wls_tx_irq_handler(int int_flag)
{
    int rc = 0;
    uint8_t data[8] = {0};

    if (int_flag & TX_INT_PING)
    {
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_PING");
    }
    if(int_flag & TX_INT_SSP)
    {
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_SSP");
    }
    if(int_flag & TX_INT_IDP){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_IDP");
    }
    if(int_flag & TX_INT_CFGP){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_CFGP");
    }
    if(int_flag & TX_INT_EPT){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_EPT");
    }
    if(int_flag & TX_INT_RPP_TO){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_RPP_TO");
    }
    if(int_flag & TX_INT_CEP_TO){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_CEP_TO");
    }
    if(int_flag & TX_INT_AC_DET){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_AC_DET");
    }
    if(int_flag & TX_INT_INIT){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_INIT");
    }
    if(int_flag & TX_INT_RP_TYPR_ERR){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_RP_TYPR_ERR");
    }
    if(int_flag & TX_INT_ASK_PKT)
    {
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_ASK_PKT");
        rc = cps_wls_get_ask_packet(data);
    }

    return rc;
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
    int int_flag;
    int int_clr;
    cps_wls_log(CPS_LOG_DEBG, "[%s] IRQ triggered\n", __func__);
    mutex_lock(&chip->irq_lock);
    if(cps_wls_get_chip_id() != 0x4035)
    {
        /*unlock i2c*/
        cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);
        cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
        cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);
        cps_wls_set_int_enable();
        
        cps_wls_log(CPS_LOG_DEBG, "[%s] CPS_I2C_UNLOCK", __func__);
    }
    
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
    if(cps_wls_get_sys_mode() == SYS_MODE_RX)
    {
        cps_wls_rx_irq_handler(int_flag);
    }
    else
    {
        cps_wls_tx_irq_handler(int_flag);
    }

    return IRQ_HANDLED;
}

static irqreturn_t wls_det_irq_handler(int irq, void *dev_id)
{
	struct cps_wls_chrg_chip *chip = dev_id;
	int tx_detected = gpio_get_value(chip->wls_det_int);

	if (tx_detected) {
		cps_wls_log(CPS_LOG_DEBG, "Detected an attach event.\n");
	} else {
		cps_wls_log(CPS_LOG_DEBG, "Detected a detach event.\n");
		chip->rx_ldo_on = false;
		power_supply_changed(chip->wl_psy);
	}

	return IRQ_HANDLED;
}


static  int cps_wls_is_ldo_on()
{
	return (chip->rx_ldo_on && gpio_get_value(chip->wls_det_int));
}

static enum power_supply_property cps_wls_chrg_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_MAX,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_MAX,
    POWER_SUPPLY_PROP_CURRENT_NOW,
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
            val->intval = cps_wls_is_ldo_on();
            break;

        case POWER_SUPPLY_PROP_TYPE:
            val->intval = chip->wl_psd.type;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_MAX:
            val->intval = 12000000;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = cps_wls_get_rx_vout() * 1000;
            break;

        case POWER_SUPPLY_PROP_CURRENT_MAX:
            val->intval = 1250000;
            break;

        case POWER_SUPPLY_PROP_CURRENT_NOW:
            val->intval = cps_wls_get_rx_iout() * 1000;
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
    return 0;
}

static void cps_wls_charger_external_power_changed(struct power_supply *psy)
{
    ;        
}

static int cps_get_fw_revision(uint32_t* fw_revision)
{
	int status = CPS_WLS_SUCCESS;
	cps_reg_s *cps_reg;
	uint32_t fw_major_revision = 0;
	uint32_t fw_minor_revision = 0;
	uint32_t fw_version;

	cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);
	cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
	cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);

	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_VER]);
	fw_version =  cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if(CPS_WLS_SUCCESS != status)
	{
		cps_wls_log(CPS_LOG_ERR, "%s failed",__func__);
	}

	if(CPS_WLS_SUCCESS == status) {
		fw_version = fw_version >> 16;
		fw_minor_revision = (fw_version) & 0xFF;
		fw_major_revision = ((fw_version) >> 8) & 0xFF;
		fw_version =  (fw_major_revision << 16) | (fw_minor_revision);
        chip->wls_fw_version = *fw_revision = fw_version;
	} else
		*fw_revision = 0;


	cps_wls_log(CPS_LOG_ERR, "%s 0x%x, minor 0x%x, major 0x%x",__func__,
		*fw_revision, fw_minor_revision, fw_major_revision);

	return status;
}

static void cps_wls_pm_set_awake(int awake)
{
	if(!chip->cps_wls_wake_lock->active && awake) {
        __pm_stay_awake(chip->cps_wls_wake_lock);
	} else if(chip->cps_wls_wake_lock->active && !awake) {
		__pm_relax(chip->cps_wls_wake_lock);
	}
}

static void cps_wls_set_boost(int val)
{
	/* Assume if we turned the boost on we want to stay awake */
	gpio_set_value(chip->wls_boost_en, val);
	gpio_set_value(chip->wls_switch_en, val);

	if(val) {
		cps_wls_pm_set_awake(1);
	} else {
		cps_wls_pm_set_awake(0);
	}
}

#define CPS_FW_MAJOR_VER_OFFSET		0xc4
#define CPS_FW_MINOR_VER_OFFSET		0xc5
static int wireless_fw_update(bool force)
{
	int  write_count, k;
	int buff0_flag = 0, buff1_flag = 0;
	u32 version;
	u16 maj_ver, min_ver;
	u8 *firmware_buf;
	u8 *p;
	int result;
	int rc;
	u32 fw_revision;
	const struct firmware *fw;

	cps_wls_set_boost(0);
	msleep(20);//20ms
	cps_wls_set_boost(1);
	msleep(100);//100ms

	firmware_buf = kzalloc(0x4800, GFP_KERNEL);  // 18K buffer
	rc = firmware_request_nowarn(&fw, chip->wls_fw_name, chip->dev);
	if (rc) {
		cps_wls_log(CPS_LOG_ERR,"Couldn't get firmware  rc=%d\n", rc);
		goto update_fail;
	}

	maj_ver = be16_to_cpu(*(__le16 *)(fw->data + CPS_FW_MAJOR_VER_OFFSET));
	maj_ver = maj_ver >> 8;
	min_ver = be16_to_cpu(*(__le16 *)(fw->data + CPS_FW_MINOR_VER_OFFSET));
	min_ver = min_ver >> 8;
	cps_wls_log(CPS_LOG_DEBG,"maj_var %#x, min_ver %#x\n", maj_ver, min_ver);
	version = maj_ver << 16 | min_ver;
	if (force)
		version = UINT_MAX;
	cps_wls_log(CPS_LOG_DEBG,"FW size: %zu version: %#x\n", fw->size, version);

	result = cps_get_fw_revision(&fw_revision);
	if(version == fw_revision) {
	    cps_wls_log(CPS_LOG_DEBG,"%s bin version %x same as fw version %x,not need update fw\n",__func__,version,fw_revision);
	    goto free_bug;
	}

	//CPS4035_BL
	//bootloader_buf = CPS4035_BOOTLOADER;
	if(CPS_WLS_FAIL == cps_wls_h_write_reg(REG_PASSWORD, PASSWORD))   goto update_fail;//set password
	if(CPS_WLS_FAIL == cps_wls_h_write_reg(REG_RESET_MCU, CPS_4035_RESET))   goto update_fail;//reset mcu
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40012334, 0x00))    goto update_fail;//enable MTP write
	if(CPS_WLS_FAIL == cps_wls_program_sram(0x20000000, CPS4035_BOOTLOADER, 0x800))  goto update_fail;//program sram
	if(CPS_WLS_FAIL == cps_wls_write_word(0x400400A0, 0x000000ff))    goto update_fail;//remap enable
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40040010, 0x00008000))    goto update_fail;//trim disable
	cps_wls_write_word(0x40040070, 0x61A00000);//sys-restart
	cps_wls_log(CPS_LOG_DEBG, " ---- system restart\n");
	msleep(100);

	if(CPS_WLS_FAIL == cps_wls_h_write_reg(REG_PASSWORD, PASSWORD))   goto update_fail;//set password
	cps_wls_h_write_reg(REG_WRITE_MODE, PROGRAM_WRITE_MODE);
	//=========================================================
	// cali bootloader code
	//=========================================================
	cps_wls_program_cmd_send(CACL_CRC_TEST);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS)
	{
	    cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader crc fail\n", __func__);
	    goto update_fail;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load bootloader successful\n", __func__);

	//=========================================================
	// LOAD firmware to MTP
	//=========================================================
	memset(firmware_buf, 0, 0x4800);
	memcpy(firmware_buf,fw->data,fw->size);
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load fw size %x\n", __func__,(int)fw->size);
	//firmware_buf = fw->data;
	//set start addr
	cps_wls_write_word(ADDR_BUFFER0, 0x0000);
	cps_wls_program_cmd_send(PGM_ADDR_SET);
	cps_wls_program_wait_cmd_done();

	//set write buffer size   defalt 64 word
	cps_wls_write_word(ADDR_BUF_SIZE, CPS_PROGRAM_BUFFER_SIZE);
	write_count = 0;
start_write_app_code:
	p = firmware_buf;
	write_count++;
	//goto program mode
	cps_wls_write_word(0x40012120, 0x1250);
	cps_wls_write_word(0x40012ee8, 0xD148);

	for (k = 0; k < (18 * 1024 / 4) / CPS_PROGRAM_BUFFER_SIZE; k++)
	{
	    if (buff0_flag == 0)
	    {
	        //write buf0
	        cps_wls_program_sram(ADDR_BUFFER0, p, CPS_PROGRAM_BUFFER_SIZE*4);
	        p = p + CPS_PROGRAM_BUFFER_SIZE*4;
	        if (buff1_flag == 1)
	        {
	            //wait finish
	            cps_wls_program_wait_cmd_done();
	            buff1_flag = 0;
	        }

	        //write buff 0 CMD
	        cps_wls_program_cmd_send(PGM_BUFFER0);
	        buff0_flag = 1;
	        continue;
	    }
	    if (buff1_flag == 0)
	    {
	        //write buf1
	        cps_wls_program_sram(ADDR_BUFFER1, p, CPS_PROGRAM_BUFFER_SIZE*4);
	        p = p + CPS_PROGRAM_BUFFER_SIZE*4;
	        if (buff0_flag == 1)
	        {
                //wait finish
                cps_wls_program_wait_cmd_done();
                buff0_flag = 0;
	        }

	        //write buff 0 CMD
	        cps_wls_program_cmd_send(PGM_BUFFER1);
	        buff1_flag = 1;
	        continue;
	    }
	}
	if (buff0_flag == 1)
	{
	    //wait finish
	    cps_wls_program_wait_cmd_done();
	    buff0_flag = 0;
	}

	if (buff1_flag == 1)
	{
         //wait finish
        cps_wls_program_wait_cmd_done();
        buff1_flag = 0;
	}

	//exit program mode
	cps_wls_write_word(0x40012120, 0x0000);
	cps_wls_write_word(0x40012ee8, 0x0000);


	//=========================================================
	// cali app crc
	//=========================================================
	cps_wls_program_cmd_send(CACL_CRC_APP);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS)
	{
        cps_wls_log(CPS_LOG_ERR, "[%s] ---- TEST APP CRC fail", __func__);
        if(write_count < 3) goto start_write_app_code;
        else goto update_fail;
	}

	//=========================================================
	// write mcu&trim flag
	//=========================================================
	//goto program mode
	cps_wls_write_word(0x40012120, 0x1250);
	cps_wls_write_word(0x40012ee8, 0xD148);
	cps_wls_program_cmd_send(PGM_WR_FLAG);
	cps_wls_program_wait_cmd_done();
	//exit program mode
	cps_wls_write_word(0x40012120, 0x0000);
	cps_wls_write_word(0x40012ee8, 0x0000);

	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- Program successful\n", __func__);

free_bug:
	cps_wls_set_boost(0);//disable power, after FW updating, need a power reset
	msleep(20);//20ms
	kfree(firmware_buf);
	release_firmware(fw);
	return CPS_WLS_SUCCESS;

update_fail:
    cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);
    return CPS_WLS_FAIL;
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

	int rc;
	uint32_t  fw_version;
	rc = cps_get_fw_revision(&fw_version);

	return sprintf(buf, "%08x\n", fw_version);
}

static DEVICE_ATTR(wireless_fw_version, 0664, wireless_fw_version_show, NULL);

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
static DEVICE_ATTR(wireless_fw_force_update, 0664, NULL, wireless_fw_force_update_store);

static ssize_t wireless_fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(false);
	if (rc < 0)
		return rc;

	return count;
}
static DEVICE_ATTR(wireless_fw_update, 0664, NULL, wireless_fw_update_store);

static ssize_t store_write_password(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp;
    tmp = simple_strtoul(buf, NULL, 0);
    
    if(tmp != 0)
    {
        cps_wls_log(CPS_LOG_DEBG, "[%s] -------write password\n", __func__);
        cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);    
        cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
        cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);
    }
    
    return count;
}
static DEVICE_ATTR(write_password, 0664, NULL, store_write_password);

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

	cps_wls_set_boost(tx_mode);
	if (tx_mode) {
	    cps_wls_enable_tx_mode();
	} else {
	    cps_wls_disable_tx_mode();
	}

	chip->tx_mode = tx_mode;
	if (chip->wl_psy)
		sysfs_notify(&chip->wl_psy->dev.parent->kobj, NULL, "tx_mode");

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

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", chip->rx_ldo_on);
}

static DEVICE_ATTR(rx_connected, S_IRUGO,
		rx_connected_show,
		NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_curr_max;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_curr_max);
	if (r) {
		cps_wls_log(CPS_LOG_ERR, "Invalid TCMD = %lu\n", wls_curr_max);
		return -EINVAL;
	}

	chip->wls_curr_max = wls_curr_max;
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

	return sprintf(buf, "%d\n", chip->wls_curr_max);
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

static void cps_wls_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
//-----------------------program---------------------
    device_create_file(dev, &dev_attr_wireless_fw_version);
    device_create_file(dev, &dev_attr_wireless_fw_update);
    device_create_file(dev, &dev_attr_wireless_fw_force_update);
//-----------------------write password--------------
    device_create_file(dev, &dev_attr_write_password);

//-----------------------RX--------------------------
    device_create_file(dev, &dev_attr_get_rx_irect);
    device_create_file(dev, &dev_attr_get_rx_vrect);
    device_create_file(dev, &dev_attr_get_rx_vout);
//-----------------------TX--------------------------
    device_create_file(dev, &dev_attr_get_tx_vin);
    device_create_file(dev, &dev_attr_get_tx_iin);
    device_create_file(dev, &dev_attr_get_tx_vrect);

    device_create_file(dev, &dev_attr_tx_mode);
    device_create_file(dev, &dev_attr_rx_connected);
    device_create_file(dev, &dev_attr_wls_input_current_limit);
    device_create_file(dev, &dev_attr_folio_mode);
}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
    struct device_node *node = chip->dev->of_node;

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

   chip->wls_switch_en = of_get_named_gpio(node, "cps_wls_switch_en", 0);
    if(!gpio_is_valid(chip->wls_switch_en))
        return -EINVAL;

   chip->wls_boost_en = of_get_named_gpio(node, "cps_wls_boost_en", 0);
    if(!gpio_is_valid(chip->wls_boost_en))
        return -EINVAL;

    of_property_read_string(node, "wireless-fw-name", &chip->wls_fw_name);
    cps_wls_log(CPS_LOG_ERR, "[%s]  wls_charge_int %d wls_det_int %d wls_switch_en %d wls_boost_en %d wls_fw_name: %s\n",
             __func__, chip->wls_charge_int, chip->wls_det_int, chip->wls_switch_en, chip->wls_boost_en,chip->wls_fw_name);
    return 0;
}

static int cps_wls_gpio_request(struct cps_wls_chrg_chip *chip)
{
    int ret =0;

	ret = devm_gpio_request_one(chip->dev, chip->wls_charge_int,
				  GPIOF_IN, "cps4035_ap_int");
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
				  GPIOF_IN, "cps4035_wls_det_int");
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"Failed to request det_int gpio, ret:%d", ret);
		return ret;
	}
	chip->wls_det_irq = gpio_to_irq(chip->wls_det_int);
	if (chip->wls_det_irq < 0) {
		cps_wls_log(CPS_LOG_ERR,"failed get det irq num %d", chip->wls_det_irq);
		return -EINVAL;
	}


	ret = devm_gpio_request_one(chip->dev, chip->wls_switch_en,
				  GPIOF_OUT_INIT_LOW, "cps4035_wls_switch_en");
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"Failed to request wls_switch_en gpio, ret:%d", ret);
		return ret;
	}

	ret = devm_gpio_request_one(chip->dev, chip->wls_boost_en,
				  GPIOF_OUT_INIT_LOW, "cps4035_wls_boost_en");
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"Failed to request wls_boost_en gpio, ret:%d", ret);
		return ret;
	}

    return ret;
}


static void cps_wls_lock_work_init(struct cps_wls_chrg_chip *chip)
{
    char *name = NULL;
    mutex_init(&chip->irq_lock);
    mutex_init(&chip->i2c_lock);
    name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", "cps_wls_wake_lock");
    chip->cps_wls_wake_lock = wakeup_source_register(NULL, name);
}


static void cps_wls_lock_destroy(struct cps_wls_chrg_chip *chip)
{
    mutex_destroy(&chip->irq_lock);
    mutex_destroy(&chip->i2c_lock);
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

static int cps_wls_chrg_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int ret=0;
    cps_wls_log(CPS_LOG_ERR, "[%s] ---->start\n", __func__);
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        cps_wls_log(CPS_LOG_ERR,"[%s] cps_debug: Unable to allocate memory\n", __func__);
        return -ENOMEM;
    }
    chip->client = client;
    chip->dev = &client->dev;
    chip->name = "cps_wls";
    chip->regmap = devm_regmap_init_i2c(client, &cps4035L_regmap_config);
    if (IS_ERR(chip->regmap)) {
        cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
        devm_kfree(&client->dev, chip);
        return PTR_ERR(chip->regmap);
    }
    i2c_set_clientdata(client, chip);
    dev_set_drvdata(&(client->dev), chip);

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

    cps_wls_lock_work_init(chip);

    cps_wls_create_device_node(&(client->dev));

    ret = cps_wls_register_psy(chip);
    if(IS_ERR(chip->wl_psy)){
        cps_wls_log(CPS_LOG_ERR, "[%s] power_supply_register wireless failed , ret = %d\n", __func__, ret);
        goto free_source;
    }

   // wake_lock(&chip->cps_wls_wake_lock);

    cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger addr low probe successful!\n", __func__);
    return ret;

free_source:
    cps_wls_lock_destroy(chip);
    cps_wls_log(CPS_LOG_ERR, "[%s] error: free resource.\n", __func__);

    return ret;
}

static void cps_wls_h_lock_work_init(struct cps_wls_chrg_chip *chip)
{
    mutex_init(&chip->i2c_lock);
}

static void cps_wls_h_lock_destroy(struct cps_wls_chrg_chip *chip)
{
    mutex_destroy(&chip->i2c_lock);
}

static int cps_wls_h_chrg_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int ret=0;
    cps_wls_log(CPS_LOG_ERR, "[%s] ---->start\n", __func__);
    chip_h = devm_kzalloc(&client->dev, sizeof(*chip_h), GFP_KERNEL);
    if (!chip_h) {
        cps_wls_log(CPS_LOG_ERR,"[%s] cps_debug: Unable to allocate memory\n", __func__);
        return -ENOMEM;
    }
    chip_h->client = client;
    chip_h->dev = &client->dev;
    chip_h->name = "cps_wls_h";
    chip_h->regmap = devm_regmap_init_i2c(client, &cps4035H_regmap_config);
    if (IS_ERR(chip_h->regmap)) {
        cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
        devm_kfree(&client->dev, chip_h);
        return PTR_ERR(chip_h->regmap);
    }
    i2c_set_clientdata(client, chip_h);
    dev_set_drvdata(&(client->dev), chip_h);

    cps_wls_h_lock_work_init(chip_h);
    cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger addr high probe successful!\n", __func__);
    return ret;
}

static void not_called_h_api(void)
{
    return;
}

static void not_called_l_api(void)
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
    rc = cps_wls_set_tx_fod_thresh_I(1500);
    rc = cps_wls_set_tx_fod_thresh_II(1500);
    rc = cps_wls_set_tx_fod_rp_thresh(3000);
    rc = cps_wls_enable_tx_mode();
    rc = cps_wls_disable_tx_mode();
    rc = cps_wls_send_fsk_packet(data, 2);
    rc = cps_wls_set_fod_para();
    return;
}


static int cps_wls_h_chrg_remove(struct i2c_client *client)
{
    not_called_h_api();
    cps_wls_h_lock_destroy(chip_h);
    kfree(chip_h);
    return 0;
}

static int cps_wls_l_chrg_remove(struct i2c_client *client)
{
    not_called_l_api();
    //cps_wls_lock_destroy(chip);
    kfree(chip);
    return 0;
}


static const struct of_device_id cps_wls_chrg_of_tbl_h[] = {
    { .compatible = "cps,wls-charger-cps4035-H", .data = NULL},
    {},
};

static const struct i2c_device_id cps_wls_charger_id_h[] = {
    {"cps-wls-charger-h", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id_h);

static struct i2c_driver cps_wls_charger_driver_h = {
    .driver = {
        .name       = CPS_WLS_H_CHRG_DRV_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = cps_wls_chrg_of_tbl_h,
    },
    .probe      = cps_wls_h_chrg_probe,
    .remove     = cps_wls_h_chrg_remove,
    .id_table   = cps_wls_charger_id_h,
};


static const struct i2c_device_id cps_wls_charger_id_l[] = {
    {"cps-wls-charger-l", 0},
    {}, 
};

static const struct of_device_id cps_wls_chrg_of_tbl_l[] = {
    { .compatible = "cps,wls-charger-cps4035-L", .data = NULL},
    {},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id_l);

static struct i2c_driver cps_wls_charger_driver_l = {
    .driver = {
        .name       = CPS_WLS_L_CHRG_DRV_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = cps_wls_chrg_of_tbl_l,
    },
    .probe      = cps_wls_chrg_probe,
    .remove     = cps_wls_l_chrg_remove,
    .id_table   = cps_wls_charger_id_l,
};

static int __init cps_wls_driver_init(void)
{
    return (i2c_add_driver(&cps_wls_charger_driver_l) & 
            i2c_add_driver(&cps_wls_charger_driver_h));
}

late_initcall(cps_wls_driver_init);

static void __exit cps_wls_driver_exit(void)
{
    i2c_del_driver(&cps_wls_charger_driver_l);
    i2c_del_driver(&cps_wls_charger_driver_h);
}

module_exit(cps_wls_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jian.deng@convenientpower.com");
MODULE_DESCRIPTION("cps_wls_charger driver");
MODULE_ALIAS("i2c:cps_wls_charger");

