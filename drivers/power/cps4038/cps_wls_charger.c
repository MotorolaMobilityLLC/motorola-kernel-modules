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
#include <cps_wls_charger.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/alarmtimer.h>

#define BOOTLOADER_FILE_NAME "/data/misc/cps/bootloader.hex"
#define FIRMWARE_FILE_NAME   "/data/misc/cps/firmware.hex"

#define CPS_WLS_CHRG_DRV_NAME "cps-wls-charger"

#define CPS_WLS_CHRG_PSY_NAME "wireless"
struct cps_wls_chrg_chip *chip = NULL;

enum {
	TX_MODE_OVERHEAT = -1,
	TX_MODE_NOT_CONNECTED = 0,
	TX_MODE_POWER_SHARE = 2,
};
/*define cps rx reg enum*/
typedef enum
{
    CPS_RX_REG_EPT_VAL,
    CPS_RX_REG_POWER_SET,
    CPS_RX_REG_VOUT_SET,
    CPS_RX_REG_OCP_TH,
    CPS_RX_REG_OVP_TH,
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
    CPS_RX_REG_ADC_IRECT,
    CPS_RX_REG_ADC_VOUT,
    CPS_RX_REG_ADC_DIE_TMP,
    CPS_RX_REG_RP_VRECT_AVG,
    CPS_RX_REG_RP_IRECT_AVG,
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
    CPS_TX_REG_UVP_TH,       
    CPS_TX_REG_OVP_TH,          
    CPS_TX_REG_FOP_MIN,        
    CPS_TX_REG_FOP_MAX,         
    CPS_TX_REG_PING_FREQ,       
    CPS_TX_REG_PING_DUTY,       
    CPS_TX_REG_PING_TIME,      
    CPS_TX_REG_PING_INTERVAL,   
    CPS_TX_REG_FOD0_TH,        
    CPS_TX_REG_FOD1_TH,       
    CPS_TX_REG_FOD2_TH,      
    CPS_TX_REG_FOD3_TH,       
    CPS_TX_REG_FOD4_TH,       
    CPS_TX_REG_FOD5_TH,      
    CPS_TX_REG_FOD6_TH,       
    CPS_TX_REG_FOD7_TH,       
    CPS_TX_REG_FOD_RP0_TH,       
    CPS_TX_REG_FOD_RP1_TH,      
    CPS_TX_REG_FOD_RP2_TH,    
    CPS_TX_REG_FOD_RP3_TH,       
    CPS_TX_REG_FOD_RP4_TH,       
    CPS_TX_REG_FOD_RP5_TH,      
    CPS_TX_REG_FOD_RP6_TH,   
    CPS_TX_REG_FOD_RP7_TH,     
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
    CPS_TX_REG_MAX
}cps_tx_reg_e;

#define RX_REG_FOD_CUR_0        0x00EC
#define RX_REG_FOD_CUR_1         0x00ED
#define RX_REG_FOD_CUR_2         0x00EE
#define RX_REG_FOD_CUR_3         0x00EF
#define RX_REG_FOD_CUR_4         0x00F0
#define RX_REG_FOD_CUR_5        0x00F1
#define RX_REG_FOD_CUR_6         0x00F2
#define RX_REG_FOD_C0_GAIN      0x00F3
#define RX_REG_FOD_C0_OFFSET    0x00F4
#define RX_REG_FOD_C1_GAIN      0x00F5
#define RX_REG_FOD_C1_OFFSET    0x00F6
#define RX_REG_FOD_C2_GAIN      0x00F7
#define RX_REG_FOD_C2_OFFSET    0x00F8
#define RX_REG_FOD_C3_GAIN      0x00F9
#define RX_REG_FOD_C3_OFFSET    0x00FA
#define RX_REG_FOD_C4_GAIN      0x00FB
#define RX_REG_FOD_C4_OFFSET    0x00FC
#define RX_REG_FOD_C5_GAIN      0x00FD
#define RX_REG_FOD_C5_OFFSET    0x00FE
#define RX_REG_FOD_C6_GAIN      0x00FF
#define RX_REG_FOD_C6_OFFSET    0x0100
#define RX_REG_FOD_C7_GAIN      0x0101
#define RX_REG_FOD_C7_OFFSET    0x0102

static bool CPS_RX_MODE_ERR = false;
static bool CPS_TX_MODE = false;
static bool CPS_RX_CHRG_FULL = false;
static void cps_rx_online_check(struct cps_wls_chrg_chip *chg);
static int cps_wls_rx_power_on(void);

typedef enum
{
	WLC_DISCONNECTED,
	WLC_CONNECTED,
	WLC_TX_TYPE_CHANGED,
	WLC_TX_POWER_CHANGED,
}wlc_status;

typedef enum {
	WLC_NONE,
	WLC_BPP,
	WLC_EPP,
	WLC_MOTO,
} mmi_wlc_type;

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
	MMI_DOCK_FAN_SPEED_LOW = 0x14,//2000
	MMI_DOCK_FAN_SPEED_HIGH = 0x28,//4000
	MMI_DOCK_FAN_DEFAULT = MMI_DOCK_FAN_SPEED_HIGH,
} mmi_dock_fan_speed;
typedef struct
{
    uint16_t     reg_name;
    uint16_t     reg_bytes_len;
    uint32_t     reg_addr;
}cps_reg_s;

cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
    /* reg name               bytes number      reg address          */
    {CPS_COMM_REG_CHIP_ID,            2,              0x0000},
    {CPS_COMM_REG_FW_VER,          2,              0x0008},
    {CPS_COMM_REG_SYS_MODE,        1,              0x000A},
    {CPS_COMM_REG_INT_EN,          4,              0x0040},
    {CPS_COMM_REG_INT_FLAG,        4,              0x0044},
    {CPS_COMM_REG_INT_CLR,         4,              0x0048},
    {CPS_COMM_REG_CMD,             1,              0x004C},
};

cps_reg_s cps_rx_reg[CPS_RX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_RX_REG_EPT_VAL,         2,              0x019A},
    {CPS_RX_REG_POWER_SET,       1,              0x0104},
    {CPS_RX_REG_VOUT_SET,        2,              0x00C0},
    {CPS_RX_REG_OCP_TH,          2,              0x00E2},
    {CPS_RX_REG_OVP_TH,          1,              0x00F0},
    {CPS_RX_REG_DROP_MIN,        2,              0x00CC},
    {CPS_RX_REG_DROP_MAX,        2,              0x00CE},
    {CPS_RX_REG_DROP_MIN_CUR,    2,              0x00D0},
    {CPS_RX_REG_DROP_MAX_CUR,    2,              0x00D2},
    {CPS_RX_REG_SS_VAL,          2,              0x0194},
    {CPS_RX_REG_CE_VAL,          2,              0x0196},
    {CPS_RX_REG_RP_VAL,          2,              0x0198},
    {CPS_RX_REG_FOP_VAL,         2,              0x018E},
    {CPS_RX_REG_NEGO_POWER,      1,              0x0186},
    {CPS_RX_REG_NEGO_PRO,        1,              0x0187},
    {CPS_RX_REG_ADC_VRECT,       2,              0x0184},
    {CPS_RX_REG_ADC_IRECT,        2,              0x0188},
    {CPS_RX_REG_ADC_VOUT,        2,              0x018A},
    {CPS_RX_REG_ADC_DIE_TMP,     2,              0x018C},
    {CPS_RX_REG_RP_VRECT_AVG,    2,              0x0190},
    {CPS_RX_REG_RP_IRECT_AVG,    2,              0x0192},
    {CPS_RX_REG_PPP_HEADER,      1,              0x0080},
    {CPS_RX_REG_PPP_COMMAND,     1,              0x0081},
    {CPS_RX_REG_PPP_DATA0,       1,               0x0082},
    {CPS_RX_REG_PPP_DATA1,       1,               0x0083},
    {CPS_RX_REG_PPP_DATA2,       1,               0x0084},
    {CPS_RX_REG_PPP_DATA3,       1,               0x0085},
    {CPS_RX_REG_PPP_DATA4,       1,               0x0086},
    {CPS_RX_REG_PPP_DATA5,       1,               0x0087},
    {CPS_RX_REG_PPP_DATA6,       1,               0x0088},
    {CPS_RX_REG_BC_HEADER,       1,              0x00A0},
    {CPS_RX_REG_BC_COMMAND,      1,          0x00A1},
    {CPS_RX_REG_BC_DATA0,        1,               0x00A2},
    {CPS_RX_REG_BC_DATA1,        1,               0x00A3},
    {CPS_RX_REG_BC_DATA2,        1,               0x00A4},
    {CPS_RX_REG_BC_DATA3,        1,               0x00A5},
    {CPS_RX_REG_BC_DATA4,        1,               0x00A6},
    {CPS_RX_REG_BC_DATA5,        1,               0x00A7},
    {CPS_RX_REG_BC_DATA6,        1,              0x00A8},
    
};

cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
    /* reg name            bytes number      reg address          */
    {CPS_TX_REG_PPP_HEADER,      1,              0x00A0},
    {CPS_TX_REG_PPP_COMMAND,     1,              0x00A1},
    {CPS_TX_REG_PPP_DATA0,       1,               0x00A2},
    {CPS_TX_REG_PPP_DATA1,       1,               0x00A3},
    {CPS_TX_REG_PPP_DATA2,       1,               0x00A4},
    {CPS_TX_REG_PPP_DATA3,       1,               0x00A5},
    {CPS_TX_REG_PPP_DATA4,       1,               0x00A6},
    {CPS_TX_REG_PPP_DATA5,       1,               0x00A7},
    {CPS_TX_REG_PPP_DATA6,       1,               0x00A8},
    {CPS_TX_REG_BC_HEADER,       1,              0x0080},
    {CPS_TX_REG_BC_COMMAND,      1,               0x0081},
    {CPS_TX_REG_BC_DATA0,        1,               0x0082},
    {CPS_TX_REG_BC_DATA1,        1,               0x0083},
    {CPS_TX_REG_BC_DATA2,        1,               0x0084},
    {CPS_TX_REG_BC_DATA3,        1,               0x0085},
    {CPS_TX_REG_BC_DATA4,        1,               0x0086},
    {CPS_TX_REG_BC_DATA5,        1,               0x0087},
    {CPS_TX_REG_BC_DATA6,        1,              0x0088},
    {CPS_TX_REG_OCP_TH,          2,              0x0202},
    {CPS_TX_REG_UVP_TH,          2,              0x0204},
    {CPS_TX_REG_OVP_TH,          2,              0x0206},
    {CPS_TX_REG_FOP_MIN,         1,              0x0210},
    {CPS_TX_REG_FOP_MAX,         1,              0x0211},
    {CPS_TX_REG_PING_FREQ,       1,              0x0212},
    {CPS_TX_REG_PING_DUTY,       1,              0x0213},
    {CPS_TX_REG_PING_TIME,       2,              0x0214},
    {CPS_TX_REG_PING_INTERVAL,   2,              0x0216},
    {CPS_TX_REG_FOD0_TH,        2,              0x0232},
    {CPS_TX_REG_FOD1_TH,       2,              0x0234},
    {CPS_TX_REG_FOD2_TH,        2,              0x0236},
    {CPS_TX_REG_FOD3_TH,       2,              0x0238},
    {CPS_TX_REG_FOD4_TH,        2,              0x023A},
    {CPS_TX_REG_FOD5_TH,       2,              0x023C},
    {CPS_TX_REG_FOD6_TH,       2,              0x023E},
    {CPS_TX_REG_FOD7_TH,       2,              0x0240},
    {CPS_TX_REG_FOD_RP0_TH,       1,              0x022A},
    {CPS_TX_REG_FOD_RP1_TH,       1,              0x022B},
    {CPS_TX_REG_FOD_RP2_TH,       1,              0x022C},
    {CPS_TX_REG_FOD_RP3_TH,       1,              0x022D},
    {CPS_TX_REG_FOD_RP4_TH,       1,              0x022E},
    {CPS_TX_REG_FOD_RP5_TH,       1,              0x022F},
    {CPS_TX_REG_FOD_RP6_TH,       1,              0x0230},
    {CPS_TX_REG_FOD_RP7_TH,       1,              0x0231},
    {CPS_TX_REG_FUNC_EN,         1,              0x0050},
    {CPS_TX_REG_MAX_CUR,         2,              0x020A},
    {CPS_TX_REG_FOP_VAL,         2,              0x02C0},
    {CPS_TX_REG_ADC_VIN,         2,              0x02C2},
    {CPS_TX_REG_ADC_VRECT,       2,              0x02C4},
    {CPS_TX_REG_ADC_I_IN,        2,              0x02C6},
    {CPS_TX_REG_CE_VAL,          1,              0x02CB},
    {CPS_TX_REG_RP_VAL,          2,              0x02CC},
    {CPS_TX_REG_EPT_RSN,         2,              0x02CE},
    {CPS_TX_REG_ADC_DIE_TEMP,    2,              0x02C8},
    {CPS_TX_REG_EPT_CODE,        1,              0x02CA},
};
static u8 CPS4038_BOOTLOADER[0x800] = {
	// CPS4038_BL_01_07_V0.1_CRC67B6
	0x78, 0x1D, 0x00, 0x20, 0xB9, 0x04, 0x00, 0x20,
	0x55, 0x01, 0x00, 0x20, 0x7D, 0x01, 0x00, 0x20,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x59, 0x01, 0x00, 0x20,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x5B, 0x01, 0x00, 0x20, 0x5D, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x5F, 0x01, 0x00, 0x20, 0x5F, 0x01, 0x00, 0x20,
	0x00, 0x01, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00,
	0xE8, 0x04, 0x00, 0x00, 0x00, 0xF0, 0x02, 0xF8,
	0x00, 0xF0, 0x30, 0xF8, 0x0C, 0xA0, 0x30, 0xC8,
	0x08, 0x38, 0x24, 0x18, 0x2D, 0x18, 0xA2, 0x46,
	0x67, 0x1E, 0xAB, 0x46, 0x54, 0x46, 0x5D, 0x46,
	0xAC, 0x42, 0x01, 0xD1, 0x00, 0xF0, 0x22, 0xF8,
	0x7E, 0x46, 0x0F, 0x3E, 0x0F, 0xCC, 0xB6, 0x46,
	0x01, 0x26, 0x33, 0x42, 0x00, 0xD0, 0xFB, 0x1A,
	0xA2, 0x46, 0xAB, 0x46, 0x33, 0x43, 0x18, 0x47,
	0xCC, 0x03, 0x00, 0x00, 0xDC, 0x03, 0x00, 0x00,
	0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26,
	0x10, 0x3A, 0x01, 0xD3, 0x78, 0xC1, 0xFB, 0xD8,
	0x52, 0x07, 0x00, 0xD3, 0x30, 0xC1, 0x00, 0xD5,
	0x0B, 0x60, 0x70, 0x47, 0x1F, 0xB5, 0x1F, 0xBD,
	0x10, 0xB5, 0x10, 0xBD, 0x00, 0xF0, 0x8D, 0xF9,
	0x11, 0x46, 0xFF, 0xF7, 0xF7, 0xFF, 0x00, 0xF0,
	0x43, 0xF9, 0x00, 0xF0, 0xA5, 0xF9, 0x03, 0xB4,
	0xFF, 0xF7, 0xF2, 0xFF, 0x03, 0xBC, 0x00, 0xF0,
	0xAB, 0xF9, 0x00, 0x00, 0xFE, 0xE7, 0xFE, 0xE7,
	0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7,
	0x02, 0x48, 0x03, 0x49, 0x03, 0x4A, 0x04, 0x4B,
	0x70, 0x47, 0x00, 0x00, 0x78, 0x18, 0x00, 0x20,
	0x78, 0x1D, 0x00, 0x20, 0x78, 0x19, 0x00, 0x20,
	0x78, 0x19, 0x00, 0x20, 0xFE, 0xE7, 0x9C, 0x48,
	0x01, 0x68, 0x00, 0x29, 0x08, 0xD1, 0x9B, 0x4A,
	0x0B, 0x21, 0xD1, 0x61, 0x99, 0x4A, 0x9A, 0x49,
	0xC0, 0x32, 0x91, 0x63, 0x01, 0x21, 0x01, 0x60,
	0x70, 0x47, 0x95, 0x48, 0x01, 0x68, 0x01, 0x29,
	0x09, 0xD1, 0x94, 0x49, 0x03, 0x22, 0xCA, 0x61,
	0x0B, 0x22, 0xCA, 0x61, 0x91, 0x4A, 0x00, 0x21,
	0xC0, 0x32, 0x91, 0x63, 0x01, 0x60, 0x70, 0x47,
	0x30, 0xB5, 0x5F, 0x22, 0x00, 0x23, 0x12, 0x02,
	0x90, 0x42, 0x04, 0xDB, 0x8D, 0x4A, 0x90, 0x42,
	0x01, 0xD0, 0x00, 0x20, 0x30, 0xBD, 0x89, 0x4A,
	0x90, 0x60, 0xD1, 0x60, 0x03, 0x21, 0xD1, 0x61,
	0x07, 0x20, 0xD0, 0x61, 0xD0, 0x61, 0xD0, 0x61,
	0xD0, 0x61, 0xD1, 0x61, 0x86, 0x48, 0x01, 0x24,
	0x15, 0x6A, 0x00, 0x2D, 0x05, 0xD0, 0x40, 0x1E,
	0xFA, 0xD2, 0xD4, 0x61, 0xD1, 0x61, 0x01, 0x23,
	0xF6, 0xE7, 0x08, 0x20, 0x40, 0x1E, 0xFD, 0xD2,
	0x18, 0x46, 0x30, 0xBD, 0x70, 0xB5, 0x04, 0x46,
	0x00, 0x20, 0x7E, 0x4D, 0x02, 0x46, 0x0E, 0xE0,
	0xA3, 0x5C, 0x1B, 0x02, 0x58, 0x40, 0x00, 0x23,
	0x06, 0x04, 0x02, 0xD5, 0x40, 0x00, 0x68, 0x40,
	0x00, 0xE0, 0x40, 0x00, 0x5B, 0x1C, 0x80, 0xB2,
	0x08, 0x2B, 0xF5, 0xDB, 0x52, 0x1C, 0x8A, 0x42,
	0xEE, 0xDB, 0x70, 0xBD, 0x00, 0x25, 0xFF, 0xF7,
	0xA2, 0xFF, 0x69, 0x1E, 0x6F, 0x48, 0xFF, 0xF7,
	0xBB, 0xFF, 0xFF, 0xF7, 0xAA, 0xFF, 0x70, 0x4F,
	0x70, 0x49, 0x08, 0x68, 0x00, 0x28, 0xFB, 0xD0,
	0x00, 0x22, 0x0A, 0x60, 0x14, 0x46, 0x01, 0x22,
	0x39, 0x68, 0x52, 0x02, 0x91, 0x42, 0x01, 0xDD,
	0x40, 0x20, 0x7B, 0xE0, 0x66, 0x22, 0x6A, 0x49,
	0x90, 0x28, 0x0A, 0x60, 0x24, 0xD0, 0x06, 0xDC,
	0x10, 0x28, 0x41, 0xD0, 0x20, 0x28, 0x5B, 0xD0,
	0x80, 0x28, 0x73, 0xD1, 0x34, 0xE0, 0xB0, 0x28,
	0x02, 0xD0, 0xC0, 0x28, 0x6E, 0xD1, 0x66, 0xE0,
	0xFF, 0xF7, 0x87, 0xFF, 0x61, 0x48, 0x84, 0x68,
	0x80, 0x14, 0x84, 0x42, 0x02, 0xDD, 0x60, 0x48,
	0x04, 0x60, 0x45, 0xE0, 0x01, 0x20, 0x21, 0x1F,
	0x40, 0x07, 0xFF, 0xF7, 0xAF, 0xFF, 0x5C, 0x4A,
	0x5C, 0x49, 0x10, 0x60, 0x61, 0x18, 0xC9, 0x6F,
	0x51, 0x60, 0x88, 0x42, 0x38, 0xD1, 0x50, 0xE0,
	0xFF, 0xF7, 0x6F, 0xFF, 0xC0, 0x20, 0x84, 0x68,
	0x00, 0x2C, 0x31, 0xDB, 0xC0, 0x01, 0x84, 0x42,
	0x2E, 0xDC, 0xC8, 0x2C, 0x2C, 0xDB, 0x21, 0x1F,
	0x00, 0x20, 0xFF, 0xF7, 0x97, 0xFF, 0x01, 0x46,
	0x4F, 0x48, 0x80, 0x3C, 0x01, 0x60, 0xE2, 0x6F,
	0x42, 0x60, 0x91, 0x42, 0x20, 0xD1, 0x38, 0xE0,
	0xFF, 0xF7, 0x49, 0xFF, 0x4C, 0x49, 0x43, 0x48,
	0xFF, 0xF7, 0x62, 0xFF, 0x04, 0x46, 0x11, 0xE0,
	0xFF, 0xF7, 0x41, 0xFF, 0x00, 0x26, 0x08, 0xE0,
	0x45, 0x48, 0xB1, 0x00, 0x41, 0x58, 0x28, 0x46,
	0xFF, 0xF7, 0x56, 0xFF, 0x04, 0x43, 0x2D, 0x1D,
	0x76, 0x1C, 0x38, 0x68, 0x86, 0x42, 0xF3, 0xDB,
	0x3F, 0x48, 0x1A, 0xE0, 0xFF, 0xF7, 0x3D, 0xFF,
	0x60, 0x1C, 0x1F, 0xD0, 0x00, 0x2C, 0x18, 0xD0,
	0xAA, 0x21, 0x39, 0x48, 0x01, 0x60, 0x8B, 0xE7,
	0xFF, 0xF7, 0x25, 0xFF, 0x00, 0x26, 0x08, 0xE0,
	0x3A, 0x48, 0xB1, 0x00, 0x41, 0x58, 0x28, 0x46,
	0xFF, 0xF7, 0x3A, 0xFF, 0x04, 0x43, 0x2D, 0x1D,
	0x76, 0x1C, 0x38, 0x68, 0x86, 0x42, 0xF3, 0xDB,
	0x34, 0x48, 0x05, 0x60, 0xE2, 0xE7, 0x30, 0x48,
	0x05, 0x68, 0x55, 0x20, 0x2C, 0x49, 0x08, 0x60,
	0x72, 0xE7, 0xFF, 0xE7, 0x40, 0x21, 0xE0, 0xE7,
	0x30, 0x48, 0x2F, 0x49, 0x01, 0x60, 0x30, 0x4A,
	0x00, 0x21, 0x91, 0x60, 0x19, 0x22, 0x2F, 0x4B,
	0xD2, 0x03, 0x5A, 0x60, 0x01, 0x60, 0x70, 0x47,
	0x70, 0xB5, 0x2C, 0x48, 0x00, 0x22, 0x80, 0x30,
	0x02, 0x62, 0x2C, 0x49, 0x2A, 0x48, 0xC8, 0x61,
	0x2B, 0x4C, 0x07, 0x23, 0x63, 0x61, 0xCA, 0x61,
	0x18, 0x4B, 0x38, 0x3B, 0xDB, 0x6B, 0x29, 0x4C,
	0xA3, 0x42, 0x0F, 0xD1, 0xC8, 0x61, 0xC1, 0x23,
	0x25, 0x4C, 0x00, 0x20, 0xDB, 0x01, 0xC0, 0x3C,
	0xC5, 0x18, 0x2D, 0x68, 0x06, 0x19, 0x35, 0x60,
	0x00, 0x1D, 0x80, 0x28, 0xF8, 0xDB, 0xCA, 0x61,
	0x01, 0x20, 0x70, 0xBD, 0x00, 0x20, 0x70, 0xBD,
	0xFF, 0x21, 0x09, 0x48, 0x01, 0x31, 0x41, 0x60,
	0x0D, 0x4A, 0x49, 0x00, 0x11, 0x60, 0x0D, 0x4A,
	0x00, 0x21, 0x11, 0x60, 0x0C, 0x4B, 0x55, 0x22,
	0x1A, 0x60, 0x01, 0x60, 0xFF, 0xF7, 0xCC, 0xFF,
	0xFF, 0xF7, 0xBE, 0xFF, 0xFF, 0xF7, 0x22, 0xFF,
	0x0C, 0x18, 0x00, 0x20, 0x00, 0x20, 0x01, 0x40,
	0x78, 0x56, 0x00, 0x00, 0xF8, 0x60, 0x00, 0x00,
	0x50, 0xC3, 0x00, 0x00, 0x21, 0x10, 0x00, 0x00,
	0x08, 0x18, 0x00, 0x20, 0x00, 0x18, 0x00, 0x20,
	0x04, 0x18, 0x00, 0x20, 0xC0, 0x00, 0x00, 0x20,
	0x00, 0x08, 0x00, 0x20, 0x80, 0xFF, 0xFF, 0x1F,
	0x85, 0xE1, 0x24, 0x57, 0x00, 0x10, 0x00, 0x20,
	0x51, 0xE5, 0xCC, 0x1A, 0x00, 0x8C, 0x00, 0x40,
	0x00, 0x80, 0x00, 0x40, 0x00, 0x00, 0x04, 0x40,
	0x50, 0x12, 0x00, 0x00, 0x40, 0xE7, 0x00, 0x40,
	0xC0, 0xF0, 0x00, 0x40, 0x4E, 0x87, 0x55, 0x74,
	0x70, 0x47, 0x70, 0x47, 0x70, 0x47, 0x70, 0x47,
	0x70, 0x47, 0x75, 0x46, 0x00, 0xF0, 0x24, 0xF8,
	0xAE, 0x46, 0x05, 0x00, 0x69, 0x46, 0x53, 0x46,
	0xC0, 0x08, 0xC0, 0x00, 0x85, 0x46, 0x18, 0xB0,
	0x20, 0xB5, 0xFF, 0xF7, 0x79, 0xFE, 0x60, 0xBC,
	0x00, 0x27, 0x49, 0x08, 0xB6, 0x46, 0x00, 0x26,
	0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5,
	0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5, 0xC0, 0xC5,
	0x40, 0x3D, 0x49, 0x00, 0x8D, 0x46, 0x70, 0x47,
	0x10, 0xB5, 0x04, 0x46, 0xC0, 0x46, 0xC0, 0x46,
	0x20, 0x46, 0xFF, 0xF7, 0x54, 0xFE, 0x10, 0xBD,
	0x00, 0x48, 0x70, 0x47, 0x14, 0x18, 0x00, 0x20,
	0x01, 0x49, 0x18, 0x20, 0xAB, 0xBE, 0xFE, 0xE7,
	0x26, 0x00, 0x02, 0x00, 0x70, 0x47, 0x00, 0x00,
	0x03, 0x49, 0x00, 0x20, 0x48, 0x60, 0x08, 0x60,
	0x02, 0x48, 0x80, 0x47, 0x02, 0x48, 0x00, 0x47,
	0x14, 0xE1, 0x00, 0x40, 0x4B, 0x04, 0x00, 0x20,
	0xCD, 0x00, 0x00, 0x20, 0xE4, 0x04, 0x00, 0x20,
	0x00, 0x08, 0x00, 0x20, 0x78, 0x15, 0x00, 0x00,
	0x10, 0x01, 0x00, 0x20, 0x08, 0x01, 0x00, 0x00,
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
//-------------------I2C APT start--------------------

static const struct regmap_config cps4038_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8, 
};
    
static const struct regmap_config cps4038_regmap32_config = {
    .reg_bits = 32,
    .val_bits = 8,
};

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

static int cps_wls_write_word(int addr, int value)
{
    int ret;
    u8 write_date[4];

    write_date[3] = (value >> 24) & 0xff;
    write_date[2] = (value >> 16) & 0xff;
    write_date[1] = (value >> 8) & 0xff;
    write_date[0] = value & 0xff;

    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap32, addr, write_date, 4);
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
    ret = regmap_raw_read(chip->regmap32, addr, read_date, 4);
    mutex_unlock(&chip->i2c_lock);
    
    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
        return CPS_WLS_FAIL;
    }
    return *(int *)read_date;
}

static int cps_wls_program_sram(int addr, u8 *date, int len)
{
      int ret;
   
    mutex_lock(&chip->i2c_lock);
    ret = regmap_raw_write(chip->regmap32, addr, date, len);
    mutex_unlock(&chip->i2c_lock);

    if (ret < 0){   
        cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
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
    int res;
    int cnt = 0;//ms
    while(1)
    {
        res = cps_wls_read_word(ADDR_FLAG);
        if(res == CPS_WLS_FAIL)  return CPS_WLS_FAIL;
 
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

        if(res == PASS)
        {
            break;
        }

        /*3s over time*/
        if((cnt++) == 3000)
        {
            cps_wls_log(CPS_LOG_ERR, "--->[%s] CMD-OVERTIME\n", __func__);
            break;
        }
    }
    return res;
 

    //return CPS_WLS_SUCCESS;
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


#if 0
static int fp_size(struct file *f)
{
    int error = -EBADF;
    struct kstat stat;

    error = vfs_getattr(&f->f_path, &stat, STATX_SIZE, AT_STATX_SYNC_AS_STAT);

    if (error == 0)
    {
        return stat.size;
    }
    else
    {
        pr_err("get file file stat error\n");
        return error;
    }
}

static int cps_file_read(char *filename, char **buf)
{
    struct file *fp;
    mm_segment_t fs;
    int size = 0;
    loff_t pos = 0;

    fp = filp_open(filename, O_RDONLY, 0);
    if (IS_ERR(fp))
    {
        pr_err("open %s file error\n", filename);
        goto end;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    size = fp_size(fp);
    if (size <= 0)
    {
        pr_err("load file:%s error\n", filename);
        goto error;
    }

    *buf = kzalloc(size + 1, GFP_KERNEL);
    vfs_read(fp, *buf, size, &pos);

error:
    filp_close(fp, NULL);
    set_fs(fs);
end:
    return size;
}

static unsigned char chartoBcd(char iChar)
{
    unsigned char mBCD = 0;

    if (iChar >= '0' && iChar <= '9') mBCD = iChar - '0';
      else if (iChar >= 'A' && iChar <= 'F') mBCD = iChar - 'A' + 0x0a;
      else if (iChar >= 'a' && iChar <= 'f') mBCD = iChar - 'a' + 0x0a;

    return mBCD;
}

static unsigned char* file_parse(char *buf, int size,
                                       unsigned char *file, int *file_length)
{
    int  i = 0, j = 0;
    int file_index = 0;
    char temp;

    if (!buf || !file)
        return NULL;
    for(i = 0; i < size; i++){
        if(buf[i] == '\n' || buf[i] == ' ' || buf[i] == '\r') {
          file_index = 0;
          continue;
        } else{
            if (file_index == 1) {
                file_index++;
                file[j++] = (unsigned char)((chartoBcd(temp) << 4) + chartoBcd(buf[i]));
            } else if (file_index == 0) {
                file_index++;
                temp = buf[i];
            }
        }

    }
    // file[j] = '\0';
    *file_length = j;

    return file;
}

/*static int bootloader_load(unsigned char *bootloader, int *bootloader_length)
{
    char *buf = NULL;
    int size = 0;

    size = cps_file_read(BOOTLOADER_FILE_NAME, &buf);
    if (size > 0){

        if (bootloader == NULL){
            kfree(buf);
            pr_err("file alloc error.\n");
            return -EINVAL;
        }

        if(file_parse(buf, size, bootloader, bootloader_length) == NULL){
            kfree(buf);
            pr_err("file parse error\n");
            return -EINVAL;
        }
        kfree(buf);
    }

    return 0;
}*/

static int firmware_load(unsigned char *firmeware, int *firmeware_length)
{
    char *buf = NULL;
    int size = 0;

    size = cps_file_read(FIRMWARE_FILE_NAME, &buf);
    if (size > 0){

        if (firmeware == NULL){
            kfree(buf);
            pr_err("file alloc error.\n");
            return -EINVAL;
        }

        if(file_parse(buf, size, firmeware, firmeware_length) == NULL){
            kfree(buf);
            pr_err("file parse error\n");
            return -EINVAL;
        }
        kfree(buf);
    }

    return 0;
}



static int update_firmware(void)
{
    int ret, i;
    int firmware_length;
    int buf0_flag = 0, buf1_flag = 0;
    //unsigned char *bootloader_buf;
    unsigned char *firmware_buf;
    //unsigned char *p;
    int result;
    int cfg_buf_size;
    int addr;

    //bootloader_buf = kzalloc(0x800, GFP_KERNEL);  // 2K buffer
    firmware_buf = kzalloc(0x6000, GFP_KERNEL);  // 24K buffer
    /***************************************************************************************
                                *                                  Step1, load to sram                                *
     ***************************************************************************************/
    cps_wls_write_word(0xFFFFFF00, 0x0000000E); /*enable 32bit i2c*/
    cps_wls_write_word(0x4000E75C, 0x00001250); /*write password*/
    cps_wls_write_word(0x40040010, 0x00000006); /*reset and halt mcu*/
    cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM HEX!\n", __func__);
    cps_wls_program_sram(0x20000000, CPS4038_BL, 0x800);

    cps_wls_write_word(0x400400A0, 0x000000FF); /*enable remap function*/
    cps_wls_write_word(0x40040010, 0x00008003); /*triming load function is disabled and run mcu*/

    msleep(10);

    cps_wls_write_word(0xFFFFFF00, 0x0000000E); /*enable 32bit i2c*/


    msleep(10);
    
    /***************************************************************************************
                                *                          Step2, bootloader crc check                                *
    ***************************************************************************************/
    cps_wls_program_cmd_send(CACL_CRC_TEST);
    result = cps_wls_program_wait_cmd_done( );

     if(result != PASS)
     {
            cps_wls_log(CPS_LOG_ERR, "[%s]  ---> BOOTLOADER CRC FAIL\n", __func__);
            goto update_fail;
     }
     cps_wls_log(CPS_LOG_DEBG,  "[%s]  ---> LOAD BOOTLOADER SUCCESSFUL\n", __func__);

     /***************************************************************************************
                                *                          Step3, load firmware to MTP                                *
     ***************************************************************************************/
     memset(firmware_buf, 0, 0x6000);
     ret = firmware_load(firmware_buf, &firmware_length);//load bootloader
     if (ret != 0) {
         cps_wls_log(CPS_LOG_ERR, "[%s] ---- firmware get error %d\n", __func__, ret);
         goto update_fail;
     }
      cps_wls_log(CPS_LOG_DEBG,  "[%s]  ---> START LOAD APP HEX\n", __func__);
      buf0_flag    = 0;
      buf1_flag    = 0;
      cfg_buf_size = 512;
      addr = 0;
      cps_wls_write_word(ADDR_BUF_SIZE, cfg_buf_size);

      result =  cps_wls_program_wait_cmd_done( );
       if(result != PASS)
       {
             cps_wls_log(CPS_LOG_ERR,   "[%s]  ---> ERASE FAIL\n", __func__);
      
             goto update_fail;
       }

        for(i = 0; i < (24 * 1024) / 4 / cfg_buf_size; i++)
        {
                      if(buf0_flag == 0)
                      {
                               cps_wls_program_sram(ADDR_BUFFER0, firmware_buf+addr, cfg_buf_size*4);
                               addr  = addr  + cfg_buf_size*4;

                               if(buf1_flag == 1)
                               {
                                     result = cps_wls_program_wait_cmd_done( );
                                     if(result != PASS)
                                     {
                                              pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n",__func__);
                                              goto update_fail;
                                     }
                                     buf1_flag = 0;
                               }
                               cps_wls_program_cmd_send(PGM_BUFFER0);
                               buf0_flag = 1;
                               continue;
         }

         if(buf1_flag == 0)
         {
                               cps_wls_program_sram(ADDR_BUFFER1, firmware_buf+addr, cfg_buf_size*4);
                               addr  = addr  + cfg_buf_size*4;

                               if(buf0_flag == 1)
                               {
                                       result =  cps_wls_program_wait_cmd_done( );
                                       if(result != PASS)
                                       {
                                                pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n",__func__);
                                               goto update_fail;
                                       }
                                       buf0_flag = 0;
                                }
                                cps_wls_program_cmd_send(PGM_BUFFER1);                           
                                buf1_flag = 1;
                                continue;
                        }
         }

         if(buf0_flag == 1)
         {
                        result = cps_wls_program_wait_cmd_done( );
                        if(result != PASS)
                        {
                               pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n",__func__);
                               goto update_fail;
                        }
                        buf0_flag = 0;
         }

         if(buf1_flag == 1)
         {
                        result = cps_wls_program_wait_cmd_done( );
                        if(result != PASS)
                        {
                               pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n",__func__);
                              goto update_fail;
                        }
                        buf1_flag = 0;
         }
         pr_info("%s: ---> LOAD APP HEX SUCCESSFUL\n",__func__);
         /***************************************************************************************
                            *                          Step4, check app CRC                                       *
         ***************************************************************************************/
        cps_wls_program_cmd_send(CACL_CRC_APP);
        result = cps_wls_program_wait_cmd_done( );
    
        if(result != PASS)
        {
                       pr_err("%s: ---> APP CRC FAIL\n",__func__);
                       goto update_fail;
        }
        pr_info("%s: ---> CHERK APP CRC SUCCESSFUL\n",__func__);
       /***************************************************************************************
                         *                          Step5, write mcu start flag                                *
       ***************************************************************************************/
      cps_wls_program_cmd_send(PGM_WR_FLAG);
      result = cps_wls_program_wait_cmd_done( );
      if(result != PASS)
      {
                       pr_err("%s:---> WRITE MCU START FLAG FAIL\n",__func__);
                       goto update_fail;
       }
               
       pr_info("%s: ---> WRITE MCU START FLAG SUCCESSFUL\n",__func__);
       cps_wls_write_word(0x40040010, 0x00000008); /*reset all system*/

       msleep(100);

       cps_wls_write_word(0xFFFFFF00, 0x00000000); /*i2c 32bit mode disable*/


       return CPS_WLS_SUCCESS;

update_fail:
    cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);
    return CPS_WLS_FAIL;
}
#endif
//****************************************************************
//-------------------I2C APT end--------------------

//-------------------CPS4038 system interface-------------------

static void cps_wls_write_password()
{
    cps_wls_log(CPS_LOG_DEBG, "[%s] -------write password\n", __func__);
//no need at cps4038
//    cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);
//    cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
//    cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);
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
static int cps_wls_get_chip_id(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CHIP_ID]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}
#endif

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

//-------------------CPS4038 RX interface-------------------
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
#endif
static int cps_wls_get_rx_neg_power(void)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_NEGO_POWER]);
    return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

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
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_IRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_iout(void)
{
    cps_reg_s *cps_reg;
    cps_wls_write_password();
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_IRECT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_rx_vout(void)
{
    cps_reg_s *cps_reg;
    cps_wls_write_password();
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_ADC_VOUT]);
    return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
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

static int  cps_get_sys_op_mode(Sys_Op_Mode *sys_mode_type)
{
	int status = CPS_WLS_SUCCESS;
	uint32_t temp = 0;
	uint8_t retries = 3;
	cps_reg_s *cps_reg;

	do{

		//msleep(10);//10ms
		retries--;
		cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_NEGO_PRO]);

		temp =  cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);

		*sys_mode_type = (temp & CPS_MASK(7, 0));
		cps_wls_log(CPS_LOG_DEBG, "CPS_REG_Sys_Op_Mode = 0x%02x, 0x%02x", temp, *sys_mode_type);
		if(*sys_mode_type == Sys_Op_Mode_AC_Missing
				|| *sys_mode_type == Sys_Op_Mode_BPP
				|| *sys_mode_type == Sys_Op_Mode_EPP
				|| *sys_mode_type == Sys_Op_Mode_MOTO_WLC) {
			cps_wls_log(CPS_LOG_DEBG, "CPS_REG_Sys_Op_Mode = 0x%02x, 0x%02x", temp, *sys_mode_type);

			break;

		}

	}while(retries);

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
        54,                 6,             // C0
        54,                 6,             // C1
        54,                 6,             // C2
        54,                 6,             // C3
        54,                 6,             // C4
        54,                 6,             // C5
        54,                 6,             // C6
        54,                 6,             // C7
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

static int cps_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
    uint16_t cmd;
    uint8_t i;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_PPP_HEADER]);


    for(i = 0; i < data_len; i++)
    {
        if(cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1) == CPS_WLS_FAIL)
        {
            return CPS_WLS_FAIL;
        }
    }

    cmd = cps_wls_get_cmd();
    cmd |= RX_CMD_SEND_DATA;
    return cps_wls_set_cmd(cmd);
}

static int cps_wls_send_handshake_packet(uint8_t *data, uint8_t data_len)
{
       return cps_wls_send_ask_packet( data, data_len);    
}

#endif
//-------------------CPS4038 TX interface-------------------
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
 * @brief  Rp power smaller than RP0 threshold(0x0252),FOD ploss trigger threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod0_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD0_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP0 threshold(0x0254), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod1_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD1_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP1 threshold(0x0256), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod2_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD2_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP3 threshold(0x0258), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod3_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD3_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP4 threshold(0x025A), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod4_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD4_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP5 threshold(0x025C), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod5_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD5_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP6 threshold(0x025E), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod6_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD6_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  Rp power larger than RP7 threshold(0x0260), FOD ploss trigger  threshold setting
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod7_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD7_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}


/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp0_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP0_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp1_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP1_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp2_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP2_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp3_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP3_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp4_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP4_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp5_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP5_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp6_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP6_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

/**
 * @brief  RP power threshold for FOD threshold
 * @note   
 * @param  None
 * @retval 
 */
static int cps_wls_set_tx_fod_rp7_thresh(int value)
{
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD_RP7_TH]);
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

static int cps_wls_get_fsk_packet(uint8_t *data)
{
    int temp;
    uint8_t i;
    uint8_t data_len;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_RX_REG_BC_HEADER]);

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
static int cps_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
    int status = CPS_WLS_SUCCESS;
    uint32_t cmd = 0, i = 0;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_rx_reg[CPS_RX_REG_PPP_HEADER]);


    for(i = 0; i < data_len; i++)
    {
        status = cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1);
        if (CPS_WLS_SUCCESS != status)
        {
            cps_wls_log(CPS_LOG_ERR, " cps wls write failed, addr 0x%02x, data 0x%02x", cps_reg->reg_addr + i, *(data + i));
            return status;
        }
    }

	cmd = cps_wls_get_cmd();

	cps_wls_log(CPS_LOG_ERR, " cps_wls_get_cmd %x\n",cmd);

	cmd |= RX_CMD_SEND_ASK;
	status = cps_wls_set_cmd(cmd);
	if(CPS_WLS_SUCCESS != status)
	{
		cps_wls_log(CPS_LOG_ERR, " cps TX_CMD_SEND_FSK failed");
	}

	return status;
}

static int cps_wls_get_ask_packet(uint8_t *data)
{
    int  temp;
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

/*static int cps_wls_set_tx_huge_metal_threshold(int value)
{
    if(value < 0 || value > 1000) return CPS_WLS_FAIL;
    cps_reg_s *cps_reg;
    cps_reg = (cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PING_OCP_TH]);
    return cps_wls_write_reg((int)cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}*/
#endif
//------------------------------IRQ Handler-----------------------------------
static int cps_wls_set_int_enable(void)
{
    uint32_t int_en;
    cps_reg_s *cps_reg;

    int_en = 0xFFFFFFFF;
    cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_EN]);

    if(CPS_WLS_FAIL == cps_wls_write_reg((int)cps_reg->reg_addr, int_en, (int)cps_reg->reg_bytes_len))  goto set_int_fail;
    return CPS_WLS_SUCCESS;

set_int_fail:
    return CPS_WLS_FAIL;
}

static void cps_bpp_icl_on()
{
	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;

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

static void cps_epp_icl_on()
{
	int icl, vbus;

	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;

	if (chip->factory_wls_en)
		return;
	cps_get_sys_op_mode(&mode_type);
	if (mode_type == Sys_Op_Mode_EPP)
	{
		chip->mode_type = mode_type;
		chip->wlc_tx_power = cps_wls_get_rx_neg_power() / 2;
		cps_epp_current_select(&icl, &vbus);
		if (chip->chg1_dev)
		{
			charger_dev_set_charging_current(chip->chg1_dev, 3150000);
			charger_dev_set_input_current(chip->chg1_dev, icl);
		}
		else
		{
			cps_init_charge_hardware();
			charger_dev_set_charging_current(chip->chg1_dev, 3150000);
			charger_dev_set_input_current(chip->chg1_dev, icl);
		}
	}
}

static int cps_wls_rx_irq_handler(int int_flag)
{
    int rc = 0;
	Sys_Op_Mode mode_type = Sys_Op_Mode_INVALID;
     uint8_t data[8] = {0};
    if (int_flag & RX_INT_POWER_ON)
    {
		CPS_RX_MODE_ERR = false;
		CPS_RX_CHRG_FULL = false;
		chip->cable_ready_wait_count = 0;
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_POWER_ON");
    }
    if(int_flag & RX_INT_LDO_OFF)
    {
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_LDO_OFF");
    }
	if (int_flag & RX_INT_LDO_ON)
	{
		cps_rx_online_check(chip);
        /* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
        /* 9 = LOW_POWER_OFF_CHARGING_BOOT */
        if(chip->bootmode == 8 || chip->bootmode == 9)
            cps_rx_online_check(chip);

		chip->rx_ldo_on = true;
		if (chip->wlc_status == WLC_DISCONNECTED)
		{
			cps_wls_set_status(WLC_CONNECTED);
		}
		cps_bpp_icl_on();
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_LDO_ON");
	}
    if(int_flag & RX_INT_READY){
            data[0] = 0x38;
            data[1] = 0x3B;
            data[2] = 0x88;
            data[3] = 0x66;
            //data[0] = 0x38;
           // cps_wls_send_handshake_packet(data,4);
		chip->rx_int_ready = true;
		cps_rx_online_check(chip);
        /* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
        /* 9 = LOW_POWER_OFF_CHARGING_BOOT */
        if(chip->bootmode == 8 || chip->bootmode == 9)
            cps_rx_online_check(chip);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_READY");
     }
     if(int_flag & RX_INT_FSK_ACK){
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ: RX_INT_FSK_ACK");
     }
     if(int_flag & RX_INT_FSK_TIMEOUT){
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ: RX_INT_FSK_ACK");
     }
     if(int_flag & RX_INT_FSK_PKT){
           cps_wls_get_fsk_packet(data);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ: RX_INT_FSK_PKT");
     }
    if(int_flag & RX_INT_OVP){
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OVP");
    }
    if(int_flag & RX_INT_AC_LOSS){}
    if(int_flag & RX_INT_OVP_TO){}
    if(int_flag & RX_INT_AC_SHORT){}
    if(int_flag & RX_INT_OTP){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OTP");
    }
    if(int_flag & RX_INT_SR_OCP){
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_OCP");
    }
    if(int_flag & RX_INT_OCP){}
    if(int_flag & RX_INT_HOCP){
          CPS_RX_MODE_ERR = true;

        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_HOCP");
    }
    if(int_flag & RX_INT_SCP){
        cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_SCP");
    }
    if(int_flag & RX_INT_SR_SW_R){}
    if(int_flag & RX_INT_SR_SW_F){}
    if(int_flag & RX_INT_FC_OK){}
//    if(int_flag & RX_INT_HS_OK){}
    if(int_flag & RX_INT_HTP){}
//    if(int_flag & RX_INT_HS_FAIL){}
    if(int_flag & RX_INT_FC_FAIL){}
    if(int_flag & RX_INT_NEGO_POWER_READY)
	{
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_NEGO_READY");
		cps_epp_icl_on();
	}

	if ((int_flag & RX_INT_HS_OK) || (int_flag & RX_INT_HS_FAIL))
	{
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  RX_INT_HS_OK or RX_INT_HS_FAIL");
		cps_get_sys_op_mode(&mode_type);
		cps_wls_log(CPS_LOG_DEBG, "[%s] op_mode %d\n", __func__, mode_type);
		if (mode_type == Sys_Op_Mode_MOTO_WLC)
		{
			chip->moto_stand = true;
			cps_wls_set_status(WLC_TX_TYPE_CHANGED);
		}
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
    if(int_flag & TX_INT_ASK_PKT) {}
    {
        rc = cps_wls_get_ask_packet(data);
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
    if(int_flag & TX_INT_ASK_ALL){}
    if(int_flag & TX_INT_RPP_TYPE_ERR){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_RP_TYPR_ERR");
    }
    if(int_flag & TX_INT_RP_ASK_ACK){
         cps_wls_log(CPS_LOG_DEBG, " CPS_WLS IRQ:  TX_INT_ASK_PKT");
    }
    if(int_flag & TX_INT_PING_OVP){}

    return rc;
}

static int mmi_mux_wls_chg_chan(enum mmi_mux_channel channel, bool on)
{
	struct charger_manager *info = NULL;
	struct charger_device *chg_psy = NULL;

	chg_psy = get_charger_by_name("primary_chg");
	if(chg_psy) {
		info = (struct charger_manager *)charger_dev_get_drvdata(chg_psy);
		if(info)
			cps_wls_log(CPS_LOG_ERR,"%s could  get charger_manager\n",__func__);
		else {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get charger_manager\n",__func__);
			return CPS_WLS_SUCCESS;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n",__func__);
		return CPS_WLS_SUCCESS;
	}

	cps_wls_log(CPS_LOG_ERR, "%s open wlc chan =%d, on = %d\n", __func__, channel, on);
	if (info->do_mux)
		info->do_mux(info, channel, on);
	else
		cps_wls_log(CPS_LOG_ERR, "%s get info->algo.do_mux fail", __func__);

	return CPS_WLS_SUCCESS;
}
#if 0
static int cps_wls_rx_power_on()
{
#if 1
	uint32_t chip_id = 0,sys_mode = 0;
	uint32_t online = 0;

	sys_mode = cps_wls_get_sys_mode();
	cps_wls_log(CPS_LOG_DEBG, "CPS_TX_MODE %d, sys_mode RX/TX, mode %d", CPS_TX_MODE, sys_mode);

	chip_id = cps_wls_get_chip_id();
	cps_wls_log(CPS_LOG_ERR, "cps get chip id: %x\n",chip_id);
	if( sys_mode == SYS_MODE_RX) {
		online = true;
	} else {
		online = false;
	}
	return online;
#else
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
	cps_wls_log(CPS_LOG_DEBG, "CPS_TX_MODE %d, sys_mode RX/TX, mode %d", CPS_TX_MODE, sys_mode);

	chip_id = cps_wls_get_chip_id();
	cps_wls_log(CPS_LOG_ERR, "cps get chip id: %x\n",chip_id);
	if(chip_id == 0x4035 && sys_mode == SYS_MODE_RX) {
		rx_power = true;
		rx_power_cnt = 0;
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
#endif
}
#endif
static void cps_rx_online_check(struct cps_wls_chrg_chip *chg)
{
    bool wls_online = false;
    struct cps_wls_chrg_chip *chip = chg;

    wls_online = cps_wls_rx_power_on();
    if(!chip->wls_online && wls_online) {
        chip->wls_online = true;
        mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, true);
        power_supply_changed(chip->wl_psy);
    }
    if(chip->wls_online && !wls_online){
        chip->wls_online = false;
        mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_CHG, false);
        power_supply_changed(chip->wl_psy);
    }
}

#define WLS_ICL_INCREASE_STEP 100000
int g_bpp_wlc_icl = 0;
static void cps_bpp_mode_icl_work(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chg = chip;
	int wls_icl = 0;

	wls_icl = 100000;
	chg->bpp_icl_done = false;
	g_bpp_wlc_icl = 0 ;
	while((wls_icl + WLS_ICL_INCREASE_STEP) <= 900000) {
		if(!chg->wls_online)
			break;
		wls_icl += WLS_ICL_INCREASE_STEP;
		msleep(200);
		if (chip->chg1_dev)
			charger_dev_set_input_current(chip->chg1_dev, wls_icl);
		else
		{
			cps_init_charge_hardware();
			charger_dev_set_input_current(chip->chg1_dev, wls_icl);
		}
		g_bpp_wlc_icl = wls_icl;
		cps_wls_log(CPS_LOG_DEBG, "cps wireless charging icl %d ua\n", wls_icl);
	}
	chg->bpp_icl_done = true;
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
    int int_flag;
    int int_clr;
    struct cps_wls_chrg_chip *chip = dev_id;
    cps_wls_log(CPS_LOG_DEBG, "[%s] IRQ triggered\n", __func__);
    mutex_lock(&chip->irq_lock);
    //if(cps_wls_get_chip_id() != 0x4035)
    //{
        /*unlock i2c*/
        //cps_wls_h_write_reg(REG_PASSWORD, PASSWORD);
       // cps_wls_h_write_reg(REG_HIGH_ADDR, HIGH_ADDR);
       // cps_wls_h_write_reg(REG_WRITE_MODE, WRITE_MODE);
        cps_wls_set_int_enable();
        chip->chip_state = true;
   //     cps_wls_set_status(WLC_DISCONNECTED);
     //   cps_wls_log(CPS_LOG_DEBG, "[%s] CPS_I2C_UNLOCK", __func__);
   // }

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

    cps_wls_log(CPS_LOG_DEBG, "cps_wls_get_sys_mode:%d\n",cps_wls_get_sys_mode() );

    return IRQ_HANDLED;
}

static irqreturn_t wls_det_irq_handler(int irq, void *dev_id)
{
	struct cps_wls_chrg_chip *chip = dev_id;
	int tx_detected = gpio_get_value(chip->wls_det_int);

	if (tx_detected) {
//		if (chip->factory_wls_en == true)
//			mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, true);
		cps_wls_log(CPS_LOG_DEBG, "Detected an attach event.\n");
	} else {
		cps_wls_log(CPS_LOG_DEBG, "Detected an detach event.\n");
		chip->rx_int_ready = false;
		chip->bpp_icl_done = false;
		if (chip->rx_ldo_on) {
			//chip->wls_online = false;
			cps_wls_set_status(WLC_DISCONNECTED);
		cps_rx_online_check(chip);
		chip->rx_ldo_on = false;
	//	if (chip->factory_wls_en == true) {
	//		chip->factory_wls_en = false;
	//		mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, false);
	//	}
		power_supply_changed(chip->wl_psy);
		}
	}
	return IRQ_HANDLED;
}
void wlc_control_pin_set(bool on);
static int cps_get_bat_soc()
{
	union power_supply_propval prop;
	struct power_supply *battery_psy = NULL;

	battery_psy = power_supply_get_by_name("battery");
	if (battery_psy == NULL || IS_ERR(battery_psy)) {
		cps_wls_log(CPS_LOG_ERR,"%s mmi_mux Couldn't get battery_psy\n",__func__);
		return CPS_WLS_FAIL;
	} else {
		power_supply_get_property(battery_psy,
			POWER_SUPPLY_PROP_CAPACITY, &prop);
		cps_wls_log(CPS_LOG_ERR,"%s battery soc:%d\n", __func__, prop.intval);
	}
	return prop.intval;
}
static bool usb_online()
{
	union power_supply_propval prop;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("charger");
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

void cps_wls_vbus_enable(bool en)
{
	int ret = 0;
	struct charger_manager *info = NULL;
	struct charger_device *chg_psy = NULL;
	static bool otg_status = 0;

	chg_psy = get_charger_by_name("primary_chg");
	if(chg_psy) {
		info = (struct charger_manager *)charger_dev_get_drvdata(chg_psy);
		if(info)
			cps_wls_log(CPS_LOG_ERR,"%s could  get charger_manager\n",__func__);
		else {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get charger_manager\n",__func__);
			return ;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n",__func__);
		return ;
	}

	if((usb_online() == true || otg_status == en) && otg_status != true) {
		cps_wls_log(CPS_LOG_ERR,"%s usb online or otg status same, no need switch\n",__func__);
		return;
	}

	ret = charger_dev_enable_otg(chg_psy, en);
	if(ret < 0){
		cps_wls_log(CPS_LOG_ERR,"%s enable otg fail\n",__func__);
	}
	otg_status = en;
       mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_OTG, en);

	return ;
}
static int wireless_en(void *input, bool en)
{
	int ret = 0;
//	struct chg_alg_device *alg;
	struct charger_manager *info = NULL;
	struct charger_device *chg_psy = NULL;

	chg_psy = get_charger_by_name("primary_chg");
	if(chg_psy) {
		info = (struct charger_manager *)charger_dev_get_drvdata(chg_psy);
		if(info)
			cps_wls_log(CPS_LOG_ERR,"%s could  get charger_manager\n",__func__);
		else {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get charger_manager\n",__func__);
			return CPS_WLS_SUCCESS;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n",__func__);
		return CPS_WLS_SUCCESS;
	}

	wlc_control_pin_set(true);
	msleep(1000);
	wlc_control_pin_set(false);
//	       mmi_mux_wls_chg_chan(MMI_MUX_CHANNEL_WLC_FACTORY_TEST, en);
	chip->factory_wls_en = en;
	cps_wls_log(CPS_LOG_ERR,"wls: wls_en %d\n",en);
	return ret;
}
static int wireless_get_chip_id(void *input)
{
	int value = chip->chip_id;

	if(0 != value)
		return value;

	cps_wls_vbus_enable(true);
	msleep(500);
	value = cps_wls_get_chip_id();
	chip->chip_id = value;
	cps_wls_vbus_enable(false);
	return value;
}

static int  wls_tcmd_register(struct cps_wls_chrg_chip *cm)
{
	int ret;

	cm->wls_tcmd_client.data = cm;
	cm->wls_tcmd_client.client_id = MOTO_CHG_TCMD_CLIENT_WLS;

	cm->wls_tcmd_client.get_chip_id = wireless_get_chip_id;
	cm->wls_tcmd_client.wls_en = wireless_en;

	ret = moto_chg_tcmd_register(&cm->wls_tcmd_client);

	return ret;
}
static enum power_supply_property cps_wls_chrg_props[] = {
    POWER_SUPPLY_PROP_CURRENT_MAX,
//    POWER_SUPPLY_PROP_CHARGING_ENABLED,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_MAX,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_MAX,
    POWER_SUPPLY_PROP_CURRENT_NOW,
//    POWER_SUPPLY_PROP_VOUT_NOW,
//    POWER_SUPPLY_PROP_VRECT,
//    POWER_SUPPLY_PROP_IRECT,
//    POWER_SUPPLY_PROP_PROTOCOL,
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
            //val->intval = cps_wls_is_ldo_on();
            val->intval = chip->wls_online;
            if (!chip->chip_state)
            {
                val->intval = chip->chip_state;
            }
            break;

        case POWER_SUPPLY_PROP_TYPE:
            val->intval = chip->wl_psd.type;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_MAX:
            val->intval = chip->MaxV * 1000;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = cps_wls_get_rx_vout() * 1000;
            break;

        case POWER_SUPPLY_PROP_CURRENT_MAX:
            val->intval = chip->MaxI * 1000;
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
	int status = CPS_WLS_SUCCESS;
	cps_reg_s *cps_reg;
	uint32_t fw_major_revision = 0;
	uint32_t fw_minor_revision = 0;
	uint32_t fw_version = 0;

	cps_reg = (cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_VER]);
	fw_version =  cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
	if(CPS_WLS_SUCCESS != status)
	{
		cps_wls_log(CPS_LOG_ERR, "%s failed",__func__);
	}

	if(CPS_WLS_SUCCESS == status) {
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
static void wireless_chip_reset()
{
#if 1
	wlc_control_pin_set(true);
	msleep(100);
	wlc_control_pin_set(false);
    cps_wls_log(CPS_LOG_DEBG,"%s\n", __func__);
#else
	struct chg_alg_device *alg;

	alg = get_chg_alg_by_name("wlc");
	if (NULL != alg) {
		chg_alg_set_prop(alg, ALG_WLC_STATE, false);
		msleep(100);
		chg_alg_set_prop(alg, ALG_WLC_STATE, true);
	}
#endif
	return;
}
static void cps_wls_set_boost(int val)
{
	/* Assume if we turned the boost on we want to stay awake */
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
	int buf0_flag = 0, buf1_flag = 0;
	u32 version;
	u16 maj_ver, min_ver;
	u8 *firmware_buf;
	int result;
	int rc,i;
	u32 fw_revision;
	const struct firmware *fw;
	int cfg_buf_size;
		int addr,ret = CPS_WLS_SUCCESS;
	CPS_TX_MODE = true;
	chip->fw_uploading = true;
	cps_wls_set_boost(1);

	firmware_buf = kzalloc(0x6000, GFP_KERNEL);  // 24K buffer
	rc = firmware_request_nowarn(&fw, chip->wls_fw_name, chip->dev);
	if (rc) {
		cps_wls_log(CPS_LOG_ERR,"Couldn't get firmware  rc=%d\n", rc);
        ret = CPS_WLS_FAIL;
		goto update_fail;
	}
	if(cps_get_bat_soc() < 10) {
		cps_wls_log(CPS_LOG_ERR,"Couldn't updata firmware because of low battery soc\n");
        ret = CPS_WLS_FAIL;
		goto update_fail;
	}

	//cps_wls_vbus_enable(true);
	//msleep(500);

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
	    cps_wls_log(CPS_LOG_DEBG,"%s bin version %x same as fw version %x,no need update fw\n",__func__,version,fw_revision);
	    ret = CPS_WLS_SUCCESS;
	    goto free_bug;
	}

	//CPS4035_BL
	//bootloader_buf = CPS4035_BOOTLOADER;
	if(CPS_WLS_FAIL == cps_wls_write_word(0xFFFFFF00, 0x0000000E)) {
         ret = CPS_WLS_FAIL;
         goto free_bug;/*enable 32bit i2c*/
    }
	if(CPS_WLS_FAIL == cps_wls_write_word(0x4000E75C, 0x00001250)) {
        ret = CPS_WLS_FAIL;
        goto free_bug;//*write password*/
    }
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40040010, 0x00000006)) {
        ret = CPS_WLS_FAIL;
        goto free_bug;/*reset and halt mcu*/
    }
    cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM HEX!\n", __func__);
	if(CPS_WLS_FAIL == cps_wls_program_sram(0x20000000, CPS4038_BOOTLOADER, 0x800)) {
        ret = CPS_WLS_FAIL;
        goto free_bug;//program sram
    }
	if(CPS_WLS_FAIL == cps_wls_write_word(0x400400A0, 0x000000FF)) {
        ret = CPS_WLS_FAIL;
        goto free_bug;//remap enable
    }
	if(CPS_WLS_FAIL == cps_wls_write_word(0x40040010, 0x00008003)) {
        ret = CPS_WLS_FAIL;
        goto free_bug;/*triming load function is disabled and run mcu*/
    }

	msleep(10);

	if(CPS_WLS_FAIL == cps_wls_write_word(0xFFFFFF00, 0x0000000E)) {
        ret = CPS_WLS_FAIL;
        goto free_bug; /*enable 32bit i2c*/
    }
    msleep(10);

	//=========================================================
	// cali bootloader code
	//=========================================================
	cps_wls_program_cmd_send(CACL_CRC_TEST);
	result = cps_wls_program_wait_cmd_done();

	if(result != PASS)
	{
	    cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader crc fail\n", __func__);
        ret = CPS_WLS_FAIL;
	    goto free_bug;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load bootloader successful\n", __func__);

	//=========================================================
	// LOAD firmware to MTP
	//=========================================================
	memset(firmware_buf, 0, 0x6000);
	memcpy(firmware_buf,fw->data,fw->size);
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load fw size %x\n", __func__,(int)fw->size);
      buf0_flag    = 0;
      buf1_flag    = 0;
      cfg_buf_size = 512;
      addr = 0;
      cps_wls_write_word(ADDR_BUF_SIZE, cfg_buf_size);

      result =  cps_wls_program_wait_cmd_done( );
       if(result != PASS)
       {
             cps_wls_log(CPS_LOG_ERR,   "[%s]  ---> ERASE FAIL\n", __func__);
             goto free_bug;
       }

        for(i = 0; i < (24 * 1024) / 4 / cfg_buf_size; i++)
        {
                      if(buf0_flag == 0)
                      {
                               cps_wls_program_sram(ADDR_BUFFER0, firmware_buf+addr, cfg_buf_size*4);
                               addr  = addr  + cfg_buf_size*4;

                               if(buf1_flag == 1)
                               {
                                     result = cps_wls_program_wait_cmd_done( );
                                     if(result != PASS)
                                     {
                                              pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n",__func__);
						    ret = CPS_WLS_FAIL;
                                              goto free_bug;
                                     }
                                     buf1_flag = 0;
                               }
                               cps_wls_program_cmd_send(PGM_BUFFER0);
                               buf0_flag = 1;
                               continue;
         }

         if(buf1_flag == 0)
         {
                               cps_wls_program_sram(ADDR_BUFFER1, firmware_buf+addr, cfg_buf_size*4);
                               addr  = addr  + cfg_buf_size*4;

                               if(buf0_flag == 1)
                               {
                                       result =  cps_wls_program_wait_cmd_done( );
                                       if(result != PASS)
                                       {
                                                pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n",__func__);
						      ret = CPS_WLS_FAIL;
                                               goto free_bug;
                                       }
                                       buf0_flag = 0;
                                }
                                cps_wls_program_cmd_send(PGM_BUFFER1);
                                buf1_flag = 1;
                                continue;
                        }
         }

         if(buf0_flag == 1)
         {
                        result = cps_wls_program_wait_cmd_done( );
                        if(result != PASS)
                        {
                               pr_err("%s: ---> WRITE BUFFER0 DATA TO MTP FAIL\n",__func__);
                               goto free_bug;
                        }
                        buf0_flag = 0;
         }

         if(buf1_flag == 1)
         {
                        result = cps_wls_program_wait_cmd_done( );
                        if(result != PASS)
                        {
                               pr_err("%s: ---> WRITE BUFFER1 DATA TO MTP FAIL\n",__func__);
                              goto free_bug;
                        }
                        buf1_flag = 0;
         }
         pr_info("%s: ---> LOAD APP HEX SUCCESSFUL\n",__func__);
         /***************************************************************************************
                            *                          Step4, check app CRC                                       *
         ***************************************************************************************/
        cps_wls_program_cmd_send(CACL_CRC_APP);
        result = cps_wls_program_wait_cmd_done( );

        if(result != PASS)
        {
                       pr_err("%s: ---> APP CRC FAIL\n",__func__);
                       goto free_bug;
        }
        pr_info("%s: ---> CHERK APP CRC SUCCESSFUL\n",__func__);
       /***************************************************************************************
                         *                          Step5, write mcu start flag                                *
       ***************************************************************************************/
      cps_wls_program_cmd_send(PGM_WR_FLAG);
      result = cps_wls_program_wait_cmd_done( );
      if(result != PASS)
      {
               pr_err("%s:---> WRITE MCU START FLAG FAIL\n",__func__);
	        ret = CPS_WLS_FAIL;
               goto free_bug;
       }

       pr_info("%s: ---> WRITE MCU START FLAG SUCCESSFUL\n",__func__);
       cps_wls_write_word(0x40040010, 0x00000008); /*reset all system*/
       msleep(100);
       cps_wls_write_word(0xFFFFFF00, 0x00000000); /*i2c 32bit mode disable*/
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- Program successful\n", __func__);

free_bug:
	cps_wls_set_boost(0);//disable power, after FW updating, need a power reset
	//cps_wls_vbus_enable(false);
	msleep(20);//20ms
	kfree(firmware_buf);
	release_firmware(fw);
	CPS_TX_MODE = false;
	chip->fw_uploading = false;
	wireless_chip_reset();
	return ret;

update_fail:
	CPS_TX_MODE = false;
	chip->fw_uploading = false;
	cps_wls_set_boost(0);
    cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);
    return ret;
}
static void cps_firmware_update_work(struct work_struct *work)
{
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

	int rc;
	uint32_t  fw_version;

	cps_wls_vbus_enable(true);
	msleep(300);
	rc = cps_get_fw_revision(&fw_version);
	cps_wls_vbus_enable(false);

	return sprintf(buf, "%08x\n", fw_version);
}

static DEVICE_ATTR(wireless_fw_version, 0664, wireless_fw_version_show, NULL);

static ssize_t wireless_fw_force_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	cps_wls_vbus_enable(true);
	msleep(300);
	rc = wireless_fw_update(true);
	cps_wls_vbus_enable(false);
	if (rc < 0)
		return rc;

	return count;
}
static DEVICE_ATTR(wireless_fw_force_update, 0664, NULL, wireless_fw_force_update_store);

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
static DEVICE_ATTR(wireless_fw_update, 0664, NULL, wireless_fw_update_store);

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
	int rx_connected;
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"wls: chip not valid\n");
		return -ENODEV;
	}

	if (chip->ntc_thermal) {
		rx_connected = TX_MODE_OVERHEAT;
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
	cps_wls_log(CPS_LOG_DEBG,"wls input_current = %lu\n", wls_input_curr_max);
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
		return sprintf(buf, "%d\n", Sys_Op_Mode_MOTO_WLC);
	else
		return sprintf(buf, "%d\n", chip->mode_type);
}

static DEVICE_ATTR(wlc_tx_type, 0444, show_wlc_tx_type, NULL);

static ssize_t show_wlc_st_changed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", chip->wlc_status);
}

static DEVICE_ATTR(wlc_st_changed, S_IRUGO, show_wlc_st_changed, NULL);

static void cps_wls_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
//-----------------------program---------------------
//    device_create_file(dev, &dev_attr_update_fw);
    device_create_file(dev, &dev_attr_wireless_fw_version);
    device_create_file(dev, &dev_attr_wireless_fw_update);
    device_create_file(dev, &dev_attr_wireless_fw_force_update);
//-----------------------write password--------------
    //device_create_file(dev, &dev_attr_write_password);

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

    device_create_file(dev, &dev_attr_wlc_fan_speed);
    device_create_file(dev, &dev_attr_wlc_light_ctl);
    device_create_file(dev, &dev_attr_wlc_tx_power);
    device_create_file(dev, &dev_attr_wlc_tx_type);
    device_create_file(dev, &dev_attr_wlc_st_changed);
}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
    struct device_node *node = chip->dev->of_node;
	struct device_node *boot_node = NULL;
	struct tags_bootmode *tag = NULL;

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

    chip->wls_mode_select = of_get_named_gpio(node, "mmi,wls_mode_select", 0);
    if(!gpio_is_valid(chip->wls_mode_select))
		cps_wls_log(CPS_LOG_ERR,"mmi wls_mode_select is %d invalid\n", chip->wls_mode_select );

    of_property_read_string(node, "wireless-fw-name", &chip->wls_fw_name);


    cps_wls_log(CPS_LOG_ERR, "[%s]  wls_charge_int %d wls_det_int %d wls_fw_name: %s\n",
             __func__, chip->wls_charge_int, chip->wls_det_int,chip->wls_fw_name);

    return 0;
}

static int cps_wls_gpio_request(struct cps_wls_chrg_chip *chip)
{
    int ret =0;

	ret = devm_gpio_request_one(chip->dev, chip->wls_charge_int,
				  GPIOF_IN, "cps4038_ap_int");
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
				  GPIOF_IN, "cps4038_wls_det_int");
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
		ret  = devm_gpio_request_one(chip->dev, chip->wls_mode_select,
				  GPIOF_OUT_INIT_HIGH, "wls_mode_select");
		if (ret  < 0)
			cps_wls_log(CPS_LOG_ERR," [%s] Failed to request wls_mode_select gpio, ret:%d", __func__, ret);
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
//    wake_lock_destroy(&chip->cps_wls_wake_lock);
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
#define WLS_RX_CAP_15W 15
#define WLS_RX_CAP_10W 10
#define WLS_RX_CAP_8W 8
#define WLS_RX_CAP_5W 5
static void cps_wls_current_select(int  *icl, int *vbus, bool *cable_ready)
{
    struct cps_wls_chrg_chip *chg = chip;
    uint32_t wls_power = 0;

    cps_wls_log(CPS_LOG_DEBG, "%s", __func__);

    if (chip->cable_ready_wait_count < 0 && !chip->moto_stand)
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
           *icl = g_bpp_wlc_icl;//100000;
           *vbus = 5000;
           return;
        }
        chg->MaxV = 5000;
        chg->MaxI = 1000;
        *icl = 900000;
        *vbus = 5000;
    }
    else if (chg->mode_type == Sys_Op_Mode_EPP)
    {
        wls_power = cps_wls_get_rx_neg_power() / 2;
        cps_wls_log(CPS_LOG_DEBG, "%s cps4038 power %dW", __func__, wls_power);
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
            chg->MaxV = 12000;
            chg->MaxI = 400;
            *icl = 400000;
            *vbus = 12000;
        }
        else
        {
            chg->MaxV = 5000;
            chg->MaxI = 1000;
            *icl = 1000000;
            *vbus = 5000;
        }
    }
    if (chip->wls_input_curr_max != 0)
        *icl = chip->wls_input_curr_max * 1000;
}

static void cps_epp_current_select(int  *icl, int *vbus)
{
    struct cps_wls_chrg_chip *chg = chip;
    uint32_t wls_power = 0;

    *icl = 400000;
    *vbus = 5000;
    if (chg->mode_type == Sys_Op_Mode_EPP)
    {
        wls_power = cps_wls_get_rx_neg_power() / 2;
        cps_wls_log(CPS_LOG_DEBG, "%s cps4038 power %dW", __func__, wls_power);
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
            chg->MaxV = 12000;
            chg->MaxI = 400;
            *icl = 400000;
            *vbus = 12000;
        }
        else
        {
            chg->MaxV = 5000;
            chg->MaxI = 1000;
            *icl = 1000000;
            *vbus = 5000;
        }
    }
}

static int cps_wls_wlc_update_light_fan(void)
{
	int status = CPS_WLS_SUCCESS;
	uint8_t data[4] = {0x38, 0x05, 0x40, 0x14};
	int retry = 2;

	if (CPS_RX_CHRG_FULL)
	{
		data[2] = MMI_DOCK_LIGHT_ON;
		data[3] = MMI_DOCK_FAN_SPEED_OFF;
	}
	else
	{
		if (chip->fan_speed == 0)
			data[3] = MMI_DOCK_FAN_SPEED_LOW;
		else
			data[3] = MMI_DOCK_FAN_SPEED_HIGH;

		if (chip->light_level == 0)
			data[2] = MMI_DOCK_LIGHT_OFF;
		else
			data[2] = MMI_DOCK_LIGHT_DEFAULT;
	}
	do
	{
//		status = cps_wls_send_fsk_packet(data, 4);
		status  = cps_wls_send_ask_packet(data, 4);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS: QI set fan/light, WLS_WLC_FAN_SPEED %d, CPS_RX_CHRG_FULL %d, WLS_WLC_LIGHT %d",
					chip->fan_speed, CPS_RX_CHRG_FULL, chip->light_level);
		cps_wls_log(CPS_LOG_DEBG, " CPS_WLS: QI set fan/light, ight 0x%x, fan 0x%x", data[2], data[3]);
		msleep(200);
		retry--;
	} while (retry);
	return 0;
}

static void  cps_wls_notify_tx_chrgfull(void)
{
	int status = CPS_WLS_SUCCESS;
	uint8_t data[2] = {0x5, 0x64};
    int retry = 5;

    do {
//        status = cps_wls_send_fsk_packet(data, 2);
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
		cps_wls_log(CPS_LOG_DEBG,"%s st change  %d -> %d\n",__func__, pre_status, chip->wlc_status);
		pre_status = chip->wlc_status;
		sysfs_notify(&chip->wl_psy->dev.parent->kobj, NULL, "wlc_st_changed");
	}

	return 0;
}

static int cps_wls_set_status(int status)
{
	chip->wlc_status = status;
	if (status == WLC_DISCONNECTED)
		chip->moto_stand = false;
	cps_wls_notify_st_changed();
	cps_wls_log(CPS_LOG_DEBG, "cps_wls_set_status:%d",status);
	return 0;
}

static void cps_wls_set_battery_soc(int uisoc)
{
    struct cps_wls_chrg_chip *chg = chip;
	int soc = uisoc;

	if (!CPS_RX_CHRG_FULL && chg->wls_online && soc == 100) {
		cps_wls_log(CPS_LOG_DEBG, "cps Notify TX, battery has been charged full !");
		if (chip->moto_stand) {
			cps_wls_wlc_update_light_fan();
		} else
			cps_wls_notify_tx_chrgfull();
		CPS_RX_CHRG_FULL = true;
	}
}

static int  wls_chg_ops_register(struct cps_wls_chrg_chip *cm)
{
	int ret;

	cm->wls_chg_ops.wls_current_select = cps_wls_current_select;
	cm->wls_chg_ops.wls_set_battery_soc = cps_wls_set_battery_soc;

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
	if(chip_id == 0x4038 && sys_mode == SYS_MODE_RX) {
		rx_power = true;
		rx_power_cnt = 0;
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

//		if (rx_power_cnt > 1) {
		if ( 1) {
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

static int cps_get_vbus(void)
{
	struct charger_manager *info = NULL;
	int ret = 0;
	int vchr = 0;
	struct charger_device *chg_psy = NULL;

	chg_psy = get_charger_by_name("primary_chg");
	if(chg_psy) {
		info = (struct charger_manager *)charger_dev_get_drvdata(chg_psy);
		if(info)
			cps_wls_log(CPS_LOG_ERR,"%s could  get charger_manager\n",__func__);
		else {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get charger_manager\n",__func__);
			return CPS_WLS_SUCCESS;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n",__func__);
		return CPS_WLS_SUCCESS;
	}

	ret = charger_dev_get_vbus(chg_psy, &vchr);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,"%s get vbus failed: %d\n", __func__, ret);
		return ret;
	}

	vchr = vchr / 1000;
//	vbus = get_vbus(info);
	cps_wls_log(CPS_LOG_ERR, "%s: vbus:%d\n", __func__,vchr);
	return vchr;
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
		if(cps_get_vbus() < 5000  || !chip->rx_int_ready)
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
//		cps_wls_tx_enable(false);
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

static void cps_init_charge_hardware()
{
	struct cps_wls_chrg_chip *chg = chip;
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
    chip->regmap = devm_regmap_init_i2c(client, &cps4038_regmap_config);
    if (IS_ERR(chip->regmap)) {
        cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
        devm_kfree(&client->dev, chip);
        return PTR_ERR(chip->regmap);
    }
    chip->regmap32 = devm_regmap_init_i2c(client, &cps4038_regmap32_config);
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
    if(gpio_is_valid(chip->wls_mode_select)) {
        gpio_set_value(chip->wls_mode_select, true);
    }

    cps_wls_lock_work_init(chip);

    cps_wls_create_device_node(&(client->dev));

    ret = cps_wls_register_psy(chip);
    if(IS_ERR(chip->wl_psy)){
        cps_wls_log(CPS_LOG_ERR, "[%s] power_supply_register wireless failed , ret = %d\n", __func__, ret);
        goto free_source;
    }
    wls_tcmd_register(chip);
    chip->rx_connected = false;
    chip->wls_online = false;
    wls_chg_ops_register(chip);
    chip->wls_input_curr_max = 0;
    chip->MaxV = 12000;
    chip->MaxI = 1150;
    chip->chip_id = 0;
    chip->factory_wls_en = false;
    chip->mode_type = Sys_Op_Mode_INVALID;
    chip->fan_speed = 0;
    chip->light_level = 1;
    chip->moto_stand = false;

    init_waitqueue_head(&chip->wait_que);
    wls_rx_init_timer(chip);
	chip->wls_wq = create_singlethread_workqueue("wls_workqueue");
	INIT_DELAYED_WORK(&chip->fw_update_work,
			  cps_firmware_update_work);
	INIT_DELAYED_WORK(&chip->bpp_icl_work,
			  cps_bpp_mode_icl_work);

    /* Register thermal zone cooling device */
    chip->tcd = thermal_of_cooling_device_register(dev_of_node(chip->dev),
		"cps_wls_charger_l", chip, &cps_tcd_ops);
    cps_init_charge_hardware();

    if(chip->bootmode == 8 || chip->bootmode == 9)
        wireless_chip_reset();
//    wake_lock(&chip->cps_wls_wake_lock);
    kthread_run(cps_rx_check_events_thread, chip, "cps_rx_check_thread");
    cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger  probe successful!\n", __func__);
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
    rc = cps_wls_set_tx_fod1_thresh(3000);
    rc = cps_wls_set_tx_fod2_thresh(3000);
    rc = cps_wls_set_tx_fod3_thresh(3000);
    rc = cps_wls_set_tx_fod4_thresh(3000);
    rc = cps_wls_set_tx_fod5_thresh(3000);
    rc = cps_wls_set_tx_fod6_thresh(3000);
    rc = cps_wls_set_tx_fod7_thresh(3000);   
    rc = cps_wls_set_tx_fod_rp0_thresh(20);
    rc = cps_wls_set_tx_fod_rp1_thresh(40);
    rc = cps_wls_set_tx_fod_rp2_thresh(60);
    rc = cps_wls_set_tx_fod_rp3_thresh(80);
    rc = cps_wls_set_tx_fod_rp4_thresh(100);
    rc = cps_wls_set_tx_fod_rp5_thresh(120);
    rc = cps_wls_set_tx_fod_rp6_thresh(140);
    rc = cps_wls_set_tx_fod_rp7_thresh(160);
    rc = cps_wls_enable_tx_mode();
    rc = cps_wls_disable_tx_mode();
    rc = cps_wls_send_fsk_packet(data, 2);
    rc = cps_wls_set_fod_para();
    return;
}

static int cps_wls_chrg_remove(struct i2c_client *client)
{
    not_called_api();
    //cps_wls_lock_destroy(chip);
    kfree(chip);
    return 0;
}

static const struct i2c_device_id cps_wls_charger_id[] = {
    {"cps-wls-charger", 0},
    {}, 
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
    { .compatible = "cps,wls-charger-cps4038", .data = NULL},
    {},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps_wls_charger_driver = {
    .driver = {
        .name       = CPS_WLS_CHRG_DRV_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = cps_wls_chrg_of_tbl,
    },
    .probe      = cps_wls_chrg_probe,
    .remove     = cps_wls_chrg_remove,
    .id_table   = cps_wls_charger_id,
};

static int __init cps_wls_driver_init(void)
{
    return (i2c_add_driver(&cps_wls_charger_driver));
}

late_initcall(cps_wls_driver_init);

static void __exit cps_wls_driver_exit(void)
{
    i2c_del_driver(&cps_wls_charger_driver);

}

module_exit(cps_wls_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("jian.deng@convenientpower.com");
MODULE_DESCRIPTION("cps_wls_charger driver");
MODULE_ALIAS("i2c:cps_wls_charger");

