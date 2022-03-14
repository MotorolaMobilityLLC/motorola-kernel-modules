/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __CPS_WLS_CHARGER_H__
#define __CPS_WLS_CHARGER_H__
#include <linux/workqueue.h>
#include "mtk_charger.h"
#include "moto_wlc.h"
#ifdef SMART_PEN_SUPPORT
#include "smart_pen_charger.h"
#endif
#define CPS_WLS_FAIL    -1
#define CPS_WLS_SUCCESS 0

/*****************************************************************************
 *  CMD REG
 ****************************************************************************/
#define REG_PASSWORD            0xF500
#define REG_RESET_MCU           0xF501
#define REG_HIGH_ADDR           0xF503
#define REG_WRITE_MODE          0xF505

#define PASSWORD                0x19E5
#define CPS_4035_RESET          0x153F
#define VERIFI_ADDR             0x1F30
#define HIGH_ADDR               0x2000
#define WRITE_MODE              0x0004  /*0x0004 : byte write    0x0005 : halfword write    0x0006 : word write*/
#define PROGRAM_WRITE_MODE      0x0006
#define VERIFI_VAL              0xAA55
/****************************************************************************
 * BOOTLOADER CMD
****************************************************************************/
#define CPS_PROGRAM_BUFFER_SIZE   512         /*64 128 256 512*/
#define ADDR_BUFFER0              0x20000800
#define ADDR_BUFFER1              0x20001000
#define ADDR_CMD                  0x20001800
#define ADDR_FLAG                 0x20001804
#define ADDR_BUF_SIZE             0x20001808
#define ADDR_FW_VER               0x2000180C

#define PGM_BUFFER0               0x10
#define PGM_BUFFER1               0x20
#define PGM_BUFFER2               0x30
#define PGM_BUFFER3               0x40
#define PGM_BUFFER0_1             0x50
#define PGM_ERASER_0              0x60
#define PGM_ERASER_1              0x70
#define PGM_WR_FLAG               0x80

#define CACL_CRC_APP              0x90
#define CACL_CRC_TEST             0xB0

#define PGM_ADDR_SET              0xC0

#define RUNNING                   0x66
#define PASS                      0x55
#define FAIL                      0xAA
#define ILLEGAL                   0x40

//****************system mode***************
#define SYS_MODE_BACK_POWER 1
#define SYS_MODE_TX         2
#define SYS_MODE_RX         3

/*rx中断定义*/
#define RX_INT_POWER_ON         (0x01<<0)
#define RX_INT_LDO_OFF          (0x01<<1)
#define RX_INT_LDO_ON           (0x01<<2)
#define RX_INT_READY            (0x01<<3)
#define RX_INT_RESERVE1         (0x01<<4)
#define RX_INT_RESERVE2         (0x01<<5)
#define RX_INT_RESERVE3         (0x01<<6)
#define RX_INT_OVP              (0x01<<7)
#define RX_INT_OTP              (0x01<<8)
#define RX_INT_OCP              (0x01<<9)
#define RX_INT_HOCP             (0x01<<10)
#define RX_INT_SCP              (0x01<<11)
#define RX_INT_RESERVE4         (0x01<<12)
#define RX_INT_INHIBIT_HIGH     (0x01<<13)

/*rx命令定义*/
#define RX_CMD_RESERVE1         (0x01<<0)
#define RX_CMD_RESERVE2         (0x01<<1)
#define RX_CMD_SEND_EPT         (0x01<<2)
#define RX_CMD_RESET_SYS        (0x01<<3)

/*tx中断定义*/
#define TX_INT_PING             (0x01<<0)
#define TX_INT_SSP              (0x01<<1)
#define TX_INT_IDP              (0x01<<2)
#define TX_INT_CFGP             (0x01<<3)
#define TX_INT_ASK_PKT          (0x01<<4)
#define TX_INT_EPT              (0x01<<5)
#define TX_INT_RPP_TO           (0x01<<6)
#define TX_INT_CEP_TO           (0x01<<7)
#define TX_INT_AC_DET           (0x01<<8)
#define TX_INT_INIT             (0x01<<9)
#define TX_INT_RESERVE2         (0x01<<10)
#define TX_INT_RP_TYPR_ERR      (0x01<<11)
#define TX_INT_FOD              (0x01<<12)

/*tx命令定义*/
#define TX_CMD_RESERVE1         (0x01<<8)
#define TX_CMD_ENTER_TX_MODE    (0x01<<9)
#define TX_CMD_EXIT_TX_MODE     (0x01<<10)
#define TX_CMD_SEND_FSK         (0x01<<11)
#define TX_CMD_RESET_SYS        (0x01<<12)

/*tx功能定义*/
#define TX_RESERVE1_EN          (0x01<<0)
#define TX_FULL_BRI             (0x01<<1)
#define TX_RESERVE2_EN          (0x01<<2)

/*tx EPT定义*/
#define EPT_POCP                (0x01<<11)
#define EPT_OTP                 (0x01<<10)
#define EPT_FOD                 (0x01<<9)
#define EPT_UVP                 (0x01<<8)
#define EPT_OVP                 (0x01<<7)
#define EPT_OCP                 (0x01<<6)
#define EPT_RPP_TO              (0x01<<5)
#define EPT_CEP_TO              (0x01<<4)
#define EPT_RCV_EPT             (0x01<<3)
#define EPT_SSP                 (0x01<<2)
#define EPT_AC_DET              (0x01<<1)
#define EPT_WRONG_PACKET        (0x01<<0)

#define AP_SYS_INFO_BASE_ADDR      0x20001D00    
#define AP_SYS_CONTROL_BASE_ADDR   0x20001D40
#define AP_TX_CONFIG_BASE_ADDR     0x20001E00    
#define AP_TX_CONTROL_BASE_ADDR    0x20001E40
#define AP_TX_REPORT_BASE_ADDR     0x20001E80
#define AP_RX_CONFIG_BASE_ADDR     0x20001F00
#define AP_RX_CONTROL_BASE_ADDR    0x20001F40
#define AP_RX_REPORT_BASE_ADDR     0x20001F80

#define QI_HEAD  (0x1f)
#define QI_CMD_SOC (0xba)
#define QI_CMD_MAC0 (0xac)
#define QI_CMD_MAC1 (0xad)
#define QT_CMD_GID  (0xca)
#define QT_CMD_ERR  (0xee)
#define QT_CMD_CHGFULL  (0xcf)

#define QI_ERR_OCP (0x03)
#define QI_ERR_OVP (0x04)
#define QI_ERR_OTP (0x05)
#define QI_ERR_UVP (0x06)
#define QI_ERR_BATT_OTP (0x07)
#define QI_ERR_COIL_OTP (0x08)
#define QI_ERR_CHG_TIMEOUT (0x09)
#define QI_ERR_CHG_FAILED (0x0a)

#define RX_FOD_GAIN_LEN 16
#define RX_FOD_CURR_LEN 7

typedef enum {
	Sys_Op_Mode_AC_Missing = 0,
	Sys_Op_Mode_BPP = 0x1,
	Sys_Op_Mode_EPP = 0x2,
	Sys_Op_Mode_PDDE= 0x4,
	Sys_Op_Mode_TX = 0x8,
	Sys_Op_Mode_TX_FOD = 0x9,
	Sys_Op_Mode_INVALID,
}Sys_Op_Mode;

#define _CPS_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))

#define CPS_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_CPS_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

static uint32_t bpp_fod_array_w_folio[RX_FOD_GAIN_LEN] =
{80, 11, 80, 11, 80, 11, 80, 11, 80, 11, 80, 11, 80, 11, 80, 11};

static uint32_t epp_fod_array_w_folio[RX_FOD_GAIN_LEN] =
{120, 32, 120, 24, 120, 21, 120, 21, 120, 19, 120, 18, 120, 17, 120, 17};
/*****************************************************************************
 *  Log
 ****************************************************************************/
#define CPS_LOG_NONE    0
#define CPS_LOG_ERR     1
#define CPS_LOG_DEBG    2
#define CPS_LOG_FULL    3
    
#define ENABLE_CPS_LOG CPS_LOG_FULL
    
#define cps_wls_log(num, fmt, args...) \
    do { \
            if (ENABLE_CPS_LOG >= (int)num) \
                pr_err(fmt, ##args); \
    } while (0)
    
/*-------------------------------------------------------------------*/

struct cps_wls_chrg_chip {
    struct i2c_client *client;
    struct device *dev;
    struct regmap *regmap;
    char *name;
    struct power_supply *wl_psy;
    struct power_supply *batt_psy;
    struct power_supply *usb_psy;
    struct power_supply *dc_psy;
    struct power_supply_desc wl_psd;
    struct power_supply_config wl_psc; 
    struct power_supply_desc batt_psd;    
    struct power_supply_desc usb_psd;
    struct power_supply_desc dc_psd;
    struct pinctrl *cps_pinctrl;
    struct pinctrl_state *cps_gpio_active;
    struct pinctrl_state *cps_gpio_suspend;

    struct wakeup_source *cps_wls_wake_lock;
    struct mutex   irq_lock;
    struct mutex   i2c_lock;
    int state;
    int wls_charge_int;
    int cps_wls_irq;
    int reg_addr;
    int reg_data;
    int rx_ovp;
    int rx_ocp;
    int rx_opp;
    int rx_ht_thl;
    int rx_vout_target;
    int rx_ept_rsn;
    int rx_irect;
    int rx_vrect;
    int rx_vout;
    int rx_die_temp;
    int rx_ntc;
    int rx_neg_power;
    int rx_neg_protocol;
    int command_flag;

    unsigned long flags;
    int rx_ldo_on;
    int wls_online;
    bool wls_disconnect;
    int wls_det_int;
    int wls_det_irq;
    const char *wls_fw_name;
    uint32_t wls_fw_version;
	/* alarm timer */
	struct alarm wls_rx_timer;
	struct timespec64 end_time;
    unsigned int rx_polling_ns;
    uint32_t tx_mode;
    uint32_t wls_input_curr_max;
    uint32_t folio_mode;
    bool rx_connected;
    struct moto_chg_tcmd_client wls_tcmd_client;
    struct moto_wls_chg_ops  wls_chg_ops;
    Sys_Op_Mode mode_type;
    uint32_t MaxV;
    uint32_t MaxI;
    uint32_t chip_id;
    //struct work_struct wls_rx_work;
	wait_queue_head_t  wait_que;
	bool wls_rx_check_thread_timeout;
	struct wakeup_source *rx_check_wakelock;
    /*wls pen*/
#ifdef SMART_PEN_SUPPORT
    struct moto_wls_pen_ops  wls_pen_ops;
    uint32_t cps_pen_status;
    uint32_t cps_pen_soc;
    bool pen_power_on;
#endif
};

typedef enum ept_reason
{
    EPT_UNKONWN = 0,
    EPT_CC,
    EPT_IF,
    EPT_OT,
    EPT_OV,
    EPT_OC,
    EPT_BF,
    EPT_RES1,
    EPT_NP,
    EPT_RES2,
    EPT_NF,
    EPT_RS,
}ept_reason_e;

static void wls_rx_start_timer(struct cps_wls_chrg_chip *info);
#endif
