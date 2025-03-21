// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/

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
#include <linux/version.h>

//mtk platform.
#define CONFIG_MTK_CLASS

#ifdef CONFIG_MTK_CLASS
#include "charger_class.h"

#ifdef CONFIG_MTK_CHARGER_V4P19
#include "mtk_charger_intf.h"
#endif /*CONFIG_MTK_CHARGER_V4P19*/

#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
#include "dvchg_class.h"
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

#define SC858X_DRV_VERSION              "1.1.0_G"

//#define BIT(x)	(1 << (x))
//#define _GENMASK(BITS, POS) \
//	((unsigned char)(((1 << (BITS)) - 1) << (POS)))

//#define GENMASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
//		_GENMASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
//				(RIGHT_BIT_POS))

typedef enum sc8565_chrg_role {
	/* parallel primary*/
	SC8565_MASTER,
	/* parallel scondary*/
	SC8565_SLAVE,
} SC8565_CHRG_ROLE;

enum sc8565_cp_op_mode {
	CP_2_1_MODE = 0,
	CP_3_1_MODE,
	CP_4_1_MODE,
};

#define SC_REG_ADDR_LENGTH_BYTE     1//number of bytes for register addres
#define SC_REG_DATA_LENGTH_BYTE     1//number of bytes for register data
#define SC8565_DEV_ID                    0x81
#define CPS2023_DEV_ID                  0x05

//SC8565_VBAT_OVP 0x01
#define SC8565_VBAT_OVP_DIS	BIT(7)
#define SC8565_VBAT_OVP_ENABLE               0
#define SC8565_VBAT_OVP_DISABLE              BIT(7)
#define SC8565_VBAT_OVP_CFG		GENMASK(4, 0)
#define SC8565_VBAT_OVP_BASE                 4450
#define SC8565_VBAT_OVP_LSB                  25

//SC8565_IBAT_OCP 0x02
#define SC8565_IBAT_OCP_DIS	BIT(7)
#define SC8565_IBAT_OCP_ENABLE               0
#define SC8565_IBAT_OCP_DISABLE              BIT(7)
#define SC8565_IBAT_OCP_CFG		GENMASK(3, 0)
#define SC8565_IBAT_OCP_BASE                 8000
#define SC8565_IBAT_OCP_LSB                  500

//SC8565_VUSB_OVP 0x03
#define SC8565_VUSB_OVPGATE_ON_DG_SET	BIT(7)
#define SC8565_VUSB_OVP_CFG		GENMASK(3, 0)
#define SC8565_VUSB_OVP_BASE                 11000
#define SC8565_VUSB_OVP_LSB                  1000
#define SC8565_VUSB_OVP_6PV5                  0x0F

//SC8565_VWPC_OVP 0x04
#define SC8565_VWPC_OVP_CFG		GENMASK(3, 0)
#define SC8565_VWPC_OVP_BASE                 11000
#define SC8565_VWPC_OVP_LSB                  1000
#define SC8565_VWPC_OVP_6PV5                  0x0F

//SC8565_VBUS_OVP 0x05
#define SC8565_VBUS_OVP_CFG                 GENMASK(7, 2)
#define SC8565_VBUS_OVP_41MODE_BASE          14000
#define SC8565_VBUS_OVP_41MODE_LSB           200
#define SC8565_VBUS_OVP_21MODE_BASE          7000
#define SC8565_VBUS_OVP_21MODE_LSB           100
#define SC8565_VBUS_OVP_11MODE_BASE          3500
#define SC8565_VBUS_OVP_11MODE_LSB           50
#define SC8565_VOUT_OVP_CFG                 GENMASK(1, 0)
#define SC8565_OUT_OVP_BASE                 4800
#define SC8565_OUT_OVP_LSB                  200

//SC8565_IBUS_OCP 0x06
#define SC8565_IBUS_OCP_DIS	BIT(7)
#define SC8565_IBUS_OCP_ENABLE               0
#define SC8565_IBUS_OCP_DISABLE              BIT(7)
#define SC8565_IBUS_OCP_MASK           BIT(6)
#define SC8565_IBUS_OCP_NOT_MASK             0
#define SC8565_IBUS_OCP_IS_MASK                 1
#define SC8565_IBUS_OCP_FLAG            BIT(5)
#define SC8565_IBUS_OCP_CFG           GENMASK(4, 0)
#define SC8565_IBUS_OCP_BASE                 2500
#define SC8565_IBUS_OCP_LSB                  125

//SC8565_IBUS_UCP 0x07
#define SC8565_IBUS_UCP_DIS	BIT(7)
#define SC8565_IBUS_UCP_ENABLE               0
#define SC8565_IBUS_UCP_DISABLE              BIT(7)
#define SC8565_IBUS_UCP_FALL_DG_CFG         GENMASK(5, 4)
#define SC8565_IBUS_UCP_FALL_DG_8US          0
#define SC8565_IBUS_UCP_FALL_DG_5MS          1
#define SC8565_IBUS_UCP_FALL_DG_20MS         2
#define SC8565_IBUS_UCP_FALL_DG_50MS         3
#define SC8565_IBUS_UCP_RISE_MASK      BIT(3)
#define SC8565_IBUS_UCP_RISE_NOT_MASK        0
#define SC8565_IBUS_UCP_RISE_IS_MASK            1
#define SC8565_IBUS_UCP_RISE_FLAG       BIT(2)
#define SC8565_IBUS_UCP_FALL_MASK      BIT(1)
#define SC8565_IBUS_UCP_FALL_NOT_MASK        0
#define SC8565_IBUS_UCP_FALL_IS_MASK            1
#define SC8565_IBUS_UCP_FALL_FLAG       BIT(0)

//SC8565_PMID2OUT_OVP 0x08
#define SC8565_PMID2OUT_OVP_DIS	BIT(7)
#define SC8565_PMID2OUT_OVP_ENABLE          0
#define SC8565_PMID2OUT_OVP_DISABLE         BIT(7)
#define SC8565_PMID2OUT_OVP_MASK        BIT(4)
#define SC8565_PMID2OUT_OVP_NOT_MASK        0
#define SC8565_PMID2OUT_OVP_IS_MASK            1
#define SC8565_PMID2OUT_OVP_FLAG       BIT(3)
#define SC8565_PMID2OUT_OVP_CFG           GENMASK(2, 0)
#define SC8565_PMID2OUT_OVP_BASE            200
#define SC8565_PMID2OUT_OVP_LSB             50

//SC8565_PMID2OUT_UVP 0x09
#define SC8565_PMID2OUT_UVP_DIS	BIT(7)
#define SC8565_PMID2OUT_UVP_ENABLE          0
#define SC8565_PMID2OUT_UVP_DISABLE         BIT(7)
#define SC8565_PMID2OUT_UVP_MASK       BIT(4)
#define SC8565_PMID2OUT_UVP_NOT_MASK        0
#define SC8565_PMID2OUT_UVP_IS_MASK            1
#define SC8565_PMID2OUT_UVP_FLAG       BIT(3)
#define SC8565_PMID2OUT_UVP_CFG            GENMASK(2, 0)
#define SC8565_PMID2OUT_UVP_BASE            50
#define SC8565_PMID2OUT_UVP_LSB             50

//SC8565_CHRGR_CTRL_1 0x0B
#define SC8565_CP_EN	BIT(7)
#define SC8565_QB_EN	BIT(6)
#define SC8565_ACDRV_MANUAL	BIT(5)
#define SC8565_WPCGATE_EN	BIT(4)
#define SC8565_OVPGATE_EN	BIT(3)
#define SC8565_VBUS_PD_EN	BIT(2)
#define SC8565_VWPC_PD_EN	BIT(1)
#define SC8565_VUSB_PD_EN	BIT(0)

//SC8565_CHRGR_CTRL_3 0x0D
#define SC8565_VBUS_INRANGE_DET_DIS         BIT(7)
#define SC8565_VBUS_INRANGE_DET_ENABLE      0
#define SC8565_VBUS_INRANGE_DET_DISABLE     BIT(7)
#define SC8565_WD_TIMEOUT_SET		GENMASK(2, 0)
#define SC8565_WD_TIMEOUT_DISABLE           0
#define SC8565_WD_TIMEOUT_0P2S              1
#define SC8565_WD_TIMEOUT_0P5S              2
#define SC8565_WD_TIMEOUT_1S                3
#define SC8565_WD_TIMEOUT_5S                4
#define SC8565_WD_TIMEOUT_30S               5
#define SC8565_WD_TIMEOUT_100S              6
#define SC8565_WD_TIMEOUT_255S              7

#define SC8565_SS_TIMEOUT_SET		GENMASK(5, 3)
#define SC8565_SS_TIMEOUT_DISABLE           0
#define SC8565_SS_TIMEOUT_40MS              1
#define SC8565_SS_TIMEOUT_80MS              2
#define SC8565_SS_TIMEOUT_320MS             3
#define SC8565_SS_TIMEOUT_1280MS            4
#define SC8565_SS_TIMEOUT_5120MS            5
#define SC8565_SS_TIMEOUT_20480MS           6
#define SC8565_SS_TIMEOUT_81920MS           7

#define CPS2023_MXGATE_STAT            BIT(6)


//SC8565_CHRGR_CTRL_4 0x0E
#define SC8565_IBAT_SNS_RES		BIT(4)
#define SC8565_REG_RST		BIT(3)
#define SC8565_MODE_CFG                    GENMASK(2, 0)
#define SC8565_FORWARD_4_1_CHARGER_MODE     0
#define SC8565_FORWARD_2_1_CHARGER_MODE     1
#define SC8565_FORWARD_1_1_CHARGER_MODE     2
#define SC8565_FORWARD_1_1_CHARGER_MODE1    3
#define SC8565_REVERSE_1_4_CONVERTER_MODE   4
#define SC8565_REVERSE_1_2_CONVERTER_MODE   5
#define SC8565_REVERSE_1_1_CONVERTER_MODE   6
#define SC8565_REVERSE_1_1_CONVERTER_MODE1  7

//SC8565_CHRGR_CTRL_5 0x0F
#define SC8565_OVPGATE_STAT            BIT(7)
#define SC8565_OVPFATE_OFF                 0
#define SC8565_OVPFATE_ON                  1
#define SC8565_WPCGATE_STAT            BIT(6)
#define SC8565_MANU_CHK_DIS            BIT(5)
#define SC8565_MANU_CHK_DIS_OFF     1
#define SC8565_MANU_CHK_DIS_ON      0
#define SC8565_WPCFATE_OFF                 0
#define SC8565_WPCFATE_ON                  1
#define SC8565_VWPC_OVP_DIS            BIT(3)
#define SC8565_VWPC_OVP_ENABLE             0
#define SC8565_VWPC_OVP_DISABLE        BIT(3)
#define SC8565_VUSB_OVP_DIS            BIT(2)
#define SC8565_VUSB_OVP_ENABLE             0
#define SC8565_VUSB_OVP_DISABLE        BIT(2)
#define SC8565_VBUS_OVP_DIS            BIT(1)
#define SC8565_VBUS_OVP_ENABLE             0
#define SC8565_VBUS_OVP_DISABLE        BIT(1)
#define SC8565_VOUT_OVP_DIS            BIT(0)
#define SC8565_VOUT_OVP_ENABLE             0
#define SC8565_VOUT_OVP_DISABLE        BIT(0)

//SC8565_INT_STAT 0x10
#define SC8565_VOUT_OK_SW_REGN_STAT  BIT(6)
#define SC8565_VOUT_OK_REV_STAT        BIT(5)
#define SC8565_VOUT_OK_CHG_STAT        BIT(4)
#define SC8565_VOUT_INSERT_STAT        BIT(3)
#define SC8565_VBUS_PRESENT_STAT       BIT(2)
#define SC8565_VWPC_INSERT_STAT       BIT(1)
#define SC8565_VUSB_INSERT_STAT        BIT(0)

//SC8565_ADC_CTRL 0x15
#define SC8565_ADC_EN	BIT(7)
#define SC8565_ADC_DISABLE                  0
#define SC8565_ADC_ENABLE                   1
#define SC8565_ADC_RATE	BIT(6)
#define SC8565_ADC_RATE_CONTINOUS           0
#define SC8565_ADC_RATE_ONESHOT             1
#define SC8565_ADC_DONE_STAT	BIT(5)
#define SC8565_ADC_DONE_FLAG           BIT(4)
#define SC8565_ADC_DONE_FALG_SHIFT          4
#define SC8565_ADC_DONE_MASK           BIT(3)
#define SC8565_ADC_DONE_NOT_MASK            0
#define SC8565_ADC_DONE_IS_MASK                1
#define SC8565_IBUS_ADC_DIS            BIT(0)
#define SC8565_IBUS_ADC_ENABLE              0
#define SC8565_IBUS_ADC_DISABLE             1

/* Register 16h */
//SC8565_ADC_FN_DIS 0x16
#define SC8565_VBUS_ADC_DIS            BIT(7)
#define SC8565_VBUS_ADC_ENABLE              0
#define SC8565_VBUS_ADC_DISABLE             BIT(7)
#define SC8565_VUSB_ADC_DIS            BIT(6)
#define SC8565_VUSB_ADC_ENABLE              0
#define SC8565_VUSB_ADC_DISABLE             BIT(6)
#define SC8565_VWPC_ADC_DIS            BIT(5)
#define SC8565_VWPC_ADC_ENABLE              0
#define SC8565_VWPC_ADC_DISABLE             BIT(5)
#define SC8565_VOUT_ADC_DIS            BIT(4)
#define SC8565_VOUT_ADC_ENABLE              0
#define SC8565_VOUT_ADC_DISABLE             BIT(4)
#define SC8565_VBAT_ADC_DIS            BIT(3)
#define SC8565_VBAT_ADC_ENABLE              0
#define SC8565_VBAT_ADC_DISABLE             BIT(3)
#define SC8565_IBAT_ADC_DIS            BIT(2)
#define SC8565_IBAT_ADC_ENABLE              0
#define SC8565_IBAT_ADC_DISABLE             BIT(2)
#define SC8565_TDIE_ADC_DIS            BIT(0)
#define SC8565_TDIE_ADC_ENABLE              0
#define SC8565_TDIE_ADC_DISABLE             BIT(0)

/* Register 17h */
#define SC8565_REG_17                       0x17
#define SC8565_IBUS_POL_H_MASK              0x0F
#define SC8565_IBUS_ADC_LSB                 15625 / 10000

/* Register 18h */
#define SC8565_REG_18                       0x18
#define SC8565_IBUS_POL_L_MASK              0xFF

/* Register 19h */
#define SC8565_REG_19                       0x19
#define SC8565_VBUS_POL_H_MASK              0x0F
#define SC8565_VBUS_ADC_LSB                 625 / 100

/* Register 1Ah */
#define SC8565_REG_1A                       0x1A
#define SC8565_VBUS_POL_L_MASK              0xFF

/* Register 1Bh */
#define SC8565_REG_1B                       0x1B
#define SC8565_VUSB_POL_H_MASK              0x0F
#define SC8565_VUSB_ADC_LSB                 625 / 100

/* Register 1Ch */
#define SC8565_REG_1C                       0x1C
#define SC8565_VUSB_POL_L_MASK              0xFF

/* Register 1Dh */
#define SC8565_REG_1D D                     0x1D
#define SC8565_VWPC_POL_H_MASK              0x0F
#define SC8565_VWPC_ADC_LSB                 625 / 100

/* Register 1Eh */
#define SC8565_REG_1E                       0x1E
#define SC8565_VWPC_POL_L_MASK              0xFF

/* Register 1Fh */
#define SC8565_REG_1F                       0x1F
#define SC8565_VOUT_POL_H_MASK              0x0F
#define SC8565_VOUT_ADC_LSB                 125 / 100

/* Register 20h */
#define SC8565_REG_20                       0x20
#define SC8565_VOUT_POL_L_MASK              0x0F

/* Register 21h */
#define SC8565_REG_21                       0x21
#define SC8565_VBAT_POL_H_MASK              0x0F
#define SC8565_VBAT_ADC_LSB                 125 / 100

/* Register 22h */
#define SC8565_REG_22                       0x22
#define SC8565_VBAT_POL_L_MASK              0xFF

/* Register 23h */
#define SC8565_REG_23                       0x23
#define SC8565_IBAT_POL_H_MASK              0x0F
#define SC8565_IBAT_ADC_LSB                 375 / 100

/* Register 24h */
#define SC8565_REG_24                       0x24
#define SC8565_IBAT_POL_L_MASK              0xFF

/* Register 25h Reserved */
#define SC8565_REG_25                       0x25

/* Register 26h Reserved */
#define SC8565_REG_26                       0x26

/* Register 27h */
#define SC8565_REG_27                       0x27
#define SC8565_TDIE_POL_H_MASK              0x01
#define SC8565_TDIE_ADC_LSB                 5 / 10

/* Register 28h */
#define SC8565_REG_28                       0x28
#define SC8565_TDIE_POL_L_MASK              0xFF

/* Register 29h Reserved */
#define SC8565_REG_29                       0x29

/* Register 6Eh */
#define SC8565_REG_6E                       0x6E
#define SC8565_DEVICE_ID_MASK               0xFF

typedef struct i2c_reg_s
{
	uint16_t	reg_name;
	uint32_t	reg_addr;
	uint16_t	reg_bytes_len;
	uint32_t	reg_def_val;
} I2C_REG_S;

typedef enum sc8565_reg_e {
	SC8565_DEVICE_VER,
	SC8565_VBAT_OVP,
	SC8565_IBAT_OCP,
	SC8565_VUSB_OVP,
	SC8565_VWPC_OVP,
	SC8565_VOUT_VBUS_OVP,
	SC8565_IBUS_OCP,
	SC8565_IBUS_UCP,
	SC8565_PMID2OUT_OVP,
	SC8565_PMID2OUT_UVP,
	SC8565_CONVERTER_STAT,
	SC8565_CHRGR_CTRL_1,
	SC8565_CHRGR_CTRL_2,
	SC8565_CHRGR_CTRL_3,
	SC8565_CHRGR_CTRL_4,
	SC8565_CHRGR_CTRL_5,
	SC8565_INT_STAT,
	SC8565_INT_FLAG,
	SC8565_INT_MASK,
	SC8565_FLT_FLAG,
	SC8565_FLT_MASK,
	SC8565_ADC_CTRL,
	SC8565_ADC_FN_DIS,
	SC8565_IBUS_ADC_1,
	SC8565_IBUS_ADC_0,
	SC8565_VBUS_ADC_1,
	SC8565_VBUS_ADC_0,
	SC8565_VUSB_ADC_1,
	SC8565_VUSB_ADC_0,
	SC8565_VWPC_ADC_1,
	SC8565_VWPC_ADC_0,
	SC8565_VOUT_ADC_1,
	SC8565_VOUT_ADC_0,
	SC8565_VBAT_ADC_1,
	SC8565_VBAT_ADC_0,
	SC8565_IBAT_ADC_1,
	SC8565_IBAT_ADC_0,
	SC8565_RESERVED_0,
	SC8565_RESERVED_1,
	SC8565_TDIE_ADC_1,
	SC8565_TDIE_ADC_0,
	SC8565_RESERVED_2,
	SC8565_DPDM_CTRL_1,
	SC8565_DPDM_CTRL_2,
	SC8565_DPDM_STAT,
	SC8565_DPDM_FLAG_MASK,
	SC8565_CHRGR_CTRL_6 = 0x40,
	SC8565_DEVICE_ID = 0x6E,
	SC8565_CHRGR_CTRL_7 = 0x7F,
	SC8565_REG_MAX,
} SC8565_REG_E;

enum {
    SC858X_STANDALONG = 0,
    SC858X_MASTER,
    SC858X_SLAVE,
};

static const char* sc858x_psy_name[] = {
    [SC858X_STANDALONG] = "cp-standalone",
    [SC858X_MASTER] = "cp-master",
    [SC858X_SLAVE] = "cp-slave",
};

static const char* sc858x_irq_name[] = {
    [SC858X_STANDALONG] = "sc858x-standalone-irq",
    [SC858X_MASTER] = "sc858x-master-irq",
    [SC858X_SLAVE] = "sc858x-slave-irq",
};

static int sc858x_mode_data[] = {
    [SC858X_STANDALONG] = SC858X_STANDALONG,
    [SC858X_MASTER] = SC858X_MASTER,
    [SC858X_SLAVE] = SC858X_SLAVE,
};

enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VUSB,
    ADC_VWPC,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    RESERVED,
    ADC_TDIE,
    ADC_MAX_NUM,
}SC_858X_ADC_CH;

static const u32 sc858x_adc_accuracy_tbl[ADC_MAX_NUM] = {
	150000,	/* IBUS */
	35000,	/* VBUS */
	35000,	/* VUSB */
    35000,	/* VWPC */
	20000,	/* VOUT */
	20000,	/* VBAT */
	200000,	/* IBAT */
    0,	/* RESERVED */
	4,	/* TDIE */
};

static const int sc858x_adc_m[] =
    {15625, 625, 625, 625, 125, 125, 375, 10, 5};

static const int sc858x_adc_l[] =
    {10000, 100, 100, 100, 100, 100, 100, 10, 10};

enum sc858x_notify {
    SC858X_NOTIFY_OTHER = 0,
	SC858X_NOTIFY_IBUSOCP,
	SC858X_NOTIFY_VBUSOVP,
	SC858X_NOTIFY_IBATOCP,
	SC858X_NOTIFY_VBATOVP,
	SC858X_NOTIFY_VOUTOVP,
};

enum sc858x_error_stata {
    ERROR_VBUS_HIGH = 0,
	ERROR_VBUS_LOW,
	ERROR_VBUS_OVP,
	ERROR_IBUS_OCP,
	ERROR_VBAT_OVP,
	ERROR_IBAT_OCP,
};

struct flag_bit {
    int notify;
    int mask;
    char *name;
};

struct intr_flag {
    int reg;
    int len;
    struct flag_bit bit[8];
};

static struct intr_flag cp_intr_flag[] = {
    { .reg = 0x01, .len = 1, .bit = {
                {.mask = BIT(5), .name = "vbat ovp flag", .notify = SC858X_NOTIFY_VBATOVP},
                },
    },
    { .reg = 0x02, .len = 1, .bit = {
                {.mask = BIT(4), .name = "ibat ocp flag", .notify = SC858X_NOTIFY_IBATOCP},
                },
    },
    { .reg = 0x03, .len = 1, .bit = {
                {.mask = BIT(5), .name = "vusb ovp flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x04, .len = 1, .bit = {
                {.mask = BIT(5), .name = "vwpc ovp flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x06, .len = 1, .bit = {
                {.mask = BIT(5), .name = "ibus ocp flag", .notify = SC858X_NOTIFY_IBUSOCP},
                },
    },
    { .reg = 0x07, .len = 2, .bit = {
                {.mask = BIT(2), .name = "ibus ucp rise flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "ibus ucp fall flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x08, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out ovp flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x09, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out uvp flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x0a, .len = 2, .bit = {
                {.mask = BIT(7), .name = "por flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "pin diag fall flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x11, .len = 7, .bit = {
                {.mask = BIT(0), .name = "vusb insert flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vwpc insert flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vout insert flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vout ok chg flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vout ok rev flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vout ok sw regn flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
    { .reg = 0x13, .len = 7, .bit = {
                {.mask = BIT(0), .name = "vout ovp flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "ss fail flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "conv ocp flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "wd timeout flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ss timeout flag", .notify = SC858X_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "tshut flag", .notify = SC858X_NOTIFY_OTHER},
                },
    },
};


/************************************************************************/
#define SC858X_DEVICE_ID                0x81
#define SC8566_DEVICE_ID                0x2D
#define CPS2023H_DEVICE_ID            0x6
#define CPS2023_DEVICE_ID              0x5

#define SC858X_REG17                    0x17
#define SC858X_REGMAX                   0x7F

#define SC858X_REG7C                    0x7C
#define SC858X_PRIVATE_CODE             0x01

enum sc858x_reg_range {
    SC858X_VBAT_OVP,
    SC858X_IBAT_OCP,
    SC858X_VBUS_OVP,
    SC858X_IBUS_OCP,
};

struct reg_range {
    u32 min;
    u32 max;
    u32 step;
    u32 offset;
    const u32 *table;
    u16 num_table;
    bool round_up;
};

#define SC858X_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
    .min = _min, \
    .max = _max, \
    .step = _step, \
    .offset = _offset, \
    .round_up = _ru, \
}

#define SC858X_CHG_RANGE_T(_table, _ru) \
    { .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }


static const struct reg_range sc858x_reg_range[] = {
    [SC858X_VBAT_OVP]      = SC858X_CHG_RANGE(4450, 5225, 25, 4450, false),
    [SC858X_IBAT_OCP]      = SC858X_CHG_RANGE(8000, 15500, 500, 8000, false),
    [SC858X_VBUS_OVP]      = SC858X_CHG_RANGE(7000, 13300, 100, 7000, false),
    [SC858X_IBUS_OCP]      = SC858X_CHG_RANGE(2500, 6375, 125, 2500, false),
};


enum sc858x_fields {
    DEVICE_VER,
    VBAT_OVP_DIS, VBAT_OVP,
    IBAT_OCP_DIS, IBAT_OCP,
    VUSB_OVP,
    VWPC_OVP,
    VBUS_OVP, VOUT_OVP,
    IBUS_OCP_DIS, IBUS_OCP,
    IBUS_UCP_DIS, IBUS_UCP_FALL_DG_SET,
    PMID2OUT_OVP_DIS, PMID2OUT_OVP,
    PMID2OUT_UVP_DIS, PMID2OUT_UVP,
    CP_SWITCHING_STAT,VBUS_ERRORHI_STAT,VBUS_ERRORLO_STAT, PIN_DIAG_FAIL,
    CP_EN, QB_EN, ACDRV_MANUAL_EN, WPCGATE_EN, OVPGATE_EN, VBUS_PD_EN, VWPC_PD_EN, VUSB_PD_EN,
    FSW_SET, FREQ_DITHER, ACDRV_HI_EN,
    VBUS_INRANGE_DET_DIS, SS_TIMEOUT, WD_TIMEOUT,
    VBAT_OVP_DG_SET, SET_IBAT_SNS_RES, REG_RST, MODE,
    TSHUT_DIS, VWPC_OVP_DIS, VUSB_OVP_DIS, VBUS_OVP_DIS, VOUT_OVP_DIS,
    ADC_EN, ADC_RATE,
    DEVICE_ID,
    F_MAX_FIELDS,
};


//REGISTER
static const struct reg_field sc858x_reg_fields[] = {
    /*reg00*/
    [DEVICE_VER] = REG_FIELD(0x00, 0, 7),
    /*reg01*/
    [VBAT_OVP_DIS] = REG_FIELD(0x01, 7, 7),
    [VBAT_OVP] = REG_FIELD(0x01, 0, 4),
    /*reg02*/
    [IBAT_OCP_DIS] = REG_FIELD(0x02, 7, 7),
    [IBAT_OCP] = REG_FIELD(0x02, 0, 3),
    /*reg03*/
    [VUSB_OVP] = REG_FIELD(0x03, 0, 3),
    /*reg04*/
    [VWPC_OVP] = REG_FIELD(0x04, 0, 3),
    /*reg05*/
    [VBUS_OVP] = REG_FIELD(0x05, 2, 7),
    [VOUT_OVP] = REG_FIELD(0x05, 0, 1),
    /*reg06*/
    [IBUS_OCP_DIS] = REG_FIELD(0x06, 7, 7),
    [IBUS_OCP] = REG_FIELD(0x06, 0, 4),
    /*reg07*/
    [IBUS_UCP_DIS] = REG_FIELD(0x07, 7, 7),
    [IBUS_UCP_FALL_DG_SET] = REG_FIELD(0x07, 4, 5),
    /*reg08*/
    [PMID2OUT_OVP_DIS] = REG_FIELD(0x08, 7, 7),
    [PMID2OUT_OVP] = REG_FIELD(0x08, 0, 2),
    /*reg09*/
    [PMID2OUT_UVP_DIS] = REG_FIELD(0x09, 7, 7),
    [PMID2OUT_UVP] = REG_FIELD(0x09, 0, 2),
    /*reg0a*/
    [VBUS_ERRORLO_STAT] = REG_FIELD(0x0a, 4, 4),
    [VBUS_ERRORHI_STAT] = REG_FIELD(0x0a, 3, 3),
    [CP_SWITCHING_STAT] = REG_FIELD(0x0a, 1, 1),
    [PIN_DIAG_FAIL] = REG_FIELD(0x0a, 0, 0),
    /*reg0b*/
    [CP_EN] = REG_FIELD(0x0b, 7, 7),
    [QB_EN] = REG_FIELD(0x0b, 6, 6),
    [ACDRV_MANUAL_EN] = REG_FIELD(0x0b, 5, 5),
    [WPCGATE_EN] = REG_FIELD(0x0b, 4, 4),
    [OVPGATE_EN] = REG_FIELD(0x0b, 3, 3),
    [VBUS_PD_EN] = REG_FIELD(0x0b, 2, 2),
    [VWPC_PD_EN] = REG_FIELD(0x0b, 1, 1),
    [VUSB_PD_EN] = REG_FIELD(0x0b, 0, 0),
    /*reg0c*/
    [FSW_SET] = REG_FIELD(0x0c, 3, 7),
    [FREQ_DITHER] = REG_FIELD(0x0c, 1, 1),
    [ACDRV_HI_EN] = REG_FIELD(0x0c, 0, 0),
    /*reg0d*/
    [VBUS_INRANGE_DET_DIS] = REG_FIELD(0x0d, 7, 7),
    [SS_TIMEOUT] = REG_FIELD(0x0d, 3, 5),
    [WD_TIMEOUT] = REG_FIELD(0x0d, 0, 2),
    /*reg0e*/
    [VBAT_OVP_DG_SET] = REG_FIELD(0x0e, 5, 5),
    [SET_IBAT_SNS_RES] = REG_FIELD(0x0e, 4, 4),
    [REG_RST] = REG_FIELD(0x0e, 3, 3),
    [MODE] = REG_FIELD(0x0e, 0, 2),
    /*reg0f*/
    [TSHUT_DIS] = REG_FIELD(0x0f, 4, 4),
    [VWPC_OVP_DIS] = REG_FIELD(0x0f, 3, 3),
    [VUSB_OVP_DIS] = REG_FIELD(0x0f, 2, 2),
    [VBUS_OVP_DIS] = REG_FIELD(0x0f, 1, 1),
    [VOUT_OVP_DIS] = REG_FIELD(0x0f, 0, 0),
    /*reg15*/
    [ADC_EN] = REG_FIELD(0x15, 7, 7),
    [ADC_RATE] = REG_FIELD(0x15, 6, 6),
    /*reg6e*/
    [DEVICE_ID] = REG_FIELD(0x6e, 0, 7),
};

static const struct regmap_config sc858x_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = 0xFF,
};

/************************************************************************/

struct sc858x_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp;
    int ibat_ocp_dis;
    int ibat_ocp;
    int vusb_ovp_dis;
    int vusb_ovp;
    int vwpc_ovp_dis;
    int vwpc_ovp;
    int vbus_ovp_dis;
    int vbus_ovp;
    int vout_ovp_dis;
    int vout_ovp;
    int ibus_ocp_dis;
    int ibus_ocp;
    int ibus_ucp_fall_dis;
    int ibus_ucp_fall;
    int pmid2out_uvp_dis;
    int pmid2out_uvp;
    int pmid2out_ovp_dis;
    int pmid2out_ovp;
    int fsw_set;
    int ss_timeout;
    int wd_timeout;
    int ibat_sns_r;
    int mode;
    int tshut_dis;
};

struct sc858x_chip {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];

    struct sc858x_cfg_e cfg;
    int irq_gpio;
    int irq;

    int mode;

    bool charge_enabled;
    int usb_present;
    int vbus_volt;
    int ibus_curr;
    int vbat_volt;
    int ibat_curr;
    int die_temp;

#ifdef CONFIG_MTK_CLASS
    struct charger_device *chg_dev;
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    struct dvchg_dev *charger_pump;
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

    const char *chg_dev_name;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *psy;

	bool mmi_disable_mux;
	char model_name[I2C_NAME_SIZE];
	int reg_addr;
	int reg_data;
	struct charger_properties chg_prop;
       int device_id;
	bool otg_delay_mos_config;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	struct mutex irq_complete;
};

#if 0//def CONFIG_MTK_CLASS
static const struct charger_properties sc858x_chg_props = {
	.alias_name = "sc858x_chg",
};
#endif /*CONFIG_MTK_CLASS*/


/********************COMMON API***********************/
__maybe_unused static u8 val2reg(enum sc858x_reg_range id, u32 val)
{
    int i;
    u8 reg;
    const struct reg_range *range= &sc858x_reg_range[id];

    if (!range)
        return val;

    if (range->table) {
        if (val <= range->table[0])
            return 0;
        for (i = 1; i < range->num_table - 1; i++) {
            if (val == range->table[i])
                return i;
            if (val > range->table[i] &&
                val < range->table[i + 1])
                return range->round_up ? i + 1 : i;
        }
        return range->num_table - 1;
    }
    if (val <= range->min)
        reg = 0;
    else if (val >= range->max)
        reg = (range->max - range->offset) / range->step;
    else if (range->round_up)
        reg = (val - range->offset) / range->step + 1;
    else
        reg = (val - range->offset) / range->step;
    return reg;
}

__maybe_unused static u32 reg2val(enum sc858x_reg_range id, u8 reg)
{
    const struct reg_range *range= &sc858x_reg_range[id];
    if (!range)
        return reg;
    return range->table ? range->table[reg] :
                  range->offset + range->step * reg;
}
/*********************************************************/
static int sc858x_field_read(struct sc858x_chip *sc,
                enum sc858x_fields field_id, int *val)
{
    int ret;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc858x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc858x_field_write(struct sc858x_chip *sc,
                enum sc858x_fields field_id, int val)
{
    int ret;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc858x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc858x_read_block(struct sc858x_chip *sc,
                int reg, uint8_t *val, int len)
{
    int ret;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev, "sc858x read %02x block failed %d\n", reg, ret);
    }

    return ret;
}



/*******************************************************/
__maybe_unused static int sc858x_detect_device(struct sc858x_chip *sc)
{
    int ret;
    int val;

    sc->device_id = 0;
    ret = sc858x_field_read(sc, DEVICE_ID, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    if (val != SC858X_DEVICE_ID && val !=SC8566_DEVICE_ID && val != CPS2023_DEVICE_ID && val != CPS2023H_DEVICE_ID) {
        dev_err(sc->dev, "%s not find SC858X, ID = 0x%02x\n", __func__, val);
        return -EINVAL;
    }
    sc->device_id = val;
    dev_err(sc->dev, "%s find SC858X, ID = 0x%02x\n", __func__, val);

    return ret;
}

__maybe_unused static int sc858x_reg_reset(struct sc858x_chip *sc)
{
	int ret = 0;
	int val = 0;
	int val_vout = 0;

	//only cps2023 need this. cps's reset operate as hw reset which damage chip, need cp_en disable firstly,then reset.
	if (sc->device_id == CPS2023H_DEVICE_ID || sc->device_id == CPS2023_DEVICE_ID){

		ret = sc858x_field_read(sc, CP_EN, &val);
		if (ret < 0) {
			dev_err(sc->dev, "%s read cp_en fail(%d)\n", __func__, ret);
		}

		ret = regmap_read(sc->regmap, SC8565_INT_STAT, &val_vout);
		if(ret){
			dev_err(sc->dev, "%s read INT STAT fail(%d)\n", __func__, ret);
		}

		dev_err(sc->dev, "%s read cp_en :%d , and INT_STAT :%d before reset\n", __func__, val, val_vout);

		if(val == 1 || val_vout&SC8565_VOUT_OK_SW_REGN_STAT) {
			ret = sc858x_field_write(sc, CP_EN, 0);
			if (ret < 0) {
				dev_err(sc->dev, "%s write cp_en fail(%d)\n", __func__, ret);
			}else
				mdelay(500);
			dev_err(sc->dev, "%s cp enabled before reset, disable it\n", __func__);
		}
	}

    return sc858x_field_write(sc, REG_RST, 1);
}

__maybe_unused static int sc858x_dump_reg(struct sc858x_chip *sc)
{
    int ret;
    int i;
    int val;

    for (i = 0; i <= 0x28; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        dev_err(sc->dev, "%s %s reg[0x%02x] = 0x%02x\n",
                __func__,sc->model_name, i, val);
    }

    ret = regmap_read(sc->regmap, 0x6E, &val);
    dev_err(sc->dev, "%s %s reg[0x6E] = 0x%02x\n",
                __func__,sc->model_name, val);

    return ret;
}


__maybe_unused static int sc858x_check_charge_enabled(struct sc858x_chip *sc, bool *enabled)
{
    int ret, val;

    ret = sc858x_field_read(sc, CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    dev_info(sc->dev,"%s:%d", __func__, val);
//    sc858x_dump_reg(sc);
    return ret;
}

__maybe_unused static int sc858x_get_status(struct sc858x_chip *sc, uint32_t *status)
{
    int ret, val;
    *status = 0;

    ret = sc858x_field_read(sc, VBUS_ERRORHI_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    ret = sc858x_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORLO_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_LOW);


    return ret;

}

__maybe_unused static int sc858x_enable_adc(struct sc858x_chip *sc, bool en)
{
    dev_info(sc->dev,"%s:%d", __func__, en);
    return sc858x_field_write(sc, ADC_EN, !!en);
}

__maybe_unused static int sc858x_set_adc_scanrate(struct sc858x_chip *sc, bool oneshot)
{
    dev_info(sc->dev,"%s:%d",__func__,oneshot);
    return sc858x_field_write(sc, ADC_RATE, !!oneshot);
}

static int sc858x_get_adc_data(struct sc858x_chip *sc,
            int channel, int *result)
{
    uint8_t val[2] = {0};
    int ret;

    if(channel >= ADC_MAX_NUM)
        return -EINVAL;

    // sc858x_enable_adc(sc, true);
    // msleep(50);

    ret = sc858x_read_block(sc, SC858X_REG17 + (channel << 1), val, 2);
    if (ret < 0) {
        return ret;
    }

    *result = (val[1] | (val[0] << 8)) *
                sc858x_adc_m[channel] / sc858x_adc_l[channel];

    dev_info(sc->dev,"%s %d %d", __func__, channel, *result);

    //sc858x_enable_adc(sc, false);

    return ret;
}

static int sc858x_set_private_code(struct sc858x_chip *sc)
{
    return regmap_write(sc->regmap, SC858X_REG7C, SC858X_PRIVATE_CODE);
}

__maybe_unused static int sc858x_set_busovp_th(struct sc858x_chip *sc, int threshold)
{
    int reg_val;
    int temp_threshold = 0;
    int ret;
    int val;

    ret = sc858x_field_read(sc, MODE, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read MODE(%d)\n", __func__, ret);
        return ret;
    }

    if (val == 0) {
        temp_threshold = threshold / 2;
    } else if (val == 2)
    {
        temp_threshold = threshold * 2;
    } else if (val == 1)
    {
        temp_threshold = threshold;
    } else {
        return -1;
    }

    reg_val = val2reg(SC858X_VBUS_OVP, temp_threshold);
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc858x_field_write(sc, VBUS_OVP, reg_val);
}

__maybe_unused static int sc858x_set_busocp_th(struct sc858x_chip *sc, int threshold)
{
    int reg_val = val2reg(SC858X_IBUS_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc858x_field_write(sc, IBUS_OCP, reg_val);
}

__maybe_unused static int sc858x_set_batovp_th(struct sc858x_chip *sc, int threshold)
{
    int reg_val = val2reg(SC858X_VBAT_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc858x_field_write(sc, VBAT_OVP, reg_val);
}

__maybe_unused static int sc858x_set_batocp_th(struct sc858x_chip *sc, int threshold)
{
    int reg_val = val2reg(SC858X_IBAT_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc858x_field_write(sc, IBAT_OCP, reg_val);
}

__maybe_unused static int sc858x_set_vbusovp_alarm(struct sc858x_chip *sc, int threshold)
{
    dev_info(sc->dev,"%s:%d", __func__, threshold);

    return 0;
}

__maybe_unused static int sc858x_set_vbatovp_alarm(struct sc858x_chip *sc, int threshold)
{
    dev_info(sc->dev,"%s:%d", __func__, threshold);

    return 0;
}


__maybe_unused static int sc858x_is_vbuslowerr(struct sc858x_chip *sc, bool *err)
{
    int ret;
    int val;

    ret = sc858x_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc858x_is_vbushigherr(struct sc858x_chip *sc, bool *err)
{
    int ret;
    int val;

    ret = sc858x_field_read(sc, VBUS_ERRORHI_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc858x_enable_charge(struct sc858x_chip *sc, bool en)
{
    int ret = 0;
    int vbus_value = 0, vout_value = 0, value = 0;
    int vbus_hi = 0, vbus_low = 0;
    dev_info(sc->dev,"%s:%d",__func__,en);

    if (!en) {
        ret |= sc858x_field_write(sc, CP_EN, !!en);

        return ret;
    } else {
        ret = sc858x_get_adc_data(sc, ADC_VBUS, &vbus_value);
        ret |= sc858x_get_adc_data(sc, ADC_VOUT, &vout_value);
        dev_info(sc->dev,"%s: vbus/vout:%d / %d = %d \r\n", __func__, vbus_value, vout_value, vbus_value*100/vout_value);

        ret |= sc858x_field_read(sc, MODE, &value);
        dev_info(sc->dev,"%s: mode:%d %s \r\n", __func__, value, (value == 0 ?"4:1":(value == 1 ?"2:1":"else")));

        ret |= sc858x_field_read(sc, VBUS_ERRORLO_STAT, &vbus_low);
        ret |= sc858x_field_read(sc, VBUS_ERRORHI_STAT, &vbus_hi);
        dev_info(sc->dev,"%s: high:%d  low:%d \r\n", __func__, vbus_hi, vbus_low);

        ret |= sc858x_field_write(sc, CP_EN, !!en);

        disable_irq(sc->irq);

        mdelay(300);

        ret |= sc858x_field_read(sc, PIN_DIAG_FAIL, &value);
        dev_info(sc->dev,"%s: pin diag fail:%d \r\n", __func__, value);

        ret |= sc858x_field_read(sc, CP_SWITCHING_STAT, &value);
        if (!value) {
            dev_info(sc->dev,"%s:enable fail \r\n", __func__);
            sc858x_dump_reg(sc);
	     ret = -EINVAL;
        } else {
            dev_info(sc->dev,"%s:enable success \r\n", __func__);
        }

        enable_irq(sc->irq);
    }

    return ret;
}

int sc858x_init_cps2023_device(struct sc858x_chip *sc) {
	int ret = 0;

	//only cps2023 need this.
	if (sc->device_id != CPS2023H_DEVICE_ID && sc->device_id != CPS2023_DEVICE_ID){
		return 0;
	}

	ret = regmap_update_bits(sc->regmap, SC8565_CHRGR_CTRL_5,
					SC8565_MANU_CHK_DIS, SC8565_MANU_CHK_DIS);
	if (ret) {
	    dev_err(sc->dev, "%s:mmi_mux dis manu_chk_dis fail ret=%d", __func__, ret);
	    return ret;
	}

	ret = regmap_update_bits(sc->regmap, SC8565_CHRGR_CTRL_3,
						CPS2023_MXGATE_STAT, 0);
	if (ret) {
		dev_err(sc->dev, "%s:mmi_mux dis mxgate fail ret=%d", __func__, ret);
		return ret;
	}

	return ret;
}

int sc858x_init_cps2023_CN2VOUT(struct sc858x_chip *sc) {
	int ret = 0;
       int val = 0;

	//only cps2023 need this.
	if (sc->device_id != CPS2023H_DEVICE_ID && sc->device_id != CPS2023_DEVICE_ID){
		return 0;
	}

	//reg_0xc6[7:0] = 0x1
	ret = regmap_write(sc->regmap, 0xC6, 0x1);
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xc6 fail ret=%d", __func__, ret);
	}
	//reg_0xff[7:0] = 0x23
	regmap_write(sc->regmap, 0xFF, 0x23);
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xff fail ret=%d", __func__, ret);
	}

	//reg_0xDC[4]=1
	ret = regmap_update_bits(sc->regmap, 0xDC,
					BIT(4), BIT(4));
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xdc fail ret=%d", __func__, ret);
	}

	ret = regmap_read(sc->regmap, 0xDC, &val);
	if (ret) {
	    dev_err(sc->dev, "%s: read 0xDC fail ret=%d,val:0x%X", __func__, ret, val);
	}
	dev_err(sc->dev, "%s: read 0xDC ret=%d,val:0x%X", __func__, ret, val);

	//reg_0xe7[7]=1
	ret = regmap_update_bits(sc->regmap, 0xE7,
					BIT(7), BIT(7));
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xe7 fail ret=%d", __func__, ret);
	}

	ret = regmap_read(sc->regmap, 0xE7, &val);
	if (ret) {
	    dev_err(sc->dev, "%s: read 0xE7 fail ret=%d,val:0x%X", __func__, ret, val);
	}
	dev_err(sc->dev, "%s: after read 0xE7 ret=%d,val:0x%X", __func__, ret, val);

	//reg_0xFF[7:0] = 0x0
	regmap_write(sc->regmap, 0xFF, 0x0);
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xFF fail ret=%d", __func__, ret);
	}

	//reg_0xc6[7:0] = 0x0
	regmap_write(sc->regmap, 0xC6, 0x0);
	if (ret) {
	    dev_err(sc->dev, "%s: write 0xc6 fail ret=%d", __func__, ret);
	}

	dev_err(sc->dev, "%s: write CN2VOUT success\n", __func__);

	return ret;
}

__maybe_unused static int sc858x_init_device(struct sc858x_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc858x_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.vbat_ovp_dis},
        {VBAT_OVP, sc->cfg.vbat_ovp},
        {IBAT_OCP_DIS, sc->cfg.ibat_ocp_dis},
        {IBAT_OCP, sc->cfg.ibat_ocp},
        {VUSB_OVP_DIS, sc->cfg.vusb_ovp_dis},
        {VUSB_OVP, sc->cfg.vusb_ovp},
        {VWPC_OVP_DIS, sc->cfg.vwpc_ovp_dis},
        {VWPC_OVP, sc->cfg.vwpc_ovp},
        {VBUS_OVP_DIS, sc->cfg.vbus_ovp_dis},
        {VBUS_OVP, sc->cfg.vbus_ovp},
        {VOUT_OVP_DIS, sc->cfg.vout_ovp_dis},
        {VOUT_OVP, sc->cfg.vout_ovp},
        {IBUS_OCP_DIS, sc->cfg.ibus_ocp_dis},
        {IBUS_OCP, sc->cfg.ibus_ocp},
        {IBUS_UCP_DIS, sc->cfg.ibus_ucp_fall_dis},
        {IBUS_UCP_FALL_DG_SET, sc->cfg.ibus_ucp_fall},
        {PMID2OUT_UVP_DIS, sc->cfg.pmid2out_uvp_dis},
        {PMID2OUT_UVP, sc->cfg.pmid2out_uvp},
        {PMID2OUT_OVP_DIS, sc->cfg.pmid2out_ovp_dis},
        {PMID2OUT_OVP, sc->cfg.pmid2out_ovp},
        {FSW_SET, sc->cfg.fsw_set},
        {WD_TIMEOUT, sc->cfg.wd_timeout},
        {SET_IBAT_SNS_RES, sc->cfg.ibat_sns_r},
        {MODE, sc->cfg.mode},
        {TSHUT_DIS, sc->cfg.tshut_dis},
    };

    ret = sc858x_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc858x_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    if (sc->mode == SC858X_SLAVE) {
        ret = sc858x_field_write(sc, VBUS_INRANGE_DET_DIS, 1);
        if (ret < 0) {
            dev_err(sc->dev, "%s Failed to set vbus in range(%d)\n", __func__, ret);
        }
    }

    sc858x_enable_adc(sc, true);
    sc858x_set_private_code(sc);
    sc858x_init_cps2023_device(sc);
    sc858x_dump_reg(sc);

    return ret;
}


/*********************mtk charger interface start**********************************/
#ifdef CONFIG_MTK_CLASS
static inline int to_sc858x_adc(enum adc_channel chan)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		return ADC_VBUS;
	case ADC_CHANNEL_VBAT:
		return ADC_VBAT;
	case ADC_CHANNEL_IBUS:
		return ADC_IBUS;
	case ADC_CHANNEL_IBAT:
		return ADC_IBAT;
	case ADC_CHANNEL_TEMP_JC:
		return ADC_TDIE;
	case ADC_CHANNEL_VOUT:
		return ADC_VOUT;
	default:
		break;
	}
	return ADC_MAX_NUM;
}


static int mtk_sc858x_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_check_charge_enabled(sc, en);

    return ret;
}

#if 0
static int mtk_sc858x_enable_chg(struct charger_device *chg_dev, bool en)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_enable_charge(sc,en);

    return ret;
}
#endif

static int mtk_sc858x_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int mv;
    mv = uV / 1000;

    return sc858x_set_busovp_th(sc, mv);
#else
	return 0;
#endif
}

static int mtk_sc858x_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ma;
    ma = uA / 1000;

    return sc858x_set_busocp_th(sc, ma);
#else
	return 0;
#endif
}

static int mtk_sc858x_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_set_batovp_th(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
#else
	return 0;
#endif
}

static int mtk_sc858x_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_set_batocp_th(sc, uA/1000);
    if (ret < 0)
        return ret;

    return ret;
#else
	return 0;
#endif
}


static int mtk_sc858x_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);

    sc858x_get_adc_data(sc, to_sc858x_adc(chan), max);

    if(chan != ADC_CHANNEL_TEMP_JC)
        *max = *max * 1000;

    if (min != max)
		*min = *max;

    return 0;
}

static int mtk_sc858x_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
    *min = *max = sc858x_adc_accuracy_tbl[to_sc858x_adc(chan)];
    return 0;
}

static int mtk_sc858x_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);

    return sc858x_is_vbuslowerr(sc,err);
}

static int mtk_sc858x_is_vbushigher(struct charger_device *chg_dev, bool *err)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);

    return sc858x_is_vbushigherr(sc,err);
}

static int mtk_sc858x_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_set_vbatovp_alarm(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
#else
	return 0;
#endif
}

static int mtk_sc858x_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    dev_err(sc->dev,"%s",__func__);
    return 0;
#else
	return 0;
#endif
}

static int mtk_sc858x_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc858x_set_vbusovp_alarm(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
#else
	return 0;
#endif
}

static int mtk_sc858x_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
#if 0
    struct sc858x_chip *sc = charger_get_data(chg_dev);
    dev_err(sc->dev,"%s",__func__);
    return 0;
#else
	return 0;
#endif
}

static int mtk_sc858x_init_chip(struct charger_device *chg_dev)
{
    struct sc858x_chip *sc = charger_get_data(chg_dev);

    return sc858x_init_device(sc);
}

#ifdef CONFIG_MOTO_CHANNEL_SWITCH
static int sc8565_get_adc_vusb(struct sc858x_chip *bq)
{
	int vac1_stat;
	int ret = 0;
	ret = regmap_read(bq->regmap, SC8565_INT_STAT, &vac1_stat);
        dev_err(bq->dev,"the vac1 stat = :%x",vac1_stat);
	if (ret) {
		return 0;
	}

        if((vac1_stat & SC8565_VUSB_INSERT_STAT)){
            ret = 1;//VAC_ONLINE
        }else{
            ret = 0;//VAC_NOT_ONLINE
        }
	dev_err(bq->dev,"the vac1 ret = :%d",ret);

	return ret;
}

static int sc8565_get_adc_vwpc(struct sc858x_chip *bq)
{
	int vac1_stat;
	int ret = 0;
	ret = regmap_read(bq->regmap, SC8565_INT_STAT, &vac1_stat);
        dev_err(bq->dev,"the vac1 stat = :%x",vac1_stat);
	if (ret) {
		return 0;
	}

        if((vac1_stat & SC8565_VWPC_INSERT_STAT)){
            ret = 1;//VAC_ONLINE
        }else{
            ret = 0;//VAC_NOT_ONLINE
        }
	dev_err(bq->dev,"the vac2 ret = :%d",ret);

	return ret;
}

static int sc8565_get_vmos_chg(struct charger_device *chg_dev, bool type, int *mos_vbus)
{
	struct sc858x_chip *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	if (type) {
		*mos_vbus = sc8565_get_adc_vusb(bq);
		pr_err("vac1 val = %d\n", *mos_vbus);
	} else {
		*mos_vbus = sc8565_get_adc_vwpc(bq);
		pr_err("vac2 val = %d\n", *mos_vbus);
	}
	return 0;
}
#endif


#endif /*CONFIG_MTK_CLASS*/
/********************mtk charger interface end*************************************************/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
static inline int sc_to_sc858x_adc(enum sc_adc_channel chan)
{
	switch (chan) {
	case SC_ADC_VBUS:
		return ADC_VBUS;
	case SC_ADC_VBAT:
		return ADC_VBAT;
	case SC_ADC_IBUS:
		return ADC_IBUS;
	case SC_ADC_IBAT:
		return ADC_IBAT;
	case SC_ADC_TDIE:
		return ADC_TDIE;
	default:
		break;
	}
	return ADC_MAX_NUM;
}


static int sc_sc858x_set_enable(struct dvchg_dev *charger_pump, bool enable)
{
    struct sc858x_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc858x_enable_charge(sc,enable);

    return ret;
}

static int sc_sc858x_get_is_enable(struct dvchg_dev *charger_pump, bool *enable)
{
    struct sc858x_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc858x_check_charge_enabled(sc, enable);

    return ret;
}

static int sc_sc858x_get_status(struct dvchg_dev *charger_pump, uint32_t *status)
{
    struct sc858x_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;

    ret = sc858x_get_status(sc, status);

    return ret;
}

static int sc_sc858x_get_adc_value(struct dvchg_dev *charger_pump, enum sc_adc_channel ch, int *value)
{
    struct sc858x_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;

    ret = sc858x_get_adc_data(sc, sc_to_sc858x_adc(ch), value);

    return ret;
}

static struct dvchg_ops sc_sc858x_dvchg_ops = {
    .set_enable = sc_sc858x_set_enable,
    .get_status = sc_sc858x_get_status,
    .get_is_enable = sc_sc858x_get_is_enable,
    .get_adc_value = sc_sc858x_get_adc_value,
};
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

/********************creat devices note start*************************************************/
static ssize_t sc858x_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc858x_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc858x");
    for (addr = 0x0; addr <= SC858X_REGMAX; addr++) {
        ret = regmap_read(sc->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc858x_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc858x_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= SC858X_REGMAX)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc858x_show_registers, sc858x_store_register);

static void sc858x_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
}
/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
#ifdef CONFIG_MTK_CLASS
static inline int status_reg_to_charger(enum sc858x_notify notify)
{
	switch (notify) {
    case SC858X_NOTIFY_IBUSOCP:
		return CHARGER_DEV_NOTIFY_IBUSOCP;
    case SC858X_NOTIFY_VBUSOVP:
		return CHARGER_DEV_NOTIFY_VBUS_OVP;
    case SC858X_NOTIFY_IBATOCP:
		return CHARGER_DEV_NOTIFY_IBATOCP;
    case SC858X_NOTIFY_VBATOVP:
		return CHARGER_DEV_NOTIFY_BAT_OVP;
    case SC858X_NOTIFY_VOUTOVP:
		return CHARGER_DEV_NOTIFY_VOUTOVP;
	default:
        return -EINVAL;
		break;
	}
	return -EINVAL;
}
#endif /*CONFIG_MTK_CLASS*/
__maybe_unused
static void sc858x_dump_check_fault_status(struct sc858x_chip *sc)
{
    u8 flag = 0;
    int i,j,k;
#ifdef CONFIG_MTK_CLASS
    int noti;
#endif /*CONFIG_MTK_CLASS*/

    for (i = 0; i <= 0X28; i++) {
        sc858x_read_block(sc, i, &flag, 1);
        dev_err(sc->dev, "%s reg[0x%02x] = 0x%02x\n", __func__, i, flag);
        for (k=0; k < ARRAY_SIZE(cp_intr_flag); k++) {
            if (cp_intr_flag[k].reg == i){
                for (j=0; j <  cp_intr_flag[k].len; j++) {
                    if (flag & cp_intr_flag[k].bit[j].mask) {
                        dev_err(sc->dev,"trigger :%s\n",cp_intr_flag[k].bit[j].name);
        #ifdef CONFIG_MTK_CLASS
                        noti = status_reg_to_charger(cp_intr_flag[k].bit[j].notify);
                        if(noti >= 0) {
                            charger_dev_notify(sc->chg_dev, noti);
                        }
        #endif /*CONFIG_MTK_CLASS*/
                    }
                }
            }
        }
    }
}

static irqreturn_t sc858x_irq_handler(int irq, void *data)
{
    struct sc858x_chip *sc = data;
    unsigned int val = 0;
    int ret;

       dev_err(sc->dev,"INT OCCURED\n");
	mutex_lock(&sc->irq_complete);
	sc->irq_waiting = true;
	if (!sc->resume_completed) {
		dev_dbg(sc->dev, "IRQ triggered before device-resume\n");
		if (!sc->irq_disabled) {
			disable_irq_nosync(irq);
			sc->irq_disabled = true;
		}
		mutex_unlock(&sc->irq_complete);
		return IRQ_HANDLED;
	}
	sc->irq_waiting = false;

    dev_err(sc->dev,"[%s]%s enter\n",sc->model_name, __func__);
	if (sc->device_id== CPS2023H_DEVICE_ID || sc->device_id== CPS2023_DEVICE_ID){
		if (sc->otg_delay_mos_config == true) {
			sc->otg_delay_mos_config = false;
			regmap_read(sc->regmap, SC8565_INT_STAT, &val);
			if(val & SC8565_VBUS_PRESENT_STAT) {
				dev_err(sc->dev,"INT enable otg typec mos\n");
				ret = regmap_update_bits(sc->regmap, SC8565_CHRGR_CTRL_1,
									SC8565_OVPGATE_EN, SC8565_OVPGATE_EN);
				if (ret) {
					dev_err(sc->dev, "mmi_mux enable otg typec mos fail ret=%d", ret);
				       mutex_unlock(&sc->irq_complete);
					return ret;
				}
			}
		}
	}

    sc858x_dump_check_fault_status(sc);
    mutex_unlock(&sc->irq_complete);

    power_supply_changed(sc->psy);
    return IRQ_HANDLED;
}

static int sc858x_register_interrupt(struct sc858x_chip *sc)
{
    int ret;

    if (gpio_is_valid(sc->irq_gpio)) {
        ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN,"sc858x_irq");
        if (ret) {
            dev_err(sc->dev,"failed to request sc858x_irq\n");
            return -EINVAL;
        }
        sc->irq = gpio_to_irq(sc->irq_gpio);
        if (sc->irq < 0) {
            dev_err(sc->dev,"failed to gpio_to_irq\n");
            return -EINVAL;
        }
    } else {
        dev_err(sc->dev,"irq gpio not provided\n");
        return -EINVAL;
    }

    if (sc->irq) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
                NULL, sc858x_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                sc858x_irq_name[sc->mode], sc);

        if (ret < 0) {
            dev_err(sc->dev,"request irq for irq=%d failed, ret =%d\n",
                            sc->irq, ret);
            return ret;
        }
        enable_irq_wake(sc->irq);
    }

    return ret;
}
/********************interrupte end*************************************************/


/************************psy start**************************************/
static enum power_supply_property sc858x_charger_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,

};

#define SC8565_MANUFACTURER "SouthChip"
static int sc858x_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc858x_chip *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc858x_get_adc_data(sc, ADC_VBUS, &result);
        if (!ret)
            sc->vbus_volt = result;
        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc858x_get_adc_data(sc, ADC_IBUS, &result);
        if (!ret)
            sc->ibus_curr = result;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = sc858x_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = sc858x_get_adc_data(sc, ADC_IBAT, &result);
        if (!ret)
            sc->ibat_curr = result;
        val->intval = sc->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc858x_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = sc->model_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SC8565_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

    default:
        return -EINVAL;
    }

    return 0;
}

static int sc858x_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    switch (prop) {
    default:
        return -EINVAL;
    }

    return 0;
}

static int sc858x_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    return 0;
}

static int sc858x_psy_register(struct sc858x_chip *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc858x_psy_name[sc->mode];

    sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    sc->psy_desc.properties = sc858x_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc858x_charger_props);
    sc->psy_desc.get_property = sc858x_charger_get_property;
    sc->psy_desc.set_property = sc858x_charger_set_property;
    sc->psy_desc.property_is_writeable = sc858x_charger_is_writeable;

    sc->psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->psy)) {
        dev_err(sc->dev, "%s failed to register psy\n", __func__);
        return PTR_ERR(sc->psy);
    }

    dev_info(sc->dev, "%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}


/************************psy end**************************************/

static int sc858x_set_work_mode(struct sc858x_chip *sc, int mode)
{
    sc->mode = mode;

    dev_err(sc->dev,"work mode is %s\n", sc->mode == SC858X_STANDALONG
        ? "standalone" : (sc->mode == SC858X_MASTER ? "master" : "slave"));

    return 0;
}
static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", bq->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	bq->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);


static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	regmap_read(bq->regmap, bq->reg_addr, &bq->reg_data);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", bq->reg_addr, bq->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
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
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = sc858x_check_charge_enabled(bq, &enable);
	if (ret < 0) {
		pr_err("sc8565: sc8565_is_chg_en not valid\n");
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
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc858x_enable_charge(bq, enable);
	if (ret) {
		pr_err("sc8565 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("sc8565  %s charging \n",
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
	struct sc858x_chip *bq = dev_get_drvdata(dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	for (addr = 0; addr <= 0x28; addr++) {
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
	struct sc858x_chip *bq = dev_get_drvdata(dev);

	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	sc858x_get_adc_data(bq, ADC_VBUS, &vbus);

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static void sc8565_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_reg_dump);
    device_create_file(dev, &dev_attr_vbus);
}

static int sc8565_enable_chg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sc858x_chip *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = sc858x_enable_charge(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return -EINVAL;
	}

	return 0;
}

static int sc8565_enable_adc(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sc858x_chip *bq = charger_get_data(chg_dev);
	if (!bq) {
		pr_err("sc8565 chip not valid\n");
		return -ENODEV;
	}

	dev_info(bq->dev, "%s %d\n", __func__, en);
	ret = sc858x_enable_adc(bq, en);
	if (ret) {
		dev_err(bq->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}


static int sc858x_config_mux(struct sc858x_chip *bq,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	unsigned int val;

	if (bq->mmi_disable_mux)
		return 0;

	 if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
	     bq-> otg_delay_mos_config = false;
	     //disable reverse mode at sc8565, cps no need.
	     if(bq->device_id != CPS2023H_DEVICE_ID  && bq->device_id != CPS2023_DEVICE_ID) {
		    sc858x_field_read(bq, MODE, &ret);
		    if(ret == 6 ) {
	            	sc858x_field_write(bq, MODE, 1);
			dev_err(bq->dev, "%s:mmi_mux dis cp otg reverse mode", __func__);
		    }
	    }
#if 0
            ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
                    BQ25980_EN_OTG, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close en otg fail ret=%d", __func__, ret);
                return ret;
            }
#endif
        }
	 if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN
		&& typec_mos != MMI_DVCHG_MUX_DISABLE && wls_mos != MMI_DVCHG_MUX_DISABLE
		 && wls_mos != MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux dis mos both fail ret=%d", __func__, ret);
                return ret;
            }
        }

        if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }
        if (wls_mos == MMI_DVCHG_MUX_CLOSE) {

            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_WPCGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }

        if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, SC8565_OVPGATE_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        } else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
#if 0
            ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
                BQ25980_EN_OTG, BQ25980_EN_OTG);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
                return ret;
            }
#endif
	     //reverse mode
	     if(bq->device_id != CPS2023H_DEVICE_ID  && bq->device_id != CPS2023_DEVICE_ID) {
	         sc858x_field_write(bq, MODE, 6);
	     }
	     bq-> otg_delay_mos_config = true;
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, SC8565_ACDRV_MANUAL);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);

            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, SC8565_OVPGATE_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }

#ifdef CONFIG_MOTO_CHANNEL_SWITCH
        } else if (typec_mos == MMI_DVCHG_MUX_OTG_WLC_OPEN) {
#if 0
            ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
                BQ25980_EN_OTG, BQ25980_EN_OTG);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
                return ret;
            }
#endif
	     //reverse mode
	     if(bq->device_id != CPS2023H_DEVICE_ID  && bq->device_id != CPS2023_DEVICE_ID) {
		  sc858x_field_write(bq, MODE, 6);
	     }
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, SC8565_ACDRV_MANUAL);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);

            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
#endif
        }

        if (wls_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_WPCGATE_EN, SC8565_WPCGATE_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  open wls mux fail ret=%d", __func__, ret);
                return ret;
            }
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
        }else if (wls_mos == MMI_DVCHG_MUX_OTG_WLC_OPEN) {
		//reverse mode
		if(bq->device_id != CPS2023H_DEVICE_ID  && bq->device_id != CPS2023_DEVICE_ID) {
			sc858x_field_write(bq, MODE, 6);
		}

		ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_WPCGATE_EN, SC8565_WPCGATE_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux  open wls mux fail ret=%d", __func__, ret);
                return ret;
            }
#endif
        } else if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, SC8565_ACDRV_MANUAL);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux menu close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            mdelay(50);

            ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_WPCGATE_EN, SC8565_WPCGATE_EN);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        }

	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, SC8565_ACDRV_MANUAL);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
	      ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_OVPGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}

	if (wls_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_ACDRV_MANUAL, SC8565_ACDRV_MANUAL);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
	     ret = regmap_update_bits(bq->regmap, SC8565_CHRGR_CTRL_1,
                    SC8565_WPCGATE_EN, 0);
            if (ret) {
                dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}

        ret = regmap_read(bq->regmap, SC8565_CHRGR_CTRL_5, &val);
       if (!ret)
               dev_err(bq->dev, "%s:mmi_mux Reg SC8565_CHRGR_CTRL_5 reg_0xF] = 0x%02X\n", __func__,val);

        ret = regmap_read(bq->regmap, SC8565_CHRGR_CTRL_1, &val);
        if (!ret)
                dev_err(bq->dev, "%s:mmi_mux Reg SC8565_CHRGR_CTRL_1 reg_0xB] = 0x%02X , device_id:%d\n", __func__, val, bq->device_id);

	sc858x_dump_reg(bq);
	return 0;
}

static int sc8565_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	struct sc858x_chip *bq  = charger_get_data(chg_dev);

	return sc858x_config_mux(bq, typec_mos, wls_mos);
}

static int sc8565_operation_mode_select(struct charger_device *chg_dev, int cp_op_mode)
{
	struct sc858x_chip *sc  = charger_get_data(chg_dev);
	int ret = 0;
	int reg_op_mode = 1;

	dev_info(sc->dev, "%s: %d\n",__func__, cp_op_mode);
	switch (cp_op_mode) {
	case CP_2_1_MODE:
		reg_op_mode = 1;
		break;
	case CP_4_1_MODE:
		reg_op_mode = 0;
		break;
	default:
		reg_op_mode = 1;
		break;
	}

	ret = sc858x_field_write(sc, MODE, reg_op_mode);
	if(ret < 0) {
		dev_err(sc->dev,"%s failed to set operation mode:%d\n", __func__, ret);
	}

	return ret;
}

static int sc8565_operation_get_max_mode(struct charger_device *chg_dev, int *cp_op_mode)
{
	int ret = 0;

	*cp_op_mode = CP_4_1_MODE;
	return ret;
}
#if 0
static const struct charger_ops sc8565_chg_ops = {
	.enable = sc8565_enable_chg,
	.is_enabled = mtk_sc858x_is_chg_enabled,
	.get_adc = sc8565_get_adc,
	.set_vbusovp = sc8565_set_vbusovp,
	.set_ibusocp = sc8565_set_ibusocp,
	.set_vbatovp = sc8565_set_vbatovp,
	.set_ibatocp = sc8565_set_ibatocp,
	.init_chip = sc8565_init_chip,
	.set_vbatovp_alarm = sc8565_set_vbatovp_alarm,
	.reset_vbatovp_alarm = sc8565_reset_vbatovp_alarm,
	.set_vbusovp_alarm = sc8565_set_vbusovp_alarm,
	.reset_vbusovp_alarm = sc8565_reset_vbusovp_alarm,
	.is_vbuslowerr = sc8565_is_vbuslowerr,
	.is_vbushigherr = sc8565_is_vbushigherr,
	.get_adc_accuracy = sc8565_get_adc_accuracy,
	.config_mux = sc8565_config_mux,
	.enable_adc = sc8565_enable_adc,
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	.get_vmos_chg = sc8565_get_vmos_chg,
#endif
};
#endif
static const struct charger_ops sc858x_chg_ops = {
	 .enable = sc8565_enable_chg,//mtk_sc858x_enable_chg,
	 .is_enabled = mtk_sc858x_is_chg_enabled,
	 .get_adc = mtk_sc858x_get_adc,
	 .get_adc_accuracy = mtk_sc858x_get_adc_accuracy,
	 .set_vbusovp = mtk_sc858x_set_vbusovp,
	 .set_ibusocp = mtk_sc858x_set_ibusocp,
	 .set_vbatovp = mtk_sc858x_set_vbatovp,
	 .set_ibatocp = mtk_sc858x_set_ibatocp,
	 .init_chip = mtk_sc858x_init_chip,
        .is_vbuslowerr = mtk_sc858x_is_vbuslowerr,
	 .is_vbushigherr = mtk_sc858x_is_vbushigher,
	 .set_vbatovp_alarm = mtk_sc858x_set_vbatovp_alarm,
	 .reset_vbatovp_alarm = mtk_sc858x_reset_vbatovp_alarm,
	 .set_vbusovp_alarm = mtk_sc858x_set_vbusovp_alarm,
	 .reset_vbusovp_alarm = mtk_sc858x_reset_vbusovp_alarm,
        .config_mux = sc8565_config_mux,
        .enable_adc = sc8565_enable_adc,
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	 .get_vmos_chg = sc8565_get_vmos_chg,
#endif
	.set_cp_operation_mode = sc8565_operation_mode_select,
	.get_cp_operation_max_mode = sc8565_operation_get_max_mode,
};

static int sc8565_register_chgdev(struct sc858x_chip *bq)
{
	bq->chg_prop.alias_name = bq->psy_desc.name;

	if(bq->mode == SC858X_SLAVE)
		bq->chg_dev = charger_device_register("secondary_dvchg", bq->dev,
						bq, &sc858x_chg_ops,
						&bq->chg_prop);
	else
		bq->chg_dev = charger_device_register("primary_dvchg", bq->dev,
						bq, &sc858x_chg_ops,
						&bq->chg_prop);
	return bq->chg_dev ? 0 : -EINVAL;
}

static int sc858x_parse_dt(struct sc858x_chip *sc, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc858x,vbat-ovp-dis", &(sc->cfg.vbat_ovp_dis)},
        {"sc,sc858x,vbat-ovp", &(sc->cfg.vbat_ovp)},
        {"sc,sc858x,ibat-ocp-dis", &(sc->cfg.ibat_ocp_dis)},
        {"sc,sc858x,ibat-ocp", &(sc->cfg.ibat_ocp)},
        {"sc,sc858x,vusb-ovp-dis", &(sc->cfg.vusb_ovp_dis)},
        {"sc,sc858x,vusb-ovp", &(sc->cfg.vusb_ovp)},
        {"sc,sc858x,vwpc-ovp-dis", &(sc->cfg.vwpc_ovp_dis)},
        {"sc,sc858x,vwpc-ovp", &(sc->cfg.vwpc_ovp)},
        {"sc,sc858x,vbus-ovp-dis", &(sc->cfg.vbus_ovp_dis)},
        {"sc,sc858x,vbus-ovp", &(sc->cfg.vbus_ovp)},
        {"sc,sc858x,vout-ovp-dis", &(sc->cfg.vout_ovp_dis)},
        {"sc,sc858x,vout-ovp", &(sc->cfg.vout_ovp)},
        {"sc,sc858x,ibus-ocp-dis", &(sc->cfg.ibus_ocp_dis)},
        {"sc,sc858x,ibus-ocp", &(sc->cfg.ibus_ocp)},
        {"sc,sc858x,ibus-ucp-fall-dis", &(sc->cfg.ibus_ucp_fall_dis)},
        {"sc,sc858x,ibus-ucp-fall", &(sc->cfg.ibus_ucp_fall)},
        {"sc,sc858x,pmid2out-ovp-dis", &(sc->cfg.pmid2out_ovp_dis)},
        {"sc,sc858x,pmid2out-ovp", &(sc->cfg.pmid2out_ovp)},
        {"sc,sc858x,pmid2out-uvp-dis", &(sc->cfg.pmid2out_uvp_dis)},
        {"sc,sc858x,pmid2out-uvp", &(sc->cfg.pmid2out_uvp)},
        {"sc,sc858x,fsw-set", &(sc->cfg.fsw_set)},
        {"sc,sc858x,ss-timeout", &(sc->cfg.ss_timeout)},
        {"sc,sc858x,wd-timeout", &(sc->cfg.wd_timeout)},
        {"sc,sc858x,ibat-sns-r", &(sc->cfg.ibat_sns_r)},
        {"sc,sc858x,mode", &(sc->cfg.mode)},
        {"sc,sc858x,tshut-dis", &(sc->cfg.tshut_dis)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            return ret;
        }
    }

    sc->irq_gpio = of_get_named_gpio(np, "sc858x,intr_gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        dev_err(sc->dev,"fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }

	sc->mmi_disable_mux = device_property_read_bool(sc->dev,
						      "mmi,disable_mux");

#ifdef CONFIG_MTK_CHARGER_V5P10
    if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        dev_err(sc->dev,"no charger name\n");
    }
#elif defined(CONFIG_MTK_CHARGER_V4P19)
    if (of_property_read_string(np, "charger_name_v4_19", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        dev_err(sc->dev,"no charger name\n");
    }
#endif /*CONFIG_MTK_CHARGER_V4P19*/

    return 0;
}

static struct of_device_id sc858x_charger_match_table[] = {
    {   .compatible = "sc,sc858x-standalone",
        .data = &sc858x_mode_data[SC858X_STANDALONG], },
    {   .compatible = "sc,sc858x-master",
        .data = &sc858x_mode_data[SC858X_MASTER], },
    {   .compatible = "sc,sc858x-slave",
        .data = &sc858x_mode_data[SC858X_SLAVE], },
    {},
};

static int sc858x_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc858x_chip *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret, i;

    dev_err(&client->dev, "%s (%s)\n", __func__, SC858X_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc858x_chip), GFP_KERNEL);
    if (!sc) {
        ret = -ENOMEM;
        goto err_kzalloc;
    }

    sc->dev = &client->dev;
    sc->client = client;
    mutex_init(&sc->irq_complete);

    sc->resume_completed = true;
    sc->irq_waiting = false;

	strncpy(sc->model_name, id->name, I2C_NAME_SIZE);
    dev_err(&client->dev, "%s (%s)\n", __func__, sc->model_name);

    sc->regmap = devm_regmap_init_i2c(client,
                            &sc858x_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        ret = PTR_ERR(sc->regmap);
        goto err_regmap_init;
    }

    for (i = 0; i < ARRAY_SIZE(sc858x_reg_fields); i++) {
        const struct reg_field *reg_fields = sc858x_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            ret = PTR_ERR(sc->rmap_fields[i]);
            goto err_regmap_field;
        }
    }

    ret = sc858x_detect_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s detect device fail\n", __func__);
        goto err_detect_dev;
    }

    i2c_set_clientdata(client, sc);
    sc858x_create_device_node(&(client->dev));

    match = of_match_node(sc858x_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        goto err_match_node;
    }

    sc858x_set_work_mode(sc, *(int *)match->data);
    if (ret) {
        dev_err(sc->dev,"Fail to set work mode!\n");
        goto err_set_mode;
    }

    ret = sc858x_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc858x_init_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    sc858x_init_cps2023_CN2VOUT(sc);

    ret = sc858x_psy_register(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s psy register failed(%d)\n", __func__, ret);
        goto err_register_psy;
    }

    ret = sc858x_register_interrupt(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

#ifdef CONFIG_MTK_CLASS
	ret = sc8565_register_chgdev(sc);
	if (ret < 0) {
		dev_err(sc->dev, "%s reg chgdev fail(%d)\n", __func__, ret);
        goto err_register_mtk_charger;
	}
#endif

#if 0//def CONFIG_MTK_CLASS
    sc->chg_dev = charger_device_register(sc->chg_dev_name,
					      &client->dev, sc,
					      &sc858x_chg_ops,
					      &sc858x_chg_props);
	if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_mtk_charger;
	}
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    sc->charger_pump = dvchg_register("sc_dvchg",
                             sc->dev, &sc_sc858x_dvchg_ops, sc);
    if (IS_ERR_OR_NULL(sc->charger_pump)) {
		ret = PTR_ERR(sc->charger_pump);
		dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_sc_charger;
	}
#endif /* CONFIG_SOUTHCHIP_DVCHG_CLASS */
    dev_err(sc->dev, "sc858x[%s] probe successfully!\n",
            sc->mode == SC858X_MASTER ? "master" : "slave");


	sc8565_create_device_node(sc->dev);

    return 0;

err_register_psy:
err_register_irq:
#ifdef CONFIG_MTK_CLASS
err_register_mtk_charger:
#endif /*CONFIG_MTK_CLASS*/
#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
err_register_sc_charger:
charger_device_unregister(sc->chg_dev);
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/
err_init_device:
    power_supply_unregister(sc->psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
err_regmap_init:
err_regmap_field:
    devm_kfree(&client->dev, sc);
err_kzalloc:
    dev_err(&client->dev,"sc858x probe fail\n");
    return ret;
}

static void sc858x_charger_remove(struct i2c_client *client)
{
    struct sc858x_chip *sc = i2c_get_clientdata(client);

    sc858x_enable_adc(sc, false);

    power_supply_unregister(sc->psy);
    mutex_destroy(&sc->irq_complete);
    devm_kfree(&client->dev, sc);
    dev_err(sc->dev,"remove Successfully\n");


}

static void sc8565_charger_shutdown(struct i2c_client *client)
{
	struct sc858x_chip *bq = i2c_get_clientdata(client);

	/*reg reset*/
	sc858x_reg_reset(bq);
	sc858x_enable_adc(bq, false);

	dev_err(bq->dev,"Shutdown Successfully\n");
}

#ifdef CONFIG_PM_SLEEP
static int sc858x_suspend(struct device *dev)
{
    struct sc858x_chip *sc = dev_get_drvdata(dev);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = false;
	mutex_unlock(&sc->irq_complete);
	dev_info(sc->dev, "Suspend successfully!");

    return 0;
}
static int sc858x_resume(struct device *dev)
{
    struct sc858x_chip *sc = dev_get_drvdata(dev);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = true;
	if (sc->irq_waiting) {
		sc->irq_disabled = false;
		enable_irq(sc->irq);
		mutex_unlock(&sc->irq_complete);
		sc858x_irq_handler(sc->irq, sc);
	} else {
		mutex_unlock(&sc->irq_complete);
	}

	power_supply_changed(sc->psy);
	dev_info(sc->dev, "Resume successfully!");

    return 0;
}
static const struct i2c_device_id bq25980_i2c_ids[] = {
	{ "sc858x-standalone", SC858X_STANDALONG },
	{ "sc858x-master", SC858X_MASTER },
	{ "sc858x-slave", SC858X_SLAVE },
	{},
};

static const struct dev_pm_ops sc858x_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc858x_suspend, sc858x_resume)
};
#endif

static struct i2c_driver sc858x_charger_driver = {
    .driver     = {
        .name   = "sc858x",
        .owner  = THIS_MODULE,
        .of_match_table = sc858x_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc858x_pm,
#endif
    },
    .probe      = sc858x_charger_probe,
    .id_table = bq25980_i2c_ids,
    .remove     = sc858x_charger_remove,
    .shutdown	= sc8565_charger_shutdown,
};

module_i2c_driver(sc858x_charger_driver);

MODULE_DESCRIPTION("SC SC858X Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");

