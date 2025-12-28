// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
 */
#ifndef __SC8586_REG__
#define __SC8586_REG__

#include <scp.h>
#include <charger_class.h>

struct sc_reg_field {
    uint32_t reg;
    uint32_t lsb;
    uint32_t msb;
    bool force_write;
};

#define sc8586_err(fmt, ...)       pr_err("sc8586_err:" fmt, ##__VA_ARGS__)
#define sc8586_info(fmt, ...)      pr_info("sc8586_info:" fmt, ##__VA_ARGS__)

#ifndef __maybe_unused
#define __maybe_unused
#endif

#define SC_REG_FIELD(_reg, _lsb, _msb) {           \
                    .reg = _reg,                \
                    .lsb = _lsb,                \
                    .msb = _msb,                \
                    }

#define SC_REG_FIELD_FORCE_WRITE(_reg, _lsb, _msb) {           \
                    .reg = _reg,                \
                    .lsb = _lsb,                \
                    .msb = _msb,                \
                    .force_write = true,        \
                    }

#define CONFIG_MTK_CLASS
#define SC8586_DEVICE_ID                0x85
#define SC8586_REG1A                    0x1A
#define SC8586_REG0F                    0x0F
#define SC8586_REG0B                    0x0B

#define CPS2043_DEVICE_ID               0x07

#define SC8586_REGMAX                   0xD1

//#define SC8586_REG7C                    0x7C
//#define SC8586_PRIVATE_CODE             0x01
enum sc8586_reg_range {
    SC8586_VBAT_OVP,
    SC8586_IBAT_OCP,
    SC8586_VBUS_OVP,
    SC8586_IBUS_OCP,
    CPS2043_VBUS_OVP,
};

enum sc8586_notify {
    SC8586_NOTIFY_OTHER = 0,
	SC8586_NOTIFY_IBUSOCP,
	SC8586_NOTIFY_VBUSOVP,
	SC8586_NOTIFY_IBATOCP,
	SC8586_NOTIFY_VBATOVP,
	SC8586_NOTIFY_VOUTOVP,
};

enum sc8586_dpdm_drive {
    SC8586_DPDM_HIZ,
    SC8586_DPDM_20K_PULL_DOWN,
    SC8586_DPDM_0P6V,
    SC8586_DPDM_2V,
    SC8586_DPDM_2P7V,
    SC8586_DPDM_3P3V,
};

enum sc8586_fc2_ctrl {
    SC8586_FC2_9V,
    SC8586_FC2_12V,
    SC8586_FC3,
    SC8586_FC2_5V,
};

enum sc8586_dpdm_vbus_stat {
    SC8586_VBUS_NO_INPUT,
    SC8586_VBUS_SDP,
    SC8586_VBUS_CDP,
    SC8586_VBUS_DCP,
    SC8586_VBUS_HVDCP,
    SC8586_VBUS_UNKNOWN,
    SC8586_VBUS_NON_STANDARD,
};

enum sc8586_dpdm_status {
    SC8586_DPDM_0P325V = 0x1,
    SC8586_DPDM_1P0V = 0x2,
    SC8586_DPDM_1P35V = 0x4,
    SC8586_DPDM_2P2V = 0x8,
    SC8586_DPDM_3V = 0x10,
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

#define SC8586_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
    .min = _min, \
    .max = _max, \
    .step = _step, \
    .offset = _offset, \
    .round_up = _ru, \
}

#define SC8586_CHG_RANGE_T(_table, _ru) \
    { .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }

#define DEVIDE_10(x) (x / 10)

static const struct reg_range sc8586_reg_range[] = {
    [SC8586_VBAT_OVP]      = SC8586_CHG_RANGE(4450, DEVIDE_10(53875), DEVIDE_10(125), 3800, false),
    [SC8586_IBAT_OCP]      = SC8586_CHG_RANGE(1260, 22680, 180, 0, false),
    [SC8586_VBUS_OVP]      = SC8586_CHG_RANGE(7500, 13500, 400, 7500, false), /*2:1*/
    [SC8586_IBUS_OCP]      = SC8586_CHG_RANGE(750, 6750, 75, 0, false),
    [CPS2043_VBUS_OVP]      = SC8586_CHG_RANGE(7000, 13500, 200, 7000, false), /*2:1*/
};

enum sc8586_fields {
    F_DEVICE_VER, /*0x00*/
    F_VBAT_OVP_DIS, F_VBAT_OVP, /*0x01*/
    F_IBAT_OCP_DIS, F_IBAT_OCP, /*0x02*/
    F_OVPGATE_ON_DG, F_VUSB_OVP_MASK, F_VUSB_OVP_FLAG, F_VUSB_OVP_STAT, F_VUSB_OVP, /*0x03*/
    F_VBUS_OVP, F_VOUT_OVP, /*0x05*/
    F_IBUS_OCP_DIS, F_IBUS_OCP, /*0x06*/
    F_IBUS_UCP_DIS, F_IBUS_UCP_FALL_DG_SET, F_IBUS_UCP_RISE_MASK, F_IBUS_UCP_RISE_FLAG,
    F_IBUS_UCP_FALL_MASK, F_IBUS_UCP_FALL_FLAG, /*0x07*/
    F_PMID2OUT_OVP_DIS, F_PMID2OUT_OVP_BLK, F_PMID2OUT_OVP_MASK,
    F_PMID2OUT_OVP_FLAG, F_PMID2OUT_OVP, /*0x08*/
    F_PMID2OUT_UVP_DIS, F_PMID2OUT_UVP_BLK, F_PMID2OUT_UVP_MASK,
    F_PMID2OUT_UVP_FLAG, F_PMID2OUT_UVP, /*0x09*/
    F_POR_FLAG, F_ACRB_USB_STAT, F_PMID_ERROR_LO_STAT, F_PMID_ERROR_HI_STAT,
    F_QB_ON_STAT, F_CP_SWITCHING_STAT, F_PIN_DIAG_FAIL_FLAG, /*0x0A*/
    F_CP_EN, F_QB_EN, F_ACDRV_MANUAL_EN, F_OVPGATE_EN,
    F_VBUS_PD_EN, F_VUSB_AUTO_PD_EN, F_VUSB_PD_EN, /*0x0B*/
    F_FSW_SET, F_FREQ_DITHER, /*0x0C*/
    F_PMID_INRANGE_DET_DIS, F_FORCE_VAC_OK, F_SS_TIMEOUT, F_WD_TIMEOUT, /*0x0D*/
    F_SET_IBAT_SNS_RES_0P5, F_VBAT_OVP_DG_SET, F_SET_IBAT_SNS_RES, F_REG_RST, F_MODE, /*0x0E*/
    F_OVPGATE_STAT, F_ACDRV_SS, F_TSHUT_DIS, F_FORCE_LDO_EN, F_VUSB_OVP_DIS, F_VBUS_OVP_DIS, F_VOUT_OVP_DIS, /*0x0F*/
    F_USB_PD_FLAG, F_USB_PD_MASK, F_USB_PD_EXIT_FLAG, F_USB_PD_EXIT_MASK,
    F_IBATSNS_HS_EN, F_IBUS_UCP_FALL_BLANKING_SET, F_IBUS_UCP_EN_METHOD_SEL, /*0x10*/
    F_ADC_EN, F_ADC_RATE, F_ADC_FREEZE, F_VBAT2_ADC_EN, /*0x18*/
    F_VBAT2_OVP_DIS, /*0x3D*/
    F_ZVS_DLY_SET, F_ZVS_MODE, /*0x3E*/
    F_VOUT_FAST_DROP_DIS, F_VOUT_FAST_DROP_FLAG, F_VOUT_FAST_DROP_MASK,
    F_VOUT_DELTA, F_VOUT_DELTA_AVG, /*0x3F*/
    F_VBAT_OVP_DG_SET_EX, F_IBUS_OCP_DG_SET, F_IBAT_OCP_DG_SET, /*0xD1*/

    F_SCP_EN, F_SCP_SOFT_RST, F_TX_CRC_DIS, F_DPDM_3P3_EN,
    F_CLR_TX_FIFO, F_CLR_RX_FIFO, F_SND_RST_TRANS, F_SND_TRANS, /*0x2E*/
    F_SCP_TX_DATA, /*0x2F*/
    F_SCP_RX_DATA, /*0x30*/
    F_SCP_TX_FIFO_CNT_STAT, F_SCP_RX_FIFO_CNT_STAT, /*0x31*/

    F_FORCE_DPDM, F_AUTO_DPDM, F_HVDCP_EN, F_FC_EN, /*0x34*/
    F_DP_DRIVE, F_DM_DRIVE, /*0x35*/
    F_FC3_MINUS, F_FC3_PLUS, F_FC2_SET, /*0x36*/
    F_VBUS_STAT, /*0x37*/
    F_DP_HIGH_TRAN_3V, F_DP_HIGH_TRAN_2P2V, F_DP_HIGH_TRAN_1P35V,
    F_DP_HIGH_TRAN_1P0V, F_DP_HIGH_TRAN_0P325V, /*0x39*/
    F_DM_HIGH_TRAN_3V, F_DM_HIGH_TRAN_2P2V, F_DM_HIGH_TRAN_1P35V,
    F_DM_HIGH_TRAN_1P0V, F_DM_HIGH_TRAN_0P325V, /*0x3A*/
    F_DPDM_POLLING_EN, /*0x3B*/
    F_DPDM_EN, /*0x3C*/

    F_DEVICE_ID, /*0x6E*/
	F_CPS_DEVICE_ID, /*0x4d*/
    F_MAX_FIELDS,
};

static const struct sc_reg_field sc8586_reg_fields[] = {
    /*reg00*/
    [F_DEVICE_VER] = SC_REG_FIELD(0x00, 0, 7),
    /*reg01*/
    [F_VBAT_OVP_DIS] = SC_REG_FIELD(0x01, 7, 7),
    [F_VBAT_OVP] = SC_REG_FIELD(0x01, 0, 6),
    /*reg02*/
    [F_IBAT_OCP_DIS] = SC_REG_FIELD(0x02, 7, 7),
    [F_IBAT_OCP] = SC_REG_FIELD(0x02, 0, 6),
    /*reg03*/
    [F_OVPGATE_ON_DG] = SC_REG_FIELD(0x03, 7, 7),
    [F_VUSB_OVP_MASK] = SC_REG_FIELD(0x03, 6, 6),
    [F_VUSB_OVP_FLAG] = SC_REG_FIELD(0x03, 5, 5),
    [F_VUSB_OVP_STAT] = SC_REG_FIELD(0x03, 4, 4),
    [F_VUSB_OVP] = SC_REG_FIELD(0x03, 0, 3),
    /*reg05*/
    [F_VBUS_OVP] = SC_REG_FIELD(0x05, 2, 5),
    [F_VOUT_OVP] = SC_REG_FIELD(0x05, 0, 1),
    /*reg06*/
    [F_IBUS_OCP_DIS] = SC_REG_FIELD(0x06, 7, 7),
    [F_IBUS_OCP] = SC_REG_FIELD(0x06, 0, 6),
    /*reg07*/
    [F_IBUS_UCP_DIS] = SC_REG_FIELD(0x07, 7, 7),
    [F_IBUS_UCP_FALL_DG_SET] = SC_REG_FIELD(0x07, 4, 5),
    [F_IBUS_UCP_RISE_MASK] = SC_REG_FIELD(0x07, 3, 3),
    [F_IBUS_UCP_RISE_FLAG] = SC_REG_FIELD(0x07, 2, 2),
    [F_IBUS_UCP_FALL_MASK] = SC_REG_FIELD(0x07, 1, 1),
    [F_IBUS_UCP_FALL_FLAG] = SC_REG_FIELD(0x07, 0, 0),
    /*reg08*/
    [F_PMID2OUT_OVP_DIS] = SC_REG_FIELD(0x08, 7, 7),
    [F_PMID2OUT_OVP_BLK] = SC_REG_FIELD(0x08, 5, 6),
    [F_PMID2OUT_OVP_MASK] = SC_REG_FIELD(0x08, 4, 4),
    [F_PMID2OUT_OVP_FLAG] = SC_REG_FIELD(0x08, 3, 3),
    [F_PMID2OUT_OVP] = SC_REG_FIELD(0x08, 0, 2),
    /*reg09*/
    [F_PMID2OUT_UVP_DIS] = SC_REG_FIELD(0x09, 7, 7),
    [F_PMID2OUT_UVP_BLK] = SC_REG_FIELD(0x09, 5, 6),
    [F_PMID2OUT_UVP_MASK] = SC_REG_FIELD(0x09, 4, 4),
    [F_PMID2OUT_UVP_FLAG] = SC_REG_FIELD(0x09, 3, 3),
    [F_PMID2OUT_UVP] = SC_REG_FIELD(0x09, 0, 2),
    /*reg0a*/
    [F_POR_FLAG] = SC_REG_FIELD(0x0a, 7, 7),
    [F_ACRB_USB_STAT] = SC_REG_FIELD(0x0a, 5, 5),
    [F_PMID_ERROR_LO_STAT] = SC_REG_FIELD(0x0a, 4, 4),
    [F_PMID_ERROR_HI_STAT] = SC_REG_FIELD(0x0a, 3, 3),
    [F_QB_ON_STAT] = SC_REG_FIELD(0x0a, 2, 2),
    [F_CP_SWITCHING_STAT] = SC_REG_FIELD(0x0a, 1, 1),
    [F_PIN_DIAG_FAIL_FLAG] = SC_REG_FIELD(0x0a, 0, 0),
    /*reg0b*/
    [F_CP_EN] = SC_REG_FIELD(0x0b, 7, 7),
    [F_QB_EN] = SC_REG_FIELD(0x0b, 6, 6),
    [F_ACDRV_MANUAL_EN] = SC_REG_FIELD(0x0b, 5, 5),
    [F_OVPGATE_EN] = SC_REG_FIELD(0x0b, 3, 3),
    [F_VBUS_PD_EN] = SC_REG_FIELD(0x0b, 2, 2),
    [F_VUSB_AUTO_PD_EN] = SC_REG_FIELD(0x0b, 1, 1),
    [F_VUSB_PD_EN] = SC_REG_FIELD(0x0b, 0, 0),
    /*reg0c*/
    [F_FSW_SET] = SC_REG_FIELD(0x0c, 3, 7),
    [F_FREQ_DITHER] = SC_REG_FIELD(0x0c, 1, 1),
    /*reg0d*/
    [F_PMID_INRANGE_DET_DIS] = SC_REG_FIELD(0x0d, 7, 7),
    [F_FORCE_VAC_OK] = SC_REG_FIELD(0x0d, 6, 6),
    [F_SS_TIMEOUT] = SC_REG_FIELD(0x0d, 3, 5),
    [F_WD_TIMEOUT] = SC_REG_FIELD(0x0d, 0, 2),
    /*reg0e*/
    [F_SET_IBAT_SNS_RES_0P5] = SC_REG_FIELD(0x0e, 7, 7),
    [F_VBAT_OVP_DG_SET] = SC_REG_FIELD(0x0e, 5, 5),
    [F_SET_IBAT_SNS_RES] = SC_REG_FIELD(0x0e, 4, 4),
    [F_REG_RST] = SC_REG_FIELD(0x0e, 3, 3),
    [F_MODE] = SC_REG_FIELD(0x0e, 0, 2),
    /*reg0f*/
    [F_OVPGATE_STAT] = SC_REG_FIELD(0x0f, 7, 7),
    [F_ACDRV_SS] = SC_REG_FIELD(0x0f, 5, 6),
    [F_TSHUT_DIS] = SC_REG_FIELD(0x0f, 4, 4),
    [F_FORCE_LDO_EN] = SC_REG_FIELD(0x0f, 3, 3),
    [F_VUSB_OVP_DIS] = SC_REG_FIELD(0x0f, 2, 2),
    [F_VBUS_OVP_DIS] = SC_REG_FIELD(0x0f, 1, 1),
    [F_VOUT_OVP_DIS] = SC_REG_FIELD(0x0f, 0, 0),
    /*reg10*/
    [F_USB_PD_FLAG] = SC_REG_FIELD(0x10, 7, 7),
    [F_USB_PD_MASK] = SC_REG_FIELD(0x10, 6, 6),
    [F_USB_PD_EXIT_FLAG] = SC_REG_FIELD(0x10, 5, 5),
    [F_USB_PD_EXIT_MASK] = SC_REG_FIELD(0x10, 4, 4),
    [F_IBATSNS_HS_EN] = SC_REG_FIELD(0x10, 3, 3),
    [F_IBUS_UCP_FALL_BLANKING_SET] = SC_REG_FIELD(0x10, 1, 2),
    [F_IBUS_UCP_EN_METHOD_SEL] = SC_REG_FIELD(0x10, 0, 0),
    /*reg18*/
    [F_ADC_EN] = SC_REG_FIELD(0x18, 7, 7),
    [F_ADC_RATE] = SC_REG_FIELD(0x18, 6, 6),
    [F_ADC_FREEZE] = SC_REG_FIELD(0x18, 2, 2),
    [F_VBAT2_ADC_EN] = SC_REG_FIELD(0x18, 1, 1),
    /*reg3d*/
    [F_VBAT2_OVP_DIS] = SC_REG_FIELD(0x3d, 7, 7),
    /*reg3e*/
    [F_ZVS_DLY_SET] = SC_REG_FIELD(0x3e, 2, 7),
    [F_ZVS_MODE] = SC_REG_FIELD(0x3e, 0, 1),
    /*reg3f*/
    [F_VOUT_FAST_DROP_DIS] = SC_REG_FIELD(0x3f, 7, 7),
    [F_VOUT_FAST_DROP_FLAG] = SC_REG_FIELD(0x3f, 6, 6),
    [F_VOUT_FAST_DROP_MASK] = SC_REG_FIELD(0x3f, 5, 5),
    [F_VOUT_DELTA] = SC_REG_FIELD(0x3f, 2, 3),
    [F_VOUT_DELTA_AVG] = SC_REG_FIELD(0x3f, 0, 1),
    /*regd1*/
    [F_VBAT_OVP_DG_SET_EX] = SC_REG_FIELD(0xd1, 6, 7),
    [F_IBUS_OCP_DG_SET] = SC_REG_FIELD(0xd1, 4, 5),
    [F_IBAT_OCP_DG_SET] = SC_REG_FIELD(0xd1, 2, 3),

    /*reg2e*/
    [F_SCP_EN] = SC_REG_FIELD(0x2e, 7, 7),
    [F_SCP_SOFT_RST] = SC_REG_FIELD(0x2e, 6, 6),
    [F_TX_CRC_DIS] = SC_REG_FIELD(0x2e, 5, 5),
    [F_DPDM_3P3_EN] = SC_REG_FIELD(0x2e, 4, 4),
    [F_CLR_TX_FIFO] = SC_REG_FIELD(0x2e, 3, 3),
    [F_CLR_RX_FIFO] = SC_REG_FIELD(0x2e, 2, 2),
    [F_SND_RST_TRANS] = SC_REG_FIELD(0x2e, 1, 1),
    [F_SND_TRANS] = SC_REG_FIELD(0x2e, 0, 0),
    /*reg2f*/
    [F_SCP_TX_DATA] = SC_REG_FIELD(0x2f, 0, 7),
    /*reg30*/
    [F_SCP_RX_DATA] = SC_REG_FIELD(0x30, 0, 7),
    /*reg31*/
    [F_SCP_TX_FIFO_CNT_STAT] = SC_REG_FIELD(0x31, 4, 7),
    [F_SCP_RX_FIFO_CNT_STAT] = SC_REG_FIELD(0x31, 0, 3),

    /*reg34*/
    [F_FORCE_DPDM] = SC_REG_FIELD_FORCE_WRITE(0x34, 7, 7),
    [F_AUTO_DPDM] = SC_REG_FIELD(0x34, 6, 6),
    [F_HVDCP_EN] = SC_REG_FIELD(0x34, 5, 5),
    [F_FC_EN] = SC_REG_FIELD(0x34, 0, 0),
    /*reg35*/
    [F_DP_DRIVE] = SC_REG_FIELD(0x35, 5, 7),
    [F_DM_DRIVE] = SC_REG_FIELD(0x35, 2, 4),
    /*reg36*/
    [F_FC3_MINUS] = SC_REG_FIELD(0x36, 3, 3),
    [F_FC3_PLUS] = SC_REG_FIELD(0x36, 2, 2),
    [F_FC2_SET] = SC_REG_FIELD(0x36, 0, 1),
    /*reg37*/
    [F_VBUS_STAT] = SC_REG_FIELD(0x37, 5, 7),
    /*reg39*/
    [F_DP_HIGH_TRAN_3V] = SC_REG_FIELD(0x39, 4, 4),
    [F_DP_HIGH_TRAN_2P2V] = SC_REG_FIELD(0x39, 3, 3),
    [F_DP_HIGH_TRAN_1P35V] = SC_REG_FIELD(0x39, 2, 2),
    [F_DP_HIGH_TRAN_1P0V] = SC_REG_FIELD(0x39, 1, 1),
    [F_DP_HIGH_TRAN_0P325V] = SC_REG_FIELD(0x39, 0, 0),
    /*reg3a*/
    [F_DM_HIGH_TRAN_3V] = SC_REG_FIELD(0x3a, 4, 4),
    [F_DM_HIGH_TRAN_2P2V] = SC_REG_FIELD(0x3a, 3, 3),
    [F_DM_HIGH_TRAN_1P35V] = SC_REG_FIELD(0x3a, 2, 2),
    [F_DM_HIGH_TRAN_1P0V] = SC_REG_FIELD(0x3a, 1, 1),
    [F_DM_HIGH_TRAN_0P325V] = SC_REG_FIELD(0x3a, 0, 0),
    /*reg3b*/
    [F_DPDM_POLLING_EN] = SC_REG_FIELD(0x3b, 7, 7),
    /*reg3c*/
    [F_DPDM_EN] = SC_REG_FIELD(0x3c, 0, 0),
    /*reg6e*/
    [F_DEVICE_ID] = SC_REG_FIELD(0x6e, 0, 7),
	/*reg4d*/
	[F_CPS_DEVICE_ID] = SC_REG_FIELD(0x4d, 0, 7),
};

struct sc8586_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp;
    int ibat_ocp_dis;
    int ibat_ocp;
    int vusb_ovp_dis;
    int vusb_ovp;
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
    int vbat2_adc_en;
    int scp_block_en;
    int dpdm_block_en;
};

struct sc8586_chip {
    struct device *dev;
    struct i2c_client *client;

    struct sc8586_cfg_e cfg;
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

    int fc3_volt_gear;

#ifdef CONFIG_MTK_CLASS
    struct charger_device *chg_dev;
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    struct dvchg_dev *charger_pump;
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

#ifdef CONFIG_SOUTHCHIP_ADAPTER_CLASS
    struct scp_dev *cp_scp_dev;
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

/*********************I2C API*********************/
u8 val2reg(enum sc8586_reg_range id, u32 val);
u32 reg2val(enum sc8586_reg_range id, u8 reg);
int sc8586_i2c_write_bytes(struct sc8586_chip *sc, uint8_t reg, uint8_t len, uint8_t *val);
int sc8586_i2c_read_bytes(struct sc8586_chip *sc, uint8_t reg, uint8_t len, uint8_t *val);
int sc8586_i2c_write_byte(struct sc8586_chip *sc, uint8_t reg, uint8_t val);
int sc8586_i2c_read_byte(struct sc8586_chip *sc, uint8_t reg, uint8_t *val);
int sc8586_field_read(struct sc8586_chip *sc,
                            enum sc8586_fields field_id, int *val);
int sc8586_field_write(struct sc8586_chip *sc,
                            enum sc8586_fields field_id, int val);

void sc8586_dump_check_scp_fault_status(struct sc8586_chip *sc);
int sc8586_get_scp_enable(struct sc8586_chip *sc);

void sc8586_dump_check_dpdm_fault_status(struct sc8586_chip *sc);
int sc8586_get_dpdm_enable(struct sc8586_chip *sc);

int sc8586_set_scp_enable(struct sc8586_chip *sc, bool enable);
int sc8586_get_scp_enable(struct sc8586_chip *sc);
int sc8586_scp_start_transaction(struct sc8586_chip *sc);
int sc8586_scp_write_tx_fifo(struct sc8586_chip *sc, uint8_t val);
#ifdef CONFIG_SOUTHCHIP_ADAPTER_CLASS
int sc8586_scp_handshake(struct scp_dev *scp);
int sc8586_scp_reset(struct scp_dev *scp);
int sc8586_scp_send_data(struct scp_dev *scp,
            uint8_t *data, uint8_t len, uint8_t *recv_data);
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

int sc8586_set_dpdm_enable(struct sc8586_chip *sc, bool enable);
int sc8586_get_dpdm_enable(struct sc8586_chip *sc);
int sc8586_set_force_dpdm(struct sc8586_chip *sc);
int sc8586_get_vbus_stat(struct sc8586_chip *sc);
int sc8586_set_fc_func_enable(struct sc8586_chip *sc, bool enable);
int sc8586_set_hvdcp_enable(struct sc8586_chip *sc, bool enable);
int sc8586_set_dp_drive(struct sc8586_chip *sc, enum sc8586_dpdm_drive dp_val);
int sc8586_set_dm_drive(struct sc8586_chip *sc, enum sc8586_dpdm_drive dm_val);
#endif
