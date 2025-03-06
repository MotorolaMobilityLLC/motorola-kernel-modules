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

#include <charger_class.h>


#define SC8541D_DRV_VERSION              "1.1.0_G"

#define SC8541D_DEVICE_ID                0x03

#define SC8541D_CTRL5_REG                0x0D

#define SC8541D_REG_11                   0x11

#define SC8541D_REG_15                   0x15

#define SC8541D_REG_1F                   0x1F

#define SC8541D_REGMAX                   0x2C

enum {
    SC8541D_STANDALONE,
    SC8541D_SLAVE,
    SC8541D_MASTER,
};

static const char* sc8541d_psy_name[] = {
    [SC8541D_STANDALONE] = "cp-standalone",
    [SC8541D_MASTER] = "cp-master",
    [SC8541D_SLAVE] = "cp-slave",
};

static const char* sc8541d_irq_name[] = {
    [SC8541D_STANDALONE] = "sc8541d-standalone-irq",
    [SC8541D_MASTER] = "sc8541d-master-irq",
    [SC8541D_SLAVE] = "sc8541d-slave-irq",
};

static int sc8541d_role_data[] = {
    [SC8541D_STANDALONE] = SC8541D_STANDALONE,
    [SC8541D_MASTER] = SC8541D_MASTER,
    [SC8541D_SLAVE] = SC8541D_SLAVE,
};

enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VAC,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_THEM,
    ADC_TDIE,
    ADC_MAX_NUM,
}ADC_CH_E;


static const u32 sc8541d_adc_accuracy_tbl[ADC_MAX_NUM] = {
    150000,	/* IBUS */
    35000,	/* VBUS */
    20000,	/* VOUT */
    20000,	/* VBAT */
    200000,	/* IBAT */
    0,	/* THEM */
    4,	/* TDIE */
};
enum sc8541d_notify {
    SC8541D_NOTIFY_OTHER = 0,
    SC8541D_NOTIFY_IBUSUCPF,
    SC8541D_NOTIFY_VBUSOVPALM,
    SC8541D_NOTIFY_VBATOVPALM,
    SC8541D_NOTIFY_IBUSOCP,
    SC8541D_NOTIFY_VBUSOVP,
    SC8541D_NOTIFY_IBATOCP,
    SC8541D_NOTIFY_VBATOVP,
    SC8541D_NOTIFY_VOUTOVP,
    SC8541D_NOTIFY_VDROVP,
};


enum sc8541d_fields{
    VBAT_OVP_DIS, VBAT_OVP,     /*reg00h*/
    IBAT_OVP_DIS, IBAT_OCP,     /*reg01h*/
    VBUS_OVP_DIS, VAC_OVP_DIS, VAC_OVP, VBUS_OVP,     /*reg02h*/
    VERSION_ID, DEVICE_ID,      /*reg03h*/
    IBUS_OCP_DIS, IBUS_OCP,     /*reg04h*/
    PMID2OUT_OVP_DIS, PMID2OUT_OVP_CFG, PMID2OUT_OVP, /*reg05h*/
    PMID2OUT_UVP_DIS, PMID2OUT_UVP, /*reg06h*/
    SET_IBATREG, SET_IBUSREG, SET_VBATREG, SET_TDIEREG, /*reg07h*/
    THEM_FLT_DIS, THEM_FLT,       /*reg08h*/
    SET_IBAT_SNS_RES, VAC_PD_EN, PMID_PD_EN, VBUS_PD_EN, REG_RST, VOUT_OVP_DIS, MODE, /*reg09h*/
    SS_TIMEOUT, IBUS_UCP_FALL_BLANKING_SET,FORCE_VAC_OK, WD_TIMEOUT,  /*reg0Ah*/
    VOUT_OVP, FREQ_SHIFT, FSW_SET,  /*reg0Bh*/
    CP_EN, VBUS_SHORT_DIS, PIN_DIAG_FALL_DIS, IBATSNS_HS_EN, EN_IBATSNS,/*reg0Ch*/
    WD_TIMEOUT_DIS, ACDRV_MANUAL_EN, ACDRV_EN, OTG_EN, ACDRV_PRE_DIS, ACDRV_DIS,/*reg0Dh*/
        TSHUT_DIS, IBUS_UCP_DIS,
    IBAT_REG_DIS, IBUS_REG_DIS, VBAT_REG_DIS, TDIE_REG_DIS,/*reg0Eh*/
    PMID2OUT_UVP_DG_SET, PMID2OUT_OVP_DG_SET, IBAT_OCP_DG_SET, VBUS_OCP_DG_SET,
        VOUT_OVP_DG_SET,    /*reg0Fh*/
    IBUS_UCP_FALL_DG_SET, IBUS_OCP_DG_SET, VBAT_OVP_DG_SET, /*reg10h*/
    CP_SWITCHING_STAT,/*reg11h*/
    VBUS2OUT_OVP_STAT, VBUS2OUT_UVP_STAT,/*reg14h*/
    VBUS_PRESENT_MASK, VOUT_TH_CHG_EN_MASK, VOUT_TH_REV_EN_MASK,/*reg19h*/
    ADC_EN, ADC_RATE, /*reg1Dh*/

    F_MAX_FIELDS,
};

struct sc8541d_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp_th;
    int ibat_ocp_dis;
    int ibat_ocp_th;
    int vbus_ovp_dis;
    int vbus_ovp_th;
    int vac_ovp_dis;
    int vac_ovp_th;
    int ibus_ocp_dis;
    int ibus_ocp_th;
    int pmid2out_ovp_dis;
    int pmid2out_ovp_th;
    int pmid2out_uvp_dis;
    int pmid2out_uvp_th;
    int ibatreg_th;
    int ibusreg_th;
    int vbatreg_th;
    int tdiereg_th;
    int them_flt_th;
    int ibat_sns_res;
    int vac_pd_en;
    int pmid_pd_en;
    int vbus_pd_en;
    int work_mode;
    int ss_timeout;
    int wd_timeout;
    int vout_ovp_th;
    int fsw_freq;
    int wd_timeout_dis;
    int pin_diag_fall_dis;
    int vout_ovp_dis;
    int them_flt_dis;
    int tshut_dis;
    int ibus_ucp_dis;
    int ibat_reg_dis;
    int ibus_reg_dis;
    int vbat_reg_dis;
    int tdie_reg_dis;
    int ibat_ocp_dg;
    int ibus_ucp_fall_dg;
    int ibus_ocp_dg;
    int vbat_ovp_dg;
};

struct sc8541d_chip {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];

    int reg_addr;
    int reg_data;

    int device_id;
    int version_id;

    int role;

    bool usb_present;
    int charge_enabled;

    int irq_gpio;
    int irq;

    /* ADC reading */
    int vbat_volt;
    int vbus_volt;
    int ibat_curr;
    int ibus_curr;
    int die_temp;

    struct charger_device *chg_dev;

    const char *chg_dev_name;

    struct sc8541d_cfg_e cfg;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *sc_cp_psy;
};

//sc8541d registers
static const struct reg_field sc8541d_reg_fields[] = {
    /*reg00*/
    [VBAT_OVP_DIS] = REG_FIELD(0x00, 7, 7),
    [VBAT_OVP] = REG_FIELD(0x00, 0, 6),
    /*reg01*/
    [IBAT_OVP_DIS] = REG_FIELD(0x01, 7, 7),
    [IBAT_OCP] = REG_FIELD(0x01, 0, 6),
    /*reg02*/
    [VBUS_OVP_DIS] = REG_FIELD(0x02, 7, 7),
    [VAC_OVP_DIS] = REG_FIELD(0x02, 6, 6),
    [VAC_OVP] = REG_FIELD(0x02, 3, 5),
    [VBUS_OVP] = REG_FIELD(0x02, 0, 2),
    /*reg03*/
    [VERSION_ID] = REG_FIELD(0x03, 4, 7),
    [DEVICE_ID] = REG_FIELD(0x03, 0, 3),
    /*reg04*/
    [IBUS_OCP_DIS] = REG_FIELD(0x04, 7, 7),
    [IBUS_OCP] = REG_FIELD(0x04, 0, 6),
    /*reg05*/
    [PMID2OUT_OVP_DIS] = REG_FIELD(0x05, 7, 7),
    [PMID2OUT_OVP_CFG] = REG_FIELD(0x05, 5, 5),
    [PMID2OUT_OVP] = REG_FIELD(0x05, 0, 2),
    /*reg06*/
    [PMID2OUT_UVP_DIS] = REG_FIELD(0x06, 7, 7),
    [PMID2OUT_UVP] = REG_FIELD(0x06, 0, 2),
    /*reg07*/
    [SET_IBATREG] = REG_FIELD(0x07, 6, 7),
    [SET_IBUSREG] = REG_FIELD(0x07, 4, 5),
    [SET_VBATREG] = REG_FIELD(0x07, 2, 3),
    [SET_TDIEREG] = REG_FIELD(0x07, 0, 1),
    /*reg08*/
    [THEM_FLT_DIS] = REG_FIELD(0x08, 7, 7),
    [THEM_FLT] = REG_FIELD(0x08, 0, 5),
    /*reg09*/
    [SET_IBAT_SNS_RES] = REG_FIELD(0x09, 7, 7),
    [VAC_PD_EN] = REG_FIELD(0x09, 6, 6),
    [PMID_PD_EN] = REG_FIELD(0x09, 5, 5),
    [VBUS_PD_EN] = REG_FIELD(0x09, 4, 4),
    [REG_RST] = REG_FIELD(0x09, 3, 3),
    [VOUT_OVP_DIS] = REG_FIELD(0x09, 2, 2),
    [MODE] = REG_FIELD(0x09, 0, 1),
    /*reg0A*/
    [SS_TIMEOUT] = REG_FIELD(0x0A, 5, 7),
    [IBUS_UCP_FALL_BLANKING_SET] = REG_FIELD(0x0A, 3, 4),
    [FORCE_VAC_OK] = REG_FIELD(0x0A, 2, 2),
    [WD_TIMEOUT] = REG_FIELD(0x0A, 0, 1),
    /*reg0B*/
    [VOUT_OVP] = REG_FIELD(0x0B, 7, 7),
    [FREQ_SHIFT] = REG_FIELD(0x0B, 4, 5),
    [FSW_SET] = REG_FIELD(0x0B, 0, 3),
    /*reg0C*/
    [CP_EN] = REG_FIELD(0x0C, 7, 7),
    [VBUS_SHORT_DIS] = REG_FIELD(0x0C, 5, 5),
    [PIN_DIAG_FALL_DIS] = REG_FIELD(0x0C, 4, 4),
    [IBATSNS_HS_EN] = REG_FIELD(0x0C, 1, 1),
    [EN_IBATSNS] = REG_FIELD(0x0C, 0, 0),
    /*reg0D*/
    [WD_TIMEOUT_DIS] = REG_FIELD(0x0D, 7, 7),
    [ACDRV_MANUAL_EN] = REG_FIELD(0x0D, 6, 6),
    [ACDRV_EN] = REG_FIELD(0x0D, 5, 5),
    [OTG_EN] = REG_FIELD(0x0D, 4, 4),
    [ACDRV_PRE_DIS] = REG_FIELD(0x0D, 3, 3),
    [ACDRV_DIS] = REG_FIELD(0x0D, 2, 2),
    [TSHUT_DIS] = REG_FIELD(0x0D, 1, 1),
    [IBUS_UCP_DIS] = REG_FIELD(0x0D, 0, 0),
    /*reg0E*/
    [IBAT_REG_DIS] = REG_FIELD(0x0E, 3, 3),
    [IBUS_REG_DIS] = REG_FIELD(0x0E, 2, 2),
    [VBAT_REG_DIS] = REG_FIELD(0x0E, 1, 1),
    [TDIE_REG_DIS] = REG_FIELD(0x0E, 0, 0),
    /*reg0F*/
    [PMID2OUT_UVP_DG_SET] = REG_FIELD(0x0F, 7, 7),
    [PMID2OUT_OVP_DG_SET] = REG_FIELD(0x0F, 6, 6),
    [IBAT_OCP_DG_SET] = REG_FIELD(0x0F, 4, 5),
    [VBUS_OCP_DG_SET] = REG_FIELD(0x0F, 2, 2),
    [VOUT_OVP_DG_SET] = REG_FIELD(0x0F, 1, 1),
    /*reg10*/
    [IBUS_UCP_FALL_DG_SET] = REG_FIELD(0x10, 6, 7),
    [IBUS_OCP_DG_SET] = REG_FIELD(0x10, 4, 5),
    [VBAT_OVP_DG_SET] = REG_FIELD(0x10, 2, 3),
    /*reg11*/
    [CP_SWITCHING_STAT] = REG_FIELD(0x11, 7, 7),
    /*reg14*/
    [VBUS2OUT_OVP_STAT] = REG_FIELD(0x14, 4, 4),
    [VBUS2OUT_UVP_STAT] = REG_FIELD(0x14, 5, 5),
    /*reg19*/
    [VOUT_TH_REV_EN_MASK] = REG_FIELD(0x19, 4, 4),
    [VOUT_TH_CHG_EN_MASK] = REG_FIELD(0x19, 3, 3),
    [VBUS_PRESENT_MASK] = REG_FIELD(0x19, 2, 2),
    /*reg1D*/
    [ADC_EN] = REG_FIELD(0x1D, 7, 7),
    [ADC_RATE] = REG_FIELD(0x1D, 6, 6),
};

static const int sc8541d_adc_m[] =
    {25, 375, 5, 125, 125, 3125, 44, 5};

static const int sc8541d_adc_l[] =
    {10, 100, 1, 100, 100, 1000, 100, 10};

static const int sc8541d_vac_ovp[] =
    {6500, 11000, 12000, 13000, 14000, 15000, 15000, 15000};

static const struct regmap_config sc8541d_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = SC8541D_REGMAX,
};


enum sc8541d_reg_range {
    SC8541D_VBAT_OVP,
    SC8541D_IBAT_OCP,
    SC8541D_VBUS_OVP,
    SC8541D_IBUS_OCP,
    SC8541D_VAC_OVP,
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

#define SC8541D_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
    .min = _min, \
    .max = _max, \
    .step = _step, \
    .offset = _offset, \
    .round_up = _ru, \
}

#define SC8541D_CHG_RANGE_T(_table, _ru) \
    { .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }


static const struct reg_range sc8541d_reg_range[] = {
    [SC8541D_VBAT_OVP]      = SC8541D_CHG_RANGE(3840, 5110, 10, 4040, false),
    [SC8541D_IBAT_OCP]      = SC8541D_CHG_RANGE(0, 12700, 100, 500, false),
    [SC8541D_VBUS_OVP]      = SC8541D_CHG_RANGE(9250, 11000, 250, 9250, false),
    [SC8541D_IBUS_OCP]      = SC8541D_CHG_RANGE(1000, 4750, 250, 0, false),
    [SC8541D_VAC_OVP]       = SC8541D_CHG_RANGE_T(sc8541d_vac_ovp, false),
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
    { .reg = 0x15, .len = 7, .bit = {
                {.mask = BIT(7), .name = "cp switching flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vbus errorhi flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vbus errorlo flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vout th rev en flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vout th chg en flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus insert flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "vout insert flag", .notify = SC8541D_NOTIFY_OTHER},
                },
    },
    { .reg = 0x16, .len = 7, .bit = {
                {.mask = BIT(7), .name = "adc done flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "pin diag fail flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ibus ucp rise flag", .notify = SC8541D_NOTIFY_IBUSUCPF},
                {.mask = BIT(4), .name = "tdie alm flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbat reg active flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "ibat reg active flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "ibus reg active flag", .notify = SC8541D_NOTIFY_OTHER},
                },
    },
    { .reg = 0x17, .len = 8, .bit = {
                {.mask = BIT(7), .name = "por flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "ss fail flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ibus ucp fall flag", .notify = SC8541D_NOTIFY_IBUSUCPF},
                {.mask = BIT(4), .name = "ibus ocp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "ibat ocp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus ovp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vout ovp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "vbat ovp flag", .notify = SC8541D_NOTIFY_OTHER},
                },
    },
    { .reg = 0x18, .len = 6, .bit = {
                {.mask = BIT(5), .name = "pmid2out uvp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "pmid2out ovp flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "tshut flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "them flt flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "ss timeout flag", .notify = SC8541D_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "wd timeout flag", .notify = SC8541D_NOTIFY_OTHER},
                },
    },
};

/********************COMMON API***********************/
__maybe_unused static u8 val2reg(enum sc8541d_reg_range id, u32 val)
{
    int i;
    u8 reg;
    const struct reg_range *range= &sc8541d_reg_range[id];

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

__maybe_unused static u32 reg2val(enum sc8541d_reg_range id, u8 reg)
{
    const struct reg_range *range= &sc8541d_reg_range[id];
    if (!range)
        return reg;
    return range->table ? range->table[reg] :
                  range->offset + range->step * reg;
}

static int sc8541d_init_device(struct sc8541d_chip *sc);
/*********************************************************/
static int sc8541d_field_read(struct sc8541d_chip *sc,
                enum sc8541d_fields field_id, unsigned int *val)
{
    int ret;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541d read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8541d_field_write(struct sc8541d_chip *sc,
                enum sc8541d_fields field_id, int val)
{
    int ret;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541d write field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8541d_read_bulk(struct sc8541d_chip *sc,
                uint8_t addr, uint8_t *val, uint8_t count)
{
    int ret;

    ret = regmap_bulk_read(sc->regmap, addr, val, count);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541d read %02x block failed %d\n", addr, ret);
    }

    return ret;
}

/*******************************************************/
static int sc8541d_reg_reset(struct sc8541d_chip *sc)
{
    return sc8541d_field_write(sc, REG_RST, 1);
}

int sc8541d_detect_device(struct sc8541d_chip *sc)
{
    int ret;
    int device_id;

    ret = sc8541d_field_read(sc, DEVICE_ID, &device_id);
    if (ret < 0) {
        dev_err(sc->dev, "get device id fail\n");
        return ret;
    }

    if (device_id != SC8541D_DEVICE_ID) {
        dev_err(sc->dev, "%s not find SC8541D, ID = 0x%02x\n", __func__, ret);
        return -EINVAL;
    }
    sc->device_id = device_id;

    ret = sc8541d_field_read(sc, VERSION_ID, &sc->version_id);

    dev_info(sc->dev, "%s Device id: 0x%02x, Version id: 0x%02x\n", __func__,
        sc->device_id, sc->version_id);

    return ret;
}

static int sc8541d_check_charge_enabled(struct sc8541d_chip *sc, int *enabled)
{
    return sc8541d_field_read(sc, CP_SWITCHING_STAT, enabled);
}

__maybe_unused static int sc8541d_set_busovp_th(struct sc8541d_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8541D_VBUS_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8541d_field_write(sc, VBUS_OVP, reg_val);
}

__maybe_unused static int sc8541d_set_busocp_th(struct sc8541d_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8541D_IBUS_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8541d_field_write(sc, IBUS_OCP, reg_val);
}

__maybe_unused static int sc8541d_set_batovp_th(struct sc8541d_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8541D_VBAT_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8541d_field_write(sc, VBAT_OVP, reg_val);
}

__maybe_unused static int sc8541d_set_batocp_th(struct sc8541d_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8541D_IBAT_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8541d_field_write(sc, IBAT_OCP, reg_val);
}

__maybe_unused static int sc8541d_is_vbuslowerr(struct sc8541d_chip *sc, bool *err)
{
    int ret;
    int val;

    ret = sc8541d_field_read(sc, VBUS2OUT_UVP_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc8541d_is_vbushigherr(struct sc8541d_chip *sc, bool *err)
{
    int ret;
    int val;

    ret = sc8541d_field_read(sc, VBUS2OUT_OVP_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

static int sc8541d_enable_adc(struct sc8541d_chip *sc, bool en)
{
    return sc8541d_field_write(sc, ADC_EN, !!en);
}

static int sc8541d_get_adc_data(struct sc8541d_chip *sc,
        int channel,  int *result)
{
    int ret;
    int reg = SC8541D_REG_1F + channel * 2;
    uint8_t reg_val[2];

    if (channel >= ADC_MAX_NUM)
        return -EINVAL;

    ret = sc8541d_read_bulk(sc, reg, reg_val, 2);

    if (ret < 0)
        return ret;

    *result = ((reg_val[0] << 8) | reg_val[1]) *
        sc8541d_adc_m[channel] / sc8541d_adc_l[channel];

    return ret;
}

static int sc8541d_dump_reg(struct sc8541d_chip *sc)
{
    int ret;
    int i;
    int val;

    for (i = 0; i <= SC8541D_REGMAX; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        dev_err(sc->dev, "%s reg[0x%02x] = 0x%02x\n",
                __func__, i, val);
    }

    return ret;
}

__maybe_unused static int sc8541d_enable_charge(struct sc8541d_chip *sc, bool en)
{
    int ret = 0;
    int vbus_value = 0, vout_value = 0, value = 0;
    int vbus_hi = 0, vbus_low = 0;
    dev_info(sc->dev,"%s:%d",__func__,en);

    if (!en) {
        ret |= sc8541d_field_write(sc, CP_EN, !!en);

        disable_irq(sc->irq);

        return ret;
    } else {
        ret = sc8541d_get_adc_data(sc, ADC_VBUS, &vbus_value);
        ret |= sc8541d_get_adc_data(sc, ADC_VOUT, &vout_value);
        dev_info(sc->dev,"%s: vbus/vout:%d / %d = %d \r\n", __func__, vbus_value, vout_value, vbus_value*100/vout_value);

        ret |= sc8541d_field_read(sc, MODE, &value);
        dev_info(sc->dev,"%s: mode:%d %s \r\n", __func__, value, (value == 0 ?"4:1":(value == 1 ?"2:1":"else")));

        ret |= sc8541d_field_read(sc, VBUS2OUT_UVP_STAT, &vbus_low);
        ret |= sc8541d_field_read(sc, VBUS2OUT_OVP_STAT, &vbus_hi);
        dev_info(sc->dev,"%s: high:%d  low:%d \r\n", __func__, vbus_hi, vbus_low);

        ret |= sc8541d_field_write(sc, CP_EN, !!en);

        //disable_irq(sc->irq);

        mdelay(300);

        ret |= sc8541d_field_read(sc, CP_SWITCHING_STAT, &value);
        if (!value) {
            dev_info(sc->dev,"%s:enable fail \r\n", __func__);
            sc8541d_dump_reg(sc);
        } else {
            dev_info(sc->dev,"%s:enable success \r\n", __func__);
        }

        enable_irq(sc->irq);
    }

    return ret;
}

/*********************mtk charger interface start**********************************/

static inline int to_sc8541d_adc(enum adc_channel chan)
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


static int mtk_sc8541d_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int ret;
    int val;

    ret = sc8541d_check_charge_enabled(sc, &val);

    *en = !!val;

    return ret;
}


static int mtk_sc8541d_enable_chg(struct charger_device *chg_dev, bool en)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8541d_enable_charge(sc,en);

    return ret;
}


static int mtk_sc8541d_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int mv;
    mv = uV / 1000;

    return sc8541d_set_busovp_th(sc, mv);
}

static int mtk_sc8541d_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int ma;
    ma = uA / 1000;

    return sc8541d_set_busocp_th(sc, ma);
}

static int mtk_sc8541d_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8541d_set_batovp_th(sc, uV/1000);

    return ret;
}

static int mtk_sc8541d_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8541d_set_batocp_th(sc, uA/1000);

    return ret;
}


static int mtk_sc8541d_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
            int *min, int *max)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);

    sc8541d_get_adc_data(sc, to_sc8541d_adc(chan), max);

    if(chan != ADC_CHANNEL_TEMP_JC)
        *max = *max * 1000;

    if (min != max)
        *min = *max;

    return 0;
}

static int mtk_sc8541d_get_adc_accuracy(struct charger_device *chg_dev,
                enum adc_channel chan, int *min, int *max)
{
    *min = *max = sc8541d_adc_accuracy_tbl[to_sc8541d_adc(chan)];
    return 0;
}

static int mtk_sc8541d_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);

    return sc8541d_is_vbuslowerr(sc,err);
}

static int mtk_sc8541d_is_vbushigherr(struct charger_device *chg_dev, bool *err)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);

    return sc8541d_is_vbushigherr(sc,err);
}

static int mtk_sc8541d_enable_adc(struct charger_device *chg_dev, bool en)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);

    return sc8541d_enable_adc(sc,en);
}

static int mtk_sc8541d_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8541d_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
    return 0;
}

static int mtk_sc8541d_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8541d_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
    return 0;
}

static int mtk_sc8541d_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret = 0;
	unsigned int val = 0;
	struct sc8541d_chip *sc = charger_get_data(chg_dev);

	if (IS_ERR_OR_NULL(sc)) {
		pr_info("%s sc is err or null.\n", __func__);
		return 0;
	}
	pr_info("%s typec_mos:%d, wls_mos:%d\n", __func__, typec_mos, wls_mos);

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
		ret = sc8541d_field_write(sc, OTG_EN, 0);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux close en otg fail ret=%d", __func__, ret);
			return ret;
		}
	}

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN
			&& typec_mos != MMI_DVCHG_MUX_DISABLE && wls_mos != MMI_DVCHG_MUX_DISABLE
			&& wls_mos != MMI_DVCHG_MUX_MANUAL_OPEN) {
		ret = sc8541d_field_write(sc, ACDRV_MANUAL_EN, 0);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux dis mos both fail ret=%d", __func__, ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
		if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
			ret = sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
			if (ret) {
				dev_err(sc->dev, "%s:mmi_mux set MANUAL_EN fail ret=%d", __func__, ret);
				return ret;
			}
		}
		ret = sc8541d_field_write(sc, ACDRV_EN, 0);
		if (ret) {
			dev_err(sc->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(100);
	}

	if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
		ret = sc8541d_field_write(sc, ACDRV_EN, 1);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
			return ret;
		}
	} else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
		ret = sc8541d_field_write(sc, OTG_EN, 1);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
			return ret;
		}

		ret = sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(100);
		ret = sc8541d_field_write(sc, ACDRV_EN, 1);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
			return ret;
		}
		ret = sc8541d_field_write(sc, ACDRV_EN, 0);
		if (ret) {
			dev_err(sc->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(1000);
	}

	ret = regmap_read(sc->regmap, SC8541D_CTRL5_REG, &val);
	dev_info(sc->dev, "%s:mmi_mux Reg[%02X] = 0x%02X, ret=%d\n",
			__func__, SC8541D_CTRL5_REG, val, ret);

	dev_info(sc->dev, "%s:ACDRV_MANUAL_EN:%d ACDRV_EN:%d OTG_EN:%d\n",
			__func__, (val & 0x40) > 0, (val & 0x20) > 0, (val & 0x10) > 0);

	return 0;
}

static int mtk_sc8541d_init_chip(struct charger_device *chg_dev)
{
    struct sc8541d_chip *sc = charger_get_data(chg_dev);

    sc8541d_init_device(sc);

    return sc8541d_enable_adc(sc,true);
}

static const struct charger_ops sc8541d_chg_ops = {
    .enable = mtk_sc8541d_enable_chg,
    .is_enabled = mtk_sc8541d_is_chg_enabled,
    .get_adc = mtk_sc8541d_get_adc,
    .get_adc_accuracy = mtk_sc8541d_get_adc_accuracy,
    .set_vbusovp = mtk_sc8541d_set_vbusovp,
    .set_ibusocp = mtk_sc8541d_set_ibusocp,
    .set_vbatovp = mtk_sc8541d_set_vbatovp,
    .set_ibatocp = mtk_sc8541d_set_ibatocp,
    .init_chip = mtk_sc8541d_init_chip,
    .is_vbuslowerr = mtk_sc8541d_is_vbuslowerr,
    .is_vbushigherr = mtk_sc8541d_is_vbushigherr,
    .set_vbatovp_alarm = mtk_sc8541d_set_vbatovp_alarm,
    .reset_vbatovp_alarm = mtk_sc8541d_reset_vbatovp_alarm,
    .set_vbusovp_alarm = mtk_sc8541d_set_vbusovp_alarm,
    .reset_vbusovp_alarm = mtk_sc8541d_reset_vbusovp_alarm,
    .enable_adc = mtk_sc8541d_enable_adc,
    .config_mux = mtk_sc8541d_config_mux,
};

static const struct charger_properties sc8541d_chg_props = {
    .alias_name = "sc8541d_chg",
};

__maybe_unused static int sc8541d_set_present(struct sc8541d_chip *sc, bool present)
{
    int ret = 0;

    sc->usb_present = present;
    if (present) {
        ret = sc8541d_init_device(sc);
    }
    return ret;
}

static enum power_supply_property sc8541d_charger_props[] = {
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
};

static int sc8541d_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8541d_chip *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8541d_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8541d_get_adc_data(sc, ADC_IBUS, &result);
        if (!ret)
            sc->ibus_curr = result;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = sc8541d_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = sc8541d_get_adc_data(sc, ADC_IBAT, &result);
        if (!ret)
            sc->ibat_curr = result;
        val->intval = sc->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8541d_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    default:
        return -EINVAL;

    }

    return 0;
}


static int sc8541d_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{

    switch (prop) {

    default:
        return -EINVAL;
    }

    return 0;
}


static int sc8541d_psy_register(struct sc8541d_chip *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc8541d_psy_name[sc->role];
    sc->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
    sc->psy_desc.properties = sc8541d_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8541d_charger_props);
    sc->psy_desc.get_property = sc8541d_charger_get_property;
    sc->psy_desc.set_property = sc8541d_charger_set_property;

    sc->sc_cp_psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->sc_cp_psy)) {
        dev_err(sc->dev, "%s failed to register psy\n", __func__);
        return PTR_ERR(sc->sc_cp_psy);
    }

    dev_info(sc->dev, "%s power supply register successfully\n", sc->psy_desc.name);
    return 0;
}

static inline int status_reg_to_charger(enum sc8541d_notify notify)
{
	switch (notify) {
	case SC8541D_NOTIFY_IBUSUCPF:
		return CHARGER_DEV_NOTIFY_IBUSUCP_FALL;
    case SC8541D_NOTIFY_VBUSOVPALM:
		return CHARGER_DEV_NOTIFY_VBUSOVP_ALARM;
    case SC8541D_NOTIFY_VBATOVPALM:
		return CHARGER_DEV_NOTIFY_VBATOVP_ALARM;
    case SC8541D_NOTIFY_IBUSOCP:
		return CHARGER_DEV_NOTIFY_IBUSOCP;
    case SC8541D_NOTIFY_VBUSOVP:
		return CHARGER_DEV_NOTIFY_VBUS_OVP;
    case SC8541D_NOTIFY_IBATOCP:
		return CHARGER_DEV_NOTIFY_IBATOCP;
    case SC8541D_NOTIFY_VBATOVP:
		return CHARGER_DEV_NOTIFY_BAT_OVP;
    case SC8541D_NOTIFY_VOUTOVP:
		return CHARGER_DEV_NOTIFY_VOUTOVP;
	default:
        return -EINVAL;
		break;
	}
	return -EINVAL;
}

__maybe_unused
static void sc8541d_dump_check_fault_status(struct sc8541d_chip *sc)
{
    u8 flag = 0;
    int i,j,k;
    int noti;
    int ret = 0;

    for (i = 0; i <= SC8541D_REGMAX; i++) {
        ret = sc8541d_read_bulk(sc, i, &flag, 1);
        dev_dbg(sc->dev, "%s reg[0x%02x] = 0x%02x, ret = %d\n", __func__, i, flag, ret);
        for (k=0; k < ARRAY_SIZE(cp_intr_flag); k++) {
            if (cp_intr_flag[k].reg == i){
                for (j=0; j <  cp_intr_flag[k].len; j++) {
                    if (flag & cp_intr_flag[k].bit[j].mask) {
                        dev_err(sc->dev,"trigger :%s\n",cp_intr_flag[k].bit[j].name);

                        noti = status_reg_to_charger(cp_intr_flag[k].bit[j].notify);
                        if(noti >= 0) {
                            charger_dev_notify(sc->chg_dev, noti);
                        }
                    }
                }
            }
        }
    }
}

static irqreturn_t sc8541d_irq_handler(int irq, void *dev_id)
{
    struct sc8541d_chip *sc = dev_id;

    dev_err(sc->dev, "%s irq trigger\n", __func__);

    sc8541d_dump_check_fault_status(sc);

    power_supply_changed(sc->sc_cp_psy);

    return IRQ_HANDLED;
}

/********************creat devices note start*************************************************/

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	int enable;
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = sc8541d_check_charge_enabled(sc, &enable);
	if (ret < 0) {
		pr_err("sc8541d: sc8541d_is_chg_en not valid\n");
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
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc8541d_enable_charge(sc, enable);
	if (ret) {
		pr_err("sc8541d Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("sc8541d  %s charging \n",
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}
	sc8541d_get_adc_data(sc, to_sc8541d_adc(ADC_CHANNEL_VBUS), &vbus);
	vbus *= 1000;

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", sc->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}

	regmap_read(sc->regmap, sc->reg_addr, &sc->reg_data);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", sc->reg_addr, sc->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_data = tmp;
	regmap_write(sc->regmap, sc->reg_addr, sc->reg_data);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t sc8541d_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8541d_chip *sc = dev_get_drvdata(dev);
    uint8_t addr;
    unsigned int val;
    uint8_t tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8541d");
    for (addr = 0x0; addr <= SC8541D_REGMAX; addr++) {
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

static ssize_t sc8541d_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8541d_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= SC8541D_REGMAX)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc8541d_show_registers, sc8541d_store_register);

static ssize_t show_typec_mos(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	if (!sc) {
		pr_err("sc8541d chip not valid\n");
		return -ENODEV;
	}
	if (sc->role == SC8541D_SLAVE) {
		dev_err(sc->dev, "%s sc8541d slave not support\n", __func__);
		return -ENODEV;
	}

	ret = regmap_read(sc->regmap, SC8541D_CTRL5_REG, &val);

	return sprintf(buf, "REG[%02X]=0x%02X ret=%d\n", SC8541D_CTRL5_REG, val, ret);
}

static ssize_t store_typec_mos(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	int ret = 0;
	struct sc8541d_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		dev_err(sc->dev, "%s sc8541d chip not valid\n", __func__);
		return -ENODEV;
	}

	if (sc->role == SC8541D_SLAVE) {
		dev_err(sc->dev, "%s sc8541d slave not support\n", __func__);
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	if (tmp == 0) {
		ret = sc8541d_field_write(sc, OTG_EN, 0);
		ret |= sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
		ret |= sc8541d_field_write(sc, ACDRV_EN, 0);
	} else if (tmp == 1) {
		ret = sc8541d_field_write(sc, OTG_EN, 0);
		ret |= sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
		ret |= sc8541d_field_write(sc, ACDRV_EN, 1);
	} else if (tmp == 2) {
		ret = sc8541d_field_write(sc, OTG_EN, 1);
		ret |= sc8541d_field_write(sc, ACDRV_MANUAL_EN, 1);
		udelay(100);
		ret |= sc8541d_field_write(sc, ACDRV_EN, 1);
	}

	if (ret) {
		dev_err(sc->dev, "%s fail ret=%d", __func__, ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(typec_mos, 0664, show_typec_mos, store_typec_mos);

static void sc8541d_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_vbus);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_registers);
    device_create_file(dev, &dev_attr_typec_mos);
}

/********************creat devices note end*************************************************/

static int sc8541d_parse_dt(struct sc8541d_chip *sc, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc8541d,vbat-ovp-dis", &(sc->cfg.vbat_ovp_dis)},
        {"sc,sc8541d,vbat-ovp", &(sc->cfg.vbat_ovp_th)},
        {"sc,sc8541d,ibat-ocp-dis", &(sc->cfg.ibat_ocp_dis)},
        {"sc,sc8541d,ibat-ocp", &(sc->cfg.ibat_ocp_th)},
        {"sc,sc8541d,vbus-ovp-dis", &(sc->cfg.vbus_ovp_dis)},
        {"sc,sc8541d,vbus-ovp", &(sc->cfg.vbus_ovp_th)},
        {"sc,sc8541d,vac-ovp-dis", &(sc->cfg.vac_ovp_dis)},
        {"sc,sc8541d,vac-ovp", &(sc->cfg.vac_ovp_th)},
        {"sc,sc8541d,ibus-ocp-dis", &(sc->cfg.ibus_ocp_dis)},
        {"sc,sc8541d,ibus-ocp", &(sc->cfg.ibus_ocp_th)},
        {"sc,sc8541d,pmid2out-ovp-dis", &(sc->cfg.pmid2out_ovp_dis)},
        {"sc,sc8541d,pmid2out-ovp", &(sc->cfg.pmid2out_ovp_th)},
        {"sc,sc8541d,pmid2out-uvp-dis", &(sc->cfg.pmid2out_uvp_dis)},
        {"sc,sc8541d,pmid2out-uvp", &(sc->cfg.pmid2out_uvp_th)},
        {"sc,sc8541d,ibatreg", &(sc->cfg.ibatreg_th)},
        {"sc,sc8541d,ibusreg", &(sc->cfg.ibusreg_th)},
        {"sc,sc8541d,vbatreg", &(sc->cfg.vbatreg_th)},
        {"sc,sc8541d,tdiereg", &(sc->cfg.tdiereg_th)},
        {"sc,sc8541d,them-flt", &(sc->cfg.them_flt_th)},
        {"sc,sc8541d,ibat-sns-res", &(sc->cfg.ibat_sns_res)},
        {"sc,sc8553,vac-pd-en", &(sc->cfg.vac_pd_en)},
        {"sc,sc8541d,pmid-pd-en", &(sc->cfg.pmid_pd_en)},
        {"sc,sc8541d,vbus-pd-en", &(sc->cfg.vbus_pd_en)},
        {"sc,sc8541d,work-mode", &(sc->cfg.work_mode)},
        {"sc,sc8541d,ss-timeout", &(sc->cfg.ss_timeout)},
        {"sc,sc8541d,wd-timeout", &(sc->cfg.wd_timeout)},
        {"sc,sc8541d,vout-ovp", &(sc->cfg.vout_ovp_th)},
        {"sc,sc8541d,fsw-freq", &(sc->cfg.fsw_freq)},
        {"sc,sc8541d,wd-timeout-dis", &(sc->cfg.wd_timeout_dis)},
        {"sc,sc8541d,pin-diag-fall-dis", &(sc->cfg.pin_diag_fall_dis)},
        {"sc,sc8541d,vout-ovp-dis", &(sc->cfg.vout_ovp_dis)},
        {"sc,sc8541d,them-flt-dis", &(sc->cfg.them_flt_dis)},
        {"sc,sc8541d,tshut-dis", &(sc->cfg.tshut_dis)},
        {"sc,sc8541d,ibus-ucp-dis", &(sc->cfg.ibus_ucp_dis)},
        {"sc,sc8541d,ibat-reg-dis", &(sc->cfg.ibat_reg_dis)},
        {"sc,sc8541d,ibus-reg-dis", &(sc->cfg.ibus_reg_dis)},
        {"sc,sc8541d,vbat-reg-dis", &(sc->cfg.vbat_reg_dis)},
        {"sc,sc8541d,tdie-reg-dis", &(sc->cfg.tdie_reg_dis)},
        {"sc,sc8541d,ibat-ocp-dg", &(sc->cfg.ibat_ocp_dg)},
        {"sc,sc8541d,ibus-ucp-fall-dg", &(sc->cfg.ibus_ucp_fall_dg)},
        {"sc,sc8541d,ibus-ocp-dg", &(sc->cfg.ibus_ocp_dg)},
        {"sc,sc8541d,vbat-ovp-dg", &(sc->cfg.ibus_ocp_dg)},
    };

    sc->irq_gpio = of_get_named_gpio(np, "sc8541d,intr_gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        dev_err(sc->dev, "no intr_gpio info\n");
        return -EINVAL;
    }

    if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        dev_err(sc->dev,"no charger name\n");
    }

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            continue;
        }
    }

    return ret;
}

static int sc8541d_init_device(struct sc8541d_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc8541d_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.vbat_ovp_dis},
        {VBAT_OVP, sc->cfg.vbat_ovp_th},
        {IBAT_OVP_DIS, sc->cfg.ibat_ocp_dis},
        {IBAT_OCP, sc->cfg.ibat_ocp_th},
        {VBUS_OVP_DIS, sc->cfg.vbus_ovp_dis},
        {VBUS_OVP, sc->cfg.vbus_ovp_th},
        {VAC_OVP_DIS, sc->cfg.vac_ovp_dis},
        {VAC_OVP, sc->cfg.vac_ovp_th},
        {IBUS_OCP_DIS, sc->cfg.ibus_ocp_dis},
        {IBUS_OCP, sc->cfg.ibus_ocp_th},
        {PMID2OUT_OVP_DIS, sc->cfg.pmid2out_ovp_dis},
        {PMID2OUT_OVP, sc->cfg.pmid2out_ovp_th},
        {PMID2OUT_UVP_DIS, sc->cfg.pmid2out_uvp_dis},
        {PMID2OUT_UVP, sc->cfg.pmid2out_uvp_th},
        {SET_IBATREG, sc->cfg.ibatreg_th},
        {SET_IBUSREG, sc->cfg.ibusreg_th},
        {SET_VBATREG, sc->cfg.vbatreg_th},
        {SET_TDIEREG, sc->cfg.tdiereg_th},
        {THEM_FLT, sc->cfg.them_flt_th},
        {SET_IBAT_SNS_RES, sc->cfg.ibat_sns_res},
        {VAC_PD_EN, sc->cfg.vac_pd_en},
        {PMID_PD_EN, sc->cfg.pmid_pd_en},
        {VBUS_PD_EN, sc->cfg.vbus_pd_en},
        {MODE, sc->cfg.work_mode},
        {SS_TIMEOUT, sc->cfg.ss_timeout},
        {WD_TIMEOUT, sc->cfg.wd_timeout},
        {VOUT_OVP, sc->cfg.vout_ovp_th},
        {FSW_SET, sc->cfg.fsw_freq},
        {WD_TIMEOUT_DIS, sc->cfg.wd_timeout_dis},
        {PIN_DIAG_FALL_DIS, sc->cfg.pin_diag_fall_dis},
        {VOUT_OVP_DIS, sc->cfg.vout_ovp_dis},
        {THEM_FLT_DIS, sc->cfg.them_flt_dis},
        {TSHUT_DIS, sc->cfg.tshut_dis},
        {IBUS_UCP_DIS, sc->cfg.ibus_ucp_dis},
        {IBAT_REG_DIS, sc->cfg.ibat_reg_dis},
        {IBUS_REG_DIS, sc->cfg.ibus_reg_dis},
        {VBAT_REG_DIS, sc->cfg.vbat_reg_dis},
        {TDIE_REG_DIS, sc->cfg.tdie_reg_dis},
        {IBAT_OCP_DG_SET, sc->cfg.ibat_ocp_dg},
        {IBUS_UCP_FALL_DG_SET, sc->cfg.ibus_ucp_fall_dg},
        {IBUS_OCP_DG_SET, sc->cfg.ibus_ocp_dg},
        {VBAT_OVP_DG_SET, sc->cfg.ibus_ocp_dg},
        {VBUS_PRESENT_MASK, 1},
        {VOUT_TH_CHG_EN_MASK, 1},
        {VOUT_TH_REV_EN_MASK, 1},
    };

    ret = sc8541d_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc8541d_field_write(sc, props[i].field_id, props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "%s Failed to init index %d registers(%d)\n", __func__, i, ret);
        }
    }

    return sc8541d_dump_reg(sc);
}

static int sc8541d_irq_register(struct sc8541d_chip *sc)
{
    int ret = 0;

    ret = devm_gpio_request(sc->dev, sc->irq_gpio, "sc8541d,intr_gpio");
    if (ret < 0) {
        dev_err(sc->dev, "failed to request GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }

    ret = gpio_direction_input(sc->irq_gpio);
    if (ret < 0) {
        dev_err(sc->dev, "failed to set GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }

    sc->irq = gpio_to_irq(sc->irq_gpio);
    if (ret < 0) {
        dev_err(sc->dev, "failed gpio to irq GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }

    ret = devm_request_threaded_irq(sc->dev, sc->irq, NULL,
                    sc8541d_irq_handler,
                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                    sc8541d_irq_name[sc->role], sc);
    if (ret < 0) {
        dev_err(sc->dev, "request thread irq failed:%d\n", ret);
        return ret;
    }

    //enable_irq_wake(sc->irq);
    disable_irq(sc->irq);
    return ret;
}

static struct of_device_id sc8541d_charger_match_table[] = {
    {   .compatible = "southchip,sc8541d-standalone",
        .data = &sc8541d_role_data[SC8541D_STANDALONE], },
    {   .compatible = "southchip,sc8541d-master",
        .data = &sc8541d_role_data[SC8541D_MASTER], },
    {   .compatible = "southchip,sc8541d-slave",
        .data = &sc8541d_role_data[SC8541D_SLAVE], },
    {},
};

static int sc8541d_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc8541d_chip *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret = 0;
    int i;

    pr_err("%s (%s)\n", __func__, SC8541D_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc8541d_chip), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->dev = &client->dev;
    sc->client = client;

    sc->regmap = devm_regmap_init_i2c(client,
                            &sc8541d_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        return -EINVAL;
    }

    for (i = 0; i < ARRAY_SIZE(sc8541d_reg_fields); i++) {
        const struct reg_field *reg_fields = sc8541d_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            return PTR_ERR(sc->rmap_fields[i]);
        }
    }

    ret = sc8541d_detect_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s detect device fail\n", __func__);
        goto err_detect_device;
    }

    i2c_set_clientdata(client, sc);
    sc8541d_create_device_node(&(client->dev));

    match = of_match_node(sc8541d_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        goto err_get_match;
    }

    sc->role = *(int *)match->data;

    ret = sc8541d_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc8541d_init_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = sc8541d_psy_register(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s psy register failed(%d)\n", __func__, ret);
        goto err_psy_register;
    }

    ret = sc8541d_irq_register(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

    sc->chg_dev = charger_device_register(sc->chg_dev_name,
					      &client->dev, sc,
					      &sc8541d_chg_ops,
					      &sc8541d_chg_props);
    if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_mtk_charger;
    }

    dev_err(sc->dev, "sc8541d[%s] probe successfully!\n",
            sc->role == SC8541D_STANDALONE ? "standalone" :
            (sc->role == SC8541D_MASTER ? "master" : "slave"));
    return 0;

err_register_mtk_charger:

err_register_irq:
err_psy_register:
    power_supply_unregister(sc->sc_cp_psy);
err_init_device:
err_parse_dt:
err_get_match:
err_detect_device:
    dev_err(sc->dev, "sc8541d probe failed!\n");
    devm_kfree(sc->dev, sc);
    return ret;
}


static void sc8541d_charger_remove(struct i2c_client *client)
{
    struct sc8541d_chip *sc = i2c_get_clientdata(client);

    sc8541d_enable_adc(sc, false);
    power_supply_unregister(sc->sc_cp_psy);

}

static void sc8541d_charger_shutdown(struct i2c_client *client)
{
	struct sc8541d_chip *sc = i2c_get_clientdata(client);

	/*reg reset*/
	sc8541d_reg_reset(sc);

	dev_info(sc->dev,"Shutdown Successfully\n");
}

#ifdef CONFIG_PM_SLEEP
static int sc8541d_suspend(struct device *dev)
{
    struct sc8541d_chip *sc = dev_get_drvdata(dev);

    dev_info(sc->dev, "Suspend successfully!");
//    if (device_may_wakeup(dev))
//        enable_irq_wake(sc->irq);
//    disable_irq(sc->irq);

    return 0;
}

static int sc8541d_resume(struct device *dev)
{
    struct sc8541d_chip *sc = dev_get_drvdata(dev);

    dev_info(sc->dev, "Resume successfully!");
//    if (device_may_wakeup(dev))
//        disable_irq_wake(sc->irq);
//    enable_irq(sc->irq);

    return 0;
}

static const struct dev_pm_ops sc8541d_pm_ops = {
    .resume		= sc8541d_resume,
    .suspend	= sc8541d_suspend,
};
#endif /*CONFIG_PM_SLEEP*/
static struct i2c_driver sc8541d_charger_driver = {
    .driver     = {
        .name   = "sc8541d-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc8541d_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc8541d_pm_ops,
#endif
    },
    .probe      = sc8541d_charger_probe,
    .remove     = sc8541d_charger_remove,
    .shutdown	= sc8541d_charger_shutdown,
};

module_i2c_driver(sc8541d_charger_driver);

MODULE_DESCRIPTION("SC SC8541D Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
