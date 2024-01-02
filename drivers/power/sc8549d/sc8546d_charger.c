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


#define SC8546D_DRV_VERSION              "1.0.0_G"

enum {
    SC8546D_ID,
};

static int id_data[] = {
    [SC8546D_ID] = 0x09,
};

enum {
    SC854X_STANDALONG = 0,
    SC854X_MASTER,
    SC854X_SLAVE,
};

static const char* sc8546d_psy_name[] = {
    [SC854X_STANDALONG] = "cp-standalone",
    [SC854X_MASTER] = "cp-master",
    [SC854X_SLAVE] = "cp-slave",
};

static const char* sc8546d_irq_name[] = {
    [SC854X_STANDALONG] = "sc8546d-standalone-irq",
    [SC854X_MASTER] = "sc8546d-master-irq",
    [SC854X_SLAVE] = "sc8546d-slave-irq",
};

static int sc8546d_mode_data[] = {
    [SC854X_STANDALONG] = SC854X_STANDALONG,
    [SC854X_MASTER]     = SC854X_MASTER,
    [SC854X_SLAVE]      = SC854X_SLAVE,
};

typedef enum{
    STANDALONE = 0,
    MASTER,
    SLAVE,
}WORK_MODE;

typedef enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VAC,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_TDIE,
    ADC_MAX_NUM,
}SC_854X_ADC_CH;

static const u32 sc8546d_adc_accuracy_tbl[ADC_MAX_NUM] = {
    150000, /* IBUS */
    35000,  /* VBUS */
    35000,  /* VAC  */
    20000,  /* VOUT */
    20000,  /* VBAT */
    200000, /* IBAT */
    4,      /* TDIE */
};

static const int sc8546d_adc_m[] =
    {1875, 375, 5, 125, 125, 3125, 5};

static const int sc8546d_adc_l[] =
    {1000, 100, 1, 100, 100, 1000, 10};

enum sc8546d_notify {
    SC854X_NOTIFY_OTHER = 0,
    SC854X_NOTIFY_IBUSUCPF,
    SC854X_NOTIFY_VBUSOVPALM,
    SC854X_NOTIFY_VBATOVPALM,
    SC854X_NOTIFY_IBUSOCP,
    SC854X_NOTIFY_VBUSOVP,
    SC854X_NOTIFY_IBATOCP,
    SC854X_NOTIFY_VBATOVP,
    SC854X_NOTIFY_VOUTOVP,
    SC854X_NOTIFY_VDROVP,
};

enum sc8546d_error_stata {
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
    { .reg = 0x02, .len = 1, .bit = {
                {.mask = BIT(4), .name = "vac ovp flag", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x03, .len = 1, .bit = {
                {.mask = 0x08, .name = "vdrop ovp flag", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x06, .len = 4, .bit = {
                {.mask = 0x01, .name = "pin diag fail", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x02, .name = "reg timeout", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x08, .name = "ss timeout", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x80, .name = "tshut flag", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x09, .len = 3, .bit = {
                {.mask = 0x08, .name = "wd timeout flag", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x20, .name = "ibus ucp rise", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x40, .name = "por flag", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x0a, .len = 1, .bit = {
                {.mask = 0x08, .name = "vbatreg active", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x0b, .len = 1, .bit = {
                {.mask = 0x08, .name = "ibatreg active", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x0c, .len = 1, .bit = {
                {.mask = 0x04, .name = "vbus uvlo fall", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x0d, .len = 3, .bit = {
                {.mask = 0x04, .name = "pmid2out ovp", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x08, .name = "pmid2out uvp", .notify = SC854X_NOTIFY_OTHER}},
    },
    { .reg = 0x0f, .len = 8, .bit = {
                {.mask = 0x01, .name = "vbat in flag", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x02, .name = "adapt in flag", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x04, .name = "ibus ucp flag", .notify = SC854X_NOTIFY_OTHER},
                {.mask = 0x08, .name = "ibus ocp flag", .notify = SC854X_NOTIFY_IBUSOCP},
                {.mask = 0x10, .name = "vbus ovp flag", .notify = SC854X_NOTIFY_VBUSOVP},
                {.mask = 0x20, .name = "ibat ocp flag", .notify = SC854X_NOTIFY_IBATOCP},
                {.mask = 0x40, .name = "vbat ovp flag", .notify = SC854X_NOTIFY_VBATOVP},
                {.mask = 0x80, .name = "vout ovp flag", .notify = SC854X_NOTIFY_VOUTOVP}},
    },
};



/************************************************************************/
#define SC8546D_REGMAX                      0xE9
#define SC854X_REG_13                       0x13

#define SC854X_VBUS_OVP_MIN                 6000
#define SC854X_VBUS_OVP_MAX                 12350
#define SC854X_VBUS_OVP_LSB                 50

#define SC854X_IBUS_OCP_MIN                 1200
#define SC854X_IBUS_OCP_MAX                 5700
#define SC854X_IBUS_OCP_LSB                 300

#define SC854X_BAT_OVP_MIN                  3500
#define SC854X_BAT_OVP_MAX                  5075
#define SC854X_BAT_OVP_LSB                  25

#define SC854X_BAT_OCP_MIN                  2000
#define SC854X_BAT_OCP_MAX                  8300
#define SC854X_BAT_OCP_LSB                  100

#define SC854X_AC_OVP_MIN                   11000
#define SC854X_AC_OVP_MAX                   17000
#define SC854X_AC_OVP_LSB                   1000
#define SC854X_AC_OVP_6P5V                  6500

enum sc8546d_reg_range {
    SC854X_VBAT_OVP,
    SC854X_IBAT_OCP,
    SC854X_VBUS_OVP,
    SC854X_IBUS_OCP,
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

#define SC854X_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
    .min = _min, \
    .max = _max, \
    .step = _step, \
    .offset = _offset, \
    .round_up = _ru, \
}

#define SC854X_CHG_RANGE_T(_table, _ru) \
    { .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }


static const struct reg_range sc8546d_reg_range[] = {
    [SC854X_VBAT_OVP] = SC854X_CHG_RANGE(3500, 5075, 25, 3500, true),
    [SC854X_IBAT_OCP] = SC854X_CHG_RANGE(2000, 8300, 100, 2000, true),
    [SC854X_VBUS_OVP] = SC854X_CHG_RANGE(6000, 12350, 50, 6000, true),
    [SC854X_IBUS_OCP] = SC854X_CHG_RANGE(1200, 5700, 300, 1200, true),
};

enum sc8546d_fields {
    VBAT_OVP_DIS, VBAT_OVP,
    IBAT_OCP_DIS, IBAT_OCP,
    VAC_OVP_DIS, OVPGATE_DIS, VAC_OVP_STAT,  VAC_OVP_FLAG, VAC_OVP_MASK, VAC_OVP,
    VERSION_ID1, DEVICE_ID1,
    VBUS_OVP_DIS, VBUS_OVP,
    IBUS_UCP_DIS, IBUS_OCP_DIS, IBUS_UCP_FALL_DEGLITCH_SET, IBUS_OCP,
    TSHUT_FLAG, TSHUT_STAT, VBUS_ERRORLO_STAT, VBUS_ERRORHI_STAT, SS_TIMEOUT_FLAG,
    CP_SWITCHING_STAT, VBUS_TH_CHG_EN_STAT, PIN_DIAG_FAIL_FLAG,
    CHG_EN, REG_RST, FREQ_SHIFT, FSW_SET,
    SS_TIMEOUT, FREQ_200K, VOUT_OVP_DIS, SET_IBAT_SNS_RES, VBUS_PD_EN, VAC_PD_EN,
    MODE, POR_FLAG, IBUS_UCP_RISE_FLAG, IBUS_UCP_RISE_MASK, WD_TIMEOUT_FLAG, WD_TIMEOUT,
    MANUAL_EN, OVPGATE_EN, REV_OVPGATE_EN, VBUS_UVLO_FALL_MASK, VBUS_UVLO_FALL_FLAG, FORVE_VAC_OK, REG_EN_IBATSNS,
    PMID2OUT_UVP, PMID2OUT_OVP, PMID2OUT_UVP_FLAG, PMID2OUT_OVP_FLAG, PMID2OUT_UVP_STAT, PMID2OUT_OVP_STAT,
    ADAPTER_INSERT_STATE, ADC_EN, ADC_RATE, ADC_FREEZE, ADC_DONE_STAT, ADC_DONE_FLAG, ADC_DONE_MASK,
    DM_ADC_EN, VBUS_ADC_DIS, VAC_ADC_DIS, VOUT_ADC_DIS, VBAT_ADC_DIS, IBAT_ADC_DIS, IBUS_ADC_DIS, TDIE_ADC_DIS,
    VERSION_ID2, DEVICE_ID2,
    PMID2OUT_OVP_BLK, PMID2OUT_UVP_BLK, I2C_VIL0P65_EN,
    OTG_EN,
    VBUS_IN_RANGE_DIS,
    F_MAX_FIELDS,
};

//REGISTER
static const struct reg_field sc8546d_reg_fields[] = {
    /*reg00*/
    [VBAT_OVP_DIS] = REG_FIELD(0x00, 7, 7),
    [VBAT_OVP] = REG_FIELD(0x00, 0, 6),
    /*reg01*/
    [IBAT_OCP_DIS] = REG_FIELD(0x01, 7, 7),
    [IBAT_OCP] = REG_FIELD(0x01, 0, 6),
    /*reg02*/
    [VAC_OVP_DIS] = REG_FIELD(0x02, 7, 7),
    [OVPGATE_DIS] = REG_FIELD(0x02, 6, 6),
    [VAC_OVP_STAT] = REG_FIELD(0x02, 5, 5),
    [VAC_OVP_FLAG] = REG_FIELD(0x02, 4, 4),
    [VAC_OVP_MASK] = REG_FIELD(0x02, 3, 3),
    [VAC_OVP] = REG_FIELD(0x02, 0, 2),
    /*reg03*/
    [VERSION_ID1] = REG_FIELD(0x03, 4, 7),
    [DEVICE_ID1] = REG_FIELD(0x03, 0, 3),
    /*reg04*/
    [VBUS_OVP_DIS] = REG_FIELD(0x04, 7, 7),
    [VBUS_OVP] = REG_FIELD(0x04, 0, 6),
    /*reg05*/
    [IBUS_UCP_DIS] = REG_FIELD(0x05, 7, 7),
    [IBUS_OCP_DIS] = REG_FIELD(0x05, 6, 6),
    [IBUS_UCP_FALL_DEGLITCH_SET] = REG_FIELD(0x05, 4, 5),
    [IBUS_OCP] = REG_FIELD(0x05, 0, 3),
    /*reg06*/
    [TSHUT_FLAG] = REG_FIELD(0x06, 7, 7),
    [TSHUT_STAT] = REG_FIELD(0x06, 6, 6),
    [VBUS_ERRORLO_STAT] = REG_FIELD(0x06, 5, 5),
    [VBUS_ERRORHI_STAT] = REG_FIELD(0x06, 4, 4),
    [SS_TIMEOUT_FLAG] = REG_FIELD(0x06, 3, 3),
    [CP_SWITCHING_STAT] = REG_FIELD(0x06, 2, 2),
    [VBUS_TH_CHG_EN_STAT] = REG_FIELD(0x06, 1, 1),
    [PIN_DIAG_FAIL_FLAG] = REG_FIELD(0x06, 0, 0),
    /*reg07*/
    [CHG_EN] = REG_FIELD(0x07, 7, 7),
    [REG_RST] = REG_FIELD(0x07, 6, 6),
    [FREQ_SHIFT] = REG_FIELD(0x07, 4, 5),
    [FSW_SET] = REG_FIELD(0x07, 0, 3),
    /*reg08*/
    [SS_TIMEOUT] = REG_FIELD(0x08, 5, 7),
    [FREQ_200K] = REG_FIELD(0x08, 4, 4),
    [VOUT_OVP_DIS] = REG_FIELD(0x08, 3, 3),
    [SET_IBAT_SNS_RES] = REG_FIELD(0x08, 2, 2),
    [VBUS_PD_EN] = REG_FIELD(0x08, 1, 1),
    [VAC_PD_EN] = REG_FIELD(0x08, 0, 0),
    /*reg09*/
    [MODE] = REG_FIELD(0x09, 7, 7),
    [POR_FLAG] = REG_FIELD(0x09, 6, 6),
    [IBUS_UCP_RISE_FLAG] = REG_FIELD(0x09, 5, 5),
    [IBUS_UCP_RISE_MASK] = REG_FIELD(0x09, 4, 4),
    [WD_TIMEOUT_FLAG] = REG_FIELD(0x09, 3, 3),
    [WD_TIMEOUT] = REG_FIELD(0x09, 0, 2),
    /*reg0C*/
    [MANUAL_EN] = REG_FIELD(0x0C, 7, 7),
    [OVPGATE_EN] = REG_FIELD(0x0C, 6, 6),
    [REV_OVPGATE_EN] = REG_FIELD(0x0C, 5, 5),
    [VBUS_UVLO_FALL_MASK] = REG_FIELD(0x0C, 3, 3),
    [VBUS_UVLO_FALL_FLAG] = REG_FIELD(0x0C, 2, 2),
    [FORVE_VAC_OK] = REG_FIELD(0x0C, 1, 1),
    [REG_EN_IBATSNS] = REG_FIELD(0x0C, 0, 0),
    /*reg0D*/
    [PMID2OUT_UVP] = REG_FIELD(0x0D, 6, 7),
    [PMID2OUT_OVP] = REG_FIELD(0x0D, 4, 5),
    [PMID2OUT_UVP_FLAG] = REG_FIELD(0x0D, 3, 3),
    [PMID2OUT_OVP_FLAG] = REG_FIELD(0x0D, 2, 2),
    [PMID2OUT_UVP_STAT] = REG_FIELD(0x0D, 1, 1),
    [PMID2OUT_OVP_STAT] = REG_FIELD(0x0D, 0, 0),
    /*reg0E*/
    [ADAPTER_INSERT_STATE] = REG_FIELD(0x0E, 1, 1),
    /*reg11*/
    [ADC_EN] = REG_FIELD(0x11, 7, 7),
    [ADC_RATE] = REG_FIELD(0x11, 6, 6),
    [ADC_FREEZE] = REG_FIELD(0x11, 5, 5 ),
    [ADC_DONE_STAT] = REG_FIELD(0x11, 2, 2),
    [ADC_DONE_FLAG] = REG_FIELD(0x11, 1, 1),
    [ADC_DONE_MASK] = REG_FIELD(0x11, 0, 0),
    /*reg12*/
    [DM_ADC_EN] = REG_FIELD(0x12, 7, 7),
    [VBUS_ADC_DIS] = REG_FIELD(0x12, 6, 6),
    [VAC_ADC_DIS] = REG_FIELD(0x12, 5, 5),
    [VOUT_ADC_DIS] = REG_FIELD(0x12, 4, 4),
    [VBAT_ADC_DIS] = REG_FIELD(0x12, 3, 3),
    [IBAT_ADC_DIS] = REG_FIELD(0x12, 2, 2),
    [IBUS_ADC_DIS] = REG_FIELD(0x12, 1, 1),
    [TDIE_ADC_DIS] = REG_FIELD(0x12, 0, 0),
    /*reg36*/
    [VERSION_ID2] = REG_FIELD(0x36, 4, 7),
    [DEVICE_ID2] = REG_FIELD(0x36, 0, 3),
    /*reg3a*/
    [PMID2OUT_OVP_BLK] = REG_FIELD(0x3A, 6, 7),
    [PMID2OUT_UVP_BLK] = REG_FIELD(0x3A, 4, 5),
    [I2C_VIL0P65_EN] = REG_FIELD(0x3A, 3, 3),
    /*reg3b*/
    [OTG_EN] = REG_FIELD(0x3B, 2, 2),
    /*reg3c*/
    [VBUS_IN_RANGE_DIS] = REG_FIELD(0x3C, 6, 6),

};

static const struct regmap_config sc8546d_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = SC8546D_REGMAX,
};

/************************************************************************/


struct sc8546d_cfg {
    int adc_rate;

    int bat_ovp_disable;
    int bat_ocp_disable;
    int bus_ovp_disable;
    int bus_ucp_disable;
    int bus_ocp_disable;

    int bat_ovp_th;
    int bat_ocp_th;
    int ac_ovp_th;
    int bus_ovp_th;
    int bus_ocp_th;

    int sense_r_mohm;

    int ibat_reg_th;
    int vbat_reg_th;
};

struct sc8546d {
    struct device *dev;
    struct device *sys_dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];
    const char *chg_dev_name;

    int reg_addr;
    int reg_data;

    int mode;

    int irq_gpio;
    int irq;

    struct mutex data_lock;
    struct mutex irq_complete;

    bool irq_waiting;
    bool irq_disabled;
    bool resume_completed;

    bool usb_present;
    bool charge_enabled;    /* Register bit status */

    int vbus_error;

    /* ADC reading */
    int vbat_volt;
    int vbus_volt;
    int vout_volt;
    int vac_volt;

    int ibat_curr;
    int ibus_curr;

    int die_temp;

    struct sc8546d_cfg cfg;

    int skip_writes;
    int skip_reads;

    struct sc8546d_platform_data *platform_data;

    struct charger_device *chg_dev;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *fc2_psy;
};

static const struct charger_properties sc8546d_chg_props = {
    .alias_name = "sc8546d_chg",
};

/********************COMMON API***********************/
__maybe_unused static u8 val2reg(enum sc8546d_reg_range id, u32 val)
{
    int i;
    u8 reg;
    const struct reg_range *range= &sc8546d_reg_range[id];

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

__maybe_unused static u32 reg2val(enum sc8546d_reg_range id, u8 reg)
{
    const struct reg_range *range= &sc8546d_reg_range[id];
    if (!range)
        return reg;
    return range->table ? range->table[reg] :
                range->offset + range->step * reg;
}
/*********************************************************/
static int sc8546d_field_read(struct sc8546d *sc,
                enum sc8546d_fields field_id, int *val)
{
    int ret;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev,"sc8546d read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8546d_field_write(struct sc8546d *sc,
                enum sc8546d_fields field_id, int val)
{
    int ret;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev,"sc8546d read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8546d_read_block(struct sc8546d *sc,
                int reg, uint8_t *val, int len)
{
    int ret;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev,"sc8546d read %02x block failed %d\n", reg, ret);
    }

    return ret;
}

static int sc8546d_write_byte(struct sc8546d *sc, int reg, uint8_t val)
{
    int ret;

    ret = regmap_write(sc->regmap, reg, val);
    if (ret < 0) {
        dev_err(sc->dev,"sc8546d write %02x failed %d\n", reg, ret);
    }

    return ret;
}

/***********************charger interface start********************************/
__maybe_unused static int sc8546d_detect_device(struct sc8546d *sc)
{
    int ret;
    int data;

    ret = sc8546d_field_read(sc, DEVICE_ID2, &data);

    if (ret < 0) {
        dev_err(sc->dev, "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    dev_info(sc->dev,"%s id 0x%2x",__func__,data);

    if (ret == 0) {
        if(data != id_data[SC8546D_ID]) {
            return -ENOMEM;
        }
    }

    return ret;
}

__maybe_unused static int sc8546d_reg_reset(struct sc8546d *sc)
{
    return sc8546d_field_write(sc, REG_RST, 1);
}

__maybe_unused static int sc8546d_dump_reg(struct sc8546d *sc)
{
    int ret;
    int i;
    int val;

    for (i = 0; i <= 0x20; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        dev_err(sc->dev,"%s reg[0x%02x] = 0x%02x\n",
                __func__, i, val);
    }

    return ret;
}

__maybe_unused static int sc8546d_enable_charge(struct sc8546d *sc, bool enable)
{
    dev_info(sc->dev,"%s:%d",__func__,enable);
    return sc8546d_field_write(sc, CHG_EN, !!enable);
}

__maybe_unused static int sc8546d_check_adapter_inserted(struct sc8546d *sc, int *result)
{
    int ret;
    int val;

    ret = sc8546d_field_read(sc, ADAPTER_INSERT_STATE, &val);

    *result = !!val;

    dev_info(sc->dev,"%s:%d",__func__,val);

    return ret;
}

__maybe_unused static int sc8546d_check_charge_enabled(struct sc8546d *sc, bool *enabled)
{
    int ret;
    int val;

    ret = sc8546d_field_read(sc, CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    dev_info(sc->dev,"%s:%d",__func__,val);

    return ret;
}

__maybe_unused static int sc8546d_get_status(struct sc8546d *sc, uint32_t *status)
{
    int ret, val;
    *status = 0;

    sc8546d_dump_reg(sc);

    ret = sc8546d_field_read(sc, VBUS_ERRORHI_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    ret = sc8546d_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORLO_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_LOW);

    dev_info(sc->dev,"%s (%d)\n", __func__, *status);

    return ret;

}

__maybe_unused static int sc8546d_enable_adc(struct sc8546d *sc, bool enable)
{
    dev_info(sc->dev,"%s:%d",__func__,enable);
    return sc8546d_field_write(sc, ADC_EN, !!enable);
}

__maybe_unused static int sc8546d_set_adc_scanrate(struct sc8546d *sc, bool oneshot)
{
    dev_info(sc->dev,"%s:%d",__func__,oneshot);
    return sc8546d_field_write(sc, ADC_RATE, !!oneshot);
}

__maybe_unused static int sc8546d_set_adc_freeze(struct sc8546d *sc, bool enable)
{
    return sc8546d_field_write(sc, ADC_FREEZE, !!enable);
}

__maybe_unused static int sc8546d_get_adc_data(struct sc8546d *sc, int channel,  int *result)
{
    int ret;
    uint8_t val[2] = {0};

    if(channel >= ADC_MAX_NUM)
        return -EINVAL;

    ret = sc8546d_read_block(sc, SC854X_REG_13 + (channel << 1), val, 2);
    if (ret < 0)
        return ret;

    *result = (val[1] | (val[0] << 8)) *
                sc8546d_adc_m[channel] / sc8546d_adc_l[channel];

    dev_info(sc->dev,"%s %d", __func__, *result);

    return ret;
}

__maybe_unused static int sc8546d_set_busovp_th(struct sc8546d *sc, int threshold)
{
    int reg_val = val2reg(SC854X_VBUS_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8546d_field_write(sc, VBUS_OVP, reg_val);
}

__maybe_unused static int sc8546d_set_busocp_th(struct sc8546d *sc, int threshold)
{
    int reg_val = val2reg(SC854X_IBUS_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8546d_field_write(sc, IBUS_OCP, reg_val);
}

__maybe_unused static int sc8546d_set_batovp_th(struct sc8546d *sc, int threshold)
{
    int reg_val = val2reg(SC854X_VBAT_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8546d_field_write(sc, VBAT_OVP, reg_val);
}

__maybe_unused static int sc8546d_set_batocp_th(struct sc8546d *sc, int threshold)
{
    int reg_val = val2reg(SC854X_IBAT_OCP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8546d_field_write(sc, IBAT_OCP, reg_val);
}

__maybe_unused static int sc8546d_set_acovp_th(struct sc8546d *sc, int threshold)
{
    dev_info(sc->dev,"%s:%d",__func__,threshold);
    if (threshold == SC854X_AC_OVP_6P5V) {
        threshold = 0x07;
        return sc8546d_field_write(sc, VAC_OVP, threshold);
    } else if (threshold < SC854X_AC_OVP_MIN)
        threshold = SC854X_AC_OVP_MIN;

    if (threshold > SC854X_AC_OVP_MAX)
        threshold = SC854X_AC_OVP_MAX;

    threshold = (threshold - SC854X_AC_OVP_MIN) / SC854X_AC_OVP_LSB;

    return sc8546d_field_write(sc, VAC_OVP, threshold);
}

__maybe_unused static int sc8546d_disable_vbus_range(struct sc8546d *sc)
{
    return sc8546d_field_write(sc, VBUS_IN_RANGE_DIS, 1);
}

__maybe_unused static int sc8546d_is_vbuslowerr(struct sc8546d *sc, bool *err)
{
    int ret;
    int val;

    ret = sc8546d_field_read(sc, VBUS_ERRORLO_STAT, &val);

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc8546d_init_device(struct sc8546d *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc8546d_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.bat_ovp_disable},
        {IBAT_OCP_DIS, sc->cfg.bat_ocp_disable},
        {IBUS_UCP_DIS, sc->cfg.bus_ucp_disable},
        {IBUS_OCP_DIS, sc->cfg.bus_ocp_disable},
        {VBUS_OVP_DIS, sc->cfg.bus_ovp_disable},

        {IBAT_OCP, sc->cfg.bat_ocp_th},
        {VBAT_OVP, sc->cfg.bat_ovp_th},
        {IBUS_OCP, sc->cfg.bus_ocp_th},
        {VBUS_OVP, sc->cfg.bus_ovp_th},
        {VAC_OVP, sc->cfg.ac_ovp_th},
        {SET_IBAT_SNS_RES, sc->cfg.sense_r_mohm},
    };

    ret = sc8546d_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev,"%s Failed to reset registers(%d)\n", __func__, ret);
    }

    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        dev_info(sc->dev,"%s (%d)\n", __func__, props[i].conv_data);
        ret = sc8546d_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    if (sc->mode == SLAVE) {
        sc8546d_disable_vbus_range(sc);
    }

    sc8546d_dump_reg(sc);

    return ret;
}

__maybe_unused static int sc8546d_init_device_protect(struct sc8546d *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc8546d_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.bat_ovp_disable},
        {IBAT_OCP_DIS, sc->cfg.bat_ocp_disable},
        {IBUS_UCP_DIS, sc->cfg.bus_ucp_disable},
        {IBUS_OCP_DIS, sc->cfg.bus_ocp_disable},
        {VBUS_OVP_DIS, sc->cfg.bus_ovp_disable},

        {IBAT_OCP, sc->cfg.bat_ocp_th},
        {VBAT_OVP, sc->cfg.bat_ovp_th},
        {IBUS_OCP, sc->cfg.bus_ocp_th},
        {VBUS_OVP, sc->cfg.bus_ovp_th},
        {VAC_OVP, sc->cfg.ac_ovp_th},
        {SET_IBAT_SNS_RES, sc->cfg.sense_r_mohm},
    };

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        dev_info(sc->dev,"%s (%d)\n", __func__, props[i].conv_data);
        ret = sc8546d_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    if (sc->mode == SLAVE) {
        sc8546d_disable_vbus_range(sc);
    }

    return ret;
}

/*********************mtk charger interface start**********************************/
static inline int to_sc8546d_adc(enum adc_channel chan)
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

static int mtk_sc8546d_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8546d_set_batovp_th(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_sc8546d_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8546d_set_batocp_th(sc, uA/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_sc8546d_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int mv;
    mv = uV / 1000;

    return sc8546d_set_busovp_th(sc, mv);
}

static int mtk_sc8546d_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ma;
    ma = uA / 1000;

    return sc8546d_set_busocp_th(sc, ma);
}

static int mtk_sc8546d_enable_chg(struct charger_device *chg_dev, bool en)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8546d_enable_charge(sc,en);

    return ret;
}

static int mtk_sc8546d_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8546d_check_charge_enabled(sc, en);

    return ret;

}

static int mtk_sc8546d_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
            int *min, int *max)
{

    struct sc8546d *sc = charger_get_data(chg_dev);

    sc8546d_get_adc_data(sc, to_sc8546d_adc(chan), max);

    if(chan != ADC_CHANNEL_TEMP_JC)
        *max = *max * 1000;
    else {
        *max = 25;
    }

    *min = *max;

    return 0;

}

static int mtk_sc8546d_get_adc_accuracy(struct charger_device *chg_dev,
                enum adc_channel chan, int *min, int *max)
{
    *min = *max = sc8546d_adc_accuracy_tbl[to_sc8546d_adc(chan)];
    return 0;
}

static int mtk_sc8546d_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    struct sc8546d *sc = charger_get_data(chg_dev);

    return sc8546d_is_vbuslowerr(sc,err);
}
static int mtk_sc8546d_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    dev_info(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8546d_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    dev_info(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8546d_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    dev_info(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8546d_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    dev_info(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8546d_init_chip(struct charger_device *chg_dev)
{
	struct sc8546d *sc = charger_get_data(chg_dev);

    return sc8546d_init_device_protect(sc);
}

static int mtk_sc8546d_enable_adc(struct charger_device *chg_dev, bool en)
{
    struct sc8546d *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8546d_enable_adc(sc,en);

    return ret;
}

static const struct charger_ops sc8546d_chg_ops = {
    .enable = mtk_sc8546d_enable_chg,
    .is_enabled = mtk_sc8546d_is_chg_enabled,
    .get_adc = mtk_sc8546d_get_adc,
    .get_adc_accuracy = mtk_sc8546d_get_adc_accuracy,
    .set_vbusovp = mtk_sc8546d_set_vbusovp,
    .set_ibusocp = mtk_sc8546d_set_ibusocp,
    .set_vbatovp = mtk_sc8546d_set_vbatovp,
    .set_ibatocp = mtk_sc8546d_set_ibatocp,
    .init_chip = mtk_sc8546d_init_chip,
    .is_vbuslowerr = mtk_sc8546d_is_vbuslowerr,
    .set_vbatovp_alarm = mtk_sc8546d_set_vbatovp_alarm,
    .reset_vbatovp_alarm = mtk_sc8546d_reset_vbatovp_alarm,
    .set_vbusovp_alarm = mtk_sc8546d_set_vbusovp_alarm,
    .reset_vbusovp_alarm = mtk_sc8546d_reset_vbusovp_alarm,
    .enable_adc = mtk_sc8546d_enable_adc,
};

/********************mtk charger interface end*************************************************/

/********************creat devices note start*************************************************/
static ssize_t sc8546d_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8546d *sc = dev_get_drvdata(dev);
    u8 addr, val, tmpbuf[300];
    int len, idx, ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8546d");
    for (addr = 0x0; addr <= 0x3C; addr++) {
        if((addr < 0x24) || (addr > 0x2B && addr < 0x33)
            || addr == 0x36 || addr == 0x3C) {
            ret = sc8546d_read_block(sc, addr, &val, 1);
            if (ret == 0) {
                len = snprintf(tmpbuf, PAGE_SIZE - idx,
                        "Reg[%.2X] = 0x%.2x\n", addr, val);
                memcpy(&buf[idx], tmpbuf, len);
                idx += len;
            }
        }
    }

    return idx;
}

static ssize_t sc8546d_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8546d *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg, val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x3C)
        sc8546d_write_byte(sc, (unsigned char)reg, (unsigned char)val);
    return count;
}

static DEVICE_ATTR(registers, 0660, sc8546d_show_registers, sc8546d_store_register);

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	bool enable;
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = sc8546d_check_charge_enabled(sc, &enable);
	if (ret < 0) {
		pr_err("sc8546d: sc8546d_is_chg_en not valid\n");
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
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc8546d_enable_charge(sc, enable);
	if (ret) {
		pr_err("sc8546d Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("sc8546d  %s charging \n",
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}
	sc8546d_get_adc_data(sc, to_sc8546d_adc(ADC_CHANNEL_VBUS), &vbus);
	vbus *= 1000;

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0444, show_vbus, NULL);

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", sc->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}

	regmap_read(sc->regmap, sc->reg_addr, &sc->reg_data);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", sc->reg_addr, sc->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8546d *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8546d chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_data = tmp;
	regmap_write(sc->regmap, sc->reg_addr, sc->reg_data);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static void sc8546d_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_vbus);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_registers);
}

/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
static inline int status_reg_to_charger(enum sc8546d_notify notify)
{
    switch (notify) {
    case SC854X_NOTIFY_IBUSUCPF:
        return CHARGER_DEV_NOTIFY_IBUSUCP_FALL;
    case SC854X_NOTIFY_VBUSOVPALM:
        return CHARGER_DEV_NOTIFY_VBUSOVP_ALARM;
    case SC854X_NOTIFY_VBATOVPALM:
        return CHARGER_DEV_NOTIFY_VBATOVP_ALARM;
    case SC854X_NOTIFY_IBUSOCP:
        return CHARGER_DEV_NOTIFY_IBUSOCP;
    case SC854X_NOTIFY_VBUSOVP:
        return CHARGER_DEV_NOTIFY_VBUS_OVP;
    case SC854X_NOTIFY_IBATOCP:
        return CHARGER_DEV_NOTIFY_IBATOCP;
    case SC854X_NOTIFY_VBATOVP:
        return CHARGER_DEV_NOTIFY_BAT_OVP;
    case SC854X_NOTIFY_VOUTOVP:
        return CHARGER_DEV_NOTIFY_VOUTOVP;
    default:
        return -EINVAL;
        break;
    }
    return -EINVAL;
}

static void sc8546d_check_fault_status(struct sc8546d *sc)
{
    u8 flag = 0;
    int i,j;
    int noti;

    mutex_lock(&sc->data_lock);

    for (i=0;i < ARRAY_SIZE(cp_intr_flag);i++) {
        sc8546d_read_block(sc, cp_intr_flag[i].reg, &flag, 1);
        for (j=0; j <  cp_intr_flag[i].len; j++) {
            if (flag & cp_intr_flag[i].bit[j].mask) {
                dev_err(sc->dev,"trigger :%s\n",cp_intr_flag[i].bit[j].name);
                noti = status_reg_to_charger(cp_intr_flag[i].bit[j].notify);
                if(noti >= 0) {
                    charger_dev_notify(sc->chg_dev, noti);
                }
            }
        }
    }

    mutex_unlock(&sc->data_lock);
}

static irqreturn_t sc8546d_charger_interrupt(int irq, void *dev_id)
{
    struct sc8546d *sc = dev_id;

    dev_err(sc->dev,"INT OCCURED\n");
#if 1
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

    sc8546d_check_fault_status(sc);

    mutex_unlock(&sc->irq_complete);

    power_supply_changed(sc->fc2_psy);
#endif
    return IRQ_HANDLED;
}

static int sc8546d_irq_register(struct sc8546d *sc)
{
    int ret;

    if (gpio_is_valid(sc->irq_gpio)) {
        ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN,"sc8546d_irq");
        if (ret) {
            dev_err(sc->dev,"failed to request sc8546d_irq\n");
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
                NULL, sc8546d_charger_interrupt,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                sc8546d_irq_name[sc->mode], sc);

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

static enum power_supply_property sc8546d_charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_TEMP,
};


static int sc8546d_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8546d *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8546d_check_adapter_inserted(sc, &result);
        val->intval = result;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8546d_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbus_volt = result * 1000;
        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8546d_get_adc_data(sc, ADC_IBAT, &result);
        if (!ret)
            sc->ibus_curr = result * 1000;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8546d_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}


static int sc8546d_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    struct sc8546d *sc = power_supply_get_drvdata(psy);

    dev_err(sc->dev,"prop = %d\n",  prop);
    switch (prop) {

    default:
        return -EINVAL;
    }

    return 0;
}


static int sc8546d_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    int ret;

    switch (prop) {

    default:
        ret = 0;
        break;
    }
    return ret;
}

static int sc8546d_psy_register(struct sc8546d *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc8546d_psy_name[sc->mode];

    sc->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
    sc->psy_desc.properties = sc8546d_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8546d_charger_props);
    sc->psy_desc.get_property = sc8546d_charger_get_property;
    sc->psy_desc.set_property = sc8546d_charger_set_property;
    sc->psy_desc.property_is_writeable = sc8546d_charger_is_writeable;


    sc->fc2_psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->fc2_psy)) {
        dev_err(sc->dev,"failed to register fc2_psy\n");
        return PTR_ERR(sc->fc2_psy);
    }

    dev_err(sc->dev,"%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}

/************************psy end**************************************/


static int sc8546d_set_work_mode(struct sc8546d *sc, int mode)
{
    switch (mode)
    {
    case SC854X_STANDALONG:
        sc->mode = STANDALONE;
        break;
    case SC854X_MASTER:
        sc->mode = MASTER;
        break;
    case SC854X_SLAVE:
        sc->mode = SLAVE;
        break;
    default:
        dev_err(sc->dev,"Not find work mode\n");
        return -ENODEV;
    }
    dev_err(sc->dev,"work mode is %s\n", sc->mode == STANDALONE
        ? "standalone" : (sc->mode == MASTER ? "master" : "slave"));

    return 0;
}

static int sc8546d_parse_dt(struct sc8546d *sc, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc8546d,bat-ovp-disable", &(sc->cfg.bat_ovp_disable)},
        {"sc,sc8546d,bat-ocp-disable", &(sc->cfg.bat_ocp_disable)},
        {"sc,sc8546d,bus-ovp-disable", &(sc->cfg.bus_ovp_disable)},
        {"sc,sc8546d,bus-ucp-disable", &(sc->cfg.bus_ucp_disable)},
        {"sc,sc8546d,bus-ocp-disable", &(sc->cfg.bus_ocp_disable)},
        {"sc,sc8546d,bat-ovp-threshold", &(sc->cfg.bat_ovp_th)},
        {"sc,sc8546d,bat-ocp-threshold", &(sc->cfg.bat_ocp_th)},
        {"sc,sc8546d,ac-ovp-threshold", &(sc->cfg.ac_ovp_th)},
        {"sc,sc8546d,bus-ovp-threshold", &(sc->cfg.bus_ovp_th)},
        {"sc,sc8546d,bus-ocp-threshold", &(sc->cfg.bus_ocp_th)},
        {"sc,sc8546d,sense-resistor-mohm", &(sc->cfg.sense_r_mohm)},
        {"sc,sc8546d,ibat-regulation-threshold", &(sc->cfg.ibat_reg_th)},
        {"sc,sc8546d,vbat-regulation-threshold", &(sc->cfg.vbat_reg_th)},
        {"sc,sc8546d,adc-rate", &(sc->cfg.adc_rate)},
    };

    sc->irq_gpio = of_get_named_gpio(np, "sc,sc8546d,irq-gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        dev_err(sc->dev,"fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }

    np = of_find_node_by_name(dev->of_node, "charger");
    if (!np) {
        dev_err(sc->dev, "not fine charger node\n");
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
            return ret;
        }
    }

    return 0;
}

static struct of_device_id sc8546d_charger_match_table[] = {
    {   .compatible = "sc,sc8546d-standalone",
        .data = &sc8546d_mode_data[SC854X_STANDALONG],},
    {   .compatible = "sc,sc8546d-master",
        .data = &sc8546d_mode_data[SC854X_MASTER],},
    {   .compatible = "sc,sc8546d-slave",
        .data = &sc8546d_mode_data[SC854X_SLAVE],},
    {},
};



static int sc8546d_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc8546d *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret, i;

    dev_err(&client->dev, "%s (%s)\n", __func__, SC8546D_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc8546d), GFP_KERNEL);
    if (!sc) {
        ret = -ENOMEM;
        goto err_kzalloc;
    }

    sc->dev = &client->dev;

    sc->client = client;

    sc->regmap = devm_regmap_init_i2c(client,
                            &sc8546d_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev,"Failed to initialize regmap\n");
        ret = PTR_ERR(sc->regmap);
        goto err_regmap_init;
    }

    for (i = 0; i < ARRAY_SIZE(sc8546d_reg_fields); i++) {
        const struct reg_field *reg_fields = sc8546d_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev,"cannot allocate regmap field\n");
            ret = PTR_ERR(sc->rmap_fields[i]);
            goto err_regmap_field;
        }
    }


    mutex_init(&sc->data_lock);
    mutex_init(&sc->irq_complete);

    sc->resume_completed = true;
    sc->irq_waiting = false;

    ret = sc8546d_detect_device(sc);
    if (ret) {
        dev_err(sc->dev,"No sc8546d device found!\n");
        goto err_detect_dev;
    }

    i2c_set_clientdata(client, sc);
    sc8546d_create_device_node(&(client->dev));

    match = of_match_node(sc8546d_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev,"device tree match not found!\n");
        goto err_match_node;
    }

    sc8546d_set_work_mode(sc, *(int *)match->data);
    if (ret) {
        dev_err(sc->dev,"Fail to set work mode!\n");
        goto err_set_mode;
    }

    ret = sc8546d_parse_dt(sc, &client->dev);
    if (ret) {
        dev_err(sc->dev,"Fail to parse dt!\n");
        goto err_parse_dt;
    }

    ret = sc8546d_init_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = sc8546d_psy_register(sc);
    if (ret) {
        dev_err(sc->dev,"Fail to register psy!\n");
        goto err_register_psy;
    }

    ret = sc8546d_irq_register(sc);
    if (ret < 0) {
        dev_err(sc->dev,"Fail to register irq!\n");
        goto err_register_irq;
    }

    sc->chg_dev = charger_device_register(sc->chg_dev_name,
                        &client->dev, sc,
                        &sc8546d_chg_ops,
                        &sc8546d_chg_props);
    if (IS_ERR_OR_NULL(sc->chg_dev)) {
        ret = PTR_ERR(sc->chg_dev);
        dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_charger;
    }

    device_init_wakeup(sc->dev, 1);

    dev_err(sc->dev,"sc8546d probe successfully!\n");

    return 0;

err_register_psy:
err_register_irq:
err_register_charger:
err_init_device:
    power_supply_unregister(sc->fc2_psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
    mutex_destroy(&sc->data_lock);
    mutex_destroy(&sc->irq_complete);
err_regmap_init:
err_regmap_field:
    devm_kfree(&client->dev, sc);
err_kzalloc:
    dev_err(&client->dev,"sc8546d probe fail\n");
    return ret;
}

#ifdef CONFIG_PM_SLEEP
static inline bool is_device_suspended(struct sc8546d *sc)
{
    return !sc->resume_completed;
}

static int sc8546d_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8546d *sc = i2c_get_clientdata(client);

    mutex_lock(&sc->irq_complete);
    sc->resume_completed = false;
    mutex_unlock(&sc->irq_complete);
    dev_err(sc->dev,"Suspend successfully!");

    return 0;
}

static int sc8546d_suspend_noirq(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8546d *sc = i2c_get_clientdata(client);

    if (sc->irq_waiting) {
        pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
        return -EBUSY;
    }
    return 0;
}

static int sc8546d_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8546d *sc = i2c_get_clientdata(client);

    mutex_lock(&sc->irq_complete);
    sc->resume_completed = true;
    if (sc->irq_waiting) {
        sc->irq_disabled = false;
        enable_irq(client->irq);
        mutex_unlock(&sc->irq_complete);
        sc8546d_charger_interrupt(client->irq, sc);
    } else {
        mutex_unlock(&sc->irq_complete);
    }

    power_supply_changed(sc->fc2_psy);
    dev_err(sc->dev,"Resume successfully!");

    return 0;
}

static const struct dev_pm_ops sc8546d_pm_ops = {
    .resume         = sc8546d_resume,
    .suspend_noirq  = sc8546d_suspend_noirq,
    .suspend        = sc8546d_suspend,
};

#endif /*CONFIG_PM_SLEEP*/


static void sc8546d_charger_remove(struct i2c_client *client)
{
    struct sc8546d *sc = i2c_get_clientdata(client);

    power_supply_unregister(sc->fc2_psy);
    mutex_destroy(&sc->data_lock);
    mutex_destroy(&sc->irq_complete);

    devm_kfree(&client->dev, sc);

}


static void sc8546d_charger_shutdown(struct i2c_client *client)
{

}

static struct i2c_driver sc8546d_charger_driver = {
    .driver     = {
        .name   = "sc8546d-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc8546d_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm    = &sc8546d_pm_ops,
#endif /*CONFIG_PM_SLEEP*/
    },
    .probe      = sc8546d_charger_probe,
    .remove     = sc8546d_charger_remove,
    .shutdown   = sc8546d_charger_shutdown,
};

module_i2c_driver(sc8546d_charger_driver);

MODULE_DESCRIPTION("SC SC854X Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
