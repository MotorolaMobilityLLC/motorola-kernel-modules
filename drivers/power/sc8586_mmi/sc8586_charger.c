// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2025 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
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
#include "sc8586_charger.h"

#ifdef CONFIG_MTK_CLASS
#include "charger_class.h"

#ifdef CONFIG_MTK_CHARGER_V4P19
#include "mtk_charger_intf.h"
#endif /*CONFIG_MTK_CHARGER_V4P19*/

#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
#include "dvchg_class.h"
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/


#define SC8586_DRV_VERSION              "1.0.0_G"

enum {
    SC8586_STANDALONG = 0,
    SC8586_MASTER,
    SC8586_SLAVE,
};

static const char* sc8586_psy_name[] = {
    [SC8586_STANDALONG] = "cp-standalone",
    [SC8586_MASTER] = "cp-master",
    [SC8586_SLAVE] = "cp-slave",
};

static const char* sc8586_irq_name[] = {
    [SC8586_STANDALONG] = "sc8586-standalone-irq",
    [SC8586_MASTER] = "sc8586-master-irq",
    [SC8586_SLAVE] = "sc8586-slave-irq",
};

static int sc8586_mode_data[] = {
    [SC8586_STANDALONG] = SC8586_STANDALONG,
    [SC8586_MASTER] = SC8586_MASTER,
    [SC8586_SLAVE] = SC8586_SLAVE,
};

enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VUSB,
    RESERVED,
    ADC_VOUT,
    ADC_VBAT1,
    ADC_IBAT,
    ADC_VBAT2,
    ADC_TDIE,
    ADC_MAX_NUM,
}SC_858X_ADC_CH;

static const u32 sc8586_adc_accuracy_tbl[ADC_MAX_NUM] = {
	150000,	/* IBUS */
	35000,	/* VBUS */
	35000,	/* VUSB */
    0,	/* RESERVED */
	20000,	/* VOUT */
	20000,	/* VBAT1 */
	200000,	/* IBAT */
    20000,	/* VBAT2 */
	4,	/* TDIE */
};

static const int sc8586_adc_m[] =
    {1875, 625, 625, 10, 125, 125, 45, 125, 5};

static const int sc8586_adc_l[] =
    {1000, 100, 100, 10, 100, 100, 10, 100, 10};

enum sc8586_error_stata {
    ERROR_VBUS_HIGH = 0,
	ERROR_VBUS_LOW,
	ERROR_VBUS_OVP,
	ERROR_IBUS_OCP,
	ERROR_VBAT_OVP,
	ERROR_IBAT_OCP,
};

static struct intr_flag cp_intr_flag[] = {
    { .reg = 0x03, .len = 1, .bit = {
                {.mask = BIT(5), .name = "vusb ovp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x07, .len = 2, .bit = {
                {.mask = BIT(2), .name = "ibus ucp rise flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "ibus ucp fall flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x08, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out ovp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x09, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out uvp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x0a, .len = 2, .bit = {
                {.mask = BIT(7), .name = "por flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "pin diag fall flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x10, .len = 2, .bit = {
                {.mask = BIT(7), .name = "usb pd flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "usb pd exit flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x12, .len = 8, .bit = {
                {.mask = BIT(0), .name = "vusb insert flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus uvlo flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vout insert flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vout ok chg flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vout ok rev flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vout ok sw regn flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vbus ok chg flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x14, .len = 7, .bit = {
                {.mask = BIT(0), .name = "vout ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "ss fail flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "conv ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "wd timeout flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ss timeout flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "tshut flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x15, .len = 5, .bit = {
                {.mask = BIT(0), .name = "ibus ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "ibat ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbat1 ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vbat2 ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vusb remove flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x18, .len = 5, .bit = {
                {.mask = BIT(4), .name = "adc done flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
};

static struct intr_flag cps_cp_intr_flag[] = {
    { .reg = 0x03, .len = 1, .bit = {
                {.mask = BIT(5), .name = "vusb ovp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x07, .len = 2, .bit = {
                {.mask = BIT(2), .name = "ibus ucp rise flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "ibus ucp fall flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x08, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out ovp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x09, .len = 1, .bit = {
                {.mask = BIT(3), .name = "pmid2out uvp flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x0a, .len = 2, .bit = {
                {.mask = BIT(7), .name = "por flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(0), .name = "pin diag fall flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x10, .len = 2, .bit = {
                {.mask = BIT(7), .name = "usb pd flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "usb pd exit flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x12, .len = 8, .bit = {
                {.mask = BIT(0), .name = "vusb insert flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus range flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vout insert flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vout ok chg flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vout ok rev flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vout ok sw regn flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vbus ok chg flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x14, .len = 8, .bit = {
                {.mask = BIT(0), .name = "vout ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "ss fail flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "conv ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "wd timeout flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ss timeout flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "tshut flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "rev fail flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x15, .len = 8, .bit = {
                {.mask = BIT(0), .name = "ibus ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "ibat ocp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbat1 ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vbat2 ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vusb pd enter flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vusb pd exit flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vusb remove flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vbus remove flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
    { .reg = 0x18, .len = 5, .bit = {
                {.mask = BIT(4), .name = "adc done flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
};

#ifdef CONFIG_MTK_CLASS
static const struct charger_properties sc8586_chg_props = {
	.alias_name = "sc8586_chg",
};
#endif /*CONFIG_MTK_CLASS*/
__maybe_unused
u8 val2reg(enum sc8586_reg_range id, u32 val) {
    int i;
    u8 reg;
    const struct reg_range *range = &sc8586_reg_range[id];

    if (!range)
        return 0;

    if (range->table) {
        if (val <= range->table[0])
            return 0;
        for (i = 0; i < range->num_table - 1; i++) {
            if (val == range->table[i]) {
                return i;
            }
            if (val > range->table[i] && val < range->table[i + 1]) {
                return range->round_up ? i + 1 : i;
            }
        }
        return range->num_table - 1;
    }
    if (val <= range->min)
        reg = (range->min - range->offset) / range->step;
    else if (val >= range->max)
        reg = (range->max - range->offset) / range->step;
    else if (range->round_up)
        reg = (val - range->offset) / range->step + 1;
    else
        reg = (val - range->offset) / range->step;
    return reg;
}

__maybe_unused
u32 reg2val(enum sc8586_reg_range id, u8 reg) {
    const struct reg_range *range = &sc8586_reg_range[id];
    if (!range)
        return reg;
    return range->table ? range->table[reg] : range->offset + range->step * reg;
}
int sc8586_i2c_write_bytes(struct sc8586_chip *sc, uint8_t reg, uint8_t len, uint8_t *val)
{
    struct i2c_client *i2c;

    if (IS_ERR_OR_NULL(sc))
        return PTR_ERR(sc);

    i2c = to_i2c_client(sc->dev);
    return i2c_smbus_write_i2c_block_data(i2c, reg, len, val);
}

int sc8586_i2c_read_bytes(struct sc8586_chip *sc, uint8_t reg, uint8_t len, uint8_t *val)
{
    struct i2c_client *i2c;

    if (IS_ERR_OR_NULL(sc))
        return PTR_ERR(sc);

    i2c = to_i2c_client(sc->dev);
    return i2c_smbus_read_i2c_block_data(i2c, reg, len, val);
}

int sc8586_i2c_write_byte(struct sc8586_chip *sc, uint8_t reg, uint8_t val)
{
    return sc8586_i2c_write_bytes(sc, reg, 1, &val);
}

int sc8586_i2c_read_byte(struct sc8586_chip *sc, uint8_t reg, uint8_t *val)
{
    return sc8586_i2c_read_bytes(sc, reg, 1, val);
}

int sc8586_field_read(struct sc8586_chip *sc,
                            enum sc8586_fields field_id, int *val)
{
    int ret;
    uint8_t reg_val = 0;
    uint8_t mask = 0;

    if (IS_ERR_OR_NULL(sc))
        return PTR_ERR(sc);
    if (NULL == val)
        return -EINVAL;

    mutex_lock(&sc->field_rw_lock);
    mask = GENMASK(sc8586_reg_fields[field_id].msb, sc8586_reg_fields[field_id].lsb);


    ret = sc8586_i2c_read_byte(sc, sc8586_reg_fields[field_id].reg, &reg_val);
    if (ret < 0) {
        sc8586_err("sc8586 read field %d fail: %d\n", field_id, ret);
        goto out;
    }

    reg_val &= mask;
    reg_val >>= sc8586_reg_fields[field_id].lsb;

    *val = reg_val;

out:
    mutex_unlock(&sc->field_rw_lock);
    return ret;
}

int sc8586_field_write(struct sc8586_chip *sc,
                            enum sc8586_fields field_id, int val)
{
    int ret;
    uint8_t reg_val = 0, tmp = 0;
    uint8_t mask = 0;

    if (IS_ERR_OR_NULL(sc))
        return PTR_ERR(sc);

    mutex_lock(&sc->field_rw_lock);
    mask = GENMASK(sc8586_reg_fields[field_id].msb, sc8586_reg_fields[field_id].lsb);

    ret = sc8586_i2c_read_byte(sc, sc8586_reg_fields[field_id].reg, &reg_val);
    if (ret < 0) {
        sc8586_err("sc8586 wr field %d fail: %d\n", field_id, ret);
        goto out;
    }

    tmp = reg_val & ~mask;
    val <<= sc8586_reg_fields[field_id].lsb;
    tmp |= val  & mask;

    if (sc8586_reg_fields[field_id].force_write || tmp != reg_val) {
        ret = sc8586_i2c_write_byte(sc, sc8586_reg_fields[field_id].reg, tmp);
    }


out:
    if (ret < 0) {
        sc8586_err("sc8586 write field %d fail: %d\n", field_id, ret);
    }

    mutex_unlock(&sc->field_rw_lock);
    return ret;
}

/*******************************************************/
__maybe_unused static int sc8586_detect_device(struct sc8586_chip *sc)
{
    int ret;
    int val;

    if (!sc)
        return -EINVAL;

    ret = sc8586_field_read(sc, F_DEVICE_ID, &val);
    if (ret >= 0 && val == SC8586_DEVICE_ID) {
        sc->device_id = val;
        return 0;
    }

    ret = sc8586_field_read(sc, F_CPS_DEVICE_ID, &val);
    if (ret >= 0 && val == CPS2043_DEVICE_ID) {
        sc->device_id = val;
        return 0;
    }

    sc8586_err("Could not detect a compatible device\n");
    return -ENODEV;
}

__maybe_unused static int sc8586_reg_reset(struct sc8586_chip *sc)
{
    return sc8586_field_write(sc, F_REG_RST, 1);
}

__maybe_unused static int sc8586_dump_reg(struct sc8586_chip *sc)
{
    int ret;
    int i;
    uint8_t val;
    //int scp_block_enable = 0;
    int dpdm_block_enable = 0;

    for (i = 0; i <= 0x2B; i++) {
        ret = sc8586_i2c_read_byte(sc, i, &val);
        sc8586_err( "%s cp reg[0x%02x] = 0x%02x\n",
                __func__, i, val);
    }

    ret = sc8586_i2c_read_byte(sc, 0x3D, &val);
    sc8586_err( "%s cp reg[0x3d] = 0x%02x\n",
                __func__, val);
    ret = sc8586_i2c_read_byte(sc, 0x3E, &val);
    sc8586_err( "%s cp reg[0x3e] = 0x%02x\n",
                __func__, val);
    ret = sc8586_i2c_read_byte(sc, 0x3F, &val);
    sc8586_err( "%s cp reg[0x3f] = 0x%02x\n",
                __func__, val);
    ret = sc8586_i2c_read_byte(sc, 0xD1, &val);
    sc8586_err( "%s cp reg[0xd1] = 0x%02x\n",
                __func__, val);

   /* scp_block_enable = sc8586_get_scp_enable(sc);
    if (scp_block_enable) {
        for (i = 0x2E; i <= 0x33; i++) {
            ret = sc8586_i2c_read_byte(sc, i, &val);
            sc8586_err( "%s scp reg[0x%02x] = 0x%02x\n",
                    __func__, i, val);
        }
    }*/

    dpdm_block_enable = sc8586_get_dpdm_enable(sc);
    if (dpdm_block_enable) {
        for (i = 0x34; i <= 0x3C; i++) {
            ret = sc8586_i2c_read_byte(sc, i, &val);
            sc8586_err( "%s scp reg[0x%02x] = 0x%02x\n",
                    __func__, i, val);
        }
    }

    return ret;
}


__maybe_unused static int sc8586_check_charge_enabled(struct sc8586_chip *sc, bool *enabled)
{
    int ret, val;

    if (!enabled)
        return -EINVAL;

    ret = sc8586_field_read(sc, F_CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    sc8586_info("%s:%d", __func__, val);

    return ret;
}

__maybe_unused static int sc8586_get_status(struct sc8586_chip *sc, uint32_t *status)
{
    int ret, val;

    if (!status)
        return -EINVAL;
    *status = 0;

    ret = sc8586_field_read(sc, F_PMID_ERROR_HI_STAT, &val);
    if (ret < 0) {
        sc8586_err( "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    ret = sc8586_field_read(sc, F_PMID_ERROR_LO_STAT, &val);
    if (ret < 0) {
        sc8586_err( "%s fail to read VBUS_ERRORLO_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_LOW);

    return ret;

}

__maybe_unused static int sc8586_enable_adc(struct sc8586_chip *sc, bool en)
{
    sc8586_info("%s:%d", __func__, en);
    return sc8586_field_write(sc, F_ADC_EN, !!en);
}

static int sc858x_enable_adc(struct charger_device *chg_dev, bool en) {
    int ret;
    struct sc8586_chip *sc = charger_get_data(chg_dev);
    if (!sc) {
        pr_err("sc8586 chip not valid\n");
        return -1;
    }
    sc8586_info("%s:%d", __func__, en);
    ret = sc8586_enable_adc(sc, en);
    if (ret) {
        sc8586_err("%s enable failed %d", __func__, en);
        return ret;
    }
    return 0;
}
__maybe_unused static int sc8586_set_adc_scanrate(struct sc8586_chip *sc, bool oneshot)
{
    sc8586_info("%s:%d", __func__, oneshot);
    return sc8586_field_write(sc, F_ADC_RATE, !!oneshot);
}

static int sc8586_get_adc_data(struct sc8586_chip *sc,
            int channel, int *result)
{
    uint8_t val[2] = {0};
    int ret;

    if (!sc || !result)
        return -EINVAL;

    if(channel >= ADC_MAX_NUM)
        return -EINVAL;

    sc8586_field_write(sc, F_ADC_FREEZE, 1);
    ret = sc8586_i2c_read_bytes(sc, SC8586_REG1A + (channel << 1), 2, val);
    if (ret < 0) {
        return ret;
    }

    *result = (val[1] | (val[0] << 8)) *
                sc8586_adc_m[channel] / sc8586_adc_l[channel];
    if (sc->device_id == CPS2043_DEVICE_ID && channel == 8) {
         sc8586_info("%s TDIE tmp - 40 %d %d", __func__, channel, *result);
        *result -= 40;
    }

    sc8586_info("%s %d %d", __func__, channel, *result);

    sc8586_field_write(sc, F_ADC_FREEZE, 0);

    return ret;
}

/*static int sc8586_set_private_code(struct sc8586_chip *sc)
{
    return regmap_write(sc->regmap, SC8586_REG7C, SC8586_PRIVATE_CODE);
}*/

__maybe_unused static int sc8586_set_busovp_th(struct sc8586_chip *sc, int threshold)
{
    int reg_val;
    int temp_threshold = 0;
    int ret;
    int val;

    if (!sc)
        return -EINVAL;

    ret = sc8586_field_read(sc, F_MODE, &val);
    if (ret < 0) {
        sc8586_err( "%s fail to read MODE(%d)\n", __func__, ret);
        return ret;
    }

    /*
      0 : 4:1
      1 : 3:1
    */

    if (val == 0) {
        temp_threshold = threshold / 2;
    } else if (val == 3) {
        temp_threshold = threshold * 2;
    } else if (val == 2) {
        temp_threshold = threshold;
    } else if (val == 1) {
        temp_threshold = (threshold * 2) / 3 ;
    } else {
        return -1;
    }

    if (sc->device_id == CPS2043_DEVICE_ID)
        reg_val = val2reg(CPS2043_VBUS_OVP, temp_threshold);
    else
        reg_val = val2reg(SC8586_VBUS_OVP, temp_threshold);
    sc8586_info("%s:%d-%d", __func__, threshold, reg_val);

    return sc8586_field_write(sc, F_VBUS_OVP, reg_val);
}

__maybe_unused static int sc8586_set_busocp_th(struct sc8586_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8586_IBUS_OCP, threshold);

    sc8586_info("%s:%d-%d", __func__, threshold, reg_val);

    return sc8586_field_write(sc, F_IBUS_OCP, reg_val);
}

__maybe_unused static int sc8586_set_batovp_th(struct sc8586_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8586_VBAT_OVP, threshold);

    sc8586_info("%s:%d-%d", __func__, threshold, reg_val);

    return sc8586_field_write(sc, F_VBAT_OVP, reg_val);
}

__maybe_unused static int sc8586_set_batocp_th(struct sc8586_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8586_IBAT_OCP, threshold);

    sc8586_info("%s:%d-%d", __func__, threshold, reg_val);

    return sc8586_field_write(sc, F_IBAT_OCP, reg_val);
}

__maybe_unused static int sc8586_set_vbusovp_alarm(struct sc8586_chip *sc, int threshold)
{
    sc8586_info("%s:%d", __func__, threshold);

    return 0;
}

__maybe_unused static int sc8586_set_vbatovp_alarm(struct sc8586_chip *sc, int threshold)
{
    sc8586_info("%s:%d", __func__, threshold);

    return 0;
}

__maybe_unused static int sc8586_is_vbuslowerr(struct sc8586_chip *sc, bool *err)
{
    int ret;
    int val;

    if (!err)
        return -EINVAL;

    ret = sc8586_field_read(sc, F_PMID_ERROR_LO_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    sc8586_info("%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc8586_is_vbushigherr(struct sc8586_chip *sc, bool *err)
{
    int ret;
    int val;

    if (!err)
        return -EINVAL;

    ret = sc8586_field_read(sc, F_PMID_ERROR_HI_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    sc8586_info("%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc8586_enable_charge(struct sc8586_chip *sc, bool en)
{
    int ret = 0;
    int vbus_value = 0, vout_value = 0, value = 0;
    int vbus_hi = 0, vbus_low = 0;

    if (!sc)
        return -EINVAL;

    sc8586_info("%s:%d",__func__,en);

    if (!en) {
        ret |= sc8586_field_write(sc, F_CP_EN, !!en);

        return ret;
    } else {
        ret = sc8586_get_adc_data(sc, ADC_VBUS, &vbus_value);
        ret |= sc8586_get_adc_data(sc, ADC_VOUT, &vout_value);
        sc8586_info("%s: vbus/vout:%d / %d = %d \r\n", __func__, vbus_value,
                        vout_value, vbus_value * 100 / vout_value);

        ret |= sc8586_field_read(sc, F_MODE, &value);
        sc8586_info("%s: mode:%d %s \r\n", __func__, value,
                (value == 0 ? "4:1": (value == 1 ? "3:1": (value == 2 ? "2:1" : "else"))));

        ret |= sc8586_field_read(sc, F_PMID_ERROR_LO_STAT, &vbus_low);
        ret |= sc8586_field_read(sc, F_PMID_ERROR_HI_STAT, &vbus_hi);
        sc8586_info("%s: high:%d  low:%d \r\n", __func__, vbus_hi, vbus_low);

        ret |= sc8586_field_write(sc, F_CP_EN, !!en);
        sc8586_dump_reg(sc);
        disable_irq(sc->irq);

        msleep(300);//delay 300ms and check if cp enabled successful

        ret |= sc8586_field_read(sc, F_PIN_DIAG_FAIL_FLAG, &value);
        sc8586_info("%s: pin diag fail:%d \r\n", __func__, value);

        ret |= sc8586_field_read(sc, F_CP_SWITCHING_STAT, &value);
        if (!value) {
            sc8586_info("%s:enable fail \r\n", __func__);
            sc8586_dump_reg(sc);
        } else {
            sc8586_info("%s:enable success \r\n", __func__);
            //sc8586_dump_reg(sc);
        }

        enable_irq(sc->irq);
    }

    return ret;
}

__maybe_unused static void init_cps2043_reg(struct sc8586_chip *sc)
 {
    sc8586_info("%s:in\n", __func__);

    sc8586_i2c_write_byte(sc, 0x01, 0x50);
    sc8586_i2c_write_byte(sc, 0x03, 0x0b);
    sc8586_i2c_write_byte(sc, 0x02, 0x80);
    sc8586_i2c_write_byte(sc, 0x05, 0xa1);
    sc8586_i2c_write_byte(sc, 0x06, 0x3c);
    sc8586_i2c_write_byte(sc, 0x07, 0x20);
    sc8586_i2c_write_byte(sc, 0x08, 0x04);
    sc8586_i2c_write_byte(sc, 0x09, 0x04);
    sc8586_i2c_write_byte(sc, 0x0c, 0x18);
    sc8586_i2c_write_byte(sc, 0x0f, 0x00);

    if ((sc->cfg.mode) != 0)
        sc8586_field_write(sc, F_MODE, sc->cfg.mode);
 }

__maybe_unused static int sc8586_init_device(struct sc8586_chip *sc)
{
    int ret = 0;
    int i;
    if (!sc)
        return -EINVAL;
    struct {
        enum sc8586_fields field_id;
        int conv_data;
    } props[] = {
        {F_VBAT_OVP_DIS, sc->cfg.vbat_ovp_dis},
        {F_VBAT_OVP, sc->cfg.vbat_ovp},
        {F_IBAT_OCP_DIS, sc->cfg.ibat_ocp_dis},
        {F_IBAT_OCP, sc->cfg.ibat_ocp},
        {F_VUSB_OVP_DIS, sc->cfg.vusb_ovp_dis},
        {F_VUSB_OVP, sc->cfg.vusb_ovp},
        {F_VBUS_OVP_DIS, sc->cfg.vbus_ovp_dis},
        {F_VBUS_OVP, sc->cfg.vbus_ovp},
        {F_VOUT_OVP_DIS, sc->cfg.vout_ovp_dis},
        {F_VOUT_OVP, sc->cfg.vout_ovp},
        {F_IBUS_OCP_DIS, sc->cfg.ibus_ocp_dis},
        {F_IBUS_OCP, sc->cfg.ibus_ocp},
        {F_IBUS_UCP_DIS, sc->cfg.ibus_ucp_fall_dis},
        {F_IBUS_UCP_FALL_DG_SET, sc->cfg.ibus_ucp_fall},
        {F_PMID2OUT_UVP_DIS, sc->cfg.pmid2out_uvp_dis},
        {F_PMID2OUT_UVP, sc->cfg.pmid2out_uvp},
        {F_PMID2OUT_OVP_DIS, sc->cfg.pmid2out_ovp_dis},
        {F_PMID2OUT_OVP, sc->cfg.pmid2out_ovp},
        {F_FSW_SET, sc->cfg.fsw_set},
        {F_SS_TIMEOUT, sc->cfg.ss_timeout},
        {F_WD_TIMEOUT, sc->cfg.wd_timeout},
        {F_SET_IBAT_SNS_RES, sc->cfg.ibat_sns_r},
        {F_MODE, sc->cfg.mode},
        {F_TSHUT_DIS, sc->cfg.tshut_dis},
        {F_VBAT2_ADC_EN, sc->cfg.ibat_sns_r},
        {F_SCP_EN, sc->cfg.scp_block_en},
        {F_DPDM_EN, sc->cfg.dpdm_block_en},
    };

    ret = sc8586_reg_reset(sc);
    if (ret < 0) {
        sc8586_err("%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    if (sc->device_id == CPS2043_DEVICE_ID) {
        init_cps2043_reg(sc);
    } else {
        for (i = 0; i < ARRAY_SIZE(props); i++)
            ret = sc8586_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    if (sc->mode == SC8586_SLAVE) {
        ret = sc8586_field_write(sc, F_PMID_INRANGE_DET_DIS, 1);
        if (ret < 0) {
            sc8586_err("%s Failed to set vbus in range(%d)\n", __func__, ret);
        }
    }

    sc8586_enable_adc(sc, true);
    //sc8586_set_fc_func_enable(sc, true);
    //sc8586_set_hvdcp_enable(sc, true);
    //sc8586_set_private_code(sc);

    sc8586_dump_reg(sc);

    return ret;
}


/*********************mtk charger interface start**********************************/
#ifdef CONFIG_MTK_CLASS
static inline int to_sc8586_adc(enum adc_channel chan)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		return ADC_VBUS;
	case ADC_CHANNEL_VBAT:
		return ADC_VBAT1;
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


static int mtk_sc8586_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sc8586_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8586_check_charge_enabled(sc, en);

    return ret;
}


static int mtk_sc8586_enable_chg(struct charger_device *chg_dev, bool en)
{
    struct sc8586_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8586_enable_charge(sc,en);

    return ret;
}


static int mtk_sc8586_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8586_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
    return 0;
}

static int mtk_sc8586_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8586_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
    return 0;
}

static int mtk_sc8586_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
    struct sc8586_chip *sc = charger_get_data(chg_dev);

    if (!min || !max)
        return -EINVAL;

    sc8586_get_adc_data(sc, to_sc8586_adc(chan), max);

    if(chan != ADC_CHANNEL_TEMP_JC)
        *max = *max * 1000;

    if (min != max)
		*min = *max;

    return 0;
}

static int mtk_sc8586_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
    if (!min || !max)
        return -EINVAL;
    *min = *max = sc8586_adc_accuracy_tbl[to_sc8586_adc(chan)];
    return 0;
}

static int mtk_sc8586_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    if (!err)
        return -EINVAL;
    struct sc8586_chip *sc = charger_get_data(chg_dev);

    return sc8586_is_vbuslowerr(sc,err);
}

static int mtk_sc8586_is_vbushigherr(struct charger_device *chg_dev, bool *err)
{
    if (!err)
        return -EINVAL;
    struct sc8586_chip *sc = charger_get_data(chg_dev);

    return sc8586_is_vbushigherr(sc,err);
}

static int mtk_sc8586_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8586_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
    //struct sc8586_chip *sc = charger_get_data(chg_dev);
    sc8586_err("%s",__func__);
    return 0;
}

static int mtk_sc8586_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_sc8586_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
    //struct sc8586_chip *sc = charger_get_data(chg_dev);
    sc8586_err("%s",__func__);
    return 0;
}

enum sc8565_cp_op_mode {
	CP_2_1_MODE = 0,
	CP_3_1_MODE,
	CP_4_1_MODE,
};

static int sc8586_operation_mode_select(struct charger_device *chg_dev, int cp_op_mode)
{
	struct sc8586_chip *sc  = charger_get_data(chg_dev);
	int ret = 0;
	int reg_op_mode = 1;

	dev_info(sc->dev, "%s: %d\n",__func__, cp_op_mode);
	switch (cp_op_mode) {
	case CP_2_1_MODE:
		reg_op_mode = 2;
		break;
	case CP_4_1_MODE:
		reg_op_mode = 0;
		break;
	default:
		reg_op_mode = 0;
		break;
	}

	ret = sc8586_field_write(sc, F_MODE, reg_op_mode);
	if(ret < 0) {
		dev_err(sc->dev,"%s failed to set operation mode:%d\n", __func__, ret);
	}

	return ret;
}

static int sc858x_config_mux(struct sc8586_chip *sc,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	u8 val;

	if (sc->mmi_disable_mux)
		return 0;

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN) {
	    sc-> otg_delay_mos_config = false;
        sc8586_field_read(sc, F_MODE, &ret);
		if(ret == 7) {
	       	sc8586_field_write(sc, F_MODE, 0);
			dev_err(sc->dev, "%s:mmi_mux dis cp otg reverse mode", __func__);
	    }
    }
        if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && typec_mos != MMI_DVCHG_MUX_DISABLE
                && wls_mos != MMI_DVCHG_MUX_DISABLE && wls_mos != MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux dis mos both fail ret=%d", __func__, ret);
                return ret;
            }
        }

        if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
            ret = sc8586_field_write(sc, F_OVPGATE_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
        }

        if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
            ret = sc8586_field_write(sc, F_OVPGATE_EN, 1);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
                return ret;
            }
        }
        if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
            ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 1);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            ret = sc8586_field_write(sc, F_OVPGATE_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux menu close wls mos fail ret=%d", __func__, ret);
                return ret;
            }
            mdelay(50);
        }

	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 1);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
	    ret = sc8586_field_write(sc, F_OVPGATE_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
                return ret;
            }
	     udelay(1000);
	}

#ifdef CONFIG_MOTO_WLS_CP_REVERSE
    if (wls_mos == MMI_DVCHG_MUX_OTG_OPEN) {
	        //reverse mode
            sc8586_field_write(sc, F_MODE, 6);
            dev_info(sc->dev, "%s:mmi_mux enable cp otg reverse boost mode", __func__);
    	    sc-> otg_delay_mos_config = true;
            ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 1);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);

           sc8586_enable_charge(sc, 1);

            udelay(100);
            ret = sc8586_field_write(sc, F_QB_EN, 1);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set F_QB_EN fail ret=%d", __func__, ret);
                return ret;
            }
    }

    if (wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
        sc8586_field_read(sc, F_MODE, &ret);
		if(ret == 6) {
            ret = sc8586_field_write(sc, F_CP_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux dis cp en ret=%d", __func__, ret);
                return ret;
            }
	       	sc8586_field_write(sc, F_MODE, 0);
            sc-> otg_delay_mos_config = false;
			dev_info(sc->dev, "%s:mmi_mux dis cp otg reverse boost mode", __func__);
            ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
                return ret;
            }
            udelay(100);
            ret = sc8586_field_write(sc, F_QB_EN, 0);
            if (ret < 0) {
                dev_err(sc->dev, "%s:mmi_mux set F_QB_EN fail ret=%d", __func__, ret);
                return ret;
            }
	    }
    }
#endif
    if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
        //reverse mode
        sc8586_field_write(sc, F_MODE, 7);
        sc-> otg_delay_mos_config = true;
        ret = sc8586_field_write(sc, F_ACDRV_MANUAL_EN, 1);
        if (ret < 0) {
            dev_err(sc->dev, "%s:mmi_mux set acdrv manual fail ret=%d", __func__, ret);
            return ret;
        }
        udelay(100);

        ret = sc8586_field_write(sc, F_OVPGATE_EN, 1);
        if (ret < 0) {
            dev_err(sc->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
            return ret;
        }
    }
        ret = sc8586_i2c_read_bytes(sc, SC8586_REG0F, 1, &val);
       if (!ret)
               dev_err(sc->dev, "%s:mmi_mux Reg SC8565_CHRGR_CTRL_5 reg_0xF] = 0x%02X\n", __func__,val);

        ret = sc8586_i2c_read_bytes(sc, SC8586_REG0B, 1, &val);
        if (!ret)
                dev_err(sc->dev, "%s:mmi_mux Reg SC8565_CHRGR_CTRL_1 reg_0xB] = 0x%02X , device_id:%d\n", __func__, val, sc->device_id);

	sc8586_dump_reg(sc);
	return 0;
}

static int sc8586_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	struct sc8586_chip *sc  = charger_get_data(chg_dev);

	return sc858x_config_mux(sc, typec_mos, wls_mos);
}

static int sc8586_operation_get_max_mode(struct charger_device *chg_dev, int *cp_op_mode)
{
	int ret = 0;

	*cp_op_mode = CP_4_1_MODE;
	return ret;
}

static int mtk_sc8586_init_chip(struct charger_device *chg_dev)
{
    struct sc8586_chip *sc = charger_get_data(chg_dev);

    return sc8586_init_device(sc);
}

static const struct charger_ops sc8586_chg_ops = {
	 .enable = mtk_sc8586_enable_chg,
	 .is_enabled = mtk_sc8586_is_chg_enabled,
	 .get_adc = mtk_sc8586_get_adc,
     .get_adc_accuracy = mtk_sc8586_get_adc_accuracy,
	 .set_vbusovp = mtk_sc8586_set_vbusovp,
	 .set_ibusocp = mtk_sc8586_set_ibusocp,
	 .set_vbatovp = mtk_sc8586_set_vbatovp,
	 .set_ibatocp = mtk_sc8586_set_ibatocp,
	 .init_chip = mtk_sc8586_init_chip,
     .is_vbuslowerr = mtk_sc8586_is_vbuslowerr,
     .is_vbushigherr = mtk_sc8586_is_vbushigherr,
	 .set_vbatovp_alarm = mtk_sc8586_set_vbatovp_alarm,
	 .reset_vbatovp_alarm = mtk_sc8586_reset_vbatovp_alarm,
	 .set_vbusovp_alarm = mtk_sc8586_set_vbusovp_alarm,
	 .reset_vbusovp_alarm = mtk_sc8586_reset_vbusovp_alarm,
     .config_mux = sc8586_config_mux,
     .enable_adc = sc858x_enable_adc,
     .set_cp_operation_mode = sc8586_operation_mode_select,
	 .get_cp_operation_max_mode = sc8586_operation_get_max_mode,
};
#endif /*CONFIG_MTK_CLASS*/
/********************mtk charger interface end*************************************************/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
static inline int sc_to_sc8586_adc(enum sc_adc_channel chan)
{
	switch (chan) {
	case SC_ADC_VBUS:
		return ADC_VBUS;
	case SC_ADC_VBAT:
		return ADC_VBAT1;
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


static int sc_sc8586_set_enable(struct dvchg_dev *charger_pump, bool enable)
{
    struct sc8586_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc8586_enable_charge(sc, enable);

    return ret;
}

static int sc_sc8586_get_is_enable(struct dvchg_dev *charger_pump, bool *enable)
{
    struct sc8586_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc8586_check_charge_enabled(sc, enable);

    return ret;
}

static int sc_sc8586_get_status(struct dvchg_dev *charger_pump, uint32_t *status)
{
    struct sc8586_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;
    if (!status)
        return -EINVAL;
    ret = sc8586_get_status(sc, status);

    return ret;
}

static int sc_sc8586_get_adc_value(struct dvchg_dev *charger_pump, enum sc_adc_channel ch, int *value)
{
    struct sc8586_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;
    if (!value)
        return -EINVAL;
    ret = sc8586_get_adc_data(sc, sc_to_sc8586_adc(ch), value);

    return ret;
}

static struct dvchg_ops sc_sc8586_dvchg_ops = {
    .set_enable = sc_sc8586_set_enable,
    .get_status = sc_sc8586_get_status,
    .get_is_enable = sc_sc8586_get_is_enable,
    .get_adc_value = sc_sc8586_get_adc_value,
};
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

/********************creat devices note start*************************************************/
static ssize_t sc8586_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8586_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    uint8_t val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8586");
    for (addr = 0x0; addr <= SC8586_REGMAX; addr++) {
        ret = sc8586_i2c_read_byte(sc, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc8586_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8586_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= SC8586_REGMAX)
        sc8586_i2c_write_byte(sc, (uint8_t)reg, (uint8_t)val);

    return count;
}

#if 0
static ssize_t sc8586_test_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    struct sc8586_chip *sc = dev_get_drvdata(dev);
    int ret;
	int val;
    int reg_val;
    int i;
    ret = sscanf(buf, "%d", &val);
    if (ret < 0) {
        sc8586_err("get parameters fail\n");
        return -EINVAL;
    }
    switch (val) {
    case 1: /*force dpdm func test*/
        sc8586_set_force_dpdm(sc);
        for (i = 0; i < 10; i++) {
            reg_val = sc8586_get_vbus_stat(sc);
            sc8586_err("get vbus stat = %d\n", reg_val);
            msleep(100);
        }
        break;
    case 2: /*dpdm drive func test*/
        for (i = 0; i < 10; i++) {
            sc8586_set_dp_drive(sc, SC8586_DPDM_3P3V);
            sc8586_set_dm_drive(sc, SC8586_DPDM_3P3V);
            msleep(10);
            sc8586_set_dp_drive(sc, SC8586_DPDM_HIZ);
            sc8586_set_dm_drive(sc, SC8586_DPDM_HIZ);
            msleep(10);
        }
        break;
    case 3: /*scp send func test*/
        sc8586_set_dp_drive(sc, SC8586_DPDM_0P6V);
        sc8586_set_dm_drive(sc, SC8586_DPDM_HIZ);
        msleep(1200);
        //sc8586_scp_write_tx_fifo(sc, 0xB);
        //sc8586_scp_start_transaction(sc);
        break;
    default:
        break;
    }

    return count;
}
#endif
static DEVICE_ATTR(registers, 0660, sc8586_show_registers, sc8586_store_register);
//static DEVICE_ATTR(sc8586_test, 0660, NULL, sc8586_test_store);

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	bool enable;
	struct sc8586_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8586: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = sc8586_check_charge_enabled(sc, &enable);
	if (ret < 0) {
		pr_err("sc8586: sc8586_is_chg_en not valid\n");
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
	struct sc8586_chip *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8586 chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc8586_enable_charge(sc, enable);
	if (ret < 0) {
		pr_err("sc8586 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("sc8586  %s charging \n",
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static void sc8586_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
    //device_create_file(dev, &dev_attr_sc8586_test);
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
}
/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
#ifdef CONFIG_MTK_CLASS
static inline int status_reg_to_charger(enum sc8586_notify notify)
{
	switch (notify) {
    case SC8586_NOTIFY_IBUSOCP:
		return CHARGER_DEV_NOTIFY_IBUSOCP;
    case SC8586_NOTIFY_VBUSOVP:
		return CHARGER_DEV_NOTIFY_VBUS_OVP;
    case SC8586_NOTIFY_IBATOCP:
		return CHARGER_DEV_NOTIFY_IBATOCP;
    case SC8586_NOTIFY_VBATOVP:
		return CHARGER_DEV_NOTIFY_BAT_OVP;
    case SC8586_NOTIFY_VOUTOVP:
		return CHARGER_DEV_NOTIFY_VOUTOVP;
	default:
        return -EINVAL;
		break;
	}
	return -EINVAL;
}
#endif /*CONFIG_MTK_CLASS*/
__maybe_unused
static void sc8586_dump_check_cp_fault_status(struct sc8586_chip *sc)
{
    u8 flag = 0;
    int i,j,k;
#ifdef CONFIG_MTK_CLASS
    int noti;
#endif /*CONFIG_MTK_CLASS*/
    if (!sc)
        return;

    for (i = 0; i <= 0x2B; i++) {
        sc8586_i2c_read_bytes(sc, i, 1, &flag);
        sc8586_info( "%s cp reg[0x%02x] = 0x%02x\n", __func__, i, flag);
        for (k=0; k < ARRAY_SIZE(cp_intr_flag); k++) {
            if (cp_intr_flag[k].reg == i){
                for (j=0; j <  cp_intr_flag[k].len; j++) {
                    if (flag & cp_intr_flag[k].bit[j].mask) {
                        sc8586_info("trigger :%s\n",cp_intr_flag[k].bit[j].name);
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

__maybe_unused
static void cps2043_dump_check_cp_fault_status(struct sc8586_chip *sc)
{
    u8 flag = 0;
    int i,j,k;
#ifdef CONFIG_MTK_CLASS
    int noti;
#endif /*CONFIG_MTK_CLASS*/
    if (!sc)
        return;
    for (i = 0; i <= 0x2B; i++) {
        sc8586_i2c_read_bytes(sc, i, 1, &flag);
        sc8586_err( "%s cp reg[0x%02x] = 0x%02x\n", __func__, i, flag);
        for (k=0; k < ARRAY_SIZE(cps_cp_intr_flag); k++) {
            if (cps_cp_intr_flag[k].reg == i){
                for (j=0; j <  cps_cp_intr_flag[k].len; j++) {
                    if (flag & cps_cp_intr_flag[k].bit[j].mask) {
                        sc8586_err("trigger :%s\n",cps_cp_intr_flag[k].bit[j].name);
#ifdef CONFIG_MTK_CLASS
                        noti = status_reg_to_charger(cps_cp_intr_flag[k].bit[j].notify);
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

static irqreturn_t sc8586_irq_handler(int irq, void *data)
{
    struct sc8586_chip *sc = data;
    int scp_block_enable = 0;
    int dpdm_block_enable = 0;

    sc8586_info("INT OCCURED\n");

    if (sc->device_id == CPS2043_DEVICE_ID)
        cps2043_dump_check_cp_fault_status(sc);
    else {
        sc8586_dump_check_cp_fault_status(sc);
        //scp_block_enable = sc8586_get_scp_enable(sc);
        if (scp_block_enable)
            sc8586_dump_check_scp_fault_status(sc);

        dpdm_block_enable = sc8586_get_dpdm_enable(sc);
        if (dpdm_block_enable)
            sc8586_dump_check_dpdm_fault_status(sc);
    }
    power_supply_changed(sc->psy);

    return IRQ_HANDLED;
}

static int sc8586_register_interrupt(struct sc8586_chip *sc)
{
    int ret = 0;
    if (!sc)
        return -EINVAL;
    if (gpio_is_valid(sc->irq_gpio)) {
        ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN, "sc8586_irq");
        if (ret) {
            sc8586_err("failed to request sc8586_irq\n");
            return -EINVAL;
        }
        sc->irq = gpio_to_irq(sc->irq_gpio);
        if (sc->irq < 0) {
            sc8586_err("failed to gpio_to_irq\n");
            return -EINVAL;
        }
    } else {
        sc8586_err("irq gpio not provided\n");
        return -EINVAL;
    }

    if (sc->irq) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
                NULL, sc8586_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                sc8586_irq_name[sc->mode], sc);

        if (ret < 0) {
            sc8586_err("request irq for irq=%d failed, ret =%d\n",
                            sc->irq, ret);
            return ret;
        }
        enable_irq_wake(sc->irq);
    }

    return ret;
}
/********************interrupte end*************************************************/


/************************psy start**************************************/
static enum power_supply_property sc8586_charger_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
};

static int sc8586_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8586_chip *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = sc8586_get_adc_data(sc, ADC_VBUS, &result);
        if (ret >= 0)
            sc->vbus_volt = result;
        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8586_get_adc_data(sc, ADC_IBUS, &result);
        if (ret >= 0)
            sc->ibus_curr = result;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8586_get_adc_data(sc, ADC_VBAT1, &result);
        if (ret >= 0)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = sc8586_get_adc_data(sc, ADC_IBAT, &result);
        if (ret >= 0)
            sc->ibat_curr = result;
        val->intval = sc->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8586_get_adc_data(sc, ADC_TDIE, &result);
        if (ret >= 0)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int sc8586_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    switch (prop) {

    default:
        return -EINVAL;
    }

    return 0;
}

static int sc8586_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    return 0;
}

static int sc8586_psy_register(struct sc8586_chip *sc)
{
    if (!sc)
        return -EINVAL;
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc8586_psy_name[sc->mode];

    sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    sc->psy_desc.properties = sc8586_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8586_charger_props);
    sc->psy_desc.get_property = sc8586_charger_get_property;
    sc->psy_desc.set_property = sc8586_charger_set_property;
    sc->psy_desc.property_is_writeable = sc8586_charger_is_writeable;

    sc->psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->psy)) {
        sc8586_err( "%s failed to register psy\n", __func__);
        return PTR_ERR(sc->psy);
    }

    sc8586_info( "%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}


/************************psy end**************************************/

static int sc8586_set_work_mode(struct sc8586_chip *sc, int mode)
{
    if (!sc)
        return -EINVAL;
    sc->mode = mode;

    sc8586_err("work mode is %s\n", sc->mode == SC8586_STANDALONG
        ? "standalone" : (sc->mode == SC8586_MASTER ? "master" : "slave"));

    return 0;
}

static int sc8586_parse_dt(struct sc8586_chip *sc, struct device *dev)
{
    if (!dev || !sc)
        return -EINVAL;
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc8586,vbat-ovp-dis", &(sc->cfg.vbat_ovp_dis)},
        {"sc,sc8586,vbat-ovp", &(sc->cfg.vbat_ovp)},
        {"sc,sc8586,ibat-ocp-dis", &(sc->cfg.ibat_ocp_dis)},
        {"sc,sc8586,ibat-ocp", &(sc->cfg.ibat_ocp)},
        {"sc,sc8586,vusb-ovp-dis", &(sc->cfg.vusb_ovp_dis)},
        {"sc,sc8586,vusb-ovp", &(sc->cfg.vusb_ovp)},
        {"sc,sc8586,vbus-ovp-dis", &(sc->cfg.vbus_ovp_dis)},
        {"sc,sc8586,vbus-ovp", &(sc->cfg.vbus_ovp)},
        {"sc,sc8586,vout-ovp-dis", &(sc->cfg.vout_ovp_dis)},
        {"sc,sc8586,vout-ovp", &(sc->cfg.vout_ovp)},
        {"sc,sc8586,ibus-ocp-dis", &(sc->cfg.ibus_ocp_dis)},
        {"sc,sc8586,ibus-ocp", &(sc->cfg.ibus_ocp)},
        {"sc,sc8586,ibus-ucp-fall-dis", &(sc->cfg.ibus_ucp_fall_dis)},
        {"sc,sc8586,ibus-ucp-fall", &(sc->cfg.ibus_ucp_fall)},
        {"sc,sc8586,pmid2out-ovp-dis", &(sc->cfg.pmid2out_ovp_dis)},
        {"sc,sc8586,pmid2out-ovp", &(sc->cfg.pmid2out_ovp)},
        {"sc,sc8586,pmid2out-uvp-dis", &(sc->cfg.pmid2out_uvp_dis)},
        {"sc,sc8586,pmid2out-uvp", &(sc->cfg.pmid2out_uvp)},
        {"sc,sc8586,fsw-set", &(sc->cfg.fsw_set)},
        {"sc,sc8586,ss-timeout", &(sc->cfg.ss_timeout)},
        {"sc,sc8586,wd-timeout", &(sc->cfg.wd_timeout)},
        {"sc,sc8586,ibat-sns-r", &(sc->cfg.ibat_sns_r)},
        {"sc,sc8586,mode", &(sc->cfg.mode)},
        {"sc,sc8586,tshut-dis", &(sc->cfg.tshut_dis)},
        {"sc,sc8686,vbat2-adc-en", &(sc->cfg.vbat2_adc_en)},
        {"sc,sc8586,scp-block-en", &(sc->cfg.scp_block_en)},
        {"sc,sc8586,dpdm-block-en", &(sc->cfg.dpdm_block_en)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            sc8586_err( "can not read %s \n", props[i].name);
            return ret;
        }
    }

    sc->irq_gpio = of_get_named_gpio(np, "sc8586,intr_gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        sc8586_err("fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }
#ifdef CONFIG_MTK_CLASS
    if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        sc8586_err("no charger name\n");
    }
#elif defined(CONFIG_MTK_CHARGER_V4P19)
    if (of_property_read_string(np, "charger_name_v4_19", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        sc8586_err("no charger name\n");
    }
#endif /*CONFIG_MTK_CHARGER_V4P19*/

    return 0;
}

static struct of_device_id sc8586_charger_match_table[] = {
    {   .compatible = "sc,sc8586-standalone",
        .data = &sc8586_mode_data[SC8586_STANDALONG], },
    {   .compatible = "sc,sc8586-master",
        .data = &sc8586_mode_data[SC8586_MASTER], },
    {   .compatible = "sc,sc8586-slave",
        .data = &sc8586_mode_data[SC8586_SLAVE], },
    {},
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc8586_charger_probe(struct i2c_client *client)
#else
static int sc8586_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
#endif
{
    struct sc8586_chip *sc;
    const struct of_device_id *match;
    if (!client)
        return -EINVAL;
    struct device_node *node = client->dev.of_node;
    int ret;

    sc8586_err("%s (%s)\n", __func__, SC8586_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc8586_chip), GFP_KERNEL);
    if (!sc) {
        ret = -ENOMEM;
        goto err_kzalloc;
    }

    sc->dev = &client->dev;
    sc->client = client;

    i2c_set_clientdata(client, sc);

    mutex_init(&sc->field_rw_lock);

    ret = sc8586_detect_device(sc);
    if (ret < 0) {
        sc8586_err( "%s detect device fail\n", __func__);
        goto err_detect_dev;
    }

    sc8586_create_device_node(&(client->dev));

    match = of_match_node(sc8586_charger_match_table, node);
    if (match == NULL) {
        sc8586_err( "device tree match not found!\n");
        goto err_match_node;
    }

    ret = sc8586_set_work_mode(sc, *(int *)match->data);
    if (ret) {
        sc8586_err("Fail to set work mode!\n");
        goto err_set_mode;
    }

    ret = sc8586_parse_dt(sc, &client->dev);
    if (ret < 0) {
        sc8586_err( "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc8586_init_device(sc);
    if (ret < 0) {
        sc8586_err( "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = sc8586_psy_register(sc);
    if (ret < 0) {
        sc8586_err( "%s psy register failed(%d)\n", __func__, ret);
        goto err_register_psy;
    }

    ret = sc8586_register_interrupt(sc);
    if (ret < 0) {
        sc8586_err( "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

#ifdef CONFIG_MTK_CLASS
    sc->chg_dev = charger_device_register(sc->chg_dev_name,
					      &client->dev, sc,
					      &sc8586_chg_ops,
					      &sc8586_chg_props);
	if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		sc8586_err("Fail to register charger!\n");
        goto err_register_mtk_charger;
	}
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    sc->charger_pump = dvchg_register("sc_dvchg",
                             sc->dev, &sc_sc8586_dvchg_ops, sc);
    if (IS_ERR_OR_NULL(sc->charger_pump)) {
		ret = PTR_ERR(sc->charger_pump);
		sc8586_err("Fail to register charger!\n");
        goto err_register_sc_charger;
	}
#endif /* CONFIG_SOUTHCHIP_DVCHG_CLASS */

    sc8586_enable_adc(sc, false);
    if (sc->mode == SC8586_MASTER) {
        sc8586_err( "sc8586[master] probe successfully!\n");
    } else if (sc->mode == SC8586_SLAVE) {
        sc8586_err( "sc8586[slave] probe successfully!\n");
    } else if (sc->mode == SC8586_STANDALONG) {
        sc8586_err( "sc8586[standalone] probe successfully!\n");
    }
    return 0;

err_register_psy:
err_register_irq:
#ifdef CONFIG_MTK_CLASS
err_register_mtk_charger:
#endif /*CONFIG_MTK_CLASS*/
#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
err_register_sc_charger:
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/
err_init_device:
    //power_supply_unregister(sc->psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
    //devm_kfree(&client->dev, sc);
err_kzalloc:
    sc8586_err("sc8586 probe fail\n");
    return ret;
}

static void sc8586_charger_remove(struct i2c_client *client)
{
    struct sc8586_chip *sc = i2c_get_clientdata(client);

    if (!sc)
        return;

    if (sc->psy)
        power_supply_unregister(sc->psy);
    mutex_destroy(&sc->field_rw_lock);
    //devm_kfree(&client->dev, sc);

}

#ifdef CONFIG_PM_SLEEP
static int sc8586_suspend(struct device *dev)
{
    struct sc8586_chip *sc = dev_get_drvdata(dev);

    sc8586_info( "Suspend successfully!");
    if (device_may_wakeup(dev))
        enable_irq_wake(sc->irq);
    disable_irq(sc->irq);

    return 0;
}
static int sc8586_resume(struct device *dev)
{
    struct sc8586_chip *sc = dev_get_drvdata(dev);

    sc8586_info( "Resume successfully!");
    if (device_may_wakeup(dev))
        disable_irq_wake(sc->irq);
    enable_irq(sc->irq);

    return 0;
}

static const struct dev_pm_ops sc8586_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc8586_suspend, sc8586_resume)
};
#endif

static struct i2c_driver sc8586_charger_driver = {
    .driver     = {
        .name   = "sc8586",
        .owner  = THIS_MODULE,
        .of_match_table = sc8586_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc8586_pm,
#endif
    },
    .probe      = sc8586_charger_probe,
    .remove     = sc8586_charger_remove,
};

module_i2c_driver(sc8586_charger_driver);

MODULE_DESCRIPTION("SC SC8586 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <boyu-wen@southchip.com>");

