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

#ifdef CONFIG_MTK_CLASS
#include "charger_class.h"

#ifdef CONFIG_MTK_CHARGER_V4P19
#include "mtk_charger_intf.h"
#endif /*CONFIG_MTK_CHARGER_V4P19*/

#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
#include "dvchg_class.h"
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

#include "sc8586_charger.h"

static struct intr_flag dpdm_intr_flag[] = {
    { .reg = 0x37, .len = 5, .bit = {
                {.mask = BIT(0), .name = "dm ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "dp ovp flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "input det done flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "fc3 plus done flag", .notify = SC8586_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "fc3 minus done flag", .notify = SC8586_NOTIFY_OTHER},
                },
    },
};

__maybe_unused
void sc8586_dump_check_dpdm_fault_status(struct sc8586_chip *sc)
{
    int ret;
    u8 flag = 0;
    int i,j,k;

    for (i = 0x34; i <= 0x3C; i++) {
        ret = sc8586_i2c_read_bytes(sc, i, 1, &flag);
        sc8586_err( "%s dpdm reg[0x%02x] = 0x%02x\n", __func__, i, flag);
        for (k=0; k < ARRAY_SIZE(dpdm_intr_flag); k++) {
            if (dpdm_intr_flag[k].reg == i){
                for (j=0; j < dpdm_intr_flag[k].len; j++) {
                    if (flag & dpdm_intr_flag[k].bit[j].mask) {
                        sc8586_err("trigger :%s\n", dpdm_intr_flag[k].bit[j].name);
                    }
                }
            }
        }
    }
}

__maybe_unused
int sc8586_set_dpdm_enable(struct sc8586_chip *sc, bool enable)
{
    int ret;

    if (enable) {
        ret = sc8586_field_write(sc, F_DPDM_EN, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    } else {
        ret = sc8586_field_write(sc, F_DPDM_EN, 0);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    }

    return ret;
}

__maybe_unused
int sc8586_get_dpdm_enable(struct sc8586_chip *sc)
{
    int ret;
    int val;

    ret = sc8586_field_read(sc, F_DPDM_EN, &val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
        return ret;
    }
    return val;
}

__maybe_unused
int sc8586_set_force_dpdm(struct sc8586_chip *sc)
{
    int ret;

    ret = sc8586_field_write(sc, F_FORCE_DPDM, 1);
    if (ret < 0) {
        sc8586_err("%s fail(%d)\n", __func__, ret);
    }
    return ret;
}

__maybe_unused
int sc8586_set_auto_dpdm(struct sc8586_chip *sc, bool enable)
{
    int ret;

    if (enable) {
        ret = sc8586_field_write(sc, F_AUTO_DPDM, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    } else {
        ret = sc8586_field_write(sc, F_AUTO_DPDM, 0);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    }

    return ret;
}

__maybe_unused
int sc8586_set_hvdcp_enable(struct sc8586_chip *sc, bool enable)
{
    int ret;

    if (enable) {
        ret = sc8586_field_write(sc, F_HVDCP_EN, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    } else {
        ret = sc8586_field_write(sc, F_HVDCP_EN, 0);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    }

    return ret;
}

__maybe_unused
int sc8586_set_fc_func_enable(struct sc8586_chip *sc, bool enable)
{
    int ret;

    if (enable) {
        ret = sc8586_field_write(sc, F_FC_EN, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    } else {
        ret = sc8586_field_write(sc, F_FC_EN, 0);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    }

    return ret;
}

__maybe_unused
int sc8586_set_dp_drive(struct sc8586_chip *sc, enum sc8586_dpdm_drive dp_val)
{
    int ret;

    ret = sc8586_field_write(sc, F_DP_DRIVE, dp_val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
    }

    return ret;
}

__maybe_unused
int sc8586_set_dm_drive(struct sc8586_chip *sc, enum sc8586_dpdm_drive dm_val)
{
    int ret;

    ret = sc8586_field_write(sc, F_DM_DRIVE, dm_val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
    }

    return ret;
}

__maybe_unused
int sc8586_set_fc2_ctrl(struct sc8586_chip *sc, enum sc8586_fc2_ctrl fc2_ctrl)
{
    int ret;

    ret = sc8586_field_write(sc, F_FC2_SET, fc2_ctrl);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
    }

    return ret;
}

__maybe_unused
int sc8586_set_fc3_increase(struct sc8586_chip *sc, int target_volt_mv)
{
    int ret;
    int step = 0;
    int i;

    if (!sc)
        return -EINVAL;

    step = (target_volt_mv - 5000) / 200;

    for (i = 0; i < step; i++) {
        ret = sc8586_field_write(sc, F_FC3_PLUS, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
            return ret;
        }
        msleep(5);
    }
    sc->fc3_volt_gear = target_volt_mv;

    return ret;
}

__maybe_unused
int sc8586_set_fc3_decrease(struct sc8586_chip *sc, int target_volt_mv)
{
    int ret;
    int step = 0;
    int i;

    if (!sc)
        return -EINVAL;

    step = (sc->fc3_volt_gear - target_volt_mv) / 200;

    for (i = 0; i < step; i++) {
        ret = sc8586_field_write(sc, F_FC3_MINUS, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
            return ret;
        }
        msleep(5);
    }

    return ret;
}

__maybe_unused
int sc8586_get_dp_stat(struct sc8586_chip *sc)
{
    int ret;
    uint8_t val = 0;
    int dp_status;

    ret = sc8586_i2c_read_byte(sc, 0x39, &val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    dp_status = (int)val;

    return dp_status;
}

__maybe_unused
int sc8586_get_dm_stat(struct sc8586_chip *sc)
{
    int ret;
    uint8_t val = 0;
    int dm_status;

    ret = sc8586_i2c_read_byte(sc, 0x3a, &val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    dm_status = (int)val;

    return dm_status;
}

__maybe_unused
int sc8586_set_dpdm_polling(struct sc8586_chip *sc, bool enable)
{
    int ret;

    if (enable) {
        ret = sc8586_field_write(sc, F_DPDM_POLLING_EN, 1);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    } else {
        ret = sc8586_field_write(sc, F_DPDM_POLLING_EN, 0);
        if (ret < 0) {
            sc8586_err( "%s fail(%d)\n", __func__, ret);
        }
    }

    return ret;
}

__maybe_unused
int sc8586_get_vbus_stat(struct sc8586_chip *sc)
{
    int ret;
    int reg_val;
    enum sc8586_dpdm_vbus_stat vbus_stat;

    ret = sc8586_field_read(sc, F_VBUS_STAT, &reg_val);
    if (ret < 0) {
        sc8586_err( "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    vbus_stat = reg_val;

    return vbus_stat;
}

