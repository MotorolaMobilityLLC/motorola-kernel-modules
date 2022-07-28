/*
* SC8551 battery charging driver
*/

#define pr_fmt(fmt)	"[sc8551] %s: " fmt, __func__

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

#include "sc8551_reg.h"
#include <charger_class.h>

typedef enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VAC,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_RSV0,
    ADC_RSV1,
    ADC_TDIE,
    ADC_MAX_NUM,
}ADC_CH;

#define SC8551_ROLE_STANDALONE  0
#define SC8551_ROLE_SLAVE       1
#define SC8551_ROLE_MASTER      2

enum {
    SC8551_STDALONE,
    SC8551_SLAVE,
    SC8551_MASTER,
};

static int sc8551_mode_data[] = {
    [SC8551_STDALONE] = SC8551_STDALONE,
    [SC8551_MASTER] = SC8551_ROLE_MASTER,
    [SC8551_SLAVE] = SC8551_ROLE_SLAVE,
};

#define sc_err(fmt, ...)								\
do {											\
    if (sc->mode == SC8551_ROLE_MASTER)						\
        printk(KERN_ERR "[sc8551-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else if (sc->mode == SC8551_ROLE_SLAVE)					\
        printk(KERN_ERR "[sc8551-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else										\
        printk(KERN_ERR "[sc8551-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define sc_info(fmt, ...)								\
do {											\
    if (sc->mode == SC8551_ROLE_MASTER)						\
        printk(KERN_INFO "[sc8551-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else if (sc->mode == SC8551_ROLE_SLAVE)					\
        printk(KERN_INFO "[sc8551-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else										\
        printk(KERN_INFO "[sc8551-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define sc_dbg(fmt, ...)								\
do {											\
    if (sc->mode == SC8551_ROLE_MASTER)						\
        printk(KERN_DEBUG "[sc8551-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else if (sc->mode == SC8551_ROLE_SLAVE)					\
        printk(KERN_DEBUG "[sc8551-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
    else										\
        printk(KERN_DEBUG "[sc8551-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);



/*end*/

struct sc8551_cfg {
    bool bat_ovp_disable;
    bool bat_ocp_disable;

    int bat_ovp_th;
    int bat_ocp_th;

    bool bus_ocp_disable;

    int bus_ovp_th;
    int bus_ocp_th;

    int ac_ovp_th;

    bool die_therm_disable;

    int die_therm_th; /*in degC*/

    int sense_r_mohm;
};

struct sc8551 {
    struct device *dev;
    struct i2c_client *client;

    int part_no;
    int revision;

    int mode;

    struct mutex data_lock;
    struct mutex i2c_rw_lock;
    struct mutex irq_complete;

    bool irq_waiting;
    bool irq_disabled;
    bool resume_completed;


    bool usb_present;
    bool charge_enabled;	/* Register bit status */

    int  vbus_error;

    int irq_gpio;
    int irq;

    /* ADC reading */
    int vbat_volt;
    int vbus_volt;
    int vout_volt;
    int vac_volt;

    int ibat_curr;
    int ibus_curr;

    int die_temp;

    bool vbat_reg;
    bool ibat_reg;
    bool sw_chg_en;

    struct sc8551_cfg *cfg;

    int skip_writes;
    int skip_reads;

    struct sc8551_platform_data *platform_data;

    struct delayed_work monitor_work;

    struct dentry *debug_root;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *fc2_psy;
	
  	int reg_addr;
  	int reg_data;
  	struct charger_device *chg_dev;
  	struct charger_properties chg_prop;
};

/************************************************************************/
static int __sc8551_read_byte(struct sc8551 *sc, u8 reg, u8 *data)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(sc->client, reg);
    if (ret < 0) {
        sc_err("i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8) ret;

    return 0;
}

static int __sc8551_write_byte(struct sc8551 *sc, int reg, u8 val)
{
    s32 ret;

    ret = i2c_smbus_write_byte_data(sc->client, reg, val);
    if (ret < 0) {
        sc_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
            val, reg, ret);
        return ret;
    }
    return 0;
}

static int sc8551_read_byte(struct sc8551 *sc, u8 reg, u8 *data)
{
    int ret;

    if (sc->skip_reads) {
        *data = 0;
        return 0;
    }

    mutex_lock(&sc->i2c_rw_lock);
    ret = __sc8551_read_byte(sc, reg, data);
    mutex_unlock(&sc->i2c_rw_lock);

    return ret;
}

static int sc8551_write_byte(struct sc8551 *sc, u8 reg, u8 data)
{
    int ret;

    if (sc->skip_writes)
        return 0;

    mutex_lock(&sc->i2c_rw_lock);
    ret = __sc8551_write_byte(sc, reg, data);
    mutex_unlock(&sc->i2c_rw_lock);

    return ret;
}

static int sc8551_update_bits(struct sc8551*sc, u8 reg,
                    u8 mask, u8 data)
{
    int ret;
    u8 tmp;

    if (sc->skip_reads || sc->skip_writes)
        return 0;

    mutex_lock(&sc->i2c_rw_lock);
    ret = __sc8551_read_byte(sc, reg, &tmp);
    if (ret) {
        sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);
        goto out;
    }

    tmp &= ~mask;
    tmp |= data & mask;

    ret = __sc8551_write_byte(sc, reg, tmp);
    if (ret)
        sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
    mutex_unlock(&sc->i2c_rw_lock);
    return ret;
}

/*********************************************************************/

static int sc8551_enable_charge(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_CHG_ENABLE;
    else
        val = SC8551_CHG_DISABLE;

    val <<= SC8551_CHG_EN_SHIFT;

    sc_err("sc8551 charger %s\n", enable == false ? "disable" : "enable");
    return sc8551_update_bits(sc, SC8551_REG_0C,
                SC8551_CHG_EN_MASK, val);
}

static int sc8551_check_charge_enabled(struct sc8551 *sc, bool *enable)
{
    int ret;
    u8 val, val1;

    ret = sc8551_read_byte(sc, SC8551_REG_0C, &val);
    if (!ret) {
        ret = sc8551_read_byte(sc, SC8551_REG_0A, &val1);
        if (!ret) {
            if ((val & SC8551_CHG_EN_MASK) && (val1 & SC8551_CONV_SWITCHING_STAT_MASK)) {
                *enable = true;
                return ret;
            }
        }
    }
    *enable = false;
    return ret;
}

static int sc8551_enable_wdt(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_WATCHDOG_ENABLE;
    else
        val = SC8551_WATCHDOG_DISABLE;

    val <<= SC8551_WATCHDOG_DIS_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_0B,
                SC8551_WATCHDOG_DIS_MASK, val);
}

static int sc8551_set_wdt(struct sc8551 *sc, int ms)
{
    u8 val;

    if (ms == 500)
        val = SC8551_WATCHDOG_0P5S;
    else if (ms == 1000)
        val = SC8551_WATCHDOG_1S;
    else if (ms == 5000)
        val = SC8551_WATCHDOG_5S;
    else if (ms == 30000)
        val = SC8551_WATCHDOG_30S;
    else
        val = SC8551_WATCHDOG_30S;

    val <<= SC8551_WATCHDOG_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_0B,
                SC8551_WATCHDOG_MASK, val);
}

static int sc8551_set_reg_reset(struct sc8551 *sc)
{
    u8 val = 1;

    val = SC8551_REG_RST_ENABLE;

    val <<= SC8551_REG_RST_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_0B,
                SC8551_REG_RST_MASK, val);
}

static int sc8551_enable_batovp(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_BAT_OVP_ENABLE;
    else
        val = SC8551_BAT_OVP_DISABLE;

    val <<= SC8551_BAT_OVP_DIS_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_00,
                SC8551_BAT_OVP_DIS_MASK, val);
}

static int sc8551_set_batovp_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold < SC8551_BAT_OVP_BASE)
        threshold = SC8551_BAT_OVP_BASE;

    val = (threshold - SC8551_BAT_OVP_BASE) / SC8551_BAT_OVP_LSB;

    val <<= SC8551_BAT_OVP_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_00,
                SC8551_BAT_OVP_MASK, val);
}

static int sc8551_enable_batocp(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_BAT_OCP_ENABLE;
    else
        val = SC8551_BAT_OCP_DISABLE;

    val <<= SC8551_BAT_OCP_DIS_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_02,
                SC8551_BAT_OCP_DIS_MASK, val);
}

static int sc8551_set_batocp_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold < SC8551_BAT_OCP_BASE)
        threshold = SC8551_BAT_OCP_BASE;

    val = (threshold - SC8551_BAT_OCP_BASE) / SC8551_BAT_OCP_LSB;

    val <<= SC8551_BAT_OCP_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_02,
                SC8551_BAT_OCP_MASK, val);
}

static int sc8551_set_busovp_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold < SC8551_BUS_OVP_BASE)
        threshold = SC8551_BUS_OVP_BASE;

    val = (threshold - SC8551_BUS_OVP_BASE) / SC8551_BUS_OVP_LSB;

    val <<= SC8551_BUS_OVP_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_06,
                SC8551_BUS_OVP_MASK, val);
}

static int sc8551_enable_busocp(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_BUS_OCP_ENABLE;
    else
        val = SC8551_BUS_OCP_DISABLE;

    val <<= SC8551_BUS_OCP_DIS_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_08,
                SC8551_BUS_OCP_DIS_MASK, val);
}


static int sc8551_set_busocp_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold < SC8551_BUS_OCP_BASE)
        threshold = SC8551_BUS_OCP_BASE;

    val = (threshold - SC8551_BUS_OCP_BASE) / SC8551_BUS_OCP_LSB;

    val <<= SC8551_BUS_OCP_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_08,
                SC8551_BUS_OCP_MASK, val);
}

static int sc8551_set_acovp_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold < SC8551_AC_OVP_BASE)
        threshold = SC8551_AC_OVP_BASE;

    if (threshold == SC8551_AC_OVP_6P5V)
        val = 0x07;
    else
        val = (threshold - SC8551_AC_OVP_BASE) /  SC8551_AC_OVP_LSB;

    val <<= SC8551_AC_OVP_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_05,
                SC8551_AC_OVP_MASK, val);
}

static int sc8551_set_vdrop_th(struct sc8551 *sc, int threshold)
{
    u8 val;

    if (threshold == 300)
        val = SC8551_VDROP_THRESHOLD_300MV;
    else
        val = SC8551_VDROP_THRESHOLD_400MV;

    val <<= SC8551_VDROP_THRESHOLD_SET_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_05,
                SC8551_VDROP_THRESHOLD_SET_MASK,
                val);
}

static int sc8551_set_vdrop_deglitch(struct sc8551 *sc, int us)
{
    u8 val;

    if (us == 8)
        val = SC8551_VDROP_DEGLITCH_8US;
    else
        val = SC8551_VDROP_DEGLITCH_5MS;

    val <<= SC8551_VDROP_DEGLITCH_SET_SHIFT;

   return sc8551_update_bits(sc, SC8551_REG_05,
                SC8551_VDROP_DEGLITCH_SET_MASK,
                val);
}

static int sc8551_enable_adc(struct sc8551 *sc, bool enable)
{
    u8 val;
	
	dev_err(sc->dev, "%s enable %d\n", __func__, enable);

    if (enable)
        val = SC8551_ADC_ENABLE;
    else
        val = SC8551_ADC_DISABLE;

    val <<= SC8551_ADC_EN_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_14,
                SC8551_ADC_EN_MASK, val);
}

static int sc8551_set_adc_scanrate(struct sc8551 *sc, bool oneshot)
{
    u8 val;

    if (oneshot)
        val = SC8551_ADC_RATE_ONESHOT;
    else
        val = SC8551_ADC_RATE_CONTINOUS;

    val <<= SC8551_ADC_RATE_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_14,
                SC8551_ADC_EN_MASK, val);
}


#define ADC_REG_BASE SC8551_REG_16
static int sc8551_get_adc_data(struct sc8551 *sc, int channel,  int *result)
{
    int ret;
    u8 val_l, val_h;
    u32 val;

    if(channel >= ADC_MAX_NUM) return 0;

    ret = sc8551_read_byte(sc, ADC_REG_BASE + (channel << 1), &val_h);
    ret = sc8551_read_byte(sc, ADC_REG_BASE + (channel << 1) + 1, &val_l);

    if (ret < 0)
        return ret;
    val = (val_h << 8) | val_l;
	
	sc_err("sc8551_get_adc_data before: channel=%d, val=%d\n", channel, val);

    if(channel == ADC_IBUS) 			val = val * /*1000 * 15625/10000*/15625/10;
    else if(channel == ADC_VBUS)		val = val * /*1000 * 375/100*/3750;
    else if(channel == ADC_VAC)			val = val * 5;
    else if(channel == ADC_VOUT)		val = val * /*1000 * 125 / 100*/1250;
    else if(channel == ADC_VBAT)		val = val * /*1000 * 125/100*/1250;
    else if(channel == ADC_IBAT)		val = val * 3125/1000 ;
    else if(channel == ADC_TDIE)		val = val * 5/10;

    *result = val;
	
	sc_err("sc8551_get_adc_data after: channel=%d, val=%d\n", channel, val);

    return ret;
}

static int sc8551_set_adc_scan(struct sc8551 *sc, int channel, bool enable)
{
    u8 reg;
    u8 mask;
    u8 shift;
    u8 val;

    if (channel > ADC_MAX_NUM)
        return -EINVAL;

    if (channel == ADC_IBUS) {
        reg = SC8551_REG_14;
        shift = SC8551_IBUS_ADC_DIS_SHIFT;
        mask = SC8551_IBUS_ADC_DIS_MASK;
    } else {
        reg = SC8551_REG_15;
        shift = 8 - channel;
        mask = 1 << shift;
    }

    if (enable)
        val = 0 << shift;
    else
        val = 1 << shift;

    return sc8551_update_bits(sc, reg, mask, val);
}

static int sc8551_set_sense_resistor(struct sc8551 *sc, int r_mohm)
{
    u8 val;

    if (r_mohm == 2)
        val = SC8551_SET_IBAT_SNS_RES_2MHM;
    else if (r_mohm == 5)
        val = SC8551_SET_IBAT_SNS_RES_5MHM;
    else
        return -EINVAL;

    val <<= SC8551_SET_IBAT_SNS_RES_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_2B,
                SC8551_SET_IBAT_SNS_RES_MASK,
                val);
}

static int sc8551_enable_regulation(struct sc8551 *sc, bool enable)
{
    u8 val;

    if (enable)
        val = SC8551_EN_REGULATION_ENABLE;
    else
        val = SC8551_EN_REGULATION_DISABLE;

    val <<= SC8551_EN_REGULATION_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_2B,
                SC8551_EN_REGULATION_MASK,
                val);
}

static int sc8551_set_ss_timeout(struct sc8551 *sc, int timeout)
{
    u8 val;

    switch (timeout) {
    case 0:
        val = SC8551_SS_TIMEOUT_DISABLE;
        break;
    case 12:
        val = SC8551_SS_TIMEOUT_12P5MS;
        break;
    case 25:
        val = SC8551_SS_TIMEOUT_25MS;
        break;
    case 50:
        val = SC8551_SS_TIMEOUT_50MS;
        break;
    case 100:
        val = SC8551_SS_TIMEOUT_100MS;
        break;
    case 400:
        val = SC8551_SS_TIMEOUT_400MS;
        break;
    case 1500:
        val = SC8551_SS_TIMEOUT_1500MS;
        break;
    case 100000:
        val = SC8551_SS_TIMEOUT_100000MS;
        break;
    default:
        val = SC8551_SS_TIMEOUT_DISABLE;
        break;
    }

    val <<= SC8551_SS_TIMEOUT_SET_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_2B,
                SC8551_SS_TIMEOUT_SET_MASK,
                val);
}

static int sc8551_set_ibat_reg_th(struct sc8551 *sc, int th_ma)
{
    u8 val;

    if (th_ma == 200)
        val = SC8551_IBAT_REG_200MA;
    else if (th_ma == 300)
        val = SC8551_IBAT_REG_300MA;
    else if (th_ma == 400)
        val = SC8551_IBAT_REG_400MA;
    else if (th_ma == 500)
        val = SC8551_IBAT_REG_500MA;
    else
        val = SC8551_IBAT_REG_500MA;

    val <<= SC8551_IBAT_REG_SHIFT;
    return sc8551_update_bits(sc, SC8551_REG_2C,
                SC8551_IBAT_REG_MASK,
                val);
}

static int sc8551_set_vbat_reg_th(struct sc8551 *sc, int th_mv)
{
    u8 val;

    if (th_mv == 50)
        val = SC8551_VBAT_REG_50MV;
    else if (th_mv == 100)
        val = SC8551_VBAT_REG_100MV;
    else if (th_mv == 150)
        val = SC8551_VBAT_REG_150MV;
    else
        val = SC8551_VBAT_REG_200MV;

    val <<= SC8551_VBAT_REG_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_2C,
                SC8551_VBAT_REG_MASK,
                val);
}


static int sc8551_get_work_mode(struct sc8551 *sc, int *mode)
{
    int ret;
    u8 val;

    ret = sc8551_read_byte(sc, SC8551_REG_0C, &val);

    if (ret) {
        sc_err("Failed to read operation mode register\n");
        return ret;
    }

    val = (val & SC8551_MS_MASK) >> SC8551_MS_SHIFT;
    if (val == SC8551_MS_MASTER)
        *mode = SC8551_ROLE_MASTER;
    else if (val == SC8551_MS_SLAVE)
        *mode = SC8551_ROLE_SLAVE;
    else
        *mode = SC8551_ROLE_STANDALONE;

    sc_info("work mode:%s\n", *mode == SC8551_ROLE_STANDALONE ? "Standalone" :
            (*mode == SC8551_ROLE_SLAVE ? "Slave" : "Master"));
    return ret;
}

static int sc8551_check_vbus_error_status(struct sc8551 *sc)
{
    int ret;
    u8 data;

    ret = sc8551_read_byte(sc, SC8551_REG_0A, &data);
    if(ret == 0){
        sc_err("vbus error >>>>%02x\n", data);
        sc->vbus_error = data;
    }

    return ret;
}

static int sc8551_set_vbus_inrange(struct sc8551 *sc, bool en)
{
    u8 data;

    if (en)
        data = SC8551_VBUS_INRANGE_ENABLE << SC8551_VBUS_INRANGE_DIS_SHIFT;
    else
        data = SC8551_VBUS_INRANGE_DISABLE << SC8551_VBUS_INRANGE_DIS_SHIFT;

    return sc8551_update_bits(sc, SC8551_REG_35,
                SC8551_VBUS_INRANGE_DIS_MASK, data);
}

static int sc8551_detect_device(struct sc8551 *sc)
{
    int ret;
    u8 data;

    ret = sc8551_read_byte(sc, SC8551_REG_13, &data);
    if (ret == 0) {
        sc->part_no = (data & SC8551_DEV_ID_MASK);
        sc->part_no >>= SC8551_DEV_ID_SHIFT;
    }

    return ret;
}

static int sc8551_parse_dt(struct sc8551 *sc, struct device *dev)
{
    int ret;
    struct device_node *np = dev->of_node;

    sc->cfg = devm_kzalloc(dev, sizeof(struct sc8551_cfg),
                    GFP_KERNEL);

    if (!sc->cfg)
        return -ENOMEM;

    sc->cfg->bat_ovp_disable = of_property_read_bool(np,
            "sc,sc8551,bat-ovp-disable");
    sc->cfg->bat_ocp_disable = of_property_read_bool(np,
            "sc,sc8551,bat-ocp-disable");
    sc->cfg->bus_ocp_disable = of_property_read_bool(np,
            "sc,sc8551,bus-ocp-disable");

    ret = of_property_read_u32(np, "sc,sc8551,bat-ovp-threshold",
            &sc->cfg->bat_ovp_th);
    if (ret) {
        sc_err("failed to read bat-ovp-threshold\n");
        return ret;
    }

    ret = of_property_read_u32(np, "sc,sc8551,bat-ocp-threshold",
            &sc->cfg->bat_ocp_th);
    if (ret) {
        sc_err("failed to read bat-ocp-threshold\n");
        return ret;
    }

    ret = of_property_read_u32(np, "sc,sc8551,bus-ovp-threshold",
            &sc->cfg->bus_ovp_th);
    if (ret) {
        sc_err("failed to read bus-ovp-threshold\n");
        return ret;
    }

    ret = of_property_read_u32(np, "sc,sc8551,bus-ocp-threshold",
            &sc->cfg->bus_ocp_th);
    if (ret) {
        sc_err("failed to read bus-ocp-threshold\n");
        return ret;
    }

    ret = of_property_read_u32(np, "sc,sc8551,ac-ovp-threshold",
            &sc->cfg->ac_ovp_th);
    if (ret) {
        sc_err("failed to read ac-ovp-threshold\n");
        return ret;
    }

    ret = of_property_read_u32(np, "sc,sc8551,sense-resistor-mohm",
            &sc->cfg->sense_r_mohm);
    if (ret) {
        sc_err("failed to read sense-resistor-mohm\n");
        return ret;
    }


    return 0;
}


static int sc8551_init_protection(struct sc8551 *sc)
{
    int ret;

    ret = sc8551_enable_batovp(sc, !sc->cfg->bat_ovp_disable);
    sc_info("%s bat ovp %s\n",
        sc->cfg->bat_ovp_disable ? "disable" : "enable",
        !ret ? "successfullly" : "failed");

    ret = sc8551_enable_batocp(sc, !sc->cfg->bat_ocp_disable);
    sc_info("%s bat ocp %s\n",
        sc->cfg->bat_ocp_disable ? "disable" : "enable",
        !ret ? "successfullly" : "failed");

    ret = sc8551_enable_busocp(sc, !sc->cfg->bus_ocp_disable);
    sc_info("%s bus ocp %s\n",
        sc->cfg->bus_ocp_disable ? "disable" : "enable",
        !ret ? "successfullly" : "failed");

    ret = sc8551_set_batovp_th(sc, sc->cfg->bat_ovp_th);
    sc_info("set bat ovp th %d %s\n", sc->cfg->bat_ovp_th,
        !ret ? "successfully" : "failed");

    ret = sc8551_set_batocp_th(sc, sc->cfg->bat_ocp_th);
    sc_info("set bat ocp threshold %d %s\n", sc->cfg->bat_ocp_th,
        !ret ? "successfully" : "failed");

    ret = sc8551_set_busovp_th(sc, sc->cfg->bus_ovp_th);
    sc_info("set bus ovp threshold %d %s\n", sc->cfg->bus_ovp_th,
        !ret ? "successfully" : "failed");

    ret = sc8551_set_busocp_th(sc, sc->cfg->bus_ocp_th);
    sc_info("set bus ocp threshold %d %s\n", sc->cfg->bus_ocp_th,
        !ret ? "successfully" : "failed");

    ret = sc8551_set_acovp_th(sc, sc->cfg->ac_ovp_th);
    sc_info("set ac ovp threshold %d %s\n", sc->cfg->ac_ovp_th,
        !ret ? "successfully" : "failed");

    return 0;
}

static int sc8551_init_adc(struct sc8551 *sc)
{

    sc8551_set_adc_scanrate(sc, false);
    sc8551_set_adc_scan(sc, ADC_IBUS, true);
    sc8551_set_adc_scan(sc, ADC_VBUS, true);
    sc8551_set_adc_scan(sc, ADC_VOUT, true);
    sc8551_set_adc_scan(sc, ADC_VBAT, true);
    sc8551_set_adc_scan(sc, ADC_IBAT, true);
    sc8551_set_adc_scan(sc, ADC_TDIE, true);
    sc8551_set_adc_scan(sc, ADC_VAC, true);

    sc8551_enable_adc(sc, true);

    return 0;
}

static int sc8551_init_regulation(struct sc8551 *sc)
{
    sc8551_set_ibat_reg_th(sc, 300);
    sc8551_set_vbat_reg_th(sc, 100);

    sc8551_set_vdrop_deglitch(sc, 5000);
    sc8551_set_vdrop_th(sc, 400);

    sc8551_enable_regulation(sc, false);

    sc8551_write_byte(sc, SC8551_REG_2E, 0x08);
    return 0;
}

static int sc8551_init_device(struct sc8551 *sc)
{
    sc8551_set_reg_reset(sc);
    sc8551_enable_wdt(sc, false);
    sc8551_set_wdt(sc, 30000);

    sc8551_set_ss_timeout(sc, 100000);
    sc8551_set_sense_resistor(sc, sc->cfg->sense_r_mohm);

    sc8551_init_protection(sc);
    sc8551_init_adc(sc);

    sc8551_init_regulation(sc);

    if (sc->mode == SC8551_ROLE_SLAVE) {
        sc8551_set_vbus_inrange(sc, false);
    }

    return 0;
}


static int sc8551_set_present(struct sc8551 *sc, bool present)
{
    sc->usb_present = present;

    if (present)
        sc8551_init_device(sc);
    return 0;
}

static ssize_t sc8551_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8551 *sc = dev_get_drvdata(dev);
    u8 addr;
    u8 val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8551");
    for (addr = 0x0; addr <= 0x31; addr++) {
        ret = sc8551_read_byte(sc, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc8551_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8551 *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x31)
        sc8551_write_byte(sc, (unsigned char)reg, (unsigned char)val);

    return count;
}


static DEVICE_ATTR(registers, 0660, sc8551_show_registers, sc8551_store_register);

static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "reg addr 0x%08x\n", sc->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

/*
static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	ret = regmap_read(sc->regmap, sc->reg_addr, &sc->reg_data);;
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", sc->reg_addr, sc->reg_data);
}

static ssize_t store_reg_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_data = tmp;
	regmap_write(sc->regmap, sc->reg_addr, sc->reg_data);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);
*/

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	bool enable;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = sc8551_check_charge_enabled(sc, &enable);
	if (ret < 0) {
		pr_err("sc8551: sc8551_is_chg_en not valid\n");
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
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	enable = simple_strtoul(buf, NULL, 0);
	ret = sc8551_enable_charge(sc, enable);
	if (ret) {
		pr_err("sc8551 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("sc8551  %s charging \n",
			   enable ? "enable" : "disable");

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

/*
static ssize_t show_reg_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	int addr;
	ssize_t size = 0;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	for (addr = 0; addr <= 0x3a; addr++) {
		ret = regmap_read(sc->regmap, addr, &val);
		if (!ret)
			dev_err(sc->dev, "Reg[%02X] = 0x%02X\n", addr, val);
		size += snprintf(buf + size, PAGE_SIZE - size,
				"reg addr 0x%08x -> 0x%08x\n", addr,val);
	}

	return size;
}
static DEVICE_ATTR(reg_dump, 0664, show_reg_dump, NULL);
*/

static ssize_t show_vbus(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus;
	struct sc8551 *sc = dev_get_drvdata(dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}
	sc8551_get_adc_data(sc, ADC_VBUS, &vbus);

	return sprintf(buf, "%d\n", vbus);
}
static DEVICE_ATTR(vbus, 0664, show_vbus, NULL);

static void sc8551_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
	device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_reg_addr);
    //device_create_file(dev, &dev_attr_reg_data);
    //device_create_file(dev, &dev_attr_reg_dump);
    device_create_file(dev, &dev_attr_vbus);
}


static enum power_supply_property sc8551_charger_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_CHARGE_COUNTER,
    POWER_SUPPLY_PROP_CHARGE_TYPE,
    //POWER_SUPPLY_PROP_SC_MAIN_CHARGER_EN,
};

static int sc8551_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8551 *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    sc_dbg(">>>>>psp = %d\n", psp);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8551_check_charge_enabled(sc, &sc->charge_enabled);
        val->intval = sc->charge_enabled;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = sc->usb_present;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8551_get_adc_data(sc, ADC_VBUS, &result);
        if (!ret)
            sc->vbus_volt = result;

        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8551_get_adc_data(sc, ADC_IBUS, &result);
        if (!ret)
            sc->ibus_curr = result;

        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8551_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER:
        sc8551_check_vbus_error_status(sc);
        val->intval = sc->vbus_error;
        break;
    /*case POWER_SUPPLY_PROP_SC_MAIN_CHARGER_EN:
        val->intval = sc->sw_chg_en;
        break;*/
    default:
        return -EINVAL;

    }

    return 0;
}


static int sc8551_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    struct sc8551 *sc = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8551_enable_charge(sc, val->intval);
        sc8551_check_charge_enabled(sc, &sc->charge_enabled);
        sc_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
                val->intval ? "enable" : "disable");
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        sc8551_set_present(sc, !!val->intval);
        break;
    /*case POWER_SUPPLY_PROP_SC_MAIN_CHARGER_EN:
        sc->sw_chg_en = val->intval;
        break;*/
    default:
        return -EINVAL;
    }

    return 0;
}


static int sc8551_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    int ret;

    switch (prop) {
    case POWER_SUPPLY_PROP_ONLINE:
    /*case POWER_SUPPLY_PROP_SC_MAIN_CHARGER_EN:
        ret = 1;
        break;*/
    default:
        ret = 0;
        break;
    }
    return ret;
}

static int sc8551_psy_register(struct sc8551 *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;


    if (sc->mode == SC8551_ROLE_MASTER)
        sc->psy_desc.name = "cp-master";
    else if (sc->mode == SC8551_ROLE_SLAVE)
        sc->psy_desc.name = "cp-slave";
    else
        sc->psy_desc.name = "cp-standalone";

    sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    sc->psy_desc.properties = sc8551_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8551_charger_props);
    sc->psy_desc.get_property = sc8551_charger_get_property;
    sc->psy_desc.set_property = sc8551_charger_set_property;
    sc->psy_desc.property_is_writeable = sc8551_charger_is_writeable;


    sc->fc2_psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->fc2_psy)) {
        sc_err("failed to register fc2_psy\n");
        return PTR_ERR(sc->fc2_psy);
    }

    sc_info("%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}

static void sc8551_check_fault_status(struct sc8551 *sc)
{
    int ret;
    u8 flag = 0;

    mutex_lock(&sc->data_lock);

    ret = sc8551_read_byte(sc, SC8551_REG_05, &flag);
    if (!ret && (flag & SC8551_AC_OVP_FLAG_MASK))
        sc_err("irq VAC OVP FLAG\n");

    ret = sc8551_read_byte(sc, SC8551_REG_08, &flag);
    if (!ret) {
        if (flag & SC8551_IBUS_UCP_RISE_FLAG_MASK)
            sc_err("irq IBUS UCP RISE FLAG\n");
        if (flag & SC8551_IBUS_UCP_FALL_FLAG_MASK)
            sc_err("irq IBUS UCP FALL FLAG\n");
    }

    ret = sc8551_read_byte(sc, SC8551_REG_0A, &flag);
    if (!ret) {
        if (flag & SC8551_TSHUT_FLAG_MASK)
            sc_err("irq TSHUT FLAG\n");
        if (flag & SC8551_SS_TIMEOUT_FLAG_MASK)
            sc_err("irq SS TIMEOUT FLAG\n");
        if (flag & SC8551_CONV_OCP_FLAG_MASK)
            sc_err("irq CONV OCP FLAG\n");
        if (flag & SC8551_PIN_DIAG_FALL_FLAG_MASK)
            sc_err("irq PIN DIAG FALL FLAG\n");
    }

    ret = sc8551_read_byte(sc, SC8551_REG_0B, &flag);
    if (!ret && (flag & SC8551_WD_TIMEOUT_FLAG_MASK))
        sc_err("irq WTD TIMEOUT FLAG\n");

    ret = sc8551_read_byte(sc, SC8551_REG_11, &flag);
    if (!ret) {
        if (flag & SC8551_BAT_OVP_FLT_FLAG_MASK)
            sc_err("irq BAT OVP FLAG\n");
        if (flag & SC8551_BAT_OCP_FLT_FLAG_MASK)
            sc_err("irq BAT OCP FLAG\n");
        if (flag & SC8551_BUS_OVP_FLT_FLAG_MASK)
            sc_err("irq BUS OVP FLAG\n");
        if (flag & SC8551_BUS_OCP_FLT_FLAG_MASK)
            sc_err("irq BUS OCP FLAG\n");
    }

    ret = sc8551_read_byte(sc, SC8551_REG_2D, &flag);
    if (!ret) {
        if (flag & SC8551_VBAT_REG_ACTIVE_FLAG_MASK)
            sc_err("irq VBAT REG ACTIVE FLAG\n");
        if (flag & SC8551_IBAT_REG_ACTIVE_FLAG_MASK)
            sc_err("irq IBAT REG ACTIVE FLAG\n");
        if (flag & SC8551_VDROP_OVP_FLAG_MASK)
            sc_err("irq VDROP OVP FLAG\n");
        if (flag & SC8551_VOUT_OVP_FLAG_MASK)
            sc_err("irq VOUT OVP FLAG\n");
    }

    ret = sc8551_read_byte(sc, SC8551_REG_2F, &flag);
    if (!ret) {
        if (flag & SC8551_PMID2OUT_UVP_FLAG_MASK)
            sc_err("irq PMID2OUT UVP FLAG\n");
        if (flag & SC8551_PMID2OUT_OVP_FLAG_MASK)
            sc_err("irq PMID2OUT OVP FLAG\n");
    }

    ret = sc8551_read_byte(sc, SC8551_REG_30, &flag);
    if (!ret && (flag & SC8551_IBUS_REG_ACTIVE_FLAG_MASK)) {
        sc_err("irq IBUS REG ACTIVE FLAG\n");
    }

    mutex_unlock(&sc->data_lock);
}

/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
static irqreturn_t sc8551_charger_interrupt(int irq, void *dev_id)
{
    struct sc8551 *sc = dev_id;

    sc_dbg("INT OCCURED\n");
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
#if 1
    /* TODO */
    sc8551_check_fault_status(sc);
#endif

#if 0
    sc8551_dump_reg(sc);
#endif
    mutex_unlock(&sc->irq_complete);
#endif
    power_supply_changed(sc->fc2_psy);

    return IRQ_HANDLED;
}

static int sc8551_irq_register(struct sc8551 *sc)
{
    int ret;
    struct device_node *node = sc->dev->of_node;

    if (!node) {
        sc_err("device tree node missing\n");
        return -EINVAL;
    }

    sc->irq_gpio = of_get_named_gpio(node, "sc,sc8551,irq-gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        sc_err("fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }else
	sc_err("valid gpio : %d\n", sc->irq_gpio);

    ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN, "sc8551_irq");
    if (ret) {
        sc_err("fail to request sc8551 irq\n");
        return EINVAL;
    }else
	sc_err("request sc8551 irq ok\n");

    sc->irq =gpio_to_irq(sc->irq_gpio);
    if (sc->irq < 0) {
        sc_err("fail to gpio to irq\n");
        return EINVAL;
    }else
	sc_err("gpio to irq: %d\n",sc->irq);

    if (sc->mode == SC8551_ROLE_STANDALONE) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq, NULL,
                sc8551_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                "sc8551 standalone irq", sc);
    } else if (sc->mode == SC8551_ROLE_MASTER) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq, NULL,
                sc8551_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                "sc8551 master irq", sc);
    } else {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq, NULL,
                sc8551_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                "sc8551 slave irq", sc);
    }
    if (ret < 0) {
        sc_err("request irq for irq=%d failed, ret=%d\n", sc->irq, ret);
        return ret;
    }
    enable_irq_wake(sc->irq);

    return ret;
}

static void determine_initial_status(struct sc8551 *sc)
{
    if (sc->irq)
        sc8551_charger_interrupt(sc->irq, sc);
}


static struct of_device_id sc8551_charger_match_table[] = {
    {
        .compatible = "sc,sc8551-standalone",
        .data = &sc8551_mode_data[SC8551_STDALONE],
    },
    {
        .compatible = "sc,sc8551-master",
        .data = &sc8551_mode_data[SC8551_MASTER],
    },
    {
        .compatible = "sc,sc8551-slave",
        .data = &sc8551_mode_data[SC8551_SLAVE],
    },
    {},
};

/*
static void sc8551_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
    device_create_file(dev, &dev_attr_reg_addr);
    //device_create_file(dev, &dev_attr_reg_data);
    //device_create_file(dev, &dev_attr_reg_dump);
    device_create_file(dev, &dev_attr_vbus);
}
*/

static int sc8551_enable_chg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sc8551 *sc = charger_get_data(chg_dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	dev_info(sc->dev, "%s %d\n", __func__, en);
	ret = sc8551_enable_charge(sc, en);
	if (ret) {
		dev_err(sc->dev, "%s enbale fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}
/*
static int sc8551_enable_adc_external(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sc8551 *sc = charger_get_data(chg_dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	dev_info(sc->dev, "%s %d\n", __func__, en);
	ret = sc8551_enable_adc(sc, en);
	if (ret) {
		dev_err(sc->dev, "%s enable fail%d\n", __func__, en);
		return ret;
	}

	return 0;
}
*/
static int sc8551_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret;
	struct sc8551 *sc = charger_get_data(chg_dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	ret = sc8551_check_charge_enabled(sc, en);
	if (ret < 0) {
		dev_err(sc->dev, "%s get chg en fail %d\n", __func__, *en);
		return ret;
	}
	dev_info(sc->dev, "%s %d\n", __func__, *en);

	return 0;
}

static int sc8551_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	int tmp, ret;
	struct sc8551 *sc  = charger_get_data(chg_dev);
	if (!sc) {
		pr_err("sc8551 chip not valid\n");
		return -ENODEV;
	}

	switch (chan) {
		case ADC_CHANNEL_VBUS:
			ret = sc8551_get_adc_data(sc, ADC_VBUS, &tmp);
            if (!ret)
                *max = tmp;
			break;
		case ADC_CHANNEL_IBUS:
			ret = sc8551_get_adc_data(sc, ADC_IBUS, &tmp);
            if (!ret)
                *max = tmp;
			break;
		case ADC_CHANNEL_VBAT:
			ret = sc8551_get_adc_data(sc, ADC_VBAT, &tmp);
            if (!ret)
                *max = tmp;
			break;
		case ADC_CHANNEL_TEMP_JC:
			/*cp die temp*/
			*max = 25;
			break;
		case ADC_CHANNEL_VOUT:
			ret = sc8551_get_adc_data(sc, ADC_VOUT, &tmp);
            if (!ret)
                *max = tmp;
			break;
		default:
			return -ENOTSUPP;
			break;
	}
	*min = *max;

	return 0;
}

static int sc8551_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	//unsigned int val;
	u8 val;
	int tmp;
	struct sc8551 *sc  = charger_get_data(chg_dev);

	//if (sc->part_no == SC8551_PART_NO) {
		tmp = sc8551_read_byte(sc, SC8551_REG_0A, &val);
		//tmp = regmap_read(sc->regmap, SC8551_REG_0A, &val);
		if (tmp)
			return tmp;

		*err = !!(val & SC8551_VBUS_ERRPRLO_STAT);
	/*} else {
		*err = 0;
	}*/
	return 0;
}

static int sc8551_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
	*min = *max = 0;

	return 0;
}

static int sc8551_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int sc8551_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int sc8551_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int sc8551_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int sc8551_init_chip(struct charger_device *chg_dev)
{
	return 0;
}

static int sc8551_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}

static int sc8551_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int sc8551_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}

static int sc8551_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}


static const struct charger_ops sc8551_chg_ops = {
	.enable = sc8551_enable_chg,
	.is_enabled = sc8551_is_chg_enabled,
	.get_adc = sc8551_get_adc,
	.set_vbusovp = sc8551_set_vbusovp,
	.set_ibusocp = sc8551_set_ibusocp,
	.set_vbatovp = sc8551_set_vbatovp,
	.set_ibatocp = sc8551_set_ibatocp,
	.init_chip = sc8551_init_chip,
	.set_vbatovp_alarm = sc8551_set_vbatovp_alarm,
	.reset_vbatovp_alarm = sc8551_reset_vbatovp_alarm,
	.set_vbusovp_alarm = sc8551_set_vbusovp_alarm,
	.reset_vbusovp_alarm = sc8551_reset_vbusovp_alarm,
	.is_vbuslowerr = sc8551_is_vbuslowerr,
	.get_adc_accuracy = sc8551_get_adc_accuracy,
	//.config_mux = sc8551_config_mux,
	//.enable_adc = sc8551_enable_adc_external,
};


static int sc8551_register_chgdev(struct sc8551 *sc)
{
	sc->chg_prop.alias_name = sc->psy_desc.name;
	sc->chg_dev = charger_device_register("primary_dvchg", sc->dev,
						sc, &sc8551_chg_ops,
						&sc->chg_prop);
	return sc->chg_dev ? 0 : -EINVAL;
}


static int sc8551_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc8551 *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret;

    sc = devm_kzalloc(&client->dev, sizeof(struct sc8551), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->dev = &client->dev;
    sc->client = client;

    mutex_init(&sc->i2c_rw_lock);
    mutex_init(&sc->data_lock);
    mutex_init(&sc->irq_complete);

    sc->resume_completed = true;
    sc->irq_waiting = false;
    sc->sw_chg_en = true;

    ret = sc8551_detect_device(sc);
    if (ret) {
        sc_err("No sc8551 device found!\n");
        goto err_1;
    }

    i2c_set_clientdata(client, sc);
    sc8551_create_device_node(&(client->dev));

    match = of_match_node(sc8551_charger_match_table, node);
    if (match == NULL) {
        sc_err("device tree match not found!\n");
        goto err_1;
    }

    sc8551_get_work_mode(sc, &sc->mode);

    sc->mode =  *(int *)match->data;

    ret = sc8551_parse_dt(sc, &client->dev);
    if (ret)
        goto err_1;

    ret = sc8551_init_device(sc);
    if (ret) {
        sc_err("Failed to init device\n");
        goto err_1;
    }

    ret = sc8551_psy_register(sc);
    if (ret)
        goto err_2;

    ret = sc8551_irq_register(sc);
    if (ret)
        goto err_2;

    device_init_wakeup(sc->dev, 1);

    determine_initial_status(sc);
	
	ret = sc8551_register_chgdev(sc);
  	if (ret < 0) {
  		dev_err(sc->dev, "%s reg chgdev fail(%d)\n", __func__, ret);
  		goto err_2;
  	}
	
	//sc8551_create_device_node(sc->dev);

    sc_info("sc8551 probe successfully, Part Num:%d\n!",
                sc->part_no);

    return 0;

err_2:
    power_supply_unregister(sc->fc2_psy);
err_1:
    mutex_destroy(&sc->i2c_rw_lock);
    mutex_destroy(&sc->data_lock);
    mutex_destroy(&sc->irq_complete);
    sc_info("sc8551 probe fail\n");
    devm_kfree(&client->dev, sc);
    return ret;
}

static inline bool is_device_suspended(struct sc8551 *sc)
{
    return !sc->resume_completed;
}

static int sc8551_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8551 *sc = i2c_get_clientdata(client);

    mutex_lock(&sc->irq_complete);
    sc->resume_completed = false;
    mutex_unlock(&sc->irq_complete);
    sc_err("Suspend successfully!");

    return 0;
}

static int sc8551_suspend_noirq(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8551 *sc = i2c_get_clientdata(client);

    if (sc->irq_waiting) {
        pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
        return -EBUSY;
    }
    return 0;
}

static int sc8551_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc8551 *sc = i2c_get_clientdata(client);


    mutex_lock(&sc->irq_complete);
    sc->resume_completed = true;
    if (sc->irq_waiting) {
        sc->irq_disabled = false;
        enable_irq(client->irq);
        mutex_unlock(&sc->irq_complete);
        sc8551_charger_interrupt(client->irq, sc);
    } else {
        mutex_unlock(&sc->irq_complete);
    }

    power_supply_changed(sc->fc2_psy);
    sc_err("Resume successfully!");

    return 0;
}
static int sc8551_charger_remove(struct i2c_client *client)
{
    struct sc8551 *sc = i2c_get_clientdata(client);

    sc8551_enable_adc(sc, false);
    power_supply_unregister(sc->fc2_psy);
    mutex_destroy(&sc->data_lock);
    mutex_destroy(&sc->irq_complete);

    return 0;
}


static void sc8551_charger_shutdown(struct i2c_client *client)
{
    struct sc8551 *sc = i2c_get_clientdata(client);

    sc8551_enable_adc(sc, false);
    mutex_destroy(&sc->i2c_rw_lock);
}

static const struct dev_pm_ops sc8551_pm_ops = {
    .resume		= sc8551_resume,
    .suspend_noirq = sc8551_suspend_noirq,
    .suspend	= sc8551_suspend,
};

static struct i2c_driver sc8551_charger_driver = {
    .driver     = {
        .name   = "sc8551-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc8551_charger_match_table,
        .pm = &sc8551_pm_ops,
    },
    .probe      = sc8551_charger_probe,
    .remove     = sc8551_charger_remove,
    .shutdown   = sc8551_charger_shutdown,
};

module_i2c_driver(sc8551_charger_driver);

MODULE_DESCRIPTION("SC SC8551 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
