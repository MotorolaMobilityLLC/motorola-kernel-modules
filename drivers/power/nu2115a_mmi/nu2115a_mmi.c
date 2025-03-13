#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include "../../drivers/power/supply/charger_class.h"
#include "nu2115a_mmi.h"

#if IS_ENABLED(CONFIG_OEM_DEVINFO)
#include <dev_info.h>
#endif

#define NU2115_DEVID		0x40

enum {
	NU2115_MASTER,
	NU2115_SLAVE,
	NU2115_STANDALONE,
};

enum {
	NU2115_ADC_IBUS = 0,
	NU2115_ADC_VBUS,
	NU2115_ADC_VAC1,
	NU2115_ADC_VAC2,
	NU2115_ADC_VOUT,
	NU2115_ADC_VBAT,
	NU2115_ADC_IBAT,
	NU2115_ADC_TSBUS,
	NU2115_ADC_TSBAT,
	NU2115_ADC_TDIE,
};

static const char *nu2115_adc_name[] = {
	[NU2115_ADC_IBUS] = "ibus",
	[NU2115_ADC_VBUS] = "vbus",
	[NU2115_ADC_VAC1] = "vac1",
	[NU2115_ADC_VAC2] = "vac2",
	[NU2115_ADC_VOUT] = "vout",
	[NU2115_ADC_VBAT] = "vbat",
	[NU2115_ADC_IBAT] = "ibat",
	[NU2115_ADC_TSBUS] = "tsbus",
	[NU2115_ADC_TSBAT] = "tsbat",
	[NU2115_ADC_TDIE] = "tdie",
};


//static const u32 irqmask_default = ~(u32)BIT(NU2115_IRQFLAG_ADC_DONE);

struct nu2115_cfg {
	/* household */
	unsigned int ac_ovp;
	unsigned int vbus_ovp;
	unsigned int bat_ucp_alm;
	unsigned int fsw_set;
	unsigned int ibat_sns_res;
	unsigned int ss_timeout;
	/* watchdog */
	bool watchdog_dis;
	unsigned int watchdog;
};

struct nu2115 {
	struct i2c_client *client;
	struct device *dev;
	struct nu2115_cfg cfg;

	struct mutex rw_lock;
	struct mutex ops_lock;
	struct mutex adc_lock;
	struct mutex irq_lock;

	u32 irqmask;

	bool charger_enable;
	int vbus_uv;
	int ibus_ua;
	int vbat_uv;
	int ibat_ua;
	int die_temp;
	int force_chg_enable_flag;
	struct charger_device *chg_dev;
	struct charger_properties chg_prop;

	struct dentry *debugfs;
	u8 debug_addr;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *psy;
};

struct nu2115 *g_dev;

static int __nu2115_read(struct nu2115 *chip, u8 reg, u8 *val)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;

	*val = (ret & 0xFF);

	return ret < 0 ? ret : 0;
}

static int nu2115_read_device(void *client, u32 addr, int len, void *dst)
{
	struct i2c_client *i2c = (struct i2c_client *)client;

	return i2c_smbus_read_i2c_block_data(i2c, addr, len, dst);
}


static int __nu2115_write(struct nu2115 *chip, u8 reg, u8 val)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);

	return ret < 0 ? ret : 0;
}

static inline int __nu2115_i2c_read_block(struct nu2115 *chip, u8 reg,
					  u32 len, u8 *data)
{
	int ret;

	ret = nu2115_read_device(chip->client, reg, len, data);

	return ret;
}

static int nu2115_i2c_read_block(struct nu2115 *chip, u8 reg, u32 len,
				 u8 *data)
{
	int ret;

	mutex_lock(&chip->rw_lock);
	ret = __nu2115_i2c_read_block(chip, reg, len, data);
	mutex_unlock(&chip->rw_lock);

	return ret;
}

static int __nu2115_update_bits(struct nu2115 *chip, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	s32 ret = 0;

	mutex_lock(&chip->rw_lock);

	ret = __nu2115_read(chip, reg, &tmp);
	if (ret < 0)
		goto out;

	tmp = (val & mask) | (tmp & (~mask));
	ret = __nu2115_write(chip, reg, tmp);

out:
	mutex_unlock(&chip->rw_lock);

	return ret;
}

static int __nu2115_set_bits(struct nu2115 *chip, u8 reg, u8 bits)
{
	return __nu2115_update_bits(chip, reg, bits, 0xFF);
}

static int __nu2115_clr_bits(struct nu2115 *chip, u8 reg, u8 bits)
{
	return __nu2115_update_bits(chip, reg, bits, 0);
}

static bool __nu2115_check_bits(struct nu2115 *chip, u8 reg, u8 bits)
{
	u8 val;
	int ret;

	ret = __nu2115_read(chip, reg, &val);
	if (ret < 0)
		return false;

	return (val & bits) ? true : false;
}

/*
static int __nu2115_read_word(struct nu2115 *chip, u8 reg, u16 *val)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		return ret;

	*val = (u16)ret;

	return 0;
}
*/

static int __nu2115_write_word(struct nu2115 *chip, u8 reg, u16 val)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);

	return ret < 0 ? ret : 0;
}

static void __maybe_unused __nu2115_dump_register(struct nu2115 *chip)
{
	u8 reg, val;
	int ret;

	for (reg = NU2115_REG_00; reg <= NU2115_REG_36; reg++) {
		ret = __nu2115_read(chip, reg, &val);
		if (ret < 0) {
			dev_info(chip->dev, "[DUMP] 0x%02x = error\n", reg);
			continue;
		}
		dev_info(chip->dev, "[DUMP] 0x%02x = 0x%02x\n", reg, val);
	}
}

static int __nu2115_set_bat_ovp(struct nu2115 *chip, unsigned int uV)
{
	u8 val = (uV - NU2115_BAT_OVP_MIN_UV)
			/ NU2115_BAT_OVP_STEP_UV;

	if (uV < NU2115_BAT_OVP_MIN_UV)
		val = 0x0;
	if (uV > NU2115_BAT_OVP_MAX_UV)
		val = 0x7F;
	dev_info(chip->dev, "%s %d(0x%02X)\n", __func__, uV, val);
	return __nu2115_update_bits(chip, NU2115_REG_00,
				     NU2115_BAT_OVP_MASK, val);
}

static int __nu2115_set_bat_ovp_alm(struct nu2115 *chip, unsigned int uV)
{
	u8 val = (uV - NU2115_BAT_OVP_ALM_MIN_UV)
			/ NU2115_BAT_OVP_ALM_STEP_UV;

	if (uV < NU2115_BAT_OVP_ALM_MIN_UV)
		val = 0x0;
	if (uV > NU2115_BAT_OVP_ALM_MAX_UV)
		val = 0x7F;
	dev_info(chip->dev, "%s %d(0x%02X)\n", __func__, uV, val);
	return __nu2115_update_bits(chip, NU2115_REG_01,
				     NU2115_BAT_OVP_ALM_MASK, val);
}

static int __nu2115_set_bat_ocp(struct nu2115 *chip, unsigned int uA)
{
	u8 val = (uA - NU2115_BAT_OCP_MIN_UA)
			/ NU2115_BAT_OCP_STEP_UA;

	if (uA < NU2115_BAT_OCP_MIN_UA)
		val = 0x0;
	if (uA > NU2115_BAT_OCP_MAX_UA)
		val = 0x7F;
	dev_info(chip->dev, "%s %d(0x%02X)\n", __func__, uA, val);
	return __nu2115_update_bits(chip, NU2115_REG_02,
				     NU2115_BAT_OCP_MASK, val);
}

static int __nu2115_set_bat_ocp_alm(struct nu2115 *chip, unsigned int uA)
{
	u8 val = (uA - NU2115_BAT_OCP_ALM_MIN_UA)
			/ NU2115_BAT_OCP_ALM_STEP_UA;

	if (uA < NU2115_BAT_OCP_ALM_MIN_UA)
		val = 0x0;
	if (uA > NU2115_BAT_OCP_ALM_MAX_UA)
		val = 0x7F;
	dev_dbg(chip->dev, "%s %d(0x%02X)\n", __func__, uA, val);
	return __nu2115_update_bits(chip, NU2115_REG_03,
				     NU2115_BAT_OCP_ALM_MASK, val);
}

static int __nu2115_set_bat_ucp_alm(struct nu2115 *chip, unsigned int uA)
{
	u8 val = (uA - NU2115_BAT_UCP_ALM_MIN_UA)
			/ NU2115_BAT_UCP_ALM_STEP_UA;

	if (uA < NU2115_BAT_UCP_ALM_MIN_UA)
		val = 0x0;
	if (uA > NU2115_BAT_UCP_ALM_MAX_UA)
		val = 0x3F;

	return __nu2115_update_bits(chip, NU2115_REG_04,
				     NU2115_BAT_UCP_ALM_MASK, val);
}

static int __nu2115_set_ac_ovp(struct nu2115 *chip, unsigned int uV)
{
	u8 val = (uV - NU2115_AC1_OVP_MIN_UV) / NU2115_AC1_OVP_STEP_UV;

	if (uV < NU2115_AC1_OVP_MIN_UV)
		val = 0x0;
	if (uV > NU2115_AC1_OVP_MAX_UV)
		val = 0x7;

	return __nu2115_update_bits(chip, NU2115_REG_05,
				     NU2115_AC1_OVP_MASK, val);
}


static int __nu2115_set_bus_ovp(struct nu2115 *chip, unsigned int uV)
{
	u8 val = (uV - NU2115_BUS_OVP_MIN_UV)
			/ NU2115_BUS_OVP_STEP_UV;

	if (uV < NU2115_BUS_OVP_MIN_UV)
		val = 0x0;
	if (uV > NU2115_BUS_OVP_MAX_UV)
		val = 0x3F;

	dev_info(chip->dev, "%s %d(0x%02X)\n", __func__, uV, val);
	return __nu2115_update_bits(chip, NU2115_REG_07,
				     NU2115_BUS_OVP_MASK, val);
}


static int __nu2115_set_bus_ovp_alm(struct nu2115 *chip, unsigned int uV)
{
	u8 val = (uV - NU2115_BUS_OVP_ALM_MIN_UV)
			/ NU2115_BUS_OVP_ALM_STEP_UV;

	if (uV < NU2115_BUS_OVP_ALM_MIN_UV)
		val = 0x0;
	if (uV > NU2115_BUS_OVP_ALM_MAX_UV)
		val = 0x3F;
	dev_dbg(chip->dev, "%s %d(0x%02X)\n", __func__, uV, val);
	return __nu2115_update_bits(chip, NU2115_REG_08,
				     NU2115_BUS_OVP_ALM_MASK, val);
}

static int __nu2115_set_bus_ocp(struct nu2115 *chip, unsigned int uA)
{
	u8 val = (uA - NU2115_BUS_OCP_MIN_UA)
			/ NU2115_BUS_OCP_STEP_UA;

	if (uA < NU2115_BUS_OCP_MIN_UA)
		val = 0x0;
	if (uA > NU2115_BUS_OCP_MAX_UA)
		val = 0x0F;

	dev_info(chip->dev, "%s %d(0x%02X)\n", __func__, uA, val);

	return __nu2115_update_bits(chip, NU2115_REG_09,
				     NU2115_BUS_OCP_MASK, val);
}

static int __nu2115_set_bus_ocp_alm(struct nu2115 *chip, unsigned int uA)
{
	u8 val = (uA - NU2115_BUS_OCP_ALM_MIN_UA)
			/ NU2115_BUS_OCP_ALM_STEP_UA;

	if (uA < NU2115_BUS_OCP_ALM_MIN_UA)
		val = 0x0;
	if (uA > NU2115_BUS_OCP_ALM_MAX_UA)
		val = 0x1F;

	dev_dbg(chip->dev, "%s %d(0x%02X)\n", __func__, uA, val);
	return __nu2115_update_bits(chip, NU2115_REG_0A,
				     NU2115_BUS_OCP_ALM_MASK, val);
}

/*
static int __nu2115_set_fsw(struct nu2115 *chip, unsigned int hz)
{
	const unsigned int fsw_set[] = {
		200000, 300000, 400000, 500000, 600000, 700000, 800000, 900000
	};
	u8 val;

	for (val = 0; val < ARRAY_SIZE(fsw_set) - 1; val++) {
		if (hz <= fsw_set[val])
			break;
	}

	return __nu2115_update_bits(chip, NU2115_REG_0D,
				     NU2115_FSW_SET_MASK,
				     val << NU2115_FSW_SET_SHIFT);
}
*/

static int __nu2115_enable_watchdog(struct nu2115 *chip, bool en)
{
	return (en ? __nu2115_clr_bits : __nu2115_set_bits)
			(chip, NU2115_REG_0D, NU2115_WATCHDOG_DIS_MASK);
}

static int __nu2115_set_watchdog(struct nu2115 *chip, unsigned int usec)
{
	const unsigned int watchdog[] = {
		500000, 1000000, 5000000, 30000000
	};
	u8 val;

	for (val = 0; val < ARRAY_SIZE(watchdog) - 1; val++) {
		if (usec <= watchdog[val])
			break;
	}

	return __nu2115_update_bits(chip, NU2115_REG_0D,
				     NU2115_WATCHDOG_MASK, val);
}

static bool __nu2115_is_chg_enabled(struct nu2115 *chip)
{
	return __nu2115_check_bits(chip, NU2115_REG_0E,
				    NU2115_CHG_EN_MASK);
}

static int __nu2115_enable_chg(struct nu2115 *chip, bool en)
{
	return (en ? __nu2115_set_bits : __nu2115_clr_bits)
			(chip, NU2115_REG_0E, NU2115_CHG_EN_MASK);
}

static bool __nu2115_is_adc_enabled(struct nu2115 *chip)
{
	return __nu2115_check_bits(chip, NU2115_REG_15,
				    NU2115_ADC_EN_MASK);
}

static int __nu2115_enable_adc(struct nu2115 *chip, bool en)
{
	return (en ? __nu2115_set_bits : __nu2115_clr_bits)
			(chip, NU2115_REG_15, NU2115_ADC_EN_MASK);
}

static int __nu2115_set_adc_fn_dis(struct nu2115 *chip, u16 adc_fn_dis)
{
	u16 val = ((adc_fn_dis & 0xFF) << 8) | ((adc_fn_dis >> 8) & 0xFF);

	return __nu2115_write_word(chip, NU2115_REG_15, val);
}

/*
static int __nu2115_set_ss_timeout(struct nu2115 *chip, unsigned int us)
{
	const unsigned int timeout[] = {
		0, 12500, 25000, 50000, 100000, 400000, 1500000, 100000000
	};
	u8 val;

	for (val = 0; val < ARRAY_SIZE(timeout); val++) {
		if (us <= timeout[val])
			break;
	}

	return __nu2115_update_bits(chip, NU2115_REG_2E,
				     NU2115_SS_TIMEOUT_SET_MASK,
				     val << NU2115_SS_TIMEOUT_SET_SHIFT);
}
*/

static int __nu2115_set_ibat_sns_res(struct nu2115 *chip, unsigned int ohm)
{
	return ((ohm == 5) ? __nu2115_set_bits : __nu2115_clr_bits)
			(chip, NU2115_REG_2E,
			 NU2115_SET_IBAT_SNS_RES_MASK);
}

static int __nu2115_convert_adc(struct nu2115 *chip, int channel, u16 val)
{
	s16 sval = (s16)val;
	int ret;
	int conv_val = 0;
	struct power_supply *bat_psy;
	union power_supply_propval prop;

	switch (channel) {
	case NU2115_ADC_IBUS:
	case NU2115_ADC_VBUS:
	case NU2115_ADC_VAC1:
	case NU2115_ADC_VAC2:
	case NU2115_ADC_VOUT:
	case NU2115_ADC_VBAT:
		/* in micro volt */
		conv_val = sval * 1000;
		dev_err(chip->dev, "%s adc: %duV (0x%04x)\n",
				nu2115_adc_name[channel], conv_val, val);
		break;
	case NU2115_ADC_IBAT:
		/* in micro amp */
		/*conv_val = sval * 1000;
		dev_err(chip->dev, "%s adc: %duA (0x%04x)\n",
				nu2115_adc_name[channel], conv_val, val);*/
		dev_info(chip->dev, "get ibat from fuel gauge\n");
		bat_psy = power_supply_get_by_name("battery");
		if (IS_ERR_OR_NULL(bat_psy)) {
			dev_info(chip->dev, "%s Couldn't get bat_psy\n", __func__);
			conv_val = 0;
		} else {
			ret = power_supply_get_property(bat_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
			conv_val = prop.intval;
			dev_info(chip->dev,"%s con_val=%d ", __func__,ret);
		}

		dev_info(chip->dev,"%s %d %d", __func__, channel, conv_val);
		break;
	case NU2115_ADC_TDIE:
		/* in degreeC */
		conv_val = sval >> 1;
		dev_err(chip->dev, "%s adc: %d.%cdegC (0x%04x)\n",
				nu2115_adc_name[channel],
				conv_val, (sval & 0x1 ? '5' : '0'), val);
		break;
	case NU2115_ADC_TSBAT:
	case NU2115_ADC_TSBUS:
		/* in percent */
		conv_val = sval * 100 / 1024;
		dev_err(chip->dev, "%s adc: %d%% (0x%04x)\n",
				nu2115_adc_name[channel], conv_val, val);
		break;
	}

	return conv_val;
}

static int nu2115_get_adc_data(struct nu2115 *chip, int channel,  int *result)
{
	int ret;
	u16 temp;
	u8 data[2];

	if (channel < NU2115_ADC_IBUS || channel > NU2115_ADC_TDIE)
		return -EINVAL;

	switch (channel) {
	case NU2115_ADC_IBUS:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_17, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_VBUS:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_19, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_VAC1:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_1B, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_VAC2:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_1D, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_VOUT:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_1F, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_VBAT:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_21, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_IBAT:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_23, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_TSBUS:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_25, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_TSBAT:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_27, 2, data);
		if (ret < 0)
			return ret;
		break;
	case NU2115_ADC_TDIE:
		ret = nu2115_i2c_read_block(chip, NU2115_REG_29, 2, data);
		if (ret < 0)
			return ret;
		break;
	default:
		dev_err(chip->dev, "failed to enable adc\n");
		break;
	}

	temp = (data[0] << 8) + data[1];

	*result = __nu2115_convert_adc(chip, channel, temp);
	dev_dbg(chip->dev, "get %s adc, reg: %2x, channel:%d, val:%4x\n",
				nu2115_adc_name[channel], (NU2115_REG_17 + (channel << 1)),
				channel, temp);

	return 0;
}


static int __nu2115_get_adc(struct nu2115 *chip, int channel, int *data)
{
	int ret;

	if (channel < NU2115_ADC_IBUS || channel > NU2115_ADC_TDIE)
		return -EINVAL;
	mutex_lock(&chip->adc_lock);
	__nu2115_enable_adc(chip,true);
	msleep(30);
	ret = nu2115_get_adc_data(chip, channel, data);
	dev_info(chip->dev,"%s ret=%d ", __func__,ret);
	__nu2115_enable_adc(chip,false);
	mutex_unlock(&chip->adc_lock);
	return 0;
}

/*
 * set intterupt bit
 */
static int __nu2115_set_irqmask(struct nu2115 *chip, u8 addr, u8 mask)
{
	int ret;
	u8 val;

	ret = __nu2115_read(chip, addr, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = __nu2115_write(chip, addr, val);

	return ret;
}

static int __nu2115_reg_reset(struct nu2115 *chip)
{
	return __nu2115_update_bits(chip, NU2115_REG_0D,
			NU2115_REG_RST_MASK, NU2115_REG_RST_MASK);
}

/*
 * check stat and flag regs for IC status
 */
static void nu2115_check_status_flags(struct nu2115 *chip)
{
	int ret;
	u8 sf_reg[22] = {0};

	dev_err(chip->dev,"---------%s---------\n", __func__);

	ret = nu2115_i2c_read_block(chip, NU2115_REG_05, 2, &sf_reg[0]);
	if (ret > 0) {
		if (sf_reg[0] & NU2115_AC1_OVP_STAT_MASK)
			dev_err(chip->dev,"REG_05:%x, AC1_OVP_STAT\n", sf_reg[0]);

		if (sf_reg[0] & NU2115_AC1_OVP_FLAG_MASK)
			dev_err(chip->dev,"REG_05:%x, AC1_OVP_FLAG\n", sf_reg[0]);

		if (sf_reg[1] & NU2115_AC2_OVP_STAT_MASK)
			dev_err(chip->dev,"REG_06:%x, AC2_OVP_STAT\n", sf_reg[1]);

		if (sf_reg[1] & NU2115_AC2_OVP_FLAG_MASK)
			dev_err(chip->dev,"REG_06:%x, AC2_OVP_FLAG\n", sf_reg[1]);
	}

	ret = nu2115_i2c_read_block(chip, NU2115_REG_09, 11, &sf_reg[2]);

	if (ret > 0) {
		if (sf_reg[2] & NU2115_IBUS_UCP_RISE_FLAG_MASK)
			dev_err(chip->dev,"REG_09:%x, IBUS_UCP_RISE_FLAG\n", sf_reg[2]);

		if (sf_reg[3] & NU2115_IBUS_UCP_FALL_FLAG_MASK)
			dev_err(chip->dev,"REG_0A:%x, IBUS_UCP_FALL_FLAG\n", sf_reg[3]);

		if (sf_reg[4] & NU2115_VOUT_OVP_STAT_MASK)
			dev_err(chip->dev,"REG_0B:%x, VOUT_OVP_STAT\n", sf_reg[4]);
		if (sf_reg[4] & NU2115_VOUT_OVP_FLAG_MASK)
			dev_err(chip->dev,"REG_0B:%x, VOUT_OVP_FLAG\n", sf_reg[4]);

		if (sf_reg[5] & NU2115_TSD_FLAG_MASK)
			dev_err(chip->dev,"REG_0C:%x, TSD_FLAG\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_TSD_STAT_MASK)
			dev_err(chip->dev,"REG_0C:%x, TSD_STAT\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_VBUS_ERRORLO_FLAG_MASK)
			dev_err(chip->dev,"REG_0C:%x, VBUS_ERRLO_FLAG\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_VBUS_ERRORHI_FLAG_MASK)
			dev_err(chip->dev,"REG_0C:%x, VBUS_ERRHI_FLAG\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_SS_TIMEOUT_FLAG_MASK)
			dev_err(chip->dev,"REG_0C:%x, SS_TIMEOUT_FLAG\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_CONV_SWITCHING_STAT_MASK)
			dev_err(chip->dev,"REG_0C:%x, CONV_ACTIVE_STAT\n", sf_reg[5]);
		if (sf_reg[5] & NU2115_PIN_DIAG_FALL_FLAG_MASK)
			dev_err(chip->dev,"REG_0C:%x, PIN_DIAG_FAIL_FLAG\n", sf_reg[5]);

		if (sf_reg[6] & NU2115_WD_TIMEOUT_FLAG_MASK)
			dev_err(chip->dev,"REG_0D:%x, WD_TIMEOUT_FLAG\n", sf_reg[6]);

		if (sf_reg[8] & NU2115_BAT_OVP_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, BAT_OVP_ALM_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_BAT_OCP_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, BAT_OCP_ALM_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_BUS_OVP_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, BUS_OVP_ALM_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_BUS_OCP_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, BUS_OCP_ALM_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_BAT_UCP_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, BAT_UCP_ALM_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_VBAT_INSERT_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, VBAT_INSERT_STAT\n", sf_reg[8]);
		if (sf_reg[8] & NU2115_ADC_DONE_STAT_MASK)
			dev_err(chip->dev,"REG_0F:%x, ADC_DONE_STAT\n", sf_reg[8]);

		if (sf_reg[9] & NU2115_BAT_OVP_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, BAT_OVP_ALM_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_BAT_OCP_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, BAT_OCP_ALM_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_BUS_OVP_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, BUS_OVP_ALM_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_BUS_OCP_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, BUS_OCP_ALM_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_BAT_UCP_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, BAT_UCP_ALM_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_VBAT_INSERT_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, VBAT_INSERT_FLAG\n", sf_reg[9]);
		if (sf_reg[9] & NU2115_ADC_DONE_FLAG_MASK)
			dev_err(chip->dev,"REG_10:%x, ADC_DONE_FLAG\n", sf_reg[9]);

		if (sf_reg[11] & NU2115_BAT_OVP_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, BAT_OVP_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_BAT_OCP_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, BAT_OCP_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_BUS_OVP_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, BUS_OVP_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_BUS_OCP_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, BUS_OCP_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_BUS_RCP_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, BUS_RCP_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_TS_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, TS_ALM_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_TS_FLT_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, TS_FLT_STAT\n", sf_reg[11]);
		if (sf_reg[11] & NU2115_TDIE_ALM_STAT_MASK)
			dev_err(chip->dev,"REG_12:%x, TDIE_ALM_STAT\n", sf_reg[11]);

		if (sf_reg[12] & NU2115_BAT_OVP_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, BAT_OVP_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_BAT_OCP_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, BAT_OCP_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_BUS_OVP_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, BUS_OVP_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_BUS_OCP_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, BUS_OCP_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_BUS_RCP_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, BUS_RCP_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_TS_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, TS_ALM_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_TS_FLT_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, TS_FLT_FLAG\n", sf_reg[12]);
		if (sf_reg[12] & NU2115_TDIE_ALM_FLAG_MASK)
			dev_err(chip->dev,"REG_13:%x, TDIE_ALM_FLAG\n", sf_reg[12]);
	}

	ret = nu2115_i2c_read_block(chip, NU2115_REG_2E, 9, &sf_reg[13]);
	if (ret > 0) {
		if (sf_reg[14] & NU2115_VAC1PRESENT_STAT_MASK)
		    dev_err(chip->dev,"REG_2F:%x, VAC1PRESENT_STAT\n", sf_reg[14]);
		if (sf_reg[14] & NU2115_VAC1PRESENT_FLAG_MASK)
		    dev_err(chip->dev,"REG_2F:%x, VAC1PRESENT_FLAG\n", sf_reg[14]);
		if (sf_reg[14] & NU2115_VAC2PRESENT_STAT_MASK)
		    dev_err(chip->dev,"REG_2F:%x, VAC2PRESENT_STAT\n", sf_reg[14]);
		if (sf_reg[14] & NU2115_VAC2PRESENT_FLAG_MASK)
		    dev_err(chip->dev,"REG_2F:%x, VAC2PRESENT_FLAG\n", sf_reg[14]);

		if (sf_reg[15] & NU2115_ACRB1_STAT_MASK)
		    dev_err(chip->dev,"REG_30:%x, ACRB1_STAT\n", sf_reg[15]);
		if (sf_reg[15] & NU2115_ACRB1_FLAG_MASK)
		    dev_err(chip->dev,"REG_30:%x, ACRB1_FLAG\n", sf_reg[15]);
		if (sf_reg[15] & NU2115_ACRB2_STAT_MASK)
		    dev_err(chip->dev,"REG_30:%x, ACRB2_STAT\n", sf_reg[15]);
		if (sf_reg[15] & NU2115_ACRB2_FLAG_MASK)
		    dev_err(chip->dev,"REG_30:%x, ACRB2_FLAG\n", sf_reg[15]);

		if (sf_reg[17] & NU2115_PMID2VOUT_UVP_FLAG_MASK)
		    dev_err(chip->dev,"REG_32:%x, PMID2VOUT_UVP_FLAG\n", sf_reg[17]);

		if (sf_reg[17] & NU2115_PMID2VOUT_OVP_FLAG_MASK)
		    dev_err(chip->dev,"REG_32:%x, PMID2VOUT_OVP_FLAG\n", sf_reg[17]);

		if (sf_reg[19] & NU2115_POWER_NG_FLAG_MASK)
		    dev_err(chip->dev,"REG_34:%x, POWER_NG_FLAG\n", sf_reg[19]);

		if (sf_reg[21] & NU2115_VBUS_PRESENT_STAT_MASK)
		    dev_err(chip->dev,"REG_36:%x, VBUS_PRESENT_STAT\n", sf_reg[21]);
		if (sf_reg[21] & NU2115_VBUS_PRESENT_FLAG_MASK)
		    dev_err(chip->dev,"REG_36:%x, VBUS_PRESENT_FLAG\n", sf_reg[21]);
	}

}

/*
 * check ACDRV bit
 */
static int __nu2115_check_acdrv_bit(struct nu2115 *chip)
{
	int ret;
	u8 val = 0;

	ret = __nu2115_read(chip, NU2115_REG_2F, &val);

	if (ret >= 0) {
		val = val & NU2115_DIS_ACDRV_MASK;
		if (val) {
			ret = __nu2115_update_bits(chip, NU2115_REG_2F,	NU2115_DIS_ACDRV_MASK, 0);
			dev_err(chip->dev, "%s:set dis_acdrv = %d\n", __func__, val);
			if (ret)
				dev_err(chip->dev, "%s: disable dis_acdrv fail ret=%d", __func__, ret);
		}
	}

	return ret;
}

static irqreturn_t nu2115_irq_handler(int irq, void *data)
{
	struct nu2115 *chip = data;

	dev_err(chip->dev, "nu2115_irq_handler do\n");
	mutex_lock(&chip->irq_lock);
	nu2115_check_status_flags(chip);
	__nu2115_check_acdrv_bit(chip);
	mutex_unlock(&chip->irq_lock);

	return IRQ_HANDLED;
}

/* charger interface */
static int nu2115_enable_chg(struct charger_device *chg_dev, bool en)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret;
	dev_err(chip->dev, "nu2115 enable: %d\n", en);
	if (!en) {
		ret = __nu2115_set_irqmask(chip, NU2115_REG_14, 0x7);
		if (ret < 0)
			dev_err(chip->dev, "failed to set irqmask\n");

		ret = __nu2115_enable_chg(chip, false);
		if (ret < 0) {
			dev_err(chip->dev, "failed to disable chg\n");
			return ret;
		}

		mutex_lock(&chip->adc_lock);

		ret = __nu2115_enable_adc(chip, false);
		if (ret < 0) {
			dev_err(chip->dev, "failed to disable adc\n");
			return ret;
		}

		mutex_unlock(&chip->adc_lock);

		goto out;
	}

	nu2115_check_status_flags(chip);

	ret = __nu2115_set_irqmask(chip, NU2115_REG_14, 0x7);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set irqmask\n");
		return ret;
	}

	ret = __nu2115_enable_adc(chip, true);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable adc\n");
		return ret;
	}
	if (en && (__nu2115_is_adc_enabled(chip) == true)) {
	    __nu2115_enable_adc(chip, false);
	    ret = __nu2115_enable_adc(chip, true);
	}

	ret =  __nu2115_write(chip, NU2115_REG_32, 0x70);
	ret = __nu2115_enable_chg(chip, true);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable chg\n");
		return ret;
	}

	if (en && !__nu2115_is_chg_enabled(chip)) {
		dev_err(chip->dev, "chg not enabled\n");
		return -EIO;
	}

	msleep(30);
	ret =  __nu2115_write(chip, NU2115_REG_32, 0x50);
	dev_err(chip->dev, "nu2115 enable ok: %d\n", en);

out:

	if (chip->cfg.watchdog_dis)
		return 0;

	ret = __nu2115_enable_watchdog(chip, false);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable watchdog\n");
		return ret;
	}

	return 0;
}

static int nu2115_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	*en = __nu2115_is_chg_enabled(chip);

	return 0;
}

static int nu2115_is_charger_enabled(struct nu2115 *chip, bool *en)
{
	*en = __nu2115_is_chg_enabled(chip);

	return 0;
}
#if 0

static int mtk_nu2115_enable_adc(struct charger_device *chg_dev, bool enable)
{
	int ret;
	struct nu2115 *chip = charger_get_data(chg_dev);

	dev_info(chip->dev,"%s:%d", __func__, enable);
	ret = __nu2115_enable_adc(chip, enable);
	if (ret < 0)
		dev_err(chip->dev, "%s fail\n", __func__);
	return ret;
}

static int nu2115_set_chg_mode(struct charger_device *chg_dev, int mode)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret = 0;

	if (mode)
	{
		ret = __nu2115_update_bits(chip, NU2115_REG_0E,
				     NU2115_CHG_MODE_MASK,
				     0x20);
		dev_err(chip->dev, "enable bapass mode\n");
	}
	else
	{
		ret = __nu2115_update_bits(chip, NU2115_REG_0E,
				     NU2115_CHG_MODE_MASK,
				     0);
		dev_err(chip->dev, "disable bapass mode\n");
	}

	return ret;
}

static int nu2115_get_chg_mode(struct charger_device *chg_dev, int *mode)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	*mode = __nu2115_check_bits(chip, NU2115_REG_0E,
				    NU2115_CHG_MODE_MASK);

	return 0;
}
#endif

static int nu2115_get_adc(struct charger_device *chg_dev,
			   enum adc_channel chan, int *min, int *max)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int channel;
	int ret;

	switch (chan) {
	case ADC_CHANNEL_VBUS:
		channel = NU2115_ADC_VBUS;
		break;
	case ADC_CHANNEL_VBAT:
		channel = NU2115_ADC_VBAT;
		break;
	case ADC_CHANNEL_IBUS:
		channel = NU2115_ADC_IBUS;
		break;
	case ADC_CHANNEL_IBAT:
		channel = NU2115_ADC_IBAT;
		break;
	case ADC_CHANNEL_TEMP_JC:
		channel = NU2115_ADC_TDIE;
		break;
	case ADC_CHANNEL_VOUT:
		channel = NU2115_ADC_VOUT;
		break;
	default:
		return -ENOTSUPP;
	}

	if (!min || !max)
		return -EINVAL;

	ret = __nu2115_get_adc(chip, channel, max);
    //dev_err(chip->dev, "%s------max=%d\n",__func__,*max);
	if (ret < 0)
		*max = 0;

	*min = *max;

	return ret;
}

static int nu2115_get_adc_accuracy(struct charger_device *chg_dev,
				    enum adc_channel chan, int *min, int *max)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		*min = 35000;
		*max = 35000;
		break;
	case ADC_CHANNEL_VBAT:
		*min = 20000;
		*max = 20000;
		break;
	case ADC_CHANNEL_IBUS:
		*min = 150000;
		*max = 150000;
		break;
	case ADC_CHANNEL_IBAT:
		*min = 200000;
		*max = 200000;
		break;
	case ADC_CHANNEL_TEMP_JC:
		*min = 4;
		*max = 4;
		break;
	case ADC_CHANNEL_VOUT:
		*min = 20000;
		*max = 20000;
		break;
	default:
		*min = 0;
		*max = 0;
		break;
	}

	return 0;
}

static int nu2115_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	return __nu2115_set_bus_ovp(chip, uV);
}

static int nu2115_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	/* uA will be 110% of target */
	__nu2115_set_bus_ocp_alm(chip, uA / 110 * 100);

	return __nu2115_set_bus_ocp(chip, uA);
}

static int nu2115_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	return __nu2115_set_bat_ovp(chip, uV);
}

static int nu2115_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret;

	mutex_lock(&chip->ops_lock);

	ret = __nu2115_set_bat_ovp_alm(chip, uV);

	mutex_unlock(&chip->ops_lock);

	return ret;
}

static int nu2115_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	mutex_lock(&chip->ops_lock);

	__nu2115_set_bits(chip, NU2115_REG_01,
			NU2115_BAT_OVP_ALM_DIS_MASK);
	__nu2115_clr_bits(chip, NU2115_REG_01,
			NU2115_BAT_OVP_ALM_DIS_MASK);

	mutex_unlock(&chip->ops_lock);

	return 0;
}

static int nu2115_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret;

	mutex_lock(&chip->ops_lock);

	ret = __nu2115_set_bus_ovp_alm(chip, uV);

	mutex_unlock(&chip->ops_lock);

	return ret;
}

#define VBUS_ERROR_LO		BIT(2)

static int nu2115_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret;
	u8 stat = 0;
	*err = false;
	return 0;

	ret = __nu2115_read(chip, NU2115_REG_35, &stat);
	if (!ret) {
		dev_err(chip->dev,"nu2115_is_vbuslowerr,NU2115_REG_0A: 0x%02X\n", stat);

		if (stat & VBUS_ERROR_LO)
			*err = true;
	}

	return 0;
}

static int nu2115_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	mutex_lock(&chip->ops_lock);

	__nu2115_set_bits(chip, NU2115_REG_08,
			NU2115_BUS_OVP_ALM_DIS_MASK);
	__nu2115_clr_bits(chip, NU2115_REG_08,
			NU2115_BUS_OVP_ALM_DIS_MASK);

	mutex_unlock(&chip->ops_lock);

	return 0;
}

static int nu2115_init_device(struct nu2115 *chip)
{
	int ret = 0;

	ret = __nu2115_reg_reset(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to reset registers(%d)\n", ret);
	}
	msleep(10);

	return ret;
}

static int nu2115_init_chip(struct charger_device *chg_dev)
{
	return 0;
}

static int nu2115_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	/* uA will be 110% of target */
	__nu2115_set_bat_ocp_alm(chip, uA / 110 * 100);

	return __nu2115_set_bat_ocp(chip, uA);
}


static int mtk_nu2115_is_vbushigher(struct charger_device *chg_dev, bool *err)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	*err =  __nu2115_check_bits(chip, NU2115_REG_35, NU2115_VBUS_ERRHI_STAT_MASK);
	dev_err(chip->dev, "nu2115_is_vbushigher,NU2115_REG_35: 0x%02X\n", *err);

	dev_err(chip->dev, "vbushigher is %s\n", *err ? "true" : "false");

	return 0;
}

#if IS_ENABLED(CONFIG_OEM_TURBO_CHARGER)
static int mtk_nu2115_is_vbat_present(struct charger_device *chg_dev, bool *present)
{
	int ret;
	struct nu2115 *chip = charger_get_data(chg_dev);

	*present =  __nu2115_check_bits(chip, NU2115_REG_0F, NU2115_VBAT_INSERT_STAT_MASK);
	dev_err(chip->dev,"nu2115_is_vbat_present,NU2115_REG_0F: 0x%02X\n", *present);

	ret = !(*present);
	return ret;
}

static int mtk_nu2115_is_vbus_present(struct charger_device *chg_dev, bool *present)
{
	bool ret;
	struct nu2115 *chip = charger_get_data(chg_dev);

	*present =  __nu2115_check_bits(chip, NU2115_REG_36, NU2115_VBUS_PRESENT_STAT_MASK);
	dev_err(chip->dev,"nu2115_is_vbus_present,NU2115_REG_36: 0x%02X\n", *present);

	ret = !(*present);

	return ret;
}
#endif

static int nu2115_enable_otg(struct charger_device *chg_dev, bool enable)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret = 0;
	u8 val;
	u8 val1;

	dev_err(chip->dev, "%s enter\n", __func__);

	if (enable)
		val = NU2115_OTG_ENABLE;
	else
		val = NU2115_OTG_DISABLE;


	ret = __nu2115_read(chip, NU2115_REG_2F, &val1);
	if (ret >= 0) {
		val1 = val1 & NU2115_EN_OTG_MASK;
		if (val == val1) {
			dev_err(chip->dev, "%s find otg bit no change, return.\n", __func__);
			__nu2115_check_acdrv_bit(chip);
			return ret;
		}
	}

	val <<= NU2115_EN_OTG_SHIFT;

	ret = __nu2115_update_bits(chip, NU2115_REG_2F,
				NU2115_EN_OTG_MASK, val);

	__nu2115_check_acdrv_bit(chip);

	return ret;
}

static int nu2115_enable_acdrv1(struct charger_device *chg_dev, bool enable)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	int ret = 0;
	u8 val;

	dev_err(chip->dev, "%s enter\n", __func__);

	if (enable)
		val = NU2115_ACDRV1_ENABLE;
	else
		val = NU2115_ACDRV1_DISABLE;

	val <<= NU2115_EN_ACDRV1_SHIFT;

	ret = __nu2115_update_bits(chip, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, val);

	return ret;
}

static bool nu2115_is_vbusovp_en(struct nu2115 *bq)
{
	u8 state;
	int ret;

	ret = __nu2115_read(bq, NU2115_REG_07, &state);
	if (ret < 0)
		return ret;

	ret = !!(state & NU2115_BUS_OVP_DIS_MASK);

	return ret;
}

static int nu2115_config_mux(struct charger_device *chg_dev,
			enum mmi_dvchg_mux_channel typec_mos, enum mmi_dvchg_mux_channel wls_mos)
{
	int ret;
	u8 val;
	struct nu2115 *bq  = charger_get_data(chg_dev);

	if (typec_mos != MMI_DVCHG_MUX_OTG_OPEN && wls_mos != MMI_DVCHG_MUX_OTG_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_2F,
			NU2115_EN_OTG_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux close en otg fail ret=%d", __func__, ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_CLOSE) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s mmi_mux close typec mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(100);
	}

	if (wls_mos == MMI_DVCHG_MUX_CLOSE) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV2_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(100);
	}

	if (typec_mos == MMI_DVCHG_MUX_CHG_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, NU2115_EN_ACDRV1_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux open typec mos fail ret=%d", __func__, ret);
			return ret;
		}

		if (nu2115_is_vbusovp_en(bq)) {
			ret = __nu2115_update_bits(bq, NU2115_REG_07,NU2115_BUS_OVP_DIS_MASK,0);
			dev_err(bq->dev, "%s need open vbus ovp function\n", __func__);
		}

	} else if (typec_mos == MMI_DVCHG_MUX_OTG_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_2F,
				NU2115_EN_OTG_MASK, NU2115_EN_OTG_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
			return ret;
		}

		udelay(100);

		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, NU2115_EN_ACDRV1_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
			return ret;
		}
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	} else if (typec_mos == MMI_DVCHG_MUX_OTG_WLC_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_2F,
				NU2115_EN_OTG_MASK, NU2115_EN_OTG_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux  en otg fail ret=%d", __func__, ret);
			return ret;
		}

		udelay(100);

		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
			return ret;
		}
#endif
	}

	if (wls_mos == MMI_DVCHG_MUX_CHG_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV2_MASK, NU2115_EN_ACDRV2_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux  open wls mux fail ret=%d", __func__, ret);
			return ret;
		}
	} else if (wls_mos == MMI_DVCHG_MUX_MANUAL_OPEN) {
		ret = __nu2115_update_bits(bq, NU2115_REG_2F,
				NU2115_EN_OTG_MASK, NU2115_EN_OTG_MASK);

		ret = __nu2115_read(bq, NU2115_REG_2F, &val);
		if (ret < 0)
			dev_err(bq->dev, "%s:NU2115_REG_2F = 0x%02X\n", __func__,val);

		ret = __nu2115_update_bits(bq, NU2115_REG_07,
				NU2115_BUS_OVP_DIS_MASK, NU2115_BUS_OVP_DIS_MASK);

		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux menu close wls mos fail ret=%d", __func__, ret);
			return ret;
		}
		mdelay(200);
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV2_MASK, NU2115_EN_ACDRV2_MASK);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux enable otg typec mos fail ret=%d", __func__, ret);
			return ret;
		}
	}

	if (typec_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV1_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux close typec mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(1000);
	}
	if (wls_mos == MMI_DVCHG_MUX_DISABLE) {
		ret = __nu2115_update_bits(bq, NU2115_REG_30,
				NU2115_EN_ACDRV2_MASK, 0);
		if (ret < 0) {
			dev_err(bq->dev, "%s:mmi_mux close wls mos fail ret=%d", __func__, ret);
			return ret;
		}
		udelay(1000);
	}

	ret = __nu2115_read(bq, NU2115_REG_2F, &val);
	if (ret >= 0) {
		dev_err(bq->dev, "%s:mmi_mux [Reg NU2115_REG_2F] = 0x%02X\n", __func__,val);
	}
	__nu2115_check_acdrv_bit(bq);

	ret = __nu2115_read(bq, NU2115_REG_30, &val);
	if (ret >= 0)
		dev_err(bq->dev, "%s:mmi_mux [Reg NU2115_REG_30] = 0x%02X\n", __func__, val);

	return 0;
}

static const struct charger_ops nu2115_chg_ops = {
	.enable = nu2115_enable_chg,
	.is_enabled = nu2115_is_chg_enabled,
	//.enable_adc = mtk_nu2115_enable_adc,
	.get_adc = nu2115_get_adc,
	.set_vbusovp = nu2115_set_vbusovp,
	.set_ibusocp = nu2115_set_ibusocp,
	.set_vbatovp = nu2115_set_vbatovp,
	.set_ibatocp = nu2115_set_ibatocp,
	.init_chip = nu2115_init_chip,
	.set_vbatovp_alarm = nu2115_set_vbatovp_alarm,
	.reset_vbatovp_alarm = nu2115_reset_vbatovp_alarm,
	.set_vbusovp_alarm = nu2115_set_vbusovp_alarm,
	.reset_vbusovp_alarm = nu2115_reset_vbusovp_alarm,
	.is_vbuslowerr = nu2115_is_vbuslowerr,
	.get_adc_accuracy = nu2115_get_adc_accuracy,
	.is_enable_otg = nu2115_enable_otg,
	.is_enable_acdrv1 = nu2115_enable_acdrv1,
	.is_vbushigherr= mtk_nu2115_is_vbushigher,
#if IS_ENABLED(CONFIG_OEM_TURBO_CHARGER)
	.is_vbat_present = mtk_nu2115_is_vbat_present,
	.is_vbus_present = mtk_nu2115_is_vbus_present,
#endif
	//.set_chg_mode = nu2115_set_chg_mode,
	//.get_chg_mode = nu2115_get_chg_mode,
	.config_mux = nu2115_config_mux,
};

/******************psy start****************************/
static enum power_supply_property nu2115_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int nu2115_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct nu2115 *chip = power_supply_get_drvdata(psy);
	int result;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = nu2115_is_charger_enabled(chip, &chip->charger_enable);
		val->intval = chip->charger_enable;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = __nu2115_get_adc(chip, NU2115_ADC_VBUS, &result);
		if (!ret)
			chip->vbus_uv = result;
		val->intval = chip->vbus_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = __nu2115_get_adc(chip, NU2115_ADC_IBUS, &result);
		if (!ret)
			chip->ibus_ua = result;
		val->intval = chip->ibus_ua;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = __nu2115_get_adc(chip, NU2115_ADC_VBAT, &result);
		if (!ret)
			chip->vbat_uv = result;
		val->intval = chip->vbat_uv;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = __nu2115_get_adc(chip, NU2115_ADC_IBAT, &result);
		if (!ret)
			chip->ibat_ua = result;
		val->intval = chip->ibat_ua;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = __nu2115_get_adc(chip, NU2115_ADC_TDIE, &result);
		if (!ret)
			chip->die_temp = result - 8; //due to the temp higher sc8541
		val->intval = chip->die_temp;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "NuVolta";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_charger_set_property(struct power_supply *psy,
	enum power_supply_property prop,
	const union power_supply_propval *val)
{
	struct nu2115 *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		 __nu2115_enable_chg(chip, val->intval);
		dev_info(chip->dev, "POWER_SUPPLY_PROP_ONLINE: %s\n",
			val->intval ? "enable" : "disable");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{

	return 0;
}

static const struct power_supply_desc nu2115_psy_desc = {
	.name = "cp-standalone",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = nu2115_charger_props,
	.num_properties = ARRAY_SIZE(nu2115_charger_props),
	.get_property = nu2115_charger_get_property,
	.set_property = nu2115_charger_set_property,
	.property_is_writeable = nu2115_charger_is_writeable,
};

static int nu2115_psy_register(struct nu2115 *chip)
{
	chip->psy_cfg.drv_data = chip;
	chip->psy_cfg.of_node = chip->dev->of_node;

	memcpy(&chip->psy_desc, &nu2115_psy_desc, sizeof(chip->psy_desc));
	chip->psy = devm_power_supply_register(chip->dev, &chip->psy_desc, &chip->psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(chip->dev, "%s failed to register psy\n", __func__);
		return PTR_ERR(chip->psy);
	}

	dev_info(chip->dev, "%s power supply register successfully\n", chip->psy_desc.name);
	return 0;
}
/******************psy end****************************/
/* debugfs interface */

static ssize_t show_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int state = 0;
	struct nu2115 *chip = dev_get_drvdata(dev);
	if (!chip) {
		pr_err("nu2115: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	state = chip->force_chg_enable_flag;
	pr_info("show nu2115  %s charging \n",
			   state ? "enable" : "disable");
end:
	return sprintf(buf, "%d\n", chip->force_chg_enable_flag);
}

static ssize_t store_force_chg_auto_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int enable;
	struct nu2115 *chip = dev_get_drvdata(dev);
	if (!chip) {
		pr_err("nu2115 chip not valid\n");
		return -ENODEV;
	}
	enable = simple_strtoul(buf, NULL, 0);
	ret = nu2115_enable_chg(chip->chg_dev, enable);
	if (ret) {
		pr_err("nu2115 Couldn't %s charging rc=%d\n",
			   enable ? "enable" : "disable", (int)ret);
		return ret;
	}

	pr_info("nu2115  %s charging \n",
			   enable ? "enable" : "disable");

	chip->force_chg_enable_flag = enable;

	return count;
}
static DEVICE_ATTR(force_chg_auto_enable, 0664, show_force_chg_auto_enable, store_force_chg_auto_enable);

static void nu2115_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_force_chg_auto_enable);
}

static int debugfs_get_data(void *data, u64 *val)
{
	struct nu2115 *chip = data;
	int ret;
	u8 temp;

	ret = __nu2115_read(chip, chip->debug_addr, &temp);
	if (ret)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct nu2115 *chip = data;
	int ret;
	u8 temp;

	temp = (u8)val;
	ret = __nu2115_write(chip, chip->debug_addr, temp);
	if (ret)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct nu2115 *chip = m->private;
	u8 reg, val;
	int ret;

	for (reg = NU2115_REG_00; reg <= NU2115_REG_36; reg++) {
		ret = __nu2115_read(chip, reg, &val);
		if (ret) {
			seq_printf(m, "0x%02x = error\n", reg);
			continue;
		}

		seq_printf(m, "0x%02x = 0x%02x\n", reg, val);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct nu2115 *chip = inode->i_private;

	return single_open(file, dump_debugfs_show, chip);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct nu2115 *chip)
{
	struct dentry *ent;

	chip->debugfs = debugfs_create_dir(chip->chg_prop.alias_name, NULL);
	if (!chip->debugfs) {
		dev_err(chip->dev, "failed to create debugfs\n");
		return -ENODEV;
	}

	debugfs_create_x8("addr", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, &chip->debug_addr);

	ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, chip, &data_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create data debugfs\n");

	ent = debugfs_create_file("dump", S_IFREG | S_IRUGO,
		chip->debugfs, chip, &dump_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create dump debugfs\n");

	return 0;
}

static int nu2115_charger_device_register(struct nu2115 *chip)
{
	chip->chg_prop.alias_name = "nu2115_standalone";
	chip->chg_dev = charger_device_register("primary_dvchg",
			chip->dev, chip, &nu2115_chg_ops, &chip->chg_prop);
	if (!chip->chg_dev)
		return -EINVAL;

	return 0;
}

static int nu2115_irq_init(struct nu2115 *chip)
{
	struct gpio_desc *irq_gpio;
	int irq;
	int ret = 0;

	irq_gpio = devm_gpiod_get(chip->dev, "nu2115,intr", GPIOD_IN);
	if (IS_ERR(irq_gpio))
		return PTR_ERR(irq_gpio);

	irq = gpiod_to_irq(irq_gpio);
	if (irq < 0) {
		dev_err(chip->dev, "%s irq mapping fail(%d)\n", __func__, irq);
		return ret;
	}
	dev_info(chip->dev, "%s irq = %d\n", __func__, irq);


	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					nu2115_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"nu2115_irq", chip);
	if (ret) {
		dev_err(chip->dev, "failed to request irq %d\n", irq);
		return ret;
	}

	return ret;
}


static int nu2115_hw_init(struct nu2115 *chip)
{
	int ret = 0;

	ret = __nu2115_set_bat_ucp_alm(chip, 2000000);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set bat_ucp_alm\n");
		return ret;
	}

	ret = __nu2115_set_ac_ovp(chip, chip->cfg.ac_ovp);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set ac_ovp\n");
		return ret;
	}

	ret =  __nu2115_set_bus_ovp(chip, chip->cfg.vbus_ovp);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set vbus_ovp\n");
		return ret;
	}

	ret = __nu2115_set_bus_ovp_alm(chip, chip->cfg.vbus_ovp);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set bus_ovp_alm\n");
		return ret;
	}

	ret = __nu2115_set_bat_ovp(chip, 4650000);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set bat_ovp\n");
		return ret;
	}

	ret = __nu2115_set_bus_ocp(chip, 4700000);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set bus_ocp\n");
		return ret;
	}
	//ret = __nu2115_set_fsw(chip, 500000);
	/*
	ret = __nu2115_set_fsw(chip, 400000);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set fsw\n");
		return ret;
	}
	*/

	ret = __nu2115_set_watchdog(chip, 500000);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set watchdog\n");
		return ret;
	}

	ret = __nu2115_enable_watchdog(chip, false);
	if (ret < 0) {
		dev_err(chip->dev, "failed to disable watchdog\n");
		return ret;
	}

	ret = __nu2115_set_adc_fn_dis(chip, 0x06);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set adc channel\n");
		return ret;
	}

	ret = __nu2115_set_ibat_sns_res(chip, 2);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set ibat_sns_res\n");
		return ret;
	}

	ret = __nu2115_set_irqmask(chip, NU2115_REG_14, 0x7);
	if (ret < 0) {
		dev_err(chip->dev, "failed to set mask\n");
		return ret;
	}

	ret = __nu2115_update_bits(chip, NU2115_REG_32,
		NU2115_PMID2VOUT_UVP_MASK|NU2115_PMID2VOUT_OVP_MASK, 0xF0);

	ret =  __nu2115_write(chip, NU2115_REG_00, 0x80);//BATOVP 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_01, 0x80);//BATOVP_ALM 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_02, 0x80);//BATOCP 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_03, 0x80);//BATOCP_ALM 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_04, 0x80);//BATUCP_ALM 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_0A, 0x80);//BUSOCP_ALM 0x80:disable
	ret =  __nu2115_write(chip, NU2115_REG_0E, 0x06);
	ret =  __nu2115_write(chip, NU2115_REG_2F, 0x00);
	ret =  __nu2115_write(chip, NU2115_REG_35, 0xC0);
	/* clear irqs */
	nu2115_check_status_flags(chip);

	return ret;
}

static int nu2115_parse_dt(struct nu2115 *chip)
{
	struct device_node *np = chip->dev->of_node;
	int ret;

	if (!np)
		return -ENODEV;

	ret = of_property_read_u32(np, "ac_ovp", &chip->cfg.ac_ovp);
	if (ret)
		chip->cfg.ac_ovp = 13500000;
	ret = of_property_read_u32(np, "vbus_ovp", &chip->cfg.vbus_ovp);
	if (ret)
		chip->cfg.ac_ovp = 9000000;
	ret = of_property_read_u32(np, "bat_ucp_alm", &chip->cfg.bat_ucp_alm);
	if (ret)
		chip->cfg.bat_ucp_alm = 2000000;
	ret = of_property_read_u32(np, "ss_timeout", &chip->cfg.ss_timeout);
	if (ret)
		chip->cfg.ss_timeout = 0;
	ret = of_property_read_u32(np, "fsw_set", &chip->cfg.fsw_set);
	if (ret)
		chip->cfg.fsw_set = 500000;
	ret = of_property_read_u32(np, "ibat_sns_res", &chip->cfg.ibat_sns_res);
	if (ret)
		chip->cfg.ibat_sns_res = 2;

	chip->cfg.watchdog_dis = of_property_read_bool(np, "watchdog_dis");
	ret = of_property_read_u32(np, "watchdog", &chip->cfg.watchdog);
	if (ret)
		chip->cfg.watchdog = 30000000;
	if (!chip->cfg.watchdog)
		chip->cfg.watchdog_dis = true;

	return 0;
}

static void determine_initial_status(struct nu2115 *chip)
{
	if (chip->client->irq)
		nu2115_irq_handler(chip->client->irq, chip);
}

static bool nu2115_detect_device(struct nu2115 *chip)
{
	int ret;
	u8 val;

	ret = __nu2115_read(chip, NU2115_REG_31, &val);
	dev_info(chip->dev,"%s ret=%d ", __func__,ret);
	dev_err(chip->dev, "nu2115_detect_device---val=0x%x\n",val);
	if (val == NU2115_DEVID)
		return true;
	else
		return false;
}

static int nu2115_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct nu2115 *chip;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, chip);
	chip->client = client;
	//chip->client->addr = 0x66;
	chip->dev = &client->dev;
	mutex_init(&chip->rw_lock);
	mutex_init(&chip->ops_lock);
	mutex_init(&chip->adc_lock);
	mutex_init(&chip->irq_lock);

	ret = nu2115_detect_device(chip);
	if(!ret) {
		dev_err(&client->dev, "nu2115_detect_device failed.\n");
		return ret;
	}

	ret = nu2115_parse_dt(chip);
	if (ret) {
		dev_err(&client->dev, "nu2115_parse_dt failed.\n");
		return ret;
	}

	ret = nu2115_init_device(chip);
	if (ret < 0) {
		dev_err(&client->dev, "nu2115_init_device failed\n");
		return ret;
	}

	ret = nu2115_hw_init(chip);
	if (ret) {
		dev_err(&client->dev, "nu2115_hw_init failed.\n");
		return ret;
	}

	ret = nu2115_psy_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s psy register failed(%d)\n", __func__, ret);
		return ret;
	}

	ret = nu2115_irq_init(chip);
	if (ret) {
		dev_err(&client->dev, "nu2115_irq_init failed.\n");
		return ret;
	}

	determine_initial_status(chip);

	ret = nu2115_charger_device_register(chip);
	if (ret) {
		dev_err(&client->dev, "nu2115_charger_device_register failed.\n");
		return ret;
	}
	chip->force_chg_enable_flag = 0;
	create_debugfs_entries(chip);
	nu2115_create_device_node(chip->dev);

#if IS_ENABLED(CONFIG_OEM_DEVINFO)
	FULL_PRODUCT_DEVICE_INFO(ID_CHARGER_PUMP, "NU2115");
#endif

	g_dev = chip;
	dev_err(&client->dev, "nu2115_charger probe OK.\n");
	__nu2115_dump_register(chip);

	return ret;
}

static int nu2115_remove(struct i2c_client *client)
{
	struct nu2115 *chip = i2c_get_clientdata(client);

	charger_device_unregister(chip->chg_dev);

	return 0;
}

static void nu2115_shutdown(struct i2c_client *client)
{
	struct nu2115 *chip = i2c_get_clientdata(client);

	/*reg reset*/
	__nu2115_reg_reset(chip);

	dev_err(chip->dev,"Shutdown Successfully\n");
}

static const struct of_device_id nu2115_of_match[] = {
	{ .compatible = "cp,nu2115", },
	{ },
};

static const struct i2c_device_id nu2115_i2c_id[] = {
	{ .name = "nu2115", },
	{ },
};

static struct i2c_driver nu2115_driver = {
	.probe = nu2115_probe,
	.remove = nu2115_remove,
	.shutdown = nu2115_shutdown,
	.driver = {
		.name = "nu2115",
		.of_match_table = nu2115_of_match,
	},
	.id_table = nu2115_i2c_id,
};
module_i2c_driver(nu2115_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("bing");
MODULE_DESCRIPTION("NuVolta NU2115 Switched Cap Fast Charger");
MODULE_VERSION("1.0");
