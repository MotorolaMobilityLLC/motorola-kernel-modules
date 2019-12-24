// SPDX-License-Identifier: GPL-2.0+
/*
 * SLG51000 High PSRR, Multi-Output Regulators
 * Copyright (C) 2019  Dialog Semiconductor
 *
 * Author: Eric Jeong, Dialog Semiconductor
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include "slg51000-regulator.h"

#define SLG51000_REGULATOR_LDO1	        0
#define SLG51000_REGULATOR_LDO2	        1
#define SLG51000_REGULATOR_LDO3	        2
#define SLG51000_REGULATOR_LDO4	        3
#define SLG51000_REGULATOR_LDO5	        4
#define SLG51000_REGULATOR_LDO6	        5
#define SLG51000_REGULATOR_LDO7	        6
#define SLG51000_MAX_REGULATORS	        7
#define SLG51000_SCTL_EVT               7
#define SLG51000_MAX_EVT_REGISTER       8

#define SLG51000_LDOHP_LV_MIN           1200000
#define SLG51000_LDOHP_HV_MIN           2400000

struct slg51000_pdata {
	int ena_gpio;
};

struct slg51000 {
	struct device *dev;
	struct regmap *regmap;
	struct slg51000_pdata regl_pdata[SLG51000_MAX_REGULATORS];
	struct regulator_desc *rdesc[SLG51000_MAX_REGULATORS];
	struct regulator_dev *rdev[SLG51000_MAX_REGULATORS];
	int chip_irq;
	int chip_cs_pin;
	int use_default_voltage;
};

struct slg51000_evt_sta {
	unsigned int ereg;
	unsigned int sreg;
};

static const struct slg51000_evt_sta es_reg[SLG51000_MAX_EVT_REGISTER] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static const struct regmap_range slg51000_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			 SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
};

static const struct regmap_range slg51000_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			 SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			 SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			 SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			 SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			 SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			 SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			 SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_MISC1, SLG51000_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_MISC1, SLG51000_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_CONF1, SLG51000_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_CONF1, SLG51000_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_TRIM2, SLG51000_LDO5_TRIM2),
	regmap_reg_range(SLG51000_LDO5_CONF1, SLG51000_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_TRIM2, SLG51000_LDO6_TRIM2),
	regmap_reg_range(SLG51000_LDO6_CONF1, SLG51000_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_CONF1, SLG51000_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_LOCK_OTP_PROG, SLG51000_OTP_LOCK_CTRL),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			 SLG51000_LOCK_GLOBAL_LOCK_CTRL1),
};

static const struct regmap_range slg51000_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
};

static const struct regmap_access_table slg51000_writeable_table = {
	.yes_ranges	= slg51000_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_writeable_ranges),
};

static const struct regmap_access_table slg51000_readable_table = {
	.yes_ranges	= slg51000_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_readable_ranges),
};

static const struct regmap_access_table slg51000_volatile_table = {
	.yes_ranges	= slg51000_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_volatile_ranges),
};

static const struct regmap_config slg51000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x8000,
	.wr_table = &slg51000_writeable_table,
	.rd_table = &slg51000_readable_table,
	.volatile_table = &slg51000_volatile_table,
};

static int slg51000_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct slg51000 *chip = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned int state;

	ret = regmap_read(chip->regmap, es_reg[id].sreg, &state);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read status register(%d)\n",
			ret);
		return ret;
	}

	if (!(state & SLG51000_STA_ILIM_FLAG_MASK)
	   && (state & SLG51000_STA_VOUT_OK_FLAG_MASK))
		return 1;
	else
		return 0;
}

static int slg51000_regulator_set_voltage_sel(struct regulator_dev *rdev, unsigned sel) {
	struct slg51000 *chip = rdev_get_drvdata(rdev);

	if (chip->use_default_voltage) {
		return 0;
	}
	return regulator_set_voltage_sel_regmap(rdev, sel);
}

static struct regulator_ops slg51000_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = slg51000_regulator_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = slg51000_regulator_set_voltage_sel,
};

static struct regulator_ops slg51000_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = slg51000_regulator_is_enabled,
};

static int slg51000_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct slg51000 *chip = config->driver_data;
	struct slg51000_pdata *rpdata = &chip->regl_pdata[desc->id];

	rpdata->ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (rpdata->ena_gpio > 0)
		config->ena_gpio = rpdata->ena_gpio;

	return 0;
}

#define SLG51000_REGL_DESC(_id, _name, _nvolt, _msel, _min, _step) \
	[SLG51000_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.id = SLG51000_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = slg51000_of_parse_cb,               \
		.ops = &slg51000_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = _nvolt,                              \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = _msel,                           \
		.vsel_mask = SLG51000_VSEL_MASK,                   \
		.vsel_reg = SLG51000_##_id##_VSEL,                 \
		.enable_reg = SLG51000_SYSCTL_MATRIX_CONF_A,       \
		.enable_mask = BIT(SLG51000_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc regls_desc[SLG51000_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, 256, 0, 2400000, 5000),
	SLG51000_REGL_DESC(LDO2, ldo2, 256, 0, 2400000, 5000),
	SLG51000_REGL_DESC(LDO3, ldo3, 256, 0, 1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, 256, 0, 1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, 256, 0, 400000, 5000),
	SLG51000_REGL_DESC(LDO6, ldo6, 256, 0, 400000, 5000),
	SLG51000_REGL_DESC(LDO7, ldo7, 256, 0, 1200000, 10000),
};

static int slg51000_regulator_init(struct slg51000 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	unsigned int reg, val;
	u8 vsel_range[2];
	int id, ret = 0;
	const unsigned int min_regs[SLG51000_MAX_REGULATORS] = {
		SLG51000_LDO1_MINV, SLG51000_LDO2_MINV, SLG51000_LDO3_MINV,
		SLG51000_LDO4_MINV, SLG51000_LDO5_MINV, SLG51000_LDO6_MINV,
		SLG51000_LDO7_MINV,
	};

	for (id = 0; id < SLG51000_MAX_REGULATORS; id++) {
		rdesc = chip->rdesc[id] = &regls_desc[id];

		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, min_regs[id],
				       vsel_range, 2);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the MIN register\n");
			return ret;
		}

		switch (id) {
		case SLG51000_REGULATOR_LDO1:
		case SLG51000_REGULATOR_LDO2:
			if (id == SLG51000_REGULATOR_LDO1)
				reg = SLG51000_LDO1_MISC1;
			else
				reg = SLG51000_LDO2_MISC1;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read voltage range of ldo%d\n",
					id + 1);
				return ret;
			}

			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			if (val & SLG51000_SEL_VRANGE_MASK)
				rdesc->min_uV = SLG51000_LDOHP_HV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			else
				rdesc->min_uV = SLG51000_LDOHP_LV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			break;

		case SLG51000_REGULATOR_LDO5:
		case SLG51000_REGULATOR_LDO6:
			if (id == SLG51000_REGULATOR_LDO5)
				reg = SLG51000_LDO5_TRIM2;
			else
				reg = SLG51000_LDO6_TRIM2;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read LDO mode register\n");
				return ret;
			}

			if (val & SLG51000_SEL_BYP_MODE_MASK) {
				rdesc->ops = &slg51000_switch_ops;
				rdesc->n_voltages = 0;
				rdesc->min_uV = 0;
				rdesc->uV_step = 0;
				rdesc->linear_min_sel = 0;
				break;
			}
			/* Fall through for else case */

		default:
			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			rdesc->min_uV = rdesc->min_uV
					+ (vsel_range[0] * rdesc->uV_step);
			break;
		}

		chip->rdev[id] = devm_regulator_register(chip->dev, rdesc,
							 &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev,
				"Failed to register regulator(%s):%d\n",
				chip->rdesc[id]->name, ret);
			return ret;
		}
	}

	return 0;
}

static irqreturn_t slg51000_irq_handler(int irq, void *data)
{
	struct slg51000 *chip = data;
	struct regmap *regmap = chip->regmap;
	u8 evt[SLG51000_MAX_EVT_REGISTER][3];
	int ret, i, handled = IRQ_NONE;

	/* Read event[0], status[1] and mask[2] register */
	for (i = 0; i < SLG51000_MAX_EVT_REGISTER; i++) {
		ret = regmap_bulk_read(regmap, es_reg[i].ereg, evt[i], 3);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read event registers(%d)\n", ret);
			return IRQ_NONE;
		}
	}

	for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
		if (!(evt[i][2] & SLG51000_IRQ_ILIM_FLAG_MASK)
		   && (evt[i][0] & SLG51000_EVT_ILIM_FLAG_MASK)) {
			mutex_lock(&chip->rdev[i]->mutex);
			regulator_notifier_call_chain(chip->rdev[i],
						REGULATOR_EVENT_OVER_CURRENT,
						NULL);
			mutex_unlock(&chip->rdev[i]->mutex);

			if (evt[i][1] & SLG51000_STA_ILIM_FLAG_MASK)
				dev_warn(chip->dev,
					 "Over-current limit(ldo%d)\n", i + 1);
			handled = IRQ_HANDLED;
		}
	}

	if (!(evt[SLG51000_SCTL_EVT][2] & SLG51000_IRQ_HIGH_TEMP_WARN_MASK)
	   && (evt[SLG51000_SCTL_EVT][0] & SLG51000_EVT_HIGH_TEMP_WARN_MASK)) {
		for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
			if (!(evt[i][1] & SLG51000_STA_ILIM_FLAG_MASK)
			   && (evt[i][1] & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
				mutex_lock(&chip->rdev[i]->mutex);
				regulator_notifier_call_chain(chip->rdev[i],
						REGULATOR_EVENT_OVER_TEMP,
						NULL);
				mutex_unlock(&chip->rdev[i]->mutex);
			}
		}
		handled = IRQ_HANDLED;
		if (evt[SLG51000_SCTL_EVT][1]
		   & SLG51000_STA_HIGH_TEMP_WARN_MASK)
			dev_warn(chip->dev, "High temperature warning!\n");
	}

	return handled;
}

static void slg51000_check_crc_and_clear_fault(struct slg51000 *chip)
{
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, SLG51000_OTP_EVENT, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read OTP CRC register\n");
		return;
	}

	if (val & SLG51000_EVT_CRC_MASK)
		dev_dbg(chip->dev, "OTP CRC failure is detected\n");

	ret = regmap_read(chip->regmap, SLG51000_SYSCTL_FAULT_LOG1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read Fault log register\n");
		return;
	}

	if (val & SLG51000_FLT_OVER_TEMP_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_OVER_TEMP\n");
	if (val & SLG51000_FLT_POWER_SEQ_CRASH_REQ_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POWER_SEQ_CRASH_REQ\n");
	if (val & SLG51000_FLT_RST_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_RST\n");
	if (val & SLG51000_FLT_POR_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POR\n");
}

static int slg51000_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct slg51000 *chip;
	int error, cs_gpio, ret;

	chip = devm_kzalloc(dev, sizeof(struct slg51000), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	cs_gpio = of_get_named_gpio(dev->of_node, "dlg,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_HIGH,
					    "slg51000_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n",
				cs_gpio, ret);
			return ret;
		}

		chip->chip_cs_pin = cs_gpio;
	}

	chip->use_default_voltage = of_property_read_bool(dev->of_node, "use-default-voltage");

	i2c_set_clientdata(client, chip);
	chip->chip_irq = client->irq;
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &slg51000_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	/* FIXME - Optimize Delay  */
	mdelay(10);
	ret = slg51000_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	slg51000_check_crc_and_clear_fault(chip);

	if (chip->chip_irq) {
		ret = devm_request_threaded_irq(dev, chip->chip_irq, NULL,
						slg51000_irq_handler,
						(IRQF_TRIGGER_LOW
						| IRQF_ONESHOT),
						"slg51000-irq", chip);
		if (ret != 0) {
			dev_err(dev, "Failed to request IRQ: %d\n",
				chip->chip_irq);
			return ret;
		}
	} else {
		dev_warn(dev, "No IRQ configured\n");
	}

	return ret;
}

static int slg51000_i2c_remove(struct i2c_client *client)
{
	struct slg51000 *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
	}

	return ret;
}

static const struct i2c_device_id slg51000_i2c_id[] = {
	{"slg51000", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, slg51000_i2c_id);

static struct i2c_driver slg51000_regulator_driver = {
	.driver = {
		.name = "slg51000-regulator",
	},
	.probe = slg51000_i2c_probe,
	.remove = slg51000_i2c_remove,
	.id_table = slg51000_i2c_id,
};

module_i2c_driver(slg51000_regulator_driver);

MODULE_AUTHOR("Eric Jeong <eric.jeong.opensource@diasemi.com>");
MODULE_DESCRIPTION("SLG51000 regulator driver");
MODULE_LICENSE("GPL");

