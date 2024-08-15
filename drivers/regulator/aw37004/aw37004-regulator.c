// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023 awinic. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define AW37004_REG_ID			0x00
#define AW37004_REG_ILIMIT		0x01
#define AW37004_REG_RDIS		0x02
#define AW37004_REG_DVDD1_VOUT		0x03
#define AW37004_REG_DVDD2_VOUT		0x04
#define AW37004_REG_AVDD1_VOUT		0x05
#define AW37004_REG_AVDD2_VOUT		0x06
#define AW37004_REG_DVDD_SEQ		0x0A
#define AW37004_REG_AVDD_SEQ		0x0B
#define AW37004_REG_ENABLE		0x0E
#define AW37004_REG_SEQCR		0x0F
#define AW37004_REG_ID2			0x19

#define AW37004_ILIMIT_STEP		140000

#define AW37004_VOUT_MASK		0xFF
#define AW37004_VOUT_N_VOLTAGE		0xFF

#define AW37004_DVDD1_EN_MASK		BIT(0)
#define AW37004_DVDD2_EN_MASK		BIT(1)
#define AW37004_AVDD1_EN_MASK		BIT(2)
#define AW37004_AVDD2_EN_MASK		BIT(3)
#define AW37004_ALL_EN_MASK		GENMASK(3, 0)

#define AW37004_DISCHG_MODE_MASK	BIT(7)
#define AW37004_DISCHG_AUTO		0
#define AW37004_DISCHG_MANUAL		BIT(7)

#define AW37004_ID			0x00
#define AW37004_ID2			0x04

#define AW37004_DRIVER_VERSION		"V1.0.0"

enum aw37004_regulator_ids {
	AW37004_DVDD1,
	AW37004_DVDD2,
	AW37004_AVDD1,
	AW37004_AVDD2,
	AW37004_MAX_REGULATORS,
};

static const struct linear_range aw37004_vol_range[] = {
	REGULATOR_LINEAR_RANGE(600000, 0x00, 0xff, 6000),
	REGULATOR_LINEAR_RANGE(600000, 0x00, 0xff, 6000),
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0xff, 12500),
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0xff, 12500),
};

struct aw37004_reg_pdata {
	unsigned int init_voltage;
	unsigned int ilimit_ua;
};

struct aw37004_regulator {
	struct device *dev;
	struct regmap *regmap;
	u32 en_gpiod;
	struct aw37004_reg_pdata reg_pdata[AW37004_MAX_REGULATORS];
};

static int aw37004_regulator_enable(struct aw37004_regulator *chip,
				    const char *buf,
				    bool enable)
{
	int ret, gpio_flag;


	chip->en_gpiod = of_get_named_gpio(chip->dev->of_node, "enable-gpio", 0);
	if (!gpio_is_valid(chip->en_gpiod)) {
		dev_err(chip->dev, "enable-gpio not specified\n");
		return -EINVAL;
	}

	ret = gpio_request(chip->en_gpiod, "aw37004_en");
	if (ret < 0) {
		dev_err(chip->dev, "aw37004 enable-gpio request failed\n");
		return ret;
	}

	gpio_flag = of_property_read_bool(chip->dev->of_node,"enable-gpio-low");

	if (gpio_flag)
		gpio_direction_output(chip->en_gpiod, 0);
	else
		gpio_direction_output(chip->en_gpiod, 1);

	return 0;


	if (strncmp(buf, "dvdd1", 5) == 0) {
		if (enable)
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_DVDD1_EN_MASK,
						AW37004_DVDD1_EN_MASK);
		else
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_DVDD1_EN_MASK,
						0);

		if (ret)
			return ret;

	} else if (strncmp(buf, "dvdd2", 5) == 0) {
		if (enable)
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_DVDD2_EN_MASK,
						AW37004_DVDD2_EN_MASK);
		else
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_DVDD2_EN_MASK,
						0);

		if (ret)
			return ret;
	} else if (strncmp(buf, "avdd1", 5) == 0) {
		if (enable)
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_AVDD1_EN_MASK,
						AW37004_AVDD1_EN_MASK);
		else
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_AVDD1_EN_MASK,
						0);

		if (ret)
			return ret;
	} else if (strncmp(buf, "avdd2", 5) == 0) {
		if (enable)
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_AVDD2_EN_MASK,
						AW37004_AVDD2_EN_MASK);
		else
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_AVDD2_EN_MASK,
						0);

		if (ret)
			return ret;
	} else if (strncmp(buf, "all", 3) == 0) {
		if (enable)
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_ALL_EN_MASK,
						AW37004_ALL_EN_MASK);
		else
			ret = regmap_update_bits(chip->regmap,
						AW37004_REG_ENABLE,
						AW37004_ALL_EN_MASK,
						0);

		if (ret)
			return ret;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int aw37004_regulator_init(struct aw37004_regulator *chip)
{
	int i, ret;
	unsigned int reg_val, val, min, step;
	struct aw37004_reg_pdata *rpdata;

	for (i = 0; i < AW37004_MAX_REGULATORS; i++) {
		rpdata = &chip->reg_pdata[i];

		val = rpdata->init_voltage;
		if (i < 2) {
			min = 600000;
			step = 6000;
		} else {
			min = 1200000;
			step = 12500;
		}

		reg_val = (val - min) / step;
		if (reg_val > AW37004_VOUT_N_VOLTAGE)
			reg_val = AW37004_VOUT_N_VOLTAGE;
		ret = regmap_write(chip->regmap, AW37004_REG_DVDD1_VOUT + i,
				   reg_val);
		if (ret) {
			dev_err(chip->dev, "vout set failed: %d\n", ret);
			return ret;
		}

		val = rpdata->ilimit_ua;
		if (i < 2)
			min = 1300000;
		else
			min = 480000;
		step = AW37004_ILIMIT_STEP;
		reg_val = (val - min) / step;
		if (reg_val > 3)
			reg_val = 3;
		ret = regmap_update_bits(chip->regmap, AW37004_REG_ILIMIT,
			(BIT(2 * i) | BIT(2 * i + 1)), reg_val << (i * 2));
		if (ret) {
			dev_err(chip->dev, "ilimit set failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static const struct regulator_ops aw37004_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int aw37004_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct aw37004_regulator *chip = config->driver_data;
	struct aw37004_reg_pdata *rpdata = &chip->reg_pdata[desc->id];
	int ret;

	ret = of_property_read_u32(np, "init-voltage", &rpdata->init_voltage);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read init-voltage:%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "ilimit-ua", &rpdata->ilimit_ua);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read ilimit-ua:%d\n", ret);
		return ret;
	}

	return 0;
}

#define AW37004_REGULATOR_DESC(_id, _name, _supply, _min, _step)	\
	[AW37004_##_id] = {						\
		.name = "aw37004-"#_name,				\
		.supply_name = _supply,					\
		.id = AW37004_##_id,					\
		.of_match = of_match_ptr(#_name),			\
		.of_parse_cb = aw37004_of_parse_cb,			\
		.n_voltages = AW37004_VOUT_N_VOLTAGE,			\
		.ops = &aw37004_regulator_ops,				\
		.linear_ranges = &aw37004_vol_range[AW37004_##_id],	\
		.n_linear_ranges = 1,					\
		.min_uV = _min,						\
		.uV_step = _step,					\
		.vsel_mask = AW37004_VOUT_MASK,				\
		.vsel_reg = AW37004_REG_##_id##_VOUT,			\
		.enable_reg = AW37004_REG_ENABLE,			\
		.enable_mask = BIT(AW37004_##_id),			\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
	}

static const struct regulator_desc aw37004_regs_desc[AW37004_MAX_REGULATORS] = {
	AW37004_REGULATOR_DESC(DVDD1, dvdd1, "VIND", 600000, 6000),
	AW37004_REGULATOR_DESC(DVDD2, dvdd2, "VIND", 600000, 6000),
	AW37004_REGULATOR_DESC(AVDD1, avdd1, "VINA", 1200000, 12500),
	AW37004_REGULATOR_DESC(AVDD2, avdd2, "VINA", 1200000, 12500),
};

static ssize_t aw322xx_sysfs_print_reg(struct regmap *regmap,
				       unsigned char reg,
				       char *buf)
{
	unsigned int reg_val = 0;
	int ret = regmap_read(regmap, reg, &reg_val);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "%#.2x=%#.2x\n", reg, reg_val);
}

static ssize_t registers_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct aw37004_regulator *chip = dev_get_drvdata(dev);
	ssize_t ret = 0;

	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_ID, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_ILIMIT, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_RDIS, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_DVDD1_VOUT, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_DVDD2_VOUT, buf + ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_AVDD1_VOUT, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_AVDD2_VOUT, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_DVDD_SEQ, buf + ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_AVDD_SEQ, buf+ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_ENABLE, buf + ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_SEQCR, buf + ret);
	ret += aw322xx_sysfs_print_reg(chip->regmap, AW37004_REG_ID2, buf+ret);
	return ret;
}

static ssize_t registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct aw37004_regulator *chip = dev_get_drvdata(dev);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	if (reg > 0x0F || val > 255)
		return -EINVAL;

	ret = regmap_write(chip->regmap, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

static DEVICE_ATTR_RW(registers);

static struct attribute *aw37004_sysfs_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static struct attribute_group aw37004_sysfs_attr_group = {
	.attrs = aw37004_sysfs_attributes,
};

static const struct regmap_config aw37004_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW37004_REG_ID2,
};

static int aw37004_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regmap *regmap;
	struct aw37004_regulator *chip;
	int i, ret;
	unsigned int data;

	pr_info("aw37004 driver version is %s\n", AW37004_DRIVER_VERSION);

	client->addr = 0x28;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_mem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "check functionality failed\n");
		ret = -EIO;
		goto err_init;
	}

	regmap = devm_regmap_init_i2c(client, &aw37004_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to create regmap: %d\n", ret);
		goto err_init;
	}

	chip->regmap = regmap;
	chip->dev = dev;
	i2c_set_clientdata(client, chip);

	ret = regmap_read(regmap, AW37004_REG_ID, &data);
	if (ret < 0 || data != AW37004_ID) {
		dev_err(dev, "Failed to read CHIP ID: %d\n", ret);
		ret = -ENODEV;
		goto err_id;
	}

	ret = regmap_read(regmap, AW37004_REG_ID2, &data);
	if (ret < 0 || data != AW37004_ID2) {
		dev_err(dev, "Failed to read CHIP ID2: %d,CHIP ID2: %d\n", ret,data);
		ret = -ENODEV;
		goto err_id;
	}

	ret = sysfs_create_group(&dev->kobj, &aw37004_sysfs_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s error creating sysfs attr files\n", __func__);
		goto err_sysfs;
	}

	config.regmap = regmap;
	config.dev = dev;
	config.driver_data = chip;

	for (i = 0; i < AW37004_MAX_REGULATORS; i++) {
		rdev = devm_regulator_register(dev, &aw37004_regs_desc[i], &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(dev, "regulator %s register failed: %d\n",
				aw37004_regs_desc[i].name, ret);
			goto err_sysfs;
		}
	}

	ret = aw37004_regulator_init(chip);
	if (ret)
		dev_err(dev, "aw37004 regulator init failed\n");

	ret = aw37004_regulator_enable(chip, "all", 1);
	if (ret)
		dev_err(dev, "aw37004 regulator enable failed\n");

	return 0;

err_sysfs:
	sysfs_remove_group(&dev->kobj, &aw37004_sysfs_attr_group);
err_init:
err_id:
	devm_kfree(chip->dev, chip);
	chip = NULL;
err_mem:
	return ret;
}

static const struct of_device_id aw37004_match_table[] = {
	{ .compatible = "awinic,aw37004", },
	{}
};
MODULE_DEVICE_TABLE(of, aw37004_match_table);

static const struct i2c_device_id aw37004_i2c_id[] = {
	{ "aw37004", },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw37004_i2c_id);

static struct i2c_driver aw37004_regulator_driver = {
	.driver = {
		.name = "aw37004",
		.of_match_table = of_match_ptr(aw37004_match_table),
	},
	.probe = aw37004_i2c_probe,
	.id_table = aw37004_i2c_id,
};
module_i2c_driver(aw37004_regulator_driver);

MODULE_DESCRIPTION("AW37004 PMIC voltage regulator driver");
MODULE_AUTHOR("Alec li <like@awinic.com>");
MODULE_LICENSE("GPL");
