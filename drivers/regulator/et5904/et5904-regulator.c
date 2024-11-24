// SPDX-License-Identifier: GPL-2.0+
/*
 * et5904, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: Huqian Motorola Mobility LLC,
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "et5904-regulator.h"

enum slg51000_regulators {
	ET5904_REGULATOR_LDO1 = 0,
	ET5904_REGULATOR_LDO2,
	ET5904_REGULATOR_LDO3,
	ET5904_REGULATOR_LDO4,
	ET5904_MAX_REGULATORS,
};

struct et5904 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[ET5904_MAX_REGULATORS];
	struct regulator_dev *rdev[ET5904_MAX_REGULATORS];
	int chip_cs_pin;
	int init_value;
	bool shutdown_ldo;
};

struct et5904_evt_sta {
	unsigned int sreg;
};

static const struct et5904_evt_sta et5904_status_reg = { ET5904_LDO_EN };

static const struct regmap_range et5904_writeable_ranges[] = {
      /* Do not let useless register writeable */
	regmap_reg_range(ET5904_CURRENT_LIMITSEL, ET5904_SEQ_STATUS),
};

static const struct regmap_range et5904_readable_ranges[] = {
	regmap_reg_range(ET5904_CHIP_REV, ET5904_REG_CHIPID2),
};

static const struct regmap_range et5904_volatile_ranges[] = {
	regmap_reg_range(ET5904_CURRENT_LIMITSEL, ET5904_SEQ_STATUS),
};

static const struct regmap_access_table et5904_writeable_table = {
	.yes_ranges	= et5904_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5904_writeable_ranges),
};

static const struct regmap_access_table et5904_readable_table = {
	.yes_ranges	= et5904_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5904_readable_ranges),
};

static const struct regmap_access_table et5904_volatile_table = {
	.yes_ranges	= et5904_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5904_volatile_ranges),
};

static const struct regmap_config et5904_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ET5904_REG_CHIPID2,
	.wr_table = &et5904_writeable_table,
	.rd_table = &et5904_readable_table,
	.volatile_table = &et5904_volatile_table,
};

static int et5904_get_current_limit(struct regulator_dev *rdev)
{
	struct et5904 *chip = rdev_get_drvdata(rdev);
	uint8_t reg_dump[ET5904_REG_NUM];
	uint8_t reg_idx;
	unsigned int val = 0;

	dev_err(chip->dev, "************ start dump et5904 register ************\n");
	dev_err(chip->dev, "register name =%s \n",rdev->desc->name);
	dev_err(chip->dev, "register 0x00:      chip version\n");
	dev_err(chip->dev, "register 0x01:      LDO CL\n");
	dev_err(chip->dev, "register 0x03~0x06: LDO1~LDO4 OUT Voltage\n");
	dev_err(chip->dev, "register 0x0e:      Bit[3:0] LDO4~LDO1 EN\n");

	for (reg_idx = 0; reg_idx < ET5904_REG_NUM; reg_idx++) {
		regmap_read(chip->regmap, reg_idx, &val);
		reg_dump[reg_idx] = val;
		dev_err(chip->dev, "Reg[0x%02x] = 0x%x", reg_idx, reg_dump[reg_idx]);
	}
	dev_err(chip->dev, "************ end dump et5904  register ************\n");

	return 0;
}

static int et5904_get_status(struct regulator_dev * rdev)
{
	struct et5904 *chip = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned int status = 0;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read enable register(%d)\n",
			ret);
		return ret;
	}

	if (!ret)
		return REGULATOR_STATUS_OFF;

	et5904_get_current_limit(rdev);

	ret = regmap_read(chip->regmap, et5904_status_reg.sreg, &status);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read status register(%d)\n",
			ret);
		return ret;
	}

	if (status & (0x01ul << id)) {
		return REGULATOR_STATUS_ON;
	} else {
		return REGULATOR_STATUS_OFF;
	}
}

static const struct regulator_ops et5904_regl_ops = {
	.enable = regulator_enable_regmap,
#ifdef PICOLEAF_DATA_EN
	.disable = NULL,
#else
	.disable = regulator_disable_regmap,
#endif
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_status = et5904_get_status,
	.get_current_limit = et5904_get_current_limit,
};

static int et5904_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	int ena_gpio;

	ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(ena_gpio))
		config->ena_gpiod = gpio_to_desc(ena_gpio);

	return 0;
}

#define ET5904_REGL_DESC(_id, _name, _s_name, _min, _step)       \
	[ET5904_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = ET5904_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = et5904_of_parse_cb,               \
		.ops = &et5904_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = ET5904_VSEL_MASK,                   \
		.vsel_reg = ET5904_##_id##_VSEL,                 \
		.enable_reg = ET5904_LDO_EN,       \
		.enable_mask = BIT(ET5904_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc et5904_regls_desc[ET5904_MAX_REGULATORS] = {
	ET5904_REGL_DESC(LDO1, ldo1, "vin1", 600000, 6000),
	ET5904_REGL_DESC(LDO2, ldo2, "vin1", 600000, 6000),
	ET5904_REGL_DESC(LDO3, ldo3, "vin2", 1200000, 12500),
	ET5904_REGL_DESC(LDO4, ldo4, "vin2", 1200000, 12500),
};

static int et5904_regulator_init(struct et5904 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	u8 vsel_range[1];
	int id, value, ret = 0;
	const unsigned int ldo_regs[ET5904_MAX_REGULATORS] = {
		ET5904_LDO1_VOUT,
		ET5904_LDO2_VOUT,
		ET5904_LDO3_VOUT,
		ET5904_LDO4_VOUT,
	};

	unsigned int initial_voltage[ET5904_MAX_REGULATORS] = {
		0x4b,//LDO1  DVDD 1.05V
		0x54,//LDO2 DVDD 1.1V
		0x80,//LDO3 AVDD 2.8V
		0x80,//LDO4 AVDD 2.8V
	};


	if(of_property_read_u32(chip->dev->of_node, "ldo1,init-value", &value) == 0){
		initial_voltage[0] = value;
	}

	if(of_property_read_u32(chip->dev->of_node, "ldo2,init-value", &value) == 0){
		initial_voltage[1] = value;
	}

	if(of_property_read_u32(chip->dev->of_node, "ldo3,init-value", &value) == 0){
		initial_voltage[2] = value;
	}

	if(of_property_read_u32(chip->dev->of_node, "ldo4,init-value", &value) == 0){
		initial_voltage[3] = value;
	}

	/*Disable all ldo output by default*/
	ret = regmap_write(chip->regmap, ET5904_LDO_EN, chip->init_value);
	if (ret < 0) {
		dev_err(chip->dev,
			"Disable all LDO output failed!!!\n");
		return ret;
	}
	/* Enable all ldo discharge by default */
	ret = regmap_write(chip->regmap, ET5904_DISCHARGE_RESISTORS, 0x8f);
	if (ret < 0) {
		dev_err(chip->dev,
			"Enable LDO discharge failed!!!\n");
		return ret;
	}
	for (id = 0; id < ET5904_MAX_REGULATORS; id++) {
		chip->rdesc[id] = &et5904_regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, ldo_regs[id],
				       vsel_range, 1);
		pr_err("et5904_regulator_init: LDO%d, default value:0x%x", (id+1), vsel_range[0]);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the ldo register\n");
			return ret;
		}

		ret = regmap_write(chip->regmap, ldo_regs[id], initial_voltage[id]);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to write inital voltage register\n");
			return ret;
		}
		pr_err("et5904_regulator_init: LDO%d, initial value:0x%x", (id+1), initial_voltage[id]);

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

static int et5904_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct et5904 *chip;
	int error, cs_gpio, ret, i, value;
	unsigned int chip_data = 0x00;
#ifdef PICOLEAF_DATA_EN
	unsigned int val;
#endif

	/* Set all register to initial value when probe driver to avoid register value was modified.
	*/
	const unsigned int initial_register[5][2] = {
		{ET5904_CURRENT_LIMITSEL, 	0x00},
		{ET5904_DISCHARGE_RESISTORS, 	0x00},
		{ET5904_LDO1_LDO2_SEQ, 	0x00},
		{ET5904_LDO3_LDO4_SEQ, 	0x00},
		{ET5904_SEQ_STATUS, 		0x00},
	};
	chip = devm_kzalloc(dev, sizeof(struct et5904), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "et5904_i2c_probe Memory error...\n");
		return -ENOMEM;
	}

	dev_info(chip->dev, "et5904_i2c_probe Enter...\n");

	if (of_property_read_u32(dev->of_node, "etek,init-value", &value) < 0) {
		dev_info(chip->dev, "et5904_i2c_probe no init_value, use default 0x0\n");
		value = 0x0;
	}
	chip->init_value = value;
	dev_info(chip->dev, "et5904_i2c_probe init_value:%d...\n", value);
	chip->shutdown_ldo = of_property_read_bool(dev->of_node, "etek,shutdown-ldo");

	mdelay(10);

	i2c_set_clientdata(client, chip);
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &et5904_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}


	ret = regmap_read(chip->regmap, ET5904_REG_CHIPID2, &chip_data);

	dev_err(chip->dev, "0x19 data : %d\n", chip_data);

	if (ret < 0) {
		dev_err(chip->dev, "Failed to read DEVICE_ID reg: %d\n", ret);
		return ret;
	}

	if(0x00 != chip_data){
		dev_info(chip->dev, "This is no et5904 ic ,probe return\n");
		return 0;
	}

	cs_gpio = of_get_named_gpio(dev->of_node, "etek,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_LOW,
					    "et5904_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n",
				cs_gpio, ret);
			return ret;
		}

		chip->chip_cs_pin = cs_gpio;
	}

	dev_info(chip->dev, "et5904_i2c_probe cs_gpio:%d...\n", cs_gpio);


	for (i = 0; i < 5; i++) {
		ret = regmap_write(chip->regmap, initial_register[i][0], initial_register[i][1]);
		if (ret < 0) {
			dev_err(chip->dev,"Failed to write register: 0x%x, value: 0x%x \n",
				initial_register[i][0], initial_register[i][1]);
		}

		dev_err(chip->dev,"Success to write register: 0x%x, value: 0x%x \n",
			initial_register[i][0], initial_register[i][1]);
	}

	ret = et5904_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	et5904_get_current_limit(chip->rdev[0]);
#ifdef PICOLEAF_DATA_EN
	dev_info(chip->dev, "overwrite 0E to 0D\n");
	ret = regmap_write(chip->regmap, ET5904_LDO_EN, 0xd);
	if (ret < 0) {
			dev_err(chip->dev,"Failed to write register: 0x%x\n",
				ET5904_LDO_EN);
		}
	regmap_read(chip->regmap, ET5904_LDO_EN, &val);
	dev_info(chip->dev, "0x0E data : %d\n", val);
#endif
	dev_info(chip->dev, "et5904_i2c_probe Exit...\n");

	return ret;
}

static int et5904_i2c_remove(struct i2c_client *client)
{
	struct et5904 *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
	}

	return ret;
}

static void et5904_i2c_shutdown(struct i2c_client *client)
{
	struct et5904 *chip = i2c_get_clientdata(client);
	unsigned int val = 0;

	if (chip->shutdown_ldo) {
		regmap_read(chip->regmap, ET5904_LDO_EN, &val);
		/* Disable AVDD1 when shutdown to meet device SPEC and avoid current leak */
		regmap_write(chip->regmap, ET5904_LDO_EN, val & ~(1<<2));
		dev_info(chip->dev, "et5904_i2c_shutdown");
	}
	if (chip) {
		regmap_write(chip->regmap, ET5904_LDO_EN, 0);
		dev_err(chip->dev, "et5904_i2c_shutdown force disable all LDOs");
	}
}

static const struct i2c_device_id et5904_i2c_id[] = {
	{"et5904", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, et5904_i2c_id);

static struct i2c_driver et5904_regulator_driver = {
	.driver = {
		.name = "et5904-regulator",
	},
	.probe = et5904_i2c_probe,
	.remove = et5904_i2c_remove,
	.shutdown = et5904_i2c_shutdown,
	.id_table = et5904_i2c_id,
};

module_i2c_driver(et5904_regulator_driver);

MODULE_AUTHOR("Huqian <huqian4@motorola.com>");
MODULE_DESCRIPTION("ET5904 regulator driver");
MODULE_LICENSE("GPL");
