// SPDX-License-Identifier: GPL-2.0+
/*
 * wl28661d, Multi-Output Regulators
 * Copyright (C) 2026  Motorola Mobility LLC,
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
#include <linux/version.h>
#include "wl28661d-regulator.h"

static int ldo_chipid = -1;
static int wl28661d_debug = 0;
module_param(wl28661d_debug,int, 0644);

enum wl28661d_regulators {
  WL28661D_REGULATOR_LDO1 = 0,
  WL28661D_REGULATOR_LDO2,
  WL28661D_REGULATOR_LDO3,
  WL28661D_REGULATOR_LDO4,
  WL28661D_MAX_REGULATORS,
};

struct wl28661d {
  struct device *dev;
  struct regmap *regmap;
  struct regulator_desc *rdesc[WL28661D_MAX_REGULATORS];
  struct regulator_dev *rdev[WL28661D_MAX_REGULATORS];
  int chip_cs_pin;
  int chip_vin1_pin;
  int init_value;
  bool shutdown_ldo;
};

struct wl28661d_evt_sta {
  unsigned int sreg;
};

static const struct wl28661d_evt_sta wl28661d_status_reg = { WL28661D_LDO_EN };

static const struct regmap_range wl28661d_writeable_ranges[] = {
  /* Do not let useless register writeable */
  regmap_reg_range(WL28661D_CURRENT_LIMITSEL, WL28661D_SEQ_STATUS),
};

static const struct regmap_range wl28661d_readable_ranges[] = {
  regmap_reg_range(WL28661D_CHIP_REV, WL28661D_SEQ_STATUS),
};

static const struct regmap_range wl28661d_volatile_ranges[] = {
  regmap_reg_range(WL28661D_CURRENT_LIMITSEL, WL28661D_SEQ_STATUS),
};

static const struct regmap_access_table wl28661d_writeable_table = {
  .yes_ranges	= wl28661d_writeable_ranges,
  .n_yes_ranges	= ARRAY_SIZE(wl28661d_writeable_ranges),
};

static const struct regmap_access_table wl28661d_readable_table = {
  .yes_ranges	= wl28661d_readable_ranges,
  .n_yes_ranges	= ARRAY_SIZE(wl28661d_readable_ranges),
};

static const struct regmap_access_table wl28661d_volatile_table = {
  .yes_ranges	= wl28661d_volatile_ranges,
  .n_yes_ranges	= ARRAY_SIZE(wl28661d_volatile_ranges),
};

static const struct regmap_config wl28661d_regmap_config = {
  .reg_bits = 8,
  .val_bits = 8,
  .max_register = WL28661D_SEQ_STATUS,
  .wr_table = &wl28661d_writeable_table,
  .rd_table = &wl28661d_readable_table,
  .volatile_table = &wl28661d_volatile_table,
};

static int wl28661d_get_current_limit(struct regulator_dev *rdev)
{
  struct wl28661d *chip = rdev_get_drvdata(rdev);
  uint8_t reg_dump[WL28661D_REG_NUM];
  uint8_t reg_idx;
  unsigned int val = 0;
  if(chip)
  {
    if(wl28661d_debug)
    {
      dev_info(chip->dev, "************ start dump wl28661d register ************\n");
      dev_info(chip->dev, "register name =%s \n",rdev->desc->name);
      dev_info(chip->dev, "register 0x00:      chip version\n");
      dev_info(chip->dev, "register 0x01:      LDO CL\n");
      dev_info(chip->dev, "register 0x03~0x06: LDO1~LDO4 OUT Voltage\n");
      dev_info(chip->dev, "register 0x0e:      Bit[3:0] LDO4~LDO1 EN\n");

      for (reg_idx = 0; reg_idx < WL28661D_REG_NUM; reg_idx++) {
        regmap_read(chip->regmap, reg_idx, &val);
        reg_dump[reg_idx] = val;
        dev_info(chip->dev, "Reg[0x%02x] = 0x%x", reg_idx, reg_dump[reg_idx]);
      }
      dev_info(chip->dev, "************ end dump wl28661d register ************\n");
    }
    else
    {
      dev_info(chip->dev, "************ dump wl28661d register disabled************\n");
    }
  }
  else
  {
    dev_err(&rdev->dev, "************ dump wl28661d registers failed ************\n");
    return -ENOMEM;
  }
  return 0;
}

static int wl28661d_get_status(struct regulator_dev * rdev)
{
  struct wl28661d *chip = rdev_get_drvdata(rdev);
  int ret, id = rdev_get_id(rdev);
  unsigned int status = 0;

  if(chip)
  {
    ret = regulator_is_enabled_regmap(rdev);
    if (ret < 0) {
      dev_err(chip->dev, "Failed to read enable register(%d)\n", ret);
      return ret;
    }

    if (!ret)
      return REGULATOR_STATUS_OFF;

    ret = wl28661d_get_current_limit(rdev);

    if(ret < 0)
    {
      dev_err(chip->dev, "Failed to get current limit(%d)\n", ret);
      return ret;
    }

    ret = regmap_read(chip->regmap, wl28661d_status_reg.sreg, &status);
    if (ret < 0) {
      dev_err(chip->dev, "Failed to read status register(%d)\n", ret);
      return ret;
    }

    if (status & (0x01ul << id)) {
      return REGULATOR_STATUS_ON;
    } else {
      return REGULATOR_STATUS_OFF;
    }
  }
  else
  {
    dev_err(&rdev->dev, "************ wl28661d get status failed ************\n");
    return -ENOMEM;
  }
}

#if 0
//ONLY FOR DEBUG
int debug_regulator_map_voltage_linear_range(struct regulator_dev *rdev,
              int min_uV, int max_uV)
{
  int ret = 0;
  ret = regulator_map_voltage_linear_range(rdev, min_uV, max_uV);
  dev_err(&rdev->dev, "moto-debug: map_voltage ret:%d min/max:%d,%d\n", ret, min_uV, max_uV);
  return ret;
}

int debug_regulator_list_voltage_linear_range(struct regulator_dev *rdev,
          unsigned int selector)
{
  int ret=0;
  ret = regulator_list_voltage_linear_range(rdev, selector);
  dev_err(&rdev->dev, "moto-debug: list_voltage ret:%d selector:%d\n", ret, selector);
  return ret;
}

int debug_regulator_set_voltage_sel_regmap(struct regulator_dev *rdev,
          unsigned int selector)
{
  int ret = 0;
  if(selector == 0)
    return ret;
  ret = regulator_set_voltage_sel_regmap(rdev, selector);
  dev_err(&rdev->dev, "moto-debug: set_voltage ret:%d selector:%d\n", ret, selector);
  return ret;
}

int debug_regulator_enable_regmap(struct regulator_dev *rdev)
{
  int ret=0;
  ret = regulator_enable_regmap(rdev);
  dev_err(&rdev->dev, "moto-debug: regulator enable ret:%d, rmap:%p en reg:%d, mask:%d\n", ret, rdev->regmap, rdev->desc->enable_reg, rdev->desc->enable_mask);
  return ret;
}

int debug_regulator_disable_regmap(struct regulator_dev *rdev)
{
  int ret=0;
  ret = regulator_disable_regmap(rdev);
  dev_err(&rdev->dev, "moto-debug: regulator disable ret:%d\n", ret);
  return ret;
}

int debug_regulator_is_enabled_regmap(struct regulator_dev *rdev)
{
  int ret=0;
  ret = regulator_is_enabled_regmap(rdev);
  dev_err(&rdev->dev, "moto-debug: regulator is enable ret:%d\n", ret);
  return ret;
}
#endif

static const struct regulator_ops wl28661d_regl_ops = {
  .enable = regulator_enable_regmap,
  // .enable = debug_regulator_enable_regmap,
  .disable = regulator_disable_regmap,
  // .disable = debug_regulator_disable_regmap,
  .is_enabled = regulator_is_enabled_regmap,
  // .is_enabled = debug_regulator_is_enabled_regmap,
  .list_voltage = regulator_list_voltage_linear,
  // .list_voltage = debug_regulator_list_voltage_linear_range,
  .map_voltage = regulator_map_voltage_linear,
  // .map_voltage = debug_regulator_map_voltage_linear_range,
  .get_voltage_sel = regulator_get_voltage_sel_regmap,
  .set_voltage_sel = regulator_set_voltage_sel_regmap,
  // .set_voltage_sel = debug_regulator_set_voltage_sel_regmap,
  .get_status = wl28661d_get_status,
  .get_current_limit = wl28661d_get_current_limit,
};

static int wl28661d_of_parse_cb(struct device_node *np, const struct regulator_desc *desc,
                               struct regulator_config *config)
{
  int ena_gpio;

  ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
  if (gpio_is_valid(ena_gpio))
    config->ena_gpiod = gpio_to_desc(ena_gpio);

  return 0;
}

#define WL28661D_REGL_DESC(_id, _name, _s_name, _min, _step)       \
  [WL28661D_REGULATOR_##_id] = {                             \
    .name = #_name,                                    \
    .supply_name = _s_name,                            \
    .id = WL28661D_REGULATOR_##_id,                    \
    .of_match = of_match_ptr(#_name),                  \
    .of_parse_cb = wl28661d_of_parse_cb,               \
    .ops = &wl28661d_regl_ops,                         \
    .regulators_node = of_match_ptr("regulators"),     \
    .n_voltages = WL28661D_N_VOLTAGES,                  \
    .min_uV = _min,                                    \
    .uV_step = _step,                                  \
    .linear_min_sel = 0,                               \
    .vsel_mask = WL28661D_VSEL_MASK,                   \
    .vsel_reg = WL28661D_##_id##_VSEL,                 \
    .enable_reg = WL28661D_LDO_EN,       \
    .enable_mask = BIT(WL28661D_REGULATOR_##_id),      \
    .type = REGULATOR_VOLTAGE,                         \
    .owner = THIS_MODULE,                              \
  }

static struct regulator_desc wl28661d_regls_desc[WL28661D_MAX_REGULATORS] = {
  WL28661D_REGL_DESC(LDO1, ldo1, "vin1", 600000, 6000),
  WL28661D_REGL_DESC(LDO2, ldo2, "vin1", 600000, 6000),
  WL28661D_REGL_DESC(LDO3, ldo3, "vin2", 1200000, 12500),
  WL28661D_REGL_DESC(LDO4, ldo4, "vin2", 1200000, 12500),
};

static int wl28661d_regulator_init(struct wl28661d *chip)
{
  struct regulator_config config = { };
  struct regulator_desc *rdesc;
  u8 vsel_range[1];
  int id, ret = 0;
  const unsigned int ldo_regs[WL28661D_MAX_REGULATORS] = {
    WL28661D_LDO1_VOUT,
    WL28661D_LDO2_VOUT,
    WL28661D_LDO3_VOUT,
    WL28661D_LDO4_VOUT,
  };

  const unsigned int initial_voltage[WL28661D_MAX_REGULATORS] = {
    0x64,//LDO1 DVDD 1.2V
    0x64,//LDO2 DVDD 1.2V
    0x80,//LDO3 AVDD 2.8V
    0x80,//LDO4 AVDD 2.8V
  };

  /*Disable all ldo output by default*/
  ret = regmap_write(chip->regmap, WL28661D_LDO_EN, chip->init_value);
  if (ret < 0) {
    dev_err(chip->dev, "Disable all LDO output failed!!!\n");
    return ret;
  }
  /* Enable all ldo discharge by default */
  ret = regmap_write(chip->regmap, WL28661D_DISCHARGE_RESISTORS, 0x8f);
  if (ret < 0) {
    dev_err(chip->dev, "Enable LDO discharge failed!!!\n");
    return ret;
  }
  for (id = 0; id < WL28661D_MAX_REGULATORS; id++) {
    chip->rdesc[id] = &wl28661d_regls_desc[id];
    rdesc = chip->rdesc[id];
    config.regmap = chip->regmap;
    config.dev = chip->dev;
    config.driver_data = chip;

    ret = regmap_bulk_read(chip->regmap, ldo_regs[id], vsel_range, 1);
    dev_dbg(chip->dev, "wl28661d_regulator_init: LDO%d, default value:0x%x", (id+1), vsel_range[0]);
    if (ret < 0) {
      dev_err(chip->dev, "Failed to read the ldo register\n");
      return ret;
    }

    ret = regmap_write(chip->regmap, ldo_regs[id], initial_voltage[id]);
    if (ret < 0) {
      dev_err(chip->dev, "Failed to write inital voltage register\n");
      return ret;
    }
    dev_dbg(chip->dev, "wl28661d_regulator_init: LDO%d, initial value:0x%x", (id+1),
            initial_voltage[id]);

    chip->rdev[id] = devm_regulator_register(chip->dev, rdesc, &config);
    if (IS_ERR(chip->rdev[id])) {
      ret = PTR_ERR(chip->rdev[id]);
      dev_err(chip->dev, "Failed to register regulator(%s):%d\n", chip->rdesc[id]->name, ret);
      return ret;
    }
  }

  return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 30))
static int wl28661d_i2c_probe(struct i2c_client *client)
#else
static int wl28661d_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
  struct device *dev = &client->dev;
  struct wl28661d *chip;
  int error, cs_gpio, vin1_gpio, ret, i, value;

  /* Set all register to initial value when probe driver to avoid register value was modified.
  */
  const unsigned int initial_register[5][2] = {
    {WL28661D_CURRENT_LIMITSEL, 	0x00},
    {WL28661D_DISCHARGE_RESISTORS, 	0x00},
    {WL28661D_LDO1_LDO2_SEQ, 	0x00},
    {WL28661D_LDO3_LDO4_SEQ, 	0x00},
    {WL28661D_SEQ_STATUS, 		0x00},
  };
  chip = devm_kzalloc(dev, sizeof(struct wl28661d), GFP_KERNEL);
  if (!chip) {
    dev_err(dev, "wl28661d_i2c_probe Memory error...\n");
    return -ENOMEM;
  }

  dev_info(dev, "wl28661d_i2c_probe Enter...\n");

  cs_gpio = of_get_named_gpio(dev->of_node, "cs-gpios", 0);
  if (cs_gpio > 0) {
    if (!gpio_is_valid(cs_gpio)) {
      dev_err(dev, "Invalid chip select pin\n");
      return -EPERM;
    }

    ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_LOW, "wl28661d_cs_pin");
    if (ret) {
      dev_err(dev, "GPIO(%d) request failed(%d)\n", cs_gpio, ret);
      return ret;
    }

    chip->chip_cs_pin = cs_gpio;
  }

  dev_info(dev, "wl28661d_i2c_probe cs_gpio:%d...\n", cs_gpio);

  vin1_gpio = of_get_named_gpio(dev->of_node, "vin1-gpios", 0);
  if (vin1_gpio > 0) {
    if (!gpio_is_valid(vin1_gpio)) {
      dev_err(dev, "Invalid vin1 select pin\n");
      return -EPERM;
    }

    ret = devm_gpio_request_one(dev, vin1_gpio, GPIOF_OUT_INIT_HIGH, "wl28661d_vin1_pin");
    if (ret) {
      dev_err(dev, "GPIO(%d) request failed(%d)\n", vin1_gpio, ret);
      return ret;
    }

    chip->chip_vin1_pin = vin1_gpio;
    dev_info(dev, "wl28661d_i2c_probe vin1_gpio:%d...\n", vin1_gpio);
  }

  if (of_property_read_u32(dev->of_node, "init-value", &value) < 0) {
    dev_info(dev, "wl28661d_i2c_probe no init_value, use default 0x0\n");
    value = 0x0;
  }
  chip->init_value = value;
  dev_info(dev, "wl28661d_i2c_probe init_value:%d...\n", value);
  chip->shutdown_ldo = of_property_read_bool(dev->of_node, "shutdown-ldo");

  mdelay(10);

  i2c_set_clientdata(client, chip);
  chip->dev = dev;
  chip->regmap = devm_regmap_init_i2c(client, &wl28661d_regmap_config);
  if (IS_ERR(chip->regmap)) {
    error = PTR_ERR(chip->regmap);
    dev_err(dev, "Failed to allocate register map: %d\n", error);
    return error;
  }

  ret = regmap_read(chip->regmap, WL28661D_CHIP_REV, &ldo_chipid);
  if (ret < 0 ) {
    dev_err(chip->dev, "Failed to read CHIP ID:0x%x, ret:%d\n", ldo_chipid, ret);
    ret = -ENODEV;
    return ret;
  } else {
    if (ldo_chipid == WL28661D_ID) {
      dev_info(chip->dev, "WL28661D CHIP ID matched!\n");
    }
    else {
      dev_err(chip->dev, "Failed to read other CHIP ID:0x%x, ret:%d\n", ldo_chipid, ret);
      ret = -ENODEV;
      return ret;
    }
  }

  for (i = 0; i < 5; i++) {
    ret = regmap_write(chip->regmap, initial_register[i][0], initial_register[i][1]);
    if (ret < 0) {
      dev_err(chip->dev,"Failed to write register: 0x%x, value: 0x%x \n", initial_register[i][0],
              initial_register[i][1]);
    }

    dev_dbg(chip->dev, "Success to write register: 0x%x, value: 0x%x \n", initial_register[i][0],
            initial_register[i][1]);
  }

  ret = wl28661d_regulator_init(chip);
  if (ret < 0) {
    dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
    return ret;
  }

  dev_info(chip->dev, "wl28661d_i2c_probe Exit...\n");

  return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void wl28661d_i2c_remove(struct i2c_client *client)
{
  struct wl28661d *chip = i2c_get_clientdata(client);
  struct gpio_desc *desc;
  int ret = 0;

  if(chip)
  {
    if (chip->chip_cs_pin > 0) {
      desc = gpio_to_desc(chip->chip_cs_pin);
      ret = gpiod_direction_output_raw(desc, GPIOF_OUT_INIT_LOW);
      if (ret) {
        dev_err(chip->dev, "wl28661d_i2c_remove cs-pin output %d\n", ret);
        return;
      }
    }

    if (chip->chip_vin1_pin > 0) {
      desc = gpio_to_desc(chip->chip_vin1_pin);
      ret = gpiod_direction_output_raw(desc, GPIOF_OUT_INIT_LOW);
      if (ret) {
        dev_err(chip->dev, "wl28661d_i2c_remove vin1-pin output %d\n", ret);
        return;
      }
    }
  }
}
#else
static int wl28661d_i2c_remove(struct i2c_client *client)
{
  struct wl28661d *chip = i2c_get_clientdata(client);
  struct gpio_desc *desc;
  int ret = 0;

  if (chip->chip_cs_pin > 0) {
    desc = gpio_to_desc(chip->chip_cs_pin);
    ret = gpiod_direction_output_raw(desc, GPIOF_OUT_INIT_LOW);
    if (ret) {
      dev_err(chip->dev, "wl28661d_i2c_remove cs-pin output %d\n", ret);
      return ret;
    }
  }

  if (chip->chip_vin1_pin > 0) {
    desc = gpio_to_desc(chip->chip_vin1_pin);
    ret = gpiod_direction_output_raw(desc, GPIOF_OUT_INIT_LOW);
    if (ret) {
      dev_err(chip->dev, "wl28661d_i2c_remove vin1-pin output %d\n", ret);
      return ret;
    }
  }

  return ret;
}
#endif

static void wl28661d_i2c_shutdown(struct i2c_client *client)
{
  struct wl28661d *chip = i2c_get_clientdata(client);
  unsigned int val = 0;

  if (chip) {
    if (chip->shutdown_ldo) {
      regmap_read(chip->regmap, WL28661D_LDO_EN, &val);
      /* Disable AVDD1 when shutdown to meet device SPEC and avoid current leak */
      regmap_write(chip->regmap, WL28661D_LDO_EN, val & ~(1<<2));
      dev_info(chip->dev, "wl28661d_i2c_shutdown");
    }
    regmap_write(chip->regmap, WL28661D_LDO_EN, 0);
    dev_err(chip->dev, "wl28661d_i2c_shutdown force disable all LDOs");
  }
}

static const struct i2c_device_id wl28661d_i2c_id[] = {
  {"wl28661d", 0},
  {},
};
MODULE_DEVICE_TABLE(i2c, wl28661d_i2c_id);

static struct i2c_driver wl28661d_regulator_driver = {
  .driver = {
    .name = "wl28661d-regulator",
  },
  .probe = wl28661d_i2c_probe,
  .remove = wl28661d_i2c_remove,
  .shutdown = wl28661d_i2c_shutdown,
  .id_table = wl28661d_i2c_id,
};

module_i2c_driver(wl28661d_regulator_driver);

MODULE_AUTHOR("Chen Zhiming <chenzm8@motorola.com>");
MODULE_DESCRIPTION("WL28661D regulator driver");
MODULE_LICENSE("GPL");
