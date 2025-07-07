// SPDX-License-Identifier: GPL-2.0-only
/*
 * AW95016A 16-Channel I2C GPIO Expander Driver
 * 
 * Copyright (C) 2025
 * 
 * Based on AW95016A datasheet v1.5
 * 
 * Features:
 * - 16 GPIO pins with configurable direction
 * - Interrupt support with latching capability
 * - Configurable drive strength (4 levels)
 * - Push-pull or open-drain output modes
 * - Internal pull-up/pull-down resistors
 * - Hardware and software reset
 * - Device tree support
 * - Bootloader configuration synchronization
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/mutex.h>

/* Register addresses */
#define AW95016A_P0DI		0x00	/* Port 0 Data Input */
#define AW95016A_P1DI		0x01	/* Port 1 Data Input */
#define AW95016A_P0DO		0x02	/* Port 0 Data Output */
#define AW95016A_P1DO		0x03	/* Port 1 Data Output */
#define AW95016A_P0INVEN	0x04	/* Port 0 Input Invert Enable */
#define AW95016A_P1INVEN	0x05	/* Port 1 Input Invert Enable */
#define AW95016A_P0DIR		0x06	/* Port 0 Direction */
#define AW95016A_P1DIR		0x07	/* Port 1 Direction */
#define AW95016A_P0DSR1		0x08	/* Port 0 Drive Strength 1 */
#define AW95016A_P0DSR2		0x09	/* Port 0 Drive Strength 2 */
#define AW95016A_P1DSR1		0x0A	/* Port 1 Drive Strength 1 */
#define AW95016A_P1DSR2		0x0B	/* Port 1 Drive Strength 2 */
#define AW95016A_P0LEN		0x0C	/* Port 0 Latch Enable */
#define AW95016A_P1LEN		0x0D	/* Port 1 Latch Enable */
#define AW95016A_P0PEN		0x0E	/* Port 0 Pull Enable */
#define AW95016A_P1PEN		0x0F	/* Port 1 Pull Enable */
#define AW95016A_P0PMD		0x10	/* Port 0 Pull Mode */
#define AW95016A_P1PMD		0x11	/* Port 1 Pull Mode */
#define AW95016A_P0MSK		0x12	/* Port 0 Interrupt Mask */
#define AW95016A_P1MSK		0x13	/* Port 1 Interrupt Mask */
#define AW95016A_P0INTST	0x14	/* Port 0 Interrupt Status */
#define AW95016A_P1INTST	0x15	/* Port 1 Interrupt Status */
#define AW95016A_P0DOMD		0x16	/* Port 0 Output Mode */
#define AW95016A_P1DOMD		0x17	/* Port 1 Output Mode */
#define AW95016A_GGCR		0x1A	/* GPIO Global Control */
#define AW95016A_STATE		0x60	/* Power Status */
#define AW95016A_GCR2		0x61	/* Global Control 2 */
#define AW95016A_RESET		0x70	/* Reset/ID */

/* Register field definitions */

/* Direction register bits */
#define AW95016A_DIR_INPUT	0
#define AW95016A_DIR_OUTPUT	1

/* Drive strength values */
#define AW95016A_DS_0_25X	0x00
#define AW95016A_DS_0_5X	0x01
#define AW95016A_DS_0_75X	0x02
#define AW95016A_DS_1X		0x03

/* Output mode values */
#define AW95016A_DOMD_PUSH_PULL	0
#define AW95016A_DOMD_OPEN_DRAIN 1

/* Pull mode values */
#define AW95016A_PMD_PULL_DOWN	0
#define AW95016A_PMD_PULL_UP	1

/* Interrupt mask values */
#define AW95016A_MSK_ENABLE	0
#define AW95016A_MSK_DISABLE	1

/* Global control register fields */
#define AW95016A_GGCR_EGC_MASK	0x03
#define AW95016A_GGCR_EGC_0_5NS	0x00
#define AW95016A_GGCR_EGC_1NS	0x01
#define AW95016A_GGCR_EGC_4NS	0x02
#define AW95016A_GGCR_EGC_8NS	0x03

/* State register fields */
#define AW95016A_STATE_PUST	BIT(4)

/* GCR2 register fields */
#define AW95016A_GCR2_BSDIS	BIT(4)

/* Reset register values */
#define AW95016A_RESET_VALUE	0x00
#define AW95016A_CHIP_ID	0x80

/* I2C addresses */
#define AW95016A_I2C_ADDR_AD_GND	0x20
#define AW95016A_I2C_ADDR_AD_VDD	0x21
#define AW95016A_I2C_ADDR_BROADCAST	0x1C

/* Device constants */
#define AW95016A_MAX_GPIO	16
#define AW95016A_PORT0_PINS	8
#define AW95016A_PORT1_PINS	8

/* Port helper macros */
#define AW95016A_PORT_FROM_PIN(pin)	((pin) / 8)
#define AW95016A_BIT_FROM_PIN(pin)	((pin) % 8)
#define AW95016A_PIN_MASK(pin)		BIT(AW95016A_BIT_FROM_PIN(pin))

/* Register pair access helpers */
struct aw95016a_reg_pair {
	u8 port0;
	u8 port1;
};

#define AW95016A_REG_PAIR(p0, p1) { .port0 = (p0), .port1 = (p1) }

/* Pin configuration structure */
struct aw95016a_pin_config {
	bool invert_enable;
	bool pull_enable;
	bool pull_up;		/* true = pull-up, false = pull-down */
	bool open_drain;	/* true = open-drain, false = push-pull */
	bool latch_enable;
	bool irq_enable;
	u8 drive_strength;	/* 0-3 corresponding to 0.25x, 0.5x, 0.75x, 1x */
};

/* Main device structure */
struct aw95016a_chip {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
	struct mutex lock;

	int irq;
	bool irq_enabled;

	/* Cached register values for efficiency */
	u8 reg_direction[2];
	u8 reg_output[2];
	u8 reg_input[2];
	u8 reg_irq_mask[2];
	u8 reg_output_mode[2];
	u8 reg_pull_enable[2];
	u8 reg_pull_mode[2];
	u8 reg_drive_strength[4];	/* P0DSR1, P0DSR2, P1DSR1, P1DSR2 */
	u8 reg_invert_enable[2];
	u8 reg_latch_enable[2];

	/* Pin configuration cache */
	struct aw95016a_pin_config pin_config[AW95016A_MAX_GPIO];
};

static const struct regmap_config aw95016a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW95016A_RESET,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,
};

/* Common register pairs */
static const struct aw95016a_reg_pair aw95016a_regs_input = 
	AW95016A_REG_PAIR(AW95016A_P0DI, AW95016A_P1DI);
static const struct aw95016a_reg_pair aw95016a_regs_output = 
	AW95016A_REG_PAIR(AW95016A_P0DO, AW95016A_P1DO);
static const struct aw95016a_reg_pair aw95016a_regs_direction = 
	AW95016A_REG_PAIR(AW95016A_P0DIR, AW95016A_P1DIR);
static const struct aw95016a_reg_pair aw95016a_regs_invert = 
	AW95016A_REG_PAIR(AW95016A_P0INVEN, AW95016A_P1INVEN);
static const struct aw95016a_reg_pair aw95016a_regs_pull_enable = 
	AW95016A_REG_PAIR(AW95016A_P0PEN, AW95016A_P1PEN);
static const struct aw95016a_reg_pair aw95016a_regs_pull_mode = 
	AW95016A_REG_PAIR(AW95016A_P0PMD, AW95016A_P1PMD);
static const struct aw95016a_reg_pair aw95016a_regs_output_mode = 
	AW95016A_REG_PAIR(AW95016A_P0DOMD, AW95016A_P1DOMD);
static const struct aw95016a_reg_pair aw95016a_regs_irq_mask = 
	AW95016A_REG_PAIR(AW95016A_P0MSK, AW95016A_P1MSK);
static const struct aw95016a_reg_pair aw95016a_regs_irq_status = 
	AW95016A_REG_PAIR(AW95016A_P0INTST, AW95016A_P1INTST);
static const struct aw95016a_reg_pair aw95016a_regs_latch_enable = 
	AW95016A_REG_PAIR(AW95016A_P0LEN, AW95016A_P1LEN);

/* Drive strength register pairs */
static const struct aw95016a_reg_pair aw95016a_regs_drive_strength_low = 
	AW95016A_REG_PAIR(AW95016A_P0DSR1, AW95016A_P1DSR1);
static const struct aw95016a_reg_pair aw95016a_regs_drive_strength_high = 
	AW95016A_REG_PAIR(AW95016A_P0DSR2, AW95016A_P1DSR2);

/* Register access helpers */
static int aw95016a_read_port_reg(struct aw95016a_chip *chip, 
				  struct aw95016a_reg_pair regs,
				  unsigned int port, u8 *val)
{
	u8 reg = (port == 0) ? regs.port0 : regs.port1;
	unsigned int tmp;
	int ret;

	ret = regmap_read(chip->regmap, reg, &tmp);
	if (ret == 0)
		*val = tmp;
	return ret;
}

static int aw95016a_update_port_reg(struct aw95016a_chip *chip,
				    struct aw95016a_reg_pair regs,
				    unsigned int port, u8 mask, u8 val)
{
	u8 reg = (port == 0) ? regs.port0 : regs.port1;
	return regmap_update_bits(chip->regmap, reg, mask, val);
}

/* Helper function to convert pin number to port and bit */
static void aw95016a_pin_to_port_bit(unsigned int pin, unsigned int *port, 
				     unsigned int *bit)
{
	*port = AW95016A_PORT_FROM_PIN(pin);
	*bit = AW95016A_BIT_FROM_PIN(pin);
}

/* Read and synchronize all GPIO configurations from chip */
static void aw95016a_sync_pin_config(struct aw95016a_chip *chip, int port, int bit)
{
	int pin = port * 8 + bit;
	struct aw95016a_pin_config *config = &chip->pin_config[pin];

	/* Sync direction */
	config->pull_enable = !!(chip->reg_pull_enable[port] & BIT(bit));
	config->pull_up = !!(chip->reg_pull_mode[port] & BIT(bit));
	config->open_drain = !!(chip->reg_output_mode[port] & BIT(bit));
	config->invert_enable = !!(chip->reg_invert_enable[port] & BIT(bit));
	config->latch_enable = !!(chip->reg_latch_enable[port] & BIT(bit));
	config->irq_enable = !(chip->reg_irq_mask[port] & BIT(bit));

	/* Sync drive strength */
	if (bit < 4) {
		config->drive_strength = (chip->reg_drive_strength[port * 2] >> (bit * 2)) & 0x03;
	} else {
		config->drive_strength = (chip->reg_drive_strength[port * 2 + 1] >> ((bit - 4) * 2)) & 0x03;
	}
}

/* Read all register configurations from chip and sync to driver state */
static int aw95016a_sync_chip_state(struct aw95016a_chip *chip)
{
	int ret, port, bit;
	unsigned int val;

	dev_info(chip->dev, "Synchronizing GPIO state from bootloader configuration\n");

	/* Read and cache all register states */

	/* Direction registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_direction, 0, &chip->reg_direction[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_direction, 1, &chip->reg_direction[1]);
	if (ret) return ret;

	/* Output registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_output, 0, &chip->reg_output[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_output, 1, &chip->reg_output[1]);
	if (ret) return ret;

	/* Input registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_input, 0, &chip->reg_input[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_input, 1, &chip->reg_input[1]);
	if (ret) return ret;

	/* Interrupt mask registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_irq_mask, 0, &chip->reg_irq_mask[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_irq_mask, 1, &chip->reg_irq_mask[1]);
	if (ret) return ret;

	/* Output mode registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_output_mode, 0, &chip->reg_output_mode[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_output_mode, 1, &chip->reg_output_mode[1]);
	if (ret) return ret;

	/* Pull enable registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_pull_enable, 0, &chip->reg_pull_enable[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_pull_enable, 1, &chip->reg_pull_enable[1]);
	if (ret) return ret;

	/* Pull mode registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_pull_mode, 0, &chip->reg_pull_mode[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_pull_mode, 1, &chip->reg_pull_mode[1]);
	if (ret) return ret;

	/* Drive strength registers */
	ret = regmap_read(chip->regmap, AW95016A_P0DSR1, &val);
	if (ret) return ret;
	chip->reg_drive_strength[0] = val;

	ret = regmap_read(chip->regmap, AW95016A_P0DSR2, &val);
	if (ret) return ret;
	chip->reg_drive_strength[1] = val;

	ret = regmap_read(chip->regmap, AW95016A_P1DSR1, &val);
	if (ret) return ret;
	chip->reg_drive_strength[2] = val;

	ret = regmap_read(chip->regmap, AW95016A_P1DSR2, &val);
	if (ret) return ret;
	chip->reg_drive_strength[3] = val;

	/* Invert enable registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_invert, 0, &chip->reg_invert_enable[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_invert, 1, &chip->reg_invert_enable[1]);
	if (ret) return ret;

	/* Latch enable registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_latch_enable, 0, &chip->reg_latch_enable[0]);
	if (ret) return ret;
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_latch_enable, 1, &chip->reg_latch_enable[1]);
	if (ret) return ret;

	/* Sync per-pin configuration */
	for (port = 0; port < 2; port++) {
		for (bit = 0; bit < 8; bit++) {
			aw95016a_sync_pin_config(chip, port, bit);
		}
	}

	/* Print current configuration for debugging */
	dev_info(chip->dev, "GPIO configuration synchronized:\n");
	dev_info(chip->dev, "  Direction: P0=0x%02x P1=0x%02x\n", 
		 chip->reg_direction[0], chip->reg_direction[1]);
	dev_info(chip->dev, "  Output:    P0=0x%02x P1=0x%02x\n", 
		 chip->reg_output[0], chip->reg_output[1]);
	dev_info(chip->dev, "  Input:     P0=0x%02x P1=0x%02x\n", 
		 chip->reg_input[0], chip->reg_input[1]);
	dev_info(chip->dev, "  IRQ Mask:  P0=0x%02x P1=0x%02x\n", 
		 chip->reg_irq_mask[0], chip->reg_irq_mask[1]);
	dev_info(chip->dev, "  Out Mode:  P0=0x%02x P1=0x%02x\n", 
		 chip->reg_output_mode[0], chip->reg_output_mode[1]);
	dev_info(chip->dev, "  Pull Ena:  P0=0x%02x P1=0x%02x\n", 
		 chip->reg_pull_enable[0], chip->reg_pull_enable[1]);
	dev_info(chip->dev, "  Pull Mode: P0=0x%02x P1=0x%02x\n", 
		 chip->reg_pull_mode[0], chip->reg_pull_mode[1]);
	dev_info(chip->dev, "  Drive Str: P0DSR1=0x%02x P0DSR2=0x%02x P1DSR1=0x%02x P1DSR2=0x%02x\n", 
		 chip->reg_drive_strength[0], chip->reg_drive_strength[1], 
		 chip->reg_drive_strength[2], chip->reg_drive_strength[3]);

	return 0;
}

/* Chip identification and initialization */
static int aw95016a_chip_init(struct aw95016a_chip *chip)
{
	unsigned int chip_id;
	int ret;

	/* Read chip ID to verify device */
	ret = regmap_read(chip->regmap, AW95016A_RESET, &chip_id);
	if (ret) {
		dev_err(chip->dev, "Failed to read chip ID: %d\n", ret);
		return ret;
	}

	if (chip_id != AW95016A_CHIP_ID) {
		dev_err(chip->dev, "Invalid chip ID: 0x%02x (expected 0x%02x)\n",
			chip_id, AW95016A_CHIP_ID);
		return -ENODEV;
	}

	dev_err(chip->dev, "AW95016A chip detected, ID: 0x%02x\n", chip_id);

	/* 
	 * Do NOT perform software reset here!
	 * We want to preserve bootloader configuration.
	 * Instead, read and synchronize the current chip state.
	 */
	ret = aw95016a_sync_chip_state(chip);
	if (ret) {
		dev_err(chip->dev, "Failed to synchronize chip state: %d\n", ret);
		return ret;
	}

	return 0;
}

/* GPIO chip operations */
static int aw95016a_gpio_direction_input(struct gpio_chip *gc, unsigned int pin)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;
	int ret;

	if (pin >= AW95016A_MAX_GPIO)
		return -EINVAL;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);

	ret = aw95016a_update_port_reg(chip, aw95016a_regs_direction, port,
				       BIT(bit), 0);
	if (ret == 0)
		chip->reg_direction[port] &= ~BIT(bit);

	mutex_unlock(&chip->lock);

	dev_info(chip->dev, "%s, pin %d\n", __func__, pin);

	return ret;
}

static int aw95016a_gpio_direction_output(struct gpio_chip *gc, unsigned int pin,
					  int value)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;
	int ret;

	if (pin >= AW95016A_MAX_GPIO)
		return -EINVAL;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);

	/* Set output value first */
	ret = aw95016a_update_port_reg(chip, aw95016a_regs_output, port,
				       BIT(bit), value ? BIT(bit) : 0);
	if (ret)
		goto out;

	if (value)
		chip->reg_output[port] |= BIT(bit);
	else
		chip->reg_output[port] &= ~BIT(bit);

	/* Then set direction to output */
	ret = aw95016a_update_port_reg(chip, aw95016a_regs_direction, port,
				       BIT(bit), BIT(bit));
	if (ret == 0)
		chip->reg_direction[port] |= BIT(bit);

out:
	mutex_unlock(&chip->lock);

	dev_info(chip->dev, "%s, pin %d, value %d\n", __func__, pin, value);

	return ret;
}

static int aw95016a_gpio_get(struct gpio_chip *gc, unsigned int pin)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;
	u8 val;
	int ret;

	if (pin >= AW95016A_MAX_GPIO)
		return -EINVAL;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);

	/* Read input register for current pin state */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_input, port, &val);
	if (ret)
		goto out;

	chip->reg_input[port] = val;
	ret = !!(val & BIT(bit));

out:
	mutex_unlock(&chip->lock);

	dev_info(chip->dev, "%s, pin %d, value %d\n", __func__, pin, ret);
	return ret;
}

static void aw95016a_gpio_set(struct gpio_chip *gc, unsigned int pin, int value)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;
	int ret;

	if (pin >= AW95016A_MAX_GPIO)
		return;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);

	ret = aw95016a_update_port_reg(chip, aw95016a_regs_output, port,
				       BIT(bit), value ? BIT(bit) : 0);
	if (ret == 0) {
		if (value)
			chip->reg_output[port] |= BIT(bit);
		else
			chip->reg_output[port] &= ~BIT(bit);
	}

	mutex_unlock(&chip->lock);

	if (ret)
		dev_err(chip->dev, "Failed to set pin %u: %d\n", pin, ret);
	else
		dev_info(chip->dev, "%s, pin %d, value %d\n", __func__, pin, value);
}

static int aw95016a_gpio_get_direction(struct gpio_chip *gc, unsigned int pin)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;

	if (pin >= AW95016A_MAX_GPIO)
		return -EINVAL;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	return (chip->reg_direction[port] & BIT(bit)) ? 
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

/* Advanced pin configuration through pinctrl interface */
static int aw95016a_gpio_set_config(struct gpio_chip *gc, unsigned int pin,
				    unsigned long config)
{
	struct aw95016a_chip *chip = gpiochip_get_data(gc);
	unsigned int port, bit;
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int ret = 0;

	if (pin >= AW95016A_MAX_GPIO)
		return -EINVAL;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_pull_enable,
					       port, BIT(bit), 0);
		if (ret == 0) {
			chip->pin_config[pin].pull_enable = false;
			chip->reg_pull_enable[port] &= ~BIT(bit);
		}
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_pull_mode,
					       port, BIT(bit), BIT(bit));
		if (ret)
			break;
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_pull_enable,
					       port, BIT(bit), BIT(bit));
		if (ret == 0) {
			chip->pin_config[pin].pull_enable = true;
			chip->pin_config[pin].pull_up = true;
			chip->reg_pull_enable[port] |= BIT(bit);
			chip->reg_pull_mode[port] |= BIT(bit);
		}
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_pull_mode,
					       port, BIT(bit), 0);
		if (ret)
			break;
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_pull_enable,
					       port, BIT(bit), BIT(bit));
		if (ret == 0) {
			chip->pin_config[pin].pull_enable = true;
			chip->pin_config[pin].pull_up = false;
			chip->reg_pull_enable[port] |= BIT(bit);
			chip->reg_pull_mode[port] &= ~BIT(bit);
		}
		break;

	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		ret = aw95016a_update_port_reg(chip, aw95016a_regs_output_mode,
					       port, BIT(bit), 
					       arg ? BIT(bit) : 0);
		if (ret == 0) {
			chip->pin_config[pin].open_drain = !!arg;
			if (arg)
				chip->reg_output_mode[port] |= BIT(bit);
			else
				chip->reg_output_mode[port] &= ~BIT(bit);
		}
		break;

	case PIN_CONFIG_DRIVE_STRENGTH:
		/* arg should be 0-3 for 0.25x, 0.5x, 0.75x, 1x */
		if (arg > 3) {
			ret = -EINVAL;
			break;
		}
		/* Drive strength is stored in pairs of bits */
		if (bit < 4) {
			ret = aw95016a_update_port_reg(chip, 
				aw95016a_regs_drive_strength_low, port,
				0x03 << (bit * 2), arg << (bit * 2));
			if (ret == 0)
				chip->reg_drive_strength[port * 2] = 
					(chip->reg_drive_strength[port * 2] & ~(0x03 << (bit * 2))) |
					(arg << (bit * 2));
		} else {
			ret = aw95016a_update_port_reg(chip,
				aw95016a_regs_drive_strength_high, port,
				0x03 << ((bit - 4) * 2), arg << ((bit - 4) * 2));
			if (ret == 0)
				chip->reg_drive_strength[port * 2 + 1] = 
					(chip->reg_drive_strength[port * 2 + 1] & ~(0x03 << ((bit - 4) * 2))) |
					(arg << ((bit - 4) * 2));
		}
		if (ret == 0)
			chip->pin_config[pin].drive_strength = arg;
		break;

	default:
		ret = -ENOTSUPP;
		break;
	}

	mutex_unlock(&chip->lock);
	return ret;
}

/* IRQ chip operations */
static void aw95016a_irq_mask(struct irq_data *data)
{
	struct aw95016a_chip *chip = irq_data_get_irq_chip_data(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int port, bit;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);
	aw95016a_update_port_reg(chip, aw95016a_regs_irq_mask, port,
				 BIT(bit), BIT(bit));
	chip->reg_irq_mask[port] |= BIT(bit);
	mutex_unlock(&chip->lock);
}

static void aw95016a_irq_unmask(struct irq_data *data)
{
	struct aw95016a_chip *chip = irq_data_get_irq_chip_data(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int port, bit;

	aw95016a_pin_to_port_bit(pin, &port, &bit);

	mutex_lock(&chip->lock);
	aw95016a_update_port_reg(chip, aw95016a_regs_irq_mask, port,
				 BIT(bit), 0);
	chip->reg_irq_mask[port] &= ~BIT(bit);
	mutex_unlock(&chip->lock);
}

static int aw95016a_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct aw95016a_chip *chip = irq_data_get_irq_chip_data(data);
	unsigned int pin = irqd_to_hwirq(data);

	/* AW95016A supports only edge interrupts on input state changes */
	if (type & IRQ_TYPE_LEVEL_MASK)
		return -EINVAL;

	/* Store interrupt type for reference */
	if (type & IRQ_TYPE_EDGE_BOTH)
		chip->pin_config[pin].irq_enable = true;
	else
		chip->pin_config[pin].irq_enable = false;

	return 0;
}

static void aw95016a_irq_bus_lock(struct irq_data *data)
{
	struct aw95016a_chip *chip = irq_data_get_irq_chip_data(data);
	mutex_lock(&chip->lock);
}

static void aw95016a_irq_bus_sync_unlock(struct irq_data *data)
{
	struct aw95016a_chip *chip = irq_data_get_irq_chip_data(data);
	mutex_unlock(&chip->lock);
}

/* Main interrupt handler */
static irqreturn_t aw95016a_irq_handler(int irq, void *data)
{
	struct aw95016a_chip *chip = data;
	u8 pending[2];
	int ret, port, bit;
	unsigned int pin;

	/* Read interrupt status registers */
	ret = aw95016a_read_port_reg(chip, aw95016a_regs_irq_status, 0, 
				     &pending[0]);
	if (ret)
		return IRQ_NONE;

	ret = aw95016a_read_port_reg(chip, aw95016a_regs_irq_status, 1,
				     &pending[1]);
	if (ret)
		return IRQ_NONE;

	/* Clear interrupts by reading input registers */
	if (pending[0] || pending[1]) {
		u8 dummy;
		aw95016a_read_port_reg(chip, aw95016a_regs_input, 0, &dummy);
		aw95016a_read_port_reg(chip, aw95016a_regs_input, 1, &dummy);
	}

	/* Handle each pending interrupt */
	for (port = 0; port < 2; port++) {
		for (bit = 0; bit < 8; bit++) {
			if (pending[port] & BIT(bit)) {
				pin = port * 8 + bit;
				handle_nested_irq(irq_find_mapping(
					chip->gpio_chip.irq.domain, pin));
			}
		}
	}

	return (pending[0] || pending[1]) ? IRQ_HANDLED : IRQ_NONE;
}

/* Initialize GPIO chip structure */
static int aw95016a_setup_gpio(struct aw95016a_chip *chip)
{
	struct gpio_chip *gc = &chip->gpio_chip;

	gc->label = "aw95016a-gpio";
	gc->parent = chip->dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->ngpio = AW95016A_MAX_GPIO;
	gc->can_sleep = true;

	gc->direction_input = aw95016a_gpio_direction_input;
	gc->direction_output = aw95016a_gpio_direction_output;
	gc->get = aw95016a_gpio_get;
	gc->set = aw95016a_gpio_set;
	gc->get_direction = aw95016a_gpio_get_direction;
	gc->set_config = aw95016a_gpio_set_config;

	gc->fwnode = dev_fwnode(chip->dev);

	return devm_gpiochip_add_data(chip->dev, gc, chip);
}

/* Initialize IRQ chip structure */
static int aw95016a_setup_irq(struct aw95016a_chip *chip)
{
	struct irq_chip *ic = &chip->irq_chip;
	struct gpio_irq_chip *girq;
	int ret;

	if (!chip->irq)
		return 0;

	ic->name = "aw95016a-irq";
	ic->irq_mask = aw95016a_irq_mask;
	ic->irq_unmask = aw95016a_irq_unmask;
	ic->irq_set_type = aw95016a_irq_set_type;
	ic->irq_bus_lock = aw95016a_irq_bus_lock;
	ic->irq_bus_sync_unlock = aw95016a_irq_bus_sync_unlock;

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					aw95016a_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"aw95016a", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ %d: %d\n", 
			chip->irq, ret);
		return ret;
	}

	/* Setup GPIO IRQ chip */
	girq = &chip->gpio_chip.irq;
	girq->chip = ic;
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;

	return 0;
}

/* Device tree parsing */
static int aw95016a_parse_dt(struct aw95016a_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	u32 val;

	if (!np)
		return 0;

	/* Parse interrupt configuration */
	chip->irq = irq_of_parse_and_map(np, 0);
	if (chip->irq <= 0) {
		dev_info(chip->dev, "No interrupt specified\n");
		chip->irq = 0;
	}

	/* Parse optional global edge control */
	if (!of_property_read_u32(np, "awinic,edge-control", &val)) {
		if (val <= 3) {
			regmap_update_bits(chip->regmap, AW95016A_GGCR,
					   AW95016A_GGCR_EGC_MASK, val);
		}
	}

	return 0;
}

/* I2C driver probe function */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int aw95016a_probe(struct i2c_client *client)
#else
static int aw95016a_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
#endif
{
	struct aw95016a_chip *chip;
	int ret;

	dev_err(&client->dev, "AW95016A GPIO expander probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C adapter doesn't support I2C\n");
		return -EIO;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;
	mutex_init(&chip->lock);

	i2c_set_clientdata(client, chip);

	/* Initialize regmap */
	chip->regmap = devm_regmap_init_i2c(client, &aw95016a_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	/* Initialize chip and sync bootloader configuration */
	ret = aw95016a_chip_init(chip);
	if (ret)
		return ret;

	/* Parse device tree */
	ret = aw95016a_parse_dt(chip);
	if (ret)
		return ret;

	/* Setup GPIO chip */
	ret = aw95016a_setup_gpio(chip);
	if (ret)
		return ret;

	/* Setup IRQ chip */
	ret = aw95016a_setup_irq(chip);
	if (ret)
		return ret;

	dev_err(&client->dev, "AW95016A GPIO expander probe done\n");

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
static void aw95016a_remove(struct i2c_client *client)
#else
static int aw95016a_remove(struct i2c_client *client)
#endif
{
	struct aw95016a_chip *chip = i2c_get_clientdata(client);

	if (chip->irq)
		free_irq(chip->irq, chip);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,1,0)
	return 0;
#endif
}

/* Device tree match table */
static const struct of_device_id aw95016a_of_match[] = {
	{ .compatible = "awinic,aw95016a", },
	{ }
};
MODULE_DEVICE_TABLE(of, aw95016a_of_match);

/* I2C device ID table */
static const struct i2c_device_id aw95016a_id[] = {
	{ "aw95016a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw95016a_id);

static struct i2c_driver aw95016a_driver = {
	.driver = {
		.name = "aw95016a-gpio",
		.of_match_table = aw95016a_of_match,
	},
	.probe = aw95016a_probe,
	.remove = aw95016a_remove,
	.id_table = aw95016a_id,
};

module_i2c_driver(aw95016a_driver);

MODULE_DESCRIPTION("AW95016A 16-Channel I2C GPIO Expander Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Awinic");