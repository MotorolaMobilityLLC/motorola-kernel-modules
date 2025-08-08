/*
 * Copyright (C) 2025 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)     "POGO_PEN_CHG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/of_gpio.h>

#define CHG_SHOW_MAX_SIZE 50

struct pen_charger {
	struct device		*dev;
	int chg_ldswtch_en_gpio;
	int chg_boost_en_gpio;
	struct iio_channel *pen_sns_chan;
	int r_sns_milliohm;
};


static int pen_charger_parse_dt(struct pen_charger *chg)
{
	struct device_node *node = chg->dev->of_node;
	int rc = 0;

	chg->chg_ldswtch_en_gpio = of_get_named_gpio(node, "mmi,chg-ldswtch-en-gpio", 0);
	if(!gpio_is_valid(chg->chg_ldswtch_en_gpio)) {
		pr_err("chg->chg_ldswtch_en_gpio is %d invalid\n", chg->chg_ldswtch_en_gpio);
		return -ENODEV;
	}
	chg->chg_boost_en_gpio = of_get_named_gpio(node, "mmi,chg-boost-en-gpio", 0);
	if(!gpio_is_valid(chg->chg_boost_en_gpio)) {
		pr_err("chg->chg_boost_en_gpio is %d invalid\n", chg->chg_boost_en_gpio);
		return -ENODEV;
	}
	rc = of_property_read_u32(node, "mmi,r-sns-milliohm", &chg->r_sns_milliohm);
	if (rc)
		chg->r_sns_milliohm = 1000;

	pr_info("ldswtch_en_gpio: %d, boost_en_gpio: %d,r_sns: %d\n",
		chg->chg_ldswtch_en_gpio, chg->chg_boost_en_gpio, chg->r_sns_milliohm);

	return 0;
}

static ssize_t pen_chg_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pen_charger *this_chip = dev_get_drvdata(dev);
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("pen_chg_enable_store: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("pen_chg_enable_store: Invalid charger suspend value = %lu\n", mode);
		return -EINVAL;
	}

	pr_info("pen_chg_enable_store: enable = %lu\n", mode);
	if (!!mode) {
		gpio_set_value(this_chip->chg_boost_en_gpio, true);
		gpio_set_value(this_chip->chg_ldswtch_en_gpio, true);
	} else {
		gpio_set_value(this_chip->chg_boost_en_gpio, false);
		gpio_set_value(this_chip->chg_ldswtch_en_gpio, false);
	}

	return count;
}

static int read_pen_chg_current(struct pen_charger *this_chip)
{
	int rc;
	int pen_sns_uv = 0;
	int pen_current_ma = 0;

	if (!this_chip) {
		pr_err("read_pen_chg_current: chip is invalid\n");
		return 0;
	}

	rc = iio_read_channel_processed(this_chip->pen_sns_chan, &pen_sns_uv);
	if (rc < 0) {
		pr_err("Error Reading pen_sns_uv- rc:%d\n", rc);
		return 0;
	}

	pen_current_ma = pen_sns_uv / this_chip->r_sns_milliohm;

	pr_info("Read pen_sns_uv:%d,pen_current_ma:%d\n", pen_sns_uv, pen_current_ma);
	return pen_current_ma;
}

static DEVICE_ATTR(pen_chg_enable, 0200,
		NULL,
		pen_chg_enable_store);

static ssize_t pen_chg_current_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct pen_charger *this_chip = dev_get_drvdata(dev);
	int pen_current_ma = 0;

    if (!this_chip) {
		pr_err("pen_chg_current_show: chip is invalid\n");
		return -ENODEV;
	}
	pen_current_ma = read_pen_chg_current(this_chip);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", pen_current_ma);
}
static DEVICE_ATTR(pen_chg_current, 0444,
		pen_chg_current_show,
		NULL);

static int pen_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg;
	int rc;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;

	rc = pen_charger_parse_dt(chg);
	if (rc) {
		pr_err("Failed to parse devicetree, rc=%d\n", rc);
		return rc;
	}
	rc  = devm_gpio_request_one(dev, chg->chg_ldswtch_en_gpio,
				  GPIOF_OUT_INIT_LOW, "chg-ldswtch-en-gpio");
	if (rc  < 0) {
			pr_err("Failed to request chg-ldswtch-en-gpio, ret:%d", rc);
			return rc;
	}
	rc  = devm_gpio_request_one(dev, chg->chg_boost_en_gpio,
				  GPIOF_OUT_INIT_LOW, "chg-boost-en-gpio");
	if (rc  < 0) {
			pr_err("Failed to request chg-boost-en-gpio, ret:%d", rc);
			return rc;
	}
	chg->pen_sns_chan = iio_channel_get(&pdev->dev,"pm8350b_pen_sns");
	if (IS_ERR(chg->pen_sns_chan)) {
		rc = PTR_ERR(chg->pen_sns_chan);
		chg->pen_sns_chan = NULL;
		pr_err("Error Getting pen_sns_chan channel- rc:%d\n",rc);
		return rc;
	}

	rc = device_create_file(dev, &dev_attr_pen_chg_current);
	if (rc) {
		pr_err("couldn't create pen_chg_current\n");
		return rc;
	}
	rc = device_create_file(dev, &dev_attr_pen_chg_enable);
	if (rc) {
		pr_err("couldn't create pen_chg_enable\n");
		return rc;
	}

	pr_info("pen_charger_probe done\n");

	return 0;
}

static int pen_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	device_remove_file(dev, &dev_attr_pen_chg_enable);
	device_remove_file(dev, &dev_attr_pen_chg_current);

	return 0;
}

static const struct of_device_id pen_charger_match_table[] = {
	{.compatible = "mmi,pogo-pen-charger"},
	{},
};

static struct platform_driver pen_charger_driver = {
	.driver	= {
		.name = "pogo_pen_charger",
		.of_match_table = pen_charger_match_table,
	},
	.probe	= pen_charger_probe,
	.remove	= pen_charger_remove,
};

module_platform_driver(pen_charger_driver);

MODULE_DESCRIPTION("Pogo Pen Charger Driver");
MODULE_LICENSE("GPL v2");
