// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved. */

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>

struct mmi_stow_drvdata {
	struct device	*dev;
	int stow_gpio;
};

/* Global var to be used in show function*/
int g_stow_gpio;

static ssize_t mmi_stow_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;

	value = gpio_get_value(g_stow_gpio);
	dev_info(dev, "stowed gpio value %d", value);

	return scnprintf(buf, PAGE_SIZE, "0x%02x", value);
}

static ssize_t mmi_stow_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* At present, nothing to do with setting state */
	return count;
}

static DEVICE_ATTR(mmi_stow, 0664, mmi_stow_show, mmi_stow_store);

static struct attribute *mmi_stow_attributes[] = {
	&dev_attr_mmi_stow.attr,
	NULL,
};

static struct attribute_group mmi_stow_attr_group = {
	.attrs = mmi_stow_attributes
};

static const struct attribute_group *mmi_stow_attr_groups[] = {
	&mmi_stow_attr_group,
	NULL,
};

static struct miscdevice mmi_stow_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mmi_stow",
	.groups = mmi_stow_attr_groups,
};

static int mmi_stow_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmi_stow_drvdata *drvdata;

	int ret = 0;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	/* Get gpio number from Device-tree */
	ret = of_get_named_gpio(dev->of_node, "mmi,stow-gpio", 0);
	if (ret < 0) {
		dev_info(dev, "can't find stow-gpio");
		drvdata->stow_gpio = 0;
		return ret;
	} else {
		dev_info(dev, "get stow-gpio[%d] from dt", ret);
		drvdata->stow_gpio = ret;
	}

	ret = devm_gpio_request(dev, drvdata->stow_gpio, "mmi-stow-gpio");
	if (ret < 0) {
		dev_err(dev, "Failed to request stow-gpio, ret:%d", ret);
		return ret;
	}

	g_stow_gpio = drvdata->stow_gpio;

	ret = misc_register(&mmi_stow_misc);
	if (ret) {
		dev_err(&pdev->dev, "Error registering device %d\n", ret);
		return ret;
	}

	dev_info(dev, "probe: All success !\n");

	return ret;
}

static void mmi_stow_remove(struct platform_device *pdev)
{
	struct mmi_stow_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	if (gpio_is_valid(drvdata->stow_gpio))
		gpio_free(drvdata->stow_gpio);
	misc_deregister(&mmi_stow_misc);
	dev_set_drvdata(&pdev->dev, NULL);
}

static const struct of_device_id mmi_stow_match_table[] = {
	{ .compatible = "moto,mmi_stow" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mmi_stow_match_table);

static struct platform_driver mmi_stow_driver = {
	.driver	= {
		.name		= "mmi_stow",
		.of_match_table	= mmi_stow_match_table,
	},
	.probe	= mmi_stow_probe,
	.remove	= mmi_stow_remove,
};
module_platform_driver(mmi_stow_driver);

MODULE_DESCRIPTION("MOTO mmi stow driver");
MODULE_LICENSE("GPL v2");
