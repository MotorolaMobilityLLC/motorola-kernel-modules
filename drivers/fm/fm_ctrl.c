/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#ifdef SUPPORT_FM_ELNA_LDO
#include <linux/regulator/consumer.h>
#endif


#define DRIVER_VERSION "0.0.1"


struct fm_ctrl_drvdata {
	struct device	*dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pstate_default;
	struct pinctrl_state *pstate_active;
	struct pinctrl_state *pstate_suspend;
	bool   factory_mode;
#ifdef SUPPORT_FM_ELNA_LDO
	/* voltage regulator handle */
	struct regulator *reg;
	/* voltage levels to be set */
	unsigned int low_vol_level;
	unsigned int high_vol_level;
#endif
};


static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static ssize_t device_name_read(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "moto fm control intf\n");
}


static ssize_t elna_en_read(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "fm_ctrl: not support!\n");
}

static ssize_t elna_en_write(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int res = 0;

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &res);
	if(ret) {
		pr_err("fm_ctrl failed to get data, set as default!\n");
	}
	if(1 == res) {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_active);
	}
	else {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_suspend);
	}
	if(ret) {
		pr_err("fm_ctrl failed to set pinctrl!\n");
	}
	else {
		pr_info("fm_ctrl set elan=%u\n", res);
	}

	return count;
}


static int fm_ctrl_pinctrl_dt_parse(struct device *dev) {
	int ret = 0;
	struct fm_ctrl_drvdata *drvdata = dev_get_drvdata(dev);

	/* Get pinctrl if target uses pinctrl */
	drvdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(drvdata->pinctrl)) {
		ret = PTR_ERR(drvdata->pinctrl);
		pr_err("%s: Pincontrol DT property returned %X\n", __func__, ret);
		return ret;
	}

	drvdata->pstate_default = pinctrl_lookup_state(drvdata->pinctrl,
		"default");
	if (IS_ERR_OR_NULL(drvdata->pstate_default)) {
		ret = PTR_ERR(drvdata->pstate_default);
		pr_err("Can not lookup default pinstate %d\n", ret);
		return -ENOENT;
	}
	drvdata->pstate_active = pinctrl_lookup_state(drvdata->pinctrl,
		"elna_active");
	if (IS_ERR_OR_NULL(drvdata->pstate_active)) {
		ret = PTR_ERR(drvdata->pstate_active);
		pr_err("Can not lookup active pinstate %d\n", ret);
		return ret;
	}
	drvdata->pstate_suspend = pinctrl_lookup_state(drvdata->pinctrl,
		"elna_suspend");
	if (IS_ERR_OR_NULL(drvdata->pstate_suspend)) {
		ret = PTR_ERR(drvdata->pstate_suspend);
		pr_err("Can not lookup suspend pinstate %d\n", ret);
		return ret;
	}

	return ret;
}


#ifdef SUPPORT_FM_ELNA_LDO
static int fm_ctrl_vreg_config(struct device *dev) {
	int ret = 0;
	struct fm_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	struct regulator *vddvreg = NULL;
	uint32_t vol_suply[2];

	vddvreg = regulator_get(dev, "vdd-elna");
	if (IS_ERR(vddvreg)) {
		ret = PTR_ERR(vddvreg);
		pr_err("%s: elna regulator_get failed ret = %d\n", __func__, ret);
		return ret;
	}
	drvdata->reg = vddvreg;

	ret = of_property_read_u32_array(np, "moto,vdd-elna-voltage", vol_suply, 2);
	if (ret < 0) {
		pr_err("%s: elna vreg info parse failed ret = %d\n", __func__, ret);
		ret =  -EINVAL;
	}
	else {
		drvdata->low_vol_level = vol_suply[0];
		drvdata->high_vol_level = vol_suply[1];
		ret = regulator_set_voltage(vddvreg, \
			drvdata->low_vol_level, \
			drvdata->high_vol_level);
		if (ret < 0) {
			pr_err("%s: elna vreg set voltage failed ret = %d\n", __func__, ret);
		}
		else {
			ret = regulator_enable(vddvreg);
			if (ret < 0) {
				pr_err("%s: elna vreg enable failed ret = %d\n", __func__, ret);
				regulator_set_voltage(vddvreg, \
					0, \
					drvdata->high_vol_level);
			}
			else {
				pr_info("%s: enable elan regulator done.\n", __func__);
			}
		}
	}

	return ret;
}
#endif


static DEVICE_ATTR(device_name, 0444, device_name_read, NULL);
static DEVICE_ATTR(elna_en, 0644, elna_en_read, elna_en_write);

static struct attribute *fm_ctrl_sysfs_attrs[] = {
	&dev_attr_device_name.attr,
	&dev_attr_elna_en.attr,
	NULL,
};

static struct attribute_group fm_ctrl_sysfs_attr_grp = {
	.attrs = fm_ctrl_sysfs_attrs,
};


static int fm_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fm_ctrl_drvdata *drvdata;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s begin\n", __func__);
	drvdata = devm_kzalloc(dev, sizeof(struct fm_ctrl_drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	ret = fm_ctrl_pinctrl_dt_parse(dev);
	if (0 != ret) {
		kfree(drvdata);
		return ret;
	}

#ifdef SUPPORT_FM_ELNA_LDO
	ret = fm_ctrl_vreg_config(dev);
	if (0 != ret) {
		kfree(drvdata);
		return ret;
	}
#endif

	drvdata->factory_mode = mmi_factory_check();
	if(drvdata->factory_mode) {
		ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_active);
		if(ret) {
			pr_err("fm_ctrl failed to set pinctrl @factory mode!\n");
		}
		else {
			pr_info("fm_ctrl enable elan @ factory mode\n");
		}
	}
	else {
		ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_default);
		if(ret) {
			pr_err("fm_ctrl failed to set pinctrl as default!\n");
		}
		else {
			pr_info("fm_ctrl disable elan as default.\n");
		}
	}

	ret = sysfs_create_group(&dev->kobj, &fm_ctrl_sysfs_attr_grp);
	if (ret) {
		pr_err("%s: sysfs group creation failed %d\n", __func__, ret);
	#ifdef SUPPORT_FM_ELNA_LDO
		regulator_put(drvdata->reg);
	#endif
		kfree(drvdata);
		return ret;
	}

	/* remove this because we donot need to support ant auto-det feature */
	//device_init_wakeup(&pdev->dev, 1);
	dev_info(&pdev->dev, "probe: All success !\n");

	return ret;
}

static int fm_ctrl_remove(struct platform_device *pdev)
{
	struct fm_ctrl_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	/* remove this because we donot need to support ant auto-det feature */
	//device_init_wakeup(&pdev->dev, 0);
	sysfs_remove_group(&pdev->dev.kobj, &fm_ctrl_sysfs_attr_grp);
#ifdef SUPPORT_FM_ELNA_LDO
	regulator_put(drvdata->reg);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(drvdata);

	return 0;
}


static const struct of_device_id fm_ctrl_match[] = {
	{ .compatible = "moto,fmctrl" },
	{}
};

static struct platform_driver fm_ctrl_plat_driver = {
	.probe = fm_ctrl_probe,
	.remove = fm_ctrl_remove,
	.driver = {
		.name = "fm_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = fm_ctrl_match,
	},
};

module_platform_driver(fm_ctrl_plat_driver);


MODULE_AUTHOR("Motorola Mobiity");
MODULE_DESCRIPTION("FMRadio control interface driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
