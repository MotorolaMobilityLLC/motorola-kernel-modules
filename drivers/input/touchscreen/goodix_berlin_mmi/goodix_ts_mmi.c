/*
 * Copyright (C) 2019 Motorola Mobility LLC
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

#include <linux/touchscreen_mmi.h>
#include "goodix_ts_mmi.h"
#include <linux/input/mt.h>

#define GET_GOODIX_DATA(dev) { \
	pdev = dev_get_drvdata(dev); \
	if (!pdev) { \
		ts_err("Failed to get platform device"); \
		return -ENODEV; \
	} \
	core_data = platform_get_drvdata(pdev); \
	if (!core_data) { \
		ts_err("Failed to get driver data"); \
		return -ENODEV; \
	} \
}

extern int goodix_ts_unregister_notifier(struct notifier_block *nb);
extern struct goodix_module goodix_modules;

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}

static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_fw_version fw_version;

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->read_version(core_data, &fw_version, false);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "GT%s",
			fw_version.patch_pid);
}

static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->get_ic_info(core_data, &core_data->ic_info, false);
	if (!ret) {
		ret = snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN,
			"%08x", core_data->ic_info.version.config_id);
		return ret;
	} else
		return -EINVAL;
}

/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_fw_version chip_ver;

	GET_GOODIX_DATA(dev);
	ret = core_data->hw_ops->read_version(core_data, &chip_ver, false);
	if (!ret) {
		ret = snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN,
			"%02x%02x", chip_ver.patch_vid[2], chip_ver.patch_vid[3]);

		return ret;
	} else
		return -EINVAL;
}

static int goodix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->bus->bus_type == GOODIX_BUS_TYPE_I2C ?
			TOUCHSCREEN_MMI_BUS_TYPE_I2C : TOUCHSCREEN_MMI_BUS_TYPE_SPI;
	return 0;
}

static int goodix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_ts_board_data *ts_bdata;

	GET_GOODIX_DATA(dev);

	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(ts_bdata->irq_gpio);
	return 0;
}

static int goodix_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = atomic_read(&core_data->irq_enabled);
	return 0;
}

static int goodix_ts_mmi_methods_get_poweron(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->power_on;
	return 0;
}

static int goodix_ts_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->update_status;
	return 0;
}

static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->hw_ops->irq_enable)
		ret = core_data->hw_ops->irq_enable(core_data, !(!state));

	return ret;
}

/* reset chip
 * type：can control software reset and hardware reset,
 * but GOODIX has no software reset,
 * so the type parameter is not used here.
 */
static int goodix_ts_mmi_methods_reset(struct device *dev, int type) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->hw_ops->reset)
		ret = core_data->hw_ops->reset(core_data, GOODIX_NORMAL_RESET_DELAY_MS);
	return ret;
}

static int goodix_ts_firmware_update(struct device *dev, char *fwname) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	ts_info("HW request update fw, %s", fwname);
	/* set firmware image name */
	if (core_data->set_fw_name)
		core_data->set_fw_name(fwname);

	ret = goodix_do_fw_update(NULL,
				UPDATE_MODE_SRC_REQUEST | UPDATE_MODE_BLOCK | UPDATE_MODE_FORCE);

	if (ret)
		ts_err("failed do fw update");

	return 0;
}

static int goodix_ts_mmi_methods_power(struct device *dev, int on) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (on == TS_MMI_POWER_ON)
		return goodix_ts_power_on(core_data);
	else if(on == TS_MMI_POWER_OFF)
		return goodix_ts_power_off(core_data);
	else {
		ts_err("Invalid power parameter %d.\n", on);
		return -EINVAL;
	}
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		if (hw_ops->gesture)
			hw_ops->gesture(core_data, 0);
		enable_irq_wake(core_data->irq);
		core_data->gesture_enabled = true;
		break;
	case TS_MMI_PM_DEEPSLEEP:
		/* enter sleep mode or power off */
		if (hw_ops->suspend)
			hw_ops->suspend(core_data);
		core_data->gesture_enabled = false;
		break;
	case TS_MMI_PM_ACTIVE:
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	return 0;
}

static int goodix_ts_mmi_pre_resume(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	ts_info("Resume start");
	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	GET_GOODIX_DATA(dev);

	atomic_set(&core_data->suspended, 0);
	if (core_data->gesture_enabled)
		disable_irq_wake(core_data->irq);

	if (hw_ops->resume)
		hw_ops->resume(core_data);

	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);
	return 0;
}

static int goodix_ts_mmi_post_suspend(struct device *dev) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	goodix_ts_release_connects(core_data);

	ts_info("Suspend end");
	return 0;
}

static struct ts_mmi_methods goodix_ts_mmi_methods = {
	.get_vendor = goodix_ts_mmi_methods_get_vendor,
	.get_productinfo = goodix_ts_mmi_methods_get_productinfo,
	.get_build_id = goodix_ts_mmi_methods_get_build_id,
	.get_config_id = goodix_ts_mmi_methods_get_config_id,
	.get_bus_type = goodix_ts_mmi_methods_get_bus_type,
	.get_irq_status = goodix_ts_mmi_methods_get_irq_status,
	.get_drv_irq = goodix_ts_mmi_methods_get_drv_irq,
	.get_poweron = goodix_ts_mmi_methods_get_poweron,
	.get_flashprog = goodix_ts_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  goodix_ts_mmi_methods_reset,
	.drv_irq = goodix_ts_mmi_methods_drv_irq,
	.power = goodix_ts_mmi_methods_power,
	/* Firmware */
	.firmware_update = goodix_ts_firmware_update,
	/* PM callback */
	.panel_state = goodix_ts_mmi_panel_state,
	.pre_resume = goodix_ts_mmi_pre_resume,
	.pre_suspend = goodix_ts_mmi_pre_suspend,
	.post_suspend = goodix_ts_mmi_post_suspend,
};

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ret = ts_mmi_dev_register(core_data->bus->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		return ret;
	}
	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	ts_mmi_dev_unregister(&pdev->dev);
}
