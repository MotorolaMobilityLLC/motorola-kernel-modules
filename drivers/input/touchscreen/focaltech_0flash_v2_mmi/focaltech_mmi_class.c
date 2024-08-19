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

#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>
#include "focaltech_core.h"
#include "focaltech_config.h"

#define MAX_ATTRS_ENTRIES 10

#define GET_TS_DATA(dev) { \
	ts_data = dev_get_drvdata(dev); \
	if (!ts_data) { \
		FTS_ERROR("Failed to get driver data"); \
		return -ENODEV; \
	} \
}

#ifdef FTS_LAST_TIME_EN
static ssize_t fts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf);
#endif

#ifdef FTS_LAST_TIME_EN
static DEVICE_ATTR(timestamp, S_IRUGO, fts_timestamp_show, NULL);
#endif

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

static ssize_t fts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_interpolation_show, fts_interpolation_store);

static int fts_mmi_set_report_rate(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int mode = 0;

	mode = fts_mmi_get_report_rate(ts_data);
	if (mode == -1) {
		return -EINVAL;
	}

	ts_data->get_mode.report_rate_mode = mode;
	if (ts_data->set_mode.report_rate_mode == mode) {
		FTS_DEBUG("The value = %d is same, so not to write", mode);
		return 0;
	}

	if (ts_data->power_disabled) {
		FTS_DEBUG("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	ret = fts_write_reg(FTS_CMD_REPORT_RATE_ADDR, mode);
	if (ret < 0) {
		FTS_ERROR("failed to set report rate, mode = %d", mode);
		return -EINVAL;
	}
	msleep(20);

	ts_data->set_mode.report_rate_mode = mode;

	FTS_INFO("Success to set %s\n", mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120HZ" :
				(mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				"Unsupported"));

	return ret;
}

static ssize_t fts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct fts_ts_data *ts_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		FTS_ERROR("Failed to convert value.");
		return -EINVAL;
	}
	mode = (mode == 2 ? 1 : mode);
	mutex_lock(&ts_data->mode_lock);
	ts_data->get_mode.interpolation = mode;
	ret = fts_mmi_set_report_rate(ts_data);
	if (ret < 0)
		goto exit;

	ret = size;
	ts_data->set_mode.interpolation = mode;
exit:
	mutex_unlock(&ts_data->mode_lock);
	return ret;
}

static ssize_t fts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	FTS_INFO("interpolation = %d.", ts_data->set_mode.interpolation);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", (ts_data->set_mode.interpolation == 1 ? 2 : 0));
}

#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

static int fts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct fts_ts_data *ts_data;
	struct fts_ts_platform_data *pdata;

	GET_TS_DATA(dev);
	pdata = ts_data->pdata;

#ifdef FTS_LAST_TIME_EN
	ADD_ATTR(timestamp);
#endif

	if (pdata->interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

#ifdef FTS_LAST_TIME_EN
static ssize_t fts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;
	ktime_t last_ktime;
	struct timespec64 last_ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	last_ktime = ts_data->last_event_time;
	ts_data->last_event_time = 0;
	mutex_unlock(&input_dev->mutex);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
#endif

static int fts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "focaltech");
}

static int fts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", FTS_CHIP_NAME);
}

/*return firmware version*/
static int fts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", 0);
}

static int fts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;
	ssize_t num_read_chars = 0;
	u8 fwver = 0;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;
	mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	fts_read_reg(FTS_REG_FW_VER, &fwver);
#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif
	num_read_chars = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fwver);

	mutex_unlock(&input_dev->mutex);
	return num_read_chars;
}

static int fts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	return 0;
}

static int fts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	return 0;
}

static int fts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	return 0;
}

static int fts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	return 0;
}

static int fts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	return 0;
}

static int fts_mmi_methods_drv_irq(struct device *dev, int state)
{
	return 0;
}

static int fts_mmi_methods_reset(struct device *dev, int type)
{
	return 0;
}

static int fts_mmi_firmware_update(struct device *dev, char *fwname)
{
	return 0;
}

static int fts_mmi_charger_mode(struct device *dev, int mode)
{
	struct fts_ts_data *ts_data;
	int ret = 0;

	GET_TS_DATA(dev);
	ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, mode);
	if(ret < 0){
		FTS_ERROR("Failed to set charger mode\n");
	}

	FTS_INFO("Success to %s charger mode\n", mode ? "Enable" : "Disable");

	return 0;
}

static int fts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct fts_ts_data *ts_data = fts_data;

#if defined(CONFIG_FTS_DOUBLE_TAP_CONTROL)
	unsigned char gesture_type = 0;
#endif

	FTS_INFO("panel state change: %d->%d\n", from, to);
	switch (to) {
		case TS_MMI_PM_GESTURE:
#ifdef FTS_SET_TOUCH_STATE
			ts_data->gesture_support = true;
			touch_set_state(TS_MMI_PM_GESTURE, TOUCH_PANEL_IDX_PRIMARY);
#else
			fts_data->gesture_support = false;
#endif

#if defined(CONFIG_FTS_DOUBLE_TAP_CONTROL)
			if (ts_data->imports && ts_data->imports->get_gesture_type) {
				ts_data->imports->get_gesture_type(ts_data->dev, &gesture_type);
			}

			if (gesture_type & TS_MMI_GESTURE_SINGLE) {
				ts_data->gesture_cmd += 0x01;
				FTS_INFO("enable single gesture mode cmd 0x%04x\n", ts_data->gesture_cmd);
			}

			if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
				ts_data->gesture_cmd += 0x02;
				FTS_INFO("enable double gesture mode cmd 0x%04x\n", ts_data->gesture_cmd);
			}
#endif
			break;

		case TS_MMI_PM_DEEPSLEEP:
			fts_data->gesture_support = false;
#ifdef FTS_SET_TOUCH_STATE
			touch_set_state(TS_MMI_PM_DEEPSLEEP, TOUCH_PANEL_IDX_PRIMARY);
#endif

			break;

		case TS_MMI_PM_ACTIVE:
			break;
		default:
			dev_warn(dev, "panel mode %d is invalid.\n", to);
			return -EINVAL;
			break;
	}

	return 0;
}

static int fts_mmi_pre_resume(struct device *dev)
{
	return 0;
}

static int fts_mmi_post_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = fts_data;
	struct fts_ts_platform_data *pdata;
	int ret = -1;
	pdata = ts_data->pdata;
	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_INFO("Already in awake state");
		return 0;
	}

	ts_data->suspended = false;
	fts_release_all_finger();

	if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
		fts_power_source_resume(ts_data);
#endif
		fts_reset_proc(200);
	}


#if FTS_PINCTRL_EN && (!FTS_POWER_SOURCE_CUST_EN)
	fts_pinctrl_select_normal(ts_data);
#endif
#ifdef CONFIG_FTS_MULTI_FW
	fts_enter_normal_fw();
#endif
	fts_wait_tp_to_valid();
	fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume(ts_data);
#endif

#if FTS_GESTURE_EN
	fts_gesture_resume(ts_data);
#endif

	mutex_lock(&ts_data->mode_lock);
	/* All IC status are cleared after reset */
	memset(&ts_data->set_mode, 0 , sizeof(ts_data->set_mode));

	if (pdata->interpolation_ctrl && ts_data->get_mode.interpolation) {
		ret = fts_write_reg(FTS_CMD_REPORT_RATE_ADDR, ts_data->get_mode.report_rate_mode);
		if (ret >= 0) {
			ts_data->set_mode.interpolation = ts_data->get_mode.interpolation;
			ts_data->set_mode.report_rate_mode = ts_data->get_mode.report_rate_mode;

			FTS_INFO("Success to set %s interpolation mode\n",
				ts_data->get_mode.report_rate_mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				(ts_data->get_mode.report_rate_mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120HZ" :
				"Unsupported"));
		}
	}
	mutex_unlock(&ts_data->mode_lock);

	ts_data->suspended = false;

#if FTS_USB_DETECT_EN
	fts_cable_detect_func(true);
#endif

	FTS_FUNC_EXIT();

	return 0;
}

static int fts_mmi_pre_suspend(struct device *dev)
{
	return 0;
}

static int fts_mmi_post_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend(ts_data);
#endif

#if FTS_GESTURE_EN
	if (fts_gesture_suspend(ts_data) == 0) {
#if defined(CONFIG_FTS_DOUBLE_TAP_CONTROL)
		fts_write_reg(FTS_GESTURE_MODE, ts_data->gesture_cmd);
		ts_data->gesture_cmd = 0;
#endif
		ts_data->suspended = true;
		return 0;
	}
#endif

	FTS_INFO("make TP enter into sleep mode");
	ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
	if (ret < 0)
		FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

	if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
		ret = fts_power_source_suspend(ts_data);
		if (ret < 0) {
			FTS_ERROR("power enter suspend fail");
		}
#endif
	}

#if FTS_PINCTRL_EN && (!FTS_POWER_SOURCE_CUST_EN)
	fts_pinctrl_select_suspend(ts_data);
#endif
	fts_release_all_finger();
	ts_data->suspended = true;
	FTS_FUNC_EXIT();

	return 0;
}

static struct ts_mmi_methods fts_mmi_methods = {
	.get_vendor = fts_mmi_methods_get_vendor,
	.get_productinfo = fts_mmi_methods_get_productinfo,
	.get_build_id = fts_mmi_methods_get_build_id,
	.get_config_id = fts_mmi_methods_get_config_id,
	.get_bus_type = fts_mmi_methods_get_bus_type,
	.get_irq_status = fts_mmi_methods_get_irq_status,
	.get_drv_irq = fts_mmi_methods_get_drv_irq,
	.get_flashprog = fts_mmi_methods_get_flashprog,
	.get_poweron = fts_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  fts_mmi_methods_reset,
	.drv_irq = fts_mmi_methods_drv_irq,
	.charger_mode = fts_mmi_charger_mode,
	/* Firmware */
	.firmware_update = fts_mmi_firmware_update,
	/* vendor specific attribute group */
	.extend_attribute_group = fts_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = fts_mmi_panel_state,
	.pre_resume = fts_mmi_pre_resume,
	.post_resume = fts_mmi_post_resume,
	.pre_suspend = fts_mmi_pre_suspend,
	.post_suspend = fts_mmi_post_suspend,

};

int fts_mmi_dev_register(struct fts_ts_data *ts_data) {
	int ret;
	mutex_init(&ts_data->mode_lock);
	ret = ts_mmi_dev_register(ts_data->dev, &fts_mmi_methods);
	if (ret) {
		dev_err(ts_data->dev, "Failed to register ts mmi\n");
		mutex_destroy(&ts_data->mode_lock);
		return ret;
	}

	/* initialize class imported methods */
	ts_data->imports = &fts_mmi_methods.exports;

	return 0;
}

void fts_mmi_dev_unregister(struct fts_ts_data *ts_data) {
	mutex_destroy(&ts_data->mode_lock);
	ts_mmi_dev_unregister(ts_data->dev);
}
