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

#include "goodix_thp.h"
#include "goodix_thp_mmi.h"
#include <linux/delay.h>
#include <linux/input/mt.h>
#include "goodix_thp_config.h"

extern struct goodix_thp_core *gdix_thp_core;

static ssize_t goodix_ts_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_log_trigger_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
static ssize_t goodix_ts_log_trigger_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_stowed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stowed_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_edge_show, goodix_ts_edge_store);
static DEVICE_ATTR(log_trigger, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_log_trigger_show, goodix_ts_log_trigger_store);
static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_interpolation_show, goodix_ts_interpolation_store);
static DEVICE_ATTR(sample, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_sample_show, goodix_ts_sample_store);
static DEVICE_ATTR(stowed, (S_IWUSR | S_IWGRP | S_IRUGO),
	goodix_ts_stowed_show, goodix_ts_stowed_store);
static DEVICE_ATTR(timestamp, S_IRUGO, goodix_ts_timestamp_show, NULL);

/* hal settings */
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define SMALL_MODE    2
#define DEFAULT_MODE   0
#define MAX_ATTRS_ENTRIES 10

#define NORMAL_DEFAULT_MODE 10
#define NORMAL_SMALL_MODE 11
#define NORMAL_BIG_MODE 12

#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

static int goodix_ts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;

	ADD_ATTR(edge);
	ADD_ATTR(log_trigger);
	if (core_data->ts_dev->board_data.interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (core_data->ts_dev->board_data.sample_ctrl)
		ADD_ATTR(sample);

	if (core_data->ts_dev->board_data.stowed_mode_ctrl)
		ADD_ATTR(stowed);

	ADD_ATTR(timestamp);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}
/*
 * HAL: args[0] suppression area, args[1] rotation direction.
 * CMD: [06 17 data0 data1],
 *      data[0] rotation direction, data[1] suppression area.
 */
static ssize_t goodix_ts_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	u8 tmp_cmd[16];
	u16 checksum = 0;
	int i;
	int edge_cmd[2] = { 0 };
	unsigned int args[2] = { 0 };
	struct goodix_thp_core *core_data = gdix_thp_core;

	ret = sscanf(buf, "%d %d", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;

	switch (args[0]) {
	case DEFAULT_MODE:
		edge_cmd[1] = DEFAULT_EDGE;
		break;
	case SMALL_MODE:
		edge_cmd[1] = SMALL_EDGE;
		break;
	case BIG_MODE:
		edge_cmd[1] = BIG_EDGE;
		break;
	case NORMAL_DEFAULT_MODE:
		edge_cmd[1] = NORMAL_DEFAULT_EDGE;
		break;
	case NORMAL_SMALL_MODE:
		edge_cmd[1] = NORMAL_SMALL_EDGE;
		break;
	case NORMAL_BIG_MODE:
		edge_cmd[1] = NORMAL_BIG_EDGE;
		break;
	default:
		ts_err("Invalid edge mode: %d!\n", args[0]);
		return -EINVAL;
	}

	if (ROTATE_0 == args[1]) {
		edge_cmd[0] = ROTATE_DEFAULT_0;
	} else if (ROTATE_90 == args[1]) {
		edge_cmd[0] = ROTATE_RIGHT_90;
	} else if (ROTATE_270 == args[1]) {
		edge_cmd[0] = ROTATE_LEFT_90;
	} else {
		ts_err("Invalid rotation mode: %d!\n", args[1]);
		return -EINVAL;
	}

	tmp_cmd[0] = 0x00; //status
	tmp_cmd[1] = 0x00; //ack
	tmp_cmd[2] = 0x06; //cmd len
	tmp_cmd[3] = 0x17; //cmd id
	tmp_cmd[4] = (u8)edge_cmd[0]; //cmd param 0,1,2
	tmp_cmd[5] = (u8)edge_cmd[1]; //cmd param 0,1,2
	for (i = 0; i < 6; i++)
		checksum += tmp_cmd[i];
	tmp_cmd[6] = (u8)checksum; //checkum lo
	tmp_cmd[7] = (u8)(checksum >> 8); //checksum hi

	memcpy(core_data->get_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
	put_frame_list(core_data, REQUEST_TYPE_CMD, tmp_cmd, 8);
	memcpy(core_data->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd));

	msleep(20);
	ret = size;
	ts_info("Success to set edge = %02x, rotation = %02x", edge_cmd[1], edge_cmd[0]);

	return ret;
}

static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("edge area = %02x, rotation = %02x\n",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
}

static int goodix_ts_mmi_charger_mode(struct device *dev, int mode)
{
	struct goodix_thp_core *core_data = gdix_thp_core;
	u8 tmp_cmd[16];
	u16 checksum = 0;
	int i;

	tmp_cmd[0] = 0x00; //status
	tmp_cmd[1] = 0x00; //ack
	tmp_cmd[2] = 0x05; //cmd len
	tmp_cmd[3] = 0xaf; //cmd id
	tmp_cmd[4] = (u8)mode; //cmd param 0,1,2
	for (i = 0; i < 5; i++)
		checksum += tmp_cmd[i];
	tmp_cmd[5] = (u8)checksum; //checkum lo
	tmp_cmd[6] = (u8)(checksum >> 8); //checksum hi

	put_frame_list(core_data, REQUEST_TYPE_CMD, tmp_cmd, 7);

	ts_info("Success to %s charger mode\n", mode ? "Enable" : "Disable");

	return 0;
}

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}

static ssize_t goodix_ts_log_trigger_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_thp_core *core_data = gdix_thp_core;
        u8 val[1] = {NOTIFY_TYPE_DUMP_REP};

	if (!buf || count <= 0)
		return 0;


        if (buf[0] == '1' || buf[0] == 1) {
                ts_info("dump rep log");
                put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, 1);
        }

	return count;
}

static ssize_t goodix_ts_log_trigger_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x", 0x01);
}

static int goodix_thp_mmi_set_report_rate(struct goodix_thp_core *core_data)
{
	int ret = 0;
	int mode = 0;
	struct thp_ts_device *tdev = core_data->ts_dev;

	mode = goodix_thp_mmi_get_report_rate(core_data);
	if (mode == -1) {
		return -EINVAL;
	}

	core_data->get_mode.report_rate_mode = mode;
	if (core_data->set_mode.report_rate_mode == mode) {
		ts_debug("The value = %d is same, so not to write", mode);
		return 0;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	//if now on high report rate and need switch to low report rate
	if ((((core_data->set_mode.report_rate_mode >> 8) & 0xFF) == REPORT_RATE_CMD_HIGH) &&
		(((mode >> 8) & 0xFF) == REPORT_RATE_CMD_LOW)) {
		ts_info("exit high report rate");
		ret = tdev->hw_ops->send_cmd(tdev, EXIT_HIGH_REPORT_RATE_CMD >> 8,
							EXIT_HIGH_REPORT_RATE_CMD & 0xFF);
		if (ret < 0) {
			ts_err("failed to exit high report rate");
			return -EINVAL;
		}
		msleep(20);
	}

	//send switch command
	ret = tdev->hw_ops->send_cmd(tdev, mode >> 8, mode & 0xFF);
	if (ret < 0) {
		ts_err("failed to set report rate, mode = %d", mode);
		return -EINVAL;
	}
	msleep(20);

	core_data->set_mode.report_rate_mode = mode;

	ts_info("Success to set %s\n", mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				(mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_360HZ" :
				(mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
				(mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
				(mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
				(mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120HZ" :
				"Unsupported"))))));

	return ret;
}

static ssize_t goodix_ts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.interpolation = mode;
	ret = goodix_thp_mmi_set_report_rate(core_data);
	if (ret < 0)
		goto exit;

	ret = size;
	core_data->set_mode.interpolation = mode;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("interpolation = %d.\n", core_data->set_mode.interpolation);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.interpolation);
}

static ssize_t goodix_ts_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_ts_device *tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.sample= mode;
	if (core_data->set_mode.sample == mode) {
		ts_debug("The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		ret = size;
		goto exit;
	}

	ret = tdev->hw_ops->send_cmd(tdev, SAMPLE_SWITCH_CMD,
						core_data->get_mode.sample);
	if (ret < 0) {
		ts_err("failed to set sample rate, mode = %lu", mode);
		goto exit;
	}

	core_data->set_mode.sample = mode;
	msleep(20);
	ts_info("Success to set %lu\n", mode);

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("sample = %d.\n", core_data->set_mode.sample);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.sample);
}

static ssize_t goodix_ts_stowed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_ts_device *tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.stowed = mode;
	if (core_data->set_mode.stowed == mode) {
		ts_debug("The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if ( core_data->power_on == 1) {
		ret = tdev->hw_ops->send_cmd(tdev, ENTER_STOWED_MODE_CMD,
			core_data->get_mode.stowed);
		if (ret < 0) {
			ts_err("Failed to set stowed mode %lu\n", mode);
			goto exit;
		}
	} else {
		ts_info("Skip stowed mode setting power_on:%d.\n", core_data->power_on);
		ret = size;
		goto exit;
	}

	core_data->set_mode.stowed = mode;
	ts_info("Success to set stowed mode %lu\n", mode);

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_stowed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("Stowed state = %d.\n", core_data->set_mode.stowed);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.stowed);
}

static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct timeval64 last_ts;

	mutex_lock(&core_data->mode_lock);
	last_ts = core_data->last_event_time;
	mutex_unlock(&core_data->mode_lock);

	return scnprintf(buf, PAGE_SIZE, "%lld.%lld\n", last_ts.tv_sec, last_ts.tv_usec);
}

static int goodix_berlin_gesture_setup(struct goodix_thp_core *core_data)
{
	int ret = 0;
	unsigned char gesture_type = 0;
	u8 val[3];

	val[0] = NOTIFY_TYPE_GESTURE;
	val[1] = 0x0;
	val[2] = 0x0;
	if (core_data->imports && core_data->imports->get_gesture_type) {
		ret = core_data->imports->get_gesture_type(core_data->ts_dev->dev, &gesture_type);
		ts_info("Provisioned gestures 0x%02x; rc = %d\n", gesture_type, ret);
	}

	if (gesture_type & TS_MMI_GESTURE_ZERO) {
		val[2] = val[2] | 0x20;
	}
	if (gesture_type & TS_MMI_GESTURE_SINGLE) {
		val[2] = val[2] | 0x10;
	}
	if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
		val[1] = val[1] | 0x80;
	}


	ts_info("Send enable gesture mode 0x%x 0x%x\n", val[1], val[2]);
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	return 0;
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct goodix_thp_core *core_data = gdix_thp_core;
	u8 val[2];

	val[0] = NOTIFY_TYPE_SCREEN;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		goodix_berlin_gesture_setup(core_data);
		val[1] = 0;
		msleep(16);
		break;
	case TS_MMI_PM_DEEPSLEEP:
		val[1] = 0;
		break;
	case TS_MMI_PM_ACTIVE:
		val[1] = 1;
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	return 0;
}

static struct ts_mmi_methods goodix_ts_mmi_methods = {
	.get_vendor = goodix_ts_mmi_methods_get_vendor,
	.charger_mode = goodix_ts_mmi_charger_mode,
	/* vendor specific attribute group */
	.extend_attribute_group = goodix_ts_mmi_extend_attribute_group,
	.panel_state = goodix_ts_mmi_panel_state,
};

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_thp_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_info("Failed to get driver data");
		return -ENODEV;
	}
	mutex_init(&core_data->mode_lock);
	ret = ts_mmi_dev_register(core_data->ts_dev->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		mutex_destroy(&core_data->mode_lock);
		return ret;
	}

	core_data->imports = &goodix_ts_mmi_methods.exports;

	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_thp_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_info("Failed to get driver data");
	else {
		mutex_destroy(&core_data->mode_lock);
		ts_mmi_dev_unregister(core_data->ts_dev->dev);
	}
}
