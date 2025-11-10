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
static ssize_t goodix_ts_pocket_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_pocket_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_fp_int_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_fp_int_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_ble_broadcast_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_hardware_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_device_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_fw_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_fw_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_report_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_stylus_report_rate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

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
static DEVICE_ATTR(pocket_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_pocket_mode_show, goodix_ts_pocket_mode_store);
static DEVICE_ATTR(stylus_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_stylus_mode_show, goodix_ts_stylus_mode_store);
static DEVICE_ATTR(fp_int, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_fp_int_show, goodix_ts_fp_int_store);
static DEVICE_ATTR(ble_broadcast, (S_IRUGO | S_IWUSR | S_IWGRP),
	NULL, goodix_ts_ble_broadcast_store);
static DEVICE_ATTR(hardware_status, S_IRUGO, goodix_ts_hardware_status_show, NULL);
static DEVICE_ATTR(device_id, (S_IRUGO | S_IWUSR | S_IWGRP),
	NULL, goodix_ts_device_id_store);
static DEVICE_ATTR(fw_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_fw_mode_show, goodix_ts_fw_mode_store);
static DEVICE_ATTR(stylus_report_rate, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_stylus_report_rate_show, goodix_ts_stylus_report_rate_store);

/* hal settings */
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define SMALL_MODE    2
#define DEFAULT_MODE   0
#define MAX_ATTRS_ENTRIES 15

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

#define GET_GOODIX_DATA(dev) { \
	pdev = dev_get_drvdata(dev); \
	if (!pdev) { \
		ts_err(NULL, "Failed to get platform device"); \
		return -ENODEV; \
	} \
	core_data = platform_get_drvdata(pdev); \
	if (!core_data) { \
		ts_err(NULL, "Failed to get driver data"); \
		return -ENODEV; \
	} \
}


static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

static int goodix_ts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->ts_dev->board_data.edge_ctrl)
		ADD_ATTR(edge);

	ADD_ATTR(log_trigger);

	if (core_data->ts_dev->board_data.interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (core_data->ts_dev->board_data.sample_ctrl)
		ADD_ATTR(sample);

	if (core_data->ts_dev->board_data.stowed_mode_ctrl)
		ADD_ATTR(stowed);

	if (core_data->ts_dev->board_data.pocket_mode_ctrl)
		ADD_ATTR(pocket_mode);

	ADD_ATTR(timestamp);

	if (core_data->ts_dev->board_data.stylus_mode_ctrl)
		ADD_ATTR(stylus_mode);

	ADD_ATTR(fp_int);

	ADD_ATTR(ble_broadcast);
	ADD_ATTR(hardware_status);
	ADD_ATTR(device_id);
	ADD_ATTR(fw_mode);

	if (core_data->ts_dev->board_data.stylus_interpolation_ctrl)
		ADD_ATTR(stylus_report_rate);

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
	int edge_cmd[2] = { 0 };
	unsigned int args[2] = { 0 };
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	struct thp_ts_device *ts_dev;
	u8 val[3];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;

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
		ts_err(ts_dev->dev, "Invalid edge mode: %d!", args[0]);
		return -EINVAL;
	}

	if (ROTATE_0 == args[1]) {
		edge_cmd[0] = ROTATE_DEFAULT_0;
	} else if (ROTATE_90 == args[1]) {
		edge_cmd[0] = ROTATE_RIGHT_90;
	} else if (ROTATE_270 == args[1]) {
		edge_cmd[0] = ROTATE_LEFT_90;
	} else {
		ts_err(ts_dev->dev, "Invalid rotation mode: %d!", args[1]);
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	memcpy(core_data->get_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
	if (!memcmp(core_data->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd))) {
		ts_info(ts_dev->dev, "The value (%02x %02x) is same,so not write.",
		edge_cmd[0], edge_cmd[1]);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info(ts_dev->dev, "The touch is in sleep state, restore the value when resume");
		ret = size;
		goto exit;
	}

	val[0] = NOTIFY_TYPE_ROTATION;
	val[1] = (u8)edge_cmd[0];
	val[2] = (u8)edge_cmd[1];
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	memcpy(core_data->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd));

	msleep(20);
	ret = size;
	ts_info(ts_dev->dev, "Success to set edge = %02x, rotation = %02x", edge_cmd[1], edge_cmd[0]);
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "edge area = %02x, rotation = %02x",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
}

static int goodix_ts_mmi_charger_mode(struct device *dev, int mode)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	u8 val[2];

	GET_GOODIX_DATA(dev);

	core_data->get_mode.charger_mode= mode;
	if (core_data->set_mode.charger_mode == mode) {
		ts_info(core_data->ts_dev->dev, "The value = %lu is same, so not to write", mode);
		return 0;
	}

	if (core_data->power_on == 0) {
		ts_info(core_data->ts_dev->dev, "The touch is in sleep state, restore the value when resume");
		return 0;
	}

	val[0] = NOTIFY_TYPE_CHARGE;
	val[1] = (u8)mode;
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	core_data->set_mode.charger_mode = mode;
	msleep(20);
	ts_info(core_data->ts_dev->dev, "Success to %s charger mode", mode ? "enable" : "disable");
	return 0;
}

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}

static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	struct thp_ts_device *tdev;

	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	if (tdev->board_data.chip_type == CHIP_TYPE_9916)
		strncpy(tdev->board_data.ic_name, "gt9916P", sizeof(tdev->board_data.ic_name));
	else if (tdev->board_data.chip_type == CHIP_TYPE_9966)
		strncpy(tdev->board_data.ic_name, "gt9966", sizeof(tdev->board_data.ic_name));
	else if (tdev->board_data.chip_type == CHIP_TYPE_9615)
		strncpy(tdev->board_data.ic_name, "gt9615", sizeof(tdev->board_data.ic_name));

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", tdev->board_data.ic_name);
}

static ssize_t goodix_ts_log_trigger_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	u8 val[1] = {NOTIFY_TYPE_DUMP_REP};
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	if (!buf || count <= 0)
		return 0;

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info(core_data->ts_dev->dev, "dump rep log");
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, 1);
		msleep(20);
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
	struct device *dev = core_data->ts_dev->dev;
	int ret = 0;
	int mode = 0;
	u8 val[3];

	mode = goodix_thp_mmi_get_report_rate(core_data);
	if (mode == -1) {
		return -EINVAL;
	}

	core_data->get_mode.report_rate_mode = mode;
	if (core_data->set_mode.report_rate_mode == mode) {
		ts_info(dev, "The value = 0x%02x is same, so not to write", mode);
		return 0;
	}

	if (core_data->power_on == 0) {
		ts_info(dev, "The touch is in sleep state, restore the value when resume");
		return 0;
	}

	//send switch command
	val[0] = NOTIFY_TYPE_SWITCH_REPORT_RATE;
	val[1] = (mode >> 8) & 0xFF;
	val[2] = mode & 0xFF;
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	msleep(20);
	core_data->set_mode.report_rate_mode = mode;

	ts_info(dev, "Success to set %s", mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				(mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_300/360HZ" :
				(mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
				(mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
				(mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
				(mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120/130HZ" :
				"Unsupported"))))));

	return ret;
}

static ssize_t goodix_ts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_info(core_data->ts_dev->dev, "Failed to convert value.");
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
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "interpolation = %d.\n", core_data->set_mode.interpolation);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.interpolation);
}

static ssize_t goodix_ts_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	u8 val[2];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_info(tdev->dev, "Failed to convert value.");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.sample= mode;
	if (core_data->set_mode.sample == mode) {
		ts_info(tdev->dev, "The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in sleep state, restore the value when resume");
		ret = size;
		goto exit;
	}

	val[0] = NOTIFY_TYPE_GAME_MODE;
	val[1] = mode ? 1 : 0;
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	core_data->set_mode.sample = mode;
	msleep(20);
	ts_info(tdev->dev, "Success to %s game mode", mode ? "enable" : "disable");

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "sample = %d.", core_data->set_mode.sample);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.sample);
}

static ssize_t goodix_ts_pocket_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev,
			"Pocket mode state = %d.", core_data->set_mode.pocket_mode);
	return scnprintf(buf, PAGE_SIZE, "%d\n", core_data->set_mode.pocket_mode);
}

static ssize_t goodix_ts_pocket_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long value = 0;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	struct thp_ts_device *tdev;
	u8 val[2];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	mutex_lock(&core_data->mode_lock);
	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		ts_err(tdev->dev, "pocket_mode: Failed to convert value");
		mutex_unlock(&core_data->mode_lock);
		return -EINVAL;
	}
	switch (value) {
		case 0x10:
		case 0x20:
			ts_info(tdev->dev, "touch pocket mode disable");
			core_data->get_mode.pocket_mode = 0;
			break;
		case 0x11:
		case 0x21:
			ts_info(tdev->dev, "touch pocket mode enable");
			core_data->get_mode.pocket_mode = 1;
			break;
		default:
			ts_info(tdev->dev, "unsupport pocket mode type, value = %lu", value);
			mutex_unlock(&core_data->mode_lock);
			return -EINVAL;
	}

	if (core_data->set_mode.pocket_mode == core_data->get_mode.pocket_mode) {
		ts_info(tdev->dev, "The value = %d is same, so not to write", core_data->get_mode.pocket_mode);
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in sleep state, restore the value when resume");
		goto exit;
	}

	val[0] = NOTIFY_TYPE_POCKET_MODE;
	val[1] = core_data->get_mode.pocket_mode;
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	core_data->set_mode.pocket_mode = core_data->get_mode.pocket_mode;
	msleep(20);

	ts_info(tdev->dev, "Success to %s pocket mode", core_data->get_mode.pocket_mode ? "enable" : "disable");
exit:
	mutex_unlock(&core_data->mode_lock);
	return size;
}

static ssize_t goodix_ts_stowed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	u8 val[2];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.stowed = mode;
	if (core_data->set_mode.stowed == mode) {
		ts_info(tdev->dev, "The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 1) {
		val[0] = NOTIFY_TYPE_STOW_MODE;
		val[1] = mode ? 1 : 0;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		msleep(20);
	} else {
		ts_info(tdev->dev, "Skip stowed mode setting power_on:%d.", core_data->power_on);
		ret = size;
		goto exit;
	}

	core_data->set_mode.stowed = mode;
	ts_info(tdev->dev, "Success to %s stow mode", mode ? "enable" : "disable");

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_stowed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev,
			"Stowed state = %d.", core_data->set_mode.stowed);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.stowed);
}

static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ktime_t last_ktime;
	struct timespec64 last_ts;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	mutex_lock(&core_data->mode_lock);
	last_ktime = core_data->last_event_time;
	core_data->last_event_time = 0;
	mutex_unlock(&core_data->mode_lock);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}

static int goodix_clock_enable(struct goodix_thp_core *core_data, bool mode)
{
	int ret = 0;
	struct thp_ts_device *tdev = core_data->ts_dev;

	if (mode) {
		if (IS_ERR_OR_NULL(core_data->stylus_clk_active)) {
			ts_err(tdev->dev, "Failed to get state clk pinctrl state:%s",
				PINCTRL_STYLUS_CLK_ACTIVE);
			core_data->stylus_clk_active = NULL;
			return -EINVAL;
		}
		ret = pinctrl_select_state(core_data->pinctrl,
					core_data->stylus_clk_active);
		if (ret < 0) {
			ts_err(tdev->dev, "Failed to select active stylus clk state, ret:%d", ret);
			return ret;
		}
		ts_info(tdev->dev, "success to enable stylus clk");
	} else {
		if (IS_ERR_OR_NULL(core_data->stylus_clk_suspend)) {
			ts_err(tdev->dev, "Failed to get state clk pinctrl state:%s",
				PINCTRL_STYLUS_CLK_SUSPEND);
			core_data->stylus_clk_suspend = NULL;
			return -EINVAL;
		}
		ret = pinctrl_select_state(core_data->pinctrl,
					core_data->stylus_clk_suspend);
		if (ret < 0) {
			ts_err(tdev->dev, "Failed to select stylus clk suspend state, ret:%d", ret);
			return ret;
		}
		ts_info(tdev->dev, "success to disable stylus clk");
	}

	return ret;
}

static int goodix_stylus_mode(struct goodix_thp_core *core_data, int mode)
{
	int ret = 0;
	u8 val[2];

	val[0] = NOTIFY_TYPE_STYLUS_CTRL;

	if (mode) {
		goodix_clock_enable(core_data, mode);
		msleep(50);
		val[1] = 1;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		msleep(20);
	} else {
		val[1] = 0;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		msleep(50);
		goodix_clock_enable(core_data, mode);
	}

	if (core_data->ts_dev->board_data.stylus_interpolation_ctrl) {
		ts_info(core_data->ts_dev->dev, "Success to %s stylus mode, Stylus tip report rate %dHZ",
			mode ? "Enable" : "Disable",
			core_data->rate_configs[core_data->current_stylus_rate_mode].report_rate);
	} else {
		ts_info(core_data->ts_dev->dev, "Success to %s stylus mode, Stylus tip report rate mode:%d",
			mode ? "Enable" : "Disable", core_data->set_mode.stylus_report_rate_mode);
	}
	return ret;
}

static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	u8 val[3];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	mutex_lock(&core_data->mode_lock);
	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_err(tdev->dev, "Failed to convert value.");
		mutex_unlock(&core_data->mode_lock);
		return -EINVAL;
	}

	core_data->get_mode.stylus_mode = mode;
	if (core_data->set_mode.stylus_mode == mode) {
		ts_info(tdev->dev, "The value = %lu is same,so not write.", mode);
		goto exit;
	}

	/* 1) If on screen on state, here can switch stylus mode directly,
	* 2) If BLE enable stylus notify after IC resume done,
	*     here will do the stylus mode switch or will do the stylus mode switch on post resume
	*/
	if ((core_data->power_on == 0) || (core_data->suspended == 1)) {
		ts_info(tdev->dev, "The touch is in sleep state, restore the value when resume");
		goto exit;
	}

	ret = goodix_stylus_mode(core_data, mode);
	if (!ret)
		core_data->set_mode.stylus_mode = mode;

	/* when exit stylus mode, need check if need restore game mode */
	if (!core_data->set_mode.stylus_mode) {
		if (core_data->ts_dev->board_data.sample_ctrl && core_data->set_mode.sample) {
			val[0] = NOTIFY_TYPE_GAME_MODE;
			val[1] = core_data->set_mode.sample;
			put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
			msleep(20);
			ts_info(dev, "Restore game mode after exit stylus mode");

			/* check if need restore high report rate */
			if (core_data->ts_dev->board_data.interpolation_ctrl && core_data->set_mode.interpolation) {
				val[0] = NOTIFY_TYPE_SWITCH_REPORT_RATE;
				val[1] = ((core_data->set_mode.report_rate_mode) >> 8) & 0xFF;
				val[2] = (core_data->set_mode.report_rate_mode) & 0xFF;
				put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
				msleep(20);
				ts_info(tdev->dev, "Success to restore %s interpolation mode",
					core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
					(core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_300/360HZ" :
					(core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
					(core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
					(core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
					(core_data->set_mode.report_rate_mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120/130HZ" :
				"Unsupported"))))));
			}
		}
	}

exit:
	mutex_unlock(&core_data->mode_lock);

	return size;
}

static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "Stylus mode = %d.", core_data->set_mode.stylus_mode);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.stylus_mode);
}

static ssize_t goodix_ts_fp_int_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_info(tdev->dev, "Failed to convert value.");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.fp_int_state= mode;
	if (core_data->set_mode.fp_int_state == mode) {
		ts_info(tdev->dev, "The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in sleep state, restore the value when resume");
		ret = size;
		goto exit;
	}

	ret = tdev->hw_ops->set_fp_int_pin(tdev, mode);
	if (!ret)
		core_data->set_mode.fp_int_state = mode;
	msleep(20);
	ts_info(tdev->dev, "Success set fp int to %s", mode ? "high" : "low");

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_fp_int_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "fp_int_state = %d.",
		core_data->set_mode.fp_int_state);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.fp_int_state);
}

static ssize_t goodix_ts_ble_broadcast_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_info(tdev->dev, "Failed to convert value.");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in sleep state, ignore the value");
		ret = size;
		goto exit;
	}

	ret = tdev->hw_ops->set_ble_broadcast(tdev, mode);
	if (!ret)
		ts_info(tdev->dev, "Success %s ble broadcast", mode ? "start" : "stop");

	msleep(20);
	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_hardware_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	u8 hardware_status = 0;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	hardware_status = core_data->open_status;
	ts_info(core_data->ts_dev->dev, "Read touch hardware status = %d", hardware_status);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", hardware_status);
}

static ssize_t goodix_ts_device_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_info(tdev->dev, "Failed to convert value.");
		return -EINVAL;
	}

	if (mode > U8_MAX) {
		ts_info(tdev->dev, "Invalid value %lu, it is out of range (0-255).", mode);
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in sleep state, ignore the value");
		ret = -EAGAIN;
		goto exit;
	}

	ret = tdev->hw_ops->set_device_id(tdev, (u8)mode);
	if (ret) {
		ts_info(tdev->dev, "Failed to send device ID to TP FW %d", ret);
		goto exit;
	}

	ts_info(tdev->dev, "Success send phone device ID to TP FW %ld", mode);
	/*
	* 20ms delay required after sending device ID to touch firmware.
	* This allows the firmware to properly process the command and
	* update its internal state before handling subsequent operations.
	* This timing is based on firmware requirements documented in
	* the touch controller datasheet.
	*/
	msleep(20);
	ret = size;

exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_fw_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct thp_ts_device *tdev;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	ret = tdev->hw_ops->send_cmd(tdev, CMD_FW_MODE, mode);
	msleep(20);

	if (!ret)
		ts_info(tdev->dev, "Set fw to %s mode", mode ? "THP" : "MCU");

	ret = size;
	return ret;
}

static ssize_t goodix_ts_fw_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x", 0x01);
}

static ssize_t goodix_ts_stylus_report_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev,
			"Stylus report rate mode = %d.", core_data->set_mode.stylus_report_rate_mode);
	return scnprintf(buf, PAGE_SIZE, "%d\n", core_data->set_mode.stylus_report_rate_mode);
}

static ssize_t goodix_ts_stylus_report_rate_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long value = 0;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	struct thp_ts_device *tdev;
	u8 val[3];

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev;

	mutex_lock(&core_data->mode_lock);
	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		ts_err(tdev->dev, "Failed to convert value");
		mutex_unlock(&core_data->mode_lock);
		return -EINVAL;
	}

	core_data->get_mode.stylus_report_rate_mode = value;
	if (core_data->set_mode.stylus_report_rate_mode == value) {
		ts_info(tdev->dev, "The value = %lu is same,so not write.", value);
		ret = -EAGAIN;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info(tdev->dev, "The touch is in power off sleep state, restore the value when resume");
		ret = -EAGAIN;
		goto exit;
	}

	val[0] = NOTIFY_TYPE_SET_STYLUSTIP_REPORT_RATE;
	if (value == 1) {
		/* switch stylus tip report rate to high */
		core_data->current_stylus_rate_mode = 1;
	} else {
		/* switch stylus tip report rate to default */
		core_data->current_stylus_rate_mode = 0;
	}
	val[1] = (core_data->rate_configs[core_data->current_stylus_rate_mode].command >> 8) & 0xFF;
	val[2] = (core_data->rate_configs[core_data->current_stylus_rate_mode].command ) & 0xFF;
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

	core_data->set_mode.stylus_report_rate_mode = core_data->get_mode.stylus_report_rate_mode;
	msleep(20);

	ts_info(tdev->dev, "Success switch stylus tip report rate to %dHZ, is on stylus mode? %s",
		core_data->rate_configs[core_data->current_stylus_rate_mode].report_rate,
		core_data->set_mode.stylus_mode? "yes" : "no");
	ret = size;

exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

int goodix_ts_mmi_post_resume(struct goodix_thp_core *core_data) {
	struct device *dev = core_data->ts_dev->dev;
	u8 val[3];
	int ret = 0;

	mutex_lock(&core_data->mode_lock);
	/* All IC status are cleared after reset */
	memset(&core_data->set_mode, 0 , sizeof(core_data->set_mode));
	/* restore data */
	if (core_data->ts_dev->board_data.stylus_mode_ctrl && core_data->get_mode.stylus_mode) {
		/* If BLE enable stylus notify before IC resume done,
		* stylus_mode_store() will not switch to stylus mode,
		* here will do the stylus mode switch
		*/
		ret = goodix_stylus_mode(core_data, core_data->get_mode.stylus_mode);
		if (!ret) {
			core_data->set_mode.stylus_mode = core_data->get_mode.stylus_mode;
			ts_info(dev, "Success to %s stylus mode",
				core_data->get_mode.stylus_mode ? "Enable" : "Disable");
		}
	}

	if (core_data->ts_dev->board_data.interpolation_ctrl && core_data->get_mode.interpolation) {
		val[0] = NOTIFY_TYPE_SWITCH_REPORT_RATE;
		val[1] = ((core_data->get_mode.report_rate_mode) >> 8) & 0xFF;
		val[2] = (core_data->get_mode.report_rate_mode) & 0xFF;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

		core_data->set_mode.interpolation = core_data->get_mode.interpolation;
		core_data->set_mode.report_rate_mode = core_data->get_mode.report_rate_mode;
		msleep(20);

		ts_info(dev, "Success to %s interpolation mode",
			core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
			(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_300/360HZ" :
			(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
			(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
			(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
			(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120/130HZ" :
		"Unsupported"))))));
	}

	if (core_data->ts_dev->board_data.stylus_interpolation_ctrl && core_data->get_mode.stylus_report_rate_mode) {
		val[0] = NOTIFY_TYPE_SET_STYLUSTIP_REPORT_RATE;
		/* switch stylus tip report rate to high */
		core_data->current_stylus_rate_mode = 1;
		val[1] = (core_data->rate_configs[core_data->current_stylus_rate_mode].command >> 8) & 0xFF;
		val[2] = (core_data->rate_configs[core_data->current_stylus_rate_mode].command ) & 0xFF;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

		core_data->set_mode.stylus_report_rate_mode = core_data->get_mode.stylus_report_rate_mode;
		msleep(20);
		ts_info(dev, "Success switch stylus tip report rate to %dHZ, is on stylus mode? %s",
			core_data->rate_configs[core_data->current_stylus_rate_mode].report_rate,
			core_data->set_mode.stylus_mode? "yes" : "no");
	}

	if (core_data->ts_dev->board_data.sample_ctrl && core_data->get_mode.sample) {
		val[0] = NOTIFY_TYPE_GAME_MODE;
		val[1] = core_data->get_mode.sample;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

		core_data->set_mode.sample = core_data->get_mode.sample;
		msleep(20);
		ts_info(dev, "Success to %d sample mode", core_data->get_mode.sample);
	}

	if (core_data->ts_dev->board_data.edge_ctrl) {
		val[0] = NOTIFY_TYPE_ROTATION;
		val[1] = (u8)(core_data->get_mode.edge_mode[0]);
		val[2] = (u8)(core_data->get_mode.edge_mode[1]);
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

		memcpy(core_data->set_mode.edge_mode, core_data->get_mode.edge_mode,
				sizeof(core_data->get_mode.edge_mode));
		msleep(20);
		ts_info(dev, "Success to set edge area = %02x, rotation = %02x",
			core_data->get_mode.edge_mode[1], core_data->get_mode.edge_mode[0]);
	}

	if (core_data->ts_dev->board_data.stowed_mode_ctrl) {
		core_data->set_mode.stowed = 0;
	}

	if (core_data->ts_dev->board_data.pocket_mode_ctrl && core_data->get_mode.pocket_mode) {
		val[0] = NOTIFY_TYPE_POCKET_MODE;
		val[1] = core_data->get_mode.pocket_mode;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		msleep(20);

		core_data->set_mode.pocket_mode = core_data->get_mode.pocket_mode;
		ts_info(dev, "Success to %s pocket mode", core_data->get_mode.pocket_mode ? "Enable" : "Disable");
	}

	if (core_data->get_mode.charger_mode) {
		val[0] = NOTIFY_TYPE_CHARGE;
		val[1] = core_data->get_mode.charger_mode;
		put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));

		core_data->set_mode.charger_mode = core_data->get_mode.charger_mode;
		msleep(20);
		ts_info(core_data->ts_dev->dev, "Success to %s charger mode",
			core_data->get_mode.charger_mode ? "enable" : "disable");
	}

	mutex_unlock(&core_data->mode_lock);

	return 0;
}

static int goodix_berlin_gesture_setup(struct goodix_thp_core *core_data)
{
	struct device *dev = core_data->ts_dev->dev;
	int ret = 0;
	unsigned char gesture_type = 0;
	u8 val[3];

	val[0] = NOTIFY_TYPE_GESTURE;
	val[1] = 0x0;
	val[2] = 0x0;
	if (core_data->imports && core_data->imports->get_gesture_type) {
		ret = core_data->imports->get_gesture_type(core_data->ts_dev->dev, &gesture_type);
		ts_info(dev, "Provisioned gestures 0x%02x; rc = %d", gesture_type, ret);
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

	ts_info(dev, "Send enable gesture mode 0x%x 0x%x", val[1], val[2]);
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	msleep(20);

	return 0;
}

static int goodix_berlin_gesture_clean(struct goodix_thp_core *core_data)
{
	u8 val[3];

	val[0] = NOTIFY_TYPE_GESTURE;
	val[1] = 0x0;
	val[2] = 0x0;

	ts_info(core_data->ts_dev->dev, "Send cmd to clean gesture mode");
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	msleep(20);

	return 0;
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;
	struct device *tdev;
	u8 val[2];

	GET_GOODIX_DATA(dev);
	tdev = core_data->ts_dev->dev;

	val[0] = NOTIFY_TYPE_SCREEN;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		goodix_berlin_gesture_setup(core_data);
		val[1] = 0;
		msleep(16);
		break;
	case TS_MMI_PM_DEEPSLEEP:
		goodix_berlin_gesture_clean(core_data);
		val[1] = 0;
		break;
	case TS_MMI_PM_ACTIVE:
		val[1] = 1;
		break;
	default:
		ts_err(tdev, "Invalid power state parameter %d.", to);
		return -EINVAL;
	}

	if (val[1]) {
		ts_info(tdev, "Send screen on cmd");
	}
	else {
		ts_info(tdev, "Send screen off cmd");
	}
	put_frame_list(core_data, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	msleep(20);

	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev;
	struct goodix_thp_core *core_data;

	GET_GOODIX_DATA(dev);

	ts_info(core_data->ts_dev->dev, "Suspend start");

	if (core_data->ts_dev->board_data.stylus_mode_ctrl && core_data->set_mode.stylus_mode) {
		mutex_lock(&core_data->mode_lock);
		ret = goodix_stylus_mode(core_data, 0x00);
		if (!ret) {
			ts_info(core_data->ts_dev->dev, "Success to exit stylus mode");
			core_data->set_mode.stylus_mode = 0x00;
			/* clear get stylus mode after suspend,
			* because after screen on we will receive BLE setting again
			*/
			core_data->get_mode.stylus_mode = 0x00;
		}
		mutex_unlock(&core_data->mode_lock);
	}

	if (core_data->ts_dev->board_data.stylus_interpolation_ctrl && core_data->set_mode.stylus_report_rate_mode) {
		/* before suspend, we need clear current stylus rate mode,
		* because stylus report rate will be cleaned after touch IC reset
		*/
		mutex_lock(&core_data->mode_lock);
		core_data->current_stylus_rate_mode = 0;
		mutex_unlock(&core_data->mode_lock);
	}

	return 0;
}

static struct ts_mmi_methods goodix_ts_mmi_methods = {
	.get_vendor = goodix_ts_mmi_methods_get_vendor,
	.get_productinfo = goodix_ts_mmi_methods_get_productinfo,
	.charger_mode = goodix_ts_mmi_charger_mode,
	/* vendor specific attribute group */
	.extend_attribute_group = goodix_ts_mmi_extend_attribute_group,
	.panel_state = goodix_ts_mmi_panel_state,
	.pre_suspend = goodix_ts_mmi_pre_suspend,
};

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret, i;
	struct thp_ts_device *ts_dev;
	struct goodix_thp_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_info(NULL, "Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;

	if (core_data->ts_dev->board_data.interpolation_ctrl) {
		if (parse_report_rate_config(core_data->ts_dev->dev)) {
			ts_err(ts_dev->dev, "Failed to parse rate config");
		} else {
			ts_info(ts_dev->dev, "rate_config_count = %d", report_rate_config_info.rate_config_count);
			ts_info(ts_dev->dev, "refresh_rate_ctrl = %d", report_rate_config_info.refresh_rate_ctrl);
			ts_info(ts_dev->dev, "interpolation_ctrl = %d", report_rate_config_info.interpolation_ctrl);
			for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
				ts_info(ts_dev->dev, "interpolation_flag = %d",
						report_rate_config_info.report_rate_info[i].interpolation_flag);
				ts_info(ts_dev->dev, "refresh_rate[0] = %d, refresh_rate[1] = %d",
						report_rate_config_info.report_rate_info[i].refresh_rate[0],
						report_rate_config_info.report_rate_info[i].refresh_rate[1]);
				ts_info(ts_dev->dev, "report_rate = %d", report_rate_config_info.report_rate_info[i].report_rate);
				ts_info(ts_dev->dev, "command = 0x%02x\n", report_rate_config_info.report_rate_info[i].command);
			}
		}
	}

	mutex_init(&core_data->mode_lock);
	ret = ts_mmi_dev_register(ts_dev->dev, &goodix_ts_mmi_methods);
	if (ret) {
		ts_err(ts_dev->dev, "Failed to register ts mmi");
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
		ts_info(NULL, "Failed to get driver data");
	else {
		mutex_destroy(&core_data->mode_lock);
		ts_mmi_dev_unregister(core_data->ts_dev->dev);
	}
}
