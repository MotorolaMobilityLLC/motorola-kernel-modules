/*
 * Copyright (C) 2023 Motorola Mobility LLC
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

#include "goodix_ts_mmi.h"
#include <linux/delay.h>
#include <linux/input/mt.h>

#define MAX_ATTRS_ENTRIES 10

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

#ifdef CONFIG_GTP_LAST_TIME
static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf);
	static DEVICE_ATTR(timestamp, S_IRUGO, goodix_ts_timestamp_show, NULL);
#endif

static ssize_t goodix_ts_stowed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stowed_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static DEVICE_ATTR(stowed, (S_IWUSR | S_IWGRP | S_IRUGO),
		goodix_ts_stowed_show, goodix_ts_stowed_store);

static ssize_t goodix_ts_pocket_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_pocket_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(pocket_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_pocket_mode_show, goodix_ts_pocket_mode_store);

static int goodix_ts_send_cmd(struct goodix_ts_core *core_data,
		u8 cmd, u8 len, u8 subCmd, u8 subCmd2);

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "gdx");
}

static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_ts_board_data *ts_bdata;
	char* ic_info;

	GET_GOODIX_DATA(dev);

	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}

	ic_info = strstr(ts_bdata->ic_name, ",");
	ic_info++;

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ic_info);
}

#define TOUCH_CFG_VERSION_ADDR    0x10076
static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	u32 cfg_id;

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->read(core_data, TOUCH_CFG_VERSION_ADDR,
			(u8*)&cfg_id,  sizeof(cfg_id));
	if (ret) {
		ts_info("failed get fw version data, %d", ret);
		return -EINVAL;
	}

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", le32_to_cpu(cfg_id));
}

#define TOUCH_FW_VERSION_ADDR    0x1007E
/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	u8 fw_id[4] = {0};

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->read(core_data, TOUCH_FW_VERSION_ADDR,
			fw_id, sizeof(fw_id));
	if (ret) {
		ts_info("failed get fw version data, %d", ret);
		return -EINVAL;
	}

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x%02x%02x%02x",
			fw_id[0], fw_id[1], fw_id[2], fw_id[3]);
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

	TO_INT(idata) = core_data->update_ctrl.status;
	return 0;
}

/* reset chip
 * type:can control software reset and hardware reset,
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

static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->hw_ops->irq_enable)
		ret = core_data->hw_ops->irq_enable(core_data, !(!state));

	return ret;
}

static int goodix_ts_mmi_methods_power(struct device *dev, int on) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	core_data->ts_mmi_power_state = on;
	schedule_delayed_work(&core_data->work, 0);
	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_esd_off(core_data);

	return 0;
}

void goodix_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

static int goodix_ts_mmi_wait_for_ready(struct device *dev)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_ts_hw_ops *hw_ops;
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ts_cmd ts_cmd;
	struct goodix_ic_info_misc *misc;
	int ret = 0;
	int retry;

	GET_GOODIX_DATA(dev);

	hw_ops = core_data->hw_ops;
	misc= &core_data->ic_info.misc;

	if (misc->cmd_addr == 0x0000) {
		ts_err("invalid cmd addr:0x0000, skip cmd");
		return -EINVAL;
	}

	retry = GOODIX_RETRY_5;
	while (retry--) {
		ts_cmd.cmd = MMI_GOODIX_CMD_COORD;
		ts_cmd.len = 4;
		ret = core_data->hw_ops->send_cmd(core_data, &ts_cmd);
		if (ret < 0)
			return ret;

		/* check command result */
		ret = hw_ops->read(core_data, misc->cmd_addr,
			cmd_ack.buf, sizeof(cmd_ack));
		if (ret < 0) {
			ts_err("failed read command ack, %d", ret);
			return -EINVAL;
		}
		ts_info("cmd ack data %*ph",
			 (int)sizeof(cmd_ack), cmd_ack.buf);

		if ((cmd_ack.buf[0] == MMI_CONFIG_CMD_STATUS_PASS) &&
			(cmd_ack.buf[1] == MMI_CMD_ACK_OK))
			break;

		goodix_ts_delay(10);
	}

	if (retry < 0) {
		ts_err("wait for IC ready timeout!");
		return -EINVAL;
	}

	return 0;
}

static unsigned int clear_bit_in_pos(unsigned int val, int bit_pos)
{
	return val &= ~(1 << bit_pos);
}

static int goodix_berlin_gesture_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	unsigned int gesture_cmd = 0xFFFF;
	int ret = 0;
	unsigned char gesture_type = 0;
	unsigned int (*mod_func)(unsigned int, int) = clear_bit_in_pos;

	if (core_data->imports && core_data->imports->get_gesture_type) {
		ret = core_data->imports->get_gesture_type(core_data->bus->dev, &gesture_type);
		ts_info("Provisioned gestures 0x%02x; rc = %d\n", gesture_type, ret);
	}
	core_data->gesture_type = gesture_type;

	if (gesture_type & TS_MMI_GESTURE_ZERO) {
		gesture_cmd = mod_func(gesture_cmd, 13);
		ts_info("enable zero gesture mode cmd 0x%04x\n", gesture_cmd);
	}
	if (gesture_type & TS_MMI_GESTURE_SINGLE) {
		gesture_cmd = mod_func(gesture_cmd, 12);
		ts_info("enable single gesture mode cmd 0x%04x\n", gesture_cmd);
	}
	if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
		gesture_cmd = mod_func(gesture_cmd, 7);
		ts_info("enable double gesture mode cmd 0x%04x\n", gesture_cmd);
	}

	hw_ops->gesture(core_data, gesture_cmd);
	ts_info("Send enable gesture mode 0x%x\n", gesture_cmd);

	return 0;
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
		hw_ops->irq_enable(core_data, false);
		goodix_berlin_gesture_setup(core_data);
		msleep(16);
		hw_ops->irq_enable(core_data, true);
		enable_irq_wake(core_data->irq);
		core_data->gesture_enabled = true;
		break;
	case TS_MMI_PM_DEEPSLEEP:
		core_data->gesture_enabled = false;
		break;
	case TS_MMI_PM_ACTIVE:
		if (hw_ops->resume)
			hw_ops->resume(core_data);
		if (core_data->gesture_enabled) {
			core_data->gesture_enabled = false;
			hw_ops->irq_enable(core_data, true);
		}
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	return 0;
}

static int goodix_ts_mmi_post_suspend(struct device *dev) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;
	int ret = 0;

	GET_GOODIX_DATA(dev);

	goodix_ts_release_connects(core_data);

	atomic_set(&core_data->post_suspended, 1);
	mutex_lock(&core_data->mode_lock);
	if (core_data->board_data.stowed_mode_ctrl && core_data->get_mode.stowed) {
		ret = goodix_ts_send_cmd(core_data, ENTER_STOWED_MODE_CMD, 5, core_data->get_mode.stowed, 0x00);
		if (ret < 0) {
			ts_err("Failed to set stowed mode %d\n", core_data->get_mode.stowed);
		} else {
			core_data->set_mode.stowed = core_data->get_mode.stowed;
			ts_info("Enable stowed mode %d success.\n", core_data->set_mode.stowed);
		}
	}
	mutex_unlock(&core_data->mode_lock);

	ts_info("Suspend end");
	return 0;
}

static int goodix_ts_mmi_pre_resume(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	ts_info("Resume start");
	GET_GOODIX_DATA(dev);

	atomic_set(&core_data->suspended, 0);
	atomic_set(&core_data->post_suspended, 0);
	if (core_data->gesture_enabled) {
		core_data->hw_ops->irq_enable(core_data, false);
		disable_irq_wake(core_data->irq);
	}

	return 0;
}

static int goodix_ts_mmi_post_resume(struct device *dev) {
	int ret = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	/* open esd */
	goodix_ts_esd_on(core_data);

	mutex_lock(&core_data->mode_lock);
	/* All IC status are cleared after reset */
	memset(&core_data->set_mode, 0 , sizeof(core_data->set_mode));
	/* TODO: restore data */
	if (core_data->board_data.pocket_mode_ctrl && core_data->get_mode.pocket_mode) {
		ret = goodix_ts_send_cmd(core_data, ENTER_POCKET_MODE_CMD, 5,
			core_data->get_mode.pocket_mode , 0x00);
		if (!ret) {
			core_data->set_mode.pocket_mode = core_data->get_mode.pocket_mode;
			ts_info("Success to %s pocket mode", core_data->get_mode.pocket_mode ? "Enable" : "Disable");
		}
	}
	mutex_unlock(&core_data->mode_lock);

	ts_info("Resume end");
	return 0;
}

static int goodix_ts_firmware_update(struct device *dev, char *fwname) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	int update_flag = UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST | UPDATE_MODE_FORCE;

	GET_GOODIX_DATA(dev);

	ts_info("HW request update fw, %s", fwname);
	/* set firmware image name */
	if (core_data->set_fw_name)
		core_data->set_fw_name(core_data, fwname);

	/* do upgrade */
	ret = goodix_do_fw_update(core_data, update_flag);
	if (ret)
		ts_err("failed do fw update");

	/* read ic info */
	ret = core_data->hw_ops->get_ic_info(core_data, &core_data->ic_info);
	if (ret < 0) {
		ts_err("failed to get ic info");
	}
	print_ic_info(&core_data->ic_info);

	/* the recomend way to update ic config is throuth ISP,
	 * if not we will send config with interactive mode
	 */
	goodix_send_ic_config(core_data, CONFIG_TYPE_NORMAL);
	return 0;
}

static int goodix_ts_send_cmd(struct goodix_ts_core *core_data,
		u8 cmd, u8 len, u8 subCmd, u8 subCmd2)
{
	int ret = 0;
	struct goodix_ts_cmd ts_cmd;

	ts_cmd.cmd = cmd;
	ts_cmd.len = len;
	ts_cmd.data[0] = subCmd;
	ts_cmd.data[1] = subCmd2;

	ret = core_data->hw_ops->send_cmd(core_data, &ts_cmd);
	return ret;
}

static int goodix_ts_mmi_charger_mode(struct device *dev, int mode)
{
	int ret = 0;
	int timeout = 50;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	/* 5000ms timeout */
	while (core_data->init_stage < CORE_INIT_STAGE2 && timeout--)
		msleep(100);

	mutex_lock(&core_data->mode_lock);
	ret = goodix_ts_send_cmd(core_data, CHARGER_MODE_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("Failed to set charger mode\n");
	}
	msleep(20);
	ts_info("Success to %s charger mode\n", mode ? "Enable" : "Disable");
	mutex_unlock(&core_data->mode_lock);

	return 0;
}

#ifdef CONFIG_GTP_LAST_TIME
static ssize_t goodix_ts_timestamp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	ktime_t last_ktime;
	struct timespec64 last_ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	mutex_lock(&core_data->mode_lock);
	last_ktime = core_data->last_event_time;
	core_data->last_event_time = 0;
	mutex_unlock(&core_data->mode_lock);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
#endif

static ssize_t goodix_ts_stowed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		ts_err("Failed to convert value.");
		return -EINVAL;
	}

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.stowed = mode;
	if (core_data->set_mode.stowed == mode) {
		ts_debug("The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if ((atomic_read(&core_data->post_suspended) == 1) && (core_data->power_on == 1)) {
		ret = goodix_ts_send_cmd(core_data, ENTER_STOWED_MODE_CMD, 5,
			core_data->get_mode.stowed, 0x00);
		if (ret < 0) {
			ts_err("Failed to set stowed mode %lu\n", mode);
			goto exit;
		}
	} else {
		ts_info("Skip stowed mode setting post_suspended:%d, power_on:%d.\n",
			atomic_read(&core_data->post_suspended), core_data->power_on);
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
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("Stowed state = %d.\n", core_data->set_mode.stowed);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.stowed);
}

static ssize_t goodix_ts_pocket_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("Pocket mode state = %d.\n", core_data->set_mode.pocket_mode);
	return scnprintf(buf, PAGE_SIZE, "%d\n", core_data->set_mode.pocket_mode);
}

static ssize_t goodix_ts_pocket_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long value = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	mutex_lock(&core_data->mode_lock);
	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		ts_err("pocket_mode: Failed to convert value\n");
		mutex_unlock(&core_data->mode_lock);
		return -EINVAL;
	}
	switch (value) {
		case 0x10:
		case 0x20:
			ts_info("touch pocket mode disable\n");
			core_data->get_mode.pocket_mode = 0;
			break;
		case 0x11:
		case 0x21:
			ts_info("touch pocket mode enable\n");
			core_data->get_mode.pocket_mode = 1;
			break;
		default:
			ts_info("unsupport pocket mode type, value = %lu\n", value);
			mutex_unlock(&core_data->mode_lock);
			return -EINVAL;
	}

	if (core_data->set_mode.pocket_mode == core_data->get_mode.pocket_mode) {
		ts_info("The value = %d is same, so not to write", core_data->get_mode.pocket_mode);
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_info("The touch is in sleep state, restore the value when resume\n");
		goto exit;
	}

	ret = goodix_ts_send_cmd(core_data, ENTER_POCKET_MODE_CMD, 5,
		core_data->get_mode.pocket_mode , 0x00);
	if (ret < 0) {
		ts_err("failed to send pocket mode cmd");
		goto exit;
	}

	core_data->set_mode.pocket_mode = core_data->get_mode.pocket_mode;
	msleep(20);

	ts_info("Success to %s pocket mode", core_data->get_mode.pocket_mode ? "Enable" : "Disable");
exit:
	mutex_unlock(&core_data->mode_lock);
	return size;
}

static int goodix_ts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

#ifdef CONFIG_GTP_LAST_TIME
	ADD_ATTR(timestamp);
#endif

	if (core_data->board_data.stowed_mode_ctrl)
		ADD_ATTR(stowed);

	if (core_data->board_data.pocket_mode_ctrl)
		ADD_ATTR(pocket_mode);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

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
	.charger_mode = goodix_ts_mmi_charger_mode,
	/* Firmware */
	.firmware_update = goodix_ts_firmware_update,
	/* vendor specific attribute group */
	.extend_attribute_group = goodix_ts_mmi_extend_attribute_group,
	/* PM callback */
	.wait_for_ready = goodix_ts_mmi_wait_for_ready,
	.pre_suspend = goodix_ts_mmi_pre_suspend,
	.panel_state = goodix_ts_mmi_panel_state,
	.post_suspend = goodix_ts_mmi_post_suspend,
	.pre_resume = goodix_ts_mmi_pre_resume,
	.post_resume = goodix_ts_mmi_post_resume,
};

static void ts_mmi_worker_func(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct goodix_ts_core *core_data =
		container_of(dw, struct goodix_ts_core, work);

	if (core_data->ts_mmi_power_state == TS_MMI_POWER_ON)
	{
		goodix_ts_power_on(core_data);
	}
	else if (core_data->ts_mmi_power_state == TS_MMI_POWER_OFF)
	{
		goodix_ts_power_off(core_data);
	} else {
		ts_err("Invalid power parameter %d.\n", core_data->ts_mmi_power_state);
	}
}

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;

	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	INIT_DELAYED_WORK(&core_data->work, ts_mmi_worker_func);
	mutex_init(&core_data->mode_lock);
	ret = ts_mmi_dev_register(core_data->bus->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		mutex_destroy(&core_data->mode_lock);
		return ret;
	}

	core_data->imports = &goodix_ts_mmi_methods.exports;

#if defined(CONFIG_GTP_LIMIT_USE_SUPPLIER)
	if (core_data->imports && core_data->imports->get_supplier) {
		ret = core_data->imports->get_supplier(core_data->bus->dev, &core_data->supplier);
	}
#endif

	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;

	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	mutex_destroy(&core_data->mode_lock);
	ts_mmi_dev_unregister(core_data->bus->dev);
}
