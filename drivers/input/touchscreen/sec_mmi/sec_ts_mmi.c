/*
 * Copyright (C) 2018 Motorola Mobility LLC
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

#define DEBUG 511

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>

#include "sec_ts.h"
#include "sec_mmi.h"

/* MMI specific sysfs entries and API */
static ssize_t sec_mmi_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_forcereflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_erase_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_hw_irqstat_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_panel_supplier_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(forcereflash, (S_IWUSR | S_IWGRP), NULL, sec_mmi_forcereflash_store);
static DEVICE_ATTR(doreflash, (S_IWUSR | S_IWGRP), NULL, sec_mmi_doreflash_store);
static DEVICE_ATTR(drv_irq, (S_IWUSR | S_IWGRP | S_IRUGO), sec_mmi_drv_irq_show, sec_mmi_drv_irq_store);
static DEVICE_ATTR(hw_irqstat, S_IRUGO, sec_mmi_hw_irqstat_show, NULL);
static DEVICE_ATTR(ic_ver, S_IRUGO, sec_mmi_ic_ver_show, NULL);
static DEVICE_ATTR(productinfo, S_IRUGO, sec_mmi_productinfo_show, NULL);
static DEVICE_ATTR(buildid, S_IRUGO, sec_mmi_buildid_show, NULL);
static DEVICE_ATTR(flashprog, S_IRUGO, sec_mmi_flashprog_show, NULL);
static DEVICE_ATTR(poweron, S_IRUGO, sec_mmi_poweron_show, NULL);
static DEVICE_ATTR(reset, (S_IWUSR | S_IWGRP), NULL, sec_mmi_reset_store);
static DEVICE_ATTR(erase_all, (S_IWUSR | S_IWGRP), NULL, sec_mmi_erase_store);
static DEVICE_ATTR(panel_supplier, S_IRUGO, sec_mmi_panel_supplier_show, NULL);

static struct attribute *mmi_attributes[] = {
	&dev_attr_forcereflash.attr,
	&dev_attr_doreflash.attr,
	&dev_attr_drv_irq.attr,
	&dev_attr_hw_irqstat.attr,
	&dev_attr_ic_ver.attr,
	&dev_attr_productinfo.attr,
	&dev_attr_buildid.attr,
	&dev_attr_flashprog.attr,
	&dev_attr_poweron.attr,
	&dev_attr_reset.attr,
	&dev_attr_erase_all.attr,
	&dev_attr_panel_supplier.attr,
	NULL,
};

static struct attribute_group mmi_attr_group = {
	.attrs = mmi_attributes,
};

/* Attribute: path (RO) */
static ssize_t path_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sec_mmi_data *data = dev_get_drvdata(dev);
	ssize_t blen;
	const char *path;

	if (!data) {
		dev_err(DEV_MMI, "data pointer is NULL\n");
		return (ssize_t)0;
	}
	path = kobject_get_path(&data->i2c_client->dev.kobj, GFP_KERNEL);
	blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
	kfree(path);
	return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "samsung");
}

static struct device_attribute class_attributes[] = {
	__ATTR_RO(path),
	__ATTR_RO(vendor),
	__ATTR_NULL
};

static int sec_mmi_touchscreen_class(
	struct sec_mmi_data *data, bool create)
{
	struct device_attribute *attrs = class_attributes;
	bool unreg_class = false;
	int i, error = 0;
	char buffer[128];
	struct sec_ts_data *ts = data->ts_ptr;
	ssize_t namelen;
	const char *class_fname = "primary";
	static char *input_dev_name;
	static struct class *touchscreen_class;
	static int minor;

	if (create) {
		minor = input_get_new_minor(data->i2c_client->addr,
						1, false);
		if (minor < 0)
			minor = input_get_new_minor(TSDEV_MINOR_BASE,
					TSDEV_MINOR_MAX, true);
		dev_dbg(DEV_MMI, "assigned minor %d\n", minor);

		touchscreen_class = get_touchscreen_class_ptr();
		dev_dbg(DEV_MMI, "class ptr %p\n", touchscreen_class);
		if (!touchscreen_class) {
			touchscreen_class = class_create(THIS_MODULE, "touchscreen");
			if (IS_ERR(touchscreen_class)) {
				error = PTR_ERR(touchscreen_class);
				touchscreen_class = NULL;
				return error;
			}
			set_touchscreen_class_ptr(touchscreen_class);
			unreg_class = true;
		}

		if (data->class_entry_name)
			class_fname = data->class_entry_name;

		dev_dbg(DEV_MMI, "class entry name %s\n", class_fname);
		data->ts_class_dev = device_create(touchscreen_class,
				NULL, MKDEV(INPUT_MAJOR, minor), data, class_fname);
		if (IS_ERR(data->ts_class_dev)) {
			error = PTR_ERR(data->ts_class_dev);
			data->ts_class_dev = NULL;
			return error;
		}

		dev_set_drvdata(data->ts_class_dev, data);

		/* construct input device name */
		namelen = scnprintf(buffer, sizeof(buffer), "samsung_mmi.%s", class_fname);
		if (namelen) {
			input_dev_name = kstrdup(buffer, GFP_KERNEL);
			ts->input_dev->name = input_dev_name;
		}

		for (i = 0; attrs[i].attr.name != NULL; ++i) {
			error = device_create_file(data->ts_class_dev, &attrs[i]);
			if (error)
				break;
		}

		if (error)
			goto device_destroy;
	} else {
		if (!touchscreen_class || !data->ts_class_dev)
			return -ENODEV;

		dev_set_drvdata(data->ts_class_dev, NULL);

		for (i = 0; attrs[i].attr.name != NULL; ++i)
			device_remove_file(data->ts_class_dev, &attrs[i]);

		device_unregister(data->ts_class_dev);
		data->ts_class_dev = NULL;
	}

	return 0;

device_destroy:
	for (--i; i >= 0; --i)
		device_remove_file(data->ts_class_dev, &attrs[i]);
	device_destroy(touchscreen_class, MKDEV(INPUT_MAJOR, minor));
	data->ts_class_dev = NULL;
	if (unreg_class)
		class_unregister(touchscreen_class);
	touchscreen_class = NULL;
	dev_err(DEV_MMI, "%s: error creating touchscreen class attrs\n", __func__);

	return -ENODEV;
}

static int sec_mmi_dt(struct sec_mmi_data *data)
{
	struct device *dev = &data->i2c_client->dev;
	struct device_node *np = dev->of_node;

	if (!of_property_read_string(np, "sec,class-entry-name", &data->class_entry_name))
		dev_info(DEV_MMI, "%s: class-entry-name property %s\n",
				__func__, data->class_entry_name);

	if (!of_property_read_string(np, "sec,bound-display", &data->bound_display))
		dev_info(DEV_MMI, "%s: bound-display property %s\n",
				__func__, data->bound_display);

	if (!of_property_read_u32(np, "sec,control-dsi", &data->ctrl_dsi))
		dev_info(DEV_MMI, "%s: ctrl-dsi property %d\n",
				__func__, data->ctrl_dsi);

	np = of_find_node_by_name(NULL, "chosen");
	if (np) {
		const char *supplier;
		char *s, *d;

		of_property_read_string(np, "mmi,panel_name", (const char **)&supplier);
		/* skip dsi_ part */
		s = (char *)supplier + 4;
		d = data->panel_supplier;
		while (*s != '_') *d++ = *s++;
		dev_info(DEV_MMI, "%s: panel-supplier %s\n",
				__func__, data->panel_supplier);

		of_node_put(np);
	}

	return 0;
}

static inline void bdc2ui(unsigned int *dest, unsigned char *src, size_t size)
{
	int i, shift = 0;
	*dest = 0;
	for (i = 0; i < size; i++, shift += 8)
		*dest += src[i] << (24 - shift);
}

static void sec_mmi_fw_read_id(struct sec_mmi_data *data)
{
	struct sec_ts_data *ts = data->ts_ptr;
	unsigned char buffer[8];
	int ret;

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_FW_VERSION, buffer, sizeof(buffer));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to read fw version (%d)\n",
				__func__, ret);
	} else {
		dev_dbg(DEV_MMI, "%s: FW VERSION: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	memset(buffer, 0, sizeof(buffer));
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, buffer, sizeof(buffer));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to read fw version (%d)\n",
				__func__, ret);
	} else {
		bdc2ui(&data->config_id, buffer+4, sizeof(data->config_id));
		dev_dbg(DEV_MMI, "%s: CONFIG VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	memset(buffer, 0, sizeof(buffer));
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, buffer, sizeof(buffer));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to read fw version (%d)\n",
				__func__, ret);
	} else {
		if (buffer[0] == 0x17)
			ts->device_id[3] = 0x7C;
		bdc2ui(&data->build_id, buffer, sizeof(data->build_id));
		dev_dbg(DEV_MMI, "%s: IMAGE VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	snprintf(data->product_id, sizeof(data->product_id), "%c%c%c%02x",
				tolower(ts->device_id[0]), tolower(ts->device_id[1]),
				ts->device_id[2], ts->device_id[3]);
}

static void sec_mmi_ic_reset(struct sec_mmi_data *data, int mode)
{
	struct sec_ts_data *ts = data->ts_ptr;

	if (!gpio_is_valid(ts->plat_data->rst_gpio) ||
		!gpio_get_value(ts->plat_data->rst_gpio))
		return;

	mutex_lock(&ts->modechange);
	__pm_stay_awake(&ts->wakelock);

	if (mode) {
		gpio_set_value(ts->plat_data->rst_gpio, 0);
		usleep_range(10, 10);
		gpio_set_value(ts->plat_data->rst_gpio, 1);
	} else {
		int ret;

		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
		if (ret < 0)
			dev_err(DEV_MMI, "%s: error sending sw reset cmd\n", __func__);
	}

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);

	mutex_unlock(&ts->modechange);
	__pm_relax(&ts->wakelock);

	dev_dbg(DEV_MMI, "%s: hw reset done\n", __func__);
}

static void sec_mmi_wait4idle(struct sec_mmi_data *data)
{
	struct sec_ts_data *ts = data->ts_ptr;
	unsigned long start_wait_jiffies = jiffies;

	do {
		if (!ts->wakelock.active)
			break;
		usleep_range(1000, 1000);
	} while (1);

	if ((jiffies - start_wait_jiffies))
		dev_info(DEV_MMI, "%s: entering suspend delayed for %ums\n",
			__func__, jiffies_to_msecs(jiffies - start_wait_jiffies));
}

static void sec_mmi_work(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct sec_mmi_data *data =
		container_of(dw, struct sec_mmi_data, detection_work);
	struct sec_ts_data *ts = data->ts_ptr;
	int probe_status;
	bool panel_ready = true;
	char *pname = NULL;

	/* DRM panel status */
	panel_ready = dsi_display_is_panel_enable(data->ctrl_dsi,
									&probe_status, &pname);
	dev_dbg(DEV_MMI, "%s: drm: probe=%d, enable=%d, panel'%s'\n",
			__func__, probe_status, panel_ready, pname);
	/* check if bound panel is not present */
	if (pname && strstr(pname, "dummy")) {
		dev_info(DEV_MMI, "%s: dummy panel detected\n", __func__);
		//rmi4_data->drm_state = DRM_ST_TERM;
	} else if (panel_ready) {
		dev_dbg(DEV_MMI, "%s: panel ready\n", __func__);
		//rmi4_data->drm_state = DRM_ST_READY;
	}

	if (ts->fw_invalid == false) {
		sec_mmi_fw_read_id(data);
		sec_ts_integrity_check(ts);

		sec_ts_sense_on(ts);
		dev_dbg(DEV_MMI, "%s: sensing turned on\n", __func__);
	}
}

static void sec_mmi_queued_resume(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct sec_mmi_data *data =
		container_of(dw, struct sec_mmi_data, resume_work);
	struct sec_ts_data *ts = data->ts_ptr;
	unsigned char buffer = 0;
	int ret;

	if (atomic_cmpxchg(&data->touch_stopped, 1, 0) == 0)
		return;

	sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS,
					&buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(DEV_MMI,
				"%s: failed to read boot status (%d)\n",
				__func__, ret);

	ts->fw_invalid = buffer == SEC_TS_STATUS_BOOT_MODE;

	dev_info(DEV_MMI, "%s: touch resumed\n", __func__);
}

static int inline sec_mmi_display_on(struct sec_mmi_data *data)
{
	/* schedule_delayed_work returns true if work has been scheduled */
	/* and false otherwise, thus return 0 on success to comply POSIX */
	return schedule_delayed_work(&data->resume_work, 0) == false;
}

static int inline sec_mmi_display_off(struct sec_mmi_data *data)
{
	struct sec_ts_data *ts = data->ts_ptr;

	if (atomic_cmpxchg(&data->touch_stopped, 0, 1) == 1)
		return 0;
	/* complete critical work */
	sec_mmi_wait4idle(data);
	sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	cancel_delayed_work_sync(&data->resume_work);

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	dev_info(DEV_MMI, "%s: touch suspended\n", __func__);

	return 0;
}

static int sec_mmi_panel_cb(struct notifier_block *nb,
		unsigned long event, void *evd)
{
	struct msm_drm_notifier *evdata = evd;
	struct sec_mmi_data *data =
		container_of(nb, struct sec_mmi_data, panel_nb);

	if (!evdata || (evdata->id != data->ctrl_dsi)) {
		dev_dbg(DEV_MMI, "%s: drm notification: id(%d) != ctrl_dsi(%d)\n",
				__func__, evdata->id, data->ctrl_dsi);
		return 0;
	}

	if ((event == MSM_DRM_EARLY_EVENT_BLANK || event == MSM_DRM_EVENT_BLANK) &&
		 evdata && evdata->data && data) {
		int *blank = evdata->data;

		dev_dbg(DEV_MMI, "%s: drm notification: event = %lu blank = %d\n",
				__func__, event, *blank);
		/* entering suspend upon early blank event */
		/* to ensure shared power supply is still on */
		/* for in-cell design touch solutions */
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			if (*blank != MSM_DRM_BLANK_POWERDOWN)
				return 0;
			sec_mmi_display_off(data);
		} else if (*blank == MSM_DRM_BLANK_UNBLANK) {
			sec_mmi_display_on(data);
		}
	}

	return 0;
}

static int sec_mmi_register_notifier(
	struct sec_mmi_data *data, bool enable)
{
	int rc;

	if (enable) {
		data->panel_nb.notifier_call = sec_mmi_panel_cb;
		rc = msm_drm_register_client(&data->panel_nb);
	} else
		rc = msm_drm_unregister_client(&data->panel_nb);

	if (!rc)
		dev_dbg(DEV_MMI, "%s: %sregistered drm notifier\n",
				__func__, enable ? "" : "un");
	else
		dev_err(DEV_MMI, "%s: unable to %sregister drm notifier\n",
				__func__, enable ? "" : "un");

	return rc;
}

static ssize_t sec_mmi_panel_supplier_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	return scnprintf(buf, PAGE_SIZE, "%s\n", data->panel_supplier);
}

static ssize_t sec_mmi_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			ts->power_status != SEC_TS_STATE_POWER_OFF);
}

static ssize_t sec_mmi_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	return scnprintf(buf, PAGE_SIZE, "%s\n", data->product_id);
}

static ssize_t sec_mmi_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	return scnprintf(buf, PAGE_SIZE, "%08x-%08x\n",
			data->build_id, data->config_id);
}

static ssize_t sec_mmi_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ts->fw_invalid ? 1 : 0);
}

static ssize_t sec_mmi_hw_irqstat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	switch (gpio_get_value(ts->plat_data->irq_gpio)) {
		case 0:
			return scnprintf(buf, PAGE_SIZE, "Low\n");
		case 1:
			return scnprintf(buf, PAGE_SIZE, "High\n");
		default:
			dev_err(DEV_TS, "%s: Failed to get irq gpio %d state\n",
					__func__, ts->plat_data->irq_gpio);
			return scnprintf(buf, PAGE_SIZE, "Unknown\n");
	}
}

static ssize_t sec_mmi_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			ts->irq_enabled ? "ENABLED" : "DISABLED");
}

static ssize_t sec_mmi_erase_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &value);
	if (err < 0) {
		dev_err(DEV_TS, "%s: Failed to convert value\n", __func__);
		return -EINVAL;
	}

	if (value != 1)
		return -EINVAL;

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	__pm_stay_awake(&ts->wakelock);

	ts->fw_invalid = true;

	mutex_unlock(&ts->modechange);
	__pm_relax(&ts->wakelock);

	return size;
}

static ssize_t sec_mmi_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int err = 0;
	err = kstrtoul(buf, 10, &value);
	if (err < 0) {
		dev_err(DEV_TS, "%s: Failed to convert value\n", __func__);
		return -EINVAL;
	}
	switch (value) {
	case 0: /* Disable irq */
		sec_ts_irq_enable(ts, false);
			break;
	case 1: /* Enable irq */
		sec_ts_irq_enable(ts, true);
			break;
	default:
			dev_err(DEV_TS, "%s: invalid value\n", __func__);
			return -EINVAL;
	}
	return size;
}

static ssize_t sec_mmi_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	data->reset(ts->mmi_ptr, 1);
	return size;
}

static ssize_t sec_mmi_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%08x\n%s%08x\n",
			"Product ID: ", data->product_id,
			"Build ID: ", data->build_id,
			"Config ID: ", data->config_id);
}

static ssize_t sec_mmi_forcereflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	data->force_calibration = true;
	return size;
}

static ssize_t sec_mmi_doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_mmi_data *data = ts->mmi_ptr;
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int result = -EFAULT;

	if (size > SEC_TS_MAX_FW_PATH) {
		dev_err(DEV_TS, "%s: FW filename is too long\n", __func__);
		return -EINVAL;
	}

	strlcpy(fw_path, buf, size);
	dev_dbg(DEV_TS, "%s: update fw from %s\n", __func__, fw_path);

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) != 0) {
		dev_err(DEV_TS, "%s: fw not available\n", __func__);
		goto err_request_fw;
	}
	dev_dbg(DEV_TS, "%s: fw size = %d\n", __func__, (int)fw_entry->size);

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	__pm_stay_awake(&ts->wakelock);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size,
					0, data->force_calibration, 0);
	if (ts->fw_invalid == false) {
		sec_mmi_fw_read_id(ts->mmi_ptr);
		sec_ts_integrity_check(ts);
	}

	data->force_calibration = false;

	mutex_unlock(&ts->modechange);
	sec_ts_irq_enable(ts, true);
	__pm_relax(&ts->wakelock);

err_request_fw:
	release_firmware(fw_entry);

	return result;
}

static void sec_mmi_sysfs(struct sec_mmi_data *data, bool enable)
{
	struct sec_ts_data *ts = data->ts_ptr;

	if (enable) {
		int ret;

		ret = sysfs_create_group(&data->i2c_client->dev.kobj, &mmi_attr_group);
		if (ret < 0) {
			input_err(true, DEV_TS,
					"%s: Failed to create MMI sysfs attributes\n", __func__);
		}
	} else
		sysfs_remove_group(&data->i2c_client->dev.kobj, &mmi_attr_group);
}

int sec_mmi_data_init(struct sec_ts_data *ts, bool enable)
{
	struct sec_mmi_data *data;

	if (enable)
		data = kzalloc(sizeof(*data), GFP_KERNEL);
	else
		data = ts->mmi_ptr;

	if (!data) {
		input_err(true, DEV_TS, "%s: MMI data is NULL\n", __func__);
		return 0;
	}

	if (enable) {
#if defined(CONFIG_DRM)
		int probe_status;
		bool panel_ready = true;
		char *pname = NULL;
#endif

		ts->mmi_ptr = (void *)data;
		data->ts_ptr = ts;
		data->i2c_client = ts->client;
		data->reset = sec_mmi_ic_reset;

		sec_mmi_dt(data);

#if defined(CONFIG_DRM)
		/* DRM panel status */
		panel_ready = dsi_display_is_panel_enable(data->ctrl_dsi,
							&probe_status, &pname);
		input_info(true, DEV_TS, "%s: drm: probe=%d, enable=%d, panel'%s'\n",
					__func__, probe_status, panel_ready, pname);
		/* check panel binding */
		if (data->bound_display && (probe_status == -ENODEV ||
				(pname && strncmp(pname, data->bound_display,
				strlen(data->bound_display))))) {
			input_err(true, DEV_TS, "%s: panel binding failed\n", __func__);
			return -ENODEV;
		}

		if (pname && strstr(pname, "dummy")) {
			input_info(true, DEV_TS, "%s: dummy panel; exiting...\n", __func__);
			return -ENODEV;
		}
#endif
		INIT_DELAYED_WORK(&data->resume_work, sec_mmi_queued_resume);
		INIT_DELAYED_WORK(&data->detection_work, sec_mmi_work);

		schedule_delayed_work(&data->detection_work, msecs_to_jiffies(50));
	}

	sec_mmi_touchscreen_class(data, enable);
	sec_mmi_register_notifier(data, enable);
	sec_mmi_sysfs(data, enable);

	if (!enable) {
		sec_mmi_sysfs(data, true);
		cancel_delayed_work(&data->resume_work);
		cancel_delayed_work(&data->detection_work);
		kfree(data);
		ts->mmi_ptr = NULL;
	}

	return 0;
}
