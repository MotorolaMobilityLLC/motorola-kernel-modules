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
#include <soc/qcom/mmi_boot_info.h>
#include <linux/touchscreen_mmi.h>

#include "sec_ts.h"
#include "sec_mmi.h"

static ssize_t sec_mmi_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_data_show(struct device *dev,
		struct device_attribute *attr, char *buf);


static DEVICE_ATTR(address, (S_IWUSR | S_IWGRP), NULL, sec_mmi_address_store);
static DEVICE_ATTR(size, (S_IWUSR | S_IWGRP), NULL, sec_mmi_size_store);
static DEVICE_ATTR(write, (S_IWUSR | S_IWGRP), NULL, sec_mmi_write_store);
static DEVICE_ATTR(data, S_IRUGO, sec_mmi_data_show, NULL);

static struct attribute *factory_attributes[] = {
	&dev_attr_address.attr,
	&dev_attr_size.attr,
	&dev_attr_data.attr,
	&dev_attr_write.attr,
	NULL,
};

static struct attribute_group factory_attr_group = {
	.attrs = factory_attributes,
};

void sec_mmi_gesture_handler(void *data) {
	struct sec_ts_gesture_status *gs =
		(struct sec_ts_gesture_status *)data;

	if (gs->eid != SEC_TS_GESTURE_EVENT) {
		pr_info("%s: invalid gesture ID\n", __func__);
		return;
	}

	pr_info("%s: GESTURE %x %x %x %x %x %x %x %x\n", __func__,
		gs->eid | (gs->stype << 4) | (gs->sf << 6),
		gs->gesture_id,
		gs->gesture_data_1,
		gs->gesture_data_2,
		gs->gesture_data_3,
		gs->gesture_data_4,
		gs->reserved_1,
		gs->left_event_5_0 | (gs->reserved_2 << 6));

	switch (gs->gesture_id) {
	case 1:
		pr_info("%s: single tap\n", __func__);
			break;
	case 2:
		pr_info("%s: zero tap; x=%x, y=%x, w=%x, p=%x\n", __func__,
			gs->gesture_data_1+((gs->gesture_data_3 & 0x0f) << 8),
			gs->gesture_data_2+((gs->gesture_data_3 & 0xf0) << 4),
			gs->gesture_data_4,
			gs->reserved_1);
			break;
	default:
		pr_info("%s: unknown id=%x\n", __func__, gs->gesture_id);
	}
}

static void sec_mmi_ic_reset(struct sec_ts_data *ts, int mode)
{
	if (!gpio_is_valid(ts->plat_data->rst_gpio) ||
		!gpio_get_value(ts->plat_data->rst_gpio))
		return;

	__pm_stay_awake(&ts->wakelock);
	mutex_lock(&ts->modechange);
	/* disable irq to ensure getting boot complete */
	sec_ts_irq_enable(ts, false);

	if (mode == 1) {
		gpio_set_value(ts->plat_data->rst_gpio, 0);
		usleep_range(10, 10);
		gpio_set_value(ts->plat_data->rst_gpio, 1);
		dev_dbg(&ts->client->dev, "%s: reset line toggled\n", __func__);
	} else {
		int ret;

		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
		if (ret < 0)
			dev_err(&ts->client->dev, "%s: error sending sw reset cmd\n", __func__);
	}

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	sec_ts_irq_enable(ts, true);

	mutex_unlock(&ts->modechange);
	__pm_relax(&ts->wakelock);

	dev_dbg(&ts->client->dev, "%s: hw reset done\n", __func__);
}

static void sec_mmi_enable_touch(struct sec_ts_data *ts)
{
	int ret;
	unsigned char buffer[8];

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, buffer, sizeof(buffer));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: failed to read fw version (%d)\n",
			__func__, ret);
	} else {
		if (buffer[0] == 0x17) {
			ts->device_id[3] = 0x7C;
			input_info(true, &ts->client->dev,
				"%s: set device_id[3] is 0x7C\n",
				__func__, ret);
		}
	}
	sec_ts_integrity_check(ts);
	sec_ts_sense_on(ts);
	dev_dbg(&ts->client->dev, "%s: touch sensing ready\n", __func__);
}

#define MAX_DATA_SZ	1024
static u8 reg_address;
static int data_size;

static ssize_t sec_mmi_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int error;
	long value;

	error = kstrtol(buf, 0, &value);
	if (error || value > MAX_DATA_SZ)
		return -EINVAL;

	reg_address = (u8)value;
	dev_info(dev, "%s: read address 0x%02X\n", __func__, reg_address);

	return size;
}

static ssize_t sec_mmi_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int error;
	long value;

	error = kstrtol(buf, 0, &value);
	if (error)
		return -EINVAL;

	data_size = (unsigned int)value;
	dev_info(dev, "%s: read size %u\n", __func__, data_size);

	return size;
}

static ssize_t sec_mmi_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	char ascii[3] = {0};
	char *sptr, *eptr;
	size_t data_sz = size;
	u8 hex[MAX_DATA_SZ];
	int byte, error;
	long value;

	dev = MMI_DEV_TO_TS_DEV(dev);

	if (*(buf + size - 1) == '\n')
		data_sz--;

	if ((data_sz%2 != 0) || (data_sz/2 > MAX_DATA_SZ)) {
		dev_err(dev, "%s: odd input\n", __func__);
		return -EINVAL;
	}

	sptr = (char *)buf;
	eptr = (char *)buf + data_sz;
	pr_debug("%s: data to write: ", __func__);
	for (byte = 0; sptr < eptr; sptr += 2, byte++) {
		memcpy(ascii, sptr, 2);
		error = kstrtol(ascii, 16, &value);
		if (error)
			break;
		hex[byte] = (u8)value;
		pr_cont("0x%02x ", hex[byte]);
	}
	pr_cont("; total=%d\n", byte);

	if (error) {
		dev_err(dev, "%s: input conversion failed\n", __func__);
		return -EINVAL;
	}

	error = sec_ts_reg_store(dev, attr, hex, byte);
	if (error != byte) {
		dev_err(dev, "%s: write error\n", __func__);
		return -EIO;
	}

	dev_info(dev, "%s: written %d bytes to address 0x%02X\n",
			__func__, byte, hex[0]);
	return size;
}

static ssize_t sec_mmi_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 buffer[MAX_DATA_SZ];
	ssize_t length, blen = 0;
	int i;

	dev = MMI_DEV_TO_TS_DEV(dev);

	/* init parameters required for sec_ts_regread_show() */
	sec_ts_lv1_params(reg_address, data_size);
	length = sec_ts_regread_show(dev, attr, buffer);
	if (length != data_size)
		return sprintf(buf,
			"error %zu reading %u bytes from reg addr=0x%02X\n",
			length, data_size, reg_address);

	dev_info(dev, "%s: read %u bytes from reg addr=0x%02X\n",
			__func__, data_size, reg_address);
	for (i = 0; i < length; i++)
		blen += scnprintf(buf + blen, PAGE_SIZE - blen, "%02X ", buffer[i]);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "\n");

	return blen;
}


static int sec_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "samsung");
}

static int sec_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%c%c%c%02x",
		tolower(ts->device_id[0]), tolower(ts->device_id[1]),
		ts->device_id[2], ts->device_id[3]);
}


static inline void bdc2ui(unsigned int *dest, unsigned char *src, size_t size)
{
	int i, shift = 0;
	*dest = 0;
	for (i = 0; i < size; i++, shift += 8)
		*dest += src[i] << (24 - shift);
}

static int sec_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned char buffer[8];
	unsigned int build_id = 0;
	int ret;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read fw version (%d)\n",
				__func__, ret);
	else {
		if (buffer[0] == 0x17)
			ts->device_id[3] = 0x7C;
		bdc2ui(&build_id, buffer, sizeof(build_id));
		dev_dbg(dev, "%s: IMAGE VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%08x", build_id);
}

static int sec_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned char buffer[8];
	unsigned int config_id = 0;
	int ret;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read fw version (%d)\n",
				__func__, ret);
	else {
		bdc2ui(&config_id, buffer+4, sizeof(config_id));
		dev_dbg(dev, "%s: CONFIG VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%08x", config_id);
}

static int sec_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int sec_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = gpio_get_value(ts->plat_data->irq_gpio);

	return 0;
}

static int sec_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = ts->irq_enabled ? 1 : 0;

	return 0;
}

static int sec_mmi_methods_get_poweron(struct device *dev, void *idata) {

	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = (ts->power_status != SEC_TS_STATE_POWER_OFF) ? 1 : 0;

	return 0;
}

static int sec_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = ts->fw_invalid ? 1 : 0;

	return 0;
}


static int sec_mmi_methods_drv_irq(struct device *dev, int state) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	switch (state) {
	case 0: /* Disable irq */
		sec_ts_irq_enable(ts, false);
			break;
	case 1: /* Enable irq */
		sec_ts_irq_enable(ts, true);
			break;
	default:
			dev_err(dev, "%s: invalid value\n", __func__);
			return -EINVAL;
	}
	return 0;

}

static int sec_mmi_methods_reset(struct device *dev, int type) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	sec_mmi_ic_reset(ts, type);

	return 0;
}

static int sec_mmi_methods_power(struct device *dev, int on) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return sec_ts_power((void *)ts, on == TS_MMI_POWER_ON);
}

static int sec_mmi_methods_refresh_rate(struct device *dev, int freq) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char f = freq & 0xff;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	dev_info(dev, "%s: writing refresh rate %dhz\n", __func__, freq);
	ret = ts->sec_ts_i2c_write(ts, 0x4C, &f, 1);
	if (ret < 0) {
		dev_err(dev, "%s: failed refresh_rate (%d)\n", __func__, ret);
		return ret;
	}
	return 0;
}

static int sec_mmi_methods_pinctrl(struct device *dev, int on) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return sec_ts_pinctrl_configure(ts, on == TS_MMI_PINCTL_ON);
}

static int sec_mmi_firmware_update(struct device *dev, char *fwname) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	int result = -EFAULT;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fwname, &ts->client->dev) != 0) {
		dev_err(dev, "%s: fw not available\n", __func__);
		goto err_request_fw;
	}
	dev_info(dev, "%s: update fw from %s, size = %d\n",
			__func__, fwname, (int)fw_entry->size);

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	__pm_stay_awake(&ts->wakelock);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size,
					0, true, 0);
	if (ts->fw_invalid == false)
		sec_mmi_enable_touch(ts);

	mutex_unlock(&ts->modechange);
	sec_ts_irq_enable(ts, true);
	__pm_relax(&ts->wakelock);

err_request_fw:
	release_firmware(fw_entry);

	return result;
}

static int sec_mmi_firmware_erase(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_err(dev, "%s: Power off state\n", __func__);
		return -EIO;
	}

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	__pm_stay_awake(&ts->wakelock);

	ts->fw_invalid = true;

	mutex_unlock(&ts->modechange);
	__pm_relax(&ts->wakelock);

	return 0;
}

static int sec_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	if (strncmp(bi_bootmode(), "mot-factory", strlen("mot-factory")) == 0)
		*group = &factory_attr_group;
	else
		*group = NULL;
	return 0;
}

static int sec_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	switch (to) {
	case TS_MMI_PM_DEEPSLEEP:
		return sec_ts_set_lowpowermode(ts, TO_SLEEP_MODE);
	case TS_MMI_PM_GESTURE:
		return sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	case TS_MMI_PM_ACTIVE:
		return sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
	default:
		dev_warn(dev, "panel mode %d is invalid.\n", to);
		return -EINVAL;
	}
}

static int sec_mmi_wait_for_ready(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
}

static int sec_mmi_pre_suspend(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned long start_wait_jiffies = jiffies;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	do {
		if (!ts->wakelock.active)
			break;
		usleep_range(1000, 1000);
	} while (1);

	if ((jiffies - start_wait_jiffies))
		dev_info(dev, "%s: entering suspend delayed for %ums\n",
			__func__, jiffies_to_msecs(jiffies - start_wait_jiffies));

	return 0;
}

static int sec_mmi_post_suspend(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_mmi_post_resume(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char buffer = 0;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	dev_dbg(dev, "%s: sending sense_on...\n", __func__);
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		dev_err(dev, "%s: failed sense_on (%d)\n", __func__, ret);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS,
					&buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read boot status (%d)\n", __func__, ret);

	ts->fw_invalid = buffer == SEC_TS_STATUS_BOOT_MODE;

	return 0;
}

static struct ts_mmi_methods sec_ts_mmi_methods = {
	.get_vendor = sec_mmi_methods_get_vendor,
	.get_productinfo = sec_mmi_methods_get_productinfo,
	.get_build_id = sec_mmi_methods_get_build_id,
	.get_config_id = sec_mmi_methods_get_config_id,
	.get_bus_type = sec_mmi_methods_get_bus_type,
	.get_irq_status = sec_mmi_methods_get_irq_status,
	.get_drv_irq = sec_mmi_methods_get_drv_irq,
	.get_poweron = sec_mmi_methods_get_poweron,
	.get_flashprog = sec_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  sec_mmi_methods_reset,
	.drv_irq = sec_mmi_methods_drv_irq,
	.power = sec_mmi_methods_power,
	.pinctrl = sec_mmi_methods_pinctrl,
	.refresh_rate = sec_mmi_methods_refresh_rate,
	/* Firmware */
	.firmware_update = sec_mmi_firmware_update,
	.firmware_erase = sec_mmi_firmware_erase,
	/* vendor specific attribute group */
	.extend_attribute_group = sec_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = sec_mmi_panel_state,
	.wait_for_ready = sec_mmi_wait_for_ready,
	.post_resume = sec_mmi_post_resume,
	.pre_suspend = sec_mmi_pre_suspend,
	.post_suspend = sec_mmi_post_suspend,
};

int sec_mmi_data_init(struct sec_ts_data *ts, bool enable)
{
	int ret;
	if (enable) {
		ret = ts_mmi_dev_register(&ts->client->dev, &sec_ts_mmi_methods);
		if (ret) {
			dev_err(&ts->client->dev, "Failed to register ts mmi\n");
			return ret;
		}
		if (ts->fw_invalid == false) {
			sec_mmi_enable_touch(ts);
		} else /* stuck in BL mode, update productinfo to report 'se77c' */
			ts->device_id[3] = 0x7C;
	} else
		ts_mmi_dev_unregister(&ts->client->dev);

	return 0;
}
