/*
 * Copyright (C) 2024 Motorola Mobility LLC
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

#define pr_fmt(fmt)     "GLINK_CHG:WLS: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/power/bm_adsp_ulog.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/mmi_relay.h>

#include "glink_device.h"

#define FOD_GAIN_MAX_LEN 16
#define FOD_CURR_MAX_LEN 7
#define TX_INT_FOD (0x01<<12)

#define SYS_MODE_BACK_POWER	1
#define SYS_MODE_TX		2
#define SYS_MODE_RX		3

enum wls_notify_event {
	NOTIFY_EVENT_WLS_RX_CONNECTED,
	NOTIFY_EVENT_WLS_RX_OVERTEMP,
	NOTIFY_EVENT_WLS_CHANGE,
	NOTIFY_EVENT_WLS_ERROR,
	NOTIFY_EVENT_WLS_WLC_CHANGE,
	NOTIFY_EVENT_WLS_RX_DEV_INFO_UPDATE,
};

struct fod_curr {
	u32 fod_curr_array[FOD_CURR_MAX_LEN];
};

struct fod_gain {
	u32 fod_gain_array[FOD_GAIN_MAX_LEN];
};

struct wls_chip {
	struct glink_dev *dev;
	u32 type;
	struct wls_info wls_info;
	bool cfg_done;
	bool init_done;
	struct power_supply *wls_psy;
	struct work_struct wls_work;
	struct dentry *debug_root;

	struct fod_curr	rx_fod_curr;
	struct fod_gain	rx_fod_gain;

	/* local rx control */
	u32 wls_rx_en;
	u32 wls_curr_max;
	u32 wls_volt_max;

	/* local tx control */
	u32 wls_tx_mode;
	u32 folio_mode;

	/* partner control */
	u32 wlc_light_ctl;
	u32 wlc_fan_speed;

	/* partner info */
	u32 wlc_status;
	u32 wlc_tx_type;
	u32 wlc_tx_power;
	u32 wlc_tx_capability;
	u32 wlc_tx_id;
	u32 wlc_tx_sn;
	u32 wlc_rx_connected;
	u32 wlc_rx_mfg;
	u32 wlc_rx_type;
	u32 wlc_rx_id;

	u32 weak_charge_disable;
};
static struct wls_chip *this_chip = NULL;

static void glink_wls_status_update(struct wls_chip *chip)
{
	int rc;
	u32 property;
	struct wls_info wls_info;
	struct glink_dev *dev = chip->dev;

	property = GLINK_PROP_WLS_INFO;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.get_property(dev, property, &wls_info,
				sizeof(wls_info));
	if (rc) {
		pr_err("Failed to read %s info, rc=%d\n", dev->name, rc);
		return;
	}
	chip->wls_info = wls_info;

	pr_info("%s: chip_id: 0x%04x, fw_ver: 0x%04x, "
		"irq_status: 0x%04x, sys_mode: %d, op_mode: %d, "
		"otg_boost: %d, wls_boost: %d\n",
		dev->name,
		wls_info.chip_id, wls_info.mtp_fw_ver,
		wls_info.irq_status, wls_info.sys_mode,
		wls_info.op_mode, wls_info.usb_otg,
		wls_info.wls_boost);

	switch (wls_info.sys_mode) {
	case SYS_MODE_RX:
		pr_info("%s:RX: rx_ept: %d,rx_ce: %d, rx_rp: %d, "
			"rx_fop: %dkHz, rx_vout: %dmV, rx_vrect: %dmV, "
			"rx_irect: %dmA, rx_neg_power: %dW, rx_dietemp: %d, "
			"wls_icl_ma: %dmA, wls_icl_therm_ma: %dmA, "
			"rx_iout_max_ma: %dmA, rx_vout_max_mv: %dmV\n",
			dev->name,
			wls_info.rx_ept, wls_info.rx_ce,
			wls_info.rx_rp, wls_info.rx_fop,
			wls_info.rx_vout_mv, wls_info.rx_vrect_mv,
			wls_info.rx_irect_ma, wls_info.rx_neg_power,
			wls_info.rx_dietemp, wls_info.wls_icl_ma,
			wls_info.wls_icl_therm_ma,
			wls_info.rx_iout_max_ma, wls_info.rx_vout_max_mv);

		pr_info("%s:TX: status: %d, tx_type: 0x%08x, tx_power: %d, "
			"tx_id: 0x%08x, tx_sn: 0x%08x, tx_cap: %d\n",
			dev->name, chip->wlc_status,
			chip->wlc_tx_type, chip->wlc_tx_power,
			chip->wlc_tx_id, chip->wlc_tx_sn,
			chip->wlc_tx_capability);

		pr_info("%s:CTRL: wls_rx_en: %d, wls_curr_max: %d, "
			"wls_volt_max: %d\n",
			dev->name,
			chip->wls_rx_en,
			chip->wls_curr_max, chip->wls_volt_max);
		break;
	case SYS_MODE_TX:
		pr_info("%s:TX: tx_iin_ma: %dmA, tx_vin_mv: %dmV, "
			"tx_vrect_mv: %dmV, tx_det_rx_power: %dmW, "
			"tx_power: %dmW, power_loss: %dmW, tx_fod: %d, "
			"tx_ept: 0x%04x\n",
			dev->name,
			wls_info.tx_iin_ma,
			wls_info.tx_vin_mv,
			wls_info.tx_vrect_mv,
			wls_info.tx_det_rx_power,
			wls_info.tx_power,
			wls_info.power_loss,
			(wls_info.irq_status & TX_INT_FOD) ? 1 : 0,
			wls_info.tx_ept);

		pr_info("%s:RX: status: %d, rx_connected:%d, rx_mfg: 0x%08x, "
			"rx_type: 0x%08x, rx_id: 0x%08x\n",
			dev->name, chip->wlc_status, chip->wlc_rx_connected,
			chip->wlc_rx_mfg, chip->wlc_rx_type, chip->wlc_rx_id);

		pr_info("%s:CTRL: wls_tx_mode: %d, folio_mode: %d, "
			"wlc_light_ctl: %d, wlc_fan_speed: %d\n",
			dev->name,
			chip->wls_tx_mode, chip->folio_mode,
			chip->wlc_light_ctl, chip->wlc_fan_speed);
		break;
	default:
		break;
	}
}

static ssize_t force_wls_en_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 rx_en;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &rx_en);
	if (rc) {
		pr_err("Invalid TCMD = %u\n", rx_en);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_EN;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property, &rx_en, sizeof(rx_en));
	if (rc) {
		pr_err("Failed to set wls_rx_en, rc=%d\n", rc);
		return rc;
	}
	chip->wls_rx_en = rx_en;
	return count;
}

static ssize_t force_wls_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wls_rx_en);
}
static DEVICE_ATTR(force_wls_en, 0664,
				force_wls_en_show,
				force_wls_en_store);

static ssize_t force_wls_volt_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 wls_volt_max;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &wls_volt_max);
	if (rc) {
		pr_err("Invalid TCMD = %u\n", wls_volt_max);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_VOLT_MAX;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&wls_volt_max, sizeof(wls_volt_max));
	if (rc) {
		pr_err("Failed to set wls_volt_max, rc=%d\n", rc);
		return rc;
	}
	chip->wls_volt_max = wls_volt_max;
	return count;
}

static ssize_t force_wls_volt_max_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wls_volt_max);
}
static DEVICE_ATTR(force_wls_volt_max, 0664,
				force_wls_volt_max_show,
				force_wls_volt_max_store);

static ssize_t force_wls_curr_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 wls_curr_max;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &wls_curr_max);
	if (rc) {
		pr_err("Invalid TCMD = %u\n", wls_curr_max);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_CURR_MAX;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&wls_curr_max, sizeof(wls_curr_max));
	if (rc) {
		pr_err("Failed to set wls_curr_max, rc=%d\n", rc);
		return rc;
	}
	chip->wls_curr_max = wls_curr_max;
	return count;
}

static ssize_t force_wls_curr_max_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wls_curr_max);
}
static DEVICE_ATTR(force_wls_curr_max, 0664,
				force_wls_curr_max_show,
				force_wls_curr_max_store);

static ssize_t wireless_chip_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int chip_id;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	glink_wls_status_update(chip);
	chip_id = chip->wls_info.chip_id;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "0x%04x\n", chip_id);
}
static DEVICE_ATTR(wireless_chip_id, S_IRUGO, wireless_chip_id_show, NULL);

static ssize_t wireless_fw_ver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int fw_ver;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	glink_wls_status_update(chip);
	fw_ver = chip->wls_info.mtp_fw_ver;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "0x%04x\n", fw_ver);
}
static DEVICE_ATTR(wireless_fw_ver, S_IRUGO, wireless_fw_ver_show, NULL);

static int fod_gain_store(struct wls_chip *chip,
				const char *buf, u32 *fod_array)
{
	int rc;
	int i = 0, sum = 0;
	char *buffer;
	u32 temp;
	u32 property;
	struct glink_dev *dev = chip->dev;

	buffer = (char *)buf;
	for (i = 0; i < FOD_GAIN_MAX_LEN; i++) {
		rc = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array[i] = temp;
		sum++;
		if (rc != 2)
			break;
	}

	if (sum != FOD_GAIN_MAX_LEN) {
		pr_err("fod_gain array len err %d\n", sum);
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_RX_FOD_GAIN;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.set_property(dev, property,
				fod_array, sizeof(struct fod_gain));
	if (rc) {
		pr_err("Failed to set fod gain array, rc=%d\n", rc);
		return rc;
	}
	return sum;
}

static ssize_t wls_fod_gain_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	fod_gain_store(chip, buf, chip->rx_fod_gain.fod_gain_array);
	return count;
}


static ssize_t wls_fod_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct wls_chip *chip = this_chip;

	for (i = 0; i < FOD_GAIN_MAX_LEN; i++) {
		count += scnprintf(buf+count, GLINK_SHOW_MAX_SIZE,
			"0x%02x ", chip->rx_fod_gain.fod_gain_array[i]);
	}
	count += scnprintf(buf+count, GLINK_SHOW_MAX_SIZE, "\n");

	return count;
}

static DEVICE_ATTR(wls_fod_gain, 0664, wls_fod_gain_show, wls_fod_gain_store);

static int fod_curr_store(struct wls_chip *chip,
				const char *buf, u32 *fod_array)
{
	int rc;
	int i = 0, sum = 0;
	char *buffer;
	u32 temp;
	u32 property;
	struct glink_dev *dev = chip->dev;

	buffer = (char *)buf;
	for (i = 0; i < FOD_CURR_MAX_LEN; i++) {
		rc = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array[i] = temp;
		sum++;
		if (rc != 2)
			break;
	}

	if (sum != FOD_CURR_MAX_LEN) {
		pr_err("fod_curr array len err %d\n", sum);
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_RX_FOD_CURR;
	property |= dev->id << DEVICE_ID_SHIFT;
	rc = dev->ops.set_property(dev, property,
				fod_array, sizeof(struct fod_curr));
	if (rc) {
		pr_err("Failed to set fod curr array, rc=%d\n", rc);
		return rc;
	}
	return sum;
}

static ssize_t wls_fod_curr_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	fod_curr_store(chip, buf, chip->rx_fod_curr.fod_curr_array);
	return count;
}

static ssize_t wls_fod_curr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct wls_chip *chip = this_chip;

	for (i = 0; i < FOD_CURR_MAX_LEN; i++) {
		count += scnprintf(buf+count, GLINK_SHOW_MAX_SIZE,
			"0x%02x ", chip->rx_fod_curr.fod_curr_array[i]);
	}
	count += scnprintf(buf+count, GLINK_SHOW_MAX_SIZE, "\n");
	return count;
}
static DEVICE_ATTR(wls_fod_curr, 0664, wls_fod_curr_show, wls_fod_curr_store);

static ssize_t tx_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 tx_mode = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &tx_mode);
	if (rc) {
		pr_err("Invalid tx_mode = %u\n", tx_mode);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_TX_MODE;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&tx_mode, sizeof(tx_mode));
	if (rc) {
		pr_err("Failed to set tx mode, rc=%d\n", rc);
		return rc;
	}
	chip->wls_tx_mode = tx_mode;
	if (chip->wls_psy) {
		sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, "tx_mode");
	}
	return count;
}

static ssize_t tx_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

#ifndef SKIP_QTI_CHARGER_CONFIRMAION
	glink_wls_status_update(chip);
#endif
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wls_tx_mode);
}
static DEVICE_ATTR(tx_mode, S_IRUGO|S_IWUSR, tx_mode_show, tx_mode_store);

static ssize_t tx_mode_vout_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 tx_vout = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	glink_wls_status_update(chip);
	tx_vout = chip->wls_info.tx_vrect_mv;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", tx_vout);
}
static DEVICE_ATTR(tx_mode_vout, S_IRUGO, tx_mode_vout_show, NULL);

static ssize_t wlc_light_ctl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 wlc_light_ctl = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &wlc_light_ctl);
	if (rc) {
		pr_err("Invalid wlc_light_ctl = %u\n", wlc_light_ctl);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_WLC_LIGHT_CTL;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&wlc_light_ctl,
				sizeof(wlc_light_ctl));
	if (rc) {
		pr_err("Failed to set light control, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_light_ctl = wlc_light_ctl;
	if (chip->wls_psy) {
		sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, "wlc_light_ctl");
	}
	return count;
}

static ssize_t wlc_light_ctl_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wlc_light_ctl);
}
static DEVICE_ATTR(wlc_light_ctl, S_IRUGO|S_IWUSR,
				wlc_light_ctl_show, wlc_light_ctl_store);

static ssize_t wlc_fan_speed_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 wlc_fan_speed = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &wlc_fan_speed);
	if (rc) {
		pr_err("Invalid wlc_fan_speed = %u\n", wlc_fan_speed);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_WLC_FAN_SPEED;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&wlc_fan_speed,
				sizeof(wlc_fan_speed));
	if (rc) {
		pr_err("Failed to set fan speed, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_fan_speed = wlc_fan_speed;
	if (chip->wls_psy) {
		sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, "wlc_fan_speed");
	}
	return count;
}

static ssize_t wlc_fan_speed_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wlc_fan_speed);
}
static DEVICE_ATTR(wlc_fan_speed, S_IRUGO|S_IWUSR,
				wlc_fan_speed_show, wlc_fan_speed_store);

static ssize_t wlc_tx_type_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	u32 type = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_WLC_TX_TYPE;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.get_property(chip->dev, property,
				&type, sizeof(type));
	if (rc) {
		pr_err("Failed to get tx type, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_tx_type = type;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", type);
}
static DEVICE_ATTR(wlc_tx_type, S_IRUGO, wlc_tx_type_show, NULL);

static ssize_t wlc_tx_power_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	u32 power = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_WLC_TX_POWER;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.get_property(chip->dev, property,
				&power, sizeof(power));
	if (rc) {
		pr_err("Failed to get tx power, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_tx_power = power;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", power);
}
static DEVICE_ATTR(wlc_tx_power, S_IRUGO, wlc_tx_power_show, NULL);

static ssize_t wlc_tx_capability_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	u32 capability = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_WLC_TX_CAPABILITY;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.get_property(chip->dev, property,
				&capability, sizeof(capability));
	if (rc) {
		pr_err("Failed to get tx capability, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_tx_capability = capability;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", capability);
}
static DEVICE_ATTR(wlc_tx_capability, S_IRUGO, wlc_tx_capability_show, NULL);

static ssize_t wlc_tx_id_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	u32 id = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_WLC_TX_ID;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.get_property(chip->dev,
				property, &id, sizeof(id));
	if (rc) {
		pr_err("Failed to get tx id, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_tx_id = id;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", id);
}
static DEVICE_ATTR(wlc_tx_id, S_IRUGO, wlc_tx_id_show, NULL);

static ssize_t wlc_tx_sn_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int rc;
	u32 property;
	u32 sn = 0;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	property = GLINK_PROP_WLS_WLC_TX_SN;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.get_property(chip->dev,
				property, &sn, sizeof(sn));
	if (rc) {
		pr_err("Failed to get tx sn, rc=%d\n", rc);
		return rc;
	}
	chip->wlc_tx_sn = sn;
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", sn);
}
static DEVICE_ATTR(wlc_tx_sn, S_IRUGO, wlc_tx_sn_show, NULL);

static ssize_t rx_connected_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				chip->wlc_rx_connected);
}
static DEVICE_ATTR(rx_connected, S_IRUGO, rx_connected_show, NULL);

static ssize_t rx_dev_manufacturing_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%#x\n",
				chip->wlc_rx_mfg);
}
static DEVICE_ATTR(rx_dev_manufacturing, S_IRUGO,
				rx_dev_manufacturing_show,
				NULL);

static ssize_t rx_dev_type_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%#x\n",
				chip->wlc_rx_type);
}
static DEVICE_ATTR(rx_dev_type, S_IRUGO, rx_dev_type_show, NULL);

static ssize_t rx_dev_id_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%#x\n",
				chip->wlc_rx_id);
}
static DEVICE_ATTR(rx_dev_id, S_IRUGO, rx_dev_id_show, NULL);

static ssize_t wlc_st_changed_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wlc_status);
}
static DEVICE_ATTR(wlc_st_changed, S_IRUGO, wlc_st_changed_show, NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 wls_curr_max;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &wls_curr_max);
	if (rc) {
		pr_err("Invalid TCMD = %u\n", wls_curr_max);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_CURR_MAX;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&wls_curr_max, sizeof(wls_curr_max));
	if (rc) {
		pr_err("Failed to set wls_curr_max, rc=%d\n", rc);
		return rc;
	}
	chip->wls_curr_max = wls_curr_max;
	return count;
}

static ssize_t wls_input_current_limit_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}
	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->wls_curr_max);
}
static DEVICE_ATTR(wls_input_current_limit, S_IRUGO|S_IWUSR,
				wls_input_current_limit_show,
				wls_input_current_limit_store);

static ssize_t wls_weak_charge_disable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 weak_charge_disable;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &weak_charge_disable);
	if (rc) {
		pr_err("Invalid TCMD = %u\n", weak_charge_disable);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_WEAK_CHARGE_CTRL;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&weak_charge_disable,
				sizeof(weak_charge_disable));
	if (rc) {
		pr_err("Failed to set weak_charger_disable, rc=%d\n", rc);
		return rc;
	}
	chip->weak_charge_disable = weak_charge_disable;
	return count;
}

static ssize_t wls_weak_charge_disable_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				chip->weak_charge_disable);
}
static DEVICE_ATTR(wls_weak_charge_disable, S_IRUGO|S_IWUSR,
				wls_weak_charge_disable_show,
				wls_weak_charge_disable_store);

static ssize_t folio_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 property;
	u32 folio_mode;
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	rc = kstrtouint(buf, 0, &folio_mode);
	if (rc) {
		pr_err("Invalid folio_mode = %u\n", folio_mode);
		return -EINVAL;
	}

	property = GLINK_PROP_WLS_FOLIO_MODE;
	property |= chip->dev->id << DEVICE_ID_SHIFT;
	rc = chip->dev->ops.set_property(chip->dev, property,
				&folio_mode, sizeof(folio_mode));
	if (rc) {
		pr_err("Failed to set folio mode, rc=%d\n", rc);
		return rc;
	}
	chip->folio_mode = folio_mode;

	return count;
}

static ssize_t folio_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct wls_chip *chip = this_chip;

	if (!chip) {
		pr_err("wls chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", chip->folio_mode);
}
static DEVICE_ATTR(folio_mode, S_IRUGO|S_IWUSR,
				folio_mode_show, folio_mode_store);

static int glink_wls_dump_info_show(struct seq_file *m, void *data)
{
	struct wls_info info;
	struct wls_chip *chip = m->private;

	glink_wls_status_update(chip);
	info = chip->wls_info;

	seq_printf(m, "CHIP_ID: 0x%04x\n", info.chip_id);

	seq_printf(m, "MTP_FW_VER: 0x%04x\n", info.mtp_fw_ver);

	seq_printf(m, "IRQ STATUS: 0x%04x\n", info.irq_status);

	seq_printf(m, "SYS_MODE:  RX/TX %d\n", info.sys_mode);

	seq_printf(m, "OP_MODE:  BPP/EPP/Moto50W 0x%x\n", info.op_mode);

	seq_printf(m, "RX_FOP:   %dkHz\n", info.rx_fop);

	seq_printf(m, "RX_VOUT: %dmV\n", info.rx_vout_mv);

	seq_printf(m, "RX_VRECT: %dmV\n", info.rx_vrect_mv);

	seq_printf(m, "RX_IRECT: %dmA\n", info.rx_irect_ma);

	seq_printf(m, "RX_EPT: 0x%04x\n", info.rx_ept);

	seq_printf(m, "RX_CE: %d\n", info.rx_ce);

	seq_printf(m, "RX_RP: %d\n", info.rx_rp);

	seq_printf(m, "RX_DieTemp: %dC\n", info.rx_dietemp);

	seq_printf(m, "RX_NEG_POWER: %dw\n", info.rx_neg_power);

	seq_printf(m, "TX_IIN: %dmA\n", info.tx_iin_ma);

	seq_printf(m, "TX_VIN: %dmV\n", info.tx_vin_mv);

	seq_printf(m, "TX_VRECT: %dmV\n", info.tx_vrect_mv);

	seq_printf(m, "TX_DET_RX_POWER: %dmW\n", info.tx_det_rx_power);

	seq_printf(m, "TX_POWER: %dmW\n", info.tx_power);

	seq_printf(m, "TX_EPT_RSN: 0x%04x\n", info.tx_ept);

	seq_printf(m, "POWER_LOSS: %dmW\n", info.power_loss);

	seq_printf(m, "TX_FOD: %d\n", (info.irq_status & TX_INT_FOD)? 1 : 0);

	seq_printf(m, "USB_OTG: %d\n", info.usb_otg);

	seq_printf(m, "WLS_BOOST: %d\n", info.wls_boost);

	seq_printf(m, "WLS_ICL_MA: %d\n", info.wls_icl_ma);

	seq_printf(m, "WLS_ICL_THERM_MA: %d\n", info.wls_icl_therm_ma);

	return 0;
}

static int glink_wls_dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct wls_chip *chip = inode->i_private;

	return single_open(file, glink_wls_dump_info_show, chip);
}

static const struct file_operations glink_wls_dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= glink_wls_dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void glink_wls_create_files(struct wls_chip *chip)
{
	int rc;
	struct dentry *entry;

	chip->debug_root = debugfs_create_dir("mmi_glink_charger", NULL);
	if (!chip->debug_root) {
		pr_err("Couldn't create debug dir\n");
	} else {
		entry = debugfs_create_file("wls_dump_info",
				S_IFREG | S_IRUGO,
				chip->debug_root, chip,
				&glink_wls_dump_debugfs_ops);
		if (!entry)
			pr_err("Couldn't create wls_dump_info debug file\n");
	}

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_tx_mode);
        if (rc)
		pr_err("couldn't create wls tx mode\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_tx_mode_vout);
        if (rc)
		pr_err("couldn't create wls tx mode vout\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_connected);
        if (rc)
		pr_err("couldn't create wls rx_connected\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_manufacturing);
        if (rc)
		pr_err("couldn't create wireless rx_dev_manufacturing\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_type);
        if (rc)
		pr_err("couldn't create wireless rx_dev_type\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_id);
        if (rc)
		pr_err("couldn't create wireless rx_dev_id\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wls_input_current_limit);
        if (rc)
		pr_err("couldn't create wls input current limit error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_folio_mode);
        if (rc)
		pr_err("couldn't create wls folio mode error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_light_ctl);
        if (rc)
		pr_err("couldn't create wlc light control error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_fan_speed);
        if (rc)
		pr_err("couldn't create wlc fan speed error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_type);
        if (rc)
		pr_err("couldn't create wlc tx type error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_power);
        if (rc)
		pr_err("couldn't create wlc tx power capacity error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_capability);
        if (rc)
		pr_err("couldn't create wlc tx capability error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_id);
        if (rc)
		pr_err("couldn't create wlc tx id error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_sn);
        if (rc)
		pr_err("couldn't create wlc tx sn error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_st_changed);
        if (rc)
		pr_err("couldn't create wlc status changed error\n");

	rc = device_create_file(chip->wls_psy->dev.parent,
				&dev_attr_wls_weak_charge_disable);
	if (rc)
		pr_err("couldn't create wlc weak charge disable error\n");

	rc = device_create_file(chip->dev->dev,
				&dev_attr_force_wls_en);
	if (rc) {
		pr_err("Couldn't create force_wls_en\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_force_wls_volt_max);
	if (rc) {
		pr_err("Couldn't create force_wls_volt_max\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_force_wls_curr_max);
	if (rc) {
		pr_err("Couldn't create force_wls_curr_max\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_wireless_chip_id);
	if (rc) {
		pr_err("Couldn't create wireless_chip_id\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_wireless_fw_ver);
	if (rc) {
		pr_err("Couldn't create wireless_fw_ver\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_wls_fod_curr);
	if (rc) {
		pr_err("Couldn't create wls_fod_curr\n");
	}

	rc = device_create_file(chip->dev->dev,
				&dev_attr_wls_fod_gain);
	if (rc) {
		pr_err("Couldn't create wls_fod_gain\n");
	}
}

static void glink_wls_remove_files(struct wls_chip *chip)
{
	device_remove_file(chip->dev->dev,
				&dev_attr_force_wls_en);
	device_remove_file(chip->dev->dev,
				&dev_attr_force_wls_volt_max);
	device_remove_file(chip->dev->dev,
				&dev_attr_force_wls_curr_max);
	device_remove_file(chip->dev->dev,
				&dev_attr_wireless_fw_ver);
	device_remove_file(chip->dev->dev,
				&dev_attr_wireless_chip_id);
	device_remove_file(chip->dev->dev,
				&dev_attr_wls_fod_curr);
	device_remove_file(chip->dev->dev,
				&dev_attr_wls_fod_gain);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_tx_mode);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_tx_mode_vout);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_connected);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_manufacturing);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_type);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_rx_dev_id);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wls_input_current_limit);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_folio_mode);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_light_ctl);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_fan_speed);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_type);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_power);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_capability);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_id);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_tx_sn);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wlc_st_changed);

	device_remove_file(chip->wls_psy->dev.parent,
				&dev_attr_wls_weak_charge_disable);

	if (chip->debug_root)
		debugfs_remove_recursive(chip->debug_root);
}

static int glink_wls_cfg(struct glink_dev *dev,
					struct glink_dev_cfg *cfg)
{
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	return 0;
}

static int glink_wls_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct wls_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("wls chip is not ready\n");
		return -ENODEV;
	}

	if (chip->init_done)
		return 0;

	if (!chip->wls_psy) {
		chip->wls_psy = power_supply_get_by_name("wireless");
		if (!chip->wls_psy) {
			pr_err("No wireless supply found\n");
			return -EAGAIN;
		}
		pr_info("wireless power supply is found\n");
	}

	rc = glink_wls_cfg(dev, cfg);
	if (rc) {
		pr_err("failed to cfg glink %s\n", dev->name);
		return rc;
	}
	chip->cfg_done = true;

	glink_wls_create_files(chip);

	chip->init_done = true;
	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_wls_deinit(struct glink_dev *dev)
{
	struct wls_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("wls chip is not ready\n");
		return -ENODEV;
	}

	if (chip->wls_psy) {
		glink_wls_remove_files(chip);
		power_supply_put(chip->wls_psy);
		chip->wls_psy = NULL;
	}
	devm_kfree(dev->dev, chip);
	dev->devdata = NULL;
	this_chip = NULL;

	return 0;
}

static void glink_wls_work(struct work_struct *work)
{
	struct wls_chip *chip = container_of(work,
				struct wls_chip, wls_work);

	if (!chip->init_done && chip->cfg_done)
		glink_wls_init(chip->dev, &chip->dev->cfg);
	glink_wls_status_update(chip);
}

static void glink_wls_event_handler(struct wls_chip *chip,
				unsigned long event,
				struct glink_dev_notify_data *notify_data)
{
	const char *sysfile = NULL;

	pr_info("wireless event notify, event=%lu\n", event);
        switch (event) {
        case NOTIFY_EVENT_WLS_RX_CONNECTED:
	/* RX connected update */
		if (notify_data->data[0] == chip->wlc_rx_connected)
			break;
		chip->wlc_rx_connected = notify_data->data[0];
		pr_info("wlc_rx_connected=%d\n", chip->wlc_rx_connected);
		sysfile = "rx_connected";
		if (chip->wls_psy) {
			sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, sysfile);
			power_supply_changed(chip->wls_psy);
		}
		break;
        case NOTIFY_EVENT_WLS_RX_OVERTEMP:
		break;
        case NOTIFY_EVENT_WLS_CHANGE:
		if (notify_data->data[0] == chip->wls_tx_mode)
			break;
		chip->wls_tx_mode = notify_data->data[0];
		pr_info("tx_mode=%d\n", chip->wls_tx_mode);
		sysfile = "tx_mode";
		if (chip->wls_psy) {
			sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, sysfile);
			power_supply_changed(chip->wls_psy);
		}
		break;
        case NOTIFY_EVENT_WLS_WLC_CHANGE:
	/* WLC status update */
		if (notify_data->data[0] == chip->wlc_status)
			break;
		chip->wlc_status = notify_data->data[0];
		pr_info("wlc_st_changed=%d\n", chip->wlc_status);
		sysfile = "wlc_st_changed";
		if (chip->wls_psy) {
			sysfs_notify(&chip->wls_psy->dev.parent->kobj,
				NULL, sysfile);
			power_supply_changed(chip->wls_psy);
		}
		break;
	case NOTIFY_EVENT_WLS_RX_DEV_INFO_UPDATE:
	/* Partner RX info update */
		if (notify_data->data[0] != chip->wlc_rx_mfg) {
			pr_info("rx_dev_mfg: %#x\n", notify_data->data[0]);
			sysfile = "rx_dev_manufacturing";
			chip->wlc_rx_mfg = notify_data->data[0];
			if (chip->wls_psy) {
				sysfs_notify(&chip->wls_psy->dev.parent->kobj,
					NULL, sysfile);
				power_supply_changed(chip->wls_psy);
			}
		}

		if (notify_data->data[1] != chip->wlc_rx_type) {
			pr_info("rx_dev_type: %#x\n", notify_data->data[1]);
			sysfile = "rx_dev_type";
			chip->wlc_rx_type = notify_data->data[1];
			if (chip->wls_psy) {
				sysfs_notify(&chip->wls_psy->dev.parent->kobj,
					NULL, sysfile);
				power_supply_changed(chip->wls_psy);
			}
		}

		if (notify_data->data[2] != chip->wlc_rx_id) {
			pr_info("rx_dev_id: %#x\n", notify_data->data[2]);
			sysfile = "rx_dev_id";
			chip->wlc_rx_id = notify_data->data[2];
			if (chip->wls_psy) {
				sysfs_notify(&chip->wls_psy->dev.parent->kobj,
					NULL, sysfile);
				power_supply_changed(chip->wls_psy);
			}
		}
		break;
        default:
		pr_err("Unknown wireless event: %#lx\n", event);
                break;
        }
}

static int glink_wls_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct wls_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("glink %s is not ready\n", dev->name);
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_WLS_CHG:
		glink_wls_event_handler(chip, notification, notify_data);
		break;
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (chip->init_done)
				glink_wls_cfg(dev, &dev->cfg);
			break;
		case MMI_GLINK_STATE_DOWN:
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_PSY_USR:
		switch (notification) {
		case MMI_PSY_CHANGE_WLS:
			schedule_work(&chip->wls_work);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		schedule_work(&chip->wls_work);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_wls_setup(struct glink_dev *dev)
{
	struct wls_chip *chip = dev->devdata;

	if (chip) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	dev->ops.init = glink_wls_init;
	dev->ops.notify	= glink_wls_notify;
	dev->ops.deinit = glink_wls_deinit;

	chip = devm_kzalloc(dev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	INIT_WORK(&chip->wls_work, glink_wls_work);
	dev->devdata = chip;
	chip->dev = dev;
	this_chip = chip;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
