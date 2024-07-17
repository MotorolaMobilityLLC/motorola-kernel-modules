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

#define pr_fmt(fmt)     "GLINK_CHG:USB: %s: " fmt, __func__

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
#include <linux/mmi_relay.h>

#include "glink_device.h"

enum {
	NOTIFY_EVENT_USB_LPD_STATUS,
	NOTIFY_EVENT_USB_CID_STATUS,
	NOTIFY_EVENT_USB_VBUS_STATUS,
};

enum {
	LPD_MITIGATE_DRP,
	LPD_MITIGATE_SNK,
	LPD_MITIGATE_SRC,
	LPD_MITIGATE_DISABLE,
	LPD_MITIGATE_INVALID,
};

enum typec_partner_type {
	TYPEC_PARTNER_NONE,
	TYPEC_PARTNER_UNKNOWN,
	TYPEC_PARTNER_SNK_USB_SDP,
	TYPEC_PARTNER_SNK_USB_OCP,
	TYPEC_PARTNER_SNK_USB_CDP,
	TYPEC_PARTNER_SNK_USB_DCP,
	TYPEC_PARTNER_SNK_USB_FLOAT,
	TYPEC_PARTNER_SNK_TYPEC_DEFAULT,
	TYPEC_PARTNER_SNK_TYPEC_RP_MEDIUM_1P5A,
	TYPEC_PARTNER_SNK_TYPEC_RP_HIGH_3A,
	TYPEC_PARTNER_SNK_DEBUG_ACCESS,
	TYPEC_PARTNER_SNK_USB_QC_2P0,
	TYPEC_PARTNER_SNK_USB_QC_3P0,
	TYPEC_PARTNER_SNK_USB_QC_3P5,
	TYPEC_PARTNER_SNK_UFCS,
	TYPEC_PARTNER_SNK_PD,
	TYPEC_PARTNER_SNK_PPS,
	TYPEC_PARTNER_SRC_TYPEC_POWERCABLE,              // RD-RA
	TYPEC_PARTNER_SRC_TYPEC_UNORIENTED_DEBUG_ACCESS, // RD/RD
	TYPEC_PARTNER_SRC_TYPEC_AUDIO_ACCESS,            // RA/RA
	TYPEC_PARTNER_WLS_SRC_BPP,
	TYPEC_PARTNER_WLS_SNK_BPP,
	TYPEC_PARTNER_WLS_SNK_EPP,
	TYPEC_PARTNER_WLS_SNK_PDDE,
	TYPEC_PARTNER_INVALID,
};

struct usb_chip {
	struct glink_dev *dev;
	struct usb_info usb_info;
	struct work_struct usb_work;

	int otp_en_gpio;
	bool therm_supported;
	unsigned long therm_state;
	struct thermal_cooling_device *cdev;

	u32 lpd_mitigate_mode;
	struct power_supply *batt_psy;
	char *uenvp[2];

	bool cfg_done;
	bool init_done;
};

static struct usb_chip *this_chip = NULL;

static ssize_t force_pmic_icl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int rc;
	unsigned long pmic_icl;
	struct usb_chip *chip = this_chip;

	if (!chip || !chip->dev) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtoul(buf, 0, &pmic_icl);
	if (rc) {
		pr_err("Invalid usb icl = %lu\n", pmic_icl);
		return -EINVAL;
	}

	rc = chip->dev->ops.set_property(chip->dev,
				GLINK_PROP_CHG_PMIC_ICL,
				&pmic_icl,
				sizeof(pmic_icl));

	return rc ? rc : count;
}

static ssize_t force_pmic_icl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int data;
	struct usb_chip *chip = this_chip;

	if (!chip || !chip->dev) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	chip->dev->ops.get_property(chip->dev,
				GLINK_PROP_CHG_PMIC_ICL,
				&data,
				sizeof(int));

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", data);
}
static DEVICE_ATTR(force_pmic_icl, 0664,
				force_pmic_icl_show,
				force_pmic_icl_store);

static ssize_t force_usb_suspend_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int rc;
	unsigned long usb_suspend;
	struct usb_chip *chip = this_chip;

	if (!chip || !chip->dev) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtoul(buf, 0, &usb_suspend);
	if (rc) {
		pr_err("Invalid usb suspend = %lu\n", usb_suspend);
		return -EINVAL;
	}

	rc = chip->dev->ops.set_property(chip->dev,
				GLINK_PROP_USB_SUSPEND,
				&usb_suspend,
				sizeof(usb_suspend));

	return rc ? rc : count;
}

static ssize_t force_usb_suspend_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int data;
	struct usb_chip *chip = this_chip;

	if (!chip || !chip->dev) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	chip->dev->ops.get_property(chip->dev,
				GLINK_PROP_USB_SUSPEND,
				&data,
				sizeof(int));

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", data);
}
static DEVICE_ATTR(force_usb_suspend, 0664,
				force_usb_suspend_show,
				force_usb_suspend_store);

static ssize_t cid_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct usb_chip *chip = this_chip;

	if (!chip) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n",
				chip->usb_info.cid_st);
}
static DEVICE_ATTR(cid_status, 0444, cid_status_show, NULL);

static ssize_t typec_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned int reset = 0;
	struct usb_chip *chip = this_chip;

	if (!chip || !chip->dev) {
		pr_err("usb chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &reset);
	if (rc) {
		pr_err("Invalid typec_reset = %d\n", reset);
		return -EINVAL;
	}

	if (reset)
		pr_warn("typec_reset triggered\n");
	else
		return count;

	rc = chip->dev->ops.set_property(chip->dev,
				GLINK_PROP_USB_TYPEC_RESET,
				&reset,
				sizeof(reset));

	return rc ? rc : count;
}
static DEVICE_ATTR(typec_reset, 0220, NULL, typec_reset_store);

static void glink_usb_notify_uevent(struct usb_chip *chip, int event)
{
	char uevent_buf[GLINK_SHOW_MAX_SIZE];

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			pr_err("No battery supply found\n");
			return;
		}
	}

	memset(uevent_buf, '\0', GLINK_SHOW_MAX_SIZE);
	if (event == NOTIFY_EVENT_USB_LPD_STATUS) {
		scnprintf(uevent_buf, GLINK_SHOW_MAX_SIZE,
			"POWER_SUPPLY_LPD_PRESENT=%s",
			chip->usb_info.lpd_st? "true" : "false");
	} else if (event == NOTIFY_EVENT_USB_CID_STATUS) {
		scnprintf(uevent_buf, GLINK_SHOW_MAX_SIZE,
			"POWER_SUPPLY_CID_STATUS=%d",
			chip->usb_info.cid_st);
	} else {
		pr_err("Invalid usb notify event: %d\n", event);
		return;
	}
	chip->uenvp[0] = uevent_buf;
	chip->uenvp[1] = NULL;
	kobject_uevent_env(&chip->batt_psy->dev.kobj, KOBJ_CHANGE, chip->uenvp);
}

static void glink_usb_work(struct work_struct *work)
{
	int rc;
	struct usb_info usb_info = {0};
	static bool lpd_ulog_triggered = false;
	static bool otg_ulog_triggered = false;
	struct usb_chip *chip = container_of(work,
				struct usb_chip, usb_work);

	if (!chip || !chip->dev) {
		pr_err("Invalid usb chip\n");
		return;
	}

	usb_info = chip->usb_info;
	rc = chip->dev->ops.get_property(chip->dev,
				GLINK_PROP_USB_INFO,
				&usb_info, sizeof(usb_info));
	if (rc) {
		pr_err("Failed to read usb info, rc=%d\n", rc);
		return;
	}

	if ((chip->usb_info.cid_st != -1 && usb_info.cid_st == -1) ||
            (!chip->usb_info.lpd_st && usb_info.lpd_st)) {
		if (!lpd_ulog_triggered && !otg_ulog_triggered)
			bm_ulog_enable_log(true);
		lpd_ulog_triggered = true;
		pr_err("LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
			" dp=%d, dm=%d\n",
			usb_info.lpd_st,
			usb_info.lpd_rsbu1,
			usb_info.lpd_rsbu2,
			usb_info.lpd_cc1,
			usb_info.lpd_cc2,
			usb_info.lpd_dp,
			usb_info.lpd_dm);
		pr_err("CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
			" vbus=%d, otg=%d\n",
			usb_info.cid_st,
			usb_info.cc_st,
			usb_info.partner_type,
			usb_info.pd_active,
			usb_info.legacy_cable,
			usb_info.vbus_st,
			usb_info.otg_st);
	} else if ((usb_info.cid_st != -1 && chip->usb_info.cid_st == -1) ||
		   (!usb_info.lpd_st && chip->usb_info.lpd_st)) {
		if (lpd_ulog_triggered && !otg_ulog_triggered)
			bm_ulog_enable_log(false);
		lpd_ulog_triggered = false;
		pr_warn("LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
			" dp=%d, dm=%d\n",
			usb_info.lpd_st,
			usb_info.lpd_rsbu1,
			usb_info.lpd_rsbu2,
			usb_info.lpd_cc1,
			usb_info.lpd_cc2,
			usb_info.lpd_dp,
			usb_info.lpd_dm);
		pr_warn("CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
			" vbus=%d, otg=%d\n",
			usb_info.cid_st,
			usb_info.cc_st,
			usb_info.partner_type,
			usb_info.pd_active,
			usb_info.legacy_cable,
			usb_info.vbus_st,
			usb_info.otg_st);
	} else {
		pr_info("LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
			" dp=%d, dm=%d\n",
			usb_info.lpd_st,
			usb_info.lpd_rsbu1,
			usb_info.lpd_rsbu2,
			usb_info.lpd_cc1,
			usb_info.lpd_cc2,
			usb_info.lpd_dp,
			usb_info.lpd_dm);
		pr_info("CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
			" vbus=%d, otg=%d\n",
			usb_info.cid_st,
			usb_info.cc_st,
			usb_info.partner_type,
			usb_info.pd_active,
			usb_info.legacy_cable,
			usb_info.vbus_st,
			usb_info.otg_st);
	}

	if (usb_info.otg_st && !usb_info.vbus_st) {
		if (!otg_ulog_triggered && !lpd_ulog_triggered)
			bm_ulog_enable_log(true);
		otg_ulog_triggered = true;
		pr_err("OTG: vbus collapse\n");
	} else if (usb_info.otg_st) {
		if (otg_ulog_triggered && !lpd_ulog_triggered)
			bm_ulog_enable_log(false);
		otg_ulog_triggered = false;
	}

	if (usb_info.lpd_st != chip->usb_info.lpd_st) {
		pr_info("LPD status transit: %d -> %d\n",
				chip->usb_info.lpd_st, !!usb_info.lpd_st);
		chip->usb_info.lpd_st = usb_info.lpd_st;
		relay_notifier_fire(BLOCKING, LPD,
				NOTIFY_EVENT_LPD_STATUS,
				(void *)&(usb_info.lpd_st));
		glink_usb_notify_uevent(chip, NOTIFY_EVENT_USB_LPD_STATUS);
	}

	if (usb_info.cid_st != chip->usb_info.cid_st) {
		pr_info("CID status transit: %d -> %d\n",
				chip->usb_info.cid_st, usb_info.cid_st);
		chip->usb_info.cid_st = usb_info.cid_st;
		relay_notifier_fire(BLOCKING, LPD,
				NOTIFY_EVENT_CID_STATUS,
				(void *)&(usb_info.cid_st));
		glink_usb_notify_uevent(chip, NOTIFY_EVENT_USB_CID_STATUS);
	}

	if (usb_info.otg_st != chip->usb_info.otg_st) {
		pr_info("OTG status transit: %d -> %d\n",
				chip->usb_info.otg_st, !!usb_info.otg_st);
		chip->usb_info.otg_st = usb_info.otg_st;
	}
	chip->usb_info = usb_info;
}

static int glink_usb_lpd_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct usb_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("usb chip is not ready\n");
		return -ENODEV;
	}

	rc = of_property_read_u32(dev->node,
					"lpd-mitigate-mode",
					&chip->lpd_mitigate_mode);
	if (rc || chip->lpd_mitigate_mode != LPD_MITIGATE_DISABLE)
		chip->lpd_mitigate_mode = LPD_MITIGATE_SNK;

	rc = dev->ops.set_property(dev, GLINK_PROP_USB_CLPD_MITIGATE_MODE,
					&chip->lpd_mitigate_mode,
					sizeof(chip->lpd_mitigate_mode));
	if (rc) {
		pr_err("Set lpd mitigate mode failed, rc=%d", rc);
		return rc;
	}

	return 0;
}

static int usb_therm_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = 1;
	return 0;
}

static int usb_therm_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	struct usb_chip *chip = cdev->devdata;

	*state = chip->therm_state;
	return 0;
}

static int usb_therm_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	int rc = 0;
	u32 value = 0;
	struct usb_chip *chip = cdev->devdata;
	struct glink_dev *dev = chip->dev;

	if (chip->therm_state == state)
		return 0;

	pr_info("usb thermal state: %lu -> %lu, typec_partner=%d\n",
				chip->therm_state, state,
				chip->usb_info.partner_type);

	if (!state && gpio_is_valid(chip->otp_en_gpio)) {
		gpio_direction_output(chip->otp_en_gpio, 0);
		pr_warn("usb otp is disabled\n");
		udelay(100);
	}

	value = !!state;
	rc = dev->ops.set_property(dev, GLINK_PROP_CHG_DISABLE,
				&value, sizeof(value));
	rc += dev->ops.set_property(dev, GLINK_PROP_CHG_SUSPEND,
					&value, sizeof(value));
	if (!!state &&
	    gpio_is_valid(chip->otp_en_gpio) &&
	    chip->usb_info.partner_type != TYPEC_PARTNER_SNK_TYPEC_DEFAULT) {
		udelay(100);
		gpio_direction_output(chip->otp_en_gpio, 1);
		pr_warn("usb otp is enabled\n");
	}
	chip->therm_state = state;

	return rc;
}

static const struct thermal_cooling_device_ops usb_therm_ops = {
	.get_max_state = usb_therm_get_max_state,
	.get_cur_state = usb_therm_get_cur_state,
	.set_cur_state = usb_therm_set_cur_state,
};

static int glink_usb_therm_init(struct glink_dev *dev,
				struct glink_dev_cfg *cfg)
{
	int rc;
	struct usb_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("usb chip is not ready\n");
		return -ENODEV;
	}

	if (chip->cdev) {
		pr_err("usb therm cooling device has already inited\n");
		return 0;
	}

	chip->therm_state = -EINVAL;
	chip->therm_supported = of_property_read_bool(dev->node,
					"therm-supported");
	if (!chip->therm_supported) {
		pr_warn("usb therm is not supported in devicetree\n");
		return 0;
	}

	if (cfg->factory_version || !cfg->softbank) {
		pr_warn("usb therm is not supported in current version\n");
		return 0;
	}

	chip->otp_en_gpio = of_get_named_gpio(dev->node, "otp-en-gpio", 0);
	if (!gpio_is_valid(chip->otp_en_gpio)) {
		pr_warn("invalid usb otp en gpio=%d\n", chip->otp_en_gpio);
		chip->otp_en_gpio = -EINVAL;
	} else {
		pr_info("usb otp en gpio=%d\n", chip->otp_en_gpio);
		rc = gpio_request(chip->otp_en_gpio, "usb-otp-en");
		if (rc) {
			pr_err("request usb-otp-en gpio=%d failed, rc=%d\n",
					chip->otp_en_gpio, rc);
			return rc;
		}
		gpio_direction_output(chip->otp_en_gpio, 0);
	}

	chip->cdev = thermal_of_cooling_device_register(dev->node,
				"usb_therm_cooler", chip, &usb_therm_ops);

	if (IS_ERR(chip->cdev)) {
		rc = PTR_ERR(chip->cdev);
		chip->cdev = NULL;
		if (gpio_is_valid(chip->otp_en_gpio))
			gpio_free(chip->otp_en_gpio);
		pr_err("Cooling register failed for usb_therm, rc=%d\n", rc);
		return rc;
	}
	pr_info("Cooling register success for usb_therm\n");

	return 0;
}

static int glink_usb_init(struct glink_dev *dev,
					struct glink_dev_cfg *cfg)
{
	int rc;
	struct usb_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("usb chip is not ready\n");
		return -ENODEV;
	}

	if (chip->init_done)
		return 0;

	rc = glink_usb_therm_init(dev, cfg);
	if (rc) {
		pr_err("failed to cfg usb therm\n");
		return rc;
	}

	rc = glink_usb_lpd_init(dev, cfg);
	if (rc) {
		pr_err("failed to cfg usb lpd\n");
		return rc;
	}
	memcpy(&dev->cfg, cfg, sizeof(struct glink_dev_cfg));
	chip->cfg_done = true;

	rc = device_create_file(dev->dev,
				&dev_attr_force_pmic_icl);
	if (rc) {
		pr_err("Couldn't create force_pmic_icl\n");
	}

	rc = device_create_file(dev->dev,
				&dev_attr_force_usb_suspend);
	if (rc) {
		pr_err("Couldn't create force_usb_suspend\n");
	}

	rc = device_create_file(dev->dev,
				&dev_attr_cid_status);
	if (rc) {
		pr_err("Couldn't create cid_status\n");
	}

	rc = device_create_file(dev->dev,
				&dev_attr_typec_reset);
	if (rc) {
		pr_err("Couldn't create typec_reset\n");
	}

	chip->init_done = true;
	pr_info("glink %s init done\n", dev->name);
	return 0;
}

static int glink_usb_deinit(struct glink_dev *dev)
{
	struct usb_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("usb chip is not ready\n");
		return -ENODEV;
	}

	if (chip->cdev) {
		thermal_cooling_device_unregister(chip->cdev);
		if (gpio_is_valid(chip->otp_en_gpio))
			gpio_free(chip->otp_en_gpio);
	}

	device_remove_file(dev->dev, &dev_attr_typec_reset);
	device_remove_file(dev->dev, &dev_attr_cid_status);
	device_remove_file(dev->dev, &dev_attr_force_pmic_icl);
	device_remove_file(dev->dev, &dev_attr_force_usb_suspend);
	if (chip->batt_psy) {
		power_supply_put(chip->batt_psy);
		chip->batt_psy = NULL;
	}
	devm_kfree(dev->dev, chip);
	dev->devdata = NULL;
	this_chip = NULL;

	return 0;
}

static int glink_usb_notify(struct glink_dev *dev,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	struct usb_chip *chip = dev->devdata;

	if (!chip) {
		pr_err("glink usb is not ready\n");
		return -ENODEV;
	}

	switch (notify_data->receiver) {
	case GLINK_NOTIFY_RECEIVER_LINK_USR:
		switch (notification) {
		case MMI_GLINK_STATE_UP:
			if (chip->init_done)
				glink_usb_lpd_init(dev, &dev->cfg);
			break;
		case MMI_GLINK_STATE_DOWN:
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_PSY_USR:
		switch (notification) {
		case MMI_PSY_CHANGE_USB:
			schedule_work(&chip->usb_work);
			break;
		default:
			break;
		}
		break;
	case GLINK_NOTIFY_RECEIVER_POLL_TASK:
		schedule_work(&chip->usb_work);
		break;
	default:
		pr_debug("Skip receiver: %#x\n", notify_data->receiver);
		break;
	}

	return 0;
}

int glink_device_usb_setup(struct glink_dev *dev)
{
	struct usb_chip *chip = dev->devdata;

	if (chip) {
		pr_warn("glink %s has already setup\n", dev->name);
		return 0;
	}

	dev->ops.init = glink_usb_init;
	dev->ops.notify	= glink_usb_notify;
	dev->ops.deinit = glink_usb_deinit;

	chip = devm_kzalloc(dev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->usb_info.cid_st = -1;
	chip->usb_info.lpd_st = -1;
	chip->usb_info.otg_st = -1;
	INIT_WORK(&chip->usb_work, glink_usb_work);
	dev->devdata = chip;
	chip->dev = dev;
	this_chip = chip;

	pr_info("glink %s setup done\n", dev->name);
	return 0;
}
