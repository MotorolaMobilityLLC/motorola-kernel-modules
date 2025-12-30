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

#include <linux/power_supply.h>
#include <linux/of.h>
#include "../mmi_glink_core.h"
#include <linux/power/qti_glink_charger_v2.h>
#include "../device_class.h"
#include "battery_glink.h"
#include "usb_glink.h"
#include <linux/power/bm_adsp_ulog.h>
#include <linux/thermal.h>
#include <linux/mmi_relay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define ULOG_DURATION_MS	60000
#define LPD_FLAG_MAX 0x40

static struct usb_glink_dev *this_chip = NULL;
static int glink_usb_init(struct usb_glink_dev *chip);
static int glink_usb_lpd_init(struct usb_glink_dev *chip);

static ssize_t cid_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct usb_glink_dev *chip = this_chip;

	if (!chip) {
		mmi_err(chip->mmi_chip, "usb chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
			chip->usb_info.cid_st);
}
static DEVICE_ATTR(cid_status, 0444, cid_status_show, NULL);

static ssize_t typec_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned int reset = 0;
	struct usb_glink_dev *chip = this_chip;

	if (!chip) {
		mmi_err(chip->mmi_chip, "usb chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &reset);
	if (rc) {
		mmi_err(chip->mmi_chip, "Invalid typec_reset = %d\n", reset);
		return -EINVAL;
	}

	mmi_warn(chip->mmi_chip, "typec_reset triggered:%d\n", reset);

	rc = qti_charger_set_property(OEM_PROP_TYPEC_RESET,
			&reset,
			sizeof(reset));

	return rc ? rc : count;
}
static DEVICE_ATTR(typec_reset, 0220, NULL, typec_reset_store);

static ssize_t typec_pwrsrc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned int current_ma = 0;
	struct usb_glink_dev *chip = this_chip;

	if (!chip) {
		mmi_err(chip->mmi_chip, "usb chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &current_ma);
	if (rc) {
		mmi_err(chip->mmi_chip, "Invalid pwrsrc, rc=%d\n", rc);
		return -EINVAL;
	}

	mmi_info(chip->mmi_chip, "pwrsrc current = %d\n", current_ma);

	rc = qti_charger_set_property(OEM_PROP_TYPEC_PWRSRC_REQUEST,
			&current_ma,
			sizeof(current_ma));

	return rc ? rc : count;
}
static DEVICE_ATTR(typec_pwrsrc, 0220, NULL, typec_pwrsrc_store);

static void glink_usb_notify_uevent(struct usb_glink_dev *chip, int event)
{
	char uevent_buf[CHG_SHOW_MAX_SIZE];

	if (!chip->batt_psy) {
		glink_usb_init(chip);
		if (!chip->batt_psy) {
			mmi_err(chip->mmi_chip, "No battery supply found\n");
			return;
		}
	}

	memset(uevent_buf, '\0', CHG_SHOW_MAX_SIZE);
	if (event == NOTIFY_EVENT_USB_LPD_STATUS) {
		scnprintf(uevent_buf, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_LPD_PRESENT=%s",
				chip->usb_info.lpd_st? "true" : "false");
	} else if (event == NOTIFY_EVENT_USB_LPD_FLAG) {
		scnprintf(uevent_buf, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_LPD_STATUS=%d",
				chip->usb_info.lpd_flag);
	} else if (event == NOTIFY_EVENT_USB_CID_STATUS) {
		scnprintf(uevent_buf, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_CID_STATUS=%d",
				chip->usb_info.cid_st);
	} else {
		mmi_err(chip->mmi_chip, "Invalid usb notify event: %d\n", event);
		return;
	}
	chip->uenvp[0] = uevent_buf;
	chip->uenvp[1] = NULL;
	kobject_uevent_env(&chip->batt_psy->dev.kobj, KOBJ_CHANGE, chip->uenvp);
}

static bool glink_usb_check_usb_info(struct usb_glink_dev *chip, struct usb_info *usb_info)
{
	if (!chip || !usb_info) {
		pr_err("Invalid usb info\n");
		return false;
	}

	if ((abs(usb_info->cid_st) > 1)
		|| (abs(usb_info->lpd_st) > 1)
		|| (abs(usb_info->pd_active) > 1)
		|| (abs(usb_info->legacy_cable) > 1)) {
		mmi_err(chip->mmi_chip, "usb_info data illegal!\n");
		return false;
	}

	return true;
}

#define VBUS_MIN_MV			4000
static void glink_usb_work(struct work_struct *work)
{
	int rc;
	struct usb_info usb_info = {0};
	static bool lpd_ulog_triggered = false;
	static bool otg_ulog_triggered = false;
	struct usb_glink_dev *chip = container_of(work,
			struct usb_glink_dev, usb_work.work);

	if (!chip) {
		mmi_err(chip->mmi_chip, "Invalid usb chip\n");
		return;
	}
	if (!chip->init_lpd_done) {
		rc = glink_usb_lpd_init(chip);
		if (rc) {
			mmi_err(chip->mmi_chip, "failed to init usb lpd\n");
		} else {
			chip->init_lpd_done = true;
		}
	}

	usb_info = chip->usb_info;
	rc = qti_charger_get_property(OEM_PROP_USB_INFO,
			&usb_info, sizeof(usb_info));
	if (rc || !glink_usb_check_usb_info(chip, &usb_info)) {
		mmi_err(chip->mmi_chip, "Failed to read usb info, rc=%d\n", rc);
		return;
	}

	if ((chip->usb_info.cid_st != -1 && usb_info.cid_st == -1) ||
			(!chip->usb_info.lpd_st && usb_info.lpd_st)) {
		if (!lpd_ulog_triggered && !otg_ulog_triggered)
			bm_ulog_enable_log(true, ULOG_DURATION_MS);
		lpd_ulog_triggered = true;
		mmi_err(chip->mmi_chip, "LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
				" dp=%d, dm=%d, stat=%#x\n",
				usb_info.lpd_st,
				usb_info.lpd_rsbu1,
				usb_info.lpd_rsbu2,
				usb_info.lpd_cc1,
				usb_info.lpd_cc2,
				usb_info.lpd_dp,
				usb_info.lpd_dm,
				usb_info.lpd_flag);
		mmi_err(chip->mmi_chip, "CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
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
			bm_ulog_enable_log(false, 0);
		lpd_ulog_triggered = false;
		mmi_warn(chip->mmi_chip, "LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
				" dp=%d, dm=%d, stat=%#x\n",
				usb_info.lpd_st,
				usb_info.lpd_rsbu1,
				usb_info.lpd_rsbu2,
				usb_info.lpd_cc1,
				usb_info.lpd_cc2,
				usb_info.lpd_dp,
				usb_info.lpd_dm,
				usb_info.lpd_flag);
		mmi_warn(chip->mmi_chip, "CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
				" vbus=%d, otg=%d\n",
				usb_info.cid_st,
				usb_info.cc_st,
				usb_info.partner_type,
				usb_info.pd_active,
				usb_info.legacy_cable,
				usb_info.vbus_st,
				usb_info.otg_st);
	} else {
		mmi_info(chip->mmi_chip, "LPD: present=%d, rsbu1=%d, rsbu2=%d, cc1=%d, cc2=%d,"
				" dp=%d, dm=%d, stat=%#x\n",
				usb_info.lpd_st,
				usb_info.lpd_rsbu1,
				usb_info.lpd_rsbu2,
				usb_info.lpd_cc1,
				usb_info.lpd_cc2,
				usb_info.lpd_dp,
				usb_info.lpd_dm,
				usb_info.lpd_flag);
		mmi_info(chip->mmi_chip, "CID: present=%d, conn=%d, ptype=%d, pd=%d, legacy=%d,"
				" vbus=%d, otg=%d\n",
				usb_info.cid_st,
				usb_info.cc_st,
				usb_info.partner_type,
				usb_info.pd_active,
				usb_info.legacy_cable,
				usb_info.vbus_st,
				usb_info.otg_st);
	}

	if (usb_info.otg_st && usb_info.vbus_st < VBUS_MIN_MV) {
		if (!otg_ulog_triggered && !lpd_ulog_triggered)
			bm_ulog_enable_log(true, ULOG_DURATION_MS);
		otg_ulog_triggered = true;
		mmi_err(chip->mmi_chip, "OTG: vbus collapse\n");
	} else if (usb_info.otg_st) {
		if (otg_ulog_triggered && !lpd_ulog_triggered)
			bm_ulog_enable_log(false, 0);
		otg_ulog_triggered = false;
	}

	if (usb_info.lpd_st != chip->usb_info.lpd_st) {
		mmi_info(chip->mmi_chip, "LPD status transit: %d -> %d\n",
				chip->usb_info.lpd_st, !!usb_info.lpd_st);
		chip->usb_info.lpd_st = usb_info.lpd_st;
//		relay_notifier_fire(BLOCKING, LPD,
//				NOTIFY_EVENT_LPD_STATUS,
//				(void *)&(usb_info.lpd_st));
		glink_usb_notify_uevent(chip, NOTIFY_EVENT_USB_LPD_STATUS);
		if (usb_info.lpd_flag > 0 && usb_info.lpd_flag < LPD_FLAG_MAX) {
		    chip->usb_info.lpd_flag = usb_info.lpd_flag;
			glink_usb_notify_uevent(chip, NOTIFY_EVENT_USB_LPD_FLAG);
		}
	}

	if (usb_info.cid_st != chip->usb_info.cid_st) {
		mmi_info(chip->mmi_chip, "CID status transit: %d -> %d\n",
				chip->usb_info.cid_st, usb_info.cid_st);
		chip->usb_info.cid_st = usb_info.cid_st;
//		relay_notifier_fire(BLOCKING, LPD,
//				NOTIFY_EVENT_CID_STATUS,
//				(void *)&(usb_info.cid_st));
		glink_usb_notify_uevent(chip, NOTIFY_EVENT_USB_CID_STATUS);
	}

	if (usb_info.otg_st != chip->usb_info.otg_st) {
		mmi_info(chip->mmi_chip, "OTG status transit: %d -> %d\n",
				chip->usb_info.otg_st, !!usb_info.otg_st);
		chip->usb_info.otg_st = usb_info.otg_st;
	}
	chip->usb_info = usb_info;
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
	struct usb_glink_dev *chip = cdev->devdata;

	*state = chip->therm_state;
	return 0;
}

static int usb_therm_set_cur_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	int rc = 0;
	u32 value = 0;
	struct usb_glink_dev *chip = cdev->devdata;

	if (chip->therm_state == state)
		return 0;

	mmi_info(chip->mmi_chip, "usb thermal state: %lu -> %lu, typec_partner=%d\n",
			chip->therm_state, state,
			chip->usb_info.partner_type);

	if (!state && gpio_is_valid(chip->otp_en_gpio)) {
		gpio_direction_output(chip->otp_en_gpio, 0);
		mmi_warn(chip->mmi_chip, "usb otp is disabled\n");
		udelay(100);
	}

	value = !!state;
	rc = qti_charger_set_property(OEM_PROP_CHG_DISABLE,
			&value, sizeof(value));
	rc += qti_charger_set_property(OEM_PROP_CHG_SUSPEND,
			&value, sizeof(value));
	if (!!state &&
			gpio_is_valid(chip->otp_en_gpio) &&
			chip->usb_info.partner_type != TYPEC_PARTNER_SNK_TYPEC_DEFAULT) {
		udelay(100);
		gpio_direction_output(chip->otp_en_gpio, 1);
		mmi_warn(chip->mmi_chip, "usb otp is enabled\n");
	}
	chip->therm_state = state;

	return rc;
}

static const struct thermal_cooling_device_ops usb_therm_ops = {
	.get_max_state = usb_therm_get_max_state,
	.get_cur_state = usb_therm_get_cur_state,
	.set_cur_state = usb_therm_set_cur_state,
};

static int glink_usb_therm_init(struct usb_glink_dev *chip)
{
	int rc;

	struct device_node *node, *child;
	const char *temp_string;

	if (!chip || !chip->mmi_chip) {
		mmi_err(chip->mmi_chip, "usb chip is not ready\n");
		return -ENODEV;
	}

	node = chip->mmi_chip->dev->of_node;
	for_each_child_of_node(node, child) {
		rc = of_property_read_string(child, "psy-name", &temp_string);
		if (rc < 0) {
			mmi_err(chip->mmi_chip, "Failed to read psy-name\n");
			return rc;
		}
		if (!strcmp(temp_string, "usb_info")) {
			chip->therm_state = -EINVAL;
			chip->therm_supported = of_property_read_bool(child,
					"therm-supported");
			if (!chip->therm_supported) {
				mmi_warn(chip->mmi_chip, "usb therm is not supported in devicetree\n");
				return 0;
			}

			if (chip->mmi_chip->factory_version || !chip->mmi_chip->is_softbank) {
				mmi_warn(chip->mmi_chip, "usb therm is not supported in current version\n");
				return 0;
			}

			chip->otp_en_gpio = of_get_named_gpio(child, "otp-en-gpio", 0);
			if (!gpio_is_valid(chip->otp_en_gpio)) {
				mmi_warn(chip->mmi_chip, "invalid usb otp en gpio=%d\n", chip->otp_en_gpio);
				chip->otp_en_gpio = -EINVAL;
			} else {
				mmi_info(chip->mmi_chip, "usb otp en gpio=%d\n", chip->otp_en_gpio);
				rc = gpio_request(chip->otp_en_gpio, "usb-otp-en");
				if (rc) {
					mmi_err(chip->mmi_chip, "request usb-otp-en gpio=%d failed, rc=%d\n",
							chip->otp_en_gpio, rc);
					return rc;
				}
				gpio_direction_output(chip->otp_en_gpio, 0);
			}

			chip->cdev = thermal_of_cooling_device_register(chip->mmi_chip->dev->of_node,
					"usb_therm_cooler", chip, &usb_therm_ops);
			if (IS_ERR(chip->cdev)) {
				rc = PTR_ERR(chip->cdev);
				chip->cdev = NULL;
				if (gpio_is_valid(chip->otp_en_gpio))
					gpio_free(chip->otp_en_gpio);
				mmi_err(chip->mmi_chip, "Cooling register failed for usb_therm, rc=%d\n", rc);
				return rc;
			}
			mmi_info(chip->mmi_chip, "Cooling register success for usb_therm\n");
			break;
		}
	}

	return 0;
}

static int glink_usb_lpd_init(struct usb_glink_dev *chip)
{
	int rc;
	struct device_node *node, *child;
	const char *temp_string;

	if (!chip || !chip->mmi_chip) {
		mmi_err(chip->mmi_chip, "usb chip is not ready\n");
		return -ENODEV;
	}

	node = chip->mmi_chip->dev->of_node;
	for_each_child_of_node(node, child) {
		rc = of_property_read_string(child, "psy-name", &temp_string);
		if (rc < 0) {
			mmi_err(chip->mmi_chip, "Failed to read psy-name\n");
			return rc;
		}
		if (!strcmp(temp_string, "usb_info")) {
			rc = of_property_read_u32(child,
					"lpd-mitigate-mode",
					&chip->lpd_mitigate_mode);
			if (rc || chip->lpd_mitigate_mode != LPD_MITIGATE_DISABLE)
				chip->lpd_mitigate_mode = LPD_MITIGATE_SNK;

			rc = qti_charger_set_property(OEM_PROP_LPD_MITIGATE_MODE,
					&chip->lpd_mitigate_mode,
					sizeof(chip->lpd_mitigate_mode));
			if (rc) {
				mmi_err(chip->mmi_chip, "Set lpd mitigate mode failed, rc=%d", rc);
				return rc;
			}
			mmi_info(chip->mmi_chip, "Init lpd mitigate mode done");
			break;
		}
	}

	return 0;
}

static int glink_usb_psy_init(struct usb_glink_dev *chip)
{
	int rc;
	if (chip->usb_psy)
		return 0;

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		mmi_err(chip->mmi_chip, "No usb power supply found\n");
		return -ENODEV;
	}

	rc = device_create_file(chip->usb_psy->dev.parent,
			&dev_attr_cid_status);
	if (rc) {
		mmi_err(chip->mmi_chip, "Couldn't create cid_status\n");
	}

	rc = device_create_file(chip->usb_psy->dev.parent,
			&dev_attr_typec_reset);
	if (rc) {
		mmi_err(chip->mmi_chip, "Couldn't create typec_reset\n");
	}

	rc = device_create_file(chip->usb_psy->dev.parent,
			&dev_attr_typec_pwrsrc);
	if (rc) {
		mmi_err(chip->mmi_chip, "Couldn't create typec_pwrsrc\n");
	}
	return rc;
}

static int glink_usb_init(struct usb_glink_dev *chip)
{
	int rc;

	if (!chip || !chip->mmi_chip) {
		mmi_err(chip->mmi_chip, "usb chip is not ready\n");
		return -ENODEV;
	}

	if (chip->batt_psy)
		return 0;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		mmi_err(chip->mmi_chip, "No batt power supply found\n");
		return -ENODEV;
	}

	rc = glink_usb_therm_init(chip);
	if (rc) {
		mmi_err(chip->mmi_chip, "failed to init usb therm\n");
		return rc;
	}

	if (!chip->init_lpd_done) {
		rc = glink_usb_lpd_init(chip);
		if (rc) {
			mmi_err(chip->mmi_chip, "failed to init usb lpd\n");
		} else {
			chip->init_lpd_done = true;
		}
	}

	glink_usb_psy_init(chip);
	chip->init_done = true;
	mmi_info(chip->mmi_chip, "glink %s init done\n", chip->name);
	return 0;
}

static int glink_usb_deinit(struct usb_glink_dev *chip)
{
	if (!chip) {
		mmi_err(chip->mmi_chip, "usb chip is not ready\n");
		return -ENODEV;
	}

	if (chip->cdev) {
		thermal_cooling_device_unregister(chip->cdev);
		if (gpio_is_valid(chip->otp_en_gpio))
			gpio_free(chip->otp_en_gpio);
	}

	device_remove_file(chip->mmi_chip->dev, &dev_attr_typec_pwrsrc);
	device_remove_file(chip->mmi_chip->dev, &dev_attr_typec_reset);
	device_remove_file(chip->mmi_chip->dev, &dev_attr_cid_status);
	if (chip->batt_psy) {
		power_supply_put(chip->batt_psy);
		chip->batt_psy = NULL;
	}

	return 0;
}

static int glink_usb_notify(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct usb_glink_dev *chip =
		container_of(nb, struct usb_glink_dev, usb_nb);

	struct power_supply *psy = data;

	if (!chip) {
		mmi_err(chip->mmi_chip, "called before usb_info valid!\n");
		return NOTIFY_DONE;
	}

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy && !chip->batt_psy &&
	    (strcmp(psy->desc->name, "battery") == 0)) {
		mmi_info(chip->mmi_chip, "init usb glink again!\n");
		glink_usb_init(chip);
	}

	if (psy && strcmp(psy->desc->name, "usb") == 0) {
		if(!chip->usb_psy) {
			mmi_info(chip->mmi_chip, "init usb psy again!\n");
			glink_usb_psy_init(chip);
		}
		schedule_delayed_work(&chip->usb_work,
			      msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

struct glink_device *usb_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{
	struct usb_glink_dev *usb_chip = NULL;
	struct glink_device * glink_dev = NULL;
	int rc = 0;

	if (!chip)
		goto exit;

	usb_chip = kzalloc(sizeof(struct usb_glink_dev),GFP_KERNEL);
	if (!usb_chip)
		goto exit;

	usb_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_USB, usb_chip);
	if (!glink_dev)
		goto exit;

	usb_chip->dev = glink_dev;
	usb_chip->mmi_chip = chip;
	usb_chip->usb_nb.notifier_call = glink_usb_notify;
	rc = power_supply_reg_notifier(&usb_chip->usb_nb);
	if (rc) {
		mmi_err(chip, "Failed to register usb_psy_notifier: %d\n", rc);
		goto exit;
	}

	usb_chip->usb_info.cid_st = -1;
	usb_chip->usb_info.lpd_st = -1;
	usb_chip->usb_info.otg_st = -1;
	INIT_DELAYED_WORK(&usb_chip->usb_work, glink_usb_work);
	glink_usb_init(usb_chip);
	this_chip = usb_chip;

	mmi_info(chip, "glink %s setup done\n", usb_chip->name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void usb_glink_device_unregister(void)
{
	if(!this_chip)
		return;
	mmi_err(this_chip->mmi_chip, "wireless_glink_device_unregister");
	cancel_delayed_work(&this_chip->usb_work);
	if (this_chip->init_done)
		glink_usb_deinit(this_chip);
	power_supply_unreg_notifier(&this_chip->usb_nb);
	glink_device_unregister(this_chip->dev);
	kfree(this_chip);
	this_chip = NULL;
}
