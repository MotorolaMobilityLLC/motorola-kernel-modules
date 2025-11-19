/*
 * Copyright (C) 2025 Motorola Mobility LLC
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

#define pr_fmt(fmt)     "POGO_PEN_CHG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/pen_detection_notify.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#define CHG_SHOW_MAX_SIZE 50
#define CHG_START_DELAY_S 1

enum charge_status {
	PEN_STAT_NOT_CHARGING,
	PEN_STAT_CHARGING_BY_ADAPTER,
	PEN_STAT_CHARGING_BY_INSERT_PEN,
	PEN_STAT_CHARGING_BY_PEN_AFTER_DETECT_CURRENT,
	PEN_STAT_CHARGING_BY_INSERT_FULL_SOC_PEN,
};

struct pen_charger {
	struct device		*dev;
	int chg_ldswtch_en_gpio;
	int chg_boost_en_gpio;
	struct iio_channel *pen_sns_chan;
	int r_sns_milliohm;
	int iterm_ma;
	int chg_buffer_s;
	int iterm_ma_insert_pen;
	int chg_buffer_insert_pen_s;
	int heartbeat_interval_s;
	bool chg_to_iterm;
	bool chg_is_present;
	bool initialized;
	bool pen_insert_flag;
	enum charge_status charging_status;
	struct notifier_block pen_notif;
	struct notifier_block chg_psy_nb;
	struct alarm chg_tmr;
	struct task_struct *chg_task;
	wait_queue_head_t chg_waitq;
	bool chg_thread_trigger;
	struct wakeup_source *charger_wakelock;
};

static void pen_charger_handle_event(struct pen_charger *chg, bool charing) {
	char *event_string = NULL;
	char *batt_uenvp[2];
	int ret;

	event_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!event_string)
		return;

	scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_CHARGING=%s", charing? "true": "false");

	batt_uenvp[0] = event_string;
	batt_uenvp[1] = NULL;
	ret = kobject_uevent_env(&chg->dev->kobj, KOBJ_CHANGE, batt_uenvp);
	if (ret)
		pr_err("kobject_uevent_fail, ret=%d",ret);
	kfree(event_string);

	return;
}

static int pen_charger_parse_dt(struct pen_charger *chg)
{
	struct device_node *node = chg->dev->of_node;
	int rc = 0;

	chg->chg_ldswtch_en_gpio = of_get_named_gpio(node, "mmi,chg-ldswtch-en-gpio", 0);
	if(!gpio_is_valid(chg->chg_ldswtch_en_gpio)) {
		pr_err("chg->chg_ldswtch_en_gpio is %d invalid\n", chg->chg_ldswtch_en_gpio);
		return -ENODEV;
	}
	chg->chg_boost_en_gpio = of_get_named_gpio(node, "mmi,chg-boost-en-gpio", 0);
	if(!gpio_is_valid(chg->chg_boost_en_gpio)) {
		pr_err("chg->chg_boost_en_gpio is %d invalid\n", chg->chg_boost_en_gpio);
		return -ENODEV;
	}
	rc = of_property_read_u32(node, "mmi,r-sns-milliohm", &chg->r_sns_milliohm);
	if (rc)
		chg->r_sns_milliohm = 1000;

	pr_info("ldswtch_en_gpio: %d, boost_en_gpio: %d,r_sns: %d\n",
		chg->chg_ldswtch_en_gpio, chg->chg_boost_en_gpio, chg->r_sns_milliohm);

	rc = of_property_read_u32(node, "mmi,charge-iterm-ma", &chg->iterm_ma);
	if (rc)
		chg->iterm_ma = 5;
	rc = of_property_read_u32(node, "mmi,charge-buffer-s", &chg->chg_buffer_s);
	if (rc)
		chg->chg_buffer_s = 7 * 60;
	rc = of_property_read_u32(node, "mmi,charge-iterm-ma-insert-pen", &chg->iterm_ma_insert_pen);
	if (rc)
		chg->iterm_ma_insert_pen = 6;
	rc = of_property_read_u32(node, "mmi,charge-buffer-insert-pen-s", &chg->chg_buffer_insert_pen_s);
	if (rc)
		chg->chg_buffer_insert_pen_s = 60;

	rc = of_property_read_u32(node, "mmi,heartbeat-interval-s", &chg->heartbeat_interval_s);
	if (rc)
		chg->heartbeat_interval_s = 2 * 60;

	pr_info("iterm_ma(normal/insert): %d,%d, buffer_s(normal/insert): %d,%d, chg->heartbeat_interval_s: %d\n",
		chg->iterm_ma, chg->iterm_ma_insert_pen, chg->chg_buffer_s,
		chg->chg_buffer_insert_pen_s ,chg->heartbeat_interval_s);

	return 0;
}

static ssize_t pen_chg_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pen_charger *this_chip = dev_get_drvdata(dev);
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("pen_chg_enable_store: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("pen_chg_enable_store: Invalid charger suspend value = %lu\n", mode);
		return -EINVAL;
	}

	pr_info("pen_chg_enable_store: enable = %lu\n", mode);
	if (!!mode) {
		gpio_set_value(this_chip->chg_boost_en_gpio, true);
		gpio_set_value(this_chip->chg_ldswtch_en_gpio, true);
	} else {
		gpio_set_value(this_chip->chg_boost_en_gpio, false);
		gpio_set_value(this_chip->chg_ldswtch_en_gpio, false);
	}

	return count;
}

#define MAX_TRY_CNT  10
static int read_pen_chg_current(struct pen_charger *this_chip)
{
	int rc;
	int pen_sns_uv = 0;
	int pen_current_ma = 0;
	int i;

	if (!this_chip) {
		pr_err("read_pen_chg_current: chip is invalid\n");
		return 0;
	}

	for (  i = 0; i < MAX_TRY_CNT; i++)
	{
		rc = iio_read_channel_processed(this_chip->pen_sns_chan, &pen_sns_uv);
		if (rc < 0)
			msleep(30);
		else
		 break;
	}
	if (rc < 0) {
		pr_err("Error Reading pen_sns_uv return 0\n");
		return 0;
	}

	pen_current_ma = pen_sns_uv / this_chip->r_sns_milliohm;

	pr_info("Read pen_sns_uv:%d,pen_current_ma:%d\n", pen_sns_uv, pen_current_ma);
	return pen_current_ma;
}

static DEVICE_ATTR(pen_chg_enable, 0200,
		NULL,
		pen_chg_enable_store);

static ssize_t pen_chg_current_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct pen_charger *this_chip = dev_get_drvdata(dev);
	int pen_current_ma = 0;

    if (!this_chip) {
		pr_err("pen_chg_current_show: chip is invalid\n");
		return -ENODEV;
	}
	pen_current_ma = read_pen_chg_current(this_chip);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", pen_current_ma);
}
static DEVICE_ATTR(pen_chg_current, 0444,
		pen_chg_current_show,
		NULL);

static void start_charge(struct pen_charger *chg, bool en)
{
	pr_info("start_charge en = %d\n", en);
	gpio_set_value(chg->chg_ldswtch_en_gpio, en);
	gpio_set_value(chg->chg_boost_en_gpio, en);


	pr_info("chg->chg_ldswtch_en_gpio  = %d\n", gpio_get_value(chg->chg_ldswtch_en_gpio));
	pr_info("chg->chg_boost_en_gpio  = %d\n", gpio_get_value(chg->chg_boost_en_gpio));
}

static void start_chg_timer(struct pen_charger *chg,
					 bool en, int time_s)
{
	int ret;

	pr_info("start_chg_timer en = %d, time_s = %d\n", en, time_s);
	if (en) {
		ret = alarm_try_to_cancel(&chg->chg_tmr);
		if (ret < 0) {
			pr_err("chg_timer was running, skip timer\n");
			return;
		}
		alarm_start_relative(&chg->chg_tmr,
				     ms_to_ktime(time_s * 1000));
	}
	else
		alarm_cancel(&chg->chg_tmr);
}

static enum alarmtimer_restart chg_tmr_handler(struct alarm *alarm, ktime_t now)
{
	struct pen_charger *chg = container_of(alarm,
						      struct pen_charger,
						      chg_tmr);
	if (!chg) {
		pr_err("chg_tmr_handler: chip is invalid\n");
		return ALARMTIMER_NORESTART;
	}

	chg->chg_thread_trigger = true;
	wake_up_interruptible(&chg->chg_waitq);

	return ALARMTIMER_NORESTART;
}

static int chg_thread_func(void *data) {
    struct pen_charger *chg = data;
    int pen_current_ma;
    int iterm_ma;
    int charge_iterm_s;

    while (1) {
        wait_event_interruptible(chg->chg_waitq,
            chg->chg_thread_trigger || kthread_should_stop());
        if (kthread_should_stop()) {
            break;
		}
		__pm_stay_awake(chg->charger_wakelock);
        chg->chg_thread_trigger = false;
		if (!chg->pen_insert_flag)
			goto exit;

		pen_current_ma = read_pen_chg_current(chg);
		if (chg->charging_status == PEN_STAT_CHARGING_BY_INSERT_PEN) {
			chg->charging_status = PEN_STAT_CHARGING_BY_PEN_AFTER_DETECT_CURRENT;
			if (pen_current_ma < chg->iterm_ma_insert_pen)
				chg->charging_status = PEN_STAT_CHARGING_BY_INSERT_FULL_SOC_PEN;
		}
		if (chg->charging_status == PEN_STAT_CHARGING_BY_INSERT_FULL_SOC_PEN) {
			iterm_ma = chg->iterm_ma_insert_pen;
			charge_iterm_s = chg->chg_buffer_insert_pen_s;
		} else {
			iterm_ma = chg->iterm_ma;
			charge_iterm_s = chg->chg_buffer_s;
		}


		if (pen_current_ma < iterm_ma) {
			if (!chg->chg_to_iterm) {
				pr_info("charge to iterm and charge buff time:%d \n",charge_iterm_s);
				chg->chg_to_iterm = true;
				start_chg_timer(chg, true, charge_iterm_s);
			} else {
				pr_info("charge Done\n");
				chg->charging_status = PEN_STAT_NOT_CHARGING;
				chg->chg_to_iterm = false;
				start_charge(chg, false);
				start_chg_timer(chg, false, 0);
				pen_charger_handle_event(chg, false);
			}
		} else {
			pr_info("charge to heartbeat_interval_s time:%d \n",chg->heartbeat_interval_s);
			chg->chg_to_iterm = false;
			start_chg_timer(chg, true, chg->heartbeat_interval_s);
		}

	exit:
		__pm_relax(chg->charger_wakelock);
	}

	 return 0;
}

static int pen_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct pen_charger *chg = container_of(self,
		struct pen_charger, pen_notif);

	if (!chg) {
		pr_err("pen_notifier_callback: chg not valid!\n");
		return NOTIFY_DONE;
	}

	if (!chg->initialized) {
		pr_err("Still not initialized\n");
		return NOTIFY_DONE;
	}

	pr_info("Received event(%lu) for pen detection\n", event);

	if (event == PEN_DETECTION_INSERT) {
		chg->pen_insert_flag = true;
		chg->charging_status = PEN_STAT_CHARGING_BY_INSERT_PEN;
		start_charge(chg, true);
		start_chg_timer(chg, true, CHG_START_DELAY_S);
		pen_charger_handle_event(chg, true);
	} else if (event == PEN_DETECTION_PULL) {
		chg->pen_insert_flag = false;
		chg->chg_to_iterm = false;
		chg->charging_status = PEN_STAT_NOT_CHARGING;
		start_chg_timer(chg, false, 0);
		start_charge(chg, false);
		pen_charger_handle_event(chg, false);
	}

	return NOTIFY_OK;
}

static int charger_psy_notify_callback(struct notifier_block *nb,
				unsigned long event, void *v)
{
	struct pen_charger *chg = container_of(nb,
				struct pen_charger, chg_psy_nb);
	struct power_supply *psy = v;
	union power_supply_propval pval;
	bool usb_present = false;
	bool wls_present = false;
	int retval;

	if (!chg) {
		pr_err("called before pen charger valid!\n");
		return NOTIFY_DONE;
	}
	if (!chg->initialized) {
		pr_err("Still not initialized\n");
		return NOTIFY_DONE;
	}
	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	if (psy && psy->desc->get_property && psy->desc->name &&
		(!strcmp(psy->desc->name, "usb") || !strcmp(psy->desc->name, "wireless"))) {
		if (!strcmp(psy->desc->name, "usb")) {
			retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &pval);
			if (retval) {
				pr_err("%s usb psy get property failed, ERR: %d\n", psy->desc->name, retval);
				return NOTIFY_DONE;
			}
			usb_present = (pval.intval) ? true : false;
		} else if (!strcmp(psy->desc->name, "wireless")) {
			retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &pval);
			if (retval) {
				pr_err("%s wls psy get property failed, ERR: %d\n", psy->desc->name, retval);
				return NOTIFY_DONE;
			}
			wls_present = (pval.intval) ? true : false;
		}
		pr_info("usb_present = %d, wls_present = %d\n",usb_present, wls_present);

		if (chg->chg_is_present == (usb_present || wls_present)) {
			pr_info("ps present state unchanged\n");
			return NOTIFY_DONE;
		}
		chg->chg_is_present = (usb_present || wls_present);

		if (!chg->pen_insert_flag) {
			pr_info("pen not present\n");
			return NOTIFY_DONE;
		} else if (chg->charging_status == PEN_STAT_CHARGING_BY_INSERT_PEN
		                 || chg->charging_status == PEN_STAT_CHARGING_BY_PEN_AFTER_DETECT_CURRENT
		                 || chg->charging_status == PEN_STAT_CHARGING_BY_INSERT_FULL_SOC_PEN) {
			pr_info("pen is charging by insert action\n");
			return NOTIFY_DONE;
		}

		if (chg->chg_is_present) {
			start_charge(chg, true);
			chg->charging_status = PEN_STAT_CHARGING_BY_ADAPTER;
			start_chg_timer(chg, true, CHG_START_DELAY_S);
			pen_charger_handle_event(chg, true);
		} else {
			chg->chg_to_iterm = false;
			start_chg_timer(chg, false, 0);
			start_charge(chg, false);
			chg->charging_status = PEN_STAT_NOT_CHARGING;
			pen_charger_handle_event(chg, false);
		}
	}

    return NOTIFY_OK;
}

static ssize_t pen_report_uevent_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pen_charger *chg = dev_get_drvdata(dev);
	unsigned long r;
	unsigned long mode;

	if (!chg) {
		pr_err("pen_report_uevent_store: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("pen_report_uevent_store: Invalid charger suspend value = %lu\n", mode);
		return -EINVAL;
	}

	pr_info("pen_report_uevent_store: enable = %lu\n",mode);
	if (!!mode && (chg->charging_status != PEN_STAT_NOT_CHARGING)) {
		pen_charger_handle_event(chg, true);
		pr_info("pen_report_uevent_store: report pen charging status\n");
	}

	return count;
}
static DEVICE_ATTR(pen_report_uevent, 0200,
		NULL,
		pen_report_uevent_store);

static int pen_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg;
	int rc;
	int pen_present;
	char *name = NULL;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;

	rc = pen_charger_parse_dt(chg);
	if (rc) {
		pr_err("Failed to parse devicetree, rc=%d\n", rc);
		return rc;
	}
	rc  = devm_gpio_request_one(dev, chg->chg_ldswtch_en_gpio,
				  GPIOF_OUT_INIT_LOW, "chg-ldswtch-en-gpio");
	if (rc  < 0) {
			pr_err("Failed to request chg-ldswtch-en-gpio, ret:%d", rc);
			return rc;
	}
	rc  = devm_gpio_request_one(dev, chg->chg_boost_en_gpio,
				  GPIOF_OUT_INIT_LOW, "chg-boost-en-gpio");
	if (rc  < 0) {
			pr_err("Failed to request chg-boost-en-gpio, ret:%d", rc);
			return rc;
	}
	chg->pen_sns_chan = devm_iio_channel_get(&pdev->dev,"pm8350b_pen_sns");
	if (IS_ERR(chg->pen_sns_chan)) {
		rc = PTR_ERR(chg->pen_sns_chan);
		chg->pen_sns_chan = NULL;
		pr_err("Error Getting pen_sns_chan channel- rc:%d\n",rc);
		return rc;
	}

	rc = device_create_file(dev, &dev_attr_pen_chg_current);
	if (rc) {
		pr_err("couldn't create pen_chg_current\n");
		return rc;
	}
	rc = device_create_file(dev, &dev_attr_pen_chg_enable);
	if (rc) {
		pr_err("couldn't create pen_chg_enable\n");
		return rc;
	}
	rc = device_create_file(dev, &dev_attr_pen_report_uevent);
	if (rc) {
		pr_err("couldn't create pen_report_uevent\n");
		return rc;
	}

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"POGO_PEN_wakelock");
	chg->charger_wakelock = wakeup_source_register(NULL, name);
	if (!chg->charger_wakelock) {
		pr_err("chg->charger_wakelock is null\n");
		return -ENOMEM;
	}
	alarm_init(&chg->chg_tmr, ALARM_BOOTTIME, chg_tmr_handler);
	init_waitqueue_head(&chg->chg_waitq);
	chg->chg_task = kthread_run(chg_thread_func, chg, "pogo_pen_chg_thread");
	if (IS_ERR(chg->chg_task)) {
		pr_err("Failed to create kernel thread\n");
		return PTR_ERR(chg->chg_task);
	}

	chg->pen_notif.notifier_call = pen_notifier_callback;
	rc = pen_detection_register_client(&chg->pen_notif);
	if (rc) {
		pr_err("Unable to register pen_notifier: rc=%d\n", rc);
		return rc;
	}
	chg->chg_psy_nb.notifier_call = charger_psy_notify_callback;
	rc = power_supply_reg_notifier(&chg->chg_psy_nb);
	if (rc) {
		pr_err("Failed to register chg_psy notifier, rc=%d\n", rc);
		return rc;
	}
	chg->initialized = true;
	pr_info("pen_charger_probe done\n");

	pen_present = pen_detection_status();
	if (pen_present == PEN_DETECTION_INSERT) {
		chg->pen_insert_flag = true;
		chg->charging_status = PEN_STAT_CHARGING_BY_INSERT_PEN;
		start_charge(chg, true);
		start_chg_timer(chg, true, CHG_START_DELAY_S);
	} else if (pen_present == PEN_DETECTION_PULL)
		chg->pen_insert_flag = false;
	pr_info("pen_charger_probe pen insert = %d\n", chg->pen_insert_flag);

	return 0;
}

static int pen_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg = dev_get_drvdata(dev);

	if (chg->charger_wakelock) {
		wakeup_source_unregister(chg->charger_wakelock);
		chg->charger_wakelock = NULL;
	}
	power_supply_unreg_notifier(&chg->chg_psy_nb);
	if (pen_detection_unregister_client(&chg->pen_notif))
		pr_err("Error occurred while unregistering pen_notifier.\n");
	alarm_cancel(&chg->chg_tmr);
	if (chg->chg_task) {
        kthread_stop(chg->chg_task);
        chg->chg_task = NULL;
    }

	device_remove_file(dev, &dev_attr_pen_report_uevent);
	device_remove_file(dev, &dev_attr_pen_chg_enable);
	device_remove_file(dev, &dev_attr_pen_chg_current);

	return 0;
}

static const struct of_device_id pen_charger_match_table[] = {
	{.compatible = "mmi,pogo-pen-charger"},
	{},
};

static struct platform_driver pen_charger_driver = {
	.driver	= {
		.name = "pogo_pen_charger",
		.of_match_table = pen_charger_match_table,
	},
	.probe	= pen_charger_probe,
	.remove	= pen_charger_remove,
};

module_platform_driver(pen_charger_driver);

MODULE_DESCRIPTION("Pogo Pen Charger Driver");
MODULE_LICENSE("GPL v2");
