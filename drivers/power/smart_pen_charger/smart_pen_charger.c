/*
 * Copyright (C) 2021 Motorola Mobility LLC
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

#define pr_fmt(fmt)     "SMART_PEN_CHG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include "smart_pen_charger.h"


static bool simulator_enabled;
module_param(simulator_enabled, bool, 0600);
MODULE_PARM_DESC(simulator_enabled, "Enable Pen Simulator");

static bool PEN_ATTACHED = false;
static bool PEN_CHRG_DISABLED = false;
//static bool CPS_TX_MODE = false;
//static bool CPS_FOLIO_MODE = false;
//static uint32_t RX_CONNECTED = false;
//static bool CPS_RX_MODE_ERR = false;
//static bool PEN_MAC0_READY = false;

static uint32_t cps_pen_status = PEN_STAT_DETACHED;
static uint32_t cps_pen_soc = 0;


static struct pen_charger *this_chip = NULL;
static struct moto_wls_pen_ops* wls_pen_ops;
static void pen_charger_handle_event(struct pen_charger *chg, int event);

static void pen_charger_handle_event(struct pen_charger *chg, int event)
{
	char *event_string = NULL;
	struct pen_charger_data *data = NULL;

	event_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!event_string)
		return;

	if (!simulator_enabled)
		data = &chg->pen_data;
	else
		data = &chg->simulator_data;

	switch (event) {
	case NOTIFY_EVENT_PEN_ID:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_ID=0x%x",
			data->id);
		break;
	case NOTIFY_EVENT_PEN_STATUS:
		if (data->status == PEN_STAT_DETACHED) {
			pr_warn("Pen has been deteched, clean pen data!\n");
			memset(data, 0, sizeof(struct pen_charger_data));
		}
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_STATUS=%s",
			pen_status_maps[data->status]);
		chg->pen_data.status = data->status;
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_status");
		break;
	case NOTIFY_EVENT_PEN_SOC:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_SOC=%d",
			data->soc);
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_soc");
		break;
	case NOTIFY_EVENT_PEN_MAC:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_MAC=%2x:%2x:%2x:%2x:%2x:%2x",
			data->mac.addr[0], data->mac.addr[1],
			data->mac.addr[2], data->mac.addr[3],
			data->mac.addr[4], data->mac.addr[5]);
		break;
	case NOTIFY_EVENT_PEN_ERROR:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_ERROR=%d",
			data->error);
		chg->pen_data.error = data->error;
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_error");
		break;
	default:
		pr_err("Invalid notify event %d\n", event);
		kfree(event_string);
		return;
	}

	if (!chg->pen_psy) {
		kfree(event_string);
		pr_warn("Pen power supply is unavailable\n");
		return;
	}

	if (chg->pen_uenvp[0])
		kfree(chg->pen_uenvp[0]);
	chg->pen_uenvp[0] = event_string;
	chg->pen_uenvp[1] = NULL;
	kobject_uevent_env(&chg->pen_psy->dev.kobj,
			KOBJ_CHANGE,
			chg->pen_uenvp);
	pr_info("Send pen event: %s\n", event_string);
}

int wls_send_oem_notification(int event, struct wls_pen_charger_notify_data *wls_pen_notify)
{
	int i;
	bool pen_changed = 0;
	//struct qti_charger_notify_data *notify_data = data;
	struct pen_charger *chg = this_chip;
	struct wls_pen_charger_notify_data *notify_data = wls_pen_notify;

	switch (event)
	{
	case NOTIFY_EVENT_PEN_ID:
		/* PEN ID update */
		pen_changed = 1;
		chg->pen_data.id = notify_data->data[0];
		break;
	case NOTIFY_EVENT_PEN_STATUS:
		/*
		* PEN status changes: attached, ready, charging,
		* discharging, charged, detached
		*/
		pen_changed = 1;
		cps_pen_status = notify_data->data[0];
		chg->pen_data.status = notify_data->data[0];
		break;
	case NOTIFY_EVENT_PEN_SOC:
		/* PEN status of charge changes */
		pen_changed = 1;
		chg->pen_data.soc = notify_data->data[0];
		cps_pen_soc = notify_data->data[0];
		break;

	case NOTIFY_EVENT_PEN_MAC:
		/* PEN MAC update */
		pen_changed = 1;
		for (i = 0; i < MAC_ADDR_LEN; i++)
			chg->pen_data.mac.addr[i] = notify_data->data[i] & 0xFF;
		break;
	case NOTIFY_EVENT_PEN_ERROR:
		/* PEN occurs error */
		pen_changed = 1;
		chg->pen_data.error = notify_data->data[0];
		break;

	case NOTIFY_EVENT_PEN_TIME_TO:
		chg->pen_events = chg->pen_events | (1 << PEN_EVENT_TIMER_TO);
		wake_up_pen_thread(chg);
		break;
	default:
		pr_err("Unknown pen event: %#x\n", event);
		break;
	}

	if (pen_changed)
		pen_charger_handle_event(chg, (int)event);

	return 0;
}
EXPORT_SYMBOL(wls_send_oem_notification);

static int wls_pen_control(uint32_t Ctl)
{
	int status = WLS_PEN_SUCCESS;
	struct pen_charger *chg = this_chip;

	if (Ctl > PEN_STAT_MAX)
		return -1;

	switch (Ctl)
	{
	case PEN_CTRL_ATTACHED:
		wls_pen_ops->wls_enable_pen_tx(true);
		PEN_ATTACHED = true;
		cps_pen_status = PEN_STAT_ATTACHED;
		RtcTime_clear();
		wls_pen_ops->wls_pen_attached();
		chg->pen_events = chg->pen_events | (1 << PEN_EVENT_ATTACHED);
		wake_up_pen_thread(chg);
		//pen_event_notify(PEN_EVENT_ATTACHED);
		break;

	case PEN_CTRL_DETACHED:
		wls_pen_ops->wls_pen_dettached();
		wls_pen_ops->wls_enable_pen_tx(false);
		PEN_ATTACHED = false;
		cps_pen_status = PEN_STAT_DETACHED;
		cps_pen_soc = 0;
		RtcTime_clear();
		chg->pen_events = chg->pen_events | (1 << PEN_EVENT_DETACHED);
		wake_up_pen_thread(chg);
		//pen_event_notify(PEN_EVENT_DETACHED);
		break;

	case PEN_CTRL_CHG_EN:
		pr_info("wls pen chg enable\n");
		//status = wls_pen_chrg_enable();
		chg->pen_events = chg->pen_events | (1 << PEN_EVENT_CHRG_ENABLE);
		wake_up_pen_thread(chg);
		//pen_event_notify(PEN_EVENT_CHRG_ENABLE);
		break;

	case PEN_CTRL_CHG_DIS:
		pr_info("wls pen chg disable\n");
		//status = wls_pen_chrg_disable();
		chg->pen_events = chg->pen_events | (1 << PEN_EVENT_CHRG_DISABLE);
		wake_up_pen_thread(chg);
		//pen_event_notify(PEN_EVENT_CHRG_DISABLE);
		break;
	}

	return status;
}

static ssize_t pen_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	u32 cmd;
	unsigned long r;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &cmd);
	if (r) {
		pr_err("Invalid pen control cmd = 0x%x\n", cmd);
		return -EINVAL;
	}

	pr_info("Send pen cmd = 0x%x\n", cmd);
	rc = wls_pen_control(cmd);
	if (rc) {
		pr_err("Failed to set control cmd=0x%x, rc=%d\n", cmd, rc);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_WO(pen_control);

static ssize_t pen_id_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int id;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &id);
	if (r) {
		pr_err("Invalid pen id = 0x%x\n", id);
		return -EINVAL;
	}

	if (chg->simulator_data.id != id && id != 0) {
		pr_info("Pen id changed by user 0x%x -> 0x%x\n",
				chg->simulator_data.id, id);
		chg->simulator_data.id = id;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_ID);
	}

	return count;
}

static ssize_t pen_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	//int rc;
	u32 id;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%x\n",
					chg->simulator_data.id);

	id = 0x12345;

	if (chg->pen_data.id != id) {
		pr_info("Pen id updated 0x%x -> 0x%x\n", chg->pen_data.id, id);
		chg->pen_data.id = id;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%x\n", chg->pen_data.id);
}
static DEVICE_ATTR(pen_id, 0664, pen_id_show, pen_id_store);

static ssize_t pen_soc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int soc;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &soc);
	if (r) {
		pr_err("Invalid pen soc = %d\n", soc);
		return -EINVAL;
	}

	if (soc > 0 && soc <= 100 && chg->simulator_data.soc != soc) {
		pr_info("Pen soc changed by user %d -> %d\n",
					chg->simulator_data.soc, soc);
		chg->simulator_data.soc = soc;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_SOC);
	}

	return count;
}

static ssize_t pen_soc_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	//int rc;
	u32 soc;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->simulator_data.soc);

	soc = 50;
	wls_pen_ops->wls_get_pen_soc(&soc);

	if (soc >= 0 && soc <= 100 && chg->pen_data.soc != soc) {
		pr_info("Pen soc updated %d -> %d\n", chg->pen_data.soc, soc);
		chg->pen_data.soc = soc;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->pen_data.soc);
}
static DEVICE_ATTR(pen_soc, 0664, pen_soc_show, pen_soc_store);

static ssize_t pen_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int i;
	u32 status = PEN_STAT_MAX;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	for (i = 0; i < PEN_STAT_MAX; i++) {
		if (strstr(buf, pen_status_maps[i]) == buf) {
			status = i;
			break;
		}
	}

	if (status >= PEN_STAT_MAX) {
		pr_err("Invalid pen status = %d\n", status);
		return -EINVAL;
	}

	if (chg->simulator_data.status != status) {
		pr_info("Pen status changed by user %s -> %s\n",
				pen_status_maps[chg->simulator_data.status],
				pen_status_maps[status]);
		chg->simulator_data.status = status;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_STATUS);
	}

	return count;
}

static ssize_t pen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n",
				pen_status_maps[chg->simulator_data.status]);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->pen_data.status);
}
static DEVICE_ATTR(pen_status, 0664, pen_status_show, pen_status_store);

static ssize_t pen_mac_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	struct pen_mac mac;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	rc = sscanf(buf, "%2x:%2x:%2x:%2x:%2x:%2x",
				&mac.addr[0], &mac.addr[1], &mac.addr[2],
				&mac.addr[3], &mac.addr[4], &mac.addr[5]);
	if (rc != MAC_ADDR_LEN) {
		pr_err("Invalid pen mac, rc = %d\n", rc);
		return -EINVAL;
	}

	if (memcmp(&chg->simulator_data.mac, &mac, sizeof(mac))) {
		pr_info("Pen mac changed by user, mac=%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
		memcpy(&chg->simulator_data.mac, &mac, sizeof(mac));
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_MAC);
	}

	return count;
}

static ssize_t pen_mac_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int i;
	int rc;
	struct pen_mac mac;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled) {
		memcpy(&mac, &chg->simulator_data.mac, sizeof(mac));
		return scnprintf(buf, CHG_SHOW_MAX_SIZE,
				"%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
	}

	memset(&mac, 0xFF, sizeof(struct pen_mac));
	rc = wls_pen_ops->wls_get_pen_mac(mac.addr);
	if (rc) {
		pr_err("Failed to read pen mac, rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < MAC_ADDR_LEN; i++)
		mac.addr[i] &= 0xFF;

	if (memcmp(&chg->pen_data.mac, &mac, sizeof(mac))) {
		pr_info("Pen mac updated, mac=%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
		memcpy(&chg->pen_data.mac, &mac, sizeof(mac));
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
}
static DEVICE_ATTR(pen_mac, 0664, pen_mac_show, pen_mac_store);

static ssize_t pen_error_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int error;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &error);
	if (r) {
		pr_err("Invalid pen error = %d\n", error);
		return -EINVAL;
	}

	if (chg->simulator_data.error != error) {
		pr_info("Pen error changed by user %d -> %d\n",
					chg->simulator_data.error, error);
		chg->simulator_data.error = error;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_ERROR);
	}

	return count;
}

static ssize_t pen_error_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->simulator_data.error);
	else
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->pen_data.error);
}
static DEVICE_ATTR(pen_error, 0664, pen_error_show, pen_error_store);

void pen_charger_psy_init()
{
	int rc;
	struct pen_charger *chg = this_chip;

	if (chg->pen_psy)
		return;

	chg->pen_psy = power_supply_get_by_name("wireless");
	if (!chg->pen_psy) {
		pr_err("No pen power supply found\n");
		return;
	}
	pr_info("Pen power supply wireless is found\n");

	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_control);
        if (rc)
		pr_err("couldn't create pen control\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_id);
        if (rc)
		pr_err("couldn't create pen id\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_soc);
        if (rc)
		pr_err("couldn't create pen soc\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_status);
        if (rc)
		pr_err("couldn't create pen status\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_mac);
        if (rc)
		pr_err("couldn't create pen mac\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_error);
        if (rc)
		pr_err("couldn't create pen error\n");
}
EXPORT_SYMBOL(pen_charger_psy_init);

int moto_wireless_pen_ops_register(struct moto_wls_pen_ops *ops)
{
	if (!ops) {
		pr_err("%s invalide wls chg ops(null)\n", __func__);
		return -EINVAL;
	}

	wls_pen_ops = ops;

	return 0;
}
EXPORT_SYMBOL(moto_wireless_pen_ops_register);

static enum alarmtimer_restart
	wls_pen_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct pen_charger *info = this_chip;
	wake_up_pen_thread(info);
	return ALARMTIMER_NORESTART;
}

static void wls_pen_start_timer(struct pen_charger *info)
{
	struct timespec64 end_time, time_now;
	ktime_t ktime, ktime_now;
	int ret = 0;

	/* If the timer was already set, cancel it */
	ret = alarm_try_to_cancel(&info->wls_pen_timer);
	if (ret < 0) {
		pr_err("%s: callback was running, skip timer\n", __func__);
		return;
	}

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	end_time.tv_sec = time_now.tv_sec + info->polling_s;
	end_time.tv_nsec = time_now.tv_nsec + info->polling_ns;
	info->endtime = end_time;
	ktime = ktime_set(info->endtime.tv_sec, info->endtime.tv_nsec);

	pr_err("%s: alarm timer start:%d, %lld %ld\n", __func__, ret,
		info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->wls_pen_timer, ktime);
}

static void wls_pen_init_timer(struct pen_charger *info)
{
	alarm_init(&info->wls_pen_timer, ALARM_BOOTTIME,
			wls_pen_alarm_timer_func);
	//wls_pen_start_timer(info);

}

void wake_up_pen_thread(struct pen_charger *info)
{
	//unsigned long flags;
	if (info == NULL)
		return;

	if (!info->pen_wakelock->active)
		__pm_stay_awake(info->pen_wakelock);
	info->wls_pen_thread_timeout = true;
	wake_up_interruptible(&info->wait_que);
}

static uint32_t current_rtc_time = 0;
static uint32_t prev_rtc_time = 0;
void  pm_rtc_get_time(uint32_t *current_rtc_time)
{
	struct timespec64 time_now;
	ktime_t ktime_now;
	//int ret = 0;

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	*current_rtc_time = time_now.tv_sec;

}
static bool rtc_time_passed_duration(uint32_t curr_time, uint32_t prev_time, uint32_t duration)
{
    bool passed = false;

    if(curr_time >= prev_time)
    {
        if((curr_time - prev_time) >= duration)
        {
            passed = true;
        }
    }
    else // system time wraps around to 0 when it exceeds the maximum value. curr_time could be smaller than prev_time
    {
        if((curr_time < prev_time)&& (curr_time + (uint32_t)(~(uint32_t)(0) - prev_time + 1) >= duration))
        {
            passed = true;
        }
    }

    return passed;
}

void RtcTime_clear(void)
{
    pm_rtc_get_time(&current_rtc_time);
    prev_rtc_time = current_rtc_time;
}

static bool maintain_time(uint32_t duration_s)
{
    pm_rtc_get_time(&current_rtc_time);
    if (prev_rtc_time == 0)
        prev_rtc_time = current_rtc_time;

    if (rtc_time_passed_duration(current_rtc_time, prev_rtc_time, duration_s)) {
	    wls_pen_log(PEN_LOG_ERR, "wls_pen maintain_time :  "
			"current_rtc_time  %d, prev_rtc_time %d,  duration %ds",
			current_rtc_time,
			prev_rtc_time,
			duration_s);
            current_rtc_time = 0;
            prev_rtc_time = 0;
            return true;
	}

    return false;
}

static int wls_pen_events_thread(void *arg)
{
	int status = WLS_PEN_SUCCESS;
	unsigned int events, ev;
	PEN_EVENT_TYPE ev_t = 0;
	struct pen_charger *info = arg;
	//int delay_time_ms = 10000; //10s

	while (1)
	{
		status = wait_event_interruptible(info->wait_que, (info->wls_pen_thread_timeout == true));
		if (status < 0)
		{
			wls_pen_log(PEN_LOG_ERR, "wls_pen %s: wait event been interrupted\n", __func__);
			continue;
		}
		/* Select the interesting pending events */
		events = (unsigned int)(info->pen_events & PEN_WAIT_EVENTS);

		/* get the next event */
		ev = events & ~(events - 1);

		/* tranlate into event type, spoken by charger detection & ICM */
		while (!(ev & (1 << ev_t)))
			ev_t++;

		/* remove from pending list */
		events &= ~ev;

		/*clear event*/
		info->pen_events = ~(1 << ev_t) & info->pen_events;
		wls_pen_log(PEN_LOG_DEBG, "wls_pen thread events:%d ev_t %d\n", events, ev_t);
		info->wls_pen_thread_timeout = false;
		/* handle the event */
		switch (ev_t)
		{
		case PEN_EVENT_ATTACHED:
			wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN_EVENT_ATTACHED");
			wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS ask pen mac");
			wls_pen_ops->wls_ask_pen_mac();
			wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS Start PenTimer");
			info->polling_s = 2; //2s
			info->pen_events = info->pen_events | (1 << PEN_EVENT_TIMER_TO);
			wls_pen_start_timer(info);
			break;
		case PEN_EVENT_DETACHED:
			wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN_EVENT_DETACHED");
			wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS Stop PenTimer");
			alarm_try_to_cancel(&info->wls_pen_timer);
			break;
		case PEN_EVENT_CHRG_ENABLE:
			wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN_EVENT_CHRG_ENABLE");
			if (!PEN_ATTACHED)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen pen has been detached, stop PenTimer");
				PEN_CHRG_DISABLED = false;
				alarm_try_to_cancel(&info->wls_pen_timer);
				break;
			}

			if (PEN_CHRG_DISABLED)
			{
				if (cps_pen_status == PEN_STAT_ATTACHED)
				{
					wls_pen_log(PEN_LOG_DEBG, "wls_pen Wait for pen ready");
					wls_pen_ops->wls_ask_pen_mac();
					msleep(2000);
					//Battman_os_sleep(2000000);
					info->pen_events = info->pen_events | (1 << PEN_EVENT_CHRG_ENABLE);
					//pen_event_notify(PEN_EVENT_CHRG_ENABLE);
					info->polling_s = 0;
					info->polling_ns = 1 * 1000 * 1000;
					wls_pen_start_timer(info);
					break;
				}

				wls_pen_ops->wls_pen_power_on(true); //enable power cps
				//Battman_os_sleep_non_deferrable(100000);//100ms
				msleep(100);//100ms
				wls_pen_ops->wls_enable_pen_tx(true);
				RtcTime_clear();
				cps_pen_status = PEN_STAT_CHARGING;
				info->pen_data.status = PEN_STAT_CHARGING;
				//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);

				wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS Start PenTimer");
				//BattMngr_Timer_Start(PenTimer, 2000);
				info->polling_s = 2; //2s
				info->pen_events = info->pen_events | (1 << PEN_EVENT_TIMER_TO);
				wls_pen_start_timer(info);
			}
			PEN_CHRG_DISABLED = false;
			break;
		case PEN_EVENT_CHRG_DISABLE:
			wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN_EVENT_CHRG_DISABLE");
			if (!PEN_ATTACHED)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen pen has been detached, stop PenTimer");
				alarm_try_to_cancel(&info->wls_pen_timer);
				PEN_CHRG_DISABLED = true;
				break;
			}

			if (!PEN_CHRG_DISABLED)
			{
				if (cps_pen_status == PEN_STAT_ATTACHED)
				{
					wls_pen_log(PEN_LOG_DEBG, "wls_pen Wait for pen ready");
					wls_pen_ops->wls_ask_pen_mac();
					msleep(2000);
					//pen_event_notify(PEN_EVENT_CHRG_DISABLE);
					info->pen_events = info->pen_events | (1 << PEN_EVENT_CHRG_DISABLE);
					info->polling_s = 0;
					info->polling_ns = 1 * 1000 * 1000;
					wls_pen_start_timer(info);
					break;
				}
				wls_pen_ops->wls_enable_pen_tx(false);
				wls_pen_ops->wls_pen_power_on(false); //enable power cps
				RtcTime_clear();
				PEN_CHRG_DISABLED = true;
				cps_pen_status = PEN_STAT_CHARGE_DISABLED;
				info->pen_data.status = PEN_STAT_CHARGE_DISABLED;
				pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);
				//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				alarm_try_to_cancel(&info->wls_pen_timer);
			}
			break;
		case PEN_EVENT_TIMER_TO:
			wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN_EVENT_TIMER_TO");
			if (!PEN_ATTACHED)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen pen has been detached, stop PenTimer");
				break;
			}

			if (PEN_CHRG_DISABLED &&
				(cps_pen_status > PEN_STAT_READY && cps_pen_status != PEN_STAT_CHARGE_DISABLED))
			{
				wls_pen_ops->wls_enable_pen_tx(false);
				wls_pen_ops->wls_pen_power_on(false); //enable power cps
				RtcTime_clear();
				cps_pen_status = PEN_STAT_CHARGE_DISABLED;
				info->pen_data.status = PEN_STAT_CHARGE_DISABLED;
				pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);
				//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				break;
			}

			if (cps_pen_status == PEN_STAT_ATTACHED)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen pen status still in PEN_STAT_ATTACHED, re-ask PEN_ATTACHED request");
				//pen_event_notify(PEN_EVENT_ATTACHED);
				//info->polling_s = 1; //1s
				info->pen_events = info->pen_events | (1 << PEN_EVENT_ATTACHED);
				info->polling_s = 0;
				info->polling_ns = 1 * 1000 * 1000;
				wls_pen_start_timer(info);
				break;
			}

			if (cps_pen_status == PEN_STAT_READY)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS cps_pen_status is ready, ask pen soc");
				wls_pen_ops->wls_ask_pen_soc();
				if (cps_pen_soc > 0)
				{
					cps_pen_status = PEN_STAT_CHARGING;
					info->pen_data.status = PEN_STAT_CHARGING;
					pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);
					//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				}
				else
				{
					wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS wait for SOC updating!");
					info->polling_s = 1; //1s
					goto Start_Timer;
				}
			}

			if (cps_pen_status == PEN_STAT_CHARGING)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS ask pen soc in charging process");
				wls_pen_ops->wls_ask_pen_soc();
				info->polling_s = 60; //60s
			}

			if (cps_pen_status == PEN_STAT_CHARGE_FULL)
			{
				wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN charging full, disable tx mode");
				wls_pen_ops->wls_pen_power_on(false); //disable power cps
				wls_pen_ops->wls_enable_pen_tx(false);
				cps_pen_status = PEN_STAT_DISCHARGING;
				RtcTime_clear();
				info->pen_data.status = PEN_STAT_DISCHARGING;
				pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);
				//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				info->polling_s = 3600; //5mins
				goto Start_Timer;
			}

			if (cps_pen_status == PEN_STAT_DISCHARGING)
			{

				if (cps_pen_soc < 80)
				{
					wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS pen soc %d, power on cps", cps_pen_soc);
					wls_pen_ops->wls_pen_power_on(true); //enable power cps
					//msleep(100);//100ms
					//cps_wls_enable_pen_tx_mode();
					wls_pen_ops->wls_enable_pen_tx(true);
					RtcTime_clear();
					cps_pen_status = PEN_STAT_CHARGING;
					info->pen_data.status = PEN_STAT_CHARGING;
					pen_charger_handle_event(info, NOTIFY_EVENT_PEN_STATUS);
					//wls_send_oem_notification(NOTIFY_EVENT_PEN_STATUS, cps_pen_status);
				}
				else
				{
					if (maintain_time(3600))
					{
						wls_pen_log(PEN_LOG_DEBG, "wls_pen PEN DISCHARGING has been going on for 1 hours, need to recheck pen soc");
						wls_pen_ops->wls_pen_power_on(true); //enable power cps
						//msleep(100);//100ms
						//cps_wls_enable_pen_tx_mode();
						wls_pen_ops->wls_enable_pen_tx(true);
						wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS ask pen soc, and wait 2s for geting new soc");
						msleep(100); //100ms
						wls_pen_ops->wls_ask_pen_soc();
						RtcTime_clear();
						info->polling_s = 60; //wait new soc within 1mins
					}
					else
					{
						wls_pen_log(PEN_LOG_DEBG, "wls_pen pen power off again due to soc %d > 80", cps_pen_soc);
						wls_pen_log(PEN_LOG_DEBG, "wls_pen CPS_WLS pen soc %d, power off cps", cps_pen_soc);
						wls_pen_ops->wls_pen_power_on(false); //disable power cps
						wls_pen_ops->wls_enable_pen_tx(false);
						cps_pen_status = PEN_STAT_DISCHARGING;
						info->polling_s = 600; // Recheck after 10mins
					}
				}
			}
		Start_Timer:
			info->pen_events = info->pen_events | (1 << PEN_EVENT_TIMER_TO);
			wls_pen_start_timer(info);

			break;
		default:
			break;
		}
		__pm_relax(info->pen_wakelock);
	}
	return status;
}

static int pen_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg;
	char *name = NULL;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	memset(&chg->pen_data, 0, sizeof(struct pen_charger_data));
	memset(&chg->simulator_data, 0, sizeof(struct pen_charger_data));

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"pen suspend wakelock");
	chg->pen_wakelock =
		wakeup_source_register(NULL, name);
	this_chip = chg;
	//pen_charger_psy_init(chg);
	init_waitqueue_head(&chg->wait_que);
	wls_pen_init_timer(chg);
	chg->polling_s = PEN_INTERVAL;
	chg->polling_ns = 0;
	chg->pen_events = 1 << PEN_STAT_DETACHED;
	kthread_run(wls_pen_events_thread, chg, "wls_pen_thread");
	return 0;
}

static int pen_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg= dev_get_drvdata(dev);

	if (chg->pen_psy) {
		if (chg->pen_uenvp[0]) {
			kfree(chg->pen_uenvp[0]);
			chg->pen_uenvp[0] = NULL;
		}
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_control);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_id);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_soc);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_status);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_mac);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_error);
		power_supply_put(chg->pen_psy);
	}

	return 0;
}

static const struct of_device_id pen_charger_match_table[] = {
	{.compatible = "mmi,pen-charger"},
	{},
};

static struct platform_driver pen_charger_driver = {
	.driver	= {
		.name = "pen_charger",
		.of_match_table = pen_charger_match_table,
	},
	.probe	= pen_charger_probe,
	.remove	= pen_charger_remove,
};

module_platform_driver(pen_charger_driver);

MODULE_DESCRIPTION("Smart Pen Charger Driver");
MODULE_LICENSE("GPL v2");
