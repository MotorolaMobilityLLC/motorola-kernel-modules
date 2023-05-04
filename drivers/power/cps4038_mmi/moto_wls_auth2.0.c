/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * version:1.3
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#include <linux/alarmtimer.h>
#include "moto_wls_auth2.0.h"


static MOTO_WLS_AUTH_T *motoauth = NULL;
static MOTO_AUTH_STATUS moto_auth_status = MOTO_AUTH_EXIT;

int motoauth_timer_start(int ms);
int motoauth_timer_stop(void);
int motoauth_event_notify(MOTOAUTH_EVENT_TYPE EventToNotify);
int motoauth_wake_up_events_thread(void);

void wls_waiting_delay(int delay_ms)
{
	int timeout = delay_ms;
	if (delay_ms <= 0)
		return;
	do {
		msleep(100);//100ms
		timeout -= 100;
		if (motoauth->wls_get_ldo_on && motoauth->wls_get_ldo_on()) {
			timeout = 0;
		}
	} while (timeout > 0);
	return;
}

int motoauth_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
	if (!motoauth || motoauth->wls_send_ask_packet == NULL) {
		return MOTO_WLS_AUTH_FAIL;
	}
	return motoauth->wls_send_ask_packet(data, data_len);
}

int moto_auth_send_ask_tx_capability(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[2] = {0x18, QI_ASK_CMD_TXCAPABILITY};

	status = motoauth_wls_send_ask_packet(data, 2);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_TXCAPABILITY");
	return status;
}

int moto_auth_send_ask_tx_id(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[2] = {0x18, QI_ASK_CMD_TXID};

	status = motoauth_wls_send_ask_packet(data, 2);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_TXID");
	return status;
}

int moto_auth_send_ask_qfod(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[5] = {0x48, QI_ASK_CMD_QFOD, 0x00, 0x28, 0x64};

	status = motoauth_wls_send_ask_packet(data, 5);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_QFOD");
	return status;
}

int moto_auth_send_ask_tx_capacity(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[2] = {0x18, QI_ASK_CMD_TXCAP};

	status = motoauth_wls_send_ask_packet(data, 2);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_TXCAP");
	return status;
}

int moto_auth_send_ask_tx_sn(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[2] = {0x18, QI_ASK_CMD_SN};

	status = motoauth_wls_send_ask_packet(data, 2);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_SN");
	return status;
}

int moto_auth_send_ask_sha_encry_num(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[6] = {0x58, QI_ASK_CMD_SHA1_NUM, 0x61, 0x62, 0x63, 0x64};

	status = motoauth_wls_send_ask_packet(data, 6);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_SHA1_NUM, 0x61, 0x62, 0x63, 0x64");
	return status;
}

int moto_auth_send_ask_sha_encry_result(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t data[2] = {0x18, QI_ASK_CMD_SHA1_RESULT};

	status = motoauth_wls_send_ask_packet(data, 2);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: QI send ask cmd QI_ASK_CMD_SHA1_RESULT");
	return status;
}

int moto_auth_printf_data(const char *name, uint8_t *data, int data_len)
{
	char buf[128] = {0x00};
	int i = 0;

	if (data_len < 128/3) {
		for (i=0; i<data_len; i++) {
			sprintf(&buf[i*3],"%02X ", data[i]);
		}
	}
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " %s->%s: len=%d", name, buf, data_len);

	return MOTO_WLS_AUTH_SUCCESS;
}

int motoauth_send_wls_sysfs_notify(const char *str)
{
	if (motoauth && motoauth->wls_sysfs_notify) {
		motoauth->wls_sysfs_notify(str);
		return MOTO_WLS_AUTH_SUCCESS;
	}
	return MOTO_WLS_AUTH_FAIL;
}

int motoauth_wls_set_status(wlc_status st)
{
	if (motoauth && motoauth->wls_set_status) {
		motoauth->wls_set_status(st);
		return MOTO_WLS_AUTH_SUCCESS;
	}
	return MOTO_WLS_AUTH_FAIL;
}

int moto_auth_get_fsk_packet(uint8_t *data, int data_len)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t tx_id[2] = {0};

	if (!motoauth)
		return MOTO_WLS_AUTH_FAIL;

	switch (data[0]) {
	case QI_ASK_CMD_TXCAPABILITY:
		if (data_len != 5) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, " data_len error with QI_ASK_CMD_TXCAPABILITY");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " QI_ASK_CMD_TXCAPABILITY: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, data[3] 0x%x, moto_auth_status %d",
			data[0], data[1], data[2], data[3], data[4], moto_auth_status);
		motoauth_timer_stop();
		motoauth->WLS_WLC_TYPE = WLC_MOTO;
		motoauth->WLS_WLC_CAPABILITY = data[1];
		motoauth->WLC_STATUS = WLC_TX_CAPABILITY_CHANGED;
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_CAPABILITY %d", motoauth->WLS_WLC_CAPABILITY);

		motoauth_wls_set_status(motoauth->WLC_STATUS);
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, "To ask TX_ID next");
		motoauth_event_notify(MOTOAUTH_EVENT_TX_ID);
		break;
	case QI_ASK_CMD_TXID:
		if (data_len != 3) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, " data_len error with QI_ASK_CMD_TXID");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		tx_id[0] = data[1];
		tx_id[1] = data[2];
		motoauth_timer_stop();
		motoauth->WLS_WLC_TYPE = WLC_MOTO;
		motoauth->WLS_WLC_ID = (uint32_t)(data[1] << 8 | data[2]);
		motoauth->WLC_STATUS = WLC_TX_ID_CHANGED;
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " QI_ASK_CMD_TXID: cmd 0x%x, data[0] 0x%x, data[1] 0x%x",
			data[0], data[1], data[2]);
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_ID %d", motoauth->WLS_WLC_ID);
		motoauth_wls_set_status(motoauth->WLC_STATUS);
		if (WLS_WLC_POWER_MAX <= 15) {
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, "To ask TX_SN next");
			motoauth_event_notify(MOTOAUTH_EVENT_TX_SN);
		} else if (tx_id[0] == 0x01 && (tx_id[1] >> 4) == 0x5 && moto_auth_status == MOTO_AUTH_TX_ID) {
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, "To ask TX_CAP next");
			motoauth_event_notify(MOTOAUTH_EVENT_TX_CAP);
		} else {
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, "To ask TX_SN next");
			motoauth_event_notify(MOTOAUTH_EVENT_TX_SN);
		}
		break;
	case QI_ASK_CMD_TXCAP:
		if (data_len != 4) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, " data_len error with QI_ASK_CMD_TXCAP");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		/*get tx cap type at data0*/
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " QI_ASK_CMD_TXCAP: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, moto_auth_status %d",
			data[0], data[1], data[2], data[3], moto_auth_status);
		if (data[0] == 0x41 && moto_auth_status == MOTO_AUTH_TX_CAP) {

			motoauth->WLS_WLC_TYPE = WLC_MOTO;
			motoauth->WLS_WLC_POWER = data[2] / 2;
			motoauth->WLC_STATUS = WLC_TX_POWER_CHANGED;
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , wlc_power %d", motoauth->WLS_WLC_POWER);
			motoauth_wls_set_status(motoauth->WLC_STATUS);
			if (motoauth->WLS_WLC_POWER <= 5) {
				motoauth_wls_log(MOTOAUTH_LOG_DEBG, " WLC_TYPE is still MOTO_WLC, power only 5W, Reset ICL to 1A");
				//sw_set_wls_icl(1000);
				if (motoauth->wls_set_wls_icl) {
						motoauth->wls_set_wls_icl(1000);
				}
				//sw_set_aicl_restart();
				if (motoauth->wls_set_aicl_restart) {
						motoauth->wls_set_aicl_restart();
				}
				motoauth_event_notify(MOTOAUTH_EVENT_TX_SN);
			} else {
				motoauth_wls_log(MOTOAUTH_LOG_DEBG, "Get corrent TX CAP : 0x41, capacity %dW, To ask REQ FAN_LEN next", data[2] / 2);
				motoauth_event_notify(MOTOAUTH_EVENT_REQ_FAN_LED);
			}
		}
		break;
	case QI_ASK_CMD_SHA1_RESULT:
		if (data_len != 5) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, " data_len error with QI_ASK_CMD_SHA1_RESULT");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " QI_ASK_CMD_SHA1_RESULT: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, data[3] 0x%x",
			data[0], data[1], data[2], data[3], data[4]);
		if (moto_auth_status == MOTO_AUTH_SHA_ENCRY_RESULT) {
			motoauth_timer_stop();
			motoauth_event_notify(MOTOAUTH_EVENT_SET_VOUT);
		}
		break;
	case QI_ASK_CMD_SN:
		if (data_len != 5) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, " data_len error with QI_ASK_CMD_SN");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " QI_ASK_CMD_SN: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, data[3] 0x%x",
			data[0], data[1], data[2], data[3], data[4]);

		motoauth->WLS_WLC_SN = (uint32_t)(data[1] << 24 | (data[2] << 16 | (data[3] << 8 | data[4])));
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_SN %d", motoauth->WLS_WLC_SN);
		motoauth_timer_stop();
		motoauth_wls_set_status(motoauth->WLC_STATUS);
		motoauth_event_notify(MOTOAUTH_EVENT_DONE);
		break;
		default:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " Not Found CMD 0x%x", data[0]);
			status = MOTO_WLS_AUTH_FAIL;
		}

	return status;
}

void motoauth_timer_work(struct work_struct *work)
{
	//struct moto_wls_auth *motoauth = container_of(work, struct moto_wls_auth, work.work);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	if (!motoauth) {
		return;
	}
	cancel_delayed_work(&motoauth->work);
	if (motoauth->work_running) {
		motoauth->events = motoauth->events | (1 << MOTOAUTH_EVENT_TIMER_TO);
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, "%s event=0x%X", __func__, (uint32_t)motoauth->events);
		motoauth_wake_up_events_thread();
	} else {
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, "%s cancel", __func__);
	}
}

int motoauth_timer_start(int ms)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " ms=%d", ms);
	if (!motoauth) {
		return MOTO_WLS_AUTH_FAIL;
	}
	motoauth->work_running = true;
	motoauth->work_delay_ms = ms;
	schedule_delayed_work(&motoauth->work, msecs_to_jiffies(ms));
	return 0;
}

int motoauth_timer_stop(void)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	//cancel_delayed_work_sync(&motoauth->work);
	//flush_delayed_work(&motoauth->work);
	cancel_delayed_work(&motoauth->work);
	motoauth->work_running = false;
	return 0;
}

int motoauth_events_process(void)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	unsigned int events, ev;
	MOTOAUTH_EVENT_TYPE ev_t = 0;
	uint32_t delay_time_ms = 1000;//, batt_soc = 0; //1s
	static MOTO_AUTH_STATUS pre_st = MOTO_AUTH_INVALID;

	if (motoauth == NULL) {
		motoauth_wls_log(MOTOAUTH_LOG_ERR, " motoauth_events_process motoauth is NULL!");
		return MOTO_WLS_AUTH_FAIL;
	}
	/* Select the interesting pending events */
	events = (unsigned int) (motoauth->events & MOTOAUTH_WAIT_EVENTS);
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " motoauth_events_process events=0x%X", events);
	while (events) {
		/* get the next event */
		ev = events & ~(events - 1);

		/* tranlate into event type, spoken by charger detection & ICM */
		while (!(ev & (1 << ev_t)))
			ev_t++;

		/* remove from pending list */
		events &= ~ev;
		motoauth->events &= ~ev;

		/* handle the event */
		switch (ev_t) {
		case MOTOAUTH_EVENT_TX_CAPABILITY:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_TX_CAPABILITY");
			moto_auth_status = MOTO_AUTH_TX_CAPABILITY;
			if (motoauth->WLC_STATUS == WLC_DISCONNECTED) {
				motoauth->WLC_STATUS = WLC_CONNECTED;
				motoauth_wls_set_status(motoauth->WLC_STATUS);
			}
			moto_auth_send_ask_tx_capability();
			motoauth_timer_start(delay_time_ms);
			break;
		case MOTOAUTH_EVENT_TX_ID:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_TX_ID");
			moto_auth_status = MOTO_AUTH_TX_ID;
			if (motoauth->WLC_STATUS == WLC_DISCONNECTED) {
				motoauth->WLC_STATUS = WLC_CONNECTED;
				motoauth_wls_set_status(motoauth->WLC_STATUS);
			}
			moto_auth_send_ask_tx_id();
			motoauth_timer_start(delay_time_ms * 2);
			break;
		case MOTOAUTH_EVENT_TX_CAP:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_TX_CAP");
			moto_auth_status = MOTO_AUTH_TX_CAP;
			moto_auth_send_ask_tx_capacity();
			break;
		case MOTOAUTH_EVENT_REQ_FAN_LED:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTOAUTH_EVENT_REQ_FAN_LED");
			moto_auth_status = MOTO_AUTH_REQ_FAN_LED;
			//cps_wls_wlc_update_light_fan();
			if (motoauth->wls_update_light_fan) {
				motoauth->wls_update_light_fan();
			}
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, "Ready to QFOD, waiting 8s delay");
			wls_waiting_delay(8000);
			motoauth_event_notify(MOTOAUTH_EVENT_QFOD);
			break;
		case MOTOAUTH_EVENT_QFOD:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_QFOD");
			moto_auth_status = MOTO_AUTH_QFOD;
			moto_auth_send_ask_qfod();
			motoauth_timer_start( delay_time_ms);
			break;
		case MOTOAUTH_EVENT_SHA_ENCRY_NUM:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_SHA_ENCRY_NUM");
			moto_auth_status = MOTO_AUTH_SHA_ENCRY_NUM;
			moto_auth_send_ask_sha_encry_num();
			motoauth_timer_start( delay_time_ms);
			break;
		case MOTOAUTH_EVENT_SHA_ENCRY_RESULT:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_SHA_ENCRY_RESULT");
			moto_auth_status = MOTO_AUTH_SHA_ENCRY_RESULT;
			moto_auth_send_ask_sha_encry_result();
			motoauth_timer_start( delay_time_ms * 2);
			break;
		case MOTOAUTH_EVENT_SET_VOUT:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_SET_VOUT");
			motoauth->WLC_VOUT_BOOSTING = true;
			//battmngr_get_power_supply_properties(POWER_SUPPLY_CAPACITY, &batt_soc);
			moto_auth_status = MOTO_AUTH_DONE;
			/*Battman_os_sleep_non_deferrable(200000);//200ms
			if (batt_soc >= MCP_ENTRY_SOC_UPPER_THD_PCT || USB_OTG_ENABLED) {
				sw_set_wls_icl(1000);
				cps_wls_req_vout_stage(12000);
				motoauth->WLC_VOUT_BOOSTING = FALSE;
			} else {
				sw_set_wls_icl(300);
				cps_wls_req_vout_stage(15000);
				cps_vout_boost_timeout(0);
			}*/
			break;
		case MOTOAUTH_EVENT_TX_SN:
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTOAUTH_EVENT_TX_SN");
			moto_auth_send_ask_tx_sn();
			moto_auth_status = MOTO_AUTH_TX_SN;
			motoauth_timer_start( delay_time_ms * 2);
			break;
		case MOTOAUTH_EVENT_TIMER_TO:
			if (pre_st != moto_auth_status) {
				motoauth->timeout_retry = 0;
				pre_st = moto_auth_status;
			}
			motoauth_timer_stop();
			motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH_EVENT_TIMER_TO st:%d\n",
					moto_auth_status);
			if (motoauth->timeout_retry ++ > 2) {
				pre_st = MOTO_AUTH_INVALID;
				motoauth_event_notify(MOTOAUTH_EVENT_DONE);
			} else if (moto_auth_status == MOTO_AUTH_QFOD) {
				motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH: Send QFOD again ");
				motoauth_event_notify(MOTOAUTH_EVENT_QFOD);
			} else if (moto_auth_status == MOTO_AUTH_SHA_ENCRY_NUM) {
				motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH: Send SHA_ENCRY_NUM again ");
				motoauth_event_notify(MOTOAUTH_EVENT_SHA_ENCRY_NUM);
			} else if (moto_auth_status == MOTO_AUTH_SHA_ENCRY_RESULT) {
				motoauth_wls_log(MOTOAUTH_LOG_DEBG, " MOTO_AUTH: Send SHA_ENCRY_RESULT again ");
				motoauth_event_notify(MOTOAUTH_EVENT_SHA_ENCRY_RESULT);
			} else if (moto_auth_status == MOTO_AUTH_TX_CAPABILITY) {
				motoauth_event_notify(MOTOAUTH_EVENT_TX_CAPABILITY);
			} else if (moto_auth_status == MOTO_AUTH_TX_ID) {
				motoauth_event_notify(MOTOAUTH_EVENT_TX_ID);
			} else if (moto_auth_status == MOTO_AUTH_TX_SN) {
				motoauth_event_notify(MOTOAUTH_EVENT_TX_SN);
			}
			break;
		case MOTOAUTH_EVENT_DONE:
			break;
		default:
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
	}
	return status;
}

static int motoauth_events_thread(void *arg)
{
	int status = MOTO_WLS_AUTH_SUCCESS;

	motoauth_wls_log(MOTOAUTH_LOG_DEBG, ": start");
	while (1) {
		if (!motoauth) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, ": motoauth is null,exit\n");
			return MOTO_WLS_AUTH_FAIL;
		}
		status = wait_event_interruptible(motoauth->wait_que, (motoauth->events_thread_timeout == true));
		if (status < 0) {
			motoauth_wls_log(MOTOAUTH_LOG_ERR, ": wait event been interrupted\n");
			continue;
		}
		motoauth->events_thread_timeout = false;
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, ": motoauth_events_process\n");
		motoauth_events_process();
		__pm_relax(motoauth->wakelock);
	}
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, ": exit\n");
	return status;
}

int motoauth_wake_up_events_thread(void)
{
	if (motoauth == NULL)
		return MOTO_WLS_AUTH_FAIL;
	if (!motoauth->wakelock->active)
		__pm_stay_awake(motoauth->wakelock);
	motoauth->events_thread_timeout = true;
	wake_up_interruptible(&motoauth->wait_que);
	return MOTO_WLS_AUTH_SUCCESS;
}

int motoauth_event_notify(MOTOAUTH_EVENT_TYPE EventToNotify)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " EventToNotify=0x%X", EventToNotify);
	if (!motoauth || EventToNotify >= MOTOAUTH_EVENT_INVALID) {
		return MOTO_WLS_AUTH_FAIL;
	}
	motoauth->events = motoauth->events | (1 << EventToNotify);
	motoauth_wake_up_events_thread();
	return status;
}

int motoauth_hs_ok_handler(Sys_Op_Mode sys_mode_type)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, " sys_mode_type=%d status=%d",
				sys_mode_type, moto_auth_status);
	moto_auth_status = MOTO_AUTH_EXIT;
	if (!motoauth->enable)
		return MOTO_WLS_AUTH_FAIL;

	if ((sys_mode_type == Sys_Op_Mode_BPP || sys_mode_type == Sys_Op_Mode_MOTO_WLC) &&
				moto_auth_status == MOTO_AUTH_EXIT) {
		motoauth_wls_log(MOTOAUTH_LOG_DEBG, " CPS_WLS: next to TX_CAPABILITY");
		if (motoauth)
			motoauth->WLS_HS_OK = true;
		motoauth_event_notify(MOTOAUTH_EVENT_TX_CAPABILITY);
	}
	return MOTO_WLS_AUTH_SUCCESS;
}

int motoauth_hs_fail_handler(void)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	if (motoauth)
		motoauth->WLS_HS_OK = false;
	moto_auth_status = MOTO_AUTH_EXIT;
	return MOTO_WLS_AUTH_SUCCESS;
}

int moto_wls_auth_clear(MOTO_WLS_AUTH_T *motoauth)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	moto_auth_status = MOTO_AUTH_EXIT;
	if (motoauth) {
		motoauth->WLS_WLC_ID = 0;
		motoauth->WLS_WLC_SN = 0;
		motoauth->WLS_WLC_CAPABILITY = 0;
		motoauth->events = 0x00;
		motoauth->WLC_STATUS = WLC_DISCONNECTED;
		motoauth->timeout_retry = 0;
		motoauth_timer_stop();
	}
	return MOTO_WLS_AUTH_SUCCESS;
}

int moto_wls_auth_init(MOTO_WLS_AUTH_T *moto_auth)
{
	int status = 0;
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	motoauth = moto_auth;
	moto_wls_auth_clear(motoauth);

	if (!motoauth->wls_send_ask_packet) {
		return MOTO_WLS_AUTH_FAIL;
	}
	init_waitqueue_head(&motoauth->wait_que);

	motoauth->wakelock = wakeup_source_register(NULL, "moto_wls_auth");

	motoauth->wls_get_fsk_packet = moto_auth_get_fsk_packet;
	kthread_run(motoauth_events_thread, motoauth, "motoauth_events_thread");

	motoauth->work_delay_ms = 10000; //10s
	INIT_DELAYED_WORK(&motoauth->work, motoauth_timer_work);

	motoauth->enable = true;
	//schedule_delayed_work(&motoauth->work, 0);

	return status;
}

int moto_wls_auth_remove(void)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	if (motoauth) {
		cancel_delayed_work_sync(&motoauth->work);
		moto_wls_auth_clear(motoauth);
		motoauth = NULL;
	}
	return MOTO_WLS_AUTH_SUCCESS;
}

int motoauth_disconnect(MOTO_WLS_AUTH_T *moto_auth)
{
	motoauth_wls_log(MOTOAUTH_LOG_DEBG, "");
	if (moto_auth) {
		moto_wls_auth_clear(moto_auth);
	}
	return MOTO_WLS_AUTH_SUCCESS;
}
