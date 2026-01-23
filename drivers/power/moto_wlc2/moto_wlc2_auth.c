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
#include <linux/completion.h>

#include <linux/alarmtimer.h>
#include <linux/mmi_wireless_class.h>
#include "moto_wlc2.h"


//static MOTO_AUTH_STATUS wls_auth_status = MOTO_AUTH_EXIT;

#define ASK_MSG_MAX_LEN 8
#define TIMEOUT_DEFAULT 2000
#define RETRY_MAX_DEFAULT 3

#define MOTO_WLS_AUTH_FAIL -1
#define MOTO_WLS_AUTH_SUCCESS 0


typedef int (*send_ask)(struct moto_wlc *wlc, int id);

typedef struct {
	int id;
	int status;
	uint8_t msg[ASK_MSG_MAX_LEN];
	int msgLen;
	int timeout;//ms
	int retryMax;
	int sendCnt;
	char name[32];
	send_ask hookFunc;
} auth_ask_t;

auth_ask_t auth_ask_tables[MOTOAUTH_EVENT_MAX] = {
	{MOTOAUTH_EVENT_TX_CAPABILITY, 0, {0x18, QI_ASK_CMD_TXCAPABILITY}, 2, TIMEOUT_DEFAULT, RETRY_MAX_DEFAULT, 0, "QI_ASK_CMD_TXCAPABILITY", NULL},
	{MOTOAUTH_EVENT_TX_ID, 0, {0x18, QI_ASK_CMD_TXID}, 2, TIMEOUT_DEFAULT, RETRY_MAX_DEFAULT, 0, "QI_ASK_CMD_TX_ID", NULL},
	{MOTOAUTH_EVENT_TX_CAP, 0, {0x18, QI_ASK_CMD_TXCAP}, 2, TIMEOUT_DEFAULT, RETRY_MAX_DEFAULT, 0, "QI_ASK_CMD_TX_CAP", NULL},
	{MOTOAUTH_EVENT_TX_SN, 0, {0x18, QI_ASK_CMD_SN}, 2, TIMEOUT_DEFAULT, RETRY_MAX_DEFAULT, 0, "QI_ASK_CMD_TX_SN", NULL},
	{MOTOAUTH_EVENT_DONE, 0, {0x00}, 0, TIMEOUT_DEFAULT, RETRY_MAX_DEFAULT, 0, "QI_ASK_CMD_DONE", NULL},
};


int wls_auth_wake_up_events_thread(struct wireless_auth *auth);
int wls_auth_clear(struct moto_wlc *wlc);

int wls_auth_printf_data(const char *name, uint8_t *data, int data_len)
{
	char buf[128] = {0x00};
	int i = 0;

	if (data_len < 128/3) {
		for (i=0; i<data_len; i++) {
			sprintf(&buf[i*3],"%02X ", data[i]);
		}
	}
	wlc_dbg(" %s->%s: len=%d", name, buf, data_len);

	return MOTO_WLS_AUTH_SUCCESS;
}

auth_ask_t * get_ask_id(int id)
{
	if (id < MOTOAUTH_EVENT_MAX)
		return & auth_ask_tables[id];
	return NULL;
}

int wls_auth_set_next_event(struct wireless_auth *auth, int event)
{
	if (IS_ERR_OR_NULL(auth))
		return MOTOAUTH_EVENT_MAX;

	pr_info("%s %d\n", __func__, auth->next_event);
	auth->next_event = (event < MOTOAUTH_EVENT_MAX) ? event : MOTOAUTH_EVENT_MAX;
	complete(&auth->recv_done);
	return auth->next_event;
}

int wls_auth_get_next_event(struct wireless_auth *auth)
{
	if (IS_ERR_OR_NULL(auth))
		return MOTOAUTH_EVENT_MAX;
	return auth->next_event;
}

int wls_auth_clean_next_event(struct wireless_auth *auth)
{
	auth->next_event = MOTOAUTH_EVENT_START;
	wls_auth_wake_up_events_thread(auth);
	return auth->next_event;
}

int wls_auth_send_ask(struct moto_wlc *wlc, auth_ask_t *pask)
{
	int rt = MOTO_WLS_AUTH_FAIL;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or NULL\n", __func__);
		return rt;
	}
	if (IS_ERR_OR_NULL(pask))
		return rt;

	if (!IS_ERR_OR_NULL(pask->hookFunc))
		return pask->hookFunc(wlc, pask->id);

	wlc_err("%s id %d: %s\n", __func__, pask->id, pask->name);
	rt = wls_rx_send_ask_packet(wlc->wls_dev, pask->msg, pask->msgLen);
	if (rt == pask->msgLen) {
		rt = MOTO_WLS_AUTH_SUCCESS;
	} else {
		rt = MOTO_WLS_AUTH_FAIL;
	}
	wlc_info("QI send ask cmd %s, st=%d rt=%d\n", pask->name, pask->status, rt);

	return rt;
}


int wls_auth_ask_clear(struct wireless_auth *auth)
{
	auth_ask_t *pask = NULL;
	int rt = MOTO_WLS_AUTH_FAIL;
	int id = 0;

	if (IS_ERR_OR_NULL(auth)) {
		wlc_err("%s auth is err or NULL\n", __func__);
		return rt;
	}

	complete(&auth->recv_done);
	for (id = 0; id < MOTOAUTH_EVENT_MAX; id++) {
		pask = get_ask_id(id);
		if (IS_ERR_OR_NULL(pask))
			return rt;
		pask->sendCnt = 0;
		pask->status = 0;
	}

	rt = MOTO_WLS_AUTH_SUCCESS;
	return rt;
}

int wls_auth_wakelock(struct wireless_auth *auth, bool en)
{
	if (IS_ERR_OR_NULL(auth)) {
		wlc_err("%s auth is err or NULL\n", __func__);
		return MOTO_WLS_AUTH_FAIL;
	}

	if (en && !auth->wakelock->active)
		__pm_stay_awake(auth->wakelock);
	else if (!en && auth->wakelock->active)
		__pm_relax(auth->wakelock);

	return 0;
}

int wls_auth_wake_up_events_thread(struct wireless_auth *auth)
{
	auth->events_thread_running = true;
	wake_up_interruptible(&auth->wait_que);
	return MOTO_WLS_AUTH_SUCCESS;
}

static int wls_auth_events_thread(void *arg)
{
	int ret = 0;
	long ret_comp = 0;
	int next_event = 0;
	auth_ask_t *pask = NULL;
	int status = 0;
	struct moto_wlc *wlc = arg;
	struct wireless_auth *auth = NULL;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or null\n", __func__);
		return ret;
	}

	auth = &wlc->auth;

	if (IS_ERR_OR_NULL(auth)) {
		wlc_err("%s auth is err or null\n", __func__);
		return ret;
	}

	wlc_info("%s start\n", __func__);

	while (true) {
		if (!auth->events_thread_running) {
			auth->auth_done = false;
			auth->auth_error = false;
		}
		//waitting wls connect
		status = wait_event_interruptible(auth->wait_que, (auth->events_thread_running));
		if (status < 0) {
			wlc_err(": wait event been interrupted\n");
			continue;
		}
		wls_auth_wakelock(auth, true);
		reinit_completion(&auth->recv_done);
		next_event = wls_auth_get_next_event(auth);
		pask = get_ask_id(next_event);
		if (IS_ERR_OR_NULL(pask)) {
			auth->auth_error = true;
		} else if (MOTO_WLS_AUTH_SUCCESS == wls_auth_send_ask(wlc, pask)) {
			ret_comp = wait_for_completion_interruptible_timeout(
					&auth->recv_done, msecs_to_jiffies(pask->timeout));
			if (ret_comp == 0)
				ret = -ETIMEDOUT;
			else if (ret_comp < 0)
				ret = -EINTR;
			else
				ret = 0;
			pask->sendCnt ++;
			if (pask->sendCnt > pask->retryMax) { //retry max
				auth->auth_error = true;
			}
		} else {
			auth->auth_error = true;
		}

		if (MOTOAUTH_EVENT_DONE == wls_auth_get_next_event(auth)) {
			wls_device_update_light_fan(wlc);
			auth->auth_done = true;
		}

		if (auth->auth_done || auth->auth_error) {
			auth->events_thread_running = false;
			wls_auth_wakelock(auth, false);
		}
	}

	return 0;
}

int wls_auth_wls_set_status(struct moto_wlc *wlc, int st)
{
	if (!IS_ERR_OR_NULL(wlc)) {
		return wls_chg_notify_st_changed(wlc, st);
	}

	return MOTO_WLS_AUTH_FAIL;
}

int wls_auth_hs_ok_handler(struct moto_wlc *wlc, int op_mode)
{
	int rt = MOTO_WLS_AUTH_FAIL;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_err("%s wlc is err or NULL\n", __func__);
		return rt;
	}

	if (!wlc->auth.enable)
		return MOTO_WLS_AUTH_FAIL;

	wls_auth_clear(wlc);
	wlc_info("%s next to TX_CAPABILITY", __func__);
	wls_auth_set_next_event(&(wlc->auth), MOTOAUTH_EVENT_START);

	wlc->auth.hs_ok = true;
	wls_auth_wake_up_events_thread(&wlc->auth);

	return MOTO_WLS_AUTH_SUCCESS;
}

int wls_auth_decode_fsk_packet(struct moto_wlc *wlc, uint8_t *data, int data_len)
{
	int status = MOTO_WLS_AUTH_SUCCESS;
	uint8_t tx_id[2] = {0};
	struct wireless_auth *auth = NULL;

	if (IS_ERR_OR_NULL(wlc)) {
		return MOTO_WLS_AUTH_FAIL;
	} else if (!wlc->auth.enable) {
		return MOTO_WLS_AUTH_FAIL;
	}
	wls_auth_printf_data(__func__, data, data_len);
	auth = &wlc->auth;

	switch (data[0]) {
	case QI_ASK_CMD_TXCAPABILITY:
		if (data_len != 5) {
			wlc_err(" data_len error with QI_ASK_CMD_TXCAPABILITY");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		wlc_dbg(" QI_ASK_CMD_TXCAPABILITY: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, data[3] 0x%x",
			data[0], data[1], data[2], data[3], data[4]);

		auth->wlc_type = WLC_MOTO;
		auth->wlc_tx_capability = data[1];
		wlc_dbg(" WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_CAPABILITY %d", auth->wlc_tx_capability);

		wls_auth_wls_set_status(wlc, WLC_TX_CAPABILITY_CHANGED);
		wls_auth_set_next_event(auth, MOTOAUTH_EVENT_TX_ID);
		break;
	case QI_ASK_CMD_TXID:
		if (data_len != 3) {
			wlc_err(" data_len error with QI_ASK_CMD_TXID");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		tx_id[0] = data[1];
		tx_id[1] = data[2];

		auth->wlc_type = WLC_MOTO;
		auth->wlc_tx_id = (uint32_t)(data[1] << 8 | data[2]);
		wlc_dbg(" QI_ASK_CMD_TXID: cmd 0x%x, data[0] 0x%x, data[1] 0x%x",
			data[0], data[1], data[2]);
		wlc_dbg(" WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_ID %d", auth->wlc_tx_id);
		wls_auth_wls_set_status(wlc, WLC_TX_ID_CHANGED);

		if (wlc->config.MaxPower <= 15) {
			wlc_dbg("To EVENT_DONE next");
			wls_auth_set_next_event(auth, MOTOAUTH_EVENT_DONE);
		} else if (tx_id[0] == 0x01 && (tx_id[1] >> 4) == 0x5) {
			wlc_dbg("To ask TX_CAP next");
			wls_auth_set_next_event(auth, MOTOAUTH_EVENT_TX_CAP);
		} else {
			wlc_dbg("To ask TX_SN next");
			wls_auth_set_next_event(auth, MOTOAUTH_EVENT_TX_SN);
		}
		break;
	case QI_ASK_CMD_TXCAP:
		if (data_len != 4) {
			wlc_err(" data_len error with QI_ASK_CMD_TXCAP");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		/*get tx cap type at data0*/
		wlc_dbg(" QI_ASK_CMD_TXCAP: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x",
			data[0], data[1], data[2], data[3]);
		if (data[0] == 0x41) {//&& wls_auth_status == MOTOAUTH_TX_CAP
			auth->wlc_type = WLC_MOTO;
			auth->wlc_tx_power = data[2] / 2;
			wlc_dbg(" WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , wlc_tx_power %d", auth->wlc_tx_power);
			wls_auth_wls_set_status(wlc, WLC_TX_POWER_CHANGED);
			if (auth->wlc_tx_power <= 5) {
				wlc_dbg(" WLC_TYPE is still MOTO_WLC, power only 5W, Reset ICL to 1A");
				//sw_set_wls_icl(1000);
				//sw_set_aicl_restart();
				wls_auth_set_next_event(auth, MOTOAUTH_EVENT_TX_SN);
			} else {
				wlc_dbg("Get corrent TX CAP : 0x41, capacity %dW, To ask REQ FAN_LEN next", data[2] / 2);
				wls_auth_set_next_event(auth, MOTOAUTH_EVENT_DONE);
			}
		}
		break;
	case QI_ASK_CMD_SN:
		if (data_len != 5) {
			wlc_err(" data_len error with QI_ASK_CMD_SN");
			status = MOTO_WLS_AUTH_FAIL;
			break;
		}
		wlc_dbg(" QI_ASK_CMD_SN: cmd 0x%x, data[0] 0x%x, data[1] 0x%x, data[2] 0x%x, data[3] 0x%x",
			data[0], data[1], data[2], data[3], data[4]);
		auth->wlc_tx_sn = (uint32_t)(data[1] << 24 | (data[2] << 16 | (data[3] << 8 | data[4])));
		wlc_dbg(" WLC_MOTO, NOTIFY_EVENT_WLS_WLC_CHANGE , WLS_WLC_SN %d", auth->wlc_tx_sn);

		wls_auth_wls_set_status(wlc, wlc->data.wlc_status);
		wls_auth_set_next_event(auth, MOTOAUTH_EVENT_DONE);
		break;
	default:
			wlc_dbg(" Not Found CMD 0x%x", data[0]);
			status = MOTO_WLS_AUTH_FAIL;
		}

	return status;
}

int wls_auth_clear(struct moto_wlc *wlc)
{
	wlc_dbg("%s\n", __func__);
	wlc->data.wlc_status = WLC_DISCONNECTED;
	wlc->auth.wlc_tx_id = 0x00;
	wlc->auth.wlc_tx_sn = 0x00;
	wlc->auth.wlc_tx_capability = 0x00;
	wlc->auth.events_thread_running = false;

	wls_auth_ask_clear(&wlc->auth);

	return MOTO_WLS_AUTH_SUCCESS;
}

int wls_auth_init(struct moto_wlc *wlc)
{
	int rt = MOTO_WLS_AUTH_FAIL;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_dbg("%s wlc is NULL or error\n", __func__);
		return rt;
	}
	wlc_dbg("%s\n", __func__);

	init_completion(&wlc->auth.recv_done);
	init_waitqueue_head(&wlc->auth.wait_que);

	wlc->auth.wakelock = wakeup_source_register(NULL, "moto_wls_auth");
	wlc->auth.enable = true;

	wls_auth_clear(wlc);
	kthread_run(wls_auth_events_thread, wlc, "wls_auth_events_thread");

	return rt;
}

int wls_auth_disconnect(struct moto_wlc* wlc)
{
	int rt = MOTO_WLS_AUTH_FAIL;

	if (IS_ERR_OR_NULL(wlc)) {
		wlc_dbg("%s wlc is err or error\n", __func__);
		return rt;
	}

	wlc->auth.hs_st = AUTH_HS_UNKONWN;
	wlc_dbg("%s\n", __func__);
	if (wlc->auth.enable) {
		wls_auth_clear(wlc);
	}

	return MOTO_WLS_AUTH_SUCCESS;
}
