/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MOTO_WLS_AUTH2_0__
#define __MOTO_WLS_AUTH2_0__
#include <linux/workqueue.h>
#include <linux/thermal.h>


#define WLS_RX_CAP_15W 15
#define WLS_RX_CAP_10W 10
#define WLS_RX_CAP_8W 8
#define WLS_RX_CAP_5W 5

#ifndef WLS_WLC_POWER_MAX
#define WLS_WLC_POWER_MAX WLS_RX_CAP_15W
#endif


#define MOTO_WLS_AUTH_FAIL	-1
#define MOTO_WLS_AUTH_SUCCESS 0

#define MOTOAUTH_LOG_NONE	0
#define MOTOAUTH_LOG_ERR	1
#define MOTOAUTH_LOG_DEBG	2
#define MOTOAUTH_LOG_FULL	3

#define ENABLE_MOTOAUTH_LOG MOTOAUTH_LOG_FULL

#define motoauth_wls_log(num, fmt, args...) \
	do { \
			if (ENABLE_MOTOAUTH_LOG >= (int)num) \
				pr_err("%s"fmt, __func__, ##args); \
	} while (0)


#define QI_ASK_CMD_TXID (0x3F)
#define QI_ASK_CMD_QFOD (0x48)
#define QI_ASK_CMD_TXCAP (0x41)
#define QI_ASK_CMD_TXCAPABILITY (0x49)
#define QI_ASK_CMD_SN (0x4C)

#define QI_ASK_CMD_SHA1_NUM (0x36)
#define QI_ASK_CMD_SHA1_RESULT (0x38)
#define QI_FSK_CMD_TXCAP (0x41)


typedef enum _MOTOAUTH_EVENT_TYPE
{
	/* General Event */
	MOTOAUTH_EVENT_EXIT,
	MOTOAUTH_EVENT_TIMER_TO,
	MOTOAUTH_EVENT_TX_CAPABILITY,
	MOTOAUTH_EVENT_TX_ID,
	MOTOAUTH_EVENT_TX_CAP,
	MOTOAUTH_EVENT_REQ_FAN_LED,
	MOTOAUTH_EVENT_QFOD,
	MOTOAUTH_EVENT_SHA_ENCRY_NUM,
	MOTOAUTH_EVENT_SHA_ENCRY_RESULT,
	MOTOAUTH_EVENT_FAN_LIGHT,
	MOTOAUTH_EVENT_SET_VOUT,
	MOTOAUTH_EVENT_TX_SN,
	MOTOAUTH_EVENT_DONE,
	MOTOAUTH_EVENT_INVALID
}MOTOAUTH_EVENT_TYPE;

#define WLS_WAIT_EVENTS	((1 << WLS_EVENT_INVALID) - 1)
#define WLS_EVENTTYPE_TO_EVENT(x)	(1 << x)

typedef enum _MOTO_AUTH_STATUS
{
	MOTO_AUTH_EXIT,
	MOTO_AUTH_TX_CAPABILITY,
	MOTO_AUTH_TX_ID,
	MOTO_AUTH_TX_CAP,
	MOTO_AUTH_REQ_FAN_LED,
	MOTO_AUTH_QFOD,
	MOTO_AUTH_SHA_ENCRY_NUM,
	MOTO_AUTH_SHA_ENCRY_RESULT,
	MOTO_AUTH_FAN_LIGHT,
	MOTO_AUTH_SET_VOUT,
	MOTO_AUTH_TX_SN,
	MOTO_AUTH_DONE,
	MOTO_AUTH_INVALID
}MOTO_AUTH_STATUS;

#define MOTOAUTH_WAIT_EVENTS	((1 << MOTOAUTH_EVENT_INVALID) - 1)
#define MOTOAUTH_EVENTTYPE_TO_EVENT(x)	(1 << x)

typedef enum _WLS_EVENT_TYPE
{
	/* General Event */
	WLS_EVENT_EXIT,
	WLS_EVENT_TIMER_TO,
	WLS_EVENT_RX_OCP,
	WLS_EVENT_RX_LDO_ON,
	WLS_EVENT_TX_DET_RX,
	WLS_EVENT_INIT_CALLBACK,
	WLS_EVENT_INVALID
}WLS_EVENT_TYPE;

typedef enum _WLS_WLC_STATUS
{
	WLC_DISCONNECTED,
	WLC_CONNECTED,
	WLC_TX_TYPE_CHANGED,
	WLC_TX_POWER_CHANGED,
	WLC_TX_CAPABILITY_CHANGED,
	WLC_TX_ID_CHANGED,
	WLC_CHGING,
	WLC_CHRG_FULL,
	WLC_ERR_FAN,
	WLC_ERR_LIGHT,
	WLC_ERR_LOWER_EFFICIENCY,
	WLC_ERR_OVERCURR,
	WLC_ERR_OVERVOLT,
	WLC_ERR_OVERTEMP,
	WLC_INVALID
}wlc_status;//WLS_WLC_STATUS;


typedef enum {
	WLC_NONE,
	WLC_BPP,
	WLC_EPP,
	WLC_MOTO = 4,
} mmi_wlc_type;


typedef enum {
	Sys_Op_Mode_AC_Missing = 0,
	Sys_Op_Mode_BPP = 0x1,
	Sys_Op_Mode_EPP = 0x2,
	Sys_Op_Mode_MOTO_WLC = 0x3,
	Sys_Op_Mode_PDDE= 0x4,
	Sys_Op_Mode_TX = 0x8,
	Sys_Op_Mode_TX_FOD = 0x9,
	Sys_Op_Mode_INVALID,
}Sys_Op_Mode;

typedef struct moto_wls_auth
{
	bool enable;
	bool init_flag;
	MOTOAUTH_EVENT_TYPE events;
	wlc_status WLC_STATUS;
	uint32_t WLS_WLC_TYPE;
	uint32_t WLS_WLC_POWER;
	uint32_t WLS_WLC_CAPABILITY;
	uint32_t WLS_WLC_ID;
	uint32_t WLS_WLC_SN;
	bool WLC_VOUT_BOOSTING;
	bool WLS_HS_OK;

	long timer_delay;

	int (*wls_send_ask_packet)(uint8_t *data, uint8_t data_len);
	int (*wls_get_fsk_packet)(uint8_t *data, int data_len);

	int (*wls_update_light_fan)(void);
	int (*wls_get_ldo_on)(void);
	int (*wls_sysfs_notify)(const char *attr);
	int (*wls_set_status)(int status);
	int (*wls_set_wls_icl)(int icl);
	int (*wls_set_aicl_restart)(void);

	struct delayed_work work;
	int work_delay_ms;
	bool work_running;

	/* thread related */
	wait_queue_head_t	wait_que;
	bool events_thread_timeout;
	struct wakeup_source *wakelock;

}MOTO_WLS_AUTH_T;

extern int motoauth_hs_ok_handler(Sys_Op_Mode sys_mode_type);
extern int motoauth_hs_fail_handler(void);
extern int moto_wls_auth_init(MOTO_WLS_AUTH_T *moto_auth);
extern int moto_auth_printf_data(const char *name, uint8_t *data, int data_len);
extern int motoauth_disconnect(MOTO_WLS_AUTH_T *moto_auth);
#endif
