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

#define MAC_ADDR_LEN 6
#define MAX_PSY_NAME_LEN 16
#define CHG_SHOW_MAX_SIZE 50

#define WLS_PEN_FAILED    -1
#define WLS_PEN_SUCCESS 0
#define PEN_WAIT_EVENTS                             ((1 << PEN_EVENT_INVALID) - 1)

enum pen_notify_event {
	NOTIFY_EVENT_PEN_ID,
	NOTIFY_EVENT_PEN_STATUS,
	NOTIFY_EVENT_PEN_SOC,
	NOTIFY_EVENT_PEN_MAC,
	NOTIFY_EVENT_PEN_TIME_TO,//add for cps call smart_pen
	NOTIFY_EVENT_PEN_ERROR,
};

enum pen_status {
	PEN_STAT_DETACHED,
	PEN_STAT_ATTACHED,
	PEN_STAT_READY,
	PEN_STAT_CHARGING,
	PEN_STAT_DISCHARGING,
	PEN_STAT_CHARGE_FULL,
	PEN_STAT_CHARGE_DISABLED,
	PEN_STAT_MAX,
};

typedef enum _PEN_EVENT_TYPE
{
    /* General Event */
    PEN_EVENT_EXIT,
    PEN_EVENT_ATTACHED,
    PEN_EVENT_DETACHED,
    PEN_EVENT_CHRG_ENABLE,
    PEN_EVENT_CHRG_DISABLE,
    PEN_EVENT_TIMER_TO,
    PEN_EVENT_INVALID
}PEN_EVENT_TYPE;
#if 0
enum pen_error {
	PEN_OK,
	PEN_ERR_UNKNOWN,
	PEN_ERR_NOT_PLUGIN,
	PEN_ERR_TRANSTER_FAILED,
	PEN_ERR_OVERTEMP,
	PEN_ERR_OVERVOLT,
	PEN_ERR_OVERCURR,
	PEN_ERR_UNDERVOLT,
	PEN_ERR_MAX,
};
#endif
typedef enum {
  PEN_OK = 0,
  PEN_ERR_UNKNOWN = -1,
  PEN_ERR_NOT_PLUGIN = -2,
  PEN_ERR_TRANSTER_FAILED = -3,
  PEN_ERR_OVERTEMP = -4,
  PEN_ERR_OVERVOLT = -5,
  PEN_ERR_OVERCURR = -6,
  PEN_ERR_UNDERVOLT = -7,
  PEN_ERR_BAT_OTP = -8,
  PEN_ERR_COIL_OTP = -9,
  PEN_ERR_CHG_TIMEOUT = -10,
  PEN_ERR_CHG_FAILED = -11,
} pen_error;

typedef enum {
  PEN_CTRL_DETACHED,
  PEN_CTRL_ATTACHED,
  PEN_CTRL_CHG_EN,
  PEN_CTRL_CHG_DIS,
  PEN_CTRL_MAX,
} pen_control;

static char *pen_status_maps[] = {
	"detached",
	"attached",
	"ready",
	"charging",
	"discharging",
	"charge_full",
	"charge_disabled",
};

struct pen_mac {
	uint32_t addr[MAC_ADDR_LEN];
};

struct pen_charger_data {
	uint32_t id;
	uint32_t soc;
	int32_t error;
	enum pen_status status;
	struct pen_mac mac;
};

struct pen_charger {
	struct device		*dev;
	struct pen_charger_data	pen_data;
	struct pen_charger_data simulator_data;
	struct power_supply	*pen_psy;
	char			*pen_uenvp[2];
	char			pen_psy_name[MAX_PSY_NAME_LEN];
	struct notifier_block   pen_nb;
	struct notifier_block   pen_psy_nb;

	/* alarm timer */
	struct alarm wls_pen_timer;
	struct timespec64 endtime;
	unsigned int pen_events;

	/* thread related */
	wait_queue_head_t  wait_que;
	bool wls_pen_thread_timeout;
	unsigned int polling_s;
	unsigned int polling_ns;
	bool wls_pen_thread_polling;
	struct wakeup_source *pen_wakelock;
	struct mutex charger_lock;
};

struct moto_wls_pen_ops {
	void *data;
	int client_id;
	void (*wls_pen_attached)(void);
	void (*wls_pen_dettached)(void);
	void (*wls_pen_power_on)(bool en);
	void (*wls_enable_pen_tx)(bool en);
	void (*wls_get_pen_soc)(uint32_t *soc);
	int (*wls_get_pen_mac)(uint32_t *Mac);
	int (*wls_ask_pen_mac)(void);
	int (*wls_ask_pen_soc)(void);
};

#define MAX_OEM_NOTIFY_DATA_LEN		8
struct wls_pen_charger_notify_data {
	uint32_t receiver;
	uint32_t data[MAX_OEM_NOTIFY_DATA_LEN];
};

#define PEN_LOG_NONE    0
#define PEN_LOG_ERR     1
#define PEN_LOG_DEBG    2
#define PEN_LOG_FULL    3

#define ENABLE_PEN_LOG PEN_LOG_FULL

#define wls_pen_log(num, fmt, args...) \
    do { \
            if (ENABLE_PEN_LOG >= (int)num) \
                pr_err(fmt, ##args); \
    } while (0)

#define PEN_INTERVAL 3600      //uint:s ->5 min
void RtcTime_clear(void);
extern void pen_charger_psy_init(void);
extern void wake_up_pen_thread(struct pen_charger *info);
extern int moto_wireless_pen_ops_register(struct moto_wls_pen_ops *ops);
extern int wls_send_oem_notification(int event, struct wls_pen_charger_notify_data *wls_pen_notify);
