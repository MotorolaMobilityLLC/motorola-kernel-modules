#ifndef __MOTO_WLC2_H
#define __MOTO_WLC2_H
#include <mtk_charger.h>
#include <mtk_charger_algorithm_class.h>
#include <linux/mmi_wireless_class.h>
#include <linux/phone_case_detection_notify.h>

#define NORMAL_FW_UPDATE 1
#define FORCE_FW_UPDATE 2
#define VBUS_VALID_MV 4600 //If vbus >= 4.6V,the vbus is valid.

#define RX_ONLINE_CHECK_MS 500
#define WLC_V_CHARGER_MIN 4600000 /* 4.6 V */
/*wireless input current and charging current*/
#define WIRELESS_CHARGER_MAX_CURRENT			3000000
#define WIRELESS_CHARGER_MAX_INPUT_CURRENT		1150000
#define DISABLE_VBAT_THRESHOLD -1
#define ECABLEOUT	1	/* cable out */
#define EHAL		2	/* hal operation error */

#define WLS_ICL_INCREASE_STEP_MA 100 /*100mA*/
#define WLS_ICL_INCREASE_DELAY 100 /*100ms*/
#define WLS_BPP_ICL_MAX_MA 1000
#define WLS_BPP_ICL_MIN_MA 300
#define WLS_BPP_ROD_THRESHOLD_CURRENT_MAX 850 /*mA*/
#define WLS_BPP_ROD_THRESHOLD_CURRENT_MIN 750 /*mA*/
#define WLS_BPP_ROD_DETECT_COUNT_MAX 3
#define WLS_ROD_STOP_BATERY_SOC 90
#define WLS_ROD_STOP_TIME (60*1000*1000*1000uL) /*60s*/

#define WLS_EPP_ROD_DETECT_COUNT_MAX 3
#define WLS_EPP_ROD_THRESHOLD_12V 9600 /*mV*/
#define WLS_EPP_ROD_THRESHOLD_10V 9000 /*mV*/
#define WLS_EPP_ROD_THRESHOLD_9V 7800 /*mV*/

#define WLS_MC_DET_CNT_MAX (60) /*60*/
#define WLS_MC_BPP_ICL_STEP_DELAY 500 /*500ms*/
#define WLS_MC_BPP_ICL_THRESHOLD 800 /*800mA*/
#define WLS_MC_EPP_ICL_DEFAULT 900000 /*900mA*/
#define WLS_MC_ICL_STEP 100000 /*100mA*/

#define QI_ASK_CMD_ADJUST_FOD (0x06)
#define QI_ASK_CMD_TXID (0x3F)
#define QI_ASK_CMD_QFOD (0x48)
#define QI_ASK_CMD_TXCAP (0x41)
#define QI_ASK_CMD_TXCAPABILITY (0x49)
#define QI_ASK_CMD_SN (0x4C)

#define QI_ASK_CMD_SHA1_NUM (0x36)
#define QI_ASK_CMD_SHA1_RESULT (0x38)
#define QI_FSK_CMD_TXCAP (0x41)

#define MOTO_15W_TX_ID (353)
#define MOTO_50W_TX_ID (337)
#define MOTO_TX_MCODE (0x004F)

#define WLS_RX_CAP_15W 15
#define WLS_RX_CAP_10W 10
#define WLS_RX_CAP_7W 7
#define WLS_RX_CAP_5W 5

typedef enum
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
} WLS_WLC_STATUS;

typedef enum {
	WLC_NONE,
	WLC_BPP,
	WLC_EPP,
	WLC_MOTO = 4,
} wlc_type_t;

typedef enum
{
	AUTH_HS_UNKONWN = 0,
	AUTH_HS_FAIL,
	AUTH_HS_OK,
} AUTH_HANDSHAKE_T;

typedef enum {
	MMI_DOCK_LIGHT_OFF = 0x10,
	MMI_DOCK_LIGHT_ON = 0x20,
	MMI_DOCK_LIGHT_BREATH_2S = 0x30,
	MMI_DOCK_LIGHT_BREATH_4S = 0x40,
	MMI_DOCK_LIGHT_DEFAULT = MMI_DOCK_LIGHT_BREATH_4S,
} MMI_DOCK_LIGHT_CTRL_T;

/* value = fan speed / 100 */
typedef enum {
	MMI_DOCK_FAN_SPEED_OFF= 0,
	MMI_DOCK_FAN_SPEED_LOW = 20,//2000
	MMI_DOCK_FAN_SPEED_HIGH = 40,//4000
	MMI_DOCK_FAN_DEFAULT = MMI_DOCK_FAN_SPEED_HIGH,
} MMI_DOCK_FAN_SPEED_T;

typedef enum {
	MOTOAUTH_EVENT_TX_CAPABILITY = 0x00,
	MOTOAUTH_EVENT_TX_ID,
	MOTOAUTH_EVENT_TX_CAP,
	MOTOAUTH_EVENT_TX_SN,
	MOTOAUTH_EVENT_DONE,
	MOTOAUTH_EVENT_MAX
} MOTOAUTH_EVENT_TYPE;

#define MOTOAUTH_EVENT_START MOTOAUTH_EVENT_TX_CAPABILITY

#define WLC_ERROR_LEVEL	1
#define WLC_INFO_LEVEL	2
#define WLC_DEBUG_LEVEL	3

#define CHARGER_STATE_NUM 8

extern int wlc_get_debug_level(void);
#define wlc_err(fmt, args...)					\
do {								\
	if (wlc_get_debug_level() >= WLC_ERROR_LEVEL) {	\
		pr_info(fmt, ##args);				\
	}							\
} while (0)

#define wlc_info(fmt, args...)					\
do {								\
	if (wlc_get_debug_level() >= WLC_ERROR_LEVEL) { \
		pr_info(fmt, ##args);			\
	}							\
} while (0)

#define wlc_dbg(fmt, args...)					\
do {								\
	if (wlc_get_debug_level() >= WLC_ERROR_LEVEL) {	\
		pr_info(fmt, ##args);				\
	}							\
} while (0)


enum wlc_state_enum {
	WLC_HW_UNINIT = 0,
	WLC_HW_FAIL,
	WLC_HW_READY,
	WLC_TA_NOT_SUPPORT,
	WLC_RUN,
	WLC_DONE,
};

enum wlc_mode_switch_enum {
	WLC_SWITCH_IDLE = 0,
	WLC_SWITCH_TO_BPP,
	WLC_SWITCH_TO_EPP,
	WLC_SWITCH_RUN,
	WLC_SWITCH_TIME_OUT,
	WLC_SWITCH_FAIL,
	WLC_SWITCH_DONE,
};

enum mc_icl_state_enum {
	MC_ICL_IDLE = 0,
	MC_ICL_RUN,
	MC_ICL_DONE,
};

struct wlc_profile {
	unsigned int vbat;
	unsigned int vchr;
};

struct wireless_ctl
{
	bool inhibit_hook;
	bool tx_mode;
	bool bpp_icl_done;
	bool epp_icl_done;
	bool mc_icl_done;
	bool set_icl_done;
	bool rx_vout_change_done;
	bool rx_offset;
	bool mode_select_force;
	bool enable_rod;
	bool rod_stop;
	bool rx_power_on;
	bool rx_ldo_on;
	bool mc_status;
	bool factory_wls_en;
	bool otg_boost_on;
	bool fw_update_force;
	bool fw_uploading;
	int fw_update_retry_cnt;

	int icl_target; //uA
	int cc_target; //uA
	int vout_target; //uV
	int input_current_max; //mA
	int rx_vout_set; //mV
	int rx_vout_threshold;
	int fan_speed;
	int light_level;
	int rx_offset_detect_count;
	int rx_ldo_detect_count;
	ktime_t rx_start_ktime;
	enum wlc_mode_switch_enum mode_switch;
	int ce_det_count;
	int mc_icl_state;
	int mc_icl_max_uA;
};

struct wireless_data
{
	bool moto_stand;
	uint16_t mcode;
	int uisoc;
	int chip_id;
	int rx_irect;
	int rx_vrect;
	int rx_vout;
	int rx_vout_set;
	int rx_vout_threshold;
	int rx_neg_power;
	int rx_fop;
	int rx_ept;
	int rx_ce;
	int rx_dietmp;

	int wls_online;
	int wlc_status;
	int wls_fw_version;
	int wlc_power;
	int vbus_select;

	int mode_type;
	int qi_mode_type;
};

struct wireless_config
{
	int chip_id;
	int MaxV;
	int MaxI;
	int MaxPower;

	int bpp_icl_min_uA;
	int bpp_icl_max_uA;
	int bpp_icl_step_uA;
	int bpp_step_delay_ms;
	int bpp_switch_time_ms;

	int epp_icl_min_uA;
	int epp_icl_max_uA;
	int epp_icl_step_uA;
	int epp_step_delay_ms;
	int epp_switch_time_ms;

	int rod_stop_battery_soc;
	int bootmode;
	bool wls_cert_mode;
	bool secure_hardware;
	bool factory_mode;
	int enable_bat_full_stop_epp;
	int enable_stop_epp;
	int enable_rx_offset_detect;

	int fw_update_soc_limit;
	bool wls_tx_support;
	bool wls_boost_support;
	bool config_otg_support;
	uint32_t config_otg_vout; //uV
	uint32_t config_otg_iout; //uA

	int enable_wls_auto_switch;
	int wls_auto_switch_overtemp;

	int enable_wls_auto_stop;
	int wls_auto_stop_overtemp;
	int wls_auto_stop_undertemp;

	bool mc_support;
};

struct wireless_auth
{
	bool enable;
	bool init_flag;
	bool vout_boosting;
	bool hs_ok;
	bool events_thread_running;
	bool auth_done;
	bool auth_error;
	struct completion recv_done;

	MOTOAUTH_EVENT_TYPE event;
	MOTOAUTH_EVENT_TYPE next_event;
	AUTH_HANDSHAKE_T hs_st;

	int wlc_type;
	int wlc_tx_id;
	int wlc_tx_sn;
	int wlc_tx_capability;
	int wlc_tx_power;

	long timer_delay;
	int timeout_retry;
	struct delayed_work work;
	int work_delay_ms;
	bool work_running;
	/* thread related */
	wait_queue_head_t wait_que;
	bool events_thread_timeout;
	struct wakeup_source *wakelock;
};


struct moto_wlc {
	struct platform_device *pdev;
	struct chg_alg_device *alg;
	struct wireless_device *wls_dev;
	struct charger_device *chg1_dev;
	struct notifier_block hall_nb;

	struct mutex access_lock;
	struct wakeup_source *suspend_lock;
	struct wakeup_source *fw_update_wake_lock;
	struct mutex cable_out_lock;
	struct mutex data_lock;
	bool is_cable_out_occur; /* Plug out happened while detect PE+20 */
	struct power_supply *bat_psy;
	struct power_supply *wls_psy;
	struct power_supply_desc wls_psd;
	int idx;
	int vbus;

	int min_charger_voltage;
	int ref_vbat; /* Vbat with cable in */
	/* module parameters*/
	int cv;
	int old_cv;
	int wlc_6pin_en;
	int stop_6pin_re_en;
	int input_current_limit1;
	int input_current_limit2;
	int charging_current_limit1;
	int charging_current_limit2;

	/* current IC setting */
	int input_current1;
	int charging_current1;
	int input_current2;
	int charging_current2;


	enum wlc_state_enum state;
	int wls_control_en;
	int mmi_fcc;
	/*wireless charger*/
	int wireless_charger_max_current;
	int wireless_charger_max_input_current;
	int input_current_limit;
	bool cable_ready;

	/*wireless thermal*/
	struct thermal_cooling_device *tcd;
	int max_state;
	int cur_state;

	struct mmi_thermal_config *wlc_thermal_com;
	int num_wlc_thermal_com;

	bool wls_online;

	struct wls_callback_ops callback_ops;

	struct wireless_ctl ctl;
	struct wireless_config config;
	struct wireless_data data;
	struct wireless_auth auth;
	struct moto_chg_tcmd_client wls_tcmd_client;

	struct workqueue_struct *wls_wq;
	struct delayed_work fw_update_work;
	struct delayed_work bpp_icl_work;
	struct delayed_work mc_icl_work;
	struct delayed_work light_fan_work;
	struct delayed_work offset_detect_work;
	struct delayed_work mode_switch_work;
	struct delayed_work dump_info_work;
	struct delayed_work rx_online_check;
};

extern int wlc_hal_init_hardware(struct chg_alg_device *alg);
extern int wlc_hal_get_boot_mode(struct chg_alg_device *alg);
extern int wlc_hal_get_uisoc(struct chg_alg_device *alg);
extern int wlc_hal_get_charger_type(struct chg_alg_device *alg);
extern int wlc_hal_set_mivr(struct chg_alg_device *alg,
	enum chg_idx chgidx, int uV);

extern bool wlc_hal_is_chip_enable(struct chg_alg_device *alg,
	enum chg_idx chgidx);
extern int wlc_hal_enable_charger(struct chg_alg_device *alg,
	enum chg_idx chgidx, bool en);
extern int wlc_hal_reset_ta(struct chg_alg_device *alg, enum chg_idx chgidx);
extern int wlc_hal_get_vbus(struct chg_alg_device *alg);
extern int wlc_hal_get_vbat(struct chg_alg_device *alg);
extern int wlc_hal_get_ibat(struct chg_alg_device *alg);
extern int wlc_hal_get_bat_property(struct chg_alg_device *alg,
			enum power_supply_property property,
			union power_supply_propval *prop);

extern int wlc_hal_set_charging_current(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 ua);
extern int wlc_hal_get_charging_current(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 *ua);
extern int wlc_hal_set_input_current(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 ua);
extern int wlc_hal_get_mivr_state(struct chg_alg_device *alg,
	enum chg_idx chgidx, bool *in_loop);

extern int wlc_hal_enable_vbus_ovp(struct chg_alg_device *alg, bool enable);
extern int wlc_hal_set_cv(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 uv);

extern int wlc_hal_get_min_charging_current(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 *uA);
extern int wlc_hal_get_min_input_current(struct chg_alg_device *alg,
	enum chg_idx chgidx, u32 *uA);

extern int wlc_hal_vbat_mon_en(struct chg_alg_device *alg,
	enum chg_idx chgidx, bool en);
extern int wlc_hal_is_charger_enable(struct chg_alg_device *alg,
	enum chg_idx chgidx, bool *en);
extern int wlc_hal_get_log_level(struct chg_alg_device *alg);

extern int wlc_hal_get_batt_temp(struct chg_alg_device *alg);
extern int wlc_hal_get_batt_cv(struct chg_alg_device *alg);

extern int wls_chg_register_psy(struct moto_wlc *wlc);
extern int wls_chg_current_select(struct moto_wlc *wlc, int *icl, int *vbus);
extern int wls_chg_notify_st_changed(struct moto_wlc *wlc, int st);
extern int wls_chg_mmi_mux_chan_set(enum mmi_mux_channel channel, bool on);
extern int wls_chg_get_mux_channel(int *mux_channel);
extern void wlc_chg_bpp_mode_icl_work(struct work_struct *work);
extern int wls_chg_notify_otg_plugin(struct moto_wlc *wlc, bool on);
extern void wlc_chg_mc_icl_work(struct work_struct *work);
extern void wlc_chg_dump_info_work(struct work_struct *work);

extern int wls_chg_power_off(struct moto_wlc *wlc);
extern int wls_device_node_create(struct device *dev);
extern void wls_device_fw_update_work(struct work_struct *work);
extern int wls_device_tcmd_register(struct moto_wlc *wlc);
extern int wls_device_update_light_fan(struct moto_wlc *wlc);
extern void wls_device_light_fan_work(struct work_struct *work);
extern int wls_device_uisoc_change(struct moto_wlc *wlc, int uisoc);
extern void wls_device_offset_detect_work(struct work_struct *work);
extern void wlc_chg_rx_online_check(struct work_struct *work);
extern int wls_device_init_light_fan(struct moto_wlc *wlc);
extern void wls_device_mode_switch_work(struct work_struct *work);
extern int wls_device_start_mode_switch(struct moto_wlc *wlc, char *str, int op_mode);
extern int wls_device_set_mode_select(struct moto_wlc *wlc, char *str, bool mode);

extern int wls_config_parse_dts(struct moto_wlc *wlc, struct device *dev);
extern bool wls_config_is_charge_only_mode(struct moto_wlc *wlc);

extern int wls_auth_init(struct moto_wlc *wlc);
extern int wls_auth_disconnect(struct moto_wlc *wlc);
extern int wls_auth_hs_ok_handler(struct moto_wlc *wlc, int op_mode);
extern int wls_auth_decode_fsk_packet(struct moto_wlc *wlc, uint8_t *data, int data_len);


#endif /* __MOTO_WLC2_H */
