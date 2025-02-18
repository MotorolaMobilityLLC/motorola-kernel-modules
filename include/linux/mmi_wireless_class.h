#ifndef _MMI_WIRELESS_CLASS_H
#define _MMI_WIRELESS_CLASS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>

//****************system mode***************
#define SYS_MODE_BACK_POWER	1
#define SYS_MODE_TX			2
#define SYS_MODE_RX			3

typedef enum
{
	WLS_EVENT_NULL = 0,
	WLS_EVENT_RX_POWER_ON,
	WLS_EVENT_RX_NEGO_POWER_READY,
	WLS_EVENT_RX_LDO_ON,
	WLS_EVENT_RX_LDO_OFF,
	WLS_EVENT_HS_OK,
	WLS_EVENT_HS_FAIL,
	WLS_EVENT_RX_FSK_PKT,
	WLS_EVENT_TX_DETECTED,//tx_detected
	WLS_EVENT_RX_BACKPOWER_MODE,
	WLS_EVENT_MAX
} WLS_EVENT_T;

struct wls_event_msg
{
	WLS_EVENT_T event;
	int len;
	uint8_t data[32];
};

typedef enum
{
	WLS_FW_UPDATE_IDLE = 0,
	WLS_FW_UPDATE_START,
	WLS_FW_UPDATE_SKIP,
	WLS_FW_UPDATE_SUCCESS,
	WLS_FW_UPDATE_ERR,
	WLS_FW_UPDATE_ERR_ALLOC,
	WLS_FW_UPDATE_ERR_BIN,
	WLS_FW_UPDATE_ERR_SIZE,
	WLS_FW_UPDATE_ERR_I2C,
	WLS_FW_UPDATE_ERR_CRC,
	WLS_FW_UPDATE_ERR_ERASE,
	WLS_FW_UPDATE_ERR_WRITE,
	WLS_FW_UPDATE_MAX
} WLS_FW_UPDATE_T;

typedef enum {
	Sys_Op_Mode_AC_Missing = 0,
	Sys_Op_Mode_BPP = 0x1,
	Sys_Op_Mode_EPP = 0x2,
	Sys_Op_Mode_MOTO_WLC = 0x3,
	Sys_Op_Mode_PDDE= 0x4,
	Sys_Op_Mode_TX = 0x8,
	Sys_Op_Mode_TX_FOD = 0x9,
	Sys_Op_Mode_INVALID,
} Sys_Op_Mode;

struct wireless_device;

struct wls_callback_ops {
	int (*event_handler)(struct wireless_device *wls_dev, struct wls_event_msg *msg);
};

struct wls_rx_ops {
	//rx sys info
	int (*get_chip_id)(struct wireless_device *wls_dev, int *chip_id);
	int (*get_fw_version)(struct wireless_device *wls_dev, int *fw_version);
	int (*get_op_mode)(struct wireless_device *wls_dev, int *mode);
	int (*get_sys_mode)(struct wireless_device *wls_dev, int *sys_mode);
	int (*get_rx_die_temp)(struct wireless_device *wls_dev, int *temp);
	int (*get_mode_select)(struct wireless_device *wls_dev, int *mode_sel);
	int (*get_fw_update_status)(struct wireless_device *wls_dev, int *status);
	int (*get_ce)(struct wireless_device *wls_dev, int *ce);
	int (*get_mcode)(struct wireless_device *wls_dev, uint16_t *mcode);

	//rx charging info
	int (*get_rx_neg_power)(struct wireless_device *wls_dev, int *power);
	int (*get_rx_irect)(struct wireless_device *wls_dev, int *cur);
	int (*get_rx_iout)(struct wireless_device *wls_dev, int *cur);
	int (*get_rx_vrect)(struct wireless_device *wls_dev, int *voltage);
	int (*get_rx_vout)(struct wireless_device *wls_dev, int *voltage);
	int (*get_rx_vout_setting)(struct wireless_device *wls_dev, int *voltage);

	int (*set_rx_vout_target)(struct wireless_device *wls_dev, int voltage);
	int (*set_irq_enable)(struct wireless_device *wls_dev, bool en);
	int (*set_mode_select)(struct wireless_device *wls_dev, bool on);
	int (*set_fw_update)(struct wireless_device *wls_dev, bool force);
	int (*set_mc_det)(struct wireless_device *wls_dev, bool on);
	int (*set_lowpower_mode)(struct wireless_device *wls_dev, bool on);

	bool (*check_ldo_on)(struct wireless_device *wls_dev);
	bool (*check_rx_power_on)(struct wireless_device *wls_dev);
	bool (*check_auth_handshake)(struct wireless_device *wls_dev);
	bool (*check_dump_info)(struct wireless_device *wls_dev);

	int (*send_ask_packet)(struct wireless_device *wls_dev, uint8_t *data, int data_len);
};

struct wls_tx_ops {
	//tx mode
	int (*set_tx_mode)(struct wireless_device *wls_dev, bool tx_mode);
	int (*get_tx_mode)(struct wireless_device *wls_dev, bool *tx_mode);

	//tx settings
	int (*get_tx_iout)(struct wireless_device *wls_dev, int *cur);
	int (*get_tx_vout)(struct wireless_device *wls_dev, int *vol);
	int (*get_tx_power)(struct wireless_device *wls_dev, int *power);
};

struct wireless_properties {
	const char *alias_name;
};

struct wireless_device {
	struct device dev;
	struct wireless_properties props;
	void *driver_data;

	const struct wls_rx_ops *rx_ops;
	const struct wls_tx_ops *tx_ops;
	const struct wls_callback_ops *callback_ops;
};

static inline void *wireless_dev_get_drvdata(const struct wireless_device *wls_dev)
{
	if (IS_ERR_OR_NULL(wls_dev))
		return NULL;
	return wls_dev->driver_data;
}

static inline bool wireless_dev_set_drvdata(struct wireless_device *wls_dev, void *data)
{
	if (IS_ERR_OR_NULL(wls_dev))
		return false;
	wls_dev->driver_data = data;

	return true;
}

static inline void *wireless_get_data( struct wireless_device *wls_dev)
{
	return dev_get_drvdata(&wls_dev->dev);
}

extern struct wireless_device *wireless_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct wls_rx_ops *rx_ops,
		const struct wls_tx_ops *tx_ops,
		const struct wireless_properties *props);

extern void wireless_device_unregister(struct wireless_device *wls_dev);

extern struct wireless_device *get_wireless_by_name(const char *name);

extern bool wireless_dev_set_callback(struct wireless_device *wls_dev, void *callback_ops);
extern int wls_dev_send_event(struct wireless_device *wls_dev, struct wls_event_msg *msg);

extern int wls_rx_get_chip_id(struct wireless_device *wls_dev, int *chip_id);
extern int wls_rx_get_fw_version(struct wireless_device *wls_dev, int *fw_version);
extern int wls_rx_get_rx_neg_power(struct wireless_device *wls_dev, int *power);
extern int wls_rx_get_op_mode(struct wireless_device *wls_dev, int *op_mode);
extern int wls_rx_get_sys_mode(struct wireless_device *wls_dev, int *sys_mode);
extern int wls_rx_get_mode_select(struct wireless_device *wls_dev, int *mode_sel);
extern int wls_rx_get_fw_update_status(struct wireless_device *wls_dev, int *status);
extern int wls_rx_get_ce(struct wireless_device *wls_dev, int *ce);
extern int wls_rx_get_mcode(struct wireless_device *wls_dev, uint16_t *mcode);

extern int wls_rx_get_rx_irect(struct wireless_device *wls_dev, int *cur);
extern int wls_rx_get_rx_iout(struct wireless_device *wls_dev, int *cur);
extern int wls_rx_get_rx_vrect(struct wireless_device *wls_dev, int *voltage);
extern int wls_rx_get_rx_vout(struct wireless_device *wls_dev, int *voltage);
extern int wls_rx_get_rx_vout_setting(struct wireless_device *wls_dev, int *voltage);

extern int wls_rx_set_irq_enable(struct wireless_device *wls_dev, bool en);
extern int wls_rx_set_mode_select(struct wireless_device *wls_dev, bool on);
extern int wls_rx_set_fw_update(struct wireless_device *wls_dev, bool force);
extern int wls_rx_set_mc_det(struct wireless_device *wls_dev, bool on);
extern int wls_rx_set_lowpower_mode(struct wireless_device *wls_dev, bool on);

extern bool wls_rx_check_ldo_on(struct wireless_device *wls_dev);
extern bool wls_rx_check_dump_info(struct wireless_device *wls_dev);

extern int wls_rx_send_ask_packet(struct wireless_device *wls_dev, uint8_t *data, int data_len);
extern int wls_get_message_size(int header);

#endif /*_MMI_WIRELESS_CLASS_H*/
