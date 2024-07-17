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

#ifndef __MMI_GLINK_DEVICE_H__
#define __MMI_GLINK_DEVICE_H__

#include <linux/of.h>
#include <linux/power_supply.h>

#define DEVICE_ID_SHIFT 8
#define GLINK_SHOW_MAX_SIZE 128
#define MAX_GLINK_NOTIFY_DATA_LEN 8

/*======================Glink Device Common Interface========================*/

enum glink_property_type {
	GLINK_PROP_SKU_TYPE,
	GLINK_PROP_HW_REVISION,
	GLINK_PROP_FACTORY_VERSION,
	GLINK_PROP_FACTORY_MODE,
	GLINK_PROP_DEMO_MODE,
	GLINK_PROP_TCMD,
	GLINK_PROP_REG_ADDRESS,
	GLINK_PROP_REG_DATA,
	GLINK_PROP_BATT_INFO,
	GLINK_PROP_BATT_PROFILE_ID,
	GLINK_PROP_CHG_INFO,
	GLINK_PROP_CHG_BC_PMAX,
	GLINK_PROP_CHG_QC_PMAX,
	GLINK_PROP_CHG_PD_PMAX,
	GLINK_PROP_CHG_WLS_PMAX,
	GLINK_PROP_CHG_SUSPEND,
	GLINK_PROP_CHG_DISABLE,
	GLINK_PROP_CHG_PMIC_ICL,
	GLINK_PROP_CHG_PARTNER_ICL,
	GLINK_PROP_CHG_PARTNER_SOC,
	GLINK_PROP_CHG_PUMP_INFO,
	GLINK_PROP_CHG_PUMP_ENABLE,
	GLINK_PROP_CHG_PUMP_WORK_MODE,
	GLINK_PROP_CHG_PUMP_OVP_GATE,
	GLINK_PROP_CHG_PUMP_MANUAL_MODE,
	GLINK_PROP_CHG_BUCK_INFO,
	GLINK_PROP_CHG_BUCK_ENABLE,
	GLINK_PROP_CHG_BUCK_ENABLE_CHG,
	GLINK_PROP_CHG_BUCK_SUSPEND,
	GLINK_PROP_CHG_BUCK_ICL,
	GLINK_PROP_CHG_BUCK_FCC,
	GLINK_PROP_CHG_BUCK_FV,
	GLINK_PROP_CHG_BCR_INFO,
	GLINK_PROP_CHG_BCR_ENABLE,
	GLINK_PROP_CHG_BCR_AUTO_BSM,
	GLINK_PROP_CHG_BCR_EXT_SW_ENABLE,
	GLINK_PROP_CHG_BCR_CHG_CURR_MAX,
	GLINK_PROP_CHG_BCR_DISCHG_CURR_MAX,
	GLINK_PROP_USB_INFO,
	GLINK_PROP_USB_SUSPEND,
	GLINK_PROP_USB_TYPEC_RESET,
	GLINK_PROP_USB_CLPD_MITIGATE_MODE,
	GLINK_PROP_WLS_EN,
	GLINK_PROP_WLS_VOLT_MAX,
	GLINK_PROP_WLS_CURR_MAX,
	GLINK_PROP_WLS_CHIP_ID,
	GLINK_PROP_WLS_PEN_CTRL,
	GLINK_PROP_WLS_PEN_ID,
	GLINK_PROP_WLS_PEN_SOC,
	GLINK_PROP_WLS_PEN_MAC,
	GLINK_PROP_WLS_PEN_STATUS,
	GLINK_PROP_WLS_RX_FOD_CURR,
	GLINK_PROP_WLS_RX_FOD_GAIN,
	GLINK_PROP_WLS_TX_MODE,
	GLINK_PROP_WLS_FOLIO_MODE,
	GLINK_PROP_WLS_INFO,
	GLINK_PROP_WLS_WLC_LIGHT_CTL,
	GLINK_PROP_WLS_WLC_FAN_SPEED,
	GLINK_PROP_WLS_WLC_TX_TYPE,
	GLINK_PROP_WLS_WLC_TX_POWER,
	GLINK_PROP_WLS_WLC_TX_CAPABILITY,
	GLINK_PROP_WLS_WLC_TX_ID,
	GLINK_PROP_WLS_WLC_TX_SN,
	GLINK_PROP_WLS_WEAK_CHARGE_CTRL,
	GLINK_PROP_THERM_PRIMARY_CHG_CONTROL,
	GLINK_PROP_THERM_SECONDARY_CHG_CONTROL,
	GLINK_PROP_ENCRYT_DATA,
	GLINK_PROP_MAX,
};

enum mmi_sku_type
{
	MMI_SKU_PRC = 0x01,
	MMI_SKU_ROW,
	MMI_SKU_NA,
	MMI_SKU_VZW,
	MMI_SKU_JPN,
	MMI_SKU_ITA,
	MMI_SKU_NAE,
	MMI_SKU_SUPERSET,
};

enum glink_dev_notify_link {
	MMI_GLINK_STATE_DOWN,
	MMI_GLINK_STATE_UP,
};

enum glink_dev_notify_psy {
	MMI_PSY_CHANGE_BATT,
	MMI_PSY_CHANGE_USB,
	MMI_PSY_CHANGE_WLS,
};

enum glink_dev_notify_receiver {
	GLINK_NOTIFY_RECEIVER_PEN_CHG = 0x0,
	GLINK_NOTIFY_RECEIVER_WLS_CHG = 0x1,
	GLINK_NOTIFY_RECEIVER_LINK_USR = 0x2,
	GLINK_NOTIFY_RECEIVER_PSY_USR = 0x3,
	GLINK_NOTIFY_RECEIVER_POLL_TASK = 0x4,
};

struct glink_dev_notify_data {
	u32 receiver;
	u32 data[MAX_GLINK_NOTIFY_DATA_LEN];
};

typedef enum {
	GLINK_DEV_TYPE_BAT = 0,
	GLINK_DEV_TYPE_CHG = 1,
	GLINK_DEV_TYPE_USB = 2,
	GLINK_DEV_TYPE_WLS = 3,
	GLINK_DEV_TYPE_PUMP = 4,
	GLINK_DEV_TYPE_BUCK = 5,
	GLINK_DEV_TYPE_BCR = 6,
	GLINK_DEV_TYPE_NUM
} glink_dev_t;

struct glink_dev_cfg {
	u16 hw_rev;
	u8 sku_type;
	bool softbank;
	bool factory_mode;
	bool factory_version;
};

struct glink_dev;
struct glink_dev_ops {
	int (*init)(struct glink_dev *dev, struct glink_dev_cfg *cfg);
	int (*get_property)(struct glink_dev *dev, u32 property,
				void *val, size_t val_len);
	int (*set_property)(struct glink_dev *dev, u32 property,
				const void *val, size_t val_len);
	int (*notify)(struct glink_dev *dev, unsigned long notification,
				struct glink_dev_notify_data *data);
	int (*deinit)(struct glink_dev *dev);
};

struct glink_dev {
	u32 id;
	glink_dev_t type;
	const char *name;
	atomic_t use_count;
	struct device *dev;
	struct device_node *node;
	struct glink_dev_ops ops;
	struct glink_dev_cfg cfg;
	void *devdata;
};

struct glink_dev *glink_device_get(glink_dev_t type, u32 id);
struct glink_dev *glink_device_get_by_phandle(const struct device_node *np,
				const char *phandle_name, int index);
void glink_device_put(struct glink_dev *dev);
int glink_device_register_notifier(struct notifier_block *nb);
int glink_device_unregister_notifier(struct notifier_block *nb);

/*=========================BATT Specific Interface===========================*/
enum dev_id_batt {
	GLINK_DEV_ID_BATT_DEF = 0,
	GLINK_DEV_ID_BATT_PRI,
	GLINK_DEV_ID_BATT_SEC,
	GLINK_DEV_ID_BATT_NUM,
};

struct battery_info {
	int batt_uv;
	int batt_ua;
	int batt_soc; /* 0 ~ 10000 indicating 0% to 100% */
	int batt_temp; /* hundredth degree */
	int batt_status;
	int batt_soh;
	int batt_cycle_count;
	int batt_full_uah;
	int batt_design_uah;
	int batt_chg_counter;
	int batt_fv_uv;
	int batt_fcc_ua;
};
const char *glink_battery_get_serial_number(struct glink_dev *dev);

/*==========================Charger Specific Interface===========================*/
enum dev_id_chg {
	GLINK_DEV_ID_CHG_DEF = 0,
	GLINK_DEV_ID_CHG_PRI,
	GLINK_DEV_ID_CHG_SEC,
	GLINK_DEV_ID_CHG_NUM,
};

struct charger_info {
	int chrg_uv;
	int chrg_ua;
	int chrg_type;
	int chrg_pmax_mw;
	bool chrg_present;
	bool chrg_otg_enabled;
	bool chrg_wired;
	int chrg_sm_st;
};

/*==========================USB Specific Interface===========================*/
struct usb_info {
	int cid_st;
	int otg_st;
	int vbus_st;
	int cc_st;
	int partner_type;
	int pd_active;
	int legacy_cable;
	int lpd_st;
	int lpd_rsbu1;
	int lpd_rsbu2;
	int lpd_cc1;
	int lpd_cc2;
	int lpd_dp;
	int lpd_dm;
};

/*=========================WLS Specific Interface===========================*/
enum dev_id_wls {
	GLINK_DEV_ID_WLS_DEFAULT = 0,
	GLINK_DEV_ID_WLS_CPS4019,
	GLINK_DEV_ID_WLS_NUM,
};

struct wls_info {
	u32 chip_id;
	u32 mtp_fw_ver;
	u32 irq_status;
	u16 sys_mode;
	u16 op_mode;
	u16 rx_fop;
	u16 rx_vout_mv;
	s16 rx_vrect_mv;
	u16 rx_irect_ma;
	u16 rx_ept;
	u16 rx_ce;
	u32 rx_rp;
	s16 rx_dietemp;
	u16 rx_neg_power;
	u16 rx_vout_max_mv;
	u16 rx_iout_max_ma;
	s16 tx_iin_ma;
	u16 tx_vin_mv;
	u16 tx_vrect_mv;
	u16 tx_det_rx_power;
	u16 tx_power;
	u16 tx_ept;
	s16 power_loss;
	u16 usb_otg;
	u16 wls_boost;
	u16 wls_icl_ma;
	u16 wls_icl_therm_ma;
};

/*==========================External Switched Charge Pump Specific Interface============================*/
enum dev_id_pump {
	GLINK_DEV_ID_PUMP_MASTER = 0,
	GLINK_DEV_ID_PUMP_SLAVE,
	GLINK_DEV_ID_PUMP_NUM,
};

struct pump_info
{
	bool chg_en;
	bool ovpgate;
	bool manual;
	bool otg_en;
	u16 chip_id;
	u8 chg_role;
	u8 work_mode;
	u8 int_stat;
	s32 ibat_ma;
	s32 ibus_ma;
	s32 vbus_mv;
	s32 vout_mv;
	s32 vac_mv;
	s32 vbat_mv;
	s32 vusb_mv;
	s32 vwpc_mv;
	s32 die_temp;
};

/*==========================External Switched Charge Bulk Specific Interface============================*/
enum dev_id_buck {
	GLINK_DEV_ID_BUCK_MASTER = 0,
	GLINK_DEV_ID_BUCK_SLAVE,
	GLINK_DEV_ID_BUCK_NUM,
};

struct buck_info
{
	bool chip_en;
	bool chg_en;
	bool input_ready;
	bool input_suspend;
	u32 input_ilimit;
	u32 input_vlimit;
	u32 batt_fcc;
	u32 batt_fv;
	u32 chg_ctrl;
	u32 chg_stat;
};

/*=====================External Switched Battery Current Regulator Specific Interface===================*/
enum dev_id_bcr {
	GLINK_DEV_ID_BCR_MASTER = 0,
	GLINK_DEV_ID_BCR_SLAVE,
	GLINK_DEV_ID_BCR_NUM,
};

struct bcr_info
{
	bool chg_en;
	s32 ext_sw_st;
	u32 work_mode;
	s32 ibat_ma;
	u32 vbat_mv;
	u32 vchg_mv;
	s32 chg_ilimit;
	s32 dischg_ilimit;
	s32 batt_temp;
	s32 die_temp;
	u32 chg_ctrl;
	u32 chg_stat;
};

#endif
