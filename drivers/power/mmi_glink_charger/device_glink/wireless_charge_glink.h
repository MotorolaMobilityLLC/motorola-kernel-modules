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
 #ifndef __WIRELESS_CHARGE_GLINK_H__
#define __WIRELESS_CHARGE_GLINK_H__
#include "../battery_host.h"
#include "../device_class.h"

enum wls_notify_event {
  NOTIFY_EVENT_WLS_RX_CONNECTED,
  NOTIFY_EVENT_WLS_RX_OVERTEMP,
  NOTIFY_EVENT_WLS_CHANGE,
  NOTIFY_EVENT_WLS_ERROR,
  NOTIFY_EVENT_WLS_WLC_CHANGE,
  NOTIFY_EVENT_WLS_RX_DEV_INFO_UPDATE,
};

struct wls_dump
{
    u32  chip_id;
    u32  mtp_fw_ver;
    u32  irq_status;
    u16  sys_mode;
    u16  op_mode;
    u16  rx_fop;
    u16  rx_vout_mv;
    s16  rx_vrect_mv;
    u16  rx_irect_ma;
    u16  rx_ept;
    u16  rx_ce;
    u32  rx_rp;
    s16  rx_dietemp;
    u16  rx_neg_power;
    s16  tx_iin_ma;
    u16  tx_vin_mv;
    u16  tx_vrect_mv;
    u16  tx_det_rx_power;
    u16  tx_power;
    u16  tx_ept;
    s16  power_loss;
    u16  usb_otg;
    u16  wls_boost;
    u16  wls_icl_ma;
    u16  wls_icl_therm_ma;
    u16  wls_mc_st;
    u16  vdd5v_st;
};

struct wireless_glink_dev {
	char			*name;
	struct device		dev;
	struct glink_device *glink_dev;
	struct power_supply	*wls_dev_psy;

	struct notifier_block	wls_nb;
	struct notifier_block	wls_glink_nb;
	struct notifier_block	mc_nb;
	struct dentry		*wls_debug_root;
	u32				tx_mode;
	u32				folio_mode;
	u32				wlc_light_ctl;
	u32				wlc_fan_speed;
	u32				wlc_status;
	u32				wlc_tx_type;
	u32				wlc_tx_power;
	u32				wlc_tx_capability;
	u32				wlc_tx_id;
	u32				wlc_tx_sn;
	u32				wls_curr_max;
	int				rx_connected;
	u32				rx_dev_mfg;
	u32				rx_dev_type;
	u32				rx_dev_id;
	u32				mc_status;
	struct dentry		*debug_root;
};

struct glink_device *wireless_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void wireless_glink_device_unregister(void);
#endif
