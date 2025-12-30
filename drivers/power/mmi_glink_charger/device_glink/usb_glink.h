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
#ifndef __USB_GLINK_H__
#define __USB_GLINK_H__

enum {
	NOTIFY_EVENT_USB_LPD_STATUS,
	NOTIFY_EVENT_USB_CID_STATUS,
	NOTIFY_EVENT_USB_VBUS_STATUS,
};

enum {
	LPD_MITIGATE_DRP,
	LPD_MITIGATE_SNK,
	LPD_MITIGATE_SRC,
	LPD_MITIGATE_DISABLE,
	LPD_MITIGATE_INVALID,
};

enum typec_partner_type {
	TYPEC_PARTNER_NONE,
	TYPEC_PARTNER_UNKNOWN,
	TYPEC_PARTNER_SNK_USB_SDP,
	TYPEC_PARTNER_SNK_USB_OCP,
	TYPEC_PARTNER_SNK_USB_CDP,
	TYPEC_PARTNER_SNK_USB_DCP,
	TYPEC_PARTNER_SNK_USB_FLOAT,
	TYPEC_PARTNER_SNK_TYPEC_DEFAULT,
	TYPEC_PARTNER_SNK_TYPEC_RP_MEDIUM_1P5A,
	TYPEC_PARTNER_SNK_TYPEC_RP_HIGH_3A,
	TYPEC_PARTNER_SNK_DEBUG_ACCESS,
	TYPEC_PARTNER_SNK_USB_QC_2P0,
	TYPEC_PARTNER_SNK_USB_QC_3P0,
	TYPEC_PARTNER_SNK_USB_QC_3P5,
	TYPEC_PARTNER_SNK_UFCS,
	TYPEC_PARTNER_SNK_PD,
	TYPEC_PARTNER_SNK_PPS,
	TYPEC_PARTNER_SRC_TYPEC_POWERCABLE,              // RD-RA
	TYPEC_PARTNER_SRC_TYPEC_UNORIENTED_DEBUG_ACCESS, // RD/RD
	TYPEC_PARTNER_SRC_TYPEC_AUDIO_ACCESS,            // RA/RA
	TYPEC_PARTNER_WLS_SRC_BPP,
	TYPEC_PARTNER_WLS_SNK_BPP,
	TYPEC_PARTNER_WLS_SNK_EPP,
	TYPEC_PARTNER_WLS_SNK_PDDE,
	TYPEC_PARTNER_INVALID,
};

struct usb_info {
	int cid_st;
	unsigned int vbus_st;
	bool otg_st;
	bool cc_st;
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

struct usb_glink_dev {
	char *name;
	struct usb_info usb_info;
	struct delayed_work usb_work;
	struct glink_device *dev;
	struct mmi_glink_chip *mmi_chip;
	struct notifier_block	usb_nb;
	struct notifier_block	usb_mmi_nb;

	int otp_en_gpio;
	bool therm_supported;
	unsigned long therm_state;
	struct thermal_cooling_device *cdev;

	u32 lpd_mitigate_mode;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	char *uenvp[2];

	bool init_done;
	bool init_lpd_done;
};

struct glink_device *usb_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void usb_glink_device_unregister(void);

#endif
