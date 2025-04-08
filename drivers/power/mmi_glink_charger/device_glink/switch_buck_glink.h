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
 #ifndef __SWITCH_BUCK_GLINK_H__
#define __SWITCH_BUCK_GLINK_H__

struct buck_dev_info
{
    u32  usb_iin;
    u32  usb_vout;
    u32  usb_suspend;
    u32  batt_fcc;
    u32  batt_fv;
    u32  chg_en;
    u32  chg_st;
    u32  chg_fault;
    u32  vbus_ctrl;
    u32  ce_en;
    u32  battfet_dis;
    u32  jeita_en;
    u32  vbus_gd;
    u32  input_det;
};

struct buck_glink_dev {
	char			*name;
	struct device		dev;
	DEV_ROLE dev_role;
	int batt_id;
	struct glink_device *glink_dev;
	struct power_supply *buck_dev_psy;
	struct buck_dev_info 	buck_info;
	struct notifier_block	buck_nb;

	struct timespec64 glink_access_time;
	uint32_t elapsed_ms;
};

struct glink_device *switch_buck_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void switch_buck_device_unregister(void);

#endif
