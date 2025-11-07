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
#ifndef __CHARGE_PUMP_H__
#define __CHARGE_PUMP_H__
#include "../device_class.h"

typedef enum msc_vendor {
	MSC_NU,
	MSC_SC,
	MSC_CPS,
	MSC_BQ,
	MSC_VENDOR_INVALID,
} MSC_VENDOR;

struct msc_sc_dev_info
{
	bool chg_en;
	bool ovpgate;
	bool manual;
	bool otg_en;
	s32 chip_id;
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

struct msc_bq_dev_info
{
	bool dischg;
	bool ovp;
	bool ocp;
	bool wdt;
	bool tflt;
	bool online;
	bool ce;
	bool hiz;
	bool bypass;
	bool sc21;
	bool sc41;
	bool acdrv1_en;
	bool acdrv2_en;
	bool reverse_en;
	bool ext_reverse_en;
	s32 chip_id;
	s32 vbat_mv;
	s32 vbus_mv;
	s32 ibat_ma;
	s32 vac2_mv;
	s32 ibus_ma;
	s32 vac1_mv;
	s32 tsbat;
	s32 tdie;
	s32 pin_short_cn;
};

struct charge_pump_dev_info
{
    s32 vendor_id;
    union {
        struct msc_sc_dev_info msc_sc_info;
        struct msc_bq_dev_info msc_bq_info;
    } msc_info;
};

struct charge_pump_glink_dev {
	char			*name;
	struct device		dev;
	DEV_ROLE dev_role;
	int batt_id;
	struct glink_device *glink_dev;
	struct power_supply	*charge_pump_dev_psy;
	struct charge_pump_dev_info 	charge_pump_dev_info;
	struct notifier_block	charge_pump_nb;

	struct timespec64 glink_access_time;
	uint32_t elapsed_ms;
};

struct glink_device *charge_pump_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts);
void charge_pump_glink_device_unregister(void);

#endif
