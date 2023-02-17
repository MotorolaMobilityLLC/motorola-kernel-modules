/*
 * Copyright (C) 2020 Motorola Mobility LLC
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
 #ifndef __MMI_SMART_BATTERY_CORE_H__
#define __MMI_SMART_BATTERY_CORE_H__

#include <linux/mmi_gauge_class.h>
#include <linux/power/moto_chg_tcmd.h>

#define mmi_err(chg, fmt, ...)			\
	do {						\
		pr_err("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_warn(chg, fmt, ...)			\
	do {						\
		pr_warn("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_info(chg, fmt, ...)			\
	do {						\
		pr_info("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_dbg(chg, fmt, ...)			\
	do {							\
		if (*chg->debug_enabled)		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

struct mmi_smart_battery {
	struct device		*dev;
	char				*name;

	struct gauge_device	*gauge_dev;
	struct power_supply	*batt_psy;
	struct moto_chg_tcmd_client batt_tcmd_client;

	bool				*debug_enabled;

	int				voltage_now;
	int				current_now;
	int				uisoc;
	int				batt_temp;
	int				batt_tte;
	int				charge_full;
	int				charge_full_design;
	int				charge_counter;
	int				cycle_count;

	int				fake_soc;
	int				fake_temp;
};

extern int mmi_batt_health_check(void);

extern int mmi_charger_update_batt_status(void);

#endif
