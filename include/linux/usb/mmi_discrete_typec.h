/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MMI_DISCRETE_TYPE_C_H
#define MMI_DISCRETE_TYPE_C_H

/* Indicates USB Type-C CC connection status */
enum mmi_power_supply_typec_mode {
	MMI_POWER_SUPPLY_TYPEC_NONE,

	/* Acting as source */
	MMI_POWER_SUPPLY_TYPEC_SINK,		/* Rd only */
	MMI_POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE,	/* Rd/Ra */
	MMI_POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY,/* Rd/Rd */
	MMI_POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER,	/* Ra/Ra */
	MMI_POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY,	/* Ra only */

	/* Acting as sink */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT,	/* Rp default */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM,	/* Rp 1.5A */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH,		/* Rp 3A */
	MMI_POWER_SUPPLY_TYPEC_NON_COMPLIANT,
};

enum {
	MMI_POWER_SUPPLY_PD_INACTIVE = 0,
	MMI_POWER_SUPPLY_PD_ACTIVE,
	MMI_POWER_SUPPLY_PD_PPS_ACTIVE,
};

#endif
