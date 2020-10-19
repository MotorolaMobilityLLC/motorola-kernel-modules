/*
 * Copyright (C) 2018 Motorola Mobility LLC
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/mmi_device.h>
#include "mmi_info.h"

struct mmi_chosen_info mmi_chosen_data;
static char *bootargs_str;

/**
 *  mmi_device_is_available - check if a device is available for use
 *
 *  @device: Node to check for availability, with locks already held
 *
 *  Returns true if the status property
 *  -- is absent,
 *  -- refers to another property using string list as following
 *     mmi,status = <path>, <okay property>, [<contains>,] <value1>[, <value2>...];
 *     and at least one <value> in the list of values matches
 *     value of /<path>/<okay property>.
 *     Partial matches accepted when "contains" provided.
 *  false otherwise
 */
bool mmi_device_is_available(struct device_node *np)
{
	bool found;
	int len, operation;
	const char *path, *mmi_dts;
	const char *mmi_dts_val, *val;
	struct property *prop;
	struct device_node *mmi_np;

	if (np == NULL)
		return true;

	prop = of_find_property(np, "mmi,status", &len);
	if (prop == NULL || len < 0) {
		return true;
	}
	path = prop->value;

	mmi_np = of_find_node_by_path(path);
	if (mmi_np == NULL)
		return false;

	mmi_dts = of_prop_next_string(prop, path);
	if (mmi_dts == NULL)
		return false;

	mmi_dts_val = of_get_property(mmi_np, mmi_dts, &len);
	if (mmi_dts_val == NULL || len <= 0)
		return false;

	val = mmi_dts;
	operation = 0;
	found = false;
	while ((val = of_prop_next_string(prop, val))) {
		if (val == NULL)
			return false;

		if (!operation) {
			operation++;
			if (!strcmp(val, "contains")) {
				operation++;
				continue;
			}
		}
		if (operation == 1) {
			if (!strcmp(val, mmi_dts_val))
				found = true;
		} else if (strstr(mmi_dts_val, val)) {
			found = true;
		}
		if (found)
			return true;
	}
	return false;
}
EXPORT_SYMBOL(mmi_device_is_available);

int mmi_get_bootarg(char *key, char **value)
{
	const char *bootargs_tmp = NULL;
	char *idx = NULL;
	char *kvpair = NULL;
	int err = 1;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_tmp_len = 0;

	if (n == NULL)
		goto err;

	if (of_property_read_string(n, "bootargs", &bootargs_tmp) != 0)
		goto putnode;

	bootargs_tmp_len = strlen(bootargs_tmp);
	if (!bootargs_str) {
		/* The following operations need a non-const
		 * version of bootargs
		 */
		bootargs_str = kzalloc(bootargs_tmp_len + 1, GFP_KERNEL);
		if (!bootargs_str)
			goto putnode;
	}
	strlcpy(bootargs_str, bootargs_tmp, bootargs_tmp_len + 1);

	idx = strnstr(bootargs_str, key, strlen(bootargs_str));
	if (idx) {
		kvpair = strsep(&idx, " ");
		if (kvpair)
			if (strsep(&kvpair, "=")) {
				*value = strsep(&kvpair, " ");
				if (*value)
					err = 0;
			}
	}

putnode:
	of_node_put(n);
err:
	return err;
}

static void mmi_of_populate_setup(void)
{
	const char *temp;
	struct device_node *n = of_find_node_by_path("/chosen");

	if (n == NULL)
		return;

	of_property_read_u32(n, "mmi,powerup_reason",
		&mmi_chosen_data.powerup_reason);
	of_property_read_u32(n, "mmi,mbmversion", &mmi_chosen_data.mbmversion);
	of_property_read_u32(n, "mmi,boot_seq", &mmi_chosen_data.boot_seq);
	of_property_read_u32(n, "mmi,prod_id", &mmi_chosen_data.product_id);
	of_property_read_u32(n, "linux,hwrev", &mmi_chosen_data.system_rev);
	of_property_read_u32(n, "linux,seriallow",
		&mmi_chosen_data.system_serial_low);
	of_property_read_u32(n, "linux,serialhigh",
		&mmi_chosen_data.system_serial_high);
	if (of_property_read_string(n, "mmi,baseband", &temp) == 0)
		strlcpy(mmi_chosen_data.baseband, temp, BASEBAND_MAX_LEN);
	if (of_property_read_string(n, "mmi,msm_hw", &temp) == 0)
		strlcpy(mmi_chosen_data.msm_hw, temp, MSMHW_MAX_LEN);

	of_node_put(n);
}

static int __init mmi_hw_info_init(void)
{
	mmi_of_populate_setup();
	mmi_storage_info_init();
	mmi_ram_info_init();
	mmi_unit_info_init();
	mmi_boot_info_init();
	return 0;
}

static void mmi_hw_info_exit(void)
{
	mmi_storage_info_exit();
	mmi_ram_info_exit();
	mmi_unit_info_exit();
	mmi_boot_info_exit();
	kfree(bootargs_str);
}

module_init(mmi_hw_info_init);
module_exit(mmi_hw_info_exit);
MODULE_DESCRIPTION("Motorola Mobility LLC. HW Info");
MODULE_LICENSE("GPL v2");
