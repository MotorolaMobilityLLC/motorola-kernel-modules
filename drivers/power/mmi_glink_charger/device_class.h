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

#ifndef __MMI_DEVICE_CLASS_H__
#define __MMI_DEVICE_CLASS_H__
#include <linux/notifier.h>
#include <linux/device.h>
#include <linux/time64.h>

typedef enum dev_type {
	DEV_NONE,
	DEV_BATT,
	DEV_SWITCH_BUCK,
	DEV_CHARGE_PUMP,
	DEV_BALANCE_CHG,
	DEV_WLS,
	DEV_USB,
	DEV_ALL,
	DEV_INVALID,
} DEV_TYPE;

typedef enum dev_role {
	DEV_MASTER,
	DEV_SLAVE,
	DEV_NUM,
} DEV_ROLE;

struct glink_device {
	struct device		dev;
	int dev_type;
	struct notifier_block nb;
	void	*dev_data;
	bool dev_present;
	struct mutex	dev_lock;
};

#define to_glink_device(obj) container_of(obj, struct glink_device, dev)
int is_dev_exist(const char *name);
struct glink_device *get_dev_by_name(const char *name);

int mmi_glink_register_notifier(struct notifier_block *nb);
void mmi_glink_unregister_notifier(struct notifier_block *nb);
int mmi_glink_notifier(DEV_TYPE type, void *data);
struct glink_device *glink_device_register(const char *name,
		struct device *parent, DEV_TYPE type, void *devdata);
void glink_device_unregister(struct glink_device *glink_dev);
void mmi_glink_class_exit(void);
int mmi_glink_class_init(void);
#endif
