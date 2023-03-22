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

#ifndef MMI_GAUGE_CLASS_H
#define MMI_GAUGE_CLASS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>

struct gauge_properties {
	const char *alias_name;
};

/* Data of notifier from gauge device */
struct fgdev_notify {
	bool soc_change;
};

struct gauge_device {
	struct gauge_properties props;
	struct fgdev_notify noti;
	const struct gauge_ops *ops;
	struct mutex ops_lock;
	struct device dev;
	struct srcu_notifier_head evt_nh;
	void	*driver_data;
	bool is_polling_mode;
};

struct gauge_ops {
	int (*get_voltage_now)(struct gauge_device *gauge_dev, int *mV);
	int (*get_current_now)(struct gauge_device *gauge_dev, int *mA);
	int (*get_capacity)(struct gauge_device *gauge_dev, int *soc);
	int (*get_temperature)(struct gauge_device *gauge_dev, int *temp);
	int (*get_tte)(struct gauge_device *gauge_dev, int *tte);
	int (*get_charge_full)(struct gauge_device *gauge_dev, int *charge_full);
	int (*get_charge_full_design)(struct gauge_device *gauge_dev, int *charge_full_design);
	int (*get_charge_counter)(struct gauge_device *gauge_dev, int *charge_counter);
	int (*get_soh)(struct gauge_device *gauge_dev, int *soh);
	int (*get_cycle_count)(struct gauge_device *gauge_dev, int *cycle_count);

	int (*set_temperature)(struct gauge_device *gauge_dev, int temp);
	int (*set_capacity)(struct gauge_device *gauge_dev, int soc);
	int (*set_charge_type)(struct gauge_device *gauge_dev, int charge_type);
};

#define to_gauge_device(obj) container_of(obj, struct gauge_device, dev)

static inline void *gauge_dev_get_drvdata(
	const struct gauge_device *gauge_dev)
{
	return gauge_dev->driver_data;
}

static inline void gauge_dev_set_drvdata(
	struct gauge_device *gauge_dev, void *data)
{
	gauge_dev->driver_data = data;
}

extern int gauge_dev_set_charge_type(struct gauge_device *gauge_dev, int charge_type);

extern int gauge_dev_set_capacity(struct gauge_device *gauge_dev, int soc);

extern int gauge_dev_set_temperature(struct gauge_device *gauge_dev, int temp);

extern int gauge_dev_get_voltage_now(struct gauge_device *gauge_dev, int *mV);

extern int gauge_dev_get_current_now(struct gauge_device *gauge_dev, int *mA);

extern int gauge_dev_get_capacity(struct gauge_device *gauge_dev, int *soc);

extern int gauge_dev_get_temperature(struct gauge_device *gauge_dev, int *temp);

extern int gauge_dev_get_tte(struct gauge_device *gauge_dev, int *tte);

extern int gauge_dev_get_charge_full(struct gauge_device *gauge_dev, int *charge_full);

extern int gauge_dev_get_charge_full_design(struct gauge_device *gauge_dev, int *charge_full_design);

extern int gauge_dev_get_soh(struct gauge_device *gauge_dev, int *soh);

extern int gauge_dev_get_charge_counter(struct gauge_device *gauge_dev, int *charge_counter);

extern int gauge_dev_get_cycle_count(struct gauge_device *gauge_dev, int *cycle_count);

extern struct gauge_device *gauge_device_register(
	const char *name,
	struct device *parent, void *devdata, const struct gauge_ops *ops,
	const struct gauge_properties *props);

extern void gauge_device_unregister(
	struct gauge_device *gauge_dev);

extern struct gauge_device *get_gauge_by_name(
	const char *name);

#endif