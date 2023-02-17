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

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mmi_gauge_class.h>

static struct class *gauge_class;

int gauge_dev_set_charge_type(struct gauge_device *gauge_dev, int charge_type)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->set_charge_type)
		return gauge_dev->ops->set_charge_type(gauge_dev, charge_type);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_set_charge_type);

int gauge_dev_set_capacity(struct gauge_device *gauge_dev, int soc)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->set_capacity)
		return gauge_dev->ops->set_capacity(gauge_dev, soc);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_set_capacity);

int gauge_dev_set_temperature(struct gauge_device *gauge_dev, int temp)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->set_temperature)
		return gauge_dev->ops->set_temperature(gauge_dev, temp);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_set_temperature);

int gauge_dev_get_voltage_now(struct gauge_device *gauge_dev, int *mV)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_voltage_now)
		return gauge_dev->ops->get_voltage_now(gauge_dev, mV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_voltage_now);

int gauge_dev_get_current_now(struct gauge_device *gauge_dev, int *mA)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_current_now)
		return gauge_dev->ops->get_current_now(gauge_dev, mA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_current_now);

int gauge_dev_get_capacity(struct gauge_device *gauge_dev, int *soc)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_capacity)
		return gauge_dev->ops->get_capacity(gauge_dev, soc);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_capacity);

int gauge_dev_get_temperature(struct gauge_device *gauge_dev, int *temp)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_temperature)
		return gauge_dev->ops->get_temperature(gauge_dev, temp);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_temperature);

int gauge_dev_get_tte(struct gauge_device *gauge_dev, int *tte)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_tte)
		return gauge_dev->ops->get_tte(gauge_dev, tte);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_tte);

int gauge_dev_get_charge_full(struct gauge_device *gauge_dev, int *charge_full)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_charge_full)
		return gauge_dev->ops->get_charge_full(gauge_dev, charge_full);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_charge_full);

int gauge_dev_get_charge_full_design(struct gauge_device *gauge_dev, int *charge_full_design)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_charge_full_design)
		return gauge_dev->ops->get_charge_full_design(gauge_dev, charge_full_design);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_charge_full_design);

int gauge_dev_get_charge_counter(struct gauge_device *gauge_dev, int *charge_counter)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_charge_counter)
		return gauge_dev->ops->get_charge_counter(gauge_dev, charge_counter);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_charge_counter);

int gauge_dev_get_cycle_count(struct gauge_device *gauge_dev, int *cycle_count)
{
	if (gauge_dev != NULL && gauge_dev->ops != NULL &&
	    gauge_dev->ops->get_cycle_count)
		return gauge_dev->ops->get_cycle_count(gauge_dev, cycle_count);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(gauge_dev_get_cycle_count);

static void gauge_device_release(struct device *dev)
{
	struct gauge_device *gauge_dev = to_gauge_device(dev);

	kfree(gauge_dev);
}

/**
 * gauge_device_register - create and register a new object of
 *   gauge_device class.
 * @name: the name of the new object
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use.
 * The methods may retrieve it by using gauge_get_data(gauge_dev).
 * @ops: the gauge operations structure.
 *
 * Creates and registers new gauge device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct gauge_device *gauge_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct gauge_ops *ops,
		const struct gauge_properties *props)
{
	struct gauge_device *gauge_dev;
	static struct lock_class_key key;
	struct srcu_notifier_head *head;
	int rc;

	pr_debug("%s: name=%s\n", __func__, name);
	gauge_dev = kzalloc(sizeof(*gauge_dev), GFP_KERNEL);
	if (!gauge_dev)
		return ERR_PTR(-ENOMEM);

	head = &gauge_dev->evt_nh;
	srcu_init_notifier_head(head);
	/* Rename srcu's lock to avoid LockProve warning */
	lockdep_init_map(&(&head->srcu)->dep_map, name, &key, 0);
	mutex_init(&gauge_dev->ops_lock);
	gauge_dev->dev.class = gauge_class;
	gauge_dev->dev.parent = parent;
	gauge_dev->dev.release = gauge_device_release;
	dev_set_name(&gauge_dev->dev, "%s", name);
	dev_set_drvdata(&gauge_dev->dev, devdata);

	/* Copy properties */
	if (props) {
		memcpy(&gauge_dev->props, props,
		       sizeof(struct gauge_properties));
	}
	rc = device_register(&gauge_dev->dev);
	if (rc) {
		kfree(gauge_dev);
		return ERR_PTR(rc);
	}
	gauge_dev->ops = ops;
	return gauge_dev;
}
EXPORT_SYMBOL(gauge_device_register);

/**
 * gauge_device_unregister - unregisters a discrete gauge device
 * object.
 * @gauge_dev: the gauge device object to be unregistered
 * and freed.
 *
 * Unregisters a previously registered via gauge_device_register object.
 */
void gauge_device_unregister(struct gauge_device *gauge_dev)
{
	if (!gauge_dev)
		return;

	mutex_lock(&gauge_dev->ops_lock);
	gauge_dev->ops = NULL;
	mutex_unlock(&gauge_dev->ops_lock);
	device_unregister(&gauge_dev->dev);
}
EXPORT_SYMBOL(gauge_device_unregister);

static int gauge_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct gauge_device *get_gauge_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct gauge_device *)NULL;
	dev = class_find_device(gauge_class, NULL, name,
				gauge_match_device_by_name);

	return dev ? to_gauge_device(dev) : NULL;

}
EXPORT_SYMBOL(get_gauge_by_name);

static ssize_t name_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct gauge_device *gauge_dev = to_gauge_device(dev);

	return snprintf(buf, 20, "%s\n",
		       gauge_dev->props.alias_name ?
		       gauge_dev->props.alias_name : "anonymous");
}

static DEVICE_ATTR_RO(name);

static struct attribute *gauge_class_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group gauge_group = {
	.attrs = gauge_class_attrs,
};

static const struct attribute_group *gauge_groups[] = {
	&gauge_group,
	NULL,
};

static void __exit gauge_class_exit(void)
{
	class_destroy(gauge_class);
}

static int __init gauge_class_init(void)
{
	gauge_class = class_create(THIS_MODULE, "mmi_gauge");
	if (IS_ERR(gauge_class)) {
		pr_err("Unable to create mmi discrete charger class; errno = %ld\n",
			PTR_ERR(gauge_class));
		return PTR_ERR(gauge_class);
	}
	gauge_class->dev_groups = gauge_groups;
	pr_info("success to create mmi gauge class \n");
	return 0;
}

module_init(gauge_class_init);
module_exit(gauge_class_exit);

MODULE_DESCRIPTION("Mmi Gauge Class Device");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
