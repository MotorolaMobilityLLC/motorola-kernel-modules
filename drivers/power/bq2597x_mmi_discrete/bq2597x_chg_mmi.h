#ifndef __BQ2597X_CHG_MMI_H__
#define __BQ2597X_CHG_MMI_H__
#include <linux/power_supply.h>
#include <linux/sysfs.h>

#define BQ2597X_SYSFS_FIELD_RW(_name, _prop)	\
{									 \
	.attr	= __ATTR(_name, 0644, bq2597x_sysfs_show, bq2597x_sysfs_store),\
	.prop	= _prop,	\
	.set	= _name##_set,						\
	.get	= _name##_get,						\
}

#define BQ2597X_SYSFS_FIELD_RO(_name, _prop)	\
{			\
	.attr   = __ATTR(_name, 0444, bq2597x_sysfs_show, bq2597x_sysfs_store),\
	.prop   = _prop,				  \
	.get	= _name##_get,						\
}

#define BQ2597X_SYSFS_FIELD_WO(_name, _prop)	\
{								   \
	.attr	= __ATTR(_name, 0200, bq2597x_sysfs_show, bq2597x_sysfs_store),\
	.prop	= _prop,	\
	.set	= _name##_set,						\
}

enum bq2597x_property {
	BQ2597X_PROP_CHARGING_ENABLED,
	BQ2597X_PROP_UPDATE_NOW,
	BQ2597X_PROP_INPUT_CURRENT_NOW,
	BQ2597X_PROP_INPUT_VOLTAGE_SETTLED,
};
#endif
