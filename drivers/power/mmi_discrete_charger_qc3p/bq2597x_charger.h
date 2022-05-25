#ifndef __BQ2597X_CHARGER_H__
#define __BQ2597X_CHARGER_H__
enum bq2597x_property {
	BQ2597X_PROP_CHARGING_ENABLED,
	BQ2597X_PROP_UPDATE_NOW,
	BQ2597X_PROP_INPUT_CURRENT_NOW,
	BQ2597X_PROP_INPUT_VOLTAGE_SETTLED,
};
extern int bq2597x_set_property(enum bq2597x_property bp,
			    int val);
extern int bq2597x_get_int_property(enum bq2597x_property bp);
extern int bq2597x_get_property(enum bq2597x_property bp,
			    int *val);
#endif
