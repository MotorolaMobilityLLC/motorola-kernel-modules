/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FCNT LIMITED 2021
/*----------------------------------------------------------------------------*/
/*
 * pt_i2c.c
 * Parade TrueTouch(TM) Standard Product I2C Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2020 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 */

#include "pt_regs.h"

#include <linux/i2c.h>
#include <linux/version.h>

#ifdef NDT_DATA_EN
extern int use_ndt_aw8680x;
#endif

#define CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT

#define PT_I2C_DATA_SIZE  (2 * 256)

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
static const struct of_device_id pt_i2c_of_match[] = {
	{ .compatible = "parade,pt_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, pt_i2c_of_match);
#endif

/*******************************************************************************
 * FUNCTION: pt_i2c_remove
 *
 * SUMMARY: Remove functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 ******************************************************************************/
static int pt_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	struct device *dev = &client->dev;
	struct pt_core_data *cd = i2c_get_clientdata(client);

	if (!cd) {
		return 0;
	}

	pt_release(cd);

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_i2c_of_match), dev);
	if (match)
		pt_devtree_clean_pdata(dev);
#endif
	i2c_set_clientdata(client, NULL);

	return 0;
}

static void pt_i2c_shutdown(struct i2c_client *client)
{
	pt_i2c_remove(client);

	return;
}

#ifdef CYPSOC_PICOLEAF_ENABLE
/*******************************************************************************
 * FUNCTION: cypsoc_picoleaf_probe
 *
 * SUMMARY: Probe functon for the Cypress PSoC
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
int cypsoc_picoleaf_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	struct cypsoc_picoleaf_data *cpd;
	int rc=0;

	(void)i2c_id;

	pr_info("cypsoc_picoleaf_probe() starts\n");

	/// get context buffers ///
	cpd = kzalloc(sizeof(*cpd), GFP_KERNEL);
	if (!cpd) {
		rc = -ENOMEM;
		pr_err("ERROR! cypsoc_picoleaf_probe(): Cypress PSoC data structure cannot be allocated in kernel\n");
		//cyp_debug(dev, DL_ERROR, "%s failed.\n", __func__);
		goto err_probe;
	}

	rc = cypsoc_picoleaf_force_power_on_hw(cpd);

	if(rc) {
		pr_err("%s: CYPSOC_PICOLEAF reset failed\n", __func__);
		return rc;
	}

	cpd->dev = &(client->dev);
	return cypsoc_picoleaf_probe_cont(cpd);
err_probe:
	return rc;
}

void cypsoc_picoleaf_shutdown(struct i2c_client *client)
{
	struct cypsoc_picoleaf_data *cpd = i2c_get_clientdata(client);
	cypsoc_picoleaf_shutdown_cont(cpd);
}

// GLOBAL VAR for sharing device data between pt and cypsoc
static struct i2c_client *i2c_clients_pt_cypsoc[2] = { NULL, NULL };

/*******************************************************************************
 * FUNCTION: pt_cypsoc_picoleaf_i2c_probe
 *
 * SUMMARY: Probe functon for the I2C module (Parade touch IC / Cypress PSoC)
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
static int pt_cypsoc_picoleaf_i2c_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	struct cypsoc_picoleaf_data *cpd;
	int rc = 0;

	pr_info("%s: probe enter, use_ndt_aw8680x = %d\n", __func__, use_ndt_aw8680x);

#ifdef NDT_DATA_EN
	if (use_ndt_aw8680x == 0) { //0 means second pressure sensor read id not compleated.
		usleep_range(50000, 60000);
		return -EPROBE_DEFER;
	}

	if (use_ndt_aw8680x == 1) {
		pr_info("%s: use_ndt_aw8680x is 1, picoleaf probe direct exit\n", __func__);
		return 0;
	}
#endif

	if (!strncmp(i2c_id->name, CYPSOC_PICOLEAF_NAME, strlen(CYPSOC_PICOLEAF_NAME))){
		rc = cypsoc_picoleaf_probe(client, i2c_id);
		if(rc) {
			pr_err("%s: raises ERROR at CYPSOC_PICOLEAF\n", __func__);
			return rc;
		}
			//cd  = dev_get_drvdata(&i2c_clients_pt_cypsoc[0]->dev);
			cpd = dev_get_drvdata(&client->dev);
			//if(cd == NULL || cpd == NULL){
			if(cpd == NULL){
				pr_err("%s: data structure is NULL!!\n", __func__);
				return -1;
			}
			//cd->cypsoc_picoleaf_data    = cpd;
			//cd->md.cypsoc_picoleaf_data = cpd;
			//cpd->pt_core_data           = cd;
			cpd->rst_gpio = 476;
			//cpd->vdd_gpio = cd->cpdata->pico_vdd_gpio;
			//cpd->vref_gpio = cd->cpdata->pico_vref_gpio;
			//if(cd->core_probe_complete == 1) {
			rc = cypsoc_picoleaf_i2c_readied(cpd);
			if (rc < 0) {
				return -EPROBE_DEFER;
			}

			cpd->probe_readid_not_esd_reset = 0;
			//create sys node
			cypsoc_picoleaf_sysclass_group_register(cpd);

			i2c_clients_pt_cypsoc[1] = client;
			cypsoc_picoleaf_firmware_update(cpd);
#ifdef NDT_DATA_EN
			use_ndt_aw8680x = 2;
#endif
			pr_info("%s: probe completed\n", __func__);
	}else{
		pr_err("%s: NAME ERROR!!\n", __func__);
	}
	return rc;
}

static void pt_cypsoc_picoleaf_i2c_shutdown(struct i2c_client *client)
{
	if (!strncmp(client->name, CYPSOC_PICOLEAF_NAME, strlen(PT_I2C_NAME))){
		cypsoc_picoleaf_shutdown(client);
	}else if(!strncmp(client->name, PT_I2C_NAME, strlen(PT_I2C_NAME))){
		pt_i2c_shutdown(client);
	}
}
#endif //CYPSOC_PICOLEAF_ENABLE

static const struct i2c_device_id pt_i2c_id[] = {
	{ PT_I2C_NAME, 0, },
#ifdef CYPSOC_PICOLEAF_ENABLE
	{ CYPSOC_PICOLEAF_NAME, 1 },
#endif
	{ }
};
MODULE_DEVICE_TABLE(i2c, pt_i2c_id);

static struct i2c_driver pt_i2c_driver = {
	.driver = {
		.name = PT_I2C_NAME,
		.owner = THIS_MODULE,
		//.pm = &pt_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
		.of_match_table = pt_i2c_of_match,
#endif
	},
#ifdef CYPSOC_PICOLEAF_ENABLE
	.probe = pt_cypsoc_picoleaf_i2c_probe,
	.remove = pt_i2c_remove,
	.shutdown = pt_cypsoc_picoleaf_i2c_shutdown,
#endif
	.id_table = pt_i2c_id,
};

#if (KERNEL_VERSION(3, 3, 0) <= LINUX_VERSION_CODE)
module_i2c_driver(pt_i2c_driver);
#else
/*******************************************************************************
 * FUNCTION: pt_i2c_init
 *
 * SUMMARY: Initialize function to register i2c module to kernel.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 ******************************************************************************/
static int __init pt_i2c_init(void)
{
	int rc = i2c_add_driver(&pt_i2c_driver);

	pr_info("%s: Parade TTDL I2C Driver (Build %s) rc=%d\n",
			__func__, PT_DRIVER_VERSION, rc);
	return rc;
}
module_init(pt_i2c_init);

/*******************************************************************************
 * FUNCTION: pt_i2c_exit
 *
 * SUMMARY: Exit function to unregister i2c module from kernel.
 *
 ******************************************************************************/
static void __exit pt_i2c_exit(void)
{
	i2c_del_driver(&pt_i2c_driver);
}
module_exit(pt_i2c_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
