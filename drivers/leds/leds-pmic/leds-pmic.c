// SPDX-License-Identifier: GPL-2.0-only
/*
 * LEDs driver for PMICs
 *
 */
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#ifdef CONFIG_MTK_MULTI_BQ_SGM_LED_SUPPORT
#include <linux/power_supply.h>
#include "bq2589x_reg.h"
#include "sgm415xx.h"
bool bq25890 = false;
bool sgm41543d = false;
void pimc_led_get_id(void);
extern struct power_supply *power_supply_get_by_name(const char *name);
#elif defined(CONFIG_MTK_BQ25890_LED_SUPPORT)
#include "bq2589x_reg.h"
bool bq25890 = true;
#elif defined(CONFIG_MTK_SGM41543D_LED_SUPPORT)
#include "sgm415xx.h"
bool sgm41543d = true;
#endif

#ifdef CONFIG_MTK_BQ2560x_SUPPORT
#include <bq2560x.h>
#endif

struct pmic_led_data {
	struct led_classdev cdev;
};

static inline struct pmic_led_data *
			cdev_to_pmic_led_data(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct pmic_led_data, cdev);
}

static void pmic_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int level;

	pr_info("pmic_led_set: value = %d\n", value);
	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

//+EKELLIS-48, yaocankun.wt, 20210401, add led control node
#ifdef CONFIG_MTK_BQ2560x_SUPPORT
	if (level == 0) {
		bq2560x_enable_statpin(0);
	}
	else
	{
		bq2560x_enable_statpin(1);
	}
#endif
//-EKELLIS-48, yaocankun.wt, 20210401, add led control node
// IKSWU-118943 jiacq4 20240403, add led control mode
#ifdef CONFIG_MTK_BQ25890_LED_SUPPORT
	if(bq25890){
		if (level == 0)
			bq2589x_enable_statpin(0);
		else
			bq2589x_enable_statpin(1);
	}
#endif
#ifdef CONFIG_MTK_SGM41543D_LED_SUPPORT
	if(sgm41543d){
		if (level == 0)
			sgm41543_enable_statpin(0);
		else
			sgm41543_enable_statpin(1);
	}
#endif
// IKSWU-118943 END

}

#ifdef CONFIG_MTK_MULTI_BQ_SGM_LED_SUPPORT
// IKSWU-121600 jiacq4 20240410, add get ID
void pimc_led_get_id(void)
{
	struct power_supply *chosen;
	chosen = power_supply_get_by_name("primary_chg");

	if(strstr(chosen->of_node->name,"bq25890")){
		pr_info("chosen bq25890\n");
		bq25890 = true;
	}else if(strstr(chosen->of_node->name,"sgm41543d")){
		pr_info("chosen sgm41543d\n");
		sgm41543d = true;
	}else{
		bq25890 = false;
		sgm41543d = false;
		pr_info("chosen node null\n");
	}
}
// IKSWU-121600 END
#endif

static const struct of_device_id of_pmic_leds_match[] = {
	{ .compatible = "pmic-leds", },
	{},
};

MODULE_DEVICE_TABLE(of, of_pmic_leds_match);

static int pmic_led_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pmic_led_data *led;
	const char *state;
	int ret;

  	led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
  	if (!led)
  		return -ENOMEM;

#ifdef CONFIG_MTK_MULTI_BQ_SGM_LED_SUPPORT
	pimc_led_get_id();
#endif

  	/* Use label else node name */
  	led->cdev.name = of_get_property(np, "label", NULL) ? : np->name;
  	led->cdev.default_trigger =
  		of_get_property(np, "linux,default-trigger", NULL);
	led->cdev.brightness_set = pmic_led_set;
	led->cdev.brightness = LED_OFF;

  	state = of_get_property(np, "default-state", NULL);
  	if (state) {
  		if (!strcmp(state, "on")) {
  			led->cdev.brightness = LED_FULL;
  			pmic_led_set(&led->cdev, LED_FULL);
	  	}
	}

	ret = devm_led_classdev_register(&pdev->dev, &led->cdev);
  	if (ret) {
  		dev_err(&pdev->dev, "unable to register led \"%s\"\n",
  			led->cdev.name);
		devm_kfree(&pdev->dev, led);
  		return ret;
  	}	

	platform_set_drvdata(pdev, led);

	return 0;
}

static void pmic_led_shutdown(struct platform_device *pdev)
{
	struct pmic_led_data *led = platform_get_drvdata(pdev);

	pmic_led_set(&led->cdev, LED_OFF);
}

static struct platform_driver pmic_led_driver = {
	.probe		= pmic_led_probe,
	.shutdown	= pmic_led_shutdown,
	.driver		= {
		.name	= "leds-pmic",
		.of_match_table = of_pmic_leds_match,
	},
};

module_platform_driver(pmic_led_driver);

MODULE_AUTHOR("Motorola Mobility");
MODULE_DESCRIPTION("PMIC LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pmic");
