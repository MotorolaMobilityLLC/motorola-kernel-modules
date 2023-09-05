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
}

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
