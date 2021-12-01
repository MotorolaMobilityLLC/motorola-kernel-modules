// SPDX-License-Identifier: GPL-2.0
// TI LM3697 LED chip family driver
// Copyright (C) 2018 Texas Instruments Incorporated - https://www.ti.com/

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/backlight.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <uapi/linux/uleds.h>

#define LM3697_REV			0x0
#define LM3697_RESET			0x1
#define LM3697_OUTPUT_CONFIG		0x10
#define LM3697_CTRL_A_B_BRT_CFG		0x16
#define LM3697_FEEDBACK_ENABLE		0x19
#define LM3697_BOOST_CTRL		0x1a
#define LM3697_PWM_CONFIG		0x1c
#define LM3697_CONTROL_A_RAMP		0x11
#define LM3697_CONTROL_B_RAMP		0x12
#define LM3697_CONTROL_A_RUN_RAMP	0x13
#define LM3697_CONTROL_A_FS_SETTING	0x17


#define LM3697_CTRL_A_BRT_LSB		0x20
#define LM3697_CTRL_A_BRT_MSB		0x21
#define LM3697_CTRL_ENABLE		0x24

#define LM3697_SW_RESET		BIT(0)

#define LM3697_CTRL_A_EN	BIT(0)
#define LM3697_CTRL_B_EN	BIT(1)
#define LM3697_CTRL_A_B_EN	(LM3697_CTRL_A_EN | LM3697_CTRL_B_EN)

#define LM3697_MAX_LED_STRINGS	3

#define LM3697_CONTROL_A	0
#define LM3697_MAX_CONTROL_BANKS 2
#define LMU_11BIT_LSB_MASK	(BIT(0) | BIT(1) | BIT(2))
#define LMU_11BIT_MSB_SHIFT	3

#define MAX_BRIGHTNESS_8BIT	255
#define MAX_BRIGHTNESS_11BIT	2047

#define LM3697_LED_DEV "lm3697-bl"
#define LM3697_NAME "lm3697-bl"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#define KERNEL_ABOVE_4_14
#endif

struct lm3697 {
	int enable_gpio;
	struct i2c_client *client;
	struct led_classdev led_dev;
	struct device dev;
	struct mutex lock;
	struct i2c_adapter *adapter;
	struct work_struct work;
	struct backlight_device *bl_dev;

	unsigned short addr;

	unsigned int brightness;
	unsigned int max_brightness;
	unsigned int default_brightness;
	unsigned int output_config;
	unsigned int ctrl_a_ramp;
	unsigned int ctrl_b_ramp;
	unsigned int runtime_ramp;
	unsigned int bl_fscal;
	unsigned int ctrl_brt_cfg;
	unsigned int feedback_enable;
	unsigned int boost_control;
	unsigned int pwm_config;
	unsigned int ctrl_bank_en;
	unsigned int bl_map;
	int  enabled;
	bool using_lsb;
};

struct lm3697 *ext_lm3697_data;

static int platform_read_i2c_block(struct i2c_client *client, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;
	unsigned char cnt = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};

		while (cnt < 5) {
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret < 0)
				dev_err(&client->dev, "%s: i2c read error\n",
								__func__);
			else
				break;

			cnt++;
			mdelay(2);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		while (cnt < 5) {
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				dev_err(&client->dev, "%s:i2c read error.\n",
								__func__);
			else
				break;

			cnt++;
			mdelay(2);
		}
	}
	return ret;
}

static int lm3697_i2c_read(struct i2c_client *client, u8 addr, u8 *val)
{
	return platform_read_i2c_block(client, &addr, 1, val, 1);
}

static int platform_write_i2c_block(struct i2c_client *client,
		char *writebuf, int writelen)
{
	int ret;
	unsigned char cnt = 0;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	while (cnt < 5) {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c write error.\n",
								__func__);
		else
			break;

		cnt++;
		mdelay(2);
	}

	return ret;
}

static int lm3697_i2c_write(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return platform_write_i2c_block(client, buf, sizeof(buf));
}

/*
static int lm3697_i2c_write_bit(struct i2c_client *client,
	unsigned int reg_addr, unsigned int  mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;

	lm3697_i2c_read(client, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	lm3697_i2c_write(client, reg_addr, reg_val);

	return 0;
}
*/

static unsigned char chipid;
static int lm3697_read_chipid(struct lm3697 *priv)
{
	int ret = -1;
	u8 value = 0;
	unsigned char cnt = 0;

	while (cnt < 5) {
		ret = lm3697_i2c_read(priv->client, LM3697_REV, &value);
		if (ret < 0) {
			pr_err("%s: failed to read reg LM3697_REG_ID: %d\n",
				__func__, ret);
		}
		switch (value) {
		case 0x01:
			pr_info("%s lm3697 detected\n", __func__);
			chipid = value;
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, value);
			break;
		}
		cnt++;

		msleep(2);
	}

	return -EINVAL;
}

int is_lm3697_chip_exist(void)
{
	if (chipid == 0x1)
		return 1;
	else
		return 0;
}

static int lm3697_brightness_map(unsigned int level)
{
	if (!ext_lm3697_data)
		return 0;

	/*MAX_LEVEL_256*/
	if (ext_lm3697_data->bl_map == 1) {
		if (level == 255)
			return 2047;
		return level * 8;
	}
	/*MAX_LEVEL_1024*/
	if (ext_lm3697_data->bl_map == 2)
		return level * 2;
	/*MAX_LEVEL_2048*/
	if (ext_lm3697_data->bl_map == 3)
		return level;

	return  level;
}

static int lm3697_gpio_init(struct lm3697 *priv)
{
	int ret = -1;

	if (gpio_is_valid(priv->enable_gpio)) {
		ret = gpio_request(priv->enable_gpio, "lm3697_gpio");
		if (ret < 0) {
			pr_err("failed to request gpio\n");
			return -1;
		}
		ret = gpio_direction_output(priv->enable_gpio, 0);
		pr_info(" request gpio init\n");
		if (ret < 0) {
			pr_err("failed to set output");
			gpio_free(priv->enable_gpio);
			return ret;
		}
		gpio_set_value(priv->enable_gpio, true);
	} else
		return -EINVAL;

	return ret;
}

static int lm3697_init(struct lm3697 *priv)
{

	pr_info("LM3697 %s\n", __func__);

	lm3697_i2c_write(priv->client, LM3697_OUTPUT_CONFIG, priv->output_config);
	lm3697_i2c_write(priv->client, LM3697_CONTROL_A_RAMP, priv->ctrl_a_ramp);
	lm3697_i2c_write(priv->client, LM3697_CONTROL_B_RAMP, priv->ctrl_b_ramp);
	lm3697_i2c_write(priv->client, LM3697_CONTROL_A_RUN_RAMP, priv->runtime_ramp);
	lm3697_i2c_write(priv->client, LM3697_CONTROL_A_FS_SETTING, priv->bl_fscal);
	/* Change BLK to Linear mode. 0x00 = exponential, 0x01 = Linear */
	lm3697_i2c_write(priv->client, LM3697_CTRL_A_B_BRT_CFG, priv->ctrl_brt_cfg);
	lm3697_i2c_write(priv->client, LM3697_FEEDBACK_ENABLE, priv->feedback_enable);
	lm3697_i2c_write(priv->client, LM3697_BOOST_CTRL, priv->boost_control);
	lm3697_i2c_write(priv->client, LM3697_PWM_CONFIG, priv->pwm_config);
	lm3697_i2c_write(priv->client, LM3697_CTRL_ENABLE, priv->ctrl_bank_en);

	return 0;
}

int  lm3697_set_brightness(struct lm3697 *drvdata, int brt_val)
{
	int ret = -1;
	pr_info("%s brt_val is %d\n", __func__, brt_val);

	if (!ext_lm3697_data)
		return 0;

	if(ext_lm3697_data->enabled == 0) {
		if (brt_val == 0) {
			//avoid duplicate standy process
			return 0;
		}
		lm3697_init(ext_lm3697_data);
		ext_lm3697_data->enabled = 1;
	}

	brt_val = lm3697_brightness_map(brt_val);

	if (brt_val == 0) {
		ext_lm3697_data->enabled = 0;
		lm3697_i2c_write(ext_lm3697_data->client, LM3697_CONTROL_A_RAMP, 0x00);
		lm3697_i2c_write(ext_lm3697_data->client, LM3697_CONTROL_B_RAMP, 0x00);
		lm3697_i2c_write(ext_lm3697_data->client, LM3697_CONTROL_A_RUN_RAMP, 0x00);
	}

	pr_info("%s LM3697_level is %d\n", __func__, brt_val);

	lm3697_i2c_write(ext_lm3697_data->client,
				LM3697_CTRL_A_BRT_LSB,
				brt_val&0x0007);

	lm3697_i2c_write(ext_lm3697_data->client,
				LM3697_CTRL_A_BRT_MSB,
				(brt_val >> 3)&0xff);

	return ret;

}

#ifdef KERNEL_ABOVE_4_14
static int lm3697_bl_get_brightness(struct backlight_device *bl_dev)
{
		return bl_dev->props.brightness;
}

static int lm3697_bl_update_status(struct backlight_device *bl_dev)
{
		struct lm3697 *drvdata = bl_get_data(bl_dev);
		int brt;

		if (bl_dev->props.state & BL_CORE_SUSPENDED)
				bl_dev->props.brightness = 0;

		brt = bl_dev->props.brightness;
		/*
		 * Brightness register should always be written
		 * not only register based mode but also in PWM mode.
		 */
		return lm3697_set_brightness(drvdata, brt);
}

static const struct backlight_ops lm3697_bl_ops = {
		.update_status = lm3697_bl_update_status,
		.get_brightness = lm3697_bl_get_brightness,
};
#endif

static void __lm3697_work(struct lm3697 *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	lm3697_set_brightness(led, value);
	mutex_unlock(&led->lock);
}

static void lm3697_work(struct work_struct *work)
{
	struct lm3697 *drvdata = container_of(work,
					struct lm3697, work);

	__lm3697_work(drvdata, drvdata->led_dev.brightness);
}

static void lm3697_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brt_val)
{
	struct lm3697 *drvdata;

	drvdata = container_of(led_cdev, struct lm3697, led_dev);
	schedule_work(&drvdata->work);
}

static void lm3697_probe_dt(struct device *dev, struct lm3697 *priv)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp;

	priv->enable_gpio = of_get_named_gpio(np, "lm3697,hwen-gpio", 0);
	pr_info("%s priv->enable_gpio --<%d>\n", __func__, priv->enable_gpio);

	rc = of_property_read_u32(np, "lm3697,output-config", &temp);
	if (rc) {
		pr_err("lm3697,output-config read fail!\n");
	} else {
		priv->output_config = temp;
		pr_info("%s lm3697,output-config --<%x >\n",
			__func__, priv->output_config);
	}

	rc = of_property_read_u32(np, "lm3697,control-a-ramp", &temp);
	if (rc) {
		pr_err("lm3697,control-a-ramp read fail!\n");
	} else {
		priv->ctrl_a_ramp = temp;
		pr_info("%s lm3697,control-a-ramp --<%x >\n",
			__func__, priv->ctrl_a_ramp);
	}

	rc = of_property_read_u32(np, "lm3697,control-b-ramp", &temp);
	if (rc) {
		pr_err("lm3697,control-b-ramp read fail!\n");
	} else {
		priv->ctrl_b_ramp = temp;
		pr_info("%s lm3697,control-b-ramp --<%x >\n",
			__func__, priv->ctrl_b_ramp);
	}

	rc = of_property_read_u32(np, "lm3697,run-time-ramp", &temp);
	if (rc) {
		pr_err("lm3697,control-b-ramp read fail!\n");
	} else {
		priv->runtime_ramp = temp;
		pr_info("%s lm3697,run-time-ramp --<%x >\n",
			__func__, priv->runtime_ramp);
	}

	rc = of_property_read_u32(np, "lm3697,bl-fscal-led", &temp);
	if (rc) {
		pr_err("lm3697,bl-fscal-led read fail!\n");
	} else {
		priv->bl_fscal = temp;
		pr_info("%s lm3697,bl-fscal-led --<%x >\n",
			__func__, priv->bl_fscal);
	}

	rc = of_property_read_u32(np, "lm3697,default-brightness", &temp);
	if (rc != 0) {
		priv->default_brightness = priv->max_brightness;
		pr_err("%s default-brightness not found, set to max %d\n", __func__, priv->default_brightness);
	}
	else {
		priv->default_brightness = temp;
		pr_info("%s default_brightness=%d\n", __func__, priv->default_brightness);
	}

	rc = of_property_read_u32(np, "lm3697,ctrl_a_b_brt_cfg", &temp);
	if (rc) {
		pr_err("lm3697,ctrl_a_b_brt_cfg read fail!\n");
	} else {
		priv->ctrl_brt_cfg = temp;
		pr_info("%s lm3697,ctrl_a_b_brt_cfg --<%x >\n",
			__func__, priv->ctrl_brt_cfg);
	}

	rc = of_property_read_u32(np, "lm3697,feedback-enable", &temp);
	if (rc) {
		pr_err("lm3697,feedback-enable read fail!\n");
	} else {
		priv->feedback_enable = temp;
		pr_info("%s lm3697,feedback-enable --<%x >\n",
			__func__, priv->feedback_enable);
	}

	rc = of_property_read_u32(np, "lm3697,boost-control", &temp);
	if (rc) {
		pr_err("lm3697,boost-control read fail!\n");
	} else {
		priv->boost_control = temp;
		pr_info("%s lm3697,boost-control --<%x >\n",
			__func__, priv->boost_control);
	}

	rc = of_property_read_u32(np, "lm3697,pwm-config", &temp);
	if (rc) {
		pr_err("lm3697,pwm_config read fail!\n");
	} else {
		priv->pwm_config = temp;
		pr_info("%s lm3697,pwm_config --<%x >\n",
			__func__, priv->pwm_config);
	}

	rc = of_property_read_u32(np, "lm3697,ctrl-bank-en", &temp);
	if (rc) {
		pr_err("lm3697,ctrl-bank-en read fail!\n");
	} else {
		priv->ctrl_bank_en = temp;
		pr_info("%s lm3697,ctrl-bank-en --<%x >\n",
			__func__, priv->ctrl_bank_en);
	}

	rc = of_property_read_u32(np, "lm3697,bl_map", &temp);
	if (rc) {
		pr_err("lm3697,bl_map read fail!\n");
	} else {
		priv->bl_map = temp;
		pr_info("%s lm3697,bl_map --<%x >\n",
			__func__, priv->bl_map);
	}

	priv->using_lsb = of_property_read_bool(np, "lm3697,using-lsb");
	pr_info("%s using_lsb --<%d>\n", __func__, priv->using_lsb);

	if (priv->using_lsb) {
		priv->brightness = 0x7ff;
		priv->max_brightness = 2047;
	} else {
		priv->brightness = 0xff;
		priv->max_brightness = 255;
	}

}
/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t lm3697_i2c_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct lm3697 *led = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		lm3697_i2c_write(led->client,
				(unsigned char)databuf[0],
				(unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t lm3697_i2c_reg_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(reg, 0664, lm3697_i2c_reg_show, lm3697_i2c_reg_store);
static struct attribute *lm3697_attributes[] = {
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group lm3697_attribute_group = {
	.attrs = lm3697_attributes
};

static int lm3697_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lm3697 *led;
#ifdef KERNEL_ABOVE_4_14
	struct backlight_device *bl_dev;
	struct backlight_properties props;
#endif
	int ret;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		ret = -EIO;
		goto err_out;
	}
	pr_err("lm3697_probe!\n");
	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	led = kzalloc(sizeof(struct lm3697), GFP_KERNEL);
	if (led == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	led->client = client;
	led->adapter = client->adapter;
	led->addr = client->addr;
	led->brightness = LED_OFF;
	led->led_dev.default_trigger = "bkl-trigger";
	led->led_dev.name = LM3697_LED_DEV;
	led->led_dev.brightness_set = lm3697_brightness_set;
	led->led_dev.max_brightness = MAX_BRIGHTNESS_11BIT;
	mutex_init(&led->lock);
	INIT_WORK(&led->work, lm3697_work);
	lm3697_probe_dt(&client->dev, led);
	i2c_set_clientdata(client, led);
	lm3697_gpio_init(led);

	ret = lm3697_read_chipid(led);
	if(ret < 0)
	{
		pr_err("Failed to read lm3697 chipid: %d\n", ret);
		goto err_init;
	}
	ret = led_classdev_register(&client->dev, &led->led_dev);
	if (ret < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		goto err_init;
	}
	led->enabled = 1;


#ifdef KERNEL_ABOVE_4_14
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = MAX_BRIGHTNESS_11BIT;
	props.max_brightness = MAX_BRIGHTNESS_11BIT;
	bl_dev = backlight_device_register(LM3697_NAME, &client->dev,
					led, &lm3697_bl_ops, &props);
#endif
	ext_lm3697_data = led;
	lm3697_init(led);
	lm3697_set_brightness(led, led->default_brightness);

	ret = sysfs_create_group(&client->dev.kobj, &lm3697_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			__func__);
		goto err_init;
	}
	pr_info("%s exit\n", __func__);

	return 0;

err_init:
	kfree(led);
	return ret;
err_out:
	return ret;
}

static int lm3697_remove(struct i2c_client *client)
{
	struct lm3697 *led = i2c_get_clientdata(client);

	int ret;

	ret = lm3697_i2c_write(client, LM3697_CTRL_ENABLE, 0);
	if (ret) {
		pr_err( "Failed to disable the device\n");
		return ret;
	}
	led_classdev_unregister(&led->led_dev);
	mutex_destroy(&led->lock);
	kfree(led);
	ext_lm3697_data = NULL;
	return 0;
}

static const struct i2c_device_id lm3697_id[] = {
	{ "lm3697", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3697_id);

static const struct of_device_id of_lm3697_leds_match[] = {
	{ .compatible = "ti,lm3697-bl", },
	{},
};

static struct i2c_driver lm3697_driver = {
    .probe		= lm3697_probe,
	.remove		= lm3697_remove,
	.id_table	= lm3697_id,
	.driver = {
		.name = "lm3697",
        .owner = THIS_MODULE,
		.of_match_table = of_lm3697_leds_match,
	},
};

module_i2c_driver(lm3697_driver);
MODULE_DESCRIPTION("LM3697 LED driver");
MODULE_LICENSE("GPL v2");
