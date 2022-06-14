/*
 * SM5350 BL Driver
 *
 * SiliconMitus SM5350 Backlight driver chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/backlight.h>

#define SM5350_NAME "sm5350-bl"

#define MAX_BRIGHTNESS			(2047)
#define BANK_NONE				0x00
#define BANK_A					0x01
#define BANK_B					0x02

#define SM5350_HVLED_CURR_SINK_OUT_CFG			0x07
#define SM5350_BRIGHTNESS_CFG					0x01

#define SM5350_REVISION_REG						0x00
#define SM5350_SW_RESET_REG						0x01
#define SM5350_HVLED_CURR_SINK_OUT_CFG_REG		0x10
#define SM5350_CTL_A_RAMP_TIME_REG				0x11
#define SM5350_CTL_B_RAMP_TIME_REG				0x12
#define SM5350_CTL_RUNTIME_RAMP_TIME_REG		0x13
#define SM5350_CTL_RUNTIME_RAMP_CFG_REG			0x14
#define SM5350_BRIGHTNESS_CFG_REG				0x16
#define SM5350_CTL_A_FULL_SCALE_CURR_REG		0x17
#define SM5350_CTL_B_FULL_SCALE_CURR_REG		0x18
#define SM5350_HVLED_CURR_SINK_FEEDBACK_REG		0x19
#define SM5350_BOOST_CTL_REG					0x1A
#define SM5350_AUTO_FREQ_THRESHOLD_REG			0x1B
#define SM5350_PWM_CFG_REG						0x1C
#define SM5350_CTL_A_BRIGHTNESS_LSB_REG			0x20
#define SM5350_CTL_A_BRIGHTNESS_MSB_REG			0x21
#define SM5350_CTL_B_BRIGHTNESS_LSB_REG			0x22
#define SM5350_CTL_B_BRIGHTNESS_MSB_REG			0x23
#define SM5350_CTL_B_BANK_EN_REG				0x24
#define SM5350_HVLED_OPEN_FAULTS_REG			0xB0
#define SM5350_HVLED_SHORT_FAULTS_REG			0xB2
#define SM5350_LED_FAULT_ENABLES_REG			0xB4

enum backlight_exp_current_align {
	ALIGN_NONE,
	ALIGN_AW99703
};

struct sm5350_data {
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct work_struct	work;
	bool enable;
	bool bank_A;
	bool bank_B;
	u8 ctl_bank_en;
	u8 pwm_cfg;
	u8 boost_ctl;
	u8 full_scale_current;
	u8 map_mode;
	unsigned int led_current_align; /* Align boost current to AW chip */
	unsigned int default_brightness;
	bool brt_code_enable;
	u16 *brt_code_table;
	int en_gpio;
	struct backlight_device *bl_dev;
};

struct sm5350_data *g_sm5350_data;

static int platform_read_i2c_block(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

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
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int sm5350_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return platform_read_i2c_block(client, &addr, 1, val, 1);
}

static int platform_write_i2c_block(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int sm5350_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;
	return platform_write_i2c_block(client, buf, sizeof(buf));
}

static int sm5350_init_registers(struct sm5350_data *drvdata)
{
	int err = 0;

	sm5350_write_reg(drvdata->client, SM5350_BOOST_CTL_REG, drvdata->boost_ctl);
	sm5350_write_reg(drvdata->client, SM5350_PWM_CFG_REG, drvdata->pwm_cfg);
	sm5350_write_reg(drvdata->client, SM5350_CTL_B_BANK_EN_REG, drvdata->ctl_bank_en);
	sm5350_write_reg(drvdata->client, SM5350_BRIGHTNESS_CFG_REG, drvdata->map_mode);
	sm5350_write_reg(drvdata->client, SM5350_HVLED_CURR_SINK_OUT_CFG_REG, SM5350_HVLED_CURR_SINK_OUT_CFG);
	sm5350_write_reg(drvdata->client, SM5350_CTL_B_FULL_SCALE_CURR_REG, drvdata->full_scale_current);

	drvdata->enable = true;

	return err;
}

static int sm5350_bl_get_brightness(struct backlight_device *bl_dev)
{
	return bl_dev->props.brightness;
}

int sm5350_set_brightness(struct sm5350_data *drvdata, int brt_val)
{
	u8 brt_LSB = 0;
	u8 brt_MSB = 0;
	int index = 0, remainder;
	int code, code1, code2;
	printk("%s backlight_val = %d\n",__func__, brt_val);

	if ((drvdata->map_mode == 0) && (drvdata->led_current_align == ALIGN_AW99703))
		brt_val = brt_val*8383/10000+324;

	if (drvdata->brt_code_enable) {
		index = brt_val / 10;
		remainder = brt_val % 10;

		code1 = drvdata->brt_code_table[index];
		code2 = drvdata->brt_code_table[index+1];

		code = (code2 - code1) * remainder / 10 + code1;

		brt_LSB = code % 0x7;
		brt_MSB = (code >> 3) & 0xFF;
		printk("brt_LSB_1 %x, brt_MSB_1 %x\n", brt_LSB, brt_MSB);
	} else {
		brt_LSB = brt_val & 0x7;
		brt_MSB = (brt_val >> 3) & 0xFF;
	}
	printk("brt_LSB %x, brt_MSB %x\n", brt_LSB, brt_MSB);

	if (drvdata->enable == false)
		sm5350_init_registers(drvdata);

	if (drvdata->bank_B) {
		sm5350_write_reg(drvdata->client, SM5350_CTL_B_BRIGHTNESS_LSB_REG, brt_LSB);
		sm5350_write_reg(drvdata->client, SM5350_CTL_B_BRIGHTNESS_MSB_REG, brt_MSB);
	}

	if (brt_val == 0)
		drvdata->enable = false;

	return 0;
}

static int sm5350_bl_update_status(struct backlight_device *bl_dev)
{
	struct sm5350_data *drvdata = bl_get_data(bl_dev);
	int brt;

	if (bl_dev->props.state & BL_CORE_SUSPENDED)
		bl_dev->props.brightness = 0;

	brt = bl_dev->props.brightness;
	/*
	 * Brightness register should always be written
	 * not only register based mode but also in PWM mode.
	 */
	return sm5350_set_brightness(drvdata, brt);
}

static const struct backlight_ops sm5350_bl_ops = {
	.update_status = sm5350_bl_update_status,
	.get_brightness = sm5350_bl_get_brightness,
};

static void dump_sm5350_regs(struct sm5350_data *drvdata)
{
	u8 brt_LSB = 0;
	u8 brt_MSB = 0;
	u8 boost_ctl, pwm_cfg, ctl_bank_en, full_scale_current, revision;

	sm5350_read_reg(drvdata->client, SM5350_BOOST_CTL_REG, &boost_ctl);
	sm5350_read_reg(drvdata->client, SM5350_PWM_CFG_REG, &pwm_cfg);
	sm5350_read_reg(drvdata->client, SM5350_CTL_B_BANK_EN_REG, &ctl_bank_en);
	sm5350_read_reg(drvdata->client, SM5350_CTL_A_FULL_SCALE_CURR_REG, &full_scale_current);
	sm5350_read_reg(drvdata->client, SM5350_CTL_B_BRIGHTNESS_LSB_REG, &brt_LSB);
	sm5350_read_reg(drvdata->client, SM5350_CTL_B_BRIGHTNESS_MSB_REG, &brt_MSB);
	sm5350_read_reg(drvdata->client, SM5350_REVISION_REG, &revision);

	pr_err(">>-- boost_ctl(0x%x), pwm_cfg(0x%x), ctl_bank_en(0x%x), full_scale_current(0x%x), brt_LSB(0x%x), brt_MSB(0x%x), revision(0x%x).\n",
		boost_ctl, pwm_cfg, ctl_bank_en, full_scale_current, brt_LSB, brt_MSB, revision);
}

static int sm5350_get_dt_data(struct device *dev, struct sm5350_data *drvdata)
{
	int rc;
	u32 tmp;
	struct device_node *of_node = NULL;
	int len;
	const char *data;
	u32 *buf;
	int i = 0;

	of_node = dev->of_node;

	drvdata->en_gpio = of_get_named_gpio(of_node, "hwen-gpio", 0);
	if (drvdata->en_gpio < 0) {
		pr_err("%s,dt not get en_gpio, en_gpio = %d\n", __func__, drvdata->en_gpio);
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node, "boost-ctl", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->boost_ctl = (!rc ? tmp : 0);
	pr_debug("%s : boost_ctl=0x%x\n",__func__, drvdata->boost_ctl);

	rc = of_property_read_u32(of_node, "map-mode", &tmp);
	drvdata->map_mode= (!rc ? tmp : 1); /* 1: linear, 0: expo, linear as default*/
	pr_debug("%s : map_mode=0x%x\n",__func__, drvdata->map_mode);

	if (of_property_read_u32(of_node, "current-align-type", &drvdata->led_current_align))
		drvdata->led_current_align = ALIGN_NONE;
	pr_debug("%s : led_current_align=0x%x\n",__func__, drvdata->led_current_align);

	rc = of_property_read_u32(of_node, "sm5350,default-brightness", &tmp);
	drvdata->default_brightness= (!rc ? tmp : MAX_BRIGHTNESS);
	pr_debug("%s : default_brightness=0x%x\n",__func__, drvdata->default_brightness);

	rc = of_property_read_u32(of_node, "pwm-cfg", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->pwm_cfg = (!rc ? tmp : 0);
	pr_debug("%s : pwm_cfg=0x%x\n",__func__, drvdata->pwm_cfg);

	rc = of_property_read_u32(of_node, "ctl-bank-en", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->ctl_bank_en = (!rc ? tmp : 0);
	pr_debug("%s : ctl_bank_en=0x%x\n",__func__, drvdata->ctl_bank_en);

	if (drvdata->ctl_bank_en & 0x01)
		drvdata->bank_A = true;
	if (drvdata->ctl_bank_en & 0x02)
		drvdata->bank_B = true;

	pr_debug("%s : bank_A=%d bank_B=%d\n",__func__, drvdata->bank_A, drvdata->bank_B);

	rc = of_property_read_u32(of_node, "full-scale-current", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->full_scale_current = (!rc ? tmp : 0);

	pr_info("bank_A = %d, bank_B = %d, pwm_cfg = 0x%x, full_scale_current = 0x%x, map_mode = 0x%x, boost_ctl = 0x%x.\n",
		drvdata->bank_A, drvdata->bank_B, drvdata->pwm_cfg, drvdata->full_scale_current, drvdata->map_mode, drvdata->boost_ctl);

	drvdata->brt_code_enable = of_property_read_bool(of_node, "brt-code-enable");

	if (drvdata->brt_code_enable == false) {
		pr_info("%s : brt_code_enable = %d, rc = %d.\n",__func__, drvdata->brt_code_enable, rc);
		return rc;
	}

	data = of_get_property(of_node, "brt-code-table", &len);
	if (!data) {
		pr_err("%s: read brt-code-table failed\n", __func__);
		//return -ENOMEM;
	}

	len /= sizeof(u32);

	buf = kzalloc(len * sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rc = of_property_read_u32_array(of_node, "brt-code-table", buf, len);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",__func__, __LINE__);
		rc = -EINVAL;
		goto end;
	}

	drvdata->brt_code_table = kzalloc(len * sizeof(u16), GFP_KERNEL);
	if (!drvdata->brt_code_table) {
		pr_err("%s:%d, allocate memory failed\n",__func__, __LINE__);
		rc = -ENOMEM;
		goto end;
	}

	for (i=0; i < len; i++) {
		drvdata->brt_code_table[i] = (u16) buf[i];
		pr_debug("%s : buf=%d i=%d\n",__func__, buf[i], i);
	}
end:
	kfree(buf);
	return rc;
}

static int sm5350_bl_enable_hw(struct sm5350_data *drvdata){

	return gpio_request_one(drvdata->en_gpio, GPIOF_OUT_INIT_HIGH, "sm5350_hwen");

}

static int sm5350_read_revision(struct sm5350_data *drvdata)
{
	int ret = -1;
	u8 value = 0;
	unsigned char cnt = 0;

	while (cnt < 5) {
		ret = sm5350_read_reg(drvdata->client, SM5350_REVISION_REG, &value);
		if (ret < 0) {
			pr_err("%s: failed to read reg SM5350_REVISION_REG: %d\n",
				__func__, ret);
			return ret;
		}
		switch (value) {
		case 0x00:
			pr_info("%s sm5350 detected\n", __func__);
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

static int sm5350_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct sm5350_data *drvdata;
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	int err = 0;

	printk("%s entry\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct sm5350_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->enable = true;

	g_sm5350_data = drvdata;

	err = sm5350_get_dt_data(&client->dev, drvdata);
	if(err < 0) {
		pr_err("%s : get dt failed\n", __func__);
		err = -ENOMEM;
		goto err_init;
	}
	err = sm5350_bl_enable_hw(drvdata);
	if (err)
		goto err_init;

	i2c_set_clientdata(client, drvdata);

	/*sm5350 read revision*/
	err = sm5350_read_revision(drvdata);
	if (err < 0) {
		pr_err("%s : ID idenfy failed\n", __func__);
		goto err_init;
	}
	/*sm5350 read revision*/
	printk("sm-sm5350 detected success\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = MAX_BRIGHTNESS;
	props.max_brightness = MAX_BRIGHTNESS;
	bl_dev = backlight_device_register(SM5350_NAME, &client->dev,
					drvdata, &sm5350_bl_ops, &props);

	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);
	drvdata->bl_dev = bl_dev;
	sm5350_init_registers(drvdata);
	dump_sm5350_regs(drvdata);
	sm5350_set_brightness(drvdata, drvdata->default_brightness);

	printk("sm-sm5350 probe okay\n");
	return 0;

err_init:
	kfree(drvdata);
err_out:
	return err;
}

static int sm5350_remove(struct i2c_client *client)
{
	struct sm5350_data *drvdata = i2c_get_clientdata(client);

	backlight_device_unregister(drvdata->bl_dev);
	kfree(drvdata);
	return 0;
}

static const struct i2c_device_id sm5350_id[] = {
	{SM5350_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
       {.compatible = "sm-sm5350",}
};

MODULE_DEVICE_TABLE(i2c, sm5350_id);

static struct i2c_driver sm5350_i2c_driver = {
	.probe = sm5350_probe,
	.remove = sm5350_remove,
	.id_table = sm5350_id,
	.driver = {
		.name = SM5350_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_i2c_driver(sm5350_i2c_driver);
MODULE_DESCRIPTION("Back Light driver for SM5350");
MODULE_LICENSE("GPL v2");
