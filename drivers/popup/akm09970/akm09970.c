/*
 * akm09970.c
 *
 * step hall driver
 *
 * Copyright (c) 2018-2019 Lenovo Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <uapi/linux/sched/types.h>
#include "akm09970.h"

#define AKM09970_DEV_NAME "akm09970"
#define DRIVER_VERSION "1.0"

/***********************************************************/
/*debug macro*/
/***********************************************************/
#define AKM09970_DBG_ENABLE 1
#ifdef AKM09970_DBG_ENABLE
#define dbg(fmt, args...)  printk("[DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)
#define dbgn(fmt, args...)
#endif
#define dbg_func_in()     dbg("[akm09970-DBG-F.IN] %s", __func__)
#define dbg_func_out()    dbg("[akm09970-DBG-F.OUT] %s", __func__)
#define dbg_line()        dbg("[LINE] %d(%s)", __LINE__, __func__)

#define log_err(pdev, fmt, args...)          \
	dev_err(pdev, "[ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define log_info(pdev, fmt, args...)        \
	dev_info(pdev, "[INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)

#define make_u16(u8h, u8l) \
	((uint16_t)(((uint16_t)(u8h) << 8) | (uint16_t)(u8l)))
#define make_s16(u8h, u8l) \
	((int16_t)(((uint16_t)(u8h) << 8) | (uint16_t)(u8l)))

#define akm09970_adc_overflow(st1) ((0x0100 & (st1)) >> 8)

static int64_t akm09970_get_delay(struct device *dev);

/* select mode by delay */
static uint8_t akm09970_select_mode(int64_t delay)
{
	uint8_t mode;

	if (delay >= AKM09970_DELAY_025HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE1;
	else if (delay >= AKM09970_DELAY_05HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE2;
	else if (delay >= AKM09970_DELAY_1HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE3;
	else if (delay >= AKM09970_DELAY_10HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE4;
	else if (delay >= AKM09970_DELAY_20HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE5;
	else if (delay >= AKM09970_DELAY_50HZ)
		mode = AKM09970_MODE_CONT_MEASURE_MODE6;
	else
		mode = AKM09970_MODE_CONT_MEASURE_MODE7;

	return mode;
}

static int16_t akm09970_set_operation_mode(struct device *dev, const uint8_t mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	uint8_t i2c_data;
	uint16_t ctrl1_data = AKM09970_SET_CNTL1;
	int16_t ret;

	log_info(&client->dev, "%s: mode = 0x%x\n", __func__, mode);

	ret = i2c_smbus_write_i2c_block_data(client,
		AKM09970_REG_CNTL1, 2, (uint8_t *)&ctrl1_data);
	if (ret < 0) {
		log_err(&client->dev, "%s: i2c ctrl1 write fail\n", __func__);
		return ret;
	}

	i2c_data = mode;
	if (mode != AKM09970_MODE_POWER_DOWN) {
		i2c_data = AKM09970_SET_MODE(i2c_data);
	}

	ret = i2c_smbus_write_i2c_block_data(client, AKM09970_REG_CNTL2, 1, &i2c_data);
	if (ret < 0) {
		log_err(&client->dev, "%s: i2c cntrl2 write fail\n", __func__);
		return ret;
	}

	return 0;
}

static int32_t akm09970_get_debug(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.debug);
}

static void akm09970_set_debug(struct device *dev, int debug)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	atomic_set(&p_data->atm.debug, debug);
}

static int32_t akm09970_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.enable);
}

static void akm09970_set_enable(struct device *dev, int32_t enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);
	uint8_t mode;
	int64_t delay;

	log_info(&client->dev, " %s: enable = 0x%x\n", __func__, enable);

	mutex_lock(&p_data->mtx.enable);

	if (enable) { /*enable if state will be changed*/
		if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
			hrtimer_cancel(&p_data->sample_timer);
			delay = akm09970_get_delay(dev);
			mode = akm09970_select_mode(delay);
			akm09970_set_operation_mode(dev, mode);
			hrtimer_start(&p_data->sample_timer, ns_to_ktime(delay), HRTIMER_MODE_REL);
		}
	} else { /*disable if state will be changed*/
		if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
			hrtimer_cancel(&p_data->sample_timer);
			akm09970_set_operation_mode(dev, AKM09970_MODE_POWER_DOWN);
		}
	}
	atomic_set(&p_data->atm.enable, enable);

	mutex_unlock(&p_data->mtx.enable);
}

static int64_t akm09970_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);
	int64_t delay = 0;

	delay = atomic64_read(&p_data->atm.delay);

	return delay;
}

static void akm09970_set_delay(struct device *dev, int64_t delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);
	ktime_t time_out;
	uint8_t mode;

	atomic64_set(&p_data->atm.delay, delay);
	log_info(&client->dev, " %s: update delay = 0x%lld\n", __func__, delay);
	mutex_lock(&p_data->mtx.enable);

	if (akm09970_get_enable(dev)) {
		hrtimer_cancel(&p_data->sample_timer);
		time_out = ns_to_ktime(delay);
		mode = akm09970_select_mode(delay);
		akm09970_set_operation_mode(dev, mode);
		hrtimer_start(&p_data->sample_timer, time_out, HRTIMER_MODE_REL);
	}

	mutex_unlock(&p_data->mtx.enable);
}

static int32_t akm09970_power_init(struct i2c_client *client)
{
	int32_t ret = 0;
	struct device *pdev = &client->dev;
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	p_data->vdd = regulator_get(pdev, "vdd");
	if (IS_ERR(p_data->vdd)) {
		ret = PTR_ERR(p_data->vdd);
		log_err(pdev, "Failed to get %s VDD ret=%d\n", p_data->type, ret);
		goto exit;
	}

	if (regulator_count_voltages(p_data->vdd) > 0) {
		ret = regulator_set_voltage(p_data->vdd,
			AKM09970_VDD_MIN_UV, AKM09970_VDD_MAX_UV);
		if (ret) {
			log_err(pdev, "Failed to set %s vdd range ret=%d\n", p_data->type, ret);
			goto err_put_vdd;
		}
	} else
		log_info(pdev, "WARING: No VDD range set, default\n");

	return 0;

err_put_vdd:
	regulator_put(p_data->vdd);
exit:
	return ret;
}

static int32_t akm09970_measure(akm09970_i2c_data *p_data, struct mag_data_type *mag_data)
{
	struct i2c_client *client = p_data->client;
	int32_t err = 0;
	uint8_t reg_data[AKM09970_BDATA_SIZE];
	uint16_t st1;

	/* read data */
	err = i2c_smbus_read_i2c_block_data(client, AKM09970_REG_ST1_XYZ,
						AKM09970_BDATA_SIZE, reg_data);
	if (err < 0) {
		log_err(&client->dev, "%s: read reg data fail\n", __func__);
		return err;
	}

	/* check st1 ERRADCEN bit. */
	st1 = make_u16(reg_data[0], reg_data[1]);
	if (akm09970_adc_overflow(st1)) {
		log_err(&client->dev, "%s: sensor data overflow\n", __func__);
	}

	mutex_lock(&p_data->mtx.data);
	mag_data->x = make_s16(reg_data[6], reg_data[7]);
	mag_data->y = make_s16(reg_data[4], reg_data[5]);
	mag_data->z = make_s16(reg_data[2], reg_data[3]);
	mutex_unlock(&p_data->mtx.data);
	if (akm09970_get_debug(&client->dev)) {
		log_info(&client->dev, "state: %d,raw data x = %d,y = %d,z = %d\n",
			st1, mag_data->x, mag_data->y, mag_data->z);
	}

	return 0;
}

static void akm09970_func(akm09970_i2c_data *p_data)
{
	struct mag_data_type raw;
	int32_t err = 0;

	err = akm09970_measure(p_data, &raw);

	if (!err) {
		mutex_lock(&p_data->mtx.data);
		input_report_abs(p_data->input_dev, ABS_X, raw.x);
		input_report_abs(p_data->input_dev, ABS_Y, raw.y);
		input_report_abs(p_data->input_dev, ABS_Z, raw.z);
		input_sync(p_data->input_dev);
		mutex_unlock(&p_data->mtx.data);
	}
}

static void akm09970_work_func(struct work_struct *work)
{
	akm09970_i2c_data *p_data = container_of(work, akm09970_i2c_data, akm09970_work);
	akm09970_func(p_data);
}

static enum hrtimer_restart akm09970_timer_action(struct hrtimer *h)
{
	akm09970_i2c_data *p_data = container_of(h, akm09970_i2c_data, sample_timer);
	p_data->sync_flag = true;
	wake_up(&p_data->sync_complete);

	hrtimer_forward_now(&p_data->sample_timer, ns_to_ktime(akm09970_get_delay(&p_data->client->dev)));

	return HRTIMER_RESTART;
}

static __ref int32_t akm09970_thread(void *arg)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)arg;
	int32_t ret = 0;

	while (!kthread_should_stop()) {
		do {
			ret = wait_event_interruptible(p_data->sync_complete,
				p_data->sync_flag || kthread_should_stop());
		} while (ret != 0);

		if (kthread_should_stop())
			break;

		p_data->sync_flag = false;

		akm09970_func(p_data);
	}

	return 0;
}

static int32_t akm09970_hard_reset(int32_t gpio_rst)
{
	int32_t ret;

	ret = gpio_direction_output(gpio_rst, 1);
	if (ret != 0) {
		dbg("reset pin pre pull up failed, ret = %d\n", ret);
		goto reset_failed;
	}
	mdelay(AKM09970_RESET_DELAY_MS);

	ret = gpio_direction_output(gpio_rst, 0);
	if (ret != 0) {
		dbg("reset pin pull down failed, ret = %d\n", ret);
		goto reset_failed;
	}
	mdelay(AKM09970_RESET_DELAY_MS);

	ret = gpio_direction_output(gpio_rst, 1);
	if (ret != 0) {
		dbg("reset pin end pull up failed, ret = %d\n", ret);
		goto reset_failed;
	}
	mdelay(AKM09970_RESET_DELAY_MS);

	dbg("hardware reset success\n");
	return 0;
reset_failed:
	dbg("hardware reset failed\n");
	return ret;
}

/*static int32_t akm09970_soft_reset(struct device *dev)
{
	uint8_t i2c_data;
	int32_t ret;

	struct i2c_client *client = to_i2c_client(dev);

	i2c_data = AKM09970_SOFT_RESET_VALUE;
	ret = i2c_smbus_write_i2c_block_data(client, AKM09970_REG_SRST, 1, &i2c_data);
	if (ret < 0) {
		log_err(&client->dev, "i2c_transfer was failed (%d),reset failed.", ret);
		return ret;
	}

	mdelay(AKM09970_RESET_DELAY_MS);
	dbg("software reset success\n");

	return 0;
}*/

static int32_t akm09970_parse_dt(struct i2c_client *client)
{
	uint32_t init_freq;
	int64_t init_delay;
	int32_t ret = 0;
	struct device *pdev = &client->dev;
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;

	ret = of_property_read_string(np, "label", &p_data->type);
	if (ret) {
		log_err(pdev, "Failed to get lable\n");
		goto exit;
	}

	ret = of_property_read_u32(np, "magnachip,init-freq", &init_freq);
	if (ret && (ret != -EINVAL)) {
		log_err(pdev, "Failed to get init-freq\n");
		goto exit;
	}

	if (init_freq < AKM09970_ODR_10HZ)
		init_freq = AKM09970_ODR_10HZ;
	else if (init_freq <= AKM09970_ODR_20HZ)
		init_freq = AKM09970_ODR_20HZ;
	else if (init_freq <= AKM09970_ODR_50HZ)
		init_freq = AKM09970_ODR_50HZ;
	else
		init_freq = AKM09970_ODR_100HZ;

	init_delay = 1000000000LL / init_freq;

	atomic_set(&p_data->atm.delay, init_delay);

	p_data->int_en = of_property_read_bool(np, "magnachip,use-interrupt");

	p_data->igpio = of_get_named_gpio_flags(pdev->of_node,
		"magnachip,gpio-int", 0, NULL);

	p_data->gpio_rst = of_get_named_gpio(np, "magnachip,gpio_rst", 0);

	p_data->use_hrtimer = of_property_read_bool(np, "magnachip,use-hrtimer");

	p_data->power_always_on = of_property_read_bool(np, "magnachip,power-always-on");

exit:
	return ret;
}

static int32_t akm09970_check_chip_id(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	uint8_t reg_wia[AKM09970_WIA_SIZE];
	uint16_t chip_id;
	int32_t ret;

	ret = i2c_smbus_read_i2c_block_data(client,
		AKM09970_REG_WIA, AKM09970_WIA_SIZE, reg_wia);
	if (ret < 0) {
		log_err(&client->dev, "read chip_id fail\n");
		return ret;
	}

	chip_id = make_u16(reg_wia[1], reg_wia[0]);
	if (chip_id != AKM09970_WIA_VALUE) {
		log_err(&client->dev, "%s: invalid device id, chip_id = 0x%x\n",
			__func__, chip_id);
		return -ENODEV;
	}
	return 0;
}

static int32_t akm09970_init_device(struct i2c_client *client)
{
	int32_t err = 0;
	struct device *pdev = &client->dev;
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	/* init variables */
	atomic_set(&p_data->atm.enable, 0);
	atomic_set(&p_data->atm.delay, AKM09970_MIN_DELAY_NS);
#ifdef AKM09970_DBG_ENABLE
	atomic_set(&p_data->atm.debug, 1);
#else
	atomic_set(&p_data->atm.debug, 0);
#endif

	akm09970_set_delay(pdev, AKM09970_MIN_DELAY_NS);
	akm09970_set_debug(pdev, 0);

	err = devm_gpio_request(pdev, p_data->gpio_rst, "gpio_rst");
	if (err != 0) {
		log_err(pdev, "Failed to request reset gpio %s (%d)", p_data->type, err);
		return err;
	}

	err = akm09970_hard_reset(p_data->gpio_rst);
	if (err) {
		log_err(pdev, "Failed to reset %s (%d)", p_data->type, err);
		return err;
	}

	err = akm09970_check_chip_id(pdev);
	if (err < 0) {
		log_err(pdev, "akm09970_check_chip_id fail\n");
		return err;
	}
	log_info(pdev, "%s initializing device was success", p_data->type);

	return 0;
}

/* input device interface */
static int32_t akm09970_input_dev_init(akm09970_i2c_data *p_data)
{
	struct input_dev *dev;
	int32_t err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = p_data->type;
	dev->id.bustype = BUS_I2C;

	input_set_drvdata(dev, p_data);
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, DEFAULT_EVENT_DATA_CAPABILITY_MIN,
		DEFAULT_EVENT_DATA_CAPABILITY_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, DEFAULT_EVENT_DATA_CAPABILITY_MIN,
		DEFAULT_EVENT_DATA_CAPABILITY_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, DEFAULT_EVENT_DATA_CAPABILITY_MIN,
		DEFAULT_EVENT_DATA_CAPABILITY_MAX, 0, 0);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	p_data->input_dev = dev;

	return 0;
}

static void akm09970_input_dev_terminate(akm09970_i2c_data *p_data)
{
	struct input_dev *dev = p_data->input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}

/* sysfs group interface */
static ssize_t akm09970_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;

	return snprintf(buf, 20, "%d\n", akm09970_get_enable(cdev));
}

static ssize_t akm09970_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;
	uint32_t enable = 0;

	if (kstrtouint(buf, 10, &enable)) {
		log_err(dev, "Error value: %s\n", buf);
		goto err;
	}

	enable = !!enable;

	if (enable == atomic_read(&p_data->atm.enable)) {
		log_info(cdev, "ignore duplicate set\n");
		goto err;
	}

	akm09970_set_enable(cdev, enable);
err:
	return count;
}

static ssize_t akm09970_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;

	return snprintf(buf, 20, "%lld\n", akm09970_get_delay(cdev));
}

static ssize_t akm09970_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;
	uint32_t freq = 0;
	int64_t delay;

	if (kstrtouint(buf, 10, &freq)) {
		log_err(dev, "Error value: %s\n", buf);
		goto err;
	}

	if (freq <= AKM09970_ODR_10HZ)
		freq = AKM09970_ODR_10HZ;
	else if (freq <= AKM09970_ODR_20HZ)
		freq = AKM09970_ODR_20HZ;
	else if (freq <= AKM09970_ODR_50HZ)
		freq = AKM09970_ODR_50HZ;
	else
		freq = AKM09970_ODR_100HZ;

	delay = 1000000000LL / freq;
	if (delay == atomic_read(&p_data->atm.delay)) {
		log_info(cdev, "ignore duplicate set\n");
		goto err;
	}

	akm09970_set_delay(cdev, delay);

err:
	return count;
}

static ssize_t akm09970_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;

	return snprintf(buf, 20, "%d\n", akm09970_get_debug(cdev));
}

static ssize_t akm09970_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;
	uint32_t debug = 0;

	if (kstrtouint(buf, 10, &debug)) {
		log_err(dev, "Error value: %s\n", buf);
		goto err;
	}

	debug = !!debug;

	if (debug == atomic_read(&p_data->atm.debug)) {
		log_info(cdev, "ignore duplicate set\n");
		goto err;
	}

	akm09970_set_debug(dev, debug);

err:
	return count;
}

static int64_t akm09970_check_var(const int16_t buf[], int32_t size, int16_t *ave_data)
{
	int32_t i;
	int64_t sum = 0, var_sum = 0, ave = 0, var = 0;

	for (i = 0; i < size; i++)
		sum = sum +buf[i];

	ave = (int64_t)(sum/size);
	*ave_data = (int16_t)ave;

	for (i = 0; i < size; i++)
		var_sum = var_sum + (buf[i] -ave)*(buf[i] -ave);
	var = (int64_t)(var_sum/size);
	return var;
}

static int64_t akm09970_check_diff(const int16_t buf[], int32_t size)
{
	int32_t i, ret = AKM09970_CHECK_ERR;
	int64_t val;

	val = buf[0];
	for (i = 0; i < size; i++) {
		if (val != buf[i]) {
			ret = 0;
			break;
		}
		val = buf[i];
	}

	return ret;
}

static int32_t akm09970_check_axis_data(const int16_t buf_data[], int32_t size, int16_t *ave_data)
{
	int64_t buf_var;
	int32_t ret;

	buf_var = akm09970_check_var(buf_data, size, ave_data);
	dbg("var[%d, %d] = %lld\n", AKM09970_VAR_LIMIT_LOW, AKM09970_VAR_LIMIT_HIGH, buf_var);
	if (buf_var <= AKM09970_VAR_LIMIT_LOW) {
		ret = akm09970_check_diff(buf_data, size);
		if (ret != 0) {
			dbg("check diff failed, too low, ret = %d\n", ret);
			return ret;
		}
	}

	if (buf_var >= AKM09970_VAR_LIMIT_HIGH) {
		dbg("check diff failed, too high\n");
		return AKM09970_CHECK_ERR;
	}

	return 0;
}

static int32_t akm09970_get_onedata(akm09970_i2c_data *p_data, struct mag_data_type *mag_data)
{
	struct i2c_client *client = p_data->client;
	struct device *cdev = &client->dev;
	int32_t ret;
	int32_t st1_retry_cnt = AKM09970_DRAY_RETRY_MAX;
	uint8_t st1_reg[AKM09970_ST1_SIZE];
	uint8_t reg_data[AKM09970_REG_ST1_XYZ];
	uint16_t st1;
	bool st1_rdy = false;

	akm09970_set_operation_mode(cdev, AKM09970_MODE_SNG_MEASURE);
	mdelay(AKM09970_DRDY_DELAY_MS);

	while (st1_retry_cnt > 0) {
		ret = i2c_smbus_read_i2c_block_data(client,
			AKM09970_REG_ST1, AKM09970_ST1_SIZE, st1_reg);
		if (ret < 0) {
			log_err(cdev, "read st1 reg data for DRDY fail\n");
			return ret;
		}
		st1 = make_u16(st1_reg[0], st1_reg[1]);
		if (st1 == AKM09970_ST1_IS_DRDY_VALUE) {
			st1_rdy = true;
			break;
		} else
			mdelay(AKM09970_DRDY_DELAY_MS);
		log_err(cdev, "ST1 not ready retry,cnt= %d, st1 = 0x%x\n",
			st1_retry_cnt, st1);
		st1_retry_cnt--;
	}

	if (st1_rdy) {
		ret = i2c_smbus_read_i2c_block_data(client, AKM09970_REG_ST1_XYZ,
			AKM09970_BDATA_SIZE, reg_data);
		if (ret < 0) {
			log_err(cdev, "read XYZ reg data fail\n");
			return ret;
		}

		mag_data->x = make_s16(reg_data[6], reg_data[7]);
		mag_data->y = make_s16(reg_data[4], reg_data[5]);
		mag_data->z = make_s16(reg_data[2], reg_data[3]);

		ret = i2c_smbus_read_i2c_block_data(client,
			AKM09970_REG_ST1, AKM09970_ST1_SIZE, st1_reg);
		if (ret < 0) {
			log_err(cdev, "read reg data for NO_DRAY fail\n");
			return ret;
		}

		st1 = make_u16(st1_reg[0], st1_reg[1]);
		if (st1 != AKM09970_ST1_NO_DRDY_VALUE) {
			log_err(cdev, "ST1 NO_DRDY check failed\n");
			return AKM09970_CHECK_ERR;
		}
	} else {
		log_err(cdev, "read onedata err\n");
		return AKM09970_CHECK_ERR;
	}

	return 0;
}

#define MEASURE_CNTS 3
static ssize_t akm09970_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct mag_data_type raw;
	struct mag_data_type data_buf[MEASURE_CNTS];
	uint32_t i, ret;
	int16_t buf_tmp[MEASURE_CNTS], axis_val;

	memset(data_buf, 0, sizeof(data_buf));

	for (i = 0; i < MEASURE_CNTS; i++) {
		ret = akm09970_get_onedata(p_data, &data_buf[i]);
		if (ret != 0)
			goto err;
	}

	for (i = 0; i < MEASURE_CNTS; i++)
		buf_tmp[i] = data_buf[i].x;
	ret = akm09970_check_axis_data(buf_tmp, MEASURE_CNTS, &axis_val);
	if (ret != 0)
		goto  err;
	raw.x = axis_val;

	for (i = 0; i < MEASURE_CNTS; i++)
		buf_tmp[i] = data_buf[i].y;
	ret = akm09970_check_axis_data(buf_tmp, MEASURE_CNTS, &axis_val);
	if (ret != 0)
		goto  err;
	raw.y = axis_val;

	for (i = 0; i < MEASURE_CNTS; i++)
		buf_tmp[i] = data_buf[i].z;
	ret = akm09970_check_axis_data(buf_tmp, MEASURE_CNTS, &axis_val);
	if (ret != 0)
		goto  err;
	raw.z = axis_val;

	return snprintf(buf, 20, "%d %d %d\n", raw.x, raw.y, raw.z);
err:
	return snprintf(buf, 20, "%d %d %d\n", 0, 0, 0);
}

static ssize_t akm09970_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	akm09970_i2c_data *p_data = (akm09970_i2c_data *)dev_get_drvdata(dev);
	struct i2c_client *client = p_data->client;
	char *p = buf;
#define DUMP_SIZE 6
	int32_t err = 0, i = 0, j = 0;
	const char *reg_name[DUMP_SIZE] = {"WIA", "ST1", "ST1_XYZ",
					  "CNTL1", "CNTL2", "SRST"};
	int32_t reg[DUMP_SIZE] = {AKM09970_REG_WIA, AKM09970_REG_ST1, AKM09970_REG_ST1_XYZ,
				  AKM09970_REG_CNTL1, AKM09970_REG_CNTL2, AKM09970_REG_SRST};
	int32_t reg_data_size[DUMP_SIZE] = {AKM09970_WIA_SIZE, AKM09970_ST1_SIZE,
					    AKM09970_BDATA_SIZE, AKM09970_CNTL1_SIZE,
					    AKM09970_CNTL2_SIZE, AKM09970_SRST_SIZE};
	uint8_t reg_data[AKM09970_BDATA_SIZE] = { 0 };

	for (i = 0; i < DUMP_SIZE; i++) {
		err = i2c_smbus_read_i2c_block_data(client, reg[i], reg_data_size[i], reg_data);
		if (err < 0) {
			log_err(&client->dev, "%s: read reg data fail\n", __func__);
			return err;
		}
		p += snprintf(p, PAGE_SIZE, "%s: ", reg_name[i]);
		for (j = 0; j < reg_data_size[i]; j++)
			p += snprintf(p, PAGE_SIZE, "0x%02x ", reg_data[j]);
		p += snprintf(p, PAGE_SIZE, "\n");

		log_info(&client->dev, " %s value: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			reg_name[i], reg_data[0], reg_data[1], reg_data[2], reg_data[3],
			reg_data[4], reg_data[5], reg_data[6], reg_data[7]);
		memset(reg_data, 0, sizeof(reg_data));
	}

	return (p-buf);
}

static DEVICE_ATTR(enable,  S_IRUGO|S_IWUSR|S_IWGRP, akm09970_enable_show, akm09970_enable_store);
static DEVICE_ATTR(delay,   S_IRUGO|S_IWUSR|S_IWGRP, akm09970_delay_show,  akm09970_delay_store);
static DEVICE_ATTR(debug,   S_IRUGO|S_IWUSR|S_IWGRP, akm09970_debug_show,  akm09970_debug_store);
static DEVICE_ATTR(rawdata, S_IRUGO|S_IWUSR|S_IWGRP, akm09970_data_show,   NULL);
static DEVICE_ATTR(dump,    S_IRUGO|S_IWUSR|S_IWGRP, akm09970_dump_show,   NULL);

static struct attribute *akm09970_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_debug.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_dump.attr,
	NULL
};

static struct attribute_group akm09970_attribute_group = {
	.attrs = akm09970_attributes
};

static const struct attribute_group *akm09970_attr_groups[] = {
	&akm09970_attribute_group,
	NULL
};

static int32_t akm09970_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t err = 0;
	akm09970_i2c_data *p_data = NULL;

	dbg_func_in();

	p_data = kzalloc(sizeof(akm09970_i2c_data), GFP_KERNEL);
	if (!p_data) {
		log_err(&client->dev, "kernel memory alocation was failed");
		err = -ENOMEM;
		goto err_nomem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		log_err(&client->dev, "i2c_check_functionality was failed");
		err = -ENODEV;
		goto err_nodev;
	}

	mutex_init(&p_data->mtx.enable);
	mutex_init(&p_data->mtx.data);
	p_data->power_enabled = false;
	p_data->client = client;

	i2c_set_clientdata(client, p_data);

	err = akm09970_parse_dt(client);
	if (err) {
		log_err(&client->dev, "Failed to parse device tree\n");
		err = -ENODEV;
		goto err_nodev;
	}

	if (!p_data->power_always_on) {
		err = akm09970_power_init(client);
		if (err) {
			log_err(&client->dev, "Failed to get sensor regulators\n");
			err = -EINVAL;
			goto err_nodev;
		}
	} else
		p_data->power_enabled = true;

	err = akm09970_init_device(client);
	if (err) {
		log_err(&client->dev, "akm09970_init_device was failed(%d)", err);
		goto err_nodev;
	}
	log_info(&client->dev, "%s was found", p_data->type);

	INIT_WORK(&p_data->akm09970_work, akm09970_work_func);
	p_data->akm09970_wq = alloc_workqueue("akm09970_wq", WQ_HIGHPRI, 0);
	if (!p_data->akm09970_wq) {
		log_err(&client->dev, "failed alloc wq");
		goto err_nodev;
	}
	p_data->sync_flag = false;
	init_waitqueue_head(&p_data->sync_complete);
	p_data->akm09970_task = kthread_create(akm09970_thread, p_data, "%s_task", p_data->type);
	if (IS_ERR(p_data->akm09970_task)) {
		log_err(&client->dev, "failed create %s task", p_data->type);
		goto err_wk;
	}
	wake_up_process(p_data->akm09970_task);

	hrtimer_init(&p_data->sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p_data->sample_timer.function = akm09970_timer_action;

	/* init input device */
	err = akm09970_input_dev_init(p_data);
	if (err) {
		log_err(&client->dev, "%s: Failed to create input dev(%d)", p_data->type, err);
		goto err_task;
	}
	log_info(&client->dev, "%s was initialized", p_data->type);

	/* create sysfs group */
	p_data->akm09970_class = class_create(THIS_MODULE, p_data->type);
	if (IS_ERR(p_data->akm09970_class)) {
		log_err(&client->dev, "Failed to create class\n");
		goto err_group;
	}

	p_data->sysfs_dev = device_create_with_groups(p_data->akm09970_class,
		&client->dev, MKDEV(0, 0), p_data, akm09970_attr_groups,
		"%s", AKM09970_CONTROL);
	if (IS_ERR(p_data->sysfs_dev)) {
		log_err(&client->dev, "Failed to create device\n");
		goto err_groups;
	}

	dbg_func_out();
	return 0;

err_groups:
	class_destroy(p_data->akm09970_class);
err_group:
	akm09970_input_dev_terminate(p_data);
err_task:
	kthread_stop(p_data->akm09970_task);
err_wk:
	destroy_workqueue(p_data->akm09970_wq);
err_nodev:
	kfree(p_data);
err_nomem:
	p_data = NULL;
	return err;
}

static int32_t akm09970_i2c_remove(struct i2c_client *client)
{
	akm09970_i2c_data *p_data = i2c_get_clientdata(client);

	akm09970_set_enable(&client->dev, 0);
	destroy_workqueue(p_data->akm09970_wq);
	kthread_stop(p_data->akm09970_task);
	regulator_put(p_data->vdd);
	device_destroy(p_data->akm09970_class, MKDEV(0, 0));
	class_destroy(p_data->akm09970_class);

	akm09970_input_dev_terminate(p_data);
	if (p_data->igpio != -1)
		gpio_free(p_data->igpio);

	kfree(p_data);

	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id akm09970_i2c_id[] = {
	{AKM09970_DEV_NAME, 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "magnachip,akm09970"},
	{},
};
#endif

static struct i2c_driver akm09970_i2c_driver = {
	.driver = {
		.name = AKM09970_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
#endif
	},
	.probe = akm09970_i2c_probe,
	.remove = akm09970_i2c_remove,
	.id_table = akm09970_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init akm09970_init(void)
{
	return i2c_add_driver(&akm09970_i2c_driver);
}

static void __exit akm09970_exit(void)
{
	i2c_del_driver(&akm09970_i2c_driver);
}

/*----------------------------------------------------------------------------*/
module_init(akm09970_init);
module_exit(akm09970_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("akm09970 hallswitch driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
