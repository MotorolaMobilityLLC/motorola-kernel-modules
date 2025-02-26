#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#include <linux/mmi_wake_lock.h>
#endif
#include <linux/phone_case_detection_notify.h>
#include <linux/version.h>

#define DRIVER_NAME "hall_phone_case_detect"
#define LOG_DBG(fmt, args...)    pr_debug(DRIVER_NAME " [DBG]" "<%s:%d>"fmt, __func__, __LINE__, ##args)
#define LOG_INFO(fmt, args...)   pr_info(DRIVER_NAME " [INFO]" "<%s:%d>"fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)    pr_err(DRIVER_NAME " [ERR]" "<%s:%d>"fmt, __func__, __LINE__, ##args)

static int hall_sensor_probe(struct platform_device *pdev);
static int hall_sensor_remove(struct platform_device *pdev);

struct hall_gpio {
	char gpio_name[32];
	int gpio;
	int irq;
	int gpio_high_report_val;
	int gpio_low_report_val;
};

static struct hall_sensor_str {
	int status;
	int enable;
	int gpio_num;
	int report_val;
	int phone_case_detect;
	struct regulator *hall_vdd;
	struct hall_gpio *gpio_list;
	spinlock_t mHallSensorLock;
	#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wake_lock;
	#else
	struct wakeup_source *wake_lock;
	#endif
	struct input_dev *hall_dev;
	struct sensors_classdev sensors_phone_case_cdev;
	struct delayed_work hall_sensor_work;
	struct delayed_work hall_sensor_irq_work;
	bool init_completed;
}* hall_sensor_dev;

static struct workqueue_struct *hall_sensor_wq;
static struct workqueue_struct *hall_sensor_irq_wq;

static BLOCKING_NOTIFIER_HEAD(phone_case_detection_notifier_list);

/**
 * phone_case_detection_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to phone_case_detection_notifier_list, which would be called when
 * puting on/off a phone case.
 */
int phone_case_detection_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&phone_case_detection_notifier_list,
						nb);
}
EXPORT_SYMBOL(phone_case_detection_register_client);

/**
 * phone_case_detection_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * phone_case_detection_notifier_list.
 */
int phone_case_detection_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&phone_case_detection_notifier_list,
						  nb);
}
EXPORT_SYMBOL(phone_case_detection_unregister_client);

/**
 * phone_case_detection_notifier_call_chain - notify clients of phone case
 * detection events.
 * @val: event PHONE_CASE_DETECTION_MOUNTED or PHONE_CASE_DETECTION_UNMOUNTED
 * @v: notifier data, NULL.
 */
static int phone_case_detection_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&phone_case_detection_notifier_list, val,
					    v);
}

#ifdef CONFIG_OF
static const struct of_device_id hall_phone_case_match[] = {
	{ .compatible = "hall,hall_phone_case_detect", },
	{}
};
#else
#define hall_phone_case_match NULL
#endif
MODULE_DEVICE_TABLE(of, hall_phone_case_match);

static struct platform_driver hall_phone_case_driver = {
	.probe		= hall_sensor_probe,
	.remove		= hall_sensor_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table    = hall_phone_case_match,
	},
};

void check_and_send(void)
{
	int i;
	int report_val = 0;
	unsigned long flags;

	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		LOG_INFO("hall report gpio%d = %d\r\n", i, gpio_get_value(hall_sensor_dev->gpio_list[i].gpio));
		if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) > 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_high_report_val;
		else if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) == 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_low_report_val;
	}
	if(hall_sensor_dev->report_val != report_val){
		LOG_INFO("hall report %d", report_val);
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		hall_sensor_dev->report_val = report_val;
		if(hall_sensor_dev->phone_case_detect) {
			input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, hall_sensor_dev->report_val);
			input_sync(hall_sensor_dev->hall_dev);
		}
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

		if(hall_sensor_dev->report_val)
			phone_case_detection_notifier_call_chain(PHONE_CASE_DETECTION_MOUNTED, NULL);
		else
			phone_case_detection_notifier_call_chain(PHONE_CASE_DETECTION_UNMOUNTED, NULL);
	}
}

/**
 * phone_case_get_hall_state - get hall state
 * return value 1: PHONE_CASE_DETECTION_MOUNTED
 * return value 0: PHONE_CASE_DETECTION_UNMOUNTED
 * return value -1: hall not enabled
 */
int phone_case_detection_get_hall_state(void)
{
	int ret = -1;

	if (hall_sensor_dev) {
		check_and_send();
		ret = hall_sensor_dev->report_val;
	}

	return ret;
}
EXPORT_SYMBOL(phone_case_detection_get_hall_state);

void hall_enable(bool enable)
{
	unsigned long flags;

	if (enable && !hall_sensor_dev->enable)
	{
		LOG_INFO("hall_phone_case_sensor enable\r\n");
		hall_sensor_dev->enable = 1;
		if(hall_sensor_dev->phone_case_detect) {
			/* report initial state on enable */
			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, hall_sensor_dev->report_val);
			input_sync(hall_sensor_dev->hall_dev);
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
		}
	}
	else if (!enable && hall_sensor_dev->enable)
	{
		LOG_INFO("hall_phone_case_sensor disable\r\n");
		hall_sensor_dev->enable = 0;
	}
}

static int hallphone_case_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	hall_sensor_dev->phone_case_detect = enable;
	hall_enable(enable);
	if (enable == 0)
	{
		input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1);
		input_sync(hall_sensor_dev->hall_dev);
	}
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static ssize_t hall_enable_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
#else
static ssize_t hall_enable_store(const struct class *class,
		const struct class_attribute *attr,
		const char *buf, size_t count)
#endif
{
	if (!strncmp(buf, "1", 1))
	{
		hall_enable(true);
	}
	else if (!strncmp(buf, "0", 1))
	{
		hall_enable(false);
	}
	return count;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static ssize_t hall_enable_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
#else
static ssize_t hall_enable_show(const struct class *class,
		const struct class_attribute *attr,
		char *buf)
#endif
{
	return sprintf(buf, "phone_case:%d\n", hall_sensor_dev->enable);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static ssize_t hall_rawdata_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
#else
static ssize_t hall_rawdata_show(const struct class *class,
		const struct class_attribute *attr,
		char *buf)
#endif
{
	return sprintf(buf, "%d\n", hall_sensor_dev->report_val);
}

static struct class_attribute class_attr_enable =
__ATTR(enable, 0660, hall_enable_show, hall_enable_store);
static struct class_attribute class_attr_rawdata =
__ATTR(rawdata, 0660, hall_rawdata_show, NULL);

static struct attribute *hall_class_attrs[] = {
	&class_attr_enable.attr,
	&class_attr_rawdata.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hall_class);

struct class hall_class = {
	.name                   = "hall",
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
	.owner                  = THIS_MODULE,
#endif
	.class_groups           = hall_class_groups,
};

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	LOG_DBG("hall_sensor_interrupt_handler = %d\r\n", irq);
	queue_delayed_work(hall_sensor_irq_wq, &hall_sensor_dev->hall_sensor_irq_work, msecs_to_jiffies(0));
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&hall_sensor_dev->wake_lock, msecs_to_jiffies(100));
#else
	PM_WAKEUP_EVENT(hall_sensor_dev->wake_lock,msecs_to_jiffies(100));
#endif
	return IRQ_HANDLED;
}

static void hall_sensor_irq_work_function(struct work_struct *work)
{
	LOG_DBG("enter hall_sensor_irq_work_function\r\n");
	if (hall_sensor_dev->init_completed) {
		cancel_delayed_work(&hall_sensor_dev->hall_sensor_work);
		queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	}
}

static void hall_sensor_work_function(struct work_struct *work)
{
	int i;
	LOG_INFO("enter hall_sensor_work_function\r\n");
	/* go on to complete the init process */
	if (!hall_sensor_dev->init_completed) {
		hall_sensor_dev->init_completed = true;
		/* schedule a short time delayed work, it may be canceled if irq
			triggers immediately after enable */
		queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, msecs_to_jiffies(5));
		for (i = 0; i < hall_sensor_dev->gpio_num; i++)
		{
			if (hall_sensor_dev->gpio_list[i].irq) {
				enable_irq(hall_sensor_dev->gpio_list[i].irq);
				enable_irq_wake(hall_sensor_dev->gpio_list[i].irq);
				LOG_INFO("enable irq: %d\r\n", hall_sensor_dev->gpio_list[i].irq);
			}
		}
		LOG_INFO("init completed\r\n");
	} else {
		check_and_send();
	}
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
	enum of_gpio_flags flags;
#endif
	struct device_node *np = pdev->dev.of_node;
	int i;
	int err ;
	const char *name_temp;
	int32_t rc = 0;
	//struct totalHallInformation *pHallInformationData = NULL;
	LOG_INFO("enter hall_sensor_probe \r\n");
	//Memory allocation
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		LOG_ERR("Memory allocation fails for hall sensor\r\n");
		ret = -ENOMEM;
		goto fail_for_mem;
	}

	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	hall_sensor_dev->enable = 0;
	hall_sensor_dev->phone_case_detect = 0;
	hall_sensor_dev->report_val = -1;
	hall_sensor_dev->init_completed = false;

	//tcmd node
	ret = of_property_read_string(np, "hall,factory-class-name", &hall_class.name);
	ret = class_register(&hall_class);
	ret = of_property_read_string(np, "hall,input-dev-name", &name_temp);
	LOG_INFO("hall_input num :%s\r\n", name_temp);

	ret =  of_property_read_u32(np,"hall,nirq-gpio-num", &hall_sensor_dev->gpio_num);
	if (ret < 0)
	{
		LOG_ERR("GPIO for hall sensor does not exist\r\n");
	}
	LOG_INFO("gpio num :%d\r\n", hall_sensor_dev->gpio_num);

	  hall_sensor_dev->hall_dev = input_allocate_device();
	  	if (!hall_sensor_dev->hall_dev){
			LOG_ERR(" hall_indev allocation fails\r\n" );
			return -ENOMEM;
		}
	hall_sensor_dev->hall_dev->name = name_temp;
	hall_sensor_dev->hall_dev->dev.parent= NULL;
	LOG_INFO("hall_ num :%s", hall_sensor_dev->hall_dev->name);
	/* Set all the keycodes */
	//input_set_capability(hall_sensor_dev->hall_dev, EV_SW, SW_CAMERA_LENS_COVER);
	__set_bit(EV_ABS, hall_sensor_dev->hall_dev->evbit);
	input_set_abs_params(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1, 100, 0, 0);

	err = input_register_device(hall_sensor_dev->hall_dev);
	if (ret) {
		LOG_ERR("halinput registration fails\r\n");
		return -ENOMEM;
	}
	input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1);
	input_sync(hall_sensor_dev->hall_dev);

	hall_sensor_dev->sensors_phone_case_cdev.sensors_enable = hallphone_case_enable;
	hall_sensor_dev->sensors_phone_case_cdev.sensors_poll_delay = NULL;
	hall_sensor_dev->sensors_phone_case_cdev.name = hall_sensor_dev->hall_dev->name;
	hall_sensor_dev->sensors_phone_case_cdev.vendor = "motorola";
	hall_sensor_dev->sensors_phone_case_cdev.version = 1;
	hall_sensor_dev->sensors_phone_case_cdev.type = SENSOR_TYPE_MOTO_HALL;
	hall_sensor_dev->sensors_phone_case_cdev.max_range = "5";
	hall_sensor_dev->sensors_phone_case_cdev.resolution = "5.0";
	hall_sensor_dev->sensors_phone_case_cdev.sensor_power = "3";
	hall_sensor_dev->sensors_phone_case_cdev.min_delay = 0;
	hall_sensor_dev->sensors_phone_case_cdev.fifo_reserved_event_count = 0;
	hall_sensor_dev->sensors_phone_case_cdev.fifo_max_event_count = 0;
	hall_sensor_dev->sensors_phone_case_cdev.delay_msec = 100;
	hall_sensor_dev->sensors_phone_case_cdev.enabled = 0;

	err = sensors_classdev_register(&hall_sensor_dev->hall_dev->dev, &hall_sensor_dev->sensors_phone_case_cdev);
	if (err < 0)
		LOG_ERR("create cap sensor_class  file failed (%d)\r\n", err);

	hall_sensor_dev->gpio_list = kzalloc(sizeof (struct hall_gpio) * hall_sensor_dev->gpio_num, GFP_KERNEL);
	if (!hall_sensor_dev->gpio_list) {
		LOG_ERR("Memory allocation fails for hall sensor\r\n");
		ret = -ENOMEM;
		goto fail_for_mem;
	}
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		char *gpio_name = hall_sensor_dev->gpio_list[i].gpio_name;
		sprintf(gpio_name, "hall,nirq-gpio-high-val_%d", i);
		ret =  of_property_read_u32(np, gpio_name, &hall_sensor_dev->gpio_list[i].gpio_high_report_val);
		sprintf(gpio_name, "hall,nirq-gpio-low-val_%d", i);
		ret =  of_property_read_u32(np, gpio_name, &hall_sensor_dev->gpio_list[i].gpio_low_report_val);
		sprintf(gpio_name, "hall,nirq-gpio_%d", i);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
		hall_sensor_dev->gpio_list[i].gpio = of_get_named_gpio_flags(np, gpio_name, 0, &flags);
#else
        hall_sensor_dev->gpio_list[i].gpio = of_get_named_gpio(np, gpio_name, 0);
#endif
		LOG_INFO("hall_sensor %s gpio = %d val = %d:%d\r\n", gpio_name, hall_sensor_dev->gpio_list[i].gpio,
						hall_sensor_dev->gpio_list[i].gpio_high_report_val,
						hall_sensor_dev->gpio_list[i].gpio_low_report_val);
		if (gpio_is_valid(hall_sensor_dev->gpio_list[i].gpio))
		{
			ret = gpio_request(hall_sensor_dev->gpio_list[i].gpio, gpio_name);
			if (ret) {
				LOG_ERR("Could not request %s : %d for hall sensor, ret: %d\r\n",
					gpio_name, hall_sensor_dev->gpio_list[i].gpio, ret);
				goto fail_for_irq;
			}
			gpio_direction_input(hall_sensor_dev->gpio_list[i].gpio);

			//set irq
			hall_sensor_dev->gpio_list[i].irq = gpio_to_irq(hall_sensor_dev->gpio_list[i].gpio);
			LOG_INFO("hall_sensor %s irq = %d, gpio = %d\r\n", gpio_name, hall_sensor_dev->gpio_list[i].irq, hall_sensor_dev->gpio_list[i].gpio);

			ret = request_irq(hall_sensor_dev->gpio_list[i].irq, hall_sensor_interrupt_handler,
					IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, gpio_name, hall_sensor_dev);
			if (ret) {
				LOG_ERR("Could not register for hall sensor interrupt, irq = %d, ret: %d\r\n",
						hall_sensor_dev->gpio_list[i].irq, ret);
				hall_sensor_dev->gpio_list[i].irq = 0;
				goto fail_for_irq;
			}
			disable_irq(hall_sensor_dev->gpio_list[i].irq);
		}
	}

	//enable vdd
	hall_sensor_dev->hall_vdd = regulator_get(&pdev->dev, "hall_vdd");
	if (IS_ERR(hall_sensor_dev->hall_vdd))
	{
		LOG_ERR("vdd error %ld\r\n", PTR_ERR(hall_sensor_dev->hall_vdd));
		hall_sensor_dev->hall_vdd = NULL;
		/* goto fail_for_irq; */
	}
	else
	{
		rc = regulator_enable(hall_sensor_dev->hall_vdd);
		LOG_INFO("hall_vdd regulator is %s, ret %d\r\n",
				regulator_is_enabled(hall_sensor_dev->hall_vdd) ?
				"on" : "off", rc);
	}

	//schedule a 1s delayed work to inform other drivers with initial state.
	//suppose 1s is enough for other drivers to get ready and register callback
	//to this notifier chain.
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	hall_sensor_irq_wq = create_singlethread_workqueue("hall_sensor_irq_wq");
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_work, hall_sensor_work_function);
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_irq_work, hall_sensor_irq_work_function);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, HZ);

	LOG_INFO("hall_sensor_probe Done\r\n");
	return 0;

	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;

fail_for_irq:
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		if (hall_sensor_dev->gpio_list[i].irq)
			free_irq(hall_sensor_dev->gpio_list[i].irq, hall_sensor_dev);
		if (gpio_is_valid(hall_sensor_dev->gpio_list[i].gpio))
			gpio_free(hall_sensor_dev->gpio_list[i].gpio);
	}
	class_unregister(&hall_class);
fail_for_mem:
	if (hall_sensor_dev->gpio_list)
		kfree(hall_sensor_dev->gpio_list);
	if (hall_sensor_dev)
		kfree(hall_sensor_dev);
	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	int i;

	/* disable irqs */
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		if (hall_sensor_dev->gpio_list[i].irq) {
			disable_irq(hall_sensor_dev->gpio_list[i].irq);
			LOG_INFO("disable hall irq:%d\r\n", hall_sensor_dev->gpio_list[i].irq);
		}
	}

	LOG_INFO("clean up hall workqueue\r\n");
	/* wait for all works being finished */
	cancel_delayed_work_sync(&hall_sensor_dev->hall_sensor_irq_work);
	cancel_delayed_work_sync(&hall_sensor_dev->hall_sensor_work);
	/* destroy work queue */
	destroy_workqueue(hall_sensor_irq_wq);
	destroy_workqueue(hall_sensor_wq);

	LOG_INFO("release hall vdd\r\n");
	if (hall_sensor_dev->hall_vdd) {
		regulator_disable(hall_sensor_dev->hall_vdd);
		regulator_put(hall_sensor_dev->hall_vdd);
	}

	LOG_INFO("free hall irq and gpio\r\n");
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		if (hall_sensor_dev->gpio_list[i].irq)
			free_irq(hall_sensor_dev->gpio_list[i].irq, hall_sensor_dev);
		if (gpio_is_valid(hall_sensor_dev->gpio_list[i].gpio))
			gpio_free(hall_sensor_dev->gpio_list[i].gpio);
	}
	LOG_INFO("free hall gpio list\r\n");
	if (hall_sensor_dev->gpio_list)
		kfree(hall_sensor_dev->gpio_list);

	LOG_INFO("free hall sensor classdev\r\n");
	sensors_classdev_unregister(&hall_sensor_dev->sensors_phone_case_cdev);

	LOG_INFO("free hall intput device\r\n");
	input_unregister_device(hall_sensor_dev->hall_dev);

	LOG_INFO("unregister hall class\r\n");
	class_unregister(&hall_class);

	LOG_INFO("free hall_sensor_dev\r\n");
	if (hall_sensor_dev)
		kfree(hall_sensor_dev);

	LOG_INFO("paltform rm\r\n");
	return 0;
}

module_platform_driver(hall_phone_case_driver);

MODULE_DESCRIPTION("Hall_sensor_phone_case Driver");
MODULE_AUTHOR("guoyc1 <guoyc1@motorola.com>");
MODULE_LICENSE("GPL v2");
