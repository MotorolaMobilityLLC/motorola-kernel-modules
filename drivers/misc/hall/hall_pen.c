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
#ifdef CONFIG_HALL_PASSIVE_PEN
#include <linux/pen_detection_notify.h>
#endif

#define DRIVER_NAME "hall_pen_detect"
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
	int pen_detect;
	struct regulator *hall_vdd;
	struct hall_gpio *gpio_list;
	spinlock_t mHallSensorLock;
	struct input_dev *hall_dev;
	struct sensors_classdev sensors_pen_cdev;
#ifdef CONFIG_HALL_PASSIVE_PEN
	struct delayed_work hall_sensor_work;
	struct delayed_work hall_sensor_dowork;
	struct input_handler pen_handler;
	struct input_handle pen_handle;
#endif
}* hall_sensor_dev;

#ifdef CONFIG_OF
static const struct of_device_id hall_pen_match[] = {
	{ .compatible = "hall_pen_detect", },
	{}
};
#else
#define hall_pen_match NULL
#endif
MODULE_DEVICE_TABLE(of, hall_pen_match);

static struct platform_driver hall_pen_driver = {
	.probe		= hall_sensor_probe,
	.remove		= hall_sensor_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table    = hall_pen_match,
	},
};

#ifdef CONFIG_HALL_PASSIVE_PEN
static struct workqueue_struct *hall_sensor_wq;
static struct workqueue_struct *hall_sensor_do_wq;
static BLOCKING_NOTIFIER_HEAD(pen_detection_notifier_list);

/**
 * pen_detection_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to pen_detection_notifier_list, which would be called when
 * the passive pen is inserted or pulled out.
 */
int pen_detection_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pen_detection_notifier_list,
						nb);
}
EXPORT_SYMBOL(pen_detection_register_client);

/**
 * pen_detection_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * pen_detection_notifier_list.
 */
int pen_detection_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pen_detection_notifier_list,
						  nb);
}
EXPORT_SYMBOL(pen_detection_unregister_client);

/**
 * pen_detection_notifier_call_chain - notify clients of pen_detection_events
 * @val: event PEN_DETECTION_INSERT or PEN_DETECTION_PULL
 * @v: notifier data, inculde display pen detection event.
 */
static int pen_detection_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&pen_detection_notifier_list, val,
					    v);
}

static void pen_do_work_function(struct work_struct *dat)
{
	LOG_INFO("[%s] hall_sensor_interrupt = %d\n", DRIVER_NAME,hall_sensor_dev->pen_detect);
	cancel_delayed_work(&hall_sensor_dev->hall_sensor_work);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
}

static void pen_report_function(struct work_struct *dat)
{
	int i;
	unsigned long flags;
	int report_val = 0;
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) > 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_high_report_val;
		else if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) == 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_low_report_val;
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	if(hall_sensor_dev->report_val != report_val){
		LOG_INFO("hall report %d", report_val);
		hall_sensor_dev->report_val = report_val;
		if(hall_sensor_dev->report_val)
			hall_sensor_dev->status = 0;
		else
			hall_sensor_dev->status = 1;
		input_report_switch(hall_sensor_dev->hall_dev, SW_PEN_INSERTED, !hall_sensor_dev->status);
		input_sync(hall_sensor_dev->hall_dev);
		if(hall_sensor_dev->report_val)
			pen_detection_notifier_call_chain(PEN_DETECTION_INSERT, NULL);
		else
			pen_detection_notifier_call_chain(PEN_DETECTION_PULL, NULL);
	}
}
#endif

void check_and_send(void)
{
	int i;
	int report_val = 0;
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		LOG_DBG("hall report gpio%d = %d", i, gpio_get_value(hall_sensor_dev->gpio_list[i].gpio));
		if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) > 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_high_report_val;
		else if (gpio_get_value(hall_sensor_dev->gpio_list[i].gpio) == 0)
			report_val |= hall_sensor_dev->gpio_list[i].gpio_low_report_val;
	}
	if(hall_sensor_dev->pen_detect &&
		hall_sensor_dev->report_val != report_val){
		LOG_INFO("hall report %d", report_val);
		hall_sensor_dev->report_val = report_val;
		input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, report_val);
		input_sync(hall_sensor_dev->hall_dev);
	}
}

void hall_enable(bool enable)
{
	int i;
	if (enable && !hall_sensor_dev->enable)
	{
		LOG_INFO("hall_sensor enable");
		for (i = 0; i < hall_sensor_dev->gpio_num; i++)
		{
			enable_irq(hall_sensor_dev->gpio_list[i].irq);
			enable_irq_wake(hall_sensor_dev->gpio_list[i].irq);
		}
		hall_sensor_dev->enable = 1;
		check_and_send();
	}
	else if (!enable && hall_sensor_dev->enable)
	{
		LOG_INFO("hall_sensor disable");
		for (i = 0; i < hall_sensor_dev->gpio_num; i++)
		{
			disable_irq(hall_sensor_dev->gpio_list[i].irq);
			disable_irq_wake(hall_sensor_dev->gpio_list[i].irq);
		}
		hall_sensor_dev->enable = 0;
	}
}

static int hallpen_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	hall_sensor_dev->report_val = -1;
#ifndef CONFIG_HALL_PASSIVE_PEN
	hall_sensor_dev->pen_detect = enable;
	hall_enable(enable);
	if (enable == 0)
	{
		input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1);
		input_sync(hall_sensor_dev->hall_dev);
	}
#endif
	return 0;
}
static ssize_t hall_enable_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
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

static ssize_t hall_rawdata_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", hall_sensor_dev->report_val);
}

static struct class_attribute class_attr_enable =
__ATTR(enable, 0660, NULL, hall_enable_store);
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
	.owner                  = THIS_MODULE,
	.class_groups           = hall_class_groups,
};

#ifdef CONFIG_HALL_PASSIVE_PEN
static struct input_device_id mID[] = {
	{ .driver_info = 1 },		//scan all device to match hall sensor
	{ },
};
int pen_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id){
	LOG_INFO("[%s] hall_sensor connect to handler\n", DRIVER_NAME);
	return 0;
}

void pen_event(struct input_handle *handle, unsigned int type, unsigned int code, int value){
	if(type==EV_SW && code==SW_PEN_INSERTED ){
		if(value != 2 && !!test_bit(code, hall_sensor_dev->hall_dev->sw) != !hall_sensor_dev->status){
			__change_bit(code,  hall_sensor_dev->hall_dev->sw);
			LOG_INFO("[%s] reset dev->sw=%d \n", DRIVER_NAME,!hall_sensor_dev->status);
		}
	}
}

bool pen_match(struct input_handler *handler, struct input_dev *dev){
	if( (dev->name && handler->name) &&
		!strcmp(dev->name,"pen_input") &&
		!strcmp(handler->name,"pen_input_handler"))
		    return true;

	return false;
}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	LOG_DBG("hall_sensor_interrupt_handler = %d", irq);
	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}
#else
static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	unsigned long flags;
	spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	LOG_DBG("hall_sensor_interrupt_handler = %d", irq);
	check_and_send();
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	return IRQ_HANDLED;
}
#endif

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	enum of_gpio_flags flags;
	struct device_node *np = pdev->dev.of_node;
	int i;
	const char *name_temp;
	//struct totalHallInformation *pHallInformationData = NULL;

	//Memory allocation
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		LOG_ERR("Memory allocation fails for hall sensor");
		ret = -ENOMEM;
		goto fail_for_mem;
	}

	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
#ifdef CONFIG_HALL_PASSIVE_PEN
	hall_sensor_dev->enable = 1;
	hall_sensor_dev->pen_detect = 1;
#else
	hall_sensor_dev->enable = 0;
	hall_sensor_dev->pen_detect = 0;
#endif
	//tcmd node
	ret = of_property_read_string(np, "hall,factory-class-name", &hall_class.name);
	ret = class_register(&hall_class);
	ret = of_property_read_string(np, "hall,input-dev-name", &name_temp);
     	LOG_INFO("hall_input num :%s", name_temp);

	ret =  of_property_read_u32(np,"hall,nirq-gpio-num", &hall_sensor_dev->gpio_num);
	if (ret < 0)
	{
		LOG_ERR("GPIO for hall sensor does not exist");
	}
	LOG_INFO("gpio num :%d", hall_sensor_dev->gpio_num);

	  hall_sensor_dev->hall_dev = input_allocate_device();
	  	if (!hall_sensor_dev->hall_dev){
			LOG_ERR(" hall_indev allocation fails\n" );
			return -ENOMEM;
		}
	hall_sensor_dev->hall_dev->name = name_temp;
	hall_sensor_dev->hall_dev->dev.parent= NULL;
	LOG_INFO("hall_ num :%s", hall_sensor_dev->hall_dev->name);
#ifdef CONFIG_HALL_PASSIVE_PEN
	/* Set all the keycodes */
	hall_sensor_dev->hall_dev->phys= "/dev/input/pen_detect";
	input_set_capability(hall_sensor_dev->hall_dev, EV_SW, SW_PEN_INSERTED);
#else
	__set_bit(EV_ABS, hall_sensor_dev->hall_dev->evbit);
	input_set_abs_params(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1, 100, 0, 0);
#endif

	ret = input_register_device(hall_sensor_dev->hall_dev);
	if (ret) {
		LOG_ERR("[%s] input registration fails\n", DRIVER_NAME);
		ret = -1;
		goto exit_input_free;
	}
#ifdef CONFIG_HALL_PASSIVE_PEN
	hall_sensor_dev->pen_handler.match=pen_match;
	hall_sensor_dev->pen_handler.connect=pen_connect;
	hall_sensor_dev->pen_handler.event=pen_event;
	hall_sensor_dev->pen_handler.name="pen_detect_handler";
	hall_sensor_dev->pen_handler.id_table = mID;
	ret = input_register_handler(& hall_sensor_dev->pen_handler);
	if(ret){
		LOG_ERR("[%s] handler registration fails\n", DRIVER_NAME);
		ret = -1;
		goto exit_unregister_input_dev;
	}
	hall_sensor_dev->pen_handle.name="pen_detect_handle";
	hall_sensor_dev->pen_handle.open=1;         //receive any event from hall sensor
	hall_sensor_dev->pen_handle.dev=hall_sensor_dev->hall_dev;
	hall_sensor_dev->pen_handle.handler=&hall_sensor_dev->pen_handler;
	ret = input_register_handle(& hall_sensor_dev->pen_handle);
	if(ret){
		LOG_ERR("[%s] handle registration fails\n", DRIVER_NAME);
		ret = -1;
		goto exit_unregister_handler;
	}
#else
	input_report_abs(hall_sensor_dev->hall_dev, ABS_DISTANCE, -1);
	input_sync(hall_sensor_dev->hall_dev);
#endif

	hall_sensor_dev->sensors_pen_cdev.sensors_enable = hallpen_enable;
	hall_sensor_dev->sensors_pen_cdev.sensors_poll_delay = NULL;
	hall_sensor_dev->sensors_pen_cdev.name = hall_sensor_dev->hall_dev->name;
	hall_sensor_dev->sensors_pen_cdev.vendor = "motorola";
	hall_sensor_dev->sensors_pen_cdev.version = 1;
	hall_sensor_dev->sensors_pen_cdev.type = SENSOR_TYPE_MOTO_HALL;
	hall_sensor_dev->sensors_pen_cdev.max_range = "5";
	hall_sensor_dev->sensors_pen_cdev.resolution = "5.0";
	hall_sensor_dev->sensors_pen_cdev.sensor_power = "0.005";
	hall_sensor_dev->sensors_pen_cdev.min_delay = 0;
	hall_sensor_dev->sensors_pen_cdev.fifo_reserved_event_count = 0;
	hall_sensor_dev->sensors_pen_cdev.fifo_max_event_count = 0;
	hall_sensor_dev->sensors_pen_cdev.delay_msec = 100;
	hall_sensor_dev->sensors_pen_cdev.enabled = 0;

	ret = sensors_classdev_register(&hall_sensor_dev->hall_dev->dev, &hall_sensor_dev->sensors_pen_cdev);
	if (ret < 0)
		LOG_ERR("create cap sensor_class  file failed (%d)\n", ret);

	hall_sensor_dev->gpio_list = kzalloc(sizeof (struct hall_gpio) * hall_sensor_dev->gpio_num, GFP_KERNEL);
	if (!hall_sensor_dev->gpio_list) {
		LOG_ERR("Memory allocation fails for hall sensor");
		ret = -ENOMEM;
		goto fail_for_mem;
	}

#ifdef CONFIG_HALL_PASSIVE_PEN
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	hall_sensor_do_wq = create_singlethread_workqueue("hall_sensor_do_wq");
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_work, pen_report_function);
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_dowork, pen_do_work_function);
	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, 0);
#endif

	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		char *gpio_name = hall_sensor_dev->gpio_list[i].gpio_name;
		sprintf(gpio_name, "hall,nirq-gpio-high-val_%d", i);
		ret =  of_property_read_u32(np, gpio_name, &hall_sensor_dev->gpio_list[i].gpio_high_report_val);
		sprintf(gpio_name, "hall,nirq-gpio-low-val_%d", i);
		ret =  of_property_read_u32(np, gpio_name, &hall_sensor_dev->gpio_list[i].gpio_low_report_val);
		sprintf(gpio_name, "hall,nirq-gpio_%d", i);
		hall_sensor_dev->gpio_list[i].gpio = of_get_named_gpio_flags(np, gpio_name, 0, &flags);
		LOG_INFO("hall_sensor %s gpio = %d val = %d:%d", gpio_name, hall_sensor_dev->gpio_list[i].gpio,
						hall_sensor_dev->gpio_list[i].gpio_high_report_val,
						hall_sensor_dev->gpio_list[i].gpio_low_report_val);
		if (gpio_is_valid(hall_sensor_dev->gpio_list[i].gpio))
		{
			ret = gpio_request(hall_sensor_dev->gpio_list[i].gpio, gpio_name);
			if (ret) {
				LOG_ERR("Could not request %s : %d for hall sensor, ret: %d",
					gpio_name, hall_sensor_dev->gpio_list[i].gpio, ret);
				goto fail_for_irq;
			}
			gpio_direction_input(hall_sensor_dev->gpio_list[i].gpio);

			//set irq
			hall_sensor_dev->gpio_list[i].irq = gpio_to_irq(hall_sensor_dev->gpio_list[i].gpio);
			LOG_INFO("hall_sensor %s irq = %d, gpio = %d", gpio_name, hall_sensor_dev->gpio_list[i].irq, hall_sensor_dev->gpio_list[i].gpio);

			ret = request_threaded_irq(hall_sensor_dev->gpio_list[i].irq, NULL, hall_sensor_interrupt_handler,
					IRQF_ONESHOT|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, gpio_name, hall_sensor_dev);
			if (ret) {
				LOG_ERR("Could not register for hall sensor interrupt, irq = %d, ret: %d",
						hall_sensor_dev->gpio_list[i].irq, ret);
				hall_sensor_dev->gpio_list[i].irq = 0;
				goto fail_for_irq;
			}
#ifndef CONFIG_HALL_PASSIVE_PEN
			disable_irq(hall_sensor_dev->gpio_list[i].irq);
#endif
		}
	}

	//enable vdd
	hall_sensor_dev->hall_vdd = regulator_get(&pdev->dev, "hall_vdd");
	if (IS_ERR(hall_sensor_dev->hall_vdd))
	{
		LOG_ERR("vdd error %ld", PTR_ERR(hall_sensor_dev->hall_vdd));
		goto fail_for_irq;
	}
	else
	{
		regulator_enable(hall_sensor_dev->hall_vdd);
		LOG_INFO("hall_vdd regulator is %s",
				regulator_is_enabled(hall_sensor_dev->hall_vdd) ?
				"on" : "off");
	}
	LOG_INFO("hall_sensor_probe Done");
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
#ifdef CONFIG_HALL_PASSIVE_PEN
exit_unregister_handler:
	input_unregister_handler(& hall_sensor_dev->pen_handler);
exit_unregister_input_dev:
	input_unregister_device(hall_sensor_dev->hall_dev);
#endif
exit_input_free:
	input_free_device(hall_sensor_dev->hall_dev);
	hall_sensor_dev->hall_dev = NULL;
	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	int i;
	regulator_disable(hall_sensor_dev->hall_vdd);
	regulator_put(hall_sensor_dev->hall_vdd);
	class_unregister(&hall_class);
	for (i = 0; i < hall_sensor_dev->gpio_num; i++)
	{
		if (hall_sensor_dev->gpio_list[i].irq)
			free_irq(hall_sensor_dev->gpio_list[i].irq, hall_sensor_dev);
		if (gpio_is_valid(hall_sensor_dev->gpio_list[i].gpio))
			gpio_free(hall_sensor_dev->gpio_list[i].gpio);
	}
	if (hall_sensor_dev->gpio_list)
		kfree(hall_sensor_dev->gpio_list);
	if (hall_sensor_dev)
		kfree(hall_sensor_dev);

	sensors_classdev_unregister(&hall_sensor_dev->sensors_pen_cdev);
	input_unregister_device(hall_sensor_dev->hall_dev);
	LOG_INFO("paltform rm");
	return 0;
}

module_platform_driver(hall_pen_driver);

MODULE_DESCRIPTION("Hall_sensor_pen Driver");
MODULE_AUTHOR("cuijy1 <cuijy1@motorola.com>");
MODULE_LICENSE("GPL v2");
