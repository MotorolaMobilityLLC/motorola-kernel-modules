#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#ifndef STK_SPREADTRUM
    #include <linux/sensors.h>
#endif // STK_SPREADTRUM
#include <linux/gpio.h>
#include <asm/uaccess.h>
#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif
#include "stk501xx.h"
#include "stk501xx_qualcomm.h"
//#include "base.h"

struct stk_data *global_stk;
struct attribute_group stk_attribute_sar_group;

static void stk_report_sar_state(struct stk_data* stk, int8_t nf_flag)
{
    stk501xx_wrapper *stk_wrapper = container_of(stk, stk501xx_wrapper, stk);
    int8_t i = 0;

    if (!stk_wrapper->channels[i].input_dev)
    {
        STK_ERR("No input device for sar data");
        return;
    }

    for (i = 0; i < ch_num; i ++)
    {
        input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, nf_flag);
        input_sync(stk_wrapper->channels[i].input_dev);
        STK_ERR("Always report FAR (%d)\n", nf_flag);
    }
}

/**
 * @brief: Get power status
 *          Send 0 or 1 to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    char en;
    en = stk->enabled;
    return scnprintf(buf, PAGE_SIZE, "enable = %d\n", en);
}

/**
 * @brief: Set power status
 *          Get 0 or 1 from userspace, then set stk8xxx power status.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    unsigned int data;
    int error;

    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        STK_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    STK_ERR("stk_enable_store, data=%d", data);

    if ((1 == data) || (0 == data))
    {
        stk501xx_set_enable(stk, data, false);
        if(1 == data)
            stk_report_sar_state(stk, 0);
        else
            stk_report_sar_state(stk, -1);
    }
    else
    {
        STK_ERR("invalid argument, en=%d", data);
    }

    return count;
}

/**
 * @brief: Get sar data
 *          Send sar data to userspce.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_value_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    int i = 0;
    uint32_t prox_flag = 0;
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    STK_ERR("stk_value_show");
    //read prox flag
    stk_read_prox_flag(stk, &prox_flag);
    stk501xx_read_sar_data(stk, prox_flag, stk->dist_phase);

    for (i = 0; i < 8; i++)
    {
        scnprintf(buf, PAGE_SIZE, "ph[%d] value=%d\n", i, stk->last_data[i]);
        STK_ERR("ph[%d] value=%d\n", i, stk->last_data[i]);
    }

    return 0;
}

static ssize_t stk_flag_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    int i = 0;
    uint32_t prox_flag = 0;
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    STK_ERR("stk_flag_show");
    //read prox flag
    stk_read_prox_flag(stk, &prox_flag);
    stk501xx_read_sar_data(stk, prox_flag, stk->dist_phase);

    for ( i = 0; i < 8; i++)
    {
        STK_ERR("ph[%d] prox flag=%d", i, stk->last_nearby[i]);
    }

    return scnprintf(buf, PAGE_SIZE, "flag=0x%x\n", prox_flag);
}

/**
 * @brief: Register writting
 *          Get address and content from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_send_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    char *token[10];
    int err, i;
    u32 addr, cmd;
    bool enable = false;

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    err = kstrtouint(token[0], 16, &addr);

    if (err)
    {
        STK_ERR("kstrtouint failed, err=%d", err);
        return err;
    }

    err = kstrtouint(token[1], 16, &cmd);

    if (err)
    {
        STK_ERR("kstrtouint failed, err=%d", err);
        return err;
    }

    STK_ERR("write reg[0x%X]=0x%X", addr, cmd);

    if (!stk->enabled)
        stk501xx_set_enable(stk, 1, true);
    else
        enable = true;

    if (STK_REG_WRITE_BLOCK(stk, (u16)addr, (u8*)&cmd, 4))
    {
        err = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk501xx_set_enable(stk, 0, true);

    if (err)
        return -1;

    return count;
}

static ssize_t stk_temp_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    STK_ERR("stk_temp_show");
    stk501xx_read_temp_data(stk, STK_ADDR_REG_RAW_PH0_REG, &stk->prev_temperature_ref_a);
    return scnprintf(buf, PAGE_SIZE, "temperature=%d\n", stk->prev_temperature_ref_a);
}

/**
 * @brief: Read all register value, then send result to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_allreg_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    int result;
    result = stk501xx_show_all_reg(stk);

    if (0 > result)
        return result;

    return (ssize_t)result;
}

/**
 * @brief: Check PID, then send chip number to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_chipinfo_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    STK_ERR("chip id=0x%x, index=0x%x", stk->chip_id, stk->chip_index);
    return scnprintf(buf, PAGE_SIZE, "pid=0x%x,index=0x%x\n", stk->chip_id, stk->chip_index);
}

static ssize_t stk_phase_cali(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    int result = 0;
    stk_data *stk = &stk_wrapper->stk;
    stk501xx_phase_reset(stk, STK_TRIGGER_REG_INIT_ALL);
    return (ssize_t)result;
}

static DEVICE_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DEVICE_ATTR(value, 0444, stk_value_show, NULL);
static DEVICE_ATTR(send, 0220, NULL, stk_send_store);
static DEVICE_ATTR(temp, 0444, stk_temp_show, NULL);
static DEVICE_ATTR(flag, 0444, stk_flag_show, NULL);
static DEVICE_ATTR(allreg, 0444, stk_allreg_show, NULL);
static DEVICE_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DEVICE_ATTR(phcali, 0444, stk_phase_cali, NULL);

static struct attribute *stk_attribute_sar[] =
{
    &dev_attr_enable.attr,
    &dev_attr_value.attr,
    &dev_attr_send.attr,
    &dev_attr_temp.attr,
    &dev_attr_flag.attr,
    &dev_attr_allreg.attr,
    &dev_attr_chipinfo.attr,
    &dev_attr_phcali.attr,
    NULL
};

struct attribute_group stk_attribute_sar_group =
{
    .name = STK501XX_NAME,
    .attrs = stk_attribute_sar,
};

/*class define */
static ssize_t class_stk_enable_show(struct class *class,
                                     struct class_attribute *attr, char *buf)
{
    char en;
    en = global_stk->enabled;
    return scnprintf(buf, PAGE_SIZE, "enable = %d\n", en);
}

static ssize_t class_stk_enable_store(struct class *class,
                                      struct class_attribute *attr, const char *buf, size_t count)
{
    unsigned int data;
    int error;

    error = kstrtouint(buf, 10, &data);

    STK_ERR("stk_enable_store, data=%d, cnt %lu, -%c-", data, count, buf[0]);
    if (error)
    {
        STK_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    STK_ERR("stk_enable_store, data=%d", data);

    if ((1 == data) || (0 == data))
    {
        stk501xx_set_enable(global_stk, data, false);
        if(1 == data)
            stk_report_sar_state(global_stk, 0);
        else
            stk_report_sar_state(global_stk, -1);
    }
    else
    {
        STK_ERR("invalid argument, en=%d", data);
    }

    return count;
}

static ssize_t class_stk_value_show(struct class *class,
                                    struct class_attribute *attr, char *buf)
{
    int i = 0;
    uint32_t prox_flag = 0;
    STK_ERR("stk_value_show");
    //read prox flag
    stk_read_prox_flag(global_stk, &prox_flag);
    stk501xx_read_sar_data(global_stk, prox_flag, global_stk->dist_phase);

    for (i = 0; i < 8; i++)
    {
        scnprintf(buf, PAGE_SIZE, "ph[%d] value=%d\n", i, global_stk->last_data[i]);
        STK_ERR("ph[%d] value=%d\n", i, global_stk->last_data[i]);
    }

    return 0;
}

static ssize_t class_stk_flag_show(struct class *class,
                                   struct class_attribute *attr, char *buf)
{
    int i = 0;
    uint32_t prox_flag = 0;
    STK_ERR("stk_flag_show");
    //read prox flag
    stk_read_prox_flag(global_stk, &prox_flag);
    stk501xx_read_sar_data(global_stk, prox_flag, global_stk->dist_phase);

    for ( i = 0; i < 8; i++)
    {
        STK_ERR("ph[%d] prox flag=%d\n", i, global_stk->last_nearby[i]);
    }

    return scnprintf(buf, PAGE_SIZE, "flag=0x%d\n", prox_flag);
}

static ssize_t class_stk_send_store(struct class *class,
                                    struct class_attribute *attr, const char *buf, size_t count)
{
    char *token[10];
    int err, i;
    u32 addr, cmd;
    bool enable = false;

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    err = kstrtouint(token[0], 16, &addr);

    if (err)
    {
        STK_ERR("kstrtouint failed, err=%d", err);
        return err;
    }

    err = kstrtouint(token[1], 16, &cmd);

    if (err)
    {
        STK_ERR("kstrtouint failed, err=%d", err);
        return err;
    }

    STK_ERR("write reg[0x%X]=0x%X", addr, cmd);

    if (!global_stk->enabled)
        stk501xx_set_enable(global_stk, 1, true);
    else
        enable = true;

    if (STK_REG_WRITE_BLOCK(global_stk, (u16)addr, (u8*)&cmd, 4))
    {
        err = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk501xx_set_enable(global_stk, 0, true);

    if (err)
        return -1;

    return count;
}

static ssize_t class_stk_temp_show(struct class *class,
                                   struct class_attribute *attr, char *buf)
{
    STK_ERR("stk_temp_show");
    stk501xx_read_temp_data(global_stk, STK_ADDR_REG_RAW_PH0_REG, &global_stk->prev_temperature_ref_a);
    return scnprintf(buf, PAGE_SIZE, "temperature=%d\n", global_stk->prev_temperature_ref_a);
}

static ssize_t class_stk_allreg_show(struct class *class,
                                     struct class_attribute *attr, char *buf)
{
    int result;
    result = stk501xx_show_all_reg(global_stk);

    if (0 > result)
        return result;

    return (ssize_t)result;
}

static ssize_t class_stk_chipinfo_show(struct class *class,
                                       struct class_attribute *attr, char *buf)
{
    STK_ERR("chip id=0x%x, index=0x%x", global_stk->chip_id, global_stk->chip_index);
    return scnprintf(buf, PAGE_SIZE, "pid=0x%x,index=0x%x\n", global_stk->chip_id, global_stk->chip_index);
}

static ssize_t class_stk_phase_cali(struct class *class,
                                    struct class_attribute *attr, char *buf)
{
    int result = 0;
    STK_ERR("class_stk_phase_cali , reset all phase\n");
    stk501xx_phase_reset(global_stk, STK_TRIGGER_REG_INIT_ALL);
    return (ssize_t)result;
}

static ssize_t class_stk_phase_cali_store(struct class *class,
        struct class_attribute *attr, const char *buf, size_t count)
{
    STK_ERR("class_stk_phase_cali_store , reset all phase\n");
    stk501xx_phase_reset(global_stk, STK_TRIGGER_REG_INIT_ALL);
    return count;
}

static ssize_t class_stk_int_state_show(struct class *class,
                struct class_attribute *attr, char *buf)
{
    STK_ERR("int state %d", global_stk->intrrupt_init_state);
    return snprintf(buf, 8, "%d\n", global_stk->intrrupt_init_state);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static struct class_attribute class_attr_enable =
    __ATTR(enable, 0664, class_stk_enable_show, class_stk_enable_store);
static struct class_attribute class_attr_value =
    __ATTR(value, 0444, class_stk_value_show, NULL);
static struct class_attribute class_attr_send =
    __ATTR(send, 0220, NULL, class_stk_send_store);
static struct class_attribute class_attr_temp =
    __ATTR(temp, 0444, class_stk_temp_show, NULL);
static struct class_attribute class_attr_flag =
    __ATTR(flag, 0444, class_stk_flag_show, NULL);
static struct class_attribute class_attr_allreg =
    __ATTR(allreg, 0444, class_stk_allreg_show, NULL);
static struct class_attribute class_attr_chipinfo =
    __ATTR(chipinfo, 0444, class_stk_chipinfo_show, NULL);
static struct class_attribute class_attr_reset =
    __ATTR(reset, 0664, class_stk_phase_cali, class_stk_phase_cali_store);
static struct class_attribute class_attr_int_state =
    __ATTR(int_state, 0444, class_stk_int_state_show, NULL);

static struct attribute *capsense_class_attrs[] =
{
    &class_attr_enable.attr,
    &class_attr_value.attr,
    &class_attr_send.attr,
    &class_attr_temp.attr,
    &class_attr_flag.attr,
    &class_attr_allreg.attr,
    &class_attr_chipinfo.attr,
    &class_attr_reset.attr,
    &class_attr_int_state.attr,
    NULL,
};

ATTRIBUTE_GROUPS(capsense_class);
#else
static struct class_attribute capsense_class_attributes[] =
{
    __ATTR(enable, 0664, class_stk_enable_show, class_stk_enable_store),
    __ATTR(value, 0444, class_stk_value_show, NULL),
    __ATTR(send, 0220, NULL, class_stk_send_store),
    __ATTR(temp, 0444, class_stk_temp_show, NULL),
    __ATTR(flag, 0444, class_stk_flag_show, NULL),
    __ATTR(allreg, 0444, class_stk_allreg_show, NULL),
    __ATTR(chipinfo, 0444, class_stk_chipinfo_show, NULL),
    __ATTR(reset, 0664, class_stk_phase_cali, class_stk_phase_cali_store),
    __ATTR(int_state, 0444, class_stk_int_state_show, NULL),
    __ATTR_NULL,
};
#endif
struct class capsense_class =
    {
        .name                   = "capsense",
        .owner                  = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        .class_groups           = capsense_class_groups,
#else
        .class_attrs            = capsense_class_attributes,
#endif
    };
/*end of class define*/
static struct stk501xx_platform_data stk_plat_data =
{
    .direction              = 1,
    .interrupt_int1_pin     = 117,
};

#ifdef STK_SENSORS_DEV
/* SAR information read by HAL */
static struct sensors_classdev stk_cdev =
{
    .name = "stk501xx",
    .vendor = "Sensortek",
    .version = 1,
    .type = SENSOR_TYPE_MOTO_CAPSENSE,
    .max_range = "5",
    .resolution = "5.0",
    .sensor_power = "3",
    .min_delay = 0,
    .max_delay = 0,
    .delay_msec = 16,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .max_latency = 0,
    .flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_enable_wakeup = NULL,
    .sensors_set_latency = NULL,
    .sensors_flush = NULL,
    .sensors_calibrate = NULL,
    .sensors_write_cal_params = NULL,
};

/*
 * @brief: The handle for enable and disable sensor.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] enabled:
 */
static int stk_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
                                   unsigned int enabled)
{
    //struct stk501xx_wrapper *stk_wrapper = container_of(sensors_cdev, stk501xx_wrapper, channels[0].sar_cdev);
    //struct stk_data *stk = &stk_wrapper->stk;
    struct stk_data *stk = global_stk;
     STK_ERR("stk_cdev_sensors_enable , en=%d\n", enabled);
    if (0 == enabled)
    {
        stk501xx_set_enable(stk, 0, false);
        stk_report_sar_state(stk, -1);
    }
    else if (1 == enabled)
    {
        stk501xx_set_enable(stk, 1, false);
        stk_report_sar_state(stk, 0);
    }
    else
    {
        STK_ERR("Invalid vlaue of input, input=%d", enabled);
        return -EINVAL;
    }

    return 0;
}

/*
 * @brief: The handle for set the sensor polling delay time.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] delay_msec:
 */
#if 0
static int stk_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
                                       unsigned int delay_msec)
{
#ifdef STK_INTERRUPT_MODE
    /* do nothing */
#elif defined STK_POLLING_MODE
    struct stk501xx_wrapper *stk_wrapper = container_of(sensors_cdev, stk501xx_wrapper, channels[0].sar_cdev);
    struct stk_data *stk = &stk_wrapper->stk;
    stk->stk_timer_info.interval_time = delay_msec * 1000;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    STK_LOG("stk_cdev_sensors_poll_delay ms=%d", delay_msec);
    return 0;
}

/*
 * @brief:
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] enable:
 */
static int stk_cdev_sensors_enable_wakeup(struct sensors_classdev *sensors_cdev,
        unsigned int enable)
{
    STK_LOG("enable=%d", enable);
    return 0;
}
#endif


/*
 * @brief: Flush sensor events in FIFO and report it to user space.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 */
static int stk_cdev_sensors_flush(struct sensors_classdev *sensors_cdev)
{
    STK_LOG("stk_cdev_sensors_flush");
    return 0;
}
#endif // STK_SENSORS_DEV

/*
 * @brief: File system setup for accel and any motion
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_input_setup(stk501xx_wrapper *stk_wrapper)
{
    int err = 0;
    uint8_t i = 0;

    for (i = 0; i < ch_num; i++)
    {
        /* input device: setup for sar */
        stk_wrapper->channels[i].input_dev = input_allocate_device();

        if (!stk_wrapper->channels[i].input_dev)
        {
            STK_ERR("input_allocate_device for sar failed");
            return -ENOMEM;
        }

        stk_wrapper->channels[i].input_dev->name = input_dev_name[i];
        stk_wrapper->channels[i].input_dev->id.bustype = BUS_I2C;
        input_set_capability(stk_wrapper->channels[i].input_dev, EV_ABS, ABS_DISTANCE);
        input_set_drvdata(stk_wrapper->channels[i].input_dev, stk_wrapper);
        err = input_register_device(stk_wrapper->channels[i].input_dev);

        if (err)
        {
            STK_ERR("Unable to register input device: %s", stk_wrapper->channels[i].input_dev->name);
            input_free_device(stk_wrapper->channels[i].input_dev);
            return err;
        }


        input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, -1);
        input_sync(stk_wrapper->channels[i].input_dev);
        STK_ERR("Initail report -1\n");


#ifdef STK_SENSORS_DEV
        memcpy(&stk_wrapper->channels[i].sar_cdev, &stk_cdev, sizeof(struct sensors_classdev));
        stk_wrapper->channels[i].sar_cdev.name = input_dev_name[i];
        stk_wrapper->channels[i].sar_cdev.sensors_enable = stk_cdev_sensors_enable;
        stk_wrapper->channels[i].sar_cdev.sensors_poll_delay = NULL;
        stk_wrapper->channels[i].sar_cdev.sensors_flush = stk_cdev_sensors_flush;
        err = sensors_classdev_register(&stk_wrapper->channels[i].input_dev->dev, &stk_wrapper->channels[i].sar_cdev);
#endif
    }

    return 0;
}

/*
 * @brief:
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return:
 *      0: Success
 *      others: Fail
 */
static int stk_init_qualcomm(stk501xx_wrapper *stk_wrapper)
{
    int8_t err = 0, i;
    /*Create fsys class*/
    err = class_register(&capsense_class);

    if (err)
    {
        STK_ERR("Create fsys class, err=%d", err);
        return err;
    }

    /*restore sys/class/capsense label*/
    //    kobject_uevent(&capsense_class.p->subsys.kobj, KOBJ_CHANGE);

    if (stk_input_setup(stk_wrapper))
    {
        return -1;
    }

    /* sysfs: create file system */

    for (i = 0; i < ch_num; i++)
    {
        err = sysfs_create_group(&stk_wrapper->channels[i].input_dev->dev.kobj,
                                 &stk_attribute_sar_group);
    }

    if (err)
    {
        STK_ERR("Fail in sysfs_create_group, err=%d", err);
        goto err_sysfs_creat_group;
    }

    if (err)
    {
        STK_ERR("Fail in sensors_classdev_register, err=%d", err);
        goto err_sensors_classdev_register;
    }

    return 0;
err_sensors_classdev_register:
    sysfs_remove_group(&stk_wrapper->i2c_mgr.client->dev.kobj, &stk_attribute_sar_group);
err_sysfs_creat_group:

    for (i = 0; i < ch_num; i++)
    {
        input_unregister_device(stk_wrapper->channels[i].input_dev);
        input_free_device(stk_wrapper->channels[i].input_dev);
    }

    return -1;
}

/*
 * @brief: Exit qualcomm related settings safely.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_exit_qualcomm(struct stk501xx_wrapper *stk_wrapper)
{
#ifdef STK_SENSORS_DEV
    int8_t i = 0;

    for (i = 0; i < ch_num; i++)
    {
        sensors_classdev_unregister(&stk_wrapper->channels[i].sar_cdev);
    }

#endif // STK_SENSORS_DEV
    sysfs_remove_group(&stk_wrapper->i2c_mgr.client->dev.kobj,
                       &stk_attribute_sar_group);

    for (i = 0; i < ch_num; i++)
    {
        input_unregister_device(stk_wrapper->channels[i].input_dev);
#ifndef STK_SENSORS_DEV
        input_free_device(stk_wrapper->channels[i].input_dev);
#endif
    }

    class_unregister(&capsense_class);
}
void stk_report_sar_data(struct stk_data* stk)
{
    stk501xx_wrapper *stk_wrapper = container_of(stk, stk501xx_wrapper, stk);
    int i = 0;
    u8 is_change = 0;
    u8 is_dist1_change = 0;
    u8 nf_flag = 0;
    uint32_t* mapping_phase = pdata->mapping_phase;

    if (!stk_wrapper->channels[i].input_dev)
    {
        STK_ERR("No input device for sar data");
        return;
    }

    for (i = 0; i < ch_num; i ++)
    {
        nf_flag = is_change = 0;
        is_change |= stk->state_change[mapping_phase[i]];
        is_dist1_change |= stk->state_change_dist1[mapping_phase[i]];
        STK_ERR("stk_report_sar_data:: change ph[%d] =%d[%d],(%d)", mapping_phase[i], stk->state_change[mapping_phase[i]], stk->state_change_dist1[mapping_phase[i]], is_change);

        if (STK_SAR_NEAR_BY == stk->last_nearby[mapping_phase[i]])
        {
            if (((stk->dist_phase & 0x07) == mapping_phase[i]) || (((stk->dist_phase & 0x70) >> 4) == mapping_phase[i]))
            {
                nf_flag = 1; // 1st thd
            }
            else
            {
                nf_flag = 2; // Second thd
            }
        }

        if (is_change != 0)
        {
            input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, nf_flag);
            input_sync(stk_wrapper->channels[i].input_dev);
            STK_ERR("stk_report_sar_data, ph[%d] nf_flag =%d\n",  mapping_phase[i], nf_flag);
        }

        if (STK_SAR_NEAR_BY == stk->last_nearby_dist1[mapping_phase[i]])
            nf_flag = 2;  // 2nd thd

        if (is_dist1_change != 0)
        {
            input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, nf_flag);
            input_sync(stk_wrapper->channels[i].input_dev);
            STK_ERR("stk_report_sar_data, ph[%d] nf_flag =%d\n",  mapping_phase[i], nf_flag);
        }
    }
}

#ifdef CONFIG_OF
/*
 * @brief: Parse data in device tree
 *
 * @param[in] dev: struct device *
 * @param[in/out] pdata: struct stk501xx_platform_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_parse_dt(struct device *dev,
                        struct stk501xx_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    const int *p;
    uint32_t int_flags;
    p = of_get_property(np, "stk,direction", NULL);

    if (p)
        pdata->direction = be32_to_cpu(*p);

    pdata->interrupt_int1_pin = of_get_named_gpio_flags(np,
                                "stk501xx,irq-gpio", 0, &int_flags);

    if (pdata->interrupt_int1_pin < 0)
    {
        STK_ERR("Unable to read stk501xx,irq-gpio");
#ifdef STK_INTERRUPT_MODE
        return pdata->interrupt_int1_pin;
#else /* no STK_INTERRUPT_MODE */
        return 0;
#endif /* STK_INTERRUPT_MODE */
    }

    pdata->phase_en = 0x7f;
    of_property_read_u32(np,"stk,phase_en",&pdata->phase_en);
    of_property_read_u32_array(np,"stk,mapping_phase", (u32*)&(pdata->mapping_phase[0]), 8);
    of_property_read_u32_array(np,"stk,sar_thd0", (u32*)&(pdata->sar_thd0[0]), 8);
    of_property_read_u32_array(np,"stk,sar_thd1", (u32*)&(pdata->sar_thd1[0]), 8);
    of_property_read_u32_array(np,"stk,coef_t", (u32*)&(pdata->coef_t[0]), 16);
    of_property_read_u32_array(np,"stk,tc_config", (u32*)&(pdata->tc_config[0]), 9);
    // load in registers from device tree
    of_property_read_u32(np,"stk,reg-num",&pdata->i2c_reg_num);
    STK_LOG("size of elements %d \n", pdata->i2c_reg_num);
    if (pdata->i2c_reg_num > 0)
    {
        // initialize platform reg data array
        pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
        if (unlikely(pdata->pi2c_reg == NULL))
        {
                STK_ERR("size of elements %d alloc error\n", pdata->i2c_reg_num);
                return -ENOMEM;
        }
        // initialize the array
        if (of_property_read_u32_array(np,"stk,reg-init",(u32*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num/sizeof(u32)))
            return -ENOMEM;
    }

    return 0; /* SUCCESS */
}
#else
static int stk_parse_dt(struct device *dev,
                        struct stk501xx_platform_data *pdata)
{
    return -ENODEV
}
#endif /* CONFIG_OF */

/*
 * @brief: Get platform data
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int get_platform_data(stk501xx_wrapper *stk_wrapper)
{
    int err = 0;
    struct i2c_client *client = stk_wrapper->i2c_mgr.client;
    struct stk501xx_platform_data *stk_platdata;

    if (client->dev.of_node)
    {
        stk_platdata = devm_kzalloc(&client->dev,
                                    sizeof(struct stk501xx_platform_data), GFP_KERNEL);

        if (!stk_platdata)
        {
            STK_ERR("Failed to allocate memory");
            return -ENOMEM;
        }

        err = stk_parse_dt(&client->dev, stk_platdata);

        if (err)
        {
            STK_ERR("stk_parse_dt err=%d", err);
            return err;
        }
    }
    else
    {
        if (NULL != client->dev.platform_data)
        {
            STK_ERR("probe with platform data");
            stk_platdata = client->dev.platform_data;
        }
        else
        {
            STK_ERR("probe with private platform data");
            stk_platdata = &stk_plat_data;
        }
    }

#ifdef STK_INTERRUPT_MODE
    stk_wrapper->stk.gpio_info.int_pin = stk_platdata->interrupt_int1_pin;
    STK_ERR("int_pin=%d", stk_wrapper->stk.gpio_info.int_pin);
#endif /* STK_INTERRUPT_MODE */
    stk_wrapper->stk.pdata = stk_platdata;
    pdata  = stk_platdata;
    //stk->direction = stk_platdata->direction;
    STK_LOG("phase_en %x \n", pdata->phase_en);
    STK_LOG("mapping_phase %d - %d\n", pdata->mapping_phase[0], pdata->mapping_phase[7]);
    STK_LOG("sar_thd0 %d - %d\n", pdata->sar_thd0[0], pdata->sar_thd0[7]);
    STK_LOG("sar_thd1 %d - %d\n", pdata->sar_thd1[0], pdata->sar_thd1[7]);
    STK_LOG("coef_t %d - %d\n", pdata->coef_t[0], pdata->coef_t[15]);
    STK_LOG("tc_config %d - %d\n", pdata->tc_config[0], pdata->tc_config[8]);
    return 0;
}

#ifdef CONFIG_CAPSENSE_USB_CAL
static void ps_notify_callback_work(struct work_struct *work)
{
    STK_ERR("class_stk_phase_USB_cali , reset all phase\n");
    stk501xx_phase_reset(global_stk, STK_TRIGGER_REG_INIT_ALL);
}

static int ps_get_state(struct power_supply *psy, bool *present)
{
    union power_supply_propval pval = {0};
    int retval;

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
    retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
            &pval);
#else
    retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
            &pval);
#endif

    if (retval) {
        STK_LOG("%s psy get property failed\n", psy->desc->name);
        return retval;
    }
    *present = (pval.intval) ? true : false;

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
    STK_LOG("%s is %s\n", psy->desc->name,
            (*present) ? "online" : "not online");
#else
    STK_LOG("%s is %s\n", psy->desc->name,
            (*present) ? "present" : "not present");
#endif

    return 0;
}

static int ps_notify_callback(struct notifier_block *self,
        unsigned long event, void *p)
{
    struct stk501xx_wrapper *data =
        container_of(self, struct stk501xx_wrapper, ps_notif);
    struct power_supply *psy = p;
    bool present;
    int retval;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
    if (event == PSY_EVENT_PROP_CHANGED
#else
    if ((event == PSY_EVENT_PROP_ADDED || event == PSY_EVENT_PROP_CHANGED)
#endif
            && psy && psy->desc->get_property && psy->desc->name &&
            !strncmp(psy->desc->name, "usb", sizeof("usb")) && data) {
        STK_LOG("ps notification: event = %lu\n", event);
        retval = ps_get_state(psy, &present);
        if (retval) {
            return retval;
        }

        if (event == PSY_EVENT_PROP_CHANGED) {
            if (data->ps_is_present == present) {
                STK_LOG("ps present state not change\n");
                return 0;
            }
        }
        data->ps_is_present = present;
        schedule_work(&data->ps_notify_work);
    }

#ifdef CONFIG_CAPSENSE_ATTACH_CAL
    if (event == PSY_EVENT_PROP_CHANGED
            && psy && psy->desc->get_property && psy->desc->name &&
            !strncmp(psy->desc->name, "phone", sizeof("phone")) && data) {
        STK_LOG("phone ps notification: event = %lu\n", event);

        retval = ps_get_state(psy, &present);
        if (retval)
            return retval;

        if (data->phone_is_present != present) {
            data->phone_is_present = present;
            schedule_work(&data->ps_notify_work);
        }
    }
#endif

    return 0;
}

#ifdef CONFIG_CAPSENSE_FLIP_CAL
static void write_flip_regs(int num_regs, struct smtc_reg_data *regs)
{
    int i;

    for(i=0; i < num_regs; i++)
    {
  /* Write all registers/values contained in i2c_reg */
        STK_LOG("Going to Write Reg from dts: 0x%x Value: 0x%x\n",
                regs[i].reg, regs[i].val);
        STK_REG_WRITE(global_stk, regs[i].reg, (uint8_t*)&regs[i].val);
    }
}

static void update_flip_regs(struct stk501xx_wrapper *data, unsigned long state)
{
    if (data->phone_flip_update_regs) {
        if (state == data->phone_flip_open_val) {
  /* Flip open */
            STK_LOG("Writing %d regs on open\n",
                data->num_flip_open_regs);
            write_flip_regs(data->num_flip_open_regs, data->flip_open_regs);
        } else {
   /* Flip closed */
            STK_LOG("Writing %d regs on close\n",
                data->num_flip_closed_regs);
            write_flip_regs(data->num_flip_closed_regs, data->flip_closed_regs);
        }
    }
}

static int flip_notify_callback(struct notifier_block *self,
        unsigned long state, void *p)
{
    struct stk501xx_wrapper *data =
        container_of(self, struct stk501xx_wrapper, flip_notif);
    struct extcon_dev *edev = p;

    if(data->ext_flip_det == edev) {
        if(data->phone_flip_state != state) {
            update_flip_regs(data, state);
            data->phone_flip_state = state;
            schedule_work(&data->ps_notify_work);
        }
    }

 return 0;
}
#endif
#endif
/*
 * @brief: Probe function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] stk_bus_ops: const struct stk_bus_ops *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
int stk_i2c_probe(struct i2c_client *client, struct common_function *common_fn)
{
    int err = 0;
    stk501xx_wrapper *stk_wrapper;
    struct stk_data *stk;
#ifdef CONFIG_CAPSENSE_USB_CAL
    struct power_supply *psy = NULL;
#endif
    STK_LOG("STK_HEADER_VERSION: %s ", STK_HEADER_VERSION);
    STK_LOG("STK_C_VERSION: %s ", STK_C_VERSION);
    STK_LOG("STK_DRV_I2C_VERSION: %s ", STK_DRV_I2C_VERSION);
    STK_LOG("STK_QUALCOMM_VERSION: %s ", STK_QUALCOMM_VERSION);

    if (NULL == client)
    {
        return -ENOMEM;
    }
    else if (!common_fn)
    {
        STK_ERR("cannot get common function. EXIT");
        return -EIO;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        err = i2c_get_functionality(client->adapter);
        STK_ERR("i2c_check_functionality error, functionality=0x%x", err);
        return -EIO;
    }

    /* kzalloc: allocate memory and set to zero. */
    stk_wrapper = kzalloc(sizeof(stk501xx_wrapper), GFP_KERNEL);

    if (!stk_wrapper)
    {
        STK_ERR("memory allocation error");
        return -ENOMEM;
    }

    stk = &stk_wrapper->stk;
    global_stk = stk;

    if (!stk)
    {
        STK_ERR("failed to allocate stk501xx_data");
        return -ENOMEM;
    }

    stk_wrapper->i2c_mgr.client = client;
    stk_wrapper->i2c_mgr.addr_type = ADDR_16BIT;
    stk->bops   = common_fn->bops;
    stk->tops   = common_fn->tops;
    stk->gops   = common_fn->gops;
    stk->sar_report_cb = stk_report_sar_data;
    i2c_set_clientdata(client, stk_wrapper);
    mutex_init(&stk_wrapper->i2c_mgr.lock);
    stk->bus_idx = stk->bops->init(&stk_wrapper->i2c_mgr);

    if (stk->bus_idx < 0)
    {
        goto err_free_mem;
    }

    if (get_platform_data(stk_wrapper))
        goto err_free_mem;

    err = stk501xx_init_client(stk);

    if (err < 0)
    {
        STK_ERR("stk501xx_init_client failed");
        goto err_exit;
    }

    if (stk_init_qualcomm(stk_wrapper))
    {
        STK_ERR("stk_init_qualcomm failed");
        goto err_exit;
    }

    stk_report_sar_state(stk, -1);
#ifdef CONFIG_CAPSENSE_USB_CAL
  /*notify usb state*/
        INIT_WORK(&stk_wrapper->ps_notify_work, ps_notify_callback_work);
        stk_wrapper->ps_notif.notifier_call = ps_notify_callback;
        err = power_supply_reg_notifier(&stk_wrapper->ps_notif);
        if (err)
            STK_ERR("Unable to register ps_notifier: %d\n", err);

        psy = power_supply_get_by_name("usb");
        if (psy) {
            err = ps_get_state(psy, &stk_wrapper->ps_is_present);
            if (err) {
                STK_ERR("psy get property failed rc=%d\n", err);
                power_supply_unreg_notifier(&stk_wrapper->ps_notif);
            }
        }
#ifdef CONFIG_CAPSENSE_FLIP_CAL
        if (of_property_read_bool(client->dev.of_node, "extcon")) {
            stk_wrapper->flip_notif.notifier_call = flip_notify_callback;
            stk_wrapper->ext_flip_det =
                extcon_get_edev_by_phandle(&client->dev, 0);
            if (IS_ERR(stk_wrapper->ext_flip_det)) {
                stk_wrapper->ext_flip_det = NULL;
                STK_ERR("failed to get extcon flip dev\n");
            } else {
                if(extcon_register_notifier(stk_wrapper->ext_flip_det,
                    EXTCON_MECHANICAL, &stk_wrapper->flip_notif))
                    STK_ERR("failed to register extcon flip dev notifier\n");
                else {
                    stk_wrapper->phone_flip_state =
                        extcon_get_state(stk_wrapper->ext_flip_det,
                            EXTCON_MECHANICAL);
                    update_flip_regs(stk_wrapper, stk_wrapper->phone_flip_state);
                }
            }
        } else
            STK_ERR("extcon not in dev tree!\n");
#endif
#endif

    STK_LOG("Success");
    return 0;
err_exit:
#ifdef STK_INTERRUPT_MODE
    STK_GPIO_IRQ_REMOVE(stk, &stk->gpio_info);
#elif defined STK_POLLING_MODE
    STK_TIMER_REMOVE(stk, &stk->stk_timer_info);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
#ifdef STK_SENSING_WATCHDOG
    STK_TIMER_REMOVE(stk, &stk->sensing_watchdog_timer_info);
#endif // STK_SENSING_WATCHDOG
err_free_mem:
    mutex_destroy(&stk_wrapper->i2c_mgr.lock);
    kfree(stk_wrapper);
    return err;
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
int stk_i2c_remove(struct i2c_client *client)
{
    stk501xx_wrapper *stk_wrapper = i2c_get_clientdata(client);
    struct stk_data *stk = &stk_wrapper->stk;
#ifdef CONFIG_CAPSENSE_USB_CAL
    cancel_work_sync(&stk_wrapper->ps_notify_work);
    power_supply_unreg_notifier(&stk_wrapper->ps_notif);
#endif

    stk_exit_qualcomm(stk_wrapper);
#ifdef STK_INTERRUPT_MODE
    STK_GPIO_IRQ_REMOVE(stk, &stk->gpio_info);
#elif defined STK_POLLING_MODE
    STK_TIMER_REMOVE(stk, &stk->stk_timer_info);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    stk->bops->remove(&stk_wrapper->i2c_mgr);
    mutex_destroy(&stk_wrapper->i2c_mgr.lock);
    kfree(stk_wrapper);
    return 0;
}

int stk501xx_suspend(struct device* dev)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    struct stk_data *stk = &stk_wrapper->stk;

    if (stk->enabled)
    {
        stk501xx_set_enable(stk, 0, true);
        stk->last_enable = true;
    }
    else
        stk->last_enable = false;

    return 0;
}

int stk501xx_resume(struct device* dev)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    struct stk_data *stk = &stk_wrapper->stk;

    if (stk->last_enable)
        stk501xx_set_enable(stk, 1, true);

    stk->last_enable = false;
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id stk501xx_match_table[] =
{
    { .compatible = "stk,stk501xx", },
    {}
};
#endif /* CONFIG_OF */

/*
 * @brief: Proble function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] id: struct i2c_device_id *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk501xx_i2c_probe(struct i2c_client* client,
                              const struct i2c_device_id* id)
{
    struct common_function common_fn =
    {
        .bops = &stk_i2c_bops,
        .tops = &stk_t_ops,
        .gops = &stk_g_ops,
    };
    return stk_i2c_probe(client, &common_fn);
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
static int stk501xx_i2c_remove(struct i2c_client* client)
{
    return stk_i2c_remove(client);
}

/**
 * @brief:
 */
static int stk501xx_i2c_detect(struct i2c_client* client, struct i2c_board_info* info)
{
    strcpy(info->type, STK501XX_NAME);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * @brief: Suspend function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk501xx_i2c_suspend(struct device* dev)
{
    return stk501xx_suspend(dev);
}

/*
 * @brief: Resume function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk501xx_i2c_resume(struct device* dev)
{
    return stk501xx_resume(dev);
}

static const struct dev_pm_ops stk501xx_pm_ops =
{
    .suspend = stk501xx_i2c_suspend,
    .resume = stk501xx_i2c_resume,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id stk501xx_acpi_id[] =
{
    {"STK501XX", 0},
    {}
};
MODULE_DEVICE_TABLE(acpi, stk501xx_acpi_id);
#endif /* CONFIG_ACPI */

static const struct i2c_device_id stk501xx_i2c_id[] =
{
    {STK501XX_NAME, 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, stk501xx_i2c_id);

static struct i2c_driver stk501xx_i2c_driver =
{
    .probe = stk501xx_i2c_probe,
    .remove = stk501xx_i2c_remove,
    .detect = stk501xx_i2c_detect,
    .id_table = stk501xx_i2c_id,
    .class = I2C_CLASS_HWMON,
    .driver = {
        .owner = THIS_MODULE,
        .name = STK501XX_NAME,
#ifdef CONFIG_PM_SLEEP
        .pm = &stk501xx_pm_ops,
#endif
#if defined(CONFIG_ACPI) && !defined(STK_SPREADTRUM)
        .acpi_match_table = ACPI_PTR(stk501xx_acpi_id),
#endif /* CONFIG_ACPI */
#if defined(CONFIG_OF) || defined(STK_SPREADTRUM)
        .of_match_table = stk501xx_match_table,
#endif /* CONFIG_OF */
    }
};
module_i2c_driver(stk501xx_i2c_driver);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk501xx sar driver");
MODULE_LICENSE("GPL");
//MODULE_VERSION(STK_QUALCOMM_VERSION);
