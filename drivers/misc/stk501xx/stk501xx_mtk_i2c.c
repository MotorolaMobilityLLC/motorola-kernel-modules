/* stk501xx_mtk_i2c.c - stk501xx SAR (driver)
 *
 * Copyright (c) 2024, Sensortek.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "stk501xx_mtk_i2c.h"

#define USB_POWER_SUPPLY_NAME   "mtk-master-charger"

static struct stk_data* gStk = NULL;

#define STK_DRI_GET_DATA(ddri) \
    dev_get_drvdata((struct device *)container_of(ddri, struct device, driver))

struct stk_data *global_stk;

static int32_t stk_init_flag = 0;


static void stk_report_sar_always_far_data(struct stk_data* stk, int8_t nf_flag)
{
    stk501xx_wrapper *stk_wrapper = container_of(stk, stk501xx_wrapper, stk);
    int32_t i = 0;

    if (!stk_wrapper->channels[i].input_dev)
    {
        STK_ERR("No input device for sar data");
        return;
    }

    for (i = 0; i < stk->pdata->ch_num; i ++)
    {
        input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, nf_flag);
        input_sync(stk_wrapper->channels[i].input_dev);
        STK_LOG("Always report FAR (%d)", nf_flag);
    }
}

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

    if (error)
    {
        STK_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    STK_LOG("stk_enable_store, data=%d", data);

    if ((1 == data) || (0 == data))
    {
        stk501xx_set_enable(global_stk, data, false);
        if(1 == data)
            stk_report_sar_always_far_data(global_stk, 0);
        else
            stk_report_sar_always_far_data(global_stk, -1);
    }
    else
    {
        STK_ERR("invalid argument, en=%d", data);
    }
    return count;
}

static ssize_t capsense_raw_data_show(struct class *class,
                                        struct class_attribute *attr, char *buf)
{
    char *p = buf;
    uint8_t i = 0;
    uint16_t reg;
    uint32_t val, rdata, cadc;
    int32_t  diff;

    for (i = 0; i < 8; i++)
    {
        //raw data
        reg = (uint16_t)(STK_ADDR_REG_RAW_PH0_REG + (i * 0x04));
        STK_REG_READ(global_stk, reg, (uint8_t*)&rdata);

        //delta
        reg = (uint16_t)(STK_ADDR_REG_DELTA_PH0_REG + (i * 0x04));
        STK_REG_READ(global_stk, reg, (uint8_t*)&val);

        if (val & 0x80000000)
        {
            //2's complement = 1's complement +1
            diff = (int32_t)(~val + 1);
            diff *= -1;
        }
        else
        {
            diff = (int32_t)(val);
        }

        //cadc
        reg = (uint16_t)(STK_ADDR_REG_CADC_PH0_REG + (i * 0x04));
        STK_REG_READ(global_stk, reg, (uint8_t*)&cadc);

        p += snprintf(p, PAGE_SIZE, "PH= %d raw = %u, delta = %d, cadc = %d \n",
                            i, rdata, diff, cadc);
    }
    return (p-buf);
}

static ssize_t class_stk_power_en(struct class *class,
                                  struct class_attribute *attr, const char *buf, size_t count)
{
    stk_report_sar_always_far_data(global_stk, 0);
    return count;
}

static ssize_t class_stk_flag_show(struct class *class,
                                   struct class_attribute *attr, char *buf)
{
    int i = 0;
    uint32_t prox_flag = 0;
    STK_LOG("stk_flag_show");
    //read prox flag
    stk_read_prox_flag(global_stk, &prox_flag);
    stk501xx_read_sar_data(global_stk, prox_flag);

    for ( i = 0; i < 8; i++)
    {
        STK_LOG("ph[%d] near/far flag=%d\n", i, global_stk->last_nearby[i]);
    }

    return scnprintf(buf, PAGE_SIZE, "prox flag=0x%d\n", prox_flag);
}
static ssize_t class_stk_phase_cali(struct class *class,
                                    struct class_attribute *attr, char *buf)
{
    int result = 0;
    uint32_t val = STK_TRIGGER_REG_INIT_ALL(global_stk->phase_en);
    STK_LOG("class_stk_phase_cali , reset all phase\n");
    stk501xx_phase_reset(global_stk, val);
    return (ssize_t)result;
}

static ssize_t class_stk_phase_cali_store(struct class *class,
        struct class_attribute *attr, const char *buf, size_t count)
{
    uint32_t val = STK_TRIGGER_REG_INIT_ALL(global_stk->phase_en);
    STK_LOG("class_stk_phase_cali_store , reset all phase\n");
    stk501xx_phase_reset(global_stk, val);

    return count;
}

static ssize_t class_stk_set_thd(struct class *class,
                                 struct class_attribute *attr, const char *buf, size_t count)
{
    u32 ph_idx, thd;

    if (sscanf(buf, "%d,%d", &ph_idx, &thd) != 2)
    {
        STK_ERR("please input two DEC numbers: ph_id,thd (ph_id: phase number, thd)\n");
        return -EINVAL;
    }

    STK_LOG("set ph[%x] = %d\n", ph_idx, thd);
    stk501xx_set_each_thd(global_stk, ph_idx, thd);
    return count;
}

static ssize_t stk_channel_en_show(struct class *class,
                                   struct class_attribute *attr, char *buf)
{
    int8_t i = 0;
    char *p = buf;
    stk501xx_wrapper *stk_wrapper = container_of(global_stk, stk501xx_wrapper, stk);
    STK_LOG("ch_num=%d", global_stk->pdata->ch_num);

    for (i = 0; i < global_stk->pdata->ch_num; i++)
    {
        STK_LOG("ch[%i]: enabled = %d", i, stk_wrapper->channels[i].enabled);
        p += snprintf(p, PAGE_SIZE, "ch[%i]: enabled = %d\n", i, stk_wrapper->channels[i].enabled);
    }

    return (p - buf);
}

static ssize_t stk_channel_en_store(struct class *class,
                                    struct class_attribute *attr, const char *buf, size_t count)
{
    uint8_t nf_flag = SAR_STATE_FAR;
    u32 ch_idx,en,val;
    bool prev_states = false;
#ifdef STK_POLLING_MODE
    bool total_states = false;
    uint8_t i;
#endif
    stk501xx_wrapper *stk_wrapper = container_of(global_stk, stk501xx_wrapper, stk);

    if (sscanf(buf, "%d,%d", &ch_idx, &en) != 2)
    {
        STK_ERR("please input two DEC numbers: ch_id,en \
                (ch_id: channel number, en: 1=enable, 0=disable)\n");
        return -EINVAL;
    }

    if ((ch_idx >= global_stk->pdata->ch_num) || (ch_idx < 0))
    {
        STK_ERR("chanel index over %d\n", global_stk->pdata->ch_num - 1);
        return count;
    }

    STK_LOG("set ch[%x] = %d\n", ch_idx, en);
    prev_states = stk_wrapper->channels[ch_idx].enabled ;
    stk_wrapper->channels[ch_idx].enabled = en ? 1 : 0;

    if (en)
    {
        STK_REG_READ(global_stk, STK_ADDR_TRIGGER_REG, (uint8_t*)&val);
        val |= (1 << ch_idx);
        STK_REG_WRITE(global_stk, STK_ADDR_TRIGGER_REG, (uint8_t*)&val);
        nf_flag = SAR_STATE_FAR;
#ifdef STK_POLLING_MODE
        STK_TIMER_START(global_stk, &global_stk->stk_timer_info);
#endif
#ifdef TEMP_COMPENSATION
        global_stk->last_prox_a_state = 0;
        global_stk->last_prox_b_state = 0;
#endif
    }
    else
    {
        STK_REG_READ(global_stk, STK_ADDR_TRIGGER_REG, (uint8_t*)&val);
        val = (uint32_t)~(~(val) | 1 << ch_idx);
        STK_REG_WRITE(global_stk, STK_ADDR_TRIGGER_REG, (uint8_t*)&val);
        nf_flag = SAR_STATE_NONE;
#ifdef TEMP_COMPENSATION
        clr_temp(global_stk);
#endif
    }

#ifdef STK_POLLING_MODE

    for (i = 0; i < global_stk->pdata->ch_num; i ++)
    {
        total_states |= stk_wrapper->channels[i].enabled;
    }

    if (!total_states)
        STK_TIMER_STOP(global_stk, &global_stk->stk_timer_info);

#endif

    if (ch_idx < global_stk->pdata->ch_num)
    {
        if (prev_states != stk_wrapper->channels[ch_idx].enabled)
        {
            input_report_abs(stk_wrapper->channels[ch_idx].input_dev, ABS_DISTANCE, nf_flag);
            input_sync(stk_wrapper->channels[ch_idx].input_dev);
            val = STK_TRIGGER_CMD_REG_INIT_ALL;
            STK_REG_WRITE(global_stk, STK_ADDR_TRIGGER_CMD, (uint8_t*)&val);
            //force read again
            STK_REG_READ(global_stk, STK_ADDR_TRIGGER_CMD, (uint8_t*)&val);
            STK_LOG("stk_enable_store, phase enable/disable\n");
        }
    }

    stk_clr_intr(global_stk, &val);
    return count;
}

static ssize_t stk_int_state_show(struct class *class,
                                  struct class_attribute *attr, char *buf)
{
    STK_LOG("stk_int_state_show");
    STK_LOG("into_int_before=%d", !global_stk->first_init);
    return scnprintf(buf, PAGE_SIZE, "%d\n", !global_stk->first_init);
}

static ssize_t reg_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
    uint32_t *val = (u32*)buf;
    stk501xx_wrapper *stk_wrapper = container_of(global_stk, stk501xx_wrapper, stk);
    stk_data *stk = &stk_wrapper->stk;

    STK_LOG("reg_show");
    if(stk_wrapper->read_flag)
    {
        stk_wrapper->read_flag = 0;
        if (STK_REG_READ(stk, stk_wrapper->read_reg, (uint8_t*)val) < 0)
        {
            return -1;
        }
        STK_LOG("read_reg=0x%x, val=0x%x", stk_wrapper->read_reg, *val);

        return 4;
    }

	return -1;
}

/*
	reg attr is for TCMD on MTK only,
	buf[6]-read_flag:
	0-real write,
	1-just transfer the reg value want to be readed
*/
static ssize_t reg_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
    stk501xx_wrapper *stk_wrapper = container_of(global_stk, stk501xx_wrapper, stk);
    stk_data *stk = &stk_wrapper->stk;
    int read_bit;
    u16 regaddr = 0;
    u32 val = 0;
    int i = 0;

    if (count != 7)
    {
        STK_ERR("%s : params error[ count == %lu ]",__func__, count);
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        STK_LOG("%s : buf[%d] = 0x%x", __func__, i, buf[i]);
    }

    regaddr = ((uint32_t)buf[0] << 8) | (uint32_t)buf[1];
    val = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5]);
    read_bit = buf[6];
    stk_wrapper->read_flag = read_bit;
    STK_LOG("write reg[0x%x]=0x%x, read_bit=%d", regaddr, val, read_bit);

    if (read_bit == 0)
    {
        if (STK_REG_WRITE(stk, (uint16_t)regaddr, (uint8_t*)&val) < 0)
        {
            return -1;
        }
    }
    else if (read_bit == 1)
    {
        stk_wrapper->read_reg = regaddr;
    }
	return count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static struct class_attribute class_attr_enable =
    __ATTR(enable, 0664, class_stk_enable_show, class_stk_enable_store);
static struct class_attribute class_attr_raw_data =
    __ATTR(raw_data, 0444, capsense_raw_data_show, NULL);
static struct class_attribute class_attr_power_enable =
    __ATTR(power_enable, 0220, NULL, class_stk_power_en);
static struct class_attribute class_attr_user_test =
    __ATTR(user_test, 0664, class_stk_phase_cali, class_stk_phase_cali_store);
static struct class_attribute class_attr_flag =
    __ATTR(flag, 0444, class_stk_flag_show, NULL);
static struct class_attribute class_attr_set_thd =
    __ATTR(set_thd, 0220, NULL, class_stk_set_thd);
static struct class_attribute class_attr_channel_en =
    __ATTR(channel_en, 0664, stk_channel_en_show, stk_channel_en_store);
static struct class_attribute class_attr_reset =
    __ATTR(reset, 0664, class_stk_phase_cali, class_stk_phase_cali_store);
static struct class_attribute class_attr_int_state =
    __ATTR(int_state, 0444, stk_int_state_show, NULL);
static struct class_attribute class_attr_reg =
	__ATTR(reg, 0660, reg_show, reg_store);

static struct attribute *capsense_class_attrs[] =
{
    &class_attr_enable.attr,
    &class_attr_raw_data.attr,
    &class_attr_power_enable.attr,
    &class_attr_user_test.attr,
    &class_attr_flag.attr,
    &class_attr_set_thd.attr,
    &class_attr_channel_en.attr,
    &class_attr_reset.attr,
    &class_attr_int_state.attr,
    &class_attr_reg.attr,
    NULL,
};

ATTRIBUTE_GROUPS(capsense_class);
#else
static struct class_attribute capsense_class_attributes[] =
{
    __ATTR(enable, 0664, class_stk_enable_show, class_stk_enable_store),
    __ATTR(raw_data, 0444, capsense_raw_data_show, NULL),
    __ATTR(power_enable, 0220, NULL, class_stk_power_en),
    __ATTR(user_test, 0664, class_stk_phase_cali, class_stk_phase_cali_store),
    __ATTR(flag, 0444, class_stk_flag_show, NULL),
    __ATTR(set_thd, 0220, NULL, class_stk_set_thd),
    __ATTR(channel_en, 0664, stk_channel_en_show, stk_channel_en_store),
    __ATTR(reset, 0664, class_stk_phase_cali, class_stk_phase_cali_store),
    __ATTR(int_state, 0444, stk_int_state_show, NULL),
    __ATTR(reg, 0660, reg_show, reg_store),
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
        .sensor_power = "0.1",
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
//    struct stk501xx_wrapper *stk_wrapper = container_of(sensors_cdev, stk501xx_wrapper, channels[0].sar_cdev);
//    struct stk_data *stk = &stk_wrapper->stk;

    struct stk_data *stk = global_stk;
    STK_LOG("stk_cdev_sensors_enable , en=%d\n", enabled);

    if (0 == enabled)
    {
        stk501xx_set_enable(stk, 0, false);
        stk_report_sar_always_far_data(stk, -1);
    }
    else if (1 == enabled)
    {
        stk501xx_set_enable(stk, 1, false);
        stk_report_sar_always_far_data(stk, 0);
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
static int stk_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
                                       unsigned int delay_msec)
{
#ifdef STK_POLLING_MODE
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
static ssize_t stk_enable_show(struct device* dev,
                               struct device_attribute* attr, char* buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    char en;
    en = stk->enabled;
    return scnprintf(buf, PAGE_SIZE, "%d\n", en);
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
    uint32_t data;
    int32_t error;
    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        STK_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    STK_LOG("stk_enable_store, data=%d", data);

    if ((1 == data) || (0 == data))
    {
        stk501xx_set_enable(stk, data, false);
        if(1 == data)
            stk_report_sar_always_far_data(stk, 0);
        else
            stk_report_sar_always_far_data(stk, -1);
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
static ssize_t stk_value_show(struct device* dev,
                              struct device_attribute* attr, char* buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    uint8_t i = 0;
    uint32_t prox_flag = 0;
    stk_data *stk = &stk_wrapper->stk;
    STK_LOG("stk_value_show");
    //read prox flag
    stk_read_prox_flag(stk, &prox_flag);
    stk501xx_read_sar_data(stk, prox_flag);

    for (i = 0; i < 8; i++)
    {
        scnprintf(buf, PAGE_SIZE, "ph[%d] value=%d\n", i, stk->last_data[i]);
        STK_LOG("ph[%d] value=%d\n", i, stk->last_data[i]);
    }

    return 0;
}

static ssize_t stk_flag_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    uint8_t i = 0;
    uint32_t prox_flag = 0;
    stk_data *stk = &stk_wrapper->stk;
    STK_LOG("stk_flag_show");
    //read prox flag
    stk_read_prox_flag(stk, &prox_flag);
    stk501xx_read_sar_data(stk, prox_flag);

    for ( i = 0; i < 8; i++)
    {
        STK_LOG("ph[%d] prox flag=%d", i, stk->last_nearby[i]);
    }

    return scnprintf(buf, PAGE_SIZE, "flag=%d\n", prox_flag);
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
    int32_t err, i;
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

    STK_LOG("write reg[0x%X]=0x%X", addr, cmd);

    if (!stk->enabled)
        stk501xx_set_enable(stk, 1, true);
    else
        enable = true;

    if (0 > STK_REG_WRITE_BLOCK(stk, (uint16_t)addr, (uint8_t*)&cmd, 4))
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
#ifdef TEMP_COMPENSATION
static ssize_t stk_temp_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    STK_LOG("stk_temp_show");
    stk501xx_read_temp_data(stk, STK_ADDR_REG_RAW_PH0_REG, &stk->prev_temperature_ref_a);
    return scnprintf(buf, PAGE_SIZE, "temperature=%d\n", stk->prev_temperature_ref_a);
}
#endif
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
    int32_t result;

    if (!buf)
        return -1;

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
    STK_LOG("chip id=0x%x, index=0x%x", stk->chip_id, stk->chip_index);
    return scnprintf(buf, PAGE_SIZE, "pid=0x%x,index=0x%x\n", stk->chip_id, stk->chip_index);
}

static ssize_t stk_phase_cali(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    int32_t result = 0;
    stk_data *stk = &stk_wrapper->stk;
    uint32_t val = STK_TRIGGER_REG_INIT_ALL(stk->phase_en);
    stk501xx_phase_reset(stk, val);
    return (ssize_t)result;
}

static ssize_t stk_conv_chk_show(struct device* dev,
                               struct device_attribute* attr, char* buf)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    char en;
    en = stk->dis_conv_done_chk;
    return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

static ssize_t stk_conv_chk_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    stk501xx_wrapper *stk_wrapper = dev_get_drvdata(dev);
    stk_data *stk = &stk_wrapper->stk;
    uint32_t data;
    int32_t error;
    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        STK_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    STK_LOG("stk_conv_chk_store, data=%d", data);

    if ((1 == data) || (0 == data))
    {
        stk->dis_conv_done_chk = data;
    }
    else
    {
        STK_ERR("invalid argument, en=%d", data);
    }
    return count;
}

static DEVICE_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DEVICE_ATTR(value, 0444, stk_value_show, NULL);
static DEVICE_ATTR(send, 0220, NULL, stk_send_store);
#ifdef TEMP_COMPENSATION
    static DEVICE_ATTR(temp, 0444, stk_temp_show, NULL);
#endif
static DEVICE_ATTR(flag, 0444, stk_flag_show, NULL);
static DEVICE_ATTR(allreg, 0444, stk_allreg_show, NULL);
static DEVICE_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DEVICE_ATTR(phcali, 0444, stk_phase_cali, NULL);
static DEVICE_ATTR(conv_chk, 0664, stk_conv_chk_show, stk_conv_chk_store);

static struct attribute *stk_attr_list[] =
{
    &dev_attr_enable.attr,
    &dev_attr_value.attr,
    &dev_attr_send.attr,
#ifdef TEMP_COMPENSATION
    &dev_attr_temp.attr,
#endif
    &dev_attr_flag.attr,
    &dev_attr_allreg.attr,
    &dev_attr_chipinfo.attr,
    &dev_attr_phcali.attr,
    &dev_attr_conv_chk.attr,
    NULL
};

static const struct attribute_group stk501xx_attr_group =
{
    .name = STK501XX_NAME,
    .attrs = stk_attr_list,
};

static int32_t stk501xx_input_open(struct input_dev* input)
{
    if (!gStk->enabled)
        stk501xx_set_enable(gStk, 1, false);

    return 0;
}

static void stk501xx_input_close(struct input_dev* input)
{
//    if (gStk->enabled)
//        stk501xx_set_enable(gStk, 0);
}

static int stk_input_setup(stk501xx_wrapper *stk_wrapper)
{
    int err = 0;
    uint8_t i = 0;

    for (i = 0; i < global_stk->pdata->ch_num; i++)
    {
        STK_LOG("register ch[%d]", i);
        /* input device: setup for sar */
        stk_wrapper->channels[i].input_dev = input_allocate_device();

        if (!stk_wrapper->channels[i].input_dev)
        {
            STK_ERR("input_allocate_device for sar failed");
            return -ENOMEM;
        }

        stk_wrapper->channels[i].input_dev->name = input_dev_name[i];
        stk_wrapper->channels[i].input_dev->id.bustype = BUS_I2C;
        stk_wrapper->channels[i].input_dev->open = stk501xx_input_open;
        stk_wrapper->channels[i].input_dev->close = stk501xx_input_close;
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

        STK_LOG("[%d] name =%s, type =%d\n", i, stk_sensord_dev[i].name, stk_sensord_dev[i].type);
#ifdef STK_SENSORS_DEV
//        stk_wrapper->channels[i].sar_cdev = stk_cdev;
        memcpy(&stk_wrapper->channels[i].sar_cdev, &stk_cdev, sizeof(struct sensors_classdev));
        stk_wrapper->channels[i].sar_cdev.name = stk_sensord_dev[i].name;
        stk_wrapper->channels[i].sar_cdev.sensors_enable = stk_cdev_sensors_enable;
        stk_wrapper->channels[i].sar_cdev.sensors_poll_delay = stk_cdev_sensors_poll_delay;
        stk_wrapper->channels[i].sar_cdev.sensors_enable_wakeup = stk_cdev_sensors_enable_wakeup;
        stk_wrapper->channels[i].sar_cdev.sensors_flush = stk_cdev_sensors_flush;
        err = sensors_classdev_register(&stk_wrapper->channels[i].input_dev->dev, &stk_wrapper->channels[i].sar_cdev);
        if (err < 0)
            STK_ERR("create %d cap sensor_class file failed (%d)\n", i, err);
#endif
        STK_LOG("register ch[%d] Done", i);
    }

    return 0;
}

/*
 * @brief: Exit mtk related settings safely.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_exit_mtk(struct stk501xx_wrapper *stk_wrapper)
{
    int8_t i = 0;

    for (i = 0; i < global_stk->pdata->ch_num; i++)
    {
#ifdef STK_SENSORS_DEV
        sensors_classdev_unregister(&stk_wrapper->channels[i].sar_cdev);
#endif // STK_SENSORS_DEV
        sysfs_remove_group(&stk_wrapper->channels[i].input_dev->dev.kobj, &stk501xx_attr_group);
        input_unregister_device(stk_wrapper->channels[i].input_dev);
        input_free_device(stk_wrapper->channels[i].input_dev);
    }
    class_unregister(&capsense_class);
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
static int32_t stk_init_mtk(stk501xx_wrapper *stk_wrapper)
{
    int32_t err = 0;
    uint8_t i = 0;
    /*Create fsys class*/
    err = class_register(&capsense_class);

    if (err)
    {
        STK_ERR("Create fsys class, err=%d", err);
        return err;
    }

    err = stk_input_setup(stk_wrapper);

    if (err)
    {
        return -1;
    }

    /* sysfs: create file system */
    for (i = 0; i < global_stk->pdata->ch_num; i++)
    {
        err = sysfs_create_group(&stk_wrapper->channels[i].input_dev->dev.kobj,
                                 &stk501xx_attr_group);
        if (err)
        {
            STK_ERR("Create fsys class[%d], err=%d", i, err);
            goto err_remove_attr;
        }
        stk_wrapper->channels[i].enabled = false;
    }

    return 0;

err_remove_attr:
    stk_exit_mtk(stk_wrapper);
    return -1;
}

void stk_report_sar_data(struct stk_data* stk)
{
    stk501xx_wrapper *stk_wrapper = container_of(stk, stk501xx_wrapper, stk);
    int32_t i = 0;
    uint8_t is_change = 0;
    uint8_t nf_flag = 0;
    uint32_t* mapping_phase = stk->pdata->mapping_phase;

    if (!stk_wrapper->channels[i].input_dev)
    {
        STK_ERR("No input device for sar data");
        return;
    }

    for (i = 0; i < stk->pdata->ch_num; i ++)
    {
        nf_flag = is_change = stk->state_change[mapping_phase[i]];
        STK_DBG("stk_report_sar_data:: change ph[%d] =%d,(%d)", i, stk->state_change[mapping_phase[i]], is_change);

        //near to far dist
        if(stk->last_nearby[mapping_phase[i]] & (1 << 3))
        {
            nf_flag = STK_SAR_NEAR_BY_DIST_3;
        }
        else if(stk->last_nearby[mapping_phase[i]] & (1 << 2))
        {
            nf_flag = STK_SAR_NEAR_BY_DIST_2;
        }
        else if(stk->last_nearby[mapping_phase[i]] & (1 << 1))
        {
            nf_flag = STK_SAR_NEAR_BY_DIST_1;
        }
        else if(stk->last_nearby[mapping_phase[i]] & (1))
        {
            nf_flag = STK_SAR_NEAR_BY_DIST_0;
            if( (stk->dist1_en!=0) && ((stk->dist1_en & (1 << mapping_phase[i])) == 0))
                nf_flag = STK_SAR_NEAR_BY_DIST_1;
        }
        else
        {
            nf_flag = STK_SAR_FAR_AWAY;
        }

        if (is_change != 0)
        {
            input_report_abs(stk_wrapper->channels[i].input_dev, ABS_DISTANCE, nf_flag);
            input_sync(stk_wrapper->channels[i].input_dev);
        }
    }
}

void stk501xx_parse_dt(struct stk_data* stk, struct device *dev)
{
#if defined STK_INTERRUPT_MODE || defined STK_VALUE_BY_DTS
    int ret;
#endif
    int i;
    int rxio_map[][2] = STK_RXIO_MAP;
#ifdef STK_INTERRUPT_MODE

    stk->gpio_info.int_pin = 0;
    ret = of_property_read_u32_array(dev->of_node, "interrupts", &stk->pdata->interrupt_int1_pin, 1);
    if(ret == 0)
    {
        STK_LOG("interrupts = %d", stk->pdata->interrupt_int1_pin);
        stk->gpio_info.int_pin = stk->pdata->interrupt_int1_pin;
        stk->gpio_info.irq = irq_of_parse_and_map(dev->of_node, 0);
        STK_LOG("irq #=%d, interrupt pin=%d", stk->gpio_info.irq, stk->gpio_info.int_pin);
    }
    else
    {
        STK_ERR("interrupts read fail");
    }

#endif // STK_INTERRUPT_MODE

#ifndef STK_VALUE_BY_DTS
    stk->pdata->ch_num          = CHANNEL_NUM;
    stk->major_phase            = MAJOR_PHASE;
    stk->ref_phase              = REF_PHASE;
    stk->dist1_en               = STK_DIST_1_EN;
    stk->phase_en               = (stk->major_phase | stk->ref_phase);
    stk->pdata->replace_reg_num = 0;
    for (i = 0; i < 8; i++)
    {
        stk->pdata->rxio_map[i].phase_idx = rxio_map[i][0];
        stk->pdata->rxio_map[i].phase_usage = rxio_map[i][1];
        STK_LOG("[%d]rxio_%d: ph=%d, usage=%d", i, stk->pdata->rxio_map[i].phase_idx, stk->pdata->rxio_map[i].phase_usage);
    }

    for (i = 0; i < 8; i++)
    {
        stk->pdata->mapping_phase[i] = i;

        // dist0 & dist1 thd
        switch (i)
        {
            default:
            case 0:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_0;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_0;
                break;
            case 1:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_1;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_1;
                break;
            case 2:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_2;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_2;
                break;
            case 3:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_3;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_3;
                break;
            case 4:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_4;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_4;
                break;
            case 5:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_5;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_5;
                break;
            case 6:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_6;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_6;
                break;
            case 7:
                stk->pdata->dist0_thd[i] = STK_DIST0_THD_7;
                stk->pdata->dist1_thd[i] = STK_DIST1_THD_7;
                break;
            }
    }
#else
    ret = of_property_read_u32(dev->of_node,"ch_num",&stk->pdata->ch_num);
    if (ret == 0)
    {
        // Success
        STK_LOG("ch_num = %d", stk->pdata->ch_num);
    }
    else
    {
        stk->pdata->ch_num = CHANNEL_NUM;
        STK_ERR("Failed to read property 'ch_num', set=%d",CHANNEL_NUM);
    }

    stk->major_phase = 0;
    ret = of_property_read_u32_array(dev->of_node, "major_phase", &stk->pdata->major_phase_arr[0], 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            if(stk->pdata->major_phase_arr[i] == 1)
                stk->major_phase |= (1<<i);
        }
        STK_LOG("major_phase=0x%x", stk->major_phase);
    }
    else
    {
        stk->major_phase = MAJOR_PHASE;
        STK_ERR("Failed to read property 'major_phase', using 'MAJOR_PHASE'=0%x", MAJOR_PHASE);
    }

    stk->ref_phase = 0;
    ret = of_property_read_u32_array(dev->of_node, "ref_phase", &stk->pdata->ref_phase_arr[0], 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            if(stk->pdata->ref_phase_arr[i] == 1)
                stk->ref_phase |= (1<<i);
        }
        STK_LOG("ref_phase=0x%x", stk->ref_phase);
    }
    else
    {
        stk->ref_phase = REF_PHASE;
        STK_ERR("Failed to read property 'ref_phase', using 'REF_PHASE'=0%x", REF_PHASE);
    }
    stk->phase_en = (stk->major_phase | stk->ref_phase);

    //report channel map
    ret = of_property_read_u32_array(dev->of_node,"mapping_phase", (u32*)&(stk->pdata->mapping_phase[0]), 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            STK_LOG("mapping_phase[%d] = %d", i, stk->pdata->mapping_phase[i]);
        }
    }
    else
    {
        STK_ERR("Failed to read property 'mapping_phase'");

        for (i = 0; i < 8; i++)
        {
            stk->pdata->mapping_phase[i] = i;
        }
    }

    // dts threshold
    ret = of_property_read_u32_array(dev->of_node,"dist0_thd", (u32*)&(stk->pdata->dist0_thd[0]), 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            STK_LOG("sar_thd0_[%d] = %d", i, stk->pdata->dist0_thd[i]);
        }
    }
    else
    {
        STK_ERR("Failed to read property 'dist0_thd', using 'STK_SAR_THD_X");
        for (i = 0; i < 8; i++)
        {
            switch (i)
            {
                default:
                case 0:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_0;
                    break;
                case 1:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_1;
                    break;
                case 2:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_2;
                    break;
                case 3:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_3;
                    break;
                case 4:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_4;
                    break;
                case 5:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_5;
                    break;
                case 6:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_6;
                    break;
                case 7:
                    stk->pdata->dist0_thd[i] = STK_DIST0_THD_7;
                    break;
            }
        }
    }

    ret = of_property_read_u32_array(dev->of_node,"dist1_thd", (u32*)&(stk->pdata->dist1_thd[0]), 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            STK_LOG("sar_thd1_[%d] = %d", i, stk->pdata->dist1_thd[i]);
        }
    }
    else
    {
        STK_ERR("Failed to read property 'dist1_thd', using 'STK_SAR_THD1_X");
        for (i = 0; i < 8; i++)
        {
            switch (i)
            {
                default:
                case 0:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_0;
                    break;
                case 1:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_1;
                    break;
                case 2:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_2;
                    break;
                case 3:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_3;
                    break;
                case 4:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_4;
                    break;
                case 5:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_5;
                    break;
                case 6:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_6;
                    break;
                case 7:
                    stk->pdata->dist1_thd[i] = STK_DIST1_THD_7;
                    break;
            }
        }
    }

    stk->dist1_en = 0;
    ret = of_property_read_u32_array(dev->of_node, "dist1_en", &stk->pdata->dist1_en_arr[0], 8);
    if (ret == 0)
    {
        // Success
        for (i = 0; i < 8; i++)
        {
            if(stk->pdata->dist1_en_arr[i] == 1)
                stk->dist1_en |= (1<<i);
        }
        STK_LOG("dist1_en=0x%x", stk->dist1_en);
    }
    else
    {
        stk->dist1_en = STK_DIST_1_EN;
        STK_ERR("Failed to read property 'dist1_en', using 'STK_DIST_1_EN'=0%x", STK_DIST_1_EN);
    }

    //TEMP_COMPENSATION config
    ret = of_property_read_u32_array(dev->of_node,"tc_config_a", (u32*)&(stk->pdata->tc_config_a[0]), 6);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_config_a  temp_delta=%d, map_ph=%d, meas_ph=%d, base_reinit_ph_1 = %d, base_reinit_ph_2 = %d, base_reinit_ph_3 = %d",
            stk->pdata->tc_config_a[0], stk->pdata->tc_config_a[1], stk->pdata->tc_config_a[2], stk->pdata->tc_config_a[3], stk->pdata->tc_config_a[4], stk->pdata->tc_config_a[5]);
        stk->tc_config_a.delta_temp_thd    = stk->pdata->tc_config_a[0];
        stk->tc_config_a.mapping_ref_phase = stk->pdata->tc_config_a[1];
        stk->tc_config_a.major_phase       = stk->pdata->tc_config_a[2];
        stk->tc_config_a.reinit_phase[0]       = stk->pdata->tc_config_a[3];
        stk->tc_config_a.reinit_phase[1]       = stk->pdata->tc_config_a[4];
        stk->tc_config_a.reinit_phase[2]       = stk->pdata->tc_config_a[5];
        stk->tc_config_a.mapping_ref_phase_reg = MAPING_REF_PHASE(stk->tc_config_a.mapping_ref_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_config_a'");
    }

    ret = of_property_read_u32_array(dev->of_node,"tc_config_b", (u32*)&(stk->pdata->tc_config_b[0]), 6);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_config_b  temp_delta=%d, map_ph=%d, meas_ph=%d, base_reinit_ph_1 = %d, base_reinit_ph_2 = %d, base_reinit_ph_3 = %d",
            stk->pdata->tc_config_b[0], stk->pdata->tc_config_b[1], stk->pdata->tc_config_b[2], stk->pdata->tc_config_b[3], stk->pdata->tc_config_b[4], stk->pdata->tc_config_b[5]);
        stk->tc_config_b.delta_temp_thd    = stk->pdata->tc_config_b[0];
        stk->tc_config_b.mapping_ref_phase = stk->pdata->tc_config_b[1];
        stk->tc_config_b.major_phase       = stk->pdata->tc_config_b[2];
        stk->tc_config_b.reinit_phase[0]       = stk->pdata->tc_config_b[3];
        stk->tc_config_b.reinit_phase[1]       = stk->pdata->tc_config_b[4];
        stk->tc_config_b.reinit_phase[2]       = stk->pdata->tc_config_b[5];
        stk->tc_config_b.mapping_ref_phase_reg = MAPING_REF_PHASE(stk->tc_config_b.mapping_ref_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_config_b'");
    }

    ret = of_property_read_u32_array(dev->of_node,"tc_config_c", (u32*)&(stk->pdata->tc_config_c[0]), 6);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_config_c  temp_delta=%d, map_ph=%d, meas_ph=%d, base_reinit_ph_1 = %d, base_reinit_ph_2 = %d, base_reinit_ph_3 = %d",
            stk->pdata->tc_config_c[0], stk->pdata->tc_config_c[1], stk->pdata->tc_config_c[2], stk->pdata->tc_config_c[3], stk->pdata->tc_config_c[4], stk->pdata->tc_config_c[5]);
        stk->tc_config_c.delta_temp_thd    = stk->pdata->tc_config_c[0];
        stk->tc_config_c.mapping_ref_phase = stk->pdata->tc_config_c[1];
        stk->tc_config_c.major_phase       = stk->pdata->tc_config_c[2];
        stk->tc_config_c.reinit_phase[0]       = stk->pdata->tc_config_c[3];
        stk->tc_config_c.reinit_phase[1]       = stk->pdata->tc_config_c[4];
        stk->tc_config_c.reinit_phase[2]       = stk->pdata->tc_config_c[5];
        stk->tc_config_c.mapping_ref_phase_reg = MAPING_REF_PHASE(stk->tc_config_c.mapping_ref_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_config_c'");
    }

    //TEMP_COMPENSATION delta des config
    ret = of_property_read_u32_array(dev->of_node,"tc_ctrl_des_a", (u32*)&(stk->pdata->tc_ctrl_des_a[0]), 3);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_ctrl_des_a  delta_ctrl_thd=%d, delta_deb_clr=%d, delta_deb_set=%d",
            stk->pdata->tc_ctrl_des_a[0], stk->pdata->tc_ctrl_des_a[1], stk->pdata->tc_ctrl_des_a[2]);

        stk->tc_config_a.delta_des_val = DELTA_DES_CTRL_VAL(stk->pdata->tc_ctrl_des_a[0], stk->pdata->tc_ctrl_des_a[1],
                                                            stk->pdata->tc_ctrl_des_a[2], stk->tc_config_a.major_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_ctrl_des_a'");
    }

    ret = of_property_read_u32_array(dev->of_node,"tc_ctrl_des_b", (u32*)&(stk->pdata->tc_ctrl_des_b[0]), 3);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_ctrl_des_b  delta_ctrl_thd=%d, delta_deb_clr=%d, delta_deb_set=%d",
            stk->pdata->tc_ctrl_des_b[0], stk->pdata->tc_ctrl_des_b[1], stk->pdata->tc_ctrl_des_b[2]);

        stk->tc_config_b.delta_des_val = DELTA_DES_CTRL_VAL(stk->pdata->tc_ctrl_des_b[0], stk->pdata->tc_ctrl_des_b[1],
                                                            stk->pdata->tc_ctrl_des_b[2], stk->tc_config_b.major_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_ctrl_des_b'");
    }

    ret = of_property_read_u32_array(dev->of_node,"tc_ctrl_des_c", (u32*)&(stk->pdata->tc_ctrl_des_c[0]), 3);
    if (ret == 0)
    {
        // Success
        STK_LOG("tc_ctrl_des_c  delta_ctrl_thd=%d, delta_deb_clr=%d, delta_deb_set=%d",
            stk->pdata->tc_ctrl_des_c[0], stk->pdata->tc_ctrl_des_c[1], stk->pdata->tc_ctrl_des_c[2]);

        stk->tc_config_c.delta_des_val = DELTA_DES_CTRL_VAL(stk->pdata->tc_ctrl_des_c[0], stk->pdata->tc_ctrl_des_c[1],
                                                            stk->pdata->tc_ctrl_des_c[2], stk->tc_config_c.major_phase);
    }
    else
    {
        STK_ERR("Failed to read property 'tc_ctrl_des_c'");
    }

    // RXIO map
    ret = of_property_read_u32_array(dev->of_node, "rxio_map", (u32*)&(stk->pdata->rxio_map[0]), sizeof(struct stk501xx_rxio_map)*8/sizeof(uint32_t));
    if(ret == 0)
    {
        for (i = 0; i < 8; i++)
        {
            STK_LOG("rxio_[%d]: ph=%d, usage=%d", i, stk->pdata->rxio_map[i].phase_idx, stk->pdata->rxio_map[i].phase_usage);
        }
    }
    else
    {
        STK_ERR("Failed to read property 'rxio_map', using 'STK_RXIO_MAP'");
        for (i = 0; i < 8; i++)
        {
            stk->pdata->rxio_map[i].phase_idx = rxio_map[i][0];
            stk->pdata->rxio_map[i].phase_usage = rxio_map[i][1];
            STK_LOG("rxio_%d: ph=%d, usage=%d", i, stk->pdata->rxio_map[i].phase_idx, stk->pdata->rxio_map[i].phase_usage);
        }
    }

    // load in registers
    ret = of_property_read_u32(dev->of_node,"replace_reg_num",&stk->pdata->replace_reg_num);
    if (ret == 0)
    {
        // Success
        STK_LOG("replace_reg_num = %d", stk->pdata->replace_reg_num);
    }
    else
    {
        stk->pdata->replace_reg_num = 0;
        STK_ERR("Failed to read property 'replace_reg_num'");
    }

    if (stk->pdata->replace_reg_num > 0)
    {
        // initialize platform reg data array
        stk->pdata->replace_reg_val = devm_kzalloc(dev,sizeof(struct stk501xx_register_table)*stk->pdata->replace_reg_num, GFP_KERNEL);
        if (unlikely(stk->pdata->replace_reg_val == NULL))
        {
            STK_ERR("replace_reg_num alloc size [%d] erro", stk->pdata->replace_reg_num);
            return;
        }

        // read array
        ret = of_property_read_u32_array(dev->of_node, "replace_reg_val", (u32*)&(stk->pdata->replace_reg_val[0]), sizeof(struct stk501xx_register_table)*stk->pdata->replace_reg_num/sizeof(u32));
        if(ret == 0)
        {
#if 0
            for (i = 0; i < stk->pdata->replace_reg_num; i++)
            {
                STK_LOG("[%d]replace reg[0x%x] = 0x%x", i, stk->pdata->replace_reg_val[i].address, stk->pdata->replace_reg_val[i].value);
            }
#endif
        }
    }
#endif // STK_VALUE_BY_DTS
    return;
}

#ifdef CONFIG_CAPSENSE_USB_CAL
static void ps_notify_callback_work(struct work_struct *work)
{
    uint32_t val = STK_TRIGGER_REG_INIT_ALL(global_stk->phase_en);
    STK_LOG("class_stk_phase_USB_cali , reset all phase\n");
    stk501xx_phase_reset(global_stk, val);
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
        !strncmp(psy->desc->name, USB_POWER_SUPPLY_NAME, sizeof(USB_POWER_SUPPLY_NAME)) && data) {
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
static void write_flip_regs(int num_regs, struct stk501xx_register_table *regs)
{
	int i;

	for(i=0; i < num_regs; i++)
	{
        /* Write all registers/values contained in i2c_reg */
        STK_LOG("Going to Write Reg from dts: 0x%x Value: 0x%x\n",
                regs[i].address, regs[i].value);
        STK_REG_WRITE(global_stk, regs[i].address, (uint8_t*)&regs[i].value);
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

/**
 * @brief: Probe function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] stk_bus_ops: const struct stk_bus_ops *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
int32_t stk_i2c_probe(struct i2c_client *client, struct common_function *common_fn)
{
    int32_t err = 0;
    stk501xx_wrapper *stk_wrapper;
    struct stk_data *stk;
#ifdef CONFIG_CAPSENSE_USB_CAL
    struct power_supply *psy = NULL;
#endif

    STK_LOG("STK_HEADER_VERSION: %s ", STK_HEADER_VERSION);
    STK_LOG("STK_C_VERSION: %s ", STK_C_VERSION);
    STK_LOG("STK_DRV_I2C_VERSION: %s ", STK_DRV_I2C_VERSION);
    STK_LOG("STK_MTK_VERSION: %s ", STK_MTK_VERSION);

    if (NULL == client)
    {
        return -ENOMEM;
    }
    else if (!common_fn)
    {
        STK_ERR("cannot get common function. EXIT");
        return -EIO;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
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

    gStk = stk;
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

    stk501xx_data_initialize(stk);
    stk501xx_parse_dt(stk, &client->dev);
    err = stk501xx_init_client(stk);

    if (err < 0)
    {
        STK_ERR("stk501xx_init_client failed");
        goto err_exit;
    }

    if (stk_init_mtk(stk_wrapper))
    {
        STK_ERR("stk_init_mtk failed");
        goto err_free_mem;
    }

    stk_report_sar_always_far_data(stk, -1);

#ifdef CONFIG_CAPSENSE_USB_CAL
        /*notify usb state*/
        INIT_WORK(&stk_wrapper->ps_notify_work, ps_notify_callback_work);
        stk_wrapper->ps_notif.notifier_call = ps_notify_callback;
        err = power_supply_reg_notifier(&stk_wrapper->ps_notif);
        if (err)
            STK_ERR("Unable to register ps_notifier: %d\n", err);

        psy = power_supply_get_by_name(USB_POWER_SUPPLY_NAME);
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
    stk_init_flag = 0;
    return 0;

err_exit:
#ifdef STK_INTERRUPT_MODE
    STK_GPIO_IRQ_REMOVE(stk, &stk->gpio_info);
#elif defined STK_POLLING_MODE
    STK_TIMER_REMOVE(stk, &stk->stk_timer_info);
    STK_LOG("Success");
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
err_free_mem:
    mutex_destroy(&stk_wrapper->i2c_mgr.lock);
    kfree(stk_wrapper);
    stk_init_flag = -1;

    return err;
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
int32_t stk_i2c_remove(struct i2c_client *client)
{
    stk501xx_wrapper *stk_wrapper = i2c_get_clientdata(client);
    struct stk_data *stk = &stk_wrapper->stk;

#ifdef CONFIG_CAPSENSE_USB_CAL
    cancel_work_sync(&stk_wrapper->ps_notify_work);
    power_supply_unreg_notifier(&stk_wrapper->ps_notif);
#endif

    stk_exit_mtk(stk_wrapper);
#ifdef STK_INTERRUPT_MODE
    STK_GPIO_IRQ_REMOVE(stk, &stk->gpio_info);
#elif defined STK_POLLING_MODE
    STK_TIMER_REMOVE(stk, &stk->stk_timer_info);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    stk->bops->remove(&stk_wrapper->i2c_mgr);
    mutex_destroy(&stk_wrapper->i2c_mgr.lock);
    kfree(stk_wrapper);
    stk_init_flag = -1;

    return 0;
}

int32_t stk501xx_suspend(struct device* dev)
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

int32_t stk501xx_resume(struct device* dev)
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
    { .compatible = "mediatek,stk501xx", },
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
static int32_t stk501xx_i2c_probe(struct i2c_client* client,
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void stk501xx_i2c_remove(struct i2c_client *client)
#else
static int32_t stk501xx_i2c_remove(struct i2c_client* client)
#endif
{
//    return stk_i2c_remove(client);
    stk501xx_wrapper *stk_wrapper = i2c_get_clientdata(client);
    struct stk_data *stk = &stk_wrapper->stk;

    stk_exit_mtk(stk_wrapper);
    stk->bops->remove(&stk_wrapper->i2c_mgr);
    mutex_destroy(&stk_wrapper->i2c_mgr.lock);
    kfree(stk_wrapper);
    stk_init_flag = -1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#else
    return 0;
#endif
}

/**
 * @brief:
 */
static int32_t stk501xx_i2c_detect(struct i2c_client* client, struct i2c_board_info* info)
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
static int32_t stk501xx_i2c_suspend(struct device* dev)
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
static int32_t stk501xx_i2c_resume(struct device* dev)
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
#ifdef CONFIG_ACPI
        .acpi_match_table = ACPI_PTR(stk501xx_acpi_id),
#endif /* CONFIG_ACPI */
#ifdef CONFIG_OF
        .of_match_table = stk501xx_match_table,
#endif /* CONFIG_OF */
    }
};

module_i2c_driver(stk501xx_i2c_driver);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk501xx sar driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(STK_MTK_VERSION);
