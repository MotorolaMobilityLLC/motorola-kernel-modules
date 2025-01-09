/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>
#include "eswin_eph861x.h"
#include "eswin_eph861x_types.h"
#include "eswin_eph861x_comms.h"
#include "eswin_ts_mmi.h"
#include "eswin_eph861x_tlv_command.h"
#include "eswin_eph861x_eswin.h"
#include "eswin_eph861x_tlv_report.h"
#include "eswin_ts_config.h"

#define GET_ESWIN_DATA(dev) { \
    ephdata = (struct eph_data*)dev_get_drvdata(dev); \
    if (!ephdata) { \
        ts_err("Failed to get driver data"); \
        return -ENODEV; \
    } \
}
#define ESWIN_EXTRA_MMI
#ifdef ESWIN_EXTRA_MMI

static ssize_t eswin_ts_edge_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);
static ssize_t eswin_ts_edge_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t eswin_ts_interpolation_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);
static ssize_t eswin_ts_interpolation_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t eswin_ts_sample_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);
static ssize_t eswin_ts_sample_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t eswin_ts_stowed_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);
static ssize_t eswin_ts_stowed_show(struct device *dev,
        struct device_attribute *attr, char *buf);
#ifdef CONFIG_ESWIN_LAST_TIME
static ssize_t eswin_ts_timestamp_show(struct device *dev,
        struct device_attribute *attr, char *buf);
#endif
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
static ssize_t eswin_ts_log_trigger_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
static ssize_t eswin_ts_log_trigger_show(struct device *dev,
        struct device_attribute *attr, char *buf);
#endif
#ifdef CONFIG_ENABLE_ESWIN_VIRTUAL_FOD
static ssize_t eswin_ts_fp_event_show(struct device *dev,
        struct device_attribute *attr, char *buf);
#endif

static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_edge_show, eswin_ts_edge_store);
static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_interpolation_show, eswin_ts_interpolation_store);
static DEVICE_ATTR(sample, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_sample_show, eswin_ts_sample_store);
static DEVICE_ATTR(stowed, (S_IWUSR | S_IWGRP | S_IRUGO),
    eswin_ts_stowed_show, eswin_ts_stowed_store);
#ifdef CONFIG_ESWIN_LAST_TIME
static DEVICE_ATTR(timestamp, S_IRUGO, eswin_ts_timestamp_show, NULL);
#endif
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
static DEVICE_ATTR(log_trigger, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_log_trigger_show, eswin_ts_log_trigger_store);
#endif

#ifdef CONFIG_ESWIN_POCKET_MODE
static ssize_t eswin_ts_pocket_mode_show(struct device *dev,
    struct device_attribute *attr, char *buf);
static ssize_t eswin_ts_pocket_mode_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(pocket_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_pocket_mode_show, eswin_ts_pocket_mode_store);
#endif

#ifdef CONFIG_ENABLE_ESWIN_VIRTUAL_FOD
static DEVICE_ATTR(fp_event, (S_IRUGO | S_IWUSR | S_IWGRP),
    eswin_ts_fp_event_show, NULL);
#endif
/* hal settings */
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define SMALL_MODE    2
#define DEFAULT_MODE   0
#define MAX_ATTRS_ENTRIES 10

#define NORMAL_DEFAULT_MODE 10
#define NORMAL_SMALL_MODE 11
#define NORMAL_BIG_MODE 12

/* eswin command setting */
#define REPORT_RATE_CMD_120HZ   0x0
#define REPORT_RATE_CMD_240HZ   0x1
#define REPORT_RATE_CMD_360HZ   0x2
#define REPORT_RATE_CMD_480HZ   0x3
#define REPORT_RATE_CMD_576HZ   0x4
#define REPORT_RATE_CMD_720HZ   0x5

#define CMD_HOST_DISCRETE               12u
#define CMD_OFFSET_CHARGER              2u
#define CMD_OFFSET_STOWED               3u
#define CMD_OFFSET_ARMED_RATE           5u
#define CMD_OFFSET_FOD                  7u
#define CMD_OFFSET_SENSITIVITY          8u
#define CMD_OFFSET_ORIENTATION          9u
#define CMD_OFFSET_EDGE_GRIP            10u
#define CMD_OFFSET_ACTIVE_RATE          11u
#define CMD_OFFSET_POCKET               14u

#define CMD_BIT_ORIENTATION_0           0u
#define CMD_BIT_ORIENTATION_90          1u
#define CMD_BIT_ORIENTATION_180         2u
#define CMD_BIT_ORIENTATION_270         3u
#define CMD_BIT_EDGE_GRIP_DEFAULT       0u
#define CMD_BIT_EDGE_GRIP_SMALL         1u
#define CMD_BIT_EDGE_GRIP_BIG           2u
#define CMD_BIT_EDGE_GRIP_NORM_DEFAULT  3u
#define CMD_BIT_EDGE_GRIP_NORM_SMALL    4u
#define CMD_BIT_EDGE_GRIP_NORM_BIG      5u

#define ADD_ATTR(name) { \
    if (idx < MAX_ATTRS_ENTRIES)  { \
        ts_info("%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
        ext_attributes[idx] = &dev_attr_##name.attr; \
        idx++; \
    } else { \
        ts_err("%s: cannot add attribute '%s'\n", __func__, #name); \
    } \
}

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
    .attrs = ext_attributes,
};

static int eswin_ts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
    int idx = 0;
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);

    if (ephdata->ephplatform->edge_ctrl)
        ADD_ATTR(edge);

    if (ephdata->ephplatform->interpolation_ctrl)
        ADD_ATTR(interpolation);

    if (ephdata->ephplatform->sample_ctrl)
        ADD_ATTR(sample);

    if (ephdata->ephplatform->stowed_mode_ctrl)
        ADD_ATTR(stowed);
#ifdef CONFIG_ESWIN_POCKET_MODE
    if (ephdata->ephplatform->pocket_mode_ctrl)
        ADD_ATTR(pocket_mode);
#endif
#ifdef CONFIG_ENABLE_ESWIN_VIRTUAL_FOD
    ADD_ATTR(fp_event);
#endif
#ifdef CONFIG_ESWIN_LAST_TIME
    ADD_ATTR(timestamp);
#endif
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
    ADD_ATTR(log_trigger);
#endif

    if (idx) {
        ext_attributes[idx] = NULL;
        *group = &ext_attr_group;
    } else
        *group = NULL;

    return 0;
}

static int eswin_ts_send_cmd(struct eph_data *ephdata,
        u16 cmd_id, u16 cmd_offset, u8 cmd_value_1, u8 cmd_value_2)
{
    int ret = 0;
    u16 write_buffer_length = TLV_WRITE_HEADER_SIZE + 1u;;
    u8 cmd[9]={0};
    u8 cmd_length = 8;

    if (0xFF != cmd_value_2)
    {
        write_buffer_length++;
        cmd_length++;
    }

    cmd[TLV_TYPE_FIELD] = TLV_CONTROL_DATA_WRITE;
    cmd[TLV_LENGTH_FIELD] = write_buffer_length & 0xFF;
    cmd[TLV_LENGTH_FIELD + 1] = (write_buffer_length >> 8) & 0xFF;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_COMPONENT_FIELD] = cmd_id & 0xFF;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_COMPONENT_FIELD + 1] = (cmd_id >> 8) & 0xFF;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_OFFSET_FIELD] = cmd_offset & 0xFF;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_OFFSET_FIELD + 1] = (cmd_offset >> 8) & 0xFF;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_HEADER_SIZE] = cmd_value_1;
    cmd[TLV_PAYLOAD_FIELD + TLV_WRITE_HEADER_SIZE + 1] = cmd_value_2;
    mutex_lock(&ephdata->comms_mutex);
    ret = eph_write_control_config(ephdata, cmd_length , cmd);
    mutex_unlock(&ephdata->comms_mutex);
    if (ret)
    {
        ts_err("eph_ts_send_cmd failed offset[%d]\n", cmd_offset);
    }

    return ret;
}

static int eswin_ts_send_report_cmd(struct eph_data *ephdata, int report_mode)
{
    int ret = 0;
    u16 cmd_report_rate;

    switch (report_mode) {
        case REPORT_RATE_CMD_720HZ:
            cmd_report_rate = 720;
            break;
        case REPORT_RATE_CMD_576HZ:
            cmd_report_rate = 576;
            break;
        case REPORT_RATE_CMD_480HZ:
            cmd_report_rate = 480;
            break;
        case REPORT_RATE_CMD_360HZ:
            cmd_report_rate = 360;
            break;
        case REPORT_RATE_CMD_240HZ:
            cmd_report_rate = 240;
            break;
        case REPORT_RATE_CMD_120HZ:
        default:
            cmd_report_rate = 120;
            break;
    }
    ts_info("report_mode %d, cmd_report_rate %d\n", report_mode, cmd_report_rate);
    ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_ACTIVE_RATE, (cmd_report_rate & 0xff), ((cmd_report_rate >> 8) & 0xff));
    return ret;
}

static int eswin_ts_mmi_set_report_rate(struct eph_data *ephdata)
{
    int ret = 0;
    int mode = 0;
   /* TODO: implement get report rate */
    mode = eswin_ts_mmi_get_report_rate(ephdata);
    if (mode == -1) {
        return -EINVAL;
    }

    ephdata->get_mode.report_rate_mode = mode;
    if (ephdata->set_mode.report_rate_mode == mode) {
        ts_info("The value = %d is same, so not to write", mode);
        return 0;
    }

    if (ephdata->power_on == 0) {
        ts_info("The touch is in sleep state, restore the value when resume\n");
        return 0;
    }

    //send switch command
    ret = eswin_ts_send_report_cmd(ephdata, mode);
    if (ret < 0) {
        ts_err("failed to set report rate, mode = %d", mode);
        return -EINVAL;
    }
    msleep(20);

    ephdata->set_mode.report_rate_mode = mode;

    ts_info("Success to set %s\n", mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120HZ" :
                (mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
                (mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_360HZ" :
                (mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
                (mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
                (mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
                ("Unsupported")))))));
    return ret;
}

static ssize_t eswin_ts_interpolation_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    unsigned long mode = 0;
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ret = kstrtoul(buf, 0, &mode);
    if (ret < 0) {
        ts_info("Failed to convert value.\n");
        return -EINVAL;
    }

    mutex_lock(&ephdata->mode_lock);
    ephdata->get_mode.interpolation = mode;
    /* TODO: implement set report rate */
    ret = eswin_ts_mmi_set_report_rate(ephdata);
    if (ret < 0)
        goto exit;

    ret = size;
    ephdata->set_mode.interpolation = mode;
exit:
    mutex_unlock(&ephdata->mode_lock);
    return ret;
}

static ssize_t eswin_ts_interpolation_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ts_info("interpolation = %d.\n", ephdata->set_mode.interpolation);
    return scnprintf(buf, PAGE_SIZE, "0x%02x", ephdata->set_mode.interpolation);
}

static ssize_t eswin_ts_sample_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    unsigned long mode = 0;
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ret = kstrtoul(buf, 0, &mode);
    if (ret < 0) {
        ts_info("Failed to convert value.\n");
        return -EINVAL;
    }

    mutex_lock(&ephdata->mode_lock);
    ephdata->get_mode.sample= mode;
    if (ephdata->set_mode.sample == mode) {
        ts_info("The value = %lu is same, so not to write", mode);
        ret = size;
        goto exit;
    }

    if (ephdata->power_on == 0) {
        ts_info("The touch is in sleep state, restore the value when resume\n");
        ret = size;
        goto exit;
    }

    ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_ARMED_RATE,
                        ephdata->get_mode.sample, 0xff);
    if (ret < 0) {
        ts_err("failed to set sample rate, mode = %lu", mode);
        goto exit;
    }

    ephdata->set_mode.sample = mode;
    msleep(20);
    ts_info("Success to set %lu\n", mode);

    ret = size;
exit:
    mutex_unlock(&ephdata->mode_lock);
    return ret;
}

static ssize_t eswin_ts_sample_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ts_info("sample = %d.\n", ephdata->set_mode.sample);
    return scnprintf(buf, PAGE_SIZE, "0x%02x", ephdata->set_mode.sample);
}

static ssize_t eswin_ts_stowed_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    unsigned long mode = 0;
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ret = kstrtoul(buf, 0, &mode);
    if (ret < 0) {
        ts_info("Failed to convert value.\n");
        return -EINVAL;
    }

    mutex_lock(&ephdata->mode_lock);
    ephdata->get_mode.stowed = mode;
    if (ephdata->set_mode.stowed == mode) {
        ts_info("The value = %lu is same, so not to write", mode);
        ret = size;
        goto exit;
    }

    if ((atomic_read(&ephdata->post_suspended) == 1) && (ephdata->power_on == 1)) {
        //ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_STOWED, ephdata->get_mode.stowed, 0xff);
        if (ret < 0) {
            ts_err("Failed to set stowed mode %lu\n", mode);
            goto exit;
        }
    } else {
        ts_info("Skip stowed mode setting post_suspended:%d, power_on:%d.\n", atomic_read(&ephdata->post_suspended), ephdata->power_on);
        ret = size;
        goto exit;
    }

    ephdata->set_mode.stowed = mode;
    ts_info("Success to set stowed mode %lu\n", mode);

    ret = size;
exit:
    mutex_unlock(&ephdata->mode_lock);
    return ret;
}

static ssize_t eswin_ts_stowed_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ts_info("Stowed state = %d.\n", ephdata->set_mode.stowed);
    return scnprintf(buf, PAGE_SIZE, "0x%02x", ephdata->set_mode.stowed);
}
#ifdef CONFIG_ENABLE_ESWIN_VIRTUAL_FOD
static ssize_t eswin_ts_fp_event_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;
    int idata = 0;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    //idata = atomic_read(&ephdata->fod_event);
    ts_info("fp_event state = 0x%02x.\n", idata);
    return scnprintf(buf, PAGE_SIZE, "0x%02x\n", idata);
}
#endif
static int eswin_ts_mmi_refresh_rate(struct device *dev, int freq)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    ts_info("eswin_ts_mmi_refresh_rate %d\n", freq);
    mutex_lock(&ephdata->mode_lock);
    ephdata->refresh_rate = freq;
    eswin_ts_mmi_set_report_rate(ephdata);
    mutex_unlock(&ephdata->mode_lock);

    return 0;
}

/*
 * HAL: args[0] suppression area, args[1] rotation direction.
 * CMD: [06 17 data0 data1],
 *      data[0] rotation direction, data[1] suppression area.
 */
static ssize_t eswin_ts_edge_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    int edge_cmd[2] = { 0 };
    unsigned int args[2] = { 0 };
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ret = sscanf(buf, "%d %d", &args[0], &args[1]);
    if (ret < 2)
        return -EINVAL;
    ts_info("args[0] %d suppression area, args[1] %d rotation direction", args[0], args[1]);
    /* TODO: fw implement all edge function */
    switch (args[0]) {
    case DEFAULT_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_DEFAULT;
        break;
    case SMALL_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_SMALL;
        break;
    case BIG_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_BIG;
        break;
    case NORMAL_DEFAULT_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_NORM_DEFAULT;
        break;
    case NORMAL_SMALL_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_NORM_SMALL;
        break;
    case NORMAL_BIG_MODE:
        edge_cmd[1] = CMD_BIT_EDGE_GRIP_NORM_BIG;
        break;
    default:
        ts_err("Invalid edge mode: %d!\n", args[0]);
        return -EINVAL;
    }

    if (ROTATE_0 == args[1]) {
        edge_cmd[0] = CMD_BIT_ORIENTATION_0;
    } else if (ROTATE_90 == args[1]) {
        edge_cmd[0] = CMD_BIT_ORIENTATION_90;
    } else if (ROTATE_180 == args[1]) {
        edge_cmd[0] = CMD_BIT_ORIENTATION_180;
    } else if (ROTATE_270 == args[1]) {
        edge_cmd[0] = CMD_BIT_ORIENTATION_270;
    } else {
        ts_err("Invalid rotation mode: %d!\n", args[1]);
        return -EINVAL;
    }

    mutex_lock(&ephdata->mode_lock);
    memcpy(ephdata->get_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
    if (!memcmp(ephdata->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd))) {
        ts_info("The value (%02x %02x) is same,so not write.\n",
                    edge_cmd[0], edge_cmd[1]);
        ret = size;
        goto exit;
    }

    if (ephdata->power_on == 0) {
        ts_info("The touch is in sleep state, restore the value when resume\n");
        ret = size;
        goto exit;
    }

    ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_ORIENTATION, edge_cmd[0], edge_cmd[1]);
    if (ret < 0) {
        ts_err("failed to send edge switch cmd");
        goto exit;
    }

    memcpy(ephdata->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
    msleep(20);
    ret = size;
    ts_info("Success to set edge = %02x, rotation = %02x", edge_cmd[1], edge_cmd[0]);
exit:
    mutex_unlock(&ephdata->mode_lock);
    return ret;
}

static ssize_t eswin_ts_edge_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ts_info("edge area = %02x, rotation = %02x\n",
        ephdata->set_mode.edge_mode[1], ephdata->set_mode.edge_mode[0]);
    return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
        ephdata->set_mode.edge_mode[1], ephdata->set_mode.edge_mode[0]);
}

#ifdef CONFIG_ESWIN_LAST_TIME
static ssize_t eswin_ts_timestamp_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;
    ktime_t last_ktime;
    struct timespec64 last_ts;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    mutex_lock(&ephdata->mode_lock);
    last_ktime = ephdata->last_event_time;
    ephdata->last_event_time = 0;
    mutex_unlock(&ephdata->mode_lock);

    last_ts = ktime_to_timespec64(last_ktime);

    return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
#endif

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
static ssize_t eswin_ts_log_trigger_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    if (!buf || count <= 0)
        return 0;

    clear_kfifo();
    frame_log_capture_start(ephdata);
    return count;
}

static ssize_t eswin_ts_log_trigger_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    return scnprintf(buf, PAGE_SIZE, "0x%02x", 0x01);
}
#endif

#ifdef CONFIG_ESWIN_POCKET_MODE
static ssize_t eswin_ts_pocket_mode_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    ts_info("Pocket mode state = %d.\n", ephdata->set_mode.pocket_mode);
    return scnprintf(buf, PAGE_SIZE, "%d\n", ephdata->set_mode.pocket_mode);
}

static ssize_t eswin_ts_pocket_mode_store(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0;
    unsigned long value = 0;
    struct eph_data *ephdata;

    dev = MMI_DEV_TO_TS_DEV(dev);
    GET_ESWIN_DATA(dev);

    mutex_lock(&ephdata->mode_lock);
    ret = kstrtoul(buf, 0, &value);
    if (ret < 0) {
        ts_err("pocket_mode: Failed to convert value\n");
        mutex_unlock(&ephdata->mode_lock);
        return -EINVAL;
    }
    ts_err("Set pocket_mode: %d\n", value);
    switch (value) {
        case 0x10:
        case 0x20:
            ts_info("touch pocket mode disable\n");
            ephdata->get_mode.pocket_mode = 0;
            break;
        case 0x11:
        case 0x21:
            ts_info("touch pocket mode enable\n");
            ephdata->get_mode.pocket_mode = 1;
            break;
        default:
            ts_info("unsupport pocket mode type, value = %lu\n", value);
            mutex_unlock(&ephdata->mode_lock);
            return -EINVAL;
    }

    if (ephdata->set_mode.pocket_mode == ephdata->get_mode.pocket_mode) {
        ts_info("The value = %d is same, so not to write", ephdata->get_mode.pocket_mode);
        goto exit;
    }

    if (ephdata->power_on == 0) {
        ts_info("The touch is in sleep state, restore the value when resume\n");
        goto exit;
    }

    ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_POCKET,
        ephdata->get_mode.pocket_mode , 0xff);
    if (ret < 0) {
        ts_err("failed to send pocket mode cmd");
        goto exit;
    }

    ephdata->set_mode.pocket_mode = ephdata->get_mode.pocket_mode;
    msleep(20);

    ts_info("Success to %s pocket mode", ephdata->get_mode.pocket_mode ? "Enable" : "Disable");
exit:
    mutex_unlock(&ephdata->mode_lock);
    return size;
}
#endif

#endif//ESWIN_EXTRA_MMI

static int eswin_ts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
    return scnprintf((char*)cdata, TS_MMI_MAX_VENDOR_LEN, "%s", "eswin");
}

static int eswin_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
    struct eph_data *ephdata;
    char ic_info[TS_MMI_MAX_VENDOR_LEN];

    GET_ESWIN_DATA(dev);

    switch(ephdata->ephdeviceinfo.product_id)
    {
        case 0x13:
        strcpy(ic_info,"EPH8611");
        break;
        case 0x23:
        strcpy(ic_info,"EPH8621");
        break;
        default:
        strcpy(ic_info,"UNKNOWE");
        break;
    }
    ts_info("eswin_ts_mmi_methods_get_productinfo > [%d] [%s] <", ephdata->ephdeviceinfo.product_id, ic_info);
    return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ic_info);
}

static int eswin_ts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
    struct eph_data *ephdata;
    char fw_build_version[32];

    GET_ESWIN_DATA(dev);
    snprintf(fw_build_version, 16, "%u-%u",
            ephdata->ephdeviceinfo.application_version_major,
            ephdata->ephdeviceinfo.application_version_minor);
    ts_info("eswin_ts_mmi_methods_get_build_id %s", fw_build_version);

    return scnprintf((char*)(cdata), TS_MMI_MAX_ID_LEN, "%s", fw_build_version);
}

static int eswin_ts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
    int ret;
    struct eph_data *ephdata;
    struct eph_device_control_config_read_command command_request;
    char buf[16] = {0};

    GET_ESWIN_DATA(dev);
    ts_info("eswin_ts_mmi_methods_get_config_id");
    command_request.header.type = 0xB;
    command_request.header.length = 0x6;
    command_request.command_id = 0xC;
    command_request.offset = 0x0;
    command_request.read_length = 0x2;
    if(!ephdata->updating_firmware)
    {
        mutex_lock(&ephdata->comms_mutex);
        ret = eph_read_control_config(ephdata, &command_request, buf);
        mutex_unlock(&ephdata->comms_mutex);
        if (ret) {
            ts_err("failed get fw version data, %d", ret);
            return -EINVAL;
        }
    }
    else
    {
        ts_info("firmware updating, in case aboring, do not read config id");
    }
    return snprintf((char*)(cdata), TS_MMI_MAX_ID_LEN, "%04x", le32_to_cpu(buf[0]));
}

static int eswin_ts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    ts_info("eswin_ts_mmi_methods_get_bus_type %x", ephdata->inputdev->id.bustype);
    TO_INT(idata) = ephdata->inputdev->id.bustype == BUS_I2C ?
            TOUCHSCREEN_MMI_BUS_TYPE_I2C : TOUCHSCREEN_MMI_BUS_TYPE_SPI;
    return 0;
}

static int eswin_ts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    ts_info("eswin_ts_mmi_methods_get_irq_status");
    TO_INT(idata) = eph_read_chg(ephdata);
    return 0;
}

static int eswin_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    ts_info("eswin_ts_mmi_methods_get_drv_irq");
    TO_INT(idata) = atomic_read(&ephdata->irq_enabled);
    return 0;
}

static int eswin_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    TO_INT(idata) = ephdata->power_on;
    return 0;
}

static int eswin_ts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);
    TO_INT(idata) = ephdata->updating_firmware;
    return 0;
}

static int eswin_ts_mmi_methods_drv_irq(struct device *dev, int state)
{
    int ret = -ENODEV;
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    ts_info("%s %d\n", __func__, state);
    ret = eph_irq_enable(ephdata, !(!state));

    return ret;
}

static int eswin_ts_mmi_methods_power(struct device *dev, int on)
{
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    ts_err("%s %d\n", __func__, on);
    if (on == TS_MMI_POWER_ON)
        return eph_power_on(ephdata);
    else if(on == TS_MMI_POWER_OFF)
        return eph_power_off(ephdata);
    else {
        ts_err("Invalid power parameter %d.\n", on);
        return -EINVAL;
    }
    return 0;
}

static int eswin_ts_mmi_charger_mode(struct device *dev, int mode)
{
    int ret = 0;
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);

    mutex_lock(&ephdata->mode_lock);

    ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_CHARGER, !!mode, 0xff);
    if (ret)
    {
        ts_err("failed to set charge mode\n");
    }
    ts_err("Success to %s charge mode\n", mode ? "Enable" : "Disable");

    mutex_unlock(&ephdata->mode_lock);

    return ret;
}

#define Y_MIN_ID 2
#define Y_MAX_ID 3
#define SCREEN_X_MAX 1080
#define SCREEN_Y_MAX 2992
#define SCREEN_EXTENDED 2600
#define SCREEN_PRIM_COMPACT 1980
#define DISP_MODE_REAR 3
#define DISP_MODE_FULL 4
#define DISP_MODE_EXTENDED 0
#define DISP_MODE_PRIM_COMPACT 1
#define DISP_MODE_PRIM_PEEK 2

static int rdArray[4];

static int eswin_ts_mmi_active_region(struct device *dev, int *reg_data)
{
    int dmode = -1;

    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    memcpy(rdArray, reg_data, sizeof(rdArray));
    if (rdArray[Y_MIN_ID] > 0) { /* REAR */
        dmode = DISP_MODE_REAR; /* Mode4 */
    } else { /* FULL or PRIMARY */
        if (rdArray[Y_MAX_ID] == SCREEN_Y_MAX)
            dmode = DISP_MODE_FULL; /* Mode5 */
        else if (rdArray[Y_MAX_ID] > SCREEN_EXTENDED)
            dmode = DISP_MODE_EXTENDED; /* Mode1 */
        else if (rdArray[Y_MAX_ID] > SCREEN_PRIM_COMPACT)
            dmode = DISP_MODE_PRIM_COMPACT; /* Mode2 */
        else
            dmode = DISP_MODE_PRIM_PEEK; /* Mode3 */
    }
    if (dmode >= 0 ) {
        // ret = ephdata->hw_ops->display_mode(ephdata, dmode);
        //TODO: display mode used in eswin!!!
        // if (!ret)
        ts_info("set active region: %d %d %d %d; dmode=%d\n",
                rdArray[0], rdArray[1], rdArray[Y_MIN_ID], rdArray[Y_MAX_ID], dmode);
    } else
        ts_err("Invalid display mode; reqion %d %d %d %d\n",
            rdArray[0], rdArray[1], rdArray[Y_MIN_ID], rdArray[Y_MAX_ID]);

    return 0;
}

static int eswin_ts_mmi_methods_get_active_region(struct device *dev, void *uidata)
{
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    memcpy((int *)uidata, rdArray, sizeof(rdArray));
    return 0;
}

/* reset chip
 * type：can control software reset and hardware reset
 * currently reset use GPIO pull low 
 */
static int eswin_ts_mmi_methods_reset(struct device *dev, int type)
{
    int ret = -ENODEV;

    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    ts_info("%s start gpio_reset \n", __func__);
    eph_reset_device(ephdata);

    return ret;
}

static int eswin_ts_firmware_update(struct device *dev, char *fwname)
{
    int ret;

    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    ret = eph_update_fw(dev, fwname);
    return ret;
}

#ifdef ESWIN_PALM_SENSOR_EN
int eswin_ts_mmi_palm_set_enable(struct device *dev, unsigned int enable)
{
    int ret = 0;
    struct eph_data *ephdata;

    GET_ESWIN_DATA(dev);

    mutex_lock(&ephdata->mode_lock);
    ephdata->get_mode.palm_detection= enable;
    if (ephdata->set_mode.palm_detection == enable) {
        ts_info("The value = %d is same, so not to write", enable);
        goto exit;
    }

    if (ephdata->power_on == 0) {
        ts_info("The touch is in sleep state, restore the value when resume\n");
        goto exit;
    }

    /* clear palm status before enable detection*/
    if (enable)
        atomic_set(&ephdata->palm_status, 0);
    else {
        if (ephdata->imports && ephdata->imports->report_palm) {
            ephdata->imports->report_palm(0);
            ts_info("Disable palm detection, report far\n");
        }
        del_timer(&ephdata->palm_release_timer);
    }

    ret = eswin_ts_send_cmd(ephdata, PALM_DETECTION_SWITCH_CMD, 5,
                        ephdata->get_mode.palm_detection, 0x00);
    if (ret < 0) {
        ts_err("failed to set palm detection, enable = %d", enable);
        goto exit;
    }

    ephdata->set_mode.palm_detection = enable;
    msleep(20);
    ts_info("Success set palm detection to %d\n", enable);

exit:
    mutex_unlock(&ephdata->mode_lock);
    return ret;
}
#endif

static int eswin_ts_mmi_panel_state(struct device *dev,
    enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
    int ret = 0;
#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
    unsigned char gesture_type = 0;
    u8 gesture_cmd = 0;
#endif
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    switch (to) {
        case TS_MMI_PM_GESTURE:
#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
            if (ephdata->imports && ephdata->imports->get_gesture_type) {
                ret = ephdata->imports->get_gesture_type(&ephdata->commsdevice->dev, &gesture_type);
            }
            /* gesture enable */
            if (gesture_type & TS_MMI_GESTURE_ZERO) {
                // NOTE: must to check
                //gesture_cmd |= BIT(0);
                ts_info("enable zero gesture mode cmd 0x%02x\n", gesture_cmd);
            }
            if (gesture_type & TS_MMI_GESTURE_SINGLE) {
                gesture_cmd |= (1 << 0);
                ts_info("enable single gesture mode cmd 0x%02x\n", gesture_cmd);
            }
            if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
                gesture_cmd |= (1 << 1);
                ts_info("enable double gesture mode cmd 0x%02x\n", gesture_cmd);
            }
            ephdata->gesture_mode = gesture_type;
            ret = eph_gesture_mode_enable(&ephdata->commsdevice->dev, (u8)gesture_cmd);
            if (ret)
            {
                ts_err("failed to send cmd enter gesture mode!\n");
            }
            ts_info("Send enable gesture mode 0x%02x 0x%02x\n", gesture_cmd, gesture_type);
            ret = enable_irq_wake(ephdata->chg_irq);
            if (ret) {
                ts_info("enable irq wake fail %d\n", ret);
            }
            ephdata->gesture_wakeup_enable = true;
#endif
            break;
        case TS_MMI_PM_DEEPSLEEP:
            ret = eph_deepsleep_enable(&ephdata->commsdevice->dev, 1);
            ts_info("eph deepsleep %d\n", ret);
            ephdata->gesture_wakeup_enable = false;
            break;
        case TS_MMI_PM_ACTIVE:
            break;
        default:
            ts_err("Invalid power state parameter %d.\n", to);
            return -EINVAL;
    }

    return 0;
}

static int eswin_ts_mmi_pre_resume(struct device *dev)
{
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);
    atomic_set(&ephdata->post_suspended, 0);
    eph_clear_all_host_touch_slots(ephdata);
    ts_info("Resume start");

    return 0;
}

static int eswin_ts_mmi_post_resume(struct device *dev)
{
    int ret = 0;

    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
    if (ephdata->gesture_wakeup_enable) {
        disable_irq_wake(ephdata->chg_irq);
        ephdata->gesture_wakeup_enable = 0;
    }
#endif

    ret = eph_screen_on_reporting(&ephdata->commsdevice->dev, 1);
    if (ret)
        ts_err("set screen_on_reporting fail %d.\n", ret);

    if(ephdata->zerotap_data[0] == 0) {
        ts_info("No FOD-DOWN, trigger baseline");
        schedule_work(&ephdata->force_baseline_work);
    }
#ifdef ESWIN_EXTRA_MMI
    mutex_lock(&ephdata->mode_lock);
    /* All IC status are cleared after reset */
    memset(&ephdata->set_mode, 0 , sizeof(ephdata->set_mode));
    /* restore data */
    if (ephdata->ephplatform->interpolation_ctrl && ephdata->get_mode.interpolation) {
        //send switch command
        ret = eswin_ts_send_report_cmd(ephdata, ephdata->get_mode.report_rate_mode);
        if (!ret) {
            ephdata->set_mode.interpolation = ephdata->get_mode.interpolation;
            ephdata->set_mode.report_rate_mode = ephdata->get_mode.report_rate_mode;
            msleep(20);

            ts_info("Success to set %s\n", ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_120HZ ? "REPORT_RATE_120HZ" :
                (ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
                (ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_360HZ" :
                (ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
                (ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
                (ephdata->get_mode.report_rate_mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
                ("Unsupported")))))));
        }
    }

    if (ephdata->ephplatform->sample_ctrl && ephdata->get_mode.sample) {
        ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_ARMED_RATE,
                        ephdata->get_mode.sample, 0xff);
        if (!ret) {
            ephdata->set_mode.sample = ephdata->get_mode.sample;
            msleep(20);
            ts_info("Success to %d sample mode\n", ephdata->get_mode.sample);
        }
    }

    if (ephdata->ephplatform->edge_ctrl) {
        ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_ORIENTATION,
                            ephdata->get_mode.edge_mode[0], ephdata->get_mode.edge_mode[1]);
        if (!ret) {
            memcpy(ephdata->set_mode.edge_mode, ephdata->get_mode.edge_mode,
                    sizeof(ephdata->get_mode.edge_mode));
            msleep(20);
            ts_info("Success to set edge area = %02x, rotation = %02x",
                ephdata->get_mode.edge_mode[1], ephdata->get_mode.edge_mode[0]);
        }
    }

#ifdef ESWIN_PALM_SENSOR_EN
    if (ephdata->get_mode.palm_detection) {
        ts_info("Restore palm_detection mode");
        ret = goodix_ts_send_cmd(ephdata, PALM_DETECTION_SWITCH_CMD, 5,
                        ephdata->get_mode.palm_detection, 0x00);
        if (!ret) {
            ephdata->set_mode.palm_detection = ephdata->get_mode.palm_detection;
            msleep(20);
            ts_info("Success to %d palm detection mode\n", ephdata->get_mode.palm_detection);
        }
    }
#endif

    if (ephdata->ephplatform->stowed_mode_ctrl) {
        ephdata->set_mode.stowed = 0;
    }

#ifdef CONFIG_ESWIN_POCKET_MODE
    if (ephdata->ephplatform->pocket_mode_ctrl && ephdata->get_mode.pocket_mode) {
        ts_info("Restore pocket mode");
        ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_POCKET,
            ephdata->get_mode.pocket_mode , 0xff);
        if (!ret) {
            ephdata->set_mode.pocket_mode = ephdata->get_mode.pocket_mode;
            ts_info("Success to %s pocket mode", ephdata->get_mode.pocket_mode ? "Enable" : "Disable");
        }
        else {
            ts_err("Failed to set pocket mode");
        }
    }
#endif

    mutex_unlock(&ephdata->mode_lock);
#ifdef CONFIG_ESWIN_FOD
    if(ephdata->zerotap_data[0]) {
        ts_info("FOD is down during PM resume  fod_enable=%d",ephdata->fod_enable);
    }
    ephdata->zerotap_data[0] = 0;
#endif

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
    atomic_set(&ephdata->allow_capture, 1);
    ts_info("Resume end, enable ghost log capture");
#endif
#endif//ESWIN_EXTRA_MMI

    ephdata->suspended = false;
    ts_info("Resume end");

    return 0;
}

static int eswin_ts_mmi_pre_suspend(struct device *dev)
{
    int ret;
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    ts_info("Suspend start");
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
	//disable/stop ghost log capture
	atomic_set(&ephdata->allow_capture, 0);
	ts_info("Suspend start, disable ghost log capture\n");

	mutex_lock(&ephdata->frame_log_lock);
	if(atomic_read(&ephdata->trigger_enable) == 1) {
		atomic_set(&ephdata->trigger_enable, 0);
		ephdata->data_valid = 0;
		frame_log_capture_stop(ephdata);
	}
	mutex_unlock(&ephdata->frame_log_lock);
#endif

    ret = eph_screen_on_reporting(&ephdata->commsdevice->dev, 0);
    if (ret)
        ts_err("set screen off reporting fail %d.\n", ret);

    if(!ephdata->gesture_wakeup_enable) {
        eph_clear_all_host_touch_slots(ephdata);
    }
    cancel_work_sync(&ephdata->force_baseline_work);

    return 0;
}

static int eswin_ts_mmi_post_suspend(struct device *dev)
{
    int ret = 0;
    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);

    atomic_set(&ephdata->post_suspended, 1);

    ephdata->suspended = true;

    if (ephdata->ephplatform->stowed_mode_ctrl && ephdata->get_mode.stowed && (ephdata->power_on == 1)) {
        //ret = eswin_ts_send_cmd(ephdata, CMD_HOST_DISCRETE, CMD_OFFSET_STOWED, ephdata->get_mode.stowed, 0xff);
        if (ret < 0) {
            ts_err("Failed to set stowed mode %d", ephdata->get_mode.stowed);
        } else {
            ephdata->set_mode.stowed = ephdata->get_mode.stowed;
            ts_info("Enable stowed mode %d success.", ephdata->set_mode.stowed);
        }
    }
    ts_info("Suspend end");

    return 0;
}

#ifdef CONFIG_ESWIN_FOD
static int eswin_ts_mmi_update_fod_mode(struct device *dev, int mode)
{
    int ret;

    struct eph_data *ephdata;
    GET_ESWIN_DATA(dev);
#if 0
    ret = eph_fod_mode_enable(&ephdata->commsdevice->dev, ((mode >0) ? 0x01 : 0x00));
#endif
    ts_info("update_fod_mode %d\n", mode);

    return ret;
}
#endif

static struct ts_mmi_methods eswin_ts_mmi_methods = {
    .get_vendor = eswin_ts_mmi_methods_get_vendor,
    .get_productinfo = eswin_ts_mmi_methods_get_productinfo,
    .get_build_id = eswin_ts_mmi_methods_get_build_id,
    .get_config_id = eswin_ts_mmi_methods_get_config_id,
    .get_bus_type = eswin_ts_mmi_methods_get_bus_type,
    .get_irq_status = eswin_ts_mmi_methods_get_irq_status,
    .get_drv_irq = eswin_ts_mmi_methods_get_drv_irq,
    .get_poweron = eswin_ts_mmi_methods_get_poweron,
    .get_flashprog = eswin_ts_mmi_methods_get_flashprog,
    .get_active_region = eswin_ts_mmi_methods_get_active_region,
    /* SET methods */
    .reset =  eswin_ts_mmi_methods_reset,
    .drv_irq = eswin_ts_mmi_methods_drv_irq,
    .power = eswin_ts_mmi_methods_power,
    .charger_mode = eswin_ts_mmi_charger_mode,
    .refresh_rate = eswin_ts_mmi_refresh_rate,
    .active_region = eswin_ts_mmi_active_region,
#ifdef ESWIN_PALM_SENSOR_EN
    .palm_set_enable = eswin_ts_mmi_palm_set_enable,
#endif
    /* Firmware */
    .firmware_update = eswin_ts_firmware_update,
#ifdef ESWIN_EXTRA_MMI
    /* vendor specific attribute group */
    .extend_attribute_group = eswin_ts_mmi_extend_attribute_group,
#endif
    /* PM callback */
    .panel_state = eswin_ts_mmi_panel_state,
    .pre_resume = eswin_ts_mmi_pre_resume,
    .post_resume = eswin_ts_mmi_post_resume,
    .pre_suspend = eswin_ts_mmi_pre_suspend,
    .post_suspend = eswin_ts_mmi_post_suspend,
#ifdef CONFIG_ESWIN_FOD
    .update_fod_mode = eswin_ts_mmi_update_fod_mode,
#endif
};

int eswin_ts_mmi_dev_register(struct comms_device *commsdevice)
{
    int ret;
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    mutex_init(&ephdata->mode_lock);
    ret = ts_mmi_dev_register(&ephdata->commsdevice->dev, &eswin_ts_mmi_methods);
    if (ret)
    {
        dev_err(&commsdevice->dev, "Failed to register ts mmi\n");
        mutex_destroy(&ephdata->mode_lock);
        return ret;
    }
    ephdata->imports = &eswin_ts_mmi_methods.exports;

    return 0;
}

void eswin_ts_mmi_dev_unregister(struct comms_device *commsdevice)
{
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    if (!ephdata)
    {
        dev_err(&commsdevice->dev,"Failed to get driver data");
    }
    mutex_destroy(&ephdata->mode_lock);
    ts_mmi_dev_unregister(&ephdata->commsdevice->dev);

    return;
}
