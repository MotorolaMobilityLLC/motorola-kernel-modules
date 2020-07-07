/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
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
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef FOCALTECH_PEN_NOTIFIER
#include <linux/pen_detection_notify.h>
#endif
#include "focaltech_config.h"
#ifdef CONFIG_DRM
#if FTS_CONFIG_DRM_PANEL
	#include <drm/drm_panel.h>
#else
	#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_FB)
	#include <linux/notifier.h>
	#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include "focaltech_core.h"

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
extern int fts_mmi_dev_register(struct fts_ts_data *ts_data);
extern void fts_mmi_dev_unregister(struct fts_ts_data *ts_data);
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */
#if FTS_USB_DETECT_EN
bool 	FTS_USB_detect_flag;
#endif
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      2800000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;

#ifdef FOCALTECH_PALM_SENSOR_EN
static struct sensors_classdev __maybe_unused palm_sensors_touch_cdev = {
    .name = "palm-gesture",
    .vendor = "Focaltech",
    .version = 1,
    .type = SENSOR_TYPE_MOTO_TOUCH_PALM,
    .max_range = "5.0",
    .resolution = "5.0",
    .sensor_power = "1",
    .min_delay = 0,
    .max_delay = 0,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 200,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
};
#endif

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
#ifdef FOCALTECH_PEN_NOTIFIER
static int fts_mcu_pen_detect_set(uint8_t pen_detect);
#endif
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;
    u8 chip_id = fts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &reg_value);
        if ((ret < 0) || (reg_value != chip_id)) {
            FTS_INFO("TP Not Ready, ReadData = 0x%x", reg_value);
        } else if (reg_value == chip_id) {
            FTS_INFO("TP Ready, Device ID = 0x%x", reg_value);
            return 0;
        }
        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */
    fts_ex_mode_recovery(ts_data);
    /* recover TP gesture state 0xD0 */
#ifdef FOCALTECH_PALM_SENSOR_EN
    if (ts_data->palm_detection_enabled) {
        fts_write_reg(0xB0, 0x01);
        FTS_INFO("Resume palm detect mode.");
    }
#endif
#if FTS_GESTURE_EN
    fts_gesture_recovery(ts_data);
#endif
#ifdef FOCALTECH_PEN_NOTIFIER
    fts_mcu_pen_detect_set(ts_data->fts_pen_detect_flag);
#endif
    FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
    FTS_DEBUG("tp reset");
    gpio_direction_output(fts_data->pdata->reset_gpio, 0);
    msleep(5);
    gpio_direction_output(fts_data->pdata->reset_gpio, 1);
    if (hdelayms) {
        msleep(hdelayms);
    }

    return 0;
}

void fts_irq_disable(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
    int ret = 0;
    u8 buf[3] = {0xEB, 0xAA, 0x09};

    ret = fts_write(buf, 3);
    if (ret < 0) {
        FTS_ERROR("hid2std cmd write fail");
    } else {
        msleep(10);
        buf[0] = buf[1] = buf[2] = 0;
        ret = fts_read(NULL, 0, buf, 3);
        if (ret < 0) {
            FTS_ERROR("hid2std cmd read fail");
        } else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
            FTS_DEBUG("hidi2c change to stdi2c successful");
        } else {
            FTS_DEBUG("hidi2c change to stdi2c not support or fail");
        }
    }
}

static int fts_get_chip_types(
    struct fts_ts_data *ts_data,
    u8 id_h, u8 id_l, bool fw_valid)
{
    int i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

    if ((0x0 == id_h) || (0x0 == id_l)) {
        FTS_ERROR("id_h/id_l is 0");
        return -EINVAL;
    }

    FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
    for (i = 0; i < ctype_entries; i++) {
        if (VALID == fw_valid) {
            if ((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
                break;
        } else {
            if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
                || ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
                || ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl)))
                break;
        }
    }

    if (i >= ctype_entries) {
        return -ENODATA;
    }

    ts_data->ic_info.ids = ctype[i];
    return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
    int ret = 0;
    u8 chip_id[2] = { 0 };
    u8 id_cmd[4] = { 0 };
    u32 id_cmd_len = 0;

    id_cmd[0] = FTS_CMD_START1;
    id_cmd[1] = FTS_CMD_START2;
    ret = fts_write(id_cmd, 2);
    if (ret < 0) {
        FTS_ERROR("start cmd write fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    id_cmd[0] = FTS_CMD_READ_ID;
    id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
    if (ts_data->ic_info.is_incell)
        id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        id_cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
    if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
        FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
        return -EIO;
    }

    id[0] = chip_id[0];
    id[1] = chip_id[1];
    return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int cnt = 0;
    u8 chip_id[2] = { 0 };

    ts_data->ic_info.is_incell = FTS_CHIP_IDC;
    ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

    for (cnt = 0; cnt < 3; cnt++) {
        fts_reset_proc(0);
        mdelay(8);

        ret = fts_read_bootid(ts_data, &chip_id[0]);
        if (ret <  0) {
            FTS_DEBUG("read boot id fail,retry:%d", cnt);
            continue;
        }

        ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
        if (ret < 0) {
            FTS_DEBUG("can't get ic informaton,retry:%d", cnt);
            continue;
        }

        break;
    }

    if (cnt >= 3) {
        FTS_ERROR("get ic informaton fail");
        return -EIO;
    }


    FTS_INFO("get ic information, chip id = 0x%02x%02x",
             ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl);

    return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void fts_show_touch_buffer(u8 *data, int datalen)
{
    int i = 0;
    int count = 0;
    char *tmpbuf = NULL;

    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        FTS_ERROR("tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    FTS_DEBUG("point buffer:%s", tmpbuf);

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

void fts_release_all_finger(void)
{
    struct input_dev *input_dev = fts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = fts_data->pdata->max_touch_number;
#endif

    FTS_FUNC_ENTER();
    mutex_lock(&fts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

    fts_data->touchs = 0;
    fts_data->key_state = 0;
    mutex_unlock(&fts_data->report_mutex);
    FTS_FUNC_EXIT();
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    int i = 0;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int *x_dim = &data->pdata->key_x_coords[0];
    int *y_dim = &data->pdata->key_y_coords[0];

    if (!data->pdata->have_key) {
        return -EINVAL;
    }
    for (i = 0; i < data->pdata->key_number; i++) {
        if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
            (y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
            if (EVENT_DOWN(data->events[index].flag)
                && !(data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 1);
                data->key_state |= (1 << i);
                FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            } else if (EVENT_UP(data->events[index].flag)
                       && (data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 0);
                data->key_state &= ~(1 << i);
                FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        input_mt_slot(data->input_dev, events[i].id);

        if (EVENT_DOWN(events[i].flag)) {
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
        } else {
            uppoint++;
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(events[i].id);
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]P%d UP!", events[i].id);
            }
        }
    }

    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                if (data->log_level >= 1) {
                    FTS_DEBUG("[B]P%d UP!", i);
                }
                va_reported = true;
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    data->touchs = touchs;

    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
            touchs++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[A]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    u8 *buf = data->point_buf;

    memset(buf, 0xFF, data->pnt_buf_size);

#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
    buf[0] = 0x01;
    ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
    if ((0xEF == buf[1]) && (0xEF == buf[2]) && (0xEF == buf[3]))
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA)
    buf[0] = 0x01;
    ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
    if (((0xEF == buf[2]) && (0xEF == buf[3]) && (0xEF == buf[4]))
    || ((ret < 0) && (0xEF == buf[1])))
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8719)
    ret = fts_read(NULL, 0, buf + 1, data->pnt_buf_size - 1);
    if ((0xEF == buf[2]) && (0xEF == buf[3]) && (0xEF == buf[4]))
#endif
    {
        /* check if need recovery fw */
        fts_fw_recovery();
        return 1;
    }

    if ((ret < 0) || ((buf[1] & 0xF0) != 0x90)) {
        FTS_ERROR("touch data(%x) abnormal,ret:%d", buf[1], ret);
        return -EIO;
    }

#if FTS_GESTURE_EN
    ret = fts_gesture_readdata(data, buf + FTS_TOUCH_DATA_LEN);
    if (0 == ret) {
        FTS_INFO("succuss to get gesture data in irq handler");
        return 1;
    }
#endif


    if (data->log_level >= 3) {
        fts_show_touch_buffer(buf, data->pnt_buf_size);
    }

    return 0;
}

#ifdef FOCALTECH_PALM_SENSOR_EN
static void fts_palm_report(bool active) {
    if (active) {
        input_report_abs(fts_data->palm_sensor_pdata->input_sensor_dev,
                         ABS_DISTANCE, 1);
        input_sync(fts_data->palm_sensor_pdata->input_sensor_dev);
        FTS_INFO("%s: palm report 1\n", __func__);
    } else {
        input_report_abs(fts_data->palm_sensor_pdata->input_sensor_dev,
                         ABS_DISTANCE, 0);
        input_sync(fts_data->palm_sensor_pdata->input_sensor_dev);
        FTS_INFO("%s: palm report 0\n", __func__);
    }
}

static int fts_palm_detect(u8 reg_data) {
    int fd_val = reg_data & 0x03;

    FTS_DEBUG("%s: 0x01=0x%0X!, fd_val=%d\n", __func__, reg_data, fd_val);
    if (fd_val == 1) {
        FTS_INFO("%s: palm detect!\n", __func__);
        return 1;
    }
    if (fd_val == 2) {
        FTS_INFO("%s: palm leave!\n", __func__);
        return 2;
    }
    return 0;
}
#endif

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;
#ifdef FOCALTECH_PALM_SENSOR_EN
    int pd_state = 0;
#endif

    ret = fts_read_touchdata(data);
    if (ret) {
        return ret;
    }

#ifdef FOCALTECH_PALM_SENSOR_EN
    if (data->palm_detection_enabled) {
        pd_state = fts_palm_detect(buf[1]);
        if (pd_state > 0) {
            if (pd_state == 1) {
                del_timer(&fts_data->palm_release_fimer);
                fts_palm_report(true);
                return -1;
	    } else if (pd_state == 2) {
                mod_timer(&fts_data->palm_release_fimer,
                          jiffies + msecs_to_jiffies(fts_data->palm_release_delay_ms));
#ifdef CONFIG_HAS_WAKELOCK
                wake_lock_timeout(&fts_data->palm_gesture_wakelock,
                                  fts_data->palm_release_delay_ms);
#else
                __pm_wakeup_event(&fts_data->palm_gesture_wakelock,
                                  fts_data->palm_release_delay_ms);
#endif
                return -1;
            }
        }
        if (buf[1] & 0x08) {
            FTS_ERROR("Invalid palm detect value. 0x%0X", buf[1]);
            return -1;
        }
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    data->touch_point = 0;

    if (data->ic_info.is_incell) {
        if ((data->point_num == 0x0F) && (buf[2] == 0xFF) && (buf[3] == 0xFF)
            && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
            FTS_DEBUG("touch buff is 0xff, need recovery state");
            fts_release_all_finger();
            fts_tp_state_recovery(data);
            return -EIO;
        }
    }

    if (data->point_num > max_touch_num) {
        FTS_INFO("invalid point_num(%d)", data->point_num);
        return -EIO;
    }

    for (i = 0; i < max_touch_num; i++) {
        base = FTS_ONE_TCH_LEN * i;
        pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else if (pointid >= max_touch_num) {
            FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
            return -EINVAL;
        }

        data->touch_point++;
        events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
        events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
        events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
        events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
        events[i].p =  buf[FTS_TOUCH_PRE_POS + base];

        if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
            FTS_INFO("abnormal touch data from fw");
            return -EIO;
        }
    }

    if (data->touch_point == 0) {
        FTS_INFO("no touch point information");
        return -EIO;
    }

    return 0;
}

#if FTS_USB_DETECT_EN
static void fts_mcu_usb_detect_set(uint8_t usb_connected)
{
	uint8_t write_data = 0;
	uint8_t read_data = 0;
	uint8_t retry_cnt = 0;
	int	ret = 0;

	do{
		if (usb_connected == 0x01) {
			write_data= 0x01;
			ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, write_data);
			if (ret < 0)
				FTS_ERROR("set register USB IN fail, ret=%d", ret);
			FTS_INFO("%s: USB detect status IN!\n", __func__);
		} else {
			write_data= 0x00;
			ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, write_data);
			if (ret < 0)
				FTS_ERROR("set register USB OUT fail, ret=%d", ret);
			FTS_INFO("%s: USB detect status OUT!\n", __func__);
		}

		ret = fts_read_reg(FTS_REG_CHARGER_MODE_EN, &read_data);
		if (ret < 0)
			FTS_ERROR("read 8b register fail, ret=%d", ret);
		retry_cnt++;
	}while((write_data != read_data) && retry_cnt < FTS_REG_RETRY_TIMES);
}

void fts_cable_detect_func(bool force_renew)
{
	struct fts_ts_data *ts_data = fts_data;
	uint8_t connect_status = 0;
	connect_status = FTS_USB_detect_flag;

	if ((connect_status != ts_data->usb_connected) || force_renew) {
		if (connect_status) {
			ts_data->usb_connected = 0x01;
		} else {
			ts_data->usb_connected = 0x00;
		}

		fts_mcu_usb_detect_set(ts_data->usb_connected);
		FTS_INFO("%s: Cable status change: 0x%2.2X\n", __func__, ts_data->usb_connected);
	}
}
#endif

#ifdef FOCALTECH_PEN_NOTIFIER
static int fts_mcu_pen_detect_set(uint8_t pen_detect)
{
    uint8_t write_data = 0;
    uint8_t read_data = 0;
    uint8_t retry_cnt = 0;
    int ret = 0;

    do{
        if (pen_detect == PEN_DETECTION_INSERT) {
            write_data= 0x01;
            ret = fts_write_reg(FTS_REG_PEN_DETECTION, write_data);
            if (ret < 0) {
                FTS_ERROR("set register PEN IN fail, ret=%d", ret);
                return ret;
            }
            FTS_INFO("%s: PEN detect status IN!\n", __func__);
        } else if (pen_detect == PEN_DETECTION_PULL) {
            write_data= 0x00;
            ret = fts_write_reg(FTS_REG_PEN_DETECTION, write_data);
            if (ret < 0) {
                FTS_ERROR("set register PEN OUT fail, ret=%d", ret);
                return ret;
            }
            FTS_INFO("%s: PEN detect status OUT!\n", __func__);
        }

        ret = fts_read_reg(FTS_REG_PEN_DETECTION, &read_data);
        if (ret < 0)
            FTS_ERROR("read 8b register fail, ret=%d", ret);
        retry_cnt++;
    }while((write_data != read_data) && retry_cnt < FTS_REG_RETRY_TIMES);

    if (retry_cnt >= FTS_REG_RETRY_TIMES) {
        FTS_ERROR("write pen status fail");
        return -EIO;
    }

    return 0;
}
#endif

static void fts_irq_read_report(void)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

#if FTS_USB_DETECT_EN
	fts_cable_detect_func(false);
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
    fts_prc_queue_work(ts_data);
#endif

    ret = fts_read_parse_touchdata(ts_data);
    if ((ret == 0) && !ts_data->suspended) {
        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data);
#else
        fts_input_report_a(ts_data);
#endif
        mutex_unlock(&ts_data->report_mutex);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(0);
#endif

}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
    fts_irq_read_report();
    return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
    ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
                               pdata->irq_gpio_flags,
                               FTS_DRIVER_NAME, ts_data);

    return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    FTS_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        FTS_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = FTS_DRIVER_NAME;
    input_dev->id.bustype = BUS_SPI;

    input_dev->dev.parent = ts_data->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        FTS_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

    ret = input_register_device(input_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

    ts_data->input_dev = input_dev;

    FTS_FUNC_EXIT();
    return 0;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
    int point_num = 0;
    int events_num = 0;

    point_num = FTS_MAX_POINTS_SUPPORT;
    ts_data->pnt_buf_size = FTS_TOUCH_DATA_LEN + FTS_GESTURE_DATA_LEN;

    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        FTS_ERROR("failed to alloc memory for point buf");
        return -ENOMEM;
    }

    events_num = point_num * sizeof(struct ts_event);
    ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
    if (!ts_data->events) {
        FTS_ERROR("failed to alloc memory for point events");
        kfree_safe(ts_data->point_buf);
        return -ENOMEM;
    }

    return 0;
}


#ifdef FOCALTECH_PALM_SENSOR_EN
static void fts_palm_sensor_release_timer_handler(unsigned long data)
{
    fts_palm_report(false);
}

static int _fts_palm_sensor_set_enable(unsigned int enable)
{
    FTS_INFO("Palm gesture set enable %d!", enable);
/*
 * If palm detect function is enabled, interrupt will not disable, IC works in
 * normal mode. But in case touch event is reported to input subsystem, skip
 * touch event when suspend flag is true. So input subsystem will not take
 * wakelock because no one report event.
 * In this case, we still need read data from IC, so AP can not enter suspend.
 */
    if (enable == 1) {
#ifdef CONFIG_HAS_WAKELOCK
        wake_lock(&fts_data->palm_gesture_read_wakelock);
#else
        __pm_stay_awake(&fts_data->palm_gesture_read_wakelock);
#endif
        fts_data->palm_detection_enabled = true;
        fts_write_reg(0xB0, 0x01);
    } else if (enable == 0) {
        fts_data->palm_detection_enabled = false;
        if (timer_pending(&fts_data->palm_release_fimer)) {
            fts_palm_report(false);
            del_timer(&fts_data->palm_release_fimer);
        }
        fts_write_reg(0xB0, 0x00);
#ifdef CONFIG_HAS_WAKELOCK
        wake_unlock(&fts_data->palm_gesture_read_wakelock);
#else
        __pm_relax(&fts_data->palm_gesture_read_wakelock);
#endif
    } else {
        FTS_INFO("unknown enable symbol\n");
    }
    return 0;
}

static int fts_palm_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
    int ret = 0;

    mutex_lock(&fts_data->suspend_resume_mutex);
    if (!fts_data->suspended)
        ret = _fts_palm_sensor_set_enable(enable);
    else {
        FTS_INFO("Gesture lazy set enable %d!", enable);
        if (enable == 1)
            fts_data->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_ENABLE;
        else if (enable == 0)
            fts_data->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_DISABLE;
        else
            FTS_INFO("unknown enable symbol\n");
    }
    mutex_unlock(&fts_data->suspend_resume_mutex);
    fts_palm_report(false);

    return ret;
}

static int fts_palm_sensor_init(struct fts_ts_data *data)
{
    struct focaltech_sensor_platform_data *sensor_pdata;
    struct input_dev *sensor_input_dev;
    int err;

    sensor_input_dev = input_allocate_device();
    if (!sensor_input_dev) {
        FTS_ERROR("Failed to allocate device");
        goto exit;
    }

    sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
                                sizeof(struct focaltech_sensor_platform_data),
                                GFP_KERNEL);
    if (!sensor_pdata) {
        FTS_ERROR("Failed to allocate memory");
        goto free_sensor_pdata;
    }
    data->palm_sensor_pdata = sensor_pdata;

    __set_bit(EV_ABS, sensor_input_dev->evbit);
    __set_bit(EV_SYN, sensor_input_dev->evbit);
    input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
                         0, 5, 0, 0);
    sensor_input_dev->name = "palm_detect";
    data->palm_sensor_pdata->input_sensor_dev = sensor_input_dev;

    err = input_register_device(sensor_input_dev);
    if (err) {
        FTS_ERROR("Unable to register device, err=%d", err);
        goto free_sensor_input_dev;
    }

    sensor_pdata->ps_cdev = palm_sensors_touch_cdev;
    sensor_pdata->ps_cdev.sensors_enable = fts_palm_sensor_set_enable;
    sensor_pdata->data = data;

    err = sensors_classdev_register(&sensor_input_dev->dev,
                                    &sensor_pdata->ps_cdev);
    if (err)
        goto unregister_sensor_input_device;

#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_init(&data->palm_gesture_wakelock, WAKE_LOCK_SUSPEND, "palm_detect_wl");
#else
    wakeup_source_init(&data->palm_gesture_wakelock, "palm_detect_wl");
#endif
#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_init(&data->palm_gesture_read_wakelock, WAKE_LOCK_SUSPEND, "palm_read_wl");
#else
    wakeup_source_init(&data->palm_gesture_read_wakelock, "palm_read_wl");
#endif

    data->palm_release_fimer.function = fts_palm_sensor_release_timer_handler;
    init_timer(&data->palm_release_fimer);
    data->palm_release_delay_ms = 850;

    return 0;

unregister_sensor_input_device:
    input_unregister_device(data->palm_sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
    input_free_device(data->palm_sensor_pdata->input_sensor_dev);
free_sensor_pdata:
    devm_kfree(&sensor_input_dev->dev, sensor_pdata);
    data->palm_sensor_pdata = NULL;
exit:
    return 1;
}

int fts_palm_sensor_remove(struct fts_ts_data *data)
{
    sensors_classdev_unregister(&data->palm_sensor_pdata->ps_cdev);
    input_unregister_device(data->palm_sensor_pdata->input_sensor_dev);
    devm_kfree(&data->palm_sensor_pdata->input_sensor_dev->dev,
               data->palm_sensor_pdata);
#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_destroy(&data->palm_gesture_wakelock);
#else
    wakeup_source_trash(&data->palm_gesture_wakelock);
#endif
#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_destroy(&data->palm_gesture_read_wakelock);
#else
    wakeup_source_trash(&data->palm_gesture_read_wakelock);
#endif
    data->palm_sensor_pdata = NULL;
    data->palm_detection_enabled = false;
    return 0;
}
#endif

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
    int ret = 0;

    ts->pinctrl = devm_pinctrl_get(ts->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        FTS_ERROR("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        FTS_ERROR("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }

    ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
    if (IS_ERR_OR_NULL(ts->pins_suspend)) {
        FTS_ERROR("Pin state[suspend] not found");
        ret = PTR_ERR(ts->pins_suspend);
        goto err_pinctrl_lookup;
    }

    ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
    if (IS_ERR_OR_NULL(ts->pins_release)) {
        FTS_ERROR("Pin state[release] not found");
        ret = PTR_ERR(ts->pins_release);
    }

    return 0;
err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_release = NULL;
    ts->pins_suspend = NULL;
    ts->pins_active = NULL;
    return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_active) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
        if (ret < 0) {
            FTS_ERROR("Set normal pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_suspend) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
        if (ret < 0) {
            FTS_ERROR("Set suspend pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl) {
        if (IS_ERR_OR_NULL(ts->pins_release)) {
            devm_pinctrl_put(ts->pinctrl);
            ts->pinctrl = NULL;
        } else {
            ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
            if (ret < 0)
                FTS_ERROR("Set gesture pin state error:%d", ret);
        }
    }

    return ret;
}
#endif /* FTS_PINCTRL_EN */

int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
    int ret = 0;

    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        FTS_ERROR("vdd is invalid");
        return -EINVAL;
    }

    FTS_FUNC_ENTER();
    if (enable) {
        if (ts_data->power_disabled) {
            FTS_DEBUG("regulator enable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
            ret = regulator_enable(ts_data->vdd);
            if (ret) {
                FTS_ERROR("enable vdd regulator failed,ret=%d", ret);
            }

            if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
                ret = regulator_enable(ts_data->vcc_i2c);
                if (ret) {
                    FTS_ERROR("enable vcc_i2c regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = false;
        }
    } else {
        if (!ts_data->power_disabled) {
            FTS_DEBUG("regulator disable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
            ret = regulator_disable(ts_data->vdd);
            if (ret) {
                FTS_ERROR("disable vdd regulator failed,ret=%d", ret);
            }
            if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
                ret = regulator_disable(ts_data->vcc_i2c);
                if (ret) {
                    FTS_ERROR("disable vcc_i2c regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = true;
        }
    }

    FTS_FUNC_EXIT();
    return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ts_data->vdd = regulator_get(ts_data->dev, "vdd");
    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        ret = PTR_ERR(ts_data->vdd);
        FTS_ERROR("get vdd regulator failed,ret=%d", ret);
        return ret;
    }

    if (regulator_count_voltages(ts_data->vdd) > 0) {
        ret = regulator_set_voltage(ts_data->vdd, FTS_VTG_MIN_UV,
                                    FTS_VTG_MAX_UV);
        if (ret) {
            FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
            regulator_put(ts_data->vdd);
            return ret;
        }
    }

    ts_data->vcc_i2c = regulator_get(ts_data->dev, "vcc_i2c");
    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
            ret = regulator_set_voltage(ts_data->vcc_i2c,
                                        FTS_I2C_VTG_MIN_UV,
                                        FTS_I2C_VTG_MAX_UV);
            if (ret) {
                FTS_ERROR("vcc_i2c regulator set_vtg failed,ret=%d", ret);
                regulator_put(ts_data->vcc_i2c);
            }
        }
    }

#if FTS_PINCTRL_EN
    fts_pinctrl_init(ts_data);
    fts_pinctrl_select_normal(ts_data);
#endif

    ts_data->power_disabled = true;
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret) {
        FTS_ERROR("fail to enable power(regulator)");
    }

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
    fts_pinctrl_select_release(ts_data);
#endif

    fts_power_source_ctrl(ts_data, DISABLE);

    if (!IS_ERR_OR_NULL(ts_data->vdd)) {
        if (regulator_count_voltages(ts_data->vdd) > 0)
            regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
        regulator_put(ts_data->vdd);
    }

    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
            regulator_set_voltage(ts_data->vcc_i2c, 0, FTS_I2C_VTG_MAX_UV);
        regulator_put(ts_data->vcc_i2c);
    }

    return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_suspend(ts_data);
#endif

    ret = fts_power_source_ctrl(ts_data, DISABLE);
    if (ret < 0) {
        FTS_ERROR("power off fail, ret=%d", ret);
    }

    return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_normal(ts_data);
#endif

    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret < 0) {
        FTS_ERROR("power on fail, ret=%d", ret);
    }

    return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        if(!data->pdata->share_reset_gpio) {
            ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
            if (ret) {
                FTS_ERROR("[GPIO]reset gpio request failed");
                goto err_irq_gpio_dir;
            }
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 1);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        FTS_ERROR("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret < 0) {
        FTS_ERROR("Unable to read %s, please check dts", name);
        pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
        pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
        pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
        pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
        return -ENODATA;
    } else {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    }

    FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    struct fts_ts_data *ts_data = fts_data;
    u32 temp_val = 0;

    FTS_FUNC_ENTER();

    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        FTS_ERROR("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key Y Coords undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
                                         pdata->key_y_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key X Coords undefined!");

        FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
                 "coords:(%d,%d),(%d,%d),(%d,%d)",
                 pdata->key_number,
                 pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_y_coords[0],
                 pdata->key_x_coords[1], pdata->key_y_coords[1],
                 pdata->key_x_coords[2], pdata->key_y_coords[2]);
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        FTS_ERROR("Unable to get reset_gpio");

    pdata->share_reset_gpio = of_property_read_bool(np, "focaltech,share_reset_gpio");
    if (pdata->share_reset_gpio)
        FTS_INFO("TP reset pin is shared with LCD");

    pdata->always_on_vio = of_property_read_bool(np, "focaltech,always_on_vio");
    if (pdata->always_on_vio)
        FTS_INFO("TP VIO always on.");

    pdata->dlfw_in_resume = of_property_read_bool(np, "focaltech,dlfw_in_resume");
    if (pdata->dlfw_in_resume)
        FTS_INFO("Reset touch when firmware abnormal in resume.");

    pdata->report_gesture_key = of_property_read_bool(np, "focaltech,report_gesture_key");
    if (pdata->report_gesture_key)
        FTS_INFO("Report tap gesture as key.");

    pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        FTS_ERROR("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get max-touch-number, please check dts");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    } else {
        if (temp_val < 2)
            pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    }

    FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

	ret = of_property_read_string(np, "focaltech,panel-supplier",
		&ts_data->panel_supplier);
	if (ret < 0) {
		ts_data->panel_supplier = NULL;
		FTS_ERROR("Unable to read panel supplier\n");
	} else {
		FTS_INFO("panel supplier is %s", (char *)ts_data->panel_supplier);
	}

    FTS_FUNC_EXIT();
    return 0;
}

#if defined(CONFIG_FB) || defined(CONFIG_DRM)
static void fts_resume_work(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
                                  resume_work);

    fts_ts_resume(ts_data->dev);
}
#endif

#ifdef FOCALTECH_PEN_NOTIFIER
static int pen_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
    int ret = 0;

    struct fts_ts_data *ts_data =
        container_of(self, struct fts_ts_data, pen_notif);
    FTS_INFO("Received event(%lu) for pen detection\n", event);

    if (event == PEN_DETECTION_INSERT)
        ts_data->fts_pen_detect_flag = PEN_DETECTION_INSERT;
    else if (event == PEN_DETECTION_PULL)
        ts_data->fts_pen_detect_flag = PEN_DETECTION_PULL;

    if ((fts_data->suspended) || (!fts_data->fw_is_running)) {
        FTS_INFO("touch in suspend or no firmware, so store.");
    } else {
        ret = fts_mcu_pen_detect_set(ts_data->fts_pen_detect_flag);
        if (ret < 0) {
            FTS_ERROR("write pen state fail");
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_DRM
#if FTS_CONFIG_DRM_PANEL
struct drm_panel *active_panel;
static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        FTS_ERROR("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            FTS_INFO("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    FTS_ERROR("no find drm_panel");
    return -ENODEV;
}

static int fts_ts_check_default_tp(struct device_node *dt, const char *prop)
{
    const char **active_tp = NULL;
    int count, tmp, score = 0;
    const char *active;
    int ret, i;

    count = of_property_count_strings(dt->parent, prop);
    if (count <= 0 || count > 3)
        return -ENODEV;

    active_tp = kcalloc(count, sizeof(char *),  GFP_KERNEL);
    if (!active_tp) {
        FTS_ERROR("FTS alloc failed\n");
        return -ENOMEM;
    }

    ret = of_property_read_string_array(dt->parent, prop,
            active_tp, count);
    if (ret < 0) {
        FTS_ERROR("fail to read %s %d\n", prop, ret);
        ret = -ENODEV;
        goto out;
    }

    for (i = 0; i < count; i++) {
        active = active_tp[i];
        if (active != NULL) {
            tmp = of_device_is_compatible(dt, active);
            if (tmp > 0)
                score++;
        }
    }

    if (score <= 0) {
        FTS_ERROR("not match this driver\n");
        ret = -ENODEV;
        goto out;
    }
    ret = 0;
out:
    kfree(active_tp);
    return ret;
}

int drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int *blank = NULL;
	struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data, fb_notif);

	if (!evdata) {
		FTS_ERROR("evdata is null");
		return 0;
	}

	if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
		  || (event == DRM_PANEL_EVENT_BLANK))) {
		FTS_INFO("event(%lu) do not need process\n", event);
		return 0;
	}

	blank = evdata->data;
	FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
	switch (*blank) {
	case DRM_PANEL_BLANK_UNBLANK:
		if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
			FTS_INFO("resume: event = %lu, not care\n", event);
		} else if (DRM_PANEL_EVENT_BLANK == event) {
			queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
		}
		break;
	case DRM_PANEL_BLANK_POWERDOWN:
		if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
			cancel_work_sync(&fts_data->resume_work);
			fts_ts_suspend(ts_data->dev);
		} else if (DRM_PANEL_EVENT_BLANK == event) {
			FTS_INFO("suspend: event = %lu, not care\n", event);
		}
		break;
	default:
		FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
		break;
	}

	return 0;
}
#else
int drm_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;
	struct fts_ts_data *ts_data =
		container_of(self, struct fts_ts_data, fb_notif);

	if (!evdata || (evdata->id != 0)) {
		return 0;
    }

    if (!(event == MSM_DRM_EARLY_EVENT_BLANK || event == MSM_DRM_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }
    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case MSM_DRM_BLANK_UNBLANK:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case MSM_DRM_BLANK_POWERDOWN:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
#ifdef FOCALTECH_PALM_SENSOR_EN
            if (ts_data->palm_detection_enabled) {
                FTS_INFO("palm detection is enabled");
                return 1;
            }
#endif
#ifdef FOCALTECH_SENSOR_EN
            if (fts_data->should_enable_gesture) {
                FTS_INFO("double tap gesture suspend\n");
                return 1;
            }
#endif
        } else if (MSM_DRM_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DAM BLANK(%d) do not need process\n", *blank);
        break;
    }

	return 0;
}
#endif
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (FB_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    fts_ts_suspend(ts_data->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    fts_ts_resume(ts_data->dev);
}
#endif

#if FTS_USB_DETECT_EN
static int fts_charger_notifier_callback(struct notifier_block *nb,
								unsigned long val, void *v) {
	int ret = 0;
	struct power_supply *psy = NULL;
	struct fts_ts_data *ts = container_of(nb, struct fts_ts_data, charger_notif);
	union power_supply_propval prop;

	psy= power_supply_get_by_name("usb");
	if (!psy) {
		FTS_ERROR("Couldn't get usbpsy\n");
		return -EINVAL;
	}
	if (!strcmp(psy->desc->name, "usb")) {
		if (psy && ts && val == POWER_SUPPLY_PROP_STATUS) {
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,&prop);
			if (ret < 0) {
				FTS_ERROR("Couldn't get POWER_SUPPLY_PROP_ONLINE rc=%d\n", ret);
				return ret;
			} else {
				FTS_USB_detect_flag = prop.intval;
				//FTS_ERROR("usb prop.intval =%d\n", prop.intval);
			}
		}
	}
	return 0;
}
#endif

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int pdata_size = sizeof(struct fts_ts_platform_data);
#if FTS_CONFIG_DRM_PANEL
	struct device_node *dp = ts_data->dev->of_node;
#endif

    FTS_FUNC_ENTER();
    FTS_INFO("%s", FTS_DRIVER_VERSION);
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata) {
        FTS_ERROR("allocate memory for platform_data fail");
        return -ENOMEM;
    }

    if (ts_data->dev->of_node) {
        ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
        if (ret)
            FTS_ERROR("device-tree parse fail");
#if FTS_CONFIG_DRM_PANEL
	if (drm_check_dt(dp)) {
		FTS_ERROR("parse drm-panel fail");
		if (!fts_ts_check_default_tp(dp, "qcom,spi-touch-active"))
			ret = -EPROBE_DEFER;
		else
			ret = -ENODEV;
		return ret;
	}
#endif
    } else {
        if (ts_data->dev->platform_data) {
            memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
        } else {
            FTS_ERROR("platform_data is null");
            return -ENODEV;
        }
    }

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (!ts_data->ts_workqueue) {
        FTS_ERROR("create fts workqueue fail");
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->bus_lock);
    mutex_init(&ts_data->suspend_resume_mutex);
#ifdef FOCALTECH_SENSOR_EN
    mutex_init(&ts_data->state_mutex);
    //unknown screen state
    ts_data->screen_state = SCREEN_UNKNOWN;
#endif

    /* Init communication interface */
    ret = fts_bus_init(ts_data);
    if (ret) {
        FTS_ERROR("bus initialize fail");
        goto err_bus_init;
    }

    ret = fts_input_init(ts_data);
    if (ret) {
        FTS_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = fts_report_buffer_init(ts_data);
    if (ret) {
        FTS_ERROR("report buffer init fail");
        goto err_report_buffer;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        FTS_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }

#if FTS_POWER_SOURCE_CUST_EN
    ret = fts_power_source_init(ts_data);
    if (ret) {
        FTS_ERROR("fail to get power(regulator)");
        goto err_power_init;
    }
#endif

#if (!FTS_CHIP_IDC)
    fts_reset_proc(200);
#endif

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        FTS_ERROR("not focal IC, unregister driver");
        goto err_irq_req;
    }

#if FTS_APK_NODE_EN
    ret = fts_create_apk_debug_channel(ts_data);
    if (ret) {
        FTS_ERROR("create apk debug node fail");
    }
#endif

#if FTS_SYSFS_NODE_EN
    ret = fts_create_sysfs(ts_data);
    if (ret) {
        FTS_ERROR("create sysfs node fail");
    }
#endif

#if FTS_POINT_REPORT_CHECK_EN
    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        FTS_ERROR("init point report check fail");
    }
#endif

    ret = fts_ex_mode_init(ts_data);
    if (ret) {
        FTS_ERROR("init glove/cover/charger fail");
    }

#if FTS_GESTURE_EN
    ret = fts_gesture_init(ts_data);
    if (ret) {
        FTS_ERROR("init gesture fail");
    }
#endif

#ifdef FOCALTECH_PALM_SENSOR_EN
    ret = fts_palm_sensor_init(ts_data);
    if (ret) {
        FTS_ERROR("init palm detect sensor fail");
    }
#endif

#if FTS_TEST_EN
    ret = fts_test_init(ts_data);
    if (ret) {
        FTS_ERROR("init production test fail");
    }
#endif

#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init(ts_data);
    if (ret) {
        FTS_ERROR("init esd check fail");
    }
#endif

    ret = fts_irq_registration(ts_data);
    if (ret) {
        FTS_ERROR("request irq failed");
        goto err_irq_req;
    }

    ret = fts_fwupg_init(ts_data);
    if (ret) {
        FTS_ERROR("init fw upgrade fail");
    }

#ifdef FOCALTECH_PEN_NOTIFIER
    ts_data->fts_pen_detect_flag = PEN_DETECTION_PULL;
    ts_data->pen_notif.notifier_call = pen_notifier_callback;
    ret = pen_detection_register_client(&ts_data->pen_notif);
    if (ret) {
        FTS_ERROR("[PEN]Unable to register pen_notifier: %d\n", ret);
    }
#endif

#ifdef CONFIG_DRM
    if (ts_data->ts_workqueue) {
        INIT_WORK(&ts_data->resume_work, fts_resume_work);
    }
    ts_data->fb_notif.notifier_call = drm_notifier_callback;
#if FTS_CONFIG_DRM_PANEL
    if (active_panel) {
        ret = drm_panel_notifier_register(active_panel, &ts_data->fb_notif);
        if (ret)
            FTS_ERROR("[DRM]drm_panel_notifier_register fail: %d\n", ret);
    } else {
        FTS_ERROR("[DRM]drm_panel_notifier_register fail: active_panel NULL!\n");
    }
#else
    ret = msm_drm_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[DRM]Unable to register fb_notifier: %d\n", ret);
    }
#endif
#elif defined(CONFIG_FB)
    if (ts_data->ts_workqueue) {
        INIT_WORK(&ts_data->resume_work, fts_resume_work);
    }
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif

#if FTS_USB_DETECT_EN
	ts_data->usb_connected = 0x00;
	ts_data->charger_notif.notifier_call = fts_charger_notifier_callback;
	ret = power_supply_reg_notifier(&ts_data->charger_notif);
	if (ret) {
		FTS_ERROR("Unable to register charger_notifier: %d\n",ret);
		goto err_register_charger_notify_failed;
	}
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
    fts_mmi_dev_register(ts_data);
#endif

    FTS_FUNC_EXIT();
    return 0;

#if FTS_USB_DETECT_EN
err_register_charger_notify_failed:
if (ts_data->charger_notif.notifier_call)
	power_supply_unreg_notifier(&ts_data->charger_notif);
#endif

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
    fts_power_source_exit(ts_data);
#endif
    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);
    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
err_report_buffer:
    input_unregister_device(ts_data->input_dev);
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
    kfree_safe(ts_data->pdata);

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
    fts_mmi_dev_unregister(ts_data);
#endif

#if FTS_USB_DETECT_EN
	if (ts_data->charger_notif.notifier_call)
		power_supply_unreg_notifier(&ts_data->charger_notif);
#endif

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit(ts_data);
#endif

#if FTS_APK_NODE_EN
    fts_release_apk_debug_channel(ts_data);
#endif

#if FTS_SYSFS_NODE_EN
    fts_remove_sysfs(ts_data);
#endif

    fts_ex_mode_exit(ts_data);

    fts_fwupg_exit(ts_data);

#if FTS_TEST_EN
    fts_test_exit(ts_data);
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_exit(ts_data);
#endif

#ifdef FOCALTECH_PALM_SENSOR_EN
    fts_palm_sensor_remove(ts_data);
#endif

#if FTS_GESTURE_EN
    fts_gesture_exit(ts_data);
#endif

    fts_bus_exit(ts_data);

    free_irq(ts_data->irq, ts_data);
    input_unregister_device(ts_data->input_dev);

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
#ifdef FOCALTECH_PEN_NOTIFIER
    if (pen_detection_unregister_client(&ts_data->pen_notif))
        FTS_ERROR("Error occurred while unregistering pen_notifier.\n");
#endif
#ifdef CONFIG_DRM
#if FTS_CONFIG_DRM_PANEL
    if (active_panel) {
        if(drm_panel_notifier_unregister(active_panel, &ts_data->fb_notif))
            FTS_ERROR("Error occurred while unregistering panel fb_notifier.\n");
    }
#else
    if (msm_drm_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_source_exit(ts_data);
#endif

    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);

    kfree_safe(ts_data->pdata);
    kfree_safe(ts_data);

    FTS_FUNC_EXIT();

    return 0;
}

static int _fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

#ifdef FOCALTECH_SENSOR_EN
    mutex_lock(&ts_data->state_mutex);
#endif

    FTS_FUNC_ENTER();
    if (ts_data->suspended) {
#ifdef FOCALTECH_SENSOR_EN
        mutex_unlock(&ts_data->state_mutex);
#endif
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
#ifdef FOCALTECH_SENSOR_EN
        mutex_unlock(&ts_data->state_mutex);
#endif
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend();
#endif

#ifdef FOCALTECH_PALM_SENSOR_EN
    if (ts_data->palm_detection_enabled) {
        ret = enable_irq_wake(ts_data->irq);
        if (ret) {
            FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
        }
        fts_release_all_finger();
        ts_data->suspended = true;
        FTS_INFO("Enter from palm detect suspend mode.");
        return 0;
    }
#endif

#if FTS_GESTURE_EN
#ifdef FOCALTECH_SENSOR_EN
    if (ts_data->should_enable_gesture) {
#endif
        if (fts_gesture_suspend(ts_data) == 0) {
            /* Enter into gesture mode(suspend) */
            ts_data->suspended = true;
#ifdef FOCALTECH_SENSOR_EN
            ts_data->screen_state = SCREEN_OFF;
            ts_data->wakeable = true;
            mutex_unlock(&ts_data->state_mutex);
            FTS_INFO("Enter gesture suspend mode.");
#endif
            return 0;
        }
#ifdef FOCALTECH_SENSOR_EN
    }
#endif
#endif

    fts_irq_disable();
    /* TP enter sleep mode */
    ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
    if (ret < 0)
        FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

    /* TP delay 50ms then lcd entery suspend*/
    mdelay(50);

    if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
        ret = fts_power_source_suspend(ts_data);
        if (ret < 0) {
            FTS_ERROR("power enter suspend fail");
        }
#endif
    }

    if (!ts_data->pdata->always_on_vio) {
        FTS_INFO("Set reset pin to 0 in suspend.");
        gpio_direction_output(ts_data->pdata->reset_gpio, 0);
    }

    fts_release_all_finger();
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
#ifdef FOCALTECH_SENSOR_EN
    ts_data->screen_state = SCREEN_OFF;
    mutex_unlock(&ts_data->state_mutex);
#endif
    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    mutex_lock(&fts_data->suspend_resume_mutex);
    ret = _fts_ts_suspend(dev);
    mutex_unlock(&fts_data->suspend_resume_mutex);

    return ret;
}

static int _fts_ts_resume(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

#ifdef FOCALTECH_SENSOR_EN
    mutex_lock(&ts_data->state_mutex);
#endif

    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
#ifdef FOCALTECH_SENSOR_EN
        mutex_unlock(&ts_data->state_mutex);
#endif
        FTS_DEBUG("Already in awake state");
        return 0;
    }

    fts_release_all_finger();

    if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
        fts_power_source_resume(ts_data);
#endif
        //fts_reset_proc(200);
    }

    if (!ts_data->pdata->always_on_vio) {
        FTS_INFO("Reset IC in resume");
        fts_reset_proc(200);
    }

    fts_irq_enable();

    if (ts_data->pdata->dlfw_in_resume) {
        ret = fts_wait_tp_to_valid();
        if(ret){
            FTS_INFO("wait tp to valid abnormal,need download fw");
            fts_fw_resume(true);
            msleep(10);
        }
    }
    fts_tp_state_recovery(ts_data);

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

#ifdef FOCALTECH_PALM_SENSOR_EN
    if (ts_data->palm_detection_enabled) {
        int ret = 0;
        ret = disable_irq_wake(ts_data->irq);
        if (ret) {
            FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
        }
        FTS_INFO("Exit from palm detect suspend mode.");
        ts_data->suspended = false;
        goto CHECK_LAZY_SET;
    }
#endif

#if FTS_GESTURE_EN

#ifdef FOCALTECH_SENSOR_EN
    if (ts_data->wakeable) {
#endif
        if (fts_gesture_resume(ts_data) == 0) {
            ts_data->suspended = false;
#ifdef FOCALTECH_SENSOR_EN
            ts_data->screen_state = SCREEN_ON;
            ts_data->wakeable = false;
            mutex_unlock(&ts_data->state_mutex);
            FTS_INFO("Exit from gesture suspend mode.");
#endif
            return 0;
        }
#ifdef FOCALTECH_SENSOR_EN
    }
#endif
#endif

    ts_data->suspended = false;

#if FTS_USB_DETECT_EN
	fts_cable_detect_func(true);
#endif

    FTS_FUNC_EXIT();
#ifdef FOCALTECH_SENSOR_EN
    mutex_unlock(&ts_data->state_mutex);
    ts_data->screen_state = SCREEN_ON;
#endif
#ifdef FOCALTECH_PALM_SENSOR_EN
CHECK_LAZY_SET:
    if (ts_data->palm_detection_lazy_set != PALM_SENSOR_LAZY_SET_NONE) {
        _fts_palm_sensor_set_enable(
            (ts_data->palm_detection_lazy_set == PALM_SENSOR_LAZY_SET_ENABLE) ? 1 : 0);
        FTS_INFO("Palm sensor lazy set done, clear flag.");
        ts_data->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_NONE;
    }
#endif
    return 0;
}

static int fts_ts_resume(struct device *dev)
{
    int ret = 0;
    mutex_lock(&fts_data->suspend_resume_mutex);
    ret = _fts_ts_resume(dev);
    mutex_unlock(&fts_data->suspend_resume_mutex);

    return ret;
}

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_probe(struct spi_device *spi)
{
    int ret = 0;
    struct fts_ts_data *ts_data = NULL;

    FTS_INFO("Touch Screen(SPI BUS) driver prboe...");
#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
    spi->mode = SPI_MODE_0;
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8719)
    spi->mode = SPI_MODE_1;
#else
    spi->mode = SPI_MODE_1;
#endif

    spi->bits_per_word = 8;
    if (spi->max_speed_hz > FTS_SPI_CLK_MAX)
        spi->max_speed_hz = FTS_SPI_CLK_MAX;

    ret = spi_setup(spi);
    if (ret) {
        FTS_ERROR("spi setup fail");
        return ret;
    }

    /* malloc memory for global struct variable */
    ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        FTS_ERROR("allocate memory for fts_data fail");
        return -ENOMEM;
    }

    fts_data = ts_data;
    ts_data->spi = spi;
    ts_data->dev = &spi->dev;
    ts_data->log_level = 1;
    spi_set_drvdata(spi, ts_data);

    ret = fts_ts_probe_entry(ts_data);
    if (ret) {
        FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }

    FTS_INFO("Touch Screen(SPI BUS) driver prboe successfully");
    return 0;
}

static int fts_ts_remove(struct spi_device *spi)
{
    return fts_ts_remove_entry(spi_get_drvdata(spi));
}

static const struct spi_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
static const struct of_device_id fts_dt_match[] = {
    {.compatible = "focaltech,fts", },
    {},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct spi_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(fts_dt_match),
    },
    .id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ret = spi_register_driver(&fts_ts_driver);
    if ( ret != 0 ) {
        FTS_ERROR("Focaltech touch screen driver init failed!");
    }
    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    spi_unregister_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
