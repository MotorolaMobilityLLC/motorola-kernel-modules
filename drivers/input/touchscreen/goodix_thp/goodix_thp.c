/************************************************************************/
/* Copyright <2019-2020> GOODIX                                        */
/*                                                                      */
/* GOODIX Confidential. This software is owned or controlled by GOODIX  */
/* and may only be used strictly in accordance with the applicable      */
/* license terms.  By expressly accepting such terms or by downloading, */
/* installing, activating and/or otherwise using the software, you are  */
/* agreeing that you have read, and that you agree to comply with and   */
/* are bound by, such license terms.                                    */
/* If you do not agree to be bound by the applicable license terms,     */
/* then you may not retain, install, activate or otherwise use the      */
/* software.                                                            */
/*                                                                      */
/************************************************************************/

#include <linux/time.h>
#include <linux/time64.h>

#include "goodix_thp.h"
#include "goodix_thp_mmi.h"
#ifndef CREATE_TRACE_POINTS
#define CREATE_TRACE_POINTS
#endif
#include "trace_touch.h"

#define GOODIX_THP_MISC_DEVICE_NAME	"thp"
#define PINCTRL_STATE_ACTIVE		"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND		"pmx_ts_suspend"
#define DEVICE_NAME			"input_agent"
#define GOOIDX_INPUT_PHYS		"goodix_ts/input0"

#define GOODIX_ESD_TICK_WRITE_DATA 0xAA
#define GOODIX_ESD_CHECK_INTERVAL (8 * HZ)
#define MAJOR_CONVERSION_SHIFT 5
#define TOUCH_MAJOR_CLAMP_THRESHOLD 254
#define TOUCH_MAJOR_MAX_VALUE 255

bool debug_log_flag;
#ifdef CONFIG_TOUCHCLASS_MMI_FORCE_ENTER_STANDBY
bool main_suspend;
#endif
static int goodix_thp_suspend(struct goodix_thp_core *core_data);
static int goodix_thp_resume(struct goodix_thp_core *core_data);
static int goodix_thp_power_on(struct goodix_thp_core *core_data);
static int goodix_ts_pinctrl_select_active(struct goodix_thp_core *core_data);
static int goodix_ts_pinctrl_select_suspend(struct goodix_thp_core *core_data);

static int goodix_thp_spi_trans(struct goodix_thp_core *cd,
                        char *tx_buf, char *rx_buf, unsigned int len)
{
        struct spi_message spi_msg;
        struct spi_device *sdev = cd->sdev;
        struct thp_ts_device *tdev = cd->ts_dev;

        struct spi_transfer xfer = {
                .tx_buf = tx_buf,
                .rx_buf = rx_buf,
                .len    = len,
        };
        int ret;

        spi_message_init(&spi_msg);
        spi_message_add_tail(&xfer, &spi_msg);

        mutex_lock(&tdev->spi_mutex);
        ret = spi_sync(sdev, &spi_msg);
        mutex_unlock(&tdev->spi_mutex);

        return ret;
}

static void goodix_thp_esd_on(struct goodix_thp_core *core_data, bool on)
{
        if (!core_data->ts_dev->board_data.esd_enable) {
                ts_info(core_data->ts_dev->dev, "ESD function is not enabled");
                return;
        }

        if (core_data->esd_on == on)
                return;

        core_data->esd_on = on;
        if (on) {
                schedule_delayed_work(&core_data->esd_work, GOODIX_ESD_CHECK_INTERVAL);
        } else {
                cancel_delayed_work(&core_data->esd_work);
        }

        ts_info(core_data->ts_dev->dev, "ESD %s", on ? "on" : "off");
}

/*
 * If irq is disabled/enabled, can not disable/enable again
 * disable - status 0; enable - status not 0
 */
static void goodix_thp_set_irq_enable(struct goodix_thp_core *core_data,
					int status)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        mutex_lock(&core_data->irq_mutex);
        if (core_data->irq_state != !!status) {
                status ? enable_irq(core_data->irq) : disable_irq(core_data->irq);
                core_data->irq_state = !!status;
                ts_info(ts_dev->dev, "%s irq",
                                status ? "enable" : "disable");
        }
        mutex_unlock(&core_data->irq_mutex);
};

static void goodix_thp_set_irq_wake_enable(struct goodix_thp_core *core_data,
					int status)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        mutex_lock(&core_data->irq_wake_mutex);
        if (core_data->irq_wake_state != !!status) {
                status ? enable_irq_wake(core_data->irq) : disable_irq_wake(core_data->irq);
                core_data->irq_wake_state = !!status;
                ts_info(ts_dev->dev, "%s irq_wake",
                                status ? "enable" : "disable");
        }
        mutex_unlock(&core_data->irq_wake_mutex);
};

static void goodix_thp_frame_wake_up(struct goodix_thp_core *core_data)
{
        mutex_lock(&(core_data->frame_mutex));
        core_data->frame_waitq_state = WAKEUP_STATE;
        wake_up_interruptible(&(core_data->frame_wq));
        mutex_unlock(&(core_data->frame_mutex));
}

static void goodix_thp_reset_frame_list(struct goodix_thp_core *core_data)
{
        mutex_lock(&core_data->frame_mutex);
        core_data->frame_mmap_list.head = 0;
        core_data->frame_mmap_list.tail = 0;
        memset(core_data->frame_mmap_list.buf, 0, MMAP_BUFFER_SIZE);
        mutex_unlock(&core_data->frame_mutex);
}

void put_frame_list(struct goodix_thp_core *core_data, int type, u8 *data, int len)
{
        struct driver_request_pkg *req_pkg;
        struct thp_frame_mmap_list *list = &core_data->frame_mmap_list;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        static uint32_t id;

        mutex_lock(&core_data->frame_mutex);
        /* check for max limit */
        if (unlikely((list->tail + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT == list->head)) {
                ts_err(ts_dev->dev, "touch_health - frame mmap buffer is full, overwriting oldest data");
                list->head = (list->head + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT; // Overwrite the oldest data
        }

        req_pkg = (struct driver_request_pkg *)&list->buf[list->tail * GOODIX_THP_MAX_FRAME_LEN];
        req_pkg->size = sizeof(req_pkg->request) + len;
        req_pkg->request.id = id++;
        req_pkg->request.type = type;
        if (likely(len > 0))
                memcpy(req_pkg->request.data, data, len);
        list->tail = (list->tail + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT;

        core_data->frame_waitq_state = WAKEUP_STATE;
        wake_up_interruptible(&(core_data->frame_wq));
        mutex_unlock(&(core_data->frame_mutex));
}

static void goodix_thp_reinit(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        ts_info(ts_dev->dev, "called");

        if (core_data->power_on == 0)
                goodix_thp_power_on(core_data);
        else
                ts_dev->hw_ops->reset(ts_dev, 100);

        goodix_thp_set_irq_wake_enable(core_data, IRQ_WAKE_DISABLE_FLAG);
        core_data->suspended = 0;
        core_data->gesture_enable = 0;
}

static int goodix_thp_open(struct inode *inode, struct file *filp)
{
        struct goodix_thp_core *core_data =
                container_of(filp->private_data, struct goodix_thp_core, thp_misc_dev);
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        ts_info(ts_dev->dev, "called");

        goodix_thp_reinit(core_data);
        return 0;
}

static int goodix_thp_release(struct inode *inode, struct file *filp)
{
        struct goodix_thp_core *core_data =
                container_of(filp->private_data, struct goodix_thp_core, thp_misc_dev);
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        ts_info(ts_dev->dev, "called");
        goodix_thp_frame_wake_up(core_data);
        return 0;
}

static long goodix_thp_ioctl_get_frame(struct goodix_thp_core *core_data, unsigned long arg)
{
        void __user *user_val = (void *)arg;
        struct thp_frame_mmap_list *list = &core_data->frame_mmap_list;
        struct thp_ioctl_frame hal_frame;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        long r = 0;

        mutex_lock(&core_data->frame_mutex);
        if (list->head == list->tail) {
                if (core_data->get_frame_wait_mode == GET_FRAME_NONBLOCK_MODE) {
                        ts_err(ts_dev->dev, "no frame");
                        r = -ENODATA;
                        goto out;
                } else {
                        core_data->frame_waitq_state = WAIT_STATE;
                        if (core_data->frame_wait_time == 0) {
                                mutex_unlock(&core_data->frame_mutex);
                                r = wait_event_interruptible(core_data->frame_wq,
                                        (core_data->frame_waitq_state == WAKEUP_STATE));
                                mutex_lock(&core_data->frame_mutex);
                                if (r < 0) {
                                        ts_err(ts_dev->dev, "Interrupted by a signal");
                                        goto out;
                                }
                        } else {
                                mutex_unlock(&core_data->frame_mutex);
                                r = wait_event_interruptible_timeout(core_data->frame_wq,
                                        (core_data->frame_waitq_state == WAKEUP_STATE),
                                        msecs_to_jiffies(core_data->frame_wait_time));
                                mutex_lock(&core_data->frame_mutex);
                                if (r < 0) {
                                        ts_err(ts_dev->dev, "Interrupted by a signal");
                                        goto out;
                                } else if (r == 0) {
                                        ts_err(ts_dev->dev, "get frame timeout");
                                        r = -ETIMEDOUT;
                                        goto out;
                                }
                        }
                }
        }

        r = 0;
        if (list->head != list->tail) {
                hal_frame.pos = list->head * GOODIX_THP_MAX_FRAME_LEN;
                hal_frame.tv_us = ktime_get_real_ns() / 1000;
                if(copy_to_user(user_val, &hal_frame, sizeof(hal_frame))) {
                        ts_err(ts_dev->dev, "Failed to copy_to_user().");
                        r = -EFAULT;
                }
        } else {
                ts_err(ts_dev->dev, "no frame");
                r = -ENODATA;
        }

out:
        mutex_unlock(&core_data->frame_mutex);
        return r;
}

static long goodix_thp_ioctl_set_reset_value(struct goodix_thp_core *core_data, unsigned long reset)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;

        ts_info(ts_dev->dev, "set reset status %ld", reset);

        gpio_set_value(ts->ts_dev->board_data.reset_gpio, !!reset);

        ts->frame_waitq_state = WAIT_STATE;
        ts->reset_state = !reset;

        return 0;
}

static long goodix_thp_ioctl_set_wait_time(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;
        unsigned int wait_frame_time = arg;

        if (arg > GOODIX_THP_MAX_TIMEOUT)
                wait_frame_time = GOODIX_THP_MAX_TIMEOUT;

        ts_info(ts_dev->dev, "set wait time %d ms.(current %dms)",
                        wait_frame_time, ts->frame_wait_time);

        if (wait_frame_time != ts->frame_wait_time) {
                mutex_lock(&(ts->frame_mutex));
                ts->frame_wait_time = wait_frame_time;
                ts->frame_waitq_state = WAKEUP_STATE;
                wake_up_interruptible(&(ts->frame_wq));
                mutex_unlock(&(ts->frame_mutex));
        }

        return 0;
}

static long goodix_thp_ioctl_spi_trans(struct goodix_thp_core *core_data, void __user *data)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        int r = 0;
        u8 *tx_buf = NULL;
        u8 *rx_buf = NULL;
        struct thp_ioctl_spi_trans_data trans_data;

        if (cd->suspended && !cd->gesture_enable)
		return 0;

        /* copy data from hal */
        if (copy_from_user(&trans_data, data,
                        sizeof(struct thp_ioctl_spi_trans_data))) {
                ts_err(ts_dev->dev, "Failed to copy_from_user().");
                return -EFAULT;
        }

        /* check sync data size */
        if (trans_data.size > GOODIX_THP_MAX_TRANS_DATA_LEN) {
                ts_err(ts_dev->dev, "trans_data.size out of range.");
                return -EINVAL;
        }

        /* alloc memory for rx/tx buf */
        rx_buf = kzalloc(trans_data.size, GFP_KERNEL);
        tx_buf = kzalloc(trans_data.size, GFP_KERNEL);
        if (!rx_buf || !tx_buf) {
                ts_err(ts_dev->dev, "buf request memory fail,trans_data.size = %d",
                        trans_data.size);
                goto exit;
        }

        /* copy hal tx buf to driver tx buf */
        r = copy_from_user(tx_buf, trans_data.tx, trans_data.size);
        if (r) {
                ts_err(ts_dev->dev, "copy in buff fail");
                goto exit;
        }

        /* spi transfer */
        r =  goodix_thp_spi_trans(cd, tx_buf, rx_buf, trans_data.size);
        if (r) {
                ts_err(ts_dev->dev, "transfer error, ret = %d", r);
                goto exit;
        }

        /* copy driver rx to hal */
        if (trans_data.rx) {
                r = copy_to_user(trans_data.rx, rx_buf, trans_data.size);
                if (r) {
                        ts_err(ts_dev->dev, "copy out buff fail");
                        goto exit;
                }
        }

exit:
        if(rx_buf){
                kfree(rx_buf);
                rx_buf = NULL;
        }
        if(tx_buf){
                kfree(tx_buf);
                tx_buf = NULL;
        }
        return r;
}

static long goodix_thp_ioctl_notify_update(struct goodix_thp_core *core_data, void __user *data)
{
        struct goodix_thp_board_data *board_data =
                        &core_data->ts_dev->board_data;
        struct thp_ioctl_update_info update_info;
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        if (copy_from_user((u8 *)&update_info, data,
                        sizeof(update_info))) {
                ts_err(ts_dev->dev, "Failed to copy_from_user().");
                return -EFAULT;
        }

        board_data->frame_addr = update_info.frame_addr;
        board_data->cmd_addr = update_info.cmd_addr;
        board_data->ges_addr = update_info.ges_addr;
        board_data->esd_addr = update_info.esd_addr;
        ts_info(ts_dev->dev, "set frame addr:0x%04X cmd addr:0x%04X ges addr:0x%04X esd addr:0x%04X",
                board_data->frame_addr,
                board_data->cmd_addr,
                board_data->ges_addr,
                board_data->esd_addr);
        return 0;
}

static long goodix_thp_ioctl_set_wait_mode(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;
        unsigned int wait_frame_mode = arg;

        mutex_lock(&(ts->frame_mutex));
        if (wait_frame_mode)
                ts->get_frame_wait_mode = GET_FRAME_BLOCK_MODE;
        else
                ts->get_frame_wait_mode = GET_FRAME_NONBLOCK_MODE;
        ts->frame_waitq_state = WAKEUP_STATE;
        wake_up_interruptible(&(ts->frame_wq));
        mutex_unlock(&(ts->frame_mutex));
        ts_info(ts_dev->dev, "set block %d", wait_frame_mode);
        return 0;
}

static long goodix_thp_ioctl_irq_enable(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        unsigned int irq_flag = (unsigned int)arg;
        goodix_thp_set_irq_enable(ts, irq_flag);
        return 0;
}

static long goodix_thp_ioctl_get_frame_buf_num(struct goodix_thp_core *core_data, unsigned long arg)
{
        return 0;
}

static long goodix_thp_ioctl_reset_frame_list(struct goodix_thp_core *core_data)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;

        ts_info(ts_dev->dev, "called");
        goodix_thp_reset_frame_list(ts);
        return 0;
}

static long goodix_thp_ioctl_get_driver_state(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;
        u32 __user *driver_state = (u32 *)arg;

        ts_info(ts_dev->dev, "driver state = %d", ts->suspended);

        if (driver_state == NULL) {
                ts_err(ts_dev->dev, "input parameter null");
                return -EINVAL;
        }

        if(copy_to_user(driver_state, &ts->suspended, sizeof(u32))) {
                ts_err(ts_dev->dev, "copy driver_state failed");
                return -EFAULT;
        }

        return 0;
}

static long goodix_thp_ioctl_get_state_change_flag(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;
        u32 __user *change_flag = (u32 *)arg;

        //ts_info("%s:state_change_flag = %d", __func__, ts->state_change_flag);

        if (change_flag == NULL) {
                ts_err(ts_dev->dev, "input parameter null");
                return -EINVAL;
        }

        if(copy_to_user(change_flag, &ts->state_change_flag, sizeof(u32))) {
                ts_err(ts_dev->dev, "copy state_change_flag failed");
                return -EFAULT;
        }

        return 0;
}

static long goodix_thp_ioctl_set_state_change_flag(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        unsigned int change_flag = arg;

        //ts_info("set state_change_flag = %d", change_flag);

        ts->state_change_flag = change_flag;

        return 0;
}

static int goodix_thp_ioctl_set_spi_speed(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *dev = ts->ts_dev;
        unsigned int speed = arg;

        if (dev->hw_ops->set_spi_speed(dev, speed))
                return -EINVAL;

        return 0;
}

static long goodix_thp_ioctl_multi_spi_trans(struct goodix_thp_core *core_data, void __user *data)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *dev = ts->ts_dev;
        struct spi_device *spi = dev->spi_dev;
        struct thp_ioctl_multi_spi_trans_data multi_data;
        struct thp_ioctl_spi_xfer_data * xfer_data = NULL;
        struct spi_transfer * xfer = NULL;
        struct spi_message msg;
        u8 *tx_buf = NULL;
        u8 *rx_buf = NULL;
        int r = 0, i = 0;
        u32 spi_speed_backup = 0;
        unsigned int tmp_len = 0;

        spi_speed_backup = dev->board_data.spi_setting.spi_max_speed;

        if (copy_from_user(&multi_data, data, sizeof(struct thp_ioctl_multi_spi_trans_data))) {
                return -EFAULT;
        }

        xfer_data =  kzalloc(multi_data.xfer_num * sizeof(*xfer_data), GFP_KERNEL);
        if (!xfer_data) {
                ts_info(dev->dev, "failed alloc memory for xfer_data");
                goto exit;
        }

        xfer =  kzalloc(multi_data.xfer_num * sizeof(*xfer), GFP_KERNEL);
        if (!xfer) {
                ts_info(dev->dev, "failed alloc memory for xfer");
                goto exit;
        }

        if (copy_from_user(xfer_data, multi_data.xfer_data,
                sizeof(struct thp_ioctl_spi_xfer_data) * multi_data.xfer_num)) {
                ts_info(dev->dev, "failed copy from user for xfer_data");
                goto exit;
        }

        rx_buf = kzalloc(5120, GFP_KERNEL);
        if (!rx_buf) {
                ts_info(dev->dev, "failed alloc buffer for rx_buf");
                goto exit;
        }
        tx_buf = kzalloc(5120, GFP_KERNEL);
        if (!tx_buf) {
                ts_info(dev->dev, "failed alloc buffer for tx_buf");
                goto exit;
        }

        spi_message_init(&msg);
        for(i = 0; i < multi_data.xfer_num; i++) {
                if(xfer_data[i].tx){
                        r = copy_from_user(tx_buf + tmp_len,
                                xfer_data[i].tx, xfer_data[i].len);
                        if (r) {
                                ts_info(dev->dev, "failed copy from user:%d", r);
                                goto exit;
                        }
                }
                xfer[i].tx_buf = tx_buf + tmp_len;
                xfer[i].rx_buf = rx_buf + tmp_len;
                xfer[i].len = xfer_data[i].len;
                xfer[i].cs_change = !!xfer_data[i].cs_change;
                spi_message_add_tail(&xfer[i], &msg);
                tmp_len += xfer_data[i].len;
        }

        if (multi_data.speed_hz == GOODIX_SPI_SPEED_WAKEUP) {
                mutex_lock(&dev->spi_mutex);
                dev->hw_ops->set_spi_speed(dev, GOODIX_SPI_SPEED_WAKEUP);
                spi_sync(spi, &msg);
                dev->hw_ops->set_spi_speed(dev, spi_speed_backup);
                mutex_unlock(&dev->spi_mutex);
        } else {
                mutex_lock(&dev->spi_mutex);
                r = spi_sync(spi, &msg);
                mutex_unlock(&dev->spi_mutex);
                if(r) {
                        ts_info(dev->dev, "failed do spi sync:%d", r);
                        goto exit;
                }
                tmp_len = 0;
                for(i = 0; i < multi_data.xfer_num; i++ ){
                        if (xfer_data[i].rx) {
                                r = copy_to_user(xfer_data[i].rx, rx_buf + tmp_len,
                                        xfer_data[i].len);
                                tmp_len += xfer_data[i].len;
                        } else {
                                tmp_len += xfer_data[i].len;
                        }
                }
        }
exit:
        kfree(tx_buf);
        kfree(rx_buf);
        kfree(xfer_data);
        kfree(xfer);
        return r;
}

static long goodix_thp_ioctl_enter_suspend(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        int r = 0;

        cd->gesture_enable = arg;
        ts_info(ts_dev->dev, "called");

        r = goodix_thp_suspend(cd);
        if (r)
                ts_err(ts_dev->dev, "failed, r %d", r);

        return r;
}

static long goodix_thp_ioctl_dump_rep_done(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *cd = core_data;
        int r = 0;

        ts_info(core_data->ts_dev->dev, "Notify raw data capture down");
        sysfs_notify(cd->imports->kobj_notify, NULL, "log_trigger");

        return r;
}

static long goodix_thp_ioctl_enter_resume(struct goodix_thp_core *core_data)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        int r = 0;

        ts_info(ts_dev->dev, "called");

        r = goodix_thp_resume(cd);
        if (r)
                ts_err(ts_dev->dev, "failed, r %d", r);

        //restore param after IC reset
        goodix_ts_mmi_post_resume(cd);

        return r;
}

static long goodix_thp_ioctl_recv_tsc_msg(struct goodix_thp_core *core_data, unsigned long arg)
{
        void __user *argp = (void __user *)arg;
        struct thp_ioctl_tsc_msg tsc_msg;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        u8 stylus_id[2];

        if (copy_from_user(&tsc_msg, argp,
                        sizeof(struct thp_ioctl_tsc_msg))) {
                ts_err(ts_dev->dev, "Failed to copy_from_user .");
                return -EFAULT;
        }

        switch (tsc_msg.cmd) {
        case SVC_CMD_MMAP_DEQUEUE:
                mutex_lock(&core_data->frame_mutex);
                if (core_data->frame_mmap_list.head != core_data->frame_mmap_list.tail) {
                        core_data->frame_mmap_list.head =
                                (core_data->frame_mmap_list.head + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT;
                }
                mutex_unlock(&core_data->frame_mutex);
                break;
        case SVC_CMD_BLE_MAC:
                memcpy(core_data->ble_mac, &tsc_msg.value[0], sizeof(core_data->ble_mac));
                memcpy(stylus_id, &tsc_msg.value[6], sizeof(stylus_id));
                core_data->uevent_message_type = PEN_MESSAGE_BLE_MAC;
                kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                ts_info(ts_dev->dev, "recv ble mac:%02x:%02x:%02x:%02x:%02x:%02x, stylusID:%02x:%02x",
                        core_data->ble_mac[5], core_data->ble_mac[4], core_data->ble_mac[3],
                        core_data->ble_mac[2], core_data->ble_mac[1], core_data->ble_mac[0],
                        stylus_id[1], stylus_id[0]);
                break;
        case SVC_CMD_GAME_FILTER:
                ts_info(ts_dev->dev, "recv game filter:%*ph", tsc_msg.len, tsc_msg.value);
                break;
        case SVC_CMD_UPDATE_VERSION:
                memcpy(core_data->ts_dev->board_data.thp_ver, tsc_msg.value, tsc_msg.len);
                ts_info(ts_dev->dev, "thp_ver:%s", tsc_msg.value);
                break;
        case SVC_CMD_HAL_INIT_FINISH:
                ts_info(ts_dev->dev, "HAL has finished");
                goodix_thp_esd_on(core_data, true);
                break;
        case SVC_CMD_OPEN_CIRCUIT:
#ifdef CONFIG_THP_FOLD
                if (core_data->pdev->id)
                    core_data->open_fold_status = tsc_msg.value[0];
                else
#endif
                core_data->open_status = tsc_msg.value[0];
#ifdef CONFIG_THP_FOLD
                if (core_data->pdev->id)
                    ts_info(ts_dev->dev, "recv fold open circuit %d", core_data->open_fold_status);
                else
#endif
                ts_info(ts_dev->dev, "recv open circuit %d", core_data->open_status);
                break;
        case SVC_CMD_BATTERY:
                core_data->battery_level = tsc_msg.value[0];
                core_data->uevent_message_type = PEN_MESSAGE_BATTERY;
                kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                ts_info(ts_dev->dev, "recv battery %d", core_data->battery_level);
                break;
        case SVC_CMD_PEN_INFO:
                memcpy(core_data->pen_info, &tsc_msg.value[0], sizeof(core_data->pen_info));
                core_data->uevent_message_type = PEN_MESSAGE_PEN_INFO;
                kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                ts_info(ts_dev->dev, "recv pen info(uid):%*ph", 9, core_data->pen_info);
                break;
        case SVC_CMD_GET_PID:
                core_data->quick_pid = tsc_msg.value[0];
                core_data->uevent_message_type = PEN_MESSAGE_PEN_QPID;
                kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                ts_debug(ts_dev->dev, "recv quick pid %d", core_data->quick_pid);
                break;
        default:
                ts_err(ts_dev->dev, "not support svc msg:0x%02x", tsc_msg.cmd);
                break;
        }

        return 0;
}

static long goodix_thp_ioctl_get_chip_type(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *ts = core_data;
        struct thp_ts_device *ts_dev = ts->ts_dev;
        u32 __user *user_val = (u32 *)arg;

        if (user_val == NULL) {
                ts_err(ts_dev->dev, "input parameter null");
                return -EINVAL;
        }

        if(copy_to_user(user_val, &ts->ts_dev->board_data.chip_type, sizeof(u32))) {
                ts_err(ts_dev->dev, "copy driver_state failed");
                return -EFAULT;
        }

        return 0;
}

/* enable or disable tsd debug socket */
static int goodix_thp_ioctl_set_tsd_state(struct goodix_thp_core *core_data, int tsd_enable)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 val[2];

        /* resume: touch power on is after display to avoid display disturb */
        ts_info(ts_dev->dev, "IN, set tsd state %d", tsd_enable);

        val[0] = NOTIFY_TYPE_TSD_CTRL;
        val[1] = tsd_enable;
        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	return 0;
}

/* enable or disable tsd debug socket */
static int goodix_thp_ioctl_set_stylus_state(struct goodix_thp_core *core_data, int stylus_enable)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 val[2];

        /* resume: touch power on is after display to avoid display disturb */
        ts_info(ts_dev->dev, "IN, set stylus state %d", stylus_enable);

        val[0] = NOTIFY_TYPE_STYLUS_CTRL;
        val[1] = stylus_enable;
        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	return 0;
}

static long goodix_thp_ioctl(struct file *filp, unsigned int cmd,
                                unsigned long arg)
{
        struct goodix_thp_core *cd =
                container_of(filp->private_data, struct goodix_thp_core, thp_misc_dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        long ret;

        switch (cmd) {
        case IOCTL_CMD_GET_FRAME:
                ret = goodix_thp_ioctl_get_frame(cd, arg);
                break;
        case IOCTL_CMD_SET_RESET_VALUE:
                ret = goodix_thp_ioctl_set_reset_value(cd, arg);
                break;
        case IOCTL_CMD_SET_WAIT_TIME:
                ret = goodix_thp_ioctl_set_wait_time(cd, arg);
                break;
        case IOCTL_CMD_SPI_TRANS:
                ret = goodix_thp_ioctl_spi_trans(cd, (void __user *)arg);
                break;
        case IOCTL_CMD_NOTIFY_UPDATE:
                ret = goodix_thp_ioctl_notify_update(cd, (void __user *)arg);
                break;
        case IOCTL_CMD_SET_WAIT_MODE:
                ret = goodix_thp_ioctl_set_wait_mode(cd, arg);
                break;
        case IOCTL_CMD_IRQ_ENABLE:
                ret = goodix_thp_ioctl_irq_enable(cd, arg);
                break;
        case IOCTL_CMD_GET_FRAME_BUF_NUM:
                ret = goodix_thp_ioctl_get_frame_buf_num(cd, arg);
                break;
        case IOCTL_CMD_RESET_FRAME_LIST:
                ret = goodix_thp_ioctl_reset_frame_list(cd);
                break;
        case IOCTL_CMD_GET_DRIVER_STATE:
                ret = goodix_thp_ioctl_get_driver_state(cd, arg);
                break;
        case IOCTL_CMD_GET_STATE_CHANGE_FLAG:
                ret = goodix_thp_ioctl_get_state_change_flag(cd, arg);
                break;
        case IOCTL_CMD_SET_STATE_CHANGE_FLAG:
                ret = goodix_thp_ioctl_set_state_change_flag(cd, arg);
                break;
        case IOCTL_CMD_SET_SPI_SPEED:
                ret = goodix_thp_ioctl_set_spi_speed(cd, arg);
                break;
        case IOCTL_CMD_MUILT_SPI_TRANS:
                ret = goodix_thp_ioctl_multi_spi_trans(cd, (void __user *)arg);
                break;
        case IOCTL_CMD_ENTER_SUSPEND:
                ret = goodix_thp_ioctl_enter_suspend(cd, arg);
                break;
        case IOCTL_CMD_DUMP_REP_DONE:
                ret = goodix_thp_ioctl_dump_rep_done(cd, arg);
                break;
        case IOCTL_CMD_ENTER_RESUME:
                ret = goodix_thp_ioctl_enter_resume(cd);
                break;
        case IOCTL_CMD_RECV_TSC_MSG:
                ret = goodix_thp_ioctl_recv_tsc_msg(cd, arg);
                break;
        case IOCTL_CMD_GET_CHIP_TYPE:
                ret = goodix_thp_ioctl_get_chip_type(cd, arg);
                break;
        case IOCTL_CMD_SET_TOOL_OPS:
                ret = goodix_thp_ioctl_set_tsd_state(cd, arg);
                break;
        default:
                ts_err(ts_dev->dev, "cmd unknown.");
                ret = 0;
        }

        return ret;
}

static int goodix_thp_mmap(struct file *filp, struct vm_area_struct *vma)
{
        struct goodix_thp_core *cd =
                container_of(filp->private_data, struct goodix_thp_core, thp_misc_dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        void *sh_mem = (void *)cd->frame_mmap_list.buf;
        size_t size = vma->vm_end - vma->vm_start;
        struct page *page = NULL;

        if (size > MMAP_BUFFER_SIZE) {
                ts_err(ts_dev->dev, "vm_size[%d] > mmap_size[%d]",
                        (int)size, MMAP_BUFFER_SIZE);
                return -EINVAL;
        }

        page = virt_to_page((unsigned long)sh_mem + (vma->vm_pgoff << PAGE_SHIFT));
        if (remap_pfn_range(vma, vma->vm_start, page_to_pfn(page),
                        size, vma->vm_page_prot)) {
                return -EAGAIN;
        }

        return 0;
}

static const struct file_operations g_thp_fops = {
        .owner = THIS_MODULE,
        .open = goodix_thp_open,
        .release = goodix_thp_release,
        .unlocked_ioctl = goodix_thp_ioctl,
        .mmap = goodix_thp_mmap,
};

/**
 * goodix_thp_power_init- Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_thp_power_init(struct goodix_thp_core *core_data)
{
        struct goodix_thp_board_data *ts_bdata;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct device *dev = NULL;
        int r = 0;

        ts_info(ts_dev->dev, "Power init");
        /* dev:i2c client device or spi slave device*/
        dev =  core_data->ts_dev->dev;
        ts_bdata = board_data(core_data);

        if (strlen(ts_bdata->avdd_name)) {
                core_data->avdd = devm_regulator_get(dev,
                                 ts_bdata->avdd_name);
                if (IS_ERR_OR_NULL(core_data->avdd)) {
                        r = PTR_ERR(core_data->avdd);
                        ts_err(ts_dev->dev, "Failed to get regulator avdd:%d", r);
                        core_data->avdd = NULL;
                        return r;
                }
                r = regulator_set_load(core_data->avdd, 50000);
                if (r) {
                    ts_err(ts_dev->dev, "set avdd load fail");
                    return r;
                }
                r = regulator_set_voltage(core_data->avdd, 3200000, 3200000);
                if (r) {
                    ts_err(ts_dev->dev, "set avdd voltage fail");
                    return r;
                }
        } else {
                ts_info(ts_dev->dev, "Avdd name is NULL[skip]");
        }

        if (strlen(ts_bdata->iovdd_name)) {
                core_data->iovdd = devm_regulator_get(dev,
                                ts_bdata->iovdd_name);
                if (IS_ERR_OR_NULL(core_data->iovdd)) {
                        r = PTR_ERR(core_data->iovdd);
                        ts_err(ts_dev->dev, "Failed to get regulator iovdd:%d", r);
                        core_data->iovdd = NULL;
                        return r;
                }
        } else {
                ts_info(ts_dev->dev, "iovdd name is NULL[skip]");
        }

        return r;
}

int goodix_thp_reset_after(struct goodix_thp_core *cd);

/**
 * goodix_thp_power_on- Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_thp_power_on(struct goodix_thp_core *core_data)
{
        struct goodix_thp_board_data *ts_bdata = board_data(core_data);
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        int r;
        int iovdd_gpio = ts_bdata->iovdd_gpio;
        int avdd_gpio = ts_bdata->avdd_gpio;

        ts_info(ts_dev->dev, "Device power on");
        if (unlikely(core_data->power_on)) {
                ts_info(ts_dev->dev, "device has already power on");
                return 0;
        }

        //config INT pin as input pull-up before power on
        goodix_ts_pinctrl_select_active(core_data);

        if (iovdd_gpio > 0) {
            gpio_direction_output(iovdd_gpio, 1);
        } else if (core_data->iovdd) {
                r = regulator_enable(core_data->iovdd);
                if (unlikely(r)) {
                        ts_err(ts_dev->dev, "Failed to enable iovdd:%d", r);
                        goto power_off;
                }
                usleep_range(3000, 3100);
        }

        if (avdd_gpio > 0) {
            gpio_direction_output(avdd_gpio, 1);
        } else if (core_data->avdd) {
                r = regulator_enable(core_data->avdd);
                if (unlikely(r)) {
                        ts_err(ts_dev->dev, "Failed to enable avdd:%d", r);
                        goto power_off;
                }
                usleep_range(15000, 15100);
        }

        if (ts_bdata->gpio_expander ) {
            /* for the expander gpio, the default sleep is not enough */
            ts_info(ts_dev->dev, "sleep 20ms for expander gpio config");
            msleep(20);
        }
        gpio_direction_output(ts_bdata->reset_gpio, 1);
        core_data->power_on = 1;
        return 0;

power_off:
        //power on fail, restore INT pin state as input pull-down
        goodix_ts_pinctrl_select_suspend(core_data);

        gpio_direction_output(ts_bdata->reset_gpio, 0);
        if (iovdd_gpio > 0) {
            gpio_direction_output(iovdd_gpio, 0);
        } else if (core_data->iovdd)
            regulator_disable(core_data->iovdd);

        if (avdd_gpio > 0) {
            gpio_direction_output(avdd_gpio, 0);
        } else if (core_data->avdd) {
            regulator_disable(core_data->avdd);
        }

        return r;
}

static void goodix_thp_power_off(struct goodix_thp_core *core_data)
{
        struct goodix_thp_board_data *ts_bdata = board_data(core_data);
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        ts_info(ts_dev->dev, "Device power off");
        if (unlikely(core_data->power_on == 0)) {
                ts_info(ts_dev->dev, "device has already power off");
                return;
        }

        gpio_direction_output(ts_bdata->reset_gpio, 0);
        if (ts_bdata->iovdd_gpio > 0) {
            gpio_direction_output(ts_bdata->iovdd_gpio, 0);
        } else if (core_data->iovdd)
            regulator_disable(core_data->iovdd);

        if (ts_bdata->avdd_gpio > 0) {
            gpio_direction_output(ts_bdata->avdd_gpio, 0);
        } else if (core_data->avdd) {
            regulator_disable(core_data->avdd);
        }
        core_data->power_on = 0;

        //config INT pin as input pull-down after touch ic power off
        goodix_ts_pinctrl_select_suspend(core_data);
}

static int goodix_thp_gpio_setup(struct goodix_thp_core *core_data)
{
        struct goodix_thp_board_data *ts_bdata = board_data(core_data);
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        int r = 0;

        ts_info(ts_dev->dev, "GPIO setup,reset-gpio:%d, irq-gpio:%d",
                ts_bdata->reset_gpio, ts_bdata->irq_gpio);

        /*
         * after kenerl3.13, gpio_ api is deprecated, new
         * driver should use gpiod_ api.
         */
        r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
                                  GPIOF_OUT_INIT_LOW, "ts_reset_gpio");
        if (r < 0) {
                ts_err(ts_dev->dev, "Failed to request reset gpio, r:%d", r);
                return r;
        }

        r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
                                  GPIOF_IN, "ts_irq_gpio");
        if (r < 0) {
                ts_err(ts_dev->dev, "Failed to request irq gpio, r:%d", r);
                return r;
        }

        if (ts_bdata->iovdd_gpio > 0) {
            r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->iovdd_gpio,
                GPIOF_OUT_INIT_LOW, "ts_iovdd_gpio");
            if (r < 0) {
                ts_err(ts_dev->dev, "Failed to request iovdd-gpio, r:%d", r);
                return r;
            }
        }

        if (ts_bdata->avdd_gpio > 0) {
            r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->avdd_gpio,
                GPIOF_OUT_INIT_LOW, "ts_avdd_gpio");
            if (r < 0) {
                ts_err(ts_dev->dev, "Failed to request avdd-gpio, r:%d", r);
                return r;
            }
        }

        return 0;
}

static int goodix_thp_gesture_irq_handler(struct goodix_thp_core *core_data)
{
        int r = 0;
        u8 clean_data = 0;
        u8 temp_data[GESTURE_KEY_DATA_LEN] = {0};
        u32 ges_addr = core_data->ts_dev->board_data.ges_addr;
        u16 gsx_data = ~core_data->gesture_enable;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct gesture_event_data mmi_event;
        static  unsigned  long  start = 0;
        int fod_down_interval = 0;
        int fod_down = core_data->zerotap_data[0];

        if (ges_addr == 0) {
                ts_err(ts_dev->dev, "gesture addr has not been assigned");
                goto exit;
        }

        /* get gesture data */
        r = ts_dev->hw_ops->read(ts_dev, ges_addr, temp_data, sizeof(temp_data));
        if (r < 0 || ((temp_data[0] & GESTURE_DATA_TYPE) == 0)) {
                ts_err(ts_dev->dev, "Read gesture data failed, r=%d, data[0]=0x%x",
                                r, temp_data[0]);
                goto re_send_ges_cmd;
        }

        /* check gesture data */
        if (checksum16_cmp(temp_data, GESTURE_DATA_HEAD_LEN, GOODIX_LE_MODE)) {
                ts_err(ts_dev->dev, "gesture data head check failed");
                ts_err(ts_dev->dev, "%*ph", GESTURE_DATA_HEAD_LEN, temp_data);
                goto re_send_ges_cmd;
        }

        /* save gesture data */
        memcpy(core_data->gesture_data, temp_data, sizeof(temp_data));

        switch (temp_data[4]) {
        case 0xCC: //double tap
                ts_info(ts_dev->dev, "get gesture event: Double tap");
                mmi_event.evcode =4;
                mmi_event.evdata.x = le16_to_cpup((__le16 *)&temp_data[8]);
                mmi_event.evdata.y = le16_to_cpup((__le16 *)&temp_data[10]);
#ifdef CONFIG_THP_FOLD
                if (core_data->pdev->id)
                    core_data->imports->report_cli_gesture(&mmi_event);
                else
#endif
                core_data->imports->report_gesture(&mmi_event);
                break;
        case 0x63: // C
                ts_info(ts_dev->dev, "get gesture event: C");
                break;
        case 0x65: // E
                ts_info(ts_dev->dev, "get gesture event: E");
                break;
        case 0x6D: // M
                ts_info(ts_dev->dev, "get gesture event: M");
                break;
        case 0x77: // W
                ts_info(ts_dev->dev, "get gesture event: W");
                break;
        case 0x40: // A
                ts_info(ts_dev->dev, "get gesture event: A");
                break;
        case 0x66: // F
                ts_info(ts_dev->dev, "get gesture event: F");
                break;
        case 0x6F: // O
                ts_info(ts_dev->dev, "get gesture event: O");
                break;
        case 0xAA: // R2L
                ts_info(ts_dev->dev, "get gesture event: right to left");
                break;
        case 0xBB: // L2R
                ts_info(ts_dev->dev, "get gesture event: left to right");
                break;
        case 0xBA: // UP
                ts_info(ts_dev->dev, "get gesture event: up");
                break;
        case 0xAB: // DOWN
                ts_info(ts_dev->dev, "get gesture event: down");
                break;
        case 0x46: // FP_DOWN
                fod_down_interval = (int)jiffies_to_msecs(jiffies-start);
                //goodix firmware do not send coordinate, need mmi touch to define a vaild coordinate thru dts
                mmi_event.evcode = 2;
                mmi_event.evdata.x= 0;
                mmi_event.evdata.y= 0;

                ts_info(ts_dev->dev, "Get FOD-DOWN gesture:%d interval:%d",fod_down,fod_down_interval);
                if(fod_down_interval > 2000)
                        fod_down = 0;
                if(fod_down_interval > 0 && fod_down_interval < 250 && fod_down) {
                        goto exit;
                }
                start = jiffies;
                //maximum allow send down event 7 times
                if(fod_down < 6)
                        core_data->imports->report_gesture(&mmi_event);
                fod_down++;

                break;
        case 0x55: // FP_UP
                ts_info(ts_dev->dev, "Get FOD-UP gesture");
                mmi_event.evcode = 3;
                mmi_event.evdata.x= 0;
                mmi_event.evdata.y= 0;
                core_data->imports->report_gesture(&mmi_event);
                fod_down = 0;
                break;
        case 0x4C: // single tap
                ts_info(ts_dev->dev, "get gesture event: single tap");
                mmi_event.evcode =1;
                mmi_event.evdata.x = le16_to_cpup((__le16 *)&temp_data[8]);
                mmi_event.evdata.y = le16_to_cpup((__le16 *)&temp_data[10]);
#ifdef CONFIG_THP_FOLD
                if (core_data->pdev->id)
                    core_data->imports->report_cli_gesture(&mmi_event);
                else
#endif
                core_data->imports->report_gesture(&mmi_event);
                break;
        default:
                ts_err(ts_dev->dev, "not support gesture type %x", temp_data[4]);
                break;
        }


        goto exit;

re_send_ges_cmd:
        /* resend gesture cmd */
        if(ts_dev->hw_ops->send_cmd(ts_dev, CMD_GESTURE, gsx_data))
                ts_info(ts_dev->dev, "warning: failed re_send gesture cmd");
exit:
        clean_data = 0;
        ts_dev->hw_ops->write(ts_dev, ges_addr, &clean_data, 1);
        core_data->zerotap_data[0] = fod_down;
        return 0;
}

/**
 * goodix_thp_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_thp_threadirq_func(int irq, void *data)
{
        struct goodix_thp_core *core_data = data;
        struct thp_ts_device *ts_dev =  core_data->ts_dev;
        u8 *read_data = (u8 *)core_data->frame_read_data;
        int r;
        static bool affinity_initialized = false;
        struct cpumask cpumask;
        static int cur_index, pre_index;
#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        struct goodix_thp_board_data *board_data =
                        &core_data->ts_dev->board_data;
        int cpu, index;
#endif
        static ktime_t last_time;
        ktime_t current_time = ktime_get();
        s64 delta = ktime_to_us(ktime_sub(current_time, last_time));
        last_time = current_time;

        //ts_info(ts_dev->dev, "IRQ latency: %lld us", delta);
        core_data->irq_trig_cnt++;

        if (unlikely(!affinity_initialized)) {
            cpumask_clear(&cpumask);
            cpumask.bits[0] = core_data->ts_dev->board_data.cpu_mask;
            if (set_cpus_allowed_ptr(current, &cpumask) != 0) {
                ts_err(ts_dev->dev, "Failed to set CPU affinity");
            } else {
                ts_info(ts_dev->dev, "CPU affinity set to mask 0x%lx", cpumask.bits[0]);
            }
            affinity_initialized = true;
        }

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        cpu = raw_smp_processor_id();

        if (cpu < NR_CPUS && (index = core_data->cpu_to_index_map[cpu]) >= 0) {
                struct cpu_boost_info *info = &core_data->boost_infos[index];

                if (core_data->boost_count < board_data->max_boost_count) {
                        freq_qos_update_request(&info->qos_req, info->max_freq);
                        core_data->boost_count++;

                        ts_debug(ts_dev->dev, "CPU%d: index=%d, freq boosted to %u kHz (count %d/%d)",
                                cpu, index, info->max_freq,
                                core_data->boost_count, board_data->max_boost_count);
                }
        }

        del_timer(&core_data->boost_timer);
#endif

        /*for check bus i2c/spi is ready or not*/
        if (unlikely(core_data->suspended && core_data->pm_suspend)) {
            r = wait_for_completion_timeout(
                        &core_data->pm_completion,
                        msecs_to_jiffies(core_data->ts_dev->board_data.irq_need_dev_resume_time));
            if (!r) {
                ts_err(ts_dev->dev, "Bus don't resume from pm(deep),timeout,skip irq");
                return IRQ_HANDLED;
            }
        }

        disable_irq_nosync(core_data->irq);
        if (core_data->ws) {
                __pm_stay_awake(core_data->ws);
        }

        /*for qaulcomn to stop cpu go to C4 idle state*/
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

        if (core_data->pm_qos_state && !core_data->suspended) {
                core_data->pm_qos_value = PM_QOS_TOUCH_WAKEUP_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)

                if (!cpu_latency_qos_request_active(&core_data->pm_qos_req)) {
                        cpu_latency_qos_add_request(&core_data->pm_qos_req, core_data->pm_qos_value);
                } else {
                        cpu_latency_qos_update_request(&core_data->pm_qos_req, core_data->pm_qos_value);
                }
#else
                pm_qos_update_request(&core_data->pm_qos_req, core_data->pm_qos_value);
#endif
        }

#endif

        if (unlikely(core_data->reset_state)) {
                ts_err(ts_dev->dev, "ignore this irq.");
                goto exit;
        }

        /* suspend irq handler */
        if (unlikely(core_data->suspended && core_data->gesture_enable)) {
                goodix_thp_gesture_irq_handler(core_data);
                goto exit;
        }

        /* get frame */
        r = ts_dev->hw_ops->get_frame(ts_dev, read_data);
        if (unlikely(r < 0)) {
                ts_err(ts_dev->dev, "failed to read frame, r %d", r);
                goto exit;
        }

        /* copy frame to frame list */
        put_frame_list(core_data, REQUEST_TYPE_FRAME, read_data, r);

        /* print frame index that write on FW  */
        cur_index = (read_data[5] << 8) | read_data[4];
        if (unlikely((cur_index != pre_index + 1) && (cur_index > pre_index)))
                ts_err(ts_dev->dev, "touch_health - frame cur_index:%d pre_index:%d, latency:%lldus",
                                    cur_index, pre_index, delta);
        pre_index = cur_index;

exit:
        enable_irq(core_data->irq);

#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

        if (PM_QOS_TOUCH_WAKEUP_VALUE == core_data->pm_qos_value) {
                core_data->pm_qos_value = PM_QOS_DEFAULT_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)

                cpu_latency_qos_remove_request(&core_data->pm_qos_req);
#else
                pm_qos_update_request(&core_data->pm_qos_req, core_data->pm_qos_value);
#endif
        }

#endif

        if (core_data->ws) {
                __pm_relax(core_data->ws);
        }

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        if (index >= 0 && core_data->boost_count <= board_data->max_boost_count) {
                struct cpu_boost_info *info = &core_data->boost_infos[index];
                freq_qos_update_request(&info->qos_req, 0);
        }

        mod_timer(&core_data->boost_timer,
                      jiffies + msecs_to_jiffies(board_data->boost_timeout));
#endif

        return IRQ_HANDLED;
}

/**
 * goodix_thp_irq_setup- Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_thp_irq_setup(struct goodix_thp_core *core_data)
{
        const struct goodix_thp_board_data *ts_bdata = board_data(core_data);
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        int r;
        struct irq_desc *desc;
        struct task_struct *task;
        struct sched_param param = { .sched_priority = 0 };

        /* if ts_bdata-> irq is invalid */
        if (ts_bdata->irq <= 0)
                core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
        else
                core_data->irq = ts_bdata->irq;

        ts_info(ts_dev->dev, "IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);

        if (core_data->pdev->id == 0)
                sprintf(core_data->irq_name, "%s", GOODIX_CORE_DRIVER_NAME);
        else
                sprintf(core_data->irq_name, "%s%d", GOODIX_CORE_DRIVER_NAME, core_data->pdev->id);
        r = devm_request_threaded_irq(&core_data->pdev->dev,
                                      core_data->irq, NULL,
                                      goodix_thp_threadirq_func,
                                      ts_bdata->irq_flags | IRQF_ONESHOT,
                                      core_data->irq_name,
                                      core_data);

        if (r < 0) {
                ts_err(ts_dev->dev, "Failed to requeset threaded irq:%d", r);
                return r;
        }

        //get irq description
        desc = irq_to_desc(core_data->irq);
        if (!desc) {
            return -EINVAL;
        }
        //get task_struct of kernel thread
        task = desc->action->thread;
        //set prio
        param.sched_priority = ts_bdata->sched_priority;
        sched_setscheduler_nocheck(task, SCHED_FIFO, &param);

        mutex_lock(&core_data->irq_mutex);
        disable_irq(core_data->irq);
        core_data->irq_state = false;
        mutex_unlock(&core_data->irq_mutex);
        ts_info(ts_dev->dev, "disable irq");

        return 0;
}

static int goodix_thp_pen_input_dev_init(struct goodix_thp_core *core_data)
{
        struct input_dev *pen_dev = NULL;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        int r;

        /* alloc input_dev */
        pen_dev = input_allocate_device();
        if (!pen_dev) {
                ts_err(ts_dev->dev, "Failed to alloc suspend input dev");
                return -ENOMEM;
        }
        core_data->pen_dev = pen_dev;

        /* init input_dev */
        if (core_data->pdev->id == 0)
                sprintf(core_data->pen_dev_name, "%s", GOODIX_THP_STYLUS_INPUT_DEVICE_NAME);
        else
                sprintf(core_data->pen_dev_name, "%s%d", GOODIX_THP_STYLUS_INPUT_DEVICE_NAME, core_data->pdev->id);
        pen_dev->name = core_data->pen_dev_name;
        pen_dev->id.bustype = BUS_SPI;
        pen_dev->id.product = 0x0210 + core_data->pdev->id;
        pen_dev->id.vendor = 0x27C6;
        pen_dev->id.version = 0x0001;

        /* set input_dev properties */
        set_bit(EV_SYN, pen_dev->evbit);
        set_bit(EV_KEY, pen_dev->evbit);
        set_bit(EV_ABS, pen_dev->evbit);
        set_bit(ABS_X, pen_dev->absbit);
        set_bit(ABS_Y, pen_dev->absbit);
        set_bit(ABS_TILT_X, pen_dev->absbit);
        set_bit(ABS_TILT_Y, pen_dev->absbit);
        set_bit(BTN_STYLUS, pen_dev->keybit);
        set_bit(BTN_STYLUS2, pen_dev->keybit);
        set_bit(BTN_STYLUS3, pen_dev->keybit);
        set_bit(BTN_TOUCH, pen_dev->keybit);
        set_bit(BTN_TOOL_PEN, pen_dev->keybit);
        set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);

	input_set_abs_params(pen_dev, ABS_X, 0,
                        core_data->ts_dev->board_data.panel_max_x - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0,
                        core_data->ts_dev->board_data.panel_max_y - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_PRESSURE, 0,
			core_data->ts_dev->board_data.panel_max_p - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_X,
			-GOODIX_PEN_MAX_TILT, GOODIX_PEN_MAX_TILT, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_Y,
			-GOODIX_PEN_MAX_TILT, GOODIX_PEN_MAX_TILT, 0, 0);

        /* register input_dev */
        r = input_register_device(pen_dev);
        if (r) {
                ts_err(ts_dev->dev, "failed to register suspend input device");
                return r;
        }

        return 0;
}

void goodix_thp_pen_input_dev_exit(struct goodix_thp_core *core_data)
{
        input_unregister_device(core_data->pen_dev);
        input_free_device(core_data->pen_dev);
}

static void goodix_thp_force_release_all(struct goodix_thp_core *core_data)
{
        struct input_dev *input_dev = core_data->input_dev;
        struct input_dev *pen_dev = core_data->pen_dev;
        int i;

        // release fingers
        for (i = 0; i < INPUT_AGENT_MAX_FINGERS; i++) {
                input_mt_slot(input_dev, i);
                input_mt_report_slot_state(input_dev, 0, 0);
        }
        input_report_key(input_dev, BTN_TOUCH, 0);
        input_report_key(input_dev, BTN_TOOL_FINGER, 0);
        input_sync(input_dev);

        //release stylus
        input_report_key(pen_dev, BTN_TOUCH, 0);
        input_report_key(pen_dev, BTN_TOOL_PEN, 0);
        input_sync(pen_dev);

        //clear previous finger state
        memset(core_data->prev_finger_state, 0, sizeof(core_data->prev_finger_state));

        //reinitialize pen action state
        core_data->pen_state = PEN_STATE_NONE;
}

/* Scale down pressure, ensure the result is > 0, and clamp to 255.*/
static inline int convert_pressure_to_touch_major(int pressure)
{
        int shifted = pressure >> MAJOR_CONVERSION_SHIFT;

        return (shifted >= TOUCH_MAJOR_CLAMP_THRESHOLD) ? TOUCH_MAJOR_MAX_VALUE : (shifted + 1);
}

static long goodix_thp_input_agent_ioctl_set_coordinate(struct goodix_thp_core *core_data, unsigned long arg)
{
        long ret = 0;
        void __user *argp = (void __user *)arg;
        struct input_dev *input_dev = core_data->input_dev;
        struct input_dev *pen_dev = core_data->pen_dev;
        struct thp_input_agent_ioctl_coor_data data;
        struct input_agent_coor_data *stylus_data = NULL;
        u8 i;
        static int pre_flags = 0;
        struct thp_ts_device *tdev = core_data->ts_dev;
        struct goodix_thp_board_data *board_data =
                        &core_data->ts_dev->board_data;
#if defined(CONFIG_ENABLE_GTP_PALM_CANCEL) || defined(CONFIG_ENABLE_GTP_PALM_CANCEL_BY_ID)
        unsigned int tool_type;
#endif
        int prev_state;
        int curr_state;
        int scaling_factor = board_data->resolution_boost;
        static unsigned int prev_stylus_key = 0;

        if (arg == 0) {
                ts_err(tdev->dev, "arg is null.");
                return -EINVAL;
        }

        /* copy data from hal */
        if (copy_from_user(&data, argp,
                        sizeof(struct thp_input_agent_ioctl_coor_data))) {
                ts_err(tdev->dev, "Failed to copy_from_user().");
                return -EFAULT;
        }

        if (data.touch_num > 0) {
            core_data->last_event_time = ktime_get_boottime();
        }

        if (data.touch[STYLUS_TRACK_ID].touch_valid == 1) {
                stylus_data = &data.touch[STYLUS_TRACK_ID];
                int is_hover = (stylus_data->p == 0) ? 1 : 0;
                int is_touch = !is_hover;

                // --- pen action state transfer detection start ---
                if (is_hover) {
                    //pen hover state
                    if (core_data->pen_state != PEN_STATE_HOVER) {
                        // from pen touch state to pen hover state
                        if (core_data->pen_state == PEN_STATE_TOUCH) {
                            ts_info(tdev->dev, "touch_health - pen_action=UP");
                        }
                        // record enter pen hover state
                        ts_info(tdev->dev, "touch_health - pen_action=HOVER_ENTER x=%d y=%d",
                                    CALC_ACTUAL_COORD(stylus_data->x, scaling_factor),
                                    CALC_ACTUAL_COORD(stylus_data->y, scaling_factor));
                        core_data->pen_state = PEN_STATE_HOVER;
                        core_data->pen_close = 1;
                        core_data->uevent_message_type = PEN_MESSAGE_PEN_CLOSE;
                        kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                    }
                } else if (is_touch) {
                    // pen touch state
                    if (core_data->pen_state != PEN_STATE_TOUCH) {
                        // from pen hover state to pen touch state
                        if (core_data->pen_state == PEN_STATE_HOVER) {
                            ts_info(tdev->dev, "touch_health - pen_action=HOVER_EXIT");
                        }
                        // record enter pen touch state
                        ts_info(tdev->dev, "touch_health - pen_action=DOWN x=%d y=%d pressure=%d",
                                CALC_ACTUAL_COORD(stylus_data->x, scaling_factor),
                                CALC_ACTUAL_COORD(stylus_data->y, scaling_factor),
                                stylus_data->p);
                        core_data->pen_state = PEN_STATE_TOUCH;
                    }
                }
                // --- pen action state transfer detection end ---

                // release all fingers
                for (i = 0; i < INPUT_AGENT_MAX_FINGERS; i++) {
                        input_mt_slot(input_dev, i);
                        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
                }
                input_report_key(input_dev, BTN_TOUCH, 0);
                input_report_key(input_dev, BTN_TOOL_FINGER, 0);
                input_sync(input_dev);
                // report stylus
                ts_debug(tdev->dev, "x:%d y:%d p:%d tilt_x:%d tilt_y:%d",
                        stylus_data->x, stylus_data->y, stylus_data->p,
                        stylus_data->tilt_x, stylus_data->tilt_y);
                input_report_abs(pen_dev, ABS_X, stylus_data->x);
                input_report_abs(pen_dev, ABS_Y, stylus_data->y);
                input_report_abs(pen_dev, ABS_TILT_X, stylus_data->tilt_x);
                input_report_abs(pen_dev, ABS_TILT_Y, stylus_data->tilt_y);
                input_report_abs(pen_dev, ABS_PRESSURE, stylus_data->p);
                input_report_key(pen_dev, BTN_TOUCH, data.hover_stat ? 0 : 1);
                input_report_key(pen_dev, BTN_TOOL_PEN, 1);

                input_report_key(pen_dev, BTN_STYLUS, data.stylus_key & 0x02);
                input_report_key(pen_dev, BTN_STYLUS2, data.stylus_key & 0x04);
                input_report_key(pen_dev, BTN_STYLUS3, data.stylus_key & 0x08);
                input_sync(pen_dev);

                if (prev_stylus_key != data.stylus_key) {
                    if ((prev_stylus_key & 0x02) != (data.stylus_key & 0x02)) {
                        if (data.stylus_key & 0x02) {
                            ts_info(tdev->dev, "touch_health - BTN_STYLUS DOWN");
                        } else {
                            ts_info(tdev->dev, "touch_health - BTN_STYLUS UP");
                        }
                    }
                    prev_stylus_key = data.stylus_key;
                }
        } else {
                // --- release pen action detection start ---
                if (core_data->pen_state == PEN_STATE_HOVER) {
                    ts_info(tdev->dev, "touch_health - pen_action=HOVER_EXIT");
                    core_data->pen_state = PEN_STATE_NONE;
                    core_data->pen_close = 0;
                    core_data->uevent_message_type = PEN_MESSAGE_PEN_CLOSE;
                    kobject_uevent(&core_data->pdev->dev.kobj, KOBJ_CHANGE);
                } else if (core_data->pen_state == PEN_STATE_TOUCH) {
                    ts_info(tdev->dev, "touch_health - pen_action=UP");
                    core_data->pen_state = PEN_STATE_NONE;
                }
                // --- release pen action detection end ---

                if (prev_stylus_key != 0) {
                    ts_info(tdev->dev, "touch_health - PEN REMOVED, reset all stylus keys");
                    prev_stylus_key = 0;
                }

                // release stylus
                input_report_key(pen_dev, BTN_TOUCH, 0);
                input_report_key(pen_dev, BTN_TOOL_PEN, 0);
                input_sync(pen_dev);
                // report fingers
                if (data.ref_not_set == 0) {
                    for (i = 0; i < INPUT_AGENT_MAX_FINGERS; i++) {
                        input_mt_slot(input_dev, i);

                        //convert finger pressure to major
                        data.touch[i].major = convert_pressure_to_touch_major(data.touch[i].p);

                        // --- check and print state change ---
                        prev_state = core_data->prev_finger_state[i];
                        curr_state = (data.touch[i].touch_valid != 0);

                        if (prev_state != curr_state) {
                            if (curr_state) {
                                // DOWN event：print finger coord and major
                                ts_info(tdev->dev, "touch_health - Finger[%d] DOWN: x=%d, y=%d, major=%d",
                                    i,
                                    CALC_ACTUAL_COORD(data.touch[i].x, scaling_factor),
                                    CALC_ACTUAL_COORD(data.touch[i].y, scaling_factor),
                                    data.touch[i].major);
                            } else {
                                // UP event
                                ts_info(tdev->dev, "touch_health - Finger[%d] UP", i);
                            }
                            // update state
                            core_data->prev_finger_state[i] = curr_state;
                        }
                        // --- check state change end ---

#ifdef CONFIG_ENABLE_GTP_PALM_CANCEL
                        tool_type = data.large_touch_stat ? MT_TOOL_PALM : MT_TOOL_FINGER;
#endif
#ifdef CONFIG_ENABLE_GTP_PALM_CANCEL_BY_ID
                        if ((tool_type != MT_TOOL_PALM) && data.touch[i].cancel_flag)
                            tool_type = MT_TOOL_PALM;
#endif

#if defined(CONFIG_ENABLE_GTP_PALM_CANCEL) || defined(CONFIG_ENABLE_GTP_PALM_CANCEL_BY_ID)
                        input_mt_report_slot_state(input_dev, tool_type, data.touch[i].touch_valid != 0);
#else
                        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, data.touch[i].touch_valid != 0);
#endif

                        if (data.touch[i].touch_valid != 0) {
#ifdef CONFIG_ENABLE_GTP_PALM_CANCEL_BY_ID
                                ts_debug(tdev->dev, "[%d] x:%d y:%d w:%d, id_palm %d, tool_type %d", i,
                                        data.touch[i].x, data.touch[i].y, data.touch[i].major,
                                        data.touch[i].cancel_flag, tool_type);
#else
                                ts_debug(tdev->dev, "[%d] x:%d y:%d w:%d", i,
                                        data.touch[i].x, data.touch[i].y, data.touch[i].major);
#endif
                                input_report_abs(input_dev, ABS_MT_POSITION_X,
                                        data.touch[i].x);
                                input_report_abs(input_dev, ABS_MT_POSITION_Y,
                                        data.touch[i].y);
                                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
                                        data.touch[i].major);
                                //	input_report_abs(input_dev, ABS_MT_TOUCH_MINOR,
                                //				data.touch[i].minor);
                                trace_touch_coord((int)(CALC_ACTUAL_COORD(data.touch[i].x, scaling_factor)), (int)(CALC_ACTUAL_COORD(data.touch[i].y, scaling_factor)));
                        }
                    }
                    input_report_key(input_dev, BTN_TOUCH, (data.touch_num > 0) ? 1 : 0);
                    input_report_key(input_dev, BTN_TOOL_FINGER, (data.touch_num > 0) ? 1 : 0);
                    input_sync(input_dev);
            }
        }

        /* fp touch flag */
        if (pre_flags != data.fp_mode) {
                if (data.fp_mode) {
                        //tdev->hw_ops->set_fp_int_pin(tdev, 1);
                        input_report_key(input_dev, BTN_TRIGGER_HAPPY1, 1);
                        input_sync(input_dev);
                        input_report_key(input_dev, BTN_TRIGGER_HAPPY1, 0);
                        input_sync(input_dev);
                        ts_info(tdev->dev, "report BTN_TRIGGER_HAPPY1");
                } else {
                        //tdev->hw_ops->set_fp_int_pin(tdev, 0);
                        input_report_key(input_dev, BTN_TRIGGER_HAPPY2, 1);
                        input_sync(input_dev);
                        input_report_key(input_dev, BTN_TRIGGER_HAPPY2, 0);
                        input_sync(input_dev);
                        ts_info(tdev->dev, "report BTN_TRIGGER_HAPPY2");
                }
                pre_flags = data.fp_mode;
        }

        return ret;
}

static int goodix_thp_input_agent_ioctl_read_status(struct goodix_thp_core *core_data, unsigned long arg)
{
        return 0;
}

static int goodix_thp_input_agent_ioctl_get_custom_info(struct goodix_thp_core *core_data, unsigned long arg)
{
        char __user *custom_info = (char *)arg;
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;

        if (!cd || !custom_info) {
                ts_err(ts_dev->dev, "args error");
                return -EINVAL;
        }

        ts_info(ts_dev->dev, "custom info:%s", cd->custom_info);

        if(copy_to_user(custom_info, cd->custom_info, sizeof(cd->custom_info))) {
                ts_err(ts_dev->dev, "copy window_info failed");
                return -EFAULT;
        }

        return 0;
}

static long goodix_thp_input_agent_ioctl_set_events(struct goodix_thp_core *core_data, unsigned long arg)
{
        long ret = 0;

        return ret;
}

int goodix_thp_input_agent_ioctl_get_events(struct goodix_thp_core *core_data, unsigned long arg)
{
        return 0;
}

static int goodix_thp_input_agent_ioctl_get_driver_state(struct goodix_thp_core *core_data, unsigned long arg)
{
        struct goodix_thp_core *cd = core_data;
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u32 __user *driver_state = (u32 *)arg;

        //ts_info("%s:driver state = %d", __func__, cd->suspended);

        if (driver_state == NULL) {
                ts_err(ts_dev->dev, "input parameter null");
                return -EINVAL;
        }

        if(copy_to_user(driver_state, &cd->suspended, sizeof(u32))) {
                ts_err(ts_dev->dev, "copy driver_state failed");
                return -EFAULT;
        }

        return 0;
}

static int goodix_thp_input_agent_open(struct inode *inode, struct file *filp)
{
        return 0;
}

static int goodix_thp_input_agent_release(struct inode *inode,
                                                struct file *filp)
{
        return 0;
}

static long goodix_thp_input_agent_ioctl(struct file *filp, unsigned int cmd,
                                unsigned long arg)
{
        struct goodix_thp_core *cd =
                container_of(filp->private_data, struct goodix_thp_core, input_misc_dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        long ret;

        switch (cmd) {
        case INPUT_AGENT_IOCTL_CMD_SET_COOR:
                ret = goodix_thp_input_agent_ioctl_set_coordinate(cd, arg);
                break;
        case INPUT_AGENT_IOCTL_READ_STATUS:
                ret = goodix_thp_input_agent_ioctl_read_status(cd, arg);
                break;
        case INPUT_AGENT_IOCTL_GET_CUSTOM_INFO:
                ret = goodix_thp_input_agent_ioctl_get_custom_info(cd, arg);
                break;
        case INPUT_AGENT_IOCTL_CMD_SET_EVENTS:
                ret = goodix_thp_input_agent_ioctl_set_events(cd, arg);
                break;
        case INPUT_AGENT_IOCTL_CMD_GET_EVENTS:
                ret = goodix_thp_input_agent_ioctl_get_events(cd, arg);
                break;
        case INPUT_AGENT_IOCTL_GET_DRIVER_STATE:
                ret = goodix_thp_input_agent_ioctl_get_driver_state(cd, arg);
                break;
        default:
                ts_err(ts_dev->dev, "cmd unkown.");
                ret = -EINVAL;
        }

        return ret;
}

static const struct file_operations g_thp_input_agent_fops = {
        .owner = THIS_MODULE,
        .open = goodix_thp_input_agent_open,
        .release = goodix_thp_input_agent_release,
        .unlocked_ioctl = goodix_thp_input_agent_ioctl,
};

static int goodix_thp_input_agent_init(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct input_dev *input_dev;
        int r;

        /* alloc input_dev */
        input_dev = input_allocate_device();
        if (!input_dev) {
                ts_err(ts_dev->dev, "Unable to allocated input device");
                return	-ENODEV;
        }
        core_data->input_dev = input_dev;

        /* init input_dev */
        if (core_data->pdev->id == 0)
                sprintf(core_data->input_dev_name, "%s", GOODIX_THP_INPUT_DEVICE_NAME);
        else
                sprintf(core_data->input_dev_name, "%s%d", GOODIX_THP_INPUT_DEVICE_NAME, core_data->pdev->id);
        input_dev->name = core_data->input_dev_name;
        input_dev->phys = core_data->input_dev_name;
        input_dev->id.bustype = BUS_SPI;
        input_dev->id.product = 0xDEAD;
        input_dev->id.vendor = 0xBEEF;
        input_dev->id.version = 10427;

        /* set input_dev properties */
        set_bit(EV_SYN, input_dev->evbit);
        set_bit(EV_KEY, input_dev->evbit);
        set_bit(EV_ABS, input_dev->evbit);
        set_bit(BTN_TOUCH, input_dev->keybit);
        set_bit(BTN_TOOL_FINGER, input_dev->keybit);
        set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

        input_set_abs_params(input_dev, ABS_MT_POSITION_X,
                        0, core_data->ts_dev->board_data.panel_max_x - 1, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
                        0, core_data->ts_dev->board_data.panel_max_y - 1, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
                        0, core_data->ts_dev->board_data.panel_max_w - 1, 0, 0);
        input_mt_init_slots(input_dev, INPUT_AGENT_MAX_FINGERS, INPUT_MT_DIRECT);
#ifdef CONFIG_ENABLE_GTP_PALM_CANCEL
        input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
                        MT_TOOL_FINGER, MT_TOOL_PALM, 0, 0);
#endif

        // gesture
        input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
        input_set_capability(input_dev, EV_KEY, KEY_GOTO);

        input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1);
        input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2);

        /* register input_dev */
        r = input_register_device(input_dev);
        if (r) {
                ts_err(ts_dev->dev, "failed to register input device");
                goto input_dev_reg_err;
        }

        if (core_data->pdev->id == 0)
                sprintf(core_data->input_misc_name, "%s", DEVICE_NAME);
        else
                sprintf(core_data->input_misc_name, "%s%d", DEVICE_NAME, core_data->pdev->id);
        core_data->input_misc_dev.minor = MISC_DYNAMIC_MINOR;
        core_data->input_misc_dev.name = core_data->input_misc_name;
        core_data->input_misc_dev.fops = &g_thp_input_agent_fops;
        r = misc_register(&core_data->input_misc_dev);
        if (r) {
                ts_err(ts_dev->dev, "failed to register misc device");
                goto misc_dev_reg_err;
        }

        return 0;

misc_dev_reg_err:
        input_unregister_device(input_dev);
input_dev_reg_err:
        return r;
}

static void goodix_thp_input_agent_exit(struct goodix_thp_core *core_data)
{
        input_unregister_device(core_data->input_dev);
        misc_deregister(&core_data->input_misc_dev);
}

/* Description:switch scan_rate
 * @buf: 0/1/2/3/4 represent 300/240/180/120/60hz
 */
static ssize_t goodix_thp_scan_rate_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *core_data = dev_get_drvdata(dev);
        struct thp_ts_device *tdev = core_data->ts_dev;
        int index = 0;

        if (sscanf(buf, "%d", &index) != 1)
                return -EINVAL;

        if (tdev->hw_ops->send_cmd(tdev, CMD_ACTIVE_SCAN_RATE, index))
                ts_err(tdev->dev,"goodix switch scan rate failed, index %d", index);

        return count;
}

/* Description: read driver version
 */
static ssize_t goodix_thp_driver_info_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{

        return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
                        GOODIX_THP_DRIVER_VERSION);

}

/* show irq infomation */
static ssize_t goodix_thp_irq_info_show(struct device *dev,
                       struct device_attribute *attr,
                       char *buf)
{
        struct goodix_thp_core *core_data = dev_get_drvdata(dev);
        struct goodix_thp_board_data *ts_bdata = board_data(core_data);
        struct irq_desc *desc;
        size_t offset = 0;
        int r;

        r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
        if (r < 0)
                return -EINVAL;

        offset += r;
        r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
             (core_data->irq_state) ?
             "enabled" : "disabled");
        if (r < 0)
                return -EINVAL;

        desc = irq_to_desc(core_data->irq);
        if (desc) {
                offset += r;
                r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
                        desc->depth);
                if (r < 0)
                return -EINVAL;
        }

        offset += r;
        r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
                core_data->irq_trig_cnt);
        if (r < 0)
                return -EINVAL;

        offset += r;
        r = snprintf(&buf[offset], PAGE_SIZE - offset, "irq gpio level:%s\n",
                (gpio_get_value(ts_bdata->irq_gpio) ?"HIGH" : "LOW"));
        if (r < 0)
                return -EINVAL;

        offset += r;
        return offset;
}

/* Description: debug read
 */
static ssize_t goodix_thp_debug_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s\n", debug_log_flag ? "enable" : "disabled");
}

/* Description: debug write
 */
static ssize_t goodix_thp_debug_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] == '0' || buf[0] == 0)
		debug_log_flag = false;
	else
		debug_log_flag = true;
	return count;
}

static ssize_t goodix_thp_screen_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        size_t offset;

        offset = sprintf(buf, "%s\n", cd->suspended ? "off" : "on");
        return offset;
}

static ssize_t goodix_thp_screen_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        u8 val[2];

        val[0] = NOTIFY_TYPE_SCREEN;

        if (buf[0] == '0' || buf[0] == 0)
                val[1] = 0;
        else
                val[1] = 1;
        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
        return count;
}

static ssize_t goodix_thp_gesture_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 val[3];

        if (count < 2) {
                ts_err(ts_dev->dev, "invalid input param len:%zu", count);
                return count;
        }

        val[0] = NOTIFY_TYPE_GESTURE;
        val[1] = buf[0];
        val[2] = buf[1];
        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
        return count;
}

static ssize_t goodix_thp_tsd_ctrl_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        goodix_thp_ioctl_set_tsd_state(cd, buf[0] != '0');
        return count;
}

static ssize_t goodix_thp_dump_rep_log_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 val[1] = {NOTIFY_TYPE_DUMP_REP};

        if (buf[0] == '1' || buf[0] == 1) {
                ts_info(ts_dev->dev, "dump rep log");
                put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 1);
        }
        return count;
}

static ssize_t goodix_thp_stylus_ctrl_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        goodix_thp_ioctl_set_stylus_state(cd, buf[0] != '0');
        return count;
}

static ssize_t goodix_thp_rawdata_ctrl_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        u8 val[2] = {NOTIFY_TYPE_RAWDATA, 0};

        if (buf[0] == 1 || buf[0] == '1')
                val[1] = 1;

        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 2);
        return count;
}

static ssize_t goodix_thp_version_info(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        return snprintf(buf, PAGE_SIZE, "%s\n", cd->ts_dev->board_data.thp_ver);
}

static ssize_t goodix_thp_logtofile_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        return sprintf(buf, "%s\n", cd->logtofile_on ? "on" : "off");
}

static ssize_t goodix_thp_logtofile_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        u8 val[2] = {NOTIFY_TYPE_LOGTOFILE, 0};

        cd->logtofile_on = 0;
        if (buf[0] == 1 || buf[0] == '1') {
                val[1] = 1;
                cd->logtofile_on = 1;
        }

        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 2);
        return count;
}

/* reg read/write */
static u32 rw_addr;
static u32 rw_len;
static u8 rw_flag;
static u8 store_buf[32];
static u8 show_buf[PAGE_SIZE];
static ssize_t goodix_thp_reg_rw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
	int ret;

	if (!rw_addr || !rw_len) {
		ts_err(ts_dev->dev, "address(0x%x) and length(%d) can't be null",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err(ts_dev->dev, "invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = ts_dev->hw_ops->read(ts_dev, rw_addr, show_buf, rw_len);
	if (ret < 0) {
		ts_err(ts_dev->dev, "failed read addr(%x) length(%d)", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
			"failed read addr(%x), len(%d)\n",
			rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n",
		rw_addr, rw_len, rw_len, show_buf);
}

static ssize_t goodix_thp_reg_rw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
	char *pos = NULL;
	char *token = NULL;
	long result = 0;
	int i;

	if (!buf || !count) {
		ts_err(ts_dev->dev, "invalid parame");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err(ts_dev->dev, "string must start with 'r/w'");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err(ts_dev->dev, "invalid address info");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err(ts_dev->dev, "failed get addr info");
			goto err_out;
		}
		rw_addr = (u32)result;
		ts_info(ts_dev->dev, "rw addr is 0x%x", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err(ts_dev->dev, "invalid length info");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err(ts_dev->dev, "failed get length info");
			goto err_out;
		}
		rw_len = (u32)result;
		if (rw_len > sizeof(store_buf)) {
			ts_err(ts_dev->dev, "data len > %lu", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err(ts_dev->dev, "invalid data info");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err(ts_dev->dev, "failed get data[%d] info", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
		}
	}

        if (rw_addr == ts_dev->board_data.cmd_addr) {
                put_frame_list(cd, REQUEST_TYPE_CMD, store_buf, rw_len);
        } else {
                ts_dev->hw_ops->write(ts_dev, rw_addr, store_buf, rw_len);
        }

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;
}

static ssize_t goodix_thp_special_area_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        return sprintf(buf, "%s\n", cd->special_area_on ? "enable" : "disabled");
}

static ssize_t goodix_thp_special_area_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 en_cmd[] = {0x00, 0x00, 0x05, 0x37, 0x01, 0x3D, 0x00};
        u8 dis_cmd[] = {0x00, 0x00, 0x05, 0x37, 0x00, 0x3C, 0x00};
        u8 val[512];

        if (count > sizeof(val) - 1) {
                ts_err(ts_dev->dev, "special area param len[%zu] > limit[%lu]", count, sizeof(val) - 1);
                return -EINVAL;
        }

        if (buf[0] == 0) {
                ts_info(ts_dev->dev, "close special area");
                cd->special_area_on = false;
                put_frame_list(cd, REQUEST_TYPE_CMD, dis_cmd, sizeof(dis_cmd));
        } else {
                ts_info(ts_dev->dev, "open special area");
                cd->special_area_on = true;
                if (count > 3) { //set special area param
                        val[0] = NOTIFY_TYPE_SPECIAL_AREA;
                        memcpy(&val[1], buf + 1, count - 1);
                        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, count);
                } else {        // only open
                        put_frame_list(cd, REQUEST_TYPE_CMD, en_cmd, sizeof(en_cmd));
                }
        }

        return count;
}

static ssize_t goodix_thp_esd_info_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *thp_core = dev_get_drvdata(dev);

        return sprintf(buf, "%s\n", thp_core->esd_on ? "enable" : "disabled");
}

static ssize_t goodix_thp_esd_info_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *thp_core = dev_get_drvdata(dev);

        if (!buf || count <= 0)
                return -EINVAL;

        if (buf[0] == 0 || buf[0] == '0')
                goodix_thp_esd_on(thp_core, false);
        else
                goodix_thp_esd_on(thp_core, true);

        return count;
}

static ssize_t goodix_thp_save_moto_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);

        return sprintf(buf, "%s\n", cd->save_moto_data_on ? "enable" : "disabled");
}

static ssize_t goodix_thp_save_moto_data_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
        struct goodix_thp_core *cd = dev_get_drvdata(dev);
        u8 val[2] = {NOTIFY_TYPE_SAVE_MOTO_DATA, 0};

        if (buf[0] == 1 || buf[0] == '1') {
                val[1] = 1;
                cd->save_moto_data_on = true;
        } else {
                val[1] = 0;
                cd->save_moto_data_on = false;
        }

        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
        ts_info(cd->ts_dev->dev, "save moto data %s", cd->save_moto_data_on ? "enable" : "disable");
        return count;
}

static DEVICE_ATTR(scan_rate, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_scan_rate_store);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_thp_driver_info_show, NULL);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_debug_show, goodix_thp_debug_store);
static DEVICE_ATTR(screen_state, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_screen_show, goodix_thp_screen_store);
static DEVICE_ATTR(gesture_enable, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_gesture_store);
static DEVICE_ATTR(tsd_ctrl, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_tsd_ctrl_store);
static DEVICE_ATTR(dump_rep_log, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_dump_rep_log_store);
static DEVICE_ATTR(stylus_ctrl, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_stylus_ctrl_store);
static DEVICE_ATTR(rawdata_ctrl, S_IWUSR | S_IWGRP, NULL,
                                goodix_thp_rawdata_ctrl_store);
static DEVICE_ATTR(version_info, S_IRUGO, goodix_thp_version_info, NULL);
static DEVICE_ATTR(logtofile, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_logtofile_show, goodix_thp_logtofile_store);
static DEVICE_ATTR(reg_rw, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_reg_rw_show, goodix_thp_reg_rw_store);
static DEVICE_ATTR(special_area, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_special_area_show, goodix_thp_special_area_store);
static DEVICE_ATTR(save_moto_data, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_save_moto_data_show, goodix_thp_save_moto_data_store);
static DEVICE_ATTR(esd_info, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_esd_info_show, goodix_thp_esd_info_store);
static DEVICE_ATTR(irq_info, S_IRUGO, goodix_thp_irq_info_show, NULL);

static struct attribute *sysfs_attrs[] = {
        &dev_attr_scan_rate.attr,
        &dev_attr_driver_info.attr,
        &dev_attr_debug.attr,
        &dev_attr_screen_state.attr,
        &dev_attr_gesture_enable.attr,
        &dev_attr_tsd_ctrl.attr,
        &dev_attr_dump_rep_log.attr,
        &dev_attr_stylus_ctrl.attr,
        &dev_attr_rawdata_ctrl.attr,
        &dev_attr_version_info.attr,
        &dev_attr_logtofile.attr,
        &dev_attr_reg_rw.attr,
        &dev_attr_special_area.attr,
        &dev_attr_save_moto_data.attr,
        &dev_attr_esd_info.attr,
        &dev_attr_irq_info.attr,
        NULL,
};

static const struct attribute_group sysfs_group = {
        .attrs = sysfs_attrs,
};

static int goodix_thp_sysfs_init(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        int ret;

        ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
        if (ret) {
                ts_err(ts_dev->dev, "failed create core sysfs group");
                return ret;
        }

        return ret;
}

static void goodix_thp_sysfs_exit(struct goodix_thp_core *core_data)
{
        sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

static int goodix_thp_suspend(struct goodix_thp_core *core_data)
{
        int r = 0;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        u16 gsx_data = ~core_data->gesture_enable;

        ts_info(ts_dev->dev, "Suspend start");

        if (unlikely(core_data->suspended == 1)) {
                ts_info(ts_dev->dev, "Already in suspend mode, exit.");
                goto exit;
        }

        goodix_thp_set_irq_enable(core_data, IRQ_DISABLE_FLAG);
        core_data->suspended = 1;
#ifdef CONFIG_TOUCHCLASS_MMI_FORCE_ENTER_STANDBY
        if (core_data->pdev->id == 0)
            main_suspend = true;
#endif
        core_data->state_change_flag = 0;
        if (core_data->esd_on)
                cancel_delayed_work_sync(&core_data->esd_work);

        if (core_data->gesture_enable == 0) {
                /* power off */
                goodix_thp_power_off(core_data);
        } else {
                ts_info(ts_dev->dev, "enter gesture mode!");
                /* send enter gesture cmd */
                r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_GESTURE, gsx_data);
                if (unlikely(r)) {
                        ts_err(ts_dev->dev, "send enter gesture cmd failed, r %d", r);
                        goto exit;
                }
                goodix_thp_set_irq_enable(core_data, IRQ_ENABLE_FLAG);
                goodix_thp_set_irq_wake_enable(core_data, IRQ_WAKE_ENABLE_FLAG);
        }
exit:
        goodix_thp_force_release_all(core_data);
        ts_info(ts_dev->dev, "Suspend end");
        return r;
}

static int goodix_thp_resume(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;

        ts_info(ts_dev->dev, "Resume start");

        if (unlikely(core_data->suspended == 0)) {
                ts_info(ts_dev->dev, "Already in normal mode,exit.");
                goto exit;
        }

        goodix_thp_set_irq_enable(core_data, IRQ_DISABLE_FLAG);

        if (core_data->gesture_enable == 0) {
                /* power on */
                goodix_thp_power_on(core_data);
                msleep(100);
        } else {
                goodix_thp_set_irq_wake_enable(core_data, IRQ_WAKE_DISABLE_FLAG);
                ts_dev->hw_ops->reset(ts_dev, 100);
        }

        core_data->suspended = 0;
#ifdef CONFIG_TOUCHCLASS_MMI_FORCE_ENTER_STANDBY
        if (core_data->pdev->id == 0)
            main_suspend = false;
#endif
        core_data->state_change_flag = 1;
        if (core_data->esd_on)
                schedule_delayed_work(&core_data->esd_work, GOODIX_ESD_CHECK_INTERVAL);
exit:
        goodix_thp_set_irq_enable(core_data, IRQ_ENABLE_FLAG);
        ts_info(ts_dev->dev, "Resume end");
        return 0;
}

#ifdef CONFIG_TOUCHCLASS_MMI_FORCE_ENTER_STANDBY
int goodix_thp_off_to_gesture(struct goodix_thp_core *core_data)
{
        int r = 0;
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        u16 gsx_data = ~core_data->gesture_enable;

        ts_info(ts_dev->dev, "Resume start");

        goodix_thp_set_irq_enable(core_data, IRQ_DISABLE_FLAG);
            /* power on */
        goodix_thp_power_on(core_data);
        msleep(100);

        core_data->suspended = 1;
        /* send enter gesture cmd */
        ts_info(ts_dev->dev, "enter gesture mode!");
        /* send enter gesture cmd */
        r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_GESTURE, gsx_data);
        if (r) {
                ts_err(ts_dev->dev, "send enter gesture cmd failed, r %d", r);
                goto exit;
        }
        goodix_thp_set_irq_enable(core_data, IRQ_ENABLE_FLAG);
        goodix_thp_set_irq_wake_enable(core_data, IRQ_WAKE_ENABLE_FLAG);
exit:
        goodix_thp_force_release_all(core_data);
        ts_info(ts_dev->dev, "Suspend end");
        return r;
}
#endif
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
static int goodix_thp_drm_notifier_callback(struct notifier_block *nb,
	unsigned long value, void *v)
{
	struct goodix_thp_core *cd =
                container_of(nb, struct goodix_thp_core, pm_notif);
        struct thp_ts_device *ts_dev = cd->ts_dev;
	int *data = (int *)v;
        u8 val[2];

        if (!cd || !v) {
                ts_err(ts_dev->dev, "invalid parameters");
                return -1;
        }

        if (value == MTK_DISP_EVENT_BLANK) {
                /* resume: touch power on is after display to avoid display disturb */
                ts_info(ts_dev->dev, "IN, MTK_DISP_EVENT_BLANK");
                if (*data == MTK_DISP_BLANK_UNBLANK) {
                        val[0] = NOTIFY_TYPE_SCREEN;
                        val[1] = 1;
                        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
                }
                ts_info(ts_dev->dev, "OUT");
        } else if (value == MTK_DISP_EARLY_EVENT_BLANK) {
                /**
                 * suspend: touch power off is before display to avoid touch report event
                 * after screen is off
                 */
                ts_info(ts_dev->dev, "IN, MTK_DISP_EARLY_EVENT_BLANK");
                if (*data == MTK_DISP_BLANK_POWERDOWN) {
                        val[0] = NOTIFY_TYPE_SCREEN;
                        val[1] = 0;
                        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
                }
                ts_info(ts_dev->dev, "OUT");
        } else {
                ts_info(ts_dev->dev, "ignore disp value %d, data %d", value, *data);
        }

	return 0;
}
#elif IS_ENABLED(CONFIG_FB)
static int goodix_thp_fb_notifier_callback(struct notifier_block *self,
                 unsigned long event, void *data)
{
        struct fb_event *evdata = data;
        int *blank = NULL;
        struct goodix_thp_core *cd = container_of(self, struct goodix_thp_core,
                pm_notif);
        struct thp_ts_device *ts_dev = cd->ts_dev;
        u8 val[2];

        blank = evdata->data;
        ts_info(ts_dev->dev, "FB event:%lu,blank:%d", event, *blank);
	if (event == FB_EVENT_BLANK) {
		if (*blank == FB_BLANK_UNBLANK) {
                        val[0] = NOTIFY_TYPE_SCREEN;
                        val[1] = 1;
                        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		} else if (*blank == FB_BLANK_POWERDOWN) {
                        val[0] = NOTIFY_TYPE_SCREEN;
                        val[1] = 0;
                        put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		}
	}

        return 0;
}
#endif

int goodix_thp_enter_tui(void)
{
        //TODO: enter tui mode
        return 0;
}
EXPORT_SYMBOL_GPL(goodix_thp_enter_tui);

int goodix_thp_exit_tui(void)
{
        //TODO: exit tui mode
        return 0;
}
EXPORT_SYMBOL_GPL(goodix_thp_exit_tui);

static void goodix_thp_esd_work(struct work_struct *work)
{
        struct delayed_work *dwork = to_delayed_work(work);
        struct goodix_thp_core *core_data =
                container_of(dwork, struct goodix_thp_core, esd_work);
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        u32 esd_addr = core_data->ts_dev->board_data.esd_addr;
        u8 esd_value;

        if (!core_data->esd_on || esd_addr == 0)
                return;

        if (!core_data->suspended) { /* Don't perform SPI operations while suspended */
                ts_dev->hw_ops->read(ts_dev, esd_addr, &esd_value, 1);
                if (esd_value == GOODIX_ESD_TICK_WRITE_DATA) {
                        ts_err(ts_dev->dev, "esd check failed, 0x%x", esd_value);
                        goodix_thp_power_off(core_data);
                        msleep(200);
                        goodix_thp_power_on(core_data);
                } else {
                        esd_value = GOODIX_ESD_TICK_WRITE_DATA;
                        ts_dev->hw_ops->write(ts_dev, esd_addr, &esd_value, 1);
                }

                schedule_delayed_work(dwork, GOODIX_ESD_CHECK_INTERVAL);
        }
}

static int goodix_thp_esd_init(struct goodix_thp_core *core_data)
{
        if (core_data->ts_dev->board_data.esd_enable) {
                ts_info(core_data->ts_dev->dev, "ESD work init");
                INIT_DELAYED_WORK(&core_data->esd_work, goodix_thp_esd_work);
        } else {
                ts_info(core_data->ts_dev->dev, "ESD function is not enabled");
        }

        return 0;
}

static int goodix_ts_pinctrl_select_active(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct device *dev = ts_dev->dev;
        int ret = 0;

        if (core_data->pinctrl && core_data->pin_sta_active) {
                ret = pinctrl_select_state(core_data->pinctrl, core_data->pin_sta_active);
                if (ret < 0) {
                        ts_err(dev, "Set active pin state error:%d", ret);
                } else {
                        ts_info(dev, "Set active pin state success:%d", ret);
                }
        }

        return ret;
}

static int goodix_ts_pinctrl_select_suspend(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct device *dev = ts_dev->dev;
        int ret = 0;

        if (core_data->pinctrl && core_data->pin_sta_suspend) {
                ret = pinctrl_select_state(core_data->pinctrl, core_data->pin_sta_suspend);
                if (ret < 0) {
                        ts_err(dev, "Set suspend pin state error:%d", ret);
                } else {
                        ts_info(dev, "Set suspend pin state success:%d", ret);
                }
        }

        return ret;
}

/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct device *dev = ts_dev->dev;
        int r = 0;

        /* get pinctrl handler from of node */
        core_data->pinctrl = devm_pinctrl_get(dev);
        if (IS_ERR_OR_NULL(core_data->pinctrl)) {
                ts_err(ts_dev->dev, "Failed to get pinctrl handler[need confirm]");
                core_data->pinctrl = NULL;
                return -EINVAL;
        }
        ts_info(ts_dev->dev, "success get pinctrl");

        /* active state */
        core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_STATE_ACTIVE);
        if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
                r = PTR_ERR(core_data->pin_sta_active);
                ts_err(ts_dev->dev, "Failed to get pinctrl state:%s, r:%d",
                                PINCTRL_STATE_ACTIVE, r);
                core_data->pin_sta_active = NULL;
                goto exit_pinctrl_put;
        }
        ts_info(dev, "success get active pinctrl state");

        /* suspend state */
        core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_STATE_SUSPEND);
        if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
                r = PTR_ERR(core_data->pin_sta_suspend);
                ts_err(ts_dev->dev, "Failed to get pinctrl state:%s, r:%d",
                                PINCTRL_STATE_SUSPEND, r);
                core_data->pin_sta_suspend = NULL;
                goto exit_pinctrl_put;
        }
        ts_info(dev, "success get suspend pinctrl state");

        return 0;
exit_pinctrl_put:
        devm_pinctrl_put(core_data->pinctrl);
        core_data->pinctrl = NULL;
        return r;
}

static int goodix_ts_stylus_clk_init(struct goodix_thp_core *core_data)
{
        struct thp_ts_device *ts_dev = core_data->ts_dev;
        struct device *dev = ts_dev->dev;
        int r = 0;

        if (IS_ERR_OR_NULL(core_data->pinctrl)) {
                ts_info(dev, "Failed to get pinctrl handler[need confirm]");
                core_data->pinctrl = NULL;
                return -EINVAL;
        }

        /* stylus active state */
        if (core_data->pdev->id == 0)
        core_data->stylus_clk_active = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_STYLUS_CLK_ACTIVE);
        else
        core_data->stylus_clk_active = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_FOLD_STYLUS_CLK_ACTIVE);

        if (IS_ERR_OR_NULL(core_data->stylus_clk_active)) {
                r = PTR_ERR(core_data->stylus_clk_active);
                ts_err(ts_dev->dev, "Failed to get pinctrl state:%s, r:%d",
                                PINCTRL_STYLUS_CLK_ACTIVE, r);
                core_data->stylus_clk_active = NULL;
                goto exit_pinctrl_put;
        }
        ts_info(ts_dev->dev, "success get stylus avtive pinctrl state");

        /* stylus suspend state */
        if (core_data->pdev->id == 0)
        core_data->stylus_clk_suspend = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_STYLUS_CLK_SUSPEND);
        else
        core_data->stylus_clk_suspend = pinctrl_lookup_state(core_data->pinctrl,
                                PINCTRL_FOLD_STYLUS_CLK_SUSPEND);

        if (IS_ERR_OR_NULL(core_data->stylus_clk_suspend)) {
                r = PTR_ERR(core_data->stylus_clk_suspend);
                ts_err(ts_dev->dev, "Failed to get pinctrl state:%s, r:%d",
                                PINCTRL_STYLUS_CLK_SUSPEND, r);
                core_data->stylus_clk_suspend = NULL;
                goto exit_pinctrl_put;
        }
        ts_info(ts_dev->dev, "success get stylus suspend pinctrl state");

        pinctrl_select_state(core_data->pinctrl, core_data->stylus_clk_suspend);
        ts_info(ts_dev->dev, "set stylus suspend pinctrl state");

        return 0;
exit_pinctrl_put:
        devm_pinctrl_put(core_data->pinctrl);
        core_data->pinctrl = NULL;

        return r;
}

static int ts_touch_info_uevent(const struct device *dev, struct kobj_uevent_env *env)
{
    int ret = 0;
    struct goodix_thp_core *cd = dev_get_drvdata(dev);
    u8 *pen_info = cd->pen_info;
    u8 *ble_mac = cd->ble_mac;

    ret = add_uevent_var(env, "UEVENT_TO=PEN_FRAMEWORK");
    if (ret)
        return ret;

    ret = add_uevent_var(env, "TYPE=TP");
    if (ret)
        return ret;

    ret = add_uevent_var(env, "MESSAGE_TYPE=%02x", cd->uevent_message_type);
    if (ret)
        return ret;

    ret = add_uevent_var(env, "PEN_CLOSE=%02x", cd->pen_close);
    if (ret)
        return ret;

    ret = add_uevent_var(env, "BAT=%02x", cd->battery_level);
    if (ret)
        return ret;

    ret = add_uevent_var(env, "MAC=%02x:%02x:%02x:%02x:%02x:%02x",
                        ble_mac[5], ble_mac[4],
                        ble_mac[3], ble_mac[2],
                        ble_mac[1], ble_mac[0]);
    if (ret)
        return ret;

    ret = add_uevent_var(env, "PEN_INFO=SN:%x%x%x%x%x%x VID:%x%x%x%x PID:%x%x%x%x",
                        ((pen_info[3] & 0x0c) >> 2),
                        (((pen_info[3] & 0x03) << 2) | ((pen_info[2] & 0x30) >> 4)),
                        (pen_info[2] & 0x0f),
                        ((pen_info[1] & 0x3c) >> 2),
                        (((pen_info[1] & 0x03) << 2) | ((pen_info[0] & 0x30) >> 4)),
                        (pen_info[0] & 0x0f), /*SN end*/
                        (((pen_info[6] & 0x03) << 2) | ((pen_info[5] & 0x30) >> 4)),
                        (pen_info[5] & 0x0f),
                        ((pen_info[4] & 0x3c) >> 2),
                        ((pen_info[4] & 0x03) << 2) | ((pen_info[3] & 0x30) >> 4),/*VID end*/
                        ((pen_info[8] & 0x3c) >> 2),
                        ((pen_info[8] & 0x03) << 2) | ((pen_info[7] & 0x30) >> 4),
                        (pen_info[7] & 0x0f),
                        ((pen_info[6] & 0x3c) >> 2)/*PID end*/
                        );
    if (ret)
        return ret;

    ret = add_uevent_var(env, "QPID=%02x", cd->quick_pid);
    if (ret)
        return ret;

    return 0;
}

const struct device_type tp_dev_type ={
    .name = "goodix_ts",
    .uevent = ts_touch_info_uevent,
};

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
static int goodix_init_cpu_boost(struct platform_device *pdev)
{
        struct cpumask cpumask;
        cpumask_var_t valid_cpus;
        int cpu, index = 0, ret = 0;
        struct thp_ts_device *ts_dev;
        struct goodix_thp_core *core_data;
        core_data = platform_get_drvdata(pdev);
        if (!core_data) {
                ts_info(NULL, "Failed to get core data");
                return -ENODEV;
            }
            ts_dev = core_data->ts_dev;

        cpumask_clear(&cpumask);
        cpumask.bits[0] = core_data->ts_dev->board_data.cpu_mask;

        if (!alloc_cpumask_var(&valid_cpus, GFP_KERNEL))
                return -ENOMEM;

        cpumask_and(valid_cpus, &cpumask, cpu_online_mask);
        core_data->qos_count = cpumask_weight(valid_cpus);

        if (core_data->qos_count == 0) {
                ts_info(ts_dev->dev, "No valid CPUs for boosting");
                free_cpumask_var(valid_cpus);
                return -EINVAL;
        }

        core_data->cpu_to_index_map = kcalloc(NR_CPUS, sizeof(int), GFP_KERNEL);
        if (!core_data->cpu_to_index_map) {
                free_cpumask_var(valid_cpus);
                return -ENOMEM;
        }

        for (cpu = 0; cpu < NR_CPUS; cpu++)
                core_data->cpu_to_index_map[cpu] = -1;

        core_data->boost_infos = kcalloc(core_data->qos_count,
                                    sizeof(struct cpu_boost_info), GFP_KERNEL);
        if (!core_data->boost_infos) {
                kfree(core_data->cpu_to_index_map);
                free_cpumask_var(valid_cpus);
                return -ENOMEM;
        }

        for_each_cpu(cpu, valid_cpus) {
                struct cpufreq_policy *policy;
                struct cpu_boost_info *info = &core_data->boost_infos[index];

                policy = cpufreq_cpu_get(cpu);
                if (!policy) {
                        ts_info(ts_dev->dev, "Failed to get policy for CPU%d", cpu);
                        continue;
                }

                info->max_freq = policy->cpuinfo.max_freq;

                if (freq_qos_add_request(&policy->constraints, &info->qos_req,
                                FREQ_QOS_MIN, 0) < 0) {
                        ts_info(ts_dev->dev, "Failed to add QoS for CPU%d", cpu);
                        cpufreq_cpu_put(policy);
                        continue;
                }

                info->initialized = true;

        core_data->cpu_to_index_map[cpu] = index;
        index++;

        cpufreq_cpu_put(policy);

        ts_info(ts_dev->dev, "Initialized CPU%d, index:%d, boost: max_freq=%u kHz",
                cpu, index-1, info->max_freq);
        }

        core_data->qos_count = index;
        free_cpumask_var(valid_cpus);

        ts_info(ts_dev->dev, "Successfully initialized CPU boost for %d CPUs",
                    core_data->qos_count);
        return ret;
}

static void goodix_boost_timer_handler(struct timer_list *t)
{
        struct goodix_thp_core *core_data = from_timer(core_data, t, boost_timer);
        core_data->boost_count = 0; // reset boost_count
        ts_debug(core_data->ts_dev->dev, "Boost counter reset");
}

static void goodix_cleanup_cpu_boost(struct goodix_thp_core *core_data)
{
        int i;

        if (!core_data->boost_infos)
                return;

        for (i = 0; i < core_data->qos_count; i++) {
                struct cpu_boost_info *info = &core_data->boost_infos[i];

                if (info->initialized && freq_qos_request_active(&info->qos_req)) {
                        freq_qos_remove_request(&info->qos_req);
                }
        }

        kfree(core_data->boost_infos);
        core_data->boost_infos = NULL;

        if (core_data->cpu_to_index_map) {
                kfree(core_data->cpu_to_index_map);
                core_data->cpu_to_index_map = NULL;
        }

        core_data->qos_count = 0;
}
#endif

/**
 * goodix_thp_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */
static int goodix_thp_probe(struct platform_device *pdev)
{
        struct goodix_thp_core *core_data = NULL;
        struct thp_ts_device *tdev;
        int r;

        /*init thp core data */
        tdev = pdev->dev.platform_data;
        if (!tdev || !tdev->hw_ops) {
                ts_err(NULL, "Invalid touch device");
                return -ENODEV;
        }
        pdev->dev.type = &tp_dev_type;

        ts_info(tdev->dev, "IN");

        core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_thp_core),
                                 GFP_KERNEL);
        if (!core_data)
                return -ENOMEM;

        core_data->frame_mmap_list.buf = kmalloc(MMAP_BUFFER_SIZE, GFP_KERNEL);
        core_data->pdev = pdev;
        core_data->ts_dev = tdev;
        mutex_init(&core_data->frame_mutex);
        mutex_init(&core_data->irq_mutex);
        mutex_init(&core_data->irq_wake_mutex);
        init_waitqueue_head(&(core_data->frame_wq));
        init_completion(&core_data->pm_completion);
        core_data->pm_suspend = false;
        /* gesture init */
        memset(core_data->gesture_type, 0xff, GESTURE_TYPE_LEN);
        memset(core_data->gesture_data, 0xff, GESTURE_KEY_DATA_LEN);
        memset(core_data->gesture_buffer_data, 0xff, GESTURE_BUFFER_DATA_LEN);
        core_data->sdev = tdev->spi_dev;
        platform_set_drvdata(pdev, core_data);

        core_data->reset_state = 0;
        core_data->get_frame_wait_mode = GET_FRAME_BLOCK_MODE;
        core_data->frame_wait_time = GOODIX_THP_DEFATULT_WAIT_FRAME_TIME;
        //initialize pen action state
        core_data->pen_state = PEN_STATE_NONE;

        /* get GPIO resource*/
        r = goodix_thp_gpio_setup(core_data);
        if (r < 0) {
                ts_err(tdev->dev, "setup gpio failed, r %d", r);
                goto out;
        }

        /* Pinctrl handle is optional. */
        r = goodix_ts_pinctrl_init(core_data);
        if (r)
                ts_err(tdev->dev, "failed init pinctrl");

        /* init stylus clock */
        if (core_data->ts_dev->board_data.stylus_mode_ctrl) {
                r = goodix_ts_stylus_clk_init(core_data);
                if (r)
                        ts_err(tdev->dev, "failed get goodix stylus clock");
        }

        /* power init & power on */
        r = goodix_thp_power_init(core_data);
        if (r < 0) {
                ts_err(tdev->dev, "power init failed, r %d", r);
                goto out;
        }

        r = goodix_thp_power_on(core_data);
        if (r < 0) {
                ts_err(tdev->dev, "power on failed, r %d", r);
                goto out;
        }

        /* board init */
        r = tdev->hw_ops->board_init(tdev);
        if (r) {
                ts_err(tdev->dev, "goodix device chip detect failed, r %d", r);
                goto out;
        }

        /* get custom info */
        r = tdev->hw_ops->get_custom_info(tdev, core_data->custom_info,
                                        GOODIX_THP_CUSTOM_INFO_LEN);
        if (r) {
                ts_err(tdev->dev, "goodix get custom info failed, r %d", r);
                goto out;
        }
        core_data->custom_info[GOODIX_THP_CUSTOM_INFO_LEN] = '\0';

        /* register misc dev */
        if (pdev->id == 0)
                sprintf(core_data->thp_misc_name, "%s", GOODIX_THP_MISC_DEVICE_NAME);
        else
                sprintf(core_data->thp_misc_name, "%s%d", GOODIX_THP_MISC_DEVICE_NAME, pdev->id);        
        core_data->thp_misc_dev.minor = MISC_DYNAMIC_MINOR;
        core_data->thp_misc_dev.name = core_data->thp_misc_name;
        core_data->thp_misc_dev.fops = &g_thp_fops;
        r = misc_register(&core_data->thp_misc_dev);
        if (r) {
                ts_err(tdev->dev, "failed to register misc device '/dev/thp', r %d", r);
                goto out;
        }

        r = goodix_thp_pen_input_dev_init(core_data);
        if (r) {
                ts_err(tdev->dev, "failed to init suspend input dev, r %d", r);
                goto err_init_pen_dev;
        }

        /* init input_agent */
        r = goodix_thp_input_agent_init(core_data);
        if (r) {
                ts_err(tdev->dev, "failed to init gdix_input_agent, r %d", r);
                goto err_init_wrapper;
        }

        /* init sysfs */
        r = goodix_thp_sysfs_init(core_data);
        if (r) {
                ts_err(tdev->dev, "failed to create sysfs, r %d", r);
                goto err_sysfs_init;
        }

        /* irq wake lock */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
        core_data->ws = wakeup_source_register(dev_name(tdev->dev));
#else
        core_data->ws = wakeup_source_register(tdev->dev, dev_name(tdev->dev));
#endif
        if (!core_data->ws) {
                ts_err(tdev->dev, "failed to allocate goodix thp wakeup source");
                r = -EINVAL;
                goto err_wakeup_source_register_failed;
        }

        /* PM QoS */
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

        if (!core_data->pm_qos_state) {
                core_data->pm_qos_value = PM_QOS_DEFAULT_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
                if (!cpu_latency_qos_request_active(&core_data->pm_qos_req)) {
                        cpu_latency_qos_add_request(&core_data->pm_qos_req, core_data->pm_qos_value);
                } else {
                        cpu_latency_qos_update_request(&core_data->pm_qos_req, core_data->pm_qos_value);
                }
#else
                pm_qos_add_request(&core_data->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, core_data->pm_qos_value);
#endif
                ts_info(tdev->dev, "add qos request in touch driver.");
                core_data->pm_qos_state = 1;
        }

#endif

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        if (goodix_init_cpu_boost(pdev)) {
                ts_err(tdev->dev, "CPU boost initialization failed");
                core_data->qos_count = 0;
        }

        //init timer for clear boost_count
        core_data->boost_count = 0;
        timer_setup(&core_data->boost_timer, goodix_boost_timer_handler, 0);
         ts_info(tdev->dev, "Touch boost config: affinity=0x%lx, boost-count=%d, timeout=%dms",
                  tdev->board_data.cpu_mask,
                  tdev->board_data.max_boost_count,
                  tdev->board_data.boost_timeout);
#endif

        if (core_data->ts_dev->board_data.stylus_interpolation_ctrl) {
                core_data->rate_configs = parse_stylus_report_rate_config(tdev->dev, &core_data->config_count);
                if (!core_data->rate_configs) {
                        ts_err(tdev->dev, "Failed to parse stylus rate config");
                        goto err_irq_setup;
                }
        }

        /* request irq */
        r = goodix_thp_irq_setup(core_data);
        if (r) {
                ts_err(tdev->dev, "goodix setup irq failed, r %d", r);
                goto err_irq_setup;
        }
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
        core_data->pm_notif.notifier_call = goodix_thp_drm_notifier_callback;
	if (mtk_disp_notifier_register("Touch", &core_data->pm_notif))
		ts_err(tdev->dev, "Failed to register disp notifier client:%d", ret);
#elif IS_ENABLED(CONFIG_FB)
        core_data->pm_notif.notifier_call = goodix_thp_fb_notifier_callback;
        r = fb_register_client(&core_data->pm_notif);
        if (r < 0)
                ts_err(tdev->dev, "[FB]Unable to register fb_notifier, ret:%d", r);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
        ts_info(tdev->dev, "goodix_ts_mmi_dev_register");
        r = goodix_ts_mmi_dev_register(pdev);
        if (r) {
            ts_info(tdev->dev, "Failed register touchscreen mmi.");
            goto err_irq_setup;
        }
#endif

        goodix_thp_esd_init(core_data);

        return 0;

err_irq_setup:
        goodix_thp_sysfs_exit(core_data);
err_wakeup_source_register_failed:
err_sysfs_init:
        goodix_thp_input_agent_exit(core_data);
err_init_wrapper:
        goodix_thp_pen_input_dev_exit(core_data);
err_init_pen_dev:
        misc_deregister(&core_data->thp_misc_dev);
out:
        if (r) {
                platform_set_drvdata(pdev, NULL);
                ts_info(tdev->dev, "Cleared pdev drvdata due to probe failure");
                kfree_safe(core_data->frame_mmap_list.buf);
                goodix_thp_power_off(core_data);
        }
        ts_info(tdev->dev, "goodix_thp_probe OUT, r:%d", r);
        return r;
}

static void goodix_thp_remove(struct platform_device *pdev)
{
        struct goodix_thp_core *core_data = platform_get_drvdata(pdev);
        struct thp_ts_device *tdev = core_data->ts_dev;

        ts_info(tdev->dev, "IN");
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
        ts_info(tdev->dev, "goodix_ts_mmi_dev_unregister");
        goodix_ts_mmi_dev_unregister(pdev);
#endif
        goodix_thp_power_off(core_data);
        goodix_thp_sysfs_exit(core_data);
        goodix_thp_input_agent_exit(core_data);
        goodix_thp_pen_input_dev_exit(core_data);
        misc_deregister(&core_data->thp_misc_dev);
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
        if (mtk_disp_notifier_unregister(&core_data->pm_notif))
                ts_info(tdev->dev, "Error occurred when unregister disp_notifier");
#elif IS_ENABLED(CONFIG_FB)
        fb_unregister_client(&core_data->pm_notif);
#endif
        kfree_safe(core_data->frame_mmap_list.buf);
        if (core_data->ts_dev->board_data.stylus_interpolation_ctrl && core_data->rate_configs) {
                free_stylus_report_rate_config(core_data->rate_configs);
        }

        /*free wakeup source*/
        if (core_data->ws) {
            wakeup_source_unregister(core_data->ws);
        }

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        goodix_cleanup_cpu_boost(core_data);
#endif

        return;
}

static const struct platform_device_id ts_core_ids[] = {
        {.name = GOODIX_CORE_DRIVER_NAME},
        {}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver thp_core_driver = {
        .driver = {
                .name = GOODIX_CORE_DRIVER_NAME,
                .owner = THIS_MODULE,
        },
        .probe = goodix_thp_probe,
        .remove = goodix_thp_remove,
        .id_table = ts_core_ids,
};

int goodix_thp_core_init(void)
{
        ts_info(NULL, "goodix thp driver v%s", GOODIX_THP_DRIVER_VERSION);

        return platform_driver_register(&thp_core_driver);
}

int goodix_thp_core_deinit(void)
{
        ts_info(NULL, "IN");

        platform_driver_unregister(&thp_core_driver);
        return 0;
}
