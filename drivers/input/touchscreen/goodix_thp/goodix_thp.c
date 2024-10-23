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

#define GOODIX_THP_MISC_DEVICE_NAME	"thp"
#define PINCTRL_STATE_ACTIVE		"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND		"pmx_ts_suspend"
#define DEVICE_NAME			"input_agent"
#define GOOIDX_INPUT_PHYS			"goodix_ts/input0"

#define QUERYBIT(longlong, bit) 	(!!(longlong[bit/8] & (1 << bit%8)))

struct goodix_thp_core *gdix_thp_core;
static struct thp_input_agent_data *g_thp_input_agent = NULL;

static int goodix_thp_suspend(struct goodix_thp_core *core_data);
static int goodix_thp_resume(struct goodix_thp_core *core_data);

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

static void goodix_thp_set_fp_int_pin(struct thp_ts_device *tdev, u8 level)
{
	static bool is_high = false;

	if (!is_high && level) {
		is_high = true;
		tdev->hw_ops->set_fp_int_pin(tdev, 1);
	} else if (is_high && !level) {
		is_high = false;
		tdev->hw_ops->set_fp_int_pin(tdev, 0);
	}
}

/*
 * If irq is disabled/enabled, can not disable/enable again
 * disable - status 0; enable - status not 0
 */
static void goodix_thp_set_irq_enable(struct goodix_thp_core *core_data,
					int status)
{
	mutex_lock(&core_data->irq_mutex);
	if (core_data->irq_state != !!status) {
		status ? enable_irq(core_data->irq) : disable_irq(core_data->irq);
		core_data->irq_state = !!status;
		ts_info("%s: %s irq", __func__,
				status ? "enable" : "disable");
	}
	mutex_unlock(&core_data->irq_mutex);
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
	static uint32_t id;

	mutex_lock(&core_data->frame_mutex);
	/* check for max limit */
	if ((list->tail + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT == list->head) {
		ts_err("frame mmap buffer is overlay");
		list->head = (list->head + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT;
	}

	req_pkg = (struct driver_request_pkg *)&list->buf[list->tail * GOODIX_THP_MAX_FRAME_LEN];
	req_pkg->size = sizeof(req_pkg->request) + len;
	req_pkg->request.id = id++;
	req_pkg->request.type = type;
	if (len > 0)
		memcpy(req_pkg->request.data, data, len);
	list->tail = (list->tail + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT;
	core_data->frame_waitq_state = WAKEUP_STATE;
	wake_up_interruptible(&(core_data->frame_wq));
	mutex_unlock(&(core_data->frame_mutex));
}

static int goodix_thp_open(struct inode *inode, struct file *filp)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("%s: called", __func__);

	/* check thp dev status */
	mutex_lock(&core_data->ts_mutex);
	core_data->open_num++;
	if (core_data->open_num > 1) {
		ts_err("%s: dev have be opened", __func__);
		mutex_unlock(&core_data->ts_mutex);
		return 0;
	}
	mutex_unlock(&core_data->ts_mutex);
	/* reset thp dev status */
	core_data->reset_state = 0;//current isn't in reset status
	core_data->get_frame_wait_mode = GET_FRAME_BLOCK_MODE;
	core_data->frame_len = GOODIX_THP_MAX_FRAME_LEN - GOODIX_THP_REQUEST_APP_SIZE;
	core_data->frame_wait_time = GOODIX_THP_DEFATULT_WAIT_FRAME_TIME;
	return 0;
}

static int goodix_thp_release(struct inode *inode, struct file *filp)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("%s: called", __func__);

	/* check thp dev status */
	mutex_lock(&core_data->ts_mutex);
	if (core_data->open_num > 0)
		core_data->open_num--;
	mutex_unlock(&core_data->ts_mutex);
	goodix_thp_frame_wake_up(core_data);
	return 0;
}

static long goodix_thp_ioctl_get_frame(unsigned long arg)
{
	void __user *user_val = (void *)arg;
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_frame_mmap_list *list = &core_data->frame_mmap_list;
	struct thp_ioctl_frame hal_frame;
	long r = 0;

	/* copy data from hal */
	if (copy_from_user(&hal_frame, user_val,
			sizeof(struct thp_ioctl_frame))) {
		ts_err("Failed to copy_from_user .");
		return -EFAULT;
	}

	core_data->frame_len = hal_frame.size;

	mutex_lock(&core_data->frame_mutex);
	if (list->head == list->tail) {
		if (core_data->get_frame_wait_mode == GET_FRAME_NONBLOCK_MODE) {
			ts_err("no frame");
			r = -ENODATA;
			goto out;
		} else {
			core_data->frame_waitq_state = WAIT_STATE;
			if (core_data->frame_wait_time == 0) {
				mutex_unlock(&core_data->frame_mutex);
				wait_event_interruptible(core_data->frame_wq,
					(core_data->frame_waitq_state == WAKEUP_STATE));
				mutex_lock(&core_data->frame_mutex);
			} else {
				mutex_unlock(&core_data->frame_mutex);
				r = wait_event_interruptible_timeout(core_data->frame_wq,
					(core_data->frame_waitq_state == WAKEUP_STATE),
					msecs_to_jiffies(core_data->frame_wait_time));
				mutex_lock(&core_data->frame_mutex);
				if (r == 0)
					r = -ETIMEDOUT;
			}
		}
	}

	if (list->head != list->tail) {
		hal_frame.pos = list->head * GOODIX_THP_MAX_FRAME_LEN;
		hal_frame.tv_us = ktime_get_real_ns() / 1000;
		if(copy_to_user(user_val, &hal_frame, sizeof(hal_frame))) {
			ts_err("Failed to copy_to_user().");
			r = -EFAULT;
			goto out;
		}
		r = 0;
	} else {
		if (r == -ETIMEDOUT) {
			ts_err("get frame timeout, timeout value[%d]",
				core_data->frame_wait_time);
		} else {
			ts_err("no frame");
			r = -ENODATA;
		}
	}

out:
	mutex_unlock(&core_data->frame_mutex);
	return r;
}

static long goodix_thp_ioctl_set_reset_value(unsigned long reset)
{
	struct goodix_thp_core *ts = gdix_thp_core;

	ts_info("%s:set reset status %ld", __func__, reset);

	gpio_set_value(ts->ts_dev->board_data.reset_gpio, !!reset);

	ts->frame_waitq_state = WAIT_STATE;
	ts->reset_state = !reset;

	return 0;
}

static long goodix_thp_ioctl_set_wait_time(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	unsigned int wait_frame_time = arg;

	if (arg > GOODIX_THP_MAX_TIMEOUT)
		wait_frame_time = GOODIX_THP_MAX_TIMEOUT;

	ts_info("set wait time %d ms.(current %dms)\n",
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

static long goodix_thp_ioctl_spi_trans(void __user *data)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	int r = 0;
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	struct thp_ioctl_spi_trans_data trans_data;

	if (cd->suspended && !cd->gesture_enable)
		return 0;

	/* copy data from hal */
	if (copy_from_user(&trans_data, data,
			sizeof(struct thp_ioctl_spi_trans_data))) {
		ts_err("Failed to copy_from_user().");
		return -EFAULT;
	}

	/* check sync data size */
	if (trans_data.size > GOODIX_THP_MAX_TRANS_DATA_LEN) {
		ts_err("trans_data.size out of range.");
		return -EINVAL;
	}

	/* alloc memory for rx/tx buf */
	rx_buf = kzalloc(trans_data.size, GFP_KERNEL);
	tx_buf = kzalloc(trans_data.size, GFP_KERNEL);
	if (!rx_buf || !tx_buf) {
		ts_err("%s:buf request memory fail,trans_data.size = %d",
			__func__,trans_data.size);
		goto exit;
	}

	/* copy hal tx buf to driver tx buf */
	r = copy_from_user(tx_buf, trans_data.tx, trans_data.size);
	if (r) {
		ts_err("%s:copy in buff fail", __func__);
		goto exit;
	}

	/* spi transfer */
	r =  goodix_thp_spi_trans(cd, tx_buf, rx_buf, trans_data.size);
	if (r) {
		ts_err("%s: transfer error, ret = %d", __func__, r);
		goto exit;
	}

	/* copy driver rx to hal */
	if (trans_data.rx) {
		r = copy_to_user(trans_data.rx, rx_buf, trans_data.size);
		if (r) {
			ts_err("%s:copy out buff fail", __func__);
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

static long goodix_thp_ioctl_notify_update(void __user *data)
{
	struct thp_ioctl_update_info update_info;

	if (copy_from_user((u8 *)&update_info, data,
			sizeof(update_info))) {
		ts_err("Failed to copy_from_user().");
		return -EFAULT;
	}

	goodix_frame_reg = update_info.frame_addr;
	goodix_cmd_reg = update_info.cmd_addr;
	ts_info("set frame addr:0x%04X cmd addr:0x%04X",
		goodix_frame_reg, goodix_cmd_reg);
	return 0;
}

static long goodix_thp_ioctl_set_wait_mode(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	unsigned int wait_frame_mode = arg;

	mutex_lock(&(ts->frame_mutex));
	if (wait_frame_mode)
		ts->get_frame_wait_mode = GET_FRAME_BLOCK_MODE;
	else
		ts->get_frame_wait_mode = GET_FRAME_NONBLOCK_MODE;
	ts->frame_waitq_state = WAKEUP_STATE;
	wake_up_interruptible(&(ts->frame_wq));
	mutex_unlock(&(ts->frame_mutex));
	ts_info("set block %d", wait_frame_mode);
	return 0;
}

static long goodix_thp_ioctl_irq_enable(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	unsigned int irq_flag = (unsigned int)arg;
	goodix_thp_set_irq_enable(ts, irq_flag);
	return 0;
}

static long goodix_thp_ioctl_get_frame_buf_num(unsigned long arg)
{
	return 0;
}

static long goodix_thp_ioctl_reset_frame_list(void)
{
	struct goodix_thp_core *ts = gdix_thp_core;

	ts_info("%s called", __func__);
	goodix_thp_reset_frame_list(ts);
	return 0;
}

static long goodix_thp_ioctl_get_driver_state(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	u32 __user *driver_state = (u32 *)arg;

	ts_info("%s:driver state = %d", __func__, ts->suspended);

	if (driver_state == NULL) {
		ts_err("%s: input parameter null", __func__);
		return -EINVAL;
	}

	if(copy_to_user(driver_state, &ts->suspended, sizeof(u32))) {
		ts_err("%s:copy driver_state failed", __func__);
		return -EFAULT;
	}

	return 0;
}

static long goodix_thp_ioctl_get_state_change_flag(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	u32 __user *change_flag = (u32 *)arg;

	//ts_info("%s:state_change_flag = %d", __func__, ts->state_change_flag);

	if (change_flag == NULL) {
		ts_err("%s: input parameter null", __func__);
		return -EINVAL;
	}

	if(copy_to_user(change_flag, &ts->state_change_flag, sizeof(u32))) {
		ts_err("%s:copy state_change_flag failed", __func__);
		return -EFAULT;
	}

	return 0;
}

static long goodix_thp_ioctl_set_state_change_flag(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	unsigned int change_flag = arg;

	//ts_info("set state_change_flag = %d", change_flag);

	ts->state_change_flag = change_flag;

	return 0;
}

static int goodix_thp_ioctl_set_spi_speed(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	struct thp_ts_device *dev = ts->ts_dev;
	unsigned int speed = arg;

	//ts_info("set spi_speed = %d", speed);

	if (goodix_thp_set_spi_speed(dev, speed))
		return -EINVAL;

	return 0;
}

static long goodix_thp_ioctl_multi_spi_trans(void __user *data)
{
	struct goodix_thp_core *ts = gdix_thp_core;
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
		ts_info("failed alloc memory for xfer_data");
		goto exit;
	}

	xfer =  kzalloc(multi_data.xfer_num * sizeof(*xfer), GFP_KERNEL);
	if (!xfer) {
		ts_info("failed alloc memory for xfer");
		goto exit;
	}

	if (copy_from_user(xfer_data, multi_data.xfer_data,
		sizeof(struct thp_ioctl_spi_xfer_data) * multi_data.xfer_num)) {
		ts_info("failed copy from user for xfer_data");
		goto exit;
	}

	rx_buf = kzalloc(5120, GFP_KERNEL);
	if (!rx_buf) {
		ts_info("failed alloc buffer for rx_buf");
		goto exit;
	}
	tx_buf = kzalloc(5120, GFP_KERNEL);
	if (!tx_buf) {
		ts_info("failed alloc buffer for tx_buf");
		goto exit;
	}

	spi_message_init(&msg);
	for(i = 0; i < multi_data.xfer_num; i++) {
		if(xfer_data[i].tx){
			r = copy_from_user(tx_buf + tmp_len,
				xfer_data[i].tx, xfer_data[i].len);
			if (r) {
				ts_info("failed copy from user:%d", r);
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
		goodix_thp_set_spi_speed(dev, GOODIX_SPI_SPEED_WAKEUP);
		spi_sync(spi, &msg);
		goodix_thp_set_spi_speed(dev, spi_speed_backup);
		mutex_unlock(&dev->spi_mutex);
	} else {
		mutex_lock(&dev->spi_mutex);
		r = spi_sync(spi, &msg);
		mutex_unlock(&dev->spi_mutex);
		if(r) {
			ts_info("failed do spi sync:%d", r);
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

static long goodix_thp_ioctl_enter_suspend(unsigned long arg)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	int r = 0;

	cd->gesture_enable = arg;
	ts_info("%s: called", __func__);

	r = goodix_thp_suspend(cd);
	if (r)
		ts_err("%s failed, r %d", __func__, r);

	return r;
}

static long goodix_thp_ioctl_enter_resume(void)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	int r = 0;

	ts_info("%s: called", __func__);

	r = goodix_thp_resume(cd);
	if (r)
		ts_err("%s failed, r %d", __func__, r);

	return r;
}

static long goodix_thp_ioctl_recv_tsc_msg(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct thp_ioctl_tsc_msg tsc_msg;
	u8 ble_mac[6];
	u8 stylus_id[2];

	if (copy_from_user(&tsc_msg, argp,
			sizeof(struct thp_ioctl_tsc_msg))) {
		ts_err("Failed to copy_from_user .");
		return -EFAULT;
	}

	switch (tsc_msg.cmd) {
	case SVC_CMD_MMAP_DEQUEUE:
		mutex_lock(&gdix_thp_core->frame_mutex);
		if (gdix_thp_core->frame_mmap_list.head != gdix_thp_core->frame_mmap_list.tail) {
			gdix_thp_core->frame_mmap_list.head =
				(gdix_thp_core->frame_mmap_list.head + 1) % GOODIX_THP_MAX_FRAME_BUF_COUNT;
		}
		mutex_unlock(&gdix_thp_core->frame_mutex);
		break;
	case SVC_CMD_BLE_MAC:
		memcpy(ble_mac, &tsc_msg.value[0], sizeof(ble_mac));
		memcpy(stylus_id, &tsc_msg.value[6], sizeof(stylus_id));
		ts_info("recv ble mac:%*ph, stylusID:%*ph", 6, ble_mac, 2, stylus_id);
		break;
	case SVC_CMD_GAME_FILTER:
		ts_info("recv game filter:%*ph", tsc_msg.len, tsc_msg.value);
		break;
	case SVC_CMD_UPDATE_VERSION:
		memcpy(gdix_thp_core->ts_dev->board_data.thp_ver, tsc_msg.value, tsc_msg.len);
		ts_info("thp_ver:%s", tsc_msg.value);
		break;
	default:
		ts_err("not support svc msg:0x%02x", tsc_msg.cmd);
		break;
	}


	return 0;
}

static long goodix_thp_ioctl_get_chip_type(unsigned long arg)
{
	struct goodix_thp_core *ts = gdix_thp_core;
	u32 __user *user_val = (u32 *)arg;

	if (user_val == NULL) {
		ts_err("input parameter null");
		return -EINVAL;
	}

	if(copy_to_user(user_val, &ts->ts_dev->board_data.chip_type, sizeof(u32))) {
		ts_err("copy driver_state failed");
		return -EFAULT;
	}

	return 0;
}

/* enable or disable tsd debug socket */
static int goodix_thp_ioctl_set_tsd_state(int tsd_enable)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[2];

	/* resume: touch power on is after display to avoid display disturb */
	ts_info("%s IN, set tsd state %d", __func__, tsd_enable);

	val[0] = NOTIFY_TYPE_TSD_CTRL;
	val[1] = tsd_enable;
	put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	return 0;
}

/* enable or disable tsd debug socket */
static int goodix_thp_ioctl_set_stylus_state(int stylus_enable)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[2];

	/* resume: touch power on is after display to avoid display disturb */
	ts_info("%s IN, set stylus state %d", __func__, stylus_enable);

	val[0] = NOTIFY_TYPE_STYLUS_CTRL;
	val[1] = stylus_enable;
	put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
	return 0;
}

static long goodix_thp_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long ret;

	switch (cmd) {
	case IOCTL_CMD_GET_FRAME:
		ret = goodix_thp_ioctl_get_frame(arg);
		break;
	case IOCTL_CMD_SET_RESET_VALUE:
		ret = goodix_thp_ioctl_set_reset_value(arg);
		break;
	case IOCTL_CMD_SET_WAIT_TIME:
		ret = goodix_thp_ioctl_set_wait_time(arg);
		break;
	case IOCTL_CMD_SPI_TRANS:
		ret = goodix_thp_ioctl_spi_trans((void __user *)arg);
		break;
	case IOCTL_CMD_NOTIFY_UPDATE:
		ret = goodix_thp_ioctl_notify_update((void __user *)arg);
		break;
	case IOCTL_CMD_SET_WAIT_MODE:
		ret = goodix_thp_ioctl_set_wait_mode(arg);
		break;
	case IOCTL_CMD_IRQ_ENABLE:
		ret = goodix_thp_ioctl_irq_enable(arg);
		break;
	case IOCTL_CMD_GET_FRAME_BUF_NUM:
		ret = goodix_thp_ioctl_get_frame_buf_num(arg);
		break;
	case IOCTL_CMD_RESET_FRAME_LIST:
		ret = goodix_thp_ioctl_reset_frame_list();
		break;
	case IOCTL_CMD_GET_DRIVER_STATE:
		ret = goodix_thp_ioctl_get_driver_state(arg);
		break;
	case IOCTL_CMD_GET_STATE_CHANGE_FLAG:
		ret = goodix_thp_ioctl_get_state_change_flag(arg);
		break;
	case IOCTL_CMD_SET_STATE_CHANGE_FLAG:
		ret = goodix_thp_ioctl_set_state_change_flag(arg);
		break;
	case IOCTL_CMD_SET_SPI_SPEED:
		ret = goodix_thp_ioctl_set_spi_speed(arg);
		break;
	case IOCTL_CMD_MUILT_SPI_TRANS:
		ret = goodix_thp_ioctl_multi_spi_trans((void __user *)arg);
		break;
	case IOCTL_CMD_ENTER_SUSPEND:
		ret = goodix_thp_ioctl_enter_suspend(arg);
		break;
	case IOCTL_CMD_ENTER_RESUME:
		ret = goodix_thp_ioctl_enter_resume();
		break;
	case IOCTL_CMD_RECV_TSC_MSG:
		ret = goodix_thp_ioctl_recv_tsc_msg(arg);
		break;
	case IOCTL_CMD_GET_CHIP_TYPE:
		ret = goodix_thp_ioctl_get_chip_type(arg);
		break;
	case IOCTL_CMD_SET_TOOL_OPS:
		ret = goodix_thp_ioctl_set_tsd_state(arg);
		break;
	default:
		ts_err("cmd unknown.");
		ret = 0;
	}

	return ret;
}

static int goodix_thp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	void *sh_mem = (void *)cd->frame_mmap_list.buf;
	size_t size = vma->vm_end - vma->vm_start;
	struct page *page = NULL;

	if (size > MMAP_BUFFER_SIZE) {
		ts_err("vm_size[%d] > mmap_size[%d]",
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

static struct miscdevice g_thp_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = GOODIX_THP_MISC_DEVICE_NAME,
	.fops = &g_thp_fops,
};

/**
 * goodix_thp_power_init- Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_thp_power_init(struct goodix_thp_core *core_data)
{
	struct goodix_thp_board_data *ts_bdata;
	struct device *dev = NULL;
	int r = 0;

	ts_info("Power init");
	/* dev:i2c client device or spi slave device*/
	dev =  core_data->ts_dev->dev;
	ts_bdata = board_data(core_data);

	if (strlen(ts_bdata->avdd_name)) {
		core_data->avdd = devm_regulator_get(dev,
				 ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			r = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", r);
			core_data->avdd = NULL;
			return r;
		}

		r = regulator_set_load(core_data->avdd, 50000);
		if (r) {
			ts_err("set avdd load fail");
			return r;
		}
		r = regulator_set_voltage(core_data->avdd, 3000000, 3000000);
		if (r) {
			ts_err("set avdd voltage fail");
			return r;
		}
	} else {
		ts_info("Avdd name is NULL[skip]");
	}

	if (strlen(ts_bdata->iovdd_name)) {
		core_data->iovdd = devm_regulator_get(dev,
				ts_bdata->iovdd_name);
		if (IS_ERR_OR_NULL(core_data->iovdd)) {
			r = PTR_ERR(core_data->iovdd);
			ts_err("Failed to get regulator iovdd:%d", r);
			core_data->iovdd = NULL;
			return r;
		}
	} else {
		ts_info("iovdd name is NULL[skip]");
	}

	return r;
}

int goodix_thp_reset_after(struct goodix_thp_core *cd);

/**
 * goodix_thp_power_on- Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_thp_power_on(struct goodix_thp_core *cd)
{
	struct goodix_thp_board_data *ts_bdata = board_data(cd);
	int ret;

	int iovdd_gpio = ts_bdata->iovdd_gpio;
	int reset_gpio = ts_bdata->reset_gpio;

	if (iovdd_gpio > 0) {
		gpio_direction_output(iovdd_gpio, 1);
	} else if (cd->iovdd) {
		ret = regulator_enable(cd->iovdd);
		if (ret < 0) {
			ts_err("Failed to enable iovdd:%d", ret);
			goto power_off;
		}
	}
	usleep_range(3000, 3100);
	if (cd->avdd) {
		ret = regulator_enable(cd->avdd);
		if (ret < 0) {
			ts_err("Failed to enable avdd:%d", ret);
			goto power_off;
		}
	}
	usleep_range(15000, 15100);
	gpio_direction_output(reset_gpio, 1);
	usleep_range(4000, 4100);

	cd->power_on = 1;
	return 0;

power_off:
	gpio_direction_output(reset_gpio, 0);
	if (iovdd_gpio > 0)
		gpio_direction_output(iovdd_gpio, 0);
	else if (cd->iovdd)
		regulator_disable(cd->iovdd);
	else if (cd->avdd)
		regulator_disable(cd->avdd);
	return ret;
}

static void goodix_thp_power_off(struct goodix_thp_core *core_data)
{
	struct goodix_thp_board_data *ts_bdata = board_data(core_data);

	ts_info("Device power off");
	if (core_data->power_on == 0) {
		ts_info("device has already power off");
		return;
	}

	gpio_direction_output(ts_bdata->reset_gpio, 0);
	if (core_data->iovdd)
		regulator_disable(core_data->iovdd);
	if (core_data->avdd)
		regulator_disable(core_data->avdd);
	core_data->power_on = 0;
}

static int goodix_thp_gpio_setup(struct goodix_thp_core *core_data)
{
	struct goodix_thp_board_data *ts_bdata = board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);

	/**
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
				  GPIOF_OUT_INIT_HIGH, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
				  GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}

	if (ts_bdata->iovdd_gpio > 0) {
		r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->iovdd_gpio,
				GPIOF_OUT_INIT_LOW, "ts_iovdd_gpio");
		if (r < 0) {
			ts_err("Failed to request iovdd-gpio, r:%d", r);
			return r;
		}
	}

	return 0;
}

static int goodix_thp_gesture_irq_handler(struct goodix_thp_core *core_data)
{
	int r = 0;
	int i;
	u8 ges_num = 0;
	u8 clean_data = 0;
	u8 temp_data[GESTURE_KEY_DATA_LEN] = {0};
	u32 ges_addr = 0x10308;
	u16 gsx_data = ~core_data->gesture_enable;
	int coor_x, coor_y;
	struct thp_ts_device *ts_dev =  core_data->ts_dev;
	struct gesture_event_data mmi_event;
	static  unsigned  long  start = 0;
	int fod_down_interval = 0;
	int fod_down = core_data->zerotap_data[0];

	if (core_data->suspend_dev == NULL) {
		ts_err("invalid input_dev!");
		r = -EINVAL;
		goto exit;
	}

	if (ts_dev->board_data.chip_type == CHIP_TYPE_9897)
		ges_addr = 0x101A0;
	else if (ts_dev->board_data.chip_type == CHIP_TYPE_9916 ||
			ts_dev->board_data.chip_type == CHIP_TYPE_9615)
		ges_addr = 0x10308;
	else if (ts_dev->board_data.chip_type == CHIP_TYPE_9966)
		ges_addr = 0x10274;
	else {
		ts_err("not support chip type %d", ts_dev->board_data.chip_type);
		return -EINVAL;
	}

	/* get gesture data */
	r = ts_dev->hw_ops->read(ts_dev, ges_addr, temp_data, sizeof(temp_data));
	if (r < 0 || ((temp_data[0] & GESTURE_DATA_TYPE) == 0)) {
		ts_err("Read gesture data failed, r=%d, data[0]=0x%x",
				r, temp_data[0]);
		goto re_send_ges_cmd;
	}

	/* check gesture data */
	if (checksum16_cmp(temp_data, GESTURE_DATA_HEAD_LEN, GOODIX_LE_MODE)) {
		ts_err("gesture data head check failed");
		ts_err("%*ph", GESTURE_DATA_HEAD_LEN, temp_data);
		goto re_send_ges_cmd;
	} else if (checksum16_cmp(&temp_data[GESTURE_DATA_HEAD_LEN],
		GESTURE_KEY_DATA_LEN - GESTURE_DATA_HEAD_LEN, GOODIX_LE_MODE)) {
		ts_err("Gesture data checksum error!");
		ts_info("Gesture data %*ph", (int)sizeof(temp_data), temp_data);
		goto re_send_ges_cmd;
	}

	/* save gesture data */
	memcpy(gdix_thp_core->gesture_data, temp_data, sizeof(temp_data));

	switch (temp_data[4]) {
	case 0xCC: //double tap
		ts_info("get gesture event: Double tap");
		mmi_event.evcode =4;
		core_data->imports->report_gesture(&mmi_event);
		break;
	case 0x63: // C
		ts_info("get gesture event: C");
		ges_num = 6;
		break;
	case 0x65: // E
		ts_info("get gesture event: E");
		ges_num = 6;
	    break;
	case 0x6D: // M
		ts_info("get gesture event: M");
		break;
	case 0x77: // W
		ts_info("get gesture event: W");
		ges_num = 5;
		break;
	case 0x40: // A
		ts_info("get gesture event: A");
		ges_num = 6;
		break;
	case 0x66: // F
		ts_info("get gesture event: F");
		ges_num = 6;
		break;
	case 0x6F: // O
		ts_info("get gesture event: O");
		ges_num = 6;
		break;
	case 0xAA: // R2L
		ts_info("get gesture event: right to left");
		break;
	case 0xBB: // L2R
		ts_info("get gesture event: left to right");
		break;
	case 0xBA: // UP
		ts_info("get gesture event: up");
		ges_num = 2;
		break;
	case 0xAB: // DOWN
		ts_info("get gesture event: down");
		ges_num = 2;
		break;
	case 0x46: // FP_DOWN
		fod_down_interval = (int)jiffies_to_msecs(jiffies-start);
		//goodix firmware do not send coordinate, need mmi touch to define a vaild coordinate thru dts
		mmi_event.evcode = 2;
		mmi_event.evdata.x= 0;
		mmi_event.evdata.y= 0;

		ts_info("Get FOD-DOWN gesture:%d interval:%d",fod_down,fod_down_interval);
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
		ts_info("Get FOD-UP gesture");
		mmi_event.evcode = 3;
		mmi_event.evdata.x= 0;
		mmi_event.evdata.y= 0;
		core_data->imports->report_gesture(&mmi_event);
		fod_down = 0;
        break;
	case 0x4C: // single tap
		ts_info("get gesture event: single tap");
		mmi_event.evcode =1;
		core_data->imports->report_gesture(&mmi_event);
		break;
	default:
		ts_err("not support gesture type %x", temp_data[4]);
		break;
	}

	for (i = 0; i < ges_num; i++) {
		coor_x = le16_to_cpup((__le16 *)&temp_data[8 + i * 4]);
		coor_y = le16_to_cpup((__le16 *)&temp_data[10 + i * 4]);
		ts_info("ges_coor_x:%d ges_coor_y:%d", coor_x, coor_y);
		input_mt_slot(g_thp_input_agent->input_dev, 0);
		input_mt_report_slot_state(g_thp_input_agent->input_dev, 0, 1);
		input_report_abs(g_thp_input_agent->input_dev, ABS_MT_POSITION_X, coor_x);
		input_report_abs(g_thp_input_agent->input_dev, ABS_MT_POSITION_Y, coor_y);
		input_report_key(g_thp_input_agent->input_dev, BTN_TOUCH, 1);
		input_sync(g_thp_input_agent->input_dev);
	}

	if (ges_num > 0) {
		input_mt_slot(g_thp_input_agent->input_dev, 0);
		input_mt_report_slot_state(g_thp_input_agent->input_dev, 0, 0);
		input_report_key(g_thp_input_agent->input_dev, BTN_TOUCH, 0);
		input_sync(g_thp_input_agent->input_dev);
	}

	goto exit;

re_send_ges_cmd:
	/* resend gesture cmd */
	if(ts_dev->hw_ops->send_cmd(ts_dev, CMD_GESTURE, gsx_data))
		ts_info("warning: failed re_send gesture cmd");
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

	if (core_data->reset_state) {
		ts_err("%s: ignore this irq.", __func__);
		return IRQ_HANDLED;
	}

	/* suspend irq handler */
	if (core_data->suspended && core_data->gesture_enable) {
		goodix_thp_gesture_irq_handler(core_data);
		return IRQ_HANDLED;
	}

	disable_irq_nosync(core_data->irq);

	/* get frame */
	r = ts_dev->hw_ops->get_frame(ts_dev, read_data, core_data->frame_len);
	if (r) {
		ts_err("failed to read frame, r %d", r);
		goto exit;
	}

	/* copy frame to frame list */
	put_frame_list(core_data, REQUEST_TYPE_FRAME, read_data, core_data->frame_len);
exit:
	enable_irq(core_data->irq);
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
	int r;

	/* if ts_bdata-> irq is invalid */
	if (ts_bdata->irq <= 0)
		core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	else
		core_data->irq = ts_bdata->irq;

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	r = devm_request_threaded_irq(&core_data->pdev->dev,
				      core_data->irq, NULL,
				      goodix_thp_threadirq_func,
				      ts_bdata->irq_flags | IRQF_ONESHOT,
				      GOODIX_CORE_DRIVER_NAME,
				      core_data);

	if (r < 0) {
		ts_err("Failed to requeset threaded irq:%d", r);
		return r;
	}

	mutex_lock(&core_data->irq_mutex);
	disable_irq(core_data->irq);
	core_data->irq_state = false;
	mutex_unlock(&core_data->irq_mutex);
	ts_info("%s: disable irq", __func__);

	return 0;
}

static int goodix_thp_suspend_input_dev_init(struct goodix_thp_core *core_data)
{
	struct input_dev *suspend_dev = NULL;
	int r;

	/* alloc input_dev */
	suspend_dev = input_allocate_device();
	if (!suspend_dev) {
		ts_err("Failed to alloc suspend input dev");
		return -ENOMEM;
	}
	core_data->suspend_dev = suspend_dev;
	input_set_drvdata(suspend_dev, core_data);

	/* init input_dev */
	suspend_dev->name = GOODIX_THP_SUSPEND_INPUT_DEVICE_NAME;
	suspend_dev->id.bustype = BUS_SPI;
	suspend_dev->phys = GOOIDX_INPUT_PHYS;
	suspend_dev->id.product = 0xDEAD;
	suspend_dev->id.vendor = 0xBEEF;
	suspend_dev->id.version = 10427;

	/* set input_dev properties */
	set_bit(EV_SYN, suspend_dev->evbit);
	set_bit(EV_KEY, suspend_dev->evbit);
	set_bit(EV_ABS, suspend_dev->evbit);
	set_bit(BTN_TOUCH, suspend_dev->keybit);
	set_bit(BTN_TOOL_FINGER, suspend_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, suspend_dev->propbit);

	input_set_abs_params(suspend_dev, ABS_MT_POSITION_X,
			0, core_data->ts_dev->board_data.panel_max_x, 0, 0);
	input_set_abs_params(suspend_dev, ABS_MT_POSITION_Y,
			0, core_data->ts_dev->board_data.panel_max_y, 0, 0);
	input_set_abs_params(suspend_dev, ABS_MT_PRESSURE,
			0, core_data->ts_dev->board_data.panel_max_p, 0, 0);
	input_set_abs_params(suspend_dev, ABS_MT_TOUCH_MAJOR,
			0, core_data->ts_dev->board_data.panel_max_w, 0, 0);
	input_set_abs_params(suspend_dev, ABS_MT_TRACKING_ID, 0, 1, 0, 0);

#ifdef GOODIX_THP_TYPE_B_PROTOCOL
	input_mt_init_slots(suspend_dev, 1, INPUT_MT_DIRECT);
#endif

	input_set_capability(suspend_dev, EV_KEY, KEY_WAKEUP);

	/* register input_dev */
	r = input_register_device(suspend_dev);
	if (r) {
		ts_err("failed to register suspend input device");
		return r;
	}

	return 0;
}

void goodix_thp_suspend_input_dev_exit(struct goodix_thp_core *core_data)
{
	input_unregister_device(core_data->suspend_dev);
	input_free_device(core_data->suspend_dev);
	core_data->suspend_dev = NULL;
}

static void goodix_thp_force_release_all(void)
{
	struct input_dev *input_dev = g_thp_input_agent->input_dev;
	int i;

	for (i = 0; i < INPUT_AGENT_MAX_POINTS; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, 0, 0);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);
}

static long goodix_thp_input_agent_ioctl_set_coordinate(unsigned long arg)
{
	long ret = 0;
	void __user *argp = (void __user *)arg;
	struct input_dev *input_dev = g_thp_input_agent->input_dev;
	struct thp_input_agent_ioctl_coor_data data;
	u8 i;
	static int pre_flags = 0;

	if (arg == 0) {
		ts_err("%s:arg is null.", __func__);
		return -EINVAL;
	}

	/* copy data from hal */
	if (copy_from_user(&data, argp,
			sizeof(struct thp_input_agent_ioctl_coor_data))) {
		ts_err("Failed to copy_from_user().");
		return -EFAULT;
	}

	gdix_thp_core->last_event_time = data.time_stamp;

	/* report coor to input system */
	for (i = 0; i < INPUT_AGENT_MAX_POINTS; i++) {
		//	ts_info("[%d]:touch_num %d, id %d, valid %d, x %d, y %d, p %d, w %d, type %d",
		//			i, data.touch_num, data.touch[i].track_id, data.touch[i].touch_valid,
		//			data.touch[i].x, data.touch[i].y, data.touch[i].p,
		//			data.touch[i].major, data.touch[i].touch_type);
#ifdef GOODIX_THP_TYPE_B_PROTOCOL
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev,
			data.touch[i].touch_type, data.touch[i].touch_valid != 0);
#endif

		if (data.touch[i].touch_valid != 0) {
			input_report_abs(input_dev, ABS_MT_POSITION_X,
						data.touch[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
						data.touch[i].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
						data.touch[i].p);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						data.touch[i].major);
			//	input_report_abs(input_dev, ABS_MT_TOUCH_MINOR,
			//				data.touch[i].minor);
			input_report_abs(input_dev, ABS_MT_TOOL_TYPE,
						data.touch[i].touch_type);
#ifndef GOODIX_THP_TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
		}
	}

	/* large touch flag */
	if (data.large_touch_stat) {
				//TODO
	} else {
				//TODO
	}

	/* BTN_TOUCH DOWN */
	if (data.touch_num > 0)
		input_report_key(input_dev, BTN_TOUCH, 1);

	/* BTN_TOUCH UP */
	if (data.touch_num == 0) {
#ifndef GOODIX_THP_TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
		input_report_key(input_dev, BTN_TOUCH, 0);
	}

	input_sync(input_dev);

	/* fp touch flag */
	if (pre_flags != data.fp_mode) {
		if (data.fp_mode) {
			goodix_thp_set_fp_int_pin(gdix_thp_core->ts_dev, 1);
			input_report_key(g_thp_input_agent->input_dev, BTN_TRIGGER_HAPPY1, 1);
			input_sync(g_thp_input_agent->input_dev);
			input_report_key(g_thp_input_agent->input_dev, BTN_TRIGGER_HAPPY1, 0);
			input_sync(g_thp_input_agent->input_dev);
			ts_info("report BTN_TRIGGER_HAPPY1");
		} else {
			goodix_thp_set_fp_int_pin(gdix_thp_core->ts_dev, 0);
			input_report_key(g_thp_input_agent->input_dev, BTN_TRIGGER_HAPPY2, 1);
			input_sync(g_thp_input_agent->input_dev);
			input_report_key(g_thp_input_agent->input_dev, BTN_TRIGGER_HAPPY2, 0);
			input_sync(g_thp_input_agent->input_dev);
			ts_info("report BTN_TRIGGER_HAPPY2");
		}
		pre_flags = data.fp_mode;
	}

	return ret;
}

static int goodix_thp_input_agent_ioctl_read_status(unsigned long arg)
{
	return 0;
}

static int goodix_thp_input_agent_ioctl_get_custom_info(unsigned long arg)
{
	char __user *custom_info = (char *)arg;
	struct goodix_thp_core *cd = gdix_thp_core;

	if (!cd || !custom_info) {
		ts_err("%s:args error", __func__);
		return -EINVAL;
	}

	ts_info("%s:custom info:%s", __func__, cd->custom_info);

	if(copy_to_user(custom_info, cd->custom_info, sizeof(cd->custom_info))) {
		ts_err("%s:copy window_info failed", __func__);
		return -EFAULT;
	}

	return 0;
}

static long goodix_thp_input_agent_ioctl_set_events(unsigned long arg)
{
	long ret = 0;

	return ret;
}

int goodix_thp_input_agent_ioctl_get_events(unsigned long arg)
{
	return 0;
}

static int goodix_thp_input_agent_ioctl_get_driver_state(unsigned long arg)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u32 __user *driver_state = (u32 *)arg;

	//ts_info("%s:driver state = %d", __func__, cd->suspended);

	if (driver_state == NULL) {
		ts_err("%s: input parameter null", __func__);
		return -EINVAL;
	}

	if(copy_to_user(driver_state, &cd->suspended, sizeof(u32))) {
		ts_err("%s:copy driver_state failed", __func__);
		return -EFAULT;
	}

	return 0;
}

static int goodix_thp_input_agent_open(struct inode *inode, struct file *filp)
{
	ts_info("%s:called", __func__);
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
	long ret;

	switch (cmd) {
	case INPUT_AGENT_IOCTL_CMD_SET_COOR:
		ret = goodix_thp_input_agent_ioctl_set_coordinate(arg);
		break;
	case INPUT_AGENT_IOCTL_READ_STATUS:
		ret = goodix_thp_input_agent_ioctl_read_status(arg);
		break;
	case INPUT_AGENT_IOCTL_GET_CUSTOM_INFO:
		ret = goodix_thp_input_agent_ioctl_get_custom_info(arg);
		break;
	case INPUT_AGENT_IOCTL_CMD_SET_EVENTS:
		ret = goodix_thp_input_agent_ioctl_set_events(arg);
		break;
	case INPUT_AGENT_IOCTL_CMD_GET_EVENTS:
		ret = goodix_thp_input_agent_ioctl_get_events(arg);
		break;
	case INPUT_AGENT_IOCTL_GET_DRIVER_STATE:
		ret = goodix_thp_input_agent_ioctl_get_driver_state(arg);
		break;
	default:
		ts_err("cmd unkown.");
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

static struct miscdevice g_thp_input_agent_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &g_thp_input_agent_fops,
};

static int goodix_thp_input_agent_init(struct goodix_thp_core *core_data)
{
	struct input_dev *input_dev;
	static struct thp_input_agent_data *input_agent;
	int r;

	if (g_thp_input_agent) {
		ts_err("%s:thp_input_agent have inited, exit", __func__);
		return 0;
	}

	/* alloc memory for input_agent */
	input_agent = kzalloc(sizeof(struct thp_input_agent_data), GFP_KERNEL);
	if (!input_agent) {
		ts_err("%s:out of memory", __func__);
		return -ENOMEM;
	}

	/* alloc input_dev */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("%s:Unable to allocated input device", __func__);
		kfree(input_agent);
		return	-ENODEV;
	}

	/* init input_dev */
	input_dev->name = GOODIX_THP_INPUT_DEVICE_NAME;
	input_dev->id.bustype = BUS_SPI;
	input_dev->phys = GOOIDX_INPUT_PHYS;
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
			0, core_data->ts_dev->board_data.panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, core_data->ts_dev->board_data.panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			0, core_data->ts_dev->board_data.panel_max_p, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, core_data->ts_dev->board_data.panel_max_w, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
		0, INPUT_AGENT_MAX_POINTS - 1, 0, 0);
	//	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR,
	//			0, input_agent->input_dev_config.minor_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
					0, MT_TOOL_MAX, 0, 0);
#ifdef GOODIX_THP_TYPE_B_PROTOCOL
	input_mt_init_slots(input_dev, INPUT_AGENT_MAX_POINTS, INPUT_MT_DIRECT);
#endif
	input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1);
	input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2);

	/* register input_dev */
	r = input_register_device(input_dev);
	if (r) {
		ts_err("%s:failed to register input device", __func__);
		goto input_dev_reg_err;
	}

	/* register input_agent misc dev */
	r = misc_register(&g_thp_input_agent_misc_device);
	if (r) {
		ts_err("%s:failed to register misc device", __func__);
		goto misc_dev_reg_err;
	}

	input_agent->input_dev = input_dev;
	g_thp_input_agent = input_agent;
	return 0;

misc_dev_reg_err:
	input_unregister_device(input_dev);
input_dev_reg_err:
	kfree(input_agent);
	return r;
}

static void goodix_thp_input_agent_exit(void)
{
	if (!g_thp_input_agent)
		return;

	input_unregister_device(g_thp_input_agent->input_dev);
	misc_deregister(&g_thp_input_agent_misc_device);
}

/* Description:switch scan_rate
 * @buf: 0/1/2/3/4 represent 300/240/180/120/60hz
 */
static ssize_t goodix_thp_scan_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_ts_device *tdev = core_data->ts_dev;
	int index = 0;

	if (sscanf(buf, "%d", &index) != 1)
		return -EINVAL;

	if (tdev->hw_ops->send_cmd(tdev, CMD_ACTIVE_SCAN_RATE, index))
		ts_err("goodix switch scan rate failed, index %d", index);

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

/* Description: debug read
 */
static ssize_t goodix_thp_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	//struct goodix_thp_core *core_data = gdix_thp_core;
	size_t offset = 0;


	return offset;
}

/* Description: debug write
 */
static ssize_t goodix_thp_debug_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	//struct goodix_thp_core *core_data = gdix_thp_core;
	int value = 0;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	return count;
}

static ssize_t goodix_thp_screen_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	size_t offset;



	offset = sprintf(buf, "%s\n", cd->suspended ? "off" : "on");
	return offset;
}

static ssize_t goodix_thp_screen_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_thp_core *cd = gdix_thp_core;
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
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[3];

	if (count < 2) {
		ts_err("invalid input param len:%zu", count);
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
	goodix_thp_ioctl_set_tsd_state(buf[0] != '0');
	return count;
}

static ssize_t goodix_thp_dump_rep_log_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[1] = {NOTIFY_TYPE_DUMP_REP};

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("dump rep log");
		put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 1);
	}
	return count;
}

static ssize_t goodix_thp_stylus_ctrl_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	goodix_thp_ioctl_set_stylus_state(buf[0] != '0');
	return count;
}

static ssize_t goodix_thp_rawdata_ctrl_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[2] = {NOTIFY_TYPE_RAWDATA, 0};

	if (buf[0] == 1 || buf[0] == '1')
		val[1] = 1;

	put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 2);
	return count;
}

static ssize_t save_moto_data_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[2] = {NOTIFY_TYPE_SAVE_MOTO_DATA, 0};

	if (buf[0] == 1 || buf[0] == '1')
		val[1] = 1;

	put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 2);
	return count;
}

static ssize_t goodix_thp_version_info(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *cd = gdix_thp_core;

	return snprintf(buf, PAGE_SIZE, "%s\n", cd->ts_dev->board_data.thp_ver);
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
	struct thp_ts_device *ts_dev = gdix_thp_core->ts_dev;
	int ret;

	if (!rw_addr || !rw_len) {
		ts_err("address(0x%x) and length(%d) can't be null",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = ts_dev->hw_ops->read(ts_dev, rw_addr, show_buf, rw_len);
	if (ret < 0) {
		ts_err("failed read addr(%x) length(%d)", rw_addr, rw_len);
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
    struct thp_ts_device *ts_dev = gdix_thp_core->ts_dev;
	char *pos = NULL;
	char *token = NULL;
	long result = 0;
	int i;

	if (!buf || !count) {
		ts_err("invalid parame");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err("string must start with 'r/w'");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid address info");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err("failed get addr info");
			goto err_out;
		}
		rw_addr = (u32)result;
		ts_info("rw addr is 0x%x", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid length info");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err("failed get length info");
			goto err_out;
		}
		rw_len = (u32)result;
		if (rw_len > sizeof(store_buf)) {
			ts_err("data len > %lu", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err("invalid data info");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err("failed get data[%d] info", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
		}
	}

	if (rw_addr == goodix_cmd_reg) {
		put_frame_list(gdix_thp_core, REQUEST_TYPE_CMD, store_buf, rw_len);
	} else {
		ts_dev->hw_ops->write(ts_dev, rw_addr, store_buf, rw_len);
	}

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;
}

static ssize_t goodix_thp_logtofile_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct goodix_thp_core *cd = gdix_thp_core;

	return sprintf(buf, "%s\n", cd->logtofile_on ? "on" : "off");
}

static ssize_t goodix_thp_logtofile_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct goodix_thp_core *cd = gdix_thp_core;
	u8 val[2] = {NOTIFY_TYPE_LOGTOFILE, 0};

	cd->logtofile_on = 0;
	if (buf[0] == 1 || buf[0] == '1') {
		val[1] = 1;
		cd->logtofile_on = 1;
	}

	put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, 2);
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
static DEVICE_ATTR(save_moto_data, S_IWUSR | S_IWGRP, NULL,
                                save_moto_data_store);
static DEVICE_ATTR(version_info, S_IRUGO, goodix_thp_version_info, NULL);
static DEVICE_ATTR(reg_rw, S_IRUGO | S_IWUSR | S_IWGRP,
		                goodix_thp_reg_rw_show, goodix_thp_reg_rw_store);
static DEVICE_ATTR(logtofile, S_IRUGO | S_IWUSR | S_IWGRP,
                                goodix_thp_logtofile_show, goodix_thp_logtofile_store);
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
	&dev_attr_save_moto_data.attr,
	&dev_attr_version_info.attr,
	&dev_attr_reg_rw.attr,
	&dev_attr_logtofile.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static int goodix_thp_sysfs_init(struct goodix_thp_core *core_data)
{
	int ret;

	ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (ret) {
		ts_err("failed create core sysfs group");
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

	ts_info("Suspend start");

	if (core_data->suspended == 1) {
		ts_info("Already in suspend mode, exit.");
		goto exit;
	}

	goodix_thp_set_irq_enable(core_data, IRQ_DISABLE_FLAG);
	core_data->suspended = 1;
	core_data->state_change_flag = 0;

	if (core_data->gesture_enable == 0) {
		/* power off */
		goodix_thp_power_off(core_data);
	} else {
		ts_info("enter gesture mode!");
		/* send enter gesture cmd */
		r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_GESTURE, gsx_data);
		if (r) {
			ts_err("send enter gesture cmd failed, r %d", r);
			goto exit;
		}
		goodix_thp_set_irq_enable(core_data, IRQ_ENABLE_FLAG);
		enable_irq_wake(core_data->irq);
	}
exit:
	goodix_thp_force_release_all();
	ts_info("Suspend end");
	return r;
}

static int goodix_thp_resume(struct goodix_thp_core *core_data)
{
	struct thp_ts_device *ts_dev = core_data->ts_dev;

	ts_info("Resume start");

	if (core_data->suspended == 0) {
		ts_info("Already in normal mode,exit.");
		goto exit;
	}

	goodix_thp_set_irq_enable(core_data, IRQ_DISABLE_FLAG);

	if (core_data->gesture_enable == 0) {
		/* power on */
		goodix_thp_power_on(core_data);
		msleep(100);
	} else {
		disable_irq_wake(core_data->irq);
		goodix_thp_reset(ts_dev, 100);
	}

	core_data->suspended = 0;
	core_data->state_change_flag = 1;
exit:
	goodix_thp_set_irq_enable(core_data, IRQ_ENABLE_FLAG);
	ts_info("Resume end");
	return 0;
}
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
static int goodix_thp_drm_notifier_callback(struct notifier_block *nb,
	unsigned long value, void *v)
{
	struct goodix_thp_core *cd =
		container_of(nb, struct goodix_thp_core, pm_notif);
	int *data = (int *)v;
	u8 val[2];

	if (!cd || !v) {
		ts_err("%s invalid parameters", __func__);
		return -1;
	}

	if (value == MTK_DISP_EVENT_BLANK) {
		/* resume: touch power on is after display to avoid display disturb */
		ts_info("%s IN, MTK_DISP_EVENT_BLANK", __func__);
		if (*data == MTK_DISP_BLANK_UNBLANK) {
			val[0] = NOTIFY_TYPE_SCREEN;
			val[1] = 1;
			put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		}
		ts_info("%s OUT", __func__);
	} else if (value == MTK_DISP_EARLY_EVENT_BLANK) {
		/**
		 * suspend: touch power off is before display to avoid touch report event
		 * after screen is off
		 */
		ts_info("%s IN, MTK_DISP_EARLY_EVENT_BLANK", __func__);
		if (*data == MTK_DISP_BLANK_POWERDOWN) {
			val[0] = NOTIFY_TYPE_SCREEN;
			val[1] = 0;
			put_frame_list(cd, REQUEST_TYPE_NOTIFY, val, sizeof(val));
		}
		ts_info("%s OUT", __func__);
	} else {
		ts_info("%s ignore disp value %d, data %d", __func__, value, *data);
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
	u8 val[2];

	blank = evdata->data;
	ts_info("FB event:%lu,blank:%d", event, *blank);
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
	int r = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_ts_device *ts_dev = core_data->ts_dev;

	ts_info("enter tui mode!");
	core_data->suspended = 1;

	/* stop frame data report */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_RAWDATA, RAWDATA_DISABLE);
	if (r)
		ts_err("send rawdata disable cmd failed, r %d", r);

	/* start touch data report */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_TOUCH_REPORT, TOUCH_DATA_ENABLE);
	if (r)
		ts_err("send touch_data enable cmd failed, r %d", r);

	/* switch to 60hz scan rate */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_ACTIVE_SCAN_RATE, SCAN_RATE_60);
	if (r)
		ts_err("send switch 60hz cmd failed, r %d", r);

	return 0;
}
EXPORT_SYMBOL_GPL(goodix_thp_enter_tui);

int goodix_thp_exit_tui(void)
{
	int r = 0;
	struct goodix_thp_core *core_data = gdix_thp_core;
	struct thp_ts_device *ts_dev = core_data->ts_dev;

	ts_info("enter tui mode!");
	core_data->suspended = 0;

	/* stop touch data report */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_TOUCH_REPORT, TOUCH_DATA_DISABLE);
	if (r)
		ts_err("send touch_data disable cmd failed, r %d", r);

	/* start frame data report */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_RAWDATA, RAWDATA_ENABLE);
	if (r)
		ts_err("send rawdata enable cmd failed, r %d", r);

	/* switch to 120hz scan rate */
	r = ts_dev->hw_ops->send_cmd(ts_dev, CMD_ACTIVE_SCAN_RATE, SCAN_RATE_120);
	if (r)
		ts_err("send switch 120hz cmd failed, r %d", r);

	return 0;
}
EXPORT_SYMBOL_GPL(goodix_thp_exit_tui);

/**
 * goodix_thp_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */
static int goodix_thp_probe(struct platform_device *pdev)
{
	struct goodix_thp_core *core_data = NULL;
	struct thp_ts_device *tdev;
	int r;

	ts_info("goodix_thp_probe IN");

	/*init thp core data */
	tdev = pdev->dev.platform_data;
	if (!tdev || !tdev->hw_ops) {
		ts_err("Invalid touch device");
		return -ENODEV;
	}

	core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_thp_core),
				 GFP_KERNEL);
	if (!core_data) {
		ts_err("Failed to allocate memory for core data");
		return -ENOMEM;
	}

	core_data->frame_mmap_list.buf = kmalloc(MMAP_BUFFER_SIZE, GFP_KERNEL);
	core_data->pdev = pdev;
	core_data->ts_dev = tdev;
	mutex_init(&core_data->frame_mutex);
	mutex_init(&core_data->ts_mutex);
	mutex_init(&core_data->irq_mutex);
	init_waitqueue_head(&(core_data->frame_wq));
	/* gesture init */
	memset(core_data->gesture_type, 0xff, GESTURE_TYPE_LEN);
	memset(core_data->gesture_data, 0xff, GESTURE_KEY_DATA_LEN);
	memset(core_data->gesture_buffer_data, 0xff, GESTURE_BUFFER_DATA_LEN);
	platform_set_drvdata(pdev, core_data);

	gdix_thp_core = core_data;
	gdix_thp_core->sdev = tdev->spi_dev;

	/* get GPIO resource*/
	r = goodix_thp_gpio_setup(core_data);
	if (r < 0) {
		ts_err("setup gpio failed, r %d", r);
		goto out;
	}

	/* power init & power on */
	r = goodix_thp_power_init(core_data);
	if (r < 0) {
		ts_err("power init failed, r %d", r);
		goto out;
	}

	r = goodix_thp_power_on(core_data);
	if (r < 0) {
		ts_err("power on failed, r %d", r);
		goto out;
	}

	/* board init */
	r = tdev->hw_ops->board_init(tdev);
	if (r) {
		ts_err("goodix device chip detect failed, r %d", r);
		goto out;
	}

	/* get custom info */
	r = tdev->hw_ops->get_custom_info(tdev, core_data->custom_info,
					GOODIX_THP_CUSTOM_INFO_LEN);
	if (r) {
		ts_err("goodix get custom info failed, r %d", r);
		goto out;
	}
	core_data->custom_info[GOODIX_THP_CUSTOM_INFO_LEN] = '\0';

	/* register misc dev */
	r = misc_register(&g_thp_misc_device);
	if (r) {
		ts_err("failed to register misc device '/dev/thp', r %d", r);
		goto out;
	}

	/* init input_dev for report coor when suspend */
	r = goodix_thp_suspend_input_dev_init(core_data);
	if (r) {
		ts_err("failed to init suspend input dev, r %d", r);
		goto err_init_suspend_dev;
	}

	/* init input_agent */
	r = goodix_thp_input_agent_init(core_data);
	if (r) {
		ts_err("failed to init gdix_input_agent, r %d", r);
		goto err_init_wrapper;
	}

	/* init sysfs */
	r = goodix_thp_sysfs_init(core_data);
	if (r) {
		ts_err("failed to create sysfs, r %d", r);
		goto err_sysfs_init;
	}

	/* request irq */
	r = goodix_thp_irq_setup(core_data);
	if (r) {
		ts_err("goodix setup irq failed, r %d", r);
		goto err_irq_setup;
	}
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	core_data->pm_notif.notifier_call = goodix_thp_drm_notifier_callback;
	if (mtk_disp_notifier_register("Touch", &core_data->pm_notif))
		ts_err("Failed to register disp notifier client:%d", ret);
#elif IS_ENABLED(CONFIG_FB)
	core_data->pm_notif.notifier_call = goodix_thp_fb_notifier_callback;
	r = fb_register_client(&core_data->pm_notif);
	if (r < 0)
		ts_err("[FB]Unable to register fb_notifier, ret:%d", r);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	ts_info("%s:goodix_ts_mmi_dev_register",__func__);
	r = goodix_ts_mmi_dev_register(pdev);
	if (r) {
		ts_info("Failed register touchscreen mmi.");
		goto out;
	}
#endif

	return 0;

err_irq_setup:
	goodix_thp_sysfs_exit(core_data);
err_sysfs_init:
	goodix_thp_input_agent_exit();
err_init_wrapper:
	goodix_thp_suspend_input_dev_exit(core_data);
err_init_suspend_dev:
	misc_deregister(&g_thp_misc_device);
out:
	ts_info("goodix_thp_probe OUT, r:%d", r);
	return r;
}

static int goodix_thp_remove(struct platform_device *pdev)
{
	struct goodix_thp_core *core_data = gdix_thp_core;

	ts_info("IN");
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	ts_info("%s:goodix_ts_mmi_dev_unregister",__func__);
	goodix_ts_mmi_dev_unregister(pdev);
#endif
	goodix_thp_power_off(core_data);
	goodix_thp_sysfs_exit(core_data);
	goodix_thp_input_agent_exit();
	goodix_thp_suspend_input_dev_exit(core_data);
	misc_deregister(&g_thp_misc_device);
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	if (mtk_disp_notifier_unregister(&core_data->pm_notif))
		ts_info("Error occurred when unregister disp_notifier");
#elif IS_ENABLED(CONFIG_FB)
	fb_unregister_client(&core_data->pm_notif);
#endif
	kfree(core_data->frame_mmap_list.buf);
	return 0;
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
	ts_info("IN");
	ts_info("goodix thp driver v%s", GOODIX_THP_DRIVER_VERSION);

	return platform_driver_register(&thp_core_driver);
}

int goodix_thp_core_deinit(void)
{
	ts_info("IN");

	platform_driver_unregister(&thp_core_driver);
	return 0;
}
