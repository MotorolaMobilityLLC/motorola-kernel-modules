/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 47376 $
 * $Date: 2019-07-12 09:06:29 +0800 (週五, 12 七月 2019) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/power_supply.h>
#include <linux/version.h>

#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#ifdef NVT_SENSOR_EN
#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock gesture_wakelock;
#else
static struct wakeup_source *gesture_wakelock;
#endif
static struct sensors_classdev __maybe_unused sensors_touch_cdev = {

	.name = "dt-gesture",
	.vendor = "Novatek",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_DOUBLE_TAP,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "1",
	.min_delay = 0,
	.max_delay = 0,
	/* WAKE_UP & SPECIAL_REPORT */
	.flags = 1 | 6,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#define REPORT_MAX_COUNT 10000
#endif

#if NVT_TOUCH_ESD_PROTECT
struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
extern void nvt_extra_proc_deinit(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
extern void nvt_mp_proc_deinit(void);
#endif

struct nvt_ts_data *ts;

#if defined(NVT_CONFIG_MULTI_SUPPLIER)
#define NVT_MAX_SUPPLIER_LEN		16
char lcm_panel_supplier[NVT_MAX_SUPPLIER_LEN] = {0};
#endif

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#ifdef NOVATECH_PEN_NOTIFIER
int nvt_mcu_pen_detect_set(uint8_t pen_detect);
#endif

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
static int nvt_disp_notifier_callback(struct notifier_block *nb,
	unsigned long value, void *v);
#endif

#ifdef NVT_MTK_CHECK_PANEL
const char *active_panel_name;
#endif

static int32_t nvt_ts_resume(struct device *dev);
static int32_t nvt_ts_suspend(struct device *dev);

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR = 0; //read from dtsi
uint32_t SPI_RD_FAST_ADDR = 0;	//read from dtsi
uint32_t panel_wakeup = 0;	//read from dtsi
static int charger_notifier_callback(struct notifier_block *nb, unsigned long val, void *v);
static int charger_set_state(struct power_supply *psy, struct usb_charger_detection *chg_detect);
static int nvt_set_charger(uint8_t charger_on_off);
static void nvt_charger_notify_work(struct work_struct *work);
static int usb_detect_flag = 0;

char *nvt_boot_firmware_name = NULL;
char *nvt_mp_firmware_name = NULL;

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK, GESTURE_SINGLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

#ifdef CONFIG_MTK_SPI
const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 5,	/* 10MHz (SPI_SPEED=100M / (high_time+low_time(10ns)))*/
	.low_time = 5,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

#ifdef CONFIG_SPI_MT65XX
const struct mtk_chip_config spi_ctrdata = {
#ifdef MT6768_NOVA_SPI_CONF
	.rx_mlsb = 1,
	.tx_mlsb = 1,
#endif
	.sample_sel = 1,
	.cs_setuptime = 2148,
};
#endif

#ifdef CONFIG_NVT_MTK_SPI
const struct mtk_chip_config spi_ctrdata = {
	.cs_holdtime = 0,
	.cs_idletime = 0,
	.tick_delay = 0,
	.sample_sel = 0,
	.cs_setuptime = 50,
};
#endif

#ifdef NVT_TOUCH_LAST_TIME
static bool time_flag = 1;
#endif

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	struct irq_desc *desc;

	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}

	desc = irq_to_desc(ts->client->irq);
	NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen enable hw bld crc function.

return:
	N/A.
*******************************************************/
void nvt_bld_crc_enable(void)
{
	uint8_t buf[4] = {0};

	//---set xdata index to BLD_CRC_EN_ADDR---
	nvt_set_page(ts->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	//---write data to index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST

	if (SPI_RD_FAST_ADDR) {
		/* disable SPI_RD_FAST */
		nvt_write_addr(SPI_RD_FAST_ADDR, 0x00);
	}
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			NVT_DBG("pass, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];
	ts->fw_type = buf[14];
	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;
		ts->fw_type = 1;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, "
					"abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	NVT_LOG("fw_ver=%d, x_num=%d, y_num=%d, abs_xmax=%d, abs_y_max=%d, max_button_num=%d!\n", ts->fw_ver, ts->x_num, ts->y_num, ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
	NVT_LOG("FW type is 0x%02X\n", buf[14]);

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTSPI"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	int32_t ret = 0;
	int32_t retries = 0;
	int8_t spi_wr = 0;
	uint8_t *buf;

	if ((count > NVT_TRANSFER_LEN + 3) || (count < 3)) {
		NVT_ERR("invalid transfer len!\n");
		return -EFAULT;
	}

	/* allocate buffer for spi transfer */
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		goto kzalloc_failed;
	}

	buf = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
	if(buf == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		kfree(str);
		str = NULL;
		goto kzalloc_failed;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		ret = -EFAULT;
		goto out;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	spi_wr = str[0] >> 7;
	memcpy(buf, str+2, ((str[0] & 0x7F) << 8) | str[1]);

	if (spi_wr == NVTWRITE) {	//SPI write
		while (retries < 20) {
			ret = CTP_SPI_WRITE(ts->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else if (spi_wr == NVTREAD) {	//SPI read
		while (retries < 20) {
			ret = CTP_SPI_READ(ts->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		memcpy(str+2, buf, ((str[0] & 0x7F) << 8) | str[1]);
		// copy buff to user if spi transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count)) {
				ret = -EFAULT;
				goto out;
			}
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		ret = -EFAULT;
		goto out;
	}

out:
	kfree(str);
    kfree(buf);
kzalloc_failed:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct proc_ops nvt_flash_fops = {
	.proc_open = nvt_flash_open,
	.proc_release = nvt_flash_close,
	.proc_read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/%s\n", DEVICE_NAME);
	NVT_LOG("============================================================\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI deinitial function.

return:
	n.a.
*******************************************************/
static void nvt_flash_proc_deinit(void)
{
	if (NVT_proc_entry != NULL) {
		remove_proc_entry(DEVICE_NAME, NULL);
		NVT_proc_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", DEVICE_NAME);
	}
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_SINGLE_CLICK    25
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];
#ifdef NVT_SENSOR_EN
	static int report_cnt = 0;
#endif

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_DBG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_DBG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_DBG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
#ifdef NVT_DOUBLE_TAP_CTRL
		case GESTURE_SINGLE_CLICK:
			keycode = KEY_F1;
			NVT_LOG("Gesture : Single Click, keycode KEY_F1:%d\n", keycode);
			break;
		case GESTURE_DOUBLE_CLICK:
			keycode = KEY_F4;
			NVT_LOG("Gesture : Double Click. keycode KEY_F4:%d\n", keycode);
			break;
#else
		case GESTURE_SINGLE_CLICK:
		case GESTURE_DOUBLE_CLICK:
			NVT_DBG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
#ifdef NVT_SENSOR_EN
			keycode = KEY_F1;
			NVT_LOG("Gesture : Tap Click. keycode KEY_F1:%d\n", keycode);
#endif
			break;
#endif
		case GESTURE_WORD_Z:
			NVT_DBG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_DBG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_DBG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_DBG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_DBG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_DBG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_DBG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_DBG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_DBG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
#ifdef NVT_SENSOR_EN
		if (!(ts->wakeable && ts->should_enable_gesture)) {
			NVT_LOG("Gesture got but wakeable not set. Skip this gesture.");
			return;
		}
		if (ts->report_gesture_key) {
			input_report_key(ts->sensor_pdata->input_sensor_dev, keycode, 1);
			input_sync(ts->sensor_pdata->input_sensor_dev);
			input_report_key(ts->sensor_pdata->input_sensor_dev, keycode, 0);
			input_sync(ts->sensor_pdata->input_sensor_dev);
#ifdef NVT_DOUBLE_TAP_CTRL
			if ((GESTURE_SINGLE_CLICK == gesture_id) || (GESTURE_DOUBLE_CLICK == gesture_id))
				NVT_LOG("report gesture keycode:%d", keycode);
#endif
			++report_cnt;
		} else {
			input_report_abs(ts->sensor_pdata->input_sensor_dev,
					ABS_DISTANCE,
					++report_cnt);
			input_sync(ts->sensor_pdata->input_sensor_dev);
		}
		NVT_LOG("input report: %d", report_cnt);
		if (report_cnt >= REPORT_MAX_COUNT) {
			report_cnt = 0;
		}
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_timeout(&gesture_wakelock, msecs_to_jiffies(5000));
#else
		PM_WAKEUP_EVENT(gesture_wakelock, 5000);
#endif
#else
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
#endif
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int nvt_get_dt_def_coords(struct device *dev, char *name)
{
	u32 coords[TOUCH_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != TOUCH_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		NVT_LOG("Unable to read novatek,def-max-resolution\n");
		return rc;
	}

	ts->abs_x_max = coords[0];
	ts->abs_y_max = coords[1];
	return rc;
}

#ifdef NVT_CONFIG_MULTI_SUPPLIER
static nvt_get_panel_supplier() {
	//parse panel supplier from cmdline
	if (saved_command_line) {
		char *sub;
		char key_prefix[] = "panel_supplier=";

		sub = strstr(saved_command_line, key_prefix);
		if (sub) {
			char *split;
			int len, len_prefix = strlen(key_prefix);

			split = strstr(sub, " ");
			if (split) {
				len = strlen(sub) - strlen(split) - len_prefix;
			} else {
				len = strlen(sub) - len_prefix;
			}

			if (len <=0)
				goto end;

			strncpy(lcm_panel_supplier, (sub + len_prefix), min(len, NVT_MAX_SUPPLIER_LEN));
			ts->panel_supplier = lcm_panel_supplier;
			NVT_LOG("panel_supplier=%s\n", ts->panel_supplier);
			return 0;
		} else {
			NVT_LOG("panel_supplier not found!");
		}
	} else {
		NVT_LOG("saved_command_line null!");
	}

	end:
	NVT_LOG("end ret -1\n");
	return -1;
}
#endif

#ifdef NVT_CHECK_DEVICE_BOOTMODE
static bool nvt_skip_charger_bootmode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool is_chargermode = false;
	int ret;
	const char *bootargs = NULL;
	char *bootmode = NULL;

	NVT_DBG("enter\n");
	if (!np) {
		NVT_LOG("chosen node NULL\n");
		return false;
	}

#ifdef CONFIG_BOOT_CONFIG
	ret = of_property_read_string(np, "mmi,bootconfig", &bootargs);
#else
	ret = of_property_read_string(np, "bootargs", &bootargs);
#endif

	NVT_DBG("ret=%d, bootargs=%s", ret, bootargs);
	if (!ret && bootargs) {
		bootmode = strstr(bootargs, "androidboot.mode=");
		if (bootmode) {
			NVT_LOG("bootmode info: %s\n", bootmode);
			bootmode = strpbrk(bootmode, "=");
			if (bootmode && (strlen(bootmode) > 1)) {
				bootmode++;
				NVT_INFO("bootmode=%s\n", bootmode);
				if (bootmode && !strncmp(bootmode, "charger", strlen("charger"))) {
					is_chargermode = true;
					NVT_LOG("is_chargermode true\n");
				}
			}
		}
		else
			NVT_LOG("bootmode null\n");
	}
	else
		NVT_LOG("get bootargs fail\n");

	of_node_put(np);

	return is_chargermode;
}
#endif

static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int32_t ret = 0;
#ifdef NVT_DOUBLE_TAP_CTRL
	uint32_t value;

	if (!of_property_read_u32(np, "novatek,supported_gesture_type", &value)) {
		ts->supported_gesture_type = (uint8_t)value;
		NVT_LOG("novatek,supported_gesture_type=%d\n", ts->supported_gesture_type);
	}
	else
		NVT_LOG("novatek,supported_gesture_type not set\n");

#endif

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

#if defined(NVT_CONFIG_MULTI_SUPPLIER)
	ret = nvt_get_panel_supplier();
#else
	ret = of_property_read_string(np, "novatek,panel-supplier",
		&ts->panel_supplier);
#endif
	if (ret < 0) {
		NVT_LOG("Unable to read panel supplier\n");
	} else {
		NVT_LOG("panel supplier is %s", (char *)ts->panel_supplier);
		nvt_boot_firmware_name = kzalloc(NVT_FILE_NAME_LENGTH, GFP_KERNEL);
		if (!nvt_boot_firmware_name) {
			NVT_LOG("%s: alloc nvt_boot_firmware_name failed\n", __func__);
			goto nvt_boot_firmware_name_alloc_failed;
		}
		nvt_mp_firmware_name = kzalloc(NVT_FILE_NAME_LENGTH, GFP_KERNEL);
		if (!nvt_boot_firmware_name) {
			NVT_LOG("%s: alloc nvt_boot_firmware_name failed\n", __func__);
			goto nvt_mp_firmware_name_alloc_failed;
		}
		snprintf(nvt_boot_firmware_name, NVT_FILE_NAME_LENGTH, "%s_novatek_ts_fw.bin",
			ts->panel_supplier);
		snprintf(nvt_mp_firmware_name, NVT_FILE_NAME_LENGTH, "%s_novatek_ts_mp.bin",
			ts->panel_supplier);
	}
	NVT_LOG("boot firmware %s, mp firmware %s", nvt_boot_firmware_name, nvt_mp_firmware_name);

	ret = nvt_get_dt_def_coords(dev, "novatek,def-max-resolution");
	if (ret) {
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	}
	NVT_LOG("novatek,def-max-resolution=%d,%d\n", ts->abs_x_max, ts->abs_y_max);

	ret = of_property_read_u32(np, "novatek,def-build-id", &ts->build_id);
	if (ret) {
		ts->build_id = 0;
		NVT_LOG("novatek,build_id undefined.\n");
	} else {
		NVT_LOG("novatek,build_id=0x%04X\n", ts->build_id);
	}

	ret = of_property_read_u32(np, "novatek,def-config-id", &ts->config_id);
	if (ret) {
		ts->config_id = 0;
		NVT_LOG("novatek,config_id undefined.\n");
	} else {
		NVT_LOG("novatek,config_id=0x%04X\n", ts->config_id);
	}

	ret = of_property_read_u32(np, "novatek,spi-rd-fast-addr", &SPI_RD_FAST_ADDR);
	if (ret) {
		NVT_LOG("not support novatek,spi-rd-fast-addr\n");
		SPI_RD_FAST_ADDR = 0;
		ret = 0;
	} else {
		NVT_LOG("SPI_RD_FAST_ADDR=0x%06X\n", SPI_RD_FAST_ADDR);
	}
	if (of_property_read_bool(np, "novatek,usb_charger")) {
		NVT_LOG("novatek,usb_charger set");
		ts->charger_detection_enable = 1;
		of_property_read_string(np, "novatek,psy-name", &ts->psy_name);
		if(of_property_read_bool(np, "novatek,usb-psp-online")) {
			NVT_LOG("novatek,usb-psp-online set\n");
			ts->usb_psp_online = 1;
		}
	} else {
		ts->charger_detection_enable = 0;
	}

	if (of_property_read_bool(np, "novatek,report_gesture_key")) {
		NVT_LOG("novatek,report_gesture_key set");
		ts->report_gesture_key = 1;
	} else {
		ts->report_gesture_key = 0;
	}
	if (of_property_read_bool(np, "novatek,panel_wakeup")) {
		NVT_LOG("novatek,panel_wakeup set");
		panel_wakeup = 1;
	} else {
		panel_wakeup = 0;
	}

	return ret;

nvt_mp_firmware_name_alloc_failed:
	kfree(nvt_boot_firmware_name);
	nvt_boot_firmware_name = NULL;
nvt_boot_firmware_name_alloc_failed:
	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
		if(nvt_boot_firmware_name)
			nvt_update_firmware(nvt_boot_firmware_name);
		else
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
		mutex_unlock(&ts->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 10;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

#if POINT_DATA_CHECKSUM
static int32_t nvt_ts_point_data_checksum(uint8_t *buf, uint8_t length)
{
   uint8_t checksum = 0;
   int32_t i = 0;

   // Generate checksum
   for (i = 0; i < length - 1; i++) {
       checksum += buf[i + 1];
   }
   checksum = (~checksum + 1);

   // Compare ckecksum and dump fail data
   if (checksum != buf[length]) {
       NVT_ERR("i2c/spi packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n",
               length, buf[length], checksum);

       for (i = 0; i < 10; i++) {
           NVT_LOG("%02X %02X %02X %02X %02X %02X\n",
                   buf[1 + i*6], buf[2 + i*6], buf[3 + i*6], buf[4 + i*6], buf[5 + i*6], buf[6 + i*6]);
       }

       NVT_LOG("%02X %02X %02X %02X %02X\n", buf[61], buf[62], buf[63], buf[64], buf[65]);

       return -1;
   }

   return 0;
}
#endif /* POINT_DATA_CHECKSUM */

#define FINGER_ENTER 0x01
#define FINGER_MOVING 0x02
#ifdef PALM_GESTURE
#define PALM_TOUCH 0x05
#define POINT_DATA_LEN 120
#define MINOR_DATA_OFFSET 70
#define ORIENT_DATA_OFFSET 110
#else
#define POINT_DATA_LEN 65
#endif

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1 + DUMMY_BYTES] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_major = 0;
#ifdef PALM_GESTURE
	uint32_t input_minor = 0;
	int32_t input_orient = 0;
#endif
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;

#if WAKEUP_GESTURE
	if (ts->bTouchIsAwake == 0) {
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	mutex_lock(&ts->lock);

	ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//--- dump SPI buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ",
			point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
*/

#if NVT_TOUCH_WDT_RECOVERY
   /* ESD protect by WDT */
	if (nvt_wdt_fw_recovery(point_data)) {
		NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
		if(nvt_boot_firmware_name)
			nvt_update_firmware(nvt_boot_firmware_name);
		else
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
		goto XFER_ERROR;
   }
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
#if NVT_TOUCH_ESD_PROTECT
		nvt_esd_check_enable(true);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		goto XFER_ERROR;
	}

#if POINT_DATA_CHECKSUM
   if (POINT_DATA_LEN >= POINT_DATA_CHECKSUM_LEN) {
       ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_CHECKSUM_LEN);
       if (ret) {
           goto XFER_ERROR;
       }
   }
#endif /* POINT_DATA_CHECKSUM */

#if WAKEUP_GESTURE
	if (ts->bTouchIsAwake == 0) {
		input_id = (uint8_t)(point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		mutex_unlock(&ts->lock);
		return IRQ_HANDLED;
	}
#endif

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;
#ifdef PALM_GESTURE
		if (((point_data[position] & 0x07) == FINGER_ENTER)
			|| ((point_data[position] & 0x07) == FINGER_MOVING)
			|| ((point_data[position] & 0x07) == PALM_TOUCH)) {	//finger down (enter & moving) or palm
#else
		if (((point_data[position] & 0x07) == FINGER_ENTER) || ((point_data[position] & 0x07) == FINGER_MOVING)) {	//finger down (enter & moving)
#endif
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;

#ifdef PALM_GESTURE_RANGE
			if((point_data[position] & 0x07) == PALM_TOUCH) { //palm
				input_minor = (uint32_t)(point_data[1 + MINOR_DATA_OFFSET + i]);
				if (input_minor == 0)
					input_minor = 1;

				input_orient = (int8_t)(point_data[1 + ORIENT_DATA_OFFSET + i]);
			}
#endif
			input_major = (uint32_t)(point_data[position + 4]);
			if (input_major == 0)
				input_major = 1;

#ifdef PALM_GESTURE
			if ((point_data[position] & 0x07) == PALM_TOUCH) { //palm
				input_p = (uint32_t)(point_data[position + 5]);
				if(input_p == 255)
					input_p = PALM_HANG;
				else if (input_p == 254)
					input_p = PALM_HAND_HOLDE;

#ifdef PALM_GESTURE_RANGE
				//dimension-to-pixel
				input_minor = input_minor * TOUCH_DEFAULT_MAX_WIDTH / PANEL_REAL_WIDTH * 100;
				input_major = input_major * TOUCH_DEFAULT_MAX_HEIGHT / PANEL_REAL_HEIGHT * 100;
#endif
			} else
#endif
			{
				if (i < 2) {
					input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
					if (input_p > TOUCH_FORCE_NUM)
						input_p = TOUCH_FORCE_NUM;
				} else {
					input_p = (uint32_t)(point_data[position + 5]);
				}
			}
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

#ifdef PALM_GESTURE
			if((point_data[position] & 0x07) == PALM_TOUCH) { //palm
				dev_dbg(&ts->client->dev, "id=%d, pressure=%d, (%d,%d), major=%d, minor=%d, orient=%d\n",
					input_id, input_p, input_x, input_y, input_major, input_minor, input_orient);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_major);
#ifdef PALM_GESTURE_RANGE
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, input_minor);
				input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, input_orient);
#endif
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
			} else
#endif
			{
				dev_dbg(&ts->client->dev, "id=%d, pressure=%d, (%d,%d), major=%d\n",
					input_id, input_p, input_x, input_y, input_major);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_major);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
			}
#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
#ifdef PALM_GESTURE_RANGE
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, 0);
#endif
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

#ifdef NVT_TOUCH_LAST_TIME
	if (finger_cnt > 0) {
		//touch down/moving
		if(time_flag) {
			ts->last_event_time = ktime_get_boottime();
			//skip all other touch moving events before UP
			time_flag = 0;
			NVT_DBG("set last_event_time\n");
		}
	} else {
		//touch UP, enable time flag for next touch
		time_flag = 1;
		NVT_DBG("finger_cnt 0, reset time_flag\n");
	}
#endif

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

XFER_ERROR:

	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(uint32_t chip_ver_trim_addr)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset();

		nvt_set_page(chip_ver_trim_addr);

		buf[0] = chip_ver_trim_addr & 0x7F;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("nvt_ts_check_chip_ver_trim: buf[0]=0x%02X\n", buf[0]);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ts->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				strncpy(ts->product_id, trim_id_table[list].trim_id, 10);
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
static int nvt_tp_power_on_reinit(void)
{
	int32_t ret;

	pr_info("%s is called\n", __func__);

	/* do esd recovery, bootloader reset */
	ret = nvt_ts_suspend(&ts->client->dev);
	if (ret) {
		pr_info("%s  is called suspend %d\n", __func__, ret);
		return ret;
	}

	ret = nvt_ts_resume(&ts->client->dev);
	if (ret)
		pr_info("%s  is called resume %d\n", __func__, ret);

	return ret;
}
#endif

#ifdef NVT_SENSOR_EN
static int nvt_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
#ifndef NVT_DOUBLE_TAP_CTRL
	NVT_LOG("Gesture set enable %d!", enable);
	mutex_lock(&ts->state_mutex);
	if (enable == 1) {
		ts->should_enable_gesture = true;
	} else if (enable == 0) {
		ts->should_enable_gesture = false;
	} else {
		NVT_LOG("unknown enable symbol\n");
	}
	mutex_unlock(&ts->state_mutex);
#endif
	return 0;
}

static int nvt_sensor_init(struct nvt_ts_data *data)
{
	struct nvt_sensor_platform_data *sensor_pdata;
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		NVT_ERR("Failed to allocate device");
		goto exit;
	}

	sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct nvt_sensor_platform_data),
			GFP_KERNEL);
	if (!sensor_pdata) {
		NVT_ERR("Failed to allocate memory");
		goto free_sensor_pdata;
	}
	data->sensor_pdata = sensor_pdata;

	if (data->report_gesture_key) {
		__set_bit(EV_KEY, sensor_input_dev->evbit);
		__set_bit(KEY_F1, sensor_input_dev->keybit);
#ifdef NVT_DOUBLE_TAP_CTRL
		__set_bit(KEY_F4, sensor_input_dev->keybit);
#endif
	} else {
		__set_bit(EV_ABS, sensor_input_dev->evbit);
		input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
				0, REPORT_MAX_COUNT, 0, 0);
	}
	__set_bit(EV_SYN, sensor_input_dev->evbit);

	sensor_input_dev->name = "double-tap";
	data->sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		NVT_ERR("Unable to register device, err=%d", err);
		goto free_sensor_input_dev;
	}

	sensor_pdata->ps_cdev = sensors_touch_cdev;
	sensor_pdata->ps_cdev.sensors_enable = nvt_sensor_set_enable;
	sensor_pdata->data = data;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
	input_free_device(data->sensor_pdata->input_sensor_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, sensor_pdata);
	data->sensor_pdata = NULL;
exit:
	return 1;
}

int nvt_sensor_remove(struct nvt_ts_data *data)
{
	sensors_classdev_unregister(&data->sensor_pdata->ps_cdev);
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
	devm_kfree(&data->sensor_pdata->input_sensor_dev->dev,
		data->sensor_pdata);
	data->sensor_pdata = NULL;
	data->wakeable = false;
	data->should_enable_gesture = false;
	if (!nvt_boot_firmware_name) {
		kfree(nvt_boot_firmware_name);
		nvt_boot_firmware_name = NULL;
	}
	if (!nvt_mp_firmware_name) {
		kfree(nvt_mp_firmware_name);
		nvt_mp_firmware_name = NULL;
	}
	return 0;
}

#ifdef NVT_DOUBLE_TAP_CTRL
void nvt_gesture_state_switch(void)
{
	if (ts->sys_gesture_type) {
		//gesture enable
		if (!ts->should_enable_gesture) {
			ts->should_enable_gesture = true;
			touch_set_state(TOUCH_LOW_POWER_STATE, TOUCH_PANEL_IDX_PRIMARY);
			NVT_LOG("gesture switch to enable");
		}
	}
	else {
		//gesture disable
		if (ts->should_enable_gesture) {
			ts->should_enable_gesture = false;
			touch_set_state(TOUCH_DEEP_SLEEP_STATE, TOUCH_PANEL_IDX_PRIMARY);
			NVT_LOG("gesture switch to disable");
		}
	}
}
#endif

#endif

#include <linux/slab.h>
#include <linux/major.h>
#include <linux/kdev_t.h>

static char *mmi_kobject_get_path(struct kobject *kobj, gfp_t gfp_mask)
{
	char *path;
	int len = 1;
	struct kobject *parent = kobj;

	do {
		if (parent->name == NULL) {
			len = 0;
			break;
		}
		len += strlen(parent->name) + 1;
		parent = parent->parent;
	} while (parent);

	if (len == 0)
		return NULL;

	path = kzalloc(len, gfp_mask);
	if (!path)
		return NULL;

	--len;
	for (parent = kobj; parent; parent = parent->parent) {
		int cur = strlen(parent->name);
		len -= cur;
		memcpy(path + len, parent->name, cur);
		*(path + --len) = '/';
	}
	pr_debug("kobject: '%s' (%p): %s: path = '%s'\n", kobj->name,
		kobj, __func__, path);

	return path;
}

/* Attribute: path (RO) */
static ssize_t path_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t blen;
	const char *path;

	path = mmi_kobject_get_path(&ts->client->dev.kobj, GFP_KERNEL);
	blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
	kfree(path);
	return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "novatek_ts");
}

static ssize_t ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int buildid;

	buildid = ts->fw_ver << 8 | ts->fw_type;
	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%04x\n%s%04x\n",
			"Product ID: ", ts->product_id,
			"Build ID: ", buildid ? buildid : ts->build_id,
			"Config ID: ", ts->nvt_pid ? ts->nvt_pid : ts->config_id);
}

#ifdef PALM_GESTURE
static ssize_t nvt_palm_settings_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", ts->palm_enabled);
}

static ssize_t nvt_palm_settings_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)

{
	int value;
	int err = 0;

	if (count > 2)
		return -EINVAL;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	err = count;

	switch (value) {
		case 0:
			ts->palm_enabled = false;
			break;
		case 1:
			ts->palm_enabled = true;
			break;
		default:
			err = -EINVAL;
			ts->palm_enabled = false;
			NVT_ERR("Invalid Value! %d\n", value);
			break;
	}

	nvt_palm_set(ts->palm_enabled);

	return err;
}
#endif

#ifdef NVT_TOUCH_LAST_TIME
static ssize_t timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ktime_t last_ktime;
	struct timespec64 last_ts;

	last_ktime = ts->last_event_time;
	ts->last_event_time = 0;

	last_ts = ktime_to_timespec64(last_ktime);
	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
#endif

#ifdef NVT_DOUBLE_TAP_CTRL
/*
 * gesture value used to indicate which gesture mode type is supported
 */
static ssize_t gesture_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	NVT_LOG("supported_gesture_type=%d", ts->supported_gesture_type);
	return scnprintf(buf, PAGE_SIZE, "%02x\n", ts->supported_gesture_type);
}

/*
 * gesture value used to indicate which gesture mode type is supported.
 * the gesture node will be stored when gesture mode updated from system
 */
static ssize_t gesture_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	int err = 0;

	err = sscanf(buf, "%d", &value);
	if (err < 0) {
		NVT_LOG("Failed to convert value\n");
		return -EINVAL;
	}

	err = count;
	switch (value) {
/*
//zero tap, not support yet
		case 0x10:
			NVT_LOG("zero tap disable\n");
			ts->sys_gesture_type &= 0xFE;
			break;
		case 0x11:
			NVT_LOG("zero tap enable\n");
			ts->sys_gesture_type |= 0x01;
			break;
*/
		case 0x20:
			NVT_LOG("single tap disable\n");
			ts->sys_gesture_type &= 0xFD;
			break;
		case 0x21:
			NVT_LOG("single tap enable\n");
			ts->sys_gesture_type |= 0x02;
			break;
		case 0x30:
			NVT_LOG("double tap disable\n");
			ts->sys_gesture_type &= 0xFB;
			break;
		case 0x31:
			NVT_LOG("double tap enable\n");
			ts->sys_gesture_type |= 0x04;
			break;
		default:
			NVT_LOG("unsupport gesture mode type\n");
			break;
	}

	nvt_gesture_state_switch();
	NVT_LOG("sys_gesture_type=%d, should_enable_gesture=%d\n", ts->sys_gesture_type, ts->should_enable_gesture);

	return err;
}

/*
 * gesture type debug value used for nvt driver to check current gesture mode
 */
static ssize_t gesture_type_dbg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", ts->sys_gesture_type);
}

static ssize_t gesture_type_dbg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	int err = 0;

	NVT_LOG("enter\n");
	err = sscanf(buf, "%d", &value);
	if (err < 0) {
		NVT_LOG("Failed to convert value\n");
		return -EINVAL;
	}

	err = count;
	value &= SYS_GESTURE_TYPE_MASK;
	NVT_LOG("value=%d\n", value);
	switch (value) {
		case SYS_GESTURE_TYPE_SINGLE_ONLY:
		case SYS_GESTURE_TYPE_DOUBLE_ONLY:
		case SYS_GESTURE_TYPE_SINGLE_DOUBLE:
			ts->sys_gesture_type = value;
			NVT_LOG("sys_gesture_type: %d\n", ts->sys_gesture_type);
			break;
		default:
			//disable gesture
			ts->sys_gesture_type = SYS_GESTURE_TYPE_DISABLE;
			NVT_LOG("unsupport gesture mode type\n");
			break;
	}

	nvt_gesture_state_switch();
	NVT_LOG("sys_gesture_type=%d, should_enable_gesture=%d\n", ts->sys_gesture_type, ts->should_enable_gesture);

	return err;
}
#endif

static struct device_attribute touchscreen_attributes[] = {
	__ATTR_RO(path),
	__ATTR_RO(vendor),
	__ATTR_RO(ic_ver),
#ifdef NVT_TOUCH_LAST_TIME
	__ATTR_RO(timestamp),
#endif
#ifdef PALM_GESTURE
	__ATTR(palm_settings, S_IRUGO | S_IWUSR | S_IWGRP, nvt_palm_settings_show, nvt_palm_settings_store),
#endif
#ifdef NVT_DOUBLE_TAP_CTRL
	__ATTR(gesture, S_IRUGO | S_IWUSR | S_IWGRP, gesture_show, gesture_store),
	__ATTR(gesture_type_dbg, S_IRUGO | S_IWUSR | S_IWGRP, gesture_type_dbg_show, gesture_type_dbg_store),
#endif
	__ATTR_NULL
};

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

/*******************************************************
Description:
	Novatek touchscreen FW function class. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_fw_class_init(bool create)
{
	struct device_attribute *attrs = touchscreen_attributes;
	int i, error = 0;
	s32 ret = 0;
	static struct class *touchscreen_class;
	static struct device *ts_class_dev;
	dev_t devno;
	static int minor;

	if (create) {
#if defined(PALM_GESTURE) || defined(NVT_DOUBLE_TAP_CTRL) || defined(NVT_TOUCH_LAST_TIME)
		ret = alloc_chrdev_region(&devno, 0, 1, NVT_PRIMARY_NAME);
#else
		ret = alloc_chrdev_region(&devno, 0, 1, NVT_SPI_NAME);
#endif

		if (ret) {
			NVT_ERR("cant`t allocate chrdev\n");
			return ret;
		}

		touchscreen_class = class_create(THIS_MODULE, "touchscreen");
		if (IS_ERR(touchscreen_class)) {
			error = PTR_ERR(touchscreen_class);
			touchscreen_class = NULL;
			return error;
		}

		ts_class_dev = device_create(touchscreen_class, NULL,
				devno,
#if defined(PALM_GESTURE) || defined(NVT_DOUBLE_TAP_CTRL) || defined(NVT_TOUCH_LAST_TIME)
				ts, NVT_PRIMARY_NAME);
#else
				ts, NVT_SPI_NAME);
#endif
		if (IS_ERR(ts_class_dev)) {
			error = PTR_ERR(ts_class_dev);
			ts_class_dev = NULL;
			return error;
		}

		for (i = 0; attrs[i].attr.name != NULL; ++i) {
			error = device_create_file(ts_class_dev, &attrs[i]);
			if (error)
				break;
		}

		if (error)
			goto device_destroy;
		else
#if defined(PALM_GESTURE) || defined(NVT_DOUBLE_TAP_CTRL) || defined(NVT_TOUCH_LAST_TIME)
			NVT_LOG("create /sys/class/touchscreen/%s Succeeded!\n", NVT_PRIMARY_NAME);
#else
			NVT_LOG("create /sys/class/touchscreen/%s Succeeded!\n", NVT_SPI_NAME);
#endif
	} else {
		if (!touchscreen_class || !ts_class_dev)
			return -ENODEV;

		for (i = 0; attrs[i].attr.name != NULL; ++i)
			device_remove_file(ts_class_dev, &attrs[i]);

		device_unregister(ts_class_dev);
		class_unregister(touchscreen_class);
	}

	return ret;

device_destroy:
	for (--i; i >= 0; --i)
		device_remove_file(ts_class_dev, &attrs[i]);
	device_destroy(touchscreen_class, MKDEV(INPUT_MAJOR, minor));
	ts_class_dev = NULL;
	class_unregister(touchscreen_class);
	NVT_ERR("error creating touchscreen class\n");

	return -ENODEV;
}

#ifdef NOVATECH_PEN_NOTIFIER
static int pen_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
    int ret = 0;
    NVT_LOG("Received event(%lu) for pen detection\n", event);

    if (event == PEN_DETECTION_INSERT)
        ts->nvt_pen_detect_flag = PEN_DETECTION_INSERT;
    else if (event == PEN_DETECTION_PULL)
        ts->nvt_pen_detect_flag = PEN_DETECTION_PULL;

    if (!ts->bTouchIsAwake || !ts->fw_ready_flag) {
        NVT_LOG("touch in suspend or no firmware, so store.");
    } else {
        ret = nvt_mcu_pen_detect_set(ts->nvt_pen_detect_flag);
        if (ret < 0) {
            NVT_ERR("write pen state fail");
        }
    }

    return 0;
}
#endif

#ifdef NVT_MTK_CHECK_PANEL
static void nvt_get_active_panel(void)
{
	int rc;
	struct device_node *chosen = of_find_node_by_name(NULL, "chosen");

	if(chosen) {
		rc = of_property_read_string(chosen, "mmi,panel_name", (const char **)&active_panel_name);
		if (rc)
			NVT_LOG("mmi,panel_name null\n");
		else
			NVT_INFO("active_panel_name=%s\n", active_panel_name);
	}
	else
		NVT_LOG("chosen node null\n");

}

static int nvt_check_panel(void)
{
	NVT_INFO("enter");
	nvt_get_active_panel();

	if (!active_panel_name)
		NVT_LOG("active_panel NULL\n");
	else if(strstr(active_panel_name, "nt366") || strstr(active_panel_name, "nt365") || strstr(active_panel_name, "nt377"))
	{
		NVT_LOG("matched active_panel:%s ", active_panel_name);
		return 0;
	}
	else
		NVT_LOG("not macthed active_panel:%s\n", active_panel_name);

	return -1;
}

#elif defined(NVT_MTK_GET_PANEL)
char active_panel_name[50] = {0};

static int nvt_get_panel(void)
{
	if (strlen(active_panel_name)) {
		NVT_LOG("got active_panel_name=%s\n", active_panel_name);
		return 0;
	}

	//bringup, parse panel name from cmdline
	NVT_LOG("enter\n");
	if (saved_command_line) {
		char *sub;
		char key_prefix[] = "mipi_mot_vid_";
		char ic_prefix[] = "nt36";
		NVT_LOG("saved_command_line is %s\n", saved_command_line);
		sub = strstr(saved_command_line, key_prefix);
		if (sub) {
			char *d;
			int n, len, len_max = 50;

			d = strstr(sub, " ");
			if (d) {
				n = strlen(sub) - strlen(d);
			} else {
				n = strlen(sub);
			}

			if (n > len_max)
				len = len_max;
			else
				len = n;

			strncpy(active_panel_name, sub, len);

			if(strstr(active_panel_name, ic_prefix)) {
				NVT_LOG("active_panel_name=%s\n", active_panel_name);
			} else {
				NVT_LOG("Not novatek panel!\n");
				return -1;
			}

		} else {
			NVT_LOG("active panel not found!\n");
			return -1;
		}
	} else {
		NVT_LOG("saved_command_line null!\n");
		return -1;
	}

	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
	void **retval = NULL;
#endif

#ifdef NVT_SENSOR_EN
	static bool initialized_sensor;
#endif

#ifdef NVT_MTK_CHECK_PANEL
	ret = nvt_check_panel();
	if (ret) {
		NVT_LOG("MTK check panel error\n");
		return ret;
	}
#elif defined(NVT_MTK_GET_PANEL)
	ret = nvt_get_panel();
	if (ret) {
		NVT_LOG("MTK get panel error\n");
		return ret;
	}
#endif

#ifdef NVT_CHECK_DEVICE_BOOTMODE
	ret = nvt_skip_charger_bootmode();
	if (ret) {
		NVT_ERR("charger bootmode, skip. ret=%d\n", ret);
		return ret;
	}
#else
	NVT_LOG("bootmode check not set\n");
#endif

	NVT_LOG("start\n");

	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc((NVT_TRANSFER_LEN+2), GFP_KERNEL);
	if(ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->rbuf = (uint8_t *)kzalloc(NVT_READ_LEN, GFP_KERNEL);
	if(ts->rbuf == NULL) {
		NVT_ERR("kzalloc for rbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_rbuf;
	}

	ts->client = client;
	spi_set_drvdata(client, ts);

	//---prepare for spi parameter---
	if (ts->client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;
	ts->client->chip_select = 0;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

#ifdef CONFIG_MTK_SPI
    /* old usage of MTK spi API */
    memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
    ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

#ifdef CONFIG_SPI_MT65XX
    /* new usage of MTK spi API */
    memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mtk_chip_config));
    ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

#ifdef CONFIG_NVT_MTK_SPI
    /*cancun MTK spi API */
    memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mtk_chip_config));
    ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

#ifdef NVT_SENSOR_EN
	mutex_init(&ts->state_mutex);
	//unknown screen state
	ts->screen_state = SCREEN_UNKNOWN;
#endif

	//---eng reset before TP_RESX high
	nvt_eng_reset();

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_ADDR);
	if (ret) {
		NVT_LOG("try to check from old chip ver trim address\n");
		ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_OLD_ADDR);
		if (ret) {
			NVT_ERR("chip is not identified\n");
			ret = -EINVAL;
			goto err_chipvertrim_failed;
		}
	}

	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
#ifdef PALM_GESTURE_RANGE
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TOUCH_DEFAULT_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0, TOUCH_DEFAULT_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, TOUCH_ORIENTATION_MIN, TOUCH_ORIENTATION_MAX, 0, 0);
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

#ifdef PALM_GESTURE
	ts->palm_enabled = false;
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			nvt_irq_enable(false);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif
#ifdef NVT_SENSOR_EN
	if (!initialized_sensor) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_init(&gesture_wakelock, WAKE_LOCK_SUSPEND, "dt-wake-lock");
#else
		PM_WAKEUP_REGISTER(&client->dev, gesture_wakelock, "dt-wake-lock");
		if (!gesture_wakelock) {
			NVT_ERR("failed to allocate wakeup source\n");
			goto err_wakeup_source_register_failed;
		}
#endif
		if (!nvt_sensor_init(ts))
			initialized_sensor = true;
	}
#endif

	if (ts->charger_detection_enable) {
		ts->charger_detection = kzalloc(sizeof(struct usb_charger_detection), GFP_KERNEL);
		if (ts->charger_detection == NULL) {
			NVT_ERR("failed to allocated memory for usb_charger_detection\n");
			goto err_charger_detection_alloc_failed;
		}
	}

	if (ts->charger_detection) {
		struct power_supply *psy = NULL;

		NVT_LOG("charger_detection on");
		ts->charger_detection->usb_connected = 0;
		ts->charger_detection->nvt_charger_notify_wq = create_singlethread_workqueue("nvt_charger_wq");
		if (!ts->charger_detection->nvt_charger_notify_wq) {
			NVT_ERR("allocate nvt_charger_notify_wq failed\n");
			goto err_charger_notify_wq_failed;
		}
		INIT_WORK(&ts->charger_detection->charger_notify_work, nvt_charger_notify_work);

		ts->charger_detection->charger_notif.notifier_call = charger_notifier_callback;
		ret = power_supply_reg_notifier(&ts->charger_detection->charger_notif);
		if (ret) {
			NVT_ERR("Unable to register charger_notifier:%d\n", ret);
			goto err_register_charger_notify_failed;
		}

		/* if power supply supplier registered brfore TP
		 * ps_notify_callback will not receive PSY_EVENT_PROP_ADDED
		 * event, and will cause miss to set TP into charger state.
		 * So check PS state in probe.
		 */
		if (ts->psy_name) {
			psy = power_supply_get_by_name(ts->psy_name);
			ts->charger_detection->psy_name = ts->psy_name;
			NVT_LOG("ts->charger_detection->psy_name=%s", ts->charger_detection->psy_name);
		}
		else
			psy = power_supply_get_by_name("usb");

		if (psy) {
			ret = charger_set_state(psy, ts->charger_detection);
			if (!ret)
				nvt_set_charger(ts->charger_detection->usb_connected);
			else {
				//release charger_detection instead of goto err_register_charger_notify_failed
				//to NOT interrupt NVT probe process
				//goto err_register_charger_notify_failed;
				if (ts->charger_detection) {
					if (ts->charger_detection->charger_notif.notifier_call)
						power_supply_unreg_notifier(&ts->charger_detection->charger_notif);

					destroy_workqueue(ts->charger_detection->nvt_charger_notify_wq);
					ts->charger_detection->nvt_charger_notify_wq = NULL;
					kfree(ts->charger_detection);
				}
			}
		}
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
	ts->fw_delay_que = 1;
#endif
#ifdef LCM_FAST_LIGHTUP
	INIT_WORK(&ts_resume_work, nova_resume_work_func);
#endif
	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_flash_proc_init_failed;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_extra_proc_init_failed;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_mp_proc_init_failed;
	}
#endif

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
	ts->disp_notifier.notifier_call = nvt_disp_notifier_callback;
	ret = mtk_disp_notifier_register("NVT Touch", &ts->disp_notifier);
	if (ret) {
		NVT_ERR("Failed to register disp notifier client:%d", ret);
		goto err_register_disp_notif_failed;
	}
#endif

#ifdef NOVATECH_PEN_NOTIFIER
	ts->fw_ready_flag = false;
	ts->nvt_pen_detect_flag = PEN_DETECTION_INSERT;
	ts->pen_notif.notifier_call = pen_notifier_callback;
	ret = pen_detection_register_client(&ts->pen_notif);
	if (ret) {
		NVT_ERR("[PEN]Unable to register pen_notifier: %d\n", ret);
		goto err_register_pen_notif_failed;
    }
#endif

	ret = nvt_fw_class_init(true);
	if (ret) {
		NVT_ERR("Create touchscreen class failed. ret=%d\n", ret);
		goto err_create_touchscreen_class_failed;
	}

	ts->bTouchIsAwake = 1;
	NVT_LOG("end\n");

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
	pr_info("%s, disp notifier register func!\n", __func__);
	if (mtk_panel_tch_handle_init()) {
		retval = mtk_panel_tch_handle_init();
		*retval = (void *)nvt_tp_power_on_reinit;
	}
#endif

	nvt_irq_enable(true);

	return 0;

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
err_register_disp_notif_failed:
	if (mtk_disp_notifier_unregister(&ts->disp_notifier))
		NVT_ERR("Error occurred while unregistering disp_notifier.\n");
#endif
err_create_touchscreen_class_failed:
#ifdef NOVATECH_PEN_NOTIFIER
    if (pen_detection_unregister_client(&ts->pen_notif))
        NVT_ERR("Error occurred while unregistering pen_notifier.\n");
err_register_pen_notif_failed:
#endif
#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
err_mp_proc_init_failed:
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
err_extra_proc_init_failed:
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
err_flash_proc_init_failed:
#endif
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
err_create_nvt_esd_check_wq_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		NVT_LOG("kthread_destroy_worker err_create\n");
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
err_register_charger_notify_failed:
	if (ts->charger_detection) {
		if (ts->charger_detection->charger_notif.notifier_call)
			power_supply_unreg_notifier(&ts->charger_detection->charger_notif);
		destroy_workqueue(ts->charger_detection->nvt_charger_notify_wq);
		ts->charger_detection->nvt_charger_notify_wq = NULL;
		kfree(ts->charger_detection);
	}
err_charger_detection_alloc_failed:
err_charger_notify_wq_failed:
	free_irq(client->irq, ts);
#ifdef NVT_SENSOR_EN
#ifndef CONFIG_HAS_WAKELOCK
err_wakeup_source_register_failed:
#endif
#endif
err_int_request_failed:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
err_chipvertrim_failed:
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	if (ts->rbuf) {
		kfree(ts->rbuf);
		ts->rbuf = NULL;
	}
err_malloc_rbuf:
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct spi_device *client)
{
	NVT_LOG("Removing driver...\n");

	nvt_fw_class_init(false);

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
	if (mtk_disp_notifier_unregister(&ts->disp_notifier))
		NVT_ERR("Error occurred while unregistering disp_notifier.\n");
#endif

#ifdef NOVATECH_PEN_NOTIFIER
    if (pen_detection_unregister_client(&ts->pen_notif))
        NVT_ERR("Error occurred while unregistering pen_notifier.\n");
#endif

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
	if (ts->charger_detection) {
		if (ts->charger_detection->charger_notif.notifier_call)
			power_supply_unreg_notifier(&ts->charger_detection->charger_notif);
		destroy_workqueue(ts->charger_detection->nvt_charger_notify_wq);
		kfree(ts->charger_detection);
	}
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif

#ifdef NVT_SENSOR_EN
	touch_set_state(TOUCH_DEEP_SLEEP_STATE, TOUCH_PANEL_IDX_PRIMARY);
	nvt_sensor_remove(ts);
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	spi_set_drvdata(client, NULL);

	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return 0;
}

static void nvt_ts_shutdown(struct spi_device *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
	if (mtk_disp_notifier_unregister(&ts->disp_notifier))
		NVT_ERR("Error occurred while unregistering disp_notifier.\n");
#endif

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!ts->bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

#ifdef NVT_DOUBLE_TAP_CTRL
	nvt_gesture_type_store(ts->sys_gesture_type);
#endif

#ifdef NVT_SENSOR_EN
	mutex_lock(&ts->state_mutex);
#endif

#if !WAKEUP_GESTURE
	nvt_irq_enable(false);
#else
#ifdef NVT_SENSOR_EN
	if (!ts->should_enable_gesture)
		nvt_irq_enable(false);
#endif
#endif

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("cancel delayed work sync\n");
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	ts->bTouchIsAwake = 0;

#if WAKEUP_GESTURE
#ifdef NVT_SENSOR_EN
	if (ts->should_enable_gesture) {
		NVT_LOG("set tap touch state:%d\n", TOUCH_LOW_POWER_STATE);
		touch_set_state(TOUCH_LOW_POWER_STATE, TOUCH_PANEL_IDX_PRIMARY);
#endif
		//---write command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);

		enable_irq_wake(ts->client->irq);
		ts->gesture_enabled = true;
		ts->wakeable = true;

		NVT_LOG("Enabled touch wakeup gesture\n");
#ifdef NVT_SENSOR_EN
	} else {
		//---write command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
		ts->gesture_enabled = false;
		ts->wakeable = false;
		NVT_LOG("enter deep sleep state\n");
		touch_set_state(TOUCH_DEEP_SLEEP_STATE, TOUCH_PANEL_IDX_PRIMARY);
	}
#endif
#else // WAKEUP_GESTURE
	//---write command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_SPI_WRITE(ts->client, buf, 2);
#endif // WAKEUP_GESTURE

	mutex_unlock(&ts->lock);

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
#ifdef PALM_GESTURE_RANGE
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, 0);
#endif
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq && ts->fw_delay_que) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		ts->fw_delay_que = 0;
		NVT_LOG("cancel_delayed_work_sync nvt_fwu_work\n");
	}
#endif
	msleep(50);

	NVT_LOG("end, bTouchIsAwake=%d\n", ts->bTouchIsAwake);
#ifdef NVT_SENSOR_EN
	ts->screen_state = SCREEN_OFF;
	mutex_unlock(&ts->state_mutex);
#endif

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{

#ifdef NVT_SENSOR_EN
	mutex_lock(&ts->state_mutex);
#endif
	if (ts->bTouchIsAwake) {
#ifdef NVT_SENSOR_EN
		mutex_unlock(&ts->state_mutex);
#endif
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

#if !WAKEUP_GESTURE
	nvt_irq_enable(true);
#else
#ifdef NVT_SENSOR_EN
	if (!ts->gesture_enabled)
		nvt_irq_enable(true);
#endif
#endif

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (ts->charger_detection) {
		queue_work(ts->charger_detection->nvt_charger_notify_wq, &ts->charger_detection->charger_notify_work);
	}

#if WAKEUP_GESTURE
#ifdef NVT_SENSOR_EN
	if (ts->wakeable) {
		disable_irq_wake(ts->client->irq);
		ts->gesture_enabled = false;
		ts->wakeable = false;
	}
#endif
#endif

	ts->bTouchIsAwake = 1;
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(0));

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

#ifdef NVT_SENSOR_EN
	ts->screen_state = SCREEN_ON;
	mutex_unlock(&ts->state_mutex);
#endif
	return 0;
}

#if defined(CFG_MTK_PANEL_NOTIFIER) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
/**
 * nvt_ts_disp_notifier_callback - mtk display notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
static int nvt_disp_notifier_callback(struct notifier_block *nb,
	unsigned long value, void *v)
{
	struct nvt_ts_data *ts = container_of(nb, struct nvt_ts_data, disp_notifier);
	int *data = (int *)v;

	if (ts && v) {
		NVT_LOG("%s IN", __func__);
#ifdef NVT_LCM_ESD_NOTIFIER
		if ((value == MTK_DISP_EARLY_EVENT_BLANK)||(value == MTK_ERALY_NOTIFY_TP_EVENT_BLANK)) {
			if ((*data == MTK_DISP_BLANK_POWERDOWN)||(*data == MTK_DISP_EARLY_BLANK_POWERDOWN)) {
				nvt_ts_suspend(&ts->client->dev);
			}
		} else if ((value == MTK_DISP_EVENT_BLANK)||(value == MTK_NOTIFY_TP_EVENT_BLANK)) {
			if ((*data == MTK_DISP_BLANK_UNBLANK)||(*data == MTK_DISP_EARLY_BLANK_UNBLANK)) {
				nvt_ts_resume(&ts->client->dev);
			}
		}
#else
		if (value == MTK_DISP_EARLY_EVENT_BLANK) {
                        if (*data == MTK_DISP_BLANK_POWERDOWN) {
                                nvt_ts_suspend(&ts->client->dev);
                        }
                } else if (value == MTK_DISP_EVENT_BLANK) {
                        NVT_LOG("%d",*data);
                        if (*data == MTK_DISP_BLANK_UNBLANK) {
                                nvt_ts_resume(&ts->client->dev);
                        }
                }

#endif
		NVT_LOG("%s OUT", __func__);
	} else {
		NVT_LOG("NT36672 touch IC can not suspend or resume");
		return -1;
	}

	return 0;
}

#endif

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};

static int nvt_set_charger(uint8_t charger_on_off)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;

	NVT_LOG("set charger: %d\n", charger_on_off);

	msleep(20);
	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		goto nvt_set_charger_out;
	}

	if (charger_on_off == USB_DETECT_IN) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = CMD_CHARGER_ON;
		ret = CTP_SPI_WRITE(ts->client, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write set charger command fail!\n");
			goto nvt_set_charger_out;
		} else {
			NVT_LOG("set charger on cmd succeeded\n");
		}
	} else if (charger_on_off == USB_DETECT_OUT) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = CMD_CHARGER_OFF;
		ret = CTP_SPI_WRITE(ts->client, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write set charger command fail!\n");
			goto nvt_set_charger_out;
		} else {
			NVT_LOG("set charger off cmd succeeded\n");
		}
	} else {
		NVT_ERR("Invalid charger parameter!\n");
		ret = -EINVAL;
	}

    nvt_set_charger_out:

	return ret;
}

static int charger_set_state(struct power_supply *psy, struct usb_charger_detection *chg_detect) {
	int ret;
	union power_supply_propval prop = {0};

	if (!psy) {
		NVT_ERR("psy null\n");
		return -EINVAL;
	}

	if (!chg_detect) {
		NVT_ERR("chg_detect null\n");
		return -EINVAL;
	}

	if (ts->usb_psp_online) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &prop);
		if (ret < 0)
			NVT_ERR("Couldn't get POWER_SUPPLY_PROP_ONLINE rc=%d\n", ret);
	} else if (!strcmp(psy->desc->name, "battery")) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &prop);
		if (ret < 0) {
			NVT_ERR("Couldn't get POWER_SUPPLY_PROP_STATUS rc=%d\n", ret);
		}
	} else {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &prop);
		if (ret < 0)
			NVT_ERR("Couldn't get POWER_SUPPLY_PROP_PRESENT rc=%d\n", ret);
	}

	if (!ret) {
		//NVT_LOG("saved usb_detect_flag=%d, prop.intval=%d", usb_detect_flag, prop.intval);
		if ((USB_DETECT_IN == prop.intval) || (USB_DETECT_OUT == prop.intval)) {
			usb_detect_flag = prop.intval;
			if (usb_detect_flag != chg_detect->usb_connected) {
				chg_detect->usb_connected = usb_detect_flag;
				//NVT_LOG("charger state updated: %d", usb_detect_flag);
			}
			else {
				//NVT_LOG("charger state not changed");
				ret = -1;
			}
		} else {
			//NVT_LOG("unsupported prop.intval");
			ret = -EINVAL;
		}
	}

	return ret;
}

static void nvt_charger_notify_work(struct work_struct *work)
{
	if (NULL == work) {
		NVT_ERR("%s:  parameter work are null!\n", __func__);
		return;
	}
	NVT_LOG("enter\n");
	if (USB_DETECT_IN == usb_detect_flag) {
		if (mutex_lock_interruptible(&ts->lock)) {
			NVT_ERR("Failed to lock in mutex_lock_interruptible(&ts->lock).\n");
			return;
		}
		nvt_set_charger(USB_DETECT_IN);
		mutex_unlock(&ts->lock);
	} else if (USB_DETECT_OUT == usb_detect_flag) {
		if (mutex_lock_interruptible(&ts->lock)) {
			NVT_ERR("Failed to lock in mutex_lock_interruptible(&ts->lock).\n");
			return;
		}
		nvt_set_charger(USB_DETECT_OUT);
		mutex_unlock(&ts->lock);
	}else{
		NVT_LOG("Charger flag:%d not currently required!\n",usb_detect_flag);
	}
}

static int charger_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *cb_v)
{
	int ret = 0;
	struct power_supply *psy = cb_v;
	struct usb_charger_detection *charger_detection =
			container_of(nb, struct usb_charger_detection, charger_notif);

	if (!charger_detection) {
		NVT_LOG("charger_detection null\n");
		return -EINVAL;
	}

	if (!psy || !psy->desc || !psy->desc->name) {
		NVT_ERR("psy null\n");
		return -EINVAL;
	}

	//check psy name
	if (charger_detection->psy_name) {
		if (strncmp(psy->desc->name, charger_detection->psy_name, sizeof(&charger_detection->psy_name))) {
			//NVT_LOG("unmatched charger psy_name:%s\n", psy->desc->name);
			return 0;
		}
	}
	else if (strncmp(psy->desc->name, "battery", sizeof("battery"))
		&& strncmp(psy->desc->name, "usb", sizeof("usb"))) {
		NVT_LOG("not charger psy_name:%s\n", psy->desc->name);
		return 0;
	}

	if (val == POWER_SUPPLY_PROP_STATUS) {
		ret = charger_set_state(psy, charger_detection);
		if (!ret && ts->bTouchIsAwake){
			 queue_work(charger_detection->nvt_charger_notify_wq,
					&charger_detection->charger_notify_work);
		}
	}
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

	//---add spi driver---
	ret = spi_register_driver(&nvt_spi_driver);
	if (ret) {
		NVT_ERR("failed to add spi driver");
		goto err_driver;
	}

	NVT_LOG("finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	spi_unregister_driver(&nvt_spi_driver);
}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
