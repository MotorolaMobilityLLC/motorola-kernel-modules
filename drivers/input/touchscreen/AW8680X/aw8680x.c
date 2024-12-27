// SPDX-License-Identifier: GPL-2.0
/*
 * @2022 awinic All Rights Reserved.
 * Description: aw8680x Qcom driver code.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include "aw_protocol_config.h"
#include "aw_protocol_map.h"
#include "aw_com_protocol.h"
#include "aw_protocol_fun.h"
#include "aw_soc_protocol_interface.h"
#include "aw8680x_bin_parse.h"
#include "aw8680x.h"

/*
 *
 * Marco
 *
 */
#define AW8680X_I2C_NAME "aw8680x_sensor"
#define AW8680X_NAME "ndt"
#define AW8680X_DRIVER_VERSION "v1.6.2.2"
#define AW8680X_I2C_RETRIES 3
#define AW8680X_BIN_INIT_DELAY 5000
#define AW8680X_ADB_BIN_INIT_DELAY 20

struct aw8680x *g_aw8680x;
/*
 * The name of the bin file that needs to be
 * obtained from /system/vendor/firmware
 */
static char *aw8680x_flash_app_bin = "aw8680x_flash_app.bin";
static char *aw8680x_flash_boot_bin = "aw8680x_flash_boot.bin";
static char *aw8680x_sram_bin = "aw8680x_sram.bin";
uint8_t use_ndt_aw8680x = 1;
EXPORT_SYMBOL_GPL(use_ndt_aw8680x);

static int32_t aw8680x_file_open(struct inode *inode, struct file *filp);
/*
 *
 * aw8680x i2c IO
 *
 */
static void aw8680x_wake_up(struct aw8680x *p_aw8680x)
{
	int32_t state_pin = 0;

	AWLOGI("enter, wake flag: %d", p_aw8680x->timer_wake_state);

	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	if ((p_aw8680x->timer_wake_state == true) &&
				(gpio_get_value_cansleep(p_aw8680x->state_gpio) == WAKE_STATUS)) {
		hrtimer_start(&p_aw8680x->hr_timer, ktime_set(AW_WAKE_TIME / 1000,
				(AW_WAKE_TIME % 1000) * 1000000), HRTIMER_MODE_REL);
	} else {
		state_pin = gpio_get_value_cansleep(p_aw8680x->state_gpio);
		if (state_pin != WAKE_STATUS) {
			usleep_range(650, 700);
			gpio_set_value_cansleep(p_aw8680x->wake_gpio, LOW_LEVEL);
		} else if (state_pin == WAKE_STATUS) {
			usleep_range(3000, 3200);
			state_pin = gpio_get_value_cansleep(p_aw8680x->state_gpio);
			if (state_pin == WAKE_STATUS) {
				gpio_set_value_cansleep(p_aw8680x->wake_gpio, LOW_LEVEL);
			} else {
				usleep_range(650, 700);
				gpio_set_value_cansleep(p_aw8680x->wake_gpio, LOW_LEVEL);
			}
		}
		usleep_range(1000, 1200);
		gpio_set_value_cansleep(p_aw8680x->wake_gpio, HIGH_LEVEL);
		hrtimer_start(&p_aw8680x->hr_timer, ktime_set(AW_WAKE_TIME / 1000,
				 (AW_WAKE_TIME % 1000) * 1000000), HRTIMER_MODE_REL);
		p_aw8680x->timer_wake_state = 1;
	}
	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));

	AWLOGI("exit, wake flag: %d", p_aw8680x->timer_wake_state);
}

static void aw8680x_wake_state_pin_judge(struct aw8680x *p_aw8680x)
{
	if ((p_aw8680x->wake_gpio_valid == true) && (p_aw8680x->state_pin_valid == true))
		aw8680x_wake_up(p_aw8680x);
	else
		AWLOGI("wake function is unsupport");

}

static unsigned char aw8680x_checksum(unsigned char *buf, int length)
{
	unsigned char checksum = 0;
	int i = 0;

	for (i = 0; i < length; i++)
		checksum += buf[i];

	AWLOGI("checksum = 0x%x", checksum);

	return checksum;
}
static int32_t aw8680x_soc_i2c_writes(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	struct i2c_msg msgs[] = {
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_WRITE_FLAG,
			.len = p_aw8680x->p_gui_data_s.soc_data_len,
			.buf = p_aw8680x->p_protocol_tx_data,
		},
	};
	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, ONE_MSG_NUM);
	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));
	if (ret < DATA_INIT)
		AWLOGE("soc i2c write error: %d", ret);

	return ret;
}

static int32_t aw8680x_soc_i2c_reads(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	struct i2c_msg msgs[] = {
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_READ_FLAG,
			.len = p_aw8680x->p_gui_data_s.ui_rd_data_len,
			.buf = p_aw8680x->p_protocol_rx_data,
		},
	};
	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, ONE_MSG_NUM);

	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));
	if (ret < DATA_INIT)
		AWLOGE("soc i2c read error: %d", ret);

	return ret;
}

static int32_t aw8680x_register_i2c_reads(struct aw8680x *p_aw8680x,
						uint8_t reg_addr, uint32_t len)
{
	int32_t ret = DATA_INIT;
	int32_t cnt = 0;
	struct i2c_msg msgs[2];

	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	msgs[0].addr = p_aw8680x->i2c->addr;
	msgs[0].flags = I2C_WRITE_FLAG;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;
	msgs[1].addr = p_aw8680x->i2c->addr;
	msgs[1].flags = I2C_READ_FLAG;
	msgs[1].len = len;
	msgs[1].buf = p_aw8680x->read_data;
	while (cnt < 3) {
		ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, TWO_MSG_NUM);
		if (ret < 0)
			AWLOGE("register i2c read error cnt: %d", cnt);
		else
			break;

		cnt++;
	}

	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));
	if (ret < DATA_INIT)
		AWLOGE("register i2c read error: %d", ret);

	return ret;
}

static int32_t aw8680x_register_i2c_writes(struct aw8680x *p_aw8680x,
				uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	int32_t ret = DATA_INIT;
	uint8_t *data = NULL;
	int32_t cnt = 0;

	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		AWLOGE("can not allocate memory");
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	while (cnt < 3) {
		ret = i2c_master_send(p_aw8680x->i2c, data, len + 1);
		if (ret < 0)
			AWLOGE("register i2c write error cnt: %d", cnt);
		else
			break;

		cnt++;
	}

	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));
	if (ret < DATA_INIT)
		AWLOGE("register i2c write error: %d", ret);
	kfree(data);

	return ret;
}

static int32_t aw8680x_register_i2c_writes_stay_boot(struct aw8680x *p_aw8680x,
				uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	int32_t ret = DATA_INIT;
	uint8_t *data = NULL;

	mutex_lock(&(p_aw8680x->aw8680x_i2c_mutex));
	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		AWLOGE("can not allocate memory");
		return -ENOMEM;
	}
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(p_aw8680x->i2c, data, len + 1);
	if (ret < DATA_INIT)
		AWLOGE("register i2c write error: %d", ret);
	mutex_unlock(&(p_aw8680x->aw8680x_i2c_mutex));

	kfree(data);
	return ret;
}

static int32_t aw8680x_pga_data_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_PGA_ADDR, AW_PGA_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read pga data, ret is : %d", ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < AW_PGA_DATA_LEN; i++) {
		AWLOGI("read data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_PGA_DATA_LEN - 2);
	if (data_checksum != p_aw8680x->read_data[AW_PGA_DATA_LEN - 1]) {
		AWLOGE("pga data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_tempera_data_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_TEMPERA_ADDR, AW_TEMPERA_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to tempera adc data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_TEMPERA_DATA_LEN; i++) {
		AWLOGI("tempera data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_TEMPERA_DATA_LEN - 2);
	AWLOGI("adc cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_TEMPERA_DATA_LEN - 1]) {
		AWLOGE("tempera data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_ADC_DR_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_ADC_DR_ADDR, AW_ADC_DR_LEN + 2);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read adc dr data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_ADC_DR_LEN + 2; i++) {
		AWLOGI("adc dr data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_ADC_DR_LEN);
	AWLOGI("adc dr cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_ADC_DR_LEN + 1]) {
		AWLOGE("adc dr data checksum err");
		return -CHECKSUM_ERR;
	}
	return AW_SUCCESS;
}

static int32_t aw8680x_DAC_DR_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DAC_DR_GET_ADDR, AW_DAC_DR_LEN + 2);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read dac dr data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_DAC_DR_LEN + 2; i++)
		AWLOGI("dac dr data[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_DAC_DR_LEN);
	AWLOGI("dac dr cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_DAC_DR_LEN + 1]) {
		AWLOGE("dac dr data checksum err");
		return -CHECKSUM_ERR;
	}
	return AW_SUCCESS;
}

static int32_t aw8680x_cali_set(struct aw8680x *p_aw8680x, uint8_t *cali_data)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	uint8_t data[3] = { 0x01, 0x01, 0x01 };

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_CALI_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read cali data, ret is : %d", ret);
		return -EIO;
	}
	mdelay(150);
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_CALI_ADDR, AW_CALI_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read cali data, ret is : %d", ret);
		return -EIO;
	}

	*cali_data = p_aw8680x->read_data[1];
	for (i = DATA_INIT; i < AW_CALI_DATA_LEN; i++) {
		AWLOGI("cali data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}
	if (*cali_data != p_aw8680x->read_data[AW_CALI_DATA_LEN - 1]) {
		AWLOGE("cali data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_sensor_status_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_SENSOR_STATUS_ADDR, AW_SENSOR_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read sensor data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_SENSOR_DATA_LEN; i++) {
		AWLOGI("sensor data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_SENSOR_DATA_LEN - 2);
	AWLOGI("sensor cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_SENSOR_DATA_LEN - 1]) {
		AWLOGE("sensor data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_adc_transfer_voltage_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_ADC_VOLTAGE_ADDR, AW_ADC_VOLTAGE_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read adc valtage data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_ADC_VOLTAGE_LEN; i++) {
		AWLOGI("adc voltage data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}
	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_ADC_VOLTAGE_LEN - 2);
	AWLOGI("adc voltage cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_ADC_VOLTAGE_LEN - 1]) {
		AWLOGE("adc voltage data checksum err");
		return -CHECKSUM_ERR;
	}
	return AW_SUCCESS;
}

static int32_t aw8680x_dac_transfer_voltage_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DAC_VOLTAGE_ADDR, AW_DAC_VOLTAGE_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read dac valtage data, ret is : %d", ret);
		return -EIO;
	}

	for (i = DATA_INIT; i < AW_DAC_VOLTAGE_LEN; i++) {
		AWLOGI("dac voltage data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}
	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_DAC_VOLTAGE_LEN - 2);
	AWLOGI("dac voltage cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_DAC_VOLTAGE_LEN - 1]) {
		AWLOGE("dac voltage data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static void aw8680x_DAC_DR_set(struct aw8680x *p_aw8680x, uint8_t *dac_dr_data)
{
	int32_t ret = DATA_INIT;
	uint8_t dr_data[AW_DAC_DR_LEN + 2] = { 0 };

	dr_data[0] = AW_DAC_DR_LEN;
	memcpy(&dr_data[1], dac_dr_data, AW_DAC_DR_LEN);
	dr_data[AW_DAC_DR_LEN + 1] = aw8680x_checksum(dac_dr_data, dr_data[0]);
	AWLOGI("dac voltage cheksum result = %d", dr_data[9]);
	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DAC_DR_SET_ADDR, dr_data,
						AW_DAC_DR_LEN + 2);
	if (ret < DATA_INIT)
		AWLOGE("failed to write dac dr data, ret is : %d", ret);
}

static void aw8680x_write_data_to_reg(struct aw8680x *p_aw8680x, uint8_t *write_data, uint8_t cmd)
{
	int32_t ret = DATA_INIT;
	unsigned char data_checksum = 0;
	uint8_t reg_data[11] = { 0 };
	uint8_t reg_len = 0;

	reg_data[0] = 0x9;
	reg_data[1] = cmd;
	if (cmd == 0x01) {
		memcpy(&reg_data[2], write_data, 8);
		data_checksum = aw8680x_checksum(write_data, 8);
		reg_data[10] = data_checksum;
		AWLOGI("cheksum result = %d", data_checksum);
		reg_len = 11;
	} else {
		memcpy(&reg_data[2], write_data, 4);
		reg_len = 6;
	}

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_WRITE_REG_ADDR, reg_data, reg_len);
	if (ret < DATA_INIT)
		AWLOGE("failed to write reg, ret is : %d", ret);
}

static int32_t aw8680x_read_data_from_reg(struct aw8680x *p_aw8680x, uint32_t *reg_data,
							uint32_t *reg_addr)
{
	int32_t ret = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_READ_REG_ADDR, 10);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read reg data, ret is : %d", ret);
		return -EIO;
	}

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], 8);
	AWLOGI("reg cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[9]) {
		AWLOGE("reg data checksum err");
		return -CHECKSUM_ERR;
	}

	memcpy(reg_addr, &p_aw8680x->read_data[1], sizeof(uint32_t));
	memcpy(reg_data, &p_aw8680x->read_data[5], sizeof(uint32_t));

	return AW_SUCCESS;
}

static int32_t aw8680x_get_adc_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, ADC_DATA_ADDR, ADC_DATA_LEN + 3);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read adc data, ret is : %d", ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < ADC_DATA_LEN + 3; i++)
		AWLOGI("adc data[%d] = 0x%x", i, p_aw8680x->read_data[i]);
	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[2], ADC_DATA_LEN);
	AWLOGI("adc cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[ADC_DATA_LEN + 2]) {
		AWLOGE("adc data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_get_force_event(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, FORCE_EVENT_ADDR, AW_CHANNEL_NUM + 2);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read force data, ret is : %d", ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < AW_CHANNEL_NUM + 2; i++)
		AWLOGI("force_event data[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1], AW_CHANNEL_NUM);
	AWLOGI("force cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_CHANNEL_NUM + 1]) {
		AWLOGE("force event checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_get_force_data(struct aw8680x *p_aw8680x, uint8_t *force_data)
{
	int32_t ret = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, FORCE_DATA_ADDR,
				AW_CHANNEL_NUM * sizeof(uint32_t) + 2);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read force data, ret is : %d", ret);
		return -EIO;
	}
	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[1],
				AW_CHANNEL_NUM * sizeof(uint32_t));
	AWLOGI("force data cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[AW_CHANNEL_NUM * sizeof(uint32_t) + 1]) {
		AWLOGE("force event checksum err");
		return -CHECKSUM_ERR;
	}
	memcpy(force_data, p_aw8680x->read_data, AW_CHANNEL_NUM * sizeof(uint32_t));

	return AW_SUCCESS;
}

static int32_t aw8680x_get_base_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, BASE_LINE_ADDR, BASELINE_LEN + 3);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read base data, ret is : %d", ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < BASELINE_LEN + 3; i++)
		AWLOGI("base data[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[2], BASELINE_LEN);
	AWLOGI("baseline cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[BASELINE_LEN + 2]) {
		AWLOGE("baseline data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_get_diff_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, DIFF_ADDR, DIFF_DATA_LEN + 3);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read diff data, ret is : %d",
									ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < DIFF_DATA_LEN + 3; i++)
		AWLOGI("diff data[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	data_checksum = aw8680x_checksum(&p_aw8680x->read_data[2], DIFF_DATA_LEN);
	AWLOGI("diff cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[DIFF_DATA_LEN + 2]) {
		AWLOGE("diff data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static int32_t aw8680x_get_diff_threshold(struct aw8680x *p_aw8680x, int16_t *diff_thre)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned char data_checksum = 0;

	ret = aw8680x_register_i2c_reads(p_aw8680x, DIFF_THRESHOLD_ADDR,
			DIFF_THRESHOLD_DATA_LEN + 1);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read diff thre, ret is : %d", ret);
		return -EIO;
	}
	data_checksum = aw8680x_checksum(p_aw8680x->read_data, DIFF_THRESHOLD_DATA_LEN);
	AWLOGI("diff cheksum result = %d", data_checksum);
	if (data_checksum != p_aw8680x->read_data[DIFF_THRESHOLD_DATA_LEN]) {
		AWLOGE("diff threshold checksum err");
		return -CHECKSUM_ERR;
	}
	memcpy(diff_thre, p_aw8680x->read_data, DIFF_THRESHOLD_DATA_LEN);
	for (i = 0; i < AW_CHANNEL_NUM; i++)
		AWLOGI("diff threshold[%d] = 0x%x", i, diff_thre[i]);

	return AW_SUCCESS;
}

static int32_t aw8680x_get_sensor_type(struct aw8680x *p_aw8680x, uint8_t *sensor_type)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, SENSOR_TYPE_ADDR, 3);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read diff thre, ret is : %d", ret);
		return -EIO;
	}

	if (p_aw8680x->read_data[1] != p_aw8680x->read_data[2]) {
		AWLOGE("sensor type checksum err");
		return -CHECKSUM_ERR;
	}
	*sensor_type = p_aw8680x->read_data[1];
	AWLOGI("sensor type = 0x%x", *sensor_type);

	return AW_SUCCESS;
}

static int32_t aw8680x_get_sw_algo_version(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, SW_ALGO_VERSION_ADDR, SW_ALGO_VERS_LEN);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read diff data, ret is : %d",
									ret);
		return -EIO;
	}
	for (i = DATA_INIT; i < SW_ALGO_VERS_LEN; i++)
		AWLOGI("sw_algo_version[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	return AW_SUCCESS;
}

static void aw8680x_enter_FTC_cali_mode(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	uint8_t i = 0;
	uint8_t data[4] = { 0x02, 0x12, 0xEE, 0x00};
	uint8_t check_data = 0;

	ret = aw8680x_register_i2c_writes(p_aw8680x, FTC_CALI_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to enter FTC mode, ret is : %d", ret);
		return;
	}

	msleep(20);
	ret = aw8680x_register_i2c_reads(p_aw8680x, FTC_CALI_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to enter FTC mode, ret is : %d", ret);
		return;
	}
	for (i = 0; i < sizeof(data); i++)
		AWLOGI("enter FTC check data = 0x%x",
			p_aw8680x->read_data[i]);

	check_data = aw8680x_checksum(&p_aw8680x->read_data[1], sizeof(data) - 2);
	if (check_data != p_aw8680x->read_data[sizeof(data) - 1])
		AWLOGE("enter FTC check data err");
}

static void aw8680x_exit_FTC_cali_mode(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	uint8_t i = 0;
	uint8_t data[4] = { 0x02, 0x13, 0xED, 0x00};
	uint8_t check_data = 0;

	ret = aw8680x_register_i2c_writes(p_aw8680x, FTC_CALI_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to exit FTC mode, ret is : %d", ret);
		return;
	}

	msleep(20);
	ret = aw8680x_register_i2c_reads(p_aw8680x, FTC_CALI_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to enter FTC mode, ret is : %d", ret);
		return;
	}
	for (i = 0; i < sizeof(data); i++)
		AWLOGI("enter FTC check data = 0x%x",
			p_aw8680x->read_data[i]);

	check_data = aw8680x_checksum(&p_aw8680x->read_data[1], sizeof(data) - 2);
	if (check_data != p_aw8680x->read_data[sizeof(data) - 1])
		AWLOGE("enter FTC check data err");
}

static int32_t aw8680x_FTC_noise_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int i = 0;
	uint8_t data[3] = { 0x01, 0x50, 0x50};
	uint8_t read_data_len = 0;
	uint8_t check_data = 0;

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DEBUG_CLEAR_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read noise, ret is : %d", ret);
		return -EIO;
	}
	mdelay(5000);
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DEBUG_CLEAR_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read noise, ret is : %d", ret);
		return -EIO;
	}
	if (p_aw8680x->read_data[0] != 0x01) {
		AWLOGE("failed to read noise, len is : %d",
			p_aw8680x->read_data[0]);
		return -CHECKSUM_ERR;
	}

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DEBUG_LEN_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read noise, ret is : %d", ret);
		return -EIO;
	}

	if (p_aw8680x->read_data[0] != 0x01) {
		AWLOGE("failed to read noise, len is : %d",
			p_aw8680x->read_data[0]);
		return -CHECKSUM_ERR;
	}

	read_data_len = p_aw8680x->read_data[1];
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_NOISE_ADDR, read_data_len + 1);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read noise, ret is : %d", ret);
		return -EIO;
	}
	check_data = aw8680x_checksum(p_aw8680x->read_data, read_data_len);
	AWLOGI("noise cheksum result = %d", check_data);
	if (check_data != p_aw8680x->read_data[read_data_len]) {
		AWLOGE("noise checksum err");
		return -CHECKSUM_ERR;
	}
	for (i = 0; i < read_data_len; i++)
		AWLOGI("FTC noise[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	return AW_SUCCESS;
}

static int32_t aw8680x_FTC_no_press_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int i = 0;
	uint8_t data[3] = { 0x01, 0x51, 0x51};
	uint8_t read_data_len = 0;
	uint8_t check_data = 0;

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DEBUG_CLEAR_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read no press data, ret is : %d", ret);
		return -EIO;
	}
	mdelay(5000);
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DEBUG_CLEAR_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read no press data, ret is : %d", ret);
		return -EIO;
	}
	if (p_aw8680x->read_data[0] != 0x01) {
		AWLOGE("failed to read no press data, len is : %d",
			p_aw8680x->read_data[0]);
		return -CHECKSUM_ERR;
	}

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DEBUG_LEN_ADDR, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read no press data, ret is : %d", ret);
		return -EIO;
	}

	if (p_aw8680x->read_data[0] != 0x01) {
		AWLOGE("failed to read no press data, len is : %d",
			p_aw8680x->read_data[0]);
		return -CHECKSUM_ERR;
	}

	read_data_len = p_aw8680x->read_data[1];
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_NOISE_ADDR, read_data_len + 1);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read no press data, ret is : %d", ret);
		return -EIO;
	}
	check_data = aw8680x_checksum(p_aw8680x->read_data, read_data_len);
	AWLOGI("no press data cheksum result = %d", check_data);
	if (check_data != p_aw8680x->read_data[read_data_len]) {
		AWLOGE("no press data checksum err");
		return -CHECKSUM_ERR;
	}
	for (i = 0; i < read_data_len; i++)
		AWLOGI("FTC no press data[%d] = 0x%x", i, p_aw8680x->read_data[i]);

	return AW_SUCCESS;
}

static void aw8680x_FTC_coef_write_to_flash(struct aw8680x *p_aw8680x, uint8_t *write_data)
{
	int32_t ret = DATA_INIT;
	uint8_t data_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	uint8_t data0[3] = { 0x01, 0x52, 0x52};
	uint8_t data1[3] = { 0x01, 0x00, 0x00};
	uint8_t flash_write_data[AW_CHANNEL_NUM * sizeof(uint32_t) + 2] = { 0 };

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DEBUG_CLEAR_ADDR, data0, sizeof(data0));
	if (ret < DATA_INIT) {
		AWLOGE("failed to write coef to flash, ret is : %d", ret);
		return;
	}
	flash_write_data[0] = data_len;
	flash_write_data[data_len + 1] = aw8680x_checksum(write_data, data_len);
	memcpy(&flash_write_data[1], write_data, data_len);
	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_WRITE_COEF_ADDR,
			 flash_write_data, data_len + 2);
	if (ret < DATA_INIT) {
		AWLOGE("failed to write coef to flash, ret is : %d", ret);
		return;
	}
	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_DEBUG_CLEAR_ADDR, sizeof(data1));
	if (ret < DATA_INIT) {
		AWLOGE("failed to write coef to flash, ret is : %d", ret);
		return;
	}
	if (p_aw8680x->read_data[0] != 0x01) {
		AWLOGE("failed to write coef to flash, ret is : %d", ret);
		return;
	}

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DEBUG_CLEAR_ADDR, data1, sizeof(data1));
	if (ret != DATA_INIT) {
		AWLOGE("failed to write coef to flash, ret is : %d", ret);
		return;
	}
}

static int32_t aw8680x_FTC_coef_read_from_flash(struct aw8680x *p_aw8680x, uint8_t *flash_data)
{
	int32_t ret = DATA_INIT;
	uint8_t data_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	uint8_t i = 0;
	uint8_t data[3] = { 0x01, 0x53, 0x53};
	uint8_t check_data = 0;

	ret = aw8680x_register_i2c_writes(p_aw8680x, AW_DEBUG_CLEAR_ADDR, data, sizeof(data));
	if (ret < DATA_INIT) {
		AWLOGE("failed to read coef to flash, ret is : %d", ret);
		return -EIO;
	}

	ret = aw8680x_register_i2c_reads(p_aw8680x, AW_READ_COEF_ADDR, data_len + 1);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read dac valtage data, ret is : %d", ret);
		return -EIO;
	}
	flash_data = p_aw8680x->read_data;
	for (i = DATA_INIT; i < data_len + 1; i++) {
		AWLOGI("flash data[%d] = 0x%x", i,
						p_aw8680x->read_data[i]);
	}
	check_data = aw8680x_checksum(p_aw8680x->read_data, data_len);
	AWLOGI("flash data cheksum result = %d", check_data);
	if (check_data != p_aw8680x->read_data[data_len]) {
		AWLOGE("flash data checksum err");
		return -CHECKSUM_ERR;
	}

	return AW_SUCCESS;
}

static void aw8680x_set_common_info(struct aw8680x *p_aw8680x,
						unsigned short soc_data_len,
						unsigned char module_id,
						unsigned char event_id)
{
	p_aw8680x->p_gui_data_s.soc_data_len = soc_data_len;
	p_aw8680x->p_gui_data_s.module_id = module_id; /* module id */
	p_aw8680x->p_gui_data_s.event_id = event_id; /* evnet id */
	p_aw8680x->module_id = module_id;
}

/* pack data and i2c_write data to mcu */
static int32_t aw8680x_send(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	/* Pack instruction data according to soc protocol */
	ret = aw_soc_protocol_pack_interface(&(p_aw8680x->p_gui_data_s),
						p_aw8680x->p_protocol_tx_data);
	if (ret != AW_SUCCESS) {
		AWLOGE("soc data update pack fail!");
		return ret;
	}
	/* Send the packaged data to the MCU through the i2c write interface */
	ret = aw8680x_soc_i2c_writes(p_aw8680x);
	if (ret < DATA_INIT)
		return ret;

	return AW_SUCCESS;
}

/* i2c_read ack_data from IC and unpack ack_data */
static int32_t aw8680x_ack(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	/* Read the information processed by the instruction
	 * through the i2c read interface
	 */
	ret = aw8680x_soc_i2c_reads(p_aw8680x);
	if (ret < DATA_INIT)
		return ret;
	/* Unpack instruction data according to soc protocol */
	ret = aw_soc_protocol_unpack_interface(&(p_aw8680x->p_gui_data_s),
						p_aw8680x->p_protocol_rx_data);
	if ((p_aw8680x->p_gui_data_s.module_id == p_aw8680x->module_id) &&
			(ret == DATA_INIT) &&
			(p_aw8680x->p_gui_data_s.err_flag == DATA_INIT) &&
			(p_aw8680x->p_protocol_rx_data[7] == 1))
		return AW_SUCCESS;

	return -ACK_ERR;
}


/* Get information in ack data area */
static void aw8680x_get_info(struct aw8680x *p_aw8680x)
{
	/* connect */
	if (p_aw8680x->p_gui_data_s.module_id == HANDSHAKE_ID) {
		if (p_aw8680x->p_gui_data_s.event_id == CONNECT_ACK_ID) {
			p_aw8680x->pc_location =
				p_aw8680x->p_gui_data_s.soc_data[0] |
				(p_aw8680x->p_gui_data_s.soc_data[1] << 8) |
				(p_aw8680x->p_gui_data_s.soc_data[2] << 16) |
				(p_aw8680x->p_gui_data_s.soc_data[3] << 24);
		}
	}
	/* flash read */
	if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_READ_ACK_ID))
		memcpy(p_aw8680x->read_data, p_aw8680x->p_gui_data_s.soc_data,
							p_aw8680x->read_len);
	if ((p_aw8680x->p_gui_data_s.module_id == RAM_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == RAM_READ_ACK_ID)) {
		memcpy(p_aw8680x->read_data, p_aw8680x->p_gui_data_s.soc_data,
							p_aw8680x->read_len);
		p_aw8680x->read_len = 0;
	}
}


static void aw8680x_send_delay(struct aw8680x *p_aw8680x)
{
	/* erase sector delay */
	if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ID)) {
		mdelay(300);
	/* write flash delay */
	} else if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_WRITE_ID)) {
		mdelay(2);
	/* write sram delay */
	} else if ((p_aw8680x->p_gui_data_s.module_id == RAM_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == RAM_WRITE_ID)) {
		mdelay(2);
	/* other delay */
	} else if ((p_aw8680x->p_gui_data_s.module_id == RAM_ID) &&
				(p_aw8680x->p_gui_data_s.event_id == RAM_READ_ID)) {
		udelay(500);
	} else {
		udelay(200);
	}
}

static void aw8680x_ack_delay(struct aw8680x *p_aw8680x)
{
	int32_t delay_count = DATA_INIT;

	/* jump delay */
	if (p_aw8680x->p_gui_data_s.module_id == END_ID)
		mdelay(JUMP_INIT_TIME);
	/* Flash option failure handling mechanism */
	while ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID)
		&& (p_aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ACK_ID)
		&& (p_aw8680x->ack_flag != 0)) {
		mdelay(5);
		delay_count += 1;
		p_aw8680x->ack_flag = aw8680x_ack(p_aw8680x);
		if (delay_count == 5)
			break;
	}
}

static int32_t aw8680x_soc_protocol_set(struct aw8680x *p_aw8680x,
				    unsigned int addr, unsigned short read_len)
{
	int32_t ret = DATA_INIT;

	/* Configure some command information */
	p_aw8680x->p_gui_data_s.addr = addr; /* Read or write addr */
	p_aw8680x->p_gui_data_s.read_len = read_len; /* Read addr data length */

	ret = aw8680x_send(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE("failed to send, ret is : %d", ret);
		return ret;
	}
	aw8680x_send_delay(p_aw8680x);
	/* Flash erase failure handling mechanism */
	p_aw8680x->ack_flag = aw8680x_ack(p_aw8680x);
	aw8680x_ack_delay(p_aw8680x);
	if (p_aw8680x->ack_flag != DATA_INIT) {
		AWLOGE("ack flag = %d", p_aw8680x->ack_flag);
		return p_aw8680x->ack_flag;
	}
	/* get ack information */
	aw8680x_get_info(p_aw8680x);

	return AW_SUCCESS;
}

static int32_t aw8680x_connect(struct aw8680x *p_aw8680x)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, HANDSHAKE_ID, CONNECT_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, SOC_ADDR, SOC_READ_LEN);
}

static int32_t aw8680x_ram_i2c_read(struct aw8680x *aw8680x, uint32_t read_addr,
					uint8_t *read_data, uint32_t read_len)
{
	int32_t ret = 0;

	aw8680x->read_len =  read_len;
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, RAM_ID, RAM_READ_ID);
	ret = aw8680x_soc_protocol_set(aw8680x, read_addr, read_len);
	if (ret != DATA_INIT) {
		AWLOGE("ram i2c read err");
		return ret;
	}
	memcpy(read_data, aw8680x->read_data, read_len);

	return ret;
}

static int32_t aw8680x_ram_i2c_write(struct aw8680x *p_aw8680x, uint32_t write_addr,
					uint8_t *write_data, uint32_t write_len)
{
	memcpy(p_aw8680x->p_gui_data_s.soc_data, write_data, write_len);
	aw8680x_set_common_info(p_aw8680x, write_len, RAM_ID, RAM_WRITE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, write_addr, SOC_READ_LEN);
}

static int32_t aw8680x_start_mode_fun(struct aw8680x *aw8680x, uint32_t flash_address,
					uint32_t isp_mode, uint8_t start_mode)
{
	int32_t ret = 0;
	uint32_t write_data = 0;
	uint8_t buf[4] = { 0 };

	if (start_mode == 0)
		write_data = AW_START_MODE_FUNC0;
	else
		write_data = AW_START_MODE_FUNC1;

	aw_set_u32_fun(buf, write_data);
	ret = aw8680x_ram_i2c_write(aw8680x, AW_REG_ISP_CR, buf, 4);
	if (ret != DATA_INIT)
		return ret;

	udelay(5);
	aw_set_u32_fun(buf, flash_address);
	ret = aw8680x_ram_i2c_write(aw8680x, AW_REG_ISP_ADR, buf, 4);
	if (ret != DATA_INIT)
		return ret;

	udelay(5);
	aw_set_u32_fun(buf, isp_mode);
	aw8680x_ram_i2c_write(aw8680x, AW_REG_ISP_CMD, buf, 4);
	if (ret != DATA_INIT)
		return ret;

	udelay(5);
	write_data = 0x00000001;
	aw_set_u32_fun(buf, write_data);
	aw8680x_ram_i2c_write(aw8680x, AW_REG_ISP_GO, buf, 4);
	if (ret != DATA_INIT)
		return ret;

	udelay(5);
	return ret;
}

static int32_t aw8680x_nvr_check_valid_value(uint8_t valid_value, uint8_t value_min, uint8_t value_max)
{
	if ((valid_value >= value_min) && (valid_value <= value_max))
		return AW_SUCCESS;

	return ERR_FLAG;
}

static int32_t aw8680x_start_for_uboot(struct aw8680x *aw8680x, uint8_t boot_mode)
{
	uint8_t i = 0;
	int8_t check_retry = 3;
	uint8_t reg_data[32] = { 0 };
	uint8_t reg_data1[32] = { 0 };
	int32_t ret = 0;

	if (boot_mode == 0) {
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R1, AW_START_MODE_FUNC2, 0);
		if (ret != DATA_INIT)
			return ret;

		for (i = 0; i < 8; i++) {
			ret = aw8680x_ram_i2c_read(aw8680x, AW_REG_ISP_RDAT0 + 4 * i,
					 &reg_data[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			AWLOGI("reg_data[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x", i,
						reg_data[4 * i], reg_data[4 * i + 1],
						reg_data[4 * i + 2], reg_data[4 * i + 3]);
		}
	} else {
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R1, AW_START_MODE_FUNC2, 1);
		if (ret != DATA_INIT)
			return ret;


		for (i = 0; i < 8; i++) {
			ret = aw8680x_ram_i2c_read(aw8680x, AW_REG_ISP_RDAT0 + 4 * i,
						 &reg_data[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			AWLOGI("reg_data[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x", i,
						reg_data[4 * i], reg_data[4 * i + 1],
						reg_data[4 * i + 2], reg_data[4 * i + 3]);
		}
	}

	if (((uint32_t *)reg_data)[0] != AW_NVR_JUDGE) {
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R0, AW_START_MODE_FUNC2, 1);
		if (ret != DATA_INIT)
			return ret;

		for (i = 0; i < 8; i++) {
			ret = aw8680x_ram_i2c_read(aw8680x, AW_REG_ISP_RDAT0 + 4 * i,
						 &reg_data[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			AWLOGI("reg_data[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x", i,
						reg_data[4 * i], reg_data[4 * i + 1],
						reg_data[4 * i + 2], reg_data[4 * i + 3]);
		}

		ret = aw8680x_nvr_check_valid_value(reg_data[8], AW_LOSC_MIN_DATA,
							AW_LOSC_MAX_DATA);
		if (ret == ERR_FLAG) /*LOSC*/
			reg_data[8] = AW_LOSC_DEFAULT_DATA;

		ret = aw8680x_nvr_check_valid_value(reg_data[9], AW_HOSC_MIN_DATA,
							AW_HOSC_MAX_DATA);
		if (ret == ERR_FLAG) /*HOSC*/
			reg_data[9] = AW_HOSC_DEFAULT_DATA;

		ret = aw8680x_nvr_check_valid_value(reg_data[11], AW_LDO_MIN_DATA,
							AW_LDO_MAX_DATA);
		if (ret == ERR_FLAG) {
			if (reg_data[11] != AW_LDO_DEFAULT_DATA) /*LDO*/
				reg_data[11] = AW_LDO_MAX_DATA;
		}
	}

	while (check_retry--) {
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R0, AW_START_MODE_FUNC4, 1);
		if (ret != DATA_INIT)
			return ret;

		mdelay(15);
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R0, AW_START_MODE_FUNC2, 1);
		if (ret != DATA_INIT)
			return ret;


		for (i = 0; i < 8; i++) {
			ret = aw8680x_ram_i2c_read(aw8680x, AW_REG_ISP_RDAT0 + 4 * i,
						&reg_data1[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			AWLOGI("reg_data1[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x", i,
						reg_data1[4 * i], reg_data1[4 * i + 1],
						reg_data1[4 * i + 2], reg_data1[4 * i + 3]);
		}

		reg_data[15] = boot_mode;
		for (i = 0; i < 8; i++) {
			AWLOGI("write1 : reg_data[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x",
						i, reg_data[4 * i], reg_data[4 * i + 1],
						reg_data[4 * i + 2], reg_data[4 * i + 3]);
			ret = aw8680x_ram_i2c_write(aw8680x, AW_REG_ISP_WDAT0 + 4 * i,
						&reg_data[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			udelay(5);
		}
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R0, AW_START_MODE_FUNC3, 1);
		if (ret != DATA_INIT)
			return ret;

		/* read chip data*/
		ret = aw8680x_start_mode_fun(aw8680x, AW_REG_SRAM_R0, AW_START_MODE_FUNC2, 1);
		if (ret != DATA_INIT)
			return ret;

		for (i = 0; i < 8; i++) {
			ret = aw8680x_ram_i2c_read(aw8680x, AW_REG_ISP_RDAT0 + 4 * i,
						&reg_data1[4 * i], 4);
			if (ret != DATA_INIT)
				return ret;

			AWLOGI("reg_data1[%d] = 0x%02x, 0x%02x, 0x%02x, 0x%02x", i,
						reg_data1[4 * i], reg_data1[4 * i + 1],
						reg_data1[4 * i + 2], reg_data1[4 * i + 3]);
		}

		if ((reg_data1[8] == reg_data[8]) && (reg_data1[9] == reg_data[9]) &&
						(reg_data1[11] == reg_data[11]))
			break;
	}
	return ret;
}

static int32_t aw8680x_flash_app_version_in_soc_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t retry = 3;

	while (retry--) {
		aw8680x_wake_state_pin_judge(p_aw8680x);
		ret = aw8680x_connect(p_aw8680x);
		if (ret == 0)
			break;
	}
	if (ret != AW_SUCCESS)
		AWLOGE("get flash app version failed, ret is :%d", ret);
	else
		p_aw8680x->flash_app_version_in_soc = p_aw8680x->pc_location;

	return ret;
}

static int32_t
aw8680x_jump_flash_app(struct aw8680x *p_aw8680x, uint32_t jump_addr)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, END_ID, FLASH_JUMP_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, jump_addr, SOC_READ_LEN);
}

static int32_t aw8680x_jump_sram(struct aw8680x *p_aw8680x, uint32_t jump_addr)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, END_ID, RAM_JUMP_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, jump_addr, SOC_READ_LEN);
}

static int32_t aw8680x_erase_sector(struct aw8680x *p_aw8680x, uint32_t addr,
							uint32_t sector_num)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, FLASH_ID, FLASH_ERASE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, addr, sector_num);
}

static void aw8680x_hw_reset(struct aw8680x *p_aw8680x)
{
	AWLOGI("enter");

	gpio_set_value_cansleep(p_aw8680x->reset_gpio, HIGH_LEVEL);
	udelay(150);
	gpio_set_value_cansleep(p_aw8680x->reset_gpio, LOW_LEVEL);
	mdelay(3);
}

static void aw8680x_stay_boot(struct aw8680x *p_aw8680x)
{
	uint8_t reg_val = AW8680X_STAY_UBOOT;

	aw8680x_register_i2c_writes_stay_boot(p_aw8680x, REG_STAY_UBOOT,
						&reg_val, REGVAL_LEN_ONE_byte);
	usleep_range(10000, 15000);
}

static int32_t aw8680x_jump_boot(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t connect_count = CONNECT_RETRY_TIME;

	while (connect_count--) {
		aw8680x_hw_reset(p_aw8680x);
		aw8680x_stay_boot(p_aw8680x);
		ret = aw8680x_connect(p_aw8680x);
		if (ret == AW_SUCCESS) {
			if (p_aw8680x->pc_location == PC_POINT_ROM_BOOT)
				break;
			if (p_aw8680x->pc_location == PC_POINT_FLASH_BOOT)
				break;
		}
	}
	if ((p_aw8680x->pc_location == PC_POINT_ROM_BOOT) ||
			(p_aw8680x->pc_location == PC_POINT_FLASH_BOOT)) {
		AWLOGI("jump boot success!");
		return AW_SUCCESS;
	}
	AWLOGE("jump boot fail! ret is %d", ret);

	return -ERR_FLAG;
}

static int32_t aw8680x_flash_cycle_write(struct aw8680x *p_aw8680x,
			int cycle_num, int effect_len, int32_t flash_flag)
{
	uint32_t valid_data_addr = DATA_INIT;
	uint32_t download_addr = DATA_INIT;
	uint32_t valid_addr_offset = DATA_INIT;
	uint32_t download_addr_offset = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		valid_data_addr =
		p_aw8680x->flash_boot_bin->header_info[0].valid_data_addr;
		download_addr =
			p_aw8680x->flash_boot_bin->header_info[0].download_addr;
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		valid_data_addr =
		p_aw8680x->flash_app_bin->header_info[0].valid_data_addr;
		download_addr =
			p_aw8680x->flash_app_bin->header_info[0].download_addr;
	}
	valid_addr_offset = valid_data_addr + cycle_num * WRITE_FLASH_MAX;
	download_addr_offset = download_addr + WRITE_FLASH_MAX * cycle_num;
	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		memcpy(p_aw8680x->p_gui_data_s.soc_data,
			&(p_aw8680x->flash_boot_bin->info.data[valid_addr_offset]),
			effect_len);
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		memcpy(p_aw8680x->p_gui_data_s.soc_data,
			&(p_aw8680x->flash_app_bin->info.data[valid_addr_offset]),
			effect_len);
	}
	aw8680x_set_common_info(p_aw8680x, effect_len, FLASH_ID, FLASH_WRITE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, download_addr_offset,
								SOC_READ_LEN);
}

static int32_t aw8680x_sram_cycle_write(struct aw8680x *p_aw8680x,
						int cycle_num, int effect_len)
{
	uint32_t valid_data_addr =
			p_aw8680x->sram_bin->header_info[0].valid_data_addr;
	uint32_t download_addr =
			p_aw8680x->sram_bin->header_info[0].download_addr;
	uint32_t valid_addr_offset =
				valid_data_addr + cycle_num * WRITE_SRAM_MAX;
	uint32_t download_addr_offset =
				download_addr + WRITE_SRAM_MAX * cycle_num;

	memcpy(p_aw8680x->p_gui_data_s.soc_data,
		&(p_aw8680x->sram_bin->info.data[valid_addr_offset]),
		effect_len);
	aw8680x_set_common_info(p_aw8680x, effect_len, RAM_ID, RAM_WRITE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, download_addr_offset,
								SOC_READ_LEN);
}

static int32_t
aw8680x_flash_write_bin_to_soc(struct aw8680x *p_aw8680x, uint8_t flash_flag)
{
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	int32_t flash_write_count = DATA_INIT;
	uint16_t flash_write_last = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
		valid_data_len =
		p_aw8680x->flash_boot_bin->header_info[0].valid_data_len;
	else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
		valid_data_len =
			p_aw8680x->flash_app_bin->header_info[0].valid_data_len;

	if (valid_data_len > WRITE_FLASH_MAX) {
		flash_write_count = valid_data_len / WRITE_FLASH_MAX;
		flash_write_last = valid_data_len % WRITE_FLASH_MAX;
	} else {
		flash_write_count = DATA_INIT;
	}

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		AWLOGI("flash boot bin flash write count = %d",
							flash_write_count);
		AWLOGI("flash boot bin flash write last = %d",
							flash_write_last);
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		AWLOGI("flash app bin flash write count = %d",
							flash_write_count);
		AWLOGI("flash app bin flash write last = %d",
							flash_write_last);
	}

	for (i = DATA_INIT; i < flash_write_count; i++) {
		ret = aw8680x_flash_cycle_write(p_aw8680x, i, WRITE_FLASH_MAX,
								flash_flag);
		if (ret != AW_SUCCESS) {
			if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
				AWLOGE("flash boot bin cycle write fail!");
			else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
				AWLOGE("flash app bin cycle write fail!");

			return ret;
		}
	}

	if (flash_write_last != AW_SUCCESS) {
		AWLOGE("flash last app write");
		ret = aw8680x_flash_cycle_write(p_aw8680x, flash_write_count,
						flash_write_last, flash_flag);
		if (ret != AW_SUCCESS) {
			if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
				AWLOGE("flash boot bin cycle write fail!");
			else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
				AWLOGE("flash app bin cycle write fail!");

			return ret;
		}
	}
	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
		AWLOGI("flash boot bin cycle write Successfully!");
	else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
		AWLOGI("flash app bin cycle write Successfully!");

	if (valid_data_len < WRITE_FLASH_MAX) {
		ret = aw8680x_flash_cycle_write(p_aw8680x, 0, valid_data_len,
								flash_flag);
		if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
			if (ret != AW_SUCCESS) {
				AWLOGE("flash boot bin cycle write fail!");
				return ret;
			}
			AWLOGI("flash boot bin cycle write Successfully!");
		} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
			if (ret != AW_SUCCESS) {
				AWLOGE("in flash app, flash cycle write fail!");
				return ret;
			}
			AWLOGI("flash app bin cycle write Successfully!");
		}
	}

	return AW_SUCCESS;

}

/*
 * function : erase flash before updating new flash firmware,
		erase flash 512 bytes every time.
 * $flash_flag :
 * AW8680X_FLASH_BOOT_UPDATE : erase flash boot, flash boot space is 4k,
 * AW8680X_FLASH_BPP_UPDATE : erase flash app.
 */
static int32_t
aw8680x_erase_flash_data(struct aw8680x *p_aw8680x, uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t erase_num = DATA_INIT;
	int32_t erase_count = 3;
	uint32_t erase_base_addr = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		erase_num = ERASE_FLASH_BOOT_SIZE;
		erase_base_addr = p_aw8680x->flash_boot_addr;
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		valid_data_len =
			p_aw8680x->flash_app_bin->header_info[0].valid_data_len;
		if (valid_data_len > ERASE_BYTE_MAX) {
			if ((valid_data_len % ERASE_BYTE_MAX) == DATA_INIT)
				erase_num = valid_data_len / ERASE_BYTE_MAX;
			else
				erase_num = valid_data_len / ERASE_BYTE_MAX + 1;
		} else {
			erase_num = 1;
		}
		erase_base_addr = p_aw8680x->flash_app_addr;
	}

	while (erase_count--) {
		AWLOGE("erase num = %d", erase_num);
		AWLOGE("erase_base_addr = %x", erase_base_addr);
		ret = aw8680x_erase_sector(p_aw8680x, erase_base_addr, erase_num);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE("erase flash app sector fail!");
		return ret;
	}

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
		AWLOGI("erase flash boot successfully!!!");
	else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
		AWLOGI("erase flash app successfully!!!");

	return AW_SUCCESS;
}

static int32_t aw8680x_flash_app_bin_update_to_soc(struct aw8680x *p_aw8680x,
							uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t update_count = 3;

	while (update_count--) {
		ret = aw8680x_erase_flash_data(p_aw8680x, flash_flag);
		if (ret != AW_SUCCESS)
			break;
		ret = aw8680x_flash_write_bin_to_soc(p_aw8680x, flash_flag);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE("update flash app fail");
		return ret;
	}
	AWLOGI("update flash app success");

	return AW_SUCCESS;
}

static int32_t aw8680x_flash_boot_bin_update_to_soc(struct aw8680x *p_aw8680x,
							uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t update_count = 3;

	aw8680x_start_for_uboot(p_aw8680x, AW_ROM_BOOT_START);
	while (update_count--) {
		ret = aw8680x_erase_flash_data(p_aw8680x, flash_flag);
		if (ret != AW_SUCCESS)
			break;

		ret = aw8680x_flash_write_bin_to_soc(p_aw8680x, flash_flag);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE("update flash boot fail");
		p_aw8680x->update_mutex_flag = true;
		return ret;
	}
	AWLOGI("update flash boot success");
	aw8680x_start_for_uboot(p_aw8680x, AW_FLASH_BOOT_START);

	return ret;
}

/*******************************************************************************
 * In flash boot bin, the main information is:
 * $info.len : the size of bin
 * $info.data : all data of bin
 * $flash_boot_addr : base address of flash boot which is saved in bin
 * $bin_data_type : the bin type
 * $header_info[0].app_version : the firmware version of flash boot bin
 *****************************************************************************/
static int32_t aw8680x_flash_boot_bin_parsed(struct aw8680x *p_aw8680x,
						const struct firmware *cont)
{
	int32_t ret = DATA_INIT;
	uint32_t bin_data_type = DATA_INIT;

	p_aw8680x->flash_boot_bin = devm_kzalloc(p_aw8680x->dev,
				cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->flash_boot_bin)) {
		AWLOGE("failed to allcating memory!");
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_BOOT_BIN_ERR;
	}
	p_aw8680x->flash_boot_bin->info.len = cont->size;
	memcpy(p_aw8680x->flash_boot_bin->info.data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw8680x_parsing_bin_file(p_aw8680x->flash_boot_bin);
	if (ret != DATA_INIT) {
		AWLOGE("AP parse flash boot bin failed!!");
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_BOOT_BIN_ERR;
	}

	p_aw8680x->flash_boot_addr =
			p_aw8680x->flash_boot_bin->header_info[0].download_addr;
	bin_data_type = p_aw8680x->flash_boot_bin->header_info[0].bin_data_type;
	AWLOGI("In flash boot bin, flash boot base addr = 0x%x",
						p_aw8680x->flash_boot_addr);
	if (bin_data_type != SOC_APP_DATA_TYPE) {
		AWLOGE("bin not soc protcol!");
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_BOOT_BIN_ERR;
	}
	if ((p_aw8680x->flash_boot_addr < FLASH_BOOT_BASE_ADDR) ||
			(p_aw8680x->flash_boot_addr > FLASH_APP_BASE_ADDR)) {
		AWLOGE("flash boot bin address err");
		AWLOGE("flash boot bin downloadaddr is 0x%x",
						p_aw8680x->flash_boot_addr);
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_BOOT_BIN_ERR;
	}
	p_aw8680x->flash_boot_version_in_bin =
			p_aw8680x->flash_boot_bin->header_info[0].app_version;
	AWLOGI("the version of flash boot in bin is : V%x",
					p_aw8680x->flash_boot_version_in_bin);
	AWLOGI("AP parse flash boot bin success!");
	aw8680x_flash_boot_bin_update_to_soc(p_aw8680x, AW8680X_FLASH_BOOT_UPDATE);
	devm_kfree(p_aw8680x->dev, p_aw8680x->flash_boot_bin);
	p_aw8680x->update_mutex_flag = true;
	p_aw8680x->flash_boot_states = true;

	return AW_SUCCESS;
}

static void aw8680x_flash_bin_loaded(const struct firmware *cont, void *context)
{
	struct aw8680x *p_aw8680x = context;

	if (!cont) {
		AWLOGE("Can't find the bin file : %s!",
							aw8680x_flash_boot_bin);
		p_aw8680x->update_mutex_flag = true;
		return;
	}
	AWLOGI("Find the bin file : %s!", aw8680x_flash_boot_bin);
	aw8680x_flash_boot_bin_parsed(p_aw8680x, cont);
	aw8680x_hw_reset(p_aw8680x);
	mdelay(GO_FLASH_APP_TIME);
	aw8680x_connect(p_aw8680x);
	AWLOGI("pc point %x", p_aw8680x->pc_location);

}

static int32_t aw8680x_adb_update_flash_boot_bin(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, 1,
			aw8680x_flash_boot_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_flash_bin_loaded);

}

/*******************************************************************************
 * function : if update failed firstly, retry update sram until three times.
 ******************************************************************************/
static int32_t aw8680x_sram_write_bin_to_soc(struct aw8680x *p_aw8680x)
{
	uint16_t write_last = DATA_INIT;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	int32_t write_count = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	valid_data_len = p_aw8680x->sram_bin->header_info[0].valid_data_len;
	AWLOGI("the valid size of sram bin is %d",
								valid_data_len);
	if (valid_data_len > WRITE_SRAM_MAX) {
		write_count = valid_data_len / WRITE_SRAM_MAX;
		write_last = valid_data_len % WRITE_SRAM_MAX;
		AWLOGI("sram write count is %d", write_count);
		AWLOGI("sram write last is %d", write_last);
		for (i = DATA_INIT; i < write_count; i++) {
			ret = aw8680x_sram_cycle_write(p_aw8680x, i,
								WRITE_SRAM_MAX);
			if (ret != AW_SUCCESS) {
				AWLOGE("sram cycle write fail");
				return ret;
			}
		}
		if (write_last != DATA_INIT) {
			ret = aw8680x_sram_cycle_write(p_aw8680x, write_count,
								write_last);
			if (ret != AW_SUCCESS) {
				AWLOGE("sram cycle write fail");
				return ret;
			}
		}
	} else {
		ret = aw8680x_sram_cycle_write(p_aw8680x, 0, valid_data_len);
		if (ret != AW_SUCCESS) {
			AWLOGE("sram cycle write fail!");
			return ret;
		}
	}
	AWLOGI("Successfully write all data to sram!!!");

	return AW_SUCCESS;
}

/*******************************************************************************
 * function : after parsing sram bin, update to sram in soc.
 * before updating sram need to jump flash boot.
 ******************************************************************************/
static int32_t aw8680x_sram_bin_retry_update(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t sram_update_retry_cont = 3;

	while (sram_update_retry_cont != DATA_INIT) {
		ret = aw8680x_sram_write_bin_to_soc(p_aw8680x);
		if (ret == AW_SUCCESS)
			return AW_SUCCESS;

		sram_update_retry_cont--;
		udelay(5);
	}
	AWLOGE("sram update error!");

	return ret;
}

static int32_t aw8680x_sram_bin_update_to_soc(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_jump_boot(p_aw8680x);
	if (ret != AW_SUCCESS) {
		AWLOGE("soc jump boot failed!");
		p_aw8680x->update_mutex_flag = true;
		return -JUMP_BOOT_FAILED;
	}
	AWLOGI("soc jump boot successfully!!");

	if ((p_aw8680x->pc_location != PC_POINT_ROM_BOOT) &&
			(p_aw8680x->pc_location != PC_POINT_FLASH_BOOT)) {
		AWLOGE("soc jump boot failed!, pc current location is 0x%x",
							p_aw8680x->pc_location);
		p_aw8680x->update_mutex_flag = true;
		return -JUMP_BOOT_FAILED;
	}

	ret = aw8680x_sram_bin_retry_update(p_aw8680x);
	if (ret != AW_SUCCESS) {
		p_aw8680x->update_mutex_flag = true;
		return -SRAM_BIN_FAILED;
	}

	ret = aw8680x_jump_sram(p_aw8680x, p_aw8680x->sram_addr);
	if (ret == AW_SUCCESS) {
		aw8680x_connect(p_aw8680x);
		if (p_aw8680x->pc_location == PC_POINT_SRAM) {
			AWLOGI("jump sram ok!!");
		} else {
			AWLOGE("jump sram failed, pc point %x",
							p_aw8680x->pc_location);
			p_aw8680x->update_mutex_flag = true;
			return -SRAM_BIN_FAILED;
		}
	} else {
		p_aw8680x->update_mutex_flag = true;
		return -SRAM_BIN_FAILED;
	}
	mdelay(SRAM_INIT_TIME);

	return AW_SUCCESS;
}

static int32_t
aw8680x_sram_bin_parsed(struct aw8680x *p_aw8680x, const struct firmware *cont)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->sram_bin = devm_kzalloc(p_aw8680x->dev, cont->size +
					sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->sram_bin)) {
		AWLOGE("failed to allcating memory!");
		return -SRAM_BIN_FAILED;
	}
	p_aw8680x->sram_bin->info.len = cont->size;
	memcpy(p_aw8680x->sram_bin->info.data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw8680x_parsing_bin_file(p_aw8680x->sram_bin);
	if (ret != AW_SUCCESS) {
		AWLOGE("parse sram bin fail!");
		return -SRAM_BIN_FAILED;
	}
	p_aw8680x->sram_addr = p_aw8680x->sram_bin->header_info[0].download_addr;
	if (p_aw8680x->sram_bin->header_info[0].bin_data_type != SOC_APP_DATA_TYPE) {
		AWLOGE("SOC APP DATA TYPE error!");
		return -SRAM_BIN_FAILED;
	}
	if ((p_aw8680x->sram_addr < SRAM_BASE_ADDR) || (p_aw8680x->sram_addr > SRAM_MAX_ADDR)) {
		AWLOGE("update sram address err!");
		AWLOGE("sram download_addr is 0x%x", p_aw8680x->sram_addr);
		return -SRAM_BIN_FAILED;
	}
	AWLOGE("parse sram bin successfully!");

	return AW_SUCCESS;
}

static int32_t
aw8680x_sram_bin_update(struct aw8680x *p_aw8680x, const struct firmware *cont)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_sram_bin_parsed(p_aw8680x, cont);
	if (ret != AW_SUCCESS)
		return ret;

	ret = aw8680x_sram_bin_update_to_soc(p_aw8680x);
	if (ret != AW_SUCCESS)
		return ret;

	return AW_SUCCESS;
}

/*******************************************************************************
 * the function is used to load sram bin and load flash bin from ap to soc.
 ******************************************************************************/
static void aw8680x_sram_bin_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = context;

	AWLOGI("AP start to get sram bin from system");

	if (!cont) {
		AWLOGE("Can't find the bin file : %s!",
							aw8680x_sram_bin);
		p_aw8680x->update_mutex_flag = true;
		return;
	}

	AWLOGI("Find the bin file : %s!!", aw8680x_sram_bin);

	ret = aw8680x_sram_bin_update(p_aw8680x, cont);
	devm_kfree(p_aw8680x->dev, p_aw8680x->sram_bin);
	if (ret != AW_SUCCESS) {
		aw8680x_connect(p_aw8680x);
		AWLOGI("pc location is 0x%x", p_aw8680x->pc_location);
		return;
	}
	aw8680x_flash_app_bin_update_to_soc(p_aw8680x,
						AW8680X_FLASH_APP_UPDATE);
	devm_kfree(p_aw8680x->dev, p_aw8680x->flash_app_bin);
	p_aw8680x->update_mutex_flag = true;
	aw8680x_hw_reset(p_aw8680x);
	mdelay(GO_FLASH_APP_TIME);
	aw8680x_connect(p_aw8680x);
	p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
	AWLOGI("updating flash app completed pc point %x", p_aw8680x->pc_location);

	if(p_aw8680x->flash_app_version_in_bin != p_aw8680x->pc_location) {
		AWLOGE("flash app update and jump app fail because of version in bin is != app version in soc!");
		p_aw8680x->flash_app_states = false;
	} else {
		AWLOGI("flash app update and jump app succese because of flash app version in bin equal to flash app version in soc.");
		p_aw8680x->flash_app_states = true;
	}

	if (p_aw8680x->irq_gpio_valid == true) {
		enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
	}
}

static int32_t aw8680x_sram_bin_get(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, 1,
			aw8680x_sram_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_sram_bin_loaded);
}

static void aw8680x_flash_app_bin_update_judge(struct aw8680x *p_aw8680x)
{
	if (p_aw8680x->adb_update_flash_app == ADB_UPDATE_FLASH_APP) {
		p_aw8680x->flash_app_update_flag = true;
	} else {
		if (p_aw8680x->flash_app_version_get_flag == true) {
			if (p_aw8680x->flash_app_version_in_bin !=
					p_aw8680x->flash_app_version_in_soc) {
				AWLOGI("flash app version in bin is higher flash app version in soc!");
				p_aw8680x->flash_app_update_flag = true;
			} else {
				AWLOGI("flash app version in bin equal to flash app version in soc.");
				p_aw8680x->flash_app_update_flag = false;
				p_aw8680x->pc_point_flash_app = p_aw8680x->flash_app_version_in_soc;
			}
		} else {
			p_aw8680x->flash_app_update_flag = true;
		}
	}

	if (p_aw8680x->flash_app_update_flag == true) {
		AWLOGI("flash app need to update!!!");
		aw8680x_sram_bin_get(p_aw8680x);
	} else {
		AWLOGI("flash app not need to update!");
		devm_kfree(p_aw8680x->dev, p_aw8680x->flash_app_bin);
		p_aw8680x->update_mutex_flag = true;
		AWLOGI("flash app jump succese because of flash app version in bin equal to flash app version in soc.");
		p_aw8680x->flash_app_states = true;

		if (p_aw8680x->irq_gpio_valid == true) {
			enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
		}
	}
}

/*******************************************************************************
 * In flash app bin, the main information is:
 * $info.len : the size of bin
 * $info.data : all data of bin
 * $flash_boot_addr : base address of flash app which is saved in bin
 * $bin_data_type : the bin type
 * $header_info[0].app_version : the firmware version of flash app bin
 *****************************************************************************/
static int32_t aw8680x_flash_app_bin_parsed(struct aw8680x *p_aw8680x,
						const struct firmware *cont)
{
	int32_t ret = DATA_INIT;
	uint32_t bin_data_type = DATA_INIT;

	p_aw8680x->flash_app_bin = devm_kzalloc(p_aw8680x->dev,
				cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->flash_app_bin)) {
		AWLOGE("AP failed allcate memory to flash app bin!");
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_APP_BIN_ERR;
	}
	p_aw8680x->flash_app_bin->info.len = cont->size;
	memcpy(p_aw8680x->flash_app_bin->info.data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw8680x_parsing_bin_file(p_aw8680x->flash_app_bin);
	if (ret != AW_SUCCESS) {
		AWLOGE("AP parse flash app bin failed!");
		p_aw8680x->update_mutex_flag = true;
		return ret;
	}
	p_aw8680x->flash_app_addr = p_aw8680x->flash_app_bin->header_info[0].download_addr;
	bin_data_type = p_aw8680x->flash_app_bin->header_info[0].bin_data_type;
	AWLOGI("In flash app bin, down load base addr is : 0x%x",
				p_aw8680x->flash_app_addr);
	if (bin_data_type != SOC_APP_DATA_TYPE) {
		AWLOGE("soc app data type in prased flash app bin is error!");
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_APP_BIN_ERR;
	}
	if ((p_aw8680x->flash_app_addr < FLASH_APP_BASE_ADDR) ||
			(p_aw8680x->flash_app_addr > FLASH_MAX_ADDR)) {
		AWLOGE("AP update flash app address err!!");
		AWLOGE("flash app download addr is 0x%x",
			p_aw8680x->flash_app_addr);
		p_aw8680x->update_mutex_flag = true;
		return -FLAH_APP_BIN_ERR;
	}
	p_aw8680x->flash_app_version_in_bin = p_aw8680x->flash_app_bin->header_info[0].app_version;
	AWLOGI("In flash app bin, the app version is : V%x",
					p_aw8680x->flash_app_version_in_bin);
	AWLOGI("In flash app soc, the app version is : V%x",
					p_aw8680x->flash_app_version_in_soc);
	AWLOGI("AP parse flash app bin success!");

	return AW_SUCCESS;
}

static void
aw8680x_flash_app_bin_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = context;

	if (!cont) {
		AWLOGE("Can't find the bin file : %s!", aw8680x_flash_app_bin);
		p_aw8680x->update_mutex_flag = true;
		return;
	}
	AWLOGI("Find the bin file : %s!", aw8680x_flash_app_bin);

	ret = aw8680x_flash_app_bin_parsed(p_aw8680x, cont);
	if (ret != AW_SUCCESS) {
		AWLOGI("flash app not need to update because flash app bin parsing failed!");
		release_firmware(cont);
		devm_kfree(p_aw8680x->dev, p_aw8680x->flash_app_bin);
		p_aw8680x->update_mutex_flag = true;
		return;
	}
	aw8680x_flash_app_bin_update_judge(p_aw8680x);
}

static int32_t aw8680x_flash_app_bin_get(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, 1,
			aw8680x_flash_app_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_flash_app_bin_loaded);
}

/*******************************************************************************
 * use bin from system docunment which must be completely.
 * the reason is why need to use 5 seconds delay work.
 * step 1: judge the version
 ******************************************************************************/
static void aw8680x_bin_work_routine(struct work_struct *work)
{
	ssize_t ret_fir = DATA_INIT;
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x =
			container_of(work, struct aw8680x, bin_work.work);

	AWLOGI("enter");

	ret = aw8680x_jump_flash_app(p_aw8680x, FLASH_APP_BASE_ADDR);
	if (ret != AW_SUCCESS) {
		AWLOGE("jump flash err!, the flash app is blank");
		p_aw8680x->flash_app_version_get_flag = false;
	} else {
		AWLOGI("jump flash OK!!");
		mdelay(FLASH_APP_VERSION_GET_TIME);
		ret = aw8680x_flash_app_version_in_soc_get(p_aw8680x);
		if (ret != AW_SUCCESS) {
			// p_aw8680x->update_mutex_flag = true;
			// return;
			p_aw8680x->flash_app_version_get_flag = false;
		} else {
			p_aw8680x->flash_app_version_get_flag = true;
			AWLOGI("flash app version in soc is : V%x",
					p_aw8680x->flash_app_version_in_soc);
		}
	}
	ret_fir = aw8680x_flash_app_bin_get(p_aw8680x);
	if (ret_fir != AW_SUCCESS)
		AWLOGE("request sram bin firmware failed!");
}

static void aw8680x_bin_init(struct aw8680x *p_aw8680x, int32_t cfg_timer_val)
{
	p_aw8680x->update_mutex_flag = false;
	p_aw8680x->flash_app_states = false;

	INIT_DELAYED_WORK(&p_aw8680x->bin_work, aw8680x_bin_work_routine);
	schedule_delayed_work(&p_aw8680x->bin_work,
					msecs_to_jiffies(cfg_timer_val));
}

static void aw8680x_adb_sram_bin_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = context;

	if (!cont) {
		AWLOGE("Can't find the bin file : %s!",
							aw8680x_sram_bin);
		p_aw8680x->update_mutex_flag = true;
		return;
	}
	AWLOGI("Find the bin file : %s!",
							aw8680x_sram_bin);

	ret = aw8680x_sram_bin_update(p_aw8680x, cont);
	devm_kfree(p_aw8680x->dev, p_aw8680x->sram_bin);
	if (ret != AW_SUCCESS) {
		p_aw8680x->update_mutex_flag = true;
		aw8680x_connect(p_aw8680x);
		AWLOGI("pc location is 0x%x", p_aw8680x->pc_location);
		return;
	}
	aw8680x_adb_update_flash_boot_bin(p_aw8680x);
}

static int32_t aw8680x_flash_boot_bin_update(struct aw8680x *p_aw8680x)
{
	p_aw8680x->update_mutex_flag = false;
	p_aw8680x->flash_boot_states = false;

	return request_firmware_nowait(THIS_MODULE, 1,
			aw8680x_sram_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_adb_sram_bin_loaded);

}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint32_t data_buf[3] = { 0 };
	uint8_t write_data[8] = { 0 };

	/*
	 * data_buf[0] == 0x01 : write reg
	 * data_buf[0] == 0x02 : read reg
	 */
	if (sscanf(buf, "%x %x %x", &data_buf[0], &data_buf[1], &data_buf[2]) == 3) {
		if ((data_buf[0] != 0x01) && (data_buf[0] != 0x02)) {
			AWLOGE("first param must be 0x01 or 0x02!");
			return count;
		}
	} else {
		AWLOGE("please confirm param num!");
		return count;
	}

	memcpy(write_data, &data_buf[1], sizeof(write_data));
	aw8680x_wake_state_pin_judge(g_aw8680x);
	aw8680x_write_data_to_reg(g_aw8680x, write_data, (uint8_t)data_buf[0]);

	return count;
}

static ssize_t reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	uint32_t reg_addr = 0;
	uint32_t reg_data = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_read_data_from_reg(g_aw8680x, &reg_data, &reg_addr);
	if (ret != 0)
		len += snprintf(buf + len, PAGE_SIZE - len, "reg get err\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "reg_addr[0x%08x]: reg_data[0x%08x]\n",
									reg_addr, reg_data);

	return len;
}

static ssize_t connect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int connect_count = 5;
	int ret = -1;

	while (connect_count--) {
		aw8680x_wake_state_pin_judge(g_aw8680x);
		ret = aw8680x_connect(g_aw8680x);
		if (ret == 0)
			break;
	}
	if (ret == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len,
							"connect success!\n");
		len += snprintf(buf + len, PAGE_SIZE - len,
					"pc location is 0x%x\n",
					g_aw8680x->pc_location);
		if (g_aw8680x->pc_location == PC_POINT_FLASH_BOOT) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"pc point flash boot!\n");
		} else if (g_aw8680x->pc_location == PC_POINT_ROM_BOOT) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"pc point rom boot!\n");
		} else if (g_aw8680x->pc_location == PC_POINT_SRAM) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"pc point sram!\n");
		} else {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"pc point flash app!!!\n");
		}
	} else
		len += snprintf(buf + len, PAGE_SIZE - len, "connect fail!\n");

	return len;
}

static ssize_t update_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (g_aw8680x->irq_gpio_valid == true)
		disable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	if (val == ADB_UPDATE_FLASH_BOOT) {
		if (g_aw8680x->flash_boot_func == false) {
			AWLOGI("flash boot update function is unsupport!");
			return count;
		}
		if (g_aw8680x->update_mutex_flag == false) {
			AWLOGE("update flash is not ok, please wait!");
			return count;
		}

		AWLOGI("update flash boot");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_flash_boot_bin_update(g_aw8680x);
	} else if (val == ADB_UPDATE_FLASH_APP) {
		if (g_aw8680x->update_mutex_flag == false) {
			AWLOGE("update flash is not ok, please wait!");
			return count;
		}

		AWLOGI("update flash app");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_bin_init(g_aw8680x, AW8680X_ADB_BIN_INIT_DELAY);
	} else {
		AWLOGE("adb information err!");
		return count;
	}
	g_aw8680x->adb_update_flash_app = val;

	return count;
}

static ssize_t jump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int32_t jump_count = 3;
	int32_t ret = -1;
	int8_t databuf[10] = { 0 };

	AWLOGI("pc location is %x, pc_point_flash_app = %x",
			g_aw8680x->pc_location, g_aw8680x->pc_point_flash_app);

	if (g_aw8680x->irq_gpio_valid == true)
		disable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	if (sscanf(buf, "%s", databuf) == 1) {
		if ((strcmp(databuf, "flash_app") == AW_SUCCESS)
		&& (g_aw8680x->pc_location != g_aw8680x->pc_point_flash_app)) {
			AWLOGI("pc point not flash app");
			while (jump_count--) {
				mdelay(FLASH_BOOT_INIT_TIME);
				ret = aw8680x_jump_flash_app(g_aw8680x,
							FLASH_APP_BASE_ADDR);
				if (ret == AW_SUCCESS) {
					AWLOGI("jump flash app OK!!");
					mdelay(FLASH_APP_INIT_TIME);
					break;
				}
			}
		} else if ((strcmp(databuf, "boot") == AW_SUCCESS) &&
			(g_aw8680x->pc_location != PC_POINT_ROM_BOOT) &&
			(g_aw8680x->pc_location != PC_POINT_FLASH_BOOT)) {
			AWLOGI("pc point not flash boot");
			aw8680x_hw_reset(g_aw8680x);
			aw8680x_stay_boot(g_aw8680x);
		} else {
			AWLOGE("Do not known jump where");
		}
	}
	if (g_aw8680x->irq_gpio_valid == true)
		enable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	return count;
}

static ssize_t reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		AWLOGI("enter adb reset");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
	}

	return count;
}


static ssize_t wakeup_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		AWLOGI("enter adb wakeup to high");
	    gpio_set_value_cansleep(g_aw8680x->wake_gpio, HIGH_LEVEL);
	} else if (val == 0){
		AWLOGI("enter adb wakeup to low");
	    gpio_set_value_cansleep(g_aw8680x->wake_gpio, LOW_LEVEL);
    } else {
		AWLOGI("enter adb wakeup and invalid parameter");
    }

	return count;
}


static ssize_t adc_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int16_t adc_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_adc_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("adc data read fail!");
	} else {
		memcpy(adc_data, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"adc data[%d] = 0x%x\n", i, adc_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t flash_app_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
				"flash app update status = 0x%x\n", g_aw8680x->flash_app_states);

	return len;
}

static ssize_t flash_boot_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
				"flash boot update status = 0x%x\n", g_aw8680x->flash_boot_states);

	return len;
}

static ssize_t force_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t force_event[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_force_event(g_aw8680x);
	if (ret != 0) {
		AWLOGE("force event read fail!");
	} else {
		memcpy(force_event, &g_aw8680x->read_data[1], AW_CHANNEL_NUM);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"force event[%d] = 0x%x\n",
					i, force_event[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t force_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t force_data_temp[AW_CHANNEL_NUM * sizeof(uint32_t)] = { 0 };
	uint32_t force_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_force_data(g_aw8680x, force_data_temp);
	if (ret != 0) {
		AWLOGE("force data read fail!");
	} else {
		memcpy(force_data, force_data_temp, AW_CHANNEL_NUM * sizeof(uint32_t));
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"force data[%d] = 0x%x\n",
					i, force_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t base_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int16_t base_line[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_base_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("base data read fail!");
	} else {
		memcpy(base_line, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"base data[%d] = 0x%x\n", i, base_line[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t diff_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int16_t diff_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_diff_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("diff data read fail!");
	} else {
		memcpy(diff_data, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"diff_data[%d] = 0x%x\n",
					i, diff_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t diff_threshold_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int16_t diff_threshold[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_diff_threshold(g_aw8680x, diff_threshold);
	if (ret != 0) {
		AWLOGE("diff threshold read fail!");
	} else {
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"diff_threshold[%d] = 0x%x\n",
					i, diff_threshold[i]);
		}
	}

	return len;
}

static ssize_t sensor_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	uint8_t sensor_type = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_sensor_type(g_aw8680x, &sensor_type);
	if (ret != 0)
		AWLOGE("sensor_type read fail!");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "sensor_type = 0x%x\n", sensor_type);

	return len;
}

static ssize_t sw_algo_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_sw_algo_version(g_aw8680x);
	if (ret != 0) {
		AWLOGE("sw algo version read fail!");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "sw algo version = %02x.%02x.%02x.%02x.%02x.%02x\n",
				g_aw8680x->read_data[0],
				g_aw8680x->read_data[1], g_aw8680x->read_data[2],
				g_aw8680x->read_data[3], g_aw8680x->read_data[4],
				g_aw8680x->read_data[5]);
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t pga_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t pga_data[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_pga_data_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "pga get err\n");
	} else {
		memcpy(pga_data, &g_aw8680x->read_data[1], sizeof(pga_data));
		for (i = 0; i < sizeof(uint16_t); i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"pga[%d] = 0x%x\n", i, pga_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t tempera_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	int16_t temperature = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_tempera_data_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "tempera get err\n");
	} else {
		temperature = (int16_t)(g_aw8680x->read_data[1] |
				 (((uint16_t)g_aw8680x->read_data[2]) << 8)) / 10;
		len += snprintf(buf + len, PAGE_SIZE - len, "tempera = %d\n", temperature);
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t adc_dr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t adc_dr_data[AW_ADC_DR_LEN] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_ADC_DR_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "adc dr get err\n");
	} else {
		memcpy(adc_dr_data, &g_aw8680x->read_data[1], sizeof(adc_dr_data));
		for (i = 0; i < AW_ADC_DR_LEN; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"adc dr data[%d] = 0x%x\n", i, adc_dr_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t dac_dr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t dac_dr_data[AW_DAC_DR_LEN] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_DAC_DR_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "dac dr get err\n");
	} else {
		memcpy(dac_dr_data, &g_aw8680x->read_data[1], sizeof(dac_dr_data));
		for (i = 0; i < AW_DAC_DR_LEN; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"dac dr data[%d] = 0x%x\n", i, dac_dr_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t dac_dr_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int i = 0;
	uint32_t dac_dr_data[AW_DAC_DR_LEN] = { 0 };

	for (i = 0; i < AW_DAC_DR_LEN; i++) {
		if (sscanf(&buf[5 * i], "%02x", &dac_dr_data[i]) != 1) {
			AWLOGE("sscanf err");
			return count;
		}
	}
	aw8680x_wake_state_pin_judge(g_aw8680x);
	aw8680x_DAC_DR_set(g_aw8680x, (uint8_t *)dac_dr_data);

	return count;
}

static ssize_t cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	uint8_t cali = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_cali_set(g_aw8680x, &cali);
	if (ret != 0)
		len += snprintf(buf + len, PAGE_SIZE - len, "cali get err\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "cali = 0x%x\n", cali);

	return len;
}

static ssize_t sensor_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t sensor_data[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_sensor_status_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "sensor status get err\n");
	} else {
		memcpy(sensor_data, &g_aw8680x->read_data[1], sizeof(sensor_data));
		for (i = 0; i < sizeof(sensor_data); i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"sensor status[%d] = 0x%x\n",
						i, sensor_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t adc_coef_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t adc_voltage[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_adc_transfer_voltage_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "adc voltage get err\n");
	} else {
		memcpy(adc_voltage, &g_aw8680x->read_data[1], sizeof(adc_voltage));
		for (i = 0; i < sizeof(adc_voltage); i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"adc voltage[%d] = 0x%x\n",
						i, adc_voltage[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	return len;
}

static ssize_t dac_coef_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t dac_voltage[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_dac_transfer_voltage_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "dac voltage get err\n");
	} else {
		memcpy(dac_voltage, &g_aw8680x->read_data[1], sizeof(dac_voltage));
		for (i = 0; i < sizeof(dac_voltage); i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"dac voltage[%d] = 0x%x\n",
						i, dac_voltage[i]);
			memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
		}
	}

	return len;
}

static ssize_t FTC_cali_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int rc = 0;
	uint32_t data_buf = 0;

	rc = kstrtouint(buf, 0, &data_buf);
	if (rc < 0)
		return rc;

	AWLOGI("value=0x%02X", data_buf);

	if (data_buf == 1) {
		AWLOGI("enter FTC_cali mode");
		aw8680x_enter_FTC_cali_mode(g_aw8680x);
	} else if (data_buf == 0) {
		AWLOGI("exit FTC_cali mode");
		aw8680x_exit_FTC_cali_mode(g_aw8680x);
	} else {
		AWLOGE("unsupported!");
	}

	return count;
}

static ssize_t FTC_coef_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint32_t data_buf[AW_CHANNEL_NUM] = { 0 };
	uint8_t write_data[sizeof(uint32_t) * AW_CHANNEL_NUM] = { 0 };

	if (sscanf(buf, "%x %x %x %x", &data_buf[0], &data_buf[1],
					 &data_buf[2], &data_buf[3]) == 4) {
		memcpy(write_data, data_buf, sizeof(write_data));
		aw8680x_wake_state_pin_judge(g_aw8680x);
		aw8680x_FTC_coef_write_to_flash(g_aw8680x, write_data);
	}

	return count;
}

static ssize_t FTC_coef_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	uint8_t flash_data[AW_CHANNEL_NUM * sizeof(uint32_t)] = { 0 };
	uint32_t FTC_coef[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_coef_read_from_flash(g_aw8680x, flash_data);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(FTC_coef, flash_data, AW_CHANNEL_NUM * sizeof(uint32_t));
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"FTC coef[%d] = 0x%x\n",
						i, FTC_coef[i]);
		}
	}

	return len;
}

static ssize_t FTC_noise_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int32_t noise_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	int32_t noise_std[AW_CHANNEL_NUM] = { 0 };
	int32_t noise_peak[AW_CHANNEL_NUM] = { 0 };
	int32_t noise_max_min[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_noise_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(noise_std, g_aw8680x->read_data, noise_len);
		memcpy(noise_max_min, &g_aw8680x->read_data[len], noise_len);
		memcpy(noise_peak, &g_aw8680x->read_data[len * 2], noise_len);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"ch%d : noise_std = %duV noise_max_min = %duV noise_peak = %duV\n",
					i, noise_std[i],
					noise_max_min[i], noise_peak[i]);
		}
	}

	return len;
}

static ssize_t FTC_no_press_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int ret = -1;
	int32_t data_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	int32_t raw0_data[AW_CHANNEL_NUM] = { 0 };
	int32_t raw0_max_min[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_no_press_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(raw0_data, g_aw8680x->read_data, data_len);
		memcpy(raw0_max_min, &g_aw8680x->read_data[data_len], data_len);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"ch%d : average data = %duV raw0_max_min = %d\n",
					i, raw0_data[i], raw0_max_min[i]);
		}
	}

	return len;
}


static ssize_t aw8680x_ndt_1hz_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	unsigned int databuf[3] = { 0 };
	unsigned char check_reg1 = 0;
	unsigned char check_reg2[2] = { 0 };
	int times = 3;
	int ret = -1;

	if (sscanf(buf, "%x %x %x", &databuf[0], &databuf[1], &databuf[2]) == 3) {
		check_reg1 = (unsigned char)databuf[0];
		check_reg2[0] = (unsigned char)databuf[1];
		check_reg2[1] = (unsigned char)databuf[2];
		aw8680x_wake_state_pin_judge(g_aw8680x);

		ret = aw8680x_register_i2c_writes(p_aw8680x, 0x56,
					&check_reg1, 1);
		if (ret < 0) {
			AWLOGE("failed to write data to 0x56");
			return count;
		}

		ret = aw8680x_register_i2c_writes(p_aw8680x, 0x01,
					check_reg2, sizeof(check_reg2));
		if (ret < 0) {
			AWLOGE("failed to write data to 0x01");
			return count;
		}

		mdelay(100);
		while(times--) {
			aw8680x_register_i2c_reads(p_aw8680x, 0x56, 1);
			if (g_aw8680x->read_data[0] == 0x05) {
				AWLOGI("1 hz set successfully");
				break;
			}
			mdelay(10);
		}
	}
	AWLOGI("reg56 is %x", g_aw8680x->read_data[0]);

	return count;
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t ndt_tp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
    uint32_t data_buf[2] = { 0 };
    uint8_t write_data[4] = { 0 };

    if (sscanf(buf, "%x %x", &data_buf[0], &data_buf[1]) == 2) {
        return count;
    } else {
        AWLOGE("please confirm param num!");
        return count;
    }

    write_data[0] = data_buf[0] & 0xFF;
    write_data[1] = (data_buf[0] >> 8) & 0xFF;

    write_data[2] = data_buf[1] & 0xFF;
    write_data[3] = (data_buf[1] >> 8) & 0xFF;

    aw8680x_wake_state_pin_judge(g_aw8680x);
    aw8680x_register_i2c_writes(g_aw8680x, 0xB8, write_data, sizeof(write_data));
    AWLOGD("write_data[0] = %d, write_data[1] = %d, write_data[2] = %d, write_data[3] = %d \n",
    write_data[0],write_data[1],write_data[2],write_data[3]);

    return count;
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t ndt_restore_coeff_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint32_t data_buf = 0;
	int32_t times = 3;
	int ret = -1;
	unsigned char restore_coeff_val = 0x01;
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &data_buf);
	if (ret < 0) {
		return ret;
	}

	AWLOGI("data_buf = %d", data_buf);

	if (data_buf == 1) {
		AWLOGI("start restore coeff!");
		aw8680x_wake_state_pin_judge(g_aw8680x);
		ret = aw8680x_register_i2c_writes(p_aw8680x, NDT_RESTORE_COEFF_ADDR, &restore_coeff_val, sizeof(restore_coeff_val));
		if (ret < DATA_INIT) {
			AWLOGE("failed to write data to NDT_RESTORE_COEFF_ADDR, ret is : %d", ret);
			return EIO;
		}

		mdelay(100);

		while(times--) {
			ret = aw8680x_register_i2c_reads(p_aw8680x, NDT_RESTORE_COEFF_ADDR, NDT_RESTORE_COEFF_LEN);
			if (ret < DATA_INIT) {
				AWLOGE("failed to read data from NDT_RESTORE_COEFF_ADDR, ret is : %d", ret);
				return -EIO;
			} else {
				if (p_aw8680x->read_data[0] == 0x00) {
					AWLOGI("restore_coeff successfully");
					return count;
				} else {
					AWLOGI("restore_coeff retry times = %d, readback NDT_RESTORE_COEFF_ADDR vlaue = 0x%x", times, p_aw8680x->read_data[0]);
				}
			}
			mdelay(10);
		}
		AWLOGI("restore_coeff failed times = %d, readback NDT_RESTORE_COEFF_ADDR vlaue = 0x%x", times, p_aw8680x->read_data[0]);
		return -EFAULT;
	} else {
		AWLOGE("unsupported!");
	}

	return count;
}


static ssize_t ndt_restore_coeff_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	unsigned char restore_coeff_val = 0x01;
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);

	aw8680x_wake_state_pin_judge(g_aw8680x);

	ret = aw8680x_register_i2c_reads(p_aw8680x, NDT_RESTORE_COEFF_ADDR, NDT_RESTORE_COEFF_LEN);
	if (ret < DATA_INIT) {
		len += snprintf(buf + len, PAGE_SIZE - len, "restore_coeff_val get err\n");
	} else {
		restore_coeff_val = p_aw8680x->read_data[0];
		len += snprintf(buf + len, PAGE_SIZE - len,
				"restore_coeff_val = 0x%x\n", restore_coeff_val);
	}

	return len;
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t force_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint32_t data_buf = 0;
    int32_t jump_count = 3;
	int ret = -1;

	ret = kstrtouint(buf, 0, &data_buf);
	if (ret < 0) {
		return ret;
	}

	AWLOGI("mode = %d", data_buf);

	if (g_aw8680x->update_mutex_flag == false) {
		AWLOGE("update flash is not ok, please wait!");
		return -EFAULT;
	}

	if (data_buf == 0) {
		AWLOGI("Disable force work mode");
		g_aw8680x->flash_app_states = false;
		gpio_set_value_cansleep(g_aw8680x->reset_gpio, HIGH_LEVEL);
		udelay(150);
		//platform close ldo power and delay sometime until power stability
		msleep(2);
	} else if (data_buf == 1) {
		AWLOGI("Enable force work mode and jump flash app");
		g_aw8680x->flash_app_states = false;
		//platform open ldo power and delay sometime until power stability
		msleep(2);
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		while (jump_count--) {
				mdelay(FLASH_BOOT_INIT_TIME);
				ret = aw8680x_jump_flash_app(g_aw8680x, FLASH_APP_BASE_ADDR);
				if (ret == AW_SUCCESS) {
					AWLOGI("jump flash app OK!!");
					mdelay(FLASH_APP_VERSION_GET_TIME);
					ret = aw8680x_flash_app_version_in_soc_get(g_aw8680x);
					if (ret != AW_SUCCESS) {
						AWLOGI("flash app version readback retry jump_count = %d", jump_count);
					} else {
						if (g_aw8680x->flash_app_version_in_bin == g_aw8680x->flash_app_version_in_soc) {
					        g_aw8680x->flash_app_states = true;
							AWLOGI("flash app version readback  check PASS!!");
							return count;
						} else {
							AWLOGI("flash app version check retry jump_count = %d", jump_count);
						}
					}
				}
		}
		AWLOGE("flash app version readback or check Failed, so jump flash app failed!!");
		return -EFAULT;
	} else if(data_buf == 2) {
		AWLOGI("Enable force work mode and Determine whether to update the flash app");
		//platform open ldo power and delay sometime until power stability
		msleep(2);
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_bin_init(g_aw8680x, AW8680X_ADB_BIN_INIT_DELAY);
    } else {
		AWLOGE("unsupported!");
	}

	return count;
}

static ssize_t ndt_reg_dump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	uint32_t i = 0;
	int16_t reg_dump_value[NDT_DEBUG_DUMP_LEN] = {0};

	aw8680x_wake_state_pin_judge(g_aw8680x);

	ret = aw8680x_register_i2c_reads(p_aw8680x, NDT_DEBUG_DUMP_ADDR, NDT_DEBUG_DUMP_LEN * 2);
	if (ret < DATA_INIT) {
		len += snprintf(buf + len, PAGE_SIZE - len, "ndt_reg_dump get err\n");
	} else {
		memcpy(reg_dump_value, &p_aw8680x->read_data[0], NDT_DEBUG_DUMP_LEN * 2);
		for (i = 0; i < NDT_DEBUG_DUMP_LEN; i++) {
			if (i == 13) {
				len += snprintf(buf + len, PAGE_SIZE - len,
						"reg_dump_value[%d] = 0x%04x\n",
						i,  reg_dump_value[i]);
			}
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg_dump_value[%d] = %hd\n",
					i,  reg_dump_value[i]);
		}
	}

	return len;
}

static DEVICE_ATTR_RO(ndt_reg_dump);
static DEVICE_ATTR_RW(ndt_restore_coeff);
static DEVICE_ATTR_WO(ndt_tp);
static DEVICE_ATTR_WO(force_mode);
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RO(connect);
static DEVICE_ATTR_WO(update);
static DEVICE_ATTR_WO(aw8680x_ndt_1hz);
static DEVICE_ATTR_RO(flash_app_status);
static DEVICE_ATTR_RO(flash_boot_status);
static DEVICE_ATTR_WO(jump);
static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_WO(wakeup);
static DEVICE_ATTR_RO(adc_data);
static DEVICE_ATTR_RO(force_event);
static DEVICE_ATTR_RO(force_data);
static DEVICE_ATTR_RO(base_data);
static DEVICE_ATTR_RO(diff_data);
static DEVICE_ATTR_RO(diff_threshold);
static DEVICE_ATTR_RO(sensor_type);
static DEVICE_ATTR_RO(sw_algo_version);
static DEVICE_ATTR_RO(pga_data);
static DEVICE_ATTR_RO(tempera_data);
static DEVICE_ATTR_RO(adc_dr);
static DEVICE_ATTR_RW(dac_dr);
static DEVICE_ATTR_RO(cali);
static DEVICE_ATTR_RO(sensor_status);
static DEVICE_ATTR_RO(adc_coef);
static DEVICE_ATTR_RO(dac_coef);
static DEVICE_ATTR_WO(FTC_cali);
static DEVICE_ATTR_RW(FTC_coef);
static DEVICE_ATTR_RO(FTC_noise);
static DEVICE_ATTR_RO(FTC_no_press);

static struct attribute *aw8680x_attributes[] = {
	&dev_attr_ndt_reg_dump.attr,
	&dev_attr_ndt_restore_coeff.attr,
	&dev_attr_ndt_tp.attr,
	&dev_attr_force_mode.attr,
	&dev_attr_reg.attr,
	&dev_attr_connect.attr,
	&dev_attr_update.attr,
	&dev_attr_aw8680x_ndt_1hz.attr,
	&dev_attr_jump.attr,
	&dev_attr_reset.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_adc_data.attr,
	&dev_attr_force_event.attr,
	&dev_attr_force_data.attr,
	&dev_attr_base_data.attr,
	&dev_attr_diff_data.attr,
	&dev_attr_diff_threshold.attr,
	&dev_attr_sensor_type.attr,
	&dev_attr_pga_data.attr,
	&dev_attr_tempera_data.attr,
	&dev_attr_adc_dr.attr,
	&dev_attr_dac_dr.attr,
	&dev_attr_cali.attr,
	&dev_attr_sensor_status.attr,
	&dev_attr_adc_coef.attr,
	&dev_attr_dac_coef.attr,
	&dev_attr_FTC_cali.attr,
	&dev_attr_FTC_coef.attr,
	&dev_attr_FTC_noise.attr,
	&dev_attr_FTC_no_press.attr,
	&dev_attr_sw_algo_version.attr,
	&dev_attr_flash_app_status.attr,
	&dev_attr_flash_boot_status.attr,
	NULL
};

static struct attribute_group aw8680x_attribute_group = {
	.attrs = aw8680x_attributes
};

static ssize_t proc_reg_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	int ret = -1;
	uint32_t reg_addr = 0;
	uint32_t reg_data = 0;
	char page[1024];

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_read_data_from_reg(g_aw8680x, &reg_data, &reg_addr);
	if (ret != 0)
		len += snprintf(page + len, PAGE_SIZE - len, "reg get err\n");
	else
		len += snprintf(page + len, PAGE_SIZE - len, "reg_addr[0x%08x]: reg_data[0x%08x]\n",
									reg_addr, reg_data);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_reg_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[100] = { 0 };
	uint32_t data_buf[3] = { 0 };
	uint8_t write_data[8] = { 0 };

	if (count > 100)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);

	/*
	 * data_buf[0] == 0x01 : write reg
	 * data_buf[0] == 0x02 : read reg
	 */
	if (sscanf(buffer, "%x %x %x", &data_buf[0], &data_buf[1], &data_buf[2]) == 3) {
		if ((data_buf[0] != 0x01) && (data_buf[0] != 0x02)) {
			AWLOGE("first param must be 0x01 or 0x02!");
			return count;
		}
	} else {
		AWLOGE("please confirm param num!");
		return count;
	}

	memcpy(write_data, &data_buf[1], sizeof(write_data));
	aw8680x_wake_state_pin_judge(g_aw8680x);
	aw8680x_write_data_to_reg(g_aw8680x, write_data, (uint8_t)data_buf[0]);

	return count;
}

static ssize_t proc_connect_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	int ret = -1;
	int connect_count = 5;
	char page[1024];

	while (connect_count--) {
		aw8680x_wake_state_pin_judge(g_aw8680x);
		ret = aw8680x_connect(g_aw8680x);
		if (ret == 0)
			break;
	}
	if (ret == 0) {
		len += snprintf(page + len, PAGE_SIZE - len,
							"connect success!\n");
		len += snprintf(page + len, PAGE_SIZE - len,
					"pc location is 0x%x\n",
					g_aw8680x->pc_location);
		if (g_aw8680x->pc_location == PC_POINT_FLASH_BOOT) {
			len += snprintf(page + len, PAGE_SIZE - len,
							"pc point flash boot!\n");
		} else if (g_aw8680x->pc_location == PC_POINT_ROM_BOOT) {
			len += snprintf(page + len, PAGE_SIZE - len,
							"pc point rom boot!\n");
		} else if (g_aw8680x->pc_location == PC_POINT_SRAM) {
			len += snprintf(page + len, PAGE_SIZE - len,
							"pc point sram!\n");
		} else {
			len += snprintf(page + len, PAGE_SIZE - len,
					"pc point flash app!!!\n");
		}
	} else
		len += snprintf(page + len, PAGE_SIZE - len, "connect fail!\n");

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_update_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[10] = { 0 };
	unsigned int val = 0;
	int rc = 0;

	if (count > 10)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);

	rc = kstrtouint(buffer, 0, &val);
	if (rc < 0)
		return rc;

	if (g_aw8680x->irq_gpio_valid == true)
		disable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	if (val == ADB_UPDATE_FLASH_BOOT) {
		if (g_aw8680x->flash_boot_func == false) {
			AWLOGI("flash boot update function is unsupport!");
			return count;
		}
		if (g_aw8680x->update_mutex_flag == false) {
			AWLOGE("update flash is not ok, please wait!");
			return count;
		}

		AWLOGI("update flash boot");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_flash_boot_bin_update(g_aw8680x);
	} else if (val == ADB_UPDATE_FLASH_APP) {
		if (g_aw8680x->update_mutex_flag == false) {
			AWLOGE("update flash is not ok, please wait!");
			return count;
		}

		AWLOGI("update flash app");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_bin_init(g_aw8680x, AW8680X_ADB_BIN_INIT_DELAY);
	} else {
		AWLOGE("adb information err!");
		return count;
	}
	g_aw8680x->adb_update_flash_app = val;

	return count;
}

static ssize_t proc_aw8680x_ndt_1hz_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[21] = { 0 };
	unsigned char check_reg1 = 0;
	unsigned char check_reg2[2] = { 0 };
	unsigned int databuf[3] = { 0 };
	int times = 3;
	int ret = -1;

	if (count > 21)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);

	if (sscanf(buffer, "%x %x %x", &databuf[0], &databuf[1], &databuf[2]) == 3) {
		check_reg1 = (unsigned char)databuf[0];
		check_reg2[0] = (unsigned char)databuf[1];
		check_reg2[1] = (unsigned char)databuf[2];
		aw8680x_wake_state_pin_judge(g_aw8680x);

		ret = aw8680x_register_i2c_writes(g_aw8680x, 0x56,
					&check_reg1, 1);
		if (ret < 0) {
			AWLOGE("failed to write data to 0x56");
			return count;
		}

		ret = aw8680x_register_i2c_writes(g_aw8680x, 0x01,
					check_reg2, sizeof(check_reg2));
		if (ret < 0) {
			AWLOGE("failed to write data to 0x01");
			return count;
		}

		mdelay(100);
		while(times--) {
			aw8680x_register_i2c_reads(g_aw8680x, 0x56, 1);
			if (g_aw8680x->read_data[0] == 0x05) {
				AWLOGI("1 hz set successfully");
				break;
			}
			mdelay(10);
		}
	}
	AWLOGI("reg56 is %x", g_aw8680x->read_data[0]);

	return count;
}

static ssize_t proc_flash_app_status_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	int ret = -1;
	char page[1024];

	len += snprintf(page + len, PAGE_SIZE - len,
				"flash app update status = 0x%x\n", g_aw8680x->flash_app_states);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_flash_boot_status_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	int ret = -1;
	char page[1024];

	len += snprintf(page + len, PAGE_SIZE - len,
				"flash boot update status = 0x%x\n", g_aw8680x->flash_boot_states);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_jump_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[21] = { 0 };
	int32_t jump_count = 3;
	int32_t ret = -1;
	int8_t databuf[10] = { 0 };

	if (count > 21)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);

	AWLOGI("pc location is %x, pc_point_flash_app = %x",
			g_aw8680x->pc_location, g_aw8680x->pc_point_flash_app);

	if (g_aw8680x->irq_gpio_valid == true)
		disable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	if (sscanf(buffer, "%s", databuf) == 1) {
		if ((strcmp(databuf, "flash_app") == AW_SUCCESS)
		&& (g_aw8680x->pc_location != g_aw8680x->pc_point_flash_app)) {
			AWLOGI("pc point not flash app");
			while (jump_count--) {
				mdelay(FLASH_BOOT_INIT_TIME);
				ret = aw8680x_jump_flash_app(g_aw8680x,
							FLASH_APP_BASE_ADDR);
				if (ret == AW_SUCCESS) {
					AWLOGI("jump flash app OK!!");
					mdelay(FLASH_APP_INIT_TIME);
					break;
				}
			}
		} else if ((strcmp(databuf, "boot") == AW_SUCCESS) &&
			(g_aw8680x->pc_location != PC_POINT_ROM_BOOT) &&
			(g_aw8680x->pc_location != PC_POINT_FLASH_BOOT)) {
			AWLOGI("pc point not flash boot");
			aw8680x_hw_reset(g_aw8680x);
			aw8680x_stay_boot(g_aw8680x);
		} else {
			AWLOGE("Do not known jump where");
		}
	}
	if (g_aw8680x->irq_gpio_valid == true)
		enable_irq(gpio_to_irq(g_aw8680x->irq_gpio));

	return count;
}

static ssize_t proc_reset_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	unsigned int val = 0;
	int rc = 0;

	if (count > 5)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);
	rc = kstrtouint(buffer, 0, &val);
	if (rc < 0)
		return rc;

	if (val == 1) {
		AWLOGI("enter adb reset");
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
	}

	return count;
}

static ssize_t proc_wakeup_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	unsigned int val = 0;
	int rc = 0;

	if (count > 5)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGI("buffer=%s", buffer);
	rc = kstrtouint(buffer, 0, &val);
	if (rc < 0)
		return rc;

	if (val == 1) {
		AWLOGI("enter adb wakeup to high");
		gpio_set_value_cansleep(g_aw8680x->wake_gpio, HIGH_LEVEL);
	} else if (val == 0){
		AWLOGI("enter adb wakeup to low");
		gpio_set_value_cansleep(g_aw8680x->wake_gpio, LOW_LEVEL);
	} else {
		AWLOGI("enter adb wakeup and invalid parameter");
	}

	return count;
}

static ssize_t proc_adc_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int16_t adc_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_adc_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("adc data read fail!");
	} else {
		memcpy(adc_data, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
				"adc data[%d] = 0x%x\n", i, adc_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_force_event_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t force_event[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_force_event(g_aw8680x);
	if (ret != 0) {
		AWLOGE("force event read fail!");
	} else {
		memcpy(force_event, &g_aw8680x->read_data[1], AW_CHANNEL_NUM);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"force event[%d] = 0x%x\n",
					i, force_event[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_force_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t force_data_temp[AW_CHANNEL_NUM * sizeof(uint32_t)] = { 0 };
	uint32_t force_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_force_data(g_aw8680x, force_data_temp);
	if (ret != 0) {
		AWLOGE("force data read fail!");
	} else {
		memcpy(force_data, force_data_temp, AW_CHANNEL_NUM * sizeof(uint32_t));
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"force data[%d] = 0x%x\n",
					i, force_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_base_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int16_t base_line[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_base_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("base data read fail!");
	} else {
		memcpy(base_line, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
				"base data[%d] = 0x%x\n", i, base_line[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_diff_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int16_t diff_data[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_diff_data(g_aw8680x);
	if (ret != 0) {
		AWLOGE("diff data read fail!");
	} else {
		memcpy(diff_data, &g_aw8680x->read_data[2], AW_CHANNEL_NUM * 2);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"diff_data[%d] = 0x%x\n",
					i, diff_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_diff_threshold_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int16_t diff_threshold[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_diff_threshold(g_aw8680x, diff_threshold);
	if (ret != 0) {
		AWLOGE("diff threshold read fail!");
	} else {
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"diff_threshold[%d] = 0x%x\n",
					i, diff_threshold[i]);
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_sensor_type_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int ret = -1;
	uint8_t sensor_type = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_sensor_type(g_aw8680x, &sensor_type);
	if (ret != 0)
		AWLOGE("sensor_type read fail!");
	else
		len += snprintf(page + len, PAGE_SIZE - len, "sensor_type = 0x%x\n", sensor_type);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_sw_algo_version_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int ret = -1;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_get_sw_algo_version(g_aw8680x);
	if (ret != 0) {
		AWLOGE("sw algo version read fail!");
	} else {
		len += snprintf(page + len, PAGE_SIZE - len, "sw algo version = %02x.%02x.%02x.%02x.%02x.%02x\n",
				g_aw8680x->read_data[0],
				g_aw8680x->read_data[1], g_aw8680x->read_data[2],
				g_aw8680x->read_data[3], g_aw8680x->read_data[4],
				g_aw8680x->read_data[5]);
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_pga_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t pga_data[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_pga_data_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "pga get err\n");
	} else {
		memcpy(pga_data, &g_aw8680x->read_data[1], sizeof(pga_data));
		for (i = 0; i < sizeof(uint16_t); i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
				"pga[%d] = 0x%x\n", i, pga_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_tempera_data_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int ret = -1;
	int16_t temperature = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_tempera_data_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "tempera get err\n");
	} else {
		temperature = (int16_t)(g_aw8680x->read_data[1] |
				 (((uint16_t)g_aw8680x->read_data[2]) << 8)) / 10;
		len += snprintf(page + len, PAGE_SIZE - len, "tempera = %d\n", temperature);
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_adc_dr_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t adc_dr_data[AW_ADC_DR_LEN] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_ADC_DR_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "adc dr get err\n");
	} else {
		memcpy(adc_dr_data, &g_aw8680x->read_data[1], sizeof(adc_dr_data));
		for (i = 0; i < AW_ADC_DR_LEN; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
				"adc dr data[%d] = 0x%x\n", i, adc_dr_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_dac_dr_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t dac_dr_data[AW_DAC_DR_LEN] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_DAC_DR_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "dac dr get err\n");
	} else {
		memcpy(dac_dr_data, &g_aw8680x->read_data[1], sizeof(dac_dr_data));
		for (i = 0; i < AW_DAC_DR_LEN; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
				"dac dr data[%d] = 0x%x\n", i, dac_dr_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_dac_dr_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	int i = 0;
	char buffer[60] = { 0 };
	uint32_t dac_dr_data[AW_DAC_DR_LEN] = { 0 };

	if (count > 60)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	AWLOGE("buffer=%s", buffer);

	for (i = 0; i < AW_DAC_DR_LEN; i++) {
		if (sscanf(&buffer[5 * i], "%02x", &dac_dr_data[i]) != 1) {
			AWLOGE("sscanf err");
			return count;
		}
	}
	aw8680x_wake_state_pin_judge(g_aw8680x);
	aw8680x_DAC_DR_set(g_aw8680x, (uint8_t *)dac_dr_data);

	return count;
}

static ssize_t proc_cali_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int ret = -1;
	uint8_t cali = 0;

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_cali_set(g_aw8680x, &cali);
	if (ret != 0)
		len += snprintf(page + len, PAGE_SIZE - len, "cali get err\n");
	else
		len += snprintf(page + len, PAGE_SIZE - len, "cali = 0x%x\n", cali);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_sensor_status_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t sensor_data[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_sensor_status_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "sensor status get err\n");
	} else {
		memcpy(sensor_data, &g_aw8680x->read_data[1], sizeof(sensor_data));
		for (i = 0; i < sizeof(sensor_data); i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"sensor status[%d] = 0x%x\n",
						i, sensor_data[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_adc_coef_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t adc_voltage[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_adc_transfer_voltage_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "adc voltage get err\n");
	} else {
		memcpy(adc_voltage, &g_aw8680x->read_data[1], sizeof(adc_voltage));
		for (i = 0; i < sizeof(adc_voltage); i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"adc voltage[%d] = 0x%x\n",
						i, adc_voltage[i]);
		}
		memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_dac_coef_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t dac_voltage[2] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_dac_transfer_voltage_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "dac voltage get err\n");
	} else {
		memcpy(dac_voltage, &g_aw8680x->read_data[1], sizeof(dac_voltage));
		for (i = 0; i < sizeof(dac_voltage); i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"dac voltage[%d] = 0x%x\n",
						i, dac_voltage[i]);
			memset(g_aw8680x->read_data, 0, sizeof(g_aw8680x->read_data));
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_FTC_cali_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	int rc = 0;
	uint32_t data_buf = 0;

	if (count > 5)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	rc = kstrtouint(buffer, 0, &data_buf);
	if (rc < 0)
		return rc;

	AWLOGI("value=0x%02X", data_buf);

	if (data_buf == 1) {
		AWLOGI("enter FTC_cali mode");
		aw8680x_enter_FTC_cali_mode(g_aw8680x);
	} else if (data_buf == 0) {
		AWLOGI("exit FTC_cali mode");
		aw8680x_exit_FTC_cali_mode(g_aw8680x);
	} else {
		AWLOGE("unsupported!");
	}

	return count;
}

static ssize_t proc_FTC_coef_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[25] = { 0 };
	uint32_t data_buf[AW_CHANNEL_NUM] = { 0 };
	uint8_t write_data[sizeof(uint32_t) * AW_CHANNEL_NUM] = { 0 };

	if (count > 25)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	if (sscanf(buffer, "%x %x %x %x", &data_buf[0], &data_buf[1],
					 &data_buf[2], &data_buf[3]) == 4) {
		memcpy(write_data, data_buf, sizeof(write_data));
		aw8680x_wake_state_pin_judge(g_aw8680x);
		aw8680x_FTC_coef_write_to_flash(g_aw8680x, write_data);
	}

	return count;
}

static ssize_t proc_FTC_coef_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	uint8_t flash_data[AW_CHANNEL_NUM * sizeof(uint32_t)] = { 0 };
	uint32_t FTC_coef[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_coef_read_from_flash(g_aw8680x, flash_data);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(FTC_coef, flash_data, AW_CHANNEL_NUM * sizeof(uint32_t));
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"FTC coef[%d] = 0x%x\n",
						i, FTC_coef[i]);
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_FTC_noise_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int32_t noise_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	int32_t noise_std[AW_CHANNEL_NUM] = { 0 };
	int32_t noise_peak[AW_CHANNEL_NUM] = { 0 };
	int32_t noise_max_min[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_noise_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(noise_std, g_aw8680x->read_data, noise_len);
		memcpy(noise_max_min, &g_aw8680x->read_data[len], noise_len);
		memcpy(noise_peak, &g_aw8680x->read_data[len * 2], noise_len);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"ch%d : noise_std = %duV noise_max_min = %duV noise_peak = %duV\n",
					i, noise_std[i],
					noise_max_min[i], noise_peak[i]);
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_FTC_no_press_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int i = 0;
	int ret = -1;
	int32_t data_len = AW_CHANNEL_NUM * sizeof(uint32_t);
	int32_t raw0_data[AW_CHANNEL_NUM] = { 0 };
	int32_t raw0_max_min[AW_CHANNEL_NUM] = { 0 };

	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_FTC_no_press_get(g_aw8680x);
	if (ret != 0) {
		len += snprintf(page + len, PAGE_SIZE - len, "FTC coef get err\n");
	} else {
		memcpy(raw0_data, g_aw8680x->read_data, data_len);
		memcpy(raw0_max_min, &g_aw8680x->read_data[data_len], data_len);
		for (i = 0; i < AW_CHANNEL_NUM; i++) {
			len += snprintf(page + len, PAGE_SIZE - len,
					"ch%d : average data = %duV raw0_max_min = %d\n",
					i, raw0_data[i], raw0_max_min[i]);
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t proc_force_mode_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	uint32_t data_buf = 0;
	int32_t jump_count = 3;
	int ret = -EFAULT;

	if (count > 5)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	ret = kstrtouint(buffer, 0, &data_buf);
	if (ret < 0) {
		return ret;
	}
	AWLOGI("mode = %d", data_buf);

	if (g_aw8680x->update_mutex_flag == false) {
		AWLOGE("update flash is not ok, please wait!");
		return -EFAULT;
	}

	if (data_buf == 0) {
		AWLOGI("Disable force work mode");
		g_aw8680x->flash_app_states = false;
		gpio_set_value_cansleep(g_aw8680x->reset_gpio, HIGH_LEVEL);
		udelay(150);
		//platform close ldo power and delay sometime until power stability
		msleep(2);
	} else if (data_buf == 1) {
		AWLOGI("Enable force work mode and jump flash app");
		//platform open ldo power and delay sometime until power stability
		msleep(2);
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		while (jump_count--) {
				mdelay(FLASH_BOOT_INIT_TIME);
				ret = aw8680x_jump_flash_app(g_aw8680x, FLASH_APP_BASE_ADDR);
				if (ret == AW_SUCCESS) {
					AWLOGI("jump flash app OK!!");
					mdelay(FLASH_APP_VERSION_GET_TIME);
					ret = aw8680x_flash_app_version_in_soc_get(g_aw8680x);
					if (ret != AW_SUCCESS) {
						AWLOGI("flash app version readback retry jump_count = %d", jump_count);
					} else {
						if (g_aw8680x->flash_app_version_in_bin == g_aw8680x->flash_app_version_in_soc) {
							g_aw8680x->flash_app_states = true;
							AWLOGI("flash app version readback  check PASS!!");
							return count;
						} else {
							AWLOGI("flash app version check retry jump_count = %d", jump_count);
						}
					}
				}
		}
		AWLOGE("flash app version readback or check Failed, so jump flash app failed!!");
		return -EFAULT;
	} else if(data_buf == 2) {
		AWLOGI("Enable force work mode and Determine whether to update the flash app");
		//platform open ldo power and delay sometime until power stability
		msleep(2);
		aw8680x_hw_reset(g_aw8680x);
		aw8680x_stay_boot(g_aw8680x);
		aw8680x_connect(g_aw8680x);
		aw8680x_bin_init(g_aw8680x, AW8680X_ADB_BIN_INIT_DELAY);
	} else {
		AWLOGE("unsupported!");
	}

	return count;
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t proc_ndt_restore_coeff_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	uint32_t data_buf = 0;
	char buffer[5] = { 0 };
	int32_t times = 3;
	int ret = -EFAULT;
	unsigned char restore_coeff_val = 0x01;

	if (count > 5)
		return count;
	if (copy_from_user(buffer, buf, count)) {
		AWLOGE("error");
		return -EFAULT;
	}

	ret = kstrtouint(buffer, 0, &data_buf);
	if (ret < 0) {
		return ret;
	}

	AWLOGI("data_buf = %d", data_buf);

	if (data_buf == 1) {
		AWLOGI("start restore coeff!");
		aw8680x_wake_state_pin_judge(g_aw8680x);
		ret = aw8680x_register_i2c_writes(g_aw8680x, NDT_RESTORE_COEFF_ADDR, &restore_coeff_val, sizeof(restore_coeff_val));
		if (ret < DATA_INIT) {
			AWLOGE("failed to write data to NDT_RESTORE_COEFF_ADDR, ret is : %d", ret);
			return EIO;
		}

		mdelay(100);

		while(times--) {
			ret = aw8680x_register_i2c_reads(g_aw8680x, NDT_RESTORE_COEFF_ADDR, NDT_RESTORE_COEFF_LEN);
			if (ret < DATA_INIT) {
				AWLOGE("failed to read data from NDT_RESTORE_COEFF_ADDR, ret is : %d", ret);
				return -EIO;
			} else {
				if (g_aw8680x->read_data[0] == 0x00) {
					AWLOGI("restore_coeff successfully");
					return count;
				} else {
					AWLOGI("restore_coeff retry times = %d, readback NDT_RESTORE_COEFF_ADDR vlaue = 0x%x", times, g_aw8680x->read_data[0]);
				}
			}
			mdelay(10);
		}
		AWLOGI("restore_coeff failed times = %d, readback NDT_RESTORE_COEFF_ADDR vlaue = 0x%x", times, g_aw8680x->read_data[0]);
		return -EFAULT;
	} else {
		AWLOGE("unsupported!");
	}

	return count;
}

static ssize_t proc_ndt_restore_coeff_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char page[1024];
	int ret = -1;
	unsigned char restore_coeff_val = 0x01;

	aw8680x_wake_state_pin_judge(g_aw8680x);

		ret = aw8680x_register_i2c_reads(g_aw8680x, NDT_RESTORE_COEFF_ADDR, NDT_RESTORE_COEFF_LEN);
	if (ret < DATA_INIT) {
		len += snprintf(page + len, PAGE_SIZE - len, "restore_coeff_val get err\n");
	} else {
		restore_coeff_val = g_aw8680x->read_data[0];
		len += snprintf(page + len, PAGE_SIZE - len,
				"restore_coeff_val = 0x%x\n", restore_coeff_val);
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_ndt_reg_dump_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	int ret = -1;
	char page[1024];
	uint32_t i = 0;
	int16_t reg_dump_value[NDT_DEBUG_DUMP_LEN] = {0};

	aw8680x_wake_state_pin_judge(g_aw8680x);

	ret = aw8680x_register_i2c_reads(g_aw8680x, NDT_DEBUG_DUMP_ADDR, NDT_DEBUG_DUMP_LEN * 2);
	if (ret < DATA_INIT) {
		len += snprintf(page + len, PAGE_SIZE - len, "ndt_reg_dump get err\n");
	} else {
		memcpy(reg_dump_value, &g_aw8680x->read_data[0], NDT_DEBUG_DUMP_LEN * 2);
		for (i = 0; i < NDT_DEBUG_DUMP_LEN; i++) {
			if (i == 13) {
				len += snprintf(page + len, PAGE_SIZE - len,
						"reg_dump_value[%d] = 0x%04x\n",
						i,  reg_dump_value[i]);
			}
			len += snprintf(page + len, PAGE_SIZE - len,
					"reg_dump_value[%d] = %hd\n",
					i,  reg_dump_value[i]);
		}
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static struct file_operations proc_ops[] = {
	{ .read = proc_reg_read, .write = proc_reg_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_connect_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_update_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_aw8680x_ndt_1hz_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_flash_app_status_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_flash_boot_status_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_jump_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_reset_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_wakeup_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_adc_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_force_event_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_force_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_base_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_diff_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_diff_threshold_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_sensor_type_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_sw_algo_version_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_pga_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_tempera_data_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_adc_dr_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_dac_dr_read, .write = proc_dac_dr_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_cali_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_sensor_status_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_adc_coef_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_dac_coef_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_FTC_cali_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_FTC_coef_read, .write = proc_FTC_coef_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_FTC_noise_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_FTC_no_press_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .write = proc_force_mode_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_ndt_restore_coeff_read, .write = proc_ndt_restore_coeff_write, .open = aw8680x_file_open, .owner = THIS_MODULE, },
	{ .read = proc_ndt_reg_dump_read, .open = aw8680x_file_open, .owner = THIS_MODULE, },
};
#else
static struct proc_ops proc_ops[] = {
	{ .proc_read = proc_reg_read, .proc_write = proc_reg_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_connect_read, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_update_write, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_aw8680x_ndt_1hz_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_flash_app_status_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_flash_boot_status_read, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_jump_write, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_reset_write, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_wakeup_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_adc_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_force_event_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_force_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_base_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_diff_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_diff_threshold_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_sensor_type_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_sw_algo_version_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_pga_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_tempera_data_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_adc_dr_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_dac_dr_read, .proc_write = proc_dac_dr_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_cali_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_sensor_status_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_adc_coef_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_dac_coef_read, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_FTC_cali_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_FTC_coef_read, .proc_write = proc_FTC_coef_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_FTC_noise_read, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_FTC_no_press_read, .proc_open = aw8680x_file_open, },
	{ .proc_write = proc_force_mode_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_ndt_restore_coeff_read, .proc_write = proc_ndt_restore_coeff_write, .proc_open = aw8680x_file_open, },
	{ .proc_read = proc_ndt_reg_dump_read, .proc_open = aw8680x_file_open, },
};
#endif

static int init_vibrator_proc(struct aw8680x *p_aw8680x)
{
	int ret = 0;
	int i = 0;
	// ssize_t len = 0;

	p_aw8680x->prEntry_da = proc_mkdir("aw_press", NULL);
	if (p_aw8680x->prEntry_da == NULL) {
		ret = -ENOMEM;
		AWLOGE("Couldn't create aw_press proc entry\n");
	}
	for (i = 0; i < sizeof(aw8680x_proc_node_name) / sizeof(*aw8680x_proc_node_name); i++) {
		// len += snprintf(buf, PAGE_SIZE - len, "proc_%s_ops\n", aw8680x_proc_node_name[i]);
		p_aw8680x->prEntry_tmp[i] = proc_create_data(aw8680x_proc_node_name[i], 0664,
							p_aw8680x->prEntry_da,
							&proc_ops[i],
							p_aw8680x);
		if (p_aw8680x->prEntry_tmp[i] == NULL) {
			ret = -ENOMEM;
			AWLOGE("Couldn't create proc entry\n");
		}
	}

	return 0;
}


static int sysclass_group_register(struct aw8680x *p_aw8680x)
{
	int ret = DATA_INIT;

	if (!p_aw8680x){
		AWLOGE("Error: p_aw8680x is NULL\n");
		return -ENOMEM;
	}

	p_aw8680x->sysfs_class = class_create(THIS_MODULE, "aw_press");
	if(!p_aw8680x->sysfs_class){
		AWLOGE("sysfs_class could not be created\n");
		ret = -ENOMEM;
	} else {
		AWLOGI("sysfs_class have be created");
	}

	if(!ret){
		p_aw8680x->sysfs_dev = device_create(p_aw8680x->sysfs_class, NULL, 0, p_aw8680x, "force_dev");
		if(!p_aw8680x->sysfs_dev){
			AWLOGE("sysfs_dev could not be created\n");
			ret = -ENOMEM;
			class_destroy(p_aw8680x->sysfs_class);
			p_aw8680x->sysfs_class = NULL;
		} else {
			AWLOGI("sysfs_dev have be created");
		}
	}
	if(!ret){
		ret = sysfs_create_group(&(p_aw8680x->sysfs_dev->kobj), &aw8680x_attribute_group);
		if(ret) {
			AWLOGE("sysfs group could not be created\n");
			ret = -ENOMEM;
			device_destroy(p_aw8680x->sysfs_class, 0);
			p_aw8680x->sysfs_dev = NULL;
			class_destroy(p_aw8680x->sysfs_class);
			p_aw8680x->sysfs_class = NULL;
		}else {
			AWLOGI("sysfs_create have be created");
		}
	}

	return DATA_INIT;
}

static int32_t aw8680x_sys_create(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = sysfs_create_group(&p_aw8680x->dev->kobj, &aw8680x_attribute_group);
	if (ret != DATA_INIT) {
		AWLOGE("creating i2c obj attr err !!!");
		return -ERR_CREAT_I2C_OBJ;
	}

	ret = init_vibrator_proc(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE("create proc obj failed!");
		return -ERR_CREAT_PROC_OBJ;
	}

	ret = sysclass_group_register(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE("create sysclass obj attr failed!");
		return -ERR_CREAT_PROC_OBJ;
	}

	return AW_SUCCESS;
}

/*****************************************************
 *
 * aw8680x apk interface
 *
 *****************************************************/
static int32_t aw8680x_file_open(struct inode *inode, struct file *filp)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	filp->private_data = (void *)g_aw8680x;

	return 0;
}

static int32_t aw8680x_file_release(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t aw8680x_file_read(struct file *filp, char *buff, size_t len,
				 loff_t *offset)
{
	struct aw8680x *p_aw8680x = (struct aw8680x *)filp->private_data;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	uint8_t *pbuff = NULL;
	uint8_t reg_addr = DATA_INIT;

	AWLOGI("enter");

	if (len > 256)
		return len;
	pbuff = kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		AWLOGE("alloc memory fail");
		return len;
	}
	/* get reg addr */
	if (copy_from_user(&reg_addr, buff, 1)) {
		kfree(pbuff);
		return len;
	}
	AWLOGI("reg_addr is 0x%x", reg_addr);
	AWLOGI("read_len is %zu", len);
	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_register_i2c_reads(p_aw8680x, reg_addr, len);
	if (ret < DATA_INIT) {
		AWLOGE("failed to read data, ret is : %d", ret);
		kfree(pbuff);
		return len;
	}
	for (i = DATA_INIT; i < len; i++) {
		pbuff[i] = p_aw8680x->read_data[i];
		AWLOGI("pbuff[%d] = 0x%02x", i, pbuff[i]);
	}
	ret = copy_to_user(buff + 1, pbuff, len);
	if (ret) {
		AWLOGI("copy to user fail");
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static ssize_t aw8680x_file_write(struct file *filp, const char *buff,
				  size_t len, loff_t *off)
{
	struct aw8680x *p_aw8680x = (struct aw8680x *)filp->private_data;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	unsigned char *pbuff = NULL;
	unsigned char reg_addr = DATA_INIT;

	AWLOGI("enter");

	if (len > 256)
		return len;
	pbuff = kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		AWLOGE("alloc memory fail");
		return len;
	}
	/* get reg addr */
	ret = copy_from_user(&reg_addr, buff, 1);
	if (ret) {
		AWLOGE("copy from user reg_addr fail");
		kfree(pbuff);
		return len;
	}
	AWLOGI("reg_addr is 0x%x", reg_addr);
	AWLOGI("write_len is %zu", len);
	/* get reg data */
	ret = copy_from_user(pbuff, buff + 1, len);
	if (ret) {
		AWLOGE("copy from user reg_data fail");
		kfree(pbuff);
		return len;
	}
	for (i = 0; i < len; i++)
		AWLOGI("pbuff[%d] = 0x%02x", i, pbuff[i]);
	aw8680x_wake_state_pin_judge(g_aw8680x);
	ret = aw8680x_register_i2c_writes(p_aw8680x, reg_addr, pbuff, len);
	if (ret < 0) {
		AWLOGE("failed to write data, ret is : %d",
									ret);
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8680x_file_read,
	.write = aw8680x_file_write,
	.open = aw8680x_file_open,
	.release = aw8680x_file_release,
};

static struct miscdevice aw8680x_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8680X_NAME,
	.fops = &fops,
};

static int32_t aw8680x_file_init(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = misc_register(&aw8680x_misc);
	if (ret) {
		AWLOGE("misc fail, ret is : %d", ret);
		return ret;
	}
	return AW_SUCCESS;
}

static enum hrtimer_restart aw8680x_hr_timer_func(struct hrtimer *timer)
{
	struct aw8680x *p_aw8680x = container_of(timer, struct aw8680x, hr_timer);

	p_aw8680x->timer_wake_state = 0;

	return HRTIMER_NORESTART;
}

static int32_t aw8680x_timer_init(struct aw8680x *p_aw8680x)
{
	AWLOGI("enter");
	if ((p_aw8680x->wake_gpio_valid == true) && (p_aw8680x->state_pin_valid == true)) {
		mutex_init(&(p_aw8680x->timer_mutex));
		hrtimer_init(&p_aw8680x->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		p_aw8680x->hr_timer.function = aw8680x_hr_timer_func;
	}

	return 0;
}

/*****************************************************
 *
 * aw8680x irq
 *
 *****************************************************/
static irqreturn_t aw8680x_irq(int irq, void *data)
{
	struct aw8680x *p_aw8680x = data;
	int16_t firmware_data[20];
	int32_t i = 0;

	AWLOGI("pc location is 0x%x, pc point flash app is : 0x%x",
				p_aw8680x->pc_location, p_aw8680x->pc_point_flash_app);
	if (p_aw8680x->pc_location == p_aw8680x->pc_point_flash_app) {
				aw8680x_register_i2c_reads(p_aw8680x, 0x20, 40);
				for (i = 0; i < 20; i++) {
					firmware_data[i] = (uint16_t)((g_aw8680x->read_data[2*i +1] << 8) |
							g_aw8680x->read_data[2*i ]);
				}

				if (firmware_data[3] != -10) {
					input_report_key(p_aw8680x->input, BTN_TOUCH, 1);
					input_report_abs(p_aw8680x->input, ABS_X, firmware_data[3]);
					input_sync(p_aw8680x->input);
				} else {
					input_report_key(p_aw8680x->input, BTN_TOUCH, 0);
					input_sync(p_aw8680x->input);
				}
		}  else {
		AWLOGE("current addr is not in flash!");
	}

	return IRQ_HANDLED;
}

static int32_t aw8680x_read_chipid_once(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_connect(p_aw8680x);
	if (ret != AW_SUCCESS) {
		AWLOGE("connect failed!!! ret is : %d", ret);
	} else {
		AWLOGI("pc location is 0x%08x", p_aw8680x->pc_location);
		if (p_aw8680x->pc_location == PC_POINT_ROM_BOOT) {
			AWLOGI("pc location is rom boot!");
			return AW_SUCCESS;
		} else if (p_aw8680x->pc_location == PC_POINT_FLASH_BOOT) {
			AWLOGI("pc location is flash boot!");
			return AW_SUCCESS;
		}


	}
	AWLOGE("pc location is  0x%08x", p_aw8680x->pc_location);
	return -CHIPID_ERR;
}

static int32_t aw8680x_read_chipid(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t read_chipid_time_retry = DATA_INIT;

	while (read_chipid_time_retry < READ_CHIPID_RETRY_TIME) {
		aw8680x_hw_reset(p_aw8680x);
		aw8680x_stay_boot(p_aw8680x);
		ret = aw8680x_read_chipid_once(p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE("read chipid retry time : %d",
							read_chipid_time_retry);
			read_chipid_time_retry++;
		} else {
			return AW_SUCCESS;
		}
	}

	return ret;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
/* Get the irq port number in the device tree */
static void
aw8680x_reset_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(p_aw8680x->reset_gpio)) {
		AWLOGI("reset gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev, p_aw8680x->reset_gpio,
						GPIOF_OUT_INIT_LOW, "aw8680x_rst");
		if (ret) {
			AWLOGE("reset request failed");
			p_aw8680x->rst_gpio_valid = false;
		} else {
			AWLOGI("reset gpio request ok!!");
			p_aw8680x->rst_gpio_valid = true;
		}
	} else {
		AWLOGE("reset gpio provided failed!");
		p_aw8680x->rst_gpio_valid = false;
	}
}

static void aw8680x_irq_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;
	int32_t irq_flags = DATA_INIT;

	p_aw8680x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (gpio_is_valid(p_aw8680x->irq_gpio)) {
		AWLOGI("irq gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev, p_aw8680x->irq_gpio,
						GPIOF_DIR_IN, "aw8680x_int");
		if (ret) {
			AWLOGE("irq request failed!");
			//p_aw8680x->irq_gpio_valid = 0;
			return;
		}
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(p_aw8680x->dev,
					gpio_to_irq(p_aw8680x->irq_gpio),
					NULL, aw8680x_irq, irq_flags,
					"aw8680x", p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE("failed to request irq, ret:%d",
									ret);
			devm_gpio_free(p_aw8680x->dev, p_aw8680x->irq_gpio);
			p_aw8680x->irq_gpio_valid = 0;
		} else {
			AWLOGI("irq gpio request ok!!");
			p_aw8680x->irq_gpio_valid = 1;
			disable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
		}
	} else {
		AWLOGE("irq gpio provided failed!");
		p_aw8680x->irq_gpio_valid = 0;
	}
}

static void aw8680x_wake_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->wake_gpio = of_get_named_gpio(np, "wake-gpio", 0);
	p_aw8680x->wake_gpio_valid = gpio_is_valid(p_aw8680x->wake_gpio);
	if (p_aw8680x->wake_gpio_valid) {
		AWLOGI("wake gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev, p_aw8680x->wake_gpio,
						GPIOF_OUT_INIT_LOW, "aw8680x_wake");
		if (ret) {
			AWLOGE("wake request failed!");
			p_aw8680x->wake_gpio_valid = false;
		} else {
			AWLOGI("wake gpio request ok!!");
			p_aw8680x->timer_wake_state = 1;
			p_aw8680x->wake_gpio_valid = true;
		}
	} else {
		AWLOGE("wake gpio provided failed!");
		p_aw8680x->wake_gpio_valid = false;
	}
}

static int32_t aw8680x_ldo_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;


    p_aw8680x->ldo_gpio = of_get_named_gpio(np, "ldo-gpio", 0);
    if (p_aw8680x->ldo_gpio < 0) {
        AWLOGE( "Failed to get ldo-gpio: %d\n", p_aw8680x->ldo_gpio);
		return ERR_FLAG;
    }


    ret = gpio_request(p_aw8680x->ldo_gpio, "ldo-gpio");
    if (ret) {
        AWLOGE("Failed to request ldo-gpio: %d\n", ret);
		return ERR_FLAG;
    }


    ret = gpio_direction_output(p_aw8680x->ldo_gpio, 1);
    if (ret) {
        AWLOGE("Failed to set ldo-gpio direction: %d\n", ret);
		return ERR_FLAG;
		gpio_free(p_aw8680x->ldo_gpio);
    }
	return AW_SUCCESS;
}

static int32_t aw8680x_state_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->state_gpio = of_get_named_gpio(np, "state-gpio", 0);
	if (gpio_is_valid(p_aw8680x->state_gpio)) {
		AWLOGI("state gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev,
						p_aw8680x->state_gpio,
						GPIOF_DIR_IN,
						"aw8680x_state");
		if (ret) {
			AWLOGE("state request failed!");
			return ERR_FLAG;
		}

		AWLOGI("state gpio request ok!!");
	} else {
		AWLOGE("state gpio provided failed!");
		return ERR_FLAG;
	}
	return AW_SUCCESS;
}

static void
aw8680x_parse_dt(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = 0;

	ret = aw8680x_ldo_gpio_set(p_aw8680x, np);
	if (ret != AW_SUCCESS) {
		AWLOGE("ldo gpio request failed!");
	}

	/* Use functions in kernel to get */
	aw8680x_reset_gpio_set(p_aw8680x, np);
	if (p_aw8680x->rst_gpio_valid == false) {
		p_aw8680x->wake_gpio_valid = false;
		p_aw8680x->state_pin_valid = false;
		AWLOGE("reset gpio request failed!");
		return;
	}

	aw8680x_wake_gpio_set(p_aw8680x, np);
	if (p_aw8680x->wake_gpio_valid == false)
		AWLOGE("wake gpio request failed!");

	ret = aw8680x_state_gpio_set(p_aw8680x, np);
	if (ret != AW_SUCCESS) {
		AWLOGE("state gpio request failed!");
		p_aw8680x->state_pin_valid = false;
	} else {
		p_aw8680x->state_pin_valid = true;
	}

	aw8680x_irq_gpio_set(p_aw8680x, np);
	if (p_aw8680x->irq_gpio_valid == false)
		AWLOGE("irq gpio request failed!");

	p_aw8680x->flash_boot_func =
			of_property_read_bool(np, "flash_boot_function_used");
	if (p_aw8680x->flash_boot_func == true)
		AWLOGI("flash boot update function is used!");
	else
		AWLOGI("flash boot update function is unused!");

	p_aw8680x->input_func = of_property_read_bool(np, "input_function_used");
	if (p_aw8680x->input_func == true)
		AWLOGI("input function is used!");
	else
		AWLOGI("input function is unused!");
}

/******************************************************
 *
 * struct aw8680x init
 *
 ******************************************************/
static void aw8680x_struct_init(struct aw8680x *p_aw8680x, struct i2c_client *i2c,
						const struct i2c_device_id *id)
{
	AWLOGI("enter");

	p_aw8680x->dev = &i2c->dev;
	p_aw8680x->i2c = i2c;
	i2c_set_clientdata(i2c, p_aw8680x);
	dev_set_drvdata(&i2c->dev, p_aw8680x);
	g_aw8680x = p_aw8680x;
	p_aw8680x->adb_update_flash_app = NO_ADB;
	p_aw8680x->update_mutex_flag = true;
	p_aw8680x->flash_app_states = false;
	p_aw8680x->flash_boot_states = false;
}

static int32_t aw8680x_input_init(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	AWLOGI("enter");

	if (p_aw8680x->input_func == true) {
		p_aw8680x->input = input_allocate_device();
		if (!p_aw8680x->input) {
			AWLOGE("failed to allocate input device");
			return -INPUT_ALLOC_ERR;
		}
		p_aw8680x->input->name = AW8680X_I2C_NAME;
		__set_bit(EV_KEY, p_aw8680x->input->evbit);
		__set_bit(EV_SYN, p_aw8680x->input->evbit);
		__set_bit(EV_ABS, p_aw8680x->input->evbit);

		input_set_capability(p_aw8680x->input, EV_KEY, BTN_TOUCH);
		input_set_capability(p_aw8680x->input, EV_ABS, ABS_X);
		input_set_capability(p_aw8680x->input, EV_ABS, ABS_PRESSURE);

		input_set_abs_params(p_aw8680x->input, ABS_X, 0, 100, 0, 0);
		input_set_abs_params(p_aw8680x->input, ABS_PRESSURE, 0x0, 0xffff, 0, 0);
		ret = input_register_device(p_aw8680x->input);
		if (ret) {
			AWLOGE("failed to register input device: %s",
						dev_name(p_aw8680x->dev));
			return -INPUT_REGISTER_ERR;
		}
		AWLOGI("input device regist OK!!");
	}

	return AW_SUCCESS;
}

unsigned int ndt_tp_transfer(unsigned int x,unsigned int y)
{
	uint8_t write_data[4] = { 0 };
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;
	unsigned int pressure = 0;

	AWLOGI("X = %d, Y = %d, use_ndt_aw8680x = %d\n", x, y, use_ndt_aw8680x);
	write_data[0] = x & 0xFF;
	write_data[1] = (x >> 8) & 0xFF;
	write_data[2] = y & 0xFF;
	write_data[3] = (y >> 8) & 0xFF;

	if ((g_aw8680x != NULL) && (use_ndt_aw8680x == 1) && (g_aw8680x->flash_app_states == true)) {
		aw8680x_wake_state_pin_judge(g_aw8680x);
		ret = aw8680x_register_i2c_writes(g_aw8680x, 0xB8, write_data, sizeof(write_data));
		if (ret < DATA_INIT) {
			AWLOGE("failed to write 0xB8, ret is : %d", ret);
			return 0;
		}

		ret = aw8680x_register_i2c_reads(g_aw8680x, AW_PRESSURE_ADDR,  AW_PRESSURE_DATA_LEN);
		if (ret < DATA_INIT) {
			AWLOGE("failed to read pressure data, ret is : %d", ret);
			return 0;
		}
		for (i = DATA_INIT; i < AW_PRESSURE_DATA_LEN; i++) {
			AWLOGI("read data[%d] = 0x%x", i,
							g_aw8680x->read_data[i]);
			pressure = g_aw8680x->read_data[i];
		}
	}

	return pressure;
}

EXPORT_SYMBOL_GPL(ndt_tp_transfer);

/* In this function can do a series of initialization work */
static int32_t
aw8680x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = NULL;
	struct device_node *np = i2c->dev.of_node;

	AWLOGI("enter");
	/* Determining the ability of the adapter */
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		AWLOGE("check functionality faile");
		return -EIO;
	}
	/* Apply for memory for device structures */
	p_aw8680x = devm_kzalloc(&i2c->dev, sizeof(struct aw8680x), GFP_KERNEL);
	if (p_aw8680x == NULL)
		return -ENOMEM;

	aw8680x_struct_init(p_aw8680x, i2c, id);
	if (np) {
		aw8680x_parse_dt(p_aw8680x, np);
		if (p_aw8680x->rst_gpio_valid == false)
			goto err_reset_gpio;
	}
	mutex_init(&(p_aw8680x->aw8680x_i2c_mutex));

	ret = aw8680x_read_chipid(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE("the ic not AW8680X");
		use_ndt_aw8680x = 0;
		goto err_chipid;
	}

	ret = aw8680x_input_init(p_aw8680x);
	if (ret == -INPUT_ALLOC_ERR)
		goto err_alloc_input;
	else if (ret == -INPUT_REGISTER_ERR)
		goto err_regist_input;

	ret = aw8680x_sys_create(p_aw8680x);
	if (ret == -ERR_CREAT_I2C_OBJ)
		goto err_create_i2c_obj;
	else if (ret == -ERR_CREAT_PROC_OBJ)
		goto err_create_proc_obj;


	aw8680x_timer_init(p_aw8680x);

	aw8680x_file_init(p_aw8680x);
	if (p_aw8680x->pc_location == PC_POINT_FLASH_BOOT)
		aw8680x_bin_init(p_aw8680x, AW8680X_BIN_INIT_DELAY);

	AWLOGI("probe completed successfully!");

	return AW_SUCCESS;
err_create_proc_obj:
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);
err_create_i2c_obj:
	input_unregister_device(p_aw8680x->input);
err_regist_input:
	input_free_device(p_aw8680x->input);
err_alloc_input:
err_chipid:
	mutex_destroy(&(p_aw8680x->aw8680x_i2c_mutex));
	if (p_aw8680x->rst_gpio_valid == true)
		devm_gpio_free(&i2c->dev, p_aw8680x->reset_gpio);

	if ((p_aw8680x->wake_gpio_valid == true) && (p_aw8680x->state_pin_valid == true))  {
		devm_gpio_free(&i2c->dev, p_aw8680x->wake_gpio);
		devm_gpio_free(&i2c->dev, p_aw8680x->state_gpio);
	}
err_reset_gpio:
	devm_kfree(&i2c->dev, p_aw8680x);
	return ret;
}

static int aw8680x_i2c_remove(struct i2c_client *i2c)
{
	struct aw8680x *aw8680x = i2c_get_clientdata(i2c);
	int i = 0;

	AWLOGI("enter");
	cancel_delayed_work_sync(&aw8680x->bin_work);
	misc_deregister(&aw8680x_misc);
	if ((aw8680x->wake_gpio_valid == true) && (aw8680x->state_pin_valid == true)) {
		devm_gpio_free(&i2c->dev, aw8680x->wake_gpio);
		devm_gpio_free(&i2c->dev, aw8680x->state_gpio);
		hrtimer_cancel(&aw8680x->hr_timer);
		mutex_destroy(&(aw8680x->timer_mutex));
	}

	for (i = 0; i < sizeof(aw8680x_proc_node_name) / sizeof(*aw8680x_proc_node_name); i++) {
		if (aw8680x->prEntry_tmp[i]) {
			remove_proc_entry(aw8680x_proc_node_name[i], aw8680x->prEntry_da);
			aw8680x->prEntry_tmp[i] = NULL;
		}
	}
	remove_proc_subtree("aw_press", NULL);
	aw8680x->prEntry_da = NULL;
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);

	if (aw8680x->input_func == true) {
		input_unregister_device(aw8680x->input);
		input_free_device(aw8680x->input);
	}

	if (aw8680x->rst_gpio_valid == true)
		devm_gpio_free(&i2c->dev, aw8680x->reset_gpio);

	mutex_destroy(&(aw8680x->aw8680x_i2c_mutex));

	devm_kfree(&i2c->dev, aw8680x);

	return AW_SUCCESS;
}

/*****************************************************
 *
 * pm sleep
 *
 *****************************************************/
#ifdef CONFIG_PM_SLEEP
static int aw8680x_suspend(struct device *dev)
{
	return 0;
}

static int aw8680x_resume(struct device *dev)
{
	return 0;
}

/* Sleep wake-up mechanism, in this function you can decide
 * what to do when the phone is in sleep wake-up state
 */
static SIMPLE_DEV_PM_OPS(aw8680x_pm_ops, aw8680x_suspend, aw8680x_resume);
#endif

static const struct i2c_device_id aw8680x_i2c_id[] = {
	{AW8680X_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw8680x_i2c_id);

/* Match device information in the device tree */
static const struct of_device_id aw8680x_dt_match[] = {
	{.compatible = "awinic,aw8680x"},
	{ },
};

static struct i2c_driver aw8680x_i2c_driver = {
	.driver = {
		   .name = AW8680X_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw8680x_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw8680x_pm_ops,
#endif
		   },
	.probe = aw8680x_i2c_probe,
	.remove = aw8680x_i2c_remove,
	.id_table = aw8680x_i2c_id,
};

static int __init aw8680x_i2c_init(void)
{
	int ret = 0;

	pr_info("late_initcall: aw8680x driver version %s\n", AW8680X_DRIVER_VERSION);
	/* Register the device driver on the i2c bus */
	ret = i2c_add_driver(&aw8680x_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8680x device into i2c\n");
		return ret;
	}
	return 0;
}

/* Entry function */
late_initcall(aw8680x_i2c_init);

static void __exit aw8680x_i2c_exit(void)
{
	i2c_del_driver(&aw8680x_i2c_driver);
}

module_exit(aw8680x_i2c_exit);

MODULE_DESCRIPTION("AW8680X Sensor Driver");
MODULE_LICENSE("GPL v2");
