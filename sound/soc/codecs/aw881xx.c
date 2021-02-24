/*
 * aw881xx.c   aw881xx codec module
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include "aw881xx.h"
#include "aw881xx_reg.h"
#include "aw881xx_monitor.h"
#include "aw881xx_cali.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW881XX_I2C_NAME "aw881xx_smartpa"

#define AW881XX_DRIVER_VERSION "v0.2.3"

#define AW881XX_RATES SNDRV_PCM_RATE_8000_48000
#define AW881XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5	/* 5ms */
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5	/* 5ms */

#define AW881XX_ACF_FILE		"aw881xx_acf.bin"


static DEFINE_MUTEX(g_aw881xx_lock);
static struct aw881xx_container *g_awinic_cfg;
static unsigned int g_aw881xx_dev_cnt;

static unsigned int g_fade_in_time = AW_1000_US / 10;
static unsigned int g_fade_out_time = AW_1000_US >> 1;
static unsigned int g_fade_step = AW_VOLUME_STEP_DB;



static char *profile_name[AW_PROFILE_MAX] = {"Music", "Voice", "Voip", "Ringtone", "Ringtone_hs", "Lowpower",
						"Bypass", "Mmi", "Fm", "Notification", "Receiver"};
static const char *const aw881xx_switch[] = {"Disable", "Enable"};

/******************************************************
 *
 * Value
 *
 ******************************************************/
static char aw881xx_cfg_name[][AW881XX_CFG_NAME_MAX] = {
	/* ui */
	{"aw881xx_pid_01_ui_fw_i2c"},
	{"aw881xx_pid_01_ui_cfg_i2c"},
};

/******************************************************
 *
 * aw881xx distinguish between codecs and components by version
 *
 ******************************************************/
static struct snd_soc_dapm_widget aw881xx_dapm_widgets[] = {
	/* playback */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "Speaker_Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("audio_out"),
	/* capture */
	SND_SOC_DAPM_AIF_OUT("AIF_TX","Speaker_Capture",0,SND_SOC_NOPM,0,0),
	SND_SOC_DAPM_INPUT("iv_in"),
};

static const struct snd_soc_dapm_route aw881xx_audio_map[] = {
	{"audio_out", NULL, "AIF_RX"},
	{"AIF_TX",NULL,"iv_in"},
};

#ifdef AW_KERNEL_VER_OVER_4_19_1
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_component,
	.aw_snd_soc_codec_get_drvdata = snd_soc_component_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_component_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_component,
	.aw_snd_soc_register_codec = snd_soc_register_component,
};
#else
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_codec,
	.aw_snd_soc_codec_get_drvdata = snd_soc_codec_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_codec_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_codec,
	.aw_snd_soc_register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef AW_KERNEL_VER_OVER_4_19_1
	return dai->component;
#else
	return dai->codec;
#endif
}

static void aw881xx_reg_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len);
static int aw881xx_load_chip_profile(struct aw881xx *aw881xx);
static int aw881xx_fw_update(struct aw881xx *aw881xx, bool up_dsp_fw_en);



/******************************************************
 *
 * aw881xx append suffix sound channel information
 *
 ******************************************************/
static void *aw881xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = NULL;

	str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);
	if (!str) {
		aw_dev_err(dev, "%s:devm_kzalloc %s failed\n",
			__func__, buf);
		return str;
	}
	memcpy(str, buf, strlen(buf));
	return str;
}

static int aw881xx_append_suffix(char *format, const char **change_name,
			struct aw881xx *aw881xx)
{
	char buf[64] = { 0 };
	int i2cbus = aw881xx->i2c->adapter->nr;
	int addr = aw881xx->i2c->addr;

	snprintf(buf, 64, format, *change_name, i2cbus, addr);
	*change_name = aw881xx_devm_kstrdup(aw881xx->dev, buf);
	if (!(*change_name))
		return -ENOMEM;

	aw_dev_dbg(aw881xx->dev, "%s:change name :%s\n",
		__func__, *change_name);
	return 0;
}

/******************************************************
 *
 * aw881xx reg write/read
 *
 ******************************************************/
static int aw881xx_i2c_writes(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	int ret = -1;
	uint8_t *data = NULL;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		aw_dev_err(aw881xx->dev, "%s: can not allocate memory\n",
				__func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw881xx->i2c, data, len + 1);
	if (ret < 0)
		aw_dev_err(aw881xx->dev,
				"%s: i2c master send error\n", __func__);

	kfree(data);
	data = NULL;

	return ret;
}

static int aw881xx_i2c_reads(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *data_buf,
				uint16_t data_len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
				.addr = aw881xx->i2c->addr,
				.flags = 0,
				.len = sizeof(uint8_t),
				.buf = &reg_addr,
				},
		[1] = {
				.addr = aw881xx->i2c->addr,
				.flags = I2C_M_RD,
				.len = data_len,
				.buf = data_buf,
				},
	};

	ret = i2c_transfer(aw881xx->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != AW881XX_READ_MSG_NUM) {
		aw_dev_err(aw881xx->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	return 0;
}

static int aw881xx_i2c_write(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint16_t reg_data)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t buf[2] = { 0 };

	buf[0] = (reg_data & 0xff00) >> 8;
	buf[1] = (reg_data & 0x00ff) >> 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = aw881xx_i2c_writes(aw881xx, reg_addr, buf, 2);
		if (ret < 0)
			aw_dev_err(aw881xx->dev,
					"%s: i2c_write cnt=%d error=%d\n",
					__func__, cnt, ret);
		else
			break;
		cnt++;
	}

	return ret;
}

static int aw881xx_i2c_read(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint16_t *reg_data)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t buf[2] = { 0 };

	while (cnt < AW_I2C_RETRIES) {
		ret = aw881xx_i2c_reads(aw881xx, reg_addr, buf, 2);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
					"%s: i2c_read cnt=%d error=%d\n",
					__func__, cnt, ret);
		} else {
			*reg_data = (buf[0] << 8) | (buf[1] << 0);
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw881xx_i2c_write_bits(struct aw881xx *aw881xx,
					uint8_t reg_addr, uint16_t mask,
					uint16_t reg_data)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_i2c_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw881xx_i2c_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

int aw881xx_reg_writes(struct aw881xx *aw881xx,
				uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_writes(aw881xx, reg_addr, buf, len);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881x_i2c_writes fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_write(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, reg_addr, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881xx_i2c_write fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_read(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t *reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_read(aw881xx, reg_addr, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: aw881xx_i2c_read fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_reg_write_bits(struct aw881xx *aw881xx,
			uint8_t reg_addr, uint16_t mask, uint16_t reg_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write_bits(aw881xx, reg_addr, mask, reg_data);
	if (ret < 0)
		aw_dev_err(aw881xx->dev,
				"%s: aw881xx_i2c_write_bits fail, ret=%d",
				__func__, ret);
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;
}

int aw881xx_dsp_write(struct aw881xx *aw881xx,
			uint16_t dsp_addr, uint16_t dsp_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, dsp_addr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_write_err;
	}

	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMDAT, dsp_data);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_write_err;
	}
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;

 dsp_write_err:
	mutex_unlock(&aw881xx->i2c_lock);
	return ret;

}

int aw881xx_dsp_read(struct aw881xx *aw881xx,
			uint16_t dsp_addr, uint16_t *dsp_data)
{
	int ret = -1;

	mutex_lock(&aw881xx->i2c_lock);
	ret = aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, dsp_addr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c write error, ret=%d\n",
				__func__, ret);
		goto dsp_read_err;
	}

	ret = aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, dsp_data);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read error, ret=%d\n",
				__func__, ret);
		goto dsp_read_err;
	}
	mutex_unlock(&aw881xx->i2c_lock);

	return ret;

 dsp_read_err:
	mutex_unlock(&aw881xx->i2c_lock);
	return ret;
}

static int aw881xx_get_profile_count(struct aw881xx *aw881xx)
{
	return aw881xx->prof_info.count;
}

static char *aw881xx_get_profile_name(struct aw881xx *aw881xx, int index)
{
	int dev_profile_id;

	if ((index >= aw881xx->prof_info.count) || (index < 0)) {
		aw_dev_err(aw881xx->dev, "%s:index[%d] overflow dev prof num[%d]\n",
				__func__, index, aw881xx->prof_info.count);
		return NULL;
	}

	if (aw881xx->prof_info.prof_desc[index].id >= AW_PROFILE_MAX) {
		aw_dev_err(aw881xx->dev, "%s:can not find match id\n", __func__);
		return NULL;
	}

	dev_profile_id = aw881xx->prof_info.prof_desc[index].id;

	return profile_name[dev_profile_id];
}

static int aw881xx_get_profile_index(struct aw881xx *aw881xx)
{
	return aw881xx->cur_prof;
}

static int aw881xx_check_profile_index(struct aw881xx *aw881xx, int index)
{
	if ((index >= aw881xx->prof_info.count) || (index < 0))
		return -EINVAL;
	else
		return 0;
}

static int aw881xx_set_profile_index(struct aw881xx *aw881xx, int index)
{
	if ((index >= aw881xx->prof_info.count) || (index < 0)) {
		return -EINVAL;
	} else {
		aw881xx->set_prof = index;

		aw_dev_info(aw881xx->dev, "%s:set prof[%s]\n", __func__,
			profile_name[aw881xx->prof_info.prof_desc[index].id]);
	}

	return 0;
}

static int aw881xx_get_profile_data(struct aw881xx *aw881xx, int index,
			struct aw_prof_desc **prof_desc)
{
	if (index >= aw881xx->prof_info.count) {
		aw_dev_err(aw881xx->dev, "%s: index[%d] overflow count[%d]\n",
			__func__, index, aw881xx->prof_info.count);
		return -EINVAL;
	}

	*prof_desc = &aw881xx->prof_info.prof_desc[index];

	return 0;
}

/*
[7 : 4]: -6DB ; [3 : 0]: 0.5DB  real_value = value * 2 : 0.5db --> 1
*/
uint32_t aw881xx_reg_val_to_db(uint32_t value)
{
	return ((value >> 4) * AW881XX_VOL_6DB_STEP +
			((value & 0x0f) % AW881XX_VOL_6DB_STEP));
}

/*[7 : 4]: -6DB ;
 *[3 : 0]: -0.5DB reg_value = value / step << 4 + value % step ;
 *step = 6 * 2
 */
static uint32_t aw881xx_db_val_to_reg(uint32_t value)
{
	return (((value / AW881XX_VOL_6DB_STEP) << 4) + (value % AW881XX_VOL_6DB_STEP));
}

int aw881xx_set_volume(struct aw881xx *aw881xx, uint32_t value)
{
	uint16_t reg_value = 0;
	uint32_t real_value = aw881xx_db_val_to_reg(value);

	/*cal real value*/
	aw881xx_reg_read(aw881xx, AW881XX_REG_HAGCCFG7, &reg_value);

	aw_dev_dbg(aw881xx->dev, "%s: value %d , 0x%x\n",
			__func__, value, real_value);

	/*[15 : 8] volume*/
	real_value = (real_value << AW881XX_VOL_REG_SHIFT) | (reg_value & 0x00ff);

	/*write value*/
	aw881xx_reg_write(aw881xx, AW881XX_REG_HAGCCFG7, real_value);

	return 0;
}

int aw881xx_get_volume(struct aw881xx *aw881xx, uint32_t *value)
{
	uint16_t reg_value = 0;
	uint32_t real_value = 0;

	/* read value */
	aw881xx_reg_read(aw881xx, AW881XX_REG_HAGCCFG7, &reg_value);

	/*[15 : 8] volume*/
	real_value = reg_value >> AW881XX_VOL_REG_SHIFT;

	real_value = aw881xx_reg_val_to_db(real_value);
	*value = real_value;

	return 0;
}

static void aw881xx_fade_in(struct aw881xx *aw881xx)
{
	int i = 0;

	if (!aw881xx->fade_en)
		return;

	if (g_fade_step == 0 || g_fade_in_time == 0) {
		aw881xx_set_volume(aw881xx, aw881xx->db_offset);
		return;
	}

	/*volume up*/
	for (i = AW_FADE_OUT_TARGET_VOL; i >= (int32_t)aw881xx->db_offset; i -= g_fade_step) {
		if (i < (int32_t)aw881xx->db_offset)
			i = (int32_t)aw881xx->db_offset;

		aw881xx_set_volume(aw881xx, i);
		usleep_range(g_fade_in_time, g_fade_in_time + 10);
	}

	if (i != (int32_t)aw881xx->db_offset)
		aw881xx_set_volume(aw881xx, aw881xx->db_offset);

}

static void aw881xx_fade_out(struct aw881xx *aw881xx)
{
	int i = 0;
	unsigned start_volume = 0;

	if (!aw881xx->fade_en)
		return;

	if (g_fade_step == 0 || g_fade_out_time == 0) {
		aw881xx_set_volume(aw881xx, AW_FADE_OUT_TARGET_VOL);
		return;
	}

	aw881xx_set_volume(aw881xx, start_volume);
	i = start_volume;
	for (i = start_volume; i <= AW_FADE_OUT_TARGET_VOL; i += g_fade_step) {
		if (i > (int32_t)AW_FADE_OUT_TARGET_VOL)
			i = (int32_t)AW_FADE_OUT_TARGET_VOL;

		aw881xx_set_volume(aw881xx, i);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
	if (i != AW_FADE_OUT_TARGET_VOL) {
		aw881xx_set_volume(aw881xx, AW_FADE_OUT_TARGET_VOL);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
}



/******************************************************
 *
 * aw881xx control
 *
 ******************************************************/
static void aw881xx_i2s_tx_enable(struct aw881xx *aw881xx, bool is_enable)
{

	if (is_enable) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCFG1,
					AW881XX_BIT_I2SCFG1_TXEN_MASK,
					AW881XX_BIT_I2SCFG1_TXEN_ENABLE);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCFG1,
					AW881XX_BIT_I2SCFG1_TXEN_MASK,
					AW881XX_BIT_I2SCFG1_TXEN_DISABLE);
	}
}

static void aw881xx_run_mute(struct aw881xx *aw881xx, bool mute)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (mute) {
		aw881xx_fade_out(aw881xx);
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PWMCTRL,
					AW881XX_BIT_PWMCTRL_HMUTE_MASK,
					AW881XX_BIT_PWMCTRL_HMUTE_ENABLE);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PWMCTRL,
					AW881XX_BIT_PWMCTRL_HMUTE_MASK,
					AW881XX_BIT_PWMCTRL_HMUTE_DISABLE);
		aw881xx_fade_in(aw881xx);
	}
}

static void aw881xx_run_pwd(struct aw881xx *aw881xx, bool pwd)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (pwd) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_PW_MASK,
					AW881XX_BIT_SYSCTRL_PW_PDN);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_PW_MASK,
					AW881XX_BIT_SYSCTRL_PW_ACTIVE);
	}
}

static void aw881xx_run_amppd(struct aw881xx *aw881xx, bool amppd)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (amppd) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_AMPPD_MASK,
					AW881XX_BIT_SYSCTRL_AMPPD_PDN);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_AMPPD_MASK,
					AW881XX_BIT_SYSCTRL_AMPPD_ACTIVE);
	}
}

static void aw881xx_dsp_enable(struct aw881xx *aw881xx, bool dsp)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (dsp) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_DSP_MASK,
					AW881XX_BIT_SYSCTRL_DSP_WORK);
	} else {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL,
					AW881XX_BIT_SYSCTRL_DSP_MASK,
					AW881XX_BIT_SYSCTRL_DSP_BYPASS);
	}
}

static void aw881xx_memclk_select(struct aw881xx *aw881xx, unsigned char flag)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (flag == AW881XX_MEMCLK_PLL) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL2,
					AW881XX_BIT_SYSCTRL2_MEMCLK_MASK,
					AW881XX_BIT_SYSCTRL2_MEMCLK_PLL);
	} else if (flag == AW881XX_MEMCLK_OSC) {
		aw881xx_reg_write_bits(aw881xx, AW881XX_REG_SYSCTRL2,
					AW881XX_BIT_SYSCTRL2_MEMCLK_MASK,
					AW881XX_BIT_SYSCTRL2_MEMCLK_OSC);
	} else {
		aw_dev_err(aw881xx->dev,
				"%s: unknown memclk config, flag=0x%x\n",
				__func__, flag);
	}
}

static int aw881xx_sysst_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	unsigned char i;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_SYSST_CHECK_MAX; i++) {
		aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
		if ((reg_val & (~AW881XX_BIT_SYSST_CHECK_MASK)) ==
			AW881XX_BIT_SYSST_CHECK) {
			ret = 0;
			return ret;
		} else {
			aw_dev_dbg(aw881xx->dev,
					"%s: check fail, cnt=%d, reg_val=0x%04x\n",
					__func__, i, reg_val);
			msleep(2);
		}
	}
	aw_dev_info(aw881xx->dev, "%s: check fail\n", __func__);

	return ret;
}

int aw881xx_get_sysint(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINT, &reg_val);
	if (ret < 0)
		aw_dev_err(aw881xx->dev, "%s: read sysint fail, ret=%d\n",
				__func__, ret);
	else
		aw_dev_dbg(aw881xx->dev, "%s:read interrupt reg = 0x%04x",
				__func__, reg_val);

	return ret;
}

void aw881xx_clear_int_status(struct aw881xx *aw881xx)
{
	/*read int status and clear*/
	aw881xx_get_sysint(aw881xx);

	/*make suer int status is clear*/
	aw881xx_get_sysint(aw881xx);
}

int aw881xx_get_iis_status(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
	if (reg_val & AW881XX_BIT_SYSST_PLLS)
		ret = 0;
	else
		aw_dev_err(aw881xx->dev, "%s:check pll lock fail,reg_val:0x%04x",
			__func__, reg_val);

	return ret;
}

static int aw881xx_syspll_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	int i;

	for (i = 0; i < AW881XX_SYSST_CHECK_MAX; i++) {
		ret = aw881xx_get_iis_status(aw881xx);
		if (ret < 0) {
			aw_dev_dbg(aw881xx->dev, "%s:check pll lock fail, cnt=%d", __func__, i);
			usleep_range(AW_2000_US, AW_2000_US + 10);
		} else {
			ret = 0;
			break;
		}
	}

	return ret;
}

int aw881xx_get_dsp_status(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_WDT, &reg_val);
	if (reg_val)
		ret = 0;

	return ret;
}

int aw881xx_get_dsp_config(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSCTRL, &reg_val);
	if (reg_val & AW881XX_BIT_SYSCTRL_DSP_BYPASS)
		aw881xx->dsp_cfg = AW881XX_DSP_BYPASS;
	else
		aw881xx->dsp_cfg = AW881XX_DSP_WORK;

	return ret;
}

int aw881xx_get_hmute(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_PWMCTRL, &reg_val);
	if (reg_val & AW881XX_BIT_PWMCTRL_HMUTE_ENABLE)
		ret = 1;
	else
		ret = 0;

	return ret;
}

static int aw881xx_get_icalk(struct aw881xx *aw881xx, int16_t *icalk)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t reg_icalk = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_EFRM, &reg_val);
	reg_icalk = (uint16_t)reg_val & AW881XX_EF_ISENSE_GAINERR_SLP_MASK;

	if (reg_icalk & AW881XX_EF_ISENSE_GAINERR_SLP_SIGN_MASK)
		reg_icalk = reg_icalk | AW881XX_EF_ISENSE_GAINERR_SLP_NEG;

	*icalk = (int16_t)reg_icalk;

	return ret;
}

static int aw881xx_get_vcalk(struct aw881xx *aw881xx, int16_t *vcalk)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t reg_vcalk = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_PRODUCT_ID, &reg_val);
	reg_val = reg_val >> AW881XX_EF_VSENSE_GAIN_SHIFT;

	reg_vcalk = (uint16_t)reg_val & AW881XX_EF_VSENSE_GAIN_MASK;

	if (reg_vcalk & AW881XX_EF_VSENSE_GAIN_SIGN_MASK)
		reg_vcalk = reg_vcalk | AW881XX_EF_VSENSE_GAIN_NEG;

	*vcalk = (int16_t)reg_vcalk;

	return ret;
}

static int aw881xx_dsp_set_vcalb(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;
	int vcalb;
	int icalk;
	int vcalk;
	int16_t icalk_val = 0;
	int16_t vcalk_val = 0;

	ret = aw881xx_get_icalk(aw881xx, &icalk_val);
	if (ret < 0)
		return ret;
	ret = aw881xx_get_vcalk(aw881xx, &vcalk_val);
	if (ret < 0)
		return ret;

	icalk = AW881XX_CABL_BASE_VALUE + AW881XX_ICABLK_FACTOR * icalk_val;
	vcalk = AW881XX_CABL_BASE_VALUE + AW881XX_VCABLK_FACTOR * vcalk_val;

	if (icalk == 0 || vcalk == 0) {
		aw_dev_err(aw881xx->dev, "%s: icalk=%d, vcalk=%d is error\n",
			__func__, icalk, vcalk);
		return -EINVAL;
	}

	vcalb = (AW881XX_VCAL_FACTOR * AW881XX_VSCAL_FACTOR /
		 AW881XX_ISCAL_FACTOR) * vcalk / icalk;

	reg_val = (uint16_t) vcalb;
	aw_dev_dbg(aw881xx->dev, "%s: icalk=%d, vcalk=%d, vcalb=%d, reg_val=0x%04x\n",
			__func__, icalk, vcalk, vcalb, reg_val);

	ret = aw881xx_dsp_write(aw881xx, AW881XX_DSP_REG_VCALB, reg_val);

	return ret;
}

static int aw881xx_set_intmask(struct aw881xx *aw881xx, bool flag)
{
	int ret = -1;

	if (flag)
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM,
					aw881xx->intmask);
	else
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM,
					AW881XX_REG_SYSINTM_MASK);
	return ret;
}


/******************************************************
 *
 * aw881xx dsp
 *
 ******************************************************/
static int aw881xx_get_vmax(struct aw881xx *aw881xx, uint16_t *vmax_val)
{
	int ret;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_VMAX, vmax_val);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw881xx_dsp_check(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t dsp_check_max = 5;
	uint16_t i = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	for (i = 0; i < dsp_check_max; i++) {
		if (aw881xx->dsp_cfg == AW881XX_DSP_WORK) {
			aw881xx_dsp_enable(aw881xx, false);
			aw881xx_dsp_enable(aw881xx, true);
			msleep(1);
			ret = aw881xx_get_dsp_status(aw881xx);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev,
						"%s: dsp wdt status error=%d\n",
						__func__, ret);
			} else {
				return 0;
			}
		} else if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
			return 0;
		} else {
			aw_dev_err(aw881xx->dev, "%s: unknown dsp cfg=%d\n",
					__func__, aw881xx->dsp_cfg);
			return -EINVAL;
		}
	}

	return 0;
}

static int aw881xx_dsp_check_process(struct aw881xx *aw881xx)
{
	int i;
	int ret = -1;

	for (i = 0; i < AW881XX_DSP_CHECK_MAX; i++) {
		ret = aw881xx_dsp_check(aw881xx);
		if (ret < 0) {
			if (i == (AW881XX_DSP_CHECK_MAX - 1))
				break;

			ret = aw881xx_fw_update(aw881xx, true);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "%s:fw update failed\n",
					__func__);
				return ret;
			}
		} else {
			aw_dev_info(aw881xx->dev, "%s: dsp check pass\n",
				__func__);
			return 0;
		}
	}

	return -EINVAL;
}

static void aw881xx_dsp_update_cali_re(struct aw881xx *aw881xx)
{
	if (aw881xx->cali_attr.cali_re != AW_ERRO_CALI_VALUE) {
		aw881xx_set_cali_re_to_dsp(&aw881xx->cali_attr);
	} else {
		aw_dev_info(aw881xx->dev, "%s: no set re=%d\n",
			__func__, aw881xx->cali_attr.cali_re);
	}
}

/******************************************************
 *
 * aw881xx reg config
 *
 ******************************************************/
static void aw881xx_reg_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len)
{
	int i;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	uint16_t read_val;

	aw_dev_dbg(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < len; i += 4) {
		reg_addr = (data[i + 1] << 8) + data[i + 0];
		reg_val = (data[i + 3] << 8) + data[i + 2];
		aw_dev_dbg(aw881xx->dev, "%s:reg = 0x%02x, val = 0x%04x\n",
				__func__, reg_addr, reg_val);
		if (reg_addr == AW881XX_REG_SYSINTM) {
			aw881xx->intmask = reg_val;
			reg_val = AW881XX_REG_SYSINTM_MASK;
		}
		if (reg_addr == AW881XX_REG_PWMCTRL) {
			aw881xx_reg_read(aw881xx, reg_addr, &read_val);
			read_val &= (~AW881XX_BIT_PWMCTRL_HMUTE_MASK);
			reg_val &= AW881XX_BIT_PWMCTRL_HMUTE_MASK;
			reg_val |= read_val;
		}

		aw881xx_reg_write(aw881xx, (uint8_t)reg_addr, (uint16_t)reg_val);
	}
	aw881xx_get_volume(aw881xx, &aw881xx->db_offset);
	aw881xx_get_dsp_config(aw881xx);

	aw_dev_dbg(aw881xx->dev, "%s:exit\n", __func__);
}

static int aw881xx_reg_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len)
{

	aw_dev_dbg(aw881xx->dev, "%s:reg len:%d\n", __func__, len);

	if (len && (data != NULL)) {
		aw881xx_reg_container_update(aw881xx, data, len);
	} else {
		aw_dev_err(aw881xx->dev, "%s:reg data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_dsp_container_update(struct aw881xx *aw881xx,
				uint8_t *data, uint32_t len, uint16_t base)
{
	int i;
#ifdef AW881XX_DSP_I2C_WRITES
	uint32_t tmp_len = 0;
#else
	uint16_t reg_val = 0;
#endif

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);
	mutex_lock(&aw881xx->i2c_lock);
#ifdef AW881XX_DSP_I2C_WRITES
	/* i2c writes */
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, base);

	for (i = 0; i < len; i += AW881XX_MAX_RAM_WRITE_BYTE_SIZE) {
		if ((len - i) < AW881XX_MAX_RAM_WRITE_BYTE_SIZE)
			tmp_len = len - i;
		else
			tmp_len = AW881XX_MAX_RAM_WRITE_BYTE_SIZE;
		aw881xx_i2c_writes(aw881xx, AW881XX_REG_DSPMDAT,
					&data[i], tmp_len);
	}
#else
	/* i2c write */
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, base);
	for (i = 0; i < len; i += 2) {
		reg_val = (data[i] << 8) + data[i + 1];
		aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMDAT, reg_val);
	}
#endif
	mutex_unlock(&aw881xx->i2c_lock);
	aw_dev_dbg(aw881xx->dev, "%s: exit\n", __func__);

	return 0;
}

static int aw881xx_dsp_fw_update(struct aw881xx *aw881xx,
			uint8_t *data, uint32_t len)
{

	aw_dev_dbg(aw881xx->dev, "%s:dsp firmware len:%d\n", __func__, len);

	if (len && (data != NULL)) {
		aw881xx_dsp_container_update(aw881xx, data,
			len, AW881XX_DSP_FW_ADDR);
		aw881xx->dsp_fw_len = len;
	} else {
		aw_dev_err(aw881xx->dev, "%s: dsp firmware data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_dsp_cfg_update(struct aw881xx *aw881xx,
			uint8_t *data, uint32_t len)
{
	int ret;

	aw_dev_dbg(aw881xx->dev, "%s:dsp config len:%d\n",
		__func__, len);

	if (len && (data != NULL)) {
		aw881xx_dsp_container_update(aw881xx, data, len, AW881XX_DSP_CFG_ADDR);
		aw881xx->dsp_cfg_len = len;
		ret = aw881xx_get_vmax(aw881xx, &aw881xx->monitor.vmax_init_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:get vmax failed\n", __func__);
			return ret;
		} else {
			aw_dev_info(aw881xx->dev, "%s:get init vmax:0x%x\n",
					__func__, aw881xx->monitor.vmax_init_val);
		}

		aw881xx_dsp_set_vcalb(aw881xx);
		aw881xx_dsp_update_cali_re(aw881xx);

	} else {
		aw_dev_err(aw881xx->dev, "%s:dsp config data is null or len is 0\n",
			__func__);
		return -EPERM;
	}

	return 0;
}

static int aw881xx_fw_update(struct aw881xx *aw881xx, bool up_dsp_fw_en)
{
	int ret = -1;
	struct aw_prof_desc *set_prof_desc = NULL;
	struct aw_sec_data_desc *sec_desc = NULL;
	char *prof_name = NULL;
	bool dsp_st = false;

	if (aw881xx->fw_status == AW881XX_FW_FAILED) {
		aw_dev_err(aw881xx->dev, "%s: fw status[%d] error\n",
			__func__, aw881xx->fw_status);
		return -EPERM;
	}

	prof_name = aw881xx_get_profile_name(aw881xx, aw881xx->set_prof);
	if (prof_name == NULL)
		return -ENOMEM;

	ret = aw881xx_get_profile_data(aw881xx, aw881xx->set_prof, &set_prof_desc);
	if (ret < 0)
		return ret;

	aw_dev_info(aw881xx->dev, "%s:start update %s\n", __func__, prof_name);

	sec_desc = set_prof_desc->sec_desc;
	ret = aw881xx_reg_update(aw881xx, sec_desc[AW_DATA_TYPE_REG].data,
					sec_desc[AW_DATA_TYPE_REG].len);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:update reg failed\n", __func__);
		return ret;
	}

	aw881xx_run_mute(aw881xx, true);
	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_OSC);

	if (up_dsp_fw_en) {
		ret = aw881xx_dsp_fw_update(aw881xx, sec_desc[AW_DATA_TYPE_DSP_FW].data,
					sec_desc[AW_DATA_TYPE_DSP_FW].len);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:update dsp fw failed\n", __func__);
			return ret;
		}
	}

	ret = aw881xx_dsp_cfg_update(aw881xx, sec_desc[AW_DATA_TYPE_DSP_CFG].data,
					sec_desc[AW_DATA_TYPE_DSP_CFG].len);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:update dsp cfg failed\n", __func__);
		return ret;
	}

	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_PLL);

	dsp_st = (aw881xx->dsp_cfg == AW881XX_DSP_WORK) ? true : false;
	aw881xx_dsp_enable(aw881xx, dsp_st);

	aw881xx->cur_prof = aw881xx->set_prof;

	aw_dev_info(aw881xx->dev, "%s:load %s done\n", __func__, prof_name);
	return 0;
}

static int aw881xx_device_start(struct aw881xx *aw881xx)
{
	int ret = -1;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	if (aw881xx->status == AW881XX_PW_ON) {
		aw_dev_info(aw881xx->dev, "%s: already power on\n", __func__);
		return 0;
	}

	aw881xx_run_pwd(aw881xx, false);
	usleep_range(AW_2000_US, AW_2000_US + 100);

	ret = aw881xx_syspll_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: pll check failed cannot start\n", __func__);
		goto pll_check_fail;
	}

	/*amppd on*/
	aw881xx_run_amppd(aw881xx, false);
	usleep_range(AW_1000_US, AW_1000_US + 50);

	/*check i2s status*/
	ret = aw881xx_sysst_check(aw881xx);
	if (ret < 0) {				/*check failed*/
		aw_dev_err(aw881xx->dev, "%s: sysst check fail\n", __func__);
		goto sysst_check_fail;
	}

	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_dsp_update_cali_re(aw881xx);

	ret = aw881xx_dsp_check_process(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp check fail\n", __func__);
		goto dsp_check_fail;
	}

	aw881xx_i2s_tx_enable(aw881xx, true);

	/*clear inturrupt*/
	aw881xx_clear_int_status(aw881xx);
	/*set inturrupt mask*/
	aw881xx_set_intmask(aw881xx, true);
	/*close mute*/
	aw881xx_run_mute(aw881xx, false);

	aw881xx_monitor_start(&aw881xx->monitor);

	aw881xx->status = AW881XX_PW_ON;

	aw_dev_info(aw881xx->dev, "%s: done\n", __func__);

	return 0;


dsp_check_fail:
sysst_check_fail:
	aw881xx_run_amppd(aw881xx, true);
pll_check_fail:
	aw881xx_run_pwd(aw881xx, true);
	/*clear inturrupt*/
	aw881xx_clear_int_status(aw881xx);
	aw881xx->status = AW881XX_PW_OFF;
	return ret;
}

static void aw881xx_software_reset(struct aw881xx *aw881xx)
{
	uint8_t reg_addr = AW881XX_REG_ID;
	uint16_t reg_val = AW881XX_SOFTWARE_RST_VALUE;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_write(aw881xx, reg_addr, reg_val);
}

static int aw881xx_phase_sync(struct aw881xx *aw881xx, bool sync_enable)
{
	struct aw_prof_desc *prof_desc = NULL;

	if (aw881xx->fw_status == AW881XX_FW_FAILED) {
		aw_dev_err(aw881xx->dev, "%s:dev acf load failed\n", __func__);
		return -EPERM;
	}

	prof_desc = &aw881xx->prof_info.prof_desc[aw881xx->cur_prof];
	if (sync_enable) {
		if ((prof_desc->sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
			(prof_desc->sec_desc[AW_DATA_TYPE_REG].len != 0)) {
			/*software reset PA*/
			aw881xx_software_reset(aw881xx);

			aw881xx_reg_container_update(aw881xx,
				prof_desc->sec_desc[AW_DATA_TYPE_REG].data,
				prof_desc->sec_desc[AW_DATA_TYPE_REG].len);
		}
	}

	return 0;
}

static void aw881xx_startup_work(struct work_struct *work)
{
	int ret;
	struct aw881xx *aw881xx = container_of(work, struct aw881xx, start_work.work);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	if (aw881xx->fw_status == AW881XX_FW_OK) {
		if (aw881xx->allow_pw == false) {
			aw_dev_info(aw881xx->dev, "%s:dev can not allow power\n",
				__func__);
			return;
		}
		mutex_lock(&aw881xx->lock);
		ret = aw881xx_device_start(aw881xx);
		if (ret) {
			aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
		} else {
			aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
		}
		mutex_unlock(&aw881xx->lock);
	} else {
		aw_dev_err(aw881xx->dev, "%s:dev acf load failed\n", __func__);
	}
}

static void aw881xx_start(struct aw881xx *aw881xx)
{
	int ret;

	if (aw881xx->cali_attr.cali_re == AW_ERRO_CALI_VALUE)
		aw881xx_get_cali_re(&aw881xx->cali_attr);


	if (aw881xx->cur_prof == aw881xx->set_prof) {
		aw881xx_phase_sync(aw881xx, aw881xx->pa_syn_en);
		aw_dev_info(aw881xx->dev, "%s: profile not change\n", __func__);
	} else {
		ret = aw881xx_fw_update(aw881xx, false);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
			return;
		}
	}

	queue_delayed_work(aw881xx->work_queue,
			&aw881xx->start_work, 0);
}

static void aw881xx_stop(struct aw881xx *aw881xx)
{
	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (aw881xx->status == AW881XX_PW_OFF) {
		aw_dev_dbg(aw881xx->dev, "%s:already power off\n", __func__);
		return;
	}

	aw881xx->status = AW881XX_PW_OFF;

	aw881xx_monitor_stop(&aw881xx->monitor);

	aw881xx_clear_int_status(aw881xx);

	aw881xx_set_intmask(aw881xx, false);

	aw881xx_run_mute(aw881xx, true);

	aw881xx_i2s_tx_enable(aw881xx, false);
	usleep_range(AW_1000_US, AW_1000_US + 100);

	/*enable amppd*/
	aw881xx_run_amppd(aw881xx, true);

	aw881xx_run_pwd(aw881xx, true);
}

static int aw881xx_prof_update(struct aw881xx *aw881xx)
{
	int ret;

	if (aw881xx->allow_pw == false) {
		aw_dev_info(aw881xx->dev, "%s:dev can not allow power\n",
			__func__);
		return 0;
	}

	if (aw881xx->status == AW881XX_PW_ON)
		aw881xx_stop(aw881xx);

	ret = aw881xx_fw_update(aw881xx, false);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
		return ret;
	}

	ret = aw881xx_device_start(aw881xx);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
		return ret;
	} else {
		aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
	}

	aw_dev_info(aw881xx->dev, "%s:update done!\n", __func__);
	return 0;
}

/******************************************************
 *
 * kcontrol
 *
 ******************************************************/
static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 50, 0);

struct soc_mixer_control aw881xx_mixer = {
	.reg = AW881XX_REG_HAGCCFG7,
	.shift = AW881XX_VOL_REG_SHIFT,
	.min = AW881XX_VOLUME_MIN,
	.max = AW881XX_VOLUME_MAX,
};

static int aw881xx_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* set kcontrol info */
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mc->max - mc->min;

	return 0;
}

static int aw881xx_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	aw881xx_get_volume(aw881xx, &value);

	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int aw881xx_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint32_t value = 0;

	/* value is right */
	value = ucontrol->value.integer.value[0];
	if (value > (mc->max - mc->min)) {
		aw_dev_err(aw881xx->dev, "%s:value over range\n", __func__);
		return -EINVAL;
	}

	aw881xx_set_volume(aw881xx, value);

	return 0;
}

static struct snd_kcontrol_new aw881xx_volume = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "aw_dev_0_rx_volume",
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
		SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.tlv.p = (digital_gain),
	.info = aw881xx_volume_info,
	.get = aw881xx_volume_get,
	.put = aw881xx_volume_put,
	.private_value = (unsigned long)&aw881xx_mixer,
};

static int aw881xx_dynamic_create_volume_control(struct aw881xx *aw881xx)
{
	struct snd_kcontrol_new *dst_control = NULL;
	char temp_buf[64] = { 0 };

	dst_control = devm_kzalloc(aw881xx->dev, sizeof(struct snd_kcontrol_new),
				GFP_KERNEL);
	if (dst_control == NULL) {
		aw_dev_err(aw881xx->dev, "%s:devm_kzalloc faild\n", __func__);
		return -ENOMEM;
	}

	memcpy(dst_control, &aw881xx_volume, sizeof(struct snd_kcontrol_new));

	snprintf(temp_buf, sizeof(temp_buf) - 1,
		"aw_dev_%d_rx_volume", aw881xx->channel);
	dst_control->name = aw881xx_devm_kstrdup(aw881xx->dev, temp_buf);
	if (dst_control->name == NULL)
		return -ENOMEM;

	aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
							dst_control, 1);
	return 0;
}

static int aw881xx_profile_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count;
	char *name = NULL;
	const char *prof_name = NULL;
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	count = aw881xx_get_profile_count(aw881xx);
	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		aw_dev_err(aw881xx->dev, "%s:get count[%d] failed\n", __func__, count);
		return 0;
	}

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item > count) {
		uinfo->value.enumerated.item = count - 1;
	}

	name = uinfo->value.enumerated.name;
	count = uinfo->value.enumerated.item;

	prof_name = aw881xx_get_profile_name(aw881xx, count);
	if (prof_name == NULL) {
		strlcpy(uinfo->value.enumerated.name, "null",
			sizeof(uinfo->value.enumerated.name));
		return 0;
	}

	strlcpy(name, prof_name, sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw881xx_profile_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw881xx_get_profile_index(aw881xx);
	aw_dev_dbg(codec->dev, "%s:profile index [%d]\n",
			__func__, aw881xx_get_profile_index(aw881xx));

	return 0;
}

static int aw881xx_profile_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	int ret;

	/* check value valid */
	ret = aw881xx_check_profile_index(aw881xx, ucontrol->value.integer.value[0]);
	if (ret) {
		aw_dev_info(codec->dev, "%s:unsupported index %d\n", __func__,
				(int)ucontrol->value.integer.value[0]);
		return 0;
	}

	if (aw881xx->set_prof == ucontrol->value.integer.value[0]) {
		aw_dev_info(codec->dev, "%s:index no change\n", __func__);
		return 0;
	}

	mutex_lock(&aw881xx->lock);
	aw881xx_set_profile_index(aw881xx, ucontrol->value.integer.value[0]);
	/*pstream = 1 no pcm just set status*/
	if (aw881xx->pstream)
		aw881xx_prof_update(aw881xx);

	mutex_unlock(&aw881xx->lock);
	aw_dev_info(codec->dev, "%s:prof id %d\n", __func__,
			(int)ucontrol->value.integer.value[0]);
	return 0;
}

static int aw881xx_switch_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	int count;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = ARRAY_SIZE(aw881xx_switch);

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item > count)
		uinfo->value.enumerated.item = count - 1;


	strlcpy(uinfo->value.enumerated.name,
		aw881xx_switch[uinfo->value.enumerated.item],
		sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw881xx_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = aw881xx->allow_pw;
	aw_dev_dbg(codec->dev, "%s:allow_pw %d\n",
			__func__, aw881xx->allow_pw);
	return 0;

}

static int aw881xx_switch_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	aw_snd_soc_codec_t *codec =
		aw_componet_codec_ops.aw_snd_soc_kcontrol_codec(kcontrol);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	int ret;

	aw_dev_dbg(codec->dev, "%s:set %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	if (aw881xx->pstream) {
		if (ucontrol->value.integer.value[0] == 0) {
			cancel_delayed_work_sync(&aw881xx->start_work);
			mutex_lock(&aw881xx->lock);
			aw881xx_stop(aw881xx);
			aw881xx->allow_pw = false;
			mutex_unlock(&aw881xx->lock);
			aw_dev_info(aw881xx->dev, "%s:stop pa\n", __func__);
		} else {
			/*if stream have open ,PA can power on*/
			cancel_delayed_work_sync(&aw881xx->start_work);
			if (aw881xx->fw_status == AW881XX_FW_OK) {
				mutex_lock(&aw881xx->lock);
				aw881xx->allow_pw = true;
				ret = aw881xx_fw_update(aw881xx, false);
				if (ret < 0) {
					aw_dev_err(aw881xx->dev, "%s:fw update failed\n", __func__);
					mutex_unlock(&aw881xx->lock);
					return ret;
				}
				ret = aw881xx_device_start(aw881xx);
				if (ret < 0)
					aw_dev_err(aw881xx->dev, "%s:start failed\n", __func__);
				else
					aw_dev_info(aw881xx->dev, "%s:start success\n", __func__);
				mutex_unlock(&aw881xx->lock);
			}
		}
	} else {
		mutex_lock(&aw881xx->lock);
		if (ucontrol->value.integer.value[0]) {
			aw881xx->allow_pw = true;
		} else {
			aw881xx->allow_pw = false;
		}
		mutex_unlock(&aw881xx->lock);
	}

	return 0;
}

static int aw881xx_dynamic_create_common_controls(struct aw881xx *aw881xx)
{
	struct snd_kcontrol_new *aw881xx_dev_control = NULL;
	char *kctl_name = NULL;

	aw881xx_dev_control = devm_kzalloc(aw881xx->codec->dev,
				sizeof(struct snd_kcontrol_new) * AW_CONTROL_NUM,
				GFP_KERNEL);
	if (aw881xx_dev_control == NULL) {
		aw_dev_err(aw881xx->dev, "%s: kcontrol malloc failed!\n", __func__);
		return -ENOMEM;
	}

	kctl_name = devm_kzalloc(aw881xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_prof", aw881xx->channel);

	aw881xx_dev_control[0].name = kctl_name;
	aw881xx_dev_control[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw881xx_dev_control[0].info = aw881xx_profile_info;
	aw881xx_dev_control[0].get = aw881xx_profile_get;
	aw881xx_dev_control[0].put = aw881xx_profile_set;

	kctl_name = devm_kzalloc(aw881xx->codec->dev, AW_NAME_BUF_MAX, GFP_KERNEL);
	if (!kctl_name)
		return -ENOMEM;

	snprintf(kctl_name, AW_NAME_BUF_MAX, "aw_dev_%d_switch", aw881xx->channel);

	aw881xx_dev_control[1].name = kctl_name;
	aw881xx_dev_control[1].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw881xx_dev_control[1].info = aw881xx_switch_info;
	aw881xx_dev_control[1].get = aw881xx_switch_get;
	aw881xx_dev_control[1].put = aw881xx_switch_set;

	aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
					aw881xx_dev_control, AW_CONTROL_NUM);

	return 0;
}

static int aw881xx_dynamic_create_controls(struct aw881xx *aw881xx)
{
	int ret;

	ret = aw881xx_dynamic_create_common_controls(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: create common control failed!\n", __func__);
		return ret;
	}

	ret = aw881xx_dynamic_create_volume_control(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: create volume control failed!\n", __func__);
		return ret;
	}

	return 0;
}

static int aw881xx_get_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_fade_in_time;

	pr_debug("%s:step time %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	return 0;

}

static int aw881xx_set_fade_in_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0] > mc->max) {
		pr_debug("%s:set val %ld overflow %d", __func__,
			ucontrol->value.integer.value[0], mc->max);
		return 0;
	}

	g_fade_in_time = ucontrol->value.integer.value[0];

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int aw881xx_get_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_fade_out_time;

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

static int aw881xx_set_fade_out_time(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0] > mc->max) {
		pr_debug("%s:set val %ld overflow %d\n", __func__,
			ucontrol->value.integer.value[0], mc->max);
		return 0;
	}

	g_fade_out_time = ucontrol->value.integer.value[0];

	pr_debug("%s:step time %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}


static struct snd_kcontrol_new aw881xx_controls[] = {
	SOC_SINGLE_EXT("aw881xx_fadein_us", 0, 0, 1000000, 0,
		aw881xx_get_fade_in_time, aw881xx_set_fade_in_time),
	SOC_SINGLE_EXT("aw881xx_fadeout_us", 0, 0, 1000000, 0,
		aw881xx_get_fade_out_time, aw881xx_set_fade_out_time),
};

static void aw881xx_add_codec_controls(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	if (aw881xx->channel == 0) {
		aw_componet_codec_ops.aw_snd_soc_add_codec_controls(aw881xx->codec,
					aw881xx_controls, ARRAY_SIZE(aw881xx_controls));
	}
}



/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static int aw881xx_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	return 0;
}

static int aw881xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	/*struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec); */

	aw_dev_info(codec->dev, "%s: fmt=0x%x\n", __func__, fmt);

	/* supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
			SND_SOC_DAIFMT_CBS_CFS) {
			aw_dev_err(codec->dev,
					"%s: invalid codec master mode\n",
					__func__);
			return -EINVAL;
		}
		break;
	default:
		aw_dev_err(codec->dev, "%s: unsupported DAI format %d\n",
				__func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	return 0;
}

static int aw881xx_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: freq=%d\n", __func__, freq);

	aw881xx->sysclk = freq;
	return 0;
}

static int aw881xx_update_hw_params(struct aw881xx *aw881xx)
{
	uint16_t reg_val = 0;
	uint32_t cco_mux_value;

	/* match rate */
	switch (aw881xx->rate) {
	case 8000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_8K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 16000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_16K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 32000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_32K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 44100:
		reg_val = AW881XX_BIT_I2SCTRL_SR_44P1K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 48000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_48K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 96000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_96K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	case 192000:
		reg_val = AW881XX_BIT_I2SCTRL_SR_192K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		break;
	default:
		reg_val = AW881XX_BIT_I2SCTRL_SR_48K;
		cco_mux_value = AW881XX_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;
		aw_dev_err(aw881xx->dev, "%s: rate can not support\n",
				__func__);
		break;
	}
	aw881xx_reg_write_bits(aw881xx, AW881XX_REG_PLLCTRL1,
				AW881XX_I2S_CCO_MUX_MASK, cco_mux_value);
	/* set chip rate */
	aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCTRL,
					AW881XX_BIT_I2SCTRL_SR_MASK, reg_val);

	/* match bit width */
	switch (aw881xx->width) {
	case 16:
		reg_val = AW881XX_BIT_I2SCTRL_FMS_16BIT;
		break;
	case 20:
		reg_val = AW881XX_BIT_I2SCTRL_FMS_20BIT;
		break;
	case 24:
		reg_val = AW881XX_BIT_I2SCTRL_FMS_24BIT;
		break;
	case 32:
		reg_val = AW881XX_BIT_I2SCTRL_FMS_32BIT;
		break;
	default:
		reg_val = AW881XX_BIT_I2SCTRL_FMS_16BIT;
		aw_dev_err(aw881xx->dev, "%s: width can not support\n",
				__func__);
		break;
	}
	/* set width */
	aw881xx_reg_write_bits(aw881xx, AW881XX_REG_I2SCTRL,
					AW881XX_BIT_I2SCTRL_FMS_MASK, reg_val);

	return 0;
}

static int aw881xx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_dbg(aw881xx->dev,
				"%s: requested rate: %d, sample size: %d\n",
				__func__, params_rate(params),
				snd_pcm_format_width(params_format(params)));
		return 0;
	}
	/* get rate param */
	aw881xx->rate = params_rate(params);
	aw_dev_dbg(aw881xx->dev, "%s: requested rate: %d, sample size: %d\n",
			__func__, aw881xx->rate,
			snd_pcm_format_width(params_format(params)));

	/* get bit width */
	aw881xx->width = params_width(params);
	aw_dev_dbg(aw881xx->dev, "%s: width = %d\n", __func__, aw881xx->width);

	mutex_lock(&aw881xx->lock);
	aw881xx_update_hw_params(aw881xx);
	mutex_unlock(&aw881xx->lock);

	return 0;
}

static int aw881xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: mute state=%d\n", __func__, mute);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK) {
		aw_dev_info(aw881xx->dev, "%s:capture\n", __func__);
		return 0;
	}

	if (!(aw881xx->flags & AW881XX_FLAG_START_ON_MUTE))
		return 0;

	if (mute) {
		aw881xx->pstream = AW881XX_AUDIO_STOP;
		cancel_delayed_work_sync(&aw881xx->start_work);
		mutex_lock(&aw881xx->lock);
		aw881xx_stop(aw881xx);
		mutex_unlock(&aw881xx->lock);
	} else {
		aw881xx->pstream = AW881XX_AUDIO_START;
		mutex_lock(&aw881xx->lock);
		aw881xx_start(aw881xx);
		mutex_unlock(&aw881xx->lock);
	}

	return 0;
}

static void aw881xx_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);

	aw_dev_info(codec->dev, "%s:enter\n", __func__);
}

static const struct snd_soc_dai_ops aw881xx_dai_ops = {
	.startup = aw881xx_startup,
	.set_fmt = aw881xx_set_fmt,
	.set_sysclk = aw881xx_set_dai_sysclk,
	.hw_params = aw881xx_hw_params,
	.mute_stream = aw881xx_mute,
	.shutdown = aw881xx_shutdown,
};

static struct snd_soc_dai_driver aw881xx_dai[] = {
	{
	.name = "aw881xx-aif",
	.id = 1,
	.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW881XX_RATES,
			.formats = AW881XX_FORMATS,
			},
	 .capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW881XX_RATES,
			.formats = AW881XX_FORMATS,
			},
	.ops = &aw881xx_dai_ops,
	.symmetric_rates = 1,
	},
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static uint8_t aw881xx_crc8_check(unsigned char *data , uint32_t data_size)
{
	uint8_t crc_value = 0x00;
	uint8_t pdatabuf = 0;
	int i;

	while (data_size--) {
		pdatabuf = *data++;
		for (i = 0; i < 8; i++) {
			/*if the lowest bit is 1*/
			if ((crc_value ^ (pdatabuf)) & 0x01) {
				/*Xor multinomial*/
				crc_value ^= 0x18;
				crc_value >>= 1;
				crc_value |= 0x80;
			} else {
				crc_value >>= 1;
			}
			pdatabuf >>= 1;
		}
	}
	return crc_value;
}

static int aw881xx_check_cfg_by_hdr(struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;
	struct aw_cfg_dde *cfg_dde = NULL;
	unsigned int end_data_offset = 0;
	unsigned int act_data = 0;
	uint8_t act_crc8 = 0;
	uint32_t hdr_ddt_len = 0;
	int i;

	if (aw_cfg == NULL) {
		pr_err("%s:aw_prof is NULL\n", __func__);
		return -ENOMEM;
	}

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;

	/*check file type id is awinic acf file*/
	if (cfg_hdr->a_id != ACF_FILE_ID) {
		pr_err("%s:not acf type file\n", __func__);
		return -EINVAL;
	}

	hdr_ddt_len = cfg_hdr->a_hdr_offset + cfg_hdr->a_ddt_size;
	if (hdr_ddt_len > aw_cfg->len) {
		pr_err("%s:hdrlen with ddt_len [%d] overflow file size[%d]\n",
			__func__, cfg_hdr->a_hdr_offset, aw_cfg->len);
		return -EINVAL;
	}

	/*check data size*/
	cfg_dde = (struct aw_cfg_dde *)((char *)aw_cfg->data + cfg_hdr->a_hdr_offset);
	act_data += hdr_ddt_len;
	for (i = 0; i < cfg_hdr->a_ddt_num; i++)
		act_data += cfg_dde[i].data_size;

	if (act_data != aw_cfg->len) {
		pr_err("%s:act_data[%d] not equal to file size[%d]!\n",
			__func__, act_data, aw_cfg->len);
		return -EINVAL;
	}

	for (i = 0; i < cfg_hdr->a_ddt_num; i++) {
		/* data check */
		end_data_offset = cfg_dde[i].data_offset + cfg_dde[i].data_size;
		if (end_data_offset > aw_cfg->len) {
			pr_err("%s:a_ddt_num[%d] end_data_offset[%d] overflow file size[%d]\n",
				__func__, i, end_data_offset, aw_cfg->len);
			return -EINVAL;
		}

		/* crc check */
		act_crc8 = aw881xx_crc8_check(aw_cfg->data + cfg_dde[i].data_offset, cfg_dde[i].data_size);
		if (act_crc8 != cfg_dde[i].data_crc) {
			pr_err("%s:a_ddt_num[%d] crc8 check failed, act_crc8:0x%x != data_crc 0x%x\n",
				__func__, i, (uint32_t)act_crc8, cfg_dde[i].data_crc);
			return -EINVAL;
		}
	}

	pr_info("%s:project name [%s]\n", __func__, cfg_hdr->a_project);
	pr_info("%s:custom name [%s]\n", __func__, cfg_hdr->a_custom);
	pr_info("%s:version name [%d.%d.%d.%d]\n", __func__,
			cfg_hdr->a_version[3], cfg_hdr->a_version[2],
			cfg_hdr->a_version[1], cfg_hdr->a_version[0]);
	pr_info("%s:author id %d\n", __func__, cfg_hdr->a_author_id);

	return 0;
}

int aw881xx_load_acf_check(struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;

	if (aw_cfg == NULL) {
		pr_err("%s: aw_prof is NULL\n", __func__);
		return -ENOMEM;
	}

	if (aw_cfg->len < sizeof(struct aw_cfg_hdr)) {
		pr_err("%s:cfg hdr size[%d] overflow file size[%d]",
			__func__, (int)sizeof(struct aw_cfg_hdr), aw_cfg->len);
		return -EINVAL;
	}

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	switch (cfg_hdr->a_hdr_version) {
	case AW_CFG_HDR_VER_0_0_0_1:
		return aw881xx_check_cfg_by_hdr(aw_cfg);
	default:
		pr_err("%s: unsupported hdr_version [0x%x]\n",
				__func__, cfg_hdr->a_hdr_version);
		return -EINVAL;
	}

	return 0;
}

static void aw881xx_update_dsp_data_order(struct aw881xx *aw881xx,
						uint8_t *data, uint32_t data_len)
{
	int i = 0;
	uint8_t tmp_val = 0;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	for (i = 0; i < data_len; i += 2) {
		tmp_val = data[i];
		data[i] = data[i + 1];
		data[i + 1] = tmp_val;
	}
}

static int aw881xx_parse_raw_reg(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_REG].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_REG].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_raw_dsp_cfg(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	aw881xx_update_dsp_data_order(aw881xx, data, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_DSP_CFG].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_DSP_CFG].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_raw_dsp_fw(struct aw881xx *aw881xx,
		uint8_t *data, uint32_t data_len, struct aw_prof_desc *prof_desc)
{
	aw_dev_info(aw881xx->dev, "%s:data_size:%d enter\n", __func__, data_len);

	aw881xx_update_dsp_data_order(aw881xx, data, data_len);

	prof_desc->sec_desc[AW_DATA_TYPE_DSP_FW].data = data;
	prof_desc->sec_desc[AW_DATA_TYPE_DSP_FW].len = data_len;

	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw881xx_parse_data_by_sec_type(struct aw881xx *aw881xx, struct aw_cfg_hdr *cfg_hdr,
			struct aw_cfg_dde *cfg_dde, struct aw_prof_desc *scene_prof_desc)
{
	switch (cfg_dde->data_type) {
	case ACF_SEC_TYPE_REG:
		return aw881xx_parse_raw_reg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_CFG:
		return aw881xx_parse_raw_dsp_cfg(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_DSP_FW:
		return aw881xx_parse_raw_dsp_fw(aw881xx,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size, scene_prof_desc);
	case ACF_SEC_TYPE_MONITOR:
		return aw_monitor_parse_fw(&aw881xx->monitor,
				(uint8_t *)cfg_hdr + cfg_dde->data_offset,
				cfg_dde->data_size);
	}

	return 0;
}

static int aw881xx_parse_dev_type(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int i = 0;
	int ret;
	int sec_num = 0;
	struct aw_cfg_dde *cfg_dde =
		(struct aw_cfg_dde *)((char *)prof_hdr + prof_hdr->a_hdr_offset);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((aw881xx->i2c->adapter->nr == cfg_dde[i].dev_bus) &&
			(aw881xx->i2c->addr == cfg_dde[i].dev_addr) &&
			(cfg_dde[i].type == AW_DEV_TYPE_ID)) {
			ret = aw881xx_parse_data_by_sec_type(aw881xx, prof_hdr, &cfg_dde[i],
					&all_prof_info->prof_desc[cfg_dde[i].dev_profile]);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "%s:parse dev type failed\n", __func__);
				return ret;
			}
			sec_num++;
		}
	}

	if (sec_num == 0) {
		aw_dev_info(aw881xx->dev, "%s:get dev type num is %d, please use default\n",
					__func__, sec_num);
		return AW_DEV_TYPE_NONE;
	}

	return AW_DEV_TYPE_OK;
}

static int aw881xx_parse_dev_default_type(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int i = 0;
	int ret;
	int sec_num = 0;
	struct aw_cfg_dde *cfg_dde =
		(struct aw_cfg_dde *)((char *)prof_hdr + prof_hdr->a_hdr_offset);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	for (i = 0; i < prof_hdr->a_ddt_num; i++) {
		if ((aw881xx->channel == cfg_dde[i].dev_index) &&
			(cfg_dde[i].type == AW_DEV_DEFAULT_TYPE_ID)) {
			ret = aw881xx_parse_data_by_sec_type(aw881xx, prof_hdr, &cfg_dde[i],
					&all_prof_info->prof_desc[cfg_dde[i].dev_profile]);
			if (ret < 0) {
				aw_dev_err(aw881xx->dev, "%s: parse default type failed\n", __func__);
				return ret;
			}
			sec_num++;
		}
	}

	if (sec_num == 0) {
		aw_dev_err(aw881xx->dev, "%s: get dev default type failed, get num[%d]\n",
					__func__, sec_num);
		return -EINVAL;
	}

	return 0;
}

static int aw881xx_load_cfg_by_hdr(struct aw881xx *aw881xx,
		struct aw_cfg_hdr *prof_hdr, struct aw_all_prof_info *all_prof_info)
{
	int ret;

	ret = aw881xx_parse_dev_type(aw881xx, prof_hdr, all_prof_info);
	if (ret < 0) {
		return ret;
	} else if (ret == AW_DEV_TYPE_NONE) {
		aw_dev_info(aw881xx->dev, "%s:get dev type num is 0, parse default dev type\n",
					__func__);
		ret = aw881xx_parse_dev_default_type(aw881xx, prof_hdr, all_prof_info);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int aw881xx_get_vaild_prof(struct aw881xx *aw881xx,
				struct aw_all_prof_info all_prof_info)
{
	int i;
	int num = 0;
	struct aw_sec_data_desc *sec_desc = NULL;
	struct aw_prof_desc *prof_desc = all_prof_info.prof_desc;
	struct aw_prof_info *prof_info = &aw881xx->prof_info;

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (prof_desc[i].prof_st == AW_PROFILE_OK) {
			sec_desc = prof_desc[i].sec_desc;
			if ((sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_REG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].len != 0)) {
				prof_info->count++;
			}
		}
	}

	aw_dev_info(aw881xx->dev, "%s: get vaild profile:%d\n",
			__func__, aw881xx->prof_info.count);

	if (!prof_info->count) {
		aw_dev_err(aw881xx->dev, "%s:no profile data\n", __func__);
		return -EPERM;
	}

	prof_info->prof_desc = devm_kzalloc(aw881xx->dev,
				prof_info->count * sizeof(struct aw_prof_desc),
				GFP_KERNEL);
	if (prof_info->prof_desc == NULL) {
		aw_dev_err(aw881xx->dev, "%s:prof_desc kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (prof_desc[i].prof_st == AW_PROFILE_OK) {
			sec_desc = prof_desc[i].sec_desc;
			if ((sec_desc[AW_DATA_TYPE_REG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_REG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_CFG].len != 0) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].data != NULL) &&
				(sec_desc[AW_DATA_TYPE_DSP_FW].len != 0)) {
				if (num >= prof_info->count) {
					aw_dev_err(aw881xx->dev, "%s:get scene num[%d] overflow count[%d]\n",
						__func__, num, prof_info->count);
					return -ENOMEM;
				}
				prof_info->prof_desc[num] = prof_desc[i];
				prof_info->prof_desc[num].id = i;
				num++;
			}
		}
	}

	return 0;
}

static int aw881xx_cfg_load(struct aw881xx *aw881xx,
				struct aw881xx_container *aw_cfg)
{
	struct aw_cfg_hdr *cfg_hdr = NULL;
	struct aw_all_prof_info all_prof_info;
	int ret;

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	memset(&all_prof_info, 0, sizeof(struct aw_all_prof_info));

	cfg_hdr = (struct aw_cfg_hdr *)aw_cfg->data;
	switch (cfg_hdr->a_hdr_version) {
	case AW_CFG_HDR_VER_0_0_0_1:
		ret = aw881xx_load_cfg_by_hdr(aw881xx, cfg_hdr, &all_prof_info);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:hdr_cersion[0x%x] parse failed\n",
					__func__, cfg_hdr->a_hdr_version);
			return ret;
		}
		break;
	default:
		aw_dev_err(aw881xx->dev, "%s:unsupported hdr_version [0x%x]\n",
				__func__, cfg_hdr->a_hdr_version);
		return -EINVAL;
	}

	ret = aw881xx_get_vaild_prof(aw881xx, all_prof_info);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:get vaild profile failed\n", __func__);
		return ret;
	}
	aw881xx->fw_status = AW881XX_FW_OK;
	aw_dev_info(aw881xx->dev, "%s:parse cfg success\n", __func__);
	return 0;
}

void aw881xx_device_deinit(struct aw881xx *aw881xx)
{
	aw_dev_dbg(aw881xx->dev, "%s:enter\n", __func__);

	if (aw881xx->prof_info.prof_desc != NULL) {
		devm_kfree(aw881xx->dev, aw881xx->prof_info.prof_desc);
		aw881xx->prof_info.prof_desc = NULL;
	}

	aw881xx->prof_info.count = 0;
}

int aw881xx_device_init(struct aw881xx *aw881xx, struct aw881xx_container *aw_cfg)
{
	int ret;

	if (aw881xx == NULL || aw_cfg == NULL) {
		pr_err("%s:aw881xx is NULL or aw_cfg is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = aw881xx_cfg_load(aw881xx, aw_cfg);
	if (ret < 0) {
		aw881xx_device_deinit(aw881xx);
		aw_dev_err(aw881xx->dev, "%s:aw_dev acf parse failed\n", __func__);
		return -EINVAL;
	}

	aw881xx->cur_prof = aw881xx->prof_info.prof_desc[0].id;
	aw881xx->set_prof = aw881xx->prof_info.prof_desc[0].id;

	ret = aw881xx_fw_update(aw881xx, true);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: fw update failed\n", __func__);
		return ret;
	}

	aw881xx->status = AW881XX_PW_ON;
	aw881xx_stop(aw881xx);

	aw_dev_info(aw881xx->dev, "%s:init done\n", __func__);

	return 0;
}

static void aw881xx_chip_profile_loaded(const struct firmware *cont, void *context)
{
	struct aw881xx *aw881xx = context;
	struct aw881xx_container *aw_cfg = NULL;
	int ret = -1;

	aw881xx->fw_status = AW881XX_FW_FAILED;
	if (!cont) {
		aw_dev_err(aw881xx->dev, "%s:failed to read %s\n",
			__func__, AW881XX_ACF_FILE);
		release_firmware(cont);
		return;
	}

	aw_dev_info(aw881xx->dev, "%s:loaded %s - size: %zu\n",
			__func__, AW881XX_ACF_FILE, cont ? cont->size : 0);

	mutex_lock(&g_aw881xx_lock);
	if (g_awinic_cfg == NULL) {
		aw_cfg = vmalloc(cont->size + sizeof(uint32_t));
		memset(aw_cfg, 0, sizeof(struct aw881xx_container));
		if (aw_cfg == NULL) {
			aw_dev_err(aw881xx->dev, "%s:aw_cfg kzalloc failed\n",
						__func__);
			release_firmware(cont);
			mutex_unlock(&g_aw881xx_lock);
			return;
		}
		aw_cfg->len = cont->size;
		memcpy(aw_cfg->data, cont->data, cont->size);
		ret = aw881xx_load_acf_check(aw_cfg);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:Load [%s] failed ....!\n",
					__func__, AW881XX_ACF_FILE);
			vfree(aw_cfg);
			aw_cfg = NULL;
			release_firmware(cont);
			mutex_unlock(&g_aw881xx_lock);
			return;
		}
		g_awinic_cfg = aw_cfg;
	} else {
		aw_cfg = g_awinic_cfg;
		aw_dev_info(aw881xx->dev, "%s:[%s] already loaded...\n",
				__func__, AW881XX_ACF_FILE);
	}
	release_firmware(cont);
	mutex_unlock(&g_aw881xx_lock);

	mutex_lock(&aw881xx->lock);
	/*aw device init*/
	ret = aw881xx_device_init(aw881xx, aw_cfg);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s:dev init failed\n", __func__);
		mutex_unlock(&aw881xx->lock);
		return;
	}

	aw881xx_dynamic_create_controls(aw881xx);

	mutex_unlock(&aw881xx->lock);

	return;

}

static int aw881xx_load_chip_profile(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter, start load %s\n", __func__, AW881XX_ACF_FILE);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					AW881XX_ACF_FILE,
					aw881xx->dev, GFP_KERNEL, aw881xx,
					aw881xx_chip_profile_loaded);

}

static void aw881xx_request_firmware_file(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	aw881xx_load_chip_profile(aw881xx);

}

static void aw881xx_load_fw_work(struct work_struct *work)
{
	struct aw881xx *aw881xx = container_of(work, struct aw881xx, load_fw_work.work);

	aw_dev_info(aw881xx->dev, "%s:enter\n", __func__);

	aw881xx_request_firmware_file(aw881xx);
}

static int aw881xx_codec_probe(aw_snd_soc_codec_t *codec)
{
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx->codec = codec;

	aw881xx->work_queue = create_singlethread_workqueue("aw881xx");
	if (aw881xx->work_queue == NULL) {
		aw_dev_err(aw881xx->dev, "%s:create workqueue failed!\n",
			__func__);
		return -EINVAL;
	}
	INIT_DELAYED_WORK(&aw881xx->start_work, aw881xx_startup_work);
	INIT_DELAYED_WORK(&aw881xx->load_fw_work, aw881xx_load_fw_work);

	aw881xx_add_codec_controls(aw881xx);

	queue_delayed_work(aw881xx->work_queue,
			&aw881xx->load_fw_work,
			msecs_to_jiffies(AW_LOAD_BIN_TIME_MS));

	aw_dev_info(aw881xx->dev, "%s: exit\n", __func__);

	return 0;
}

#ifdef AW_KERNEL_VER_OVER_4_19_1
static void aw881xx_codec_remove(struct snd_soc_component *component)
{
	/*struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);*/

	aw_dev_info(component->dev, "%s: enter\n", __func__);

	return;
}
#else
static int aw881xx_codec_remove(struct snd_soc_codec *codec)
{
	/*struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);*/

	aw_dev_info(codec->dev, "%s: enter\n", __func__);

	return 0;
}
#endif

static unsigned int aw881xx_codec_read(aw_snd_soc_codec_t *codec,
					unsigned int reg)
{
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint16_t value = 0;
	int ret = -1;

	aw_dev_dbg(aw881xx->dev, "%s: enter\n", __func__);

	if (aw881xx_reg_access[reg] & REG_RD_ACCESS) {
		ret = aw881xx_reg_read(aw881xx, reg, &value);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: read register failed\n",
					__func__);
			return ret;
		}
	} else {
		aw_dev_dbg(aw881xx->dev, "%s: register 0x%x no read access\n",
				__func__, reg);
		return ret;
	}
	return ret;
}

static int aw881xx_codec_write(aw_snd_soc_codec_t *codec,
				unsigned int reg, unsigned int value)
{
	int ret = -1;
	struct aw881xx *aw881xx =
		aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_dbg(aw881xx->dev, "%s: enter, reg is 0x%x value is 0x%x\n",
			__func__, reg, value);

	if (aw881xx_reg_access[reg] & REG_WR_ACCESS)
		ret = aw881xx_reg_write(aw881xx, (uint8_t) reg,
					(uint16_t) value);
	else
		aw_dev_dbg(aw881xx->dev, "%s: register 0x%x no write access\n",
				__func__, reg);

	return ret;
}

#ifdef AW_KERNEL_VER_OVER_4_19_1
static struct snd_soc_component_driver soc_codec_dev_aw881xx = {
	.probe = aw881xx_codec_probe,
	.remove = aw881xx_codec_remove,
	.read = aw881xx_codec_read,
	.write = aw881xx_codec_write,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw881xx = {
	.probe = aw881xx_codec_probe,
	.remove = aw881xx_codec_remove,
	.read = aw881xx_codec_read,
	.write = aw881xx_codec_write,
	.reg_cache_size = AW881XX_REG_MAX,
	.reg_word_size = 2,
	.component_driver = {
	.dapm_widgets = aw881xx_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw881xx_dapm_widgets),
	.dapm_routes = aw881xx_audio_map,
	.num_dapm_routes = ARRAY_SIZE(aw881xx_audio_map),
	},
};
#endif

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw881xx_interrupt_setup(struct aw881xx *aw881xx)
{
	uint16_t reg_val = 0;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINTM, &reg_val);
	reg_val &= (~AW881XX_BIT_SYSINTM_PLLM);
	reg_val &= (~AW881XX_BIT_SYSINTM_OTHM);
	reg_val &= (~AW881XX_BIT_SYSINTM_OCDM);
	aw881xx_reg_write(aw881xx, AW881XX_REG_SYSINTM, reg_val);
}

static void aw881xx_interrupt_handle(struct aw881xx *aw881xx)
{
	uint16_t reg_val = 0;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSST, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSST=0x%x\n", __func__, reg_val);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINT, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSINT=0x%x\n", __func__, reg_val);

	aw881xx_reg_read(aw881xx, AW881XX_REG_SYSINTM, &reg_val);
	aw_dev_info(aw881xx->dev, "%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw881xx_irq(int irq, void *data)
{
	struct aw881xx *aw881xx = data;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	aw881xx_interrupt_handle(aw881xx);

	aw_dev_info(aw881xx->dev, "%s: exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw881xx_parse_gpio_dt(struct aw881xx *aw881xx,
				 struct device_node *np)
{
	aw881xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw881xx->reset_gpio < 0) {
		aw_dev_err(aw881xx->dev,
			"%s: no reset gpio provided, will not hw reset\n",
			__func__);
		return -EIO;
	} else {
		aw_dev_info(aw881xx->dev, "%s: reset gpio provided ok\n",
			__func__);
	}

	aw881xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw881xx->irq_gpio < 0)
		aw_dev_info(aw881xx->dev,
		"%s: no irq gpio provided.\n", __func__);
	else
		aw_dev_info(aw881xx->dev,
		"%s: irq gpio provided ok.\n", __func__);

	return 0;
}

static void aw881xx_parse_channel_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t channel_value;

	ret = of_property_read_u32(np, "sound-channel", &channel_value);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read sound-channel failed, use default: 0\n",
			__func__);
		channel_value = AW881XX_CHAN_VAL_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read sound-channel value is : %d\n",
			__func__, channel_value);
	}

	aw881xx->channel = channel_value;
}


static void aw881xx_parse_pa_sync_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t pa_syn_en;

	ret = of_property_read_u32(np, "pa-syn-enable", &pa_syn_en);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read pa-syn-enable failed, use default: 0\n",
			__func__);
		pa_syn_en = AW881XX_PA_SYNC_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read a-syn-enable value is: %d\n",
			__func__, pa_syn_en);
	}

	aw881xx->pa_syn_en = pa_syn_en;
}

static void aw881xx_parse_fade_enable_dt(struct aw881xx *aw881xx,
					struct device_node *np)
{
	int ret = -1;
	uint32_t fade_en;

	ret = of_property_read_u32(np, "fade-enable", &fade_en);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev,
			"%s:read fade-enable failed, use default: 0\n",
			__func__);
		fade_en = AW881XX_FADE_IN_OUT_DEFAULT;
	} else {
		aw_dev_info(aw881xx->dev, "%s: read fade-enable value is: %d\n",
			__func__, fade_en);
	}

	aw881xx->fade_en = fade_en;
}

static int aw881xx_parse_dt(struct device *dev, struct aw881xx *aw881xx,
				struct device_node *np)
{
	int ret;

	aw881xx_parse_channel_dt(aw881xx, np);
	aw881xx_parse_pa_sync_dt(aw881xx, np);
	aw881xx_parse_fade_enable_dt(aw881xx, np);
	ret = aw881xx_parse_gpio_dt(aw881xx, np);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw881xx_hw_reset(struct aw881xx *aw881xx)
{
	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	if (gpio_is_valid(aw881xx->reset_gpio)) {
		gpio_set_value_cansleep(aw881xx->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw881xx->reset_gpio, 1);
		msleep(1);
	} else {
		aw_dev_err(aw881xx->dev, "%s: failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static void aw881xx_ui_update_cfg_name(struct aw881xx *aw881xx)
{
	char aw881xx_head[] = { "aw881xx_pid_" };
	char buf[20] = { 0 };
	uint8_t i = 0;
	uint8_t head_index = 0;
	int i2cbus = aw881xx->i2c->adapter->nr;
	int addr = aw881xx->i2c->addr;

	memcpy(aw881xx->ui_cfg_name, aw881xx_cfg_name, sizeof(aw881xx_cfg_name));
	head_index = sizeof(aw881xx_head) - 1;

	/*add product information*/
	snprintf(buf, sizeof(buf), "%02x", aw881xx->pid);
	for (i = 0; i < AW_UI_MODE_MAX; i++)
		memcpy(aw881xx->ui_cfg_name[i] + head_index, buf, strlen(buf));

	/*add sound channel information*/
	snprintf(buf, sizeof(buf), "_%x_%x.bin", i2cbus, addr);
	for (i = 0; i < AW_UI_MODE_MAX; i++) {
		head_index = strlen(aw881xx->ui_cfg_name[i]);
		memcpy(aw881xx->ui_cfg_name[i] + head_index, buf, strlen(buf) + 1);
	}

	for (i = 0; i < AW_UI_MODE_MAX; i++)
		aw_dev_dbg(aw881xx->dev, "%s: id[%d], [%s]\n",
				__func__, i, aw881xx->ui_cfg_name[i]);
}

static int aw881xx_read_dsp_pid(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_REG_DSP_PID, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read AW881XX_REG_DSP_PID: %d\n",
				__func__, ret);
		return -EIO;
	}

	switch (reg_val) {
	case AW881XX_DSP_PID_01:
		aw881xx->pid = AW881XX_PID_01;
		break;
	case AW881XX_DSP_PID_03:
		aw881xx->pid = AW881XX_PID_03;
		break;
	default:
		aw_dev_info(aw881xx->dev, "%s: unsupported dsp_pid=0x%04x\n",
				__func__, reg_val);
		return -EIO;
	}

	aw881xx_ui_update_cfg_name(aw881xx);

	return 0;
}

static int aw881xx_read_product_id(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t reg_val = 0;
	uint16_t product_id = 0;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_PRODUCT_ID, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read REG_EFRL: %d\n",
			__func__, ret);
		return -EIO;
	}

	product_id = reg_val & (~AW881XX_BIT_PRODUCT_ID_MASK);
	switch (product_id) {
	case AW881XX_PID_01:
		aw881xx->pid = AW881XX_PID_01;
		break;
	case AW881XX_PID_03:
		aw881xx->pid = AW881XX_PID_03;
		break;
	default:
		aw881xx->pid = AW881XX_PID_03;
	}

	aw881xx_ui_update_cfg_name(aw881xx);

	return 0;
}

static int aw881xx_read_chipid(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint16_t reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw881xx_reg_read(aw881xx, AW881XX_REG_ID, &reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev,
				"%s: failed to read chip id, ret=%d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_val) {
		case AW881XX_CHIPID:
			aw881xx->chipid = AW881XX_CHIPID;
			aw881xx->flags |= AW881XX_FLAG_START_ON_MUTE;
			aw881xx->flags |= AW881XX_FLAG_SKIP_INTERRUPTS;

			aw881xx_read_product_id(aw881xx);

			aw_dev_info(aw881xx->dev,
				"%s: chipid=0x%04x, product_id=0x%02x\n",
				__func__, aw881xx->chipid, aw881xx->pid);
			return 0;
		default:
			aw_dev_info(aw881xx->dev,
				"%s: unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;
		usleep_range(AW_5000_US, AW_5000_US + 100);
	}

	return -EINVAL;
}


static int aw881xx_ui_dsp_cfg_loaded(struct aw881xx *aw881xx)
{
	struct aw881xx_container *aw881xx_cfg = NULL;
	const struct firmware *cont = NULL;
	int ret = -1;

	ret = request_firmware(&cont, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE],
				aw881xx->dev);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read %s\n",
			__func__, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE]);
		release_firmware(cont);
		return ret;
	}

	aw_dev_info(aw881xx->dev, "%s: loaded %s - size: %zu\n",
		__func__, aw881xx->ui_cfg_name[AW_UI_DSP_CFG_MODE],
				cont ? cont->size : 0);

	aw881xx_cfg = devm_kzalloc(aw881xx->dev,
			cont->size + sizeof(uint32_t), GFP_KERNEL);
	if (aw881xx_cfg == NULL) {
		aw_dev_err(aw881xx->dev, "%s: alloc failed!\n", __func__);
		release_firmware(cont);
		return ret;
	}
	aw881xx_cfg->len = cont->size;
	memcpy(aw881xx_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	aw881xx_update_dsp_data_order(aw881xx,
			aw881xx_cfg->data, aw881xx_cfg->len);

	aw881xx_dsp_cfg_update(aw881xx,
		aw881xx_cfg->data, aw881xx_cfg->len);

	devm_kfree(aw881xx->dev, aw881xx_cfg);
	aw881xx_cfg = NULL;

	return 0;
}


static int aw881xx_ui_dsp_fw_loaded(struct aw881xx *aw881xx)
{
	struct aw881xx_container *aw881xx_cfg = NULL;
	const struct firmware *cont = NULL;
	int ret = -1;

	ret = request_firmware(&cont, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE], aw881xx->dev);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: failed to read %s\n",
				__func__, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE]);
		release_firmware(cont);
		return ret;
	}

	aw_dev_info(aw881xx->dev, "%s: loaded %s - size: %zu\n",
			__func__, aw881xx->ui_cfg_name[AW_UI_DSP_FW_MODE],
			cont ? cont->size : 0);

	aw881xx_cfg = devm_kzalloc(aw881xx->dev,
			cont->size + sizeof(uint32_t), GFP_KERNEL);
	if (aw881xx_cfg == NULL) {
		aw_dev_err(aw881xx->dev, "%s: devm_kzalloc failed!\n", __func__);
		release_firmware(cont);
		return -ENOMEM;
	}
	aw881xx_cfg->len = cont->size;
	memcpy(aw881xx_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	aw881xx_update_dsp_data_order(aw881xx,
				aw881xx_cfg->data, aw881xx_cfg->len);
	aw881xx_dsp_fw_update(aw881xx, aw881xx_cfg->data, aw881xx_cfg->len);

	devm_kfree(aw881xx->dev, aw881xx_cfg);
	aw881xx_cfg = NULL;

	return 0;
}

static int aw881xx_ui_update_dsp(struct aw881xx *aw881xx)
{
	int ret = -1;

	aw_dev_info(aw881xx->dev, "%s: enter\n", __func__);

	ret = aw881xx_syspll_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: syspll check failed\n", __func__);
		return ret;
	}

	aw881xx_get_dsp_config(aw881xx);
	aw881xx_read_dsp_pid(aw881xx);

	aw881xx_run_mute(aw881xx, true);
	aw881xx_dsp_enable(aw881xx, false);
	aw881xx_memclk_select(aw881xx, AW881XX_MEMCLK_PLL);

	ret = aw881xx_ui_dsp_fw_loaded(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp fw loading failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = aw881xx_ui_dsp_cfg_loaded(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp cfg loading failed: %d\n",
			__func__, ret);
		return ret;
	}

	ret = aw881xx_sysst_check(aw881xx);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s: sysst check fail\n",
				__func__);
		return ret;
	}

	ret = aw881xx_dsp_check(aw881xx);
	if (ret < 0) {
		aw_dev_info(aw881xx->dev, "%s: dsp cfg update error\n",
				__func__);
		return ret;
	}

	aw881xx->status = AW881XX_PW_OFF;
	ret = aw881xx_device_start(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: start fail, ret=%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

/******************************************************
 *
 * sys group attribute
 *
 ******************************************************/
static ssize_t aw881xx_reg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw881xx_reg_write(aw881xx, databuf[0], databuf[1]);

	return count;
}

static ssize_t aw881xx_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint16_t reg_val = 0;

	for (i = 0; i < AW881XX_REG_MAX; i++) {
		if (aw881xx_reg_access[i] & REG_RD_ACCESS) {
			aw881xx_reg_read(aw881xx, i, &reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}
	return len;
}

static ssize_t aw881xx_rw_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw881xx->reg_addr = (uint8_t) databuf[0];
		aw881xx_reg_write(aw881xx, databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw881xx->reg_addr = (uint8_t) databuf[0];
	}

	return count;
}

static ssize_t aw881xx_rw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t reg_val = 0;

	if (aw881xx_reg_access[aw881xx->reg_addr] & REG_RD_ACCESS) {
		aw881xx_reg_read(aw881xx, aw881xx->reg_addr, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%04x\n", aw881xx->reg_addr,
				reg_val);
	}
	return len;
}

static ssize_t aw881xx_dsp_rw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t reg_val = 0;

	mutex_lock(&aw881xx->i2c_lock);
	aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD, aw881xx->dsp_addr);
	aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"dsp:0x%04x=0x%04x\n", aw881xx->dsp_addr, reg_val);
	aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"dsp:0x%04x=0x%04x\n", aw881xx->dsp_addr + 1, reg_val);
	mutex_unlock(&aw881xx->i2c_lock);

	return len;
}

static ssize_t aw881xx_dsp_rw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw881xx->dsp_addr = (unsigned int)databuf[0];
		aw881xx_dsp_write(aw881xx, databuf[0], databuf[1]);
		aw_dev_dbg(aw881xx->dev, "%s: get param: %x %x\n", __func__,
				databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw881xx->dsp_addr = (unsigned int)databuf[0];
		aw_dev_dbg(aw881xx->dev, "%s: get param: %x\n", __func__,
				databuf[0]);
	}

	return count;
}

static ssize_t aw881xx_dsp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	unsigned int val = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	aw_dev_dbg(aw881xx->dev, "%s: value=%d\n", __func__, val);

	if (val) {
		cancel_delayed_work_sync(&aw881xx->start_work);

		mutex_lock(&aw881xx->lock);
		aw881xx_ui_update_dsp(aw881xx);
		mutex_unlock(&aw881xx->lock);
	}

	return count;
}

static ssize_t aw881xx_dsp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	uint16_t reg_val = 0;
	int ret = -1;

	aw881xx_get_dsp_config(aw881xx);
	if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
		len += snprintf((char *)(buf + len), PAGE_SIZE - len,
				"%s: aw881xx dsp bypass\n", __func__);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"aw881xx dsp working\n");
		ret = aw881xx_get_iis_status(aw881xx);
		if (ret < 0) {
			len += snprintf((char *)(buf + len),
					PAGE_SIZE - len,
					"%s: aw881xx no iis signal\n",
					__func__);
		} else {
			aw_dev_dbg(aw881xx->dev, "%s: aw881xx_dsp_firmware:\n",
					__func__);
			mutex_lock(&aw881xx->i2c_lock);
			aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD,
					AW881XX_DSP_FW_ADDR);
			for (i = 0; i < aw881xx->dsp_fw_len; i += 2) {
				aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
				aw_dev_dbg(aw881xx->dev,
					"%s: dsp_fw[0x%04x]:0x%02x,0x%02x\n",
					__func__, i, (reg_val >> 0) & 0xff,
					(reg_val >> 8) & 0xff);
			}
			aw_dev_dbg(aw881xx->dev, "\n");

			aw_dev_dbg(aw881xx->dev, "%s: aw881xx_dsp_cfg:\n",
					__func__);
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw881xx dsp config:\n");
			aw881xx_i2c_write(aw881xx, AW881XX_REG_DSPMADD,
					AW881XX_DSP_CFG_ADDR);
			for (i = 0; i < aw881xx->dsp_cfg_len; i += 2) {
				aw881xx_i2c_read(aw881xx, AW881XX_REG_DSPMDAT, &reg_val);
				len += snprintf(buf + len, PAGE_SIZE - len,
					"%02x,%02x,", (reg_val >> 0) & 0xff,
					(reg_val >> 8) & 0xff);
				aw_dev_dbg(aw881xx->dev,
					"%s: dsp_cfg[0x%04x]:0x%02x,0x%02x\n",
					__func__, i, (reg_val >> 0) & 0xff,
						(reg_val >> 8) & 0xff);
				if ((i / 2 + 1) % 8 == 0) {
					len += snprintf(buf + len,
							PAGE_SIZE - len, "\n");
				}
			}
			mutex_unlock(&aw881xx->i2c_lock);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
		}
	}

	return len;
}

static ssize_t aw881xx_dsp_fw_ver_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	/*struct aw881xx *aw881xx = dev_get_drvdata(dev);*/

	return count;
}

static ssize_t aw881xx_dsp_fw_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t dsp_addr = 0x8c00;
	uint16_t temp_val = 0;
	uint32_t dsp_val = 0;

	for (dsp_addr = 0x8c00; dsp_addr < 0x9000; dsp_addr++) {
		dsp_val = 0;
		aw881xx_dsp_read(aw881xx, dsp_addr, &temp_val);
		dsp_val |= (temp_val << 0);
		aw881xx_dsp_read(aw881xx, dsp_addr + 1, &temp_val);
		dsp_val |= (temp_val << 16);
		if ((dsp_val & 0x7fffff00) == 0x7fffff00) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"dsp fw ver: v1.%u\n",
					0xff - (dsp_val & 0xff));
			break;
		}
	}

	return len;
}

static ssize_t aw881xx_spk_temp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret;
	int16_t reg_val;

	ret = aw881xx_reg_read(aw881xx, AW881XX_REG_ASR2, (uint16_t *)&reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: read addr:0x%x failed\n",
					__func__, AW881XX_REG_ASR2);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len,
			"Temp:%d\n", reg_val);

	return len;
}

static ssize_t aw881xx_diver_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"driver version:%s\n", AW881XX_DRIVER_VERSION);

	return len;
}

static ssize_t aw881xx_fade_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t fade_en;

	if (1 == sscanf(buf, "%u", &fade_en))
		aw881xx->fade_en = fade_en;

	aw_dev_info(aw881xx->dev, "%s: set fade %d\n",
		__func__, aw881xx->fade_en);

	return count;
}

static ssize_t aw881xx_fade_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"fade_en: %u\n", aw881xx->fade_en);

	return len;
}

static ssize_t aw881xx_pa_sync_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	uint32_t pa_syn_en;

	if (1 == sscanf(buf, "%u", &pa_syn_en))
		aw881xx->pa_syn_en = pa_syn_en;

	aw_dev_info(aw881xx->dev, "%s: set pa_syn_en %d\n",
		__func__, aw881xx->pa_syn_en);

	return count;
}

static ssize_t aw881xx_pa_sync_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"pa_syn_en: %u\n", aw881xx->pa_syn_en);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
			aw881xx_reg_show, aw881xx_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO,
			aw881xx_rw_show, aw881xx_rw_store);
static DEVICE_ATTR(dsp_rw, S_IWUSR | S_IRUGO,
			aw881xx_dsp_rw_show, aw881xx_dsp_rw_store);
static DEVICE_ATTR(dsp, S_IWUSR | S_IRUGO,
			aw881xx_dsp_show, aw881xx_dsp_store);
static DEVICE_ATTR(dsp_fw_ver, S_IWUSR | S_IRUGO,
			aw881xx_dsp_fw_ver_show, aw881xx_dsp_fw_ver_store);
static DEVICE_ATTR(spk_temp, S_IRUGO,
			aw881xx_spk_temp_show, NULL);
static DEVICE_ATTR(driver_ver, S_IRUGO,
			aw881xx_diver_ver_show, NULL);
static DEVICE_ATTR(fade_en, S_IWUSR | S_IRUGO,
	aw881xx_fade_enable_show, aw881xx_fade_enable_store);
static DEVICE_ATTR(pa_sync_en, S_IWUSR | S_IRUGO,
	aw881xx_pa_sync_enable_show, aw881xx_pa_sync_enable_store);


static struct attribute *aw881xx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rw.attr,
	&dev_attr_dsp_rw.attr,
	&dev_attr_dsp.attr,
	&dev_attr_dsp_fw_ver.attr,
	&dev_attr_spk_temp.attr,
	&dev_attr_driver_ver.attr,
	&dev_attr_fade_en.attr,
	&dev_attr_pa_sync_en.attr,
	NULL
};

static struct attribute_group aw881xx_attribute_group = {
	.attrs = aw881xx_attributes
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw881xx_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai = NULL;
	struct aw881xx *aw881xx = NULL;
	struct device_node *np = i2c->dev.of_node;
	const char *aw881xx_rst = "aw881xx_rst";
	const char *aw881xx_int = "aw881xx_int";
	const char *aw881xx_irq_name = "aw881xx";
	int irq_flags = 0;
	int ret = -1;

	aw_dev_info(&i2c->dev, "%s: enter\n", __func__);

	aw_dev_info(&i2c->dev, "%s: dapm already haved \n", __func__);
	
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		aw_dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw881xx = devm_kzalloc(&i2c->dev, sizeof(struct aw881xx), GFP_KERNEL);
	if (aw881xx == NULL)
		return -ENOMEM;

	aw881xx->dev = &i2c->dev;
	aw881xx->i2c = i2c;

	i2c_set_clientdata(i2c, aw881xx);
	mutex_init(&aw881xx->lock);
	mutex_init(&aw881xx->i2c_lock);

	/* aw881xx rst & int */
	if (np) {
		ret = aw881xx_parse_dt(&i2c->dev, aw881xx, np);
		if (ret < 0) {
			aw_dev_err(&i2c->dev, "%s:failed to parse device tree node\n",
				__func__);
			return ret;
		}
	} else {
		aw881xx->reset_gpio = -1;
		aw881xx->irq_gpio = -1;
	}

	if (gpio_is_valid(aw881xx->reset_gpio)) {
		ret = aw881xx_append_suffix("%s_%x_%x", &aw881xx_rst, aw881xx);
		if (ret < 0)
			return ret;

		ret = devm_gpio_request_one(&i2c->dev, aw881xx->reset_gpio,
					GPIOF_OUT_INIT_LOW, aw881xx_rst);
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			return ret;
		}
	}

	if (gpio_is_valid(aw881xx->irq_gpio)) {
		ret = aw881xx_append_suffix("%s_%x_%x", &aw881xx_int, aw881xx);
		if (ret < 0)
			return ret;

		ret = devm_gpio_request_one(&i2c->dev, aw881xx->irq_gpio,
						GPIOF_DIR_IN, aw881xx_int);
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			return ret;
		}
	}

	/* hardware reset */
	aw881xx_hw_reset(aw881xx);

	/* aw881xx chip id */
	ret = aw881xx_read_chipid(aw881xx);
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s: aw881xx_read_chipid failed ret=%d\n",
			__func__, ret);
		return ret;
	}
	
	/* register codec */
	dai = devm_kzalloc(&i2c->dev, sizeof(aw881xx_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	memcpy(dai, aw881xx_dai, sizeof(aw881xx_dai));
	ret = aw881xx_append_suffix("%s-%x-%x", &dai->name, aw881xx);
	if (ret < 0)
		return ret;
	ret = aw881xx_append_suffix("%s_%x_%x",
		&dai->playback.stream_name, aw881xx);
	if (ret < 0)
		return ret;
	ret = aw881xx_append_suffix("%s_%x_%x",
		&dai->capture.stream_name, aw881xx);
	if (ret < 0)
		return ret;

	/* change dapm */
	ret = aw881xx_append_suffix("%s_%x_%x",
		&aw881xx_dapm_widgets[0].sname, aw881xx);
	if (ret < 0)
		return ret;

	ret = aw881xx_append_suffix("%s_%x_%x",
		&aw881xx_dapm_widgets[2].sname, aw881xx);
	if (ret < 0)
		return ret;
	
	ret = aw_componet_codec_ops.aw_snd_soc_register_codec(&i2c->dev,
					&soc_codec_dev_aw881xx,
					dai, ARRAY_SIZE(aw881xx_dai));
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s failed to register aw881xx: %d\n",
				__func__, ret);
		return ret;
	}

	/* aw881xx irq */
	if (gpio_is_valid(aw881xx->irq_gpio) &&
		!(aw881xx->flags & AW881XX_FLAG_SKIP_INTERRUPTS)) {
		ret = aw881xx_append_suffix("%s_%x_%x",
			&aw881xx_irq_name, aw881xx);
		if (ret < 0)
			goto err_irq_append_suffix;
		aw881xx_interrupt_setup(aw881xx);
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw881xx->irq_gpio),
						NULL, aw881xx_irq, irq_flags,
						aw881xx_irq_name, aw881xx);
		if (ret != 0) {
			aw_dev_err(&i2c->dev,
				"failed to request IRQ %d: %d\n",
				gpio_to_irq(aw881xx->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		aw_dev_info(&i2c->dev, "%s skipping IRQ registration\n",
				__func__);
		/* disable feature support if gpio was invalid */
		aw881xx->flags |= AW881XX_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw881xx);
	ret = sysfs_create_group(&i2c->dev.kobj, &aw881xx_attribute_group);
	if (ret < 0) {
		aw_dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
				__func__);
		goto err_sysfs;
	}
	aw881xx_cali_init(&aw881xx->cali_attr);
	aw881xx_monitor_init(&aw881xx->monitor);

	aw881xx->allow_pw = true;
	aw881xx->work_queue = NULL;
	aw881xx->fw_status = AW881XX_FW_FAILED;

	mutex_lock(&g_aw881xx_lock);
	g_aw881xx_dev_cnt++;
	mutex_unlock(&g_aw881xx_lock);
	aw_dev_info(&i2c->dev, "%s: dev_cnt %d completed successfully\n",
		__func__, g_aw881xx_dev_cnt);

	return 0;

err_sysfs:
err_irq:
err_irq_append_suffix:
	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);
	return ret;
}

static int aw881xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw881xx *aw881xx = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);

	aw881xx_monitor_deinit(&aw881xx->monitor);
	aw881xx_device_deinit(aw881xx);
	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);

	mutex_lock(&g_aw881xx_lock);
	g_aw881xx_dev_cnt--;
	if (g_aw881xx_dev_cnt == 0) {
		if (g_awinic_cfg != NULL) {
			vfree(g_awinic_cfg);
			g_awinic_cfg = NULL;
		}
	}
	mutex_unlock(&g_aw881xx_lock);

	return 0;
}

static const struct i2c_device_id aw881xx_i2c_id[] = {
	{AW881XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw881xx_i2c_id);

static struct of_device_id aw881xx_dt_match[] = {
	{.compatible = "awinic,aw881xx_smartpa"},
	{},
};

static struct i2c_driver aw881xx_i2c_driver = {
	.driver = {
		.name = AW881XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw881xx_dt_match),
		},
	.probe = aw881xx_i2c_probe,
	.remove = aw881xx_i2c_remove,
	.id_table = aw881xx_i2c_id,
};

static int __init aw881xx_i2c_init(void)
{
	int ret = -1;

	pr_info("%s: aw881xx driver version %s\n",
			__func__, AW881XX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw881xx_i2c_driver);
	if (ret)
		pr_err("%s: fail to add aw881xx device into i2c\n", __func__);

	return ret;
}

module_init(aw881xx_i2c_init);

static void __exit aw881xx_i2c_exit(void)
{
	i2c_del_driver(&aw881xx_i2c_driver);
}

module_exit(aw881xx_i2c_exit);

MODULE_DESCRIPTION("ASoC AW881XX Smart PA Driver");
MODULE_LICENSE("GPL v2");
