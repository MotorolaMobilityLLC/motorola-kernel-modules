// SPDX-License-Identifier: GPL-2.0
/*
 * Awinic high voltage LRA haptic driver
 *
 * Copyright (c) 2021-2023 awinic. All Rights Reserved.
 *
 * Author: Ethan <renzhiqiang@awinic.com>
 */

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

static void aw8672x_vbat_mode_config(struct aw_haptic *, uint8_t);

/******************************************************
 *
 * aw8672x codec
 *
 ******************************************************/
#ifdef AW_SND_SOC_CODEC
#ifdef KERNEL_OVER_4_19
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
#ifdef KERNEL_OVER_4_19
	return dai->component;
#else
	return dai->codec;
#endif
}

static void aw8672x_i2s_enable(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_EN_DLL_MASK,
					 AW8672X_BIT_SYSCTRL3_EN_DLL_ON);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_I2SCFG1,
					 AW8672X_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8672X_BIT_I2SCFG1_I2S_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_EN_DLL_MASK,
					 AW8672X_BIT_SYSCTRL3_EN_DLL_OFF);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_I2SCFG1,
					 AW8672X_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8672X_BIT_I2SCFG1_I2S_DISABLE);
	}
}

static int aw8672x_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	return 0;
}

static int aw8672x_set_fmt(struct snd_soc_dai *dai, uint32_t fmt)
{
	aw_info("fmt=0x%X", fmt);

	return 0;
}

static int aw8672x_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id, uint32_t freq, int dir)
{
	aw_info("freq=%d", freq);

	return 0;
}

static int aw8672x_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	uint8_t mask = 0;
	uint8_t reg_val = 0;
	uint8_t bit_width = 0;
	uint32_t sample_rate = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_err("steam is capture");
		return 0;
	}

	sample_rate = params_rate(params);
	aw_info("sample rate = %u", sample_rate);
	switch (sample_rate) {
	case 48000:
		reg_val |= AW8672X_BIT_I2SCFG2_I2S_SR_48K;
		break;
	case 96000:
		reg_val |= AW8672X_BIT_I2SCFG2_I2S_SR_96K;
		break;
	default:
		reg_val |= AW8672X_BIT_I2SCFG2_I2S_SR_48K;
		aw_err("default use 48K");
		break;
	}

	bit_width = params_width(params);
	aw_info("bit width = %d", bit_width);
	switch (bit_width) {
	case 16:
		reg_val |= AW8672X_BIT_I2SCFG2_BCK_MODE_16;
		break;
	case 24:
		reg_val |= AW8672X_BIT_I2SCFG2_BCK_MODE_24;
		break;
	case 32:
		reg_val |= AW8672X_BIT_I2SCFG2_BCK_MODE_32;
		break;
	default:
		reg_val |= AW8672X_BIT_I2SCFG2_BCK_MODE_32;
		aw_err("default use 32 bit");
		break;
	}

	mask = AW8672X_BIT_I2SCFG2_I2S_SR_MASK & AW8672X_BIT_I2SCFG2_BCK_MODE_MASK;
	mutex_lock(&aw_haptic->lock);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_I2SCFG2, mask, reg_val);
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int aw8672x_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	uint8_t reg_val = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_err("steam is capture");
		return 0;
	}
	aw_info("mute state=%d", mute);
	if (mute) {
		mutex_lock(&aw_haptic->lock);
		aw8672x_i2s_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->lock);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw8672x_i2s_enable(aw_haptic, true);
		usleep_range(1000, 1500);
		haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_GLBRD5, &reg_val, AW_I2C_BYTE_ONE);
		if (reg_val != 0x0a) {
			aw_err("i2s config err, glb_state=0x%02X", reg_val);
			aw8672x_i2s_enable(aw_haptic, false);
		}
		mutex_unlock(&aw_haptic->lock);
	}

	return 0;
}

static void aw8672x_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_err("steam is capture");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	aw8672x_i2s_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
}

static const struct snd_soc_dai_ops aw8672x_dai_ops = {
	.startup = aw8672x_startup,
	.set_fmt = aw8672x_set_fmt,
	.set_sysclk = aw8672x_set_dai_sysclk,
	.hw_params = aw8672x_hw_params,
	.mute_stream = aw8672x_mute,
	.shutdown = aw8672x_shutdown,
};

static struct snd_soc_dai_driver aw8672x_dai[] = {
	{
		.name = "aw8672x-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &aw8672x_dai_ops,
	},
};

static int aw8672x_codec_probe(aw_snd_soc_codec_t *codec)
{
	uint8_t reg_val[2] = { 0 };
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	reg_val[0] |= AW8672X_BIT_I2SCFG1_SLOT_NUM_I2S;
	reg_val[0] |= AW8672X_BIT_I2SCFG1_I2S_MODE_PHILIP;
	reg_val[0] |= AW8672X_BIT_I2SCFG1_RX_SLOTVLD_1;

	reg_val[1] |= AW8672X_BIT_I2SCFG2_FSYN_TYP_ONE_SLOT;
	reg_val[1] |= AW8672X_BIT_I2SCFG2_I2S_INT_ENABLE;

	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_I2SCFG1, reg_val, AW_I2C_BYTE_TWO);

	return 0;
}

#ifdef KERNEL_OVER_4_19
static void aw8672x_codec_remove(aw_snd_soc_codec_t *component)
{
	aw_info("enter");
}
#else
static int aw8672x_codec_remove(aw_snd_soc_codec_t *codec)
{
	aw_info("enter");

	return 0;
}
#endif

static aw_snd_soc_codec_driver_t soc_codec_dev_aw8672x = {
	.probe = aw8672x_codec_probe,
	.remove = aw8672x_codec_remove,
};

static int aw8672x_snd_soc_init(struct device *dev)
{
	int ret = 0;
	struct snd_soc_dai_driver *dai;

	/* register codec */
	dai = devm_kzalloc(dev, sizeof(aw8672x_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	memcpy(dai, aw8672x_dai, sizeof(aw8672x_dai));
	aw_info("dai->name(%s)", dai->name);

	ret = aw_componet_codec_ops.aw_snd_soc_register_codec(dev, &soc_codec_dev_aw8672x,
							      dai, ARRAY_SIZE(aw8672x_dai));
	if (ret < 0) {
		aw_err("failed to register aw8672x: %d", ret);
		return ret;
	}

	return 0;
}
#endif

static int aw8672x_check_qualify(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t reg_val = 0;

	aw_info("enter");
	ret = haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_EFCFG6, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if (!(reg_val & 0x80)) {
		aw_err("unqualified chip!");
		return -ERANGE;
	}

	return 0;
}

static void aw8672x_reg_unlock(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t reg_val = 0;

	if (flag) {
		/* Unlock register */
		reg_val = AW8672X_BIT_TMCFG_TM_UNLOCK;
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_TMCFG, &reg_val, AW_I2C_BYTE_ONE);
	} else {
		/* Lock register */
		reg_val = AW8672X_BIT_TMCFG_TM_LOCK;
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_TMCFG, &reg_val, AW_I2C_BYTE_ONE);
	}
}

static void aw8672x_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_WAVDAT_MODE_MASK,
					 AW8672X_BIT_SYSCTRL3_RATE_48K);
		break;
	case AW_PWM_24K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_WAVDAT_MODE_MASK,
					 AW8672X_BIT_SYSCTRL3_RATE_24K);
		break;
	case AW_PWM_12K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_WAVDAT_MODE_MASK,
					 AW8672X_BIT_SYSCTRL3_RATE_12K);
		break;
	case AW_PWM_8K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
					 AW8672X_BIT_SYSCTRL3_WAVDAT_MODE_MASK,
					 AW8672X_BIT_SYSCTRL3_RATE_8K);
		break;
	default:
		break;
	}
}

static void aw8672x_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_PLAYCFG2, &gain, AW_I2C_BYTE_ONE);
}

static void aw8672x_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{

}

static void aw8672x_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	if (bst_vol < AW8672X_CHARGEPUMP_6P25V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_6V;
	else if (bst_vol < AW8672X_CHARGEPUMP_6P5V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_6P25V;
	else if (bst_vol < AW8672X_CHARGEPUMP_6P75V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_6P5V;
	else if (bst_vol < AW8672X_CHARGEPUMP_7V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_6P75V;
	else if (bst_vol < AW8672X_CHARGEPUMP_7P25V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_7V;
	else if (bst_vol < AW8672X_CHARGEPUMP_7P5V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_7P25V;
	else if (bst_vol < AW8672X_CHARGEPUMP_7P75V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_7P5V;
	else if (bst_vol < AW8672X_CHARGEPUMP_8V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_7P75V;
	else if (bst_vol < AW8672X_CHARGEPUMP_8P25V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_8V;
	else if (bst_vol < AW8672X_CHARGEPUMP_8P5V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_8P25V;
	else if (bst_vol < AW8672X_CHARGEPUMP_8P75V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_8P5V;
	else if (bst_vol < AW8672X_CHARGEPUMP_9V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_8P75V;
	else if (bst_vol < AW8672X_CHARGEPUMP_9P25V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_9V;
	else if (bst_vol < AW8672X_CHARGEPUMP_9P5V)
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_9P25V;
	else
		reg_val = AW8672X_BIT_ANACFG13_CP_SET_OVP_9P5V;

	aw8672x_reg_unlock(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_ANACFG13,
				 AW8672X_BIT_ANACFG13_CP_SET_OVP_MASK, reg_val);
	aw8672x_reg_unlock(aw_haptic, false);
}

static void aw8672x_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t seq)
{
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_WAVCFG1 + wav, &seq, AW_I2C_BYTE_ONE);
}

static void aw8672x_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_WAVCFG9 + (wav / 2),
					 AW8672X_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_WAVCFG9 + (wav / 2),
					 AW8672X_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw8672x_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data, uint32_t len)
{
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_RTPDATA, data, len);
}

static void aw8672x_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSINTM,
					 AW8672X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8672X_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSINTM,
					 AW8672X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8672X_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8672x_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr_l = (uint8_t)(aw_haptic->ram.base_addr & 0x00ff);

	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_RAMADDRH, AW8672X_BIT_RAMADDRH_MASK,
				 (uint8_t)(aw_haptic->ram.base_addr >> 8));
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_RAMADDRL, &ram_addr_l, AW_I2C_BYTE_ONE);
}

static void aw8672x_auto_brake_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8672X_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8672X_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static void aw8672x_f0_detect(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG1,
					 AW8672X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8672X_BIT_CONTCFG1_F0_DET_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG1,
					 AW8672X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8672X_BIT_CONTCFG1_F0_DET_DISABLE);
	}
}

static uint8_t aw8672x_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_GLBRD5, &state, AW_I2C_BYTE_ONE);
	state &= AW_GLBRD_STATE_MASK;
	aw_dbg("glb state value is 0x%02X", state);

	return state;
}

static void aw8672x_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t go_on = AW8672X_BIT_PLAYCFG4_GO_ON;
	uint8_t stop_on = AW8672X_BIT_PLAYCFG4_STOP_ON;

	aw_dbg("enter, flag = %d", flag);
	if (flag)
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_PLAYCFG4, &go_on, AW_I2C_BYTE_ONE);
	else
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_PLAYCFG4, &stop_on, AW_I2C_BYTE_ONE);
}

static int aw8672x_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw8672x_get_glb_state(aw_haptic);
		if (reg_val == AW8672X_BIT_GLBRD5_STATE_STANDBY) {
			aw_info("entered standby!");
			return 0;
		}
		aw_dbg("wait for standby");
		usleep_range(2000, 2500);
	}
	aw_err("do not enter standby automatically");

	return -ERANGE;
}

static void aw8672x_bst_mode_config(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_BST_BOOST_MODE:
		aw_info("haptic bst mode = boost");
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG1,
					 AW8672X_BIT_PLAYCFG1_EN_CP_2X_MASK,
					 AW8672X_BIT_PLAYCFG1_EN_CP_2X_CHARGEPUMP);
		break;
	case AW_BST_BYPASS_MODE:
		aw_info("haptic bst mode = bypass");
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG1,
					 AW8672X_BIT_PLAYCFG1_EN_CP_2X_MASK,
					 AW8672X_BIT_PLAYCFG1_EN_CP_2X_BYPASS);
		break;
	default:
		aw_err("error mode %d", mode);
		break;
	}
}

static void aw8672x_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL2,
					 AW8672X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8672X_BIT_SYSCTRL2_STANDBY_ON);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL2,
					 AW8672X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8672X_BIT_SYSCTRL2_STANDBY_OFF);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8672x_auto_brake_mode(aw_haptic, false);
		aw8672x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8672x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8672x_auto_brake_mode(aw_haptic, true);
		aw8672x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8672x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_RTP);
		aw8672x_auto_brake_mode(aw_haptic, true);
		aw8672x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8672x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8672x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8672X_BIT_PLAYCFG3_PLAY_MODE_CONT);
		aw8672x_auto_brake_mode(aw_haptic, true);
		aw8672x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8672x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static void aw8672x_stop(struct aw_haptic *aw_haptic)
{
	aw8672x_play_go(aw_haptic, false);
	aw8672x_wait_enter_standby(aw_haptic);
	aw8672x_play_mode(aw_haptic, AW_STANDBY_MODE);
}

static void aw8672x_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL2,
					 AW8672X_BIT_SYSCTRL2_RAMINIT_MASK,
					 AW8672X_BIT_SYSCTRL2_RAMINIT_ON);
		usleep_range(1000, 1050);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL2,
					 AW8672X_BIT_SYSCTRL2_RAMINIT_MASK,
					 AW8672X_BIT_SYSCTRL2_RAMINIT_OFF);
	}
}

static void aw8672x_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRIMCFG5,
					 AW8672X_BIT_TRIMCFG5_TRIM_LRA_MASK, 0x00);
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X", aw_haptic->f0_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRIMCFG5,
					 AW8672X_BIT_TRIMCFG5_TRIM_LRA_MASK,
					 (char)aw_haptic->f0_cali_data);
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X", aw_haptic->osc_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRIMCFG5,
					 AW8672X_BIT_TRIMCFG5_TRIM_LRA_MASK,
					 (char)aw_haptic->osc_cali_data);
		break;
	default:
		aw_err("error param!");
		break;
	}
}

static uint8_t aw8672x_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_TRIMCFG5, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= (~AW8672X_BIT_TRIMCFG5_TRIM_LRA_MASK);

	return reg_val;
}

static void aw8672x_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_VBATCTRL,
					 AW8672X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8672X_BIT_VBATCTRL_VBAT_MODE_HW);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_VBATCTRL,
					 AW8672X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8672X_BIT_VBATCTRL_VBAT_MODE_SW);
	}
}

static void aw8672x_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime, uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PWMCFG1, AW8672X_BIT_PWMCFG1_PRC_EN_MASK,
				 AW8672X_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW8672X_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW8672X_BIT_PWMCFG3_PRLVL_MASK));
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_PWMCFG4, &prtime, AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PWMCFG3,
					 AW8672X_BIT_PWMCFG3_PR_EN_MASK,
					 AW8672X_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw8672x_cont_config(struct aw_haptic *aw_haptic)
{
	uint8_t drv2_time = 0xFF;

	/* work mode */
	aw8672x_play_mode(aw_haptic, AW_CONT_MODE);
	/* f0 driver level */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG6,
				 (AW8672X_BIT_CONTCFG6_TRACK_EN_MASK &
				  AW8672X_BIT_CONTCFG6_DRV1_LVL_MASK),
				 ((aw_haptic->info.is_enabled_track_en << 7) |
				  aw_haptic->info.cont_drv1_lvl));

	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG7,
			     &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* DRV2_TIME */
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG9, &drv2_time, AW_I2C_BYTE_ONE);
	/* cont play go */
	aw8672x_play_go(aw_haptic, true);
}

static void aw8672x_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t trig_prio = 0x6c;

	aw_info("enter");
	/*if enable one-wire, trig1 priority must be less than trig2 and trig3*/
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_GLBCFG4, &trig_prio, AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG8,
				 AW8672X_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
				 AW8672X_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw8672x_trig1_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[0].trig_level   = aw_haptic->info.trig_cfg[0];
	aw_haptic->trig[0].trig_polar   = aw_haptic->info.trig_cfg[1];
	aw_haptic->trig[0].pos_enable   = aw_haptic->info.trig_cfg[2];
	aw_haptic->trig[0].pos_sequence = aw_haptic->info.trig_cfg[3];
	aw_haptic->trig[0].neg_enable   = aw_haptic->info.trig_cfg[4];
	aw_haptic->trig[0].neg_sequence = aw_haptic->info.trig_cfg[5];
	aw_haptic->trig[0].trig_brk     = aw_haptic->info.trig_cfg[6];
	aw_haptic->trig[0].trig_bst     = aw_haptic->info.trig_cfg[7];
}

static void aw8672x_trig2_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[1].trig_level   = aw_haptic->info.trig_cfg[8];
	aw_haptic->trig[1].trig_polar   = aw_haptic->info.trig_cfg[9];
	aw_haptic->trig[1].pos_enable   = aw_haptic->info.trig_cfg[10];
	aw_haptic->trig[1].pos_sequence = aw_haptic->info.trig_cfg[11];
	aw_haptic->trig[1].neg_enable   = aw_haptic->info.trig_cfg[12];
	aw_haptic->trig[1].neg_sequence = aw_haptic->info.trig_cfg[13];
	aw_haptic->trig[1].trig_brk     = aw_haptic->info.trig_cfg[14];
	aw_haptic->trig[1].trig_bst     = aw_haptic->info.trig_cfg[15];
}

static void aw8672x_trig3_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[2].trig_level   = aw_haptic->info.trig_cfg[16];
	aw_haptic->trig[2].trig_polar   = aw_haptic->info.trig_cfg[17];
	aw_haptic->trig[2].pos_enable   = aw_haptic->info.trig_cfg[18];
	aw_haptic->trig[2].pos_sequence = aw_haptic->info.trig_cfg[19];
	aw_haptic->trig[2].neg_enable   = aw_haptic->info.trig_cfg[20];
	aw_haptic->trig[2].neg_sequence = aw_haptic->info.trig_cfg[21];
	aw_haptic->trig[2].trig_brk     = aw_haptic->info.trig_cfg[22];
	aw_haptic->trig[2].trig_bst     = aw_haptic->info.trig_cfg[23];
}

static void aw8672x_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_MODE_EDGE;
	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_POLAR_NEG;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_POLAR_POS;
	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG1_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG7,
				 (AW8672X_BIT_TRGCFG7_TRG1_MODE_MASK &
				  AW8672X_BIT_TRGCFG7_TRG1_POLAR_MASK &
				  AW8672X_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
				  AW8672X_BIT_TRGCFG7_TRG1_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[0].pos_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG1, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[0].neg_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG4, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8672x_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_MODE_EDGE;
	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_POLAR_NEG;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_POLAR_POS;
	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG7_TRG2_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG7,
				 (AW8672X_BIT_TRGCFG7_TRG2_MODE_MASK &
				  AW8672X_BIT_TRGCFG7_TRG2_POLAR_MASK &
				  AW8672X_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
				  AW8672X_BIT_TRGCFG7_TRG2_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[1].pos_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG2, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[1].neg_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG5, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8672x_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_MODE_EDGE;
	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_POLAR_NEG;
	else
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_POLAR_POS;
	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRGCFG8_TRG3_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG8,
				 (AW8672X_BIT_TRGCFG8_TRG3_MODE_MASK &
				  AW8672X_BIT_TRGCFG8_TRG3_POLAR_MASK &
				  AW8672X_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
				  AW8672X_BIT_TRGCFG8_TRG3_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[2].pos_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG3, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[2].neg_enable)
		trig_config |= AW8672X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8672X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_TRGCFG6, (AW8672X_BIT_TRG_ENABLE_MASK &
				 AW8672X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8672x_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;

	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8672X_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG3,
					 AW8672X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8672X_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw8672x_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* int enable */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSINTM,
				 (AW8672X_BIT_SYSINTM_CP_OVPM_MASK &
				  AW8672X_BIT_SYSINTM_UVLM_MASK &
				  AW8672X_BIT_SYSINTM_OCDM_MASK &
				  AW8672X_BIT_SYSINTM_OTM_MASK),
				 (AW8672X_BIT_SYSINTM_CP_OVPM_OFF |
				  AW8672X_BIT_SYSINTM_UVLM_ON |
				  AW8672X_BIT_SYSINTM_OCDM_ON |
				  AW8672X_BIT_SYSINTM_OTM_ON));
}

static int aw8672x_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8672x_get_glb_state(aw_haptic);
	if (glb_state == AW8672X_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}

	return rtp_state;
}

static void aw8672x_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_RAMDATA, buf + i, size);
		i += size;
	}
}

static void aw8672x_get_first_wave_addr(struct aw_haptic *aw_haptic, uint8_t *wave_addr)
{
	uint8_t reg_array[3] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_RAMDATA, reg_array, AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_array[1];
	wave_addr[1] = reg_array[2];
}

static void aw8672x_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint32_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_WAVCFG1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static size_t aw8672x_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	size_t count = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_WAVCFG9, reg_val, AW_SEQUENCER_LOOP_SIZE);
	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;
		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02x\n", i * 2 + 2,
				  aw_haptic->loop[i * 2 + 1]);
	}

	return count;
}

static void aw8672x_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
}

static uint8_t aw8672x_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
	reg_val >>= 7;

	return reg_val;
}

static int aw8672x_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);

	if (reg_val & AW8672X_BIT_SYSINT_CP_OVPI) {
		ret = AW_IRQ_CP_OVP;
		aw_err("chip cp_ovp int error");
	}

	if (reg_val & AW8672X_BIT_SYSINT_UVLI) {
		ret = AW_IRQ_UVLO;
		aw_err("chip uvlo int error");
	}

	if (reg_val & AW8672X_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_err("chip over current int error");
	}

	if (reg_val & AW8672X_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_err("chip over temperature int error");
	}

	if (reg_val & AW8672X_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_info("chip playback done");
	}
	if (reg_val & AW8672X_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		aw_info("aw_haptic rtp mode fifo almost full!");
	}

	if (reg_val & AW8672X_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;

	return ret;
}

static int aw8672x_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_CONTRD14, reg_val, AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("lra_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	f0_tmp = AW8672X_F0_FORMULA(f0_reg);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("lra_f0=%d", aw_haptic->f0);
#else
	/* cont_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_CONTRD16, reg_val, AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("cont_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	f0_tmp = AW8672X_F0_FORMULA(f0_reg);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("cont_f0=%d", aw_haptic->f0);
#endif

	return 0;
}

static int aw8672x_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t d2s_gain_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8672x_stop(aw_haptic);
	/* config max d2s_gain */
	haptic_hv_i2c_reads(aw_haptic,  AW8672X_REG_DETCFG2, &reg_val, AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8672X_BIT_DETCFG2_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_D2S_GAIN_MASK,
				 AW8672X_BIT_DETCFG2_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8672x_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	aw8672x_f0_detect(aw_haptic, true);
	/* cont config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG6,
				 AW8672X_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_PLAYCFG3, &reg_val, AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & AW8672X_BIT_PLAYCFG3_BRK_ENABLE;
	aw8672x_auto_brake_mode(aw_haptic, true);
	/* f0 driver level & time */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG6,
				 AW8672X_BIT_CONTCFG6_DRV1_LVL_MASK, aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG7, cont_config, AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("awinic->info.cont_track_margin = 0!");
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG11,
				     &aw_haptic->info.cont_track_margin, AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (!aw_haptic->info.f0_pre)
		return -ERANGE;
	drv_width = AW_DRV_WIDTH_FORMULA(aw_haptic->info.f0_pre, aw_haptic->info.cont_brk_gain,
					 aw_haptic->info.cont_track_margin);
	if (drv_width < AW_REG_VALUE_MIN)
		drv_width = AW_REG_VALUE_MIN;
	else if (drv_width > AW_REG_VALUE_MAX)
		drv_width = AW_REG_VALUE_MAX;
	cont_config[0] = (uint8_t)drv_width;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG3, &cont_config[0], AW_I2C_BYTE_ONE);
	/* play go */
	aw8672x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8672x_wait_enter_standby(aw_haptic);
	ret = aw8672x_read_f0(aw_haptic);
	/* restore default config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8672x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8672x_auto_brake_mode(aw_haptic, true);
	else
		aw8672x_auto_brake_mode(aw_haptic, false);

	return ret;
}

static int aw8672x_ram_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t d2s_gain_default = 0;
	int ret = 0;

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return -ERANGE;
	}
	if (aw_haptic->ram.ram_num < AW_RAM_GET_F0_SEQ) {
		aw_err("miss ram get f0 waveform!");
		return -ERANGE;
	}
	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8672x_stop(aw_haptic);
	/* config max d2s_gain */
	haptic_hv_i2c_reads(aw_haptic,  AW8672X_REG_DETCFG2, &reg_val, AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8672X_BIT_DETCFG2_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_D2S_GAIN_MASK,
				 AW8672X_BIT_DETCFG2_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8672x_play_mode(aw_haptic, AW_RAM_MODE);
	/* enable f0 detect */
	aw8672x_f0_detect(aw_haptic, true);
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_PLAYCFG3, &reg_val, AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & AW8672X_BIT_PLAYCFG3_BRK_ENABLE;
	aw8672x_auto_brake_mode(aw_haptic, true);
	aw8672x_set_bst_vol(aw_haptic, 6558);
	aw8672x_set_wav_seq(aw_haptic, 0x00, AW_RAM_GET_F0_SEQ);
	aw8672x_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8672x_set_wav_loop(aw_haptic, 0x00, 0x02);
	/* play go */
	aw8672x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8672x_wait_enter_standby(aw_haptic);
	ret = aw8672x_read_f0(aw_haptic);
	/* restore default config */
	aw8672x_set_bst_vol(aw_haptic, aw_haptic->vmax);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8672x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8672x_auto_brake_mode(aw_haptic, true);
	else
		aw8672x_auto_brake_mode(aw_haptic, false);

	return ret;
}

static uint8_t aw8672x_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8672X_BIT_SYSST_FF_AFS;
	reg_val >>= 3;

	return reg_val;
}

static uint8_t aw8672x_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8672X_BIT_SYSST_FF_AES;
	reg_val >>= 4;

	return reg_val;
}

static uint8_t aw8672x_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSST2, &state, AW_I2C_BYTE_ONE);
	state &= AW8672X_BIT_SYSST2_FF_EMPTY;

	return state;
}

static int aw8672x_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW8672X_BIT_DETCFG2_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_16:
		d2s_gain = 16;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW8672X_BIT_DETCFG2_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}

	return d2s_gain;
}

static void aw8672x_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint8_t adc_fs_default = 0;
	uint8_t d2s_gain = 0;
	uint32_t lra_code = 0;

	aw8672x_ram_init(aw_haptic, true);
	aw8672x_stop(aw_haptic);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8672X_BIT_DETCFG2_DET_SEQ0_RL);
	/* ADC_FS */
	haptic_hv_i2c_reads(aw_haptic,  AW8672X_REG_DETCFG1, &reg_val[0], AW_I2C_BYTE_ONE);
	adc_fs_default = reg_val[0] & AW8672X_BIT_DETCFG1_ADC_FS;
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_ADC_FS_MASK,
				 AW8672X_BIT_DETCFG1_ADC_FS_24KHZ);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_NA);

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_DETCFG2, &reg_val[0], AW_I2C_BYTE_ONE);
	d2s_gain = reg_val[0] & AW8672X_BIT_DETCFG2_D2S_GAIN;
	d2s_gain = aw8672x_select_d2s_gain(d2s_gain);
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_DETRD1, reg_val, AW_I2C_BYTE_TWO);
	lra_code = ((reg_val[0] & AW8672X_BIT_DETRD1_AVG_DATA) << 8) + reg_val[1];
	aw_haptic->lra = AW8672X_LRA_FORMULA(lra_code, d2s_gain);
	/* restore default config */
	aw8672x_ram_init(aw_haptic, false);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_ADC_FS_MASK,
				 adc_fs_default);
}

static void aw8672x_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8672x_set_wav_seq(aw_haptic, 0x00, seq);
	aw8672x_set_wav_loop(aw_haptic, 0x00, AW8672X_BIT_WAVLOOP_INIFINITELY);
}

static void aw8672x_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t vbat_code = 0;

	aw8672x_stop(aw_haptic);
	aw8672x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8672X_BIT_DETCFG2_DET_SEQ0_VBAT);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_NA);
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_DETRD1, reg_val, AW_I2C_BYTE_TWO);
	vbat_code = ((reg_val[0] & AW8672X_BIT_DETRD1_AVG_DATA) << 8) + reg_val[1];
	aw_haptic->vbat = AW8672X_VBAT_FORMULA(vbat_code);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_info("vbat max limit = %d", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_info("vbat min limit = %d", aw_haptic->vbat);
	}
	aw_info("aw_haptic->vbat=%dmV, vbat_code=0x%02X", aw_haptic->vbat, vbat_code);
	aw8672x_ram_init(aw_haptic, false);
}

static ssize_t aw8672x_get_reg(struct aw_haptic *aw_haptic, ssize_t len, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW8672X_REG_ANACFG16 + 1] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_RSTCFG, reg_array, AW8672X_REG_RTPDATA);
	haptic_hv_i2c_reads(aw_haptic, (AW8672X_REG_RTPDATA + 1),
			    &reg_array[AW8672X_REG_RTPDATA + 1],
			    (AW8672X_REG_RAMDATA - AW8672X_REG_RTPDATA - 1));
	haptic_hv_i2c_reads(aw_haptic, (AW8672X_REG_RAMDATA + 1),
			    &reg_array[AW8672X_REG_RAMDATA + 1],
			    (AW8672X_REG_ANACFG16 - AW8672X_REG_RAMDATA));
	for (i = 0; i <= AW8672X_REG_ANACFG16; i++)
		if ((i != AW8672X_REG_RTPDATA) && (i != AW8672X_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);

	return len;
}

static int aw8672x_offset_cali(struct aw_haptic *aw_haptic)
{
	int os_code = 0;
	int d2s_gain = 0;
	uint8_t reg_val[3] = { 0 };

	aw8672x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8672X_BIT_DETCFG2_DET_SEQ0_OS);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG1, AW8672X_BIT_DETCFG1_DET_GO_MASK,
				 AW8672X_BIT_DETCFG1_DET_GO_NA);
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_DETCFG2, &reg_val[2], AW_I2C_BYTE_ONE);
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_DETRD1, reg_val, AW_I2C_BYTE_TWO);
	aw8672x_ram_init(aw_haptic, false);
	reg_val[2] &= (~AW8672X_BIT_DETCFG2_D2S_GAIN_MASK);
	d2s_gain = aw8672x_select_d2s_gain(reg_val[2]);
	if (d2s_gain <= 0) {
		aw_err("d2s_gain is error");
		return -ERANGE;
	}
	os_code = ((reg_val[0] & (~AW8672X_BIT_DETRD1_AVG_DATA_H_MASK)) << 8) | reg_val[1];
	os_code = AW8672X_OS_FORMULA(os_code, d2s_gain);
	if (os_code > 15 || os_code < -15) {
		aw_info("offset calibration out of range");
		return -ERANGE;
	}

	return 0;
}

static void aw8672x_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_info("one wire is enabled!");
		aw8672x_one_wire_init(aw_haptic);
	} else {
		aw8672x_trig1_param_init(aw_haptic);
		aw8672x_trig1_param_config(aw_haptic);
	}

	aw8672x_trig2_param_init(aw_haptic);
	aw8672x_trig3_param_init(aw_haptic);
	aw8672x_trig2_param_config(aw_haptic);
	aw8672x_trig3_param_config(aw_haptic);
}

#ifdef AW_CHECK_RAM_DATA
static int aw8672x_check_ram_data(struct aw_haptic *aw_haptic, uint8_t *cont_data,
				  uint8_t *ram_data, uint32_t len)
{
	int i = 0;

	for (i = 0; i < len; i++) {
		if (ram_data[i] != cont_data[i]) {
			aw_err("check ramdata error, addr=0x%04x, ram_data=0x%02x, file_data=0x%02x",
				i, ram_data[i], cont_data[i]);
			return -ERANGE;
		}
	}

	return 0;
}
#endif

static int aw8672x_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t reg_val[3] = {0};
	uint8_t rtp_addr[5] = {0};
#ifdef AW_CHECK_RAM_DATA
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
#endif
	int i = 0;
	int ret = 0;
	int len = 0;
	uint32_t temp = 0;
	uint32_t shift = 0;
	uint32_t base_addr = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw8672x_ram_init(aw_haptic, true);
	/* Enter standby mode */
	aw8672x_stop(aw_haptic);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr = (uint32_t)((awinic_cont->data[0 + shift] << 8) |
				   (awinic_cont->data[1 + shift]));
	base_addr = aw_haptic->ram.base_addr;
	/* ADDRH */
	rtp_addr[0] = awinic_cont->data[0 + shift];
	/* ADDRL */
	rtp_addr[1] = awinic_cont->data[1 + shift];
	/* FIFO AEH FIFO AFH */
	rtp_addr[2] = (uint8_t)(AW8672X_FIFO_AE_ADDR_H(base_addr) |
				AW8672X_FIFO_AF_ADDR_H(base_addr));
	/* FIFO AEL */
	rtp_addr[3] = (uint8_t)(AW8672X_FIFO_AE_ADDR_L(base_addr));
	/* FIFO AFL */
	rtp_addr[4] = (uint8_t)(AW8672X_FIFO_AF_ADDR_L(base_addr));
	aw_info("base_addr = %d", base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_RTPCFG1, rtp_addr, AW_I2C_BYTE_FIVE);
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_RTPCFG3, reg_val, AW_I2C_BYTE_THREE);
	temp = ((reg_val[0] & AW8672X_BIT_RTPCFG3_FIFO_AFH) << 24) |
	       ((reg_val[0] & AW8672X_BIT_RTPCFG3_FIFO_AEH) << 4) | reg_val[1];
	aw_info("almost_empty_threshold = %d", (uint16_t)temp);
	temp = temp | (reg_val[2] << 16);
	aw_info("almost_full_threshold = %d", temp >> 16);
	/* ram */
	aw8672x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_RAMDATA, &awinic_cont->data[i], len);
		i += len;
	}
#ifdef AW_CHECK_RAM_DATA
	aw8672x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_RAMDATA, ram_data, len);
		ret = aw8672x_check_ram_data(aw_haptic, &awinic_cont->data[i], ram_data, len);
		if (ret < 0)
			break;
		i += len;
	}
	if (ret)
		aw_err("ram data check sum error");
	else
		aw_info("ram data check sum pass");
#endif
	/* RAMINIT Disable */
	aw8672x_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static uint64_t aw8672x_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_SYSCTRL2, &reg_val, AW_I2C_BYTE_ONE);
	fre_val = reg_val & (~AW8672X_BIT_SYSCTRL3_WAVDAT_MODE_MASK);
	if (fre_val == AW8672X_BIT_SYSCTRL3_RATE_24K)
		theory_time = aw_haptic->rtp_len * 1000 / 24;	/* 24K */
	else if (fre_val == AW8672X_BIT_SYSCTRL3_RATE_48K)
		theory_time = aw_haptic->rtp_len * 1000 / 48;	/* 48K */
	else if (fre_val == AW8672X_BIT_SYSCTRL3_RATE_8K)
		theory_time = aw_haptic->rtp_len * 1000 / 8;	/* 8K */
	else
		theory_time = aw_haptic->rtp_len * 1000 / 12;	/* 12K */
	aw_info("microsecond:%llu  theory_time = %llu", aw_haptic->microsecond, theory_time);

	return theory_time;
}

static void aw8672x_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			    struct device_node *np)
{
	uint8_t duration_time[3];
	uint8_t trig_config_temp[24];
	uint32_t val = 0;

	val = of_property_read_u8(np, "aw8672x_gain_bypass", &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_info("aw8672x_gain_bypass not found");
	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("f0_pre not found");
	val = of_property_read_u8(np, "aw8672x_f0_cali_percent", &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("aw8672x_f0_cali_percent not found");
	val = of_property_read_u8(np, "aw8672x_cont_smart_loop", &aw_haptic->info.cont_smart_loop);
	if (val != 0)
		aw_info("aw8672x_cont_smart_loop not found");
	val = of_property_read_u8(np, "aw8672x_cont_drv1_lvl", &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_info("aw8672x_cont_drv1_lvl not found");
	val = of_property_read_u32(np, "aw8672x_cont_lra_vrms", &aw_haptic->info.cont_lra_vrms);
	if (val != 0)
		aw_info("aw8672x_cont_lra_vrms not found");
	val = of_property_read_u8(np, "aw8672x_cont_drv1_time", &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_info("aw8672x_cont_drv1_time not found");
	val = of_property_read_u8(np, "aw8672x_cont_drv2_time", &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_info("aw8672x_cont_drv2_time not found");
	val = of_property_read_u8(np, "aw8672x_cont_brk_gain", &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_info("aw8672x_cont_brk_gain not found");
	val = of_property_read_u8(np, "aw8672x_d2s_gain", &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_info("aw8672x_d2s_gain not found");
	val = of_property_read_u8(np, "aw8672x_cont_brk_time", &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_info("aw8672x_cont_brk_time not found");
	val = of_property_read_u8(np, "aw8672x_cont_track_margin",
				  &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_info("aw8672x_cont_track_margin not found");
	aw_haptic->info.is_enabled_track_en =
		of_property_read_bool(np, "aw8672x_is_enabled_track_en");
	aw_info("aw_haptic->info.is_enabled_track_en = %d", aw_haptic->info.is_enabled_track_en);
	aw_haptic->info.is_enabled_auto_bst =
		of_property_read_bool(np, "aw8672x_is_enabled_auto_bst");
	aw_info("aw_haptic->info.is_enabled_auto_bst = %d", aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->info.is_enabled_one_wire = of_property_read_bool(np,
						 "aw8672x_is_enabled_one_wire");
	aw_info("aw_haptic->info.is_enabled_one_wire = %d", aw_haptic->info.is_enabled_one_wire);
	val = of_property_read_u8_array(np, "aw8672x_duration_time",
					duration_time, ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw8672x_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time, sizeof(duration_time));
	val = of_property_read_u32(np, "aw8672x_bst_vol_default", &aw_haptic->info.bst_vol_default);
	if (val != 0)
		aw_info("aw8672x_bst_vol_default not found");
	val = of_property_read_u8_array(np, "aw8672x_trig_config",
					trig_config_temp, ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("aw8672x_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp, sizeof(trig_config_temp));
	aw_dbg("aw_haptic->info.bst_vol_default: %d", aw_haptic->info.bst_vol_default);
}

static void aw8672x_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Set I2C broadcast addr */
	reg_val[0] = (uint8_t)aw_haptic->i2c->addr;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_SYSCTRL4, reg_val, AW_I2C_BYTE_ONE);
	/* Cont drv2 lvl */
	drv2_lvl = AW8672X_DRV2_LVL_FORMULA(aw_haptic->info.f0_pre, aw_haptic->info.cont_lra_vrms);
	if (drv2_lvl > AW_DRV2_LVL_MAX)
		aw_haptic->info.cont_drv2_lvl = AW_DRV2_LVL_MAX;
	else
		aw_haptic->info.cont_drv2_lvl = (uint8_t)drv2_lvl;
	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_PLAYCFG2, reg_val, AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	haptic_hv_i2c_reads(aw_haptic, AW8672X_REG_WAVCFG1, reg_val, AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	/* Set gain_bypass */
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_SYSCTRL3,
				 AW8672X_BIT_SYSCTRL3_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass);

	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG1,
				 AW8672X_BIT_CONTCFG1_SMART_LOOP_MASK,
				 aw_haptic->info.cont_smart_loop << 7);
	/* d2s_gain */
	if (!aw_haptic->info.d2s_gain)
		aw_err("aw_haptic->info.d2s_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_DETCFG2, AW8672X_BIT_DETCFG2_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);

	/* cont_brk_time */
	if (!aw_haptic->info.cont_brk_time)
		aw_err("aw_haptic->info.cont_brk_time = 0!");
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);

	/* cont_brk_gain */
	if (!aw_haptic->info.cont_brk_gain)
		aw_err("aw_haptic->info.cont_brk_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG5,
				 AW8672X_BIT_CONTCFG5_BRK_GAIN_MASK, aw_haptic->info.cont_brk_gain);
	aw8672x_protect_config(aw_haptic, AW8672X_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW8672X_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);

	haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_PLAYCFG1,
				 AW8672X_BIT_PLAYCFG1_BACK_OPEN_MASK,
				 AW8672X_BIT_PLAYCFG1_BACK_OPEN_OPEN_CLASSD);

	aw8672x_reg_unlock(aw_haptic, true);
	reg_val[0] = AW8672X_BIT_ANACFG11_DEFAULT_VALUE;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_ANACFG11, &reg_val[0], AW_I2C_BYTE_ONE);

	reg_val[1] = AW8672X_BIT_ANACFG14_DEFAULT_VALUE;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_ANACFG14, &reg_val[1], AW_I2C_BYTE_ONE);
	aw8672x_reg_unlock(aw_haptic, false);
}

static ssize_t cont_drv_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl,
			aw_haptic->info.cont_drv2_lvl);

	return len;
}

static ssize_t cont_drv_lvl_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];

		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG6,
					 AW8672X_BIT_CONTCFG6_DRV1_LVL_MASK,
					 aw_haptic->info.cont_drv1_lvl);
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_CONTCFG7,
					 AW8672X_BIT_CONTCFG7_DRV2_LVL_MASK,
					 aw_haptic->info.cont_drv2_lvl);
	}

	return count;
}

static ssize_t cont_drv_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time, aw_haptic->info.cont_drv2_time);

	return len;
}

static ssize_t cont_drv_time_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_time = databuf[0];
		aw_haptic->info.cont_drv2_time = databuf[1];
		reg_array[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
		reg_array[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
		haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG8, reg_array, AW_I2C_BYTE_TWO);
	}

	return count;
}

static ssize_t cont_brk_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw_haptic->info.cont_brk_time);

	return len;
}

static ssize_t cont_brk_time_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtou8(buf, 0, &aw_haptic->info.cont_brk_time);
	if (rc < 0)
		return rc;
	haptic_hv_i2c_writes(aw_haptic, AW8672X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);

	return count;
}

static ssize_t trig_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	for (i = 0; i < AW_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d, pos_enable=%d, pos_sequence=%d, neg_enable=%d, neg_sequence=%d trig_brk=%d, trig_bst=%d\n",
				i + 1,
				aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar,
				aw_haptic->trig[i].pos_enable,
				aw_haptic->trig[i].pos_sequence,
				aw_haptic->trig[i].neg_enable,
				aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk,
				aw_haptic->trig[i].trig_bst);
	}

	return len;
}

static ssize_t trig_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	uint32_t databuf[9] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%u %u %u %u %u %u %u %u %u", &databuf[0], &databuf[1], &databuf[2],
	    &databuf[3], &databuf[4], &databuf[5], &databuf[6], &databuf[7], &databuf[8]) == 9) {
		aw_info("%d, %d, %d, %d, %d, %d, %d, %d, %d", databuf[0], databuf[1], databuf[2],
			databuf[3], databuf[4], databuf[5], databuf[6], databuf[7], databuf[8]);
		if (databuf[0] < 1 || databuf[0] > 3) {
			aw_info("input trig_num out of range!");
			return count;
		}
		if (databuf[0] == 1 && aw_haptic->info.is_enabled_one_wire) {
			aw_info("trig1 pin used for one wire!");
			return count;
		}
		if (!aw_haptic->ram_init) {
			aw_err("ram init failed, not allow to play!");
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num || databuf[6] > aw_haptic->ram.ram_num) {
			aw_err("input seq value out of range!");
			return count;
		}
		databuf[0] -= 1;

		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		aw_haptic->trig[databuf[0]].trig_bst = databuf[8];
		mutex_lock(&aw_haptic->lock);
		switch (databuf[0]) {
		case 0:
			aw8672x_trig1_param_config(aw_haptic);
			break;
		case 1:
			aw8672x_trig2_param_config(aw_haptic);
			break;
		case 2:
			aw8672x_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t rtp_auto_sin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %u\n", aw_haptic->rtp_cnt);

	return len;
}

static ssize_t rtp_auto_sin_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_err("kstrtouint fail");
		return rc;
	}
	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < aw_haptic->rtp_num)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_RTPCFG1,
					 AW8672X_BIT_RTPCFG1_RTP_AUTO_SIN_MASK,
					 AW8672X_BIT_RTPCFG1_RTP_AUTO_SIN_ENABLE);
	} else if (val == 0) {
		aw_haptic->state = 0;
		haptic_hv_i2c_write_bits(aw_haptic, AW8672X_REG_RTPCFG1,
					 AW8672X_BIT_RTPCFG1_RTP_AUTO_SIN_MASK,
					 AW8672X_BIT_RTPCFG1_RTP_AUTO_SIN_DISABLE);
	} else {
		aw_haptic->state = 0;
		aw_err("input number error:%d", val);
	}
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);

	return count;
}

static DEVICE_ATTR_RW(cont_drv_lvl);
static DEVICE_ATTR_RW(cont_drv_time);
static DEVICE_ATTR_RW(cont_brk_time);
static DEVICE_ATTR_RW(trig);
static DEVICE_ATTR_RW(rtp_auto_sin);

static struct attribute *aw8672x_vibrator_attributes[] = {
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	&dev_attr_rtp_auto_sin.attr,
	NULL
};

static struct attribute_group aw8672x_vibrator_attribute_group = {
	.attrs = aw8672x_vibrator_attributes
};

static int aw8672x_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj, &aw8672x_vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error create aw8672x sysfs attr files");
		return ret;
	}

	return 0;
}

struct aw_haptic_func aw8672x_func_list = {
	.play_stop = aw8672x_stop,
	.ram_init = aw8672x_ram_init,
	.get_vbat = aw8672x_get_vbat,
	.creat_node = aw8672x_creat_node,
	.get_f0 = aw8672x_get_f0,
	.ram_get_f0 = aw8672x_ram_get_f0,
	.cont_config = aw8672x_cont_config,
	.offset_cali = aw8672x_offset_cali,
	.get_irq_state = aw8672x_get_irq_state,
	.check_qualify = aw8672x_check_qualify,
	.judge_rtp_going = aw8672x_judge_rtp_going,
	.set_bst_peak_cur = aw8672x_set_bst_peak_cur,
	.get_theory_time = aw8672x_get_theory_time,
	.get_lra_resistance = aw8672x_get_lra_resistance,
	.set_pwm = aw8672x_set_pwm,
	.play_mode = aw8672x_play_mode,
	.set_bst_vol = aw8672x_set_bst_vol,
	.interrupt_setup = aw8672x_interrupt_setup,
	.set_repeat_seq = aw8672x_set_repeat_seq,
	.auto_bst_enable = aw8672x_auto_bst_enable,
	.vbat_mode_config = aw8672x_vbat_mode_config,
	.set_wav_seq = aw8672x_set_wav_seq,
	.set_wav_loop = aw8672x_set_wav_loop,
	.set_ram_addr = aw8672x_set_ram_addr,
	.set_rtp_data = aw8672x_set_rtp_data,
	.container_update = aw8672x_container_update,
	.protect_config = aw8672x_protect_config,
	.parse_dt = aw8672x_parse_dt,
	.trig_init = aw8672x_trig_init,
	.irq_clear = aw8672x_irq_clear,
	.get_wav_loop = aw8672x_get_wav_loop,
	.play_go = aw8672x_play_go,
	.misc_para_init = aw8672x_misc_para_init,
	.set_rtp_aei = aw8672x_set_rtp_aei,
	.set_gain = aw8672x_set_gain,
	.upload_lra = aw8672x_upload_lra,
	.bst_mode_config = aw8672x_bst_mode_config,
	.get_reg = aw8672x_get_reg,
	.get_prctmode = aw8672x_get_prctmode,
	.get_trim_lra = aw8672x_get_trim_lra,
	.get_ram_data = aw8672x_get_ram_data,
	.get_first_wave_addr = aw8672x_get_first_wave_addr,
	.get_glb_state = aw8672x_get_glb_state,
	.get_osc_status = aw8672x_get_osc_status,
	.rtp_get_fifo_afs = aw8672x_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw8672x_rtp_get_fifo_aes,
	.get_wav_seq = aw8672x_get_wav_seq,
#ifdef AW_SND_SOC_CODEC
	.snd_soc_init = aw8672x_snd_soc_init,
#endif
};
