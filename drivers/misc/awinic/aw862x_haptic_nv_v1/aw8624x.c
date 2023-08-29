// SPDX-License-Identifier: GPL-2.0
/*
 * aw8624x.c
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * Author: <wangzhi@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
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
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include "haptic_nv.h"
#include "haptic_nv_reg.h"

static void aw8624x_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* edge int mode */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL5,
				 (AW8624X_BIT_SYSCTRL5_INT_MODE_MASK &
				  AW8624X_BIT_SYSCTRL5_INT_EDGE_MODE_MASK),
				 (AW8624X_BIT_SYSCTRL5_INT_MODE_EDGE |
				  AW8624X_BIT_SYSCTRL5_INT_EDGE_MODE_POS));
	/* int enable */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSINTM,
				 (AW8624X_BIT_SYSINTM_UVLM_MASK & AW8624X_BIT_SYSINTM_FF_AEM_MASK &
				  AW8624X_BIT_SYSINTM_FF_AFM_MASK & AW8624X_BIT_SYSINTM_OCDM_MASK &
				  AW8624X_BIT_SYSINTM_OTM_MASK & AW8624X_BIT_SYSINTM_DONEM_MASK),
				 (AW8624X_BIT_SYSINTM_UVLM_ON | AW8624X_BIT_SYSINTM_FF_AEM_OFF |
				  AW8624X_BIT_SYSINTM_FF_AFM_OFF | AW8624X_BIT_SYSINTM_OCDM_ON |
				  AW8624X_BIT_SYSINTM_OTM_ON | AW8624X_BIT_SYSINTM_DONEM_OFF));
}

static void aw8624x_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data, uint32_t len)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_RTPDATA, data, len);
}

static uint8_t aw8624x_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_GLBRD5, &state, AW_I2C_BYTE_ONE);

	return state;
}

static int aw8624x_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);
	if (reg_val & AW8624X_BIT_SYSINT_UVLI)
		aw_err("chip uvlo int error");
	if (reg_val & AW8624X_BIT_SYSINT_OCDI)
		aw_err("chip over current int error");
	if (reg_val & AW8624X_BIT_SYSINT_OTI)
		aw_err("chip over temperature int error");
	if (reg_val & AW8624X_BIT_SYSINT_DONEI)
		aw_info("chip playback done");
	if (reg_val & AW8624X_BIT_SYSINT_FF_AFI)
		aw_info("rtp mode fifo almost full!");
	if (reg_val & AW8624X_BIT_SYSINT_FF_AEI) {
		aw_info("rtp fifo almost empty");
		ret = AW_IRQ_ALMOST_EMPTY;
	}

	return ret;
}

static uint8_t aw8624x_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8624X_BIT_SYSST_FF_AFS;
	reg_val = reg_val >> 3;

	return reg_val;
}

static uint8_t aw8624x_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8624X_BIT_SYSST_FF_AES;
	reg_val = reg_val >> 4;

	return reg_val;
}

static void aw8624x_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSINTM,
					 AW8624X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8624X_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSINTM,
					 AW8624X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8624X_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8624x_auto_brk_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8624X_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8624X_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static void aw8624x_config(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_ANACFG8,
				 AW8624X_BIT_ANACFG8_TRTF_CTRL_HDRV_MASK,
				 AW8624X_BIT_ANACFG8_TRTF_CTRL_HDRV);
	if (aw_haptic->info.cont_brk_time) {
		reg_val = (uint8_t)aw_haptic->info.cont_brk_time;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG10, &reg_val, AW_I2C_BYTE_ONE);
	} else {
		aw_err("dts_info.cont_brk_time=0");
	}
	if (aw_haptic->info.cont_brk_gain) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG5,
					 AW8624X_BIT_CONTCFG5_BRK_GAIN_MASK,
					 aw_haptic->info.cont_brk_gain);
	} else {
		aw_err("dts_info.cont_brk_gain=0");
	}

}

static void aw8624x_play_stop(struct aw_haptic *aw_haptic)
{
	bool force_flag = true;
	uint8_t val = 0;
	int cnt = 40;

	aw_info("enter");
	aw_haptic->play_mode = AW_STANDBY_MODE;
	val = AW8624X_BIT_PLAYCFG4_STOP_ON;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PLAYCFG4, &val, AW_I2C_BYTE_ONE);
	while (cnt) {
		haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_GLBRD5, &val, AW_I2C_BYTE_ONE);
		if ((val & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY) {
			force_flag = false;
			aw_info("entered standby! glb_state=0x%02X", val);
			break;
		}
		cnt--;
		usleep_range(AW_STOP_DELAY_MIN, AW_STOP_DELAY_MAX);
	}
	if (force_flag) {
		aw_err("force to enter standby mode!");
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2,
					 AW8624X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8624X_BIT_SYSCTRL2_STANDBY_ON);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2,
					 AW8624X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8624X_BIT_SYSCTRL2_STANDBY_OFF);
	}
}

static void aw8624x_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2,
					 AW8624X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8624X_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW_PWM_24K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2,
					 AW8624X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8624X_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW_PWM_12K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2,
					 AW8624X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8624X_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
}

static void aw8624x_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		aw8624x_play_stop(aw_haptic);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		aw8624x_set_pwm(aw_haptic, AW_PWM_12K);
		aw8624x_auto_brk_config(aw_haptic, false);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		aw8624x_set_pwm(aw_haptic, AW_PWM_12K);
		aw8624x_auto_brk_config(aw_haptic, true);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		aw8624x_auto_brk_config(aw_haptic, aw_haptic->info.is_enabled_auto_brk);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_CONT);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		aw8624x_set_pwm(aw_haptic, AW_PWM_24K);
		aw8624x_auto_brk_config(aw_haptic, true);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_RTP);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		aw8624x_set_pwm(aw_haptic, AW_PWM_12K);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8624X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	default:
		aw_err("play mode %u error", play_mode);
		break;
	}
}

static void aw8624x_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSINT, &val, AW_I2C_BYTE_ONE);
	aw_info("SYSINT=0x%02X", val);
}

static void aw8624x_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t val = 0;

	aw_info("enter");
	if (flag == true) {
		val = AW8624X_BIT_PLAYCFG4_GO_ON;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PLAYCFG4, &val, AW_I2C_BYTE_ONE);
	} else {
		val = AW8624X_BIT_PLAYCFG4_STOP_ON;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PLAYCFG4, &val, AW_I2C_BYTE_ONE);
	}
	val = aw8624x_get_glb_state(aw_haptic);
	aw_info("reg:0x%02X=0x%02X", AW8624X_REG_GLBRD5, val);
}

static void aw8624x_haptic_start(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw8624x_play_go(aw_haptic, true);
}

static void aw8624x_raminit(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL1,
					 AW8624X_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW8624X_BIT_SYSCTRL1_RAMINIT_ON);
		usleep_range(1000, 1050);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL1,
					 AW8624X_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW8624X_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw8624x_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t vbat_code = 0;

	aw_info("enter");
	aw8624x_play_stop(aw_haptic);
	aw8624x_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_DETCFG2, AW8624X_BIT_DETCFG2_VBAT_GO_MASK,
				 AW8624X_BIT_DETCFG2_VABT_GO_ON);
	usleep_range(AW_VBAT_DELAY_MIN, AW_VBAT_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_VBAT, &reg_val, AW_I2C_BYTE_ONE);
	vbat_code = (vbat_code | reg_val) << 2;
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_LO, &reg_val, AW_I2C_BYTE_ONE);
	vbat_code = vbat_code | ((reg_val & AW8624X_BIT_DET_LO_VBAT) >> 4);
	aw8624x_raminit(aw_haptic, false);
	aw_haptic->vbat = AW8624X_VBAT_FORMULA(vbat_code);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_info("vbat max limit = %dmV", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_info("vbat min limit = %dmV", aw_haptic->vbat);
	}
	aw_info("vbat=%dmV, vbat_code=0x%02X", aw_haptic->vbat, vbat_code);
}

static void aw8624x_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PLAYCFG2, &gain, AW_I2C_BYTE_ONE);
}

static void aw8624x_set_trim_lra(struct aw_haptic *aw_haptic, uint8_t val)
{
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_TRIMCFG3,
				 AW8624X_BIT_TRIMCFG3_TRIM_LRA_MASK, val);
}

static void aw8624x_cont_config(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	/* work mode */
	aw8624x_play_mode(aw_haptic, AW_CONT_MODE);
	/* cont config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG6,
				 (AW8624X_BIT_CONTCFG6_TRACK_EN_MASK &
				 AW8624X_BIT_CONTCFG6_DRV1_LVL_MASK),
				 ((aw_haptic->info.is_enabled_track_en << 7) |
				 (uint8_t)aw_haptic->info.cont_drv1_lvl));
	reg_val = (uint8_t)aw_haptic->info.cont_drv2_lvl;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG7, &reg_val, AW_I2C_BYTE_ONE);
	/* DRV2_TIME */
	reg_val = 0xFF;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG9, &reg_val, AW_I2C_BYTE_ONE);
	/* cont play go */
	aw8624x_play_go(aw_haptic, true);
}

static ssize_t aw8624x_get_reg(struct aw_haptic *aw_haptic, ssize_t len, char *buf)
{
	len = haptic_nv_read_reg_array(aw_haptic, buf, len, AW8624X_REG_ID,
				       AW8624X_REG_RTPDATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len, AW8624X_REG_RTPDATA + 1,
				       AW8624X_REG_RAMDATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len, AW8624X_REG_RAMDATA + 1,
				       AW8624X_REG_ANACFG8);

	return len;
}

static void aw8624x_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime, uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PWMCFG1, AW8624X_BIT_PWMCFG1_PRC_EN_MASK,
				 AW8624X_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW8624X_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW8624X_BIT_PWMCFG3_PRLVL_MASK));
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_PWMCFG4, &prtime, AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PWMCFG3,
					 AW8624X_BIT_PWMCFG3_PR_EN_MASK,
					 AW8624X_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw8624x_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t val = 0;
	uint8_t array[8] = {0};

	aw_info("enter");
	/* Get seq and gain */
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_WAVCFG1, &val, AW_I2C_BYTE_ONE);
	aw_haptic->index = val & AW8624X_BIT_WAVCFG_SEQ;
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_PLAYCFG2, &val, AW_I2C_BYTE_ONE);
	aw_haptic->gain = val;
	aw_info("gain=0x%02X, index=0x%02X", aw_haptic->gain, aw_haptic->index);
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_WAVCFG1, array, AW_SEQUENCER_SIZE);
	memcpy(aw_haptic->seq, array, AW_SEQUENCER_SIZE);
	/* GAIN_BYPASS config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL5,
				 AW8624X_BIT_SYSCTRL5_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass << 6);
	/* CONT SMART_LOOP config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG5,
				 AW8624X_BIT_CONTCFG5_SMART_LOOP_MASK,
				 aw_haptic->info.is_enabled_smart_loop << 4);
	/* RTP INT_BRK config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL1,
				 AW8624X_BIT_SYSCTRL1_INT_BRK_MASK,
				 aw_haptic->info.is_enabled_inter_brake << 6);

	/* d2s_gain */
	if (!aw_haptic->info.d2s_gain) {
		aw_err("dts_info.d2s_gain = 0!");
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL5,
					 AW8624X_BIT_SYSCTRL5_D2S_GAIN_MASK,
					 aw_haptic->info.d2s_gain);
	}

	if (!aw_haptic->info.f0_d2s_gain) {
		aw_err("dts_info.f0_d2s_gain = 0!");
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL6,
				 AW8624X_BIT_SYSCTRL6_F0_D2S_GAIN_MASK,
				 aw_haptic->info.f0_d2s_gain);
	}

	aw_haptic->info.cont_drv2_lvl = AW8624X_DRV2_LVL_FORMULA(aw_haptic->info.f0_pre,
								 aw_haptic->info.lra_vrms);
	aw_info("lra_vrms=%u, cont_drv2_lvl=0x%02X", aw_haptic->info.lra_vrms,
		aw_haptic->info.cont_drv2_lvl);
	if (aw_haptic->info.cont_drv2_lvl > AW8624X_DRV2_LVL_MAX) {
		aw_err("cont_drv2_lvl[0x%02X] is error, restore max vale[0x%02X]",
		       aw_haptic->info.cont_drv2_lvl, AW8624X_DRV2_LVL_MAX);
		aw_haptic->info.cont_drv2_lvl = AW8624X_DRV2_LVL_MAX;
	}
	aw8624x_config(aw_haptic);
	aw8624x_set_pwm(aw_haptic, AW_PWM_12K);
	aw8624x_protect_config(aw_haptic, AW8624X_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW8624X_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
}

static int aw8624x_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_5:
		d2s_gain = 5;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW8624X_BIT_SYSCTRL5_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}

	return d2s_gain;
}

static int aw8624x_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	int os_code = 0;
	int d2s_gain = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSCTRL5, reg_val, AW_I2C_BYTE_ONE);
	reg_val[0] = (~AW8624X_BIT_SYSCTRL5_D2S_GAIN_MASK) & reg_val[0];
	d2s_gain = aw8624x_select_d2s_gain(reg_val[0]);
	if (d2s_gain < 0) {
		aw_err("d2s_gain is error");
		return -ERANGE;
	}
	aw8624x_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_DETCFG1, AW8624X_BIT_DETCFG1_RL_OS_MASK,
				 AW8624X_BIT_DETCFG1_OS);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_DETCFG2, AW8624X_BIT_DETCFG2_DIAG_GO_MASK,
				 AW8624X_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(3000, 3500);
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_OS, &reg_val[0], AW_I2C_BYTE_ONE);
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_LO, &reg_val[1], AW_I2C_BYTE_ONE);
	aw8624x_raminit(aw_haptic, false);
	os_code = (reg_val[1] & (~AW8624X_BIT_DET_LO_OS_MASK)) >> 2;
	os_code = (reg_val[0] << 2) | os_code;
	os_code = AW8624X_OS_FORMULA(os_code, d2s_gain);
	aw_info("os_code is %d mV", os_code);
	if (os_code > 15 || os_code < -15)
		return -ERANGE;

	return 0;
}

static void aw8624x_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t val = 0;

	aw_info("enter");
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		val = AW8624X_BIT_GLBCFG2_START_DLY_250US;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_GLBCFG2, &val, AW_I2C_BYTE_ONE);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL1,
					 AW8624X_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW8624X_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL1,
					 AW8624X_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW8624X_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
}

static void aw8624x_calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	f0_cali_step = 100000 * ((int)aw_haptic->f0 - (int)aw_haptic->info.f0_pre) /
				((int)aw_haptic->info.f0_pre * AW8624X_F0_CALI_ACCURACY);
	aw_info("f0_cali_step=%d", f0_cali_step);
	if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = 64 + (f0_cali_step / 10 + 1);
		else
			f0_cali_step = 64 + f0_cali_step / 10;
	} else {	/* f0_cali_step < 0 */
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = 64 + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = 64 + f0_cali_step / 10;
	}
	if (f0_cali_step >= 64)
		f0_cali_lra = (char)f0_cali_step - 64;
	else
		f0_cali_lra = (char)f0_cali_step + 64;
	/* update cali step */
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_info("f0_cali_data=0x%02X", aw_haptic->f0_cali_data);
}

static void aw8624x_read_cont_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;

	aw_info("enter");
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_CONTCFG13, val, AW_I2C_BYTE_TWO);
	f0_reg = (f0_reg | val[0]) << 8;
	f0_reg |= (val[1] << 0);
	if (!f0_reg) {
		aw_err("didn't get cont f0 because f0_reg value is 0!");
		aw_haptic->cont_f0 = 0;
		return;
	}
	aw_haptic->cont_f0 = AW8624X_F0_FORMULA(f0_reg);
	aw_info("cont_f0=%u", aw_haptic->cont_f0);
}

static int aw8624x_read_lra_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;

	aw_info("enter");
	/* F_LRA_F0 */
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_CONTCFG11, val, AW_I2C_BYTE_TWO);
	f0_reg = (f0_reg | val[0]) << 8;
	f0_reg |= (val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("didn't get lra f0 because f0_reg value is 0!");
		return -ERANGE;
	}
	aw_haptic->f0 = AW8624X_F0_FORMULA(f0_reg);
	aw_info("lra_f0=%u", aw_haptic->f0);

	return 0;
}

static int aw8624x_get_f0(struct aw_haptic *aw_haptic)
{
	bool get_f0_flag = false;
	uint8_t brk_en_temp = 0;
	uint8_t val[3] = {0};
	int drv_width = 0;
	int cnt = 200;
	int ret = 0;

	aw_info("enter");
	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8624x_play_stop(aw_haptic);
	/* f0 calibrate work mode */
	aw8624x_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG1,
				 AW8624X_BIT_CONTCFG1_PEAK_NUM_MASK,
				 AW8624X_BIT_CONTCFG1_PEAK_NUM_3);
	/* cont config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG6,
				 AW8624X_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto brake */
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_PLAYCFG3, &val[0], AW_I2C_BYTE_ONE);
	brk_en_temp = AW8624X_BIT_PLAYCFG3_BRK & val[0];
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3, AW8624X_BIT_PLAYCFG3_BRK_EN_MASK,
				 AW8624X_BIT_PLAYCFG3_BRK_ENABLE);
	/* f0 driver level */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG6,
				 AW8624X_BIT_CONTCFG6_DRV1_LVL_MASK, aw_haptic->info.cont_drv1_lvl);
	val[0] = (uint8_t)aw_haptic->info.cont_drv2_lvl;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG7, val, AW_I2C_BYTE_ONE);
	val[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
	val[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG8, val, AW_I2C_BYTE_TWO);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("dts_info.cont_track_margin = 0!");
	} else {
		val[0] = (uint8_t)aw_haptic->info.cont_track_margin;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG4, &val[0], AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (!aw_haptic->info.f0_pre)
		return -ERANGE;
	drv_width = AW8624X_DRV_WIDTH_FORMULA(aw_haptic->info.f0_pre,
					      aw_haptic->info.cont_track_margin,
					      aw_haptic->info.cont_brk_gain);
	if (drv_width < AW_DRV_WIDTH_MIN)
		drv_width = AW_DRV_WIDTH_MIN;
	if (drv_width > AW_DRV_WIDTH_MAX)
		drv_width = AW_DRV_WIDTH_MAX;
	val[0] = (uint8_t)drv_width;
	aw_info("cont_drv_width=0x%02X", val[0]);
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG3, &val[0], AW_I2C_BYTE_ONE);
	/* cont play go */
	aw8624x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	while (cnt) {
		haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_GLBRD5, &val[0], AW_I2C_BYTE_ONE);
		if ((val[0] & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY) {
			get_f0_flag = true;
			aw_info("entered standby! glb_state=0x%02X", val[0]);
			break;
		}
		cnt--;
		aw_dbg("waitting for standby,glb_state=0x%02X", val[0]);
		usleep_range(AW_F0_DELAY_MIN, AW_F0_DELAY_MAX);
	}
	if (get_f0_flag) {
		ret = aw8624x_read_lra_f0(aw_haptic);
		if (ret < 0)
			aw_err("read lra f0 is failed");
		aw8624x_read_cont_f0(aw_haptic);
	} else {
		ret = -ERANGE;
		aw_err("enter standby mode failed, stop reading f0!");
	}
	/* disable f0 detect */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG1,
				 AW8624X_BIT_CONTCFG1_PEAK_NUM_MASK,
				 AW8624X_BIT_CONTCFG1_PEAK_NUM_NO_F0_DET);
	/* recover auto break config */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_PLAYCFG3,
				 AW8624X_BIT_PLAYCFG3_BRK_EN_MASK, brk_en_temp);

	return ret;
}

static void aw8624x_set_base_addr(struct aw_haptic *aw_haptic)
{
	uint32_t base_addr = 0;
	uint8_t val = 0;

	aw_info("enter");
	base_addr = aw_haptic->ram.base_addr;
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_RTPCFG1, AW8624X_BIT_RTPCFG1_ADDRH_MASK,
				 (uint8_t)AW_SET_BASEADDR_H(base_addr));
	val = (uint8_t)AW_SET_BASEADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_RTPCFG2, &val, AW_I2C_BYTE_ONE);
}

static void aw8624x_set_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t val[3] = {0};

	aw_info("enter");
	ae_addr_h = (uint8_t)AW8624X_SET_AEADDR_H(aw_haptic->ram.base_addr);
	af_addr_h = (uint8_t)AW8624X_SET_AFADDR_H(aw_haptic->ram.base_addr);
	val[0] = ae_addr_h | af_addr_h;
	val[1] = (uint8_t)AW8624X_SET_AEADDR_L(aw_haptic->ram.base_addr);
	val[2] = (uint8_t)AW8624X_SET_AFADDR_L(aw_haptic->ram.base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_RTPCFG3, val, AW_I2C_BYTE_THREE);
}

static void aw8624x_get_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t ae_addr_l = 0;
	uint8_t af_addr_l = 0;
	uint8_t val[3] = {0};

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_RTPCFG3, val, AW_I2C_BYTE_THREE);
	ae_addr_h = ((val[0]) & AW8624X_BIT_RTPCFG3_FIFO_AEH) >> 4;
	ae_addr_l = val[1];
	af_addr_h = ((val[0]) & AW8624X_BIT_RTPCFG3_FIFO_AFH);
	af_addr_l = val[2];
	aw_info("almost_empty_threshold = %u,almost_full_threshold = %u",
		(uint16_t)((ae_addr_h << 8) | ae_addr_l),
		(uint16_t)((af_addr_h << 8) | af_addr_l));
}

static void aw8624x_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	aw_info("enter");
	val[0] = (uint8_t)AW_SET_RAMADDR_H(base_addr);
	val[1] = (uint8_t)AW_SET_RAMADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_RAMADDRH, val, AW_I2C_BYTE_TWO);
}

static void aw8624x_set_ram_data(struct aw_haptic *aw_haptic, uint8_t *data, int len)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_RAMDATA, data, len);
}

static void aw8624x_get_first_wave_addr(struct aw_haptic *aw_haptic, uint32_t *first_wave_addr)
{
	uint8_t val[3] = {0};

	aw_info("enter");
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_RAMDATA, val, AW_I2C_BYTE_THREE);
	*first_wave_addr = (val[1] << 8 | val[2]);
}

static void aw8624x_haptic_trig_param_init(struct aw_haptic *aw_haptic, uint8_t pin)
{
	struct aw_haptic_dts_info info = aw_haptic->info;

	switch (pin) {
	case AW_TRIG1:
		aw_haptic->trig[0].trig_level = info.trig_cfg[0];
		aw_haptic->trig[0].trig_polar = info.trig_cfg[1];
		aw_haptic->trig[0].pos_enable = info.trig_cfg[2];
		aw_haptic->trig[0].pos_sequence = info.trig_cfg[3];
		aw_haptic->trig[0].neg_enable = info.trig_cfg[4];
		aw_haptic->trig[0].neg_sequence = info.trig_cfg[5];
		aw_haptic->trig[0].trig_brk = info.trig_cfg[6];
		break;
	default:
		break;
	}
}

static int aw8624x_haptic_trig_param_config(struct aw_haptic *aw_haptic, uint8_t pin)
{
	uint8_t trig_polar_lev_brk = 0x00;
	uint8_t trig_pos_seq = 0x00;
	uint8_t trig_neg_seq = 0x00;

	switch (pin) {
	case AW_TRIG1:
		trig_polar_lev_brk = aw_haptic->trig[0].trig_polar << 2 |
				     aw_haptic->trig[0].trig_level << 1 |
				     aw_haptic->trig[0].trig_brk;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_TRGCFG7,
					 AW8624X_BIT_TRGCFG7_TRG1_POR_LEV_BRK_MASK,
					 trig_polar_lev_brk << 5);
		trig_pos_seq = aw_haptic->trig[0].pos_enable << 7 | aw_haptic->trig[0].pos_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_TRGCFG1,
				     &trig_pos_seq, AW_I2C_BYTE_ONE);
		trig_neg_seq = aw_haptic->trig[0].neg_enable << 7 | aw_haptic->trig[0].neg_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_TRGCFG4,
				     &trig_neg_seq, AW_I2C_BYTE_ONE);
		aw_info("trig1 config ok!");
		break;
	default:
		break;
	}

	return 0;
}

static void aw8624x_set_trig(struct aw_haptic *aw_haptic, uint8_t pin)
{
	aw_info("enter");
	aw8624x_haptic_trig_param_init(aw_haptic, pin);
	aw8624x_haptic_trig_param_config(aw_haptic, pin);
}

static void aw8624x_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw8624x_set_trig(aw_haptic, AW_TRIG1);
}

static void aw8624x_parse_dts(struct aw_haptic *aw_haptic, struct device_node *np)
{
	uint32_t duration_time[3];
	uint32_t trig_config_temp[21];
	uint32_t val = 0;

	val = of_property_read_u32(np, "aw8624x_gain_bypass", &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_info("aw8624x_gain_bypass not found");
	val = of_property_read_u32(np, "aw8624x_vib_lk_f0_cali", &aw_haptic->info.lk_f0_cali);
	if (val != 0)
		aw_info("aw8624x_vib_lk_f0_cali not found");
	val = of_property_read_u32(np, "aw8624x_vib_mode", &aw_haptic->info.mode);
	if (val != 0)
		aw_info("aw8624x_vib_mode not found");
	val = of_property_read_u32(np, "aw8624x_vib_f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("vib_f0_pre not found");
	val = of_property_read_u32(np, "aw8624x_vib_f0_cali_percen",
				   &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("vib_f0_cali_percent not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_drv1_lvl", &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_info("vib_cont_drv1_lvl not found");
	val = of_property_read_u32(np, "aw8624x_vib_lra_vrms", &aw_haptic->info.lra_vrms);
	if (val != 0)
		aw_info("vib_cont_lra_vrms not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_brk_time", &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_info("vib_cont_brk_time not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_brk_gain", &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_info("vib_cont_brk_gain not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_drv1_time",
				   &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_info("vib_cont_drv1_time not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_drv2_time",
				   &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_info("vib_cont_drv2_time not found");
	val = of_property_read_u32(np, "aw8624x_vib_cont_track_margin",
				   &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_info("vib_cont_track_margin not found");
	val = of_property_read_u32(np, "aw8624x_vib_d2s_gain", &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_info("vib_d2s_gain not found");
	val = of_property_read_u32(np, "aw8624x_vib_f0_d2s_gain", &aw_haptic->info.f0_d2s_gain);
	if (val != 0)
		aw_err("vib_f0_d2s_gain not found");
	val = of_property_read_u32_array(np, "aw8624x_vib_trig_config", trig_config_temp,
					 ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("vib_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp, sizeof(trig_config_temp));
	val = of_property_read_u32_array(np, "aw8624x_vib_duration_time", duration_time,
					 ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw8624x_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time, sizeof(duration_time));
	aw_haptic->info.is_enabled_track_en =
		of_property_read_bool(np, "aw8624x_vib_is_enabled_track_en");
	aw_info("aw_haptic->info.is_enabled_track_en = %d", aw_haptic->info.is_enabled_track_en);
	aw_haptic->info.is_enabled_smart_loop =
		of_property_read_bool(np, "aw8624x_vib_is_enabled_smart_loop");
	aw_info("aw_haptic->info.is_enabled_smart_loop = %d",
					aw_haptic->info.is_enabled_smart_loop);
	aw_haptic->info.is_enabled_inter_brake =
		of_property_read_bool(np, "aw8624x_vib_is_enabled_inter_brake");
	aw_info("aw_haptic->info.is_enabled_inter_brake = %d",
					aw_haptic->info.is_enabled_inter_brake);
	aw_haptic->info.is_enabled_auto_brk =
		of_property_read_bool(np, "aw8624x_vib_is_enabled_auto_brk");
	aw_info("aw_haptic->info.is_enabled_auto_brk = %d",
					aw_haptic->info.is_enabled_auto_brk);
}

static void aw8624x_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_WAVCFG1, aw_haptic->seq, len);
}

static void aw8624x_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t seq)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_WAVCFG1 + wav, &seq, AW_I2C_BYTE_ONE);
}

static void aw8624x_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_WAVCFG9 + (wav / 2),
					 AW8624X_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_WAVCFG9 + (wav / 2),
					 AW8624X_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw8624x_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8624x_set_wav_seq(aw_haptic, 0x00, seq);
	aw8624x_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8624x_set_wav_loop(aw_haptic, 0x00, AW8624X_BIT_WAVLOOP_INIFINITELY);
}

static void aw8624x_get_gain(struct aw_haptic *aw_haptic, uint8_t *gain)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_PLAYCFG2, gain, AW_I2C_BYTE_ONE);
}

static void aw8624x_get_wav_loop(struct aw_haptic *aw_haptic, uint8_t *val)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_WAVCFG9, val, AW_SEQUENCER_LOOP_SIZE);
}

static void aw8624x_get_ram_data(struct aw_haptic *aw_haptic, uint8_t *ram_data, int size)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_RAMDATA, ram_data, size);
}

static void aw8624x_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t lra_code = 0;
	int d2s_gain = 0;

	d2s_gain = aw8624x_select_d2s_gain(aw_haptic->info.d2s_gain);
	if (d2s_gain <= 0) {
		aw_err("d2s_gain is error");
		return;
	}
	aw8624x_raminit(aw_haptic, true);
	/* enter standby mode */
	aw8624x_play_stop(aw_haptic);
	usleep_range(AW_STOP_DELAY_MIN, AW_STOP_DELAY_MAX);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL2, AW8624X_BIT_SYSCTRL2_STANDBY_MASK,
				 AW8624X_BIT_SYSCTRL2_STANDBY_OFF);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_DETCFG1, AW8624X_BIT_DETCFG1_RL_OS_MASK,
				 AW8624X_BIT_DETCFG1_RL);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_DETCFG2, AW8624X_BIT_DETCFG2_DIAG_GO_MASK,
				 AW8624X_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(AW_RL_DELAY_MIN, AW_RL_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_RL, &reg_val, AW_I2C_BYTE_ONE);
	lra_code = (lra_code | reg_val) << 2;
	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_DET_LO, &reg_val, AW_I2C_BYTE_ONE);
	lra_code = lra_code | (reg_val & AW8624X_BIT_DET_LO_RL);
	aw8624x_raminit(aw_haptic, false);
	aw_haptic->lra = AW8624X_RL_FORMULA(lra_code, d2s_gain);
	aw_info("aw_haptic->lra = %u", aw_haptic->lra);
}

static uint8_t aw8624x_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
	reg_val >>= 7;

	return reg_val;
}

static uint8_t aw8624x_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8624x_get_glb_state(aw_haptic);
	if (glb_state == AW_BIT_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}

	return rtp_state;
}

static uint64_t aw8624x_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSCTRL2, &reg_val, AW_I2C_BYTE_ONE);
	fre_val = reg_val & AW8624X_BIT_SYSCTRL2_RATE;
	if (fre_val == AW8624X_BIT_SYSCTRL2_RATE_48K)
		theory_time = aw_haptic->rtp_len * 1000 / 48;	/*48K*/
	else if (fre_val == AW8624X_BIT_SYSCTRL2_RATE_24K)
		theory_time = aw_haptic->rtp_len * 1000 / 24;	/*24K*/
	else
		theory_time = aw_haptic->rtp_len * 1000 / 12 ;	/*12K*/
	aw_info("microsecond:%llu  theory_time = %llu", aw_haptic->microsecond, theory_time);

	return theory_time;
}

static uint8_t aw8624x_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_SYSST2, &state, AW_I2C_BYTE_ONE);
	state &= AW8624X_BIT_SYSST2_FF_EMPTY;

	return state;
}

static int aw8624x_check_qualify(struct aw_haptic *aw_haptic)
{
	uint8_t reg = 0;
	int ret = 0;

	aw_info("enter");
	ret = haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_EFRD9, &reg, AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if ((reg & 0x80) == 0x80)
		return 0;
	aw_err("register 0x64 error: 0x%02X", reg);

	return -ERANGE;
}

static ssize_t cont_drv_lvl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl, aw_haptic->info.cont_drv2_lvl);

	return len;
}

static ssize_t cont_drv_lvl_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG6,
					 AW8624X_BIT_CONTCFG6_DRV1_LVL_MASK,
					 (uint8_t)aw_haptic->info.cont_drv1_lvl);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_CONTCFG7,
					 AW8624X_BIT_CONTCFG7_DRV2_LVL_MASK,
					 (uint8_t)aw_haptic->info.cont_drv2_lvl);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t cont_drv_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time, aw_haptic->info.cont_drv2_time);

	return len;
}

static ssize_t cont_drv_time_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint8_t reg_val[2] = {0};
	uint32_t databuf[2] = { 0, 0 };

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_time = databuf[0];
		aw_haptic->info.cont_drv2_time = databuf[1];
		reg_val[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
		reg_val[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
		haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG8, reg_val, AW_I2C_BYTE_TWO);
	}
	mutex_unlock(&aw_haptic->lock);

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
	uint8_t reg_val = 0;
	int err, val;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	err = kstrtoint(buf, 16, &val);
	if (err != 0) {
		aw_err("format not match!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->info.cont_brk_time = val;
	reg_val = aw_haptic->info.cont_brk_time;
	haptic_nv_i2c_writes(aw_haptic, AW8624X_REG_CONTCFG10, &reg_val, AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t trig_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	ssize_t len = 0;
	uint8_t trig_num = 3;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	for (i = 0; i < trig_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "trig%d: trig_level=%u, trig_polar=%u",
				i + 1, aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar);
		len += snprintf(buf + len, PAGE_SIZE - len, "pos_enable=%u, pos_sequence=%u,",
				aw_haptic->trig[i].pos_enable, aw_haptic->trig[i].pos_sequence);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"neg_enable=%u, neg_sequence=%u trig_brk=%u\n",
				aw_haptic->trig[i].neg_enable, aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk);
	}

	return len;
}

static ssize_t trig_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[9] = { 0 };

	if (sscanf(buf, "%u %u %u %u %u %u %u %u", &databuf[0], &databuf[1], &databuf[2],
	    &databuf[3], &databuf[4], &databuf[5], &databuf[6], &databuf[7]) == 8) {
		aw_info("%u, %u, %u, %u, %u, %u, %u, %u", databuf[0], databuf[1], databuf[2],
			databuf[3], databuf[4], databuf[5], databuf[6], databuf[7]);
		if (databuf[0] < 0 || databuf[0] > 2) {
			aw_err("input seq value out of range!");
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
		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		mutex_lock(&aw_haptic->lock);
		aw8624x_haptic_trig_param_config(aw_haptic, databuf[0]);
		mutex_unlock(&aw_haptic->lock);
	} else
		aw_err("please input eight parameters");

	return count;
}

static ssize_t f0_d2s_gain_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	switch (val) {
	case 1:
		aw_haptic->info.f0_d2s_gain = 0;
		break;
	case 2:
		aw_haptic->info.f0_d2s_gain = 1;
		break;
	case 4:
		aw_haptic->info.f0_d2s_gain = 2;
		break;
	case 5:
		aw_haptic->info.f0_d2s_gain = 3;
		break;
	case 8:
		aw_haptic->info.f0_d2s_gain = 4;
		break;
	case 10:
		aw_haptic->info.f0_d2s_gain = 5;
		break;
	case 20:
		aw_haptic->info.f0_d2s_gain = 6;
		break;
	case 40:
		aw_haptic->info.f0_d2s_gain = 7;
		break;
	default:
		aw_err("err f0_d2s_gain param");
	}
	haptic_nv_i2c_write_bits(aw_haptic, AW8624X_REG_SYSCTRL6,
				 AW8624X_BIT_SYSCTRL6_F0_D2S_GAIN_MASK,
				 aw_haptic->info.f0_d2s_gain);
	mutex_unlock(&aw_haptic->lock);

	return count;
}


static ssize_t f0_d2s_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int f0_d2s_gain = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	f0_d2s_gain = aw8624x_select_d2s_gain(aw_haptic->info.f0_d2s_gain);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0_d2s_gain = %d\n", f0_d2s_gain);

	return len;
}

static DEVICE_ATTR_RW(cont_drv_lvl);
static DEVICE_ATTR_RW(cont_drv_time);
static DEVICE_ATTR_RW(cont_brk_time);
static DEVICE_ATTR_RW(trig);
static DEVICE_ATTR_RW(f0_d2s_gain);

static struct attribute *aw8624x_vibrator_attributes[] = {
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	&dev_attr_f0_d2s_gain.attr,
	NULL
};

static struct attribute_group aw8624x_vibrator_attribute_group = {
	.attrs = aw8624x_vibrator_attributes
};

static void aw8624x_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj, &aw8624x_vibrator_attribute_group);
	if (ret < 0)
		aw_err("error create aw8624x sysfs attr files");
}

struct aw_haptic_func aw8624x_func_list = {
	.ram_init = aw8624x_raminit,
	.parse_dts = aw8624x_parse_dts,
	.trig_init = aw8624x_trig_init,
	.play_mode = aw8624x_play_mode,
	.play_stop = aw8624x_play_stop,
	.irq_clear = aw8624x_irq_clear,
	.creat_node = aw8624x_creat_node,
	.cont_config = aw8624x_cont_config,
	.offset_cali = aw8624x_offset_cali,
	.haptic_start = aw8624x_haptic_start,
	.read_cont_f0 = aw8624x_read_cont_f0,
	.check_qualify = aw8624x_check_qualify,
	.judge_rtp_going = aw8624x_judge_rtp_going,
	.protect_config = aw8624x_protect_config,
	.misc_para_init = aw8624x_misc_para_init,
	.interrupt_setup = aw8624x_interrupt_setup,
	.rtp_get_fifo_afs = aw8624x_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw8624x_rtp_get_fifo_aes,
	.vbat_mode_config = aw8624x_vbat_mode_config,
	.calculate_cali_data = aw8624x_calculate_cali_data,
	.set_gain = aw8624x_set_gain,
	.get_gain = aw8624x_get_gain,
	.set_wav_seq = aw8624x_set_wav_seq,
	.get_wav_seq = aw8624x_get_wav_seq,
	.set_wav_loop = aw8624x_set_wav_loop,
	.get_wav_loop = aw8624x_get_wav_loop,
	.set_ram_data = aw8624x_set_ram_data,
	.get_ram_data = aw8624x_get_ram_data,
	.set_fifo_addr = aw8624x_set_fifo_addr,
	.get_fifo_addr = aw8624x_get_fifo_addr,
	.set_rtp_aei = aw8624x_set_rtp_aei,
	.set_rtp_data = aw8624x_set_rtp_data,
	.set_ram_addr = aw8624x_set_ram_addr,
	.set_trim_lra = aw8624x_set_trim_lra,
	.set_base_addr = aw8624x_set_base_addr,
	.set_repeat_seq = aw8624x_set_repeat_seq,
	.get_f0 = aw8624x_get_f0,
	.get_reg = aw8624x_get_reg,
	.get_vbat = aw8624x_get_vbat,
	.get_prctmode = aw8624x_get_prctmode,
	.get_irq_state = aw8624x_get_irq_state,
	.get_glb_state = aw8624x_get_glb_state,
	.get_osc_status = aw8624x_get_osc_status,
	.get_theory_time = aw8624x_get_theory_time,
	.get_lra_resistance = aw8624x_get_lra_resistance,
	.get_first_wave_addr = aw8624x_get_first_wave_addr,
};
