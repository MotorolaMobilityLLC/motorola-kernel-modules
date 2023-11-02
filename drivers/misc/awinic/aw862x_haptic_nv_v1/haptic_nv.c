// SPDX-License-Identifier: GPL-2.0
/*
 * File: haptic_nv.c
 *
 * Author: <chelvming@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/proc_fs.h>
#include <linux/mman.h>

#include "haptic_nv.h"
#include "haptic_nv_reg.h"

#define HAPTIC_NV_DRIVER_VERSION	"v1.12.0.10"

static char aw_ram_name[][AW_NAME_MAX] = {
	{"aw862x_haptic.bin"},
	{"aw862xx_haptic.bin"},
};

static char aw_rtp_name[][AW_NAME_MAX] = {
	{"aw862xx_rtp.bin"},
	{"aw862xx_rtp_Argo_Navis.bin"},
	{"aw862xx_rtp_Attentive.bin"},
	{"aw862xx_rtp_Awake.bin"},
	{"aw862xx_rtp_Bird_Loop.bin"},
	{"aw862xx_rtp_Brilliant_Times.bin"},
	{"aw862xx_rtp_Chimey_Phone.bin"},
	{"aw862xx_rtp_Complex.bin"},
	{"aw862xx_rtp_Crazy_Dream.bin"},
	{"aw862xx_rtp_Curve_Ball_Blend.bin"},
	{"aw862xx_rtp_Digital_Phone.bin"},
	{"aw862xx_rtp_Electrovision.bin"},
	{"aw862xx_rtp_Ether_Shake.bin"},
	{"aw862xx_rtp_Fateful_Words.bin"},
	{"aw862xx_rtp_Flutey_Phone.bin"},
	{"aw862xx_rtp_Future_Funk.bin"},
	{"aw862xx_rtp_Future_Hi_Tech.bin"},
	{"aw862xx_rtp_Girtab.bin"},
	{"aw862xx_rtp_Hello.bin"},
	{"aw862xx_rtp_Hexagon.bin"},
	{"aw862xx_rtp_Hydra.bin"},
	{"aw862xx_rtp_Insert_Coin.bin"},
	{"aw862xx_rtp_Jumping_Dots.bin"},
	{"aw862xx_rtp_Keys.bin"},
	{"aw862xx_rtp_Loopy.bin"},
	{"aw862xx_rtp_Loopy_Lounge.bin"},
	{"aw862xx_rtp_Modular.bin"},
	{"aw862xx_rtp_Momentum.bin"},
	{"aw862xx_rtp_Morning.bin"},
	{"aw862xx_rtp_Moto.bin"},
	{"aw862xx_rtp_Natural.bin"},
	{"aw862xx_rtp_New_Player.bin"},
	{"aw862xx_rtp_Onward.bin"},
	{"aw862xx_rtp_Organ_Dub.bin"},
	{"aw862xx_rtp_Overclocked.bin"},
	{"aw862xx_rtp_Pegasus.bin"},
	{"aw862xx_rtp_Pyxis.bin"},
	{"aw862xx_rtp_Regrade.bin"},
	{"aw862xx_rtp_Scarabaeus.bin"},
	{"aw862xx_rtp_Sceptrum.bin"},
	{"aw862xx_rtp_Simple.bin"},
	{"aw862xx_rtp_Solarium.bin"},
	{"aw862xx_rtp_Sparse.bin"},
	{"aw862xx_rtp_Terrabytes.bin"},
	{"aw862xx_rtp_Zero_Hour.bin"},
	{"aw862xx_rtp_Play.bin"},
	{"aw862xx_rtp_TJINGLE.bin"},
	{"aw862xx_rtp_Verizon_Airwaves.bin"},
	{"aw862xx_rtp_City_Lights.bin"},
	{"aw862xx_rtp_Firefly.bin"},
	{"aw862xx_rtp_Now_or_Never.bin"},
};

#ifdef AW_TIKTAP
static struct aw_haptic *g_aw_haptic;
#endif
#ifdef AW_DOUBLE
struct aw_haptic *left;
struct aw_haptic *right;
#endif

/*********************************************************
 *
 * I2C Read/Write
 *
 *********************************************************/
int haptic_nv_i2c_reads(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	int ret = -1;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw_haptic->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw_haptic->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
			},
	};

	ret = i2c_transfer(aw_haptic->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_err("transfer failed. reg_addr=0x%02X", reg_addr);
		return ret;
	} else if (ret != AW_I2C_READ_MSG_NUM) {
		aw_err("transfer failed(size error). reg_addr=0x%02X", reg_addr);
		return -ENXIO;
	}

	return ret;
}

int haptic_nv_i2c_writes(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	uint8_t *data = NULL;
	int ret = -1;

	data = kmalloc(len + 1, GFP_KERNEL);
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw_haptic->i2c, data, len + 1);
	if (ret < 0)
		aw_err("i2c master send 0x%02X err", reg_addr);
	kfree(data);

	return ret;
}

int haptic_nv_i2c_write_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint32_t mask,
			     uint8_t reg_data)
{
	uint8_t reg_val = 0;
	int ret = -1;

	ret = haptic_nv_i2c_reads(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = haptic_nv_i2c_writes(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c write error, ret=%d", ret);
		return ret;
	}

	return 0;
}

ssize_t haptic_nv_read_reg_array(struct aw_haptic *aw_haptic, char *buf, ssize_t len,
				 uint8_t head_reg_addr, uint8_t tail_reg_addr)
{
	int reg_num = 0;
	int i = 0;
	uint8_t reg_array[AW_REG_MAX] = {0};

	reg_num = tail_reg_addr - head_reg_addr + 1;
	haptic_nv_i2c_reads(aw_haptic, head_reg_addr, reg_array, reg_num);
	for (i = 0 ; i < reg_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02X=0x%02X\n",
				head_reg_addr + i, reg_array[i]);
	}

	return len;
}

static void pm_qos_enable(struct aw_haptic *aw_haptic, bool enable)
{
#ifdef KERNEL_OVER_5_10
	if (enable) {
		if (!cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_add_request(&aw_haptic->aw_pm_qos_req_vb,
						    CPU_LATENCY_QOC_VALUE);
	} else {
		if (cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#else
	if (enable) {
		if (!pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_add_request(&aw_haptic->aw_pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY,
					   AW_PM_QOS_VALUE_VB);
	} else {
		if (pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#endif
}

#ifdef AW_CHECK_RAM_DATA
static int parse_ram_data(struct aw_haptic *aw_haptic, uint32_t len, uint8_t *cont_data,
			  uint8_t *ram_data)
{
	int i = 0;

	aw_info("enter");
	for (i = 0; i < len; i++) {
		if (ram_data[i] != cont_data[i]) {
			aw_err("check ramdata error, addr=0x%04X, ram_data=0x%02X, file_data=0x%02X",
				i, ram_data[i], cont_data[i]);
			return -ERANGE;
		}
	}

	return 0;
}

static int check_ram_data(struct aw_haptic *aw_haptic, struct aw_haptic_container *aw_fw)
{
	int i = 0;
	int len = 0;
	int ret = 0;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};

	aw_haptic->func->set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < aw_fw->len) {
		if ((aw_fw->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = aw_fw->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		aw_haptic->func->get_ram_data(aw_haptic, ram_data, len);
		ret = parse_ram_data(aw_haptic, len, &aw_fw->data[i], ram_data);
		if (ret < 0)
			break;
		i += len;
	}

	return ret;
}
#endif

static void write_ram_data(struct aw_haptic *aw_haptic, struct aw_haptic_container *aw_fw)
{
	int i = 0;
	int len = 0;

	aw_haptic->func->set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < aw_fw->len) {
		if ((aw_fw->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = aw_fw->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		aw_haptic->func->set_ram_data(aw_haptic, &aw_fw->data[i], len);
		i += len;
	}
}

static int parse_dt_gpio(struct aw_haptic *aw_haptic, struct device *dev, struct device_node *np)
{
	aw_haptic->is_used_rst_pin = of_property_read_bool(np, "aw8624x_no_reset") ? 0 : 1;
	if (aw_haptic->is_used_rst_pin) {
		aw_haptic->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
		if (aw_haptic->reset_gpio < 0) {
			aw_err("no reset gpio provide");
			return -EPERM;
		}
		aw_info("reset gpio provide ok %d", aw_haptic->reset_gpio);
	} else {
		aw_info("not use rst pin");
	}

	aw_haptic->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw_haptic->irq_gpio < 0) {
		aw_err("no irq gpio provided.");
		aw_haptic->is_used_irq_pin = false;
	} else {
		aw_info("irq gpio provide ok irq = %d.", aw_haptic->irq_gpio);
		aw_haptic->is_used_irq_pin = true;
	}
#ifdef AW_DOUBLE
	if (of_device_is_compatible(np, "awinic,haptic_nv_l")) {
		aw_info("compatible left vibrator.");
		memcpy(aw_haptic->mark, "left", sizeof("left"));
		left = NULL;
		left = aw_haptic;
	} else if (of_device_is_compatible(np, "awinic,haptic_nv_r")) {
		aw_info("compatible right vibrator.");
		memcpy(aw_haptic->mark, "right", sizeof("right"));
		right = NULL;
		right = aw_haptic;
	} else {
		aw_err("compatible failed.");
		return -ERANGE;
	}
#endif

	return 0;
}

static void parse_dts_i2c_addr(struct aw_haptic *aw_haptic, struct device *dev,
			       struct device_node *np)
{
	uint32_t val = 0;

	val = of_property_read_u32(np, "aw862xx_i2c_addr", &aw_haptic->aw862xx_i2c_addr);
	if (val)
		aw_err("configure aw862xx_i2c_addr error");
	else
		aw_info("configure aw862xx_i2c_addr ok");
}

static void hw_reset(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	if (aw_haptic->is_used_rst_pin) {
		if (gpio_is_valid(aw_haptic->reset_gpio)) {
			gpio_set_value_cansleep(aw_haptic->reset_gpio, 0);
			usleep_range(1000, 2000);
			gpio_set_value_cansleep(aw_haptic->reset_gpio, 1);
			usleep_range(8000, 8500);
		} else {
			aw_err("failed");
		}
	} else {
		aw_info("aw_haptic->is_used_rst_pin is %d", aw_haptic->is_used_rst_pin);
	}
}

static void sw_reset(struct aw_haptic *aw_haptic)
{
	uint8_t reset = AW_BIT_RESET;

	aw_info("enter");
	haptic_nv_i2c_writes(aw_haptic, AW_REG_ID, &reset, AW_I2C_BYTE_ONE);
	usleep_range(2000, 2500);
}

static int read_chipid(struct aw_haptic *aw_haptic, uint32_t *reg_val, uint8_t type)
{
	uint8_t value[2] = {0};
	int cnt = 0;
	int ret = -1;

	while (cnt < AW_I2C_RETRIES) {
		ret = haptic_nv_i2c_reads(aw_haptic, AW_REG_CHIPIDH, &value[0], AW_I2C_BYTE_ONE);
		if (value[0] == AW8623X_CHIP_ID_H) {
			ret = haptic_nv_i2c_reads(aw_haptic, AW8623X_REG_CHIPIDL, &value[1],
						  AW_I2C_BYTE_ONE);
			*reg_val = value[1] | value[0] << 8;
		} else if (value[0] == AW8624X_CHIP_ID_H) {
			ret = haptic_nv_i2c_reads(aw_haptic, AW8624X_REG_CHIPIDL, &value[1],
						  AW_I2C_BYTE_ONE);
			*reg_val = value[1] | value[0] << 8;
		} else {
			ret = haptic_nv_i2c_reads(aw_haptic, AW_REG_ID, &value[0], AW_I2C_BYTE_ONE);
			*reg_val = value[0];
		}
		if (ret < 0) {
			if (type == AW_FIRST_TRY)
				aw_info("reading chip id");
			else if (type == AW_LAST_TRY)
				aw_err("i2c_read cnt=%d error=%d", cnt, ret);
			else
				aw_err("type is error");
		} else {
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int parse_chipid(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t reg = 0;
	uint32_t reg_val = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		/* hardware reset */
		hw_reset(aw_haptic);

		ret = read_chipid(aw_haptic, &reg_val, AW_FIRST_TRY);
		if (ret < 0) {
			aw_haptic->i2c->addr = (u16)aw_haptic->aw862xx_i2c_addr;
			aw_info("try to replace i2c addr [(0x%02X)] to read chip id again",
				aw_haptic->i2c->addr);
			ret = read_chipid(aw_haptic, &reg_val, AW_LAST_TRY);
			if (ret < 0)
				break;
		}

		switch (reg_val) {
#ifdef AW862X_DRIVER
		case AW8623_CHIP_ID:
			aw_haptic->name = AW8623;
			aw_info("detected aw8623.");
			return 0;
		case AW8624_CHIP_ID:
			aw_haptic->name = AW8624;
			aw_info("detected aw8624.");
			return 0;
#endif

#ifdef AW862XX_DRIVER
		case AW8622X_CHIP_ID:
			haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_EFRD9, &reg, AW_I2C_BYTE_ONE);
			if ((reg & 0x41) == AW86223_EF_ID) {
				aw_haptic->name = AW86223;
				aw_info("aw86223 detected");
				return 0;
			}
			if ((reg & 0x41) == AW86224_EF_ID) {
				aw_haptic->name = AW86224;
				aw_info("aw86224 or aw86225 detected");
				return 0;
			}
			aw_info("unsupported ef_id = (0x%02X)", reg);
			break;
		case AW86214_CHIP_ID:
			haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_EFRD9, &reg, AW_I2C_BYTE_ONE);
			if ((reg & 0x41) == AW86214_EF_ID) {
				aw_haptic->name = AW86214;
				aw_info("aw86214 detected");
				return 0;
			}
			aw_info("unsupported ef_id = (0x%02X)", reg);
			break;
#endif

#ifdef AW8623X_DRIVER
		case AW86233_CHIP_ID:
			aw_haptic->name = AW86233;
			aw_info("aw86233 detected");
			return 0;
		case AW86234_CHIP_ID:
			aw_haptic->name = AW86234;
			aw_info("aw86234 detected");
			return 0;
		case AW86235_CHIP_ID:
			aw_haptic->name = AW86235;
			aw_info("aw86235 detected");
			return 0;
#endif

#ifdef AW8624X_DRIVER
		case AW86243_CHIP_ID:
			aw_haptic->name = AW86243;
			aw_info("aw86243 detected");
			return 0;
		case AW86245_CHIP_ID:
			aw_haptic->name = AW86245;
			aw_info("aw86245 detected");
			return 0;
#endif
		default:
			aw_info("unsupport device revision (0x%02X)", reg_val);
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

static int chip_private_init(struct aw_haptic *aw_haptic)
{
	switch (aw_haptic->name) {
#ifdef AW862X_DRIVER
	case AW8623:
	case AW8624:
		aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
		aw_haptic->ram_file_num = AW862X_RAM_FILE;
		break;
#endif

#ifdef AW862XX_DRIVER
	case AW86214:
	case AW86223:
	case AW86224:
	case AW86225:
		aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
		aw_haptic->ram_file_num = AW862XX_RAM_FILE;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
		aw_haptic->ram_file_num = AW862XX_RAM_FILE;
		break;
#endif

#ifdef AW8624X_DRIVER
	case AW86243:
	case AW86245:
		aw_haptic->trim_lra_boundary = AW8624X_TRIM_LRA_BOUNDARY;
		aw_haptic->ram_file_num = AW862XX_RAM_FILE;
		break;
#endif

	default:
		aw_info("unexpected chip!");
		return -EINVAL;
	}

	return 0;
}

static int func_ptr_init(struct aw_haptic *aw_haptic, struct device *dev)
{
	switch (aw_haptic->name) {
#ifdef AW862X_DRIVER
	case AW8623:
	case AW8624:
		aw_haptic->func = &aw862x_func_list;
		break;
#endif

#ifdef AW862XX_DRIVER
	case AW86214:
	case AW86223:
	case AW86224:
	case AW86225:
		aw_haptic->func = &aw862xx_func_list;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		aw_haptic->func = &aw8623x_func_list;
		break;
#endif

#ifdef AW8624X_DRIVER
	case AW86243:
	case AW86245:
		aw_haptic->func = &aw8624x_func_list;
		break;
#endif

	default:
		aw_info("unexpected chip!");
		return -EINVAL;
	}

	return 0;
}

static int write_rtp_data(struct aw_haptic *aw_haptic)
{
	uint32_t buf_len = 0;
	int ret = -ERANGE;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	if (!aw_rtp) {
		aw_info("aw_rtp is null, break!");
		return ret;
	}

	if (aw_haptic->rtp_cnt < (aw_haptic->ram.base_addr)) {
		if ((aw_rtp->len - aw_haptic->rtp_cnt) < (aw_haptic->ram.base_addr))
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		else
			buf_len = aw_haptic->ram.base_addr;
	} else if ((aw_rtp->len - aw_haptic->rtp_cnt) < (aw_haptic->ram.base_addr >> 2)) {
		buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
	} else {
		buf_len = aw_haptic->ram.base_addr >> 2;
	}

#ifdef AW_ENABLE_RTP_PRINT_LOG
	aw_info("buf_len = %u", buf_len);
#endif
#ifdef AW_DOUBLE
	if (aw_haptic->rtp_cnt == 0 && aw_haptic->dual_flag) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			ret = down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
		aw_info("dual rtp play start");
	}
#endif
	aw_haptic->func->set_rtp_data(aw_haptic, &aw_rtp->data[aw_haptic->rtp_cnt], buf_len);
	aw_haptic->rtp_cnt += buf_len;

	return 0;

}

static int judge_rtp_load_end(struct aw_haptic *aw_haptic)
{
	uint8_t glb_st = 0;
	int ret = -ERANGE;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	glb_st = aw_haptic->func->get_glb_state(aw_haptic);

	if ((aw_haptic->rtp_cnt == aw_rtp->len) ||
	    ((glb_st & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY)) {
		if (aw_haptic->rtp_cnt != aw_rtp->len)
			aw_err("rtp play suspend!");
		else
			aw_info("rtp update complete!,cnt=%u", aw_haptic->rtp_cnt);
		aw_haptic->rtp_cnt = 0;
		aw_haptic->rtp_init = false;
		aw_haptic->func->set_rtp_aei(aw_haptic, false);
		ret = 0;
	}

	return ret;
}

static void rtp_play(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
	       && (aw_haptic->play_mode == AW_RTP_MODE)) {

#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("rtp cnt = %u", aw_haptic->rtp_cnt);
#endif
		ret = write_rtp_data(aw_haptic);
		if (ret != 0)
			break;
		ret = judge_rtp_load_end(aw_haptic);
		if (ret == 0)
			break;
	}
}

static irqreturn_t irq_handle(int irq, void *data)
{
	int irq_state = 0;
	struct aw_haptic *aw_haptic = data;

	aw_info("enter");

	do {
		irq_state = aw_haptic->func->get_irq_state(aw_haptic);

		if (irq_state == AW_IRQ_ALMOST_EMPTY) {
			if (aw_haptic->rtp_init) {
				mutex_lock(&aw_haptic->rtp_lock);
				rtp_play(aw_haptic);
				mutex_unlock(&aw_haptic->rtp_lock);
			} else {
				aw_info("rtp_init: %d", aw_haptic->rtp_init);
			}
		}

		if (aw_haptic->play_mode != AW_RTP_MODE)
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
	} while (irq_state);

	aw_info("exit");

	return IRQ_HANDLED;
}

static int irq_config(struct device *dev, struct aw_haptic *aw_haptic)
{
	int ret = -1;
	int irq_flags = 0;

	aw_info("enter");
	if (gpio_is_valid(aw_haptic->irq_gpio) && !(aw_haptic->flags & AW_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw_haptic->func->interrupt_setup(aw_haptic);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(dev, gpio_to_irq(aw_haptic->irq_gpio), NULL,
						irq_handle, irq_flags, "aw_haptic", aw_haptic);
		if (ret != 0) {
			aw_err("failed to request IRQ %d: %d",
			       gpio_to_irq(aw_haptic->irq_gpio), ret);
			return ret;
		}
	} else {
		aw_info("skipping IRQ registration");
		/* disable feature support if gpio was invalid */
		aw_haptic->flags |= AW_FLAG_SKIP_INTERRUPTS;
	}

	return 0;
}

static int get_ram_num(struct aw_haptic *aw_haptic)
{
	uint32_t first_wave_addr = 0;

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, ram_num = 0!");
		return -EPERM;
	}

	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_first_wave_addr(aw_haptic, &first_wave_addr);
	aw_haptic->ram.ram_num = (first_wave_addr - aw_haptic->ram.base_addr - 1) / 4;
	aw_info("first wave addr = 0x%04X, ram_num = %u", first_wave_addr, aw_haptic->ram.ram_num);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int container_update(struct aw_haptic *aw_haptic, struct aw_haptic_container *aw_fw)
{
	uint32_t shift = 0;
	int ret = 0;

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->ram_init(aw_haptic, true);
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr = (uint32_t)((aw_fw->data[0 + shift] << 8) |
					      (aw_fw->data[1 + shift]));
	aw_info("base_addr = %u", aw_haptic->ram.base_addr);
	if (!aw_haptic->ram.base_addr)
		aw_info("no set FIFO space for rtp play!");
	aw_haptic->func->set_base_addr(aw_haptic);
	aw_haptic->func->set_fifo_addr(aw_haptic);
	aw_haptic->func->get_fifo_addr(aw_haptic);
	write_ram_data(aw_haptic, aw_fw);

#ifdef AW_CHECK_RAM_DATA
	ret = check_ram_data(aw_haptic, aw_fw);
	if (ret)
		aw_err("ram data check sum error");
	else
		aw_info("ram data check sum pass");
#endif

	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");

	return ret;
}

static void ram_load(const struct firmware *cont, void *context)
{
	uint16_t check_sum = 0;
	int i = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = context;
	struct aw_haptic_container *aw_fw;
	uint32_t ram_file_num = aw_haptic->ram_file_num;
#ifdef AW_READ_BIN_FLEXBALLY
	static uint8_t load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif

	if (!cont) {
		aw_err("failed to read %s", aw_ram_name[ram_file_num]);
		release_firmware(cont);

#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw_haptic->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_info("start hrtimer:load_cont%u", load_cont);
		}
#endif
		return;
	}

	aw_info("loaded %s - size: %zu", aw_ram_name[ram_file_num], cont ? cont->size : 0);
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_err("check sum err: check_sum=0x%04X", check_sum);
		release_firmware(cont);
		return;
	}
	aw_info("check sum pass.");
	aw_haptic->ram.check_sum = check_sum;

	/* aw ram update */
	aw_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw_fw) {
		release_firmware(cont);
		aw_err("Error allocating memory");
		return;
	}
	aw_fw->len = cont->size;
	memcpy(aw_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = container_update(aw_haptic, aw_fw);
	if (ret) {
		aw_err("ram firmware update failed!");
	} else {
		aw_haptic->ram_init = true;
		aw_haptic->ram.len = aw_fw->len - aw_haptic->ram.ram_shift;

		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->trig_init(aw_haptic);
		mutex_unlock(&aw_haptic->lock);

		aw_info("ram firmware update complete!");
		get_ram_num(aw_haptic);
	}
	kfree(aw_fw);
	aw_fw = NULL;

}

static int ram_update(struct aw_haptic *aw_haptic)
{
	aw_haptic->ram_init = false;
	aw_haptic->rtp_init = false;

	return request_firmware_nowait(THIS_MODULE, 1, aw_ram_name[aw_haptic->ram_file_num],
				       aw_haptic->dev, GFP_KERNEL, aw_haptic, ram_load);
}

static void ram_play(struct aw_haptic *aw_haptic)
{
#ifdef AW_DOUBLE
	int ret = -1;

	if (aw_haptic->activate_mode == AW_RAM_MODE)
		aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
	aw_haptic->func->play_mode(aw_haptic, aw_haptic->activate_mode);
	if (aw_haptic->dual_flag) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			ret = down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
	}
#else
	aw_haptic->func->play_mode(aw_haptic, aw_haptic->activate_mode);
#endif
	aw_haptic->func->haptic_start(aw_haptic);
}

#ifdef AW_DURATION_DECIDE_WAVEFORM
static int analyse_duration_range(struct aw_haptic *aw_haptic)
{
	int i = 0;
	int ret = 0;
	int len = 0;
	int *duration_time = NULL;

	aw_info("enter");
	len = ARRAY_SIZE(aw_haptic->info.duration_time);
	duration_time = aw_haptic->info.duration_time;
	if (len < 2) {
		aw_err("duration time range error");
		return -ERANGE;
	}

	for (i = (len - 1); i > 0; i--) {
		if (duration_time[i] > duration_time[i-1])
			continue;
		else
			break;

	}

	if (i > 0) {
		aw_err("duration time range error");
		ret = -ERANGE;
	}

	return ret;
}

static int ram_config(struct aw_haptic *aw_haptic, int duration)
{
	uint8_t wavseq = 0;
	uint8_t wavloop = 0;
	int ret = 0;

	aw_info("enter");
	ret = analyse_duration_range(aw_haptic);
	if (ret < 0)
		return ret;

	if ((duration > 0) && (duration < aw_haptic->info.duration_time[0])) {
		wavseq = 3;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if ((duration >= aw_haptic->info.duration_time[0]) &&
		   (duration < aw_haptic->info.duration_time[1])) {
		wavseq = 2;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if ((duration >= aw_haptic->info.duration_time[1]) &&
		   (duration < aw_haptic->info.duration_time[2])) {
		wavseq = 1;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if (duration >= aw_haptic->info.duration_time[2]) {
		wavseq = 4;
		wavloop = 15;
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
	} else {
		aw_err("duration time error, duration= %d", duration);
		aw_haptic->activate_mode = AW_NULL;
		return -ERANGE;
	}
	aw_info("duration %d, select index %u", aw_haptic->duration, wavseq);
	aw_haptic->func->set_wav_seq(aw_haptic, 0, wavseq);
	aw_haptic->func->set_wav_loop(aw_haptic, 0, wavloop);
	aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
	aw_haptic->func->set_wav_loop(aw_haptic, 1, 0);

	return 0;
}
#endif

static void upload_lra(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t reg_val;

	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		reg_val = 0x00;
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X", aw_haptic->f0_cali_data);
		reg_val = (uint8_t)aw_haptic->f0_cali_data;
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X", aw_haptic->osc_cali_data);
		reg_val = (uint8_t)aw_haptic->osc_cali_data;
		break;
	default:
		aw_err("flag is error");
		reg_val = 0x00;
		break;
	}
	aw_haptic->func->set_trim_lra(aw_haptic, reg_val);
}

static int judge_within_cali_range(struct aw_haptic *aw_haptic)
{
	uint32_t f0_cali_min = 0;
	uint32_t f0_cali_max = 0;

	aw_info("enter");
	f0_cali_min = aw_haptic->info.f0_pre * (100 - aw_haptic->info.f0_cali_percent) / 100;
	f0_cali_max = aw_haptic->info.f0_pre * (100 + aw_haptic->info.f0_cali_percent) / 100;

	aw_info("f0_pre = %u, f0_cali_min = %u, f0_cali_max = %u, f0 = %u",
		aw_haptic->info.f0_pre, f0_cali_min, f0_cali_max, aw_haptic->f0);

	if (aw_haptic->f0 < f0_cali_min) {
		aw_err("lra f0 is too small, lra_f0 = %u!", aw_haptic->f0);
#ifdef AW_MAXIMUM_F0_CALI_DATA
		aw_haptic->f0_cali_data = aw_haptic->trim_lra_boundary;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
#endif
		return -ERANGE;
	}

	if (aw_haptic->f0 > f0_cali_max) {
		aw_err("lra f0 is too large, lra_f0 = %u!", aw_haptic->f0);
#ifdef AW_MAXIMUM_F0_CALI_DATA
		aw_haptic->f0_cali_data = aw_haptic->trim_lra_boundary - 1;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
#endif
		return -ERANGE;
	}

	return 0;
}

static int f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");
	upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->get_f0(aw_haptic)) {
		aw_err("get f0 failed");
	} else {
		/* max and min limit */
		ret = judge_within_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		/* calculate cali step */
		aw_haptic->func->calculate_cali_data(aw_haptic);
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	aw_haptic->func->play_stop(aw_haptic);

	return ret;
}

static void ram_vbat_comp(struct aw_haptic *aw_haptic, bool flag)
{
	int temp_gain = 0;

	aw_info("enter");
	if (flag) {
		if (aw_haptic->ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE) {
			aw_haptic->func->get_vbat(aw_haptic);
			temp_gain = aw_haptic->gain * AW_VBAT_REFER / aw_haptic->vbat;
			if (temp_gain > (AW_DEFAULT_GAIN * AW_VBAT_REFER / AW_VBAT_MIN)) {
				temp_gain = AW_DEFAULT_GAIN * AW_VBAT_REFER / AW_VBAT_MIN;
				aw_info("gain limit=%d", temp_gain);
			}
			aw_haptic->func->set_gain(aw_haptic, temp_gain);
			aw_info("ram vbat comp open, set_gain 0x%02X", temp_gain);
		} else {
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
			aw_info("ram vbat comp close, set_gain 0x%02X", aw_haptic->gain);
		}
	} else {
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		aw_info("ram vbat comp close, set_gain 0x%02X", aw_haptic->gain);
	}
}

static int osc_calculation_time(struct aw_haptic *aw_haptic)
{
	uint32_t base_addr = aw_haptic->ram.base_addr;
	uint32_t buf_len = 0;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	if (!aw_haptic->func->rtp_get_fifo_afs(aw_haptic)) {
		aw_dbg("haptic_rtp_get_fifo_afi, rtp_cnt= %u", aw_haptic->rtp_cnt);

		mutex_lock(&aw_haptic->rtp_lock);
		pm_qos_enable(aw_haptic, true);
		if ((aw_rtp->len - aw_haptic->rtp_cnt) < (base_addr >> 2))
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		else
			buf_len = (base_addr >> 2);

		if (aw_haptic->rtp_cnt != aw_rtp->len) {
			if (aw_haptic->timeval_flags == 1) {
				aw_haptic->kstart = ktime_get();
				aw_haptic->timeval_flags = 0;
			}
			aw_haptic->func->set_rtp_data(aw_haptic, &aw_rtp->data[aw_haptic->rtp_cnt],
						      buf_len);
			aw_haptic->rtp_cnt += buf_len;
		}
		pm_qos_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->rtp_lock);
	}

	if (aw_haptic->func->get_osc_status(aw_haptic)) {
		aw_haptic->kend = ktime_get();
		aw_info("osc trim playback done aw_haptic->rtp_cnt= %u", aw_haptic->rtp_cnt);
		return 0;
	}
	aw_haptic->kend = ktime_get();
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend, aw_haptic->kstart));
	if (aw_haptic->microsecond > AW_OSC_CALI_MAX_LENGTH) {
		aw_info("osc trim time out! aw_haptic->rtp_cnt %u", aw_haptic->rtp_cnt);
		return 0;
	}

	return -ERANGE;
}

static int rtp_fw_load(struct aw_haptic *aw_haptic, uint32_t file_num)
{
	int ret = 0;
	const struct firmware *rtp_file;

	ret = request_firmware(&rtp_file, aw_rtp_name[file_num], aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s", aw_rtp_name[file_num]);
		return ret;
	}

	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = NULL;
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		aw_err("error allocating memory");
		return -ENOMEM;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_haptic->rtp_len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes", aw_rtp_name[file_num], aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	return 0;
}

static int rtp_osc_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");
	/*aw_haptic add stop,for irq interrupt during calibrate */
	aw_haptic->func->play_stop(aw_haptic);

	mutex_lock(&aw_haptic->rtp_lock);
	ret = rtp_fw_load(aw_haptic, 0);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->rtp_lock);
		return ret;
	}
	mutex_unlock(&aw_haptic->rtp_lock);

	aw_haptic->rtp_init = false;
	aw_haptic->rtp_cnt = 0;
	aw_haptic->timeval_flags = 1;

	ram_vbat_comp(aw_haptic, false);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	disable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->func->haptic_start(aw_haptic);
	while (1) {
		ret = osc_calculation_time(aw_haptic);
		if (!ret)
			break;
	}
	enable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend, aw_haptic->kstart));
	/*calibration osc */
	aw_info("microsecond: %llu", aw_haptic->microsecond);

	return 0;
}

static void rtp_trim_lra_cali(struct aw_haptic *aw_haptic)
{
	uint32_t lra_trim_code = 0;
	/*0.1 percent below no need to calibrate */
	uint32_t osc_cali_threshold = 10;
	uint32_t code = 0;
	unsigned long real_time = 0;
	unsigned long theory_time = 0;

	upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (rtp_osc_cali(aw_haptic))
		return;
	real_time = aw_haptic->microsecond;
	theory_time = aw_haptic->func->get_theory_time(aw_haptic);
	if (theory_time == real_time) {
		aw_info("theory_time == real_time: %ld, no need to calibrate!", real_time);
		return;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(real_time - theory_time) > (theory_time/50), can't calibrate!");
			return;
		}

		if ((real_time - theory_time) < (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %ld, theory_time: %ld, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		code = ((real_time - theory_time) * 4) / (theory_time / 1000);
		code = ((code % 10 < 5) ? 0 : 1) + code / 10;
		code = aw_haptic->trim_lra_boundary + code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(theory_time - real_time) > (theory_time / 50), can't calibrate!");
			return;
		}
		if ((theory_time - real_time) < (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %ld, theory_time: %ld, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		code = ((theory_time - real_time) * 4) / (theory_time / 1000);
		code = ((code % 10 < 5) ? 0 : 1) + code / 10;
		code = aw_haptic->trim_lra_boundary - code;
	}
	if (code >= aw_haptic->trim_lra_boundary)
		lra_trim_code = code - aw_haptic->trim_lra_boundary;
	else
		lra_trim_code = code + aw_haptic->trim_lra_boundary;

	aw_haptic->osc_cali_data = lra_trim_code;
	upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_info("real_time: %lu, theory_time: %lu", real_time, theory_time);
	aw_info("code: %u, trim_lra: 0x%02X", code, lra_trim_code);
}

static int parse_awrw_data(struct aw_haptic *aw_haptic, const char *buf)
{
	uint8_t i = 0;
	uint8_t reg_num = aw_haptic->i2c_info.reg_num;
	const char *temp_buf = NULL;
	char data_buf[AWRW_CMD_UNIT] = { 0 };
	uint32_t value = 0;
	int ret = 0;
	int len = strlen(buf) - (AWRW_CMD_UNIT * 3);

	if (len < 0) {
		aw_err("parse data error");
		return -ERANGE;
	}
	temp_buf = &buf[AWRW_CMD_UNIT * 3];
	for (i = 0; i < reg_num; i++) {
		if (((i + 1) * AWRW_CMD_UNIT) > len) {
			aw_err("parse data error");
			return -ERANGE;
		}
		memcpy(data_buf, &temp_buf[i * AWRW_CMD_UNIT], 4);
		data_buf[4] = '\0';
		ret = kstrtouint(data_buf, 0, &value);
		if (ret < 0) {
			aw_err("kstrtouint error");
			return ret;
		}
		aw_haptic->i2c_info.reg_data[i] = (uint8_t)value;
	}

	return 0;
}

static int haptic_audio_ctr_list_insert(struct aw_haptic *aw_haptic,
					struct aw_haptic_ctr *haptic_ctr)
{
	struct aw_haptic_ctr *p_new = NULL;
	struct aw_haptic_audio *haptic_audio = &aw_haptic->haptic_audio;

	aw_info("enter");
	p_new = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (p_new == NULL)
		return -ENOMEM;

	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;
	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));
	aw_info("exit");

	return 0;
}

static void haptic_audio_ctrl_list_clr(struct aw_haptic_audio *haptic_audio)
{
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
		p_ctr = NULL;
	}
}

static void haptic_audio_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
}

static void haptic_audio_off(struct aw_haptic *aw_haptic)
{
	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, 0x80);
	aw_haptic->func->play_stop(aw_haptic);
	haptic_audio_ctrl_list_clr(&aw_haptic->haptic_audio);
	mutex_unlock(&aw_haptic->lock);
}

static void haptic_ctr_list_config(struct aw_haptic *aw_haptic, struct aw_haptic_ctr *p_ctr,
				   struct aw_haptic_ctr *p_ctr_bak,
				   struct aw_haptic_audio *haptic_audio)
{
	uint32_t list_input_cnt = 0;
	uint32_t list_output_cnt = 0;
	uint32_t list_diff_cnt = 0;
	uint32_t list_del_cnt = 0;

	list_for_each_entry_safe(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		list_input_cnt =  p_ctr->cnt;
		break;
	}
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		list_output_cnt =  p_ctr->cnt;
		break;
	}

	if (list_input_cnt > list_output_cnt)
		list_diff_cnt = list_input_cnt - list_output_cnt;

	if (list_input_cnt < list_output_cnt)
		list_diff_cnt = 32 + list_input_cnt - list_output_cnt;

	if (list_diff_cnt > 2) {
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list), list) {
			if ((p_ctr->play == 0) && (AW_CMD_ENABLE == (AW_CMD_ENABLE & p_ctr->cmd))) {
				list_del(&p_ctr->list);
				kfree(p_ctr);
				list_del_cnt++;
			}
			if (list_del_cnt == list_diff_cnt)
				break;
		}
	}
}

static void parse_haptic_audio_data(struct aw_haptic *aw_haptic)
{
	bool ctr_list_flag = false;
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;
	struct aw_haptic_audio *haptic_audio = &(aw_haptic->haptic_audio);

	memset(&(haptic_audio->ctr), 0, sizeof(struct aw_haptic_ctr));
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		ctr_list_flag = true;
		break;
	}

	if (ctr_list_flag == false)
		aw_dbg("ctr list empty");
	if (ctr_list_flag == true)
		haptic_ctr_list_config(aw_haptic, p_ctr, p_ctr_bak, haptic_audio);

	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		haptic_audio->ctr.cnt = p_ctr->cnt;
		haptic_audio->ctr.cmd = p_ctr->cmd;
		haptic_audio->ctr.play = p_ctr->play;
		haptic_audio->ctr.wavseq = p_ctr->wavseq;
		haptic_audio->ctr.loop = p_ctr->loop;
		haptic_audio->ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		p_ctr = NULL;
		break;
	}

	if (haptic_audio->ctr.play) {
		aw_info("cnt=%u, cmd=%u, play=%u, wavseq=%u, loop=%u, gain=%u",
			haptic_audio->ctr.cnt, haptic_audio->ctr.cmd, haptic_audio->ctr.play,
			haptic_audio->ctr.wavseq, haptic_audio->ctr.loop, haptic_audio->ctr.gain);
	}

}

static void haptic_audio_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, haptic_audio.work);

	int rtp_is_going_on = 0;

	aw_dbg("enter");

	mutex_lock(&aw_haptic->haptic_audio.lock);
	parse_haptic_audio_data(aw_haptic);
	/* rtp mode jump */
	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);
	if (rtp_is_going_on) {
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	mutex_unlock(&aw_haptic->haptic_audio.lock);

#ifdef AW_DOUBLE
	if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_ENABLE) {
		mutex_lock(&aw_haptic->lock);
		/* haptic config */
		if (aw_haptic->haptic_audio.ctr.cmd & AW_CMD_L_EN) {
			left->func->play_stop(left);
			left->func->play_mode(left, AW_RAM_MODE);
			left->func->set_wav_seq(left, 0x00, aw_haptic->haptic_audio.ctr.wavseq);
			left->func->set_wav_seq(left, 0x01, 0x00);
			left->func->set_wav_loop(left, 0x00, aw_haptic->haptic_audio.ctr.loop);
			left->func->set_gain(left, aw_haptic->haptic_audio.ctr.gain);
		}
		if (aw_haptic->haptic_audio.ctr.cmd & AW_CMD_R_EN) {
			right->func->play_stop(right);
			right->func->play_mode(right, AW_RAM_MODE);
			right->func->set_wav_seq(right, 0x00, aw_haptic->haptic_audio.ctr.wavseq);
			right->func->set_wav_seq(right, 0x01, 0x00);
			right->func->set_wav_loop(right, 0x00, aw_haptic->haptic_audio.ctr.loop);
			right->func->set_gain(right, aw_haptic->haptic_audio.ctr.gain);
		}
		/* play go */
		if (aw_haptic->haptic_audio.ctr.cmd & AW_CMD_L_EN)
			left->func->haptic_start(left);
		if (aw_haptic->haptic_audio.ctr.cmd & AW_CMD_R_EN)
			right->func->haptic_start(right);
		mutex_unlock(&aw_haptic->lock);
	} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_STOP) {
		mutex_lock(&aw_haptic->lock);
		left->func->play_stop(left);
		right->func->play_stop(right);
		mutex_unlock(&aw_haptic->lock);
	} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_GAIN) {
		mutex_lock(&aw_haptic->lock);
		left->func->set_gain(left, aw_haptic->haptic_audio.ctr.gain);
		right->func->set_gain(right, aw_haptic->haptic_audio.ctr.gain);
		mutex_unlock(&aw_haptic->lock);
	}
#else
	if ((AW_CMD_HAPTIC & aw_haptic->haptic_audio.ctr.cmd) == AW_CMD_ENABLE) {
		if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_ENABLE) {
			aw_info("haptic audio play start");

			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00,
						     aw_haptic->haptic_audio.ctr.wavseq);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00,
						      aw_haptic->haptic_audio.ctr.loop);
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->haptic_audio.ctr.gain);
			aw_haptic->func->haptic_start(aw_haptic);
			mutex_unlock(&aw_haptic->lock);

		} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_STOP) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);

		} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_GAIN) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->haptic_audio.ctr.gain);
			mutex_unlock(&aw_haptic->lock);
		}
	}
#endif
}

static enum hrtimer_restart haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic, haptic_audio.timer);
	int time = aw_haptic->haptic_audio.timer_val;

	queue_work(aw_haptic->work_queue, &aw_haptic->haptic_audio.work);
	hrtimer_start(&aw_haptic->haptic_audio.timer,
		      ktime_set(time / 1000000, (time % 1000000) * 1000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->state);
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr, const char *buf,
			   size_t count)
{
	return count;
}

static ssize_t duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t duration_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw_haptic->duration = val;
	aw_info("duration=%d", aw_haptic->duration);

	return count;
}

static ssize_t activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	int rc = 0;
	uint32_t val = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	if (val != 0 && val != 1)
		return count;

	aw_info("value=%u", val);

	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->state = val;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->vibrator_work);

	return count;
}

static ssize_t activate_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", aw_haptic->activate_mode);
}

static ssize_t activate_mode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t index_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	aw_haptic->func->get_wav_seq(aw_haptic, 1);
	aw_haptic->index = aw_haptic->seq[0];

	return snprintf(buf, PAGE_SIZE, "index = %d\n", aw_haptic->index);
}

static ssize_t index_store(struct device *dev, struct device_attribute *attr, const char *buf,
			   size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	if (val > aw_haptic->ram.ram_num) {
		aw_err("input value out of range!");
		return count;
	}

	aw_info("value=%u", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->index = val;
	aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint8_t gain = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_gain(aw_haptic, &gain);
	mutex_unlock(&aw_haptic->lock);

	return snprintf(buf, PAGE_SIZE, "gain = 0x%02X\n", gain);
}

static ssize_t gain_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw_info("value=0x%02X", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gain = val;
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t seq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, AW_SEQUENCER_SIZE);
	for (i = 0; i < AW_SEQUENCER_SIZE; i++)
		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d = %u\n", i + 1,
				  aw_haptic->seq[i]);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t seq_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%X %X", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= AW_SEQUENCER_SIZE || databuf[1] > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		aw_info("seq%u=0x%02X", databuf[0], databuf[1]);

		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_seq(aw_haptic, (uint8_t)databuf[0],
					     aw_haptic->seq[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t rtp_interface_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t rtp_interface_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t loop_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_loop(aw_haptic, reg_val);
	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02X\n", i * 2 + 1,
				  aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02X\n", i * 2 + 2,
				  aw_haptic->loop[i * 2 + 1]);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_info("seq%u loop=0x%02X", databuf[0], databuf[1]);

		mutex_lock(&aw_haptic->lock);
		aw_haptic->loop[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_loop(aw_haptic, (uint8_t)databuf[0],
					      aw_haptic->loop[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	len = aw_haptic->func->get_reg(aw_haptic, len, buf);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	uint8_t val = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%X %X", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];
		mutex_lock(&aw_haptic->lock);
		haptic_nv_i2c_writes(aw_haptic, (uint8_t)databuf[0], &val, AW_I2C_BYTE_ONE);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t ram_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	int i = 0;
	int j = 0;
	int size = 0;
	ssize_t len = 0;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
	uint8_t buffer[AW_RAMDATA_SHOW_LINE_BUFFER_SZIE] = {0};
	uint8_t unit[AW_RAMDATA_SHOW_UINT_SIZE] = {0};

	mutex_lock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "Please cat log\n");
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;

		aw_haptic->func->get_ram_data(aw_haptic, ram_data, size);
		for (j = 0; j < size; j++) {
			sprintf(unit, "0x%02X ", ram_data[j]);
			strcat(buffer, unit);
			if ((j + 1) % AW_RAMDATA_SHOW_COLUMN == 0 || j == size - 1) {
				aw_info("[ram_data]:%s", buffer);
				memset(buffer, 0, AW_RAMDATA_SHOW_LINE_BUFFER_SZIE);
			}
		}
		i += size;
	}
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t ram_update_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		ram_update(aw_haptic);

	return count;
}

static ssize_t f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->get_f0(aw_haptic);
	upload_lra(aw_haptic, AW_F0_CALI_LRA);
	mutex_unlock(&aw_haptic->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw_haptic->f0);

	return len;
}

static ssize_t f0_store(struct device *dev, struct device_attribute *attr, const char *buf,
			size_t count)
{
	return count;
}


static ssize_t cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw_haptic->f0);

	return len;
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		mutex_lock(&aw_haptic->lock);
		f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}


static ssize_t f0_save_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",
			aw_haptic->f0_cali_data);

	return len;
}

static ssize_t f0_save_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;
	aw_haptic->f0_cali_data = val;

	return count;
}

static ssize_t cont_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->read_cont_f0(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw_haptic->cont_f0);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t cont_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (val)
		aw_haptic->func->cont_config(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return count;
}


static ssize_t vbat_monitor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_vbat(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %u\n", aw_haptic->vbat);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t vbat_monitor_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	return count;
}


static ssize_t lra_resistance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_lra_resistance(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %u\n", aw_haptic->lra);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t lra_resistance_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	return count;
}

static ssize_t prctmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;
	uint8_t reg_val = 0;

	mutex_lock(&aw_haptic->lock);
	reg_val = aw_haptic->func->get_prctmode(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode = %u\n", reg_val);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t prctmode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0, 0 };
	uint32_t prtime = 0;
	uint32_t prlvl = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		prtime = databuf[0];
		prlvl = databuf[1];

		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->protect_config(aw_haptic, prtime, prlvl);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t ram_vbat_comp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp = %u\n",
			aw_haptic->ram_vbat_comp);

	return len;
}

static ssize_t ram_vbat_comp_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	if (val)
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	else
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw_haptic->lock);

	return count;
}


static ssize_t ram_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	get_ram_num(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "ram_num = %u\n", aw_haptic->ram.ram_num);

	return len;
}


static ssize_t awrw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint8_t reg_num = aw_haptic->i2c_info.reg_num;
	uint8_t flag = aw_haptic->i2c_info.flag;
	uint8_t *reg_data = aw_haptic->i2c_info.reg_data;
	uint8_t i = 0;
	ssize_t len = 0;

	if (!reg_num) {
		aw_err("awrw parameter error");
		return len;
	}
	if (flag == AW_READ) {
		for (i = 0; i < reg_num; i++)
			len += snprintf(buf + len, PAGE_SIZE - len, "0x%02X,", reg_data[i]);
		len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");
	}

	return len;
}

static ssize_t awrw_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	uint32_t datatype[3] = { 0 };
	int ret = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%x %x %x", &datatype[0], &datatype[1], &datatype[2]) == 3) {
		if (!datatype[1]) {
			aw_err("awrw parameter error");
			return count;
		}
		aw_haptic->i2c_info.flag = (uint8_t)datatype[0];
		aw_haptic->i2c_info.reg_num = (uint8_t)datatype[1];
		aw_haptic->i2c_info.first_addr = (uint8_t)datatype[2];

		mutex_lock(&aw_haptic->lock);
		if (aw_haptic->i2c_info.flag == AW_WRITE) {
			ret = parse_awrw_data(aw_haptic, buf);
			if (ret < 0) {
				mutex_unlock(&aw_haptic->lock);
				return count;
			}
			haptic_nv_i2c_writes(aw_haptic, aw_haptic->i2c_info.first_addr,
					     aw_haptic->i2c_info.reg_data,
					     aw_haptic->i2c_info.reg_num);
		} else if (aw_haptic->i2c_info.flag == AW_READ) {
			haptic_nv_i2c_reads(aw_haptic, aw_haptic->i2c_info.first_addr,
					    aw_haptic->i2c_info.reg_data,
					    aw_haptic->i2c_info.reg_num);
		} else {
			aw_err("flag is error");
		}
		mutex_unlock(&aw_haptic->lock);

	} else
		aw_err("missing number of parameters");

	return count;
}

static ssize_t gun_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->gun_type);
}

static ssize_t gun_type_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%u", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_type = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t bullet_nr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->bullet_nr);
}

static ssize_t bullet_nr_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%u", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t haptic_audio_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.delay_val=%dus\n",
			aw_haptic->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.timer_val=%dus\n",
			aw_haptic->haptic_audio.timer_val);

	return len;
}

static ssize_t haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[2] = { 0 };

	if (sscanf(buf, "%u %u", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->haptic_audio.delay_val = databuf[0];
		aw_haptic->haptic_audio.timer_val = databuf[1];
	}

	return count;
}

static ssize_t haptic_audio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%u\n", aw_haptic->haptic_audio.ctr.cnt);

	return len;
}

static int haptic_audio_config(struct aw_haptic *aw_haptic, uint32_t databuf[])
{
	struct aw_haptic_ctr *hap_ctr = NULL;
	int delay = aw_haptic->haptic_audio.delay_val;

	aw_info("enter");
	hap_ctr = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (hap_ctr == NULL)
		return -ENOMEM;

	mutex_lock(&aw_haptic->haptic_audio.lock);
	hap_ctr->cnt = (uint8_t)databuf[0];
	hap_ctr->cmd = (uint8_t)databuf[1];
	hap_ctr->play = (uint8_t)databuf[2];
	hap_ctr->wavseq = (uint8_t)databuf[3];
	hap_ctr->loop = (uint8_t)databuf[4];
	hap_ctr->gain = (uint8_t)databuf[5];

	haptic_audio_ctr_list_insert(aw_haptic, hap_ctr);

	if (hap_ctr->cmd == 0xff) {
		aw_info("haptic_audio stop");
		if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
			aw_info("cancel haptic_audio_timer");
			hrtimer_cancel(&aw_haptic->haptic_audio.timer);
			aw_haptic->haptic_audio.ctr.cnt = 0;
			haptic_audio_off(aw_haptic);
		}
	} else {
		if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
			/* */
		} else {
			aw_info("start haptic_audio_timer");
			haptic_audio_init(aw_haptic);
			hrtimer_start(&aw_haptic->haptic_audio.timer,
				      ktime_set(delay / 1000000, (delay % 1000000) * 1000),
				      HRTIMER_MODE_REL);
		}
	}
	kfree(hap_ctr);
	mutex_unlock(&aw_haptic->haptic_audio.lock);

	return 0;
}

static ssize_t haptic_audio_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t databuf[6] = { 0 };
	int rtp_is_going_on = 0;

	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);

	if (rtp_is_going_on) {
		aw_info("RTP is running, stop audio haptic");
		return count;
	}

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}

	if (sscanf(buf, "%u %u %u %u %u %u", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5]) == 6) {
		if (databuf[2]) {
			aw_info("cnt=%u, cmd=%u, play=%u", databuf[0], databuf[1], databuf[2]);
			aw_info("wavseq=%u, loop=%u, gain=%u", databuf[3], databuf[4], databuf[5]);
			haptic_audio_config(aw_haptic, databuf);
		}
	}

	return count;
}

#ifdef AW_DOUBLE
static ssize_t dual_cont_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	right->func->play_stop(right);
	left->func->play_stop(left);
	if (val) {
		upload_lra(right, AW_F0_CALI_LRA);
		upload_lra(left, AW_F0_CALI_LRA);
		right->func->cont_config(right);
		left->func->cont_config(left);
	}

	return count;
}

static ssize_t dual_index_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d %d\n", left->index, right->index);

	return len;
}

static ssize_t dual_index_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	int index_l = 0;
	int index_r = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%d %d", &index_l, &index_r) == 2) {
		aw_info("index_l=%d index_r=%d", index_l, index_r);
		if (index_l > aw_haptic->ram.ram_num || index_r > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		mutex_lock(&aw_haptic->lock);
		left->index = index_l;
		left->func->set_repeat_seq(left, left->index);
		right->index = index_r;
		right->func->set_repeat_seq(right, right->index);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t dual_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "dual_mode = %u\n", aw_haptic->activate_mode);
}

static ssize_t dual_mode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			       size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	left->activate_mode = val;
	right->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t dual_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t dual_duration_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int duration_l = 0;
	int duration_r = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	if (sscanf(buf, "%d %d", &duration_l, &duration_r) == 2) {
		mutex_lock(&aw_haptic->lock);
		left->duration = duration_l;
		right->duration = duration_r;
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t dual_activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t dual_activate_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%u", val);
	if (!left->ram_init || !right->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	left->state = val;
	right->state = val;
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	queue_work(left->work_queue, &left->vibrator_work);
	queue_work(right->work_queue, &right->vibrator_work);

	return count;
}
#endif

static DEVICE_ATTR_RW(state);
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RW(duration);
static DEVICE_ATTR_RW(activate);
static DEVICE_ATTR_RW(activate_mode);
static DEVICE_ATTR_RW(index);
static DEVICE_ATTR_RW(gain);
static DEVICE_ATTR_RW(seq);
static DEVICE_ATTR_RW(rtp_interface);
static DEVICE_ATTR_RW(loop);
static DEVICE_ATTR_RW(ram_update);
static DEVICE_ATTR_RW(f0);
static DEVICE_ATTR_RW(cali);
static DEVICE_ATTR_RW(f0_save);
static DEVICE_ATTR_RW(cont);
static DEVICE_ATTR_RW(vbat_monitor);
static DEVICE_ATTR_RW(lra_resistance);
static DEVICE_ATTR_RW(prctmode);
static DEVICE_ATTR_RW(ram_vbat_comp);
static DEVICE_ATTR_RO(ram_num);
static DEVICE_ATTR_RW(awrw);
static DEVICE_ATTR_RW(gun_type);
static DEVICE_ATTR_RW(bullet_nr);
static DEVICE_ATTR_RW(haptic_audio_time);
static DEVICE_ATTR_RW(haptic_audio);
#ifdef AW_DOUBLE
static DEVICE_ATTR_WO(dual_cont);
static DEVICE_ATTR_RW(dual_index);
static DEVICE_ATTR_RW(dual_mode);
static DEVICE_ATTR_RW(dual_duration);
static DEVICE_ATTR_RW(dual_activate);
#endif

static struct attribute *vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_rtp_interface.attr,
	&dev_attr_loop.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_cont.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_ram_num.attr,
	&dev_attr_awrw.attr,
	&dev_attr_gun_type.attr,
	&dev_attr_bullet_nr.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_haptic_audio.attr,
#ifdef AW_DOUBLE
	&dev_attr_dual_cont.attr,
	&dev_attr_dual_index.attr,
	&dev_attr_dual_mode.attr,
	&dev_attr_dual_duration.attr,
	&dev_attr_dual_activate.attr,
#endif
	NULL
};

static struct attribute_group vibrator_attribute_group = {
	.attrs = vibrator_attributes
};

static ssize_t rtp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %u", aw_haptic->rtp_cnt);

	return len;
}

static ssize_t rtp_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_err("kstrtouint fail");
		return count;
	}

	if (!aw_haptic->ram.base_addr) {
		aw_err("no set FIFO space for rtp play!");
		return count;
	}

	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < aw_haptic->rtp_num_max)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
	} else if (val == 0) {
		aw_haptic->state = 0;
	} else {
		aw_haptic->state = 0;
		aw_err("input number error:%u", val);
	}
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);

	return count;
}

static ssize_t osc_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_cali_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	if (val == 1) {
		rtp_trim_lra_cali(aw_haptic);
	} else if (val == 2) {
		upload_lra(aw_haptic, AW_OSC_CALI_LRA);
		rtp_osc_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t osc_save_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_save_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;
	aw_haptic->osc_cali_data = val;

	return count;
}

#ifdef AW_DOUBLE
static ssize_t dual_rtp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "left_rtp_cnt = %u\n", left->rtp_cnt);
	len += snprintf(buf + len, PAGE_SIZE - len, "right_rtp_cnt = %u\n", right->rtp_cnt);

	return len;
}

static ssize_t dual_rtp_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
	int rtp_l = 0;
	int rtp_r = 0;

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%d %d", &rtp_l, &rtp_r) == 2) {
		if (rtp_l > 0 && rtp_l < aw_haptic->rtp_num_max) {
			left->state = 1;
			left->rtp_file_num = rtp_l;
		} else if (rtp_l == 0) {
			left->state = 0;
		} else {
			left->state = 0;
			aw_err("input number error:%d", rtp_l);
		}
		if (rtp_r > 0 && rtp_r < aw_haptic->rtp_num_max) {
			right->state = 1;
			right->rtp_file_num = rtp_r;
		} else if (rtp_r == 0) {
			right->state = 0;
		} else {
			right->state = 0;
			aw_err("input number error:%d", rtp_r);
		}
	}
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	queue_work(left->work_queue, &left->rtp_work);
	queue_work(right->work_queue, &right->rtp_work);

	return count;
}
#endif

static DEVICE_ATTR_RW(rtp);
static DEVICE_ATTR_RW(osc_cali);
static DEVICE_ATTR_RW(osc_save);
#ifdef AW_DOUBLE
static DEVICE_ATTR_RW(dual_rtp);
#endif

static struct attribute *rtp_attributes[] = {
	&dev_attr_rtp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_osc_save.attr,
#ifdef AW_DOUBLE
	&dev_attr_dual_rtp.attr,
#endif
	NULL
};

static struct attribute_group rtp_attribute_group = {
	.attrs = rtp_attributes
};

#ifdef AW_TIKTAP
static inline unsigned int tiktap_get_sys_msecs(void)
{
#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
	struct timespec64 ts64;

	ktime_get_coarse_real_ts64(&ts64);
#else
	struct timespec64 ts64 = current_kernel_time64();
#endif

	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void tiktap_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, tiktap_work);
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int count = 100;
	unsigned char reg_val = 0x10;
	unsigned char glb_state_val = 0;
	unsigned int write_start;
	unsigned int buf_cnt = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->tiktap_stop_flag = false;
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	while (true && count--) {
		if (tiktap_buf->status == MMAP_BUF_DATA_VALID) {
			aw_haptic->func->haptic_start(aw_haptic);
			mdelay(1);
			break;
		} else if (aw_haptic->tiktap_stop_flag == true) {
			mutex_unlock(&aw_haptic->lock);
			return;
		}
		mdelay(1);
	}
	if (count <= 0) {
		aw_err("wait 100 ms but start_buf->status != VALID! status = 0x%02x",
		       tiktap_buf->status);
		aw_haptic->tiktap_stop_flag = true;
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	aw_haptic->tiktap_ready = true;
	mutex_unlock(&aw_haptic->lock);

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	write_start = tiktap_get_sys_msecs();
	while (true) {
		if (tiktap_get_sys_msecs() > (write_start + 800)) {
			aw_err("Failed! tiktap endless loop");
			break;
		}
		reg_val = aw_haptic->func->rtp_get_fifo_aes(aw_haptic);
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((glb_state_val & AW_BIT_GLBRD_STATE_MASK) != AW_BIT_STATE_RTP_GO) {
			aw_err("tiktap glb_state != RTP_GO!, glb_state = 0x%02x", glb_state_val);
			break;
		}
		if ((aw_haptic->tiktap_stop_flag == true) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_FINISHED) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_INVALID)) {
			aw_info("tiktap exit! tiktap_buf->status = 0x%02x", tiktap_buf->status);
			break;
		} else if ((tiktap_buf->status == MMAP_BUF_DATA_VALID) && (reg_val & 0x01)) {
			aw_info("buf_cnt = %u, bit = %u, length = %d!",
				buf_cnt, tiktap_buf->bit, tiktap_buf->length);

			aw_haptic->func->set_rtp_data(aw_haptic, tiktap_buf->data, tiktap_buf->length);
			tiktap_buf->status = MMAP_BUF_DATA_FINISHED;

			tiktap_buf = tiktap_buf->kernel_next;
			write_start = tiktap_get_sys_msecs();
			buf_cnt++;
		} else {
			mdelay(1);
		}
	}
	pm_qos_enable(aw_haptic, false);
	aw_haptic->tiktap_stop_flag = true;
	mutex_unlock(&aw_haptic->rtp_lock);
}

static void tiktap_clean_buf(struct aw_haptic *aw_haptic, int status)
{
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int i = 0;

	for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
		tiktap_buf->status = status;
		tiktap_buf = tiktap_buf->kernel_next;
	}
}

static long tiktap_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int tmp = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = (struct aw_haptic *)file->private_data;

	switch (cmd) {
	case TIKTAP_GET_HWINFO:
		aw_info("cmd = TIKTAP_GET_HWINFO!");
		tmp = aw_haptic->name;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_GET_F0:
		aw_info("cmd = TIKTAP_GET_F0!");
		tmp = aw_haptic->f0;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_STOP_MODE:
		aw_info("cmd = TIKTAP_STOP_MODE!");
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		break;
	case TIKTAP_RTP_MODE:
		/* aw_info("cmd = TIKTAP_RTP_MODE!"); */
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		queue_work(aw_haptic->work_queue, &aw_haptic->tiktap_work);
		break;
	case TIKTAP_SETTING_GAIN:
		aw_info("cmd = TIKTAP_SETTING_GAIN!");
		if (arg > 0x80)
			arg = 0x80;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->set_gain(aw_haptic, (uint8_t)arg);
		mutex_unlock(&aw_haptic->lock);
		break;
	default:
		aw_info("unknown cmd = %u", cmd);
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long tiktap_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	ret = tiktap_unlocked_ioctl(file, cmd, arg);

	return ret;
}
#endif

static int tiktap_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct aw_haptic *aw_haptic = (struct aw_haptic *)file->private_data;
	int ret = 0;

#if KERNEL_VERSION(4, 7, 0) < LINUX_VERSION_CODE
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) |
			      calc_vm_flag_bits(MAP_SHARED);

	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		    VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;

	if (!vma || (pgprot_val(vma->vm_page_prot) != pgprot_val(vm_get_page_prot(vm_flags)))) {
		aw_err("vm_page_prot error!");
		return -EPERM;
	}

	if (!vma || ((vma->vm_end - vma->vm_start) != (PAGE_SIZE << AW_TIKTAP_MMAP_PAGE_ORDER))) {
		aw_err("mmap size check err!");
		return -EPERM;
	}
#endif
	phys = virt_to_phys(aw_haptic->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT),
			      (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		aw_err("mmap failed!");
		return ret;
	}

	aw_info("success!");

	return ret;
}

static int tiktap_file_open(struct inode *inode, struct file *file)
{
#ifdef AW_DOUBLE
	file->private_data = (void *)left;
#else
	file->private_data = (void *)g_aw_haptic;
#endif
	return 0;
}

#ifdef AW_DOUBLE
static int tiktap_file_open_r(struct inode *inode, struct file *file)
{
	file->private_data = (void *)right;
	return 0;
}
#endif

#ifdef KERNEL_OVER_5_10
static const struct proc_ops tiktap_proc_ops = {
	.proc_mmap = tiktap_file_mmap,
	.proc_open = tiktap_file_open,
	.proc_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = tiktap_compat_ioctl,
#endif
};
#ifdef AW_DOUBLE
static const struct proc_ops tiktap_proc_ops_r = {
	.proc_mmap = tiktap_file_mmap,
	.proc_open = tiktap_file_open_r,
	.proc_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = tiktap_compat_ioctl,
#endif
};
#endif
#else
static const struct file_operations tiktap_proc_ops = {
	.owner = THIS_MODULE,
	.mmap = tiktap_file_mmap,
	.open = tiktap_file_open,
	.unlocked_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tiktap_compat_ioctl,
#endif
};
#ifdef AW_DOUBLE
static const struct file_operations tiktap_proc_ops_r = {
	.owner = THIS_MODULE,
	.mmap = tiktap_file_mmap,
	.open = tiktap_file_open_r,
	.unlocked_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tiktap_compat_ioctl,
#endif
};
#endif
#endif

static int tiktap_init(struct aw_haptic *aw_haptic)
{
	struct mmap_buf_format *tiktap_start_buf;
	struct proc_dir_entry *tiktap_config_proc = NULL;

	/* Create proc file node */
#ifdef AW_DOUBLE
	if (of_device_is_compatible(aw_haptic->i2c->dev.of_node, "awinic,haptic_nv_l"))
		tiktap_config_proc = proc_create(AW_TIKTAP_PROCNAME, 0664, NULL, &tiktap_proc_ops);

	if (of_device_is_compatible(aw_haptic->i2c->dev.of_node, "awinic,haptic_nv_r"))
		tiktap_config_proc = proc_create(AW_TIKTAP_PROCNAME_R, 0664, NULL, &tiktap_proc_ops_r);
#else
	tiktap_config_proc = proc_create(AW_TIKTAP_PROCNAME, 0664, NULL, &tiktap_proc_ops);
#endif
	if (tiktap_config_proc == NULL) {
		aw_err("create proc file failed!");
		return -EPERM;
	}
	aw_info("create proc file success!");

	/* Construct shared memory */
	tiktap_start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, AW_TIKTAP_MMAP_PAGE_ORDER);
	if (tiktap_start_buf == NULL) {
		aw_err("Error __get_free_pages failed");
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(tiktap_start_buf));
	{
		struct mmap_buf_format *temp;
		unsigned int i = 0;

		temp = tiktap_start_buf;
		for (i = 1; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
			temp->kernel_next = (tiktap_start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = tiktap_start_buf;

		temp = tiktap_start_buf;
		for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
			temp->bit = i;
			temp = temp->kernel_next;
		}
	}

	aw_haptic->aw_config_proc = tiktap_config_proc;
	aw_haptic->start_buf = tiktap_start_buf;
	/* init flag and work */
	aw_haptic->tiktap_stop_flag = true;
	aw_haptic->tiktap_ready = false;
	INIT_WORK(&aw_haptic->tiktap_work, tiktap_work_routine);

	return 0;
}
#endif

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic, timer);

	aw_info("enter");
	aw_haptic->state = 0;
	queue_work(aw_haptic->work_queue, &aw_haptic->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, vibrator_work);

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
#ifdef AW_DURATION_DECIDE_WAVEFORM
		if (ram_config(aw_haptic, aw_haptic->duration) < 0) {
			mutex_unlock(&aw_haptic->lock);
			return;
		}
#endif
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			ram_play(aw_haptic);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			ram_play(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else if (aw_haptic->activate_mode == AW_CONT_MODE) {
			aw_haptic->func->cont_config(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int wait_enter_rtp_mode(struct aw_haptic *aw_haptic, int cnt)
{
	bool rtp_work_flag = false;
	uint8_t ret = 0;

	while (cnt) {
		ret = aw_haptic->func->judge_rtp_going(aw_haptic);
		if (ret) {
			rtp_work_flag = true;
			aw_info("RTP_GO!");
			break;
		}
		cnt--;
		aw_info("wait for RTP_GO, glb_state=0x%02X", ret);
		usleep_range(AW_RTP_DELAY_MIN, AW_RTP_DELAY_MAX);
	}
	if (!rtp_work_flag) {
		aw_haptic->func->play_stop(aw_haptic);
		aw_err("failed to enter RTP_GO status!");
		return -ERANGE;
	}

	return 0;
}

static void rtp_work_routine(struct work_struct *work)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, rtp_work);

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	if (hrtimer_active(&aw_haptic->timer))
		hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	if (!aw_haptic->state) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	/* fw loaded */
	mutex_lock(&aw_haptic->rtp_lock);
	ret = rtp_fw_load(aw_haptic, aw_haptic->rtp_file_num);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	mutex_unlock(&aw_haptic->rtp_lock);

	/* rtp config */
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;
	upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->haptic_start(aw_haptic);
	usleep_range(AW_RTP_DELAY_MIN, AW_RTP_DELAY_MAX);
	ret = wait_enter_rtp_mode(aw_haptic, 200);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	aw_haptic->rtp_cnt = 0;
	rtp_play(aw_haptic);
	if (aw_haptic->play_mode == AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, true);
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
}

#ifdef TIMED_OUTPUT
static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic, vib_dev);

	if (hrtimer_active(&aw_haptic->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw_haptic->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic, vib_dev);

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return;
	}

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (value > 0) {
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		ram_vbat_comp(aw_haptic, false);
		aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
		aw_haptic->func->haptic_start(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");
}
#else
static enum led_brightness brightness_get(struct led_classdev *cdev)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	return aw_haptic->amplitude;
}

static void brightness_set(struct led_classdev *cdev, enum led_brightness level)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return;
	}
	aw_haptic->amplitude = level;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->state = aw_haptic->amplitude;
	aw_haptic->activate_mode = AW_RAM_MODE;
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->vibrator_work);
}
#endif

static int vibrator_init(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");

#ifdef TIMED_OUTPUT
	aw_info("TIMED_OUT FRAMEWORK!");
#ifdef AW_DOUBLE
	ret = memcmp(aw_haptic->mark, "left", sizeof("left"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_l";
	ret = memcmp(aw_haptic->mark, "right", sizeof("right"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_r";
#else
	aw_haptic->vib_dev.name = "vibrator";
#endif
	aw_haptic->vib_dev.get_time = vibrator_get_time;
	aw_haptic->vib_dev.enable = vibrator_enable;

	ret = timed_output_dev_register(&(aw_haptic->vib_dev));
	if (ret < 0) {
		aw_err("fail to create timed output dev");
		return ret;
	}
#else
	aw_info("loaded in leds_cdev framework!");
#ifdef AW_DOUBLE
	ret = memcmp(aw_haptic->mark, "left", sizeof("left"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_l";
	ret = memcmp(aw_haptic->mark, "right", sizeof("right"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_r";
#else
	aw_haptic->vib_dev.name = "vibrator";
#endif
	aw_haptic->vib_dev.brightness_get = brightness_get;
	aw_haptic->vib_dev.brightness_set = brightness_set;
	ret = devm_led_classdev_register(&aw_haptic->i2c->dev, &aw_haptic->vib_dev);
	if (ret < 0) {
		aw_err("fail to create led dev");
		return ret;
	}
#endif

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj, &vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error creating sysfs common attr files");
		return ret;
	}
		/*
	 * Android has a habit of trying to set the vibrator trigger to "transient",
	 * which destroys our own "activate", "duration", and "state" attributes.
	 * Remove the "trigger" attribute provided by leds_class so that no one can
	 * change the trigger of the vibrator LED device.
	 *
	 * Luckily, `sysfs_remove_file` only uses the `name` field, so we can use a
	 * compound literal instead of having to find the proper attribute struct
	 */
	sysfs_remove_file(&aw_haptic->vib_dev.dev->kobj,
	                  &((struct attribute){.name = "trigger"}));

	if (aw_haptic->is_used_irq_pin) {
		ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj, &rtp_attribute_group);
		if (ret < 0) {
			aw_err("error creating sysfs rtp attr files");
			return ret;
		}
	}
	hrtimer_init(&aw_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->timer.function = vibrator_timer_func;
	INIT_WORK(&aw_haptic->vibrator_work, vibrator_work_routine);
	INIT_WORK(&aw_haptic->rtp_work, rtp_work_routine);
	mutex_init(&aw_haptic->lock);
	mutex_init(&aw_haptic->rtp_lock);
	sema_init(&aw_haptic->sema, 1);

	return 0;
}

static void haptic_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw_haptic->haptic_audio.delay_val = 1;
	aw_haptic->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.ctr_list));
	hrtimer_init(&aw_haptic->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->haptic_audio.timer.function = haptic_audio_timer_func;
	INIT_WORK(&aw_haptic->haptic_audio.work, haptic_audio_work_routine);
	mutex_init(&aw_haptic->haptic_audio.lock);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_num_max = sizeof(aw_rtp_name) / AW_NAME_MAX;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	aw_haptic->f0_pre = aw_haptic->info.f0_pre;
	aw_haptic->gun_type = AW_GUN_TYPE_DEF_VAL;
	aw_haptic->bullet_nr = AW_BULLET_NR_DEF_VAL;
	aw_haptic->func->play_mode(aw_haptic, AW_STANDBY_MODE);
	aw_haptic->func->misc_para_init(aw_haptic);
	aw_haptic->func->vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
	aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;

	if (aw_haptic->info.lk_f0_cali) {
		aw_haptic->f0_cali_data = aw_haptic->info.lk_f0_cali;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
	} else {
		f0_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");
}

static void ram_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, ram_work.work);

	ram_update(aw_haptic);
}

static void ram_work_init(struct aw_haptic *aw_haptic)
{
	int ram_timer_val = AW_RAM_WORK_DELAY_INTERVAL;

	INIT_DELAYED_WORK(&aw_haptic->ram_work, ram_work_routine);
	schedule_delayed_work(&aw_haptic->ram_work, msecs_to_jiffies(ram_timer_val));
}

#ifdef AW_INPUT_FRAMEWORK
static void input_stop_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, stop_work);

	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
}

static void input_gain_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, input_gain_work);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
}

static void input_vib_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, input_vib_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00, aw_haptic->index);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
			ram_play(aw_haptic);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
			ram_play(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int input_upload_effect(struct input_dev *dev, struct ff_effect *effect,
			       struct ff_effect *old)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);
	short wav_id = 0;
	int wav_id_max = 0;
	int ret = 0;

	mutex_lock(&aw_haptic->lock);
	switch (effect->type) {
	case FF_CONSTANT:
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
		aw_haptic->duration = effect->replay.length;
		aw_haptic->index = aw_haptic->ram.ram_num;
		aw_info("waveform id = %d", aw_haptic->index);
		break;
	case FF_PERIODIC:
		ret = copy_from_user(&wav_id, effect->u.periodic.custom_data, sizeof(short));
		if (ret) {
			aw_err("copy from user error %d!!", ret);
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		aw_info("waveform id = %d", wav_id);
		wav_id_max = sizeof(aw_rtp_name) / sizeof(*aw_rtp_name) +
			     aw_haptic->ram.ram_num - 1;
		if (wav_id > 0 && wav_id < aw_haptic->ram.ram_num) {
			aw_haptic->activate_mode = AW_RAM_MODE;
			aw_haptic->index = wav_id;
		} else if (wav_id > aw_haptic->ram.ram_num && wav_id <= wav_id_max) {
			aw_haptic->activate_mode = AW_RTP_MODE;
			aw_haptic->rtp_file_num = wav_id - aw_haptic->ram.ram_num;
		} else {
			aw_haptic->activate_mode = AW_STANDBY_MODE;
			aw_err("waveform id is error");
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		break;
	default:
		aw_err("Unsupported effect type: %u", effect->type);
		break;
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int input_playback(struct input_dev *dev, int effect_id, int val)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (val > 0) {
		aw_haptic->state = 1;
	} else {
		queue_work(aw_haptic->work_queue, &aw_haptic->stop_work);
		return 0;
	}

	switch (aw_haptic->activate_mode) {
	case AW_RAM_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->input_vib_work);
		break;
	case AW_RAM_LOOP_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->input_vib_work);
		break;
	case AW_RTP_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);
		break;
	default:
		aw_err("Unsupported mode: %d", aw_haptic->activate_mode);
		break;
	}

	return 0;
}

static int input_erase(struct input_dev *dev, int effect_id)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	aw_haptic->duration = 0;

	return 0;
}

static void input_set_gain(struct input_dev *dev, uint16_t gain)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (gain > 0x7fff)
		gain = 0x7fff;
	aw_haptic->gain = gain * 0x80 / 0x7fff;
	queue_work(aw_haptic->work_queue, &aw_haptic->input_gain_work);
	aw_info("aw_haptic->gain = 0x%02x", aw_haptic->gain);
}

static int input_framework_init(struct aw_haptic *aw_haptic)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = devm_input_allocate_device(aw_haptic->dev);
	if (input_dev == NULL)
		return -ENOMEM;
	input_dev->name = "aw-haptic-nv";
	input_set_drvdata(input_dev, aw_haptic);
	aw_haptic->input_dev = input_dev;
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	ret = input_ff_create(input_dev, AW_EFFECT_NUMBER);
	if (ret < 0) {
		aw_err("create input FF device failed, rc=%d\n", ret);
		return ret;
	}
	input_dev->ff->upload = input_upload_effect;
	input_dev->ff->playback = input_playback;
	input_dev->ff->erase = input_erase;
	input_dev->ff->set_gain = input_set_gain;
	INIT_WORK(&aw_haptic->input_gain_work, input_gain_work_routine);
	INIT_WORK(&aw_haptic->stop_work, input_stop_work_routine);
	INIT_WORK(&aw_haptic->input_vib_work, input_vib_work_routine);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		aw_err("register input device failed, rc=%d\n", ret);
		input_ff_destroy(aw_haptic->input_dev);
		return ret;
	}

	return ret;
}
#endif

static int aw_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw_haptic *aw_haptic;
	struct device_node *np = i2c->dev.of_node;

	pr_info("<%s>%s: enter\n", AW_I2C_NAME, __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("<%s>%s: check_functionality failed\n", AW_I2C_NAME, __func__);
		return -EIO;
	}

	aw_haptic = devm_kzalloc(&i2c->dev, sizeof(struct aw_haptic), GFP_KERNEL);
	if (aw_haptic == NULL)
		return -ENOMEM;

	aw_haptic->dev = &i2c->dev;
	aw_haptic->i2c = i2c;

	i2c_set_clientdata(i2c, aw_haptic);
#ifdef AW_INPUT_FRAMEWORK
	ret = input_framework_init(aw_haptic);
	if (ret < 0)
		goto err_input_config;
#endif
	/* aw_haptic rst & int */
	if (np) {
		ret = parse_dt_gpio(aw_haptic, &i2c->dev, np);
		if (ret) {
			aw_err("failed to parse gpio");
			goto err_parse_dt;
		}
	} else {
		aw_haptic->reset_gpio = -1;
		aw_haptic->irq_gpio = -1;
	}

	if (aw_haptic->is_used_rst_pin) {
		if (gpio_is_valid(aw_haptic->reset_gpio)) {
			ret = devm_gpio_request_one(&i2c->dev, aw_haptic->reset_gpio,
						GPIOF_OUT_INIT_LOW, "awinic_rst");
			if (ret) {
				aw_err("rst request failed");
				goto err_reset_gpio_request;
			}
		}
	}

	if (gpio_is_valid(aw_haptic->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->irq_gpio,
					    GPIOF_DIR_IN, "awinic_int");
		if (ret) {
			aw_err("int request failed");
			goto err_irq_gpio_request;
		}
	}

	parse_dts_i2c_addr(aw_haptic, &i2c->dev, np);
	/* aw_haptic chip id */
	ret = parse_chipid(aw_haptic);
	if (ret < 0) {
		aw_err("read_chipid failed ret=%d", ret);
		goto err_id;
	}

	ret = chip_private_init(aw_haptic);
	if (ret < 0) {
		aw_err("chip_private_init failed ret=%d", ret);
		goto err_chip_private_init;
	}

	ret = func_ptr_init(aw_haptic, &i2c->dev);
	if (ret < 0) {
		aw_err("ctrl_init failed ret=%d", ret);
		goto err_ctrl_init;
	}

	ret = aw_haptic->func->check_qualify(aw_haptic);
	if (ret < 0) {
		aw_err("qualify check failed ret=%d", ret);
		goto err_ctrl_init;
	}

	sw_reset(aw_haptic);
	ret = aw_haptic->func->offset_cali(aw_haptic);
	if (ret < 0)
		sw_reset(aw_haptic);

	aw_haptic->func->parse_dts(aw_haptic, np);

	ret = irq_config(&i2c->dev, aw_haptic);
	if (ret != 0) {
		aw_err("irq_config failed ret=%d", ret);
		goto err_irq_config;
	}

#ifdef AW_TIKTAP
	g_aw_haptic = aw_haptic;
	ret = tiktap_init(aw_haptic);
	if (ret) {
		aw_err("tiktap_init failed ret = %d", ret);
		goto err_irq_config;
	}
#endif
	vibrator_init(aw_haptic);
	haptic_init(aw_haptic);
	aw_haptic->work_queue = create_singlethread_workqueue("aw_haptic_vibrator_work_queue");
	if (!aw_haptic->work_queue) {
		aw_err("Error creating aw_haptic_vibrator_work_queue");
		goto err_irq_config;
	}
	aw_haptic->func->creat_node(aw_haptic);
	ram_work_init(aw_haptic);
	dev_set_drvdata(&i2c->dev, aw_haptic);
	aw_info("probe completed successfully!");
	return 0;

err_irq_config:
err_ctrl_init:
err_chip_private_init:
err_irq_gpio_request:
err_reset_gpio_request:
err_parse_dt:
err_id:
err_input_config:
	devm_kfree(&i2c->dev, aw_haptic);
	aw_haptic = NULL;
	return ret;
}

#ifdef KERNEL_OVER_6_1
static void aw_i2c_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_info("enter");
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#else
	sysfs_remove_group(&aw_haptic->vib_dev.dev->kobj, &vibrator_attribute_group);
	devm_led_classdev_unregister(&aw_haptic->i2c->dev, &aw_haptic->vib_dev);
#endif
#ifdef AW_INPUT_FRAMEWORK
	cancel_work_sync(&aw_haptic->input_gain_work);
	cancel_work_sync(&aw_haptic->stop_work);
	cancel_work_sync(&aw_haptic->input_vib_work);
	input_unregister_device(aw_haptic->input_dev);
	input_ff_destroy(aw_haptic->input_dev);
#endif
	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->haptic_audio.work);
	hrtimer_cancel(&aw_haptic->haptic_audio.timer);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->haptic_audio.lock);
#ifdef AW_TIKTAP
	cancel_work_sync(&aw_haptic->tiktap_work);
	ClearPageReserved(virt_to_page(aw_haptic->start_buf));
	free_pages((unsigned long)aw_haptic->start_buf, AW_TIKTAP_MMAP_PAGE_ORDER);
	aw_haptic->start_buf = NULL;
	proc_remove(aw_haptic->aw_config_proc);
	aw_haptic->aw_config_proc = NULL;
#endif
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
}
#else
static int aw_i2c_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_info("enter");
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#else
	sysfs_remove_group(&aw_haptic->vib_dev.dev->kobj, &vibrator_attribute_group);
	devm_led_classdev_unregister(&aw_haptic->i2c->dev, &aw_haptic->vib_dev);
#endif
#ifdef AW_INPUT_FRAMEWORK
	cancel_work_sync(&aw_haptic->input_gain_work);
	cancel_work_sync(&aw_haptic->stop_work);
	cancel_work_sync(&aw_haptic->input_vib_work);
	input_unregister_device(aw_haptic->input_dev);
	input_ff_destroy(aw_haptic->input_dev);
#endif
	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->haptic_audio.work);
	hrtimer_cancel(&aw_haptic->haptic_audio.timer);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->haptic_audio.lock);
#ifdef AW_TIKTAP
	cancel_work_sync(&aw_haptic->tiktap_work);
	ClearPageReserved(virt_to_page(aw_haptic->start_buf));
	free_pages((unsigned long)aw_haptic->start_buf, AW_TIKTAP_MMAP_PAGE_ORDER);
	aw_haptic->start_buf = NULL;
	proc_remove(aw_haptic->aw_config_proc);
	aw_haptic->aw_config_proc = NULL;
#endif
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);

	return 0;
}
#endif

static int aw_i2c_suspend(struct device *dev)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static int aw_i2c_resume(struct device *dev)
{
	pr_info("<%s>%s enter\n", AW_I2C_NAME, __func__);

	return 0;
}

static SIMPLE_DEV_PM_OPS(aw_pm_ops, aw_i2c_suspend, aw_i2c_resume);

static const struct i2c_device_id aw_i2c_id[] = {
	{AW_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw_i2c_id);

static const struct of_device_id aw_dt_match[] = {
#ifdef AW_DOUBLE
	{.compatible = "awinic,haptic_nv_r"},
	{.compatible = "awinic,haptic_nv_l"},
#else
	{.compatible = "awinic,haptic_nv"},
#endif
	{},
};

static struct i2c_driver aw_i2c_driver = {
	.driver = {
		   .name = AW_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw_pm_ops,
#endif
		   },
	.probe = aw_i2c_probe,
	.remove = aw_i2c_remove,
	.id_table = aw_i2c_id,
};

static int __init aw_i2c_init(void)
{
	int ret = 0;

	pr_info("<%s>%s: aw_haptic driver version %s\n", AW_I2C_NAME, __func__,
		HAPTIC_NV_DRIVER_VERSION);
	ret = i2c_add_driver(&aw_i2c_driver);
	if (ret) {
		pr_err("<%s>%s: fail to add aw_haptic device into i2c\n", AW_I2C_NAME, __func__);
		return ret;
	}
	return 0;
}
module_init(aw_i2c_init);

static void __exit aw_i2c_exit(void)
{
	i2c_del_driver(&aw_i2c_driver);
}
module_exit(aw_i2c_exit);

MODULE_DESCRIPTION("AWINIC Haptic Driver");
MODULE_LICENSE("GPL v2");
