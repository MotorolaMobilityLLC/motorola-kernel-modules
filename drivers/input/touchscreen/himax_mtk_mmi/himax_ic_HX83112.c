/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83112 chipset
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_ic_HX83112.h"
#include "himax_modular.h"

static void hx83112_chip_init(void)
{
	private_ts->chip_cell_type = CHIP_IS_IN_CELL;
	I("%s:IC cell type = %d\n",  __func__,
		private_ts->chip_cell_type);
	(IC_CHECKSUM) = HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	(FW_VER_MAJ_FLASH_ADDR) = 49157;  /*0x00C005*/
	(FW_VER_MIN_FLASH_ADDR) = 49158;  /*0x00C006*/
	(CFG_VER_MAJ_FLASH_ADDR) = 49408;  /*0x00C100*/
	(CFG_VER_MIN_FLASH_ADDR) = 49409;  /*0x00C101*/
	(CID_VER_MAJ_FLASH_ADDR) = 49154;  /*0x00C002*/
	(CID_VER_MIN_FLASH_ADDR) = 49155;  /*0x00C003*/
	(CFG_TABLE_FLASH_ADDR) = 0x10000;
	(CFG_TABLE_FLASH_ADDR_T) = (CFG_TABLE_FLASH_ADDR);
}

static bool hx83112_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	do {
		if (cnt == 0
		|| (tmp_data[0] != 0xA5
		&& tmp_data[0] != 0x00
		&& tmp_data[0] != 0x87))
			g_core_fp.fp_register_write(
				pfw_op->addr_ctrl_fw_isr,
				DATA_LEN_4,
				pfw_op->data_fw_stop,
				0);

		/*msleep(20);*/
		usleep_range(10000, 10001);
		/* check fw status */
		g_core_fp.fp_register_read(
			pic_op->addr_cs_central_state,
			ADDR_LEN_4, tmp_data, 0);

		if (tmp_data[0] != 0x05) {
			I("%s: Do not need wait FW, Status = 0x%02X!\n",
					__func__, tmp_data[0]);
			break;
		}

		g_core_fp.fp_register_read(pfw_op->addr_ctrl_fw_isr,
			4, tmp_data, false);
		I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__,
			cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 10) && check_en == true);

	cnt = 0;

	do {
		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = pic_op->data_i2c_psw_lb[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 * I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = pic_op->data_i2c_psw_ub[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_ub[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 * Check enter_save_mode
		 */
		g_core_fp.fp_register_read(
			pic_op->addr_cs_central_state,
			ADDR_LEN_4,
			tmp_data,
			0);
		I("%s: Check enter_save_mode data[0]=%X\n", __func__,
			tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/**
			 * Reset TCON
			 */
			g_core_fp.fp_register_write(
				pic_op->addr_tcon_on_rst,
				DATA_LEN_4,
				pic_op->data_rst,
				0);
			usleep_range(1000, 1001);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(
				pic_op->addr_tcon_on_rst,
				DATA_LEN_4,
				tmp_data,
				0);
			/**
			 * Reset ADC
			 */
			g_core_fp.fp_register_write(
				pic_op->addr_adc_on_rst,
				DATA_LEN_4,
				pic_op->data_rst,
				0);
			usleep_range(1000, 1001);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(
				pic_op->addr_adc_on_rst,
				DATA_LEN_4,
				tmp_data,
				0);
			goto SUCCEED;
		} else {
			/*msleep(10);*/
#if defined(HX_RST_PIN_FUNC)
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		}
	} while (cnt++ < 5);

	return false;
SUCCEED:
	return true;
}

static bool hx83112ab_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	do {
		if (cnt == 0
		|| (tmp_data[0] != 0xA5
		&& tmp_data[0] != 0x00
		&& tmp_data[0] != 0x87))
			g_core_fp.fp_register_write(
				pfw_op->addr_ctrl_fw_isr,
				DATA_LEN_4,
				pfw_op->data_fw_stop,
				0);

		/*msleep(20);*/
		usleep_range(10000, 10001);
		/* check fw status */
		g_core_fp.fp_register_read(
			pic_op->addr_cs_central_state,
			ADDR_LEN_4, tmp_data, 0);

		if (tmp_data[0] != 0x05) {
			I("%s: Do not need wait FW, Status = 0x%02X!\n",
					__func__, tmp_data[0]);
			break;
		}

		g_core_fp.fp_register_read(pfw_op->addr_ctrl_fw_isr,
			4, tmp_data, false);
		I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__,
			cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 10) && check_en == true);

	cnt = 0;

	do {
		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = pic_op->data_i2c_psw_lb[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 * I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = pic_op->data_i2c_psw_ub[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_ub[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}
		usleep_range(500, 501);
		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x00
		 */
		tmp_data[0] = 0x00;

		ret = himax_bus_write(pic_op->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}
		usleep_range(500, 501);
		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = pic_op->data_i2c_psw_lb[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 * I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = pic_op->data_i2c_psw_ub[0];

		ret = himax_bus_write(pic_op->adr_i2c_psw_ub[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 * Check enter_save_mode
		 */
		g_core_fp.fp_register_read(
			pic_op->addr_cs_central_state,
			ADDR_LEN_4,
			tmp_data,
			0);
		I("%s: Check enter_save_mode data[0]=%X\n", __func__,
			tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/**
			 * Reset TCON
			 */
			g_core_fp.fp_register_write(
				pic_op->addr_tcon_on_rst,
				DATA_LEN_4,
				pic_op->data_rst,
				0);
			usleep_range(1000, 1001);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(
				pic_op->addr_tcon_on_rst,
				DATA_LEN_4,
				tmp_data,
				0);
			/**
			 * Reset ADC
			 */
			g_core_fp.fp_register_write(
				pic_op->addr_adc_on_rst,
				DATA_LEN_4,
				pic_op->data_rst,
				0);
			usleep_range(1000, 1001);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(
				pic_op->addr_adc_on_rst,
				DATA_LEN_4,
				tmp_data,
				0);
			goto SUCCEED;
		} else {
			/*msleep(10);*/
#if defined(HX_RST_PIN_FUNC)
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		}
	} while (cnt++ < 5);

	return false;
SUCCEED:
	return true;
}

#if defined(HX_ZERO_FLASH)
static void himax_hx83112f_reload_to_active(void)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		g_core_fp.fp_register_write(addr, DATA_LEN_4, data, 0);
		usleep_range(1000, 1100);
		g_core_fp.fp_register_read(addr, DATA_LEN_4, data, 0);
		I("%s: data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__,
				data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01
		|| data[0] != 0xEC)
		&& retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_hx83112f_resume_ic_action(void)
{
#if !defined(HX_RESUME_HW_RESET)
	himax_hx83112f_reload_to_active();
#endif
}

static void himax_hx83112f_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;
	int ret = 0;

	I("Enter %s\n", __func__);
	private_ts->notouch_frame = private_ts->ic_notouch_frame;
	g_core_fp.fp_interface_on();
	g_core_fp.fp_register_write(pfw_op->addr_ctrl_fw_isr,
		sizeof(pfw_op->data_clear), pfw_op->data_clear, 0);
	/*msleep(20);*/
	usleep_range(10000, 10001);
	if (!FlashMode) {
#if defined(HX_RST_PIN_FUNC)
		g_core_fp.fp_ic_reset(false, false);
#else
		g_core_fp.fp_system_reset();
#endif
	} else {
		do {
			g_core_fp.fp_register_read(
				pfw_op->addr_flag_reset_event,
				DATA_LEN_4, tmp_data, 0);
			I("%s:Read status from IC = %X,%X\n", __func__,
					tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01
			|| tmp_data[0] != 0x00)
			&& retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#if defined(HX_RST_PIN_FUNC)
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		} else {
			I("%s:OK and Read status from IC = %X,%X\n", __func__,
				tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			ret = himax_bus_write(
				pic_op->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0) {
				E("%s: cmd=%x i2c access fail!\n",
				__func__,
				pic_op->adr_i2c_psw_lb[0]);
			}

				ret = himax_bus_write(
					pic_op->adr_i2c_psw_ub[0],
					tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0) {
				E("%s: cmd=%x i2c access fail!\n",
				__func__,
				pic_op->adr_i2c_psw_ub[0]);
			}
		}
	}
	himax_hx83112f_reload_to_active();
}

#endif

static void hx83112_func_re_init(void)
{
	g_core_fp.fp_sense_off = hx83112_sense_off;
	g_core_fp.fp_chip_init = hx83112_chip_init;
}

static void hx83112_reg_re_init(void)
{
	private_ts->ic_notouch_frame = hx83112_notouch_frame;
}

static void hx83112f_reg_re_init(void)
{
	himax_parse_assign_cmd(hx83112f_fw_addr_raw_out_sel,
		pfw_op->addr_raw_out_sel,
		sizeof(pfw_op->addr_raw_out_sel));
	private_ts->ic_notouch_frame = hx83112f_notouch_frame;
}

static void hx83112f_func_re_init(void)
{
#if defined(HX_ZERO_FLASH)
	g_core_fp.fp_resume_ic_action = himax_hx83112f_resume_ic_action;
	g_core_fp.fp_sense_on = himax_hx83112f_sense_on;
	g_core_fp.fp_0f_reload_to_active = himax_hx83112f_reload_to_active;
#endif
}

static void hx83112ab_func_re_init(void)
{
	g_core_fp.fp_sense_off = hx83112ab_sense_off;
}

static bool hx83112_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	ret = himax_mcu_in_cmd_struct_init();
	if (ret < 0) {
		ret_data = false;
		E("%s:cmd_struct_init Fail:\n", __func__);
		return ret_data;
	}

	himax_mcu_in_cmd_init();

	hx83112_reg_re_init();
	hx83112_func_re_init();

	ret = himax_bus_read(pic_op->addr_conti[0],
			tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return false;
	}

	if (g_core_fp.fp_sense_off(false) == false) {
		ret_data = false;
		E("%s:fp_sense_off Fail:\n", __func__);
		return ret_data;
	}
	for (i = 0; i < 5; i++) {
		ret = g_core_fp.fp_register_read(
			pfw_op->addr_icid_addr,
			DATA_LEN_4,
			tmp_data,
			false);

		if (ret != 0) {
			ret_data = false;
			E("%s:fp_register_read Fail:\n", __func__);
			return ret_data;
		}
		I("%s:Read driver IC ID = %X, %X, %X\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83)
		&& (tmp_data[2] == 0x11)
		&& ((tmp_data[1] == 0x2a)
		|| (tmp_data[1] == 0x2b)
		|| (tmp_data[1] == 0x2e)
		|| (tmp_data[1] == 0x2f))) {
			if (tmp_data[1] == 0x2a) {
				strlcpy(private_ts->chip_name,
					HX_83112A_SERIES_PWON, 30);
				ic_data->ic_adc_num =
					hx83112a_data_adc_num;
				hx83112ab_func_re_init();
			} else if (tmp_data[1] == 0x2b) {
				strlcpy(private_ts->chip_name,
					HX_83112B_SERIES_PWON, 30);
				ic_data->ic_adc_num =
					hx83112b_data_adc_num;
				hx83112ab_func_re_init();
			} else if (tmp_data[1] == 0x2e) {
				strlcpy(private_ts->chip_name,
					HX_83112E_SERIES_PWON, 30);
				ic_data->ic_adc_num =
					hx83112e_data_adc_num;
				hx83112ab_func_re_init();
			} else if (tmp_data[1] == 0x2f) {
				strlcpy(private_ts->chip_name,
					HX_83112F_SERIES_PWON, 30);
				ic_data->ic_adc_num =
					hx83112f_data_adc_num;
				hx83112f_reg_re_init();
				hx83112f_func_re_init();
			}

			I("%s:IC name = %s\n", __func__,
				private_ts->chip_name);

			I("Himax IC package %x%x%x in\n", tmp_data[3],
					tmp_data[2], tmp_data[1]);
			ret_data = true;
			goto FINAL;
		} else {
			ret_data = false;
			E("%s:Read driver ID register Fail:\n", __func__);
			E("Could NOT find Himax Chipset\n");
			E("Please check 1.VCCD,VCCA,VSP,VSN\n");
			E("2. LCM_RST,TP_RST\n");
			E("3. Power On Sequence\n");
		}
	}
FINAL:

	return ret_data;
}

bool _hx83112_init(void)
{
	bool ret = false;

	I("%s\n", __func__);
	ret = hx83112_chip_detect();
	return ret;
}
