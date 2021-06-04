/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_upgrade_ft5452.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-15
*
* Abstract:
*
* Reference:
*
*****************************************************************************/
/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "../focaltech_flash.h"

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u8 pb_file_ft5452i[] = {
#include "../include/pramboot/FT5452i_Pramboot_v4.1_20210427.i"
};

extern struct fts_upgrade *fwupgrade;


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/************************************************************************
* Name: fts_ft5452_upgrade
* Brief:
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/

static u16 fts_ft5452i_crc16_calc_host(u8 *pbuf, u32 length)
{
    u16 ecc = 0;
    u32 i = 0;
    u32 j = 0;

    for ( i = 0; i < length; i += 2 ) {
        ecc ^= ((pbuf[i] << 8) | (pbuf[i + 1]));
        for (j = 0; j < 16; j ++) {
            if (ecc & 0x01)
                ecc = (u16)((ecc >> 1) ^ AL2_FCS_COEF);
            else
                ecc >>= 1;
        }
    }

    return ecc;
}


static int fts_ft5452i_pram_ecc_cal_algo(
    struct fts_upgrade *upg,
    u32 start_addr,
    u32 ecc_length)
{
    int ret = 0;
    int i = 0;
    int ecc = 0;
    u8 val[2] = { 0 };
    u8 tmp = 0;
    u8 cmd[8] = { 0 };

    FTS_INFO("read out pramboot checksum");
    if ((!upg) || (!upg->func)) {
        FTS_ERROR("upg/func is null");
        return -EINVAL;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC;
    cmd[1] = BYTE_OFF_16(start_addr);
    cmd[2] = BYTE_OFF_8(start_addr);
    cmd[3] = BYTE_OFF_0(start_addr);
    cmd[4] = BYTE_OFF_16(ecc_length);
    cmd[5] = BYTE_OFF_8(ecc_length);
    cmd[6] = BYTE_OFF_0(ecc_length);
    cmd[7] = 0xCC;
    ret = fts_write(cmd, 8);
    if (ret < 0) {
        FTS_ERROR("write pramboot ecc cal cmd fail");
        return ret;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        msleep(1);
        ret = fts_read(cmd, 1, val, 1);
        if (ret < 0) {
            FTS_ERROR("ecc_finish read cmd fail");
            return ret;
        }
        tmp = 0x33;
        if (tmp == val[0])
            break;
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        FTS_ERROR("wait ecc finish fail");
        return -EIO;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
    ret = fts_read(cmd, 1, val, 2);
    if (ret < 0) {
        FTS_ERROR("read pramboot ecc fail");
        return ret;
    }

    ecc = ((u16)(val[0] << 8) + val[1]) & 0x0000FFFF;
    return ecc;
}



static int fts_ft5452i_pram_init(void)
{
    int ret = 0;
    u8 reg_val = 0;
    u8 wbuf[3] = { 0 };

    FTS_INFO("pramboot initialization");

    /* read flash ID */
    wbuf[0] = FTS_CMD_FLASH_TYPE;
    ret = fts_read(wbuf, 1, &reg_val, 1);
    if (ret < 0) {
        FTS_ERROR("read flash type fail");
        return ret;
    }

    /* set flash clk */
    wbuf[0] = FTS_CMD_FLASH_TYPE;
    wbuf[1] = reg_val;
    wbuf[2] = 0x00;
    ret = fts_write(wbuf, 3);
    if (ret < 0) {
        FTS_ERROR("write flash type fail");
        return ret;
    }

    return 0;
}


static int fts_ft5452i_fwupg_get_boot_state(
    struct fts_upgrade *upg,
    enum FW_STATUS *fw_sts)
{
    int ret = 0;
    u8 cmd[4] = { 0 };
    u32 cmd_len = 0;
    u8 val[3] = { 0 };
    struct ft_chip_t *ids = NULL;

    FTS_INFO("**********read boot id**********");
    if ((!upg) || (!upg->func) || (!upg->ts_data) || (!fw_sts)) {
        FTS_ERROR("upg/func/ts_data/fw_sts is null");
        return -EINVAL;
    }

    if (upg->func->hid_supported)
        fts_hid2std();

    cmd[0] = FTS_CMD_START1;
    cmd[1] = FTS_CMD_START2;
    if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
        cmd_len = 1;
    else
        cmd_len = 2;
    ret = fts_write(cmd, cmd_len);
    if (ret < 0) {
        FTS_ERROR("write 55 cmd fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    cmd[0] = FTS_CMD_READ_ID;
    cmd[1] = cmd[2] = cmd[3] = 0x00;
    if (fts_data->ic_info.is_incell ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0))
        cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(cmd, cmd_len, val, 3);
    if (ret < 0) {
        FTS_ERROR("write 90 cmd fail");
        return ret;
    }
    FTS_INFO("read boot id:0x%02x%02x%02x", val[0], val[1], val[2]);

    ids = &upg->ts_data->ic_info.ids;
    if ((val[0] == ids->rom_idh) && (val[1] == ids->rom_idl)) {
        FTS_INFO("tp run in romboot");
        *fw_sts = FTS_RUN_IN_ROM;
    } else if ((val[0] == ids->pb_idh) && (val[1] == ids->pb_idl)) {
        FTS_INFO("tp run in pramboot");
        *fw_sts = FTS_RUN_IN_PRAM;
    } else if ((val[0] == ids->bl_idh) && (val[1] == ids->bl_idl)) {
        FTS_INFO("tp run in bootloader");
        *fw_sts = FTS_RUN_IN_BOOTLOADER;
    }

    return 0;
}

static bool fts_ft5452i_fwupg_check_state(
    struct fts_upgrade *upg, enum FW_STATUS rstate)
{
    int ret = 0;
    int i = 0;
    enum FW_STATUS cstate = FTS_RUN_IN_ERROR;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        ret = fts_ft5452i_fwupg_get_boot_state(upg, &cstate);
        /* FTS_DEBUG("fw state=%d, retries=%d", cstate, i); */
        if (cstate == rstate)
            return true;
        msleep(FTS_DELAY_READ_ID);
    }

    return false;
}


static int fts_ft5452i_fwupg_reset_to_romboot(struct fts_upgrade *upg)
{
    int ret = 0;
    int i = 0;
    u8 cmd = FTS_CMD_RESET;
    enum FW_STATUS state = FTS_RUN_IN_ERROR;

    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("pram/rom/bootloader reset cmd write fail");
        return ret;
    }
    mdelay(10);

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        ret = fts_ft5452i_fwupg_get_boot_state(upg, &state);
        if (FTS_RUN_IN_ROM == state)
            break;
        mdelay(5);
    }
    if (i >= FTS_UPGRADE_LOOP) {
        FTS_ERROR("reset to romboot fail");
        return -EIO;
    }

    return 0;
}

static int fts_ft5452i_pram_write_buf(struct fts_upgrade *upg, u8 *buf, u32 len)
{
    int ret = 0;
    u32 i = 0;
    u32 j = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number;
    u32 packet_len = 0;
    u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN] = { 0 };
    u8 ecc_tmp = 0;
    int ecc_in_host = 0;
    u32 cmdlen = 0;

    FTS_INFO("write pramboot to pram");
    if ((!upg) || (!upg->func) || !buf) {
        FTS_ERROR("upg/func/buf is null");
        return -EINVAL;
    }

    FTS_INFO("pramboot len=%d", len);
    if ((len < PRAMBOOT_MIN_SIZE) || (len > PRAMBOOT_MAX_SIZE)) {
        FTS_ERROR("pramboot length(%d) fail", len);
        return -EINVAL;
    }

    packet_number = len / FTS_FLASH_PACKET_LENGTH;
    remainder = len % FTS_FLASH_PACKET_LENGTH;
    if (remainder > 0)
        packet_number++;
    packet_len = FTS_FLASH_PACKET_LENGTH;

    for (i = 0; i < packet_number; i++) {
        offset = i * FTS_FLASH_PACKET_LENGTH;
        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;

        packet_buf[0] = FTS_ROMBOOT_CMD_SET_PRAM_ADDR;
        packet_buf[1] = BYTE_OFF_16(offset);
        packet_buf[2] = BYTE_OFF_8(offset);
        packet_buf[3] = BYTE_OFF_0(offset);

        ret = fts_write(packet_buf, FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN);
        if (ret < 0) {
            FTS_ERROR("pramboot set write address(%d) fail", i);
            return ret;
        }

        packet_buf[0] = FTS_ROMBOOT_CMD_WRITE;
        cmdlen = 1;

        for (j = 0; j < packet_len; j++) {
            packet_buf[cmdlen + j] = buf[offset + j];
            if (ECC_CHECK_MODE_XOR == upg->func->pram_ecc_check_mode) {
                ecc_tmp ^= packet_buf[cmdlen + j];
            }
        }

        ret = fts_write(packet_buf, packet_len + cmdlen);
        if (ret < 0) {
            FTS_ERROR("pramboot write data(%d) fail", i);
            return ret;
        }
    }

    if ((ECC_CHECK_MODE_CRC16 == upg->func->pram_ecc_check_mode) ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)) {
        ecc_in_host = (int)fts_ft5452i_crc16_calc_host(buf, len);
    } else {
        ecc_in_host = (int)ecc_tmp;
    }

    return ecc_in_host;
}

static int fts_ft5452i_pram_start(void)
{
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;
    int ret = 0;

    FTS_INFO("remap to start pramboot");

    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("write start pram cmd fail");
        return ret;
    }
    msleep(FTS_DELAY_PRAMBOOT_START);

    return 0;
}


static int fts_ft5452i_pram_write_remap(struct fts_upgrade *upg)
{
    int ret = 0;
    int ecc_in_host = 0;
    int ecc_in_tp = 0;
    u8 *pb_buf = NULL;
    u32 pb_len = 0;

    FTS_INFO("write pram and remap");
    if (!upg || !upg->func || !upg->func->pramboot) {
        FTS_ERROR("upg/func/pramboot is null");
        return -EINVAL;
    }

    if (upg->func->pb_length < FTS_MIN_LEN) {
        FTS_ERROR("pramboot length(%d) fail", upg->func->pb_length);
        return -EINVAL;
    }

    pb_buf = upg->func->pramboot;
    pb_len = upg->func->pb_length;

    /* write pramboot to pram */
    ecc_in_host = fts_ft5452i_pram_write_buf(upg, pb_buf, pb_len);
    if (ecc_in_host < 0) {
        FTS_ERROR( "write pramboot fail");
        return ecc_in_host;
    }

    /* read out checksum */
    ecc_in_tp = fts_ft5452i_pram_ecc_cal_algo(upg, 0, pb_len);
    if (ecc_in_tp < 0) {
        FTS_ERROR( "read pramboot ecc fail");
        return ecc_in_tp;
    }

    FTS_INFO("pram ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    /*  pramboot checksum != fw checksum, upgrade fail */
    if (ecc_in_host != ecc_in_tp) {
        FTS_ERROR("pramboot ecc check fail");
        return -EIO;
    }

    /*start pram*/
    ret = fts_ft5452i_pram_start();
    if (ret < 0) {
        FTS_ERROR("pram start fail");
        return ret;
    }

    return 0;
}


static int fts_ft5452i_write_pramboot_private(void)
{
    int ret = 0;
    bool state = 0;
    enum FW_STATUS status = FTS_RUN_IN_ERROR;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("**********pram write and init**********");
    if ((NULL == upg) || (NULL == upg->func)) {
        FTS_ERROR("upgrade/func is null");
        return -EINVAL;
    }

    if (!upg->func->pramboot_supported) {
        FTS_ERROR("ic not support pram");
        return -EINVAL;
    }

    FTS_DEBUG("check whether tp is in romboot or not ");
    /* need reset to romboot when non-romboot state */
    ret = fts_ft5452i_fwupg_get_boot_state(upg, &status);
    if (status != FTS_RUN_IN_ROM) {
        if (FTS_RUN_IN_PRAM == status) {
            FTS_INFO("tp is in pramboot, need send reset cmd before upgrade");
            ret = fts_ft5452i_pram_init();
            if (ret < 0) {
                FTS_ERROR("pramboot(before) init fail");
                return ret;
            }
        }

        FTS_INFO("tp isn't in romboot, need send reset to romboot");
        ret = fts_ft5452i_fwupg_reset_to_romboot(upg);
        if (ret < 0) {
            FTS_ERROR("reset to romboot fail");
            return ret;
        }
    }

    /* check the length of the pramboot */
    ret = fts_ft5452i_pram_write_remap(upg);
    if (ret < 0) {
        FTS_ERROR("pram write fail, ret=%d", ret);
        return ret;
    }

    FTS_DEBUG("after write pramboot, confirm run in pramboot");
    state = fts_ft5452i_fwupg_check_state(upg, FTS_RUN_IN_PRAM);
    if (!state) {
        FTS_ERROR("not in pramboot");
        return -EIO;
    }

    ret = fts_ft5452i_pram_init();
    if (ret < 0) {
        FTS_ERROR("pramboot init fail");
        return ret;
    }

    return 0;
}


static int fts_ft5452i_upgrade(u8 *buf, u32 len)
{
    int ret = 0;
    u32 start_addr = 0;
    u8 cmd[4] = { 0 };
    int ecc_in_host = 0;
    int ecc_in_tp = 0;
    int i = 0;
    u8 wbuf[7] = { 0 };
    u8 reg_val[4] = {0};

    if ((NULL == buf) || (len < FTS_MIN_LEN)) {
        FTS_ERROR("buffer/len(%x) is invalid", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_UPGRADE_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_DATA_LEN;
    cmd[1] = BYTE_OFF_16(len);
    cmd[2] = BYTE_OFF_8(len);
    cmd[3] = BYTE_OFF_0(len);
    ret = fts_write(cmd, FTS_CMD_DATA_LEN_LEN);
    if (ret < 0) {
        FTS_ERROR("data len cmd write fail");
        goto fw_reset;
    }

    ret = fts_fwupg_erase(FTS_REASE_APP_DELAY);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto fw_reset;
    }

    /* write app */
    start_addr = upgrade_func_ft5452i.appoff;
    ecc_in_host = fts_flash_write_buf(start_addr, buf, len, 1);
    if (ecc_in_host < 0 ) {
        FTS_ERROR("flash write fail");
        goto fw_reset;
    }

    FTS_INFO( "**********read out checksum**********");

    /* check sum init */
    wbuf[0] = FTS_CMD_ECC_INIT;
    ret = fts_write(wbuf, 1);
    if (ret < 0) {
        FTS_ERROR("ecc init cmd write fail");
        return ret;
    }

    /* send commond to start checksum */
    wbuf[0] = FTS_CMD_ECC_CAL;
    wbuf[1] = BYTE_OFF_16(start_addr);
    wbuf[2] = BYTE_OFF_8(start_addr);
    wbuf[3] = BYTE_OFF_0(start_addr);

    wbuf[4] = BYTE_OFF_16(len);
    wbuf[5] = BYTE_OFF_8(len);
    wbuf[6] = BYTE_OFF_0(len);

    FTS_DEBUG("ecc calc startaddr:0x%04x, len:%d", start_addr, len);
    ret = fts_write(wbuf, 7);
    if (ret < 0) {
        FTS_ERROR("ecc calc cmd write fail");
        return ret;
    }

    msleep(len / 256);

    /* read status if check sum is finished */
    for (i = 0; i < FTS_RETRIES_ECC_CAL; i++) {
        wbuf[0] = FTS_CMD_FLASH_STATUS;
        reg_val[0] = reg_val[1] = 0x00;
        fts_read(wbuf, 1, reg_val, 2);
        FTS_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
        if ((0xF0 == reg_val[0]) && (0x55 == reg_val[1])) {
            break;
        }
        msleep(FTS_RETRIES_DELAY_ECC_CAL);
    }

    /* read out check sum */
    wbuf[0] = FTS_CMD_ECC_READ;
    ret = fts_read(wbuf, 1, reg_val, 1);
    if (ret < 0) {
        FTS_ERROR( "ecc read cmd write fail");
        return ret;
    }
    ecc_in_tp = reg_val[0];

    FTS_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        goto fw_reset;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(200);
    return 0;

fw_reset:
    FTS_INFO("upgrade fail, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }
    return -EIO;
}

struct upgrade_func upgrade_func_ft5452i = {
    .ctype = {0x89},
    .fwveroff = 0x010E,
    .fwcfgoff = 0x1FFB0,
    .appoff = 0x0000,
    .pramboot_supported = true,
    .pramboot = pb_file_ft5452i,
    .pb_length = sizeof(pb_file_ft5452i),
    .pram_ecc_check_mode = ECC_CHECK_MODE_CRC16,
    .write_pramboot_private = fts_ft5452i_write_pramboot_private,
    .new_return_value_from_ic = true,
    .hid_supported = false,
    .upgrade = fts_ft5452i_upgrade,
};
