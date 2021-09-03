#include <linux/kernel.h>

#include "cts_config.h"
#include "cts_firmware.h"
#include "cts_platform.h"

#define _CTS_TCS_C_
#include "cts_tcs.h"
#undef  _CTS_TCS_C_

#define TEST_RESULT_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

#define DIFFDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)


static u8 dump_flag = 1;

void dump_spi(const char *prefix, u8 *data, size_t datalen)
{
    u8 str[1024];
    int offset = 0;
    int i;

    if (!dump_flag) {
        return ;
    }

    offset += snprintf(str + offset, sizeof(str) - offset, "%s", prefix);
    for (i = 0; i < datalen; i++) {
        offset += snprintf(str + offset, sizeof(str) - offset, " %02x", data[i]);
    }
    cts_err("%s", str);
}

static int cts_tcs_spi_xtrans(const struct cts_device *cts_dev, u8 *tx,
        size_t txlen, u8*rx, size_t rxlen)
{
    int ret;
    struct spi_message msg;
    struct spi_transfer xfer[2];
    struct chipone_ts_data *cts_data = container_of(cts_dev,
        struct chipone_ts_data, cts_dev);

    memset(&xfer[0], 0, sizeof(struct spi_transfer));
    xfer[0].cs_change = 0;
    xfer[0].delay_usecs = 0;
    xfer[0].speed_hz = cts_dev->pdata->spi_speed * 1000u;
    xfer[0].tx_buf = tx;
    xfer[0].rx_buf = NULL;
    xfer[0].len = txlen;
    xfer[0].bits_per_word = 8;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer[0],    &msg);
    ret = spi_sync(cts_data->spi_client, &msg);
    if (ret < 0) {
        cts_err("spi_sync xfer[0] failed: %d", ret);
        return ret;
    }
    udelay(100);

    memset(&xfer[1], 0, sizeof(struct spi_transfer));
    xfer[1].cs_change = 0;
    xfer[1].delay_usecs = 0;
    xfer[1].speed_hz = cts_dev->pdata->spi_speed * 1000u;
    xfer[1].tx_buf = NULL;
    xfer[1].rx_buf = rx;
    xfer[1].len = rxlen;
    xfer[1].bits_per_word = 8;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer[1],    &msg);
    ret = spi_sync(cts_data->spi_client, &msg);
    if (ret < 0) {
        cts_err("spi_sync xfer[1] failed: %d", ret);
        return ret;
    }
    udelay(100);

    return 0;
}

static int cts_tcs_spi_read_pack(u8 *tx, TcsCmdValue_t *tcv, u16 rdatalen)
{
    tcs_tx_head *txhdr = (tcs_tx_head *)tx;
    int packlen = 0;
    u16 crc16;

    txhdr->addr = TCS_RD_ADDR;
    txhdr->cmd =
        (tcv->baseFlag << 15) |
        (tcv->isRead   << 14) |
        (tcv->isWrite  << 13) |
        (tcv->classID  <<  8) |
        (tcv->cmdID    <<  0);
    txhdr->datlen = rdatalen;
    crc16 = cts_crc16((const u8 *)txhdr, offsetof(tcs_tx_head, crc16));
    txhdr->crc16 = crc16;
    packlen += sizeof(tcs_tx_head);

    return packlen;
}

static int cts_tcs_spi_write_pack(u8 *tx, TcsCmdValue_t *tcv,
        u8 *wdata, u16 wdatalen)
{
    tcs_tx_head *txhdr = (tcs_tx_head *)tx;
    int packlen = 0;
    u16 crc16;

    txhdr->addr = TCS_WR_ADDR;
    txhdr->cmd =
        (tcv->baseFlag << 15) |
        (tcv->isRead   << 14) |
        (tcv->isWrite  << 13) |
        (tcv->classID  <<  8) |
        (tcv->cmdID    <<  0);
    txhdr->datlen = wdatalen;
    crc16 = cts_crc16((const u8 *)txhdr, offsetof(tcs_tx_head, crc16));
    txhdr->crc16 = crc16;
    packlen += sizeof(tcs_tx_head);

    if (wdatalen > 0) {
        memcpy(tx + sizeof(tcs_tx_head), wdata, wdatalen);
        crc16 = cts_crc16(wdata, wdatalen);
        *(tx + sizeof(tcs_tx_head) + wdatalen) = ((crc16 >> 0) & 0xFF);
        *(tx + sizeof(tcs_tx_head) + wdatalen + 1) = ((crc16 >> 8) & 0xFF);
        packlen += wdatalen + sizeof(crc16);
    }

    return packlen;
}

int cts_tcs_spi_read(const struct cts_device * cts_dev,
        enum TcsCmdIndex cmdIdx, u8 *rdata, size_t rdatalen)
{
    static u8 tx[2048];
    static u8 rx[2048];
    int txlen;

    txlen = cts_tcs_spi_read_pack(tx, TcsCmdValue + cmdIdx, rdatalen);
    /* dump_spi(">> ", tx, txlen); */
    cts_tcs_spi_xtrans(cts_dev, tx, txlen, rx, rdatalen + sizeof(tcs_rx_tail));
    /* dump_spi("<< ", rx, rdatalen + sizeof(tcs_rx_tail)); */
    /**
     * if (rx[rdatalen] != 0) {
     *     cts_err("spi_sync failed: err=%d", rx[rdatalen]);
     * }
     */
    memcpy(rdata, rx, rdatalen);

    return 0;
}

int cts_tcs_spi_write(const struct cts_device * cts_dev,
        enum TcsCmdIndex cmdIdx, u8 *wdata, size_t wdatalen)
{
    static u8 tx[2048];
    static u8 rx[2048];
    int txlen;

    txlen = cts_tcs_spi_write_pack(tx, TcsCmdValue + cmdIdx, wdata, wdatalen);
    /* dump_spi(">> ", tx, txlen); */
    cts_tcs_spi_xtrans(cts_dev, tx, txlen, rx, wdatalen + sizeof(tcs_rx_tail));
    /* dump_spi("<< ", rx, wdatalen + sizeof(tcs_rx_tail)); */
    return rx[0];
}

int cts_tcs_get_fw_ver(const struct cts_device *cts_dev, u16 *fwver)
{
    int ret;
    u8 buf[4];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_FW_VER_RO,
        buf, sizeof(buf));
    if (!ret) {
        *fwver = buf[0] | (buf[1] << 8);
        return 0;
    }
    return -1;
}

int cts_tcs_get_lib_ver(const struct cts_device *cts_dev, u16 *libver)
{
    int ret;
    u8 buf[4];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_FW_VER_RO, buf,
        sizeof(buf));
    if (!ret) {
        *libver = buf[2] | (buf[3] << 8);
        return 0;
    }
    return -1;
}

int cts_tcs_get_fw_id(const struct cts_device *cts_dev, u16 *fwid)
{
    int ret;
    u8 buf[4];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_CHIP_FW_ID_RO,
        buf, sizeof(buf));
    if (!ret) {
        *fwid = buf[0] | (buf[1] << 8);;
        return 0;
    }

    return -1;
}

int cts_tcs_get_ddi_ver(const struct cts_device *cts_dev, u8 *ddiver)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_DDI_CODE_VER_RO,
        buf, sizeof(buf));
    if (!ret) {
        *ddiver = buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_res_x(const struct cts_device *cts_dev, u16 *res_x)
{
    int ret;
    u8 buf[10];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_TOUCH_XY_INFO_RO,
        buf, sizeof(buf));
    if (!ret) {
        *res_x = buf[0] | (buf[1] << 8);
        return 0;
    }
    return -1;
}

int cts_tcs_get_res_y(const struct cts_device *cts_dev, u16 *res_y)
{
    int ret;
    u8 buf[10];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_TOUCH_XY_INFO_RO,
        buf, sizeof(buf));
    if (!ret) {
        *res_y = buf[2] | (buf[3] << 8);
        return 0;
    }
    return -1;
}

int cts_tcs_get_rows(const struct cts_device *cts_dev, u8 *rows)
{
    int ret;
    u8 buf[10];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_TOUCH_XY_INFO_RO,
        buf, sizeof(buf));
    if (!ret) {
        *rows = buf[5];
        return 0;
    }
    return -1;
}

int cts_tcs_get_cols(const struct cts_device *cts_dev, u8 *cols)
{
    int ret;
    u8 buf[10];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_INFO_TOUCH_XY_INFO_RO,
        buf, sizeof(buf));
    if (!ret) {
        *cols = buf[4];
        return 0;
    }
    return -1;
}

int cts_tcs_get_flip_x(const struct cts_device *cts_dev, bool *flip_x)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_COORD_FLIP_X_EN_RW,
        buf, sizeof(buf));
    if (!ret) {
        *flip_x = !!buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_flip_y(const struct cts_device *cts_dev, bool *flip_y)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_COORD_FLIP_Y_EN_RW,
        buf, sizeof(buf));
    if (!ret) {
        *flip_y = !!buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_swap_axes(const struct cts_device *cts_dev, bool *swap_axes)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_COORD_SWAP_AXES_EN_RW,
        buf, sizeof(buf));
    if (!ret) {
        *swap_axes = !!buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_int_mode(const struct cts_device *cts_dev, u8 *int_mode)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_INT_MODE_RW,
        buf, sizeof(buf));
    if (!ret) {
        *int_mode = buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_int_keep_time(const struct cts_device *cts_dev,
        u16 *int_keep_time)
{
    int ret;
    u8 buf[2];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_INT_KEEP_TIME_RW,
        buf, sizeof(buf));
    if (!ret) {
        *int_keep_time = (buf[0] | (buf[1] << 8));
        return 0;
    }
    return -1;

}

int cts_tcs_get_esd_method(const struct cts_device *cts_dev, u8 *esd_method)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_DDI_ESD_OPTIONS_RW,
        buf, sizeof(buf));
    if (!ret) {
        *esd_method = buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_get_esd_protection(const struct cts_device *cts_dev,
        u8 *esd_protection)
{
    int ret;
    u8 buf[4];

    buf[0] = 0x01;
    buf[1] = 0x56;
    buf[2] = 0x81;
    buf[3] = 0x00;

    ret = cts_tcs_spi_write(cts_dev,
        TP_STD_CMD_TP_DATA_OFFSET_AND_TYPE_CFG_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    ret = cts_tcs_spi_read(cts_dev,
        TP_STD_CMD_TP_DATA_OFFSET_AND_TYPE_CFG_RW,
        esd_protection, sizeof(u8));
    if (!ret) {
        return 0;
    }
    return -1;
}

int cts_tcs_get_data_ready_flag(const struct cts_device *cts_dev, u8 *ready)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_DAT_RDY_FLAG_RW,
        buf, sizeof(buf));
    if (!ret) {
        *ready = buf[0];
        return 0;
    }
    return -1;
}

int cts_tcs_clr_data_ready_flag(const struct cts_device *cts_dev)
{
    int ret;
    u8 ready = 0;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_DAT_RDY_FLAG_RW,
        &ready, sizeof(ready));
    return ret;
}

int cts_tcs_enable_get_rawdata(const struct cts_device *cts_dev)
{
    int ret;
    u8 buf[1];

    buf[0] = 0x01;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_DAT_TRANS_IN_NORMAL_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}

int cts_tcs_disable_get_rawdata(const struct cts_device *cts_dev)
{
    int ret;
    u8 buf[1];

    buf[0] = 0x00;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_DAT_TRANS_IN_NORMAL_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}

int cts_tcs_enable_get_cneg(const struct cts_device *cts_dev)
{
    int ret;
    u8 buf[1];

    buf[0] = 0x01;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_CNEG_RD_EN_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}

int cts_tcs_disable_get_cneg(const struct cts_device *cts_dev)
{
    int ret;
    u8 buf[1];

    buf[0] = 0x00;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_CNEG_RD_EN_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}

int cts_tcs_is_cneg_ready(const struct cts_device *cts_dev, u8 *ready)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_CNEG_RDY_FLAG_RW,
        buf, sizeof(buf));
    if (!ret) {
        *ready = buf[0];
        return 0;
    }

    return ret;
}

int cts_tcs_quit_guesture_mode(const struct cts_device *cts_dev)
{
    int ret;
    u8 buf[1];

    buf[0] = 0x00;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_MNT_FORCE_EXIT_MNT_WO,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}


int cts_tcs_get_rawdata(const struct cts_device *cts_dev, u8 *buf, size_t size)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_TP_DATA_RAW_RO, buf,
        RAWDATA_BUFFER_SIZE(cts_dev));

    return ret;
}

int cts_tcs_get_diffdata(const struct cts_device *cts_dev, u8 *buf, size_t size)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_TP_DATA_DIFF_RO, buf,
        DIFFDATA_BUFFER_SIZE(cts_dev));

    return ret;
}

int cts_tcs_get_cneg(const struct cts_device *cts_dev, u8 *buf, size_t size)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_TP_DATA_CNEG_RO, buf, size);

    return ret;

}

int cts_tcs_read_hw_reg(const struct cts_device *cts_dev, u32 addr,
        u8 *regbuf, size_t size)
{
    int ret;
    u8 buf[4];

    buf[0] = 1;
    buf[1] = ((addr >>  0) & 0xFF);
    buf[2] = ((addr >>  8) & 0xFF);
    buf[3] = ((addr >> 16) & 0xFF);

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_TP_DATA_OFFSET_AND_TYPE_CFG_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_TP_DATA_READ_START_RO,
        regbuf, size);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

int cts_tcs_write_hw_reg(const struct cts_device *cts_dev, u32 addr,
        u8 *regbuf, size_t size)
{
    int ret;
    u8 *buf;

    buf = kmalloc(size + 6, GFP_KERNEL);
    if (buf == NULL) {
        return -ENOMEM;
    }

    buf[0] = ((size >>  0) & 0xFF);
    buf[1] = ((size >>  8) & 0xFF);
    buf[2] = ((addr >>  0) & 0xFF);
    buf[3] = ((addr >>  8) & 0xFF);
    buf[4] = ((addr >> 16) & 0xFF);
    buf[5] = 0x00;
    memcpy(buf + 6, regbuf, size);

    ret = cts_tcs_spi_write(cts_dev,
        TP_STD_CMD_TP_DATA_WR_REG_RAM_SEQUENCE_WO,
        buf, size + 6);
    if (ret != 0) {
        kfree(buf);
        return -1;
    }

    kfree(buf);

    return ret;

}

int cts_tcs_read_fw_reg(const struct cts_device *cts_dev, u32 addr,
        u8 *regbuf, size_t size)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_READ_RO, regbuf, size);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

int cts_tcs_write_fw_reg(const struct cts_device *cts_dev, u32 addr,
        u8 *regbuf, size_t size)
{
    int ret;
    u8 buf[4];

    buf[0] = 0x01;
    buf[1] = ((addr >>  0) & 0xFF);
    buf[2] = ((addr >>  8) & 0xFF);
    buf[3] = ((addr >> 16) & 0xFF);


    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_TP_DATA_OFFSET_AND_TYPE_CFG_RW,
        buf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_TP_DATA_OFFSET_AND_TYPE_CFG_RW,
        regbuf, sizeof(buf));
    if (ret != 0) {
        return -1;
    }

    return ret;
}


int cts_tcs_get_touchinfo(const struct cts_device *cts_dev,
        struct cts_device_touch_info *touch_info)
{
    int ret;

    cts_dbg("Get touch info");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Get touch info in program mode");
        return -ENODEV;
    }

    if (cts_dev->rtdata.suspended) {
        cts_warn("Get touch info while is suspended");
        return -ENODEV;
    }

    memset(touch_info, 0, sizeof(*touch_info));
    dump_flag = 0;
    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_TP_DATA_COORDINATES_RO,
        (u8 *)touch_info, sizeof(*touch_info));
    dump_flag = 1;

    return ret;
}

int cts_tcs_get_workmode(const struct cts_device *cts_dev, u8 *workmode)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_CURRENT_WORKMODE_RO, buf, sizeof(buf));
    if (!ret) {
        *workmode = buf[0];
        return 0;
    }

    return -1;
}

int cts_tcs_set_workmode(const struct cts_device *cts_dev, u8 workmode)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_WORK_MODE_RW, &workmode, sizeof(workmode));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_set_openshort_mode(const struct cts_device *cts_dev, u8 mode)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_OPENSHORT_MODE_SEL_RW, &mode, sizeof(mode));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_set_tx_vol(const struct cts_device *cts_dev, u8 txvol)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_SYS_STS_VSTIM_LVL_RW, &txvol, sizeof(txvol));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_is_enabled_get_rawdata(const struct cts_device *cts_dev, u8 *enable)
{
   int ret;
   u8 buf[1];

   ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_SYS_STS_DAT_TRANS_IN_NORMAL_RW, buf, sizeof(buf));
   if (!ret) {
       *enable = buf[0];
       return 0;
   }

   return ret;
}

int cts_tcs_set_short_test_type(const struct cts_device *cts_dev, u8 short_type)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_OPENSHORT_SHORT_SEL_RW, &short_type, sizeof(short_type));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_is_openshort_enabled(const struct cts_device *cts_dev, u8 *enabled)
{
    int ret;
    u8 buf[1];

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_OPENSHORT_EN_RW, buf, sizeof(buf));
    if (!ret) {
        *enabled = buf[0];
        return 0;
    }

    return -1;
}


int cts_tcs_set_openshort_enable(const struct cts_device *cts_dev, u8 enable)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_OPENSHORT_EN_RW, &enable, sizeof(enable));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_set_esd_enable(const struct cts_device *cts_dev, u8 enable)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_DDI_ESD_EN_RW, &enable, sizeof(enable));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_is_cneg_enabled(const struct cts_device *cts_dev, u8 *enabled)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_CNEG_EN_RW, enabled, sizeof(*enabled));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_is_mnt_enabled(const struct cts_device *cts_dev, u8 *enabled)
{
    int ret;

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_MNT_EN_RW, enabled, sizeof(*enabled));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_set_cneg_enable(const struct cts_device *cts_dev, u8 enable)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_CNEG_EN_RW, &enable, sizeof(enable));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_set_mnt_enable(const struct cts_device *cts_dev, u8 enable)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_MNT_EN_RW, &enable, sizeof(enable));
    if (!ret) {
        return 0;
    }

    return -1;
}

int cts_tcs_is_display_on(const struct cts_device *cts_dev, u8 *display_on)
{
    int ret;
    u8 buf[0];

    ret = cts_tcs_spi_read(cts_dev, TP_STD_CMD_OPENSHORT_SHORT_DISP_ON_EN_RW, buf, sizeof(buf));
    if (!ret) {
        *display_on = buf[0];
        return 0;
    }

    return ret;
}

int cts_tcs_set_pwr_mode(const struct cts_device *cts_dev, u8 pwr_mode)
{
	int ret;
	u8 buf[0];

	buf[0] = pwr_mode;
	ret = cts_tcs_spi_write(cts_dev,TP_STD_CMD_SYS_STS_PWR_STATE_RW, buf, 1);
	return ret;
}

int cts_tcs_set_display_on(const struct cts_device *cts_dev, u8 display_on)
{
    int ret;

    ret = cts_tcs_spi_write(cts_dev, TP_STD_CMD_OPENSHORT_SHORT_DISP_ON_EN_RW, &display_on, sizeof(display_on));
    if (!ret) {
        return 0;
    }

    return -1;
}

