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
#include <linux/version.h>

#include "goodix_thp.h"


/* flag */
#define SPI_FLAG_WR					0xF0
#define SPI_FLAG_RD					0xF1
#define MASK_8BIT					0xFF

#define MOVE_8BIT					8
#define MOVE_16BIT					16
#define MOVE_24BIT					24

/* length*/
#define GOODIX_MASK_ID_LEN				6
#define GOODIX_MAX_IC_INFO_LEN				300
#define GOODIX_SPI_BUFF_MAX_SIZE			(8 * 1024 + 16)
#define GOODIX_CMD_LEN_9897				6
#define GOODIX_FRAME_LEN_MAX				2048
#define DEBUG_BUF_LEN					160
#define DEBUG_AFE_DATA_BUF_LEN				20
#define DEBUG_AFE_DATA_BUF_OFFSET			DEBUG_AFE_DATA_BUF_LEN

/* times*/
#define VERSION_INFO_READ_RETRY				5
#define SEND_COMMAND_RETRY				6
#define CHECK_COMMAND_RETRY				5
#define IC_INFO_READ_RETRY				3

/* command ack info */
#define CMD_ACK_BUFFER_OVERFLOW				0x01
#define CMD_ACK_CHECKSUM_ERROR				0x02
#define CMD_ACK_BUSY					0x04
#define CMD_ACK_OK					0x80
#define CMD_ACK_IDLE					0xFF
#define CMD_ACK_UNKNOWN					0x00
#define CMD_ACK_ERROR					(-1)

/* others */
#define RESET_LOW_DELAY_US				2000
#define SEND_COMMAND_END_DELAY				10
#define WAIT_FOR_COMMUNICATION_CHECK_DELAY              10

#define GOODIX_MASK_ID					"BERLIN"
#define GOODIX_MASK_ID_NOCODE				"NOCODE"

#define GOODIX_READ_WRITE_BYTE_OFFSET_GT9897            9

#define GOODIX_RETRY_5					5
#define GOODIX_RETRY_10					10
#define GOODIX_SPI_NORMAL_MODE_0			0x01
#define GOODIX_TRIM_D12_LEVEL				0x3C
#define GOODIX_CLK_STA1_ENABLE				0x77
#define GOODIX_RESET_EN					0xFA
#define GOODIX_CLK_STA0_ENABLE				0xFF
#define GOODIX_SPI_MODE_REG				0xC900
#define GOODIX_REG_TRIM_D12				0xD006
#define GOODIX_REG_CLK_STA1				0xD806
#define GOODIX_REG_CLK_STA0				0xD807
#define GOODIX_REG_RESET				0xD808
#define HOLD_CPU_REG_W					0x0002
#define HOLD_CPU_REG_R					0x2000
#define GOODIX_READ_WRITE_BYTE_OFFSET_GT9916            8

#pragma pack(push, 1)
struct thp_goodix_cmd {
        union {
                struct {
                        u8 state;
                        u8 ack;
                        u8 len;
                        u8 cmd;
                        u8 data[2];
                        u16 checksum;
                };
                u8 buf[8];
	};
};

struct goodix_flash_cmd {
	union {
		struct {
			u8 status;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 fw_type;
			u16 fw_len;
			u32 fw_addr;
			u16 checksum;
		};
		u8 buf[16];
	};
};

struct goodix_version_info {
	u8 rom_pid[6];               /* rom PID */
	u8 rom_vid[3];               /* Mask VID */
	u8 rom_vid_reserved;
	u8 patch_pid[8];              /* Patch PID */
	u8 patch_vid[4];              /* Patch VID */
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	u16 checksum;
};
#pragma pack(pop)

u16 checksum16_cmp(u8 *data, u32 size, int mode)
{
        u16 cal_checksum = 0;
        u16 checksum;
        u32 i;

        if (size < sizeof(u16)) {
                ts_err("inval size %d", size);
                return 1;
        }

        for (i = 0; i < size - sizeof(u16); i++)
                cal_checksum += data[i];
        if (mode == GOODIX_BE_MODE)
                checksum = (data[size - sizeof(u16)] << MOVE_8BIT) +
                        data[size - 1];
        else
                checksum = data[size - sizeof(u16)] +
                        (data[size - 1] << MOVE_8BIT);

        return cal_checksum == checksum ? 0 : 1;
}

u8 checksum_u8(u8 *data, u32 size)
{
        u8 checksum = 0;
        u32 i;

        for (i = 0; i < size; i++)
                checksum += data[i];
        return checksum;
}

u8 checksum8_u16(const u8 *data, u32 size)
{
        int non_zero_count = 0;
        u16 checksum = 0;
        u32 i;

        if (size < sizeof(u16)) {
                ts_err("inval size %d", size);
                return 1;
        }

        for (i = 0; i < size - 2; i++) {
                if (data[i])
                        non_zero_count++;
                checksum += data[i];
        }

        if (!non_zero_count) {
                ts_err("buf data is all 0\n");
                return 0xFF;
        }
        return checksum + ((data[i] << 8) + data[i + 1]);
}

u32 checksum16_u32(const u8 *data, int size)
{
    int i;
    u32 checksum = 0;

        if (size < sizeof(u32)) {
                ts_err("inval size %d", size);
                return 1;
        }

    for (i = 0; i < size - 4; i += 2)
    {
        checksum += (data[i] << 8) | data[i + 1];
    }
    checksum += (data[i] << 24) +
                (data[i + 1] << 16) +
                (data[i + 2] << 8) +
                (data[i + 3]);
    return checksum;
}

#ifdef CONFIG_OF
static int goodix_thp_parse_spi_setting(struct device_node *node,
        struct goodix_thp_board_data *board_data)
{
        int r;
        unsigned int value;
        struct thp_spi_setting *spi_setting;

        spi_setting = &board_data->spi_setting;

        /* max freq */
        value = 0;
        r = of_property_read_u32(node, "spi-max-frequency", &value);
        if (!r) {
                spi_setting->spi_max_speed = value;
                ts_info("spi-max-frequency configed %d", value);
        } else {
                ts_err("invalid spi-max-frequency, r %d", r);
                goto exit;
        }

        /* spi mode */
        value = 0;
        r = of_property_read_u32(node, "spi-mode", &value);
        if (!r) {
                spi_setting->spi_mode = value;
                ts_info("spi-mode configed %d", value);
        } else {
                ts_err("invalid spi-mode, r %d", r);
                goto exit;
        }

        /* bits per word */
        value = 0;
        r = of_property_read_u32(node, "bits-per-word", &value);
        if (!r) {
                spi_setting->bits_per_word = value;
                ts_info("bits-per-word configed %d", value);
        } else {
                ts_err("invalid bits-per-word, r %d", r);
                goto exit;
        }

exit:
        return r;
}

/**
 * goodix_thp_parse_dt_resolution - parse resolution from dt
 * @node: devicetree node
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_thp_parse_dt_resolution(struct device_node *node,
                struct goodix_thp_board_data *board_data)
{
        int r;

        r = of_property_read_u32(node, "goodix,panel-max-x",
                                 &board_data->panel_max_x);
        if (r)
                ts_err("invalid goodix,panel-max-x, r %d", r);

        r = of_property_read_u32(node, "goodix,panel-max-y",
                                 &board_data->panel_max_y);
        if (r)
                ts_err("invalid goodix,panel-max-y, r %d", r);

        r = of_property_read_u32(node, "goodix,panel-max-w",
                                 &board_data->panel_max_w);
        if (r)
                ts_err("invalid goodix,panel-max-w, r %d", r);

        r = of_property_read_u32(node, "goodix,panel-max-p",
                                 &board_data->panel_max_p);
        if (r)
                ts_err("invalid goodix,panel-max-p, r %d", r);

        return 0;
}

/**
 * goodix_thp_parse_dt- parse board data from dt
 * @dev: pointer to device
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_thp_parse_dt(struct device_node *node,
        struct goodix_thp_board_data *board_data)
{
        //struct property *prop;
        const char *name_tmp;
        int r;

        if (!board_data) {
                ts_err("invalid board data");
                return -EINVAL;
        }

        /* get spi property */
        r = goodix_thp_parse_spi_setting(node, board_data);
        if (r < 0) {
                ts_err("parse spi config fail from dt: %d", r);
                return -EINVAL;
        }

        /* get chip-type*/
        r = of_property_read_u32(node, "chip-type",
                        &board_data->chip_type);
        if (r) {
                ts_err("invalid chip_type");
                return -EINVAL;
        }
        ts_info("get chip-type[%d] from dt", board_data->chip_type);

        /* get gpio property*/
        r = of_get_named_gpio(node, "goodix,reset-gpio", 0);
        if (r < 0) {
                ts_err("invalid reset-gpio in dt: %d", r);
                return -EINVAL;
        }
        ts_info("get reset-gpio[%d] from dt", r);
        board_data->reset_gpio = r;

        r = of_get_named_gpio(node, "goodix,irq-gpio", 0);
        if (r < 0) {
                ts_err("invalid irq-gpio in dt: %d", r);
                return -EINVAL;
        }
        ts_info("get irq-gpio[%d] from dt", r);
        board_data->irq_gpio = r;

        /* get irq trigger type property*/
        r = of_property_read_u32(node, "goodix,irq-flags",
                        &board_data->irq_flags);
        if (r) {
                ts_err("invalid irq-flags");
                return -EINVAL;
        }

        /* get power property*/
        memset(board_data->avdd_name, 0, sizeof(board_data->avdd_name));
        r = of_property_read_string(node, "goodix,avdd-name", &name_tmp);
        if (!r) {
                ts_info("avdd name form dt: %s", name_tmp);
                if (strlen(name_tmp) < sizeof(board_data->avdd_name))
                        strncpy(board_data->avdd_name,
                                name_tmp, sizeof(board_data->avdd_name));
                else
                        ts_info("invalied avdd name length: %ld > %ld",
                                strlen(name_tmp),
                                sizeof(board_data->avdd_name));
        }

        memset(board_data->iovdd_name, 0, sizeof(board_data->iovdd_name));
        r = of_property_read_string(node, "goodix,iovdd-name", &name_tmp);
        if (!r) {
                ts_info("avdd name form dt: %s", name_tmp);
                if (strlen(name_tmp) < sizeof(board_data->iovdd_name))
                        strncpy(board_data->iovdd_name,
                                name_tmp, sizeof(board_data->iovdd_name));
                else
                        ts_info("invalied avdd name length: %ld > %ld",
                                strlen(name_tmp),
                                sizeof(board_data->iovdd_name));
        }

        r = of_get_named_gpio(node, "goodix,iovdd-gpio", 0);
        if (r < 0) {
            ts_info("can't find iovdd-gpio, use other power supply");
            board_data->iovdd_gpio = 0;
        } else {
            ts_info("get iovdd-gpio[%d] from dt", r);
            board_data->iovdd_gpio = r;
        }

        r = of_property_read_u32(node, "goodix,power-on-delay-us",
                                &board_data->power_on_delay_us);
        if (!r) {
                /* 1000ms is too large, maybe you have pass a wrong value */
                if (board_data->power_on_delay_us > 1000 * 1000) {
                        ts_err("Power on delay time exceed 1s, please check");
                        board_data->power_on_delay_us = 0;
                }
        }

        r = of_property_read_u32(node, "goodix,power-off-delay-us",
                                &board_data->power_off_delay_us);
        if (!r) {
                /* 1000ms is too large, maybe you have pass */
                if (board_data->power_off_delay_us > 1000 * 1000) {
                        ts_err("Power off delay time exceed 1s, please check");
                        board_data->power_off_delay_us = 0;
                }
        }

        /* get xyz resolutions */
        r = goodix_thp_parse_dt_resolution(node, board_data);
        if (r < 0) {
                ts_err("Failed to parse resolutions:%d", r);
                return r;
        }

        board_data->interpolation_ctrl = of_property_read_bool(node,
                "goodix,interpolation-ctrl");
        if (board_data->interpolation_ctrl)
            ts_info("support goodix interpolation mode");

        board_data->sample_ctrl = of_property_read_bool(node,
                "goodix,sample-ctrl");
        if (board_data->sample_ctrl)
            ts_info("support goodix sample mode");

        board_data->stowed_mode_ctrl = of_property_read_bool(node,
                "goodix,stowed-mode-ctrl");
        if (board_data->stowed_mode_ctrl)
            ts_info("Support goodix touch stowed mode");

        board_data->pocket_mode_ctrl = of_property_read_bool(node,
            "goodix,pocket-mode-ctrl");
        if (board_data->pocket_mode_ctrl)
            ts_info("Support goodix touch pocket mode");

        board_data->edge_ctrl = of_property_read_bool(node,
                "goodix,edge-ctrl");
        if (board_data->edge_ctrl)
            ts_info("support goodix edge mode");

        r = of_property_read_u32(node, "touchpanel,irq_need_dev_resume_time", &board_data->irq_need_dev_resume_time);
        if (r) {
            ts_info("board_data->irq_need_dev_resume_time not specified");
            board_data->irq_need_dev_resume_time = 50;
        }
        ts_info("board_data->irq_need_dev_resume_time = %d ms", board_data->irq_need_dev_resume_time);

        if (of_property_read_u32(node, "goodix,sched-priority", &board_data->sched_priority)) {
            ts_info("Failed to read sched-priority");
            board_data->sched_priority = 49;
        }

        if (of_property_read_u32(node, "goodix,cpu-affinity", &board_data->cpu_mask)) {
            ts_info("Failed to read cpu-affinity");
            board_data->cpu_mask = 0xff;
        }

        return 0;
}
#endif

/**
 * goodix_thp_spi_read- read device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - spi transter error
 */
static int goodix_thp_spi_read(struct thp_ts_device *dev, unsigned int addr,
        unsigned char *data, unsigned int len)
{
        struct spi_device *spi = dev->spi_dev;
        u8 *rx_buf = dev->rx_buff;
        u8 *tx_buf = dev->tx_buff;
        struct spi_transfer xfers;
        struct spi_message spi_msg;
        int ret = 0;

        mutex_lock(&dev->spi_mutex);
        spi_message_init(&spi_msg);
        memset(&xfers, 0, sizeof(xfers));

        if (dev->board_data.chip_type == CHIP_TYPE_9916 ||
                        dev->board_data.chip_type == CHIP_TYPE_9966 ||
                        dev->board_data.chip_type == CHIP_TYPE_9615) {
                tx_buf[0] = SPI_FLAG_RD; /* 0xF1 start read flag */
                tx_buf[1] = (addr >> MOVE_24BIT) & MASK_8BIT;
                tx_buf[2] = (addr >> MOVE_16BIT) & MASK_8BIT;
                tx_buf[3] = (addr >> MOVE_8BIT) & MASK_8BIT;
                tx_buf[4] = addr & MASK_8BIT;
                tx_buf[5] = MASK_8BIT;
                tx_buf[6] = MASK_8BIT;
                tx_buf[7] = MASK_8BIT;

                xfers.tx_buf = tx_buf;
                xfers.rx_buf = rx_buf;
                xfers.len = len + GOODIX_READ_WRITE_BYTE_OFFSET_GT9916;
                //TODO：to confirm later yuanjie
                xfers.cs_change = 0;
                spi_message_add_tail(&xfers, &spi_msg);
        } else if (dev->board_data.chip_type == CHIP_TYPE_9897) {
                tx_buf[0] = SPI_FLAG_RD; /* 0xF1 start read flag */
                tx_buf[1] = (addr >> MOVE_24BIT) & MASK_8BIT;
                tx_buf[2] = (addr >> MOVE_16BIT) & MASK_8BIT;
                tx_buf[3] = (addr >> MOVE_8BIT) & MASK_8BIT;
                tx_buf[4] = addr & MASK_8BIT;
                tx_buf[5] = MASK_8BIT;
                tx_buf[6] = MASK_8BIT;
                tx_buf[7] = MASK_8BIT;
                tx_buf[8] = MASK_8BIT;

                xfers.tx_buf = tx_buf;
                xfers.rx_buf = rx_buf;
                xfers.len = len + GOODIX_READ_WRITE_BYTE_OFFSET_GT9897;
                xfers.cs_change = 1;
                spi_message_add_tail(&xfers, &spi_msg);
        }

        ret = spi_sync(spi, &spi_msg);
        if (ret < 0) {
                ts_err("Spi transfer error:%d", ret);
                goto exit;
        }

        if (dev->board_data.chip_type == CHIP_TYPE_9916 ||
                        dev->board_data.chip_type == CHIP_TYPE_9966 ||
                        dev->board_data.chip_type == CHIP_TYPE_9615) {
                memcpy(data, &rx_buf[GOODIX_READ_WRITE_BYTE_OFFSET_GT9916], len);
        } else if (dev->board_data.chip_type == CHIP_TYPE_9897) {
                memcpy(data, &rx_buf[GOODIX_READ_WRITE_BYTE_OFFSET_GT9897], len);
        }

exit:
        mutex_unlock(&dev->spi_mutex);
        return ret;
}

/**
 * goodix_thp_spi_write- write device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - spi transter error.
 */
static int goodix_thp_spi_write(struct thp_ts_device *dev, unsigned int addr,
                unsigned char *data, unsigned int len)
{
        struct spi_device *spi = dev->spi_dev;
        u8 *tx_buf = dev->tx_buff;
        struct spi_transfer xfers;
        struct spi_message spi_msg;
        int ret = 0;

        mutex_lock(&dev->spi_mutex);
        spi_message_init(&spi_msg);
        memset(&xfers, 0, sizeof(xfers));

        if (dev->board_data.chip_type == CHIP_TYPE_9916 ||
                        dev->board_data.chip_type == CHIP_TYPE_9966 ||
                        dev->board_data.chip_type == CHIP_TYPE_9615) {
                tx_buf[0] = SPI_FLAG_WR; /* 0xF1 start read flag */
                tx_buf[1] = (addr >> MOVE_24BIT) & MASK_8BIT;
                tx_buf[2] = (addr >> MOVE_16BIT) & MASK_8BIT;
                tx_buf[3] = (addr >> MOVE_8BIT) & MASK_8BIT;
                tx_buf[4] = addr & MASK_8BIT;
                memcpy(&tx_buf[5], data, len);
                xfers.len = len + 5;
        }  else {
		memcpy(&tx_buf[0], data, len);
		xfers.len = len;
	}

        xfers.tx_buf = tx_buf;
        xfers.cs_change = 0;
        spi_message_add_tail(&xfers, &spi_msg);

        ret = spi_sync(spi, &spi_msg);
        if (ret < 0)
                ts_err("Spi transfer error:%d", ret);

        mutex_unlock(&dev->spi_mutex);

        return ret;
}

static int goodix_thp_get_cmd_ack(struct thp_ts_device *tdev, unsigned int ack_reg)
{
        int ret;
        int i;
        u8 cmd_ack = 0;
        u8 cmd_ack_buf[8] = {0};

        for (i = 0; i < CHECK_COMMAND_RETRY; i++) {
                /* check command result */
                ret = goodix_thp_spi_read(tdev, ack_reg,
                        cmd_ack_buf, sizeof(cmd_ack_buf));
                if (ret < 0) {
                        ts_err("%s: failed read cmd ack info, ret %d",
                                __func__, ret);
                        return -EINVAL;
                }

		cmd_ack = cmd_ack_buf[1];
                if (cmd_ack != CMD_ACK_OK) {
                        ret = CMD_ACK_ERROR;
                        if (cmd_ack == CMD_ACK_BUFFER_OVERFLOW) {
                                mdelay(10); /* delay 10 ms */
                                break;
                        } else if ((cmd_ack == CMD_ACK_BUSY) ||
                                (cmd_ack == CMD_ACK_UNKNOWN)) {
                                mdelay(10);
                                continue;
                        }
                        mdelay(1); /* delay 1 ms */
                        break;
                }
                ret = 0;
                mdelay(SEND_COMMAND_END_DELAY);
		goto exit;
        }
exit:
        return ret;
}

static int goodix_thp_send_cmd(struct thp_ts_device *tdev, u8 cmd, u16 data)
{
        int ret;
        int i;
        u32 cmd_addr = tdev->board_data.cmd_addr;
        struct thp_goodix_cmd cmd_send = {0};

        if (cmd_addr == 0) {
                ts_err("cmd addr has not been assigned");
                return -EINVAL;
        }

        cmd_send.len = GOODIX_CMD_LEN_9897;
        cmd_send.cmd = cmd;
        cmd_send.data[0] = data & MASK_8BIT;
        cmd_send.data[1] = (data >> MOVE_8BIT) & MASK_8BIT;
        cmd_send.checksum = cpu_to_le16(cmd_send.len + cmd_send.cmd +
                                        cmd_send.data[0] + cmd_send.data[1]);
        
        for (i = 0; i < SEND_COMMAND_RETRY; i++) {
                ret = goodix_thp_spi_write(tdev, cmd_addr, cmd_send.buf,
                        sizeof(cmd_send));
                if (ret < 0) {
                        ts_err("failed send command, ret %d", ret);
                        return -EINVAL;
                }

                ret = goodix_thp_get_cmd_ack(tdev, cmd_addr);
		if (ret) {
			ts_err("cmd ack read back error, ret %d", ret);
			continue;
		} else {
			ts_info("cmd ack read back ok!");
			break;
		}
	}

        return ret;
}

static int goodix_thp_select_spi_mode(struct goodix_thp_core *cd)
{
        int ret;
        int i;
        u8 w_value = GOODIX_SPI_NORMAL_MODE_0;
        u8 r_value;

        for (i = 0; i < GOODIX_RETRY_5; i++) {
                cd->ts_dev->hw_ops->write(cd->ts_dev, GOODIX_SPI_MODE_REG,
                                &w_value, 1);
                ret = cd->ts_dev->hw_ops->read(cd->ts_dev, GOODIX_SPI_MODE_REG,
                                &r_value, 1);
                if (!ret && r_value == w_value) {
                        return 0;
                }
        }
        ts_err("failed switch SPI mode after reset");
        return -EINVAL;
}

int goodix_thp_reset_after(struct goodix_thp_core *cd)
{
        u8 reg_val[2] = {0};
        u8 temp_buf[12] = {0};
        int ret;
        int retry;

        ts_info("IN");
        usleep_range(5000, 5100);

        /* select spi mode */
        ret = goodix_thp_select_spi_mode(cd);
        if (ret < 0)
                return ret;

        /* hold cpu */
        retry = GOODIX_RETRY_10;
        while (retry--) {
                reg_val[0] = 0x01;
                reg_val[1] = 0x00;
                ret = cd->ts_dev->hw_ops->write(cd->ts_dev, HOLD_CPU_REG_W, reg_val, 2);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, HOLD_CPU_REG_R, &temp_buf[0], 4);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, HOLD_CPU_REG_R, &temp_buf[4], 4);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, HOLD_CPU_REG_R, &temp_buf[8], 4);
                if (!ret && !memcmp(&temp_buf[0], &temp_buf[4], 4) &&
                        !memcmp(&temp_buf[4], &temp_buf[8], 4) &&
                        !memcmp(&temp_buf[0], &temp_buf[8], 4)) {
                        break;
                }
        }
        if (retry < 0) {
                ts_err("failed to hold cpu");
                return -EINVAL;
        }

        /* enable sta0 clk */
        retry = GOODIX_RETRY_5;
        while (retry--) {
                reg_val[0] = GOODIX_CLK_STA0_ENABLE;
                ret = cd->ts_dev->hw_ops->write(cd->ts_dev, GOODIX_REG_CLK_STA0, reg_val, 1);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, GOODIX_REG_CLK_STA0, temp_buf, 1);
                if (!ret && temp_buf[0] == GOODIX_CLK_STA0_ENABLE)
                        break;
        }
        if (retry < 0) {
                ts_err("failed to enable sta0 clk");
                return -EINVAL;
        }

        /* enable sta1 clk */
        retry = GOODIX_RETRY_5;
        while (retry--) {
                reg_val[0] = GOODIX_CLK_STA1_ENABLE;
                ret = cd->ts_dev->hw_ops->write(cd->ts_dev, GOODIX_REG_CLK_STA1, reg_val, 1);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, GOODIX_REG_CLK_STA1, temp_buf, 1);
                if (!ret && temp_buf[0] == GOODIX_CLK_STA1_ENABLE)
                        break;
        }
        if (retry < 0) {
                ts_err("failed to enable sta1 clk");
                return -EINVAL;
        }

        /* set D12 level */
        retry = GOODIX_RETRY_5;
        while (retry--) {
                reg_val[0] = GOODIX_TRIM_D12_LEVEL;
                ret = cd->ts_dev->hw_ops->write(cd->ts_dev, GOODIX_REG_TRIM_D12, reg_val, 1);
                ret |= cd->ts_dev->hw_ops->read(cd->ts_dev, GOODIX_REG_TRIM_D12, temp_buf, 1);
                if (!ret && temp_buf[0] == GOODIX_TRIM_D12_LEVEL)
                        break;
        }
        if (retry < 0) {
                ts_err("failed to set D12");
                return -EINVAL;
        }

        usleep_range(5000, 5100);
        /* soft reset */
        reg_val[0] = GOODIX_RESET_EN;
        ret = cd->ts_dev->hw_ops->write(cd->ts_dev, GOODIX_REG_RESET, reg_val, 1);
        if (ret < 0)
                return ret;

        /* select spi mode */
        ret = goodix_thp_select_spi_mode(cd);
        if (ret < 0)
                return ret;

        ts_info("OUT");

        return 0;
}

/* prepare to confirm spi normal.
 * If confirmed 0 will return.
 */
static int goodix_thp_prepare(struct thp_ts_device *ts_dev)
{
        int retry = 3;
	u8 tx_buf[5] = {0};
	u8 rx_buf[5] = {0};

        ts_info("%s IN", __func__);

        /* reset ic */
        ts_dev->hw_ops->reset(ts_dev, 5);

        memset(tx_buf, 0xAA, sizeof(tx_buf));
        while (retry--) {
                goodix_thp_spi_write(ts_dev, 0x10000, tx_buf, sizeof(tx_buf));
                goodix_thp_spi_read(ts_dev, 0x10000, rx_buf, sizeof(rx_buf));
		if (!memcmp(tx_buf, rx_buf, sizeof(tx_buf)))
			break;
		usleep_range(5000, 5100);
        }
        if (retry < 0) {
                ts_err("device confirm failed, rx_buf:%*ph", 5, rx_buf);
                return -EINVAL;
        }

        ts_info("device online");
        msleep(100);
        return 0;
}

static int goodix_thp_get_ic_version(struct thp_ts_device *tdev)
{
        struct goodix_version_info version_info;
        int len, ret, retry;
	u8 temp_buf[DEBUG_BUF_LEN] = {0};
        u32 ver_addr;

        if (tdev->board_data.chip_type == CHIP_TYPE_9897)
                ver_addr = 0x1000C;
        else
                ver_addr = 0x10014;

        len = sizeof(version_info);
        memset(&version_info, 0, len);
        for (retry = 0; retry < VERSION_INFO_READ_RETRY; retry++) {
                ret = goodix_thp_spi_read(tdev, ver_addr,
                                (u8 *)&version_info, len);
                if (!ret && !checksum16_cmp((u8 *)&version_info,
					len, GOODIX_LE_MODE))
			break;
                /* retry need delay 10ms */
		usleep_range(10000, 11000);
        }
        ts_info("hw info: ret %d, retry %d", ret, retry);
        if (retry == VERSION_INFO_READ_RETRY) {
                ts_err("failed read module info");
                return -EINVAL;
        }

        memcpy(temp_buf, version_info.rom_pid, sizeof(version_info.rom_pid));
        ts_info("rom_pid:%s", temp_buf);
	ts_info("rom_vid:%*ph", (int)sizeof(version_info.rom_vid),
		version_info.rom_vid);
        memcpy(temp_buf, version_info.patch_pid, sizeof(version_info.patch_pid));
        ts_info("patch_pid:%s", temp_buf);
	ts_info("patch_vid:%*ph", (int)sizeof(version_info.patch_vid),
		version_info.patch_vid);

        return 0;
}

static int goodix_thp_board_init(struct thp_ts_device *tdev)
{
        int ret = -1;

        ts_info("%s IN", __func__);

        if (!tdev) {
                ts_err("thp_ts_device null!");
                return -EINVAL;
        }

        /* reset ic & ic_init */
        ret = goodix_thp_prepare(tdev);
        if (ret) {
                ts_err("goodix device prepare failed, ret %d", ret);
                goto out;
        }

        /* get ic version */
        goodix_thp_get_ic_version(tdev);

out:
        return ret;
}

static int is_valid_custom_info(char *id)
{
        int i;

        if (id == NULL)
                return false;
        
        for (i = 0; i < GOODIX_THP_CUSTOM_INFO_LEN; i++) {
                if (!isascii(*id) || !isalnum(*id))
                        return false;
                id++;
        }

        return true;
}

static int goodix_thp_get_custom_info(struct thp_ts_device *tdev, char *buf,
                                        unsigned int len)
{
        char custom_info[GOODIX_THP_CUSTOM_INFO_LEN + 1] = {0};

        if (tdev->board_data.chip_type == CHIP_TYPE_9897)
                strncpy(custom_info, "P1729S1300", len);
        else if (tdev->board_data.chip_type == CHIP_TYPE_9916 ||
                        tdev->board_data.chip_type == CHIP_TYPE_9966 ||
                        tdev->board_data.chip_type == CHIP_TYPE_9615)
                strncpy(custom_info, "H018AM2900", len);
        ts_info("custom_info[0-9] %*ph", 10, custom_info);
        
        if (is_valid_custom_info(custom_info)) {
                strncpy(buf, custom_info, len);
        } else {
                ts_err("%s:get custom info fail", __func__);
                return -EIO;
        }
        return 0;
}

#define FRAME_HEAD_LEN 14
static int goodix_thp_get_frame(struct thp_ts_device *tdev, char *buf)
{        
        u32 addr = tdev->board_data.frame_addr;
        u16 frame_len;
        u16 cal_checksum = 0;
        u16 checksum;
        int ret;
        int i;

        if (!tdev) {
                ts_err("thp_ts_device null!");
                return -EINVAL;
        }

        if (addr == 0) {
                ts_err("frame addr has not been assigned");
                return -EINVAL;
        }

        ret = goodix_thp_spi_read(tdev, addr, buf, FRAME_HEAD_LEN);
        if (ret < 0) {
                ts_err("read frame head failed");
                return ret;
        }
        for (i = 0; i < FRAME_HEAD_LEN - 2; i++)
                cal_checksum += buf[i];
        checksum = le16_to_cpup((__le16 *)&buf[FRAME_HEAD_LEN - 2]);
        if (checksum != cal_checksum) {
                ts_err("frame head checksum error, %*ph", FRAME_HEAD_LEN, buf);
                return -EINVAL;
        }

        frame_len = le16_to_cpup((__le16 *)&buf[6]);
        if (frame_len == 0 || frame_len > GOODIX_THP_MAX_FRAME_LEN) {
                ts_err("invalid frame_len:%d", frame_len);
                return -EINVAL;
        }
        goodix_thp_spi_read(tdev, addr + FRAME_HEAD_LEN,
                buf + FRAME_HEAD_LEN, frame_len - FRAME_HEAD_LEN);

        return frame_len;
}

static int goodix_thp_get_version(struct thp_ts_device *tdev, u64 *version)
{


    return 0;
}

/* level: 1-HIGH 0-LOW */
static int goodix_thp_set_fp_int_pin(struct thp_ts_device *dev, u8 level)
{
        int ret;

        ret = goodix_thp_send_cmd(dev, 0x30, level);
        if (ret < 0) {
                ts_err("failed to set fp pin level");
                return ret;
        }
        return 0;
}

static int goodix_thp_reset(struct thp_ts_device *dev, u32 delay_ms)
{
        ts_info("reset %dms", delay_ms);

        gpio_direction_output(dev->board_data.reset_gpio, 0);
        udelay(RESET_LOW_DELAY_US);
        gpio_direction_output(dev->board_data.reset_gpio, 1);
        if (delay_ms > 0) {
                if (delay_ms < 20)
                        usleep_range(delay_ms * 1000, delay_ms * 1000 + 100);
                else
                        msleep(delay_ms);
        }
        return 0;
}

static int goodix_thp_set_spi_speed(struct thp_ts_device *dev, u32 speed)
{
        struct spi_device *spi = dev->spi_dev;
        int ret;

        spi->mode = dev->board_data.spi_setting.spi_mode;
        spi->max_speed_hz = speed;
        spi->bits_per_word = dev->board_data.spi_setting.bits_per_word;

        ret = spi_setup(spi);
        if (ret)
                ts_err("failed setup spi speed to %d, ret %d",
                                spi->max_speed_hz, ret);

        return ret;
}

/* hardware opeation funstions */
static const struct goodix_thp_hw_ops hw_spi_ops = {
        .read = goodix_thp_spi_read,
        .write = goodix_thp_spi_write,
        .send_cmd = goodix_thp_send_cmd,
        .board_init = goodix_thp_board_init,
        .get_custom_info = goodix_thp_get_custom_info,
        .get_frame = goodix_thp_get_frame,
        .get_version = goodix_thp_get_version,
        .set_fp_int_pin = goodix_thp_set_fp_int_pin,
        .reset = goodix_thp_reset,
        .set_spi_speed = goodix_thp_set_spi_speed,
};

static void goodix_pdev_release(struct device *dev)
{
        struct platform_device *pdev = to_platform_device(dev);

        ts_info("goodix pdev released, id:%d", pdev->id);
        kfree(pdev);
}

static int goodix_spi_probe(struct spi_device *spi)
{
        struct platform_device *pdev = NULL;
        struct thp_ts_device *ts_dev = NULL;
        static int pdev_id;
        int r = 0;

        ts_info("%s IN", __func__);

        /* init thp device data */
        ts_dev = devm_kzalloc(&spi->dev,
                sizeof(struct thp_ts_device), GFP_KERNEL);
        if (!ts_dev) {
                ts_err("out of memory");
                return -ENOMEM;
        }

        /* alloc memory for spi transfer buffer */
        ts_dev->tx_buff = devm_kzalloc(&spi->dev, GOODIX_SPI_BUFF_MAX_SIZE, GFP_KERNEL);
        ts_dev->rx_buff = devm_kzalloc(&spi->dev, GOODIX_SPI_BUFF_MAX_SIZE, GFP_KERNEL);
        if (!ts_dev->tx_buff || !ts_dev->rx_buff) {
                ts_err("%s: out of memory\n", __func__);
                r = -ENOMEM;
                goto err_spi_buf;
        }

        ts_dev->name = "Goodix ThpDevcie";
        ts_dev->spi_dev= spi;
        ts_dev->dev= &spi->dev;
        ts_dev->hw_ops = &hw_spi_ops;
        mutex_init(&ts_dev->spi_mutex);

        /* parse device tree property */
        if (IS_ENABLED(CONFIG_OF) && spi->dev.of_node) {
                r = goodix_thp_parse_dt(spi->dev.of_node,
                                    &ts_dev->board_data);
                if (r < 0) {
                        ts_err("failed parse device info form dts, %d", r);
                        r = -EINVAL;
                        goto err_spi_buf;
                }
        } else {
                ts_err("no valid device tree node found");
                r = -ENODEV;
                goto err_spi_buf;
        }

        /* init spi_device */
        spi->mode = ts_dev->board_data.spi_setting.spi_mode;
        spi->bits_per_word = ts_dev->board_data.spi_setting.bits_per_word;
        spi->max_speed_hz = ts_dev->board_data.spi_setting.spi_max_speed;

        /* init ts core device */
        pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
        if (!pdev) {
                ts_err("kzalloc core dev pdev faliled!");
                r = -ENOMEM;
                goto err_spi_buf;
        }

        /*
         * you can find this platform dev in
         * /sys/devices/platfrom/goodix_thp.0
         * pdev->dev.parent = &spi->dev;
         */
        pdev->name = GOODIX_CORE_DRIVER_NAME;
        pdev->id = pdev_id++;
        pdev->num_resources = 0;
        pdev->dev.platform_data = ts_dev;
        pdev->dev.release = goodix_pdev_release;
        spi_set_drvdata(spi, pdev);

        /* register platform device, then the goodix_thp_core
         * module will probe the touch deivce.
         */
        ts_info("platform device register, id:%d", pdev->id);
        r = platform_device_register(pdev);
        if (r) {
                ts_err("failed register goodix platform device, %d", r);
                goto err_pdev;
        }

        ts_info("%s OUT", __func__);
        return r;

err_pdev:
        if (pdev) {
                kfree(pdev);
                pdev = NULL;
        }
err_spi_buf:
        ts_info("OUT, %d", r);
        return r;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void goodix_spi_remove(struct spi_device *spi)
{
        struct platform_device *pdev = spi_get_drvdata(spi);

	ts_info("goodix spi driver remove, id:%d", pdev->id);
	platform_device_unregister(pdev);
}
#else
static int goodix_spi_remove(struct spi_device *spi)
{
        struct platform_device *pdev = spi_get_drvdata(spi);

        ts_info("goodix spi driver remove, id:%d", pdev->id);
        platform_device_unregister(pdev);
        return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id spi_matchs[] = {
        {.compatible = "goodix_thp",},
        {},
};
#endif

static const struct spi_device_id spi_id_table[] = {
        {GOODIX_THP_DRIVER_NAME, 0},
        {},
};

static int goodix_spi_resume(struct device *dev)
{
        struct goodix_thp_core *core_data = dev_get_drvdata(dev);

        core_data->bus_ready = true;
        wake_up_interruptible(&core_data->wait);
        return 0;
}

static int goodix_spi_suspend(struct device *dev)
{
        struct goodix_thp_core *core_data = dev_get_drvdata(dev);

        core_data->bus_ready = false;
        return 0;
}

static const struct dev_pm_ops goodix_pm_ops = {
        .suspend = goodix_spi_suspend,
        .resume = goodix_spi_resume,
};

static struct spi_driver goodix_spi_driver = {
        .driver = {
                .name = GOODIX_THP_DRIVER_NAME,
                .owner = THIS_MODULE,
                .bus = &spi_bus_type,
                .of_match_table = spi_matchs,
                .pm = &goodix_pm_ops,
        },
        .id_table = spi_id_table,
        .probe = goodix_spi_probe,
        .remove = goodix_spi_remove,
};

static int __init goodix_thp_spi_init(void)
{
        int ret;

        ts_info("Goodix thp driver init");
        ret = goodix_thp_core_init();
        if (ret < 0) {
                ts_err("failed register platform driver");
                return ret;
        }
        return spi_register_driver(&goodix_spi_driver);
}

static void __exit goodix_thp_spi_exit(void)
{
        ts_info("Goodix thp spi driver exit");
        spi_unregister_driver(&goodix_spi_driver);
        goodix_thp_core_deinit();
}

late_initcall(goodix_thp_spi_init);
module_exit(goodix_thp_spi_exit);

MODULE_DESCRIPTION("Goodix THP Driver");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
