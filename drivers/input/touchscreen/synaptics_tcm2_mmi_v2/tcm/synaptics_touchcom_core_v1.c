// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm C library
 *
 * Copyright (C) 2017-2024 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/*
 * This file implements the TouchComm version 1 command-response protocol.
 */

#include "synaptics_touchcom_core_dev.h"

#ifdef TOUCHCOMM_VERSION_1

#define TCM_V1_MESSAGE_MARKER 0xa5
#define TCM_V1_MESSAGE_PADDING 0x5a

/* Header of TouchComm v1 Message Packet */
struct tcm_v1_message_header {
	union {
		struct {
			unsigned char marker;
			unsigned char code;
			unsigned char length[2];
		};
		unsigned char data[MESSAGE_HEADER_SIZE];
	};
};

/*
 *  Terminate the command processing
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    void.
 */
static void syna_tcm_v1_terminate(struct tcm_dev *tcm_dev)
{
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_BUSY)
		return;

	LOGI("Terminate the processing of command %02X\n\n", tcm_msg->command);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_TERMINATED);
	syna_pal_completion_complete(cmd_completion);
}

/*
 *  Discard a corrupted message
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *
 * return
 *    void.
 */
static void syna_tcm_v1_discard_message(struct tcm_dev *tcm_dev)
{
	int retval;
	unsigned int rd_size;
	unsigned char *buf = NULL;
	int retry = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	rd_size = tcm_dev->max_rd_size;

	if (rd_size == 0)
		rd_size = 64;

	buf = syna_pal_mem_alloc(rd_size + 1, sizeof(unsigned char));
	if (!buf) {
		LOGE("Fail to allocate local buffer\n");
		return;
	}

	do {
		retval = syna_tcm_read(tcm_dev, buf, rd_size);
		if (retval < 0) {
			LOGE("Fail to read %d bytes to device\n", rd_size);
			goto exit;
		}

		if (buf[1] == 0x00)
			break;

		syna_pal_sleep_us(1000, 2000);
	} while (++retry < 100);

exit:
	if (buf)
		syna_pal_mem_free(buf);
}

/*
 *  Update the CRC info being appended at the end of message packet
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *
 * return
 *    void.
 */
static void syna_tcm_v1_update_crc(struct tcm_dev *tcm_dev)
{
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned int offset;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;

	if (!tcm_msg->has_crc)
		return;

	if (tcm_msg->payload_length == 0)
		return;

	syna_tcm_buf_lock(&tcm_msg->in);

	offset = MESSAGE_HEADER_SIZE + tcm_msg->payload_length;
	if (tcm_msg->in.buf_size <= offset + 1)
		return;

	/* copy crc bytes which are followed by EOM (0x5a) */
	tcm_msg->crc_bytes = (unsigned short)syna_pal_le2_to_uint(
		&tcm_msg->in.buf[offset + 1]); /* skip EOM */

	if (tcm_msg->has_extra_rc) {
		/* copy an extra rc byte which is appended after crc */
		offset += TCM_MSG_CRC_LENGTH;
		if (tcm_msg->in.buf_size >= offset + 1)
			tcm_msg->rc_byte = tcm_msg->in.buf[offset + 1];
	}

	if (tcm_msg->has_extra_rc) {
		LOGD("CRC read: 0x%04X, RC read: 0x%02X\n",
			tcm_msg->crc_bytes, tcm_msg->rc_byte);
	} else {
		LOGD("CRC read: 0x%04X\n", tcm_msg->crc_bytes);
	}

	syna_tcm_buf_unlock(&tcm_msg->in);
}

/*
 *  Set up the capability of message reading and writing.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *    [ in] wr_size: the max. size for each write
 *    [ in] rd_size: the max. size for each read
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_set_up_max_rw_size(struct tcm_dev *tcm_dev,
	unsigned int wr_size, unsigned int rd_size)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	/* apply the max write size */
	if (tcm_dev->max_wr_size != wr_size) {
		tcm_dev->max_wr_size = wr_size;
		LOGD("Set the max write size to %d bytes\n", tcm_dev->max_wr_size);
	}

	/* apply the max read size */
	if (tcm_dev->max_rd_size != rd_size) {
		tcm_dev->max_rd_size = rd_size;
		LOGD("Set the max read size to %d bytes\n", tcm_dev->max_rd_size);
	}

	return 0;
}
/*
 *  Check the max size of message reading and writing
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_check_max_rw_size(struct tcm_dev *tcm_dev)
{
	unsigned int wr_size = 0;
	unsigned int rd_size = 0;
	struct tcm_identification_info *id_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	id_info = &tcm_dev->id_info;

	if (id_info->version != 1) {
		LOGE("Invalid identify report stored\n");
		return -ERR_INVAL;
	}

	wr_size = syna_pal_le2_to_uint(id_info->max_write_size);

	if (wr_size == 0) {
		LOGE("Invalid max write size from identify report\n");
		return -ERR_INVAL;
	}

	/* check the max write size between the identify report and the platform's settings */
	if (wr_size != tcm_dev->max_wr_size) {
		if (tcm_dev->max_wr_size == 0)
			wr_size = tcm_dev->max_wr_size;
		else
			wr_size = MIN(wr_size, tcm_dev->max_wr_size);
	}

	/* no such definition of the max read size in v1, so use the platform's settings */
	rd_size = tcm_dev->max_rd_size;

	return syna_tcm_v1_set_up_max_rw_size(tcm_dev, wr_size, rd_size);
}

/*
 *  Parse the identification info packet and get the essential info
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] data:     buffer containing the identification info packet
 *    [ in] size:     size of data buffer
 *    [ in] data_len: length of actual data
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_parse_idinfo(struct tcm_dev *tcm_dev,
	unsigned char *data, unsigned int size, unsigned int data_len)
{
	int retval;
	unsigned int build_id = 0;
	struct tcm_identification_info *id_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!data) || (data_len == 0)) {
		LOGE("Invalid given data buffer\n");
		return -ERR_INVAL;
	}

	id_info = &tcm_dev->id_info;

	retval = syna_pal_mem_cpy((unsigned char *)id_info,
			sizeof(struct tcm_identification_info),
			data,
			size,
			MIN(sizeof(*id_info), data_len));
	if (retval < 0) {
		LOGE("Fail to copy identification info\n");
		return retval;
	}

	build_id = syna_pal_le4_to_uint(id_info->build_id);

	if (tcm_dev->packrat_number != build_id)
		tcm_dev->packrat_number = build_id;

	LOGD("fw mode:0x%02x, build id:%d\n", id_info->mode, build_id);

	tcm_dev->dev_mode = id_info->mode;

	return 0;
}

/*
 *  Handle the TouchCom report read in by the read_message(),
 *  and copy the data from internal buffer.in to internal buffer.report.
 *
 *  Additionally, call the corresponding callback functions of report types if registered.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    void.
 */
static void syna_tcm_v1_dispatch_report(struct tcm_dev *tcm_dev)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;
	unsigned char report_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	report_code = tcm_msg->status_report_code;

	if (tcm_msg->payload_length == 0) {
		tcm_dev->report_buf.data_length = 0;
		goto exit;
	}

	/* store the received report into the internal buffer.report */
	syna_tcm_buf_lock(&tcm_dev->report_buf);

	retval = syna_tcm_buf_alloc(&tcm_dev->report_buf,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.report\n");
		syna_tcm_buf_unlock(&tcm_dev->report_buf);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_pal_mem_cpy(tcm_dev->report_buf.buf,
			tcm_dev->report_buf.buf_size,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to copy payload to buf_report\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_tcm_buf_unlock(&tcm_dev->report_buf);
		goto exit;
	}

	tcm_dev->report_buf.data_length = tcm_msg->payload_length;

	syna_tcm_buf_unlock(&tcm_msg->in);
	syna_tcm_buf_unlock(&tcm_dev->report_buf);

	if (report_code == REPORT_IDENTIFY) {
		/* parse the identify report */
		syna_tcm_buf_lock(&tcm_msg->in);
		retval = syna_tcm_v1_parse_idinfo(tcm_dev,
				&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
				tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to parse identification data\n");
			syna_tcm_buf_unlock(&tcm_msg->in);
			return;
		}
		syna_tcm_buf_unlock(&tcm_msg->in);

		/* in case, the identify info packet is resulted from the command */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_BUSY) {
			switch (tcm_msg->command) {
			case CMD_RESET:
				LOGD("Reset by command 0x%02X\n", tcm_msg->command);
				ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
			case CMD_ENTER_PRODUCTION_TEST_MODE:
#ifdef TOUCHCOMM_TDDI
			case CMD_REBOOT_TO_ROM_BOOTLOADER:
			case CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE:
#endif
				ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			default:
				if (tcm_dev->testing_purpose) {
					ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
					syna_pal_completion_complete(cmd_completion);
				} else {
					LOGI("Unexpected 0x%02X report received\n", REPORT_IDENTIFY);
					ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
					syna_pal_completion_complete(cmd_completion);
				}
				break;
			}
		}
	}

	/* dispatch the report to the proper callbacks if registered */
	if (tcm_dev->cb_report_dispatcher[report_code].cb) {
		syna_tcm_buf_lock(&tcm_dev->report_buf);
		tcm_dev->cb_report_dispatcher[report_code].cb(
				report_code,
				tcm_dev->report_buf.buf,
				tcm_dev->report_buf.data_length,
				tcm_dev->cb_report_dispatcher[report_code].private_data);
		syna_tcm_buf_unlock(&tcm_dev->report_buf);
	}
exit:
	return;
}

/*
 *  Handle the response packet read in by the read_message(),
 *  and copy the data from internal buffer.in to internal buffer.resp.
 *
 *  Complete the completion event at the end of function.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    void.
 */
static void syna_tcm_v1_dispatch_response(struct tcm_dev *tcm_dev)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_BUSY)
		return;

	tcm_msg->response_code = tcm_msg->status_report_code;

	if (tcm_msg->payload_length == 0) {
		tcm_dev->resp_buf.data_length = 0;
		goto exit;
	}

	/* copy the received resp data into the internal buffer.resp */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);

	retval = syna_tcm_buf_alloc(&tcm_dev->resp_buf,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.resp\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_pal_mem_cpy(tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to copy payload to internal resp_buf\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	tcm_dev->resp_buf.data_length = tcm_msg->payload_length;

	syna_tcm_buf_unlock(&tcm_msg->in);

	if (tcm_msg->command == CMD_IDENTIFY) {
		retval = syna_tcm_v1_parse_idinfo(tcm_dev,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			tcm_dev->resp_buf.data_length);
		if (retval < 0) {
			LOGE("Fail to parse identify packet from resp_buf\n");
			syna_tcm_buf_unlock(&tcm_dev->resp_buf);
			goto exit;
		}
	}

	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

exit:
	switch (tcm_msg->response_code) {
	case STATUS_IDLE:
		break;
	case STATUS_OK:
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
		syna_pal_completion_complete(cmd_completion);
		break;
	case STATUS_CONTINUED_READ:
		LOGE("Out-of-sync continued read\n");
		break;
	default:
		LOGE("Incorrect Status code, 0x%02x, for command %02x\n",
			tcm_msg->response_code, tcm_msg->command);
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
		syna_pal_completion_complete(cmd_completion);
		break;
	}
}


/*
 *  Read in a TouchCom packet from device.
 *
 * param
 *    [ in] tcm_dev:    the TouchComm device handle
 *    [ in] rd_length:  number of reading bytes;
 *                      '0' means to read the message header only
 *    [out] buf:        pointer to a buffer which is stored the retrieved data
 *    [ in] buf_size:   size of the buffer pointed
 *    [ in] extra_crc:  flag to read in extra crc bytes
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_read(struct tcm_dev *tcm_dev, unsigned int rd_length,
	unsigned char *buf, unsigned int buf_size, bool extra_crc)
{
	int retval;
	unsigned int max_rd_size;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!buf) {
		LOGE("Invalid internal buffer to store data\n");
		return -ERR_INVAL;
	}


	if (rd_length == 0)
		return 0;

	if (rd_length > buf_size) {
		LOGE("Invalid read length, len: %d, buf_size: %d\n",
			rd_length, buf_size);
		return -ERR_INVAL;
	}

	max_rd_size = tcm_dev->max_rd_size;

	if ((max_rd_size != 0) && (rd_length > max_rd_size)) {
		LOGE("Invalid read length, len: %d, max_rd_size: %d\n",
			rd_length, max_rd_size);
		return -ERR_INVAL;
	}

	retval = syna_tcm_read(tcm_dev, buf, rd_length);
	if (retval < 0) {
		LOGE("Fail to read %d bytes to device\n", rd_length);
		goto exit;
	}

	/* check the message header */
	if (buf[0] != TCM_V1_MESSAGE_MARKER) {
		LOGE("Incorrect header marker, 0x%02x\n", buf[0]);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/*
 *  Assemble the TouchCom v1 packet and send the packet to device.
 *
 * param
 *    [ in] tcm_dev:     the TouchComm device handle
 *    [ in] command:     command code
 *    [ in] payload:     data payload if any
 *    [ in] payload_len: length of data payload if any
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_write(struct tcm_dev *tcm_dev, unsigned char command,
	unsigned char *payload, unsigned int payload_len)
{
	int retval = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned int total_length;
	unsigned int remaining_length;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length, wr_length;
	unsigned int iterations = 0, offset = 0;
	unsigned short crc = 0xFFFF;
	unsigned char crc16[TCM_MSG_CRC_LENGTH] = { 0 };
	unsigned char tmp;
	bool last = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	/* include the command byte and two bytes of length in the message header */
	total_length = payload_len + 3;

	/* append the crc to the packet if the feature is enabled */
	if (tcm_msg->has_crc) {
		crc = syna_tcm_crc16(&command, 1, crc);
		tmp = (unsigned char)payload_len & 0xff;
		crc = syna_tcm_crc16(&tmp, 1, crc);
		tmp = (unsigned char)(payload_len >> 8) & 0xff;
		crc = syna_tcm_crc16(&tmp, 1, crc);
		if (payload_len > 0)
			crc = syna_tcm_crc16(payload, payload_len, crc);

		LOGD("CRC appended: 0x%04X\n", crc);
		crc16[0] = (unsigned char)crc & 0xff;
		crc16[1] = (unsigned char)(crc >> 8);

		total_length += TCM_MSG_CRC_LENGTH;
	}

	if (tcm_dev->max_wr_size == 0)
		chunk_space = total_length;
	else
		chunk_space = tcm_dev->max_wr_size;

	if (tcm_dev->hw->alignment_enabled)
		chunk_space = syna_pal_int_alignment(chunk_space, tcm_dev->hw->alignment_base, false);

	chunks = syna_pal_int_division(total_length, chunk_space, true);
	chunks = chunks == 0 ? 1 : chunks;

	remaining_length = payload_len;

	syna_tcm_buf_lock(&tcm_msg->out);

	/* separate into several sub-transfers if the overall size is over
	 * than the maximum write size.
	 */
	offset = 0;
	for (iterations = 0; iterations < chunks; iterations++) {

		last = ((iterations + 1) == chunks);

		if (remaining_length > chunk_space)
			xfer_length = (iterations == 0) ? chunk_space - 3 : chunk_space - 1;
		else
			xfer_length = remaining_length;

		if (tcm_dev->hw->alignment_enabled) {
			if (last && (xfer_length > tcm_dev->hw->alignment_boundary)) {
				xfer_length = syna_pal_int_alignment(xfer_length, tcm_dev->hw->alignment_base, false) - 1;
				if (xfer_length != remaining_length) {
					chunks += 1;
					last = false;
				}
			}
		}

		retval = syna_tcm_buf_alloc(&tcm_msg->out, chunk_space);
		if (retval < 0) {
			LOGE("Fail to allocate memory for internal buf.out\n");
			goto exit;
		}

		if (iterations == 0) {
			tcm_msg->out.buf[0] = command;
			tcm_msg->out.buf[1] = (unsigned char)payload_len;
			tcm_msg->out.buf[2] = (unsigned char)(payload_len >> 8);

			if (payload_len > 0) {
				retval = syna_pal_mem_cpy(&tcm_msg->out.buf[3],
					tcm_msg->out.buf_size - 3,
					&payload[0],
					payload_len,
					xfer_length
				);
			}

			wr_length = 3 + xfer_length;
		} else {
			tcm_msg->out.buf[0] = CMD_CONTINUE_WRITE;

			retval = syna_pal_mem_cpy(&tcm_msg->out.buf[1],
				tcm_msg->out.buf_size - 1,
				&payload[offset],
				payload_len - offset,
				xfer_length
			);

			wr_length = 1 + xfer_length;
		}
		if (retval < 0) {
			LOGE("Fail to copy payload data to internal buf.out\n");
			goto exit;
		}

		/* append the crc16 value if supported */
		if ((tcm_msg->has_crc) && last) {
			retval = syna_pal_mem_cpy(&tcm_msg->out.buf[offset],
					tcm_msg->out.buf_size - offset,
					crc16,
					(unsigned int)sizeof(crc16),
					(unsigned int)sizeof(crc16)
					);
			if (retval < 0) {
				LOGE("Fail to append crc16\n");
				goto exit;
			}

			offset += TCM_MSG_CRC_LENGTH;
			wr_length += TCM_MSG_CRC_LENGTH;
		}

		offset += xfer_length;

		/* write command packet to the device */
		retval = syna_tcm_write(tcm_dev,
			tcm_msg->out.buf,
			wr_length
		);
		if (retval < 0) {
			LOGE("Fail to write %d bytes to device\n", wr_length);
			goto exit;
		}

		remaining_length -= xfer_length;

#ifndef OS_WIN
		if (!last)
			syna_pal_sleep_us(tcm_msg->turnaround_time[0], tcm_msg->turnaround_time[1]);
#endif
	}

exit:
	syna_tcm_buf_unlock(&tcm_msg->out);

	return retval;
}

/*
 *  Continuously read in the remaining data payload until the end of data.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *    [ in] length:  requested length of remaining payload
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_continued_read(struct tcm_dev *tcm_dev, unsigned int length)
{
	int retval = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned char code = STATUS_INVALID;
	unsigned int iterations = 0, offset = 0;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;
	bool last = false;
	int retry = 0, retry_limit = 5;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	if ((length == 0) || (tcm_msg->payload_length == 0))
		return 0;

	if ((length & 0xffff) == 0xffff)
		return -ERR_INVAL;

	/* continued read packet contains the header, payload, and a padding */
	total_length = MESSAGE_HEADER_SIZE + tcm_msg->payload_length + 1;
	/* length to read, remember a padding at the end */
	remaining_length = length + 1;

	/* read extra crc if supported */
	if (tcm_msg->has_crc) {
		total_length += TCM_MSG_CRC_LENGTH;
		remaining_length += TCM_MSG_CRC_LENGTH;
	}
	if (tcm_msg->has_extra_rc) {
		total_length += TCM_EXTRA_RC_LENGTH;
		remaining_length += TCM_EXTRA_RC_LENGTH;
	}
	  /* read in one more EOM when crc bytes appending */
	if (tcm_msg->has_crc || tcm_msg->has_extra_rc) {
		total_length += 1;
		remaining_length += 1;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	/* in case the current buf.in is smaller than requested size */
	retval = syna_tcm_buf_realloc(&tcm_msg->in, total_length + 1);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf_in\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		return -ERR_NOMEM;
	}

	/* available chunk space for payload =
	 *     total chunk size - (marker + status code)
	 */
	if (tcm_dev->max_rd_size == 0)
		chunk_space = total_length;
	else
		chunk_space = tcm_dev->max_rd_size;

	if (tcm_dev->hw->alignment_enabled)
		chunk_space = syna_pal_int_alignment(chunk_space, tcm_dev->hw->alignment_base, false);

	chunks = syna_pal_int_division(total_length, chunk_space, true);
	chunks = chunks == 0 ? 1 : chunks;

	offset = MESSAGE_HEADER_SIZE + (tcm_msg->payload_length - length);

	syna_tcm_buf_lock(&tcm_msg->temp);

	for (iterations = 0; iterations < chunks; iterations++) {

		last = ((iterations + 1) == chunks);

		if (remaining_length > chunk_space)
			xfer_length = chunk_space - 2;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			tcm_msg->in.buf[offset] = TCM_V1_MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		if (tcm_dev->hw->alignment_enabled) {
			if (last && (xfer_length > tcm_dev->hw->alignment_boundary)) {
				xfer_length = syna_pal_int_alignment(xfer_length, tcm_dev->hw->alignment_base, false) - 2;
				if (xfer_length != remaining_length) {
					chunks += 1;
					last = false;
				}
			}
		}

		/* allocate the internal temp buffer */
		retval = syna_tcm_buf_alloc(&tcm_msg->temp, xfer_length + 2);
		if (retval < 0) {
			LOGE("Fail to allocate memory for internal buf.temp\n");
			goto exit;
		}

		do {
#ifndef OS_WIN
			/* delay for the bus turnaround time */
			syna_pal_sleep_us(tcm_msg->turnaround_time[0], tcm_msg->turnaround_time[1]);
#endif
			/* retrieve data from the bus
			 * data should include header marker and status code
			 */
			retval = syna_tcm_v1_read(tcm_dev,
					xfer_length + 2,
					tcm_msg->temp.buf,
					tcm_msg->temp.buf_size,
					(tcm_msg->has_crc) && last);
			if (retval < 0) {
				retry++;
				LOGE("Fail to read %d bytes from device\n",
					xfer_length + 2);
				continue;
			}

			tcm_msg->temp.data_length = xfer_length + 2;

			/* check the data content */
			code = tcm_msg->temp.buf[1];

			if (code == STATUS_CONTINUED_READ)
				break;

			retry++;
		} while (retry < retry_limit);

		if (code != STATUS_CONTINUED_READ) {
			LOGE("Incorrect status code 0x%02x at chunk %d/%d\n",
					code, iterations, chunks);
			retval = -ERR_TCMMSG;
			goto exit;
		}

		/* copy data from internal buffer.temp to buffer.in */
		retval = syna_pal_mem_cpy(&tcm_msg->in.buf[offset],
				tcm_msg->in.buf_size - offset,
				&tcm_msg->temp.buf[2],
				tcm_msg->temp.buf_size - 2,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy payload\n");
			goto exit;
		}

		offset += xfer_length;
		remaining_length -= xfer_length;
	}

	tcm_msg->in.data_length = offset;
exit:
	syna_tcm_buf_unlock(&tcm_msg->temp);
	syna_tcm_buf_unlock(&tcm_msg->in);

	return retval;
}

/*
 *  The entry to read in a TouchComm message packet from device.
 *  Meanwhile, handle and dispatch the message accordingly.
 *
 * param
 *    [ in] tcm_dev:            the TouchComm device handle
 *    [out] status_report_code: status code or report code in the packet
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_read_message(struct tcm_dev *tcm_dev,
	unsigned char *status_report_code)
{
	int retval = 0;
	struct tcm_v1_message_header *header;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	unsigned int len = 0;
	bool do_predict = false;
	unsigned int tmp_len;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	rw_mutex = &tcm_msg->rw_mutex;

	/* predict reading is applied when doing report streaming only */
	if (tcm_msg->predict_reads)
		do_predict = (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_IDLE);

	if (status_report_code)
		*status_report_code = STATUS_INVALID;

	tcm_msg->status_report_code = STATUS_IDLE;

	syna_pal_mutex_lock(rw_mutex);

	syna_tcm_buf_lock(&tcm_msg->in);

	/* read in the message header */
	len = MESSAGE_HEADER_SIZE;

	/* if predict read enabled, plus the predicted length
	 * and determine the length to read
	 */
	if (do_predict) {
		if (tcm_msg->predict_length > 0) {
			len += tcm_msg->predict_length;
			if (tcm_msg->has_crc)
				len += TCM_MSG_CRC_LENGTH;
			if (tcm_msg->has_extra_rc)
				len += TCM_EXTRA_RC_LENGTH;
			/* end of message byte */
			len += 1;
		}
	}

	if ((tcm_dev->hw->alignment_enabled) && (len > tcm_dev->hw->alignment_boundary))
		len = syna_pal_int_alignment(len, tcm_dev->hw->alignment_base, false);

	/* ensure the size of in.buf, do re-allocate if needed */
	if (len > tcm_msg->in.buf_size) {
		retval = syna_tcm_buf_alloc(&tcm_msg->in, len);
		if (retval < 0) {
			LOGE("Fail to allocate memory for buf_in\n");
			syna_tcm_buf_unlock(&tcm_msg->in);

			tcm_msg->status_report_code = STATUS_INVALID;
			tcm_msg->payload_length = 0;
			goto exit;
		}
	}

	/* read in the message from device */
	retval = syna_tcm_v1_read(tcm_dev,
			len,
			tcm_msg->in.buf,
			tcm_msg->in.buf_size,
			false);
	if (retval < 0) {
		LOGE("Fail to read message %d bytes from device\n", len);
		syna_tcm_buf_unlock(&tcm_msg->in);

		tcm_msg->status_report_code = STATUS_INVALID;
		tcm_msg->payload_length = 0;
		goto exit;
	}

	/* check the message header */
	header = (struct tcm_v1_message_header *)tcm_msg->in.buf;

	tcm_msg->payload_length = syna_pal_le2_to_uint(header->length);

	if ((header->code != STATUS_IDLE) || (tcm_msg->payload_length > 0)) {
		LOGD("Status code: 0x%02x, length: %d (%02x %02x %02x %02x)\n",
			header->code, tcm_msg->payload_length,
			header->data[0], header->data[1], header->data[2],
			header->data[3]);
	}

	if (header->code != 0)
		tcm_msg->status_report_code = header->code;

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* do dispatch if this's the full message */
	if (tcm_msg->payload_length == 0)
		goto do_dispatch;

	if (header->code == STATUS_CONTINUED_READ) {
		LOGD("Unexpected continued packet received, %02x %02x %02x %02x\n",
			header->data[0], header->data[1], header->data[2], header->data[3]);
		syna_tcm_v1_discard_message(tcm_dev);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	/* calculate the remaining length for continued reads
	 *
	 * if enabling predict read or having extra bytes to read,
	 * the value of 'len' shall be larger than the message header;
	 * so removing the size of pre-read was the actual size to read.
	 *
	 * otherwise, the total length of payload shall be retrieved
	 */
	tmp_len = len - MESSAGE_HEADER_SIZE;
	if (len > MESSAGE_HEADER_SIZE)
		len = (tcm_msg->payload_length > tmp_len) ?
			(tcm_msg->payload_length - tmp_len) : 0;
	else
		len = tcm_msg->payload_length;

	/* retrieve the remaining data, if any */
	retval = syna_tcm_v1_continued_read(tcm_dev, len);
	if (retval < 0) {
		LOGE("Fail to do continued read, length: %d (%02x %02x %02x %02x)\n",
			len, header->data[0], header->data[1],
			header->data[2], header->data[3]);
		goto exit;
	}

do_dispatch:
	/* refill the header for dispatching */
	syna_tcm_buf_lock(&tcm_msg->in);

	tcm_msg->in.buf[0] = TCM_V1_MESSAGE_MARKER;
	tcm_msg->in.buf[1] = tcm_msg->status_report_code;
	tcm_msg->in.buf[2] = (unsigned char)tcm_msg->payload_length;
	tcm_msg->in.buf[3] = (unsigned char)(tcm_msg->payload_length >> 8);

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* update crc data if needed */
	syna_tcm_v1_update_crc(tcm_dev);

	/* duplicate the data to the external buffer */
	if (tcm_dev->cb_data_duplicator[tcm_msg->status_report_code].cb) {
		syna_tcm_buf_lock(&tcm_msg->in);
		tcm_dev->cb_data_duplicator[tcm_msg->status_report_code].cb(
				tcm_msg->status_report_code,
				&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->payload_length,
				tcm_dev->cb_data_duplicator[tcm_msg->status_report_code].private_data);
		syna_tcm_buf_unlock(&tcm_msg->in);
	}

	/* process the retrieved packet */
	if (tcm_msg->status_report_code >= REPORT_IDENTIFY)
		syna_tcm_v1_dispatch_report(tcm_dev);
	else
		syna_tcm_v1_dispatch_response(tcm_dev);

	/* copy the status report code to caller */
	if (status_report_code)
		*status_report_code = tcm_msg->status_report_code;

	/* update the length for the predict reading */
	if (do_predict) {
		if (tcm_dev->max_rd_size == 0)
			tcm_msg->predict_length = tcm_msg->payload_length;
		else
			tcm_msg->predict_length = MIN(tcm_msg->payload_length,
				tcm_dev->max_rd_size - MESSAGE_HEADER_SIZE - 1);

		if (tcm_msg->status_report_code < REPORT_IDENTIFY)
			tcm_msg->predict_length = 0;
	}

	retval = 0;

exit:
#ifndef OS_WIN
	syna_pal_sleep_us(tcm_msg->turnaround_time[0], tcm_msg->turnaround_time[1]);
#endif
	syna_pal_mutex_unlock(rw_mutex);

	return retval;
}
/*
 *  A block function to wait for the ATTN assertion.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] timeout:       timeout time waiting for the assertion
 * return
 *    void.
 */
static void syna_tcm_v1_wait_for_attn(struct tcm_dev *tcm_dev, int timeout)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	/* if set, invoke the custom function to wait for the ATTN assertion;
	 * otherwise, wait for the completion event.
	 */
	if (tcm_dev->hw->ops_wait_for_attn)
		tcm_dev->hw->ops_wait_for_attn(tcm_dev->hw, timeout);
	else
		syna_pal_completion_wait_for(&tcm_dev->msg_data.cmd_completion, timeout);
}
/*
 *  The entry of the command processing.
 *
 *  Send a command and payload to the device.
 *  Then, process and read in the response of the command.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] command:       TouchComm command
 *    [ in] payload:       data payload, if any
 *    [ in] payload_len:   length of data payload, if any
 *    [out] resp_code:     response code returned
 *    [ in] resp_reading:  method to read in the response
 *                         a positive value presents the ms time delay for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_v1_write_message(struct tcm_dev *tcm_dev,
	unsigned char command, unsigned char *payload,
	unsigned int payload_len, unsigned char *resp_code,
	unsigned int resp_reading)
{
	int retval = 0;
	unsigned int timeout = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *cmd_mutex = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	syna_pal_completion_t *cmd_completion = NULL;
	bool in_polling = false;
	bool irq_disabled = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_mutex = &tcm_msg->cmd_mutex;
	rw_mutex = &tcm_msg->rw_mutex;
	cmd_completion = &tcm_msg->cmd_completion;

	if (resp_code)
		*resp_code = STATUS_INVALID;

	/* indicate which mode is used */
	in_polling = (resp_reading != CMD_RESPONSE_IN_ATTN);

	syna_pal_mutex_lock(cmd_mutex);

	syna_pal_mutex_lock(rw_mutex);

	ATOMIC_SET(tcm_dev->command_processing, 1);
	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_BUSY);

	/* reset the command completion */
	syna_pal_completion_reset(cmd_completion);

	tcm_msg->command = command;

	LOGD("Command: 0x%02x, payload len: %d  %s\n",
		command, payload_len, (in_polling) ? "(by polling)" : "");

	/* disable irq in case of polling mode */
	if (in_polling)
		irq_disabled = (syna_tcm_enable_irq(tcm_dev, false) > 0);

	retval = syna_tcm_v1_write(tcm_dev,
			command,
			payload,
			payload_len);
	if (retval < 0) {
		syna_pal_mutex_unlock(rw_mutex);
		goto exit;
	}

	syna_pal_mutex_unlock(rw_mutex);

	/* process the command response either by polling or by ATTN */
	timeout = 0;
	do {
		if (in_polling) {
			timeout += resp_reading;
			syna_pal_sleep_ms(resp_reading);
		} else {
			timeout += tcm_msg->command_timeout_time >> 2;
			syna_tcm_v1_wait_for_attn(tcm_dev, tcm_msg->command_timeout_time);
		}

		/* stop the processing if terminated */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_TERMINATED) {
			retval = 0;
			goto exit;
		}

		/* whatever the way of processing, attempt to read in a message if not completed */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_BUSY) {
			retval = syna_tcm_v1_read_message(tcm_dev, NULL);
			if (retval < 0)
				continue;
		}

		/* break the loop if the valid response was retrieved */
		if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_BUSY)
			break;

	} while (timeout < tcm_msg->command_timeout_time);


	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_IDLE) {
		if (timeout >= tcm_msg->command_timeout_time) {
			LOGE("Timed out wait for response of command 0x%02x (%dms)\n",
				command, tcm_msg->command_timeout_time);
			retval = -ERR_TIMEDOUT;
		} else {
			LOGE("Fail to get valid response 0x%02x of command 0x%02x\n",
				tcm_msg->status_report_code, command);
			retval = -ERR_TCMMSG;
		}
		goto exit;
	}

	retval = 0;

exit:
	/* copy response code to the caller */
	if (resp_code)
		*resp_code = tcm_msg->response_code;

	tcm_msg->command = CMD_NONE;

	/* recovery the irq only when running in polling mode
	 * and then irq also has been disabled previously
	 */
	if (in_polling && irq_disabled)
		syna_tcm_enable_irq(tcm_dev, true);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
	ATOMIC_SET(tcm_dev->command_processing, 0);

	syna_pal_mutex_unlock(cmd_mutex);

	return retval;
}
/*
 *  Process the startup packet of TouchComm V1 firmware
 *
 *  For TouchComm v1 protocol, the packet must start with a specific maker code.
 *  If so, read in the remaining packet and assign the associated operations.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] bypass:   flag to bypass the detection
 *    [ in] do_reset: flag to issue a reset if falling down to error
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_v1_detect(struct tcm_dev *tcm_dev, bool bypass, bool do_reset)
{
	int retval;
	unsigned char *data;
	unsigned int data_size;
	unsigned int rd_size;
	struct tcm_v1_message_header *header;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned short default_crc = 0x5a5a;
	unsigned short default_rc = 0x5a;
	unsigned char resp_code;
	unsigned char command;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (bypass)
		goto set_ops;

	tcm_msg = &tcm_dev->msg_data;

	data_size = (unsigned int)sizeof(struct tcm_identification_info);
	rd_size = data_size + MESSAGE_HEADER_SIZE + TCM_MSG_CRC_LENGTH;

	syna_pal_mutex_lock(&tcm_msg->rw_mutex);
	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_tcm_buf_alloc(&tcm_msg->in, rd_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for buf_in\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_pal_mutex_unlock(&tcm_msg->rw_mutex);
		return retval;
	}

	data = tcm_msg->in.buf;
	retval = syna_tcm_read(tcm_dev, data, rd_size);
	if (retval < 0) {
		LOGE("Fail to retrieve start-up data from bus\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_pal_mutex_unlock(&tcm_msg->rw_mutex);
		return retval;
	}

	syna_tcm_buf_unlock(&tcm_msg->in);
	syna_pal_mutex_unlock(&tcm_msg->rw_mutex);

	LOGD("start-up data %02x %02x %02x %02x ... (read %d bytes)\n",
		data[0], data[1], data[2], data[3], rd_size);

	header = (struct tcm_v1_message_header *)data;

	if (header->marker != TCM_V1_MESSAGE_MARKER)
		return -ERR_NODEV;

	/* determine to have crc support */
	syna_tcm_v1_update_crc(tcm_dev);
	/* if all crc bytes belong to EOM, crc feature is yet enabled */
	if (tcm_dev->msg_data.crc_bytes == default_crc)
		tcm_msg->has_crc = false;
	/* if rc byte belongs to EOM, extra rc feature is yet enabled */
	if (tcm_dev->msg_data.rc_byte == default_rc)
		tcm_msg->has_extra_rc = false;


	/* the identify report should be the first packet at startup
	 * otherwise, send the command to identify
	 */
	if (header->code != REPORT_IDENTIFY) {
		command = (do_reset) ? CMD_RESET : CMD_IDENTIFY;
		retval = syna_tcm_v1_write_message(tcm_dev, command,
				NULL, 0, &resp_code, tcm_dev->reset_delay_time);
		if (retval < 0) {
			LOGE("Fail to identify at startup\n");
			return -ERR_TCMMSG;
		}
	}

	/* parse the identify info packet if needed */
	if (tcm_dev->dev_mode == MODE_UNKNOWN) {
		syna_tcm_buf_lock(&tcm_msg->in);
		retval = syna_tcm_v1_parse_idinfo(tcm_dev,
				(unsigned char *)&data[MESSAGE_HEADER_SIZE],
				data_size + MESSAGE_HEADER_SIZE, data_size);
		syna_tcm_buf_unlock(&tcm_msg->in);
		if (retval < 0) {
			LOGE("Fail to parse identify report at startup\n");
			return -ERR_TCMMSG;
		}
	}

	/* set up the max. reading length at startup */
	retval = syna_tcm_v1_check_max_rw_size(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to setup the max length to read/write\n");
		return -ERR_TCMMSG;
	}

	LOGI("TouchCom v1 detected\n");
	if ((tcm_msg->has_crc) || (tcm_msg->has_extra_rc))
		LOGI("Support of message CRC(%s) and extra RC(%s)\n",
				(tcm_msg->has_crc) ? "yes" : "no", (tcm_msg->has_extra_rc) ? "yes" : "no");

set_ops:
	tcm_dev->read_message = syna_tcm_v1_read_message;
	tcm_dev->write_message = syna_tcm_v1_write_message;
	tcm_dev->set_max_rw_size = syna_tcm_v1_set_up_max_rw_size;
	tcm_dev->check_max_rw_size = syna_tcm_v1_check_max_rw_size;
	tcm_dev->terminate = syna_tcm_v1_terminate;

	tcm_dev->msg_data.predict_length = 0;
	tcm_dev->protocol = TOUCHCOMM_V1;

	return 0;
}


#endif /* end of TOUCHCOMM_VERSION_1 */
