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
 * This file implements foundational APIs for the use of TouchComm core library
 * and also all foundational APIs supported in the Synaptics TouchComm protocol.
 */

#include "synaptics_touchcom_func_base.h"


 /*
  * Default timings for command processing
  *
  *    CMD_RESPONSE_TIMEOUT_MS         : Timeout for a command processing
  *    RETRY_US                        : Time period for next try of read/write
  *    DELAY_TURNAROUND_US             : Bus turnaround time
  *    DEFAULT_FW_MODE_SWITCH_DELAY_MS : Time period for fw mode switching
  *    DEFAULT_RESET_DELAY_MS          : Default delay after reset in case of improper settings
  */

#define CMD_RESPONSE_TIMEOUT_MS (3000)

#define RETRY_US_MIN (5000)
#define RETRY_US_MAX (10000)

#define DELAY_TURNAROUND_US_MIN (50)
#define DELAY_TURNAROUND_US_MAX (100)

#define DEFAULT_FW_MODE_SWITCH_DELAY_MS (100)

#define DEFAULT_RESET_DELAY_MS (100)


/*
 *  Set up the specific timing for the command processing
 *
 * param
 *    [ in] tcm_msg: handle of message wrapper
 *    [ in] product: the required timing settings for products
 *                   alternatively, set 'NULL' and then do setup through the third argument 'setting'
 *    [ in] setting: '0' if using 'product' to update
 *                   otherwise, a positive value to change a particular setting
 *    [ in] type:    target to change
 *                       TIMINGS_ALL         - set up all timings
 *                       TIMINGS_CMD_TIMEOUT - update the command timeout time
 *                       TIMINGS_CMD_POLLING - update the response polling time
 *                       TIMINGS_TURNAROUND  - update the command turnaround time
 *                       TIMINGS_CMD_RETRY   - update the command retry periods
 *                       TIMINGS_FW_SWITCH   - update the firmware switching time
 *                       TIMINGS_RESET_DELAY - update the delay time after reset
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_config_timings(struct tcm_dev *tcm_dev, struct tcm_timings *product,
	unsigned int setting, unsigned int type)
{
	struct tcm_message_data_blob *tcm_msg = &tcm_dev->msg_data;
	unsigned int time = setting;

	if (!tcm_msg) {
		LOGE("Invalid parameter of tcm_msg\n");
		return -ERR_INVAL;
	}

	if (!product && (type == TIMINGS_ALL)) {
		LOGE("Invalid timing settings of product\n");
		return -ERR_INVAL;
	}

	if ((type & TIMINGS_TURNAROUND) == TIMINGS_TURNAROUND) {
		if (product) {
			time = product->cmd_turnaround_us[0];
			if (time != 0)
				tcm_msg->turnaround_time[0] = product->cmd_turnaround_us[0];
			time = product->cmd_turnaround_us[1];
			if (time != 0)
				tcm_msg->turnaround_time[1] = product->cmd_turnaround_us[1];

			LOGD("Set timing: Turnaround time(%d %d)\n",
				tcm_msg->turnaround_time[0], tcm_msg->turnaround_time[1]);
		}
	}
	if ((type & TIMINGS_CMD_TIMEOUT) == TIMINGS_CMD_TIMEOUT) {
		if (product && (product->cmd_timeout_ms != 0))
			time = product->cmd_timeout_ms;
		if (time != 0) {
			tcm_msg->command_timeout_time = time;
			LOGD("Set timing: Command timeout(%d)\n", tcm_msg->command_timeout_time);
		}
	}
	if ((type & TIMINGS_CMD_POLLING) == TIMINGS_CMD_POLLING) {
		if (product && (product->cmd_polling_ms != 0))
			time = product->cmd_polling_ms;
		if (time != 0) {
			tcm_msg->command_polling_time = time;
			LOGD("Set timing: Response polling(%d)\n", tcm_msg->command_polling_time);
		}
	}
	if ((type & TIMINGS_CMD_RETRY) == TIMINGS_CMD_RETRY) {
		if (product) {
			time = product->cmd_retry_us[0];
			if (time != 0)
				tcm_msg->retry_time[0] = product->cmd_retry_us[0];
			time = product->cmd_retry_us[1];
			if (time != 0)
				tcm_msg->retry_time[1] = product->cmd_retry_us[1];

			LOGD("Set timing: Command retry time(%d %d)\n",
				tcm_msg->retry_time[0], tcm_msg->retry_time[1]);
		}
	}
	if ((type & TIMINGS_FW_SWITCH) == TIMINGS_FW_SWITCH) {
		if (product && (product->fw_switch_delay_ms != 0))
			time = product->fw_switch_delay_ms;
		if (time != 0) {
			tcm_dev->fw_mode_switching_time = time;
			LOGD("Set timing: Firmware switch(%d)\n", tcm_dev->fw_mode_switching_time);
		}
	}
	if ((type & TIMINGS_RESET_DELAY) == TIMINGS_RESET_DELAY) {
		if (product && (product->reset_delay_ms != 0))
			time = product->reset_delay_ms;
		if (time != 0) {
			tcm_dev->reset_delay_time = time;
			LOGD("Set timing: Firmware reset(%d)\n", tcm_dev->reset_delay_time);
		}
	}

	return 0;
}

/*
 *  Initialize the TouchComm message wrapper.
 *  Setup internal buffers and the relevant structures for command processing.
 *
 * param
 *    [ in] tcm_msg: handle of message wrapper
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_init_message_handler(struct tcm_message_data_blob *tcm_msg)
{
	if (!tcm_msg) {
		LOGE("Invalid parameter of tcm_msg\n");
		return -ERR_INVAL;
	}

	/* initialize internal buffers */
	syna_tcm_buf_init(&tcm_msg->in);
	syna_tcm_buf_init(&tcm_msg->out);
	syna_tcm_buf_init(&tcm_msg->temp);

	/* allocate the completion event for command processing */
	if (syna_pal_completion_alloc(&tcm_msg->cmd_completion) < 0) {
		LOGE("Fail to allocate cmd completion event\n");
		return -ERR_INVAL;
	}

	/* allocate the cmd_mutex for command protection */
	if (syna_pal_mutex_alloc(&tcm_msg->cmd_mutex) < 0) {
		LOGE("Fail to allocate cmd_mutex\n");
		return -ERR_INVAL;
	}

	/* allocate the rw_mutex for rw protection */
	if (syna_pal_mutex_alloc(&tcm_msg->rw_mutex) < 0) {
		LOGE("Fail to allocate rw_mutex\n");
		return -ERR_INVAL;
	}

	/* set default state of command_status  */
	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
	tcm_msg->command = CMD_NONE;
	tcm_msg->status_report_code = STATUS_IDLE;
	tcm_msg->payload_length = 0;
	tcm_msg->seq_toggle = 0;

	/* allocate the internal buffer.in at first */
	syna_tcm_buf_lock(&tcm_msg->in);

	if (syna_tcm_buf_alloc(&tcm_msg->in, MESSAGE_HEADER_SIZE) < 0) {
		LOGE("Fail to allocate memory for buf.in (size = %d)\n",
			MESSAGE_HEADER_SIZE);
		tcm_msg->in.buf_size = 0;
		tcm_msg->in.data_length = 0;
		syna_tcm_buf_unlock(&tcm_msg->in);
		return -ERR_NOMEM;
	}
	tcm_msg->in.buf_size = MESSAGE_HEADER_SIZE;

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* initialize the features of message handling */
	tcm_msg->predict_reads = false;
	tcm_msg->predict_length = 0;
	tcm_msg->has_crc = false;
	tcm_msg->crc_bytes = 0;
	tcm_msg->has_extra_rc = false;
	tcm_msg->rc_byte = 0;

	/* initialize the timings for message handling */
	tcm_msg->command_timeout_time = CMD_RESPONSE_TIMEOUT_MS;
	tcm_msg->command_polling_time = CMD_RESPONSE_DEFAULT_POLLING_DELAY_MS;
	tcm_msg->turnaround_time[0] = DELAY_TURNAROUND_US_MIN;
	tcm_msg->turnaround_time[1] = DELAY_TURNAROUND_US_MAX;
	tcm_msg->retry_time[0] = RETRY_US_MIN;
	tcm_msg->retry_time[1] = RETRY_US_MAX;

	return 0;
}

/*
 *  Remove message wrapper interface as well as internal buffers.
 *  Call the function once the message wrapper is no longer needed.
 *
 * param
 *    [in] tcm_msg: message wrapper structure
 *
 * return
 *    void.
 */
static void syna_tcm_del_message_handler(struct tcm_message_data_blob *tcm_msg)
{
	/* release the mutex */
	syna_pal_mutex_free(&tcm_msg->rw_mutex);
	syna_pal_mutex_free(&tcm_msg->cmd_mutex);

	/* release the completion event */
	syna_pal_completion_free(&tcm_msg->cmd_completion);

	/* release internal buffers  */
	syna_tcm_buf_release(&tcm_msg->temp);
	syna_tcm_buf_release(&tcm_msg->out);
	syna_tcm_buf_release(&tcm_msg->in);
}

/*
 *  Allocate and initialize the TouchCom core device module.
 *
 *  This function must be called in order to allocate the main device handle,
 *  structure syna_tcm_dev, which will be passed to all other functions within
 *  the entire source code.
 *
 *  The hardware platform interface is must to provide.
 *
 * param
 *    [out] ptcm_dev_ptr: a pointer to the TouchComm device handle
 *    [ in] hw:           hardware platform interface
 *    [ in] parent_ptr:   data structure representing the parent device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_allocate_device(struct tcm_dev **ptcm_dev_ptr,
	struct tcm_hw_platform *hw, void *parent_ptr)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = NULL;

	if (!hw) {
		LOGE("Invalid hardware interface\n");
		return -ERR_INVAL;
	}

	if (!hw->ops_read_data) {
		LOGE("Invalid hw read operation\n");
		return -ERR_INVAL;
	}
	if (!hw->ops_write_data) {
		LOGE("Invalid hw write operation\n");
		return -ERR_INVAL;
	}

	LOGI("Prepare to allocate TouchComm core module ...\n");

	*ptcm_dev_ptr = NULL;

	/* allocate the core device handle */
	tcm_dev = (struct tcm_dev *)syna_pal_mem_alloc(
			1,
			sizeof(struct tcm_dev));
	if (!tcm_dev) {
		LOGE("Fail to create tcm device handle\n");
		return -ERR_NOMEM;
	}

	/* points to the shell implementations */
	tcm_dev->parent = parent_ptr;
	tcm_dev->hw = hw;

	tcm_dev->write_message = NULL;
	tcm_dev->read_message = NULL;
	tcm_dev->terminate = NULL;
	tcm_dev->set_max_rw_size = NULL;

	/* initialize the capability of read write
	 * these values may be updated after the start-up packet
	 */
	tcm_dev->max_rd_size = hw->rd_chunk_size;
	tcm_dev->max_wr_size = hw->wr_chunk_size;

	/* initialize the mutex */
	if (syna_pal_mutex_alloc(&tcm_dev->irq_en_mutex) < 0) {
		LOGE("Fail to allocate irq_en_mutex\n");
		goto err_init_mutex;
	}

	/* allocate internal buffers */
	syna_tcm_buf_init(&tcm_dev->report_buf);
	syna_tcm_buf_init(&tcm_dev->resp_buf);
	syna_tcm_buf_init(&tcm_dev->touch_config);

	/* initialize the command wrapper interface */
	retval = syna_tcm_init_message_handler(&tcm_dev->msg_data);
	if (retval < 0) {
		LOGE("Fail to initialize command interface\n");
		goto err_init_message_handler;
	}

	tcm_dev->fw_mode_switching_time = DEFAULT_FW_MODE_SWITCH_DELAY_MS;
	tcm_dev->reset_delay_time = DEFAULT_RESET_DELAY_MS;

	/* initialize the default running mode */
	tcm_dev->dev_mode = MODE_UNKNOWN;

	/* return the created device handle */
	*ptcm_dev_ptr = tcm_dev;

	LOGI("TouchComm core module created, ver.: %d.%02d.%02d\n",
		(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
		(unsigned char)SYNA_TCM_CORE_LIB_VERSION & 0xff,
		SYNA_TCM_CORE_LIB_CUSTOM_CODE);
	LOGI("Platform capability: support_attn(%s)\n",
		(hw->support_attn) ? "yes" : "no");
	if (hw->alignment_enabled) {
		LOGI("Platform capability: data alignment(%s), base(%d), boundary(%d)\n",
			(hw->alignment_base > 0) ? "yes" : "no", hw->alignment_base, hw->alignment_boundary);
	}

	return 0;

err_init_message_handler:
	syna_tcm_buf_release(&tcm_dev->touch_config);
	syna_tcm_buf_release(&tcm_dev->report_buf);
	syna_tcm_buf_release(&tcm_dev->resp_buf);

	syna_pal_mutex_free(&tcm_dev->irq_en_mutex);
err_init_mutex:
	tcm_dev->hw = NULL;

	syna_pal_mem_free((void *)tcm_dev);

	return retval;
}

/*
 *  Remove the TouchCom core device module.
 *  This function must be invoked when the library is no longer needed.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    void.
 */
void syna_tcm_remove_device(struct tcm_dev *tcm_dev)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	/* release the command interface */
	syna_tcm_del_message_handler(&tcm_dev->msg_data);

	/* release resources */
	syna_tcm_buf_release(&tcm_dev->touch_config);
	syna_tcm_buf_release(&tcm_dev->report_buf);
	syna_tcm_buf_release(&tcm_dev->resp_buf);

	syna_pal_mutex_free(&tcm_dev->irq_en_mutex);

	tcm_dev->parent = NULL;
	tcm_dev->hw = NULL;

	/* release the TouchComm device handle */
	syna_pal_mem_free((void *)tcm_dev);

	LOGI("TouchComm core module removed\n");
}
/*
 *  Determine the type of device being connected, and distinguish which
 *  version of TouchCom firmware running on the device.
 *
 *  This function should be called to communicate with the specific sensor device.
 *
 * param
 *    [ in] tcm_dev:          the TouchComm device handle
 *    [ in] mode:             mode for the protocol detection
 *                            [0:3] 0x1 DETECT_VERSION_1 - detect TouchComm version 1 only
 *                                  0x2 DETECT_VERSION_2 - detect TouchComm version 2 only
 *                            [4:7] 0x8 BYPASS_STARTUP   - bypass the checking of startup packet
 *
 *    [ in] reset_to_detect:  set if willing to use 'reset' command for the detection
 *
 * return
 *    the current mode running on the device in case of success, a negative value otherwise.
 */
int syna_tcm_detect_device(struct tcm_dev *tcm_dev, unsigned int mode, bool reset_to_detect)
{
	int retval = -ERR_NODEV;
	bool bypass = ((mode & PROTOCOL_BYPASS_STARTUP_PACKET) == PROTOCOL_BYPASS_STARTUP_PACKET);
	int protocol = (mode & 0xF);

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->dev_mode = MODE_UNKNOWN;
	tcm_dev->protocol = 0;

	switch (protocol) {
	case PROTOCOL_DETECT_VERSION_1:
#ifdef TOUCHCOMM_VERSION_1
		if (syna_tcm_v1_detect(tcm_dev, bypass, reset_to_detect) < 0) {
			if (tcm_dev->msg_data.in.buf_size > 0) {
				LOGE("Fail to detect TouchComm v1 device, %02x %02x %02x %02x ...\n",
					tcm_dev->msg_data.in.buf[0], tcm_dev->msg_data.in.buf[1],
					tcm_dev->msg_data.in.buf[2], tcm_dev->msg_data.in.buf[3]);
			}
			return -ERR_NODEV;
		}
#else
		LOGE("Implementations of Touchcomm v%d is not built in\n", PROTOCOL_DETECT_VERSION_1);
		return -ERR_INVAL;
#endif
		break;
	case PROTOCOL_DETECT_VERSION_2:
#ifdef TOUCHCOMM_VERSION_2
		if (syna_tcm_v2_detect(tcm_dev, bypass, reset_to_detect) < 0) {
			if (tcm_dev->msg_data.in.buf_size > 0) {
				LOGE("Fail to detect TouchComm v2 device, %02x %02x %02x %02x ...\n",
					tcm_dev->msg_data.in.buf[0], tcm_dev->msg_data.in.buf[1],
					tcm_dev->msg_data.in.buf[2], tcm_dev->msg_data.in.buf[3]);
			}
			return -ERR_NODEV;
		}
#else
		LOGE("Implementations of Touchcomm v%d is not built in\n", PROTOCOL_DETECT_VERSION_2);
		return -ERR_INVAL;
#endif
		break;
	default:
		LOGE("Invalid version of TouchComm protocol\n");
		return -ERR_INVAL;
	}

	if ((!tcm_dev->write_message) || (!tcm_dev->read_message)) {
		LOGE("Invalid TouchCom R/W operations\n");
		LOGE("Fail to allocate the handler for TouchComm device\n");
		return retval;
	}

	/* skip the mode dispatching if no needs to detect the protocol */
	if (bypass)
		return protocol;

	/* check the running mode */
	switch (tcm_dev->dev_mode) {
	case MODE_APPLICATION_FIRMWARE:
		LOGI("Device in Application FW, build id: %d, %s\n",
			tcm_dev->packrat_number,
			tcm_dev->id_info.part_number);
		break;
	case MODE_BOOTLOADER:
		LOGI("Device in Bootloader\n");
		break;
#ifdef TOUCHCOMM_TDDI
	case MODE_TDDI_BOOTLOADER:
		LOGI("Device in TDDI Bootloader\n");
		break;
	case MODE_ROMBOOTLOADER:
		LOGI("Device in ROM Bootloader\n");
		break;
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		LOGI("Device in multi-chip TDDI Bootloader\n");
		break;
#endif
#ifdef TOUCHCOMM_SMART_BRIDGE
	case MODE_DISPLAY_APPLICATION_FIRMWARE:
		LOGI("Device in Display Application FW, build id: %d, %s\n",
			tcm_dev->packrat_number,
			tcm_dev->id_info.part_number);
		break;
	case MODE_DISPLAY_ROMBOOTLOADER:
		LOGI("Device in Display ROM Bootloader\n");
		break;
#endif
	default:
		LOGW("Found TouchCom device, but unknown mode:0x%02x detected\n",
			tcm_dev->dev_mode);
		break;
	}

	return tcm_dev->dev_mode;
}

/*
 *  Helper to read out TouchComm messages when ATTN is asserted.
 *  After returning, the ATTN signal should be no longer asserted.
 *
 *  The 'code' returned will guide the caller on the next action.
 *  For example, do touch reporting if the returned code belongs to REPORT_TOUCH.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *    [out] code:    received report code
 *    [out] data:    a user buffer for data returned
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_event_data(struct tcm_dev *tcm_dev, unsigned char *code,
	struct tcm_buffer *data)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!code) {
		LOGE("Invalid parameter\n");
		return -ERR_INVAL;
	}

	/* retrieve the event data */
	retval = tcm_dev->read_message(tcm_dev, code);
	if (retval < 0) {
		LOGE("Fail to read messages\n");
		return retval;
	}

	/* exit if no buffer provided */
	if (!data)
		goto exit;

	/* if gathering a report, copy to the user buffer */
	if ((*code >= REPORT_IDENTIFY) && (*code != STATUS_INVALID)) {
		if (tcm_dev->report_buf.data_length == 0)
			goto exit;

		retval = syna_tcm_buf_copy(data, &tcm_dev->report_buf);
		if (retval < 0) {
			LOGE("Fail to copy data, report type: %x\n", *code);
			goto exit;
		}
	}

	/* if gathering a response, copy to the user buffer */
	if ((*code > STATUS_IDLE) && (*code <= STATUS_ERROR)) {
		if (tcm_dev->resp_buf.data_length == 0)
			goto exit;

		retval = syna_tcm_buf_copy(data, &tcm_dev->resp_buf);
		if (retval < 0) {
			LOGE("Fail to copy data, status code: %x\n", *code);
			goto exit;
		}
	}

exit:
	return retval;
}

/*
 *  Request an IDENTIFY report from device.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] id_info:       the identification info packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_identify(struct tcm_dev *tcm_dev,
	struct tcm_identification_info *id_info, unsigned int resp_reading)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_IDENTIFY,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_IDENTIFY);
		goto exit;
	}

	tcm_dev->dev_mode = tcm_dev->id_info.mode;

	LOGI("TCM Fw mode: 0x%02x, TCM ver.: %d\n",
		tcm_dev->id_info.mode, tcm_dev->id_info.version);

	if (id_info == NULL)
		goto exit;

	/* copy identify info to caller */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);
	retval = syna_pal_mem_cpy((unsigned char *)id_info,
			sizeof(struct tcm_identification_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			MIN(sizeof(*id_info), tcm_dev->resp_buf.data_length));
	if (retval < 0) {
		LOGE("Fail to copy identify info to caller\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

exit:
	return retval;
}

/*
 *  Perform a soft reset.
 *  At the end of a successful reset, an IDENTIFY report shall be received.
 *
 *  Caller shall be aware that the firmware will be reloaded after reset.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_reset(struct tcm_dev *tcm_dev, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char orig_mode;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	orig_mode = tcm_dev->dev_mode;

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->reset_delay_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	/* select the proper period to loop over the resp of reset */
	if (resp_reading != CMD_RESPONSE_IN_ATTN) {
		if (tcm_dev->reset_delay_time > resp_reading) {
			resp_reading = tcm_dev->reset_delay_time;
			LOGD("Apply the default settings %dms in resp polling\n", resp_reading);
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RESET,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_RESET);
		goto exit;
	}

	/* current device mode is expected to be updated
	 * because identification report will be received after reset
	 */
	tcm_dev->dev_mode = tcm_dev->id_info.mode;

	if (orig_mode != tcm_dev->dev_mode) {
		LOGI("Different device mode 0x%02X running after reset\n", tcm_dev->dev_mode);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 *  Enable or disable the requested TouchComm report.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] report_code:   the requested report code being generated
 *    [ in] en:            '1' for enabling; '0' for disabling
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_enable_report(struct tcm_dev *tcm_dev, unsigned char report_code,
	bool en, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char command;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	command = (en) ? CMD_ENABLE_REPORT : CMD_DISABLE_REPORT;

	retval = tcm_dev->write_message(tcm_dev,
			command,
			&report_code,
			1,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to %s 0x%02x report\n",
			command, (en)?"enable":"disable", report_code);
		goto exit;
	}

	LOGD("Report 0x%x %s\n", report_code, (en) ? "enabled" : "disabled");

exit:
	return retval;
}
/*
 *  Helper to acquire one report data by polling
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] report_code:   the requested report code waiting for
 *    [out] buffer:        buffer stored the report data
 *    [ in] polling_ms:    the period to attempt acquiring the report data
 *    [ in] timeout:       timeout time in ms to wait for a report data
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_wait_for_report(struct tcm_dev *tcm_dev, unsigned char report_code,
	struct tcm_buffer *buffer, unsigned int polling_ms, unsigned int timeout_ms)
{
	int retval = 0;
	unsigned char code = 0;
	unsigned int time = 0;
	bool irq_disabled = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!buffer) {
		LOGE("Invalid data buffer being used to store the report data\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	/* indicate which mode is used */
	if (tcm_dev->hw->support_attn)
		irq_disabled = (syna_tcm_enable_irq(tcm_dev, false) > 0);

	do {
		time += polling_ms;
		syna_pal_sleep_ms(polling_ms);

		retval = syna_tcm_get_event_data(tcm_dev, &code, buffer);
		if (retval < 0)
			continue;

		if ((report_code == code) && (buffer->data_length > 0))
			break;

	} while (time < timeout_ms);

	/* recovery the irq only when running in polling mode
	 * and then irq also has been disabled previously
	 */
	if (tcm_dev->hw->support_attn && irq_disabled)
		syna_tcm_enable_irq(tcm_dev, true);

	if ((time >= timeout_ms) && ((report_code != code) || (buffer->data_length == 0)))
		retval = -ERR_TIMEDOUT;

	return retval;
}
#ifdef TOUCHCOMM_SMART_BRIDGE
/*
 *  Requests to run the display rombootloader firmware.
 *  Once the completion of firmware switching, an IDENTIFY report shall be received.
 *
 * param
 *    [ in] tcm_dev:         the TouchComm device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_run_display_rom_bootloader_fw(struct tcm_dev *tcm_dev,
	unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (fw_switch_delay == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER,
			NULL,
			0,
			NULL,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER);
		goto exit;
	}

	if (!IS_DISPLAY_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter display rom bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Display ROM Bootloader (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}
#endif
#ifdef TOUCHCOMM_TDDI
/*
 *  Requests to run the rombootloader firmware.
 *  Once the completion of firmware switching, an IDENTIFY report shall be received.
 *
 * param
 *    [ in] tcm_dev:         the TouchComm device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_run_rom_bootloader_fw(struct tcm_dev *tcm_dev,
	unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (fw_switch_delay == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REBOOT_TO_ROM_BOOTLOADER,
			NULL,
			0,
			NULL,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REBOOT_TO_ROM_BOOTLOADER);
		goto exit;
	}

	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter rom bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("ROM Bootloader (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}
#endif
/*
 *  Requests to run the bootloader firmware.
 *  Once the completion of firmware switching, an IDENTIFY report shall be received.
 *
 * param
 *    [ in] tcm_dev:         the TouchComm device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_run_bootloader_fw(struct tcm_dev *tcm_dev,
	unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (fw_switch_delay == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_BOOTLOADER_FIRMWARE,
			NULL,
			0,
			NULL,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_RUN_BOOTLOADER_FIRMWARE);
		goto exit;
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Bootloader Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 *  Requests to run the application firmware.
 *  Once the completion of firmware switching, an IDENTIFY report shall be received.
 *
 * param
 *    [ in] tcm_dev:         the TouchComm device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_run_application_fw(struct tcm_dev *tcm_dev,
	unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (fw_switch_delay == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			NULL,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_RUN_APPLICATION_FIRMWARE);
		goto exit;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGW("Fail to enter application fw, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Application Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 *  Request to switch the firmware mode running on.
 *
 * param
 *    [ in] tcm_dev:         the TouchComm device handle
 *    [ in] mode:            target firmware mode
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_switch_fw_mode(struct tcm_dev *tcm_dev, unsigned char mode,
	unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (fw_switch_delay == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGN("No support of ATTN, use polling mode instead\n");
		}
	} else {
		if (fw_switch_delay < tcm_dev->fw_mode_switching_time) {
			fw_switch_delay = tcm_dev->fw_mode_switching_time;
			LOGD("Apply the default settings %dms in resp polling\n", fw_switch_delay);
		}
	}

	switch (mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = syna_tcm_run_application_fw(tcm_dev, fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to application mode\n");
			goto exit;
		}
		break;
	case MODE_BOOTLOADER:
		retval = syna_tcm_run_bootloader_fw(tcm_dev, fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to bootloader mode\n");
			goto exit;
		}
		break;
#ifdef TOUCHCOMM_TDDI
	case MODE_TDDI_BOOTLOADER:
	case MODE_TDDI_HDL_BOOTLOADER:
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		retval = syna_tcm_run_bootloader_fw(tcm_dev, fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case MODE_ROMBOOTLOADER:
		retval = syna_tcm_run_rom_bootloader_fw(tcm_dev, fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to rom bootloader mode\n");
			goto exit;
		}
		break;
#endif
#ifdef TOUCHCOMM_SMART_BRIDGE
	case MODE_DISPLAY_ROMBOOTLOADER:
		retval = syna_tcm_run_display_rom_bootloader_fw(tcm_dev, fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to display rom bootloader mode\n");
			goto exit;
		}
		break;
#endif
	default:
		LOGE("Invalid firmware mode requested\n");
		retval = -ERR_INVAL;
		goto exit;
	}
	/* update the max. read length and the max. write length */
	if (tcm_dev->check_max_rw_size)
		tcm_dev->check_max_rw_size(tcm_dev);

	retval = 0;

exit:
	return retval;
}

/*
 *  Request the bootloader information.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] boot_info:     the boot info packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_boot_info(struct tcm_dev *tcm_dev,
	struct tcm_boot_info *boot_info, unsigned int resp_reading)
{
	int retval = 0;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in bootloader mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_BOOT_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(struct tcm_boot_info), resp_data_len);

	/* save the boot_info */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);
	retval = syna_pal_mem_cpy((unsigned char *)&tcm_dev->boot_info,
			sizeof(struct tcm_boot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy boot info\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

	if (boot_info) {
		/* copy boot_info to caller */
		retval = syna_pal_mem_cpy((unsigned char *)boot_info,
				sizeof(struct tcm_boot_info),
				(unsigned char *)&tcm_dev->boot_info,
				sizeof(tcm_dev->boot_info),
				copy_size);
		if (retval < 0) {
			LOGE("Fail to copy boot info to caller\n");
			goto exit;
		}
	}


exit:
	return retval;
}

/*
 *  Request the application information.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] app_info:      the application info packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_app_info(struct tcm_dev *tcm_dev,
	struct tcm_application_info *app_info, unsigned int resp_reading)
{
	int retval = 0;
	unsigned int app_status;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_APPLICATION_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(tcm_dev->app_info), resp_data_len);

	/* save the app_info */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);
	retval = syna_pal_mem_cpy((unsigned char *)&tcm_dev->app_info,
			sizeof(struct tcm_application_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy application info\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

	/* copy app_info to caller */
	if (app_info) {
		retval = syna_pal_mem_cpy((unsigned char *)app_info,
				sizeof(struct tcm_application_info),
				(unsigned char *)&tcm_dev->app_info,
				sizeof(tcm_dev->app_info),
				copy_size);
		if (retval < 0) {
			LOGE("Fail to copy application info to caller\n");
			goto exit;
		}
	}

	app_status = syna_pal_le2_to_uint(tcm_dev->app_info.status);

	if (app_status == APP_STATUS_BAD_APP_CONFIG) {
		LOGE("Bad application firmware, status: 0x%x\n", app_status);
		retval = -ERR_TCMMSG;
		goto exit;
	} else if (app_status != APP_STATUS_OK) {
		LOGE("Incorrect application status, 0x%x\n", app_status);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	tcm_dev->max_objects = syna_pal_le2_to_uint(tcm_dev->app_info.max_objects);
	tcm_dev->max_x = syna_pal_le2_to_uint(tcm_dev->app_info.max_x);
	tcm_dev->max_y = syna_pal_le2_to_uint(tcm_dev->app_info.max_y);

	tcm_dev->cols = syna_pal_le2_to_uint(tcm_dev->app_info.num_of_image_cols);
	tcm_dev->rows = syna_pal_le2_to_uint(tcm_dev->app_info.num_of_image_rows);
	syna_pal_mem_cpy((unsigned char *)tcm_dev->config_id, MAX_SIZE_CONFIG_ID,
			tcm_dev->app_info.customer_config_id, MAX_SIZE_CONFIG_ID, MAX_SIZE_CONFIG_ID);

	LOGD("App info version: %d, status: %d\n",
		syna_pal_le2_to_uint(tcm_dev->app_info.version), app_status);
	LOGD("App info: max_objs: %d, max_x:%d, max_y: %d, trx: %dx%d\n",
		tcm_dev->max_objects, tcm_dev->max_x, tcm_dev->max_y, tcm_dev->rows, tcm_dev->cols);

exit:
	return retval;
}

/*
 *  Request the static configuration.
 *  The size of static configuration is available from the app info.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] buf:           buffer stored the static configuration
 *    [ in] buf_size:      the size of given buffer
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_static_config(struct tcm_dev *tcm_dev, unsigned char *buf,
	unsigned int buf_size, unsigned int resp_reading)
{
	int retval = 0;
	unsigned int size;
	struct tcm_application_info *app_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	app_info = &tcm_dev->app_info;

	size = syna_pal_le2_to_uint(app_info->static_config_size);

	if (size > buf_size) {
		LOGE("Invalid buffer input, given size: %d (actual: %d)\n",
			buf_size, size);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_STATIC_CONFIG,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_STATIC_CONFIG);
		goto exit;
	}

	if ((buf == NULL) || (buf_size < tcm_dev->resp_buf.data_length))
		goto exit;

	/* copy app_info to caller */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);
	retval = syna_pal_mem_cpy((unsigned char *)buf,
			buf_size,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			tcm_dev->resp_buf.data_length);
	if (retval < 0) {
		LOGE("Fail to copy static config data to caller\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

exit:
	return retval;
}

/*
 *  Update the static configuration.
 *  The size of static configuration is available from the app info.
 *
 * param
 *    [ in] tcm_dev:          the TouchComm device handle
 *    [ in] config_data:      the data of static configuration
 *    [ in] config_data_size: the size of given data
 *    [ in] resp_reading:     method to read in the response
 *                            a positive value presents the ms time delay for polling;
 *                            or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_set_static_config(struct tcm_dev *tcm_dev, unsigned char *config_data,
	unsigned int config_data_size, unsigned int resp_reading)
{
	int retval = 0;
	unsigned int size;
	struct tcm_application_info *app_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	app_info = &tcm_dev->app_info;

	size = syna_pal_le2_to_uint(app_info->static_config_size);

	if (size != config_data_size) {
		LOGE("Invalid static config size, given: %d (actual: %d)\n",
			config_data_size, size);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_STATIC_CONFIG,
			config_data,
			config_data_size,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_SET_STATIC_CONFIG);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 *  Get the value from the a single field of the dynamic configuration.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] id:            target field id
 *    [out] value:         the value returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_dynamic_config(struct tcm_dev *tcm_dev, unsigned char id,
	unsigned short *value, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char out;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	out = (unsigned char)id;

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_DYNAMIC_CONFIG,
			&out,
			sizeof(out),
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to get dynamic field 0x%x\n",
			CMD_GET_DYNAMIC_CONFIG, (unsigned char)id);
		goto exit;
	}

	/* return dynamic config data */
	if (tcm_dev->resp_buf.data_length < 2) {
		LOGE("Invalid resp data size, %d\n",
			tcm_dev->resp_buf.data_length);
		goto exit;
	}

	*value = (unsigned short)syna_pal_le2_to_uint(tcm_dev->resp_buf.buf);

	LOGD("Get %d from dynamic field 0x%x\n", *value, id);

	retval = 0;

exit:
	return retval;
}

/*
 *  Update the value to the selected field of the dynamic configuration.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] id:            target field id
 *    [ in] value:         the value to the selected field
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_set_dynamic_config(struct tcm_dev *tcm_dev, unsigned char id,
	unsigned short value, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char out[3];

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	LOGD("Set %d to dynamic field 0x%x\n", value, id);

	out[0] = (unsigned char)id;
	out[1] = (unsigned char)value;
	out[2] = (unsigned char)(value >> 8);

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_DYNAMIC_CONFIG,
			out,
			sizeof(out),
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to set %d to field 0x%x\n",
			CMD_SET_DYNAMIC_CONFIG, value, (unsigned char)id);
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/*
 *  Request to rezero the baseline estimate.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_rezero(struct tcm_dev *tcm_dev, unsigned int resp_reading)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REZERO,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REZERO);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 *  Config the device into low power sleep mode or the normal active mode.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] en:            '1' to low power deep sleep mode; '0' to active mode
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_sleep(struct tcm_dev *tcm_dev, bool en, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char command;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	command = (en) ? CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	retval = tcm_dev->write_message(tcm_dev,
			command,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%x\n", command);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 *  Query the supported features in firmware.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] info:          the features description packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_features(struct tcm_dev *tcm_dev, struct tcm_features_info *info,
	unsigned int resp_reading)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_FEATURES,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_FEATURES);
		goto exit;
	}

	if (info == NULL)
		goto exit;

	/* copy features_info to caller */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);
	retval = syna_pal_mem_cpy((unsigned char *)info,
		sizeof(struct tcm_features_info),
		tcm_dev->resp_buf.buf,
		tcm_dev->resp_buf.buf_size,
		MIN(sizeof(*info), tcm_dev->resp_buf.data_length));
	if (retval < 0) {
		LOGE("Fail to copy features_info to caller\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

	retval = 0;
exit:
	return retval;
}

/*
 *  Request to run a specified production test.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] test_item:     the requested testing item
 *    [out] tdata:         testing data returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_run_production_test(struct tcm_dev *tcm_dev, unsigned char test_item,
	struct tcm_buffer *tdata, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char test_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	test_code = (unsigned char)test_item;

	retval = tcm_dev->write_message(tcm_dev,
			CMD_PRODUCTION_TEST,
			&test_code,
			1,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to run production test PID%02X (command 0x%02X)\n",
			test_code, CMD_PRODUCTION_TEST);
		goto exit;
	}

	if (tdata == NULL)
		goto exit;

	/* copy testing data to caller */
	retval = syna_tcm_buf_copy(tdata, &tcm_dev->resp_buf);
	if (retval < 0) {
		LOGE("Fail to copy testing data\n");
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}
#ifdef TOUCHCOMM_SMART_BRIDGE
/*
 *  Request to reset the smart bridge product.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_reset_smart_bridge(struct tcm_dev *tcm_dev, unsigned int resp_reading)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->reset_delay_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	/* select the proper period to loop over the resp of reset */
	if (resp_reading != CMD_RESPONSE_IN_ATTN) {
		if (tcm_dev->reset_delay_time > resp_reading) {
			resp_reading = tcm_dev->reset_delay_time;
			LOGD("Apply the board settings %dms in resp polling\n", resp_reading);
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SMART_BRIDGE_RESET,
			NULL,
			0,
			NULL,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_RESET);
		goto exit;
	}

exit:
	return retval;
}
#endif
/*
 *  Helper to process the custom command.
 *
 * param
 *    [ in] tcm_dev:        the TouchComm device handle
 *    [ in] command:        TouchComm command
 *    [ in] payload:        data payload, if any
 *    [ in] payload_length: length of data payload, if any
 *    [out] resp_code:      response code returned
 *    [out] resp:           buffer to store the response data
 *    [ in] resp_reading:   method to read in the response
 *                           a positive value presents the ms time delay for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_send_command(struct tcm_dev *tcm_dev, unsigned char command,
	unsigned char *payload, unsigned int payload_length, unsigned char *code,
	struct tcm_buffer *resp, unsigned int resp_reading)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (resp_reading == CMD_RESPONSE_IN_ATTN) {
		if (!tcm_dev->hw->support_attn) {
			resp_reading = tcm_dev->msg_data.command_polling_time;
			LOGN("No support of IRQ control, use polling mode instead\n");
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			command,
			payload,
			payload_length,
			&resp_code,
			resp_reading);
	if (retval < 0)
		LOGE("Fail to run command 0x%02x\n", command);

	if (code)
		*code = resp_code;

	/* exit if no buffer provided */
	if (!resp)
		goto exit;

	/* if gathering a report, copy to the user buffer */
	if (IS_A_REPORT(resp_code) && (tcm_dev->report_buf.data_length > 0)) {
		if (syna_tcm_buf_copy(resp, &tcm_dev->report_buf) < 0) {
			LOGE("Fail to copy data, report type: %x\n", resp_code);
			retval = -ERR_NOMEM;
			goto exit;
		}
	} else if (IS_A_RESPONSE(resp_code) && (tcm_dev->resp_buf.data_length > 0)) {
		/* if gathering a response, copy to the user buffer */
		if (syna_tcm_buf_copy(resp, &tcm_dev->resp_buf) < 0) {
			LOGE("Fail to copy resp data, status code: %x\n", resp_code);
			retval = -ERR_NOMEM;
			goto exit;
		}
	}

exit:
	return retval;
}

/*
 *  Enable the feature of predict reading.
 *
 *  This feature aims to read in all data in one transaction.
 *  In contrast to the predict reading, standard reads typically require two
 *  transfers for the header and the payload data.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_enable_predict_reading(struct tcm_dev *tcm_dev, bool en)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->msg_data.predict_reads = en;
	tcm_dev->msg_data.predict_length = 0;

	LOGI("Predicted reading is %s\n", (en) ? "enabled":"disabled");

	return 0;
}

/*
 *  Register callback function to handle the particular report.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] code:     the target report code to register
 *    [ in] p_cb:     the function pointer to the callback
 *    [ in] data:     private data to pass to the callback
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_set_report_dispatcher(struct tcm_dev *tcm_dev,
		unsigned char code, tcm_message_callback_t p_cb, void *private_data)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (code < 0x10)
		LOGW("The given code 0x%X may not belongs to report\n", code);

	tcm_dev->cb_report_dispatcher[code].cb = p_cb;
	tcm_dev->cb_report_dispatcher[code].private_data = private_data;

	LOGI("Dispatcher for report 0x%02X is registered\n", code);

	return 0;
}

/*
 *  Register callback function to duplicate the data.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] code:     the target report to handle
 *    [ in] p_cb:     the function pointer to callback
 *    [ in] data:     private data to pass to the callback
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_set_data_duplicator(struct tcm_dev *tcm_dev,
		unsigned char code, tcm_message_callback_t p_cb, void *private_data)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->cb_data_duplicator[code].cb = p_cb;
	tcm_dev->cb_data_duplicator[code].private_data = private_data;

	return 0;
}

/*
 *  Clear the callback of data duplicator registered previously.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_clear_data_duplicator(struct tcm_dev *tcm_dev)
{
	int idx;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	for (idx = 0; idx < MAX_REPORT_TYPES; idx++) {
		tcm_dev->cb_data_duplicator[idx].cb = NULL;
		tcm_dev->cb_data_duplicator[idx].private_data = NULL;
	}

	return 0;
}

/*
 *  Terminate the command processing.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    none
 */
void syna_tcm_clear_command_processing(struct tcm_dev *tcm_dev)
{
	if ((!tcm_dev) || (!tcm_dev->terminate))
		return;

	tcm_dev->terminate(tcm_dev);
}

