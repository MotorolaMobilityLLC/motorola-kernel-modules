/* SPDX-License-Identifier: GPL-2.0-only */
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
 * This file declares foundational APIs for the use of TouchComm core library
 * and also all foundational APIs supported in the Synaptics TouchComm protocol.
 */

#ifndef _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_
#define _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#include "synaptics_touchcom_core_dev.h"


/* Types of configurable timing settings being used in syna_tcm_set_command_timings */
enum tcm_message_timings {
	TIMINGS_ALL = 0xFFFF,

	/* basic settings for command processing */
	TIMINGS_CMD_TIMEOUT = 0x01,      /* timing for command timeout */
	TIMINGS_CMD_POLLING = 0x02,      /* timing for the response polling */
	TIMINGS_TURNAROUND = 0x04,       /* timing for command turnaround time */
	TIMINGS_CMD_RETRY = 0x08,        /* timing for command retry */

	/* settings for the specific commands, applied only when processed by polling */
	TIMINGS_FW_SWITCH = 0x100,        /* time period when fw mode is switched */
	TIMINGS_RESET_DELAY = 0x200,      /* an additional delay after a reset */
};


/*
 * TCM Core Library Interface Definitions
 */

/*
 *  Allocate and initialize the TouchCom core device module.
 *
 *  This function must be invoked to initialize the module before calling other functions.
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
	struct tcm_hw_platform *hw, void *parent_ptr);

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
void syna_tcm_remove_device(struct tcm_dev *tcm_dev);

/*
 *  Determine the type of device being connected, and distinguish which
 *  version of TouchCom firmware running on the device.
 *
 * param
 *    [ in] tcm_dev:          the TouchComm device handle
 *    [ in] mode:             mode for the protocol detection
 *                            [0:3] x1 DETECT_VERSION_1 - detect TouchComm version 1 only
 *                                  x2 DETECT_VERSION_2 - detect TouchComm version 2 only
 *                            [4:7] x8 FORCE_ASSIGNMENT - direct assign the protocol without the detection
 *
 *    [ in] reset_to_detect:  set if willing to use 'reset' command for the detection
 *
 * return
 *    the current mode running on the device in case of success, a negative value otherwise.
 */
int syna_tcm_detect_device(struct tcm_dev *tcm_dev, unsigned int mode, bool reset_to_detect);

/*
 *  Set up the specific timing for the command processing
 *
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
	unsigned int setting, unsigned int type);


/*
 * Generic API Definitions
 */

/*
 *  Helper to read out TouchComm messages when ATTN is asserted.
 *  After returning, the ATTN signal should be no longer asserted.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *    [out] code:    received report code
 *    [out] data:    a user buffer for data returned
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_event_data(struct tcm_dev *tcm_dev,
	unsigned char *code, struct tcm_buffer *report);

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
	struct tcm_identification_info *id_info, unsigned int resp_reading);

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
int syna_tcm_reset(struct tcm_dev *tcm_dev, unsigned int resp_reading);

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
	bool en, unsigned int resp_reading);

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
int syna_tcm_switch_fw_mode(struct tcm_dev *tcm_dev,
	unsigned char mode, unsigned int fw_switch_delay);

/*
 *  Request the bootloader information.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] boot_info:     the boot info packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_boot_info(struct tcm_dev *tcm_dev,
	struct tcm_boot_info *boot_info, unsigned int resp_reading);

/*
 *  Request the application information.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [out] app_info:      the application info packet returned
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_app_info(struct tcm_dev *tcm_dev,
	struct tcm_application_info *app_info, unsigned int resp_reading);

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
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_get_static_config(struct tcm_dev *tcm_dev, unsigned char *buf,
	unsigned int buf_size, unsigned int resp_reading);

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
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_set_static_config(struct tcm_dev *tcm_dev, unsigned char *config_data,
	unsigned int config_data_size, unsigned int resp_reading);

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
	unsigned short *value, unsigned int resp_reading);

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
	unsigned short value, unsigned int resp_reading);

/*
 *  Request to rezero the baseline estimate.
 *
 * param
 *    [ in] tcm_dev:       the TouchComm device handle
 *    [ in] resp_reading:  method to read in the response
 *                          a positive value presents the ms time delay for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_rezero(struct tcm_dev *tcm_dev, unsigned int resp_reading);

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
int syna_tcm_sleep(struct tcm_dev *tcm_dev, bool en, unsigned int resp_reading);

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
	unsigned int resp_reading);

/*
 *  Request to run a specified production test.
 *  Items are listed at enum test_code (PID$).
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
	struct tcm_buffer *tdata, unsigned int resp_reading);

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
int syna_tcm_reset_smart_bridge(struct tcm_dev *tcm_dev, unsigned int resp_reading);

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
	struct tcm_buffer *resp, unsigned int delay_ms_resp);




/*
 * API Definitions for Driver Features
 */

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
int syna_tcm_enable_predict_reading(struct tcm_dev *tcm_dev, bool en);

/*
 *  Terminate the command processing.
 *
 * param
 *    [ in] tcm_dev: the TouchComm device handle
 *
 * return
 *    none
 */
void syna_tcm_clear_command_processing(struct tcm_dev *tcm_dev);


/*
 * Helpers to set up callbacks
 */

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
	unsigned char code, tcm_message_callback_t p_cb, void *private_data);

/*
 *  Set up callback function to duplicate the data.
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
int syna_tcm_set_data_duplicator(struct tcm_dev *tcm_dev,
	unsigned char code, tcm_message_callback_t p_cb, void *private_data);

/*
 *  Clear the callback of data duplicator registered previously.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_clear_data_duplicator(struct tcm_dev *tcm_dev);

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
	struct tcm_buffer *buffer, unsigned int polling_ms, unsigned int timeout_ms);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end of _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_ */
