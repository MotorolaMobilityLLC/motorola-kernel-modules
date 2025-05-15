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
 * This file is the topmost header file and also includes the main context
 * structure and the definitions for the communication of TouchComm protocol.
 */

#ifndef _SYNAPTICS_TOUCHCOM_CORE_DEV_H_
#define _SYNAPTICS_TOUCHCOM_CORE_DEV_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "synaptics_touchcom_platform.h"


#define SYNA_TCM_CORE_LIB_VERSION		0x0209
#define SYNA_TCM_CORE_LIB_CUSTOM_CODE	0x00


#if defined(_WIN32) || defined(WIN32) || defined(__WIN32__)
#define OS_WIN
#endif

#if defined(TOUCHCOMM_VERSION_1)
/* build in the relevent implementations for protocol version 1 */
#define HAS_VERSION_1_SUPPORT
#endif

#if defined(TOUCHCOMM_VERSION_2)
/* build in the relevent implementations for protocol version 1 */
#define HAS_VERSION_2_SUPPORT
#endif


#if defined(TOUCHCOMM_TDDI)
/* enable the support of TDDI product */
#define HAS_TDDI_SUPPORT
#endif

#if defined(TOUCHCOMM_SMART_BRIDGE)
/* enable the support of display product */
#define HAS_DISPLAY_SUPPORT
#endif


 /*
  * Touchcomm Protocols
  *
  *   PROTOCOL_DETECT_VERSION_1:  Specify the protocol version 1
  *   PROTOCOL_DETECT_VERSION_2:  Specify the protocol version 2
  *   PROTOCOL_BYPASS_STARTUP_PACKET:  Request to bypass the startup packet
  */
#define PROTOCOL_DETECT_VERSION_1 (0x01)
#define PROTOCOL_DETECT_VERSION_2 (0x02)

#define PROTOCOL_BYPASS_STARTUP_PACKET (0x80)


/*
 * Common parameters
 *
 *    MAX_NUM_OBJECTS       :  Maximum number of objects being detected
 *    MAX_SIZE_GESTURE_DATA :  Maximum size of gesture data
 *    MAX_SIZE_CONFIG_ID    :  Maximum size of customer configuration ID
 *    MAX_REPORT_TYPES      :  Maximum types of message
 */
#define MAX_NUM_OBJECTS (10)

#define MAX_SIZE_GESTURE_DATA (8)

#define MAX_SIZE_CONFIG_ID (16)

#define MAX_REPORT_TYPES (256)

#ifdef TOUCHCOMM_TDDI
/*
 * TDDI parameters
 *
 *    MAX_NUM_KNOB_OBJECTS  :  Maximum number of knob objects
 */

#define MAX_NUM_KNOB_OBJECTS (2)
#endif

/*
 * Common definitions for Touchcomm message
 *
 *    MESSAGE_HEADER_SIZE: The size of message header
 *    TCM_MSG_CRC_LENGTH : Length of message CRC data
 *    TCM_EXTRA_RC_LENGTH: Length of extra rolling counter
 */
#define MESSAGE_HEADER_SIZE (4)

#define TCM_MSG_CRC_LENGTH  (2)
#define TCM_EXTRA_RC_LENGTH (1)

/*
 * Definitions for command processing
 *
 *    CMD_RESPONSE_DEFAULT_POLLING_DELAY_MS : Time period to poll the response
 *    CMD_RESPONSE_IN_ATTN                  : Process command by ATTN-driven
 *    CMD_RESPONSE_IN_POLLING               : Process command by polling
 *    COMMAND_RETRY_TIMES                   : Maximum time to retry the command
 */

#define CMD_RESPONSE_DEFAULT_POLLING_DELAY_MS (20)

#define CMD_RESPONSE_IN_ATTN (0)
#define CMD_RESPONSE_IN_POLLING (CMD_RESPONSE_DEFAULT_POLLING_DELAY_MS)

#define COMMAND_RETRY_TIMES (5)

/*
 * Default timings regarding to reflash
 *
 *    DEFAULT_FLASH_ERASE_DELAY_US : Time required for a flash erase per 'page'
 *    DEFAULT_FLASH_WRITE_DELAY_US : Time required for a flash write per 'block'
 *    DEFAULT_FLASH_READ_DELAY_US  : Time required for a flash read per 'word'
 */

#define DEFAULT_FLASH_ERASE_DELAY_US (20000)
#define DEFAULT_FLASH_WRITE_DELAY_US (20)
#define DEFAULT_FLASH_READ_DELAY_US  (10)


/* Macro to show string in log */
#define STR(x) #x

/* Helpers to check the device mode */
#define IS_APP_FW_MODE(mode) \
	(mode == MODE_APPLICATION_FIRMWARE)

#define IS_NOT_APP_FW_MODE(mode) \
	(!IS_APP_FW_MODE(mode))

#ifdef TOUCHCOMM_TDDI
#define IS_BOOTLOADER_MODE(mode) \
	((mode == MODE_BOOTLOADER) || \
	(mode == MODE_TDDI_BOOTLOADER)  || \
	(mode == MODE_TDDI_HDL_BOOTLOADER) || \
	(mode == MODE_MULTICHIP_TDDI_BOOTLOADER))
#else
#define IS_BOOTLOADER_MODE(mode) \
	((mode == MODE_BOOTLOADER))
#endif

#ifdef TOUCHCOMM_TDDI
#define IS_TDDI_BOOTLOADER_MODE(mode) \
	((mode == MODE_TDDI_BOOTLOADER)  || \
	(mode == MODE_TDDI_HDL_BOOTLOADER) || \
	(mode == MODE_MULTICHIP_TDDI_BOOTLOADER))

#define IS_ROM_BOOTLOADER_MODE(mode) \
	(mode == MODE_ROMBOOTLOADER)
#endif

#ifdef TOUCHCOMM_SMART_BRIDGE
#define IS_DISPLAY_ROM_BOOTLOADER_MODE(mode) \
	(mode == MODE_DISPLAY_ROMBOOTLOADER)
#endif

#define IS_A_REPORT(code) \
	((code >= 0x10) && (code != 0xFF))

#define IS_A_RESPONSE(code) \
	((code > 0) && (code < 0x10))


/* Definitions of error codes */
enum error_codes {
	ERR_MASK = 0xf0,
	ERR_INVAL = 0xf1,      /* invalid parameters */
	ERR_TCMMSG = 0xf2,     /* touchcomm message errors */
	ERR_NOMEM = 0xf3,      /* out of memory */
	ERR_TIMEDOUT = 0xf4,   /* execution timeout */
	ERR_NODEV = 0xf5,      /* no touchcomm device */
	ERR_BUSY = 0xf6,       /* device is busy */
};

/* Version of TouchComm Firmware */
enum tcm_firmware_protocol {
	TOUCHCOMM_NONE = 0,
	TOUCHCOMM_V1 = 1,
	TOUCHCOMM_V2 = 2,
};

/* Definitions of TouchComm firmware modes */
enum tcm_firmware_mode {
	MODE_UNKNOWN = 0x00,
	MODE_APPLICATION_FIRMWARE = 0x01,
	MODE_HOSTDOWNLOAD_FIRMWARE = 0x02,
#ifdef TOUCHCOMM_TDDI
	MODE_ROMBOOTLOADER = 0x04,
#endif
	MODE_BOOTLOADER = 0x0b,
#ifdef TOUCHCOMM_TDDI
	MODE_TDDI_BOOTLOADER = 0x0c,
	MODE_TDDI_HDL_BOOTLOADER = 0x0d,
#endif
	MODE_PRODUCTIONTEST_FIRMWARE = 0x0e,
#ifdef TOUCHCOMM_TDDI
	MODE_MULTICHIP_TDDI_BOOTLOADER = 0xab,
#endif
#ifdef TOUCHCOMM_SMART_BRIDGE
	MODE_DISPLAY_ROMBOOTLOADER = 0x40,
	MODE_DISPLAY_APPLICATION_FIRMWARE = 0x41,
#endif

	MODE_RMI_MICRO_BOOTLOADER = 0xff,
};

/* Status of Application Firmware */
enum tcm_app_status {
	APP_STATUS_OK = 0x00,
	APP_STATUS_BOOTING = 0x01,
	APP_STATUS_UPDATING = 0x02,
	APP_STATUS_BAD_APP_CONFIG = 0xff,
};

/* Field id for dynamic config command */
enum dynamic_tcm_config_id {
	DC_UNKNOWN = 0x00,
	DC_DISABLE_DOZE = 0x01,
	DC_DISABLE_NOISE_MITIGATION = 0x02,
	DC_DISABLE_FREQUENCY_SHIFT = 0x03,
	DC_REQUEST_FREQUENCY_INDEX = 0x04,
	DC_DISABLE_HSYNC = 0x05,
	DC_REZERO_ON_EXIT_DEEP_SLEEP = 0x06,
	DC_ENABLE_CHARGER_CONNECTED = 0x07,
	DC_DISABLE_BASELINE_RELAXATION = 0x08,
	DC_ENABLE_WAKEUP_GESTURE_MODE = 0x09,
	DC_REQUEST_TESTING_FINGERS = 0x0a,
	DC_ENABLE_GRIP_SUPPRESSION = 0x0b,
	DC_ENABLE_THICK_GLOVE = 0x0c,
	DC_ENABLE_GLOVE = 0x0d,
	DC_ENABLE_FACE_DETECTION = 0x0e,
	DC_INHIBIT_ACTIVE_GESTURE = 0x0f,
	DC_DISABLE_PROXIMITY = 0x10,
	DC_CONTROL_LBP_HBP = 0x11,
};

/* Generic Touchcomm commands */
enum tcm_command {
	CMD_NONE = 0x00,
	CMD_CONTINUE_WRITE = 0x01,
	CMD_IDENTIFY = 0x02,
	CMD_RESET = 0x04,
	CMD_ENABLE_REPORT = 0x05,
	CMD_DISABLE_REPORT = 0x06,
#ifdef TOUCHCOMM_VERSION_2
	CMD_TCM2_ACK = 0x07,
	CMD_TCM2_SET_MAX_READ_LENGTH = 0x09,
	CMD_TCM2_GET_REPORT = 0x0a,
#endif
	CMD_GET_BOOT_INFO = 0x10,
	CMD_ERASE_FLASH = 0x11,
	CMD_WRITE_FLASH = 0x12,
	CMD_READ_FLASH = 0x13,
	CMD_RUN_APPLICATION_FIRMWARE = 0x14,
	CMD_SPI_MASTER_WRITE_THEN_READ = 0x15,
#ifdef TOUCHCOMM_TDDI
	CMD_REBOOT_TO_ROM_BOOTLOADER = 0x16,
#endif
	CMD_RUN_BOOTLOADER_FIRMWARE = 0x1f,
	CMD_GET_APPLICATION_INFO = 0x20,
	CMD_GET_STATIC_CONFIG = 0x21,
	CMD_SET_STATIC_CONFIG = 0x22,
	CMD_GET_DYNAMIC_CONFIG = 0x23,
	CMD_SET_DYNAMIC_CONFIG = 0x24,
	CMD_GET_TOUCH_REPORT_CONFIG = 0x25,
	CMD_SET_TOUCH_REPORT_CONFIG = 0x26,
	CMD_REZERO = 0x27,
	CMD_COMMIT_CONFIG = 0x28,
	CMD_DESCRIBE_DYNAMIC_CONFIG = 0x29,
	CMD_PRODUCTION_TEST = 0x2a,
	CMD_SET_CONFIG_ID = 0x2b,
	CMD_ENTER_DEEP_SLEEP = 0x2c,
	CMD_EXIT_DEEP_SLEEP = 0x2d,
	CMD_GET_TOUCH_INFO = 0x2e,
	CMD_GET_DATA_LOCATION = 0x2f,
	CMD_DOWNLOAD_CONFIG = 0x30,
	CMD_ENTER_PRODUCTION_TEST_MODE = 0x31,
	CMD_GET_FEATURES = 0x32,
	CMD_CALIBRATE = 0x33,
	CMD_START_APPLICATION_ACQUISITION = 0x37,
	CMD_STOP_APPLICATION_ACQUISITION = 0x38,
	CMD_SET_GLOBAL_STATIC_CONFIG = 0x39,
#ifdef TOUCHCOMM_TDDI
	CMD_GET_ROMBOOT_INFO = 0x40,
	CMD_WRITE_PROGRAM_RAM = 0x41,
	CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE = 0x42,
	CMD_SPI_MASTER_WRITE_THEN_READ_EXTENDED = 0x43,
	CMD_ENTER_IO_BRIDGE_MODE = 0x44,
	CMD_ROMBOOT_DOWNLOAD = 0x45,
#endif
#ifdef TOUCHCOMM_SMART_BRIDGE
	CMD_SMART_BRIDGE_RESET = 0x49,
	CMD_GET_DISPLAY_APP_INFO = 0x50,
	CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER = 0x51,
#endif
	CMD_OPTIMIZED_WRITE_FLASH = 0xFE,
};

/* Status codes after the command processing
 *
 *    0x00: (v1)      no commands are pending and no reports are available.
 *    0x01: (v1 & v2) the previous command succeeded.
 *    0x03: (v1 & v2) the payload continues a previous response.
 *    0x04: (v2)      command was written, but no reports were available.
 *    0x07: (v2)      the previous write was successfully received.
 *    0x08: (v2)      the previous write was corrupt. The host should resend.
 *    0x09: (v2)      the previous command failed.
 *    0x0c: (v1 & v2) write was larger than the device's receive buffer.
 *    0x0d: (v1 & v2) a command was sent before the previous command completed.
 *    0x0e: (v1 & v2) the requested command is not implemented.
 *    0x0f: (v1 & v2) generic communication error, probably incorrect payload.
 *
 * Driver-defined status
 *    0xfe: self-defined status for a corrupted packet.
 *    0xff: self-defined status for an invalid data.
 */
enum tcm_status_code {
	STATUS_IDLE = 0x00,
	STATUS_OK = 0x01,
	STATUS_CONTINUED_READ = 0x03,
#ifdef TOUCHCOMM_VERSION_2
	STATUS_NO_REPORT_AVAILABLE = 0x04,
	STATUS_ACK = 0x07,
	STATUS_RETRY_REQUESTED = 0x08,
	STATUS_CMD_FAILED = 0x09,
#endif
	STATUS_RECEIVE_BUFFER_OVERFLOW = 0x0c,
	STATUS_PREVIOUS_COMMAND_PENDING = 0x0d,
	STATUS_NOT_IMPLEMENTED = 0x0e,
	STATUS_ERROR = 0x0f,

	STATUS_INVALID = 0xff,
};

/* Generic Touchcomm reports */
enum tcm_report_type {
	REPORT_IDENTIFY = 0x10,
	REPORT_TOUCH = 0x11,
	REPORT_DELTA = 0x12,
	REPORT_RAW = 0x13,
	REPORT_BASELINE = 0x14,
};

/* List the states during the command processing  */
enum tcm_command_status {
	CMD_STATE_IDLE = 0,
	CMD_STATE_BUSY = 1,
	CMD_STATE_TERMINATED = 2,
	CMD_STATE_ERROR = -1,
};

/* Layout of the structure of internal buffer */
struct tcm_buffer {
	unsigned char *buf;
	unsigned int buf_size;
	unsigned int data_length;
	syna_pal_mutex_t buf_mutex;
	unsigned char ref_cnt;
};

/* Definitions of timing settings */
struct tcm_timings {
	/* timeout time of command processing */
	int cmd_timeout_ms;
	/* time interval to process command by polling */
	int cmd_polling_ms;
	/* bus turnaround time (0: min / 1: max) */
	int cmd_turnaround_us[2];
	/* command retry delay (0: min / 1: max) */
	int cmd_retry_us[2];
	/* timings for flash operations (0: erase / 1: write / 2: read) */
	int flash_ops_delay_us[3];
	/* time delay for firmware mode switching */
	int fw_switch_delay_ms;
	/* time delay after issuing a reset */
	int reset_delay_ms;
};

/* Definitions of TouchComm Identify Info Packet
 *
 * The identify packet provides the basic TouchComm information and indicate
 * that the device is ready to receive commands.
 *
 * The report is received whenever the device initially powers up, resets,
 * or switches fw between bootloader and application modes.
 */
struct tcm_identification_info {
	unsigned char version;
	unsigned char mode;
	unsigned char part_number[16];
	unsigned char build_id[4];
	unsigned char max_write_size[2];
	/* extension in identify packet ver.2 */
	unsigned char current_read_size[2];
	unsigned char max_read_size[2];
	unsigned char reserved[20];
};


/* Definitions of TouchComm Application Information Packet */
struct tcm_application_info {
	unsigned char version[2];
	unsigned char status[2];
	unsigned char static_config_size[2];
	unsigned char dynamic_config_size[2];
	unsigned char app_config_start_write_block[2];
	unsigned char app_config_size[2];
	unsigned char max_touch_report_config_size[2];
	unsigned char max_touch_report_payload_size[2];
	unsigned char customer_config_id[MAX_SIZE_CONFIG_ID];
	unsigned char max_x[2];
	unsigned char max_y[2];
	unsigned char max_objects[2];
	unsigned char num_of_buttons[2];
	unsigned char num_of_image_rows[2];
	unsigned char num_of_image_cols[2];
	unsigned char has_hybrid_data[2];
	unsigned char num_of_force_elecs[2];
};

/* Definitions of TouchComm boot information packet */
struct tcm_boot_info {
	unsigned char version;
	unsigned char status;
	unsigned char asic_id[2];
	unsigned char write_block_size_words;
	unsigned char erase_page_size_words[2];
	unsigned char max_write_payload_size[2];
	unsigned char last_reset_reason;
	union {
		struct {
			unsigned char supplemental_reset_code[2];
			unsigned char boot_or_otp_config_start_block[2];
			unsigned char boot_or_otp_config_size_blocks[2];
		} v1;
		struct {
			unsigned char supplemental_reset_code[2];
			unsigned char boot_config_start_block[2];
			unsigned char boot_config_size_blocks[2];
			unsigned char display_config_start_block[4];
			unsigned char display_config_length_blocks[2];
			unsigned char backup_display_config_start_block[4];
			unsigned char backup_display_config_length_blocks[2];
			unsigned char custom_otp_start_block[2];
			unsigned char custom_otp_size_blocks[2];
		} v2;
		struct {
			unsigned char reserved_byte_10;
			unsigned char reserved_byte_11;
			unsigned char supplemental_reset_code[4];
			unsigned char boot_or_otp_config_start_block[2];
			unsigned char boot_or_otp_config_size_blocks[2];
			unsigned char mtp_config_start_page;
			unsigned char mtp_config_num_pages;
		} v3;
	};
};
#ifdef TOUCHCOMM_TDDI
/* Definitions of TouchComm ROMboot information packet */
struct tcm_romboot_info {
	unsigned char version;
	unsigned char status;
	unsigned char asic_id[2];
	unsigned char write_block_size_words;
	unsigned char max_write_payload_size[2];
	unsigned char last_reset_reason;
	unsigned char pc_at_time_of_last_reset[2];
};
#endif
/* Definitions of TouchComm features description packet */
struct tcm_features_info {
	unsigned char byte[16];
};

/* Data blobs for touch reporting */
struct tcm_objects_data_blob {
	unsigned char status;
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int x_width;
	unsigned int y_width;
	unsigned int z;
	unsigned int tx_pos;
	unsigned int rx_pos;
	unsigned int custom_data[5];
};
struct tcm_gesture_data_blob {
	union {
		struct {
			unsigned char tap_x[2];
			unsigned char tap_y[2];
		};
		struct {
			unsigned char swipe_x[2];
			unsigned char swipe_y[2];
			unsigned char swipe_direction[2];
		};
		unsigned char data[MAX_SIZE_GESTURE_DATA];
	};
};
#ifdef TOUCHCOMM_TDDI
struct tcm_knob_data_blob {
	bool is_updated;
	unsigned short angle;
	unsigned short grasp;
	bool is_clicked;
	unsigned short click;
};
#endif
struct tcm_touch_data_blob {

	/* for each active objects */
	unsigned int obji;
	unsigned int num_of_active_objects;
	struct tcm_objects_data_blob object_data[MAX_NUM_OBJECTS];

	/* for gesture */
	unsigned int gesture_id;
	struct tcm_gesture_data_blob gesture_data;

	/* various data */
	unsigned int timestamp;
	unsigned int buttons_state;
	unsigned int frame_rate;
	unsigned int power_im;
	unsigned int cid_im;
	unsigned int rail_im;
	unsigned int cid_variance_im;
	unsigned int nsm_frequency;
	unsigned int nsm_state;
	unsigned int num_of_cpu_cycles;
	unsigned int fd_data;
	unsigned int force_data;
	unsigned int fingerprint_area_meet;
	unsigned int sensing_mode;
#ifdef TOUCHCOMM_TDDI
	/* for knob */
	struct tcm_knob_data_blob knob[MAX_NUM_KNOB_OBJECTS];
#endif
};

/* Callback to customize the handling of message
 *
 * Definitions of callback function
 * param
 *    [ in] code:          the code of message
 *    [ in] data:          data to handle
 *    [ in] data_size:     size of data
 *    [ in] callback_data: private data to callback function;
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
typedef int (*tcm_message_callback_t) (const unsigned char code,
	const unsigned char *data, unsigned int data_size, void *callback_data);

struct tcm_message_callback {
	void *private_data;
	tcm_message_callback_t cb;
};

/* Callback to parse custom touch data
 *
 * Definitions of callback function
 * param
 *    [ in]    code:          the code of touch entity to parse
 *    [ in]    config:        the report configuration stored
 *    [in/out] config_offset: current position in the report config;
 *                            function shall update and return this value
 *    [ in]    report:        touch report
 *    [in/out] report_offset: current position in the touch report
 *                            function shall update and return this value
 *    [ in]    report_size:   size of given touch report
 *    [ in] callback_data:    private data to callback function;
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
typedef int (*tcm_custom_touch_callback_t) (const unsigned char code,
	const unsigned char *config, unsigned int *config_offset,
	const unsigned char *report, unsigned int *report_offset,
	unsigned int report_size, void *callback_data);

struct tcm_custom_touch_entity_callback {
	void *private_data;
	tcm_custom_touch_callback_t cb;
};

struct tcm_custom_gesture_callback {
	void *private_data;
	tcm_custom_touch_callback_t cb;
};

/*
 * Structures for TouchComm message handling module
 *
 * The context structure for the processing of TouchComm message.
 */
struct tcm_message_data_blob {

	/* parameters for command processing */
	syna_pal_atomic_t command_status;
	unsigned char command;
	unsigned char status_report_code;
	unsigned char response_code;
	unsigned int payload_length;
	unsigned char seq_toggle;

	/* timings for command processing */
	unsigned int command_timeout_time;
	unsigned int command_polling_time;
	unsigned int turnaround_time[2];
	unsigned int retry_time[2];

	/* completion event for command processing */
	syna_pal_completion_t cmd_completion;

	/* internal buffers
	 *   in  : buffer storing the data being read 'in'
	 *   out : buffer storing the data being sent 'out'
	 *   temp: 'temp' buffer used for continued read operation
	 */
	struct tcm_buffer in;
	struct tcm_buffer out;
	struct tcm_buffer temp;

	/* mutex to protect the command processing */
	syna_pal_mutex_t cmd_mutex;

	/* mutex to ensure that only a read or a write is requested */
	syna_pal_mutex_t rw_mutex;

	/* flag for the enabling of predict reading
	 * predict reading aims to retrieve all data in one transfer;
	 * otherwise, separately reads the header and payload data
	 */
	bool predict_reads;
	unsigned int predict_length;

	/* variables for crc info */
	bool has_crc;
	unsigned short crc_bytes;
	bool has_extra_rc;
	unsigned char rc_byte;
};

/*
 * TouchComm core device context structure
 *
 * The device context contains parameters and internal buffers, that will
 * be passed to all other functions that expects a device handle.
 *
 * This structure can be allocated by syna_tcm_allocate_device(),
 * and be released by syna_tcm_remove_device() if no longer needed.
 */
struct tcm_dev {
	/* point to the parent device */
	void *parent;

	/* basic device information */
	unsigned char protocol;
	unsigned char dev_mode;
	unsigned int packrat_number;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	unsigned int rows;
	unsigned int cols;
	unsigned char config_id[MAX_SIZE_CONFIG_ID];
	unsigned int is_locked;

	/* capability of each read/write data transferred */
	unsigned int max_wr_size;
	unsigned int max_rd_size;

	/* hardware platform interface */
	struct tcm_hw_platform *hw;
	/* resources of irq control */
	syna_pal_mutex_t irq_en_mutex;

	/* firmware info packet */
	struct tcm_identification_info id_info;
	struct tcm_application_info app_info;
	struct tcm_boot_info boot_info;

	/* internal buffers
	 *   report: record the TouchComm report to caller
	 *   resp  : record the command response to caller
	 */
	struct tcm_buffer report_buf;
	struct tcm_buffer resp_buf;

	/* touch report configuration */
	struct tcm_buffer touch_config;
	unsigned int end_config_loop;
	unsigned int bits_config_loop;
	unsigned int bits_config_heading;
	unsigned int bits_config_tailing;

	/* time settings for the certain scenarios */
	unsigned int fw_mode_switching_time;
	unsigned int reset_delay_time;

	/* flag indicating under the processing of production testing */
	bool testing_purpose;

	/* data for Touchcomm message handling */
	syna_pal_atomic_t command_processing;
	struct tcm_message_data_blob msg_data;

	/* flag to indicate an on-going process of fw update */
	syna_pal_atomic_t firmware_flashing;

	/* flag to indicate an on-going process of touch format update */
	syna_pal_atomic_t touch_config_update;

	/* abstraction to read a TouchComm message from device
	 *
	 * param
	 *    [ in] tcm_dev:            the TouchComm device handle
	 *    [out] status_report_code: status code or report code in the packet
	 *
	 * return
	 *    0 or positive value in case of success, a negative value otherwise.
	 */
	int (*read_message)(struct tcm_dev *tcm_dev,
		unsigned char *status_report_code);

	/* abstraction to write a TouchComm message and retrieve the response
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
	int (*write_message)(struct tcm_dev *tcm_dev, unsigned char command,
		unsigned char *payload, unsigned int payload_length,
		unsigned char *resp_code, unsigned int resp_reading);

	/* abstraction to terminate the command processing
	 *
	 * param
	 *    [ in] tcm_dev: the TouchComm device handle
	 *
	 * return
	 *    void.
	 */
	void (*terminate)(struct tcm_dev *tcm_dev);

	/* abstraction to set up the maximum read/write size
	 *
	 * param
	 *    [ in] tcm_dev: the TouchComm device handle
	 *    [ in] wr_size: the max. size for each write
	 *    [ in] rd_size: the max. size for each read
	 *
	 * return
	 *    0 or positive value in case of success, a negative value otherwise.
	 */
	int (*set_max_rw_size)(struct tcm_dev *tcm_dev, unsigned int wr_size, unsigned int rd_size);

	/* abstraction to check the maximum read/write size
	 *
	 * param
	 *    [ in] tcm_dev: the TouchComm device handle
	 *
	 * return
	 *    0 or positive value in case of success, a negative value otherwise.
	 */
	int (*check_max_rw_size)(struct tcm_dev *tcm_dev);


	/* callback to handle the custom touch entity */
	struct tcm_custom_touch_entity_callback cb_custom_touch_entity_handler;
	/* callback to handle the custom gesture */
	struct tcm_custom_gesture_callback cb_custom_gesture_handler;
	/* callbacks for the handling of reports */
	struct tcm_message_callback cb_report_dispatcher[MAX_REPORT_TYPES];
	/* callback to duplicate the data to external buffer */
	struct tcm_message_callback cb_data_duplicator[MAX_REPORT_TYPES];
#ifdef HAS_PROGRESS_FEEDBACK
	/* callback to update the progress */
	void (*cb_progress_callback)(unsigned int progress, unsigned int total);
#endif
};
/* end of structure syna_tcm_dev */


/*
 * Helpers for the protocol detection
 */

#ifdef TOUCHCOMM_VERSION_1
/*
 *  Detect whether TouchComm ver.1 firmware is running.
 *  Function is implemented in synaptics_tcm2_core_v1.c.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] bypass:   flag to bypass the detection
 *    [ in] do_reset: flag to issue a reset if falling down to error
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_v1_detect(struct tcm_dev *tcm_dev, bool bypass, bool do_reset);
#endif

#ifdef TOUCHCOMM_VERSION_2
/*
 *  Detect whether TouchComm ver.2 firmware is running.
 *  Function is implemented in synaptics_tcm2_core_v2.c.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] bypass:   flag to bypass the detection
 *    [ in] do_reset: flag to issue a reset if falling down to error
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_v2_detect(struct tcm_dev *tcm_dev, bool bypass, bool do_reset);
#endif


/*
 * Helpers for buffers management
 */

/*
 * Allocate an internal buffer.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *    [ in] size: required size to be allocated
 *
 * return
 *     0 or positive value in case of success, a negative value otherwise.
 */
static inline int syna_tcm_buf_alloc(struct tcm_buffer *pbuf,
	unsigned int size)
{
	if (!pbuf) {
		LOGE("Invalid buffer structure\n");
		return -ERR_INVAL;
	}

	if (size > pbuf->buf_size) {
		if (pbuf->buf)
			syna_pal_mem_free((void *)pbuf->buf);

		pbuf->buf = (unsigned char *)syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!(pbuf->buf)) {
			LOGE("Fail to allocate memory (size = %d)\n",
				(int)(size*sizeof(unsigned char)));
			pbuf->buf_size = 0;
			pbuf->data_length = 0;
			return -ERR_NOMEM;
		}
		pbuf->buf_size = size;
	}

	syna_pal_mem_set(pbuf->buf, 0x00, pbuf->buf_size);
	pbuf->data_length = 0;

	return 0;
}
/*
 *  Extend the existed buffer if the current size is less than the requirement.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *    [ in] size: required size to be extended
 *
 * return
 *     0 or positive value in case of success, a negative value otherwise.
 */
static inline int syna_tcm_buf_realloc(struct tcm_buffer *pbuf,
	unsigned int size)
{
	int retval;
	unsigned char *temp_src;
	unsigned int temp_size = 0;

	if (!pbuf) {
		LOGE("Invalid buffer structure\n");
		return -ERR_INVAL;
	}

	if (size > pbuf->buf_size) {
		temp_src = pbuf->buf;
		temp_size = pbuf->buf_size;

		pbuf->buf = (unsigned char *)syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!(pbuf->buf)) {
			LOGE("Fail to allocate memory (size = %d)\n",
				(int)(size * sizeof(unsigned char)));
			syna_pal_mem_free((void *)temp_src);
			pbuf->buf_size = 0;
			return -ERR_NOMEM;
		}

		retval = syna_pal_mem_cpy(pbuf->buf,
				size,
				temp_src,
				temp_size,
				temp_size);
		if (retval < 0) {
			LOGE("Fail to copy data\n");
			syna_pal_mem_free((void *)temp_src);
			syna_pal_mem_free((void *)pbuf->buf);
			pbuf->buf_size = 0;
			return retval;
		}

		syna_pal_mem_free((void *)temp_src);
		pbuf->buf_size = size;
	}

	return 0;
}
/*
 *  Initialize the buffer.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *
 * return
 *     none
 */
static inline void syna_tcm_buf_init(struct tcm_buffer *pbuf)
{
	pbuf->buf_size = 0;
	pbuf->data_length = 0;
	pbuf->ref_cnt = 0;
	pbuf->buf = NULL;
	syna_pal_mutex_alloc(&pbuf->buf_mutex);
}
/*
 *  Protect the access of the buffer.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *
 * return
 *     none
 */
static inline void syna_tcm_buf_lock(struct tcm_buffer *pbuf)
{
	if (pbuf->ref_cnt != 0)
		LOGE("Buffer access out-of balance, %d\n", pbuf->ref_cnt);

	syna_pal_mutex_lock(&pbuf->buf_mutex);
	pbuf->ref_cnt++;
}
/*
 *  Open the access of the buffer.
 *
 * param
 *    [ in] pbuf: pointer to an internal buffer
 *
 * return
 *     none
 */
static inline void syna_tcm_buf_unlock(struct tcm_buffer *pbuf)
{
	if (pbuf->ref_cnt != 1)
		LOGE("Buffer access out-of balance, %d\n", pbuf->ref_cnt);

	pbuf->ref_cnt--;
	syna_pal_mutex_unlock(&pbuf->buf_mutex);
}
/*
 *  Release the buffer.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *
 * return
 *     none
 */
static inline void syna_tcm_buf_release(struct tcm_buffer *pbuf)
{
	if (pbuf->ref_cnt != 0)
		LOGE("Buffer still in used, %d references\n", pbuf->ref_cnt);

	syna_pal_mutex_free(&pbuf->buf_mutex);
	syna_pal_mem_free((void *)pbuf->buf);
	pbuf->buf_size = 0;
	pbuf->data_length = 0;
	pbuf->ref_cnt = 0;
}
/*
 *  Clear the buffer content.
 *
 * param
 *    [ in] pbuf: pointer to a buffer
 *
 * return
 *     none
 */
static inline void syna_tcm_buf_clear(struct tcm_buffer *pbuf)
{
	if (!pbuf->buf)
		return;

	if (pbuf->data_length == 0)
		return;

	syna_pal_mem_set((void *)pbuf->buf, 0x00, pbuf->buf_size);
	pbuf->data_length = 0;
}
/*
 *  Wrap up the data copying from the source buffer to the destination buffer.
 *  The size of destination buffer may be reallocated, if the size is smaller
 *  than the actual data size to copy.
 *
 * param
 *    [out] dest: pointer to a buffer
 *    [ in] src:  required size to be extended
 *
 * return
 *     0 or positive value in case of success, a negative value otherwise.
 */
static inline int syna_tcm_buf_copy(struct tcm_buffer *dest,
	struct tcm_buffer *src)
{
	int retval = 0;

	syna_tcm_buf_lock(dest);
	syna_tcm_buf_lock(src);

	if (dest->buf_size < src->data_length) {
		retval = syna_tcm_buf_alloc(dest, src->data_length + 1);
		if (retval < 0) {
			LOGE("Fail to reallocate the given buffer, size: %d\n",
				src->data_length + 1);

			syna_tcm_buf_unlock(src);
			syna_tcm_buf_unlock(dest);

			return -ERR_NOMEM;
		}
	}

	/* copy data content to the destination */
	retval = syna_pal_mem_cpy(dest->buf,
			dest->buf_size,
			src->buf,
			src->buf_size,
			src->data_length);
	if (retval < 0) {
		LOGE("Fail to copy data to caller, size: %d\n",
			src->data_length);

		syna_tcm_buf_unlock(src);
		syna_tcm_buf_unlock(dest);

		return retval;
	}

	dest->data_length = src->data_length;

	syna_tcm_buf_unlock(src);
	syna_tcm_buf_unlock(dest);

	return 0;
}


/*
 * Abstractions of hardware-specific operations
 */

 /*
  *  Abstract the operation of data reading regardless the type of bus.
  *
  * param
  *    [ in] tcm_dev:  the TouchComm device handle
  *    [out] rd_data:  buffer for storing data retrieved from device
  *    [ in] rd_len:   length of reading data in bytes
  *
  * return
  *    0 or positive value in case of success, a negative value otherwise.
  */
static inline int syna_tcm_read(struct tcm_dev *tcm_dev,
	unsigned char *rd_data, unsigned int rd_len)
{
	struct tcm_hw_platform *hw;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	hw = tcm_dev->hw;
	if (!hw) {
		LOGE("Invalid handle of hardware platform\n");
		return -ERR_INVAL;
	}

	if (!hw->ops_read_data) {
		LOGE("Invalid hardware read operation, ops_read_data is null\n");
		return -ERR_NODEV;
	}

	return hw->ops_read_data(hw, rd_data, rd_len);
}

/*
 *  Abstract the operation of data writing regardless the type of bus.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] wr_data:  data to write
 *    [ in] wr_len:   length of written data in bytes
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static inline int syna_tcm_write(struct tcm_dev *tcm_dev,
	unsigned char *wr_data, unsigned int wr_len)
{
	struct tcm_hw_platform *hw;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	hw = tcm_dev->hw;
	if (!hw) {
		LOGE("Invalid handle of hardware platform\n");
		return -ERR_INVAL;
	}

	if (!hw->ops_write_data) {
		LOGE("Invalid hardware write operation, ops_write_data is null\n");
		return -ERR_NODEV;
	}

	return hw->ops_write_data(hw, wr_data, wr_len);
}

/*
 *  Abstract the operation of interrupt control.
 *
 * param
 *    [ in] tcm_dev:  the TouchComm device handle
 *    [ in] en:       '1' for enabling, and '0' for disabling
 *
 * return
 *    0 if nothing to do, positive value in case of success, a negative value otherwise.
 */
static inline int syna_tcm_enable_irq(struct tcm_dev *tcm_dev, bool en)
{
	int retval = 0;
	struct tcm_hw_platform *hw;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	hw = tcm_dev->hw;
	if (!hw) {
		LOGE("Invalid handle of hardware platform\n");
		return -ERR_INVAL;
	}

	if (!hw->ops_enable_attn)
		return 0;

	syna_pal_mutex_lock(&tcm_dev->irq_en_mutex);
	retval = hw->ops_enable_attn(hw, en);
	syna_pal_mutex_unlock(&tcm_dev->irq_en_mutex);

	return retval;
}


/*
 * Helpers of CRC calculations
 */

#ifdef TOUCHCOMM_VERSION_2
/*
 *  Calculate the crc-6 with polynomial
 *
 * param
 *    [ in] p:    byte array for the calculation
 *    [ in] bits: number of bits
 *
 * return
 *    the crc-6 value
 */
static inline unsigned char syna_tcm_crc6(unsigned char *p,
	unsigned int bits)
{
	unsigned short r = 0x003F << 2;
	unsigned short x;
	static unsigned short crc6_table[16] = {
		0,  268,  536,  788, 1072, 1340, 1576, 1828,
		2144, 2412, 2680, 2932, 3152, 3420, 3656, 3908
	};

	for (; bits > 8; bits -= 8) {
		r ^= *p++;
		r = (r << 4) ^ crc6_table[r >> 4];
		r = (r << 4) ^ crc6_table[r >> 4];
	}

	if (bits > 0) {
		x = *p;
		while (bits--) {
			if (x & 0x80)
				r ^= 0x80;

			x <<= 1;
			r <<= 1;
			if (r & 0x100)
				r ^= (0x03 << 2);
		}
	}

	return (unsigned char)((r >> 2) & 0x3F);
}
#endif

/*
 *  Calculate the crc-16
 *
 * param
 *    [ in] p:   byte array for the calculation
 *    [ in] len: length in bytes
 *    [ in] val: the initial value given
 *
 * return
 *    the crc-16 value
 */
static inline unsigned short syna_tcm_crc16(unsigned char *p,
	unsigned int len, unsigned short val)
{
	unsigned short r = val;
	static unsigned short crc16_table[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
	};

	if (len == 0)
		return r;

	while (len--)
		r = (r << 8) ^ crc16_table[(r >> 8) ^ *p++];

	return r;
}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end of _SYNAPTICS_TOUCHCOM_CORE_DEV_H_ */
