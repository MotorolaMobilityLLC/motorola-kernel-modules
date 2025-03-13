/*
 * omnivision TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 omnivision Incorporated. All rights reserved.
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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include "omnivision_tcm_core.h"

#define TYPE_B_PROTOCOL

//#define REPORT_Z_MAJOR_VALUE
#define MAX_Z_VALUE 1000
#define MAX_MAJOR_VALUE 255

//#define USE_DEFAULT_TOUCH_REPORT_CONFIG

#define TOUCH_REPORT_CONFIG_SIZE 128

#define SUPPORT_FACE_DETECT 0

#define SUPPORT_KNUCKLE_DATA_REPORT 0

enum touch_status {
	LIFT = 0,
	FINGER = 1,
	GLOVED_FINGER = 2,
	NOP = -1,
};

enum gesture_id {
	NO_GESTURE_DETECTED = 0,
	GESTURE_DOUBLE_TAP = 0XF4,
	GESTURE_SINGLE_TAP = 0xF1, //0X1E,
};

enum touch_report_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,
	TOUCH_GESTURE_ID,
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,
	TOUCH_NUM_OF_ACTIVE_OBJECTS,
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
	TOUCH_FACE_DETECT,
	TOUCH_GESTURE_DATA,
	TOUCH_OBJECT_N_FORCE,
	TOUCH_FINGERPRINT_AREA_MEET,
	TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	TOUCH_TUNING_SMALL_OBJECT_PARAMS,
	TOUCH_TUNING_0D_BUTTONS_VARIANCE,
#if SUPPORT_KNUCKLE_DATA_REPORT
	TOUCH_KNUCKLE_DATA = 0xca,
#endif
  TOUCH_REPORT_PALM_DETECTED = 200,
};

struct object_data {
	unsigned char status;
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int x_width;
	unsigned int y_width;
	unsigned int z;
	unsigned int tx_pos;
	unsigned int rx_pos;
};

struct input_params {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
};

#if SUPPORT_KNUCKLE_DATA_REPORT
#define KNUCKLE_DATA_SIZE 102
#endif
struct touch_data {
	struct object_data *object_data;
	unsigned int timestamp;
	unsigned int buttons_state;
	unsigned int gesture_id;
	unsigned int frame_rate;
	unsigned int power_im;
	unsigned int cid_im;
	unsigned int rail_im;
	unsigned int cid_variance_im;
	unsigned int nsm_frequency;
	unsigned int nsm_state;
	unsigned int num_of_active_objects;
	unsigned int num_of_cpu_cycles;
	unsigned int fd_data;
	unsigned int force_data;
	unsigned int fingerprint_area_meet;
#if SUPPORT_KNUCKLE_DATA_REPORT
	unsigned char knuckle_data[KNUCKLE_DATA_SIZE];
#endif
	unsigned int palm_status;
};

struct touch_hcd {
	bool irq_wake;
	bool init_touch_ok;
	bool suspend_touch;
	bool suspend_touch_finger;
	unsigned char *prev_status;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	struct mutex report_mutex;
	struct input_dev *input_dev;
	struct touch_data touch_data;
	struct input_params input_params;
	struct ovt_tcm_buffer out;
	struct ovt_tcm_buffer resp;
	struct ovt_tcm_hcd *tcm_hcd;
};

#ifdef OVT_TAP_SENSOR_EN
#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock gesture_wakelock;
#else
static struct wakeup_source *gesture_wakelock;
#endif
static struct sensors_classdev __maybe_unused sensors_touch_cdev = {
    .name = "dt-gesture",
    .vendor = "omnivision",
    .version = 1,
    .type = SENSOR_TYPE_MOTO_DOUBLE_TAP,
    .max_range = "5.0",
    .resolution = "5.0",
    .sensor_power = "1",
    .min_delay = 0,
    .max_delay = 0,
    /* WAKE_UP & SPECIAL_REPORT */
    .flags = 1 | 6,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 200,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
};
#endif

#ifdef CONFIG_TP_LAST_TIME
static bool time_flag = 1;
#endif

static struct touch_hcd *touch_hcd;
#if SUPPORT_FACE_DETECT
static int ovt_check_face_state(int current_face_state);
#endif
/**
 * touch_free_objects() - Free all touch objects
 *
 * Report finger lift events to the input subsystem for all touch objects.
 */
static void touch_free_objects(void)
{
#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif

	if (touch_hcd->input_dev == NULL)
		return;

	mutex_lock(&touch_hcd->report_mutex);

#ifdef TYPE_B_PROTOCOL
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		input_mt_slot(touch_hcd->input_dev, idx);
#ifdef REPORT_Z_MAJOR_VALUE
		input_report_abs(touch_hcd->input_dev, ABS_MT_PRESSURE, 0);
		input_report_abs(touch_hcd->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#endif
		input_mt_report_slot_state(touch_hcd->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(touch_hcd->input_dev,
			BTN_TOUCH, 0);
	input_report_key(touch_hcd->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(touch_hcd->input_dev);
#endif
	input_sync(touch_hcd->input_dev);

	mutex_unlock(&touch_hcd->report_mutex);

	return;
}

/**
 * touch_get_report_data() - Retrieve data from touch report
 *
 * Retrieve data from the touch report based on the bit offset and bit length
 * information from the touch report configuration.
 */
static int touch_get_report_data(unsigned int offset,
		unsigned int bits, unsigned int *data)
{
	unsigned char mask;
	unsigned char byte_data;
	unsigned int output_data;
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int data_bits;
	unsigned int available_bits;
	unsigned int remaining_bits;
	unsigned char *touch_report;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (bits == 0 || bits > 32) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid number of bits\n");
		return -EINVAL;
	}

	if (offset + bits > tcm_hcd->report.buffer.data_length * 8) {
		*data = 0;
		return 0;
	}

	touch_report = tcm_hcd->report.buffer.buf;

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}

/**
 * touch_parse_report() - Parse touch report
 *
 * Traverse through the touch report configuration and parse the touch report
 * generated by the device accordingly to retrieve the touch data.
 */
static int touch_parse_report(void)
{
	int retval;
	bool active_only;
	bool num_of_active_objects;
	unsigned char code;
	unsigned int size;
	unsigned int idx;
	unsigned int obj;
	unsigned int next;
	unsigned int data;
	unsigned int bits;
	unsigned int offset;
	unsigned int objects;
	unsigned int active_objects;
	unsigned int report_size;
	unsigned int config_size;
	unsigned char *config_data;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	static unsigned int end_of_foreach;

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

	config_data = tcm_hcd->config.buf;
	config_size = tcm_hcd->config.data_length;

	report_size = tcm_hcd->report.buffer.data_length;

	size = sizeof(*object_data) * touch_hcd->max_objects;
	memset(touch_hcd->touch_data.object_data, 0x00, size);

	num_of_active_objects = false;

	idx = 0;
	offset = 0;
	objects = 0;
	active_objects = 0;
	active_only = false;

	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_END:
			goto exit;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < active_objects)
						idx = next;
				} else if (offset < report_size * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < touch_hcd->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get timestamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &obj);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object index\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object classification\n");
				return retval;
			}
			object_data[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_FORCE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object force\n");
				return retval;
			}
			touch_data->force_data = data;
			offset += bits;
			break;
		case TOUCH_REPORT_PALM_DETECTED:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get palm status\n");
				return retval;
			}
			touch_data->palm_status = data;
			offset += bits;
			break;
		case TOUCH_FINGERPRINT_AREA_MEET:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object force\n");
				return retval;
			}
			touch_data->fingerprint_area_meet = data;
			LOGN(tcm_hcd->pdev->dev.parent,
					"fingerprint_area_meet = %x\n",
					touch_data->fingerprint_area_meet);
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_ID:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->gesture_id = data;
			offset += bits;
			if (touch_data->gesture_id || dbg_level_en)
				OVT_INFO("gesture_id = 0x%x\n", touch_data->gesture_id);
			else
				OVT_DEBUG("gesture_id = %x\n", touch_data->gesture_id);
			break;
		case TOUCH_FRAME_RATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_DATA:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0) {
				if (0 == end_of_foreach) {
					LOGE(tcm_hcd->pdev->dev.parent,
						"Invalid report, num_active and end_foreach are 0\n");
					return 0;
				}
				idx = end_of_foreach;
			}
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get num CPU cycles used since last frame\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_FACE_DETECT:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to detect face\n");
				return retval;
			}
			touch_data->fd_data = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[idx++];
			offset += bits;
			break;
#if SUPPORT_KNUCKLE_DATA_REPORT
		case TOUCH_KNUCKLE_DATA:
			bits = config_data[idx++];
			bits = bits | (config_data[idx++] << 8);
			retval = secure_memcpy(&touch_data->knuckle_data[0], KNUCKLE_DATA_SIZE, &tcm_hcd->report.buffer.buf[offset/8], bits / 8, bits / 8);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to copy knuckle data\n");
				return retval;
			}
			offset += bits;
			break;
#endif
		default:
			bits = config_data[idx++];
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

/**
 * touch_report() - Report touch events
 *
 * Retrieve data from the touch report generated by the device and report touch
 * events to the input subsystem.
 */
static void touch_report(void)
{
	int retval;
	unsigned int idx;
	unsigned int x;
	unsigned int y;
#ifdef REPORT_Z_MAJOR_VALUE
	unsigned int z;
	unsigned int major;
#endif
	unsigned int temp;
	unsigned int status;
	unsigned int touch_count;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!touch_hcd->init_touch_ok)
		return;

	if (touch_hcd->input_dev == NULL)
		return;

	if (touch_hcd->suspend_touch)
		return;

	mutex_lock(&touch_hcd->report_mutex);

	retval = touch_parse_report();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to parse touch report\n");
		goto exit;
	}

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

#if SUPPORT_FACE_DETECT
	ovt_check_face_state(touch_data->fd_data);
	touch_data->fd_data = FACE_STATUS_NONE;
#endif

#if WAKEUP_GESTURE
	if (tcm_hcd->in_suspend &&
			 tcm_hcd->wakeup_gesture_enabled) {
		int keycode;

#ifdef OVT_DOUBLE_TAP_CTRL
		if (touch_data->gesture_id == GESTURE_DOUBLE_TAP) {
			keycode = KEY_F4;
			OVT_INFO("double tap report KEY_F4\n");
		}
		else {
			keycode = KEY_F1;
			OVT_INFO("single tap report KEY_F1\n");
		}

		input_report_key(tcm_hcd->sensor_pdata->input_sensor_dev, keycode, 1);
		input_sync(tcm_hcd->sensor_pdata->input_sensor_dev);
		input_report_key(tcm_hcd->sensor_pdata->input_sensor_dev, keycode, 0);
		input_sync(tcm_hcd->sensor_pdata->input_sensor_dev);
		PM_WAKEUP_EVENT(gesture_wakelock, 5000);
#else
		if (touch_data->gesture_id == GESTURE_DOUBLE_TAP)
			keycode = KEY_POWER;
		else
			keycode = KEY_U;

		input_report_key(touch_hcd->input_dev, keycode, 1);
		input_sync(touch_hcd->input_dev);
		input_report_key(touch_hcd->input_dev, keycode, 0);
		input_sync(touch_hcd->input_dev);
#endif
	}
#endif

	if ((tcm_hcd->in_suspend) || (touch_hcd->suspend_touch_finger))
		goto exit;

	touch_count = 0;

	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		if (touch_hcd->prev_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 0);
			if (tcm_hcd->log_level > 2) OVT_INFO("LIFT, touch UP");
#endif
			break;
		case FINGER:
		case GLOVED_FINGER:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
#ifdef REPORT_Z_MAJOR_VALUE
			z = object_data[idx].z;
			major = (object_data[idx].x_width + object_data[idx].y_width) / 2;
#endif
			if (bdata->swap_axes) {
				temp = x;
				x = y;
				y = temp;
			}
			if (bdata->x_flip)
				x = touch_hcd->input_params.max_x - x;
			if (bdata->y_flip)
				y = touch_hcd->input_params.max_y - y;
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(touch_hcd->input_dev,
					BTN_TOUCH, 1);
			input_report_key(touch_hcd->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_Z_MAJOR_VALUE
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_PRESSURE, z);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_TOUCH_MAJOR, major);
#endif

#ifndef TYPE_B_PROTOCOL
			input_mt_sync(touch_hcd->input_dev);
#endif

#ifdef REPORT_Z_MAJOR_VALUE
			LOGD(tcm_hcd->pdev->dev.parent,
					"Finger %d: x = %d, y = %d z = %d, x_width:%d, y_width:%d\n",
					idx, x, y, z, object_data[idx].x_width, object_data[idx].y_width);
#else
			LOGD(tcm_hcd->pdev->dev.parent,
					"Finger %d: x = %d, y = %d\n",
					idx, x, y);
#endif
			touch_count++;
			if (tcm_hcd->log_level > 2) OVT_INFO("Finger %d: x = %d, y = %d, touch_count:%d\n", idx, x, y, touch_count);
			break;
		default:
			break;
		}

		touch_hcd->prev_status[idx] = object_data[idx].status;
	}

	if (touch_count == 0) {
		input_report_key(touch_hcd->input_dev,
				BTN_TOUCH, 0);
		input_report_key(touch_hcd->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(touch_hcd->input_dev);
#endif
	}

	input_sync(touch_hcd->input_dev);

#ifdef CONFIG_TP_LAST_TIME
	if (touch_count > 0) {
		//touch down/moving
		if(time_flag) {
			tcm_hcd->last_event_time = ktime_get_boottime();
			//skip all other touch moving events before UP
			time_flag = 0;
			if (tcm_hcd->log_level > 1) OVT_INFO("set last_event_time, touch_count:%d\n", touch_count);
		}
	}
	else {
		//touch_count 0, enable time flag for next touch
		time_flag = 1;
		if (tcm_hcd->log_level > 1) OVT_INFO("reset time_flag 1 for last_event\n");
	}
#endif

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	return;
}

/**
 * touch_set_input_params() - Set input parameters
 *
 * Set the input parameters of the input device based on the information
 * retrieved from the application information packet. In addition, set up an
 * array for tracking the status of touch objects.
 */
static int touch_set_input_params(void)
{
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_X, 0, touch_hcd->max_x, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_Y, 0, touch_hcd->max_y, 0, 0);

#ifdef REPORT_Z_MAJOR_VALUE
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_PRESSURE, 0, MAX_Z_VALUE, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_TOUCH_MAJOR, 0, MAX_MAJOR_VALUE, 0, 0);
#endif

	input_mt_init_slots(touch_hcd->input_dev, touch_hcd->max_objects,
			INPUT_MT_DIRECT);

	touch_hcd->input_params.max_x = touch_hcd->max_x;
	touch_hcd->input_params.max_y = touch_hcd->max_y;
	touch_hcd->input_params.max_objects = touch_hcd->max_objects;

	if (touch_hcd->max_objects == 0)
		return 0;

	kfree(touch_hcd->prev_status);
	touch_hcd->prev_status = kzalloc(touch_hcd->max_objects, GFP_KERNEL);
	if (!touch_hcd->prev_status) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->prev_status\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * touch_get_input_params() - Get input parameters
 *
 * Retrieve the input parameters to register with the input subsystem for
 * the input device from the application information packet. In addition,
 * the touch report configuration is retrieved and stored.
 */
static int touch_get_input_params(void)
{
	int retval;
	unsigned int temp;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	app_info = &tcm_hcd->app_info;
	touch_hcd->max_x = le2_to_uint(app_info->max_x);
	touch_hcd->max_y = le2_to_uint(app_info->max_y);
	touch_hcd->max_objects = le2_to_uint(app_info->max_objects);

	if (bdata->swap_axes) {
		temp = touch_hcd->max_x;
		touch_hcd->max_x = touch_hcd->max_y;
		touch_hcd->max_y = temp;
	}

	LOCK_BUFFER(tcm_hcd->config);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			&tcm_hcd->config.buf,
			&tcm_hcd->config.buf_size,
			&tcm_hcd->config.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(tcm_hcd->config);
		return retval;
	}

	UNLOCK_BUFFER(tcm_hcd->config);

	return 0;
}

/**
 * touch_set_input_dev() - Set up input device
 *
 * Allocate an input device, configure the input device based on the particular
 * input events to be reported, and register the input device with the input
 * subsystem.
 */
static int touch_set_input_dev(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	touch_hcd->input_dev = input_allocate_device();
	if (touch_hcd->input_dev == NULL) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate input device\n");
		return -ENODEV;
	}

	touch_hcd->input_dev->name = TOUCH_INPUT_NAME;
	touch_hcd->input_dev->phys = TOUCH_INPUT_PHYS_PATH;
	touch_hcd->input_dev->id.product = OMNIVISION_TCM_ID_PRODUCT;
	touch_hcd->input_dev->id.version = OMNIVISION_TCM_ID_VERSION;
	touch_hcd->input_dev->dev.parent = tcm_hcd->pdev->dev.parent;
	input_set_drvdata(touch_hcd->input_dev, tcm_hcd);

	set_bit(EV_SYN, touch_hcd->input_dev->evbit);
	set_bit(EV_KEY, touch_hcd->input_dev->evbit);
	set_bit(EV_ABS, touch_hcd->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_hcd->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, touch_hcd->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, touch_hcd->input_dev->propbit);
#endif

#if WAKEUP_GESTURE
#ifdef OVT_DOUBLE_TAP_CTRL
	set_bit(KEY_F1, touch_hcd->input_dev->keybit);
	set_bit(KEY_F4, touch_hcd->input_dev->keybit);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_F1);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_F4);
#else
	set_bit(KEY_POWER, touch_hcd->input_dev->keybit);
	set_bit(KEY_U, touch_hcd->input_dev->keybit);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_POWER);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_U);
#endif
#endif

	retval = touch_set_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set input parameters\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}

	retval = input_register_device(touch_hcd->input_dev);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register input device\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}

	return 0;
}
/*

*/
#if SUPPORT_FACE_DETECT
static int ovt_check_face_state(int current_face_state)
{
	//static int pre_face_state = FACE_STATUS_NONE;
	if ((current_face_state == FACE_FAR_FROM_1_2_3_CLOSE_WHEN_SCREEN_ON) ||
		(current_face_state == FACE_FAR_FROM_1_2_3_CLOSE_WHEN_SCREEN_OFF)) {
		//report far event
		printk("tcm check face far\n");
	} else if ((current_face_state == FACE_CLOSE_1_SMALL_SIGNAL) ||
				(current_face_state == FACE_CLOSE_FROM_1_2_3_CLOSE_WHEN_SCREEN_OFF)) {

		//report close event
		printk("tcm check face close\n");
	}
	return 0;
}
#endif
/**
 * touch_set_report_config() - Set touch report configuration
 *
 * Send the SET_TOUCH_REPORT_CONFIG command to configure the format and content
 * of the touch report.
 */
static int touch_set_report_config(void)
{
	int retval;
	unsigned int idx;
	unsigned int length;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

#ifdef USE_DEFAULT_TOUCH_REPORT_CONFIG
	return 0;
#endif

	app_info = &tcm_hcd->app_info;
	length = le2_to_uint(app_info->max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&touch_hcd->out,
			length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	idx = 0;
#if WAKEUP_GESTURE
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_ID;
	touch_hcd->out.buf[idx++] = 8;
#endif
#if SUPPORT_FACE_DETECT
	touch_hcd->out.buf[idx++] = TOUCH_FACE_DETECT;
	touch_hcd->out.buf[idx++] = 8;
#endif
#if SUPPORT_KNUCKLE_DATA_REPORT
	touch_hcd->out.buf[idx++] = TOUCH_KNUCKLE_DATA;
	touch_hcd->out.buf[idx++] = (KNUCKLE_DATA_SIZE * 8) & 0xff;
	touch_hcd->out.buf[idx++] = (KNUCKLE_DATA_SIZE * 8) >> 8;
#endif
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_PALM_DETECTED;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_INDEX;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_WIDTH;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_WIDTH;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Z;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_END;
	touch_hcd->out.buf[idx++] = TOUCH_END;

	LOCK_BUFFER(touch_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_TOUCH_REPORT_CONFIG,
			touch_hcd->out.buf,
			length,
			&touch_hcd->resp.buf,
			&touch_hcd->resp.buf_size,
			&touch_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	LOGN(tcm_hcd->pdev->dev.parent,
			"Set touch config done\n");

	return 0;
}

/**
 * touch_check_input_params() - Check input parameters
 *
 * Check if any of the input parameters registered with the input subsystem for
 * the input device has changed.
 */
static int touch_check_input_params(void)
{
	unsigned int size;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (touch_hcd->max_x == 0 && touch_hcd->max_y == 0)
		return 0;

	if (touch_hcd->input_params.max_objects != touch_hcd->max_objects) {
		kfree(touch_hcd->touch_data.object_data);
		size = sizeof(*touch_hcd->touch_data.object_data);
		size *= touch_hcd->max_objects;
		touch_hcd->touch_data.object_data = kzalloc(size, GFP_KERNEL);
		if (!touch_hcd->touch_data.object_data) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for touch_hcd->touch_data.object_data\n");
			return -ENOMEM;
		}
		return 1;
	}

	if (touch_hcd->input_params.max_x != touch_hcd->max_x)
		return 1;

	if (touch_hcd->input_params.max_y != touch_hcd->max_y)
		return 1;

	return 0;
}

/**
 * touch_set_input_reporting() - Configure touch report and set up new input
 * device if necessary
 *
 * After a device reset event, configure the touch report and set up a new input
 * device if any of the input parameters has changed after the device reset.
 */
static int touch_set_input_reporting(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);

		return 0;
	}

	touch_hcd->init_touch_ok = false;

	touch_free_objects();

	mutex_lock(&touch_hcd->report_mutex);

	retval = touch_set_report_config();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set report config\n");
		goto exit;
	}

	retval = touch_get_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get input parameters\n");
		goto exit;
	}

	retval = touch_check_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to check input parameters\n");
		goto exit;
	} else if (retval == 0) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Input parameters unchanged\n");
		goto exit;
	}

	if (touch_hcd->input_dev != NULL) {
		input_unregister_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
	}

	retval = touch_set_input_dev();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input device\n");
		goto exit;
	}

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	touch_hcd->init_touch_ok = retval < 0 ? false : true;

	return retval;
}

#ifdef OVT_TAP_SENSOR_EN
static int ovt_tap_sensor_set_enable(struct sensors_classdev *sensors_cdev,
    unsigned int enable)
{
    //OVT_DEBUG("double tap ctrl, do nothing\n");
    return 0;
}

static int ovt_tap_sensor_init(struct ovt_tcm_hcd *data)
{
    struct ovt_tap_sensor_platform_data *sensor_pdata;
    struct input_dev *sensor_input_dev;
    int err;

    sensor_input_dev = input_allocate_device();
    if (!sensor_input_dev) {
        OVT_ERROR("Failed to allocate device");
        goto exit;
    }

    sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
                                sizeof(struct ovt_tap_sensor_platform_data),
                                GFP_KERNEL);
    if (!sensor_pdata) {
        OVT_ERROR("Failed to allocate memory");
        goto free_sensor_pdata;
    }
    data->sensor_pdata = sensor_pdata;

    __set_bit(EV_KEY, sensor_input_dev->evbit);
    __set_bit(KEY_F1, sensor_input_dev->keybit);
    __set_bit(KEY_F4, sensor_input_dev->keybit);
    __set_bit(EV_SYN, sensor_input_dev->evbit);

    sensor_input_dev->name = "double-tap";
    data->sensor_pdata->input_sensor_dev = sensor_input_dev;

    err = input_register_device(sensor_input_dev);
    if (err) {
        OVT_ERROR("Unable to register device, err=%d", err);
        goto free_sensor_input_dev;
    }

    sensor_pdata->ps_cdev = sensors_touch_cdev;
    sensor_pdata->ps_cdev.sensors_enable = ovt_tap_sensor_set_enable;
    sensor_pdata->data = data;

    err = sensors_classdev_register(&sensor_input_dev->dev,
                                    &sensor_pdata->ps_cdev);
    if (err)
        goto unregister_sensor_input_device;

    OVT_INFO("done");
    return 0;

unregister_sensor_input_device:
    input_unregister_device(data->sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
    input_free_device(data->sensor_pdata->input_sensor_dev);
free_sensor_pdata:
    devm_kfree(&sensor_input_dev->dev, sensor_pdata);
    data->sensor_pdata = NULL;
exit:
    OVT_INFO("exit 1");
    return 1;
}

int ovt_tap_sensor_remove(struct ovt_tcm_hcd *data)
{
    if (!data || !data->sensor_pdata) {
	    OVT_INFO("data null return");
	    return 0;
    }

    sensors_classdev_unregister(&data->sensor_pdata->ps_cdev);
    input_unregister_device(data->sensor_pdata->input_sensor_dev);
    devm_kfree(&data->sensor_pdata->input_sensor_dev->dev,
               data->sensor_pdata);
    data->sensor_pdata = NULL;
    //data->wakeable = false;
    data->wakeup_gesture_enabled = false;
    OVT_INFO("end");
    return 0;
}
#endif

int touch_init(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
#ifdef OVT_TAP_SENSOR_EN
    static bool initialized_tap_sensor;
#endif

	OVT_INFO("enter");
	touch_hcd = kzalloc(sizeof(*touch_hcd), GFP_KERNEL);
	if (!touch_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd\n");
		return -ENOMEM;
	}

	touch_hcd->tcm_hcd = tcm_hcd;

	mutex_init(&touch_hcd->report_mutex);

	INIT_BUFFER(touch_hcd->out, false);
	INIT_BUFFER(touch_hcd->resp, false);

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
		goto err_set_input_reporting;
	}

	tcm_hcd->report_touch = touch_report;

#ifdef OVT_TAP_SENSOR_EN
    if (!initialized_tap_sensor) {
#ifdef CONFIG_HAS_WAKELOCK
        wake_lock_init(&gesture_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#else
        PM_WAKEUP_REGISTER(tcm_hcd->pdev->dev.parent, gesture_wakelock, "poll-wake-lock");
        if (!gesture_wakelock) {
            OVT_ERROR("failed to allocate wakeup source\n");
            //return -ENOMEM;
        }
#endif

        if (!ovt_tap_sensor_init(tcm_hcd))
            initialized_tap_sensor = true;

        //OVT_DEBUG("initialized_tap_sensor:%d", initialized_tap_sensor);
    }
#endif

    OVT_INFO("end");
    return 0;

err_set_input_reporting:
	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

	return retval;
}

int touch_remove(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!touch_hcd)
		goto exit;

	OVT_INFO("enter");
	tcm_hcd->report_touch = NULL;

	if (touch_hcd->input_dev)
		input_unregister_device(touch_hcd->input_dev);

#ifdef OVT_TAP_SENSOR_EN
	ovt_tap_sensor_remove(tcm_hcd);
#endif

	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

exit:

	return 0;
}

int touch_reinit(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	if (!touch_hcd) {
		retval = touch_init(tcm_hcd);
		return retval;
	}

	touch_free_objects();

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Application mode is not running (firmware mode = %d)\n",
				tcm_hcd->id_info.mode);
		return 0;
	}

	if (!tcm_hcd->in_hdl_mode) {
		retval = tcm_hcd->identify(tcm_hcd, false);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
			return retval;
		}
	}

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
	}

	return retval;
}

int touch_early_suspend(struct ovt_tcm_hcd *tcm_hcd)
{
	OVT_FUNC_ENTER();
	if (!touch_hcd)
		return 0;

	touch_hcd->suspend_touch_finger = true;
	if (tcm_hcd->wakeup_gesture_enabled)
		touch_hcd->suspend_touch = false;
	else
		touch_hcd->suspend_touch = true;

	touch_free_objects();

	return 0;
}

#ifdef WAKEUP_GESTURE
int ovt_gesture_suspend(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	OVT_FUNC_ENTER();
	if (tcm_hcd->wakeup_gesture_enabled) {

#ifndef OVT_PANEL_GESTURE_NOTIFY_SUPPORT
		unsigned short gesture_cmd = 0;
#endif

		if (!touch_hcd->irq_wake) {
			enable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = true;
		}

		touch_hcd->suspend_touch = false;

#ifndef OVT_PANEL_GESTURE_NOTIFY_SUPPORT
		retval = tcm_hcd->set_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, 1);
		if (retval < 0) {
			OVT_ERROR("Failed to enable wakeup gesture mode\n");
			return retval;
		}

		if(tcm_hcd->wakeup_gesture_enabled == 1) {
			gesture_cmd = 0x8000;//single tap
		} else if(tcm_hcd->wakeup_gesture_enabled == 2) {
			gesture_cmd = 0x0001;//double
		} else if(tcm_hcd->wakeup_gesture_enabled == 3) {
			gesture_cmd = 0x8001;//all
		} else
			OVT_ERROR("invalid gesture mode:%d\n", tcm_hcd->wakeup_gesture_enabled);

		retval = tcm_hcd->set_dynamic_config(tcm_hcd, 0xFE, gesture_cmd);
		if (retval < 0)
			OVT_ERROR("Failed to set gesture_cmd: 0x%hx\n", gesture_cmd);
		else
			OVT_INFO("set gesture_cmd: 0x%hx for mode:%d\n", gesture_cmd, tcm_hcd->wakeup_gesture_enabled);
#endif

		touch_set_state(TOUCH_LOW_POWER_STATE, TOUCH_PANEL_IDX_PRIMARY);

#ifndef OVT_PANEL_GESTURE_NOTIFY_SUPPORT
#ifdef OVT_STOWED_MODE_SUPPORT
		if (g_tcm_hcd->get_stowed) {
			retval = ovt_tcm_sleep(tcm_hcd, 1);
			if (retval < 0)
				OVT_INFO("fail to enable stowed mode when supsend\n");
			else {
				g_tcm_hcd->set_stowed = g_tcm_hcd->get_stowed;
				OVT_INFO("Enable stowed mode when suspend\n");
			}
		}
#endif
#endif

		//OVT_INFO("gesture enable mode:%d\n", tcm_hcd->wakeup_gesture_enabled);
	}
	else {
		touch_set_state(TOUCH_DEEP_SLEEP_STATE, TOUCH_PANEL_IDX_PRIMARY);
		OVT_INFO("gesture disable, touch deep sleep state\n");
	}

	return retval;
}
#endif

int touch_suspend(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!touch_hcd)
		return 0;

	OVT_FUNC_ENTER();
	touch_hcd->suspend_touch_finger = true;
	touch_hcd->suspend_touch = true;

	touch_free_objects();

#ifdef WAKEUP_GESTURE
	ovt_gesture_suspend(tcm_hcd);
#endif

	return 0;
}

int touch_resume(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	OVT_FUNC_ENTER();
	if (!touch_hcd)
		return 0;

	touch_hcd->suspend_touch = false;
	touch_hcd->suspend_touch_finger = false;

	if (tcm_hcd->wakeup_gesture_enabled) {
		if (touch_hcd->irq_wake) {
			disable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = false;
		}

#ifdef OVT_STOWED_MODE_SUPPORT
		g_tcm_hcd->set_stowed = 0;
#endif

		retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_IN_WAKEUP_GESTURE_MODE,
				0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable wakeup gesture mode\n");
			return retval;
		}
	}

	OVT_FUNC_EXIT();
	return 0;
}




