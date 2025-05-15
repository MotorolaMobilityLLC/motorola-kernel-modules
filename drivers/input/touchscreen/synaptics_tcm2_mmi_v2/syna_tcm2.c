// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
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
 * This file implements the Synaptics device driver running under Linux kernel
 * input device subsystem, and also communicate with Synaptics touch controller
 * through TouchComm command-response protocol.
 */

#include "syna_tcm2.h"
#include "syna_tcm2_cdev.h"
#include "syna_tcm2_platform.h"
#include "tcm/synaptics_touchcom_core_dev.h"
#include "tcm/synaptics_touchcom_func_base.h"
#include "tcm/synaptics_touchcom_func_touch.h"
#ifdef REFLASH_DISCRETE_TOUCH
#include "synaptics_touchcom_func_reflash.h"
#endif
#ifdef REFLASH_TDDI
#include "synaptics_touchcom_func_reflash_tddi.h"
#endif

#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
/* An example of the format of custom touch configuration  */
static unsigned char custom_touch_format[] = {
	/* entity code */                    /* bits */
#ifdef ENABLE_WAKEUP_GESTURE
	TOUCH_REPORT_GESTURE_ID,                8,
#endif
	TOUCH_REPORT_NUM_OF_ACTIVE_OBJECTS,     8,
	TOUCH_REPORT_FOREACH_ACTIVE_OBJECT,
	TOUCH_REPORT_OBJECT_N_INDEX,            8,
	TOUCH_REPORT_OBJECT_N_CLASSIFICATION,   8,
	TOUCH_REPORT_OBJECT_N_X_POSITION,       16,
	TOUCH_REPORT_OBJECT_N_Y_POSITION,       16,
	TOUCH_REPORT_FOREACH_END,
	TOUCH_REPORT_END
};
#endif

#ifdef RESET_ON_RESUME
/* The delayed time before issuing a reset if reset on resume is required */
#define RESET_ON_RESUME_DELAY_MS (100)
#endif



#if defined(ENABLE_HELPER)
/*
 * Example of the helper work.
 * Helper could run on the other context and send the secondary command to device.
 *
 * param
 *    [ in] work: data for work used
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static void syna_dev_helper_work(struct work_struct *work)
{
	unsigned char task;
	struct syna_tcm_helper *helper =
			container_of(work, struct syna_tcm_helper, work);
	struct syna_tcm *tcm =
			container_of(helper, struct syna_tcm, helper);

	task = ATOMIC_GET(helper->task);

	switch (task) {
	case HELP_RESET_DETECTED:
		LOGD("Reset caught (device mode:0x%x)\n", tcm->tcm_dev->dev_mode);
		break;
	default:
		break;
	}

	ATOMIC_SET(helper->task, HELP_NONE);
}
#endif
/*
 * Example to process the report resulted by the unexpected reset.
 *
 * param
 *    [ in]    code:          the code of current touch entity
 *    [ in]    report:        touch report given
 *    [ in]    report_size:   size of given touch report
 *    [ in]    callback_data: private data being passed to the callback function
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_process_unexpected_reset(const unsigned char code,
	const unsigned char *report, unsigned int report_size,
	void *callback_data)
{
	struct syna_tcm *tcm = (struct syna_tcm *)callback_data;

	if (!tcm) {
		LOGE("Invalid data to process\n");
		return -EINVAL;
	}

#ifdef RESET_ON_RESUME
	if (tcm->pwr_state != PWR_ON)
		return 0;
#endif

	LOGN("Device has been reset, may be the spontaneous reset\n");

#if defined(ENABLE_HELPER)
	/* send the command through helper thread */
	if (!tcm->helper.workqueue) {
		LOGW("No helper thread created\n");
		return -EINVAL;
	}

	if (ATOMIC_GET(tcm->helper.task) == HELP_NONE) {
		ATOMIC_SET(tcm->helper.task, HELP_RESET_DETECTED);
		queue_work(tcm->helper.workqueue, &tcm->helper.work);
	}
#endif

	return 0;
}
#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
/*
 * Callback to parse the custom or non-standard touch entity from the touch report.
 *
 * Please be noted that this function will be invoked in ISR so don't issue another
 * touchcomm command here. If really needed, please assign a task to helper thread.
 *
 * param
 *    [ in]    code:          the code of current touch entity
 *    [ in]    config:        the report configuration stored
 *    [in/out] config_offset: offset of current position in report config,
 *                            and then return the updated position.
 *    [ in]    report:        touch report given
 *    [in/out] report_offset: offset of current position in touch report,
 *                            the updated position should be returned.
 *    [ in]    report_size:   size of given touch report
 *    [ in]    callback_data: pointer to caller data passed to callback function
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_parse_custom_touch_data_cb(const unsigned char code,
	const unsigned char *config, unsigned int *config_offset,
	const unsigned char *report, unsigned int *report_offset,
	unsigned int report_size, void *callback_data)
{
	/*
	 * sample code to demonstrate how to parse the custom touch entity
	 * from the touch report, additional modifications will be needed.
	 *
	 * struct syna_tcm *tcm = (struct syna_tcm *)callback_data;
	 * unsigned int data;
	 * unsigned int bits;
	 *
	 * switch (code) {
	 * case CUSTOM_ENTITY_CODE:
	 *		bits = config[(*config_offset)++];
	 *		syna_tcm_get_touch_data(report, report_size, *report_offset, bits, &data);
	 *		*report_offset += bits;
	 *		return bits;
	 *	default:
	 *		LOGW("Unknown touch config code (idx:%d 0x%02x)\n", *config_offset, code);
	 *		return (-EINVAL);
	 *	}
	 *
	 */

	return (-EINVAL);
}
#endif

/*
 * Release all relevant touched events.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_free_input_events(struct syna_tcm *tcm)
{
	struct input_dev *input_dev = tcm->input_dev;
#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif

	if (input_dev == NULL)
		return;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

#ifdef TYPE_B_PROTOCOL
	for (idx = 0; idx < MAX_NUM_OBJECTS; idx++) {
		input_mt_slot(input_dev, idx);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(input_dev);
#endif
	input_sync(input_dev);

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

}
#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
static void syna_dev_report_input_knob_events(struct syna_tcm *tcm)
{
	struct input_dev *knob_dev;
	struct tcm_touch_data_blob *touch_data;
	struct tcm_knob_data_blob *knob_data;
	int idx;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	touch_data = &tcm->tp_data;

	for (idx = 0; idx < MAX_NUM_KNOB_OBJECTS; idx++) {
#ifndef HAVE_THE_SECOND_KNOB
		if (idx > 0)
			break;
#endif
		knob_dev = tcm->input_knob_dev[idx];
		if (knob_dev == NULL)
			continue;

		knob_data = &touch_data->knob[idx];

		if (knob_data->is_updated) {
			input_report_key(knob_dev, BTN_WHEEL, knob_data->grasp);
			if (knob_data->grasp)
				input_report_abs(knob_dev, ABS_WHEEL, knob_data->angle);
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(knob_dev);
#endif
			input_sync(knob_dev);
		}

		if (knob_data->is_clicked) {
			input_report_key(knob_dev, BTN_SELECT, knob_data->click);
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(knob_dev);
#endif
			input_sync(knob_dev);
		}
	}

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);
}
#endif
/*
 * Report touched events to the input subsystem.
 * Assuming that touched data including touch_data_blob and
 * objects_data_blob are ready to use.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_report_input_events(struct syna_tcm *tcm)
{
	unsigned int idx;
	unsigned int x;
	unsigned int y;
	int wx;
	int wy;
	unsigned int status;
	unsigned int touch_count;
	struct input_dev *input_dev = tcm->input_dev;
	unsigned int max_objects = tcm->tcm_dev->max_objects;
	struct tcm_touch_data_blob *touch_data;
	struct tcm_objects_data_blob *object_data;

	if (input_dev == NULL)
		return;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	touch_data = &tcm->tp_data;
	object_data = &touch_data->object_data[0];

#ifdef ENABLE_WAKEUP_GESTURE
	if ((tcm->pwr_state == LOW_PWR) && tcm->irq_wake) {
		if (touch_data->gesture_id) {
			LOGD("Gesture detected, id:%d\n",
				touch_data->gesture_id);

			input_report_key(input_dev, KEY_WAKEUP, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_WAKEUP, 0);
			input_sync(input_dev);
		}
	}
#endif

	if (tcm->pwr_state == LOW_PWR)
		goto exit;

	touch_count = 0;

	for (idx = 0; idx < max_objects; idx++) {
		if (tcm->prev_obj_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, idx);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 0);
#endif
			break;
		case FINGER:
		case GLOVED_OBJECT:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
			wx = object_data[idx].x_width;
			wy = object_data[idx].y_width;

#ifdef REPORT_SWAP_XY
			x = x ^ y;
			y = x ^ y;
			x = x ^ y;
#endif
#ifdef REPORT_FLIP_X
			x = tcm->input_dev_params.max_x - x;
#endif
#ifdef REPORT_FLIP_Y
			y = tcm->input_dev_params.max_y - y;
#endif
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(input_dev, idx);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_report_key(input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#ifdef REPORT_TOUCH_WIDTH
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MAJOR, MAX_SYN(wx, wy));
			input_report_abs(input_dev,
					ABS_MT_TOUCH_MINOR, MIN_SYN(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
			LOGD("Finger %d: x = %d, y = %d\n", idx, x, y);
			touch_count++;
			break;
		default:
			break;
		}

		tcm->prev_obj_status[idx] = object_data[idx].status;
	}

	if (touch_count == 0) {
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_key(input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
	}

	input_sync(input_dev);

exit:
	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

}
/*
 * Process the touch report.
 *
 * param
 *    [ in]    code:          the code of given report type
 *    [ in]    report:        report data given
 *    [ in]    report_size:   size of the given report
 *    [ in]    callback_data: private data being passed to the callback function
 *
 * return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_dev_process_touch_report(const unsigned char code,
	const unsigned char *report, unsigned int report_size,
	void *callback_data)
{
	int retval;
	struct syna_tcm *tcm = (struct syna_tcm *)callback_data;

	if (!tcm) {
		LOGE("Invalid callback data\n");
		return -EINVAL;
	}

	if (code != REPORT_TOUCH) {
		LOGE("Invalid report to process, report:%d\n", code);
		return -EINVAL;
	}

	/* not report to input device subsystem if the cdev interface is in used */
	if ((tcm->char_dev_ref_count > 0) && !tcm->concurrent_reporting)
		return 0;

	/* parse touch report once received */
	retval = syna_tcm_parse_touch_report(tcm->tcm_dev,
			(unsigned char *)report,
			report_size,
			&tcm->tp_data);
	if (retval < 0) {
		LOGE("Fail to parse touch report\n");
		return retval;
	}
	/* report the touch event to system */
	syna_dev_report_input_events(tcm);
#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
	/* report the knob event to system */
	syna_dev_report_input_knob_events(tcm);
#endif
	return 0;
}

#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
/*
 * Allocate an input device for knob and set up relevant parameters
 * to the input subsystem.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_create_input_knob_device(struct syna_tcm *tcm)
{
	int retval = 0;
	int idx = 0;
#ifdef DEV_MANAGED_API
	struct device *dev;
#endif

#ifdef DEV_MANAGED_API
	dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return -EINVAL;
	}
#endif

	for (idx = 0; idx < MAX_NUM_KNOB_OBJECTS; idx++) {
#ifndef HAVE_THE_SECOND_KNOB
		if (idx > 0)
			break;
#endif
		if (tcm->input_knob_dev[idx]) {
			input_unregister_device(tcm->input_knob_dev[idx]);
			tcm->input_knob_dev[idx] = NULL;
		}

#ifdef DEV_MANAGED_API
		tcm->input_knob_dev[idx] = devm_input_allocate_device(dev);
#else /* Legacy API */
		tcm->input_knob_dev[idx] = input_allocate_device();
#endif
		if (tcm->input_knob_dev[idx] == NULL) {
			LOGE("Fail to allocate input device %d for knob\n", idx);
			break;
		}

		tcm->input_knob_dev[idx]->name = KNOB_INPUT_NAME;
		tcm->input_knob_dev[idx]->phys = KNOB_INPUT_PHYS_PATH;
		tcm->input_knob_dev[idx]->id.product = SYNAPTICS_TCM_DRIVER_ID;
		tcm->input_knob_dev[idx]->id.version = SYNAPTICS_TCM_DRIVER_VERSION;
		tcm->input_knob_dev[idx]->dev.parent = tcm->pdev->dev.parent;
		input_set_drvdata(tcm->input_knob_dev[idx], tcm);

		input_set_capability(tcm->input_knob_dev[idx], EV_KEY, BTN_SELECT);
		input_set_capability(tcm->input_knob_dev[idx], EV_KEY, BTN_WHEEL);
		input_set_capability(tcm->input_knob_dev[idx], EV_ABS, ABS_WHEEL);

		input_set_abs_params(tcm->input_knob_dev[idx], ABS_WHEEL, 0, 32, 0, 0);

		retval = input_register_device(tcm->input_knob_dev[idx]);
		if (retval < 0) {
			LOGE("Fail to register input device for knob\n");
			input_free_device(tcm->input_knob_dev[idx]);
			tcm->input_knob_dev[idx] = NULL;
			return retval;
		}
	}

	return 0;
}
/*
 * Release the input device for knob allocated previously.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_release_input_knob_device(struct syna_tcm *tcm)
{
	int idx = 0;

	for (idx = 0; idx < MAX_NUM_KNOB_OBJECTS; idx++) {
#ifndef HAVE_THE_SECOND_KNOB
		if (idx > 0)
			break;
#endif
		if (tcm->input_knob_dev[idx]) {
			input_unregister_device(tcm->input_knob_dev[idx]);
			tcm->input_knob_dev[idx] = NULL;
		}
	}
}
#endif

/*
 * Allocate an input device and set up relevant parameters to the
 * input subsystem.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_create_input_device(struct syna_tcm *tcm)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	struct input_dev *input_dev = NULL;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return -EINVAL;
	}

	input_dev = devm_input_allocate_device(dev);
#else /* Legacy API */
	input_dev = input_allocate_device();
#endif
	if (input_dev == NULL) {
		LOGE("Fail to allocate input device\n");
		return -ENODEV;
	}

	input_dev->name = TOUCH_INPUT_NAME;
	input_dev->phys = TOUCH_INPUT_PHYS_PATH;
	input_dev->id.product = SYNAPTICS_TCM_DRIVER_ID;
	input_dev->id.version = SYNAPTICS_TCM_DRIVER_VERSION;
	input_dev->dev.parent = tcm->pdev->dev.parent;
	input_set_drvdata(input_dev, tcm);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

#ifdef ENABLE_WAKEUP_GESTURE
	set_bit(KEY_WAKEUP, input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, tcm_dev->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, tcm_dev->max_y, 0, 0);

	input_mt_init_slots(input_dev, tcm_dev->max_objects, INPUT_MT_DIRECT);

#ifdef REPORT_TOUCH_WIDTH
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
#endif

	tcm->input_dev_params.max_x = tcm_dev->max_x;
	tcm->input_dev_params.max_y = tcm_dev->max_y;
	tcm->input_dev_params.max_objects = tcm_dev->max_objects;

	retval = input_register_device(input_dev);
	if (retval < 0) {
		LOGE("Fail to register input device\n");
		input_free_device(input_dev);
		input_dev = NULL;
		return retval;
	}

	tcm->input_dev = input_dev;

	return 0;
}
/*
 * Release the input device allocated previously.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_release_input_device(struct syna_tcm *tcm)
{
	if (!tcm->input_dev)
		return;

	input_unregister_device(tcm->input_dev);

	tcm->input_dev = NULL;
}
/*
 * Check whether it's need to register the new input device
 * if any of parameters has been changed.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    true if parameters are mismatched, false otherwise.
 */
static bool syna_dev_check_input_params(struct syna_tcm *tcm)
{
	struct tcm_dev *tcm_dev = tcm->tcm_dev;

	if (tcm_dev->max_x == 0 && tcm_dev->max_y == 0)
		return false;

	if (tcm->input_dev_params.max_x != tcm_dev->max_x)
		return true;

	if (tcm->input_dev_params.max_y != tcm_dev->max_y)
		return true;

	if (tcm->input_dev_params.max_objects != tcm_dev->max_objects)
		return true;

	if (tcm_dev->max_objects > MAX_NUM_OBJECTS) {
		LOGW("Out of max num objects defined, in app_info: %d\n",
			tcm_dev->max_objects);
		return false;
	}

	LOGN("Input parameters non-changed\n");
	return false;
}

/*
 * Set up input device to the input subsystem.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_dev_set_up_input_device(struct syna_tcm *tcm)
{
	int retval;

	if (IS_NOT_APP_FW_MODE(tcm->tcm_dev->dev_mode)) {
		LOGN("Application firmware not running, current mode: %02x\n",
			tcm->tcm_dev->dev_mode);
		return 0;
	}

	syna_dev_free_input_events(tcm);

	if (!syna_dev_check_input_params(tcm))
		return 0;

	syna_pal_mutex_lock(&tcm->tp_event_mutex);

	if (tcm->input_dev != NULL)
		syna_dev_release_input_device(tcm);

	retval = syna_dev_create_input_device(tcm);
	if (retval < 0) {
		LOGE("Fail to create input device\n");
		syna_pal_mutex_unlock(&tcm->tp_event_mutex);
		return retval;
	}

	/* register the handling function for touch reports */
	retval = syna_tcm_set_report_dispatcher(tcm->tcm_dev,
			REPORT_TOUCH, syna_dev_process_touch_report, (void *)tcm);
	if (retval < 0)
		LOGE("Fail to register the touch report handling function\n");

	syna_pal_mutex_unlock(&tcm->tp_event_mutex);

	return 0;
}

/*
 * Interrupt handling routine.
 *
 * Function is called when the interrupt is asserted.
 * The purposes of this handler is to read out an event generated by device.
 *
 * param
 *    [ in] irq:  interrupt line
 *    [ in] data: private data being passed to the handler function
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static irqreturn_t syna_dev_isr(int irq, void *data)
{
	int retval;
	unsigned char code = 0;
	struct syna_tcm *tcm = data;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;

	if (unlikely(gpio_get_value(attn->irq_gpio) != attn->irq_on_state))
		goto exit;

	tcm->isr_pid = current->pid;

	/* retrieve the original report date generated by firmware */
	retval = syna_tcm_get_event_data(tcm->tcm_dev,
			&code,
			&tcm->event_data);
	if (retval < 0) {
		LOGE("Fail to get event data\n");
		goto exit;
	}

exit:
	return IRQ_HANDLED;
}

/*
 * Request interrupt line and register the interrupt handling routine.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_request_irq(struct syna_tcm *tcm)
{
	int retval;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		retval = -EINVAL;
		goto exit;
	}
#endif

	if (attn->irq_gpio < 0) {
		LOGE("Invalid IRQ GPIO\n");
		retval = -EINVAL;
		goto exit;
	}

	attn->irq_id = gpio_to_irq(attn->irq_gpio);

#ifdef DEV_MANAGED_API
	retval = devm_request_threaded_irq(dev,
			attn->irq_id,
			NULL,
			syna_dev_isr,
			attn->irq_flags,
			PLATFORM_DRIVER_NAME,
			tcm);
#else /* Legacy API */
	retval = request_threaded_irq(attn->irq_id,
			NULL,
			syna_dev_isr,
			attn->irq_flags,
			PLATFORM_DRIVER_NAME,
			tcm);
#endif
	if (retval < 0) {
		LOGE("Fail to request threaded irq\n");
		goto exit;
	}

	attn->irq_enabled = true;

	LOGI("Interrupt handler registered\n");

exit:
	return retval;
}

/*
 * Release an interrupt line allocated previously
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_release_irq(struct syna_tcm *tcm)
{
	struct tcm_hw_platform *hw = &tcm->hw_if->hw_platform;
	struct syna_hw_attn_data *attn = &tcm->hw_if->bdata_attn;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return;
	}
#endif

	if (attn->irq_id <= 0)
		return;

	if (hw->ops_enable_attn)
		hw->ops_enable_attn(hw, false);

#ifdef DEV_MANAGED_API
	devm_free_irq(dev, attn->irq_id, tcm);
#else
	free_irq(attn->irq_id, tcm);
#endif

	attn->irq_id = 0;
	attn->irq_enabled = false;

	LOGI("Interrupt handler released\n");
}

/*
 * Initialization including the preparation of app info and the
 * configuration of touch report.
 *
 * param
 *    [ in] tcm: tcm driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_set_up_app_fw(struct syna_tcm *tcm)
{
	int retval = 0;
	struct tcm_dev *tcm_dev;
	struct syna_hw_attn_data *attn;
	unsigned int resp_handling;

	if (!tcm)
		return -EINVAL;

	tcm_dev = tcm->tcm_dev;
	attn = &tcm->hw_if->bdata_attn;
	if (attn && (attn->irq_id) && (attn->irq_enabled))
		resp_handling = CMD_RESPONSE_IN_ATTN;
	else
		resp_handling = tcm_dev->msg_data.command_polling_time;

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGN("Application firmware not running, current mode: %02x\n",
			tcm_dev->dev_mode);
		return -EINVAL;
	}

	/* collect app info containing most of sensor information */
	retval = syna_tcm_get_app_info(tcm_dev, &tcm_dev->app_info, resp_handling);
	if (retval < 0) {
		LOGE("Fail to get application info\n");
		return retval;
	}

#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
	/* set up the format of touch report */
	retval = syna_tcm_set_touch_report_config(tcm_dev,
			custom_touch_format,
			(unsigned int)sizeof(custom_touch_format),
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to setup the custom touch report format\n");
		return retval;
	}
#endif
	/* preserve the format of touch report */
	retval = syna_tcm_preserve_touch_report_config(tcm_dev, resp_handling);
	if (retval < 0) {
		LOGE("Fail to preserve touch report config\n");
		return retval;
	}

#ifdef ENABLE_CUSTOM_TOUCH_ENTITY
	/* set up custom touch data parsing method */
	retval = syna_tcm_set_custom_touch_entity_callback(tcm_dev,
			syna_dev_parse_custom_touch_data_cb, tcm);
	if (retval < 0) {
		LOGE("Fail to set up custom touch data parsing method\n");
		return retval;
	}
#endif
	return 0;
}


#if defined(REFLASH_DISCRETE_TOUCH) || defined(REFLASH_TDDI)
/*
 * Helper to perform firmware update.
 *
 * param
 *    [ in] tcm:   tcm driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_dev_do_reflash(struct syna_tcm *tcm, bool force)
{
	int retval;
	const struct firmware *fw_entry = NULL;
	const unsigned char *fw_image = NULL;
	unsigned int fw_image_size;
	struct tcm_dev *tcm_dev;

	if (!tcm)
		return -EINVAL;

	tcm_dev = tcm->tcm_dev;

	/* get firmware image */
	retval = request_firmware(&fw_entry,
			FW_IMAGE_NAME,
			tcm->pdev->dev.parent);
	if (retval < 0) {
		LOGE("Fail to request %s\n", FW_IMAGE_NAME);
		return retval;
	}

	fw_image = fw_entry->data;
	fw_image_size = fw_entry->size;

	LOGD("Firmware image size = %d\n", fw_image_size);

#ifdef REFLASH_TDDI
	retval = syna_tcm_tddi_do_fw_update(tcm_dev,
			fw_image,
			fw_image_size,
			CMD_RESPONSE_IN_ATTN,
			force,
			tcm->is_tddi_multichip);
#else
	retval = syna_tcm_do_fw_update(tcm_dev,
			fw_image,
			fw_image_size,
			CMD_RESPONSE_IN_ATTN,
			force);
#endif
	if (retval < 0)
		LOGE("Fail to do reflash\n");

	LOGI("Firmware mode %02X after reflash\n", tcm_dev->dev_mode);

	release_firmware(fw_entry);
	fw_entry = NULL;
	fw_image = NULL;

	return retval;
}
#ifdef STARTUP_REFLASH
/*
 * Example to perform firmware update at the system startup.
 *
 * param
 *    [ in] work: handle of work structure
 *
 * return
 *    void.
 */
static void syna_dev_reflash_startup_work(struct work_struct *work)
{
	int retval;
	struct delayed_work *delayed_work;
	struct syna_tcm *tcm;

	delayed_work = container_of(work, struct delayed_work, work);
	tcm = container_of(delayed_work, struct syna_tcm, reflash_work);

	syna_pal_completion_wait_for(&tcm->init_completed, 1000);
	if (!tcm->init_done) {
		LOGE("Initialization not completed yet\n");
		return;
	}

	pm_stay_awake(&tcm->pdev->dev);

	retval = syna_dev_do_reflash(tcm, false);
	if (retval < 0)
		goto exit;

	/* re-initialize the app fw */
	retval = syna_dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("Fail to set up app fw after fw update\n");
		goto exit;
	}

	/* ensure the settings of input device
	 * if needed, re-create a new input device
	 */
	retval = syna_dev_set_up_input_device(tcm);
	if (retval < 0) {
		LOGE("Fail to register input device\n");
		goto exit;
	}
exit:
	pm_relax(&tcm->pdev->dev);
}
#endif /* end of STARTUP_REFLASH */
#endif

#if defined(LOW_POWER_MODE)
/*
 * Enable or disable the low power gesture mode.
 *
 * param
 *    [ in] tcm:           tcm driver handle
 *    [ in] en:            '1' to enable low power gesture mode; '0' to disable
 *    [ in] resp_handling: set up the handling of response to command
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_enable_lowpwr_gesture(struct syna_tcm *tcm, bool en,
	unsigned int resp_handling)
{
	int retval = 0;
	unsigned short config;
	struct syna_hw_attn_data *attn;

	if (!tcm)
		return -EINVAL;

	attn = &tcm->hw_if->bdata_attn;

	if (!tcm->lpwg_enabled)
		return 0;

	if (en) {
		if (!tcm->irq_wake) {
			enable_irq_wake(attn->irq_id);
			tcm->irq_wake = true;
		}
		config = 1;
	} else {
		if (tcm->irq_wake) {
			disable_irq_wake(attn->irq_id);
			tcm->irq_wake = false;
		}
		config = 0;
	}

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_WAKEUP_GESTURE_MODE,
			config,
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to %s wakeup gesture via dynamic config command\n",
			(en) ? "enable" : "disable");
		return retval;
	}

	return retval;
}
#endif
#if defined(LOW_POWER_MODE) && !defined(RESET_ON_RESUME)
/*
 * Enter normal sensing mode
 *
 * param
 *    [ in] tcm: tcm driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_enter_normal_sensing(struct syna_tcm *tcm)
{
	int retval = 0;
	struct syna_hw_attn_data *attn;
	struct tcm_dev *tcm_dev;
	unsigned int resp_handling;

	if (!tcm)
		return -EINVAL;

	tcm_dev = tcm->tcm_dev;
	attn = &tcm->hw_if->bdata_attn;
	if ((attn->irq_id) && (attn->irq_enabled))
		resp_handling = CMD_RESPONSE_IN_ATTN;
	else
		resp_handling = tcm_dev->msg_data.command_polling_time;

	/* bring out of sleep mode. */
	retval = syna_tcm_sleep(tcm->tcm_dev, false, resp_handling);
	if (retval < 0) {
		LOGE("Fail to exit deep sleep\n");
		return retval;
	}

	/* disable low power gesture mode, if needed */
	if (tcm->lpwg_enabled) {
		retval = syna_dev_enable_lowpwr_gesture(tcm, false, resp_handling);
		if (retval < 0) {
			LOGE("Fail to disable low power gesture mode\n");
			return retval;
		}
	}

	return 0;
}
#endif
#ifdef LOW_POWER_MODE
/*
 * Enter low-power saving mode, that could be lower power gesture
 * mode or deep sleep mode.
 *
 * param
 *    [ in] tcm: tcm driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_enter_lowpwr_sensing(struct syna_tcm *tcm)
{
	int retval = 0;
	struct syna_hw_attn_data *attn;
	struct tcm_dev *tcm_dev;
	unsigned int resp_handling;

	if (!tcm)
		return -EINVAL;

	tcm_dev = tcm->tcm_dev;
	attn = &tcm->hw_if->bdata_attn;
	if ((attn->irq_id) && (attn->irq_enabled))
		resp_handling = CMD_RESPONSE_IN_ATTN;
	else
		resp_handling = tcm_dev->msg_data.command_polling_time;

	/* enable low power gesture mode, if needed */
	if (tcm->lpwg_enabled) {
		retval = syna_dev_enable_lowpwr_gesture(tcm, true, resp_handling);
		if (retval < 0) {
			LOGE("Fail to disable low power gesture mode\n");
			return retval;
		}
	} else {
	/* enter sleep mode for non-LPWG cases */
		if (!tcm->slept_in_early_suspend) {
			retval = syna_tcm_sleep(tcm->tcm_dev, true, resp_handling);
			if (retval < 0) {
				LOGE("Fail to enter deep sleep\n");
				return retval;
			}
		}
	}

	return 0;
}
#endif
#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_BRIDGE)
/*
 * Put device into suspend state.
 * This function is supposed to be invoked through DRM_Bridge framework.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_panel_suspend(struct device *dev)
{
	struct syna_tcm *tcm = dev_get_drvdata(dev);

	return tcm->dev_suspend(dev);
}
/*
 * Resume from the suspend state.
 * This function is supposed to be invoked through DRM_Bridge framework.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_panel_resume(struct device *dev)
{
	struct syna_tcm *tcm = dev_get_drvdata(dev);

	return tcm->dev_resume(dev);
}
#endif
#if defined(USE_FB)
/*
 * Receive the early suspend event from the display.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_early_suspend(struct device *dev)
{
	int retval;
	struct syna_tcm *tcm = dev_get_drvdata(dev);

	/* exit directly if device is already in suspend state */
	if (tcm->pwr_state != PWR_ON)
		return 0;

	if (!tcm->lpwg_enabled) {
		retval = syna_tcm_sleep(tcm->tcm_dev, true);
		if (retval < 0) {
			LOGE("Fail to enter deep sleep\n");
			return retval;
		}
	}

	tcm->slept_in_early_suspend = true;

	return 0;
}
/*
 * Catch the screen on/off event from display.
 *
 * param
 *    [ in] nb:     pointer to notifier_block
 *    [ in] action: fb action
 *    [ in] data:   private data for callback
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_disp_notifier_cb(struct notifier_block *nb,
	unsigned long action, void *data)
{
	int retval;
	int transition;
	struct fb_event *evdata = data;
	struct syna_tcm *tcm = container_of(nb, struct syna_tcm, fb_notifier);
	int time = 0;
	int disp_blank_powerdown;
	int disp_early_event_blank;
	int disp_blank;
	int disp_blank_unblank;

	if (!evdata || !evdata->data || !tcm)
		return 0;

	retval = 0;

	disp_blank_powerdown = FB_BLANK_POWERDOWN;
	disp_early_event_blank = FB_EARLY_EVENT_BLANK;
	disp_blank = FB_EVENT_BLANK;
	disp_blank_unblank = FB_BLANK_UNBLANK;

	transition = *(int *)evdata->data;

	/* confirm the firmware flashing is completed before screen off */
	if (transition == disp_blank_powerdown) {
		while (ATOMIC_GET(tcm->tcm_dev->firmware_flashing)) {
			syna_pal_sleep_ms(500);

			time += 500;
			if (time >= 5000) {
				LOGE("Timed out waiting for re-flashing\n");
				ATOMIC_SET(tcm->tcm_dev->firmware_flashing, 0);
				return -ETIMEDOUT;
			}
		}
	}

	if (action == disp_early_event_blank &&
		transition == disp_blank_powerdown) {
		retval = syna_dev_early_suspend(&tcm->pdev->dev);
	} else if (action == disp_blank) {
		if (transition == disp_blank_powerdown) {
			retval = tcm->dev_suspend(&tcm->pdev->dev);
			tcm->fb_ready = 0;
		} else if (transition == disp_blank_unblank) {
#ifndef RESUME_EARLY_UNBLANK
			retval = tcm->dev_resume(&tcm->pdev->dev);
			tcm->fb_ready++;
#endif
		} else if (action == disp_early_event_blank &&
			transition == disp_blank_unblank) {
#ifdef RESUME_EARLY_UNBLANK
			retval = tcm->dev_resume(&tcm->pdev->dev);
			tcm->fb_ready++;
#endif
		}
	}

	return 0;
}
#endif
#endif
/*
 * Resume from the suspend state.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_resume(struct device *dev)
{
	int retval;
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct tcm_dev *tcm_dev;
	struct syna_hw_interface *hw_if;
	struct syna_hw_attn_data *attn;
	unsigned int resp_handling;
#ifdef RESET_ON_RESUME
	unsigned char status;
#endif

	if (!tcm)
		return -EINVAL;

	tcm_dev = tcm->tcm_dev;
	hw_if = tcm->hw_if;
	attn = &hw_if->bdata_attn;
	if (attn && (attn->irq_id) && (attn->irq_enabled))
		resp_handling = CMD_RESPONSE_IN_ATTN;
	else
		resp_handling = tcm_dev->msg_data.command_polling_time;

	/* exit directly if device isn't in suspend state */
	if (tcm->pwr_state == PWR_ON)
		return 0;

	LOGI("Prepare to resume device\n");


	/* clear all input events  */
	syna_dev_free_input_events(tcm);

#ifdef RESET_ON_RESUME
	LOGI("Do reset on resume\n");
	syna_pal_sleep_ms(RESET_ON_RESUME_DELAY_MS);

	if (hw_if->ops_hw_reset) {
		hw_if->ops_hw_reset(hw_if);

		/* manually read in the event after reset if attn is disabled */
		if (!attn->irq_enabled) {
			retval = syna_tcm_get_event_data(tcm->tcm_dev,
				&status, NULL);
			if ((retval < 0) || (status != REPORT_IDENTIFY)) {
				LOGE("Fail to complete hw reset\n");
				goto exit;
			}
		}

	} else {
		retval = syna_tcm_reset(tcm->tcm_dev, resp_handling);
		if (retval < 0) {
			LOGE("Fail to do sw reset\n");
			goto exit;
		}
	}
#else
#ifdef LOW_POWER_MODE
	/* enter normal power mode */
	retval = syna_dev_enter_normal_sensing(tcm);
	if (retval < 0) {
		LOGE("Fail to enter normal power mode\n");
		goto exit;
	}
#endif
	retval = syna_tcm_rezero(tcm->tcm_dev, resp_handling);
	if (retval < 0) {
		LOGE("Fail to rezero\n");
		goto exit;
	}
#endif
	tcm->pwr_state = PWR_ON;

	LOGI("Prepare to set up application firmware\n");

	/* set up app firmware */
	retval = syna_dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("Fail to set up app firmware on resume\n");
		goto exit;
	}

	retval = 0;

	LOGI("Device resumed (pwr_state:%d)\n", tcm->pwr_state);

exit:
	/* enable irq */
	if ((!attn->irq_enabled) && (hw_if->hw_platform.ops_enable_attn))
		hw_if->hw_platform.ops_enable_attn(&hw_if->hw_platform, true);

	tcm->slept_in_early_suspend = false;

	return retval;
}
/*
 * Put device into suspend state, that could be the lower power gesture
 * mode or sleep mode.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_suspend(struct device *dev)
{
	struct syna_tcm *tcm = dev_get_drvdata(dev);
	struct syna_hw_interface *hw_if = tcm->hw_if;

	/* exit directly if device is already in suspend state */
	if (tcm->pwr_state != PWR_ON)
		return 0;

	LOGI("Prepare to suspend device\n");

	/* clear all input events  */
	syna_dev_free_input_events(tcm);

#ifdef LOW_POWER_MODE
	/* enter power saved mode if power is not off */
	if (syna_dev_enter_lowpwr_sensing(tcm) < 0) {
		LOGE("Fail to enter power suspended mode\n");
		return -EIO;
	}
	tcm->pwr_state = LOW_PWR;
#else
	tcm->pwr_state = PWR_OFF;
#endif

	syna_tcm_clear_command_processing(tcm->tcm_dev);

	/* once lpwg is enabled, irq should be alive.
	 * otherwise, disable irq in suspend.
	 */
	if ((!tcm->lpwg_enabled) && (hw_if->hw_platform.ops_enable_attn))
		hw_if->hw_platform.ops_enable_attn(&hw_if->hw_platform, false);

	LOGI("Device suspended (pwr_state:%d)\n", tcm->pwr_state);

	return 0;
}
/*
 * Output the device information.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    void.
 */
static void syna_dev_show_info(struct syna_tcm *tcm)
{
	struct syna_hw_interface *hw_if = tcm->hw_if;
	bool has_custom_tp_config = false;
	bool startup_reflash_enabled = false;
	bool rst_on_resume_enabled = false;
	bool background_helper_enabled  = false;

	if (!tcm->is_connected)
		return;

	LOGI("Config: max. write size(%d), max. read size(%d)\n",
		tcm->tcm_dev->max_wr_size, tcm->tcm_dev->max_rd_size);

#ifdef USE_CUSTOM_TOUCH_REPORT_CONFIG
	has_custom_tp_config = true;
#endif
#ifdef STARTUP_REFLASH
	startup_reflash_enabled = true;
#endif
#ifdef RESET_ON_RESUME
	rst_on_resume_enabled = true;
#endif
#ifdef ENABLE_HELPER
	background_helper_enabled = true;
#endif

#ifdef TOUCHCOMM_TDDI
	LOGI("Config: touch/display devices, multichip(%s)\n",
		(tcm->is_tddi_multichip) ? "yes" : "no");
#endif
	LOGI("Config: startup reflash(%s), hw reset(%s), rst on resume(%s)\n",
		(startup_reflash_enabled) ? "yes" : "no",
		(hw_if->ops_hw_reset) ? "yes" : "no",
		(rst_on_resume_enabled) ? "yes" : "no");
	LOGI("Config: lpwg mode(%s), custom tp config(%s) helper work(%s)\n",
		(tcm->lpwg_enabled) ? "yes" : "no",
		(has_custom_tp_config) ? "yes" : "no",
		(background_helper_enabled) ? "yes" : "no");
}

/*
 * Disconnect and power off the device.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_disconnect(struct syna_tcm *tcm)
{
	struct syna_hw_interface *hw_if = tcm->hw_if;

	if (tcm->is_connected == false) {
		LOGI("%s already disconnected\n", PLATFORM_DRIVER_NAME);
		return 0;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGI("Disconnect from bare mode\n");
		goto exit;
	}

#ifdef STARTUP_REFLASH
	if (tcm->reflash_workqueue) {
		cancel_delayed_work_sync(&tcm->reflash_work);
		flush_workqueue(tcm->reflash_workqueue);
		destroy_workqueue(tcm->reflash_workqueue);
		tcm->reflash_workqueue = NULL;
	}
#endif

	/* free interrupt line */
	if (hw_if->bdata_attn.irq_id)
		syna_dev_release_irq(tcm);

	/* unregister input device */
#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
	syna_dev_release_input_knob_device(tcm);
#endif
	syna_dev_release_input_device(tcm);

	tcm->input_dev_params.max_x = 0;
	tcm->input_dev_params.max_y = 0;
	tcm->input_dev_params.max_objects = 0;

exit:
	/* power off */
	if (hw_if->ops_power_on)
		hw_if->ops_power_on(hw_if, false);

	tcm->pwr_state = PWR_OFF;
	tcm->is_connected = false;

	LOGI("Device %s disconnected\n", PLATFORM_DRIVER_NAME);

	return 0;
}

/*
 * Connect and power on the device.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_connect(struct syna_tcm *tcm)
{
	int retval;
	struct syna_hw_interface *hw_if = tcm->hw_if;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;

	if (!tcm_dev) {
		LOGE("Invalid tcm_dev\n");
		return -EINVAL;
	}

	if (tcm->is_connected) {
		LOGI("Device %s already connected\n", PLATFORM_DRIVER_NAME);
		return 0;
	}

	/* power on the connected device */
	if (hw_if->ops_power_on) {
		retval = hw_if->ops_power_on(hw_if, true);
		if (retval < 0)
			return -ENODEV;
		if (hw_if->bdata_pwr.power_delay_ms > 0)
			syna_pal_sleep_ms(hw_if->bdata_pwr.power_delay_ms);
	}

#ifdef RESET_ON_CONNECT
	/* perform a hardware reset */
	if (hw_if->ops_hw_reset)
		hw_if->ops_hw_reset(hw_if);
#endif

	/* detect which modes of touch controller is running */
#if defined(TOUCHCOMM_VERSION_1)
	retval = syna_tcm_detect_device(tcm->tcm_dev, PROTOCOL_DETECT_VERSION_1, false);
#elif defined(TOUCHCOMM_VERSION_2)
	retval = syna_tcm_detect_device(tcm->tcm_dev, PROTOCOL_DETECT_VERSION_2, false);
#else
	LOGW("TouchComm protocol is not specified, switch to Bare mode.\n");
	/* 'Bare' mode is a special software mode to bypass all control from userspace */
	tcm->pwr_state = BARE_MODE;
	LOGI("Device %s config into bare mode\n", PLATFORM_DRIVER_NAME);
	return 0;
#endif
	if (retval < 0) {
		LOGE("Fail to detect the device\n");
		goto err_detect_dev;
	}

	if (tcm_dev->dev_mode == MODE_APPLICATION_FIRMWARE) {
		retval = syna_dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up application firmware\n");

			/* switch to bootloader mode when failed */
			LOGI("Switch device to bootloader mode instead\n");
			syna_tcm_switch_fw_mode(tcm_dev, MODE_BOOTLOADER,
					tcm_dev->fw_mode_switching_time);
		} else {
			/* allocate and register to input device subsystem */
			retval = syna_dev_set_up_input_device(tcm);
			if (retval < 0) {
				LOGE("Fail to set up input device\n");
				goto err_setup_input_dev;
			}
#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
			retval = syna_dev_create_input_knob_device(tcm);
			if (retval < 0) {
				LOGE("Fail to set up input device for knob\n");
				goto err_setup_input_dev;
			}
#endif
		}
	} else {
		LOGN("Application firmware not running, current mode: %02x\n", tcm_dev->dev_mode);

		if (tcm_dev->dev_mode == MODE_BOOTLOADER) {
			retval = syna_tcm_get_boot_info(tcm_dev, NULL, CMD_RESPONSE_IN_POLLING);
			if (retval)
				LOGI("Bootloader status: 0x%x\n", tcm_dev->boot_info.status);
		}
	}

	/* register the handling of report resulting from the unexpected reset */
	retval = syna_tcm_set_report_dispatcher(tcm_dev,
			REPORT_IDENTIFY, syna_dev_process_unexpected_reset, (void *)tcm);
	if (retval < 0)
		LOGE("Fail to register the handling function of unexpected reset\n");

	/* register the interrupt handler */
	retval = syna_dev_request_irq(tcm);
	if (retval < 0) {
		LOGE("Fail to request the interrupt line\n");
		goto err_request_irq;
	}

	/* for the reference,
	 * create a delayed work to perform fw update during the startup time
	 */
#ifdef STARTUP_REFLASH
	tcm->reflash_workqueue = create_singlethread_workqueue("syna_reflash");
	INIT_DELAYED_WORK(&tcm->reflash_work, syna_dev_reflash_startup_work);
	queue_delayed_work(tcm->reflash_workqueue, &tcm->reflash_work, msecs_to_jiffies(200));
#endif

	tcm->pwr_state = PWR_ON;
	tcm->is_connected = true;

	syna_dev_show_info(tcm);

	LOGI("Device %s connected\n", PLATFORM_DRIVER_NAME);

	return 0;

err_request_irq:
	/* unregister input device */
	syna_dev_release_input_device(tcm);
#if defined(TOUCHCOMM_TDDI) && defined(REPORT_KNOB)
	syna_dev_release_input_knob_device(tcm);
#endif
err_setup_input_dev:
err_detect_dev:
	return retval;
}

#ifdef USE_DRM_BRIDGE
/*
 * To register a panel bridge based on DAM Bridge framework
 */
struct drm_connector *syna_dev_get_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static void syna_dev_panel_enable(struct drm_bridge *bridge)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	LOGD("Panel bridge enabled (pwr_state:%d)\n", tcm->pwr_state);
}

static void syna_dev_panel_disable(struct drm_bridge *bridge)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	LOGD("Panel bridge disabled (pwr_state:%d)\n", tcm->pwr_state);
}

static void syna_dev_panel_mode_set(struct drm_bridge *bridge,
	const struct drm_display_mode *mode,
	const struct drm_display_mode *adjusted_mode)
{
	struct syna_tcm *tcm =
			container_of(bridge, struct syna_tcm, panel_bridge);

	if (!tcm->connector || !tcm->connector->state) {
		LOGI("Get bridge connector.\n");
		tcm->connector = syna_dev_get_connector(bridge);
	}
	LOGD("Panel bridge mode set (pwr_state:%d)\n", tcm->pwr_state);
}

static const struct drm_bridge_funcs panel_bridge_ops = {
	.enable = syna_dev_panel_enable,
	.disable = syna_dev_panel_disable,
	.mode_set = syna_dev_panel_mode_set,
};

static int syna_dev_register_panel(struct syna_tcm *tcm)
{
#ifdef CONFIG_OF
	tcm->panel_bridge.of_node = tcm->pdev->dev.parent->of_node;
#endif
	tcm->panel_bridge.funcs = &panel_bridge_ops;
	drm_bridge_add(&tcm->panel_bridge);

	return 0;
}

static void syna_dev_unregister_panel(struct drm_bridge *bridge)
{
	struct drm_bridge *node;

	drm_bridge_remove(bridge);

	if (!bridge->dev)
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node)
		if (node == bridge) {
			if (bridge->funcs->detach)
				bridge->funcs->detach(bridge);
			list_del(&bridge->chain_node);
			break;
		}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}
#endif

/*
 * Probe of TouchComm device driver.
 *
 * param
 *    [ in] pdev: pointer to platform device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_dev_probe(struct platform_device *pdev)
{
	int retval;
	struct syna_tcm *tcm = NULL;
	struct tcm_dev *tcm_dev = NULL;
	struct syna_hw_interface *hw_if = NULL;
	struct tcm_timings timings;

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		LOGE("Fail to find hardware configuration\n");
		return -EINVAL;
	}

	tcm = syna_pal_mem_alloc(1, sizeof(struct syna_tcm));
	if (!tcm) {
		LOGE("Fail to create the handle of syna_tcm\n");
		return -ENOMEM;
	}

	syna_pal_completion_alloc(&tcm->init_completed);

	/* allocate the TouchCom device handle */
	retval = syna_tcm_allocate_device(&tcm_dev,
		&hw_if->hw_platform, (void *)tcm);
	if ((retval < 0) || (!tcm_dev)) {
		LOGE("Fail to allocate TouchCom device handle\n");
		goto err_allocate_tcm;
	}

	tcm->tcm_dev = tcm_dev;
	tcm->pdev = pdev;
	tcm->hw_if = hw_if;

	/* configure the timings for processing */
	timings.cmd_timeout_ms = hw_if->product.default_cmd_timeout_ms;
	timings.cmd_polling_ms = hw_if->product.default_cmd_polling_ms;
	timings.cmd_turnaround_us[0] = hw_if->product.default_cmd_turnaround_us[0];
	timings.cmd_turnaround_us[1] = hw_if->product.default_cmd_turnaround_us[1];
	timings.cmd_retry_us[0] = hw_if->product.default_cmd_retry_us[0];
	timings.cmd_retry_us[1] = hw_if->product.default_cmd_retry_us[1];
	timings.flash_ops_delay_us[0] = hw_if->product.default_flash_delay_us[0];
	timings.flash_ops_delay_us[1] = hw_if->product.default_flash_delay_us[1];
	timings.flash_ops_delay_us[2] = hw_if->product.default_flash_delay_us[2];
	timings.fw_switch_delay_ms = hw_if->product.default_fw_switch_delay_ms;
	timings.reset_delay_ms = hw_if->bdata_rst.reset_delay_ms;

	retval = syna_tcm_config_timings(tcm_dev, &timings, 0, TIMINGS_ALL);
	if (retval < 0) {
		LOGE("Fail to config the timings\n");
		goto err_setup_timings;
	}

	/* basic initialization */
	syna_tcm_buf_init(&tcm->event_data);

	syna_pal_mutex_alloc(&tcm->tp_event_mutex);

#ifdef ENABLE_WAKEUP_GESTURE
	tcm->lpwg_enabled = true;
#else
	tcm->lpwg_enabled = false;
#endif
#ifdef TOUCHCOMM_TDDI
#ifdef IS_TDDI_MULTICHIP
	tcm->is_tddi_multichip = true;
#else
	tcm->is_tddi_multichip = false;
#endif
#endif
	tcm->irq_wake = false;

	tcm->is_connected = false;
	tcm->pwr_state = PWR_OFF;

	tcm->dev_connect = syna_dev_connect;
	tcm->dev_disconnect = syna_dev_disconnect;
	tcm->dev_set_up_app_fw = syna_dev_set_up_app_fw;
	tcm->dev_resume = syna_dev_resume;
	tcm->dev_suspend = syna_dev_suspend;

	tcm->userspace_app_info = NULL;

	platform_set_drvdata(pdev, tcm);

	device_init_wakeup(&pdev->dev, 1);

	/* connect to target device */
	retval = syna_dev_connect(tcm);
	if (retval < 0) {
#ifdef FORCE_CONNECTION
		LOGW("Failed on device detection\n");
		LOGN("Install driver anyway due to the force connection\n");
#else
		LOGE("Fail to connect to the device\n");
		syna_pal_mutex_free(&tcm->tp_event_mutex);
		goto err_connect;
#endif
	}

	/* create the device file and register to char device classes */
	retval = syna_cdev_create(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create the device sysfs\n");
		syna_pal_mutex_free(&tcm->tp_event_mutex);
		goto err_create_cdev;
	}

#ifdef HAS_SYSFS_INTERFACE
	retval = syna_sysfs_create_dir(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create sysfs dir\n");
		retval = -ENOTDIR;
		goto err_create_dir;
	}
#endif
#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_BRIDGE)
	retval = syna_dev_register_panel(tcm);
	if (retval < 0) {
		LOGE("Fail to register panel bridge\n");
		goto err_disp_notifier;
	}
#endif
#if defined(USE_FB)
	tcm->fb_notifier.notifier_call = syna_dev_disp_notifier_cb;
	retval = fb_register_client(&tcm->fb_notifier);
	if (retval < 0) {
		LOGE("Fail to register FB notifier client\n");
		goto err_disp_notifier;
	}
#endif
#endif

#if defined(ENABLE_HELPER)
	ATOMIC_SET(tcm->helper.task, HELP_NONE);
	tcm->helper.workqueue =
			create_singlethread_workqueue("synaptics_tcm_helper");
	INIT_WORK(&tcm->helper.work, syna_dev_helper_work);
#endif

	LOGI("TouchComm driver, %s ver.: %d.%s, installed\n",
		PLATFORM_DRIVER_NAME,
		SYNAPTICS_TCM_DRIVER_VERSION,
		SYNAPTICS_TCM_DRIVER_SUBVER);

	tcm->init_done = true;
	syna_pal_completion_complete(&tcm->init_completed);

	return 0;

#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_BRIDGE) || defined(USE_FB)
err_disp_notifier:
#endif
#endif
#ifdef HAS_SYSFS_INTERFACE
err_create_dir:
	syna_cdev_remove(tcm);
#endif
err_create_cdev:
	syna_dev_disconnect(tcm);
#ifndef FORCE_CONNECTION
err_connect:
#endif
	syna_tcm_buf_release(&tcm->event_data);
	syna_pal_mutex_free(&tcm->tp_event_mutex);
err_setup_timings:
	syna_tcm_remove_device(tcm_dev);
err_allocate_tcm:
	syna_pal_mem_free((void *)tcm);
	syna_pal_completion_free(&tcm->init_completed);

	return retval;
}

/*
 * Release all resources allocated previously.
 *
 * param
 *    [ in] pdev: pointer to platform device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static void syna_dev_remove(struct platform_device *pdev)
{
	struct syna_tcm *tcm = platform_get_drvdata(pdev);

	if (!tcm) {
		LOGW("Invalid handle to remove\n");
	}

#if defined(ENABLE_HELPER)
	cancel_work_sync(&tcm->helper.work);
	flush_workqueue(tcm->helper.workqueue);
	destroy_workqueue(tcm->helper.workqueue);
#endif
#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_DRM_BRIDGE)
	syna_dev_unregister_panel(&tcm->panel_bridge);
#else
	fb_unregister_client(&tcm->fb_notifier);
#endif
#endif

#ifdef HAS_SYSFS_INTERFACE
	syna_sysfs_remove_dir(tcm);
#endif
	/* remove the cdev and sysfs nodes */
	syna_cdev_remove(tcm);

	/* check the connection status, and do disconnection */
	if (syna_dev_disconnect(tcm) < 0)
		LOGE("Fail to do device disconnection\n");

	if (tcm->userspace_app_info != NULL)
		syna_pal_mem_free(tcm->userspace_app_info);

	syna_tcm_buf_release(&tcm->event_data);
	syna_pal_mutex_free(&tcm->tp_event_mutex);

	/* remove the allocated tcm device */
	syna_tcm_remove_device(tcm->tcm_dev);

	/* release the device context */
	syna_pal_completion_free(&tcm->init_completed);
	syna_pal_mem_free((void *)tcm);
}

/*
 * Release all resources.
 *
 * param
 *    [in] pdev: pointer to platform device
 *
 * return
 *    void.
 */
static void syna_dev_shutdown(struct platform_device *pdev)
{
	syna_dev_remove(pdev);
}


/* Definitions of TouchComm platform device */
#ifdef CONFIG_PM
static const struct dev_pm_ops syna_dev_pm_ops = {
#if defined(USE_DRM_BRIDGE)
	.suspend = syna_dev_panel_suspend,
	.resume = syna_dev_panel_resume,
#elif !defined(ENABLE_DISP_NOTIFIER)
	.suspend = syna_dev_suspend,
	.resume = syna_dev_resume,
#endif
};
#endif

static struct platform_driver syna_dev_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &syna_dev_pm_ops,
#endif
	},
	.probe = syna_dev_probe,
	.remove = syna_dev_remove,
	.shutdown = syna_dev_shutdown,
};



/*
 * Entry of the TouchComm device driver.
 */

static int __init syna_dev_module_init(void)
{
	int retval;

	retval = syna_hw_interface_init();
	if (retval < 0)
		return retval;

	return platform_driver_register(&syna_dev_driver);
}

static void __exit syna_dev_module_exit(void)
{
	platform_driver_unregister(&syna_dev_driver);

	syna_hw_interface_exit();
}

module_init(syna_dev_module_init);
module_exit(syna_dev_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TouchComm Touch Driver");
MODULE_LICENSE("GPL v2");

