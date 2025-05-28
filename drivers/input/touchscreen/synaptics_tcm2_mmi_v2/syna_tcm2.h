/* SPDX-License-Identifier: GPL-2.0
 *
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
 * The header file for the Synaptics TouchComm reference driver.
 */

#ifndef _SYNAPTICS_TCM2_DRIVER_H_
#define _SYNAPTICS_TCM2_DRIVER_H_

#include "syna_tcm2_platform.h"
#include "tcm/synaptics_touchcom_core_dev.h"
#include "tcm/synaptics_touchcom_func_base.h"
#include <linux/pinctrl/consumer.h>

#define PLATFORM_DRIVER_NAME "synaptics_tcm"

#define TOUCH_INPUT_NAME "synaptics_tcm_touch"
#define TOUCH_INPUT_PHYS_PATH "synaptics_tcm/touch_input"

#define CHAR_DEVICE_NAME "tcm"
#define CHAR_DEVICE_MODE (0x0600)

#define SYNAPTICS_TCM_DRIVER_ID (1 << 0)
#define SYNAPTICS_TCM_DRIVER_VERSION 1
#define SYNAPTICS_TCM_DRIVER_SUBVER "7.1"

#define PINCTRL_STYLUS_CLK_ACTIVE       "stylus_clk_active"
#define PINCTRL_STYLUS_CLK_SUSPEND      "stylus_clk_suspend"


/*
 * Modules Configurations
 */

/* HAS_SYSFS_INTERFACE
 *         Open to enable the sysfs kernel attributes.
 *         Typically, it's aligned with the deconfig, CONFIG_TOUCHSCREEN_SYNA_TCM2_SYSFS
 */
//#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_SYSFS)
#define HAS_SYSFS_INTERFACE
//#endif

/* HAS_REFLASH_FEATURE
 *         Open to enable firmware reflash features.
 *         Typically, it's aligned with the deconfig, CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH
 */
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH)
#define HAS_REFLASH_FEATURE
#ifdef TOUCHCOMM_TDDI
#define REFLASH_TDDI
#else
#define REFLASH_DISCRETE_TOUCH
#endif
#endif
/* HAS_TESTING_FEATURE
 *         Open to enable testing features.
 *         Typically, it's aligned with the deconfig, CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING
 */
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING)
#define HAS_TESTING_FEATURE
#endif


/*
 * Driver Configurations
 */

/* TYPE_B_PROTOCOL
 *         Open to enable the multi-touch (MT) protocol
 */
#define TYPE_B_PROTOCOL

/* RESET_ON_CONNECT
 *         Open if willing to issue a reset when connecting to the
 *         touch controller. Set "enable" in default.
 */
#define RESET_ON_CONNECT

/* RESET_ON_RESUME
 *         Open if willing to issue a reset to the touch controller
 *         from suspend. Set "disable" in default.
 */
/* #define RESET_ON_RESUME */

/* LOW_POWER_MODE
 *         Open if willing to enter the lower power mode when the system
 *         going to the suspend mode; otherwise, expected that no power
 *         supplied. Set "enable" in default.
 */
#define LOW_POWER_MODE

#if defined(LOW_POWER_MODE)
/* ENABLE_WAKEUP_GESTURE
 *         Open if having wake-up gesture support.
 */
/* #define ENABLE_WAKEUP_GESTURE */
#endif

/* REPORT_SWAP_XY
 *  REPORT_FLIP_X
 *  REPORT_FLIP_Y
 *         Open if willing to modify the touch data before sending to the
 *         input event subsystem. Set "disable" in default.
 */
/* #define REPORT_SWAP_XY */
/* #define REPORT_FLIP_X */
/* #define REPORT_FLIP_Y */

/*  REPORT_TOUCH_WIDTH
 *         Open if willing to add the width data to the input event.
 */
#define REPORT_TOUCH_WIDTH

#if defined(TOUCHCOMM_TDDI)
/*  REPORT_KNOB
 *         Open if willing to add the knob data to the input event.
 *
 *  HAVE_THE_SECOND_KNOB
 *         Open if willing to support two knob events
 */
#define REPORT_KNOB

#ifdef REPORT_KNOB
#define KNOB_INPUT_NAME "synaptics_tcm_knob"
#define KNOB_INPUT_PHYS_PATH "synaptics_tcm/knob_input"
#endif

/* #define HAVE_THE_SECOND_KNOB */

#endif

/* USE_CUSTOM_TOUCH_REPORT_CONFIG
 *         Open if willing to set up the format of touch report.
 *         The custom_touch_format[] array in syna_tcm2.c can be used
 *         to describe the customized report format.
 */
/* #define USE_CUSTOM_TOUCH_REPORT_CONFIG */

/* ENABLE_CUSTOM_TOUCH_ENTITY
 *         Open if having the requirements to parse the custom touch code entity.
 */
/* #define ENABLE_CUSTOM_TOUCH_ENTITY */

/* STARTUP_REFLASH
 *         Open if willing to do fw checking and update at startup.
 *         The firmware image will be obtained by request_firmware() API,
 *         so please ensure the image is built-in or included properly.
 */
#if defined(HAS_REFLASH_FEATURE)
/* #define STARTUP_REFLASH */

#define FW_IMAGE_NAME "synaptics/firmware.img"
#endif

/* ENABLE_DISP_NOTIFIER
 *         Open if having display notification event and willing to listen
 *         the event from display driver.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(CONFIG_FB) || defined(CONFIG_DRM_BRIDGE)
/* #define ENABLE_DISP_NOTIFIER */
#endif
/* USE_FB
 *         Open if having the support of FB (Frame Buffer) and willing to listen
 *         the event from display driver.
 *         This property is available only when CONFIG_FB in used
 */
#if defined(ENABLE_DISP_NOTIFIER) && defined(CONFIG_FB)
#define USE_FB
#endif
/* RESUME_EARLY_UNBLANK
 *         Open if willing to resume in early un-blanking state.
 *         This property is available only when ENABLE_DISP_NOTIFIER
 *         feature is enabled.
 */
#ifdef ENABLE_DISP_NOTIFIER
/* #define RESUME_EARLY_UNBLANK */
#endif
/* USE_DRM_BRIDGE
 *         Open if having the support of DRM bridge and willing to listen
 *         the event from display driver.
 *         This property is available only when CONFIG_DRM_BRIDGE in used
 */
#if defined(ENABLE_DISP_NOTIFIER) && defined(CONFIG_DRM_BRIDGE)
#define USE_DRM_BRIDGE
#endif

/* ENABLE_EXTERNAL_FRAME_PROCESS
 *         Open if willing to pass the data to the userspace application.
 */
#define ENABLE_EXTERNAL_FRAME_PROCESS

/* FORCE_CONNECTION
 *         Force to install the driver even though the occurrence of errors.
 */
#define FORCE_CONNECTION

/* ENABLE_HELPER
 *         Open if willing to do additional handling in the background workqueue.
 */
/* #define ENABLE_HELPER */


#if defined(TOUCHCOMM_TDDI)
/* IS_TDDI_MULTICHIP
 *         Indicate the TDDI multichip architecture
 */
/* #define IS_TDDI_MULTICHIP */
#endif



/*
 * Definitions of TouchComm device driver
 */


/* Enumeration of the power states */
enum power_state {
	PWR_OFF = 0,
	PWR_ON,
	LOW_PWR,
	BARE_MODE,
};

#if defined(ENABLE_HELPER)
/* Definitions of the background helper thread */
enum helper_task {
	HELP_NONE = 0,
	HELP_RESET_DETECTED,
};

struct syna_tcm_helper {
	syna_pal_atomic_t task;
	struct work_struct work;
	struct workqueue_struct *workqueue;
};
#endif

/*
 * Context of Synaptics TouchComm device driver
 *
 * The structure defines the kernel specific data and the essentials
 * for the device driver.
 */
struct syna_tcm {

	/* Context for the use of TouchComm core library */
	struct tcm_dev *tcm_dev;

	/* Pointer to platform device */
	struct platform_device *pdev;

	/* Stuff related to touch data */
	struct tcm_touch_data_blob tp_data;
	unsigned char prev_obj_status[MAX_NUM_OBJECTS];

	/* Abstraction of hardware interface */
	struct syna_hw_interface *hw_if;

	/* Stuff related to irq event */
	syna_pal_mutex_t tp_event_mutex;
	struct tcm_buffer event_data;
	pid_t isr_pid;
	bool irq_wake;

	/* Stuff related to cdev interface */
	struct cdev char_dev;
	dev_t char_dev_num;
	int char_dev_ref_count;
	struct class *device_class;
	struct device *device;

//#if defined(HAS_SYSFS_INTERFACE)
	/* Stuff related to sysfs attributes */
	struct kobject *sysfs_dir;
	struct kobject *sysfs_dbg_dir;
//#if defined(HAS_TESTING_FEATURE)
	struct kobject *sysfs_testing_dir;
//#endif
//#endif

	/* Stuff related to the registration of input device */
	struct input_dev *input_dev;
	struct input_params {
		unsigned int max_x;
		unsigned int max_y;
		unsigned int max_objects;
	} input_dev_params;
#ifdef REPORT_KNOB
	struct input_dev *input_knob_dev[MAX_NUM_KNOB_OBJECTS];
#endif

#if defined(STARTUP_REFLASH)
	/* Workqueue used for firmware update */
	struct delayed_work reflash_work;
	struct workqueue_struct *reflash_workqueue;
#endif

#if defined(ENABLE_DISP_NOTIFIER)
#if defined(USE_FB)
	struct notifier_block fb_notifier;
	unsigned char fb_ready;
#endif
#if defined(USE_DRM_BRIDGE)
	struct drm_bridge panel_bridge;
	struct drm_connector *connector;
	bool is_panel_lp_mode;
#endif
#endif

#if defined(ENABLE_EXTERNAL_FRAME_PROCESS)
	/* Kernel FIFO */
	unsigned int fifo_remaining_frame;
	struct list_head frame_fifo_queue;
	wait_queue_head_t wait_frame;
#endif

#if defined(ENABLE_HELPER)
	/* Background workqueue */
	struct syna_tcm_helper helper;
#endif

	/* Misc. variables */
	int pwr_state;
	bool slept_in_early_suspend;
	bool lpwg_enabled;
	bool is_connected;
	bool init_done;
#if defined(TOUCHCOMM_TDDI)
	bool is_tddi_multichip;
#endif
	bool concurrent_reporting;
	syna_pal_completion_t init_completed;

	struct pinctrl *pinctrl;
	struct pinctrl_state *stylus_clk_active;
	struct pinctrl_state *stylus_clk_suspend;

	/* Pointer of userspace application info data */
	void *userspace_app_info;

	/* Abstractions */
	int (*dev_connect)(struct syna_tcm *tcm);
	int (*dev_disconnect)(struct syna_tcm *tcm);
	int (*dev_set_up_app_fw)(struct syna_tcm *tcm);
	int (*dev_resume)(struct device *dev);
	int (*dev_suspend)(struct device *dev);
};

/*
 * Helpers for the registration of chardev nodes
 */
int syna_cdev_create(struct syna_tcm *ptcm, struct platform_device *pdev);
void syna_cdev_remove(struct syna_tcm *ptcm);

#if defined(REFLASH_DISCRETE_TOUCH) || defined(REFLASH_TDDI)
/*
 * Helper to perform firmware update
 */
int syna_dev_do_reflash(struct syna_tcm *tcm, bool force);
#endif

#ifdef HAS_SYSFS_INTERFACE
/*
 * Helpers for the registration of sysfs attributes.
 */
int syna_sysfs_create_dir(struct syna_tcm *tcm, struct platform_device *pdev);
void syna_sysfs_remove_dir(struct syna_tcm *tcm);

#ifdef HAS_TESTING_FEATURE
/*
 * Attributes for the example of production testing.
 */
int syna_testing_create_dir(struct syna_tcm *tcm);
void syna_testing_remove_dir(struct syna_tcm *tcm);
#endif

#endif

#endif /* end of _SYNAPTICS_TCM2_DRIVER_H_ */

