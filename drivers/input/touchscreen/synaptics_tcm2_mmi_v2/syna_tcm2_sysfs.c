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
 * This file implements sysfs attributes in the reference driver.
 */

#include <linux/string.h>

#include "syna_tcm2.h"
#include "tcm/synaptics_touchcom_core_dev.h"
#include "tcm/synaptics_touchcom_func_base.h"
#ifdef REFLASH_TDDI
#include "synaptics_touchcom_func_reflash_tddi.h"
#endif
#ifdef REFLASH_DISCRETE_TOUCH
#include "synaptics_touchcom_func_reflash.h"
#endif


#define SYSFS_ROOT_DIR "sysfs"
#define SYSFS_SUB_DIR "utility"

/*
 * Debugging attribute to issue a reset.
 * Input 1: for a sw reset
 *       2: for a hardware reset
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_reset_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct syna_tcm *tcm;
	struct syna_hw_attn_data *attn;
	unsigned char code;

	p_dev = container_of(kobj->parent->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	attn = &tcm->hw_if->bdata_attn;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if ((tcm->pwr_state == BARE_MODE) || (input == 2)) {
		if (!tcm->hw_if->ops_hw_reset) {
			LOGE("No hardware reset support\n");
			goto exit;
		}
		tcm->hw_if->ops_hw_reset(tcm->hw_if);
		/* manually read in the event after reset if attn is disabled */
		if (!attn->irq_enabled)
			syna_tcm_get_event_data(tcm->tcm_dev, &code, NULL);

	} else if (input == 1) {
		retval = syna_tcm_reset(tcm->tcm_dev,
			tcm->tcm_dev->msg_data.command_polling_time);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			goto exit;
		}
	} else {
		LOGW("Unknown option %d (1:sw / 2:hw)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	/* check the fw setup in case the settings is changed */
	if (IS_APP_FW_MODE(tcm->tcm_dev->dev_mode)) {
		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up app fw\n");
			goto exit;
		}
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_reset =
	__ATTR(reset, 0220, NULL, syna_sysfs_reset_store);

/*
 * Debugging attribute to disable/enable the irq
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_irq_en_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct syna_tcm *tcm;
	struct tcm_hw_platform *hw;

	p_dev = container_of(kobj->parent->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	hw = &tcm->hw_if->hw_platform;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!hw || !hw->ops_enable_attn)
		return 0;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no irq support\n");
		retval = count;
		goto exit;
	}

	/* disable the interrupt line */
	if (input == 0) {
		retval = hw->ops_enable_attn(hw, false);
		if (retval < 0) {
			LOGE("Fail to disable interrupt\n");
			goto exit;
		}
	} else if (input == 1) {
	/* enable the interrupt line */
		retval = hw->ops_enable_attn(hw, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			goto exit;
		}
	} else {
		LOGW("Unknown option %d (0:disable / 1:enable)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_irq_en =
	__ATTR(irq_en, 0220, NULL, syna_sysfs_irq_en_store);


/*
 * Debugging attribute to change the power state.
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_pwr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	struct device *p_dev;
	struct syna_tcm *tcm;

	p_dev = container_of(kobj->parent->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if (strncmp(buf, "resume", 6) == 0) {
		if (tcm->dev_resume)
			tcm->dev_resume(p_dev);
	} else if (strncmp(buf, "suspend", 7) == 0) {
		if (tcm->dev_suspend)
			tcm->dev_suspend(p_dev);
	} else {
		LOGW("Unknown option %s\n", buf);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_pwr =
	__ATTR(power_state, 0220, NULL, syna_sysfs_pwr_store);

#if defined(HAS_REFLASH_FEATURE)
/*
 * Debugging attribute to manually do firmware update.
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_fw_update_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	struct device *p_dev;
	struct syna_tcm *tcm;
	unsigned int input;

	p_dev = container_of(kobj->parent->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	retval = syna_dev_do_reflash(tcm, true);
	if (retval < 0) {
		LOGE("Fail to do reflash\n");
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_fw_update =
	__ATTR(fw_update, 0220, NULL, syna_sysfs_fw_update_store);
#endif

/* Definitions of debugging sysfs attributes */
static struct attribute *attrs_debug[] = {
	&kobj_attr_reset.attr,
	&kobj_attr_irq_en.attr,
	&kobj_attr_pwr.attr,
#if defined(HAS_REFLASH_FEATURE)
	&kobj_attr_fw_update.attr,
#endif
	NULL,
};

static struct attribute_group attr_debug_group = {
	.attrs = attrs_debug,
};

/*
 * Attribute to enable/disable the debugging attributes.
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_debug_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct syna_tcm *tcm;

	p_dev = container_of(kobj->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if ((input == 1) && (!tcm->sysfs_dbg_dir)) {
		tcm->sysfs_dbg_dir = kobject_create_and_add(SYSFS_SUB_DIR, tcm->sysfs_dir);
		if (!tcm->sysfs_dbg_dir) {
			LOGE("Fail to create sysfs sub directory for debugging\n");
			return -ENOTDIR;
		}

		if (sysfs_create_group(tcm->sysfs_dbg_dir, &attr_debug_group) < 0) {
			LOGE("Fail to create sysfs debug group\n");
			kobject_put(tcm->sysfs_dbg_dir);
			return -ENOTDIR;
		}
	} else if (input == 0) {
		if (tcm->sysfs_dbg_dir) {
			sysfs_remove_group(tcm->sysfs_dbg_dir, &attr_debug_group);
			kobject_put(tcm->sysfs_dbg_dir);
			tcm->sysfs_dbg_dir = NULL;
		}
	} else {
		LOGW("Unknown option %d (0:disable / 1:enable)\n", input);
		return -EINVAL;
	}

	return count;
}

static struct kobj_attribute kobj_attr_debug =
	__ATTR(debug, 0220, NULL, syna_sysfs_debug_store);

/*
 * Attribute to show the device and driver information to the console.
 *
 * param
 *    [ in] kobj:  handle of kernel object
 *    [ in] attr:  handle of kernel attribute
 *    [out] buf:  string buffer shown on console
 *
 * return
 *    string output in case of success, a negative value otherwise.
 */
static ssize_t syna_sysfs_info_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct device *p_dev;
	struct syna_tcm *tcm;
	struct tcm_dev *tcm_dev;
	int i;

	p_dev = container_of(kobj->parent, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);
	tcm_dev = tcm->tcm_dev;

	count = 0;

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Driver version:     %d.%s\n",
			SYNAPTICS_TCM_DRIVER_VERSION,
			SYNAPTICS_TCM_DRIVER_SUBVER);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Core lib version:   %d.%02d\n\n",
			(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
			(unsigned char)SYNA_TCM_CORE_LIB_VERSION);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if ((!tcm->is_connected) && (tcm->pwr_state != BARE_MODE)) {
		retval = scnprintf(buf, PAGE_SIZE - count,
				"Device is NOT connected\n");
		count += retval;
		retval = count;
		goto exit;
	} else if (tcm->pwr_state == BARE_MODE) {
		retval = scnprintf(buf, PAGE_SIZE - count,
				"Device in BARE connection\n");
		count += retval;
		retval = count;
		goto exit;
	}

	retval = scnprintf(buf, PAGE_SIZE - count,
			 "Character Dev. Node: /dev/%s* (ref. count:%d)\n\n",
			CHAR_DEVICE_NAME, tcm->char_dev_ref_count);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
			"TouchComm version:  %d\n", tcm_dev->id_info.version);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	switch (tcm_dev->id_info.mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = scnprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Application Firmware, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	case MODE_BOOTLOADER:
		retval = scnprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Bootloader, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	default:
		retval = scnprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Mode 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	}
	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Part number:        ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = syna_pal_mem_cpy(buf,
			PAGE_SIZE - count,
			tcm_dev->id_info.part_number,
			sizeof(tcm_dev->id_info.part_number),
			sizeof(tcm_dev->id_info.part_number));
	if (retval < 0) {
		LOGE("Fail to copy part number string\n");
		goto exit;
	}
	buf += sizeof(tcm_dev->id_info.part_number);
	count += sizeof(tcm_dev->id_info.part_number);

	retval = scnprintf(buf, PAGE_SIZE - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Packrat number:     %d\n\n", tcm_dev->packrat_number);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (tcm_dev->id_info.mode != MODE_APPLICATION_FIRMWARE) {
		retval = count;
		goto exit;
	}

	retval = scnprintf(buf, PAGE_SIZE - count, "Config ID:          ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	for (i = 0; i < MAX_SIZE_CONFIG_ID; i++) {
		retval = scnprintf(buf, PAGE_SIZE - count,
			"0x%2x ", tcm_dev->config_id[i]);
		if (retval < 0)
			goto exit;
		buf += retval;
		count += retval;
	}

	retval = scnprintf(buf, PAGE_SIZE - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
		"Max X & Y:          %d, %d\n", tcm_dev->max_x, tcm_dev->max_y);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
		"Num of objects:     %d\n", tcm_dev->max_objects);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
		"Num of cols & rows: %d, %d\n", tcm_dev->cols, tcm_dev->rows);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
		"Max. Read Size:     %d bytes\n", tcm_dev->max_rd_size);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, PAGE_SIZE - count,
		"Max. Write Size:    %d bytes\n", tcm_dev->max_wr_size);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_info =
	__ATTR(info, 0444, syna_sysfs_info_show, NULL);

/* Definitions of sysfs attributes */
static struct attribute *attrs[] = {
	&kobj_attr_info.attr,
	&kobj_attr_debug.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*
 * Create a directory for the use of sysfs attributes.
 *
 * param
 *    [ in] tcm:  the driver handle
 *    [ in] pdev: pointer to platform device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_sysfs_create_dir(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;

	tcm->sysfs_dir = kobject_create_and_add(SYSFS_ROOT_DIR, &pdev->dev.kobj);
	if (!tcm->sysfs_dir) {
		LOGE("Fail to create sysfs directory\n");
		return -ENOTDIR;
	}

	retval = sysfs_create_group(tcm->sysfs_dir, &attr_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");

		kobject_put(tcm->sysfs_dir);
		return retval;
	}

#ifdef HAS_TESTING_FEATURE
	retval = syna_testing_create_dir(tcm);
	if (retval < 0) {
		LOGE("Fail to create testing sysfs\n");

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);
		kobject_put(tcm->sysfs_dir);
		return retval;
	}
#endif

	return 0;
}
/*
 * Remove the directory for the use of sysfs attributes.
 *
 * param
 *    [ in] tcm: the driver handle
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
void syna_sysfs_remove_dir(struct syna_tcm *tcm)
{
	if (!tcm) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	if (tcm->sysfs_dir) {
#ifdef HAS_TESTING_FEATURE
		syna_testing_remove_dir(tcm);
#endif
		if (tcm->sysfs_dbg_dir) {
			sysfs_remove_group(tcm->sysfs_dbg_dir, &attr_debug_group);
			kobject_put(tcm->sysfs_dbg_dir);
		}

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);
		kobject_put(tcm->sysfs_dir);
	}

}
