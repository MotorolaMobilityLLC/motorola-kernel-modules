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
 * This file implements the functions to parse the Synaptics image file for
 * firmware update.
 */

#include "synaptics_touchcom_image_parsing.h"


#define IMAGE_FILE_MAGIC_VALUE (0x4818472b)

#define FLASH_AREA_MAGIC_VALUE (0x7c05e516)

#define JSON_AREA_MAGIC_VALUE (0xC1FB41D8)

 /* The easy-to-use macros  */
#define CRC32(data, length) \
	(syna_pal_crc32(~0, data, length) ^ ~0)


/* The Partition Descriptor of each data area defined in the image file */
struct area_descriptor {
	unsigned char magic_value[4];
	unsigned char id_string[16];
	unsigned char flags[4];
	unsigned char destination_addr_words[4];
	unsigned char length_bytes[4];
	unsigned char checksum[4];
};


/*
 *  Return the string ID of target data partition
 *
 * param
 *    [ in] area: target partition
 *
 * return
 *    the string associated to the data partition
 */
char *syna_tcm_get_partition_id_string(enum flash_area area)
{
	switch (area) {
	case AREA_BOOT_CODE:
		return "BOOT_CODE";
	case AREA_BOOT_CONFIG:
		return "BOOT_CONFIG";
	case AREA_APP_CODE:
		return "APP_CODE";
	case AREA_APP_CODE_COPRO:
		return "APP_CODE_COPRO";
	case AREA_APP_CONFIG:
		return "APP_CONFIG";
	case AREA_PROD_TEST:
		return "APP_PROD_TEST";
	case AREA_DISP_CONFIG:
		return "DISPLAY";
	case AREA_F35_APP_CODE:
		return "F35_APP_CODE";
	case AREA_FORCE_TUNING:
		return "FORCE";
	case AREA_GAMMA_TUNING:
		return "GAMMA";
	case AREA_TEMPERATURE_GAMM_TUNING:
		return "TEMPERATURE_GAMM";
	case AREA_CUSTOM_LCM:
		return "LCM";
	case AREA_LOOKUP:
		return "LOOKUP";
	case AREA_CUSTOM_OEM:
		return "OEM";
	case AREA_OPEN_SHORT_TUNING:
		return "OPEN_SHORT";
	case AREA_CUSTOM_OTP:
		return "OTP";
	case AREA_PPDT:
		return "PPDT";
	case AREA_ROMBOOT_APP_CODE:
		return "ROMBOOT_APP_CODE";
	case AREA_TOOL_BOOT_CONFIG:
		return "TOOL_BOOT_CONFIG";
	case AREA_TOOL_JSON:
		return "JSON_CONFIG_AREA";
	case AREA_TOOL_CUSTOM_CS:
		return "CUSTOM_CS_AREA";
	case AREA_TOOL_CUSTOM_LOCKDOWN:
		return "CUSTOM_LOCKDOWN_AREA";
	default:
		return "";
	}
}
/*
 *  Query the corresponding ID of target data partition
 *
 * param
 *    [ in] str: string to look for
 *
 * return
 *    the corresponding ID in case of success, AREA_NONE (0) otherwise.
 */
static int syna_tcm_get_partition_id(char *str)
{
	int area;
	char *target;
	unsigned int len;

	for (area = AREA_MAX - 1; area >= 0; area--) {
		target = (char *)FLASH_PARTITION_ID_STR((enum flash_area)area);
		len = syna_pal_str_len(target);

		if (syna_pal_str_cmp(str, target, len) == 0)
			return area;
	}

	LOGW("Un-defined area string, %s\n", str);
	return (int)AREA_NONE;
}
/*
 *  Create the specific structure representing the target data partition
 *
 * param
 *    [out] partition:  partition info used for storing the parsed data
 *    [ in] area:       id of target partition
 *    [ in] descriptor: descriptor of partition
 *    [ in] content:    content of data
 *    [ in] length:     size of data
 *
 * return
 *     0 or positive value in case of success, a negative value otherwise.
 */
static int syna_tcm_save_flash_partition_data(struct area_block *partition,
	unsigned int area, struct area_descriptor *descriptor,
	const unsigned char *content, const unsigned int length)
{
	unsigned int tmp;
	unsigned int checksum;
	unsigned int destination_addr;

	if (!partition) {
		LOGE("Invalid partition buffer to store data\n");
		return -ERR_INVAL;
	}

	if (!descriptor) {
		LOGE("Invalid descriptor for partition\n");
		return -ERR_INVAL;
	}

	if (area == AREA_TOOL_JSON) {
		partition->size = length;
		partition->data = content;
		partition->flash_addr = 0;
		partition->id = (unsigned char)AREA_TOOL_JSON;
		partition->described = true;

		LOGI("AREA_TOOL_JSON area - size:%d\n", length);
	} else {

		destination_addr = syna_pal_le4_to_uint(descriptor->destination_addr_words);
		destination_addr = destination_addr * 2;
		checksum = syna_pal_le4_to_uint(descriptor->checksum);
		tmp = CRC32((const char *)content, length);

		if (checksum != tmp) {
			LOGE("partition %s checksum error, image file: 0x%x (0x%x)\n",
				FLASH_PARTITION_ID_STR((enum flash_area)area), checksum, tmp);
			return -ERR_INVAL;
		}

		partition->size = length;
		partition->data = content;
		partition->flash_addr = destination_addr;
		partition->id = (unsigned char)area;
		partition->described = true;
		partition->checksum = checksum;

		LOGI("%s area - address:0x%08x (%d), size:%d\n",
			FLASH_PARTITION_ID_STR((enum flash_area)area),
			partition->flash_addr, partition->flash_addr, partition->size);
	}

	return 0;
}
/*
 *  Parse the given firmware image file and turn into the binary data for partitions.
 *
 * param
 *    [ in] binary:  given binary, the raw data read from the external image file
 *    [ in] size:    size of given binary
 *    [out] image:   parsed data
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_tcm_parse_fw_image(const unsigned char *binary, unsigned int size, struct image_info *image)
{
	int retval = 0;
	unsigned int idx;
	unsigned int addr;
	unsigned int offset;
	unsigned int magic_value;
	unsigned int num_of_areas;
	struct area_descriptor *descriptor;
	unsigned int length;
	const unsigned char *content;
	int area;

	if (!binary) {
		LOGE("Invalid given data\n");
		return -ERR_INVAL;
	}

	if (!image) {
		LOGE("Invalid image blob to store the parsed data\n");
		return -ERR_INVAL;
	}

	if (size == 0) {
		LOGE("Invalid image data\n");
		return -ERR_INVAL;
	}

	syna_pal_mem_set(image, 0x00, sizeof(struct image_info));

	image->header = (struct image_header *)binary;
	image->size = size;

	magic_value = syna_pal_le4_to_uint(image->header->magic_value);
	if (magic_value != IMAGE_FILE_MAGIC_VALUE) {
		LOGE("Invalid image file magic value\n");
		return -ERR_INVAL;
	}

	offset = sizeof(struct image_header);
	num_of_areas = syna_pal_le4_to_uint(image->header->num_of_areas);

	/* walk through the partitions */
	for (idx = 0; idx < num_of_areas; idx++) {
		addr = syna_pal_le4_to_uint(binary + offset);
		descriptor = (struct area_descriptor *)(binary + addr);
		offset += 4;

		magic_value = syna_pal_le4_to_uint(descriptor->magic_value);
		if (magic_value == FLASH_AREA_MAGIC_VALUE) {
			area = syna_tcm_get_partition_id((char *)descriptor->id_string);
			if ((area <= (int)AREA_NONE) || (area >= (int)AREA_MAX))
				continue;

			length = syna_pal_le4_to_uint(descriptor->length_bytes);
			content = (unsigned char *)descriptor + sizeof(*descriptor);
		} else if (magic_value == JSON_AREA_MAGIC_VALUE) {
			area = AREA_TOOL_JSON;
			length = syna_pal_le4_to_uint(binary + addr + 4);
			content = (unsigned char *)(binary + addr + 8);
		} else {
			length = 0;
			content = NULL;
			continue;
		}

		retval = syna_tcm_save_flash_partition_data(&image->data[area], area, descriptor, content, length);
		if (retval < 0) {
			LOGD("Fail to save the partition data of %s\n", FLASH_PARTITION_ID_STR((enum flash_area)area));
			continue;
		}
	}
	return 0;
}

