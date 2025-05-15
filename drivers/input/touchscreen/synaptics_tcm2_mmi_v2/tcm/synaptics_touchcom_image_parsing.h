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
 * This file declares the functions and structures being used to parse the Synaptics
 * image file for firmware update.
 */

#ifndef _SYNAPTICS_TOUCHCOM_PARSE_FW_IMAGE_H_
#define _SYNAPTICS_TOUCHCOM_PARSE_FW_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "synaptics_touchcom_core_dev.h"


/* The easy-to-use macros  */
#define FLASH_PARTITION_ID_STR(area) \
	(syna_tcm_get_partition_id_string(area))


/* List of a data partitions */
enum flash_area {
	AREA_NONE = 0,
	/* please add the declarations below */

	AREA_APP_CODE,
	AREA_APP_CODE_COPRO,
	AREA_APP_CONFIG,
	AREA_DISP_CONFIG,
	AREA_BOOT_CODE,
	AREA_BOOT_CONFIG,
	AREA_PROD_TEST,
	AREA_F35_APP_CODE,
	AREA_FORCE_TUNING,
	AREA_GAMMA_TUNING,
	AREA_TEMPERATURE_GAMM_TUNING,
	AREA_CUSTOM_LCM,
	AREA_LOOKUP,
	AREA_CUSTOM_OEM,
	AREA_OPEN_SHORT_TUNING,
	AREA_CUSTOM_OTP,
	AREA_PPDT,
	AREA_ROMBOOT_APP_CODE,
	AREA_TOOL_BOOT_CONFIG,

	/* please add the declarations above */
	AREA_TOOL_JSON,
	AREA_TOOL_CUSTOM_CS,
	AREA_TOOL_CUSTOM_LOCKDOWN,
	AREA_MAX,
};

/* Header of the content of app config defined in the image file */
struct app_config_header {
	unsigned short magic_value[4];
	unsigned char checksum[4];
	unsigned char length[2];
	unsigned char build_id[4];
	unsigned char customer_config_id[16];
};
struct app_config_header_v2 {
	unsigned short magic_value[4];
	unsigned char checksum[4];
	unsigned char length[4];
	unsigned char build_id[4];
	unsigned char customer_config_id[16];
};
/* Definitions of a data area defined in the firmware file */
struct area_block {
	bool described;
	const unsigned char *data;
	unsigned int size;
	unsigned int flash_addr;
	unsigned char id;
	unsigned int checksum;
};
/* Header of firmware image file */
struct image_header {
	unsigned char magic_value[4];
	unsigned char num_of_areas[4];
};
/* Structure of image data after the file parsing */
struct image_info {
	unsigned int size;
	struct image_header *header;
	struct area_block data[AREA_MAX];
};


/*
 * Standard API Definitions
 */
/*
 *  Return the string ID of target data partition
 *
 * param
 *    [ in] area: target partition
 *
 * return
 *    the string associated to the data partition
 */
char *syna_tcm_get_partition_id_string(enum flash_area area);

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
int syna_tcm_parse_fw_image(const unsigned char *binary, unsigned int size, struct image_info *image);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end of _SYNAPTICS_TOUCHCOM_PARSE_FW_IMAGE_H_ */
