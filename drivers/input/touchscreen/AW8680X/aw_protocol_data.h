/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
 * All rights reserved.
 * Description: Communication protocol data related header file
 */
#ifndef __AW_PROTOCOL_DATA_H
#define __AW_PROTOCOL_DATA_H
#include <linux/types.h>
#include "aw_type.h"
#include "aw_protocol_config.h"


#define PROTOCOL_TOTAL_LEN		(256)
#define PROTOCOL_L1_S_LEN		(sizeof(struct protocol_l1_struct))
#define PROTOCOL_L2_S_LEN		(sizeof(struct protocol_l2_struct))
#define IIC_L3_DATA_LEN			(PROTOCOL_TOTAL_LEN - PROTOCOL_L1_S_LEN - PROTOCOL_L2_S_LEN)
#define PROTOCOL_S_LEN			(sizeof(struct protocol_data_struct))
#define CHECK_PROTOCOL_LEN		(PROTOCOL_L1_S_LEN + PROTOCOL_L2_S_LEN - 1)

enum check_ack_enum {
	AW_ACK_BUSY = 0x00,
	AW_ACK_FREE = 0x01,
};

enum version_enum {
	AW_VERSION_1 = 0x01, /* protocol version */
	AW_VERSION_2 = 0x02,
	AW_VERSION_3 = 0x03,
};

enum id_enum {
	PC_ADDRESS_ID  = 0x01,
	MCU_ADDRESS_ID = 0x02,
	AP_ADDRESS_ID  = 0x04,
	AW_ADDRESS_ID  = 0x08,
};

enum module_enum {
	AW_ID_FREE			= 0X00,
	HANDSHAKE_ID			= 0x01,
	RAM_ID				= 0x02,
	FLASH_ID			= 0x03,
	END_ID				= 0x04,
	UI_IRQ_ID			= 0x05,
	AW_MODULE_ID_ERR		= 0Xff,
};

enum connect_enum {
	CONNECT_ID = 0x01,
	CONNECT_ACK_ID = 0x02,
	MOUDLE_EVNET_NO_RESPOND = 0x08,
	PROTOCOL_ID = 0x11,
	PROTOCOL_ACK_ID = 0x12,
	VERSION_ID = 0x21,
	VERSION_ACK_ID = 0x22,
	CHIP_ID = 0x31,
	CHIP_ID_ACK = 0x32,
	CHIP_DATE = 0x33,
	CHIP_DATE_ACK = 0x34,
	AW_EVENT_ID_ERR = 0Xff,
};

enum ram_enum {
	RAM_WRITE_ID	 = 0x01,
	RAM_WRITE_ACK_ID = 0x02,
	RAM_READ_ID		 = 0x11,
	RAM_READ_ACK_ID	 = 0x12,
};

enum flash_enum {
	FLASH_WRITE_ID		 = 0x01,
	FLASH_WRITE_ACK_ID	 = 0x02,
	FLASH_READ_ID		  = 0x11,
	FLASH_READ_ACK_ID	  = 0x12,
	FLASH_ERASE_ID		 = 0x21,
	FLASH_ERASE_ACK_ID	 = 0x22,
	FLASH_ERASE_CHIP_ID	= 0X23,
	FLASH_ERASE_CHIP_ACK_ID = 0X24,
};

enum end_enum {
	RAM_JUMP_ID		= 0x01,
	RAM_JUMP_ACK_ID		= 0x02,
	FLASH_JUMP_ID		= 0x11,
	FLASH_JUMP_ACK_ID	= 0x12,
	ROM_JUMP_ID		= 0x21,
	ROM_JUMP_ACK_ID		= 0x22,
};

enum ui_irq_enum {
	READ_INT_STA_ID			= 0x01,
	READ_INT_STA_ACK_ID		= 0x02,
};

enum check_flag_enum {
	AW_FLAG_OK = 0x00,
	AW_FLAG_FAIL = 0x01,
	AW_CHECK_HEADER_ERR = 0x02,
	AW_VERSION_ERR = 0x03,
	AW_ADDRESS_ID_ERR = 0x04,
	AW_CHECK_DATA_ERR = 0x05,
	AW_CHECK_ID_ERR = 0x06,
	AW_ADDRESS_ERR = 0x07,
	AW_HANDSHAKE_ERR = 0x10,
	AW_RAM_WRITE_ERR = 0x20,
	AW_RAM_READ_ERR = 0x21,
	AW_FLASH_WRITE_ERR = 0x30,
	AW_FLASH_READ_ERR = 0x31,
	AW_FLASH_SECTOR_ERR = 0x32,
	AW_FLASH_CHIP_ERR = 0x33,
	AW_END_ERR = 0x40,
	AW_TIME_OUT_ERR = 0x41,
	AW_EXCEPTION_ERR = 0xff,
};

struct protocol_l1_struct {
	uint8_t check_header;
	uint8_t version;
	uint8_t adress;
};

struct protocol_l2_struct {
	uint8_t module_id;
	uint8_t event_id;
	uint8_t payload_length_l;
	uint8_t payload_length_h;
	uint8_t ack;
	uint8_t check_data;
};

struct protocol_data_struct {
	struct protocol_l1_struct protocol_l1_s;
	struct protocol_l2_struct protocol_l2_s;
	uint8_t protocol_l3_data[IIC_L3_DATA_LEN];
};

struct data_buff_struct {
	uint8_t dest_adr;
	uint8_t src_adr;
};

#ifdef AW_PROTOCOL_MCU
struct gui_to_soc_struct {
	uint16_t version_num; /* Effective data length */
	uint16_t soc_data_len; /* Effective data length */
	uint16_t ui_rd_data_len; /* Data length to be read after the upper computer finishes writing data */
	uint16_t read_len; /* Read addr data length */
	uint32_t addr; /* Address where writes data */
	uint8_t device_commu;
	uint8_t device_addr;
	uint8_t dest_adr; /* Destination address means: to whom? */
	uint8_t src_adr; /* source addres means:form where? */
	uint8_t module_id; /* module id */
	uint8_t event_id; /* evnet id */
	uint8_t err_flag; /* Operation status of lower computer */
	uint8_t reserved0; /* reserved */
	uint8_t reserved1; /* reserved */
	uint8_t soc_data[IIC_L3_DATA_LEN]; /* Effective data */
};
#endif /* AW_PROTOCOL_MCU */

#endif
