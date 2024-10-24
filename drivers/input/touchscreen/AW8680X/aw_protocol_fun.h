/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
 * All rights reserved.
 * Description: Communication protocol fun related header file
 */
#ifndef __AW_PROTOCOL_FUN_H
#define __AW_PROTOCOL_FUN_H
#include <linux/types.h>
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#define GET_3BYT(data) ((uint8_t)((data)>>24))
#define GET_2BYT(data) ((uint8_t)((data)>>16))
#define GET_1BYT(data) ((uint8_t)((data)>>8))
#define GET_0BYT(data) ((uint8_t)((data)>>0))

#define CLEAN_HEAD_BYTE			((uint32_t)0x00FFFFFF)
#define CHECK_ACK_LEN			(0x01)
#define HANDSHAKE_ACK_LEN		(0x05)
#define ID_NO_RESPOND_LEN		(0x04)
#define AW_LOCATION_UBOOT		((uint32_t)0x00000001)
#define AW_FLASH_BOOT			((uint32_t)0x00010002)
#define AW_RAM_BOOT				((uint32_t)0x00010003)
#define READ_ACK_LENGTH			((uint16_t)0x05) /* flags and addresses */

#define ERROR_LOCATION_DATA		(0)
#define FUN_LOCATION_DATA		(1)
#define WRITE_DATA_ADDR			(0)
#define WRITE_USE_DATA			(4)
#define LENGTH_DATA				(0)
#define LENGTH_DATA_H			(1)
#define READ_DATA_ADDR			(2)
#define READ_DATA_ADDR_ACK		(1)
#define READ_SEND_DATA			(5)
#define END_ADDR				(0)
/* IF module or evnet id err ,ack moudel_id location length */
#define ERR_ACK_MODULE			(1)
/* IF module or evnet id err ,ack evenet_id location length */
#define ERR_ACK_EVNENT_LEN		(2)
/* IF module or evnet id err ,AW chip location length */
#define ERR_ACK_CHIP_LEN		(3)

uint8_t aw_set_u32_fun(uint8_t *u8_addr, uint32_t u32_data);
uint32_t aw_get_u32_fun(uint8_t *u8_addr);
uint8_t aw_get_rx_module_id(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t aw_set_tx_module_id(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t module_id_num);
uint8_t aw_set_tx_data_error(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t err_data);
uint8_t aw_set_tx_location_fun(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint32_t boot_num);
uint8_t aw_get_rx_event_id(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t aw_set_tx_event_id(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t event_id_data);
uint32_t aw_get_rx_write_data_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint16_t aw_get_rx_payload_length(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t aw_get_rx_check_data(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t *aw_get_rx_write_use_data(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint16_t aw_get_rx_read_erase_length(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint32_t aw_get_rx_read_data_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t *aw_get_tx_send_data_buff(struct protocol_data_struct *p_aw_protocol_tx_data_s);
uint8_t aw_set_tx_read_data_addr(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint32_t data_addr);
uint32_t aw_get_rx_end_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s);
uint8_t aw_set_tx_data_ack(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t ack_data);
uint32_t aw_set_err_id_fun(struct protocol_data_struct *p_aw_protocol_rx_data_s,
	struct protocol_data_struct *p_aw_protocol_tx_data_s, enum id_enum aw_adress);

#endif
