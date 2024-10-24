/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
 * All rights reserved.
 * Description: Communication protocol related header file
 */
#ifndef __AW_COM_PROTOCOL_H
#define __AW_COM_PROTOCOL_H
#include <linux/types.h>
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#define GET_HIGH_4BIT(data)		(((data) & 0xf0)>>4)
#define GET_LOW_4BIT(data)		((data) & 0x0f)

#ifdef AW_PROTOCOL_MCU

#define UI_LENGTH_LEN	(2U)
#define NO_RESPOND_LEN	(3U)
#define UI_ADDR_LEN		(4U)
#define UI_L1_L2_LEN	(9U)

uint8_t aw_soc_protocol_pack(struct gui_to_soc_struct *p_gui_data_s,
						uint8_t *p_aw_protocol_tx_data);
enum check_flag_enum aw_soc_protoco_unpack(struct gui_to_soc_struct *p_gui_data_s,
						uint8_t *p_aw_protocol_rx_data);

#endif /* AW_PROTOCOL_MCU */

uint8_t check_sum(uint8_t *buf, uint16_t len);
enum check_flag_enum check_protocol_header_data(struct protocol_data_struct *
				p_aw_protocol_rx_data_s, enum id_enum aw_id_e);
uint8_t aw_set_protocol_header_data(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint16_t data_len);

#endif
