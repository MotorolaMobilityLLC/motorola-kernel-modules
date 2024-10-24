// SPDX-License-Identifier: GPL-2.0
/*
 * @2022 awinic All Rights Reserved.
 * Description: aw8680x Qcom driver code.
 */
#include <linux/types.h>
#include "aw_protocol_config.h"

#ifdef SOC_PROTOCOL_VALID
#include "aw_protocol_map.h"
#include "aw_protocol_fun.h"

static const uint16_t id_table[] = {
	GET_16_DATA(HANDSHAKE_ID, CONNECT_ID), GET_16_DATA(HANDSHAKE_ID,
							CONNECT_ACK_ID),
	GET_16_DATA(RAM_ID, RAM_WRITE_ID), GET_16_DATA(RAM_ID,
							RAM_WRITE_ACK_ID),
	GET_16_DATA(RAM_ID, RAM_READ_ID), GET_16_DATA(RAM_ID, RAM_READ_ACK_ID),
	GET_16_DATA(FLASH_ID, FLASH_WRITE_ID), GET_16_DATA(FLASH_ID,
							FLASH_WRITE_ACK_ID),
	GET_16_DATA(FLASH_ID, FLASH_READ_ID), GET_16_DATA(FLASH_ID,
							FLASH_READ_ACK_ID),
	GET_16_DATA(FLASH_ID, FLASH_ERASE_ID), GET_16_DATA(FLASH_ID,
							FLASH_ERASE_ACK_ID),
	GET_16_DATA(FLASH_ID, FLASH_ERASE_CHIP_ID), GET_16_DATA(FLASH_ID,
						FLASH_ERASE_CHIP_ACK_ID),
	GET_16_DATA(END_ID, RAM_JUMP_ID), GET_16_DATA(END_ID, RAM_JUMP_ACK_ID),
	GET_16_DATA(END_ID, FLASH_JUMP_ID), GET_16_DATA(END_ID,
							FLASH_JUMP_ACK_ID),
	GET_16_DATA(HANDSHAKE_ID, PROTOCOL_ID), GET_16_DATA(HANDSHAKE_ID,
							PROTOCOL_ACK_ID),
	GET_16_DATA(HANDSHAKE_ID, VERSION_ID), GET_16_DATA(HANDSHAKE_ID,
							VERSION_ACK_ID),
	GET_16_DATA(UI_IRQ_ID, READ_INT_STA_ID), GET_16_DATA(UI_IRQ_ID,
							READ_INT_STA_ACK_ID),
	GET_16_DATA(HANDSHAKE_ID, CHIP_ID), GET_16_DATA(HANDSHAKE_ID,
							CHIP_ID_ACK),
	GET_16_DATA(HANDSHAKE_ID, CHIP_DATE), GET_16_DATA(HANDSHAKE_ID,
							CHIP_DATE_ACK),
	GET_16_DATA(AW_ID_FREE, AW_ID_FREE), GET_16_DATA(HANDSHAKE_ID,
						MOUDLE_EVNET_NO_RESPOND),
	GET_16_DATA(END_ID, ROM_JUMP_ID), GET_16_DATA(END_ID, ROM_JUMP_ACK_ID),
};

uint8_t aw_check_id_fun(uint8_t module_id, uint8_t event_id)
{
	uint8_t i = 0;
	uint8_t i_max = 0;
	uint16_t id_num = 0;

	i_max = sizeof(id_table) / sizeof(uint16_t);
	id_num = GET_16_DATA(module_id, event_id);
	for (i = 0; i < i_max; i++) {
		if (id_num == id_table[i])
			return AW_OK;
	}

	return AW_FAIL;
}

#ifdef AW_PROTOCOL_MCU

uint8_t aw_set_ui_read_data_len(struct gui_to_soc_struct *p_gui_data_s)
{
	uint16_t id_num = 0;

#ifdef POINTER_CHECK
	if (p_gui_data_s == AW_NULL)
		return AW_FAIL;
#endif
	id_num = GET_16_DATA(p_gui_data_s->module_id, p_gui_data_s->event_id);

	if ((id_num == id_table[CONNECT_NUM]) ||
		(id_num == id_table[PROTOCOL_VERSION_NUM]) ||
		(id_num == id_table[VERSION_INFOR_NUM]) ||
		(id_num == id_table[CHIP_ID_NUM]) ||
		(id_num == id_table[CHIP_DATE_NUM])) {
		p_gui_data_s->ui_rd_data_len = DATA_ACK_LEN;
	} else if ((id_num == id_table[RAM_READ_NUM]) ||
				(id_num == id_table[FLASH_READ_NUM]) ||
				(id_num == id_table[UI_IRQ_NUM])) {
		p_gui_data_s->ui_rd_data_len = (p_gui_data_s->read_len + DATA_ACK_LEN);
	} else {
		p_gui_data_s->ui_rd_data_len = DATA_ACK_ERR_LEN;
	}

	return AW_OK;
}

#endif /* AW_PROTOCOL_MCU */

#endif /* SOC_PROTOCOL_VALID */
