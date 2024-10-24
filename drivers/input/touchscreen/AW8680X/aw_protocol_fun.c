// SPDX-License-Identifier: GPL-2.0
/*
 * @2022 awinic All Rights Reserved.
 * Description: aw8680x Qcom driver code.
 */
#include <linux/types.h>
#include "aw_protocol_config.h"

#ifdef SOC_PROTOCOL_VALID
#include "aw_protocol_fun.h"
#include "aw_protocol_map.h"

uint8_t aw_set_u32_fun(uint8_t *u8_addr, uint32_t u32_data)
{
#ifdef POINTER_CHECK
		if (u8_addr == AW_NULL)
			return AW_FAIL;
#endif

	u8_addr[0] = GET_0BYT(u32_data);
	u8_addr[1] = GET_1BYT(u32_data);
	u8_addr[2] = GET_2BYT(u32_data);
	u8_addr[3] = GET_3BYT(u32_data);

	return AW_OK;
}

uint32_t aw_get_u32_fun(uint8_t *u8_addr)
{
#ifdef POINTER_CHECK
	if (u8_addr == AW_NULL)
		return AW_FAIL;
#endif

	return (uint32_t)((u8_addr[0]<<0) + (u8_addr[1]<<8) +
				(u8_addr[2]<<16) + (u8_addr[3]<<24));
}

static uint16_t aw_get_u16_fun(uint8_t *u8_addr)
{
#ifdef POINTER_CHECK
	if (u8_addr == AW_NULL)
		return AW_FAIL;
#endif

	return (uint16_t)((u8_addr[1] << 8) | u8_addr[0]);
}

uint8_t aw_set_tx_data_ack(struct protocol_data_struct *p_aw_protocol_tx_data_s,
								uint8_t ack_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.ack = ack_data;
	return AW_OK;
}

uint8_t aw_set_tx_data_error(struct protocol_data_struct *p_aw_protocol_tx_data_s,
								uint8_t err_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l3_data[ERROR_LOCATION_DATA] =
								     err_data;

	return AW_OK;
}

uint8_t aw_set_tx_location_fun(struct protocol_data_struct *p_aw_protocol_tx_data_s,
								uint32_t boot_num)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	aw_set_u32_fun(&p_aw_protocol_tx_data_s->protocol_l3_data
						[FUN_LOCATION_DATA], boot_num);

	return AW_OK;
}

uint32_t aw_get_rx_write_data_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
							[WRITE_DATA_ADDR]);
}

uint8_t *aw_get_rx_write_use_data(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_NULL;
#endif

	return &p_aw_protocol_rx_data_s->protocol_l3_data[WRITE_USE_DATA];
}

uint8_t aw_set_tx_module_id(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t module_id_num)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.module_id = module_id_num;

	return AW_OK;
}

uint8_t aw_get_rx_module_id(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.module_id;
}

uint8_t aw_set_tx_event_id(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint8_t event_id_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.event_id = event_id_data;

	return AW_OK;
}

uint8_t aw_get_rx_event_id(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.event_id;
}

uint16_t aw_get_rx_payload_length(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return GET_16_DATA(p_aw_protocol_rx_data_s->protocol_l2_s.payload_length_h,
				p_aw_protocol_rx_data_s->protocol_l2_s.payload_length_l);
}

uint8_t aw_get_rx_check_data(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.check_data;
}

uint16_t aw_get_rx_read_erase_length(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u16_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
								[LENGTH_DATA]);
}

uint32_t aw_get_rx_read_data_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
							[READ_DATA_ADDR]);
}

uint8_t aw_set_tx_read_data_addr(struct protocol_data_struct *p_aw_protocol_tx_data_s,
							uint32_t data_addr)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	aw_set_u32_fun(&p_aw_protocol_tx_data_s->protocol_l3_data
					[READ_DATA_ADDR_ACK], data_addr);

	return AW_OK;
}

uint8_t *aw_get_tx_send_data_buff(struct protocol_data_struct *p_aw_protocol_tx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_NULL;
#endif

	return &p_aw_protocol_tx_data_s->protocol_l3_data[READ_SEND_DATA];
}

uint32_t aw_get_rx_end_addr(struct protocol_data_struct *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
								[END_ADDR]);
}

uint32_t aw_set_err_id_fun(struct protocol_data_struct *p_aw_protocol_rx_data_s,
				struct protocol_data_struct *p_aw_protocol_tx_data_s,
				enum id_enum aw_adress)
{
#ifdef POINTER_CHECK
	if ((p_aw_protocol_rx_data_s == AW_NULL) ||
	    (p_aw_protocol_tx_data_s == AW_NULL))
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_MODULE] =
			p_aw_protocol_rx_data_s->protocol_l2_s.module_id;
	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_EVNENT_LEN] =
				p_aw_protocol_rx_data_s->protocol_l2_s.event_id;
	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_CHIP_LEN] = aw_adress;

	return AW_OK;
}

#endif /* SOC_PROTOCOL_VALID */


