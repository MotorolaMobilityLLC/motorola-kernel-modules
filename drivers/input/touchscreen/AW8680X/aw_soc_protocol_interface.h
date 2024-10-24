/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
 * All rights reserved.
 * Description: Communication protocol related header file
 */
#ifndef __AW_SOC_PROTOCOL_INTERFACE_H
#define __AW_SOC_PROTOCOL_INTERFACE_H
#include <linux/types.h>
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

uint8_t aw_soc_protocol_pack_interface(struct gui_to_soc_struct *p_gui_data_s,
							uint8_t *p_aw_protocol_tx_data);
uint8_t aw_soc_protocol_unpack_interface(struct gui_to_soc_struct *p_gui_data_s,
							uint8_t *p_aw_protocol_rx_data);

#endif
