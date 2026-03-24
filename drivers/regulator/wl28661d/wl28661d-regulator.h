// SPDX-License-Identifier: GPL-2.0+
/*
 * WL28661D, Multi-Output Regulators
 * Copyright (C) 2026  Motorola Mobility LLC,
 *
 * Author: Chen Zhiming, Motorola Mobility LLC,
 */

#ifndef __WL28661D_REGISTERS_H__
#define __WL28661D_REGISTERS_H__

/* Registers */
#define WL28661D_REG_NUM (WL28661D_SEQ_STATUS-WL28661D_CHIP_REV+1)

#define WL28661D_CHIP_REV 0x00
#define WL28661D_CURRENT_LIMITSEL 0x01
#define WL28661D_DISCHARGE_RESISTORS 0x02
#define WL28661D_LDO1_VOUT 0x03
#define WL28661D_LDO2_VOUT 0x04
#define WL28661D_LDO3_VOUT 0x05
#define WL28661D_LDO4_VOUT 0x06
#define WL28661D_LDO1_LDO2_SEQ 0x0a
#define WL28661D_LDO3_LDO4_SEQ 0x0b
#define WL28661D_LDO_EN 0x0e
#define WL28661D_SEQ_STATUS 0x0f


/* WL28661D_LDO1_VSEL ~ WL28661D_LDO4_VSEL =
 * 0x03, 0x04, 0x05, 0x06
 */
#define  WL28661D_LDO1_VSEL                      WL28661D_LDO1_VOUT
#define  WL28661D_LDO2_VSEL                      WL28661D_LDO2_VOUT
#define  WL28661D_LDO3_VSEL                      WL28661D_LDO3_VOUT
#define  WL28661D_LDO4_VSEL                      WL28661D_LDO4_VOUT


#define  WL28661D_VSEL_SHIFT                     0
#define  WL28661D_VSEL_MASK                      (0xff << 0)

#define  WL28661D_N_VOLTAGES                     256

#define  WL28661D_ID                             0x33

#endif /* __WL28661D_REGISTERS_H__ */
