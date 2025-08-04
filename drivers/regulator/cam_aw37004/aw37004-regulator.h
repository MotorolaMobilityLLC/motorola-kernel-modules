// SPDX-License-Identifier: GPL-2.0+
/*
 * AW37004, Multi-Output Regulators
 * Copyright (C) 2024  Motorola Mobility LLC,
 *
 * Author: Chen Zhiming, Motorola Mobility LLC,
 */

#ifndef __AW37004_REGISTERS_H__
#define __AW37004_REGISTERS_H__

/* Registers */
#define AW37004_REG_NUM (AW37004_CHIP_REV2-AW37004_CHIP_REV+1)

#define AW37004_CHIP_REV 0x00
#define AW37004_CURRENT_LIMITSEL 0x01
#define AW37004_DISCHARGE_RESISTORS 0x02
#define AW37004_LDO1_VOUT 0x03
#define AW37004_LDO2_VOUT 0x04
#define AW37004_LDO3_VOUT 0x05
#define AW37004_LDO4_VOUT 0x06
#define AW37004_LDO1_LDO2_SEQ 0x0a
#define AW37004_LDO3_LDO4_SEQ 0x0b
#define AW37004_LDO_EN 0x0e
#define AW37004_SEQ_STATUS 0x0f
#define AW37004_CHIP_REV2 0x19


/* AW37004_LDO1_VSEL ~ AW37004_LDO4_VSEL =
 * 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
 */
#define  AW37004_LDO1_VSEL                      AW37004_LDO1_VOUT
#define  AW37004_LDO2_VSEL                      AW37004_LDO2_VOUT
#define  AW37004_LDO3_VSEL                      AW37004_LDO3_VOUT
#define  AW37004_LDO4_VSEL                      AW37004_LDO4_VOUT


#define  AW37004_VSEL_SHIFT                     0
#define  AW37004_VSEL_MASK                      (0xff << 0)

#define  AW37004_N_VOLTAGES                     256

#define  COMMON_ID                              0x00
#define  AW37004_ID2                            0x04
#define  WL28661D_CHIP_ID                       0x33

#endif /* __AW37004_REGISTERS_H__ */
