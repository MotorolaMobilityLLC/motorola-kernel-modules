/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef __SX9377_H__
#define __SX9377_H__

#ifdef CONFIG_CAPSENSE_FLIP_CAL
#include <linux/extcon.h>
#endif

//#define COMPILE_MOTO_DRV

typedef enum{
	PMIC_LDO,
	ALWAYS_ON,
	EXTERNAL_LDO,
}POWER_SUPPLY;



#define REG_IRQ_SRC		0x4000
#define REG_IRQ_MASK        0x4004

#define REG_RESET		0x4240
#define REG_CMD			0x4280
#define REG_CMD_STATE       0x4284

#define REG_WHOAMI		0x42CC
#define REG_PHEN		0x8024
#define REG_PROX_STATUS 0x8000
#define REG_USE_PH0		0x8210
#define REG_AVG_PH0		0x8230
#define REG_DIF_PH0		0x8250
#define REG_OFF_PH0		0x802C

#define REG_DBG_SEL         0x8274
#define REG_DLT_VAR         0x8284
#define REG_RAW_DATA        0x8280

//=================================================================================================
//Chip specific defination
#define SMTC_SX937X
#define NUM_PHASES              8
#define SMTC_CHIP_NAME          "sx9377"
#define SMTC_DRIVER_NAME        "smtc_sx9377"
#define SMTC_COMPATIBLE_NAME    "semtech,sx9377"
#define OFFSET_VAL_MASK         0x3FFF
#define OFFSET_PH_REG_SHIFT     3
#define IRQ_NAME                "smtc sx9377 IRQ"
#define PHEN_MASK               0xFF
#define COMPENSATION_MASK       0xFF00
#define COMPENSATION_OFF        16
#define AVG_FLT_PH_OFF          0x20
#define SATURATED_USEFUL        0x3FFFFC00

#endif
