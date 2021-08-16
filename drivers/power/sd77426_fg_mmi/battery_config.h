/*****************************************************************************
* Copyright(c) O2Micro, 2019. All rights reserved.
*       
* O2Micro [OZ8806] Source Code Reference Design
* File:[battery_config.h]
*       
* This Source Code Reference Design for O2MICRO [OZ8806] access 
* ("Reference Design") is solely for the use of PRODUCT INTEGRATION REFERENCE ONLY, 
* and contains confidential and privileged information of O2Micro International 
* Limited. O2Micro shall have no liability to any PARTY FOR THE RELIABILITY, 
* SERVICEABILITY FOR THE RESULT OF PRODUCT INTEGRATION, or results from: (i) any 
* modification or attempted modification of the Reference Design by any party, or 
* (ii) the combination, operation or use of the Reference Design with non-O2Micro 
* Reference Design. Use of the Reference Design is at user's discretion to qualify 
* the final work result.
*****************************************************************************/

#ifndef	__O2_BATTERY_CONFIG_H 
#define	__O2_BATTERY_CONFIG_H __FILE__


#define OZ88106_I2C_NUM    2

//#define	EXT_THERMAL_READ
		
#define OZ8806_VOLTAGE  		    4350
#define O2_CONFIG_CAPACITY  		5158
#define O2_CONFIG_RSENSE  		    20
#define O2_CONFIG_EOC  				200
#define OZ8806_EOD 				    3400
#define O2_CONFIG_BOARD_OFFSET		12
#define O2_SOC_START_THRESHOLD_VOL 3700
#define O2_CONFIG_RECHARGE			100
#define O2_TEMP_PULL_UP_R           121000 //121K
#define O2_TEMP_REF_VOLTAGE         1800 //mv
#define BATTERY_WORK_INTERVAL		10

#define RPULL (68000)
#define RDOWN (5100)

#define ENABLE_10MIN_END_CHARGE_FUN

#endif