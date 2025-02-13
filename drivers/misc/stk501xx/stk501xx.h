/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for stk501xx SAR sensor chip.
 */
#ifndef __STK501XX_H__
#define __STK501XX_H__

#include "common_define.h"
#include "stk501xx_config.h"
#include "stk501xx_ver.h"
#ifdef MCU_GESTURE
    #include "stk_gesture_lib.h"
#endif

#ifdef STK_REG_READ
    #undef STK_REG_READ
#endif
#ifdef STK_REG_WRITE
    #undef STK_REG_WRITE
#endif
#define STK_REG_READ stk501xx_read
#define STK_REG_WRITE stk501xx_write

/*****************************************************************************
 * Global variable
 *****************************************************************************/
enum
{
    DIST_GAIN_512 = 0x0,
    DIST_GAIN_256 = 0x1,
    DIST_GAIN_128 = 0x2,
    DIST_GAIN_64  = 0x3,
    DIST_GAIN_32  = 0x4,
    DIST_GAIN_16  = 0x5,
    DIST_GAIN_8   = 0x6,
    DIST_GAIN_4   = 0x7,
};

//Threshold fill in delta decimal value
#define STK_DIST0_THD_0                       4500
#define STK_DIST0_THD_1                       4500
#define STK_DIST0_THD_2                       4500
#define STK_DIST0_THD_3                       4500
#define STK_DIST0_THD_4                       4500
#define STK_DIST0_THD_5                       4500
#define STK_DIST0_THD_6                       4500
#define STK_DIST0_THD_7                       4500

//Below for STK_DIST_1_EN use
#define STK_DIST1_THD_0                       9000
#define STK_DIST1_THD_1                       9000
#define STK_DIST1_THD_2                       9000
#define STK_DIST1_THD_3                       9000
#define STK_DIST1_THD_4                       9000
#define STK_DIST1_THD_5                       9000
#define STK_DIST1_THD_6                       9000
#define STK_DIST1_THD_7                       9000
//end of STK_DIST_1_EN use

#define STK_CADC_DIFF                       30

#define STK_COEF_T_POS_PH0                  128    //follow by STK_ADDR_CORRECTION_PH0 [17:08]
#define STK_COEF_T_NEG_PH0                  128    //follow by STK_ADDR_CORRECTION_PH0 [29:20]
#define STK_COEF_T_POS_PH1                  128    //follow by STK_ADDR_CORRECTION_PH1 [17:08]
#define STK_COEF_T_NEG_PH1                  128    //follow by STK_ADDR_CORRECTION_PH1 [29:20]
#define STK_COEF_T_POS_PH2                  128    //follow by STK_ADDR_CORRECTION_PH2 [17:08]
#define STK_COEF_T_NEG_PH2                  128    //follow by STK_ADDR_CORRECTION_PH2 [29:20]
#define STK_COEF_T_POS_PH3                  128    //follow by STK_ADDR_CORRECTION_PH3 [17:08]
#define STK_COEF_T_NEG_PH3                  128    //follow by STK_ADDR_CORRECTION_PH3 [29:20]
#define STK_COEF_T_POS_PH4                  128    //follow by STK_ADDR_CORRECTION_PH4 [17:08]
#define STK_COEF_T_NEG_PH4                  128    //follow by STK_ADDR_CORRECTION_PH4 [29:20]
#define STK_COEF_T_POS_PH5                  128    //follow by STK_ADDR_CORRECTION_PH5 [17:08]
#define STK_COEF_T_NEG_PH5                  128    //follow by STK_ADDR_CORRECTION_PH5 [29:20]
#define STK_COEF_T_POS_PH6                  128    //follow by STK_ADDR_CORRECTION_PH6 [17:08]
#define STK_COEF_T_NEG_PH6                  128    //follow by STK_ADDR_CORRECTION_PH6 [29:20]
#define STK_COEF_T_POS_PH7                  128    //follow by STK_ADDR_CORRECTION_PH7 [17:08]
#define STK_COEF_T_NEG_PH7                  128    //follow by STK_ADDR_CORRECTION_PH7 [29:20]

#define STK501XX_ID                         0x6503
#define STK_POLLING_TIME                    100000//us
#define IDLE_WAITING_6MS                    (600000/STK_POLLING_TIME) // default waintg 6ms.

#define CHANNEL_NUM                         8

#define MAJOR_PHASE \
  ((0 << 0) |  /* ph0 */ \
   (1 << 1) |  /* ph1 */ \
   (1 << 2) |  /* ph2 */ \
   (1 << 3) |  /* ph3 */ \
   (1 << 4) |  /* ph4 */ \
   (0 << 5) |  /* ph5 */ \
   (0 << 6) |  /* ph6 */ \
   (0 << 7)    /* ph7 */)

#define REF_PHASE \
  ((1 << 0) |  /* ph0 */ \
   (0 << 1) |  /* ph1 */ \
   (0 << 2) |  /* ph2 */ \
   (0 << 3) |  /* ph3 */ \
   (0 << 4) |  /* ph4 */ \
   (0 << 5) |  /* ph5 */ \
   (0 << 6) |  /* ph6 */ \
   (0 << 7)    /* ph7 */)

#define PHASE_EN (MAJOR_PHASE|REF_PHASE)

#define STK_DIST_1_EN  \
  ((0 << 0) |  /* ph0 */ \
   (1 << 1) |  /* ph1 */ \
   (0 << 2) |  /* ph2 */ \
   (1 << 3) |  /* ph3 */ \
   (0 << 4) |  /* ph4 */ \
   (0 << 5) |  /* ph5 */ \
   (0 << 6) |  /* ph6 */ \
   (0 << 7)    /* ph7 */)

#define STK_DIST_2_EN \
  ((0 << 0) |  /* ph0 */ \
   (0 << 1) |  /* ph1 */ \
   (0 << 2) |  /* ph2 */ \
   (0 << 3) |  /* ph3 */ \
   (0 << 4) |  /* ph4 */ \
   (0 << 5) |  /* ph5 */ \
   (0 << 6) |  /* ph6 */ \
   (0 << 7)    /* ph7 */)

#define STK_DIST_3_EN  \
  ((0 << 0) |  /* ph0 */ \
   (0 << 1) |  /* ph1 */ \
   (0 << 2) |  /* ph2 */ \
   (0 << 3) |  /* ph3 */ \
   (0 << 4) |  /* ph4 */ \
   (0 << 5) |  /* ph5 */ \
   (0 << 6) |  /* ph6 */ \
   (0 << 7)    /* ph7 */)

//Phase PAD usage
#define STK_PH_OPT       0x0
#define STK_PH_HZ        0x1
#define STK_PH_SHIELD    0x3
#define STK_PH_MEASURE   0x4

//RXIO mapping to phase and PH PAD usage
//(phase),(phase PAD usage)
#define STK_RXIO_0  {0, STK_PH_MEASURE}
#define STK_RXIO_1  {1, STK_PH_MEASURE}
#define STK_RXIO_2  {2, STK_PH_MEASURE}
#define STK_RXIO_3  {0, STK_PH_OPT}
#define STK_RXIO_4  {4, STK_PH_MEASURE}
#define STK_RXIO_5  {5, STK_PH_MEASURE}
#define STK_RXIO_6  {6, STK_PH_MEASURE}
#define STK_RXIO_7  {7, STK_PH_MEASURE}

#define STK_RXIO_MAP \
{ \
    STK_RXIO_0, \
    STK_RXIO_1, \
    STK_RXIO_2, \
    STK_RXIO_3, \
    STK_RXIO_4, \
    STK_RXIO_5, \
    STK_RXIO_6, \
    STK_RXIO_7 \
}

//Auto generate RXIO value depend on phase function
//ph_idx: input phase index
//val: PAD usage(STK_RXIO_HZ, STK_RXIO_HZ, STK_RXIO_MS)
#define STK_RXIO_SET(ph_idx,val) \
((0x22222222 & (~(0x00000000 |(0xF << ((ph_idx)*4)))))\
| ((val) << ((ph_idx)*4)))

//Auto generate CUSTOM IRQ value depend on DIST_x_EN
//en_bit: input STK_DIST_x_EN define value
//dist_idx: input DSIT_x x value
//ph_idx: input phase index
//output: CUSTOM_CTRL value
#define STK_DIST_CUST_IRQ(en_bit, dist_idx, ph_idx) ( \
    (en_bit & (1 << (1 + ph_idx))) ? \
    (((1 + ph_idx) << 4) | (1 << (dist_idx)))|\
    (((1 + ph_idx) << 4) | (1 << (dist_idx))) << 8 |\
    (((1 + ph_idx) << 4) | (1 << (dist_idx))) << 16 |\
    (((1 + ph_idx) << 4) | (1 << (dist_idx))) << 24 : 0 \
)

typedef struct stk501xx_register_table
{
    uint16_t address;
    uint32_t value;
} stk501xx_register_table;

typedef struct stk501xx_rxio_map
{
    uint32_t phase_idx;
    uint32_t phase_usage;
} stk501xx_rxio_map;

enum
{
    STK_SAR_FAR_AWAY        = 0,
    STK_SAR_NEAR_BY_DIST_0  = 1,
    STK_SAR_NEAR_BY_DIST_1  = 2,
    STK_SAR_NEAR_BY_DIST_2  = 3,
    STK_SAR_NEAR_BY_DIST_3  = 4,
    STK_SAR_NEAR_BY_UNKNOWN = 5
};

/*****************************************************************************
 * stk501xx register, start
 *****************************************************************************/
#define STK_ADDR_IRQ_SOURCE                      0x0000
#define STK_IRQ_SOURCE_CLOSE_IRQ_MASK            0x02
#define STK_IRQ_SOURCE_FAR_IRQ_MASK              0x04
#define STK_IRQ_SOURCE_CONVDONE_IRQ_MASK         0x10
#define STK_IRQ_SOURCE_CUST_A_IRQ_MASK           0x20
#define STK_IRQ_SOURCE_CUST_B_IRQ_MASK           0x40
#define STK_IRQ_SOURCE_CUST_C_IRQ_MASK           0x80
#define STK_IRQ_SOURCE_CUST_D_IRQ_MASK           0x100
#define STK_IRQ_SOURCE_SENSING_WDT_IRQ_MASK      0x00100000
#define STK_IRQ_SOURCE_I2C_WDT_IRQ_MASK          0x00200000

#define STK_ADDR_IRQ_SOURCE_ENABLE_REG                      0x0004
#define STK_IRQ_SOURCE_ENABLE_REG_CLOSE_ANY_IRQ_EN_SHIFT    1
#define STK_IRQ_SOURCE_ENABLE_REG_CLOSE_ANY_IRQ_EN_MASK     0x00000002

#define STK_IRQ_SOURCE_ENABLE_REG_FAR_ANY_IRQ_EN_SHIFT      2
#define STK_IRQ_SOURCE_ENABLE_REG_FAR_ANY_IRQ_EN_MASK       0x00000004

#define STK_IRQ_SOURCE_ENABLE_REG_PHRST_IRQ_EN_SHIFT        3
#define STK_IRQ_SOURCE_ENABLE_REG_PHRST_IRQ_EN_MASK         0x00000008

#define STK_IRQ_SOURCE_ENABLE_REG_CONVDONE_IRQ_EN_SHIFT     4
#define STK_IRQ_SOURCE_ENABLE_REG_CONVDONE_IRQ_EN_MASK      0x00000010

#define STK_IRQ_SOURCE_ENABLE_REG_CUST_A_IRQ_EN_SHIFT       5
#define STK_IRQ_SOURCE_ENABLE_REG_CUST_A_IRQ_EN_MASK        0x00000020

#define STK_IRQ_SOURCE_ENABLE_REG_CUST_B_IRQ_EN_SHIFT       6
#define STK_IRQ_SOURCE_ENABLE_REG_CUST_B_IRQ_EN_MASK        0x00000040

#define STK_IRQ_SOURCE_ENABLE_REG_CUST_C_IRQ_EN_SHIFT       7
#define STK_IRQ_SOURCE_ENABLE_REG_CUST_C_IRQ_EN_MASK        0x00000080

#define STK_IRQ_SOURCE_ENABLE_REG_CUST_D_IRQ_EN_SHIFT       8
#define STK_IRQ_SOURCE_ENABLE_REG_CUST_D_IRQ_EN_MASK        0x00000100

#define STK_IRQ_SOURCE_ENABLE_REG_DELTA_DES_IRQ_EN_SHIFT    13
#define STK_IRQ_SOURCE_ENABLE_REG_DELTA_DES_IRQ_EN_MASK     0x00002000

#define STK_IRQ_SOURCE_ENABLE_REG_SATURATION_IRQ_EN_SHIFT   18
#define STK_IRQ_SOURCE_ENABLE_REG_SATURATION_IRQ_EN_MASK    0x00040000

#define STK_ADDR_SOFT_RESET                     0x0034
#define STK_SOFT_RESET_CMD                      0x00A5

#define STK_ADDR_CHIP_INDEX                     0x0040
#define STK_CHIP_INDEX_CHIP_ID__MASK            0xFFFF0000
#define STK_CHIP_INDEX_CHIP_ID__SHIFT           16
#define STK_CHIP_INDEX_F__MASK                  0x000000FF

#define STK_ADDR_RXIO0_MUX_REG                  0x0058
//#define STK_RXIO0_MUX_REG_VALUE                 0x22222224 //PH0

#define STK_ADDR_RXIO1_MUX_REG                  0x005C
//#define STK_RXIO1_MUX_REG_VALUE                 0x22222242 //PH1

#define STK_ADDR_RXIO2_MUX_REG                  0x0060
//#define STK_RXIO2_MUX_REG_VALUE                 0x22222422 //PH2

#define STK_ADDR_RXIO3_MUX_REG                  0x0064
//#define STK_RXIO3_MUX_REG_VALUE                 0x00000000 //Digital mode

#define STK_ADDR_RXIO4_MUX_REG                  0x0068
//#define STK_RXIO4_MUX_REG_VALUE                 0x22242222

#define STK_ADDR_RXIO5_MUX_REG                  0x006C
//#define STK_RXIO5_MUX_REG_VALUE                 0x22422222

#define STK_ADDR_RXIO6_MUX_REG                  0x0070
//#define STK_RXIO6_MUX_REG_VALUE                 0x24222222

#define STK_ADDR_RXIO7_MUX_REG                  0x0074
//#define STK_RXIO7_MUX_REG_VALUE                 0x42222222

#define STK_ADDR_ADP_BASELINE_0                 0x00B0
#define STK_ADP_BASELINE_0_VALUE                0x00050000

#define STK_ADDR_ADP_BASELINE_1                 0x00B4
#define STK_ADP_BASELINE_1_VALUE                0x10101010

#define STK_ADDR_ADP_BASELINE_2                 0x00B8
#define STK_ADP_BASELINE_2_VALUE                0x10101010

#define STK_ADDR_IRQ_CONFIG                     0x00C4
#define STK_IRQ_CONFIG_SENS_RATE_OPT_SHIFT      24

#define STK_ADDR_SCAN_PERIOD                    0x00D0
#define STK_SCAN_PERIOD_VALUE                   0x000007D0

#define STK_ADDR_WATCH_DOG                      0x00D4
#define STK_SNS_WATCH_DOG_MASK                  0x000000FF
#define STK_SNS_WATCH_DOG_VALUE                 0x000000FF
#define STK_I2C_WDT_MASK                        0x0000FF00
#define STK_I2C_WDT_VALUE                       0x00004000

#define STK_ADDR_TRIGGER_CMD                    0x0100
#define STK_TRIGGER_CMD_REG_INIT_ALL            0x0000000F
#define STK_TRIGGER_CMD_REG_BY_PHRST            0x0000000A

#define STK_ADDR_TRIGGER_REG                    0x0104
#define STK_TRIGGER_REG_PHEN_SHIFT              0
#define STK_TRIGGER_REG_PHEN_MASK               0xFF
#define STK_TRIGGER_REG_PHEN_DISABLE_ALL        0x00000000

#define STK_TRIGGER_REG_INIT_ALL(ph_en)         (((ph_en) << 8) | (ph_en))
#define STK_TRIGGER_REG_PHRST_PHASE             (1 << (8 + DELDEA_A_MAPPING_PHASE))

#define STK_TRIGGER_REG_ENTER_PAUSE_MODE        0x0

#define STK_ADDR_RX_NL_CTRL                     0x0134

#define STK_ADDR_FAIL_STAT_DET_2                0x0148
#define STK_FAIL_STAT_DET_2_VALUE               0x08003F00

#define STK_ADDR_DETECT_STATUS_1                0x0184
#define STK_DETECT_STATUS_1_PROX_STATE_MASK     0xFF00
#define STK_DETECT_STATUS_1_SATURATION_STATE_MASK     0x0000FF

#define STK_ADDR_DETECT_STATUS_3                0x018C

#define STK_ADDR_DETECT_STATUS_4                0x0190
#define STK_DETECT_STATUS_4_DES_STAT_A_MASK     0x01
#define STK_DETECT_STATUS_4_DES_STAT_B_MASK     0x02
#define STK_DETECT_STATUS_4_DES_STAT_C_MASK     0x04

//Each pahse contol reg
#define STK_ADDR_SCAN_OPT_PH0                   0x0200
#define STK_ADDR_TX_CTRL_PH0                    0x0208
#define STK_ADDR_SENS_CTRL_PH0                  0x020C
#define STK_ADDR_FILT_CFG_PH0                   0x0210
#define STK_ADDR_CORRECTION_PH0                 0x0214
#define STK_ADDR_NOISE_DECT_PH0                 0x0218
#define STK_ADDR_CADC_OPT0_PH0                  0x021C
#define STK_ADDR_STARTUP_THD_PH0                0x0224
#define STK_ADDR_PROX_CTRL0_PH0                 0x022C
#define STK_ADDR_PROX_CTRL1_PH0                 0x0230

#define STK_SCAN_OPT_PH0_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH0_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH0_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH0_VALUE                 0x10004003
#define STK_FILT_CFG_PH0_VALUE                  0x00000111
#define STK_CORRECTION_PH0_VALUE                0x08008000
#define STK_NOISE_DECT_PH0_VALUE                0x00000004
#define STK_CADC_OPT0_PH0_VALUE                 0x00060000
#define STK_STARTUP_THD_PH0_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH0_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH0_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH0_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH0_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH1                   0x0240
#define STK_ADDR_TX_CTRL_PH1                    0x0248
#define STK_ADDR_SENS_CTRL_PH1                  0x024C
#define STK_ADDR_FILT_CFG_PH1                   0x0250
#define STK_ADDR_CORRECTION_PH1                 0x0254
#define STK_ADDR_NOISE_DECT_PH1                 0x0258
#define STK_ADDR_CADC_OPT0_PH1                  0x025C
#define STK_ADDR_STARTUP_THD_PH1                0x0264
#define STK_ADDR_PROX_CTRL0_PH1                 0x026C
#define STK_ADDR_PROX_CTRL1_PH1                 0x0270

#define STK_SCAN_OPT_PH1_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH1_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH1_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH1_VALUE                 0x10004003
#define STK_FILT_CFG_PH1_VALUE                  0x00000111
#define STK_CORRECTION_PH1_VALUE                0x08008000
#define STK_NOISE_DECT_PH1_VALUE                0x00000004
#define STK_CADC_OPT0_PH1_VALUE                 0x00060000
#define STK_STARTUP_THD_PH1_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH1_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH1_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH1_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH1_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH2                   0x0280
#define STK_ADDR_TX_CTRL_PH2                    0x0288
#define STK_ADDR_SENS_CTRL_PH2                  0x028C
#define STK_ADDR_FILT_CFG_PH2                   0x0290
#define STK_ADDR_CORRECTION_PH2                 0x0294
#define STK_ADDR_NOISE_DECT_PH2                 0x0298
#define STK_ADDR_CADC_OPT0_PH2                  0x029C
#define STK_ADDR_STARTUP_THD_PH2                0x02A4
#define STK_ADDR_PROX_CTRL0_PH2                 0x02AC
#define STK_ADDR_PROX_CTRL1_PH2                 0x02B0

#define STK_SCAN_OPT_PH2_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH2_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH2_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH2_VALUE                 0x10004003
#define STK_FILT_CFG_PH2_VALUE                  0x00000111
#define STK_CORRECTION_PH2_VALUE                0x08008000
#define STK_NOISE_DECT_PH2_VALUE                0x00000004
#define STK_CADC_OPT0_PH2_VALUE                 0x00060000
#define STK_STARTUP_THD_PH2_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH2_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH2_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH2_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH2_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH3                   0x02C0
#define STK_ADDR_TX_CTRL_PH3                    0x02C8
#define STK_ADDR_SENS_CTRL_PH3                  0x02CC
#define STK_ADDR_FILT_CFG_PH3                   0x02D0
#define STK_ADDR_CORRECTION_PH3                 0x02D4
#define STK_ADDR_NOISE_DECT_PH3                 0x02D8
#define STK_ADDR_CADC_OPT0_PH3                  0x02DC
#define STK_ADDR_STARTUP_THD_PH3                0x02E4
#define STK_ADDR_PROX_CTRL0_PH3                 0x02EC
#define STK_ADDR_PROX_CTRL1_PH3                 0x02F0

#define STK_SCAN_OPT_PH3_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH3_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH3_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH3_VALUE                 0x10004003
#define STK_FILT_CFG_PH3_VALUE                  0x00000111
#define STK_CORRECTION_PH3_VALUE                0x08008000
#define STK_NOISE_DECT_PH3_VALUE                0x00000004
#define STK_CADC_OPT0_PH3_VALUE                 0x00060000
#define STK_STARTUP_THD_PH3_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH3_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH3_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH3_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH3_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH4                   0x0300
#define STK_ADDR_TX_CTRL_PH4                    0x0308
#define STK_ADDR_SENS_CTRL_PH4                  0x030C
#define STK_ADDR_FILT_CFG_PH4                   0x0310
#define STK_ADDR_CORRECTION_PH4                 0x0314
#define STK_ADDR_NOISE_DECT_PH4                 0x0318
#define STK_ADDR_CADC_OPT0_PH4                  0x031C
#define STK_ADDR_STARTUP_THD_PH4                0x0324
#define STK_ADDR_PROX_CTRL0_PH4                 0x032C
#define STK_ADDR_PROX_CTRL1_PH4                 0x0330

#define STK_SCAN_OPT_PH4_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH4_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH4_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH4_VALUE                 0x10004003
#define STK_FILT_CFG_PH4_VALUE                  0x00000111
#define STK_CORRECTION_PH4_VALUE                0x08008000
#define STK_NOISE_DECT_PH4_VALUE                0x00000004
#define STK_CADC_OPT0_PH4_VALUE                 0x00060000
#define STK_STARTUP_THD_PH4_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH4_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH4_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH4_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH4_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH5                   0x0340
#define STK_ADDR_TX_CTRL_PH5                    0x0348
#define STK_ADDR_SENS_CTRL_PH5                  0x034C
#define STK_ADDR_FILT_CFG_PH5                   0x0350
#define STK_ADDR_CORRECTION_PH5                 0x0354
#define STK_ADDR_NOISE_DECT_PH5                 0x0358
#define STK_ADDR_CADC_OPT0_PH5                  0x035C
#define STK_ADDR_STARTUP_THD_PH5                0x0364
#define STK_ADDR_PROX_CTRL0_PH5                 0x036C
#define STK_ADDR_PROX_CTRL1_PH5                 0x0370

#define STK_SCAN_OPT_PH5_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH5_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH5_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH5_VALUE                 0x10004003
#define STK_FILT_CFG_PH5_VALUE                  0x00000111
#define STK_CORRECTION_PH5_VALUE                0x08008000
#define STK_NOISE_DECT_PH5_VALUE                0x00000004
#define STK_CADC_OPT0_PH5_VALUE                 0x00060000
#define STK_STARTUP_THD_PH5_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH5_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH5_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH5_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH5_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH6                   0x0380
#define STK_ADDR_TX_CTRL_PH6                    0x0388
#define STK_ADDR_SENS_CTRL_PH6                  0x038C
#define STK_ADDR_FILT_CFG_PH6                   0x0390
#define STK_ADDR_CORRECTION_PH6                 0x0394
#define STK_ADDR_NOISE_DECT_PH6                 0x0398
#define STK_ADDR_CADC_OPT0_PH6                  0x039C
#define STK_ADDR_STARTUP_THD_PH6                0x03A4
#define STK_ADDR_PROX_CTRL0_PH6                 0x03AC
#define STK_ADDR_PROX_CTRL1_PH6                 0x03B0

#define STK_SCAN_OPT_PH6_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH6_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH6_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH6_VALUE                 0x10004003
#define STK_FILT_CFG_PH6_VALUE                  0x00000111
#define STK_CORRECTION_PH6_VALUE                0x08008000
#define STK_NOISE_DECT_PH6_VALUE                0x00000004
#define STK_CADC_OPT0_PH6_VALUE                 0x00060000
#define STK_STARTUP_THD_PH6_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH6_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH6_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH6_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH6_VALUE                0x00000007
#endif

#define STK_ADDR_SCAN_OPT_PH7                   0x03C0
#define STK_ADDR_TX_CTRL_PH7                    0x03C8
#define STK_ADDR_SENS_CTRL_PH7                  0x03CC
#define STK_ADDR_FILT_CFG_PH7                   0x03D0
#define STK_ADDR_CORRECTION_PH7                 0x03D4
#define STK_ADDR_NOISE_DECT_PH7                 0x03D8
#define STK_ADDR_CADC_OPT0_PH7                  0x03DC
#define STK_ADDR_STARTUP_THD_PH7                0x03E4
#define STK_ADDR_PROX_CTRL0_PH7                 0x03EC
#define STK_ADDR_PROX_CTRL1_PH7                 0x03F0

#define STK_SCAN_OPT_PH7_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH7_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH7_VALUE                   0x06000B0B //66.67kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH7_VALUE                 0x10004003
#define STK_FILT_CFG_PH7_VALUE                  0x00000111
#define STK_CORRECTION_PH7_VALUE                0x08008000
#define STK_NOISE_DECT_PH7_VALUE                0x00000004
#define STK_CADC_OPT0_PH7_VALUE                 0x00060000
#define STK_STARTUP_THD_PH7_VALUE               0x3FFFFF00
#ifdef MCU_GESTURE
    #define STK_PROX_CTRL0_PH7_VALUE                0xFFFFFF80
    #define STK_PROX_CTRL1_PH7_VALUE                0x00000007
#else
    #define STK_PROX_CTRL0_PH7_VALUE                0x32323232
    #define STK_PROX_CTRL1_PH7_VALUE                0x00000007
#endif

#define STK_ADDR_DELTADES_A_CTRL                0x0400
#define STK_ADDR_DELTADES_B_CTRL                0x0404
#define STK_ADDR_DELTADES_C_CTRL                0x0408

#define STK_ADDR_CORR_ENGA_0                    0x0430
#define STK_ADDR_CORR_ENGA_1                    0x0434
#define STK_ADDR_CORR_ENGB_0                    0x0438
#define STK_ADDR_CORR_ENGB_1                    0x043C
#define STK_ADDR_CORR_ENGC_0                    0x0440
#define STK_ADDR_CORR_ENGC_1                    0x0444
#define STK_ADDR_CORR_ENGD_0                    0x0448
#define STK_ADDR_CORR_ENGD_1                    0x044C

#define STK_ADDR_CUSTOM_A_CTRL0                 0x0480
#define STK_ADDR_CUSTOM_A_CTRL1                 0x0484
#define STK_ADDR_CUSTOM_B_CTRL0                 0x0488
#define STK_ADDR_CUSTOM_B_CTRL1                 0x048C
#define STK_ADDR_CUSTOM_C_CTRL0                 0x0490
#define STK_ADDR_CUSTOM_C_CTRL1                 0x0494
#define STK_ADDR_CUSTOM_D_CTRL0                 0x0498
#define STK_ADDR_CUSTOM_D_CTRL1                 0x049C


#define STK_ADDR_REG_RAW_PH0_REG                0x0500
#define STK_ADDR_REG_RAW_PH1_REG                0x0504
#define STK_ADDR_REG_RAW_PH2_REG                0x0508
#define STK_ADDR_REG_RAW_PH3_REG                0x050C
#define STK_ADDR_REG_RAW_PH4_REG                0x0510
#define STK_ADDR_REG_RAW_PH5_REG                0x0514
#define STK_ADDR_REG_RAW_PH6_REG                0x0518
#define STK_ADDR_REG_RAW_PH7_REG                0x051C

#define STK_ADDR_REG_DELTA_PH0_REG              0x0540
#define STK_ADDR_REG_DELTA_PH1_REG              0x0544
#define STK_ADDR_REG_DELTA_PH2_REG              0x0548
#define STK_ADDR_REG_DELTA_PH3_REG              0x054C
#define STK_ADDR_REG_DELTA_PH4_REG              0x0550
#define STK_ADDR_REG_DELTA_PH5_REG              0x0554
#define STK_ADDR_REG_DELTA_PH6_REG              0x0558
#define STK_ADDR_REG_DELTA_PH7_REG              0x055C

#define STK_ADDR_REG_CADC_PH0_REG               0x0560
#define STK_ADDR_REG_CADC_PH1_REG               0x0564
#define STK_ADDR_REG_CADC_PH2_REG               0x0568
#define STK_ADDR_REG_CADC_PH3_REG               0x056C
#define STK_ADDR_REG_CADC_PH4_REG               0x0570
#define STK_ADDR_REG_CADC_PH5_REG               0x0574
#define STK_ADDR_REG_CADC_PH6_REG               0x0578
#define STK_ADDR_REG_CADC_PH7_REG               0x057C

#define STK_ADDR_INHOUSE_CMD                    0x0A00
#define STK_ADDR_TRIM_LOCK                      0x0010
#define STK_ADDR_CADC_SMOOTH                    0x0120
#define STK_CADC_REG__FIX_VALUE__MASK           0x00060001

#define STK_ADDR_REG_TCON_PWR                   0x0740
#define STK_ADDR_REG_TCON_LIGHT_LOAD_0          0x0744
#define STK_ADDR_REG_TCON_LIGHT_LOAD_1          0x0748
#define STK_ADDR_REG_TCON_LIGHT_LOAD_2          0x074C
#define STK_ADDR_REG_TCON_MEDIUM_LOAD_0         0x0750
#define STK_ADDR_REG_TCON_MEDIUM_LOAD_1         0x0754
#define STK_ADDR_REG_TCON_MEDIUM_LOAD_2         0x0758
#define STK_ADDR_REG_TCON_HEAVY_LOAD_0          0x075C
#define STK_ADDR_REG_TCON_HEAVY_LOAD_1          0x0760
#define STK_ADDR_REG_TCON_HEAVY_LOAD_2          0x0764


/*****************************************************************************
 * stk501xx register, end
 *****************************************************************************/
#define STK501XX_NAME    "stk501xx"
static const uint16_t STK_ID[1] = { STK501XX_ID };

typedef struct stk_data stk_data;
typedef void (*STK_REPORT_CB)(struct stk_data *);


#ifdef TEMP_COMPENSATION

    typedef struct  temp_comp_config
    {
         /* Modify by design*/
        uint32_t delta_temp_thd;
        uint32_t mapping_ref_phase; // ref phase, 0~7 mapping phase 0~7, this for phase 1
        uint32_t major_phase;       // measure phase
        uint32_t reinit_phase[3];   // base reinit phase
        //below auto gen dont edit
        uint16_t mapping_ref_phase_reg;
        uint32_t delta_des_val;
    }temp_comp_config;

    #define MAPING_REF_PHASE(ref_ph_idx)           (STK_ADDR_REG_RAW_PH0_REG + ((ref_ph_idx)*4))
    #define DELTA_DES_CTRL_VAL(ctrl_thd, deb_clr, deb_set, major_ph_idx) (0x01 | ((ctrl_thd) <<16) | ((deb_clr) <<12) | ((deb_set) <<8) | ((major_ph_idx) <<4))
    #define BASE_REINIT_DELTA_DES                  0x200000

    /*Modify by design*/
    #define DELTA_TEMP_THD_A                       80000
    #define DELTA_A_MAPPING_PHASE                  0 // ref channel, 0~7 mapping phase 0~7, this for phase 1
    #define DELTA_A_MEASURE_PHASE                  1 // measure phase
    #define DELTA_A_DES_THD                        20 // measure phase
    #define DELTA_A_DES_DEB_CLR                    3 // debounce clear criteria
    #define DELTA_A_DES_DEB_SET                    3 // debounce setting criteria

    #define DELTA_TEMP_THD_B                       80000
    #define DELTA_B_MAPPING_PHASE                  2 // ref channel, 0~7 mapping phase 0~7, this for phase 3
    #define DELTA_B_MEASURE_PHASE                  3 //  measure phase
    #define DELTA_B_DES_THD                        20 // measure phase
    #define DELTA_B_DES_DEB_CLR                    3 // debounce clear criteria
    #define DELTA_B_DES_DEB_SET                    3 // debounce setting criteria

    #define DELTA_TEMP_THD_C                       80000
    #define DELTA_C_MAPPING_PHASE                  4 // ref channel, 0~7 mapping phase 0~7, this for phase 5
    #define DELTA_C_MEASURE_PHASE                  5 //  measure phase
    #define DELTA_C_DES_THD                        20 // measure phase
    #define DELTA_C_DES_DEB_CLR                    3 // debounce clear criteria
    #define DELTA_C_DES_DEB_SET                    3 // debounce setting criteria

    /*End of modify by design*/

#endif // TEMP_COMPENSATION

struct stk501xx_platform_data
{
    uint32_t                        interrupt_int1_pin;
    uint32_t                        ch_num;
    uint32_t                        replace_reg_num;
    struct stk501xx_register_table *replace_reg_val;
    uint32_t                        major_phase_arr[8];
    uint32_t                        ref_phase_arr[8];
    uint32_t                        mapping_phase[8];
    uint32_t                        dist0_thd[8];
    uint32_t                        dist1_thd[8];
    uint32_t                        dist1_en_arr[8];
    uint32_t                        tc_config_a[6]; // delta_temp_thd, mapping_ref_phase, major_phase, base_reinit_ph_1, base_reinit_ph_2, base_reinit_ph_3
    uint32_t                        tc_config_b[6]; // delta_temp_thd, mapping_ref_phase, major_phase, base_reinit_ph_1, base_reinit_ph_2, base_reinit_ph_3
    uint32_t                        tc_config_c[6]; // delta_temp_thd, mapping_ref_phase, major_phase, base_reinit_ph_1, base_reinit_ph_2, base_reinit_ph_3
    uint32_t                        tc_ctrl_des_a[3]; //delta_ctrl_thd, delta_deb_clr, delta_deb_set
    uint32_t                        tc_ctrl_des_b[3]; //delta_ctrl_thd, delta_deb_clr, delta_deb_set
    uint32_t                        tc_ctrl_des_c[3]; //delta_ctrl_thd, delta_deb_clr, delta_deb_set
    struct stk501xx_rxio_map        rxio_map[8]; // rxio mapping phase index and usage
};

struct stk_data
{
    const struct stk_bus_ops        *bops;
    const struct stk_timer_ops      *tops;
    const struct stk_gpio_ops       *gops;
    STK_REPORT_CB                   sar_report_cb;
    int32_t                         bus_idx;
    bool                            first_init;
    bool                            enabled;            /* chip is enabled or not */
    bool                            last_enable;        /* record current power status. For Suspend/Resume used. */
    uint8_t                         power_mode;
    uint8_t                         recv;
    uint8_t                         last_nearby[8];
    uint8_t                         last_nearby_dist1[8];
    uint8_t                         state_change[8];
    uint8_t                         state_change_dist1[8];
    int32_t                         last_data[8];
    uint32_t                        last_cadc[8];
    uint32_t                        fix_cadc[8];
    uint32_t                        chip_id;
    uint32_t                        chip_index;
    bool                            wait_phase_reset;
    stk_timer_info                  phase_reset_timer_info;
    uint32_t                        major_phase; // MAJOR_PHASE
    uint32_t                        ref_phase; //REF_PHASE
    uint32_t                        phase_en; // PHASE_EN
    uint32_t                        dist1_en;
    struct stk501xx_platform_data   *pdata;
#ifdef TEMP_COMPENSATION
    bool                            last_prox_a_state;
    bool                            last_prox_b_state;
    bool                            last_prox_c_state;
    int32_t                         prev_temperature_ref_a;
    int32_t                         next_temperature_ref_a;
    int32_t                         prev_temperature_ref_b;
    int32_t                         next_temperature_ref_b;
    int32_t                         prev_temperature_ref_c;
    int32_t                         next_temperature_ref_c;
    bool reinit[8];
    uint8_t                         descent_cnt_a;
    uint8_t                         descent_cnt_b;
    uint8_t                         descent_cnt_c;
    struct temp_comp_config         tc_config_a;
    struct temp_comp_config         tc_config_b;
    struct temp_comp_config         tc_config_c;
#endif
#ifdef STK_INTERRUPT_MODE
    stk_gpio_info                   gpio_info;
    int32_t                         int_pin;
#endif
#if (defined STK_POLLING_MODE || defined MCU_GESTURE)
    stk_timer_info                  stk_timer_info;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
#ifdef MCU_GESTURE
    GestureType                     gesture_state;
    bool                            gs_timer_is_running;
    uint8_t                         gs_idle_count;
#endif
    bool                            dis_conv_done_chk;
#ifdef STK_FIX_CADC
    bool                            is_cali;
#endif
};


#define STK501XX_SAR_REPORT(stk_data)                        if ((stk_data)->sar_report_cb)          ((stk_data)->sar_report_cb(stk_data))

void stk501xx_set_enable(struct stk_data* stk, char enable, bool pause_mode);
int32_t stk_read_prox_flag(struct stk_data* stk, uint32_t* prox_flag);
void stk501xx_read_temp_data(struct stk_data* stk, uint16_t reg, int32_t *temperature);
void stk501xx_read_sar_data(struct stk_data* stk, uint32_t prox_flag);
int8_t stk501xx_set_each_thd(struct stk_data* stk, uint8_t idx, uint32_t thd);
int32_t stk501xx_show_all_reg(struct stk_data* stk);
int32_t stk501xx_init_client(struct stk_data *stk);
int32_t stk501xx_sw_reset(struct stk_data* stk);
int32_t stk501xx_read(struct stk_data* stk, unsigned short addr, void *buf);
int32_t stk501xx_write(struct stk_data* stk, unsigned short addr, unsigned char* val);
void stk501xx_data_initialize(struct stk_data* stk);
void stk501xx_phase_reset(struct stk_data* stk, uint32_t phase_reset_reg);
void stk_work_queue(void *stkdata);
void stk_clr_intr(struct stk_data* stk, uint32_t* flag);
void clr_temp(struct stk_data* stk);
void stk501xx_disable_fix_cadc(struct stk_data* stk, uint8_t fix_ph);
/* Algorithm loading in the last*/
#ifdef MCU_GESTURE
    #include "stk_gesture_lib.h"
#endif
#endif /* __STK501XX_H__ */
