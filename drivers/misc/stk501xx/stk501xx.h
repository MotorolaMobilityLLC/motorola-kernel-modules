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
static struct stk501xx_platform_data *pdata = NULL;

enum{
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
#define STK_SAR_THD_0                       (pdata?pdata->sar_thd0[0]:260000)  //ref
#define STK_SAR_THD_1                       (pdata?pdata->sar_thd0[1]:20000)
#define STK_SAR_THD_2                       (pdata?pdata->sar_thd0[2]:8000)
#define STK_SAR_THD_3                       (pdata?pdata->sar_thd0[3]:260000)  //ref
#define STK_SAR_THD_4                       (pdata?pdata->sar_thd0[4]:10000)
#define STK_SAR_THD_5                       (pdata?pdata->sar_thd0[5]:14000)
#define STK_SAR_THD_6                       (pdata?pdata->sar_thd0[6]:1000)
#define STK_SAR_THD_7                       (pdata?pdata->sar_thd0[7]:260000)

#define STK_SAR_THD1_0                      (pdata?pdata->sar_thd1[0]:260000)
#define STK_SAR_THD1_1                      (pdata?pdata->sar_thd1[1]:10000)
#define STK_SAR_THD1_2                      (pdata?pdata->sar_thd1[2]:260000)
#define STK_SAR_THD1_3                      (pdata?pdata->sar_thd1[3]:260000)
#define STK_SAR_THD1_4                      (pdata?pdata->sar_thd1[4]:260000)
#define STK_SAR_THD1_5                      (pdata?pdata->sar_thd1[5]:7000)
#define STK_SAR_THD1_6                      (pdata?pdata->sar_thd1[6]:260000)
#define STK_SAR_THD1_7                      (pdata?pdata->sar_thd1[7]:260000)


#define STK_CADC_DIFF                       30

#define STK_COEF_T_POS_PH0 (pdata?pdata->coef_t[0]:128) // follow by STK_ADDR_CORRECTION_PH0 [17:08]
#define STK_COEF_T_NEG_PH0 (pdata?pdata->coef_t[1]:128) // follow by STK_ADDR_CORRECTION_PH0 [29:20]
#define STK_COEF_T_POS_PH1 (pdata?pdata->coef_t[2]:132) // follow by STK_ADDR_CORRECTION_PH1 [17:08]
#define STK_COEF_T_NEG_PH1 (pdata?pdata->coef_t[3]:131) // follow by STK_ADDR_CORRECTION_PH1 [29:20]
#define STK_COEF_T_POS_PH2 (pdata?pdata->coef_t[4]:133) // follow by STK_ADDR_CORRECTION_PH2 [17:08]
#define STK_COEF_T_NEG_PH2 (pdata?pdata->coef_t[5]:130) // follow by STK_ADDR_CORRECTION_PH2 [29:20]
#define STK_COEF_T_POS_PH3 (pdata?pdata->coef_t[6]:128) // follow by STK_ADDR_CORRECTION_PH3 [17:08]
#define STK_COEF_T_NEG_PH3 (pdata?pdata->coef_t[7]:128) // follow by STK_ADDR_CORRECTION_PH3 [29:20]
#define STK_COEF_T_POS_PH4 (pdata?pdata->coef_t[8]:188) // follow by STK_ADDR_CORRECTION_PH4 [17:08]
#define STK_COEF_T_NEG_PH4 (pdata?pdata->coef_t[9]:205) // follow by STK_ADDR_CORRECTION_PH4 [29:20]
#define STK_COEF_T_POS_PH5 (pdata?pdata->coef_t[10]:136) // follow by STK_ADDR_CORRECTION_PH5 [17:08]
#define STK_COEF_T_NEG_PH5 (pdata?pdata->coef_t[11]:156) // follow by STK_ADDR_CORRECTION_PH5 [29:20]
#define STK_COEF_T_POS_PH6 (pdata?pdata->coef_t[12]:201) // follow by STK_ADDR_CORRECTION_PH6 [17:08]
#define STK_COEF_T_NEG_PH6 (pdata?pdata->coef_t[13]:228) // follow by STK_ADDR_CORRECTION_PH6 [29:20]
#define STK_COEF_T_POS_PH7 (pdata?pdata->coef_t[14]:128) // follow by STK_ADDR_CORRECTION_PH7 [17:08]
#define STK_COEF_T_NEG_PH7 (pdata?pdata->coef_t[15]:128) // follow by STK_ADDR_CORRECTION_PH7 [29:20]

#define STK501XX_ID                         0x6503

#define STK_POLLING_TIME                    100000//us

#define IDLE_WAITING_6MS                    (600000/STK_POLLING_TIME) // default waintg 6ms.


#ifdef MCU_GESTURE
    #define PHASE_EN 0x3F  //use all phase
#else
    #define PHASE_EN 0x7F  //use phase 0 1 2 3 4 5 6
#endif

#ifdef TEMP_COMPENSATION
    /* Modify by design*/
    #define DELTA_TEMP_THD_A                       (pdata?pdata->tc_config[0]:500000)
    #define DELTA_A_MAPPING_PHASE                  (pdata?pdata->tc_config[1]:0)  // ref channel, 0~7 mapping phase 0~7, this for phase 1
    #define DELTA_A_MEASURE_PHASE                  (pdata?pdata->tc_config[2]:1)  // measure phase
/*End of modify by design*/
    #define DELTA_A_MAPPING_PHASE_REG              (STK_ADDR_REG_RAW_PH0_REG  + (DELTA_A_MAPPING_PHASE * 4))
    #define STK_ADDR_DELTADES_A_CTRL_VALUE          0x01403301 | (DELTA_A_MEASURE_PHASE <<4)

/* Modify by design*/
    #define DELTA_TEMP_THD_B                       (pdata?pdata->tc_config[3]:64000)
    #define DELTA_B_MAPPING_PHASE                  (pdata?pdata->tc_config[4]:3)  // ref channel, 0~7 mapping phase 0~7, this for phase 2
    #define DELTA_B_MEASURE_PHASE                  (pdata?pdata->tc_config[5]:4) //  measure phase
/*End of modify by design*/
    #define DELTA_B_MAPPING_PHASE_REG              (STK_ADDR_REG_RAW_PH0_REG  + (DELTA_B_MAPPING_PHASE * 4))
    #define STK_ADDR_DELTADES_B_CTRL_VALUE          0x01403301 | (DELTA_B_MEASURE_PHASE <<4)
    #define BASE_REINIT_DELTA_DES                  0x200000

/* Modify by design*/
    #define DELTA_TEMP_THD_C                       (pdata?pdata->tc_config[6]:80000)
    #define DELTA_C_MAPPING_PHASE                  (pdata?pdata->tc_config[7]:4) // ref channel, 0~7 mapping phase 0~7, this for phase 5
    #define DELTA_C_MEASURE_PHASE                  (pdata?pdata->tc_config[8]:5) //  measure phase
/*End of modify by design*/
    #define DELTA_C_MAPPING_PHASE_REG              (STK_ADDR_REG_RAW_PH0_REG  + (DELTA_C_MAPPING_PHASE * 4))
    #define STK_ADDR_DELTADES_C_CTRL_VALUE          0x01403301 | (DELTA_C_MEASURE_PHASE <<4)
    #define BASE_REINIT_DELTA_DES                  0x200000

#endif

typedef struct stk501xx_register_table
{
    uint16_t address;
    uint32_t value;
} stk501xx_register_table;

typedef enum
{
    STK_SAR_FAR_AWAY = 0,
    STK_SAR_NEAR_BY,
    STK_SAR_NEAR_BY_UNKNOWN
} stk_sar_nearby_type;

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
//#define STK_RXIO0_MUX_REG_VALUE                 0x22242222 //PH4 ANT4

#define STK_ADDR_RXIO1_MUX_REG                  0x005C
//#define STK_RXIO1_MUX_REG_VALUE                 0x22222222 //PH2 ANT1

#define STK_ADDR_RXIO2_MUX_REG                  0x0060
//#define STK_RXIO2_MUX_REG_VALUE                 0x22422222 //PH5 Ant5&6&6&7&11 ref

#define STK_ADDR_RXIO3_MUX_REG                  0x0064
//#define STK_RXIO3_MUX_REG_VALUE                 0x00000000 //Digital mode

#define STK_ADDR_RXIO4_MUX_REG                  0x0068
//#define STK_RXIO4_MUX_REG_VALUE                 0x22222224 //PH0 ANT0&1 ref

#define STK_ADDR_RXIO5_MUX_REG                  0x006C
//#define STK_RXIO5_MUX_REG_VALUE                 0x22224222 //PH3 ANT4 ref

#define STK_ADDR_RXIO6_MUX_REG                  0x0070
//#define STK_RXIO6_MUX_REG_VALUE                 0x22222242 //PH1 ANT0

#define STK_ADDR_RXIO7_MUX_REG                  0x0074
//#define STK_RXIO7_MUX_REG_VALUE                 0x24222222 //PH6 ANT5&6&7&11

#define STK_ADDR_ADP_BASELINE_0                 0x00B0
//#define STK_ADP_BASELINE_0_VALUE                0x00050000

#define STK_ADDR_ADP_BASELINE_1                 0x00B4
//#define STK_ADP_BASELINE_1_VALUE                0x10101010

#define STK_ADDR_ADP_BASELINE_2                 0x00B8
//#define STK_ADP_BASELINE_2_VALUE                0x10101010

#define STK_ADDR_IRQ_CONFIG                     0x00C4
#define STK_IRQ_CONFIG_SENS_RATE_OPT_SHIFT      24

#define STK_ADDR_SCAN_PERIOD                    0x00D0
#define STK_SCAN_PERIOD_VALUE                   0x00000960 // 0821 modify 120ms

#define STK_ADDR_WATCH_DOG                      0x00D4
#define STK_SNS_WATCH_DOG_MASK                  0x000000FF
//#define STK_SNS_WATCH_DOG_VALUE                 0x000000FF
#define STK_I2C_WDT_MASK                        0x0000FF00
//#define STK_I2C_WDT_VALUE                       0x00004000

#define STK_ADDR_TRIGGER_CMD                    0x0100
#define STK_TRIGGER_CMD_REG_INIT_ALL            0x0000000F
#define STK_TRIGGER_CMD_REG_BY_PHRST            0x0000000A

#define STK_ADDR_TRIGGER_REG                    0x0104
#define STK_TRIGGER_REG_PHEN_SHIFT              0
#define STK_TRIGGER_REG_PHEN_MASK               0xFF
#define STK_TRIGGER_REG_PHEN_DISABLE_ALL        0x00000000

#define STK_TRIGGER_REG_INIT_ALL                ((PHASE_EN << 8) | PHASE_EN)
#define STK_TRIGGER_REG_PHRST_PHASE             (1 << (8 + DELDEA_A_MAPPING_PHASE))

#define STK_TRIGGER_REG_ENTER_PAUSE_MODE        0x0
#define STK_TRIGGER_REG_EXIT_PAUSE_MODE         STK_TRIGGER_REG_INIT_ALL

#define STK_ADDR_RX_NL_CTRL                     0x0134

#define STK_ADDR_FAIL_STAT_DET_2                0x0148
//#define STK_FAIL_STAT_DET_2_VALUE               0x08003F00

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
/*
#define STK_SCAN_OPT_PH0_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH0_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH0_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH0_VALUE                 0x20004003
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
*/
#define STK_TX_CTRL_PH0_VALUE                   0x05001210   // 0821 modify
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
/*
#define STK_SCAN_OPT_PH1_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH1_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH1_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH1_VALUE                 0x20004003
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
*/
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
/*
#define STK_SCAN_OPT_PH2_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH2_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH2_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH2_VALUE                 0x20004003
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
*/
#define STK_TX_CTRL_PH2_VALUE           0x07001204 // 0821 modify
#define STK_SENS_CTRL_PH2_VALUE         0x00005003
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
/*
#define STK_SCAN_OPT_PH3_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH3_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH3_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH3_VALUE                 0x20004003
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
*/
#define STK_TX_CTRL_PH3_VALUE 0x05001204 // 0821 modify
#define STK_SENS_CTRL_PH3_VALUE 0x00005003 // 0821 modify
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
/*
#define STK_SCAN_OPT_PH4_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH4_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH4_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH4_VALUE                 0x20004003
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
*/
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
/*
#define STK_SCAN_OPT_PH5_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH5_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH5_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH5_VALUE                 0x20004003
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
*/
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
/*
#define STK_SCAN_OPT_PH6_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH6_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH6_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH6_VALUE                 0x20004003
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
*/
#define STK_TX_CTRL_PH6_VALUE 0x07001204 // 0821 modify
#define STK_SENS_CTRL_PH6_VALUE 0x00005003  // 0821 modify
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
/*
#define STK_SCAN_OPT_PH7_VALUE                  0x00000000
#ifdef MCU_GESTURE
    #define STK_TX_CTRL_PH7_VALUE                   0x05000404
#else
    #define STK_TX_CTRL_PH7_VALUE                   0x07001212 //41kHz, F_res=512
#endif
#define STK_SENS_CTRL_PH7_VALUE                 0x20004003
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
*/
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


/*****************************************************************************
 * stk501xx register, end
 *****************************************************************************/
#define STK501XX_NAME    "stk501xx"
static const uint16_t STK_ID[1] = { STK501XX_ID };

typedef struct stk_data stk_data;
typedef void (*STK_REPORT_CB)(struct stk_data *);

struct smtc_reg_data
{
    uint32_t reg;
    uint32_t val;
};

struct stk501xx_platform_data
{
    unsigned char   direction;
    int32_t             interrupt_int1_pin;
    int i2c_reg_num;
    struct smtc_reg_data *pi2c_reg;
    u32 phase_en;
    u32 mapping_phase[8];
    u32 sar_thd0[8];
    u32 sar_thd1[8];
    u32 coef_t[16];
    u32 tc_config[9];
};

struct stk_data
{
    const struct stk_bus_ops        *bops;
    const struct stk_timer_ops      *tops;
    const struct stk_gpio_ops       *gops;
    STK_REPORT_CB                   sar_report_cb;
    int32_t                         bus_idx;
    bool                            enabled;            /* chip is enabled or not */
    bool                            last_enable;        /* record current power status. For Suspend/Resume used. */
    uint8_t                         power_mode;
    uint8_t                         recv;
    stk_sar_nearby_type             last_nearby[8];
    stk_sar_nearby_type             last_nearby_dist1[8];
    uint16_t                        dist_phase;
#ifdef MCU_GESTURE
    GestureType                     gesture_state;
    bool                            gs_timer_is_running;
    uint8_t                         gs_idle_count;
#endif
    uint8_t                         state_change[8];
    uint8_t                         state_change_dist1[8];
    int32_t                         last_data[8];
    uint32_t                        chip_id;
    uint32_t                        chip_index;
    bool                            wait_phase_reset;
    stk_timer_info                  phase_reset_timer_info;
    struct stk501xx_platform_data   *pdata;
#ifdef TEMP_COMPENSATION
    bool                            last_prox_a_state;
    bool                            last_prox_b_state;
    bool                            last_prox_c_state;
    int32_t                         prev_temperature_ref_a;
    int32_t                         next_temperature_ref_a;
    int32_t                         prev_temperature_ref_b;
    int32_t                         next_temperature_ref_b;
    int                             prev_temperature_ref_c;
    int                             next_temperature_ref_c;
    bool                            reinit[8];
    uint8_t                         decent_cnt_a;
    uint8_t                         decent_cnt_b;
    uint8_t                         decent_cnt_c;
#endif
#ifdef STK_INTERRUPT_MODE
    stk_gpio_info                   gpio_info;
    int32_t                         int_pin;
#endif
    uint8_t                         intrrupt_init_state;
#if (defined STK_POLLING_MODE || defined MCU_GESTURE)
    stk_timer_info                  stk_timer_info;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    bool                            first_init;
};


//#define STK_TAG                 "[stkSAR]"

#define STK501XX_SAR_REPORT(stk_data)                        if ((stk_data)->sar_report_cb)          ((stk_data)->sar_report_cb(stk_data))

void stk501xx_set_enable(struct stk_data* stk, char enable, bool pause_mode);
int32_t stk_read_prox_flag(struct stk_data* stk, uint32_t* prox_flag);
void stk501xx_read_temp_data(struct stk_data* stk, uint16_t reg, int32_t *temperature);
void stk501xx_read_sar_data(struct stk_data* stk, uint32_t prox_flag, uint16_t dist_phase);
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
/* Algorithm loading in the last*/
#ifdef MCU_GESTURE
#include "stk_gesture_lib.h"
#endif
#endif /* __STK501XX_H__ */
