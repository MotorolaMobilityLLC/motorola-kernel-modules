
#ifndef __SC8549_HEADER__
#define __SC8549_HEADER__

/* Register 00h */
#define SC8549_REG_00                       0x00
#define	SC8549_BAT_OVP_DIS_MASK             0x80
#define	SC8549_BAT_OVP_DIS_SHIFT            7
#define	SC8549_BAT_OVP_ENABLE               0
#define	SC8549_BAT_OVP_DISABLE              1

#define SC8549_BAT_OVP_MASK                 0x3F
#define SC8549_BAT_OVP_SHIFT                0
#define SC8549_BAT_OVP_BASE                 3500
#define SC8549_BAT_OVP_LSB                  25

/* Register 01h */
#define SC8549_REG_01                       0x01
#define	SC8549_BAT_OCP_DIS_MASK             0x80
#define	SC8549_BAT_OCP_DIS_SHIFT            7
#define SC8549_BAT_OCP_ENABLE               0
#define SC8549_BAT_OCP_DISABLE              1

#define SC8549_BAT_OCP_MASK                 0x3F
#define SC8549_BAT_OCP_SHIFT                0
#define SC8549_BAT_OCP_BASE                 2000
#define SC8549_BAT_OCP_LSB                  100

/* Register 02h */
#define SC8549_REG_02                       0x02
#define SC8549_AC_OVP_STAT_MASK             0x20
#define SC8549_AC_OVP_STAT_SHIFT            5

#define SC8549_AC_OVP_FLAG_MASK             0x10
#define SC8549_AC_OVP_FLAG_SHIFT            4

#define SC8549_AC_OVP_MASK_MASK             0x08
#define SC8549_AC_OVP_MASK_SHIFT            3

#define SC8549_AC_OVP_MASK                  0x07
#define SC8549_AC_OVP_SHIFT                 0
#define SC8549_AC_OVP_BASE                  11
#define SC8549_AC_OVP_LSB                   1
#define SC8549_AC_OVP_6P5V                  65

/* Register 03h */
#define SC8549_REG_03                       0x03
#define SC8549_VDROP_OVP_DIS_MASK           0x80
#define SC8549_VDROP_OVP_DIS_SHIFT          7
#define SC8549_VDROP_OVP_ENABLE             0
#define SC8549_VDROP_OVP_DISABLE            1

#define SC8549_VDROP_OVP_STAT_MASK          0x10
#define SC8549_VDROP_OVP_STAT_SHIFT         4

#define SC8549_VDROP_OVP_FLAG_MASK          0x08
#define SC8549_VDROP_OVP_FLAG_SHIFT         3

#define SC8549_VDROP_OVP_MASK_MASK          0x04
#define SC8549_VDROP_OVP_MASK_SHIFT         2

#define SC8549_VDROP_THRESHOLD_SET_MASK	    0x02
#define SC8549_VDROP_THRESHOLD_SET_SHIFT    1
#define SC8549_VDROP_THRESHOLD_300MV	    0
#define SC8549_VDROP_THRESHOLD_400MV	    1

#define SC8549_VDROP_DEGLITCH_SET_MASK	    0x01
#define SC8549_VDROP_DEGLITCH_SET_SHIFT	    0
#define SC8549_VDROP_DEGLITCH_8US		    1
#define SC8549_VDROP_DEGLITCH_5MS		    0

/* Register 04h */
#define SC8549_REG_04                       0x04
#define SC8549_VBUS_OVP_DIS_MASK            0x80
#define SC8549_VBUS_OVP_DIS_SHIFT           7
#define SC8549_VBUS_OVP_ENABLE              0
#define SC8549_VBUS_OVP_DISABLE             1

#define SC8549_BUS_OVP_MASK                 0x7F
#define SC8549_BUS_OVP_SHIFT                0
#define SC8549_BUS_OVP_BASE                 6000
#define SC8549_BUS_OVP_LSB                  50

/* Register 05h */
#define SC8549_REG_05                       0x05
#define SC8549_BUS_UCP_DIS_MASK             0x80
#define SC8549_BUS_UCP_DIS_SHIFT            7
#define	SC8549_BUS_UCP_ENABLE               0
#define	SC8549_BUS_UCP_DISABLE              1

#define SC8549_BUS_OCP_DIS_MASK             0x40
#define SC8549_BUS_OCP_DIS_SHIFT            6
#define	SC8549_BUS_OCP_ENABLE               0
#define	SC8549_BUS_OCP_DISABLE              1

#define SC8549_BUS_UCP_FALL_DEGLITCH_MASK   0x20
#define SC8549_BUS_UCP_FALL_DEGLITCH_SHIFT  5
#define SC8549_BUS_UCP_FALL_DEGLITCH_10US   0
#define SC8549_BUS_UCP_FALL_DEGLITCH_5MS    1

#define SC8549_BUS_OCP_MASK                 0x0F
#define SC8549_BUS_OCP_SHIFT                0
#define SC8549_BUS_OCP_BASE                 1200
#define SC8549_BUS_OCP_LSB                  300

/* Register 06h */
#define SC8549_REG_06                       0x06
#define SC8549_TSHUT_FLAG_MASK              0x80
#define SC8549_TSHUT_FLAG_SHIFT             7

#define SC8549_TSHUT_STAT_MASK              0x40
#define SC8549_TSHUT_STAT_SHIFT             6

#define SC8549_VBUS_ERRORLO_STAT_MASK       0x20
#define SC8549_VBUS_ERRORLO_STAT_SHIFT      5

#define SC8549_VBUS_ERRORHI_STAT_MASK       0x10
#define SC8549_VBUS_ERRORHI_STAT_SHIFT      4

#define SC8549_SS_TIMEOUT_FLAG_MASK         0x08
#define SC8549_SS_TIMEOUT_FLAG_SHIFT        3

#define SC8549_CONV_SWITCHING_STAT_MASK     0x04
#define SC8549_CONV_SWITCHING_STAT_SHIFT    2

#define SC8549_REG_TIMEOUT_FLAG_MASK        0x02
#define SC8549_REG_TIMEOUT_FLAG_SHIFT       1

#define SC8549_PIN_DIAG_FALL_FLAG_MASK      0x01
#define SC8549_PIN_DIAG_FALL_FLAG_SHIFT     0

/* Register 07h */
#define SC8549_REG_07                       0x07
#define SC8549_CHG_EN_MASK                  0x80
#define SC8549_CHG_EN_SHIFT                 7
#define SC8549_CHG_ENABLE                   1
#define SC8549_CHG_DISABLE                  0

#define SC8549_REG_RST_MASK                 0x40
#define SC8549_REG_RST_SHIFT                6
#define SC8549_REG_RST_ENABLE               1
#define SC8549_REG_RST_DISABLE              0

#define SC8549_FREQ_SHIFT_MASK              0x18
#define SC8549_FREQ_SHIFT_SHIFT             3
#define SC8549_FREQ_SHIFT_NORMINAL          0
#define SC8549_FREQ_SHIFT_POSITIVE10        1
#define SC8549_FREQ_SHIFT_NEGATIVE10        2
#define SC8549_FREQ_SHIFT_SPREAD_SPECTRUM   3

#define SC8549_FSW_SET_MASK					0x07
#define SC8549_FSW_SET_SHIFT				0
#define SC8549_FSW_SET_300KHZ				0
#define SC8549_FSW_SET_350KHZ				1
#define SC8549_FSW_SET_400KHZ				2
#define SC8549_FSW_SET_450KHZ				3
#define SC8549_FSW_SET_500KHZ				4
#define SC8549_FSW_SET_550KHZ				5
#define SC8549_FSW_SET_600KHZ				6
#define SC8549_FSW_SET_750KHZ				7

/* Register 08h */
#define SC8549_REG_08                       0x08
#define SC8549_SS_TIMEOUT_SET_MASK          0xE0
#define SC8549_SS_TIMEOUT_SET_SHIFT         5
#define SC8549_SS_TIMEOUT_DISABLE           0
#define SC8549_SS_TIMEOUT_40MS              1
#define SC8549_SS_TIMEOUT_80MS              2
#define SC8549_SS_TIMEOUT_320MS             3
#define SC8549_SS_TIMEOUT_1280MS            4
#define SC8549_SS_TIMEOUT_5120MS            5
#define SC8549_SS_TIMEOUT_20480MS           6
#define SC8549_SS_TIMEOUT_81920MS           7

#define SC8549_REG_TIMEOUT_DIS_MASK         0x10
#define SC8549_REG_TIMEOUT_DIS_SHIFT        4
#define SC8549_REG_TIMEOUT_ENABLE           0
#define SC8549_REG_TIMEOUT_DISABLE          1

#define SC8549_VOUT_OVP_DIS_MASK            0x08
#define SC8549_VOUT_OVP_DIS_SHIFT           3
#define SC8549_VOUT_OVP_ENABLE              0
#define SC8549_VOUT_OVP_DISABLE             1

#define SC8549_SET_IBAT_SNS_RES_MASK        0x04
#define SC8549_SET_IBAT_SNS_RES_SHIFT       2
#define SC8549_SET_IBAT_SNS_RES_5MHM        0
#define SC8549_SET_IBAT_SNS_RES_2MHM        1

#define SC8549_VBUS_PD_EN_MASK              0x02
#define SC8549_VBUS_PD_EN_SHIFT             1
#define SC8549_VBUS_PD_ENABLE               1
#define SC8549_VBUS_PD_DISABLE              0

#define SC8549_VAC_PD_EN_MASK               0x01
#define SC8549_VAC_PD_EN_SHIFT              0
#define SC8549_VAC_PD_ENABLE                1
#define SC8549_VAC_PD_DISABLE               0

/* Register 09h */
#define SC8549_REG_09                       0x09
#define SC8549_CHARGE_MODE_MASK             0x80
#define SC8549_CHARGE_MODE_SHIFT            7
#define SC8549_CHARGE_MODE_2_1              0
#define SC8549_CHARGE_MODE_1_1              1

#define SC8549_PRO_FLAG_MASK                0x40
#define SC8549_PRO_FLAG_SHIFT               6
#define SC8549_PRO_FLAG_HAPPENED            1
#define SC8549_PRO_FLAG_NOT_HAPPENED        0

#define SC8549_IBUS_UCP_RISE_FLAG_MASK      0x20
#define SC8549_IBUS_UCP_RISE_FLAG_SHIFT     5

#define SC8549_IBUS_UCP_RISE_MASK_MASK      0x10
#define SC8549_IBUS_UCP_RISE_MASK_SHIFT     4
#define SC8549_IBUS_UCP_RISE_MASK_ENABLE    1
#define SC8549_IBUS_UCP_RISE_MASK_DISABLE   0

#define SC8549_WD_TIMEOUT_FLAG_MASK         0x08
#define SC8549_WD_TIMEOUT_FLAG_SHIFT        3

#define SC8549_WATCHDOG_TIMEOUT_MASK        0x07
#define SC8549_WATCHDOG_TIMEOUT_SHIFT       0
#define SC8549_WATCHDOG_DISABLE             0
#define SC8549_WATCHDOG_0P2S                1
#define SC8549_WATCHDOG_0P5S                2
#define SC8549_WATCHDOG_1S                  3
#define SC8549_WATCHDOG_5S                  4
#define SC8549_WATCHDOG_30S                 5

/* Register 0Ah */
#define SC8549_REG_0A                       0x0A
#define SC8549_VBAT_REG_EN_MASK             0x80
#define SC8549_VBAT_REG_EN_SHIFT            7
#define SC8549_VBAT_REG_ENABLE              1
#define SC8549_VBAT_REG_DISABLE             0

#define SC8549_VBAT_REG_ACTIVE_STAT_MASK    0x10
#define SC8549_VBAT_REG_ACTIVE_STAT_SHIFT   4

#define SC8549_VBAT_REG_ACTIVE_FLAG_MASK    0x08
#define SC8549_VBAT_REG_ACTIVE_FLAG_SHIFT   3

#define SC8549_VBAT_REG_ACTIVE_MASK_MASK    0x04
#define SC8549_VBAT_REG_ACTIVE_MASK_SHIFT   2
#define SC8549_VBAT_REG_ACTIVE_NOT_MASK     0
#define SC8549_VBAT_REG_ACTIVE_MASK         1

#define SC8549_VBAT_REG_MASK                0x03
#define SC8549_VBAT_REG_SHIFT               0
#define SC8549_VBAT_REG_50MV                0
#define SC8549_VBAT_REG_100MV               1
#define SC8549_VBAT_REG_150MV               2
#define SC8549_VBAT_REG_200MV               3

/* Register 0Bh */
#define SC8549_REG_0B                       0x0B
#define SC8549_IBAT_REG_EN_MASK             0x80
#define SC8549_IBAT_REG_EN_SHIFT            7
#define SC8549_IBAT_REG_ENABLE              1
#define SC8549_IBAT_REG_DISABLE             0

#define SC8549_IBAT_REG_ACTIVE_STAT_MASK    0x10
#define SC8549_IBAT_REG_ACTIVE_STAT_SHIFT   4

#define SC8549_IBAT_REG_ACTIVE_FLAG_MASK    0x08
#define SC8549_IBAT_REG_ACTIVE_FLAG_SHIFT   3

#define SC8549_IBAT_REG_ACTIVE_MASK_MASK    0x04
#define SC8549_IBAT_REG_ACTIVE_MASK_SHIFT   2
#define SC8549_IBAT_REG_ACTIVE_NOT_MASK     0
#define SC8549_IBAT_REG_ACTIVE_MASK         1

#define SC8549_IBAT_REG_MASK                0x03
#define SC8549_IBAT_REG_SHIFT               0
#define SC8549_IBAT_REG_200MA               0
#define SC8549_IBAT_REG_300MA               1
#define SC8549_IBAT_REG_400MA               2
#define SC8549_IBAT_REG_500MA               3

/* Register 0Ch */
#define SC8549_REG_0C                       0x0C
#define SC8549_IBUS_REG_EN_MASK             0x80
#define SC8549_IBUS_REG_EN_SHIFT            7
#define SC8549_IBUS_REG_ENABLE              1
#define SC8549_IBUS_REG_DISABLE             0

#define SC8549_IBUS_REG_ACTIVE_STAT_MASK    0x40
#define SC8549_IBUS_REG_ACTIVE_STAT_SHIFT   6

#define SC8549_IBUS_REG_ACTIVE_FLAG_MASK    0x20
#define SC8549_IBUS_REG_ACTIVE_FLAG_SHIFT   5

#define SC8549_IBUS_REG_ACTIVE_MASK_MASK    0x10
#define SC8549_IBUS_REG_ACTIVE_MASK_SHIFT   4
#define SC8549_IBUS_REG_ACTIVE_NOT_MASK     0
#define SC8549_IBUS_REG_ACTIVE_MASK         1

#define SC8549_IBUS_REG_MASK                0x0F
#define SC8549_IBUS_REG_SHIFT               0
#define SC8549_IBUS_REG_BASE                1200
#define SC8549_IBUS_REG_LSB                 300

/* Register 0Dh */
#define SC8549_REG_0D                       0x0D
#define SC8549_PMID2OUT_UVP_MASK            0xC0
#define SC8549_PMID2OUT_UVP_SHIFT           6
#define SC8549_PMID2OUT_UVP_50MV            0
#define SC8549_PMID2OUT_UVP_100MV           1
#define SC8549_PMID2OUT_UVP_150MV           2
#define SC8549_PMID2OUT_UVP_200MV           3

#define SC8549_PMID2OUT_OVP_MASK            0x30
#define SC8549_PMID2OUT_OVP_SHIFT           4
#define SC8549_PMID2OUT_OVP_200MV           0
#define SC8549_PMID2OUT_OVP_300MV           1
#define SC8549_PMID2OUT_OVP_400MV           2
#define SC8549_PMID2OUT_OVP_500MV           3

#define SC8549_PMID2OUT_UVP_FLAG_MASK       0x08
#define SC8549_PMID2OUT_UVP_FLAG_SHIFT      3

#define SC8549_PMID2OUT_OVP_FLAG_MASK       0x04
#define SC8549_PMID2OUT_OVP_FLAG_SHIFT      2

#define SC8549_PMID2OUT_UVP_STAT_MASK       0x02
#define SC8549_PMID2OUT_UVP_STAT_SHIFT      1

#define SC8549_PMID2OUT_OVP_STAT_MASK       0x01
#define SC8549_PMID2OUT_OVP_STAT_SHIFT      0

/* Register 0Eh */
#define SC8549_REG_0E                       0x0E
#define SC8549_VOUT_OVP_STAT_MASK           0x80
#define SC8549_VOUT_OVP_STAT_SHIFT          7

#define SC8549_VBAT_OVP_STAT_MASK           0x40
#define SC8549_VBAT_OVP_STAT_SHIFT          6

#define SC8549_IBAT_OCP_STAT_MASK           0x20
#define SC8549_IBAT_OCP_STAT_SHIFT          5

#define SC8549_VBUS_OVP_STAT_MASK           0x10
#define SC8549_VBUS_OVP_STAT_SHIFT          4

#define SC8549_IBUS_OCP_STAT_MASK           0x08
#define SC8549_IBUS_OCP_STAT_SHIFT          3

#define SC8549_IBUS_UCP_FALL_STAT_MASK      0x04
#define SC8549_IBUS_UCP_FALL_STAT_SHIFT     2

#define SC8549_ADAPTER_INSERT_STAT_MASK     0x02
#define SC8549_ADAPTER_INSERT_STAT_SHIFT    1

#define SC8549_VBAT_INSERT_STAT_MASK        0x01
#define SC8549_VBAT_INSERT_STAT_SHIFT       0

/* Register 0Fh */
#define SC8549_REG_0F                       0x0F
#define SC8549_VOUT_OVP_FLAG_MASK           0x80
#define SC8549_VOUT_OVP_FLAG_SHIFT          7

#define SC8549_VBAT_OVP_FLAG_MASK           0x40
#define SC8549_VBAT_OVP_FLAG_SHIFT          6

#define SC8549_IBAT_OCP_FLAG_MASK           0x20
#define SC8549_IBAT_OCP_FLAG_SHIFT          5

#define SC8549_VBUS_OVP_FLAG_MASK           0x10
#define SC8549_VBUS_OVP_FLAG_SHIFT          4

#define SC8549_IBUS_OCP_FLAG_MASK           0x08
#define SC8549_IBUS_OCP_FLAG_SHIFT          3

#define SC8549_IBUS_UCP_FALL_FLAG_MASK      0x04
#define SC8549_IBUS_UCP_FALL_FLAG_SHIFT     2

#define SC8549_ADAPTER_INSERT_FLAG_MASK     0x02
#define SC8549_ADAPTER_INSERT_FLAG_SHIFT    1

#define SC8549_VBAT_INSERT_FLAG_MASK        0x01
#define SC8549_VBAT_INSERT_FLAG_SHIFT       0

/* Register 10h */
#define SC8549_REG_10                       0x10
#define SC8549_VOUT_OVP_MASK_MASK           0x80
#define SC8549_VOUT_OVP_MASK_SHIFT          7

#define SC8549_VBAT_OVP_MASK_MASK           0x40
#define SC8549_VBAT_OVP_MASK_SHIFT          6

#define SC8549_IBAT_OCP_MASK_MASK           0x20
#define SC8549_IBAT_OCP_MASK_SHIFT          5

#define SC8549_VBUS_OVP_MASK_MASK           0x10
#define SC8549_VBUS_OVP_MASK_SHIFT          4

#define SC8549_IBUS_OCP_MASK_MASK           0x08
#define SC8549_IBUS_OCP_MASK_SHIFT          3

#define SC8549_IBUS_UCP_FALL_MASK_MASK      0x04
#define SC8549_IBUS_UCP_FALL_MASK_SHIFT     2

#define SC8549_ADAPTER_INSERT_MASK_MASK     0x02
#define SC8549_ADAPTER_INSERT_MASK_SHIFT    1

#define SC8549_VBAT_INSERT_MASK_MASK        0x01
#define SC8549_VBAT_INSERT_MASK_SHIFT       0

/* Register 11h */
#define SC8549_REG_11                       0x11
#define SC8549_ADC_EN_MASK                  0x80
#define SC8549_ADC_EN_SHIFT                 7
#define SC8549_ADC_ENABLE                   1
#define SC8549_ADC_DISABLE                  0

#define SC8549_ADC_RATE_MASK                0x40
#define SC8549_ADC_RATE_SHIFT               6
#define SC8549_ADC_RATE_CONTINOUS           0
#define SC8549_ADC_RATE_ONESHOT             1

#define SC8549_ADC_DONE_STAT_MASK           0x04
#define SC8549_ADC_DONE_STAT_SHIFT          2

#define SC8549_ADC_DONE_FLAG_MASK           0x02
#define SC8549_ADC_DONE_FLAG_SHIFT          1

#define SC8549_ADC_DONE_MASK_MASK           0x01
#define SC8549_ADC_DONE_MASK_SHIFT          0

/* Register 12h */
#define SC8549_REG_12                       0x12
#define SC8549_VBUS_ADC_DIS_MASK            0x40
#define SC8549_VBUS_ADC_DIS_SHIFT           6
#define SC8549_VBUS_ADC_ENABLE              0
#define SC8549_VBUS_ADC_DISABLE             1

#define SC8549_VAC_ADC_DIS_MASK             0x20
#define SC8549_VAC_ADC_DIS_SHIFT            5
#define SC8549_VAC_ADC_ENABLE               0
#define SC8549_VAC_ADC_DISABLE              1

#define SC8549_VOUT_ADC_DIS_MASK            0x10
#define SC8549_VOUT_ADC_DIS_SHIFT           4
#define SC8549_VOUT_ADC_ENABLE              0
#define SC8549_VOUT_ADC_DISABLE             1

#define SC8549_VBAT_ADC_DIS_MASK            0x08
#define SC8549_VBAT_ADC_DIS_SHIFT           3
#define SC8549_VBAT_ADC_ENABLE              0
#define SC8549_VBAT_ADC_DISABLE             1

#define SC8549_IBAT_ADC_DIS_MASK            0x04
#define SC8549_IBAT_ADC_DIS_SHIFT           2
#define SC8549_IBAT_ADC_ENABLE              0
#define SC8549_IBAT_ADC_DISABLE             1

#define SC8549_IBUS_ADC_DIS_MASK            0x02
#define SC8549_IBUS_ADC_DIS_SHIFT           1
#define SC8549_IBUS_ADC_ENABLE              0
#define SC8549_IBUS_ADC_DISABLE             1

#define SC8549_TDIE_ADC_DIS_MASK            0x01
#define SC8549_TDIE_ADC_DIS_SHIFT           0
#define SC8549_TDIE_ADC_ENABLE              0
#define SC8549_TDIE_ADC_DISABLE             1

/* Register 13h */
#define SC8549_REG_13                       0x13
#define SC8549_IBUS_POL_H_MASK              0x0F
#define SC8549_IBUS_ADC_LSB_MUL             1875
#define SC8549_IBUS_ADC_LSB_DIV             1000

/* Register 14h */
#define SC8549_REG_14                       0x14
#define SC8549_IBUS_POL_L_MASK              0xFF

/* Register 15h */
#define SC8549_REG_15                       0x15
#define SC8549_VBUS_POL_H_MASK              0x0F
#define SC8549_VBUS_ADC_LSB_MUL             375
#define SC8549_VBUS_ADC_LSB_DIV             100

/* Register 16h */
#define SC8549_REG_16                       0x16
#define SC8549_VBUS_POL_L_MASK              0xFF

/* Register 17h */
#define SC8549_REG_17                       0x17
#define SC8549_VAC_POL_H_MASK               0x0F
#define SC8549_VAC_ADC_LSB                  5

/* Register 18h */
#define SC8549_REG_18                       0x18
#define SC8549_VAC_POL_L_MASK               0xFF

/* Register 19h */
#define SC8549_REG_19                       0x19
#define SC8549_VOUT_POL_H_MASK              0x0F
#define SC8549_VOUT_ADC_LSB_MUL             125
#define SC8549_VOUT_ADC_LSB_DIV             100

/* Register 1Ah */
#define SC8549_REG_1A                       0x1A
#define SC8549_VOUT_POL_L_MASK              0x0F

/* Register 1Bh */
#define SC8549_REG_1B                       0x1B
#define SC8549_VBAT_POL_H_MASK              0x0F
#define SC8549_VBAT_ADC_LSB_MUL             125
#define SC8549_VBAT_ADC_LSB_DIV             100

/* Register 1Ch */
#define SC8549_REG_1C                       0x1C
#define SC8549_VBAT_POL_L_MASK              0xFF

/* Register 1Dh */
#define SC8549_REG_1D                       0x1D
#define SC8549_IBAT_POL_H_MASK              0x0F
#define SC8549_IBAT_ADC_LSB_MUL             3125
#define SC8549_IBAT_ADC_LSB_DIV             1000

/* Register 1Eh */
#define SC8549_REG_1E                       0x1E
#define SC8549_IBAT_POL_L_MASK              0xFF

/* Register 1Fh */
#define SC8549_REG_1F                       0x1F
#define SC8549_TDIE_POL_H_MASK              0x01
#define SC8549_TDIE_ADC_LSB_MUL             5
#define SC8549_TDIE_ADC_LSB_DIV             10

/* Register 20h */
#define SC8549_REG_20                       0x20
#define SC8549_TDIE_POL_L_MASK              0xFF

/* Register 21h */
#define SC8549_REG_21                       0x21
#define SC8549_DM_500K_PD_EN_MASK           0x80
#define SC8549_DM_500K_PD_EN_SHIFT          7
#define SC8549_DM_500K_PD_ENABLE            1
#define SC8549_DM_500K_PD_DISABLE           0

#define SC8549_DP_500K_PD_EN_MASK           0x40
#define SC8549_DP_500K_PD_EN_SHIFT          6
#define SC8549_DP_500K_PD_ENABLE            1
#define SC8549_DP_500K_PD_DISABLE           0

#define SC8549_DM_20K_PD_EN_MASK            0x20
#define SC8549_DM_20K_PD_EN_SHIFT           5
#define SC8549_DM_20K_PD_ENABLE             1
#define SC8549_DM_20K_PD_DISABLE            0

#define SC8549_DP_20K_PD_EN_MASK            0x10
#define SC8549_DP_20K_PD_EN_SHIFT           4
#define SC8549_DP_20K_PD_ENABLE             1
#define SC8549_DP_20K_PD_DISABLE            0

#define SC8549_DM_SINK_EN_MASK              0x08
#define SC8549_DM_SINK_EN_SHIFT             3
#define SC8549_DM_SINK_ENABLE               1
#define SC8549_DM_SINK_DISABLE              0

#define SC8549_DP_SINK_EN_MASK              0x04
#define SC8549_DP_SINK_EN_SHIFT             2
#define SC8549_DP_SINK_ENABLE               1
#define SC8549_DP_SINK_DISABLE              0

#define SC8549_DP_SRC_10UA_MASK             0x02
#define SC8549_DP_SRC_10UA_SHIFT            1
#define SC8549_DP_SRC_10UA                  1
#define SC8549_DP_SRC_MORE_THAN_250UA       0

#define SC8549_DPDM_EN_MASK                 0x01
#define SC8549_DPDM_EN_SHIFT                0
#define SC8549_DPDM_ENABLE                  1
#define SC8549_DPDM_DISABLE                 0

/* Register 22h */
#define SC8549_REG_22                       0x22
#define SC8549_DPDM_OVP_DIS_MASK            0x40
#define SC8549_DPDM_OVP_DIS_SHIFT           6
#define SC8549_DPDM_OVP_ENABLE              0
#define SC8549_DPDM_OVP_DISABLE             1

#define SC8549_DM_BUFF_MASK                 0x30
#define SC8549_DM_BUFF_SHIFT                4
#define SC8549_DM_BUFF_0P6V                 0
#define SC8549_DM_BUFF_2V                   1
#define SC8549_DM_BUFF_2P7V                 2
#define SC8549_DM_BUFF_3P3V                 3

#define SC8549_DP_BUFF_MASK                 0x0C
#define SC8549_DP_BUFF_SHIFT                2
#define SC8549_DP_BUFF_0P6V                 0
#define SC8549_DP_BUFF_2V                   1
#define SC8549_DP_BUFF_2P7V                 2
#define SC8549_DP_BUFF_3P3V                 3

#define SC8549_DM_BUFF_EN_MASK              0x02
#define SC8549_DM_BUFF_EN_SHIFT             1
#define SC8549_DM_BUFF_ENABLE               1
#define SC8549_DM_BUFF_DISABLE              0

#define SC8549_DP_BUFF_EN_MASK              0x01
#define SC8549_DP_BUFF_EN_SHIFT             0
#define SC8549_DP_BUFF_ENABLE               1
#define SC8549_DP_BUFF_DISABLE              0

/* Register 23h */
#define SC8549_REG_23                       0x23
#define SC8549_VDM_READ_MASK                0x38
#define SC8549_VDM_READ_SHIFT               3
#define SC8549_VDM_READ_0V_0P325V           0
#define SC8549_VDM_READ_0P325V_1V           1
#define SC8549_VDM_READ_1V_1P35V            2
#define SC8549_VDM_READ_1P35V_2P2V          3
#define SC8549_VDM_READ_2P2V_3V             4
#define SC8549_VDM_READ_3V_3P3V             5

#define SC8549_VDP_READ_MASK                0x07
#define SC8549_VDP_READ_SHIFT               0
#define SC8549_VDP_READ_0V_0P325V           0
#define SC8549_VDP_READ_0P325V_1V           1
#define SC8549_VDP_READ_1V_1P35V            2
#define SC8549_VDP_READ_1P35V_2P2V          3
#define SC8549_VDP_READ_2P2V_3V             4
#define SC8549_VDP_READ_3V_3P3V             5

/* Register 24h */
#define SC8549_REG_24                       0x24
#define SC8549_DM_LOW_MASK_MASK             0x40
#define SC8549_DM_LOW_MASK_SHIFT            6

#define SC8549_DM_LOW_FLAG_MASK             0x20
#define SC8549_DM_LOW_FLAG_SHIFT            5

#define SC8549_DP_LOW_MASK_MASK             0x10
#define SC8549_DP_LOW_MASK_SHIFT            4

#define SC8549_DP_LOW_FLAG_MASK             0x08
#define SC8549_DP_LOW_FLAG_SHIFT            3

#define SC8549_DPDM_OVP_MASK_MASK           0x04
#define SC8549_DPDM_OVP_MASK_SHIFT          2

#define SC8549_DPDM_OVP_FLAG_MASK           0x02
#define SC8549_DPDM_OVP_FLAG_SHIFT          1

#define SC8549_DPDM_OVP_STAT_MASK           0x01
#define SC8549_DPDM_OVP_STAT_SHIFT          0

/* Register 36h */
#define SC8549_REG_36                       0x36
#define SC8549_DEVICE_ID                    0x66


#endif
