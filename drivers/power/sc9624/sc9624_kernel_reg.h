// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
 */
#ifndef __SC9624_REG__
#define __SC9624_REG__
#include <linux/mmi_wireless_class.h>
/// -----------------------------------------------------------------
///                      Work Mode
/// -----------------------------------------------------------------
enum {
    RX_MODE = 0,
    TX_MODE,
};

/// -----------------------------------------------------------------
///                      AP Command
/// -----------------------------------------------------------------
typedef enum {
    RX_INT_CLR             = BIT(0),
    RX_CHIP_RESET          = BIT(1),
    RX_SEND_PPP            = BIT(2),
    RX_LVP_CHANGE          = BIT(3),
    RX_OVP_CHANGE          = BIT(4),
    RX_OCP_CHANGE          = BIT(5),
    RX_ILIMIT_CHANGE       = BIT(6),
    RX_OTP_CHANGE          = BIT(7),
    RX_VDD_DISABLE         = BIT(8),    //NOT USED
    RX_VDD_ENABLE          = BIT(9),    //NOT USED
    RX_HVWDGT_DISABLE      = BIT(10),
    RX_HVWDGT_FEED         = BIT(11),
    RX_SEND_EPT            = BIT(12),
    RX_SEND_RENEG          = BIT(13),
    RX_VOUT_CHANGE         = BIT(14),
    RX_VOUT_ENABLE         = BIT(15),
    RX_VOUT_DISABLE        = BIT(16),
    RX_RPP_8BIT            = BIT(17),
    RX_RPP_24BIT           = BIT(18),
    //Reserved
    RX_LOWPOWER_MODE       = BIT(30),
    RX_REFRESH             = BIT(31),
} RX_CMD;

typedef enum {
    TX_INT_CLR                = BIT(0),
    TX_CHIP_RESET             = BIT(1),
    TX_SEND_PPP               = BIT(2),
    TX_LVP_CHANGE             = BIT(3),
    TX_OVP_CHANGE             = BIT(4),
    TX_OCP_CHANGE             = BIT(5),
    TX_ILIMIT_CHANGE          = BIT(6),
    TX_OTP_CHANGE             = BIT(7),
    TX_VDD_DISABLE            = BIT(8),    //NOT USED
    TX_VDD_ENABLE             = BIT(9),    //NOT USED
    TX_HVWDGT_DISABLE         = BIT(10),
    TX_HVWDGT_FEED            = BIT(11),
    //Reserved1
    TX_DISABLE                = BIT(18),
    TX_ENABLE                 = BIT(19),
    TX_BRG_AUTO               = BIT(20),
    TX_BRG_MANUAL             = BIT(21),
    TX_PWM_UPDATE             = BIT(22),
    TX_PWM_CONFIG             = BIT(23),
    TX_FOD_ENABLE             = BIT(24),
    TX_FOD_DISABLE            = BIT(25),
    TX_PING_OCP_CHANGE        = BIT(26),
    TX_PING_OVP_CHANGE        = BIT(27),
    TX_OPEN_LOOP              = BIT(28),
    //Reserved2
    TX_REFRESH                = BIT(31),
} TX_CMD;

/// -----------------------------------------------------------------
///                      sys mode
/// -----------------------------------------------------------------
typedef union{
    uint32_t value;
    struct {
        uint32_t STANDBY        : 1;
        uint32_t RECEIVER       : 1;
        uint32_t TRANSMITTER    : 1;
        uint32_t BPP_MODE       : 1;
        uint32_t EPP_MODE       : 1;
        uint32_t PREIVATE_MODE  : 1;
        uint32_t RPP_24BIT      : 1;
        uint32_t VDD_STATUS     : 1;
        uint32_t AC_MODE        : 1;
        uint32_t SLEEP_MODE     : 1;
        uint32_t HVWDGT_STATUS  : 1;
        uint32_t FOD_STATUS     : 1;
        uint32_t Reserved       : 20;
    };
} SYSMODE;

/// -----------------------------------------------------------------
///                      EPT RESON
/// -----------------------------------------------------------------
typedef enum{
    UNKOWN              = 0X00,
    CHARGE_COMPLETE     = 0X01,
    INTERNAL_FAULT      = 0X02,
    OVER_TEMPERATURE    = 0X03,
    OVER_VOLTAGE        = 0X04,
    OVER_CURRENT        = 0X05,
    BATTERY_FAILURE     = 0X06,
    RESERVED1           = 0X07,
    NO_RESPONSE         = 0X08,
    RESERVED2           = 0X09,
    NEGO_FAILURE        = 0X0A,
    RESTART_PT          = 0X0B,
}EPT_RESON;

/// -----------------------------------------------------------------
///                      INT FLAG
/// -----------------------------------------------------------------
typedef union {
    uint32_t value;
    struct {
        uint32_t OCP             : 1;
        uint32_t VOUT_OVP        : 1;
        uint32_t CLAMP_OVP       : 1;
        uint32_t NGAGE_OVP       : 1;
        uint32_t VOUT_LVP        : 1;
        uint32_t OTP             : 1;
        uint32_t OTP_160C        : 1;
        uint32_t SLEEP           : 1;
        uint32_t MODE_CHANGE     : 1;
        uint32_t FSK_RECV        : 1;
        uint32_t PPP_SUCCESS     : 1;
        uint32_t AFC_DET         : 1;
        uint32_t EPP_DET         : 1;
        uint32_t POWERON         : 1;
        uint32_t SS_PKT          : 1;
        uint32_t ID_PKT          : 1;
        uint32_t CONFIG_PKT      : 1;
        uint32_t RX_READY        : 1;
        uint32_t LDO_ON          : 1;
        uint32_t LDO_OFF         : 1;
        uint32_t PLDO            : 1;
        uint32_t SCP             : 1;
        uint32_t Reserved        : 10;
    };
} RX_INT;

typedef union {
    uint32_t value;
    struct {
        uint32_t OCP             : 1;
        uint32_t VOUT_OVP        : 1;
        uint32_t CLAMP_OVP       : 1;
        uint32_t NGAGE_OVP       : 1;
        uint32_t VIN_LVP         : 1;
        uint32_t OTP             : 1;
        uint32_t OTP_160C        : 1;
        uint32_t SLEEP           : 1;
        uint32_t MODE_CHANGE     : 1;
        uint32_t ASK_RECV        : 1;
        uint32_t PPP_TIMEOUT     : 1;
        uint32_t PPP_SUCCESS     : 1;
        uint32_t AFC_DET         : 1;
        uint32_t EPP_DET         : 1;
        uint32_t DETECT_RX       : 1;
        uint32_t REMOVE_POWER    : 1;
        uint32_t FOD             : 1;
        uint32_t DETECT_TX       : 1;
        uint32_t CEP_TIMEOUT     : 1;
        uint32_t PRR_TIMEOUT     : 1;
        uint32_t Reserved        : 12;
    };
} TX_INT;

/// -----------------------------------------------------------------
///                              WPC TYPE
///
/// -----------------------------------------------------------------
#define MAX_ASK_SIZE            32
#define MAX_FSK_SIZE            32

typedef struct {
    uint8_t header;

    uint8_t max_power             : 6;
    uint8_t power_class           : 2;

    uint8_t reserved0;

    uint8_t count                 : 3;
    uint8_t ZERO                  : 1;
    uint8_t reserved1             : 3;
    uint8_t prop                  : 1;

    uint8_t window_offset         : 3;
    uint8_t window_size           : 5;

    uint8_t reserved2             : 4;
    uint8_t depth                 : 2;
    uint8_t polarity              : 1;
    uint8_t neg                   : 1;
} CfgPktType;

typedef struct {
    uint8_t  header;

    uint8_t  minor_version        : 4;
    uint8_t  major_version        : 4;

    uint16_t manufacturer_code;

    uint32_t basic_device_identifier3 : 7;
    uint32_t ext                      : 1;
    uint32_t basic_device_identifier2 : 8;
    uint32_t basic_device_identifier1 : 8;
    uint32_t basic_device_identifier0 : 8;
} IdPktType;

typedef struct {
    uint8_t SS;
    uint8_t PCH;
    uint16_t RSV;
} SSPktType;

typedef struct {
    int8_t cep;
    uint8_t rsv;
    uint8_t rpp;
} PTpktType;

typedef struct {
    uint16_t mcode;
    uint8_t  minor           : 4;
    uint8_t  major           : 4;
    uint8_t  afc;
} TxInfoType;

typedef union {
    uint8_t value;
    struct {
        uint8_t depth             : 2;
        uint8_t polarity          : 1;
        uint8_t reserved          : 5;
    };
} FSKParametersType;

typedef union {              // power transfer contract
    uint32_t value;
    struct {
        uint8_t RPPTHeader;
        uint8_t guaranteed_power; // The value in this field is in units of 0.5 W.
        uint8_t max_power;
        FSKParametersType fsk;
    };
} ContractType;

typedef struct {
    uint8_t guaranteed_power      : 6;
    uint8_t power_class           : 2;
    uint8_t potential_power       : 6;
    uint8_t reserved0             : 2;
    uint8_t not_ressens           : 1;
    uint8_t WPID                  : 1;
    uint8_t reserved1             : 6;
} CapabilityType;

typedef struct {
    uint8_t ping                  : 1;
    uint8_t ptr                   : 1;
    uint8_t rsv0                  : 6;
    uint8_t rsv1;
} BRGManualType;

typedef struct {
    union {
        uint8_t buf[MAX_ASK_SIZE];
        struct {
            uint8_t header;
            uint8_t msg[MAX_ASK_SIZE-1];
        };
    };
} AskType;

typedef struct {
    union {
        uint8_t buf[MAX_FSK_SIZE];
        struct {
            uint8_t header;
            uint8_t msg[MAX_FSK_SIZE-1];
        };
    };
} FskType;

typedef struct {
    uint8_t G;
    int8_t Offs;
} FodType;

/// -----------------------------------------------------------------
///                      Customer Registers
///            SRAM address: 0X20000000 ~ 0X20000200
/// -----------------------------------------------------------------
#define OFFSET(TYPE, MEMBER, OFF)                                       \
    TYPE temp;                                                          \
    OFF = (unsigned long)(&(temp.MEMBER)) - (unsigned long)(&(temp));

#define CUSTOMER_REGISTERS_BASE_ADDR      0x20000000

typedef struct {                      // <offset>
    // information setting
    uint16_t        ChipID;             // 0X0000
    uint16_t        CustID;             // 0X0002
    uint32_t        FwVer;              // 0X0004
    uint32_t        HwVer;              // 0X0008
    uint32_t        GitVer;             // 0X000C
    uint16_t        mCode;              // 0X0010 - wpc manufacturer code
    uint16_t        Reserved0012;       // 0X0012
    uint32_t        Reserved0014;       // 0X0014
    uint32_t        FirmwareSize;       // 0X0018
    uint32_t        FirmwareCheck;      // 0X001C
    // receiver setting
    uint16_t        StartOCP;           // 0X0020
    uint16_t        StartOVP;           // 0X0022
    uint16_t        Ilimit;             // 0X0024
    uint16_t        OCP;                // 0X0026
    uint16_t        OVP;                // 0X0028
    uint16_t        OPP;                // 0X002A
    uint16_t        OVP0;               // 0X002C
    uint16_t        OVP1;               // 0X002E
    uint16_t        Vtarget;            // 0X0030
    uint16_t        MaxVrect;           // 0X0032
    uint32_t        VoutSet;            // 0X0034
    uint16_t        EPPVout[4];         // 0X0038
    uint16_t        RecMode2F;          // 0X0040
    uint16_t        RecMode2H;          // 0X0042
    uint16_t        RecHalf;            // 0X0044
    uint16_t        RecFull;            // 0X0046
    uint32_t        RecHys;             // 0X0048
    uint32_t        VrectX;             // 0X004C
    uint32_t        VrectY;             // 0X0050
    uint32_t        VrectAdj;           // 0X0054
    uint16_t        DefaultRPPInterval; // 0X0058
    uint16_t        RPPInterval;        // 0X005A
    uint32_t        RSV1;               // 0X005C
    FodType         FOD[8];             // 0X0060
    FodType         EPPFod[8];          // 0X0070
    uint8_t         FSKDepth;           // 0X0080
    uint8_t         FSKPolarity;        // 0X0081
    uint16_t        FSKThreshold;       // 0X0082
    uint32_t        OTP;                // 0X0084
    uint32_t        Waketime;           // 0X0088
    uint32_t        RSV2[5];            // 0X008C
    uint32_t        TxSetting[24];      // 0X00A0

    uint32_t        CMD;                // 0X0100
    RX_INT          IntEn;              // 0X0104
    RX_INT          IntFlag;            // 0X0108
    RX_INT          IntClr;             // 0X010C
    uint32_t        CEPCnt;             // 0X0110
    SYSMODE         SYSMode;            // 0X0114
    uint32_t        Random;             // 0X0118
    uint32_t        HvWdgtCnt;          // 0x011C

    AskType         Ask;                // 0X0120
    FskType         Fsk;                // 0X0140

    uint32_t        Vout;               // 0X0160
    uint32_t        Iout;               // 0X0164
    uint32_t        Vrect;              // 0X0168
    uint32_t        Vping;              // 0X016C
    uint32_t        PRx;                // 0X0170
    uint32_t        Frequecy;           // 0x0174
    uint32_t        Tdie;               // 0X0178
    uint32_t        RSV6[5];            // 0X017C

    uint32_t        EPTReson;           // 0X0190
    uint32_t        RecMode;            // 0X0194
    IdPktType       IdPkt;              // 0X0198
    SSPktType       SSPkt;              // 0X01A0
    CfgPktType      CfgPkt;             // 0X01A4
    ContractType    ReqContract;        // 0X01AC
    ContractType    CurContract;        // 0X01B0
    PTpktType       PTPkt;              // 0X01B4
    uint32_t        RSV7[2];            // 0X01B8
    uint32_t        MaxPower;           // 0X01C0
    TxInfoType      TxInfo;             // 0X01C4
    CapabilityType  Capability;         // 0X01C8
    uint32_t        RSV8[13];           // 0X01CC
} RXCustType;

typedef struct {                      // <offset>
    // information setting
    uint16_t        ChipID;             // 0X0000
    uint16_t        CustID;             // 0X0002
    uint32_t        FwVer;              // 0X0004
    uint32_t        HwVer;              // 0X0008
    uint32_t        GitVer;             // 0X000C
    uint16_t        mCode;              // 0X0010 - wpc manufacturer code
    uint16_t        Reserved0012;       // 0X0012
    uint32_t        Reserved0014;       // 0X0014
    uint32_t        FirmwareSize;       // 0X0018
    uint32_t        FirmwareCheck;      // 0X001C
    // transmitter setting
    uint16_t        MinFreq;            // 0X00A0
    uint16_t        MaxFreq;            // 0X00A2
    uint16_t        LVP_PING;           // 0X00A4
    uint16_t        LVP_PT;             // 0X00A6
    uint16_t        OCP_PING;           // 0X00A8
    uint16_t        OCP_PT;             // 0X00AA
    uint16_t        SWOVP_PING;         // 0X00AC
    uint16_t        SWOVP_PT;           // 0X00AE
    uint16_t        HWOVP_OVP1;         // 0X00B0
    uint16_t        HWOVP_OVP0;         // 0X00B2
    uint32_t        OTP;                // 0X00B4
    uint32_t        RSV1;               // 0X00B8
    uint16_t        PingInterval;       // 0X00BC
    uint16_t        PingTimeout;        // 0X00BE
    uint16_t        PingTFreq;          // 0X00C0
    uint16_t        PingDuty;           // 0X00C2
    uint32_t        RSV2[3];            // 0X00C4
    uint8_t         DeadTime;           // 0X00D0
    uint8_t         MinDuty;            // 0X00D1
    uint16_t        PWM_RSV;            // 0X00D2
    BRGManualType   BRGManual;          // 0X00D4
    uint16_t        SwitchVoltage;      // 0X00D6
    uint16_t        SwitchCurr;         // 0X00D8
    uint16_t        SwitchHys;          // 0X00DA
    uint16_t        SwitchFreq;         // 0X00DC
    uint16_t        SwitchDuty;         // 0X00DE
    uint8_t         FODIndex;           // 0X00E0
    uint8_t         FODCnt;             // 0X00E1
    uint16_t        FODRSV;             // 0X00E2
    uint16_t        FODPloss[6];        // 0X00E4
    uint32_t        RSV3[4];            // 0X00F0

    uint32_t        CMD;                // 0X0100
    TX_INT          IntEn;              // 0X0104
    TX_INT          IntFlag;            // 0X0108
    TX_INT          IntClr;             // 0X010C
    uint32_t        CEPCnt;             // 0X0110
    SYSMODE         SYSMode;            // 0X0114
    uint32_t        Random;             // 0X0118
    uint32_t        HvWdgtCnt;          // 0x011C

    AskType         Ask;                // 0X0120
    FskType         Fsk;                // 0X0140
    uint32_t        RSV6;               // 0X015C
    uint32_t        Vin;                // 0X0160
    uint32_t        Iin;                // 0X0164
    uint32_t        VBRG;               // 0X0168
    uint32_t        RSV7;               // 0X016C
    uint16_t        POWER_PRx;          // 0X0170
    uint16_t        POWER_PTx;          // 0X0172
    uint16_t        Frequecy;           // 0x0174
    uint16_t        Mperiod;            // 0X0176
    uint32_t        Tdie;               // 0X0178
    uint32_t        Duty;               // 0X017C
    uint32_t        RSV8[4];            // 0X0180

    uint32_t        EPTReson;           // 0X0190
    uint32_t        RSV9;               // 0X0194
    IdPktType       IdPkt;              // 0X0198
    SSPktType       SSPkt;              // 0X01A0
    CfgPktType      CfgPkt;             // 0X01A4
    ContractType    ReqContract;        // 0X01AC
    ContractType    CurContract;        // 0X01B0
    PTpktType       PTPkt;              // 0X01B4
    uint32_t        RSV10[2];           // 0X01B8
    uint32_t        MaxPower;           // 0X01C0
    TxInfoType      TxInfo;             // 0X01C4
    uint32_t        RSV11[14];          // 0X01C8
} TXCustType;

#define sc_err(fmt, ...)                                                \
    do {                                                                \
        printk(KERN_ERR "[sc9624]:%s:" fmt, __func__, ##__VA_ARGS__);   \
    } while(0);

#define sc_info(fmt, ...)                                               \
    do {                                                                \
        printk(KERN_INFO "[sc9624]:%s:" fmt, __func__, ##__VA_ARGS__);  \
    } while(0);

#define sc_dbg(fmt, ...)                                                \
    do {                                                                \
        printk(KERN_DEBUG "[sc9624]:%s:" fmt, __func__, ##__VA_ARGS__); \
    } while(0);


struct sc9624_config {
};

struct sc9624 {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;

    struct mutex data_lock;
    struct mutex i2c_rw_lock;
    struct semaphore suspend_lock;

    struct kthread_worker irq_worker;
    struct kthread_work irq_work;
    struct task_struct *irq_worker_task;
    struct wakeup_source *irq_wake_lock;
    int irq_gpio;
    int irq;
    struct semaphore wls_det_lock;
    struct kthread_worker wls_det_worker;
    struct kthread_work wls_det_work;
    struct task_struct *wls_det_worker_task;
    struct wakeup_source *wls_det_wake_lock;
    int wls_det_gpio;
    int wls_det_irq;

    int work_mode;
    bool fw_program;
    bool fw_update_force;
    int fw_update_status;

    struct sc9624_config config;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *wl_psy;
    uint16_t reg_addr;
    uint32_t reg_data;
    const char *wls_fw_name;
    int wls_mode_select;
    int wls_mc_det;
    SYSMODE sys_mode;
    struct wireless_device *wls_dev;
    struct wls_rx_ops rx_ops;
    struct mutex event_lock;
};

int mtp_program(struct sc9624 *sc);
int mtp_erase(struct sc9624 *sc);
int sc9624_get_fwver(struct sc9624 *sc, uint32_t *fw_ver);
#endif
