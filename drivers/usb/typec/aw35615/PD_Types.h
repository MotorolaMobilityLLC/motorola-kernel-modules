/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinc.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __USBPD_TYPES_H__
#define __USBPD_TYPES_H__
#include "aw_types.h"

#define STAT_BUSY               0
#define STAT_SUCCESS            1
#define STAT_ERROR              2
#define STAT_ABORT              3

/* USB PD Header Message Definitions */
#define PDPortRoleSink          0
#define PDPortRoleSource        1
#define PDDataRoleUFP           0
#define PDDataRoleDFP           1
#define PDCablePlugSource       0
#define PDCablePlugPlug         1

/* USB PD Control Message Types */
#define CMTGoodCRC              0x1
#define CMTGotoMin              0x2
#define CMTAccept               0x3
#define CMTReject               0x4
#define CMTPing                 0x5
#define CMTPS_RDY               0x6
#define CMTGetSourceCap         0x7
#define CMTGetSinkCap           0x8
#define CMTDR_Swap              0x9
#define CMTPR_Swap              0xa
#define CMTVCONN_Swap           0xb
#define CMTWait                 0xc
#define CMTSoftReset            0xd
#define CMTNotSupported         0x10
#define CMTGetSourceCapExt      0x11
#define CMTGetStatus            0x12
#define CMTGetFRSwap            0x13
#define CMTGetPPSStatus         0x14
#define CMTGetCountryCodes      0x15
#define CMTGetSourceInfo        0x17
#define CMTGetRevisonMessage    0x18

/* USB PD Data Message Types */
#define DMTSourceCapabilities   0x1
#define DMTRequest              0x2
#define DMTBIST                 0x3
#define DMTSinkCapabilities     0x4
#define DMTBatteryStatus        0x5
#define DMTAlert                0x6
#define DMTGetCountryInfo       0x7
#define DMTSourceInfo           0xb
#define DMTRevisionInfo         0xc
#define DMTVenderDefined        0xf

/* Extended message types opcode */
#ifdef AW_HAVE_EXT_MSG
#define EXTSourceCapExt         0x1
#define EXTStatus               0x2
#define EXTGetBatteryCap        0x3
#define EXTGetBatteryStatus     0x4
#define EXTBatteryCapabilities  0x5
#define EXTGetManufacturerInfo  0x6
#define EXTManufacturerInfo     0x7
#define EXTSecurityRequest      0x8
#define EXTSecurityResponse     0x9
#define EXTFirmUpdateRequest    0xa
#define EXTFirmUpdateResponse   0xb
#define EXTPPSStatus            0xc
#define EXTCountryInfo          0xd
#define EXTCountryCodes         0xe

/** Extended message data size in bytes
 *  This must be set in the extended header
 */
#define EXT_STATUS_MSG_BYTES        5
#define EXT_MAX_MSG_LEN             260
#endif /* AW_HAVE_EXT_MSG */
#define EXT_MAX_MSG_LEGACY_LEN      26

/* BIST Data Objects */
#define BDO_BIST_Receiver_Mode      0x0  /* Not Implemented */
#define BDO_BIST_Transmit_Mode      0x1  /* Not Implemented */
#define BDO_Returned_BIST_Counters  0x2  /* Not Implemented */
#define BDO_BIST_Carrier_Mode_0     0x3  /* Not Implemented */
#define BDO_BIST_Carrier_Mode_1     0x4  /* Not Implemented */
#define BDO_BIST_Carrier_Mode_2     0x5  /* Implemented */
#define BDO_BIST_Carrier_Mode_3     0x6  /* Not Implemented */
#define BDO_BIST_Eye_Pattern        0x7  /* Not Implemented */
#define BDO_BIST_Test_Data          0x8  /* Implemented */

/* USB PD Timing Parameters in milliseconds * scale factor */
/* Some values are changed from spec to allow for processor response time. */
#define tRetry                  (3       * TICK_SCALE_TO_MS)
#define tNoResponse             (5000    * TICK_SCALE_TO_MS)
#define tSenderResponse         (26      * TICK_SCALE_TO_MS)
#define tTypeCSendSourceCap     (150     * TICK_SCALE_TO_MS)
#define tSinkWaitCap            (500     * TICK_SCALE_TO_MS)
#define tTypeCSinkWaitCap       (550     * TICK_SCALE_TO_MS)
#define tSrcTransition          (25      * TICK_SCALE_TO_MS)
#define tPSHardReset            (30      * TICK_SCALE_TO_MS)
#define tPSHardResetMax         (34      * TICK_SCALE_TO_MS)
#define tPSTransition           (515     * TICK_SCALE_TO_MS)
#define tPSSourceOff            (835     * TICK_SCALE_TO_MS)
#define tPSSourceOn             (435     * TICK_SCALE_TO_MS)
#define tVCONNSourceOn          (90      * TICK_SCALE_TO_MS)
#define tVCONNTransition        (20      * TICK_SCALE_TO_MS)
#define tBISTContMode           (35      * TICK_SCALE_TO_MS)
#define tSwapSourceStart        (25      * TICK_SCALE_TO_MS)
#define tSrcRecover             (675     * TICK_SCALE_TO_MS)
#define tSrcRecoverMax          (1000    * TICK_SCALE_TO_MS)
#define tGoodCRCDelay           (3       * TICK_SCALE_TO_MS)
#define t5To12VTransition       (8       * TICK_SCALE_TO_MS)
#define tSafe0V                 (650     * TICK_SCALE_TO_MS)
#define tSrcTurnOn              (275     * TICK_SCALE_TO_MS)
#define tVBusSwitchDelay        (5       * TICK_SCALE_TO_MS)
#define tVBusPollShort          (10      * TICK_SCALE_TO_MS)
#define tVBusPollLong           (100     * TICK_SCALE_TO_MS)
#define tSourceRiseTimeout      (350     * TICK_SCALE_TO_MS)
#define tHardResetOverhead      (0       * TICK_SCALE_TO_MS)
#define tPPSTimeout             (14000   * TICK_SCALE_TO_MS)
#define tPPSRequest             (5000    * TICK_SCALE_TO_MS)
#define tWaitCableReset         (1       * TICK_SCALE_TO_MS)
#define tChunkReceiverRequest   (15      * TICK_SCALE_TO_MS)
#define tChunkReceiverResponse  (15      * TICK_SCALE_TO_MS)
#define tChunkSenderRequest     (30      * TICK_SCALE_TO_MS)
#define tChunkSenderResponse    (30      * TICK_SCALE_TO_MS)
#define tChunkingNotSupported   (40      * TICK_SCALE_TO_MS)

#define nHardResetCount         2
#define nRetryCount             3
#define nCapsCount              50
#define nDiscoverIdentityCount  18  /* Common multiple of 2 and 3 retries */
/**
 * @brief Message header struct
 */
typedef union {
	AW_U16 word;
	AW_U8 byte[2];

	struct {
		AW_U16 MessageType :5;         /* 0-4      : Message Type */
		AW_U16 PortDataRole :1;        /* 5        : Port Data Role */
		AW_U16 SpecRevision :2;        /* 6-7      : Specification Revision */
		AW_U16 PortPowerRole :1;       /* 8        : Port Power Role */
		AW_U16 MessageID :3;           /* 9-11     : Message ID */
		AW_U16 NumDataObjects :3;      /* 12-14    : Number of Data Objects */
		AW_U16 Extended :1;            /* 15       : Extended message */
	};
} sopMainHeader_t;

/**
 * @brief Cable message header struct
 */
typedef union {
	AW_U16 word;
	AW_U8 byte[2];

	struct {
		AW_U16 MessageType :5;         /* 0-4      : Message Type */
		AW_U16:1;                      /* 5        : */
		AW_U16 SpecRevision :2;        /* 6-7      : Specification Revision */
		AW_U16 CablePlug :1;           /* 8        : Cable Plug */
		AW_U16 MessageID :3;           /* 9-11     : Message ID */
		AW_U16 NumDataObjects :3;      /* 12-14    : Number of Data Objects */
		AW_U16 Extended :1;            /* 15       : Extended message */
	};
} sopPlugHeader_t;

/**
 * @brief Extended message header struct
 */
typedef union {
	AW_U16 word;
	AW_U8 byte[2];
	struct {
		AW_U16 DataSize :9;            /* Bit[0-8] Length of data in bytes */
		AW_U16:1;                      /* Bit[9] */
		AW_U16 ReqChunk :1;            /* Bit[10] Request chunk boolean */
		AW_U16 ChunkNum :4;            /* Bit[11-14] Chunk Number */
		AW_U16 Chunked :1;             /* Bit[15] Chunked flag */
	};
} ExtHeader_t;

/**
 * @brief PDO structs for supplies, sinks, requests, etc.
 */
typedef union {
	AW_U32 object;
	AW_U16 word[2];
	AW_U8 byte[4];
	struct {
		AW_U32:30;
		AW_U32 SupplyType :2;
	} PDO;                              /* General purpose PDO */
	struct {
		AW_U32 MaxCurrent :10;         /* Max current in 10mA units */
		AW_U32 Voltage :10;            /* Voltage in 50mV units */
		AW_U32 PeakCurrent :2;         /* Peak current (from Ioc ratings) */
		AW_U32:3;
		AW_U32 DataRoleSwap :1;        /* Supports DR_Swap message */
		AW_U32 USBCommCapable :1;      /* USB communications capable */
		AW_U32 ExternallyPowered :1;   /* Externally powered  */
		AW_U32 USBSuspendSupport :1;   /* USB Suspend Supported */
		AW_U32 DualRolePower :1;       /* Dual-Role power */
		AW_U32 SupplyType :2;          /* Supply Type - Fixed: 0  */
	} FPDOSupply;                       /* Fixed PDO for Supplies */
	struct {
		AW_U32 OperationalCurrent :10; /* Operational current in 10mA units */
		AW_U32 Voltage :10;            /* Voltage in 50mV units */
		AW_U32:5;
		AW_U32 DataRoleSwap :1;        /* Supports DR_Swap message */
		AW_U32 USBCommCapable :1;      /* USB communications capable */
		AW_U32 ExternallyPowered :1;   /* Externally powered  */
		AW_U32 HigherCapability :1;    /* Sink needs more than vSafe5V */
		AW_U32 DualRolePower :1;       /* Dual-Role power */
		AW_U32 SupplyType :2;          /* Supply Type - Fixed: 0  */
	} FPDOSink;                         /* Fixed Power Data Object for Sinks */
	struct {
		AW_U32 MaxPower :10;           /* Max power in 250mW units */
		AW_U32 MinVoltage :10;         /* Minimum Voltage in 50mV units */
		AW_U32 MaxVoltage :10;         /* Maximum Voltage in 50mV units */
		AW_U32 SupplyType :2;          /* Supply Type - Battery: 1 */
	} BPDO;                             /* Battery Power Data Object */
	struct {
		AW_U32 MaxCurrent :10;         /* Max current in 10mA units */
		AW_U32 MinVoltage :10;         /* Minimum Voltage in 50mV units */
		AW_U32 MaxVoltage :10;         /* Maximum Voltage in 50mV units */
		AW_U32 SupplyType :2;          /* Supply Type - Variable: 2 */
	} VPDO;                             /* Variable Power Data Object */
	struct {
		AW_U32 MaxCurrent :7;          /* Max current in 50mA units */
		AW_U32:1;
		AW_U32 MinVoltage :8;          /* Min voltage in 100mV units */
		AW_U32:1;
		AW_U32 MaxVoltage :8;          /* Max voltage in 100mV units */
		AW_U32:5;
		AW_U32 SupplyType :2;          /* Apdo type 3 */
	} PPSAPDO;
	struct {
		AW_U32 MinMaxCurrent :10;      /* Min/Max current in 10mA units */
		AW_U32 OpCurrent :10;          /* Operating current in 10mA units */
		AW_U32:3;
		AW_U32 UnChnkExtMsgSupport :1; /* Unchunked extended msg supported */
		AW_U32 NoUSBSuspend :1;        /* USB suspend not available */
		AW_U32 USBCommCapable :1;      /* USB communications capable */
		AW_U32 CapabilityMismatch :1;  /* Sink cannot make valid request */
		AW_U32 GiveBack :1;            /* Sink will respond to the GotoMin */
		AW_U32 ObjectPosition :3;      /* Object being requested */
		AW_U32:1;
	} FVRDO;                            /* Fixed/Variable Request Data Object*/
	struct {
		AW_U32 MinMaxPower :10;        /* Min/Max power in 250mW units */
		AW_U32 OpPower :10;            /* Operating power in 250mW units */
		AW_U32:3;
		AW_U32 UnChnkExtMsgSupport :1; /* Unchunked extended msg supported */
		AW_U32 NoUSBSuspend :1;        /* USB suspend not available */
		AW_U32 USBCommCapable :1;      /* USB communications capable */
		AW_U32 CapabilityMismatch :1;  /* Sink cannot make valid request */
		AW_U32 GiveBack :1;            /* Sink will respond to the GotoMin */
		AW_U32 ObjectPosition :3;      /* Object being requested */
		AW_U32:1;
	} BRDO;                             /* Battery Request Data Object */
	struct {
		AW_U32 OpCurrent :7;           /* Operating current in 50mA units */
		AW_U32:2;
		AW_U32 Voltage :12;            /* Requested voltage in 20mV units */
		AW_U32:2;
		AW_U32 UnChnkExtMsgSupport :1; /* Unchunked extended msg supported */
		AW_U32 NoUSBSuspend :1;        /* USB suspend not available */
		AW_U32 USBCommCapable :1;      /* USB communications capable */
		AW_U32 CapabilityMismatch :1;  /* Sink cannot make valid request */
		AW_U32:1;
		AW_U32 ObjectPosition :3;      /* PDO object index */
		AW_U32:1;
	} PPSRDO;                           /* PPS Request Data Object */
	struct {
		AW_U32 OutputVoltage: 16;      /* Output voltage in 20mV, or 0xFFFF */
		AW_U32 OutputCurrent: 8;       /* Output current in 50mA, or 0xFF */
		AW_U32:1;
		AW_U32 Ptf:2;                  /* Temperature flag */
		AW_U32 Omf:1;                  /* Current foldback mode */
		AW_U32:4;
	} PPSSDB;                           /* PPS Data block */
	struct {
		AW_U32:16;
		AW_U32 HotSwapBat0:1;
		AW_U32 HotSwapBat1:1;
		AW_U32 HotSwapBat2:1;
		AW_U32 HotSwpaBat3:1;
		AW_U32 FixedBat4:1;
		AW_U32 FixedBat5:1;
		AW_U32 FixedBat6:1;
		AW_U32 FixedBat7:1;
		AW_U32:1;
		AW_U32 Battery:1;              /* Battery status changed */
		AW_U32 OCP:1;                  /* Over current */
		AW_U32 OTP:1;                  /* Over temperature */
		AW_U32 OpCondition:1;          /* Operating condition */
		AW_U32 Input:1;                /* Input change event */
		AW_U32 OVP:1;                  /* Over Voltage */
		AW_U32:1;
	} ADO;                              /* Alert Data Object */
	struct {
		AW_U32 VendorDefined :15;      /* Defined by the vendor */
		AW_U32 VDMType :1;             /* Unstructured or structured header */
		AW_U32 VendorID :16;           /* Unique VID value */
	} UVDM;
	struct {
		AW_U32 Command :5;             /* VDM Command */
		AW_U32:1;
		AW_U32 CommandType :2;         /* Init, ACK, NAK, BUSY... */
		AW_U32 ObjPos :3;              /* Object position */
		AW_U32:2;
		AW_U32 Version :2;             /* Structured VDM version */
		AW_U32 VDMType :1;             /* Unstructured or structured header */
		AW_U32 SVID :16;               /* Unique SVID value */
	} SVDM;
} doDataObject_t;

/**
 * @brief Generic PDO struct
 */
typedef struct {
	AW_U32 Voltage :10;
	AW_U32 Current :10;
	AW_U32 MinVoltage :10;
	AW_U32 MaxVoltage :10;
	AW_U32 MaxPower :10;
	AW_U32 SupplyType :2;
} doPDO_t;

/**
 * @brief Supply status struct
 */
typedef union {
	AW_U8 byte[5];
	struct {
		AW_U8 InternalTemp : 8;        /* Bit[0-7] Temperature (Deg C) */
		AW_U8:1;                       /* Bit[0] */
		AW_U8 ExternalPower : 1;       /* Bit[1] Externally powered */
		AW_U8 ACDC : 1;                /* Bit[2] Ext. Powered 0:DC 1:AC */
		AW_U8 InternalBattery : 1;     /* Bit[3] Int. Powered - Battery */
		AW_U8 InternalNonBattery : 1;  /* Bit[4] Int. Powered - Non-battery */
		AW_U8:3;                       /* Bit[5-7] */
		AW_U8 PresentBatteryInput : 8; /* Bit[0-7] */
		AW_U8:1;                       /* Bit[0] */
		AW_U8 EventOCP : 1;            /* Bit[1] Chunk Number */
		AW_U8 EventOTP : 1;            /* Bit[2] Chunk Number */
		AW_U8 EventOVP : 1;            /* Bit[3] Chunk Number */
		AW_U8 EventCVCF : 1;           /* Bit[4] Chunk Number */
		AW_U8:3;                       /* Bit[5-7] */
		AW_U8:1;                       /* Bit[0] */
		AW_U8 TemperatureStatus : 2;   /* Bit[1-2] Temp Code */
		AW_U8:5;                       /* Bit[3-7] */
	};
} Status_t;

/**
 * @brief PPS status struct
 */
typedef union {
	AW_U32 object;
	AW_U16 word[2];
	AW_U8 byte[4];
	struct {
		AW_U32 OutputVoltage :16;      /* Bit[0-15] Output Voltage (20mV) */
		AW_U32 OutputCurrent :8;       /* Bit[0-7] Output Current (50mA) */
		AW_U32:4;                      /* Bit[4-7] */
		AW_U32 FlagOMF :1;             /* Bit[3] Operating Mode Flag */
		AW_U32 FlagPTF :2;             /* Bit[1-2] Present Temp Flag */
		AW_U32:1;                      /* Bit[0] */
	};
} PPSStatus_t;

typedef union {
	AW_U8 byte[25];
	struct {                              /* Byte Offsets */
		AW_U16 VID:16;                /* 0-1   : Vendor ID */
		AW_U16 PID:16;                /* 2-3   : Product ID */
		AW_U32 XID:32;                /* 4-7   : USB-IF extended ID */

		AW_U8 FWVersion:8;           /* 8     : Firmware Version */
		AW_U8 HWVersion:8;            /* 9     : Hardware Version */
		AW_U8 VotageReg:8;            /* 10    : Bitfield (See Spec) */
		AW_U8 HoldupTime:8;           /* 11    : Output Hold Time */
		AW_U8 Compliance:8;           /* 12    : Bitfield (See spec) */
		AW_U8 TouchCurrent:8;         /* 13    : Bitfield (See spec) */
		AW_U16 PeakCurrent1:16;       /* 14-15 : Bitfield (See spec) */
		AW_U16 PeakCurrent2:16;       /* 16-17 : Bitfield (See spec) */
		AW_U16 PeakCurrent3:16;       /* 18-19 : Bitfield (See spec) */

		AW_U8 TouchTemp:8;            /* 20    : Temp Spec */
		AW_U8 SourceInputs:8;         /* 21    : Bitfield (See spec) */
		AW_U8 Batteries:8;            /* 22    : Battery Count */
		AW_U8 SprPDP:8;               /* 23    : Source's PDP */
		AW_U8 EprPDP:8;               /* 24    : EPR Source 's PDP */
	};
} ExtSrcCapBlock_t;

/**
 * @brief Country info response struct
 */
typedef union {
	AW_U8 bytes[260];
	struct {
		AW_U8 CountryCode[2];
		AW_U8 Reserved[2];
		AW_U8 Data[256];
	};
} CountryInfoResp;

/**
 * @brief Country code request struct
 */
typedef union {
	AW_U8 bytes[4];
	struct {
		AW_U8 Reserved[2];
		AW_U8 CountryCode[2];
	};
} CountryInfoReq;

/**
 * @brief Non-message entries for the PD message log
 */
typedef enum {
	pdtNone = 0,                /* Reserved token (nothing) */
	pdtAttach,                  /* USB PD attached */
	pdtDetach,                  /* USB PD detached */
	pdtHardReset,               /* USB PD hard reset */
	pdtBadMessageID,            /* An incorrect message ID was received */
	pdtCableReset,              /* Cable reset */
	pdtHardResetTxd,            /* USB PD hard reset transmitted */
	pdtHardResetRxd,            /* USB PD hard reset received */
} USBPD_BufferTokens_t;

/**
 * @brief Policy Engine state machine enum
 */
typedef enum {
	/* 0 */
	peDisabled = 0,             /* Policy engine is disabled */
	FIRST_PE_ST = peDisabled,   /* mark lowest value in enum */
	peErrorRecovery,            /* Error recovery state */
	peSourceHardReset,          /* Received a hard reset */
	peSourceSendHardReset,      /* Source send a hard reset */
	peSourceSoftReset,          /* Received a soft reset */
	peSourceSendSoftReset,      /* Send a soft reset */
	peSourceStartup,            /* Initial state */
	peSourceSendCaps,           /* Send the source capabilities */
	peSourceDiscovery,          /* Waiting to detect a USB PD sink */
	peSourceDisabled,           /* Disabled state */
	/* 10 */
	peSourceTransitionDefault,  /* Transition to default 5V state */
	peSourceNegotiateCap,       /* Negotiate capability and PD contract */
	peSourceCapabilityResponse, /* Respond to a request with a reject/wait */
	peSourceWaitNewCapabilities, /* Wait for new Source Caps from DPM */
	peSourceTransitionSupply,   /* Transition the power supply */
	peSourceReady,              /* Contract is in place and voltage is stable */
	peSourceGiveSourceCaps,     /* State to resend source capabilities */
	peSourceGetSinkCaps,        /* State to request the sink capabilities */
	peSourceSendPing,           /* State to send a ping message */
	peSourceGotoMin,            /* State to send gotoMin and prep the supply */
	/* 20 */
	peSourceGiveSinkCaps,       /* State to send sink caps if dual-role */
	peSourceGetSourceCaps,      /* State to request source caps from the UFP */
	peSourceSendDRSwap,         /* State to send a DR_Swap message */
	peSourceEvaluateDRSwap,     /* Evaluate DR swap request */
	peSourceAlertReceived,      /* Source has received an alert */

	peSinkHardReset,            /* Received a hard reset */
	peSinkSendHardReset,        /* Sink send hard reset */
	peSinkSoftReset,            /* Sink soft reset */
	peSinkSendSoftReset,        /* Sink send soft reset */
	peSinkTransitionDefault,    /* Transition to the default state */
	/* 30 */
	peSinkStartup,              /* Initial sink state */
	peSinkDiscovery,            /* Sink discovery state */
	peSinkWaitCaps,             /* Sink wait for capabilities state */
	peSinkEvaluateCaps,         /* Sink evaluate the rx'd source caps */
	peSinkSelectCapability,     /* Sink selecting a capability */
	peSinkTransitionSink,       /* Sink transitioning the current power */
	peSinkReady,                /* Sink ready state */
	peSinkGiveSinkCap,          /* Sink send capabilities state */
	peSinkGetSourceCap,         /* Sink get source capabilities state */
	peSinkGetSinkCap,           /* Sink get the sink caps of the source */
	/* 40 */
	peSinkGiveSourceCap,        /* Sink send the source caps if dual-role */
	peSinkSendDRSwap,           /* State to send a DR_Swap message */
	peSinkAlertReceived,        /* Sink received alert message */
	peSinkEvaluateDRSwap,       /* Evaluate DR swap request */
	peSourceSendVCONNSwap,      /* Initiate a VCONN swap sequence */
	peSourceEvaluateVCONNSwap,  /* Evaluate VCONN swap request */
	peSinkSendVCONNSwap,        /* Initiate a VCONN swap sequence */
	peSinkEvaluateVCONNSwap,    /* Evaluate VCONN swap request */
	peSourceSendPRSwap,         /* Initiate a PR Swap sequence */
	peSourceEvaluatePRSwap,     /* Evaluate PR swap request */
	/* 50 */
	peSinkSendPRSwap,           /* Initiate a PR Swap sequence */
	peSinkEvaluatePRSwap,       /* Evaluate PR swap request */

	peGetCountryCodes,          /* Send Get country code message */
	peGiveCountryCodes,         /* Send country codes */
	peNotSupported,             /* Send a reject/NS to unknown msg */
	peGetPPSStatus,             /* Sink request PPS source status */
	peGivePPSStatus,            /* Source provide PPS status */
	peGiveCountryInfo,          /* Send country info */
	peSourceGiveSourceCapExt,    /* Send source cap extended info */

	/* VDM states */
	peGiveVdm,
	/* ---------- UFP VDM State Diagram ---------- */
	peUfpVdmGetIdentity,        /* Requesting Identity information from DPM */
	FIRST_VDM_STATE = peUfpVdmGetIdentity,
	/* 60 */
	peUfpVdmSendIdentity,       /* Sending Discover Identity ACK */

	peUfpVdmGetSvids,           /* Requesting SVID info from DPM */
	peUfpVdmSendSvids,          /* Sending Discover SVIDs ACK */
	peUfpVdmGetModes,           /* Requesting Mode info from DPM */
	peUfpVdmSendModes,          /* Sending Discover Modes ACK */
	peUfpVdmEvaluateModeEntry,  /* Evaluate request to enter a mode */
	peUfpVdmModeEntryNak,       /* Sending Enter Mode NAK response */
	peUfpVdmModeEntryAck,       /* Sending Enter Mode ACK response */
	peUfpVdmModeExit,           /* Evalute request to exit mode */
	peUfpVdmModeExitNak,        /* Sending Exit Mode NAK reponse */
	peUfpVdmModeExitAck,        /* Sending Exit Mode ACK Response */

	/* ---------- UFP VDM Attention State Diagram ---------- */
	peUfpVdmAttentionRequest,   /* Sending Attention Command */
	/* ---------- DFP to UFP VDM Discover Identity State Diagram ---------- */
	peDfpUfpVdmIdentityRequest, /* Sending Identity Request */
	peDfpUfpVdmIdentityAcked,   /* Inform DPM of Identity */
	peDfpUfpVdmIdentityNaked,   /* Inform DPM of result */
	/* ---------- DFP to Cable Plug VDM Discover Identity State Diagram --- */
	peDfpCblVdmIdentityRequest, /* Sending Identity Request */
	peDfpCblVdmIdentityAcked,   /* Inform DPM */
	peDfpCblVdmIdentityNaked,   /* Inform DPM */
	/* ---------- DFP VDM Discover SVIDs State Diagram ---------- */
	peDfpVdmSvidsRequest,       /* Sending Discover SVIDs request */
	peDfpVdmSvidsAcked,         /* Inform DPM */
	peDfpVdmSvidsNaked,         /* Inform DPM */

	/* ---------- DFP VDM Discover Modes State Diagram ---------- */
	peDfpVdmModesRequest,       /* Sending Discover Modes request */
	peDfpVdmModesAcked,         /* Inform DPM */
	peDfpVdmModesNaked,         /* Inform DPM */
	/* ---------- DFP VDM Enter Mode State Diagram ---------- */
	peDfpVdmModeEntryRequest,   /* Sending Mode Entry request */
	peDfpVdmModeEntryAcked,     /* Inform DPM */
	peDfpVdmModeEntryNaked,     /* Inform DPM */
	/* ---------- DFP VDM Exit Mode State Diagram ---------- */
	peDfpVdmModeExitRequest,    /* Sending Exit Mode request */
	peDfpVdmExitModeAcked,      /* Inform DPM */
	/* if Exit Mode not Acked, go to Hard Reset state */
	/* ---------- Source Startup VDM Discover Identity State Diagram ------ */
	peSrcVdmIdentityRequest,    /* sending Discover Identity request */
	peSrcVdmIdentityAcked,      /* inform DPM */

	peSrcVdmIdentityNaked,      /* inform DPM */
	/* ---------- DFP VDM Attention State Diagram ---------- */
	peDfpVdmAttentionRequest,   /* Attention Request received */
	/* ---------- Cable Ready VDM State Diagram ---------- */
	peCblReady,                 /* Cable power up state? */
	/* ---------- Cable Discover Identity VDM State Diagram ---------- */
	peCblGetIdentity,           /* Discover Identity request received */
	peCblGetIdentityNak,        /* Respond with NAK */
	peCblSendIdentity,          /* Respond with Ack */
	/* ---------- Cable Discover SVIDs VDM State Diagram ---------- */
	peCblGetSvids,              /* Discover SVIDs request received */
	peCblGetSvidsNak,           /* Respond with NAK */
	peCblSendSvids,             /* Respond with ACK */
	/* ---------- Cable Discover Modes VDM State Diagram ---------- */
	peCblGetModes,              /* Discover Modes request received */

	peCblGetModesNak,           /* Respond with NAK */
	peCblSendModes,             /* Respond with ACK */
	/* ---------- Cable Enter Mode VDM State Diagram ---------- */
	peCblEvaluateModeEntry,     /* Enter Mode request received */
	peCblModeEntryAck,          /* Respond with NAK */
	peCblModeEntryNak,          /* Respond with ACK */
	/* ---------- Cable Exit Mode VDM State Diagram ---------- */
	peCblModeExit,              /* Exit Mode request received */
	peCblModeExitAck,           /* Respond with NAK */
	peCblModeExitNak,           /* Respond with ACK */
	/* ---------- DP States ---------- */
	peDpRequestStatus,          /* Requesting PP Status */
	peDpRequestStatusAck,
	peDpRequestStatusNak,
	peDpRequestConfig,         /* Request Port partner Config */
	peDpRequestConfigAck,
	peDpRequestConfigNak,
	LAST_VDM_STATE = peDpRequestConfigNak,
	/* ---------- BIST Receive Mode --------------------- */
	PE_BIST_Receive_Mode,       /* Bist Receive Mode */
	PE_BIST_Frame_Received,     /* Test Frame received by Protocol layer */
	/* ---------- BIST Carrier Mode and Eye Pattern ----- */
	PE_BIST_Carrier_Mode_2,     /* BIST Carrier Mode 2 */
	PE_BIST_Test_Data,          /* BIST Test Data State */
	LAST_PE_ST = PE_BIST_Test_Data, /* mark last value in enum */
	dbgGetRxPacket,             /* Debug point for Rx packet handling */
	dbgSendTxPacket,            /* Debug point for Tx packet handling */
	peSendCableReset,           /* State to send cable reset */
	peSendGenericCommand,       /* Send an arbitrary command from the GUI */
	peSendGenericData,          /* Send arbitrary data from the GUI */
	peGiveSourceInfo,
	peGiveRevisonMessage,
	peGetBatteryCap,
	peGetBatteryStatus,
} PolicyState_t;

/**
 * @brief Protocol state machine enum
 */
typedef enum {
	PRLDisabled = 0,            /* Protocol state machine is disabled */
	PRLIdle,                    /* Ready to Rx or Tx */
	PRLReset,                   /* Rx'd a soft reset or exit from hard reset */
	PRLResetWait,               /* Waiting for the hard reset to complete */
	PRLRxWait,                  /* Actively receiving a message */
	PRLTxSendingMessage,        /* Pass msg to the device */
	PRLTxWaitForPHYResponse,    /* Wait for activity on CC or a timeout */
	PRLTxVerifyGoodCRC,         /* Verify the good CRC message */
	PRLManualRetries,           /* Handle retries manually */
	/* ------- BIST Receiver Test -------- */
	PRL_BIST_Rx_Reset_Counter,  /* Reset BISTErrorCounter and preload PRBS */
	PRL_BIST_Rx_Test_Frame,     /* Wait for test Frame form PHY */
	PRL_BIST_Rx_Error_Count,    /* Set and send BIST error count msg to PHY */
	PRL_BIST_Rx_Inform_Policy,  /* Inform PE error count has been sent */
} ProtocolState_t;

/**
 * @brief Protocol transmit state machine enum
 */
typedef enum {
	txIdle = 0,
	txReset,
	txSend,
	txBusy,
	txWait,
	txSuccess,
	txError,
	txCollision,
	txAbort
} PDTxStatus_t;

/**
 * @brief Supply type for PDOs
 */
typedef enum {
	pdoTypeFixed = 0,
	pdoTypeBattery,
	pdoTypeVariable,
	pdoTypeAugmented,
	pdoTypeReserved
} pdoSupplyType;

/**
 * @brief Auto VDM process status
 */
typedef enum {
	AUTO_VDM_INIT,
	AUTO_VDM_DISCOVER_ID_PP,
	AUTO_VDM_DISCOVER_SVIDS_PP,
	AUTO_VDM_DISCOVER_MODES_PP,
	AUTO_VDM_ENTER_MODE_PP,
	AUTO_VDM_DP_GET_STATUS,
	AUTO_VDM_DP_SET_CONFIG,
	AUTO_VDM_DONE
} VdmDiscoveryState_t;

/**
 * @brief Supported SOP types
 */
typedef enum {
	SOP_TYPE_SOP,
	SOP_TYPE_SOP1,
	SOP_TYPE_SOP2,
	SOP_TYPE_SOP1_DEBUG,
	SOP_TYPE_SOP2_DEBUG,
	SOP_TYPE_NUM,
	SOP_TYPE_ERROR = 0xFF
} SopType;

/**
 * @brief Supported PD Specification Revisions
 */
typedef enum {
	USBPDSPECREV1p0 = 0x0,
	USBPDSPECREV2p0 = 0x1,
	USBPDSPECREV3p0 = 0x2,
	USBPDSPECREVMAX = 0x3
} SpecRev;

/**
 * @brief State of extended message Tx/Rx handling
 */
typedef enum {
	NoXfer,
	TXing,
	RXing,
} ExtMsgState_t;

/*
 * @brief State for cable reset
 */
typedef enum {
	CBL_RST_DISABLED,
	CBL_RST_START,
	CBL_RST_VCONN_SOURCE,
	CBL_RST_DR_DFP,
	CBL_RST_SEND,
} CableResetState_t;

#endif /* __USBPD_TYPES_H__ */
