/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .h file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef _PORT_H_
#define _PORT_H_
#include "TypeC_Types.h"
#include "PD_Types.h"
#include "aw35615_driver.h"
#ifdef AW_HAVE_VDM
#include "vdm/vdm_callbacks_defs.h"
#endif /* AW_HAVE_VDM */
#ifdef AW_HAVE_DP
#include "vdm/DisplayPort/dp.h"
#endif /* AW_HAVE_DP */
#include "modules/observer.h"
#include "modules/dpm.h"
#include "timer.h"

/* Size of Rx/Tx FIFO protocol buffers */
#define AW_PROTOCOL_BUFFER_SIZE 64

/* Number of timer objects in list */
#define AW_NUM_TIMERS 10

#ifndef NUM_PORTS
#define NUM_PORTS		(1) /* Number of ports in this system */
#endif /* NUM_PORTS */

/**
 * Current port connection state
 */
typedef enum {
	SINK = 0,
	SOURCE
} SourceOrSink;

/**
 * All state variables here for now.
 */
struct Port {
	DevicePolicy_t         *dpm;
	PortConfig_t            PortConfig;
	AW_U8                  PortID;                     /* Optional Port ID */
	AW_U8                  I2cAddr;
	DeviceReg_t             Registers;
	AW_BOOL                TCIdle;                     /* True: Type-C idle */
	AW_BOOL                PEIdle;                     /* True: PE idle */
	AW_U32                 Bist_Flag;
	AW_U8                  wait_toggle_num;
	AW_U8                  debounce_vbus;
	AW_U8                  vbus_flag_last;
	AW_U8                  try_wait_vbus;
	AW_U8                  old_pe_state;
	AW_U8                  old_policy_state;
	AW_U8                  old_ConnState;

	AW_U32                 SINK_num;
	AW_U8                  SINK_Flag;
	AW_BOOL                sink_timer;
	AW_U32                 SOURCE_num;
	AW_U8                  SOURCE_Flag;
	AW_U8                  SOURCE_Flag_end;
	AW_U8                  get_sink_cap_flag;
	AW_U8                  sink_bist_reg;
	AW_U16                 snk_pdo_size;
	AW_U32                 snk_pdo_vol[7];
	AW_U32                 snk_pdo_cur[7];
	AW_U16                 src_pdo_size;
	AW_U32                 src_pdo_vol[7];
	AW_U32                 src_pdo_cur[7];
	AW_U8                  snk_tog_time; /* toggle sink time */
	AW_U8                  src_tog_time; /* toggle source time */
	AW_S32                 usb_commcapable;

	/* All Type C State Machine variables */
	CCTermType              CCTerm;                     /* Active CC */
	CCTermType              CCTermCCDebounce;           /* Debounced CC */
	CCTermType              CCTermPDDebounce;
	CCTermType              CCTermPDDebouncePrevious;
	CCTermType              VCONNTerm;
	AW_BOOL                 pd_state;
	AW_BOOL                 no_clear_message;

	SourceOrSink            sourceOrSink;           /* TypeC Src or Snk */
	USBTypeCCurrent         SinkCurrent;            /* PP Current */
	USBTypeCCurrent         SourceCurrent;          /* Our Current */
	CCOrientation           CCPin;                  /* CC == CC1 or CC2 */
	AW_BOOL                SMEnabled;               /* TypeC SM Enabled */
	ConnectionState         ConnState;              /* TypeC State */
	AW_U8                  TypeCSubState;           /* TypeC Substate */
	AW_U16                 DetachThreshold;         /* TypeC detach level */
	AW_U8                  loopCounter;             /* Count and prevent attach/detach loop */
	AW_BOOL                C2ACable;                /* Possible C-to-A type cable detected */
	/* All Policy Engine variables */
	PolicyState_t           PolicyState;             /* PE State */
	AW_U8                  PolicySubIndex;           /* PE Substate */
	SopType                 PolicyMsgTxSop;          /* Tx to SOP? */
	AW_BOOL                USBPDActive;              /* PE Active */
	AW_BOOL                USBPDEnabled;             /* PE Enabled */
	AW_BOOL                PolicyIsSource;           /* Policy Src/Snk? */
	AW_BOOL                PolicyIsDFP;              /* Policy DFP/UFP? */
	AW_BOOL                PolicyHasContract;        /* Have contract */
	AW_BOOL                isContractValid;          /* PD Contract Valid */
	AW_BOOL                IsHardReset;              /* HR is occurring */
	AW_BOOL                IsPRSwap;                 /* PR is occurring */
	AW_BOOL                IsVCONNSource;            /* VConn state */
	AW_BOOL                USBPDTxFlag;              /* Have msg to Tx */
	AW_U8                  CollisionCounter;         /* Collisions for PE */
	AW_U8                  HardResetCounter;
	AW_U8                  CapsCounter;                /* Startup caps tx'd */

	sopMainHeader_t         src_cap_header;
	sopMainHeader_t         snk_cap_header;
	doDataObject_t          src_caps[7];
	doDataObject_t          snk_caps[7];

	AW_U8                  PdRevSop;              /* Partner spec rev */
	AW_U8                  PdRevCable;                 /* Cable spec rev */
	AW_BOOL                PpsEnabled;                 /* True if PPS mode */
	AW_BOOL                src_support_pd;             /* source support pd */
	AW_BOOL                src_support_pps;            /* source support pps */
	AW_BOOL                src_support_fixed;          /* source support fixed */

	AW_BOOL                WaitingOnHR;                /* HR shortcut */
	AW_BOOL                WaitSent;                   /* Waiting on PR Swap */

	AW_BOOL                WaitInSReady;               /* Snk/SrcRdy Delay */

	/* All Protocol State Machine Variables */
	ProtocolState_t         ProtocolState;              /* Protocol State */
	sopMainHeader_t         PolicyRxHeader;             /* Header Rx'ing */
	sopMainHeader_t         PolicyTxHeader;             /* Header Tx'ing */
	sopMainHeader_t         PDTransmitHeader;           /* Header to Tx */
	sopMainHeader_t         SrcCapsHeaderReceived;      /* Recent caps */
	sopMainHeader_t         SnkCapsHeaderReceived;      /* Recent caps */
	doDataObject_t          PolicyRxDataObj[7];         /* Rx'ing objects */
	doDataObject_t          PolicyTxDataObj[7];         /* Tx'ing objects */
	doDataObject_t          PDTransmitObjects[7];       /* Objects to Tx */
	doDataObject_t          SrcCapsReceived[7];         /* Recent caps header */
	doDataObject_t          SnkCapsReceived[7];         /* Recent caps header */
	doDataObject_t          USBPDContract;              /* Current PD request */
	doDataObject_t          SinkRequest;                /* Sink request  */
	doDataObject_t          PartnerCaps;                /* PP's Sink Caps */
	doPDO_t                 vendor_info_source[7];      /* Caps def'd by VI */
	doPDO_t                 vendor_info_sink[7];        /* Caps def'd by VI */
	SopType                 ProtocolMsgRxSop;           /* SOP of msg Rx'd */
	SopType                 ProtocolMsgTxSop;           /* SOP of msg Tx'd */
	PDTxStatus_t            PDTxStatus;                 /* Protocol Tx state */
	AW_BOOL                ProtocolMsgRx;              /* Msg Rx'd Flag */
	AW_BOOL                ProtocolMsgRxPending;       /* Msg in FIFO */
	AW_U8                  ProtocolTxBytes;            /* Bytes to Tx */
	AW_U8                  ProtocolTxBuffer[AW_PROTOCOL_BUFFER_SIZE];
	AW_U8                  ProtocolRxBuffer[AW_PROTOCOL_BUFFER_SIZE];
	AW_U8                  MessageIDCounter[SOP_TYPE_NUM]; /* Local ID count */
	AW_U8                  MessageID[SOP_TYPE_NUM];    /* Local last msg ID */

	AW_BOOL                DoTxFlush;                  /* Collision -> Flush */

	/* Timer objects */
	struct TimerObj         PDDebounceTimer;            /* First debounce */
	struct TimerObj         CCDebounceTimer;            /* Second debounce */
	struct TimerObj         StateTimer;                 /* TypeC state timer */
	struct TimerObj         LoopCountTimer;             /* Loop delayed clear */
	struct TimerObj         PolicyStateTimer;           /* PE state timer */
	struct TimerObj         ProtocolTimer;              /* Prtcl state timer */
	struct TimerObj         SwapSourceStartTimer;       /* PR swap delay */
	struct TimerObj         PpsTimer;                   /* PPS timeout timer */
	struct TimerObj         VBusPollTimer;              /* VBus monitor timer */
	struct TimerObj         VdmTimer;                   /* VDM timer */

	struct TimerObj         *Timers[AW_NUM_TIMERS];

#ifdef AW_HAVE_EXT_MSG
	ExtSrcCapBlock_t       SrcCapExt;
	ExtMsgState_t          ExtTxOrRx;                  /* Tx' or Rx'ing  */
	ExtHeader_t            ExtTxHeader;
	ExtHeader_t            ExtRxHeader;
	AW_BOOL                ExtWaitTxRx;                /* Waiting to Tx/Rx */
	AW_U16                 ExtChunkOffset;             /* Next chunk offset */
	AW_U8                  ExtMsgBuffer[260];
	AW_U8                  ExtChunkNum;                /* Next chunk number */
#else
	ExtHeader_t             ExtHeader;                  /* For sending NS */
	AW_BOOL                WaitForNotSupported;        /* Wait for timer */
#endif

#ifdef AW_HAVE_VDM
	VdmDiscoveryState_t     AutoVdmState;
	VdmManager              vdmm;
	PolicyState_t           vdm_next_ps;
	PolicyState_t           originalPolicyState;
	SvidInfo                core_svid_info;
	SopType                 VdmMsgTxSop;
	doDataObject_t          vdm_msg_obj[7];
	AW_U32                 my_mode;
	AW_U32                 vdm_msg_length;
	AW_BOOL                sendingVdmData;
	AW_BOOL                VdmTimerStarted;
	AW_BOOL                vdm_timeout;
	AW_BOOL                vdm_expectingresponse;
	AW_BOOL                svid_enable;
	AW_BOOL                mode_enable;
	AW_BOOL                mode_entered;
	AW_BOOL                AutoModeEntryEnabled;
	AW_S32                 AutoModeEntryObjPos;
	AW_S16                 svid_discvry_idx;
	AW_BOOL                svid_discvry_done;
	AW_U16                 my_svid;
	AW_U16                 discoverIdCounter;
	AW_BOOL                cblPresent;
	CableResetState_t       cblRstState;
#endif /* AW_HAVE_VDM */

#ifdef AW_HAVE_DP
	DisplayPortData_t       DisplayPortData;
#endif /* AW_HAVE_DP */
};

/**
 * @brief Initializes the port structure and state machine behaviors
 *
 * Initializes the port structure with default values.
 * Also sets the i2c address of the device and enables the TypeC state machines.
 *
 * @param port Pointer to the port structure
 * @param i2c_address 8-bit value with bit zero (R/W bit) set to zero.
 * @return None
 */
void PortInit(Port_t *port);

/**
 * @brief Set the next Type-C state.
 *
 * Also clears substate and logs the new state value.
 *
 * @param port Pointer to the port structure
 * @param state Next Type-C state
 * @return None
 */
void SetTypeCState(Port_t *port, ConnectionState state);
/* @brief Set the next Policy Engine state.
 *
 * Also clears substate and logs the new state value.
 *
 * @param port Pointer to the port structure
 * @param state Next Type-C state
 * @return None
 */
void SetPEState(Port_t *port, PolicyState_t state);

/**
 * @brief Revert the ports current setting to the configured value.
 *
 * @param port Pointer to the port structure
 * @return None
 */
void SetConfiguredCurrent(Port_t *port);

/* Forward Declarations */
void SetPortDefaultConfiguration(Port_t *port);
void InitializeRegisters(Port_t *port);
void InitializeTypeCVariables(Port_t *port);
void InitializePDProtocolVariables(Port_t *port);
void InitializePDPolicyVariables(Port_t *port);
#endif /* _PORT_H_ */
