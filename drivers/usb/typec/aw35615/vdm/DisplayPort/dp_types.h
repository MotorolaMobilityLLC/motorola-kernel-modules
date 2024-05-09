/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name     : dp_types.h
 * Author	  : awinic
 * Date	    : 2021-09-10
 * Description   : .C file function description
 * Version	 : 1.0
 * Function List :
 ******************************************************************************/
#ifdef AW_HAVE_DP

#ifndef __DISPLAYPORT_TYPES_H__
#define __DISPLAYPORT_TYPES_H__

#include "../../aw_types.h"

/* DisplayPort-specific commands */
typedef enum {
	DP_COMMAND_ATTENTION = 0x06,
	DP_COMMAND_STATUS = 0x10, /* Status Update */
	DP_COMMAND_CONFIG = 0x11 /* Configure */
} DisplayPortCommand_t;

typedef enum {
	DP_Plug = 0, /* DP present in plug */
	DP_Receptacle = 1, /* DP present is receptacle */
} DisplayPortReceptacle_t;

typedef union {
	AW_U32 word;
	AW_U8 byte[4];
	struct {
		AW_U32 UfpDCapable :1; /* 0 - reserved, 01b - UFP_D, 10b - DFP, 11b - Both */
		AW_U32 DfpDCapable :1;
		AW_U32 Transport :4;
		AW_U32 ReceptacleIndication:1; /* 1 = Receptacle, 0 = Plug */
		AW_U32 USB2p0NotUsed:1; /* 1 = USB r2.0 signaling not required, 0 = _may_ be required */
		AW_U32 DFP_DPinAssignments:8;
		AW_U32 UFP_DPinAssignments:8;
		AW_U32 Rsvd0:8;
	};
} DisplayPortCaps_t;

/* DisplayPort DFP_D/UFP_D Connected */
typedef enum {
	DP_MODE_DISABLED = 0, /* Neither DFP_D nor UFP_D is connected, or adaptor is disabled */
	DP_MODE_DFP_D = 1, /* DFP_D is connected */
	DP_MODE_UFP_D = 2, /* UFP_D is connected */
	DP_MODE_BOTH = DP_MODE_DFP_D | DP_MODE_UFP_D/* Both DFP_D and UFP_D are connected */
} DisplayPortConn_t;

typedef union {
	AW_U32 word;
	AW_U8 byte[4];
	struct {
		DisplayPortConn_t Connection:2;/* Connected-to information */
		AW_U32 PowerLow:1;/* Adaptor has detected low power and disabled DisplayPort support */
		AW_U32 Enabled:1;/* Adaptor DisplayPort functionality is enabled and operational */
		AW_U32 MultiFunctionPreferred:1;/* Multi-function preference */
		/* 0 = Maintain current configuration,*/
		/*1 = request switch to USB Configuration (if in DP Config) */
		AW_U32 UsbConfigRequest:1;
		/* 0 = Maintain current mode, 1 = Request exit from DisplayPort Mode (if in DP Mode) */
		AW_U32 ExitDpModeRequest:1;
		AW_U32 HpdState:1;/* 0 = HPD_Low, 1 = HPD_High */
		AW_U32 IrqHpd:1;/* 0 = No IRQ_HPD since last status message, 1 = IRQ_HPD */
		AW_U32 Rsvd0:23;
	};
} DisplayPortStatus_t;

/* Select configuration */
typedef enum {
	DP_CONF_USB = 0,/* Set configuration for USB */
	DP_CONF_DFP_D = 1,/* Set configuration for UFP_U as DFP_D */
	DP_CONF_UFP_D = 2,/* Set configuration for UFP_U as UFP_D */
	DP_CONF_RSVD = 3
} DisplayPortMode_t;

/* Signaling for Transport of DisplayPort Protocol */
typedef enum {
	DP_CONF_SIG_UNSPECIFIED = 0,/* Signaling unspecified (only for USB Configuration) */
	DP_CONF_SIG_DP_V1P3 = 1,/* Select DP v1.3 signaling rates and electrical settings */
	DP_CONF_SIG_GEN2 = 2/* Select Gen 2 signaling rates and electrical specifications */
} DpConfSignaling_t;

typedef enum {
	DP_PIN_ASSIGN_NS = 0, /* Not selected */
	DP_PIN_ASSIGN_C = 4,
	DP_PIN_ASSIGN_D = 8,
	DP_PIN_ASSIGN_E = 16,
} DisplayPortPinAssign_t;

typedef union {
	AW_U32 word;
	AW_U8 byte[4];
	struct {
	DisplayPortMode_t Mode:2;/* UFP_D, DFP_D */
	DpConfSignaling_t SignalConfig:4; /* Signaling configuration */
	AW_U32 Rsvd0:2;
	DisplayPortPinAssign_t PinAssign:8; /* Configure UFP_U with DFP_D Pin Assigment */
	AW_U32 Rsvd1:8;
};
} DisplayPortConfig_t;

typedef struct {
	AW_BOOL DpAutoModeEntryEnabled; /* Automatically enter mode if DP_SID found */
	AW_BOOL DpEnabled; /* DP is enabled */
	AW_BOOL DpConfigured; /* Currently acting as DP UFP_D or DFP_D */
	AW_BOOL DpCapMatched; /* Port's capability matches partner's capability */
	AW_U32  DpModeEntered; /* Mode index of display port mode entered */
	/*  DisplayPort Status/Config objects */
	DisplayPortCaps_t DpCap; /* Port DP capability */
	DisplayPortStatus_t DpStatus; /* Port DP status sent during Discv. Mode or attention */
	/* Port Partner Status/Config objects */
	DisplayPortCaps_t   DpPpCap;/* Port partner capability rx from Modes */
	/* Port partner configuration to select, valid when DpCapMatched == AW_TRUE */
	DisplayPortConfig_t DpPpConfig;
	DisplayPortStatus_t DpPpStatus; /* Previous status of port partner */
} DisplayPortData_t;

#endif  /* __DISPLAYPORT_TYPES_H__ */

#endif /* AW_HAVE_DP */
