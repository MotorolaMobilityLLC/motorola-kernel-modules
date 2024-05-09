/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021,Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name	 : vdm_callbacks_defs.h
 * Author		: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __AW_VDM_CALLBACKS_DEFS_H__
#define __AW_VDM_CALLBACKS_DEFS_H__
/*
 * This file defines types for callbacks that the VDM block will use.
 * The intention is for the user to define functions that return data
 * that VDM messages require, ex whether or not to enter a mode.
 */
#ifdef AW_HAVE_VDM

#include "vdm_types.h"
#include "../PD_Types.h"
#include "../modules/dpm.h"

typedef Identity(*RequestIdentityInfo)(Port_t *port);
typedef SvidInfo(*RequestSvidInfo)(Port_t *port);
typedef ModesInfo(*RequestModesInfo)(Port_t *port, AW_U16);
typedef AW_BOOL(*ModeEntryRequest)(Port_t *port, AW_U16 svid, AW_U32 mode_index);
typedef AW_BOOL(*ModeExitRequest)(Port_t *port, AW_U16 svid, AW_U32 mode_index);
typedef AW_BOOL(*EnterModeResult)(Port_t *port, AW_BOOL success, AW_U16 svid, AW_U32 mode_index);
typedef void (*ExitModeResult)(Port_t *port, AW_BOOL success, AW_U16 svid, AW_U32 mode_index);
typedef void (*InformIdentity)(Port_t *port, AW_BOOL success, SopType sop, Identity id);
typedef void (*InformSvids)(Port_t *port, AW_BOOL success, SopType sop, SvidInfo svid_info);
typedef void (*InformModes)(Port_t *port, AW_BOOL success, SopType sop, ModesInfo modes_info);
typedef void (*InformAttention)(Port_t *port, AW_U16 svid, AW_U8 mode_index);

/*
 * VDM Manager object, so I can have multiple instances intercommunicating using the same functions!
 */
typedef struct {
	/* callbacks! */
	RequestIdentityInfo req_id_info;
	RequestSvidInfo req_svid_info;
	RequestModesInfo req_modes_info;
	ModeEntryRequest req_mode_entry;
	ModeExitRequest req_mode_exit;
	EnterModeResult enter_mode_result;
	ExitModeResult exit_mode_result;
	InformIdentity inform_id;
	InformSvids inform_svids;
	InformModes inform_modes;
	InformAttention inform_attention;
} VdmManager;

#endif /* AW_HAVE_VDM */

#endif /* __AW_VDM_CALLBACKS_DEFS_H__ */
