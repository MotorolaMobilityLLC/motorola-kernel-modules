/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name     : vdm.h
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *******************************************************************************/
#ifndef __VDM_MANAGER_H__
#define __VDM_MANAGER_H__
#include "../vendor_info.h"
#include "fsc_vdm_defs.h"
#include "vdm_callbacks_defs.h"
#include "../PD_Types.h"
#include "../Port.h"

#ifdef AW_HAVE_VDM

#define SVID_DEFAULT SVID1_SOP
#define MODE_DEFAULT 0x0001
#define SVID_AUTO_ENTRY 0x1057
#define MODE_AUTO_ENTRY 0x0001

#define NUM_VDM_MODES 6
#define MAX_NUM_SVIDS_PER_SOP 30
#define MAX_SVIDS_PER_MESSAGE 12
#define MIN_DISC_ID_RESP_SIZE 3

/* Millisecond values ticked by 1ms timer. */
#define tVDMSenderResponse (27 * TICK_SCALE_TO_MS)
#define tVDMWaitModeEntry  (45 * TICK_SCALE_TO_MS)
#define tVDMWaitModeExit   (45 * TICK_SCALE_TO_MS)

/*
 * Initialization functions.
 */
AW_S32 initializeVdm(Port_t *port);

/*
 * Functions to go through PD VDM flow.
 */
/* Initiations from DPM */
AW_S32 requestDiscoverIdentity(Port_t *port, SopType sop);
AW_S32 requestDiscoverSvids(Port_t *port, SopType sop);
AW_S32 requestDiscoverModes(Port_t *port, SopType sop, AW_U16 svid);
AW_S32 requestSendAttention(Port_t *port, SopType sop, AW_U16 svid, AW_U8 mode);
AW_S32 requestEnterMode(Port_t *port, SopType sop, AW_U16 svid, AW_U32 mode_index);
AW_S32 requestExitMode(Port_t *port, SopType sop, AW_U16 svid, AW_U32 mode_index);
AW_S32 requestExitAllModes(void);

/* receiving end */
AW_S32 processVdmMessage(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processDiscoverIdentity(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processDiscoverSvids(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processDiscoverModes(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processEnterMode(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processExitMode(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processAttention(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);
AW_S32 processSvidSpecific(Port_t *port, SopType sop, AW_U32 *arr_in, AW_U32 length_in);

/* Private function */
AW_BOOL evalResponseToSopVdm(Port_t *port, doDataObject_t vdm_hdr);
AW_BOOL evalResponseToCblVdm(Port_t *port, doDataObject_t vdm_hdr);
void sendVdmMessageWithTimeout(Port_t *port, SopType sop, AW_U32 *arr, AW_U32 length, AW_S32 n_pe);
void vdmMessageTimeout(Port_t *port);
void startVdmTimer(Port_t *port, AW_S32 n_pe);
void sendVdmMessageFailed(Port_t *port);
void resetPolicyState(Port_t *port);

void sendVdmMessage(Port_t *port, SopType sop, AW_U32 *arr, AW_U32 length, PolicyState_t next_ps);

AW_BOOL evaluateModeEntry(Port_t *port, AW_U32 mode_in);

#endif /* AW_HAVE_VDM */
#endif /* __VDM_MANAGER_H__ */
