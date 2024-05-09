/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name     : dp.h
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *******************************************************************************/
#ifdef AW_HAVE_DP
#ifndef __DISPLAYPORT_DP_H__
#define __DISPLAYPORT_DP_H__

#include "../../PD_Types.h"
#include "../vdm_types.h"
#include "dp_types.h"
#include "../../Port.h"

void DP_Initialize(struct Port *port);

/*  Interface used by VDM/policy engines */
/*  Initiate status request - call to get status of port partner */
void DP_RequestPartnerStatus(struct Port *port);
/*  Initiate config request - call to configure port partner */
void DP_RequestPartnerConfig(struct Port *port, DisplayPortConfig_t in);

/** Logic to decide capability to accecpt from UFP_U */
AW_BOOL DP_EvaluateSinkCapability(struct Port *port, AW_U32 mode_in);

/*  Process special DisplayPort commands. */
/*  Returns AW_TRUE when the message isn't processed and AW_FALSE otherwise */
AW_BOOL DP_ProcessCommand(struct Port *port, AW_U32 *arr_in);

/*  Internal helper functions */
/*  Send status data to port partner */
void DP_SendPortStatus(struct Port *port, doDataObject_t svdmh_in);

/*  Reply to a config request (to port partner) */
void DP_SendPortConfig(struct Port *port, doDataObject_t svdmh_in, AW_BOOL success);
void DP_SendAttention(struct Port *port);

void DP_SetPortMode(struct Port *port, DisplayPortMode_t conn);

/*  Internal DP functionality to be customized per system */
void DP_ProcessPartnerAttention(struct Port *port, DisplayPortStatus_t stat);
void DP_ProcessConfigResponse(struct Port *port, AW_BOOL success);
AW_BOOL DP_ProcessConfigRequest(struct Port *port, DisplayPortConfig_t config);
void DP_UpdatePartnerStatus(struct Port *port, DisplayPortStatus_t status, AW_BOOL success);

#endif /* __DISPLAYPORT_DP_H__ */

#endif // AW_HAVE_DP
