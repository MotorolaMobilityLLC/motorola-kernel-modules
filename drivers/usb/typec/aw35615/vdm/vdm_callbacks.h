/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name     : vdm_callbacks.h
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *******************************************************************************/
#ifndef __DPM_EMULATION_H__
#define __DPM_EMULATION_H__

#include "vdm_types.h"
#include "../PD_Types.h"

#ifdef AW_HAVE_VDM

ModesInfo vdmRequestModesInfo(Port_t *port, AW_U16 svid);

Identity vdmRequestIdentityInfo(Port_t *port);
SvidInfo vdmRequestSvidInfo(Port_t *port);
AW_BOOL vdmModeEntryRequest(Port_t *port, AW_U16 svid, AW_U32 mode_index);
AW_BOOL vdmModeExitRequest(Port_t *port, AW_U16 svid, AW_U32 mode_index);
AW_BOOL vdmEnterModeResult(Port_t *port, AW_BOOL success, AW_U16 svid, AW_U32 mode_index);

void vdmExitModeResult(Port_t *port, AW_BOOL success, AW_U16 svid, AW_U32 mode_index);
void vdmInformIdentity(Port_t *port, AW_BOOL success, SopType sop, Identity id);
void vdmInformSvids(Port_t *port, AW_BOOL success, SopType sop, SvidInfo svid_info);
void vdmInformModes(Port_t *port, AW_BOOL success, SopType sop, ModesInfo modes_info);
void vdmInformAttention(Port_t *port, AW_U16 svid, AW_U8 mode_index);
void vdmInitDpm(Port_t *port);

#endif /* AW_HAVE_VDM */

#endif /* __DPM_EMULATION_H__ */

