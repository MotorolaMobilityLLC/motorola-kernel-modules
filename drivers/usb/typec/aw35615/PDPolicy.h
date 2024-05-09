/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author		: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#ifndef _PDPOLICY_H_
#define _PDPOLICY_H_
#include "aw_types.h"
#include "Port.h"

void USBPDEnable(Port_t *port, AW_BOOL DeviceUpdate, SourceOrSink TypeCDFP);
void USBPDDisable(Port_t *port, AW_BOOL DeviceUpdate);

void USBPDPolicyEngine(Port_t *port);
void PolicyErrorRecovery(Port_t *port);

#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void PolicySourceSendHardReset(Port_t *port);
void PolicySourceSoftReset(Port_t *port, SopType sop);
void PolicySourceSendSoftReset(Port_t *port);
void PolicySourceStartup(Port_t *port);
void PolicySourceDiscovery(Port_t *port);
void PolicySourceSendCaps(Port_t *port);
void PolicySourceDisabled(Port_t *port);
void PolicySourceTransitionDefault(Port_t *port);
void PolicySourceNegotiateCap(Port_t *port);
void PolicySourceTransitionSupply(Port_t *port);
void PolicySourceCapabilityResponse(Port_t *port);
void PolicySourceReady(Port_t *port);
void PolicySourceGiveSourceCap(Port_t *port);
void PolicySourceGetSourceCap(Port_t *port);
void PolicySourceGetSinkCap(Port_t *port);
void PolicySourceSendPing(Port_t *port);
void PolicySourceGotoMin(Port_t *port);
void PolicySourceGiveSinkCap(Port_t *port);
void PolicySourceSendDRSwap(Port_t *port);
void PolicySourceEvaluateDRSwap(Port_t *port);
void PolicySourceSendVCONNSwap(Port_t *port);
void PolicySourceEvaluateVCONNSwap(Port_t *port);
void PolicySourceSendPRSwap(Port_t *port);
void PolicySourceEvaluatePRSwap(Port_t *port);
void PolicySourceWaitNewCapabilities(Port_t *port);
void PolicySourceAlertReceived(Port_t *port);
void PolicyGetBatteryCap(Port_t *port);
void PolicyGetBatteryStatus(Port_t *port);
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_SNK
void PolicySinkSendHardReset(Port_t *port);
void PolicySinkSoftReset(Port_t *port);
void PolicySinkSendSoftReset(Port_t *port);
void PolicySinkTransitionDefault(Port_t *port);
void PolicySinkStartup(Port_t *port);
void PolicySinkDiscovery(Port_t *port);
void PolicySinkWaitCaps(Port_t *port);
void PolicySinkEvaluateCaps(Port_t *port);
void PolicySinkSelectCapability(Port_t *port);
void PolicySinkTransitionSink(Port_t *port);
void PolicySinkReady(Port_t *port);
void PolicySinkGiveSinkCap(Port_t *port);
void PolicyGiveSourceInfo(Port_t *port);
void PolicyGiveRevisonMessage(Port_t *port);
void PolicySinkGetSinkCap(Port_t *port);
void PolicySinkGiveSourceCap(Port_t *port);
void PolicySinkGetSourceCap(Port_t *port);
void PolicySinkSendDRSwap(Port_t *port);
void PolicySinkEvaluateDRSwap(Port_t *port);
void PolicySinkSendVCONNSwap(Port_t *port);
void PolicySinkEvaluateVCONNSwap(Port_t *port);
void PolicySinkSendPRSwap(Port_t *port);
void PolicySinkEvaluatePRSwap(Port_t *port);
void PolicySinkAlertReceived(Port_t *port);
#endif /* AW_HAVE_SNK */

void PolicyNotSupported(Port_t *port);
void PolicyInvalidState(Port_t *port);
void policyBISTReceiveMode(Port_t *port);
void policyBISTFrameReceived(Port_t *port);
void policyBISTCarrierMode2(Port_t *port);
void policyBISTTestData(Port_t *port);

#ifdef AW_HAVE_EXT_MSG
void PolicyGetCountryCodes(Port_t *port);
void PolicyGiveCountryCodes(Port_t *port);
void PolicyGiveCountryInfo(Port_t *port);
void PolicyGetPPSStatus(Port_t *port);
void PolicyGivePPSStatus(Port_t *port);
void PolicySourceCapExtended(struct Port *port);
#endif /* AW_HAVE_EXT_MSG */

void PolicySendGenericCommand(Port_t *port);
void PolicySendGenericData(Port_t *port);

void PolicySendHardReset(Port_t *port);

AW_U8 PolicySendCommand(Port_t *port, AW_U8 Command, PolicyState_t nextState,
			AW_U8 subIndex, SopType sop);

AW_U8 PolicySendData(Port_t *port, AW_U8 MessageType, void *data,
			AW_U32 len, PolicyState_t nextState,
			AW_U8 subIndex, SopType sop, AW_BOOL extMsg);

void UpdateCapabilitiesRx(Port_t *port, AW_BOOL IsSourceCaps);

void processDMTBIST(Port_t *port);

#ifdef AW_HAVE_VDM
/* Shim functions for VDM code */
void InitializeVdmManager(Port_t *port);
void convertAndProcessVdmMessage(Port_t *port, SopType sop);
void doVdmCommand(Port_t *port);
void doDiscoverIdentity(Port_t *port);
void doDiscoverSvids(Port_t *port);
void PolicyGiveVdm(Port_t *port);
void PolicyVdm(Port_t *port);
void autoVdmDiscovery(Port_t *port);
void PolicySendCableReset(Port_t *port);
void processCableResetState(Port_t *port);
#endif /* AW_HAVE_VDM */

SopType TokenToSopType(AW_U8 data);

#endif /* _PDPOLICY_H_ */
