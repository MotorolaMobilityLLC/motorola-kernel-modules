/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-12-17
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef _AW_TYPEC_H_
#define _AW_TYPEC_H_
#include "Port.h"

/* Type C Timing Parameters */
#define tAMETimeout     (900 * TICK_SCALE_TO_MS) /* Alternate Mode Entry Time */
#define tCCDebounce     (120 * TICK_SCALE_TO_MS)
#define tPDDebounce     (15  * TICK_SCALE_TO_MS)
#define tTryTimeout     (100 * TICK_SCALE_TO_MS)
#define tDRPTryWait     (100 * TICK_SCALE_TO_MS)
#define tErrorRecovery  (30  * TICK_SCALE_TO_MS) /* Delay in Error Recov State */

#define tDeviceToggle   (3   * TICK_SCALE_TO_MS) /* CC pin measure toggle */
#define tTOG2           (30  * TICK_SCALE_TO_MS) /* DRP Toggle timing */
#define tIllegalCable   (500 * TICK_SCALE_TO_MS) /* Reconnect loop reset time */
#define tOrientedDebug  (100 * TICK_SCALE_TO_MS) /* DebugAcc Orient Delay */
#define tLoopReset      (100 * TICK_SCALE_TO_MS)
#define tAttachWaitPoll (20  * TICK_SCALE_TO_MS) /* Periodic poll in AW.Src */
/* Switch from Default to correct advertisement in AW.Src */
#define tAttachWaitAdv  (20  * TICK_SCALE_TO_MS)
#define VbusTimeout     (10  * TICK_SCALE_TO_MS)
#define VBUSDEBOUNCE    (50)
#define WAITTOGGLE      (30)

/* Attach-Detach loop count - Halt after N loops */
#define MAX_CABLE_LOOP  60

void StateMachineTypeC(Port_t *port);
void StateMachineDisabled(Port_t *port);
void StateMachineErrorRecovery(Port_t *port);
void StateMachineUnattached(Port_t *port);

#ifdef AW_HAVE_SNK
void StateMachineAttachWaitSink(Port_t *port);
void StateMachineAttachedSink(Port_t *port);
void StateMachineTryWaitSink(Port_t *port);
void StateMachineDebugAccessorySink(Port_t *port);
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_VBUS_ONLY
void StateMachineAttachedVbusOnly(Port_t *port);
#endif /* AW_HAVE_VBUS_ONLY */

#if (defined(AW_HAVE_DRP) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void StateMachineTrySink(Port_t *port);
#endif /* AW_HAVE_DRP || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_SRC
void StateMachineAttachWaitSource(Port_t *port);
void StateMachineTryWaitSource(Port_t *port);
#ifdef AW_HAVE_DRP
void StateMachineUnattachedSource(Port_t *port);    /* AW.Snk -> Unattached */
#endif /* AW_HAVE_DRP */
void StateMachineAttachedSource(Port_t *port);
void StateMachineTrySource(Port_t *port);
void StateMachineDebugAccessorySource(Port_t *port);
#endif /* AW_HAVE_SRC */

#ifdef AW_HAVE_ACCMODE
void StateMachineAttachWaitAccessory(Port_t *port);
void StateMachineAudioAccessory(Port_t *port);
void StateMachinePoweredAccessory(Port_t *port);
void StateMachineUnsupportedAccessory(Port_t *port);
void SetStateAudioAccessory(Port_t *port);
#endif /* AW_HAVE_ACCMODE */

void SetStateErrorRecovery(Port_t *port);
void SetStateUnattached(Port_t *port);

#ifdef AW_HAVE_SNK
void SetStateAttachWaitSink(Port_t *port);
void SetStateAttachedSink(Port_t *port);
void SetStateDebugAccessorySink(Port_t *port);
#ifdef AW_HAVE_DRP
void RoleSwapToAttachedSink(Port_t *port);
#endif /* AW_HAVE_DRP */
void SetStateTryWaitSink(Port_t *port);
#endif /* AW_HAVE_SNK */

#if (defined(AW_HAVE_DRP) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void SetStateTrySink(Port_t *port);
#endif /* AW_HAVE_DRP || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_SRC
void SetStateAttachWaitSource(Port_t *port);
void SetStateAttachedSource(Port_t *port);
void SetStateDebugAccessorySource(Port_t *port);
#ifdef AW_HAVE_DRP
void RoleSwapToAttachedSource(Port_t *port);
#endif /* AW_HAVE_DRP */
void SetStateTrySource(Port_t *port);
void SetStateTryWaitSource(Port_t *port);
#ifdef AW_HAVE_DRP
void SetStateUnattachedSource(Port_t *port);
#endif /* AW_HAVE_DRP */
#endif /* AW_HAVE_SRC */

#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
void SetStateAttachWaitAccessory(Port_t *port);
void SetStateUnsupportedAccessory(Port_t *port);
void SetStatePoweredAccessory(Port_t *port);
#endif /* (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)) */

void SetStateIllegalCable(Port_t *port);
void StateMachineIllegalCable(Port_t *port);

void updateSourceCurrent(Port_t *port);
void updateSourceMDACHigh(Port_t *port);
void updateSourceMDACLow(Port_t *port);

void ToggleMeasure(Port_t *port);

CCTermType DecodeCCTermination(Port_t *port);
#if defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
CCTermType DecodeCCTerminationSource(Port_t *port);
AW_BOOL IsCCPinRa(Port_t *port);
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */
#ifdef AW_HAVE_SNK
CCTermType DecodeCCTerminationSink(Port_t *port);
#endif /* AW_HAVE_SNK */

void UpdateSinkCurrent(Port_t *port, CCTermType term);
AW_BOOL VbusVSafe0V(Port_t *port);

#ifdef AW_HAVE_SNK
AW_BOOL VbusUnder5V(Port_t *port);
#endif /* AW_HAVE_SNK */

AW_BOOL isVSafe5V(Port_t *port);
AW_BOOL isVBUSOverVoltage(Port_t *port, AW_U16 vbus_mv);

void resetDebounceVariables(Port_t *port);
void setDebounceVariables(Port_t *port, CCTermType term);
void setDebounceVariables(Port_t *port, CCTermType term);
void debounceCC(Port_t *port);

#if defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
void setStateSource(Port_t *port, AW_BOOL vconn);
void DetectCCPinSource(Port_t *port);
void updateVCONNSource(Port_t *port);
void updateVCONNSource(Port_t *port);
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_SNK
void setStateSink(Port_t *port);
void DetectCCPinSink(Port_t *port);
void updateVCONNSource(Port_t *port);
void updateVCONNSink(Port_t *port);
#endif /* AW_HAVE_SNK */

void clearState(Port_t *port);

void UpdateCurrentAdvert(Port_t *port, USBTypeCCurrent Current);

void SetStateDisabled(Port_t *port);
#endif/* _AW_TYPEC_H_ */

