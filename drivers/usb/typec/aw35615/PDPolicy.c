// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author	: awinic
 * Date		  : 2021-09-09
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#include <linux/delay.h>
#include "platform_helpers.h"
#include "vendor_info.h"
#include "PD_Types.h"
#include "PDPolicy.h"
#include "PDProtocol.h"
#include "TypeC.h"
#include "aw35615_driver.h"

#ifdef AW_HAVE_VDM
#include "vdm/vdm_callbacks.h"
#include "vdm/vdm_callbacks_defs.h"
#include "vdm/vdm.h"
#include "vdm/vdm_types.h"
#include "vdm/bitfield_translators.h"
#include "vdm/DisplayPort/dp_types.h"
#include "vdm/DisplayPort/dp.h"
#endif /* AW_HAVE_VDM */

/* USB PD Enable / Disable Routines */
void USBPDEnable(Port_t *port, AW_BOOL DeviceUpdate, SourceOrSink TypeCDFP)
{
	port->PortConfig.reqPRSwapAsSrc = Requests_PR_Swap_As_Src;
	port->PortConfig.reqPRSwapAsSnk = Requests_PR_Swap_As_Snk;
	port->PortConfig.reqDRSwapToDfpAsSink = Attempt_DR_Swap_to_Dfp_As_Sink;
	port->PortConfig.reqDRSwapToUfpAsSrc = Attempt_DR_Swap_to_Ufp_As_Src;
	port->PortConfig.reqVconnSwapToOnAsSink = Attempt_Vconn_Swap_to_On_As_Sink;
	port->PortConfig.reqVconnSwapToOffAsSrc = Attempt_Vconn_Swap_to_Off_As_Src;

	port->IsHardReset = AW_FALSE;
	port->IsPRSwap = AW_FALSE;
	port->HardResetCounter = 0;

	if (port->USBPDEnabled == AW_TRUE) {
		ResetProtocolLayer(port, AW_TRUE);

		/* Check CC pin to monitor */
		if (port->CCPin == CC1) {
			port->Registers.Switches.TXCC1 = 1;
			port->Registers.Switches.MEAS_CC1 = 1;

			port->Registers.Switches.TXCC2 = 0;
			port->Registers.Switches.MEAS_CC2 = 0;
		} else if (port->CCPin == CC2) {
			port->Registers.Switches.TXCC2 = 1;
			port->Registers.Switches.MEAS_CC2 = 1;

			port->Registers.Switches.TXCC1 = 0;
			port->Registers.Switches.MEAS_CC1 = 0;
		}

		if (port->CCPin != CCNone) {
			port->USBPDActive = AW_TRUE;

			port->PolicyIsSource = (TypeCDFP == SOURCE) ? AW_TRUE : AW_FALSE;
			port->PolicyIsDFP = (TypeCDFP == SOURCE) ? AW_TRUE : AW_FALSE;
			port->IsVCONNSource = (TypeCDFP == SOURCE) ? AW_TRUE : AW_FALSE;

			if (port->PolicyIsSource) {
				SetPEState(port, peSourceStartup);
				port->Registers.Switches.POWERROLE = 1;
				port->Registers.Switches.DATAROLE = 1;
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				port->Registers.Control.ENSOP2 = SOP_PP_Capable;
			} else {
				SetPEState(port, peSinkStartup);
				TimerDisable(&port->PolicyStateTimer);
				port->Registers.Switches.POWERROLE = 0;
				port->Registers.Switches.DATAROLE = 0;

				port->Registers.Control.ENSOP1 = 0;
				port->Registers.Control.ENSOP1DP = 0;
				port->Registers.Control.ENSOP2 = 0;
				port->Registers.Control.ENSOP2DB = 0;
			}

			/* port->Registers.Switches.AUTO_CRC = 0; */
			port->Registers.Control.AUTO_PRE = 0;
			port->Registers.Control.AUTO_RETRY = 1;

			DeviceWrite(port, regControl0, 4, &port->Registers.Control.byte[0]);

			if (DeviceUpdate)
				DeviceWrite(port, regSwitches1, 1,
						&port->Registers.Switches.byte[1]);

			TimerDisable(&port->SwapSourceStartTimer);
		}

		port->PEIdle = AW_FALSE;

#ifdef AW_HAVE_VDM
		if (Attempts_Discov_SOP)
			port->AutoVdmState = AUTO_VDM_INIT;
		else
			port->AutoVdmState = AUTO_VDM_DONE;
		port->cblPresent = AW_FALSE;
		port->cblRstState = CBL_RST_DISABLED;
#endif /* AW_HAVE_VDM */
	}
}

void USBPDDisable(Port_t *port, AW_BOOL DeviceUpdate)
{
	if (port->Registers.Control.BIST_MODE2 != 0) {
		port->Registers.Control.BIST_MODE2 = 0;
		DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
	}

	port->IsHardReset = AW_FALSE;
	port->IsPRSwap = AW_FALSE;

	port->PEIdle = AW_TRUE;

	port->PdRevSop = port->PortConfig.PdRevPreferred;
	port->PdRevCable = port->PortConfig.PdRevPreferred;
	port->USBPDActive = AW_FALSE;
	port->src_support_pd = AW_FALSE;
	port->src_support_pps = AW_FALSE;
	port->usb_commcapable = AW_FALSE;
	port->ProtocolState = PRLDisabled;
	SetPEState(port, peDisabled);
	port->PolicyIsSource = AW_FALSE;
	port->PolicyHasContract = AW_FALSE;
	port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

	notify_observers(BIST_DISABLED, port->I2cAddr, 0);
	notify_observers(PD_NO_CONTRACT, port->I2cAddr, 0);

	if (DeviceUpdate) {
		/* Disable the BMC transmitter (both CC1 & CC2) */
		port->Registers.Switches.TXCC1 = 0;
		port->Registers.Switches.TXCC2 = 0;

		/* Turn off Auto CRC */
		/* port->Registers.Switches.AUTO_CRC = 0; */
		DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);
	}

	/* Disable the internal oscillator for USB PD */
	port->Registers.Power.POWER = 0x7;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);
	ProtocolFlushRxFIFO(port);
	ProtocolFlushTxFIFO(port);

	/* Mask PD Interrupts */
	port->Registers.Mask.M_COLLISION = 1;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->Registers.MaskAdv.M_RETRYFAIL = 1;
	port->Registers.MaskAdv.M_TXSENT = 1;
	port->Registers.MaskAdv.M_HARDRST = 1;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	port->Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

	/* Force VBUS check */
	port->Registers.Status.I_COMP_CHNG = 1;
}

/* USB PD Policy Engine Routines */
void USBPDPolicyEngine(Port_t *port)
{
	if (port->PolicyState != port->old_policy_state) {
		AW_LOG("PolicyState %d\n", port->PolicyState);
		port->old_policy_state = port->PolicyState;
	}
	switch (port->PolicyState) {
	case peDisabled:
		break;
	case peErrorRecovery:
		PolicyErrorRecovery(port);
		break;
#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
	/* Source States */
	case peSourceSendHardReset:
		PolicySourceSendHardReset(port);
		break;
	case peSourceSendSoftReset:
		PolicySourceSendSoftReset(port);
		break;
	case peSourceSoftReset:
		PolicySourceSoftReset(port, port->ProtocolMsgRxSop);
		break;
	case peSourceStartup:
		PolicySourceStartup(port);
		break;
	case peSourceDiscovery:
		PolicySourceDiscovery(port);
		break;
	case peSourceSendCaps:
		PolicySourceSendCaps(port);
		break;
	case peSourceDisabled:
		PolicySourceDisabled(port);
		break;
	case peSourceTransitionDefault:
		PolicySourceTransitionDefault(port);
		break;
	case peSourceNegotiateCap:
		PolicySourceNegotiateCap(port);
		break;
	case peSourceCapabilityResponse:
		PolicySourceCapabilityResponse(port);
		break;
	case peSourceWaitNewCapabilities:
		PolicySourceWaitNewCapabilities(port);
		break;
	case peSourceTransitionSupply:
		PolicySourceTransitionSupply(port);
		break;
	case peSourceReady:
		PolicySourceReady(port);
		break;
	case peSourceGiveSourceCaps:
		PolicySourceGiveSourceCap(port);
		break;
	case peSourceGetSinkCaps:
		PolicySourceGetSinkCap(port);
		break;
	case peSourceSendPing:
		PolicySourceSendPing(port);
		break;
	case peSourceGotoMin:
		PolicySourceGotoMin(port);
		break;
	case peSourceGiveSinkCaps:
		PolicySourceGiveSinkCap(port);
		break;
	case peSourceGetSourceCaps:
		PolicySourceGetSourceCap(port);
		break;
	case peSourceSendDRSwap:
		PolicySourceSendDRSwap(port);
		break;
	case peSourceEvaluateDRSwap:
		PolicySourceEvaluateDRSwap(port);
		break;
	case peSourceSendVCONNSwap:
		PolicySourceSendVCONNSwap(port);
		break;
	case peSourceEvaluateVCONNSwap:
		PolicySourceEvaluateVCONNSwap(port);
		break;
	case peSourceSendPRSwap:
		PolicySourceSendPRSwap(port);
		break;
	case peSourceEvaluatePRSwap:
		PolicySourceEvaluatePRSwap(port);
		break;
	case peSourceAlertReceived:
		PolicySourceAlertReceived(port);
		break;
#endif /* AW_HAVE_SRC */
#ifdef AW_HAVE_SNK
	/* Sink States */
	case peSinkStartup:
		PolicySinkStartup(port);
		break;
	case peSinkSendHardReset:
		PolicySinkSendHardReset(port);
		break;
	case peSinkSoftReset:
		PolicySinkSoftReset(port);
		break;
	case peSinkSendSoftReset:
		PolicySinkSendSoftReset(port);
		break;
	case peSinkTransitionDefault:
		PolicySinkTransitionDefault(port);
		break;
	case peSinkDiscovery:
		PolicySinkDiscovery(port);
		break;
	case peSinkWaitCaps:
		PolicySinkWaitCaps(port);
		break;
	case peSinkEvaluateCaps:
		PolicySinkEvaluateCaps(port);
		break;
	case peSinkSelectCapability:
		PolicySinkSelectCapability(port);
		break;
	case peSinkTransitionSink:
		PolicySinkTransitionSink(port);
		break;
	case peSinkReady:
		PolicySinkReady(port);
		break;
	case peSinkGiveSinkCap:
		PolicySinkGiveSinkCap(port);
		break;
	case peSinkGetSourceCap:
		PolicySinkGetSourceCap(port);
		break;
	case peSinkGetSinkCap:
		PolicySinkGetSinkCap(port);
		break;
	case peSinkGiveSourceCap:
		PolicySinkGiveSourceCap(port);
		break;
	case peSinkSendDRSwap:
		PolicySinkSendDRSwap(port);
		break;
	case peSinkEvaluateDRSwap:
		PolicySinkEvaluateDRSwap(port);
		break;
	case peSinkSendVCONNSwap:
		PolicySinkSendVCONNSwap(port);
		break;
	case peSinkEvaluateVCONNSwap:
		PolicySinkEvaluateVCONNSwap(port);
		break;
	case peSinkSendPRSwap:
		PolicySinkSendPRSwap(port);
		break;
	case peSinkEvaluatePRSwap:
		PolicySinkEvaluatePRSwap(port);
		break;
	case peSinkAlertReceived:
		PolicySinkAlertReceived(port);
		break;
#endif /* AW_HAVE_SNK */
	case peNotSupported:
		PolicyNotSupported(port);
		break;
#ifdef AW_HAVE_VDM
	case peSendCableReset:
		PolicySendCableReset(port);
		break;
#endif /* AW_HAVE_VDM */
#ifdef AW_HAVE_VDM
	case peGiveVdm:
		PolicyGiveVdm(port);
		break;
#endif /* AW_HAVE_VDM */
#ifdef AW_HAVE_EXT_MSG
	case peGiveCountryCodes:
		PolicyGiveCountryCodes(port);
		break;
	case peGetCountryCodes:
		PolicyGetCountryCodes(port);
		break;
	case peGiveCountryInfo:
		PolicyGiveCountryInfo(port);
		break;
	case peGivePPSStatus:
		PolicyGivePPSStatus(port);
		break;
	case peGetPPSStatus:
		PolicyGetPPSStatus(port);
		break;
	case peSourceGiveSourceCapExt:
		PolicySourceCapExtended(port);
		break;
#endif /* AW_HAVE_EXT_MSG */
	case PE_BIST_Receive_Mode:
		policyBISTReceiveMode(port);
		break;
	case PE_BIST_Frame_Received:
		policyBISTFrameReceived(port);
		break;
	case PE_BIST_Carrier_Mode_2:
		policyBISTCarrierMode2(port);
		break;
	case PE_BIST_Test_Data:
		policyBISTTestData(port);
		break;
	case peSendGenericCommand:
		PolicySendGenericCommand(port);
		break;
	case peSendGenericData:
		PolicySendGenericData(port);
		break;
	case peGiveSourceInfo:
		PolicyGiveSourceInfo(port);
		break;
	case peGiveRevisonMessage:
		PolicyGiveRevisonMessage(port);
		break;
	case peGetBatteryCap:
		PolicyGetBatteryCap(port);
		break;
	case peGetBatteryStatus:
		PolicyGetBatteryStatus(port);
		break;
	default:
#ifdef AW_HAVE_VDM
		if ((port->PolicyState >= FIRST_VDM_STATE) && (port->PolicyState <= LAST_VDM_STATE))
			PolicyVdm(port); /* Valid VDM state */
		else
#endif /* AW_HAVE_VDM */
			PolicyInvalidState(port);/* Invalid state, reset */
		break;
	}
}

void PolicyErrorRecovery(Port_t *port)
{
	SetStateErrorRecovery(port);
}

#ifdef AW_HAVE_VDM
void PolicySendCableReset(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		port->Registers.Control.AUTO_RETRY = 0;
		DeviceWrite(port, regControl3, 1, &port->Registers.Control.byte[3]);
		ProtocolSendCableReset(port);
		TimerStart(&port->PolicyStateTimer, tWaitCableReset);
		port->PolicySubIndex++;
		break;
	default:
		if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port,
				port->PolicyIsSource == AW_TRUE ? peSourceReady : peSinkReady);
			port->Registers.Control.AUTO_RETRY = 1;
			DeviceWrite(port, regControl3, 1, &port->Registers.Control.byte[3]);
			port->cblRstState = CBL_RST_DISABLED;
		}
	}
}
#endif /* AW_HAVE_VDM */

#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void PolicySourceSendHardReset(Port_t *port)
{
	PolicySendHardReset(port);
}

void PolicySourceSoftReset(Port_t *port, SopType sop)
{
	if (PolicySendCommand(port, CMTAccept, peSourceSendCaps, 0, sop) == STAT_SUCCESS)
#ifdef AW_HAVE_VDM
		port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
}

void PolicySourceSendSoftReset(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTSoftReset, peSourceSendSoftReset, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
			port->WaitingOnHR = AW_TRUE;
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects == 0) &&
				(port->PolicyRxHeader.MessageType == CMTAccept)) {
				#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
				#endif /* AW_HAVE_VDM */
				SetPEState(port, peSourceDiscovery);
			} else {
				SetPEState(port, peSourceSendHardReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer))
			SetPEState(port, peSourceSendHardReset);
		else
			port->PEIdle = AW_TRUE;
		break;
	}
}

void PolicySourceStartup(Port_t *port)
{
	/* AW_LOG("enter PolicySubIndex =%d\n", port->PolicySubIndex); */
#ifdef AW_HAVE_VDM
	AW_S32 i;
#endif /* AW_HAVE_VDM */

	switch (port->PolicySubIndex) {
	case 0:
		/* Set masks for PD */
#ifdef AW_GSCE_FIX
		port->Registers.Mask.M_CRC_CHK = 0; /* Added for GSCE workaround */
#endif /* AW_GSCE_FIX */
		port->Registers.Mask.M_COLLISION = 0;
		DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
		port->Registers.MaskAdv.M_RETRYFAIL = 0;
		port->Registers.MaskAdv.M_HARDSENT = 0;
		port->Registers.MaskAdv.M_TXSENT = 0;
		port->Registers.MaskAdv.M_HARDRST = 0;
		DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
		port->Registers.MaskAdv.M_GCRCSENT = 0;
		DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

		/* Disable auto-flush RxFIFO */
		if (port->Registers.Control.BIST_TMODE == 1) {
			port->Registers.Control.BIST_TMODE = 0;
			DeviceWrite(port, regControl3, 1, &port->Registers.Control.byte[3]);
		}

		port->USBPDContract.object = 0;
		port->PartnerCaps.object = 0;
		port->IsPRSwap = AW_FALSE;
		port->WaitSent = AW_TRUE;
		port->IsHardReset = AW_FALSE;
		port->PolicyIsSource = AW_TRUE;
		port->PpsEnabled = AW_FALSE;

		ResetProtocolLayer(port, AW_FALSE);
		port->Registers.Switches.POWERROLE = port->PolicyIsSource;
		/* port->Registers.Slice.byte = 0x60; */
		/* DeviceWrite(port, regSlice, 1, &port->Registers.Slice.byte); */
		/* port->Registers.Switches.AUTO_CRC = 1; */
		DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

		port->Registers.Power.POWER = 0xF;
		DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

		port->CapsCounter = 0;
		port->CollisionCounter = 0;
		port->PolicySubIndex++;
		break;
	case 1:
		/* Wait until we reach vSafe5V and delay if coming from PR Swap */
		if ((isVBUSOverVoltage(port, VBUS_MV_VSAFE5V_L) ||
			(port->ConnState == PoweredAccessory)) &&
			(TimerExpired(&port->SwapSourceStartTimer) ||
			 TimerDisabled(&port->SwapSourceStartTimer))) {
			/* Delay once VBus is present for potential switch delay. */
			TimerStart(&port->PolicyStateTimer, tVBusSwitchDelay);

			port->PolicySubIndex++;
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			TimerDisable(&port->SwapSourceStartTimer);
			msleep(150);
			SetPEState(port, peSourceSendCaps);
#ifdef AW_HAVE_VDM
			port->discoverIdCounter = 0;
			if (Attempts_DiscvId_SOP_P_First) {
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				/*port->Registers.Control.ENSOP2 = SOP_PP_Capable; */
				DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
				if (port->PdRevSop == USBPDSPECREV3p0 && port->IsVCONNSource) {
					requestDiscoverIdentity(port, SOP_TYPE_SOP1);
					port->discoverIdCounter++;
				} else if (port->PdRevSop == USBPDSPECREV2p0 && port->PolicyIsDFP) {
					requestDiscoverIdentity(port, SOP_TYPE_SOP1);
					port->discoverIdCounter++;
				}
			}

			port->mode_entered = AW_FALSE;
			port->core_svid_info.num_svids = 0;
			for (i = 0; i < MAX_NUM_SVIDS; i++)
				port->core_svid_info.svids[i] = 0;
			port->AutoModeEntryObjPos = -1;
			port->svid_discvry_done = AW_FALSE;
			port->svid_discvry_idx  = -1;
#endif /* AW_HAVE_VDM */

#ifdef AW_HAVE_DP
			DP_Initialize(port);
#endif /* AW_HAVE_DP */
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		port->PolicySubIndex = 0;
		break;
	}
}

void PolicySourceDiscovery(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (TimerDisabled(&port->PolicyStateTimer) == AW_TRUE) {
			/* Start timer only when inactive. When waiting for timer send */
			/* sop' discovery messages */
			TimerStart(&port->PolicyStateTimer, tTypeCSendSourceCap);
#ifdef AW_HAVE_VDM
			if (Attempts_DiscvId_SOP_P_First && port->discoverIdCounter *
					(DPM_Retries(port, SOP_TYPE_SOP1) + 1) <
					nDiscoverIdentityCount) {
				port->discoverIdCounter++;
				requestDiscoverIdentity(port, SOP_TYPE_SOP1);
			}
#endif
		}
		port->PolicySubIndex++;
		break;
	default:
		if ((port->HardResetCounter > nHardResetCount) &&
			(port->PolicyHasContract == AW_TRUE)) {
			TimerDisable(&port->PolicyStateTimer);
			/* If we previously had a contract in place, go to ErrorRecovery */
			SetPEState(port, peErrorRecovery);
		} else if ((port->HardResetCounter > nHardResetCount) &&
				 (port->PolicyHasContract == AW_FALSE)) {
			TimerDisable(&port->PolicyStateTimer);
			/* Otherwise, disable and wait for detach */
			SetPEState(port, peSourceDisabled);
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			if (port->CapsCounter > nCapsCount)
				SetPEState(port, peSourceDisabled);
			else
				SetPEState(port, peSourceSendCaps);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySourceSendCaps(Port_t *port)
{
	if ((DPM_GetSourceCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSourceCap(port->dpm, port) == NULL))
		return;

	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendData(port,
				DMTSourceCapabilities,
				DPM_GetSourceCap(port->dpm, port),
				DPM_GetSourceCapHeader(port->dpm, port)->
				NumDataObjects*4,
				peSourceSendCaps, 1,
				SOP_TYPE_SOP, AW_FALSE) == STAT_SUCCESS) {
			port->HardResetCounter = 0;
			port->CapsCounter = 0;
			/* tSenderResponse(24ms - 30ms) The timer has several ms deviation*/
			TimerStart(&port->PolicyStateTimer, 27);
			port->WaitingOnHR = AW_TRUE;
		}
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects == 1) &&
				(port->PolicyRxHeader.MessageType == DMTRequest)) {
				SetPEState(port, peSourceNegotiateCap);

				/* Set the revision to the lowest of the two port partner */
				DPM_SetSpecRev(port, SOP_TYPE_SOP,
						(SpecRev)port->PolicyRxHeader.SpecRevision);
			} else {
				/* Otherwise it was a message that we didn't expect */
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			port->ProtocolMsgRx = AW_FALSE;
			SetPEState(port, peSourceSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySourceDisabled(Port_t *port)
{
	/* Clear the USB PD contract (output power to 5V default) */
	port->USBPDContract.object = 0;

	/* Wait for a hard reset or detach... */
	if (port->loopCounter == 0) {
		port->PEIdle = AW_TRUE;

		if (port->Registers.Power.POWER != 0x7) {
			port->Registers.Power.POWER = 0x7;
			DeviceWrite(port, regPower, 1,
					&port->Registers.Power.byte);
		}
	}
}

void PolicySourceTransitionDefault(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (TimerExpired(&port->PolicyStateTimer)) {
			port->PolicyHasContract = AW_FALSE;
			port->PdRevSop = port->PortConfig.PdRevPreferred;
			port->PdRevCable = port->PortConfig.PdRevPreferred;
			port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

			notify_observers(BIST_DISABLED, port->I2cAddr, 0);
			notify_observers(PD_NO_CONTRACT, port->I2cAddr, 0);

			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 1:
		platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
		/* vbus discharge */
		port->Registers.Control5.VBUS_DIS_SEL = 1;
		DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);


		if (!port->PolicyIsDFP) {
			port->PolicyIsDFP = AW_TRUE;
			port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

			port->Registers.Control.ENSOP1 = SOP_P_Capable;
			port->Registers.Control.ENSOP2 = SOP_PP_Capable;
			DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
		}

		if (port->IsVCONNSource) {
			/* Turn off VConn */
			port->Registers.Switches.VCONN_CC1 = 0;
			port->Registers.Switches.VCONN_CC2 = 0;
			DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
		}

		ProtocolFlushTxFIFO(port);
		ProtocolFlushRxFIFO(port);
		ResetProtocolLayer(port, AW_TRUE);

		TimerStart(&port->PolicyStateTimer, tPSHardResetMax + tSafe0V + tSrcRecover);

		port->PolicySubIndex++;
		break;
	case 2:
		if (VbusVSafe0V(port)) {
			/* vbus discharge */
			port->Registers.Control5.VBUS_DIS_SEL = 0;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
			TimerStart(&port->PolicyStateTimer, tSrcRecover);
			port->PolicySubIndex++;
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			if (port->PolicyHasContract) {
				SetPEState(port, peErrorRecovery);
			} else {
				port->PolicySubIndex = 4;
				TimerDisable(&port->PolicyStateTimer);
			}
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		if (TimerExpired(&port->PolicyStateTimer))
			port->PolicySubIndex++;
		else
			port->PEIdle = AW_TRUE;
		break;
	default:
		platform_set_pps_voltage(port->PortID, SET_VOUT_5000MV);

		/* Turn on VConn */
		if (Type_C_Sources_VCONN) {
			if (port->CCPin == CC1)
				port->Registers.Switches.VCONN_CC2 = 1;
			else
				port->Registers.Switches.VCONN_CC1 = 1;

			DeviceWrite(port, regSwitches0, 1,
					&port->Registers.Switches.byte[0]);
		}
		port->IsVCONNSource = AW_TRUE;

		TimerDisable(&port->SwapSourceStartTimer);

		SetPEState(port, peSourceStartup);
		break;
	}
}

void PolicySourceNegotiateCap(Port_t *port)
{
	AW_BOOL reqAccept = AW_FALSE;
	AW_U8 objPosition;
	AW_U32 minvoltage, maxvoltage;

	if ((DPM_GetSourceCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSourceCap(port->dpm, port) == NULL))
		return;

	objPosition = port->PolicyRxDataObj[0].FVRDO.ObjectPosition - 1;

	if (objPosition < DPM_GetSourceCapHeader(port->dpm, port)->NumDataObjects) {
		/* Enable PPS if pps request and capability request can be matched */
		if (DPM_GetSourceCap(port->dpm, port)[objPosition].
				PDO.SupplyType == pdoTypeAugmented) {
			/* Convert from 100mv (pdo) to 20mV (req) to compare */
			minvoltage = DPM_GetSourceCap(port->dpm, port)[objPosition].
				PPSAPDO.MinVoltage * 5;
			maxvoltage = DPM_GetSourceCap(port->dpm, port)[objPosition].
				PPSAPDO.MaxVoltage * 5;

			reqAccept =
				((port->PolicyRxDataObj[0].PPSRDO.OpCurrent <=
				DPM_GetSourceCap(port->dpm, port)[objPosition].
				PPSAPDO.MaxCurrent) &&
				(port->PolicyRxDataObj[0].PPSRDO.Voltage >= minvoltage) &&
				(port->PolicyRxDataObj[0].PPSRDO.Voltage <= maxvoltage));
		} else if (port->PolicyRxDataObj[0].FVRDO.OpCurrent <=
			DPM_GetSourceCap(port->dpm, port)[objPosition].
				FPDOSupply.MaxCurrent) {
			reqAccept = AW_TRUE;
		}
	}

	if (reqAccept)
		SetPEState(port, peSourceTransitionSupply);
	else
		SetPEState(port, peSourceCapabilityResponse);

	port->PEIdle = AW_FALSE;
}

void PolicySourceTransitionSupply(Port_t *port)
{
	AW_BOOL newIsPPS = AW_FALSE;

	if (DPM_GetSourceCap(port->dpm, port) == NULL)
		return;

	switch (port->PolicySubIndex) {
	case 0:
		PolicySendCommand(port, CMTAccept, peSourceTransitionSupply, 1, SOP_TYPE_SOP);
		break;
	case 1:
		port->USBPDContract.object = port->PolicyRxDataObj[0].object;
		newIsPPS = (DPM_GetSourceCap(port->dpm, port)
				[port->USBPDContract.FVRDO.ObjectPosition - 1].PDO.SupplyType ==
				pdoTypeAugmented) ? AW_TRUE : AW_FALSE;

		if (newIsPPS && port->PpsEnabled) {
			/* Skip the SrcTransition period for PPS requests */
			/* Note: In a system with more than one PPS PDOs, this will need
			 * to include a check to see if the request is from within the same
			 * APDO (no SrcTransition delay) or a different
			 * APDO (with SrcTransition delay).
			 */
			port->PolicySubIndex += 2;
		} else {
			TimerStart(&port->PolicyStateTimer, tSrcTransition);
			port->PolicySubIndex++;
		}

		port->PpsEnabled = newIsPPS;

		break;
	case 2:
		if (TimerExpired(&port->PolicyStateTimer))
			port->PolicySubIndex++;
		else
			port->PEIdle = AW_TRUE;
		break;
	case 3:
		port->PolicyHasContract = AW_TRUE;
		DPM_TransitionSource(port->dpm, port, port->USBPDContract.FVRDO.ObjectPosition - 1);

		TimerStart(&port->PolicyStateTimer, tSourceRiseTimeout);
		port->PolicySubIndex++;

		if (port->PpsEnabled)
			TimerStart(&port->PpsTimer, tPPSTimeout);
		else
			TimerDisable(&port->PpsTimer);
		break;
	case 4:
		if (TimerExpired(&port->PolicyStateTimer))
			port->PolicySubIndex++;

		else if (DPM_IsSourceCapEnabled(port->dpm, port,
						port->USBPDContract.FVRDO.ObjectPosition - 1))
			port->PolicySubIndex++;
		else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		if (PolicySendCommand(port, CMTPS_RDY, peSourceReady, 0,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			/* Set to 1.5A (SinkTxNG) */
			if (port->SourceCurrent != utcc1p5A) {
				UpdateCurrentAdvert(port, utcc1p5A);
				updateSourceMDACHigh(port);
			}

		/*notify_observers(PD_NEW_CONTRACT, port->I2cAddr, &port->USBPDContract);*/
		}
		break;
	}
}

void PolicySourceCapabilityResponse(Port_t *port)
{
	if (port->PolicyHasContract) {
		if (port->isContractValid)
			PolicySendCommand(port, CMTReject, peSourceReady, 0, SOP_TYPE_SOP);
		else
			PolicySendCommand(port, CMTReject, peSourceSendHardReset, 0, SOP_TYPE_SOP);
	} else {
		PolicySendCommand(port, CMTReject, peSourceWaitNewCapabilities, 0, SOP_TYPE_SOP);
	}
}

void PolicySourceReady(Port_t *port)
{
	if (port->ProtocolMsgRx) {
		/* Message received from partner */
		port->ProtocolMsgRx = AW_FALSE;
		if (port->PolicyRxHeader.NumDataObjects == 0) {
			switch (port->PolicyRxHeader.MessageType) {
			case CMTGetSourceCap:
				SetPEState(port, peSourceSendCaps);
				break;
			case CMTGetSinkCap:
				SetPEState(port, peSourceGiveSinkCaps);
				break;
			case CMTDR_Swap:
				SetPEState(port, peSourceEvaluateDRSwap);
				break;
			case CMTPR_Swap:
				SetPEState(port, peSourceEvaluatePRSwap);
				break;
			case CMTVCONN_Swap:
				SetPEState(port, peSourceEvaluateVCONNSwap);
				break;
			case CMTSoftReset:
				SetPEState(port, peSourceSoftReset);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case CMTGetCountryCodes:
				SetPEState(port, peGiveCountryCodes);
				break;
#endif /* 0 */
			case CMTGetPPSStatus:
				SetPEState(port, peGivePPSStatus);
				break;
			case CMTReject:
			case CMTNotSupported:
				/* Rx'd Reject/NS are ignored - notify DPM if needed */
				break;
			/* Unexpected messages */
			case CMTAccept:
			case CMTWait:
				SetPEState(port, peSourceSendSoftReset);
				break;
			case CMTGetSourceCapExt:
				SetPEState(port, peSourceGiveSourceCapExt);
				break;
			case CMTGetSourceInfo:
				SetPEState(port, peGiveSourceInfo);
				break;
			case CMTGetRevisonMessage:
				SetPEState(port, peGiveRevisonMessage);
				break;
			default:
				SetPEState(port, peNotSupported);
				break;
			}
		} else if (port->PolicyRxHeader.Extended == 1) {
			switch (port->PolicyRxHeader.MessageType) {
			case EXTGetBatteryCap:
				SetPEState(port, peGetBatteryCap);
				break;
			case EXTGetBatteryStatus:
				SetPEState(port, peGetBatteryStatus);
				break;
			default:
#ifndef AW_HAVE_EXT_MSG
				port->ExtHeader.byte[0] = port->PolicyRxDataObj[0].byte[0];
				port->ExtHeader.byte[1] = port->PolicyRxDataObj[0].byte[1];
				if (port->ExtHeader.DataSize > EXT_MAX_MSG_LEGACY_LEN) {
					/* Request takes more than one chunk */
					/* which is not supported */
					port->WaitForNotSupported = AW_TRUE;
				}
#endif
				SetPEState(port, peNotSupported);
				break;
			}
		} else {
			switch (port->PolicyRxHeader.MessageType) {
			case DMTSourceCapabilities:
			case DMTSinkCapabilities:
				break;
			case DMTRequest:
				SetPEState(port, peSourceNegotiateCap);
				break;
			case DMTAlert:
				SetPEState(port, peSourceAlertReceived);
				break;
#ifdef AW_HAVE_VDM
			case DMTVenderDefined:
				convertAndProcessVdmMessage(port, port->ProtocolMsgRxSop);
				break;
#endif /* AW_HAVE_VDM */
			case DMTBIST:
				processDMTBIST(port);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case DMTGetCountryInfo:
				SetPEState(port, peGiveCountryInfo);
				break;
#endif /* 0 */
			default:
				SetPEState(port, peNotSupported);
				break;
			}
		}
	} else if (port->USBPDTxFlag) {
		/* Has the device policy manager requested us to send a message? */
		if (port->PDTransmitHeader.NumDataObjects == 0) {
			switch (port->PDTransmitHeader.MessageType) {
			case CMTGetSinkCap:
				SetPEState(port, peSourceGetSinkCaps);
				break;
			case CMTGetSourceCap:
				SetPEState(port, peSourceGetSourceCaps);
				break;
			case CMTPing:
				SetPEState(port, peSourceSendPing);
				break;
			case CMTGotoMin:
				SetPEState(port, peSourceGotoMin);
				break;
#ifdef AW_HAVE_DRP
			case CMTPR_Swap:
				SetPEState(port, peSourceSendPRSwap);
				break;
#endif /* AW_HAVE_DRP */
			case CMTDR_Swap:
				SetPEState(port, peSourceSendDRSwap);
				break;
			case CMTVCONN_Swap:
				SetPEState(port, peSourceSendVCONNSwap);
				break;
			case CMTSoftReset:
				SetPEState(port, peSourceSendSoftReset);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case CMTGetCountryCodes:
				SetPEState(port, peGetCountryCodes);
				break;
#endif /* 0 */
			default:
				break;
			}
		} else {
			switch (port->PDTransmitHeader.MessageType) {
			case DMTSourceCapabilities:
				SetPEState(port, peSourceSendCaps);
				break;
			case DMTVenderDefined:
#ifdef AW_HAVE_VDM
				doVdmCommand(port);
#endif /* AW_HAVE_VDM */
				break;
			default:
				break;
			}
		}
		port->USBPDTxFlag = AW_FALSE;
	} else if (port->PpsEnabled && TimerExpired(&port->PpsTimer)) {
		SetPEState(port, peSourceSendHardReset);
	} else if (port->PartnerCaps.object == 0) {
		if (port->WaitInSReady == AW_TRUE) {
			if (TimerExpired(&port->PolicyStateTimer)) {
				TimerDisable(&port->PolicyStateTimer);
				SetPEState(port, peSourceGetSinkCaps);
			}
		} else {
			TimerStart(&port->PolicyStateTimer, 20 * TICK_SCALE_TO_MS);
			port->WaitInSReady = AW_TRUE;
		}
	} else if ((port->PortConfig.PortType == USBTypeC_DRP) &&
			 (port->PortConfig.reqPRSwapAsSrc) &&
			 (port->PartnerCaps.FPDOSink.DualRolePower == 1)) {
		SetPEState(port, peSourceSendPRSwap);
	} else if (port->PortConfig.reqDRSwapToUfpAsSrc == AW_TRUE &&
			 port->PolicyIsDFP == AW_TRUE &&
			 DR_Swap_To_UFP_Supported) {
		/* Set it to false so there is only one try */
		port->PortConfig.reqDRSwapToUfpAsSrc = AW_FALSE;
		SetPEState(port, peSourceSendDRSwap);
	} else if (port->PortConfig.reqVconnSwapToOffAsSrc == AW_TRUE &&
			 port->IsVCONNSource == AW_TRUE &&
			 VCONN_Swap_To_Off_Supported) {
		/* Set it to false so there is only one try */
		port->PortConfig.reqVconnSwapToOffAsSrc = AW_FALSE;
		SetPEState(port, peSourceSendVCONNSwap);
	}
#ifdef AW_HAVE_VDM
	else if (port->PolicyIsDFP && (port->AutoVdmState != AUTO_VDM_DONE)) {
		autoVdmDiscovery(port);
	} else if (port->cblRstState > CBL_RST_DISABLED) {
		processCableResetState(port);
	}
#endif /* AW_HAVE_VDM */
	else {
		port->PEIdle = AW_TRUE;
		if (port->SourceCurrent != utcc3p0A) {
			/* Set to 3.0A (SinkTXOK) */
			UpdateCurrentAdvert(port, utcc3p0A);
			updateSourceMDACHigh(port);
		}
	}
}

void PolicySourceGiveSourceCap(Port_t *port)
{
	if (DPM_GetSourceCapHeader(port->dpm, port) == NULL)
		return;

	PolicySendData(port, DMTSourceCapabilities,
				DPM_GetSourceCap(port->dpm, port),
				DPM_GetSourceCapHeader(port->dpm, port)->
						NumDataObjects * sizeof(doDataObject_t),
						peSourceReady, 0, SOP_TYPE_SOP, AW_FALSE);
}

void PolicySourceGetSourceCap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTGetSourceCap, peSourceGetSourceCaps, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects > 0) &&
				(port->PolicyRxHeader.MessageType == DMTSourceCapabilities)) {
				UpdateCapabilitiesRx(port, AW_TRUE);
				SetPEState(port, peSourceReady);
			} else if ((port->PolicyRxHeader.NumDataObjects == 0) &&
					 ((port->PolicyRxHeader.MessageType == CMTReject) ||
					  (port->PolicyRxHeader.MessageType == CMTNotSupported))) {
				SetPEState(port, peSinkReady);
			} else {
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySourceGetSinkCap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTGetSinkCap, peSourceGetSinkCaps, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		}
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects > 0) &&
				(port->PolicyRxHeader.MessageType == DMTSinkCapabilities)) {
				UpdateCapabilitiesRx(port, AW_FALSE);
				SetPEState(port, peSourceReady);
			} else {
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySourceGiveSinkCap(Port_t *port)
{
	if ((DPM_GetSinkCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSinkCap(port->dpm, port) == NULL))
		return;

#ifdef AW_HAVE_DRP
	if (port->PortConfig.PortType == USBTypeC_DRP) {
		PolicySendData(port,
				DMTSinkCapabilities,
				DPM_GetSinkCap(port->dpm, port),
				DPM_GetSinkCapHeader(port->dpm, port)->
				NumDataObjects * sizeof(doDataObject_t),
				peSourceReady, 0,
				SOP_TYPE_SOP, AW_FALSE);
	} else
#endif /* AW_HAVE_DRP */
	{
		SetPEState(port, peNotSupported);
	}
}

void PolicySourceSendPing(Port_t *port)
{
	PolicySendCommand(port, CMTPing, peSourceReady, 0, SOP_TYPE_SOP);
}

void PolicySourceGotoMin(Port_t *port)
{
	if (port->ProtocolMsgRx) {
		port->ProtocolMsgRx = AW_FALSE;
		if (port->PolicyRxHeader.NumDataObjects == 0) {
			switch (port->PolicyRxHeader.MessageType) {
			case CMTSoftReset:
				SetPEState(port, peSourceSoftReset);
				break;
			default:
				/* If we receive any other command (including Reject & Wait),
				 * just go back to the ready state without changing
				 */
				break;
			}
		}
	} else {
		switch (port->PolicySubIndex) {
		case 0:
			PolicySendCommand(port, CMTGotoMin, peSourceGotoMin, 1, SOP_TYPE_SOP);
			break;
		case 1:
			TimerStart(&port->PolicyStateTimer, tSrcTransition);
			port->PolicySubIndex++;
			break;
		case 2:
			if (TimerExpired(&port->PolicyStateTimer))
				port->PolicySubIndex++;
			else
				port->PEIdle = AW_TRUE;
			break;
		case 3:
			/* Adjust the power supply if necessary... */
			port->PolicySubIndex++;
			break;
		case 4:
			/* Validate the output is ready prior to sending PS_RDY */
			port->PolicySubIndex++;
			break;
		default:
			PolicySendCommand(port, CMTPS_RDY, peSourceReady, 0, SOP_TYPE_SOP);
			break;
		}
	}
}

void PolicySourceSendDRSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTDR_Swap, peSourceSendDRSwap,
					1, SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->PolicyIsDFP =
						(port->PolicyIsDFP == AW_TRUE) ? AW_FALSE : AW_TRUE;
					port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
					DeviceWrite(port, regSwitches1, 1,
						&port->Registers.Switches.byte[1]);

					notify_observers(DATA_ROLE, port->I2cAddr, NULL);
					if (port->PdRevSop == USBPDSPECREV2p0) {
						/* In PD2.0, DFP controls SOP* coms */
						if (port->PolicyIsDFP == AW_TRUE) {
							port->Registers.Control.ENSOP1 =
									SOP_P_Capable;
							port->Registers.Control.ENSOP2 =
									SOP_PP_Capable;
#ifdef AW_HAVE_VDM
							port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
						} else {
							port->Registers.Control.ENSOP1 = 0;
							port->Registers.Control.ENSOP2 = 0;
						}
						DeviceWrite(port, regControl1, 1,
								&port->Registers.Control.byte[1]);
					}
					/* Fall through */
					break;
				case CMTReject:
					SetPEState(port, peSourceReady);
					break;
				default:
					SetPEState(port, peSourceSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySourceEvaluateDRSwap(Port_t *port)
{
#ifdef AW_HAVE_VDM
	/* If were are in modal operation, send a hard reset */
	if (port->mode_entered == AW_TRUE) {
		SetPEState(port, peSourceSendHardReset);
		return;
	}
#endif /* AW_HAVE_VDM */

	if (!DR_Swap_To_UFP_Supported) {
		PolicySendCommand(port, CMTReject, peSourceReady, 0, port->ProtocolMsgRxSop);
	} else {
		if (PolicySendCommand(port, CMTAccept, peSourceReady, 0,
					port->ProtocolMsgRxSop) == STAT_SUCCESS) {
			port->PolicyIsDFP = (port->PolicyIsDFP == AW_TRUE) ? AW_FALSE : AW_TRUE;
			port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

			notify_observers(DATA_ROLE, port->I2cAddr, NULL);

			if (port->PdRevSop == USBPDSPECREV2p0) {
				/* In PD2.0, DFP controls SOP* coms */
				if (port->PolicyIsDFP == AW_TRUE) {
					port->Registers.Control.ENSOP1 = SOP_P_Capable;
					port->Registers.Control.ENSOP2 = SOP_PP_Capable;
#ifdef AW_HAVE_VDM
					port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
				} else {
					port->Registers.Control.ENSOP1 = 0;
					port->Registers.Control.ENSOP2 = 0;
				}
				DeviceWrite(port, regControl1, 1,
							&port->Registers.Control.byte[1]);
			}
		}
	}
}

void PolicySourceSendVCONNSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTVCONN_Swap, peSourceSendVCONNSwap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		}
		break;
	case 1:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->PolicySubIndex++;
					break;
				case CMTWait:
				case CMTReject:
					SetPEState(port, peSourceReady);
					break;
				default:
					SetPEState(port, peSourceSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		if (port->IsVCONNSource) {
			TimerStart(&port->PolicyStateTimer, tVCONNSourceOn);
			port->PolicySubIndex++;
		} else {
			if (Type_C_Sources_VCONN) {
				if (port->CCPin == CC1)
					port->Registers.Switches.VCONN_CC2 = 1;
				else
					port->Registers.Switches.VCONN_CC1 = 1;

				DeviceWrite(port, regSwitches0, 1,
						&port->Registers.Switches.byte[0]);
			}

			port->IsVCONNSource = AW_TRUE;

			if (port->PdRevSop == USBPDSPECREV3p0) {
				/* In PD3.0, VConn Source controls SOP* coms */
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				port->Registers.Control.ENSOP2 = SOP_PP_Capable;
				DeviceWrite(port, regControl1, 1,
						&port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
			}

			TimerStart(&port->PolicyStateTimer, tVCONNTransition);
			port->PolicySubIndex = 4;
		}
		break;
	case 3:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					/* Turn off our VConn */
					port->Registers.Switches.VCONN_CC1 = 0;
					port->Registers.Switches.VCONN_CC2 = 0;
					DeviceWrite(port, regSwitches0, 1,
							&port->Registers.Switches.byte[0]);

					port->IsVCONNSource = AW_FALSE;

					if (port->PdRevSop == USBPDSPECREV3p0) {
						/* In PD3.0, VConn Source controls SOP* coms */
						port->Registers.Control.ENSOP1 = 0;
						port->Registers.Control.ENSOP2 = 0;
						DeviceWrite(port, regControl1, 1,
							&port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
						port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
					}
					SetPEState(port, peSourceReady);
					break;
				default:
					/* For all other commands received, simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		if (TimerExpired(&port->PolicyStateTimer) == AW_TRUE) {
			port->PolicySubIndex++;
			/* Fall through to default. Immediately send PS_RDY */
		} else {
			port->PEIdle = AW_TRUE;
			break;
		}
	default:
		PolicySendCommand(port, CMTPS_RDY, peSourceReady, 0, SOP_TYPE_SOP);
		break;
	}
}

void PolicySourceEvaluateVCONNSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		/* Accept/Reject */
		if ((port->IsVCONNSource && VCONN_Swap_To_Off_Supported) ||
			(!port->IsVCONNSource && VCONN_Swap_To_On_Supported)) {
			PolicySendCommand(port, CMTAccept, peSourceEvaluateVCONNSwap, 1,
					port->ProtocolMsgRxSop);
		} else {
			PolicySendCommand(port, CMTReject, peSourceReady, 0,
						port->ProtocolMsgRxSop);
		}
		break;
	case 1:
		/* Swap to On */
		if (port->IsVCONNSource) {
			TimerStart(&port->PolicyStateTimer, tVCONNSourceOn);
			port->PolicySubIndex++;
		} else {
			if (Type_C_Sources_VCONN) {
				if (port->CCPin == CC1)
					port->Registers.Switches.VCONN_CC2 = 1;
				else
					port->Registers.Switches.VCONN_CC1 = 1;

				DeviceWrite(port, regSwitches0, 1,
						&port->Registers.Switches.byte[0]);
			}

			port->IsVCONNSource = AW_TRUE;

			if (port->PdRevSop == USBPDSPECREV3p0) {
				/* In PD3.0, VConn Source controls SOP* coms */
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				port->Registers.Control.ENSOP2 = SOP_PP_Capable;
				DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
			}

			TimerStart(&port->PolicyStateTimer, tVCONNTransition);
			port->PolicySubIndex = 3;
		}
		break;
	case 2:
		/* Swap to Off */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					/* Disable the VCONN source */
					port->Registers.Switches.VCONN_CC1 = 0;
					port->Registers.Switches.VCONN_CC2 = 0;
					DeviceWrite(port, regSwitches0, 1,
							&port->Registers.Switches.byte[0]);

					port->IsVCONNSource = AW_FALSE;

					if (port->PdRevSop == USBPDSPECREV3p0) {
						/* In PD3.0, VConn Source controls SOP* coms */
						port->Registers.Control.ENSOP1 = 0;
						port->Registers.Control.ENSOP2 = 0;
						DeviceWrite(port, regControl1, 1,
							&port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
						port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
					}

					SetPEState(port, peSourceReady);
					break;
				default:
					/* For all other commands received, simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		/* Done - PS_RDY */
		if (TimerExpired(&port->PolicyStateTimer) == AW_TRUE) {
			port->PolicySubIndex++;
			/* Fall through. Immediately send PS_RDY after timer expires. */
		} else {
			port->PEIdle = AW_TRUE;
			break;
		}
	default:
		PolicySendCommand(port, CMTPS_RDY, peSourceReady, 0, port->ProtocolMsgRxSop);
		break;
	}
}

void PolicySourceSendPRSwap(Port_t *port)
{
#ifdef AW_HAVE_DRP
	AW_U8 Status;
	/* Disable the config flag if we got here during automatic request */
	port->PortConfig.reqPRSwapAsSrc = AW_FALSE;
	switch (port->PolicySubIndex) {
	case 0:
		/* Send Request */
		if (PolicySendCommand(port, CMTPR_Swap, peSourceSendPRSwap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	case 1:
		/* Wait for accept */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->IsPRSwap = AW_TRUE;
					port->PolicyHasContract = AW_FALSE;
					port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;
					RoleSwapToAttachedSink(port);

					TimerStart(&port->PolicyStateTimer, tSrcTransition);
					port->PolicySubIndex++;
					break;
				case CMTWait:
				case CMTReject:
					SetPEState(port, peSourceReady);
					port->IsPRSwap = AW_FALSE;
					break;
				default:
					SetPEState(port, peSourceSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSourceSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceReady);
			port->IsPRSwap = AW_FALSE;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		/* Turn off VBus */
		if (TimerExpired(&port->PolicyStateTimer)) {
			platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
			notify_observers(POWER_ROLE, port->I2cAddr, NULL);

			TimerStart(&port->PolicyStateTimer, tPSHardResetMax + tSafe0V);
			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		/* Swap to Sink */
		if (VbusVSafe0V(port)) {
			port->Registers.Control4.EN_PAR_CFG = 1;
			DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);
			port->Registers.Control5.VBUS_DIS_SEL = 1;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);

			port->PolicyIsSource = AW_FALSE;
			port->Registers.Switches.POWERROLE = port->PolicyIsSource;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);
			port->PolicySubIndex++;
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peErrorRecovery);
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		/* Signal to partner */
		port->Registers.Control4.EN_PAR_CFG = 1;
		DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);
		port->Registers.Control5.VBUS_DIS_SEL = 0;
		DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);

		port->Registers.Switches.PU_EN1 = 0;
		port->Registers.Switches.PDWN1 = 1;
		port->Registers.Switches.PU_EN2 = 0;
		port->Registers.Switches.PDWN2 = 1;
		DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

		Status = PolicySendCommand(port, CMTPS_RDY, peSourceSendPRSwap, 5, SOP_TYPE_SOP);
		if (Status == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tPSSourceOn);
		else if (Status == STAT_ERROR)
			SetPEState(port, peErrorRecovery);
		break;
	case 5:
		/* Wait for PS_Rdy */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					port->PolicySubIndex++;
					TimerStart(&port->PolicyStateTimer, tGoodCRCDelay);
					break;
				default:
					/* For all other commands received, simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peErrorRecovery);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		/* Move on to Sink Startup */
		if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkStartup);
			TimerDisable(&port->PolicyStateTimer);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
#endif /* AW_HAVE_DRP */
}

void PolicySourceEvaluatePRSwap(Port_t *port)
{
#ifdef AW_HAVE_DRP
	AW_U8 Status;

	switch (port->PolicySubIndex) {
	case 0:
		/* Send either the Accept or Reject command */
		if (!Accepts_PR_Swap_As_Src ||
			port->PortConfig.PortType != USBTypeC_DRP ||
			DPM_GetSourceCap(port->dpm, port)[0].
				FPDOSupply.DualRolePower == AW_FALSE) {
			PolicySendCommand(port, CMTReject, peSourceReady, 0,
					port->ProtocolMsgRxSop);
		} else if (port->WaitSent == AW_FALSE) {
			if (PolicySendCommand(port, CMTWait, peSourceGetSourceCaps, 0,
						port->ProtocolMsgRxSop) == STAT_SUCCESS)
				port->WaitSent = AW_TRUE;
		}
//		else if(port->PartnerCaps.FPDOSupply.DualRolePower == AW_FALSE ||
//			/* Determine Accept/Reject based on DRP bit in current PDO */
//				((DPM_GetSourceCap(port->dpm, port)[0].FPDOSupply.
//					ExternallyPowered == AW_TRUE) &&
//			/* Must also reject if we are ext powered and partner is not */
//				 (port->PartnerCaps.FPDOSupply.SupplyType == pdoTypeFixed) &&
//				 (port->PartnerCaps.FPDOSupply.ExternallyPowered == AW_FALSE))) {
//			if (PolicySendCommand(port, CMTReject, peSourceReady, 0,
//						port->ProtocolMsgRxSop) == STAT_SUCCESS)
//				port->WaitSent = AW_FALSE;
//		}
		else {
			if (PolicySendCommand(port, CMTAccept, peSourceEvaluatePRSwap, 1,
						port->ProtocolMsgRxSop) == STAT_SUCCESS) {
				port->IsPRSwap = AW_TRUE;
				port->WaitSent = AW_FALSE;
				port->PolicyHasContract = AW_FALSE;
				port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

			// notify_observers(PD_NO_CONTRACT, port->I2cAddr, 0);

				RoleSwapToAttachedSink(port);
				TimerStart(&port->PolicyStateTimer, tSrcTransition);
			}
		}
		break;
	case 1:
		/* Disable our VBus */
		if (TimerExpired(&port->PolicyStateTimer)) {
			platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
			notify_observers(POWER_ROLE, port->I2cAddr, NULL);
			TimerStart(&port->PolicyStateTimer, tPSHardResetMax + tSafe0V);
			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		/* Wait on discharge */
		if (VbusVSafe0V(port)) {
			//AW_LOG("enable discharge\n");
			port->Registers.Control4.EN_PAR_CFG = 1;
			DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);
			port->Registers.Control5.VBUS_DIS_SEL = 1;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
			TimerStart(&port->PolicyStateTimer, 5);
			port->PolicyIsSource = AW_FALSE;
			port->Registers.Switches.POWERROLE = port->PolicyIsSource;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);
			port->PolicySubIndex++;
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peErrorRecovery);
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		/* Wait on transition time */
		if (TimerExpired(&port->PolicyStateTimer)) {
			//AW_LOG("dis discharge\n");
			port->Registers.Control4.EN_PAR_CFG = 1;
			DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);
			port->Registers.Control5.VBUS_DIS_SEL = 0;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		/* Signal to partner */
		port->Registers.Switches.PU_EN1 = 0;
		port->Registers.Switches.PDWN1 = 1;
		port->Registers.Switches.PU_EN2 = 0;
		port->Registers.Switches.PDWN2 = 1;

		DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

		Status =
		PolicySendCommand(port, CMTPS_RDY, peSourceEvaluatePRSwap, 5, SOP_TYPE_SOP);
		if (Status == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tPSSourceOn);
		else if (Status == STAT_ERROR)
			SetPEState(port, peErrorRecovery);
		break;
	case 5:
		/* Wait to receive a PS_RDY message from the new DFP */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					port->PolicySubIndex++;
					port->IsPRSwap = AW_FALSE;
					TimerStart(&port->PolicyStateTimer, tGoodCRCDelay);
					break;
				default:
					/* For all the other commands received, */
					/* simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			port->IsPRSwap = AW_FALSE;
			SetPEState(port, peErrorRecovery);
			port->PEIdle = AW_FALSE;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		/* Move on to sink startup */
		if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkStartup);
			TimerDisable(&port->PolicyStateTimer);

			/* Disable auto PR swap flag to prevent swapping back */
			port->PortConfig.reqPRSwapAsSnk = AW_FALSE;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
#else
	PolicySendCommand(port, CMTReject, peSourceReady, 0, port->ProtocolMsgRxSop);
#endif /* AW_HAVE_DRP */
}

void PolicySourceWaitNewCapabilities(Port_t *port)
{
	if (port->loopCounter == 0) {
		port->PEIdle = AW_TRUE;

		port->Registers.Mask.M_COMP_CHNG = 0;
		DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
		port->Registers.MaskAdv.M_HARDRST = 0;
		DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
		port->Registers.MaskAdv.M_GCRCSENT = 0;
		DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);
	}

	switch (port->PolicySubIndex) {
	case 0:
		/* Wait for Policy Manager to change source capabilities */
		break;
	default:
		/* Transition to peSourceSendCapabilities */
		SetPEState(port, peSourceSendCaps);
		break;
	}
}
#endif /* AW_HAVE_SRC */


void PolicySourceAlertReceived(Port_t *port)
{
	if (port->PolicyRxDataObj[0].ADO.Battery ||
		port->PolicyRxDataObj[0].ADO.OTP ||
		port->PolicyRxDataObj[0].ADO.OVP ||
		port->PolicyRxDataObj[0].ADO.OpCondition ||
		port->PolicyRxDataObj[0].ADO.Input) {
		/* Send Get_Status */
		notify_observers(ALERT_EVENT, port->I2cAddr,
				&port->PolicyRxDataObj[0].object);
	}

	SetPEState(port, peSourceReady);
}

void PolicyNotSupported(Port_t *port)
{
	if (port->PdRevSop == USBPDSPECREV2p0) {
		PolicySendCommand(port, CMTReject,
			port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
			port->ProtocolMsgRxSop);
	} else {
		switch (port->PolicySubIndex) {
#ifndef AW_HAVE_EXT_MSG
		case 0:
			if (port->WaitForNotSupported == AW_TRUE) {
				TimerStart(&port->PolicyStateTimer, tChunkingNotSupported);
				port->PolicySubIndex++;
				port->WaitForNotSupported = AW_FALSE;
			} else {
				port->PolicySubIndex = 2;
			}
			break;
		case 1:
			if (TimerExpired(&port->PolicyStateTimer)) {
				TimerDisable(&port->PolicyStateTimer);
				port->PolicySubIndex++;
				/* Fall throught to default */
			} else {
				break;
			}
		case 2:
#endif
		default:
			PolicySendCommand(port, CMTNotSupported,
					port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
					port->ProtocolMsgRxSop);
			break;
		}
	}
}

#ifdef AW_HAVE_SNK
void PolicySinkSendHardReset(Port_t *port)
{
	PolicySendHardReset(port);
}

void PolicySinkSoftReset(Port_t *port)
{
	if (PolicySendCommand(port, CMTAccept, peSinkWaitCaps, 0, SOP_TYPE_SOP) == STAT_SUCCESS) {
		TimerStart(&port->PolicyStateTimer, tSinkWaitCap);
#ifdef AW_HAVE_VDM
		port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
	}
}

void PolicySinkSendSoftReset(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTSoftReset, peSinkSendSoftReset, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
			port->WaitingOnHR = AW_TRUE;
		}
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects == 0) &&
				(port->PolicyRxHeader.MessageType == CMTAccept)) {
#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
				SetPEState(port, peSinkWaitCaps);
				TimerStart(&port->PolicyStateTimer, tSinkWaitCap);
			} else {
				SetPEState(port, peSinkSendHardReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySinkTransitionDefault(Port_t *port)
{
	//AW_LOG("enter PolicySubIndex = %d\n", port->PolicySubIndex);
	switch (port->PolicySubIndex) {
	case 0:
		port->Registers.Power.POWER = 0x7;
		DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

		/* Set up VBus measure interrupt to watch for vSafe0V */
		port->Registers.Measure.MEAS_VBUS = 1;
		port->Registers.Measure.MDAC = VBUS_MDAC_0P84V;
		DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

		port->IsHardReset = AW_TRUE;
		port->PolicyHasContract = AW_FALSE;
		port->PdRevSop = port->PortConfig.PdRevPreferred;
		port->PdRevCable = port->PortConfig.PdRevPreferred;
		port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

		notify_observers(BIST_DISABLED, port->I2cAddr, 0);

		notify_observers(PD_NO_CONTRACT, port->I2cAddr, 0);

		TimerStart(&port->PolicyStateTimer, tPSHardResetMax + tSafe0V);

		if (port->PolicyIsDFP) {
			port->PolicyIsDFP = AW_FALSE;
			port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

			port->Registers.Control.ENSOP1 = 0;
			port->Registers.Control.ENSOP2 = 0;
			DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
		}

		if (port->IsVCONNSource) {
			port->Registers.Switches.VCONN_CC1 = 0;
			port->Registers.Switches.VCONN_CC2 = 0;
			DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
			port->IsVCONNSource = AW_FALSE;
		}

		ProtocolFlushTxFIFO(port);
		ProtocolFlushRxFIFO(port);
		ResetProtocolLayer(port, AW_TRUE);
		port->PolicySubIndex++;
		break;
	case 1:
		if (port->Registers.Status.I_COMP_CHNG &&
			!isVBUSOverVoltage(port, VBUS_MV_VSAFE0V + MDAC_MV_LSB)) {
			port->PolicySubIndex++;

			/* Set up VBus measure interrupt to watch for vSafe5V */
			port->Registers.Measure.MEAS_VBUS = 1;
			port->Registers.Measure.MDAC = VBUS_MDAC_4P62;
			DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

			TimerStart(&port->PolicyStateTimer, tSrcRecoverMax + tSrcTurnOn);
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			SetPEState(port, peSinkStartup);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		if (port->Registers.Status.I_COMP_CHNG &&
			isVBUSOverVoltage(port, VBUS_MV_VSAFE5V_L)) {
			TimerDisable(&port->PolicyStateTimer);
			port->PolicySubIndex++;
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			SetPEState(port, peSinkStartup);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		SetPEState(port, peSinkStartup);
		break;
	}
}

void PolicySinkStartup(Port_t *port)
{
#ifdef AW_HAVE_VDM
	AW_S32 i;
#endif /* AW_HAVE_VDM */

	/* Set or restore MDAC detach value */
	port->Registers.Measure.MEAS_VBUS = 1;
	port->Registers.Measure.MDAC = (port->DetachThreshold / MDAC_MV_LSB) - 1;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

#ifdef AW_GSCE_FIX
	port->Registers.Mask.M_CRC_CHK = 0; /* Added for GSCE workaround */
#endif /* AW_GSCE_FIX */
	port->Registers.Mask.M_COLLISION = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->Registers.MaskAdv.M_RETRYFAIL = 0;
	port->Registers.MaskAdv.M_HARDSENT = 0;
	port->Registers.MaskAdv.M_TXSENT = 0;
	port->Registers.MaskAdv.M_HARDRST = 0;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	port->Registers.MaskAdv.M_GCRCSENT = 0;
	DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

	if (port->Registers.Control.BIST_TMODE == 1) {
		port->Registers.Control.BIST_TMODE = 0;
		DeviceWrite(port, regControl3, 1, &port->Registers.Control.byte[3]);
	}

	port->USBPDContract.object = 0;
	port->PartnerCaps.object = 0;
	port->IsPRSwap = AW_FALSE;
	port->PolicyIsSource = AW_FALSE;
	port->PpsEnabled = AW_FALSE;

	ResetProtocolLayer(port, AW_FALSE);

	port->Registers.Switches.POWERROLE = port->PolicyIsSource;
	port->Registers.Slice.byte = port->sink_bist_reg;
	DeviceWrite(port, regSlice, 1, &port->Registers.Slice.byte);
	//port->Registers.Switches.AUTO_CRC = 1;
	DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

	port->Registers.Power.POWER = 0xF;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

	port->CapsCounter = 0;
	port->CollisionCounter = 0;
	TimerDisable(&port->PolicyStateTimer);
	SetPEState(port, peSinkDiscovery);

#ifdef AW_HAVE_VDM
	port->mode_entered = AW_FALSE;

	port->core_svid_info.num_svids = 0;
	for (i = 0; i < MAX_NUM_SVIDS; i++)
		port->core_svid_info.svids[i] = 0;
	port->AutoModeEntryObjPos = -1;
	port->svid_discvry_done = AW_FALSE;
	port->svid_discvry_idx  = -1;
	port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */

#ifdef AW_HAVE_DP
	DP_Initialize(port);
#endif /* AW_HAVE_DP */
}

void PolicySinkDiscovery(Port_t *port)
{

	if (isVSafe5V(port)) {
		port->IsHardReset = AW_FALSE;

		SetPEState(port, peSinkWaitCaps);
		TimerStart(&port->PolicyStateTimer, tTypeCSinkWaitCap);
		if (port->get_sink_cap_flag && !port->Registers.Switches.AUTO_CRC) {
			port->get_sink_cap_flag = AW_FALSE;
			port->Registers.Switches.AUTO_CRC = 1;
			DeviceWrite(port, regSwitches1, 1,
					&port->Registers.Switches.byte[1]);
		}
	} else {
		TimerStart(&port->PolicyStateTimer, tVBusPollShort);
		port->PEIdle = AW_TRUE;
	}
}

void PolicySinkWaitCaps(Port_t *port)
{
	if (port->ProtocolMsgRx) {
		port->ProtocolMsgRx = AW_FALSE;
		if ((port->PolicyRxHeader.NumDataObjects > 0) &&
			(port->PolicyRxHeader.MessageType == DMTSourceCapabilities)) {
			TimerDisable(&port->PolicyStateTimer);
			UpdateCapabilitiesRx(port, AW_TRUE);

			/* Align our PD spec rev with the port partner */
			DPM_SetSpecRev(port, SOP_TYPE_SOP,
					(SpecRev)port->PolicyRxHeader.SpecRevision);

			SetPEState(port, peSinkEvaluateCaps);
		} else if ((port->PolicyRxHeader.NumDataObjects == 0) &&
				(port->PolicyRxHeader.MessageType == CMTSoftReset)) {
			SetPEState(port, peSinkSoftReset);
		}
	} else if ((port->PolicyHasContract == AW_TRUE) &&
			(port->HardResetCounter > nHardResetCount)) {
		SetPEState(port, peErrorRecovery);
	} else if ((port->HardResetCounter <= nHardResetCount) &&
			 TimerExpired(&port->PolicyStateTimer)) {
		SetPEState(port, peSinkSendHardReset);
	} else {
		port->PEIdle = AW_TRUE;
	}
}

void PolicySinkEvaluateCaps(Port_t *port)
{
	AW_U8 i, j;
	AW_BOOL src_support_pps = AW_FALSE;

	if ((DPM_GetSinkCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSinkCap(port->dpm, port) == NULL))
		return;

	port->HardResetCounter = 0;
	port->src_support_fixed = AW_FALSE;

	for (i = DPM_GetSinkCapHeader(port->dpm, port)->NumDataObjects; i > 0; i--) {
		if (DPM_GetSinkCap(port->dpm, port)[i - 1].PDO.SupplyType == pdoTypeAugmented) {
			for (j = 0; j < port->SrcCapsHeaderReceived.NumDataObjects; j++) {
				if (port->SrcCapsReceived[j].PDO.SupplyType == pdoTypeAugmented) {
					if ((port->SrcCapsReceived[j].PPSAPDO.MaxVoltage  >= DPM_GetSinkCap(port->dpm, port)[i - 1].PPSAPDO.MaxVoltage) &&
						(port->SrcCapsReceived[j].PPSAPDO.MinVoltage <=  DPM_GetSinkCap(port->dpm, port)[i - 1].PPSAPDO.MinVoltage) &&
						(port->SrcCapsReceived[j].PPSAPDO.MaxCurrent >= DPM_GetSinkCap(port->dpm, port)[i - 1].PPSAPDO.MaxCurrent)) {
						src_support_pps = AW_TRUE;
						break;
					}
				}
			}

			if (src_support_pps == AW_TRUE)
				break;
		} else if (DPM_GetSinkCap(port->dpm, port)[i - 1].PDO.SupplyType == pdoTypeFixed) {
			for (j = 0; j < port->SrcCapsHeaderReceived.NumDataObjects; j++) {
				AW_LOG("[%d].object = 0x%x\n", j, port->SrcCapsReceived[j].object);
				if (port->SrcCapsReceived[j].PDO.SupplyType == pdoTypeFixed) {
					if (port->SrcCapsReceived[j].FPDOSupply.Voltage  == DPM_GetSinkCap(port->dpm, port)[i - 1].FPDOSink.Voltage) {
						port->src_support_fixed = AW_TRUE;
						break;
					}
				}
			}

			if (port->src_support_fixed == AW_TRUE)
				break;
		}
	}

	AW_LOG("i = %d j = %d pps = %d fixed = %d\n", i, j, src_support_pps, port->src_support_fixed);
	if ((port->src_support_fixed == AW_TRUE) || (src_support_pps == AW_TRUE)) {
		port->PartnerCaps.object = port->SrcCapsReceived[0].object;

		/* Initialize common fields */
		port->SinkRequest.object = 0;
		port->SinkRequest.FVRDO.ObjectPosition = (j + 1) & 0x07; // select vol level
		port->SinkRequest.FVRDO.GiveBack = port->PortConfig.SinkGotoMinCompatible;
		port->SinkRequest.FVRDO.NoUSBSuspend = port->PortConfig.SinkUSBSuspendOperation;
		port->SinkRequest.FVRDO.USBCommCapable = port->PortConfig.SinkUSBCommCapable;

		if (src_support_pps == AW_TRUE) {
			port->SinkRequest.PPSRDO.OpCurrent = DPM_GetSinkCap(port->dpm, port)[i - 1].PPSAPDO.MaxCurrent;
			port->SinkRequest.PPSRDO.Voltage = DPM_GetSinkCap(port->dpm, port)[i - 1].PPSAPDO.MinVoltage * 5;
			port->PpsEnabled = AW_TRUE;
		} else if (port->src_support_fixed == AW_TRUE) {
			port->PpsEnabled = AW_FALSE;
			if (port->SrcCapsReceived[j].FPDOSupply.MaxCurrent >= DPM_GetSinkCap(port->dpm, port)[i - 1].FPDOSink.OperationalCurrent)
				port->SinkRequest.FVRDO.OpCurrent = DPM_GetSinkCap(port->dpm, port)[i - 1].FPDOSink.OperationalCurrent;
			else
				port->SinkRequest.FVRDO.OpCurrent = port->SrcCapsReceived[j].FPDOSupply.MaxCurrent;
			port->SinkRequest.FVRDO.MinMaxCurrent = port->SrcCapsReceived[j].FPDOSupply.MaxCurrent;
		}

		SetPEState(port, peSinkSelectCapability);
	} else if ((port->src_support_fixed == AW_FALSE) && (src_support_pps == AW_FALSE)) {
		port->PartnerCaps.object = port->SrcCapsReceived[0].object;

		/* Initialize common fields */
		port->SinkRequest.object = 0;
		port->SinkRequest.FVRDO.ObjectPosition = 1; // select vol level
		port->SinkRequest.FVRDO.GiveBack = port->PortConfig.SinkGotoMinCompatible;
		port->SinkRequest.FVRDO.NoUSBSuspend = port->PortConfig.SinkUSBSuspendOperation;
		port->SinkRequest.FVRDO.USBCommCapable = port->PortConfig.SinkUSBCommCapable;
		port->SinkRequest.FVRDO.OpCurrent = port->SrcCapsReceived[0].FPDOSupply.MaxCurrent;
		port->SinkRequest.FVRDO.MinMaxCurrent = port->SrcCapsReceived[0].FPDOSupply.MaxCurrent;

		SetPEState(port, peSinkSelectCapability);
	} else {
		/* For now, we are just going to go back to the wait state */
		/* instead of sending a reject or reset (may change in future) */
		SetPEState(port, peSinkWaitCaps);
		TimerStart(&port->PolicyStateTimer, tTypeCSinkWaitCap);
	}
}

void PolicySinkSelectCapability(Port_t *port)
{
	AW_U8 j;

	/* AW_LOG("enter PolicySubIndex = %d\n", port->PolicySubIndex); */
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendData(port, DMTRequest, &port->SinkRequest,
						sizeof(doDataObject_t), peSinkSelectCapability, 1,
						SOP_TYPE_SOP, AW_FALSE) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
			port->WaitingOnHR = AW_TRUE;
			for (j = 0; j < port->SrcCapsHeaderReceived.NumDataObjects; j++) {
				if (port->SrcCapsReceived[j].PDO.SupplyType == pdoTypeAugmented) {
					port->src_support_pps = AW_TRUE;
					break;
				}
			}
		}
		break;
	case 1:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					/* Check if PPS was selected (Here as well, for GUI req) */
					port->PpsEnabled =
						(port->SrcCapsReceived[port->SinkRequest.FVRDO.ObjectPosition - 1].PDO.SupplyType
						== pdoTypeAugmented) ? AW_TRUE : AW_FALSE;

					port->PolicyHasContract = AW_TRUE;
					port->USBPDContract.object = port->SinkRequest.object;
					TimerStart(&port->PolicyStateTimer, tPSTransition);
					SetPEState(port, peSinkTransitionSink);

					if (port->PpsEnabled == AW_TRUE)
						TimerStart(&port->PpsTimer, tPPSRequest);
					break;
				case CMTWait:
				case CMTReject:
					if (port->PolicyHasContract)
						SetPEState(port, peSinkReady);
					else {
						SetPEState(port, peSinkWaitCaps);

						/* Make sure we don't send reset to prevent loop */
						port->HardResetCounter = nHardResetCount + 1;
					}
					break;
				case CMTSoftReset:
					SetPEState(port, peSinkSoftReset);
					break;
				default:
					SetPEState(port, peSinkSendSoftReset);
					break;
				}
			} else {
				switch (port->PolicyRxHeader.MessageType) {
				case DMTSourceCapabilities:
					UpdateCapabilitiesRx(port, AW_TRUE);
					SetPEState(port, peSinkEvaluateCaps);
					break;
				default:
					SetPEState(port, peSinkSendSoftReset);
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySinkTransitionSink(Port_t *port)
{
	port->PEIdle = AW_FALSE;

	if (port->ProtocolMsgRx) {
		port->ProtocolMsgRx = AW_FALSE;
		if (port->PolicyRxHeader.NumDataObjects == 0) {
			switch (port->PolicyRxHeader.MessageType) {
			case CMTPS_RDY: {
				SetPEState(port, peSinkReady);
				notify_observers(PD_NEW_CONTRACT, port->I2cAddr,
						&port->USBPDContract);
				port->src_support_pd = AW_TRUE;

				if (port->PpsEnabled) {
					/* 80% --> (V * 20) * (80 / 100) */
					//port->DetachThreshold =
						//port->USBPDContract.PPSRDO.Voltage * 16;
				} else {
					if (port->PartnerCaps.FPDOSupply.Voltage == 100) {
						port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;
					} else {
						/* 80% --> (V * 50) * (80 / 100) */
						port->DetachThreshold =
							port->PartnerCaps.FPDOSupply.Voltage * 40;
					}
					TimerDisable(&port->PolicyStateTimer);
				}
				break;
			}
			case CMTSoftReset:
				SetPEState(port, peSinkSoftReset);
				break;
			default:
				SetPEState(port, peSinkSendHardReset);
				break;
			}
		} else {
			switch (port->PolicyRxHeader.MessageType) {
			case DMTSourceCapabilities:
				UpdateCapabilitiesRx(port, AW_TRUE);
				SetPEState(port, peSinkEvaluateCaps);
				break;
			default:
				SetPEState(port, peSinkSendHardReset);
				break;
			}
		}
	} else if (TimerExpired(&port->PolicyStateTimer)) {
		SetPEState(port, peSinkSendHardReset);
	} else {
		port->PEIdle = AW_TRUE;
	}
}

void PolicySinkReady(Port_t *port)
{
	if (port->ProtocolMsgRx) {
		/* Handle a received message */
		port->ProtocolMsgRx = AW_FALSE;
		if (port->PolicyRxHeader.NumDataObjects == 0) {
			switch (port->PolicyRxHeader.MessageType) {
			case CMTGotoMin:
				SetPEState(port, peSinkTransitionSink);
				TimerStart(&port->PolicyStateTimer, tPSTransition);
				break;
			case CMTGetSinkCap:
				SetPEState(port, peSinkGiveSinkCap);
				port->get_sink_cap_flag = AW_TRUE;
				break;
			case CMTGetSourceCap:
				SetPEState(port, peSinkGiveSourceCap);
				break;
			case CMTDR_Swap:
				SetPEState(port, peSinkEvaluateDRSwap);
				break;
			case CMTPR_Swap:
				SetPEState(port, peSinkEvaluatePRSwap);
				break;
			case CMTVCONN_Swap:
				SetPEState(port, peSinkEvaluateVCONNSwap);
				break;
			case CMTSoftReset:
				SetPEState(port, peSinkSoftReset);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case CMTGetCountryCodes:
				SetPEState(port, peGiveCountryCodes);
				break;
#endif /* 0 */
			case CMTPing:
				/* Ping ignored */
				break;
			case CMTReject:
			case CMTNotSupported:
				/* Rx'd Reject/NS are ignored - notify DPM if needed */
				break;
			/* Unexpected messages */
			case CMTAccept:
			case CMTWait:
				SetPEState(port, peSinkSendSoftReset);
				break;
			case CMTGetRevisonMessage:
				SetPEState(port, peGiveRevisonMessage);
				break;
			default:
				SetPEState(port, peNotSupported);
				break;
			}
		} else if (port->PolicyRxHeader.Extended == 1) {
			switch (port->PolicyRxHeader.MessageType) {
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case EXTStatus:
				/* todo inform policy manager */
				/* Send Get PPS Status in response to any event flags */
				/* Make sure data message is correct */
				if ((port->PolicyRxDataObj[0].ExtHeader.DataSize ==
						EXT_STATUS_MSG_BYTES) &&
					(port->PolicyRxDataObj[1].SDB.OCP ||
					port->PolicyRxDataObj[1].SDB.OTP ||
					(port->PpsEnabled &&
					port->PolicyRxDataObj[1].SDB.CVorCF))) {
					PolicySendCommand(port, CMTGetPPSStatus, peSinkReady,
							0, port->ProtocolMsgTxSop);
				}
				break;
#endif /* 0 */
			default:
#ifndef AW_HAVE_EXT_MSG
				port->ExtHeader.byte[0] = port->PolicyRxDataObj[0].byte[0];
				port->ExtHeader.byte[1] = port->PolicyRxDataObj[0].byte[1];
				if (port->ExtHeader.DataSize > EXT_MAX_MSG_LEGACY_LEN) {
					/* Request takes more than one chunk which is not
					 * supported
					 */
					port->WaitForNotSupported = AW_TRUE;
				}
#endif
				SetPEState(port, peNotSupported);
				break;
			}
		} else {
			switch (port->PolicyRxHeader.MessageType) {
			case DMTSourceCapabilities:
				UpdateCapabilitiesRx(port, AW_TRUE);
				SetPEState(port, peSinkEvaluateCaps);
				break;
			case DMTSinkCapabilities:
				break;
			case DMTAlert:
				SetPEState(port, peSinkAlertReceived);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case DMTGetCountryInfo:
				SetPEState(port, peGiveCountryInfo);
				break;
#endif /* 0 */
#ifdef AW_HAVE_VDM
			case DMTVenderDefined:
				convertAndProcessVdmMessage(port, port->ProtocolMsgRxSop);
				break;
#endif /* AW_HAVE_VDM */
			case DMTBIST:
				processDMTBIST(port);
				break;
			default:
				SetPEState(port, peNotSupported);
				break;
			}
		}
	} else if (port->USBPDTxFlag) {
		/* Has the device policy manager requested us to send a message? */
		if (port->PDTransmitHeader.NumDataObjects == 0) {
			switch (port->PDTransmitHeader.MessageType) {
			case CMTGetSourceCap:
				SetPEState(port, peSinkGetSourceCap);
				break;
			case CMTGetSinkCap:
				SetPEState(port, peSinkGetSinkCap);
				break;
			case CMTDR_Swap:
				SetPEState(port, peSinkSendDRSwap);
				break;
			case CMTVCONN_Swap:
				SetPEState(port, peSinkSendVCONNSwap);
				break;
#ifdef AW_HAVE_DRP
			case CMTPR_Swap:
				SetPEState(port, peSinkSendPRSwap);
				break;
#endif /* AW_HAVE_DRP */
			case CMTSoftReset:
				SetPEState(port, peSinkSendSoftReset);
				break;
#ifdef AW_DEBUG_CODE /* Not implemented yet */
			case CMTGetCountryCodes:
				SetPEState(port, peGetCountryCodes);
				break;
#endif /* 0 */
			case CMTGetPPSStatus:
				SetPEState(port, peGetPPSStatus);
				break;
			default:
				break;
			}
		} else {
			switch (port->PDTransmitHeader.MessageType) {
			case DMTRequest:
				port->SinkRequest.object = port->PDTransmitObjects[0].object;
				SetPEState(port, peSinkSelectCapability);
				break;
			case DMTVenderDefined:
#ifdef AW_HAVE_VDM
				doVdmCommand(port);
#endif /* AW_HAVE_VDM */
				break;
			default:
				break;
			}
		}
		port->USBPDTxFlag = AW_FALSE;
	} else if ((port->PortConfig.PortType == USBTypeC_DRP) &&
			 (port->PortConfig.reqPRSwapAsSnk) &&
			 (port->PartnerCaps.FPDOSupply.DualRolePower == AW_TRUE)) {
		SetPEState(port, peSinkSendPRSwap);
	} else if (port->PortConfig.reqDRSwapToDfpAsSink == AW_TRUE &&
			 port->PolicyIsDFP == AW_FALSE &&
			 DR_Swap_To_DFP_Supported) {
		port->PortConfig.reqDRSwapToDfpAsSink = AW_FALSE;
		SetPEState(port, peSinkSendDRSwap);
	} else if (port->PortConfig.reqVconnSwapToOnAsSink == AW_TRUE &&
			 port->IsVCONNSource == AW_FALSE &&
			 VCONN_Swap_To_On_Supported) {
		port->PortConfig.reqVconnSwapToOnAsSink = AW_FALSE;
		SetPEState(port, peSinkSendVCONNSwap);
	}
#ifdef AW_HAVE_VDM
	else if (port->PolicyIsDFP && (port->AutoVdmState != AUTO_VDM_DONE)) {
		if (port->WaitInSReady == AW_TRUE) {
			if (TimerExpired(&port->PolicyStateTimer) ||
				TimerDisabled(&port->PolicyStateTimer)) {
				TimerDisable(&port->PolicyStateTimer);
				autoVdmDiscovery(port);
			}
		} else {
			TimerStart(&port->PolicyStateTimer, 20 * TICK_SCALE_TO_MS);
			port->WaitInSReady = AW_TRUE;
		}
	} else if (port->cblRstState > CBL_RST_DISABLED) {
		processCableResetState(port);
	}
#endif /* AW_HAVE_VDM */
	else if (port->PpsEnabled == AW_TRUE && TimerExpired(&port->PpsTimer)) {
		/* PPS Timer send sink request to keep PPS */
		/* Request stored in the SinkRequest object */
		SetPEState(port, peSinkSelectCapability);
	} else {
		/* Wait for VBUSOK or HARDRST or GCRCSENT */
		port->PEIdle = AW_TRUE;
	}
}

void PolicySinkGiveSinkCap(Port_t *port)
{
	if ((DPM_GetSinkCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSinkCap(port->dpm, port) == NULL))
		return;

	PolicySendData(port, DMTSinkCapabilities,
				DPM_GetSinkCap(port->dpm, port),
				DPM_GetSinkCapHeader(port->dpm, port)->NumDataObjects *
					sizeof(doDataObject_t),
				peSinkReady, 0, SOP_TYPE_SOP, AW_FALSE);
}

void PolicyGiveSourceInfo(Port_t *port)
{
	AW_U32 sido = 0x50505;

	PolicySendData(port, DMTSourceInfo, &sido, 4,
			peSinkReady, 0, SOP_TYPE_SOP, AW_FALSE);
}

void PolicyGiveRevisonMessage(Port_t *port)
{
	AW_U32 rmdo = 0x31100000;

	PolicySendData(port, DMTRevisionInfo, &rmdo, 4,
			peSinkReady, 0, SOP_TYPE_SOP, AW_FALSE);
}

void PolicyGetBatteryCap(Port_t *port)
{
	AW_U8 info[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0x01};

	PolicySendData(port, EXTBatteryCapabilities, info, 9,
			peSourceReady, 0, SOP_TYPE_SOP, AW_TRUE);
}

void PolicyGetBatteryStatus(Port_t *port)
{
	AW_U32 sido = 0xFFFF0100;

	PolicySendData(port, DMTBatteryStatus, &sido, 4,
			peSourceReady, 0, SOP_TYPE_SOP, AW_FALSE);
}

void PolicySinkGetSinkCap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTGetSinkCap, peSinkGetSinkCap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects > 0) &&
				(port->PolicyRxHeader.MessageType == DMTSinkCapabilities)) {
				/* Notify DPM or others of new sink caps */
				SetPEState(port, peSinkReady);
			} else if ((port->PolicyRxHeader.NumDataObjects == 0) &&
					 ((port->PolicyRxHeader.MessageType == CMTReject) ||
					  (port->PolicyRxHeader.MessageType == CMTNotSupported))) {
				SetPEState(port, peSinkReady);
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySinkGiveSourceCap(Port_t *port)
{
	if (DPM_GetSourceCapHeader(port->dpm, port) == NULL)
		return;

#ifdef AW_HAVE_DRP
	if (port->PortConfig.PortType == USBTypeC_DRP) {
		PolicySendData(port, DMTSourceCapabilities,
					DPM_GetSourceCap(port->dpm, port),
					DPM_GetSourceCapHeader(port->dpm, port)->NumDataObjects *
						sizeof(doDataObject_t),
					peSinkReady, 0,
					SOP_TYPE_SOP, AW_FALSE);
	} else
#endif /* AW_HAVE_DRP */
	{
		SetPEState(port, peNotSupported);
	}
}

void PolicySinkGetSourceCap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTGetSourceCap, peSinkGetSourceCap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects > 0) &&
				(port->PolicyRxHeader.MessageType == DMTSourceCapabilities)) {
				/* Notify DPM or others of new source caps */
				UpdateCapabilitiesRx(port, AW_TRUE);
				SetPEState(port, peSinkEvaluateCaps);
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySinkSendDRSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTDR_Swap, peSinkSendDRSwap, 1,
				SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->PolicyIsDFP =
							(port->PolicyIsDFP == AW_TRUE) ? AW_FALSE : AW_TRUE;

					port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
					DeviceWrite(port, regSwitches1, 1,
							&port->Registers.Switches.byte[1]);

					notify_observers(DATA_ROLE, port->I2cAddr, NULL);
					if (port->PdRevSop == USBPDSPECREV2p0) {
						/* In PD2.0, DFP controls SOP* coms */
						if (port->PolicyIsDFP == AW_TRUE) {
							port->Registers.Control.ENSOP1 = SOP_P_Capable;
							port->Registers.Control.ENSOP2 = SOP_PP_Capable;
#ifdef AW_HAVE_VDM
							port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
						} else {
							port->Registers.Control.ENSOP1 = 0;
							port->Registers.Control.ENSOP2 = 0;
						}
						DeviceWrite(port, regControl1, 1,
								&port->Registers.Control.byte[1]);
					}
					/* Fall through */
					break;
				case CMTReject:
					SetPEState(port, peSinkReady);
					break;
				default:
					SetPEState(port, peSinkSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySinkEvaluateDRSwap(Port_t *port)
{
#ifdef AW_HAVE_VDM
	if (port->mode_entered == AW_TRUE) {
		SetPEState(port, peSinkSendHardReset);
		return;
	}
#endif /* AW_HAVE_VDM */

	if (!DR_Swap_To_DFP_Supported) {
		PolicySendCommand(port, CMTReject, peSinkReady, 0, SOP_TYPE_SOP);
	} else {
		if (PolicySendCommand(port, CMTAccept, peSinkReady, 0,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			port->PolicyIsDFP = (port->PolicyIsDFP == AW_TRUE) ? AW_FALSE : AW_TRUE;
			port->Registers.Switches.DATAROLE = port->PolicyIsDFP;
			DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

			notify_observers(DATA_ROLE, port->I2cAddr, NULL);
			if (port->PdRevSop == USBPDSPECREV2p0) {
				/* In PD2.0, DFP controls SOP* coms */
				if (port->PolicyIsDFP == AW_TRUE) {
					port->Registers.Control.ENSOP1 = SOP_P_Capable;
					port->Registers.Control.ENSOP2 = SOP_PP_Capable;
#ifdef AW_HAVE_VDM
					port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
				} else {
					port->Registers.Control.ENSOP1 = 0;
					port->Registers.Control.ENSOP2 = 0;
				}
				DeviceWrite(port, regControl1, 1,
							&port->Registers.Control.byte[1]);
			}
		}
	}
}

void PolicySinkSendVCONNSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTVCONN_Swap, peSinkSendVCONNSwap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		}
		break;
	case 1:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->PolicySubIndex++;
					break;
				case CMTWait:
				case CMTReject:
					SetPEState(port, peSinkReady);
					break;
				default:
					SetPEState(port, peSinkSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		if (port->IsVCONNSource) {
			TimerStart(&port->PolicyStateTimer, tVCONNSourceOn);
			port->PolicySubIndex++;
		} else {
			if (Type_C_Sources_VCONN) {
				if (port->CCPin == CC1) {
					port->Registers.Switches.VCONN_CC2 = 1;
					port->Registers.Switches.PDWN2 = 0;
				} else {
					port->Registers.Switches.VCONN_CC1 = 1;
					port->Registers.Switches.PDWN1 = 0;
				}

				DeviceWrite(port, regSwitches0, 1,
						&port->Registers.Switches.byte[0]);
			}

			port->IsVCONNSource = AW_TRUE;

			if (port->PdRevSop == USBPDSPECREV3p0) {
				/* In PD3.0, VConn Source controls SOP* coms */
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				port->Registers.Control.ENSOP2 = SOP_PP_Capable;
				DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
			}
			TimerStart(&port->PolicyStateTimer, tVCONNTransition);
			port->PolicySubIndex = 4;
		}
		break;
	case 3:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					/* Turn off our VConn */
					port->Registers.Switches.VCONN_CC1 = 0;
					port->Registers.Switches.VCONN_CC2 = 0;
					port->Registers.Switches.PDWN1 = 1;
					port->Registers.Switches.PDWN2 = 1;
					DeviceWrite(port, regSwitches0, 1,
							&port->Registers.Switches.byte[0]);

					port->IsVCONNSource = AW_FALSE;

					if (port->PdRevSop == USBPDSPECREV3p0) {
						/* In PD3.0, VConn Source controls SOP* coms */
						port->Registers.Control.ENSOP1 = 0;
						port->Registers.Control.ENSOP2 = 0;
						DeviceWrite(port, regControl1, 1,
								&port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
						port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
					}

					SetPEState(port, peSinkReady);
					break;
				default:
					/* For all other commands received, simply ignore them and */
					/* wait for timer expiration */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		if (TimerExpired(&port->PolicyStateTimer) == AW_TRUE) {
			port->PolicySubIndex++;
			/* Fall through to immediately send PS_RDY when timer expires. */
		} else {
			port->PEIdle = AW_TRUE;
			break;
		}
	default:
		PolicySendCommand(port, CMTPS_RDY, peSinkReady, 0, SOP_TYPE_SOP);
		break;
	}
}

void PolicySinkEvaluateVCONNSwap(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if ((port->IsVCONNSource && VCONN_Swap_To_Off_Supported) ||
			(!port->IsVCONNSource && VCONN_Swap_To_On_Supported)) {
			PolicySendCommand(port, CMTAccept, peSinkEvaluateVCONNSwap, 1,
					SOP_TYPE_SOP);
		} else {
			PolicySendCommand(port, CMTReject, peSinkReady, 0, SOP_TYPE_SOP);
		}
		break;
	case 1:
		if (port->IsVCONNSource) {
			TimerStart(&port->PolicyStateTimer, tVCONNSourceOn);
			port->PolicySubIndex++;
		} else {
			if (Type_C_Sources_VCONN) {
				if (port->CCPin == CC1) {
					port->Registers.Switches.VCONN_CC2 = 1;
					port->Registers.Switches.PDWN2 = 0;
				} else {
					port->Registers.Switches.VCONN_CC1 = 1;
					port->Registers.Switches.PDWN1 = 0;
				}

				DeviceWrite(port, regSwitches0, 1,
						&port->Registers.Switches.byte[0]);
			}

			port->IsVCONNSource = AW_TRUE;

			if (port->PdRevSop == USBPDSPECREV3p0) {
				/* In PD3.0, VConn Source controls SOP* coms */
				port->Registers.Control.ENSOP1 = SOP_P_Capable;
				port->Registers.Control.ENSOP2 = SOP_PP_Capable;
				DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
				port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
			}

			TimerStart(&port->PolicyStateTimer, tVCONNTransition);

			/* Move on to sending the PS_RDY message after the timer expires */
			port->PolicySubIndex = 3;
		}
		break;
	case 2:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					port->Registers.Switches.VCONN_CC1 = 0;
					port->Registers.Switches.VCONN_CC2 = 0;
					port->Registers.Switches.PDWN1 = 1;
					port->Registers.Switches.PDWN2 = 1;
					DeviceWrite(port, regSwitches0, 1,
								&port->Registers.Switches.byte[0]);

					port->IsVCONNSource = AW_FALSE;

					if (port->PdRevSop == USBPDSPECREV3p0) {
						/* In PD3.0, VConn Source controls SOP* coms */
						port->Registers.Control.ENSOP1 = 0;
						port->Registers.Control.ENSOP2 = 0;
						DeviceWrite(port, regControl1, 1,
							&port->Registers.Control.byte[1]);
#ifdef AW_HAVE_VDM
						port->discoverIdCounter = 0;
#endif /* AW_HAVE_VDM */
					}

					SetPEState(port, peSinkReady);
					break;
				default:
					/* For all other commands received, simply ignore them*/
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSourceSendHardReset);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		if (TimerExpired(&port->PolicyStateTimer) == AW_TRUE) {
			port->PolicySubIndex++;
			/* Fall through if timer expired. Immediately send PS_RDY. */
		} else {
			port->PEIdle = AW_TRUE;
			break;
		}
	default:
		PolicySendCommand(port, CMTPS_RDY, peSinkReady, 0, SOP_TYPE_SOP);
		break;
	}
}

void PolicySinkSendPRSwap(Port_t *port)
{
#ifdef AW_HAVE_DRP
	AW_U8 Status;
	/* Disable the config flag if we got here during automatic request */
	port->PortConfig.reqPRSwapAsSnk = AW_FALSE;
	switch (port->PolicySubIndex) {
	case 0:
		/* Send swap message */
		if (PolicySendCommand(port, CMTPR_Swap, peSinkSendPRSwap, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS) {
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		}
		break;
	case 1:
		/* Wait for Accept */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTAccept:
					port->IsPRSwap = AW_TRUE;
					port->PolicyHasContract = AW_FALSE;
					port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

					//notify_observers(POWER_ROLE, port->I2cAddr, NULL);

					TimerStart(&port->PolicyStateTimer, tPSSourceOff);
					port->PolicySubIndex++;
					break;
				case CMTWait:
				case CMTReject:
					SetPEState(port, peSinkReady);
					port->IsPRSwap = AW_FALSE;
					break;
				default:
					SetPEState(port, peSinkSendSoftReset);
					break;
				}
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peSinkReady);
			port->IsPRSwap = AW_FALSE;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		/* Wait for PS_RDY and perform the swap */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					RoleSwapToAttachedSource(port);
					port->PolicyIsSource = AW_TRUE;
					port->Registers.Switches.POWERROLE = port->PolicyIsSource;
					DeviceWrite(port, regSwitches1, 1,
								&port->Registers.Switches.byte[1]);
					TimerDisable(&port->PolicyStateTimer);
					port->PolicySubIndex++;
					break;
				default:
					/* For all other commands received, simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peErrorRecovery);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		/* Wait on VBus */
		if (isVSafe5V(port)) {
			/* Delay once VBus is present for potential switch delay. */
			TimerStart(&port->PolicyStateTimer, tVBusSwitchDelay);

			port->PolicySubIndex++;
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 5:
		Status = PolicySendCommand(port, CMTPS_RDY, peSourceStartup, 0, SOP_TYPE_SOP);
		if (Status == STAT_ERROR)
			SetPEState(port, peErrorRecovery);
		else if (Status == STAT_SUCCESS)
			TimerStart(&port->SwapSourceStartTimer, tSwapSourceStart);
		break;
	default:
		port->PolicySubIndex = 0;
		break;
	}
#endif /* AW_HAVE_DRP */
}

void PolicySinkEvaluatePRSwap(Port_t *port)
{
#ifdef AW_HAVE_DRP
	AW_U8 Status;

	switch (port->PolicySubIndex) {
	case 0:
		/* Send either the Accept or Reject */
		if (!Accepts_PR_Swap_As_Snk ||
			(port->PortConfig.PortType != USBTypeC_DRP) ||
			(port->SrcCapsReceived[0].FPDOSupply.DualRolePower == AW_FALSE)) {
			PolicySendCommand(port, CMTReject, peSinkReady, 0, port->ProtocolMsgRxSop);
		} else {
			port->IsPRSwap = AW_TRUE;
			if (PolicySendCommand(port, CMTAccept, peSinkEvaluatePRSwap, 1,
						SOP_TYPE_SOP) == STAT_SUCCESS) {
				port->PolicyHasContract = AW_FALSE;
				port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

				//notify_observers(POWER_ROLE, port->I2cAddr, NULL);

				TimerStart(&port->PolicyStateTimer, tPSSourceOff);
			}
		}
		break;
	case 1:
		/* Wait for PS_RDY and perform the swap */
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if (port->PolicyRxHeader.NumDataObjects == 0) {
				switch (port->PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					RoleSwapToAttachedSource(port);
					port->PolicyIsSource = AW_TRUE;

					port->Registers.Switches.POWERROLE = port->PolicyIsSource;
					DeviceWrite(port, regSwitches1, 1,
							&port->Registers.Switches.byte[1]);

					TimerDisable(&port->PolicyStateTimer);
					port->PolicySubIndex++;
					break;
				default:
					/* For all other commands received, simply ignore them */
					break;
				}
			}
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, peErrorRecovery);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		/* Wait for VBUS to rise */
		if (isVSafe5V(port)) {
			/* Delay once VBus is present for potential switch delay. */
			TimerStart(&port->PolicyStateTimer, tVBusSwitchDelay);

			port->PolicySubIndex++;
		} else {
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			port->PEIdle = AW_TRUE;
		}
		break;
	case 3:
		if (TimerExpired(&port->PolicyStateTimer)) {
			TimerDisable(&port->PolicyStateTimer);
			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 4:
		Status = PolicySendCommand(port, CMTPS_RDY, peSourceStartup, 0, SOP_TYPE_SOP);
		if (Status == STAT_ERROR) {
			SetPEState(port, peErrorRecovery);
		} else if (Status == STAT_SUCCESS) {
			port->IsPRSwap = AW_FALSE;
			TimerStart(&port->SwapSourceStartTimer, tSwapSourceStart);

			/* Disable auto PR swap flag to prevent swapping back */
			port->PortConfig.reqPRSwapAsSrc = AW_FALSE;
		}
		break;
	default:
		port->PolicySubIndex = 0;
		break;
	}
#else
	PolicySendCommand(port, CMTReject, peSinkReady, 0, port->ProtocolMsgRxSop);
#endif /* AW_HAVE_DRP */
}
#endif /* AW_HAVE_SNK */

void PolicySinkAlertReceived(Port_t *port)
{
	if (port->PolicyRxDataObj[0].ADO.Battery ||
		port->PolicyRxDataObj[0].ADO.OCP ||
		port->PolicyRxDataObj[0].ADO.OTP ||
		port->PolicyRxDataObj[0].ADO.OpCondition ||
		port->PolicyRxDataObj[0].ADO.Input) {
		/* Send Get_Status */
		notify_observers(ALERT_EVENT, port->I2cAddr, &port->PolicyRxDataObj[0].object);
	}

	SetPEState(port, peSinkReady);
}

#ifdef AW_HAVE_VDM
void PolicyGiveVdm(Port_t *port)
{
	port->PEIdle = AW_TRUE;

	if (port->ProtocolMsgRx &&
		port->PolicyRxHeader.MessageType == DMTVenderDefined &&
		!(port->sendingVdmData && port->vdm_expectingresponse)) {
		/* Received something while trying to transmit, and */
		/* it isn't an expected response... */
		sendVdmMessageFailed(port);
	} else if (port->sendingVdmData) {

		AW_U8 result = PolicySendData(port, DMTVenderDefined,
						port->vdm_msg_obj,
						port->vdm_msg_length*sizeof(doDataObject_t),
						port->vdm_next_ps, 0,
						port->VdmMsgTxSop, AW_FALSE);
		if (result == STAT_SUCCESS) {
			if (port->vdm_expectingresponse)
				startVdmTimer(port, port->PolicyState);
			else
				resetPolicyState(port);
			port->sendingVdmData = AW_FALSE;
		} else if (result == STAT_ERROR) {
			sendVdmMessageFailed(port);
			port->sendingVdmData = AW_FALSE;
		}

		port->PEIdle = AW_FALSE;
	} else {
		sendVdmMessageFailed(port);
	}
}

void PolicyVdm(Port_t *port)
{
	if (port->ProtocolMsgRx) {
		if (port->PolicyRxHeader.NumDataObjects != 0) {
			switch (port->PolicyRxHeader.MessageType) {
			case DMTVenderDefined:
				convertAndProcessVdmMessage(port, port->ProtocolMsgRxSop);
				port->ProtocolMsgRx = AW_FALSE;
				break;
			default:
				resetPolicyState(port);
				break;
			}
		} else {
			resetPolicyState(port);
		}
	} else {
		port->PEIdle = AW_TRUE;
	}

	if (port->VdmTimerStarted && (TimerExpired(&port->VdmTimer))) {
		if (port->PolicyState == peDfpUfpVdmIdentityRequest)
			port->AutoVdmState = AUTO_VDM_DONE;
		vdmMessageTimeout(port);
	}
}

#endif /* AW_HAVE_VDM */

void PolicyInvalidState(Port_t *port)
{
	/* reset if we get to an invalid state */
	if (port->PolicyIsSource)
		SetPEState(port, peSourceSendHardReset);
	else
		SetPEState(port, peSinkSendHardReset);
}

void PolicySendGenericCommand(Port_t *port)
{
	AW_U8 status;

	switch (port->PolicySubIndex) {
	case 0:
		status = PolicySendCommand(port, port->PDTransmitHeader.MessageType,
				peSendGenericCommand, 1, SOP_TYPE_SOP);
		if (status == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		else if (status == STAT_ERROR)
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			/* Check and handle message response */
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			/* If no response, or no response expected, state will time out */
			/* after tSenderResponse. */

			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicySendGenericData(Port_t *port)
{
	AW_U8 status;

	switch (port->PolicySubIndex) {
	case 0:
		status = PolicySendData(port, port->PDTransmitHeader.MessageType,
				port->PDTransmitObjects,
				port->PDTransmitHeader.NumDataObjects * sizeof(doDataObject_t),
				peSendGenericData, 1,
				port->PolicyMsgTxSop, AW_FALSE);
		if (status == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		else if (status == STAT_ERROR)
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			/* Check and handle message response */
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			/* If no response, or no response expected, state will time out */
			/* after tSenderResponse. */
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

/* General PD Messaging */
void PolicySendHardReset(Port_t *port)
{
	AW_U8 data;

	if (!port->IsHardReset) {
		port->HardResetCounter++;
		port->IsHardReset = AW_TRUE;
		data = port->Registers.Control.byte[3] | 0x40;
		DeviceWrite(port, regControl3, 1, &data);
	}
}

AW_U8 PolicySendCommand(Port_t *port, AW_U8 Command, PolicyState_t nextState,
		AW_U8 subIndex, SopType sop)
{
	AW_U8 Status = STAT_BUSY;

	switch (port->PDTxStatus) {
	case txIdle:
		port->PolicyTxHeader.word = 0;
		port->PolicyTxHeader.NumDataObjects = 0;
		port->PolicyTxHeader.MessageType = Command;
		if (sop == SOP_TYPE_SOP) {
			port->PolicyTxHeader.PortDataRole = port->PolicyIsDFP;
			port->PolicyTxHeader.PortPowerRole = port->PolicyIsSource;
		} else {
			port->PolicyTxHeader.PortDataRole = 0;
			/* Cable plug field when SOP' & SOP'', currently not used */
			port->PolicyTxHeader.PortPowerRole = 0;
		}
		port->PolicyTxHeader.SpecRevision = DPM_SpecRev(port, sop);
		port->ProtocolMsgTxSop = sop;
		port->PDTxStatus = txSend;

		/* Shortcut to transmit */
		if (port->ProtocolState == PRLIdle)
			ProtocolIdle(port);
		break;
	case txSend:
	case txBusy:
	case txWait:
		/* Waiting for GoodCRC or timeout of the protocol */
		if (TimerExpired(&port->ProtocolTimer)) {
			TimerDisable(&port->ProtocolTimer);
			port->ProtocolState = PRLIdle;
			port->PDTxStatus = txIdle;

			/* Jumping to the next state might not be appropriate error
			 * handling for a timeout, but will prevent a hang in case
			 * the caller isn't expecting it.
			 */
			SetPEState(port, nextState);
			port->PolicySubIndex = subIndex;

			Status = STAT_ERROR;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case txSuccess:
		SetPEState(port, nextState);
		port->PolicySubIndex = subIndex;
		Status = STAT_SUCCESS;
		break;
	case txError:
		/* Didn't receive a GoodCRC message... */
		if (port->PolicyState == peSourceSendSoftReset)
			SetPEState(port, peSourceSendHardReset);
		else if (port->PolicyState == peSinkSendSoftReset)
			SetPEState(port, peSinkSendHardReset);
		else if (port->PolicyIsSource)
			SetPEState(port, peSourceSendSoftReset);
		else
			SetPEState(port, peSinkSendSoftReset);
		Status = STAT_ERROR;
		break;
	case txCollision:
		port->CollisionCounter++;
		if (port->CollisionCounter > nRetryCount) {
			if (port->PolicyIsSource)
				SetPEState(port, peSourceSendHardReset);
			else
				SetPEState(port, peSinkSendHardReset);

			port->PDTxStatus = txReset;
			Status = STAT_ERROR;
		} else {
			/* Clear the transmitter status for the next operation */
			port->PDTxStatus = txIdle;
		}
		break;
	case txAbort:
		if (port->PolicyIsSource)
			SetPEState(port, peSourceReady);
		else
			SetPEState(port, peSinkReady);

		Status = STAT_ABORT;
		port->PDTxStatus = txIdle;

		break;
	default:
		if (port->PolicyIsSource)
			SetPEState(port, peSourceSendHardReset);
		else
			SetPEState(port, peSinkSendHardReset);

		port->PDTxStatus = txReset;
		Status = STAT_ERROR;
		break;
	}
	return Status;
}

AW_U8 PolicySendData(Port_t *port, AW_U8 MessageType, void *data,
					AW_U32 len, PolicyState_t nextState,
					AW_U8 subIndex, SopType sop, AW_BOOL extMsg)
{
	AW_U8 Status = STAT_BUSY;
	AW_U32 i;
	AW_U32 len_temp = 0;
	AW_U8 *pData = (AW_U8 *)data;
	AW_U8 *pOutBuf = (AW_U8 *)port->PolicyTxDataObj;

	switch (port->PDTxStatus) {
	case txIdle:
		port->PolicyTxHeader.word = 0;
		port->PolicyTxHeader.SpecRevision = DPM_SpecRev(port, sop);

		if ((port->PolicyTxHeader.SpecRevision == USBPDSPECREV2p0) && (MessageType == DMTSourceCapabilities)
			&& (port->src_caps[port->src_pdo_size - 1].FPDOSupply.SupplyType == pdoTypeAugmented))
			len_temp = len - 4;
		else
			len_temp = len;

		port->PolicyTxHeader.MessageType = MessageType;
		port->PolicyTxHeader.NumDataObjects = len_temp / 4;
		if (sop == SOP_TYPE_SOP) {
			port->PolicyTxHeader.PortDataRole = port->PolicyIsDFP;
			port->PolicyTxHeader.PortPowerRole = port->PolicyIsSource;
		} else {
			/* Cable plug field when SOP' & SOP'', currently not used */
			port->PolicyTxHeader.PortPowerRole = 0;
			port->PolicyTxHeader.PortDataRole = 0;
		}

		if (extMsg == AW_TRUE) {
#ifdef AW_HAVE_EXT_MSG
			/* Set extended bit */
			port->PolicyTxHeader.Extended = 1;
			/* Initialize extended messaging state */
			port->ExtChunkOffset = 0;
			port->ExtChunkNum = 0;
			port->ExtTxOrRx = TXing;
			port->ExtWaitTxRx = AW_FALSE;
			/* Set extended header */
			port->ExtTxHeader.word = 0;
			len_temp = (len_temp > EXT_MAX_MSG_LEN) ? EXT_MAX_MSG_LEN : len_temp;
			port->ExtTxHeader.Chunked = 1;
			port->ExtTxHeader.DataSize = len_temp;
			/* Set the tx buffer */
			pOutBuf = port->ExtMsgBuffer;
#endif /* AW_HAVE_EXT_MSG */
		}

		/* Copy message */
		for (i = 0; i < len_temp; i++)
			pOutBuf[i] = pData[i];

		if (port->PolicyState == peSourceSendCaps)
			port->CapsCounter++;

		port->ProtocolMsgTxSop = sop;
		port->PDTxStatus = txSend;

		/* Shortcut to transmit */
		if (port->ProtocolState == PRLIdle)
			ProtocolIdle(port);
		break;
	case txSend:
	case txBusy:
	case txWait:
		/* Waiting for GoodCRC or timeout of the protocol */
		if (TimerExpired(&port->ProtocolTimer)) {
			TimerDisable(&port->ProtocolTimer);
			port->ProtocolState = PRLIdle;
			port->PDTxStatus = txIdle;

			/* Jumping to the next state might not be appropriate error
			 * handling for a timeout, but will prevent a hang in case
			 * the caller isn't expecting it.
			 */
			SetPEState(port, nextState);
			port->PolicySubIndex = subIndex;

			Status = STAT_ERROR;

#ifdef AW_HAVE_EXT_MSG
			/* Possible timeout when trying to send a chunked message to
			 * a device that doesn't support chunking.
			 * TODO - Notify DPM of failure if necessary.
			 */
			port->ExtWaitTxRx = AW_FALSE;
			port->ExtChunkNum = 0;
			port->ExtTxOrRx = NoXfer;
			port->ExtChunkOffset = 0;
#endif /* AW_HAVE_EXT_MSG */
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case txCollision:
		Status = STAT_ERROR;
		break;
	case txSuccess:
#ifdef AW_HAVE_EXT_MSG
		if (extMsg == AW_TRUE &&
			port->ExtChunkOffset < port->ExtTxHeader.DataSize) {
			port->PDTxStatus = txBusy;
			break;
		}
#endif /* AW_HAVE_EXT_MSG */
		SetPEState(port, nextState);
		port->PolicySubIndex = subIndex;
		TimerDisable(&port->ProtocolTimer);
		Status = STAT_SUCCESS;
		break;
	case txError:
		/* Didn't receive a GoodCRC message... */
		if (sop == SOP_TYPE_SOP) {
			if (port->PolicyState == peSourceSendCaps &&
				port->PolicyHasContract == AW_FALSE) {
				SetPEState(port, peSourceSendCaps);
				if (port->MessageIDCounter[SOP_TYPE_SOP] == 0x07)
					port->MessageIDCounter[SOP_TYPE_SOP] = 0;
				else
					port->MessageIDCounter[SOP_TYPE_SOP]++;
				msleep(180);
			} else if (port->PolicyIsSource) {
				SetPEState(port, peSourceSendSoftReset);
			} else {
				SetPEState(port, peSinkSendSoftReset);
			}
		} else {
#ifdef AW_HAVE_VDM
			if (port->cblPresent == AW_TRUE)
				port->cblRstState = CBL_RST_START;
#endif /* AW_HAVE_VDM */
		}
			Status = STAT_ERROR;
			break;
	case txAbort:
		if (port->PolicyIsSource)
			SetPEState(port, peSourceReady);
		else
			SetPEState(port, peSinkReady);

		Status = STAT_ABORT;
		port->PDTxStatus = txIdle;

		break;
	default:
		if (port->PolicyIsSource)
			SetPEState(port, peSourceSendHardReset);
		else
			SetPEState(port, peSinkSendHardReset);

		port->PDTxStatus = txReset;
		Status = STAT_ERROR;
		break;
	}

	return Status;
}

void UpdateCapabilitiesRx(Port_t *port, AW_BOOL IsSourceCaps)
{
	AW_U32 i;
	sopMainHeader_t *capsHeaderRecieved;
	doDataObject_t *capsReceived;

	if (IsSourceCaps == AW_TRUE) {
		capsHeaderRecieved = &port->SrcCapsHeaderReceived;
		capsReceived = port->SrcCapsReceived;
	} else {
		capsHeaderRecieved = &port->SnkCapsHeaderReceived;
		capsReceived = port->SnkCapsReceived;
	}

	capsHeaderRecieved->word = port->PolicyRxHeader.word;

	for (i = 0; i < capsHeaderRecieved->NumDataObjects; i++)
		capsReceived[i].object = port->PolicyRxDataObj[i].object;

	for (i = capsHeaderRecieved->NumDataObjects; i < 7; i++)
		capsReceived[i].object = 0;

	port->PartnerCaps.object = capsReceived[0].object;
	port->usb_commcapable = capsReceived[0].FPDOSupply.USBCommCapable;
}

#ifdef AW_HAVE_EXT_MSG
void PolicyGiveCountryCodes(Port_t *port)
{
	AW_U32 noCodes = gCountry_codes[0] | gCountry_codes[1] << 8;

	PolicySendData(port, EXTCountryCodes, (AW_U8 *)gCountry_codes, noCodes*2+2,
				port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
				SOP_TYPE_SOP, AW_TRUE);
}

void PolicyGetCountryCodes(Port_t *port)
{
	 PolicySendCommand(port, CMTGetCountryCodes,
				port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
				SOP_TYPE_SOP);
}

void PolicyGiveCountryInfo(Port_t *port)
{
	/* Allocate 4-byte buffer so we don't need a full 260 byte data */
	AW_U8 buf[4] = {0};
	CountryInfoReq *req = (CountryInfoReq *)port->PolicyRxDataObj;

	/* Echo the first to bytes of country code */
	CountryInfoResp *resp = (CountryInfoResp *)buf;

	resp->CountryCode[1] = req->CountryCode[0];
	resp->CountryCode[0] = req->CountryCode[1];
	resp->Reserved[0] = resp->Reserved[1] = 0;
	PolicySendData(port, EXTCountryInfo, resp, 4,
				(port->PolicyIsSource == AW_TRUE) ? peSourceReady : peSinkReady,
				0, SOP_TYPE_SOP, AW_TRUE);
}
#endif /* AW_HAVE_EXT_MSG */

#ifdef AW_HAVE_EXT_MSG
void PolicyGetPPSStatus(Port_t *port)
{
	switch (port->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(port, CMTGetPPSStatus, peGetPPSStatus, 1,
					SOP_TYPE_SOP) == STAT_SUCCESS)
			TimerStart(&port->PolicyStateTimer, tSenderResponse);
		break;
	default:
		if (port->ProtocolMsgRx) {
			port->ProtocolMsgRx = AW_FALSE;
			if ((port->PolicyRxHeader.NumDataObjects > 0) &&
				(port->PolicyRxHeader.MessageType == EXTPPSStatus))
				SetPEState(port, port->PolicyIsSource ?
						peSourceReady : peSinkReady);
			else
				SetPEState(port, port->PolicyIsSource ?
						peSourceSendHardReset : peSinkSendHardReset);
		} else if (TimerExpired(&port->PolicyStateTimer)) {
			SetPEState(port, port->PolicyIsSource ? peSourceReady : peSinkReady);
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	}
}

void PolicyGivePPSStatus(Port_t *port)
{
	PPSStatus_t ppsstatus;

	switch (port->PolicySubIndex) {
	case 0:
		/* Only need to get the values once */
		ppsstatus.OutputVoltage = platform_get_pps_voltage(port->PortID) / 20;
		ppsstatus.OutputCurrent = 0xFF; /* Not supported field for now */
		ppsstatus.byte[3] = 0x00;

		port->PolicySubIndex++;
		/* Fall through */
	case 1:
		PolicySendData(port, EXTPPSStatus, ppsstatus.byte, 4,
					port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
					SOP_TYPE_SOP, AW_TRUE);
		break;
	default:
		break;
	}
}

void PolicySourceCapExtended(struct Port *port)
{
#ifdef AW_DEBUG_CODE
	PolicySendData(port, EXTSourceCapExt, port->SrcCapExt.byte, 25,
			port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
			SOP_TYPE_SOP, AW_TRUE);
#endif

	PolicySendCommand(port, CMTNotSupported,
		port->PolicyIsSource ? peSourceReady : peSinkReady, 0,
		port->ProtocolMsgRxSop);
}

#endif /* AW_HAVE_EXT */

/* BIST Receive Mode */
void policyBISTReceiveMode(Port_t *port)
{
	/* Not Implemented */
	/* Tell protocol layer to go to BIST Receive Mode */
	/* Go to BIST_Frame_Received if a test frame is received */
	/* Transition to SRC_Transition_to_Default, SNK_Transition_to_Default, */
	/* or CBL_Ready when Hard_Reset received */
}

void policyBISTFrameReceived(Port_t *port)
{
	/* Not Implemented */
	/* Consume BIST Transmit Test Frame if received */
	/* Transition back to BIST_Frame_Received when a BIST Test Frame */
	/* has been received. Transition to SRC_Transition_to_Default, */
	/* SNK_Transition_to_Default, or CBL_Ready when Hard_Reset received */
}

/* BIST Carrier Mode and Eye Pattern */
void policyBISTCarrierMode2(Port_t *port)
{
	AW_U8 tx_enable = 0;

	switch (port->PolicySubIndex) {
	case 0:
		/* Tell protocol layer to go to BIST_Carrier_Mode_2 */
		port->Registers.Control.BIST_MODE2 = 1;
		DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);

		/* Set the bit to enable the transmitter */
		port->Registers.Control.TX_START = 1;
		DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
		port->Registers.Control.TX_START = 0;

		TimerStart(&port->PolicyStateTimer, tBISTContMode);

		port->PolicySubIndex++;
		break;
	case 1:
		/* Disable and wait on GoodCRC */
		if (TimerExpired(&port->PolicyStateTimer)) {

			/* Turn off tx BCM */
			tx_enable = port->Registers.Switches.byte[1] & 0xfc;
			DeviceWrite(port, regSwitches1, 1, &tx_enable);

			/* Disable BIST_Carrier_Mode_2 (PD_RESET does not do this) */
			port->Registers.Control.BIST_MODE2 = 0;
			DeviceWrite(port, regControl1, 1, &port->Registers.Control.byte[1]);

			/* Delay to allow preamble to finish */
			TimerStart(&port->PolicyStateTimer, tGoodCRCDelay);

			port->PolicySubIndex++;
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	case 2:
		/* Transition to SRC_Transition_to_Default, SNK_Transition_to_Default,
		 * or CBL_Ready when BISTContModeTimer times out
		 */
		if (TimerExpired(&port->PolicyStateTimer)) {
			ProtocolFlushTxFIFO(port);

			if (port->PolicyIsSource) {
#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
				SetPEState(port, peSourceReady);
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */
			} else {
#ifdef AW_HAVE_SNK
				SetPEState(port, peSinkReady);
#endif /* AW_HAVE_SNK */
			}
		} else {
			port->PEIdle = AW_TRUE;
		}
		break;
	default:
		SetPEState(port, peErrorRecovery);
		break;
	}
}

void policyBISTTestData(Port_t *port)
{
	if (port->SINK_Flag == 1) {
		port->SINK_num--;
		if (port->SINK_num == 11990) {
			AW_LOG("enter sink bist dis int\n");
			port->Registers.Mask.M_BC_LVL = 1;
			port->Registers.Mask.M_CRC_CHK = 1;
			port->Registers.Mask.M_ACTIVITY = 1;
			DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

			port->Registers.MaskAdv.M_GCRCSENT = 1;
			DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);
		}
	}

	if (port->SOURCE_Flag == 1) {
		port->SOURCE_num--;
		if (port->SOURCE_num == 11990) {
			AW_LOG("enter source bist dis int\n");
			port->Registers.Mask.M_BC_LVL = 1;
			port->Registers.Mask.M_CRC_CHK = 1;
			port->Registers.Mask.M_ACTIVITY = 1;
			DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

			port->Registers.MaskAdv.M_GCRCSENT = 1;
			DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);
		}
	}

	port->PEIdle = AW_TRUE;
}

#ifdef AW_HAVE_VDM
void InitializeVdmManager(Port_t *port)
{
	initializeVdm(port);

	/* configure callbacks */
	port->vdmm.req_id_info = &vdmRequestIdentityInfo;
	port->vdmm.req_svid_info = &vdmRequestSvidInfo;
	port->vdmm.req_modes_info = &vdmRequestModesInfo;
	port->vdmm.enter_mode_result = &vdmEnterModeResult;
	port->vdmm.exit_mode_result = &vdmExitModeResult;
	port->vdmm.inform_id = &vdmInformIdentity;
	port->vdmm.inform_svids = &vdmInformSvids;
	port->vdmm.inform_modes = &vdmInformModes;
	port->vdmm.inform_attention = &vdmInformAttention;
	port->vdmm.req_mode_entry = &vdmModeEntryRequest;
	port->vdmm.req_mode_exit = &vdmModeExitRequest;
}

void convertAndProcessVdmMessage(Port_t *port, SopType sop)
{
	/* Form the word arrays that VDM block expects */
	AW_U32 i;
	AW_U32 vdm_arr[7];

	for (i = 0; i < port->PolicyRxHeader.NumDataObjects; i++)
		vdm_arr[i] = port->PolicyRxDataObj[i].object;

	processVdmMessage(port, sop, vdm_arr, port->PolicyRxHeader.NumDataObjects);
}

void sendVdmMessage(Port_t *port, SopType sop, AW_U32 *arr, AW_U32 length, PolicyState_t next_ps)
{
	AW_U32 i;

	/* 'cast' to type that PolicySendData expects */
	/* didn't think this was necessary, but it fixed some problems. - Gabe */
	port->vdm_msg_length = length;
	port->vdm_next_ps = next_ps;
	for (i = 0; i < port->vdm_msg_length; i++)
		port->vdm_msg_obj[i].object = arr[i];
	port->VdmMsgTxSop = sop;
	port->sendingVdmData = AW_TRUE;
	port->VdmTimerStarted = AW_FALSE;
	SetPEState(port, peGiveVdm);
}

void doVdmCommand(Port_t *port)
{
	AW_U32 command;
	AW_U32 svid;
	AW_U32 mode_index;
	SopType sop;

	if (port->PDTransmitObjects[0].UVDM.VDMType == 0) {
		/* Transmit unstructured VDM messages directly from the GUI/DPM */
		PolicySendData(port, DMTVenderDefined,
					port->PDTransmitObjects,
					port->PDTransmitHeader.NumDataObjects *
						sizeof(doDataObject_t),
					port->PolicyIsSource == AW_TRUE ? peSourceReady:peSinkReady,
					0, port->PolicyMsgTxSop, AW_FALSE);
		return;
	}

	command = port->PDTransmitObjects[0].byte[0] & 0x1F;
	svid = 0;
	svid |= (port->PDTransmitObjects[0].byte[3] << 8);
	svid |= (port->PDTransmitObjects[0].byte[2] << 0);

	mode_index = 0;
	mode_index = port->PDTransmitObjects[0].byte[1] & 0x7;

	/* PolicyMsgTxSop must be set with correct type by calling */
	/* routine when setting USBPDTxFlag */

	sop = port->PolicyMsgTxSop;

#ifdef AW_HAVE_DP
	if (svid == DP_SID) {
		if (command == DP_COMMAND_STATUS) {
			DP_RequestPartnerStatus(port);
		} else if (command == DP_COMMAND_CONFIG) {
			DisplayPortConfig_t temp;

			temp.word = port->PDTransmitObjects[1].object;
			DP_RequestPartnerConfig(port, temp);
		}
	}
#endif /* AW_HAVE_DP */

	if (command == DISCOVER_IDENTITY)
		requestDiscoverIdentity(port, sop);
	else if (command == DISCOVER_SVIDS)
		requestDiscoverSvids(port, sop);
	else if (command == DISCOVER_MODES)
		requestDiscoverModes(port, sop, svid);
	else if (command == ENTER_MODE)
		requestEnterMode(port, sop, svid, mode_index);
	else if (command == EXIT_MODE)
		requestExitMode(port, sop, svid, mode_index);
}

void processCableResetState(Port_t *port)
{
	switch (port->cblRstState) {
	case CBL_RST_START:
		if (port->IsVCONNSource) {
			port->cblRstState = CBL_RST_VCONN_SOURCE;
		} else if (port->PolicyIsDFP) {
			port->cblRstState = CBL_RST_DR_DFP;
		} else {
			/* Must be dfp and vconn source. Start with VCONN Swap */
			SetPEState(port, port->PolicyIsSource ?
					peSourceSendVCONNSwap : peSinkSendVCONNSwap);
			port->cblRstState = CBL_RST_VCONN_SOURCE;
		}
		break;
	case CBL_RST_VCONN_SOURCE:
		if (port->IsVCONNSource) {
			if (port->PolicyIsDFP) {
				port->cblRstState = CBL_RST_SEND;
			} else {
				/* Must be dfp and vconn source*/
				SetPEState(port, port->PolicyState ?
						peSourceSendDRSwap : peSinkSendDRSwap);
				port->cblRstState = CBL_RST_DR_DFP;
			}
		} else {
			/* VCONN Swap might have failed */
			port->cblRstState = CBL_RST_DISABLED;
		}
		break;
	case CBL_RST_DR_DFP:
		if (port->PolicyIsDFP) {
			if (port->IsVCONNSource) {
				port->cblRstState = CBL_RST_SEND;
			} else {
				/* Must be dfp and vconn source*/
				SetPEState(port, port->PolicyIsSource ?
						peSourceSendVCONNSwap : peSinkSendVCONNSwap);
				port->cblRstState = CBL_RST_VCONN_SOURCE;
			}
		} else {
			port->cblRstState = CBL_RST_DISABLED;
		}
		break;
	case CBL_RST_SEND:
		if (port->PolicyIsDFP && port->IsVCONNSource)
			SetPEState(port, peSendCableReset);
		else
			port->cblRstState = CBL_RST_DISABLED;
		break;
	case CBL_RST_DISABLED:
	default:
		break;
	}
}

/* This function assumes we're already in either Source or Sink Ready states! */
void autoVdmDiscovery(Port_t *port)
{
	/* Wait for protocol layer to become idle */
	if (port->PDTxStatus == txIdle) {
		switch (port->AutoVdmState) {
		case AUTO_VDM_INIT:
		case AUTO_VDM_DISCOVER_ID_PP:
			requestDiscoverIdentity(port, SOP_TYPE_SOP);
			port->AutoVdmState = AUTO_VDM_DISCOVER_SVIDS_PP;
			break;
		case AUTO_VDM_DISCOVER_SVIDS_PP:
			if (port->svid_discvry_done == AW_FALSE)
				requestDiscoverSvids(port, SOP_TYPE_SOP);
			else
				port->AutoVdmState = AUTO_VDM_DISCOVER_MODES_PP;
			break;
		case AUTO_VDM_DISCOVER_MODES_PP:
			if (port->svid_discvry_idx >= 0) {
				requestDiscoverModes(port, SOP_TYPE_SOP,
					port->core_svid_info.svids[port->svid_discvry_idx]);
				port->AutoVdmState = AUTO_VDM_ENTER_MODE_PP;
			} else {
				/* No known SVIDs found */
				port->AutoVdmState = AUTO_VDM_DONE;
			}
			break;
		case AUTO_VDM_ENTER_MODE_PP:
			if (port->AutoModeEntryObjPos > 0) {
				requestEnterMode(
						port, SOP_TYPE_SOP,
						port->core_svid_info.svids[port->svid_discvry_idx],
						port->AutoModeEntryObjPos);
#ifdef AW_HAVE_DP
				if (port->core_svid_info.svids[port->svid_discvry_idx] == DP_SID) {
					port->AutoVdmState = AUTO_VDM_DP_GET_STATUS;
					break;
				}
#endif /* AW_HAVE_DP */
				port->AutoVdmState = AUTO_VDM_DP_GET_STATUS;
			}
			port->AutoVdmState = AUTO_VDM_DONE;
			break;
#ifdef AW_HAVE_DP
		case AUTO_VDM_DP_GET_STATUS:
			if (port->DisplayPortData.DpModeEntered > 0) {
				DP_RequestPartnerStatus(port);
				port->AutoVdmState = AUTO_VDM_DP_SET_CONFIG;
			} else {
				port->AutoVdmState = AUTO_VDM_DONE;
			}
			break;
		case AUTO_VDM_DP_SET_CONFIG:
			if (port->DisplayPortData.DpPpStatus.Connection == DP_MODE_BOTH) {
				/* If both connected send status with only one connected */
				DP_SetPortMode(port, (DisplayPort_Preferred_Snk) ?
						DP_CONF_UFP_D : DP_CONF_DFP_D);
				port->AutoVdmState = AUTO_VDM_DP_GET_STATUS;
			} else if (port->DisplayPortData.DpCapMatched &&
					 port->DisplayPortData.DpPpStatus.Connection > 0) {
				DP_RequestPartnerConfig(port, port->DisplayPortData.DpPpConfig);
				port->AutoVdmState = AUTO_VDM_DONE;
			} else {
				port->AutoVdmState = AUTO_VDM_DONE;
			}
			break;
#endif /* AW_HAVE_DP */
		default:
			port->AutoVdmState = AUTO_VDM_DONE;
			break;
		}
	}
}

#endif /* AW_HAVE_VDM */

/* This function is AW35615 specific */
SopType TokenToSopType(AW_U8 data)
{
	SopType ret;
	AW_U8 maskedSop = data & 0xE0;

	if (maskedSop == SOP_CODE_SOP)
		ret = SOP_TYPE_SOP;
	else if (maskedSop == SOP_CODE_SOP1)
		ret = SOP_TYPE_SOP1;
	else if (maskedSop == SOP_CODE_SOP2)
		ret = SOP_TYPE_SOP2;
	else if (maskedSop == SOP_CODE_SOP1_DEBUG)
		ret = SOP_TYPE_SOP2_DEBUG;
	else if (maskedSop == SOP_CODE_SOP2_DEBUG)
		ret = SOP_TYPE_SOP2_DEBUG;
	else
		ret = SOP_TYPE_ERROR;

	return ret;
}

void processDMTBIST(Port_t *port)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	AW_U8 bdo = port->PolicyRxDataObj[0].byte[3] >> 4;

	if ((DPM_GetSourceCapHeader(port->dpm, port) == NULL) ||
			(DPM_GetSourceCap(port->dpm, port) == NULL))
		return;

	notify_observers(BIST_ENABLED, port->I2cAddr, 0);

	switch (bdo) {
	case BDO_BIST_Carrier_Mode_2:
		/* Only enter BIST for 5V contract */
		if (DPM_GetSourceCap(port->dpm, port)[
				port->USBPDContract.FVRDO.ObjectPosition - 1].FPDOSupply.Voltage
				== 100) {
			SetPEState(port, PE_BIST_Carrier_Mode_2);
			port->ProtocolState = PRLIdle;
		}

		/* Don't idle here - mode setup in next state */
		break;
	case BDO_BIST_Test_Data:
	default:
		/* Only enter BIST for 5V contract */
		if (DPM_GetSourceCap(port->dpm, port)[
				port->USBPDContract.FVRDO.ObjectPosition - 1].FPDOSupply.Voltage
				== 100) {
			/* Auto-flush RxFIFO */
			port->Registers.Control.BIST_TMODE = 1;
			DeviceWrite(port, regControl3, 1,
					&port->Registers.Control.byte[3]);

			port->SINK_Flag = 0;
			port->SOURCE_Flag = 0;

			if (port->PolicyIsSource == 0) {
				port->SINK_Flag = 1;
				port->SINK_num = 12000;
				if (port->SOURCE_Flag_end) {
					AW_LOG("source_end_timer = %d\n", chip->source_end_timer);
					schedule_delayed_work(&chip->bist_delay_work, msecs_to_jiffies(chip->source_end_timer));
				} else {
					AW_LOG("sink_timer = %d\n", chip->sink_timer);
					schedule_delayed_work(&chip->bist_delay_work, msecs_to_jiffies(chip->sink_timer));
				}
			}

			if (port->PolicyIsSource == 1) {
				port->SINK_Flag = 0;
				port->SOURCE_Flag = 1;
				port->SOURCE_num = 12000;
				port->Registers.Slice.byte = 0x60;
				DeviceWrite(port, regSlice, 1, &port->Registers.Slice.byte);
				AW_LOG("source_timer = %d\n", chip->source_timer);
				schedule_delayed_work(&chip->bist_delay_work, msecs_to_jiffies(chip->source_timer));
			}

			SetPEState(port, PE_BIST_Test_Data);
			/* Disable Protocol layer so we don't read FIFO */
			port->ProtocolState = PRLDisabled;
		}

		port->PEIdle = AW_TRUE;
		break;
	}
}
