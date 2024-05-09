// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author		: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#include "platform_helpers.h"
#include "vendor_info.h"
#include "aw35615_driver.h"
#include "TypeC_Types.h"
#include "TypeC.h"
#include "PDPolicy.h"
#include "PDProtocol.h"

void StateMachineTypeC(Port_t *port)
{
	do {
		if (!port->SMEnabled)
			return;

		port->TCIdle = AW_FALSE;

		if (platform_get_device_irq_state(port->PortID)) {
			/* Read the interrupta, interruptb, status0, status1 and*/
			/* interrupt registers.*/
			DeviceRead(port, regInterrupta, 5, &port->Registers.Status.byte[2]);
		}

		if (port->USBPDActive) {
			port->PEIdle = AW_FALSE;

			/* Protocol operations */
			USBPDProtocol(port);

			/* Policy Engine operations */
			USBPDPolicyEngine(port);

#ifdef AW_HAVE_EXT_MSG
		/* Extended messaging may require additional chunk handling */
		/* before idling. */
		if (port->ExtTxOrRx != NoXfer)
			port->PEIdle = AW_FALSE;/* Don't allow system to idle */
#endif /* AW_HAVE_EXT_MSG */
		}

		if (port->ConnState != port->old_ConnState) {
			AW_LOG("port->ConnState = %d\n", port->ConnState);
			port->old_ConnState = port->ConnState;
		}

		switch (port->ConnState) {
		case Disabled:
			StateMachineDisabled(port);
			break;
		case ErrorRecovery:
			StateMachineErrorRecovery(port);
			break;
		case Unattached:
			StateMachineUnattached(port);
			break;
#ifdef AW_HAVE_SNK
		case AttachWaitSink:
			StateMachineAttachWaitSink(port);
			break;
		case AttachedSink:
			StateMachineAttachedSink(port);
			break;
#ifdef AW_HAVE_DRP
		case TryWaitSink:
			StateMachineTryWaitSink(port);
			break;
#endif /* AW_HAVE_DRP */
#endif /* AW_HAVE_SNK */
#if (defined(AW_HAVE_DRP) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
		case TrySink:
			StateMachineTrySink(port);
			break;
#endif /* AW_HAVE_DRP || (AW_HAVE_SNK && AW_HAVE_ACCMODE)) */
#ifdef AW_HAVE_SRC
		case AttachWaitSource:
			StateMachineAttachWaitSource(port);
			break;
		case AttachedSource:
			StateMachineAttachedSource(port);
			break;
#ifdef AW_HAVE_DRP
		case TryWaitSource:
			StateMachineTryWaitSource(port);
			break;
		case TrySource:
			StateMachineTrySource(port);
			break;
		case UnattachedSource:
			StateMachineUnattachedSource(port);
			break;
#endif /* AW_HAVE_DRP */
		case DebugAccessorySource:
			StateMachineDebugAccessorySource(port);
			break;
#endif /* AW_HAVE_SRC */
#ifdef AW_HAVE_ACCMODE
		case AudioAccessory:
			StateMachineAudioAccessory(port);
			break;
#ifdef AW_HAVE_SNK
		case DebugAccessorySink:
			StateMachineDebugAccessorySink(port);
			break;
#endif /* AW_HAVE_SNK */
#endif /* AW_HAVE_ACCMODE */
#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
		case AttachWaitAccessory:
			StateMachineAttachWaitAccessory(port);
			break;
		case UnsupportedAccessory:
			StateMachineUnsupportedAccessory(port);
			break;
		case PoweredAccessory:
			StateMachinePoweredAccessory(port);
			break;
#endif /* AW_HAVE_SNK && AW_HAVE_ACCMODE */
		case IllegalCable:
			StateMachineIllegalCable(port);
			break;
#ifdef AW_HAVE_VBUS_ONLY
		case AttachVbusOnlyok:
			StateMachineAttachedVbusOnly(port);
			break;
#endif /* AW_HAVE_VBUS_ONLY */
		default:
			SetStateUnattached(port);
			break;
		}

		/* Clear the interrupt registers after processing the state machines */
		port->Registers.Status.Interrupt1 = 0;
		port->Registers.Status.InterruptAdv = 0;

	} while (port->TCIdle == AW_FALSE || port->PEIdle == AW_FALSE);

	/* AW_LOG("exit\n"); */
}

void StateMachineDisabled(Port_t *port)
{
	SetStateDisabled(port);
}

void StateMachineErrorRecovery(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	if (TimerExpired(&port->StateTimer))
		SetStateUnattached(port);
}

void StateMachineUnattached(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	if (TimerExpired(&port->LoopCountTimer)) {
		/* Detached for ~100ms - safe to clear the loop counter */
		TimerDisable(&port->LoopCountTimer);
		port->loopCounter = 0;
	}

	if ((port->Registers.Status.I_TOGDONE == 0) && port->Registers.Status.VBUSOK) {
		if (port->wait_toggle_num--) {
			TimerStart(&port->LoopCountTimer, VbusTimeout);
			return;
		}
	} else {
		TimerDisable(&port->LoopCountTimer);
		port->wait_toggle_num = WAITTOGGLE;
	}

	if (port->Registers.Status.I_TOGDONE) {
#ifdef WATERPROOF
		/* waterproof function trigger event*/
		AW_LOG("waterproof function trigger event\n")
#endif
		//TimerDisable(&port->LoopCountTimer);
		DeviceRead(port, regStatus1a, 1, &port->Registers.Status.byte[1]);
		AW_LOG("regStatus1a=  0x%x\n", port->Registers.Status.TOGSS);

		switch (port->Registers.Status.TOGSS) {
#ifdef AW_HAVE_SNK
		case 0x5: /* Rp detected on CC1 */
			port->CCPin = CC1;
			SetStateAttachWaitSink(port);
			break;
		case 0x6: /* Rp detected on CC2 */
			port->CCPin = CC2;
			SetStateAttachWaitSink(port);
			break;
#endif /* AW_HAVE_SNK */
#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
		case 0x1: /* Rd detected on CC1 */
			port->CCPin = CCNone; /* Wait to re-check orientation */
#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
			if ((port->PortConfig.PortType == USBTypeC_Sink) &&
			((port->PortConfig.audioAccSupport) ||
			(port->PortConfig.poweredAccSupport)))
				SetStateAttachWaitAccessory(port);
#endif /* AW_HAVE_SNK && AW_HAVE_ACCMODE */
#if defined(AW_HAVE_SRC) && defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)
			else
#endif /* AW_HAVE_SRC && AW_HAVE_SNK && AW_HAVE_ACCMODE */
#ifdef AW_HAVE_SRC
				SetStateAttachWaitSource(port);
#endif /* AW_HAVE_SRC */
			break;
		case 0x2: /* Rd detected on CC2 */
			port->CCPin = CCNone; /* Wait to re-check orientation */
#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
			if ((port->PortConfig.PortType == USBTypeC_Sink) &&
			((port->PortConfig.audioAccSupport) ||
			(port->PortConfig.poweredAccSupport)))
				SetStateAttachWaitAccessory(port);
#endif /* AW_HAVE_SNK && AW_HAVE_ACCMODE */
#if defined(AW_HAVE_SRC) && defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)
			else
#endif /* AW_HAVE_SRC && AW_HAVE_SNK && AW_HAVE_ACCMODE */
#ifdef AW_HAVE_SRC
				SetStateAttachWaitSource(port);
#endif /* AW_HAVE_SRC */
			break;
		case 0x7: /* Ra detected on both CC1 and CC2 */
			port->CCPin = CCNone;
#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
			if ((port->PortConfig.PortType == USBTypeC_Sink) &&
			((port->PortConfig.audioAccSupport) ||
			(port->PortConfig.poweredAccSupport)))
				SetStateAttachWaitAccessory(port);
#endif /* defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE) */
#if defined(AW_HAVE_SRC) && defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)
			else
#endif /* AW_HAVE_SRC && AW_HAVE_SNK && AW_HAVE_ACCMODE */
#if defined(AW_HAVE_SRC)
				if (port->PortConfig.PortType != USBTypeC_Sink)
					SetStateAttachWaitSource(port);
#endif /* AW_HAVE_SRC */
			break;
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */
		default:
			/* Shouldn't get here, but just in case reset everything... */
			port->Registers.Control.TOGGLE = 0;
			DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);
			platform_delay_10us(1);
			/* Re-enable the toggle state machine... (allows us to get */
			/* another I_TOGDONE interrupt) */
			port->Registers.Control.TOGGLE = 1;
			DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);
			break;
		}
#ifdef AW_HAVE_VBUS_ONLY
	} else if (port->Registers.Status.VBUSOK) {
		/* check slow discharge or noise on vbus */
		port->Registers.Power.POWER = 0x7;
		DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);
		if (isVSafe5V(port) && (port->vbus_flag_last == VBUS_DIS)) {
			port->Registers.Switches.byte[0] = 0x03;
			DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

			/* Set up VBus measure interrupt to watch for detach */
			port->Registers.Measure.MEAS_VBUS = 1;
			port->Registers.Measure.MDAC = (port->DetachThreshold / MDAC_MV_LSB) - 1;
			port->Registers.Mask.M_COMP_CHNG = 0;
			port->Registers.Mask.M_BC_LVL = 0;
			DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
			DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

			/* Disable the Toggle functionality */
			port->Registers.Control.TOGGLE = 0;
			DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

			notify_observers(VBUS_OK, port->I2cAddr, NULL);
			port->debounce_vbus = VBUSDEBOUNCE;
			AW_LOG("VBUS_OK\n");
			//port->vbus_flag_last = VBUS_OK;
			SetTypeCState(port, AttachVbusOnlyok);
		} else {
			DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);
			SetStateUnattached(port);
		}
#endif /* AW_HAVE_VBUS_ONLY */
	}

	/* reset Source Caps & Header */
	port->src_cap_header.word = 0;
	port->src_cap_header.NumDataObjects = port->src_pdo_size;
	port->src_cap_header.MessageType = DMTSourceCapabilities;
	port->src_cap_header.SpecRevision   = port->PortConfig.PdRevPreferred;

	VIF_InitializeSrcCaps(port->src_caps);

	/* reset Sink Caps & Header */
	port->snk_cap_header.word = 0;
	port->snk_cap_header.NumDataObjects = port->snk_pdo_size;
	port->snk_cap_header.MessageType = DMTSinkCapabilities;
	port->snk_cap_header.SpecRevision   = port->PortConfig.PdRevPreferred;

	VIF_InitializeSnkCaps(port->snk_caps);

	/* AW_LOG("exit\n"); */
}

#ifdef AW_HAVE_SNK
void StateMachineAttachWaitSink(Port_t *port)
{
	/* AW_LOG("enter\n"); */

	port->TCIdle = AW_TRUE;

	debounceCC(port);

	if (port->Registers.Status.ACTIVITY == 1)
		return; /* PD Traffic will prevent correct use of BC_LVL during debouncing */

	/* AW_LOG("CCTermPDDebounce = %d\n", port->CCTermPDDebounce); */
	/* Look for an open termination for > tPDDebounce. */
	if (port->CCTermPDDebounce == CCTypeOpen) {
		/* PDDebounce Expired means the selected pin is open. Check other CC. */
		ToggleMeasure(port);
		port->CCPin = (port->CCPin == CC1) ? CC2 : CC1;
		port->CCTerm = DecodeCCTerminationSink(port);
		if (port->CCTerm == CCTypeOpen) {
			/* Other pin is open as well - detach. */
#ifdef AW_HAVE_DRP
			if (port->PortConfig.PortType == USBTypeC_DRP) {
				SetStateUnattachedSource(port);
			} else
#endif
			{
				SetStateUnattached(port);
			}
		} else {
			/* Other pin is attached.  Continue debouncing other pin. */
			TimerDisable(&port->PDDebounceTimer);
			port->CCTermPDDebounce = CCTypeUndefined;
		}
		return;
	}

	/* AW_LOG("CCTermCCDebounce = %d\n", port->CCTermCCDebounce); */
	/* CC Debounce the selected CC pin. */
	if (port->CCTermCCDebounce == CCTypeRdUSB) {
		updateVCONNSink(port);

		if (isVSafe5V(port)) {
			if ((port->VCONNTerm >= CCTypeRdUSB) && (port->VCONNTerm < CCTypeUndefined)) {
				/* Rp-Rp */
				if (Type_C_Is_Debug_Target_SNK)
					SetStateDebugAccessorySink(port);
			} else if (port->VCONNTerm == CCTypeOpen) {
				/* Rp-Open */
#ifdef AW_HAVE_DRP
				if ((port->PortConfig.PortType == USBTypeC_DRP) &&
					 port->PortConfig.SrcPreferred)
					SetStateTrySource(port);
				else
#endif
					SetStateAttachedSink(port);
			}
		} else {
			AW_LOG("wait vbus || bc_level, port->try_wait_vbus = %d\n", port->try_wait_vbus);
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			if (port->try_wait_vbus++ > 20)
				SetStateUnattached(port);
		}
	}

	/* AW_LOG("exit\n"); */
}
#endif

#ifdef AW_HAVE_SRC
void StateMachineAttachWaitSource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	AW_BOOL ccPinIsRa = AW_FALSE;

	port->TCIdle = AW_TRUE;

	/* Update source current - can only toggle with Default and may be using */
	/* 3A advertisement to prevent non-compliant cable looping. */

	if (port->Registers.Control.HOST_CUR != port->SourceCurrent &&
			TimerExpired(&port->StateTimer)) {
		TimerDisable(&port->StateTimer);
		updateSourceCurrent(port);
	}

	updateVCONNSource(port);
	ccPinIsRa = IsCCPinRa(port);

	/* Checking pins can cause extra COMP interrupts. */
	platform_delay_10us(12);
	DeviceRead(port, regInterrupt, 1, &port->Registers.Status.Interrupt1);

	debounceCC(port);

	if (port->CCTerm == CCTypeOpen) {
		if (port->VCONNTerm == CCTypeOpen || port->VCONNTerm == CCTypeRa) {
			SetStateUnattached(port);
		} else {
			/* CC pin may have switched (compliance test) - swap here */
			port->CCPin = (port->CCPin == CC1) ? CC2 : CC1;
			setStateSource(port, AW_FALSE);
		}
		return;
	}

	if (ccPinIsRa) {
		if (port->VCONNTerm >= CCTypeRdUSB && port->VCONNTerm < CCTypeUndefined) {
			/* The toggle state machine may have stopped on an Ra - swap here */
			port->CCPin = (port->CCPin == CC1) ? CC2 : CC1;
			setStateSource(port, AW_FALSE);
			return;
		}
#ifdef AW_HAVE_DRP
		else if (port->VCONNTerm == CCTypeOpen &&
				port->PortConfig.PortType == USBTypeC_DRP) {
			/* Dangling Ra cable - could still attach to Snk or Src Device. */
			/* Disconnect and keep looking for full connection. */
			/* Reset loopCounter to prevent landing in IllegalCable state. */

			port->loopCounter = 1;
			SetStateUnattached(port);
			return;
		}
#endif
	}

	/* Wait on CC Debounce for connection */
	if (port->CCTermCCDebounce != CCTypeUndefined) {
		if (ccPinIsRa) {
#ifdef AW_HAVE_ACCMODE
			if (port->PortConfig.audioAccSupport &&
			(port->VCONNTerm == CCTypeRa))
				SetStateAudioAccessory(port);
#endif
		} else if ((port->CCTermCCDebounce >= CCTypeRdUSB) &&
				(port->CCTermCCDebounce < CCTypeUndefined) &&
				(port->VCONNTerm >= CCTypeRdUSB) &&
				(port->VCONNTerm < CCTypeUndefined)) {
			/* Both pins Rd and Debug (DTS) mode supported */
			if (VbusVSafe0V(port)) {
				if (Type_C_Is_Debug_Target_SRC)
					SetStateDebugAccessorySource(port);
			} else {
				TimerStart(&port->VBusPollTimer, tVBusPollShort);
				TimerDisable(&port->StateTimer);
			}
		} else if ((port->CCTermCCDebounce >= CCTypeRdUSB) &&
				(port->CCTermCCDebounce < CCTypeUndefined) &&
				((port->VCONNTerm == CCTypeOpen) ||
				(port->VCONNTerm == CCTypeRa))) {
			/* One pin Rd */
			if (VbusVSafe0V(port)) {
#ifdef AW_HAVE_DRP
				if (port->PortConfig.SnkPreferred)
					SetStateTrySink(port);
				else
#endif
				{
					SetStateAttachedSource(port);
				}
			} else {
				TimerStart(&port->VBusPollTimer, tVBusPollShort);
			}
		} else {
			/* In the current configuration, we may be here with Ra-Open */
			/* (cable with nothing attached) so periodically poll for attach */
			/* or open or VBus present. */
			if (TimerDisabled(&port->StateTimer) || TimerExpired(&port->StateTimer))
				TimerStart(&port->StateTimer, tAttachWaitPoll);
		}
	}
}
#endif

#ifdef AW_HAVE_SNK
void StateMachineAttachedSink(Port_t *port)
{
	/* AW_LOG("enter\n"); */

	port->TCIdle = AW_TRUE;

	/* Monitor for VBus drop to detach */
	/* Round up detach threshold to help check slow discharge or noise on VBus */

	/* AW_LOG("I_COMP_CHNG %d\n", port->Registers.Status.I_COMP_CHNG); */
	if (port->Registers.Status.I_COMP_CHNG == 1) {
		if (!port->IsPRSwap && !port->IsHardReset &&
		!isVBUSOverVoltage(port, port->DetachThreshold)) {
			SetStateUnattached(port);
			return;
		}
	}

	if (!port->IsPRSwap)
		debounceCC(port);

	/* If using PD, sink can monitor CC as well as VBUS to allow detach */
	/* during a hard reset */

	if (port->USBPDActive && !port->IsPRSwap && port->CCTermPDDebounce == CCTypeOpen) {
		SetStateUnattached(port);
		return;
	}

	/* Update the advertised current */
	if (port->CCTermPDDebounce != CCTypeUndefined)
		UpdateSinkCurrent(port, port->CCTermPDDebounce);

	/* AW_LOG("exit\n"); */
}
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_VBUS_ONLY
void StateMachineAttachedVbusOnly(Port_t *port)
{
	/* AW_LOG("enter\n"); */

	port->TCIdle = AW_TRUE;

	/* Monitor for VBus drop to detach */
	/* Round up detach threshold to help check slow discharge or noise on VBus */
	AW_LOG("I_VBUSOK = %d VBUSOK = %d\n", port->Registers.Status.I_VBUSOK, port->Registers.Status.VBUSOK);
	if (!port->Registers.Status.VBUSOK) {
		if (port->vbus_flag_last == VBUS_OK && VbusVSafe0V(port)) {
			AW_LOG("SetStateUnattached for vbus\n");
			TimerDisable(&port->LoopCountTimer);
			SetStateUnattached(port);
		} else if (port->debounce_vbus--) {
			TimerStart(&port->LoopCountTimer, VbusTimeout);
		}
	}
}
#endif /* AW_HAVE_VBUS_ONLY */

#ifdef AW_HAVE_SRC
void StateMachineAttachedSource(Port_t *port)
{
	/* AW_LOG("enter TypeCSubState = %d\n", port->TypeCSubState); */

	port->TCIdle = AW_TRUE;

	switch (port->TypeCSubState) {
	case 0:
		if ((port->loopCounter != 0) || (port->Registers.Status.I_COMP_CHNG == 1))
			port->CCTerm = DecodeCCTerminationSource(port);

		if ((port->CCTerm == CCTypeOpen) && (!port->IsPRSwap)) {
			platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
			/* aw_vbus_path_disalbe(); */
			/* vbus discharge */
			port->Registers.Control5.VBUS_DIS_SEL = 1;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);

			notify_observers(CC_NO_ORIENT, port->I2cAddr, 0);

			USBPDDisable(port, AW_TRUE);
			/* VConn off and Pulldowns while detatching */
			port->Registers.Switches.byte[0] = 0x03;
			DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

			port->TypeCSubState++;

			TimerStart(&port->StateTimer, tSafe0V);
		} else if ((port->loopCounter != 0) && (TimerExpired(&port->StateTimer)
				|| port->PolicyHasContract)) {
			/* Valid attach, so reset loop counter and go Idle */
			TimerDisable(&port->StateTimer);
			port->loopCounter = 0;
			port->Registers.Mask.M_COMP_CHNG = 0;
			DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
		}
		break;
	case 1:
#ifdef AW_HAVE_DRP
		if ((port->PortConfig.PortType == USBTypeC_DRP) && port->PortConfig.SrcPreferred) {
			SetStateTryWaitSink(port);
			/* Disable VBus discharge after 10ms in TW.Snk */
			TimerStart(&port->StateTimer, tVBusPollShort);
			return;
		}
#endif /* AW_HAVE_DRP */

		if (VbusVSafe0V(port) || TimerExpired(&port->StateTimer))
			SetStateUnattached(port);
		else
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
		break;
	default:
		SetStateErrorRecovery(port);
		break;
	}
}
#endif /* AW_HAVE_SRC */

#ifdef AW_HAVE_DRP
void StateMachineTryWaitSink(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);

	if (port->CCTermPDDebounce == CCTypeOpen) {
		SetStateUnattached(port);
		return;
	}

	if (TimerExpired(&port->StateTimer)) {
		TimerDisable(&port->StateTimer);

		/* vbus discharge */
		port->Registers.Control5.VBUS_DIS_SEL = 0;
		DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
	}

	if (isVSafe5V(port)) {
		if ((port->CCTermCCDebounce > CCTypeOpen) &&
			(port->CCTermCCDebounce < CCTypeUndefined)) {
			/* vbus discharge */
			port->Registers.Control5.VBUS_DIS_SEL = 0;
			DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
			SetStateAttachedSink(port);
		}
	} else {
		TimerStart(&port->VBusPollTimer, tVBusPollShort);
	}
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_DRP
void StateMachineTrySource(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);

	if ((port->CCTermPDDebounce > CCTypeRa) &&
		(port->CCTermPDDebounce < CCTypeUndefined) &&
		((port->VCONNTerm == CCTypeOpen) ||
		(port->VCONNTerm == CCTypeRa))) {
		/* If the CC pin is Rd for at least tPDDebounce */
		SetStateAttachedSource(port);
	} else if (TimerExpired(&port->StateTimer)) {
		TimerDisable(&port->StateTimer);

		/* Move onto the TryWait.Snk state to not get stuck in here */
		SetStateTryWaitSink(port);
	}
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_SRC
void StateMachineDebugAccessorySource(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	updateVCONNSource(port);

	/* Checking pins can cause extra COMP interrupts. */
	platform_delay_10us(12);
	DeviceRead(port, regInterrupt, 1, &port->Registers.Status.Interrupt1);

	debounceCC(port);

	if (port->CCTerm == CCTypeOpen) {
		SetStateUnattached(port);
		return;
	} else if ((port->CCTermPDDebounce >= CCTypeRdUSB)
			&& (port->CCTermPDDebounce < CCTypeUndefined)
			&& (port->VCONNTerm >= CCTypeRdUSB)
			&& (port->VCONNTerm < CCTypeUndefined)
			&& port->USBPDActive == AW_FALSE) {
		if (port->CCTermPDDebounce > port->VCONNTerm) {
			port->CCPin = port->Registers.Switches.MEAS_CC1 ? CC1 :
						(port->Registers.Switches.MEAS_CC2 ? CC2 : CCNone);
			USBPDEnable(port, AW_TRUE, SOURCE);
		} else if (port->VCONNTerm > port->CCTermPDDebounce) {
			ToggleMeasure(port);
			port->CCPin = port->Registers.Switches.MEAS_CC1 ? CC1 :
						(port->Registers.Switches.MEAS_CC2 ? CC2 : CCNone);
			USBPDEnable(port, AW_TRUE, SOURCE);
		}
	}
}
#endif

#ifdef AW_HAVE_ACCMODE
void StateMachineAudioAccessory(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);

	/* Wait for a detach */
	if (port->CCTermCCDebounce == CCTypeOpen) {
		SetStateUnattached(port);
		notify_observers(CC_AUDIO_OPEN, port->I2cAddr, NULL);
	}
}
#endif /* AW_HAVE_ACCMODE */

#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
void StateMachineAttachWaitAccessory(Port_t *port)
{
	AW_BOOL ccIsRa = AW_FALSE;

	port->TCIdle = AW_TRUE;

	updateVCONNSource(port);
	ccIsRa = IsCCPinRa(port);

	/* Checking pins can cause extra COMP interrupts. */
	platform_delay_10us(12);
	DeviceRead(port, regInterrupt, 1, &port->Registers.Status.Interrupt1);

	debounceCC(port);

	if (ccIsRa && (port->VCONNTerm >= CCTypeRdUSB && port->VCONNTerm < CCTypeUndefined)) {
		/* The toggle state machine may have stopped on an Ra - swap here */
		port->CCPin = (port->CCPin == CC1) ? CC2 : CC1;
		setStateSource(port, AW_FALSE);
		port->TCIdle = AW_FALSE;
		return;
	} else if ((port->CCTerm == CCTypeOpen && port->VCONNTerm == CCTypeRa) ||
			(ccIsRa && port->VCONNTerm == CCTypeOpen)) {
		/* Dangling Ra cable - could still attach to Snk or Src Device. */
		/* Disconnect and keep looking for full connection. */
		/* Reset loopCounter to prevent landing in IllegalCable state. */
		port->loopCounter = 1;
		SetStateUnattached(port);
		return;
	} else if (((port->CCTerm >= CCTypeRdUSB) &&
			  (port->CCTerm < CCTypeUndefined) &&
			  (port->VCONNTerm == CCTypeOpen)) ||
			 ((port->VCONNTerm >= CCTypeRdUSB) &&
			  (port->VCONNTerm < CCTypeUndefined) &&
			  (port->CCTerm == CCTypeOpen))) {
		/* Rd-Open or Open-Rd shouldn't have generated an attach but */
		/* the aw35615 reports it anyway. */
		SetStateUnattached(port);
		return;
	}

	if (ccIsRa && (port->CCTermCCDebounce >= CCTypeRdUSB) &&
	(port->CCTermCCDebounce < CCTypeUndefined))
		port->CCTermCCDebounce = CCTypeRa;

	if (port->PortConfig.audioAccSupport &&
	port->CCTermCCDebounce == CCTypeRa &&
	port->VCONNTerm == CCTypeRa) {
		SetStateAudioAccessory(port);
	} else if (port->CCTermCCDebounce == CCTypeOpen) {
		SetStateUnattached(port);
	} else if (port->PortConfig.poweredAccSupport && (port->CCTermCCDebounce >= CCTypeRdUSB)
		&& (port->CCTermCCDebounce < CCTypeUndefined) && (port->VCONNTerm == CCTypeRa)) {
		SetStatePoweredAccessory(port);
	} else if ((port->CCTermCCDebounce >= CCTypeRdUSB) && (port->CCTermCCDebounce < CCTypeUndefined)
			&& (port->VCONNTerm == CCTypeOpen)) {
		SetStateTrySink(port);
	}
}

void StateMachinePoweredAccessory(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);

	if (port->CCTerm == CCTypeOpen)
		SetStateUnattached(port);
#ifdef AW_HAVE_VDM
	else if (port->mode_entered == AW_TRUE) {
		/* Disable tAMETimeout if we enter a mode */
		TimerDisable(&port->StateTimer);

		port->loopCounter = 0;

		if (port->PolicyState == peSourceReady) {
			/* Unmask COMP register to detect detach */
			port->Registers.Mask.M_COMP_CHNG = 0;
			DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
		}
	}
#endif /* AW_HAVE_VDM */
	else if (TimerExpired(&port->StateTimer)) {
		/* If we have timed out and haven't entered an alternate mode... */
		if (port->PolicyHasContract)
			SetStateUnsupportedAccessory(port);
		else
			SetStateTrySink(port);
	}
}

void StateMachineUnsupportedAccessory(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);

	if (port->CCTerm == CCTypeOpen)
		SetStateUnattached(port);
}
#endif /* (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE) */

#if (defined(AW_HAVE_DRP) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void StateMachineTrySink(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	switch (port->TypeCSubState) {
	case 0:
		if (TimerExpired(&port->StateTimer)) {
			TimerStart(&port->StateTimer, tDRPTryWait);
			port->TypeCSubState++;
		}
		break;
	case 1:
		debounceCC(port);

		if (port->Registers.Status.ACTIVITY == 1) {
			/* PD Traffic will prevent correct use of BC_LVL during debounce */
			return;
		}

		if (isVSafe5V(port)) {
			if (port->CCTermPDDebounce == CCTypeRdUSB)
				SetStateAttachedSink(port);
		}

#ifdef AW_HAVE_ACCMODE
		else if ((port->PortConfig.PortType == USBTypeC_Sink) && (TimerExpired(&port->StateTimer)
			|| TimerDisabled(&port->StateTimer)) && (port->CCTermPDDebounce == CCTypeOpen))
			SetStateUnsupportedAccessory(port);
#endif /* AW_HAVE_ACCMODE */

#ifdef AW_HAVE_DRP
		else if ((port->PortConfig.PortType == USBTypeC_DRP) && (port->CCTermPDDebounce == CCTypeOpen))
			SetStateTryWaitSource(port);

#endif /* AW_HAVE_DRP */
		else {
			AW_LOG("wait vbus || bc_level\n");
			TimerStart(&port->VBusPollTimer, tVBusPollShort);
			if (port->try_wait_vbus++ > 20)
				SetStateUnattached(port);
		}
		break;
	default:
		break;
	}

}
#endif /* AW_HAVE_DRP || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_DRP
void StateMachineTryWaitSource(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);
	updateVCONNSource(port);

	if (VbusVSafe0V(port) && (((port->CCTermPDDebounce >= CCTypeRdUSB) &&
			(port->CCTermPDDebounce < CCTypeUndefined)) &&
			((port->VCONNTerm == CCTypeRa) ||
			(port->VCONNTerm == CCTypeOpen)))) {
		AW_LOG("try source to source\n");
		SetStateAttachedSource(port);
	}
	/* After tTryTimeout transition to Unattached.SNK if Rd is not */
	/* detected on exactly one CC pin */
	else if (TimerExpired(&port->StateTimer) || TimerDisabled(&port->StateTimer)) {
		if (!((port->CCTerm >= CCTypeRdUSB) &&
		(port->CCTerm < CCTypeUndefined)) &&
		((port->VCONNTerm == CCTypeRa) ||
		(port->VCONNTerm == CCTypeOpen)))
			SetStateUnattached(port);
	} else {
		TimerStart(&port->VBusPollTimer, tVBusPollShort);
	}
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_DRP
void StateMachineUnattachedSource(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);
	updateVCONNSource(port);

	if ((port->CCTerm == CCTypeRa) && (port->VCONNTerm == CCTypeRa)) {
		SetStateAttachWaitSource(port);
	} else if ((port->CCTerm >= CCTypeRdUSB) && (port->CCTerm < CCTypeUndefined)
			&& ((port->VCONNTerm == CCTypeRa) || (port->VCONNTerm == CCTypeOpen))) {
		port->CCPin = CC1;
		SetStateAttachWaitSource(port);
	} else if ((port->VCONNTerm >= CCTypeRdUSB) && (port->VCONNTerm < CCTypeUndefined)
			&& ((port->CCTerm == CCTypeRa) || (port->CCTerm == CCTypeOpen))) {
		port->CCPin = CC2;
		SetStateAttachWaitSource(port);
	} else if (TimerExpired(&port->StateTimer)) {
		SetStateUnattached(port);
	}
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_SNK
void StateMachineDebugAccessorySink(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	debounceCC(port);
	updateVCONNSink(port);

	if (!isVBUSOverVoltage(port, port->DetachThreshold)) {
		SetStateUnattached(port);
	} else if ((port->CCTermPDDebounce >= CCTypeRdUSB)
			&& (port->CCTermPDDebounce < CCTypeUndefined)
			&& (port->VCONNTerm >= CCTypeRdUSB)
			&& (port->VCONNTerm < CCTypeUndefined)
			&& port->USBPDActive == AW_FALSE) {
		if (port->CCTermPDDebounce > port->VCONNTerm) {
			port->CCPin =
			port->Registers.Switches.MEAS_CC1 ? CC1 :
			(port->Registers.Switches.MEAS_CC2 ? CC2 : CCNone);
			USBPDEnable(port, AW_TRUE, SINK);
		} else if (port->VCONNTerm > port->CCTermPDDebounce) {
			ToggleMeasure(port);
			port->CCPin =
			port->Registers.Switches.MEAS_CC1 ? CC1 :
			(port->Registers.Switches.MEAS_CC2 ? CC2 : CCNone);
			USBPDEnable(port, AW_TRUE, SINK);
		}
	}
}
#endif /* AW_HAVE_SNK */

/* State Machine Configuration */
void SetStateDisabled(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	SetTypeCState(port, Disabled);
	TimerDisable(&port->StateTimer);

	clearState(port);
}

void SetStateErrorRecovery(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	SetTypeCState(port, ErrorRecovery);
	TimerStart(&port->StateTimer, tErrorRecovery);

	clearState(port);
}

void SetStateUnattached(Port_t *port)
{
	AW_U8 i = 0;

	for (i = 0; i < AW_NUM_TIMERS; ++i)
		TimerDisable(port->Timers[i]);

	port->Registers.Reset.SW_RES = 1;
	DeviceWrite(port, regReset, 1, &port->Registers.Reset.byte);

	ProtocolTrimOscFun(port);

	port->Registers.Control4.EN_PAR_CFG = 1;
	DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);

	/* vbus discharge */
	port->Registers.Control5.VBUS_DIS_SEL = 1;
	DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);

	/* Enable interrupt Pin */
	port->Registers.Control.INT_MASK = 0;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);

	/* These two control values allow detection of Ra-Ra or Ra-Open. */
	/* Enabling them will allow some corner case devices to connect where */
	/* they might not otherwise. */

	port->try_wait_vbus = 0;
	port->pd_state = AW_FALSE;
	port->Registers.Control.TOG_RD_ONLY = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);
	port->Registers.Control4.TOG_EXIT_AUD = 1; /* enable Ra Toggle only in cts test*/
	port->Registers.Control5.EN_PD3_MSG = 1;
	DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
	port->Registers.Switches.SPECREV = 1; /* enable good-crc specrev 2.0*/
	port->Registers.Switches.AUTO_CRC = 1;
	DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);

	port->TCIdle = AW_TRUE;

	SetTypeCState(port, Unattached);

	clearState(port);

	//port->Registers.Control.TOGGLE = 0;
	//DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

	if (port->snk_tog_time)
		DeviceWrite(port, regControl7, 1, &port->snk_tog_time);
	if (port->src_tog_time)
		DeviceWrite(port, regControl8, 1, &port->src_tog_time);

	port->Registers.Measure.MEAS_VBUS = 0;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	port->wait_toggle_num = WAITTOGGLE;
	port->vbus_flag_last = VBUS_DIS;
#ifdef AW_HAVE_VBUS_ONLY
	port->Registers.Mask.M_VBUSOK = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
#endif /* AW_HAVE_VBUS_ONLY */
	port->Registers.MaskAdv.M_TOGDONE = 0;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);

	/* Host current must be set to default for Toggle Functionality */
	if (port->Registers.Control.HOST_CUR != 0x1) {
		port->Registers.Control.HOST_CUR = 0x1;
		DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	}

	if (port->PortConfig.PortType == USBTypeC_DRP)
		port->Registers.Control.MODE = 0x1;/* DRP - Configure Rp/Rd toggling */
#ifdef AW_HAVE_ACCMODE
	else if ((port->PortConfig.PortType == USBTypeC_Sink) &&
			((port->PortConfig.audioAccSupport) ||
			(port->PortConfig.poweredAccSupport))) {
		/* Sink + Acc - Configure Rp/Rd toggling */
		port->Registers.Control.MODE = 0x1;
	}
#endif /* AW_HAVE_ACCMODE */
	else if (port->PortConfig.PortType == USBTypeC_Source) {
		/* Source - Look for Rd */
		port->Registers.Control.MODE = 0x3;
	} else {
		/* Sink - Look for Rp */
		port->Registers.Control.MODE = 0x2;
	}

	/* Delay before re-enabling toggle */
	platform_delay_10us(25);
	port->Registers.Control.TOGGLE = 1;
	DeviceWrite(port, regControl0, 3, &port->Registers.Control.byte[0]);

	port->Registers.Control5.VBUS_DIS_SEL = 0;
	DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);

	port->SrcCapsHeaderReceived.word = 0;
	port->SnkCapsHeaderReceived.word = 0;
	for (i = 0; i < 7; i++) {
		port->SrcCapsReceived[i].object = 0;
		port->SnkCapsReceived[i].object = 0;
	}

	/* Wait to clear the connect loop counter till detached for > ~100ms. */
	TimerStart(&port->LoopCountTimer, tLoopReset);
}

#ifdef AW_HAVE_SNK
void SetStateAttachWaitSink(Port_t *port)
{
	port->TCIdle = AW_FALSE;

	SetTypeCState(port, AttachWaitSink);

	/* Swap toggle state machine current if looping */
	if (port->loopCounter++ > MAX_CABLE_LOOP) {
		SetStateIllegalCable(port);
		return;
	}

	setStateSink(port);

	/* Disable the Toggle functionality */
	port->Registers.Control.TOGGLE = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

	/* Enable interrupts */
	port->Registers.Mask.M_ACTIVITY = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	/* Check for a possible C-to-A cable situation */
	if (isVSafe5V(port))
		port->C2ACable = AW_TRUE;

	/* Check for detach before continuing - AW35615-210*/
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);
	if (port->Registers.Status.BC_LVL == 0)
		SetStateUnattached(port);

	/* AW_LOG("exit\n"); */
}

void SetStateDebugAccessorySink(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;
	port->loopCounter = 0;

	SetTypeCState(port, DebugAccessorySink);
	setStateSink(port);

	port->Registers.Measure.MEAS_VBUS = 1;
	port->Registers.Measure.MDAC = (port->DetachThreshold / MDAC_MV_LSB) - 1;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	/* TODO RICK platform_double_56k_cable()? */
	notify_observers(CC1_AND_CC2, port->I2cAddr, NULL);

	TimerStart(&port->StateTimer, tOrientedDebug);
}
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_SRC
void SetStateAttachWaitSource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	SetTypeCState(port, AttachWaitSource);

	/* Swap toggle state machine current if looping */
	if (port->loopCounter++ > MAX_CABLE_LOOP) {
		SetStateIllegalCable(port);
		return;
	}

	/* Disabling the toggle bit here may cause brief pulldowns (they are the */
	/* default), so call setStateSource first to set pullups, */
	/* then disable the toggle bit, then re-run DetectCCPinSource to make sure */
	/* we have the correct pin selected. */
	setStateSource(port, AW_FALSE);

	/* To help prevent detection of a non-compliant cable, briefly set the */
	/* advertised current to 3A here.  It will be reset after tAttachWaitAdv */

	if (port->Registers.Control.HOST_CUR != 0x3) {
		port->Registers.Control.HOST_CUR = 0x3;
		DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	}
	updateSourceMDACHigh(port);

	/* Disable toggle */
	port->Registers.Control.TOGGLE = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

	/* Recheck for termination / orientation */
	DetectCCPinSource(port);
	setStateSource(port, AW_FALSE);

	/* Enable interrupts */
	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	/* After a delay, switch to the appropriate advertisement pullup */
	TimerStart(&port->StateTimer, tAttachWaitAdv);
}
#endif /* AW_HAVE_SRC */

#ifdef AW_HAVE_ACCMODE
void SetStateAttachWaitAccessory(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_FALSE;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
	SetTypeCState(port, AttachWaitAccessory);

	setStateSource(port, AW_FALSE);

	port->Registers.Control.TOGGLE = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

	/* Enable interrupts */
	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	TimerDisable(&port->StateTimer);
}
#endif /* AW_HAVE_ACCMODE */

#ifdef AW_HAVE_SRC
void SetStateAttachedSource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	SetTypeCState(port, AttachedSource);

	setStateSource(port, AW_TRUE);

	/* Enable 5V VBus */
	platform_set_pps_voltage(port->PortID, SET_VOUT_5000MV);

	notify_observers((port->CCPin == CC1) ? CC1_ORIENT : CC2_ORIENT, port->I2cAddr, 0);

	USBPDEnable(port, AW_TRUE, SOURCE);

	/* Start delay to check for illegal cable looping */
	TimerStart(&port->StateTimer, tIllegalCable);
}
#endif /* AW_HAVE_SRC */

#ifdef AW_HAVE_SNK
void SetStateAttachedSink(Port_t *port)
{
	port->TCIdle = AW_TRUE;

	/* Default to 5V detach threshold */
	port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;

	port->loopCounter = 0;

	SetTypeCState(port, AttachedSink);

	setStateSink(port);
	TimerDisable(&port->CCDebounceTimer);

	notify_observers((port->CCPin == CC1) ? CC1_ORIENT : CC2_ORIENT, port->I2cAddr, 0);

	port->CCTerm = DecodeCCTerminationSink(port);
	UpdateSinkCurrent(port, port->CCTerm);

	USBPDEnable(port, AW_TRUE, SINK);
	TimerDisable(&port->StateTimer);
}
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_DRP
void RoleSwapToAttachedSink(Port_t *port)
{
	SetTypeCState(port, AttachedSink);
	port->sourceOrSink = SINK;
	port->no_clear_message = AW_TRUE;

	/* Watch VBUS for sink disconnect */
	port->Registers.Measure.MEAS_VBUS = 1;
	port->Registers.Measure.MDAC = (port->DetachThreshold / MDAC_MV_LSB) - 1;
	port->Registers.Mask.M_COMP_CHNG = 0;
	port->Registers.Mask.M_VBUSOK = 1;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	port->SinkCurrent = utccNone;
	resetDebounceVariables(port);
	TimerDisable(&port->StateTimer);
	TimerDisable(&port->PDDebounceTimer);
	TimerStart(&port->CCDebounceTimer, tCCDebounce);
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_DRP
void RoleSwapToAttachedSource(Port_t *port)
{
	port->Registers.Measure.MEAS_VBUS = 0;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	platform_set_pps_voltage(port->PortID, SET_VOUT_5000MV);
	SetTypeCState(port, AttachedSource);
	port->sourceOrSink = SOURCE;
	resetDebounceVariables(port);
	updateSourceMDACHigh(port);

	/* Swap Pullup/Pulldown */
	port->Registers.Switches.PU_EN1 = 1;
	port->Registers.Switches.PDWN1 = 0;
	port->Registers.Switches.PU_EN2 = 1;
	port->Registers.Switches.PDWN2 = 0;
	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

	/* Enable comp change interrupt */
	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	notify_observers(POWER_ROLE, port->I2cAddr, NULL);

	port->no_clear_message = AW_TRUE;
	port->SinkCurrent = utccNone;
	TimerDisable(&port->StateTimer);
	TimerDisable(&port->PDDebounceTimer);
	TimerStart(&port->CCDebounceTimer, tCCDebounce);
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_DRP
void SetStateTryWaitSink(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_FALSE;

	/* Mask all */
	port->Registers.Mask.byte = 0xFF;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	port->Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

	USBPDDisable(port, AW_TRUE);

	SetTypeCState(port, TryWaitSink);

	setStateSink(port);

	TimerDisable(&port->StateTimer);
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_DRP
void SetStateTrySource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_FALSE;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
	SetTypeCState(port, TrySource);

	setStateSource(port, AW_FALSE);

	TimerStart(&port->StateTimer, tTryTimeout);
}
#endif /* AW_HAVE_DRP */

#if (defined(AW_HAVE_DRP) || \
	 (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void SetStateTrySink(Port_t *port)
{
	//AW_LOG("enter\n");
	port->TCIdle = AW_FALSE;

	SetTypeCState(port, TrySink);
	USBPDDisable(port, AW_TRUE);
	setStateSink(port);

	/* Enable interrupts */
	port->Registers.Mask.M_ACTIVITY = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	TimerStart(&port->StateTimer, tTryTimeout);
}
#endif /* AW_HAVE_DRP || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_DRP
void SetStateTryWaitSource(Port_t *port)
{
	/* AW_LOG("AW35615 -\n"); */
	port->TCIdle = AW_FALSE;

	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);

	SetTypeCState(port, TryWaitSource);

	setStateSource(port, AW_FALSE);

	TimerStart(&port->StateTimer, tTryTimeout);
}
#endif /* AW_HAVE_DRP */

#ifdef AW_HAVE_SRC
void SetStateDebugAccessorySource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->TCIdle = AW_FALSE;
	port->loopCounter = 0;

	platform_set_pps_voltage(port->PortID, SET_VOUT_5000MV);
	SetTypeCState(port, DebugAccessorySource);

	setStateSource(port, AW_FALSE);

	TimerStart(&port->StateTimer, tOrientedDebug);
}
#endif /* AW_HAVE_SRC */

#ifdef AW_HAVE_ACCMODE
void SetStateAudioAccessory(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_FALSE;

	port->loopCounter = 0;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
	SetTypeCState(port, AudioAccessory);

	setStateSource(port, AW_FALSE);

	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	notify_observers(CC_AUDIO_ORIENT, port->I2cAddr, NULL);

	TimerDisable(&port->StateTimer);
}
#endif /* AW_HAVE_ACCMODE */

#if (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE))
void SetStatePoweredAccessory(Port_t *port)
{
	/* AW_LOG("AW35615 -\n"); */
	port->TCIdle = AW_FALSE;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);

	SetTypeCState(port, PoweredAccessory);
	setStateSource(port, AW_TRUE);

	/* If the current is default set it to 1.5A advert (Must be 1.5 or 3.0) */
	if (port->Registers.Control.HOST_CUR != 0x2) {
		port->Registers.Control.HOST_CUR = 0x2;
		DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	}

	notify_observers((port->CCPin == CC1) ? CC1_ORIENT : CC2_ORIENT, port->I2cAddr, 0);

	USBPDEnable(port, AW_TRUE, SOURCE);

	TimerStart(&port->StateTimer, tAMETimeout);
}

void SetStateUnsupportedAccessory(Port_t *port)
{
	/* AW_LOG("AW35615 -\n"); */
	port->TCIdle = AW_TRUE;

	/* Mask for COMP */
	port->Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
	SetTypeCState(port, UnsupportedAccessory);
	setStateSource(port, AW_FALSE);

	/* Must advertise default current */
	port->Registers.Control.HOST_CUR = 0x1;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	USBPDDisable(port, AW_TRUE);

	TimerDisable(&port->StateTimer);

	notify_observers(ACC_UNSUPPORTED, port->I2cAddr, 0);
}
#endif /* (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)) */

#ifdef AW_HAVE_DRP
void SetStateUnattachedSource(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	/* Currently only implemented for AttachWaitSnk to Unattached for DRP */
	port->TCIdle = AW_FALSE;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);
	SetTypeCState(port, UnattachedSource);
	port->CCPin = CCNone;

	setStateSource(port, AW_FALSE);

	USBPDDisable(port, AW_TRUE);

	TimerStart(&port->StateTimer, tTOG2);
}
#endif /* AW_HAVE_DRP */

/* Type C Support Routines */
void updateSourceCurrent(Port_t *port)
{
	switch (port->SourceCurrent) {
	case utccDefault:
		/* Set the host current to reflect the default USB power */
		port->Registers.Control.HOST_CUR = 0x1;
		break;
	case utcc1p5A:
		/* Set the host current to reflect 1.5A */
		port->Registers.Control.HOST_CUR = 0x2;
		break;
	case utcc3p0A:
		/* Set the host current to reflect 3.0A */
		port->Registers.Control.HOST_CUR = 0x3;
		break;
	default:
		/* This assumes that there is no current being advertised */
		/* Set the host current to disabled */
		port->Registers.Control.HOST_CUR = 0x0;
		break;
	}
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
}

void updateSourceMDACHigh(Port_t *port)
{
	switch (port->Registers.Control.HOST_CUR) {
	case 0x1:
		/* Set up DAC threshold to 1.6V (default USB current advertisement) */
		port->Registers.Measure.MDAC = MDAC_1P596V;
		break;
	case 0x2:
		/* Set up DAC threshold to 1.6V */
		port->Registers.Measure.MDAC = MDAC_1P596V;
		break;
	case 0x3:
		/* Set up DAC threshold to 2.6V */
		port->Registers.Measure.MDAC = MDAC_2P604V;
		break;
	default:
		/* This assumes that there is no current being advertised */
		/* Set up DAC threshold to 1.6V (default USB current advertisement) */
		port->Registers.Measure.MDAC = MDAC_1P596V;
		break;
	}
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
}

void updateSourceMDACLow(Port_t *port)
{
	switch (port->Registers.Control.HOST_CUR) {
	case 0x1:
		/* Set up DAC threshold to 210mV (default USB current advertisement) */
		port->Registers.Measure.MDAC = MDAC_0P210V;
		break;
	case 0x2:
		/* Set up DAC threshold to 420mV */
		port->Registers.Measure.MDAC = MDAC_0P420V;
		break;
	case 0x3:
		/* Set up DAC threshold to 798mV */
		port->Registers.Measure.MDAC = MDAC_0P798V;
		break;
	default:
		/* This assumes that there is no current being advertised */
		/* Set up DAC threshold to 1596mV (default USB current advertisement) */
		port->Registers.Measure.MDAC = MDAC_1P596V;
		break;
	}
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
}

void ToggleMeasure(Port_t *port)
{
	/* Toggle measure block between CC pins */
	if (port->Registers.Switches.MEAS_CC2 == 1) {
		port->Registers.Switches.MEAS_CC1 = 1;
		port->Registers.Switches.MEAS_CC2 = 0;
	} else if (port->Registers.Switches.MEAS_CC1 == 1) {
		port->Registers.Switches.MEAS_CC1 = 0;
		port->Registers.Switches.MEAS_CC2 = 1;
	}

	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
}

CCTermType DecodeCCTermination(Port_t *port)
{
	switch (port->sourceOrSink) {
#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
	case SOURCE:
		return DecodeCCTerminationSource(port);
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */
#ifdef AW_HAVE_SNK
	case SINK:
		return DecodeCCTerminationSink(port);
#endif /* AW_HAVE_SNK */
	default:
		return CCTypeUndefined;
	}
}

#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
CCTermType DecodeCCTerminationSource(Port_t *port)
{
	CCTermType Termination = CCTypeUndefined;
	regMeasure_t saved_measure = port->Registers.Measure;

	/* Make sure MEAS_VBUS is cleared */
	if (port->Registers.Measure.MEAS_VBUS != 0) {
		port->Registers.Measure.MEAS_VBUS = 0;
		DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
	}

	/* Assume we are called with MDAC high. */
	/* Delay to allow measurement to settle */
	/* platform_delay_ms(3); */
	platform_delay_10us(100);
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);

	if (port->Registers.Status.COMP == 1) {
		/* Open voltage */
		Termination = CCTypeOpen;
		return Termination;
	} else if ((port->Registers.Switches.MEAS_CC1 && (port->CCPin == CC1)) ||
			(port->Registers.Switches.MEAS_CC2 && (port->CCPin == CC2))) {
		/* Optimization determines whether the pin is Open or Rd.  Ra level */
		/* is checked elsewhere.  This prevents additional changes to the MDAC */
		/* level which causes a continuous cycle of additional interrupts. */
		switch (port->Registers.Control.HOST_CUR) {
		case 0x1:
			Termination = CCTypeRdUSB;
			break;
		case 0x2:
			Termination = CCTypeRd1p5;
			break;
		case 0x3:
			Termination = CCTypeRd3p0;
			break;
		case 0x0:
			break;
		}
		return Termination;
	}

	/* Lower than open voltage - Rd or Ra */
	updateSourceMDACLow(port);

	/* Delay to allow measurement to settle */
	/* platform_delay_ms(1); */
	platform_delay_10us(100);
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);

	if (port->Registers.Status.COMP == 0) {
		/* Lower than Ra threshold is Ra */
		Termination = CCTypeRa;
	} else {
		/* Higher than Ra threshold is Rd */
		switch (port->Registers.Control.HOST_CUR) {
		case 0x1:
			Termination = CCTypeRdUSB;
			break;
		case 0x2:
			Termination = CCTypeRd1p5;
			break;
		case 0x3:
			Termination = CCTypeRd3p0;
			break;
		case 0x0:
			break;
		}
	}

	/* Restore Measure register */
	port->Registers.Measure = saved_measure;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	return Termination;
}

AW_BOOL IsCCPinRa(Port_t *port)
{
	AW_BOOL isRa = AW_FALSE;
	regMeasure_t saved_measure = port->Registers.Measure;

	/* Make sure MEAS_VBUS is cleared */
	port->Registers.Measure.MEAS_VBUS = 0;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	/* Lower than open voltage - Rd or Ra */
	updateSourceMDACLow(port);

	/* Delay to allow measurement to settle */
	platform_delay_10us(25);
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);

	isRa = (port->Registers.Status.COMP == 0) ? AW_TRUE : AW_FALSE;

	/* Restore Measure register */
	port->Registers.Measure = saved_measure;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	return isRa;
}
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE) */

#ifdef AW_HAVE_SNK
CCTermType DecodeCCTerminationSink(Port_t *port)
{
	CCTermType Termination;

	/* Delay to allow measurement to settle */
	platform_delay_10us(25);
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);

	/* Determine which level */
	/* AW_LOG("BC_LVL = %d\n", port->Registers.Status.BC_LVL); */
	switch (port->Registers.Status.BC_LVL) {
	case 0x0:
		/* If BC_LVL is lowest it's open */
		Termination = CCTypeOpen;
		break;
	case 0x1:
		/* If BC_LVL is 1, it's default */
		Termination = CCTypeRdUSB;
		break;
	case 0x2:
		/* If BC_LVL is 2, it's vRd1p5 */
		Termination = CCTypeRd1p5;
		break;
	default:
		/* Otherwise it's vRd3p0 */
		Termination = CCTypeRd3p0;
		break;
	}

	return Termination;
}
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_SNK
void UpdateSinkCurrent(Port_t *port, CCTermType term)
{
	switch (term) {
	case CCTypeRdUSB:
		/* If we detect the default... */
		port->SinkCurrent = utccDefault;
		break;
	case CCTypeRd1p5:
		/* If we detect 1.5A */
		port->SinkCurrent = utcc1p5A;
		break;
	case CCTypeRd3p0:
		/* If we detect 3.0A */
		port->SinkCurrent = utcc3p0A;
		break;
	default:
		port->SinkCurrent = utccNone;
		break;
	}
}
#endif /* AW_HAVE_SNK */

void UpdateCurrentAdvert(Port_t *port, USBTypeCCurrent Current)
{
	/* SourceCurrent value is of type USBTypeCCurrent */
	if (Current < utccInvalid) {
		port->SourceCurrent = Current;
		updateSourceCurrent(port);
	}
}

AW_BOOL VbusVSafe0V(Port_t *port)
{
	return (!isVBUSOverVoltage(port, 3200)) ? AW_TRUE : AW_FALSE;
}

AW_BOOL isVSafe5V(Port_t *port)
{
	return isVBUSOverVoltage(port, VBUS_MV_VSAFE5V_L);
}

AW_BOOL isVBUSOverVoltage(Port_t *port, AW_U16 vbus_mv)
{
	/* PPS Implementation requires better resolution vbus measurements than */
	/* the MDAC can provide.  If available on the platform, use an ADC */
	/* channel to measure the current vbus voltage. */

	regMeasure_t measure;

	AW_U8 val;
	AW_BOOL ret;
	AW_BOOL mdacUpdated = AW_FALSE;

	/* Setup for VBUS measurement */
	measure.byte = 0;
	measure.MEAS_VBUS = 1;
	measure.MDAC = vbus_mv / MDAC_MV_LSB;

	/* The actual value of MDAC is less by 1 */
	if (measure.MDAC > 0)
		measure.MDAC -= 1;

	if (port->Registers.Measure.byte != measure.byte) {
		/* Update only if required */
		DeviceWrite(port, regMeasure, 1, &measure.byte);
		mdacUpdated = AW_TRUE;
		/* Delay to allow measurement to settle */
		platform_delay_10us(300);
	}

	DeviceRead(port, regStatus0, 1, &val);
	/* COMP = bit 5 of status0 (Device specific?) */
	val &= 0x20;

	/* Determine return value based on COMP */
	ret = (val) ? AW_TRUE : AW_FALSE;

	if (mdacUpdated == AW_TRUE) {
		/* Restore register values */
		DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
	}

	return ret;
}

void DetectCCPinSource(Port_t *port)
{
	/* AW_LOG("enter\n"); */

	CCTermType CCTerm;
	AW_BOOL CC1IsRa = AW_FALSE;

	/* Enable CC pull-ups and CC1 measure */
	port->Registers.Switches.byte[0] = 0xC4;

	DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

	CCTerm = DecodeCCTermination(port);

	if ((CCTerm >= CCTypeRdUSB) && (CCTerm < CCTypeUndefined)) {
		port->CCPin = CC1;
		return;
	} else if (CCTerm == CCTypeRa) {
		CC1IsRa = AW_TRUE;
	}

	/* Enable CC pull-ups and CC2 measure */
	port->Registers.Switches.byte[0] = 0xC8;

	DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

	CCTerm = DecodeCCTermination(port);

	if ((CCTerm >= CCTypeRdUSB) && (CCTerm < CCTypeUndefined)) {
		port->CCPin = CC2;
		return;
	}

	/* Only Ra found... on CC1 or CC2? */
	/* This supports correct dangling Ra cable behavior. */

	port->CCPin = CC1IsRa ? CC1 : (CCTerm == CCTypeRa) ? CC2 : CCNone;
}

void DetectCCPinSink(Port_t *port)
{
	CCTermType CCTerm;

	port->Registers.Switches.byte[0] = 0x07;
	DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

	CCTerm = DecodeCCTermination(port);

	if ((CCTerm > CCTypeRa) && (CCTerm < CCTypeUndefined)) {
		port->CCPin = CC1;
		return;
	}

	port->Registers.Switches.byte[0] = 0x0B;
	DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

	CCTerm = DecodeCCTermination(port);

	if ((CCTerm > CCTypeRa) && (CCTerm < CCTypeUndefined)) {
		port->CCPin = CC2;
		return;
	}
}

void resetDebounceVariables(Port_t *port)
{
	port->CCTerm = CCTypeUndefined;
	port->CCTermCCDebounce = CCTypeUndefined;
	port->CCTermPDDebounce = CCTypeUndefined;
	port->CCTermPDDebouncePrevious = CCTypeUndefined;
	port->VCONNTerm = CCTypeUndefined;
}

void debounceCC(Port_t *port)
{
	/* The functionality here should work correctly using the Idle mode. */
	/* Will idling, a CC change or timer interrupt will */
	/* generate an appropriate update to the debounce state. */
	/* Grab the latest CC termination value */
	CCTermType CCTermCurrent = DecodeCCTermination(port);

	/* While debouncing to connect as a Sink, only care about one value for Rp. */
	/* When in AttachedSink state, debounce for sink sub-state. */
	if (port->sourceOrSink == SINK && port->ConnState != AttachedSink &&
	(CCTermCurrent == CCTypeRd1p5 || CCTermCurrent == CCTypeRd3p0))
		CCTermCurrent = CCTypeRdUSB;

	/* Check to see if the value has changed... */
	if (port->CCTerm != CCTermCurrent) {
		/* AW_LOG("CCTerm != CCTermCurrent\n"); */
		/* If it has, update the value */
		port->CCTerm = CCTermCurrent;

		/* Restart the debounce timer (wait 10ms before detach) */
		TimerStart(&port->PDDebounceTimer, tPDDebounce);
	}

	/* Check to see if our debounce timer has expired... */
	if (TimerExpired(&port->PDDebounceTimer)) {
		/* AW_LOG("PDDebounceTimer over\n"); */
		/* Update the CC debounced values */
		port->CCTermPDDebounce = port->CCTerm;
		TimerDisable(&port->PDDebounceTimer);
	}

	/* CC debounce */
	if (port->CCTermPDDebouncePrevious != port->CCTermPDDebounce) {
		/* AW_LOG("CCTermPDDebouncePrevious != port->CCTermPDDebounce\n"); */
		/* If the PDDebounce values have changed */
		/* Update the previous value */
		port->CCTermPDDebouncePrevious = port->CCTermPDDebounce;

		/* Reset the tCCDebounce timers */
		TimerStart(&port->CCDebounceTimer, tCCDebounce - tPDDebounce);

		/* Set CC debounce values to undefined while it is being debounced */
		port->CCTermCCDebounce = CCTypeUndefined;
	}

	if (TimerExpired(&port->CCDebounceTimer)) {
		/* AW_LOG("port->CCDebounceTimer over\n"); */
		/* Update the CC debounced values */
		port->CCTermCCDebounce = port->CCTermPDDebouncePrevious;
		TimerDisable(&port->CCDebounceTimer);
	}
}

#if (defined(AW_HAVE_SRC) || (defined(AW_HAVE_SNK) && defined(AW_HAVE_ACCMODE)))
void updateVCONNSource(Port_t *port)
{
	/* Assumes PUs have been set */

	/* Save current Switches */
	AW_U8 saveRegister = port->Registers.Switches.byte[0];

	/* Toggle measure to VCONN */
	ToggleMeasure(port);

	port->VCONNTerm = DecodeCCTermination(port);

	/* Restore Switches */
	port->Registers.Switches.byte[0] = saveRegister;
	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
}

void setStateSource(Port_t *port, AW_BOOL vconn)
{
	port->sourceOrSink = SOURCE;
	resetDebounceVariables(port);
	updateSourceCurrent(port);
	updateSourceMDACHigh(port);

#ifdef AW_HAVE_VBUS_ONLY
	/* Prevent false alarm of vbus-ok detection function in otg mode */
	port->Registers.Mask.M_VBUSOK = 0x1;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
#endif /* AW_HAVE_VBUS_ONLY */

	/* Enable everything except internal oscillator */
	port->Registers.Power.POWER = 0x7;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

	/* For automated testing */
	if (port->CCPin == CCNone)
		DetectCCPinSource(port);

	//AW_LOG("port->CCPin = %d\n", port->CCPin);

	if (port->CCPin == CC1) {
		/* Enable CC pull-ups and CC1 measure */
		port->Registers.Switches.byte[0] = 0xC4;
	} else {
		/* Enable CC pull-ups and CC2 measure */
		port->Registers.Switches.byte[0] = 0xC8;
	}
	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

	updateVCONNSource(port);

	/* Turn on VConn after checking the VConn termination */
	if (vconn && Type_C_Sources_VCONN) {
		if (port->CCPin == CC1)
			port->Registers.Switches.VCONN_CC2 = 1;
		else
			port->Registers.Switches.VCONN_CC1 = 1;

		DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
	}

	port->SinkCurrent = utccNone;

	TimerDisable(&port->PDDebounceTimer);
	TimerStart(&port->CCDebounceTimer, tCCDebounce);
}
#endif /* AW_HAVE_SRC || (AW_HAVE_SNK && AW_HAVE_ACCMODE)) */

#ifdef AW_HAVE_SNK
void setStateSink(Port_t *port)
{
	/* Disable the vbus outputs */
	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);

	port->sourceOrSink = SINK;
	resetDebounceVariables(port);

	/* Enable everything except internal oscillator */
	port->Registers.Power.POWER = 0x7;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

	/* For automated testing */
	if (port->CCPin == CCNone)
		DetectCCPinSink(port);

	if (port->CCPin == CC1) {
		/* If we detected CC1 as an Rp, enable Rd's on CC1 */
		port->Registers.Switches.byte[0] = 0x07;
	} else {
		/* If we detected CC2 as an Rp, enable Rd's on CC2 */
		port->Registers.Switches.byte[0] = 0x0B;
	}
	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

	/* Set up VBus measure interrupt to watch for detach */
	port->Registers.Measure.MEAS_VBUS = 1;
	port->Registers.Measure.MDAC = (port->DetachThreshold / MDAC_MV_LSB) - 1;
	port->Registers.Mask.M_COMP_CHNG = 0;
	port->Registers.Mask.M_BC_LVL = 0;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

	updateVCONNSink(port);

	TimerDisable(&port->PDDebounceTimer);
	TimerStart(&port->CCDebounceTimer, tCCDebounce);
}

void updateVCONNSink(Port_t *port)
{
	/* Assumes Rd has been set */
	ToggleMeasure(port);

	port->VCONNTerm = DecodeCCTermination(port);

	ToggleMeasure(port);
}

#endif /* AW_HAVE_SNK */

void clearState(Port_t *port)
{
	/* Disable the vbus outputs */
	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);

	USBPDDisable(port, AW_TRUE);

	/* Mask/disable interrupts */
	port->Registers.Mask.byte = 0xFF;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	port->Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

	port->Registers.Control.TOGGLE = 0; /* Disable toggling */
	port->Registers.Control.HOST_CUR = 0x0;	 /* Clear PU advertisement */
	DeviceWrite(port, regControl0, 3, &port->Registers.Control.byte[0]);

	port->Registers.Switches.byte[0] = 0x00;/* Disable PU, PD, etc. */
	DeviceWrite(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);

	SetConfiguredCurrent(port);
	resetDebounceVariables(port);
	port->CCPin = CCNone;

	TimerDisable(&port->PDDebounceTimer);
	TimerDisable(&port->CCDebounceTimer);

	notify_observers(CC_NO_ORIENT, port->I2cAddr, 0);

	port->Registers.Power.POWER = 0x1;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);
}

void SetStateIllegalCable(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	port->TCIdle = AW_TRUE;

	port->loopCounter = 0;

	platform_set_pps_voltage(port->PortID, SET_VOUT_0000MV);

	/* Disable toggle */
	port->Registers.Control.TOGGLE = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);

	SetTypeCState(port, IllegalCable);

	UpdateCurrentAdvert(port, utcc3p0A);

	/* This level (MDAC == 0x24) seems to be appropriate for 3.0A PU's */
	port->Registers.Measure.MDAC = MDAC_1P596V - 1;
	port->Registers.Measure.MEAS_VBUS = 0;
	DeviceWrite(port, regMeasure, 1, &port->Registers.Measure.byte);

	port->sourceOrSink = SOURCE;

	/* Enable everything except internal oscillator */
	port->Registers.Power.POWER = 0x7;
	DeviceWrite(port, regPower, 1, &port->Registers.Power.byte);

	/* Enable CC pull-ups and pull-downs and CC1 measure */
	port->Registers.Switches.byte[0] = 0xC7;
	DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

	port->CCPin = CC1;
	port->CCTerm = DecodeCCTermination(port);

	if ((port->CCTerm >= CCTypeRdUSB) && (port->CCTerm < CCTypeUndefined)) {
	} else {
		port->CCPin = CC2;

		/* Enable CC pull-ups and pull-downs and CC2 measure */
		port->Registers.Switches.byte[0] = 0xCB;
		DeviceWrite(port, regSwitches0, 1, &(port->Registers.Switches.byte[0]));

		port->CCTerm = DecodeCCTermination(port);
	}

	if ((port->CCTerm >= CCTypeRdUSB) && (port->CCTerm < CCTypeUndefined)) {
		port->Registers.Mask.M_COMP_CHNG = 0;
		port->Registers.Mask.M_BC_LVL = 0;
		DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);

		TimerDisable(&port->StateTimer);
	} else {
		/* Couldn't find an appropriate termination - detach and try again */
		SetStateUnattached(port);
	}
}

void StateMachineIllegalCable(Port_t *port)
{
	/* This state provides a stable landing point for dangling or illegal */
	/* cables.  These are either unplugged travel adapters or */
	/* some flavor of unplugged Type-C to Type-A cable (Rp to VBUS). */
	/* The combination of capacitance and PullUp/PullDown causes a repeated */
	/* cycle of attach-detach that could continue ad infinitum or until the */
	/* cable or travel adapter is plugged in.  This state breaks the loop and */
	/* waits for a change in termination. */
	/* NOTE: In most cases this requires VBUS bleed resistor (~7kohm) */

	port->TCIdle = AW_TRUE;

	if (port->Registers.Status.I_COMP_CHNG == 1) {
		port->CCTerm = DecodeCCTermination(port);
		if (port->CCTerm == CCTypeOpen)
			SetStateUnattached(port);
	}
}

