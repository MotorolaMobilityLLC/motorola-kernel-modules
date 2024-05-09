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
#include "vendor_info.h"
#include "Port.h"
#include "TypeC.h"
#include "PDPolicy.h"
#include "PDProtocol.h"
#include "platform_helpers.h"

#ifdef AW_HAVE_VDM
#include "vdm/vdm_callbacks.h"
#endif /* AW_HAVE_VDM */

void PortInit(Port_t *port)
{
	AW_U8 i;

	port->PortConfig.PdRevPreferred = PD_Specification_Revision;
	port->PdRevSop = port->PortConfig.PdRevPreferred;
	port->PdRevCable = port->PortConfig.PdRevPreferred;
	SetPortDefaultConfiguration(port);
	InitializeRegisters(port);
	InitializeTypeCVariables(port);
	InitializePDProtocolVariables(port);
	InitializePDPolicyVariables(port);

	/* Add timer objects to list to make timeout checking easier */
	port->Timers[0] = &port->PDDebounceTimer;
	port->Timers[1] = &port->CCDebounceTimer;
	port->Timers[2] = &port->StateTimer;
	port->Timers[3] = &port->LoopCountTimer;
	port->Timers[4] = &port->PolicyStateTimer;
	port->Timers[5] = &port->ProtocolTimer;
	port->Timers[6] = &port->SwapSourceStartTimer;
	port->Timers[7] = &port->PpsTimer;
	port->Timers[8] = &port->VBusPollTimer;
	port->Timers[9] = &port->VdmTimer;

	for (i = 0; i < AW_NUM_TIMERS; ++i)
		TimerDisable(port->Timers[i]);
}

void SetTypeCState(Port_t *port, ConnectionState state)
{
	port->ConnState = state;
	port->TypeCSubState = 0;
}

void SetPEState(Port_t *port, PolicyState_t state)
{
	port->PolicyState = state;
	port->PolicySubIndex = 0;

	port->PDTxStatus = txIdle;
	port->WaitingOnHR = AW_FALSE;
	port->WaitInSReady = AW_FALSE;
	if (port->PolicyState != port->old_pe_state) {
		port->old_pe_state = port->PolicyState;
		notify_observers(PD_STATE_CHANGED, port->I2cAddr, 0);
	}
}

/**
 * Initalize port policy variables to default. These are changed later by
 * policy manager.
 */
void SetPortDefaultConfiguration(Port_t *port)
{
#ifdef AW_HAVE_SNK
	port->PortConfig.SinkRequestMaxVoltage = 0;
	port->PortConfig.SinkRequestMaxPower = PD_Power_as_Sink;
	port->PortConfig.SinkRequestOpPower = PD_Power_as_Sink;
	port->PortConfig.SinkGotoMinCompatible = GiveBack_May_Be_Set;
	port->PortConfig.SinkUSBSuspendOperation = No_USB_Suspend_May_Be_Set;
	port->PortConfig.SinkUSBCommCapable = USB_Comms_Capable;
#endif /* AW_HAVE_SNK */

#ifdef AW_HAVE_ACCMODE
	port->PortConfig.audioAccSupport = Type_C_Supports_Audio_Accessory;
	port->PortConfig.poweredAccSupport = Type_C_Is_VCONN_Powered_Accessory;
#endif /* AW_HAVE_ACCMODE */

	port->PortConfig.RpVal = utccDefault;

	if ((Rp_Value + 1) > utccNone && (Rp_Value + 1) < utccInvalid)
		port->PortConfig.RpVal = (USBTypeCCurrent)(Rp_Value + 1);

	switch (PD_Port_Type) {
	case 0:
		/* Consumer Only */
		port->PortConfig.PortType = USBTypeC_Sink;
		break;
	case 1:
		/* Consumer/Provider */
		if (Type_C_State_Machine == 1)
			port->PortConfig.PortType = USBTypeC_Sink;
		else if (Type_C_State_Machine == 2)
			port->PortConfig.PortType = USBTypeC_DRP;
		else
			port->PortConfig.PortType = USBTypeC_UNDEFINED;
		break;
	case 2:
		/* Provider/Consumer */
		if (Type_C_State_Machine == 0)
			port->PortConfig.PortType = USBTypeC_Source;
		else if (Type_C_State_Machine == 2)
			port->PortConfig.PortType = USBTypeC_DRP;
		else
			port->PortConfig.PortType = USBTypeC_UNDEFINED;
		break;
	case 3:
		/* Provider Only */
		port->PortConfig.PortType = USBTypeC_Source;
		break;
	case 4:
		port->PortConfig.PortType = USBTypeC_DRP;
		break;
	default:
		port->PortConfig.PortType = USBTypeC_UNDEFINED;
		break;
	}

	/* Avoid undefined port type */
	if (port->PortConfig.PortType == USBTypeC_UNDEFINED)
		port->PortConfig.PortType = USBTypeC_DRP;

#ifdef AW_HAVE_DRP
	if (port->PortConfig.PortType == USBTypeC_DRP) {
		port->PortConfig.SrcPreferred = Type_C_Implements_Try_SRC;
		port->PortConfig.SnkPreferred = Type_C_Implements_Try_SNK;
	} else {
		port->PortConfig.SrcPreferred = AW_FALSE;
		port->PortConfig.SnkPreferred = AW_FALSE;
	}
#endif /* AW_HAVE_DRP */
}

void InitializeRegisters(Port_t *port)
{
	AW_U8 reset = 0x01;

	DeviceWrite(port, regReset, 1, &reset);

	DeviceRead(port, regDeviceID, 1, &port->Registers.DeviceID.byte);
	DeviceRead(port, regSwitches0, 1, &port->Registers.Switches.byte[0]);
	DeviceRead(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);
	DeviceRead(port, regMeasure, 1, &port->Registers.Measure.byte);
	DeviceRead(port, regSlice, 1, &port->Registers.Slice.byte);
	DeviceRead(port, regControl0, 1, &port->Registers.Control.byte[0]);
	DeviceRead(port, regControl1, 1, &port->Registers.Control.byte[1]);
	DeviceRead(port, regControl2, 1, &port->Registers.Control.byte[2]);
	DeviceRead(port, regControl3, 1, &port->Registers.Control.byte[3]);
	DeviceRead(port, regMask, 1, &port->Registers.Mask.byte);
	DeviceRead(port, regPower, 1, &port->Registers.Power.byte);
	DeviceRead(port, regReset, 1, &port->Registers.Reset.byte);
	DeviceRead(port, regOCPreg, 1, &port->Registers.OCPreg.byte);
	DeviceRead(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	DeviceRead(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);
	DeviceRead(port, regControl4, 1, &port->Registers.Control4.byte);
	DeviceRead(port, regControl5, 1, &port->Registers.Control5.byte);
	DeviceRead(port, regStatus0a, 1, &port->Registers.Status.byte[0]);
	DeviceRead(port, regStatus1a, 1, &port->Registers.Status.byte[1]);
	DeviceRead(port, regInterrupta, 1, &port->Registers.Status.byte[2]);
	DeviceRead(port, regInterruptb, 1, &port->Registers.Status.byte[3]);
	DeviceRead(port, regStatus0, 1, &port->Registers.Status.byte[4]);
	DeviceRead(port, regStatus1, 1, &port->Registers.Status.byte[5]);
	DeviceRead(port, regInterrupt, 1, &port->Registers.Status.byte[6]);
}

void InitializeTypeCVariables(Port_t *port)
{
	port->Registers.Mask.byte = 0xFF;
	DeviceWrite(port, regMask, 1, &port->Registers.Mask.byte);
	port->Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(port, regMaska, 1, &port->Registers.MaskAdv.byte[0]);
	port->Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(port, regMaskb, 1, &port->Registers.MaskAdv.byte[1]);

	/* Enable interrupt Pin */
	port->Registers.Control.INT_MASK = 0;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);

	/* These two control values allow detection of Ra-Ra or Ra-Open. */
	/* Enabling them will allow some corner case devices to connect where */
	/* they might not otherwise. */
	port->Registers.Control.TOG_RD_ONLY = 0;
	DeviceWrite(port, regControl2, 1, &port->Registers.Control.byte[2]);
	port->Registers.Control4.TOG_EXIT_AUD = 1; /* enable Ra Toggle only in cts test*/
	port->Registers.Control4.EN_PAR_CFG = 1;
	DeviceWrite(port, regControl4, 1, &port->Registers.Control4.byte);
	port->Registers.Control5.EN_PD3_MSG = 1;
	DeviceWrite(port, regControl5, 1, &port->Registers.Control5.byte);
	port->Registers.Switches.SPECREV = 1; /* enable good-crc specrev 2.0*/
	port->Registers.Switches.AUTO_CRC = 0;
	DeviceWrite(port, regSwitches1, 1, &port->Registers.Switches.byte[1]);
	port->TCIdle = AW_TRUE;
	port->SMEnabled = AW_FALSE;

	SetTypeCState(port, Disabled);

	port->DetachThreshold = VBUS_MV_VSAFE5V_DISC;
	port->CCPin = CCNone;
	port->C2ACable = AW_FALSE;
	port->vbus_flag_last = VBUS_DIS;

	resetDebounceVariables(port);

#ifdef AW_HAVE_SNK
	/* Clear the current advertisement initially */
	port->SinkCurrent = utccNone;
#endif /* AW_HAVE_SNK */
}

void InitializePDProtocolVariables(Port_t *port)
{
	port->DoTxFlush = AW_FALSE;
}

void InitializePDPolicyVariables(Port_t *port)
{
	port->isContractValid = AW_FALSE;

	port->IsHardReset = AW_FALSE;
	port->IsPRSwap = AW_FALSE;

	port->WaitingOnHR = AW_FALSE;

	port->PEIdle = AW_TRUE;
	port->USBPDActive = AW_FALSE;
	port->USBPDEnabled = AW_TRUE;

	/* Source Caps & Header */
	port->src_cap_header.word = 0;
	port->src_cap_header.NumDataObjects = port->src_pdo_size;
	port->src_cap_header.MessageType = DMTSourceCapabilities;
	port->src_cap_header.SpecRevision = port->PortConfig.PdRevPreferred;

	VIF_InitializeSrcCaps(port->src_caps);

	/* Sink Caps & Header */
	port->snk_cap_header.word = 0;
	port->snk_cap_header.NumDataObjects = port->snk_pdo_size;
	port->snk_cap_header.MessageType = DMTSinkCapabilities;
	port->snk_cap_header.SpecRevision = port->PortConfig.PdRevPreferred;

	VIF_InitializeSnkCaps(port->snk_caps);

#ifdef AW_HAVE_VDM
	InitializeVdmManager(port);
	vdmInitDpm(port);
	port->AutoModeEntryObjPos = -1;
	port->discoverIdCounter = 0;
	port->cblPresent = AW_FALSE;
	port->cblRstState = CBL_RST_DISABLED;
#endif /* AW_HAVE_VDM */
}

void SetConfiguredCurrent(Port_t *port)
{
	switch (port->PortConfig.RpVal) {
	case 1:
		port->SourceCurrent = utccDefault;
		break;
	case 2:
		port->SourceCurrent = utcc1p5A;
		break;
	case 3:
		port->SourceCurrent = utcc3p0A;
		break;
	default:
		port->SourceCurrent = utccNone;
		break;
	}
}
