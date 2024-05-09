// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name	:dpm.c
 * Author	:awinic
 * Date		:2021-09-10
 * Description	:.C file function description
 * Version	:1.0
 * Function List :
 ******************************************************************************/
#include "../vendor_info.h"
#include "dpm.h"
#include "../TypeC.h"
#include "../platform_helpers.h"

typedef enum {
	dpmSelectSourceCap,
	dpmIdle,
} DpmState_t;

/** Definition of DPM interface
 *
 */
struct devicePolicy_t {
	Port_t *ports[NUM_PORTS];/*< List of port managed*/
	AW_U8  num_ports;
	DpmState_t  dpm_state;
};

static DevicePolicy_t devicePolicyMgr;

void DPM_Init(DevicePolicy_t **dpm)
{
	devicePolicyMgr.num_ports = 0;
	devicePolicyMgr.dpm_state = dpmIdle;

	*dpm = &devicePolicyMgr;
}

void DPM_AddPort(DevicePolicy_t *dpm, Port_t *port)
{
	dpm->ports[dpm->num_ports++] = port;
}

sopMainHeader_t *DPM_GetSourceCapHeader(DevicePolicy_t *dpm, Port_t *port)
{
	/* The DPM has access to all ports.  If needed, update this port here based
	 * on the status of other ports - e.g. power sharing, etc.
	 */
	return &(port->src_cap_header);
}

sopMainHeader_t *DPM_GetSinkCapHeader(DevicePolicy_t *dpm, Port_t *port)
{
	/* The DPM has access to all ports.  If needed, update this port here based
	 * on the status of other ports - e.g. power sharing, etc.
	 */
	return &(port->snk_cap_header);
}

doDataObject_t *DPM_GetSourceCap(DevicePolicy_t *dpm, Port_t *port)
{
	/* The DPM has access to all ports.  If needed, update this port here based
	 * on the status of other ports - e.g. power sharing, etc.
	 */
	return port->src_caps;
}

doDataObject_t *DPM_GetSinkCap(DevicePolicy_t *dpm, Port_t *port)
{
	/* The DPM has access to all ports.  If needed, update this port here based
	 * on the status of other ports - e.g. power sharing, etc.
	 */
	return port->snk_caps;
}

AW_BOOL DPM_TransitionSource(DevicePolicy_t *dpm, Port_t *port, AW_U8 index)
{
	AW_BOOL status = AW_TRUE;

	if (port->src_caps[index].PDO.SupplyType == pdoTypeFixed) {
	/* Convert 10mA units to mA for setting supply current */
		platform_set_pps_current(port->PortID,
					 port->PolicyRxDataObj[0].FVRDO.OpCurrent * 10);
	platform_set_pps_voltage(port->PortID,
				(port->src_caps[index].FPDOSupply.Voltage * 50));
	} else if (port->src_caps[index].PDO.SupplyType == pdoTypeAugmented) {
	/* PPS request is already in 20mV units */
		platform_set_pps_voltage(port->PortID,
					 port->PolicyRxDataObj[0].PPSRDO.Voltage * 20);

	/* Convert 50mA units to mA for setting supply current */
	platform_set_pps_current(port->PortID,
				 port->PolicyRxDataObj[0].PPSRDO.OpCurrent * 50);
	}

	return status;
}

AW_BOOL DPM_IsSourceCapEnabled(DevicePolicy_t *dpm, Port_t *port, AW_U8 index)
{
	AW_BOOL status = AW_FALSE;
	AW_U32 sourceVoltage = 0;

	if (port->src_caps[index].PDO.SupplyType == pdoTypeFixed) {
		sourceVoltage = port->src_caps[index].FPDOSupply.Voltage;

		if (!isVBUSOverVoltage(port,
			VBUS_MV_NEW_MAX(VBUS_PD_TO_MV(sourceVoltage)) + MDAC_MV_LSB) &&
			isVBUSOverVoltage(port, VBUS_MV_NEW_MIN(VBUS_PD_TO_MV(sourceVoltage))))
			status = AW_TRUE;
	} else if (port->src_caps[index].PDO.SupplyType == pdoTypeAugmented) {
		sourceVoltage = port->USBPDContract.PPSRDO.Voltage;

		if (!isVBUSOverVoltage(port, VBUS_MV_NEW_MAX(VBUS_PPS_TO_MV(sourceVoltage)) +
			MDAC_MV_LSB) &&
			isVBUSOverVoltage(port, VBUS_MV_NEW_MIN(VBUS_PPS_TO_MV(sourceVoltage))))
			status = AW_TRUE;
	}

	return status;
}

SpecRev DPM_SpecRev(Port_t *port, SopType sop)
{
	if (sop == SOP_TYPE_SOP) {
		/* Port Partner */
		return (SpecRev)port->PdRevSop;
	} else if (sop == SOP_TYPE_SOP1 || sop == SOP_TYPE_SOP2) {
		/* Cable marker */
		return (SpecRev)port->PdRevCable;
	}

	/* Debug, default, etc. Handle as needed. */
	return (SpecRev)USBPDSPECREV2p0;
}

void DPM_SetSpecRev(Port_t *port, SopType sop, SpecRev rev)
{
	if (rev >= USBPDSPECREVMAX) {
		/* Compliance test tries invalid revision value */
		rev = (SpecRev)(USBPDSPECREVMAX - 1);
	}

	if (sop == SOP_TYPE_SOP && port->PdRevSop > rev)
		port->PdRevSop = rev;
	else if (sop == SOP_TYPE_SOP1 || sop == SOP_TYPE_SOP2)
		port->PdRevCable = rev;

	/* Adjust according to compatibility table */
	if (port->PdRevSop == USBPDSPECREV2p0 && port->PdRevCable == USBPDSPECREV3p0)
		port->PdRevCable = USBPDSPECREV2p0;
}

#ifdef AW_HAVE_VDM
SvdmVersion DPM_SVdmVer(Port_t *port, SopType sop)
{
	return (DPM_SpecRev(port, sop) == USBPDSPECREV2p0) ? V1P0 : V2P0;
}
#endif /* AW_HAVE_VDM */

AW_U8 DPM_Retries(Port_t *port, SopType sop)
{
	SpecRev rev = DPM_SpecRev(port, sop);

	return (rev == USBPDSPECREV3p0) ? 2 : 3;
}

