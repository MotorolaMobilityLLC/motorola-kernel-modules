/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef VENDOR_MACROS_H
#define VENDOR_MACROS_H
#include "PD_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PORT_SUPPLY_FIRST_FIXED(idx)\
{	.FPDOSupply = {\
		.MaxCurrent        = Src_PDO_Max_Current##idx,\
		.Voltage           = Src_PDO_Voltage##idx,\
		.PeakCurrent       = Src_PDO_Peak_Current##idx,\
		.DataRoleSwap      = DR_Swap_To_UFP_Supported,\
		.USBCommCapable    = USB_Comms_Capable,\
		.ExternallyPowered = Unconstrained_Power,\
		.USBSuspendSupport = SWAP(USB_Suspend_May_Be_Cleared),\
		.DualRolePower     = Accepts_PR_Swap_As_Src,\
		.SupplyType        = Src_PDO_Supply_Type##idx}}

#define PORT_SUPPLY_TYPE_0(idx)\
{	.FPDOSupply = {\
		.MaxCurrent  = Src_PDO_Max_Current##idx,\
		.Voltage     = Src_PDO_Voltage##idx,\
		.PeakCurrent = Src_PDO_Peak_Current##idx,\
		.SupplyType  = Src_PDO_Supply_Type##idx}}

#define PORT_SUPPLY_TYPE_1(idx)\
{	.BPDO = {\
		.MaxPower   = Src_PDO_Max_Power##idx,\
		.MinVoltage = Src_PDO_Min_Voltage##idx,\
		.MaxVoltage = Src_PDO_Max_Voltage##idx,\
		.SupplyType = Src_PDO_Supply_Type##idx}}

#define PORT_SUPPLY_TYPE_2(idx)\
{	.VPDO = {\
		.MaxCurrent = Src_PDO_Max_Current##idx,\
		.MinVoltage = Src_PDO_Min_Voltage##idx,\
		.MaxVoltage = Src_PDO_Max_Voltage##idx,\
		.SupplyType = Src_PDO_Supply_Type##idx}}

#define PORT_SUPPLY_TYPE_3(idx)\
{	.PPSAPDO = {\
		.MaxCurrent = Src_PDO_Max_Current##idx,\
		.MinVoltage = Src_PDO_Min_Voltage##idx,\
		.MaxVoltage = Src_PDO_Max_Voltage##idx,\
		.SupplyType = Src_PDO_Supply_Type##idx}}

#define PORT_SUPPLY_TYPE_(idx, type)      PORT_SUPPLY_TYPE_##type(idx)
#define CREATE_SUPPLY_PDO(idx, type)      PORT_SUPPLY_TYPE_(idx, type)
#define CREATE_SUPPLY_PDO_FIRST(idx)      PORT_SUPPLY_FIRST_FIXED(idx)

/* Sink Fixed Supply Power Data Object */
#define PORT_SINK_TYPE_0(idx)\
{	.FPDOSink = {\
		.OperationalCurrent = Snk_PDO_Op_Current##idx,\
		.Voltage            = Snk_PDO_Voltage##idx,\
		.DataRoleSwap       = DR_Swap_To_DFP_Supported,\
		.USBCommCapable     = USB_Comms_Capable,\
		.ExternallyPowered  = Unconstrained_Power,\
		.HigherCapability   = Higher_Capability_Set,\
		.DualRolePower      = Accepts_PR_Swap_As_Snk,\
		.SupplyType         = pdoTypeFixed}}
/* Battery Supply Power Data Object */
#define PORT_SINK_TYPE_1(idx)\
{	.BPDO = {\
		.MaxPower   = Snk_PDO_Op_Power##idx,\
		.MinVoltage = Snk_PDO_Min_Voltage##idx,\
		.MaxVoltage = Snk_PDO_Max_Voltage##idx,\
		.SupplyType = Snk_PDO_Supply_Type##idx}}
/* Variable Supply (non-Battery) Power Data Object */
#define PORT_SINK_TYPE_2(idx)\
{	.VPDO = {\
		.MaxCurrent = Snk_PDO_Op_Current##idx,\
		.MinVoltage = Snk_PDO_Min_Voltage##idx,\
		.MaxVoltage = Snk_PDO_Max_Voltage##idx,\
		.SupplyType = Snk_PDO_Supply_Type##idx}}
/* Augmented Power Data Object */
#define PORT_SINK_TYPE_3(idx)\
{	.PPSAPDO = {\
		.MaxCurrent = Snk_PDO_Op_Current##idx,\
		.MinVoltage = Snk_PDO_Min_Voltage##idx,\
		.MaxVoltage = Snk_PDO_Max_Voltage##idx,\
		.SupplyType = Snk_PDO_Supply_Type##idx}}

#define PORT_SINK_TYPE_(idx, type)      PORT_SINK_TYPE_##type(idx)
#define CREATE_SINK_PDO(idx, type)      PORT_SINK_TYPE_(idx, type)

/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
#ifdef AW_HAVE_EXT_MSG
static const AW_U8 gCountry_codes[6] = {
	2, 0, /* 2-byte Number of country codes */

	/* country codes follow */
	'U', 'S', 'C', 'N'
};
#endif

void VIF_InitializeSrcCaps(doDataObject_t *src_caps);
void VIF_InitializeSnkCaps(doDataObject_t *snk_caps);

/** Helpers **/
#define YES 1
#define NO 0
#define SWAP(X) ((X) ? 0 : 1)

#ifdef __cplusplus
}
#endif

#endif /* VENDOR_MACROS_H */

