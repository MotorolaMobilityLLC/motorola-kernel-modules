/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name	 : dpm.h
 * Author		: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#ifndef MODULES_DPM_H_
#define MODULES_DPM_H_
#include "../PD_Types.h"
#include "../TypeC_Types.h"
#include "../vdm/vdm_types.h"

/*****************************************************************************
 * Forward declarations
 *****************************************************************************/
typedef struct devicePolicy_t DevicePolicy_t;
typedef DevicePolicy_t *DevicePolicyPtr_t;

typedef struct Port Port_t;

/**
 * Configuration structure for port. Port looks at the structure to decide
 * policy settings. This is shared variables by both Port and DPM.
 */
typedef struct {
	USBTypeCPort	PortType;		/* Snk/Src/DRP */
	AW_BOOL		SrcPreferred;		/* Source preferred (DRP) */
	AW_BOOL		SnkPreferred;		/* Sink preferred (DRP) */
	AW_BOOL		SinkGotoMinCompatible;	/* Sink GotoMin supported. */
	AW_BOOL		SinkUSBSuspendOperation;/* USB suspend capable */
	AW_BOOL		SinkUSBCommCapable;	/* USB communications capable */
	AW_U32		 SinkRequestMaxVoltage;	/* Sink Maximum voltage */
	AW_U32		 SinkRequestMaxPower;	/* Sink Maximum power */
	AW_U32		 SinkRequestOpPower;	/* Sink Operating power */
	AW_BOOL		audioAccSupport;	/* Audio Acc support */
	AW_BOOL		poweredAccSupport;	/* Powered Acc support */
	AW_BOOL		reqDRSwapToDfpAsSink;	/* Request DR swap as sink */
	AW_BOOL		reqDRSwapToUfpAsSrc;	/* Request DR swap as source */
	AW_BOOL		reqVconnSwapToOnAsSink;	/* Request Vconn swap */
	AW_BOOL		reqVconnSwapToOffAsSrc;	/* Request Vconn swap */
	AW_BOOL		reqPRSwapAsSrc;		/* Request PR swap as source */
	AW_BOOL		reqPRSwapAsSnk;		/* Request PR swap as sink*/
	USBTypeCCurrent RpVal;			/* Pull up value to use */
	AW_U8		PdRevPreferred;		/* PD Rev to use */
} PortConfig_t;

/**
 * @brief Initializes the DPM object pointer
 * @param[in] DPM pointer type object
 */
void DPM_Init(DevicePolicyPtr_t *dpm);

/**
 * @brief Adds port to the list of ports managed by dpm
 * @param[in] dpm object to which the port is added
 * @param[in] port object which is added to DPM list
 */
void DPM_AddPort(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get source cap header for the port object.
 * @param[in] dpm pointer to device policy object
 * @param[in] port requesting source cap header
 */
sopMainHeader_t *DPM_GetSourceCapHeader(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get sink cap header for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting sink cap header
 */
sopMainHeader_t *DPM_GetSinkCapHeader(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get the source cap for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting source caps
 */
doDataObject_t *DPM_GetSourceCap(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Get sink cap for the port object
 * @param[in] dpm pointer to device policy object
 * @param[in] port object requesting sink cap
 */
doDataObject_t *DPM_GetSinkCap(DevicePolicyPtr_t dpm, Port_t *port);

/**
 * @brief Called by the usb PD/TypeC core to ask device policy to transition
 * to the capability advertised specified by port and index.
 * @param[in] dpm pointer to the device policy object
 * @param[in] port advertising the source capability
 * @param[in] index of the source capability object
 */
AW_BOOL DPM_TransitionSource(DevicePolicyPtr_t dpm, Port_t *port, AW_U8 index);

/**
 * @brief Called by usb PD/TypeC core to ask device policy if the source
 * is ready after the transition. It returns true if the source transition has
 * completed and successful.
 * @param[in] dpm pointer to the device policy object
 * @param[in] port advertising the source capability
 * @param[in] index of the source capability object
 */
AW_BOOL DPM_IsSourceCapEnabled(DevicePolicyPtr_t dpm, Port_t *port, AW_U8 index);

/**
 * @brief Returns appropriate spec revision value per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
SpecRev DPM_SpecRev(Port_t *port, SopType sop);

/**
 * @brief Sets appropriate spec revision value per SOP* and adjusts for
 * compatibility.
 * @param[in] current port object
 * @param[in] SOP in question
 */
void DPM_SetSpecRev(Port_t *port, SopType sop, SpecRev rev);

#ifdef AW_HAVE_VDM
/**
 * @brief Returns appropriate SVDM revision value per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
SvdmVersion DPM_SVdmVer(Port_t *port, SopType sop);
#endif /* AW_HAVE_VDM */

/**
 * @brief Returns appropriate number of retries (based on spec rev) per SOP*.
 * @param[in] current port object
 * @param[in] SOP in question
 */
AW_U8 DPM_Retries(Port_t *port, SopType sop);

#endif /* MODULES_DPM_H_ */
