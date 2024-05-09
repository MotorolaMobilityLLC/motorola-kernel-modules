/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved.************
 *******************************************************************************
 * File Name	 : fsc_vdm_defs.h
 * Author		: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __AW_VDM_DEFS_H__
#define __AW_VDM_DEFS_H__
#include "vdm_types.h"

#ifdef AW_HAVE_VDM

/*
 *  definition/configuration object - these are all the things that the system needs to configure.
 */
typedef struct {
	AW_BOOL data_capable_as_usb_host : 1;
	AW_BOOL data_capable_as_usb_device : 1;
	ProductType product_type : 3;
	AW_BOOL modal_operation_supported   : 1;
	AW_U16 usb_vendor_id : 16;
	/* for Cert Stat VDO, "allocated by USB-IF during certification" */
	AW_U32 test_id : 20;
	AW_U16 usb_product_id : 16;
	AW_U16 bcd_device : 16;
	AW_U8 cable_hw_version : 4;
	AW_U8 cable_fw_version : 4;
	CableToType cable_to_type : 2;
	CableToPr cable_to_pr : 1;
	CableLatency cable_latency : 4;
	CableTermType cable_term : 2;
	SsDirectionality sstx1_dir_supp : 1;
	SsDirectionality sstx2_dir_supp : 1;
	SsDirectionality ssrx1_dir_supp : 1;
	SsDirectionality ssrx2_dir_supp : 1;
	VbusCurrentHandlingCapability vbus_current_handling_cap : 2;
	VbusThruCable vbus_thru_cable : 1;
	Sop2Presence sop2_presence : 1;

	VConnFullPower vconn_full_power : 3;
	VConnRequirement vconn_requirement : 1;
	VBusRequirement vbus_requirement : 1;

	UsbSsSupport usb_ss_supp : 3;
	AmaUsbSsSupport ama_usb_ss_supp : 3;

	AW_U32 num_svids;
	AW_U16 svids[MAX_NUM_SVIDS];
	AW_U32 num_modes_for_svid[MAX_NUM_SVIDS];

	/* TODO: A lot of potential wasted memory here... */
	AW_U32 modes[MAX_NUM_SVIDS][MAX_MODES_PER_SVID];

} VendorDefinition;

#endif /* AW_HAVE_VDM */

#endif /* __AW_VDM_DEFS_H__*/
