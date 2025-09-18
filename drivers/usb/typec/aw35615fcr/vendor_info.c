// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-12-17
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#include "vendor_info.h"
#include "PD_Types.h"
#include "aw35615_global.h"

void VIF_InitializeSrcCaps(Port_t *port)
{
	AW_U8 i;
	int index1 = 0;
	int index2= 0;
	int index3 = 0;
	int src_pdo_size = 0;

	doDataObject_t gSrc_caps[7] = {
		/* macro expects index starting at 1 and type */
		CREATE_SUPPLY_PDO_FIRST(1),
		//CREATE_SUPPLY_PDO(1, Src_PDO_Supply_Type1),
		CREATE_SUPPLY_PDO(2, Src_PDO_Supply_Type2),
		CREATE_SUPPLY_PDO(3, Src_PDO_Supply_Type3),
		CREATE_SUPPLY_PDO(4, Src_PDO_Supply_Type4),
		CREATE_SUPPLY_PDO(5, Src_PDO_Supply_Type5),
		CREATE_SUPPLY_PDO(6, Src_PDO_Supply_Type6),
		CREATE_SUPPLY_PDO(7, Src_PDO_Supply_Type7),
	};

	if (port == NULL) {
		return;
	}

	for (i = 0; i < port->src_pdo_size; ++i) {
		gSrc_caps[i].FPDOSupply.Voltage = port->src_pdo_vol[i] / 50;
		gSrc_caps[i].FPDOSupply.MaxCurrent = port->src_pdo_cur[i] / 10;
		gSrc_caps[i].FPDOSupply.SupplyType = 0;
		if (i > 0) {
			gSrc_caps[i].FPDOSupply.USBCommCapable = 0;
			gSrc_caps[i].FPDOSupply.DataRoleSwap = 0;
		}
	}

	if (port->src_pps_size) {
		for (i = port->src_pdo_size; i < port->src_pdo_size + port->src_pps_size; ++i) {
			gSrc_caps[i].object = 0;
			src_pdo_size = port->src_pdo_size;
			index1 = (i - src_pdo_size) * 2;
			index2 = (i - src_pdo_size) * 2 + 1;
			index3 = i - src_pdo_size;

			if ((index1 >= 0) && (index1 < SRC_PPS_VOL_SIZE)) {
				gSrc_caps[i].PPSAPDO.MinVoltage = port->src_pps_vol[index1] / 100;
			} else {
				gSrc_caps[i].PPSAPDO.MinVoltage = 0;
			}
			if ((index2 >= 0) && (index2 < SRC_PPS_VOL_SIZE)) {
				gSrc_caps[i].PPSAPDO.MaxVoltage = port->src_pps_vol[index2] / 100;
			} else {
				gSrc_caps[i].PPSAPDO.MaxVoltage = 0;
			}
			if ((index3 >= 0) && (index3 < SRC_PPS_CUR_SIZE)) {
				gSrc_caps[i].PPSAPDO.MaxCurrent = port->src_pps_cur[index3] / 50;
			} else {
				gSrc_caps[i].PPSAPDO.MaxCurrent = 0;
			}
			gSrc_caps[i].PPSAPDO.SupplyType = 3;
		}
	}

	for (i = 0; i < 7; ++i)
		port->src_caps[i].object = gSrc_caps[i].object;
}
void VIF_InitializeSnkCaps(Port_t *port)
{
	AW_U8 i;
	int index1 = 0;
	int index2= 0;
	int index3 = 0;
	int snk_pdo_size = 0;


	doDataObject_t gSnk_caps[7] = {
		/* macro expects index start at 1 and type */
		CREATE_SINK_PDO(1, Snk_PDO_Supply_Type1),
		CREATE_SINK_PDO(2, Snk_PDO_Supply_Type2),
		CREATE_SINK_PDO(3, Snk_PDO_Supply_Type3),
		CREATE_SINK_PDO(4, Snk_PDO_Supply_Type4),
		CREATE_SINK_PDO(5, Snk_PDO_Supply_Type5),
		CREATE_SINK_PDO(6, Snk_PDO_Supply_Type6),
		CREATE_SINK_PDO(7, Snk_PDO_Supply_Type7),
	};

	if (port == NULL) {
		return;
	}

	for (i = 0; i < port->snk_pdo_size; ++i) {
		gSnk_caps[i].FPDOSink.Voltage = port->snk_pdo_vol[i] / 50;
		gSnk_caps[i].FPDOSink.OperationalCurrent = port->snk_pdo_cur[i] / 10;
		gSnk_caps[i].FPDOSink.SupplyType = 0;
		if (i > 0) {
			gSnk_caps[i].FPDOSink.DataRoleSwap = 0;
			gSnk_caps[i].FPDOSink.USBCommCapable = 0;
			gSnk_caps[i].FPDOSink.ExternallyPowered = 0;
			gSnk_caps[i].FPDOSink.HigherCapability = 0;
			gSnk_caps[i].FPDOSink.DualRolePower = 0;
		}
	}

	if (port->snk_pps_size) {
		for (i = port->snk_pdo_size; i < (port->snk_pdo_size + port->snk_pps_size); ++i) {
			gSnk_caps[i].object = 0;
			snk_pdo_size = port->snk_pdo_size;
			index1 = (i - snk_pdo_size) * 2;
			index2 = (i - snk_pdo_size) * 2 + 1;
			index3 = i - snk_pdo_size;

			if ((index1 >= 0) && (index1 < SNK_PPS_VOL_SIZE)) {
				gSnk_caps[i].PPSAPDO.MinVoltage = port->snk_pps_vol[index1] / 100;
			} else {
				gSnk_caps[i].PPSAPDO.MinVoltage = 0;
			}
			if ((index2 >= 0) && (index2 < SNK_PPS_VOL_SIZE)) {
				gSnk_caps[i].PPSAPDO.MaxVoltage = port->snk_pps_vol[index2] / 100;
			} else {
				gSnk_caps[i].PPSAPDO.MaxVoltage = 0;
			}
			if ((index3 >= 0) && (index3 < SNK_PPS_CUR_SIZE)) {
				gSnk_caps[i].PPSAPDO.MaxCurrent = port->snk_pps_cur[index3] / 50;
			} else {
				gSnk_caps[i].PPSAPDO.MaxCurrent = 0;
			}
			gSnk_caps[i].PPSAPDO.SupplyType = 3;
		}
	}

	for (i = 0; i < 7; ++i)
		port->snk_caps[i].object = gSnk_caps[i].object;
}

