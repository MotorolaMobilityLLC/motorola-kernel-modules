// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved.************
 *******************************************************************************
 * Author		: awinic
 * Date		  : 2021-09-09
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 *******************************************************************************/
#include "platform_helpers.h"
#include "core.h"
#include "TypeC.h"
#include "PDProtocol.h"
#include "PDPolicy.h"
#include "PD_Types.h"
/*
 * Call this function to initialize the core.
 */
void core_initialize(Port_t *port)
{
	PortInit(port);
	core_enable_typec(port, AW_TRUE);
	core_set_state_unattached(port);
}

/*
 * Call this function to enable or disable the core Type-C state machine.
 */
void core_enable_typec(Port_t *port, AW_BOOL enable)
{
	port->SMEnabled = enable;
}

/*
 * Call this function to enable or disable the core PD state machines.
 */
void core_enable_pd(Port_t *port, AW_BOOL enable)
{
	port->USBPDEnabled = enable;
}

/*
 * Call this function to run the state machines.
 */
void core_state_machine(Port_t *port)
{
	AW_U8 data = port->Registers.Control.byte[3] | 0x40;  /* Hard Reset bit */

	/* Check on HardReset timeout (shortcut for SenderResponse timeout) */
	if ((port->WaitingOnHR == AW_TRUE) && TimerExpired(&port->PolicyStateTimer))
		DeviceWrite(port, regControl3, 1, &data);

	/* Update the current port being used and process the port */
	/* The Protocol and Policy functions are called from within this call */
	StateMachineTypeC(port);
}

/*
 * Check for the next required timeout to support timer interrupt functionality
 */
AW_U32 core_get_next_timeout(Port_t *port)
{
	AW_U32 time = 0;
	AW_U32 nexttime = 0xFFFFFFFF;
	AW_U8 i;

	for (i = 0; i < AW_NUM_TIMERS; ++i) {
		time = TimerRemaining(port->Timers[i]);
		if (time > 0 && time < nexttime)
			nexttime = time;
	}

	if (nexttime == 0xFFFFFFFF)
		nexttime = 0;

	return nexttime;
}

AW_U8 core_get_cc_orientation(Port_t *port)
{
	return port->CCPin;
}

void core_set_state_unattached(Port_t *port)
{
	SetStateUnattached(port);
}

void core_reset_pd(Port_t *port)
{
	port->USBPDEnabled = AW_TRUE;
	USBPDEnable(port, AW_TRUE, port->sourceOrSink);
}

void core_set_advertised_current(Port_t *port, AW_U8 value)
{
	UpdateCurrentAdvert(port, (USBTypeCCurrent)value);
}

void core_set_drp(Port_t *port)
{
#ifdef AW_HAVE_DRP
	port->PortConfig.PortType = USBTypeC_DRP;
	port->PortConfig.SnkPreferred = AW_FALSE;
	port->PortConfig.SrcPreferred = AW_FALSE;
	SetStateUnattached(port);
#endif /* AW_HAVE_DRP */
}

void core_set_try_snk(Port_t *port)
{
#ifdef AW_HAVE_DRP
	port->PortConfig.PortType = USBTypeC_DRP;
	port->PortConfig.SnkPreferred = AW_TRUE;
	port->PortConfig.SrcPreferred = AW_FALSE;
	SetStateUnattached(port);
#endif /* AW_HAVE_DRP */
}

void core_set_try_src(Port_t *port)
{
#ifdef AW_HAVE_DRP
	port->PortConfig.PortType = USBTypeC_DRP;
	port->PortConfig.SnkPreferred = AW_FALSE;
	port->PortConfig.SrcPreferred = AW_TRUE;
	SetStateUnattached(port);
#endif /* AW_HAVE_DRP */
}

void core_set_source(Port_t *port)
{
#ifdef AW_HAVE_SRC
	port->PortConfig.PortType = USBTypeC_Source;
	SetStateUnattached(port);
#endif /* AW_HAVE_SRC */
}

void core_set_sink(Port_t *port)
{
#ifdef AW_HAVE_SNK
	port->PortConfig.PortType = USBTypeC_Sink;
	SetStateUnattached(port);
#endif /* AW_HAVE_SNK */
}

