/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-09
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *******************************************************************************/

#ifndef _AW_CORE_H
#define _AW_CORE_H
#include "Port.h"

/**
 * @brief Initializes the core port structure.
 *
 * Initializes the port structure with default values.
 * Also sets the i2c address of the device and enables the TypeC state machines.
 *
 * @param port Pointer to the port structure
 * @param i2c_address 8-bit value with bit zero (R/W bit) set to zero.
 * @return None
 */
void core_initialize(Port_t *port);

/**
 * @brief Enable the TypeC or PD state machines.
 *
 * Sets whether the state machine functions are executed.
 * Does no initialization or reset of variables.
 *
 * @param port Pointer to the port structure
 * @param enable AW_TRUE/AW_FALSE
 * @return None
 */
void core_enable_typec(Port_t *port, AW_BOOL enable);
void core_enable_pd(Port_t *port, AW_BOOL enable);

/**
 * @brief Core state machine entry point.
 *
 * Called from a high level while(1) loop or interrupt handler to process
 * port status and control.
 * May block briefly (e.g. for I2C peripheral access) but should otherwise
 * return after a single pass through the state machines.
 *
 * @param port Pointer to the port structure
 * @return None
 */
void core_state_machine(Port_t *port);

/**
 * @brief Determine the time to the next expiring core timer.
 *
 * The state machines occasionally need to wait for or to do something.
 * Instead of allowing the state machines to block, use this value to
 * implement a (for example) timer interrupt to delay the next call of the
 * core state machines.
 * This function returns the time to the next event in the resolution set
 * by the Platform (milliseconds * TICK_SCALE_TO_MS).
 *
 * @param port Pointer to the port structure
 * @return Interval to next timeout event (See brief and platform timer setup)
 */
AW_U32 core_get_next_timeout(Port_t *port);

/**
 * @brief Detatch and reset the current port.
 *
 * Instruct the port state machines to detach and reset to their
 * looking-for-connection state.
 *
 * @param port Pointer to the port structure
 * @return None
 */
void core_set_state_unattached(Port_t *port);

/**
 * @brief Reset PD state machines.
 *
 * For the current connection, re-initialize and restart the
 * PE and Protocol state machines.
 *
 * @param port Pointer to the port structure
 * @return None
 */
void core_reset_pd(Port_t *port);

/**
 * @brief Return advertised source current available to this device.
 *
 * If this port is the Sink device, return the available current being
 * advertised by the port partner.
 *
 * @param port Pointer to the port structure
 * @return Current in mA - or 0 if this port is the source.
 */
//AW_U16 core_get_advertised_current(Port_t *port);

/**
 * @brief Set advertised source current for this device.
 *
 * If this port is the Source device, set the currently advertised source
 * current.
 *
 * @param port Pointer to the port structure
 * @param value Source current enum requested
 * @return None
 */
void core_set_advertised_current(Port_t *port, AW_U8 value);


/**
 * @brief Return current CC orientation
 *
 *  0 = No CC pin
 *  1 = CC1 is the CC pin
 *  2 = CC2 is the CC pin
 *
 * @param port Pointer to the port structure
 * @return CC orientation
 */
AW_U8 core_get_cc_orientation(Port_t *port);

/**
 * @brief High level port configuration
 *
 * DRP is automatically enabled when setting Try.*
 * Try.* is automatically disabled when setting DRP/SRC/SNK
 *
 * @param port Pointer to the port structure
 * @return None
 */
void core_set_drp(Port_t *port);
void core_set_try_snk(Port_t *port);
void core_set_try_src(Port_t *port);
void core_set_source(Port_t *port);
void core_set_sink(Port_t *port);

#endif /* _AW_CORE_H */

