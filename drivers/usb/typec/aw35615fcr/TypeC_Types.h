/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-12-17
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __AW_TYPEC_TYPES_H__
#define __AW_TYPEC_TYPES_H__

/**
 * @brief Port type defines whether the port is Source, Sink, or DRP
 */
typedef enum {
	USBTypeC_Sink = 0,
	USBTypeC_Source,
	USBTypeC_DRP,
	USBTypeC_Debug,
	USBTypeC_UNDEFINED = 99
} USBTypeCPort;

/**
 * CC pin orientation
 */
typedef enum {
	CCNone,
	CC1,
	CC2
} CCOrientation;

/**
 * @brief Type-C state machine state enum
 */
typedef enum {
	Disabled = 0,
	ErrorRecovery,
	Unattached,
	AttachWaitSink,
	AttachedSink,
	AttachWaitSource,
	AttachedSource,
	TrySource,
	TryWaitSink,
	TrySink,
	TryWaitSource,
	AudioAccessory,
	DebugAccessorySource,
	AttachWaitAccessory,
	PoweredAccessory,
	UnsupportedAccessory,
	DelayUnattached,
	UnattachedSource,
	DebugAccessorySink,
	AttachWaitDebSink,
	AttachedDebSink,
	AttachWaitDebSource,
	AttachedDebSource,
	TryDebSource,
	TryWaitDebSink,
	UnattachedDebSource,
	IllegalCable,
	AttachVbusOnlyok,
} ConnectionState;

/**
 * @brief Defines possible CC pin terminations (from either side)
 */
typedef enum {
	CCTypeOpen = 0,
	CCTypeRa,
	CCTypeRdUSB,
	CCTypeRd1p5,
	CCTypeRd3p0,
	CCTypeUndefined
} CCTermType;

/**
 * @brief Defines the possible source current advertisements
 */
typedef enum {
	utccNone = 0,
	utccDefault,
	utcc1p5A,
	utcc3p0A,
	utccInvalid,
} USBTypeCCurrent;

#endif /* __AW_TYPEC_TYPES_H__ */

