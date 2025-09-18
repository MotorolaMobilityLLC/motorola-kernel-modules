/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************/
#include "aw35615_global.h"

struct aw35615_chip *g_chip[NUM_PORTS]; /* Our driver's relevant data */

struct aw35615_chip *aw35615_GetChip(AW_U8 portId)
{
	return g_chip[portId];      /*return a pointer to our structs */
}

void aw35615_SetChip(struct aw35615_chip *newChip)
{
	if (newChip != NULL) {
		g_chip[newChip->port.PortID] = newChip;   /*assign the pointer to our struct */
	}
}
