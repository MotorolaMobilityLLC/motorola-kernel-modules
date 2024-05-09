// SPDX-License-Identifier: GPL-2.0
#include "aw35615_global.h"

struct aw35615_chip *g_chip;  /* Our driver's relevant data */

struct aw35615_chip *aw35615_GetChip(void)
{
	return g_chip;      /*return a pointer to our structs */
}

void aw35615_SetChip(struct aw35615_chip *newChip)
{
	g_chip = newChip;   /*assign the pointer to our struct */
}
