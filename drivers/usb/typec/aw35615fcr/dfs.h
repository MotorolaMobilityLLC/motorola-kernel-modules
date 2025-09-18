/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************/
#ifndef _AW_DFS_H_
#define _AW_DFS_H_

#ifdef AW_DEBUG

#include "aw_types.h"

/*******************************************************************************
 * Function:        aw_DFS_Init
 * Input:           none
 * Return:          0 on success, error code otherwise
 * Description:     Initializes methods for using DebugFS.
 *******************************************************************************/
AW_S32 aw_DFS_Init(struct aw35615_chip *chip);

AW_S32 aw_DFS_Cleanup(struct aw35615_chip *chip);

#endif /* AW_DEBUG */

#endif /* _AW_DFS_H_ */
