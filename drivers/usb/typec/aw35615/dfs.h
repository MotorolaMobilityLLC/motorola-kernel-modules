/* SPDX-License-Identifier: GPL-2.0 */
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
AW_S32 aw_DFS_Init(void);

AW_S32 aw_DFS_Cleanup(void);

#endif /* AW_DEBUG */

#endif /* _AW_DFS_H_ */
