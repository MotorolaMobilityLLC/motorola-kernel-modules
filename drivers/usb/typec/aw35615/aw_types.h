/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *Copyright (C), 2020-2021, Shanghai awinic technology Co.,Ltd.
 *all rights reserved
 *******************************************************************************
 * File Name     : aw_type.h
 * Author        : awinic
 * Date          : 2021-11-30
 * Description   : .h file function description
 * Version       : V1.0
 * Function List :
 ******************************************************************************/
#ifndef __AW_TYPES_H__
#define __AW_TYPES_H__
#include <linux/module.h>

#ifndef AW_CONST
#define AW_CONST		const
#endif

#ifndef AW_ZERO
#define AW_ZERO			(0)
#endif

#ifndef AW_NULL
#define AW_NULL			((void *)0)
#endif

#ifndef AW_FALSE
#define AW_FALSE		(0)
#endif

#ifndef AW_TRUE
#define AW_TRUE			(1)
#endif

#ifndef AW_FAIL
#define AW_FAIL			(-1)
#endif

#ifndef AW_RET
#define AW_RET			(1)
#endif

#ifndef AW_OK
#define AW_OK			(0)
#endif

#ifndef AW_BOOL
typedef unsigned char AW_BOOL;
#endif

#ifndef AW_S8
typedef signed char AW_S8;
#endif

#ifndef AW_S16
typedef signed short AW_S16;
#endif

#ifndef AW_S32
typedef signed int AW_S32;
#endif

#ifndef AW_S64
typedef signed long long AW_S64;
#endif

#ifndef AW_U8
typedef unsigned char AW_U8;
#endif

#ifndef AW_U16
typedef unsigned short AW_U16;
#endif

#ifndef AW_U32
typedef unsigned int AW_U32;
#endif

#ifndef AW_U64
typedef unsigned long long AW_U64;
#endif

#define AWINIC_DEBUG
#ifdef AWINIC_DEBUG
#define AWINIC_LOG_NAME "AW35615"
#define AW_LOG(format, arg...)	 pr_err("[%s] %s %d: " format, AWINIC_LOG_NAME, \
		 __func__, __LINE__, ##arg)
#else
#define AW_LOG(format, arg...)
#endif

#endif  /* __AW_TYPES_H__ */

