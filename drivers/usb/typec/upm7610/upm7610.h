/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LINUX_UPM7610_H
#define __LINUX_UPM7610_H

#include "inc/std_tcpci_v10.h"
#include "inc/pd_dbg_info.h"

/*show debug message or not */
#define ENABLE_UPM7610_DBG					0

/* UPM7610 Private RegMap */
#define UPM7610_REG_VCONN_CTRL				(0x90)
#define UPM7610_REG_OTP_CTRL				(0x91)
#define UPM7610_REG_CC_CTRL					(0x92)

#define UPM7610_REG_VDR_DEF_STATUS			(0x97)
#define UPM7610_REG_VDR_DEF_ALERT			(0x98)
#define UPM7610_REG_VDR_DEF_ALERT_MASK		(0x99)

#define UPM7610_REG_RESET_CTRL				(0xA0)

#define UPM7610_REG_HIDDEN_MODE				(0xB0)
#define UPM7610_REG_DEBUG_B9				(0xB9)
#define UPM7610_REG_TRIM_R4					(0xC4)
#define UPM7610_REG_TRIM_R5					(0xC5)
#define UPM7610_REG_TRIM_C6					(0xC6)

/*
 * Device ID
 */
#define UPM7610_DID		0x0100

/*
 * UPM7610_REG_VCONN_CTRL 			(0x90)
 */
#define UPM7610_REG_VCONN_DISCHG_EN			(1<<5)

/*
 * UPM7610_REG_OTP_CTRL 			(0x91)
 */
#define UPM7610_REG_OTP_EN					(1<<0)

/*
 * UPM7610_REG_CC_CTRL 				(0x92)
 */
#define UPM7610_REG_DISABLED_REQ			(1<<6)

/*
 * UPM7610_REG_VENDOR_DEFINED_STATUS (0x97)
 */
#define UPM7610_REG_REF_STATUS				(1<<7)
#define UPM7610_REG_OTP_STATUS				(1<<4)
#define UPM7610_REG_VSAFE0V_STATUS			(1<<1)

/*
 * UPM7610_REG_VENDOR_DEFINED_ALERT (0x98)
 */
#define UPM7610_REG_REF_DISCNT				(1<<7)
#define UPM7610_REG_RA_DETACH				(1<<5)
#define UPM7610_REG_OTP						(1<<4)
#define UPM7610_REG_CC_OVP					(1<<2)
#define UPM7610_REG_VSAFE0V_STATUS_CHG		(1<<1)

/*
 * UPM7610_REG_VENDOR_DEFINED_ALERT_MASK (0x99)
 */
#define UPM7610_REG_REF_DISCNT_MASK				(1<<7)
#define UPM7610_REG_RA_DETACH_MASK				(1<<5)
#define UPM7610_REG_OTP_MASK					(1<<4)
#define UPM7610_REG_CC_OVP_MASK					(1<<2)
#define UPM7610_REG_VSAFE0V_STATUS_MASK			(1<<1)

/*
 * UPM7610_REG_HIDDEN_MODE (0xB0)
 */
#define UPM7610_REG_HMODE_ENTER				(0x6E)
#define UPM7610_REG_HMODE_EXIT				(0x00)

/*
 * UPM7610_REG_DEBUG_B9 (0xB9)
 */
#define UPM7610_REG_FUNC_SW_MASK				(0xF0)
#define UPM7610_REG_FUNC_SW_SHIFT				(4)
#define UPM7610_REG_FUNC_SW_ID				    (0x1)

/*
 * UPM7610_REG_TRIM_R5 (0xC5)
 */
#define UPM7610_TRIM_R5_WIN_MASK				(0x07)
#define UPM7610_TRIM_R5_WIN_ID				    (0x03)

/*
 * UPM7610_REG_TRIM_C6 (0xC6)
 */
#define UPM7610_TRIM_C6_WIN_MASK				(0x03)
#define UPM7610_TRIM_C6_WIN_ID				    (0x02)

#define TCPC_V10_REG_EXTENDED_STATUS			(1<<13)


#if ENABLE_UPM7610_DBG
#define UPM7610_INFO(format, args...) \
	pd_dbg_info("%s() line-%d: " format,\
	__func__, __LINE__, ##args)
#else
#define UPM7610_INFO(foramt, args...)
#endif

#endif /* #ifndef __LINUX_UPM7610_H */
