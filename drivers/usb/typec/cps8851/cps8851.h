/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 Motorola Mobility LLC.
 */

#ifndef __LINUX_CPS8851_H
#define __LINUX_CPS8851_H

#include "inc/std_tcpci_v10.h"
#include "inc/pd_dbg_info.h"

/*show debug message or not */
#define ENABLE_CPS8851_DBG	0

#define CONFIG_TCPC_VSAFE0V_DETECT_IC 1

/* CPS8851 Private RegMap */

#define CPS8851_REG_CONFIG_GPIO0			(0x71)

#define CPS8851_REG_PHY_CTRL1				(0x80)

#define CPS8851_REG_CLK_CTRL2				(0x87)
#define CPS8851_REG_CLK_CTRL3				(0x88)

#define CPS8851_REG_PRL_FSM_RESET			(0x8D)

#define CPS8851_REG_BMC_CTRL				(0x90)
#define CPS8851_REG_BMCIO_RXDZSEL			(0x93)
#define CPS8851_REG_VCONN_CLIMITEN			(0x95)

#define CPS8851_REG_RT_STATUS				(0x97)
#define CPS8851_REG_RT_INT					(0x98)
#define CPS8851_REG_RT_MASK					(0x99)

#define CPS8851_REG_CP_STATUS				(0x97)
#define CPS8851_REG_CP_INT					(0x98)
#define CPS8851_REG_CP_MASK					(0x99)

#define CPS8851_REG_IDLE_CTRL				(0x9B)
#define CPS8851_REG_INTRST_CTRL				(0x9C)
#define CPS8851_REG_WATCHDOG_CTRL			(0x9D)
#define CPS8851_REG_I2CRST_CTRL				(0X9E)

#define CPS8851_REG_SWRESET				(0xA0)
#define CPS8851_REG_TTCPC_FILTER			(0xA1)
#define CPS8851_REG_DRP_TOGGLE_CYCLE		(0xA2)
#define CPS8851_REG_DRP_DUTY_CTRL			(0xA3)
#define CPS8851_REG_CP_OTSD					(0xA5)
#define CPS8851_REG_CP_OTSD_MASK			(0xA6)
#define CPS8851_REG_BMCIO_RXDZEN			(0xAF)

#define CPS8851_REG_PASSWORD				(0xB0)
#define CPS8851_REG_PD_SDAC					(0xB5)
#define CPS8851_REG_PD_TCPC					(0xB8)
#define CPS8851_REG_PD_OPT_ZTX_SEL			(0xD0)
#define CPS8851_REG_PD_OPT_TX_SLEW_SEL		(0xD1)
#define CPS8851_REG_PD_OPT_DB_IBIAS_EN		(0xD3)

#define CPS8851_REG_UNLOCK_PW_2				(0xF0)
#define CPS8851_REG_UNLOCK_PW_1				(0xF1)
#define CPS8851_REG_EFUSE5				(0xF6)

/*
 * Device ID
 */

#define RT1711H_DID_A		0x2170
#define RT1711H_DID_B		0x2171
#define RT1711H_DID_C		0x2172

#define RT1715_DID_D			0x2173

#define CPS8851_DID_A		0x2173

/*
 * CPS8851_REG_VBUS_VOLTAGE			(0x71)
 */

#define CPS8851_REG_VBUS_VSAFE0V	(0x00)
#define CPS8851_REG_VBUS_PRESENT	(0x01)
#define CPS8851_REG_VBUS_9V			(0x02)
#define CPS8851_REG_VBUS_12V		(0x03)
#define CPS8851_REG_VBUS_15V		(0x04)
#define CPS8851_REG_VBUS_20V		(0x05)

/*
 * CPS8851_REG_VDDIO_SEL			(0x7C)
 */

#define CPS8851_REG_VDDIO_1P2V		(0x00)
#define CPS8851_REG_VDDIO_1P8V		(0x01)

/*
 * CPS8851_REG_VCONNOCP				(0x93)
 */

#define CPS8851_VCONNOCP_LEVEL_200MA	(0x00)
#define CPS8851_VCONNOCP_LEVEL_300MA	(0x01)
#define CPS8851_VCONNOCP_LEVEL_400MA	(0x02)
#define CPS8851_VCONNOCP_LEVEL_500MA	(0x03)
#define CPS8851_VCONNOCP_LEVEL_600MA	(0x04) /* default */
#define CPS8851_REG_VCONNOCP_SHIFT		(0x05)
#define CPS8851_REG_VCONNOCP_SET(level)	\
	((level) << CPS8851_REG_VCONNOCP_SHIFT)

/*
 * CPS8851_REG_PHY_CTRL1			(0x80)
 */

#define CPS8851_REG_PHY_CTRL1_SET( \
	retry_discard, toggle_cnt, bus_idle_cnt, rx_filter) \
	((retry_discard << 7) | (toggle_cnt << 4) | \
	(bus_idle_cnt << 2) | (rx_filter & 0x03))

/*
 * CPS8851_REG_CLK_CTRL2			(0x87)
 */

#define CPS8851_REG_CLK_DIV_600K_EN		(1<<7)
#define CPS8851_REG_CLK_BCLK2_EN		(1<<6)
#define CPS8851_REG_CLK_BCLK2_TG_EN		(1<<5)
#define CPS8851_REG_CLK_DIV_300K_EN		(1<<3)
#define CPS8851_REG_CLK_CK_300K_EN		(1<<2)
#define CPS8851_REG_CLK_BCLK_EN			(1<<1)
#define CPS8851_REG_CLK_BCLK_TH_EN		(1<<0)

/*
 * CPS8851_REG_CLK_CTRL3			(0x88)
 */

#define CPS8851_REG_CLK_OSCMUX_RG_EN	(1<<7)
#define CPS8851_REG_CLK_CK_24M_EN		(1<<6)
#define CPS8851_REG_CLK_OSC_RG_EN		(1<<5)
#define CPS8851_REG_CLK_DIV_2P4M_EN		(1<<4)
#define CPS8851_REG_CLK_CK_2P4M_EN		(1<<3)
#define CPS8851_REG_CLK_PCLK_EN			(1<<2)
#define CPS8851_REG_CLK_PCLK_RG_EN		(1<<1)
#define CPS8851_REG_CLK_PCLK_TG_EN		(1<<0)

/*
 * CPS8851_REG_BMC_CTRL				(0x90)
 */

#define CPS8851_REG_IDLE_EN				(1<<6)
#define CPS8851_REG_DISCHARGE_EN			(1<<5)
#define CPS8851_REG_BMCIO_LPRPRD			(1<<4)
#define CPS8851_REG_BMCIO_LPEN				(1<<3)
#define CPS8851_REG_BMCIO_BG_EN				(1<<2)
#define CPS8851_REG_VBUS_DET_EN				(1<<1)
#define CPS8851_REG_BMCIO_OSC_EN			(1<<0)

/*
 * CPS8851_REG_RT_STATUS				(0x97)
 */

#define CPS8851_REG_RA_DETACH				(1<<5)
#define CPS8851_REG_VBUS_80				(1<<1)

/*
 * CPS8851_REG_RT_INT				(0x98)
 */

#define CPS8851_REG_INT_RA_DETACH			(1<<5)
#define CPS8851_REG_INT_WATCHDOG			(1<<2)
#define CPS8851_REG_INT_VBUS_80				(1<<1)
#define CPS8851_REG_INT_WAKEUP				(1<<0)

/*
 * CPS8851_REG_RT_MASK				(0x99)
 */

#define CPS8851_REG_M_RA_DETACH				(1<<5)
#define CPS8851_REG_M_WATCHDOG				(1<<2)
#define CPS8851_REG_M_VBUS_80				(1<<1)
#define CPS8851_REG_M_WAKEUP				(1<<0)

/*
 * CPS8851_REG_IDLE_CTRL				(0x9B)
 */

#define CPS8851_REG_CK_300K_SEL				(1<<7)
#define CPS8851_REG_SHIPPING_OFF			(1<<5)
#define CPS8851_REG_ENEXTMSG				(1<<4)
#define CPS8851_REG_AUTOIDLE_EN				(1<<3)

/*
 * CPS8851_REG_EFUSE5					(0xF6)
 */

#define CPS8851_REG_M_VBUS_CAL				GENMASK(7, 5)
#define CPS8851_REG_S_VBUS_CAL				5
#define CPS8851_REG_MIN_VBUS_CAL			-4

#define CPS8851_REG_WATCHDOG_TO_SHIFT		(6)
#define CPS8851_REG_WDT_TO_1S				(0)
#define CPS8851_REG_WDT_TO_2S				(1)

/* timeout = (tout*2+1) * 6.4ms, watchdog timeout = wdt_tout + 1s */

#define CPS8851_REG_IDLE_SET(dummy, ship_dis, auto_idle, tout, wdt_tout) \
	((ship_dis << 5) | (auto_idle << 3) | \
	((wdt_tout) << CPS8851_REG_WATCHDOG_TO_SHIFT) | (tout & 0x07))

/*
 * CPS8851_REG_INTRST_CTRL			(0x9C)
 */

#define CPS8851_REG_INTRST_EN				(1<<7)

/* timeout = (tout+1) * 0.2sec */
#define CPS8851_REG_INTRST_SET(en, tout) \
	((en << 7) | (tout & 0x03))

/*
 * CPS8851_REG_WATCHDOG_CTRL		(0x9D)
 */

#define CPS8851_REG_WATCHDOG_EN				(1<<7)

/* timeout = (tout+1) * 0.4sec */
#define CPS8851_REG_WATCHDOG_CTRL_SET(en, tout)	\
	((en << 7) | (tout & 0x07))

/*
 * CPS8851_REG_I2CRST_CTRL		(0x9E)
 */

#define CPS8851_REG_I2CRST_EN				(1<<7)

/*
 * CPS8851_REG_CP_OTSD					(0xA5)
 */

#define CPS8851_REG_OTSD_STATE			(1<<2)

/*
 * CPS8851_REG_CP_OTSD_MASK			(0xA6)
 */

#define CPS8851_REG_M_OTSD_STATE			(1<<2)

/* timeout = (tout+1) * 12.5ms */
#define CPS8851_REG_I2CRST_SET(en, tout)	\
	((en << 7) | (tout & 0x0f))

#if ENABLE_CPS8851_DBG
#define CPS8851_INFO(format, args...) \
	pd_dbg_info("%s() line-%d: " format,\
	__func__, __LINE__, ##args)
#else
#define CPS8851_INFO(foramt, args...)
#endif

#endif /* #ifndef __LINUX_CPS8851_H */
