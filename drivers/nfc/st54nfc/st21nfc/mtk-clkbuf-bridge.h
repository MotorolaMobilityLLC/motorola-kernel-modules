/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#ifndef __MTK_CLK_BUF_BRIDGE_H__
#define __MTK_CLK_BUF_BRIDGE_H__

enum clk_buf_ret_type {
	CLK_BUF_DISABLE = 0,
	CLK_BUF_ENABLE  = 1,
	CLK_BUF_NOT_SUPPORT = 2,
	CLK_BUF_OK = 3,
	CLK_BUF_FAIL = 4,
};

/* clk_buf_id: users of clock buffer */
#ifndef _clk_buf_id_
#define _clk_buf_id_
enum clk_buf_id {
	CLK_BUF_BB_MD		= 0,
	CLK_BUF_CONN,
	CLK_BUF_NFC,
	CLK_BUF_RF,
	CLK_BUF_UFS		= 6,
	CLK_BUF_INVALID
};
#endif
/* xo_id: clock buffer list */
#ifndef _xo_id_
#define _xo_id_
enum xo_id {
	XO_SOC	= 0,
	XO_WCN,
	XO_NFC,
	XO_CEL,
	XO_AUD,		/* Disabled */
	XO_PD,		/* Disabled */
	XO_EXT,		/* UFS */
	XO_NUMBER
};
#endif
enum {
	BBLPM_SKIP = (1 << 0),
	BBLPM_WCN = (1 << XO_WCN),
	BBLPM_NFC = (1 << XO_NFC),
	BBLPM_CEL = (1 << XO_CEL),
	BBLPM_EXT = (1 << XO_EXT),
};

/*******************************************************************************
 * Bridging from platform -> clkbuf.ko
 ******************************************************************************/
typedef u8 (*clk_buf_bridge_get_xo_ctrl_cb)(enum xo_id);
typedef int (*clk_buf_bridge_get_enter_bblpm_cond_cb)(u32 *bblpm_cond);
typedef int (*clk_buf_bridge_set_xo_ctrl_cb)(enum clk_buf_id, bool);
typedef int (*clk_buf_bridge_set_flight_mode_cb)(bool);
typedef int (*clk_buf_bridge_set_bblpm_cb)(bool);
typedef void (*clk_buf_bridge_dump_log_cb)(void);


struct clk_buf_bridge {
	clk_buf_bridge_get_xo_ctrl_cb get_xo_ctrl_cb;
	clk_buf_bridge_get_enter_bblpm_cond_cb get_bblpm_enter_cond_cb;
	clk_buf_bridge_set_xo_ctrl_cb set_xo_ctrl_cb;
	clk_buf_bridge_set_flight_mode_cb set_flight_mode_cb;
	clk_buf_bridge_set_bblpm_cb set_bblpm_cb;
	clk_buf_bridge_dump_log_cb dump_log_cb;
};

void clk_buf_export_platform_bridge_register(struct clk_buf_bridge *cb);
void clk_buf_export_platform_bridge_unregister(void);
extern enum clk_buf_ret_type clk_buf_ctrl(enum clk_buf_id id, bool onoff);
extern enum clk_buf_ret_type clk_buf_set_by_flightmode(bool on);
extern void clk_buf_control_bblpm(bool on);
extern void clk_buf_dump_clkbuf_log(void);
extern u8 clk_buf_get_xo_en_sta(enum xo_id id);
extern u32 clk_buf_bblpm_enter_cond(void);

#endif

