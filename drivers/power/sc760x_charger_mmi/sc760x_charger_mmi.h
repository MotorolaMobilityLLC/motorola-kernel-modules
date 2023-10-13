/*
 * Copyright (c) 2022 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SC760X_CHARGER_H
#define _SC760X_CHARGER_H

#include <linux/i2c.h>
#include <charger_class.h>

#define SC760X_MANUFACTURER	"South Chip"
#define SC760X_NAME		"sc7603"

#define SC760X_DRV_VERSION              "1.0.0_G"

enum {
    SC760X_MASTER = 0,
    SC760X_SLAVE,
};

#define IBAT_CHG_LIM_BASE               50
#define IBAT_CHG_LIM_LSB                50

#define ITRICHG_BASE                    12500//uA
#define ITRICHG_LSB                     12500//uA

#define IPRECHG_BASE                    50
#define IPRECHG_LSB                     50

#define VFC_CHG_BASE                    2800
#define VFC_CHG_LSB                     50

#define BAT_OVP_BASE                    4000
#define BAT_OVP_LSB                     50

#define SC760x_TERMCHRG_I_DEF_uA		180000
#define SC760x_ICHRG_I_MAX_uA			3780000
#define SC760x_ICHRG_I_DEF_uA			2000000
#define SC760x_VREG_V_MAX_uV	    4600000

typedef enum {
    ADC_IBAT,
    ADC_VBAT,
    ADC_VCHG,
    ADC_TBAT,
    ADC_TDIE,
    ADC_MAX_NUM,
}ADC_CH;

//work mode
typedef enum {
    WORK_FULLY_OFF = 0,
    WORK_TRICKLE_CHARGE,
    WORK_PRE_CHARGE,
    WORK_FULLY_ON,
    WORK_CURRENT_REGULATION,
    WORK_POWER_REGULATION,
}WORK_MODE_T;

//work mode
typedef enum {
    DISCHARGING = 0,
    CHARGING,
    NOT_CHARGING,
}CHG_STAT_T;

enum sc760x_fields {
    DEVICE_REV, DEVICE_ID,
    IBAT_CHG_LIM,
    POW_LIM_DIS, VBALANCE_DIS, POW_LIM,
    ILIM_DIS, IBAT_CHG_LIM_DIS, BAT_DET_DIS, VDIFF_CHECK_DIS, LS_OFF, SHIP_EN,
    REG_RST, EN_LOWPOWER, VDIFF_OPEN_TH, AUTO_BSM_DIS, AUTO_BSM_TH, SHIP_WT,
    ITRICHG, VPRE_CHG,
    IPRECHG, VFC_CHG,
    CHG_OVP_DIS, CHG_OVP,
    BAT_OVP_DIS, BAT_OVP,
    CHG_OCP_DIS, CHG_OCP,
    DSG_OCP_DIS, DSG_OCP,
    TDIE_FLT_DIS, TDIE_FLT,
    TDIE_ALRM_DIS, TDIE_ALRM,
    CHG_OVP_DEG, BAT_OVP_DEG, CHG_OCP_DEG, DSG_OCP_DEG,
    AUTO_BSM_DEG,
    WORK_MODE, BAT_ABSENT_STAT, VDUFF_STAT,
    ADC_EN, ADC_RATE, ADC_FREEZE, ADC_DONE_MASK,
    F_MAX_FIELDS,
};

struct sc760x_chg_platform_data {
    int bat_chg_lim_disable;
    int bat_chg_lim;
    int pow_lim_disable;
    int ilim_disable;
    int load_switch_disable;
    int lp_mode_enable;
    int itrichg;
    int iprechg;
    int vfc_chg;
    int chg_ovp_disable;
    int chg_ovp;
    int bat_ovp_disable;
    int bat_ovp;
    int chg_ocp_disable;
    int chg_ocp;
    int dsg_ocp_disable;
    int dsg_ocp;
    int tdie_flt_disable;
    int tdie_alm_disable;
    int tdie_alm;
    const char *chg_name;
};

struct sc760x_init_data {
	u32 ichg;	/* charge current		*/
	u32 iterm;	/* termination current		*/
	u32 max_ichg;
	u32 max_vreg;
	u32 vrechg;
	bool charger_disabled;
};

struct sc760x_state {
	int vbus_adc;
	int ibus_adc;
	int ibat_adc;
	int vbat_adc;
	int vchg_adc;
	int tbat_adc;
	int tdie_adc;
	WORK_MODE_T work_mode;
};

struct sc760x_chip {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];
    struct mutex lock;
    struct mutex i2c_rw_lock;
    struct power_supply		*charger_psy;
    struct wakeup_source *charger_wakelock;
    struct charger_device *chgdev;

    struct sc760x_chg_platform_data *pdata;
    struct sc760x_init_data init_data;
    struct sc760x_state state;
    int user_ichg; /* user charge current		*/
    int user_ilim; /* user input current		*/
    int user_chg_en; /* user charging enable	*/
    int user_chg_susp; /* user charger suspend	*/
    int user_gpio_en; /* user gpio en	*/

    int irq;
    bool irq_enabled;
    int role;
    int sc760x_enable;

    struct delayed_work charge_monitor_work;
};

#endif /* _SC760X_CHARGER_H */
