// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022-2025 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regmap.h>
#include <linux/version.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/iio/consumer.h>
#include <linux/phy/phy.h>
#if IS_ENABLED(CONFIG_OEM_DEVINFO)
#include <dev_info.h>
#endif

#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

#include "charger_class.h"
#include "mtk_charger.h"
#ifdef CONFIG_MTK_CHARGER_V4P19
#include "charger_type.h"
#include "mtk_charger_intf.h"
#endif /*CONFIG_MTK_CHARGER_V4P19*/

#define SC8989X_DRV_VERSION		"1.0.0_G"
#define SC8989X_CHIP_ID 			1
#define R_VBUS_CHARGER_1   330
#define R_VBUS_CHARGER_2   39
#define SPECIAL_TYPE_MAX_RETRY 1
#define MAX_TRY  100

static struct proc_dir_entry *entry;
static bool dump_reg_enable;
enum sc8960x_part_no {
	SC89890H_PN_NUM = 0x04,
	UPM6920A_PN_NUM = 0x03,
	SC89895_PN_NUM = 0x04,
	SC8950_PN_NUM = 0x02,
	SC89890W_PN_NUM = 0x07,
};

enum vindpm_track {
	SC8989X_TRACK_DIS,
	SC8989X_TRACK_200,
	SC8989X_TRACK_250,
	SC8989X_TRACK_300,
};

#define SC8989X_VINDPM_MAX	15300 //mV

#define SC8989X_REG7D		0x7D
#define SC8989X_KEY1		0x48
#define SC8989X_KEY2		0x54
#define SC8989X_KEY3		0x53
#define SC8989X_KEY4		0x38

#define SC8989X_ADC_EN		0x87
#define SC8989X_DPDM3		0x88
#define SC8989X_PRIVATE		0x99

#define MAX_REG_NUM		0x14
#define SINGLE_DUMP_LEN		22
#define TOTAL_DUMP_LEN		(SINGLE_DUMP_LEN * (MAX_REG_NUM + 1))

#define UPM6920A_REG_CFG_MODE     0xA9
#define UPM6920A_CFG_MODE_ENABLE  0x6E
#define UPM6920A_CFG_MODE_DISABLE 0x00

#define MMI_HVDCP_DETECT_TIMER       1500
#define QC3P_AUTHEN_LOW_THR_MV       5500000
#define QC3P_AUTHEN_HIGH_THR_MV      6500000
#define QC3P_AUTHEN_NONE_THR_MV      9200000
#define QC3P_AUTHEN_45W_THR_MV       8200000
#define QC3P_AUTHEN_27W_THR_MV       7200000
#define QC3P_AUTHEN_18W_THR_MV       6200000
#define MMI_HVDCP2_VOLTAGE_STANDARD  8000000
#define MMI_HVDCP3_VOLTAGE_STANDARD  7500000
#define QC3P_MSLEEP_1500DELAY        1500
#define QC3P_MSLEEP_500DELAY         500
#define QC3P_MSLEEP_300DELAY         300
#define QC3P_MSLEEP_100DELAY         100
#define QC3P_MSLEEP_30DELAY          30
#define QC3P_UDELAY_2500DELAY        2500
#define DPDM_DRIVE_3V3               6
#define DPDM_DRIVE_0V6               2

#define DEFAULT_HIZ_CUT_TIME_EXPRIE 300UL

enum sc8989x_vbus_stat {
	VBUS_STAT_NO_INPUT = 0,
	VBUS_STAT_SDP,
	VBUS_STAT_CDP,
	VBUS_STAT_DCP,
	VBUS_STAT_HVDCP,
	VBUS_STAT_UNKOWN,
	VBUS_STAT_NONSTAND,
	VBUS_STAT_OTG,
};

enum sc8989x_chg_stat {
	CHG_STAT_NOT_CHARGE = 0,
	CHG_STAT_PRE_CHARGE,
	CHG_STAT_FAST_CHARGE,
	CHG_STAT_CHARGE_DONE,
};

enum sc8989x_adc_channel {
	SC8989X_ADC_VBAT,
	SC8989X_ADC_VSYS,
	SC8989X_ADC_VBUS,
	SC8989X_ADC_ICC,
	SC8989X_ADC_IBUS,
};

enum {
	SC8989X_VBUSSTAT_NOINPUT = 0,
	SC8989X_VBUSSTAT_SDP,
	SC8989X_VBUSSTAT_CDP,
	SC8989X_VBUSSTAT_DCP,
	SC8989X_VBUSSTAT_HVDCP,
	SC8989X_VBUSSTAT_FLOAT,
	SC8989X_VBUSSTAT_NON_STD,
	SC8989X_VBUSSTAT_OTG,
};

enum sc8989x_fields {
	EN_HIZ, EN_ILIM, IINDPM,
	DP_DRIVE, DM_DRIVE, VINDPM_OS,
	CONV_START, CONV_RATE, BOOST_FRE, ICO_EN, HVDCP_EN, FORCE_DPDM,
	AUTO_DPDM_EN,
	FORCE_DSEL, WD_RST, OTG_CFG, CHG_CFG, VSYS_MIN, VBATMIN_SEL,
	EN_PUMPX, ICC,
	ITC, ITERM,
	CV, VBAT_LOW, VRECHG,
	EN_ITERM, STAT_DIS, TWD, EN_TIMER, TCHG, JEITA_ISET,
	BAT_COMP, VCLAMP, TJREG,
	FORCE_ICO, TMR2X_EN, BATFET_DIS, JETTA_VSET_WARM, BATFET_DLY, BATFET_RST_EN,
	PUMPX_UP, PUMPX_DN,
	V_OTG, PFM_OTG_DIS, IBOOST_LIM,
	VBUS_STAT, CHG_STAT, PG_STAT, VSYS_STAT,
	FORCE_VINDPM, VINDPM,
	ADC_VBAT,
	ADC_VSYS,
	VBUS_GD,ADC_VBUS,
	BC_12_DONE, ADC_ICC,
	VINDPM_STAT, IINDPM_STAT,
	REG_RST, ICO_STAT, PN, NTC_PROFILE, DEV_VERSION,
	CX_DP_DRIVE, CX_DM_DRIVE,
	CX_OPT_HICCUP,
	VBAT_REG_LSB,
	CX_REG40,
	CX_HS_OCP,
	CX_TRILIM,
	ADC_IBUS,
	DP3P3V_DM0V_EN,
	F_VINDPM_TRACK,
	CX_RESET_INTER,
	CFGINIT_BIT,
	CFGINIT2_BIT,
	F_MAX_FIELDS,
};

enum sc8989x_reg_range {
	SC8989X_IINDPM,
	SC8989X_ICHG,
	SC8989X_IBOOST,
	SC8989X_VBAT_REG,
	SC8989X_VINDPM,
	SC8989X_ITERM,
	SC8989X_VBAT,
	SC8989X_VSYS,
	SC8989X_VBUS,
	SC8989X_ICC,
	SC8989X_IBUS,
	UPM6920A_ICHG,
	UPM6920A_ITERM,
	CX25890HQ_IINDPM,
	CX25890HQ_ICHG,
	CX25890HQ_ITERM,
};

enum attach_type {
	ATTACH_TYPE_NONE,
	ATTACH_TYPE_PWR_RDY,
	ATTACH_TYPE_TYPEC,
	ATTACH_TYPE_PD,
	ATTACH_TYPE_PD_SDP,
	ATTACH_TYPE_PD_DCP,
	ATTACH_TYPE_PD_NONSTD,
};

struct reg_range {
	u32 min;
	u32 max;
	u32 step;
	u32 offset;
	const u32 *table;
	u16 num_table;
	bool round_up;
};

struct sc8989x_cfg_e {
	const char *chg_name;
	int ico_en;
	int hvdcp_en;
	int auto_dpdm_en;
	int vsys_min;
	int vbatmin_sel;
	int itrick;
	int iterm;
	int vbat_cv;
	int vbat_low;
	int vrechg;
	int en_term;
	int stat_dis;
	int wd_time;
	int en_timer;
	int charge_timer;
	int bat_comp;
	int vclamp;
	int votg;
	int iboost;
	int force_vindpm;
	int vindpm;
	int vindpm_track;
};

/* These default values will be applied if there's no property in dts */
static struct sc8989x_cfg_e sc8989x_default_cfg = {
	.chg_name = "primary_chg",
	.ico_en = 0,
	.hvdcp_en = 1,
	.auto_dpdm_en = 1,
	.vsys_min = 5,
	.vbatmin_sel = 0,
	.itrick = 3,
	.iterm = 4,
	.vbat_cv = 23,
	.vbat_low = 1,
	.vrechg = 0,
	.en_term = 1,
	.stat_dis = 0,
	.wd_time = 0,
	.en_timer = 1,
	.charge_timer = 2,
	.bat_comp = 0,
	.vclamp = 0,
	.votg = 12,
	.iboost = 7,
	.force_vindpm = 0,
	.vindpm = 18,
	.vindpm_track = SC8989X_TRACK_250,
};

struct sc8989x_chip {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	struct charger_device *chg_dev;
#ifdef CONFIG_MTK_CHARGER_V4P19
	enum charger_type adp_type;
	struct power_supply *chg_psy;
	struct delayed_work psy_dwork;
#endif /*CONFIG_MTK_CHARGER_V4P19*/
	bool mmi_charging_full;
	int chg_type;
	int psy_usb_type;
	int irq_gpio;
	int irq;
	struct delayed_work force_detect_dwork;
	struct delayed_work ibus_enable_dwork;
	int force_detect_count;
	int power_good;
	int vbus_good;
	int is_upm6920A;
	int is_cx25890HQ;
	int is_sc89890h;
	uint8_t dev_id;
	struct power_supply_desc psy_desc;
	struct sc8989x_cfg_e *cfg;
	struct power_supply *psy;
	struct power_supply *chg_psy;
	struct iio_channel *vbus;
	int retry_count;
	atomic_t vbus_good_flag;
	atomic_t attach;
	atomic_t reset_vindpm;
	struct adapter_device *pd_adapter;
	struct mutex dpdm_lock;
	/*for external qc protocol ic such as wt6670f*/
	struct delayed_work detect_qc_dwork;
	int pulse_cnt;
	struct adapter_device *qc_dev;
	bool	qc_is_detect;
	bool	qc_is_start_detect;
	int	qc_chg_type;
	struct power_supply *batt_psy;

	/*for software HVDCP detected*/
	struct delayed_work mmi_hvdcp_detect_dwork;
	bool mmi_hvdcp_support;
	bool only_buck_qc_support;
	bool mmi_hvdcp_trig_flag;
	struct task_struct	*mmi_hvdcp_authen_task;
	wait_queue_head_t	mmi_hvdcp_wait_que;
	struct	semaphore sem_dpdm;
	int	mmi_qc3p_power;
	bool	mmi_qc3p_rerun_done;
	int otg_enable;
	int upm6920_iterm;
	int icharge_default_value;
	int cx25890HQ_iterm;
	int upm6920A_votg;
	int cx25890HQ_votg;
	int cx_otg_trilmt_init;
	bool ibus_dis;
	bool disablehiz_isset_flg;
	bool wait_hiz;
	struct delayed_work hiz_cut_dwork;
	bool hiz_cut_flag;
};

static const u32 sc8989x_iboost[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150, 2450,
};

#define SC8989X_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
	.min = _min, \
	.max = _max, \
	.step = _step, \
	.offset = _offset, \
	.round_up = _ru, \
}

#define SC8989X_CHG_RANGE_T(_table, _ru) \
	{ .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }

static const struct reg_range sc8989x_reg_range_ary[] = {
	[SC8989X_IINDPM] = SC8989X_CHG_RANGE(100, 3250, 50, 100, false),
	[SC8989X_ICHG] = SC8989X_CHG_RANGE(0, 4000, 60, 0, false),
	[SC8989X_ITERM] = SC8989X_CHG_RANGE(30, 930, 60, 30, false),
	[SC8989X_VBAT_REG] = SC8989X_CHG_RANGE(3840, 4848, 16, 3840, false),
	[SC8989X_VINDPM] = SC8989X_CHG_RANGE(3900, 15300, 100, 2600, false),
	[SC8989X_IBOOST] = SC8989X_CHG_RANGE_T(sc8989x_iboost, false),
	[SC8989X_VBAT] = SC8989X_CHG_RANGE(2304, 4848, 20, 2304, false),
	[SC8989X_VSYS] = SC8989X_CHG_RANGE(2304, 4848, 20, 2304, false),
	[SC8989X_VBUS] = SC8989X_CHG_RANGE(2600, 15300, 100, 2600, false),
	[SC8989X_ICC] = SC8989X_CHG_RANGE(0, 6350, 50, 0, false),
	[SC8989X_IBUS] = SC8989X_CHG_RANGE(0, 6350, 50, 0, false),
	[UPM6920A_ICHG] = SC8989X_CHG_RANGE(0, 4000, 64, 0, false),
	[UPM6920A_ITERM] = SC8989X_CHG_RANGE(64, 1024, 64, 64, false),
	[CX25890HQ_IINDPM] = SC8989X_CHG_RANGE(100, 3100, 50, 100, false),
	[CX25890HQ_ICHG] = SC8989X_CHG_RANGE(0, 5056, 64, 0, false),
	[CX25890HQ_ITERM] = SC8989X_CHG_RANGE(64, 1024, 64, 64, false),
};

//REGISTER
static const struct reg_field sc8989x_reg_fields[] = {
	/*reg00 */
	[EN_HIZ] = REG_FIELD(0x00, 7, 7),
	[EN_ILIM] = REG_FIELD(0x00, 6, 6),
	[IINDPM] = REG_FIELD(0x00, 0, 5),
	/*reg01 */
	[DP_DRIVE] = REG_FIELD(0x01, 5, 7),
	[DM_DRIVE] = REG_FIELD(0x01, 2, 4),
	[VINDPM_OS] = REG_FIELD(0x01, 0, 0),
	/*reg02 */
	[CONV_START] = REG_FIELD(0x02, 7, 7),
	[CONV_RATE] = REG_FIELD(0x02, 6, 6),
	[BOOST_FRE] = REG_FIELD(0x02, 5, 5),
	[ICO_EN] = REG_FIELD(0x02, 4, 4),
	[HVDCP_EN] = REG_FIELD(0x02, 3, 3),
	[FORCE_DPDM] = REG_FIELD(0x02, 1, 1),
	[AUTO_DPDM_EN] = REG_FIELD(0x02, 0, 0),
	/*reg03 */
	[FORCE_DSEL] = REG_FIELD(0x03, 7, 7),
	[WD_RST] = REG_FIELD(0x03, 6, 6),
	[OTG_CFG] = REG_FIELD(0x03, 5, 5),
	[CHG_CFG] = REG_FIELD(0x03, 4, 4),
	[VSYS_MIN] = REG_FIELD(0x03, 1, 3),
	[VBATMIN_SEL] = REG_FIELD(0x03, 0, 0),
	/*reg04 */
	[EN_PUMPX] = REG_FIELD(0x04, 7, 7),
	[ICC] = REG_FIELD(0x04, 0, 6),
	/*reg05 */
	[ITC] = REG_FIELD(0x05, 4, 7),
	[ITERM] = REG_FIELD(0x05, 0, 3),
	/*reg06 */
	[CV] = REG_FIELD(0x06, 2, 7),
	[VBAT_LOW] = REG_FIELD(0x06, 1, 1),
	[VRECHG] = REG_FIELD(0x06, 0, 0),
	/*reg07 */
	[EN_ITERM] = REG_FIELD(0x07, 7, 7),
	[STAT_DIS] = REG_FIELD(0x07, 6, 6),
	[TWD] = REG_FIELD(0x07, 4, 5),
	[EN_TIMER] = REG_FIELD(0x07, 3, 3),
	[TCHG] = REG_FIELD(0x07, 1, 2),
	[JEITA_ISET] = REG_FIELD(0x07, 0, 0),
	/*reg08 */
	[BAT_COMP] = REG_FIELD(0x08, 5, 7),
	[VCLAMP] = REG_FIELD(0x08, 2, 4),
	[TJREG] = REG_FIELD(0x08, 0, 1),
	/*reg09 */
	[FORCE_ICO] = REG_FIELD(0x09, 7, 7),
	[TMR2X_EN] = REG_FIELD(0x09, 6, 6),
	[BATFET_DIS] = REG_FIELD(0x09, 5, 5),
	[JETTA_VSET_WARM] = REG_FIELD(0x09, 4, 4),
	[BATFET_DLY] = REG_FIELD(0x09, 3, 3),
	[BATFET_RST_EN] = REG_FIELD(0x09, 2, 2),
	[PUMPX_UP] = REG_FIELD(0x09, 1, 1),
	[PUMPX_DN] = REG_FIELD(0x09, 0, 0),
	/*reg0A */
	[V_OTG] = REG_FIELD(0x0A, 4, 7),
	[PFM_OTG_DIS] = REG_FIELD(0x0A, 3, 3),
	[IBOOST_LIM] = REG_FIELD(0x0A, 0, 2),
	/*reg0B */
	[VBUS_STAT] = REG_FIELD(0x0B, 5, 7),
	[CHG_STAT] = REG_FIELD(0x0B, 3, 4),
	[PG_STAT] = REG_FIELD(0x0B, 2, 2),
	[VSYS_STAT] = REG_FIELD(0x0B, 0, 0),
	/*reg0D */
	[FORCE_VINDPM] = REG_FIELD(0x0D, 7, 7),
	[VINDPM] = REG_FIELD(0x0D, 0, 6),
	/*reg0E */
	[ADC_VBAT] = REG_FIELD(0x0E, 0, 6),
	/*reg0F */
	[ADC_VSYS] = REG_FIELD(0x0F, 0, 6),
	/*reg11 */
	[VBUS_GD] = REG_FIELD(0x11, 7, 7),
	[ADC_VBUS] = REG_FIELD(0x11, 0, 6),
	/*reg12 */
	[BC_12_DONE] = REG_FIELD(0x12, 7, 7),
	[ADC_ICC] = REG_FIELD(0x12, 0, 6),
	/*reg13 */
	[VINDPM_STAT] = REG_FIELD(0x13, 7, 7),
	[IINDPM_STAT] = REG_FIELD(0x13, 6, 6),
	/*reg14 */
	[REG_RST] = REG_FIELD(0x14, 7, 7),
	[ICO_STAT] = REG_FIELD(0x14, 6, 6),
	[PN] = REG_FIELD(0x14, 3, 5),
	[NTC_PROFILE] = REG_FIELD(0x14, 2, 2),
	[DEV_VERSION] = REG_FIELD(0x14, 0, 1),
	/*reg15 */
	[CX_DP_DRIVE] = REG_FIELD(0x15, 5, 7),
	[CX_DM_DRIVE] = REG_FIELD(0x15, 2, 4),
	/*reg17 */
	[CX_OPT_HICCUP] = REG_FIELD(0x17, 7, 7),
	/*reg40 */
	[VBAT_REG_LSB] = REG_FIELD(0x40, 6, 6),
	[CX_REG40] = REG_FIELD(0x40, 0, 7),
	/*reg47 */
	[CX_HS_OCP] = REG_FIELD(0x47, 4, 6),
	/*reg84 */
	[CX_TRILIM] = REG_FIELD(0x84, 0, 7),
	/*reg86 */
	[ADC_IBUS] = REG_FIELD(0x86, 1, 7),
	/*reg83 */
	[DP3P3V_DM0V_EN] = REG_FIELD(0x83, 5, 5),
	/*reg85 */
	[F_VINDPM_TRACK] = REG_FIELD(0x85, 1, 2),
	/*reg89 */
	[CX_RESET_INTER] = REG_FIELD(0x89, 0, 7),
	/*regC4 */
	[CFGINIT_BIT] = REG_FIELD(0xC4, 7, 7),
	/*regCB */
	[CFGINIT2_BIT] = REG_FIELD(0xCB, 0, 3),
};

static int sc8989x_regmap_write(void *context, unsigned int reg,
				       unsigned int val)
{
    struct sc8989x_chip *sc = context;
    struct i2c_client *i2c = to_i2c_client(sc->dev);
    int ret = 0;

    if (val > 0xff || reg > 0xff)
        return -EINVAL;

    if (reg == 0x2) {
        ret = i2c_smbus_read_byte_data(i2c, reg);
        ret &= 0x2;
        if (ret) {
            val &= 0xFD;
            dev_err(sc->dev, "bc12 do not finish, clear force dpdm bit, new val = 0x%x\n", val);
        }
    }

    return i2c_smbus_write_byte_data(i2c, reg, val);
}

static int sc8989x_regmap_read(void *context, unsigned int reg,
				      unsigned int *val)
{
	struct sc8989x_chip *sc = context;
	struct i2c_client *i2c = to_i2c_client(sc->dev);
	int ret;

	if (reg > 0xff)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

static const struct regmap_bus sc8989x_regmap_bus = {
	.reg_write = sc8989x_regmap_write,
	.reg_read = sc8989x_regmap_read,
};

static const struct regmap_config sc8989x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static bool is_pd_rdy(struct sc8989x_chip *sc);
static int sc8989x_get_vbus(struct charger_device *chgdev, u32 *vbus);

/********************COMMON API***********************/
static u8 val2reg(enum sc8989x_reg_range id, u32 val)
{
	int i;
	u8 reg;
	const struct reg_range *range = &sc8989x_reg_range_ary[id];

	if (!range)
		return val;

	if (range->table) {
		if (val <= range->table[0])
			return 0;
		for (i = 1; i < range->num_table - 1; i++) {
			if (val == range->table[i])
				return i;
			if (val > range->table[i] && val < range->table[i + 1])
				return range->round_up ? i + 1 : i;
		}
		return range->num_table - 1;
	}
	if (val <= range->min)
		reg = (range->min - range->offset) / range->step;
	else if (val >= range->max)
		reg = (range->max - range->offset) / range->step;
	else if (range->round_up)
		reg = (val - range->offset) / range->step + 1;
	else
		reg = (val - range->offset) / range->step;
	return reg;
}



static bool is_factory_build(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;
	const char *bootargs = NULL;
	const char *mmi_bootconfig = NULL;
	char *bl_version = NULL;
	char *end = NULL;
	bool mmi_bootconfig_set = false;

	if (!np)
		goto err_putnode1;
	if (!of_property_read_string(np, "bootargs", &bootargs)) {
			bl_version = strstr(bootargs, "androidboot.bootloader=");
			if (bl_version) {
				end = strpbrk(bl_version, " ");
				bl_version = strpbrk(bl_version, "=");
			} else {
				mmi_bootconfig_set = true;
			}

			if (bl_version && end > bl_version &&
			    strnstr(bl_version, "factory", end - bl_version)) {
				factory = true;
				goto err_putnode1;
			}
		}

	if (mmi_bootconfig_set && (!of_property_read_string(np, "mmi,bootconfig", &mmi_bootconfig)) ) {
			bl_version = strstr(mmi_bootconfig, "androidboot.bootloader=");
			if (bl_version) {
				end = strpbrk(bl_version, "\n");
				bl_version = strpbrk(bl_version, "=");
			}

			if (bl_version && end > bl_version &&
			    strnstr(bl_version, "factory", end - bl_version)) {
				factory = true;
			}
		}
err_putnode1:
        if (np)
                of_node_put(np);

        return factory;
}

#if IS_ENABLED(CONFIG_MOTO_WLC_ALG_SUPPORT)
static int mmi_is_wireless_online(void);
#endif

static u32 reg2val(enum sc8989x_reg_range id, u8 reg)
{
	const struct reg_range *range = &sc8989x_reg_range_ary[id];
	if (!range)
		return reg;
	return range->table ? range->table[reg] : range->offset + range->step * reg;
}

/*********************I2C API*********************/
static int sc8989x_field_read(struct sc8989x_chip *sc,
	enum sc8989x_fields field_id, int *val)
{
	int ret;

	ret = regmap_field_read(sc->rmap_fields[field_id], val);
	if (ret < 0) {
		dev_err(sc->dev, "sc8989x read field %d fail: %d\n", field_id, ret);
	}

	return ret;
}

static int sc8989x_field_write(struct sc8989x_chip *sc,
	enum sc8989x_fields field_id, int val)
{
	int ret;

	if (field_id == DM_DRIVE || field_id == DP_DRIVE ||
		field_id == CX_DM_DRIVE || field_id == CX_DP_DRIVE) {
		if (sc->otg_enable)
			return -1;
	}

	ret = regmap_field_write(sc->rmap_fields[field_id], val);
	if (ret < 0) {
		dev_err(sc->dev, "sc8989x read field %d fail: %d\n", field_id, ret);
	}

	return ret;
}

/*********************CHIP API*********************/
int Charger_Detect_Init(struct sc8989x_chip *sc)
{
	struct phy *phy;
	int ret;

#if IS_ENABLED(CONFIG_WLC_WO_BOOST)
	if (is_factory_build() && mmi_is_wireless_online())
		return 0;
#endif

	phy = phy_get(sc->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_err(sc->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_BC11_SET);

	dev_err(sc->dev, "%s\n", __func__);
	if (ret)
		dev_err(sc->dev, "failed to set phy ext mode\n");
	phy_put(sc->dev, phy);
	return ret;
}

int Charger_Detect_Release(struct sc8989x_chip *sc)
{
	struct phy *phy;
	int ret;
	phy = phy_get(sc->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_err(sc->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_BC11_CLR);

	dev_err(sc->dev, "%s\n", __func__);
	if (ret)
		dev_err(sc->dev, "failed to set phy ext mode\n");
	phy_put(sc->dev, phy);
	return ret;

}

__maybe_unused static int cx25890hq_write_reg40(struct sc8989x_chip *sc, bool enable)
{
	int ret = 0;
	int try_count = 10;
	int val;

	if (enable) {
		while (try_count--)
		{
			sc8989x_field_write(sc, CX_REG40, 0x00);
			sc8989x_field_write(sc, CX_REG40, 0x50);
			sc8989x_field_write(sc, CX_REG40, 0x57);
			sc8989x_field_write(sc, CX_REG40, 0x44);

			ret = sc8989x_field_read(sc, CX_REG40, &val);
			if (val == 0x03) {
				ret = 1;
				break;
			}
		}
	} else {
		ret = sc8989x_field_write(sc, CX_REG40, 0x00);
	}

	return ret;
}

static int sc8989x_set_key(struct sc8989x_chip *sc)
{
	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A || sc->is_cx25890HQ) {
		return 0;
	}
	regmap_write(sc->regmap, SC8989X_REG7D, SC8989X_KEY1);
	regmap_write(sc->regmap, SC8989X_REG7D, SC8989X_KEY2);
	regmap_write(sc->regmap, SC8989X_REG7D, SC8989X_KEY3);
	return regmap_write(sc->regmap, SC8989X_REG7D, SC8989X_KEY4);
}

static int sc8989x_set_tmr2x(struct sc8989x_chip *sc, bool enable)
{
	int reg_val = enable ? 1 : 0;

	if (sc == NULL) {
		return -EINVAL;
	}

	return sc8989x_field_write(sc, TMR2X_EN, reg_val);
}

static int sc8989x_set_wa(struct sc8989x_chip *sc)
{
	int ret;
	int val;

	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A || sc->is_cx25890HQ) {
		return 0;
	}

	ret = regmap_read(sc->regmap, SC8989X_DPDM3, &val);
	if (ret < 0) {
		sc8989x_set_key(sc);
	}

	regmap_write(sc->regmap, SC8989X_DPDM3, SC8989X_PRIVATE);

	return sc8989x_set_key(sc);
}

__maybe_unused static int sc8989x_set_vbat_lsb(struct sc8989x_chip *sc, bool en)
{
	int ret;
	int val;

	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A || sc->is_cx25890HQ) {
		return 0;
	}

	ret = sc8989x_field_read(sc, VBAT_REG_LSB, &val);
	if (ret < 0) {
		sc8989x_set_key(sc);
	}

	sc8989x_field_write(sc, VBAT_REG_LSB, en);

	return sc8989x_set_key(sc);
}

__maybe_unused static int sc8989x_adc_ibus_en(struct sc8989x_chip *sc, bool en)
{
	int ret;
	int val;

	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A || sc->is_cx25890HQ) {
		return 0;
	}

	ret = regmap_read(sc->regmap, SC8989X_ADC_EN, &val);
	if (ret < 0) {
		sc8989x_set_key(sc);
	}
	ret = regmap_read(sc->regmap, SC8989X_ADC_EN, &val);
	val = en ? val | BIT(2) : val & ~BIT(2);

	regmap_write(sc->regmap, SC8989X_ADC_EN, val);

	return sc8989x_set_key(sc);
}

__maybe_unused static int sc8989x_get_adc_ibus(struct sc8989x_chip *sc, int *val)
{
	int ret;
	int reg = 0;

	ret = sc8989x_field_read(sc, ADC_IBUS, &reg);
	if (ret < 0) {
		sc8989x_set_key(sc);
	}
	ret = sc8989x_field_read(sc, ADC_IBUS, &reg);
	*val = reg2val(SC8989X_IBUS, (u8)reg);

	return 0;
}

__maybe_unused static int __sc8989x_get_adc(struct sc8989x_chip *sc, enum sc8989x_adc_channel chan, int *val)
{
	int reg_val, ret = 0;
	enum sc8989x_fields field_id;
	enum sc8989x_reg_range range_id;
	//stat adc conversion  default: one shot
	sc8989x_field_write(sc, CONV_START, true);
	msleep(20);
	switch (chan) {
	case SC8989X_ADC_VBAT:
		field_id = ADC_VBAT;
		range_id = SC8989X_VBAT;
		break;
	case SC8989X_ADC_VSYS:
		field_id = ADC_VSYS;
		range_id = SC8989X_VSYS;
		break;
	case SC8989X_ADC_VBUS:
		field_id = ADC_VBUS;
		range_id = SC8989X_VBUS;
		break;
	case SC8989X_ADC_ICC:
		field_id = ADC_ICC;
		range_id = SC8989X_VBAT;
		break;
	case SC8989X_ADC_IBUS:
		sc8989x_get_adc_ibus(sc, val);
		dev_info(sc->dev, "get_adc channel: %d val: %d", chan, *val);
		return 0;
	default:
		goto err;
	}

	ret = sc8989x_field_read(sc, field_id, &reg_val);
	if (ret < 0)
		goto err;
	*val = reg2val(range_id, reg_val);
	dev_info(sc->dev, "get_adc channel: %d val: %d", chan, *val);
	return 0;
err:
	dev_err(sc->dev, "get_adc fail channel: %d", chan);
	return -EINVAL;
}

static bool is_apdo_rdy(struct sc8989x_chip *sc) {
	int type = 0;

	if (IS_ERR_OR_NULL(sc)) {
		pr_err("%s: sc is ERR or NULL\n", __func__);
		return false;
	}

	if (IS_ERR_OR_NULL(sc->pd_adapter)) {
		sc->pd_adapter = get_adapter_by_name("pd_adapter");
		if (IS_ERR_OR_NULL(sc->pd_adapter)) {
			pr_err("%s: No pd adapter found\n", __func__);
			return false;
		}
	}

	type = adapter_dev_get_property(sc->pd_adapter, PD_TYPE);
	pr_info("%s pd_type: %d\n", __func__, type);

	if (type == MTK_PD_CONNECT_PE_READY_SNK_APDO/* &&
		sc->psy_usb_type == POWER_SUPPLY_USB_TYPE_DCP &&
		sc->chg_type == POWER_SUPPLY_TYPE_USB_DCP*/)
		return true;
	else
		return false;
}

static irqreturn_t sc8989x_irq_handler(int irq, void *data);
static void sc8989x_hiz_cut_dwork_handler(struct work_struct *work) {
    struct sc8989x_chip *sc = container_of(work,
                                    struct sc8989x_chip,
                                    hiz_cut_dwork.work);
    dev_info(sc->dev, "disable hiz cut end\n");
    sc->hiz_cut_flag = false;
    sc8989x_irq_handler(sc->irq, (void *)sc);
}

static void determine_initial_status(struct sc8989x_chip *sc);
static int sc8989x_set_vindpm_track(struct sc8989x_chip *sc,enum vindpm_track track);
static int sc8989x_set_vindpm(struct sc8989x_chip *sc, int volt_mv);
static int sc8989x_get_iindpm(struct sc8989x_chip *sc, int *curr_ma);
static int sc8989x_set_iindpm(struct sc8989x_chip *sc, int curr_ma);
static int sc8989x_normal_set_hiz(struct sc8989x_chip *sc, bool enable)
{
	int ret = 0;
	int reg_val = enable ? 1 : 0;
	int curr_ma;
	int vbus_good;

	if (sc == NULL) {
		return -EINVAL;
	}
	if (sc->is_upm6920A) {
		if (reg_val) {
			ret = sc8989x_field_read(sc, VBUS_GD, &vbus_good);
			if (!vbus_good) {
				sc->wait_hiz = 1;
				dev_err(sc->dev, " power good not ready,dont hiz\n");
				return ret;
			}
			ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
			dev_err(sc->dev, "sc8989x_normal_set_hiz\n");
		} else {
			sc->wait_hiz = 0;
			if (sc->disablehiz_isset_flg) {
				sc->disablehiz_isset_flg = false;
				if (is_apdo_rdy(sc) == false) {
					ret = sc8989x_get_iindpm(sc, &curr_ma);
					if (curr_ma == 2000) {
						sc8989x_set_iindpm(sc, 500);
						ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
						msleep(5);
						sc8989x_set_iindpm(sc, curr_ma);
					} else {
						ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
					}
				}
			} else {
				ret = sc8989x_get_iindpm(sc, &curr_ma);
				if (curr_ma == 2000) {
					sc8989x_set_iindpm(sc, 500);
					ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
					msleep(5);
					sc8989x_set_iindpm(sc, curr_ma);
				} else {
					ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
				}
			}
		}
	} else if (sc->is_sc89890h){
		int read_reg_val = 0;
		int ret = sc8989x_field_read(sc, EN_HIZ, &read_reg_val);
		if (ret < 0) {
			dev_err(sc->dev, "%s: sc8989x read hiz fail: %d\n", __func__, ret);
			return ret;
		}
		if (read_reg_val == reg_val) {
			dev_info(sc->dev, "hiz already %d\n", reg_val);
			return 0;
		}
		if (reg_val) {
			if (sc->hiz_cut_flag && is_apdo_rdy(sc)) {
			    dev_info(sc->dev, "disable hiz not end, after 300ms retry\n");
			    return -EBUSY;
			}
		} else {
			if (is_apdo_rdy(sc)) {
				schedule_delayed_work(&sc->hiz_cut_dwork,msecs_to_jiffies(DEFAULT_HIZ_CUT_TIME_EXPRIE));
				sc->hiz_cut_flag = true;
			}
			if (sc->disablehiz_isset_flg) {
				sc->disablehiz_isset_flg = false;
				if (is_apdo_rdy(sc) == false)
					ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
				return ret;
			}
		}
		ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
	} else {
		if (reg_val) {
			ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
		} else {
			if (sc->disablehiz_isset_flg) {
				sc->disablehiz_isset_flg = false;
				if (is_apdo_rdy(sc) == false)
					ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
			} else {
				ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
			}
		}
	}
	//ret = sc8989x_field_write(sc, EN_HIZ, reg_val);
	/*if (!reg_val) {
		atomic_set(&sc->vbus_good_flag, 1);
		dev_err(sc->dev, "tcmd cancel  sc8989x_normal_set_hiz");
		msleep(300);
		atomic_set(&sc->vbus_good_flag, 0);
		sc8989x_set_vindpm_track(sc, SC8989X_TRACK_300);
		determine_initial_status(sc);
	}*/
	return ret;
}

__maybe_unused static int sc8989x_set_hiz(struct sc8989x_chip *sc, bool enable)
{
	int ret = 0;
	int uisoc = -1;
	int bat_vol = 3450;
	struct power_supply *bat_psy = NULL;
	union power_supply_propval prop;

	bat_psy = power_supply_get_by_name("battery");
	if (bat_psy == NULL) {
		dev_err(sc->dev, "[%s]psy is not rdy\n", __func__);
		uisoc = -1;
		bat_vol = 4001;
	}

	if (bat_psy) {
		ret = power_supply_get_property(bat_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		if (ret < 0) {
			dev_err(sc->dev, "can't get battery vol!!!");
			goto out;
		}
		bat_vol = prop.intval / 1000;

		ret = power_supply_get_property(bat_psy,
				POWER_SUPPLY_PROP_CAPACITY, &prop);
		if (ret < 0) {
			dev_err(sc->dev, "can't get uisoc!!!");
			goto out;
		}
		uisoc = prop.intval;
	}

	dev_notice(sc->dev, "[Factory Test:%d][%s] uisoc:%d battery_voltage:%d\n",
			is_factory_build(), __func__, uisoc, bat_vol);
	if (is_factory_build() && (uisoc >= 65 || (uisoc == -1 && bat_vol > 4000))) {
		ret = sc8989x_normal_set_hiz(sc, enable);
		goto out;
	} else {
		if (enable) {
			if ((sc->qc_chg_type == USB_TYPE_QC3P_18)
				|| (sc->qc_chg_type == USB_TYPE_QC3P_27)
				|| (sc->qc_chg_type == USB_TYPE_QC3P_45)) {
				ret = sc8989x_set_vindpm(sc, 12000);
				dev_info(sc->dev, "[%s]Cannot set HIZ mode while QC3+ is active (type=%d)\n", __func__,sc->qc_chg_type);
				goto out;
			}
		}
		ret = sc8989x_normal_set_hiz(sc, enable);
	}
out:
	power_supply_put(bat_psy);
	return ret;
}

static int sc8989x_set_iindpm(struct sc8989x_chip *sc, int curr_ma)
{
	int reg_val = val2reg(SC8989X_IINDPM, curr_ma);

	if (sc->is_cx25890HQ) {
		reg_val = val2reg(CX25890HQ_IINDPM, curr_ma);
		if (reg_val >= 32)
			reg_val += 3;
	}

	sc8989x_field_write(sc, EN_ILIM, 0);

	return sc8989x_field_write(sc, IINDPM, reg_val);
}

static int sc8989x_get_iindpm(struct sc8989x_chip *sc, int *curr_ma)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, IINDPM, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read iindpm failed(%d)\n", ret);
		return ret;
	}

	if (sc->is_cx25890HQ) {
		if (reg_val >= 35)
			reg_val -= 3;
		*curr_ma = reg2val(CX25890HQ_IINDPM, reg_val);
	} else {
		*curr_ma = reg2val(SC8989X_IINDPM, reg_val);
	}

	return ret;
}

__maybe_unused static int sc8989x_set_dpdm_hiz(struct sc8989x_chip *sc)
{
	if (sc->is_cx25890HQ) {
		sc8989x_field_write(sc, CX_DP_DRIVE, 0);
		return sc8989x_field_write(sc, CX_DM_DRIVE, 0);
	} else {
		sc8989x_field_write(sc, DP_DRIVE, 0);
		return sc8989x_field_write(sc, DM_DRIVE, 0);
	}
}

__maybe_unused static int sc8989x_set_dpdm_0V(struct sc8989x_chip *sc)
{
	if (sc->is_cx25890HQ) {
		sc8989x_field_write(sc, CX_DP_DRIVE, 1);
		return sc8989x_field_write(sc, CX_DM_DRIVE, 1);
	} else {
		sc8989x_field_write(sc, DP_DRIVE, 1);
		return sc8989x_field_write(sc, DM_DRIVE, 1);
	}
}

static int sc8989x_set_chg_term(struct sc8989x_chip *sc, bool en)
{
	if (sc == NULL) {
		return -EINVAL;
	}

	if (en)
		return sc8989x_field_write(sc, EN_ITERM, 1);
	else
		return sc8989x_field_write(sc, EN_ITERM, 0);
}

__maybe_unused static int sc8989x_reset_wdt(struct sc8989x_chip *sc)
{
	return sc8989x_field_write(sc, WD_RST, 1);
}

static int sc8989x_set_chg_enable(struct sc8989x_chip *sc, bool enable)
{
	int reg_val = enable ? 1 : 0;

	return sc8989x_field_write(sc, CHG_CFG, reg_val);
}

__maybe_unused static int sc8989x_check_chg_enabled(struct sc8989x_chip *sc, bool * enable)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, CHG_CFG, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read charge enable failed(%d)\n", ret);
		return ret;
	}
	*enable = ! !reg_val;

	return ret;
}

__maybe_unused static int sc8989x_set_otg_enable(struct sc8989x_chip *sc, bool enable)
{
	int reg_val = enable ? 1 : 0;

	if (sc->is_cx25890HQ) {
		cx25890hq_write_reg40(sc,true);
		if(enable) {
			sc8989x_field_read(sc, CX_TRILIM, &sc->cx_otg_trilmt_init);
			sc8989x_field_write(sc, CX_TRILIM, 0x00);
		} else {
			sc8989x_field_write(sc, CX_TRILIM, sc->cx_otg_trilmt_init);
		}

		cx25890hq_write_reg40(sc,false);
	}

	return sc8989x_field_write(sc, OTG_CFG, reg_val);
}

__maybe_unused static int sc8989x_set_iboost(struct sc8989x_chip *sc, int curr_ma)
{
	int reg_val = val2reg(SC8989X_IBOOST, curr_ma);

	return sc8989x_field_write(sc, IBOOST_LIM, reg_val);
}

static int sc8989x_set_ichg(struct sc8989x_chip *sc, int curr_ma)
{
	int reg_val;

	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A) {
		reg_val = val2reg(UPM6920A_ICHG, curr_ma);
	} else if (sc->is_cx25890HQ) {
		reg_val = val2reg(CX25890HQ_ICHG, curr_ma);
	} else {
		reg_val = val2reg(SC8989X_ICHG, curr_ma);
	}

	return sc8989x_field_write(sc, ICC, reg_val);
}

static int sc8989x_get_ichg(struct sc8989x_chip *sc, int *curr_ma)
{
	int ret, reg_val;

	if ((sc == NULL) || (curr_ma == NULL)) {
		return -EINVAL;
	}

	ret = sc8989x_field_read(sc, ICC, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read ICC failed(%d)\n", ret);
		return ret;
	}

	if (sc->is_upm6920A) {
		*curr_ma = reg2val(UPM6920A_ICHG, reg_val);
		return ret;
	} else if (sc->is_cx25890HQ) {
		*curr_ma = reg2val(CX25890HQ_ICHG, reg_val);
		return ret;
	}

	*curr_ma = reg2val(SC8989X_ICHG, reg_val);

	return ret;
}

static int sc8989x_set_term_curr(struct sc8989x_chip *sc, int curr_ma)
{
	int reg_val;

	if (sc == NULL) {
		return -EINVAL;
	}

	if (sc->is_upm6920A) {
		reg_val = val2reg(UPM6920A_ITERM, curr_ma);
	} else if (sc->is_cx25890HQ) {
		reg_val = val2reg(CX25890HQ_ITERM, curr_ma);
	} else {
		reg_val = val2reg(SC8989X_ITERM, curr_ma);
	}


	return sc8989x_field_write(sc, ITERM, reg_val);
}

static int sc8989x_get_term_curr(struct sc8989x_chip *sc, int *curr_ma)
{
	int ret, reg_val;

	if ((sc == NULL) || (curr_ma == NULL)) {
		return -EINVAL;
	}

	ret = sc8989x_field_read(sc, ITERM, &reg_val);
	if (ret)
		return ret;

	if (sc->is_upm6920A) {
		*curr_ma = reg2val(UPM6920A_ITERM, reg_val);
	} else if (sc->is_cx25890HQ) {
		*curr_ma = reg2val(CX25890HQ_ITERM, reg_val);
	} else {
		*curr_ma = reg2val(SC8989X_ITERM, reg_val);
	}


	return ret;
}

__maybe_unused static int sc8989x_set_safet_timer(struct sc8989x_chip *sc, bool enable)
{
	int reg_val = enable ? 1 : 0;

	return sc8989x_field_write(sc, EN_TIMER, reg_val);
}

__maybe_unused static int sc8989x_check_safet_timer(struct sc8989x_chip *sc, bool * enabled)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, EN_TIMER, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read ICEN_TIMERC failed(%d)\n", ret);
		return ret;
	}

	*enabled = reg_val ? true : false;

	return ret;
}

static int sc8989x_set_vbat(struct sc8989x_chip *sc, int volt_mv)
{
	int reg_val = val2reg(SC8989X_VBAT_REG, volt_mv);

	return sc8989x_field_write(sc, CV, reg_val);
}

static int sc8989x_get_vbat(struct sc8989x_chip *sc, int *volt_mv)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, CV, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read vbat reg failed(%d)\n", ret);
		return ret;
	}

	*volt_mv = reg2val(SC8989X_VBAT_REG, reg_val);

	return ret;
}

static int sc8989x_set_vindpm(struct sc8989x_chip *sc, int volt_mv)
{
	int reg_val, ret;

	if (sc->ibus_dis) {
		volt_mv = 12000;
	}

	dev_err(sc->dev, "set_vindpm  = %d\n",volt_mv);
	reg_val = val2reg(SC8989X_VINDPM, volt_mv);

	ret = sc8989x_field_write(sc, FORCE_VINDPM, 1);

	ret = sc8989x_field_write(sc, VINDPM, reg_val);

	return ret;
}

static int sc8989x_get_vindpm(struct sc8989x_chip *sc, int *volt_mv)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, VINDPM, &reg_val);
	if (ret)
		return ret;

	*volt_mv = reg2val(SC8989X_VINDPM, reg_val);

	return ret;
}

static int sc8989x_force_dpdm(struct sc8989x_chip *sc)
{
	int val;
#if 0
	int ret;

	dev_err(sc->dev, "sc8989x_force_dpdm\n");

	ret = regmap_read(sc->regmap, 0x02, &val);
	if (ret < 0) {
		return ret;
	}
	val |= 0x02;
	ret = regmap_write(sc->regmap, 0x02, val);
	sc->power_good = 0;

	return ret;
#else
	struct i2c_client *i2c = to_i2c_client(sc->dev);

	val = i2c_smbus_read_byte_data(i2c, 0x2);
	dev_err(sc->dev, "sc8989x_force_dpdm val=0x%x\n", val);
	val |= 0x02;
	i2c_smbus_write_byte_data(i2c, 0x2, val);
	sc->power_good = 0;

	return 0;
#endif
}

static int sc8989x_get_charge_stat(struct sc8989x_chip *sc)
{
	int ret;
	int chg_stat, vbus_stat;

	ret = sc8989x_field_read(sc, CHG_STAT, &chg_stat);
	if (ret)
		return ret;

	ret = sc8989x_field_read(sc, VBUS_STAT, &vbus_stat);
	if (ret)
		return ret;

	if (vbus_stat == VBUS_STAT_OTG) {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		switch (chg_stat) {
		case CHG_STAT_NOT_CHARGE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case CHG_STAT_PRE_CHARGE:
		case CHG_STAT_FAST_CHARGE:
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHG_STAT_CHARGE_DONE:
			return POWER_SUPPLY_STATUS_FULL;
		}
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

__maybe_unused static int sc8989x_check_charge_done(struct sc8989x_chip *sc, bool * chg_done)
{
	int ret, reg_val;

	ret = sc8989x_field_read(sc, CHG_STAT, &reg_val);
	if (ret) {
		dev_err(sc->dev, "read charge stat failed(%d)\n", ret);
		return ret;
	}

	*chg_done = (reg_val == CHG_STAT_CHARGE_DONE) ? true : false;
	return ret;
}

static bool sc8989x_detect_device(struct sc8989x_chip *sc)
{
	int ret;
	int val;
	int dev_version;
	int cfginit2_value = 0;

	if (sc == NULL) {
		return false;
	}

	ret = sc8989x_field_read(sc, PN, &val);
	if (ret < 0 || !(val == SC89890H_PN_NUM ||
		val == SC8950_PN_NUM || val == SC89890W_PN_NUM ||
		val == UPM6920A_PN_NUM)) {
		dev_err(sc->dev, "not find sc8989x, part_no = %d\n", val);
		return false;
	}

	ret = sc8989x_field_read(sc, DEV_VERSION, &dev_version);
	dev_err(sc->dev, "dev_version = %d\n", dev_version);

	if (val == UPM6920A_PN_NUM && dev_version == 0) {
		sc->is_cx25890HQ = 1;
	} else if (val == UPM6920A_PN_NUM) {
		sc->is_upm6920A = 1;
		regmap_write(sc->regmap, UPM6920A_REG_CFG_MODE, UPM6920A_CFG_MODE_ENABLE);
		sc8989x_field_write(sc, CFGINIT_BIT,0);
		sc8989x_field_write(sc, CFGINIT2_BIT, 0x05);
		sc8989x_field_read(sc, CFGINIT2_BIT, &cfginit2_value);
		regmap_write(sc->regmap, UPM6920A_REG_CFG_MODE, UPM6920A_CFG_MODE_DISABLE);
	} else if (val == SC89890H_PN_NUM) {
		sc->is_sc89890h = 1;
	}

	sc->dev_id = val;
	dev_info(sc->dev, "%s: part_no = %d, cfginit2_value=0x%x", __func__, sc->dev_id, cfginit2_value);

	return true;
}

static int sc8989x_dump_register(struct sc8989x_chip *sc)
{
	int ret = 0;
	int i;
	int val;
	char reg_buff[TOTAL_DUMP_LEN] = {0};
	char temp_buff[SINGLE_DUMP_LEN] = {0};

	if (dump_reg_enable) {
		for (i = 0; i <= MAX_REG_NUM; i++) {
			ret = regmap_read(sc->regmap, i, &val);
			if (ret < 0) {
				dev_err(sc->dev, "%s, failed to read reg\n", __func__);
				return ret;
			}
			snprintf(temp_buff, SINGLE_DUMP_LEN, "reg[0x%02x] = 0x%02x, ", i, val);
			strcat(reg_buff, temp_buff);
		}
		dev_info(sc->dev, "%s: %s", __func__, reg_buff);
	} else {
		dev_err(sc->dev, "%s, dump register has been disabled\n", __func__);
	}

	return ret;
}


static void sc8989x_qc_software_reset(struct sc8989x_chip *sc)
{
	if (!sc) {
		return;
	}
	sc->pulse_cnt = 0;
	sc->qc_chg_type = USB_TYPE_UNKNOWN;
	sc->qc_is_detect = false;
	sc->mmi_qc3p_rerun_done = false;
}

/********************MTK OPS***********************/
static int sc8989x_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);
	sc8989x_field_write(sc, CONV_START, true);
	/* Enable charging */
	ret = sc8989x_set_chg_enable(sc, true);
	if (ret) {
		dev_err(sc->dev, "Failed to enable charging:%d\n", ret);
	}

	return ret;
}

static int sc8989x_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);

	if(sc->qc_dev){
		adapter_dev_reset_chg_type(sc->qc_dev);
		sc->pulse_cnt = 0;
		sc->qc_chg_type = 0;
		sc->qc_is_detect = false;
	}

	sc8989x_field_write(sc, CONV_START, false);
	ret = sc8989x_set_chg_enable(sc, false);
	if (ret) {
		dev_err(sc->dev, "Failed to disable charging:%d\n", ret);
	}
	sc8989x_qc_software_reset(sc);

	return ret;
}

static int sc8989x_set_eoc_curr(struct charger_device *chg_dev, u32 uA)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int reg_val = val2reg(SC8989X_ITERM, uA / 1000);
	dev_info(sc->dev, "%s: iterm curr = %duA\n", __func__, uA);
	return sc8989x_field_write(sc, ITERM, reg_val);
}

static int sc8989x_enable(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	ret = sc8989x_set_chg_enable(sc, en);

	dev_info(sc->dev, "%s charger %s\n", en ? "enable" : "disable",
		!ret ? "successfully" : "failed");

	return ret;
}

static int sc8989x_is_enabled(struct charger_device *chg_dev, bool * enabled)
{
	int ret;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	ret = sc8989x_check_chg_enabled(sc, enabled);
	dev_info(sc->dev, "charger is %s\n",
		*enabled ? "charging" : "not charging");

	return ret;
}

static int sc8989x_get_charging_current(struct charger_device *chg_dev, u32 * curr)
{
	int ret = 0;
	int curr_ma;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	ret = sc8989x_get_ichg(sc, &curr_ma);
	if (!ret) {
		*curr = curr_ma * 1000;
	}

	return ret;
}

static int sc8989x_set_charging_current(struct charger_device *chg_dev, u32 curr)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s: charge curr = %duA\n", __func__, curr);

	return sc8989x_set_ichg(sc, curr / 1000);
}

static int sc8989x_get_input_current(struct charger_device *chg_dev, u32 * curr)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int curr_ma;
	int ret;

	dev_info(sc->dev, "%s\n", __func__);

	ret = sc8989x_get_iindpm(sc, &curr_ma);
	if (!ret) {
		*curr = curr_ma * 1000;
	}

	return ret;
}

static int sc8989x_set_input_current(struct charger_device *chg_dev, u32 curr)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s: iindpm curr = %duA\n", __func__, curr);

	return sc8989x_set_iindpm(sc, curr / 1000);
}

static int sc8989x_get_constant_voltage(struct charger_device *chg_dev, u32 * volt)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int volt_mv;
	int ret;

	dev_info(sc->dev, "%s\n", __func__);

	ret = sc8989x_get_vbat(sc, &volt_mv);
	if (!ret) {
		*volt = volt_mv * 1000;
	}

	return ret;
}

static int sc8989x_get_mivr_voltage(struct charger_device *chg_dev, u32 * volt)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int volt_mv;
	int ret;

	ret = sc8989x_get_vindpm(sc, &volt_mv);
	if (!ret) {
		*volt = volt_mv * 1000;
	}

	return ret;
}

static int sc8989x_set_constant_voltage(struct charger_device *chg_dev, u32 volt)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s: charge volt = %duV\n", __func__, volt);

	return sc8989x_set_vbat(sc, volt / 1000);
}

static int sc8989x_set_vindpm_track(struct sc8989x_chip *sc,enum vindpm_track track)
{
	int ret;
	int val;

	ret = sc8989x_field_read(sc, F_VINDPM_TRACK, &val);
	if (ret < 0) {
		sc8989x_set_key(sc);
	}
	dev_info(sc->dev, "%s\n", __func__);
	sc8989x_field_write(sc, F_VINDPM_TRACK, track);

	return sc8989x_set_key(sc);
}

static int sc8989x_kick_wdt(struct charger_device *chg_dev)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);
	return sc8989x_reset_wdt(sc);
}

static int sc8989x_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s: vindpm volt = %d\n", __func__, volt);

	return sc8989x_set_vindpm(sc, volt / 1000);
}

static int sc8989x_is_charging_done(struct charger_device *chg_dev, bool * done)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int ret;

	ret = sc8989x_check_charge_done(sc, done);

	dev_info(sc->dev, "%s: charge %s done\n", __func__, *done ? "is" : "not");
	return ret;
}

static int sc8989x_get_mivr_state(struct charger_device *chgdev, bool *active)
{
	int ret;
	u32 val;
	struct sc8989x_chip *sc = dev_get_drvdata(&chgdev->dev);

	*active = false;
	ret = sc8989x_field_read(sc, VINDPM_STAT, &val);
	if (ret < 0)
		return ret;
	*active = val;
	dev_info(sc->dev, "%s: charge %s is in vindpm\n", __func__, *active ? "is" : "not");
	return 0;
}

static int sc8989x_get_min_ichg(struct charger_device *chg_dev, u32 * curr)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	*curr = 60 * 1000;
	dev_err(sc->dev, "%s\n", __func__);
	return 0;
}

static int sc8989x_dump_registers(struct charger_device *chg_dev)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);

	return sc8989x_dump_register(sc);
}

static int sc8989x_send_ta_current_pattern(struct charger_device *chg_dev, bool is_increase)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int ret;
	int i;

	dev_info(sc->dev, "%s: %s\n", __func__, is_increase ?
		"pumpx up" : "pumpx dn");
	//pumpx start
	ret = sc8989x_set_iindpm(sc, 100);
	if (ret)
		return ret;
	msleep(10);

	for (i = 0; i < 5; i++) {
		if (i < (is_increase ? 2 : 3)) {
			sc8989x_set_iindpm(sc, 800);
			is_increase ? msleep(100) : msleep(300);
		} else {
			sc8989x_set_iindpm(sc, 800);
			is_increase ? msleep(300) : msleep(100);
		}
		sc8989x_set_iindpm(sc, 100);
		msleep(100);
	}

	//pumpx stop
	sc8989x_set_iindpm(sc, 800);
	msleep(500);
	//pumpx wdt, max 240ms
	sc8989x_set_iindpm(sc, 100);
	msleep(100);

	return sc8989x_set_iindpm(sc, 1500);
}

static int sc8989x_send_ta20_current_pattern(struct charger_device *chg_dev, u32 uV)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	u8 val = 0;
	int i;

	if (uV < 5500000) {
		uV = 5500000;
	} else if (uV > 15000000) {
		uV = 15000000;
	}

	val = (uV - 5500000) / 500000;

	dev_info(sc->dev, "%s ta20 vol=%duV, val=%d\n", __func__, uV, val);

	sc8989x_set_iindpm(sc, 100);
	msleep(150);

	for (i = 4; i >= 0; i--) {
		sc8989x_set_iindpm(sc, 800);
		(val & (1 << i)) ? msleep(100) : msleep(50);
		sc8989x_set_iindpm(sc, 100);
		(val & (1 << i)) ? msleep(50) : msleep(100);
	}

	sc8989x_set_iindpm(sc, 800);
	msleep(150);
	sc8989x_set_iindpm(sc, 100);
	msleep(240);

	return sc8989x_set_iindpm(sc, 800);
}

static int sc8989x_set_ta20_reset(struct charger_device *chg_dev)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);
	int curr;
	int ret;

	ret = sc8989x_get_iindpm(sc, &curr);

	ret = sc8989x_set_iindpm(sc, 100);
	msleep(300);
	ret = sc8989x_set_iindpm(sc, curr);//TODO
	return ret;
}

static int sc8989x_get_adc(struct charger_device *chg_dev,
		enum adc_channel chan, int *min, int *max)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);						
	enum sc8989x_adc_channel sc_chan;
	int val;

	switch (chan) {
	case ADC_CHANNEL_VBAT:
		sc_chan = SC8989X_ADC_VBAT;
		break;
	case ADC_CHANNEL_VSYS:
		sc_chan = SC8989X_ADC_VSYS;
		break;
	case ADC_CHANNEL_VBUS:
		sc_chan = SC8989X_ADC_VBUS;
		break;
	case ADC_CHANNEL_IBUS:
		sc_chan = SC8989X_ADC_IBUS;
		break;
	default:
		return -95; 
	}

	if (sc->is_upm6920A) {
		if (chan == ADC_CHANNEL_VBUS) {
			sc8989x_get_vbus (sc->chg_dev, min);
			dev_info(sc->dev, "%s sc8989x_get_vbus chan=%d, %d\n", __func__, chan, *min);
			*max = *min;
			return 0;
		} else {
			sc8989x_field_read(sc, EN_HIZ, &val);
			if (val) {
				dev_info(sc->dev, "%s hiz dont adc,chan=%d\n", __func__, chan);
				*min = 0;
				*max = *min;
				return 0;
			}
		}
	}

	__sc8989x_get_adc(sc, sc_chan, min);
	*min = *min * 1000;
	*max = *min;

	return 0;
}

static int sc8989x_set_otg(struct charger_device *chg_dev, bool enable)
{
	int ret;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	/* UPM6920A: Reset DP/DM lines before changing OTG state.
	 * This sequence of driving to 0V then Hi-Z is required to ensure the lines are fully released,
	 * preventing issues with subsequent charger detection.
	 */
	sc->otg_enable = enable;
	if (enable && (sc->is_upm6920A || sc->is_sc89890h)
		 && sc->qc_is_start_detect)  {
		sc8989x_set_dpdm_0V(sc);
		sc8989x_set_dpdm_hiz(sc);
		dev_info(sc->dev, "QC detection was active, resetting DP/DM to Hi-Z before enabling OTG\n");
	}

	if (sc->otg_enable)
		sc8989x_set_hiz(sc, !enable);
	ret = sc8989x_set_otg_enable(sc, enable);
	ret |= sc8989x_set_chg_enable(sc, !enable);
	dev_info(sc->dev, "%s OTG %s\n", enable ? "enable" : "disable",
		!ret ? "successfully" : "failed");

	return ret;
}

static int sc8989x_set_safety_timer(struct charger_device *chg_dev, bool enable)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s  %s\n", __func__, enable ? "enable" : "disable");

	return sc8989x_set_safet_timer(sc, enable);
}

static int sc8989x_is_safety_timer_enabled(struct charger_device *chg_dev, bool * enabled)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	return sc8989x_check_safet_timer(sc, enabled);
}

static int sc8989x_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s otg curr = %d\n", __func__, curr);
	return sc8989x_set_iboost(sc, curr / 1000);
}

static int sc8989x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);

#ifdef CONFIG_MTK_CHARGER_V4P19
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
#else
	switch (event) {
	case EVENT_FULL:
		sc->mmi_charging_full = true;
		break;
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		sc->mmi_charging_full = false;
		break;
	default:
		break;
	}
#endif /*CONFIG_MTK_CHARGER_V4P19*/
	power_supply_changed(sc->psy);

	return 0;
}

static int sc8989x_enable_hz(struct charger_device *chg_dev, bool enable)
{
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s %s\n", __func__, enable ? "enable" : "disable");

	return sc8989x_set_hiz(sc, enable);
}

__maybe_unused static int sc8989x_enable_powerpath(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s, en= %d\n", __func__, en);

	/* Enable charging */
	ret = sc8989x_set_chg_enable(sc, en);
	if (ret) {
		dev_err(sc->dev, "Failed to enable charging:%d\n", ret);
	}
	return ret;
}

__maybe_unused static int sc8989x_is_powerpath_enabled(struct charger_device *chg_dev, bool * en)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	dev_info(sc->dev, "%s\n", __func__);

	/* Enable charging */
	ret = sc8989x_check_chg_enabled(sc, en);
	if (ret) {
		dev_err(sc->dev, "Failed to check charger enable:%d\n", ret);
	}
	return ret;
}

#if IS_ENABLED(CONFIG_OEM_TURBO_CHARGER)
static int sc8989x_enable_dpdm_hiz(struct charger_device *chg_dev)
{
	int ret;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	ret = sc8989x_set_dpdm_hiz(sc);
	if (ret < 0)
		dev_err(sc->dev, "%s set dpdm hiz failed ret(%d)\n", __func__, ret);

	return ret;
}
#endif

static int sc8989x_enable_terminate(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	ret = sc8989x_set_chg_term(sc, en);
	if (ret < 0)
			dev_err(sc->dev, "%s failed ret(%d)\n", __func__, ret);

	return ret;
}

static int sc8989x_get_vbus(struct charger_device *chgdev, u32 *vbus)
{
        int ret, value;
        struct sc8989x_chip *sc = dev_get_drvdata(&chgdev->dev);

        ret = iio_read_channel_processed(sc->vbus, &value);
        if (ret < 0) {
                dev_err(sc->dev, "get vbus voltage failed");
                return -EINVAL;
        }
        *vbus = value + R_VBUS_CHARGER_1 * value / R_VBUS_CHARGER_2;
        *vbus = *vbus * 1000;
        dev_info(sc->dev, "vbus voltage: %d", *vbus);
        return ret;
}

static int sc8989x_get_chip_id(struct charger_device *chg_dev, int *id)
{
	int ret = 0;
	struct sc8989x_chip *sc = dev_get_drvdata(&chg_dev->dev);

	*id = SC8989X_CHIP_ID;
	dev_info(sc->dev, "%s id = %d \n", __func__, *id);
	return ret;
}

static int sc8989x_enable_qc20_hvdcp_9v(struct sc8989x_chip *sc)
{
	int ret;
	int dp_val, dm_val;

	if (!sc) {
		return -EINVAL;
	}

	/*dp and dm connected,dp 0.6V dm 0V*/
	dp_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 0.6V
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 0.6V
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
	    return ret;

	dm_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 0.6V
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 0.6V
	}

	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	msleep(QC3P_MSLEEP_1500DELAY);

	dm_val = 1;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 0V
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 0V
	}

	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;
	msleep(QC3P_MSLEEP_500DELAY);

	/* dp 3.3v and dm 0.6v out 9V */
	dp_val = 6;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 3.3v
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 3.3v
	}

	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	dm_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 0.6v
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 0.6v
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	return ret;
}

static int sc8989x_adjust_qc20_hvdcp_5v(struct sc8989x_chip *sc)
{
	int ret;
	int dp_val, dm_val;

	if (!sc) {
		return -EINVAL;
	}

	/* dp 0.6v and dm 0v out 5V */
	dp_val = 2;
	if (sc->is_cx25890HQ){
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 0.6V
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 0.6V
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
	    return ret;

	dm_val = 1;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 0V
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 0V
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	return ret;
}

// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sc8989x_qc30_step_up_vbus(struct sc8989x_chip *sc)
{
	int ret;
	int dp_val;

	if (!sc) {
		return -EINVAL;
	}

	/*  dm 3.3v to dm 0.6v  step up 200mV when IC is QC3.0 mode*/
	dp_val = 6;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 3.3V
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 3.3V
	}
	if (ret)
	    return ret;

	udelay(QC3P_UDELAY_2500DELAY);
	dp_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 0.6V
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 0.6V
	}
	if (ret)
	    return ret;

	udelay(QC3P_UDELAY_2500DELAY);
	return ret;
}
// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sc8989x_qc30_step_down_vbus(struct sc8989x_chip *sc)
{
	int ret;
	int dm_val;

	if (!sc) {
		return -EINVAL;
	}

	/* dp 0.6v and dm 0.6v step down 200mV when IC is QC3.0 mode*/
	dm_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 0.6v
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 0.6v
	}
	if (ret)
		return ret;


	udelay(QC3P_UDELAY_2500DELAY);
	dm_val = 6;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 3.3v
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 3.3v
	}
	if (ret)
		return ret;
	udelay(QC3P_UDELAY_2500DELAY);

	return ret;
}

static int sc8989x_detected_qc30_hvdcp(struct sc8989x_chip *sc, int *charger_type)
{
	int ret = 0;
	int dp_val, dm_val;
	int i=0, vbus_voltage;

	if (!sc) {
		return -EINVAL;
	}

	/* dp 0.6v and dm 3.3v entry QC3.0 mode */
	dp_val = 2;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DP_DRIVE, dp_val); //dp 0.6V
	} else {
		ret = sc8989x_field_write(sc, DP_DRIVE, dp_val); //dp 0.6V
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
	    return ret;

	dm_val = 6;
	if (sc->is_cx25890HQ) {
		ret = sc8989x_field_write(sc, CX_DM_DRIVE, dm_val); //dm 3.3v
	} else {
		ret = sc8989x_field_write(sc, DM_DRIVE, dm_val); //dm 3.3v
	}
	dev_dbg(sc->dev, "%s: %d  ret=%d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	msleep(QC3P_MSLEEP_100DELAY);

	for (i = 0; i < 16; i++) {
		ret = sc8989x_qc30_step_up_vbus(sc);
		if (ret)
			dev_err(sc->dev, "%s qc30 step up vbus error\n", __func__);
	}

	msleep(QC3P_MSLEEP_100DELAY);

	sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	dev_info(sc->dev, "%s vbus voltage now = %d in detected qc30\n", __func__,vbus_voltage);

	if (vbus_voltage > MMI_HVDCP3_VOLTAGE_STANDARD) {
		if (charger_type == NULL)
			return -EINVAL;
		*charger_type = USB_TYPE_QC30;
		dev_info(sc->dev, "%s QC3.0 charger detected\n", __func__);

		for (i = 0; i < 16; i++) {
			ret = sc8989x_qc30_step_down_vbus(sc);
			if (ret)
				dev_err(sc->dev, "%s qc30 step down vbus error\n", __func__);
		}
	}


	sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	dev_info(sc->dev, "%s vbus voltage now = %d after detected qc30\n", __func__,vbus_voltage);
	return ret;
}

static int sc8989x_dp_dm(struct charger_device *chg_dev, int val)
{
	struct sc8989x_chip *sc;
	int ret = 0;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}
	dev_info(sc->dev, "%s: val=%d\n", __func__, val);

	switch (val) {
	case DP_DM_FORCE_QC2_5V:
		ret = sc8989x_adjust_qc20_hvdcp_5v(sc);
		break;
	case DP_DM_FORCE_QC3_5V:
	case DP_DM_FORCE_QC3P_5V:
		//ret = wt6670f_force_qc3_5V();
		break;
	case DP_DM_DP_PULSE:
		ret = sc8989x_qc30_step_up_vbus(sc);
		if (ret)
			dev_err(sc->dev, "qc protocol ic set vbus up failed\n");
		else
			sc->pulse_cnt++;
		break;
	case DP_DM_DM_PULSE:
		ret = sc8989x_qc30_step_down_vbus(sc);
		if (ret)
			dev_err(sc->dev, "qc protocol ic set vbus down failed\n");
		else if (sc->pulse_cnt > 0)
			sc->pulse_cnt--;
		break;
	default:
		break;
	}

	return ret;
}

static int mmi_qc_is_detect(struct charger_device *chg_dev, bool *val)
{
	struct sc8989x_chip *sc;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	if (val == NULL) {
		return -EINVAL;
	}
	*val = sc->qc_is_detect;

	return 0;
}

static int mmi_get_protocol(struct charger_device *chg_dev, int *val)
{
	struct sc8989x_chip *sc;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	if (val == NULL) {
		return -EINVAL;
	}
	*val = sc->qc_chg_type;

	return 0;
}

static int mmi_set_dp_dm(struct charger_device *chg_dev, int val)
{
	int ret = -EINVAL;
	struct sc8989x_chip *sc;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	mutex_lock(&sc->dpdm_lock);

	if (sc->qc_is_detect == true) {
		mutex_unlock(&sc->dpdm_lock);
		return ret;
	}
	switch (val) {
	case DP_DM_DP_PULSE:
		if (sc->qc_dev) {
			ret = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DP_PULSE);
			if (ret < 0)
				dev_err(sc->dev, "qc protocol ic set vbus up failed\n");
			else
				sc->pulse_cnt++;
		} else if  (sc->mmi_hvdcp_support) {
			ret = sc8989x_qc30_step_up_vbus(sc);
			if (ret)
				dev_err(sc->dev, "qc protocol ic set vbus up failed\n");
			else
				sc->pulse_cnt++;
			dev_info(sc->dev, "%s:%d: ret:%d,%d, \n", __func__, __LINE__, ret, sc->pulse_cnt);
		}
		break;
	case DP_DM_DM_PULSE:
		if (sc->qc_dev) {
			ret = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DM_PULSE);
			if (ret < 0)
				dev_err(sc->dev, "qc protocol ic set vbus down failed\n");
			else if (sc->pulse_cnt > 0)
				sc->pulse_cnt--;
		} else if (sc->mmi_hvdcp_support) {
			ret = sc8989x_qc30_step_down_vbus(sc);
			if (ret)
				dev_err(sc->dev, "qc protocol ic set vbus down failed\n");
			else if (sc->pulse_cnt > 0)
				sc->pulse_cnt--;
			dev_info(sc->dev, "%s:%d: ret:%d,%d, \n", __func__, __LINE__, ret, sc->pulse_cnt);
		}
		break;
	default:
		break;
	}

	mutex_unlock(&sc->dpdm_lock);
	return ret;
}

static int mmi_get_dp_dm(struct charger_device *chg_dev, int *val)
{
	int ret = 0;
	struct sc8989x_chip *sc;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	if (val == NULL) {
		return -EINVAL;
	}
	*val = sc->pulse_cnt;

	return ret;
}

#define HVDCP_POWER_MIN			15000
#define HVDCP_VOLTAGE_BASIC		5000000
#define HVDCP_VOLTAGE_NOM		(HVDCP_VOLTAGE_BASIC - 200000)
#define HVDCP_VOLTAGE_MAX		(HVDCP_VOLTAGE_BASIC + 200000)
#define HVDCP_VOLTAGE_MIN		4000000
#define HVDCP_PULSE_COUNT_MAX 	((HVDCP_VOLTAGE_BASIC - 5000000) / 200000 + 2)
int mmi_config_qc_charger(struct charger_device *chg_dev)
{
	int rc = 0;
	int vbus_uv;
	struct sc8989x_chip *sc;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	if (!sc->qc_dev && !sc->mmi_hvdcp_support) {
		dev_warn(sc->dev, "qc protocol don't ready, exit\n");
		return -EINVAL;
	}

	if (!sc->chg_dev) {
		dev_warn(sc->dev, "charger dev don't ready, exit\n");
		return -EINVAL;
	}

	rc = sc8989x_get_vbus(sc->chg_dev, &vbus_uv);
	if (rc < 0) {
		dev_warn(sc->dev, "%s get vbus failed\n",__func__);
		return -EINVAL;
	}

	if(vbus_uv < 4000000 || vbus_uv > 6000000) {
		dev_warn(sc->dev, "vbus is not for qc3.0\n");
		return -EINVAL;
	}

	pr_info("pulse_cnt=%d, vbus_uv=%d\n", sc->pulse_cnt, vbus_uv);
	if (vbus_uv < HVDCP_VOLTAGE_NOM && sc->pulse_cnt < HVDCP_PULSE_COUNT_MAX) {
		if (sc->qc_dev)
			rc = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DP_PULSE);
		else if (sc->mmi_hvdcp_support)
			rc = sc8989x_dp_dm(sc->chg_dev, DP_DM_DP_PULSE);
		if (rc)
			dev_err(sc->dev, "qc protocol ic set vbus up failed\n");
		else
			sc->pulse_cnt++;
	} else if (vbus_uv > HVDCP_VOLTAGE_MAX && sc->pulse_cnt > 0 ) {
		if (sc->qc_dev)
			rc = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DM_PULSE);
		else if (sc->mmi_hvdcp_support)
			rc = sc8989x_dp_dm(sc->chg_dev, DP_DM_DM_PULSE);
		if (rc)
			dev_err(sc->dev, "qc protocol ic set vbus down failed\n");
		else {
			sc->pulse_cnt--;
		}
	} else {
		dev_info(sc->dev, "QC3.0 output configure completed\n");
		rc = 0;
		return rc;
	}
	msleep(QC3P_MSLEEP_100DELAY);
	return rc;
}

#define QC30_STEP 200000
int mmi_config_qc30_charger_voltage(struct charger_device *chg_dev)
{
	int rc = 0;
	int vbus_uv;
	struct sc8989x_chip *sc;
	int target_uV = 6600000;//6.6V
	int i;
	int val;
	int step;
	int qc_step;
	int ret = 0;

	if (!chg_dev) {
		return -EINVAL;
	}
	sc = dev_get_drvdata(&chg_dev->dev);
	if (!sc) {
		return -EINVAL;
	}

	if (!sc->only_buck_qc_support) {
		dev_info(sc->dev, "unsupport only buck qc\n");
		return ret;
	}

	if (!sc->qc_dev && !sc->mmi_hvdcp_support) {
		dev_warn(sc->dev, "qc protocol don't ready, exit\n");
		return -EINVAL;
	}

	if (!sc->chg_dev) {
		dev_warn(sc->dev, "charger dev don't ready, exit\n");
		return -EINVAL;
	}

	pr_info("%s qc_chg_type=%d", __func__, sc->qc_chg_type );
	if (sc->qc_chg_type == USB_TYPE_QC30) {
		qc_step = QC30_STEP;
	} else {
		return ret;
	}

	mdelay(200);
	rc = sc8989x_get_vbus(sc->chg_dev, &vbus_uv);
	if (rc < 0) {
		dev_warn(sc->dev, "%s get vbus failed\n",__func__);
		return -EINVAL;
	}

	if(target_uV < vbus_uv) {
		step = (vbus_uv - target_uV) / qc_step;
		val = DP_DM_DM_PULSE;
	} else {
		step = (target_uV - vbus_uv) / qc_step;
		val = DP_DM_DP_PULSE;
	}

	pr_info("step=%d, vbus_uv=%d\n", step, vbus_uv);
	for (i = 0; i < step; i++) {
		if (val == DP_DM_DM_PULSE) {
			if (sc->qc_dev) {
				ret = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DM_PULSE);
				if (ret < 0)
					dev_err(sc->dev, "qc protocol ic set vbus down failed\n");
			} else {
				ret = sc8989x_qc30_step_down_vbus(sc);
				if (ret)
					pr_err("HVDCP: %s qc3p step down vbus error\n", __func__);
			}
		} else {
			if (sc->qc_dev) {
				ret = adapter_dev_dp_dm(sc->qc_dev, DP_DM_DP_PULSE);
				if (ret < 0)
					dev_err(sc->dev, "qc protocol ic set vbus up failed\n");
			} else {
				ret = sc8989x_qc30_step_up_vbus(sc);
				if (ret)
					pr_err("HVDCP: %s qc3p step up vbus error\n", __func__);
			}
		}
		mdelay(10);
	}

	rc = sc8989x_get_vbus(sc->chg_dev, &vbus_uv);
	if (rc < 0) {
		dev_warn(sc->dev, "%s get vbus failed\n",__func__);
		return -EINVAL;
	}

	/*
		if (vbus > 8500000)
			ret = sc8989x_set_vindpm(sc, 7800);
	*/
	pr_info("step=%d, target_vbus=%d\n", step, vbus_uv);
	return rc;
}

void get_qc_charger_type_func_work(struct work_struct *work)
{
	struct delayed_work *detect_qc_dwork = NULL;
	struct sc8989x_chip *sc;
	bool early_notified = false;
	bool need_retry = false;
	bool m_chg_ready = false;
	int early_chg_type = 0;
	int count = 0;
	int ret;
	union power_supply_propval val;

	detect_qc_dwork = container_of(work, struct delayed_work, work);
	if(detect_qc_dwork == NULL) {
		pr_info("Can't get charge_monitor_work\n");
		return ;
	}
	sc = container_of(detect_qc_dwork, struct sc8989x_chip, detect_qc_dwork);
	if(sc == NULL) {
		pr_err("Can't get sc8989x_chip \n");
		return ;
	}

	if (!sc->qc_dev) {
		pr_err("qc protocol ic dev is not ready, exit \n");
		return;
	}

	adapter_dev_reset_chg_type(sc->qc_dev);
	pr_info("start qc detected \n");
	sc->qc_is_detect = true;

	if (sc->icharge_default_value > 0) {
		sc8989x_set_charging_current(sc->chg_dev,sc->icharge_default_value);
	} else {
		sc8989x_set_charging_current(sc->chg_dev,1000000);
	}

	do{
		m_chg_ready = false;
		sc->qc_chg_type = 0;
		early_notified = false;
		need_retry = false;
		early_chg_type = 0;
		adapter_dev_start_detection(sc->qc_dev);
		while((!m_chg_ready)&&(count<100)) {
			ret = power_supply_get_property(sc->psy, POWER_SUPPLY_PROP_ONLINE, &val);
			if (val.intval <= 0) {
			      pr_info("[%s] ONLINE: %d, skip detecting0,ret=%d\n",__func__, val.intval, ret);
			      break;
			}
			msleep(30);
			count++;
			adapter_dev_is_charger_ready(sc->qc_dev, &m_chg_ready);

			if(!early_notified){
			      adapter_dev_get_protocol(sc->qc_dev, &early_chg_type);
			}

			if(early_chg_type == USB_TYPE_QC3P_18 || early_chg_type == USB_TYPE_QC3P_27){
				pr_info("[%s] qc early type is QC3+: %d, skip detecting\n",__func__, early_chg_type);
				break;
			}
				pr_info("qc waiting early type: 0x%x, detect ready: 0x%x, count: %d\n", early_chg_type, m_chg_ready, count);
		}

		adapter_dev_get_protocol(sc->qc_dev, &sc->qc_chg_type);

		if(sc->qc_chg_type == USB_TYPE_OCP && !need_retry){
			need_retry = true;
		} else {
			need_retry = false;
		}
		ret = power_supply_get_property(sc->psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (val.intval) {
			power_supply_changed(sc->psy);
		}
		else {
			pr_info("[%s] ONLINE: %d, skip detecting1\n",__func__, val.intval);
			break;
		}
		pr_info("[%s] qc charge type is  0x%x\n",__func__, sc->qc_chg_type);
	}while(need_retry);

	sc->qc_is_detect = false;

	if(sc->qc_chg_type == USB_TYPE_QC20){
		adapter_dev_dp_dm(sc->qc_dev, DP_DM_FORCE_QC2_5V);
		pr_info("Force set qc2 5V");
		msleep(100);
	}else if(sc->qc_chg_type == USB_TYPE_QC30){
		adapter_dev_dp_dm(sc->qc_dev, DP_DM_FORCE_QC3_5V);
		msleep(100);
		sc->pulse_cnt = 0;
		pr_info("Force set qc3 5V");
	}

	mmi_config_qc30_charger_voltage(sc->chg_dev);

	if (sc->qc_chg_type != USB_TYPE_QC3P_27) {
		sc8989x_set_charging_current(sc->chg_dev,3000000);
	} else {
		sc8989x_set_charging_current(sc->chg_dev,500000);
	}
}

void mmi_start_hvdcp_detect_work(struct work_struct *work)
{
	struct delayed_work *mmi_hvdcp_detect_dwork = NULL;
	struct sc8989x_chip *sc = NULL;
	int ret;
	int vbus_uv = 0;

	mmi_hvdcp_detect_dwork = container_of(work, struct delayed_work, work);
	if(mmi_hvdcp_detect_dwork == NULL) {
		pr_err("Can't get mmi_hvdcp_detect_dwork\n");
		return ;
	}
	sc = container_of(mmi_hvdcp_detect_dwork, struct sc8989x_chip, mmi_hvdcp_detect_dwork);
	if(sc == NULL) {
		pr_err("Can't get sc8989x_chip \n");
		return ;
	}

	if (!sc->mmi_hvdcp_support) {
		pr_err("HVDCP: mmi hvdcp don't support, exit \n");
		return;
	}

	if (!IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = sc8989x_get_vbus(sc->chg_dev, &vbus_uv);
		if (ret < 0) {
			pr_err("%s get vbus failed\n",__func__);
			return;
		}
	}
	pr_info("HVDCP: %s get vbus %d uv\n",__func__,vbus_uv);
	if (is_pd_rdy(sc) || vbus_uv > MMI_HVDCP2_VOLTAGE_STANDARD) {
		pr_info("HVDCP: pd adaptor ready, exit qc detected\n");
		return;
	}

	if (vbus_uv < HVDCP_VOLTAGE_MIN) {
		pr_err("HVDCP: vbus_uv < 4V, exit qc detected\n");
		return;
	}

	pr_info("HVDCP: mmi start hvdcp detect\n");
	sc->mmi_hvdcp_trig_flag = true;
	wake_up_interruptible(&sc->mmi_hvdcp_wait_que);
}

static int sc8989x_detected_qc3p_hvdcp(struct sc8989x_chip *sc, int *charger_type)
{
	int ret = 0;
	int i=0, vbus_voltage;

	if (!sc) {
		return -EINVAL;
	}
	msleep(QC3P_MSLEEP_100DELAY);

	ret = sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	if (ret < 0) {
		pr_err("%s get vbus failed\n",__func__);
		return -EINVAL;
	}

	if (vbus_voltage < QC3P_AUTHEN_LOW_THR_MV
		|| vbus_voltage > QC3P_AUTHEN_HIGH_THR_MV) {
		/*do qc3p rerun*/
		pr_err("HVDCP: qc3p voltage is invalid\n");
		return -EINVAL;
	}

	for (i = 0;i < 3; i++) {
		ret = sc8989x_qc30_step_up_vbus(sc);
		if (ret)
			pr_err("HVDCP: %s qc3p step up vbus error\n", __func__);

		ret = sc8989x_qc30_step_down_vbus(sc);
		if (ret)
			pr_err("HVDCP: %s qc3p step down vbus error\n", __func__);

	}

	msleep(QC3P_MSLEEP_30DELAY);

	ret = sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	if (ret < 0) {
		pr_err("%s get vbus failed\n",__func__);
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		ret = sc8989x_qc30_step_up_vbus(sc);
		if (ret)
			pr_err("HVDCP: %s qc3p step up vbus error\n", __func__);
	}

	for (i = 0; i < 2; i++) {
		ret = sc8989x_qc30_step_down_vbus(sc);
		if (ret)
			pr_err("HVDCP: %s qc3p step down vbus error\n", __func__);
	}

	msleep(QC3P_MSLEEP_30DELAY);

	pr_info("HVDCP: %s vbus voltage now = %d after detected qc3p\n", __func__,vbus_voltage);

	if (vbus_voltage > QC3P_AUTHEN_NONE_THR_MV)
		sc->mmi_qc3p_power = MMI_POWER_SUPPLY_QC3P_NONE;
	else if (vbus_voltage > QC3P_AUTHEN_45W_THR_MV) {
		sc->mmi_qc3p_power = MMI_POWER_SUPPLY_QC3P_45W;
		if (charger_type == NULL)
			return -EINVAL;
		*charger_type = USB_TYPE_QC3P_45;
	} else if (vbus_voltage > QC3P_AUTHEN_27W_THR_MV) {
		sc->mmi_qc3p_power = MMI_POWER_SUPPLY_QC3P_27W;
		if (charger_type == NULL)
			return -EINVAL;
		*charger_type = USB_TYPE_QC3P_27;
	} else if (vbus_voltage > QC3P_AUTHEN_18W_THR_MV) {
		sc->mmi_qc3p_power = MMI_POWER_SUPPLY_QC3P_18W;
		if (charger_type == NULL)
			return -EINVAL;
		*charger_type = USB_TYPE_QC3P_18;
	} else
		sc->mmi_qc3p_power = MMI_POWER_SUPPLY_QC3P_NONE;

	if (sc->mmi_qc3p_power != MMI_POWER_SUPPLY_QC3P_NONE) {
		if (charger_type == NULL)
			return -EINVAL;
		pr_info("HVDCP: %s detected qc3p, qc3p power = %d, *charger_type=%d\n",
					__func__, sc->mmi_qc3p_power, *charger_type);
	} else {
		pr_err("HVDCP: qc3p power is invalid\n");
		return -EINVAL;
	}

	return ret;
}
static int sc8989x_detected_qc20_hvdcp(struct sc8989x_chip *sc, int *charger_type)
{
	int ret;
	int vbus_voltage;

	if (!sc || !charger_type) {
		return -EINVAL;
	}
	//do qc2.0 detected
	ret = sc8989x_enable_qc20_hvdcp_9v(sc);
	if (ret) {
		dev_err(sc->dev, "Can't enable qc20 hvdcp 9V\n");
		return ret;
	}

	msleep(QC3P_MSLEEP_300DELAY);

	sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	dev_info(sc->dev, "vbus voltage now = %d\n", vbus_voltage);

	if (vbus_voltage > MMI_HVDCP2_VOLTAGE_STANDARD) {
		dev_info(sc->dev, "QC20 charger detected\n");
		if (charger_type == NULL)
			return -EINVAL;
		*charger_type = USB_TYPE_QC20;
		ret = sc8989x_adjust_qc20_hvdcp_5v(sc);
		if (ret) {
			dev_err(sc->dev, "Can't adjust qc20 hvdcp 5V\n");
		}
	} else {
		*charger_type = USB_TYPE_UNKNOWN;
		dev_info(sc->dev, "charger type is not HVDCP\n");
		return ret;
	}

	msleep(QC3P_MSLEEP_300DELAY);
	sc8989x_get_vbus(sc->chg_dev, &vbus_voltage);
	dev_info(sc->dev, "vbus voltage now = %d after qc20 detected\n", vbus_voltage);
	return ret;
}

static int mmi_hvdcp_detect_kthread(void *param)
{
	struct sc8989x_chip *sc = param;
	int ret;
	int charger_type = USB_TYPE_UNKNOWN;
	union power_supply_propval val;
	int vbus_uv;

	do {

		wait_event_interruptible(sc->mmi_hvdcp_wait_que, sc->mmi_hvdcp_trig_flag || kthread_should_stop());
		if (kthread_should_stop())
			break;

		//down(&sc->sem_dpdm);
		sc->qc_is_start_detect = true;
		sc->mmi_hvdcp_trig_flag = false;
		charger_type = USB_TYPE_UNKNOWN;
		if (sc->icharge_default_value > 0) {
			sc8989x_set_charging_current(sc->chg_dev,sc->icharge_default_value);
		} else {
			sc8989x_set_charging_current(sc->chg_dev,1000000);
		}
		//mt6375_chg_field_set(sc, F_IAICR, 500);
		//mt6375_chg_set_usbsw(sc, USBSW_CHG);

rerun:
		ret = sc8989x_get_vbus(sc->chg_dev, &vbus_uv);
		if (ret < 0) {
			pr_err("%s get vbus failed\n",__func__);
			goto out;
		}

		if (is_pd_rdy(sc) || vbus_uv > MMI_HVDCP2_VOLTAGE_STANDARD) {
			pr_info("HVDCP: pd adaptor ready, exit qc detected\n");
			goto out;
		}

		if (vbus_uv < HVDCP_VOLTAGE_MIN) {
			pr_err("HVDCP: vbus_uv < 4V, exit qc detected\n");
			goto out;
		}

		sc->qc_is_detect = true;
		Charger_Detect_Init(sc);
		pr_info("HVDCP: mmi_hvdcp_detect_kthread begin\n");

		//do qc2.0 detected
		ret = sc8989x_detected_qc20_hvdcp(sc, &charger_type);
		if (ret) {
			pr_err("HVDCP: Can't detect qc20 hvdcp\n");
			goto out;
		}
		sc->qc_chg_type = charger_type;
		charger_dev_notify(sc->chg_dev, CHARGER_DEV_NOTIFY_CTD_DONE);

		if (charger_type != USB_TYPE_QC20)
			goto out;

		//do qc3.0 detected
		ret = sc8989x_detected_qc30_hvdcp(sc, &charger_type);
		if (ret) {
			pr_err("HVDCP: Can't detect qc30 hvdcp\n");
		}

		//do qc3p detected
		if (charger_type == USB_TYPE_QC30) {
			ret = sc8989x_detected_qc3p_hvdcp(sc, &charger_type);
			if (ret) {
				if (sc->mmi_qc3p_rerun_done == false) {
					pr_info("HVDCP: Rerun detect hvdcp\n");
					sc->mmi_qc3p_rerun_done = true;
					//pull down dpdm for rerun HVDCP detected
					sc8989x_set_dpdm_0V(sc);
					msleep(QC3P_MSLEEP_100DELAY);
					goto rerun;
				} else {
					pr_err("HVDCP: Can't detect qc3p hvdcp\n");
				}
			}
		}

		ret = power_supply_get_property(sc->psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!val.intval)
			goto out;

		sc->qc_chg_type = charger_type;
		sc->qc_is_detect = false;

		if(sc->qc_chg_type == USB_TYPE_QC20) {
			/* dp 3.3v and dm 0.6v out 9V */
			if (sc->is_cx25890HQ) {
				ret = sc8989x_field_write(sc, CX_DP_DRIVE, DPDM_DRIVE_3V3);
			} else {
				ret = sc8989x_field_write(sc, DP_DRIVE, DPDM_DRIVE_3V3);
			}
			if (ret)
				goto out;
			if (sc->is_cx25890HQ) {
				ret = sc8989x_field_write(sc, CX_DM_DRIVE, DPDM_DRIVE_0V6);
			} else {
				ret = sc8989x_field_write(sc, DM_DRIVE, DPDM_DRIVE_0V6);
			}
			if (ret)
				goto out;
			dev_info(sc->dev,"Force set qc2 9V");
		}

		//notify charging policy to update charger type
		if ((sc->qc_chg_type == USB_TYPE_QC30)
			|| (sc->qc_chg_type == USB_TYPE_QC3P_18 )
			|| (sc->qc_chg_type == USB_TYPE_QC3P_27)
			|| (sc->qc_chg_type == USB_TYPE_QC3P_45)) {
			charger_dev_notify(sc->chg_dev, CHARGER_DEV_NOTIFY_CTD_DONE);
		}

		msleep(QC3P_MSLEEP_300DELAY);

out:

		sc->qc_is_detect = false;
		if ((sc->qc_chg_type == USB_TYPE_QC30)
			&& (sc->only_buck_qc_support)) {
			mmi_config_qc30_charger_voltage(sc->chg_dev);
		} else if ((sc->qc_chg_type != USB_TYPE_QC3P_18 )
			&& (sc->qc_chg_type != USB_TYPE_QC3P_27)
			&& (sc->qc_chg_type != USB_TYPE_QC3P_45)) {
			sc8989x_set_dpdm_0V(sc);
			sc8989x_set_dpdm_hiz(sc);
		}
		sc->qc_is_start_detect = false;
		pr_info("HVDCP: mmi_hvdcp_detect_kthread end\n");
	}while(!kthread_should_stop());

	pr_info("HVDCP: qc3 kthread stop\n");
	return 0;
}

static struct charger_ops sc8989x_chg_ops = {
	/* Normal charging */
	.plug_in = sc8989x_plug_in,
	.plug_out = sc8989x_plug_out,
	.enable = sc8989x_enable,
	.is_enabled = sc8989x_is_enabled,
	.get_charging_current = sc8989x_get_charging_current,
	.set_charging_current = sc8989x_set_charging_current,
	.get_input_current = sc8989x_get_input_current,
	.set_input_current = sc8989x_set_input_current,
	.get_constant_voltage = sc8989x_get_constant_voltage,
	.set_constant_voltage = sc8989x_set_constant_voltage,
	.kick_wdt = sc8989x_kick_wdt,
	.set_mivr = sc8989x_set_ivl,
	.get_mivr = sc8989x_get_mivr_voltage,
	.get_mivr_state = sc8989x_get_mivr_state,
	.is_charging_done = sc8989x_is_charging_done,
	.get_min_charging_current = sc8989x_get_min_ichg,
	.dump_registers = sc8989x_dump_registers,

	/* Safety timer */
	.enable_safety_timer = sc8989x_set_safety_timer,
	.is_safety_timer_enabled = sc8989x_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = sc8989x_enable_powerpath,
	//.is_powerpath_enabled = sc8989x_is_powerpath_enabled,

	/* OTG */
	.enable_otg = sc8989x_set_otg,
	.set_boost_current_limit = sc8989x_set_boost_ilmt,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = sc8989x_send_ta_current_pattern,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = sc8989x_send_ta20_current_pattern,
	.reset_ta = sc8989x_set_ta20_reset,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_vbus_adc = sc8989x_get_vbus,
	.get_adc = sc8989x_get_adc,
	.set_eoc_current = sc8989x_set_eoc_curr,
	.event = sc8989x_do_event,
	.enable_hz = sc8989x_enable_hz,
#if IS_ENABLED(CONFIG_OEM_TURBO_CHARGER)
	.enable_dpdm_hz = sc8989x_enable_dpdm_hiz,
#endif
	.enable_termination = sc8989x_enable_terminate,
	.get_chip_id = sc8989x_get_chip_id,
	.qc_is_detect = mmi_qc_is_detect,
	.get_protocol = mmi_get_protocol,
	.config_qc_charger = mmi_config_qc_charger,
	.set_dp_dm = mmi_set_dp_dm,
	.get_dp_dm = mmi_get_dp_dm,
};

static const struct charger_properties sc8989x_chg_props = {
	.alias_name = "sc8989x_chg",
};

#ifdef CONFIG_MTK_CHARGER_V4P19
static void sc8989x_inform_psy_dwork_handler(struct work_struct *work)
{
	int ret = 0;
	union power_supply_propval propval;
	struct sc8989x_chip *sc = container_of(work,
		struct sc8989x_chip, psy_dwork.work);

	if (!sc->chg_psy) {
		sc->chg_psy = power_supply_get_by_name("charger");
		if (!sc->chg_psy) {
			pr_err("%s get power supply fail\n", __func__);
			mod_delayed_work(system_wq, &sc->psy_dwork, msecs_to_jiffies(2000));
			return ;
		}
	}

	if (sc->adp_type != CHARGER_UNKNOWN)
		propval.intval = 1;
	else
		propval.intval = 0;

	ret = power_supply_set_property(sc->chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &propval);

	if (ret < 0)
		pr_notice("inform power supply online failed:%d\n", ret);

	propval.intval = sc->adp_type;

	ret = power_supply_set_property(sc->chg_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_notice("inform power supply charge type failed:%d\n", ret);
}
#endif /*CONFIG_MTK_CHARGER_V4P19*/

static void sc8989x_set_chg_type(struct sc8989x_chip *sc, int type)
{
#ifdef CONFIG_MTK_CHARGER_V4P19
	switch (type) {
	case VBUS_STAT_NO_INPUT:
		sc->adp_type = CHARGER_UNKNOWN;
		break;
	case VBUS_STAT_SDP:
	case VBUS_STAT_CDP:
	case VBUS_STAT_UNKOWN:
		sc->adp_type = STANDARD_HOST;
		//Charger_detect_Release();
		break;
	case VBUS_STAT_DCP:
	case VBUS_STAT_HVDCP:
		sc->adp_type = STANDARD_CHARGER;
		break;
	case VBUS_STAT_NONSTAND:
		sc->adp_type = NONSTANDARD_CHARGER;
		break;
	default:
		sc->adp_type = CHARGER_UNKNOWN;
		break;
	}

	schedule_delayed_work(&sc->psy_dwork, 0);
#endif /*CONFIG_MTK_CHARGER_V4P19*/
}

/**********************interrupt*********************/
static int sc8989x_get_charger_type(struct sc8989x_chip *sc);

__maybe_unused static int sc8989x_wait_power_good(struct sc8989x_chip *sc)
{
	int ret = 0;
	int reg_val;
	int tries = 0;

	dev_info(sc->dev," sc8989x_wait_power_good \n");
	while (tries < MAX_TRY) {
		ret = sc8989x_field_read(sc, PG_STAT, &reg_val);
		if (reg_val == 1) {
			dev_info(sc->dev,"sc8989x_wait_power_good GD ,try %d times\n", tries);
			break;
		} else {
			msleep(10);
			tries++;
		}
	}

	if (tries == MAX_TRY) {
		dev_info(sc->dev,"sc8989x_wait_power_good alwalys is 0,try %d times?\n", tries);
	}
	return ret;
}

static void sc8989x_force_detection_dwork_handler(struct work_struct *work)
{
	int ret;
	struct sc8989x_chip *sc = container_of(work,
				struct sc8989x_chip, force_detect_dwork.work);

	Charger_Detect_Init(sc);
	ret = sc8989x_force_dpdm(sc);
	if (ret) {
		dev_err(sc->dev, "%s: force dpdm failed(%d)\n", __func__, ret);
		return;
	}

	sc->force_detect_count++;
	//if (sc->is_upm6920A)
		return;
	msleep(600);

	sc8989x_get_charger_type(sc);
	power_supply_changed(sc->psy);
}

static int sc8989x_get_vbus_stat(struct sc8989x_chip *sc);
static void upm6920a_ibus_enable_dwork(struct work_struct *work)
{
	int ret;
	int type = 0;
	struct sc8989x_chip *sc = container_of(work,
				struct sc8989x_chip, ibus_enable_dwork.work);
	int vbus_good;

	atomic_set(&sc->reset_vindpm, 1);
	sc->ibus_dis = 0;
	ret = sc8989x_set_vindpm(sc, 4600);
	ret = sc8989x_field_read(sc, VBUS_GD, &vbus_good);
	if (ret) {
		return;
	}
	dev_info(sc->dev,"ret=%d,%d ibus_enabled vindpm_reset \n", ret, vbus_good);
	if (!sc->is_upm6920A) {
		type = sc8989x_get_vbus_stat(sc);
		if (vbus_good && type == VBUS_STAT_NO_INPUT) {
			schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(10));
		}
	}
}

static int sc8989x_get_vbus_stat(struct sc8989x_chip *sc)
{
	int ret;
	int reg_val = 0;

	ret = sc8989x_field_read(sc, VBUS_STAT, &reg_val);
	dev_info(sc->dev, "%s: charger type: %d\n", __func__, reg_val);
	if (ret) {
		return ret;
	}
	return reg_val;
}

static int sc8989x_do_bc12(struct sc8989x_chip *sc)
{
	int ret = 0;
	int reg_val;
	int tries = 0;
	int reset_vindpm = 0;

	dev_info(sc->dev," sc8989x_do_bc12 \n");
	reset_vindpm = atomic_read(&sc->reset_vindpm);
	if (reset_vindpm == 0) {
		ret = sc8989x_set_vindpm(sc, 12000);
		sc->ibus_dis = 1;
	}
	while (tries < MAX_TRY) {
		ret = sc8989x_field_read(sc, VBUS_GD, &reg_val);
		if (reg_val == 1) {
			dev_info(sc->dev,"VBUS GD ,try %d times\n", tries);
			/* For UPM6920A, this work only triggers the BC1.2 detection.
			 * The result is handled asynchronously in the IRQ handler via the
			 * BC_12_DONE interrupt. For other chips, this work handles both
			 * triggering and polling for the result.
			 */
			if (sc->is_upm6920A)
				schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(80));
			else
				schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(80));
			break;
		} else {
			msleep(10);
			tries++;
		}
	}

	if (tries == MAX_TRY) {
		dev_info(sc->dev,"VBUS GD alwalys is 0,try %d times?\n", tries);
    }

    return ret;
}

static int sc8989x_get_charger_type(struct sc8989x_chip *sc)
{
	int ret;
	int reg_val = 0;

	if (is_factory_build()) {
		sc8989x_wait_power_good(sc);
	}
	ret = sc8989x_field_read(sc, VBUS_STAT, &reg_val);
	if (ret) {
		return ret;
	}

	switch (reg_val) {
	case VBUS_STAT_NO_INPUT:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sc->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		dev_info(sc->dev, "%s: charger type: NO INPUT\n", __func__);
		break;
	case VBUS_STAT_SDP:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sc->chg_type = POWER_SUPPLY_TYPE_USB;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		dev_info(sc->dev, "%s: charger type: SDP\n", __func__);
		break;
	case VBUS_STAT_CDP:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		sc->chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		dev_info(sc->dev, "%s: charger type: CDP\n", __func__);
		break;
	case VBUS_STAT_DCP:
	case VBUS_STAT_HVDCP:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		sc->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		dev_info(sc->dev, "%s: charger type: DCP\n", __func__);
		if (sc->qc_dev)
			schedule_delayed_work(&sc->detect_qc_dwork, msecs_to_jiffies(MMI_HVDCP_DETECT_TIMER)); //for wait PD detected complete
		else if (sc->mmi_hvdcp_support)
			schedule_delayed_work(&sc->mmi_hvdcp_detect_dwork, msecs_to_jiffies(0)); //start HVDCP detection immediately to reduce UI latency
		break;
	case VBUS_STAT_UNKOWN:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sc->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		dev_info(sc->dev, "%s: charger type: UNKNOWN\n", __func__);
		if (sc->force_detect_count < 3) {
			dev_info(sc->dev, "[%s] SC8989x charger type: UNKNOWN, retry bc12 count:%d\n", __func__, sc->force_detect_count);
			schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(100));
		}
		break;
	case VBUS_STAT_NONSTAND:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		sc->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		dev_info(sc->dev, "%s: charger type: NON STANDARD\n", __func__);
		if (sc->force_detect_count < 10) {
			dev_info(sc->dev, "[%s] SC8989x charger type: NON STANDARD, retry bc12 count:%d\n", __func__, sc->force_detect_count);
			schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(100));
		}
		break;
	default:
		sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sc->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		dev_info(sc->dev, "%s: charger type: default\n", __func__);
		break;
	}

	if (sc->chg_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		if (sc->wait_hiz) {
			sc->wait_hiz = 0;
			ret = sc8989x_normal_set_hiz(sc,true);
		}
	}

	sc8989x_set_chg_type(sc, reg_val);

	if (reg_val == VBUS_STAT_SDP || reg_val == VBUS_STAT_CDP) {
		Charger_Detect_Release(sc);
	}

	dev_info(sc->dev, "%s vbus stat: 0x%02x\n", __func__, reg_val);

	return ret;
}

static irqreturn_t sc8989x_irq_handler(int irq, void *data)
{
	int ret;
	int reg_val;
	bool prev_vbus_gd;
	//int type;
	bool bc12_done = 0;
	struct sc8989x_chip *sc = (struct sc8989x_chip *)data;
	bool pre_power_gd;

	if (sc == NULL) {
		return IRQ_HANDLED;
	}

	dev_info(sc->dev, "%s: sc8989x_irq_handler\n", __func__);
	if (sc->is_upm6920A) {
		ret = sc8989x_field_read(sc, BC_12_DONE, &reg_val);
		if (ret) {
			return IRQ_HANDLED;
		}
		bc12_done = !!reg_val;
	} else if (sc->is_sc89890h && is_apdo_rdy(sc)) {
		if (sc->hiz_cut_flag) {
			dev_info(sc->dev, "%s: sc8989x in the process of disable hiz\n", __func__);
			return IRQ_HANDLED;
		}
	}

	ret = sc8989x_field_read(sc, VBUS_GD, &reg_val);
	if (ret) {
		return IRQ_HANDLED;
	}
	if ((reg_val ==0) && (atomic_read(&sc->vbus_good_flag)))
	{
		dev_info(sc->dev, "%s: enter tcmd sc8989x_irq_handler test\n", __func__);
		return IRQ_HANDLED;
	}
	prev_vbus_gd = sc->vbus_good;
	sc->vbus_good = !!reg_val;
	dev_info(sc->dev, "%s: prev_vbus_gd:%d, vbus_gd:%d\n", __func__, prev_vbus_gd, sc->vbus_good);

	ret = sc8989x_field_read(sc, PG_STAT, &reg_val);
	if (ret) {
		return IRQ_HANDLED;
	}
	pre_power_gd = sc->power_good;
	sc->power_good = !!reg_val;
	dev_info(sc->dev, "%s: pre_power_gd:%d, power_good:%d\n", __func__, pre_power_gd, sc->power_good);
	if (!prev_vbus_gd && sc->vbus_good) {
		dev_info(sc->dev, "%s: adapter/usb inserted\n", __func__);
		ret = sc8989x_set_vindpm(sc, 12000);
		schedule_delayed_work(&sc->ibus_enable_dwork, msecs_to_jiffies(1500));
		/*sc->force_detect_count = 0;
		type = sc8989x_get_vbus_stat(sc);
		Charger_Detect_Init(sc);
		sc->retry_count = 0;
		dev_info(sc->dev, "%s: adapter/usb inserted\n", __func__);
		if ((type == VBUS_STAT_NO_INPUT) && (sc->retry_count <  SPECIAL_TYPE_MAX_RETRY)) {
			schedule_delayed_work(&sc->force_detect_dwork, msecs_to_jiffies(500));
			++(sc->retry_count);
		}*/

#if IS_ENABLED(CONFIG_WLC_WO_BOOST)
		sc8989x_set_vindpm_track(sc, sc->cfg->vindpm_track);
#else
		sc8989x_set_vindpm_track(sc, SC8989X_TRACK_300);
#endif
		if (sc->icharge_default_value > 0) {
			sc8989x_set_charging_current(sc->chg_dev,sc->icharge_default_value);
		} else {
			sc8989x_set_charging_current(sc->chg_dev,1000000);
		}
		dev_info(sc->dev, "%s: set icc 1000ma\n", __func__);
#if IS_ENABLED(CONFIG_FACTORY_BUILD)
		ret = sc8989x_set_chg_enable(sc, false);
		if (ret)
			dev_err(sc->dev, "%s disable charging failed\n", __func__);
#endif
	} else if (prev_vbus_gd && !sc->vbus_good) {
		dev_info(sc->dev, "%s: adapter/usb removed\n", __func__);
		Charger_Detect_Release(sc);
		cancel_delayed_work(&sc->ibus_enable_dwork);
		sc->ibus_dis = 0;
		atomic_set(&sc->reset_vindpm, 0);
		if (sc->is_upm6920A || sc->is_sc89890h) {
			if (sc->psy_usb_type == POWER_SUPPLY_USB_TYPE_DCP &&
				sc->chg_type == POWER_SUPPLY_TYPE_USB_DCP) {
				sc8989x_set_dpdm_0V(sc);
			}
		}
		sc8989x_set_dpdm_hiz(sc);
		sc->qc_chg_type = 0;
		if (sc->batt_psy)
			power_supply_changed(sc->batt_psy);
		//sc8989x_get_charger_type(sc);
		power_supply_changed(sc->psy);
	}
	if (!sc->is_upm6920A) {
		if (!pre_power_gd && sc->power_good && sc->vbus_good) {
			bc12_done = 1;
		}
	}
	if (bc12_done) {
		sc8989x_get_charger_type(sc);
		power_supply_changed(sc->psy);
	}

	sc8989x_dump_register(sc);
	return IRQ_HANDLED;
}

/**********************system*********************/
static int sc8989x_parse_dt(struct sc8989x_chip *sc)
{
	struct device_node *np = sc->dev->of_node;
	int i;
	int ret = 0;
	struct {
		char *name;
		int *conv_data;
	} props[] = {
		{"sc,sc8989x,ico-en", &(sc->cfg->ico_en)},
		{"sc,sc8989x,hvdcp-en", &(sc->cfg->hvdcp_en)},
		{"sc,sc8989x,auto-dpdm-en", &(sc->cfg->auto_dpdm_en)},
		{"sc,sc8989x,vsys-min", &(sc->cfg->vsys_min)},
		{"sc,sc8989x,vbatmin-sel", &(sc->cfg->vbatmin_sel)},
		{"sc,sc8989x,itrick", &(sc->cfg->itrick)},
		{"sc,sc8989x,iterm", &(sc->cfg->iterm)},
		{"sc,sc8989x,vbat-cv", &(sc->cfg->vbat_cv)},
		{"sc,sc8989x,vbat-low", &(sc->cfg->vbat_low)},
		{"sc,sc8989x,vrechg", &(sc->cfg->vrechg)},
		{"sc,sc8989x,en-term", &(sc->cfg->en_term)},
		{"sc,sc8989x,stat-dis", &(sc->cfg->stat_dis)},
		{"sc,sc8989x,wd-time", &(sc->cfg->wd_time)},
		{"sc,sc8989x,en-timer", &(sc->cfg->en_timer)},
		{"sc,sc8989x,charge-timer", &(sc->cfg->charge_timer)},
		{"sc,sc8989x,bat-comp", &(sc->cfg->bat_comp)},
		{"sc,sc8989x,vclamp", &(sc->cfg->vclamp)},
		{"sc,sc8989x,votg", &(sc->cfg->votg)},
		{"sc,sc8989x,iboost", &(sc->cfg->iboost)},
		{"sc,sc8989x,vindpm", &(sc->cfg->vindpm)},
		{"sc,vindpm-track", &(sc->cfg->vindpm_track)},
	};

	sc->mmi_hvdcp_support = of_property_read_bool(np, "mmi,hvdcp-support");
	sc->only_buck_qc_support = of_property_read_bool(np, "mmi,only_buck_qc_support");

	sc->irq_gpio = of_get_named_gpio(np, "sc,intr-gpio", 0);
	if (sc->irq_gpio < 0)
		dev_err(sc->dev, "%s sc,intr-gpio is not available\n", __func__);

	if (of_property_read_string(np, "charger_name", &sc->cfg->chg_name) < 0) {
		sc->cfg->chg_name = "primary_chg";
		dev_err(sc->dev, "%s no charger name\n", __func__);
	}

	/* initialize data for optional properties */
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = of_property_read_u32(np, props[i].name, props[i].conv_data);
		if (ret < 0) {
			dev_err(sc->dev, "%s not find\n", props[i].name);
			continue;
		}
	}

	ret = of_property_read_u32(np, "sc,upm6920,iterm", &sc->upm6920_iterm);
	if (ret < 0) {
		dev_err(sc->dev, "%s not find\n", "sc,upm6920,iterm");
		sc->upm6920_iterm = 0;
	}

	if (sc->is_upm6920A && (sc->upm6920_iterm != 0)) {
		sc->cfg->iterm = sc->upm6920_iterm;
	}

	ret = of_property_read_u32(np, "sc,sc8989x,icharge", &sc->icharge_default_value);
	if (ret < 0) {
		dev_err(sc->dev, "%s not find\n", "sc,sc8989x,icharge");
		sc->icharge_default_value = 0;
	}

	ret = of_property_read_u32(np, "sc,cx25890HQ,iterm", &sc->cx25890HQ_iterm);
	if (ret < 0) {
		dev_err(sc->dev, "%s not find\n", "sc,cx25890HQ,iterm");
		sc->cx25890HQ_iterm = 0;
	}

	if (sc->is_cx25890HQ && (sc->cx25890HQ_iterm != 0)) {
		sc->cfg->iterm = sc->cx25890HQ_iterm;
	}

	ret = of_property_read_u32(np, "sc,cx25890HQ,votg", &sc->cx25890HQ_votg);
	if (ret < 0) {
		dev_err(sc->dev, "%s not find\n", "sc,cx25890HQ,votg");
		sc->cx25890HQ_votg = 0;
	}

	if (sc->is_cx25890HQ) {
		if (sc->cx25890HQ_votg != 0)
			sc->cfg->votg = sc->cx25890HQ_votg;

		/*The cx25890HQ must set auto dpdm en to 1*/
		sc->cfg->auto_dpdm_en = 1;
	}


	ret = of_property_read_u32(np, "sc,upm6920A,votg", &sc->upm6920A_votg);
	if (ret < 0) {
		dev_err(sc->dev, "%s not find\n", "sc,upm6920A,votg");
		sc->upm6920A_votg = 0;
	}

	if (sc->is_upm6920A && (sc->upm6920A_votg != 0)) {
		sc->cfg->votg = sc->upm6920A_votg;
	}

	return 0;
}

static int sc8989x_init_device(struct sc8989x_chip *sc)
{
	int ret = 0;
	int i;

	struct {
		enum sc8989x_fields field_id;
		int conv_data;
	} props[] = {
		{ICO_EN, sc->cfg->ico_en},
		{HVDCP_EN, sc->cfg->hvdcp_en},
		{AUTO_DPDM_EN, sc->cfg->auto_dpdm_en},
		{VSYS_MIN, sc->cfg->vsys_min},
		{VBATMIN_SEL, sc->cfg->vbatmin_sel},
		{ITC, sc->cfg->itrick},
		{ITERM, sc->cfg->iterm},
		{CV, sc->cfg->vbat_cv},
		{VBAT_LOW, sc->cfg->vbat_low},
		{VRECHG, sc->cfg->vrechg},
		{EN_ITERM, sc->cfg->en_term},
		{STAT_DIS, sc->cfg->stat_dis},
		{TWD, sc->cfg->wd_time},
		{EN_TIMER, sc->cfg->en_timer},
		{TCHG, sc->cfg->charge_timer},
		{BAT_COMP, sc->cfg->bat_comp},
		{VCLAMP, sc->cfg->vclamp},
		{V_OTG, sc->cfg->votg},
		{IBOOST_LIM, sc->cfg->iboost},
		{VINDPM, sc->cfg->vindpm},
	};

	//reg reset;
	sc8989x_field_write(sc, REG_RST, 1);
	if (sc->icharge_default_value > 0) {
		sc8989x_set_ichg(sc, sc->icharge_default_value / 1000);
	}

	if (sc->is_cx25890HQ) {
		cx25890hq_write_reg40(sc,true);
		sc8989x_field_write(sc, CX_RESET_INTER, 0x83);
		sc8989x_field_write(sc, CX_OPT_HICCUP, 1);
		sc8989x_field_read(sc, CX_TRILIM, &sc->cx_otg_trilmt_init);
		sc8989x_field_write(sc, CX_HS_OCP, 0);
		cx25890hq_write_reg40(sc,false);
	}

	for (i = 0; i < ARRAY_SIZE(props); i++) {
//		dev_info(sc->dev, "%d--->%d\n", props[i].field_id, props[i].conv_data);
		ret = sc8989x_field_write(sc, props[i].field_id, props[i].conv_data);
	}

	sc8989x_adc_ibus_en(sc, true);
	sc8989x_set_tmr2x(sc, false);
	sc8989x_set_wa(sc);

	ret = sc8989x_dump_register(sc);
	return ret;
}

static int sc8989x_register_interrupt(struct sc8989x_chip *sc)
{
	int ret = 0;

	ret = devm_gpio_request(sc->dev, sc->irq_gpio, "chr-irq");
	if (ret < 0) {
		dev_err(sc->dev, "failed to request GPIO%d ; ret = %d", sc->irq_gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(sc->irq_gpio);
	if (ret < 0) {
		dev_err(sc->dev, "failed to set GPIO%d ; ret = %d", sc->irq_gpio, ret);
		return ret;
	}

	sc->irq = gpio_to_irq(sc->irq_gpio);
	if (ret < 0) {
		dev_err(sc->dev, "failed gpio to irq GPIO%d ; ret = %d", sc->irq_gpio, ret);
		return ret;
	}

	ret = devm_request_threaded_irq(sc->dev, sc->irq, NULL,
		sc8989x_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"chr_stat", sc);

	if (ret < 0) {
		dev_err(sc->dev, "request thread irq failed:%d\n", ret);
		return ret;
	} else {
		dev_err(sc->dev, "request thread irq pass:%d  sc->irq =%d\n", ret, sc->irq);
	}

	enable_irq_wake(sc->irq);

	return 0;
}

static ssize_t dump_reg_ctrl_write(struct file *filp,
	const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[8] = {0};
	long val = 0;
	int ret = 0;

	if (cnt >= sizeof(buf)) {
		pr_err( "%s cnt is invalid\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&buf, ubuf, cnt)) {
		pr_err("%s cnt is invalid\n", __func__);
		return -EFAULT;
	}

	buf[cnt] = 0;
	ret = kstrtoul(buf, 10, (unsigned long *)&val);
	if (ret < 0) {
		pr_err("%s cnt is invalid\n", __func__);
		return ret;
	}

	dump_reg_enable = val;
	pr_info("%s dump_reg_enable is %s\n", __func__, dump_reg_enable ? "enable" : "disable");

	return cnt;
}

static void determine_initial_status(struct sc8989x_chip *sc)
{
	sc8989x_irq_handler(sc->irq, (void *)sc);
}

static int dump_reg_ctrl_show(struct seq_file *m, void *v)
{
	seq_printf(m, "dump reg enable is %s\n", dump_reg_enable ? "enable" : "disable");

	return 0;
}

static int dump_reg_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_reg_ctrl_show, inode->i_private);
}

static const struct proc_ops dump_reg_ctrl_fops = {
	.proc_open = dump_reg_ctrl_open,
	.proc_write = dump_reg_ctrl_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
//reigster
static ssize_t sc8989x_show_registers(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sc8989x_chip *sc = dev_get_drvdata(dev);
	uint8_t addr;
	unsigned int val;
	uint8_t tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8989x");
	for (addr = 0x1; addr <= 0x14; addr++) {
		ret = regmap_read(sc->regmap, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sc8989x_store_register(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sc8989x_chip *sc = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	unsigned int reg;
	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x14)
		regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, sc8989x_show_registers, sc8989x_store_register);

static int sc8989x_create_device_node(struct device *dev)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_registers);
	if (ret < 0) {
		dev_err(dev, "[%s] failed to create register attr\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void sc8989x_destory_device_node(struct device *dev)
{
	device_remove_file(dev, &dev_attr_registers);
}

static enum power_supply_property sc8989x_chg_psy_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
};

static enum power_supply_usb_type sc8989x_chg_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
};

static int sc8989x_chg_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_MOTO_WLC_ALG_SUPPORT)
static int mmi_is_wireless_online(void) {
	static struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	if (info == NULL || IS_ERR(info)) {
		chg_psy = power_supply_get_by_name("mtk-master-charger");
		if (chg_psy == NULL || IS_ERR(chg_psy)) {
			pr_err("%s: get chg_psy failed\n", __func__);
			return 0;
		} else {
			info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
		}

	}

	return info->wireless_online;
}
#endif //get wireless online

static bool is_pd_rdy(struct sc8989x_chip *sc) {
	int type = 0;

	if (IS_ERR_OR_NULL(sc)) {
		pr_err("%s: sc is ERR or NULL\n", __func__);
		return false;
	}

	if (IS_ERR_OR_NULL(sc->pd_adapter)) {
		sc->pd_adapter = get_adapter_by_name("pd_adapter");
		if (IS_ERR_OR_NULL(sc->pd_adapter)) {
			pr_err("%s: No pd adapter found\n", __func__);
			return false;
		}
	}

	type = adapter_dev_get_property(sc->pd_adapter, PD_TYPE);
	//pr_info("%s pd_type: %d\n", __func__, type);

	if (type == MTK_PD_CONNECT_PE_READY_SNK_APDO ||
		type == MTK_PD_CONNECT_PE_READY_SNK ||
		type == MTK_PD_CONNECT_PE_READY_SNK_PD30)
		return true;
	else
		return false;
}

static int sc8989x_chg_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct sc8989x_chip *sc = power_supply_get_drvdata(psy);
	int ret = 0;
	int data = 0;
	int icl = 0;
	int tcpc_attach = 0;

	if (!sc) {
		dev_err(sc->dev, "%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "SouthChip";
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		tcpc_attach = atomic_read(&sc->attach);
		if (sc->vbus_good || tcpc_attach == ATTACH_TYPE_TYPEC)
			val->intval = 1;
		else
			val->intval = 0;
#if IS_ENABLED(CONFIG_MOTO_WLC_ALG_SUPPORT)
		if (sc->mmi_charging_full && mmi_is_wireless_online()) {
			break;
		}
#endif
		if (!sc->vbus_good)
			sc->mmi_charging_full = false;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc8989x_get_charge_stat(sc);
		if (ret < 0)
			break;
		val->intval = ret;
		if (sc->mmi_charging_full == true)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc8989x_get_ichg(sc, &data);
		if (ret)
			break;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc8989x_get_vbat(sc, &data);
		if (ret < 0)
			break;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc8989x_get_iindpm(sc, &data);
		if (ret < 0)
			break;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc8989x_get_vindpm(sc, &data);
		if (ret < 0)
			break;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		sc8989x_get_vbus(sc->chg_dev, &(val->intval));
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = sc8989x_get_term_curr(sc, &data);
		if (ret < 0)
			break;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = sc->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		sc8989x_get_iindpm(sc, &icl);
		if (is_pd_rdy(sc) && (icl > 500)
			&& !(sc->chg_type == POWER_SUPPLY_TYPE_USB))
		{
			val->intval = 3225000;
			break;
		}

		if (is_pd_rdy(sc) && (icl > 500)
			&& (sc->chg_type == POWER_SUPPLY_TYPE_USB))
		{
			val->intval = 2000000;
			break;
		}

		if (sc->psy_usb_type == POWER_SUPPLY_USB_TYPE_SDP)
			val->intval = 500000;
		else if (sc->psy_usb_type == POWER_SUPPLY_USB_TYPE_CDP)
			val->intval = 1500000;
		else if (sc->psy_usb_type == POWER_SUPPLY_USB_TYPE_DCP)
			val->intval = 3225000;
		else
			val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		sc8989x_get_iindpm(sc, &icl);
		if (is_pd_rdy(sc) && (icl > 500)
			&& !(sc->chg_type == POWER_SUPPLY_TYPE_USB))
			val->intval = 9000000;
		else
			val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
                val->intval = sc->disablehiz_isset_flg;
                break;
	case POWER_SUPPLY_PROP_TYPE:
#if IS_ENABLED(CONFIG_MOTO_WLC_ALG_SUPPORT)
		tcpc_attach = atomic_read(&sc->attach);
		if (!tcpc_attach && mmi_is_wireless_online()) {
			sc->psy_desc.type = POWER_SUPPLY_TYPE_WIRELESS;
		}
#endif
		val->intval = sc->psy_desc.type;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sc8989x_chg_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct sc8989x_chip *sc = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		atomic_set(&sc->attach, val->intval);
		if (val->intval == 2) {
			dev_info(sc->dev, "%s: %d, start charger detection\n", __func__, val->intval);
			sc8989x_do_bc12(sc);
		} else if (val->intval == 0) {
			sc->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			sc->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
			cancel_delayed_work(&sc->force_detect_dwork);
			power_supply_changed(sc->psy);
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc8989x_set_chg_enable(sc, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc8989x_set_ichg(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc8989x_set_vbat(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc8989x_set_iindpm(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc8989x_set_vindpm(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = sc8989x_set_term_curr(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		dev_info(sc->dev, "%s: %d, common charger\n", __func__, val->intval);
		sc->disablehiz_isset_flg = val->intval;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static char *sc8989x_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static const struct power_supply_desc sc8989x_psy_desc = {
	.name = "primary_chg",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = sc8989x_chg_psy_usb_types,
	.num_usb_types = ARRAY_SIZE(sc8989x_chg_psy_usb_types),
	.properties = sc8989x_chg_psy_properties,
	.num_properties = ARRAY_SIZE(sc8989x_chg_psy_properties),
	.property_is_writeable = sc8989x_chg_property_is_writeable,
	.get_property = sc8989x_chg_get_property,
	.set_property = sc8989x_chg_set_property,
};

static int sc8989x_psy_register(struct sc8989x_chip *sc)
{
	struct power_supply_config cfg = {
		.drv_data = sc,
		.of_node = sc->dev->of_node,
		.supplied_to = sc8989x_psy_supplied_to,
		.num_supplicants = ARRAY_SIZE(sc8989x_psy_supplied_to),
	};

	memcpy(&sc->psy_desc, &sc8989x_psy_desc, sizeof(sc->psy_desc));
	sc->psy = devm_power_supply_register(sc->dev, &sc->psy_desc, &cfg);

	return IS_ERR(sc->psy) ? PTR_ERR(sc->psy) : 0;
}

static struct of_device_id sc8989x_of_device_id[] = {
	{.compatible = "southchip,sc89890h",},
	{.compatible = "southchip,sc89890w",},
	{.compatible = "southchip,sc89895",},
	{.compatible = "southchip,sc8950",},
	{},
};

MODULE_DEVICE_TABLE(of, sc8989x_of_device_id);

static int sc8989x_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sc8989x_chip *sc;
	int ret = 0;
	int i;

	sc = devm_kzalloc(&client->dev, sizeof(struct sc8989x_chip), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->client = client;
	dev_info(sc->dev, "sc8989x_charger_probe start\n");

	//sc->regmap = devm_regmap_init_i2c(client, &sc8989x_regmap_config);
	sc->regmap = devm_regmap_init(sc->dev, &sc8989x_regmap_bus, sc,
                                    &sc8989x_regmap_config);
	if (IS_ERR(sc->regmap)) {
		dev_err(sc->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}
	sc->vbus = devm_iio_channel_get(sc->dev, "pmic_vbus");
	if (IS_ERR_OR_NULL(sc->vbus)) {
		dev_err(sc->dev, "sc89890h get vbus failed\n");
		return -EPROBE_DEFER;
	}

	for (i = 0; i < ARRAY_SIZE(sc8989x_reg_fields); i++) {
		const struct reg_field *reg_fields = sc8989x_reg_fields;
		sc->rmap_fields[i] = devm_regmap_field_alloc(sc->dev, sc->regmap, reg_fields[i]);
		if (IS_ERR(sc->rmap_fields[i])) {
			dev_err(sc->dev, "cannot allocate regmap field\n");
			return PTR_ERR(sc->rmap_fields[i]);
		}
	}

	i2c_set_clientdata(client, sc);
	sc8989x_create_device_node(&(client->dev));
	mutex_init(&sc->dpdm_lock);

	sc->qc_dev = get_adapter_by_name("qc_protocol_ic");
	if (sc->qc_dev) {
		dev_info(sc->dev, "Found qc protocol ic dev\n");
	} else {
		dev_info(sc->dev, "Don't find qc protocol ic dev\n");
	}

	INIT_DELAYED_WORK(&sc->force_detect_dwork, sc8989x_force_detection_dwork_handler);
	INIT_DELAYED_WORK(&sc->detect_qc_dwork, get_qc_charger_type_func_work);
	INIT_DELAYED_WORK(&sc->mmi_hvdcp_detect_dwork, mmi_start_hvdcp_detect_work);
	INIT_DELAYED_WORK(&sc->ibus_enable_dwork, upm6920a_ibus_enable_dwork);
	INIT_DELAYED_WORK(&sc->hiz_cut_dwork, sc8989x_hiz_cut_dwork_handler);

#ifdef CONFIG_MTK_CHARGER_V4P19
	INIT_DELAYED_WORK(&sc->psy_dwork, sc8989x_inform_psy_dwork_handler);
#endif /*CONFIG_MTK_CHARGER_V4P19*/
	if (!sc8989x_detect_device(sc)) {
		ret = -ENODEV;
		goto err_nodev;
	}

	sc->qc_is_start_detect = false;
	sc->cfg = &sc8989x_default_cfg;
	ret = sc8989x_parse_dt(sc);
	if (ret < 0) {
		dev_err(sc->dev, "parse dt fail(%d)\n", ret);
		goto err_parse_dt;
	}

	ret = sc8989x_init_device(sc);
	if (ret < 0) {
		dev_err(sc->dev, "init device fail(%d)\n", ret);
		goto err_init_dev;
	}

	ret = sc8989x_psy_register(sc);
	if (ret) {
		dev_err(sc->dev, "%s psy register fail(%d)\n", __func__, ret);
		goto err_psy;
	}

	if (sc->mmi_hvdcp_support) {
		sc->mmi_hvdcp_authen_task = kthread_create(mmi_hvdcp_detect_kthread, sc, "mmi_hvdcp_authen");
		if (IS_ERR(sc->mmi_hvdcp_authen_task)) {
			ret = PTR_ERR(sc->mmi_hvdcp_authen_task);
			dev_err(sc->dev, "Failed to create mmi_hvdcp_authen_task ret = %d\n", ret);
			return ret;
		}
		init_waitqueue_head(&sc->mmi_hvdcp_wait_que);
		wake_up_process(sc->mmi_hvdcp_authen_task);
	}

	ret = sc8989x_register_interrupt(sc);
	if (ret < 0) {
		dev_err(sc->dev, "%s register irq fail(%d)\n", __func__, ret);
		goto err_register_irq;
	}
	/* Register charger device */
	sc->chg_dev = charger_device_register(sc->cfg->chg_name,
		sc->dev, sc, &sc8989x_chg_ops, &sc8989x_chg_props);

	if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		dev_notice(sc->dev, "%s register chg dev fail(%d)\n", __func__, ret);
		goto err_register_chg_dev;
	}
	device_init_wakeup(sc->dev, 1);

	dump_reg_enable = true;
	sc->ibus_dis = 0;
	entry = proc_create("dump_reg_ctrl", 0664, NULL, &dump_reg_ctrl_fops);
	if (!entry) {
		dev_err(sc->dev, "%s create proc directory failed\n", __func__);
	}
	sc->mmi_charging_full = false;
	sc->power_good = 1;
	sc->disablehiz_isset_flg = false;
	sc->hiz_cut_flag = false;
	determine_initial_status(sc);
	sc8989x_dump_register(sc);

	sc->batt_psy = power_supply_get_by_name("battery");
	if (IS_ERR_OR_NULL(sc->batt_psy)) {
		dev_err(sc->dev, "Failed to get battery power supply\n");
		sc->batt_psy = NULL; // Ensure it's NULL on failure
	}

#if IS_ENABLED(CONFIG_OEM_DEVINFO)
	FULL_PRODUCT_DEVICE_INFO(ID_SWITCH_CHARGER, "SC89890H");
#endif

	dev_info(sc->dev, "sc8989x probe successfully\n!");

	return 0;

err_register_irq:
err_register_chg_dev:
err_psy:
err_init_dev:
err_parse_dt:
err_nodev:
	dev_err(sc->dev, "sc8989x probe failed!\n");
	devm_kfree(&client->dev, sc);
	return -ENODEV;
}

static int sc8989x_charger_remove(struct i2c_client *client)
{
	struct sc8989x_chip *sc = i2c_get_clientdata(client);

	if (sc) {
		dev_info(sc->dev, "%s\n", __func__);
		cancel_delayed_work_sync(&sc->detect_qc_dwork);
		cancel_delayed_work_sync(&sc->mmi_hvdcp_detect_dwork);
		kthread_stop(sc->mmi_hvdcp_authen_task);
		mutex_destroy(&sc->dpdm_lock);
		sc8989x_destory_device_node(sc->dev);
		charger_device_unregister(sc->chg_dev);
		power_supply_put(sc->psy);
		if (sc->batt_psy)
			power_supply_put(sc->batt_psy);
	}
	return 0;
}

static void sc8989x_charger_shutdown(struct i2c_client *client)
{
	struct sc8989x_chip *sc = i2c_get_clientdata(client);

	if (sc) {
		dev_info(sc->dev, "%s, reset registers\n", __func__);
		sc8989x_field_write(sc, REG_RST, 1);
	}
}

#ifdef CONFIG_PM_SLEEP
static int sc8989x_suspend(struct device *dev)
{
	struct sc8989x_chip *sc = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(sc->irq);

	return 0;
}

static int sc8989x_resume(struct device *dev)
{
	struct sc8989x_chip *sc = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		disable_irq_wake(sc->irq);

	return 0;
}

static const struct dev_pm_ops sc8989x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sc8989x_suspend, sc8989x_resume)
};
#endif /* CONFIG_PM_SLEEP */

static struct i2c_driver sc8989x_charger_driver = {
	.driver = {
		.name = "sc8989x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sc8989x_of_device_id),
#ifdef CONFIG_PM_SLEEP
		.pm = &sc8989x_pm_ops,
#endif
	},
	.probe = sc8989x_charger_probe,
	.remove = sc8989x_charger_remove,
	.shutdown = sc8989x_charger_shutdown,
};

module_i2c_driver(sc8989x_charger_driver);

MODULE_DESCRIPTION("SC SC8989X Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
