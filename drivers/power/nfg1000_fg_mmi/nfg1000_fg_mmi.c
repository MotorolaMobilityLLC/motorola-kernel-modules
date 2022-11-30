/*
 * nfg1000 fuel gauge driver
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[nfg1000] %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/power/moto_chg_tcmd.h>


#define mmi_info	pr_info
#define mmi_dbg	pr_debug
#define mmi_err	pr_err
#define mmi_log	pr_err

#define queue_delayed_work_time  8000
#define queue_start_work_time    1

#define	INVALID_REG_ADDR	0xFF

#define FG_FLAGS_FD				BIT(4)
#define	FG_FLAGS_FC				BIT(5)
#define	FG_FLAGS_DSG				BIT(6)
#define FG_FLAGS_RCA				BIT(9)

enum mmi_fg_reg_idx {
	BQ_FG_REG_CTRL = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_CURRENT,	/* Real Current */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_BATT_STATUS,	/* BatteryStatus */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_RM,		/* Remaining Capacity */
	BQ_FG_REG_CC,		/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DC,		/* Design Capacity */
	BQ_FG_REG_ALT_MAC,	/* AltManufactureAccess*/
	BQ_FG_REG_MAC_CHKSUM,	/* MACChecksum */
	NUM_REGS,
};

static u8 nfg1000_regs[NUM_REGS] = {
	0x00,	/* CONTROL */
	0x06,	/* TEMP */
	0x08,	/* VOLT */
	0x0C,	/* REAL CURRENT*/
	0x14,	/* AVG CURRENT */
	0x0A,	/* FLAGS */
	0x16,	/* Time to empty */
	0x18,	/* Time to full */
	0x12,	/* Full charge capacity */
	0x10,	/* Remaining Capacity */
	0x2A,	/* CycleCount */
	0x2C,	/* State of Charge */
	0x2E,	/* State of Health */
	0x3C,	/* Design Capacity */
	0x3E,	/* AltManufacturerAccess*/
	0x60,	/* MACChecksum */
};

enum mmi_fg_mac_cmd {
	FG_MAC_CMD_CTRL_STATUS	= 0x0000,
	FG_MAC_CMD_DEV_TYPE	= 0x0001,
	FG_MAC_CMD_FW_VER	= 0x0002,
	FG_MAC_CMD_HW_VER	= 0x0003,
	FG_MAC_CMD_IF_SIG	= 0x0004,
	FG_MAC_CMD_CHEM_ID	= 0x0006,
	FG_MAC_CMD_GAUGING	= 0x0021,
	FG_MAC_CMD_SEAL		= 0x0030,
	FG_MAC_CMD_DEV_RESET	= 0x0041,
	FG_MAC_CMD_ENTER_ROM	= 0x0F00,
};


enum {
	SEAL_STATE_RSVED,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
	SEAL_STATE_FA,
};


enum mmi_fg_device {
	NFG1000,
};

static const unsigned char *device2str[] = {
	"nfg1000",
};

struct mmi_fg_chip {
	struct device *dev;
	struct i2c_client *client;

	struct workqueue_struct *fg_workqueue;
	struct delayed_work battery_delay_work;
	struct power_supply *batt_psy;
	struct iio_channel *Batt_NTC_channel;
	struct iio_channel *vref_channel;
	struct fg_temp *ntc_temp_table;

	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex update_lock;

	bool resume_completed;

	u8 chip;
	u8 regs[NUM_REGS];
	/* status tracking */

	bool batt_fc;
	bool batt_fd;

	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */

	int seal_state;
	int batt_tte;
	int batt_soc;
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr;

	int batt_cyclecnt;	/* cycle count */	
	int force_update;	

	/* debug */
	int skip_reads;
	int skip_writes;

	bool fake_battery;
	int fake_soc;
	int fake_temp;
	int rbat_pull_up_r;

	struct power_supply *fg_psy;
	struct power_supply_desc fg_psy_d;
	struct moto_chg_tcmd_client batt_tcmd_client;

	int (*mmi_get_property)(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val);
	int (*mmi_set_property)(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val);
	void (*mmi_fg_update_thread)(struct work_struct *work);

};

#if 0
static int __fg_read_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		mmi_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8)ret;

	return 0;
}

static int __fg_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mmi_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		mmi_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}
#endif

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		mmi_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}
static int __fg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;
	struct i2c_msg msg[2];
	int i;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret >= 0)
			return ret;
		else
			msleep(5);
	}
	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;
	struct i2c_msg msg;
	u8 data[64];
	char* addr;
	int i = 0;

	addr = kmalloc(sizeof(data), GFP_KERNEL);

	data[0] = reg;
	memcpy(&data[1], buf, len);
	memcpy(addr, data, sizeof(data));

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = addr;
	msg.len = len + 1;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret >= 0) {
			kfree(addr);
			return ret;
		}
		else
			msleep(5);
	}

	kfree(addr);
	return ret;
}

#if 0
static int fg_read_byte(struct mmi_fg_chip *mmi, u8 reg, u8 *val)
{
	int ret;

	if (mmi->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_read_byte(mmi->client, reg, val);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;
}


static int fg_write_byte(struct mmi_fg_chip *mmi, u8 reg, u8 val)
{
	int ret;

	if (mmi->skip_writes)
		return 0;

	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_write_byte(mmi->client, reg, val);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;
}
#endif

static int fg_read_word(struct mmi_fg_chip *mmi, u8 reg, u16 *val)
{
	int ret;

	if (mmi->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_read_word(mmi->client, reg, val);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;
}
/*
static int fg_write_word(struct mmi_fg_chip *mmi, u8 reg, u16 val)
{
	int ret;

	if (mmi->skip_writes)
		return 0;

	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_write_word(mmi->client, reg, val);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;
}
*/

static int fg_read_block(struct mmi_fg_chip *mmi, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (mmi->skip_reads)
		return 0;
	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_read_block(mmi->client, reg, buf, len);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct mmi_fg_chip *mmi, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (mmi->skip_writes)
		return 0;

	mutex_lock(&mmi->i2c_rw_lock);
	ret = __fg_write_block(mmi->client, reg, data, len);
	mutex_unlock(&mmi->i2c_rw_lock);

	return ret;
}

static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++)
		sum += data[i];
	sum &= 0xFF;

	return 0xFF - sum;
}

#if 1
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	mmi_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	mmi_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif

static int fg_mac_read_block(struct mmi_fg_chip *mmi, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 cksum_calc, cksum;
	u8 t_buf[40];
	u8 t_len;
	int i;

	t_buf[0] = (u8)(cmd >> 0) & 0xFF;
	t_buf[1] = (u8)(cmd >> 8) & 0xFF;
	ret = fg_write_block(mmi, mmi->regs[BQ_FG_REG_ALT_MAC], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(100);

	ret = fg_read_block(mmi, mmi->regs[BQ_FG_REG_ALT_MAC], t_buf, 36);
	if (ret < 0)
		return ret;

	fg_print_buf("mac_read_block", t_buf, 36);

	cksum = t_buf[34];
	t_len = t_buf[35];

	cksum_calc = checksum(t_buf, t_len - 2);
	if (cksum_calc != cksum)
		return -1;

	for (i = 0; i < len; i++)
		buf[i] = t_buf[i+2];

	return 0;
}

static int fg_read_HW_version(struct mmi_fg_chip *mmi)
{
	int ret;
	u8 buf[36]= {0};
	u16 version =0;
	ret = fg_mac_read_block(mmi, FG_MAC_CMD_HW_VER, buf, 2);
	if (ret < 0) {
		mmi_err("Failed to read hw version:%d\n", ret);
		return -1;
	}
	version =  buf[0] << 8 | buf[1];
	mmi_log("hw Ver:0x%04X\n", version);

	return version;
}



static int fg_read_status(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 flags;

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&mmi->data_lock);
	mmi->batt_fc			= !!(flags & FG_FLAGS_FC);
	mmi->batt_fd		= !!(flags & FG_FLAGS_FD);
	mmi->batt_rca		= !!(flags & FG_FLAGS_RCA);
	mmi->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&mmi->data_lock);

	return ret;
}

static int fg_read_rsoc(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 soc = 0;

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		mmi_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}
	mmi_info("RSOC = %d", soc);

	return soc;

}

struct fg_temp {
	signed int BatteryTemp;
	signed int TemperatureR;
};

struct fg_temp fg_temp_table[23] = {
		{-40, 195652},
		{-35, 148171},
		{-30, 113347},
		{-25, 87559},
		{-20, 68237},
		{-15, 53650},
		{-10, 42506},
		{-5, 33892},
		{0, 27219},
		{5, 22021},
		{10, 17926},
		{15, 14674},
		{20, 12081},
		{25, 10000},
		{30, 8315},
		{35, 6948},
		{40, 5834},
		{45, 4917},
		{50, 4161},
		{55, 3535},
		{60, 3014},
		{65, 2588},
		{70, 2227}
};
/* ============================================================ */
/* voltage to battery temperature */
/* ============================================================ */
int adc_battemp(struct mmi_fg_chip *mmi_fg, int res)
{
	int i = 0;
	int res1 = 0, res2 = 0;
	int tbatt_value = -200, tmp1 = 0, tmp2 = 0;
	struct fg_temp *ptable;

	ptable = mmi_fg->ntc_temp_table;
	if (res >= ptable[0].TemperatureR) {
		tbatt_value = -40;
	} else if (res <= ptable[22].TemperatureR) {
		tbatt_value = 70;
	} else {
		res1 = ptable[0].TemperatureR;
		tmp1 = ptable[0].BatteryTemp;

		for (i = 0; i <= 22; i++) {
			if (res >= ptable[i].TemperatureR) {
				res2 = ptable[i].TemperatureR;
				tmp2 = ptable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				res1 = ptable[i].TemperatureR;
				tmp1 = ptable[i].BatteryTemp;
			}
		}

		tbatt_value = (((res - res2) * tmp1) +
			((res1 - res) * tmp2)) / (res1 - res2);
	}
	mmi_info("[%s] %d %d %d %d %d %d\n",
		__func__,
		res1, res2, res, tmp1,
		tmp2, tbatt_value);

	return tbatt_value;
}

static int fg_read_temperature(struct mmi_fg_chip *mmi)
{
	int batt_ntc_v = 0;
	int bif_v = 0;
	int tres_temp,delta_v, batt_temp;

	iio_read_channel_processed(mmi->Batt_NTC_channel, &batt_ntc_v);
	iio_read_channel_processed(mmi->vref_channel, &bif_v);

	tres_temp = batt_ntc_v * (mmi->rbat_pull_up_r);
	delta_v = bif_v - batt_ntc_v; //1.8v -batt_ntc_v
	tres_temp = div_s64(tres_temp, delta_v);

	batt_temp = adc_battemp(mmi, tres_temp);
	batt_temp *= 10;
	mmi_info("batt_temp = %d \n",batt_temp);
/*
	int ret;
	u16 temp = 0;
	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		mmi_err("could not read temperature, ret = %d\n", ret);
		return ret;
	}
	temp -= 2730;
	mmi_info("temperature = %d",  temp);
*/
	return batt_temp;

}

static int fg_read_volt(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		mmi_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}
	mmi_info(" volt = %d", volt);

	return volt;

}

static int fg_read_current(struct mmi_fg_chip *mmi, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_CURRENT], &avg_curr);
	if (ret < 0) {
		mmi_err("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);
	mmi_info(" curr = %d", *curr);

	return ret;
}

static int fg_read_fcc(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 fcc;

	if (mmi->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		mmi_err("FCC command not supported!\n");
		return 0;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0)
		mmi_err("could not read FCC, ret=%d\n", ret);
	mmi_info(" fcc = %d", fcc);

	return fcc;
}

static int fg_read_dc(struct mmi_fg_chip *mmi)
{

	int ret;
	u16 dc;

	if (mmi->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		mmi_err("DesignCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_DC], &dc);

	if (ret < 0) {
		mmi_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	mmi_info(" DesignCapacity = %d", dc);
	return dc;
}


static int fg_read_rm(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 rm;

	if (mmi->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		mmi_err("RemainingCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_RM], &rm);

	if (ret < 0) {
		mmi_err("could not read DC, ret=%d\n", ret);
		return ret;
	}
	mmi_info(" RemainingCapacity = %d", rm);

	return rm;

}

static int fg_read_cyclecount(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 cc;

	if (mmi->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		mmi_err("Cycle Count not supported!\n");
		return -1;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		mmi_err("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	mmi_info(" Cycle Count = %d", cc);

	return cc;
}

static int fg_read_tte(struct mmi_fg_chip *mmi)
{
	int ret;
	u16 tte;

	if (mmi->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		mmi_err("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		mmi_err("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	mmi_info(" Time To Empty = %d", tte);
	return tte;
}

static int fg_get_batt_status(struct mmi_fg_chip *mmi)
{

	fg_read_status(mmi);

	if (mmi->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (mmi->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (mmi->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

}


static int fg_get_batt_capacity_level(struct mmi_fg_chip *mmi)
{

	if (mmi->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (mmi->batt_rca)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (mmi->batt_fd)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

}


static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/*POWER_SUPPLY_PROP_HEALTH,*//*implement it in battery power_supply*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};


static int fake_fg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct mmi_fg_chip *mmi = power_supply_get_drvdata(psy);

	mutex_lock(&mmi->update_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 4200* 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = 50;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 250;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = 10000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = 5000 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 5000 * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	default:
		mutex_unlock(&mmi->update_lock);
		return -EINVAL;
	}

	mutex_unlock(&mmi->update_lock);

	return 0;
}


static int fg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct mmi_fg_chip *mmi = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&mmi->update_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(mmi);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg_read_volt(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret >= 0)
			mmi->batt_volt = ret;
		val->intval = mmi->batt_volt * 1000;
		mutex_unlock(&mmi->data_lock);

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&mmi->data_lock);
		fg_read_current(mmi, &mmi->batt_curr);
		val->intval = mmi->batt_curr * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (mmi->fake_soc >= 0) {
			val->intval = mmi->fake_soc;
			break;
		}
		ret = fg_read_rsoc(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret >= 0)
			mmi->batt_soc = ret;
		val->intval = mmi->batt_soc;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(mmi);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if (mmi->fake_temp != -EINVAL) {
			val->intval = mmi->fake_temp;
			break;
		}
		ret = fg_read_temperature(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret > 0)
			mmi->batt_temp = ret;
		val->intval = mmi->batt_temp;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_read_tte(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret >= 0)
			mmi->batt_tte = ret;

		val->intval = mmi->batt_tte;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret > 0)
			mmi->batt_fcc = ret;
		val->intval = mmi->batt_fcc * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg_read_dc(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret > 0)
			mmi->batt_dc = ret;
		val->intval = mmi->batt_dc * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_read_cyclecount(mmi);
		mutex_lock(&mmi->data_lock);
		if (ret >= 0)
			mmi->batt_cyclecnt = ret;
		val->intval = mmi->batt_cyclecnt;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	default:
		mutex_unlock(&mmi->update_lock);
		return -EINVAL;
	}

	mutex_unlock(&mmi->update_lock);

	return 0;
}
static void fg_dump_registers(struct mmi_fg_chip *mmi);

static int fg_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct mmi_fg_chip *mmi = power_supply_get_drvdata(psy);

	mutex_lock(&mmi->update_lock);

	fg_dump_registers(mmi);
	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		mmi->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mmi->fake_soc = val->intval;
		power_supply_changed(mmi->fg_psy);
		break;
	default:
		mutex_unlock(&mmi->update_lock);
		return -EINVAL;
	}

	mutex_unlock(&mmi->update_lock);

	return 0;
}


static int fg_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}



static int fg_psy_register(struct mmi_fg_chip *mmi)
{
	struct power_supply_config fg_psy_cfg = {};

	mmi->fg_psy_d.name = "battery";
	mmi->fg_psy_d.type = POWER_SUPPLY_TYPE_BATTERY;
	mmi->fg_psy_d.properties = fg_props;
	mmi->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	mmi->fg_psy_d.get_property = mmi->mmi_get_property;
	mmi->fg_psy_d.set_property = mmi->mmi_set_property;
	mmi->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = mmi;
	fg_psy_cfg.num_supplicants = 0;
	mmi->fg_psy = power_supply_register(mmi->dev,
						&mmi->fg_psy_d,
						&fg_psy_cfg);
	if (IS_ERR(mmi->fg_psy)) {
		mmi_err("Failed to register fg_psy");
		return PTR_ERR(mmi->fg_psy);
	}

	return 0;
}


static void fg_psy_unregister(struct mmi_fg_chip *mmi)
{
	power_supply_unregister(mmi->fg_psy);
}

static const u8 fg_dump_regs[] = {
	0x00, 0x02, 0x04, 0x06,
	0x08, 0x0A, 0x0C, 0x0E,
	0x10, 0x16, 0x18, 0x1A,
	0x1C, 0x1E, 0x20, 0x28,
	0x2A, 0x2C, 0x2E, 0x30,
	0x66, 0x68, 0x6C, 0x6E,
	0x70,
};

static ssize_t fg_attr_show_Ra_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mmi_fg_chip *mmi = i2c_get_clientdata(client);
	u8 t_buf[40];
	u8 temp_buf[40];
	int ret;
	int i, idx, len;

	mutex_lock(&mmi->update_lock);

	ret = fg_mac_read_block(mmi, 0x40C0, t_buf, 32);
	if (ret < 0) {
		mutex_unlock(&mmi->update_lock);
		return 0;
	}

	idx = 0;
	len = sprintf(temp_buf, "Ra Flag:0x%02X\n", t_buf[0] << 8 | t_buf[1]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "RaTable:\n");
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	for (i = 1; i < 16; i++) {
		len =
		    sprintf(temp_buf, "%d ", t_buf[i*2] << 8 | t_buf[i*2 + 1]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}

	mutex_unlock(&mmi->update_lock);

	return idx;
}

static ssize_t fg_attr_show_Qmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mmi_fg_chip *mmi = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[64];
	int len;

	memset(t_buf, 0, 64);

	mutex_lock(&mmi->update_lock);

	ret = fg_mac_read_block(mmi, 0x4146, t_buf, 2);
	if (ret < 0) {
		mutex_unlock(&mmi->update_lock);
		return 0;
	}

	len =
	    sprintf(buf, "Qmax Cell 0 = %d\n", (t_buf[0] << 8) | t_buf[1]);

	mutex_unlock(&mmi->update_lock);

	return len;
}

static DEVICE_ATTR(RaTable, S_IRUGO, fg_attr_show_Ra_table, NULL);
static DEVICE_ATTR(Qmax, S_IRUGO, fg_attr_show_Qmax, NULL);
static struct attribute *fg_attributes[] = {
	&dev_attr_RaTable.attr,
	&dev_attr_Qmax.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static void fg_dump_registers(struct mmi_fg_chip *mmi)
{
	int i;
	int ret;
	u16 val;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(mmi, fg_dump_regs[i], &val);
		if (!ret)
			mmi_err("Reg[%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}
}

static void fake_fg_update_thread(struct work_struct *work)
{
	return;
}

static void fg_update_thread(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct mmi_fg_chip *mmi;

	delay_work = container_of(work, struct delayed_work, work);
	mmi = container_of(delay_work, struct mmi_fg_chip, battery_delay_work);

	/* get battery power supply */
	if (!mmi->batt_psy) {
		mmi->batt_psy = power_supply_get_by_name("battery");
		if (!mmi->batt_psy)
			mmi_log(" get batt_psy fail\n");
	}


	mutex_lock(&mmi->update_lock);
	fg_read_status(mmi);
	mutex_lock(&mmi->data_lock);

	mmi->batt_soc = fg_read_rsoc(mmi);
	mmi->batt_volt = fg_read_volt(mmi);
	fg_read_current(mmi, &mmi->batt_curr);
	mmi->batt_temp = fg_read_temperature(mmi);
	mmi->batt_rm = fg_read_rm(mmi);

	mutex_unlock(&mmi->data_lock);

	mutex_unlock(&mmi->update_lock);

	if (mmi->batt_psy) {
		power_supply_changed(mmi->batt_psy);
	}

	mmi_log("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		mmi->batt_soc, mmi->batt_volt, mmi->batt_curr, mmi->batt_temp);

	queue_delayed_work(mmi->fg_workqueue, &mmi->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));

}

static int  tcmd_get_bat_temp(void *input, int* val)
{
	int ret = 0;
	struct mmi_fg_chip *mmi_fg = (struct mmi_fg_chip *)input;

	*val = mmi_fg->batt_temp / 10;

	return ret;
}

static int  tcmd_get_bat_voltage(void *input, int* val)
{
	int ret = 0;
	struct mmi_fg_chip *mmi_fg = (struct mmi_fg_chip *)input;

	*val = mmi_fg->batt_volt * 1000;

	return ret;
}

static int battery_tcmd_register(struct mmi_fg_chip *mmi_fg)
{
	int ret = 0;

	mmi_fg->batt_tcmd_client.data = mmi_fg;
	mmi_fg->batt_tcmd_client.client_id = MOTO_CHG_TCMD_CLIENT_BAT;

	mmi_fg->batt_tcmd_client.get_bat_temp = tcmd_get_bat_temp;
	mmi_fg->batt_tcmd_client.get_bat_voltage = tcmd_get_bat_voltage;

	ret = moto_chg_tcmd_register(&mmi_fg->batt_tcmd_client);

	return ret;
}

static int mmi_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	int ret;
	struct mmi_fg_chip *mmi;
	u8 *regs;

	mmi = devm_kzalloc(&client->dev, sizeof(*mmi), GFP_KERNEL);

	if (!mmi)
		return -ENOMEM;

	mmi->dev = &client->dev;
	mmi->client = client;
	mmi->chip = id->driver_data;

	mmi->batt_soc	= -ENODATA;
	mmi->batt_fcc	= -ENODATA;
	mmi->batt_rm	= -ENODATA;
	mmi->batt_dc	= -ENODATA;
	mmi->batt_volt	= -ENODATA;
	mmi->batt_temp	= -ENODATA;
	mmi->batt_curr	= -ENODATA;
	mmi->batt_cyclecnt = -ENODATA;

	mmi->fake_soc	= -EINVAL;
	mmi->fake_temp	= -EINVAL;
	mmi->fake_battery = false;

	if (mmi->chip == NFG1000) {
		regs = nfg1000_regs;
	} else {
		mmi_err("unexpected fuel gauge: %d\n", mmi->chip);
		regs = nfg1000_regs;
	}

	memcpy(mmi->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, mmi);

	mutex_init(&mmi->i2c_rw_lock);
	mutex_init(&mmi->data_lock);
	mutex_init(&mmi->update_lock);
	mmi->resume_completed = true;


	device_init_wakeup(mmi->dev, 1);

	ret=fg_read_HW_version(mmi);
	if (ret < 0) {
		mmi->fake_battery = true;
		mmi_info("don't have real battery,use fake battery\n");
	}

	mmi->Batt_NTC_channel = devm_iio_channel_get(mmi->dev, "bat_temp");
	if (IS_ERR(mmi->Batt_NTC_channel)) {
		mmi_err( "failed to get batt_therm IIO channel\n");
		ret = PTR_ERR(mmi->Batt_NTC_channel);
	}
	mmi->vref_channel = devm_iio_channel_get(mmi->dev, "vref");
	if (IS_ERR(mmi->vref_channel)) {
		mmi_err( "failed to get vref_channel IIO channel\n");
		ret = PTR_ERR(mmi->vref_channel);
	}
	mmi->ntc_temp_table = fg_temp_table;

	ret = of_property_read_u32(mmi->client->dev.of_node , "uirbat_pull_up_r_full", &mmi->rbat_pull_up_r);
	if (ret < 0) {
		mmi->rbat_pull_up_r = 24 * 1000;
		mmi_err("Failed to get uirbat_pull_up_r_full, err:%d, use default 24Kpull_up_r\n", ret);
	}

	if(mmi->fake_battery){
		mmi->mmi_get_property = fake_fg_get_property;
		mmi->mmi_set_property = NULL;
		mmi->mmi_fg_update_thread = fake_fg_update_thread;
	} else {
		mmi->mmi_get_property = fg_get_property;
		mmi->mmi_set_property = fg_set_property;
		mmi->mmi_fg_update_thread = fg_update_thread;
	}

	ret = fg_psy_register(mmi);
	if (ret)
		mmi_err("Failed to register power_supply, err:%d\n", ret);

	ret = sysfs_create_group(&mmi->dev->kobj, &fg_attr_group);
	if (ret)
		mmi_err("Failed to register sysfs, err:%d\n", ret);

	mmi->fg_workqueue = create_singlethread_workqueue("nfg1000_gauge");
	INIT_DELAYED_WORK(&mmi->battery_delay_work, mmi->mmi_fg_update_thread);
	queue_delayed_work(mmi->fg_workqueue, &mmi->battery_delay_work , msecs_to_jiffies(queue_start_work_time));

	battery_tcmd_register(mmi);
	mmi_log("mmi fuel gauge probe successfully, %s\n", device2str[mmi->chip]);

	return 0;

}


static inline bool is_device_suspended(struct mmi_fg_chip *mmi)
{
	return !mmi->resume_completed;
}


static int mmi_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mmi_fg_chip *mmi = i2c_get_clientdata(client);

	cancel_delayed_work(&mmi->battery_delay_work);
	mmi->resume_completed = false;

	return 0;
}


static int mmi_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mmi_fg_chip *mmi = i2c_get_clientdata(client);

	mmi->resume_completed = true;
	queue_delayed_work(mmi->fg_workqueue, &mmi->battery_delay_work, msecs_to_jiffies(1));

	power_supply_changed(mmi->fg_psy);

	return 0;


}

static int mmi_fg_remove(struct i2c_client *client)
{
	struct mmi_fg_chip *mmi = i2c_get_clientdata(client);

	fg_psy_unregister(mmi);

	mutex_destroy(&mmi->data_lock);
	mutex_destroy(&mmi->i2c_rw_lock);
	mutex_destroy(&mmi->update_lock);

	sysfs_remove_group(&mmi->dev->kobj, &fg_attr_group);

	return 0;

}

static void mmi_fg_shutdown(struct i2c_client *client)
{
	pr_info("mmi fuel gauge driver shutdown!\n");
}

static const struct of_device_id mmi_fg_match_table[] = {
	{ .compatible = "HXZY,nfg1000" },
	{},
};
MODULE_DEVICE_TABLE(of, mmi_fg_match_table);

static const struct i2c_device_id mmi_fg_id[] = {
	{ "nfg1000", NFG1000 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mmi_fg_id);

static const struct dev_pm_ops mmi_fg_pm_ops = {
	.resume		= mmi_fg_resume,
	.suspend	= mmi_fg_suspend,
};

static struct i2c_driver mmi_fg_driver = {
	.driver	= {
		.name   = "nfg1000_fg",
		.owner  = THIS_MODULE,
		.of_match_table = mmi_fg_match_table,
		.pm     = &mmi_fg_pm_ops,
	},
	.id_table       = mmi_fg_id,

	.probe          = mmi_fg_probe,
	.remove		= mmi_fg_remove,
	.shutdown	= mmi_fg_shutdown,

};

module_i2c_driver(mmi_fg_driver);

MODULE_DESCRIPTION("HXZY NFG1000 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

