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
#include <linux/mmi_gauge_class.h>


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

#define I2C_MAX_BUFFER_SIZE			4096    //ONE PAGE
#define I2C_NO_REG_DATA		0XFF
#define I2C_BLOCK_MAX_BUFFER_SIZE		36	    //ONE BLOCK
#define NFG1000_FULL_KEY       0x05060708
#define NFG1000_UNSEAL_KEY     0x01020304
#define NFG1000_CHIP_NAME      0x0661019A
#define NFG1000_RESET_CMD	0x8a
#define PROGRAM_ERROR_UNSEAL		0x01
#define PROGRAM_ERROR_ENTER_BOOT	0x02
#define PROGRAM_ERROR_SHA256		0x04
#define PROGRAM_ERROR_CHIP_NAME		0x08
#define PROGRAM_ERROR_APP_ERASE		0x09
#define PROGRAM_ERROR_APP_WRITE		0x0A
#define PROGRAM_ERROR_APP_CHECKCRC	0x0C
#define PROGRAM_ERROR_DATA_ERASE	0x0F
#define PROGRAM_ERROR_DATA_WRITE	0x10
#define PROGRAM_ERROR_DATA_CHECKCRC	0x11
#define PROGRAM_ERROR_EXIT_BOOT		0x12
#define PROGRAM_ERROR_SEAL		0x14
#define ERROR_CODE_I2C_WRITE		0xe0
#define ERROR_CODE_I2C_READ		0xe1
#define ERROR_CODE_CHECKSUM		0xe2
#define ERROR_CODE_RERUNCODE		0xe3
#define ERROR_CODE_MCUCODE		0xe4
#define ERROR_CODE_UPDATEFILELEN	0xe5
#define ERROR_CODE_OPENFILE		0xe6
#define APP_SIZE					49152

#define NFG1000_RESET_WAIT_TIME 100 //(100ms)
#define NFG1000_write_WAIT_TIME 25
#define NFG1000_com_WAIT_TIME 5
#define NFG1000_erase_WAIT_TIME 1000
#define NFG1000_boot_WAIT_TIME 2000
#define NFG1000_seal_WAIT_TIME 1200
#define NFG1000_SUCESS_CODE  0x79

#define NAKE_DWORD_8BITS(HH,HL,LH,LL) ((u32)(HH)<<24)|((u32)(HL)<<16)|((u32)(LH)<<8)|((u32)(LL))
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
	struct work_struct  fg_upgrade_work;
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
	int rbat_pull_up_r;

	int unseal_key;
	int mcu_auth_code;
	const char *battsn_buf;
	u32 batt_param_version;
	u8 *fw_version;
	u8 *fw_data;
	u8 *params_data;
	u32 fw_start_addr;
	u32 param_start_addr;
	bool do_upgrading;

	struct power_supply *fg_psy;
	struct power_supply_desc fg_psy_d;
	struct gauge_device	*gauge_dev;
	const char *gauge_dev_name;

	int (*mmi_get_property)(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val);
	int (*mmi_set_property)(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val);
	void (*mmi_fg_update_thread)(struct work_struct *work);

};

extern int mmi_batt_health_check(void);
#ifdef CONFIG_MOTO_REMOVE_MTK_GAUGE
extern int mmi_charger_update_batt_status(void);
#endif
static int fg_get_capacity(struct gauge_device *gauge_dev, int *soc);

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
	u8 msg_len;

	if(reg == I2C_NO_REG_DATA)
	{
		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_RD;
		msg[0].buf = buf;
		msg[0].len = len;
		msg_len = 1;
	}
	else
	{
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].buf = &reg;
		msg[0].len = 1;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = len;
		msg_len = 2;
	}
	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, msg, msg_len);
		if (ret >= 0)
			return ret;
		else
			msleep(5);
	}
	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
	int ret;
	struct i2c_msg msg;
	u8 data[I2C_MAX_BUFFER_SIZE];
	char* addr;
	int i = 0;

	addr = kmalloc(sizeof(data), GFP_KERNEL);
	if(!buf || (len>=I2C_MAX_BUFFER_SIZE))
	{
		mmi_err("nfg1000_i2c_write:condition is error!!\n");
		return -1;
	}

	if (reg == I2C_NO_REG_DATA) {
		memcpy(data, buf, len);
		msg.len = len;
	} else {
		data[0] = reg;
		memcpy(&data[1], buf, len);
		msg.len = len + 1;
	}
	memcpy(addr, data, sizeof(data));

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = addr;

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

static int fg_write_block(struct mmi_fg_chip *mmi, u8 reg, u8 *data, u16 len)
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

/*--------------------------------------upgrade begin------------------------------------------------------*/

u8 code_static_DF_sig[2] = {0};
u8 sha_256_ramdom_data[64] = {0x20, 0x31, 0x0C, 0xFF, 0x52, 0xF5, 0x06, 0x2E, 0x1D, 0x74,0x73, 0xDD, 0x75, 0xAD, 0x16, 0x6F, 0x09, 0x61, 0x5B, 0xD2,0xE2, 0xA6, 0xE8, 0x62, 0x31, 0x5F, 0x17, 0xC9, 0xDF, 0x52,0x45, 0xE6, 0xBA};
u8 sha_256_pass_word[64] = {0xD0, 0x81, 0xCA, 0x11, 0xBF, 0xF1, 0xFA, 0xA9, 0xA6, 0xF8,0x2A, 0xD5, 0x6C, 0x94, 0x47, 0x00, 0x93, 0x75, 0xE8, 0xE6,0xC2, 0x97, 0x8B, 0x7D, 0x17, 0x44, 0xC7, 0x0E, 0x99, 0xB9,0xC0, 0x67, 0xDF};
u8 app_flash_erase_order[195] = {0x00,0x5F,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x08,0x00,0x09,0x00,0x0A,0x00,0x0B,0x00,0x0C,0x00,0x0D,0x00,0x0E,0x00,0x0F,0x00,
								0x10,0x00,0x11,0x00,0x12,0x00,0x13,0x00,0x14,0x00,0x15,0x00,0x16,0x00,0x17,0x00,0x18,0x00,0x19,0x00,0x1A,0x00,0x1B,0x00,0x1C,0x00,0x1D,0x00,0x1E,0x00,0x1F,0x00,0x20,0x00,0x21,
								0x00,0x22,0x00,0x23,0x00,0x24,0x00,0x25,0x00,0x26,0x00,0x27,0x00,0x28,0x00,0x29,0x00,0x2A,0x00,0x2B,0x00,0x2C,0x00,0x2D,0x00,0x2E,0x00,0x2F,0x00,0x30,0x00,0x31,0x00,0x32,0x00,
								0x33,0x00,0x34,0x00,0x35,0x00,0x36,0x00,0x37,0x00,0x38,0x00,0x39,0x00,0x3A,0x00,0x3B,0x00,0x3C,0x00,0x3D,0x00,0x3E,0x00,0x3F,0x00,0x40,0x00,0x41,0x00,0x42,0x00,0x43,0x00,0x44,
								0x00,0x45,0x00,0x46,0x00,0x47,0x00,0x48,0x00,0x49,0x00,0x4A,0x00,0x4B,0x00,0x4C,0x00,0x4D,0x00,0x4E,0x00,0x4F,0x00,0x50,0x00,0x51,0x00,0x52,0x00,0x53,0x00,0x54,0x00,0x55,0x00,
								0x56,0x00,0x57,0x00,0x58,0x00,0x59,0x00,0x5A,0x00,0x5B,0x00,0x5C,0x00,0x5D,0x00,0x5E,0x00,0x5F,0x5F};
u8 data_flash_erase_order[] = {0x00,0x07,0x01,0x86,0x01,0x87,0x01,0x88,0x01,0x89,0x01,0x8A,0x01,0x8B,0x01,0x8C,0x01,0x8D,0x07};
u8 nfg1000_Dataflash_updata_CMD[] = {0x00,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0D,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1C};
static u32 crc_table[256];

static s32 nfg1000_GetReturnCode(struct mmi_fg_chip *di)
{
	int ret = 0;u8 uReCode[2] = {0};

	ret = fg_read_block(di,I2C_NO_REG_DATA,uReCode,1);
	if(ret < 0)
	{
		mmi_err(":read reg: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_READ;
	}

	if(uReCode[0] != NFG1000_SUCESS_CODE)
	{
		mmi_err(":return code error:%x!!\n",uReCode[0]);
		return -ERROR_CODE_RERUNCODE;
	}

	return 0;
}

static void init_crc_table(void)
{
	u32 c;
	u32 i, j;

	for (i = 0; i < 256; i++) {
		c = i;
		for (j = 0; j < 8; j++) {
			if (c & 1)
				c = 0xedb88320L ^ (c >> 1);
			else
				c = c >> 1;
		}
		crc_table[i] = c;
	}
}

static u32 crc32(u32 crc,u8 *buffer, u32 size)
{
	u32 i;

	for (i = 0; i < size; i++)
	{
		crc = crc_table[(crc ^ buffer[i]) & 0xff] ^ (crc >> 8);
	}
	crc = crc ^ 0xFFFFFFFF;
	return crc ;
}

static int nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(struct mmi_fg_chip *dev, u16 reg, u8 *poutbuf, u32 len)
{
	u8 reg_data[2];
	u8 block_data[36];
	int ret;

	reg_data[0] = (reg >> 0) & 0xFF;
	reg_data[1] = (reg >> 8) & 0xFF;

	if(!poutbuf) {
		mmi_err(":poutbuf is NULL!!\n");
		return -1;
	}

	ret = fg_write_block(dev, dev->regs[BQ_FG_REG_ALT_MAC], reg_data, 2);
	if(ret < 0) {
		mmi_err(":write reg:%d failed!!\n",reg);
		return -1;
	}
	ret = fg_read_block(dev, dev->regs[BQ_FG_REG_ALT_MAC], block_data, 36);
	if(ret < 0) {
		mmi_err(":read reg:%d failed!!\n",reg);
		return -1;
	}
	//fg_print_buf("command_read_with_CHECKSUM",block_data, 36);
	if((reg_data[0] != block_data[0]) || (reg_data[1] != block_data[1]) ||
		(block_data[34] != checksum(block_data,34)) || (block_data[35] != len+4)) {
		mmi_err(":command:%d failed!!\n",checksum(block_data,34));
		return -2;
	}
	memcpy(poutbuf,&block_data[2],len);
	msleep(NFG1000_com_WAIT_TIME);

	return 0;
}

static int nfg1000_i2c_BLOCK_command_write_with_CHECKSUM(struct mmi_fg_chip *dev,u16 reg,u8 *pinbuf,u32 len)
{
	u8 databuf[36] = {0};
	int ret;

	databuf[0] = (reg >> 0) & 0xFF;
	databuf[1] = (reg >> 8) & 0xFF;

	if(!pinbuf || (len>=I2C_BLOCK_MAX_BUFFER_SIZE))
	{
		printk("nfg1000_i2c_BLOCK_command_write_with_CHECKSUM:condition is error!!\n");
		return -1;
	}

	if(len)
		memcpy(&databuf[2],pinbuf,len);
	databuf[34] = checksum(&databuf[0],34);
	databuf[35] = len+4;

	ret = fg_write_block(dev,dev->regs[BQ_FG_REG_ALT_MAC],databuf,36);
	if(ret < 0)
	{
		printk("nfg1000_i2c_BLOCK_command_write_with_CHECKSUM:write reg:%d failed!!\n",reg);
		return -1;
	}
	msleep(NFG1000_com_WAIT_TIME);

	return 0;
}

static int nfg1000_ota_unseal(struct mmi_fg_chip *di)
{
	int ret;int i=0;
	u8 u8pwd[2] = {0};
	u8 seal_state_read[32] = {0};
	u16 seal_state_cmd = 0x0054;

	if (di->unseal_key == 0) {
		mmi_err("nfg1000_battery_unseal:unseal failed due to missing key\n");
		return -ERROR_CODE_CHECKSUM;
	}

	for(i = 0;i < 3;i++)
	{	mmi_info("unseal_key 0x01020304");
		//unseal_key:0x01020304
		u8pwd[0] = ((NFG1000_UNSEAL_KEY>>24))&0xff;
		u8pwd[1] = (NFG1000_UNSEAL_KEY>>16)&0xff;
		ret = fg_write_block(di,di->regs[BQ_FG_REG_ALT_MAC],u8pwd,2);
		if(ret < 0) {
			mmi_err("nfg1000_ota_program_unseal_state_check: step1 unseal write error!%x\n", ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		msleep(NFG1000_write_WAIT_TIME);

		u8pwd[0] = ((NFG1000_UNSEAL_KEY>>8))&0xff;
		u8pwd[1] = ((NFG1000_UNSEAL_KEY>>0))&0xff;
		ret = fg_write_block(di,di->regs[BQ_FG_REG_ALT_MAC],u8pwd,2);
		if(ret < 0 ) {
			mmi_err("nfg1000_ota_program_unseal_state_check:step2 unseal write error!%x\n", ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		msleep(NFG1000_RESET_WAIT_TIME);

		mmi_info("unseal_key 0x05060708");
		u8pwd[0] = ((NFG1000_FULL_KEY>>24))&0xff;
		u8pwd[1] = ((NFG1000_FULL_KEY>>16))&0xff;
		ret = fg_write_block(di,di->regs[BQ_FG_REG_ALT_MAC],u8pwd,2);
		if(ret < 0) {
			mmi_err("nfg1000_ota_program_unseal_state_check:step3 unseal write error!%x\n", ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		msleep(NFG1000_write_WAIT_TIME);

		u8pwd[0] = ((NFG1000_FULL_KEY>>8))&0xff;
		u8pwd[1] = ((NFG1000_FULL_KEY>>0))&0xff;
		ret = fg_write_block(di,di->regs[BQ_FG_REG_ALT_MAC],u8pwd,2);
		if(ret < 0) {
			mmi_err("nfg1000_ota_program_unseal_state_check:step4 unseal write error!%x\n", ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		msleep(NFG1000_write_WAIT_TIME);

		ret = nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di, seal_state_cmd, seal_state_read, 4);
		if(ret) {
			mmi_err("nfg1000_ota_program_unseal_state_check:step5 unseal state read error!%x\n", ret);
			msleep(NFG1000_boot_WAIT_TIME);
			return -ERROR_CODE_I2C_WRITE;
		}
		if((seal_state_read[1] & 0x03) == 0x01) {
			break;
		}
	}

	if(i == 3) {
		mmi_err("nfg1000_ota_program_unseal_state_check:step6 unseal opeartion error!%x\n", ret);
		return -ERROR_CODE_CHECKSUM;
	}

	return 0;
}

//firmware_version_check
static bool nfg1000_ota_program_check_fw_upgrade(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 i = 0;
	u8 fw_ver_read[12] = {0};
	bool upgrade_status = false;

	ret = nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di, FG_MAC_CMD_FW_VER, fw_ver_read, 12);
	if(ret) {
		mmi_err(": From fg ic read the  firmware version error!\n");
		return upgrade_status;
	}

	mmi_info(":Read From fuelgauge, Firmware version=[%02x %02x %02x %02x %02x]\n",
		fw_ver_read[0], fw_ver_read[1],fw_ver_read[2],fw_ver_read[3],fw_ver_read[4]);
	for(i = 0; i < 5; i++) {
		if(fw_ver_read[i] != di->fw_version[i]) {
			mmi_info(":firmware version is not same!into upgrade...\n");
			upgrade_status = true;
			break;
		}
	}

	return upgrade_status;
}

static const char *get_battery_serialnumber(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *battsn_buf;
	int retval;

	battsn_buf = NULL;

	if (np)
		retval = of_property_read_string(np, "mmi,battid",
						 &battsn_buf);
	else
		return NULL;

	if ((retval == -EINVAL) || !battsn_buf) {
		mmi_info(" Battsn unused\n");
		of_node_put(np);
		return NULL;

	} else
		mmi_info("Battsn = %s\n", battsn_buf);

	of_node_put(np);

	return battsn_buf;
}

//battery parameter version check
static bool nfg1000_ota_program_check_batt_params_version(struct mmi_fg_chip *di)
{
	bool upgrade_status = false;
	u8 dataflash_ver_read[32] = {0};
	u16 fg_param_version = 0xFFFF;
	const char *dev_sn = NULL;
	/*
	int ret = 0;
	u8 i = 0;
	ret = nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di,0x440D ,dataflash_ver_read,32);
	if(ret)
	{
		printk("nfg1000_ota_program_dataflash_version_check:dataflash version read error!\n");
		return -ERROR_CODE_I2C_WRITE;
	}
	for(i = 0;i < 10;i++)
	{
		if(dataflash_ver_read[i] != di->battsn_buf[i])
		{
			break;
		}
	}
	if(i != 10)
	{
		mmi_info("nfg1000_ota_program_dataflash_version_check: battery version different!\n");
		return -PROGRAM_BATTERY_VERSION;
	}
	*/
	if(nfg1000_ota_unseal(di))
	{
		return PROGRAM_ERROR_UNSEAL;
	}

	if (nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di,0x440B ,dataflash_ver_read,32) < 0)
		return upgrade_status;

	fg_param_version = ((dataflash_ver_read[25] - 0x30) << 8);
	fg_param_version |= (dataflash_ver_read[26] - 0x30);

	mmi_info(":the fg_param_version=0x%04x\n", fg_param_version);
	mmi_info(":latest battery parameter version=0x%04x\n", di->batt_param_version);

	if(fg_param_version < di->batt_param_version)
	{
		dev_sn = get_battery_serialnumber();
		if (dev_sn != NULL && di->battsn_buf != NULL) {
			if (strnstr(dev_sn, di->battsn_buf, 10)) {
				mmi_info(":battsn compared,the battery parameter data need upgrade!\n");
				upgrade_status = true;
			}
		}
	}

	return upgrade_status;
}

//enter bootload
static int nfg1000_ota_program_step1_EnterBootLoad(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8Data[2] = {0};
	u8 uReCode[2] = {0};
	u8 retry_cnt = 0;

	u8Data[1] = 0;
	u8Data[0] = NFG1000_RESET_CMD;
	//reset nfg1000
	mmi_info("reset nfg1000");
	ret = fg_write_block(di, di->regs[BQ_FG_REG_ALT_MAC], u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",di->regs[BQ_FG_REG_ALT_MAC]);
		return -ERROR_CODE_I2C_WRITE;
	}
	msleep(NFG1000_RESET_WAIT_TIME);

	u8Data[0] = 0x3f;
	for(retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data,1);
		ret = fg_read_block(di, I2C_NO_REG_DATA, uReCode,1);
		if(uReCode[0] == NFG1000_SUCESS_CODE)
			break;
	}
	if(uReCode[0] != NFG1000_SUCESS_CODE)
	{
		msleep(NFG1000_boot_WAIT_TIME);
		for(retry_cnt = 0; retry_cnt < 3; retry_cnt++)
		{
			ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 1);
			ret = fg_read_block(di, I2C_NO_REG_DATA, uReCode, 1);
			if(uReCode[0] == NFG1000_SUCESS_CODE)
				break;
		}
	}
	if(uReCode[0] != NFG1000_SUCESS_CODE) {
		mmi_err(":return code error:%x!!\n",uReCode[0]);
		return -ERROR_CODE_RERUNCODE;
	}

	return 0;
}

u8 CalcXorsum(u8 *inbuf,int len)
{
	int i;
	u8 xorsum = 0;

	for(i=0;i<len;i++)
	{
		xorsum ^=inbuf[i];
	}

	return xorsum;
}

#define SHA_DATA_SIZE	32

static int nfg1000_ota_program_step2_ShaAuth(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8Data[64] = {0};

	u8Data[0] = 0XE6;
	u8Data[1] = 0X19;
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	memcpy(&u8Data[0], sha_256_ramdom_data, SHA_DATA_SIZE);
	u8Data[SHA_DATA_SIZE] = CalcXorsum(sha_256_ramdom_data, SHA_DATA_SIZE);
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, SHA_DATA_SIZE+1);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}
	msleep(NFG1000_RESET_WAIT_TIME);

	memcpy(&u8Data[0], sha_256_pass_word, SHA_DATA_SIZE);
	u8Data[SHA_DATA_SIZE] = CalcXorsum(sha_256_pass_word, SHA_DATA_SIZE);
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, SHA_DATA_SIZE+1);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	msleep(NFG1000_RESET_WAIT_TIME);

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	return 0;
}

#define MCU_AUTH_CODE_SIZE		4
static int nfg1000_ota_program_step3_mcuAuth(struct mmi_fg_chip *di)
{
	int ret = 0;
	u32 tmpMcuCode =0;
	u8 u8Data[8] = {0};
	u8 uReCode[8] = {0};

	u8Data[0] = 0x91;
	u8Data[1] = 0x6e;
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	u8Data[0] = 0;
	u8Data[1] = 2;
	u8Data[2] = 0xb;
	u8Data[3] = 0xf8;
	//u8Data[4] = u8Data[0]^ u8Data[1]^u8Data[2]^ u8Data[3];
	u8Data[4] = CalcXorsum(u8Data,4);
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	memset(u8Data, 0, sizeof(u8Data));
	u8Data[0] = 0x3;
	u8Data[1] = (u8)0xff - ((u8)MCU_AUTH_CODE_SIZE - 1);//03 fc
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	ret = fg_read_block(di, I2C_NO_REG_DATA, uReCode, 4);
	if(ret < 0) {
		mmi_err(":read reg: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_READ;
	}

	tmpMcuCode = NAKE_DWORD_8BITS(uReCode[3], uReCode[2], uReCode[1], uReCode[0]);
	if(di->mcu_auth_code != tmpMcuCode) {
		mmi_err(":mcu code check error: file_code:%x  tmpMcuCode:%d !\n", di->mcu_auth_code, tmpMcuCode);
		return -ERROR_CODE_MCUCODE;
	}

	return 0;
}

static int nfg1000_ota_program_step4_EraseFlash(struct mmi_fg_chip *di)
{
	int ret = 0;
	int datalen = 0;
	u8 u8Data[200] = {0};

	u8Data[0] = 0xc4;u8Data[1] = 0x3b;
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	memcpy(u8Data, app_flash_erase_order, sizeof(app_flash_erase_order));
	datalen = sizeof(app_flash_erase_order);
	if(datalen <= 0) {
		return datalen;
	}

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, datalen);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	msleep(NFG1000_erase_WAIT_TIME);

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	return 0;
}

static int nfg1000_ota_program_step5_WriteUpdatefile(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8checksum = 0;
	u8 u8Data[260] = {0};
	s32 i = 0;
	s32 j = 0;
	s32 k = 0;
	u32 tmpaddr = 0;

	for(i=256; i < APP_SIZE; i+=256)
	{
		u8Data[0] = 0xb1;
		u8Data[1] = 0x4e;
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
		if(ret < 0) {
			mmi_err(":Error write 0xb14e reg loop=%d, error: %d!!\n", i, ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		ret = nfg1000_GetReturnCode(di);
		if(ret) {
			mmi_err(":Error write 0xb14e GetReturnCode error, loop=%d, error: %d!!\n", i, ret);
			return ret;
		}

		tmpaddr = di->fw_start_addr+i;
		u8Data[0] = (tmpaddr>>24)&0xff;
		u8Data[1] = (tmpaddr>>16)&0xff;
		u8Data[2] = (tmpaddr>>8)&0xff;
		u8Data[3] = (tmpaddr)&0xff;
		u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
		if(ret < 0) {
			mmi_err(":Error write fw start addr reg loop=%d,error: %d!!\n", i, ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		ret = nfg1000_GetReturnCode(di);
		if(ret) {
			mmi_err(":Error write fw start addr GetReturnCode loop=%d,error: %d!!\n", i, ret);
			return ret;
		}

		u8checksum = 0;
		u8Data[0] = 0xff;
		u8checksum ^= u8Data[0];
		for(j=0; j<256; j++)
		{
			u8checksum ^= di->fw_data[j+i];
			u8Data[1+j] = di->fw_data[j+i];
		}
		u8Data[1+j] = u8checksum;
		for (k=1; k<=3; k++) {
			ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 258);
			if(ret < 0) {
				mmi_err(":Error write fw data loop=%d,error: %d!!\n", i, ret);
				return -ERROR_CODE_I2C_WRITE;
			}
			msleep(120);
			ret = nfg1000_GetReturnCode(di);
			if(ret) {
				mmi_err(":Error write fw data GetReturnCode loop=%d,error: %d!!\n", i, ret);
				if (k == 3)
					return ret;
			} else {
				break; //write fw data pass
			}
		}
	}

	for(i=252; i>=0; i=i-4)
	{
		u8Data[0] = 0xb1;
		u8Data[1] = 0x4e;
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
		if(ret < 0) {
			mmi_err(":Error before 252bye,write 0xb14e reg loop=%d, error: %d!!\n", i, ret);
			return -ERROR_CODE_I2C_WRITE;
		}

		ret = nfg1000_GetReturnCode(di);
		if(ret) {
			mmi_err(":Error before 252bye,write 0xb14e GetReturnCode error, loop=%d, error: %d!!\n", i, ret);
			return ret;
		}

		tmpaddr = di->fw_start_addr+i;
		u8Data[0] = (tmpaddr>>24)&0xff;
		u8Data[1] = (tmpaddr>>16)&0xff;
		u8Data[2] = (tmpaddr>>8)&0xff;
		u8Data[3] = (tmpaddr)&0xff;
		u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
		if(ret < 0) {
			mmi_err(":Error before 252bye,write fw start addr,loop=%d error: %d!!\n", i, ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		ret = nfg1000_GetReturnCode(di);
		if(ret) {
			mmi_err(":Error before 252bye, write fw start addr GetReturnCode loop=%d,error: %d!!\n", i, ret);
			return ret;
		}

		u8checksum = 0;
		u8Data[0] = 0x3;
		u8checksum ^= u8Data[0];
		for(j=0; j<4; j++)
		{
			u8checksum ^= di->fw_data[j+i];
			u8Data[1+j] = di->fw_data[j+i];
		}
		u8Data[1+j] = u8checksum;
		for (k=1; k<=3; k++) {
			ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data,6);
			if(ret < 0) {
				mmi_err(":Error write before 252byte data loop=%d,error: %x !!\n", i, ret);
				return -ERROR_CODE_I2C_WRITE;
			}
			msleep(NFG1000_write_WAIT_TIME);

			ret = nfg1000_GetReturnCode(di);
			if(ret) {
				mmi_err(":Error write before 252byte data GetReturnCode loop=%d,error: %d!!\n", i, ret);
				if (k == 3)
					return ret;
			} else {
				break; //write fw data pass
			}
		}
	}
	return 0;
}

static int nfg1000_ota_program_step6_CheckCrc(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8Data[8] = {0};
	u8 uReCode[8] = {0};
	u32 tmpaddr = 0;
	u32 Crc32tmp = 0;
	u32 Crc32_code = 0;

	u8Data[0] = 0xd0;
	u8Data[1] = 0x2f;
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err("nfg1000_ota_program_step6_CheckCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	tmpaddr = di->fw_start_addr;
	u8Data[0] = (tmpaddr>>24)&0xff;
	u8Data[1] = (tmpaddr>>16)&0xff;
	u8Data[2] = (tmpaddr>>8)&0xff;
	u8Data[3] = (tmpaddr)&0xff;
	u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
	if(ret < 0) {
		mmi_err("nfg1000_ota_program_step6_CheckCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	tmpaddr = di->fw_start_addr + APP_SIZE - 1;
	u8Data[0] = (tmpaddr>>24)&0xff;u8Data[1] = (tmpaddr>>16)&0xff;
	u8Data[2] = (tmpaddr>>8)&0xff;
	u8Data[3] = (tmpaddr)&0xff;
	u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
	if(ret < 0) {
		mmi_err("nfg1000_ota_program_step6_CheckCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		return ret;
	}

	msleep(NFG1000_RESET_WAIT_TIME);
	ret = fg_read_block(di, I2C_NO_REG_DATA, uReCode, 4);
	if(ret < 0) {
		mmi_err("nfg1000_ota_program_step6_CheckCrc:read reg: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_READ;
	}

	Crc32tmp = NAKE_DWORD_8BITS(uReCode[3],uReCode[2],uReCode[1],uReCode[0]);
	init_crc_table();
	Crc32_code  = crc32(0xFFFFFFFF, di->fw_data, APP_SIZE);
	if(Crc32_code != Crc32tmp) {
		mmi_err("nfg1000_ota_program_step6_CheckCrc:read reg: %x error!!\n",  Crc32_code);
		return -ERROR_CODE_I2C_READ;
	}

	return 0;
}

static int nfg1000_ota_program_step7_ExitBoot(struct mmi_fg_chip *di)
{
	int ret = 0;

	u8 u8Data[8] = {0};
	u8Data[0] = 0xA1;
	u8Data[1] = 0x5E;
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		mmi_err("nfg1000 write addr 0xA15E GetReturnCode fail");
		return ret;
	}
	u8Data[0] = 0;
	u8Data[1] = 0;
	u8Data[2] = 0;
	u8Data[3] = 0;
	u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];
	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
	if(ret < 0) {
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		mmi_err("nfg1000_GetReturnCode fail 1");
		return ret;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret) {
		mmi_err("nfg1000_GetReturnCode fail 2");
		return ret;
	}
	msleep(NFG1000_seal_WAIT_TIME);

	return 0;
}

static int nfg1000_ota_program_step8_EraseDATA(struct mmi_fg_chip *di)
{
	int ret = 0;

	u8 u8Data[20] = {0};
	u8Data[0] = 0xc4;
	u8Data[1] = 0x3b;

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0)
	{
		mmi_err(":write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret)
	{
		return ret;
	}

	memcpy(u8Data,data_flash_erase_order,sizeof(data_flash_erase_order));

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, sizeof(data_flash_erase_order));
	if(ret < 0)
	{
		mmi_err("nfg1000_ota_program_step8_EraseDATA:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	msleep(NFG1000_erase_WAIT_TIME);
	ret = nfg1000_GetReturnCode(di);
	if(ret)
	{
		mmi_err("nfg1000_GetReturnCode error");
		return ret;
	}

	return 0;
}

#define CHEMID_START_DATAFLASH	3072
#define CHEMID_STOP_DATAFLASH	4096
static int nfg1000_ota_program_step9_WriteDatafile(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8Data[260] = {0};
	u8 u8checksum = 0;
	u32 i=0;
	u32 j=0;
	u32 tmpaddr = 0;

	for(i = CHEMID_START_DATAFLASH;i < CHEMID_STOP_DATAFLASH;i += 256)
	{
		u8Data[0] = 0xb1;
		u8Data[1] = 0x4e;
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
		if(ret < 0)
		{
			mmi_err("nfg1000_ota_program_step9_WriteDatafile:write reg: %x error!!\n",I2C_NO_REG_DATA);
			return -ERROR_CODE_I2C_WRITE;
		}

		ret = nfg1000_GetReturnCode(di);
		if(ret)
		{
			mmi_err("nfg1000_GetReturnCode error");
			return ret;
		}

		tmpaddr = di->param_start_addr + i;
		u8Data[0] = (tmpaddr>>24)&0xff;
		u8Data[1] = (tmpaddr>>16)&0xff;
		u8Data[2] = (tmpaddr>>8)&0xff;
		u8Data[3] = (tmpaddr)&0xff;
		u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];

		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 5);
		if(ret < 0)
		{
			printk("nfg1000_ota_program_step9_WriteDatafile:write reg: %x error!!\n",I2C_NO_REG_DATA);
			return -ERROR_CODE_I2C_WRITE;
		}
		ret = nfg1000_GetReturnCode(di);
		if(ret)
		{
			mmi_err("nfg1000_GetReturnCode error");
			return ret;
		}

		u8checksum = 0;
		u8Data[0] = 0xff;
		u8checksum ^= u8Data[0];
		for(j=0;j<256;j++)
		{
			u8checksum ^= di->params_data[j+i];
			u8Data[1+j] = di->params_data[j+i];
		}

		u8Data[1+j] = u8checksum;
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 258);
		if(ret < 0)
		{
			mmi_err("nfg1000_ota_program_step9_WriteDatafile:write reg: %x error!!\n",I2C_NO_REG_DATA);
			return -ERROR_CODE_I2C_WRITE;
		}
		msleep(NFG1000_RESET_WAIT_TIME);
		ret = nfg1000_GetReturnCode(di);
		if(ret)
		{
			mmi_err("nfg1000_GetReturnCode error");
			return ret;
		}
	}

	return 0;
}

static int nfg1000_ota_program_step10_CheckDataCrc(struct mmi_fg_chip *di)
{
	int ret = 0;
	u8 u8Data[8] = {0};
	u8 uReCode[8] = {0};
	u32 tmpaddr = 0;
	u32 Crc32tmp = 0;
	u32 Crc32_code = 0;

	u8Data[0] = 0xd0;
	u8Data[1] = 0x2f;

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 2);
	if(ret < 0)
	{
		mmi_err("nfg1000_ota_program_step10_CheckDataCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	ret = nfg1000_GetReturnCode(di);
	if(ret)
	{
		mmi_err("nfg1000_GetReturnCode error");
		return ret;
	}

	tmpaddr = 0x30C00;
	u8Data[0] = (tmpaddr>>24)&0xff;
	u8Data[1] = (tmpaddr>>16)&0xff;
	u8Data[2] = (tmpaddr>>8)&0xff;
	u8Data[3] = (tmpaddr)&0xff;
	u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data,5);
	if(ret < 0)
	{
		mmi_err("nfg1000_ota_program_step10_CheckDataCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret)
	{
		mmi_err("nfg1000_GetReturnCode error");
		return ret;
	}


	tmpaddr = 0x03FF;
	u8Data[0] = (tmpaddr>>24)&0xff;
	u8Data[1] = (tmpaddr>>16)&0xff;
	u8Data[2] = (tmpaddr>>8)&0xff;
	u8Data[3] = (tmpaddr)&0xff;
	u8Data[4] = u8Data[0]^u8Data[1]^u8Data[2]^u8Data[3];

	ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data,5);
	if(ret < 0)
	{
		mmi_err("nfg1000_ota_program_step10_CheckDataCrc:write reg: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}
	ret = nfg1000_GetReturnCode(di);
	if(ret)
	{
		mmi_err("nfg1000_GetReturnCode error");
		return ret;
	}
	msleep(NFG1000_RESET_WAIT_TIME);

	//read crc and compare
	ret = fg_read_block(di, I2C_NO_REG_DATA,uReCode,4);
	if(ret < 0)
	{
		mmi_err("nfg1000_ota_program_step10_CheckDataCrc:read output crc: %x error!!\n", I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_READ;
	}
	Crc32tmp = NAKE_DWORD_8BITS(uReCode[3],uReCode[2],uReCode[1],uReCode[0]);

	init_crc_table();
	Crc32_code  = crc32(0xFFFFFFFF, &di->params_data[CHEMID_START_DATAFLASH], 1024);
	if(Crc32_code != Crc32tmp)
	{
		mmi_err("nfg1000_ota_program_step10_CheckDataCrc:calculate crc: %x error!!\n",  Crc32_code);
		return -ERROR_CODE_I2C_READ;
	}

	return 0;
}



// updata config information

static int nfg1000_ota_updata_config(struct mmi_fg_chip *di)
{
	u8 ret;
	u8 i = 0;
	u8 j = 0;
	u8 addr_cmd = 0;
	u8 retry_cnt = 0;
	u8 retry_flag = 0;
	u16 dataflash_base_addr = 0x4400;
	u8 config_data_read[32] = {0};

	for(i = 0; i < sizeof(nfg1000_Dataflash_updata_CMD);)
	{
		addr_cmd = nfg1000_Dataflash_updata_CMD[i];

		ret = nfg1000_i2c_BLOCK_command_write_with_CHECKSUM(di, dataflash_base_addr + addr_cmd, &di->params_data[addr_cmd*32],32);

		msleep(NFG1000_RESET_WAIT_TIME);
		if(ret)
		{
			mmi_err("nfg1000_ota_updata_config:write reg: %x error!!\n",ret);
			break;
		}
		ret = nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di, dataflash_base_addr + addr_cmd,config_data_read,32);
		if(ret)
		{
			mmi_err("nfg1000_ota_updata_config:read reg: %x error!!\n",ret);
			return -ERROR_CODE_I2C_WRITE;
		}

		for(j = 0;j < 32;j++)
		{
			if(config_data_read[j] != di->params_data[addr_cmd*32 + j])
			{
				retry_flag = 1;
				retry_cnt++;
				break;
			}
		}
		if(retry_flag == 0)
		{
			retry_cnt = 0;
			i++;
		}
		else
		{
			if(retry_cnt > 2)
			{
				mmi_err("nfg1000_ota_updata_config:config updata reg: %x error!!\n",addr_cmd);
				return -ERROR_CODE_I2C_WRITE;
			}
			retry_flag = 0;
		}
	}
	if(i != sizeof(nfg1000_Dataflash_updata_CMD))
	{
		mmi_err("nfg1000_ota_updata config: %x error!!\n",I2C_NO_REG_DATA);
		return -ERROR_CODE_I2C_WRITE;
	}

	return 0;
}


static int nfg1000_ota_seal(struct mmi_fg_chip *di)
{
	int ret;
	int i=0;
	u8 u8Data[8] = {0};
	u8 seal_state_read[32] = {0};
	u16 seal_state_cmd = 0x0054;

	for(i = 0; i < 3; i++)
	{
		u8Data[0] = 0x3E;
		u8Data[1] = 0x30;
		u8Data[2] = 0;
		ret = fg_write_block(di, I2C_NO_REG_DATA, u8Data, 3);
		if(ret < 0) {
			mmi_err("nfg1000_ota_seal:write reg: %x error!!\n",I2C_NO_REG_DATA);
			return -ERROR_CODE_I2C_WRITE;
		}
		ret = nfg1000_i2c_BLOCK_command_read_with_CHECKSUM(di, seal_state_cmd, seal_state_read, 4);
		if(ret) {
			mmi_err("nfg1000_ota_program_seal_state_check:seal state read error!%x\n", ret);
			return -ERROR_CODE_I2C_WRITE;
		}
		if((seal_state_read[1] & 0x03) == 0x03)
			break;
	}
	if(i == 3) {
		mmi_err("nfg1000_ota_program_seal_state_check:seal opeartion error!%x\n", ret);
		return -ERROR_CODE_I2C_WRITE;
	}

	return 0;
}

void nfg1000_ota_init(struct mmi_fg_chip *di)
{
	di->unseal_key = NFG1000_UNSEAL_KEY;
	di->mcu_auth_code = NFG1000_CHIP_NAME;
	di->fw_start_addr = 0;
	di->param_start_addr = 0x30000;

	return;
}

static u8 *nfg1000_upgrade_read_firmware(char *bin_name, struct mmi_fg_chip *mmi_fg)
{
	const struct firmware *fw;
	int ret;
	u8 *des_buf = NULL;

	ret = request_firmware(&fw, bin_name, mmi_fg->dev);
	if (ret || fw->size <= 0 ) {
		mmi_info("Couldn't get nfg1000_firmware  rc=%d\n", ret);
		return NULL;
	}
	des_buf = kzalloc(fw->size, GFP_KERNEL);
	memset(des_buf, 0, fw->size);
	memcpy(des_buf, fw->data, fw->size);
	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}

	return des_buf;
}

static ssize_t nfg1000_upgrade_Params(struct mmi_fg_chip *di)
{
	if (di->params_data == NULL)
		di->params_data = nfg1000_upgrade_read_firmware("NFG1000A_battery_parameter.bin", di);
	if (di->params_data == NULL) {
		mmi_err("battery paramter data is null, exit upgrade.");
		return PROGRAM_ERROR_EXIT_BOOT;
	}

	mmi_info("step1 EnterBootLoad");
	if(nfg1000_ota_program_step1_EnterBootLoad(di))
	{
		return PROGRAM_ERROR_ENTER_BOOT;
	}
	mmi_info("step2 ShaAuth");
	if(nfg1000_ota_program_step2_ShaAuth(di))
	{
		return PROGRAM_ERROR_SHA256;
	}
	mmi_info("step3 mcuAuth");
	if(nfg1000_ota_program_step3_mcuAuth(di))
	{
		return PROGRAM_ERROR_CHIP_NAME;
	}
	mmi_info("step8_EraseDATA");
	if(nfg1000_ota_program_step8_EraseDATA(di))
	{
		return PROGRAM_ERROR_DATA_ERASE;
	}
	mmi_info("step9_WriteDatafile");
	if(nfg1000_ota_program_step9_WriteDatafile(di))
	{
		return PROGRAM_ERROR_DATA_WRITE;
	}
	mmi_info("step10_CheckDataCrc");
	if(nfg1000_ota_program_step10_CheckDataCrc(di))
	{
		return PROGRAM_ERROR_APP_CHECKCRC;
	}
	mmi_info("step7_ExitBoot");
	if(nfg1000_ota_program_step7_ExitBoot(di))
	{
		return PROGRAM_ERROR_EXIT_BOOT;
	}
	mmi_info("ota_unseal");
	if(nfg1000_ota_unseal(di))
	{
		return PROGRAM_ERROR_UNSEAL;
	}
	mmi_info("ota_updata_config");
	if(nfg1000_ota_updata_config(di))
	{
		return PROGRAM_ERROR_UNSEAL;
	}
	mmi_info("ota_seal");
	if(nfg1000_ota_seal(di))
	{
		return PROGRAM_ERROR_SEAL;
	}

	return 0;
}

static ssize_t nfg1000_upgrade_APP(struct mmi_fg_chip *di)
{
	mmi_info("step1 EnterBootLoad");
	if(nfg1000_ota_program_step1_EnterBootLoad(di))
	{
		return PROGRAM_ERROR_ENTER_BOOT;
	}
	mmi_info("step2 ShaAuth");
	if(nfg1000_ota_program_step2_ShaAuth(di))
	{
		return PROGRAM_ERROR_SHA256;
	}
	mmi_info("step3 mcuAuth");
	if(nfg1000_ota_program_step3_mcuAuth(di))
	{
		return PROGRAM_ERROR_CHIP_NAME;
	}
	mmi_info("step4 EraseFlash");
	if(nfg1000_ota_program_step4_EraseFlash(di))
	{
		return PROGRAM_ERROR_APP_ERASE;
	}
	mmi_info("step5 WriteUpdatefile");
	if(nfg1000_ota_program_step5_WriteUpdatefile(di))
	{
		mmi_info("step5 WriteUpdatefile failed");
		return PROGRAM_ERROR_APP_WRITE;
	}
	mmi_info("step6 CheckCrc");
	if(nfg1000_ota_program_step6_CheckCrc(di))
	{
		return PROGRAM_ERROR_APP_CHECKCRC;
	}
	mmi_info("step7 ExitBoot");
	if(nfg1000_ota_program_step7_ExitBoot(di))
	{
		return PROGRAM_ERROR_EXIT_BOOT;
	}
	mmi_info("ota seal");
	if(nfg1000_ota_seal(di))
	{
		return PROGRAM_ERROR_SEAL;
	}

	return 0;
}

static bool is_atm_mode()
{
	const char *bootargs_ptr = NULL;
	char *bootargs_str = NULL;
	char *idx = NULL;
	char *kvpair = NULL;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_ptr_len = 0;
	char *value = NULL;
	bool factory_mode = false;

	if (n == NULL)
		goto err_putnode;

	bootargs_ptr = (char *)of_get_property(n, "mmi,bootconfig", NULL);

	if (!bootargs_ptr) {
		mmi_err("%s: failed to get mmi,bootconfig\n", __func__);
		goto err_putnode;
	}

	bootargs_ptr_len = strlen(bootargs_ptr);
	if (!bootargs_str) {
		/* Following operations need a non-const version of bootargs */
		bootargs_str = kzalloc(bootargs_ptr_len + 1, GFP_KERNEL);
		if (!bootargs_str)
			goto err_putnode;
	}
	strlcpy(bootargs_str, bootargs_ptr, bootargs_ptr_len + 1);

	idx = strnstr(bootargs_str, "androidboot.atm=", strlen(bootargs_str));
	if (idx) {
		kvpair = strsep(&idx, " ");
		if (kvpair)
			if (strsep(&kvpair, "=")) {
				value = strsep(&kvpair, "\n");
			}
	}
	if (value) {
		if (!strncmp(value, "enable", strlen("enable"))) {
			factory_mode = true;
		}
	}
	kfree(bootargs_str);

err_putnode:
	if (n)
		of_node_put(n);

	return factory_mode;
}

static void nfg1000_upgrade_func(struct work_struct *work)
{
	struct mmi_fg_chip *di = container_of(work, struct mmi_fg_chip, fg_upgrade_work);
	int count = 1;

	//MMI_STOPSHIP: <fuelgauge>: remove the limits of fg upgrade function.
	if (0) {
		if (is_atm_mode() == false) {
			mmi_info("only factory-mode support fuelgauge upgrade,exit");
			return;
		}
	}

	if (di->fake_battery == true) {
		mmi_info("fake battery not support upgrade,exit");
		return;
	}

	nfg1000_ota_init(di);

	fg_get_capacity(di->gauge_dev, &di->batt_soc);
	di->fake_battery = true;
	di->do_upgrading = true;

	if (nfg1000_ota_program_check_fw_upgrade(di) == false) {
		mmi_info("fw not need upgrade,exit");
	} else {
		if (di->fw_data == NULL)
			di->fw_data = nfg1000_upgrade_read_firmware("NFG1000A_firmware.bin", di);
		if (di->fw_data == NULL) {
			mmi_err("fw data is null, exit upgrade.");
			goto upgrade_error;
		}
		mmi_info("ota unseal");
		if(nfg1000_ota_unseal(di))
		{
			mmi_err("ota unseal,failed, exit upgrade");
			goto upgrade_error;
		}
		for (count=1; count<=3; count++) {
			if (nfg1000_upgrade_APP(di) != 0) {
				mmi_err("nfg1000_upgrade_APP failed, retry=%d", count);
				if (count == 3) {
					mmi_err("nfg1000_upgrade_APP failed, use fake battery");
					goto upgrade_error;
				}
			} else {
				mmi_info("nfg1000_upgrade_APP successfully!!");
				break;
			}
		}
	}

	if (nfg1000_ota_program_check_batt_params_version(di) == false) {
		mmi_info("battery params not need upgrade,exit");
	} else {
		for (count=1; count<=3; count++) {
			if (nfg1000_upgrade_Params(di) != 0) {
				mmi_err("nfg1000_upgrade_Params failed, retry=%d", count);
				if (count == 3) {
					mmi_err("nfg1000_upgrade_Params failed, use fake battery");
					goto upgrade_error;
				}
			} else {
				mmi_info("nfg1000_upgrade_Params successfully!!");
				break;
			}
		}
	}
	di->do_upgrading = false;
	di->fake_battery = false;

upgrade_error:
	if (di->fw_data) {
		di->fw_data =NULL;
		kfree(di->fw_data);
	}
	if (di->params_data) {
		di->params_data =NULL;
		kfree(di->params_data);
	}

}

/*------------------------------------upgrade end---------------------------------------------------*/


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
	u16 fgTemp;

	iio_read_channel_processed(mmi->Batt_NTC_channel, &batt_ntc_v);
	iio_read_channel_processed(mmi->vref_channel, &bif_v);

	tres_temp = batt_ntc_v * (mmi->rbat_pull_up_r);
	delta_v = bif_v - batt_ntc_v; //1.8v -batt_ntc_v
	tres_temp = div_s64(tres_temp, delta_v);

	batt_temp = adc_battemp(mmi, tres_temp);
	batt_temp *= 10;
	mmi_info("read batt temperature from PMIC,temp = %d \n",batt_temp);

	if (batt_temp >= 700) {
		if (fg_read_word(mmi, mmi->regs[BQ_FG_REG_TEMP], &fgTemp) <0 ) {
			mmi_err("could not read temperature from FG\n");
		}
		else {
			fgTemp -= 2730;
			mmi_info("read batt temperature from FG, temp= %d",  fgTemp);
			batt_temp = fgTemp;
		}
	}

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
		return -1;
	}

	ret = fg_read_word(mmi, mmi->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0) {
		mmi_err("could not read FCC, ret=%d\n", ret);
		return ret;
	}

	mmi_info(" fcc = %d", fcc);

	return fcc;
}

static int fg_read_dc(struct mmi_fg_chip *mmi)
{

	int ret;
	u16 dc;

	if (mmi->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		mmi_err("DesignCapacity command not supported!\n");
		return -1;
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

#ifdef CONFIG_MOTO_REMOVE_MTK_GAUGE
	return mmi_charger_update_batt_status();
#else
	fg_read_status(mmi);

	if (mmi->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (mmi->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (mmi->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif

}

static int fg_get_batt_capacity_level(struct mmi_fg_chip *mmi_fg)
{
	int uisoc = mmi_fg->batt_soc;

	if (uisoc >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (uisoc >= 80 && uisoc < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (uisoc >= 20 && uisoc < 80)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (uisoc > 0 && uisoc < 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (uisoc == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

int fg_get_voltage_now(struct gauge_device *gauge_dev, int *mV)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*mV = 4200;
	else {
		ret = fg_read_volt(mmi);
		if (ret >= 0)
			*mV = ret;
	}

	return 0;
}

int fg_get_current_now(struct gauge_device *gauge_dev, int *mA)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*mA = 0;
	else {
		ret = fg_read_current(mmi, mA);
	}

	return 0;
}

int fg_get_capacity(struct gauge_device *gauge_dev, int *soc)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->do_upgrading)
		*soc = mmi->batt_soc;
	else if (mmi->fake_battery)
		*soc = 50;
	else {
		ret = fg_read_rsoc(mmi);
		if (ret >= 0)
			*soc = ret;
	}

	return 0;
}

int fg_get_temp(struct gauge_device *gauge_dev, int *temp)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);

	if (mmi->fake_battery)
		*temp = 250;
	else {
		*temp = fg_read_temperature(mmi);
	}

	return 0;
}

int fg_get_tte(struct gauge_device *gauge_dev, int *tte)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*tte = 10000;
	else {
		ret = fg_read_tte(mmi);
		if (ret >= 0)
			*tte = ret;
	}

	return 0;
}

int fg_get_charge_full(struct gauge_device *gauge_dev, int *charge_full)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*charge_full = 5000 * 1000;
	else {
		ret = fg_read_fcc(mmi);
		if (ret > 0)
			*charge_full = ret;
	}

	return 0;
}

int fg_get_charge_full_design(struct gauge_device *gauge_dev, int *charge_full_design)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*charge_full_design = 5000 * 1000;
	else {
		ret = fg_read_dc(mmi);
		if (ret > 0)
			*charge_full_design = ret;
	}

	return 0;
}

int fg_get_charge_counter(struct gauge_device *gauge_dev, int *charge_counter)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*charge_counter = 2500 * 1000;
	else {
		ret = fg_read_rm(mmi);
		if (ret > 0)
			*charge_counter = ret;
	}

	return 0;
}

int fg_get_cycle_count(struct gauge_device *gauge_dev, int *cycle_count)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);
	int ret = 0;

	if (mmi->fake_battery)
		*cycle_count = 1;
	else {
		ret = fg_read_cyclecount(mmi);
		if (ret >= 0)
			*cycle_count = ret;
	}

	return 0;
}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int fg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct mmi_fg_chip *mmi = power_supply_get_drvdata(psy);

	mutex_lock(&mmi->update_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(mmi);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&mmi->data_lock);
		fg_get_voltage_now(mmi->gauge_dev, &mmi->batt_volt);
		val->intval = mmi->batt_volt * 1000;
		mutex_unlock(&mmi->data_lock);

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&mmi->data_lock);
		fg_get_current_now(mmi->gauge_dev, &mmi->batt_curr);
		val->intval = mmi->batt_curr * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&mmi->data_lock);
		fg_get_capacity(mmi->gauge_dev, &mmi->batt_soc);
		val->intval = mmi->batt_soc;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(mmi);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&mmi->data_lock);
		fg_get_temp(mmi->gauge_dev, &mmi->batt_temp);
		val->intval = mmi->batt_temp;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		mutex_lock(&mmi->data_lock);
		fg_get_tte(mmi->gauge_dev, &mmi->batt_tte);
		val->intval = mmi->batt_tte;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		mutex_lock(&mmi->data_lock);
		fg_get_charge_full(mmi->gauge_dev, &mmi->batt_fcc);
		val->intval = mmi->batt_fcc * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		mutex_lock(&mmi->data_lock);
		fg_get_charge_full_design(mmi->gauge_dev, &mmi->batt_dc);
		val->intval = mmi->batt_dc * 1000;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		mutex_lock(&mmi->data_lock);
		fg_get_charge_counter(mmi->gauge_dev, &mmi->batt_rm);
		val->intval = mmi->batt_rm * 1000;
		mutex_unlock(&mmi->data_lock);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		mutex_lock(&mmi->data_lock);
		fg_get_cycle_count(mmi->gauge_dev, &mmi->batt_cyclecnt);
		val->intval = mmi->batt_cyclecnt;
		mutex_unlock(&mmi->data_lock);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mmi_batt_health_check();
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

static int battery_chargeType_to_FG(struct mmi_fg_chip *mmi, int ffc_enable)
{
	int ret;
	u16 cmd;

	if( ffc_enable == 1) {
		cmd = 0x003E;
	} else {
		cmd = 0x003F;
	}

	ret = fg_write_word(mmi, mmi->regs[BQ_FG_REG_ALT_MAC], cmd);
	if (ret < 0) {
		mmi_err("fail to write chargeType to fg, ret=%d\n", ret);
		return ret;
	}

	return ret;
}

int fg_set_charge_type(struct gauge_device *gauge_dev, int charge_type)
{
	struct mmi_fg_chip *mmi = dev_get_drvdata(&gauge_dev->dev);

	if (!mmi->fake_battery)
		battery_chargeType_to_FG(mmi, charge_type);

	return 0;
}

static int fg_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct mmi_fg_chip *mmi_fg = power_supply_get_drvdata(psy);

	if (mmi_fg->fake_battery)
		return 0;

	mutex_lock(&mmi_fg->update_lock);

	fg_dump_registers(mmi_fg);
	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
		battery_chargeType_to_FG(mmi_fg, val->intval);
		break;
	default:
		mutex_unlock(&mmi_fg->update_lock);
		return -EINVAL;
	}

	mutex_unlock(&mmi_fg->update_lock);

	return 0;
}


static int fg_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
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

	mmi->fg_psy_d.name = "bms";
	mmi->fg_psy_d.type = POWER_SUPPLY_TYPE_MAINS;
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

static void fg_update_thread(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct mmi_fg_chip *mmi;
	int rsoc;

	delay_work = container_of(work, struct delayed_work, work);
	mmi = container_of(delay_work, struct mmi_fg_chip, battery_delay_work);


	if (mmi->fake_battery || mmi->do_upgrading)
		return;

	/* get battery power supply */
	if (!mmi->batt_psy) {
		mmi->batt_psy = power_supply_get_by_name("battery");
		if (!mmi->batt_psy)
			mmi_log(" get batt_psy fail\n");
	}


	mutex_lock(&mmi->update_lock);
	fg_read_status(mmi);
	mutex_lock(&mmi->data_lock);

	rsoc = fg_read_rsoc(mmi);
	mmi->batt_volt = fg_read_volt(mmi);
	fg_read_current(mmi, &mmi->batt_curr);
	mmi->batt_temp = fg_read_temperature(mmi);
	mmi->batt_rm = fg_read_rm(mmi);

	mutex_unlock(&mmi->data_lock);

	mutex_unlock(&mmi->update_lock);

	if (mmi->batt_psy) {
		if (rsoc != mmi->batt_soc) {
			mmi->batt_soc = rsoc;
			power_supply_changed(mmi->batt_psy);
		}
	}

	mmi_log("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		mmi->batt_soc, mmi->batt_volt, mmi->batt_curr, mmi->batt_temp);

	queue_delayed_work(mmi->fg_workqueue, &mmi->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));

}

static const struct gauge_properties nfg1000_gauge_props = {
	.alias_name = "nfg1000",
};

static struct gauge_ops nfg1000_gauge_ops = {
	.get_voltage_now = fg_get_voltage_now,
	.get_current_now = fg_get_current_now,
	.get_capacity = fg_get_capacity,
	.get_temperature = fg_get_temp,
	.get_tte = fg_get_tte,
	.get_charge_full = fg_get_charge_full,
	.get_charge_full_design = fg_get_charge_full_design,
	.get_charge_counter = fg_get_charge_counter,
	.get_cycle_count = fg_get_cycle_count,
	.set_charge_type = fg_set_charge_type,
};

static int mmi_parse_dt(struct mmi_fg_chip *mmi_fg)
{
	struct device_node *np = mmi_fg->client->dev.of_node;
	int byte_len;
	int rc;

	rc = of_property_read_u32(np , "uirbat_pull_up_r_full", &mmi_fg->rbat_pull_up_r);
	if (rc < 0) {
		mmi_fg->rbat_pull_up_r = 24 * 1000;
		mmi_err("Failed to get uirbat_pull_up_r_full, err:%d, use default 24Kpull_up_r\n", rc);
	}

	if (of_find_property(np, "latest_fw_version", &byte_len)) {
		mmi_fg->fw_version= (u8 *)devm_kzalloc(&mmi_fg->client->dev, byte_len, GFP_KERNEL);
		if (mmi_fg->fw_version == NULL) {
			mmi_err(" devm_kzalloc fail,exit parse dts");
			return -ENOMEM;
		}
		rc = of_property_read_u8_array(np,
			"latest_fw_version", mmi_fg->fw_version, byte_len / sizeof(u8));
		if (rc < 0) {
			mmi_err("Couldn't read mmi fw version = %d\n", rc);
			return -ENOMEM;
		}
	}
	// read batt id
	rc = of_property_read_string(np, "df_serialnum", &mmi_fg->battsn_buf);
	if (rc)
		mmi_err("No Default Serial Number defined\n");
	else if (mmi_fg->battsn_buf)
		mmi_info("Default Serial Number %s\n", mmi_fg->battsn_buf);

	// read battery param version
	rc = of_property_read_u32(np , "latest_batt_param_version", &mmi_fg->batt_param_version);
	if (rc < 0) {
		mmi_fg->batt_param_version = 0;
		mmi_info("Failed to get batt_param_version, err:%d, set batt_param_versions=0\n", rc);
	}

	return 0;
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
	mmi->batt_volt	=4200;
	mmi->batt_temp	= 250;
	mmi->batt_curr	= -ENODATA;
	mmi->batt_cyclecnt = -ENODATA;

	mmi->fake_battery = false;
	mmi->do_upgrading = false;
	mmi->battsn_buf = NULL;
	mmi->fw_data = NULL;
	mmi->params_data = NULL;

	mmi_parse_dt(mmi);

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
	mmi->mmi_fg_update_thread = fg_update_thread;
	mmi->mmi_get_property = fg_get_property;
	mmi->mmi_set_property = fg_set_property;

	ret = fg_psy_register(mmi);
	if (ret)
		mmi_err("Failed to register power_supply, err:%d\n", ret);

	ret = sysfs_create_group(&mmi->dev->kobj, &fg_attr_group);
	if (ret)
		mmi_err("Failed to register sysfs, err:%d\n", ret);

	mmi->gauge_dev_name = "bms";
	mmi->gauge_dev = gauge_device_register(mmi->gauge_dev_name,
					      &client->dev, mmi,
					      &nfg1000_gauge_ops,
					      &nfg1000_gauge_props);
	if (IS_ERR_OR_NULL(mmi->gauge_dev)) {
		ret = PTR_ERR(mmi->gauge_dev);
		mmi_err("Failed to register gauge device, err:%d\n", ret);
		return ret;
	}

	mmi->fg_workqueue = create_singlethread_workqueue("nfg1000_gauge");
	INIT_DELAYED_WORK(&mmi->battery_delay_work, mmi->mmi_fg_update_thread);
	queue_delayed_work(mmi->fg_workqueue, &mmi->battery_delay_work , msecs_to_jiffies(queue_start_work_time));

	INIT_WORK(&mmi->fg_upgrade_work, nfg1000_upgrade_func);
	schedule_work(&mmi->fg_upgrade_work);

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

