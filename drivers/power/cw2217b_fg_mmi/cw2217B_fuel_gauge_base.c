#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sizes.h>
#include <linux/regulator/consumer.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/iio/consumer.h>
#include <linux/mmi_gauge_class.h>

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C

#define GPIO_ENABLE_MIN_TEMP    0
#define GPIO_ENABLE_MAX_TEMP    0
#define GPIO_ENABLE_SOC_CHANGE  0
#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */

#define CWFG_NAME               "cw2217"
#define SIZE_OF_PROFILE         80
#define USER_RSENSE             10 /* mhom rsense */

#define CW_SLEEP_20MS           20
#define CW_SLEEP_10MS           10
#define COMPLEMENT_CODE_U16     0x8000
#define CW_SLEEP_100MS          100
#define CW_SLEEP_200MS          200
#define CW_SLEEP_COUNTS         50
#define CW_TRUE                 1
#define CW_RETRY_COUNT          3

#define CW2217_NOT_ACTIVE          1
#define CW2217_PROFILE_NOT_READY   2
#define CW2217_PROFILE_NEED_UPDATE 3

#define CW_BPD_TEMP (-400)

#define cw_err(fg, fmt, ...)			\
	do {						\
		pr_err("FG_CW2217-%s error: %s: " fmt, fg->battName,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define cw_info(fg, fmt, ...)			\
	do {						\
		pr_info("FG_CW2217-%s: %s: " fmt, fg->battName,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define cw_printk(fmt, arg...)  \
	printk("FG_CW2217 : %s-%d : " fmt, __FUNCTION__ ,__LINE__,##arg)
/*
static unsigned char config_profile_info[SIZE_OF_PROFILE] = {
	0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xA6,0xB1,0xB2,0xCE,0xC1,0xC1,0x9E,0x5F,
	0x36,0xFF,0xFF,0xF9,0xBA,0x92,0x7F,0x6B,
	0x60,0x5D,0x58,0x94,0xE7,0xDE,0x9A,0xDC,
	0xC2,0xCA,0xD2,0xD7,0xD6,0xD3,0xD1,0xCC,
	0xCA,0xC9,0xCE,0xB8,0x9C,0x92,0x88,0x81,
	0x75,0x71,0x7F,0x91,0xAB,0x83,0x71,0x70,
	0x20,0x00,0x57,0x10,0x02,0xB0,0x5D,0x00,
	0x00,0x00,0x64,0x26,0xD3,0x51,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x94,
};
*/
struct cw_battery {
	struct i2c_client *client;
	struct device *dev;
	struct gauge_device *gauge_dev;
	struct regulator *vdd_i2c_vreg;
	struct power_supply *cw_bat_psy;
	unsigned char config_profile_info[SIZE_OF_PROFILE];
	struct iio_channel *Batt_NTC_channel;
	struct iio_channel *vref_channel;
	struct fg_temp *ntc_temp_table;
	bool has_ext_ntc;
	int  rbat_pull_up_r;
	int  chip_id;
	int  voltage_now;
	int  ic_soc_h;
	int  ic_soc_l;
	int  raw_soc;
	int  temp;
	int  current_now;
	int  cycle;
	int  soh;
	int  charge_counter;
	int  fw_version;
	int  fcc_design;
	int  fcc;
	int  batt_status;
	bool present;
	bool factory_mode;
	int  sense_r_mohm;
	const char *battName;
};

/* CW2217 iic read function */
static int cw_read(struct cw_battery *cw_bat, unsigned char reg, unsigned char buf[])
{
	int ret;
	struct i2c_client *client = cw_bat->client;

	ret = i2c_smbus_read_i2c_block_data(client, reg, 1, buf);
	if (ret < 0)
		cw_err(cw_bat, "IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic write function */
static int cw_write(struct cw_battery *cw_bat, unsigned char reg, unsigned char const buf[])
{
	int ret;
	struct i2c_client *client = cw_bat->client;

	ret = i2c_smbus_write_i2c_block_data(client, reg, 1, &buf[0] );
	if (ret < 0)
		cw_err(cw_bat, "IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic read word function */
static int cw_read_word(struct cw_battery *cw_bat, unsigned char reg, unsigned char buf[])
{
	int ret;
	unsigned char reg_val[2] = { 0, 0 };
	unsigned int temp_val_buff;
	unsigned int temp_val_second;
	struct i2c_client *client = cw_bat->client;

	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, reg_val );
	if (ret < 0)
		cw_err(cw_bat, "IIC error %d\n", ret);
	temp_val_buff = (reg_val[0] << 8) + reg_val[1];

	msleep(1);
	ret = i2c_smbus_read_i2c_block_data(client, reg, 2, reg_val );
	if (ret < 0)
		cw_err(cw_bat, "IIC error %d\n", ret);
	temp_val_second = (reg_val[0] << 8) + reg_val[1];

	if (temp_val_buff != temp_val_second) {
		msleep(1);
		ret = i2c_smbus_read_i2c_block_data( client, reg, 2, reg_val );
		if (ret < 0)
			cw_err(cw_bat, "IIC error %d\n", ret);
		temp_val_buff = (reg_val[0] << 8) + reg_val[1];
	}

	buf[0] = reg_val[0];
	buf[1] = reg_val[1];

	return ret;
}

/* CW2217 iic write profile function */
static int cw_write_profile(struct cw_battery *cw_bat, unsigned char const buf[])
{
	int ret;
	int i;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_write(cw_bat, REG_BAT_PROFILE + i, &buf[i]);
		if (ret < 0) {
			cw_info(cw_bat, "IIC error %d\n", ret);
			return ret;
		}
	}

	return ret;
}

/*
 * CW2217 Active function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_active(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	cw_info(cw_bat, "%s\n", __func__);

	ret = cw_write(cw_bat, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_ACTIVE;
	ret = cw_write(cw_bat, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return 0;
}

/*
 * CW2217 Sleep function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_sleep(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	cw_info(cw_bat, "%s\n", __func__);

	ret = cw_write(cw_bat, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_SLEEP;
	ret = cw_write(cw_bat, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return 0;
}

/*
 * The 0x00 register is an UNSIGNED 8bit read-only register. Its value is fixed to 0xA0 in shutdown
 * mode and active mode.
 */
static int cw_get_chip_id(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int chip_id;

	ret = cw_read(cw_bat, REG_CHIP_ID, &reg_val);
	if (ret < 0)
		return ret;

	chip_id = reg_val;  /* This value must be 0xA0! */
	cw_bat->chip_id = chip_id;
	cw_info(cw_bat, "chip_id=%d", chip_id);

	return 0;
}

/*
 * The VCELL register(0x02 0x03) is an UNSIGNED 14bit read-only register that updates the battery voltage continuously.
 * Battery voltage is measured between the VCELL pin and VSS pin, which is the ground reference. A 14bit
 * sigma-delta A/D converter is used and the voltage resolution is 312.5uV. (0.3125mV is *5/16)
 */
static int cw_get_voltage(struct gauge_device *gauge_dev, int *mV)
{
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	unsigned int voltage;
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);

	ret = cw_read_word(cw_bat, REG_VCELL_H, reg_val);
	if (ret < 0) {
		cw_info(cw_bat, "error\n");
		return ret;
	}
	voltage = (reg_val[0] << 8) + reg_val[1];
	voltage = voltage  * 5 / 16;
	cw_bat->voltage_now= voltage;
	*mV = voltage;
	cw_info(cw_bat, "voltage =%d mV\n", *mV);

	return 0;
}

/*
 * The SOC register(0x04 0x05) is an UNSIGNED 16bit read-only register that indicates the SOC of the battery. The
 * SOC shows in % format, which means how much percent of the battery's total available capacity is
 * remaining in the battery now. The SOC can intrinsically adjust itself to cater to the change of battery status,
 * including load, temperature and aging etc.
 * The high byte(0x04) contains the SOC in 1% unit which can be directly used if this resolution is good
 * enough for the application. The low byte(0x05) provides more accurate fractional part of the SOC and its
 * LSB is (1/256) %.
 */
static int cw_get_capacity(struct gauge_device *gauge_dev, int *soc)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int ret;
	unsigned char reg_val[2] = { 0, 0 };

	ret = cw_read_word(cw_bat, REG_SOC_INT, reg_val);
	if (ret < 0)
		return ret;
	cw_bat->ic_soc_h = reg_val[0];
	cw_bat->ic_soc_l = reg_val[1];
	cw_bat->raw_soc = cw_bat->ic_soc_h;

	 //if ((soc_h * 256 + soc_l) * 100 /256) > 70)
	//	cw_bat->raw_soc ++;

	*soc = cw_bat->raw_soc;
	 cw_info(cw_bat, "soc=%d\n", *soc);

	return 0;
}

static bool is_atm_mode(void)
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
int adc_battemp(struct cw_battery *cw_bat, int res)
{
	int i = 0;
	int res1 = 0, res2 = 0;
	int tbatt_value = -200, tmp1 = 0, tmp2 = 0;
	struct fg_temp *ptable;

	ptable = cw_bat->ntc_temp_table;
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
	 cw_info(cw_bat,"[%s] %d %d %d %d %d %d\n",
		__func__,
		res1, res2, res, tmp1,
		tmp2, tbatt_value);

	return tbatt_value;
}
/*
 * The TEMP register is an UNSIGNED 8bit read only register.
 * It reports the real-time battery temperature
 * measured at TS pin. The scope is from -40 to 87.5 degrees Celsius,
 * LSB is 0.5 degree Celsius. TEMP(C) = - 40 + Value(0x06 Reg) / 2
 */
static int cw_get_temp(struct gauge_device *gauge_dev, int *temp_out)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int ret;
	unsigned char reg_val;
	int batt_ntc_v = 0;
	int bif_v = 0;
	int tres_temp,delta_v, batt_temp;;

	if (cw_bat->has_ext_ntc) {
		iio_read_channel_processed(cw_bat->Batt_NTC_channel, &batt_ntc_v);
		iio_read_channel_processed(cw_bat->vref_channel, &bif_v);

		tres_temp = batt_ntc_v * (cw_bat->rbat_pull_up_r);
		delta_v = bif_v - batt_ntc_v; //1.8v -batt_ntc_v
		tres_temp = div_s64(tres_temp, delta_v);

		batt_temp = adc_battemp(cw_bat, tres_temp) * 10;
		cw_info(cw_bat,"read batt temperature from PMIC,temp = %d \n",batt_temp);
	} else {
		ret = cw_read(cw_bat, REG_TEMP, &reg_val);
		if (ret < 0)
			return ret;

		batt_temp = (int)reg_val * 10 / 2 - 400;

		if(cw_bat->factory_mode && (-400 == batt_temp))
			batt_temp = 250;
		cw_info(cw_bat, "battery pack NTC temprature=%d\n", batt_temp);
	}

	cw_bat->temp = batt_temp;
	*temp_out = batt_temp;

	return 0;
}

/* get complement code function, unsigned short must be U16 */
static long get_complement_code(unsigned short raw_code)
{
	long complement_code;
	int dir;

	if (0 != (raw_code & COMPLEMENT_CODE_U16)){
		dir = -1;
		raw_code =  (~raw_code) + 1;
	} else {
		dir = 1;
	}
	complement_code = (long)raw_code * dir;

	return complement_code;
}

/*
 * CURRENT is a SIGNED 16bit register(0x0E 0x0F) that reports current A/D converter result of the voltage across the
 * current sense resistor, 10mohm typical. The result is stored as a two's complement value to show positive
 * and negative current. Voltages outside the minimum and maximum register values are reported as the
 * minimum or maximum value.
 * The register value should be divided by the sense resistance to convert to amperes. The value of the
 * sense resistor determines the resolution and the full-scale range of the current readings. The LSB of 0x0F
 * is (52.4/32768)uV.
 * The default value is 0x0000, stands for 0mA. 0x7FFF stands for the maximum charging current and 0x8001 stands for
 * the maximum discharging current.
 */
static int cw_get_current(struct gauge_device *gauge_dev, int *mA)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	int cw_current;
	unsigned short current_reg;  /* unsigned short must u16 */

	ret = cw_read_word(cw_bat, REG_CURRENT_H, reg_val);
	if (ret < 0)
		return ret;

	current_reg = (reg_val[0] << 8) + reg_val[1];
	cw_current = get_complement_code(current_reg);
	cw_bat->current_now = cw_current  * 160 / cw_bat->sense_r_mohm / 100;
	*mA = cw_bat->current_now;
	cw_info(cw_bat, "current=%dmA\n", *mA);

	return 0;
}

/*
 * CYCLECNT is an UNSIGNED 16bit register(0xA4 0xA5) that counts cycle life of the battery. The LSB of 0xA5 stands
 * for 1/16 cycle. This register will be clear after enters shutdown mode
 */
static int cw_get_cycle_count(struct gauge_device *gauge_dev, int *cycle_count)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int ret;
	unsigned char reg_val[2] = {0, 0};
	int cycle;

	ret = cw_read_word(cw_bat, REG_CYCLE_H, reg_val);
	if (ret < 0)
		return ret;

	cycle = (reg_val[0] << 8) + reg_val[1];
	cw_bat->cycle = cycle / 16;
	*cycle_count = cw_bat->cycle;
	cw_info(cw_bat, "cycle_count=%d\n", *cycle_count);

	return 0;
}

/*
 * SOH (State of Health) is an UNSIGNED 8bit register(0xA6) that represents the level of battery aging by tracking
 * battery internal impedance increment. When the device enters active mode, this register refresh to 0x64
 * by default. Its range is 0x00 to 0x64, indicating 0 to 100%. This register will be clear after enters shutdown
 * mode.
 */
static int cw_get_soh(struct gauge_device *gauge_dev, int *soh)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int ret;
	unsigned char reg_val;

	ret = cw_read(cw_bat, REG_SOH, &reg_val);
	if (ret < 0)
		return ret;
	cw_bat->soh = reg_val;
	*soh = reg_val;
	cw_info(cw_bat, "soh=%d\n", *soh);

	return 0;
}

#if 0
static int cw_get_stb_current(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	long long stb_current;
	unsigned short stb_current_reg;  /*unsigned short must u16*/

	ret = cw_read_word(cw_bat, REG_STB_CUR_H, reg_val);
	if (ret < 0)
		return ret;

	stb_current_reg = (reg_val[0] << 8) + reg_val[1];
	stb_current = get_complement_code(STB_current_reg);
	stb_current = stb_current  * 160 / 16 / USER_RSENSE / 100;
	cw_bat->stb_current = stb_current;

	return 0;
}
#endif

static int cw_get_charge_full(struct gauge_device *gauge_dev, int *charge_full)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);

	*charge_full = (cw_bat->fcc_design * cw_bat->soh ) / 100;
	cw_bat->fcc = *charge_full;
	cw_info(cw_bat, "charge_full=%d\n", *charge_full);

	return 0;
}

static int cw_get_charge_full_design(struct gauge_device *gauge_dev, int *charge_full_design)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);

	*charge_full_design = cw_bat->fcc_design;

	return 0;
}

static int cw_get_charge_counter(struct gauge_device *gauge_dev, int *charge_counter)
{
	struct cw_battery *cw_bat = dev_get_drvdata(&gauge_dev->dev);
	int full_capacity;

	full_capacity = (cw_bat->fcc_design * cw_bat->soh) / 100;
	*charge_counter = div_s64(full_capacity * cw_bat->raw_soc, 100);
	cw_bat->charge_counter = *charge_counter;
	cw_info(cw_bat, "charge_counter=%d\n", *charge_counter);

	return 0;
}

/*
 * FW_VERSION register reports the firmware (FW) running in the chip. It is fixed to 0x00 when the chip is
 * in shutdown mode. When in active mode, Bit [7:6] are fixed to '01', which stand for the CW2217B and Bit
 * [5:0] stand for the FW version running in the chip. Note that the FW version is subject to update and contact
 * sales office for confirmation when necessary.
*/
static int cw_get_fw_version(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int fw_version;

	ret = cw_read(cw_bat, REG_FW_VERSION, &reg_val);
	if (ret < 0)
		return ret;

	fw_version = reg_val;
	cw_bat->fw_version = fw_version;

	return 0;
}

static int cw_init_data(struct gauge_device *gauge_dev)
{
	int value = 0;

	cw_get_voltage(gauge_dev,&value);
	cw_get_capacity(gauge_dev,&value);
	cw_get_temp(gauge_dev,&value);
	cw_get_current(gauge_dev,&value);
	cw_get_soh(gauge_dev, &value);
	cw_get_charge_full(gauge_dev,&value);
	cw_get_charge_counter(gauge_dev,&value);

	return 0;
}

/*CW2217 update profile function, Often called during initialization*/
static int cw_config_start_ic(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int count = 0;

	ret = cw2217_sleep(cw_bat);
	if (ret < 0)
		return ret;

	/* update new battery info */
	ret = cw_write_profile(cw_bat, cw_bat->config_profile_info);
	if (ret < 0) {
		cw_err(cw_bat, "cw_write_profile error %d\n", ret);
		return ret;
	}

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;
	ret = cw_write(cw_bat, REG_SOC_ALERT, &reg_val);
	if (ret < 0) {
		cw_err(cw_bat, "error set UPDATE_FLAG AND SOC INTTERRUP VALUE %d\n", ret);
		return ret;
	}

	/*close all interruptes*/
	reg_val = 0;
	ret = cw_write(cw_bat, REG_GPIO_CONFIG, &reg_val);
	if (ret < 0) {
		cw_err(cw_bat, "close all interruptes error %d\n", ret);
		return ret;
	}

	ret = cw2217_active(cw_bat);
	if (ret < 0) {
		cw_err(cw_bat, "cw2217_active error %d\n", ret);
		return ret;
	}

	while (CW_TRUE) {
		msleep(CW_SLEEP_100MS);
		cw_read(cw_bat, REG_IC_STATE, &reg_val);
		if (IC_READY_MARK == (reg_val & IC_READY_MARK))
			break;
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw2217_sleep(cw_bat);
			return -1;
		}
	}

	return 0;
}

/*
 * Get the cw2217 running state
 * Determine whether the profile needs to be updated
*/
static int cw2217_get_state(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int i;
	int reg_profile;

	ret = cw_read(cw_bat, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	cw_info(cw_bat, "REG_MODE_CONFIG 0x%2x = 0x%2x\n", REG_MODE_CONFIG, reg_val);
	if (reg_val != CONFIG_MODE_ACTIVE)
		return CW2217_NOT_ACTIVE;

	ret = cw_read(cw_bat, REG_SOC_ALERT, &reg_val);
	if (ret < 0)
		return ret;
	if (0x00 == (reg_val & CONFIG_UPDATE_FLG))
		return CW2217_PROFILE_NOT_READY;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read(cw_bat, (REG_BAT_PROFILE + i), &reg_val);
		if (ret < 0)
			return ret;
		reg_profile = REG_BAT_PROFILE + i;
		if (cw_bat->config_profile_info[i] != reg_val) {
			cw_info(cw_bat, "config profile mismatch, config_profile_info[%d]: 0x%2x, reg_val[0x%2x]: 0x%2x\n",
				i, cw_bat->config_profile_info[i], reg_profile, reg_val);
			break;
		}
	}
	if ( i != SIZE_OF_PROFILE)
		return CW2217_PROFILE_NEED_UPDATE;

	return 0;
}

/*CW2217 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;

	cw_info(cw_bat, "%s\n", __func__);
	ret = cw2217_get_state(cw_bat);
	if (ret < 0) {
		cw_err(cw_bat, "iic read write error");
		return ret;
	}

	if (ret != 0) {
		cw_info(cw_bat, "update profile and restart ic!\n");
		ret = cw_config_start_ic(cw_bat);
		if (ret < 0) {
			cw_err(cw_bat, "cw_config_start_ic error, %d\n", ret);
			return ret;
		}
	}
	cw_info(cw_bat, "cw2217 init success!\n");

	return 0;
}

static void cw_hw_init_work(struct cw_battery *cw_bat)
{

	int ret;
	int loop = 0;

	cw_get_fw_version(cw_bat);

	ret = cw_init(cw_bat);
	while ((loop++ < CW_RETRY_COUNT) && (ret != 0)) {
		msleep(CW_SLEEP_200MS);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		cw_info(cw_bat, "%s : cw2217 init fail!\n", __func__);
		return ;
	}

	return ;
}

static const char *cw_get_battery_serialnumber(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *battsn_buf;
	int retval;

	battsn_buf = NULL;

	if (np)
		retval = of_property_read_string(np, "mmi,battid", &battsn_buf);
	else
		return NULL;

	if ((retval == -EINVAL) || !battsn_buf) {
		cw_printk(" Battsn unused\n");
		of_node_put(np);
		return NULL;
	} else
		cw_printk("Battsn = %s\n", battsn_buf);

	of_node_put(np);

	return battsn_buf;
}

static struct device_node *cw_get_profile_by_serialnumber(
		struct cw_battery *cw_bat)
{
	struct device_node *np = cw_bat->client->dev.of_node;
	struct device_node *node, *df_node, *sn_node;
	const char *sn_buf, *df_sn, *dev_sn;
	int rc;

	if (!np)
		return NULL;

	dev_sn = NULL;
	df_sn = NULL;
	sn_buf = NULL;
	df_node = NULL;
	sn_node = NULL;

	dev_sn = cw_get_battery_serialnumber();

	rc = of_property_read_string(np, "df-serialnum",
				     &df_sn);
	if (rc)
		cw_info(cw_bat, "No Default Serial Number defined\n");
	else if (df_sn)
		cw_info(cw_bat, "Default Serial Number %s\n", df_sn);

	for_each_child_of_node(np, node) {
		rc = of_property_read_string(node, "serialnum",
					     &sn_buf);
		if (!rc && sn_buf) {
			if (dev_sn)
				if (strnstr(dev_sn, sn_buf, 32))
					sn_node = node;
			if (df_sn)
				if (strnstr(df_sn, sn_buf, 32))
					df_node = node;
		}
	}

	if (sn_node) {
		node = sn_node;
		df_node = NULL;
		cw_info(cw_bat, "Battery Match Found using %s\n", sn_node->name);
	} else if (df_node) {
		node = df_node;
		sn_node = NULL;
		cw_info(cw_bat, "Battery Match Found using default %s\n",
				df_node->name);
	} else {
		cw_info(cw_bat, "No Battery Match Found!\n");
		return NULL;
	}

	return node;
}

static int cw_parse_dts(struct cw_battery *cw_bat)
{
	struct device_node *np = cw_bat->client->dev.of_node;
	struct device_node *batt_profile_node = NULL;
	int rc;

	rc = of_property_read_string(np, "fg-psy-name", &cw_bat->battName);
	if (rc) {
		cw_bat->battName = "bms";
		rc = 0;
	}

	rc = of_property_read_u32(np, "sense_r_mohm", &cw_bat->sense_r_mohm);
	if(rc < 0)
		cw_bat->sense_r_mohm = USER_RSENSE;

	batt_profile_node = cw_get_profile_by_serialnumber(cw_bat);
	if (!batt_profile_node)
		return -1;

	rc = of_property_read_u8_array(batt_profile_node, "config_profile_info", cw_bat->config_profile_info, SIZE_OF_PROFILE);
	if (rc < 0)
		cw_info(cw_bat, "error,get profile_info fail from dts,exit \n");

	rc = of_property_read_u32(batt_profile_node, "fcc_design", &cw_bat->fcc_design);
	if (rc < 0)
		cw_info(cw_bat, "error,get fcc_design,exit \n");
	else
		cw_info(cw_bat, "get fcc_design=%d \n", cw_bat->fcc_design);

	cw_bat->has_ext_ntc = of_property_read_bool(np, "has_ext_ntc");
	if (cw_bat->has_ext_ntc) {
		rc = of_property_read_u32(np , "rbat_pull_up_r", &cw_bat->rbat_pull_up_r);
		if (rc < 0) {
			cw_bat->rbat_pull_up_r = 24 * 1000;
			cw_info(cw_bat,"Failed to get rbat_pull_up_r, err:%d, use default 24K pull_up_r\n", rc);
		}
	}

	return 0;
}

static int cw_battery_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct cw_battery *cw_bat;

	cw_bat = power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cw_bat->raw_soc;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !!cw_bat->voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = cw_bat->voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = cw_bat->charge_counter * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = cw_bat->fcc * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = cw_bat->fcc_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (cw_bat->factory_mode)
			cw_get_current(cw_bat->gauge_dev, &cw_bat->current_now);
		val->intval = cw_bat->current_now* 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = cw_bat->temp;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = cw_bat->cycle;
		break;
	default:
		break;
	}
	return 0;
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static struct gauge_ops cw_gauge_ops = {
	.get_voltage_now = cw_get_voltage,
	.get_current_now = cw_get_current,
	.get_capacity = cw_get_capacity,
	.get_temperature =cw_get_temp,
	//.get_tte = cw_get_tte,
	.get_charge_full = cw_get_charge_full,
	.get_charge_full_design = cw_get_charge_full_design,
	.get_charge_counter = cw_get_charge_counter,
	.get_cycle_count = cw_get_cycle_count,
	.get_soh = cw_get_soh,
	//.set_charge_type = cw_set_charge_type,
};

static const struct gauge_properties cw_gauge_props = {
	.alias_name = CWFG_NAME,
};
static int cw2217_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct cw_battery *cw_bat;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		cw_printk("%s : cw_bat create fail!\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cw_bat);
	cw_bat->client = client;
	cw_bat->dev = &client->dev;

	ret = cw_parse_dts(cw_bat);
	if (ret) {
		cw_err(cw_bat, "Failed to parse cw2217 properties\n");
		return ret;
	}

	cw_bat->vdd_i2c_vreg = devm_regulator_get_optional(
					&cw_bat->client->dev,
					"vdd-i2c");
	if (IS_ERR_OR_NULL(cw_bat->vdd_i2c_vreg)) {
		cw_info(cw_bat, "%s: Could not get vdd-i2c power regulator\n", __func__);
		cw_bat->vdd_i2c_vreg = NULL;
	} else {
		ret = regulator_enable(cw_bat->vdd_i2c_vreg);
		cw_info(cw_bat, "%s: Enable vdd-i2c, ret=%d\n", __func__, ret);
		ret = 0;
	}

	ret = cw_get_chip_id(cw_bat);
	if (ret < 0) {
		cw_err(cw_bat,"iic read write error");
		goto error;
	}
	if (cw_bat->chip_id != IC_VCHIP_ID){
		cw_err(cw_bat,"not cw2217B\n");
		goto error;
	}

	if (cw_bat->has_ext_ntc) {
		cw_bat->Batt_NTC_channel = devm_iio_channel_get(cw_bat->dev, "bat_temp");
		if (IS_ERR(cw_bat->Batt_NTC_channel)) {
			cw_err(cw_bat, "failed to get batt_therm IIO channel\n");
			ret = PTR_ERR(cw_bat->Batt_NTC_channel);
		}
		cw_bat->vref_channel = devm_iio_channel_get(cw_bat->dev, "vref");
		if (IS_ERR(cw_bat->vref_channel)) {
			cw_err(cw_bat, "failed to get vref_channel IIO channel\n");
			ret = PTR_ERR(cw_bat->vref_channel);
		}
		cw_bat->ntc_temp_table = fg_temp_table;
	}

	cw_hw_init_work(cw_bat);

	if(is_atm_mode())
		cw_bat->factory_mode = true;

	cw_bat->gauge_dev= gauge_device_register(cw_bat->battName,
					      &client->dev, cw_bat,
					      &cw_gauge_ops,
					      &cw_gauge_props);
	if (IS_ERR_OR_NULL(cw_bat->gauge_dev)) {
		ret = PTR_ERR(cw_bat->gauge_dev);
		cw_err(cw_bat, "failed to register battery: %d\n", ret);
		goto error;
	}

	cw_init_data(cw_bat->gauge_dev);

	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc) {
		ret = -ENOMEM;
		goto error;
	}
	psy_cfg.drv_data = cw_bat;
	psy_desc->name = cw_bat->battName;
	psy_desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;
	cw_bat->cw_bat_psy = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if (IS_ERR_OR_NULL(cw_bat->cw_bat_psy)) {
		ret = PTR_ERR(cw_bat->cw_bat_psy);
		cw_err(cw_bat, "failed to register battery: %d\n", ret);
		goto error;
	}

	cw_info(cw_bat, "cw2217 driver probe success!\n");

	return 0;

error:
	if (cw_bat->vdd_i2c_vreg) {
		if (regulator_is_enabled(cw_bat->vdd_i2c_vreg))
			regulator_disable(cw_bat->vdd_i2c_vreg);
		devm_regulator_put(cw_bat->vdd_i2c_vreg);
	}
	return ret;
}

static void cw2217_remove(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	if (cw_bat->vdd_i2c_vreg) {
		if (regulator_is_enabled(cw_bat->vdd_i2c_vreg))
			regulator_disable(cw_bat->vdd_i2c_vreg);
		devm_regulator_put(cw_bat->vdd_i2c_vreg);
	}
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct cw_battery *cw_bat = i2c_get_clientdata(client);
	// ??
	return 0;
}

static int cw_bat_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct cw_battery *cw_bat = i2c_get_clientdata(client);
	// ??
	return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};
#endif

enum cw2217_compatible_id {
	CW2217_STANDALONE,
	CW2217_MASTER,
	CW2217_SLAVE,
};

static const struct i2c_device_id cw2217_id_table[] = {
	{ "cw2217-standalone", CW2217_STANDALONE },
	{ "cw2217-master", CW2217_MASTER },
	{ "cw2217-slave", CW2217_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, cw2217_id_table);

static const struct of_device_id cw2217_match_table[] = {
	{ .compatible = "cellwise,cw2217-standalone", .data = (void *)CW2217_STANDALONE},
	{ .compatible = "cellwise,cw2217-master", .data = (void *)CW2217_MASTER},
	{ .compatible = "cellwise,cw2217-slave", .data = (void *)CW2217_SLAVE},
	{ },
};
MODULE_DEVICE_TABLE(of, cw2217_match_table);

static struct i2c_driver cw2217_driver = {
	.driver   = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
		.pm = &cw_bat_pm_ops,
#endif
		.owner = THIS_MODULE,
		.of_match_table = cw2217_match_table,
	},
	.probe = cw2217_probe,
	.remove = cw2217_remove,
	.id_table = cw2217_id_table,
};

static int __init cw2217_init(void)
{
	cw_printk("cw2217_init \n");
	i2c_add_driver(&cw2217_driver);
	return 0;
}

static void __exit cw2217_exit(void)
{
	i2c_del_driver(&cw2217_driver);
}

module_init(cw2217_init);
module_exit(cw2217_exit);

MODULE_AUTHOR("Cellwise FAE");
MODULE_DESCRIPTION("CW2217 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL");
