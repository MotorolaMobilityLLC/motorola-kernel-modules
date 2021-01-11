#ifndef _AW99703_REG_H_
#define _AW99703_REG_H_

/*********************************************************
 *
 * kernel version
 *
 ********************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#define KERNEL_ABOVE_4_14
#endif

/********************************************
 * Register List
 *******************************************/
#define AW99703_REG_MAX  0x70
#define AW99703_REG_ID				0x00
#define AW99703_REG_SFTRST			0x01
#define AW99703_REG_MODE			0x02
#define AW99703_REG_LEDCUR			0x03
#define AW99703_REG_BSTCTR1			0x04
#define AW99703_REG_BSTCTR2			0x05
#define AW99703_REG_LEDLSB			0x06
#define AW99703_REG_LEDMSB			0x07
#define AW99703_REG_PWM				0x08
#define AW99703_REG_TURNCFG			0x09
#define AW99703_REG_TRANCFG			0x0a
#define AW99703_REG_FLASH			0x0b
#define AW99703_REG_AFHIGH			0x0c
#define AW99703_REG_AFLOW			0x0d
#define AW99703_REG_FLAGS1			0x0e
#define AW99703_REG_FLAGS2			0x0f
#define AW99703_REG_FLAGS3			0x11
#define AW99703_REG_AUTOZERO			0x21
#define AW99703_REG_EMI				0x22
#define AW99703_REG_BSTCTR3			0x23
#define AW99703_REG_BSTCTR4			0x24
#define AW99703_REG_BSTCTR5			0x25
#define AW99703_REG_LEDCFG			0x26
#define AW99703_REG_DITHER			0x27
#define AW99703_REG_PWMMSB			0x28
#define AW99703_REG_PWMLSB			0x29
#define AW99703_REG_TEST			0x31
#define AW99703_REG_FLTDIS			0x33
#define AW99703_REG_EFUSE1			0x40
#define AW99703_REG_EFUSE2			0x41
#define AW99703_REG_EFUSE3			0x42
#define AW99703_REG_EFUSE4			0x43
#define AW99703_REG_EFUSE5			0x44
#define AW99703_REG_EFUSE6			0x45
#define AW99703_REG_EFUSE7			0x46
#define AW99703_REG_EFUSE8			0x47
#define AW99703_REG_EFRUN			0x48
#define AW99703_REG_EFMODE			0x49
#define AW99703_REG_WPRT1			0x67
#define AW99703_REG_WPRT2			0x68
#define AW99703_REG_SCANEN			0x69


/********************************************
 * Register Access
 *******************************************/
#define REG_NONE_ACCESS		0
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)

const unsigned char aw99703_reg_access[AW99703_REG_MAX] = {
	[AW99703_REG_ID] = REG_RD_ACCESS,
	[AW99703_REG_SFTRST] = REG_WR_ACCESS,
	[AW99703_REG_MODE] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_LEDCUR] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_BSTCTR1] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_BSTCTR2] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_LEDLSB] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_LEDMSB] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_PWM] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_TURNCFG] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_TRANCFG] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_FLASH] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_AFHIGH] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_AFLOW] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW99703_REG_FLAGS1] = REG_RD_ACCESS,
	[AW99703_REG_FLAGS2] = REG_RD_ACCESS,
	[AW99703_REG_FLAGS3] = REG_RD_ACCESS,
	[AW99703_REG_AUTOZERO] = REG_RD_ACCESS,
	[AW99703_REG_EMI] = REG_RD_ACCESS,
	[AW99703_REG_BSTCTR3] = REG_RD_ACCESS,
	[AW99703_REG_BSTCTR4] = REG_RD_ACCESS,
	[AW99703_REG_BSTCTR5] = REG_RD_ACCESS,
	[AW99703_REG_LEDCFG] = REG_RD_ACCESS,
	[AW99703_REG_DITHER] = REG_RD_ACCESS,
	[AW99703_REG_PWMMSB] = REG_RD_ACCESS,
	[AW99703_REG_PWMLSB] = REG_RD_ACCESS,
	[AW99703_REG_TEST] = REG_RD_ACCESS,
	[AW99703_REG_FLTDIS] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE1] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE2] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE3] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE4] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE5] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE6] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE7] = REG_RD_ACCESS,
	[AW99703_REG_EFUSE8] = REG_RD_ACCESS,
	[AW99703_REG_EFRUN] = REG_RD_ACCESS,
	[AW99703_REG_EFMODE] = REG_RD_ACCESS,
	[AW99703_REG_WPRT1] = REG_RD_ACCESS,
	[AW99703_REG_WPRT2] = REG_RD_ACCESS,
	[AW99703_REG_SCANEN] = REG_RD_ACCESS,
};

#define MAX_BRIGHTNESS			(2047)
/******************************************************
* Register Detail
*****************************************************/
/*SFTRST:0x01*/
#define AW99703_SFTRST_MASK			(~(1<<0))
#define AW99703_SFTRST_NOT_RST			(0<<0)
#define AW99703_SFTRST_RST			(1<<0)

/*MODE:0x02*/
#define AW99703_MODE_PDIS_MASK			(~(1<<4))
#define AW99703_MODE_PDIS_ENABLE		(0<<4)
#define AW99703_MODE_PDIS_DISABLE		(1<<4)
#define AW99703_MODE_MAP_MASK			(~(1<<2))
#define AW99703_MODE_MAP_EXPONENTIAL		(0<<2)
#define AW99703_MODE_MAP_LINEAR			(1<<2)
#define AW99703_MODE_WORKMODE_MASK		(~(3<<0))
#define AW99703_MODE_WORKMODE_STANDBY		(0<<0)
#define AW99703_MODE_WORKMODE_BACKLIGHT		(1<<0)
#define AW99703_MODE_WORKMODE_FLASH		(2<<0)

/*MODE:0x03*/
#define AW99703_LEDCUR_BLFS_MASK		(~(31<<3))
#define AW99703_LEDCUR_CHANNEL_MASK		(~(7<<0))
#define AW99703_LEDCUR_CH3_ENABLE		(1<<2)
#define AW99703_LEDCUR_CH3_DISABLE		(0<<2)
#define AW99703_LEDCUR_CH2_ENABLE		(1<<1)
#define AW99703_LEDCUR_CH2_DISABLE		(0<<1)
#define AW99703_LEDCUR_CH1_ENABLE		(1<<0)
#define AW99703_LEDCUR_CH1_DISABLE		(0<<0)

/*BSTCTR1:0x04*/
#define AW99703_BSTCTR1_SF_SFT_MASK		(~(3<<6))
#define AW99703_BSTCTR1_SF_SFT_UP20		(1<<6)
#define AW99703_BSTCTR1_SF_SFT_DOWN12		(2<<6)
#define AW99703_BSTCTR1_SF_SFT_DOWN24		(3<<6)
#define AW99703_BSTCTR1_SF_MASK			(~(1<<5))
#define AW99703_BSTCTR1_SF_500KHZ		(0<<5)
#define AW99703_BSTCTR1_SF_1000KHZ		(1<<5)
#define AW99703_BSTCTR1_OVPSEL_MASK		(~(7<<2))
#define AW99703_BSTCTR1_OVPSEL_17V		(0<<2)
#define AW99703_BSTCTR1_OVPSEL_24V		(1<<2)
#define AW99703_BSTCTR1_OVPSEL_31V		(2<<2)
#define AW99703_BSTCTR1_OVPSEL_38V		(3<<2)
#define AW99703_BSTCTR1_OVPSEL_41P5V		(4<<2)
#define AW99703_BSTCTR1_OCPSEL_MASK		(~(3<<0))
#define AW99703_BSTCTR1_OCPSEL_0P7A		(0<<0)
#define AW99703_BSTCTR1_OCPSEL_1P6A		(1<<0)
#define AW99703_BSTCTR1_OCPSEL_2P46A		(2<<0)
#define AW99703_BSTCTR1_OCPSEL_3P3A		(3<<0)

/*BSTCTR2:0x05*/
#define AW99703_BSTCTR2_AFEN_MASK		(~(1<<7))
#define AW99703_BSTCTR2_AFEN_ENABLE		(1<<7)
#define AW99703_BSTCTR2_AFEN_DISABLE		(0<<7)
#define AW99703_BSTCTR2_IDCTSEL_MASK		(~(1<<6))
#define AW99703_BSTCTR2_IDCTSEL_4P7UH		(0<<6)
#define AW99703_BSTCTR2_IDCTSEL_10UH		(1<<6)
#define AW99703_BSTCTR2_EMISEL_MASK		(~(7<<3))
#define AW99703_BSTCTR2_EMISEL_TYPICAL		(0<<3)
#define AW99703_BSTCTR2_EMISEL_SLOW1		(1<<3)
#define AW99703_BSTCTR2_EMISEL_SLOW2		(2<<3)
#define AW99703_BSTCTR2_EMISEL_SLOW3		(3<<3)
#define AW99703_BSTCTR2_EMISEL_FAST1		(4<<3)
#define AW99703_BSTCTR2_EMISEL_FAST2		(5<<3)
#define AW99703_BSTCTR2_EMISEL_FAST3		(6<<3)
#define AW99703_BSTCTR2_EMISEL_FAST4		(7<<3)
#define AW99703_BSTCTR2_ADEN_MASK		(~(1<<2))
#define AW99703_BSTCTR2_ADEN_ENABLE		(1<<2)
#define AW99703_BSTCTR2_ADEN_DISABLE		(0<<2)

/*PWM:0x08*/
#define AW99703_PWM_P_SF_MASK			(~(3<<6))
#define AW99703_PWM_P_SF_800KHZ			(0<<6)
#define AW99703_PWM_P_SF_4MKHZ			(1<<6)
#define AW99703_PWM_P_SF_24MKHZ			(2<<6)
#define AW99703_PWM_P_ACT_MASK			(~(1<<5))
#define AW99703_PWM_P_ACT_HIGH			(0<<5)
#define AW99703_PWM_P_ACT_LOW			(1<<5)
#define AW99703_PWM_P_HYS_MASK			(~(7<<2))
#define AW99703_PWM_P_HYS_NONE			(0<<2)
#define AW99703_PWM_P_HYS_1LSB			(1<<2)
#define AW99703_PWM_P_HYS_2LSB			(2<<2)
#define AW99703_PWM_P_HYS_3LSB			(3<<2)
#define AW99703_PWM_P_HYS_4LSB			(4<<2)
#define AW99703_PWM_P_HYS_5LSB			(5<<2)
#define AW99703_PWM_P_HYS_6LSB			(6<<2)
#define AW99703_PWM_P_HYS_7LSB			(7<<2)
#define AW99703_PWM_P_FLT_MASK			(~(3<<0))
#define AW99703_PWM_P_FLT_NONE			(0<<0)
#define AW99703_PWM_P_FLT_100MS			(1<<0)
#define AW99703_PWM_P_FLT_150MS			(2<<0)
#define AW99703_PWM_P_FLT_200MS			(3<<0)

/*TURNCFG:0x09*/
#define AW99703_TURNCFG_ON_TIM_MASK		(~(15<<4))
#define AW99703_TURNCFG_OFF_TIM_MASK		(~(15<<0))

/*TRANCFG:0x0A*/
#define AW99703_TRANCFG_PWM_TIM_MASK		(~(7<<4))
#define AW99703_TRANCFG_I2C_TIM_MASK		(~(15<<0))

#define MAX_BRIGHTNESS		(2047)
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

struct aw99703_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct device dev;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex lock;
	struct work_struct work;
	enum led_brightness brightness;
	bool enable;
	unsigned char pwm_cfg;
	unsigned char full_scale_current;
	bool brt_code_enable;
	unsigned short *brt_code_table;
	int hwen_gpio;
	unsigned int  pwm_mode;
	bool using_lsb;
	bool bl_reconfig_enable;
	char panel_info[16];
	unsigned int bl_slow_reg;
	unsigned int pwm_period;
	unsigned int full_scale_led;
	unsigned int ramp_on_time;
	unsigned int ramp_off_time;
	unsigned int pwm_trans_dim;
	unsigned int i2c_trans_dim;
	unsigned int channel;
	unsigned int ovp_level;
	unsigned int frequency;
	unsigned int default_brightness;
	unsigned int max_brightness;
	unsigned int induct_current;
	unsigned int flash_current;
	unsigned int flash_timeout;
	unsigned int bl_map;
	struct backlight_device *bl_dev;
};

#endif
