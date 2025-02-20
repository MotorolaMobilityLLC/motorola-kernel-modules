/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __AW8680X_H__
#define __AW8680X_H__

#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#define AWINIC_DEBUG
#ifdef AWINIC_DEBUG
#define AWLOGD(format, ...) \
		pr_debug("[%s][%04d]%s: " format "\n", "aw8680x_sensor", __LINE__, __func__, ##__VA_ARGS__)

#define AWLOGI(format, ...) \
		pr_info("[%s][%04d]%s: " format "\n", "aw8680x_sensor", __LINE__, __func__, ##__VA_ARGS__)

#define AWLOGE(format, ...) \
		pr_err("[%s][%04d]%s: " format "\n", "aw8680x_sensor", __LINE__, __func__, ##__VA_ARGS__)


#else
#define AWLOGD(format, arg...)
#define AWLOGI(format, arg...)
#define AWLOGE(format, arg...)
#endif

/*********************************************************
 *
 * struct
 *
 ********************************************************/
#define NUM_NODES				(32)
#define AW_CHANNEL_NUM				(6)
#define AW_CHANNEL_MAX_NUM			(12)
#define SOC_APP_DATA_TYPE			0x21
#define PC_POINT_ROM_BOOT			0x00000001
#define PC_POINT_FLASH_BOOT			0x00010002
#define PC_POINT_SRAM				0x00010003

#define ERASE_BYTE_MAX				512
#define WRITE_FLASH_MAX				64
#define READ_FLASH_MAX				64
#define WRITE_SRAM_MAX				(64 * 3)

#define AW_WAKE_TIME				(1800)

#define REG_ADDR				0x01
#define SOC_ADDR				0x00000000
#define SOC_DATA_LEN				0x0000
#define SOC_READ_LEN				0x0000

#define KEY_DATA_ADDR				0x12
#define REG_SCAN_MODE_SWITCH_ADDR		0x56
#define REGVAL_LEN_ONE_byte			1
#define RESET_INIT_TIME				5
#define CHIP_INIT_TIME				15
#define JUMP_INIT_TIME				20
#define CONNECT_RETRY_TIME			3
#define READ_CHIPID_RETRY_TIME			3
#define REG_STAY_UBOOT				0x01
#define AW8680X_STAY_UBOOT			1

#define FLASH_APP_VERSION_GET_TIME		50
#define SRAM_INIT_TIME				20
#define FLASH_APP_INIT_TIME			30
#define FLASH_BOOT_INIT_TIME			10
#define GO_FLASH_APP_TIME			(50)

/* register */
#define AW_PGA_ADDR				(0x10)
#define AW_TEMPERA_ADDR				(0x11)
#define AW_ADC_DR_ADDR				(0x12)
#define AW_DAC_DR_GET_ADDR			(0x13)
#define AW_CALI_ADDR				(0x14)
#define AW_SENSOR_STATUS_ADDR			(0x15)
#define AW_ADC_VOLTAGE_ADDR			(0x16)
#define AW_DAC_VOLTAGE_ADDR			(0x17)
#define AW_DAC_DR_SET_ADDR			(0x18)
#define AW_WRITE_REG_ADDR			(0xC2)
#define AW_READ_REG_ADDR			(0xC3)
#define AW_DEBUG_CLEAR_ADDR			(0xC5)
#define AW_NOISE_ADDR				(0xC6)
#define AW_DEBUG_LEN_ADDR			(0xC7)
#define ADC_DATA_ADDR				(0xD0)
#define BASE_LINE_ADDR				(0xD1)
#define FORCE_EVENT_ADDR			(0xD2)
#define DIFF_ADDR				(0xD3)
#define DIFF_THRESHOLD_ADDR			(0xD4)
#define SENSOR_TYPE_ADDR			(0xD5)
#define SW_ALGO_VERSION_ADDR			(0xDE)
#define FTC_CALI_ADDR				(0xE0)
#define AW_WRITE_COEF_ADDR			(0xE1)
#define AW_READ_COEF_ADDR			(0xE2)
#define FORCE_DATA_ADDR				(0xE3)
#define AW_PRESSURE_ADDR			(0xAB)
#define AW_TEMPERA_DATA_LEN			(0x4)
#define AW_PGA_DATA_LEN				(0x4)
#define AW_CALI_DATA_LEN			(0x3)
#define AW_SENSOR_DATA_LEN			(0x4)
#define AW_ADC_DR_LEN				((AW_CHANNEL_NUM + 2) * 2)
#define AW_DAC_DR_LEN				(AW_CHANNEL_NUM * 2)
#define ADC_DATA_LEN				(AW_CHANNEL_MAX_NUM * 2)
#define BASELINE_LEN				(AW_CHANNEL_MAX_NUM * 2)
#define DIFF_DATA_LEN				(AW_CHANNEL_MAX_NUM * 2)
#define DIFF_THRESHOLD_DATA_LEN			(AW_CHANNEL_NUM * 2)
#define SW_ALGO_VERS_LEN			(0x6)
#define AW_ADC_VOLTAGE_LEN			(0x4)
#define AW_DAC_VOLTAGE_LEN			(0x4)
#define AW_PRESSURE_DATA_LEN		(0x1)
#define DATA_INIT				(0)


/* ndt register */
#define NDT_DEBUG_DUMP_ADDR			(0x20)
#define NDT_DEBUG_DUMP_LEN			(17)
#define NDT_RESTORE_COEFF_ADDR		(0xB5)
#define NDT_RESTORE_COEFF_LEN		(1)

/* about i2c msg */
#define ONE_MSG_NUM				1
#define TWO_MSG_NUM				2

/* flash and sram address division */
#define FLASH_BOOT_BASE_ADDR			0x01000000
#define FLASH_APP_BASE_ADDR			0x01001000
#define FLASH_MAX_ADDR				0x01010000
#define SRAM_BASE_ADDR				0x20001000
#define SRAM_MAX_ADDR				0x20002000
#define ERASE_FLASH_BOOT_SIZE			8

#define AW_REG_SRAM_R0				(0x02000000)
#define AW_REG_SRAM_R1				(0x02000600)
#define AW_REG_ISP_CR				(0x5000C000)
#define AW_REG_ISP_ADR				(0x5000C004)
#define AW_REG_ISP_CMD				(0x5000C010)
#define AW_REG_ISP_GO				(0x5000C014)
#define AW_REG_ISP_WDAT0			(0x5000C080)
#define AW_REG_ISP_RDAT0			(0x5000C0C0)
#define AW_FLASH_BOOT_START			(0x69)
#define AW_ROM_BOOT_START			(0x00)
#define AW_START_MODE_FUNC0			(0x00000017)
#define AW_START_MODE_FUNC1			(0x00000097)
#define AW_START_MODE_FUNC2			(0x000002F0)
#define AW_START_MODE_FUNC3			(0x000002F2)
#define AW_START_MODE_FUNC4			(0x000002F4)
#define AW_NVR_JUDGE				(0x86802)

#define AW_LOSC_MAX_DATA			(0x1E)
#define AW_LOSC_DEFAULT_DATA			(0x1D)
#define AW_LOSC_MIN_DATA			(0x1C)
#define AW_HOSC_MAX_DATA			(0xB4)
#define AW_HOSC_DEFAULT_DATA			(0xA8)
#define AW_HOSC_MIN_DATA			(0xA0)
#define AW_LDO_MAX_DATA				(0xAF)
#define AW_LDO_DEFAULT_DATA			(0xA0)
#define AW_LDO_MIN_DATA				(0xAD)

#define WAKE_STATUS				(0)
#define NO_ADB					0

enum gpio_level_signal {
	LOW_LEVEL,
	HIGH_LEVEL,
};

enum adb_update_flash {
	ADB_UPDATE_FLASH_BOOT = 1,
	ADB_UPDATE_FLASH_APP,
};

enum aw8680x_i2c_flag {
	I2C_WRITE_FLAG,
	I2C_READ_FLAG,
};

enum return_flag_enum {
	AW_SUCCESS,
	ERR_FLAG,
	CHIPID_ERR,
	ACK_ERR,
	ERR_JUMP,
	NOT_NEED_UPDATE,
	IRQ_REGISTER_ERR,
	IRQ_THREAD_ERR,
	ERR_CREAT_I2C_OBJ,
	ERR_CREAT_PROC_OBJ,
	ERR_ADD_SYS_OBJ,
	ERR_CREAT_SYS_OBJ,
	INPUT_ALLOC_ERR,
	INPUT_REGISTER_ERR,
	SRAM_BIN_FAILED,
	JUMP_BOOT_FAILED,
	FLAH_BOOT_BIN_ERR,
	FLAH_APP_BIN_ERR,
	CHECKSUM_ERR,
};


enum report_mode_enum {
	IRQ_MODE_SET = true,
	POLLING_MODE_SET = false,
};

enum scan_mode_switch_enum {
	SWITCH = 1,
	HIGH_SPEED = 2,
	LOW_SPEED = 3,
	POWER_OFF = 4,
};

enum updata_enum {
	AW8680X_FLASH_NO_UPDATE,
	AW8680X_FLASH_BOOT_UPDATE,
	AW8680X_FLASH_APP_UPDATE,
};

struct data_container {
	unsigned int len;
	unsigned char data[];
};

/*
 * ic_flag : confirm rom boot or flash boot start
 * update_mutex_flag : updating one bin, the other bin must not update
 * flash_app_update_flag : confirm updating the flash app bin
 * adb_update_flash_app : adb update flash app is not confirm flash app version
 * flash_app_states : adb update flash app is not confirm flash app version
 * flash_boot_states : adb update flash app is not confirm flash app version
 */
struct aw8680x {
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;
	struct delayed_work bin_work;
	struct aw_bin *sram_bin;
	struct aw_bin *flash_app_bin;
	struct aw_bin *flash_boot_bin;
	struct hrtimer hr_timer;
	struct proc_dir_entry *prEntry_da;
	struct proc_dir_entry *prEntry_tmp[NUM_NODES];
	bool flash_boot_func;
	bool input_func;
	bool flash_app_version_get_flag;
	struct class *sysfs_class;
	struct device *sysfs_dev;

	struct gui_to_soc_struct p_gui_data_s;
	struct mutex aw8680x_i2c_mutex;
	struct mutex timer_mutex;
	unsigned char p_protocol_tx_data[PROTOCOL_TOTAL_LEN];
	unsigned char p_protocol_rx_data[PROTOCOL_TOTAL_LEN];
	char read_data[PROTOCOL_TOTAL_LEN];
	unsigned char irq_key_data;
	unsigned char module_id;
	unsigned char reg_addr;
	unsigned short read_len;

	int32_t irq_gpio;
	int32_t reset_gpio;
	int32_t wake_gpio;
	int32_t state_gpio;
	int32_t ldo_gpio;
	uint32_t flash_app_version_in_soc;
	uint32_t flash_boot_version_in_bin;
	uint32_t flash_app_version_in_bin;
	uint32_t pc_location;
	uint32_t pc_point_flash_app;
	uint32_t flash_app_addr;
	uint32_t flash_boot_addr;
	uint32_t sram_addr;
	uint8_t wake_gpio_valid;
	uint8_t state_pin_valid;
	uint8_t rst_gpio_valid;
	uint8_t irq_gpio_valid;
	int8_t ack_flag;
	uint8_t ic_flag;
	uint8_t timer_wake_state;
	uint8_t update_mutex_flag;
	uint8_t flash_app_update_flag;
	uint8_t adb_update_flash_app;
	uint8_t flash_app_states;
	uint8_t flash_boot_states;
	uint8_t sysclass_register;
};

static char aw8680x_proc_node_name[][NUM_NODES] = {
	/* 0 */		{"reg"},
	/* 1 */		{"connect"},
	/* 2 */		{"update"},
	/* 3 */		{"aw8680x_ndt_1hz"},
	/* 4 */		{"flash_app_status"},
	/* 5 */		{"flash_boot_status"},
	/* 6 */		{"jump"},
	/* 7 */		{"reset"},
	/* 8 */		{"wakeup"},
	/* 9 */		{"adc_data"},
	/* 10 */	{"force_event"},
	/* 11 */	{"force_data"},
	/* 12 */	{"base_data"},
	/* 13 */	{"diff_data"},
	/* 14 */	{"diff_threshold"},
	/* 15 */	{"sensor_type"},
	/* 16 */	{"sw_algo_version"},
	/* 17 */	{"pga_data"},
	/* 18 */	{"tempera_data"},
	/* 19 */	{"adc_dr"},
	/* 20 */	{"dac_dr"},
	/* 21 */	{"cali"},
	/* 22 */	{"sensor_status"},
	/* 23 */	{"adc_coef"},
	/* 24 */	{"dac_coef"},
	/* 25 */	{"FTC_cali"},
	/* 26 */	{"FTC_coef"},
	/* 27 */	{"FTC_noise"},
	/* 28 */	{"FTC_no_press"},
	/* 29 */	{"force_mode"},
	/* 30 */	{"ndt_restore_coeff"},
	/* 31 */	{"ndt_reg_dump"},
};
#endif

unsigned int ndt_tp_transfer(unsigned int x,unsigned int y);
