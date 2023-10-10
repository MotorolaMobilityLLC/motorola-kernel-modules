/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _HAPTIC_NV_H_
#define _HAPTIC_NV_H_
/*********************************************************
 *
 * Macro Control
 *
 *********************************************************/
#define AW_CHECK_RAM_DATA
#define AW_READ_BIN_FLEXBALLY
#define AW_INPUT_FRAMEWORK
#define AW_TIKTAP
/* #define AW_DOUBLE */
#define AW_DURATION_DECIDE_WAVEFORM
/* #define AW_ENABLE_RTP_PRINT_LOG */
/* #define AW862X_MUL_GET_F0 */
/* #define AW_MAXIMUM_F0_CALI_DATA */
#define AW862X_DRIVER
#define AW862XX_DRIVER
#define AW8623X_DRIVER
#define AW8624X_DRIVER
/*********************************************************
 *
 * Haptic_NV CHIPID
 *
 *********************************************************/
#define AW8623_CHIP_ID				(0x23)
#define AW8624_CHIP_ID				(0x24)
#define AW8622X_CHIP_ID				(0x00)
#define AW86214_CHIP_ID				(0x01)
#define AW8623X_CHIP_ID_H			(0x23)
#define AW86233_CHIP_ID				(0x2330)
#define AW86234_CHIP_ID				(0x2340)
#define AW86235_CHIP_ID				(0x2350)
#define AW8624X_CHIP_ID_H			(0x24)
#define AW86243_CHIP_ID				(0x2430)
#define AW86245_CHIP_ID				(0x2450)
/*********************************************************
 *
 * Marco
 *
 *********************************************************/
#define AW_I2C_NAME				"haptic_nv"
#define AW_I2C_RETRIES				(5)
#define AW_I2C_READ_MSG_NUM			(2)
#define AW_I2C_BYTE_ONE				(1)
#define AW_I2C_BYTE_TWO				(2)
#define AW_I2C_BYTE_THREE			(3)
#define AW_I2C_BYTE_FOUR			(4)
#define AW_I2C_BYTE_FIVE			(5)
#define AW_I2C_BYTE_SIX				(6)
#define AW_I2C_BYTE_SEVEN			(7)
#define AW_I2C_BYTE_EIGHT			(8)

#define AWRW_SIZE				(220)
#define AW_REG_MAX				(0xFF)
#define AW_NAME_MAX				(64)
#define AW_VBAT_MIN				(3000)
#define AW_VBAT_MAX				(4500)
#define AW_TRIG_NUM				(3)
#define AWRW_CMD_UNIT				(5)
#define AW_VBAT_REFER				(4200)
#define AW_DEFAULT_GAIN				(0x80)
#define AW_SEQUENCER_SIZE			(8)
#define AW_PM_QOS_VALUE_VB			(400)
#define AW_GUN_TYPE_DEF_VAL			(0xFF)
#define AW_DRV_WIDTH_MIN			(0)
#define AW_DRV_WIDTH_MAX			(255)
#define AW_BULLET_NR_DEF_VAL			(0)
#define CPU_LATENCY_QOC_VALUE			(0)
#define AW_SEQUENCER_LOOP_SIZE			(4)
#define AW_RAMDATA_SHOW_COLUMN			(16)
#define AW_READ_CHIPID_RETRIES			(5)
#define AW_RAMDATA_RD_BUFFER_SIZE		(1024)
#define AW_RAMDATA_WR_BUFFER_SIZE		(2048)
#define AW_RAMDATA_SHOW_UINT_SIZE		(6)
#define AW_READ_CHIPID_RETRY_DELAY		(2)
#define AW_EFFECT_NUMBER			(3)
#define AW_RAM_WORK_DELAY_INTERVAL		(8000)
#define AW_RAMDATA_SHOW_LINE_BUFFER_SZIE	(100)

#define AW_RL_DELAY_MIN				(3000)
#define AW_RL_DELAY_MAX				(3500)
#define AW_F0_DELAY_MIN				(10000)
#define AW_F0_DELAY_MAX				(10500)
#define AW_RTP_DELAY_MIN			(2000)
#define AW_RTP_DELAY_MAX			(2500)
#define AW_PLAY_DELAY_MIN			(2000)
#define AW_PLAY_DELAY_MAX			(2500)
#define AW_STOP_DELAY_MIN			(2000)
#define AW_STOP_DELAY_MAX			(2500)
#define AW_VBAT_DELAY_MIN			(2000)
#define AW_VBAT_DELAY_MAX			(2500)
#define AW_OSC_TRIM_PARAM			(50)
#define AW_OSC_CALI_MAX_LENGTH			(5100000)

#define AW_SET_RAMADDR_H(addr)			((addr) >> 8)
#define AW_SET_RAMADDR_L(addr)			((addr) & 0x00FF)
#define AW_SET_BASEADDR_H(addr)			((addr) >> 8)
#define AW_SET_BASEADDR_L(addr)			((addr) & 0x00FF)
/*********************************************************
 *
 * aw862x marco
 *
 ********************************************************/
#define AW862X_F0_CALI_ACCURACY			(25)
#define AW862X_MUL_GET_F0_RANGE			(150)
#define AW862X_MUL_GET_F0_NUM			(3)

#define AW862X_VBAT_FORMULA(code)		(6100 * (code) / 256)
#define AW862X_F0_FORMULA(reg, coeff)		(1000000000 / ((reg) * (coeff)))
#define AW862X_RL_FORMULA(reg_val)		(298 * (reg_val))
#define AW862X_SET_AEADDR_H(addr)		((((addr) >> 1) >> 8))
#define AW862X_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW862X_SET_AFADDR_H(addr)		(((addr) - ((addr) >> 2)) >> 8)
#define AW862X_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
/*********************************************************
 *
 * aw862xx marco
 *
 ********************************************************/
#define AW862XX_DRV2_LVL_MAX			(127)
#define AW862XX_F0_CALI_ACCURACY		(24)
#define AW862XX_VBAT_FORMULA(code)		(6100 * (code) / 1024)
#define AW862XX_OS_FORMULA(os_code, d2s_gain)	(2440 * ((os_code) - 512) / (1024 * ((d2s_gain) + 1)))
#define AW862XX_F0_FORMULA(code)		(384000 * 10 / (code))
#define AW862XX_RL_FORMULA(code, d2s_gain)	(((code) * 678 * 100) / (1024 * (d2s_gain)))
#define AW862XX_SET_AEADDR_H(addr)		((((addr) >> 1) >> 4) & 0xF0)
#define AW862XX_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW862XX_SET_AFADDR_H(addr)		((((addr) - ((addr) >> 2)) >> 8) & 0x0F)
#define AW862XX_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
#define AW862XX_DRV2_LVL_FORMULA(f0, vrms)	((((f0) < 1800) ? 1809920 : 1990912) / 1000 * (vrms) / 61000)
#define AW862XX_DRV_WIDTH_FORMULA(f0, margin, brk_gain) \
						((240000 / (f0)) - (margin) - (brk_gain) - 8)

/*********************************************************
 *
 * aw8623x marco
 *
 ********************************************************/
#define AW8623X_DRV2_LVL_MAX			(127)
#define AW8623X_F0_CALI_ACCURACY		(24)

#define AW8623X_VBAT_FORMULA(code)		(6100 * (code) / 1023)
#define AW8623X_F0_FORMULA(code)		(384000 * 10 / (code))
#define AW8623X_RL_FORMULA(code, d2s_gain)	(((code) * 678 * 1000) / (1023 * (d2s_gain)))
#define AW8623X_OS_FORMULA(code, d2s_gain)	(2440 * ((code) - 512) / (1023 * (1 + d2s_gain)))
#define AW8623X_SET_AEADDR_H(addr)		((((addr) >> 1) >> 4) & 0xF0)
#define AW8623X_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW8623X_SET_AFADDR_H(addr)		((((addr) - ((addr) >> 2)) >> 8) & 0x0F)
#define AW8623X_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
#define AW8623X_DRV2_LVL_FORMULA(f0, vrms)	((((f0) < 1800) ? 1809920 : 1990912) / 1000 * (vrms) / 40000)
#define AW8623X_DRV_WIDTH_FORMULA(f0, margin, brk_gain) \
						((240000 / (f0)) - (margin) - (brk_gain) - 8)

/*********************************************************
 *
 * aw8624x marco
 *
 ********************************************************/
#define AW8624X_DRV2_LVL_MAX			(127)
#define AW8624X_F0_CALI_ACCURACY		(24)

#define AW8624X_VBAT_FORMULA(code)		(6100 * (code) / 1023)
#define AW8624X_F0_FORMULA(code)		(384000 * 10 / (code))
#define AW8624X_RL_FORMULA(code, d2s_gain)	(((code) * 610 * 1000) / (1023 * (d2s_gain)))
#define AW8624X_OS_FORMULA(code, d2s_gain)	(2440 * ((code) - 512) / (1023 * (1 + d2s_gain)))
#define AW8624X_SET_AEADDR_H(addr)		((((addr) >> 1) >> 4) & 0xF0)
#define AW8624X_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW8624X_SET_AFADDR_H(addr)		((((addr) - ((addr) >> 2)) >> 8) & 0x0F)
#define AW8624X_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
#define AW8624X_DRV2_LVL_FORMULA(f0, vrms)	((((f0) < 1800) ? 1809920 : 1990912) / 1000 * (vrms) / 61000)
#define AW8624X_DRV_WIDTH_FORMULA(f0, margin, brk_gain) \
						((240000 / (f0)) - (margin) - (brk_gain) - 8)

/*********************************************************
 *
 * Conditional Marco
 *
 *********************************************************/

#ifdef AW_TIKTAP
#define AW_TIKTAP_PROCNAME			"tiktap_buf"
#define AW_TIKTAP_PROCNAME_R			"tiktap_buf_r"
#define AW_TIKTAP_MMAP_PAGE_ORDER		(2)
#define AW_TIKTAP_MMAP_BUF_SUM			(16)
#define AW_TIKTAP_MMAP_BUF_SIZE			(1000)

#pragma pack(4)
struct mmap_buf_format {
	uint8_t status;
	uint8_t bit;
	int16_t length;

	struct mmap_buf_format *kernel_next;
	struct mmap_buf_format *user_next;
	uint8_t reg_addr;
	int8_t data[AW_TIKTAP_MMAP_BUF_SIZE];
}; /* 1024 byte */
#pragma pack()

#define TIKTAP_IOCTL_GROUP		0xFF
#define TIKTAP_GET_F0			_IO(TIKTAP_IOCTL_GROUP, 0x01)
#define TIKTAP_GET_HWINFO		_IO(TIKTAP_IOCTL_GROUP, 0x02)
#define TIKTAP_SET_FREQ			_IO(TIKTAP_IOCTL_GROUP, 0x03)
#define TIKTAP_SETTING_GAIN		_IO(TIKTAP_IOCTL_GROUP, 0x04)
#define TIKTAP_SETTING_SPEED		_IO(TIKTAP_IOCTL_GROUP, 0x05)
#define TIKTAP_SETTING_BSTVOL		_IO(TIKTAP_IOCTL_GROUP, 0x06)
#define TIKTAP_ON_MODE			_IO(TIKTAP_IOCTL_GROUP, 0x07)
#define TIKTAP_OFF_MODE			_IO(TIKTAP_IOCTL_GROUP, 0x08)
#define TIKTAP_RTP_MODE			_IO(TIKTAP_IOCTL_GROUP, 0x09)
#define TIKTAP_RTP_IRQ_MODE		_IO(TIKTAP_IOCTL_GROUP, 0x0A)
#define TIKTAP_STOP_MODE		_IO(TIKTAP_IOCTL_GROUP, 0x0B)
#define TIKTAP_STOP_RTP_MODE		_IO(TIKTAP_IOCTL_GROUP, 0x0C)
#define TIKTAP_WRITE_REG		_IO(TIKTAP_IOCTL_GROUP, 0x0D)
#define TIKTAP_READ_REG			_IO(TIKTAP_IOCTL_GROUP, 0x0E)
#define TIKTAP_BST_SWITCH		_IO(TIKTAP_IOCTL_GROUP, 0x0F)
#define TIKTAP_GET_SPEED		_IO(TIKTAP_IOCTL_GROUP, 0x10)

enum {
	MMAP_BUF_DATA_VALID = 0x55,
	MMAP_BUF_DATA_FINISHED = 0xAA,
	MMAP_BUF_DATA_INVALID = 0xFF,
};
#endif

#if KERNEL_VERSION(4, 4, 1) >= LINUX_VERSION_CODE
#define TIMED_OUTPUT
#endif

#if KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE
#define KERNEL_OVER_5_10
#endif

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
#define KERNEL_OVER_6_1
#endif

#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
typedef struct timed_output_dev cdev_t;
#else
typedef struct led_classdev cdev_t;
#endif
/*********************************************************
 *
 * Log Format
 *
 *********************************************************/
#ifdef AW_DOUBLE
#define aw_err(format, ...) \
	pr_err("[haptic_nv]<%s>%s: " format "\n", aw_haptic->mark, __func__, ##__VA_ARGS__)

#define aw_info(format, ...) \
	pr_info("[haptic_nv]<%s>%s: " format "\n", aw_haptic->mark, __func__, ##__VA_ARGS__)

#define aw_dbg(format, ...) \
	pr_debug("[haptic_nv]<%s>%s: " format "\n", aw_haptic->mark, __func__, ##__VA_ARGS__)
#else
#define aw_err(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define aw_info(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define aw_dbg(format, ...) \
	pr_debug("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)
#endif
/*********************************************************
 *
 * Enum Define
 *
 *********************************************************/
enum awrw_flag {
	AW_WRITE = 0,
	AW_READ = 1,
};

enum aw_haptic_flags {
	AW_FLAG_NONR = 0,
	AW_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw_haptic_work_mode {
	AW_STANDBY_MODE = 0,
	AW_RAM_MODE = 1,
	AW_RAM_LOOP_MODE = 2,
	AW_CONT_MODE = 3,
	AW_RTP_MODE = 4,
	AW_TRIG_MODE = 5,
	AW_NULL = 6,
};

enum aw_haptic_cont_vbat_comp_mode {
	AW_CONT_VBAT_SW_COMP_MODE = 0,
	AW_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw_haptic_ram_vbat_comp_mode {
	AW_RAM_VBAT_COMP_DISABLE = 0,
	AW_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw_haptic_pwm_mode {
	AW_PWM_48K = 0,
	AW_PWM_24K = 1,
	AW_PWM_12K = 2,
};

enum aw_haptic_play {
	AW_PLAY_NULL = 0,
	AW_PLAY_ENABLE = 1,
	AW_PLAY_STOP = 2,
	AW_PLAY_GAIN = 8,
};

enum aw_haptic_cmd {
	AW_CMD_NULL = 0,
	AW_CMD_ENABLE = 1,
	AW_CMD_HAPTIC = 0x0F,
	AW_CMD_TP = 0x10,
	AW_CMD_R_EN = 0x40,
	AW_CMD_L_EN = 0x80,
	AW_CMD_SYS = 0xF0,
	AW_CMD_STOP = 255,
};

enum aw_haptic_cali_lra {
	AW_WRITE_ZERO = 0,
	AW_F0_CALI_LRA = 1,
	AW_OSC_CALI_LRA = 2,
};

enum aw_haptic_awrw_flag {
	AW_SEQ_WRITE = 0,
	AW_SEQ_READ = 1,
};

enum aw_trim_lra {
	AW_TRIM_LRA_BOUNDARY = 0x20,
	AW8624X_TRIM_LRA_BOUNDARY = 0x40,
};

enum aw_haptic_read_chip_type {
	AW_FIRST_TRY = 0,
	AW_LAST_TRY = 1,
};

enum aw_haptic_chip_name {
	AW_CHIP_NULL = 0,
	AW86223 = 1,
	AW86224 = 2,
	AW86225 = 3,
	AW86214 = 4,
	AW8623 = 5,
	AW8624 = 6,
	AW86233 = 7,
	AW86234 = 8,
	AW86235 = 9,
	AW86243 = 10,
	AW86245 = 11,
};

enum aw_haptic_irq_state {
	AW_IRQ_ALMOST_EMPTY = 1,
};

enum aw_haptic_dts_data_type {
	AW_DTS_NULL = 0,
	AW_DTS_DATA_U32 = 1,
	AW_DTS_DATA_ARRAY = 2,
	AW_DTS_DATA_BOOL = 3,
};

enum aw_haptic_protect_config {
	AW_PROTECT_OFF = 0,
	AW_PROTECT_EN = 1,
	AW_PROTECT_CFG_1 = 0x2D,
	AW_PROTECT_CFG_2 = 0x3E,
	AW_PROTECT_CFG_3 = 0X3F,
};

enum aw_haptic_ram_file_num {
	AW862X_RAM_FILE = 0,
	AW862XX_RAM_FILE = 1,
};

enum aw_haptic_pin {
	AW_TRIG1 = 0,
	AW_TRIG2 = 1,
	AW_TRIG3 = 2,
	AW_IRQ = 3,
};

/*********************************************************
 *
 * Enum aw8623x
 *
 *********************************************************/
enum aw8623x_sram_size_flag {
	AW8623X_HAPTIC_SRAM_1K = 0,
	AW8623X_HAPTIC_SRAM_2K = 1,
	AW8623X_HAPTIC_SRAM_3K = 2,
};

/*********************************************************
 *
 * Enum aw862xx
 *
 *********************************************************/
enum aw862xx_ef_id {
	AW86223_EF_ID = 0x01,
	AW86224_EF_ID = 0x00,
	AW86225_EF_ID = 0x00,
	AW86214_EF_ID = 0x41,
};

enum aw862xx_sram_size_flag {
	AW862XX_HAPTIC_SRAM_1K = 0,
	AW862XX_HAPTIC_SRAM_2K = 1,
	AW862XX_HAPTIC_SRAM_3K = 2,
};
/*********************************************************
 *
 * Enum aw862x
 *
 *********************************************************/
enum aw862x_pwm_clk {
	AW862X_CLK_24K = 2,
	AW862X_CLK_12K = 3,
};
/*********************************************************
 *
 * Struct Define
 *
 *********************************************************/
struct trig {
	uint8_t enable;
	uint8_t trig_edge;
	uint8_t trig_brk;
	uint8_t trig_level;
	uint8_t trig_polar;
	uint8_t pos_enable;
	uint8_t neg_enable;
	uint8_t pos_sequence;
	uint8_t neg_sequence;
};

struct aw_haptic_ram {
	uint8_t ram_num;
	uint8_t version;
	uint8_t ram_shift;
	uint8_t baseaddr_shift;

	uint32_t len;
	uint32_t check_sum;
	uint32_t base_addr;
};

struct aw_haptic_ctr {
	uint8_t cnt;
	uint8_t cmd;
	uint8_t play;
	uint8_t loop;
	uint8_t gain;
	uint8_t wavseq;
	struct list_head list;
};

struct aw_i2c_info {
	uint8_t flag;
	uint8_t reg_num;
	uint8_t first_addr;
	uint8_t reg_data[AWRW_SIZE];
};

struct aw_haptic_audio {
	int delay_val;
	int timer_val;
	struct mutex lock;
	struct hrtimer timer;
	struct list_head list;
	struct work_struct work;
	struct aw_haptic_ctr ctr;
	struct list_head ctr_list;
};

struct aw_haptic_dts_data {
	void *dts_val;
	uint8_t type;
	uint32_t len;
};

struct aw_haptic_dts_info {
	bool is_enabled_auto_brk;
	bool is_enabled_track_en;
	/* aw8624x */
	bool is_enabled_smart_loop;
	bool is_enabled_inter_brake;

	uint32_t mode;
	uint32_t f0_cali_percent;
	uint32_t f0_pre;
	uint32_t lra_vrms;
	uint32_t lk_f0_cali;
	uint32_t cont_tset;
	uint32_t cont_drv1_lvl;
	uint32_t cont_drv2_lvl;
	uint32_t duration_time[3];
	uint32_t trig_cfg[21];
	/* aw862x */
	uint32_t cont_td;
	uint32_t cont_zc_thr;
	uint32_t cont_num_brk;
	uint32_t f0_coeff;
	uint32_t cont_brake[24];
	uint32_t bemf_config[4];
	uint32_t sw_brake[2];
	uint32_t wavseq[16];
	uint32_t wavloop[10];
	uint32_t td_brake[3];
	uint32_t f0_trace_parameter[4];
	/* aw862xx */
	uint32_t gain_bypass;
	uint32_t cont_drv1_time;
	uint32_t cont_drv2_time;
	uint32_t cont_brk_time;
	uint32_t cont_track_margin;
	uint32_t cont_brk_gain;
	uint32_t d2s_gain;
	uint32_t prctmode[3];
	/* aw8624x */
	uint32_t f0_d2s_gain;
};

struct aw_haptic {
	bool rtp_init;
	bool ram_init;
	bool dual_flag;
	bool is_used_irq_pin;
	bool is_used_rst_pin;

	uint8_t flags;
	uint8_t play_mode;
	uint8_t chipid_flag;
	uint8_t max_pos_beme;
	uint8_t max_neg_beme;
	uint8_t activate_mode;
	uint8_t ram_vbat_comp;
	uint8_t seq[AW_SEQUENCER_SIZE];
	uint8_t loop[AW_SEQUENCER_SIZE];
	uint8_t trim_lra_boundary;
#ifdef AW_DOUBLE
	uint8_t mark[15];
#endif

	int name;
	int gain;
	int state;
	int index;
	int irq_gpio;
	int duration;
	int effect_id;
	int amplitude;
	int reset_gpio;

	uint32_t f0;
	uint32_t lra;
	uint32_t vbat;
	uint32_t level;
	uint32_t f0_pre;
	uint32_t cont_f0;
	uint32_t rtp_cnt;
	uint32_t rtp_len;
	uint32_t gun_type;
	uint32_t bullet_nr;
	uint32_t theory_time;
	uint32_t interval_us;
	uint32_t f0_cali_data;
	uint32_t rtp_file_num;
	uint32_t rtp_num_max;
	uint32_t ram_file_num;
	uint32_t timeval_flags;
	uint32_t osc_cali_data;
	uint32_t aw862xx_i2c_addr;
	uint64_t microsecond;

	cdev_t vib_dev;

	ktime_t kend;
	ktime_t kstart;
	ktime_t current_t;
	ktime_t pre_enter_t;
#ifdef AW_TIKTAP
	bool tiktap_ready;
	bool vib_stop_flag;
	bool tiktap_stop_flag;
	struct work_struct tiktap_work;
	struct proc_dir_entry *aw_config_proc;
	struct mmap_buf_format *start_buf;
#endif
	struct mutex lock;
	struct device *dev;
	struct hrtimer timer;
	struct mutex rtp_lock;
	struct i2c_client *i2c;
	struct aw_haptic_ram ram;
	struct aw_haptic_func *func;
	struct aw_i2c_info i2c_info;
	struct input_dev *input_dev;
	struct semaphore sema;
	struct work_struct rtp_work;
	struct work_struct stop_work;
	struct delayed_work ram_work;
	struct trig trig[AW_TRIG_NUM];
	struct aw_haptic_dts_info info;
	struct work_struct vibrator_work;
	struct work_struct input_gain_work;
	struct work_struct input_vib_work;
	struct aw_haptic_container *aw_rtp;
	struct aw_haptic_audio haptic_audio;
	struct workqueue_struct *work_queue;
	struct pm_qos_request aw_pm_qos_req_vb;
};

struct aw_haptic_container {
	int len;
	uint8_t data[];
};

struct aw_haptic_func {
	int (*check_qualify)(struct aw_haptic *aw_haptic);
	int (*get_irq_state)(struct aw_haptic *aw_haptic);
	int (*get_f0)(struct aw_haptic *aw_haptic);
	int (*offset_cali)(struct aw_haptic *aw_haptic);
	void (*creat_node)(struct aw_haptic *aw_haptic);
	void (*read_cont_f0)(struct aw_haptic *aw_haptic);
	void (*parse_dts)(struct aw_haptic *aw_haptic, struct device_node *np);
	void (*trig_init)(struct aw_haptic *aw_haptic);
	void (*irq_clear)(struct aw_haptic *aw_haptic);
	void (*haptic_start)(struct aw_haptic *aw_haptic);
	void (*play_stop)(struct aw_haptic *aw_haptic);
	void (*play_mode)(struct aw_haptic *aw_haptic, uint8_t play_mode);
	void (*cont_config)(struct aw_haptic *aw_haptic);
	void (*ram_init)(struct aw_haptic *aw_haptic, bool flag);
	void (*misc_para_init)(struct aw_haptic *aw_haptic);
	void (*interrupt_setup)(struct aw_haptic *aw_haptic);
	void (*vbat_mode_config)(struct aw_haptic *aw_haptic, uint8_t flag);
	void (*protect_config)(struct aw_haptic *aw_haptic, uint8_t prtime, uint8_t prlvl);
	void (*calculate_cali_data)(struct aw_haptic *aw_haptic);
	void (*set_gain)(struct aw_haptic *aw_haptic, uint8_t gain);
	void (*get_gain)(struct aw_haptic *aw_haptic, uint8_t *gain);
	void (*set_wav_seq)(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t seq);
	void (*get_wav_seq)(struct aw_haptic *aw_haptic, uint32_t len);
	void (*set_wav_loop)(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t loop);
	void (*get_wav_loop)(struct aw_haptic *aw_haptic, uint8_t *val);
	void (*set_rtp_data)(struct aw_haptic *aw_haptic, uint8_t *data, uint32_t len);
	void (*set_fifo_addr)(struct aw_haptic *aw_haptic);
	void (*get_fifo_addr)(struct aw_haptic *aw_haptic);
	void (*set_ram_data)(struct aw_haptic *aw_haptic, uint8_t *data, int len);
	void (*get_ram_data)(struct aw_haptic *aw_haptic, uint8_t *ram_data, int size);
	void (*set_ram_addr)(struct aw_haptic *aw_haptic);
	void (*set_repeat_seq)(struct aw_haptic *aw_haptic, uint8_t seq);
	void (*set_base_addr)(struct aw_haptic *aw_haptic);
	void (*set_trim_lra)(struct aw_haptic *aw_haptic, uint8_t val);
	void (*set_rtp_aei)(struct aw_haptic *aw_haptic, bool flag);
	void (*get_vbat)(struct aw_haptic *aw_haptic);
	void (*get_lra_resistance)(struct aw_haptic *aw_haptic);
	void (*get_first_wave_addr)(struct aw_haptic *aw_haptic, uint32_t *first_wave_addr);
	ssize_t (*get_reg)(struct aw_haptic *aw_haptic, ssize_t len, char *buf);
	uint8_t (*get_prctmode)(struct aw_haptic *aw_haptic);
	uint8_t (*get_glb_state)(struct aw_haptic *aw_haptic);
	uint8_t (*get_osc_status)(struct aw_haptic *aw_haptic);
	uint8_t (*judge_rtp_going)(struct aw_haptic *aw_haptic);
	uint8_t (*rtp_get_fifo_afs)(struct aw_haptic *aw_haptic);
	uint8_t (*rtp_get_fifo_aes)(struct aw_haptic *aw_haptic);
	uint64_t (*get_theory_time)(struct aw_haptic *aw_haptic);
};
/*********************************************************
 *
 * Function Call
 *
 *********************************************************/
#ifdef AW862X_DRIVER
extern struct aw_haptic_func aw862x_func_list;
#endif

#ifdef AW862XX_DRIVER
extern struct aw_haptic_func aw862xx_func_list;
#endif

#ifdef AW8623X_DRIVER
extern struct aw_haptic_func aw8623x_func_list;
#endif

#ifdef AW8624X_DRIVER
extern struct aw_haptic_func aw8624x_func_list;
#endif

extern int haptic_nv_i2c_reads(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len);
extern int haptic_nv_i2c_writes(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len);
extern int haptic_nv_i2c_write_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint32_t mask, uint8_t reg_data);
extern ssize_t haptic_nv_read_reg_array(struct aw_haptic *aw_haptic, char *buf, ssize_t len,
					uint8_t head_reg_addr, uint8_t tail_reg_addr);
#endif
