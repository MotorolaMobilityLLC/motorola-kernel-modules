#ifndef _FS3001_H_
#define _FS3001_H_


#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#include <sound/control.h>
#include <sound/soc.h>
#include "fshaptic.h"
#include <linux/timekeeping.h>

//Marco
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 1)
#define FS_KERNEL_VER_OVER_4_19
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 1)
#define FS_KERNEL_VER_OVER_5_10
#endif


#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
typedef struct timed_output_dev cdev_t;
#else
typedef struct led_classdev cdev_t;
#endif

#define FS3001_I2C_RETRIES		(5)
#define FS3001_I2C_RETRIES		(5)
#define FS3001_RTP_NAME_MAX		(64)
#define FS3001_SEQUENCER_SIZE		(8)
#define FS3001_SEQUENCER_LOOP_SIZE	(4)
#define FS3001_OSC_CALI_MAX_LENGTH	(11000000)
#define FS3001_PM_QOS_VALUE_VB		(400)
#define FS3001_VBAT_REFER		(4200)
#define FS3001_VBAT_MIN		(3000)
#define FS3001_VBAT_MAX		(5500)
#define FS3001_TRIG_NUM		(3)
#define FS3001_I2C_RETRY_DELAY		(2)
#define FS3001_OFFSET_RETRIES		(3)
#define FS3001_BASE_FRE		(384000)


#define FS3001_F0_REF_DEFAULT				(1700)
#define FS3001_CONT_DRV1_LVL_DEFAULT		(0x7F)
#define FS3001_CONT_DRV2_LVL_DEFAULT		(0x50)
#define FS3001_CONT_DRV1_TIME_DEFAULT		(0x04)
#define FS3001_CONT_DRV2_TIME_DEFAULT		(0x06)
#define FS3001_CONT_1_PERIOD_DEFAULT		(0x3F0)
#define FS3001_BRK_SLOPETH_DEFAULT			(0x60)
#define FS3001_BRK_GAIN_DEFAULT				(0x80)
#define FS3001_BRK_TIMES_DEFAULT			(0x0A)
#define FS3001_BRK_NOISE_GATE_DEFAULT		(0x10)
#define FS3001_BRK_1_PERIOD_DEFAULT			(0x300)
#define FS3001_GAIN_ADJUST_DEFAULT			(0x00)
#define FS3001_BRK_PGAGAIN_DEFAULT			(0x05)
#define FS3001_BRK_MARGIN_DEFAULT			(0x10)
#define FS3001_PLAY_RAM_SRATE_DEFAULT		(0x00)
#define FS3001_PLAY_RTP_SRATE_DEFAULT		(0x00)

#define FS3001_DEFAULT_VIB_MODE				(0x03)
#define FS3001_VBAT_MODE_DEFAULT			(0x00)
#define FS3001_LK_F0_CALI_DEFAULT			(0x00)
#define FS3001_BYPASS_SYSTEM_GAIN_DEFAULT	(0x00)
#define FS3001_F0_CALI_DATA_MODE_DEFAULT	(0x00)
#define FS3001_LR_PGAGAIN_DEFAULT			(0x05)

#define FS3001_REG_INITS_DEFAULT			(0xffffff)
#define FS3001_DURATION_TIME_DEFAULT		(30)

#define FS3001_RTP_ID_BOUNDARY_DEFAULT		(0x00)
#define FS3001_RTP_MAX_DEFAULT				(0x03)
#define FS3001_RTP_TIME_DEFAULT				(20)


// static int wf_s_repeat[4] = { 1, 2, 4, 8 };

enum fs3001_flags 
{
	FS3001_FLAG_NONR = 0,
	FS3001_FLAG_SKIP_INTERRUPTS = 1,
};

enum fs3001_haptic_work_mode 
{
	FS3001_HAPTIC_STANDBY_MODE = 0,
	FS3001_HAPTIC_RAM_MODE = 1,
	FS3001_HAPTIC_RTP_MODE = 2,
	FS3001_HAPTIC_TRIG_MODE = 3,
	FS3001_HAPTIC_CONT_MODE = 4,
	FS3001_HAPTIC_RAM_LOOP_MODE = 5,
	FS3001_HAPTIC_F0_DETECT_MODE = 6,
	FS3001_HAPTIC_F0_CALI_MODE = 7,
};

enum fs3001_haptic_activate_mode 
{
	FS3001_HAPTIC_ACTIVATE_RAM_MODE = 0,
	FS3001_HAPTIC_ACTIVATE_CONT_MODE = 1,
    FS3001_HAPTIC_ACTIVATE_RTP_MODE = 2,
	FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
};

enum fs3001_f0_cali_data_mode
{
	FS3001_F0_CALI_DATA_SELF_MODE = 0,
	FS3001_F0_CALI_DATA_CMDLINE_MODE = 1,
	FS3001_F0_CALI_DATA_DTS_MODE = 2,
};


//refer to doc 55.4
//0:disable  
//1:play enable(gain software)+brake disable  
//2:play enable(register configuration)+brake disable   
//3:play enable(auto-detected)+brake disable  
//4:play enable+brake enable (register configuration)  
//5:play enable+brake enable (auto-detected)
enum fs3001_haptic_vbat_compensate_mode 
{
	FS3001_HAPTIC_VBAT_COMP_DISABLE = 0,
	FS3001_HAPTIC_VBAT_COMP_PLAY_GAIN_BRK_DISABLE = 1,
	FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_DISABLE = 2,
	FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_DISABLE = 3,
	FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_ENABLE = 4,
	FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_ENABLE = 5,
};

enum fs3001_haptic_f0_flag 
{
	FS3001_HAPTIC_LRA_F0 = 0,
	FS3001_HAPTIC_CALI_F0 = 1,
};

enum fs3001_auto_brake_mode 
{
	FS3001_AUTO_BRAKE_DISABLE = 0,
	FS3001_AUTO_BRAKE_ENABLE = 1,
};


enum fs3001_f0_cali_mode 
{
	FS3001_F0_CALI_MODE_AUTO = 0,
	FS3001_F0_CALI_MODE_FORMULA = 1,
};

enum fs3001_haptic_pwm_mode 
{
	FS3001_PWM_12K = 0,
	FS3001_PWM_24K = 1,
	FS3001_PWM_48K = 2,
};

enum fs3001_haptic_play 
{
	FS3001_HAPTIC_PLAY_NULL = 0,
	FS3001_HAPTIC_PLAY_ENABLE = 1,
	FS3001_HAPTIC_PLAY_STOP = 2,
	FS3001_HAPTIC_PLAY_GAIN = 8,
};

enum fs3001_haptic_cmd 
{
	FS3001_HAPTIC_CMD_NULL = 0,
	FS3001_HAPTIC_CMD_ENABLE = 1,
	FS3001_HAPTIC_CMD_HAPTIC = 0x0f,
	FS3001_HAPTIC_CMD_TP = 0x10,
	FS3001_HAPTIC_CMD_SYS = 0xf0,
	FS3001_HAPTIC_CMD_STOP = 255,
};

enum fs3001_haptic_cali_lra 
{
	FS3001_WRITE_ZERO = 0,
	FS3001_F0_CALI = 1,
	FS3001_OSC_CALI = 2,
};

enum fs3001_haptic_rtp_mode 
{
	FS3001_RTP_SHORT = 4,
	FS3001_RTP_LONG = 5,
	FS3001_RTP_SEGMENT = 6,
};



//Struct Define
struct fs3001_trig 
{
	unsigned char trig_level;
	unsigned char trig_polar;
	unsigned char pos_enable;
	unsigned char pos_sequence;
	unsigned char neg_enable;
	unsigned char neg_sequence;
	unsigned char trig_brk;
};


struct fs3001_dts_info 
{
	unsigned int fs3001_f0_ref;
	unsigned int fs3001_auto_brake;
	unsigned int fs3001_f0_cali_mode;
	unsigned int fs3001_cont_drv1_lvl;
	unsigned int fs3001_cont_drv2_lvl;
	unsigned int fs3001_cont_drv1_time;
	unsigned int fs3001_cont_drv2_time;
	unsigned int fs3001_cont_1_period;
	unsigned int fs3001_brk_slopeth;
	unsigned int fs3001_brk_gain;
	unsigned int fs3001_brk_times;
	unsigned int fs3001_brk_noise_gate;
	unsigned int fs3001_brk_1_period;
	unsigned int fs3001_brk_pgagain;
	unsigned int fs3001_brk_margin;
	unsigned int fs3001_play_ram_srate;
	unsigned int fs3001_play_rtp_srate;



	unsigned int fs3001_default_vib_mode;
	unsigned int fs3001_vbat_mode;
	unsigned int fs3001_lk_f0_cali;
	unsigned int fs3001_bypass_system_gain;
	unsigned int fs3001_f0_cali_data_mode;
	unsigned int fs3001_lr_pgagain;

	unsigned int fs3001_duration_time[3];
	unsigned int fs3001_reg_inits[10];
	
	unsigned int fs3001_rtp_id_boundary;
	unsigned int fs3001_rtp_max;
	unsigned int fs3001_rtp_time[FS3001_RTP_MAX_DEFAULT];


	unsigned int trig_config[24];
    //20231117
	unsigned int fs3001_gain_adjust;
	
	
};

struct fs3001 
{
	struct regmap *regmap;
	struct i2c_client *i2c;

	//struct snd_soc_codec *codec; 
	struct device *dev;
	struct input_dev *input;
	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct long_vibrate_work;
	struct work_struct rtp_work;
	struct work_struct set_gain_work;
	struct delayed_work ram_work;
	struct delayed_work stop_work;
	struct fs3001_trig trig[FS3001_TRIG_NUM];
	struct fs3001_dts_info dts_info;
	struct fileops fileops;
	struct ram ram;
	struct timespec64 start, end;
	struct fs3001_container *rtp_container;

	cdev_t vib_dev;

#ifdef FS_HAPSTREAM
	struct work_struct rtp_hapstream;
	struct work_struct rtp_irq_hapstream;
	struct fs3001_container *hapstream_rtp;
	struct proc_dir_entry *fs_config_proc;
	bool hapstream_stop_flag;

	struct mmap_buf_format *start_buf;
	bool vib_stop_flag;
#endif

	unsigned char seq[FS3001_SEQUENCER_SIZE];
	unsigned char loop[FS3001_SEQUENCER_SIZE];
	unsigned char rtp_init;//in fs3001_rtp_work_routine, after memcpy(fs3001->rtp_container->data, rtp_file->data, rtp_file->size),rtp_init will be set to 1
	unsigned char ram_init;//set to 1 after ram bin loaded
	unsigned char rtp_routine_on;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;
	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;
	unsigned char play_mode;
	unsigned char activate_mode;

	unsigned char wk_lock_flag;

	bool isUsedIntn;

	int name;
	int reset_gpio;
	int irq_gpio;
	int state;
	int duration;
	int amplitude;
	int vmax;
	int gain;
	int sysclk;
	int rate;
	int width;
	int pstream;
	int cstream;

	unsigned int gun_type;
	unsigned int bullet_nr;
	unsigned int rtp_cnt;
	unsigned int rtp_file_num;
	unsigned int f0;
	unsigned int cont_f0;
	unsigned int theory_time;
	unsigned int vbat;
	unsigned int offset;
	unsigned int lra;
	unsigned int ram_update_flag;
	unsigned int rtp_update_flag;
	unsigned int osc_cali_data;
	unsigned int f0_cali_data;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned int sys_frequency;
	unsigned int rtp_len;
	unsigned long int microsecond;

	u16 new_gain;
	unsigned char level;
	struct haptic_audio haptic_audio;
	unsigned int osc_cali_run;
	unsigned char ram_vbat_comp;
	atomic_t is_in_rtp_loop;
	atomic_t exit_in_rtp_loop;
	wait_queue_head_t wait_q;
	wait_queue_head_t stop_wait_q;
	struct workqueue_struct *work_queue;
	struct work_struct vibrator_work;
	//ram monitor
#ifdef FS_RAM_STATE_OUTPUT
	struct delayed_work ram_monitor_work;
#endif
#ifdef INPUT_DEV
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct pwm_device *pwm_dev;
	struct qti_hap_config config;
	struct qti_hap_play_info play;
	struct qti_hap_effect *predefined;
	struct qti_hap_effect constant;
	struct regulator *vdd_supply;
	struct hrtimer stop_timer;
	struct hrtimer hap_disable_timer;
	spinlock_t bus_lock;
	ktime_t last_sc_time;
	int play_irq;
	int sc_irq;
	int effects_count;
	int sc_det_count;
	u16 reg_base;
	bool perm_disable;
	bool play_irq_en;
	bool vdd_enabled;
	int effect_type;
	int effect_id;
	int test_val;
#endif
	unsigned char fs3001_debug_enable;

};

struct fs3001_container 
{
	int len;
	unsigned char data[];
};

extern char fs3001_check_qualify(struct fs3001 *fs3001);
extern int fs3001_parse_dt(struct device *dev, struct fs3001 *fs3001,struct device_node *np);
extern void fs3001_interrupt_setup(struct fs3001 *fs3001);
extern int fs3001_vibrator_init(struct fs3001 *fs3001);
extern int fs3001_haptic_init(struct fs3001 *fs3001);
extern void fs3001_reg_init(struct fs3001 *fs3001);
extern void fs3001_f0_cali_setting_init(struct fs3001 *fs3001);
extern int fs3001_ram_work_init(struct fs3001 *fs3001);
extern irqreturn_t fs3001_irq(int irq, void *data);
extern struct attribute_group fs3001_vibrator_attribute_group;
extern void fs3001_haptics_set_gain(struct input_dev *dev, u16 gain);
extern int fs3001_haptics_erase(struct input_dev *dev, int effect_id);
extern int fs3001_haptics_playback(struct input_dev *dev, int effect_id,int val);
extern int fs3001_haptics_upload_effect (struct input_dev *dev,struct ff_effect *effect,struct ff_effect *old);
extern void fs3001_haptics_set_gain_work_routine(struct work_struct *work);

#endif
