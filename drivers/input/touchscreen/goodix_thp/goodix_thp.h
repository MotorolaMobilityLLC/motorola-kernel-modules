/************************************************************************/
/* Copyright <2019-2020> GOODIX                                        */
/*                                                                      */
/* GOODIX Confidential. This software is owned or controlled by GOODIX  */
/* and may only be used strictly in accordance with the applicable      */
/* license terms.  By expressly accepting such terms or by downloading, */
/* installing, activating and/or otherwise using the software, you are  */
/* agreeing that you have read, and that you agree to comply with and   */
/* are bound by, such license terms.                                    */
/* If you do not agree to be bound by the applicable license terms,     */
/* then you may not retain, install, activate or otherwise use the      */
/* software.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef _GOODIX_THP_H_
#define _GOODIX_THP_H_

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/pinctrl/consumer.h>
#include <linux/cpufreq.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
#include "mtk_disp_notify.h"
#elif IS_ENABLED(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS
#include <linux/pm_qos.h>
#define PM_QOS_TOUCH_WAKEUP_VALUE 400
#endif

#ifdef GTP_PEN_NOTIFIER
#include <linux/pen_detection_notify.h>
#endif

/* macros definition */
#define GOODIX_THP_DRIVER_VERSION                       "1.0.2.8"
#define GOODIX_THP_DRIVER_NAME                          "goodix_thp_drvier"
#define GOODIX_CORE_DRIVER_NAME                         "goodix_thp"
#define GOODIX_THP_STYLUS_INPUT_DEVICE_NAME             "goodix_stylus_input"
#define GOODIX_THP_INPUT_DEVICE_NAME                    "gdix_input_agent"

/*chip_type*/
#define CHIP_TYPE_9897                                  1
#define CHIP_TYPE_9916                                  2
#define CHIP_TYPE_9966                                  3
#define CHIP_TYPE_9615                                  4

#define GOODIX_THP_MAX_FRAME_LEN                        (10 * 1024)
#define GOODIX_THP_MAX_TRANS_DATA_LEN                   (4096 * 32)
#define GOODIX_THP_MAX_FRAME_BUF_COUNT                  20
#define GOODIX_THP_CUSTOM_INFO_LEN                      10
#define GOODIX_MAX_STR_LABLE_LEN                        32
#define GOODIX_THP_REQUEST_APP_SIZE                     12

#define GOODIX_THP_DEFATULT_WAIT_FRAME_TIME             2500

/* cmd definition */
//TODO:to confirm
#define CMD_SLEEP                                       0x84
#define CMD_GESTURE                                     0xA6
#define CMD_EXIT_GESTURE                                0xA7
#define CMD_RAWDATA                                     0x90
#define CMD_TOUCH_REPORT                                0x91
#define CMD_ACTIVE_SCAN_RATE                            0x9D

/* 9897 reg definition */
#define REG_INT_REPORT_TYPE_FLAG_9897                   0x101A0
#define REG_GESTURE_DATA_9897                           0x101A0
#define REG_GESTURE_BUFFER_DATA_9897                    0x101CA

/* 9916 reg definition */
//TODO:to confirm
#define REG_INT_REPORT_TYPE_FLAG_9916                   0x10308
#define REG_GESTURE_DATA_9916                           0x10308
#define REG_GESTURE_BUFFER_DATA_9916                    0x1029E

/*others*/
#define WAIT_STATE					0
#define WAKEUP_STATE					1
#define GET_FRAME_BLOCK_MODE			        1
#define GET_FRAME_NONBLOCK_MODE				0
#define IRQ_ENABLE_FLAG					1
#define IRQ_DISABLE_FLAG				0
#define IRQ_WAKE_ENABLE_FLAG				1
#define IRQ_WAKE_DISABLE_FLAG				0
#define GESTURE_DOUBLE_CLICK				0
#define GESTURE_SINGLE_CLICK				1

#define RAWDATA_DISABLE					0
#define RAWDATA_ENABLE					1
#define TOUCH_DATA_DISABLE				0
#define TOUCH_DATA_ENABLE				1
#define SLEEP_MODE					0
#define GOODIX_BE_MODE  				0
#define GOODIX_LE_MODE  				1

//TODO:to update
#define SCAN_RATE_240					1
#define SCAN_RATE_180					2
#define SCAN_RATE_120					3
#define SCAN_RATE_60					4

#define GESTURE_DATA_TYPE				0x20

#define INPUT_AGENT_MAX_FINGERS			        10
#define INPUT_AGENT_MAX_STYLUS			        1
#define INPUT_AGENT_MAX_POINTS  ((INPUT_AGENT_MAX_FINGERS) + (INPUT_AGENT_MAX_STYLUS))
#define STYLUS_TRACK_ID                                 10
#define GOODIX_THP_MAX_TIMEOUT				5000u
#define GOODIX_SPI_SPEED_WAKEUP				3500
#define GOODIX_PEN_MAX_TILT				90

#define GESTURE_DATA_HEAD_LEN				8
#define GESTURE_TYPE_LEN				32
#define GESTURE_KEY_DATA_LEN				42
#define GESTURE_BUFFER_DATA_LEN				514

#define GOODIX_THP_TYPE_B_PROTOCOL
#define EQUAL_ZERO(r)               			((r) == 0)

#define SCREEN_OFF                                      0
#define SCREEN_ON                                       1

/* ioctl cmd for afehal */
#define IO_TYPE	 (0xB8)
#define IOCTL_CMD_GET_FRAME \
                _IOWR(IO_TYPE, 0x01, struct thp_ioctl_frame)
#define IOCTL_CMD_SET_RESET_VALUE			_IOW(IO_TYPE, 0x02, u32)
#define IOCTL_CMD_SET_WAIT_TIME				_IOW(IO_TYPE, 0x03, u32)
#define IOCTL_CMD_SPI_TRANS \
                _IOWR(IO_TYPE, 0x04, struct thp_ioctl_spi_trans_data)
#define IOCTL_CMD_NOTIFY_UPDATE \
                _IOW(IO_TYPE, 0x05, struct thp_ioctl_update_info)
#define IOCTL_CMD_SET_WAIT_MODE				_IOW(IO_TYPE, 0x06, u32)

#define IOCTL_CMD_IRQ_ENABLE				_IOW(IO_TYPE, 0x07, u32)
#define IOCTL_CMD_GET_FRAME_BUF_NUM			_IOW(IO_TYPE, 0x08, u32)
#define IOCTL_CMD_RESET_FRAME_LIST			_IOW(IO_TYPE, 0x09, u32)
#define IOCTL_CMD_GET_DRIVER_STATE			_IOR(IO_TYPE, 0x0A, u32)
#define IOCTL_CMD_GET_STATE_CHANGE_FLAG		        _IOR(IO_TYPE, 0x0B, u32)
#define IOCTL_CMD_SET_STATE_CHANGE_FLAG		        _IOW(IO_TYPE, 0x0C, u32)
#define IOCTL_CMD_SET_SPI_SPEED				_IOW(IO_TYPE, 0x0D, u32)
#define IOCTL_CMD_MUILT_SPI_TRANS \
                _IOWR(IO_TYPE, 0x0E, struct thp_ioctl_multi_spi_trans_data)
#define IOCTL_CMD_ENTER_SUSPEND				_IOW(IO_TYPE, 0x0F, u32)
#define IOCTL_CMD_ENTER_RESUME				_IO(IO_TYPE, 0x10)
#define IOCTL_CMD_RECV_TSC_MSG \
                _IOW(IO_TYPE, 0x11, struct thp_ioctl_tsc_msg)
#define IOCTL_CMD_GET_CHIP_TYPE                         _IOR(IO_TYPE, 0x12, u32)
#define IOCTL_CMD_SET_TOOL_OPS                          _IOW(IO_TYPE, 0x14, u32)
#define IOCTL_CMD_DUMP_REP_DONE                         _IOW(IO_TYPE, 0x15, u32)

/* ioctl cmd for daemon */
#define INPUT_AGENT_IO_TYPE  (0xB9)
#define INPUT_AGENT_IOCTL_CMD_SET_COOR \
        _IOWR(INPUT_AGENT_IO_TYPE, 0x01, \
                struct thp_input_agent_ioctl_coor_data)
#define INPUT_AGENT_IOCTL_READ_STATUS \
        _IOR(INPUT_AGENT_IO_TYPE, 0x02, u32)
#define INPUT_AGENT_IOCTL_CMD_SET_EVENTS \
        _IOR(INPUT_AGENT_IO_TYPE, 0x03, u32)
#define INPUT_AGENT_IOCTL_CMD_GET_EVENTS \
        _IOR(INPUT_AGENT_IO_TYPE, 0x04, u32)
#define INPUT_AGENT_IOCTL_GET_CUSTOM_INFO \
        _IOR(INPUT_AGENT_IO_TYPE, 0x05, u32)
#define INPUT_AGENT_IOCTL_GET_DRIVER_STATE \
        _IOR(INPUT_AGENT_IO_TYPE, 0x06, u32)

#define PINCTRL_STYLUS_CLK_ACTIVE       "stylus_clk_active"
#define PINCTRL_STYLUS_CLK_SUSPEND      "stylus_clk_suspend"

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

typedef enum {
        REQUEST_TYPE_FRAME = 1,
        REQUEST_TYPE_CMD,
        REQUEST_TYPE_NOTIFY,
        REQUEST_TYPE_SEND_CFG,
        REQUEST_TYPE_GET_CFG,
        REQUEST_TYPE_GET_DATA
} REQUEST_TYPE_T;

typedef enum {
        NOTIFY_TYPE_SCREEN = 1,
        NOTIFY_TYPE_CHARGE,
        NOTIFY_TYPE_GESTURE,
        NOTIFY_TYPE_TSD_CTRL,
        NOTIFY_TYPE_DUMP_REP,
        NOTIFY_TYPE_STYLUS_CTRL,
        NOTIFY_TYPE_RAWDATA,
        NOTIFY_TYPE_LOGTOFILE,
        NOTIFY_TYPE_SPECIAL_AREA,
        NOTIFY_TYPE_GAME_MODE,
        NOTIFY_TYPE_STOW_MODE,
        NOTIFY_TYPE_POCKET_MODE,
        NOTIFY_TYPE_ROTATION,
        NOTIFY_TYPE_SWITCH_REPORT_RATE,
        NOTIFY_TYPE_SAVE_MOTO_DATA,
        NOTIFY_TYPE_SHIPMODE,
} NOTIFY_TYPE_T;

enum pen_action_state {
    PEN_STATE_NONE,
    PEN_STATE_HOVER,
    PEN_STATE_TOUCH
};

#ifdef GTP_PEN_NOTIFIER
#define GTP_FINGER_MODE	0
#define GTP_PEN_MODE		1
#endif

enum pen_message_type {
    PEN_MESSAGE_BATTERY,
    PEN_MESSAGE_BLE_MAC,
    PEN_MESSAGE_PEN_INFO,
    PEN_MESSAGE_PEN_CLOSE,
    PEN_MESSAGE_PEN_QPID
};

#pragma pack(push, 1)
struct driver_response_app_pkg {
        uint32_t id;
        uint32_t type;
        uint32_t status;
        uint8_t data[0];
};

struct driver_response_pkg {
        uint32_t size;
        struct driver_response_app_pkg response;
};

struct driver_request_app_pkg {
        uint32_t id;
        uint32_t type;
        uint8_t data[];
};

struct driver_request_pkg {
        uint32_t size;
        struct driver_request_app_pkg request;
};
#pragma pack(pop)

struct thp_ioctl_frame {
        uint32_t pos;
        uint32_t size;
        uint64_t tv_us; /* tiemstamp us */
};

struct thp_ioctl_spi_trans_data {
        char __user *tx;
        char __user *rx;
        unsigned int size;
};

struct thp_ioctl_spi_xfer_data {
        char __user *tx;
        char __user *rx;
        unsigned int len;
        unsigned short delay_usecs;
        unsigned char cs_change;
        unsigned char reserved[3];
};

struct thp_ioctl_multi_spi_trans_data {
        unsigned int speed_hz;
        unsigned int xfer_num;
        unsigned int reserved[2];
        struct thp_ioctl_spi_xfer_data __user * xfer_data;
};

enum {
        SVC_CMD_MMAP_DEQUEUE = 31,
        SVC_CMD_BLE_MAC,
        SVC_CMD_GAME_FILTER,
        SVC_CMD_UPDATE_VERSION,
        SVC_CMD_HAL_INIT_FINISH = 36,
        SVC_CMD_BATTERY = 38,
        SVC_CMD_BUTTON,
        SVC_CMD_PEN_INFO,
        SVC_CMD_OPEN_CIRCUIT = 42,
        SVC_CMD_GET_PID,
};

#define MAX_TSC_MSG_DATA_LEN 128
struct thp_ioctl_tsc_msg {
        u32 cmd;
        u16 len;
        u8 value[MAX_TSC_MSG_DATA_LEN];
};

struct thp_ioctl_update_info {
        u32 frame_addr;
        u32 cmd_addr;
        u32 ges_addr;
        u32 esd_addr;
};

/* struct definition*/
struct thp_spi_setting {
        u32 spi_max_speed;
        u16 spi_mode;
        u8 bits_per_word;
};

struct goodix_mode_info {
        int sample;
        int report_rate_mode;
        int edge_mode[2];
        int interpolation;
        int stowed;
        int pocket_mode;
        int stylus_mode;
        int fp_int_state;
        int charger_mode;
};

struct goodix_thp_board_data {
        char avdd_name[GOODIX_MAX_STR_LABLE_LEN];
        char iovdd_name[GOODIX_MAX_STR_LABLE_LEN];
        char ic_name[GOODIX_MAX_STR_LABLE_LEN];
        unsigned int reset_gpio;
        unsigned int irq_gpio;
        int irq;
        unsigned int irq_flags;
        int iovdd_gpio;

        unsigned int power_on_delay_us;
        unsigned int power_off_delay_us;
        unsigned int panel_max_x;
        unsigned int panel_max_y;
        unsigned int panel_max_w; /*major and minor*/
        unsigned int panel_max_p; /*pressure*/
        unsigned int chip_type;
        bool esd_enable;
        unsigned int frame_addr;
        unsigned int cmd_addr;
        unsigned int ges_addr;
        unsigned int esd_addr;
        char thp_ver[100];
        struct thp_spi_setting spi_setting;
        bool report_rate_ctrl;
        bool interpolation_ctrl;
        bool sample_ctrl;
        bool stowed_mode_ctrl;
        bool pocket_mode_ctrl;
        bool edge_ctrl;
        bool stylus_mode_ctrl;
        int irq_need_dev_resume_time; /*control setting of wait resume time*/
        u32 sched_priority;
        u32 cpu_mask;
#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        int max_boost_count;
        int boost_timeout;
#endif
};

#define MMAP_BUFFER_SIZE (GOODIX_THP_MAX_FRAME_LEN * GOODIX_THP_MAX_FRAME_BUF_COUNT)
struct thp_frame_mmap_list {
        char *buf;
        u32 head;
        u32 tail;
};

#define MAX_SCAN_FREQ_NUM            8
#define MAX_SCAN_RATE_NUM            8
#define MAX_FREQ_NUM_STYLUS          8
#define MAX_STYLUS_SCAN_FREQ_NUM     6

struct thp_ts_device {
        char *name;
        char *tx_buff;
        char *rx_buff;
        struct mutex spi_mutex;
        struct device *dev;
        struct spi_device *spi_dev;
        const struct goodix_thp_hw_ops *hw_ops;
        struct goodix_thp_board_data board_data;
};

struct input_agent_coor_data {
        unsigned char down;
        unsigned char touch_valid; /* 0:invalid !=0:valid */
        int x;
        int y;
        int p;          // pen only
        int tilt_x;     // pen only
        int tilt_y;     // pen only
        int track_id;
        int major;
        int minor;
        int cancel_flag;
        unsigned int touch_type;
};

struct timeval64 {
        uint64_t tv_sec;
        uint64_t tv_usec;
};

struct thp_input_agent_ioctl_coor_data {
        struct input_agent_coor_data touch[INPUT_AGENT_MAX_POINTS];
        int touch_num;
        int down_num;
        unsigned char fp_mode;				/* 0:normal mode;1:fp mode*/
        unsigned char hover_stat;			/* 0:normal stat;1:hover stat*/
        unsigned char large_touch_stat;		        /* 0:normal touch stat;1:large_touch_stat*/
        unsigned char ref_not_set;
        struct timeval64 time_stamp;
        unsigned char stylus_key;
};

struct goodix_thp_hw_ops {
        int (*read)(struct thp_ts_device *dev, unsigned int addr,
                         unsigned char *data, unsigned int len);
        int (*write)(struct thp_ts_device *dev, unsigned int addr,
                        unsigned char *data, unsigned int len);
        int (*send_cmd)(struct thp_ts_device *tdev, u8 cmd, u16 data);
        int (*board_init)(struct thp_ts_device *ts_dev);
        int (*get_custom_info)(struct thp_ts_device *tdev, char *buf, unsigned int len);
        int (*get_frame)(struct thp_ts_device *dev, char *data);
        int (*get_version)(struct thp_ts_device *dev, u64 *version);
        int (*set_fp_int_pin)(struct thp_ts_device *dev, u8 level);
        int (*set_ble_broadcast)(struct thp_ts_device *dev, u8 enable);
        int (*reset)(struct thp_ts_device *dev, u32 delay_ms);
        int (*set_spi_speed)(struct thp_ts_device *dev, u32 speed);
};

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
struct cpu_boost_info {
        struct freq_qos_request qos_req;
        unsigned int max_freq;
        bool initialized;
};
#endif

struct goodix_thp_core {
        char thp_misc_name[32];
        struct miscdevice thp_misc_dev;
        char input_misc_name[32];
        struct miscdevice input_misc_dev;
        struct spi_device *sdev;
        struct thp_ts_device *ts_dev;
        struct platform_device *pdev;
        char input_dev_name[32];
        struct input_dev *input_dev;
        char pen_dev_name[32];
        struct input_dev *pen_dev;
        struct regulator *avdd;
        struct regulator *iovdd;
        struct thp_frame_mmap_list frame_mmap_list;
        struct mutex frame_mutex;
        struct mutex irq_mutex;
        struct mutex irq_wake_mutex;
        struct mutex mode_lock;
        bool esd_on;
        struct delayed_work esd_work;

#ifdef CONFIG_PINCTRL
        struct pinctrl *pinctrl;
        struct pinctrl_state *pin_sta_active;
        struct pinctrl_state *pin_sta_suspend;
        struct pinctrl_state *stylus_clk_active;
        struct pinctrl_state *stylus_clk_suspend;
#endif
#if IS_ENABLED(CONFIG_FB) || IS_ENABLED(CONFIG_DRM_MEDIATEK)
        struct notifier_block pm_notif;
#endif
        bool save_moto_data_on;
        bool special_area_on;
        bool logtofile_on;
        bool irq_state;
        bool irq_wake_state;
        u32 suspended;
        u16 gesture_enable;
        u32 state_change_flag;
        char irq_name[16];
        int irq;
        int power_on;
        int get_frame_wait_mode;
        unsigned int frame_len;
        unsigned int frame_wait_time;
        u8 reset_state;
        u8 frame_waitq_state;
        u8 frame_read_data[GOODIX_THP_MAX_FRAME_LEN];
        char custom_info[GOODIX_THP_CUSTOM_INFO_LEN + 1];
        wait_queue_head_t frame_wq;
        u8 gesture_type[GESTURE_TYPE_LEN];
        u8 gesture_data[GESTURE_KEY_DATA_LEN];
        u8 gesture_buffer_data[GESTURE_BUFFER_DATA_LEN];

        struct goodix_mode_info set_mode;
        struct goodix_mode_info get_mode;
        int refresh_rate;
        int zerotap_data[1];
        /* touchscreen_mmi */
        struct ts_mmi_class_methods *imports;
        struct timeval64 last_event_time;
        struct wakeup_source *ws;
        struct completion pm_completion;
        bool pm_suspend;
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS
        struct pm_qos_request pm_qos_req;
        int pm_qos_value;
        int pm_qos_state;
#endif
        u8 prev_finger_state[INPUT_AGENT_MAX_FINGERS]; // recording the prev finger state
        enum pen_action_state pen_state;

#ifdef CONFIG_ENABLE_TOUCH_CPU_BOOST
        int boost_count;
        struct timer_list boost_timer;

        struct cpu_boost_info *boost_infos;
        int *cpu_to_index_map;
        int qos_count;
#endif

#ifdef GTP_PEN_NOTIFIER
	int initialized;
	int gtp_pen_detect_flag;
	struct notifier_block pen_notif;
#endif
#ifdef CONFIG_GTP_HARDWARE_STATUS
	u8 open_status;
#endif
        u8 uevent_message_type;
        u8 pen_close;
        u8 battery_level;
        u8 ble_mac[6];
        u8 pen_info[9];
        u8 quick_pid;
};

extern bool debug_log_flag;
void _ts_info(struct device *dev, const char *func, int line, const char *fmt, ...);
void _ts_err(struct device *dev, const char *func, int line, const char *fmt, ...);
void _ts_debug(struct device *dev, const char *func, int line, const char *fmt, ...);

#define ts_info(dev, fmt, ...) \
        _ts_info(dev, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ts_err(dev, fmt, ...) \
        _ts_err(dev, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ts_debug(dev, fmt, ...) \
        do { \
                if (debug_log_flag) \
                        _ts_debug(dev, __func__, __LINE__, fmt, ##__VA_ARGS__); \
        } while (0)

/*
 * get board data pointer
 */
static inline struct goodix_thp_board_data *board_data(
                struct goodix_thp_core *core)
{
        if (!core || !core->ts_dev)
                return NULL;
        return &(core->ts_dev->board_data);
}

int goodix_thp_core_init(void);
int goodix_thp_core_deinit(void);
u16 checksum16_cmp(u8 *data, u32 size, int mode);
u8 checksum_u8(u8 *data, u32 size);
u8 checksum8_u16(const u8 *data, u32 size);

void put_frame_list(struct goodix_thp_core *core_data, int type, u8 *data, int len);
int goodix_ts_mmi_post_resume(struct goodix_thp_core *core_data);
int goodix_stylus_mode(struct goodix_thp_core *core_data, int mode);

#endif /* _GOODIX_THP_H_ */
