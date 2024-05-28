/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __FF_CORE_H__
#define __FF_CORE_H__

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <linux/delay.h>


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/* Max driver version buffer length. */
#define FF_DRV_VERSION_LEN          32
/* Max gesture key number, don't modify it */
#define FF_GK_MAX                   8


/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
 * Key code and value
 */
typedef struct {
    unsigned int code;
    int value;
} ff_key_event_t;

/*
 * fingerprint driver configuration
 */
typedef struct {
    /* Using asynchronous notification mechanism instead of NETLINK. */
    bool enable_fasync;

    /* Gesture(Key emulation & Navigation) key codes. */
    int32_t gesture_keycode[FF_GK_MAX];

    /* For '/dev/spidevB.C' of REE-Emulation. */
    bool enable_spidev;
    int32_t spidev_bus;
    int32_t spidev_c_s;

    /* For obsolete driver that doesn't support device tree. */
    int32_t gpio_mosi_pin;
    int32_t gpio_miso_pin;
    int32_t gpio_ck_pin;
    int32_t gpio_cs_pin;
    int32_t gpio_rst_pin;
    int32_t gpio_int_pin;
    int32_t gpio_power_pin;
    int32_t gpio_iovcc_pin;

    /* Logging driver to logcat through uevent mechanism. */
    int32_t log_level;
    bool logcat_driver;
} ff_driver_config_t;

typedef enum {
    FF_POLLEVT_NONE = 0,
    FF_POLLEVT_INTERRUPT = (1 << 0),
    FF_POLLEVT_SCREEN_ON = (1 << 1),
    FF_POLLEVT_SCREEN_OFF = (1 << 2),
} ff_poll_event_t;

typedef enum {
    FF_EVENT_NONE = 0,
    FF_EVENT_FASYNC = 1,
    FF_EVENT_NETLINK = 2,
    FF_EVENT_POLL = 3,
} ff_event_t;

typedef struct {
    uint16_t ver;
    uint16_t reserved1;
    uint32_t fbs_power_const      : 1;
    uint32_t fbs_wakelock         : 1;
    uint32_t fbs_screen_onoff     : 1;
    uint32_t fbs_pollevt          : 1;
    uint32_t fbs_reset_hl         : 1;
    uint32_t fbs_spiclk           : 1;
    uint32_t fbs_spiclk_hz        : 1;
    uint32_t fbs_unused           : 25;
    uint32_t reserved2[32];
} ff_feature_t;

/*
 * Focaltech fingerprint main structure
 */
typedef struct {
    struct cdev ff_cdev;
    struct class *ff_class;
    struct platform_device *pdev;
    struct device *fp_dev;
    struct spi_device *spi;
    struct fasync_struct *async_queue;
    struct input_dev *input;
    struct work_struct event_work;
    wait_queue_head_t wait_queue_head;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
    struct wakeup_source wake_lock;
    struct wakeup_source wake_lock_ctl;
#endif
    struct wakeup_source *p_ws;
    struct wakeup_source *p_ws_ctl;

    struct notifier_block fb_notif;

    struct regulator *ff_vdd;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_reset_low;
    struct pinctrl_state *pins_reset_high;
    struct pinctrl_state *pins_irq_as_int;
    struct pinctrl_state *pins_power_low;
    struct pinctrl_state *pins_power_high;

    ff_event_t event_type;
    ff_poll_event_t poll_event;
    ff_feature_t feature;
    ff_driver_config_t ff_config;
    uint16_t chip_id;
    int probe_id_ret;
    u32 spi_clk_hz;
    int irq_num;
    int irq_gpio;
    int reset_gpio;
    int vdd_gpio;
    bool b_use_regulator;
    bool b_use_pinctrl;
    bool b_irq_enabled;
    bool b_driver_inited;
    bool b_ff_probe;
    bool b_screen_onoff_event;
    bool b_screen_on;
    bool b_power_always_on;
    bool b_power_on;
    bool b_gesture_support;
    bool b_read_chipid;
    bool b_ree;
    bool b_spiclk;
    bool b_spiclk_enabled;

} ff_context_t;


/***
 * ioctl magic codes
 ***/
#define FF_IOC_MAGIC 'f'

/* Allocate/Release driver resource (GPIO/SPI etc.). */
#define FF_IOC_INIT_DRIVER      _IO(FF_IOC_MAGIC, 0x00)
#define FF_IOC_FREE_DRIVER      _IO(FF_IOC_MAGIC, 0x01)

/* HW reset the fingerprint module. */
#define FF_IOC_RESET_DEVICE     _IO(FF_IOC_MAGIC, 0x02)
#define FF_IOC_RESET_DEVICE_HL  _IOW(FF_IOC_MAGIC, 0x02, uint32_t)

/* Low-level IRQ control. */
#define FF_IOC_ENABLE_IRQ       _IO(FF_IOC_MAGIC, 0x03)
#define FF_IOC_DISABLE_IRQ      _IO(FF_IOC_MAGIC, 0x04)

/* SPI bus clock control, for power-saving purpose. */
#define FF_IOC_ENABLE_SPI_CLK   _IO(FF_IOC_MAGIC, 0x05)
#define FF_IOC_DISABLE_SPI_CLK  _IO(FF_IOC_MAGIC, 0x06)

/* Fingerprint module power control. */
#define FF_IOC_ENABLE_POWER     _IO(FF_IOC_MAGIC, 0x07)
#define FF_IOC_DISABLE_POWER    _IO(FF_IOC_MAGIC, 0x08)

/* Androind system-wide key event, for navigation purpose. */
#define FF_IOC_REPORT_KEY_EVENT _IOW(FF_IOC_MAGIC, 0x09, ff_key_event_t)

/* Sync 'ff_driver_config_t', the driver configuration. */
#define FF_IOC_SYNC_CONFIG      _IOWR(FF_IOC_MAGIC, 0x0a, ff_driver_config_t)

/* Query the driver version string. */
#define FF_IOC_GET_VERSION      _IOR(FF_IOC_MAGIC, 0x0b, const char)

/* Set vendor driver info. */
#define FF_IOC_SET_VENDOR_INFO  _IOW(FF_IOC_MAGIC, 0x0c, const char)

/* Acquire the WakeLock for a while. */
#define FF_IOC_WAKELOCK_ACQUIRE _IOW(FF_IOC_MAGIC, 0x0D, uint32_t)

/* Release the WakeLock immediately. */
#define FF_IOC_WAKELOCK_RELEASE _IO(FF_IOC_MAGIC, 0x0E)

/* device unprobe*/
#define FF_IOC_UNPROBE          _IO(FF_IOC_MAGIC, 0x0F)

/* set log level */
#define FF_IOC_SET_LOGLEVEL     _IOW(FF_IOC_MAGIC, 0x10, int)
#define FF_IOC_GET_LOGLEVEL     _IOR(FF_IOC_MAGIC, 0x10, int)

/*get feature*/
#define FF_IOC_GET_FEATURE      _IOR(FF_IOC_MAGIC, 0x11, ff_feature_t)

/* set event type */
#define FF_IOC_SET_EVENT_TYPE   _IOW(FF_IOC_MAGIC, 0x12, int)
#define FF_IOC_GET_EVENT_TYPE   _IOR(FF_IOC_MAGIC, 0x12, int)

/*get event info*/
#define FF_IOC_GET_EVENT_INFO   _IOR(FF_IOC_MAGIC, 0x13, int)

/*get spi-clk hz*/
#define FF_IOC_GET_SPICLK_HZ    _IOR(FF_IOC_MAGIC, 0x14, uint32_t)


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern ff_context_t *g_ff_ctx;

int ff_init_driver(void);
int ff_free_driver(void);
int fts_power_sequence(bool power_on);

#endif /* __FF_CORE_H__ */
