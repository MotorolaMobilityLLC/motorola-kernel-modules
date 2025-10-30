/*
 * Copyright (C) 2025 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CXSEMI_UWB_H_
#define _CXSEMI_UWB_H_

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/pinctrl/consumer.h>
#include <linux/atomic.h>
#include <linux/gpio/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif

#if IS_ENABLED(CONFIG_FB)
#include <linux/notifier.h>
#endif

/* Driver and device identification constants */
#define CXSEMI_UWB_DRIVER_NAME         "cxsemi_uwb_driver"
#define CXSEMI_UWB_DEVICE_NAME         "cxsemi_uwb"
#define CXSEMI_UWB_RST_GPIO_NAME       "cxsemi_uwb_rst_gpio"
#define CXSEMI_UWB_IRQ_GPIO_NAME       "cxsemi_uwb_irq_gpio"

/* IOCTL command definitions */
#define CXSEMI_UWB_MAGIC        'C'
#define CXSEMI_UWB_DISABLE      _IO(CXSEMI_UWB_MAGIC, 0)
#define CXSEMI_UWB_ENABLE       _IO(CXSEMI_UWB_MAGIC, 1)
#define CXSEMI_UWB_RESET        _IO(CXSEMI_UWB_MAGIC, 3)
#define CXSEMI_UWB_GET_STATE    _IOR(CXSEMI_UWB_MAGIC, 4, int)

/* Device state definitions */
enum cxsemi_uwb_device_state {
    CXSEMI_UWB_STATE_UNINITIALIZED = 0,
    CXSEMI_UWB_STATE_READY,
    CXSEMI_UWB_STATE_SUSPENDED,
    CXSEMI_UWB_STATE_ERROR,
};

/* Error code definitions */
enum cxsemi_uwb_error_codes {
    CXSEMI_UWB_SUCCESS = 0,
    CXSEMI_UWB_ERROR_INVALID_PARAM = -1000,
    CXSEMI_UWB_ERROR_SPI_TRANSFER = -1001,
    CXSEMI_UWB_ERROR_GPIO_CONFIG = -1002,
    CXSEMI_UWB_ERROR_IRQ_REGISTRATION = -1003,
    CXSEMI_UWB_ERROR_MEMORY_ALLOC = -1004,
    CXSEMI_UWB_ERROR_DEVICE_TREE = -1005,
    CXSEMI_UWB_ERROR_DEVICE_STATE = -1006,
    CXSEMI_UWB_ERROR_BUSY = -1007,
};

/**
 * struct cxsemi_uwb_spi_setting - SPI configuration parameters
 */
struct cxsemi_uwb_spi_setting {
    u32 spi_max_speed;
    u16 spi_mode;
    u8 bits_per_word;
};

/**
 * struct cxsemi_uwb_board_data - Board-specific configuration data
 */
struct cxsemi_uwb_board_data {
    int rst_gpio;
    int irq_gpio;
    int irq;
    u32 irq_flags;
    struct cxsemi_uwb_spi_setting spi_setting;
};

/**
 * struct cxsemi_uwb_device - Main UWB device structure
 */
struct cxsemi_uwb_device {
    /* Buffers */
    u8 *tx_buff;
    u8 *rx_buff;
	u8 spi_bits_per_word;
	u32 spi_max_speed_hz_current;
	u32 spi_max_speed_hz_original;

    /* Synchronization */
    struct mutex mutex;
    struct mutex spi_mutex;  /* Separate mutex for SPI operations */
    wait_queue_head_t read_wait;

    /* Device management */
    struct miscdevice miscdev;
    struct spi_device *spi_dev;
    atomic_t irq_occurred;
    atomic_t device_state;
    bool device_initialized;
    bool gpio_configured;
    bool irq_enabled;
    bool single_read_mode;
    struct mutex read_mode_mutex;

    /* Board data */
    struct cxsemi_uwb_board_data board_data;

    /* Statistics */
    atomic_long_t spi_transfer_errors;
    atomic_long_t irq_count;
};

#endif
