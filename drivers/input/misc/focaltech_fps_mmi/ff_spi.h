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

#ifndef __FF_SPI_H__
#define __FF_SPI_H__

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include "ff_log.h"
#include "ff_core.h"

#define CONFIG_FINGERPRINT_FOCALTECH_SPI_SUPPORT
#define CONFIG_FINGERPRINT_FOCALTECH_SPICLK_SUPPORT

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
#if IS_ENABLED(CONFIG_TRUSTKERNEL_TEE_SUPPORT)
enum spi_cpol {
    SPI_CPOL_0,
    SPI_CPOL_1
};

enum spi_cpha {
    SPI_CPHA_0,
    SPI_CPHA_1
};

enum spi_mlsb {
    SPI_LSB,
    SPI_MSB
};

enum spi_endian {
    SPI_LENDIAN,
    SPI_BENDIAN
};

enum spi_transfer_mode {
    FIFO_TRANSFER,
    DMA_TRANSFER,
    OTHER1,
    OTHER2,
};

enum spi_pause_mode {
    PAUSE_MODE_DISABLE,
    PAUSE_MODE_ENABLE
};
enum spi_finish_intr {
    FINISH_INTR_DIS,
    FINISH_INTR_EN,
};

enum spi_deassert_mode {
    DEASSERT_DISABLE,
    DEASSERT_ENABLE
};

enum spi_ulthigh {
    ULTRA_HIGH_DISABLE,
    ULTRA_HIGH_ENABLE
};

enum spi_tckdly {
    TICK_DLY0,
    TICK_DLY1,
    TICK_DLY2,
    TICK_DLY3
};

enum spi_irq_flag {
    IRQ_IDLE,
    IRQ_BUSY
};

enum spi_sample_sel {
    POSITIVE_EDGE,
    NEGATIVE_EDGE

};

enum spi_cs_pol {
    ACTIVE_LOW,
    ACTIVE_HIGH
};

struct mt_chip_conf {
    unsigned int setuptime;
    unsigned int holdtime;
    unsigned int high_time;
    unsigned int low_time;
    unsigned int cs_idletime;
    unsigned int ulthgh_thrsh;
    enum spi_sample_sel sample_sel;
    enum spi_cs_pol cs_pol;
    enum spi_cpol cpol;
    enum spi_cpha cpha;
    enum spi_mlsb tx_mlsb;
    enum spi_mlsb rx_mlsb;
    enum spi_endian tx_endian;
    enum spi_endian rx_endian;
    enum spi_transfer_mode com_mod;
    enum spi_pause_mode pause;
    enum spi_finish_intr finish_intr;
    enum spi_deassert_mode deassert;
    enum spi_ulthigh ulthigh;
    enum spi_tckdly tckdly;
};
#endif

struct ff_spi_context {
    struct spi_device *spi;
    struct miscdevice mdev;
    struct mutex bus_lock;
    u8 *bus_tx_buf;
    u8 *bus_rx_buf;
    bool b_misc;
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
#ifdef CONFIG_FINGERPRINT_FOCALTECH_SPI_SUPPORT
int ff_spi_init(void);
void ff_spi_exit(void);
#endif

#endif /* __FF_SPI_H__ */
