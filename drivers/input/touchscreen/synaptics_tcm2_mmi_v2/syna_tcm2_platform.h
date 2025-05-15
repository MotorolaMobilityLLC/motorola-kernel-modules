/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2024 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/*
 * This file declares the platform-specific or hardware relevant data.
 */

#ifndef _SYNAPTICS_TCM2_PLATFORM_H_
#define _SYNAPTICS_TCM2_PLATFORM_H_

#include "tcm/synaptics_touchcom_platform.h"
#include "syna_tcm2_runtime.h"

/*
 * Capability of bus transferred
 *
 *    RD_CHUNK_SIZE: the max. transferred size for one 'read' operation
 *    WR_CHUNK_SIZE: the max. transferred size for one 'write' operation
 */
#define RD_CHUNK_SIZE (1024)
#define WR_CHUNK_SIZE (1024)

/* Type of power supply */
enum power_supply {
	PSU_REGULATOR = 1,
	PSU_GPIO,
};


/*
 * Definitions of hardware interface
 */

/* Hardware Data for bus transferred */
struct syna_hw_bus_data {
	/* clock frequency in hz */
	unsigned int frequency_hz;
	/* parameters for i2c */
	unsigned int i2c_addr;
	/* parameters for spi */
	unsigned int spi_mode;
	unsigned int spi_byte_delay_us;
	unsigned int spi_block_delay_us;
	/* mutex to protect the i/o */
	syna_pal_mutex_t io_mutex;
	/* parameters for io switch */
	int switch_gpio;
	int switch_state;
};

/* Hardware Data for ATTN signal */
struct syna_hw_attn_data {
	/* parameters */
	int irq_gpio;
	int irq_on_state;
	unsigned long irq_flags;
	int irq_id;
	bool irq_enabled;
	/* mutex to protect the irq control */
	syna_pal_mutex_t irq_en_mutex;
};

/* Hardware Data for RST_N pin */
struct syna_hw_rst_data {
	/* parameters */
	int reset_gpio;
	int reset_on_state;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
};

/* Hardware Data for power control */
struct power_setup {
	int control;
	const char *regulator_name;
	void *regulator_dev;
	int gpio;
	int voltage;
	unsigned int power_on_delay_ms;
	unsigned int power_off_delay_ms;
};
struct syna_hw_pwr_data {
	/* VDD */
	struct power_setup vdd;
	/* IO VDD */
	struct power_setup vio;
	/* indicate the state of powering on */
	int power_on_state;
	/* the delay time after the completion of power sequence */
	unsigned int power_delay_ms;
};

/* Product specific data */
struct product_specific {
	/* time settings for command processing */
	int default_cmd_timeout_ms;
	int default_cmd_polling_ms;
	int default_cmd_turnaround_us[2];
	int default_cmd_retry_us[2];
	/* time settings for flash operations */
	int default_fw_switch_delay_ms;
	/* time settings for flash operations */
	int default_flash_delay_us[3];
};

/* Abstractions of hardware-specific interface */
struct syna_hw_interface {
	/* The handle of target platform */
	void *pdev;

	/* Hardware abstraction interface linked to tcm/ core lib. */
	struct tcm_hw_platform hw_platform;

	/* Hardware resources */
	struct syna_hw_bus_data bdata_io;
	struct syna_hw_attn_data bdata_attn;
	struct syna_hw_rst_data bdata_rst;
	struct syna_hw_pwr_data bdata_pwr;

	/* Product specific data */
	struct product_specific product;

	/* Implementation of power on/off operation */
	int (*ops_power_on)(struct syna_hw_interface *hw_if, bool on);

	/* Implementation of hardware reset operation */
	void (*ops_hw_reset)(struct syna_hw_interface *hw_if);
};


/*
 * Common Helpers
 */

/*
 * Initialize the hardware module.
 *
 * param
 *    void
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_hw_interface_init(void);

/*
 * Delete the hardware module.
 *
 * param
 *    void
 *
 * return
 *    void.
 */
void syna_hw_interface_exit(void);


#endif /* end of _SYNAPTICS_TCM2_PLATFORM_H_ */
