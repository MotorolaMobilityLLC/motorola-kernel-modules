// SPDX-License-Identifier: GPL-2.0
/*
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
 * This file is the reference code of platform SPI bus module being used to
 * communicate with Synaptics TouchComm device over SPI.
 */

#include <linux/spi/spi.h>

#include "syna_tcm2.h"
#include "syna_tcm2_platform.h"

#if (KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE)
#define SPI_HAS_DELAY_USEC
#endif

#define SPI_MODULE_NAME "synaptics_tcm_spi"

#define XFER_ATTEMPTS 5

static struct platform_device *p_device;

static unsigned char *rx_buf;
static unsigned char *tx_buf;
static unsigned int buf_size;
static struct spi_transfer *xfer;


/*
 * Request and return the device pointer for managed
 *
 * param
 *     void.
 *
 * return
 *     a device pointer allocated previously
 */
#if defined(DEV_MANAGED_API)
struct device *syna_request_managed_device(void)
{
	if (!p_device)
		return NULL;

	return p_device->dev.parent;
}
#endif


/*
 * Release the GPIO.
 *
 * param
 *     [ in] gpio:   the target gpio
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_put_gpio(int gpio)
{
	/* release gpios */
	if (gpio <= 0) {
		LOGE("Invalid gpio pin\n");
		return -EINVAL;
	}

	gpio_free(gpio);
	LOGD("GPIO-%d released\n", gpio);

	return 0;
}
/*
 * Request a gpio and perform the requested setup
 *
 * param
 *    [ in] gpio:   the target gpio
 *    [ in] dir:    default direction of gpio
 *    [ in] state:  default state of gpio
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_get_gpio(int gpio, int dir, int state, char *label)
{
	int retval;

	if (gpio < 0) {
		LOGE("Invalid gpio pin\n");
		return -EINVAL;
	}

	retval = scnprintf(label, 16, "tcm_gpio_%d\n", gpio);
	if (retval < 0) {
		LOGE("Fail to set GPIO label\n");
		return retval;
	}

	retval = gpio_request(gpio, label);
	if (retval < 0) {
		LOGE("Fail to request GPIO %d\n", gpio);
		return retval;
	}

	if (dir == 0)
		retval = gpio_direction_input(gpio);
	else
		retval = gpio_direction_output(gpio, state);

	if (retval < 0) {
		LOGE("Fail to set GPIO %d direction\n", gpio);
		return retval;
	}

	LOGD("GPIO-%d requested\n", gpio);

	return 0;
}
/*
 * Release the regulator.
 *
 * param
 *    [ in] reg_dev: regulator to release
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_put_regulator(struct regulator *reg_dev)
{
	if (!reg_dev) {
		LOGE("Invalid regulator device\n");
		return -EINVAL;
	}
#ifdef DEV_MANAGED_API
	devm_regulator_put(reg_dev);
#else /* Legacy API */
	regulator_put(reg_dev);
#endif

	return 0;
}
/*
 * Requested a regulator according to the name.
 *
 * param
 *    [ in] name: name of requested regulator
 *
 * return
 *    on success, return the pointer to the requested regulator; otherwise, on error.
 */
static struct regulator *syna_spi_get_regulator(const char *name)
{
	struct regulator *reg_dev = NULL;
	struct device *dev = p_device->dev.parent;

	if (name != NULL && *name != 0) {
#ifdef DEV_MANAGED_API
		reg_dev = devm_regulator_get(dev, name);
#else /* Legacy API */
		reg_dev = regulator_get(dev, name);
#endif
		if (IS_ERR(reg_dev)) {
			LOGW("Regulator is not ready\n");
			return (struct regulator *)PTR_ERR(reg_dev);
		}
	}

	return reg_dev;
}
/*
 * Parse and obtain board specific data from the device tree source file.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] dev: device model
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
#ifdef CONFIG_OF
static int syna_spi_parse_dt(struct syna_hw_interface *hw_if,
		struct device *dev)
{
	int retval;
	struct property *prop;
	struct device_node *np = dev->of_node;
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;
	struct product_specific *product = &hw_if->product;
	int temp_value[5] = { 0 };

	attn->irq_gpio = -1;
	prop = of_find_property(np, "synaptics,irq-gpio", NULL);
	if (prop && prop->length)
		attn->irq_gpio = of_get_named_gpio(np, "synaptics,irq-gpio", 0);

	attn->irq_flags = (IRQF_ONESHOT | IRQF_TRIGGER_LOW);
	prop = of_find_property(np, "synaptics,irq-flags", NULL);
	if (prop && prop->length) {
		of_property_read_u32(np, "synaptics,irq-flags", (unsigned int *)&temp_value[0]);
		attn->irq_flags = temp_value[0];
	}

	attn->irq_on_state = 0;
	prop = of_find_property(np, "synaptics,irq-on-state", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,irq-on-state", &attn->irq_on_state);


	pwr->power_on_state = 1;
	prop = of_find_property(np, "synaptics,power-on-state", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,power-on-state", &pwr->power_on_state);

	pwr->power_delay_ms = 0;
	prop = of_find_property(np, "synaptics,power-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,power-delay-ms", &pwr->power_delay_ms);

	pwr->vdd.control = 0;
	prop = of_find_property(np, "synaptics,vdd-control", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vdd-control", &pwr->vdd.control);

	pwr->vdd.regulator_name = NULL;
	prop = of_find_property(np, "synaptics,vdd-name", NULL);
	if (prop && prop->length)
		of_property_read_string(np, "synaptics,vdd-name", &pwr->vdd.regulator_name);

	pwr->vdd.gpio = -1;
	prop = of_find_property(np, "synaptics,vdd-gpio", NULL);
	if (prop && prop->length)
		pwr->vdd.gpio = of_get_named_gpio(np, "synaptics,vdd-gpio", 0);

	pwr->vdd.power_on_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vdd-power-on-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vdd-power-on-delay-ms", &pwr->vdd.power_on_delay_ms);

	pwr->vdd.power_off_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vdd-power-off-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vdd-power-off-delay-ms", &pwr->vdd.power_off_delay_ms);

	pwr->vio.control = 0;
	prop = of_find_property(np, "synaptics,vio-control", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vio-control", &pwr->vio.control);

	pwr->vio.regulator_name = NULL;
	prop = of_find_property(np, "synaptics,vio-name", NULL);
	if (prop && prop->length)
		of_property_read_string(np, "synaptics,vio-name", &pwr->vio.regulator_name);

	pwr->vio.gpio = -1;
	prop = of_find_property(np, "synaptics,vio-gpio", NULL);
	if (prop && prop->length)
		pwr->vio.gpio = of_get_named_gpio(np, "synaptics,vio-gpio", 0);

	pwr->vio.power_on_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vio-power-on-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vio-power-on-delay-ms", &pwr->vio.power_on_delay_ms);

	pwr->vio.power_off_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vio-power-off-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,vio-power-off-delay-ms", &pwr->vio.power_off_delay_ms);


	rst->reset_on_state = 0;
	prop = of_find_property(np, "synaptics,reset-on-state", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,reset-on-state", &rst->reset_on_state);

	rst->reset_gpio = -1;
	prop = of_find_property(np, "synaptics,reset-gpio", NULL);
	if (prop && prop->length)
		rst->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);

	prop = of_find_property(np, "synaptics,reset-active-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,reset-active-ms", &rst->reset_active_ms);

	prop = of_find_property(np, "synaptics,reset-delay-ms", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,reset-delay-ms", &rst->reset_delay_ms);


	bus->switch_gpio = -1;
	prop = of_find_property(np, "synaptics,io-switch-gpio", NULL);
	if (prop && prop->length)
		bus->switch_gpio = of_get_named_gpio(np, "synaptics,io-switch-gpio", 0);

	prop = of_find_property(np, "synaptics,io-switch-state", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,io-switch-state", &bus->switch_state);


	bus->spi_byte_delay_us = 0;
	prop = of_find_property(np, "synaptics,spi-byte-delay-us", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,spi-byte-delay-us", &bus->spi_byte_delay_us);

	bus->spi_block_delay_us = 0;
	prop = of_find_property(np, "synaptics,spi-block-delay-us", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,spi-block-delay-us", &bus->spi_block_delay_us);

	bus->spi_mode = 0;
	prop = of_find_property(np, "synaptics,spi-mode", NULL);
	if (prop && prop->length)
		of_property_read_u32(np, "synaptics,spi-mode", &bus->spi_mode);


	prop = of_find_property(np, "synaptics,chunks", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32_array(np, "synaptics,chunks", temp_value, 2);
		if (retval >= 0) {
			hw_if->hw_platform.rd_chunk_size = temp_value[0];
			hw_if->hw_platform.wr_chunk_size = temp_value[1];
		}
	}

	prop = of_find_property(np, "synaptics,flash-access-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32_array(np, "synaptics,flash-access-delay-usy",
				temp_value, 3);
		if (retval >= 0) {
			product->default_flash_delay_us[0] = temp_value[0];
			product->default_flash_delay_us[1] = temp_value[1];
			product->default_flash_delay_us[2] = temp_value[2];
		}
	}

	prop = of_find_property(np, "synaptics,command-timeout-ms", NULL);
	if (prop && prop->length)
		retval = of_property_read_u32(np, "synaptics,command-timeout-ms",
				&product->default_cmd_timeout_ms);

	prop = of_find_property(np, "synaptics,command-polling-ms", NULL);
	if (prop && prop->length)
		retval = of_property_read_u32(np, "synaptics,command-polling-ms",
				&product->default_cmd_polling_ms);

	prop = of_find_property(np, "synaptics,command-turnaround-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32_array(np, "synaptics,command-turnaround-us",
				temp_value, 2);
		if (retval >= 0) {
			product->default_cmd_turnaround_us[0] = temp_value[0];
			product->default_cmd_turnaround_us[1] = temp_value[1];
		}
	}

	prop = of_find_property(np, "synaptics,command-retry-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32_array(np, "synaptics,command-retry-us",
				temp_value, 2);
		if (retval >= 0) {
			product->default_cmd_retry_us[0] = temp_value[0];
			product->default_cmd_retry_us[1] = temp_value[1];
		}
	}

	prop = of_find_property(np, "synaptics,fw-switch-delay-ms", NULL);
	if (prop && prop->length)
		retval = of_property_read_u32(np, "synaptics,fw-switch-delay-ms",
				&product->default_fw_switch_delay_ms);


	LOGI("Load from dt: chunk size(%d %d) reset (%d %d) vdd delay(%d %d) vio delay(%d %d)\n",
		hw_if->hw_platform.rd_chunk_size, hw_if->hw_platform.wr_chunk_size,
		rst->reset_active_ms, rst->reset_delay_ms, pwr->vdd.power_on_delay_ms,
		pwr->vdd.power_off_delay_ms, pwr->vio.power_on_delay_ms, pwr->vio.power_off_delay_ms);
	LOGI("Load from dt: command timeout(%d) turnaround time(%d %d) retry time(%d %d)\n",
		product->default_cmd_timeout_ms, product->default_cmd_turnaround_us[0],
		product->default_cmd_turnaround_us[1], product->default_cmd_retry_us[0],
		product->default_cmd_retry_us[1]);
	LOGI("Load from dt: flash erase(%d) flash write(%d) flash read(%d) fw switch(%d)\n",
		product->default_flash_delay_us[0], product->default_flash_delay_us[1],
		product->default_flash_delay_us[2], product->default_fw_switch_delay_ms);

	return 0;
}
#endif

/*
 * Release the resources for the use of ATTN.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_release_attn_resources(struct syna_hw_interface *hw_if)
{
	struct syna_hw_attn_data *attn;

	if (!hw_if)
		return -EINVAL;

	attn = &hw_if->bdata_attn;
	if (!attn)
		return -EINVAL;

	syna_pal_mutex_free(&attn->irq_en_mutex);

	if (attn->irq_gpio > 0)
		syna_spi_put_gpio(attn->irq_gpio);

	return 0;
}
/*
 * Initialize the resources for the use of ATTN.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_request_attn_resources(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_attn_gpio[32] = {0};
	struct syna_hw_attn_data *attn;

	if (!hw_if)
		return -EINVAL;

	attn = &hw_if->bdata_attn;
	if (!attn)
		return -EINVAL;

	syna_pal_mutex_alloc(&attn->irq_en_mutex);

	if (attn->irq_gpio > 0) {
		retval = syna_spi_get_gpio(attn->irq_gpio, 0, 0, str_attn_gpio);
		if (retval < 0) {
			LOGE("Fail to request GPIO %d for attention\n",
				attn->irq_gpio);
			return retval;
		}
	}

	return 0;
}
/*
 * Release the resources for the use of hardware reset.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_release_reset_resources(struct syna_hw_interface *hw_if)
{
	struct syna_hw_rst_data *rst;

	if (!hw_if)
		return -EINVAL;

	rst = &hw_if->bdata_rst;
	if (!rst)
		return -EINVAL;

	if (rst->reset_gpio > 0)
		syna_spi_put_gpio(rst->reset_gpio);

	return 0;
}
/*
 * Initialize the resources for the use of hardware reset.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_request_reset_resources(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_rst_gpio[32] = {0};
	struct syna_hw_rst_data *rst;

	if (!hw_if)
		return -EINVAL;

	rst = &hw_if->bdata_rst;
	if (!rst)
		return -EINVAL;

	if (rst->reset_gpio > 0) {
		retval = syna_spi_get_gpio(rst->reset_gpio, 1,
				rst->reset_on_state, str_rst_gpio);
		if (retval < 0) {
			LOGE("Fail to request GPIO %d for reset\n",
				rst->reset_gpio);
			return retval;
		}
	}

	return 0;
}
/*
 * Release the resources for the use of bus transferring.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_release_bus_resources(struct syna_hw_interface *hw_if)
{
	struct syna_hw_bus_data *bus;

	if (!hw_if)
		return -EINVAL;

	bus = &hw_if->bdata_io;
	if (!bus)
		return -EINVAL;

	syna_pal_mutex_free(&bus->io_mutex);

	if (bus->switch_gpio > 0)
		syna_spi_put_gpio(bus->switch_gpio);

	if (rx_buf) {
		syna_pal_mem_free((void *)rx_buf);
		rx_buf = NULL;
	}

	if (tx_buf) {
		syna_pal_mem_free((void *)tx_buf);
		tx_buf = NULL;
	}

	if (xfer) {
		syna_pal_mem_free((void *)xfer);
		xfer = NULL;
	}

	return 0;
}
/*
 * Initialize the resources for the use of bus transferring.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_request_bus_resources(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_switch_gpio[32] = {0};
	struct syna_hw_bus_data *bus;
	struct spi_device *spi;

	if (!hw_if)
		return -EINVAL;

	bus = &hw_if->bdata_io;
	if (!bus)
		return -EINVAL;

	spi = (struct spi_device *)hw_if->pdev;
	if (!spi)
		return -EINVAL;

	syna_pal_mutex_alloc(&bus->io_mutex);

	spi->bits_per_word = 8;
	switch (bus->spi_mode) {
	case 0:
		spi->mode = SPI_MODE_0;
		break;
	case 1:
		spi->mode = SPI_MODE_1;
		break;
	case 2:
		spi->mode = SPI_MODE_2;
		break;
	case 3:
		spi->mode = SPI_MODE_3;
		break;
	}
	retval = spi_setup(spi);
	if (retval < 0) {
		LOGE("Fail to set up SPI protocol driver\n");
		return retval;
	}

	if (bus->switch_gpio > 0) {
		retval = syna_spi_get_gpio(bus->switch_gpio, 1,
				bus->switch_state, str_switch_gpio);
		if (retval < 0) {
			LOGE("Fail to request GPIO %d for io switch\n", bus->switch_gpio);
			return retval;
		}
	}

	return 0;
}
/*
 * Release the resources for the use of power control.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_release_power_resources(struct syna_hw_interface *hw_if)
{
	struct syna_hw_pwr_data *pwr;

	if (!hw_if)
		return -EINVAL;

	pwr = &hw_if->bdata_pwr;
	if (!pwr)
		return -EINVAL;

	/* release power resource for vio */
	if (pwr->vio.control == PSU_REGULATOR) {
		if (pwr->vio.regulator_dev)
			syna_spi_put_regulator(pwr->vio.regulator_dev);
	} else if (pwr->vio.control > 0) {
		if (pwr->vio.gpio > 0)
			syna_spi_put_gpio(pwr->vio.gpio);
	}
	/* release power resource for VDD */
	if (pwr->vdd.control == PSU_REGULATOR) {
		if (pwr->vdd.regulator_dev)
			syna_spi_put_regulator(pwr->vdd.regulator_dev);
	} else if (pwr->vdd.control > 0) {
		if (pwr->vdd.gpio > 0)
			syna_spi_put_gpio(pwr->vdd.gpio);
	}

	return 0;
}
/*
 * Initialize the resources for the use of power control.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *    0 in case of success, a negative value otherwise.
 */
static int syna_spi_request_power_resources(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_vdd_gpio[32] = {0};
	static char str_avdd_gpio[32] = {0};
	struct syna_hw_pwr_data *pwr;

	if (!hw_if)
		return -EINVAL;

	pwr = &hw_if->bdata_pwr;
	if (!pwr)
		return -EINVAL;

	if (pwr->vdd.control == 0) {
		if (pwr->vdd.regulator_name && (strlen(pwr->vdd.regulator_name) > 0))
			pwr->vdd.control = PSU_REGULATOR;
	}
	if (pwr->vio.control == 0) {
		if (pwr->vio.regulator_name && (strlen(pwr->vio.regulator_name) > 0))
			pwr->vio.control = PSU_REGULATOR;
	}

	/* request power resource for  VDD */
	if (pwr->vdd.control == PSU_REGULATOR) {
		if (!pwr->vdd.regulator_name || (strlen(pwr->vdd.regulator_name) <= 0)) {
			LOGE("Fail to get regulator for vdd, no given name of vdd\n");
			return -ENXIO;
		}
		pwr->vdd.regulator_dev = syna_spi_get_regulator(pwr->vdd.regulator_name);
		if (IS_ERR((struct regulator *)pwr->vdd.regulator_dev)) {
			LOGE("Fail to request regulator for vdd\n");
			return -ENXIO;
		}
	} else if (pwr->vdd.control > 0) {
		if (pwr->vdd.gpio > 0) {
			retval = syna_spi_get_gpio(pwr->vdd.gpio, 1, !pwr->power_on_state,
					str_avdd_gpio);
			if (retval < 0) {
				LOGE("Fail to request GPIO %d for vdd\n", pwr->vdd.gpio);
				return retval;
			}
		}
	}
	/* request power resource for VIO */
	if (pwr->vio.control == PSU_REGULATOR) {
		if (!pwr->vio.regulator_name || (strlen(pwr->vio.regulator_name) <= 0)) {
			LOGE("Fail to get regulator for vio, no given name of vio\n");
			return -ENXIO;
		}
		pwr->vio.regulator_dev = syna_spi_get_regulator(pwr->vio.regulator_name);
		if (IS_ERR((struct regulator *)pwr->vio.regulator_dev)) {
			LOGE("Fail to configure regulator for vio\n");
			return -ENXIO;
		}
	} else if (pwr->vio.control > 0)  {
		if (pwr->vio.gpio > 0) {
			retval = syna_spi_get_gpio(pwr->vio.gpio, 1, !pwr->power_on_state,
					str_vdd_gpio);
			if (retval < 0) {
				LOGE("Fail to request GPIO %d for vio\n", pwr->vio.gpio);
				return retval;
			}
		}
	}

	return 0;
}


/*
 * Enable or disable the kernel irq.
 *
 * param
 *    [ in] hw:    the handle of abstracted hardware interface
 *    [ in] en:    '1' for enabling, and '0' for disabling
 *
 * return
 *   0 in case of nothing changed, positive value in case of success, a negative value otherwise.
 */
static int syna_spi_enable_irq(struct tcm_hw_platform *hw, bool en)
{
	int retval = 0;
	struct syna_hw_interface *hw_if = (struct syna_hw_interface *)hw->device;
	struct syna_hw_attn_data *attn;

	if (!hw_if) {
		LOGE("Invalid handle of hw_if\n");
		return -ENXIO;
	}

	attn = &hw_if->bdata_attn;
	if (!attn || (attn->irq_id == 0))
		return -ENXIO;

	syna_pal_mutex_lock(&attn->irq_en_mutex);

	/* enable the handling of interrupt */
	if (en) {
		if (attn->irq_enabled) {
			LOGD("Interrupt already enabled\n");
			goto exit;
		}

		enable_irq(attn->irq_id);
		attn->irq_enabled = true;
		retval = 1;
		LOGD("Interrupt enabled\n");
	}
	/* disable the handling of interrupt */
	else {
		if (!attn->irq_enabled) {
			LOGD("Interrupt already disabled\n");
			goto exit;
		}

		disable_irq_nosync(attn->irq_id);
		attn->irq_enabled = false;
		retval = 1;
		LOGD("Interrupt disabled\n");
	}

exit:
	syna_pal_mutex_unlock(&attn->irq_en_mutex);

	return retval;
}
/*
 * Toggle the hardware gpio pin to perform the chip reset.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *
 * return
 *     void.
 */
static void syna_spi_hw_reset(struct syna_hw_interface *hw_if)
{
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;

	if (rst->reset_gpio == 0)
		return;

	LOGD("Prepare to toggle reset, hold:%d delay:%d\n",
		rst->reset_active_ms, rst->reset_delay_ms);

	gpio_set_value(rst->reset_gpio, (rst->reset_on_state & 0x01));
	syna_pal_sleep_ms(rst->reset_active_ms);
	gpio_set_value(rst->reset_gpio, ((!rst->reset_on_state) & 0x01));
	syna_pal_sleep_ms(rst->reset_delay_ms);

	LOGD("Reset done\n");

}
/*
 * Power on touch controller through regulators or gpios for PWM.
 *
 * param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] on:    '1' for powering on, and '0' for powering off
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_spi_power_on(struct syna_hw_interface *hw_if, bool on)
{
	int retval = 0;
	struct syna_hw_pwr_data *pwr;

	pwr = &hw_if->bdata_pwr;
	if (!pwr)
		return -EINVAL;

	LOGD("Prepare to %s power ...\n", (on) ? "enable" : "disable");

	if (on) {
		if (pwr->vdd.control > 0) {
			/* power on VDD */
			if (pwr->vdd.control == PSU_REGULATOR) {
				if (IS_ERR((struct regulator *)pwr->vdd.regulator_dev)) {
					LOGE("Invalid regulator for vdd\n");
					goto exit;
				}
				retval = regulator_enable((struct regulator *)pwr->vdd.regulator_dev);
				if (retval < 0) {
					LOGE("Fail to enable vdd regulator\n");
					goto exit;
				}
			} else {
				if (pwr->vdd.gpio > 0)
					gpio_set_value(pwr->vdd.gpio, pwr->power_on_state);
			}

			if (pwr->vdd.power_on_delay_ms > 0)
				syna_pal_sleep_ms(pwr->vdd.power_on_delay_ms);
		}

		if (pwr->vio.control > 0) {
			/* power on VIO */
			if (pwr->vio.control == PSU_REGULATOR) {
				if (IS_ERR((struct regulator *)pwr->vio.regulator_dev)) {
					LOGE("Invalid regulator for vio\n");
					goto exit;
				}
				retval = regulator_enable((struct regulator *)pwr->vio.regulator_dev);
				if (retval < 0) {
					LOGE("Fail to enable vio regulator\n");
					goto exit;
				}
			} else {
				if (pwr->vio.gpio > 0)
					gpio_set_value(pwr->vio.gpio, pwr->power_on_state);
			}

			if (pwr->vio.power_on_delay_ms > 0)
				syna_pal_sleep_ms(pwr->vio.power_on_delay_ms);
		}
	} else {
		if (pwr->vio.control > 0) {
			/* power off VIO */
			if (pwr->vio.control == PSU_REGULATOR) {
				regulator_disable((struct regulator *)pwr->vio.regulator_dev);
			} else {
				if (pwr->vio.gpio > 0)
					gpio_set_value(pwr->vio.gpio, !pwr->power_on_state);
			}

			if (pwr->vio.power_off_delay_ms > 0)
				syna_pal_sleep_ms(pwr->vio.power_off_delay_ms);
		}
		if (pwr->vdd.control > 0) {
			/* power off VDD */
			if (pwr->vdd.control == PSU_REGULATOR) {
				regulator_disable((struct regulator *)pwr->vdd.regulator_dev);
			} else {
				if (pwr->vdd.gpio > 0)
					gpio_set_value(pwr->vdd.gpio, !pwr->power_on_state);
			}

			if (pwr->vdd.power_off_delay_ms > 0)
				syna_pal_sleep_ms(pwr->vdd.power_off_delay_ms);
		}
	}

	LOGI("Device power %s\n", (on) ? "On" : "Off");

exit:
	return retval;
}
/*
 * Allocate the buffers for SPI transferring.
 *
 * param
 *    [ in] count: number of spi_transfer structures to send
 *    [ in] size:  size of temporary buffer
 *
 * return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_alloc_mem(unsigned int count, unsigned int size)
{
	static unsigned int xfer_count;

	if (count > xfer_count) {
		syna_pal_mem_free((void *)xfer);
		xfer = syna_pal_mem_alloc(count, sizeof(*xfer));
		if (!xfer) {
			LOGE("Fail to allocate memory for xfer\n");
			xfer_count = 0;
			return -ENOMEM;
		}
		xfer_count = count;
	} else {
		syna_pal_mem_set(xfer, 0, count * sizeof(*xfer));
	}

	if (size > buf_size) {
		if (rx_buf) {
			syna_pal_mem_free((void *)rx_buf);
			rx_buf = NULL;
		}
		if (tx_buf) {
			syna_pal_mem_free((void *)tx_buf);
			tx_buf = NULL;
		}

		rx_buf = syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!rx_buf) {
			LOGE("Fail to allocate memory for rx_buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		tx_buf = syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!tx_buf) {
			LOGE("Fail to allocate memory for tx_buf\n");
			buf_size = 0;
			return -ENOMEM;
		}

		buf_size = size;
	}

	return 0;
}

/*
 * Implement the SPI transaction to read out data over SPI bus.
 *
 * param
 *    [ in] hw:      the handle of abstracted hardware interface
 *    [out] rd_data: buffer for storing data retrieved from device
 *    [ in] rd_len:  number of bytes retrieved from device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_spi_read(struct tcm_hw_platform *hw, unsigned char *rd_data,
	unsigned int rd_len)
{
	int retval;
	unsigned int idx;
	struct syna_hw_interface *hw_if = (struct syna_hw_interface *)hw->device;
	struct spi_message msg;
	struct spi_device *spi;
	struct syna_hw_bus_data *bus;

	if (!hw_if) {
		LOGE("Invalid handle of hw_if\n");
		return -ENXIO;
	}

	spi = hw_if->pdev;
	bus = &hw_if->bdata_io;
	if (!spi || !bus) {
		LOGE("Invalid bus io device\n");
		return -ENXIO;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	if ((rd_len & 0xffff) == 0xffff) {
		LOGE("Invalid read length 0x%X\n", (rd_len & 0xffff));
		retval = -EINVAL;
		goto exit;
	}

	spi_message_init(&msg);

	if (bus->spi_byte_delay_us == 0)
		retval = syna_spi_alloc_mem(1, rd_len);
	else
		retval = syna_spi_alloc_mem(rd_len, rd_len);
	if (retval < 0) {
		LOGE("Fail to allocate memory\n");
		goto exit;
	}

	if (bus->spi_byte_delay_us == 0) {
		syna_pal_mem_set(tx_buf, 0xff, rd_len);
		xfer[0].len = rd_len;
		xfer[0].tx_buf = tx_buf;
		xfer[0].rx_buf = rx_buf;
#ifdef SPI_HAS_DELAY_USEC
		if (bus->spi_block_delay_us)
			xfer[0].delay_usecs = bus->spi_block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		tx_buf[0] = 0xff;
		for (idx = 0; idx < rd_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = tx_buf;
			xfer[idx].rx_buf = &rx_buf[idx];
#ifdef SPI_HAS_DELAY_USEC
			xfer[idx].delay_usecs = bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == rd_len - 1))
				xfer[idx].delay_usecs = bus->spi_block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval != 0) {
		LOGE("Failed to complete SPI transfer, error = %d\n", retval);
		goto exit;
	}
	retval = syna_pal_mem_cpy(rd_data, rd_len, rx_buf, rd_len, rd_len);
	if (retval < 0) {
		LOGE("Fail to copy rx_buf to rd_data\n");
		goto exit;
	}

	retval = rd_len;

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}

/*
 * Implement the SPI transaction to write data over SPI bus.
 *
 * param
 *    [ in] hw:      the handle of abstracted hardware interface
 *    [ in] wr_data: written data
 *    [ in] wr_len:  length of written data in bytes
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_spi_write(struct tcm_hw_platform *hw, unsigned char *wr_data,
	unsigned int wr_len)
{
	int retval;
	unsigned int idx;
	struct syna_hw_interface *hw_if = (struct syna_hw_interface *)hw->device;
	struct spi_message msg;
	struct spi_device *spi;
	struct syna_hw_bus_data *bus;

	if (!hw_if) {
		LOGE("Invalid handle of hw_if\n");
		return -ENXIO;
	}

	spi = hw_if->pdev;
	bus = &hw_if->bdata_io;
	if (!spi || !bus) {
		LOGE("Invalid bus io device\n");
		return -ENXIO;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	if ((wr_len & 0xffff) == 0xffff) {
		LOGE("Invalid write length 0x%X\n", (wr_len & 0xffff));
		retval = -EINVAL;
		goto exit;
	}

	spi_message_init(&msg);

	if (bus->spi_byte_delay_us == 0)
		retval = syna_spi_alloc_mem(1, wr_len);
	else
		retval = syna_spi_alloc_mem(wr_len, wr_len);
	if (retval < 0) {
		LOGE("Failed to allocate memory\n");
		goto exit;
	}

	retval = syna_pal_mem_cpy(tx_buf, wr_len, wr_data, wr_len, wr_len);
	if (retval < 0) {
		LOGE("Fail to copy wr_data to tx_buf\n");
		goto exit;
	}

	if (bus->spi_byte_delay_us == 0) {
		xfer[0].len = wr_len;
		xfer[0].tx_buf = tx_buf;
#ifdef SPI_HAS_DELAY_USEC
		if (bus->spi_block_delay_us)
			xfer[0].delay_usecs = bus->spi_block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < wr_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &tx_buf[idx];
#ifdef SPI_HAS_DELAY_USEC
			xfer[idx].delay_usecs = bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == wr_len - 1))
				xfer[idx].delay_usecs = bus->spi_block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval != 0) {
		LOGE("Fail to complete SPI transfer, error = %d\n", retval);
		goto exit;
	}

	retval = wr_len;

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}

/* An example of hardware settings for an SPI device */
static struct syna_hw_interface syna_spi_hw_if = {
	.hw_platform = {
		.type = BUS_TYPE_SPI,
		.rd_chunk_size = RD_CHUNK_SIZE,
		.wr_chunk_size = WR_CHUNK_SIZE,
		.ops_read_data = syna_spi_read,
		.ops_write_data = syna_spi_write,
		.ops_enable_attn = syna_spi_enable_irq,
		.support_attn = true,
#ifdef DATA_ALIGNMENT
		.alignment_base = ALIGNMENT_BASE,
		.alignment_boundary = ALIGNMENT_SIZE_BOUNDARY,
#endif
	},
	.ops_power_on = syna_spi_power_on,
	.ops_hw_reset = syna_spi_hw_reset,
};

/*
 * Probe and register the platform spi device.
 *
 * param
 *    [ in] spi: spi device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
static int syna_spi_probe(struct spi_device *spi)
{
	int retval;

#ifdef CONFIG_OF
	syna_spi_parse_dt(&syna_spi_hw_if, &spi->dev);
#endif

	/* keep the i/o device */
	syna_spi_hw_if.pdev = spi;

	p_device->dev.parent = &spi->dev;
	p_device->dev.platform_data = &syna_spi_hw_if;

	syna_spi_hw_if.hw_platform.device = &syna_spi_hw_if;

	/* initialize resources for the use of power */
	retval = syna_spi_request_power_resources(&syna_spi_hw_if);
	if (retval < 0) {
		LOGE("Fail to request resources for power\n");
		return retval;
	}

	/* initialize resources for the use of bus transferring */
	retval = syna_spi_request_bus_resources(&syna_spi_hw_if);
	if (retval < 0) {
		LOGE("Fail to request resources for bus\n");
		syna_spi_release_power_resources(&syna_spi_hw_if);
		return retval;
	}

	/* initialize resources for the use of reset */
	retval = syna_spi_request_reset_resources(&syna_spi_hw_if);
	if (retval < 0) {
		LOGE("Fail to request resources for reset\n");
		syna_spi_release_bus_resources(&syna_spi_hw_if);
		syna_spi_release_power_resources(&syna_spi_hw_if);
		return retval;
	}

	/* initialize resources for the use of attn */
	retval = syna_spi_request_attn_resources(&syna_spi_hw_if);
	if (retval < 0) {
		LOGE("Fail to request resources for attn\n");
		syna_spi_release_reset_resources(&syna_spi_hw_if);
		syna_spi_release_bus_resources(&syna_spi_hw_if);
		syna_spi_release_power_resources(&syna_spi_hw_if);
		return retval;
	}

	/* register the platform device */
	retval = platform_device_register(p_device);
	if (retval < 0) {
		LOGE("Fail to register platform device\n");
		return retval;
	}

	return 0;
}

/*
 * Unregister the platform spi device.
 *
 * param
 *    [ in] spi: spi device
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
#if (KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE)
static void syna_spi_remove(struct spi_device *spi)
#else
static int syna_spi_remove(struct spi_device *spi)
#endif
{
	/* release resources */
	syna_spi_release_attn_resources(&syna_spi_hw_if);
	syna_spi_release_reset_resources(&syna_spi_hw_if);
	syna_spi_release_bus_resources(&syna_spi_hw_if);
	syna_spi_release_power_resources(&syna_spi_hw_if);

#if (KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE)
	return;
#else
	return 0;
#endif
}
/*
 * Release the platform SPI device.
 *
 * param
 *    [ in] dev: pointer to device
 *
 * return
 *    none
 */
static void syna_spi_release(struct device *dev)
{
	LOGI("SPI device removed\n");
}

/* Example of an spi device driver */
static const struct spi_device_id syna_spi_id_table[] = {
	{SPI_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(spi, syna_spi_id_table);

#ifdef CONFIG_OF
static const struct of_device_id syna_spi_of_match_table[] = {
	{
		.compatible = "synaptics,tcm-spi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, syna_spi_of_match_table);
#else
#define syna_spi_of_match_table NULL
#endif

static struct spi_driver syna_spi_driver = {
	.driver = {
		.name = SPI_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = syna_spi_of_match_table,
	},
	.probe = syna_spi_probe,
	.remove = syna_spi_remove,
	.id_table = syna_spi_id_table,
};

/* Example of a platform device */
static struct platform_device syna_spi_device = {
	.name = PLATFORM_DRIVER_NAME,
	.dev = {
		.release = syna_spi_release,
	}
};


/*
 * Initialize the hardware module.
 *
 * param
 *    void
 *
 * return
 *    0 or positive value in case of success, a negative value otherwise.
 */
int syna_hw_interface_init(void)
{
	int retval;

	p_device = &syna_spi_device;

	/* register the spi driver */
	retval = spi_register_driver(&syna_spi_driver);
	if (retval < 0) {
		LOGE("Fail to add spi driver\n");
		return retval;
	}

	buf_size = 0;
	rx_buf = NULL;
	tx_buf = NULL;

	return retval;
}

/*
 * Delete the hardware module.
 *
 * param
 *    void
 *
 * return
 *    void.
 */
void syna_hw_interface_exit(void)
{
	/* unregister the spi driver */
	spi_unregister_driver(&syna_spi_driver);

	/* unregister the platform device */
	platform_device_unregister(&syna_spi_device);
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TouchCom SPI Bus Module");
MODULE_LICENSE("GPL v2");

