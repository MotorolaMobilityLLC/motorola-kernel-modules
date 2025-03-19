/*
 * omnivision TCM touchscreen driver

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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include "omnivision_tcm_core.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
#include <linux/platform_data/spi-mt65xx.h>
#define CONFIG_OVT_MTK_SPI_TIME
#endif

static unsigned char *buf;

static unsigned int buf_size;

static struct spi_transfer *xfer;

static struct ovt_tcm_bus_io bus_io;

static struct ovt_tcm_hw_interface hw_if;

static struct platform_device *ovt_tcm_spi_device;
/*
static struct pinctrl *pinctrl;
static struct pinctrl_state *pin_spi_mode_default;
*/

#ifdef CONFIG_OVT_MTK_SPI_TIME
const struct mtk_chip_config st_spi_ctrdata = {
       .sample_sel = 0,
       .cs_setuptime = 55,
       .cs_holdtime = 0,
       .cs_idletime = 0,
       .tick_delay = 0,
};
#endif

#ifdef OVT_MTK_CHECK_PANEL
const char *active_panel_name;
static void ovt_get_active_panel(void)
{
	int rc;
	struct device_node *chosen = of_find_node_by_name(NULL, "chosen");

	if(chosen) {
		rc = of_property_read_string(chosen, "mmi,panel_name", (const char **)&active_panel_name);
		if (rc)
			OVT_INFO("mmi,panel_name null\n");
		else
			OVT_DEBUG("active_panel=%s\n", active_panel_name);
	}
	else
		OVT_INFO("chosen node null\n");

}

static int ovt_get_panel(void)
{
	OVT_DEBUG("enter");
	ovt_get_active_panel();

	if (!active_panel_name)
		OVT_INFO("active_panel NULL\n");
	else if(strstr(active_panel_name, "_td"))
	{
		OVT_INFO("matched active_panel: %s ", active_panel_name);
		return 0;
	}
	else
		OVT_INFO("unmacth active_panel: %s\n", active_panel_name);

	return -1;
}
#endif

#ifdef CONFIG_DRM_CHECK_DT
static struct drm_panel *active_tcm_panel;

struct drm_panel *tcm_get_panel(void)
{
	return active_tcm_panel;
}

EXPORT_SYMBOL(tcm_get_panel);

static int ovt_tcm_check_dt(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	printk("%s, enter\n", __func__);
	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			printk("%s, active_tcm_panel find ok\n", __func__);
			active_tcm_panel = panel;
			return 0;
		}
	}
	printk("%s, find panel error exit\n", __func__);
	return PTR_ERR(panel);
}
#endif
#ifdef CONFIG_OF
static int parse_dt(struct device *dev, struct ovt_tcm_board_data *bdata)
{
	int retval;
	u32 value;
	struct property *prop;
	struct device_node *np = dev->of_node;
	const char *name;

	OVT_FUNC_ENTER();

	prop = of_find_property(np, "omnivision,irq-gpio", NULL);
	if (prop && prop->length) {
		bdata->irq_gpio = of_get_named_gpio(np,
				"omnivision,irq-gpio", 0);
		OVT_INFO("irq-gpio get\n");
	} else {
		bdata->irq_gpio = -1;
		OVT_INFO("irq-gpio get fail, set as -1\n");
	}

#ifdef OVT_DOUBLE_TAP_CTRL
	retval = of_property_read_u32(np, "omnivision,supported_gesture_type", &value);
	if (!retval) {
		bdata->supported_gesture_type = (uint8_t)value;
		OVT_INFO("omnivision,supported_gesture_type=%d\n", bdata->supported_gesture_type);
	}
	else
		OVT_INFO("omnivision,supported_gesture_type not set\n");
#endif

	retval = of_property_read_u32(np, "omnivision,irq-on-state", &value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	retval = of_property_read_string(np, "omnivision,pwr-reg-name", &name);
	if (retval < 0)
		bdata->pwr_reg_name = NULL;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "omnivision,bus-reg-name", &name);
	if (retval < 0)
		bdata->bus_reg_name = NULL;
	else
		bdata->bus_reg_name = name;

	prop = of_find_property(np, "omnivision,power-gpio", NULL);
	if (prop && prop->length) {
		bdata->power_gpio = of_get_named_gpio(np,
				"omnivision,power-gpio", 0);
	} else {
		bdata->power_gpio = -1;
		OVT_DEBUG("power-gpio not set\n");
	}

	prop = of_find_property(np, "omnivision,power-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,power-on-state",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read omnivision,power-on-state property\n");
			return retval;
		} else {
			bdata->power_on_state = value;
		}
	} else {
		bdata->power_on_state = 0;
	}

	prop = of_find_property(np, "omnivision,power-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,power-delay-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read omnivision,power-delay-ms property\n");
			return retval;
		} else {
			bdata->power_delay_ms = value;
		}
	} else {
		bdata->power_delay_ms = 0;
	}

	prop = of_find_property(np, "omnivision,reset-gpio", NULL);
	if (prop && prop->length) {
		bdata->reset_gpio = of_get_named_gpio(np,
				"omnivision,reset-gpio", 0);
		OVT_INFO("reset-gpio get\n");
	} else {
		bdata->reset_gpio = -1;
		OVT_INFO("reset-gpio get fail\n");
	}

	prop = of_find_property(np, "omnivision,reset-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,reset-on-state",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read omnivision,reset-on-state property\n");
			return retval;
		} else {
			bdata->reset_on_state = value;
		}
	} else {
		bdata->reset_on_state = 0;
	}

	prop = of_find_property(np, "omnivision,reset-active-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,reset-active-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read omnivision,reset-active-ms property\n");
			return retval;
		} else {
			bdata->reset_active_ms = value;
		}
	} else {
		bdata->reset_active_ms = 0;
	}

	prop = of_find_property(np, "omnivision,reset-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,reset-delay-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,reset-delay-ms property\n");
			return retval;
		} else {
			bdata->reset_delay_ms = value;
		}
	} else {
		bdata->reset_delay_ms = 0;
	}

	prop = of_find_property(np, "omnivision,tpio-reset-gpio", NULL);
	if (prop && prop->length) {
		bdata->tpio_reset_gpio = of_get_named_gpio(np,
				"omnivision,tpio-reset-gpio", 0);
	} else {
		bdata->tpio_reset_gpio = -1;
		OVT_DEBUG("tpio-reset-gpio not set\n");
	}

	prop = of_find_property(np, "omnivision,x-flip", NULL);
	bdata->x_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "omnivision,y-flip", NULL);
	bdata->y_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "omnivision,swap-axes", NULL);
	bdata->swap_axes = prop > 0 ? true : false;

	prop = of_find_property(np, "omnivision,byte-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,byte-delay-us",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,byte-delay-us property\n");
			return retval;
		} else {
			bdata->byte_delay_us = value;
		}
	} else {
		bdata->byte_delay_us = 0;
	}

	prop = of_find_property(np, "omnivision,block-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,block-delay-us",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,block-delay-us property\n");
			return retval;
		} else {
			bdata->block_delay_us = value;
		}
	} else {
		bdata->block_delay_us = 0;
	}

	prop = of_find_property(np, "omnivision,spi-mode", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,spi-mode",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,spi-mode property\n");
			return retval;
		} else {
			bdata->spi_mode = value;
		}
	} else {
		bdata->spi_mode = 0;
	}

	prop = of_find_property(np, "omnivision,ubl-max-freq", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,ubl-max-freq",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,ubl-max-freq property\n");
			return retval;
		} else {
			bdata->ubl_max_freq = value;
		}
	} else {
		bdata->ubl_max_freq = 0;
	}

	prop = of_find_property(np, "omnivision,ubl-byte-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "omnivision,ubl-byte-delay-us",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read omnivision,ubl-byte-delay-us property\n");
			return retval;
		} else {
			bdata->ubl_byte_delay_us = value;
		}
	} else {
		bdata->ubl_byte_delay_us = 0;
	}

	OVT_FUNC_EXIT();
	return 0;
}
#endif

static int ovt_tcm_spi_alloc_mem(struct ovt_tcm_hcd *tcm_hcd,
		unsigned int count, unsigned int size)
{
	static unsigned int xfer_count;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);

	if (count > xfer_count) {
		kfree(xfer);
		xfer = kcalloc(count, sizeof(*xfer), GFP_KERNEL);
		if (!xfer) {
			LOGE(&spi->dev,
					"Failed to allocate memory for xfer\n");
			xfer_count = 0;
			return -ENOMEM;
		}
		xfer_count = count;
	} else {
		memset(xfer, 0, count * sizeof(*xfer));
	}

	if (size > buf_size) {
		if (buf_size)
			kfree(buf);
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			LOGE(&spi->dev,
					"Failed to allocate memory for buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}

static int ovt_tcm_spi_rmi_read(struct ovt_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int idx;
	unsigned int mode;
	unsigned int byte_count;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	byte_count = length + 2;

	if (bdata->ubl_byte_delay_us == 0)
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, 2, byte_count);
	else
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, byte_count, 3);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)(addr >> 8) | 0x80;
	buf[1] = (unsigned char)addr;

	if (bdata->ubl_byte_delay_us == 0) {
		xfer[0].len = 2;
		xfer[0].tx_buf = buf;
		xfer[0].speed_hz = bdata->ubl_max_freq;
		spi_message_add_tail(&xfer[0], &msg);
		memset(&buf[2], 0xff, length);
		xfer[1].len = length;
		xfer[1].tx_buf = &buf[2];
		xfer[1].rx_buf = data;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
		if (bdata->block_delay_us)
			xfer[1].delay_usecs = bdata->block_delay_us;
#endif
		xfer[1].speed_hz = bdata->ubl_max_freq;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		buf[2] = 0xff;
		for (idx = 0; idx < byte_count; idx++) {
			xfer[idx].len = 1;
			if (idx < 2) {
				xfer[idx].tx_buf = &buf[idx];
			} else {
				xfer[idx].tx_buf = &buf[2];
				xfer[idx].rx_buf = &data[idx - 2];
			}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
			xfer[idx].delay_usecs = bdata->ubl_byte_delay_us;
			if (bdata->block_delay_us && (idx == byte_count - 1))
				xfer[idx].delay_usecs = bdata->block_delay_us;
#endif
			xfer[idx].speed_hz = bdata->ubl_max_freq;
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	mode = spi->mode;
	spi->mode = SPI_MODE_3;

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		LOGE(&spi->dev,
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

	spi->mode = mode;

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_rmi_write(struct ovt_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int mode;
	unsigned int byte_count;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
#endif
	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	byte_count = length + 2;

	retval = ovt_tcm_spi_alloc_mem(tcm_hcd, 1, byte_count);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)(addr >> 8) & ~0x80;
	buf[1] = (unsigned char)addr;
	retval = secure_memcpy(&buf[2],
			buf_size - 2,
			data,
			length,
			length);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to copy write data\n");
		goto exit;
	}

	xfer[0].len = byte_count;
	xfer[0].tx_buf = buf;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
	if (bdata->block_delay_us)
		xfer[0].delay_usecs = bdata->block_delay_us;
#endif
	spi_message_add_tail(&xfer[0], &msg);

	mode = spi->mode;
	spi->mode = SPI_MODE_3;

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		LOGE(&spi->dev,
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

	spi->mode = mode;

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_read(struct ovt_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	if (bdata->byte_delay_us == 0)
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, 1, length);
	else
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, length, 1);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory\n");
		goto exit;
	}

	if (bdata->byte_delay_us == 0) {
		memset(buf, 0xff, length);
		xfer[0].len = length;
		xfer[0].tx_buf = buf;
		xfer[0].rx_buf = data;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
		if (bdata->block_delay_us)
			xfer[0].delay_usecs = bdata->block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		buf[0] = 0xff;
		for (idx = 0; idx < length; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = buf;
			xfer[idx].rx_buf = &data[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
			xfer[idx].delay_usecs = bdata->byte_delay_us;
			if (bdata->block_delay_us && (idx == length - 1))
				xfer[idx].delay_usecs = bdata->block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		LOGE(&spi->dev,
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_write(struct ovt_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	if (bdata->byte_delay_us == 0)
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, 1, length);
	else
		retval = ovt_tcm_spi_alloc_mem(tcm_hcd, length, 1);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to allocate memory\n");
		goto exit;
	}

	if (bdata->byte_delay_us == 0) {
		xfer[0].len = length;
		xfer[0].tx_buf = data;
		xfer[0].rx_buf = buf;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
		if (bdata->block_delay_us)
			xfer[0].delay_usecs = bdata->block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < length; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &data[idx];
			xfer[idx].rx_buf = &buf[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
			xfer[idx].delay_usecs = bdata->byte_delay_us;
			if (bdata->block_delay_us && (idx == length - 1))
				xfer[idx].delay_usecs = bdata->block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		LOGE(&spi->dev,
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

#ifdef OVT_CHECK_DEVICE_BOOTMODE
static bool is_bootmode_charger(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool charger_mode = false;
	int ret;
	const char *bootargs = NULL;
	char *bootmode = NULL;

	OVT_INFO("is_bootmode_charger enter");
	if (!np){
		OVT_ERROR("chosen node NULL\n");
			return false;
	}

#ifdef CONFIG_BOOT_CONFIG
	OVT_INFO("BOOT_CONFIG is define\n");
	ret = of_property_read_string(np, "mmi,bootconfig", &bootargs);
#else
	OVT_INFO("BOOT_CONFIG is not define\n");
	ret = of_property_read_string(np, "bootargs", &bootargs);
#endif

	OVT_INFO("ret=%d, bootargs=%s\n", ret, bootargs);
	if(!ret && bootargs) {
		bootmode = strstr(bootargs, "androidboot.mode=");
		if(bootmode) {
			OVT_INFO("bootmode info: %s\n", bootmode);
			bootmode = strpbrk(bootmode, "=");
			if (bootmode && (strlen(bootmode) > 1)) {
				bootmode++;
				OVT_INFO("bootmode=%s\n", bootmode);
				if (bootmode && !strncmp(bootmode, "charger", strlen("charger"))) {
					charger_mode = true;
					OVT_INFO("Charger_smode true\n");
				}
			}
		} else
			OVT_ERROR("bootmode NULL\n");
	} else
		OVT_ERROR("get boottargs fail\n");

	of_node_put(np);

	OVT_INFO("Charger mode = %d\n",charger_mode);

	return charger_mode;
}
#endif

static int ovt_tcm_spi_probe(struct spi_device *spi)
{
	int retval;

	OVT_FUNC_ENTER();
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		LOGE(&spi->dev,
				"Full duplex not supported by host\n");
		return -EIO;
	}

	ovt_tcm_spi_device = platform_device_alloc(PLATFORM_DRIVER_NAME, 0);
	if (!ovt_tcm_spi_device) {
		LOGE(&spi->dev,
				"Failed to allocate platform device\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	hw_if.bdata = devm_kzalloc(&spi->dev, sizeof(*hw_if.bdata), GFP_KERNEL);
	if (!hw_if.bdata) {
		LOGE(&spi->dev,
				"Failed to allocate memory for board data\n");
		return -ENOMEM;
	}

#ifdef OVT_MTK_CHECK_PANEL
	retval = ovt_get_panel();
	if (retval) {
		OVT_INFO("MTK ovt panel fail, return %d\n", retval);
		return retval;
	}
#elif defined(CONFIG_DRM_CHECK_DT)
	retval = ovt_tcm_check_dt(np);
	if (retval == -EPROBE_DEFER)
		return retval;
#endif

#ifdef OVT_CHECK_DEVICE_BOOTMODE
	if(is_bootmode_charger()){
		OVT_INFO("Charger mode,ignore insmod ovt modules\n");
		return -ENODEV;
	}
#endif

	parse_dt(&spi->dev, hw_if.bdata);
/*
	pinctrl = devm_pinctrl_get(spi->controller->dev.parent);
	if (IS_ERR_OR_NULL(pinctrl)) {
		LOGE(&spi->dev,"Failed to get pinctrl handler[need confirm]\n");
		pinctrl = NULL;
	}
	// default spi mode
	pin_spi_mode_default = pinctrl_lookup_state(
				pinctrl, "lamu_spi_mode");
	if (IS_ERR_OR_NULL(pin_spi_mode_default)) {
		LOGE(&spi->dev,"Failed to get pinctrl state:%s\n", "lamu_spi_mode");
		pin_spi_mode_default = NULL;

	} else {
		retval = pinctrl_select_state(pinctrl, pin_spi_mode_default);
		if (retval < 0)
			LOGE(&spi->dev,"Failed to select default pinstate, retval:%d \n", retval);
		retval = 0;
	}
*/
#else
	hw_if.bdata = spi->dev.platform_data;
#endif

	switch (hw_if.bdata->spi_mode) {
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

	bus_io.type = BUS_SPI;
	bus_io.read = ovt_tcm_spi_read;
	bus_io.write = ovt_tcm_spi_write;
	bus_io.rmi_read = ovt_tcm_spi_rmi_read;
	bus_io.rmi_write = ovt_tcm_spi_rmi_write;

	hw_if.bus_io = &bus_io;

	spi->bits_per_word = 8;
#ifdef CONFIG_OVT_MTK_SPI_TIME
	spi->controller_data = (void *)&st_spi_ctrdata;
	OVT_INFO("config cs_setuptime:%d", st_spi_ctrdata.cs_setuptime);
#else
	spi->cs_setup.value = 6; //6 us
	//spi->cs_setup.unit = 0;
	OVT_INFO("config cs_setup.value:%d", spi->cs_setup.value);
#endif

	retval = spi_setup(spi);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to set up SPI protocol driver\n");
		return retval;
	}

	ovt_tcm_spi_device->dev.parent = &spi->dev;
	ovt_tcm_spi_device->dev.platform_data = &hw_if;

	retval = platform_device_add(ovt_tcm_spi_device);
	if (retval < 0) {
		LOGE(&spi->dev,
				"Failed to add platform device\n");
		return retval;
	}

	OVT_INFO("success, spi mode:%d, return 0\n", spi->mode);
	return 0;
}

static int ovt_tcm_spi_remove(struct spi_device *spi)
{
	ovt_tcm_spi_device->dev.platform_data = NULL;

	platform_device_unregister(ovt_tcm_spi_device);
	OVT_INFO("end");

	return 0;
}

static const struct spi_device_id ovt_tcm_id_table[] = {
	{SPI_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_id_table);

#ifdef CONFIG_OF
static struct of_device_id ovt_tcm_of_match_table[] = {
	{
		.compatible = "omnivision,tcm-spi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match_table);
#else
#define ovt_tcm_of_match_table NULL
#endif

static struct spi_driver ovt_tcm_spi_driver = {
	.driver = {
		.name = SPI_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ovt_tcm_of_match_table,
	},
	.probe = ovt_tcm_spi_probe,
	.remove = ovt_tcm_spi_remove,
	.id_table = ovt_tcm_id_table,
};

int ovt_tcm_bus_init(void)
{
	return spi_register_driver(&ovt_tcm_spi_driver);
}
EXPORT_SYMBOL(ovt_tcm_bus_init);

void ovt_tcm_bus_exit(void)
{
	kfree(buf);

	kfree(xfer);

	spi_unregister_driver(&ovt_tcm_spi_driver);

	return;
}
EXPORT_SYMBOL(ovt_tcm_bus_exit);

MODULE_AUTHOR("omnivision, Inc.");
MODULE_DESCRIPTION("omnivision TCM SPI Bus Module");
MODULE_LICENSE("GPL v2");
