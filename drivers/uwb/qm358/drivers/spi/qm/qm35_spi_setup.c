/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include "qm35_spi.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
#define kv(a, b, c) KERNEL_VERSION(a, b, c)
#define lvc LINUX_VERSION_CODE
#define check_rev(ma, mi, rev) \
	((kv(ma, mi, rev) <= lvc) && (lvc < kv(ma, mi + 1, 0)))
#if check_rev(5, 4, 242) || check_rev(5, 10, 163)
#define HAVE_IRQF_NO_AUTOEN 1
#else
#define HAVE_IRQF_NO_AUTOEN 0
#endif
#else
#define HAVE_IRQF_NO_AUTOEN 1
#endif

/*
 * QM35 SPI hardware setup
 */

static int qm35_setup_one_regulator(const char *name, struct device *dev,
				    struct regulator **out)
{
	struct regulator *regulator;

	regulator = devm_regulator_get_optional(dev, name);
	if (IS_ERR(regulator)) {
		int err = PTR_ERR(regulator);
		dev_notice(dev, "No %s regulator defined in device tree (%d)\n",
			   name, err);
		regulator = NULL;
	}
	*out = regulator;
	return !!regulator;
}

/**
 * qm35_setup_regulators() - Setup QM35 power regulators.
 * @qmspi: The QM35 SPI device to get regulator config from DT.
 *
 * This will search for configured regulators to use from the device tree.
 * All of them are optional, so missing one will just be noticed to dmesg.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_setup_regulators(struct qm35_spi *qmspi)
{
	struct qm35_regulators *power = &qmspi->regulators;
	struct device *dev = qmspi->base.dev;
	int idx = 0;

	/* Try new names first. */
	idx += qm35_setup_one_regulator("vdd1_reg", dev, &power->vdd[idx]);
	if (!idx)
		goto legacy;
	/* Only use new names. */
	idx += qm35_setup_one_regulator("vdd2_reg", dev, &power->vdd[idx]);
	idx += qm35_setup_one_regulator("vdd3_reg", dev, &power->vdd[idx]);
	idx += qm35_setup_one_regulator("vdd4_reg", dev, &power->vdd[idx]);
	idx += qm35_setup_one_regulator("vdd5_reg", dev, &power->vdd[idx]);

	/* At least one regulator defined. */
	return 0;

legacy:
	/* Try legacy names. */
	idx += qm35_setup_one_regulator("power_reg_1p8", dev, &power->vdd[idx]);
	idx += qm35_setup_one_regulator("power_reg_2p5", dev, &power->vdd[idx]);
	idx += qm35_setup_one_regulator("power_reg", dev, &power->vdd[idx]);

	if (!idx)
		dev_warn(dev, "No regulators, assuming always on\n");
	return 0;
}

//Add by motorola begin
static struct gpio_desc *global_temporary_gpio_csn;
void qm35_set_csn_level(int level)
{
    gpiod_set_value(global_temporary_gpio_csn, level ? 1 : 0);
}
//Add by motorola end

/**
 * qm35_setup_gpios() - Setup all GPIOs from DT.
 * @qmspi: The QM35 SPI device to get GPIOs config from DT.
 *
 * Get the RESET GPIO to use from the DT and configure it in output
 * open-drain mode and activate it. This ensure the QM35 device is
 * in reset state, IRQ pin low from the end of this function.
 *
 * Also get the READY GPIO from the DT and configure it as input.
 * It is used during HSSPI transfers to check FW is ready to handle
 * an SPI transfer.
 *
 * And get the WAKEUP and EXTON GPIOs from the DT and configure them.
 * They are used to check if the device is in low power mode and wake it up.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_setup_gpios(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	const char warn_cansleep[] =
		"%s GPIO can sleep! This affects performance!\n";
	struct gpio_desc *gpio;

	//Add by motorola begin
	global_temporary_gpio_csn = devm_gpiod_get_optional(dev, "csn", GPIOD_OUT_LOW);
	if (!global_temporary_gpio_csn)
	{
		dev_err(dev, "%s: csn-gpios not found", __func__);
	} else {
	dev_err(dev, "%s: csn-gpios found!!", __func__);
	}
	gpio = devm_gpiod_get_optional(dev, "uwbccc", GPIOD_OUT_HIGH);
	if (!gpio || IS_ERR(gpio)) {
		int ret = PTR_ERR(gpio);
		dev_err(dev,
			"Device does not support UWBCCC GPIO control (%d)\n",
			ret);
	}
	//Add by motorola end
	/* Retrieve RESET GPIO from device tree and initialise as output
	 * active low. */
	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (!gpio || IS_ERR(gpio)) {
		int ret = PTR_ERR(gpio);
		dev_err(dev,
			"Device does not support RESET GPIO control (%d)\n",
			ret);
		return ret;
	}
	qmspi->reset_gpio = gpio;
	if (!(qm35_debug_flags & QMSPI_NO_RESET))
		gpiod_set_value_cansleep(qmspi->reset_gpio, 1);

	if (qm35_debug_flags & QMSPI_LOW_GPIO) {
		dev_warn(
			dev,
			"READY, WAKEUP and EXTON GPIOs disabled by debug_flags\n");
		return 0;
	}

	/* Retrieve READY GPIO from device tree and initialise as input. */
	gpio = devm_gpiod_get_optional(dev, "ready", GPIOD_IN);
	if (!gpio || IS_ERR(gpio)) {
		int ret = PTR_ERR(gpio);
		dev_warn(dev, "Device does not support READY GPIO (%d)\n", ret);
	} else {
		qmspi->ready_gpio = gpio;
		if (gpiod_cansleep(qmspi->ready_gpio))
			dev_warn(dev, warn_cansleep, "READY");
	}

	/* Retrieve READY GPIO from device tree and initialise as output
	 * active low. */
	gpio = devm_gpiod_get_optional(dev, "wakeup", GPIOD_OUT_LOW);
	if (!gpio || IS_ERR(gpio)) {
		int ret = PTR_ERR(gpio);
		dev_warn(dev,
			 "Device does not support WAKEUP GPIO control (%d)\n",
			 ret);
	} else {
		qmspi->wakeup_gpio = gpio;
		if (gpiod_cansleep(qmspi->wakeup_gpio))
			dev_warn(dev, warn_cansleep, "WAKEUP");
	}

	/* Retrieve EXTON GPIO from device tree and initialise as input. */
	gpio = devm_gpiod_get_optional(dev, "exton", GPIOD_IN);
	if (!gpio || IS_ERR(gpio)) {
		int ret = PTR_ERR(gpio);
		dev_warn(dev, "Device does not support EXTON GPIO (%d)\n", ret);
	} else {
		qmspi->exton_gpio = gpio;
		if (gpiod_cansleep(qmspi->exton_gpio))
			dev_warn(dev, warn_cansleep, "EXTON");
	}
	return 0;
}

/**
 * qm35_irq_handler() - Hard-IRQ handler for the QM35 IRQ.
 * @irq: IRQ number.
 * @context: IRQ handler context, our QM35 SPI instance.
 *
 * Called when an IRQ happened and need to be processes. This
 * handler just queue the information to the QM35 SPI processing
 * thread.
 *
 * Returns: IRQ_HANDLED.
 */
static irqreturn_t qm35_irq_handler(int irq, void *context)
{
	struct qm35_spi *qmspi = context;

	atomic_set(&qmspi->should_read, true);
	qm35_enqueue_irq(qmspi);

	return IRQ_HANDLED;
}

/**
 * qm35_setup_irq() - Setup GPIO and request IRQ for the QM35 SPI device.
 * @qmspi: The QM35 SPI device to setup IRQ config from DT.
 *
 * Ensure the IRQ is correctly configured and install the hard IRQ handler.
 * The IRQ is immediately disabled, waiting the device to be started.
 *
 * Note: If the reset GPIO is deasserted before this function, a spurious
 *       IRQ may be handled and qm35_irq_handler() called.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_setup_irq(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	struct gpio_desc *gpio;
	int irq = 0;
	int irq_flags, ret;

	/* First, check the presence of "irq-gpios" in DT. */
	gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(gpio)) {
		dev_warn(dev,
			 "Device does not support IRQ GPIO, "
			 "firmware update will not be possible (%ld)\n",
			 PTR_ERR(gpio));
		gpio = NULL;
	}
	/* Save IRQ GPIO. */
	qmspi->irq_gpio = gpio;

	/* Then, if "irq-gpios" was found, try to use it as IRQ. */
	if (gpio) {
		irq = gpiod_to_irq(gpio);
		if (irq > 0)
			/* Save IRQ. */
			qmspi->spi->irq = irq;
		else
			dev_warn(
				dev,
				"Could not get IRQ corresponding to GPIO (%d)\n",
				irq);
	}

	/* Otherwise, fall back to IRQ configured by spi_probe() using the
	 * "interrupt-parent" or "interrupts-extended" properties, if available.
	 */
	if (irq <= 0)
		irq = qmspi->spi->irq;
	if (irq <= 0) {
		dev_err(dev, "Device does not support IRQ\n");
		return -ENXIO;
	}

	/* Set required IRQ trigger mode in IRQ flags */
	irq_flags = irq_get_trigger_type(irq);
	if (!irq_flags)
		irq_flags = IRQF_TRIGGER_HIGH;
#if HAVE_IRQF_NO_AUTOEN
	/* Hook interruption but without auto-enable at lower-level. */
	irq_flags |= IRQF_NO_AUTOEN;
#else
	irq_set_status_flags(irq, IRQ_NOAUTOEN);
#endif
	ret = devm_request_irq(dev, irq, qm35_irq_handler, irq_flags,
			       dev_name(dev), qmspi);
	if (ret)
		dev_err(dev, "Could not request the IRQ %d: %d\n", irq, ret);
	return ret;
}
