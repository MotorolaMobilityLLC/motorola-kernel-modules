/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/clk.h>

#include "qm35_spi.h"
#include "qm35_spi_pm.h"
#include "qm35_spi_trc.h"

#ifndef AUTOSUSPEND_DELAY_MS
#define AUTOSUSPEND_DELAY_MS 5000
#endif

/*
 * QM35 SPI regulator management
 */

static int qm35_power_supply_one(struct regulator *regulator, bool on)
{
	int rc;

	if (on)
		rc = regulator_enable(regulator);
	else
		rc = regulator_disable(regulator);
	return rc;
}

/**
 * qm35_spi_power_supply() - Handle power supply requests.
 * @qmspi: QM35 SPI instance to mange power.
 * @on: Requested QM35 state.
 *
 * Manage power-supply using configured regulators from device tree.
 *
 * If no regulator are configured, this function does nothing. If at least one
 * regulator is configured, this function power-on the device when called with
 * `on` equals to `true` and power-down it when called with `on` equals to
 * false.
 *
 * It uses an atomic counter to power-on/power-off only when required and
 * support multiple calls (on-off-on-on-off-off).
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_power_supply(struct qm35_spi *qmspi, bool on)
{
	struct qm35_regulators *power = &qmspi->regulators;
	struct device *dev = qmspi->base.dev;
	int i, rc;

	/* Early return if no regulator defined */
	if (!power->vdd[0])
		return 0;
	/* Early return if no action required */
	if (on) {
		if (atomic_inc_return(&power->enabled) > 1)
			return 0;
	} else {
		if (!atomic_dec_and_test(&power->enabled))
			return 0;
	}
	dev_dbg(dev, "Power-%s QM35 device\n", on ? "on" : "off");
	/* Change defined regulators state */
	for (i = 0; i < QM35_MAX_REGULATORS; i++) {
		struct regulator *reg = power->vdd[i];
		if (!reg)
			break;
		rc = qm35_power_supply_one(reg, on);
		if (rc < 0) {
			dev_err(dev, "Regulator %s failed for vdd%d (%d)\n",
				on ? "enable" : "disable", i, rc);
			return rc;
		}
	}
	/* Add some delay to wait regulator stable */
	usleep_range(qm35_regulator_delay_us, qm35_regulator_delay_us + 100);
	return rc;
}

/**
 * qm35_spi_clock_control() - enable/disable clock.
 * @qmspi: QM35 SPI instance.
 * @enable: Requested clock state.
 */
static void qm35_spi_clock_control(struct qm35_spi *qmspi, bool enable)
{
	struct device *dev = qmspi->base.dev;

	if (qmspi->clk) {
		if (enable) {
			if (!qmspi->clk_enabled) {
				int rc = clk_prepare_enable(qmspi->clk);
				if(rc)
					dev_err(dev, "%s: uwb clock enable failed", __func__);
				else {
					qmspi->clk_enabled = true;
					dev_info(dev, "%s: uwb clock enabled\n", __func__);
				}
			}
		} else {
			if (qmspi->clk_enabled) {
				clk_disable_unprepare(qmspi->clk);
				qmspi->clk_enabled = false;
				dev_info(dev, "%s: uwb clock disabled\n", __func__);
			}
		}
	}
}

/*
 * QM35 SPI power management
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 11))
/* Need to define pm_runtime_resume_and_get() if not backported. */
#define kv(a, b, c) KERNEL_VERSION(a, b, c)
#define lvc LINUX_VERSION_CODE
#define check_rev(ma, mi, rev) \
	((kv(ma, mi, rev) <= lvc) && (lvc < kv(ma, mi + 1, 0)))
/* Function back-ported for version 5.4, starting at revision 86. */
#if !check_rev(5, 4, 86)
/* This kernel version doesn't seem to provide pm_runtime_resume_and_get(),
 * so define it here. */
static inline int pm_runtime_resume_and_get(struct device *dev)
{
	int ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}
	return 0;
}

#endif
#undef check_rev
#undef lvc
#undef kv
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 11)) */

/**
 * qm35_spi_pm_setup() - Setup QM35 power management.
 * @qmspi: The QM35 SPI device to setup power management.
 *
 * This will initialise all required to have a functional power management
 * for this device.
 *
 * Especially, it configures the device to be a wakeup source and set the IRQ
 * as the wake IRQ.
 *
 * It also sets the suspend_reset flag according to debug_flags parameter. It
 * is not related anymore to regulator configuration in device tree.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_spi_pm_setup(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	trace_qm35_spi_pm_setup(qmspi);
	qmspi->suspend_reset = !(QMSPI_NO_RESET_ON_SUSPEND & qm35_debug_flags);
	pm_runtime_set_autosuspend_delay(dev, AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	device_init_wakeup(dev, true);
	dev_pm_set_wake_irq(dev, qmspi->spi->irq);
	return 0;
}

/**
 * qm35_spi_pm_remove() - Remove QM35 power management.
 * @qmspi: The QM35 SPI device to remove power management.
 *
 * This will tear down PM for this device.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_spi_pm_remove(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	trace_qm35_spi_pm_remove(qmspi);
	dev_pm_clear_wake_irq(dev);
	device_set_wakeup_capable(dev, false);

	pm_runtime_get_sync(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);

	/* As the device was resumed either by device_driver_detach() or by
	 * pm_runtime_get_sync() just before, we need to ensure IRQ is disabled
	 * and power supplies are down. */

	/* Disable interrupts (counting) */
	disable_irq(qmspi->spi->irq);
	/* Power-down device */
	qm35_spi_power_supply(qmspi, false);
	return 0;
}

/**
 * qm35_spi_pm_start() - PM runtime resume and get.
 * @qmspi: QM35 SPI instance.
 *
 * Returns: 0 or negative error.
 */
int qm35_spi_pm_start(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	int ret;
	trace_qm35_spi_pm_start(qmspi);
	ret = pm_runtime_resume_and_get(dev);
	qmspi->started = atomic_read(&dev->power.usage_count) > 0;
	return ret;
}

/**
 * qm35_spi_pm_stop() - PM runtime put.
 * @qmspi: QM35 SPI instance.
 *
 * Returns: 0 or negative error.
 */
int qm35_spi_pm_stop(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	int ret;
	trace_qm35_spi_pm_stop(qmspi);
	pm_runtime_mark_last_busy(dev);
	ret = pm_runtime_put(dev);
	qmspi->started = atomic_read(&dev->power.usage_count) > 0;
	return ret;
}

/**
 * qm35_spi_pm_resume() - PM runtime sync resume.
 * @qmspi: QM35 SPI instance.
 *
 * Returns: 0 or negative error.
 */
int qm35_spi_pm_resume(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	int rc;

	trace_qm35_spi_pm_resume(qmspi);
	pm_runtime_mark_last_busy(dev);
	rc = pm_runtime_resume(dev);
	/* Resume return 1 if already active! */
	return rc < 0 ? rc : 0;
}

/**
 * qm35_spi_pm_idle() - PM runtime idle and auto-suspend.
 * @qmspi: QM35 SPI instance.
 *
 * Use async PM request API.
 *
 * Returns: 0 or negative error.
 */
int qm35_spi_pm_idle(struct qm35_spi *qmspi)
{
	struct device *dev = qmspi->base.dev;
	trace_qm35_spi_pm_idle(qmspi);
	pm_runtime_mark_last_busy(dev);
	return pm_request_idle(dev);
}

/*
 * QM35 SPI PM runtime callbacks
 */

/**
 * qm35_pm_runtime_suspend() - PM runtime suspend callback.
 * @dev: Device instance.
 *
 * Power-down the QM35 and disable the IRQ. This callback is called by the PM
 * runtime framework only if the device is idle and not active.
 *
 * Returns: 0 or a negative error.
 */
static int __maybe_unused qm35_pm_runtime_suspend(struct device *dev)
{
	struct qm35_spi *qmspi = dev_get_drvdata(dev);
	int rc;

	trace_qm35_pm_runtime_suspend(qmspi);

	/* Hold reset_gpio if wanted. */
	if (qmspi->suspend_reset) {
		gpiod_set_value_cansleep(qmspi->reset_gpio, 1);
		/* Ensure minimum reset duration if resume callback called
		 * immediately after this one. */
		usleep_range(QM35_RESET_DURATION_US,
			     QM35_RESET_DURATION_US + 100);
	}

	/* Disable interrupts (counting) */
	disable_irq(qmspi->spi->irq);

	/* Power-down device */
	rc = qm35_spi_power_supply(qmspi, false);
	if (rc < 0) {
		dev_warn(dev, "Failure to power-off the QM35! (%d)\n", rc);
	}

	qm35_spi_clock_control(qmspi, false);

	trace_qm35_pm_runtime_suspend_return(qmspi, rc);
	return rc;
}

/**
 * qm35_pm_runtime_resume() - PM runtime resume callback.
 * @dev: Device instance.
 *
 * Power-on the device and enable IRQ. This callback is called by the PM
 * runtime framework when the device become active (even temporary).
 *
 * Returns: 0 or negative error.
 */
static int __maybe_unused qm35_pm_runtime_resume(struct device *dev)
{
	struct qm35_spi *qmspi = dev_get_drvdata(dev);
	int rc;

	trace_qm35_pm_runtime_resume(qmspi);

	qm35_spi_clock_control(qmspi, true);

	rc = qm35_spi_power_supply(qmspi, true);
	if (rc < 0) {
		dev_err(dev, "Failure to power-on the QM35! (%d)\n", rc);
		goto error;
	}

	/* Enable IRQ (counting) */
	enable_irq(qmspi->spi->irq);

	/* Release reset_gpio if wanted. */
	if (qmspi->suspend_reset) {
		gpiod_set_value_cansleep(qmspi->reset_gpio, 0);
		/* Ensure minimum reset backoff duration to avoid entering
		 * bootrom command mode if SPI message is sent immediately after
		 * this callback. */
		usleep_range(QM35_RESET_BACKOFF_DURATION_US,
			     2 * QM35_RESET_BACKOFF_DURATION_US);
	}

error:
	trace_qm35_pm_runtime_resume_return(qmspi, rc);
	return rc;
}

static int __maybe_unused qm35_pm_runtime_idle(struct device *dev)
{
	struct qm35_spi *qmspi = dev_get_drvdata(dev);
	trace_qm35_pm_runtime_idle(qmspi);

	/* TODO: Implement idle feature.
	 * This should be useful to ensure QM35 remain powered as soon as a
	 * scheduler/region is configured to not loss this configuration.
	 */

	trace_qm35_pm_runtime_idle_return(qmspi, 0);
	return 0;
}

/*
 * QM35 SPI System sleep callbacks
 */

/**
 * qm35_system_suspend() - System suspend callback.
 * @dev: Device instance.
 *
 * If device is active (UCI char device is opened or WPAN interface is up),
 * configure the IRQ to be a wakeup source and leave the QM35 running.
 *
 * Else power-down the QM35 and disable the IRQ using the PM runtime suspend
 * callback.
 *
 * Returns: 0 or a negative error.
 */
static int __maybe_unused qm35_system_suspend(struct device *dev)
{
	struct qm35_spi *qmspi = dev_get_drvdata(dev);
	int ret = 0;

	trace_qm35_system_suspend(qmspi);
	/* qm35_spi_pm_setup() set device wakeup capable. So don't need to
	 * call device_may_wakeup() as it always returns true! */
	if (qmspi->started) {
		/* UCI char device opened or WPAN interface up & running. */
		int irq = qmspi->spi->irq;
		enable_irq_wake(irq);
	} else {
		/* Ensure device is suspended if not used. */
		ret = pm_runtime_force_suspend(dev);
	}
	trace_qm35_system_suspend_return(qmspi, ret);
	return ret;
}

/**
 * qm35_system_resume() - System resume callback.
 * @dev: Device instance.
 *
 * If device is active (UCI char device is opened or WPAN interface is up),
 * just remove the added wakeup source on the IRQ by the suspend callback.
 *
 * Else power-up the QM35 and enable the IRQ using the PM runtime resume
 * callback.
 *
 * Returns: 0 or a negative error.
 */
static int __maybe_unused qm35_system_resume(struct device *dev)
{
	struct qm35_spi *qmspi = dev_get_drvdata(dev);
	int ret = 0;

	trace_qm35_system_suspend(qmspi);
	/* qm35_spi_pm_setup() set device wakeup capable. So don't need to
	 * call device_may_wakeup() as it always returns true! */
	if (qmspi->started) {
		/* UCI char device opened or WPAN interface up & running. */
		int irq = qmspi->spi->irq;
		disable_irq_wake(irq);
	} else {
		/* Ensure device is suspended if not used. */
		ret = pm_runtime_force_resume(dev);
	}
	trace_qm35_system_suspend_return(qmspi, ret);
	return ret;
}

/* clang-format off */
const struct dev_pm_ops __maybe_unused qm35_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(qm35_pm_runtime_suspend,
			   qm35_pm_runtime_resume,
			   qm35_pm_runtime_idle)
	SET_SYSTEM_SLEEP_PM_OPS(qm35_system_suspend,
				qm35_system_resume)
};
/* clang-format on */
