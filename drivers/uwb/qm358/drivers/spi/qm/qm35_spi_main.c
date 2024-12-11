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
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/skbuff.h>
#ifdef CONFIG_EVENT_TRACING
#include <linux/trace_events.h> /* for trace_set_clr_event() */
#endif
#include <linux/mmi_device.h>
#include <linux/version.h>
#include <linux/wait.h>

#include "qm35_core.h"
#include "qm35_hsspi.h"
#include "qm35_spi.h"
#include "qm35_spi_pm.h"
#include "qm35_spi_transport.h"
#include "qm35_spi_trc.h"
#include "qm35_transport.h"
#include <linux/clk.h>

#ifndef DEFAULT_DEBUG_FLAGS
#define DEFAULT_DEBUG_FLAGS QMSPI_AUTO_TRACE
#endif

/* If qm35_##name##_set_by_param is true, the parameter is overridden by insmod.
 * Else the parameter is read from the device tree if it exists. */
#define QM35_CONFIGURE_PARAMETER(name, type, fmt_type, device, node)          \
	do {                                                                  \
		if (qm35_##name##_set_by_param)                               \
			dev_info(&device,                                     \
				 #name " set by module parameter: " fmt_type  \
				       "\n",                                  \
				 qm35_##name);                                \
		else if (!of_property_read_##type(node, #name, &qm35_##name)) \
			dev_info(&device,                                     \
				 #name " set by device tree: " fmt_type "\n", \
				 qm35_##name);                                \
	} while (0)

#define QM35_DECLARE_PARAMETER(name, type, perm, desc)                 \
	static bool qm35_##name##_set_by_param = false;                \
	static int qm35_##name##_set(const char *val,                  \
				     const struct kernel_param *kp)    \
	{                                                              \
		qm35_##name##_set_by_param = true;                     \
		return param_set_##type(val, kp);                      \
	}                                                              \
	static const struct kernel_param_ops qm35_##name##_ops = {     \
		.set = qm35_##name##_set,                              \
		.get = param_get_##type,                               \
	};                                                             \
	module_param_cb(name, &qm35_##name##_ops, &qm35_##name, perm); \
	MODULE_PARM_DESC(name, desc)

static int qm35_cpu = -1;
QM35_DECLARE_PARAMETER(cpu, int, 0444,
		       "CPU on which the QM35 processing thread will run");

static u32 qm35_qos_latency = 0;
QM35_DECLARE_PARAMETER(
	qos_latency, uint, 0644,
	"Latency request to PM QoS on active ranging in microsecond");

u32 qm35_regulator_delay_us = 1000;
QM35_DECLARE_PARAMETER(
	regulator_delay_us, uint, 0444,
	"Delay after regulator state changed (default is 1000us)");

u32 qm35_hsspi_delay_us = 0;
QM35_DECLARE_PARAMETER(
	hsspi_delay_us, uint, 0644,
	"Minimum delay between two SPI transfers (default is 0us)");

u32 qm35_debug_flags = DEFAULT_DEBUG_FLAGS;
QM35_DECLARE_PARAMETER(debug_flags, uint, 0660, "Debug flags bit-field");

static u32 qm35_spi_max_speed_hz = 0;
module_param_named(spi_max_speed_hz, qm35_spi_max_speed_hz, uint, 0444);
MODULE_PARM_DESC(spi_max_speed_hz, "Maximum SPI transfers speed in Hz");

const char *qm35_fw_name = "";
QM35_DECLARE_PARAMETER(
	fw_name, charp, 0644,
	"Name of the firmware file to be loaded, replacing the default one");

const char *const optionnal_modules[] = { "qm35_logs", "qm35_coredump",
					  "qm35_uci_dev", "qm35_ieee_nl" };

/**
 * qm35_spi_isr() - Process the QM35 IRQ.
 * @qmspi: QM35 SPI instance.
 *
 * Called from the QM35 SPI thread, it informs the QM35 core device that an
 * event has been received by calling the qm35_transport_event() function.
 *
 * Context: Called from qm35_spi_thread().
 * Returns: Zero on success else a negative error.
 */
int qm35_spi_isr(struct qm35_spi *qmspi)
{
	enum qm35_transport_events event = QM35_TRANSPORT_EVENT_IRQ;

	trace_qm35_spi_isr(qmspi);
	qm35_transport_event(&qmspi->base, event);
	trace_qm35_spi_isr_return(qmspi);
	return 0;
}

/**
 * qm35_spi_awake_handle() - QM35 transport handler for AWAKE packets.
 * @data: Pointer to qm35_spi structure.
 * @skb: AWAKE packet received.
 *
 * Context: Always called from qm35_transport_event().
 */
static void qm35_spi_awake_handle(void *data, struct sk_buff *skb)
{
	struct qm35_spi *qmspi = data;

	trace_qm35_spi_awake_handle(qmspi);
	/* If FW send an AWAKE packet, we can use async wakeup. */
	qmspi->async_wakeup = true;
	/* Receiving an AWAKE packet is condition to wake-up qm35_spi_send(). */
	qmspi->wakeup_event = true;
	/* Starting with kernel version 6.1, wake_up return an int. But to
	 * remain compatible with version 5.19 and less, assume it is void. */
	wake_up(&qmspi->wakeup_wait);
	/* Free the received skb. */
	consume_skb(skb);
}

/**
 * qm35_spi_driver_probe() - Probe and initialize QM35 SPI device.
 * @spi: The SPI device to probe and initialize.
 *
 * Called when module is loaded and an SPI controller driver exist.
 *
 * This function allocates private device structure with qm35_alloc_device()
 * and then initializes all HW related stuff (SPI, GPIOs, IRQ...).
 *
 * Then, if there are no errors, the device is registered with
 * qm35_register_device() which also starts the probing using the provided
 * transport callbacks.
 *
 * Return: 0 if device probed correctly else a negative error.
 */
static int qm35_spi_driver_probe(struct spi_device *spi)
{
	struct qm35_spi *qmspi;
	struct qm35 *qm;
	struct device_node *node = spi->dev.of_node;
	struct qm35_transport transport = qm35_spi_transport;
	struct clk *uwb_clk;
	bool uwb_clk_enabled = false;

	int rc;

	dev_info(&spi->dev, "Probing new QM35 SPI device...\n");

	if (spi->dev.of_node && !mmi_device_is_available(spi->dev.of_node)) {
		pr_err("%s : mmi: device not supported\n", __func__);
		return -ENODEV;
	} else {
		pr_err("%s : supported uwb device found\n", __func__);
	}

	uwb_clk = devm_clk_get(&spi->dev, "uwb_rf_clk5");
	if (IS_ERR(uwb_clk)) {
		dev_err(&spi->dev, "%s: uwb_clk not found", __func__);
	} else {
		rc = clk_prepare_enable(uwb_clk);
		if(rc)
			dev_err(&spi->dev, "%s: uwb_clk enable failed", __func__);
		else
			uwb_clk_enabled = true;
	}


	/* Parameters management. */
	QM35_CONFIGURE_PARAMETER(cpu, s32, "%d", spi->dev, node);
	QM35_CONFIGURE_PARAMETER(qos_latency, u32, "%u", spi->dev, node);
	QM35_CONFIGURE_PARAMETER(regulator_delay_us, u32, "%u", spi->dev, node);
	QM35_CONFIGURE_PARAMETER(hsspi_delay_us, u32, "%u", spi->dev, node);
	QM35_CONFIGURE_PARAMETER(debug_flags, u32, "%u", spi->dev, node);
	QM35_CONFIGURE_PARAMETER(fw_name, string, "%s", spi->dev, node);

	/* Allocate QM35 core device */
	if (qm35_debug_flags & QMSPI_NO_PROBING) {
		transport.flags |= QM35_TRANSPORT_NO_PROBING;
	}
	qm = qm35_alloc_device(&spi->dev, sizeof(*qmspi), &transport);
	if (!qm) {
		rc = -ENOMEM;
		goto err_alloc_hw;
	}
	qmspi = qm35_to_qm35_spi(qm);
	qmspi->spi = spi;
	spi_set_drvdata(spi, qmspi);

	/* Initialise early wait queue and mutex */
	init_waitqueue_head(&qmspi->worker.work_wq);
	mutex_init(&qmspi->worker.mtx);

	/* Setup transport related. */
	qm35_spi_transport_setup(qmspi);

	/* Setup SPI parameters */
	spi->bits_per_word = 8;
#if (KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE)
	spi->rt = 1;
#endif
#if ((KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE) && \
     (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)))
	/* Quirk to force spi_set_cs() in spi_setup() to do something!
	   !!! Not doing this results in CS line stay LOW and SPIRDY IRQ
	   isn't fired later when powering-on the device. Previous kernel
	   don't have this bug as it always apply new CS state. */
	spi->master->last_cs_enable = true;
#endif
	/* Save configured device max speed,
	 * and set it from module parameter if provided.
	 */
	qmspi->of_max_speed_hz = spi->max_speed_hz;
	if (qm35_spi_max_speed_hz)
		spi->max_speed_hz = qm35_spi_max_speed_hz;
	else
		qm35_spi_max_speed_hz = spi->max_speed_hz;
	/* Setup SPI and put CS line in HIGH state! */
	rc = spi_setup(spi);
	if (rc != 0)
		goto err_spi_setup;
	dev_info(qm->dev, "Setup mode: %d, %u bits/w, %u Hz max, can_dma: %d\n",
		 (int)(spi->mode & (SPI_CPOL | SPI_CPHA)), spi->bits_per_word,
		 spi->max_speed_hz, spi->master->can_dma != NULL);

	/* Request and setup regulators if available */
	qm35_setup_regulators(qmspi);
	/* Request and setup the reset GPIO pin */
	rc = qm35_setup_gpios(qmspi);
	if (rc != 0)
		goto err_setup_gpios;
	/* Request and setup the irq GPIO pin */
	rc = qm35_setup_irq(qmspi);
	if (rc != 0)
		goto err_setup_irq;

	/* Start event processing thread */
	rc = qm35_thread_run(qmspi, qm35_cpu);
	if (rc != 0)
		goto err_thread_run;

	/* Setup PM runtime power-management. */
	rc = qm35_spi_pm_setup(qmspi);
	if (rc != 0)
		goto err_setup_pm;

#if IS_ENABLED(CONFIG_QM35_FLASHING)
	/* Initialize firmware update mutex. */
	mutex_init(&qmspi->fw.update_lock);
#endif

	/* Initialize info file mutex. */
	mutex_init(&qmspi->info_mutex);

	/* Init async wakeup support and packet handler. */
	init_waitqueue_head(&qmspi->wakeup_wait);
	qm35_transport_register(qm, QM35_TRANSPORT_MSG_AWAKE,
				QM35_TRANSPORT_PRIO_NORMAL,
				qm35_spi_awake_handle, qmspi);

	/* Register MCPS 802.15.4 device */
	rc = qm35_register_device(qm);
	if (rc) {
		dev_err(qm->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	/* All is ok */
	dev_info(&spi->dev, "Registered QM35 SPI HW%d\n", qm->dev_id);

	/* Save info need for qm35_nl_get_calibration() */
	qm->worker_pid = qmspi->worker.thread->pid;
#if (KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE)
#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE)
	qm->transport_pid = spi->controller->kworker->task->pid;
#else
	qm->transport_pid = spi->controller->kworker.task->pid;
#endif
#else
	qm->transport_pid = spi->master->kworker.task->pid;
#endif

	if (uwb_clk_enabled) {
		qmspi->clk_enabled = true;
		qmspi->clk = uwb_clk;
	}
	return 0;

err_register_hw:
	mutex_lock(&qmspi->info_mutex);
	if (qmspi->info_bin_attr.attr.name)
		sysfs_remove_bin_file(&spi->dev.kobj, &qmspi->info_bin_attr);
	mutex_unlock(&qmspi->info_mutex);
	mutex_destroy(&qmspi->info_mutex);
#if IS_ENABLED(CONFIG_QM35_FLASHING)
	mutex_destroy(&qmspi->fw.update_lock);
#endif
	qm35_spi_pm_remove(qmspi);
err_setup_pm:
	qm35_thread_stop(qmspi);
err_thread_run:
	/* TODO: Un-setup IRQ? */
err_setup_irq:
	/* TODO: Un-setup GPIOs? */
err_setup_gpios:
err_spi_setup:
	qm35_free_device(qm);
	spi_set_drvdata(spi, NULL);
err_alloc_hw:
	return rc;
}

/**
 * qm35_spi_driver_remove() - Remove QM35 SPI device.
 * @spi: The SPI device to remove.
 *
 * Called when module is unloaded, this function removes all
 * sysfs/debugfs files, unregisters this QM35 device and them
 * frees all remaining resources.
 *
 * Return: always 0.
 */
static int qm35_spi_driver_remove(struct spi_device *spi)
{
	struct qm35_spi *qmspi = spi_get_drvdata(spi);
	struct qm35 *qm;

	if (!qmspi)
		return 0;
	qm = &qmspi->base;
	dev_info(&spi->dev, "Removing QM35 SPI HW%d\n", qm->dev_id);

	/* Restore configured device max speed. */
	spi->max_speed_hz = qmspi->of_max_speed_hz;
	spi_setup(spi);
	mutex_lock(&qmspi->info_mutex);
	if (qmspi->info_bin_attr.attr.name)
		sysfs_remove_bin_file(&spi->dev.kobj, &qmspi->info_bin_attr);
	mutex_unlock(&qmspi->info_mutex);
	mutex_destroy(&qmspi->info_mutex);
	/* Unregister subsystems. */
	qm35_unregister_device(qm);
#if IS_ENABLED(CONFIG_QM35_FLASHING)
	/* Mark the firmware update mutex uninitialized. */
	mutex_destroy(&qmspi->fw.update_lock);
#endif
	/* Disable PM runtime subsystem. */
	qm35_spi_pm_remove(qmspi);
	/* Stop event processing thread. */
	qm35_thread_stop(qmspi);
	/* Free allocated structures. */
	qm35_free_device(qm);
	spi_set_drvdata(spi, NULL);
	return 0;
}

#if (KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE)
static void qm35_spi_driver_remove_void(struct spi_device *spi)
{
	qm35_spi_driver_remove(spi);
}
#define qm35_spi_driver_remove qm35_spi_driver_remove_void
#endif

enum {
	QM35,
};

static const struct of_device_id qm35_spi_of_ids[] = {
	{ .compatible = "qorvo,qm35_spi", .data = (void *)QM35 },
	{},
};
MODULE_DEVICE_TABLE(of, qm35_spi_of_ids);

static const struct spi_device_id qm35_spi_ids[] = {
	{ "qm35_spi", QM35 },
	{},
};
MODULE_DEVICE_TABLE(spi, qm35_spi_ids);

static struct spi_driver qm35_spi_driver = {
	.driver = {
		.name = "qm35_spi",
		.of_match_table = of_match_ptr(qm35_spi_of_ids),
		.pm = pm_ptr(&qm35_dev_pm_ops),
	},
	.id_table = qm35_spi_ids,
	.probe = qm35_spi_driver_probe,
	.remove = qm35_spi_driver_remove,
};

/**
 * qm35_spi_register_driver() - Register QM35 SPI device driver.
 * @sdrv: The SPI driver to register.
 *
 * Register the SPI framework device driver.
 *
 * Also handle the AUTO_TRACE bit in the debug_flags module parameter.
 *
 * Return: always 0.
 */
static inline int qm35_spi_register_driver(struct spi_driver *sdrv)
{
#ifdef CONFIG_EVENT_TRACING
	if (qm35_debug_flags & QMSPI_AUTO_TRACE) {
		trace_set_clr_event(THIS_MODULE->name, NULL, 1);
		trace_set_clr_event("qm35", NULL, 1);
	}
#endif
	if (qm35_debug_flags & QMSPI_AUTO_LOAD_OPTIONAL) {
		int i, r;
		for (i = 0; i < ARRAY_SIZE(optionnal_modules); i++) {
			r = request_module("%s", optionnal_modules[i]);
			if (r < 0) {
				pr_warn("Fail to auto-load %s.ko module!",
					optionnal_modules[i]);
			}
		}
	}
	return spi_register_driver(sdrv);
}

/**
 * qm35_spi_unregister_driver() - Unregister QM35 SPI device driver.
 * @sdrv: The SPI driver to unregister.
 *
 * Unregister the SPI framework device driver.
 *
 * Return: always 0.
 */
static inline void qm35_spi_unregister_driver(struct spi_driver *sdrv)
{
	spi_unregister_driver(sdrv);
#ifdef CONFIG_EVENT_TRACING
	if (qm35_debug_flags & QMSPI_AUTO_TRACE) {
		trace_set_clr_event(THIS_MODULE->name, NULL, 0);
		trace_set_clr_event("qm35", NULL, 0);
	}
#endif
}

module_driver(qm35_spi_driver, qm35_spi_register_driver,
	      qm35_spi_unregister_driver);

#ifdef GITVERSION
MODULE_VERSION(GITVERSION);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Girault <david.girault@qorvo.com>");
MODULE_DESCRIPTION("Qorvo QM35 SPI transport driver");
