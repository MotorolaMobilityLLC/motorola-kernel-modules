/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2024 Qorvo US, Inc.
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
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>

#include "qm35_hsspi.h"
#include "qm35_spi.h"
#include "qm35_spi_pm.h"
#include "qm35_spi_transport.h"
#include "qm35_spi_trc.h"
#include "qm35_transport.h"

#define QM35_COREDUMP_IN_PROGRESS 100

#if IS_ENABLED(CONFIG_QM35_FLASHING)

#ifndef CONFIG_QM35_FIRMWARE_DIR
#define CONFIG_QM35_FIRMWARE_DIR "qorvo/"
#endif

/* clang-format off */
#define QM35_FIRMWARE_ARRAY(series, revision)                              \
static const char *const qm35##series##_##revision##_fw_list[] = {         \
	CONFIG_QM35_FIRMWARE_DIR "qm35" #series "_" #revision "_prod.bin", \
	CONFIG_QM35_FIRMWARE_DIR "qm35" #series "_" #revision ".bin",      \
	CONFIG_QM35_FIRMWARE_DIR "qm35" #series "_prod.bin",               \
	CONFIG_QM35_FIRMWARE_DIR "qm35" #series ".bin",                    \
	CONFIG_QM35_FIRMWARE_DIR "qm35_prod.bin",                          \
	CONFIG_QM35_FIRMWARE_DIR "qm35.bin",                               \
	NULL                                                               \
}
QM35_FIRMWARE_ARRAY(7xx, b0);
QM35_FIRMWARE_ARRAY(7xx, c0);
QM35_FIRMWARE_ARRAY(8xx, a0);

static const char *const qm35_default_fw_list[] = {
	CONFIG_QM35_FIRMWARE_DIR "qm35_prod.bin",
	CONFIG_QM35_FIRMWARE_DIR "qm35.bin",
	NULL
};
/* clang-format on */

#endif

/**
 * info_read() - Read received device info.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @pos: Offset pointer.
 * @count: Buffer size.
 *
 * Returns: Written size.
 */
static ssize_t info_read(struct file *filp, struct kobject *kobp,
			 struct bin_attribute *bin_attr, char *buf, loff_t pos,
			 size_t count)
{
	struct qm35_spi *qmspi =
		container_of(bin_attr, struct qm35_spi, info_bin_attr);
	int ret;

	mutex_lock(&qmspi->info_mutex);
	ret = memory_read_from_buffer(buf, count, &pos, &qmspi->infobuf,
				      qmspi->len_infobuf);
	mutex_unlock(&qmspi->info_mutex);

	return ret;
}

/*
 * QM35 SPI transport implementation
 */

/**
 * qm35_spi_start() - QM35 transport start callback.
 * @qm35: QM35 core instance to start.
 *
 * Power-on the device so it can be used by the core
 * QM35 module and enable IRQ.
 *
 * See qm35_spi_pm_start() for implementation details.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_start(struct qm35 *qm35)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct device *dev = qmspi->base.dev;
	int rc;

	trace_qm35_spi_start(qmspi);
	dev_dbg(dev, "Starting\n");

	if (!try_module_get(THIS_MODULE)) {
		rc = -ENODEV;
		dev_err(dev, "Fail to increase refcnt for %s module!\n",
			THIS_MODULE->name);
		goto error;
	}

	/* Power-on device through PM runtime API. */
	rc = qm35_spi_pm_start(qmspi);
	if (rc < 0) {
		dev_err(dev, "Failure to start the QM35! (%d)\n", rc);
		module_put(THIS_MODULE);
	}
error:
	trace_qm35_spi_start_return(qmspi, rc);
	return rc;
}

/**
 * qm35_spi_stop() - QM35 transport stop callback.
 * @qm35: QM35 core instance to stop.
 *
 * Power-off the device and disable IRQ.
 *
 * See qm35_spi_power_supply() for implementation details.
 */
static void qm35_spi_stop(struct qm35 *qm35)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct device *dev = qmspi->base.dev;
	int rc;

	trace_qm35_spi_stop(qmspi);

	/* Power-down device through PM runtime API. */
	rc = qm35_spi_pm_stop(qmspi);
	if (rc < 0)
		dev_warn(dev, "Failure to stop PM for the QM35! (%d)\n", rc);

	module_put(THIS_MODULE);
	dev_dbg(dev, "Stopped\n");
	trace_qm35_spi_stop_return(qmspi, rc);
}

/**
 * qm35_spi_set_cs_level() - QM35 SPI set CS level.
 * @qmspi: QM35 SPI instance of which to set CS level.
 * @level: CS level to set.
 *
 * Use a bufferless SPI transfer to set the CS GPIO level.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_set_cs_level(struct qm35_spi *qmspi, bool level)
{
	int rc;

	struct spi_transfer xfer = {
		.cs_change = !level,
	};

	trace_qm35_spi_set_cs_level(qmspi);

	rc = spi_sync_transfer(qmspi->spi, &xfer, 1);
	trace_qm35_spi_set_cs_level_return(qmspi, rc);
	return rc;
}

/**
 * qm35_wait_ready_work() - Worker function for waiting chip ready.
 * @qmspi: QM35 SPI instance to issue reset.
 * @in: Worker function input parameters.
 * @out: Worker function output buffer.
 *
 * This function is used to ensure qm35_hsspi_recv() is called from inside the
 * qm35_spi_thread() function.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_wait_ready_work(struct qm35_spi *qmspi, const void *in,
				void *out)
{
	struct qm35_hsspi_header header;
	int loop = 100;
	int ret;

	while (loop) {
		/* Wait 10ms before first and between all flags read */
		mdelay(10);
		/* Read SOC flags only */
		ret = qm35_hsspi_recv(qmspi, &header, NULL, 0);
		if (ret < 0)
			break;
		/* Check SOC flag */
		if (header.flags & HSSPI_SOC_RDY) {
			/* Check if FW want to send a coredump. */
			if (header.flags & HSSPI_SOC_ODW &&
			    header.ul_value == QM35_TRANSPORT_MSG_COREDUMP)
				ret = QM35_COREDUMP_IN_PROGRESS;
			/* When the QM35 boots, the boot ROM first verifies its
			 * firmware signature, before jumping to it. In some
			 * chip revisions, the boot ROM also replies to HSSPI
			 * messages during this stage, by setting both bits
			 * HSSPI_SOC_RDY and HSSPI_SOC_BOOTROM_RDY high. This
			 * can be distinguished from replies by the firmware,
			 * where only HSSPI_SOC_RDY is high.
			 */
			if (!(header.flags & HSSPI_SOC_BOOTROM_RDY))
				break; /* SOC is really ready. */
		}
		/* Retry? */
		loop--;
	}
	return ret ?: loop ? 0 : -ETIMEDOUT;
}

/**
 * qm35_spi_reset_wait_ready() - QM35 SPI reset and wait ready.
 * @qmspi: QM35 SPI instance to reset.
 * @bootrom: Reset to bootrom command mode.
 * @wait: Wait QM35 ready after reset.
 *
 * Reset the device using the RESET GPIO from the DT.
 * The gpio cansleep api is used because RESET GPIO does not
 * require performance as READY/WAKEUP/EXTON GPIO.
 * For other gpio if the underlying GPIO chip driver can sleep
 * a warning will be traced at each accesses, which is not
 * useful for the RESET GPIO.
 *
 * Returns: Zero on success else a negative error code.
 */
int qm35_spi_reset_wait_ready(struct qm35_spi *qmspi, bool bootrom, bool wait)
{
	struct device *dev = qmspi->base.dev;
	struct qm35_work work;
	int ret = 0;

	trace_qm35_spi_reset(qmspi);
	disable_irq(qmspi->spi->irq);
	if (!(qm35_debug_flags & QMSPI_NO_RESET)) {
		dev_warn(dev, "Resetting\n");
		/* In original uci-kernel-module, the reset also changes the
		 * QM35 UCI char device state field to RESET before and UNKNOWN
		 * after the GPIO toggling. Further IRQ will change state again
		 * to READY.
		 */
		if (bootrom)
			/* Hold CS low if reset to bootrom was requested. */
			qm35_spi_set_cs_level(qmspi, 0);
		gpiod_set_value_cansleep(qmspi->reset_gpio, 1);
		usleep_range(QM35_RESET_DURATION_US,
			     QM35_RESET_DURATION_US + 100);
		gpiod_set_value_cansleep(qmspi->reset_gpio, 0);
		if (!bootrom) {
			/* Ensure minimum reset backoff duration, as the chip
			 * takes some time to exit reset state. */
			usleep_range(QM35_RESET_BACKOFF_DURATION_US,
				     2 * QM35_RESET_BACKOFF_DURATION_US);
		} else {
			/* Ensure bootrom-specific minimum reset backoff
			 * duration, as the chip takes some time to exit reset
			 * state and initialize its crypto IP, and only reads
			 * the boot selection pins afterwards. */
			usleep_range(
				QM35_BOOTROM_RESET_BACKOFF_DURATION_US,
				2 * QM35_BOOTROM_RESET_BACKOFF_DURATION_US);

			/* Reset CS level after reset to bootrom. */
			qm35_spi_set_cs_level(qmspi, 1);
		}
		/* Reset qm35_state. */
		qmspi->base.state = QM35_STATE_UNKNOWN;
	}
	if (bootrom && wait) {
		dev_warn(dev, "Bypassed waiting for QM35 ready, "
			      "resetting to bootrom command mode\n");
	} else if (wait) {
		dev_info(dev, "Waiting QM35 ready...\n");
		memset(&work, 0, sizeof(work));
		work.cmd = qm35_wait_ready_work;
		ret = qm35_enqueue(qmspi, &work);
		if (ret < 0)
			dev_warn(dev, "Reset timed out\n");
	}
	enable_irq(qmspi->spi->irq);
	if (ret == QM35_COREDUMP_IN_PROGRESS) {
		/* Wait coredump finished. */
		msleep(750);
		/* Ensure no error is returned. */
		ret = 0;
	}
	trace_qm35_spi_reset_return(qmspi, ret);
	return ret;
}

/**
 * qm35_spi_reset() - QM35 transport reset callback.
 * @qm35: QM35 core instance to reset.
 * @bootrom: Reset to bootrom command mode.
 *
 * Reset the device using the RESET GPIO from the DT and wait it to be ready.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_reset(struct qm35 *qm35, bool bootrom)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	/* If bootrom reset is requested, do not wait. */
	return qm35_spi_reset_wait_ready(qmspi, bootrom, !bootrom);
}

#if IS_ENABLED(CONFIG_QM35_FLASHING)

/**
 * qm35_spi_fw_update_single() - Attempt flashing a single firmware file.
 * @qm35: QM35 core instance to update.
 * @current_ver: Firmware version currently running on the QM35.
 * @fw_name: Name of the firmware file to load.
 * @force: Force the firmware update.
 *
 * Returns:
 * * 2 on success with firmware flashed;
 * * 1 on success without firmware flashed;
 * * 0 on failure without firmware flashed;
 * * else a negative error code.
 */
static int qm35_spi_fw_update_single(struct qm35 *qm35,
				     const struct qm35_fw_version *current_ver,
				     const char *fw_name, bool force)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct qm35_fw_version fw_ver;
	bool fw_version_found = false;
	bool run_fw_upgrade = false;
	int rc = 0;

	trace_qm35_spi_fw_update_single(qmspi, fw_name, force);

	if (qm35_fw_load(qmspi, fw_name))
		goto error;

	/* Get the version of qm35 firmware on the filesystem. */
	if (!qm35_fw_get_vendor_version(qmspi, &fw_ver)) {
		qm35_fw_version_print(dev_info, qm35->dev,
				      "Loaded firmware version", &fw_ver);
		fw_version_found = true;
	} else {
		dev_info(qm35->dev, "Loaded firmware version not found%s\n",
			 force ? "" : ", no firmware upgrade");
	}

	if (force) {
		run_fw_upgrade = true;
	} else if (fw_version_found) {
		/* Run the firmware upgrade if the versions differ, ignoring oem version. */
		if (qm35_fw_version_cmp(current_ver, &fw_ver)) {
			run_fw_upgrade = true;
		} else {
			dev_info(
				qm35->dev,
				"Currently running firmware version and loaded firmware "
				"version are identical, no firmware upgrade\n");
			/* In this case, returning without flashing is considered a success
			 * instead of a failure (file not found or not containing a version).
			 */
			rc = 1;
		}
	}

	if (run_fw_upgrade) {
		rc = qm35_fw_upgrade(qmspi);
		if (rc == -EINTR)
			/* If it was interrupted by a signal, don't reset and
			 * exit immediately.
			 */
			goto error;
		if (rc == 0)
			rc = 2;
	} else {
		/* If qm35_fw_upgrade() is called, the firmware will be freed by
		 * qm35_fw_upgrade_work(). Only call qm35_fw_free() otherwise.
		 */
		qm35_fw_free(qmspi);
	}
error:
	trace_qm35_spi_fw_update_single_return(qmspi, rc);
	return rc;
}

#endif

/**
 * qm35_spi_fw_update() - QM35 transport firmware update callback.
 * @qm35: QM35 core instance to update.
 * @current_ver: Firmware version currently running on the QM35.
 * @device_id: QM35 device_id.
 * @fw_name: Name of the firmware file to be loaded, replacing the default one.
 *
 * Call qm35_spi_fw_update_single() on given @fw_name or on a list of compatible
 * firmware file names until upgrade succeeds.
 *
 * Returns:
 * * 2 on success with firmware flashed;
 * * 1 on success without firmware flashed;
 * * 0 on failure without firmware flashed;
 * * else a negative error code.
 */
static int qm35_spi_fw_update(struct qm35 *qm35,
			      struct qm35_fw_version *current_ver,
			      u16 device_id, const char *fw_name)
{
#if IS_ENABLED(CONFIG_QM35_FLASHING)
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	bool force = false;
	int rc = 1;

	trace_qm35_spi_fw_update(qmspi, fw_name, !current_ver);
	mutex_lock(&qmspi->fw.update_lock);

	/* Disable IRQ to avoid the standard IRQ management that disturbs the
	 * firmware update process.
	 */
	disable_irq(qmspi->spi->irq);

	if (qm35_debug_flags & QMSPI_NO_FW_UPDATE) {
		dev_info(
			qm35->dev,
			"Firmware upgrade disabled by debug_flags module parameter\n");
		goto error;
	}

	if (qm35_debug_flags & QMSPI_FORCE_FW_UPDATE || !current_ver) {
		const char *force_cause;

		force = true;
		if (qm35_debug_flags & QMSPI_FORCE_FW_UPDATE)
			force_cause = "debug_flags module parameter";
		else
			force_cause = "firmware version not available";

		dev_warn(qm35->dev, "Firmware upgrade triggered by %s\n",
			 force_cause);
	}

	if (fw_name && fw_name[0]) {
		/* If fw_name was passed as an argument, use it. */
		rc = qm35_spi_fw_update_single(qm35, current_ver, fw_name,
					       force);
	} else if (!sysfs_streq(qm35_fw_name, "")) {
		/* Else, use qm35_fw_name if set to a non-empty string. */
		rc = qm35_spi_fw_update_single(qm35, current_ver, qm35_fw_name,
					       force);
	} else {
		/* Else, derive list of compatible firmware names from device id. */
		const char *const *fw_list;
		int i = 0;

		if (!device_id)
			device_id = qm35_fw_get_device_id(qmspi);

		switch (device_id) {
		case 0x420:
			fw_list = qm357xx_b0_fw_list;
			break;
		case 0x430:
			fw_list = qm357xx_c0_fw_list;
			break;
		case 0x440:
			fw_list = qm358xx_a0_fw_list;
			break;
		default:
			fw_list = qm35_default_fw_list;
		}
		do {
			rc = qm35_spi_fw_update_single(qm35, current_ver,
						       fw_list[i], force);
			if (rc < 0)
				/* As soon as a flashing attempt fails, we can't
				 * skip flashing based on the currently running
				 * version anymore.
				 */
				force = true;
		} while (rc < 1 && fw_list[++i] && rc != -EINTR);
	}
error:
	if (rc < 0)
		dev_err(qm35->dev, "Firmware upgrade failed with %d\n", rc);
	else if (rc == 0)
		dev_warn(qm35->dev,
			 "Firmware upgrade failed "
			 "(file not found or not containing a version)\n");
	qm35_fw_deinit_qmrom(qmspi);
	enable_irq(qmspi->spi->irq);
	mutex_unlock(&qmspi->fw.update_lock);
	trace_qm35_spi_fw_update_return(qmspi, rc);
	return rc;
#else
	dev_warn(qm35->dev, "Firmware upgrade disabled at built time. "
			    "Use qm-flashing tool.\n");
	return 0;
#endif
}

/**
 * qm35_spi_power() - QM35 transport power callback.
 * @qm35: QM35 core instance to power manage.
 * @on: Int that decides if we power on or off.
 *
 * Force power management using PM runtime management functions.
 *
 * Used to power down the device while keeping the UCI char device open.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_power(struct qm35 *qm35, int on)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	int rc = 0;
	trace_qm35_spi_power(qmspi, on);
	if (on) {
		if (!qmspi->started)
			rc = qm35_spi_pm_start(qmspi);
	} else {
		if (qmspi->started)
			rc = qm35_spi_pm_stop(qmspi);
	}
	trace_qm35_spi_power_return(qmspi, rc);
	return rc;
}

/**
 * qm35_send_work() - Worker function for sending.
 * @qmspi: QM35 SPI instance to send frame to.
 * @in: Worker function input parameters.
 * @out: Worker function output buffer.
 *
 * This function is use to ensure qm35_hsspi_send() is called from inside the
 * qm35_spi_thread() function.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_send_work(struct qm35_spi *qmspi, const void *in, void *out)
{
	const struct qm35_spi_work_params *params =
		(const struct qm35_spi_work_params *)in;
	int ret;

	/* Ensure the device is wake-up. */
	ret = qm35_hsspi_wakeup(qmspi, params->wakeup);
	if (ret == -EINPROGRESS)
		return ret; /* Async wakeup activated. */
	/* If queued send already made by qm35_recv_job(), return known result. */
	if ((qm35_debug_flags & QMSPI_COMBINED_WRITE) &&
	    !atomic_read(&qmspi->should_write))
		return qmspi->work_send.ret;
	/* Direct cast of type because enum match expected ul_value. */
	ret = qm35_hsspi_send(qmspi, (u8)params->type, params->data_out,
			      params->size);
	if (!ret && (qm35_debug_flags & QMSPI_COMBINED_WRITE))
		atomic_set(&qmspi->should_write, false);
	return ret;
}

/**
 * qm35_spi_send() - QM35 transport send callback.
 * @qm35: QM35 core instance.
 * @type: Message type to send.
 * @data: Pointer to payload data to send.
 * @size: Size of payload payload data to send.
 *
 * Send an HSSPI frame to QM35 SPI device.
 *
 * Calls qm35_hsspi_send() through the high-prio thread to send a frame to the
 * QM35 HW.
 *
 * It also ensures the QM35 HW is powered by calling the qm35_enqueue() function
 * between the qm35_spi_pm_resume() and qm35_spi_pm_idle() functions.
 *
 * Returns: Zero on success else a negative error code.
 */
static int qm35_spi_send(struct qm35 *qm35, enum qm35_transport_msg_type type,
			 const void *data, size_t size)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct device *dev = qmspi->base.dev;
	struct qm35_spi_work_params params = {
		.type = type,
		.data_out = data,
		.size = size,
		.wakeup = false,
	};
	bool wakeup_forced = false;
	int retry_count = QM35_HSSPI_RETRY_COUNT;
	int retry_udelay = QM35_HSSPI_RETRY_DELAY_US;
	int ret;

	/* FIXME: Two calls of qm35_spi_send from different thread may override
	 * this structure. This should be protected soon! Regression added when
	 * support for full-duplex exchanges was added. Transport and bypass
	 * APIs may guaranteed this at higher level. */

	/* Copy send params to qmspi structure. */
	qmspi->send_params = params;
	/* This MUST be set after all other parameters saved in qmspi!
	 * Especially 'send_params'. */
	if (qm35_debug_flags & QMSPI_COMBINED_WRITE)
		atomic_set(&qmspi->should_write, true);
	trace_qm35_spi_send(qmspi);
	dev_dbg(dev, "Sending...\n");
	ret = qm35_spi_pm_resume(qmspi);
	if (ret)
		goto error;

	ret = qm35_enqueue(qmspi, &qmspi->work_send);
	while ((ret == -EAGAIN || ret == -EBUSY) && retry_count--) {
		usleep_range(retry_udelay, 2 * retry_udelay);
		retry_udelay *= 2;
		if (!qmspi->exton_gpio) {
			qmspi->send_params.wakeup = false;
			/* Update send params to force wakeup if needed. */
			if (ret == -EBUSY && !wakeup_forced) {
				qmspi->send_params.wakeup = true;
				wakeup_forced = true;
				/* If the chip is sleeping on last retry, retry
				 * once more. */
				retry_count = !retry_count ? 1 : retry_count;
			}
		}
		ret = qm35_enqueue(qmspi, &qmspi->work_send);
		if (ret == -EINPROGRESS) {
			/* Async wakeup in progress, wait IRQ from outside
			 * high-prio thread. If QMSPI_COMBINED_WRITE is set,
			 * the packet will be sent while reading the AWAKE
			 * packet. */
			ret = wait_event_interruptible_hrtimeout(
				qmspi->wakeup_wait, qmspi->wakeup_event,
				ns_to_ktime(QM35_WAKEUP_DELAY_US * 2000));
			trace_qm35_spi_send_awake(qmspi, ret);
			/* If async wakeup used, don't count last try as a try. */
			retry_count++;
			/* Ensure no forced wakeup for next call. */
			ret = -EAGAIN;
		}
	}
	if (ret && (qm35_debug_flags & QMSPI_COMBINED_WRITE))
		atomic_set(&qmspi->should_write, false);
	/* Request auto-suspend in all cases. */
	qm35_spi_pm_idle(qmspi);
error:
	if (ret)
		dev_warn(dev, "Sending failed, device not ready! (%d)\n", ret);
	trace_qm35_spi_send_return(qmspi, ret);
	return ret;
}

/**
 * qm35_recv_work() - Worker function for receiving.
 * @qmspi: QM35 SPI instance to send frame to.
 * @in: Worker function input parameters.
 * @out: Worker function output buffer.
 *
 * This function is use to ensure qm35_hsspi_recv() is called from inside the
 * qm35_spi_thread() function.
 *
 * Returns: Received payload length on success else a negative error code.
 */
static int qm35_recv_work(struct qm35_spi *qmspi, const void *in, void *out)
{
	struct qm35_spi_work_params *params =
		(struct qm35_spi_work_params *)out;
	struct qm35_hsspi_header header;
	int ret;

	/* Ensure the device is wake-up. */
	qm35_hsspi_wakeup(qmspi, false);
	/* Reset should_read flag so next send won't do PRD. */
	atomic_set(&qmspi->should_read, false);
	/* Read available message to provided buffer. */
	ret = qm35_hsspi_recv(qmspi, &header, params->data_in, params->size);
	if (ret < 0)
		return ret;
	params->type = (enum qm35_transport_msg_type)header.ul_value;
	params->flags = (int)header.flags;
	return ret;
}

/**
 * qm35_spi_recv() - QM35 transport receive callback.
 * @qm35: QM35 core instance.
 * @data: Pointer to buffer where to store payload data.
 * @size: Size of buffer where to store payload data.
 * @type: Pointer to store received message type.
 * @flags: Pointer to store received message flags.
 *
 * Receive an HSSPI frame from QM35 SPI device and return payload, type and
 * flags in the provided pointers.
 *
 * This function use qm35_enqueue() to ensure SPI transfers always occurs
 * inside the QM35 SPI thread, qm35_spi_thread().
 *
 * This function is always called from the qm35_transport_event() in reply
 * to an IRQ. So QM35 chip is available, no need to wake it up.
 *
 * Calls qm35_hsspi_recv() through the high-prio thread to read a frame from the
 * QM35 HW.
 *
 * It also ensures the QM35 HW is kept powered by calling the qm35_enqueue()
 * function between the qm35_spi_pm_resume() and qm35_spi_pm_idle() functions.
 *
 * The latest call also ensures the PM framework resets auto-suspend delay.
 *
 * Returns: The length of payload data received or a negative error.
 */
static ssize_t qm35_spi_recv(struct qm35 *qm35, void *data, size_t size,
			     enum qm35_transport_msg_type *type, int *flags)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct device *dev = qmspi->base.dev;
	struct qm35_spi_work_params params = {
		.type = 0,
		.flags = 0,
		.data_in = data,
		.size = size,
	};
	int retry_count = QM35_HSSPI_RETRY_COUNT;
	int retry_udelay = QM35_HSSPI_RETRY_DELAY_US;
	int ret;
	qmspi->work_recv.out = &params;
	trace_qm35_spi_recv(qmspi);
	dev_dbg(dev, "Receiving...\n");
	ret = qm35_spi_pm_resume(qmspi);
	if (ret)
		goto error;
	/* Receive with retry. */
	ret = qm35_enqueue(qmspi, &qmspi->work_recv);
	while (ret == -EAGAIN && retry_count--) {
		usleep_range(retry_udelay, 2 * retry_udelay);
		retry_udelay *= 2;
		ret = qm35_enqueue(qmspi, &qmspi->work_recv);
	}
	if (ret >= 0) {
		*type = params.type;
		*flags = params.flags;
	} else {
		/* Need to redo PRD. */
		atomic_set(&qmspi->prd_done, false);
	}
	/* Request auto-suspend in all cases. */
	qm35_spi_pm_idle(qmspi);
error:
	trace_qm35_spi_recv_return(qmspi, ret);
	return ret;
}

/**
 * qm35_spi_probe() - QM35 transport probe callback.
 * @qm35: QM35 core instance to probe.
 * @infobuf: Buffer to fill with the uci device info.
 * @len: Length of the buffer.
 *
 * Probe the device using the UCI probe functions.
 *
 * This callback is called in qm35_device_register() to do the device
 * probing and is called right after qm35_spi_reset().
 *
 * It calls qm35_uci_probe_setup() to install UCI packet handler, then waits
 * for the state notification.
 *
 * It sends a soft-reset command using qm35_uci_probe_device_reset() if there
 * is no notification after the hard-reset.
 *
 * It also retrieves the device info using qm35_uci_probe_device_info().
 *
 * Finally, it calls qm35_uci_probe_cleanup() to remove the UCI packet handler
 * before returning.
 *
 * Returns: 0 on success else a negative error code.
 */
static int qm35_spi_probe(struct qm35 *qm35, char *infobuf, size_t len)
{
	struct qm35_spi *qmspi = qm35_to_qm35_spi(qm35);
	struct device *dev = qm35->dev;
	bool soft_reset_sent = false;
	int rc;

	trace_qm35_spi_probe(qmspi);
	/* Install packets handler for probing. */
	qm35_uci_probe_setup(qmspi);

	/* Wait QM35's state change. */
	while (qm35_state_wait(qm35, QM35_STATE_READY) <= 0) {
		/* Exit loop with error if second try (soft-reset) failed. */
		if (soft_reset_sent) {
			rc = -ETIMEDOUT;
			goto cleanup_probe;
		}
		/* Issue a soft reset command if there is no state
	 	 * notification after the HARD reset. */
		rc = qm35_uci_probe_device_reset(qmspi);
		if (rc)
			/* Force the firmware upgrade on the first
		 	* soft reset error or exit with error. */
			goto cleanup_probe;
		soft_reset_sent = true;
	}

	rc = qm35_uci_probe_device_info(
		qmspi, (struct qm35_uci_device_info *)infobuf, len);
	if (rc)
		goto cleanup_probe;

	mutex_lock(&qmspi->info_mutex);
	/* Save info */
	qmspi->len_infobuf = min(
		sizeof(qmspi->infobuf),
		sizeof(struct qm35_uci_device_info) +
			((struct qm35_uci_device_info *)infobuf)->vendor_length);
	memcpy(&qmspi->infobuf, infobuf, qmspi->len_infobuf);
	/* Remove sysfs control file. */
	if (qmspi->info_bin_attr.attr.name)
		sysfs_remove_bin_file(&dev->kobj, &qmspi->info_bin_attr);
	/* Create info file. */
	sysfs_bin_attr_init(&qmspi->info_bin_attr);
	qmspi->info_bin_attr.size = qmspi->len_infobuf;
	qmspi->info_bin_attr.read = info_read;
	qmspi->info_bin_attr.attr.mode = 0444; /* RO */
	qmspi->info_bin_attr.attr.name = "fwinfo";
	rc = sysfs_create_bin_file(&dev->kobj, &qmspi->info_bin_attr);
	if (rc) {
		dev_warn(dev,
			 "Cannot create fwinfo file with probed data. (%d)\n",
			 rc);
		qmspi->info_bin_attr.attr.name = NULL;
		rc = 0; /* Ignore this error. */
	}
	mutex_unlock(&qmspi->info_mutex);

cleanup_probe:
	qm35_uci_probe_cleanup(qmspi);
	trace_qm35_spi_probe_return(qmspi, rc);
	return rc;
}

static const struct qm35_transport_ops qm35_spi_transport_ops = {
	.start = qm35_spi_start,
	.stop = qm35_spi_stop,
	.reset = qm35_spi_reset,
	.fw_update = qm35_spi_fw_update,
	.power = qm35_spi_power,
	.send = qm35_spi_send,
	.recv = qm35_spi_recv,
	.probe = qm35_spi_probe,
};

const struct qm35_transport qm35_spi_transport = {
	.name = "qm35_spi",
	.ops = &qm35_spi_transport_ops
};

/**
 * qm35_spi_transport_setup() - Setup required fields in struct qm35_spi.
 * @qmspi: The QM35 SPI device instance to setup.
 */
void qm35_spi_transport_setup(struct qm35_spi *qmspi)
{
	qmspi->work_recv.cmd = qm35_recv_work;
	qmspi->work_send.cmd = qm35_send_work;
	qmspi->work_send.in = &qmspi->send_params;
}
