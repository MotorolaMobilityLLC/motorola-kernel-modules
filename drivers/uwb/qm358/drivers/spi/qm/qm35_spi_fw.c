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
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>

#include "qm35_spi.h"
#include "qm35_spi_trc.h"
#include "spi_rom_protocol.h"
#include "qmrom_spi.h"
#include "qm357xx_fwpkg.h"
#include "fwupdater.h"

/* Newer kernels define FW_ACTION_UEVENT instead of FW_ACTION_HOTPLUG.
 * Define it for older kernels.
 */
#ifndef FW_ACTION_UEVENT
#define FW_ACTION_UEVENT FW_ACTION_HOTPLUG
#endif

#define QMROM_SPI_SPEED 0 /* Let libqmrom use its default value. */
#define QMROM_COMM_RETRIES 10 /* As seen in other libqmrom clients. */

static int qm35_fw_init_qmrom(struct qm35_spi *qmspi)
{
	if (!qmspi->irq_gpio) {
		dev_err(&qmspi->spi->dev, "IRQ GPIO not defined, "
					  "cannot use boot ROM command mode\n");
		return -EFAULT;
	}
	if (!qmspi->fw.handle) {
		qmrom_set_log_device(&qmspi->spi->dev, LOG_WARN);
		qmspi->fw.handle = qmrom_init(&qmspi->spi, &qmspi->reset_gpio,
					      &qmspi->irq_gpio, QMROM_SPI_SPEED,
					      QMROM_COMM_RETRIES,
					      qmrom_spi_reset_device,
					      DEVICE_GEN_UNKNOWN);
		if (!qmspi->fw.handle) {
			dev_err(&qmspi->spi->dev, "qmrom_init failed\n");
			return -EFAULT;
		}
		dev_info(
			&qmspi->spi->dev,
			"QM35 boot ROM found: Device ID %#06x, Chip revision %#04x\n",
			qmspi->fw.handle->device_version,
			qmspi->fw.handle->chip_rev);
	}
	return 0;
}

void qm35_fw_deinit_qmrom(struct qm35_spi *qmspi)
{
	if (!qmspi->fw.handle)
		return;

	qmrom_deinit(qmspi->fw.handle);
	qmspi->fw.handle = NULL;
	/* Always reset the chip at the end of a libqmrom session, to make sure
	 * it is not still in boot ROM command mode.
	 * QM358xx A0 boot ROM does not take care of this, for instance.
	 */
	qm35_spi_reset_wait_ready(qmspi, false, false);
}

/**
 * qm35_fw_flash_macro_pkg() - Flash a macro package.
 * @handle: libqmrom handle.
 * @fw: Firmware macro package to be flashed.
 *
 * Return: 0 if the firmware has been successfully upgraded else a negative error.
 */
static int qm35_fw_flash_macro_pkg(struct qmrom_handle *handle,
				   const struct firmware *fw)
{
	struct spi_device *spi = *(struct spi_device **)handle->spi_handle;
	const u8 *fw_data;
	u32 fw_size;
	int rc;

	rc = qm357xx_rom_fw_macro_pkg_get_fw_idx(fw, OEM_FW_IMAGE, &fw_size,
						 &fw_data);
	if (rc) {
		dev_err(&spi->dev,
			"qm357xx_rom_fw_macro_pkg_get_fw_idx failed with %d\n",
			rc);
		return rc;
	}
	if (*(u32 *)fw_data != CRYPTO_FIRMWARE_PACK_MAGIC_VALUE) {
		dev_err(&spi->dev, "Wrong magic for firmware package\n");
		return -EINVAL;
	}

	/* No need to wait for the firmware updater to finish booting,
	 * it's fast enough.
	 */
	handle->skip_check_fw_boot = true;
	rc = qm357xx_rom_flash_fw(handle, fw, FW_UPDATER);
	handle->skip_check_fw_boot = false;
	if (rc) {
		dev_err(&spi->dev, "qm357xx_rom_flash_fw failed with %d\n", rc);
		return rc;
	}

	/* Use the default value from spi_device. */
	qmrom_spi_set_freq(0);
	rc = run_fwupdater(handle, fw_data, fw_size);
	if (rc) {
		dev_err(&spi->dev, "run_fwupdater failed with %d\n", rc);
		return rc;
	}

	return rc;
}

/**
 * qm35_fw_upgrade_work() - Worker function for qm35_fw_upgrade().
 * @qmspi: QM35 SPI instance.
 * @in: Worker function input parameters.
 * @out: Worker function output buffer.
 *
 * Return: 0 if the firmware has been successfully upgraded else a negative error.
 */
static int qm35_fw_upgrade_work(struct qm35_spi *qmspi, const void *in,
				void *out)
{
	struct qmrom_handle *handle;
	int rc;

	trace_qm35_fw_upgrade_work(qmspi);

	/* Run the firmware upgrade. */
	rc = qm35_fw_init_qmrom(qmspi);
	if (rc)
		goto error;
	handle = qmspi->fw.handle;

	switch (qmspi->fw.fw_format) {
	case QM35FW_FW_PKG:
		/* Increase retry count to 100, as the QM358xx can take up to
		 * 50 ms to process and reply to some commands,
		 * and qmrom_spi_wait_for_ready_line() can return after 1 ms.
		 */
		handle->comms_retries = 100;
		rc = qm358xx_rom_load_fw_pkg(handle, qmspi->fw.fw_img);
		if (rc)
			dev_err(&qmspi->spi->dev,
				"qm358xx_rom_load_fw_pkg failed with %d\n", rc);
		break;
	case QM35FW_MACRO_PKG:
		rc = qm35_fw_flash_macro_pkg(handle, qmspi->fw.fw_img);
		break;
	default:
		/* No need to wait for the new firmware to finish booting,
		 * the driver will handle that.
		 */
		handle->skip_check_fw_boot = true;
		rc = qm357xx_rom_flash_fw(handle, qmspi->fw.fw_img,
					  NOT_MACRO_PKG);
		if (rc)
			dev_err(&qmspi->spi->dev,
				"qm357xx_rom_flash_fw failed with %d\n", rc);
	}
	qm35_fw_deinit_qmrom(qmspi);
error:
	qm35_fw_free(qmspi);
	trace_qm35_fw_upgrade_work_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_upgrade() - Upgrade the firmware of the QM35 chip.
 * @qmspi: QM35 SPI instance.
 *
 * Return: 0 if the firmware has been successfully upgraded else a negative error.
 */
int qm35_fw_upgrade(struct qm35_spi *qmspi)
{
	struct qm35_work work = { qm35_fw_upgrade_work, NULL, NULL, 0 };
	int rc;
	unsigned long start, duration;

	trace_qm35_fw_upgrade(qmspi);

	if (!qmspi) {
		rc = -EINVAL;
		goto error;
	}

	/* Check if a firmware is already loaded. */
	if (!qmspi->fw.is_loaded) {
		rc = -EFAULT;
		goto error;
	}

	dev_info(&qmspi->spi->dev, "Firmware upgrade started\n");
	/* Enqueue the firmware upgrade. */
	start = jiffies;
	rc = qm35_enqueue(qmspi, &work);
	duration = jiffies_delta_to_msecs(jiffies - start);
	if (!rc)
		dev_info(&qmspi->spi->dev,
			 "Firmware upgrade successful (%ld.%03ld seconds)\n",
			 duration / MSEC_PER_SEC, duration % MSEC_PER_SEC);

error:
	trace_qm35_fw_upgrade_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_get_device_id_work() - Worker function for qm35_fw_get_device_id().
 * @qmspi: QM35 SPI instance.
 * @in: Worker function input parameters.
 * @out: Worker function output buffer.
 *
 * Return: positive device id value if it could be obtained, else a negative error.
 */
static int qm35_fw_get_device_id_work(struct qm35_spi *qmspi, const void *in,
				      void *out)
{
	int rc;

	trace_qm35_fw_get_device_id_work(qmspi);

	/* Run the device probing. */
	rc = qm35_fw_init_qmrom(qmspi);
	if (rc)
		goto error;

	rc = qmspi->fw.handle->device_version;
	/* Do not call qm35_fw_deinit_qmrom() here, as the same libqmrom session
	 * can be used for the first firmware upgrade attempt.
	 */
error:
	trace_qm35_fw_get_device_id_work_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_get_device_id() - Get the QM35 device id from its bootrom.
 * @qmspi: QM35 SPI instance.
 *
 * Return: positive device id value if it could be obtained, else a negative error.
 */
int qm35_fw_get_device_id(struct qm35_spi *qmspi)
{
	struct qm35_work work = { qm35_fw_get_device_id_work, NULL, NULL, 0 };
	int rc;

	trace_qm35_fw_get_device_id(qmspi);

	if (!qmspi) {
		rc = -EINVAL;
		goto error;
	}

	/* Enqueue the device probing. */
	rc = qm35_enqueue(qmspi, &work);
error:
	trace_qm35_fw_get_device_id_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_get_fw_pkg_hdr_version() - Get the version of a firmware package header.
 * @qmspi: QM35 SPI instance.
 * @fw_data: Pointer to the firmware package header in the image.
 * @version: Pointer where the version will be stored.
 *
 * Return: 0 if the version is found else a negative error.
 */
static int qm35_fw_get_fw_pkg_hdr_version(struct qm35_spi *qmspi,
					  const u8 *fw_data,
					  struct qm35_fw_version *version)
{
	const char *fw_version;
	int rc;

	if (*(u32 *)fw_data != CRYPTO_FIRMWARE_PACK_MAGIC_VALUE) {
		dev_err(&qmspi->spi->dev,
			"Wrong magic for firmware package header\n");
		return -EINVAL;
	}

	fw_version = ((struct fw_pkg_hdr_t *)fw_data)->fw_version;
	if (fw_version[0] == '\0') {
		dev_err(&qmspi->spi->dev,
			"Firmware version not set in package header\n");
		return -EINVAL;
	}

	rc = sscanf(fw_version, "QMV=%hhu.%hhu.%hhurc%hhu_%llu",
		    &version->major, &version->minor, &version->patch,
		    &version->rc, &version->build_id);
	if (rc != 5) {
		dev_err(&qmspi->spi->dev,
			"Wrong format for firmware version\n");
		return -EINVAL;
	}
	version->oem_major = 0;
	version->oem_minor = 0;
	version->oem_patch = 0;

	return 0;
}

/**
 * qm35_fw_get_fw_pkg_version() - Get the version of a firmware package.
 * @qmspi: QM35 SPI instance.
 * @version: Pointer where the version will be stored.
 *
 * Return: 0 if the version is found else a negative error.
 */
static int qm35_fw_get_fw_pkg_version(struct qm35_spi *qmspi,
				      struct qm35_fw_version *version)
{
	if (!qmspi || !version)
		return -EINVAL;

	return qm35_fw_get_fw_pkg_hdr_version(qmspi, qmspi->fw.fw_img->data,
					      version);
}

/**
 * qm35_fw_get_macro_pkg_version() - Get the version of a firmware macro package.
 * @qmspi: QM35 SPI instance.
 * @version: Pointer where the version will be stored.
 *
 * Return: 0 if the version is found else a negative error.
 */
static int qm35_fw_get_macro_pkg_version(struct qm35_spi *qmspi,
					 struct qm35_fw_version *version)
{
	const u8 *fw_data;
	u32 fw_size;
	int rc;

	if (!qmspi || !version)
		return -EINVAL;

	rc = qm357xx_rom_fw_macro_pkg_get_fw_idx(qmspi->fw.fw_img, OEM_FW_IMAGE,
						 &fw_size, &fw_data);
	if (rc) {
		dev_err(&qmspi->spi->dev,
			"qm357xx_rom_fw_macro_pkg_get_fw_idx failed with %d\n",
			rc);
		return rc;
	}

	return qm35_fw_get_fw_pkg_hdr_version(qmspi, fw_data, version);
}

/**
 * qm35_fw_search_version() - Search for the version inside of a firmware binary.
 * @qmspi: QM35 SPI instance.
 * @version: Pointer where the version will be stored.
 *
 * Return: 0 if the version is found else a negative error.
 */
static int qm35_fw_search_version(struct qm35_spi *qmspi,
				  struct qm35_fw_version *version)
{
	const char marker[] = "UCIFWVER";
	const char *start, *end;
	int ml = sizeof(marker) - 1;

	if (!qmspi || !version)
		return -EINVAL;

	/* Search for UCI FW version marker on a 4-byte aligned boundary. */
	start = qmspi->fw.fw_img->data;
	end = start + qmspi->fw.fw_img->size - ml;
	while (start < end) {
		if (!memcmp(start, marker, ml))
			break;
		start += 4;
	}
	if ((start + sizeof(*version)) <= end) {
		/* Marker found. */
		start += ml;
		version->major = start[0];
		version->minor = start[1];
		version->patch = start[2];
		version->rc = start[3];
		version->build_id = get_unaligned_le64(&start[4]);
		version->oem_major = start[12];
		version->oem_minor = start[13];
		version->oem_patch = start[14];
		return 0;
	}
	return -EINVAL;
}

/**
 * qm35_fw_load() - Load a firmware file in memory.
 * @qmspi: QM35 SPI instance.
 * @fw_name: Name of the firmware file to load.
 *
 * Return: 0 if the firmware has been successfully loaded else a negative error.
 */
int qm35_fw_load(struct qm35_spi *qmspi, const char *fw_name)
{
	const char *fw_format;
	int rc;

	trace_qm35_fw_load(qmspi, fw_name);

	if (!qmspi || !fw_name || fw_name[0] == '\0') {
		rc = -EINVAL;
		goto error;
	}

	/* Check if a firmware is already loaded. */
	if (qmspi->fw.is_loaded) {
		rc = -EBUSY;
		goto error;
	}

	get_device(&qmspi->spi->dev);
	rc = request_firmware(&qmspi->fw.fw_img, fw_name,
				     &qmspi->spi->dev);
	put_device(&qmspi->spi->dev);
	if (rc) {
		if (rc == -ENOENT)
			dev_info(&qmspi->spi->dev, "Firmware %s not found\n",
				 fw_name);
		else
			dev_err(&qmspi->spi->dev,
				"request_firmware_direct of %s failed with %d\n",
				fw_name, rc);
		goto error;
	}

	qmspi->fw.is_loaded = true;
	if (*(u32 *)qmspi->fw.fw_img->data ==
	    CRYPTO_FIRMWARE_PACK_MAGIC_VALUE) {
		qmspi->fw.fw_format = QM35FW_FW_PKG;
		fw_format = "firmware package";
	} else if (*(u32 *)qmspi->fw.fw_img->data ==
		   CRYPTO_MACRO_FIRMWARE_PACK_MAGIC_VALUE) {
		qmspi->fw.fw_format = QM35FW_MACRO_PKG;
		fw_format = "macro package";
	} else {
		qmspi->fw.fw_format = QM35FW_STITCHED;
		fw_format = "stitched";
	}
	dev_info(&qmspi->spi->dev, "Firmware %s loaded (format: %s)\n", fw_name,
		 fw_format);

error:
	trace_qm35_fw_load_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_free() - Free the firmware from memory.
 * @qmspi: QM35 SPI instance.
 *
 * Return: 0 if the memory has been successfully freed else a negative error.
 */
int qm35_fw_free(struct qm35_spi *qmspi)
{
	int rc = 0;

	trace_qm35_fw_free(qmspi);

	if (!qmspi) {
		rc = -EINVAL;
		goto error;
	}

	if (qmspi->fw.is_loaded && qmspi->fw.fw_img)
		release_firmware(qmspi->fw.fw_img);
	qmspi->fw.is_loaded = false;
	qmspi->fw.fw_img = NULL;
error:
	trace_qm35_fw_free_return(qmspi, rc);
	return rc;
}

/**
 * qm35_fw_get_vendor_version() - Return the vendor version of a firmware binary.
 * @qmspi: QM35 SPI instance.
 * @version: Pointer where the version will be stored.
 *
 * Return: 0 if the version has been successfully read else a negative error.
 */
int qm35_fw_get_vendor_version(struct qm35_spi *qmspi,
			       struct qm35_fw_version *version)
{
	int rc;

	trace_qm35_fw_get_vendor_version(qmspi);

	if (!qmspi || !version) {
		rc = -EINVAL;
		goto error;
	}

	/* Check if a firmware is already loaded. */
	if (!qmspi->fw.is_loaded) {
		return -EFAULT;
		goto error;
	}

	switch (qmspi->fw.fw_format) {
	case QM35FW_FW_PKG:
		rc = qm35_fw_get_fw_pkg_version(qmspi, version);
		break;
	case QM35FW_MACRO_PKG:
		rc = qm35_fw_get_macro_pkg_version(qmspi, version);
		break;
	default:
		rc = qm35_fw_search_version(qmspi, version);
	}
error:
	trace_qm35_fw_get_vendor_version_return(qmspi, rc);
	return rc;
}
