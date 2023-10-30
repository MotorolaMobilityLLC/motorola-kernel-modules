// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2023 Qorvo US, Inc.
 *
 */

#ifndef __KERNEL__
#include <stddef.h>
#endif

#include <linux/uwb/qmrom_spi.h>
#include <linux/uwb/qmrom_log.h>
#include <linux/uwb/qmrom_utils.h>
#include <linux/uwb/spi_rom_protocol.h>

#include <linux/uwb/fwupdater.h>

/* Extract from C0 rom code */
#define MAX_CERTIFICATE_SIZE 0x400
#define MAX_CHUNK_SIZE 3072
#define WAIT_REBOOT_DELAY_MS 250
#define WAIT_SS_RDY_CHUNK_TIMEOUT 100
#define WAIT_SS_RDY_STATUS_TIMEOUT 10
#define RESULT_RETRIES 3
#define RESULT_CMD_INTERVAL_MS 50
#define CRC_TYPE uint32_t
#define CRC_SIZE (sizeof(CRC_TYPE))
#define TRANPORT_HEADER_SIZE (sizeof(struct stc) + CRC_SIZE)
#define EMERGENCY_SPI_FREQ 1000000 /* 1MHz */

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#ifndef __KERNEL__
_Static_assert(MAX_CHUNK_SIZE >= CRYPTO_IMAGES_CERT_PKG_SIZE);
_Static_assert(TRANPORT_HEADER_SIZE + MAX_CERTIFICATE_SIZE < MAX_CHUNK_SIZE);
#endif

/* local stats */
static int gstats_spi_errors;
static int gstats_ss_rdy_timeouts;

static int send_data_chunks(struct qmrom_handle *handle, char *data,
			    size_t size);
static int check_fw_boot(struct qmrom_handle *handle);

int run_fwupdater(struct qmrom_handle *handle, char *fwpkg_bin, size_t size)
{
	int rc;

	gstats_spi_errors = 0;
	gstats_ss_rdy_timeouts = 0;

	if (size < sizeof(struct fw_pkg_hdr_t) +
			   sizeof(struct fw_pkg_img_hdr_t) +
			   CRYPTO_IMAGES_CERT_PKG_SIZE +
			   CRYPTO_FIRMWARE_CHUNK_MIN_SIZE) {
		LOG_ERR("%s cannot extract enough data from fw package binary\n",
			__func__);
		return -EINVAL;
	}

	rc = send_data_chunks(handle, fwpkg_bin, size);
	if (rc) {
		LOG_ERR("Sending image failed with %d\n", rc);
		return rc;
	}
	return 0;
}

static int run_fwupdater_get_status(struct qmrom_handle *handle,
				    struct stc *hstc, struct stc *sstc,
				    struct fw_updater_status_t *status)
{
	uint32_t i = 0;
	CRC_TYPE *crc = (CRC_TYPE *)(hstc + 1);
	bool owa;
	memset(hstc, 0, TRANPORT_HEADER_SIZE + sizeof(*status));

	while (i++ < RESULT_RETRIES) {
		// Poll the QM
		sstc->all = 0;
		hstc->all = 0;
		*crc = 0;
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc, TRANPORT_HEADER_SIZE);
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_STATUS_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.pre_read = 1;
		*crc = 0;
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc, TRANPORT_HEADER_SIZE);
		// LOG_INFO("Pre-Read received:\n");
		// hexdump(LOG_INFO, sstc, sizeof(sstc));

		/* Stops the loop when QM has a result to share */
		owa = sstc->soc_flags.out_waiting;
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_STATUS_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.read = 1;
		hstc->len = sizeof(*status);
		*crc = 0;
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc,
				   TRANPORT_HEADER_SIZE + sizeof(*status));
		// LOG_INFO("Read received:\n");
		// hexdump(LOG_INFO, sstc, sizeof(*hstc) + sizeof(uint32_t));
		if (owa) {
			memcpy(status, sstc->payload, sizeof(*status));
			if (status->magic == FWUPDATER_STATUS_MAGIC)
				break;
		}
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_STATUS_TIMEOUT);
		// Failed to get the status, reduces the spi speed to
		// an emergency speed to maximize the chance to get the
		// final status
		qmrom_spi_set_freq(EMERGENCY_SPI_FREQ);
		gstats_spi_errors++;
	}
	if (status->magic != FWUPDATER_STATUS_MAGIC) {
		LOG_ERR("Timedout waiting for result\n");
		return -1;
	}
	return 0;
}

static CRC_TYPE checksum(const void *data, const size_t size)
{
	CRC_TYPE crc = 0;
	CRC_TYPE *ptr = (CRC_TYPE *)data;
	CRC_TYPE remainder = size & (CRC_SIZE - 1);
	size_t idx;

	for (idx = 0; idx < size; idx += CRC_SIZE, ptr++)
		crc += *ptr;

	if (remainder) {
		crc += ((uint8_t *)data)[size - 1];
		if (remainder > 1) {
			crc += ((uint8_t *)data)[size - 2];
			if (remainder > 2) {
				crc += ((uint8_t *)data)[size - 3];
			}
		}
	}
	return crc;
}

static void prepare_hstc(struct stc *hstc, char *data, size_t len)
{
	CRC_TYPE *crc = (CRC_TYPE *)(hstc + 1);
	void *payload = crc + 1;

	hstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->len = len + CRC_SIZE;
	*crc = checksum(data, len);
#ifdef CONFIG_INJECT_ERROR
	*crc += 2;
#endif
	memcpy(payload, data, len);
}

static int xfer_payload_prep_next(struct qmrom_handle *handle,
				  const char *step_name, struct stc *hstc,
				  struct stc *sstc, struct stc *hstc_next,
				  char **data, size_t *size)
{
	int rc = 0, nb_retry = CONFIG_NB_RETRIES;
	CRC_TYPE *crc = (CRC_TYPE *)(hstc + 1);

	do {
		int ss_rdy_rc, irq_up;
		sstc->all = 0;
		rc = qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
					(const char *)hstc,
					hstc->len + sizeof(struct stc));
		if (hstc_next) {
			/* Don't wait idle, prepare the next hstc to be sent */
			size_t to_send = MIN(MAX_CHUNK_SIZE, *size);
			prepare_hstc(hstc_next, *data, to_send);
			*size -= to_send;
			*data += to_send;
			hstc_next = NULL;
		}
		ss_rdy_rc = qmrom_spi_wait_for_ready_line(
			handle->ss_rdy_handle, WAIT_SS_RDY_CHUNK_TIMEOUT);
		if (ss_rdy_rc) {
			LOG_ERR("%s Waiting for ss-rdy failed with %d (nb_retry %d , crc 0x%x)\n",
				step_name, ss_rdy_rc, nb_retry, *crc);
			gstats_ss_rdy_timeouts++;
			rc = -EAGAIN;
		}
		irq_up = qmrom_spi_read_irq_line(handle->ss_irq_handle);
		if ((!rc && !sstc->soc_flags.ready) || irq_up) {
			LOG_ERR("%s Retry rc %d, sstc 0x%08x, irq %d, crc %08x\n",
				step_name, rc, sstc->all, irq_up, *crc);
			rc = -EAGAIN;
			gstats_spi_errors++;
		}
#ifdef CONFIG_INJECT_ERROR
		(*crc)--;
#endif
	} while (rc && --nb_retry > 0);
	if (rc) {
		LOG_ERR("%s transfer failed with %d - (sstc 0x%08x)\n",
			step_name, rc, sstc->all);
	}
	return rc;
}

static int xfer_payload(struct qmrom_handle *handle, const char *step_name,
			struct stc *hstc, struct stc *sstc)
{
	return xfer_payload_prep_next(handle, step_name, hstc, sstc, NULL, NULL,
				      NULL);
}

static int send_data_chunks(struct qmrom_handle *handle, char *data,
			    size_t size)
{
	struct fw_updater_status_t status;
	struct stc *hstc, *sstc, *hstc_current, *hstc_next;
	uint32_t chunk_nr = 0;
	char *rx, *tx;
	CRC_TYPE *crc;
	int rc = 0;

	LOG_DBG("chunk_nr:%u\n", chunk_nr);

	qmrom_alloc(rx, MAX_CHUNK_SIZE + TRANPORT_HEADER_SIZE);
	qmrom_alloc(tx, 2 * (MAX_CHUNK_SIZE + TRANPORT_HEADER_SIZE));
	if (!rx || !tx) {
		LOG_ERR("Rx/Tx buffers allocation failure\n");
		rc = -ENOMEM;
		goto exit_nomem;
	}

	sstc = (struct stc *)rx;
	hstc = (struct stc *)tx;
	hstc_current = hstc;
	hstc_next = (struct stc *)&tx[MAX_CHUNK_SIZE + TRANPORT_HEADER_SIZE];
	crc = (CRC_TYPE *)(hstc + 1);

	/* wait for the QM to be ready */
	rc = qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					   WAIT_SS_RDY_CHUNK_TIMEOUT);
	if (rc)
		LOG_ERR("Waiting for ss-rdy failed with %d\n", rc);

	/* Sending the fw package header */
	prepare_hstc(hstc, data, sizeof(struct fw_pkg_hdr_t));
	LOG_INFO("Sending the fw package header (%zu bytes, crc is 0x%08x)\n",
		 sizeof(struct fw_pkg_hdr_t), *crc);
	// hexdump(LOG_INFO, hstc->payload + 4, sizeof(struct fw_pkg_hdr_t));
	rc = xfer_payload(handle, "fw package header", hstc, sstc);
	if (rc)
		goto exit;
	/* Move the data to the next offset minus the header footprint */
	size -= sizeof(struct fw_pkg_hdr_t);
	data += sizeof(struct fw_pkg_hdr_t);

	/* Sending the image header */
	prepare_hstc(hstc, data, sizeof(struct fw_pkg_img_hdr_t));
	LOG_INFO("Sending the image header (%zu bytes crc 0x%08x)\n",
		 sizeof(struct fw_pkg_img_hdr_t), *crc);
	// hexdump(LOG_INFO, hstc->payload + 4, sizeof(struct fw_pkg_img_hdr_t));
	rc = xfer_payload(handle, "image header", hstc, sstc);
	if (rc)
		goto exit;
	size -= sizeof(struct fw_pkg_img_hdr_t);
	data += sizeof(struct fw_pkg_img_hdr_t);

	/* Sending the cert chain */
	prepare_hstc(hstc, data, CRYPTO_IMAGES_CERT_PKG_SIZE);
	LOG_INFO("Sending the cert chain (%d bytes crc 0x%08x)\n",
		 CRYPTO_IMAGES_CERT_PKG_SIZE, *crc);
	rc = xfer_payload(handle, "cert chain", hstc, sstc);
	if (rc)
		goto exit;
	size -= CRYPTO_IMAGES_CERT_PKG_SIZE;
	data += CRYPTO_IMAGES_CERT_PKG_SIZE;

	/* Sending the fw image */
	LOG_INFO("Sending the image (%zu bytes)\n", size);
	LOG_DBG("Sending a chunk (%zu bytes crc 0x%08x)\n",
		MIN(MAX_CHUNK_SIZE, size), *crc);
	prepare_hstc(hstc_current, data, MIN(MAX_CHUNK_SIZE, size));
	size -= hstc_current->len - CRC_SIZE;
	data += hstc_current->len - CRC_SIZE;
	do {
		rc = xfer_payload_prep_next(handle, "data chunk", hstc_current,
					    sstc, hstc_next, &data, &size);
		if (rc)
			goto exit;
		chunk_nr++;
		/* swap hstcs */
		hstc = hstc_current;
		hstc_current = hstc_next;
		hstc_next = hstc;
	} while (size);

	/* Sends the last now */
	rc = xfer_payload_prep_next(handle, "data chunk", hstc_current, sstc,
				    NULL, NULL, NULL);

exit:
	// tries to get the flashing status anyway...
	rc = run_fwupdater_get_status(handle, hstc, sstc, &status);
	if (!rc) {
		if (status.status) {
			LOG_ERR("Flashing failed, fw updater status %#x (errors: sub %#x, crc %u, rram %u, crypto %d)\n",
				status.status, status.suberror,
				status.crc_errors, status.rram_errors,
				status.crypto_errors);
			rc = status.status;
		} else {
			if (gstats_ss_rdy_timeouts + gstats_spi_errors +
			    status.crc_errors + status.rram_errors +
			    status.crypto_errors) {
				LOG_WARN(
					"Flashing succeeded with errors (host %u, ss_rdy_timeout %u, QM %u, crc %u, rram %u, crypto %d)\n",
					gstats_spi_errors,
					gstats_ss_rdy_timeouts,
					status.spi_errors, status.crc_errors,
					status.rram_errors,
					status.crypto_errors);
			} else {
				LOG_INFO(
					"Flashing succeeded without any errors\n");
			}

			if (!handle->skip_check_fw_boot)
				rc = check_fw_boot(handle);
		}
	} else {
		LOG_ERR("run_fwupdater_get_status returned %d\n", rc);
	}
exit_nomem:
	if (rx)
		qmrom_free(rx);
	if (tx)
		qmrom_free(tx);
	return rc;
}

static int check_fw_boot(struct qmrom_handle *handle)
{
	uint8_t raw_flags;
	struct stc hstc, sstc;

	handle->dev_ops.reset(handle->reset_handle);

	qmrom_msleep(WAIT_REBOOT_DELAY_MS);

	// Poll the QM
	sstc.all = 0;
	hstc.all = 0;
	while(sstc.all == 0)
		qmrom_spi_transfer(handle->spi_handle, (char *)&sstc,
					(const char *)&hstc, sizeof(hstc));

	raw_flags = sstc.raw_flags;
	/* The ROM code sends the same quartets for the first byte of each xfers */
	if (((raw_flags & 0xf0) >> 4) == (raw_flags & 0xf)) {
		LOG_ERR("%s: firmware not properly started: %#x\n",
			__func__, raw_flags);
		return -2;
	}
	return 0;
}
