// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2023 Qorvo US, Inc.
 *
 */

#ifndef __KERNEL__
#include <stddef.h>
#endif

#include <qmrom_spi.h>
#include <qmrom_log.h>
#include <qmrom_utils.h>
#include <spi_rom_protocol.h>

#include <fwupdater.h>

/* Extract from C0 rom code */
#define MAX_CERTIFICATE_SIZE 0x400
#define MAX_CHUNK_SIZE 3072
#define WAIT_REBOOT_DELAY_MS 250
#define WAIT_SS_IRQ_AFTER_RESET_TIMEOUT_MS 450
#define WAIT_SS_IRQ_CHUNK_TIMEOUT 100
#define WAIT_SS_IRQ_STATUS_TIMEOUT 10
#define RESULT_RETRIES 3
#define RESULT_POLL_RETRIES 20
#define RESULT_CMD_INTERVAL_MS 50
#define CKSUM_TYPE uint32_t
#define CKSUM_SIZE (sizeof(CKSUM_TYPE))
#define TRANPORT_HEADER_SIZE (sizeof(struct stc) + CKSUM_SIZE)
#define EMERGENCY_SPI_FREQ 1000000 /* 1MHz */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef __KERNEL__
_Static_assert(MAX_CHUNK_SIZE >= CRYPTO_IMAGES_CERT_PKG_SIZE);
_Static_assert(TRANPORT_HEADER_SIZE + MAX_CERTIFICATE_SIZE < MAX_CHUNK_SIZE);
#endif

#ifdef WRITE_STATS
static uint64_t total_time_ns;
static uint32_t total_bytes, total_chunks;
static uint32_t max_write_time_ns, min_write_time_ns = ~0U;

static void update_write_stats(ktime_t start_time, uint32_t chunk_size)
{
	uint64_t elapsed_time_ns =
		ktime_to_ns(ktime_sub(ktime_get(), start_time));

	total_time_ns += elapsed_time_ns;
	total_bytes += MAX_CHUNK_SIZE;
	total_chunks++;

	if (elapsed_time_ns > max_write_time_ns)
		max_write_time_ns = elapsed_time_ns;
	if (elapsed_time_ns < min_write_time_ns)
		min_write_time_ns = elapsed_time_ns;
}

static void dump_stats(void)
{
	LOG_WARN(
		"Updater flashing time stats: %u bytes over %llu us (max chunk size %u, "
		"%u chunks, write timings: mean %llu us, min %u us, max %u us)\n",
		total_bytes, div_u64(total_time_ns, 1000), MAX_CHUNK_SIZE,
		total_chunks, div_u64(total_time_ns, total_chunks * 1000),
		min_write_time_ns / 1000, max_write_time_ns / 1000);

	/* Reset stats */
	total_time_ns = 0;
	total_bytes = 0;
	total_chunks = 0;
	max_write_time_ns = 0;
	min_write_time_ns = ~0U;
}
#endif

/* local stats */
static int gstats_spi_errors;
static int gstats_ss_irq_timeouts;

static int send_data_chunks(struct qmrom_handle *handle, const char *data,
			    size_t size);

int run_fwupdater(struct qmrom_handle *handle, const char *fwpkg_bin,
		  size_t size)
{
	int rc;

	gstats_spi_errors = 0;
	gstats_ss_irq_timeouts = 0;

	if (size < sizeof(struct fw_pkg_hdr_t) +
			   sizeof(struct fw_pkg_img_hdr_t) +
			   CRYPTO_IMAGES_CERT_PKG_SIZE +
			   CRYPTO_FIRMWARE_CHUNK_MIN_SIZE) {
		LOG_ERR("Cannot extract enough data from fw package binary\n");
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
	int i, j;
	CKSUM_TYPE *cksum = (CKSUM_TYPE *)(hstc + 1);
	bool owa, oa;
	memset(hstc, 0, TRANPORT_HEADER_SIZE + sizeof(*status));

	for (i = 0; i < RESULT_RETRIES; ++i) {
		// Poll the QM until ODW is set
		for (j = 0; j < RESULT_POLL_RETRIES; ++j) {
			sstc->all = 0;
			hstc->all = 0;
			*cksum = 0;
			qmrom_spi_transfer_delay(handle->spi_handle,
						 (char *)sstc,
						 (const char *)hstc,
						 TRANPORT_HEADER_SIZE);
			owa = sstc->soc_flags.out_waiting;
			if (owa)
				break;
			qmrom_msleep(WAIT_SS_IRQ_STATUS_TIMEOUT);
		}
		if (!owa)
			goto err;

		// Then send a pre-read
		qmrom_spi_wait_for_irq_line(handle->ss_irq_handle,
					    WAIT_SS_IRQ_STATUS_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.pre_read = 1;
		*cksum = 0;
		qmrom_spi_transfer_delay(handle->spi_handle, (char *)sstc,
					 (const char *)hstc,
					 TRANPORT_HEADER_SIZE);
		LOG_DBG("Pre-Read received:\n");
		hexdump(LOG_DBG, sstc, sizeof(sstc));

		owa = sstc->soc_flags.out_waiting;
		if (!owa || !sstc->len)
			goto err;

		// And finally send a read
		qmrom_spi_wait_for_irq_line(handle->ss_irq_handle,
					    WAIT_SS_IRQ_STATUS_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.read = 1;
		hstc->len = sizeof(*status);
		*cksum = 0;
		qmrom_spi_transfer_delay(
			handle->spi_handle, (char *)sstc, (const char *)hstc,
			TRANPORT_HEADER_SIZE + sizeof(*status));
		LOG_DBG("Read received:\n");
		hexdump(LOG_DBG, sstc, TRANPORT_HEADER_SIZE + sstc->len);

		oa = sstc->soc_flags.out_active;
		if (oa && sstc->len >= sizeof(*status)) {
			memcpy(status, sstc->payload, sizeof(*status));
			if (status->magic == FWUPDATER_STATUS_MAGIC)
				break;
		}

	err:
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

static CKSUM_TYPE checksum(const void *data, const size_t size)
{
	CKSUM_TYPE cksum = 0;
	CKSUM_TYPE *ptr = (CKSUM_TYPE *)data;
	CKSUM_TYPE remainder = size & (CKSUM_SIZE - 1);
	size_t idx;

	for (idx = 0; idx < size; idx += CKSUM_SIZE, ptr++)
		cksum += *ptr;

	if (!remainder)
		return cksum;

	while (remainder) {
		cksum += ((uint8_t *)data)[size - remainder];
		remainder--;
	}

	return cksum;
}

static void prepare_hstc(struct stc *hstc, const char *data, size_t len)
{
	CKSUM_TYPE *cksum = (CKSUM_TYPE *)(hstc + 1);
	void *payload = cksum + 1;

	hstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->len = len + CKSUM_SIZE;
	*cksum = checksum(data, len);
#if IS_ENABLED(CONFIG_INJECT_ERROR)
	*cksum += 2;
#endif
	memcpy(payload, data, len);
}

static int poll_err_flag(struct qmrom_handle *handle, struct stc *sstc)
{
	/* At the time poll_err_flag() is called, the hstc_next struct has
	 * likely already been filled by prepare_hstc(), and the hstc struct
	 * cannot be overwritten, in case it needs to be retransmitted.
	 *
	 * So, since we're only transmitting TRANPORT_HEADER_SIZE bytes, reuse
	 * sstc payload to store the hstc buffer.
	 */
	const char *hstc_buf = (char *)sstc->payload + CKSUM_SIZE;

	memset(sstc, 0, 2 * TRANPORT_HEADER_SIZE);
	qmrom_spi_transfer_delay(handle->spi_handle, (char *)sstc, hstc_buf,
				 TRANPORT_HEADER_SIZE);
	return sstc->soc_flags.err;
}

static int xfer_payload_prep_next(struct qmrom_handle *handle,
				  const char *step_name, struct stc *hstc,
				  struct stc *sstc, struct stc *hstc_next,
				  const char **data, size_t *size)
{
	int rc = 0, nb_retry = CONFIG_NB_RETRIES;
	CKSUM_TYPE *cksum = (CKSUM_TYPE *)(hstc + 1);
#ifdef WRITE_STATS
	ktime_t start_time;
#endif

	do {
		int ss_irq_rc, err;

		/* Wait for the QM to be ready before sending the payload */
		ss_irq_rc = qmrom_spi_wait_for_irq_line(
			handle->ss_irq_handle, WAIT_SS_IRQ_CHUNK_TIMEOUT);
		if (ss_irq_rc) {
			LOG_ERR("%s Waiting for IRQ failed with %d (nb_retry %d , cksum 0x%x)\n",
				step_name, ss_irq_rc, nb_retry, *cksum);
			gstats_ss_irq_timeouts++;
			rc = -EAGAIN;
		}

		/* Send the payload */
		sstc->all = 0;
#ifdef WRITE_STATS
		start_time = ktime_get();
#endif
		rc = qmrom_spi_transfer_delay(handle->spi_handle, (char *)sstc,
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

		/* Wait for the QM to be ready before sending the poll */
		ss_irq_rc = qmrom_spi_wait_for_irq_line(
			handle->ss_irq_handle, WAIT_SS_IRQ_CHUNK_TIMEOUT);
#ifdef WRITE_STATS
		update_write_stats(start_time, hstc->len - CKSUM_SIZE);
#endif
		if (ss_irq_rc) {
			LOG_ERR("%s Waiting for IRQ failed with %d (nb_retry %d , cksum 0x%x)\n",
				step_name, ss_irq_rc, nb_retry, *cksum);
			gstats_ss_irq_timeouts++;
			rc = -EAGAIN;
		}

		err = poll_err_flag(handle, sstc);
		if ((!rc && !sstc->soc_flags.ready) || err) {
			LOG_ERR("%s Retry rc %d, sstc 0x%08x, err %d, cksum %08x\n",
				step_name, rc, sstc->all, err, *cksum);
			rc = -EAGAIN;
			gstats_spi_errors++;
		}
#if IS_ENABLED(CONFIG_INJECT_ERROR)
		(*cksum)--;
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

static int send_data_chunks(struct qmrom_handle *handle, const char *data,
			    size_t size)
{
	struct fw_updater_status_t status;
	struct stc *hstc, *sstc, *hstc_current, *hstc_next;
	char *rx, *tx;
	CKSUM_TYPE *cksum;
	int rc = 0;

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
	cksum = (CKSUM_TYPE *)(hstc + 1);

	/* Sending the fw package header */
	prepare_hstc(hstc, data, sizeof(struct fw_pkg_hdr_t));
	LOG_INFO("Sending the fw package header (%zu bytes, cksum is 0x%08x)\n",
		 sizeof(struct fw_pkg_hdr_t), *cksum);
	// hexdump(LOG_INFO, hstc->payload + 4, sizeof(struct fw_pkg_hdr_t));
	rc = xfer_payload(handle, "fw package header", hstc, sstc);
	if (rc)
		goto exit;
	/* Move the data to the next offset minus the header footprint */
	size -= sizeof(struct fw_pkg_hdr_t);
	data += sizeof(struct fw_pkg_hdr_t);

	/* Sending the image header */
	prepare_hstc(hstc, data, sizeof(struct fw_pkg_img_hdr_t));
	LOG_INFO("Sending the image header (%zu bytes cksum 0x%08x)\n",
		 sizeof(struct fw_pkg_img_hdr_t), *cksum);
	// hexdump(LOG_INFO, hstc->payload + 4, sizeof(struct fw_pkg_img_hdr_t));
	rc = xfer_payload(handle, "image header", hstc, sstc);
	if (rc)
		goto exit;
	size -= sizeof(struct fw_pkg_img_hdr_t);
	data += sizeof(struct fw_pkg_img_hdr_t);

	/* Sending the cert chain */
	prepare_hstc(hstc, data, CRYPTO_IMAGES_CERT_PKG_SIZE);
	LOG_INFO("Sending the cert chain (%d bytes cksum 0x%08x)\n",
		 CRYPTO_IMAGES_CERT_PKG_SIZE, *cksum);
	rc = xfer_payload(handle, "cert chain", hstc, sstc);
	if (rc)
		goto exit;
	size -= CRYPTO_IMAGES_CERT_PKG_SIZE;
	data += CRYPTO_IMAGES_CERT_PKG_SIZE;

	/* Sending the fw image */
	LOG_INFO("Sending the image (%zu bytes)\n", size);
	LOG_DBG("Sending a chunk (%zu bytes cksum 0x%08x)\n",
		MIN(MAX_CHUNK_SIZE, size), *cksum);
	prepare_hstc(hstc_current, data, MIN(MAX_CHUNK_SIZE, size));
	size -= hstc_current->len - CKSUM_SIZE;
	data += hstc_current->len - CKSUM_SIZE;
	do {
		rc = xfer_payload_prep_next(handle, "data chunk", hstc_current,
					    sstc, hstc_next, &data, &size);
		if (rc)
			goto exit;
		/* swap hstcs */
		hstc = hstc_current;
		hstc_current = hstc_next;
		hstc_next = hstc;
	} while (size);
	/* Sends the last chunk now */
	rc = xfer_payload(handle, "data chunk", hstc_current, sstc);

exit:
#ifdef WRITE_STATS
	dump_stats();
#endif

	// tries to get the flashing status anyway...
	rc = run_fwupdater_get_status(handle, hstc, sstc, &status);
	if (!rc) {
		if (status.status) {
			LOG_ERR("Flashing failed, fw updater status %#x (errors: sub %#x, cksum %u, rram %u, crypto %d)\n",
				status.status, status.suberror,
				status.cksum_errors, status.rram_errors,
				status.crypto_errors);
			rc = status.status;
		} else {
			if (gstats_ss_irq_timeouts + gstats_spi_errors +
			    status.cksum_errors + status.rram_errors +
			    status.crypto_errors) {
				LOG_WARN(
					"Flashing succeeded with errors (host %u, IRQ timeout %u, QM %u, cksum %u, rram %u, crypto %d)\n",
					gstats_spi_errors,
					gstats_ss_irq_timeouts,
					status.spi_errors, status.cksum_errors,
					status.rram_errors,
					status.crypto_errors);
			} else {
				LOG_INFO(
					"Flashing succeeded without any errors\n");
			}

			if (!handle->skip_check_fw_boot) {
				handle->dev_ops.reset(handle->reset_handle);
				qmrom_msleep(WAIT_REBOOT_DELAY_MS);
				rc = qmrom_check_fw_boot_state(
					handle,
					WAIT_SS_IRQ_AFTER_RESET_TIMEOUT_MS);
			}
		}
	} else {
		LOG_ERR("run_fwupdater_get_status returned %d (host %u, IRQ timeout %u)\n",
			rc, gstats_spi_errors, gstats_ss_irq_timeouts);
	}
exit_nomem:
	if (rx)
		qmrom_free(rx);
	if (tx)
		qmrom_free(tx);
	return rc;
}
