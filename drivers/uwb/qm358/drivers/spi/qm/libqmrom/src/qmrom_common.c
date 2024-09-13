// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2022 Qorvo US, Inc.
 *
 */

#include <qmrom.h>
#include <qmrom_log.h>
#include <qmrom_spi.h>
#include <qmrom_utils.h>
#include <spi_rom_protocol.h>

#define SPI_PROBE_CLOCKRATE 1000000
#define RSP_OFFSET 0
#define READY_FOR_CS_LOW_RSP 0
#define C0_INITIAL_MESSAGE_CHIP_REV_OFFSET 10
#define C0_CHIP_REV 0x0430

int qm357xx_rom_b0_probe_device(struct qmrom_handle *handle);
int qm357xx_rom_c0_probe_device(struct qmrom_handle *handle);
int qm358xx_rom_probe_device(struct qmrom_handle *handle);

static void qmrom_free_stcs(struct qmrom_handle *h)
{
	if (h->hstc)
		qmrom_free(h->hstc);
	if (h->sstc)
		qmrom_free(h->sstc);
}

#ifdef CHECK_STCS
void check_stcs(const char *func, int line, struct qmrom_handle *h)
{
	uint32_t *buff = (uint32_t *)h->hstc;
	if (buff[MAX_STC_FRAME_LEN / sizeof(uint32_t)] != 0xfeeddeef) {
		LOG_ERR("%s:%d - hstc %pK corrupted\n", func, line,
			(void *)h->hstc);
	} else {
		LOG_ERR("%s:%d - hstc %pK safe\n", func, line, (void *)h->hstc);
	}
	buff = (uint32_t *)h->sstc;
	if (buff[MAX_STC_FRAME_LEN / sizeof(uint32_t)] != 0xfeeddeef) {
		LOG_ERR("%s:%d - sstc %pK corrupted\n", func, line,
			(void *)h->sstc);
	} else {
		LOG_ERR("%s:%d - sstc %pK safe\n", func, line, (void *)h->sstc);
	}
}
#endif

static int qmrom_allocate_stcs(struct qmrom_handle *h)
{
	int rc = 0;
	uint8_t *tx_buf = NULL, *rx_buf = NULL;

	qmrom_alloc(tx_buf, MAX_STC_FRAME_LEN + sizeof(uint32_t));
	if (tx_buf == NULL) {
		rc = -ENOMEM;
		goto out;
	}

	qmrom_alloc(rx_buf, MAX_STC_FRAME_LEN + sizeof(uint32_t));
	if (rx_buf == NULL) {
		qmrom_free(tx_buf);
		rc = -ENOMEM;
		goto out;
	}

#ifdef CHECK_STCS
	((uint32_t *)tx_buf)[MAX_STC_FRAME_LEN / sizeof(uint32_t)] = 0xfeeddeef;
	((uint32_t *)rx_buf)[MAX_STC_FRAME_LEN / sizeof(uint32_t)] = 0xfeeddeef;
#endif
	h->hstc = (struct stc *)tx_buf;
	h->sstc = (struct stc *)rx_buf;
	return rc;
out:
	qmrom_free_stcs(h);
	return rc;
}

int qmrom_spi_transfer_delay(void *handle, char *rbuf, const char *wbuf,
			     size_t size)
{
	int rc;

	rc = qmrom_spi_transfer(handle, rbuf, wbuf, size);
	qmrom_usleep(SPI_INTERCMD_DELAY_US);
	return rc;
}

int qmrom_poll(struct qmrom_handle *handle)
{
	handle->hstc->all = 0;

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

int qmrom_pre_read(struct qmrom_handle *handle)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.pre_read = 1;
	handle->hstc->ul = 1;

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

int qmrom_read(struct qmrom_handle *handle)
{
	size_t rd_size = handle->sstc->len;
	if (rd_size > MAX_STC_FRAME_LEN)
		return SPI_ERR_INVALID_STC_LEN;
	LOG_DBG("%s: reading %zu bytes...\n", __func__, rd_size);
	memset(handle->hstc, 0, sizeof(struct stc) + rd_size);
	handle->hstc->host_flags.read = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = handle->sstc->len;

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + rd_size);
}

int qmrom_write_cmd(struct qmrom_handle *handle, uint8_t cmd)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = 1;
	handle->hstc->payload[0] = cmd;

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

int qmrom_write_cmd32(struct qmrom_handle *handle, uint32_t cmd)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = sizeof(cmd);
	memcpy(handle->hstc->payload, &cmd, sizeof(cmd));

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

int qmrom_write_size_cmd(struct qmrom_handle *handle, uint8_t cmd,
			 uint16_t data_size, const char *data)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = data_size + 1;
	handle->hstc->payload[0] = cmd;
	memcpy(&handle->hstc->payload[1], data, data_size);

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

int qmrom_write_size_cmd32(struct qmrom_handle *handle, uint32_t cmd,
			   uint16_t data_size, const char *data)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = data_size + sizeof(cmd);
	memcpy(handle->hstc->payload, &cmd, sizeof(cmd));
	memcpy(&handle->hstc->payload[sizeof(cmd)], data, data_size);

	return qmrom_spi_transfer_delay(handle->spi_handle,
					(char *)handle->sstc,
					(const char *)handle->hstc,
					sizeof(struct stc) + handle->hstc->len);
}

/* Poll SoC until a flag is set or retries expires. */
void qmrom_poll_soc(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;

	qmrom_poll(handle);
	while (--retries && handle->sstc->raw_flags == 0) {
		qmrom_msleep(DEFAULT_SPI_LATENCY_MS);
		qmrom_poll(handle);
	}
}

/* Return true if only ready flag is set. */
inline bool qmrom_is_soc_ready(struct qmrom_handle *handle)
{
	return (handle->sstc->raw_flags & 0xf0) == 0x20;
}

/* Poll SoC and consume messages until only ready flag is set. */
int qmrom_wait_ready(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;
	int rc;

	qmrom_poll_soc(handle);
	while (--retries && !qmrom_is_soc_ready(handle)) {
		if (handle->sstc->soc_flags.out_waiting) {
			rc = qmrom_pre_read(handle);
			if (!rc && handle->sstc->soc_flags.out_waiting)
				qmrom_read(handle);
		}
		/* Do another poll after a read, as out_active is still set. */
		qmrom_poll_soc(handle);
	}

	if (!qmrom_is_soc_ready(handle))
		LOG_ERR("%s failed after %d retries\n", __func__,
			handle->comms_retries);

	return qmrom_is_soc_ready(handle) ? 0 : SPI_ERR_WAIT_READY_TIMEOUT;
}

/* Wait for IRQ line then read response. */
int qmrom_poll_cmd_resp(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;
	int rc;

	do {
		rc = qmrom_spi_wait_for_irq_line(handle->ss_irq_handle,
						 SPI_READY_TIMEOUT_MS);
		if (rc)
			goto loop_end;

		rc = qmrom_pre_read(handle);
		if (rc || !handle->sstc->soc_flags.out_waiting)
			goto loop_end;

		rc = qmrom_read(handle);
		if (rc || !handle->sstc->soc_flags.out_active)
			goto loop_end;
		return rc;
	loop_end:
		qmrom_msleep(DEFAULT_SPI_LATENCY_MS);
	} while (--retries);

	LOG_ERR("%s failed after %d retries\n", __func__,
		handle->comms_retries);

	return -1;
}

/*
 * Unfortunately, A0, B0 and C0 have different
 * APIs to get the chip version...
 *
 */
int qmrom_probe_device(struct qmrom_handle *handle,
		       enum device_generation_e dev_gen_hint)
{
	int retries = handle->comms_retries;
	int rc;

	if (handle->spi_speed == 0)
		qmrom_spi_set_freq(SPI_PROBE_CLOCKRATE);
	else
		qmrom_spi_set_freq(handle->spi_speed);

	do {
		rc = qmrom_reboot_bootloader(handle);
		if (rc) {
			LOG_ERR("%s: cannot reset the device...\n", __func__);
			return rc;
		}

		/* Wait for initial response message sent upon entering command mode. */
		rc = qmrom_poll_cmd_resp(handle);
		if (rc) {
			LOG_INFO(
				"%s: did not get initial response from device\n",
				__func__);
			return rc;
		}

		rc = -1;
		/* The initial response message is:
		 * - 8 bytes on QM358xx
		 * - 12 bytes on QM357xx C0
		 * - 1 byte on QM357xx B0
		 */
		switch (handle->sstc->len) {
		case 8:
			if (dev_gen_hint != DEVICE_GEN_QM358XX &&
			    dev_gen_hint != DEVICE_GEN_QPF51XX &&
			    dev_gen_hint != DEVICE_GEN_UNKNOWN)
				break;
			/* Initial response message checked in function. */
			rc = qm358xx_rom_probe_device(handle);
			break;
		case 12:
			if (dev_gen_hint != DEVICE_GEN_QM357XX &&
			    dev_gen_hint != DEVICE_GEN_UNKNOWN)
				break;
			/* Check response and device_id. */
			if (handle->sstc->payload[RSP_OFFSET] ==
				    READY_FOR_CS_LOW_RSP &&
			    be16toh(SSTC2UINT16(
				    handle,
				    C0_INITIAL_MESSAGE_CHIP_REV_OFFSET)) ==
				    C0_CHIP_REV)
				rc = qm357xx_rom_c0_probe_device(handle);
			else
				LOG_ERR("%s: bad response (%d, 0x%04x)\n",
					__func__,
					handle->sstc->payload[RSP_OFFSET],
					be16toh(SSTC2UINT16(
						handle,
						C0_INITIAL_MESSAGE_CHIP_REV_OFFSET)));
			break;
		case 1:
			if (dev_gen_hint != DEVICE_GEN_QM357XX &&
			    dev_gen_hint != DEVICE_GEN_UNKNOWN)
				break;
			/* Check response. */
			if (handle->sstc->payload[RSP_OFFSET] ==
			    READY_FOR_CS_LOW_RSP)
				rc = qm357xx_rom_b0_probe_device(handle);
			else
				LOG_ERR("%s: bad response (%d)\n", __func__,
					handle->sstc->payload[RSP_OFFSET]);
			break;
		}
	} while (--retries && rc);

	return rc;
}

struct qmrom_handle *qmrom_init(void *spi_handle, void *reset_handle,
				void *ss_irq_handle, int spi_speed,
				int comms_retries, reset_device_fn reset,
				enum device_generation_e dev_gen_hint)
{
	struct qmrom_handle *handle;
	int rc;

	qmrom_alloc(handle, sizeof(struct qmrom_handle));
	if (!handle) {
		LOG_ERR("%s: Couldn't allocate %zu bytes...\n", __func__,
			sizeof(struct qmrom_handle));
		return NULL;
	}
	rc = qmrom_allocate_stcs(handle);
	if (rc) {
		LOG_ERR("%s: Couldn't allocate stcs...\n", __func__);
		qmrom_free(handle);
		return NULL;
	}

	handle->spi_handle = spi_handle;
	handle->reset_handle = reset_handle;
	handle->ss_irq_handle = ss_irq_handle;
	handle->comms_retries = comms_retries;
	handle->chip_rev = CHIP_REVISION_UNKNOWN;
	handle->device_version = -1;
	handle->spi_speed = spi_speed;
	handle->skip_check_fw_boot = false;

	handle->dev_ops.reset = reset;

	rc = qmrom_probe_device(handle, dev_gen_hint);
	if (rc) {
		LOG_ERR("%s: qmrom_probe_device returned %d!\n", __func__, rc);
		qmrom_free_stcs(handle);
		qmrom_free(handle);
		return NULL;
	}

	check_stcs(__func__, __LINE__, handle);
	return handle;
}

void qmrom_deinit(struct qmrom_handle *handle)
{
	LOG_DBG("Deinitializing %pK\n", (void *)handle);
	qmrom_free_stcs(handle);
	qmrom_free(handle);
}

int qmrom_reboot_bootloader(struct qmrom_handle *handle)
{
	int rc;

	qmrom_msleep(1000);
	rc = qmrom_spi_set_cs_level(handle->spi_handle, 1);
	LOG_ERR("%s: qmrom_reboot_bootloader is called\n", __func__);
	if (rc) {
		LOG_ERR("%s: spi_set_cs_level(0) failed with %d\n", __func__,
			rc);
		return rc;
	}
	qmrom_msleep(SPI_RST_LOW_DELAY_MS);

	handle->dev_ops.reset(handle->reset_handle);

	qmrom_msleep(SPI_RST_LOW_DELAY_MS);

	rc = qmrom_spi_set_cs_level(handle->spi_handle, 0);
	if (rc) {
		LOG_ERR("%s: spi_set_cs_level(1) failed with %d\n", __func__,
			rc);
		return rc;
	}

	qmrom_msleep(SPI_RST_LOW_DELAY_MS);

	return 0;
}
