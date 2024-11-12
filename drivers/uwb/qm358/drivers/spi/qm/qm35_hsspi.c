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
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/atomic.h>
#include <linux/errno.h>

#include "qm35_hsspi.h"
#include "qm35_spi.h"
#include "qm35_spi_trc.h"

/**
 * struct qm35_hsspi_message - SPI message with two transfers.
 * @msg: SPI message.
 * @tr: SPI transfer table.
 */
struct qm35_hsspi_message {
	struct spi_message msg;
	struct spi_transfer tr[2];
};

/**
 * qm35_hsspi_delay() - Helper function to add delay between SPI accesses.
 */
static inline void qm35_hsspi_delay(void)
{
	if (qm35_hsspi_delay_us)
		usleep_range(qm35_hsspi_delay_us, qm35_hsspi_delay_us * 2);
}

/**
 * qm35_hsspi_setup() - Helper function to setup SPI message header.
 * @xfer: The struct qm35_spi_message to setup.
 * @shdr: The sent header used for the first transfer.
 * @rhdr: The received header used for the first transfer.
 * @data: True if setup message with a data transfer.
 */
static void qm35_hsspi_setup(struct qm35_hsspi_message *xfer,
			     struct qm35_hsspi_header *shdr,
			     struct qm35_hsspi_header *rhdr, bool data)
{
	/* Init transfers first because spi_message_init_with_transfer don't! */
	memset(xfer->tr, 0, sizeof(xfer->tr));
	/* Then construct SPI message */
	spi_message_init_with_transfers(&xfer->msg, xfer->tr, data ? 2 : 1);
	/* Finally, setup headers transfer (full-duplex) */
	xfer->tr[0].tx_buf = shdr;
	xfer->tr[0].rx_buf = rhdr;
	xfer->tr[0].len = shdr ? sizeof(*shdr) : 0;
}

/**
 * qm35_hsspi_setup_tx() - Helper function to setup SPI message for data TX.
 * @xfer: The struct qm35_spi_message to setup.
 * @shdr: The sent header used for the first transfer.
 * @rhdr: The received header used for the first transfer.
 * @data: The data buffer for the second transfer.
 * @length: The length of data to write.
 */
static void qm35_hsspi_setup_tx(struct qm35_hsspi_message *xfer,
				struct qm35_hsspi_header *shdr,
				struct qm35_hsspi_header *rhdr,
				const void *data, size_t length)
{
	qm35_hsspi_setup(xfer, shdr, rhdr, true);
	/* And setup data transfer */
	xfer->tr[1].tx_buf = data;
	xfer->tr[1].len = length;
}

/**
 * qm35_hsspi_setup_rx() - Helper function to setup SPI message for data RX.
 * @xfer: The struct qm35_spi_message to setup.
 * @shdr: The sent header used for the first transfer.
 * @rhdr: The received header used for the first transfer.
 * @data: The data buffer for the second transfer.
 * @length: The length of data to read.
 */
static void qm35_hsspi_setup_rx(struct qm35_hsspi_message *xfer,
				struct qm35_hsspi_header *shdr,
				struct qm35_hsspi_header *rhdr, void *data,
				size_t length)
{
	qm35_hsspi_setup(xfer, shdr, rhdr, true);
	/* And setup data transfer */
	xfer->tr[1].rx_buf = data;
	xfer->tr[1].len = length;
}

/**
 * qm35_hsspi_wakeup() - Wake-up QM35 SPI device through long SPI transaction.
 * @qmspi: QM35 SPI instance.
 * @force: Flag to force wakeup if no EXTON GPIO defined.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_hsspi_wakeup(struct qm35_spi *qmspi, bool force)
{
	int rc = !force;

	/* Check current power-state of QM35 if EXTON GPIO defined. */
	if (qmspi->exton_gpio) {
		/* Read current EXTON GPIO state. */
		rc = gpiod_get_value(qmspi->exton_gpio);
		if (rc < 0)
			rc = 0; /* If error, force wakeup! */
	}
	/* Nothing to do if EXTON GPIO active or NOT forced by caller. */
	if (rc)
		return 0;

	/* Wakeup device!
	 * Trace wake-up here because I don't want traces if
	 * nothing to do. */
	trace_qm35_hsspi_wakeup(qmspi);
	if (qmspi->wakeup_gpio) {
		gpiod_set_value(qmspi->wakeup_gpio, 1);
		usleep_range(QM35_WAKEUP_DURATION_US,
			     QM35_WAKEUP_DURATION_US + 100);
		gpiod_set_value(qmspi->wakeup_gpio, 0);
		if (!qmspi->async_wakeup) {
			/* After wake-up the FW need little time to restore it's context. */
			usleep_range(QM35_WAKEUP_DELAY_US,
				     QM35_WAKEUP_DELAY_US * 3 / 2);
		}
	} else {
		/* Wakeup using an SPI transaction */
		struct qm35_hsspi_message xfer;
		struct spi_transfer *tr = &xfer.tr[0];
		unsigned delay_gpioless;
		if (qmspi->async_wakeup) {
			delay_gpioless = QM35_WAKEUP_DURATION_US;
		} else {
			/* Use a longer wake-up SPI transaction to ensure CS is low when QM FW
			 * has started, which guaranteed QM stay alive for 10ms more. */
			delay_gpioless = QM35_WAKEUP_DELAY_US * 3 / 2;
		}
		/* Setup a no-data transfer! */
		qm35_hsspi_setup(&xfer, NULL, NULL, false);
		/* Add a delay after transfer. See spi_transfer_delay_exec() called by
		   spi_transfer_one_message(). */
#if (KERNEL_VERSION(5, 13, 0) > LINUX_VERSION_CODE)
		tr->delay_usecs = delay_gpioless;
#else
		tr->delay.unit = SPI_DELAY_UNIT_USECS;
		tr->delay.value = delay_gpioless;
#endif
		rc = spi_sync(qmspi->spi, &xfer.msg);
	}
	if (!rc && qmspi->async_wakeup) {
		qmspi->wakeup_event = false;
		rc = -EINPROGRESS;
	}
	return rc;
}

/**
 * qm35_hsspi_wait_ready() - Wait for QM35 SPI device to be ready.
 * @qmspi: QM35 SPI instance.
 *
 * Loop until QM35 SPI device is ready. Check device status by reading the
 * READY GPIO directly or the SOC header flags using an SPI read transaction.
 *
 * NOTE: QM35_HSSPI_WAIT_SPI must be defined to check RDY bit status using SPI
 * transactions. The FW must support SPI transactions at any time (at least
 * between PRD & RD) to use this.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_hsspi_wait_ready(struct qm35_spi *qmspi)
{
#ifdef QM35_HSSPI_WAIT_SPI
	struct qm35_hsspi_header hdr = { 0, 0, 0 };
	struct qm35_hsspi_message xfer;
#endif
	/* Wait up to 1ms. */
	const int wait_us = 1000;
	const int delay_us = 10;
	int count = wait_us / delay_us;
	bool ready = false;
	int ret = 0;

	if (!qmspi->ready_gpio) {
#ifdef QM35_HSSPI_WAIT_SPI
		/* Setup message for SPI check (header only message). */
		qm35_hsspi_setup(&xfer, &hdr, &hdr, false);
#else
		/* Assume always ready. Send/recv work as usual after delay. */
		qm35_hsspi_delay();
		return 0;
#endif
	}

	trace_qm35_hsspi_wait_ready(qmspi, count);
	while (count) {
#ifdef QM35_HSSPI_WAIT_SPI
		if (!qmspi->ready_gpio) {
			/* Now execute this spi message synchronously */
			ret = spi_sync(qmspi->spi, &xfer.msg);
			if (ret)
				break;
			trace_qm35_hsspi_soc_header(qmspi, &hdr);
			ready = (hdr.flags & HSSPI_SOC_RDY) != 0;
		} else {
#endif
			/* Read current GPIO state */
			ret = gpiod_get_value(qmspi->ready_gpio);
			if (ret < 0)
				break;
			ready = ret > 0;
#ifdef QM35_HSSPI_WAIT_SPI
		}
#endif
		if (ready)
			break;
		usleep_range(delay_us, delay_us + 1);
		count--;
	}
	if (ready)
		ret = 0;
	else if (!count) /* Timeout? */
		ret = -EAGAIN;

	/* Delay IRQ enable or next SPI exchange. */
	qm35_hsspi_delay();

	trace_qm35_hsspi_wait_ready_return(qmspi, ret, count);
	return ret;
}

#ifdef QM35_HSSPI_TESTS
/* Ensure functions below call a mock function instead of real one. */
int ku_qm35_hsspi_wait_ready(struct qm35_spi *qmspi);
#define qm35_hsspi_wait_ready ku_qm35_hsspi_wait_ready
#endif

/**
 * qm35_hsspi_prd() - Send a message to the QM35 SPI device.
 * @qmspi: QM35 SPI instance.
 * @xfer: The struct qm35_spi_message to setup.
 * @sent: The sent header used for the first transfer.
 * @recv: The received header used for the first transfer.
 * @check: The flag asserting if there is data.
 *
 * Retrieve and check SOC status before allowing a read.
 *
 * Return: 0 on success, else a negative error code.
 */
static int qm35_hsspi_prd(struct qm35_spi *qmspi,
			  struct qm35_hsspi_message *xfer,
			  struct qm35_hsspi_header *sent,
			  struct qm35_hsspi_header *recv, bool check)
{
	int ret;
	/* Setup the HSSPI header for PRE-READ */
	sent->flags = check ? 0 : HSSPI_HOST_PRD;
	sent->ul_value = 0;
	sent->length = 0;
	/* Setup message for PRE-READ (header only message) */
	qm35_hsspi_setup(xfer, sent, recv, false);
	trace_qm35_hsspi_host_header(qmspi, sent);
	/* Execute full-duplex header only transaction */
	ret = spi_sync(qmspi->spi, &xfer->msg);
	if (ret)
		return ret;
	trace_qm35_hsspi_soc_header(qmspi, recv);
	if (check)
		return 0;
	/* Check SOC state and available bytes */
	if (!(recv->flags & HSSPI_SOC_ODW) || !recv->length)
		/* SOC was not ready to send data! Retry! */
		return -EAGAIN;
	/* Save incoming frame length. */
	qmspi->prd_length = recv->length;
	atomic_set(&qmspi->prd_done, true);
	return 0;
}

/**
 * qm35_hsspi_send() - Send a message to the QM35 SPI device.
 * @qmspi: QM35 SPI instance.
 * @ul_value: HSSPI header ul_value field to set.
 * @data: Payload data.
 * @length: Payload data length.
 *
 * Send a message to the QM35 SPI device with the correct HSSPI header.
 * Use two SPI transaction to allow zero-copy mode of provided payload.
 *
 * Return: 0 on success, else a negative error code.
 */
int qm35_hsspi_send(struct qm35_spi *qmspi, u8 ul_value, const void *data,
		    size_t length)
{
	struct qm35_hsspi_header hdr;
	struct qm35_hsspi_message xfer;
	bool do_prd;
	int ret = 0;

	trace_qm35_hsspi_send(qmspi);

	if (IS_ERR_OR_NULL(qmspi))
		ret = -EINVAL;
	else if (!data || !length)
		ret = -EINVAL;
	else
		ret = qm35_hsspi_wait_ready(qmspi);
	if (ret)
		goto error;
	/* Setup the HSSPI header */
	hdr.ul_value = ul_value;
	hdr.length = length;
	/* Setup message for WRITE */
	qm35_hsspi_setup_tx(&xfer, &hdr, &hdr, data, length);
	trace_qm35_hsspi_data(qmspi, data, (int)length);
	/* Finalize the HSSPI header */
	do_prd = atomic_read(&qmspi->should_read);
	hdr.flags = HSSPI_HOST_WR | (HSSPI_HOST_PRD * do_prd);
	trace_qm35_hsspi_host_header(qmspi, &hdr);
	/* Now execute this spi message synchronously */
	ret = spi_sync(qmspi->spi, &xfer.msg);
	if (ret)
		goto error;
	trace_qm35_hsspi_soc_header(qmspi, &hdr);
	/* Check SOC flags */
	if (hdr.flags == 0x00 || hdr.flags == 0xff) {
		/* SOC may be in sleep mode. */
		ret = -EBUSY;
	} else if (!(hdr.flags & HSSPI_SOC_RDY)) {
		/* SOC wasn't ready to receive! */
		ret = -EAGAIN;
	}
	if (do_prd && (hdr.flags & HSSPI_SOC_ODW) && hdr.length) {
		qmspi->prd_length = hdr.length;
		atomic_set(&qmspi->prd_done, true);
	}

error:
	trace_qm35_hsspi_send_return(qmspi, ret);
	return ret;
}

/**
 * qm35_hsspi_recv() - Receive a message from the QM35 SPI device.
 * @qmspi: QM35 SPI instance.
 * @header: Buffer to store received HSSPI header.
 * @data: Buffer for payload data.
 * @size: Size of payload data buffer.
 *
 * Read a message from the QM35 and return header and payload in provided
 * pointers. The caller must check header flags, ul_value and length.
 *
 * Read must be done in two steps to retrieve and check SOC status first,
 * then to read the available length bytes.
 *
 * If data is NULL, just read the header (check only mode).
 *
 * This function also support combined read/write if another thread had
 * pushed send_params and set should_write.
 *
 * Return: Received message length on success, else a negative error code.
 */
int qm35_hsspi_recv(struct qm35_spi *qmspi, struct qm35_hsspi_header *header,
		    void *data, size_t size)
{
	struct qm35_hsspi_header hdr;
	struct qm35_hsspi_message xfer;
	bool check = !data;
	size_t length;
	int ret = 0;

	trace_qm35_hsspi_recv(qmspi, size);

	if (IS_ERR_OR_NULL(qmspi))
		ret = -EINVAL;
	else if (IS_ERR_OR_NULL(header))
		ret = -EINVAL;
	else if (data && !size)
		ret = -EINVAL;
	else if (!check)
		ret = qm35_hsspi_wait_ready(qmspi);
	if (ret)
		goto error;

	if (!atomic_read(&qmspi->prd_done)) {
		ret = qm35_hsspi_prd(qmspi, &xfer, &hdr, header, check);
		if (ret || check)
			goto error;
		/* No error nor checking, prd_length was set. */
	}
	if (unlikely(size < qmspi->prd_length)) {
		/* Provided buffer isn't enough for incoming frame */
		ret = -EMSGSIZE;
		/* Need to redo PRD. */
		atomic_set(&qmspi->prd_done, false);
		/* TODO: ensure code in this block match with FW behaviour.
		 * We may need to do partial read here instead. */
		goto error;
	}
	length = qmspi->prd_length;

	/* Wait device is ready. */
	ret = qm35_hsspi_wait_ready(qmspi);
	if (ret)
		goto error;

	/* Setup the HSSPI header for READ */
	hdr.flags = HSSPI_HOST_RD;
	hdr.ul_value = 0;
	hdr.length = 0;
	/* Setup message for READ with data (with 2 spi_transfer) */
	qm35_hsspi_setup_rx(&xfer, &hdr, header, data, length);
	/* Handle combined WRITE case if needed. */
	if (atomic_read(&qmspi->should_write)) {
		struct qm35_spi_work_params *params = &qmspi->send_params;
		/* If buffer isn't long enough to contain data to write, don't
		 * do combined write. */
		if (params->size > size)
			goto nowrite;
		/* As SPI framework support one buffer for both TX/RX, use the
		 * provided receive buffer to send. So copy data to send to
		 * buffer. */
		memcpy(data, params->data_out, params->size);
		/* Update transaction to set TX buffer and length. */
		xfer.tr[1].tx_buf = data;
		xfer.tr[1].len = max(length, params->size);
		/* Finally, adjust HSSPI HOST header sent. */
		hdr.ul_value = params->type;
		hdr.length = params->size;
		hdr.flags |= HSSPI_HOST_WR;
		/* Trace data to be sent. */
		trace_qm35_hsspi_data(qmspi, data, params->size);
	}
nowrite:
	trace_qm35_hsspi_host_header(qmspi, &hdr);
	/* Execute full-duplex read transaction */
	ret = spi_sync(qmspi->spi, &xfer.msg);
	if (ret)
		goto error;
	trace_qm35_hsspi_soc_header(qmspi, header);
	/* Check status and length. */
	if (hdr.flags & HSSPI_HOST_WR) {
		if (!(header->flags & HSSPI_SOC_RDY)) {
			/* Don't return an error, read may succeed, only combined
			   write failed if SOC not ready. */
			qmspi->work_send.ret = -EAGAIN;
			/* Need to retry, either from here, or from send work.
			   So we keep should_write as-is. */
		} else {
			/* No error */
			qmspi->work_send.ret = 0;
			/* Combined write ok, reset should_write so send work
			   does nothing. */
			atomic_set(&qmspi->should_write, false);
		}
	}
	if (header->length != length) {
		/* PRD OK but length has changed for the RD!
		   Let caller retry itself with prd_done reset. */
		ret = -EPROTO;
	} else if (!(header->flags & HSSPI_SOC_OA)) {
		/* PRD OK but bad SOC state during read!
		   Let caller retry itself without resetting prd_done so only
		   the RD is redo. */
		ret = -EAGAIN;
	} else {
		/* Trace received data. */
		trace_qm35_hsspi_data(qmspi, data, length);
	}

	/* Always redo PRD after a successful RD transaction, a protocol error
	   or if the ODW flag is set. */
	if (!ret || ret == -EPROTO || (header->flags & HSSPI_SOC_ODW))
		atomic_set(&qmspi->prd_done, false);
	/* On success, return length of received message. */
	if (!ret)
		ret = length;

error:
	trace_qm35_hsspi_recv_return(qmspi, ret);
	return ret;
}
