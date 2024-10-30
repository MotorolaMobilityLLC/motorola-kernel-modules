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
#ifndef QM35_TRANSPORT_H
#define QM35_TRANSPORT_H

#include <linux/types.h>

/* QM35_TRANSPORT_MSG_LOG packet exceed UCI_MAX_PACKET_SIZE */
#define QM35_MAX_PACKET_SIZE 1024

/* Forward declaration to avoid need to include qm35.h here. */
struct qm35;
struct qm35_fw_version;

struct sk_buff;

/**
 * enum qm35_transport_msg_type - Supported message types by transport ops
 * @QM35_TRANSPORT_MSG_RESERVED_HSSPI: Special type for HSSPI transport
 * @QM35_TRANSPORT_MSG_BOOTLOADER: Bootloader message
 * @QM35_TRANSPORT_MSG_UCI: UCI message
 * @QM35_TRANSPORT_MSG_COREDUMP: Coredump message
 * @QM35_TRANSPORT_MSG_LOG: Log message
 * @QM35_TRANSPORT_MSG_QTRACE: Qtrace message
 * @QM35_TRANSPORT_MSG_AWAKE: Awake message
 * @QM35_TRANSPORT_MSG_MAX: Number of message types
 *
 * Values of this enum match the ul_value of HSSPI protocol used by QM35
 * when connected through SPI.
 */
enum qm35_transport_msg_type {
	QM35_TRANSPORT_MSG_RESERVED_HSSPI,
	QM35_TRANSPORT_MSG_BOOTLOADER,
	QM35_TRANSPORT_MSG_UCI,
	QM35_TRANSPORT_MSG_COREDUMP,
	QM35_TRANSPORT_MSG_LOG,
	QM35_TRANSPORT_MSG_QTRACE,
	QM35_TRANSPORT_MSG_AWAKE,
	QM35_TRANSPORT_MSG_MAX
};

/**
 * enum qm35_transport_priority - Supported handler priorities.
 * @QM35_TRANSPORT_PRIO_HIGH: High priority handler.
 * @QM35_TRANSPORT_PRIO_NORMAL: Normal priority handler.
 * @QM35_TRANSPORT_PRIO_COUNT: Number of priorities.
 */
enum qm35_transport_priority {
	QM35_TRANSPORT_PRIO_HIGH,
	QM35_TRANSPORT_PRIO_NORMAL,
	QM35_TRANSPORT_PRIO_COUNT
};

/**
 * enum qm35_transport_events - Type of event received from lower level.
 * @QM35_TRANSPORT_EVENT_IRQ: A QM35 interruption occured.
 * @QM35_TRANSPORT_EVENT_NOTIFICATION: A UCI notification is available.
 * @QM35_TRANSPORT_EVENT_RESPONSE: A UCI response is available.
 * @QM35_TRANSPORT_EVENT_MAX: Number of event types.
 *
 * Note: no event is defined yet for the other message types.
 */
enum qm35_transport_events {
	QM35_TRANSPORT_EVENT_IRQ,
	QM35_TRANSPORT_EVENT_NOTIFICATION,
	QM35_TRANSPORT_EVENT_RESPONSE,
	QM35_TRANSPORT_EVENT_MAX,
};

/**
 * struct qm35_transport_ops - QM35 transport operation.
 * @start: Activate the device (optional).
 * @stop: Deactivate the device (optional).
 * @reset: Force a device reset.
 * @power: Force a power down/up.
 * @fw_update: Update the QM35 firmware.
 * @send: Send a frame of a specific type to device.
 * @recv: Recv a frame from device, with type and flags.
 * @probe: Probe the QM35 device.
 *
 * The result of the recv function is the actual size of the buffer (positive)
 * or an error code (negative).
 */
struct qm35_transport_ops {
	int (*start)(struct qm35 *qm35);
	void (*stop)(struct qm35 *qm35);
	int (*reset)(struct qm35 *qm35, bool bootrom);
	int (*power)(struct qm35 *qm35, int on);
	int (*fw_update)(struct qm35 *qm35, struct qm35_fw_version *current_ver,
			 u16 device_id, const char *fw_name);
	int (*send)(struct qm35 *qm35, enum qm35_transport_msg_type type,
		    const void *data, size_t size);
	ssize_t (*recv)(struct qm35 *qm35, void *data, size_t size,
			enum qm35_transport_msg_type *type, int *flags);
	int (*probe)(struct qm35 *qm35, char *infobuf, size_t len);
};

/**
 * enum qm35_transport_flags - QM35 transport flags.
 * @QM35_TRANSPORT_NO_PROBING: When set, no probing is allowed during register.
 * @QM35_TRANSPORT_ASYNC_PROBING: When set, probing is performed asynchronously.
 */
enum qm35_transport_flags {
	QM35_TRANSPORT_NO_PROBING = 0b00000001,
	QM35_TRANSPORT_ASYNC_PROBING = 0b00000010,
};

/**
 * struct qm35_transport - QM35 transport information.
 * @name: Transport name used to generate device name.
 * @ops: Transport operations to use.
 * @flags: Transport specific flags.
 */
struct qm35_transport {
	const char *name;
	const struct qm35_transport_ops *ops;
	enum qm35_transport_flags flags;
};

/**
 * typedef qm35_transport_recv_cb - Callback type to for RX events.
 * @data: Private data of the callback handler.
 * @skb: The received packet to process.
 *
 * The provided @skb uses the @cb array field to provide:
 * * The transport packet type (in `skb->cb[0]`)
 * * The low-level transport reception flags (in `skb->cb[1]`)
 * * The low-level transport event value (in `skb->cb[2]`)
 */
typedef void (*qm35_transport_recv_cb)(void *data, struct sk_buff *skb);

/**
 * struct qm35_transport_recv_handler - QM35 transport receive handler info.
 * @data: Private data passed to callback.
 * @cb: Callback function.
 */
struct qm35_transport_recv_handler {
	void *data;
	qm35_transport_recv_cb cb;
};

/* Private transport API */
int qm35_transport_start(struct qm35 *qm35);
void qm35_transport_stop(struct qm35 *qm35);
int qm35_transport_reset(struct qm35 *qm35, bool bootrom);
int qm35_transport_power(struct qm35 *qm35, int on);

int qm35_transport_send(struct qm35 *qm35, enum qm35_transport_msg_type type,
			const void *data, size_t length);

int qm35_transport_fw_update(struct qm35 *qm35,
			     struct qm35_fw_version *current_ver, u16 device_id,
			     const char *fw_name);

int qm35_transport_probe(struct qm35 *qm35, char *infobuf, size_t len);

/* Public exported API for transport modules */
int qm35_transport_event(struct qm35 *qm35, enum qm35_transport_events event);

int qm35_transport_register(struct qm35 *qm35,
			    enum qm35_transport_msg_type type,
			    enum qm35_transport_priority prio,
			    qm35_transport_recv_cb callback, void *data);
int qm35_transport_unregister(struct qm35 *qm35,
			      enum qm35_transport_msg_type type,
			      enum qm35_transport_priority prio,
			      qm35_transport_recv_cb callback);

#define qm35_transport_send_direct(q, t, d, l) \
	((q)->transport_ops->send)((q), (t), (d), (l))

#ifdef QM35_TRANSPORT_TESTS

#include "mocks/ku_alloc_free_skb.h"

/* Ensure modified functions aren't exported! */
#undef EXPORT_SYMBOL
#define EXPORT_SYMBOL(x)

#endif /* QM35_TRANSPORT_TESTS */

#endif /* QM35_TRANSPORT_H */
