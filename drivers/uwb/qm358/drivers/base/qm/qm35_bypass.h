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
#ifndef __QM35_BYPASS_H
#define __QM35_BYPASS_H

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/kernel.h>

#include "qm35_transport.h"

/**
 * enum qm35_bypass_events - Bypass event types.
 * @QM35_BYPASS_IRQ: A QM35 interruption occurred.
 * @QM35_BYPASS_NOTIFICATION: Notification event from the low level device.
 * @QM35_BYPASS_RESPONSE: Response event from the low level device.
 * @QM35_BYPASS_MAX: Count of event types.
 */
enum qm35_bypass_events {
	QM35_BYPASS_IRQ,
	QM35_BYPASS_NOTIFICATION,
	QM35_BYPASS_RESPONSE,
	QM35_BYPASS_MAX
};

/**
 * enum qm35_bypass_actions - Bypass control actions.
 * @QM35_BYPASS_ACTION_RESET: Force reset of the low level device.
 * @QM35_BYPASS_ACTION_FWUPD: Force firmware update.
 * @QM35_BYPASS_ACTION_POWER: Force power on or off.
 * @QM35_BYPASS_ACTION_MSG_TYPE: Change/read expected message type.
 * @QM35_BYPASS_ACTION_MAX: Count of action types.
 */
enum qm35_bypass_actions {
	QM35_BYPASS_ACTION_RESET,
	QM35_BYPASS_ACTION_FWUPD,
	QM35_BYPASS_ACTION_POWER,
	QM35_BYPASS_ACTION_MSG_TYPE,
	QM35_BYPASS_ACTION_MAX
};

/**
 * typedef qm35_bypass_listener_cb - Callback function type for qm35_bypass_open()
 * @data: private data of the callback, set on qm35_bypass_open() call
 * @event: the event to forward to the callback owner
 *
 * Return: Zero on success, else a negative error code.
 */
typedef int (*qm35_bypass_listener_cb)(void *data,
				       enum qm35_bypass_events event);

/**
 * struct qm35_bypass_channel - Bypass channel structure.
 * @bypass: Back-pointer to associated struct qm35_bypass.
 * @list: List of opened bypass channels.
 * @listener: Callback function called on bypass event.
 * @listener_data: Argument passed to @listener.
 * @expected_type: Type of received packet to forward to bypass channel.
 * @packets: List of received packets.
 * @lock: Lock to protect packets list.
 */
struct qm35_bypass_channel {
	struct qm35_bypass *bypass;
	struct list_head list;
	qm35_bypass_listener_cb listener;
	void *listener_data;
	enum qm35_transport_msg_type expected_type;
	struct list_head packets;
	spinlock_t lock;
};

/**
 * typedef qm35_bypass_handle - Bypass channel handle.
 *
 * A pointer to &struct qm35_bypass_channel.
 */
typedef struct qm35_bypass_channel *qm35_bypass_handle;

/**
 * struct qm35_bypass - Bypass data structure.
 * @lock: Lock to protect open and close operations.
 * @channels: List of opened channels.
 * @opened: Number of opened channels.
 */
struct qm35_bypass {
	spinlock_t lock;
	struct list_head channels;
	atomic_t opened;
};

/**
 * qm35_bypass_bound() - Check bound status of the bypass channel.
 * @hnd: The bypass channel handle to check.
 * @out: Optional pointer to buffer to write current expected packet type.
 *
 * Return: True if the bypass channel is bound to a transport packet type,
 *  else false.
 */
static inline int qm35_bypass_bound(qm35_bypass_handle hnd,
				    enum qm35_transport_msg_type *out)
{
	if (hnd->expected_type != QM35_TRANSPORT_MSG_MAX) {
		if (out)
			*out = hnd->expected_type;
		return true;
	}
	return false;
}

qm35_bypass_handle qm35_bypass_open(struct qm35 *qm35,
				    qm35_bypass_listener_cb cb,
				    void *priv_data);

int qm35_bypass_queue_check(qm35_bypass_handle hnd);
int qm35_bypass_close(qm35_bypass_handle hnd);
int qm35_bypass_send(qm35_bypass_handle hnd, void *buffer, size_t len);
int qm35_bypass_recv(qm35_bypass_handle hnd, void __user *buffer, size_t len,
		     enum qm35_transport_msg_type *type, int *flags);
int qm35_bypass_control(qm35_bypass_handle hnd, enum qm35_bypass_actions action,
			long *param);

#ifdef QM35_BYPASS_TESTS
#include "mocks/ku_base.h"
#define KU_NO_KZALLOC_MOCK
#include "mocks/ku_alloc_free.h"
#define KU_NO_ALLOC_SKB_MOCK
#include "mocks/ku_alloc_free_skb.h"
#define KU_NO_COPY_FROM_USER_MOCK
#include "mocks/ku_copy_user.h"
#include "mocks/ku_module_get_put.h"
#define KU_NO_SEND_MOCK
#include "mocks/ku_transport.h"

/* Ensure modified functions aren't exported! */
#undef EXPORT_SYMBOL
#define EXPORT_SYMBOL(x)

#endif /* QM35_BYPASS_TESTS */

#endif /* __QM35_BYPASS_H */
