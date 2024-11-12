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
#ifndef __QM35_UCI_DEV_H
#define __QM35_UCI_DEV_H

#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/fs.h>

#include "qm35_bypass.h"

#define QM35_UCI_DEV_DEVICE_NAME "uci"
#define QM35_UCI_DEV_DEVICE_NAME_SIZE 8

#define QM35_UCI_DEV_MAX_PACKET_SIZE 1024

struct qm35_uci_dev;

/**
 * struct qm35_uci_dev_channel - UCI char device channel structure.
 * @list: List element to put this channel in channels list.
 * @uci_dev: Back-pointer to UCI char device structure.
 * @bypass: Underlying bypass channel handle.
 * @event: Last received bypass event.
 * @wait_queue: File read wait queue.
 * @data_available: Data are available.
 * @write_buffer: The write buffer.
 */
struct qm35_uci_dev_channel {
	struct list_head list;
	struct qm35_uci_dev *uci_dev;
	qm35_bypass_handle bypass;
	enum qm35_bypass_events event;
	wait_queue_head_t wait_queue;
	bool data_available;
	char write_buffer[QM35_UCI_DEV_MAX_PACKET_SIZE];
};

typedef struct qm35_uci_dev_channel *qm35_uci_dev_handle;

/**
 * struct qm35_uci_dev - UCI char device structure.
 * @miscdev: The miscdevice.
 * @name: The name of the miscdevice.
 * @qm35: Pointer the associated QM35 instance to this miscdevice.
 * @state: State of this UCI char device.
 * @lock: Mutex protecting the channels list update.
 * @channels: The opened channels on the associated QM35 device.
 * @dev_list: List element of the chained qm35_uci_dev devices.
 */
struct qm35_uci_dev {
	struct miscdevice miscdev;
	char name[QM35_UCI_DEV_DEVICE_NAME_SIZE];
	struct qm35 *qm35;
	unsigned int state;
	struct mutex lock;
	struct list_head channels;
	struct list_head dev_list;
};

#ifdef QM35_UCI_DEV_TESTS

#include "mocks/ku_base.h"
#include "mocks/ku_alloc_free.h"
#include "mocks/ku_copy_user.h"
#include "mocks/ku_get_dev_id.h"
#include "mocks/ku_get_device.h"
#include "mocks/ku_notifier.h"
#include "mocks/ku_wait_event.h"

/* Declare our wrapper functions */
qm35_bypass_handle ku_qm35_bypass_open(struct qm35 *qm35,
				       qm35_bypass_listener_cb cb,
				       void *priv_data);
int ku_qm35_bypass_close(qm35_bypass_handle hnd);
int ku_qm35_bypass_queue_check(qm35_bypass_handle hnd);
int ku_qm35_bypass_send(qm35_bypass_handle hnd, void *buffer, size_t len);
int ku_qm35_bypass_recv(qm35_bypass_handle hnd, void *buffer, size_t len,
			enum qm35_transport_msg_type *type, int *flags);
int ku_qm35_bypass_control(qm35_bypass_handle hnd,
			   enum qm35_bypass_actions action, long *param);

int ku_misc_register(struct miscdevice *misc);
void ku_misc_deregister(struct miscdevice *misc);

void ku_poll_wait(struct file *filp, wait_queue_head_t *wait_address,
		  poll_table *p);

/* Redefine some functions to use our test wrappers */
#define qm35_bypass_open ku_qm35_bypass_open
#define qm35_bypass_close ku_qm35_bypass_close
#define qm35_bypass_queue_check ku_qm35_bypass_queue_check
#define qm35_bypass_send ku_qm35_bypass_send
#define qm35_bypass_recv ku_qm35_bypass_recv
#define qm35_bypass_control ku_qm35_bypass_control

#define misc_register ku_misc_register
#define misc_deregister ku_misc_deregister

#define poll_wait ku_poll_wait

/* Ensure modified functions aren't exported! */
#undef EXPORT_SYMBOL
#define EXPORT_SYMBOL(x)

#endif /* QM35_UCI_DEV_TESTS */

#endif /* __QM35_UCI_DEV_H */
