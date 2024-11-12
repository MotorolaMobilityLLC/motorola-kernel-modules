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
#ifndef __QM35_CORE_H
#define __QM35_CORE_H

#include <linux/types.h>

/* Forward declarations to avoid lot of includes here. */
struct qm35;
struct qm35_transport;
struct device;
struct dentry;

/**
 * enum qm35_state - State of QM35 device.
 * @QM35_STATE_UNKNOWN:
 * 	QM35 state is unknow, when starting the probe.
 * @QM35_STATE_READY:
 * 	QM35 is ready.
 * @QM35_STATE_ACTIVE:
 * 	QM35 is performing a communication or ranging.
 * @QM35_STATE_ERROR:
 * 	QM35 needs to be reset.
 */
enum qm35_state {
	QM35_STATE_UNKNOWN,
	QM35_STATE_READY,
	QM35_STATE_ACTIVE,
	QM35_STATE_ERROR,
};

struct qm35 *qm35_alloc_device(struct device *dev, size_t priv_size,
			       const struct qm35_transport *transport);
void qm35_free_device(struct qm35 *qm35);

int qm35_register_device(struct qm35 *qm35);
int qm35_unregister_device(struct qm35 *qm35);
int qm35_get_dev_id(struct qm35 *qm35);
struct device *qm35_get_device(struct qm35 *qm35);
struct dentry *qm35_get_debug_root(struct qm35 *qm35);

void qm35_state_set(struct qm35 *qm35, enum qm35_state device_state);
int qm35_state_wait(struct qm35 *qm35, enum qm35_state device_state);

#ifdef QM35_CORE_TESTS
#define KU_NO_KMALLOC_MOCK
#include "mocks/ku_alloc_free.h"
#define KU_NO_POWER_MOCK
#define KU_NO_FREE_PACKET_MOCK
#define KU_NO_REGISTER_MOCK
#define KU_NO_UNREGISTER_MOCK
#define KU_NO_SEND_MOCK
#include "mocks/ku_transport.h"
#include "mocks/ku_wait_event.h"

/* The following wrapper MUST be set to ensure the core API don't use
 * functions that have their own tests suite. This will result in
 * conflicting type for test->priv structure. */

/* These ones will be declared in qm35_ids.h, included AFTER qm35_core.h
   in qm35_core.c */
#define qm35_new_id ku_qm35_new_id
#define qm35_remove_id ku_qm35_remove_id

/* This one will be declared in qm35_notifier.h, included AFTER qm35_core.h
   in qm35_core.c */
#define qm35_notifier_notify ku_qm35_notifier_notify

/* Ensure modified functions aren't exported! */
#undef EXPORT_SYMBOL
#define EXPORT_SYMBOL(x)

#endif /* QM35_CORE_TESTS */

#endif /* __QM35_CORE_H */
