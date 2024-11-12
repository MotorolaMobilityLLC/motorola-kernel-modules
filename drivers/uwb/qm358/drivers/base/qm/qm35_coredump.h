/*
 * This file is part of the UWB stack for Linux.
 *
 * Copyright (c) 2020-2023 Qorvo US, Inc.
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
#ifndef __QM35_COREDUMP_H
#define __QM35_COREDUMP_H

#include <linux/ratelimit.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "qm35_transport.h"

#define QM35_COREDUMP_NAME "coredump"

/**
 * struct qm35_coredump_data - QM35 Coredump data structure.
 * @list: List of coredumps.
 * @bin_attr: Binary attribute in sysfs.
 * @rfw: Async remove file work struct.
 * @size: Size of the received coredump.
 * @remain: Remaining space in the receive buffer.
 * @offset: Current position in the receive buffer.
 * @csum: Checksum of coredump given by the firmware.
 * @status: Reception status of the coredump, sent back to the firmware.
 * @name: Coredump name in sysfs.
 * @buffer: Buffer containing the received coredump. Right after this structure.
 *
 * This structure hold one COREDUMP received from the QM35 firmware. It
 * allows driver to not lost one dump if the QM35 firmware crash at high rate
 * and sends a lot of coredump after each reboot.
 */
struct qm35_coredump_data {
	struct list_head list;
	struct bin_attribute bin_attr;
	struct work_struct rfw;
	size_t size;
	size_t remain;
	size_t offset;
	uint16_t csum;
	uint8_t status;
	char name[16];
	char buffer[];
};

/**
 * struct qm35_coredump - QM35 Coredump extension structure.
 * @dev_list: List of coredump extension instances.
 * @qm35: Backpointer to QM35 instance.
 * @dev: Backpointer to underlying device instance.
 * @bin_attr: Write-only coredump attribute for commands.
 * @dwork: Delayed work to check timeout receiving coredump.
 * @dump_rls: Rate limit state for received coredumps.
 * @dump_list: List of all received coredumps.
 * @dump_lock: Lock to protect @dump_list.
 * @dump_num: Last coredump index number.
 * @dump_cnt: Currently stored coredumps counter.
 */
struct qm35_coredump {
	struct list_head dev_list;
	struct qm35 *qm35;
	struct device *dev;
	struct delayed_work dwork;
	struct bin_attribute bin_attr;
	struct ratelimit_state dump_rls;
	struct list_head dump_list;
	spinlock_t dump_lock;
	atomic_t dump_num;
	unsigned dump_cnt;
};

/* No exported functions. */

#ifdef QM35_COREDUMP_TESTS

#include "mocks/ku_base.h"
#include "mocks/ku_alloc_free.h"
#define KU_NO_ALLOC_SKB_MOCK
#include "mocks/ku_alloc_free_skb.h"
#include "mocks/ku_get_device.h"
#include "mocks/ku_notifier.h"
#define KU_NO_START_MOCK
#define KU_NO_STOP_MOCK
#define KU_NO_RESET_MOCK
#define KU_NO_POWER_MOCK
#define KU_NO_FW_UPD_MOCK
#define KU_NO_PROBE_MOCK
#include "mocks/ku_transport.h"
#include "mocks/ku_schedule_work.h"

int ku_sysfs_create_bin_file(struct kobject *kobj,
			     struct bin_attribute *bin_attr);
#define sysfs_create_bin_file ku_sysfs_create_bin_file

int ku_work_busy(struct work_struct *work);
#define work_busy ku_work_busy

#endif /* QM35_COREDUMP_TESTS */
#endif /* __QM35_COREDUMP_H */
