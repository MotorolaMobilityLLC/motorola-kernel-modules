/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
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
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/version.h>

#include "qm35_core.h"
#include "qm35_logs.h"
#include "qm35_notifier.h"
#include "qm35_transport.h"

#define LOG_PRINTK 0 /* Set to 1 if you want log on printk when they arrive. */

static LIST_HEAD(logs_list);

/* Firmware source code located here :
 * qm-firmware/lib/log_backend_hsspi/src/log_backend_hsspi_ctrl.c
 *
 * Each packet got a header with packet id, length and an optional payload.
 *
 * There is 4 Commands:
 * QM35_LOGS_DATA (Read only):
 * 	Payload contains only a string, length is given in header.
 * QM35_LOGS_SET_LEVEL (Read / Write):
 * 	Set a log sources level, payload contains source id and level (2 bytes).
 * 	Firmware will answer the same message with value currently set.
 * QM35_LOGS_GET_LEVEL (Read / Write):
 * 	Ask a log sources level, payload contains sources id (and can be longer)
 * 	Firmware will answer the logs source id and level.
 * QM35_LOGS_GET_SOURCES (Read / Write)
 * 	Ask to get sources name and id, packet header is enough
 * 	Firmware will ansswer the number of sources, and a loop containing
 * 	source id, source level, source name (null terminated)
 */

/* Sysfs tree structure used:
 *
 * /sys/bus/spi/devices/spiX.Y: Base directory from QM35 device instance.
 * └─fwlogs: Main log entries (export all logs from all FW log modules).
 *
 * Notes:
 * - Reading fwlogs will cleanup memory and remove all stored logs.
 */

/* Debugfs tree structure used:
 *
 * /sys/kernel/debug/uwb/spiX.Y: Base directory from QM35 core instance.
 * └─logs: Sub-directory created/removed by this module.
 *   ├─xx_module_name: Directories created for each log module returned by the FW
 *   │ │               (xx is the source id followed by real module name).
 *   │ ├─log: This module log entries.
 *   │ └─level: allow to read & write log level of the module in the FW.
 *   ├─yy_other_module_name
 *
 * Notes:
 * - Reading logs/xx_module/log will remove only logs for this module. These
 *   logs are removed from fwlogs in device sysfs too!
 */

#define PARENT_NAME_HDR "xx_" /* "xx_name" */
#define PARENT_NAME_OFFSET (sizeof(PARENT_NAME_HDR) - 1)

/**
 * struct qm35_logs_header - Header use in transport logs (R/W).
 * @cmd_id: Command id (listed in qm35_logs_cmd_id).
 * @body_size: Payload length (after the header).
 */
struct qm35_logs_header {
	uint16_t cmd_id;
	uint16_t body_size;
};

/**
 * struct qm35_logs_cmd - Command used in transport logs (write side).
 * @hdr: See struct qm35_logs_header.
 * @source_id: Sources logs identifier given by QM35_LOGS_GET_SOURCES.
 * @level: Target level of log asked to the firmware.
 *
 * When @hdr.cmd_id is
 * * QM35_LOGS_GET_SOURCES, source_id and level are ignored
 * * QM35_LOGS_GET_LEVEL, level is ignored
 */
struct qm35_logs_cmd {
	struct qm35_logs_header hdr;
	uint8_t source_id;
	uint8_t level;
};

/**
 * struct qm35_logs_rsp - Reception structure used in transport logs.
 * @hdr: See struct qm35_logs_header.
 * @data: Raw data sent by the firmware (size given in header).
 */
struct qm35_logs_rsp {
	struct qm35_logs_header hdr;
	char data[];
};

/**
 * struct qm35_qtraces_header - Header use in transport qtrace (R/W).
 * @body_size: Packet length (including the header).
 * @type_id: Packet type id (listed in enum qtrace_dump_packet_type).
 * @module_id: Module id.
 *
 * For info packet type, the module_id is the first record module_id. It is
 * followed by level, module name string and another records starting with
 * module_id.
 *
 * For data packet type, the module_is is the module_id for all dumped qtraces
 * in that packet.
 *
 * Keep sync with qtrace_dump.h.
 */
struct qm35_qtraces_header {
	uint16_t body_size;
	uint8_t type_id;
	uint8_t module_id;
};

#define QTRACE_PKT_HDR_SIZE sizeof(struct qm35_qtraces_header)

/**
 * qm35_logs_list_clear() - Remove all stored data.
 * @ll: Pointer to struct qm35_logs_list to work on.
 * @qm35: QM35 instance used to free packets.
 */
static void qm35_logs_list_clear(struct qm35_logs_list *ll, struct qm35 *qm35)
{
	struct list_head *l = &ll->list;
	struct sk_buff *s;
	unsigned long flags;

	spin_lock_irqsave(&ll->lock, flags);
	while ((s = list_first_entry_or_null(l, struct sk_buff, list))) {
		list_del(&s->list);
		ll->count--;
		spin_unlock_irqrestore(&ll->lock, flags);
		kfree_skb(s);
		spin_lock_irqsave(&ll->lock, flags);
	}
	spin_unlock_irqrestore(&ll->lock, flags);
}

/**
 * qm35_logs_list_remove_first() - Remove first packed stored.
 * @ll: Pointer to struct qm35_logs_list to work on.
 * @qm35: QM35 instance used to free packets.
 *
 * Remove the first log packet stored in the list IF list is full (if count >=
 * QM35_LOGS_MAX_PACKET_STORED).
 * Called BEFORE a LOG packet is added to the list. Not used for QTRACE packets.
 */
static void qm35_logs_list_remove_first(struct qm35_logs_list *ll,
					struct qm35 *qm35)
{
	struct sk_buff *first;
	unsigned long flags;

	spin_lock_irqsave(&ll->lock, flags);
	if (ll->count < QM35_LOGS_MAX_PACKET_STORED) {
		spin_unlock_irqrestore(&ll->lock, flags);
		return;
	}
	first = list_first_entry(&ll->list, struct sk_buff, list);
	list_del(&first->list);
	ll->count--;
	spin_unlock_irqrestore(&ll->lock, flags);
	kfree_skb(first);
}

/**
 * qm35_logs_list_append() - Add a new log at the end of the queue.
 * @ll: Pointer to struct qm35_logs_list to work on.
 * @skb: packet to store
 *
 * We store directly the provided packet to avoid data copy. It will be freed
 * later, when an application open and read the log file.
 *
 * Returns: Zero on success or negative error.
 */
static void qm35_logs_list_append(struct qm35_logs_list *ll,
				  struct sk_buff *skb)
{
	unsigned long flags;

	spin_lock_irqsave(&ll->lock, flags);
	list_add_tail(&skb->list, &ll->list);
	ll->count++;
	spin_unlock_irqrestore(&ll->lock, flags);
}

/**
 * qm35_qtraces_read() - Read qtraces.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @count: Buffer size.
 * @pos: Current offset.
 *
 * Copy all available qtrace packets to provided buffer.
 *
 * Returns: read size on success, else a negative error code.
 */
static ssize_t qm35_qtraces_read(struct file *filp, struct kobject *kobp,
				 struct bin_attribute *bin_attr, char *buf,
				 loff_t pos, size_t count)
{
	struct qm35_logs *qml =
		container_of(bin_attr, struct qm35_logs, qtraces.bin_attr);
	struct qm35_logs_list *ll = &qml->qtraces;
	struct list_head *l = &ll->list;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned copied = 0;
	int error = 0;
	size_t remain = count;
	char __user *cur = buf;

	((void)pos); /* unused */

	spin_lock_irqsave(&ll->lock, flags);
	while ((skb = list_first_entry_or_null(l, struct sk_buff, list))) {
		if (remain < skb->len) {
			error = copied ? 0 : -ENOSPC;
			break; /* no space in buffer. */
		}
		/* Early delete from list. */
		list_del(&skb->list);
		ll->count--;
		/* Ensure producer thread can add new entries in list during
		 * copy and free. */
		spin_unlock_irqrestore(&ll->lock, flags);

		/* Copy this qtrace packet. */
		memcpy(cur, skb->data, skb->len);
		remain -= skb->len;
		cur += skb->len;
		copied++;

		/* Free packet. */
		kfree_skb(skb);

		/* Re-lock to continue for list reading. */
		spin_lock_irqsave(&ll->lock, flags);
	}
	spin_unlock_irqrestore(&ll->lock, flags);
	if (error)
		return error;
	return cur - buf;
}

/**
 * qm35_qtraces_write() - Write qtraces.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @count: Buffer size.
 * @pos: Current offset.
 *
 * Send provided buffer directly to FW using the QTRACE message type.
 *
 * Returns: write size on success, else a negative error code.
 */
static ssize_t qm35_qtraces_write(struct file *filp, struct kobject *kobp,
				  struct bin_attribute *bin_attr, char *buf,
				  loff_t pos, size_t count)
{
	struct qm35_logs *qml =
		container_of(bin_attr, struct qm35_logs, qtraces.bin_attr);
	ssize_t rc;

	mutex_lock(&qml->file_mutex);
	rc = qm35_transport_send(qml->qm35, QM35_TRANSPORT_MSG_QTRACE, buf,
				 count);
	mutex_unlock(&qml->file_mutex);
	if (rc < 0)
		return rc;
	return count;
}

/**
 * qm35_qtraces_packet_recv() - Packet handler for QTRACE messages.
 * @data: Pointer to qm35_logs structure.
 * @skb: QTRACE packet received.
 *
 * This is the registered QTRACE packet handler. It stores the received packet
 * in a packet list attached to the ``qtraces`` file in device sysfs.
 *
 * The ``qtraces`` file is also notified with sysfs_notify() to allow user-space
 * application to be informed when new QTRACE packet is available.
 */
static void qm35_qtraces_packet_recv(void *data, struct sk_buff *skb)
{
	struct qm35_logs *qml = (struct qm35_logs *)data;
	struct qm35_qtraces_header *qtrace_pkt;

	/* Need to be error prone for packet coming from FW. */
	if (skb->len < QTRACE_PKT_HDR_SIZE)
		goto freepkt;
	qtrace_pkt = (struct qm35_qtraces_header *)skb->data;
	if (skb->len < qtrace_pkt->body_size)
		goto freepkt;
	/* All packet will be processed by user-space application.
	 * Save all of them in the right list. */
	qm35_logs_list_append(&qml->qtraces, skb);
	if (qml->qtraces.bin_attr.attr.name) {
		sysfs_notify(&qml->dev->kobj, NULL,
			     qml->qtraces.bin_attr.attr.name);
	}
	return;

freepkt:
	kfree_skb(skb);
	return;
}

/**
 * check_name() - Helper to check module name.
 * @parent: Pointer to struct dentry of parent directory.
 * @log_pkt: Log packet to check.
 *
 * Helper function for qm35_logs_read_common() to ease reading code.
 *
 * Returns: True if module name in log packet matches the parent name
 *          else false.
 */
static bool check_name(struct dentry *parent, struct qm35_logs_rsp *log_pkt)
{
	/* Logs look like this : [00:00:00.000,086] <inf> calib_init: (...)
	 * calib_init is the module name.
	 * The log format is defined by zephyr log backend so we need to find the
	 * text after ">", there was ability to change the log format in zephyr so
	 * there maybe more work than just catching this. */
	const char *parent_name = (const char *)parent->d_name.name;
	const char *pname = &parent_name[PARENT_NAME_OFFSET];
	size_t plen = strlen((const char *)pname);
	char *pkt_end = log_pkt->data + log_pkt->hdr.body_size;
	char *module_name_end;
	char *module_name_pos =
		strnchr(log_pkt->data, pkt_end - log_pkt->data, '>');
	if (!module_name_pos)
		return false;
	/* Skip "> " */
	module_name_pos += 2;
	/* Check buffer overflow and compare module name size */
	module_name_end =
		strnchr(module_name_pos, pkt_end - module_name_pos, ':');
	if (!module_name_end)
		return false;
	if ((module_name_pos + plen) >= pkt_end ||
	    (module_name_end - module_name_pos) != plen)
		return false;

	if (strncmp(pname, module_name_pos, plen) != 0)
		return false;

	return true;
}

/**
 * qm35_logs_read_common() - Common function for log reading.
 * @qml: The instance pointer.
 * @parent: The pointer to parent dentry of file.
 * @buf: Userspace buffer where to put logs.
 * @count: Size of buffer.
 * @ppos: Current offset.
 *
 * If @parent is NULL, it is called by qm35_logs_read(), the read callback of
 * "fwlogs" binary attribute file in device sysfs. So the provided buffer is
 * kernel space, not in user-space.
 *
 * Returns: Number of bytes added to buffer or negative error.
 */
static ssize_t qm35_logs_read_common(struct qm35_logs *qml,
				     struct dentry *parent, char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct qm35_logs_list *ll = &qml->logs;
	struct sk_buff *skb, *n;
	unsigned long flags;
	unsigned copied = 0;
	int error = 0;
	size_t remain = count;
	char __user *cur = buf;

	spin_lock_irqsave(&ll->lock, flags);
	/* TODO: Remove per-category logs in debugfs (UWB-20260).
	 *
	 * Per-category logs bring little value in practice, compared to fwlogs
	 * file in sysfs.
	 * Furthermore, per-category logs depend on filtering using the
	 * check_name() function, which forces us to acquire the lock for the
	 * duration of the iteration over the whole list.
	 * Without the lock, it could be possible for 2 threads concurrently
	 * reading logs to call list_del() on the same element of the list,
	 * causing a kernel panic.
	 * And finally keeping the lock also has an impact on driver latency:
	 * when a log packet is received while a read operation is in progress,
	 * the SPI thread first has to wait for it to complete.
	 *
	 * Once the per-category logs and filtering are removed, the
	 * list_for_each_entry_safe() loop can be replaced with a
	 * while(list_first_entry_or_null()) one, and the lock can be release
	 * between iterations, just like in qm35_qtraces_read().
	 */
	list_for_each_entry_safe (skb, n, &ll->list, list) {
		struct qm35_logs_rsp *log_pkt;
		bool copy;

		log_pkt = (struct qm35_logs_rsp *)skb->data;
		if (remain < log_pkt->hdr.body_size) {
			error = copied ? 0 : -ENOSPC;
			break; /* no space in buffer. */
		}

		copy = !parent || check_name(parent, log_pkt);
		if (!copy)
			continue;

		/* Copy this log entry. */
		if (parent) {
			if (copy_to_user(cur, log_pkt->data,
					 log_pkt->hdr.body_size)) {
				error = -EFAULT;
				break;
			}
		} else {
			memcpy(cur, log_pkt->data, log_pkt->hdr.body_size);
		}

		/* Update sizes and position. */
		remain -= log_pkt->hdr.body_size;
		cur += log_pkt->hdr.body_size;
		copied++;

		/* Remove from list and free packet. */
		list_del(&skb->list);
		ll->count--;
		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&ll->lock, flags);
	if (error)
		return error;
	*ppos += cur - buf;
	return cur - buf;
}

/**
 * qm35_logs_module_open() - File open callback function.
 * @inode: inode opened
 * @file: file opened
 *
 * Returns: 0.
 */
static int qm35_logs_module_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

/**
 * qm35_logs_module_release() - File release callback function.
 * @inode: The inode of the miscdevice.
 * @file: The file of the opened miscdevice to close.
 *
 * Context: User context.
 * Return: 0.
 */
static int qm35_logs_module_release(struct inode *inode, struct file *file)
{
	return 0;
}

/**
 * qm35_logs_module_read() - Read logs level of a source.
 * @f: Standard file struct.
 * @buf: Userland data buffer.
 * @count: Userland data length.
 * @ppos: Position offset.
 *
 * Source is identified by the parent dir whose name start with the source_id.
 *
 * Returns: Zero or a negative error
 */
static ssize_t qm35_logs_module_read(struct file *f, char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct qm35_logs *qml = (struct qm35_logs *)(f->private_data);
	struct dentry *parent = f->f_path.dentry->d_parent;

	return qm35_logs_read_common(qml, parent, buf, count, ppos);
}

static const struct file_operations qm35_logs_module_fops = {
	.owner = THIS_MODULE,
	.open = qm35_logs_module_open,
	.release = qm35_logs_module_release,
	.read = qm35_logs_module_read,
};

/**
 * qm35_logs_module_level_read() - Read logs level of a source.
 * @f: Standard file struct.
 * @buf: Userland data buffer.
 * @count: Userland data length.
 * @ppos: Position offset.
 *
 * Source is identified by the parent dir whose name start with the source_id.
 *
 * Returns: Zero or a negative error
 */
static ssize_t qm35_logs_module_level_read(struct file *f, char __user *buf,
					   size_t count, loff_t *ppos)
{
	const uint8_t *parent = f->f_path.dentry->d_parent->d_name.name;
	unsigned int id = simple_strtoul((const char *)parent, NULL, 10);
	struct qm35_logs_cmd p = { .hdr.cmd_id = QM35_LOGS_GET_LEVEL };
	struct qm35_logs *qml = (struct qm35_logs *)(f->private_data);
	int ret;
	char buffer[16];

	if (!qml)
		return -ENODEV;

	p.hdr.body_size = sizeof(p) - sizeof(p.hdr);
	p.source_id = id;

	qml->rx_id = QM35_LOGS_MAX;

	mutex_lock(&qml->file_mutex);

	qm35_transport_send(qml->qm35, QM35_TRANSPORT_MSG_LOG, &p, sizeof(p));
	/* LCOV_EXCL_BR_START */ /* Generate 21 branches! */
	ret = wait_event_interruptible_timeout(qml->rx_queue,
					       qml->rx_id == p.hdr.cmd_id, HZ);
	/* LCOV_EXCL_BR_STOP */

	mutex_unlock(&qml->file_mutex);

	if (ret == 0)
		return -EIO;

	ret = snprintf(buffer, 16, "%d\n", qml->rx_result);
	if (copy_to_user(buf, buffer, ret))
		ret = -EFAULT;
	return ret;
}

/**
 * qm35_logs_module_level_write() - Set logs level of a source.
 * @f: Standard file struct.
 * @buf: Userland data buffer.
 * @count: Userland data length.
 * @ppos: Position offset.
 *
 * Source is identified by the parent dir whose name start with the source_id.
 *
 * Returns: read size or a negative error
 */
static ssize_t qm35_logs_module_level_write(struct file *f,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	const uint8_t *parent = f->f_path.dentry->d_parent->d_name.name;
	unsigned int id = simple_strtoul((const char *)parent, NULL, 10);
	struct qm35_logs_cmd p = { .hdr.cmd_id = QM35_LOGS_SET_LEVEL };
	struct qm35_logs *qml = (struct qm35_logs *)(f->private_data);
	uint8_t level;
	ssize_t rc;
	int ret;

	if (!qml)
		return -ENODEV;

	rc = kstrtou8_from_user(buf, count, 10, &level);
	if (rc < 0)
		return rc;

	p.hdr.body_size = sizeof(p) - sizeof(p.hdr);
	p.source_id = id;
	p.level = level;

	qml->rx_id = QM35_LOGS_MAX;

	mutex_lock(&qml->file_mutex);

	qm35_transport_send(qml->qm35, QM35_TRANSPORT_MSG_LOG, &p, sizeof(p));
	/* LCOV_EXCL_BR_START */ /* Generate 21 branches! */
	ret = wait_event_interruptible_timeout(qml->rx_queue,
					       qml->rx_id == p.hdr.cmd_id, HZ);
	/* LCOV_EXCL_BR_STOP */

	mutex_unlock(&qml->file_mutex);

	if (ret == 0)
		return -EIO;
	/* Could be too big / more than what was compiled / or need to set
	 * CONFIG_LOG_RUNTIME_FILTERING in QM35 firmware. */
	if (qml->rx_result != level)
		return -EINVAL;

	return count;
}

static const struct file_operations qm35_logs_module_level_fops = {
	.owner = THIS_MODULE,
	.open = qm35_logs_module_open,
	.release = qm35_logs_module_release,
	.read = qm35_logs_module_level_read,
	.write = qm35_logs_module_level_write,
};

/**
 * qm35_logs_read() - Read logs.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @count: Buffer size.
 * @pos: Current offset.
 *
 * Just call qm35_logs_read_common() with NULL parent to read all available
 * logs.
 *
 * Returns: read size on success, else a negative error code.
 */
static ssize_t qm35_logs_read(struct file *filp, struct kobject *kobp,
			      struct bin_attribute *bin_attr, char *buf,
			      loff_t pos, size_t count)
{
	struct qm35_logs *qml =
		container_of(bin_attr, struct qm35_logs, logs.bin_attr);
	return qm35_logs_read_common(qml, NULL, buf, count, &pos);
}

/**
 * qm35_logs_handle_get_src() - Handle QM35_LOGS_GET_SOURCES response.
 * @qml: qm35_logs structure
 * @log_pkt: packet received
 *
 * Returns: Zero on success or negative error.
 */
static int qm35_logs_handle_get_src(struct qm35_logs *qml,
				    struct qm35_logs_rsp *log_pkt)
{
	struct dentry *new_dir;
	char dirname[128];
	uint8_t i, id, nb_src;
	int pos = 0;

	if (log_pkt->hdr.body_size < 1) /* nb_src  */
		return -EINVAL;

	nb_src = log_pkt->data[pos++];

	for (i = 0; i < nb_src; i++) {
		int len;
		/* Check length is at least id + lvl + NUL char */
		if ((pos + 3) > log_pkt->hdr.body_size)
			return -EMSGSIZE;

		id = log_pkt->data[pos++];
		pos++; /* ignore level */

		len = strlen(&log_pkt->data[pos]) + 1;
		if ((pos + len) > log_pkt->hdr.body_size)
			return -EMSGSIZE;

		/* Module name is 30 char max in firmware. */
		snprintf(dirname, sizeof(dirname), "%02d_%s", id,
			 &log_pkt->data[pos]);
		pos += len;

		new_dir = debugfs_create_dir(dirname, qml->debugfs_path);
		if (IS_ERR(new_dir)) {
			dev_err(qml->dev, "Unable to create dir %s\n", dirname);
			continue;
		}
		debugfs_create_file("level", S_IRUSR, new_dir, qml,
				    &qm35_logs_module_level_fops);
		debugfs_create_file("log", S_IRUSR, new_dir, qml,
				    &qm35_logs_module_fops);
	}
	return 0;
}

/**
 * qm35_logs_packet_recv() - Packet handler for QM35_TRANSPORT_MSG_LOG messages.
 * @data: Pointer to qm35_logs structure.
 * @skb: LOG packet received.
 */
static void qm35_logs_packet_recv(void *data, struct sk_buff *skb)
{
	struct qm35_logs *qml = (struct qm35_logs *)data;
	struct device *dev = qml->dev;
	struct qm35_logs_rsp *log_pkt = (struct qm35_logs_rsp *)skb->data;

	/* Need to be error prone for packet coming from FW. */
	if (skb->len < sizeof(struct qm35_logs_header))
		goto freepkt;
	if (skb->len <
	    (sizeof(struct qm35_logs_header) + log_pkt->hdr.body_size))
		goto freepkt;

	switch (log_pkt->hdr.cmd_id) {
	case QM35_LOGS_DATA:
		qm35_logs_list_remove_first(&qml->logs, qml->qm35);
		qm35_logs_list_append(&qml->logs, skb);
#if LOG_PRINTK != 0
		dev_info(dev, "FW: %.*s\n", log_pkt->hdr.body_size,
			 log_pkt->data);
#endif
		if (qml->logs.bin_attr.attr.name) {
			sysfs_notify(&qml->dev->kobj, NULL,
				     qml->logs.bin_attr.attr.name);
		}
		return; /* Pkt will be freed on read */
	case QM35_LOGS_GET_SOURCES:
		if (qm35_logs_handle_get_src(qml, log_pkt))
			dev_err(dev, "corrupted QM35_LOGS_GET_SOURCES reply\n");
		break;
	case QM35_LOGS_SET_LEVEL:
	case QM35_LOGS_GET_LEVEL:
		if (log_pkt->hdr.body_size >= 2)
			qml->rx_result = log_pkt->data[1];
		else
			dev_err(dev, "bad log packet length\n");
		break;
	default:
		dev_warn(dev, "unknown log command received %d\n",
			 log_pkt->hdr.cmd_id);
		break;
	}

	qml->rx_id = log_pkt->hdr.cmd_id;
	wake_up(&qml->rx_queue);
freepkt:
	kfree_skb(skb);
	return;
}

static struct qm35_logs *qm35_logs_search(struct qm35 *qm35)
{
	/* We cannot rely anymore on the registered packet handler since it
	 * may be removed by qm35_logs_online() if it fails. Need to lookup
	 * the logs_list instead now.
	 */
	struct qm35_logs *qml;
	/* Search corresponding qm35_logs structure. */
	list_for_each_entry (qml, &logs_list, dev_list) {
		if (qml->qm35 == qm35)
			break;
	}
	if (list_entry_is_head(qml, &logs_list, dev_list))
		return NULL; /* Not found. */
	return qml;
}

/**
 * qm35_logs_request() - Retrieve all log sources from the firmware.
 * @work: The work_struct embedded in struct qm35_logs.
 *
 * This function request the LOG source from the FW. If reply is received
 * and handled correctly by the LOG packet handler, all the debugfs directories
 * are created.
 */
static void qm35_logs_request(struct work_struct *work)
{
	struct qm35_logs *qml =
		container_of(work, struct qm35_logs, request_work);
	struct device *dev = qml->dev;
	struct qm35 *qm35 = qml->qm35;
	int rc;

	const struct qm35_logs_cmd p = {
		.hdr = { .cmd_id = QM35_LOGS_GET_SOURCES, .body_size = 0, },
		.level = 0,
		.source_id = 0,
	};

	qml->rx_id = QM35_LOGS_MAX;
	rc = qm35_transport_send(qm35, QM35_TRANSPORT_MSG_LOG, &p, sizeof(p));
	if (rc) {
		dev_err(dev, "%s: Unable to send logs get sources command\n",
			THIS_MODULE->name);
		return;
	}
	/* LCOV_EXCL_BR_START */ /* Generate 22 branches! */
	rc = wait_event_interruptible_timeout(
		qml->rx_queue, qml->rx_id == QM35_LOGS_GET_SOURCES, HZ);
	/* LCOV_EXCL_BR_STOP */
	if (rc <= 0) { /* interrupted or timeout */
		dev_err(dev, "%s: Unable to retrieve firmware log sources\n",
			THIS_MODULE->name);
	}
}

/**
 * qm35_logs_init() - Initialise logs for specified QM35 core device.
 * @qm35: QM35 device instance
 *
 * Init local structure to handle incoming packets and register LOG and QTRACE
 * packets handlers using qm35_transport_register() so all received packets of
 * these type will be handled by this sub-module.
 *
 * The newly allocated struct qm35_logs is finally put in the logs_list to allow
 * instance management. After this function is called, the new instance is ready
 * to receive LOG and QTRACE packets.
 *
 * Returns: Zero or a negative error
 */
static int qm35_logs_init(struct qm35 *qm35)
{
	struct device *dev = qm35_get_device(qm35);
	struct qm35_logs *qml;
	int rc = -ENOMEM;

	qml = kzalloc(sizeof(*qml), GFP_KERNEL);
	if (!qml) {
		dev_err(dev, "%s: Cannot allocate memory\n", THIS_MODULE->name);
		goto error;
	}
	qml->qm35 = qm35;
	qml->dev = dev;

	INIT_LIST_HEAD(&qml->qtraces.list);
	spin_lock_init(&qml->qtraces.lock);

	rc = qm35_transport_register(qm35, QM35_TRANSPORT_MSG_QTRACE,
				     QM35_TRANSPORT_PRIO_NORMAL,
				     qm35_qtraces_packet_recv, qml);
	if (rc) {
		dev_err(dev, "%s: Fail to register QTRACE transport handler\n",
			THIS_MODULE->name);
		goto err_qtraces;
	}

	INIT_LIST_HEAD(&qml->logs.list);
	spin_lock_init(&qml->logs.lock);
	init_waitqueue_head(&qml->rx_queue);
	INIT_WORK(&qml->request_work, qm35_logs_request);
	mutex_init(&qml->file_mutex);

	rc = qm35_transport_register(qm35, QM35_TRANSPORT_MSG_LOG,
				     QM35_TRANSPORT_PRIO_NORMAL,
				     qm35_logs_packet_recv, qml);
	if (rc) {
		dev_err(dev, "%s: Fail to register LOG transport handler\n",
			THIS_MODULE->name);
		goto err_logs;
	}

	/* Save in local list */
	list_add_tail(&qml->dev_list, &logs_list);
	return 0;

err_logs:
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_QTRACE,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_qtraces_packet_recv);
err_qtraces:
	kfree(qml);
error:
	dev_err(dev, "%s: Failed to initialize (%d)\n", THIS_MODULE->name, rc);
	return rc;
}

/**
 * qm35_logs_online() - Initialise fwlogs and qtraces files for specified QM35 device.
 * @qm35: QM35 device instance
 *
 * Finish initialization of the after QM35 device is online. It creates ``fwlogs` and
 * ``qtraces`` in device root directory in sysfs.
 *
 * Then, it sends a LOG command to retrieve QM35 FW log modules and constructs
 * files tree in ``/sys/kernel/debug/uwb/DEVNAME/`` according the received LOG
 * INFO response.
 *
 * Returns: Zero or a negative error
 */
static int qm35_logs_online(struct qm35 *qm35)
{
	struct qm35_logs *qml = qm35_logs_search(qm35);
	struct device *dev;
	struct dentry *root;
	int rc = -EINVAL;

	if (!qml) {
		/* Another module have registered this message type.
		 * Do nothing for this QM35 instance. */
		dev = qm35_get_device(qm35);
		dev_err(dev, "%s: Cannot get associated qm35_logs instance!\n",
			THIS_MODULE->name);
		return rc;
	}
	dev = qml->dev;

	/* Create qtraces file. */
	sysfs_bin_attr_init(&qml->qtraces.bin_attr);
	qml->qtraces.bin_attr.size = 0;
	qml->qtraces.bin_attr.read = qm35_qtraces_read;
	qml->qtraces.bin_attr.write = qm35_qtraces_write;
	qml->qtraces.bin_attr.private = qml;
	qml->qtraces.bin_attr.attr.mode = 0644; /* RO */
	qml->qtraces.bin_attr.attr.name = "qtraces";
	rc = sysfs_create_bin_file(&dev->kobj, &qml->qtraces.bin_attr);
	if (rc) {
		dev_err(dev,
			"%s: Unable to create 'qtraces' file in device sysfs\n",
			THIS_MODULE->name);
		goto err_qtraces;
	}

	/* Create main log file. */
	sysfs_bin_attr_init(&qml->logs.bin_attr);
	qml->logs.bin_attr.size = 0;
	qml->logs.bin_attr.read = qm35_logs_read;
	qml->logs.bin_attr.private = qml;
	qml->logs.bin_attr.attr.mode = 0444; /* RO */
	qml->logs.bin_attr.attr.name = "fwlogs";
	rc = sysfs_create_bin_file(&dev->kobj, &qml->logs.bin_attr);
	if (rc) {
		dev_err(dev,
			"%s: Unable to create 'fwlogs' file in device sysfs\n",
			THIS_MODULE->name);
		goto err_main_log;
	}

	/* Get dentry for /sys/kernel/debug/uwb/spiX.Y directory. */
	root = qm35_get_debug_root(qm35);
	/* Create logs sub-directory. */
	qml->debugfs_path = debugfs_create_dir("logs", root);
	if (IS_ERR_OR_NULL(qml->debugfs_path)) {
		dev_err(dev,
			"%s: Unable to create 'logs' directory in debugfs\n",
			THIS_MODULE->name);
		rc = PTR_ERR(qml->debugfs_path);
		goto err_dir;
	}

	/* Retrieve log sources from FW in another thread.
	 * Failure to retrieve log sources isn't a critical error.
	 */
	schedule_work(&qml->request_work);
	return 0;

err_dir:
	sysfs_remove_bin_file(&dev->kobj, &qml->logs.bin_attr);
err_main_log:
	qml->logs.bin_attr.attr.name = NULL;
	sysfs_remove_bin_file(&dev->kobj, &qml->qtraces.bin_attr);
err_qtraces:
	qml->qtraces.bin_attr.attr.name = NULL;
	/* Ensure no more packets can be received (no file to read them) */
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_LOG,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_logs_packet_recv);
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_QTRACE,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_qtraces_packet_recv);
	dev_err(dev, "%s: Failed to finish initialization (%d)\n",
		THIS_MODULE->name, rc);
	return rc;
}

/**
 * qm35_logs_deinit() - Cleanup logs for specified QM35 core device.
 * @qm35: QM35 device instance
 *
 * It removes all the files from the ``/sys/kernel/debug/uwb/DEVNAME`` tree,
 * removes the ``fwlogs`` and ``qtraces`` files from device root sysfs directory,
 * and frees all unconsumed LOG and QTRACE packets.
 *
 * It will also call ``qm35_transport_unregister()`` to unregister the LOG and
 * QTRACE message handlers in transport API.
 */
static void qm35_logs_deinit(struct qm35 *qm35)
{
	struct device *dev = qm35_get_device(qm35);
	struct qm35_logs *qml = qm35_logs_search(qm35);

	if (!qml) {
		/* Another module have registered this message type.
		 * Do nothing for this QM35 instance. */
		dev_err(dev, "%s: Cannot get associated LOG instance!\n",
			THIS_MODULE->name);
		return;
	}

	/* Remove from list. */
	list_del(&qml->dev_list);

	/* Remove all files. */
	if (qml->logs.bin_attr.attr.name)
		sysfs_remove_bin_file(&dev->kobj, &qml->logs.bin_attr);
	if (qml->qtraces.bin_attr.attr.name)
		sysfs_remove_bin_file(&dev->kobj, &qml->qtraces.bin_attr);
	if (!IS_ERR_OR_NULL(qml->debugfs_path))
		debugfs_remove(qml->debugfs_path);

	/* Remove transport callback */
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_LOG,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_logs_packet_recv);
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_QTRACE,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_qtraces_packet_recv);

	/* Free all remaining packets. */
	mutex_lock(&qml->file_mutex);
	qm35_logs_list_clear(&qml->logs, qml->qm35);
	qm35_logs_list_clear(&qml->qtraces, qml->qm35);
	mutex_unlock(&qml->file_mutex);
	mutex_destroy(&qml->file_mutex);

	/* Free instance. */
	kfree(qml);
}

/**
 * qm35_logs_notifier() - Callback function for struct notifier_block.
 * @nb: The notifier_block.
 * @action: The notifier event.
 * @data: The data provided by the notifier.
 *
 * This notifier callback function handles the create/destroy of the sysfs and
 * debugfs files according the new/delete events from QM35 core for all QM35
 * device instances.
 *
 * It calls ``qm35_logs_init()`` to instantiate the required logs and qtraces
 * structure when a NEW event is received.
 *
 * It calls ``qm35_logs_online()`` to create all sysfs/debugfs files when an
 * ONLINE event is received.
 *
 * If calls ``qm35_logs_deinit()`` to revert what previous function
 * made when a DELETE event is received.
 *
 * Context: Kernel thread context.
 * Return: 0 on success, else -EINVAL if @data is NULL or if the @action is
 * unknown, else the qm35_logs_init() error code.
 */
static int qm35_logs_notifier(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	enum qm35_notifier_events event = action;
	struct qm35 *qm35 = data;
	int rc = 0;

	if (!qm35)
		return notifier_from_errno(-EINVAL);

	switch (event) {
	case QM35_NOTIFIER_EVENT_NEW:
		/* Ignore the return value. We don't want to stop notifier chain. */
		qm35_logs_init(qm35);
		break;
	case QM35_NOTIFIER_EVENT_ONLINE:
		/* Ignore the return value. We don't want to stop notifier chain. */
		qm35_logs_online(qm35);
		break;
	case QM35_NOTIFIER_EVENT_DELETE:
		qm35_logs_deinit(qm35);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return notifier_from_errno(rc);
}

static struct notifier_block nb = {
	.notifier_call = qm35_logs_notifier,
	.next = NULL,
	.priority = 0,
};

/**
 * qm35_logs_register_notifier() - Install QM35 notifier callback.
 *
 * Called when module is loaded and calls qm35_register_notifier() to get
 * notified when QM35 device instance is modified.
 *
 * Context: User context.
 * Return: 0 on success, else qm35_register_notifier() error code.
 */
static int qm35_logs_register_notifier(void)
{
	return qm35_register_notifier(&nb);
}

/**
 * qm35_logs_unregister_notifier() - Remove logs and QM35 notifier callback.
 *
 * Called when module is unloaded and calls qm35_unregister_notifier() to
 * remove the notifier callback. It also calls qm35_logs_deinit() for all
 * known QM35 instances to remove all files and free allocated structures.
 *
 * Context: User context.
 * Return: 0 on success, else qm35_unregister_notifier() error code.
 */
static int qm35_logs_unregister_notifier(void)
{
	struct qm35_logs *cur, *n;

	list_for_each_entry_safe (cur, n, &logs_list, dev_list) {
		qm35_logs_deinit(cur->qm35);
	}
	return qm35_unregister_notifier(&nb);
}

#ifndef QM35_LOGS_TESTS

static int __init qm35_logs_module_init(void)
{
	return qm35_logs_register_notifier();
}

static void __exit qm35_logs_module_exit(void)
{
	qm35_logs_unregister_notifier();
}
module_init(qm35_logs_module_init);
module_exit(qm35_logs_module_exit);

#ifdef GITVERSION
MODULE_VERSION(GITVERSION);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Romuald Després <romuald.despres@qorvo.com>");
MODULE_DESCRIPTION("Qorvo QM35 Logs driver");

#endif /* !QM35_LOGS_TESTS */
