/*
 * This file is part of the UWB stack for linux.
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
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fsnotify.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/ratelimit.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "qm35_core.h"
#include "qm35_coredump.h"
#include "qm35_notifier.h"
#include "qm35_transport.h"

static u32 qm35_coredumps_max = 32;
module_param_named(coredumps_max, qm35_coredumps_max, uint, 0644);
MODULE_PARM_DESC(coredumps_max,
		 "Maximum number of simultaneously saved coredumps");

static u32 qm35_coredumps_burst = DEFAULT_RATELIMIT_BURST;
module_param_named(coredumps_burst, qm35_coredumps_burst, uint, 0444);
MODULE_PARM_DESC(coredumps_burst, "Maximum number of saved coredumps for 30s");

#define COREDUMPS_RATELIMIT_INTERVAL (30 * HZ)

static LIST_HEAD(coredumps_list);

/* Coredump packet format is:
 * - cmd_id, uint8_t, where values can be one from enum qm35_coredump_cmd_id,
 * - command payload or nothing.
 *
 * Where command payload is:
 * - struct coredump_hdr_ntf for COREDUMP_HEADER_NTF,
 * - coredump data for COREDUMP_BODY_NTF,
 * - status, uint8_t, for COREDUMP_RCV_STATUS
 * - empty for COREDUMP_FORCE_CMD.
 */

enum qm35_coredump_cmd_id {
	COREDUMP_HEADER_NTF,
	COREDUMP_BODY_NTF,
	COREDUMP_RCV_STATUS,
	COREDUMP_FORCE_CMD,
};

enum qm35_coredump_status { COREDUMP_RCV_NACK, COREDUMP_RCV_ACK };

struct __packed coredump_hdr_ntf {
	uint32_t size;
	uint16_t csum;
};

struct __packed coredump_pkt {
	uint8_t cmd_id;
	union {
		struct coredump_hdr_ntf hdr;
		uint8_t status;
		uint8_t data[0];
	};
};

static void qm35_coredumpx_remove(struct work_struct *work);
static ssize_t qm35_coredumpx_read(struct file *filp, struct kobject *kobp,
				   struct bin_attribute *bin_attr, char *buf,
				   loff_t pos, size_t count);

static void qm35_coredump_free_data(struct qm35_coredump_data *cdd)
{
	list_del(&cdd->list);
	kfree(cdd);
}

static uint16_t qm35_coredump_csum(struct qm35_coredump_data *cdd)
{
	uint16_t csum = 0;
	uint8_t *val = (uint8_t *)cdd->buffer;
	uint8_t *end = (uint8_t *)cdd->buffer + cdd->size;
	while (val < end)
		csum += *val++;
	return csum;
}

static void qm35_coredump_send_rcv_status(struct qm35_coredump *cd, uint8_t s)
{
	struct coredump_pkt reply;
	reply.cmd_id = COREDUMP_RCV_STATUS;
	reply.status = s;
	qm35_transport_send(cd->qm35, QM35_TRANSPORT_MSG_COREDUMP, &reply,
			    2); /* cannot use sizeof(reply). */
}

static void qm35_coredump_force(struct qm35_coredump *cd)
{
	uint8_t packet = COREDUMP_FORCE_CMD;
	qm35_transport_send(cd->qm35, QM35_TRANSPORT_MSG_COREDUMP, &packet,
			    sizeof(packet));
}

static struct qm35_coredump *qm35_coredump_search(struct qm35 *qm35)
{
	struct qm35_coredump *cd = NULL;
	if (!qm35)
		return NULL;
	/* Search corresponding coredump structure. */
	list_for_each_entry (cd, &coredumps_list, dev_list) {
		if (cd->qm35 == qm35)
			break;
	}
	if (list_entry_is_head(cd, &coredumps_list, dev_list))
		return NULL; /* Not found. */
	return cd;
}

static void qm35_coredump_check(struct work_struct *work)
{
	struct qm35_coredump *cd =
		container_of(work, struct qm35_coredump, dwork.work);
	struct device *dev = cd->dev;
	struct qm35_coredump_data *cdd;

	spin_lock(&cd->dump_lock);
	if (list_empty(&cd->dump_list)) {
		spin_unlock(&cd->dump_lock);
		dev_warn(dev, "Failed to check coredump, already read?\n");
		return;
	}
	cdd = list_first_entry(&cd->dump_list, struct qm35_coredump_data, list);
	spin_unlock(&cd->dump_lock);
	if (cdd->remain) {
		const char fmt[] = "Coredump incomplete: expected size: %ld, "
				   "received: %ld, remaining: %lu\n";
		dev_warn(dev, fmt, cdd->size, cdd->offset, cdd->remain);
		qm35_coredump_send_rcv_status(cd,
					      cdd->status & COREDUMP_RCV_ACK);
	}
}

static int qm35_coredump_handle_header(struct qm35_coredump *cd,
				       struct sk_buff *skb)
{
	struct device *dev = cd->dev;
	struct qm35_coredump_data *cdd;
	struct coredump_hdr_ntf chn;
	size_t length = skb->len;

	if (length < sizeof(chn))
		return -EINVAL;

	/* May be unaligned, so copy to stack to avoid problem when porting. */
	memcpy(&chn, skb->data, sizeof(chn));
	skb_pull(skb, sizeof(chn));

	dev_info(dev, "Receiving coredump with len %d and crc 0x%x\n", chn.size,
		 chn.csum);

	cdd = kmalloc(sizeof(*cdd) + chn.size, GFP_KERNEL);
	if (!cdd) {
		dev_err(dev, "Failed to allocate coredump memory\n");
		return -ENOMEM;
	}

	/* Fully initialize structure */
	INIT_LIST_HEAD(&cdd->list);
	cdd->size = chn.size;
	cdd->remain = chn.size;
	cdd->csum = chn.csum;
	cdd->offset = 0;
	cdd->status = COREDUMP_RCV_NACK;
	snprintf(cdd->name, 16, "coredump%d",
		 atomic_add_return(1, &cd->dump_num));

	spin_lock(&cd->dump_lock);
	/* Check previous coredump was fully received. QM may have been reset! */
	if (!list_empty(&cd->dump_list)) {
		const char fmt[] = "Deletes incomplete coredump with len %lu. "
				   "Missing %lu bytes\n";
		struct qm35_coredump_data *prev;
		prev = list_first_entry(&cd->dump_list,
					struct qm35_coredump_data, list);
		if (prev->remain) {
			/* Since a new coredump reception is started, previous
			 * will never be completed. Delete it. */
			dev_warn(dev, fmt, cdd->size, cdd->remain);
			qm35_coredump_free_data(prev);
			cd->dump_cnt--;
		}
	}
	list_add(&cdd->list, &cd->dump_list);
	cd->dump_cnt++;
	spin_unlock(&cd->dump_lock);

	schedule_delayed_work(&cd->dwork, HZ * 10);
	return 0;
}

static int qm35_coredump_handle_body(struct qm35_coredump *cd,
				     struct sk_buff *skb)
{
	struct device *dev = cd->dev;
	size_t length = skb->len;
	struct qm35_coredump_data *cdd;
	uint16_t csum;
	int ret;

	spin_lock(&cd->dump_lock);
	if (list_empty(&cd->dump_list)) {
		spin_unlock(&cd->dump_lock);
		dev_err(dev, "Failed to save coredump, memory not allocated\n");
		return -EINVAL;
	}
	cdd = list_first_entry(&cd->dump_list, struct qm35_coredump_data, list);
	spin_unlock(&cd->dump_lock);
	if (length > cdd->remain) {
		dev_err(dev,
			"Coredump overflow: max size: %ld, wr_idx: %ld, cd size: %ld\n",
			cdd->size, cdd->offset, length);
		return -ENOMEM;
	}
	memcpy(cdd->buffer + cdd->offset, skb->data, length);
	cdd->offset += length;
	cdd->remain -= length;
	if (cdd->remain)
		return 0;

	/* Terminated. All received. */
	cancel_delayed_work(&cd->dwork);
	csum = qm35_coredump_csum(cdd);
	dev_info(dev, "Coredump received, csum: 0x%x, header csum: 0x%x\n",
		 csum, cdd->csum);
	if (csum == cdd->csum)
		cdd->status |= COREDUMP_RCV_ACK;
	qm35_coredump_send_rcv_status(cd, cdd->status & COREDUMP_RCV_ACK);

	/* Rate-limit the number of stored coredumps. */
	if (!__ratelimit(&cd->dump_rls)) {
		dev_warn(dev, "File %s dropped because rate limit\n",
			 cdd->name);
		goto freedump;
	}
	/* Check number of stored coredumps. */
	spin_lock(&cd->dump_lock);
	if (cd->dump_cnt > qm35_coredumps_max) {
		dev_warn(dev, "File %s dropped because max limit\n", cdd->name);
		goto freedump_locked;
	}
	spin_unlock(&cd->dump_lock);

	/* Create sysfs bin attr file to present this coredump. */
	memset(&cdd->bin_attr, 0, sizeof(cdd->bin_attr));
	sysfs_bin_attr_init(&cdd->bin_attr);
	cdd->bin_attr.attr.name = cdd->name;
	cdd->bin_attr.attr.mode = 0444; /* RO */
	cdd->bin_attr.size = cdd->offset;
	cdd->bin_attr.read = qm35_coredumpx_read;
	cdd->bin_attr.private = cd;
	ret = sysfs_create_bin_file(&dev->kobj, &cdd->bin_attr);
	if (ret) {
		dev_warn(dev, "Failed to create read-only %s file (err %d)\n",
			 cdd->name, ret);
		/* Error sysfs to expose this coredump. Delete it. */
		cdd->bin_attr.attr.name = NULL;
		goto freedump;
	} else {
		dev_info(dev, "File %s created\n", cdd->name);
		INIT_WORK(&cdd->rfw, qm35_coredumpx_remove);
		/* Notify coredump file to wakeup application. */
		if (cd->bin_attr.attr.name) {
			sysfs_notify(&cd->dev->kobj, NULL,
				     cd->bin_attr.attr.name);
		}
	}
	return 0;

freedump:
	spin_lock(&cd->dump_lock);
freedump_locked:
	qm35_coredump_free_data(cdd);
	cd->dump_cnt--;
	spin_unlock(&cd->dump_lock);
	return 0;
}

/**
 * qm35_coredump_handle_pkt() - Handler one COREDUMP packet.
 * @cd: Pointer to struct qm35_coredump.
 * @skb: The COREDUMP packet to process.
 *
 * Context: Called from qm35_transport_event() with received packet.
 */
static void qm35_coredump_handle_pkt(struct qm35_coredump *cd,
				     struct sk_buff *skb)
{
	struct device *dev = cd->dev;
	enum qm35_coredump_cmd_id cmd;
	size_t length = skb->len;

	if (length < sizeof(uint8_t))
		goto error;
	/* Get received command. */
	cmd = (enum qm35_coredump_cmd_id)skb->data[0];
	skb_pull(skb, sizeof(uint8_t));
	/* Handle command. */
	switch (cmd) {
	case COREDUMP_HEADER_NTF:
		qm35_coredump_handle_header(cd, skb);
		break;
	case COREDUMP_BODY_NTF:
		qm35_coredump_handle_body(cd, skb);
		break;
	default:
		dev_err(dev, "Wrong coredump cmd_id received: 0x%x\n",
			(unsigned)cmd);
		break;
	}
error:
	kfree_skb(skb);
}

/**
 * qm35_coredump_event_cb() - Packet handler to handle COREDUMP packets.
 * @data: Pointer to qm35_coredump structure.
 * @skb: COREDUMP packet received.
 *
 * It is just a simple wrapper to call qm35_coredump_handle_pkt().
 *
 * Context: Always called from qm35_transport_event().
 */
static void qm35_coredump_event_cb(void *data, struct sk_buff *skb)
{
	qm35_coredump_handle_pkt((struct qm35_coredump *)data, skb);
}

/**
 * qm35_coredumpx_remove() - Async remove coredumpX buffer and sysfs file.
 * @work: Pointer to rfw field inside struct qm35_coredump_data.
 *
 * As kernel 5.4 and below don't export sysfs_remove_file_self() we need to use
 * another mechanism based on sysfs_remove_bin_file() which cannot be called
 * from a sysfs file callbacks.
 *
 * So all is made asynchronously by this work function.
 */
static void qm35_coredumpx_remove(struct work_struct *work)
{
	struct qm35_coredump_data *cdd =
		container_of(work, struct qm35_coredump_data, rfw);
	struct qm35_coredump *cd = cdd->bin_attr.private;
	struct device *dev = cd->dev;

	/* Take mutex and remove file. */
	sysfs_remove_bin_file(&dev->kobj, &cdd->bin_attr);
	/* Remove from list and free coredump */
	spin_lock(&cd->dump_lock);
	qm35_coredump_free_data(cdd);
	cd->dump_cnt--;
	spin_unlock(&cd->dump_lock);
}

/**
 * qm35_coredumpx_read() - Read coredumpX buffer.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @count: Buffer size.
 * @pos: Offset pointer.
 *
 * Use the memory_read_from_buffer() function to implement the read logic.
 * Since file is created after coredump was fully received, no blocking mode
 * management is required.
 *
 * Since inode size is known, the caller function, sysfs_kf_bin_read(), won't
 * call this function when all contents were already read. So the returned
 * value won't be 0.
 *
 * Returns: read size on success, else a negative error code.
 */
static ssize_t qm35_coredumpx_read(struct file *filp, struct kobject *kobp,
				   struct bin_attribute *bin_attr, char *buf,
				   loff_t pos, size_t count)
{
	struct qm35_coredump_data *cdd =
		container_of(bin_attr, struct qm35_coredump_data, bin_attr);
	int ret;
	ret = memory_read_from_buffer(buf, count, &pos, cdd->buffer,
				      cdd->offset);
	if (pos == cdd->offset)
		/* This call has completed the read. Start async deletion work. */
		schedule_work(&cdd->rfw);
	return ret;
}

/**
 * qm35_coredump_read() - Read available coredump list.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: Pointer to application buffer.
 * @count: Buffer size.
 * @pos: Offset pointer.
 *
 * Return list of available coredumpX files to read. Application must close
 * then re-open the coredump file after it handle the first results because
 * list has changed.
 *
 * If FIRST read() after open() returns 0, then application may use poll()
 * to wait for a new coredump to be available.
 *
 * Returns: read size on success, else a negative error code.
 */
static ssize_t qm35_coredump_read(struct file *filp, struct kobject *kobp,
				  struct bin_attribute *bin_attr, char *buf,
				  loff_t pos, size_t count)
{
	struct qm35_coredump *cd =
		container_of(bin_attr, struct qm35_coredump, bin_attr);
	struct qm35_coredump_data *cdd;
	int ret = 0, len = 0;

	/* File need to be closed after successful read call.
	 * This is because the caller may have read one coredumpX file and
	 * so it has been removed, changing the list. We can't detect this.
	 * This also ensure `cat /path/to/coredump` will exit properly. */
	if (pos)
		return 0;

	spin_lock(&cd->dump_lock);
	list_for_each_entry (cdd, &cd->dump_list, list) {
		ret = snprintf(buf + len, count - len, "%s\n",
			       cdd->bin_attr.attr.name);
		if (ret >= count - len) {
			ret = -ENOSPC;
			break;
		}
		len += ret;
	}
	spin_unlock(&cd->dump_lock);
	return len ?: ret;
}

/**
 * qm35_coredump_write() - Write coredump command.
 * @filp: The struct file instance.
 * @kobp: Device kernel object associated.
 * @bin_attr: Pointer to written binary attribute.
 * @buf: User-space buffer to write.
 * @count: Buffer size.
 * @pos: Offset pointer.
 *
 * Returns: written size on success, else a negative error code.
 */
static ssize_t qm35_coredump_write(struct file *filp, struct kobject *kobp,
				   struct bin_attribute *bin_attr, char *buf,
				   loff_t pos, size_t count)
{
	struct qm35_coredump *cd =
		container_of(bin_attr, struct qm35_coredump, bin_attr);
	struct device *dev = cd->dev;
	u8 force = 0;

	if (kstrtou8(buf, 10, &force))
		return -EFAULT;
	if (force)
		qm35_coredump_force(cd);
	else
		dev_warn(dev, "Write non null value to force coredump\n");
	return count;
}

/**
 * qm35_coredump_register() - Register a coredump extension.
 * @qm35: The associated QM35 device.
 *
 * Allocate required struct qm35_coredump and register the COREDUMP transport
 * callback to receive COREDUMP packets as soon as possible.
 *
 * Context: User-space process (modprobe).
 * Returns: 0 on success, else a negative error code.
 */
static int qm35_coredump_register(struct qm35 *qm35)
{
	struct device *dev = qm35_get_device(qm35);
	struct qm35_coredump *coredump;
	int rc;

	coredump = kzalloc(sizeof(*coredump), GFP_KERNEL);
	if (!coredump) {
		dev_err(dev, "Cannot allocate coredump receiver memory!\n");
		return -ENOMEM;
	}

	/* Setup instance and add to instance list. */
	coredump->qm35 = qm35;
	coredump->dev = dev;
	spin_lock_init(&coredump->dump_lock);
	INIT_LIST_HEAD(&coredump->dump_list);
	INIT_DELAYED_WORK(&coredump->dwork, qm35_coredump_check);
	ratelimit_state_init(&coredump->dump_rls, COREDUMPS_RATELIMIT_INTERVAL,
			     qm35_coredumps_burst);
	/* Avoid generic message in __ratelimit(). */
	ratelimit_set_flags(&coredump->dump_rls, RATELIMIT_MSG_ON_RELEASE);

	/* Override default handler by our handler. */
	rc = qm35_transport_register(qm35, QM35_TRANSPORT_MSG_COREDUMP,
				     QM35_TRANSPORT_PRIO_NORMAL,
				     qm35_coredump_event_cb, coredump);
	if (rc < 0)
		goto error_free;

	/* Init ok, add to list. */
	list_add_tail(&coredump->dev_list, &coredumps_list);
	return 0;

error_free:
	kfree(coredump);
	return rc;
}

/**
 * qm35_coredump_online() - Finalize setup for a COREDUMP instance.
 * @qm35: The associated QM35 device.
 *
 * After probing of @qm35 instance succeed, ensure the required debugfs
 * file is created to allow application retrieve coredumps.
 *
 * Context: User-space process (modprobe).
 * Returns: 0 on success, else a negative error code.
 */
static int qm35_coredump_online(struct qm35 *qm35)
{
	struct qm35_coredump *coredump = qm35_coredump_search(qm35);
	struct device *dev;
	int rc;

	if (!coredump)
		return -EINVAL;
	dev = coredump->dev;

	/* Create control file. */
	sysfs_bin_attr_init(&coredump->bin_attr);
	coredump->bin_attr.attr.name = "coredump";
	coredump->bin_attr.attr.mode = 0600; /* RW */
	coredump->bin_attr.read = qm35_coredump_read;
	coredump->bin_attr.write = qm35_coredump_write;
	rc = sysfs_create_bin_file(&dev->kobj, &coredump->bin_attr);
	if (rc) {
		const char msg[] = "Failed to create coredump file (err %d)\n";
		dev_warn(dev, msg, rc);
	}
	return 0;
}

/**
 * qm35_coredump_deregister() - Deregister a coredump extension.
 * @qm35: The associated QM35 device.
 *
 * Context: User-space process (modprobe).
 */
static void qm35_coredump_deregister(struct qm35 *qm35)
{
	struct qm35_coredump *coredump = qm35_coredump_search(qm35);
	struct qm35_coredump_data *cdd, *n;
	struct device *dev;

	if (!coredump)
		return;
	dev = coredump->dev;

	/* Remove all handler. We don't want capture any coredump packet anymore. */
	qm35_transport_unregister(qm35, QM35_TRANSPORT_MSG_COREDUMP,
				  QM35_TRANSPORT_PRIO_NORMAL,
				  qm35_coredump_event_cb);
	cancel_delayed_work_sync(&coredump->dwork);

retry:
	/* Free un-read received coredump (and remove sysfs file too). */
	spin_lock(&coredump->dump_lock);
	list_for_each_entry_safe (cdd, n, &coredump->dump_list, list) {
		const char msg[] = "Coredump unread data remain "
				   "(%lu bytes, %lu missing).\n";
		if (work_busy(&cdd->rfw)) {
			/* Work is pending or busy. Since work needs to take
			 * dump_lock to finish, we just need to retry later. */
			spin_unlock(&coredump->dump_lock);
			msleep(10);
			goto retry;
		}
		dev_warn(dev, msg, cdd->offset, cdd->remain);
		if (!cdd->remain)
			sysfs_remove_bin_file(&dev->kobj, &cdd->bin_attr);
		qm35_coredump_free_data(cdd);
		coredump->dump_cnt--;
	}
	spin_unlock(&coredump->dump_lock);
	/* Avoid generic message in ratelimit_state_exit(). */
	ratelimit_set_flags(&coredump->dump_rls, 0);
	ratelimit_state_exit(&coredump->dump_rls);

	/* Remove sysfs control file. */
	if (coredump->bin_attr.attr.name)
		sysfs_remove_bin_file(&dev->kobj, &coredump->bin_attr);

	/* Remove from list and free. */
	list_del(&coredump->dev_list);
	kfree(coredump);
}

/**
 * qm35_coredump_notifier() - The struct notifier_block callback function.
 * @nb: The notifier_block.
 * @action: The QM35 notifier event.
 * @data: The QM35 instance provided by the notifier.
 *
 * This callback function is called when a new QM35 device is registered or
 * unregistered in the system.
 *
 * Context: User-space process (modprobe).
 * Returns: NOTIFY_OK on success, else notifier_from_errno(-EINVAL) if @data
 *          is NULL or if the @action is unknown.
 */
static int qm35_coredump_notifier(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	enum qm35_notifier_events event = action;
	struct qm35 *qm35 = data;
	int rc = 0;

	if (!qm35)
		return notifier_from_errno(-EINVAL);

	switch (event) {
	case QM35_NOTIFIER_EVENT_NEW:
		/* Ignore error, don't want to stop notifier chain. */
		qm35_coredump_register(qm35);
		break;
	case QM35_NOTIFIER_EVENT_ONLINE:
		/* Ignore error, don't want to stop notifier chain. */
		qm35_coredump_online(qm35);
		break;
	case QM35_NOTIFIER_EVENT_DELETE:
		qm35_coredump_deregister(qm35);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return notifier_from_errno(rc);
}

static struct notifier_block nb = {
	.notifier_call = qm35_coredump_notifier,
	.next = NULL,
	.priority = 0,
};

/**
 * qm35_coredump_init() - Module init function.
 *
 * Context: User context.
 * Returns: 0 on success, else qm35_register_notifier() error code.
 */
static int qm35_coredump_init(void)
{
	return qm35_register_notifier(&nb);
}

/**
 * qm35_coredump_exit() - Module exit function.
 *
 * Context: User context.
 * Returns: 0 on success, else qm35_unregister_notifier() error code.
 */
static int qm35_coredump_exit(void)
{
	struct qm35_coredump *cur, *n;

	list_for_each_entry_safe (cur, n, &coredumps_list, dev_list) {
		qm35_coredump_deregister(cur->qm35);
	}
	return qm35_unregister_notifier(&nb);
}

#ifndef QM35_COREDUMP_TESTS

static int __init qm35_coredump_module_init(void)
{
	return qm35_coredump_init();
}

static void __exit qm35_coredump_module_exit(void)
{
	qm35_coredump_exit();
}

module_init(qm35_coredump_module_init);
module_exit(qm35_coredump_module_exit);

#ifdef GITVERSION
MODULE_VERSION(GITVERSION);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Girault <david.girault@qorvo.com>");
MODULE_DESCRIPTION("Qorvo QM35 Coredump extractor");

#endif /* !QM35_COREDUMP_TESTS */
