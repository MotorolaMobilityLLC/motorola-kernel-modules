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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include "qm35_core.h"
#include "qm35_ids.h"
#include "qm35_notifier.h"
#include "qm35_transport.h"
#include "qm35_trc.h"

#define qm35_notify_new(qm) qm35_notifier_notify(qm, QM35_NOTIFIER_EVENT_NEW)
#define qm35_notify_online(qm) \
	qm35_notifier_notify(qm, QM35_NOTIFIER_EVENT_ONLINE)
#define qm35_notify_delete(qm) \
	qm35_notifier_notify(qm, QM35_NOTIFIER_EVENT_DELETE)

/* Root uwb entry in debugfs for all QM35 instances. */
struct dentry *debugfs_uwb_root = NULL;

/**
 * qm35_state_wait() - Wait the QM35 state's change.
 * @qm35: QM35 device instance.
 * @device_state: Device state to wait for.
 *
 * Wait the QM35 state's change through a wait queue.
 * Waken up by qm35_state_set().
 *
 * Returns: 0 if the timeout elapsed,
 *          negative value if error occurs or
 *          remaining jiffies if state changed before the timeout elapsed
 */
int qm35_state_wait(struct qm35 *qm35, enum qm35_state device_state)
{
	int ret;
	ret = wait_event_interruptible_timeout(qm35->wait_state,
					       qm35->state == device_state, HZ);
	return ret;
}
EXPORT_SYMBOL(qm35_state_wait);

/**
 * qm35_state_set() - Set the QM35 state.
 * @qm35: QM35 device instance.
 * @device_state: The device state.
 *
 * Wakeup a dedicated wait queue, that allows to synchronize
 * some operations with the state's change.
 *
 * Context: Called from UCI notification.
 */
void qm35_state_set(struct qm35 *qm35, enum qm35_state device_state)
{
	trace_qm35_state(qm35, qm35->state, device_state);
	qm35->state = device_state;

	wake_up(&qm35->wait_state);
}
EXPORT_SYMBOL(qm35_state_set);

/**
 * qm35_parse_device_info() - Parse UCI_CORE GET_DEVICE_INFO response.
 * @qm35: QM35 device instance.
 * @info: QM35 device info response.
 * @current_ver: QM35 firmware version.
 * @device_id: QM35 device_id.
 *
 * Fill @current_ver and @device_id using @info.
 *
 * Return: Zero on success, else a negative error.
 */
static int qm35_parse_device_info(struct qm35 *qm35,
				  struct qm35_uci_device_info *info,
				  struct qm35_fw_version *current_ver,
				  u16 *device_id)
{
#define uci_version_to_triplet(v) \
	be16_to_cpu(v) >> 8, be16_to_cpu(v) >> 4 & 0xf, be16_to_cpu(v) & 0xf
	dev_info(
		qm35->dev,
		"Found QM35 (uci: %u.%u.%u, mac: %u.%u.%u, phy: %u.%u.%u, uci_test: %u.%u.%u)\n",
		uci_version_to_triplet(info->uci_version),
		uci_version_to_triplet(info->mac_version),
		uci_version_to_triplet(info->phy_version),
		uci_version_to_triplet(info->uci_test_version));
#undef uci_version_to_triplet

	/* Handle well-known length from QMFW (52 bytes) and Polaris FW (12 bytes). */
	if (info->vendor_length >= 52) {
		current_ver->major = info->vendor_data[0];
		current_ver->minor = info->vendor_data[1];
		current_ver->patch = info->vendor_data[2];
		current_ver->rc = info->vendor_data[3];
		current_ver->build_id =
			get_unaligned_le64(&info->vendor_data[4]);
		current_ver->oem_major = info->vendor_data[12];
		current_ver->oem_minor = info->vendor_data[13];
		current_ver->oem_patch = info->vendor_data[14];

		*device_id = get_unaligned_le16(&info->vendor_data[47]);
	} else if (info->vendor_length >= 12) {
		memset(current_ver, 0, sizeof(*current_ver));
		current_ver->major = info->vendor_data[10];
		current_ver->minor = info->vendor_data[9];
		current_ver->patch = info->vendor_data[8];

		*device_id = get_unaligned_le16(info->vendor_data);
	} else {
		dev_warn(qm35->dev, "No firmware version found.\n");
		return -EINVAL;
	}
	qm35_fw_version_print(dev_info, qm35->dev, "Running firmware version",
			      current_ver);
	dev_info(qm35->dev, "Device id: 0x%04x\n", *device_id);
	return 0;
}

/**
 * qm35_alloc_device() - allocate a QM35 device
 * @dev: the corresponding generic device given by transport module
 * @priv_size: the total size of private data to allocate
 * @transport: the transport description
 *
 * Allocate a new QM35 device and initialise required fields in the returned
 * structure. It is assumed this structure is the first field in the holding
 * transport specific structure and so the priv_size field MUST be more than
 * sizeof(struct qm35).
 *
 * All parameters are validated and this function returns NULL in case of
 * error with one of them.
 *
 * Context: Called from bus device probing or module init functions.
 * Returns: the address of allocated private data or PTR_ERR() value.
 */
struct qm35 *qm35_alloc_device(struct device *dev, size_t priv_size,
			       const struct qm35_transport *transport)
{
	struct qm35 *qm35;
	int rc = -EINVAL;

	trace_qm35_alloc_device(priv_size);

	if (!dev || !transport || !transport->name || !transport->ops)
		goto error;
	if (!transport->ops->send || !transport->ops->recv)
		goto error;
	if (priv_size < sizeof(struct qm35))
		goto error;

	/* First, allocate a new HW with required private data for us. */
	rc = -ENOMEM;
	priv_size = ALIGN(priv_size, SMP_CACHE_BYTES);
	qm35 = kzalloc(priv_size, GFP_KERNEL);
	if (!qm35)
		goto error;
	qm35->dev = dev;
	qm35->transport_ops = transport->ops;
	qm35->transport_flags = transport->flags;
	qm35->state = QM35_STATE_UNKNOWN;

	/* Allocate a new device ID and store pointer on it. */
	rc = qm35_new_id(qm35);
	if (rc < 0)
		goto error_free;

	/* TODO: More field initialisation
	 * - mutexes,
	 * - spinlock,
	 * - list nodes... */
	spin_lock_init(&qm35->lock);
	spin_lock_init(&qm35->bypass_data.lock);
	INIT_LIST_HEAD(&qm35->bypass_data.channels);
	init_waitqueue_head(&qm35->wait_state);
	dev_info(qm35->dev,
		 "Created new QM35 instance (hw#%d, transport: %s, with flags: "
		 "%u)\n",
		 qm35->dev_id, transport->name, qm35->transport_flags);
	trace_qm35_alloc_device_return(qm35);
	return qm35;

error_free:
	kfree(qm35);
error:
	qm35 = ERR_PTR(rc);
	trace_qm35_alloc_device_return(qm35);
	return qm35;
}
EXPORT_SYMBOL(qm35_alloc_device);

/**
 * qm35_free_device() - free a QM35 device
 * @qm35: the QM35 device private structure to free
 *
 * Free a QM35 device. The device MUST be unregistered first.
 *
 * Context: Called from bus device probing or module init/exit functions.
 */
void qm35_free_device(struct qm35 *qm35)
{
	if (IS_ERR_OR_NULL(qm35))
		return;

	trace_qm35_free_device(qm35);

	/* Remove device ID. */
	qm35_remove_id(qm35->dev_id);
	/* Finally free */
	kfree(qm35);
}
EXPORT_SYMBOL(qm35_free_device);

/**
 * qm35_register_device() - register a QM35 device
 * @qm35: the QM35 device private structure to register
 *
 * Register a QM35 device.
 *
 * This function initialises the required sub-systems and probes for a QM35 device
 * using the transport functions provided by the caller.
 *
 * It is assumed by this function that the provided transport callbacks are
 * ready to be used, so this function can probe the QM35 device and start the
 * firmware upgrade process if required.
 *
 * This function also notify auxiliary modules that a new QM35 device exist and it
 * is online, allowing them to allocate and link their data structure to this QM35
 * device.
 *
 * Context: Called from bus device probing or module init/exit functions.
 * Return: Zero on success, else a negative error.
 */
int qm35_register_device(struct qm35 *qm35)
{
	struct qm35_uci_device_info *info;
	char infobuf[64];
	struct qm35_fw_version current_ver;
	u16 device_id = 0;
	bool version_ok = false;
	bool fw_update_tried = false;
	int rc = 0;

	trace_qm35_register_device(qm35);
	if (IS_ERR_OR_NULL(qm35)) {
		rc = -EINVAL;
		goto notify_new_error;
	}
	if (qm35->transport_flags & QM35_TRANSPORT_ASYNC_PROBING) {
		/* TODO: Implement asynchronous probing. */
		rc = -ENOSYS;
		/* TODO: if asynchronous probing is asked, shouldn't
		 * we also defer firmware update? */
		goto notify_new_error;
	}

	/* Notify auxiliary modules that a new QM35 is available. */
	qm35_notify_new(qm35);

	/* Ensure QM35 is powered up. */
	rc = qm35_transport_start(qm35);
	if (rc)
		goto start_error;

	/* Force a HARD reset using configured GPIO before probing. */
	rc = qm35_transport_reset(qm35, false);
	/* If probing is disabled by the transport driver, we cannot probe.
	 * No need to check return value of reset. We won't try to update
	 * firmware either. */
	if (qm35->transport_flags & QM35_TRANSPORT_NO_PROBING)
		goto register_hw;

	do {
		/* If flashing has failed, redo HARD reset. */
		if (fw_update_tried)
			rc = qm35_transport_reset(qm35, false);
		/* Check reset status and force the firmware upgrade if reset()
		 * transport callback returns an error. */
		if (rc)
			goto fw_update;

		/* Call transport probe function. */
		rc = qm35_transport_probe(qm35, infobuf, sizeof(infobuf));
		if (rc < 0)
			/* Force the firmware upgrade on the first device info error,
			 * or exit with error. */
			goto fw_update;
		info = (struct qm35_uci_device_info *)infobuf;
		version_ok = !qm35_parse_device_info(qm35, info, &current_ver,
						     &device_id);

	fw_update:
		if (fw_update_tried && rc) {
			/* Exit with error on second error. */
			goto probe_error;
		}
		if (!fw_update_tried) {
			/* Start FW download process on the first iteration. */
			rc = qm35_transport_fw_update(
				qm35, version_ok ? &current_ver : NULL,
				device_id, NULL);
			/* If qm35_transport_fw_update() was interrupted by a
			 * signal, exit immediately. */
			if (rc == -EINTR)
				goto probe_error;
			/* If flashing was not attempted, set rc to 0. */
			else if (rc == 1)
				rc = 0;
		}
		fw_update_tried = true;
		/* Retry probing if flashing was attempted. */
	} while (rc);

register_hw:
	qm35->debugfs_root =
		debugfs_create_dir(dev_name(qm35->dev), debugfs_uwb_root);
	if (IS_ERR_OR_NULL(qm35->debugfs_root)) {
		rc = PTR_ERR(qm35->debugfs_root);
		goto debugfs_error;
	}

	/* Notify auxiliary modules that a new QM35 is online. */
	qm35_notify_online(qm35);

	/* Ensure QM35 is powered down.
	 * QM35 will be powered up and down by IEEE802154 or Bypass APIs
	 * on demand. */
	qm35_transport_stop(qm35);

	trace_qm35_register_device_return(qm35, 0);
	return 0;

debugfs_error:
probe_error:
	qm35_transport_stop(qm35);
start_error:
	qm35_notify_delete(qm35);
notify_new_error:
	trace_qm35_register_device_return(qm35, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_register_device);

/**
 * qm35_unregister_device() - unregister a QM35 device
 * @qm35: the QM35 device private structure to unregister
 *
 * Unregister a QM35 device and associated ieee802154 HW.
 *
 * Context: Called from bus device probing or module init/exit functions.
 * Return: Zero on success, else a negative error.
 */
int qm35_unregister_device(struct qm35 *qm35)
{
	int rc = 0;

	trace_qm35_unregister_device(qm35);
	if (IS_ERR_OR_NULL(qm35)) {
		rc = -EINVAL;
		goto error;
	}

	/* Notify listener that a QM35 is deleted. */
	qm35_notify_delete(qm35);

	/* Remove per-device debugfs root directory. */
	debugfs_remove(qm35->debugfs_root);
	qm35->debugfs_root = NULL;

error:
	trace_qm35_unregister_device_return(qm35, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_unregister_device);

/**
 * qm35_get_dev_id() - Return a QM35 device ID.
 * @qm35: The QM35 instance.
 *
 * This function is called when the /dev/uciX special file is created.
 *
 * Context: User context.
 * Return: The dev_id of @qm35 on success, else -EINVAL if @qm35 is NULL.
 */
int qm35_get_dev_id(struct qm35 *qm35)
{
	if (!qm35)
		return -EINVAL;
	return qm35->dev_id;
}
EXPORT_SYMBOL(qm35_get_dev_id);

/**
 * qm35_get_device() - Return a underlying struct device of QM35 instance.
 * @qm35: The QM35 instance.
 *
 * This helper function can be used by QM35 extension to retrieve underlying
 * struct device.
 *
 * Context: User context.
 * Return: Pointer to struct device, else -EINVAL if @qm35 is NULL.
 */
struct device *qm35_get_device(struct qm35 *qm35)
{
	if (!qm35)
		return NULL;
	return qm35->dev;
}
EXPORT_SYMBOL(qm35_get_device);

/**
 * qm35_get_debug_root() - Return the device specific directory in debugfs.
 * @qm35: The QM35 instance.
 *
 * This function can be called by any extension modules who need to add files
 * into debugfs to retrieve root directory entry where to put them.
 *
 * Context: User context (probing phase).
 * Return: The struct dentry pointer, else PTR_ERR(-EINVAL) if
 *         @qm35 is NULL.
 */
struct dentry *qm35_get_debug_root(struct qm35 *qm35)
{
	if (!qm35)
		return ERR_PTR(-EINVAL);
	return qm35->debugfs_root;
}
EXPORT_SYMBOL(qm35_get_debug_root);

#ifndef QM35_CORE_TESTS

static int __init qm35_init(void)
{
	pr_info("%s: init\n", THIS_MODULE->name);
	pr_info("%s: Driver version: %s\n", THIS_MODULE->name, GITVERSION);
	debugfs_uwb_root = debugfs_create_dir("uwb", NULL);
	if (IS_ERR_OR_NULL(debugfs_uwb_root)) {
		pr_warn("%s: cannot create uwb directory entry in debugfs!!!\n",
			THIS_MODULE->name);
	}
	return 0;
}

static void __exit qm35_exit(void)
{
	pr_info("%s: Exit\n", THIS_MODULE->name);
	if (!IS_ERR_OR_NULL(debugfs_uwb_root))
		debugfs_remove(debugfs_uwb_root);
}

module_init(qm35_init);
module_exit(qm35_exit);

#ifdef GITVERSION
MODULE_VERSION(GITVERSION);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Girault <david.girault@qorvo.com>");
MODULE_DESCRIPTION("Qorvo QM35 Core APIs");

#endif /* !QM35_CORE_TESTS */
