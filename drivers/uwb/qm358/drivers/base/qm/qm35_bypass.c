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
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "qm35.h"
#include "qm35_ids.h"
#include "qm35_bypass.h"
#include "qm35_transport.h"
#include "qm35_trc.h"

/**
 * qm35_bypass_event_cb() - Receive event transport handler callback.
 * @data: Pointer to bypass structure given at registration time.
 * @skb: Received packet to handle.
 *
 * When bypass is opened, this handler is registered to receive the expected
 * packet type. This remove the need to check if packet need to be handled
 * or not by the bypass. It will.
 */
void qm35_bypass_event_cb(void *data, struct sk_buff *skb)
{
	qm35_bypass_handle hnd = (qm35_bypass_handle)data;
	enum qm35_bypass_events bypass_event = QM35_BYPASS_IRQ;

	/* Sanity check received packet type. */
	if (skb->cb[0] != hnd->expected_type) {
		kfree_skb(skb);
		return;
	}

	/* Add packet to the list of received packets. */
	spin_lock(&hnd->lock);
	list_add_tail(&skb->list, &hnd->packets);
	spin_unlock(&hnd->lock);

	/* Call registered listener.
	 * Will in turn call qm35_bypass_recv() which will
	 * return this stored packet and free it. */
	hnd->listener(hnd->listener_data, bypass_event);
}

/**
 * qm35_bypass_set_expected() - Associated bypass channel to packet type.
 * @hnd: The bypass channel handle to configure.
 * @expected: The new expected packet type to handle.
 *
 * This function is called by qm35_bypass_open() or qm35_bypass_control() by
 * the client (``/dev/uciX``).
 *
 * It install the qm35_bypass_event_cb() transport handler for the specified
 * packet type. This may fail if this expected message type is already
 * registered by another bypass channel.
 *
 * If registration of packet type handler has failed, the bypass channel is
 * not bound to any message type.
 *
 * Context: User context.
 * Return: Zero on success else a negative error.
 */
static int qm35_bypass_set_expected(qm35_bypass_handle hnd,
				    enum qm35_transport_msg_type expected)
{
	struct qm35 *qm35 = container_of(hnd->bypass, struct qm35, bypass_data);
	enum qm35_transport_msg_type cur;
	int rc;

	if (expected == hnd->expected_type)
		return 0;
	if (qm35_bypass_bound(hnd, &cur)) {
		/* Remove previous type handler (cannot fail). */
		qm35_transport_unregister(qm35, cur, QM35_TRANSPORT_PRIO_HIGH,
					  qm35_bypass_event_cb);
		hnd->expected_type = QM35_TRANSPORT_MSG_MAX;
	}
	/* Install handler for current expected type. May fail if this expected
	 * message type is already registered by another bypass channel. */
	rc = qm35_transport_register(qm35, expected, QM35_TRANSPORT_PRIO_HIGH,
				     qm35_bypass_event_cb, hnd);
	if (!rc)
		/* Save new expected type. */
		hnd->expected_type = expected;
	return rc;
}

/**
 * qm35_bypass_cleanup() - Cleanup a bypass channel.
 * @hnd: The bypass channel handle to configure.
 *
 * This function free any resources associated to this bypass channel. This
 * includes all unread packets stored in the list and the given @hnd.
 *
 * This function also ensures the qm35_bypass_event_cb() transport handler for
 * the expected packet type is unregistered correctly.
 *
 * Return: Number of freed unread skb or zero if none.
 */
static int qm35_bypass_cleanup(qm35_bypass_handle hnd)
{
	struct qm35 *qm35 = container_of(hnd->bypass, struct qm35, bypass_data);
	enum qm35_transport_msg_type cur;
	struct sk_buff *p, *n;
	int rc = 0;

	/* Remove previous registered handler. */
	if (qm35_bypass_bound(hnd, &cur)) {
		qm35_transport_unregister(qm35, cur, QM35_TRANSPORT_PRIO_HIGH,
					  qm35_bypass_event_cb);
	}
	/* Free all remaining packet in list. */
	list_for_each_entry_safe (p, n, &hnd->packets, list) {
		list_del(&p->list);
		kfree_skb(p);
		rc++;
	}
	/* Reset fields. */
	module_put(THIS_MODULE);
	kfree(hnd);
	return rc;
}

/**
 * qm35_bypass_open() - Open a bypass channel.
 * @qm35: The QM35 instance on which the new bypass channel connects.
 * @cb: Callback function called when an event occurs on the device.
 * @priv_data: Private data transmitted when @cb is called.
 *
 * This function is called when the ``/dev/uciX`` special file is opened.
 * It allows the QM35 UCI char dev driver to register its callback function
 * associated with its private data. This callback function is called when
 * an event occurs from the device and needs to be forwarded to the
 * ``/dev/uciX`` special file while bypass is opened.
 *
 * While the bypass channel is opened and configured for a specific type of
 * messages, all communication from other auxiliary modules which use the same
 * messages type are disabled.
 *
 * The reference count of the module is incremented with ``try_module_get()``.
 *
 * It also calls the ``qm35_transport_start()`` to allow the transport module
 * to power-on the device if not yet started by other core sub-module.
 *
 * Context: User context.
 * Return: The bypass channel handle on success, else ERR_PTR(-EINVAL) if
 *  @qm35 or @cb are NULL, ERR_PTR(-EBUSY) if the bypass channel is already
 *  opened on this @qm35 instance.
 */
qm35_bypass_handle qm35_bypass_open(struct qm35 *qm35,
				    qm35_bypass_listener_cb cb, void *priv_data)
{
	struct qm35_bypass *bypass = &qm35->bypass_data;
	qm35_bypass_handle hnd;
	int rc = -EINVAL;

	trace_qm35_bypass_open(qm35, cb, priv_data);
	if (!qm35 || !cb) {
		goto error;
	}

	hnd = kmalloc(sizeof(struct qm35_bypass_channel), GFP_KERNEL);
	if (!hnd) {
		rc = -ENOMEM;
		goto error;
	}

	/* Setup new bypass channel handle. */
	hnd->bypass = bypass;
	hnd->listener = cb;
	hnd->listener_data = priv_data;
	spin_lock_init(&hnd->lock);
	INIT_LIST_HEAD(&hnd->packets);
	hnd->expected_type = QM35_TRANSPORT_MSG_MAX;

	spin_lock(&bypass->lock);
	list_add_tail(&hnd->list, &bypass->channels);
	spin_unlock(&bypass->lock);

	if (!try_module_get(THIS_MODULE)) {
		rc = -ENOENT;
		goto error_free;
	}

	if (atomic_inc_return(&bypass->opened) == 1) {
		/* First opening. Ensure the QM35 chip is started */
		rc = qm35_transport_start(qm35);
		if (rc < 0)
			goto error_put;
	}

	trace_qm35_bypass_open_return(qm35, hnd);
	return hnd;

error_put:
	module_put(THIS_MODULE);
	atomic_dec(&bypass->opened);
error_free:
	spin_lock(&bypass->lock);
	list_del(&hnd->list);
	spin_unlock(&bypass->lock);
	kfree(hnd);
error:
	trace_qm35_bypass_open_return(qm35, ERR_PTR(rc));
	return ERR_PTR(rc);
}
EXPORT_SYMBOL(qm35_bypass_open);

/**
 * qm35_bypass_queue_check() - Returns the status of the bypass packet queue.
 * @hnd: The bypass channel handle.
 *
 * This function returns the status of the bypass packet queue of the passed
 * the bypass handle.
 *
 * Context: User context or kernel thread context.
 * Return: True if the packet list is not empty, false if it is empty. Else
 *  -EINVAL if @hnd is NULL.
 */
int qm35_bypass_queue_check(qm35_bypass_handle hnd)
{
	int rc;

	if (IS_ERR_OR_NULL(hnd))
		return -EINVAL;

	/* Check the bypass packet list to ensure there is something to read. */
	spin_lock(&hnd->lock);
	rc = !list_empty(&hnd->packets);
	spin_unlock(&hnd->lock);

	return rc;
}
EXPORT_SYMBOL(qm35_bypass_queue_check);

/**
 * qm35_bypass_close() - Close a bypass channel.
 * @hnd: The bypass channel handle.
 *
 * This function is called when the ``/dev/uciX`` special file is closed.
 *
 * It resets ``qm35->bypass_data.opened`` to ``false`` and decrease the module
 * reference count by calling ``module_put()``.
 *
 * It will also call ``qm35_transport_stop()`` to allow the transport module to
 * power-down the device if no other active user remains.
 *
 * Context: User context.
 * Return: Number of unread packets freed on success, else -EINVAL if @hnd is
 *   invalid.
 */
int qm35_bypass_close(qm35_bypass_handle hnd)
{
	struct qm35_bypass *bypass;
	struct qm35 *qm35 = NULL;
	int rc = -EINVAL;

	trace_qm35_bypass_close(hnd);
	if (IS_ERR_OR_NULL(hnd))
		goto error;
	bypass = hnd->bypass;
	qm35 = container_of(bypass, struct qm35, bypass_data);

	if (atomic_dec_and_test(&bypass->opened)) {
		/* Last close. No more channel opened. Ensure the QM35 chip is
		 * stopped. */
		qm35_transport_stop(qm35);
	}

	/* Remove from list. */
	spin_lock(&bypass->lock);
	list_del(&hnd->list);
	spin_unlock(&bypass->lock);

	/* Cleanup the bypass channel. */
	rc = qm35_bypass_cleanup(hnd);
	if (rc)
		dev_warn(qm35->dev,
			 "Bypass channel closed while %d packet(s) remain in "
			 "queue!\n",
			 rc);

error:
	trace_qm35_bypass_close_return(qm35, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_bypass_close);

/**
 * qm35_bypass_control() - Control a bypass channel.
 * @hnd: The bypass channel handle.
 * @action: The action to perform on the QM35 instance.
 * @param: The action parameter if any.
 *
 * This function allow external module to interract with the core or the
 * transport modules.
 *
 * It currently support the following operations:
 *
 * 1. Reset the device.
 * 2. Launch FW update process.
 * 3. Configure the expected message type.
 * 4. Manually power-off the device.
 *
 * More actions may be added depending of needs. See ``enum qm35_bypass_actions``.
 *
 * This function is called when the /dev/uciX special file IOCTL api is used.
 *
 * Context: User context.
 * Return: Zero or positive value on success else a negative error:
 *   * -EINVAL if @hnd or @param (when required by action) are invalid,
 *   * -EOPNOTSUPP if unknown command.
 */
int qm35_bypass_control(qm35_bypass_handle hnd, enum qm35_bypass_actions action,
			long *param)
{
	struct qm35_bypass *bypass;
	struct qm35 *qm35;
	enum qm35_transport_msg_type cur, new;
	int rc = -EINVAL;

	trace_qm35_bypass_control(hnd, action, param);
	if (IS_ERR_OR_NULL(hnd))
		goto error;
	bypass = hnd->bypass;
	qm35 = container_of(bypass, struct qm35, bypass_data);

	/* Now perform the requested action (outside critical section) */
	switch (action) {
	case QM35_BYPASS_ACTION_RESET:
		if (IS_ERR_OR_NULL(param))
			break;
		qm35_bypass_set_expected(hnd, QM35_TRANSPORT_MSG_UCI);
		rc = qm35_transport_reset(qm35, *param);
		break;
	case QM35_BYPASS_ACTION_MSG_TYPE:
		if (!qm35_bypass_bound(hnd, &cur))
			cur = QM35_TRANSPORT_MSG_MAX;
		if (param) {
			new = *param;
			rc = qm35_bypass_set_expected(hnd, new);
			if (!rc)
				rc = cur;
		} else {
			rc = cur;
		}
		break;
	case QM35_BYPASS_ACTION_FWUPD:
		rc = qm35_transport_fw_update(qm35, NULL, 0, (char *)param);
		/* Reprobe device in case of success with firmware flashed. */
		if (rc == 2) {
			char infobuf[64];
			/* Temporarily unregister qm35_bypass_event_cb() so
			 * qm35_uci_probe_handle() can be set up instead. */
			qm35_transport_unregister(qm35, hnd->expected_type,
						  QM35_TRANSPORT_PRIO_HIGH,
						  qm35_bypass_event_cb);
			rc = qm35_transport_probe(qm35, infobuf, sizeof(infobuf)) ?:
				     rc;
			qm35_transport_register(qm35, hnd->expected_type,
						QM35_TRANSPORT_PRIO_HIGH,
						qm35_bypass_event_cb, hnd);
		}
		break;
	case QM35_BYPASS_ACTION_POWER:
		if (IS_ERR_OR_NULL(param))
			break;
		rc = qm35_transport_power(qm35, *param);
		break;
	default:
		dev_warn(qm35->dev, "Unsupported action %d\n", action);
		rc = -EOPNOTSUPP;
	}
error:
	trace_qm35_bypass_control_return(hnd, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_bypass_control);

/**
 * qm35_bypass_send() - Send data to transport.
 * @hnd: The bypass channel handle.
 * @buffer: data buffer to be send
 * @len: len of @buffer
 *
 * This function allows modules using the bypass to send data to device.
 *
 * This function is called to send data written on ``/dev/uciX``. It forwards
 * the data to a low level transports modules using the ``send()`` transport
 * callback function.
 *
 * This function is called when data written on /dev/uciX should be forwarded
 * to a low level transports modules.
 *
 * Context: User context.
 * Return: Send transport callback function result else a negative error:
 *   * -EINVAL if any parameter is not valid,
 *   * -EBADTYPE if the bypass channel isn't bound yet to any packet type.
 */
int qm35_bypass_send(qm35_bypass_handle hnd, void *buffer, size_t len)
{
	struct qm35_bypass *bypass;
	struct qm35 *qm35;
	int rc = -EINVAL;

	trace_qm35_bypass_send(hnd);
	if (IS_ERR_OR_NULL(hnd) || !buffer || !len)
		goto error;
	if (!qm35_bypass_bound(hnd, NULL)) {
		rc = -EBADTYPE;
		goto error;
	}

	bypass = hnd->bypass;
	qm35 = container_of(bypass, struct qm35, bypass_data);

	/* Checks passed, now call send transport callback. */
	rc = qm35_transport_send_direct(qm35, hnd->expected_type, buffer, len);
error:
	trace_qm35_bypass_send_return(hnd, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_bypass_send);

/**
 * qm35_bypass_recv() - Receive data from queue.
 * @hnd: The bypass channel handle.
 * @buffer: Data buffer to store message from user-space.
 * @len: Size of @buffer.
 * @type: Received message type.
 * @flags: Received message flags.
 *
 * This function allows modules using the bypass to get received data from the
 * device.
 *
 * This function is called when data received from a low level transport module
 * should be forwarded to ``/dev/uciX``. It returns the already received packet,
 * saved by qm35_bypass_event_cb() before the registered listener function is called.
 *
 * Context: User context.
 * Return: Received message length on success else a negative error:
 *   * -EAGAIN if no packet is available,
 *   * -EINVAL if any parameter is not valid,
 *   * -EFAULT if cannot copy packet to provided user buffer,
 *   * -EBADTYPE if the bypass channel isn't bound yet to any packet type.
 */
int qm35_bypass_recv(qm35_bypass_handle hnd, void __user *buffer, size_t len,
		     enum qm35_transport_msg_type *type, int *flags)
{
	struct sk_buff *skb;
	int rc = -EINVAL;

	trace_qm35_bypass_recv(hnd);
	if (IS_ERR_OR_NULL(hnd) || !type || !flags || !buffer || !len)
		goto error;
	if (!qm35_bypass_bound(hnd, NULL)) {
		rc = -EBADTYPE;
		goto error;
	}

	/* Take next packet in the list. */
	spin_lock(&hnd->lock);
	skb = list_first_entry_or_null(&hnd->packets, struct sk_buff, list);
	if (skb) {
		/* Check length of received packet. */
		if (len > skb->len)
			len = skb->len;
		/* Early remove from the list while locked if needed. */
		if (len == skb->len)
			list_del(&skb->list);
	}
	spin_unlock(&hnd->lock);
	if (!skb) {
		/* Return early if nothing received.
		 * Since this function is called after the listener had been
		 * called, we always have a packet ready in blocking mode.
		 * In non-blocking mode, just return this error if nothing
		 * received and don't call the transport receive callback. */
		rc = -EAGAIN;
		goto error;
	}

	/* Copy received frame to user-space buffer (outside lock section) */
	if (copy_to_user(buffer, skb->data, len)) {
		rc = -EFAULT;
	} else {
		/* Retrieve metadata from skb control block. */
		*type = skb->cb[0];
		*flags = skb->cb[1];
		/* Update skb and free it if needed. */
		if (skb->len == len)
			kfree_skb(skb);
		else
			skb_pull(skb, len);
		/* Returns total bytes copied to buffer. */
		rc = len;
	}
error:
	trace_qm35_bypass_recv_return(hnd, rc);
	return rc;
}
EXPORT_SYMBOL(qm35_bypass_recv);
