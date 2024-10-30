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
#ifndef __QM35_UCI_DEV_IOCTL_H
#define __QM35_UCI_DEV_IOCTL_H

#define QM35_UCI_DEV_IOC_TYPE 'U'

/**
 * DOC: UCI char device IOCTLs
 *
 * The UCI char device supports the following IOCTLs.
 *
 * * %QM35_CTRL_RESET (1): Issue a device reset and return device state.
 * * %QM35_CTRL_RESET_EXT (1): Issue a device reset.
 *
 *   This IOCTL allows resetting the chip to bootrom command mode by setting the
 *   parameter to 1.
 * * %QM35_CTRL_GET_STATE (2): Retrieve current state of device.
 *
 *   This IOCTL is deprecated and is here to maintain compatibility with other
 *   UCI char dev drivers.
 * * %QM35_CTRL_FW_UPLOAD (3): Run FW update process and return device state.
 * * %QM35_CTRL_FW_UPLOAD_EXT (3): Run FW update process. The name of the
 *   firmware file to be loaded can optionally be passed as a parameter.
 *
 *   Both %QM35_CTRL_FW_UPLOAD and %QM35_CTRL_FW_UPLOAD_EXT are synchronous.
 *
 *   In case of success of %QM35_CTRL_FW_UPLOAD and %QM35_CTRL_FW_UPLOAD_EXT,
 *   the return value of the ioctl() call will be the value returned by
 *   qm35_transport_fw_update().
 * * %QM35_CTRL_POWER (4): Manual power management of the device.
 *
 *   This IOCTL is deprecated and is here to maintain compatibility with other
 *   UCI char dev driver. This driver will always do power-management itself when
 *   the UCI char is opened or closed. This IOCTL may be used to force power-down
 *   the device while keeping the UCI device opened but it's better to close the
 *   device correctly as it allows to unload the transport driver (which is not
 *   possible when UCI char device is kept opened).
 * * %QM35_CTRL_SET_STATE (5): Change the current device state value.
 *
 *   This IOCTL allows an app to change the current state, but this will be only
 *   until another event comes since the state is reset to ready each time an
 *   event is received.
 * * %QM35_CTRL_GET_TYPE (6): Retrieve current type of message exchanged.
 *
 *   By default, when opened, the type of message received and sent is set to
 *   UCI. This IOCTL allows applications to retrieve the current type.
 * * %QM35_CTRL_SET_TYPE (7): Set type of message exchanged.
 *
 *   This IOCTL allows applications to change the type of messages sent to  or
 *   received from the device, allowing them to manually flash, retrieve logs or
 *   change logs configuration.
 * * %QM35_CTRL_WAIT_EVENT (8): Wait for an event without consuming CPU when
 *   file is open in NON-BLOCKING mode. You may use the poll() API instead.
 */

#define QM35_FIRMWARE_FILENAME_SIZE 64

/**
 * enum qm35_uci_dev_states - States of the UCI char device.
 * @QM35_UCI_DEV_CTRL_STATE_UNKNOWN: Unknown state.
 * @QM35_UCI_DEV_CTRL_STATE_OFF: Power off state.
 * @QM35_UCI_DEV_CTRL_STATE_RESET: Reset state.
 * @QM35_UCI_DEV_CTRL_STATE_COREDUMP: Core dump state.
 * @QM35_UCI_DEV_CTRL_STATE_READY: Ready state.
 * @QM35_UCI_DEV_CTRL_STATE_FW_DOWNLOADING: Firmware downloading state.
 * @QM35_UCI_DEV_CTRL_STATE_UCI_APP: UCI application state.
 */
enum qm35_uci_dev_states {
	QM35_UCI_DEV_CTRL_STATE_UNKNOWN = 0x0000,
	QM35_UCI_DEV_CTRL_STATE_OFF = 0x0001,
	QM35_UCI_DEV_CTRL_STATE_RESET = 0x0002,
	QM35_UCI_DEV_CTRL_STATE_COREDUMP = 0x0004,
	QM35_UCI_DEV_CTRL_STATE_READY = 0x0008,
	QM35_UCI_DEV_CTRL_STATE_FW_DOWNLOADING = 0x0010,
	QM35_UCI_DEV_CTRL_STATE_UCI_APP = 0x0020,
};

/**
 * struct qm35_fwupload_params - Parameters for FW_UPLOAD_EXT ioctl.
 * @fw_name: Name of the firmware file to be loaded, replacing the default one.
 */
struct qm35_fwupload_params {
	char fw_name[QM35_FIRMWARE_FILENAME_SIZE];
};

#define QM35_CTRL_RESET _IOR(QM35_UCI_DEV_IOC_TYPE, 1, unsigned int)
#define QM35_CTRL_RESET_EXT _IOW(QM35_UCI_DEV_IOC_TYPE, 1, unsigned int)
#define QM35_CTRL_GET_STATE _IOR(QM35_UCI_DEV_IOC_TYPE, 2, unsigned int)
#define QM35_CTRL_FW_UPLOAD _IOR(QM35_UCI_DEV_IOC_TYPE, 3, unsigned int)
#define QM35_CTRL_FW_UPLOAD_EXT \
	_IOW(QM35_UCI_DEV_IOC_TYPE, 3, struct qm35_fwupload_params)
#define QM35_CTRL_POWER _IOW(QM35_UCI_DEV_IOC_TYPE, 4, unsigned int)
#define QM35_CTRL_SET_STATE _IOW(QM35_UCI_DEV_IOC_TYPE, 5, unsigned int)
#define QM35_CTRL_GET_TYPE _IOR(QM35_UCI_DEV_IOC_TYPE, 6, unsigned int)
#define QM35_CTRL_SET_TYPE _IOW(QM35_UCI_DEV_IOC_TYPE, 7, unsigned int)
#define QM35_CTRL_WAIT_EVENT _IOR(QM35_UCI_DEV_IOC_TYPE, 8, unsigned int)

#endif /* __QM35_UCI_DEV_IOCTL_H */
