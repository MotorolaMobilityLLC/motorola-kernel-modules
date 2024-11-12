/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2024 Qorvo US, Inc.
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
#ifndef __QM35_SPI_TRANSPORT_H
#define __QM35_SPI_TRANSPORT_H

#include "qm35_transport.h"

extern const struct qm35_transport qm35_spi_transport;

struct qm35_spi;
void qm35_spi_transport_setup(struct qm35_spi *qmspi);

#ifdef QM35_SPI_TRANSPORT_TESTS

#include "mocks/ku_base.h"
#include "mocks/ku_module_get_put.h"
#define KU_NO_WAKE_UP_MOCK
#include "mocks/ku_wait_event.h"

static int ku_qm35_spi_pm_start(struct qm35_spi *qmspi);
static int ku_qm35_spi_pm_stop(struct qm35_spi *qmspi);
static int ku_qm35_spi_pm_resume(struct qm35_spi *qmspi);
static int ku_qm35_spi_pm_idle(struct qm35_spi *qmspi);
#define qm35_spi_pm_start ku_qm35_spi_pm_start
#define qm35_spi_pm_stop ku_qm35_spi_pm_stop
#define qm35_spi_pm_resume ku_qm35_spi_pm_resume
#define qm35_spi_pm_idle ku_qm35_spi_pm_idle

struct qm35_hsspi_header;
static int ku_qm35_hsspi_recv(struct qm35_spi *qmspi,
			      struct qm35_hsspi_header *header, void *data,
			      size_t size);
static int ku_qm35_hsspi_send(struct qm35_spi *qmspi, u8 ul_value,
			      const void *data, size_t length);
static int ku_qm35_hsspi_wakeup(struct qm35_spi *qmspi, bool force);
#define qm35_hsspi_recv ku_qm35_hsspi_recv
#define qm35_hsspi_send ku_qm35_hsspi_send
#define qm35_hsspi_wakeup ku_qm35_hsspi_wakeup

static void ku_qm35_uci_probe_setup(struct qm35_spi *qmspi);
static void ku_qm35_uci_probe_cleanup(struct qm35_spi *qmspi);
static int ku_qm35_uci_probe_device_reset(struct qm35_spi *qmspi);
struct qm35_uci_device_info;
static int ku_qm35_uci_probe_device_info(struct qm35_spi *qmspi,
					 struct qm35_uci_device_info *info,
					 size_t info_sz);
#define qm35_uci_probe_setup ku_qm35_uci_probe_setup
#define qm35_uci_probe_cleanup ku_qm35_uci_probe_cleanup
#define qm35_uci_probe_device_reset ku_qm35_uci_probe_device_reset
#define qm35_uci_probe_device_info ku_qm35_uci_probe_device_info

static int ku_qm35_fw_upgrade(struct qm35_spi *qmspi);
static int ku_qm35_fw_get_device_id(struct qm35_spi *qmspi);
static void ku_qm35_fw_deinit_qmrom(struct qm35_spi *qmspi);
static int ku_qm35_fw_load(struct qm35_spi *qmspi, const char *fw_name);
static int ku_qm35_fw_free(struct qm35_spi *qmspi);
struct qm35_fw_version;
static int ku_qm35_fw_get_vendor_version(struct qm35_spi *qmspi,
					 struct qm35_fw_version *version);
#define qm35_fw_upgrade ku_qm35_fw_upgrade
#define qm35_fw_get_device_id ku_qm35_fw_get_device_id
#define qm35_fw_deinit_qmrom ku_qm35_fw_deinit_qmrom
#define qm35_fw_load ku_qm35_fw_load
#define qm35_fw_free ku_qm35_fw_free
#define qm35_fw_get_vendor_version ku_qm35_fw_get_vendor_version

static int ku_qm35_enqueue(struct qm35_spi *qmspi, struct qm35_work *cmd);
#define qm35_enqueue ku_qm35_enqueue

static int ku_qm35_state_wait(struct qm35 *qm35, enum qm35_state device_state);
#define qm35_state_wait ku_qm35_state_wait

static int ku_spi_sync_transfer(struct spi_device *spi,
				struct spi_transfer *xfers,
				unsigned int num_xfers);
#define spi_sync_transfer ku_spi_sync_transfer

static int ku_gpiod_set_value_cansleep(void *gpio, int value);
#define gpiod_set_value_cansleep ku_gpiod_set_value_cansleep

static void ku_disable_irq(unsigned int irq);
static void ku_enable_irq(unsigned int irq);
#define disable_irq ku_disable_irq
#define enable_irq ku_enable_irq

static int ku_sysfs_create_bin_file(struct kobject *kobj,
				    const struct bin_attribute *attr);
static void ku_sysfs_remove_bin_file(struct kobject *kobj,
				     const struct bin_attribute *attr);
#define sysfs_create_bin_file ku_sysfs_create_bin_file
#define sysfs_remove_bin_file ku_sysfs_remove_bin_file

#endif /* QM35_SPI_TRANSPORT_TESTS */
#endif /* __QM35_SPI_TRANSPORT_H */
