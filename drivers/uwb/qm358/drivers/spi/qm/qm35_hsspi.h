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
#ifndef __QM35_HSSPI_H
#define __QM35_HSSPI_H

#include <linux/bitops.h>

struct qm35_spi;

#define QM35_HSSPI_RETRY_COUNT 3
#define QM35_HSSPI_RETRY_DELAY_US 50

#define QM35_WAKEUP_DURATION_US 500
#define QM35_WAKEUP_DELAY_US 3000

/* See peg_spi_driver/HSSPI.{c,h} at
   https://gitlab.com/qorvo/uwb-eng/qm35xxx-pegasus/drivers/peg_spi_driver */

enum hsspi_flags_host {
	HSSPI_HOST_RD = BIT(5),
	HSSPI_HOST_PRD = BIT(6),
	HSSPI_HOST_WR = BIT(7),
};

enum hsspi_flags_soc {
	HSSPI_SOC_BOOTROM_ERR = BIT(0),
	HSSPI_SOC_BOOTROM_RDY = BIT(1),
	HSSPI_SOC_BOOTROM_OA = BIT(2),
	HSSPI_SOC_BOOTROM_ODW = BIT(3),
	HSSPI_SOC_ERR = BIT(4),
	HSSPI_SOC_RDY = BIT(5),
	HSSPI_SOC_OA = BIT(6),
	HSSPI_SOC_ODW = BIT(7),
};

/* Keep enum qm35_transport_msg_type sync with this one. */
enum hsspi_ul_value {
	UL_RESERVED,
	UL_BOOT_FLASH,
	UL_UCI_APP,
	UL_COREDUMP,
	UL_LOG,
	UL_QTRACE,
	UL_AWAKE
};

/**
 * struct qm35_hsspi_header - QM35 HSSPI protocol header.
 * @flags: Combination of enum hsspi_flags_host or enum hsspi_flags_soc.
 * @ul_value: One of enum hsspi_ul_value.
 * @length: Transaction length, including this header.
 */
struct qm35_hsspi_header {
	u8 flags;
	u8 ul_value;
	u16 length;
} __packed;

int qm35_hsspi_send(struct qm35_spi *qmspi, u8 ul_value, const void *data,
		    size_t length);
int qm35_hsspi_recv(struct qm35_spi *qmspi, struct qm35_hsspi_header *header,
		    void *data, size_t size);
int qm35_hsspi_wakeup(struct qm35_spi *qmspi, bool force);

#ifdef QM35_HSSPI_TESTS

#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

int ku_spi_sync(struct spi_device *spi, struct spi_message *message);
int ku_gpiod_get_value(struct gpio_desc *gpio);
int ku_gpiod_set_value(struct gpio_desc *gpio, int val);

/* Redefine some functions to use our test wrappers */
#define spi_sync ku_spi_sync
#define gpiod_get_value ku_gpiod_get_value
#define gpiod_set_value ku_gpiod_set_value

#endif /* QM35_HSSPI_TESTS */

#endif /* __QM35_HSSPI_H */
