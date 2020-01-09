/*
 * Copyright (C) 2013 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define ST21NFC_MAGIC	0xEA

#define ST21NFC_NAME "st21nfc"
/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP _IO(ST21NFC_MAGIC, 0x01)
#define ST21NFC_PULSE_RESET _IO(ST21NFC_MAGIC, 0x02)
#define ST21NFC_SET_POLARITY_RISING _IO(ST21NFC_MAGIC, 0x03)
#define ST21NFC_SET_POLARITY_FALLING _IO(ST21NFC_MAGIC, 0x04)
#define ST21NFC_SET_POLARITY_HIGH _IO(ST21NFC_MAGIC, 0x05)
#define ST21NFC_SET_POLARITY_LOW _IO(ST21NFC_MAGIC, 0x06)
#define ST21NFC_GET_POLARITY _IO(ST21NFC_MAGIC, 0x07)
#define ST21NFC_RECOVERY _IO(ST21NFC_MAGIC, 0x08)
#define ST21NFC_USE_ESE _IOW(ST21NFC_MAGIC, 0x09, unsigned int)

// Keep compatibility with older user applications.
#define ST21NFC_LEGACY_GET_WAKEUP _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_LEGACY_PULSE_RESET _IOR(ST21NFC_MAGIC, 0x02, unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_RISING _IOR(ST21NFC_MAGIC, 0x03, \
	unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_FALLING _IOR(ST21NFC_MAGIC, 0x04, \
	unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_HIGH _IOR(ST21NFC_MAGIC, 0x05, unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_LOW _IOR(ST21NFC_MAGIC, 0x06, unsigned int)
#define ST21NFC_LEGACY_GET_POLARITY _IOR(ST21NFC_MAGIC, 0x07, unsigned int)
#define ST21NFC_LEGACY_RECOVERY _IOR(ST21NFC_MAGIC, 0x08, unsigned int)

struct st21nfc_platform_data {
	unsigned int irq_gpio;
	unsigned int ena_gpio;
	unsigned int reset_gpio;
	unsigned int clkreq_gpio;
	unsigned int polarity_mode;
};

#define ST54SPI_CB_RESET_END 0
#define ST54SPI_CB_RESET_START 1
#define ST54SPI_CB_ESE_NOT_USED 2
#define ST54SPI_CB_ESE_USED 3
void st21nfc_register_st54spi_cb(void (*cb)(int, void *), void *data);
void st21nfc_unregister_st54spi_cb(void);
