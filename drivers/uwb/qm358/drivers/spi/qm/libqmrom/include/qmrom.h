// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2021 Qorvo US, Inc.
 *
 */

#ifndef __QMROM_H__
#define __QMROM_H__

#ifndef __KERNEL__
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#else
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#define PRIu32 "u"
#define PRIu64 "llu"
#endif

#include "qmrom_endian.h"
#include "qmrom_error.h"

#undef CHECK_STCS

enum chip_revision_e {
	CHIP_REVISION_A0 = 0xA0,
	CHIP_REVISION_A1 = 0xA1,
	CHIP_REVISION_B0 = 0xB0,
	CHIP_REVISION_C0 = 0xC0,
	CHIP_REVISION_C2 = 0xC2,
	CHIP_REVISION_UNKNOWN = 0xFF
};

enum device_generation_e {
	DEVICE_GEN_QM357XX,
	DEVICE_GEN_QM358XX,
	DEVICE_GEN_QPF51XX,
	DEVICE_GEN_UNKNOWN = 0xFF
};

enum device_version_e {
	DEVICE_VER_QM357XX_A0 = 0x0410,
	DEVICE_VER_QM357XX_B0 = 0x0420,
	DEVICE_VER_QM357XX_C0 = 0x0430,
	DEVICE_VER_QM358XX = 0x0440,
	DEVICE_VER_QPF51XX = 0x0450,
	DEVICE_VER_UNKNOWN = 0xFFFF
};

struct qmrom_handle;

#include <qm357xx_rom.h>
#include <qm358xx_rom.h>

#define SSTC2UINT32(handle, offset)                                      \
	({                                                               \
		uint32_t tmp = 0xbeefdeed;                               \
		if ((handle)->sstc->len >= (offset) + sizeof(tmp))       \
			memcpy(&tmp, &(handle)->sstc->payload[(offset)], \
			       sizeof(tmp));                             \
		tmp;                                                     \
	})

#define SSTC2UINT16(handle, offset)                                      \
	({                                                               \
		uint16_t tmp = 0xbeed;                                   \
		if ((handle)->sstc->len >= (offset) + sizeof(tmp))       \
			memcpy(&tmp, &(handle)->sstc->payload[(offset)], \
			       sizeof(tmp));                             \
		tmp;                                                     \
	})

/* Those functions allow the libqmrom to call
 * device specific functions
 */
typedef int (*reset_device_fn)(void *handle);

struct device_ops {
	reset_device_fn reset;
};

/* Those functions allow the libqmrom to call
 * revision specific functions for QM357XX
 */
typedef int (*flash_fw_fn)(struct qmrom_handle *handle,
			   const struct firmware *fw);
typedef int (*flash_unstitched_fw_fn)(struct qmrom_handle *handle,
				      const struct unstitched_firmware *fw);
typedef int (*flash_debug_cert_fn)(struct qmrom_handle *handle,
				   struct firmware *dbg_cert);
typedef int (*erase_debug_cert_fn)(struct qmrom_handle *handle);

/* Those functions allow the libqmrom to call
 * revision specific functions for QM358XX
 */
typedef int (*gen_secrets_fn)(struct qmrom_handle *handle);
typedef int (*load_asset_pkg_fn)(struct qmrom_handle *handle,
				 const struct firmware *asset_pkg);
typedef int (*load_fw_pkg_fn)(struct qmrom_handle *handle,
			      const struct firmware *fw_pkg);
typedef int (*load_secure_dbg_pkg_fn)(struct qmrom_handle *handle,
				      const struct firmware *dbg_pkg);
typedef int (*erase_secure_dbg_pkg_fn)(struct qmrom_handle *handle);
typedef int (*run_test_mode_fn)(struct qmrom_handle *handle);
typedef int (*load_sram_fw_fn)(struct qmrom_handle *handle,
			       const struct firmware *sram_fw);
struct rom_code_ops {
	flash_fw_fn flash_fw;
	flash_unstitched_fw_fn flash_unstitched_fw;
	flash_debug_cert_fn flash_debug_cert;
	erase_debug_cert_fn erase_debug_cert;
	gen_secrets_fn gen_secrets;
	load_asset_pkg_fn load_asset_pkg;
	load_fw_pkg_fn load_fw_pkg;
	load_secure_dbg_pkg_fn load_secure_dbg_pkg;
	erase_secure_dbg_pkg_fn erase_secure_dbg_pkg;
	run_test_mode_fn run_test_mode;
	load_sram_fw_fn load_sram_fw;
};

struct qmrom_handle {
	void *spi_handle;
	void *reset_handle;
	void *ss_irq_handle;
	int comms_retries;
	enum device_generation_e dev_gen;
	enum chip_revision_e chip_rev;
	uint16_t device_version;
	struct device_ops dev_ops;
	int spi_speed;
	struct rom_code_ops rom_ops;
	struct stc *hstc;
	struct stc *sstc;
	union {
		struct qm357xx_soc_infos qm357xx_soc_info;
		struct qm358xx_soc_infos qm358xx_soc_info;
	};
	bool skip_check_fw_boot;
};

struct qmrom_handle *qmrom_init(void *spi_handle, void *reset_handle,
				void *ss_irq_handle, int spi_speed,
				int comms_retries, reset_device_fn reset,
				enum device_generation_e dev_gen_hint);
void qmrom_deinit(struct qmrom_handle *handle);
int qmrom_reboot_bootloader(struct qmrom_handle *handle);

int qmrom_spi_transfer_delay(void *handle, char *rbuf, const char *wbuf,
			     size_t size);

int qmrom_poll(struct qmrom_handle *handle);
int qmrom_pre_read(struct qmrom_handle *handle);
int qmrom_read(struct qmrom_handle *handle);

int qmrom_write_cmd(struct qmrom_handle *handle, uint8_t cmd);
int qmrom_write_cmd32(struct qmrom_handle *handle, uint32_t cmd);
int qmrom_write_size_cmd(struct qmrom_handle *handle, uint8_t cmd,
			 uint16_t data_size, const char *data);
int qmrom_write_size_cmd32(struct qmrom_handle *handle, uint32_t cmd,
			   uint16_t data_size, const char *data);

void qmrom_poll_soc(struct qmrom_handle *handle);
bool qmrom_is_soc_ready(struct qmrom_handle *handle);
int qmrom_wait_ready(struct qmrom_handle *handle);
int qmrom_poll_cmd_resp(struct qmrom_handle *handle);

#ifdef CHECK_STCS
void check_stcs(const char *func, int line, struct qmrom_handle *h);
#else
#define check_stcs(f, l, h)
#endif
#endif /* __QMROM_H__ */
