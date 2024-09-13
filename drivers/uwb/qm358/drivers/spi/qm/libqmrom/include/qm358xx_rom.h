// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2021 Qorvo US, Inc.
 *
 */

#ifndef __QM358XX_ROM_H__
#define __QM358XX_ROM_H__

#define QM358XX_ROM_UUID_LEN 0x10
#define QM358XX_ROM_GIT_VERSION_LEN 8

/* Life cycle state definitions. */
#define LCS_TEST (0x00) /* Previously: CC_BSV_CHIP_MANUFACTURE_LCS */
#define LCS_OEM (0x01) /* Previously: CC_BSV_DEVICE_MANUFACTURE_LCS */
#define LCS_USER (0x02) /* Previously: CC_BSV_SECURE_LCS */
#define LCS_RMA (0x03) /* Previously: CC_BSV_RMA_LCS */

struct qm358xx_soc_infos {
	uint8_t uuid[QM358XX_ROM_UUID_LEN];
	uint32_t customer_id;
	uint32_t product_id;
	uint32_t arb_ver_icv;
	uint32_t arb_ver_oem;
	uint8_t lcs_state;
	uint8_t working_model;
	uint8_t business_ctx;
	uint8_t secure_debug;
	uint8_t secure_boot_oem;
	uint8_t fw_encryption_oem;
	uint8_t lock_debug;
	uint8_t sec_ver_icv;
	uint8_t sec_ver_oem;
	uint8_t enc_l2_lock;
	uint8_t git_version[QM358XX_ROM_GIT_VERSION_LEN];
};

int qm358xx_rom_gen_secrets(struct qmrom_handle *handle);
int qm358xx_rom_load_asset_pkg(struct qmrom_handle *handle,
			       struct firmware *asset_pkg);
int qm358xx_rom_load_fw_pkg(struct qmrom_handle *handle,
			    const struct firmware *fw_pkg);
int qm358xx_rom_load_secure_dbg_pkg(struct qmrom_handle *handle,
				    const struct firmware *dbg_pkg);
int qm358xx_rom_erase_secure_dbg_pkg(struct qmrom_handle *handle);
int qm358xx_rom_run_test_mode(struct qmrom_handle *handle);
int qm358xx_rom_load_sram_fw(struct qmrom_handle *handle,
			     struct firmware *sram_fw);

#endif /* __QM358XX_ROM_H__ */
