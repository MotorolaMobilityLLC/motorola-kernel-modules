/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __RT9471_CHARGER_H
#define __RT9471_CHARGER_H

#define RT9471_DEVICE_ADDR	0x53
#define RT9470_DEVID		0x09
#define RT9470D_DEVID		0x0A
#define RT9471_DEVID		0x0D
#define RT9471D_DEVID		0x0E

enum rt9471_reg_addr {
	RT9471_REG_OTGCFG = 0x00,
	RT9471_REG_TOP,
	RT9471_REG_FUNCTION,
	RT9471_REG_IBUS,
	RT9471_REG_VBUS,
	RT9471_REG_PRECHG,
	RT9471_REG_REGU,
	RT9471_REG_VCHG,
	RT9471_REG_ICHG,
	RT9471_REG_CHGTIMER,
	RT9471_REG_EOC,
	RT9471_REG_INFO,
	RT9471_REG_JEITA,
	RT9471_REG_PUMPEXP,
	RT9471_REG_DPDMDET,
	RT9471_REG_STATUS,
	RT9471_REG_STAT0,
	RT9471_REG_STAT1,
	RT9471_REG_STAT2,
	RT9471_REG_STAT3,
	RT9471_REG_IRQ0 = 0x20,
	RT9471_REG_IRQ1,
	RT9471_REG_IRQ2,
	RT9471_REG_IRQ3,
	RT9471_REG_MASK0 = 0x30,
	RT9471_REG_MASK1,
	RT9471_REG_MASK2,
	RT9471_REG_MASK3,
	RT9471_REG_HIDDEN_0 = 0x40,
	RT9471_REG_HIDDEN_2 = 0x42,
	RT9471_REG_TOP_HDEN,
	RT9471_REG_BUCK_HDEN1 = 0x45,
	RT9471_REG_BUCK_HDEN2 = 0x46,
	RT9471_REG_BUCK_HDEN3 = 0x54,
	RT9471_REG_BUCK_HDEN4,
	RT9471_REG_OTG_HDEN2 = 0x58,
	RT9471_REG_BUCK_HDEN5,
	RT9471_REG_PASSCODE1 = 0xA0,
};

/* ========== OTGCFG 0x00 ============ */
#define RT9471_OTGCC_MASK	BIT(0)

/* ========== TOP 0x01 ============ */
#define RT9471_QONRST_MASK	BIT(7)
#define RT9471_DISI2CTO_MASK	BIT(3)
#define RT9471_WDTCNTRST_MASK	BIT(2)
#define RT9471_WDT_SHIFT	0
#define RT9471_WDT_MASK		0x03

/* ========== FUNCTION 0x02 ============ */
#define RT9471_BATFETDIS_SHIFT	7
#define RT9471_BATFETDIS_MASK	BIT(7)
#define RT9471_HZ_SHIFT		5
#define RT9471_HZ_MASK		BIT(5)
#define RT9471_OTG_EN_SHIFT	1
#define RT9471_OTG_EN_MASK	BIT(1)
#define RT9471_CHG_EN_SHIFT	0
#define RT9471_CHG_EN_MASK	BIT(0)

/* ========== IBUS 0x03 ============ */
#define RT9471_AICC_EN_SHIFT	7
#define RT9471_AICC_EN_MASK	BIT(7)
#define RT9471_AUTOAICR_MASK	BIT(6)
#define RT9471_AICR_SHIFT	0
#define RT9471_AICR_MASK	0x3F
#define RT9471_AICR_MIN		50000
#define RT9471_AICR_MAX		3200000
#define RT9471_AICR_STEP	50000

/* ========== VBUS 0x04 ============ */
#define RT9471_VAC_OVP_SHIFT	6
#define RT9471_VAC_OVP_MASK	0xC0
#define RT9471_MIVR_SHIFT	0
#define RT9471_MIVR_MASK	0x0F
#define RT9471_MIVR_MIN		3900000
#define RT9471_MIVR_MAX		5400000
#define RT9471_MIVR_STEP	100000
#define RT9471_MIVRTRACK_SHIFT	4
#define RT9471_MIVRTRACK_MASK	0x30

/* ========== VCHG 0x07 ============ */
#define RT9471_VRECHG_MASK	BIT(7)
#define RT9471_CV_SHIFT		0
#define RT9471_CV_MASK		0x7F
#define RT9471_CV_MIN		3900000
#define RT9471_CV_MAX		4700000
#define RT9471_CV_STEP		10000

/* ========== ICHG 0x08 ============ */
#define RT9471_ICHG_SHIFT	0
#define RT9471_ICHG_MASK	0x3F
#define RT9471_ICHG_MIN		0
#define RT9471_ICHG_MAX		3150000
#define RT9471_ICHG_STEP	50000

/* ========== CHGTIMER 0x09 ============ */
#define RT9471_SAFETMR_EN_SHIFT	7
#define RT9471_SAFETMR_EN_MASK	BIT(7)
#define RT9471_SAFETMR_SHIFT	4
#define RT9471_SAFETMR_MASK	0x30
#define RT9471_SAFETMR_MIN	5
#define RT9471_SAFETMR_MAX	20
#define RT9471_SAFETMR_STEP	5

/* ========== EOC 0x0A ============ */
#define RT9471_IEOC_SHIFT	4
#define RT9471_IEOC_MASK	0xF0
#define RT9471_IEOC_MIN		50000
#define RT9471_IEOC_MAX		800000
#define RT9471_IEOC_STEP	50000
#define RT9471_TE_MASK		BIT(1)
#define RT9471_EOC_RST_SHIFT	0
#define RT9471_EOC_RST_MASK	BIT(0)

/* ========== INFO 0x0B ============ */
#define RT9471_REGRST_MASK	BIT(7)
#define RT9471_DEVID_SHIFT	3
#define RT9471_DEVID_MASK	0x78
#define RT9471_DEVREV_SHIFT	0
#define RT9471_DEVREV_MASK	0x03

/* ========== JEITA 0x0C ============ */
#define RT9471_JEITA_EN_MASK	BIT(7)

/* ========== PUMPEXP 0x0D ============ */
#define RT9471_PE_EN_MASK	BIT(7)
#define RT9471_PE_SEL_MASK	BIT(6)
#define RT9471_PE10_INC_MASK	BIT(5)
#define RT9471_PE20_CODE_SHIFT	0
#define RT9471_PE20_CODE_MASK	0x1F
#define RT9471_PE20_CODE_MIN	5500000
#define RT9471_PE20_CODE_MAX	20000000
#define RT9471_PE20_CODE_STEP	500000

/* ========== DPDMDET 0x0E ============ */
#define RT9471_BC12_EN_MASK	BIT(7)

/* ========== STATUS 0x0F ============ */
#define RT9471_PORTSTAT_SHIFT	4
#define RT9471_PORTSTAT_MASK	0xF0
#define RT9471_ICSTAT_SHIFT	0
#define RT9471_ICSTAT_MASK	0x0F

/* ========== STAT0 0x10 ============ */
#define RT9471_ST_VBUSGD_SHIFT		7
#define RT9471_ST_VBUSGD_MASK		BIT(7)
#define RT9471_ST_CHGRDY_SHIFT		6
#define RT9471_ST_CHGRDY_MASK		BIT(6)
#define RT9471_ST_IEOC_SHIFT		5
#define RT9471_ST_IEOC_MASK		BIT(5)
#define RT9471_ST_BGCHG_SHIFT		4
#define RT9471_ST_BGCHG_MASK		BIT(4)
#define RT9471_ST_CHGDONE_SHIFT		3
#define RT9471_ST_CHGDONE_MASK		BIT(3)
#define RT9471_ST_BC12_DONE_SHIFT	0
#define RT9471_ST_BC12_DONE_MASK	BIT(0)

/* ========== STAT1 0x11 ============ */
#define RT9471_ST_MIVR_SHIFT	7
#define RT9471_ST_MIVR_MASK	BIT(7)
#define RT9471_ST_AICR_MASK	BIT(6)
#define RT9471_ST_BATOV_MASK	BIT(1)

/* ========== STAT2 0x12 ============ */
#define RT9471_ST_SYSMIN_SHIFT	1

/* ========== STAT3 0x13 ============ */
#define RT9471_ST_VACOV_SHIFT	6
#define RT9471_ST_VACOV_MASK	BIT(6)

/* ========== HIDDEN_0 0x40 ============ */
#define RT9471_CHIP_REV_SHIFT	5
#define RT9471_CHIP_REV_MASK	0xE0

/* ========== HIDDEN_2 0x42 ============ */
#define RT9471_FORCE_HZ_SHIFT	2
#define RT9471_FORCE_HZ_MASK	BIT(2)

/* ========== TOP_HDEN 0x43 ============ */
#define RT9471_FORCE_EN_VBUS_SINK_SHIFT	4
#define RT9471_FORCE_EN_VBUS_SINK_MASK	BIT(4)

/* ========== OTG_HDEN2 0x58 ============ */
#define RT9471_REG_OTG_RES_COMP_SHIFT	4
#define RT9471_REG_OTG_RES_COMP_MASK	0x30


#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
enum WT_charger_type{
	WT_CHG_TYPE_BEGIN = 0,
	WT_CHG_TYPE_FC,
	WT_CHG_TYPE_SDP,
	WT_CHG_TYPE_CDP,
	WT_CHG_TYPE_DCP,
	WT_CHG_TYPE_QC2,
	WT_CHG_TYPE_QC3,
	WT_CHG_TYPE_OCP,
	WT_CHG_TYPE_QC3P_18W,//0x8
	WT_CHG_TYPE_QC3P_27W,
	WT6670_CHG_TYPE_UNKNOWN,
	Z350_CHG_TYPE_HVDCP = 0x10,
};

enum mmi_qc3p_ext_iio_channels {
	/*qc3p*/
	SMB5_USB_REAL_TYPE,
	SMB5_QC3P_POWER,
	SMB5_QC3P_START_DETECT,
	SMB5_QC3P_DETECTION_READY,
	SMB5_QC3P_START_POLICY,
	SMB5_BC12_START_DETECT,
	SMB5_BC12_DETECTION_READY,
	SMB5_READ_USBIN_VOLTAGE,
};

static const char * const mmi_qc3p_ext_iio_chan_name[] = {
	/*qc3p*/
	[SMB5_USB_REAL_TYPE] = "wt6670_usb_real_type",
	[SMB5_QC3P_POWER] = "wt6670_usb_qc3p_power",
	[SMB5_QC3P_START_DETECT] = "wt6670_start_detection",
	[SMB5_QC3P_DETECTION_READY] "wt6670_detection_ready",
	[SMB5_QC3P_START_POLICY] "qc3p_start_policy",
	[SMB5_BC12_START_DETECT] = "wt6670_start_bc12_detection",
	[SMB5_BC12_DETECTION_READY] = "wt6670_detection_bc12_ready",
	[SMB5_READ_USBIN_VOLTAGE] = "read_usbin_voltage",
};

#endif
#endif /* __RT9471_CHARGER_H */
