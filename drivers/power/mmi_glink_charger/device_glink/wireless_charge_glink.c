/*
 * Copyright (C) 2024 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/power_supply.h>
#include <linux/notifier.h>
#include "mmi_glink_core.h"
#include "qti_glink_charger_v2.h"
#include "device_class.h"
#include "wireless_charge_glink.h"

static struct mmi_glink_chip *this_root_chip =  NULL;
static struct wireless_glink_dev *this_chip = NULL;

static void wireless_psy_init(struct wireless_glink_dev *chip);

 static int wireless_notify_handler(struct notifier_block *nb, unsigned long event, void *data)
{
	int rc = -1;
	struct wls_dump wls_info;
	struct wireless_glink_dev *wls_chip =
			container_of(nb, struct wireless_glink_dev, wls_nb);

	mmi_dbg(this_root_chip, "wireless: notify-dev %ld", event);
	if (event == DEV_WLS || event == DEV_ALL) {

		if (!wls_chip->wls_dev_psy) {
			wireless_psy_init(wls_chip);
		}

		rc = qti_charger_get_property(OEM_PROP_WLS_DUMP_INFO,
					&wls_info,
					sizeof(struct wls_dump));

		if (rc) {
			mmi_err(this_root_chip, "qti_glink: Get WLS_DUM_INFO property failed");
			return rc;
		}

		mmi_info(this_root_chip, "Wireless dump info -1: CHIP_ID: 0x%04x, MTP_FW_VER: 0x%04x, IRQ STATUS: 0x%04x, "
			"SYS_MODE:  RX/TX %d, OP_MODE:  BPP/EPP 0x%x, RX_FOP: %dkHz, RX_VOUT: %dmV, "
			"RX_VRECT: %dmV, RX_IRECT: %dmV, RX_NEG_POWER: %dw ",
			wls_info.chip_id,
			wls_info.mtp_fw_ver,
			wls_info.irq_status,
			wls_info.sys_mode,
			wls_info.op_mode,
			wls_info.rx_fop,
			wls_info.rx_vout_mv,
			wls_info.rx_vrect_mv,
			wls_info.rx_irect_ma,
			wls_info.rx_neg_power);

		mmi_info(this_root_chip, "Wireless dump info -2: TX_IIN: %dmA, TX_VIN: %dmV, TX_VRECT: %dmV, "
			"TX_DET_RX_POWER: %dmW, TX_POWER: %dmW, POWER_LOSS: %dmW, TX_FOD: %d, "
			"RX_CONNECTED: %d, RX_DEV_INFO: 0x%x:0x%x:0x%x, TX_EPT_RSN: 0x%04x, ",
			wls_info.tx_iin_ma,
			wls_info.tx_vin_mv,
			wls_info.tx_vrect_mv,
			wls_info.tx_det_rx_power,
			wls_info.tx_power,
			wls_info.power_loss,
			(wls_info.irq_status & (0x01<<12)) ? 1 : 0,
			wls_chip->rx_connected,
			wls_chip->rx_dev_mfg,
			wls_chip->rx_dev_type,
			wls_chip->rx_dev_id,
			wls_info.tx_ept);

		mmi_info(this_root_chip, "Wireless dump info -3: rx_ept: %d, rx_ce: %d, "
			"rx_rp: %d, rx_dietemp: %d, USB_OTG: %d, WLS_BOOST: %d, WLS_ICL_MA: %dmA, WLS_ICL_THERM_MA: %dmA",
			wls_info.rx_ept,
			wls_info.rx_ce,
			wls_info.rx_rp,
			wls_info.rx_dietemp,
			wls_info.usb_otg,
			wls_info.wls_boost,
			wls_info.wls_icl_ma,
			wls_info.wls_icl_therm_ma);


		mmi_info(this_root_chip, "Wireless dump info -4: WLC Stand: tx_type %d, tx_power: %d, "
			"fan: %d, light: %d, status: %d",
			wls_chip->wlc_tx_type,
			wls_chip->wlc_tx_power,
			wls_chip->wlc_fan_speed,
			wls_chip->wlc_light_ctl,
			wls_chip->wlc_status);
	}
	return NOTIFY_DONE;
}

//ATTRIBUTE_GROUPS(qti_charger);
#define TX_INT_FOD      (0x01<<12)
static int show_wls_dump_info(struct seq_file *m, void *data)
{
	struct wls_dump wls_info;
	int rc = -1;

	rc = qti_charger_get_property(OEM_PROP_WLS_DUMP_INFO,
				&wls_info,
				sizeof(struct wls_dump));

	if (rc) {
		mmi_err(this_root_chip, "qti_glink: Get WLS_DUM_INFO property failed");
		return rc;
	}

	seq_printf(m, "CHIP_ID: 0x%04x\n", wls_info.chip_id);

	seq_printf(m, "MTP_FW_VER: 0x%04x\n", wls_info.mtp_fw_ver);

	seq_printf(m, "IRQ STATUS: 0x%04x\n", wls_info.irq_status);

	seq_printf(m, "SYS_MODE:  RX/TX %d\n", wls_info.sys_mode);

	seq_printf(m, "OP_MODE:  BPP/EPP/Moto50W 0x%x\n", wls_info.op_mode);

	seq_printf(m, "RX_FOP:   %dkHz\n", wls_info.rx_fop);

	seq_printf(m, "RX_VOUT: %dmV\n",  wls_info.rx_vout_mv);

	seq_printf(m, "RX_VRECT: %dmV\n",  wls_info.rx_vrect_mv);

	seq_printf(m, "RX_IRECT: %dmA\n",  wls_info.rx_irect_ma);

	seq_printf(m, "RX_EPT: 0x%04x\n",  wls_info.rx_ept);

	seq_printf(m, "RX_CE: %d\n",  wls_info.rx_ce);

	seq_printf(m, "RX_RP: %d\n",  wls_info.rx_rp);

	seq_printf(m, "RX_DieTemp: %dC\n",  wls_info.rx_dietemp);

	seq_printf(m, "RX_NEG_POWER: %dw\n",  wls_info.rx_neg_power);

	seq_printf(m, "TX_IIN: %dmA\n",  wls_info.tx_iin_ma);

	seq_printf(m, "TX_VIN: %dmV\n",  wls_info.tx_vin_mv);

	seq_printf(m, "TX_VRECT: %dmV\n",  wls_info.tx_vrect_mv);

	seq_printf(m, "TX_DET_RX_POWER: %dmW\n",  wls_info.tx_det_rx_power);

	seq_printf(m, "TX_POWER: %dmW\n",  wls_info.tx_power);

	seq_printf(m, "TX_EPT_RSN: 0x%04x\n",  wls_info.tx_ept);

	seq_printf(m, "POWER_LOSS: %dmW\n",  wls_info.power_loss);

	seq_printf(m, "TX_FOD: %d\n",  (wls_info.irq_status & TX_INT_FOD) ? 1 : 0);

	seq_printf(m, "USB_OTG: %d\n",  wls_info.usb_otg);

	seq_printf(m, "WLS_BOOST: %d\n",  wls_info.wls_boost);

	seq_printf(m, "WLS_ICL_MA: %d\n",  wls_info.wls_icl_ma);

	seq_printf(m, "WLS_ICL_THERM_MA: %d\n",  wls_info.wls_icl_therm_ma);

	return 0;
}

static int wls_dump_info_debugfs_open(struct inode *inode, struct file *file)
{
	struct wireless_glink_dev *chip = inode->i_private;

	return single_open(file, show_wls_dump_info, chip);
}

static const struct file_operations wls_dump_info_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= wls_dump_info_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entries(struct wireless_glink_dev *chip)
{
	struct dentry *ent;

	chip->wls_debug_root = debugfs_create_dir("wireless_glink", NULL);
	if (!chip->wls_debug_root) {
		mmi_err(this_root_chip, "Couldn't create debug dir\n");
		return;
	}

	ent = debugfs_create_file("wls_dump_info", S_IFREG | S_IRUGO,
				  chip->wls_debug_root, chip,
				  &wls_dump_info_debugfs_ops);
	if (!ent)
		mmi_err(this_root_chip, "Couldn't create wls_dump_info debug file\n");
}

static ssize_t tx_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long tx_mode;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &tx_mode);
	if (r) {
		pr_err("Invalid tx_mode = %lu\n", tx_mode);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_WLS_TX_MODE,
				&tx_mode,
				sizeof(tx_mode));
	chg->tx_mode = tx_mode;
	if (chg->wls_dev_psy)
		sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "tx_mode");

	return r ? r : count;
}

static ssize_t tx_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct wireless_glink_dev *chg = this_chip;
	u32 tx_mode = 0;
	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}
	qti_charger_get_property(OEM_PROP_WLS_TX_MODE,
				&tx_mode,
				sizeof(tx_mode));

	chg->tx_mode = tx_mode;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", tx_mode);
}

static DEVICE_ATTR(tx_mode, S_IRUGO|S_IWUSR, tx_mode_show, tx_mode_store);

static ssize_t tx_mode_vout_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wireless_glink_dev *chg = this_chip;
	struct wls_dump wls_info;
	u32 tx_vout = 0;
	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_DUMP_INFO,
				&wls_info,
				sizeof(struct wls_dump));

	tx_vout = wls_info.tx_vrect_mv;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", tx_vout);
}

static DEVICE_ATTR(tx_mode_vout, S_IRUGO,
		tx_mode_vout_show,
		NULL);

static ssize_t wlc_light_ctl_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wlc_light_ctl;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wlc_light_ctl);
	if (r) {
		pr_err("Invalid wlc_light_ctl = %lu\n", wlc_light_ctl);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_WLS_WLC_LIGHT_CTL,
				&wlc_light_ctl,
				sizeof(wlc_light_ctl));
	chg->wlc_light_ctl = wlc_light_ctl;
	if (chg->wls_dev_psy)
		sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "wlc_light_ctl");

	return r ? r : count;
}

static ssize_t wlc_light_ctl_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->wlc_light_ctl);
}
static DEVICE_ATTR(wlc_light_ctl, S_IRUGO|S_IWUSR, wlc_light_ctl_show, wlc_light_ctl_store);


static ssize_t wlc_fan_speed_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wlc_fan_speed;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wlc_fan_speed);
	if (r) {
		pr_err("Invalid wlc_fan_speed = %lu\n", wlc_fan_speed);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_WLS_WLC_FAN_SPEED,
				&wlc_fan_speed,
				sizeof(wlc_fan_speed));
	chg->wlc_fan_speed = wlc_fan_speed;
	if (chg->wls_dev_psy)
		sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "wlc_fan_speed");

	return r ? r : count;
}

static ssize_t wlc_fan_speed_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->wlc_fan_speed);
}
static DEVICE_ATTR(wlc_fan_speed, S_IRUGO|S_IWUSR, wlc_fan_speed_show, wlc_fan_speed_store);

static ssize_t wlc_tx_type_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 type = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_WLC_TX_TYPE,
				&type,
				sizeof(type));

	chg->wlc_tx_type = type;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", type);
}

static DEVICE_ATTR(wlc_tx_type, S_IRUGO,
		wlc_tx_type_show,
		NULL);

static ssize_t wlc_tx_power_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 power = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_WLC_TX_POWER,
				&power,
				sizeof(power));

	chg->wlc_tx_power = power;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", power);
}

static DEVICE_ATTR(wlc_tx_power, S_IRUGO,
		wlc_tx_power_show,
		NULL);

static ssize_t wlc_tx_capability_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 capability = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_WLC_TX_CAPABILITY,
				&capability,
				sizeof(capability));

	chg->wlc_tx_capability = capability;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", capability);
}

static DEVICE_ATTR(wlc_tx_capability, S_IRUGO,
		wlc_tx_capability_show,
		NULL);

static ssize_t wlc_tx_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 id = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_WLC_TX_ID,
				&id,
				sizeof(id));

	chg->wlc_tx_id = id;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", id);
}

static DEVICE_ATTR(wlc_tx_id, S_IRUGO,
		wlc_tx_id_show,
		NULL);

static ssize_t wlc_tx_sn_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 sn = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_WLC_TX_SN,
				&sn,
				sizeof(sn));

	chg->wlc_tx_sn = sn;
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", sn);
}

static DEVICE_ATTR(wlc_tx_sn, S_IRUGO,
		wlc_tx_sn_show,
		NULL);

static ssize_t rx_connected_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->rx_connected);
}

static DEVICE_ATTR(rx_connected, S_IRUGO,
		rx_connected_show,
		NULL);

static ssize_t rx_dev_manufacturing_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 rx_dev_mfg = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_RX_DEV_MFG,
				&rx_dev_mfg,
				sizeof(rx_dev_mfg));

	chg->rx_dev_mfg= rx_dev_mfg;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%#x\n", chg->rx_dev_mfg);
}
static DEVICE_ATTR(rx_dev_manufacturing, S_IRUGO,
		rx_dev_manufacturing_show,
		NULL);

static ssize_t rx_dev_type_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 rx_dev_type = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_RX_DEV_TYPE,
				&rx_dev_type,
				sizeof(rx_dev_type));

	chg->rx_dev_type = rx_dev_type;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%#x\n", chg->rx_dev_type);
}
static DEVICE_ATTR(rx_dev_type, S_IRUGO,
		rx_dev_type_show,
		NULL);

static ssize_t rx_dev_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 rx_dev_id = 0;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_get_property(OEM_PROP_WLS_RX_DEV_ID,
				&rx_dev_id,
				sizeof(rx_dev_id));

	chg->rx_dev_id = rx_dev_id;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%#x\n", chg->rx_dev_id);
}
static DEVICE_ATTR(rx_dev_id, S_IRUGO,
		rx_dev_id_show,
		NULL);

static ssize_t wlc_st_changed_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->wlc_status);
}

static DEVICE_ATTR(wlc_st_changed, S_IRUGO,
		wlc_st_changed_show,
		NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_curr_max;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_curr_max);
	if (r) {
		mmi_err(this_root_chip, "Invalid TCMD = %lu\n", wls_curr_max);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_WLS_CURR_MAX,
				&wls_curr_max,
				sizeof(wls_curr_max));

	chg->wls_curr_max = wls_curr_max;
	return r ? r : count;
}

static ssize_t wls_input_current_limit_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->wls_curr_max);
}
static DEVICE_ATTR(wls_input_current_limit, S_IRUGO|S_IWUSR, wls_input_current_limit_show, wls_input_current_limit_store);

static ssize_t folio_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long folio_mode;
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &folio_mode);
	if (r) {
		pr_err("Invalid folio_mode = %lu\n", folio_mode);
		return -EINVAL;
	}

	r = qti_charger_set_property(OEM_PROP_WLS_FOLIO_MODE,
				&folio_mode,
				sizeof(folio_mode));
	chg->folio_mode = folio_mode;

	return r ? r : count;
}

static ssize_t folio_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct wireless_glink_dev *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->folio_mode);
}
static DEVICE_ATTR(folio_mode, S_IRUGO|S_IWUSR, folio_mode_show, folio_mode_store);


static int wireless_charger_notify_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct qti_charger_notify_data *notify_data = data;
	struct wireless_glink_dev *chg = container_of(nb, struct wireless_glink_dev, wls_glink_nb);

	if (notify_data->receiver != OEM_NOTIFY_RECEIVER_WLS_CHG) {
		pr_err("Skip mis-matched receiver: %#x\n", notify_data->receiver);
		return 0;
	}

        switch (event) {
        case NOTIFY_EVENT_WLS_RX_CONNECTED:
	/* RX connected update */
		if (notify_data->data[0] != chg->rx_connected) {
			if (chg->wls_dev_psy) {
				pr_info("report rx_connected %d\n", notify_data->data[0]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "rx_connected");
			}
		}
		chg->rx_connected = notify_data->data[0];
	            break;
        case NOTIFY_EVENT_WLS_RX_OVERTEMP:
		break;
        case NOTIFY_EVENT_WLS_CHANGE:
		if (notify_data->data[0] != chg->tx_mode) {
			if (chg->wls_dev_psy) {
				pr_info("report tx_mode %d\n", notify_data->data[0]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "tx_mode");
			}
		}
		break;
        case NOTIFY_EVENT_WLS_WLC_CHANGE:
	/* WLC status update */
		if (notify_data->data[0] != chg->wlc_status) {
			chg->wlc_status = notify_data->data[0];
			if (chg->wls_dev_psy) {
				pr_info("report wlc_st_changed %d\n", notify_data->data[0]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "wlc_st_changed");
			}
		}
		break;
        case NOTIFY_EVENT_WLS_RX_DEV_INFO_UPDATE:
	/* RX dev info update */
		if (notify_data->data[0] != chg->rx_dev_mfg) {
			if (chg->wls_dev_psy) {
				pr_info("report rx_dev_mfg %#x\n", notify_data->data[0]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "rx_dev_manufacturing");
			}
			chg->rx_dev_mfg = notify_data->data[0];
		}
		if (notify_data->data[1] != chg->rx_dev_type) {
			if (chg->wls_dev_psy) {
				pr_info("report rx_dev_type %#x\n", notify_data->data[1]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "rx_dev_type");
			}
			chg->rx_dev_type = notify_data->data[1];
		}
		if (notify_data->data[2] != chg->rx_dev_id) {
			if (chg->wls_dev_psy) {
				pr_info("report rx_dev_id %#x\n", notify_data->data[2]);
				sysfs_notify(&chg->wls_dev_psy->dev.parent->kobj, NULL, "rx_dev_id");
			}
			chg->rx_dev_id = notify_data->data[2];
		}
		break;
        default:
		pr_err("Unknown wireless event: %#lx\n", event);
                break;
        }

	if (chg->wls_dev_psy) {
		pr_info("wireless charger notify, event %lu\n", event);
		power_supply_changed(chg->wls_dev_psy);
	}

        return 0;
}

 static void wireless_psy_init(struct wireless_glink_dev *chip)
{
	int rc;

	if (chip->wls_dev_psy)
		return;

	chip->wls_dev_psy = power_supply_get_by_name("wireless");
	if (!chip->wls_dev_psy) {
		pr_err("No wireless power supply found\n");
		return;
	}
	pr_info("wireless power supply is found\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_tx_mode);
        if (rc)
		pr_err("couldn't create wireless tx mode\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_tx_mode_vout);
        if (rc)
		pr_err("couldn't create wireless tx mode vout\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_connected);
        if (rc)
		pr_err("couldn't create wireless rx_connected\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_manufacturing);
        if (rc)
		pr_err("couldn't create wireless rx_dev_manufacturing\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_type);
        if (rc)
		pr_err("couldn't create wireless rx_dev_type\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_id);
        if (rc)
		pr_err("couldn't create wireless rx_dev_id\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wls_input_current_limit);
        if (rc)
		pr_err("couldn't create wireless input current limit error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_folio_mode);
        if (rc)
		pr_err("couldn't create wireless folio mode error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_light_ctl);
        if (rc)
		pr_err("couldn't create wireless wlc light control error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_fan_speed);
        if (rc)
		pr_err("couldn't create wireless wlc fan speed error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_type);
        if (rc)
		pr_err("couldn't create wireless wlc tx type error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_power);
        if (rc)
		pr_err("couldn't create wireless wlc tx power capacity error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_capability);
        if (rc)
		pr_err("couldn't create wireless wlc tx capability error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_id);
        if (rc)
		pr_err("couldn't create wireless wlc tx id error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_sn);
        if (rc)
		pr_err("couldn't create wireless wlc tx sn error\n");

	rc = device_create_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_st_changed);
        if (rc)
		pr_err("couldn't create wireless wlc status changed error\n");

	chip->wls_glink_nb.notifier_call = wireless_charger_notify_callback;
	rc = qti_charger_register_notifier(&chip->wls_glink_nb);
	if (rc)
		pr_err("Failed to register notifier, rc=%d\n", rc);
}

static void wireless_psy_deinit(struct wireless_glink_dev *chip)
{
	if (!chip->wls_dev_psy)
		return;

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_tx_mode);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_tx_mode_vout);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_connected);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_manufacturing);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_type);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_rx_dev_id);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wls_input_current_limit);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_folio_mode);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_light_ctl);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_fan_speed);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_type);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_power);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_capability);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_id);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_tx_sn);

	device_remove_file(chip->wls_dev_psy->dev.parent,
				&dev_attr_wlc_st_changed);

	qti_charger_unregister_notifier(&chip->wls_glink_nb);

	power_supply_put(chip->wls_dev_psy);
	chip->wls_dev_psy = NULL;
}

struct glink_device *wireless_glink_device_register(struct mmi_glink_chip *chip, struct mmi_glink_dev_dts_info *dev_dts)
{

	struct wireless_glink_dev *wls_chip = NULL;
	struct glink_device * glink_dev = NULL;

	if (!chip)
		goto exit;

	this_root_chip = chip;

	wls_chip = kzalloc(sizeof(struct wireless_glink_dev),GFP_KERNEL);
	if (!wls_chip)
		goto exit;

	wls_chip->name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s", dev_dts->glink_dev_name);
	glink_dev = glink_device_register(dev_dts->glink_dev_name, chip->dev, DEV_WLS, wls_chip);
	if (!glink_dev)
		goto exit;

	wls_chip->wls_nb.notifier_call = wireless_notify_handler;
	mmi_glink_register_notifier(&wls_chip->wls_nb);

	wireless_psy_init(wls_chip);
	create_debugfs_entries(wls_chip);
	this_chip = wls_chip;

	mmi_err(chip, "wireless glink device %s register successfully", dev_dts->glink_dev_name);
	return glink_dev;
exit:
	return (struct glink_device *)NULL;
}

void wireless_glink_device_unregister(void)
{
	if(!this_chip)
		return;
	mmi_err(this_root_chip, "wireless_glink_device_unregister");
	if (this_chip->wls_debug_root)
		debugfs_remove_recursive(this_chip->wls_debug_root);
	wireless_psy_deinit(this_chip);
	mmi_glink_unregister_notifier(&this_chip->wls_nb);
	glink_device_unregister(this_chip->glink_dev);
	kfree(this_chip);
	this_chip = NULL;
	this_root_chip = NULL;
}
