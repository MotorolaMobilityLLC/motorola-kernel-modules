/*
 * Copyright (C) 2020 Motorola Mobility LLC
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

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/power/bm_adsp_ulog.h>

#include "mmi_charger.h"

/* PPM specific definitions */
#define MSG_OWNER_OEM			32782
#define MSG_TYPE_REQ_RESP		1
#define OEM_PROPERTY_DATA_SIZE		16

#define OEM_READ_BUF_REQ		0x10000
#define OEM_WRITE_BUF_REQ		0x10001

#define OEM_WAIT_TIME_MS		5000

#define BATT_DEFAULT_ID 107000
#define BATT_SN_UNKNOWN "unknown-sn"

#define OEM_BM_ULOG_SIZE		4096

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for qti glink charger driver");

enum oem_property_type {
	OEM_PROP_BATT_INFO,
	OEM_PROP_CHG_INFO,
	OEM_PROP_CHG_PROFILE_INFO,
	OEM_PROP_CHG_PROFILE_DATA,
	OEM_PROP_CHG_FV,
	OEM_PROP_CHG_FCC,
	OEM_PROP_CHG_ITERM,
	OEM_PROP_CHG_FG_ITERM,
	OEM_PROP_CHG_BC_PMAX,
	OEM_PROP_CHG_QC_PMAX,
	OEM_PROP_CHG_PD_PMAX,
	OEM_PROP_CHG_WLS_PMAX,
	OEM_PROP_CHG_SUSPEND,
	OEM_PROP_CHG_DISABLE,
	OEM_PROP_DEMO_MODE,
	OEM_PROP_FACTORY_MODE,
	OEM_PROP_FACTORY_VERSION,
	OEM_PROP_TCMD,
	OEM_PROP_PMIC_ICL,
	OEM_PROP_REG_ADDRESS,
	OEM_PROP_REG_DATA,
	OEM_PROP_MAX,
};

struct battery_info {
	int batt_uv;
	int batt_ua;
	int batt_soc; /* 0 ~ 10000 indicating 0% to 100% */
	int batt_temp; /* hundredth degree */
	int batt_status;
	int batt_full_uah;
	int batt_design_uah;
};

struct charger_info {
	int chrg_uv;
	int chrg_ua;
	int chrg_type;
	int chrg_pmax_mw;
	int chrg_present;
};

struct charger_profile_info {
	int fg_iterm;
        int chrg_iterm;
        int max_fv_uv;
        int max_fcc_ua;
        int vfloat_comp_uv;
        int demo_fv_uv;
        int data_bk_size; /* number of byte for each data block */
	int data_size; /* number of byte for while data */
	int profile_id; /* profile id for charging profile selection in ADSP */
};

struct oem_read_buf_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			data_size;
};

struct oem_read_buf_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			buf[OEM_PROPERTY_DATA_SIZE];
	u32			data_size;
};

struct oem_write_buf_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			buf[OEM_PROPERTY_DATA_SIZE];
	u32			data_size;
};

struct oem_write_buf_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct qti_charger {
	char				*name;
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct completion		read_ack;
	struct completion		write_ack;
	struct mutex			read_lock;
	struct mutex			write_lock;
	struct oem_read_buf_resp_msg	rx_buf;
	atomic_t			rx_valid;
	struct work_struct		setup_work;
	atomic_t			state;
	u32				chrg_taper_cnt;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_info		chg_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_constraint	constraint;
	struct mmi_charger_driver	*driver;
	u32				*profile_data;
	struct charger_profile_info	profile_info;
	void				*ipc_log;
	bool				*debug_enabled;
};

static int find_profile_id(struct qti_charger *chg)
{
	int i;
	int rc;
	int count;
	int profile_id = -EINVAL;
	struct profile_sn_map {
		const char *id;
		const char *sn;
	} *map_table;

	count = of_property_count_strings(chg->dev->of_node, "profile-ids-map");
	if (count <= 0 || (count % 2)) {
		mmi_err(chg, "Invalid profile-ids-map in DT, rc=%d\n", count);
		return -EINVAL;
	}

	map_table = devm_kmalloc_array(chg->dev, count / 2,
					sizeof(struct profile_sn_map),
					GFP_KERNEL);
	if (!map_table)
		return -ENOMEM;

	rc = of_property_read_string_array(chg->dev->of_node, "profile-ids-map",
					(const char **)map_table,
					count);
	if (rc < 0) {
		mmi_err(chg, "Failed to get profile-ids-map, rc=%d\n", rc);
		profile_id = rc;
		goto free_map;
	}

	for (i = 0; i < count / 2 && map_table[i].sn; i++) {
		mmi_info(chg, "profile_ids_map[%d]: id=%s, sn=%s\n", i,
					map_table[i].id, map_table[i].sn);
		if (!strcmp(map_table[i].sn, chg->batt_info.batt_sn))
			profile_id = i;
	}

	if (profile_id >= 0 && profile_id < count / 2) {
		i = profile_id;
		profile_id = 0;
		rc = kstrtou32(map_table[i].id, 0, &profile_id);
		if (rc) {
			mmi_err(chg, "Invalid id: %s, sn: %s\n",
						map_table[i].id,
						map_table[i].sn);
			profile_id = rc;
		} else {
			mmi_info(chg, "profile id: %s(%d), sn: %s\n",
						map_table[i].id,
						profile_id,
						map_table[i].sn);
		}
	} else {
		mmi_warn(chg, "No matched profile id in profile-ids-map\n");
	}

free_map:
	devm_kfree(chg->dev, map_table);

	return profile_id;
}

static int handle_oem_read_ack(struct qti_charger *chg, void *data, size_t len)
{
	if (len != sizeof(chg->rx_buf)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(chg->rx_buf));
		atomic_set(&chg->rx_valid, 0);
		return -EINVAL;
	}

	memcpy(&chg->rx_buf, data, sizeof(chg->rx_buf));
	atomic_set(&chg->rx_valid, 1);
	complete(&chg->read_ack);
	mmi_dbg(chg, "read ack for property: %u\n", chg->rx_buf.oem_property_id);

	return 0;
}

static int handle_oem_write_ack(struct qti_charger *chg, void *data, size_t len)
{
	struct oem_write_buf_resp_msg *msg_ptr;

	if (len != sizeof(*msg_ptr)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(*msg_ptr));
		return -EINVAL;
	}

	msg_ptr = data;
	if (msg_ptr->ret_code) {
		mmi_err(chg, "write ack, ret_code: %u\n", msg_ptr->ret_code);
		return -EINVAL;
	}

	mmi_dbg(chg, "write ack\n");
	complete(&chg->write_ack);

	return 0;
}

static int oem_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct qti_charger *chg = priv;

	mmi_dbg(chg, "owner: %u type: %u opcode: 0x%x len:%zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (hdr->opcode == OEM_READ_BUF_REQ)
		handle_oem_read_ack(chg, data, len);
	else if (hdr->opcode == OEM_WRITE_BUF_REQ)
		handle_oem_write_ack(chg, data, len);
	else
		mmi_err(chg, "Unknown message opcode: %d\n", hdr->opcode);

	return 0;
}

static void oem_state_cb(void *priv, enum pmic_glink_state state)
{
	struct qti_charger *chg = priv;

	mmi_dbg(chg, "state: %d\n", state);

	atomic_set(&chg->state, state);

	switch (state) {
	case PMIC_GLINK_STATE_DOWN:
	case PMIC_GLINK_STATE_UP:
		schedule_work(&chg->setup_work);
		break;
	default:
		break;
	}
}

static int qti_charger_write(struct qti_charger *chg, u32 property,
			       const void *val, size_t val_len)
{
	struct oem_write_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		mmi_err(chg, "Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN)
		return 0;

	memset(&oem_buf, 0, sizeof(oem_buf));
	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_WRITE_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;
	memcpy(oem_buf.buf, val, val_len);

	mutex_lock(&chg->write_lock);
	reinit_completion(&chg->write_ack);

	mmi_dbg(chg, "Start data write for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chg->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		mmi_err(chg, "Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chg->write_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		mmi_err(chg, "timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
	}
out:
	mmi_dbg(chg, "Complete data write for property: %u\n", property);
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int qti_charger_read(struct qti_charger *chg, u32 property,
			       void *val, size_t val_len)
{
	struct oem_read_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		mmi_err(chg, "Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN)
		return 0;

	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_READ_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;

	mutex_lock(&chg->read_lock);
	reinit_completion(&chg->read_ack);

	mmi_dbg(chg, "Start data read for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chg->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		mmi_err(chg, "Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chg->read_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		mmi_err(chg, "timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
	}

	if (!atomic_read(&chg->rx_valid)) {
		rc = -ENODATA;
		goto out;
	}

	memcpy(val, chg->rx_buf.buf, val_len);
	atomic_set(&chg->rx_valid, 0);
out:
	mmi_dbg(chg, "Complete data read for property: %u\n", property);
	mutex_unlock(&chg->read_lock);

	return rc;
}

static int qti_charger_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
	int rc;
	struct qti_charger *chg = data;
	int batt_status = chg->batt_info.batt_status;

	rc = qti_charger_read(chg, OEM_PROP_BATT_INFO,
				&chg->batt_info,
				sizeof(struct battery_info));
	if (rc)
		return rc;

	if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

	chg->batt_info.batt_ma /= 1000;
	chg->batt_info.batt_mv /= 1000;
        chg->batt_info.batt_soc /= 100;
	chg->batt_info.batt_temp /= 100;
	memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));

	if (batt_status != chg->batt_info.batt_status) {
		bm_ulog_print_log(OEM_BM_ULOG_SIZE);
	}

	return rc;
}

static int qti_charger_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
	int rc;
	struct qti_charger *chg = data;
	int chrg_type = chg->chg_info.chrg_type;
	int chrg_pmax_mw = chg->chg_info.chrg_pmax_mw;
	int chrg_present = chg->chg_info.chrg_present;

	rc = qti_charger_read(chg, OEM_PROP_CHG_INFO,
				&chg->chg_info,
				sizeof(struct charger_info));
	if (rc)
		return rc;

	chg->chg_info.chrg_mv /= 1000;
	chg->chg_info.chrg_ma /= 1000;
	if (!chg->chg_info.chrg_present &&
	    chg->chg_info.chrg_type != 0)
		chg->chg_info.chrg_present = 1;
	memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));

	if (chrg_type != chg->chg_info.chrg_type ||
	    chrg_present != chg->chg_info.chrg_present ||
	    chrg_pmax_mw != chg->chg_info.chrg_pmax_mw) {
		bm_ulog_print_log(OEM_BM_ULOG_SIZE);
	}

	return rc;
}

static int qti_charger_config_charge(void *data, struct mmi_charger_cfg *config)
{
	int rc;
	u32 value;
	struct qti_charger *chg = data;

	/* configure the charger if changed */
	if (config->target_fv != chg->chg_cfg.target_fv) {
		value = config->target_fv * 1000;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FV,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.target_fv = config->target_fv;
	}
	if (config->target_fcc != chg->chg_cfg.target_fcc) {
		value = config->target_fcc * 1000;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FCC,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.target_fcc = config->target_fcc;
	}
	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		value = config->charger_suspend;
		rc = qti_charger_write(chg, OEM_PROP_CHG_SUSPEND,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charger_suspend = config->charger_suspend;
	}
	if (config->charging_disable != chg->chg_cfg.charging_disable) {
		value = config->charging_disable;
		rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charging_disable = config->charging_disable;
	}

	if (config->taper_kickoff != chg->chg_cfg.taper_kickoff) {
		chg->chg_cfg.taper_kickoff = config->taper_kickoff;
		chg->chrg_taper_cnt = 0;
	}

	if (config->full_charged != chg->chg_cfg.full_charged) {
		chg->chg_cfg.full_charged = config->full_charged;
	}

	if (config->chrg_iterm != chg->chg_cfg.chrg_iterm) {
		value = config->chrg_iterm;
		rc = qti_charger_write(chg, OEM_PROP_CHG_ITERM,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.chrg_iterm = config->chrg_iterm;
	}
	if (config->fg_iterm != chg->chg_cfg.fg_iterm) {
		value = config->fg_iterm;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FG_ITERM,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.fg_iterm = config->fg_iterm;
	}

	if (config->charging_reset != chg->chg_cfg.charging_reset) {
		if (config->charging_reset) {
			value = 1;
			rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
						&value,
						sizeof(value));
			msleep(200);
			value = 0;
			rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
						&value,
						sizeof(value));
		}
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	return 0;
}

#define TAPER_COUNT 2
static bool qti_charger_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct qti_charger *chg = data;

	if (abs(chg->batt_info.batt_ma) <= tapered_ma) {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	} else
		chg->chrg_taper_cnt = 0;

	return is_tapered;
}

static bool qti_charger_is_charge_halt(void *data)
{
	struct qti_charger *chg = data;

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		return true;

	return false;
}

static void qti_charger_set_constraint(void *data,
			struct mmi_charger_constraint *constraint)
{
	int rc;
	u32 value;
	struct qti_charger *chg = data;

	if (constraint->demo_mode != chg->constraint.demo_mode) {
		value = constraint->demo_mode;
		rc = qti_charger_write(chg, OEM_PROP_DEMO_MODE,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.demo_mode = constraint->demo_mode;
	}

	if (constraint->factory_version != chg->constraint.factory_version) {
		value = constraint->factory_version;
		rc = qti_charger_write(chg, OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_version = constraint->factory_version;
	}

	if (constraint->factory_mode != chg->constraint.factory_mode) {
		value = constraint->factory_mode;
		rc = qti_charger_write(chg, OEM_PROP_FACTORY_MODE,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_mode = constraint->factory_mode;
	}

	if (constraint->dcp_pmax != chg->constraint.dcp_pmax) {
		value = constraint->dcp_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_BC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.dcp_pmax = constraint->dcp_pmax;
	}

	if (constraint->hvdcp_pmax != chg->constraint.hvdcp_pmax) {
		value = constraint->hvdcp_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_QC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.hvdcp_pmax = constraint->hvdcp_pmax;
	}

	if (constraint->pd_pmax != chg->constraint.pd_pmax) {
		value = constraint->pd_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_PD_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.pd_pmax = constraint->pd_pmax;
	}

	if (constraint->wls_pmax != chg->constraint.wls_pmax) {
		value = constraint->wls_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_WLS_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.wls_pmax = constraint->wls_pmax;
	}
}

static int qti_charger_write_profile(struct qti_charger *chg)
{
	int rc;
	char *data;
	int offset;
	int max_size = OEM_PROPERTY_DATA_SIZE * sizeof(u32);

	rc = qti_charger_write(chg, OEM_PROP_CHG_PROFILE_INFO,
				&chg->profile_info,
				sizeof(struct charger_profile_info));
	if (rc) {
		mmi_err(chg, "qti charger write profile info failed, rc=%d\n", rc);
		return rc;
	}

	if (!chg->profile_data)
		return 0;

	offset = 0;
	data = (char *)chg->profile_data;
	while (offset < chg->profile_info.data_size) {
		if ((chg->profile_info.data_size - offset) > max_size) {
			rc = qti_charger_write(chg,
					OEM_PROP_CHG_PROFILE_DATA,
					data + offset,
					max_size);
			offset += max_size;
		} else {
			rc = qti_charger_write(chg,
					OEM_PROP_CHG_PROFILE_DATA,
					data + offset,
					chg->profile_info.data_size - offset);
			offset += chg->profile_info.data_size - offset;
		}
		if (rc) {
			mmi_err(chg, "qti charger write profile data failed, rc=%d\n", rc);
			break;
		}
	}

	return rc;
}

static ssize_t tcmd_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", mode);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_TCMD,
				&mode,
				sizeof(mode));

	return r ? r : count;
}

static ssize_t tcmd_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_TCMD,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}


static DEVICE_ATTR(tcmd, 0664,
		tcmd_show,
		tcmd_store);

static ssize_t force_pmic_icl_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmic_icl;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmic_icl);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", pmic_icl);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_PMIC_ICL,
				&pmic_icl,
				sizeof(pmic_icl));

	return r ? r : count;
}

static ssize_t force_pmic_icl_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_PMIC_ICL,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}


static DEVICE_ATTR(force_pmic_icl, 0664,
		force_pmic_icl_show,
		force_pmic_icl_store);

static ssize_t addr_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	u16 addr;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou16(buf, 0, &addr);
	if (r) {
		mmi_err(chg, "Invalid reg_address = 0x%x\n", addr);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_REG_ADDRESS,
				&addr,
				sizeof(addr));

	return r ? r : count;
}

static DEVICE_ATTR_WO(addr);

static ssize_t data_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	u8 data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou8(buf, 0, &data);
	if (r) {
		mmi_err(chg, "Invalid reg_data = 0x%x\n", data);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_REG_DATA,
				&data,
				sizeof(data));

	return r ? r : count;
}

static ssize_t data_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u8 data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_REG_DATA,
				&data,
				sizeof(data));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%x\n", data);
}

static DEVICE_ATTR(data, 0664,
		data_show,
		data_store);

static int qti_charger_init(struct qti_charger *chg)
{
	int rc;
	u32 value;
	struct mmi_charger_driver *driver;

	if (chg->driver) {
		mmi_warn(chg, "qti charger has already inited\n");
		return 0;
	}

	chg->constraint.factory_mode = mmi_is_factory_mode();
	value = chg->constraint.factory_mode;
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_MODE,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory mode failed, rc=%d\n", rc);
		return rc;
	}

	chg->constraint.factory_version = mmi_is_factory_version();
	value = chg->constraint.factory_version;
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory ver failed, rc=%d\n", rc);
		return rc;
	}

	rc = qti_charger_write_profile(chg);
	if (rc) {
		mmi_err(chg, "qti charger set profile data failed, rc=%d\n", rc);
		return rc;
	}

	driver = devm_kzalloc(chg->dev,
				sizeof(struct mmi_charger_driver),
				GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = chg->name;
	driver->dev = chg->dev;
	driver->data = chg;
	driver->get_batt_info = qti_charger_get_batt_info;
	driver->get_chg_info = qti_charger_get_chg_info;
	driver->config_charge = qti_charger_config_charge;
	driver->is_charge_tapered = qti_charger_is_charge_tapered;
	driver->is_charge_halt = qti_charger_is_charge_halt;
	driver->set_constraint = qti_charger_set_constraint;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		mmi_err(chg, "qti charger init failed, rc=%d\n", rc);
	} else {
		mmi_info(chg, "qti charger init successfully\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_tcmd);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create tcmd\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_pmic_icl);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_pmic_icl\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_addr);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create addr\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_data);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create data\n");
	}

	bm_ulog_print_mask_log(BM_ALL, BM_LOG_LEVEL_INFO, OEM_BM_ULOG_SIZE);

	return 0;
}

static void qti_charger_deinit(struct qti_charger *chg)
{
	int rc;

	if (!chg->driver) {
		mmi_info(chg, "qti charger has not inited yet\n");
		return;
	}

	device_remove_file(chg->dev, &dev_attr_tcmd);
	device_remove_file(chg->dev, &dev_attr_force_pmic_icl);

	device_remove_file(chg->dev, &dev_attr_addr);

	device_remove_file(chg->dev, &dev_attr_data);

	/* unregister driver from mmi charger */
	rc = mmi_unregister_charger_driver(chg->driver);
	if (rc) {
		mmi_err(chg, "qti charger deinit failed, rc=%d\n", rc);
	} else {
		devm_kfree(chg->dev, chg->driver);
		chg->driver = NULL;
	}
}

static void qti_charger_setup_work(struct work_struct *work)
{
	struct qti_charger *chg = container_of(work,
				struct qti_charger, setup_work);
	enum pmic_glink_state state;

	state = atomic_read(&chg->state);
	if (state == PMIC_GLINK_STATE_UP) {
		qti_charger_init(chg);
	} else if (state == PMIC_GLINK_STATE_DOWN) {
		qti_charger_deinit(chg);
	}
}

static int qti_charger_parse_dt(struct qti_charger *chg)
{
	int rc;
	int i, j;
	int n = 0;
	int bk_num;
	int bk_size;
	char bk_buf[128];
	int byte_len;
	const char *df_sn = NULL, *dev_sn = NULL;
	struct device_node *node;

	node = chg->dev->of_node;
	dev_sn = mmi_get_battery_serialnumber();
	if (!dev_sn) {
		rc = of_property_read_string(node, "mmi,df-serialnum",
						&df_sn);
		if (!rc && df_sn) {
			mmi_info(chg, "Default Serial Number %s\n", df_sn);
		} else {
			mmi_err(chg, "No Default Serial Number defined\n");
			df_sn = BATT_SN_UNKNOWN;
		}
		strcpy(chg->batt_info.batt_sn, df_sn);
	} else {
		strcpy(chg->batt_info.batt_sn, dev_sn);
	}

	chg->profile_info.profile_id = find_profile_id(chg);
	if (chg->profile_info.profile_id < 0)
		chg->profile_info.profile_id = BATT_DEFAULT_ID;

	rc = of_property_read_u32(node, "mmi,chrg-iterm-ma",
				  &chg->profile_info.chrg_iterm);
	if (rc) {
		chg->profile_info.chrg_iterm = 300000;
	} else {
		chg->profile_info.chrg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,fg-iterm-ma",
				  &chg->profile_info.fg_iterm);
	if (rc) {
		chg->profile_info.fg_iterm =
			chg->profile_info.chrg_iterm + 50000;
	} else {
		chg->profile_info.fg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,vfloat-comp-uv",
				  &chg->profile_info.vfloat_comp_uv);
	if (rc)
		chg->profile_info.vfloat_comp_uv = 0;

	rc = of_property_read_u32(node, "mmi,max-fv-mv",
				  &chg->profile_info.max_fv_uv);
	if (rc)
		chg->profile_info.max_fv_uv = 4400;
	chg->profile_info.max_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,max-fcc-ma",
				  &chg->profile_info.max_fcc_ua);
	if (rc)
		chg->profile_info.max_fcc_ua = 4000;
	chg->profile_info.max_fcc_ua *= 1000;

	rc = of_property_read_u32(node, "mmi,demo-fv-mv",
				  &chg->profile_info.demo_fv_uv);
	if (rc)
		chg->profile_info.demo_fv_uv = 4000;
	chg->profile_info.demo_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,profile-data-block-size",
				  &chg->profile_info.data_bk_size);
	if (rc)
		chg->profile_info.data_bk_size = 4;
	chg->profile_info.data_bk_size *= 4;

	chg->profile_info.data_size = 0;
	if (of_find_property(node, "mmi,profile-data", &byte_len)) {
		if (byte_len % chg->profile_info.data_bk_size) {
			mmi_err(chg, "DT error wrong profile data\n");
			chg->profile_info.data_bk_size = 0;
			return -ENODEV;
		}
		bk_num = byte_len / chg->profile_info.data_bk_size;
		chg->profile_data = (u32 *)devm_kzalloc(chg->dev, byte_len,
							GFP_KERNEL);
		if (chg->profile_data == NULL) {
			chg->profile_info.data_bk_size = 0;
			return -ENOMEM;
		}

		rc = of_property_read_u32_array(node,
				"mmi,profile-data",
				chg->profile_data,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_err(chg, "Couldn't read profile data, rc = %d\n", rc);
			devm_kfree(chg->dev, chg->profile_data);
			chg->profile_data = NULL;
			chg->profile_info.data_bk_size = 0;
			return rc;
		}

		chg->profile_info.data_size = byte_len;
		mmi_info(chg, "profile data: block size: %d, num: %d\n",
				chg->profile_info.data_bk_size, bk_num);
		bk_size = chg->profile_info.data_bk_size / 4;
		for (i = 0; i < bk_num; i++) {
			memset(bk_buf, '\0', sizeof(bk_buf));
			n = sprintf(bk_buf, "block%d:", i);
			for (j = 0; j < bk_size; j++) {
				n += sprintf(bk_buf + n, " %d",
					chg->profile_data[i * bk_size + j]);
			}
			mmi_info(chg, "%s\n", bk_buf);
		}
	}

	return 0;
}

static int qti_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data;
	struct qti_charger *chg;
	int rc;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	INIT_WORK(&chg->setup_work, qti_charger_setup_work);
	mutex_init(&chg->read_lock);
	mutex_init(&chg->write_lock);
	init_completion(&chg->read_ack);
	init_completion(&chg->write_ack);
	atomic_set(&chg->rx_valid, 0);
	atomic_set(&chg->state, PMIC_GLINK_STATE_UP);
	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->name = "qti_glink_charger";

	chg->debug_enabled = &debug_enabled;
	chg->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chg->ipc_log)
		mmi_warn(chg, "Error in creating ipc_log_context\n");

	rc = qti_charger_parse_dt(chg);
	if (rc) {
		mmi_err(chg, "dt paser failed, rc=%d\n", rc);
		return rc;
	}

	client_data.id = MSG_OWNER_OEM;
	client_data.name = "oem";
	client_data.msg_cb = oem_callback;
	client_data.priv = chg;
	client_data.state_cb = oem_state_cb;

	chg->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(chg->client)) {
		rc = PTR_ERR(chg->client);
		if (rc != -EPROBE_DEFER)
			mmi_err(chg, "Error in registering with pmic_glink rc=%d\n",
				rc);
		return rc;
	}

	qti_charger_init(chg);

	return 0;
}

static int qti_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qti_charger *chg= dev_get_drvdata(dev);
	int rc;

	qti_charger_deinit(chg);

	rc = pmic_glink_unregister_client(chg->client);
	if (rc < 0)
		mmi_err(chg, "pmic_glink_unregister_client failed rc=%d\n",
			rc);

	return rc;
}

static const struct of_device_id qti_charger_match_table[] = {
	{.compatible = "mmi,qti-glink-charger"},
	{},
};

static struct platform_driver qti_charger_driver = {
	.driver	= {
		.name = "qti_glink_charger",
		.of_match_table = qti_charger_match_table,
	},
	.probe	= qti_charger_probe,
	.remove	= qti_charger_remove,
};

module_platform_driver(qti_charger_driver);

MODULE_DESCRIPTION("QTI Glink Charger Driver");
MODULE_LICENSE("GPL v2");
