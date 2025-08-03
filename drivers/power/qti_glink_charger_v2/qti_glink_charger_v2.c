/*
 * Copyright (C) 2020-2021 Motorola Mobility LLC
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
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,6,0)
#include <linux/soc/qcom/qti_pmic_glink.h>
#else
#include <linux/soc/qcom/pmic_glink.h>
#endif

#include <linux/power/bm_adsp_ulog.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <soc/qcom/mmi_boot_info.h>

#include <linux/power/qti_glink_charger_v2.h>

/* PPM specific definitions */
#define MSG_OWNER_OEM			32782
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2
#define OEM_PROPERTY_DATA_SIZE		16

#define OEM_READ_BUF_REQ		0x0101
#define OEM_WRITE_BUF_REQ		0x0102
#define OEM_NOTIFY_IND			0x0103
#define OEM_SEND_RECE_BUF		0x0104

#define OEM_WAIT_TIME_MS		5000

#define OEM_BM_ULOG_SIZE		4096

#define VBUS_MIN_MV			4000

#define FOD_GAIN_MAX_LEN 16
#define FOD_CURR_MAX_LEN 7

#define RADIO_MAX_LEN 33
#define CHG_SHOW_MAX_SIZE 50
#define MMI_LOG_PAGES (50)
#define MMI_LOG_DIR "qti_glink_charger"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for qti glink charger driver");

#define mmi_err(chg, fmt, ...)			\
	do {						\
		pr_err("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"E %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_warn(chg, fmt, ...)			\
	do {						\
		pr_warn("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"W %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_info(chg, fmt, ...)			\
	do {						\
		pr_info("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"I %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_dbg(chg, fmt, ...)			\
	do {							\
		if (*chg->debug_enabled)		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
			"D %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

struct oem_notify_ind_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
	u32			receiver;
	u32			data[MAX_OEM_NOTIFY_DATA_LEN];
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

struct usb_info {
	int cid_st;
	unsigned int vbus_st;
	bool otg_st;
	bool cc_st;
	int partner_type;
	int pd_active;
	int legacy_cable;
	int lpd_st;
	int lpd_rsbu1;
	int lpd_rsbu2;
	int lpd_cc1;
	int lpd_cc2;
	int lpd_dp;
	int lpd_dm;
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
	struct work_struct		notify_work;
	struct oem_notify_ind_msg	notify_msg;
	atomic_t			state;


	int demo_mode;
	bool factory_mode;
	bool factory_version;

	struct usb_info			usb_info;
	void				*ipc_log;
	bool				*debug_enabled;
};

static struct qti_charger *this_chip = NULL;
static BLOCKING_NOTIFIER_HEAD(qti_chg_notifier_list);

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

static int handle_oem_notification(struct qti_charger *chg, void *data, size_t len)
{
	struct oem_notify_ind_msg *notify_msg = data;
	if (len != sizeof(*notify_msg)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(*notify_msg));
		return -EINVAL;
	}

	mmi_info(chg, "notification: %#x on receiver: %#x\n",
				notify_msg->notification,
				notify_msg->receiver);

	pm_stay_awake(chg->dev);
	memcpy(&chg->notify_msg, notify_msg, sizeof(*notify_msg));
	schedule_work(&chg->notify_work);

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
	else if (hdr->opcode == OEM_NOTIFY_IND)
		handle_oem_notification(chg, data, len);
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

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
		return -ENOTCONN;
	}

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
		bm_ulog_print_log(OEM_BM_ULOG_SIZE);
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

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
		return -ENOTCONN;
	}

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

	if (chg->rx_buf.data_size != val_len) {
		mmi_dbg(chg, "Invalid data size %u, on property: %u\n",
				chg->rx_buf.data_size, property);
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

int qti_charger_set_property(u32 property, const void *val, size_t val_len)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return qti_charger_write(chg, property, val, val_len);
}
EXPORT_SYMBOL(qti_charger_set_property);

int qti_charger_get_property(u32 property, void *val, size_t val_len)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return qti_charger_read(chg, property, val, val_len);
}
EXPORT_SYMBOL(qti_charger_get_property);

int qti_charger_register_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&qti_chg_notifier_list, nb);
}
EXPORT_SYMBOL(qti_charger_register_notifier);

int qti_charger_unregister_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_unregister(&qti_chg_notifier_list, nb);
}
EXPORT_SYMBOL(qti_charger_unregister_notifier);

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

static ssize_t force_wls_en_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_en;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_en);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_en);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_EN,
				&wls_en,
				sizeof(wls_en));

	return r ? r : count;
}

static ssize_t force_wls_en_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_EN,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_en, 0664,
		force_wls_en_show,
		force_wls_en_store);

static ssize_t force_usb_suspend_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long usb_suspend;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &usb_suspend);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", usb_suspend);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_USB_SUSPEND,
				&usb_suspend,
				sizeof(usb_suspend));

	return r ? r : count;
}

static ssize_t force_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_USB_SUSPEND,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_usb_suspend, 0664,
		force_usb_suspend_show,
		force_usb_suspend_store);

static ssize_t force_wls_volt_max_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_volt_max;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_volt_max);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_volt_max);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_VOLT_MAX,
				&wls_volt_max,
				sizeof(wls_volt_max));

	return r ? r : count;
}

static ssize_t force_wls_volt_max_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_VOLT_MAX,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_volt_max, 0664,
		force_wls_volt_max_show,
		force_wls_volt_max_store);

static ssize_t force_wls_curr_max_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_curr_max;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_curr_max);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_curr_max);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_CURR_MAX,
				&wls_curr_max,
				sizeof(wls_curr_max));

	return r ? r : count;
}

static ssize_t force_wls_curr_max_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_CURR_MAX,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_curr_max, 0664,
		force_wls_curr_max_show,
		force_wls_curr_max_store);

static ssize_t wireless_chip_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_CHIP_ID,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%04x\n", data);
}

static DEVICE_ATTR(wireless_chip_id, S_IRUGO,
		wireless_chip_id_show,
		NULL);

static ssize_t wireless_fw_ver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_FW_VER,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%04x\n", data);
}

static DEVICE_ATTR(wireless_fw_ver, S_IRUGO,
		wireless_fw_ver_show,
		NULL);

static ssize_t tcmd_current_battid_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	char batt_id[32]={0};
	int rc;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	rc = qti_charger_read(chg, OEM_PROP_TCMD_CURRENT_BATTID,
			batt_id, sizeof(batt_id));
	if (rc) {
		pr_err("QTI: qti read current battid failed, rc = %d\n", rc);
		return rc;
	}
	batt_id[sizeof(batt_id) - 1] = '\0';
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n", batt_id);
}
static DEVICE_ATTR(tcmd_current_battid, S_IRUGO, tcmd_current_battid_show, NULL);

static ssize_t tcmd_current_flip_battid_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        char batt_id[32]={0};
        int rc;
        struct qti_charger *chg = dev_get_drvdata(dev);

        if (!chg) {
                pr_err("QTI: chip not valid\n");
                return -ENODEV;
        }

        rc = qti_charger_read(chg, OEM_PROP_TCMD_CURRENT_FLIP_BATTID,
                        batt_id, sizeof(batt_id));
        if (rc) {
                pr_err("QTI: qti read current flip battid failed, rc = %d\n", rc);
		return rc;
        }
        batt_id[sizeof(batt_id) - 1] = '\0';
        return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n", batt_id);
}
static DEVICE_ATTR(tcmd_current_flip_battid, S_IRUGO, tcmd_current_flip_battid_show, NULL);

static ssize_t addr_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	u32 addr;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou32(buf, 0, &addr);
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
	u32 data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou32(buf, 0, &data);
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
	u32 data;
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

static ssize_t cid_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int rc;
	struct qti_charger *chg = this_chip;
	struct usb_info usb_info = {0};

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}
	chg->usb_info.cid_st = -1;

	rc = qti_charger_read(chg, OEM_PROP_USB_INFO,
			&usb_info, sizeof(usb_info));
	if (!rc) {
		chg->usb_info = usb_info;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->usb_info.cid_st);
}
static DEVICE_ATTR(cid_status, S_IRUGO, cid_status_show, NULL);

static ssize_t typec_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int r;
	unsigned int reset = 0;
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou32(buf, 0, &reset);
	if (r) {
		pr_err("Invalid typec_reset = %d\n", reset);
		return -EINVAL;
	}

	if (reset)
		mmi_warn(chg, "typec_reset triggered\n");
	else
		return count;

	r = qti_charger_write(chg, OEM_PROP_TYPEC_RESET,
			&reset,
			sizeof(reset));

	return r ? r : count;
}
static DEVICE_ATTR(typec_reset, S_IWUSR|S_IWGRP, NULL, typec_reset_store);

static ssize_t fg_operation_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long fg_operation_cmd;
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &fg_operation_cmd);
	if (r) {
		pr_err("Invalid fg_operation_cmd = %lu\n", fg_operation_cmd);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_FG_OPERATION,
			&fg_operation_cmd,
			sizeof(fg_operation_cmd));

	return r ? r : count;
}
static DEVICE_ATTR(fg_operation, S_IWUSR|S_IWGRP, NULL, fg_operation_store);

static char *bootargs_str = NULL;
static size_t bootargs_str_len = 0;
static int mmi_get_bootarg_dt(char *key, char **value, char *prop, char *spl_flag)
{
	const char *bootargs_tmp = NULL;
	char *idx = NULL;
	char *kvpair = NULL;
	int err = 1;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_tmp_len = 0;

	if (n == NULL)
		goto err;

	if (of_property_read_string(n, prop, &bootargs_tmp) != 0)
		goto putnode;

	bootargs_tmp_len = strlen(bootargs_tmp);
	if (bootargs_tmp_len >= bootargs_str_len) {
		if (bootargs_str)
			kfree(bootargs_str);
		bootargs_str = kzalloc(bootargs_tmp_len + 1, GFP_KERNEL);
		if (!bootargs_str) {
			err = -ENOMEM;
			goto putnode;
		}
		bootargs_str_len = bootargs_tmp_len + 1;
	} else {
		memset(bootargs_str, '\0', bootargs_str_len);
	}

	strscpy(bootargs_str, bootargs_tmp, bootargs_tmp_len + 1);

	idx = strnstr(bootargs_str, key, strlen(bootargs_str));
	if (idx) {
		kvpair = strsep(&idx, " ");
		if (kvpair)
			if (strsep(&kvpair, "=")) {
				*value = strsep(&kvpair, spl_flag);
				if (*value)
					err = 0;
			}
	}

putnode:
	of_node_put(n);
err:
	return err;
}

static int mmi_get_bootarg(char *key, char **value)
{
#ifdef CONFIG_BOOT_CONFIG
	return mmi_get_bootarg_dt(key, value, "mmi,bootconfig", "\n");
#else
	return mmi_get_bootarg_dt(key, value, "bootargs", " ");
#endif
}

static int mmi_get_sku_type(struct qti_charger *chg, u8 *sku_type)
{
	char *s = NULL;
	char androidboot_radio_str[RADIO_MAX_LEN];

	if (mmi_get_bootarg("androidboot.radio=", &s) == 0) {
		if (s != NULL) {
			strscpy(androidboot_radio_str, s, RADIO_MAX_LEN);
			if (!strncmp("PRC", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_PRC;
			} else if (!strncmp("ROW", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_ROW;
			} else if (!strncmp("NA", androidboot_radio_str, 2)) {
				*sku_type = MMI_CHARGER_SKU_NA;
			} else if (!strncmp("VZW", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_VZW;
			} else if (!strncmp("JPN", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_JPN;
			} else if (!strncmp("ITA", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_ITA;
			} else if (!strncmp("NAE", androidboot_radio_str, 3)) {
				*sku_type = MMI_CHARGER_SKU_NAE;
			} else if (!strncmp("SUPERSET", androidboot_radio_str, 8)) {
				*sku_type = MMI_CHARGER_SKU_SUPERSET;
			} else {
				*sku_type = 0;
			}
			mmi_info(chg, "SKU type: %s, 0x%02x\n", androidboot_radio_str, *sku_type);
			return 0;
		} else {
			mmi_err(chg, "Could not get SKU type\n");
			return -1;
		}
	} else {
		mmi_err(chg, "Could not get radio bootarg\n");
		return -1;
	}
}

static int mmi_get_hw_revision(struct qti_charger *chg, u16 *hw_rev)
{
	char *s = NULL;
	char androidboot_hwrev_str[RADIO_MAX_LEN];
	int ret;

	if (mmi_get_bootarg("androidboot.hwrev=", &s) == 0) {
		if (s != NULL) {
			strscpy(androidboot_hwrev_str, s, RADIO_MAX_LEN);
			ret = kstrtou16(androidboot_hwrev_str, 16, hw_rev);
			if (ret < 0) {
				mmi_info(chg, "kstrtou16 error: %d \n", ret);
				return -1;
			}
			mmi_info(chg, "HW revision: 0x%x\n", *hw_rev);
			return 0;
		} else {
			mmi_err(chg, "Could not get HW  revision\n");
			return -1;
		}
	} else {
		mmi_err(chg, "Could not get hwrev bootarg\n");
		return -1;
	}
}

#if 0
/*************************
 * USB   COOLER   START  *
 *************************/
static bool mmi_is_softbank_sku(struct qti_charger *chg)
{
	char *s = NULL;
	bool is_softbank = false;
	char androidboot_carrier_str[RADIO_MAX_LEN];

	if (mmi_get_bootarg("androidboot.carrier=", &s) == 0) {
		mmi_info(chg, "Get bootarg androidboot.hardware.sku success");
		if (s != NULL) {
			strscpy(androidboot_carrier_str, s, RADIO_MAX_LEN);
			mmi_info(chg, "carrier: %s", androidboot_carrier_str);
			if (!strncmp("softbank", androidboot_carrier_str, 8)) {
				is_softbank = true;
			}
		}
	}
	return is_softbank;
}
#endif

static bool mmi_is_factory_mode(void)
{
	char *mode = NULL;

	if ((this_chip && this_chip->factory_mode) ||
	    !strncmp(bi_bootmode(), "mot-factory", 11))
		return true;

	if (mmi_get_bootarg("androidboot.mode=", &mode) ||
		!mode || strncmp("mot-factory", mode, 11)) {
		return false;
	}
	return true;
}

static bool mmi_is_factory_version(void)
{
	char *bootloader = NULL;

	if (this_chip && this_chip->factory_version)
		return true;

	if (mmi_get_bootarg("androidboot.bootloader=", &bootloader) ||
		!bootloader || !strnstr(bootloader, "factory", strlen(bootloader))) {
		return false;
	}
	return true;
}

static int qti_charger_parameters_init(struct qti_charger *chg)
{
	int rc;
	u32 value;
	u8 sku_type = 0;
	u16 hw_rev = 0;

	value = mmi_is_factory_mode();
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_MODE,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory mode failed, rc=%d\n", rc);
	}
	chg->factory_mode = value;

	value = mmi_is_factory_version();
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory ver failed, rc=%d\n", rc);
	}
	chg->factory_version = value;

	//set SKU type
	if ((rc = mmi_get_sku_type(chg, &sku_type)) == 0) {
		rc = qti_charger_write(chg, OEM_PROP_SKU_TYPE,
						&sku_type,
						sizeof(sku_type));
		if (rc) {
			mmi_err(chg, "qti charger set SKU type failed, rc=%d\n", rc);
		}
	} else {
		mmi_err(chg, "Fail to get sku type\n");
	}
	//set HW revision
	if ((rc = mmi_get_hw_revision(chg, &hw_rev)) == 0) {
		rc = qti_charger_write(chg, OEM_PROP_HW_REVISION,
						&hw_rev,
						sizeof(hw_rev));
		if (rc) {
			mmi_err(chg, "qti charger set HW revision failed, rc=%d\n", rc);
		}
	} else {
		mmi_err(chg, "Fail to get HW revision\n");
	}

	chg->usb_info.cid_st = -1;

	return rc;
}

static int qti_charger_init(struct qti_charger *chg)
{
	int rc;

	rc = qti_charger_parameters_init(chg);
	if (rc) {
		mmi_err(chg,
			   "qti_charger_parameters_init failed\n");
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
				&dev_attr_force_wls_en);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_en\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_usb_suspend);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_usb_suspend\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_wls_volt_max);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_volt_max\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_wls_curr_max);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_curr_max\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_wireless_chip_id);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create wireless_chip_id\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_wireless_fw_ver);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create wireless_fw_ver\n");
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

	rc = device_create_file(chg->dev,
			&dev_attr_tcmd_current_battid);
        if (rc) {
                mmi_err(chg,
                           "Couldn't create tcmd_current_battid\n");
        }

	rc = device_create_file(chg->dev,
                        &dev_attr_tcmd_current_flip_battid);
        if (rc) {
                mmi_err(chg,
                           "Couldn't create tcmd_current_flip_battid\n");
        }

	rc = device_create_file(chg->dev,
				&dev_attr_cid_status);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create cid_status\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_typec_reset);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create typec_reset\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_fg_operation);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create fg_operation\n");
	}

	bm_ulog_print_mask_log(BM_ALL, BM_LOG_LEVEL_INFO, OEM_BM_ULOG_SIZE);
	mmi_err(chg, "qti_charger_init complete\n");
	return 0;
}

static void qti_charger_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qti_charger *chg= dev_get_drvdata(dev);

	mmi_info(chg, "qti_charger_shutdown\n");

	return;
}

static void qti_charger_deinit(struct qti_charger *chg)
{
	device_remove_file(chg->dev, &dev_attr_fg_operation);
	device_remove_file(chg->dev, &dev_attr_typec_reset);
	device_remove_file(chg->dev, &dev_attr_cid_status);
	device_remove_file(chg->dev, &dev_attr_tcmd_current_battid);
	device_remove_file(chg->dev, &dev_attr_tcmd_current_flip_battid);
	device_remove_file(chg->dev, &dev_attr_tcmd);
	device_remove_file(chg->dev, &dev_attr_force_pmic_icl);
	device_remove_file(chg->dev, &dev_attr_force_wls_en);
	device_remove_file(chg->dev, &dev_attr_force_usb_suspend);
	device_remove_file(chg->dev, &dev_attr_force_wls_volt_max);
	device_remove_file(chg->dev, &dev_attr_force_wls_curr_max);
	device_remove_file(chg->dev, &dev_attr_wireless_chip_id);
	device_remove_file(chg->dev, &dev_attr_addr);
	device_remove_file(chg->dev, &dev_attr_data);
}

static void qti_charger_setup_work(struct work_struct *work)
{
	struct qti_charger *chg = container_of(work,
				struct qti_charger, setup_work);
	enum pmic_glink_state state;
	unsigned long notification;
	struct qti_charger_notify_data notify_data;

	state = atomic_read(&chg->state);
	if (state == PMIC_GLINK_STATE_UP) {
		mmi_info(chg, "ADSP glink state is up\n");
		qti_charger_parameters_init(chg);
		notification = PMIC_GLINK_STATE_UP;
		notify_data.receiver = OEM_NOTIFY_RECEIVER_MMI_CHG;
		blocking_notifier_call_chain(&qti_chg_notifier_list,
				notification,
				&notify_data);
	} else if (state == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
	}
}

static void qti_charger_notify_work(struct work_struct *work)
{
	unsigned long notification;
	struct qti_charger_notify_data notify_data;
	struct qti_charger *chg = container_of(work,
				struct qti_charger, notify_work);

	notification = chg->notify_msg.notification;
	notify_data.receiver = chg->notify_msg.receiver;
	memcpy(notify_data.data, chg->notify_msg.data,
				sizeof(u32) * MAX_OEM_NOTIFY_DATA_LEN);
	blocking_notifier_call_chain(&qti_chg_notifier_list,
				notification,
				&notify_data);
	pm_relax(chg->dev);
}

static int qti_charger_parse_dt(struct qti_charger *chg)
{

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
	INIT_WORK(&chg->notify_work, qti_charger_notify_work);
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

	this_chip = chg;
	device_init_wakeup(chg->dev, true);
	qti_charger_init(chg);
	return 0;
}

static void qti_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qti_charger *chg= dev_get_drvdata(dev);
	int rc;

	qti_charger_deinit(chg);
	rc = pmic_glink_unregister_client(chg->client);
	if (rc < 0)
		mmi_err(chg, "pmic_glink_unregister_client failed rc=%d\n",
			rc);

	if (bootargs_str) {
		kfree(bootargs_str);
		bootargs_str = NULL;
		bootargs_str_len = 0;
	}

	return;
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
	.shutdown = qti_charger_shutdown,
};

module_platform_driver(qti_charger_driver);

MODULE_DESCRIPTION("QTI Glink Charger Driver");
MODULE_LICENSE("GPL v2");
