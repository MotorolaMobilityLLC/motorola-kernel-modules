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

#define pr_fmt(fmt)     "GLINK_CHG:CORE: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#if (KERNEL_VERSION(6, 6, 30) > LINUX_VERSION_CODE)
#include <linux/soc/qcom/pmic_glink.h>
#else
#include <linux/soc/qcom/qti_pmic_glink.h>
#endif
#include <linux/power/bm_adsp_ulog.h>

#include "mmi_charger.h"
#include "glink_device.h"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for adsp glink charger driver");

/* PPM specific definitions */
#define MSG_OWNER_OEM			32782
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2
#define OEM_PROPERTY_DATA_SIZE		16
#define OEM_READ_BUF_REQ		0x10000
#define OEM_WRITE_BUF_REQ		0x10001
#define OEM_NOTIFY_IND			0x10002
#define OEM_WAIT_TIME_MS		5000

enum oem_notify_transmitter {
	NOTIFY_TX_ADSP,
	NOTIFY_TX_PSY,
	NOTIFY_TX_NUM
};

struct oem_notify_ind_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
	u32			receiver;
	u32			data[MAX_GLINK_NOTIFY_DATA_LEN];
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

struct glink_chip {
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct completion		read_ack;
	struct completion		write_ack;
	struct mutex			read_lock;
	struct mutex			write_lock;
	struct oem_read_buf_resp_msg	rx_buf;
	atomic_t			rx_valid;
	struct work_struct		setup_work;
	struct delayed_work		notify_work;
	u32				polling_interval_ms;
	struct oem_notify_ind_msg	notify_msg[NOTIFY_TX_NUM];
	u32				notify_tx_mask;
	atomic_t			state;
	struct notifier_block		psy_notifier;
	struct glink_dev		*glink_dev_list;
	u32				glink_dev_count;
	u32				current_tcmd;
};

static struct glink_chip *this_chip = NULL;
static BLOCKING_NOTIFIER_HEAD(glink_device_notifier_list);

extern int glink_device_battery_setup(struct glink_dev *dev);
extern int glink_device_charger_setup(struct glink_dev *dev);
extern int glink_device_usb_setup(struct glink_dev *dev);
extern int glink_device_wls_setup(struct glink_dev *dev);
extern int glink_device_pump_setup(struct glink_dev *dev);
extern int glink_device_buck_setup(struct glink_dev *dev);
extern int glink_device_bcr_setup(struct glink_dev *dev);

static int handle_oem_read_ack(struct glink_chip *chip, void *data, size_t len)
{
	if (len != sizeof(chip->rx_buf)) {
		pr_err("Incorrect received length %zu expected %lu\n", len,
			sizeof(chip->rx_buf));
		atomic_set(&chip->rx_valid, 0);
		return -EINVAL;
	}

	memcpy(&chip->rx_buf, data, sizeof(chip->rx_buf));
	atomic_set(&chip->rx_valid, 1);
	complete(&chip->read_ack);
	pr_debug("read ack for property: %u\n", chip->rx_buf.oem_property_id);

	return 0;
}

static int handle_oem_write_ack(struct glink_chip *chip, void *data, size_t len)
{
	struct oem_write_buf_resp_msg *msg_ptr;

	if (len != sizeof(*msg_ptr)) {
		pr_err("Incorrect received length %zu expected %lu\n", len,
			sizeof(*msg_ptr));
		return -EINVAL;
	}

	msg_ptr = data;
	if (msg_ptr->ret_code) {
		pr_err("write ack, ret_code: %u\n", msg_ptr->ret_code);
		return -EINVAL;
	}

	pr_debug("write ack\n");
	complete(&chip->write_ack);

	return 0;
}

static int handle_oem_notification(struct glink_chip *chip, void *data, size_t len)
{
	struct oem_notify_ind_msg *notify_msg = data;
	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect received length %zu expected %lu\n", len,
			sizeof(*notify_msg));
		return -EINVAL;
	}

	pr_debug("notification: %#x on receiver: %#x\n",
				notify_msg->notification,
				notify_msg->receiver);

	pm_stay_awake(chip->dev);
	memcpy(&chip->notify_msg[NOTIFY_TX_ADSP],
				notify_msg, sizeof(*notify_msg));
	chip->notify_tx_mask |= 1 << NOTIFY_TX_ADSP;
	cancel_delayed_work(&chip->notify_work);
	schedule_delayed_work(&chip->notify_work, msecs_to_jiffies(0));

	return 0;
}

static int oem_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct glink_chip *chip = priv;

	pr_debug("owner: %u type: %u opcode: 0x%x len:%zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (hdr->opcode == OEM_READ_BUF_REQ)
		handle_oem_read_ack(chip, data, len);
	else if (hdr->opcode == OEM_WRITE_BUF_REQ)
		handle_oem_write_ack(chip, data, len);
	else if (hdr->opcode == OEM_NOTIFY_IND)
		handle_oem_notification(chip, data, len);
	else
		pr_err("Unknown message opcode: %d\n", hdr->opcode);

	return 0;
}

static void oem_state_cb(void *priv, enum pmic_glink_state state)
{
	struct glink_chip *chip = priv;

	pr_debug("state: %d\n", state);

	atomic_set(&chip->state, state);

	switch (state) {
	case PMIC_GLINK_STATE_DOWN:
	case PMIC_GLINK_STATE_UP:
		schedule_work(&chip->setup_work);
		break;
	default:
		break;
	}
}

static int glink_device_write(struct glink_chip *chip, u32 property,
				const void *val, size_t val_len)
{
	struct oem_write_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		pr_err("Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chip->state) == PMIC_GLINK_STATE_DOWN) {
		pr_err("ADSP glink state is down\n");
		return -ENOTCONN;
	}

	memset(&oem_buf, 0, sizeof(oem_buf));
	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_WRITE_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;
	memcpy(oem_buf.buf, val, val_len);

	mutex_lock(&chip->write_lock);
	reinit_completion(&chip->write_ack);

	pr_debug("Start data write for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chip->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chip->write_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		pr_err("timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
	}
out:
	pr_debug("Complete data write for property: %u\n", property);
	mutex_unlock(&chip->write_lock);
	return rc;
}

static int glink_device_read(struct glink_chip *chip, u32 property,
				void *val, size_t val_len)
{
	struct oem_read_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		pr_err("Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chip->state) == PMIC_GLINK_STATE_DOWN) {
		pr_err("ADSP glink state is down\n");
		return -ENOTCONN;
	}

	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_READ_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;

	mutex_lock(&chip->read_lock);
	reinit_completion(&chip->read_ack);

	pr_debug("Start data read for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chip->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chip->read_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		pr_err("timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
	}

	if (!atomic_read(&chip->rx_valid)) {
		rc = -ENODATA;
		goto out;
	}

	if (chip->rx_buf.data_size != val_len) {
		pr_err("Invalid data size %u, on property: %u\n",
				chip->rx_buf.data_size, property);
		rc = -ENODATA;
		goto out;
	}

	memcpy(val, chip->rx_buf.buf, val_len);
	atomic_set(&chip->rx_valid, 0);
out:
	pr_debug("Complete data read for property: %u\n", property);
	mutex_unlock(&chip->read_lock);

	return rc;
}

static int glink_device_set_property(struct glink_dev *dev, u32 property,
				const void *val, size_t val_len)
{
	struct glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	return glink_device_write(chip, property, val, val_len);
}

static int glink_device_get_property(struct glink_dev *dev, u32 property,
				void *val, size_t val_len)
{
	struct glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	return glink_device_read(chip, property, val, val_len);
}

struct glink_dev *glink_device_get(glink_dev_t type, u32 id)
{
	int i;
	struct glink_dev *dev = NULL;
	struct glink_chip *chip = this_chip;

	if (!chip) {
		pr_err("glink chip is not ready\n");
		return NULL;
	}

	if (type < 0 || type > GLINK_DEV_TYPE_NUM) {
		pr_err("Invalid glink device type=%d\n", type);
		return NULL;
	}

	for (i = 0; i < chip->glink_dev_count; i++) {
		dev = &chip->glink_dev_list[i];
		if (!dev->name || !dev->node || !dev->dev)
			continue;
		if (dev->type == type && dev->id == id) {
			atomic_inc(&dev->use_count);
			return dev;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(glink_device_get);

struct glink_dev *glink_device_get_by_phandle(const struct device_node *np,
				const char *phandle_name, int index)
{
	int i;
	struct glink_dev *dev;
	struct device_node *node;
	struct glink_chip *chip = this_chip;

	node = of_parse_phandle(np, phandle_name, index);
	if (!of_device_is_available(node)) {
		of_node_put(node);
		pr_err("no %s found from %s\n", phandle_name, np->name);
		return NULL;
	}

	for (i = 0; i < chip->glink_dev_count; i++) {
		dev = &chip->glink_dev_list[i];
		if (dev->node == node) {
			atomic_inc(&dev->use_count);
			of_node_put(node);
			return dev;
		}
	}

	pr_err("no matched glink device %s\n", phandle_name);
	of_node_put(node);
        return NULL;
}
EXPORT_SYMBOL(glink_device_get_by_phandle);

void glink_device_put(struct glink_dev *dev)
{
	if (atomic_read(&dev->use_count) > 0) {
		atomic_dec(&dev->use_count);
	}
}
EXPORT_SYMBOL(glink_device_put);

int glink_device_register_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&glink_device_notifier_list, nb);
}
EXPORT_SYMBOL(glink_device_register_notifier);

int glink_device_unregister_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_unregister(&glink_device_notifier_list, nb);
}
EXPORT_SYMBOL(glink_device_unregister_notifier);

static ssize_t tcmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 tcmd;
	struct glink_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &tcmd);
	if (rc) {
		pr_err("Invalid TCMD = 0x%08x\n", tcmd);
		return -EINVAL;
	}

	rc = glink_device_write(chip,
				GLINK_PROP_TCMD,
				&tcmd,
				sizeof(tcmd));

	if (!rc) {
		chip->current_tcmd = tcmd;
		pr_info("Send TCMD = 0x%08x\n", tcmd);
	}

	return rc ? rc : count;
}

static ssize_t tcmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 data = 0;
	struct glink_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	glink_device_read(chip,
				GLINK_PROP_TCMD,
				&data,
				sizeof(data));

	if ((data & 0xFFFF) != (chip->current_tcmd & 0xFFFF)) {
		pr_err("Invalid TCMD data = 0x%08x\n", data);
		return -EINVAL;
	}

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%d\n", data >> 16);
}
static DEVICE_ATTR(tcmd, 0664, tcmd_show, tcmd_store);

static ssize_t addr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 addr;
	struct glink_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &addr);
	if (rc) {
		pr_err("Invalid reg_address = 0x%x\n", addr);
		return -EINVAL;
	}

	rc = glink_device_write(chip,
				GLINK_PROP_REG_ADDRESS,
				&addr,
				sizeof(addr));

	return rc ? rc : count;
}
static DEVICE_ATTR(addr, 0220, NULL, addr_store);

static ssize_t data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	u32 data;
	struct glink_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	rc = kstrtou32(buf, 0, &data);
	if (rc) {
		pr_err("Invalid reg_data = 0x%x\n", data);
		return -EINVAL;
	}

	rc = glink_device_write(chip,
				GLINK_PROP_REG_DATA,
				&data,
				sizeof(data));

	return rc ? rc : count;
}

static ssize_t data_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 data;
	struct glink_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	glink_device_read(chip,
				GLINK_PROP_REG_DATA,
				&data,
				sizeof(data));

	return scnprintf(buf, GLINK_SHOW_MAX_SIZE, "%x\n", data);
}
static DEVICE_ATTR(data, 0664, data_show, data_store);

static char *bootargs_str = NULL;
static size_t bootargs_str_len = 0;
static int get_bootarg_dt(char *key, char **value,
				char *prop, char *spl_flag)
{
	int rc = 0;
	char *idx = NULL;
	char *kvpair = NULL;
	size_t bootargs_tmp_len = 0;
	const char *bootargs_tmp = NULL;
	struct device_node *np = of_find_node_by_path("/chosen");

	if (np == NULL)
		return -EINVAL;

	rc = of_property_read_string(np, prop, &bootargs_tmp);
	if (rc) {
		goto putnode;
	}

	bootargs_tmp_len = strlen(bootargs_tmp);
	if (bootargs_tmp_len >= bootargs_str_len) {
		if (bootargs_str)
			kfree(bootargs_str);
		bootargs_str = kzalloc(bootargs_tmp_len + 1, GFP_KERNEL);
		if (!bootargs_str) {
			rc = -ENOMEM;
			goto putnode;
		}
		bootargs_str_len = bootargs_tmp_len + 1;
	} else {
		memset(bootargs_str, '\0', bootargs_str_len);
	}

	strlcpy(bootargs_str, bootargs_tmp, bootargs_tmp_len + 1);
	idx = strnstr(bootargs_str, key, strlen(bootargs_str));
	if (!idx) {
		rc = -EINVAL;
		goto putnode;
	}

	kvpair = strsep(&idx, " ");
	if (!kvpair) {
		rc = -EINVAL;
		goto putnode;
	}

	if (strsep(&kvpair, "=") &&
	    (*value = strsep(&kvpair, spl_flag))) {
		rc = 0;
	}

putnode:
	of_node_put(np);
	return rc;
}

static int get_bootarg(char *key, char **value)
{
#ifdef CONFIG_BOOT_CONFIG
	return get_bootarg_dt(key, value, "mmi,bootconfig", "\n");
#else
	return get_bootarg_dt(key, value, "bootargs", " ");
#endif
}

static int get_sku_type(u8 *sku_type)
{
	char *radio = NULL;

	if (get_bootarg("androidboot.radio=", &radio)
	    || !radio) {
		pr_err("Could not get HW revision\n");
		return -EINVAL;
	}

	if (!strncmp("PRC", radio, 3)) {
		*sku_type = MMI_SKU_PRC;
	} else if (!strncmp("ROW", radio, 3)) {
		*sku_type = MMI_SKU_ROW;
	} else if (!strncmp("NA", radio, 2)) {
		*sku_type = MMI_SKU_NA;
	} else if (!strncmp("VZW", radio, 3)) {
		*sku_type = MMI_SKU_VZW;
	} else if (!strncmp("JPN", radio, 3)) {
		*sku_type = MMI_SKU_JPN;
	} else if (!strncmp("ITA", radio, 3)) {
		*sku_type = MMI_SKU_ITA;
	} else if (!strncmp("NAE", radio, 3)) {
		*sku_type = MMI_SKU_NAE;
	} else if (!strncmp("SUPERSET", radio, 8)) {
		*sku_type = MMI_SKU_SUPERSET;
	} else {
		*sku_type = 0;
	}
	pr_info("SKU type:%s(0x%02x)\n", radio, *sku_type);
	return 0;
}

static int get_hw_revision(u16 *hw_rev)
{
	int rc;
	char *hwrev = NULL;

	if (get_bootarg("androidboot.hwrev=", &hwrev)
	    || !hwrev) {
		pr_err("Could not get HW revision\n");
		return -EINVAL;
	}

	rc = kstrtou16(hwrev, 16, hw_rev);
	if (rc < 0) {
		pr_err("kstrtou16 fail at hwrev:%s\n", hwrev);
		return rc;
	}
	pr_info("HW revision: 0x%x\n", *hw_rev);
	return 0;
}

static bool is_softbank_carrier(void)
{
	char *carrier = NULL;

	if (get_bootarg("androidboot.carrier=", &carrier)
	   || !carrier ||
	   strncmp("softbank", carrier, 8)) {
		return false;
	}
	return true;
}

static int glink_device_create(struct glink_chip *chip)
{
	int rc;
	int i;
	u32 count;
	u32 type;
	struct glink_dev *dev = NULL;
	struct device_node *np = chip->dev->of_node;
	struct device_node *child = NULL;

	count = of_get_available_child_count(np);
	if (count == 0) {
		pr_err("no available glink device in dt\n");
		return -EINVAL;
	}

	dev = devm_kzalloc(chip->dev, sizeof(*dev) * count, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	chip->glink_dev_list = dev;
	chip->glink_dev_count = count;

	for_each_available_child_of_node(np, child) {
		rc = of_property_read_u32(child, "type", &type);
		if (rc) {
			pr_err("Invalid device type in %s, rc=%d\n",
				child->name, rc);
			of_node_put(child);
			goto free_dev;
		} else {
			dev->id = type & 0xFF;
			dev->type = (type >> 8) & 0xFF;
		}

		if (dev->type >= GLINK_DEV_TYPE_NUM) {
			pr_err("Invalid device type in %s, type=%d\n",
				child->name, dev->type);
			rc = -EINVAL;
			of_node_put(child);
			goto free_dev;
		}
		dev->node = child;
		dev->name = kbasename(child->full_name);
		dev->dev = chip->dev;
		atomic_set(&dev->use_count, 0);
		dev->ops.set_property = glink_device_set_property;
		dev->ops.get_property = glink_device_get_property;
		dev++;
	}

	for (i = 0; i < count; i++) {
		dev = &chip->glink_dev_list[i];
		switch (dev->type) {
		case GLINK_DEV_TYPE_BAT:
			rc = glink_device_battery_setup(dev);
			break;
		case GLINK_DEV_TYPE_CHG:
			rc = glink_device_charger_setup(dev);
			break;
		case GLINK_DEV_TYPE_USB:
			rc = glink_device_usb_setup(dev);
			break;
		case GLINK_DEV_TYPE_WLS:
			rc = glink_device_wls_setup(dev);
			break;
		case GLINK_DEV_TYPE_PUMP:
			rc = glink_device_pump_setup(dev);
			break;
		case GLINK_DEV_TYPE_BUCK:
			rc = glink_device_buck_setup(dev);
			break;
		case GLINK_DEV_TYPE_BCR:
			rc = glink_device_bcr_setup(dev);
			break;
		default:
			break;
		}
		if (rc) {
			pr_err("glink %s setup failed, rc=%d\n", dev->name, rc);
			memset(dev, 0, sizeof(struct glink_dev));
		}
	}
	return 0;

free_dev:
	devm_kfree(chip->dev, chip->glink_dev_list);
	chip->glink_dev_list = NULL;
	chip->glink_dev_count = 0;
	return rc;
}

static int glink_device_delete(struct glink_chip *chip)
{
	int i;
	struct glink_dev *dev = NULL;

	for (i = 0; i < chip->glink_dev_count; i++) {
		dev = &chip->glink_dev_list[i];
		if (!dev->dev)
			continue;
		if (atomic_read(&dev->use_count) > 0) {
			pr_err("glink device :%s still in use\n", dev->name);
			return -EBUSY;
		}

		if (dev->ops.deinit)
			dev->ops.deinit(dev);
		if (dev->devdata)
			devm_kfree(chip->dev, dev->devdata);
		memset(dev, 0, sizeof(struct glink_dev));
	}

	devm_kfree(chip->dev, chip->glink_dev_list);
	chip->glink_dev_list = NULL;
	chip->glink_dev_count = 0;
	return 0;
}

static int glink_device_cfg(struct glink_chip *chip)
{
	int i;
	int rc;
	struct glink_dev *dev;
	struct glink_dev_cfg cfg =  {0};

	rc = get_hw_revision(&cfg.hw_rev);
	if (rc) {
		pr_err("Fail to get HW revision\n");
		return rc;
	}

	rc = get_sku_type(&cfg.sku_type);
	if (rc) {
		pr_err("Fail to get sku type\n");
		return rc;
	}

	cfg.softbank = is_softbank_carrier();
	cfg.factory_mode = mmi_is_factory_mode();
	cfg.factory_version = mmi_is_factory_version();

	for (i = 0; i < chip->glink_dev_count; i++) {
		dev = &chip->glink_dev_list[i];
		if (!dev->dev || !dev->ops.init) {
			continue;
		}
		rc = dev->ops.init(dev, &cfg);
		if (rc) {
			pr_err("init device %s failed\n", dev->name);
		}
	}

	return 0;
}

static int glink_device_init(struct glink_chip *chip)
{
	int rc;

	rc = glink_device_create(chip);
	if (rc) {
		pr_err("Couldn't create glink device\n");
		return rc;
	}

	rc = glink_device_cfg(chip);
	if (rc) {
		pr_err("Couldn't configure glink device\n");
		return rc;
	}

	rc = device_create_file(chip->dev, &dev_attr_tcmd);
	if (rc) {
		pr_err("Couldn't create tcmd\n");
	}

	rc = device_create_file(chip->dev, &dev_attr_addr);
	if (rc) {
		pr_err("Couldn't create addr\n");
	}

	rc = device_create_file(chip->dev, &dev_attr_data);
	if (rc) {
		pr_err("Couldn't create data\n");
	}

	rc = of_property_read_u32(chip->dev->of_node,
				"polling-interval-ms",
				&chip->polling_interval_ms);
	if (rc)
		chip->polling_interval_ms = 0;

	schedule_delayed_work(&chip->notify_work, msecs_to_jiffies(0));

	return 0;
}

static int glink_device_deinit(struct glink_chip *chip)
{
	int rc;

	cancel_delayed_work(&chip->notify_work);
	device_remove_file(chip->dev, &dev_attr_tcmd);
	device_remove_file(chip->dev, &dev_attr_addr);
	device_remove_file(chip->dev, &dev_attr_data);
	rc = glink_device_delete(chip);
	return rc;
}

static int glink_device_psy_notifier_call(struct notifier_block *nb,
				unsigned long val,
				void *v)
{
	struct power_supply *psy = v;
        struct glink_chip *chip = container_of(nb,
				struct glink_chip, psy_notifier);

	if (!chip) {
		pr_err("failed to get glink chip in psy notify\n");
		return NOTIFY_DONE;
	}

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy) {
		struct oem_notify_ind_msg notify_msg = {0};
		notify_msg.receiver = GLINK_NOTIFY_RECEIVER_PSY_USR;
		if (strcmp(psy->desc->name, "battery") == 0)
			notify_msg.notification = MMI_PSY_CHANGE_BATT;
		else if (strcmp(psy->desc->name, "usb") == 0)
			notify_msg.notification = MMI_PSY_CHANGE_USB;
		else if (strcmp(psy->desc->name, "wireless") == 0)
			notify_msg.notification = MMI_PSY_CHANGE_WLS;
		else
			return NOTIFY_OK;

		memcpy(&chip->notify_msg[NOTIFY_TX_PSY],
				&notify_msg, sizeof(notify_msg));
		chip->notify_tx_mask |= 1 << NOTIFY_TX_PSY;
		cancel_delayed_work(&chip->notify_work);
		schedule_delayed_work(&chip->notify_work, msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

static void glink_device_setup_work(struct work_struct *work)
{
	struct glink_chip *chip = container_of(work,
				struct glink_chip, setup_work);
	enum pmic_glink_state state;
	struct oem_notify_ind_msg notify_msg = {0};

	state = atomic_read(&chip->state);
	if (state == PMIC_GLINK_STATE_UP) {
		pr_warn("ADSP glink state is up\n");
		glink_device_cfg(chip);
		notify_msg.notification = MMI_GLINK_STATE_UP;
	} else if (state == PMIC_GLINK_STATE_DOWN) {
		pr_warn("ADSP glink state is down\n");
		notify_msg.notification = MMI_GLINK_STATE_DOWN;
	} else {
		return;
	}

	pm_stay_awake(chip->dev);
	notify_msg.receiver = GLINK_NOTIFY_RECEIVER_LINK_USR;
	memcpy(&chip->notify_msg[NOTIFY_TX_ADSP],
				&notify_msg, sizeof(notify_msg));
	chip->notify_tx_mask |= 1 << NOTIFY_TX_ADSP;
	cancel_delayed_work(&chip->notify_work);
	schedule_delayed_work(&chip->notify_work, msecs_to_jiffies(0));
}

static void glink_device_notification_broadcast(struct glink_chip *chip,
				unsigned long notification,
				struct glink_dev_notify_data *notify_data)
{
	int i;
	struct glink_dev *dev = NULL;

	for (i = 0; i < chip->glink_dev_count; i++) {
		dev = &chip->glink_dev_list[i];
		if (!dev->dev || !dev->ops.notify)
			continue;
		dev->ops.notify(dev, notification, notify_data);
	}

	blocking_notifier_call_chain(&glink_device_notifier_list,
				notification,
				notify_data);
}

static void glink_device_notify_work(struct work_struct *work)
{
	int i;
	u32 tx_mask;
	unsigned long notification;
	struct glink_dev_notify_data notify_data;
	struct glink_chip *chip = container_of(work,
				struct glink_chip, notify_work.work);

	if (!chip->notify_tx_mask &&
	    chip->polling_interval_ms > 0) {
		memset(&notify_data, 0, sizeof(notify_data));
		notification = 0;
		notify_data.receiver = GLINK_NOTIFY_RECEIVER_POLL_TASK;
		glink_device_notification_broadcast(chip, notification,
					&notify_data);
	}

	for (i = 0; i < NOTIFY_TX_NUM && chip->notify_tx_mask; i++) {
		tx_mask = 1 << i;
		if (!(chip->notify_tx_mask & tx_mask))
			continue;
		chip->notify_tx_mask &= ~tx_mask;
		memset(&notify_data, 0, sizeof(notify_data));
		notification = chip->notify_msg[i].notification;
		notify_data.receiver = chip->notify_msg[i].receiver;
		memcpy(notify_data.data, chip->notify_msg[i].data,
				sizeof(u32) * MAX_GLINK_NOTIFY_DATA_LEN);
		memset(&chip->notify_msg[i], 0, sizeof(notify_data));
		glink_device_notification_broadcast(chip, notification,
				&notify_data);
	}

	if (chip->polling_interval_ms > 0) {
		schedule_delayed_work(&chip->notify_work,
			msecs_to_jiffies(chip->polling_interval_ms));
	}

	pm_relax(chip->dev);
}

static int glink_device_probe(struct platform_device *pdev)
{
	int rc;
	struct glink_chip *chip;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	INIT_WORK(&chip->setup_work, glink_device_setup_work);
	INIT_DELAYED_WORK(&chip->notify_work, glink_device_notify_work);
	mutex_init(&chip->read_lock);
	mutex_init(&chip->write_lock);
	init_completion(&chip->read_ack);
	init_completion(&chip->write_ack);
	atomic_set(&chip->rx_valid, 0);
	atomic_set(&chip->state, PMIC_GLINK_STATE_UP);
	chip->dev = dev;
	platform_set_drvdata(pdev, chip);
	device_init_wakeup(chip->dev, true);

	client_data.id = MSG_OWNER_OEM;
	client_data.name = "oem";
	client_data.msg_cb = oem_callback;
	client_data.priv = chip;
	client_data.state_cb = oem_state_cb;

	chip->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(chip->client)) {
		rc = PTR_ERR(chip->client);
		if (rc != -EPROBE_DEFER)
			pr_err("pmic_glink register failed, rc=%d\n", rc);
		devm_kfree(dev, chip);
		return rc;
	}

	chip->psy_notifier.notifier_call = glink_device_psy_notifier_call;
	rc = power_supply_reg_notifier(&chip->psy_notifier);
	if (rc) {
		pr_err("Failed to register psy_notifier: %d\n", rc);
		pmic_glink_unregister_client(chip->client);
		devm_kfree(dev, chip);
		return rc;
	}

	this_chip = chip;
	glink_device_init(chip);
	pr_info("Glink device driver init done\n");
	return 0;
}

static int glink_device_remove(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct glink_chip *chip = dev_get_drvdata(dev);

	rc = glink_device_deinit(chip);
	if (rc) {
		pr_err("glink device deinit failed rc=%d\n", rc);
		return rc;
	}

	rc = pmic_glink_unregister_client(chip->client);
	if (rc < 0)
		pr_err("pmic_glink_unregister_client failed rc=%d\n", rc);

	if (bootargs_str) {
		kfree(bootargs_str);
		bootargs_str = NULL;
		bootargs_str_len = 0;
	}
	devm_kfree(dev, chip);
	this_chip = NULL;

	return rc;
}

static void glink_device_shutdown(struct platform_device *pdev)
{
	pr_info("glink device driver shutdown\n");
	return;
}

static const struct of_device_id glink_device_match_table[] = {
	{.compatible = "mmi,adsp-glink-charger"},
	{},
};

static struct platform_driver glink_device_driver = {
	.driver	= {
		.name = "adsp_glink_charger",
		.of_match_table = glink_device_match_table,
	},
	.probe	= glink_device_probe,
	.remove	= glink_device_remove,
	.shutdown = glink_device_shutdown,
};

module_platform_driver(glink_device_driver);

MODULE_DESCRIPTION("ADSP Glink Charger Driver");
MODULE_LICENSE("GPL v2");
