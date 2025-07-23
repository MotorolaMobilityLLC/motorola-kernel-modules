// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"BM_ULOG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ipc_logging.h>
#include <linux/rpmsg.h>
#if (KERNEL_VERSION(6, 6, 30) > LINUX_VERSION_CODE)
#include <linux/soc/qcom/pmic_glink.h>
#else
#include <linux/soc/qcom/qti_pmic_glink.h>
#endif
#include <linux/power/bm_adsp_ulog.h>

#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* owner/type/opcodes for battery charger */
#define MSG_OWNER_BC                    32778
#define MSG_TYPE_REQ_RESP		1

#define BM_ULOG_GET			0x18
#define BM_ULOG_PROP_SET		0x19
#define BM_INIT_ULOG_GET		0x23

/* Generic definitions */
#define BM_ULOG_WAIT_TIME_MS		5000
#define MAX_ULOG_READ_BUFFER_SIZE	8192
#define BM_ULOG_PAGES			(50)
#define INIT_ULOG_RETRY_CNT		3

#define bm_info(bmdev, fmt, ...)		\
	do {					\
		printk(KERN_INFO "BM_ULOG: " fmt, ##__VA_ARGS__); \
		ipc_log_string(bmdev->ipc_log, fmt, ##__VA_ARGS__); \
	} while (0)

#define bm_dbg(bmdev, fmt, ...)			\
	do {					\
		if (bmdev->debug_enabled && *bmdev->debug_enabled)  \
			printk(KERN_INFO "BM_ULOG: " fmt, ##__VA_ARGS__); \
		else				\
			pr_debug(fmt, ##__VA_ARGS__);	\
		ipc_log_string(bmdev->ipc_log, fmt, ##__VA_ARGS__); \
	} while (0)

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable ulog to Kernel debug log");

struct bm_ulog_prop_req {
	struct pmic_glink_hdr	hdr;
	u64			categories;
	u32			level;
};

struct bm_ulog_req {
	struct pmic_glink_hdr	hdr;
	u32			max_logsize;
};

struct bm_ulog_resp {
	struct pmic_glink_hdr	hdr;
	char read_buffer[MAX_ULOG_READ_BUFFER_SIZE];
};

struct bm_ulog_dev {
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct mutex			lock;
	struct completion		ack;
	u32				level;
	u64				categories;
	struct dentry			*debugfs_dir;
	bool				*debug_enabled;
	void				*ipc_log;
	struct task_struct		*bm_ulog_task;
	char				ulog_buffer[MAX_ULOG_READ_BUFFER_SIZE];
	bool				ulog_enabled;
	struct delayed_work		ulog_complete_work;
};

static struct bm_ulog_dev *g_bmdev = NULL;

static int bm_ulog_write(struct bm_ulog_dev *bmdev, void *data, size_t len)
{
	int rc;

	mutex_lock(&bmdev->lock);
	reinit_completion(&bmdev->ack);
	rc = pmic_glink_write(bmdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bmdev->ack,
				msecs_to_jiffies(BM_ULOG_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			mutex_unlock(&bmdev->lock);
			return -ETIMEDOUT;
		}
		rc = 0;
	}
	mutex_unlock(&bmdev->lock);

	return rc;
}

static int bm_ulog_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct bm_ulog_dev *bmdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	switch (hdr->opcode) {
	case BM_ULOG_PROP_SET:
		complete(&bmdev->ack);
		break;
	case BM_ULOG_GET:
	case BM_INIT_ULOG_GET:
		if (len != sizeof(struct bm_ulog_resp)) {
			pr_err("Incorrect len %zu for bm ulog resp\n", len);
			break;
		}
		memcpy(bmdev->ulog_buffer,
				((struct bm_ulog_resp *)data)->read_buffer,
				MAX_ULOG_READ_BUFFER_SIZE);
		complete(&bmdev->ack);
		break;
	default:
		pr_err("Unknown opcode %u\n", hdr->opcode);
		break;
	}

	return 0;
}

static int bm_ulog_set_mask(struct bm_ulog_dev *bmdev,
		     enum bm_ulog_category_bitmap categories,
		     enum bm_ulog_level_type level)
{
	int rc;
	struct bm_ulog_prop_req prop_req = { { 0 } };

	if (level > BM_LOG_LEVEL_ALL_LOGS)
		level = BM_LOG_LEVEL_ALL_LOGS;

	prop_req.level = level;
	prop_req.categories = categories;
	prop_req.hdr.owner = MSG_OWNER_BC;
	prop_req.hdr.type = MSG_TYPE_REQ_RESP;
	prop_req.hdr.opcode = BM_ULOG_PROP_SET;

	rc = bm_ulog_write(bmdev, &prop_req, sizeof(prop_req));
	if (rc) {
		pr_err("BM ulog set mask failed\n");
		return rc;
	}

	bmdev->level = level;
	bmdev->categories = categories;
	pr_debug("BM ulog categories: 0x%llx, level: %d\n",
					(u64)categories, level);

	return 0;
}

static int bm_ulog_request_log(struct bm_ulog_dev *bmdev, u32 size)
{
	int rc;
	u32 max_logsize;
	struct bm_ulog_req ulog_req = { { 0 } };

	if (size > 0 && size < MAX_ULOG_READ_BUFFER_SIZE)
		max_logsize = size;
	else
		max_logsize = MAX_ULOG_READ_BUFFER_SIZE;

	ulog_req.hdr.owner = MSG_OWNER_BC;
	ulog_req.hdr.type = MSG_TYPE_REQ_RESP;
	ulog_req.hdr.opcode = BM_ULOG_GET;
	ulog_req.max_logsize = max_logsize;

	rc = bm_ulog_write(bmdev, &ulog_req, sizeof(ulog_req));

	return rc;
}

static int bm_ulog_request_init_log(struct bm_ulog_dev *bmdev, u32 size)
{
	int rc;
	u32 max_logsize;
	struct bm_ulog_req ulog_req = { { 0 } };

	if (size > 0 && size < MAX_ULOG_READ_BUFFER_SIZE)
		max_logsize = size;
	else
		max_logsize = MAX_ULOG_READ_BUFFER_SIZE;

	ulog_req.hdr.owner = MSG_OWNER_BC;
	ulog_req.hdr.type = MSG_TYPE_REQ_RESP;
	ulog_req.hdr.opcode = BM_INIT_ULOG_GET;
	ulog_req.max_logsize = max_logsize;

	rc = bm_ulog_write(bmdev, &ulog_req, sizeof(ulog_req));

	return rc;
}

int bm_ulog_get_log(char *buf, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!bmdev->debug_enabled) {
		pr_err("BM ulog debug_enabled invalid\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!buf) {
		pr_err("BM ulog invalid buf=%p\n", buf);
		return -EINVAL;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	memcpy(buf, bmdev->ulog_buffer, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_get_log);

int bm_ulog_get_mask_log(enum bm_ulog_category_bitmap categories,
		    enum bm_ulog_level_type level,
		    char *buf, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!bmdev->debug_enabled) {
		pr_err("BM ulog debug_enabled invalid\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!buf) {
		pr_err("BM ulog invalid buf=%p\n", buf);
		return -EINVAL;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_set_mask(bmdev, categories, level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	memcpy(buf, bmdev->ulog_buffer, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_get_mask_log);

#define TIMESTAMP_LEN 18
#define TIMESTAMP_DIV 19200000
static int bm_ulog_print_buffer(struct bm_ulog_dev *bmdev, u32 size)
{
	int i;
	int header = 0;
	int lines = 0;
	int line_len = 0;
	u64 timestamp = 0;
	int hh = 0, mm = 0, ss = 0, ms = 0;
	char timestamp_str[TIMESTAMP_LEN + 1];

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	for (i = 0; i < size && header < size; i++) {
		line_len = 0;
		if (bmdev->ulog_buffer[i] == '\x0a') {
			bmdev->ulog_buffer[i] = '\0';
			if (header < i)
				line_len = i - header;
		} else if (bmdev->ulog_buffer[i] == '\0') {
			if (header < i)
				line_len = i - header;
			size = i;
		}

		if (line_len < TIMESTAMP_LEN) {
			if (line_len > 0) {
				header = i + 1;
				lines++;
			}
			continue;
		}

		timestamp_str[TIMESTAMP_LEN] = '\0';
		memcpy(timestamp_str, &bmdev->ulog_buffer[header],
			TIMESTAMP_LEN);
		if (!kstrtou64(timestamp_str, 0, &timestamp)) {
			ms = (timestamp * 1000 / TIMESTAMP_DIV) % 1000;
			timestamp /= TIMESTAMP_DIV;
			hh = timestamp / 3600;
			mm = (timestamp % 3600) / 60;
			ss = timestamp % 60;
		}
		if (bmdev->ulog_enabled) {
			bm_info(bmdev, "[%02d:%02d:%02d.%03d]%s\n",
				hh, mm, ss, ms,
				&bmdev->ulog_buffer[header + TIMESTAMP_LEN]);
		} else {
			bm_dbg(bmdev, "[%02d:%02d:%02d.%03d]%s\n",
				hh, mm, ss, ms,
				&bmdev->ulog_buffer[header + TIMESTAMP_LEN]);
		}
		header = i + 1;
		lines++;
	}
	if (lines > 0)
		pr_debug("recv len=%d, lines=%d\n", i, lines);
	return lines > 0? i : 0;
}

static int bm_ulog_print_init_log(u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!bmdev->debug_enabled) {
		pr_err("BM ulog debug_enabled invalid\n");
		return -ENODEV;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_init_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}

int bm_ulog_print_log(u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!bmdev->debug_enabled) {
		pr_err("BM ulog debug_enabled invalid\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_print_log);

int bm_ulog_print_mask_log(enum bm_ulog_category_bitmap categories,
		      enum bm_ulog_level_type level, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!bmdev->debug_enabled) {
		pr_err("BM ulog debug_enabled invalid\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_set_mask(bmdev, categories, level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_print_mask_log);

int bm_ulog_enable_log(bool enable, unsigned int duration_ms)
{
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (bmdev->ulog_enabled != enable) {
		bmdev->ulog_enabled = enable;
		if (bmdev->bm_ulog_task) {
			wake_up_process(bmdev->bm_ulog_task);
		}
		pr_info("BM ulog is %s\n", enable? "enabled":"disabled");
	}

	if (bmdev->bm_ulog_task) {
		pm_relax(bmdev->dev);
		cancel_delayed_work(&bmdev->ulog_complete_work);
		if (enable && duration_ms > 0) {
			pr_info("BM ulog duration = %d\n", duration_ms);
			pm_stay_awake(bmdev->dev);
			schedule_delayed_work(&bmdev->ulog_complete_work,
					msecs_to_jiffies(duration_ms));
		}
	}
	return 0;
}
EXPORT_SYMBOL(bm_ulog_enable_log);

#ifdef CONFIG_DEBUG_FS
static int bm_ulog_dump_show(struct seq_file *s, void *unused)
{
	int rc;
	struct bm_ulog_dev *bmdev = s->private;

	rc = bm_ulog_set_mask(bmdev, bmdev->categories, bmdev->level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, MAX_ULOG_READ_BUFFER_SIZE);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}
	seq_puts(s, bmdev->ulog_buffer);

	return 0;
}

static int bm_ulog_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm_ulog_dump_show, inode->i_private);
}

static const struct file_operations bm_ulog_fops = {
	.open			= bm_ulog_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static void bm_ulog_add_debugfs(struct bm_ulog_dev *bmdev)
{
	int rc;
	struct dentry *dir, *file;

	dir = debugfs_create_dir("bm_ulog", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create bm ulog debugfs directory, rc=%d\n",
			rc);
		return;
	}

	file = debugfs_create_file("dump", 0444, dir, bmdev, &bm_ulog_fops);
	if (IS_ERR(file)) {
		rc = PTR_ERR(file);
		pr_err("Failed to create ulog dump debugfs file, rc=%d\n",
			rc);
		debugfs_remove_recursive(dir);
		return;
	}

	debugfs_create_x64("categories", 0664, dir, &bmdev->categories);
	debugfs_create_x32("level", 0664, dir, &bmdev->level);

	bmdev->debugfs_dir = dir;
}
#else
static void bm_ulog_add_debugfs(struct bm_ulog_dev *bmdev) { }
#endif

static void bm_ulog_complete_work(struct work_struct *work)
{
	struct bm_ulog_dev *bmdev = container_of(work,
						struct bm_ulog_dev,
						ulog_complete_work.work);

	if (!bmdev) {
		pr_err("Invalid bmdev\n");
		return;
	}

	if (bmdev->ulog_enabled) {
		pr_info("bm ulog completed for ulog enabling\n");
		bmdev->ulog_enabled = false;
		if (bmdev->bm_ulog_task) {
			wake_up_process(bmdev->bm_ulog_task);
		}
	}
	pm_relax(bmdev->dev);
}

static int bm_ulog_kthread(void *param)
{
	struct bm_ulog_dev * bmdev = param;
	int read_count = 0;
	int sleep_ms = 0;

	bm_info(bmdev, "bm ulog kthread start\n");
	do {
		if (bmdev->ulog_enabled ||
		    (bmdev->debug_enabled && *bmdev->debug_enabled)) {
			bm_ulog_request_log(bmdev, MAX_ULOG_READ_BUFFER_SIZE);
			read_count = bm_ulog_print_buffer(bmdev, MAX_ULOG_READ_BUFFER_SIZE);

			if (read_count > 1024) {
				sleep_ms = 50;
			} else if (read_count > 128) {
				sleep_ms = 100;
			} else {
				sleep_ms = 200;
			}
		} else {
			sleep_ms = 1000;
		}
		msleep(sleep_ms);
	} while(!kthread_should_stop());

	bm_info(bmdev, "bm ulog kthread exit\n");
	return 0;
}

bool bm_ulog_is_bm_ulog_enabled(struct bm_ulog_dev *bmdev)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool rt = false;
	const char *bootargs = NULL;
	char *bm_ulog_enabled = NULL;

	if (bmdev && bmdev->debug_enabled && *bmdev->debug_enabled) {
		bm_info(bmdev, "bmdev->bm_ulog_enabled is true\n");
		return true;
	}

	if (!np) {
		bm_info(bmdev, "np is null\n");
		return false;
	}

	if (!of_property_read_string(np, "bootargs", &bootargs)) {
		bm_ulog_enabled = strstr(bootargs, "bm_ulog_enabled=1");
		bm_info(bmdev, "of_property_read_string bm_ulog_enabled=%s\n", bm_ulog_enabled);
		if (bm_ulog_enabled) {
			rt = true;
		}
	}

	of_node_put(np);

	bm_info(bmdev,"bm ulog rt = %d\n", rt);
	return rt;
}

static int bm_ulog_probe(struct platform_device *pdev)
{
	int rc;
	struct bm_ulog_dev *bmdev;
	struct pmic_glink_client_data client_data = { };
	struct device_node *node = pdev->dev.of_node;
	bool init_log_enabled, init_debug_enabled;

	bmdev = devm_kzalloc(&pdev->dev, sizeof(*bmdev), GFP_KERNEL);
	if (!bmdev)
		return -ENOMEM;

	rc = of_property_read_u64(node, "categories", &bmdev->categories);
	if (rc)
		bmdev->categories = BM_ALL;
	rc = of_property_read_u32(node, "level", &bmdev->level);
	if (rc)
		bmdev->level = BM_LOG_LEVEL_INFO;

	init_log_enabled = of_property_read_bool(node, "init-log-enabled");

	bmdev->dev = &pdev->dev;
	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_manager_adsp_ulog";
	client_data.msg_cb = bm_ulog_callback;
	client_data.priv = bmdev;

	bmdev->client = pmic_glink_register_client(bmdev->dev, &client_data);
	if (IS_ERR(bmdev->client)) {
		rc = PTR_ERR(bmdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(bmdev->dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}

	mutex_init(&bmdev->lock);
	init_completion(&bmdev->ack);
	platform_set_drvdata(pdev, bmdev);
	bmdev->debug_enabled = &debug_enabled;
	g_bmdev = bmdev;
	bmdev->ipc_log = ipc_log_context_create(BM_ULOG_PAGES, "bm_ulog", 0);
	if (!bmdev->ipc_log)
		dev_err(bmdev->dev, "Failed to create ipc log\n");

	if (init_log_enabled) {
		int ulog_retry_cnt = 0;
		init_debug_enabled = debug_enabled;
		debug_enabled = init_log_enabled;
		bm_ulog_set_mask(bmdev, bmdev->categories, bmdev->level);
		do {
			bm_ulog_print_init_log(MAX_ULOG_READ_BUFFER_SIZE);
			bm_ulog_print_log(MAX_ULOG_READ_BUFFER_SIZE);
			bm_info(bmdev, "ulog_retry_cnt = %d\n", ulog_retry_cnt);
		} while (++ulog_retry_cnt < INIT_ULOG_RETRY_CNT);
		debug_enabled = init_debug_enabled;
	}

	debug_enabled = bm_ulog_is_bm_ulog_enabled(bmdev);
	bm_info(bmdev, "bm_ulog_check_debug_enabled debug_enabled=%d\n", debug_enabled);
	bmdev->bm_ulog_task = kthread_create(bm_ulog_kthread, bmdev, "bm_ulog_kthread");
	if (IS_ERR_OR_NULL(bmdev->bm_ulog_task)) {
		bmdev->bm_ulog_task = NULL;
		bm_info(bmdev, "Failed to create bm_ulog_task ret = %ld\n", PTR_ERR(bmdev->bm_ulog_task));
	} else {
		device_init_wakeup(bmdev->dev, true);
		wake_up_process(bmdev->bm_ulog_task);
		bm_info(bmdev, "Successed to create bm_ulog_task\n");
		INIT_DELAYED_WORK(&bmdev->ulog_complete_work, bm_ulog_complete_work);
	}

	bm_ulog_add_debugfs(bmdev);

	bm_info(bmdev, "BM adsp ulog driver initialized successfully\n");

	return 0;
}

static int bm_ulog_remove(struct platform_device *pdev)
{
	struct bm_ulog_dev *bmdev = platform_get_drvdata(pdev);
	int rc;

	if (bmdev->bm_ulog_task) {
		kthread_stop(bmdev->bm_ulog_task);
		cancel_delayed_work(&bmdev->ulog_complete_work);
		pm_relax(bmdev->dev);
	}
	ipc_log_context_destroy(bmdev->ipc_log);
	debugfs_remove_recursive(bmdev->debugfs_dir);
	rc = pmic_glink_unregister_client(bmdev->client);
	if (rc < 0) {
		pr_err("Error unregistering from pmic_glink, rc=%d\n", rc);
		return rc;
	}
	g_bmdev = NULL;

	return 0;
}

static const struct of_device_id bm_ulog_match_table[] = {
	{ .compatible = "qcom,bm-adsp-ulog" },
	{},
};

static struct platform_driver bm_ulog_driver = {
	.driver	= {
		.name = "bm_adsp_ulog",
		.of_match_table = bm_ulog_match_table,
	},
	.probe	= bm_ulog_probe,
	.remove	= bm_ulog_remove,
};
module_platform_driver(bm_ulog_driver);

MODULE_DESCRIPTION("QTI Glink battery manager adsp ulog driver");
MODULE_LICENSE("GPL v2");
