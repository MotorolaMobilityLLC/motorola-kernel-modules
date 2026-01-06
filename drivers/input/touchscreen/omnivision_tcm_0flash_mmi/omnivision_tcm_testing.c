/*
 * Omnivision TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 Omnivision Incorporated. All rights reserved.

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND Omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL Omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF Omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, Omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/hrtimer.h>
#include <linux/rtc.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_OVT_LOG_CAPTURE
#include <linux/sysfs.h>
#include <linux/kobject.h>
#endif

#include "omnivision_tcm_core.h"
#include "omnivision_tcm_testing.h"

#ifdef CONFIG_OVT_LOG_CAPTURE
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%-5d "
#define DATA_FORMAT_STR     "%-5d "
#endif

#define SYSFS_DIR_NAME "testing"

#define OVT_TCM_LIMIT_TSR_IMAGE_NAME "omnivision/tsr_limit.img"

#define REPORT_TIMEOUT_MS 5000

#ifdef CONFIG_OVT_LOG_CAPTURE
static int raw_delta_frame_cnt = 10;
#endif

#define testing_sysfs_show(t_name) \
static ssize_t testing_sysfs_##t_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = testing_##t_name(); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to do "#t_name" test\n"); \
		goto exit; \
	} \
\
	retval = snprintf(buf, PAGE_SIZE, \
			"%s\n", \
			testing_hcd->result ? "Passed" : "Failed"); \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
}

#define testing_sysfs_raw_delta_show(t_name) \
static ssize_t testing_sysfs_##t_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	int count = 0; \
	int rows, cols, i, j; \
	int data_value; \
	unsigned char *report_data_buf; \
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd; \
	struct ovt_tcm_app_info *app_info; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = testing_##t_name(); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to do "#t_name" test\n"); \
		goto exit; \
	} \
\
	count = 0; \
\
	retval = snprintf(buf, PAGE_SIZE - count, \
			"#t_name data:  %d\n", \
			tcm_hcd->id_info.version); \
	buf += retval; \
	count += retval; \
\
	app_info = &tcm_hcd->app_info; \
	rows = le2_to_uint(app_info->num_of_image_rows); \
	cols = le2_to_uint(app_info->num_of_image_cols); \
	report_data_buf = testing_hcd->report.buf; \
	data_value = 0; \
	for (i = 0; i < rows; i++) { \
		for (j = 0; j < cols; j++) { \
			data_value = (short)le2_to_uint(&report_data_buf[(i * cols + j) * 2]); \
			retval = snprintf(buf, PAGE_SIZE - count,"%04d, ", data_value); \
			buf += retval; \
			count += retval; \
		} \
		retval = snprintf(buf, PAGE_SIZE - count,"\n"); \
		buf += retval; \
		count += retval; \
	} \
\
	retval = snprintf(buf, PAGE_SIZE - count,"\n"); \
	buf += retval; \
	count += retval; \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return count; \
}

#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))

/* Sync with the Comm2 Release 21 */
/* Not all are implemented for every ASIC */
enum test_code {
	TEST_NOT_IMPLEMENTED = 0x00,
	TEST_PT1_TRX_TRX_SHORTS = 0x01,
	TEST_PT2_TRX_SENSOR_OPENS = 0x02,
	TEST_PT3_TRX_GROUND_SHORTS = 0x03,
	TEST_PT5_FULL_RAW_CAP = 0x05,
	TEST_PT6_EE_SHORT = 0x06,
	TEST_PT7_DYNAMIC_RANGE = 0x07,
	TEST_PT8_HIGH_RESISTANCE = 0x08,
	TEST_PT10_DELTA_NOISE = 0x0a,
	TEST_PT11_OPEN_DETECTION = 0x0b,
	TEST_PT12 = 0x0c,
	TEST_PT13 = 0x0d,
	TEST_PT14_DOZE_DYNAMIC_RANGE = 0x0e,
	TEST_PT15_DOZE_NOISE = 0x0f,
	TEST_PT16_SENSOR_SPEED = 0x10,
	TEST_PT17_ADC_RANGE = 0x11,
	TEST_PT18_HYBRID_ABS_RAW = 0x12,
	TEST_PT29_HYBRID_ABS_NOISE = 0x1D,
};

struct testing_hcd {
	bool result;
	unsigned char report_type;
	unsigned int report_index;
	unsigned int num_of_reports;
	struct kobject *sysfs_dir;
	struct proc_dir_entry *proc_csv;
#ifdef CONFIG_OVT_SELFTEST_ENABLE
	struct proc_dir_entry *ovt_selftest_proc_entry;
#endif
	struct ovt_tcm_buffer out;
	struct ovt_tcm_buffer resp;
	struct ovt_tcm_buffer report;
	struct ovt_tcm_buffer process;
	struct ovt_tcm_buffer output;
	struct ovt_tcm_hcd *tcm_hcd;
	struct ovt_tcm_test_threshold testing_csv_threshold;
	int (*collect_reports)(enum report_type report_type,
			unsigned int num_of_reports);
	const struct firmware *limit_tsr_fw_entry;
#ifdef PT1_GET_PIN_ASSIGNMENT
#define MAX_PINS (64)
	unsigned char *satic_cfg_buf;
	short tx_pins[MAX_PINS];
	short tx_assigned;
	short rx_pins[MAX_PINS];
	short rx_assigned;
	short guard_pins[MAX_PINS];
	short guard_assigned;
#endif

};

#ifdef CONFIG_OVT_LOG_CAPTURE
extern struct device *ts_class_dev;
#endif

DECLARE_COMPLETION(report_complete);

DECLARE_COMPLETION(testing_remove_complete);

#ifdef CONFIG_OVT_SELFTEST_ENABLE
struct proc_dir_entry *touch_info_dir;
#endif

static struct testing_hcd *testing_hcd;

/* testing implementation */
static int testing_device_id(void);

static int testing_config_id(void);

static int testing_reset_open(void);

static int testing_pt01_trx_trx_short(void);

static int testing_pt05_full_raw(void);

static int testing_pt07_dynamic_range(void);

static int testing_pt10_noise(void);

static int testing_do_testing(void);

static int testing_pt11_open_detection(void);

static int testing_raw_data(void);

static int testing_delta_data(void);

SHOW_PROTOTYPE(testing, testing_size)

/* nodes for testing */
SHOW_PROTOTYPE(testing, device_id)
SHOW_PROTOTYPE(testing, config_id)
SHOW_PROTOTYPE(testing, reset_open)
SHOW_PROTOTYPE(testing, pt01_trx_trx_short)
SHOW_PROTOTYPE(testing, pt05_full_raw)
SHOW_PROTOTYPE(testing, pt07_dynamic_range)
SHOW_PROTOTYPE(testing, pt10_noise)
SHOW_PROTOTYPE(testing, do_testing)
SHOW_PROTOTYPE(testing, pt11_open_detection)
SHOW_PROTOTYPE(testing, raw_data)
SHOW_PROTOTYPE(testing, delta_data)
#ifdef CONFIG_OVT_LOG_CAPTURE
STORE_PROTOTYPE(testing, log_trigger)
#endif


static struct device_attribute *attrs[] = {
	ATTRIFY(testing_size),
	ATTRIFY(device_id),
	ATTRIFY(config_id),
	ATTRIFY(pt01_trx_trx_short),
	ATTRIFY(pt05_full_raw),
	ATTRIFY(pt07_dynamic_range),
	ATTRIFY(pt10_noise),
	ATTRIFY(pt11_open_detection),
	ATTRIFY(do_testing),
	ATTRIFY(reset_open),
	ATTRIFY(raw_data),
	ATTRIFY(delta_data),
#ifdef CONFIG_OVT_LOG_CAPTURE
	ATTRIFY(log_trigger),
#endif
};

static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct bin_attribute bin_attr = {
	.attr = {
		.name = "data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = testing_sysfs_data_show,
};

testing_sysfs_show(device_id)

testing_sysfs_show(config_id)

testing_sysfs_show(pt01_trx_trx_short)

testing_sysfs_show(pt05_full_raw)

testing_sysfs_show(pt07_dynamic_range)

testing_sysfs_show(pt10_noise)

testing_sysfs_show(pt11_open_detection)

testing_sysfs_show(reset_open)

testing_sysfs_raw_delta_show(raw_data)

testing_sysfs_raw_delta_show(delta_data)

#ifdef CONFIG_OVT_LOG_CAPTURE
void ovt_format_frame_data_to_print_buf(int frame_cnt, int frame_size, int rows, int cols, unsigned char *frame_data, unsigned char *output_str, int str_size)
{
	int frame_idx = 0;
	int i, j;
	int data_value;
	unsigned char *report_data_buf;

	memset(output_str, 0, str_size);
	while (frame_idx < frame_cnt) {
		report_data_buf = frame_data + frame_idx * frame_size;
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str), "frame %d :\n", frame_idx);
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				data_value = (short)le2_to_uint(&report_data_buf[(i * cols + j) * 2]);
				snprintf(output_str + strlen(output_str), str_size - strlen(output_str), "%04d, ", data_value);
			}
			snprintf(output_str + strlen(output_str), str_size - strlen(output_str), "\n");
		}
		frame_idx++;
	}
}

int ovt_tp_rawdata_capture(struct device *dev)
{
	int retval;
	int count = 0;
	int rows, cols, i, j;
	int data_value;
	unsigned char *report_data_buf;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_app_info *app_info;
	char line_buf[4*1024]={0};
	mutex_lock(&tcm_hcd->extif_mutex);

	OVT_INFO("capture rawdata start\n");
	retval = testing_raw_data();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do raw_data test\n");
		return -ENOMEM;
	}

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	report_data_buf = testing_hcd->report.buf;
	data_value = 0;

	for (i = 0; i < rows; i++) {
		if (count >= sizeof(line_buf) - 1) {
			OVT_INFO("line_buf overflow! total count: %d, buf size: %zu\n", count, sizeof(line_buf));
			break;
		}
		count += scnprintf(line_buf + count, sizeof(line_buf) - count, ROW_NUM_FORMAT_STR, i);
		for (j = 0; j < cols; j++) {
			data_value = (short)le2_to_uint(&report_data_buf[(i * cols + j) * 2]);
			count += scnprintf(line_buf + count, sizeof(line_buf) - count, DATA_FORMAT_STR, data_value);
		}
		if (count < sizeof(line_buf) - 1) {
        	count += scnprintf(line_buf + count, sizeof(line_buf) - count, "\n");
    	}
	}
	ts_put_fifo_with_discard(line_buf, count);

	sysfs_notify(&ts_class_dev->kobj, NULL, "log_trigger");

	mutex_unlock(&tcm_hcd->extif_mutex);

	return 0;
}

int ovt_tp_diffdata_capture(struct device *dev)
{
	int retval;
	int count = 0;
	int rows, cols, i, j;
	int data_value;
	unsigned char *report_data_buf;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_app_info *app_info;
	char line_buf[4*1024]={0};

	mutex_lock(&tcm_hcd->extif_mutex);

	OVT_INFO("capture diffdata start\n");
	retval = testing_delta_data();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do delta test\n");
		return -ENOMEM;
	}

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	report_data_buf = testing_hcd->report.buf;
	data_value = 0;

	for (i = 0; i < rows; i++) {
		if (count >= sizeof(line_buf) - 1) {
			OVT_INFO("line_buf overflow! total count: %d, buf size: %zu\n", count, sizeof(line_buf));
			break;
		}
		count += scnprintf(line_buf + count, sizeof(line_buf) - count, ROW_NUM_FORMAT_STR, i);
		for (j = 0; j < cols; j++) {
			data_value = (short)le2_to_uint(&report_data_buf[(i * cols + j) * 2]);
			count += scnprintf(line_buf + count, sizeof(line_buf) - count, DATA_FORMAT_STR, data_value);
		}
		if (count < sizeof(line_buf) - 1) {
			count += scnprintf(line_buf + count, sizeof(line_buf) - count, "\n");
		}
	}

	ts_put_fifo_with_discard(line_buf, count);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return 0;
}

int ovt_tp_data_dump_capture(struct device *dev)
{
	int ret = 0;
	ts_clear_kfifo();
	ret = ovt_tp_diffdata_capture(dev);

	ret = ovt_tp_rawdata_capture(dev);

	return ret;
}
#endif

//#define LIMIT_FROM_CSV_FILE 1

#ifdef LIMIT_FROM_CSV_FILE

static int ovt_tcm_get_thr_from_csvfile(void)
{
	int ret = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	struct ovt_tcm_app_info *app_info = NULL;
	struct ovt_tcm_test_threshold *threshold = &(testing_hcd->testing_csv_threshold);
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	char file_path[256] = {0};

	strncpy(file_path, "/data/ovt_tcm_", sizeof(file_path));
	// if (tcm_hcd->hw_if->bdata->project_id) {
	// 	strncat(file_path, tcm_hcd->hw_if->bdata->project_id, sizeof(file_path));
	// }
	strncat(file_path, "cap_limits.csv", sizeof(file_path) - strlen(file_path) - 1);

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	printk("ovt tcm csv parser: rows: %d, cols: %d file_path:%s", rows, cols, file_path);
	ret = ovt_tcm_parse_csvfile(file_path, CSV_RAW_DATA_MIN_ARRAY,
			threshold->raw_data_min_limits, rows, cols);
	if (ret) {
		printk("ovt tcm csv parser: %s: Failed get %s \n", __func__, CSV_RAW_DATA_MIN_ARRAY);
		return ret;
	}

	ret = ovt_tcm_parse_csvfile(file_path, CSV_RAW_DATA_MAX_ARRAY,
			threshold->raw_data_max_limits, rows, cols);
	if (ret) {
		printk("ovt tcm csv parser: %s: Failed get %s \n", __func__, CSV_RAW_DATA_MAX_ARRAY);
		return ret;
	}

	ret = ovt_tcm_parse_csvfile(file_path, CSV_OPEN_SHORT_MIN_ARRAY,
			threshold->open_short_min_limits, rows, cols);
	if (ret) {
		printk("ovt tcm csv parser: %s: Failed get %s \n", __func__, CSV_OPEN_SHORT_MIN_ARRAY);
		return ret;
	}

	ret = ovt_tcm_parse_csvfile(file_path, CSV_OPEN_SHORT_MAX_ARRAY,
			threshold->open_short_max_limits, rows, cols);
	if (ret) {
		printk("ovt tcm csv parser: %s: Failed get %s \n", __func__, CSV_OPEN_SHORT_MAX_ARRAY);
		return ret;
	}

	ret = ovt_tcm_parse_csvfile(file_path, CSV_LCD_NOISE_ARRAY,
			threshold->lcd_noise_max_limits, rows, cols);
	if (ret) {
		printk("ovt tcm csv parser: %s: Failed get %s \n", __func__, CSV_LCD_NOISE_ARRAY);
		return ret;
	}

	printk("ovt tcm csv parser: %s: success \n", __func__);
	return 0;
}
#endif

static char *g_testing_output_buf;
#define OUTPUT_TO_CSV_STRING_LEN 100*1024

static ssize_t testing_sysfs_do_testing_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	int test_result;
	//struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	//mutex_lock(&tcm_hcd->extif_mutex);

	test_result = testing_do_testing();

	if (test_result) {
		retval = snprintf(buf, PAGE_SIZE, "%s\n","fail");
	} else {
		retval = snprintf(buf, PAGE_SIZE, "%s\n","pass");
	}
	return retval;
}

static ssize_t testing_sysfs_testing_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);

	LOCK_BUFFER(testing_hcd->output);

	retval = snprintf(buf, PAGE_SIZE,
			"%u\n",
			testing_hcd->output.data_length);

	UNLOCK_BUFFER(testing_hcd->output);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int retval;
	unsigned int readlen;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);

	LOCK_BUFFER(testing_hcd->output);

	readlen = MIN(count, testing_hcd->output.data_length - pos);

	retval = secure_memcpy(buf,
			count,
			&testing_hcd->output.buf[pos],
			testing_hcd->output.buf_size - pos,
			readlen);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to copy report data\n");
	} else {
		retval = readlen;
	}

	UNLOCK_BUFFER(testing_hcd->output);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

#ifdef CONFIG_OVT_LOG_CAPTURE
static int capture_data_type = 0;

static ssize_t testing_sysfs_log_trigger_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_app_info *app_info;
	int rows, cols , frame_size;

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	frame_size = rows * cols * 2;
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	capture_data_type = input;

	if (capture_data_type == 1) {
		//raw
		testing_raw_data();

	} else if (capture_data_type == 2) {
		//delta
		testing_delta_data();
	}
	if (!g_testing_output_buf) {
		g_testing_output_buf = vmalloc(OUTPUT_TO_CSV_STRING_LEN);
		if (!g_testing_output_buf) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"can not alloc buffer for g_testing_output_buf\n");
			return -1;
		}
	}
	ovt_format_frame_data_to_print_buf(raw_delta_frame_cnt, frame_size, rows, cols, testing_hcd->report.buf, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
	return count;
}
#endif

/*/proc/fts_test_csv*/
static int ovt_csv_show(struct seq_file *s, void *v)
{
    seq_printf(s, "%s", g_testing_output_buf);
    return 0;
}

static int ovt_csv_open(struct inode *inode, struct file *file)
{
    return single_open(file, ovt_csv_show, PDE_DATA(inode));
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ovt_proccsv_fops = {
    .proc_open = ovt_csv_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations ovt_proccsv_fops = {
    .open = ovt_csv_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

static int testing_run_prod_test_item(enum test_code test_code)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	if (tcm_hcd->features.dual_firmware &&
			tcm_hcd->id_info.mode != MODE_PRODUCTIONTEST_FIRMWARE) {
		retval = tcm_hcd->switch_mode(tcm_hcd, FW_MODE_PRODUCTION_TEST);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to run production test firmware\n");
			return retval;
		}
	} else if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		return -ENODEV;
	}

	LOCK_BUFFER(testing_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for testing_hcd->out.buf\n");
		UNLOCK_BUFFER(testing_hcd->out);
		return retval;
	}

	testing_hcd->out.buf[0] = test_code;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	return 0;
}

static int testing_collect_reports(enum report_type report_type,
		unsigned int num_of_reports)
{
	int retval;
	bool completed;
	unsigned int timeout;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	testing_hcd->report_index = 0;
	testing_hcd->report_type = report_type;
	testing_hcd->num_of_reports = num_of_reports;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&report_complete);
#else
	INIT_COMPLETION(report_complete);
#endif

	LOCK_BUFFER(testing_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for testing_hcd->out.buf\n");
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_ENABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_ENABLE_REPORT));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	completed = false;
	timeout = REPORT_TIMEOUT_MS * num_of_reports;

	retval = wait_for_completion_timeout(&report_complete,
			msecs_to_jiffies(timeout));
	if (retval == 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Timed out waiting for report collection\n");
	} else {
		completed = true;
	}

	LOCK_BUFFER(testing_hcd->out);

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_DISABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_DISABLE_REPORT));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	if (completed)
		retval = 0;
	else
		retval = -EIO;

exit:
	testing_hcd->report_type = 0;

	return retval;
}

static void testing_get_frame_size_words(unsigned int *size, bool image_only)
{
	unsigned int rows;
	unsigned int cols;
	unsigned int hybrid;
	unsigned int buttons;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	hybrid = le2_to_uint(app_info->has_hybrid_data);
	buttons = le2_to_uint(app_info->num_of_buttons);

	*size = rows * cols;

	if (!image_only) {
		if (hybrid)
			*size += rows + cols;
		*size += buttons;
	}

	return;
}

static void testing_standard_frame_output(bool image_only)
{
	int retval;
	unsigned int data_size;
	unsigned int header_size;
	unsigned int output_size;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	app_info = &tcm_hcd->app_info;

	testing_get_frame_size_words(&data_size, image_only);

	header_size = sizeof(app_info->num_of_buttons) +
			sizeof(app_info->num_of_image_rows) +
			sizeof(app_info->num_of_image_cols) +
			sizeof(app_info->has_hybrid_data);

	output_size = header_size + data_size * 2;

	LOCK_BUFFER(testing_hcd->output);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->output,
			output_size);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for testing_hcd->output.buf\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	retval = secure_memcpy(testing_hcd->output.buf,
			testing_hcd->output.buf_size,
			&app_info->num_of_buttons[0],
			header_size,
			header_size);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy header data\n");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	output_size = header_size;

	LOCK_BUFFER(testing_hcd->resp);

	retval = secure_memcpy(testing_hcd->output.buf + header_size,
			testing_hcd->output.buf_size - header_size,
			testing_hcd->resp.buf,
			testing_hcd->resp.buf_size,
			testing_hcd->resp.data_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy test data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	output_size += testing_hcd->resp.data_length;

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_hcd->output.data_length = output_size;

	UNLOCK_BUFFER(testing_hcd->output);

	return;
}
static const char *device_id_limit = "3618";
static int testing_device_id(void)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_identification *id_info;
	char *strptr = NULL;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	id_info = &tcm_hcd->id_info;

	strptr = strnstr(id_info->part_number,
					device_id_limit,
					sizeof(id_info->part_number));
	if (strptr != NULL)
		testing_hcd->result = true;
	else
		LOGE(tcm_hcd->pdev->dev.parent,
				"Device ID is mismatching, FW: %s (%s)\n",
				id_info->part_number, device_id_limit);

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return 0;
}

static int testing_config_id(void)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
#if 0
	struct ovt_tcm_app_info *app_info;
#endif
	int i;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

#if 0
	app_info = &tcm_hcd->app_info;
#endif

	testing_hcd->result = true;
	for (i = 0; i < sizeof(config_id_limit); i++) {
		if (config_id_limit[i] !=
				tcm_hcd->app_info.customer_config_id[i]) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Config ID is mismatching at byte %d\n",
					i);
			testing_hcd->result = false;
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return 0;
}

#ifdef PT1_GET_PIN_ASSIGNMENT

static int testing_get_static_config(unsigned char *buf, unsigned int buf_len)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	if (!buf) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid parameter\n");
		return -EINVAL;
	}

	LOCK_BUFFER(testing_hcd->out);
	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
					CMD_GET_STATIC_CONFIG,
					NULL,
					0,
					&testing_hcd->resp.buf,
					&testing_hcd->resp.buf_size,
					&testing_hcd->resp.data_length,
					NULL,
					0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_STATIC_CONFIG));
		goto exit;
	}

	if (testing_hcd->resp.data_length != buf_len) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Cfg size mismatch\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = secure_memcpy(buf,
				buf_len,
				testing_hcd->resp.buf,
				testing_hcd->resp.buf_size,
				buf_len);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy cfg data\n");
		goto exit;
	}

exit:
	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	return retval;
}

static bool testing_is_pins_assigned(int pin)
{
	int i;
	short *tx_pins = testing_hcd->tx_pins;
	short tx_assigned = testing_hcd->tx_assigned;
	short *rx_pins = testing_hcd->rx_pins;
	short rx_assigned = testing_hcd->rx_assigned;
	short *guard_pins = testing_hcd->guard_pins;
	short guard_assigned = testing_hcd->guard_assigned;

	for (i = 0; i < tx_assigned; i++) {
		if (pin == tx_pins[i])
			return true;
	}
	for (i = 0; i < rx_assigned; i++) {
		if (pin == rx_pins[i])
			return true;
	}
	for (i = 0; i < guard_assigned; i++) {
		if (pin == guard_pins[i])
			return true;
	}

	return false;
}

static int testing_get_pins_mapping(unsigned char *cfg_data, int length)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	int i, j = 0;
	int idx;

	int offset_tx_pin = cfg_imagetxes.offset/8;
	int length_tx_pin = cfg_imagetxes.length/8;
	int offset_rx_pin = cfg_imagerxes.offset/8;
	int length_rx_pin = cfg_imagerxes.length/8;

	int num_tx_guard = 0;
	int offset_num_tx_guard = cfg_numtxguards.offset/8;
	int length_num_tx_guard = cfg_numtxguards.length/8;
	int offset_tx_guard = cfg_txguardpins.offset/8;
	int length_tx_guard = cfg_txguardpins.length/8;

	int num_rx_guard = 0;
	int offset_num_rx_guard = cfg_numrxguards.offset/8;
	int length_num_rx_guard = cfg_numrxguards.length/8;
	int offset_rx_guard = cfg_rxguardpins.offset/8;
	int length_rx_guard = cfg_rxguardpins.length/8;

	if (!cfg_data) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid parameter\n");
		return -EINVAL;
	}

	testing_hcd->tx_assigned = 0;
	testing_hcd->rx_assigned = 0;
	testing_hcd->guard_assigned = 0;

	/* get tx pins mapping */
	if (0 == (offset_tx_pin + length_tx_pin))
		goto get_rx_pins;

	if (length > offset_tx_pin + length_tx_pin) {

		testing_hcd->tx_assigned = (length_tx_pin/2);

		idx = 0;
		for (i = 0; i < (length_tx_pin/2); i++) {
			testing_hcd->tx_pins[i] =
				(short)cfg_data[offset_tx_pin + idx] |
				(short)(cfg_data[offset_tx_pin + idx + 1] << 8);
			idx += 2;

			LOGD(tcm_hcd->pdev->dev.parent,
					"tx[%d] = %2d\n",
					i, testing_hcd->tx_pins[i]);
		}
	}

get_rx_pins:
	/* get rx pins mapping */
	if (0 == (offset_rx_pin + length_rx_pin))
		goto get_num_tx_guards;

	if (length > offset_rx_pin + length_rx_pin) {

		testing_hcd->rx_assigned = (length_rx_pin/2);

		idx = 0;
		for (i = 0; i < (length_rx_pin/2); i++) {
			testing_hcd->rx_pins[i] =
				(short)cfg_data[offset_rx_pin + idx] |
				(short)(cfg_data[offset_rx_pin + idx + 1] << 8);
			idx += 2;

			LOGD(tcm_hcd->pdev->dev.parent,
					"rx[%d] = %2d\n",
					i, testing_hcd->rx_pins[i]);
		}
	}

get_num_tx_guards:
	/* get number of tx guards */
	if (0 == (offset_num_tx_guard + length_num_tx_guard))
		goto get_num_rx_guards;

	if (length > offset_num_tx_guard + length_num_tx_guard) {

		num_tx_guard = (short)cfg_data[offset_num_tx_guard] |
				(short)(cfg_data[offset_num_tx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_tx_guard;
	}

get_num_rx_guards:
	/* get number of rx guards */
	if (0 == (offset_num_rx_guard + length_num_rx_guard))
		goto get_guards;

	if (length > offset_num_rx_guard + length_num_rx_guard) {

		num_rx_guard = (short)cfg_data[offset_num_rx_guard] |
				(short)(cfg_data[offset_num_rx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_rx_guard;
	}

get_guards:
	if (testing_hcd->guard_assigned > 0)
		LOGD(tcm_hcd->pdev->dev.parent,
				"num of guards = %2d\n",
				testing_hcd->guard_assigned);

	/* get tx guards mapping */
	if ((num_tx_guard > 0) &&
		(length > offset_tx_guard + length_tx_guard)) {
		idx = 0;
		for (i = 0; i < num_tx_guard; i++) {
			testing_hcd->guard_pins[j] =
			  (short)cfg_data[offset_tx_guard + idx] |
			  (short)(cfg_data[offset_tx_guard + idx + 1] << 8);

			LOGD(tcm_hcd->pdev->dev.parent,
					"guard_pins[%d] = %2d\n",
					i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	/* get rx guards mapping */
	if ((num_rx_guard > 0) &&
		(length > offset_rx_guard + length_rx_guard)) {
		idx = 0;
		for (i = 0; i < num_rx_guard; i++) {
			testing_hcd->guard_pins[j] =
			  (short)cfg_data[offset_rx_guard + idx] |
			  (short)(cfg_data[offset_rx_guard + idx + 1] << 8);

			LOGD(tcm_hcd->pdev->dev.parent,
					"guard_pins[%d] = %2d\n",
					i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	return 0;
}

#endif /* end of PT1_GET_PIN_ASSIGNMENT */

static int testing_pt01_trx_trx_short(void)
{
	int retval;
	int i, j;
	int phy_pin;
	bool do_pin_test = false;
#if 0
	struct ovt_tcm_app_info *app_info;
#endif
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned int size;
	unsigned char limit;
	unsigned char *buf;
	unsigned char data;
#ifdef PT1_GET_PIN_ASSIGNMENT
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length;
#endif

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

#if 0
	app_info = &tcm_hcd->app_info;
#endif

#ifdef PT1_GET_PIN_ASSIGNMENT
	satic_cfg_length = le2_to_uint(app_info->static_config_size);

	if (!testing_hcd->satic_cfg_buf) {
		satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
		if (!satic_cfg_buf) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for satic_cfg_buf\n");
			goto exit;
		}

		retval = testing_get_static_config(satic_cfg_buf,
						satic_cfg_length);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get static config\n");
			goto exit;
		}

		testing_hcd->satic_cfg_buf = satic_cfg_buf;
	}

	if ((testing_hcd->tx_assigned <= 0) ||
		(testing_hcd->rx_assigned <= 0) ||
		(testing_hcd->guard_assigned <= 0)) {

		if (!testing_hcd->satic_cfg_buf) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get proper satic_cfg\n");
			goto exit;
		}

		retval = testing_get_pins_mapping(testing_hcd->satic_cfg_buf,
							satic_cfg_length);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get pins mapping\n");
			goto exit;
		}
	}
#endif

	retval = testing_run_prod_test_item(TEST_PT1_TRX_TRX_SHORTS);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		return -EIO;
	}

	LOCK_BUFFER(testing_hcd->resp);

	size =
		sizeof(pt1_limits) / sizeof(pt1_limits[0]);

	if (size < testing_hcd->resp.data_length) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (i = 0; i < testing_hcd->resp.data_length; i++) {

		data = buf[i];
		LOGD(tcm_hcd->pdev->dev.parent,
				"[%d]: 0x%02x\n", i, data);
		for (j = 0; j < 8; j++) {

			phy_pin = (i*8 + j);

#ifdef PT1_GET_PIN_ASSIGNMENT
			do_pin_test = testing_is_pins_assigned(phy_pin);
#else
			do_pin_test = true;
#endif
			limit = CHECK_BIT(pt1_limits[i], j);
			if (do_pin_test) {
				if (CHECK_BIT(data, j) != limit) {
					LOGE(tcm_hcd->pdev->dev.parent,
							"pin-%2d : fail\n",
							phy_pin);
					testing_hcd->result = false;
				} else
					LOGD(tcm_hcd->pdev->dev.parent,
							"pin-%2d : pass\n",
							phy_pin);
			}
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

exit:
#ifdef PT1_GET_PIN_ASSIGNMENT
	kfree(satic_cfg_buf);
#endif

	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do reset\n");
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static int testing_pt05_full_raw(void)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned short data;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	frame_size = rows * cols * 2;

	retval = testing_run_prod_test_item(TEST_PT5_FULL_RAW_CAP);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size != testing_hcd->resp.data_length) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Frame size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt5_hi_limits) / sizeof(pt5_hi_limits[0]);
	limits_cols =
		sizeof(pt5_hi_limits[0]) / sizeof(pt5_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt5_lo_limits) / sizeof(pt5_lo_limits[0]);
	limits_cols =
		sizeof(pt5_lo_limits[0]) / sizeof(pt5_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);

			if (data  > pt5_hi_limits[row][col] ||
					data  < pt5_lo_limits[row][col]) {

				LOGE(tcm_hcd->pdev->dev.parent,
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data, pt5_lo_limits[row][col],
					pt5_hi_limits[row][col]);

				testing_hcd->result = false;
			}

			idx += 2;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do reset\n");
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static int testing_pt07_dynamic_range(void)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int data;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&frame_size_words, false);

	retval = testing_run_prod_test_item(TEST_PT7_DYNAMIC_RANGE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size_words != testing_hcd->resp.data_length / 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Frame size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt7_hi_limits) / sizeof(pt7_hi_limits[0]);
	limits_cols =
		sizeof(pt7_hi_limits[0]) / sizeof(pt7_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt7_lo_limits) / sizeof(pt7_lo_limits[0]);
	limits_cols =
		sizeof(pt7_lo_limits[0]) / sizeof(pt7_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = le2_to_uint(&buf[idx * 2]);
			if (data > pt7_hi_limits[row][col] ||
					data < pt7_lo_limits[row][col]) {

				LOGE(tcm_hcd->pdev->dev.parent,
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data, pt7_lo_limits[row][col],
					pt7_hi_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(false);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do reset\n");
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static int testing_pt10_noise(void)
{
	int retval;
	short data;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&frame_size_words, true);

	retval = testing_run_prod_test_item(TEST_PT10_DELTA_NOISE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size_words != testing_hcd->resp.data_length / 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Frame size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt10_limits) / sizeof(pt10_limits[0]);
	limits_cols =
		sizeof(pt10_limits[0]) / sizeof(pt10_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (short)le2_to_uint(&buf[idx * 2]);
			if (data > pt10_limits[row][col]) {

				LOGE(tcm_hcd->pdev->dev.parent,
					"fail at (%2d, %2d) data = %5d, limit = %4d\n",
					row, col, data, pt10_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(false);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do reset\n");
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}



static int testing_do_test_item(enum test_code test_item, int limit_rows, int limit_cols, const int* limit_data_low, const int *limit_data_high, struct file *fp, char *output_str, int str_size)
{
	int retval;
	int data, max_data, min_data, ave_data, sum_data;
	unsigned char *buf;
	int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;

	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");

	testing_hcd->result = false;
	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	max_data = 0;
	min_data = 0;
	ave_data = 0;
	sum_data = 0;

	if (fp) {
		ovt_tcm_store_to_file(fp, "\n%s do test item %d enter\n", __func__, test_item);
	}
	if (output_str) {
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str),
			"%s do test item %d enter\n", __func__, test_item);
	}

	retval = testing_run_prod_test_item(test_item);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str),
			"%s fail to get cmd response of %d\n", __func__, test_item);
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);


	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	if ((limit_cols != cols) || (limit_rows != rows)) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"incorrect cols and rows size of item:%d\n", test_item);
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str),
			"%s incorrect cols and rows size of item:%d\n", __func__, test_item);
		testing_hcd->result = false;
		goto exit;
	}


	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			int limit_low, limit_high;
			if ((limit_cols == 1) && (limit_rows == 1)) {
				if (limit_data_low)
					limit_low = limit_data_low[0];
				if (limit_data_high)
				limit_high = limit_data_high[0];
			} else {
				if (limit_data_low)
					limit_low = limit_data_low[row * limit_cols + col];
				if (limit_data_high)
					limit_high = limit_data_high[row * limit_cols + col];
			}
			if (test_item == TEST_PT10_DELTA_NOISE || test_item == TEST_PT11_OPEN_DETECTION) {
				data = (short)le2_to_uint(&buf[idx * 2]);
			} else {
				data = (unsigned short)le2_to_uint(&buf[idx * 2]);
			}
			if (idx == 0) {
				min_data = data;
				max_data = data;
			}
			if (fp) {
				ovt_tcm_store_to_file(fp, "%d,", data);
			}
			if (output_str) {
				snprintf(output_str + strlen(output_str), str_size - strlen(output_str),  "%d,", data);
			}
			if ((idx + 1) % cols == 0 && idx != 0) {
				if (fp)
					ovt_tcm_store_to_file(fp, "\n");
				if (output_str) {
					snprintf(output_str + strlen(output_str), str_size - strlen(output_str),  "\n");
				}
			}
			sum_data += data;

			if (data > max_data) {
				max_data = data;
			}
			if (data < min_data) {
				min_data = data;
			}
			if (limit_data_low) {
				if (data < limit_low) {
					testing_hcd->result = false;
					LOGE(tcm_hcd->pdev->dev.parent,
						"fail at (%2d, %2d) data = %5d, limit = %4d\n",
					row, col, data, limit_data_low[row * limit_cols + col]);
				}
			}
			if (limit_data_high) {
				if (data > limit_high) {
					testing_hcd->result = false;
					LOGE(tcm_hcd->pdev->dev.parent,
						"fail at (%2d, %2d) data = %5d, limit = %4d\n",
					row, col, data, limit_data_high[row * limit_cols + col]);
				}
			}
			idx++;
		}
	}
	if (idx != 0)
		ave_data = sum_data / idx;
	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;

exit:

	if (fp) {
		ovt_tcm_store_to_file(fp, "\n%s do test item min:%d  max:%d ave:%d\n", __func__, min_data, max_data, ave_data);
		ovt_tcm_store_to_file(fp, "\n%s do test item %d end, result is %s\n", __func__, test_item, (testing_hcd->result)?"pass":"fail");
	}
	if (output_str) {
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str), "\n%s do test item min:%d  max:%d ave:%d\n", __func__, min_data, max_data, ave_data);
		snprintf(output_str + strlen(output_str), str_size - strlen(output_str), "\n%s do test item %d end, result is %s\n", __func__, test_item, (testing_hcd->result)?"pass":"fail");
	}
	LOGN(tcm_hcd->pdev->dev.parent,
			"test item:%d Result = %s\n", test_item, (testing_hcd->result)?"pass":"fail");
	return (testing_hcd->result)? 0 : -1;
}

int lcd_id;
static int testing_do_testing(void)
{
	int retval;
	int error_count = 0;
#if 0
	unsigned int rows;
	unsigned int cols;
	struct ovt_tcm_app_info *app_info;
#endif
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	tcm_hcd->b_is_doing_test_flag = true;
	retval = tcm_hcd->reset_n_reinit(tcm_hcd, false, false);
	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		LOGE(tcm_hcd->pdev->dev.parent, "can not update to test img\n");
		error_count++;
		goto exit;
	}
#if 0
	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
#endif

#ifdef LIMIT_FROM_CSV_FILE
    uint8_t file_path[256];
	struct file *fp = NULL;
    mm_segment_t old_fs;
	loff_t pos;
    struct rtc_time rtc_now_time;
#endif

	if (!g_testing_output_buf) {
		g_testing_output_buf = vmalloc(OUTPUT_TO_CSV_STRING_LEN);
		if (!g_testing_output_buf) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"can not alloc buffer for g_testing_output_buf\n");
			return -1;
		}
	}
	memset(g_testing_output_buf, 0, OUTPUT_TO_CSV_STRING_LEN);

	snprintf(g_testing_output_buf + strlen(g_testing_output_buf), OUTPUT_TO_CSV_STRING_LEN - strlen(g_testing_output_buf),
		"start to do test,\n");

#ifdef LIMIT_FROM_CSV_FILE

	retval = ovt_tcm_get_thr_from_csvfile();
	if (retval < 0) {
		printk("ovt_tcm_get_thr_from_csvfile error, return\n");
		snprintf(g_testing_output_buf + strlen(g_testing_output_buf), OUTPUT_TO_CSV_STRING_LEN - strlen(g_testing_output_buf),
		"ovt_tcm_get_thr_from_csvfile error\n");
		error_count++;
		goto sys_err;
	}
	retval = testing_do_test_item(TEST_PT7_DYNAMIC_RANGE, TX_NUM_MAX, RX_NUM_MAX, testing_hcd->testing_csv_threshold.raw_data_min_limits,
		testing_hcd->testing_csv_threshold.raw_data_max_limits, fp, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
	if (retval < 0) {
		error_count++;
	}

	retval = testing_do_test_item(TEST_PT10_DELTA_NOISE, TX_NUM_MAX, RX_NUM_MAX, NULL,
		testing_hcd->testing_csv_threshold.lcd_noise_max_limits, fp, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
	if (retval < 0) {
		error_count++;
	}
	retval = testing_do_test_item(TEST_PT11_OPEN_DETECTION, TX_NUM_MAX, RX_NUM_MAX, testing_hcd->testing_csv_threshold.open_short_min_limits,
		testing_hcd->testing_csv_threshold.open_short_max_limits, fp, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
	if (retval < 0) {
		error_count++;
	}
#else
	if(lcd_id == 0x000d){
		retval = testing_do_test_item(TEST_PT7_DYNAMIC_RANGE, 32, 18, pt7_low_limits_new,
			pt7_hi_limits_new, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}

		retval = testing_do_test_item(TEST_PT10_DELTA_NOISE, 32, 18, NULL,
			pt10_high_limits_new, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}
		retval = testing_do_test_item(TEST_PT11_OPEN_DETECTION, 32, 18, pt11_low_limits_new,
			pt11_hi_limits_new, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}
	}
	else if(lcd_id == 0x0101){
		retval = testing_do_test_item(TEST_PT7_DYNAMIC_RANGE, 36, 26, pt7_low_limits_new1,
			pt7_hi_limits_new1, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}

		retval = testing_do_test_item(TEST_PT10_DELTA_NOISE, 36, 26, NULL,
			pt10_high_limits_new1, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}
		retval = testing_do_test_item(TEST_PT17_ADC_RANGE, 36, 26, NULL,
			pt17_hi_limits_new, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}
		retval = testing_do_test_item(TEST_PT18_HYBRID_ABS_RAW, 36, 26, pt18_low_limits_new,
			pt18_hi_limits_new, NULL, g_testing_output_buf, OUTPUT_TO_CSV_STRING_LEN);
		if (retval < 0) {
			error_count++;
		}
	}
#endif
#ifdef LIMIT_FROM_CSV_FILE
	rtc_time_to_tm(get_seconds(), &rtc_now_time);
	if (error_count) {
		//test fail result
		sprintf(file_path, "/sdcard/tp_%s_test_data_%02d%02d%02d-%02d%02d%02d-fail.csv", tcm_hcd->id_info.part_number,
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	} else {
		//test pass result
		sprintf(file_path, "/sdcard/tp_%s_test_data_%02d%02d%02d-%02d%02d%02d-success.csv", tcm_hcd->id_info.part_number,
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	}

    // old_fs = get_fs();
    // set_fs(KERNEL_DS);
    fp = filp_open(file_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (IS_ERR_OR_NULL(fp)) {
        printk("ovt tcm Open log file '%s' failed.\n", file_path);

		snprintf(g_testing_output_buf + strlen(g_testing_output_buf), OUTPUT_TO_CSV_STRING_LEN - strlen(g_testing_output_buf),
			"can not open file:%s\n", file_path);
        // set_fs(old_fs);
        goto sys_err;
    }

	pos = 0;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	vfs_write(fp, g_testing_output_buf, strlen(g_testing_output_buf), &pos);
	printk("g_testing_output_buf , strlen is %d\n", (int)strlen(g_testing_output_buf));
	if (!IS_ERR_OR_NULL(fp)) {
		printk("ovt tcm csv parser: filp close\n");
		filp_close(fp, NULL);
		fp = NULL;
	}
    set_fs(old_fs);
#else
	kernel_write(fp, g_testing_output_buf, strlen(g_testing_output_buf), &pos);
	filp_close(fp, NULL);
#endif
#endif
#ifdef LIMIT_FROM_CSV_FILE
sys_err:
#endif

exit:
	tcm_hcd->b_is_doing_test_flag = false;
	retval = tcm_hcd->reset_n_reinit(tcm_hcd, false, false);
	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		LOGE(tcm_hcd->pdev->dev.parent, "can not update to normal img\n");
	}

	if (error_count) {
		return -1;
	}
    return 0;
}

static int testing_pt11_open_detection(void)
{
	int retval;
	short data;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int image_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&image_size_words, true);

	retval = testing_run_prod_test_item(TEST_PT11_OPEN_DETECTION);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run test\n");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (image_size_words != testing_hcd->resp.data_length / 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Image size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt11_hi_limits) / sizeof(pt11_hi_limits[0]);
	limits_cols =
		sizeof(pt11_hi_limits[0]) / sizeof(pt11_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt11_lo_limits) / sizeof(pt11_lo_limits[0]);
	limits_cols =
		sizeof(pt11_lo_limits[0]) / sizeof(pt11_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Mismatching limits data\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (short)le2_to_uint(&buf[idx * 2]);
			if (data > pt11_hi_limits[row][col] ||
					data < pt11_lo_limits[row][col]) {

				LOGE(tcm_hcd->pdev->dev.parent,
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data,
					pt11_lo_limits[row][col],
					pt11_hi_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(true);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do reset\n");
		}
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static int testing_raw_data(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
#ifdef CONFIG_OVT_LOG_CAPTURE
	retval = testing_collect_reports(REPORT_RAW, raw_delta_frame_cnt);
#else
	retval = testing_collect_reports(REPORT_RAW, 1);
#endif
	if (retval >= 0) {
		LOGN(tcm_hcd->pdev->dev.parent, "success to collect raw data\n");
	} else {
		LOGN(tcm_hcd->pdev->dev.parent, "fail to collect raw data\n");
	}
	return retval;
}

static int testing_delta_data(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
#ifdef CONFIG_OVT_LOG_CAPTURE
	retval = testing_collect_reports(REPORT_DELTA, raw_delta_frame_cnt);
#else
	retval = testing_collect_reports(REPORT_DELTA,1);
#endif
	if (retval >= 0) {
		LOGN(tcm_hcd->pdev->dev.parent, "success to collect raw data\n");
	} else {
		LOGN(tcm_hcd->pdev->dev.parent, "fail to collect raw data\n");
	}
	return retval;
}

static int testing_reset_open(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	LOGN(tcm_hcd->pdev->dev.parent,
			"Start testing\n");
	testing_hcd->result = false;

	if (bdata->reset_gpio < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Hardware reset unavailable\n");
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->reset_mutex);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, false);
#endif
	ovt_tcm_request_gpio(tcm_hcd, bdata->reset_gpio, true);
	gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
	msleep(bdata->reset_active_ms);
	gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
	msleep(bdata->reset_delay_ms);
	ovt_tcm_request_gpio(tcm_hcd, bdata->reset_gpio, false);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	mutex_unlock(&tcm_hcd->reset_mutex);

	if (tcm_hcd->id_info.mode == MODE_APPLICATION_FIRMWARE) {
		retval = tcm_hcd->switch_mode(tcm_hcd, FW_MODE_BOOTLOADER);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enter bootloader mode\n");
			return retval;
		}
	} else {
		retval = tcm_hcd->identify(tcm_hcd, false);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do identification\n");
			goto run_app_firmware;
		}
	}

	if (tcm_hcd->boot_info.last_reset_reason == reset_open_limit)
		testing_hcd->result = true;
	else
		testing_hcd->result = false;

	retval = 0;

run_app_firmware:
	if (tcm_hcd->switch_mode(tcm_hcd, FW_MODE_APPLICATION) < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run application firmware\n");
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static void testing_report(void)
{
	int retval;
	unsigned int offset;
	unsigned int report_size;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	report_size = tcm_hcd->report.buffer.data_length;

	LOCK_BUFFER(testing_hcd->report);

	if (testing_hcd->report_index == 0) {
		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&testing_hcd->report,
				report_size * testing_hcd->num_of_reports);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for testing_hcd->report.buf\n");
			UNLOCK_BUFFER(testing_hcd->report);
			return;
		}
	}

	if (testing_hcd->report_index < testing_hcd->num_of_reports) {
		offset = report_size * testing_hcd->report_index;

		retval = secure_memcpy(testing_hcd->report.buf + offset,
				testing_hcd->report.buf_size - offset,
				tcm_hcd->report.buffer.buf,
				tcm_hcd->report.buffer.buf_size,
				tcm_hcd->report.buffer.data_length);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to copy report data\n");
			UNLOCK_BUFFER(testing_hcd->report);
			return;
		}

		testing_hcd->report_index++;
		testing_hcd->report.data_length += report_size;
	}

	UNLOCK_BUFFER(testing_hcd->report);

	if (testing_hcd->report_index == testing_hcd->num_of_reports)
		complete(&report_complete);

	return;
}

#ifdef CONFIG_OVT_SELFTEST_ENABLE
static void *ovt_selftest_seq_start(struct seq_file *m, loff_t *pos)
{
    return *pos < 1 ? (void *)1 : NULL;
}

static void *ovt_selftest_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
    ++*pos;
    return NULL;
}

static int ovt_selftest_seq_show(struct seq_file *m, void *v)
{
	int retval;
	int test_result;
	char *buf = kmalloc(PAGE_SIZE, GFP_KERNEL);

    if (!buf) {
        return -ENOMEM;
    }
	//struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	//mutex_lock(&tcm_hcd->extif_mutex);

	test_result = testing_do_testing();

	if (test_result) {
		retval = snprintf(buf, PAGE_SIZE, "%s\n","fail");
	} else {
		retval = snprintf(buf, PAGE_SIZE, "%s\n","pass");
	}
	seq_puts(m, buf);
	kfree(buf);
	return 0;
}
static void ovt_selftest_seq_stop(struct seq_file *m, void *v)
{
    return;
}

static const struct seq_operations ovt_selftest_seq_ops = {
    .start = ovt_selftest_seq_start,
	.next = ovt_selftest_seq_next,
    .show = ovt_selftest_seq_show,
    .stop = ovt_selftest_seq_stop,
};

static int ovt_selftest_open(struct inode *inode, struct file *file)
{
    return seq_open(file, &ovt_selftest_seq_ops);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0)
static const struct proc_ops ovt_selftest_fops = {
	.proc_open = ovt_selftest_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations ovt_selftest_fops = {
	.owner = THIS_MODULE,
	.open = ovt_selftest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif
#endif

static int testing_init(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	int idx;

	testing_hcd = kzalloc(sizeof(*testing_hcd), GFP_KERNEL);
	if (!testing_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for testing_hcd\n");
		return -ENOMEM;
	}

	testing_hcd->tcm_hcd = tcm_hcd;

	testing_hcd->collect_reports = testing_collect_reports;

	INIT_BUFFER(testing_hcd->out, false);
	INIT_BUFFER(testing_hcd->resp, false);
	INIT_BUFFER(testing_hcd->report, false);
	INIT_BUFFER(testing_hcd->process, false);
	INIT_BUFFER(testing_hcd->output, false);

	testing_hcd->sysfs_dir = kobject_create_and_add(SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!testing_hcd->sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create sysfs directory\n");
		retval = -EINVAL;
		goto err_sysfs_create_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		//retval = sysfs_create_file(testing_hcd->sysfs_dir,
		retval = sysfs_create_file(&tcm_hcd->pdev->dev.kobj,	//default path
				&(*attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create sysfs file\n");
			goto err_sysfs_create_file;
		}
	}

	retval = sysfs_create_bin_file(testing_hcd->sysfs_dir, &bin_attr);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create sysfs bin file\n");
		goto err_sysfs_create_bin_file;
	}

#ifdef CONFIG_OVT_SELFTEST_ENABLE
	if (!touch_info_dir) {
		touch_info_dir = proc_mkdir("touch_info", NULL);
	}
	if (!touch_info_dir) {
		LOGE(tcm_hcd->pdev->dev.parent, "Can not create touch_info_dir\n");
		return -ENOMEM;
	}

	testing_hcd->ovt_selftest_proc_entry = proc_create("tp_selftest_result", 0644, touch_info_dir, &ovt_selftest_fops);
	if (testing_hcd->ovt_selftest_proc_entry == NULL) {
		LOGE(tcm_hcd->pdev->dev.parent, "create /proc/touch_info/tp_selftest_result Failed!\n");
		return -1;
	}
#endif

	testing_hcd->proc_csv = proc_create_data("ovt_test_csv", 0777, NULL, &ovt_proccsv_fops, NULL);
	if (NULL == testing_hcd->proc_csv) {
		LOGE(tcm_hcd->pdev->dev.parent, "create proc_csv entry fail");
	}
	return 0;

err_sysfs_create_bin_file:
err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(testing_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(testing_hcd->sysfs_dir);

err_sysfs_create_dir:
	RELEASE_BUFFER(testing_hcd->output);
	RELEASE_BUFFER(testing_hcd->process);
	RELEASE_BUFFER(testing_hcd->report);
	RELEASE_BUFFER(testing_hcd->resp);
	RELEASE_BUFFER(testing_hcd->out);

	kfree(testing_hcd);
	testing_hcd = NULL;

	return retval;
}

static int testing_remove(struct ovt_tcm_hcd *tcm_hcd)
{
	int idx;

	if (!testing_hcd)
		goto exit;

	sysfs_remove_bin_file(testing_hcd->sysfs_dir, &bin_attr);

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++)
		sysfs_remove_file(testing_hcd->sysfs_dir, &(*attrs[idx]).attr);

#ifdef CONFIG_OVT_SELFTEST_ENABLE
	if (testing_hcd->ovt_selftest_proc_entry != NULL) {
		remove_proc_entry("tp_selftest_result", touch_info_dir);
		testing_hcd->ovt_selftest_proc_entry = NULL;
		LOGN(tcm_hcd->pdev->dev.parent,"Removed /proc/touch_info/tp_selftest_result\n");
	}
	if (touch_info_dir != NULL) {
		remove_proc_entry("touch_info", NULL);
		touch_info_dir = NULL;
		LOGE(tcm_hcd->pdev->dev.parent,"Removed /proc/touch_info\n");
	}
#endif
	kobject_put(testing_hcd->sysfs_dir);

	RELEASE_BUFFER(testing_hcd->output);
	RELEASE_BUFFER(testing_hcd->process);
	RELEASE_BUFFER(testing_hcd->report);
	RELEASE_BUFFER(testing_hcd->resp);
	RELEASE_BUFFER(testing_hcd->out);
	if (testing_hcd->proc_csv) {
		proc_remove(testing_hcd->proc_csv);
	}
#ifdef CONFIG_OVT_SELFTEST_ENABLE
	if (testing_hcd->ovt_selftest_proc_entry) {
		proc_remove(testing_hcd->ovt_selftest_proc_entry);
	}
#endif
	kfree(testing_hcd);
	testing_hcd = NULL;

exit:
	complete(&testing_remove_complete);

	return 0;
}

static int testing_reinit(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	if (!testing_hcd) {
		retval = testing_init(tcm_hcd);
		return retval;
	}

	return 0;
}

static int testing_syncbox(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!testing_hcd)
		return 0;

	if (tcm_hcd->report.id == testing_hcd->report_type)
		testing_report();

	return 0;
}

static struct ovt_tcm_module_cb testing_module = {
	.type = TCM_TESTING,
	.init = testing_init,
	.remove = testing_remove,
	.syncbox = testing_syncbox,
#ifdef REPORT_NOTIFIER
	.asyncbox = NULL,
#endif
	.reinit = testing_reinit,
	.suspend = NULL,
	.resume = NULL,
	.early_suspend = NULL,
};

int testing_module_init(void)
{
	return ovt_tcm_add_module(&testing_module, true);
}
EXPORT_SYMBOL(testing_module_init);

void testing_module_exit(void)
{
	ovt_tcm_add_module(&testing_module, false);

	wait_for_completion(&testing_remove_complete);

	return;
}
EXPORT_SYMBOL(testing_module_exit);
/*
module_init(testing_module_init);
module_exit(testing_module_exit);

MODULE_AUTHOR("Omnivision, Inc.");
MODULE_DESCRIPTION("Omnivision TCM Testing Module");
MODULE_LICENSE("GPL v2");*/
