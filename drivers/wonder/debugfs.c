// SPDX-License-Identifier: GPL-2.0
/*
 * Google Wonder WiFi Virtual Soft-MAC Driver
 *
 * Debugfs implementation for the Wonder driver.
 */
#define LOG_MODULE_NAME "debugfs"

#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "core.h"
#include "wonder_log.h"
#include "mac80211.h"
#include "wondertap_internal.h"

static struct dentry *wonder_debugfs_root;

static int wonder_channel_status_report_show(struct seq_file *m, void *v)
{
	struct wonder_data *wonder = m->private;
	struct wondertap_data *wondertap = &wonder->wondertap_data;
	struct wondertap_channel_status_report *report;
	u32 num_channels;
	size_t size;
	int ret, i;

	mutex_lock(&wondertap->lock);
	num_channels = wondertap->cached_channel_schedule.channel_list_len;
	mutex_unlock(&wondertap->lock);

	if (num_channels == 0) {
		seq_puts(m, "Channel hopping list is empty.\n");
		return 0;
	}

	size = sizeof(*report) + num_channels * sizeof(struct wondertap_channel_status);
	report = kzalloc(size, GFP_KERNEL);
	if (!report)
		return -ENOMEM;

	report->channel_status_len = num_channels;
	ret = wondertap_get_channel_status_report(wondertap, report);
	if (ret) {
		seq_printf(m, "Failed to get channel status report: %d\n", ret);
		kfree(report);
		return 0;
	}

	seq_printf(m, "Current Hopping Request TSF: 0x%08x\n",
		report->current_channel_hopping_request_tsf);
	seq_printf(m, "Current Channel Index: %u\n", report->current_channel_index);
	seq_printf(m, "Channel Status Length: %u\n", report->channel_status_len);

	for (i = 0; i < report->channel_status_len; i++) {
		struct wondertap_channel_status *status = &report->status[i];
		seq_printf(m, "\n  [%d] Freq: %u MHz\n", i, status->freq);
		seq_printf(m, "      Switch TSF: 0x%08x\n", status->channel_switch_tsf);
		seq_printf(m, "      Start TSF:  0x%08x\n", status->channel_start_tsf);
		seq_printf(m, "      End TSF:    0x%08x\n", status->channel_end_tsf);
		seq_printf(m, "      TX: %u frames, %llu bytes\n", status->tx_frames,
			status->tx_bytes);
		seq_printf(m, "      RX: %u frames, %llu bytes\n", status->rx_frames,
			status->rx_bytes);
	}

	kfree(report);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(wonder_channel_status_report);

static int wonder_channel_schedule_request_show(struct seq_file *m, void *v)
{
	struct wonder_data *wonder = m->private;
	struct wondertap_data *wondertap = &wonder->wondertap_data;
	struct channel_schedule_request *schedule;
	int i;

	mutex_lock(&wondertap->lock);
	schedule = &wondertap->cached_channel_schedule;

	if (schedule->channel_list_len == 0) {
		seq_puts(m, "Channel hopping schedule list is empty.\n");
		mutex_unlock(&wondertap->lock);
		return 0;
	}

	seq_printf(m, "Channel List Length: %u\n", schedule->channel_list_len);
	seq_printf(m, "Next Channel Index: %u\n", schedule->next_channel_index);
	seq_printf(m, "Dwell Time (TU): %u\n", schedule->dwell_time_tu);
	seq_printf(m, "Target Switch TSF: 0x%08x\n", schedule->target_switch_time_tsf);

	seq_puts(m, "\nChannel List:\n");
	if (schedule->channel_list) {
		for (i = 0; i < schedule->channel_list_len; i++) {
			seq_printf(m, "  [%d] Freq: %u MHz, BW: %u, Role: %u\n", i,
				   schedule->channel_list[i].freq,
				   schedule->channel_list[i].bandwidth,
				   schedule->channel_list[i].role);
		}
	}

	mutex_unlock(&wondertap->lock);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(wonder_channel_schedule_request);

int wonder_debugfs_init(void *wonder)
{
	wonder_debugfs_root = debugfs_create_dir("wonder", NULL);
	if (!wonder_debugfs_root)
		return -ENODEV;

	debugfs_create_file("channel_status_report", 0644, wonder_debugfs_root,
			    wonder, &wonder_channel_status_report_fops);
	debugfs_create_file("channel_schedule_request", 0644, wonder_debugfs_root,
			    wonder, &wonder_channel_schedule_request_fops);
	debugfs_create_bool("amsdu_enable", 0644, wonder_debugfs_root,
			    &((struct wonder_data *)wonder)->amsdu_enable);
	debugfs_create_u32("amsdu_threshold", 0644, wonder_debugfs_root,
			    &((struct wonder_data *)wonder)->amsdu_threshold);
	debugfs_create_u32("amsdu_delay", 0644, wonder_debugfs_root,
			    &((struct wonder_data *)wonder)->amsdu_delay);
	return 0;
}

void wonder_debugfs_exit(void)
{
	debugfs_remove_recursive(wonder_debugfs_root);
}