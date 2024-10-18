/* Copyright (c) 2024, Motorola Mobility LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "bk_log (%s): " fmt, __func__

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/init_syscalls.h>
#include <linux/panic_notifier.h>
#include <linux/kdebug.h>
#include <trace/hooks/sysrqcrash.h>

#define LOGS_NUM 5
#define WRITE_EXPIRE 1000 // 1s
#define BLKDEV_WAIT_TIME (10*1000) // 10s
#define WRITE_SIZE_ONCE (100*1024) // 100KB
#define MAX_LOG_SIZE (32UL*1024*1024) // 32MB

#define PARTNAME_SIZE 32
#define BK_PARTLABEL "PARTLABEL="
#define BK_PARTITION_NAME "logks" // default store partition
#define BK_PARTLABEL_NAME BK_PARTLABEL BK_PARTITION_NAME

struct log_entry {
	char *buffer;
	unsigned size;
	struct list_head entry;
};

struct bk_log {
	char partlabel_name[PARTNAME_SIZE];
	/* always wake up thread to write log to storage */
	bool always_on;
	bool stop_console;
	char log_cycle;
	/* previous page buffer */
	char *pre_page;
	/* size of previous page */
	unsigned int pre_size;
	/* the bio write pos*/
	size_t bio_pos;
	size_t written_size;
	size_t left_size;
	size_t block_size;
	size_t partition_size;
	struct list_head log_head;
	struct task_struct *store_thread;
	struct block_device *blkdev;
	spinlock_t list_lock;
	wait_queue_head_t log_wq;
};
struct bk_log *bklog;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#define PDE_DATA(x) pde_data(x)
#endif

static struct page *addr_to_page(void *addr)
{
	if (is_vmalloc_addr(addr))
		return vmalloc_to_page(addr);
	else
		return virt_to_page(addr);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#define NR_BIO_MAX_PAGES BIO_MAX_VECS
#else
#define NR_BIO_MAX_PAGES BIO_MAX_PAGES
#endif

static int bk_submit_bio(struct block_device *bdev, void *buf, int pages, int opf, loff_t pos)
{
	int i, ret = 0;
	struct bio *bio;
	int num, left_pages = pages;

	pr_debug("%s: pages %d left_pages %d begin\n", __func__, pages, left_pages);
	while (left_pages > 0) {
		num = (left_pages >= NR_BIO_MAX_PAGES) ? NR_BIO_MAX_PAGES : left_pages;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		bio = bio_alloc(bdev, num, 0, GFP_KERNEL);
#else
		bio = bio_alloc(GFP_KERNEL, num);
#endif
		if (!bio)
			return -ENOMEM;

		bio->bi_iter.bi_sector = (pos / PAGE_SIZE + pages - left_pages) * (PAGE_SIZE >> 9);
		bio->bi_opf = opf;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		bio_set_dev(bio, bdev);
#endif

		for (i = 0; i < num; i++) {
			if (!bio_add_page(bio, addr_to_page(buf + (pages - left_pages + i) * PAGE_SIZE), PAGE_SIZE, 0)) {
				bio_put(bio);
				return -EIO;
			}
		}

		ret = submit_bio_wait(bio);
		if (ret)
			pr_err("Submit bio err %d,%d,%d", num, opf, ret);

		bio_put(bio);
		left_pages -= num;
		pr_debug("%s: pages %d left_pages %d\n", __func__, pages, left_pages);
	}
	return ret;
}

static ssize_t rw_bdev(struct block_device *bdev, void *buf, size_t count, int opf, loff_t pos)
{
	int ret;

	ret = bk_submit_bio(bdev, buf, DIV_ROUND_UP(count, PAGE_SIZE), opf, pos);
	return  ret < 0 ? ret : count;
}

static ssize_t kernel_write_stub(struct block_device *bdev, void *buf, size_t count, loff_t pos)
{
	return rw_bdev(bdev, buf, count, REQ_OP_WRITE | REQ_SYNC, pos);
}

static ssize_t kernel_read_stub(struct block_device *bdev, void *buf, size_t count, loff_t pos)
{
	return rw_bdev(bdev, buf, count, REQ_OP_READ, pos);
}

static void bk_console_write(struct console *con, const char *s, unsigned c)
{
	struct log_entry *log;
	size_t left;

	if (!c || !bklog || bklog->stop_console)
		return;

	log = kzalloc(sizeof(struct log_entry), GFP_ATOMIC | __GFP_NOFAIL);
	if (!log)
		return;

	log->buffer = kzalloc(c, GFP_ATOMIC | __GFP_NOFAIL);
	if (!log->buffer)
		return;

	memcpy(log->buffer, s, c);
	log->size = c;

	spin_lock(&bklog->list_lock);
	list_add_tail(&log->entry, &bklog->log_head);
	left = bklog->left_size = bklog->left_size + log->size;
	spin_unlock(&bklog->list_lock);

	if (left > WRITE_SIZE_ONCE)
		wake_up(&bklog->log_wq);
}

static struct console bk_console = {
	.write	= bk_console_write,
	.index	= -1,
};

static void force_flush_logs(void)
{
	unsigned int delay_time;

	bklog->always_on = true;
	wake_up(&bklog->log_wq);

	if (bklog->left_size) {
		delay_time = max((int)100, (int) (10 * DIV_ROUND_UP(bklog->left_size, PAGE_SIZE)));
		pr_info("delay %ums to write %zuB logs\n", delay_time, bklog->left_size);
		mdelay(delay_time);
	}
}

static int bk_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	if (!bklog || bklog->stop_console)
		return 0;

	pr_info("bk log reboot notify");
	force_flush_logs();

	return 0;
}

static struct notifier_block bk_reboot_notify = {
	.notifier_call = bk_reboot,
	.priority = INT_MAX, // highest priority
};

/*
static struct notifier_block bk_panic_notify = {
	.notifier_call = bk_reboot,
	.priority = INT_MAX,
};
*/

static struct notifier_block bk_die_notify = {
	.notifier_call = bk_reboot,
	.priority = INT_MAX,
};

static void __nocfi sysrq_crash(void *p, void *data)
{
	if (!bklog || bklog->stop_console)
		return;

	pr_info("sysrq crash");
	force_flush_logs();
}

static int get_blkdev(struct bk_log *bk)
{
	dev_t dev;

	if (bk->blkdev)
		return 0;

	dev = name_to_dev_t(bk->partlabel_name);
	if (!dev) {
		pr_debug("Failed to get device %s!\n", bk->partlabel_name);
		return -ENOTBLK;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	bk->blkdev = blkdev_get_by_dev(dev, BLK_OPEN_READ | BLK_OPEN_WRITE | BLK_OPEN_EXCL, bk, NULL);
#else
	// open with FMODE_EXCL flag so that the other processes cann't open it
	bk->blkdev = blkdev_get_by_dev(dev, FMODE_READ | FMODE_WRITE | FMODE_EXCL, bk);
#endif
	if (IS_ERR(bk->blkdev)) {
		pr_err("Failed get block device %s\n", bk->partlabel_name);
		return -ENOTBLK;
	}
	bk->partition_size = i_size_read(bk->blkdev->bd_inode);
	bk->block_size = rounddown((min(bk->partition_size, MAX_LOG_SIZE) - PAGE_SIZE) / bk->log_cycle, PAGE_SIZE);
	pr_info("Get device %s, size:%zu\n", bk->partlabel_name, bk->block_size);

	return 0;
}

static void erase_block(struct block_device *blkdev, size_t size, size_t pos)
{
	int i;
	char *buffer;
	unsigned long start_time = jiffies;

	buffer = vzalloc(size);
	if (!buffer) {
		pr_err("Failed to alloc %zu memory", size);
		return;
	}

	if (!pos)
		goto erase;

	kernel_read_stub(blkdev, buffer, PAGE_SIZE, pos);
	for (i = 0; i < PAGE_SIZE; i++) {
		if (buffer[i] != 0)
			break;
	}

	if (i == PAGE_SIZE) {
		pr_debug("The first page is all 0, takes %u ms \n", jiffies_to_msecs(jiffies - start_time));
		return;
	}

	memset(buffer, 0, PAGE_SIZE);

erase:
	kernel_write_stub(blkdev, buffer, size, pos);
	vfree(buffer);

	pr_info("It takes %u ms to erase block\n", jiffies_to_msecs(jiffies - start_time));
}

static bool update_bio_pos(struct block_device *blkdev, size_t block_size, size_t partition_size, char cycle, size_t *pos)
{
	char *buffer;
	int count = 0;
	bool erase_all = false;
	size_t erase_size, erase_pos;

	buffer = vzalloc(PAGE_SIZE);
	if (!buffer) {
		pr_err("Failed to alloc a page memory");
		return erase_all;
	}
	kernel_read_stub(blkdev, buffer, PAGE_SIZE, 0);

	sscanf(buffer, "%d\n", &count);
	if (count < 0 || count > cycle) {
		count = cycle;
		erase_all = true;
	}

	count = (count % cycle) + 1;
	*pos = PAGE_SIZE + (count -1) * block_size;

	erase_size = erase_all ? partition_size : block_size;
	erase_pos = erase_all ? 0 : *pos;
	erase_block(blkdev, erase_size, erase_pos);

	sprintf(buffer, "%d\n", count);
	kernel_write_stub(blkdev, buffer, PAGE_SIZE, 0);
	vfree(buffer);

	pr_debug("update pos:%d, %zu, %zu, %zu\n", count, *pos, erase_size, block_size);
	return erase_all;
}

static int store_work_thread(void *data)
{
	size_t total_size = 0, io_write_size = 0, left_size = 0;
	struct log_entry *log;
	struct bk_log *bk = (struct bk_log *)data;
	unsigned long start_time;
	int ret = 0;
	char *buffer;

	start_time = jiffies;
	while (true) {
		if (!get_blkdev(bk))
			break;

		if (time_after(jiffies, start_time + msecs_to_jiffies(BLKDEV_WAIT_TIME))) {
			pr_err("The device %s is not existent, thread exit!\n", bk->partlabel_name);
			ret = -ENOTBLK;
			goto out;
		}

		mdelay(10);
	}

	update_bio_pos(bk->blkdev, bk->block_size, bk->partition_size, bk->log_cycle, &bk->bio_pos);

	bk->pre_page = vzalloc(PAGE_SIZE);
	if (!bk->pre_page) {
		pr_err("Failed to malloc buffer\n");
		ret = -ENOMEM;
		goto out;
	}

	bk->pre_page[0] = '\n';
	bk->pre_size = 1;

	while (!kthread_should_stop() && !bk->stop_console) {
		wait_event_interruptible_timeout(bk->log_wq, bk->left_size > WRITE_SIZE_ONCE || bk->always_on, msecs_to_jiffies(WRITE_EXPIRE));
		left_size = bk->left_size;
		pr_debug("thread wake! %zu,%u\n", left_size, bk->pre_size);
		if (list_empty(&bk->log_head) || !left_size)
			continue;

		buffer = vzalloc(left_size + bk->pre_size);
		if (!buffer) {
			pr_err("Failed to malloc buffer %zu\n", left_size + bk->pre_size);
			continue;
		}

		total_size = bk->pre_size;
		if (total_size)
			memcpy(buffer, bk->pre_page , total_size);

		spin_lock(&bk->list_lock);
		// compose small log fragment into bio page
		while (!list_empty(&bk->log_head)) {
			log = list_first_entry(&bk->log_head, struct log_entry, entry);

			if (!log || !log->buffer || !log->size ) {
				if (log && log->buffer)
					kfree(log->buffer);

				if (log) {
					list_del(&log->entry);
					kfree(log);
				}
				continue;
			}

			if (bk->written_size + total_size - bk->pre_size + log->size > bk->block_size) {
				bk->stop_console = true;
				pr_info("Stop!!! total size %zu\n", bk->written_size);
				break;
			}

			if (total_size + log->size > left_size + bk->pre_size)
				break;

			memcpy(buffer + total_size, log->buffer, log->size);
			total_size += log->size;
			list_del(&log->entry);
			kfree(log->buffer);
			kfree(log);
		}

		bk->left_size -= total_size - bk->pre_size;
		spin_unlock(&bk->list_lock);

		io_write_size = total_size < PAGE_SIZE ? total_size : rounddown(total_size, PAGE_SIZE);
		kernel_write_stub(bk->blkdev, buffer, io_write_size, bk->bio_pos);

		bk->bio_pos += total_size - bk->pre_size;
		bk->written_size += total_size - bk->pre_size ;
		bk->pre_size = total_size & (PAGE_SIZE - 1);
		if (bk->pre_size != 0) {
			// Copy previous logs to compose the next page
			memcpy(bk->pre_page, buffer + rounddown(total_size, PAGE_SIZE), bk->pre_size);
		}

		pr_debug("Write size %zu,%zu,%lu,%zu\n", io_write_size, total_size, rounddown(total_size, PAGE_SIZE), bk->bio_pos);
		vfree(buffer);
	}

out:
	bklog->store_thread = NULL;
	return ret;
}

static int get_bootargs(char *key, char *value, int size)
{
	int ret = false;
	const char *bootargs_ptr = NULL;
	char *bootargs_str = NULL;
	char *idx = NULL;
	char *kvpair = NULL, *kvalue = NULL;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_ptr_len = 0;

	if (n == NULL)
		goto err;

	if (of_property_read_string(n, "mmi,bootconfig", &bootargs_ptr) != 0)
		goto err_putnode;

	bootargs_ptr_len = strlen(bootargs_ptr);
	/* Following operations need a non-const version of bootargs */
	bootargs_str = kzalloc(bootargs_ptr_len + 1, GFP_KERNEL);
	if (!bootargs_str)
		goto err_putnode;

	strlcpy(bootargs_str, bootargs_ptr, bootargs_ptr_len + 1);

	idx = strnstr(bootargs_str, key, strlen(bootargs_str));

	if (!idx)
		goto out;

	kvpair = strsep(&idx, " ");
	if (kvpair && strsep(&kvpair, "="))
		kvalue = strsep(&kvpair, "\n");

	if (kvalue && size >= strlen(kvalue)) {
		ret = true;
		memcpy(value, kvalue, strlen(kvalue));
	}

out:
	kfree(bootargs_str);
err_putnode:
	of_node_put(n);
err:
	return ret;
}

static int get_part_name(char *name)
{
	char partition_name[PARTNAME_SIZE] = { 0 };

	if (get_bootargs("androidboot.bklog_part=", partition_name, PARTNAME_SIZE)) {
		strscpy(name, BK_PARTLABEL, PARTNAME_SIZE);
		strlcat(name, partition_name, PARTNAME_SIZE - strlen(BK_PARTLABEL));
		return true;
	}

	return false;
}

static int get_cycle(void)
{
	char buffer[10] = { 0 };
	int cycle = 0;

	if (get_bootargs("androidboot.bklog_cycle=", buffer, 10))
		sscanf(buffer, "%d\n", &cycle);

	return cycle;
}

static int allow_to_dump(void)
{
	char enable[PARTNAME_SIZE] = { 0 };

	if (get_bootargs("androidboot.dump_bklog=", enable, PARTNAME_SIZE) &&
		(strcmp(enable, "true") == 0 || strcmp(enable, "1") == 0))
		return true;

	if (get_bootargs("androidboot.bklog_part=", enable, PARTNAME_SIZE) && enable[0] != 0)
		return true;

	if (get_bootargs("androidboot.bklog_cycle=", enable, PARTNAME_SIZE) && enable[0] != 0)
		return true;

	return false;
}

static __init int bk_log_init(void)
{
	int ret = 0;

	if (!allow_to_dump()) {
		pr_info("Not allow to dump\n");
		return 0; //should return 0 or else first init will reboot to bootloader
	}

	bklog = kzalloc(sizeof(struct bk_log), GFP_KERNEL);
	if (!bklog)
		return -ENOMEM;
	bklog->always_on = false;

	if (!get_part_name(bklog->partlabel_name))
		memcpy(bklog->partlabel_name, BK_PARTLABEL_NAME, strlen(BK_PARTLABEL_NAME));

	bklog->log_cycle = get_cycle();
	if (!bklog->log_cycle)
		bklog->log_cycle = LOGS_NUM;

	INIT_LIST_HEAD(&bklog->log_head);
	init_waitqueue_head(&bklog->log_wq);
	spin_lock_init(&bklog->list_lock);

	bklog->store_thread = kthread_run(store_work_thread, (void *)bklog, "store_bklog");
	if (IS_ERR(bklog->store_thread)) {
		pr_err("Failed to create thread\n");
		ret = -ENOMEM;
		goto out_error;
	}

	strscpy(bk_console.name, "bklog", sizeof(bk_console.name));
	bk_console.flags = CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME;
	register_console(&bk_console);

	register_reboot_notifier(&bk_reboot_notify);
	register_die_notifier(&bk_die_notify);
	register_trace_android_vh_sysrq_crash(sysrq_crash, NULL);
	//atomic_notifier_chain_register(&panic_notifier_list, &bk_panic_notify);

	return 0;

out_error:
	kfree(bklog);
	return ret;
}

static void __exit bk_log_exit(void)
{
	struct log_entry *log, *tmp;

	if (bklog == NULL)
		return;

	bklog->stop_console = true;
	if (bklog->store_thread)
		kthread_stop(bklog->store_thread);

	unregister_console(&bk_console);
	unregister_reboot_notifier(&bk_reboot_notify);
	unregister_die_notifier(&bk_die_notify);
	unregister_trace_android_vh_sysrq_crash(sysrq_crash, NULL);
	//atomic_notifier_chain_unregister(&panic_notifier_list, &bk_panic_notify);

	if (bklog->blkdev) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
		blkdev_put(bklog->blkdev, bklog);
#else
		blkdev_put(bklog->blkdev, FMODE_READ | FMODE_WRITE | FMODE_EXCL);
#endif
		bklog->blkdev = NULL;
	}

	spin_lock(&bklog->list_lock);
	list_for_each_entry_safe(log, tmp, &bklog->log_head, entry) {
		list_del(&log->entry);
		kfree(log->buffer);
		kfree(log);
	}
	spin_unlock(&bklog->list_lock);

	if (bklog->pre_page) {
		vfree(bklog->pre_page);
		bklog->pre_page = NULL;
	}

	if (bklog) {
		kfree(bklog);
		bklog = NULL;
	}
}

module_init(bk_log_init);
module_exit(bk_log_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Store boot kernel log to storage");
