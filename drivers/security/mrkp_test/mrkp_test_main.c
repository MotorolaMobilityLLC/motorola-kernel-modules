// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/pgalloc.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/init.h>
#include <linux/kprobes.h>
#include <linux/list.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pagewalk.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include <trace/hooks/avc.h>
#include <trace/hooks/creds.h>
#include <trace/hooks/syscall_check.h>

#include "mrkp_test.h"

#define TEST(unused, func)\
	static int func(struct seq_file *m)

#define DEFINE_TEST(testcase)\
static int testcase##_show(struct seq_file *m, void *v) {return testcase(m);}\
\
static int testcase##_open(struct inode *inode, struct file *file){\
	return single_open(file, testcase##_show, NULL);\
}\
static const struct file_operations testcase##_fops = {\
	.open = testcase##_open,\
	.read = seq_read,\
	.llseek = seq_lseek,\
	.release = single_release,\
};

#define ADD_TEST(dir, testcase)\
debugfs_create_file(#testcase, 0444, dir, NULL, &testcase##_fops);


#define mrkp_test_msg(fmt, args...)      seq_printf(m, "[mrkp_test] "fmt"\n", ##args)

static noinline void __perform_attack(struct seq_file *m, void *taddr)
{
	*(char *)(taddr) = '0';
	if (*(volatile char*)(taddr) == '0')
		mrkp_test_msg("attack succeeded\n");
	else
		mrkp_test_msg("attack failed\n");
}

static void try_to_attack(struct seq_file *m, unsigned long addr, bool is_resource_pa)
{
	phys_addr_t phys_addr;
	struct page *tpage;
	void *taddr;

	if (is_resource_pa) {
		taddr = ioremap(addr, 4096);
	} else {
		phys_addr = __virt_to_phys(addr);
		tpage = phys_to_page(phys_addr);
		taddr = vmap(&tpage, 1, VM_MAP, PAGE_KERNEL);
	}

	if (!taddr) {
		seq_puts(m, "invalid taddr, abort test\n");
		return;
	}

	__perform_attack(m, taddr);
}

static void try_to_attack_ko(struct seq_file *m, unsigned long addr)
{
	phys_addr_t phys_addr;
	struct page *tpage;
	void *taddr;

	phys_addr = vmalloc_to_pfn((void *)addr) << PAGE_SHIFT;
	tpage = phys_to_page(phys_addr);
	taddr = vmap(&tpage, 1, VM_MAP, PAGE_KERNEL);
	if (!taddr) {
		seq_puts(m, "abort test\n");
		return;
	}

	__perform_attack(m, taddr);
}

/******************************************************
 * Test mrkp_module_loaded
 ******************************************************/
TEST(mrkp_test, mrkp_test_001_ko)
{
	const char *name = "moto_kern_prot";
	struct module *pMod, *list_mod;
	int ret = -1;

	mrkp_test_msg(" [SCN001]: check mrkp driver alive\n");
	pMod = NULL;
	preempt_disable();
	list_for_each_entry(list_mod, THIS_MODULE->list.prev, list) {
		if (strcmp(list_mod->name, name) == 0 ){
			pMod = list_mod;
			break;
		}
	}
	preempt_enable();
	mrkp_test_msg(" [SCN001]: module address : %lx\n", (unsigned long)pMod);
	if (!pMod)
		return 0;

	ret = pMod->state;
	if (ret==0)
		seq_puts(m, "PASS\n");

	return 0;
}

/******************************************************
 * Test mrkp_self_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_002_mrkp)
{
	const char *name = "moto_kern_prot";
	struct module *pMod, *list_mod;
	const struct module_memory* mrkp_module_text;
	unsigned long ro_addr;
	mrkp_test_msg(" [SCN002]: attack mrkp (no toleration)\n");
	pMod = NULL;
	preempt_disable();
	list_for_each_entry(list_mod, THIS_MODULE->list.prev, list) {
		if (strcmp(list_mod->name, name) == 0 ){
			pMod = list_mod;
			break;
		}
	}
	preempt_enable();
	if (!pMod)
		return 0;

	mrkp_module_text = &pMod->mem[MOD_TEXT];
	ro_addr = (unsigned long)mrkp_module_text,
	mrkp_test_msg(" [SCN002]: rodata address: %lx\n", ro_addr);

	/* Start attack */
	try_to_attack_ko(m, ro_addr);

	/* Dump result */
	seq_puts(m, "KE! should not come here\n");

	return 0;
}

/******************************************************
 * Test driver_protection(against any module corruption)
 ******************************************************/
TEST(mrkp_test, mrkp_test_003_driver)
{
	struct module *pMod = THIS_MODULE;
	const struct module_memory* mod_mem_text;
	unsigned long ro_addr;
	mrkp_test_msg(" [SCN003]: attack driver (no toleration)\n");
	if (!pMod)
		return 0;

	mod_mem_text = &pMod->mem[MOD_TEXT];
	ro_addr = (unsigned long)mod_mem_text,
	mrkp_test_msg(" [SCN003]: driver module rodata address: %lx\n", ro_addr);

	/* Start attack */
	try_to_attack_ko(m, ro_addr);

	/* Dump result */
	seq_puts(m, "KE! should not come here\n");
	return 0;
}


#define TOLERATION_CNT	(3)

static uint64_t p_stext = 0;
static uint64_t p_etext = 0;
static uint64_t p__init_begin = 0;
static uint64_t p_jel_start = 0;
static uint64_t p_jel_end = 0;

/******************************************************
 * Test kernel_pages(text)_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_004_kernel_text)
{
	unsigned long addr_start;
	static int cnt = 1;

	mrkp_test_msg("[SCN007]: attack kernel code (... %d)\n", cnt);
	mrkp_get_krn_region_info(&p_stext, &p_etext, KERN_REGION_TEXT);
	addr_start = (unsigned long)p_stext;

	if (!addr_start)
		return 0;

	mrkp_test_msg("[SCN007]: kernel code address: %lx\n", addr_start);

	/* Start attack */
	try_to_attack(m, addr_start, false);

	/* Dump result */
	if (cnt <= TOLERATION_CNT)
		seq_printf(m, "(%d) tolerated\n", cnt++);
	else
		seq_puts(m, "KE! should not come here\n");

	return 0;
}

/******************************************************
 * Test kernel_rodata_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_005_kernel_rodata)
{
	unsigned long addr_start;
	static int cnt = 1;

	mrkp_test_msg("[SCN006]: attack kernel RO data (... %d)\n", cnt);
	mrkp_get_krn_region_info(&p_etext, &p__init_begin, KERN_REGION_RODATA);
	addr_start = (unsigned long)p_etext;

	if (!addr_start)
		return 0;

	mrkp_test_msg("[SCN006]: kernel rodata address: %lx\n", addr_start);

	/* Start attack */
	try_to_attack(m, addr_start, false);

	/* Dump result */
	if (cnt <= TOLERATION_CNT)
		seq_printf(m, "(%d) tolerated\n", cnt++);
	else
		seq_puts(m, "KE! should not come here\n");

	return 0;
}

/******************************************************
 * Test kernel_jump lable table_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_006_kernel_jel)
{
	unsigned long addr_start;
	static int cnt = 1;

	mrkp_test_msg("[SCN007]: attack kernel jump label table (... %d)\n", cnt);
	mrkp_get_krn_region_info(&p_jel_start, &p_jel_end, KERN_REGION_JEL);
	addr_start = (unsigned long)p_jel_start;

	if (!addr_start)
		return 0;

	mrkp_test_msg("[SCN007]: kernel jel address: %lx\n", addr_start);

	/* Start attack */
	try_to_attack(m, addr_start, false);

	/* Dump result */
	if (cnt <= TOLERATION_CNT)
		seq_printf(m, "(%d) tolerated\n", cnt++);
	else
		seq_puts(m, "KE! should not come here\n");

	return 0;
}

/******************************************************
 * Test registered CMA protection for JEL lookup table
 ******************************************************/
TEST(mrkp_test, mrkp_test_007_jel_lookup_table)
{
	struct device_node *mrkp_memory_node = 0;
	struct resource mrkp_memory_resource = { 0 };
	static int cnt = 1;

	mrkp_memory_node = of_find_node_by_name(NULL, "kern_prot_region");
	if (!mrkp_memory_node) {
		pr_err("%s fail: of_find_node_by_name\n", __func__);
		return 0;
	}
	if (of_address_to_resource(mrkp_memory_node, 0, &mrkp_memory_resource)) {
		pr_err("%s fail: of_address_to_resource\n", __func__);
		return 0;
	}

	mrkp_test_msg("[SCN007]: attack mrkp CMA address: %llx\n",
				(uint64_t)(mrkp_memory_resource.start));

	/* Start attack */
	try_to_attack(m, mrkp_memory_resource.start, true);

	/* Dump result */
	if (cnt <= TOLERATION_CNT)
		seq_printf(m, "(%d) tolerated\n", cnt++);
	else
		seq_puts(m, "KE! should not come here\n");

	return 0;
}

/******************************************************
 * Test selinux_avc_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_008_avc)
{
	//TODO: Add sepolicy test
	return 0;
}

/******************************************************
 * Test task_credential_protection
 ******************************************************/
TEST(mrkp_test, mrkp_test_009_creds)
{
	//TODO: Add cred structure test
	return 0;
}

DEFINE_TEST(mrkp_test_001_ko)
DEFINE_TEST(mrkp_test_002_mrkp)
DEFINE_TEST(mrkp_test_003_driver)
DEFINE_TEST(mrkp_test_004_kernel_text)
DEFINE_TEST(mrkp_test_005_kernel_rodata)
DEFINE_TEST(mrkp_test_006_kernel_jel)
DEFINE_TEST(mrkp_test_007_jel_lookup_table)
DEFINE_TEST(mrkp_test_008_avc)
DEFINE_TEST(mrkp_test_009_creds)

static struct dentry *dir;
static void add_tests(void)
{
	dir = debugfs_create_dir("mrkp_test", NULL);
	if (dir == NULL)
		return;

	ADD_TEST(dir, mrkp_test_001_ko);
	ADD_TEST(dir, mrkp_test_002_mrkp);
	ADD_TEST(dir, mrkp_test_003_driver);
	ADD_TEST(dir, mrkp_test_004_kernel_text);
	ADD_TEST(dir, mrkp_test_005_kernel_rodata);
	ADD_TEST(dir, mrkp_test_006_kernel_jel);
	ADD_TEST(dir, mrkp_test_007_jel_lookup_table);
	ADD_TEST(dir, mrkp_test_008_avc);
	ADD_TEST(dir, mrkp_test_009_creds);
}

static int __init mrkp_test_init(void)
{
	add_tests();
	pr_info("mrkp_test module loaded\n");
	return 0;
}

static void __exit mrkp_test_exit(void)
{
	pr_info("mrkp_test module unloaded\n");
}

module_init(mrkp_test_init);
module_exit(mrkp_test_exit);
MODULE_LICENSE("GPL");
