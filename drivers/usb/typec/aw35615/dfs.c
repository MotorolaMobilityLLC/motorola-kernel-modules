// SPDX-License-Identifier: GPL-2.0
/*********************************************************************************
 * @brief       Implements a set of DebugFS accessors
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 ********************************************************************************/

#ifdef AW_DEBUG

#include "dfs.h"

#include <linux/seq_file.h>

#include "aw35615_global.h"
#include "sysfs_header.h"

/* Type-C state log formatted output */
static int typec_log_show(struct seq_file *m, void *v)
{
	struct Port *p = (struct Port *)m->private;

	AW_U16 state;
	AW_U16 TimeStampSeconds, TimeStampMS10ths;

	if (!p)
		return -1;

	while (ReadStateLog(&p->TypeCStateLog, &state, &TimeStampMS10ths, &TimeStampSeconds))
		seq_printf(m, "[%4u.%04u] %s\n",
				TimeStampSeconds,
				TimeStampMS10ths,
				TYPEC_STATE_TBL[state]);

	return 0;
}

static int typec_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, typec_log_show, inode->i_private);
}

static const struct file_operations typec_log_fops = {
	.open     = typec_log_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release
};

/* Policy Engine state log formatted output */
static int pe_log_show(struct seq_file *m, void *v)
{
	struct Port *p = (struct Port *)m->private;

	AW_U16 state;
	AW_U16 TimeStampSeconds, TimeStampMS10ths;

	if (!p)
		return -1;

	while (ReadStateLog(&p->PDStateLog, &state, &TimeStampMS10ths, &TimeStampSeconds))
		seq_printf(m, "[%4u.%04u] %s\n",
				TimeStampSeconds,
				TimeStampMS10ths,
				PE_STATE_TBL[state]);

	return 0;
}

static int pe_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, pe_log_show, inode->i_private);
}

static const struct file_operations pe_log_fops = {
	.open     = pe_log_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release
};

/*
 * Initialize DebugFS file objects
 */
AW_S32 aw_DFS_Init(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	/* Try to create our top level dir */
	chip->debugfs_parent = debugfs_create_dir("aw35615", NULL);

	if (!chip->debugfs_parent) {
		pr_err("AWINIC  %s - Couldn't create DebugFS dir!\n", __func__);
		return -ENOMEM;
	}

	debugfs_create_file("tc_log", 0444, chip->debugfs_parent, &(chip->port), &typec_log_fops);
	debugfs_create_file("pe_log", 0444, chip->debugfs_parent, &(chip->port), &pe_log_fops);

	return 0;
}

/*
 * Cleanup/remove unneeded DebugFS file objects
 */
AW_S32 aw_DFS_Cleanup(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (chip->debugfs_parent != NULL)
		debugfs_remove_recursive(chip->debugfs_parent);

	return 0;
}

#endif /* AW_DEBUG */
