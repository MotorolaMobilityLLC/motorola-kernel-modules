/*
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2012 Motorola Mobility. All rights reserved.
 * Copyright (C) 2018 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/of_address.h>
#include <linux/nvmem-consumer.h>
#include <linux/panic_notifier.h>
#include <linux/sched.h>      /* current */
#include <linux/cred.h>       /* current_cred() */


static char *sys_restart_mode = "NULL";
module_param(sys_restart_mode, charp, 0644);

#define RESET_EXTRA_SW_BOOT_REASON     BIT(7)
#define RESET_EXTRA_PANIC_REASON       BIT(3)
#define RESET_EXTRA_REBOOT_BL_REASON   BIT(2)
#define RESET_EXTRA_SW_REBOOT_REASON   BIT(0)

struct moto_reboot_reason {
	struct device *dev;
	int reboot_notify_status;
	struct notifier_block reboot_nb;
	struct notifier_block restart_nb;
	struct notifier_block panic_nb;
	struct nvmem_cell *nvmem_oem_cell;
};

struct moto_poweroff_reason {
	const char *cmd;
	unsigned char pon_reason;
};

static struct moto_poweroff_reason extra_reasons[] = {
	{ "bootloader",		RESET_EXTRA_REBOOT_BL_REASON},
	{}
};

static char *reboot_white_list[] = { "init", "ufs_ffu_device", NULL};

static int reboot_moto_call;

static int moto_reboot_reason_panic(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct moto_reboot_reason *reboot = container_of(this,
		struct moto_reboot_reason, panic_nb);
	unsigned char val = RESET_EXTRA_PANIC_REASON;

	nvmem_cell_write(reboot->nvmem_oem_cell, &val,
			sizeof(val));
	pr_err("%s: save panic flag\n", __func__);

	return NOTIFY_OK;
}

void moto_reboot_call_notifier(char *call_name)
{
	reboot_moto_call += 1;
	pr_warn("%s: %s reboot_moto_call [%d]", __func__, call_name, reboot_moto_call);
}
EXPORT_SYMBOL(moto_reboot_call_notifier);

static void check_reboot(void)
{
	int trigger_bug = 1;
	int i = 0;

	pr_warn("%s:(%s pid=%d uid=%d) reboot_moto_call [%d]",
			__func__, current->comm, current->pid,
			current_cred()->uid.val, reboot_moto_call);

	pr_warn("sys_restart_mode: %s", sys_restart_mode);

	if(reboot_moto_call) {
		trigger_bug = 0;
	} else {
		while (reboot_white_list[i] != NULL) {
			if(!strncmp(current->comm, reboot_white_list[i], strlen(reboot_white_list[i]))) {
				trigger_bug = 0;
				break;
			}
			i++;
		}
	}

	if(trigger_bug) {
#ifdef CONFIG_MOTO_UNKNOWN_REBOOT_DEBUG
		pr_err("kernel config unknown reboot debug, trigger crash");
		BUG();
#endif
		if (!strcmp(sys_restart_mode, "panic")) {
			pr_err("cmdline config unknown reboot debug, trigger crash");
			BUG();
		}
		pr_err("unknown reboot, but no trigger crash");
	}

	return;
}

static int moto_reboot_reason_reboot(struct notifier_block *this,
				     unsigned long event, void *ptr)
{
	char *cmd = ptr;
	struct moto_reboot_reason *reboot = container_of(this,
		struct moto_reboot_reason, reboot_nb);
	struct moto_poweroff_reason *reason;
	unsigned char val = RESET_EXTRA_SW_REBOOT_REASON;

	nvmem_cell_write(reboot->nvmem_oem_cell, &val,
			sizeof(val));

	reboot->reboot_notify_status = 1;

	check_reboot();

	if (!cmd)
		return NOTIFY_OK;

	for (reason = extra_reasons; reason->cmd; reason++) {
		if (!strcmp(cmd, reason->cmd)) {
			nvmem_cell_write(reboot->nvmem_oem_cell,
					 &reason->pon_reason,
					 sizeof(reason->pon_reason));
	        pr_warn("%s: record sw reboot flag during reboot\n", __func__);
			break;
		}
	}

	return NOTIFY_OK;
}

static int moto_reboot_reason_restart(struct notifier_block *this,
				     unsigned long event, void *ptr)
{
	struct moto_reboot_reason *reboot = container_of(this,
		struct moto_reboot_reason, restart_nb);
	unsigned char val = RESET_EXTRA_SW_REBOOT_REASON;

	if (reboot->reboot_notify_status)
		return NOTIFY_OK;

	nvmem_cell_write(reboot->nvmem_oem_cell, &val,
			sizeof(val));

	pr_warn("%s: record sw reboot flag during restart\n", __func__);

	check_reboot();

	return NOTIFY_OK;
}

static int moto_reboot_reason_probe(struct platform_device *pdev)
{
	struct moto_reboot_reason *reboot;
	unsigned char val = RESET_EXTRA_SW_BOOT_REASON;
	int ret;

	reboot = devm_kzalloc(&pdev->dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->dev = &pdev->dev;

	reboot->nvmem_oem_cell = nvmem_cell_get(reboot->dev, "extra_restart_reason");

	if (IS_ERR(reboot->nvmem_oem_cell))
		return PTR_ERR(reboot->nvmem_oem_cell);

	reboot->panic_nb.notifier_call = moto_reboot_reason_panic;
	reboot->panic_nb.priority = INT_MAX;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &reboot->panic_nb);

	reboot->reboot_notify_status = 0;
	reboot_moto_call = 0;

	reboot->reboot_nb.notifier_call = moto_reboot_reason_reboot;
	reboot->reboot_nb.priority = 255;
	register_reboot_notifier(&reboot->reboot_nb);

	reboot->restart_nb.notifier_call = moto_reboot_reason_restart;
	reboot->restart_nb.priority = 255;
	register_restart_handler(&reboot->restart_nb);

	platform_set_drvdata(pdev, reboot);

	ret = nvmem_cell_write(reboot->nvmem_oem_cell, &val, sizeof(val));
	pr_err("update sw boot flag, ret = %d\n", ret);

	return 0;
}

static void moto_reboot_reason_remove(struct platform_device *pdev)
{
	struct moto_reboot_reason *reboot = platform_get_drvdata(pdev);

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &reboot->panic_nb);
	unregister_reboot_notifier(&reboot->reboot_nb);
	unregister_restart_handler(&reboot->restart_nb);

	return;
}

static const struct of_device_id of_moto_reboot_reason_match[] = {
	{ .compatible = "moto,reboot-reason", },
	{},
};
MODULE_DEVICE_TABLE(of, of_moto_reboot_reason_match);

static struct platform_driver moto_reboot_reason_driver = {
	.probe = moto_reboot_reason_probe,
	.remove = moto_reboot_reason_remove,
	.driver = {
		.name = "moto-reboot-reason",
		.of_match_table = of_match_ptr(of_moto_reboot_reason_match),
	},
};

module_platform_driver(moto_reboot_reason_driver);

MODULE_DESCRIPTION("moto Reboot Reason Driver");
MODULE_LICENSE("GPL v2");
