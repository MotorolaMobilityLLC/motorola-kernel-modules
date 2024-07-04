/*
 * Copyright (C) 2024 Motorola Mobility LLC
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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/version.h>
#if KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE
#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
#include <soc/qcom/minidump.h>
#endif
#endif

static int add_cpusys_to_minidump(const struct device_node *np)
{
	struct resource res;
	struct device_node *node;
#if KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE
#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
	struct md_region md_entry;
#endif
#endif
	int err = 0;

	/* Get cpusys memory rigion */
	node = of_parse_phandle(np, "cpusys-mem", 0);
	if (!node) {
		pr_err("No %s specified\n", "cpusys memory-region");
		goto err;
	}

	err = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (err) {
		pr_err("No memory address assigned to the cpusys region\n");
		goto err;
	}

	//add cpusys region to minidump
#if KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE
#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
	strscpy(md_entry.name, "CPUSYS", sizeof(md_entry.name));
	md_entry.virt_addr = (uintptr_t)phys_to_virt(res.start);
	md_entry.phys_addr = res.start;
	md_entry.size = resource_size(&res);
	if (msm_minidump_add_region(&md_entry) < 0){
		pr_err("Failed to add CPUSYS section in Minidump\n");
		err = -EINVAL;
		goto err;
	}
	pr_info("cpusys_dump initialized, addr:0x%llx size:0x%llx\n",md_entry.phys_addr,
				md_entry.size);
#endif
#endif
err:
	return err;
}

static int mmi_qcom_minidump_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error = 0;

	/* add cpusys memory rigion */
	error = add_cpusys_to_minidump(dev->of_node);
	if (error) {
		return error;
	}

	return 0;
}

static const struct of_device_id mmi_qcom_minidump_match_table[] = {
	{ .compatible = "moto,mmi_qcom_minidump" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmi_qcom_minidump_match_table);

static struct platform_driver mmi_qcom_minidump_driver = {
	.probe  = mmi_qcom_minidump_probe,
	.driver = {
		   .name = "mmi_qcom_minidump",
		   .of_match_table = mmi_qcom_minidump_match_table,
	},
};

static int __init mmi_qcom_minidump_init(void)
{
	return platform_driver_register(&mmi_qcom_minidump_driver);
}

static void __exit mmi_qcom_minidump_exit(void)
{
	platform_driver_unregister(&mmi_qcom_minidump_driver);
}

module_init(mmi_qcom_minidump_init);
module_exit(mmi_qcom_minidump_exit);

MODULE_DESCRIPTION("Motorola, Inc. mmi_qcom_minidump Driver");
MODULE_LICENSE("GPL v2");
