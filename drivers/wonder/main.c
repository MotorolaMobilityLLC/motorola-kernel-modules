/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Wonder WiFi Virtual Soft-MAC Driver
 *
 * Main entry point for the Wonder driver module. This file handles the
 * initialization and cleanup of the driver, locating the physical
 * network device and kicking off the mac80211 registration process.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "core.h"
#include "mac80211.h"
#include "wonder_log.h"
#include "include/wonder/wondertap.h"
#include "wondertap_internal.h"

/* Module parameter for setting the physical device name */
module_param(physical_name, charp, 0444);
MODULE_PARM_DESC(physical_name, "Interface name to use (e.g., wlan0, radiotap0, ...)");


#define WONDER_MAX_COMPAT_VERSIONS 4
static int wonder_ver_match_table[WONDER_VERSION_MAX][WONDER_MAX_COMPAT_VERSIONS] = {
	{ WONDER_VERSION_1_0, -1 },
	{ WONDER_VERSION_1_1, -1 },
	{ WONDER_VERSION_1_2, -1 },
	{ WONDER_VERSION_1_3, -1 },
	{ WONDER_VERSION_1_4, -1 },
	{ WONDER_VERSION_1_4_1, WONDER_VERSION_1_4, -1 },
	{ WONDER_VERSION_1_5, WONDER_VERSION_1_4, WONDER_VERSION_1_4_1, -1 },
};

static bool wonder_ver_can_support(enum wondertap_ver slave, enum wondertap_ver master)
{
	int i;

	if (master < 0 || master >= WONDER_VERSION_MAX)
		return false;
	if (slave < 0 || slave >= WONDER_VERSION_MAX)
		return false;

	for (i = 0; i < WONDER_MAX_COMPAT_VERSIONS; i++) {
		if (wonder_ver_match_table[master][i] == -1)
			break;
		if (wonder_ver_match_table[master][i] == slave) {
			return true;
		}
	}
	return false;
}

static int wonder_master_bind(struct device *dev)
{
	struct wondertap_data *wondertap = dev_get_drvdata(dev);
	struct platform_device *wlan_pdev;
	struct wondertap_priv *wlan_priv;
	int ret;

	dev_info(dev, "%s(): Binding...\n", __func__);

	ret = component_bind_all(dev, NULL);
	if (ret)
		return ret;

	wlan_pdev = of_find_device_by_node(wondertap->wlan_node);
	if (!wlan_pdev) {
		dev_err(dev, "%s(): Cannot find wlan platform device!\n", __func__);
		component_unbind_all(dev, NULL);
		return -ENODEV;
	}

	wlan_priv = platform_get_drvdata(wlan_pdev);
	if (!wlan_priv || !wlan_priv->wonder_ops)
		goto err;

	if (!wonder_ver_can_support(wlan_priv->ver, wondertap->ver)) {
		dev_err(dev, "%s(): wondertap interface version mismatch(%d,%d)!\n",
			__func__, wlan_priv->ver, wondertap->ver);
		goto err;
	}
	/* All matched, hook the ops to wondertap interface. */
	wondertap_register_ops(wlan_priv->wonder_ops);
	wondertap->wonder_ops = wlan_priv->wonder_ops;
	put_device(&wlan_pdev->dev);
	dev_info(dev, "%s(): Connected to wlan ver %d!\n", __func__, wlan_priv->ver);
	return 0;
err:
	dev_err(dev, "%s(): wlan driver data invalid!\n", __func__);
	put_device(&wlan_pdev->dev);
	component_unbind_all(dev, NULL);
	return -EINVAL;
}

static void wonder_master_unbind(struct device *dev)
{
	struct wondertap_data *wondertap = dev_get_drvdata(dev);

	dev_info(dev, "%s(): Unbinding...\n", __func__);

	wondertap_unregister_ops(wondertap->wonder_ops);
	component_unbind_all(dev, NULL);
}

static const struct component_master_ops wonder_comp_ops = {
	.bind = wonder_master_bind,
	.unbind = wonder_master_unbind,
};

static int wonder_compare_dev(struct device *dev, void *data)
{
	return (dev->of_node == data);
}

static int wonder_probe(struct platform_device *pdev)
{
	struct wonder_data *wonder;
	struct wondertap_data *wondertap;
	struct component_match *match = NULL;
	struct device_node *provider_node;
	int ret;

	dev_info(&pdev->dev, "%s(): probe\n", __func__);
	wonder = wonder_mac80211_init();
	if (!wonder)
		return -ENODEV;

	wondertap = &wonder->wondertap_data;
	provider_node = of_parse_phandle(pdev->dev.of_node, "wondertap-provider", 0);
	if (!provider_node) {
		dev_err(&pdev->dev, "%s(): Missing 'wondertap-provider' in DTS\n", __func__);
		return -ENODEV;
	}
	wondertap->wlan_node = provider_node;
	/* Assign wondertap interface version will be used in the match process. */
	wondertap->ver = WONDER_VERSION_1_5;
	platform_set_drvdata(pdev, wondertap);
	component_match_add_release(&pdev->dev, &match, NULL,
							wonder_compare_dev, provider_node);

	of_node_put(provider_node);
	ret = component_master_add_with_match(&pdev->dev, &wonder_comp_ops, match);
	if (ret) {
		dev_err(&pdev->dev, "%s(): Missing component\n", __func__);
		return -ENODEV;
	}
	return wonder_debugfs_init(wonder);
}

static void wonder_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &wonder_comp_ops);
	wonder_debugfs_exit();
	wonder_mac80211_exit();
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int wonder_remove_wrapper(struct platform_device *pdev)
{
	wonder_remove(pdev);
	return 0;
}
#define wonder_remove wonder_remove_wrapper
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0) */

static const struct of_device_id wonder_dt_ids[] = {
	{ .compatible = "google,wonder-drv-v1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wonder_dt_ids);

static struct platform_driver wonder_driver = {
	.probe = wonder_probe,
	.remove = wonder_remove,
	.driver = {
		.name = "wonder",
		.of_match_table = wonder_dt_ids,
	},
};
module_platform_driver(wonder_driver);

MODULE_DESCRIPTION("Google Wonder Virtual mac80211 Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Android WiFi Team");
