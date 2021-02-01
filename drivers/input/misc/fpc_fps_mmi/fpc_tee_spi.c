/*
 * FPC Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks.
 * *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 *
 * Copyright (c) 2018 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

//#define  AP_CONTROL_CLK       1
//#define  USE_PLATFORM_BUS     1
#define  USE_SPI_BUS 1

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define FPC_RESET_LOW_US 5000
#define FPC_RESET_HIGH1_US 100
#define FPC_RESET_HIGH2_US 5000
#define FPC_TTW_HOLD_TIME 1000

static const char * const pctl_names[] = {
	"fingerprint_reset_low",
	"fingerprint_reset_high",
};

struct fpc_data {
	struct device *dev;
#if defined(USE_SPI_BUS)
	struct spi_device *spidev;
#elif defined(USE_PLATFORM_BUS)
	struct platform_device *spidev;
#endif
#ifdef AP_CONTROL_CLK
	struct clk *parent-clk;
	struct clk *sel-clk;
	struct clk *spi-clk;
#endif

	struct pinctrl *pinctrl_fpc;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	int irq_gpio;
	int irq_num;
	int rst_gpio;
	int vdd_gpio;
	bool wakeup_enabled;
	bool request_irq;
	bool init;
	struct wakeup_source ttw_wl;
	bool clocks_enabled;
};

static DEFINE_MUTEX(spidev_set_gpio_mutex);

extern void mt_spi_disable_master_clk(struct spi_device *spidev);
extern void mt_spi_enable_master_clk(struct spi_device *spidev);

static irqreturn_t fpc_irq_handler(int irq, void *handle);

#ifdef AP_CONTROL_CLK
static long ap_spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
				|| ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void ap_spi_clock_set(struct fpc_data *fpc, int speed)
{
	long rate ;
	int rc;

	rate = ap_spi_clk_max_rate(fpc->core_clk, speed);
	pr_info("%s: requested clock frequency:%d sys matched clock is %d",
				__func__, speed,rate);
	if (rate < 0) {
			pr_info("%s: no match found for requested clock frequency:%d",
				__func__, speed);
		//return;
		rate = speed ;
	}
	rc = clk_set_rate(fpc->core_clk, rate);
	rc = clk_set_rate(fpc->iface_clk, rate);

	if (rc < 0){
		pr_info("SPI clk set failed\n");
	} else {
		fpc->clk_speed = speed;
	}
}

static int fpc_spi_clk_init(struct fpc_data *fpc)
{
	pr_debug("%s: enter\n", __func__);

	fpc->clk_enabled = 0;
	fpc->core_clk = clk_get(&data->spi->dev, "spi1");
	if (IS_ERR_OR_NULL(data->core_clk)) {
		pr_err("%s: fail to get core_clk\n", __func__);
		return -EPERM;
	}
	fpc->iface_clk = clk_get(&fpc->spi->dev, "gate_spi1_clk");
	if (IS_ERR_OR_NULL(fpc->iface_clk)) {
		pr_err("%s: fail to get iface_clk\n", __func__);
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -ENOENT;
	}
	return 0;
}

static int fpc_spi_clk_enable(struct fpc_data *fpc)
{
	int err;

	pr_debug("%s: enter\n", __func__);

	if (fpc->clocks_enabled)
		return 0;

	err = clk_prepare_enable(fpc->core_clk);
	if (err) {
		pr_err("%s: fail to enable core_clk\n", __func__);
		return -EPERM;
	}

	err = clk_prepare_enable(fpc->iface_clk);
	if (err) {
		pr_err("%s: fail to enable iface_clk\n", __func__);
		clk_disable_unprepare(fpc->core_clk);
		return -ENOENT;
	}
	pr_info("%s spi_clock enable done",__func__);
	fpc->clocks_enabled = true;

	return 0;
}

static int fpc_spi_clk_disable(struct fpc_data *fpc)
{
	pr_debug("%s: enter\n", __func__);

	if (!fpc->clocks_enabled)
		return 0;

	clk_disable_unprepare(fpc->core_clk);
	clk_disable_unprepare(fpc->iface_clk);
	fpc->clocks_enabled = false;
	pr_info("%s spi_clock disable done",__func__);
	return 0;
}

static int fpc_spi_clk_uninit(struct fpc_data *fpc)
{
	pr_debug("%s: enter\n", __func__);

	if (fpc->clk_enabled)
		fpc_spi_clk_disable(fpc);

	if (!IS_ERR_OR_NULL(fpc->core_clk)) {
		clk_put(fpc->core_clk);
		fpc->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(fpc->iface_clk)) {
		clk_put(fpc->iface_clk);
		fpc->iface_clk = NULL;
	}

	return 0;
}
#endif /* AP_CONTROL_CLK */


static int select_pin_ctl(struct fpc_data *fpc, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc->dev;

	for (i = 0; i < ARRAY_SIZE(fpc->pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			mutex_lock(&spidev_set_gpio_mutex);
			rc = pinctrl_select_state(fpc->pinctrl_fpc, fpc->pinctrl_state[i]);
			mutex_unlock(&spidev_set_gpio_mutex);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static int set_clks(struct fpc_data *fpc, bool enable)
{
	int rc = 0;
	if (enable) {
#ifdef AP_CONTROL_CLK
		fpc_spi_clk_enable(gf_dev);
		pr_info("enable ap clock\n");
#else
		mt_spi_enable_master_clk(fpc->spidev);
#endif
		fpc->clocks_enabled = true;
		rc = 1;
	} else {
#ifdef AP_CONTROL_CLK
		fpc_spi_clk_disable(gf_dev);
		pr_info("disable ap clock\n");
#else
		mt_spi_disable_master_clk(fpc->spidev);
#endif
		rc = 0;
	}

	return rc;
}

static int hw_reset(struct  fpc_data *fpc)
{
	int irq_gpio;
	struct device *dev = fpc->dev;

	select_pin_ctl(fpc, "fingerprint_reset_high");
	usleep_range(FPC_RESET_HIGH1_US, FPC_RESET_HIGH1_US + 100);

	select_pin_ctl(fpc, "fingerprint_reset_low");
	usleep_range(FPC_RESET_LOW_US, FPC_RESET_LOW_US + 100);

	select_pin_ctl(fpc, "fingerprint_reset_high");
	usleep_range(FPC_RESET_HIGH2_US, FPC_RESET_HIGH2_US + 100);

	irq_gpio = gpio_get_value(fpc->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);

	dev_info(dev, "Using GPIO#%d as IRQ.\n", fpc->irq_gpio);
	dev_info(dev, "Using GPIO#%d as RST.\n", fpc->rst_gpio);

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc_data *fpc = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		rc = hw_reset(fpc);
		return rc ? rc : count;
	} else {
		return -EINVAL;
	}
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);
	ssize_t ret = count;

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc->wakeup_enabled = false;
		smp_wmb();
	} else {
		ret = -EINVAL;
	}
	return ret;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

static ssize_t active_get(struct device *device,
			struct device_attribute *attribute,
			char *buffer)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	int result = 0;

	if (fpc->init) {
		result = 1;
	}

	return scnprintf(buffer, PAGE_SIZE, "%i\n", result);
}
static DEVICE_ATTR(active, S_IRUSR, active_get, NULL);

static int fpc_dts_request(struct fpc_data *fpc)
{
	struct spi_device *spidev = fpc->spidev;
	int rc = 0;
	size_t i;

	if (NULL == fpc->pinctrl_fpc) {
		fpc->pinctrl_fpc = devm_pinctrl_get(&spidev->dev);

		if (IS_ERR(fpc->pinctrl_fpc)) {
			rc = PTR_ERR(fpc->pinctrl_fpc);
			dev_err(fpc->dev, "Cannot find pinctrl_fpc rc = %d.\n", rc);
			goto exit;
		}

		for (i = 0; i < ARRAY_SIZE(fpc->pinctrl_state); i++) {
			const char *n = pctl_names[i];
			struct pinctrl_state *state = pinctrl_lookup_state(fpc->pinctrl_fpc, n);
			if (IS_ERR(state)) {
				dev_err(fpc->dev, "cannot find '%s'\n", n);
				rc = -EINVAL;
				goto exit;
			}
			dev_info(fpc->dev, "found pin control %s\n", n);
			fpc->pinctrl_state[i] = state;
		}
	}

exit:
	return rc;
}

static int fpc_dts_release(struct fpc_data *fpc)
{
	if (NULL != fpc->pinctrl_fpc) {
		devm_pinctrl_put(fpc->pinctrl_fpc);
		fpc->pinctrl_fpc = NULL;
	}

	return 0;
}

static int fpc_hw_res_request(struct fpc_data *fpc)
{
	struct spi_device *spidev = fpc->spidev;
	struct device *dev = &spidev->dev;
	int irqf = 0;
	int irq_num = 0;
	int rc = 0;

	if (!fpc->vdd_gpio) {
		fpc->vdd_gpio = of_get_named_gpio(dev->of_node, "fpc,vdd", 0);
		dev_info(dev, "Using GPIO#%d as  vdd. enable\n", fpc->vdd_gpio);
		if (!gpio_is_valid(fpc->vdd_gpio)){
			dev_err(dev, "invalid vdd gpio!");
			fpc->vdd_gpio = 0;
			//return -EINVAL;
		} else {
			rc = devm_gpio_request(dev, fpc->vdd_gpio, "fpc_vdd");
			if (rc) {
				pr_err("failed to request vdd gpio, rc = %d\n", rc);
				goto err_vdd;
			}
			gpio_direction_output(fpc->vdd_gpio , 1);
			gpio_set_value(fpc->vdd_gpio, 1);
			dev_info(dev, "vdd_gpio %d set to high.\n", fpc->vdd_gpio);
		}
	}

	if (!fpc->request_irq) {
		fpc->irq_gpio = of_get_named_gpio(dev->of_node, "fpc,irq", 0);
		dev_info(dev, "Using GPIO#%d as IRQ.\n", fpc->irq_gpio);
		if (!gpio_is_valid(fpc->irq_gpio)){
			dev_err(dev, "invalid irq gpio!");
			return -EINVAL;
		}

		gpio_direction_input(fpc->irq_gpio);
		irq_num = gpio_to_irq(fpc->irq_gpio);
		dev_info(dev, "requested irq gpio %d\n", irq_num);
		if (!irq_num) {
			rc = -EINVAL;
			dev_err(dev, "get irq_num error rc = %d.\n", rc);
			goto exit;
		}

		fpc->irq_num = irq_num;

		fpc->wakeup_enabled = false;

		irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
		if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
			irqf |= IRQF_NO_SUSPEND;
			device_init_wakeup(dev, 1);
		}

		rc = devm_request_threaded_irq(dev, irq_num,
			NULL, fpc_irq_handler, irqf,
			dev_name(dev), fpc);
		if (rc) {
			dev_err(dev, "could not request irq %d\n", irq_num);
			goto exit;
		}
		dev_info(dev, "requested thread irq %d\n", irq_num);

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(irq_num);

		fpc->request_irq = true;
	}
err_vdd:
	if (fpc->vdd_gpio != 0) {
		devm_gpio_free(dev,fpc->vdd_gpio);
		fpc->vdd_gpio = 0;
		pr_info("remove vdd_gpio success\n");
	}
exit:
	return rc;
}

static int fpc_hw_res_release(struct fpc_data *fpc)
{
	struct spi_device *spidev = fpc->spidev;
	struct device *dev = &spidev->dev;
	if (fpc->request_irq) {
		//disable_irq_wake(fpc->irq_num);
		//disable_irq(fpc->irq_num);
		devm_free_irq(fpc->dev, fpc->irq_num, fpc);
		fpc->request_irq = false;
	}
	if (fpc->vdd_gpio) {
		devm_gpio_free(dev,fpc->vdd_gpio);
		fpc->vdd_gpio = 0;
		pr_info("remove vdd_gpio success\n");
	}
	return 0;
}

static ssize_t hw_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);
	ssize_t ret = count;
#ifdef USER_SPACE_SPI_INIT
	dev_info(dev, "%s: enter\n", __func__);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		dev_info(dev, "%s: enable\n", __func__);
		fpc_dts_request(fpc);
		fpc_hw_res_request(fpc);
		hw_reset(fpc);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		dev_info(dev, "%s: disable\n", __func__);
		fpc_hw_res_release(fpc);
		fpc_dts_release(fpc);
	} else {
		dev_info(dev, "%s: unkown!\n", __func__);
		ret = -EINVAL;
	}
#endif
	return ret;
}

static DEVICE_ATTR(hw_enable, S_IWUSR, NULL, hw_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			struct device_attribute *attribute,
			char *buffer)
{
	struct fpc_data *fpc = dev_get_drvdata(device);

	int irq = gpio_get_value(fpc->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			struct device_attribute *attribute,
			const char *buffer, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(device);

	dev_dbg(fpc->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t clk_enable_set(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	return set_clks(fpc, (*buf == '1')) ? : count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

static struct attribute *fpc_attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_active.attr,
	&dev_attr_hw_enable.attr,
	NULL
};

static const struct attribute_group const fpc_attribute_group = {
	.attrs = fpc_attributes,
};

static irqreturn_t fpc_irq_handler(int irq, void *handle)
{
	struct fpc_data *fpc = handle;
	struct device *dev = fpc->dev;
	static int current_level = 0; // We assume low level from start
	current_level = !current_level;
	if (current_level) {
		dev_dbg(dev, "Reconfigure irq to trigger in low level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		dev_dbg(dev, "Reconfigure irq to trigger in high level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...)
	*/
	smp_rmb();
	if (fpc->wakeup_enabled)
		__pm_wakeup_event(&fpc->ttw_wl, FPC_TTW_HOLD_TIME);

	sysfs_notify(&fpc->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}
#define FINGERPRINT_INT_COMPATIBLE "mediatek,fingerprint-fpc"

#if defined(USE_SPI_BUS)
static int fpc_tee_probe(struct spi_device *spidev)
#elif defined(USE_PLATFORM_BUS)
static int fpc_tee_probe(struct platform_device *spidev)
#endif

{
	struct device *dev = NULL;
	struct fpc_data *fpc;
	int rc = 0;
#if defined(USE_SPI_BUS)
	dev = &spidev->dev;
	spidev->dev.of_node = of_find_compatible_node(NULL,
						NULL, FINGERPRINT_INT_COMPATIBLE);
	if (!spidev->dev.of_node) {
		dev_err(dev, "of node %s is no found\n",FINGERPRINT_INT_COMPATIBLE);
		rc = -EINVAL;
		goto exit;
	}
#elif defined(USE_PLATFORM_BUS)
	dev = &spidev->dev;
#endif

	dev_info(dev, "%s\n", __func__);
	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc) {
		dev_err(dev, "failed to allocate memory for struct fpc_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc->dev = dev;
	dev_set_drvdata(dev, fpc);
	fpc->spidev = spidev;
	fpc->spidev->irq = 0; /*SPI_MODE_0*/

#ifndef USER_SPACE_SPI_INIT
	dev_info(dev, "%s: enable\n", __func__);
	fpc_dts_request(fpc);
	fpc_hw_res_request(fpc);
#endif
	wakeup_source_init(&fpc->ttw_wl, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &fpc_attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}
#ifdef AP_CONTROL_CLK
	pr_info("Get the clk resource.\n");
	/* Enable spi clock */
	if (fpc_spi_clk_init(fpc))
		goto spi_probe_clk_init_failed;

	if (fpc_spi_clk_enable(fpc))
		goto spi_probe_clk_enable_failed;
	/* Initialize spi clock to 25M for FOD  */
	spi_clock_set(fpc, 1000000*25*4);
	dev_info(dev, "%s spi_clock enable done",__func__);
#endif
	fpc->init = true;
	hw_reset(fpc);
	dev_info(dev, "%s: ok\n", __func__);
#ifdef AP_CONTROL_CLK
spi_probe_clk_enable_failed:
	gfspi_ioctl_clk_uninit(gf_dev);
spi_probe_clk_init_failed:
#endif
exit:
	return rc;
}
#if defined(USE_SPI_BUS)
static int fpc_tee_remove(struct spi_device *spidev)
#elif defined(USE_PLATFORM_BUS)
static int fpc_tee_remove(struct platform_device *spidev)
#endif
{
	struct  fpc_data *fpc = dev_get_drvdata(&spidev->dev);
	dev_info(&spidev->dev, "%s: disable\n", __func__);
#ifndef USER_SPACE_SPI_INIT
	fpc_hw_res_release(fpc);
	fpc_dts_release(fpc);
#endif
	sysfs_remove_group(&spidev->dev.kobj, &fpc_attribute_group);
	wakeup_source_trash(&fpc->ttw_wl);
	dev_info(&spidev->dev, "%s\n", __func__);

	return 0;
}

static struct of_device_id mt6797_of_match[] = {
	{ .compatible = "fpc,fpc_spi", },
	{}
};
MODULE_DEVICE_TABLE(of, mt6797_of_match);

#if defined(USE_SPI_BUS)
static struct spi_driver fpc_tee_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver fpc_tee_driver = {
#endif

	.driver = {
		.name	= "fpc_spi",
		.bus = &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = mt6797_of_match,
	},
	.probe	= fpc_tee_probe,
	.remove	= fpc_tee_remove,
#if defined(USE_PLATFORM_BUS)
	.resume 	=	fpc_tee_resume,
	.suspend 	=	fpc_tee_suspend,
#endif
};

static int __init fpc_sensor_init(void)
{
	int status;

#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&fpc_tee_driver);
#elif defined(USE_SPI_BUS)
	status = spi_register_driver(&fpc_tee_driver);
#endif
	if (status < 0)
		printk("%s, fpc_sensor_init failed.\n", __func__);

	return status;
}
module_init(fpc_sensor_init);

static void __exit fpc_sensor_exit(void)
{
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&fpc_tee_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&fpc_tee_driver);
#endif
}
module_exit(fpc_sensor_exit);

MODULE_LICENSE("GPL");
