/*
 * Copyright (C) 2013-2016, Shenzhen Huiding Technology Co., Ltd.
 * All Rights Reserved.
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
//new added
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/pm_wakeup.h>
//#include <teei_fp.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/* MTK header */
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif

/* there is no this file on standardized GPIO platform */
#ifdef CONFIG_MTK_GPIO
#include "mtk_gpio.h"
#include "mach/gpio_const.h"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,10,0)
#include <mt-plat/sync_write.h>
#endif
#include <linux/of_address.h>


#include "gf_spi_tee.h"
#include  <linux/regulator/consumer.h>
#include <linux/version.h>

/**************************defination******************************/
#define GF_DEV_NAME "goodix_fp"
#define GF_DEV_MAJOR 0	/* assigned */

#define GF_CLASS_NAME "goodix_fp"
#define GF_INPUT_NAME "gf-keys"

#define GF_LINUX_VERSION "V1.01.04"

#define GF_NETLINK_ROUTE 29   /* for GF test temporary, need defined in include/uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

/*************************************************************/

/* for Upstream SPI ,just tell SPI about the clock */
#ifdef CONFIG_SPI_MT65XX
u32 gf_spi_speed = 1*1000000;
#endif

/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
static struct wakeup_source *fp_wakeup_source;
#else
static struct wakeup_source fp_wakeup_source;
#endif
static unsigned int bufsiz = (25 * 1024);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "maximum data bytes for SPI message");

#ifdef CONFIG_OF
static const struct of_device_id gf_of_match[] = {
	{ .compatible = "goodix,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, gf_of_match);
#endif

int gf_spi_read_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf);

struct regulator *buck;
/* for netlink use */
static int pid = 0;

static u8 g_vendor_id = 0;

static ssize_t active_get(struct device *device,
			struct device_attribute *attribute,
			char *buffer)
{
	struct gf_device *gf_dev =  dev_get_drvdata(device);
	int result = 0;

	if (NULL != gf_dev && gf_dev->init) {
		result = 1;
	}

	return scnprintf(buffer, PAGE_SIZE, "%i\n", result);
}
static DEVICE_ATTR(active, S_IRUSR, active_get, NULL);

static struct attribute *goodix_attributes[] = {
	&dev_attr_active.attr,
	NULL
};

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
static const struct attribute_group goodix_attribute_group = {
#else
static const struct attribute_group const goodix_attribute_group = {
#endif
	.attrs = goodix_attributes,
};

#ifndef CONFIG_SPI_MT65XX
const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 50, /* 1MHz */
	.low_time = 50,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,
	/* .com_mod = DMA_TRANSFER, */

	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

/* -------------------------------------------------------------------- */
/* timer function								*/
/* -------------------------------------------------------------------- */
/*
#define TIME_START	   0
#define TIME_STOP	   1

static long int prev_time, cur_time;

long int kernel_time(unsigned int step)
{
	cur_time = ktime_to_us(ktime_get());
	if (step == TIME_START) {
		prev_time = cur_time;
		return 0;
	} else if (step == TIME_STOP) {
		gf_debug(DEBUG_LOG, "%s, use: %ld us\n", __func__, (cur_time - prev_time));
		return cur_time - prev_time;
	}
	prev_time = cur_time;
	return -1;
}
*/

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								  */
/* -------------------------------------------------------------------- */
static int gf_get_gpio_dts_info(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	int ret = 0;
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;

	// it means we have got the pin controllers.
	if (NULL != gf_dev->pinctrl_gpios) {
		return 0;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-goodix");
	if (node) {
		pdev = of_find_device_by_node(node);
		if (pdev) {
			gf_dev->pinctrl_gpios = devm_pinctrl_get(&pdev->dev);
			if (IS_ERR(gf_dev->pinctrl_gpios)) {
				ret = PTR_ERR(gf_dev->pinctrl_gpios);
				gf_dev->pinctrl_gpios = NULL;
				gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl\n", __func__);
				return ret;
			}
		} else {
			gf_debug(ERR_LOG, "%s platform device is null\n", __func__);
			return -EINVAL;
		}
	} else {
		gf_debug(ERR_LOG, "%s device node is null\n", __func__);
		return -EINVAL;
	}

	if(of_property_read_bool(node,"rgltr-ctrl-support")) {
		gf_dev->rgltr_ctrl_support = 1;
	} else {
		gf_dev->rgltr_ctrl_support = 0;
		pr_err("goodix: No regulator control parameter defined\n");
	}
	if (gf_dev->rgltr_ctrl_support) {
		gf_dev->pwr_supply = regulator_get(&pdev->dev, "fp,vdd");
		if (IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			gf_dev->pwr_supply = NULL;
			gf_dev->rgltr_ctrl_support = 0;
			pr_warn("goodix Unable to get fp,vdd");
		} else {
			ret = of_property_read_u32_array(node, "fp,voltage-range", gf_dev->pwr_voltage_range, 2);
			if (ret) {
				gf_dev->pwr_voltage_range[0] = -1;
				gf_dev->pwr_voltage_range[1] = -1;
			}
			if (regulator_count_voltages(gf_dev->pwr_supply) > 0) {
				if((gf_dev->pwr_voltage_range[0] >0) && (gf_dev->pwr_voltage_range[1] > 0))
					ret = regulator_set_voltage(gf_dev->pwr_supply,
						gf_dev->pwr_voltage_range[0], gf_dev->pwr_voltage_range[1]);
				if (ret) {
					pr_warn("goodix: %s : set vdd regulator voltage failed %d \n", __func__, ret);
				}
			}
		}
	}

	gf_dev->pwr_gpio = of_get_named_gpio(node, "fp-gpio-ven", 0);
	if (gf_dev->pwr_gpio < 0) {
		pr_warn("goodix: failed to get pwr gpio!\n");
	} else {
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			ret = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
			if (ret) {
				pr_err("goodix: failed to request pwr gpio, rc = %d\n", ret);
				goto err_pwr;
			}
			gpio_direction_output(gf_dev->pwr_gpio, 1);
		}
	}

	gf_dev->pins_reset_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_reset_high");
	if (IS_ERR(gf_dev->pins_reset_high)) {
		ret = PTR_ERR(gf_dev->pins_reset_high);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_high\n", __func__);
		return ret;
	}
	gf_dev->pins_reset_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_reset_low");
	if (IS_ERR(gf_dev->pins_reset_low)) {
		ret = PTR_ERR(gf_dev->pins_reset_low);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_low\n", __func__);
		return ret;
	}

	gf_debug(DEBUG_LOG, "%s success\n", __func__);
	return 0;
err_pwr:
	if (!IS_ERR_OR_NULL(gf_dev->pwr_supply))
	{
		pr_info(" %s goodix:  devm_regulator_put \n", __func__);
		regulator_put(gf_dev->pwr_supply);
		gf_dev->pwr_supply= NULL;
	}
#endif
	return ret;
}

static int gf_hw_power_enable(struct gf_device *gf_dev, u8 onoff)
{
	/* TODO: LDO configure */
	static int enable = 1;
	int rc = 0;

	if (onoff && enable) {
		gf_debug(INFO_LOG, "%s, enable\n", __func__);
		if(gf_dev->rgltr_ctrl_support && !IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			rc = regulator_enable(gf_dev->pwr_supply);
			pr_warn("goodix:  %s : enable  pwr_supply return %d \n", __func__, rc);
		}
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			gpio_direction_output(gf_dev->pwr_gpio, 1);
			pr_warn(" goodix: %s : set  pwr_gpio:%d  1\n", __func__, gf_dev->pwr_gpio);
		}
//		hwPowerOn(MT6331_POWER_LDO_VIBR, VOL_2800, "fingerprint");
		enable = 0;
		#ifdef CONFIG_OF
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
		if (0 != rc) {
			gf_debug(ERR_LOG, "%s, pinctrl_select_state failed:pins_reset_low.\n", __func__);
		}
		mdelay(15);

		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
		if (0 != rc) {
			gf_debug(ERR_LOG, "%s, pinctrl_select_state failed:pins_reset_high.\n", __func__);
		}
		#endif
	}
	else if (!onoff && !enable) {
		if (gf_dev->rgltr_ctrl_support  && !IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			rc = regulator_disable(gf_dev->pwr_supply);
			pr_warn(" goodix: %s : disable  pwr_supply return %d \n", __func__, rc);
		}
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			gpio_direction_output(gf_dev->pwr_gpio, 0);
			pr_warn(" goodix: %s : set  pwr_gpio:%d 0 \n", __func__,gf_dev->pwr_gpio);
		}
//		hwPowerDown(MT6331_POWER_LDO_VIBR, "fingerprint");
		enable = 1;
	}

	return rc;
}

static void gf_spi_clk_enable(struct gf_device *gf_dev, u8 bonoff)
{
	static int count = 0;
	pr_err("%s line:%d try to control spi clk\n", __func__,__LINE__);
#ifdef CONFIG_MTK_CLKMGR
        if (bonoff && (count == 0)) {
                gf_debug(DEBUG_LOG, "%s, start to enable spi clk && count = %d.\n", __func__, count);
                enable_clock(MT_CG_PERI_SPI0, "spi");
                count = 1;
        } else if ((count > 0) && (bonoff == 0)) {
                gf_debug(DEBUG_LOG, "%s, start to disable spi clk&& count = %d.\n", __func__, count);
                disable_clock(MT_CG_PERI_SPI0, "spi");
                count = 0;
    }
#else


	if (bonoff && (count == 0)) {
		pr_err("%s line:%d enable spi clk\n", __func__,__LINE__);
		mt_spi_enable_master_clk(gf_dev->spi);
		count = 1;
	} else if ((count > 0) && (bonoff == 0)) {
		pr_err("%s line:%d disable spi clk\n", __func__,__LINE__);
		mt_spi_disable_master_clk(gf_dev->spi);
		count = 0;
	}
#endif
}

static void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	struct device_node *node;
	int gpio;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-goodix");

	gpio = of_get_named_gpio(node, "int-gpios", 0);

	if (node) {
		//gpio_direction_input(gpio);

		//gf_dev->irq = irq_of_parse_and_map(node, 0);
		gf_dev->irq = gpio_to_irq(gpio);
		gf_debug(INFO_LOG, "requested gpio=%d irq=%d\n", gpio, gf_dev->irq);
#ifndef CONFIG_MTK_EIC
		irq_set_irq_wake(gf_dev->irq, 1);
#else
		enable_irq_wake(gf_dev->irq);
#endif
	}
#endif
}

/* delay ms after reset */
static void gf_hw_reset(struct gf_device *gf_dev, u8 delay)
{
#ifdef CONFIG_OF
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
	mdelay(15);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif

	if (delay) {
		/* delay is configurable */
		mdelay(delay);
	}
}

static void gf_enable_irq(struct gf_device *gf_dev)
{
	if (1 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_count = 1;
		gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
	}
}

static void gf_disable_irq(struct gf_device *gf_dev)
{
	if (0 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
	} else {
		disable_irq(gf_dev->irq);
		gf_dev->irq_count = 0;
		gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
	}
}


/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
static void gf_netlink_send(struct gf_device *gf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;

	gf_debug(INFO_LOG, "[%s] : enter, send command %d\n", __func__, command);
	if (NULL == gf_dev->nl_sk) {
		gf_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
		return;
	}

	if (0 == pid) {
		gf_debug(ERR_LOG, "[%s] : invalid native process pid\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		gf_debug(ERR_LOG, "[%s] : nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(gf_dev->nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret == 0) {
		gf_debug(ERR_LOG, "[%s] : send failed\n", __func__);
		return;
	}

	gf_debug(INFO_LOG, "[%s] : send done, data length is %d\n", __func__, ret);
}

static void gf_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];

	gf_debug(INFO_LOG, "[%s] : enter \n", __func__);

	skb = skb_get(__skb);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;
		gf_debug(INFO_LOG, "[%s] : pid: %d, msg: %s\n", __func__, pid, str);

	} else {
		gf_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
	}

	kfree_skb(skb);
}

static int gf_netlink_init(struct gf_device *gf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gf_netlink_recv;

	gf_dev->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
		return -1;
	}

	gf_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
	return 0;
}

static int gf_netlink_destroy(struct gf_device *gf_dev)
{
	if (gf_dev->nl_sk != NULL) {
		netlink_kernel_release(gf_dev->nl_sk);
		gf_dev->nl_sk = NULL;
		return 0;
	}

	gf_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
	return -1;
}

/* -------------------------------------------------------------------- */
/* early suspend callback and suspend/resume functions          */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
}

static void gf_late_resume(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
}
#else

static int gf_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct gf_device *gf_dev = NULL;
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;
	FUNC_ENTRY();

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
		return 0;

	gf_dev = container_of(self, struct gf_device, notifier);
	blank = *(int *)evdata->data;

	gf_debug(INFO_LOG, "[%s] : enter, blank=0x%x\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		gf_debug(INFO_LOG, "[%s] : lcd on notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
		break;

	case FB_BLANK_POWERDOWN:
		gf_debug(INFO_LOG, "[%s] : lcd off notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
		break;

	default:
		gf_debug(INFO_LOG, "[%s] : other notifier, ignore\n", __func__);
		break;
	}
	FUNC_EXIT();
	return retval;
}

#endif //CONFIG_HAS_EARLYSUSPEND

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = (struct gf_device *)handle;
	FUNC_ENTRY();

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
	__pm_wakeup_event(fp_wakeup_source, 500);
#else
	__pm_wakeup_event(&fp_wakeup_source, 500);
#endif

	gf_netlink_send(gf_dev, GF_NETLINK_IRQ);
	gf_dev->sig_count++;

	FUNC_EXIT();
	return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_device *gf_dev = NULL;
	struct gf_key gf_key;
	gf_nav_event_t nav_event = GF_NAV_NONE;
	uint32_t nav_input = 0;
	uint32_t key_input = 0;
	int retval = 0;
	u8  buf    = 0;
	u8 netlink_route = GF_NETLINK_ROUTE;
	struct gf_ioc_chip_info info;

	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -EINVAL;

	/* Check access direction once here; don't repeat below.
	* IOC_DIR is from the user perspective, while access_ok is
	* from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
#if LINUX_VERSION_CODE > KERNEL_VERSION(5,10,0)
		retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
#else
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
#endif

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
#if LINUX_VERSION_CODE > KERNEL_VERSION(5,10,0)
		retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
#else
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
#endif

	if (retval)
		return -EINVAL;

	gf_dev = (struct gf_device *)filp->private_data;
	if (!gf_dev) {
		gf_debug(ERR_LOG, "%s: gf_dev IS NULL ======\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case GF_IOC_INIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_INIT gf init======\n", __func__);
		gf_debug(INFO_LOG, "%s: Linux Version %s\n", __func__, GF_LINUX_VERSION);

		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}

		if (gf_dev->system_status) {
			gf_debug(INFO_LOG, "%s: system re-started======\n", __func__);
			break;
		}
		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "goodix_fp_irq", gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
		else
			gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
		gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
		gf_dev->early_suspend.suspend = gf_early_suspend,
		gf_dev->early_suspend.resume = gf_late_resume,
		register_early_suspend(&gf_dev->early_suspend);
#else
		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
		fb_register_client(&gf_dev->notifier);
#endif

		gf_dev->sig_count = 0;
		gf_dev->system_status = 1;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);
		break;

	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		g_vendor_id = info.vendor_id;

		gf_debug(INFO_LOG, "%s: vendor_id 0x%x\n", __func__, g_vendor_id);
		gf_debug(INFO_LOG, "%s: mode 0x%x\n", __func__, info.mode);
		gf_debug(INFO_LOG, "%s: operation 0x%x\n", __func__, info.operation);
		break;

	case GF_IOC_EXIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_EXIT ======\n", __func__);
		gf_disable_irq(gf_dev);
		if (gf_dev->irq) {
			free_irq(gf_dev->irq, gf_dev);
			gf_dev->irq_count = 0;
			gf_dev->irq = 0;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		if (gf_dev->early_suspend.suspend)
			unregister_early_suspend(&gf_dev->early_suspend);
#else
		fb_unregister_client(&gf_dev->notifier);
#endif

		gf_dev->system_status = 0;
		gf_debug(INFO_LOG, "%s: gf exit finished ======\n", __func__);
		break;

	case GF_IOC_RESET:
		printk("%s: chip reset command\n", __func__);
		gf_hw_reset(gf_dev, 60);
		break;

	case GF_IOC_ENABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_IRQ ======\n", __func__);
		gf_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_IRQ ======\n", __func__);
		gf_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
		printk("%s: GF_IOC_ENABLE_SPI_CLK ======\n", __func__);
		gf_spi_clk_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_SPI_CLK:
		printk("%s: GF_IOC_DISABLE_SPI_CLK ======\n", __func__);
		gf_spi_clk_enable(gf_dev, 0);
		break;

	case GF_IOC_ENABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 0);
		break;

	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			gf_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (GF_KEY_HOME == gf_key.key) {
			key_input = GF_KEY_INPUT_HOME;
		} else if (GF_KEY_POWER == gf_key.key) {
			key_input = GF_KEY_INPUT_POWER;
		} else if (GF_KEY_CAMERA == gf_key.key) {
			key_input = GF_KEY_INPUT_CAMERA;
		} else {
			/* add special key define */
			key_input = gf_key.key;
		}
		gf_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
				__func__, key_input, gf_key.key, gf_key.value);

		if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAMERA == gf_key.key) && (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_input, 0);
			input_sync(gf_dev->input);
		}

		if (GF_KEY_HOME == gf_key.key) {
		    input_report_key(gf_dev->input, key_input, gf_key.value);
		    input_sync(gf_dev->input);
		}

		break;

	case GF_IOC_NAV_EVENT:
	    gf_debug(ERR_LOG, "nav event");
		if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
			gf_debug(ERR_LOG, "Failed to copy nav event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		switch (nav_event) {
		    case GF_NAV_FINGER_DOWN:
		    gf_debug(ERR_LOG, "nav finger down");
			break;

		    case GF_NAV_FINGER_UP:
		    gf_debug(ERR_LOG, "nav finger up");
			break;

		    case GF_NAV_DOWN:
			nav_input = GF_NAV_INPUT_DOWN;
			gf_debug(ERR_LOG, "nav down");
			break;

		    case GF_NAV_UP:
			nav_input = GF_NAV_INPUT_UP;
			gf_debug(ERR_LOG, "nav up");
			break;

		    case GF_NAV_LEFT:
			nav_input = GF_NAV_INPUT_LEFT;
			gf_debug(ERR_LOG, "nav left");
			break;

		    case GF_NAV_RIGHT:
			nav_input = GF_NAV_INPUT_RIGHT;
			gf_debug(ERR_LOG, "nav right");
			break;

		    case GF_NAV_CLICK:
			nav_input = GF_NAV_INPUT_CLICK;
			gf_debug(ERR_LOG, "nav click");
			break;

		    case GF_NAV_HEAVY:
			nav_input = GF_NAV_INPUT_HEAVY;
			break;

		    case GF_NAV_LONG_PRESS:
			nav_input = GF_NAV_INPUT_LONG_PRESS;
			break;

		    case GF_NAV_DOUBLE_CLICK:
			nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
			break;

		    default:
			gf_debug(INFO_LOG, "%s: not support nav event nav_event: %d ======\n", __func__, nav_event);
			break;
		}

		if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
		    input_report_key(gf_dev->input, nav_input, 1);
		    input_sync(gf_dev->input);
		    input_report_key(gf_dev->input, nav_input, 0);
		    input_sync(gf_dev->input);
		}
		break;

	case GF_IOC_ENTER_SLEEP_MODE:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENTER_SLEEP_MODE ======\n", __func__);
		break;

	case GF_IOC_GET_FW_INFO:
		gf_debug(INFO_LOG, "%s: GF_IOC_GET_FW_INFO ======\n", __func__);
		buf = gf_dev->need_update;

		gf_debug(DEBUG_LOG, "%s: firmware info  0x%x\n", __func__, buf);
		if (copy_to_user((void __user *)arg, (void *)&buf, sizeof(u8))) {
			gf_debug(ERR_LOG, "Failed to copy data to user\n");
			retval = -EFAULT;
		}

		break;
	case GF_IOC_REMOVE:
		gf_debug(INFO_LOG, "%s: GF_IOC_REMOVE ======\n", __func__);

		/*gf_netlink_destroy(gf_dev);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->input == NULL) {
			mutex_unlock(&gf_dev->release_lock);
			break;
		}
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);

		cdev_del(&gf_dev->cdev);
		sysfs_remove_group(&gf_dev->spi->dev.kobj, &goodix_attribute_group);
		device_destroy(gf_dev->class, gf_dev->devno);
		list_del(&gf_dev->device_entry);
		unregister_chrdev_region(gf_dev->devno, 1);
		class_destroy(gf_dev->class);
		gf_hw_power_enable(gf_dev, 0);
		gf_spi_clk_enable(gf_dev, 0);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->spi_buffer != NULL) {
			kfree(gf_dev->spi_buffer);
			gf_dev->spi_buffer = NULL;
		}
		mutex_unlock(&gf_dev->release_lock);

		spi_set_drvdata(gf_dev->spi, NULL);
		gf_dev->spi = NULL;
		mutex_destroy(&gf_dev->buf_lock);
		mutex_destroy(&gf_dev->release_lock);*/

		break;

	default:
		gf_debug(ERR_LOG, "gf doesn't support this command(%x)\n", cmd);
		break;
	}

	FUNC_EXIT();
	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	FUNC_ENTRY();

	retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

	FUNC_EXIT();
	return retval;
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
	return -EFAULT;
}

/* -------------------------------------------------------------------- */
/* device function								  */
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devno == inode->i_rdev) {
			gf_debug(INFO_LOG, "%s, Found\n", __func__);
			status = 0;
			break;
		}
	}
	mutex_unlock(&device_list_lock);


	gf_get_gpio_dts_info(gf_dev);

	gf_hw_power_enable(gf_dev, 1);

	if (status == 0) {
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		gf_debug(INFO_LOG, "%s, Success to open device. irq = %d\n", __func__, gf_dev->irq);
	} else {
		gf_debug(ERR_LOG, "%s, No device for minor %d\n", __func__, iminor(inode));
	}
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int    status = 0;

	FUNC_ENTRY();
	gf_dev = filp->private_data;
	if (gf_dev->irq)
		gf_disable_irq(gf_dev);
	gf_dev->need_update = 0;
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.	It'll simplify things
	* too, except for the locking.
	*/
	.write =	gf_write,
	.read =		gf_read,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif
	.open =		gf_open,
	.release =	gf_release,
	.poll	= gf_poll,
};

/*-------------------------------------------------------------------------*/

int gf_spi_read_bytes(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf)
 {
	 struct spi_message msg;
	 struct spi_transfer *xfer = NULL;
	 u8 *tmp_buf = NULL;
	 u32 package, reminder, retry;

	 package = (data_len + 2) / 1024;
	 reminder = (data_len + 2) % 1024;

	 if ((package > 0) && (reminder != 0)) {
		 xfer = kzalloc(sizeof(*xfer) * 4, GFP_KERNEL);
		 retry = 1;
	 } else {
		 xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		 retry = 0;
	 }
	 if (xfer == NULL) {
		 gf_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
		 return -ENOMEM;
	 }

	 tmp_buf = gf_dev->spi_buffer;

 #ifndef CONFIG_SPI_MT65XX
	 /* switch to DMA mode if transfer length larger than 32 bytes */
	 if ((data_len + 1) > 32) {
		 gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif

	 spi_message_init(&msg);
	 *tmp_buf = 0xF0;
	 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	 *(tmp_buf + 2) = (u8)(addr & 0xFF);
	 xfer[0].tx_buf = tmp_buf;
	 xfer[0].len = 3;

#ifdef CONFIG_SPI_MT65XX
	xfer[0].speed_hz = gf_spi_speed;
#endif

	 xfer[0].delay_usecs = 5;
	 spi_message_add_tail(&xfer[0], &msg);
	 spi_sync(gf_dev->spi, &msg);

	 spi_message_init(&msg);
	 /* memset((tmp_buf + 4), 0x00, data_len + 1); */
	 /* 4 bytes align */
	 *(tmp_buf + 4) = 0xF1;
	 xfer[1].tx_buf = tmp_buf + 4;
	 xfer[1].rx_buf = tmp_buf + 4;

	 if (retry)
		 xfer[1].len = package * 1024;
	 else
		 xfer[1].len = data_len + 1;

#ifdef CONFIG_SPI_MT65XX
		 xfer[1].speed_hz = gf_spi_speed;
#endif
	 xfer[1].delay_usecs = 5;
	 spi_message_add_tail(&xfer[1], &msg);
	 spi_sync(gf_dev->spi, &msg);

	 /* copy received data */
	 if (retry)
		 memcpy(rx_buf, (tmp_buf + 5), (package * 1024 - 1));
	 else
		 memcpy(rx_buf, (tmp_buf + 5), data_len);

	 /* send reminder SPI data */
	 if (retry) {
		 addr = addr + package * 1024 - 2;
		 spi_message_init(&msg);

		 *tmp_buf = 0xF0;
		 *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
		 *(tmp_buf + 2) = (u8)(addr & 0xFF);
		 xfer[2].tx_buf = tmp_buf;
		 xfer[2].len = 3;
#ifdef CONFIG_SPI_MT65XX
		xfer[2].speed_hz = gf_spi_speed;
#endif
		 xfer[2].delay_usecs = 5;
		 spi_message_add_tail(&xfer[2], &msg);
		 spi_sync(gf_dev->spi, &msg);

		 spi_message_init(&msg);
		 *(tmp_buf + 4) = 0xF1;
		 xfer[3].tx_buf = tmp_buf + 4;
		 xfer[3].rx_buf = tmp_buf + 4;
		 xfer[3].len = reminder + 1;
#ifdef CONFIG_SPI_MT65XX
		xfer[2].speed_hz = gf_spi_speed;
#endif
		 xfer[3].delay_usecs = 5;
		 spi_message_add_tail(&xfer[3], &msg);
		 spi_sync(gf_dev->spi, &msg);

		 memcpy((rx_buf + package * 1024 - 1), (tmp_buf + 6), (reminder - 1));
	 }

#ifndef CONFIG_SPI_MT65XX
	 /* restore to FIFO mode if has used DMA */
	 if ((data_len + 1) > 32) {
		 gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		 spi_setup(gf_dev->spi);
	 }
#endif
	 kfree(xfer);
	 if (xfer != NULL)
		 xfer = NULL;

	 return 0;
 }

static int gf_probe(struct spi_device *spi);
static int gf_remove(struct spi_device *spi);

static struct spi_driver gf_spi_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gf_of_match,
#endif
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int gf_probe(struct spi_device *spi)
{
	struct gf_device *gf_dev = NULL;
	int status = -EINVAL;

	FUNC_ENTRY();

	// set chip_select to 0 since the hw cs only support 0.
	spi->chip_select = 0;

	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
	if (!gf_dev) {
		status = -ENOMEM;
		goto err;
	}

	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->release_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->device_count     = 0;
	gf_dev->probe_finish     = 0;
	gf_dev->system_status    = 0;
	gf_dev->need_update      = 0;

	/* Initialize the driver data */
	gf_dev->spi = spi;

	dev_set_drvdata(&spi->dev, gf_dev);

	/* setup SPI parameters */
	/* CPOL=CPHA=0, speed 1MHz */
	gf_dev->spi->mode = SPI_MODE_0;
	gf_dev->spi->bits_per_word = 8;
	gf_dev->spi->max_speed_hz = 1 * 1000 * 1000;
#ifndef CONFIG_SPI_MT65XX
	memcpy(&gf_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
	gf_dev->spi->controller_data = (void *)&gf_dev->spi_mcc;

	//spi_setup(gf_dev->spi);
#endif
	gf_dev->irq = 0;
	//spi_set_drvdata(spi, gf_dev);

	/* allocate buffer for SPI transfer */
	gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
	if (gf_dev->spi_buffer == NULL) {
		status = -ENOMEM;
		goto err_buf;
	}

	/* create class */
	gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_dev->class)) {
		gf_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
		status = -ENODEV;
		goto err_class;
	}

	/* get device no */
	if (GF_DEV_MAJOR > 0) {
		gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
		status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&gf_dev->devno, gf_dev->device_count++, 1, GF_DEV_NAME);
	}
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to alloc devno.\n", __func__);
		goto err_devno;
	} else {
		gf_debug(INFO_LOG, "%s, major=%d, minor=%d\n", __func__, MAJOR(gf_dev->devno), MINOR(gf_dev->devno));
	}

	/* create device */
	gf_dev->device = device_create(gf_dev->class, &spi->dev, gf_dev->devno, gf_dev, GF_DEV_NAME);
	if (IS_ERR(gf_dev->device)) {
		gf_debug(ERR_LOG, "%s, Failed to create device.\n", __func__);
		status = -ENODEV;
		goto err_device;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
	}

	/* create sysfs */
	status = sysfs_create_group(&spi->dev.kobj, &goodix_attribute_group);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed to create sysfs file.\n", __func__);
		status = -ENODEV;
		goto err_sysfs;
	} else {
		gf_debug(INFO_LOG, "%s, Success create sysfs file.\n", __func__);
	}

	/* cdev init and add */
	cdev_init(&gf_dev->cdev, &gf_fops);
	gf_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed to add cdev.\n", __func__);
		goto err_cdev;
	}

	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		gf_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
		status = -ENOMEM;
		goto err_input;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_KEY_INPUT_HOME, gf_dev->input->keybit);

	__set_bit(GF_KEY_INPUT_MENU, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_BACK, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_POWER, gf_dev->input->keybit);

	__set_bit(GF_NAV_INPUT_UP, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOWN, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_RIGHT, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LEFT, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_CAMERA, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOUBLE_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LONG_PRESS, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_HEAVY, gf_dev->input->keybit);

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input)) {
		gf_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
		status = -ENODEV;
		goto err_input_2;
	}

	/* wakeup source init */
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
	fp_wakeup_source = wakeup_source_register(&spi->dev, "fingerprint wakelock");
#else
	wakeup_source_init(&fp_wakeup_source, "fingerprint wakelock");
#endif

	/* netlink interface init */
	status = gf_netlink_init(gf_dev);
	if (status == -1) {
		mutex_lock(&gf_dev->release_lock);
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);
		goto err_input;
	}

	gf_dev->probe_finish = 1;
	gf_dev->is_sleep_mode = 0;

	// set flag if probe successfully including reading hw id.
	gf_dev->init = true;

	FUNC_EXIT();

	pr_info("gf_probe ok\n");
	return 0;

err_input_2:
	mutex_lock(&gf_dev->release_lock);
	input_free_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

err_input:
	cdev_del(&gf_dev->cdev);

err_cdev:
	sysfs_remove_group(&spi->dev.kobj, &goodix_attribute_group);

err_sysfs:
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

err_device:
	unregister_chrdev_region(gf_dev->devno, 1);

err_devno:
	class_destroy(gf_dev->class);

err_class:
	pr_err("%s cannot find the sensor,now exit\n", __func__);
	gf_hw_power_enable(gf_dev, 0);
	kfree(gf_dev->spi_buffer);
err_buf:
	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
	gf_dev->spi = NULL;
	kfree(gf_dev);
	gf_dev = NULL;
err:
	FUNC_EXIT();
	return status;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf_device *gf_dev = spi_get_drvdata(spi);

	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
		gf_dev->irq_count = 0;
		gf_dev->irq = 0;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (gf_dev->early_suspend.suspend)
		unregister_early_suspend(&gf_dev->early_suspend);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif

	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->input == NULL) {
		kfree(gf_dev);
		mutex_unlock(&gf_dev->release_lock);
		FUNC_EXIT();
		return 0;
	}
	input_unregister_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->spi_buffer != NULL) {
		kfree(gf_dev->spi_buffer);
		gf_dev->spi_buffer = NULL;
	}
	mutex_unlock(&gf_dev->release_lock);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
	wakeup_source_unregister(fp_wakeup_source);
#else
	wakeup_source_trash(&fp_wakeup_source);
#endif
	gf_netlink_destroy(gf_dev);
	cdev_del(&gf_dev->cdev);
	sysfs_remove_group(&spi->dev.kobj, &goodix_attribute_group);
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

	unregister_chrdev_region(gf_dev->devno, 1);
	class_destroy(gf_dev->class);
	gf_hw_power_enable(gf_dev, 0);
	gf_spi_clk_enable(gf_dev, 0);
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		gf_dev->pwr_gpio = -1;
		pr_info("goodix: remove pwr_gpio success\n");
	}
	if (gf_dev->rgltr_ctrl_support && !IS_ERR_OR_NULL(gf_dev->pwr_supply))
	{
		pr_info(" goodix: %s : devm_regulator_put \n", __func__);
		regulator_put(gf_dev->pwr_supply);
		gf_dev->pwr_supply= NULL;
	}
	spin_lock_irq(&gf_dev->spi_lock);
	spi_set_drvdata(spi, NULL);
	gf_dev->spi = NULL;
	spin_unlock_irq(&gf_dev->spi_lock);

	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);

	kfree(gf_dev);
	FUNC_EXIT();
	return 0;
}

/*-------------------------------------------------------------------------*/
static int __init gf_init(void)
{
	int status = 0;

	status = spi_register_driver(&gf_spi_driver);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
		return -EINVAL;
	}

	return status;
}
late_initcall(gf_init);

static void __exit gf_exit(void)
{
	spi_unregister_driver(&gf_spi_driver);
}
module_exit(gf_exit);


MODULE_AUTHOR("goodix");
MODULE_DESCRIPTION("Goodix Fingerprint chip GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206/GF5216/GF5208 TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf_spi");
