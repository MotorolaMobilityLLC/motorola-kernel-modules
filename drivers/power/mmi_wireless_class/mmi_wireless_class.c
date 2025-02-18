#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/power_supply.h>

#include <linux/mmi_wireless_class.h>

#define to_wireless_device(obj) container_of(obj, struct wireless_device, dev)

static struct class *wireless_class = NULL;

#define WLS_RX_CHECK(x)\
bool wls_rx_check_##x(struct wireless_device *wls_dev)\
{\
	if (wls_dev != NULL && wls_dev->rx_ops != NULL && wls_dev->rx_ops->check_##x)\
		return wls_dev->rx_ops->check_##x(wls_dev);\
	return false;\
}\
EXPORT_SYMBOL(wls_rx_check_##x);

#define WLS_RX_GET(x,y)\
int wls_rx_get_##x(struct wireless_device *wls_dev, y)\
{\
	if (wls_dev != NULL && wls_dev->rx_ops != NULL && wls_dev->rx_ops->get_##x)\
		return wls_dev->rx_ops->get_##x(wls_dev, a);\
	return -EOPNOTSUPP;\
}\
EXPORT_SYMBOL(wls_rx_get_##x);

#define WLS_RX_SET(x,y)\
int wls_rx_set_##x(struct wireless_device *wls_dev, y)\
{\
	if (wls_dev != NULL && wls_dev->rx_ops != NULL && wls_dev->rx_ops->set_##x)\
		return wls_dev->rx_ops->set_##x(wls_dev, a);\
	return -EOPNOTSUPP;\
}\
EXPORT_SYMBOL(wls_rx_set_##x);

#define WLS_TX_CHECK(x)\
bool wls_tx_check_##x(struct wireless_device *wls_dev)\
{\
	if (wls_dev != NULL && wls_dev->tx_ops != NULL && wls_dev->tx_ops->check_##x)\
		return wls_dev->tx_ops->check_##x(wls_dev);\
	return false;\
}\
EXPORT_SYMBOL(wls_tx_check_##x);

#define WLS_TX_GET(x,y)\
int wls_tx_get_##x(struct wireless_device *wls_dev, y)\
{\
	if (wls_dev != NULL && wls_dev->tx_ops != NULL && wls_dev->tx_ops->get_##x)\
		return wls_dev->tx_ops->get_##x(wls_dev, a);\
	return -EOPNOTSUPP;\
}\
EXPORT_SYMBOL(wls_tx_get_##x);

#define WLS_TX_SET(x,y)\
int wls_tx_set_##x(struct wireless_device *wls_dev, y)\
{\
	if (wls_dev != NULL && wls_dev->tx_ops != NULL && wls_dev->tx_ops->set_##x)\
		return wls_dev->tx_ops->set_##x(wls_dev, a);\
	return -EOPNOTSUPP;\
}\
EXPORT_SYMBOL(wls_tx_set_##x);


WLS_RX_GET(chip_id, int *a);
WLS_RX_GET(fw_version, int *a);
WLS_RX_GET(rx_neg_power, int *a);
WLS_RX_GET(op_mode, int *a);
WLS_RX_GET(sys_mode, int *a);
WLS_RX_GET(rx_die_temp, int *a);
WLS_RX_GET(mode_select, int *a);
WLS_RX_GET(fw_update_status, int *a);
WLS_RX_GET(ce, int *a);
WLS_RX_GET(mcode, uint16_t *a);

WLS_RX_GET(rx_irect, int *a);
WLS_RX_GET(rx_iout, int *a);
WLS_RX_GET(rx_vrect, int *a);
WLS_RX_GET(rx_vout, int *a);
WLS_RX_GET(rx_vout_setting, int *a);

WLS_RX_SET(irq_enable, bool a);
WLS_RX_SET(mode_select, bool a);
WLS_RX_SET(fw_update, bool a);
WLS_RX_SET(mc_det, bool a);
WLS_RX_SET(lowpower_mode, bool a);

WLS_RX_CHECK(ldo_on);
WLS_RX_CHECK(dump_info);

int wls_get_message_size(int header)
{
	int size = 0;

	if (header < 0x20) {
		size = 1;
	} else if (header < 0x80) {
		size = header / 16;
	} else if (header < 0xE0) {
		size = header / 8 - 8;
	} else {
		size = header / 4 - 36;
	}

	return size;
}
EXPORT_SYMBOL(wls_get_message_size);

int wls_rx_send_ask_packet(struct wireless_device *wls_dev, uint8_t *data, int data_len)
{
	if (wls_dev != NULL &&
		wls_dev->rx_ops != NULL &&
		wls_dev->rx_ops->send_ask_packet)
		return wls_dev->rx_ops->send_ask_packet(wls_dev, data, data_len);
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(wls_rx_send_ask_packet);

static ssize_t name_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct wireless_device *wls_dev = to_wireless_device(dev);

	return snprintf(buf, 20, "%s\n",
				wls_dev->props.alias_name ?
				wls_dev->props.alias_name : "anonymous");
}

static DEVICE_ATTR_RO(name);

static struct attribute *wireless_class_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group wireless_group = {
	.attrs = wireless_class_attrs,
};

static const struct attribute_group *wireless_groups[] = {
	&wireless_group,
	NULL,
};

static void wireless_device_release(struct device *dev)
{
	struct wireless_device *wls_dev = to_wireless_device(dev);

	if (!IS_ERR_OR_NULL(wls_dev))
		kfree(wls_dev);
}

struct wireless_device *wireless_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct wls_rx_ops *rx_ops,
		const struct wls_tx_ops *tx_ops,
		const struct wireless_properties *props)
{
	struct wireless_device *wls_dev = NULL;
	int rc;

	pr_info("%s: name=%s\n", __func__, name);

	if(IS_ERR_OR_NULL(wireless_class)) {
		pr_err("%s: wireless_class is err or null\n", __func__);
		return NULL;
	}

	wls_dev = kzalloc(sizeof(*wls_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(wls_dev)) {
		pr_err("%s: wls_dev kzalloc failed\n", __func__);
		return NULL;
	}

	wls_dev->dev.class = wireless_class;
	wls_dev->dev.parent = parent;
	wls_dev->dev.release = wireless_device_release;
	dev_set_name(&wls_dev->dev,"%s", name);
	dev_set_drvdata(&wls_dev->dev, devdata);

	/* Copy properties */
	if (props) {
		memcpy(&wls_dev->props, props, sizeof(struct wireless_properties));
	}
	rc = device_register(&wls_dev->dev);
	if (rc) {
		kfree(wls_dev);
		pr_err("%s device register failed!\n", __func__);
		return ERR_PTR(rc);
	}

	if (!IS_ERR_OR_NULL(rx_ops))
		wls_dev->rx_ops = rx_ops;

	if (!IS_ERR_OR_NULL(tx_ops))
		wls_dev->tx_ops = tx_ops;

	pr_info("%s successfully, wireless_dev=%p\n", __func__, wls_dev);

	return wls_dev;
}
EXPORT_SYMBOL(wireless_device_register);

void wireless_device_unregister(struct wireless_device *wls_dev)
{
	if (IS_ERR_OR_NULL(wireless_class))
		return;

	if (IS_ERR_OR_NULL(wls_dev))
		return;

	device_unregister(&wls_dev->dev);
}
EXPORT_SYMBOL(wireless_device_unregister);

bool wireless_dev_set_callback(struct wireless_device *wls_dev, void *callback_ops)
{
	if (IS_ERR_OR_NULL(wls_dev) || IS_ERR_OR_NULL(callback_ops))
		return false;
	wls_dev->callback_ops = callback_ops;
	return (!IS_ERR_OR_NULL(wls_dev->callback_ops));
}
EXPORT_SYMBOL(wireless_dev_set_callback);

int wls_dev_send_event(struct wireless_device *wls_dev, struct wls_event_msg *msg)
{
	if (IS_ERR_OR_NULL(wls_dev) ||
		IS_ERR_OR_NULL(wls_dev->callback_ops)||
		IS_ERR_OR_NULL(wls_dev->callback_ops->event_handler)) {
		return -1;
	}

	return wls_dev->callback_ops->event_handler(wls_dev, msg);
}
EXPORT_SYMBOL(wls_dev_send_event);

static int wireless_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;
	return strcmp(dev_name(dev), name) == 0;
}

struct wireless_device *get_wireless_by_name(const char *name)
{
	struct device *dev;

	if (IS_ERR_OR_NULL(wireless_class))
		return NULL;

	if (!name)
		return NULL;

	dev = class_find_device(wireless_class, NULL, name,
				wireless_match_device_by_name);
	pr_info("%s wireless_dev=%p\n", __func__, dev);

	return dev ? to_wireless_device(dev) : NULL;
}
EXPORT_SYMBOL(get_wireless_by_name);

static void __exit wireless_class_exit(void)
{
	class_destroy(wireless_class);
}

static int __init wireless_class_init(void)
{
	wireless_class = class_create(THIS_MODULE, "mmi_wireless_class");
	if (IS_ERR(wireless_class)) {
		pr_notice("Unable to create wireless class; errno = %ld\n",
			PTR_ERR(wireless_class));
		return PTR_ERR(wireless_class);
	}
	pr_info("%s wireless_class=%p\n", __func__, wireless_class);
	wireless_class->dev_groups = wireless_groups;
	return 0;
}

module_init(wireless_class_init);
module_exit(wireless_class_exit);

MODULE_DESCRIPTION("Mmi Wireless Class Device");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");

