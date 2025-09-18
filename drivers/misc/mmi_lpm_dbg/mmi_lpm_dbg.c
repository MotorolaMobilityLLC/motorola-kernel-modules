#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/cpuidle.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/string.h>

#include <linux/mmi_lpm_dbg.h>

enum subsystems {
	MOD_NONE = 0,	//num "0"
	MOD_MD = 0,	//bit 0
	MOD_SCP,		//bit 1
	MOD_MAX
};

static char* subsystem_name[MOD_MAX] = {
	"MD",
	"SCP",
};

struct mmi_lpm_dbg_dev {
	struct device *dev;
	struct device_driver *driver;
	struct device_node *node;
	struct delayed_work check_work;

	unsigned char nonsleep_subsystem;
	unsigned int re_trigger_threshold;//unit second

	unsigned int trigger_cnt[MOD_MAX];
	unsigned long abnormal_time_sum[MOD_MAX];

	unsigned int trigger_threshold;//unit second
	unsigned int check_work_delay;

	unsigned long ap_sleep_time;
	unsigned long md_sleep_time;

	unsigned long pre_scp_sleep_count;
	unsigned long scp_sleep_count;
};

static struct mmi_lpm_dbg_dev mmi_lpm_dbg_dev;
static DEFINE_MUTEX(send_uevent_mutex);

static long sleep_dbg = -1;

static int mmi_lpm_dbg_send_uevent(struct mmi_lpm_dbg_dev *dev_data) {
	struct kobj_uevent_env *env;
	char nonsleep_subsystem[8] = "";
	unsigned long abnormal_time_sum = 0;
	int trigger_cnt = 0;
	int i;
	int ret = 0;

	if ((!dev_data) || (dev_data->nonsleep_subsystem == MOD_NONE)) {
		pr_info("%s skip\n", __func__);
		return -1;
	}

	env = kzalloc(sizeof(*env), GFP_KERNEL);
	if (!env) {
		pr_err("%s: alloc env error", __func__);
		return -1;
	}

	mutex_lock(&send_uevent_mutex);
	for (i = 0; i < MOD_MAX; i++) {
		if (dev_data->nonsleep_subsystem & (1 << i)) {
			abnormal_time_sum = dev_data->abnormal_time_sum[i];
			strcpy(nonsleep_subsystem, (!subsystem_name[i]) ? "Null" : subsystem_name[i]);
			trigger_cnt = dev_data->trigger_cnt[i];

			dev_data->nonsleep_subsystem &= ~(1 << i);

			break;
		}
	}
	mutex_unlock(&send_uevent_mutex);

	if (i >= MOD_MAX) {
		pr_info("%s no ab mode flag 0x%x\n", __func__, dev_data->nonsleep_subsystem);
		goto end;
	}

	add_uevent_var(env, "ABMOD=%s", nonsleep_subsystem);
	add_uevent_var(env, "ABDUR=%ld", abnormal_time_sum);
	add_uevent_var(env, "CNT=%d", trigger_cnt);

	ret = kobject_uevent_env(&dev_data->dev->kobj, KOBJ_CHANGE, env->envp);
	if (ret)
		pr_err("%s: fail, ret=%d", __func__, ret);

	pr_info("%s %s send %s\n", __func__, nonsleep_subsystem, env->envp[0]);

end:
	kfree(env);

	return ret;
}

static unsigned long mmi_lpm_dbg_scp_check_sleep(struct mmi_lpm_dbg_dev *dev_data)
{
	if (!dev_data)
		return -EINVAL;

	if (dev_data->nonsleep_subsystem & (1 << MOD_SCP)) {
		pr_info("%s ab stats still on\n", __func__);
		mmi_lpm_dbg_send_uevent(dev_data);
		return 0;
	}

	/* sleep_dbg is just for debug ops */
	if (sleep_dbg >= 0)
		dev_data->scp_sleep_count = sleep_dbg;

	/* normal state, reset abnormal states parameters */
	if (dev_data->scp_sleep_count != dev_data->pre_scp_sleep_count) {
		dev_data->pre_scp_sleep_count = dev_data->scp_sleep_count;
		dev_data->abnormal_time_sum[MOD_SCP] = 0;
		dev_data->trigger_cnt[MOD_SCP] = 0;

		return 0;
	}

	/* start abnormal state time sum */
	dev_data->abnormal_time_sum[MOD_SCP] += dev_data->ap_sleep_time;
	pr_info("%s ab time %ld cnt %d\n", __func__,
		dev_data->abnormal_time_sum[MOD_SCP],
		dev_data->trigger_cnt[MOD_SCP]);

	/* abnormal state time time reach event trigger target */
	if (dev_data->abnormal_time_sum[MOD_SCP] >= dev_data->trigger_threshold) {
		if (dev_data->abnormal_time_sum[MOD_SCP]
				< dev_data->re_trigger_threshold * dev_data->trigger_cnt[MOD_SCP] ) {
			return 0;
		}

		dev_data->nonsleep_subsystem |= (1 << MOD_SCP);
		dev_data->trigger_cnt[MOD_SCP]++;

		mmi_lpm_dbg_send_uevent(dev_data);
	}

	return dev_data->abnormal_time_sum[MOD_SCP];
}

static unsigned long mmi_lpm_dbg_md_check_sleep(struct mmi_lpm_dbg_dev *dev_data)
{
	if (!dev_data)
		return -EINVAL;

	if (dev_data->nonsleep_subsystem & (1 << MOD_MD)) {
		pr_info("%s ab stats still on\n", __func__);
		mmi_lpm_dbg_send_uevent(dev_data);
		return 0;
	}

	/* sleep_dbg is just for debug ops */
	if (sleep_dbg >= 0)
		dev_data->md_sleep_time = sleep_dbg;

	if (dev_data->md_sleep_time > 0) {
		dev_data->abnormal_time_sum[MOD_MD] = 0;
		dev_data->trigger_cnt[MOD_MD] = 0;
		return 0;

	}

	/* start abnormal state time sum */
	dev_data->abnormal_time_sum[MOD_MD] += dev_data->ap_sleep_time;
	pr_info("%s ab time %ld cnt %d\n",
			__func__,
			dev_data->abnormal_time_sum[MOD_MD],
			dev_data->trigger_cnt[MOD_MD]);

	/* abnormal state time duration reach event trigger target */
	if (dev_data->abnormal_time_sum[MOD_MD] >= dev_data->trigger_threshold) {
		if (dev_data->abnormal_time_sum[MOD_MD]
				< dev_data->re_trigger_threshold * dev_data->trigger_cnt[MOD_MD])
			return 0;

		dev_data->nonsleep_subsystem |= (1 << MOD_MD);
		dev_data->trigger_cnt[MOD_MD]++;

		mmi_lpm_dbg_send_uevent(dev_data);
	}

	return dev_data->abnormal_time_sum[MOD_MD];
}

int mmi_lpm_dbg_subsytem_sleep_call(struct notifier_block *notifier, unsigned long event, void *ptr)
{
	struct mmi_lpm_dbg_dev* dev_data;
	unsigned int* time = (unsigned int*) ptr;

	dev_data = &mmi_lpm_dbg_dev;

	if ((!dev_data) || (!time))
		return NOTIFY_DONE;

	switch (event) {
		case SLEEP_TIME_TYPE_AP:
			dev_data->ap_sleep_time = *time;
			break;
		case SLEEP_TIME_TYPE_MD:
			dev_data->md_sleep_time = *time;
			break;
		case SLEEP_TIME_TYPE_SCP:
			dev_data->scp_sleep_count = *time;
			break;
		default:
			pr_info("%s invalid type %lu", __func__, event);
	}

	pr_debug("%s %lu slp %u", __func__, event, *time);

	return NOTIFY_DONE;
}

static struct notifier_block subsytem_sleep_notifier = {
 	.notifier_call = mmi_lpm_dbg_subsytem_sleep_call
};

static void mmi_lpm_dbg_check_work(struct work_struct *data)
{
	struct delayed_work *dwork = to_delayed_work(data);
	struct mmi_lpm_dbg_dev *dev_data =
			container_of(dwork, struct mmi_lpm_dbg_dev, check_work);

	if (!dev_data)
		return;

	mmi_lpm_dbg_md_check_sleep(dev_data);
	mmi_lpm_dbg_scp_check_sleep(dev_data);

	return;
}

/*-------------------------------debug interface----------------------------------*/

/* Would be at /sys/bus/platform/driver/mmi_lpm_dbg/event_trigger */
static ssize_t event_trigger_store(struct device_driver *driver,
	const char *buf, size_t count)
{
	int ret = 0;
	long i;

	if ((!buf) || (strlen(buf) < 1)) {
		pr_notice("%s Invalid input!!\n", __func__);
		return -EINVAL;
	}

	ret = kstrtol(buf, 10, &i);
	if (ret < 0) {
		pr_notice("kstrtol failed\n");
		return -EINVAL;
	}

	if (i >= MOD_MAX) {
		pr_info("%s unknown %ld! Cat this point for help!\n", __func__, i);
		return -EINVAL;
	}

	pr_info("%s trigger a event for %s!\n", __func__, subsystem_name[i]);
	mmi_lpm_dbg_dev.nonsleep_subsystem |= (1 << i);
	mmi_lpm_dbg_send_uevent(&mmi_lpm_dbg_dev);

	return count;
}

static ssize_t event_trigger_show(struct device_driver *driver, char *buf)
{
	int ret = 0;

	if (!buf) {
		pr_notice("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}

	ret = sprintf(buf, "%d for %s; %d for %s\n", 0, subsystem_name[0], 1, subsystem_name[1]);
	if (ret < 0)
		pr_notice("snprintf failed\n");

	return strlen(buf);
}

static DRIVER_ATTR_RW(event_trigger);

static ssize_t sleep_dbg_store(struct device_driver *driver,
	const char *buf, size_t count)
{
	int ret = 0;

	if ((!buf) || (strlen(buf) < 1)) {
		pr_notice("%s Invalid input!!\n", __func__);
		return -EINVAL;
	}

	ret = kstrtol(buf, 10, &sleep_dbg);
	if (ret < 0)
		pr_notice("%s kstrtol failed\n", __func__);

	pr_info("%s set %lu\n", __func__, sleep_dbg);

	return count;
}

static ssize_t sleep_dbg_show(struct device_driver *driver, char *buf)
{
	int ret = 0;

	if (buf == NULL) {
		pr_notice("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}

	ret = sprintf(buf, "%ld\n", sleep_dbg);
	if (ret < 0)
		pr_notice("snprintf failed\n");

	return strlen(buf);
}

DRIVER_ATTR_RW(sleep_dbg);

static ssize_t trigger_thresh_dbg_store(struct device_driver *driver,
	const char *buf, size_t count)
{
	int ret = 0;

	if ((!buf) || (strlen(buf) < 1)) {
		pr_notice("%s Invalid input!!\n", __func__);
		return -EINVAL;
	}

	ret = kstrtouint(buf, 10, &mmi_lpm_dbg_dev.trigger_threshold);
	if (ret < 0)
		pr_notice("kstrtol failed\n");

	pr_info("%s set %d\n", __func__, mmi_lpm_dbg_dev.trigger_threshold);

	return count;
}

static ssize_t trigger_thresh_dbg_show(struct device_driver *driver, char *buf)
{
	int ret = 0;

	if (buf == NULL) {
		pr_notice("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}

	ret = sprintf(buf, "%d\n", mmi_lpm_dbg_dev.trigger_threshold);
	if (ret < 0)
		pr_notice("snprintf failed\n");

	return strlen(buf);
}

static DRIVER_ATTR_RW(trigger_thresh_dbg);

static ssize_t re_trigger_dbg_store(struct device_driver *driver,
	const char *buf, size_t count)
{
	int ret = 0;

	if ((!buf) || (strlen(buf) < 1)) {
		pr_notice("%s() Invalid input!!\n", __func__);
		return -EINVAL;
	}

	ret = kstrtouint(buf, 10, &mmi_lpm_dbg_dev.re_trigger_threshold);
	if (ret < 0)
		pr_notice("kstrtol failed\n");

	pr_info("%s set %d\n", __func__, mmi_lpm_dbg_dev.re_trigger_threshold);

	return count;
}

static ssize_t re_trigger_dbg_show(struct device_driver *driver, char *buf)
{
	int ret = 0;

	if (buf == NULL) {
		pr_notice("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}

	ret = sprintf(buf, "%d\n", mmi_lpm_dbg_dev.re_trigger_threshold);
	if (ret < 0)
		pr_notice("snprintf failed\n");

	return strlen(buf);
}

static DRIVER_ATTR_RW(re_trigger_dbg);

static struct driver_attribute *mmi_event_attr_list[] = {
	&driver_attr_event_trigger,
	//&driver_attr_sleep_dbg,
	&driver_attr_trigger_thresh_dbg,
	&driver_attr_re_trigger_dbg,
};

static int mmi_lpm_dbg_create_attr(struct device_driver *driver)
{
	int idx, err;
	int num = ARRAY_SIZE(mmi_event_attr_list);

	if (!driver) {
		pr_info("%s drv is null\n", __func__);
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, mmi_event_attr_list[idx]);
		if (err) {
			pr_info("%s driver_create_file %d err:%d\n",
					__func__, idx, err);
			break;
		}
	}
	return err;
}
/*-------------------------------debug interface end------------------------------*/

#ifdef CONFIG_PM_SLEEP
/*
 * @brief: Suspend function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int32_t mmi_lpm_dbg_suspend(struct device* dev)
{
	struct mmi_lpm_dbg_dev *dev_data = dev_get_drvdata(dev);

	cancel_delayed_work(&dev_data->check_work);

	return 0;
}

/*
 * @brief: Resume function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int32_t mmi_lpm_dbg_resume(struct device* dev)
{
	struct mmi_lpm_dbg_dev *dev_data = dev_get_drvdata(dev);

	schedule_delayed_work(&dev_data->check_work, msecs_to_jiffies(dev_data->check_work_delay));

	return 0;
}

const struct dev_pm_ops mmi_lpm_dbg_pm_ops =
{
	.suspend = mmi_lpm_dbg_suspend,
	.resume = mmi_lpm_dbg_resume,
};
#endif /* CONFIG_PM_SLEEP */

static int mmi_lpm_dbg_probe(struct platform_device *pdev)
{
	struct mmi_lpm_dbg_dev *drvdata;
	struct device_node *node;
	int i;

	drvdata = &mmi_lpm_dbg_dev;
	//drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if ((!drvdata) || (!pdev))
		return -ENOMEM;

	pr_info("%s\n", __func__);

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	node = drvdata->dev->of_node;
	if (!node) {
		pr_info("%s not availiable here\n", __func__);
		return 0;
	}

	drvdata->driver = pdev->dev.driver;//&pdriver->driver;
	if ((drvdata->dev == NULL) || (drvdata->driver == NULL)) {
		pr_info("%s dev or driver is null\n", __func__);
		return 0;
	}

	if (of_property_read_u32(node , "trigger-threshold", &drvdata->trigger_threshold))
		drvdata->trigger_threshold = 1200;//default threshold 20min

	if (of_property_read_u32(node , "check-work-delay", &drvdata->check_work_delay))
		drvdata->check_work_delay = 200;//default delay 200ms

	if (of_property_read_u32(node , "re-trigger-threshold", &drvdata->re_trigger_threshold))
		drvdata->re_trigger_threshold = 7200;//default repeat threshold 2 h

	pr_info("%s trigger thresh %d, work delay %d, retrigger %d\n",
			__func__,
			drvdata->trigger_threshold,
			drvdata->check_work_delay,
			drvdata->re_trigger_threshold);

	for(i = 0; i < MOD_MAX; i++)
		drvdata->trigger_cnt[i] = 0;

	for(i = 0; i < MOD_MAX; i++)
		drvdata->abnormal_time_sum[i] = 0;

	drvdata->pre_scp_sleep_count= 0;
	drvdata->nonsleep_subsystem = MOD_NONE;

	INIT_DELAYED_WORK(&drvdata->check_work, mmi_lpm_dbg_check_work);

	mmi_lpm_dbg_register_notifier(&subsytem_sleep_notifier);

	mmi_lpm_dbg_create_attr(drvdata->driver);

	return 0;
}

static int mmi_lpm_dbg_remove(struct platform_device *pdev)
{
	struct mmi_lpm_dbg_dev *drvdata;

	if (!pdev) {
		return -EINVAL;
	}

	drvdata = dev_get_drvdata(&pdev->dev);
	if (!drvdata) {
		return -EINVAL;
	}

	cancel_delayed_work(&drvdata->check_work);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static const struct of_device_id mmi_lpm_dbg_match_table[] = {
	{ .compatible = "mmi,lpm-dbg" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mmi_lpm_dbg_match_table);

static struct platform_driver mmi_lpm_dbg_driver = {
	.driver	= {
		.name		= "mmi_lpm_dbg",
		.of_match_table	= mmi_lpm_dbg_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &mmi_lpm_dbg_pm_ops,
#endif
	},
	.probe	= mmi_lpm_dbg_probe,
	.remove	= mmi_lpm_dbg_remove,

};
module_platform_driver(mmi_lpm_dbg_driver);

MODULE_DESCRIPTION("MOTO mmi lpm_dbg driver");
MODULE_LICENSE("GPL v2");
