/*
 * Copyright (C) 2025 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "sys_monitor: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/suspend.h>
#include <linux/rtc.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <net/rtnetlink.h>
#include <net/sock.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tcp.h>
#include <linux/timekeeping.h>
#include <linux/kernel_stat.h>
#include <linux/power_supply.h>

#include <clocksource/arm_arch_timer.h>

#define STATE_MAX 60

#define SLEEP_REC_COUNT 60
#define MAX_WAKEUP_NAME_SIZE 32
#define SUBSYS_NAME_LEN 16

#if IS_ENABLED(CONFIG_QCOM_STATS)
#include <linux/soc/qcom/smem.h>

struct subsystem_data {
	const char *name;
	u32 smem_item;
	u32 pid;
	bool not_present;
};

static struct subsystem_data subsystems[] = {
	{ "modem", 605, 1 },
	{ "wpss", 605, 13 },
	{ "adsp", 606, 2 },
	{ "cdsp", 607, 5 },
	{ "cdsp1", 607, 12 },
	{ "gpdsp0", 607, 17 },
	{ "gpdsp1", 607, 18 },
	{ "slpi", 608, 3 },
	{ "gpu", 609, 0 },
	{ "display", 610, 0 },
	{ "adsp_island", 613, 2 },
	{ "slpi_island", 613, 3 },
	{ "apss", 631, QCOM_SMEM_HOST_ANY },
};

struct stats_data {
	bool appended_stats_avail;
	void __iomem *base;
};

struct sleep_stats {
	u32 stat_type;
	u32 count;
	u64 last_entered_at;
	u64 last_exited_at;
	u64 accumulated;
};

#endif

struct subsys_state {
	char name[SUBSYS_NAME_LEN];
	uint64_t sleep_time;
};

struct suspend_state {
	time64_t sec;
	int charge_counter;
	int sleep_time;
	int current_avg;
	int capacity;
	int subsys_count;
	unsigned int wakeup_irq;
	uid_t wakeup_uid;
	char wakeup_name[MAX_WAKEUP_NAME_SIZE];
	struct subsys_state subsys_state[14];
};

static struct suspend_state suspend_state[SLEEP_REC_COUNT];
static atomic_t __read_mostly global_dump_first_pkg = ATOMIC_INIT(0);

static DEFINE_MUTEX(sleep_state_mutex);
static struct power_supply *batt_psy;
static ktime_t last_suspend_time;
static bool wakeup_netfilter_ready = false;
static uid_t net_wakeup_uid = 0;
static int cur_idx = 0;

static int get_battery_property(enum power_supply_property psp)
{
	union power_supply_propval prop = {0,};
	int ret, value = 0;

	if (!batt_psy) {
		batt_psy = power_supply_get_by_name("battery");
		if (!batt_psy)
			return 0;
	}

	ret = power_supply_get_property(batt_psy, psp, &prop);

	if (!ret)
		value = prop.intval;

	return value;
}

ssize_t sleep_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i,j;
	int once_len, len = 0;
	int total_record = cur_idx;
	int remain_elements;
	struct tm tm;
	time64_t seconds;

	mutex_lock(&sleep_state_mutex);
	for (i = 0; i < cur_idx; i++) {
		seconds = suspend_state[i].sec;
		time64_to_tm(seconds, 0 , &tm);
		once_len = len;
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02d:%02d:%02d|%d|%d|%d|%d|%d|%s|%d|",
					tm.tm_hour, tm.tm_min, tm.tm_sec, suspend_state[i].charge_counter,
					suspend_state[i].current_avg, suspend_state[i].capacity, suspend_state[i].sleep_time,
					suspend_state[i].wakeup_irq, suspend_state[i].wakeup_name, suspend_state[i].wakeup_uid);
		for (j = 0; j < suspend_state[i].subsys_count; j++) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%s:%llu|",
					suspend_state[i].subsys_state[j].name, suspend_state[i].subsys_state[j].sleep_time / 19200000L);
		}
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		once_len = len - once_len;
		if ((PAGE_SIZE - len) < (once_len + 128))
			break;
	}
	i++;
	if (i < total_record) {
		memmove(&suspend_state[0], &suspend_state[i], (total_record - i) * sizeof(struct suspend_state));
		cur_idx = total_record - i;
	} else {
		cur_idx = 0;
	}
	remain_elements = total_record - cur_idx;
	memset(&suspend_state[cur_idx], 0 , remain_elements * sizeof(struct suspend_state));

	mutex_unlock(&sleep_state_mutex);

	return len;
}

static void record_sleep_stats(ktime_t sleep_time)
{
#if IS_ENABLED(CONFIG_QCOM_STATS)
	static uint64_t prev_duration[ARRAY_SIZE(subsystems)];
	u64 accumulated;
	uint64_t delta_duration;
	int j, i;
	struct subsystem_data *subsystem;
	struct sleep_stats *stat;

	for (i = 0, j = 0; i < ARRAY_SIZE(subsystems); i++) {
		subsystem = &subsystems[i];

		if (subsystem->not_present)
			continue;

		stat = qcom_smem_get(subsystem->pid, subsystem->smem_item, NULL);
		if (IS_ERR(stat)) {
			subsystem->not_present = true;
			continue;
		} else
			subsystem->not_present = false;

		accumulated = stat->accumulated;
		if (stat->last_entered_at > stat->last_exited_at)
			accumulated += arch_timer_read_counter()
			       - stat->last_entered_at;
		delta_duration = accumulated - prev_duration[i];
		prev_duration[i] = accumulated;
		suspend_state[cur_idx].subsys_state[j].sleep_time = delta_duration;
		memcpy(suspend_state[cur_idx].subsys_state[j].name, subsystem->name, SUBSYS_NAME_LEN);
		suspend_state[cur_idx].subsys_state[j].name[SUBSYS_NAME_LEN - 1] = '\0';
		suspend_state[cur_idx].subsys_count = ++j;
	}
#else
	strlcpy(suspend_state[cur_idx].subsys_state[0].name, "dummy", SUBSYS_NAME_LEN);
	suspend_state[cur_idx].subsys_state[0].sleep_time = 100000;
	suspend_state[cur_idx].subsys_state[0].name[SUBSYS_NAME_LEN - 1] = '\0';
	suspend_state[cur_idx].subsys_count = 1;
#endif

	suspend_state[cur_idx].sec = ktime_get_real_seconds() - sys_tz.tz_minuteswest * 60;
	suspend_state[cur_idx].current_avg = get_battery_property(POWER_SUPPLY_PROP_CURRENT_AVG);
	suspend_state[cur_idx].charge_counter = get_battery_property(POWER_SUPPLY_PROP_CHARGE_COUNTER);
	suspend_state[cur_idx].capacity = get_battery_property(POWER_SUPPLY_PROP_CAPACITY);
	suspend_state[cur_idx].sleep_time = sleep_time / 1000;
	suspend_state[cur_idx].wakeup_irq = 0;//pm_wakeup_irq();
	suspend_state[cur_idx].wakeup_uid = net_wakeup_uid;
	if (suspend_state[cur_idx].wakeup_irq) {
		struct irq_desc *desc;
		const char *name = "null";
		desc = irq_to_desc(suspend_state[cur_idx].wakeup_irq);
		if (desc == NULL)
			name = "stray irq";
		else if (desc->action && desc->action->name)
			name = desc->action->name;

		strlcpy(suspend_state[cur_idx].wakeup_name, name, MAX_WAKEUP_NAME_SIZE);
	}

	cur_idx++;
}

static int sys_monitor_pm_event(struct notifier_block *notifier, unsigned long pm_event,
				void *unused)
{
	ktime_t now;
	ktime_t delta;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		last_suspend_time = ktime_get_boottime();
		atomic_set(&global_dump_first_pkg, 1);
		net_wakeup_uid = 0;
		break;
	case PM_POST_SUSPEND:
		now = ktime_get_boottime();
		delta = ktime_sub(now, last_suspend_time);
		if (ktime_to_ms(delta) < 500)
			break;
		mutex_lock(&sleep_state_mutex);
		record_sleep_stats(ktime_to_ms(delta));
		if (cur_idx >= SLEEP_REC_COUNT)
			cur_idx = 0;
		mutex_unlock(&sleep_state_mutex);
		atomic_set(&global_dump_first_pkg, 0);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static void report_first_packet_after_wakeup(struct sk_buff *skb)
{
	struct sock *sk = NULL;
	uid_t uid = 0;

	if (!skb)
		return;

	sk = skb_to_full_sk(skb);
	if (!sk || !sk_fullsock(sk) || !refcount_inc_not_zero(&sk->sk_refcnt)) {
		return;
	}
	uid = sk->sk_uid.val;
	sock_put(sk);

	if (uid >= 10000) {
		net_wakeup_uid = uid;
	}
}

static unsigned int wakeup_nf_ipv4_in(void *priv,
					struct sk_buff *skb,
					const struct nf_hook_state *state)
{
	if (atomic_read(&global_dump_first_pkg) != 0) {
		atomic_set(&global_dump_first_pkg, 0);
		report_first_packet_after_wakeup(skb);
	}

	return NF_ACCEPT;
}

static unsigned int wakeup_nf_ipv6_in(void *priv,
					struct sk_buff *skb,
					const struct nf_hook_state *state)
{
	if (atomic_read(&global_dump_first_pkg) != 0) {
		atomic_set(&global_dump_first_pkg, 0);
		report_first_packet_after_wakeup(skb);
	}

	return NF_ACCEPT;
}

static struct nf_hook_ops wakeup_nf_ops[] = {
	{
		.hook     = wakeup_nf_ipv4_in,
		.pf       = NFPROTO_IPV4,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_SELINUX_LAST + 1,
	},
	{
		.hook     = wakeup_nf_ipv6_in,
		.pf       = NFPROTO_IPV6,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP6_PRI_SELINUX_LAST + 1,
	},
};

static void wakeup_netfilter_deinit(void)
{
	struct net *net;

	if (wakeup_netfilter_ready) {
		rtnl_lock();
		for_each_net(net) {
			nf_unregister_net_hooks(net, wakeup_nf_ops, ARRAY_SIZE(wakeup_nf_ops));
		}
		rtnl_unlock();
	}

	wakeup_netfilter_ready = false;
}

static int wakeup_netfilter_init(void)
{
	struct net *net = NULL;
	int err = 0;

	rtnl_lock();
	for_each_net(net) {
		err = nf_register_net_hooks(net, wakeup_nf_ops, ARRAY_SIZE(wakeup_nf_ops));
		if (err != 0) {
			pr_err("%s: register netfilter failed!\n", __func__);
			break;
		}
	}
	rtnl_unlock();

	if (err != 0) {
		wakeup_netfilter_deinit();
		return -1;
	}

	atomic_set(&global_dump_first_pkg, 0);
	wakeup_netfilter_ready = true;
	pr_info("%s: register netfilter successfuly!\n", __func__);
	return 0;
}

static struct notifier_block sys_monitor_notifier_func = {
	.notifier_call = sys_monitor_pm_event,
	.priority = 0,
};

int monitor_sleep_init(void)
{
	int ret = 0;

	ret = register_pm_notifier(&sys_monitor_notifier_func);
	if (ret) {
		pr_err("Failed to register power debug notifier\n");
		return ret;
	}
	
	wakeup_netfilter_init();

	return 0;
}

void monitor_sleep_exit(void)
{
	unregister_pm_notifier(&sys_monitor_notifier_func);
	wakeup_netfilter_deinit();
}


