/*
 * Copyright (C) 2023 Motorola Mobility LLC
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
#define pr_fmt(fmt) "moto_binder: " fmt

#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <net/rtnetlink.h>
#include <net/sock.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tcp.h>
#include <linux/jiffies.h>
#include <linux/hashtable.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <trace/events/power.h>
#include "moto_binder.h"

atomic_t __read_mostly global_dump_first_pkg = ATOMIC_INIT(0);


static void report_first_packet_after_wakeup(struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct sock *sk;
	uid_t uid;
	uint hook;
	struct net_device *dev = NULL;

	/* skb protection code */
	if (!skb || !skb->len || !state) {
		return;
	}

	hook = state->hook;
	if (NF_INET_LOCAL_IN == hook) {
		dev = state->in;
	}
	if (NULL == dev) {
		return;
	}
	/* skb protection code end */

	sk = skb_to_full_sk(skb);
	if (!sk) {
		return;
	}
	if (sk && !refcount_inc_not_zero(&sk->sk_refcnt)) {
		return;
	}
	uid = sk->sk_uid.val;
	if (uid < MIN_USERAPP_UID) {
		sock_put(sk);
		return;
	}
	sock_put(sk);
	moto_binder_write_status(PACKET_AFTER_WAKEUP, 0, 0, uid, 0, 0, 0);
}

static unsigned int moto_nf_ipv4_in(void *priv,
					struct sk_buff *skb,
					const struct nf_hook_state *state)
{
	if (atomic_read(&global_dump_first_pkg) != 0) {
		atomic_set(&global_dump_first_pkg, 0);
		report_first_packet_after_wakeup(skb, state);
	}

	return NF_ACCEPT;
}

static unsigned int moto_nf_ipv6_in(void *priv,
					struct sk_buff *skb,
					const struct nf_hook_state *state)
{
	if (atomic_read(&global_dump_first_pkg) != 0) {
		atomic_set(&global_dump_first_pkg, 0);
		report_first_packet_after_wakeup(skb, state);
	}

	return NF_ACCEPT;
}

/* Only monitor input network packages */
static struct nf_hook_ops moto_binder_nf_ops[] = {
	{
		.hook     = moto_nf_ipv4_in,
		.pf       = NFPROTO_IPV4,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_SELINUX_LAST + 1,
	},
	{
		.hook     = moto_nf_ipv6_in,
		.pf       = NFPROTO_IPV6,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP6_PRI_SELINUX_LAST + 1,
	},
};

#ifdef CONFIG_PM
static void moto_binder_suspend_resume_hook(void *data, const char *action, int val, bool start)
{
	if (!strcmp(action, "suspend_enter") && !start) {
		atomic_set(&global_dump_first_pkg, 1);
	}
}
#endif

void moto_netfilter_deinit(void)
{
	struct net *net;

	rtnl_lock();
	for_each_net(net) {
		nf_unregister_net_hooks(net, moto_binder_nf_ops, ARRAY_SIZE(moto_binder_nf_ops));
	}
	rtnl_unlock();

#ifdef CONFIG_PM
	unregister_trace_suspend_resume(moto_binder_suspend_resume_hook, NULL);
#endif
}

int moto_netfilter_init(void)
{
	struct net *net = NULL;
	int err = 0;

	rtnl_lock();
	for_each_net(net) {
		err = nf_register_net_hooks(net, moto_binder_nf_ops, ARRAY_SIZE(moto_binder_nf_ops));
		if (err != 0) {
			pr_err("%s: register netfilter failed!\n", __func__);
			break;
		}
	}
	rtnl_unlock();

#ifdef CONFIG_PM
	register_trace_suspend_resume(moto_binder_suspend_resume_hook, NULL);
#endif

	if (err != 0) {
		moto_netfilter_deinit();
		return -1;
	}

	pr_info("%s: register netfilter successfuly!\n", __func__);
	return 0;
}
