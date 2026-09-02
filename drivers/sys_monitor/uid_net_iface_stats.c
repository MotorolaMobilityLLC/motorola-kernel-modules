/*
 * Copyright (C) 2026 Motorola Mobility LLC
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
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/jhash.h>
#include <linux/hashtable.h>
#include <linux/spinlock.h>
#include <linux/uidgid.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <net/inet_hashtables.h>

#if IS_ENABLED(CONFIG_IPV6)
#include <net/ipv6.h>
#include <net/inet6_hashtables.h>
#endif

#include "monitor.h"


#define STATS_HASH_BITS 10
#ifndef IFNAMSIZ
#define IFNAMSIZ 16
#endif

struct uid_iface_val {
	kuid_t uid;
	char ifname[IFNAMSIZ];
	u64 rx_bytes;
	u64 rx_packets;
	u64 tx_bytes;
	u64 tx_packets;
	struct hlist_node hnode;
};

static DEFINE_HASHTABLE(stats_table, STATS_HASH_BITS);
static DEFINE_SPINLOCK(stats_lock);

static int stats_enabled = 1; // 0: off, 1: on

// Extern the exported TCP and UDP hash tables
extern struct inet_hashinfo tcp_hashinfo;
extern struct udp_table udp_table;

static struct sock *lookup_socket_v4(struct net *net, const struct sk_buff *skb, const struct net_device *indev)
{
	const struct iphdr *iph = ip_hdr(skb);
	__be32 saddr = iph->saddr;
	__be32 daddr = iph->daddr;
	__be16 sport, dport;
	struct sock *sk = NULL;

	if (iph->protocol == IPPROTO_TCP) {
		struct tcphdr _hdr;
		const struct tcphdr *th = skb_header_pointer(skb, ip_hdrlen(skb), sizeof(_hdr), &_hdr);
		if (th) {
			sport = th->source;
			dport = th->dest;
			// Lookup established TCP socket (fully sufficient for mobile client traffic)
			sk = __inet_lookup_established(net, &tcp_hashinfo, saddr, sport, daddr, ntohs(dport), indev->ifindex, 0);
		}
	} else if (iph->protocol == IPPROTO_UDP) {
		struct udphdr _hdr;
		const struct udphdr *uh = skb_header_pointer(skb, ip_hdrlen(skb), sizeof(_hdr), &_hdr);
		if (uh) {
			sport = uh->source;
			dport = uh->dest;
			sk = udp4_lib_lookup(net, saddr, sport, daddr, dport, indev->ifindex);
		}
	}
	return sk;
}

#if IS_ENABLED(CONFIG_IPV6)
static struct sock *lookup_socket_v6(struct net *net, const struct sk_buff *skb, const struct net_device *indev)
{
	const struct ipv6hdr *ip6h = ipv6_hdr(skb);
	const struct in6_addr *saddr = &ip6h->saddr;
	const struct in6_addr *daddr = &ip6h->daddr;
	__be16 sport, dport;
	struct sock *sk = NULL;
	u8 proto = ip6h->nexthdr;
	int offset = sizeof(struct ipv6hdr);

	if (proto == IPPROTO_TCP) {
		struct tcphdr _hdr;
		const struct tcphdr *th = skb_header_pointer(skb, offset, sizeof(_hdr), &_hdr);
		if (th) {
			sport = th->source;
			dport = th->dest;
			// Lookup established IPv6 TCP socket
			sk = __inet6_lookup_established(net, &tcp_hashinfo, saddr, sport, daddr, ntohs(dport), indev->ifindex, 0);
		}
	} else if (proto == IPPROTO_UDP) {
		struct udphdr _hdr;
		const struct udphdr *uh = skb_header_pointer(skb, offset, sizeof(_hdr), &_hdr);
		if (uh) {
			sport = uh->source;
			dport = uh->dest;
			sk = udp6_lib_lookup(net, saddr, sport, daddr, dport, indev->ifindex);
		}
	}
	return sk;
}
#endif

static inline kuid_t skb_uid(const struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct sock *sk = skb ? skb->sk : NULL;
	kuid_t uid = INVALID_UID;

	if (sk) {
		struct sock *full_sk = sk_to_full_sk(sk);
		if (full_sk && sk_fullsock(full_sk))
			uid = full_sk->sk_uid;
	} else if (state->hook == NF_INET_LOCAL_OUT) {
		uid = current_uid();
	} else if (state->hook == NF_INET_LOCAL_IN) {
		struct sock *lookup_sk = NULL;
		struct net *net = state->net;
		const struct net_device *indev = state->in;

		if (state->pf == NFPROTO_IPV4) {
			lookup_sk = lookup_socket_v4(net, skb, indev);
		}
#if IS_ENABLED(CONFIG_IPV6)
		else if (state->pf == NFPROTO_IPV6) {
			lookup_sk = lookup_socket_v6(net, skb, indev);
		}
#endif

		if (lookup_sk) {
			if (sk_fullsock(lookup_sk))
				uid = lookup_sk->sk_uid;
			sock_gen_put(lookup_sk);
		}
	}
	return uid;
}

static inline void update_stats(kuid_t uid, const char *ifname, unsigned int len, bool is_tx)
{
	struct uid_iface_val *entry;
	u32 key_hash;
	unsigned long flags;

	if (!uid_valid(uid) || from_kuid(&init_user_ns, uid) == 0)
		return;

	if (ifname && !strcmp(ifname, "lo"))
		return;

	u32 uid_val = from_kuid(&init_user_ns, uid);
	key_hash = jhash(ifname, strnlen(ifname, IFNAMSIZ), uid_val);
	spin_lock_irqsave(&stats_lock, flags);
	hash_for_each_possible(stats_table, entry, hnode, key_hash) {
		if (uid_eq(entry->uid, uid) && !strncmp(entry->ifname, ifname, IFNAMSIZ)) {
			goto found;
		}
	}
	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry) {
		spin_unlock_irqrestore(&stats_lock, flags);
		return;
	}
	entry->uid = uid;
	strscpy(entry->ifname, ifname, IFNAMSIZ);
	hash_add(stats_table, &entry->hnode, key_hash);
found:
	if (is_tx) {
		entry->tx_bytes += len;
		entry->tx_packets++;
	} else {
		entry->rx_bytes += len;
		entry->rx_packets++;
	}
	spin_unlock_irqrestore(&stats_lock, flags);
}

static void clear_stats_table(void)
{
	struct uid_iface_val *entry;
	struct hlist_node *tmp;
	unsigned long flags;
	int bkt;
	spin_lock_irqsave(&stats_lock, flags);
	hash_for_each_safe(stats_table, bkt, tmp, entry, hnode) {
		hash_del(&entry->hnode);
		kfree(entry);
	}
	spin_unlock_irqrestore(&stats_lock, flags);
}

static unsigned int hook_func(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	kuid_t uid;
	const char *ifname;
	if (!skb || !state || !stats_enabled)
		return NF_ACCEPT;
	if (state->hook == NF_INET_LOCAL_OUT && state->out) {
		uid = skb_uid(skb, state);
		ifname = state->out->name;
		update_stats(uid, ifname, skb->len, true);
	} else if (state->hook == NF_INET_LOCAL_IN && state->in) {
		uid = skb_uid(skb, state);
		ifname = state->in->name;
		update_stats(uid, ifname, skb->len, false);
	}
	return NF_ACCEPT;
}

static struct nf_hook_ops nf_ops[] = {
	{
		.hook = hook_func,
		.pf = NFPROTO_IPV4,
		.hooknum = NF_INET_LOCAL_OUT,
		.priority = NF_IP_PRI_LAST,
	},
	{
		.hook = hook_func,
		.pf = NFPROTO_IPV4,
		.hooknum = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_LAST,
	},
	{
		.hook = hook_func,
		.pf = NFPROTO_IPV6,
		.hooknum = NF_INET_LOCAL_OUT,
		.priority = NF_IP_PRI_LAST,
	},
	{
		.hook = hook_func,
		.pf = NFPROTO_IPV6,
		.hooknum = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_LAST,
	},
};


ssize_t uid_iface_stats_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	if (kstrtoint(buf, 10, &val) == 0) {
		if (val == 1) {
			stats_enabled = 1;
		} else if (val == 0) {
			stats_enabled = 0;
		} else if (val == 2) {
			clear_stats_table();
		}
	}
	return count;
}

ssize_t uid_iface_stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct uid_iface_val *snapshot, *entry;
	unsigned long flags;
	int count = 0;
	int bkt, len = 0;
	int max_entries = 512;

	snapshot = kvmalloc_array(max_entries, sizeof(struct uid_iface_val), GFP_KERNEL);
	if (!snapshot) return -ENOMEM;

	spin_lock_irqsave(&stats_lock, flags);

	hash_for_each(stats_table, bkt, entry, hnode) {
		if (count >= max_entries) break;
		snapshot[count] = *entry;
		count++;
	}

	spin_unlock_irqrestore(&stats_lock, flags);

	len += scnprintf(buf + len, PAGE_SIZE - len, "uid ifname rx_bytes rx_packets tx_bytes tx_packets\n");
	for(int i = 0; i < count; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u %s %llu %llu %llu %llu\n",
			from_kuid(&init_user_ns, snapshot[i].uid), snapshot[i].ifname,
			snapshot[i].rx_bytes, snapshot[i].rx_packets, snapshot[i].tx_bytes, snapshot[i].tx_packets);
		if (PAGE_SIZE - len < 128)
			break;
	}

	kvfree(snapshot);
	return len;
}

static int net_uid_iface_stats_nf_init(void)
{
	int ret = nf_register_net_hooks(&init_net, nf_ops, ARRAY_SIZE(nf_ops));
	if (ret)
		pr_err("net_uid_iface_stats: nf_register_net_hooks failed: %d\n", ret);
	return ret;
}

static void net_uid_iface_stats_nf_exit(void)
{
	nf_unregister_net_hooks(&init_net, nf_ops, ARRAY_SIZE(nf_ops));
	clear_stats_table();
}

sys_monitor_attr(uid_iface_stats);

static struct attribute *net_stats_attrs[] = {
	&uid_iface_stats_attr.attr,
	NULL,
};

static const struct attribute_group net_stats_attr_group = {
	.attrs = net_stats_attrs,
};

int monitor_net_stats_init(struct kobject *parent_kobj)
{
	int ret = 0;
	ret = sysfs_create_group(parent_kobj, &net_stats_attr_group);
	if (ret) {
		pr_err("%s: misc node create failed\n", __func__);
		ret = -ENOMEM;
	}

	return net_uid_iface_stats_nf_init();
}

void monitor_net_stats_exit(struct kobject *parent_kobj)
{
	sysfs_remove_group(parent_kobj, &net_stats_attr_group);
	net_uid_iface_stats_nf_exit();
}
