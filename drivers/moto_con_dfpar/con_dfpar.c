/*
 * Copyright (C) 2023 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#define pr_fmt(fmt) "DFPAR: " fmt
#include <linux/types.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/kallsyms.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/netlink.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/skbuff.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/sched/task.h>
#include <linux/signal.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <trace/events/power.h>


#define PROC_NUMBUF 13

#define NIPQUAD(addr) \
		((unsigned char *)&addr)[0], \
		((unsigned char *)&addr)[1], \
		((unsigned char *)&addr)[2], \
		((unsigned char *)&addr)[3]

#define NIPQUAD_FMT "%u.%u.%u.%u"

#define NIP6(addr) \
		ntohs((addr).s6_addr16[0]), \
		ntohs((addr).s6_addr16[1]), \
		ntohs((addr).s6_addr16[2]), \
		ntohs((addr).s6_addr16[3]), \
		ntohs((addr).s6_addr16[4]), \
		ntohs((addr).s6_addr16[5]), \
		ntohs((addr).s6_addr16[6]), \
		ntohs((addr).s6_addr16[7])

#define NIP6_FMT "%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x"

static int dump_packet_rx = 0;
#ifdef TRACK_OUT_PACKET
static int dump_packet_tx = 0;
#endif

static void detect_packet_owner(struct sk_buff *skb)
{
	struct task_struct *task = &init_task;

	struct sock *sk = NULL;
	struct file *filp = NULL;
	u32 uid = 0;

	if (NULL == skb) {
		return;
	}

	sk = skb_to_full_sk(skb);
	if (NULL == sk || sock_flag(sk, SOCK_DEAD) || NULL == sk->sk_socket) {
		pr_info("socket is null\n");
		return;
	}

	filp = sk->sk_socket->file;
	if (NULL == filp) {
		return;
	}

	uid = filp->f_cred->fsuid.val;
	if (!uid) {
		pr_info("uid is null\n");
		return;
	}

	for_each_process(task)
	{
		if (task->cred->uid.val == uid) {
			pr_info("Packet Info: UID: %d, name: %s\n", uid, task->comm);
			break;
		}
	}
}

static void dump_v4packet(struct sk_buff *skb) {
	struct iphdr *iph = NULL;

	iph = ip_hdr(skb);

	if (NULL == iph) {
		return;
	}

	if (iph->protocol == IPPROTO_UDP || iph->protocol == IPPROTO_TCP) {
		pr_info("SA: "NIPQUAD_FMT" DA: "NIPQUAD_FMT"\n", NIPQUAD(iph->saddr), NIPQUAD(iph->daddr));
	}
}

static void dump_v6packet(struct sk_buff *skb) {
	struct ipv6hdr *ipv6h = NULL;

	ipv6h = ipv6_hdr(skb);

	if (NULL == ipv6h) {
		return;
	}

	if (ipv6h->nexthdr == IPPROTO_UDP || ipv6h->nexthdr == IPPROTO_TCP) {
		pr_info("SA: "NIP6_FMT" DA: "NIP6_FMT"\n", NIP6(ipv6h->saddr), NIP6(ipv6h->daddr));
	}
}

static unsigned int con_dfpar_enable = 0; //default off

static ssize_t con_dfpar_enable_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int err, enable;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtouint(strstrip(buffer), 0, &enable);
	if (err)
		return err;

	con_dfpar_enable = enable;

	return count;
}

static ssize_t con_dfpar_enable_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", con_dfpar_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops con_dfpar_enable_ops = {
	.proc_write	= con_dfpar_enable_write,
	.proc_read	= con_dfpar_enable_read,
	.proc_lseek	= default_llseek,
};

#define DFPAR_DIR "con_dfpar"
#define DFPAR_ENABLE "con_dfpar_enable"

static int __init proc_con_dfpar_init(void)
{
	struct proc_dir_entry *proc_entry;
	proc_entry = proc_mkdir(DFPAR_DIR, NULL);

	if (!proc_entry) {
		pr_err("create con_dfpar fail\n");
		return -ENOENT;
	}

	proc_create(DFPAR_ENABLE, 0666, proc_entry, &con_dfpar_enable_ops);

	return 0;
}

static unsigned int con_dfpar_ip4_in_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (dump_packet_rx != 0) {
		pr_info("Dump the first rx ipv4 packet\n");
		detect_packet_owner(skb);
		dump_v4packet(skb);
		dump_packet_rx = 0;
	}
	return NF_ACCEPT;
}

static unsigned int con_dfpar_ip6_in_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (dump_packet_rx != 0) {
		pr_info("Dump the first rx ipv6 packet\n");
		detect_packet_owner(skb);
		dump_v6packet(skb);
		dump_packet_rx = 0;
	}
	return NF_ACCEPT;
}

#ifdef TRACK_OUT_PACKET
static unsigned int con_dfpar_ip4_out_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (dump_packet_tx != 0) {
		pr_info("Dump the first tx ipv4 packet\n");
		detect_packet_owner(skb);
		dump_v4packet(skb);
		dump_packet_tx = 0;
	}
	return NF_ACCEPT;
}

static unsigned int con_dfpar_ip6_out_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (dump_packet_tx != 0) {
		pr_info("Dump the first tx ipv6 packet\n");
		detect_packet_owner(skb);
		dump_v6packet(skb);
		dump_packet_tx = 0;
	}
	return NF_ACCEPT;
}
#endif

static struct nf_hook_ops con_dfpar_hook_ops[] __read_mostly = {
	{
		.hook		= con_dfpar_ip4_in_hook,
		.pf		= NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_RAW_BEFORE_DEFRAG + 1,
	},
	{
		.hook		= con_dfpar_ip6_in_hook,
		.pf		= NFPROTO_IPV6,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP6_PRI_RAW_BEFORE_DEFRAG + 1,
	},
#ifdef TRACK_OUT_PACKET
	{
		.hook		= con_dfpar_ip4_out_hook,
		.pf		= NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_OUT,
		.priority	= NF_IP_PRI_RAW_BEFORE_DEFRAG + 1,
	},
	{
		.hook		= con_dfpar_ip6_out_hook,
		.pf		= NFPROTO_IPV6,
		.hooknum	= NF_INET_LOCAL_OUT,
		.priority	= NF_IP6_PRI_RAW_BEFORE_DEFRAG + 1,
	},
#endif
};

#ifdef CONFIG_PM
static void con_dfpar_suspend_resume_hook(void *data, const char *action, int val, bool start)
{
	if (con_dfpar_enable != 0) {
		if (!strcmp(action, "suspend_enter") && !start) {
			pr_info("suspend exit, will dump the first received TCP/UDP packet");
			dump_packet_rx = 1;
#ifdef TRACK_OUT_PACKET
			dump_packet_tx = 1;
#endif
		}
	}
}
#endif

static int __init con_dfpar_init(void)
{
	int ret = 0;
	ret = proc_con_dfpar_init();
	if (ret != 0)
		return ret;

	ret = nf_register_net_hooks(&init_net, con_dfpar_hook_ops,
				    ARRAY_SIZE(con_dfpar_hook_ops));
	if (ret != 0) {
		pr_err("Register net hook failed, ret=%d\n", ret);
		return ret;
	}
#ifdef CONFIG_PM
	register_trace_suspend_resume(con_dfpar_suspend_resume_hook, NULL);
#endif
	return ret;
}

static void __exit con_dfpar_exit(void)
{
	nf_unregister_net_hooks(&init_net, con_dfpar_hook_ops,
				ARRAY_SIZE(con_dfpar_hook_ops));
#ifdef CONFIG_PM
	unregister_trace_suspend_resume(con_dfpar_suspend_resume_hook, NULL);
#endif
}

module_init(con_dfpar_init);
module_exit(con_dfpar_exit);
MODULE_LICENSE("GPL v2");
