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
#define pr_fmt(fmt) "skb_sched: " fmt
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
#include <linux/signal.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/task.h>
#include <linux/string.h>
#include <trace/events/task.h>

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

#define PROC_NUMBUF 13

/*
 * At most allow setting 10 tasks
 */
#define TASK_SIZE_MAX 10
#define PRIO_STRING 10
#define INVALID 0xFF
/*
 * task_array_comm: comm1,comm2,comm3..
 * task_array_prio: prio1,prio2,prio3..
 */

#define SKBSCHED_DIR "skb_sched"
#define SKBSCHED_ENABLE "skb_sched_enable"
#define APP_LIST "app_list"
#define PRIO_LIST "priority_list"
#define DSCP_LIST "dscp_list"

struct proc_dir_entry *proc_entry;
static char task_array_comm[TASK_SIZE_MAX][TASK_COMM_LEN] = {{0}};
static u32 task_array_uid[TASK_SIZE_MAX] = {0};
static char task_array_prio[TASK_SIZE_MAX][PRIO_STRING] = {{0}};
static u8 task_array_dscp[TASK_SIZE_MAX] = {0};
static int task_num = 0;

static bool enable_debug = false;
module_param(enable_debug, bool, 0644);
MODULE_PARM_DESC(enable_debug, "Enable more debug logs");

static unsigned int skb_sched_enable = 0;
static ssize_t skb_sched_enable_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
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

	skb_sched_enable = enable;

	return count;
}

static ssize_t skb_sched_enable_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", skb_sched_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops skb_sched_enable_ops = {
	.proc_write	= skb_sched_enable_write,
	.proc_read	= skb_sched_enable_read,
	.proc_lseek	= default_llseek,
};

/*
 * app list format app1,app2,app3,app4....,
 */
static ssize_t app_list_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char *buffer = (char *)kzalloc(count, GFP_KERNEL);
	int offset = 0, read_count = 0, i = 0;

	if (!buffer)
		return -ENOMEM;

	if (copy_from_user(buffer, buf, count)) {
		kfree(buffer);
		return -EFAULT;
	}

	if (buffer[count-1] != ',' && buffer[count-2] != ',') { //Need to be compatible with echo command
		pr_err("Incorrect paramters, the last character must be ','\n");
		kfree(buffer);
		return -EINVAL;
	}
	/*
	 * sscanf is different with glibc. A field width is required when using %[^], we
	 * set it to 16 since TASK_COMM_LEN is 16. Refer to lib/vsprinf.c
	 */
	while(i < TASK_SIZE_MAX && sscanf(buffer + offset, "%16[^,], %n", task_array_comm[i], &read_count) > 0) {
		/* The max read count should be TASK_COMM_LEN + a char of ','*/
		if (read_count > TASK_COMM_LEN + 1 || ((*(buffer + offset + read_count - 1) != ',') &&
				(count != offset + read_count))) {
			pr_err("Incorrect paramters\n");
			kfree(buffer);
			return -EINVAL;
		}

		offset += read_count;
		strcpy(task_array_prio[i], "nset"); //Default set to nset
		task_array_dscp[i] = INVALID;
		i++;
		task_num = i;
	}

	kfree(buffer);
	return count;
}

static int app_list_show(struct seq_file *m, void *v)
{
	int i;
	for (i = 0; i < TASK_SIZE_MAX; i++) {
		seq_printf(m, "app_list[%d][%d][%s]\n", i, task_array_uid[i], task_array_comm[i]);
	}
	return 0;
}

static int app_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, app_list_show, NULL);
}

static const struct proc_ops app_list_ops = {
	.proc_open	= app_list_open,
	.proc_write	= app_list_write,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

/*
 * priority list format prio1,prio2,prio3,prio4....,
 */
static ssize_t priority_list_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char *buffer = (char *)kzalloc(count, GFP_KERNEL);
	int offset = 0, read_count = 0, i = 0;
	if (!buffer)
		return -ENOMEM;

	if (copy_from_user(buffer, buf, count)) {
		kfree(buffer);
		return -EFAULT;
	}

	if (buffer[count-1] != ',' && buffer[count-2] != ',') { //Need to be compatible with echo command
		pr_err("Incorrect paramters, the last character must be ','\n");
		kfree(buffer);
		return -EINVAL;
	}

	while (i < task_num && sscanf(buffer + offset, "%5[^,], %n", task_array_prio[i], &read_count) > 0) {
		if (*(buffer + offset + read_count - 1) != ',' && (count != offset + read_count)) {
			pr_err("Incorrect paramters\n");
			kfree(buffer);
			return -EINVAL;
		}
		offset += read_count;
		i++;
	}
	kfree(buffer);
	return count;
}

static int priority_list_show(struct seq_file *m, void *v)
{
	int i;
	for (i = 0; i < TASK_SIZE_MAX; i++) {
		seq_printf(m, "priority_list[%d] %s \n", i, task_array_prio[i]);
	}
	return 0;
}

static int priority_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, priority_list_show, NULL);
}

static const struct proc_ops priority_list_ops = {
	.proc_open	= priority_list_open,
	.proc_write	= priority_list_write,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

/*
 * priority list format prio1,prio2,prio3,prio4....,
 */
static ssize_t dscp_list_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char *buffer = (char *)kzalloc(count, GFP_KERNEL);
	int offset = 0, read_count = 0, i = 0;
	char value[4] = {0};
	if (!buffer)
		return -ENOMEM;

	if (copy_from_user(buffer, buf, count)) {
		kfree(buffer);
		return -EFAULT;
	}

	if (buffer[count-1] != ',' && buffer[count-2] != ',') { //Need to be compatible with echo command
		pr_err("Incorrect paramters, the last character must be ','\n");
		kfree(buffer);
		return -EINVAL;
	}

	while (i < task_num && sscanf(buffer + offset, "%2[^,],%n", value, &read_count) > 0) {
		if (((*(buffer + offset + read_count - 1) != ',') && (count != offset + read_count)) ||
				kstrtou8(value, 10, &task_array_dscp[i])) {
			pr_err("Incorrect paramters\n");
			kfree(buffer);
			return -EINVAL;
		}
		offset += read_count;
		i++;
	}
	kfree(buffer);
	return count;
}

static int dscp_list_show(struct seq_file *m, void *v)
{
	int i;
	for (i = 0; i < TASK_SIZE_MAX; i++) {
		seq_printf(m, "dscp_list[%d] 0x%02x \n", i, task_array_dscp[i]);
	}
	return 0;
}

static int dscp_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, dscp_list_show, NULL);
}

static const struct proc_ops dscp_list_ops = {
	.proc_open	= dscp_list_open,
	.proc_write	= dscp_list_write,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int __init proc_skb_sched_init(void)
{
	proc_entry = proc_mkdir(SKBSCHED_DIR, NULL);

	if (!proc_entry) {
		pr_err("create skb_sched proc fail\n");
		return -ENOENT;
	}

	proc_create(SKBSCHED_ENABLE, 0666, proc_entry, &skb_sched_enable_ops);
	proc_create(APP_LIST, 0666, proc_entry, &app_list_ops);
	proc_create(PRIO_LIST, 0666, proc_entry, &priority_list_ops);
	proc_create(DSCP_LIST, 0666, proc_entry, &dscp_list_ops);

	return 0;
}

static void proc_skb_sched_deinit(void)
{
	remove_proc_entry(SKBSCHED_ENABLE, proc_entry);
	remove_proc_entry(APP_LIST, proc_entry);
	remove_proc_entry(PRIO_LIST, proc_entry);
	remove_proc_entry(DSCP_LIST, proc_entry);
	remove_proc_entry(SKBSCHED_DIR, NULL);
}

/*
 * pkt priority refer to pkt_sched.h
 * TC_PRIO_BESTEFFORT              0
 * TC_PRIO_FILLER                  1
 * TC_PRIO_BULK                    2
 * TC_PRIO_INTERACTIVE_BULK        4
 * TC_PRIO_INTERACTIVE             6
 * TC_PRIO_CONTROL                 7
 * TC_PRIO_MAX                     15
 */
static void set_pkt_prio(struct sk_buff *skb)
{
	struct net_device *dev = NULL;
	struct iphdr *iph = NULL;
	struct ipv6hdr *ipv6h = NULL;
	struct sock *sk = NULL;
	int ori_prio = 0, prio = 0, i = 0;
	u8 tos, dscp;

	if (skb == NULL)
		return;

	ori_prio = skb->priority;
	sk = skb_to_full_sk(skb);
	if (sk) {
		for (i = 0; i < task_num; i++) {
			if (sk->sk_uid.val == task_array_uid[i] && task_array_uid[i] > 0) {
				if (!strcasecmp(task_array_prio[i], "fast")) {
					prio = 7; // enqueue to fifo 0
				} else if (!strcasecmp(task_array_prio[i], "mid")) {
					prio = 0; // enqueue to fifo 1
				} else if (!strcasecmp(task_array_prio[i], "slow")) {
					prio = 1; // enqueue to fifo 2
				} else {
					prio = INVALID;
				}
				if (prio != INVALID) {
					skb->priority = prio;
				}
				/*
				 * -----------IPv6 Header-------------
				 * 0-----3 4----9 10-11 12----------32
				 * Version  DSCP   CU     FLow Label
				 *
				 *------------IPv4 Header-------------
				 * 0-----3 4----7 8----13 14-15 16--32
				 * Version Header  DSCP    CU   Length
				 */
				if (skb->protocol == htons(ETH_P_IPV6)) {
					ipv6h = ipv6_hdr(skb);
					tos = (ipv6h->priority << 4) | (ipv6h->flow_lbl[0] >> 4);
					dscp = (tos & 0xfc) >> 2;
					if (task_array_dscp[i] != INVALID) {
						ipv6h->priority = (((task_array_dscp[i] << 2) & 0xf0) >> 4);
						ipv6h->flow_lbl[0] |= ((task_array_dscp[i] & 0x03) << 6);
					}
				} else {
					iph = ip_hdr(skb);
					tos = iph->tos;
					dscp = (tos & 0xfc) >> 2;
					if (task_array_dscp[i] != INVALID) {
						iph->tos = (task_array_dscp[i] << 2) | (tos & 0x03);
					}
				}
				if (enable_debug) {
					dev = skb_dst(skb)->dev;
					if (dev == NULL) {
						return;
					}
					if (skb->protocol == htons(ETH_P_IPV6)) {
						trace_printk("[%s][%d][TOS:0x%02x->0x%02x][OrigDSCP:0x%02x][Priority %d->%d]SA: "NIP6_FMT" DA: "NIP6_FMT"\n",
							dev->name, sk->sk_uid.val, tos, (ipv6h->priority << 4) | (ipv6h->flow_lbl[0] >> 4),dscp, ori_prio,
							skb->priority, NIP6(ipv6h->saddr), NIP6(ipv6h->daddr));
					} else {
						trace_printk("[%s][%d][TOS:0x%02x->0x%02x][OrigDSCP:0x%02x][Priority %d->%d]SA: "NIPQUAD_FMT" DA: "NIPQUAD_FMT"\n",
						dev->name, sk->sk_uid.val, tos, iph->tos, dscp, ori_prio, skb->priority, NIPQUAD(iph->saddr), NIPQUAD(iph->daddr));
					}
				}
				break;
			}
		}
	} else {
		// Do not change skb priority
		return;
	}
}

static unsigned int skb_sched_ip4_out_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (skb_sched_enable)
		set_pkt_prio(skb);
	return NF_ACCEPT;
}

static unsigned int skb_sched_ip6_out_hook(void *priv,
		struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (skb_sched_enable)
		set_pkt_prio(skb);
	return NF_ACCEPT;
}

static struct nf_hook_ops skb_sched_hook_ops[] __read_mostly = {
	{
		.hook		= skb_sched_ip4_out_hook,
		.pf		= NFPROTO_IPV4,
		.hooknum	= NF_INET_POST_ROUTING,
		.priority	= NF_INET_NUMHOOKS + 1, // Make sure we set it after netfilter
	},
	{
		.hook		= skb_sched_ip6_out_hook,
		.pf		= NFPROTO_IPV6,
		.hooknum	= NF_INET_POST_ROUTING,//NF_INET_LOCAL_OUT,
		.priority	= NF_INET_NUMHOOKS + 1,
	},
};

static void get_process_handler(void *data, struct task_struct *task, unsigned long clone_flags)
{
	int i;
	for (i = 0; i < task_num; i++) {
		if (!strcmp(task->comm, task_array_comm[i])) {
			task_array_uid[i] = task->cred->uid.val;
			break;
		}
	}
}

static int __init skb_sched_init(void)
{
	int ret = 0;
	ret = proc_skb_sched_init();
	if (ret != 0)
		return ret;

	ret = nf_register_net_hooks(&init_net, skb_sched_hook_ops,
				    ARRAY_SIZE(skb_sched_hook_ops));
	if (ret != 0) {
		pr_err("Register net hook failed, ret=%d\n", ret);
		return ret;
	}
	ret = register_trace_task_newtask(get_process_handler, NULL);
	if (ret != 0) {
		pr_err("Register trace_task_rename hook failed, ret=%d\n", ret);
		return ret;
	}
	return ret;
}

static void __exit skb_sched_exit(void)
{
	proc_skb_sched_deinit();
	nf_unregister_net_hooks(&init_net, skb_sched_hook_ops,
				ARRAY_SIZE(skb_sched_hook_ops));
	unregister_trace_task_newtask(get_process_handler, NULL);
}

module_init(skb_sched_init);
module_exit(skb_sched_exit);
MODULE_LICENSE("GPL v2");
