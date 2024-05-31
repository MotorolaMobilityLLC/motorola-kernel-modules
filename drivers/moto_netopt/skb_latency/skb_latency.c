/*
 * Copyright (C) 2024 Motorola Mobility LLC
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
#define pr_fmt(fmt) "skb_latency: " fmt
#include <linux/types.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/kallsyms.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/netlink.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/list.h>
#include <linux/signal.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/task.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tcp.h>

struct proc_dir_entry *proc_entry;
#define SKB_LATENCY_DIR "skb_latency"
#define LATENCY_ENABLE "enable"
#define MONITOR_APP "comm_name"
#define PROC_NUMBUF 13
char comm_name[TASK_COMM_LEN] = {0};
u32 task_uid = 0;
static unsigned int latency_enable = 0;

static bool enable_debug = false;
module_param(enable_debug, bool, 0644);
MODULE_PARM_DESC(enable_debug, "Enable more debug logs");

typedef struct tracked_tcp_seq {
    /* tcp socket */
    struct sock *sock;
    /* TCP seq */
    __be32 seq;
    /* kernel time stamp */
    ktime_t tstamp;
    struct list_head list;
} tracked_tcp_seq;

// At most tracking the latest 512 packets
#define MAX_TRACKED_COUNT 512
static int list_length = 0;
static u64 measured_rtt = 0;
struct list_head tracked_list = LIST_HEAD_INIT(tracked_list);

static DEFINE_SPINLOCK(skb_latency_lock);

/*
 * Create a new node for a TCP packet, to record the seq and the kernel time stamp
 * Free the oldest node if exist nodes count exceeds to MAX_TRACKED_COUNT
 */
void create_node(struct sock *sock, __be32 seq) {
    tracked_tcp_seq *new_entry = NULL;
    new_entry = kmalloc(sizeof(tracked_tcp_seq), GFP_KERNEL);
    if (new_entry) {
        new_entry->seq = seq;
        new_entry->sock = sock;
        new_entry->tstamp = ktime_get_real();
        list_add_tail(&new_entry->list, &tracked_list);
        list_length++;

        if (list_length > MAX_TRACKED_COUNT) {
            tracked_tcp_seq *entry = list_first_entry_or_null(&tracked_list, tracked_tcp_seq, list);
            if (entry) {
                list_del(&entry->list);
                kfree(entry);
                list_length--;
            }
        }
    }
}

/*
 * Find the node that has the same seq. If found, get the timestamp of this node and
 * free all oldest nodes before this node
 */
ktime_t find_and_delete_olders(struct sock *sock, __be32 target_seq) {
    ktime_t result_tstamp = 0;
    tracked_tcp_seq *entry, *tmp, *pos;
    bool found = false;

    list_for_each_entry_safe_reverse(entry, tmp, &tracked_list, list) {
        if (entry->sock == sock && entry->seq <= target_seq) {
            pos = tmp;
            found = true;
            result_tstamp = entry->tstamp;
            break;
        }
    }
    if (!found) {
        return 0;
    } else {
        list_for_each_entry_safe(entry, tmp, &tracked_list, list) {
            if (entry == pos)
                break;
            list_del(&entry->list);
            kfree(entry);
            list_length--;
        }
    }
    return result_tstamp;
}

/*
 * Free all exist nodes
 */
void clean_all_nodes(void) {
    tracked_tcp_seq *entry, *tmp;
    list_for_each_entry_safe(entry, tmp, &tracked_list, list) {
        list_del(&entry->list);
        kfree(entry);
    }
    list_length = 0;
}

/*
 * Get the seq of a TCP packet before sending, and store the seq to a new node
 */
static unsigned int hook_func_out(void *priv, struct sk_buff *skb, const struct nf_hook_state *state) {
    struct tcphdr *th = NULL;

    if (!latency_enable || !task_uid)
        return NF_ACCEPT;

    if (skb->protocol == htons(ETH_P_IP) &&
        ((struct iphdr *)ip_hdr(skb))->protocol == IPPROTO_TCP &&
        ((struct sock *)skb_to_full_sk(skb))->sk_uid.val == task_uid) {
        th = tcp_hdr(skb);
        if (th && skb->sk) {
            spin_lock(&skb_latency_lock);
            create_node(skb->sk, th->seq);
            spin_unlock(&skb_latency_lock);
        }
    }
    return NF_ACCEPT;
}

/*
 * Get the ack_seq of a received TCP packet. Traverse the nodes list to find the corresponding sent
 * TCP packet.
 * RTT = (time stamp of the ack_seq) - (time stamp of the seq)
 */
static unsigned int hook_func_in(void *priv, struct sk_buff *skb, const struct nf_hook_state *state) {
    struct tcphdr *th = NULL;
    __be32 ack_seq = 0;
    ktime_t tstamp = 0;
    ktime_t result_tstamp = 0;
    struct sock *sk = NULL;

    if (!latency_enable || !task_uid)
        return NF_ACCEPT;

    sk = skb->sk;
    if (!sk) {
        return NF_ACCEPT;
    /* We need to make sure the socket has not been destoryed */
    } else if (refcount_inc_not_zero(&sk->sk_refcnt)) {
        if (skb->protocol == htons(ETH_P_IP) &&
            ((struct iphdr *)ip_hdr(skb))->protocol == IPPROTO_TCP &&
            sk->sk_uid.val == task_uid) {
            th = tcp_hdr(skb);
            if (th) {
                ack_seq = th->ack_seq;
                tstamp = ktime_get_real();
                spin_lock(&skb_latency_lock);
                result_tstamp = find_and_delete_olders(sk, ack_seq);
                spin_unlock(&skb_latency_lock);
                if (result_tstamp != 0) {
                    measured_rtt = ktime_to_ns(ktime_sub(tstamp, result_tstamp));
                    if (enable_debug) {
                        pr_info("Measured RTT for skb: %llu ns\n", measured_rtt);
                    }
                }
            }
        }
        sock_put(sk);
    }
    return NF_ACCEPT;
}

static ssize_t comm_name_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct task_struct *task = &init_task;
    if (count > TASK_COMM_LEN - 1)
        count = TASK_COMM_LEN - 1;

    if (copy_from_user(comm_name, buf, count))
        return -EFAULT;

    comm_name[count] = '\0';
    for_each_process(task)
    {
        if (strcmp(task->comm, comm_name) == 0) {
            task_uid = task->cred->uid.val;
            pr_info("Update tracked process: UID %d, COMM: [%s]\n", task_uid, task->comm);
            spin_lock(&skb_latency_lock);
            clean_all_nodes();
            spin_unlock(&skb_latency_lock);
            break;
        }
    }
    return count;
}

static ssize_t comm_name_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[64];
    size_t len = 0;
    len = snprintf(buffer, sizeof(buffer), "[%u]: Latest measured RTT was %llu ns\n", task_uid, measured_rtt);
    return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops comm_name_ops = {
    .proc_write	= comm_name_write,
    .proc_read	= comm_name_read,
    .proc_lseek	= default_llseek,
};

static ssize_t latency_enable_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
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
    latency_enable = enable;
    if (!latency_enable) {
        spin_lock(&skb_latency_lock);
        clean_all_nodes();
        spin_unlock(&skb_latency_lock);
    }
    return count;
}

static ssize_t latency_enable_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[64];
    size_t len = 0;
    len = snprintf(buffer, sizeof(buffer), "enabled:%u, cached_length: %u\n", latency_enable, list_length);
    return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops skb_latency_enable_ops = {
    .proc_write	= latency_enable_write,
    .proc_read	= latency_enable_read,
    .proc_lseek	= default_llseek,
};

static int __init proc_init(void)
{
    proc_entry = proc_mkdir(SKB_LATENCY_DIR, NULL);

    if (!proc_entry) {
        pr_err("create SKB latency proc fail\n");
        return -ENOENT;
    }

    proc_create(LATENCY_ENABLE, 0666, proc_entry, &skb_latency_enable_ops);
    proc_create(MONITOR_APP, 0666, proc_entry, &comm_name_ops);

    return 0;
}

static void proc_deinit(void)
{
    remove_proc_entry(LATENCY_ENABLE, proc_entry);
    remove_proc_entry(MONITOR_APP, proc_entry);
    remove_proc_entry(SKB_LATENCY_DIR, NULL);
}

static struct nf_hook_ops latency_hook_ops[] __read_mostly = {
    {
        .hook		= hook_func_out,
        .pf		= PF_INET,
        .hooknum	= NF_INET_LOCAL_OUT,
        .priority	= NF_IP_PRI_FIRST,
    },
    {
        .hook		= hook_func_in,
        .pf		= PF_INET,
        .hooknum	= NF_INET_LOCAL_IN,
        .priority	= NF_IP_PRI_FIRST,
    },
};

static int __init network_latency_init(void) {
    int ret = 0;
    ret = proc_init();
    if (ret != 0)
        return ret;
    nf_register_net_hooks(&init_net, latency_hook_ops, ARRAY_SIZE(latency_hook_ops));
    return 0;
}

static void __exit network_latency_exit(void) {
    if(proc_entry)
        proc_deinit();
    nf_unregister_net_hooks(&init_net, latency_hook_ops, ARRAY_SIZE(latency_hook_ops));
    spin_lock(&skb_latency_lock);
    clean_all_nodes();
    spin_unlock(&skb_latency_lock);
}

module_init(network_latency_init);
module_exit(network_latency_exit);
MODULE_LICENSE("GPL v2");
