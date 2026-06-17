#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/plist.h>
#include <trace/hooks/power.h>

#ifndef S32_MAX
#define S32_MAX ((s32)0x7fffffff)
#endif
#define BOOST_FREQ_MAX S32_MAX // full power

#define MAX_TRACKED_REQS 32
#define MAX_CLUSTERS 3

static atomic_t is_boosting = ATOMIC_INIT(0);
static atomic_t is_self_updating = ATOMIC_INIT(0); //avoid hook self update
static struct delayed_work boost_timeout_work;
static s32 user_target_freq[MAX_CLUSTERS] = {BOOST_FREQ_MAX, BOOST_FREQ_MAX, BOOST_FREQ_MAX};
static int user_target_cpu[MAX_CLUSTERS] = {-1, -1, -1};
static unsigned long moto_boost_time = 10000;

// Tracked requests during active boost
struct tracked_qos_req {
    struct freq_qos_request *req;
    s32 original_value;
    s32 target_value;
};
static struct tracked_qos_req tracked_reqs[MAX_TRACKED_REQS];
static int tracked_count = 0;
static DEFINE_SPINLOCK(track_lock);
static void vendor_hook_freq_qos_add_request(void *data, struct freq_constraints *qos,
                                            struct freq_qos_request *req,
                                            enum freq_qos_req_type type, int value, int ret)
{
    int i;
    bool found = false;

    // We only care about tracking requests restricting maximum frequency
    if (ret || !req || type != FREQ_QOS_MAX || (atomic_read(&is_boosting)== 0)) return;

    spin_lock(&track_lock);
    // Find if it's already in the table or insert it in an empty slot
    for (i = 0; i < MAX_TRACKED_REQS; i++) {
        if (tracked_reqs[i].req == req) {
            tracked_reqs[i].original_value = value;
            found = true;
            break;
        }
    }

    if (!found) {
        for (i = 0; i < MAX_TRACKED_REQS; i++) {
            if (!tracked_reqs[i].req ) {
                tracked_reqs[i].req = req;
                tracked_reqs[i].original_value = value;
                found = true;
                pr_debug("moto_boost: Add REQ (%d) initial value %d.\n", i, value);
                break;
            }
        }
        if (!found) {
            pr_err("touch_boost: Add MAX_TRACKED_REQS (%d) reached during ADD! Track failed.\n", MAX_TRACKED_REQS);
        }
    }
    spin_unlock(&track_lock);
}
static void vendor_hook_freq_qos_remove_request(void *data, struct freq_qos_request *req)
{
    int i;
    if (!req || req->type != FREQ_QOS_MAX || (atomic_read(&is_boosting)== 0)) return;

    spin_lock(&track_lock);
    for (i = 0; i < MAX_TRACKED_REQS; i++) {
        if (tracked_reqs[i].req == req) {
            tracked_reqs[i].req = NULL;
            pr_debug("moto_boost: Remove REQS (%d)!\n", i);
            break;
        }
    }
    spin_unlock(&track_lock);
}

static void vendor_hook_freq_qos_update_request(void *data, struct freq_qos_request *req, int value)
{
    int i;
    bool found = false;

    if (!req || req->type != FREQ_QOS_MAX || (atomic_read(&is_boosting)== 0)) return;

    //let go if self updating
    if (atomic_read(&is_self_updating)) return;

    spin_lock(&track_lock);
     //Update the Qos request tracking record
    for (i = 0; i < MAX_TRACKED_REQS; i++) {
        if (tracked_reqs[i].req == req) {
            tracked_reqs[i].original_value = value; //remember the original request
            found = true;
            // Trick the kernel into returning 0 and skipping freq_qos_apply
            req->pnode.prio = value;
            pr_debug("moto_boost: Update REQ (%d) new value %d bypassed!\n", i, value);
            break;
        }
    }
    if (!found) {
        // slot full
        pr_err("moto_boost: Not tracked, NO need to update REQS!\n");
    }

    spin_unlock(&track_lock);
}
static void force_qos_update(bool enable_boost)
{
    if (enable_boost) {
        struct cpufreq_policy *policy;
        unsigned int cpu;
        int i;

        spin_lock(&track_lock);
        // Clear previous tracked list
        tracked_count = 0;
        for (i = 0; i < MAX_TRACKED_REQS; i++) {
            tracked_reqs[i].req = NULL;
            tracked_reqs[i].original_value = 0;
            tracked_reqs[i].target_value = 0;
        }

        // Query all CPU policies
        for_each_possible_cpu(cpu) {
            policy = cpufreq_cpu_get(cpu);
            if (!policy) continue;

            // Only process each cluster's policy once (cpu == policy->cpu)
            if (cpu == policy->cpu) {
                struct freq_qos_request *req;
                struct plist_head *head = &policy->constraints.max_freq.list;
                s32 current_target = BOOST_FREQ_MAX;
                int j;

                for (j = 0; j < MAX_CLUSTERS; j++) {
                    if (user_target_cpu[j] == cpu) {
                        current_target = user_target_freq[j];
                        break;
                    }
                }

                plist_for_each_entry(req, head, pnode) {
                    if (tracked_count >= MAX_TRACKED_REQS) {
                        pr_err("moto_boost: MAX_TRACKED_REQS (%d) reached!\n", MAX_TRACKED_REQS);
                        break;
                    }
                    tracked_reqs[tracked_count].req = req;
                    tracked_reqs[tracked_count].original_value = req->pnode.prio;
                    tracked_reqs[tracked_count].target_value = current_target;
                    tracked_count++;
                }
            }
            cpufreq_cpu_put(policy);
        }
        spin_unlock(&track_lock);
        atomic_set(&is_self_updating, 1);
        // Update the tracked requests to lift limits
        for (i = 0; i < MAX_TRACKED_REQS; i++) {
            if (tracked_reqs[i].req && tracked_reqs[i].target_value > 0) {
                freq_qos_update_request(tracked_reqs[i].req, tracked_reqs[i].target_value);
                pr_debug("moto_boost: boosting REQ (%d) target value(%d)!\n", i, tracked_reqs[i].target_value);
            }
        }
        atomic_set(&is_self_updating, 0);
    } else {
        int i;
        struct tracked_qos_req local_reqs[MAX_TRACKED_REQS];

        spin_lock(&track_lock);
        for (i = 0; i < MAX_TRACKED_REQS; i++) {
            local_reqs[i] = tracked_reqs[i];
            tracked_reqs[i].req = NULL;
            tracked_reqs[i].original_value = 0;
            tracked_reqs[i].target_value = 0;
        }
        tracked_count = 0;
        spin_unlock(&track_lock);

        // Restore the requests to their original value
        atomic_set(&is_self_updating, 1);
        for (i = 0; i < MAX_TRACKED_REQS; i++) {
            if (local_reqs[i].req) {
                // Force an update to apply
                if (local_reqs[i].req->pnode.prio == local_reqs[i].original_value)
                    local_reqs[i].req->pnode.prio -= 1;

                freq_qos_update_request(local_reqs[i].req, local_reqs[i].original_value);
                pr_debug("moto_boost: restore REQ (%d) to original value(%d)!\n", i, local_reqs[i].original_value);
            }
        }
        atomic_set(&is_self_updating, 0);
    }
}

static void boost_timeout_handler(struct work_struct *work)
{
    atomic_set(&is_boosting, 0);
    force_qos_update(false);
}

ssize_t moto_boost_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[128];
	char *str, *token;
	int parsed_cluster, parsed_freq;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);

	while ((token = strsep(&str, ",")) != NULL) {
		if (!*token) continue;
		if (kstrtoint(strstrip(token), 10, &parsed_cluster)) break;

		token = strsep(&str, ",");
		if (!token) break;
		if (kstrtoint(strstrip(token), 10, &parsed_freq)) break;

		if (parsed_cluster >= 0 && parsed_freq > 0) {
			int i;
			bool found = false;
			for (i = 0; i < MAX_CLUSTERS; i++) {
				if (user_target_cpu[i] == parsed_cluster) {
					user_target_freq[i] = parsed_freq;
					found = true;
					break;
				}
			}
			if (!found) {
				for (i = 0; i < MAX_CLUSTERS; i++) {
					if (user_target_cpu[i] == -1) {
						user_target_cpu[i] = parsed_cluster;
						user_target_freq[i] = parsed_freq;
						break;
					}
				}
			}
		}
	}

	atomic_set(&is_boosting, 1);

	mod_delayed_work(system_wq, &boost_timeout_work, msecs_to_jiffies(moto_boost_time));
    pr_info("moto_boost: target values (cpu%d:%d, cpu%d:%d, cpu%d:%d) received!\n",
            user_target_cpu[0], user_target_freq[0],
            user_target_cpu[1], user_target_freq[1],
            user_target_cpu[2], user_target_freq[2]);
	force_qos_update(true);

	return count;
}

ssize_t moto_boost_proc_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len;

	len = snprintf(buffer, sizeof(buffer), "%d\n", atomic_read(&is_boosting));
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

ssize_t moto_boost_time_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	int val;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	if (kstrtoint(strstrip(buffer), 10, &val) == 0 && val > 0) {
		moto_boost_time = val;
	}

	return count;
}

ssize_t moto_boost_time_proc_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len;

	len = snprintf(buffer, sizeof(buffer), "%lu\n", moto_boost_time);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

ssize_t moto_deboost_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	int val;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	if (kstrtoint(strstrip(buffer), 10, &val) == 0 && val > 0) {
		if (atomic_read(&is_boosting)) {
			atomic_set(&is_boosting, 0);
			if (cancel_delayed_work_sync(&boost_timeout_work))
				force_qos_update(false);
			pr_info("moto_boost: manual deboost triggered!\n");
		}
	}

	return count;
}

ssize_t moto_deboost_proc_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len;

	len = snprintf(buffer, sizeof(buffer), "0\n");
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

int moto_boost_init(void)
{
    int ret;
    INIT_DELAYED_WORK(&boost_timeout_work, boost_timeout_handler);

    ret = register_trace_android_vh_freq_qos_update_request(vendor_hook_freq_qos_update_request, NULL);
    if (ret) {
        pr_err("Failed to register VH freq_qos_update_request policy hook\n");
    }
    ret = register_trace_android_vh_freq_qos_remove_request(vendor_hook_freq_qos_remove_request, NULL);
    if (ret) {
        pr_err("Failed to register VH freq_qos_remove_request policy hook\n");
    }
    ret = register_trace_android_vh_freq_qos_add_request(vendor_hook_freq_qos_add_request, NULL);
    if (ret) {
        pr_err("Failed to register VH freq_qos_add_request policy hook\n");
    }
    pr_info("moto_boost driver initialized %d.\n", ret);
    return ret;
}
