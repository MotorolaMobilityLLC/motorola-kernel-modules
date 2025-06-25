/*
 * Copyright (c) 2025 Motorola Inc.
 */

#define pr_fmt(fmt) "mdpf: " fmt

#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#include "mdpf_sysfs.h"
#include "msched_common.h"

#define MOTO_MDPF_PROC_DIR	"moto_mdpf"

struct proc_dir_entry *mdpf_root;

enum {
	OPT_STR_TYPE = 0,
	OPT_STR_PID,
	OPT_STR_VAL,
	OPT_STR_MAX = 3,
};

static ssize_t proc_mdpf_task_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos) {
	char buffer[MAX_PROC_SIZE];
	char *str, *token;
	char opt_str[OPT_STR_MAX][9] = {"0", "0", "0"};
	int cnt = 0;
	int pid = 0;
	int ux_type = 0;
	int err = 0;

	if (!is_enabled(UX_ENABLE_MDPF))
		return -EACCES;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	// at least 2 params.
	if (cnt < OPT_STR_MAX -1)
		return -EFAULT;

	// 2nd param must be pid.
	err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
	if (err || pid <= 0 || pid > PID_MAX_DEFAULT)
		return err;

	// write pid state
	if (!strncmp(opt_str[OPT_STR_TYPE], "w", 1) && cnt == OPT_STR_MAX) {
		err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ux_type);
		if (err || ux_type <= 0)
			return err;

		task_ux_type_set(pid, ux_type);

	// clear pid state
	} else if (!strncmp(opt_str[OPT_STR_TYPE], "c", 1) && cnt == OPT_STR_MAX) {
		err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ux_type);
		if (err || ux_type < 0)
			return err;

		task_ux_type_clear(pid, ux_type);
	} else {
		return -EFAULT;
	}

	return count;
}

static inline bool check_cred(struct task_struct *p) {
	const struct cred *cred, *tcred;
	bool ret = true;

	cred = current_cred();
	tcred = get_task_cred(p);
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid) &&
	    !ns_capable(tcred->user_ns, CAP_SYS_NICE)) {
		ret = false;
	}
	put_cred(tcred);
	return ret;
}

static ssize_t is_tgid_system_ui_store(struct file *filp,
		const char __user *ubuf,
		size_t count, loff_t *pos) {
	unsigned int val;
	char buf[MAX_PROC_SIZE];
	struct task_struct *p, *gp;
	char tgid_comm[TASK_COMM_LEN] = {0};

	if (!is_enabled(UX_ENABLE_MDPF))
		return -EACCES;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val) || val > PID_MAX_LIMIT)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(val);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);
	if (!check_cred(p)) {
		put_task_struct(p);
		rcu_read_unlock();
		return -EACCES;
	}

	gp = find_task_by_vpid(global_sysui_tgid);
	if (gp) {
		if(val == global_sysui_tgid) {
			put_task_struct(p);
			rcu_read_unlock();
			return count;
		}
	}

	strlcpy(tgid_comm, p->comm, TASK_COMM_LEN);
	put_task_struct(p);
	rcu_read_unlock();

	if (strstr(tgid_comm, "systemui")) {
		return count;
	}  else {
		return -ENOMSG;
	}
}

static ssize_t is_tgid_sf_store(struct file *filp,
		const char __user *ubuf,
		size_t count, loff_t *pos) {
	unsigned int val;
	char buf[MAX_PROC_SIZE];
	struct task_struct *p, *gp;
	char tgid_comm[TASK_COMM_LEN] = {0};

	if (!is_enabled(UX_ENABLE_MDPF))
		return -EACCES;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val) || val > PID_MAX_LIMIT)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(val);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);
	if (!check_cred(p)) {
		put_task_struct(p);
		rcu_read_unlock();
		return -EACCES;
	}

	gp = find_task_by_vpid(global_sf_tgid);
	if (gp) {
		if(val == global_sf_tgid) {
			put_task_struct(p);
			rcu_read_unlock();
			return count;
		}
	}

	strlcpy(tgid_comm, p->comm, TASK_COMM_LEN);
	put_task_struct(p);
	rcu_read_unlock();

	if (strstr(tgid_comm, "surfaceflinger")) {
		return count;
	}  else {
		return -ENOMSG;
	}
}

static const struct proc_ops proc_mdpf_task_fops = {
	.proc_write		= proc_mdpf_task_write,
};

static const struct proc_ops proc_is_tgid_sf_fops = {
	.proc_write		= is_tgid_sf_store,
};

static const struct proc_ops proc_is_tgid_system_ui_fops = {
	.proc_write		= is_tgid_system_ui_store,
};

int mdpf_proc_init(void) {
	struct proc_dir_entry *pe, *parent;

	parent = proc_mkdir(MOTO_MDPF_PROC_DIR, NULL);
	if (!parent) {
		sched_err("failed to create proc dir moto_mdpf\n");
		goto err_creat_mdpf_root;
	}
	mdpf_root = parent;

	pe = proc_create("mdpf_task", 0660, parent, &proc_mdpf_task_fops);
	if (!pe) {
		pr_debug("failed with creating file node mdpf_task\n");
		goto err_creat_mdpf_task;
	}

	pe = proc_create("is_tgid_system_ui", 0660, parent, &proc_is_tgid_system_ui_fops);
	if (!pe) {
		pr_debug("failed with creating file node is_tgid_system_ui\n");
		goto err_creat_mdpf_is_tgid_system_ui;
	}

	pe = proc_create("is_tgid_sf", 0660, parent, &proc_is_tgid_sf_fops);
	if (!pe) {
		pr_debug("failed with creating file node is_tgid_sf\n");
		goto err_creat_mdpf_is_tgid_sf;
	}
	return 0;

err_creat_mdpf_is_tgid_sf:
	remove_proc_entry("is_tgid_system_ui", mdpf_root);

err_creat_mdpf_is_tgid_system_ui:
	remove_proc_entry("mdpf_task", mdpf_root);

err_creat_mdpf_task:
	remove_proc_entry(MOTO_MDPF_PROC_DIR, NULL);

err_creat_mdpf_root:
	return -ENOENT;
}

void mdpf_proc_deinit(void)
{
	remove_proc_entry("is_tgid_sf", mdpf_root);
	remove_proc_entry("is_tgid_system_ui", mdpf_root);
	remove_proc_entry("mdpf_task", mdpf_root);
	remove_proc_entry(MOTO_MDPF_PROC_DIR, NULL);
}

