/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _IOSCHED_UTIL_H_
#define _IOSCHED_UTIL_H_

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
void blk_fill_rwbs_op(char *rwbs, unsigned int op);
int m_blk_mq_debugfs_rq_show(struct seq_file *m, struct request *rq);
#endif

#endif /*  _IOSCHED_UTIL_H_ */


