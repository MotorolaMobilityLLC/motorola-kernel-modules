// SPDX-License-Identifier: GPL-2.0

#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/blkdev.h>
#include <linux/debugfs.h>

#include <linux/blk-mq.h>
#include "iosched_util.h"

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
void blk_fill_rwbs_op(char *rwbs, unsigned int op)
{
	int i = 0;

    if (op & REQ_PREFLUSH)
        rwbs[i++] = 'F';

    switch (op & REQ_OP_MASK) {
    case REQ_OP_WRITE:
    case REQ_OP_WRITE_SAME:
        rwbs[i++] = 'W';
        break;
    case REQ_OP_DISCARD:
        rwbs[i++] = 'D';
        break;
    case REQ_OP_SECURE_ERASE:
        rwbs[i++] = 'D';
        rwbs[i++] = 'E';
        break;
    case REQ_OP_FLUSH:
        rwbs[i++] = 'F';
        break;
    case REQ_OP_READ:
        rwbs[i++] = 'R';
        break;
    default:
        rwbs[i++] = 'N';
    }

    if (op & REQ_FUA)
        rwbs[i++] = 'F';
    if (op & REQ_RAHEAD)
        rwbs[i++] = 'A';
    if (op & REQ_SYNC)
        rwbs[i++] = 'S';
    if (op & REQ_META)
        rwbs[i++] = 'M';

    rwbs[i] = '\0';

}


int m_blk_mq_debugfs_rq_show(struct seq_file *m, struct request *rq)
{
	//const struct blk_mq_ops *const mq_ops = rq->q->mq_ops;
	const unsigned int op = req_op(rq);
	//const char *op_str = blk_op_str(op);

	seq_printf(m, "%p {{.op=", rq);
    seq_printf(m, "%u", op);
#if 0
	if (strcmp(op_str, "UNKNOWN") == 0)
		seq_printf(m, "%u", op);
	else
		seq_printf(m, "%s", op_str);
	seq_puts(m, ", .cmd_flags=");
	blk_flags_show(m, rq->cmd_flags & ~REQ_OP_MASK, cmd_flag_name,
		       ARRAY_SIZE(cmd_flag_name));
	seq_puts(m, ", .rq_flags=");
	blk_flags_show(m, (__force unsigned int)rq->rq_flags, rqf_name,
		       ARRAY_SIZE(rqf_name));
	seq_printf(m, ", .state=%s", blk_mq_rq_state_name(blk_mq_rq_state(rq)));
	seq_printf(m, ", .tag=%d, .internal_tag=%d", rq->tag,
		   rq->internal_tag);
	if (mq_ops->show_rq)
		mq_ops->show_rq(m, rq);
#endif
	seq_puts(m, "}}\n");
	return 0;
}
EXPORT_SYMBOL_GPL(m_blk_mq_debugfs_rq_show);
#endif
