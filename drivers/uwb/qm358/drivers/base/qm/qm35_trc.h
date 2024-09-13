/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM qm35

#if !defined(__QM35_TRACE) || defined(TRACE_HEADER_MULTI_READ)
#define __QM35_TRACE

#if !defined(QM35_BYPASS_TRACES) && !defined(QM35_CORE_TRACES) && \
	!defined(QM35_NOTIFIER_TRACES)
/* Activate all traces if not included from tests. */
#define QM35_CORE_TRACES
#define QM35_BYPASS_TRACES
#define QM35_NOTIFIER_TRACES
#endif

#include <linux/tracepoint.h>
#include <linux/version.h>

#include "qm35.h"
#include "qm35_bypass.h"
#include "qm35_bypass_trc.h"
#include "qm35_notifier_trc.h"

/* clang-format off */
#define QM35_STATE_ENTRY(x) __field(enum qm35_state, x)
#define QM35_STATE_ASSIGN(x) __entry->x = x
#define QM35_STATE_PR_FMT(x) ", " #x " state: %s"
#define qm35_state_name(name) {  QM35_STATE_##name, #name }
#define QM35_STATE_SYMBOLS        \
	qm35_state_name(UNKNOWN), \
	qm35_state_name(READY),   \
	qm35_state_name(ACTIVE),  \
	qm35_state_name(ERROR)
/* clang-format on */
TRACE_DEFINE_ENUM(QM35_STATE_UNKNOWN);
TRACE_DEFINE_ENUM(QM35_STATE_READY);
TRACE_DEFINE_ENUM(QM35_STATE_ACTIVE);
TRACE_DEFINE_ENUM(QM35_STATE_ERROR);
#define QM35_STATE_PR_ARG(x) __print_symbolic(__entry->x, QM35_STATE_SYMBOLS)

#define QM_ENTRY __field(int, qm_id)
#define QM_ASSIGN __entry->qm_id = (qm ? qm->dev_id : -1)
#define QM_PR_FMT "hw#%d"
#define QM_PR_ARG __entry->qm_id

/* We don't want clang-format to modify the following events definition!
   Look at net/wireless/trace.h for the required format. */
/* clang-format off */

/**********************************
 *	Basic event classes	  *
 **********************************/

#if defined(QM35_CORE_TRACES) || defined(QM35_BYPASS_TRACES) || \
	defined(QM35_NOTIFIER_TRACES)

DECLARE_EVENT_CLASS(qm_evt_with_return,
	TP_PROTO(struct qm35 *qm, int ret),
	TP_ARGS(qm, ret),
	TP_STRUCT__entry(
		QM_ENTRY
		__field(int, ret)
	),
	TP_fast_assign(
		QM_ASSIGN;
		__entry->ret = ret;
	),
	TP_printk(QM_PR_FMT ", return: %d", QM_PR_ARG, __entry->ret)
);

#endif

/************************************************
 *	QM35 core device functions traces	*
 ************************************************/
#ifdef QM35_CORE_TRACES

DECLARE_EVENT_CLASS(qm_only_evt,
	TP_PROTO(struct qm35 *qm),
	TP_ARGS(qm),
	TP_STRUCT__entry(
		QM_ENTRY
	),
	TP_fast_assign(
		QM_ASSIGN;
	),
	TP_printk(QM_PR_FMT, QM_PR_ARG)
);

TRACE_EVENT(qm35_alloc_device,
	TP_PROTO(size_t priv_size),
	TP_ARGS(priv_size),
	TP_STRUCT__entry(
		__field(size_t, priv_size)
	),
	TP_fast_assign(
		__entry->priv_size = priv_size;
	),
	TP_printk("priv_size: %zu", __entry->priv_size)
);

DEFINE_EVENT(qm_only_evt, qm35_alloc_device_return,
	TP_PROTO(struct qm35 *qm),
	TP_ARGS(qm)
);

DEFINE_EVENT(qm_only_evt, qm35_register_device,
	TP_PROTO(struct qm35 *qm),
	TP_ARGS(qm)
);

DEFINE_EVENT(qm_evt_with_return, qm35_register_device_return,
	TP_PROTO(struct qm35 *qm, int ret),
	TP_ARGS(qm, ret)
);

DEFINE_EVENT(qm_only_evt, qm35_unregister_device,
	TP_PROTO(struct qm35 *qm),
	TP_ARGS(qm)
);

DEFINE_EVENT(qm_evt_with_return, qm35_unregister_device_return,
	TP_PROTO(struct qm35 *qm, int ret),
	TP_ARGS(qm, ret)
);

DEFINE_EVENT(qm_only_evt, qm35_free_device,
	TP_PROTO(struct qm35 *qm),
	TP_ARGS(qm)
);

TRACE_EVENT(qm35_state,
	TP_PROTO(struct qm35 *qm,
		 enum qm35_state old,
		 enum qm35_state new),
	TP_ARGS(qm, old, new),
	TP_STRUCT__entry(
		QM_ENTRY
		QM35_STATE_ENTRY(old)
		QM35_STATE_ENTRY(new)
	),
	TP_fast_assign(
		QM_ASSIGN;
		QM35_STATE_ASSIGN(old);
		QM35_STATE_ASSIGN(new);
	),
	TP_printk(QM_PR_FMT QM35_STATE_PR_FMT(old) QM35_STATE_PR_FMT(new),
		  QM_PR_ARG, QM35_STATE_PR_ARG(old), QM35_STATE_PR_ARG(new))
);

#endif /* QM35_CORE_TRACES */

/****************************************
 *	QM35 bypass functions traces	*
 ****************************************/
#ifdef QM35_BYPASS_TRACES

TRACE_EVENT(qm35_bypass_open,
	TP_PROTO(struct qm35 *qm, void *cb, void *priv_data),
	TP_ARGS(qm, cb, priv_data),
	TP_STRUCT__entry(
		QM_ENTRY
		__field(void *, cb)
		__field(void *, priv_data)
	),
	TP_fast_assign(
		QM_ASSIGN;
		__entry->cb = cb;
		__entry->priv_data = priv_data;
	),
	TP_printk(QM_PR_FMT ", callback: %ps, priv_data: %p",
		  QM_PR_ARG, __entry->cb, __entry->priv_data)
);

TRACE_EVENT(qm35_bypass_open_return,
	TP_PROTO(struct qm35 *qm, qm35_bypass_handle hnd),
	TP_ARGS(qm, hnd),
	TP_STRUCT__entry(
		QM_ENTRY
		__field(void *, hnd)
	),
	TP_fast_assign(
		QM_ASSIGN;
		__entry->hnd = hnd;
	),
	TP_printk(QM_PR_FMT ", channel: %pe",
		  QM_PR_ARG, __entry->hnd)
);

#define qmbh2qm(x) (container_of((x)->bypass, struct qm35, bypass_data))
#define QM_ASSIGN_BH(x)							\
	__entry->qm_id = (IS_ERR_OR_NULL(x) ? -1 : qmbh2qm(x)->dev_id);	\
	__entry->hnd = x
#define QM_ENTRY_BH				\
	QM_ENTRY				\
	__field(void *, hnd)

DECLARE_EVENT_CLASS(qmbh_only_evt,
	TP_PROTO(qm35_bypass_handle qmbh),
	TP_ARGS(qmbh),
	TP_STRUCT__entry(
		QM_ENTRY_BH
	),
	TP_fast_assign(
		QM_ASSIGN_BH(qmbh);
	),
	TP_printk(QM_PR_FMT ", channel: %pe", QM_PR_ARG, __entry->hnd)
);

DECLARE_EVENT_CLASS(qmbh_evt_with_return,
	TP_PROTO(qm35_bypass_handle qmbh, int ret),
	TP_ARGS(qmbh, ret),
	TP_STRUCT__entry(
		QM_ENTRY_BH
		__field(int, ret)
	),
	TP_fast_assign(
		QM_ASSIGN_BH(qmbh);
		__entry->ret = ret;
	),
	TP_printk(QM_PR_FMT ", channel: %pe, return: %d",
		  QM_PR_ARG, __entry->hnd, __entry->ret)
);

DEFINE_EVENT(qmbh_only_evt, qm35_bypass_close,
	TP_PROTO(qm35_bypass_handle qmbh),
	TP_ARGS(qmbh)
);

DEFINE_EVENT(qm_evt_with_return, qm35_bypass_close_return,
	TP_PROTO(struct qm35 *qm, int ret),
	TP_ARGS(qm, ret)
);

DEFINE_EVENT(qmbh_only_evt, qm35_bypass_send,
	TP_PROTO(qm35_bypass_handle qmbh),
	TP_ARGS(qmbh)
);

DEFINE_EVENT(qmbh_evt_with_return, qm35_bypass_send_return,
	TP_PROTO(qm35_bypass_handle qmbh, int ret),
	TP_ARGS(qmbh, ret)
);

DEFINE_EVENT(qmbh_only_evt, qm35_bypass_recv,
	TP_PROTO(qm35_bypass_handle qmbh),
	TP_ARGS(qmbh)
);

DEFINE_EVENT(qmbh_evt_with_return, qm35_bypass_recv_return,
	TP_PROTO(qm35_bypass_handle qmbh, int ret),
	TP_ARGS(qmbh, ret)
);

TRACE_EVENT(qm35_bypass_control,
	TP_PROTO(qm35_bypass_handle qmbh, enum qm35_bypass_actions action,
		 long *param),
	TP_ARGS(qmbh, action, param),
	TP_STRUCT__entry(
		QM_ENTRY_BH
		BYPASS_ACTION_ENTRY
		__field(long, param)
	),
	TP_fast_assign(
		QM_ASSIGN_BH(qmbh);
		BYPASS_ACTION_ASSIGN(action);
		__entry->param = param ? *param : 0;
	),
	TP_printk(QM_PR_FMT ", channel: %pe" BYPASS_ACTION_PR_FMT ", param: 0x%lx",
		  QM_PR_ARG, __entry->hnd, BYPASS_ACTION_PR_ARG, __entry->param)
);

DEFINE_EVENT(qmbh_evt_with_return, qm35_bypass_control_return,
	TP_PROTO(qm35_bypass_handle qmbh, int ret),
	TP_ARGS(qmbh, ret)
);

#endif /* QM35_BYPASS_TRACES */

/****************************************
 *	QM35 Notifier functions traces	*
 ****************************************/
#ifdef QM35_NOTIFIER_TRACES

DECLARE_EVENT_CLASS(qm_notifier_evt,
	TP_PROTO(struct notifier_block *nb),
	TP_ARGS(nb),
	TP_STRUCT__entry(
		__field(void *, cb)
	),
	TP_fast_assign(
		__entry->cb = nb->notifier_call;
	),
	TP_printk("callback: %ps", __entry->cb)
);

DEFINE_EVENT(qm_notifier_evt, qm35_register_notifier,
	TP_PROTO(struct notifier_block *nb),
	TP_ARGS(nb)
);

DEFINE_EVENT(qm_notifier_evt, qm35_unregister_notifier,
	TP_PROTO(struct notifier_block *nb),
	TP_ARGS(nb)
);

TRACE_EVENT(qm35_notify_one,
	TP_PROTO(struct notifier_block *nb, enum qm35_notifier_events evt, struct qm35 *qm),
	TP_ARGS(nb, evt, qm),
	TP_STRUCT__entry(
		__field(void *, cb)
		NOTIFIER_EVENT_ENTRY
		QM_ENTRY
	),
	TP_fast_assign(
		__entry->cb = nb->notifier_call;
		NOTIFIER_EVENT_ASSIGN(evt);
		QM_ASSIGN;
	),
	TP_printk(QM_PR_FMT ", callback: %ps" NOTIFIER_EVENT_PR_FMT,
		  QM_PR_ARG, __entry->cb, NOTIFIER_EVENT_PR_ARG)
);

TRACE_EVENT(qm35_notifier_notify,
	TP_PROTO(struct qm35 *qm, enum qm35_notifier_events evt),
	TP_ARGS(qm, evt),
	TP_STRUCT__entry(
		QM_ENTRY
		NOTIFIER_EVENT_ENTRY
	),
	TP_fast_assign(
		QM_ASSIGN;
		NOTIFIER_EVENT_ASSIGN(evt);
	),
	TP_printk(QM_PR_FMT NOTIFIER_EVENT_PR_FMT,
		  QM_PR_ARG, NOTIFIER_EVENT_PR_ARG)
);

DEFINE_EVENT(qm_evt_with_return, qm35_notifier_notify_return,
	TP_PROTO(struct qm35 *qm, int ret),
	TP_ARGS(qm, ret)
);

#endif /* QM35_NOTIFIER_TRACES */

/* clang-format on */
#endif /* !__QM35_TRACE || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
/* clang-format off */
#define TRACE_INCLUDE_PATH drivers/base/qm
/* clang-format on */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE qm35_trc
#include <trace/define_trace.h>
