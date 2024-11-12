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
#define TRACE_SYSTEM qm35_spi

#if !defined(__QM35_SPI_TRACE) || defined(TRACE_HEADER_MULTI_READ)
#define __QM35_SPI_TRACE

#if !defined(QM35_HSSPI_TRACES) && !defined(QM35_SPI_FW_TRACES) && \
	!defined(QM35_SPI_PM_TRACES) && !defined(QM35_SPI_TRANSPORT_TRACES)
/* Activate all traces if not included from tests. */
#define QM35_SPI_MAIN_TRACES
#define QM35_SPI_TRANSPORT_TRACES
#define QM35_SPI_FW_TRACES
#define QM35_SPI_PM_TRACES
#define QM35_HSSPI_TRACES
#define QM35_THREAD_TRACES
#define QM35_QMROM_TRACES
#endif

#include <linux/tracepoint.h>
#include <linux/version.h>

#include "qm35_spi.h"
#include "qm35_hsspi.h"
#include "qm35_transport_trc.h"

#define MAXNAME 32
#define QMS_ENTRY __field(int, qms_id)
#define QMS_ASSIGN __entry->qms_id = qms->base.dev_id
#define QMS_PR_FMT "hw#%d"
#define QMS_PR_ARG __entry->qms_id

#define DATA_ENTRY __dynamic_array(u8, data, len)
#define DATA_ASSIGN \
	memcpy(__get_dynamic_array(data), data, __get_dynamic_array_len(data))
#define DATA_PR_FMT ", data: %s"
#define DATA_PR_ARG \
	__print_hex(__get_dynamic_array(data), __get_dynamic_array_len(data))

/* QM35 HSSPI flags */

#define HSSPI_SOC_FLAGS_ENTRY __field(u8, socflags)
#define HSSPI_SOC_FLAGS_ASSIGN(x) entry->socflags = (x)
#define HSSPI_SOC_FLAGS_PR_FMT ", socflags: %s"
/* clang-format off */
#define hsspi_soc_name(name) { HSSPI_SOC_##name, #name }
#define HSSPI_SOC_FLAGS				\
	hsspi_soc_name(ERR),			\
	hsspi_soc_name(RDY),			\
	hsspi_soc_name(OA),			\
	hsspi_soc_name(ODW)
/* clang-format on */
TRACE_DEFINE_ENUM(HSSPI_SOC_ERR);
TRACE_DEFINE_ENUM(HSSPI_SOC_RDY);
TRACE_DEFINE_ENUM(HSSPI_SOC_OA);
TRACE_DEFINE_ENUM(HSSPI_SOC_ODW);
#define HSSPI_SOC_FLAGS_PR_ARG \
	__print_flags(__entry->socflags, "|", HSSPI_SOC_FLAGS)

#define HSSPI_HOST_FLAGS_ENTRY __field(u8, hostflags)
#define HSSPI_HOST_FLAGS_ASSIGN(x) entry->hostflags = (x)
#define HSSPI_HOST_FLAGS_PR_FMT ", hostflags: %s"
/* clang-format off */
#define hsspi_host_name(name) { HSSPI_HOST_##name, #name }
#define HSSPI_HOST_FLAGS			\
	hsspi_host_name(RD),			\
	hsspi_host_name(PRD),			\
	hsspi_host_name(WR)
/* clang-format on */
TRACE_DEFINE_ENUM(HSSPI_HOST_RD);
TRACE_DEFINE_ENUM(HSSPI_HOST_PRD);
TRACE_DEFINE_ENUM(HSSPI_HOST_WR);
#define HSSPI_HOST_FLAGS_PR_ARG \
	__print_flags(__entry->hostflags, "|", HSSPI_HOST_FLAGS)

/* We don't want clang-format to modify the following events definition!
   Look at net/wireless/trace.h for the required format. */
/* clang-format off */

/**********************************
 *	Basic event classes	  *
 **********************************/

DECLARE_EVENT_CLASS(qms_only_evt,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms),
	TP_STRUCT__entry(
		QMS_ENTRY
	),
	TP_fast_assign(
		QMS_ASSIGN;
	),
	TP_printk(QMS_PR_FMT, QMS_PR_ARG)
);

DECLARE_EVENT_CLASS(qms_evt_with_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, ret)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->ret = ret;
	),
	TP_printk(QMS_PR_FMT ", return: %d", QMS_PR_ARG, __entry->ret)
);

#ifdef QM35_SPI_MAIN_TRACES
DECLARE_EVENT_CLASS(qms_evt_with_fwname_force,
	TP_PROTO(struct qm35_spi *qms, const char *fw_name, bool force),
	TP_ARGS(qms, fw_name, force),
	TP_STRUCT__entry(
		QMS_ENTRY
		__string(fw_name, fw_name)
		__field(bool, force)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__assign_str(fw_name, fw_name)
		__entry->force = force;
	),
	TP_printk(QMS_PR_FMT ", fw_name: %s, force: %d", QMS_PR_ARG, __get_str(fw_name), __entry->force)
);
#endif

/************************************************
 *	qm35_spi transport functions traces	*
 ************************************************/
#ifdef QM35_SPI_TRANSPORT_TRACES

DEFINE_EVENT(qms_only_evt, qm35_spi_start,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_start_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_stop,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_stop_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_set_cs_level,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_set_cs_level_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_reset,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_reset_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_evt_with_fwname_force, qm35_spi_fw_update_single,
	TP_PROTO(struct qm35_spi *qms, const char *fw_name, bool force),
	TP_ARGS(qms, fw_name, force)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_fw_update_single_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_evt_with_fwname_force, qm35_spi_fw_update,
	TP_PROTO(struct qm35_spi *qms, const char *fw_name, bool force),
	TP_ARGS(qms, fw_name, force)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_fw_update_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qm35_spi_power,
	TP_PROTO(struct qm35_spi *qms, int on),
	TP_ARGS(qms, on),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, on)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->on = on;
	),
	TP_printk(QMS_PR_FMT ", power: %s", QMS_PR_ARG, __entry->on ? "ON" : "OFF")
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_power_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_send,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_send_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qm35_spi_send_awake,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, ret)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->ret = ret;
	),
	TP_printk(QMS_PR_FMT ", ret: %d", QMS_PR_ARG, __entry->ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_recv,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_recv_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_probe,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_spi_probe_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

#endif /* QM35_SPI_TRANSPORT_TRACES */

/****************************************
 *	qm35_spi_main functions traces	*
 ****************************************/
#ifdef QM35_SPI_MAIN_TRACES

DEFINE_EVENT(qms_only_evt, qm35_spi_awake_handle,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_isr,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_isr_return,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

#endif /* QM35_SPI_MAIN_TRACES */

/****************************************
 *	qm35_spi_fw functions traces	*
 ****************************************/
#ifdef QM35_SPI_FW_TRACES

DEFINE_EVENT(qms_only_evt, qm35_fw_upgrade_work,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_upgrade_work_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_fw_upgrade,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_upgrade_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_fw_get_device_id_work,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_get_device_id_work_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_fw_get_device_id,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_get_device_id_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qm35_fw_load,
	TP_PROTO(struct qm35_spi *qms, const char *fw_name),
	TP_ARGS(qms, fw_name),
	TP_STRUCT__entry(
		QMS_ENTRY
		__string(fw_name, fw_name)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__assign_str(fw_name, fw_name)
	),
	TP_printk(QMS_PR_FMT ", fw_name: %s", QMS_PR_ARG, __get_str(fw_name))
);


DEFINE_EVENT(qms_evt_with_return, qm35_fw_load_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_fw_free,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_free_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_fw_get_vendor_version,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_fw_get_vendor_version_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

#endif /* QM35_SPI_FW_TRACES */

/****************************************
 *	qm35_hsspi functions traces	*
 ****************************************/
#ifdef QM35_HSSPI_TRACES

DEFINE_EVENT(qms_only_evt, qm35_hsspi_wakeup,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

TRACE_EVENT(qm35_hsspi_soc_header,
	TP_PROTO(struct qm35_spi *qms, struct qm35_hsspi_header *hdr),
	TP_ARGS(qms, hdr),
	TP_STRUCT__entry(
		QMS_ENTRY
		HSSPI_SOC_FLAGS_ENTRY
		TRANSPORT_MSG_TYPE_ENTRY
		__field(u16, len)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		HSSPI_SOC_FLAGS_ASSIGN(hdr->flags);
		TRANSPORT_MSG_TYPE_ASSIGN(hdr->ul_value);
		__entry->len = hdr->length;
	),
	TP_printk(QMS_PR_FMT HSSPI_SOC_FLAGS_PR_FMT TRANSPORT_MSG_TYPE_PR_FMT
		  ", len: %d", QMS_PR_ARG, HSSPI_SOC_FLAGS_PR_ARG,
		  TRANSPORT_MSG_TYPE_PR_ARG, __entry->len)
);

TRACE_EVENT(qm35_hsspi_host_header,
	TP_PROTO(struct qm35_spi *qms, struct qm35_hsspi_header *hdr),
	TP_ARGS(qms, hdr),
	TP_STRUCT__entry(
		QMS_ENTRY
		HSSPI_HOST_FLAGS_ENTRY
		TRANSPORT_MSG_TYPE_ENTRY
		__field(u16, len)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		HSSPI_HOST_FLAGS_ASSIGN(hdr->flags);
		TRANSPORT_MSG_TYPE_ASSIGN(hdr->ul_value);
		__entry->len = hdr->length;
	),
	TP_printk(QMS_PR_FMT HSSPI_HOST_FLAGS_PR_FMT TRANSPORT_MSG_TYPE_PR_FMT
		  ", len: %d", QMS_PR_ARG, HSSPI_HOST_FLAGS_PR_ARG,
		  TRANSPORT_MSG_TYPE_PR_ARG, __entry->len)
);

TRACE_EVENT(qm35_hsspi_data,
	TP_PROTO(struct qm35_spi *qms, const void *data, int len),
	TP_ARGS(qms, data, len),
	TP_STRUCT__entry(
		QMS_ENTRY
		DATA_ENTRY
		__field(int, len)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		DATA_ASSIGN;
		__entry->len = len;
	),
	TP_printk(QMS_PR_FMT ", length: %d" DATA_PR_FMT,
		  QMS_PR_ARG, __entry->len, DATA_PR_ARG)
);

DEFINE_EVENT(qms_only_evt, qm35_hsspi_send,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_hsspi_send_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qm35_hsspi_recv,
	TP_PROTO(struct qm35_spi *qms, int len),
	TP_ARGS(qms, len),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, len)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->len = len;
	),
	TP_printk(QMS_PR_FMT ", buflen: %d", QMS_PR_ARG, __entry->len)
);

DEFINE_EVENT(qms_evt_with_return, qm35_hsspi_recv_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qm35_hsspi_wait_ready,
	TP_PROTO(struct qm35_spi *qms, int count),
	TP_ARGS(qms, count),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, count)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->count = count;
	),
	TP_printk(QMS_PR_FMT ", count: %d", QMS_PR_ARG, __entry->count)
);


TRACE_EVENT(qm35_hsspi_wait_ready_return,
	TP_PROTO(struct qm35_spi *qms, int ret, int count),
	    TP_ARGS(qms, ret, count),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, ret)
		__field(int, count)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->ret = ret;
		__entry->count = count;
	),
	TP_printk(QMS_PR_FMT ", return : %d, count: %d", QMS_PR_ARG,
		  __entry->ret, __entry->count)
);

#endif /* QM35_HSSPI_TRACES */

/****************************************
 *	qm35_thread functions traces	*
 ****************************************/
#ifdef QM35_THREAD_TRACES

DECLARE_EVENT_CLASS(qms_cmd_queue,
	TP_PROTO(struct qm35_spi *qms, struct qm35_work *cmd),
	TP_ARGS(qms, cmd),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(void *, cmd)
		__field(const void *, in)
		__field(void *, out)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->cmd = cmd->cmd;
		__entry->in = cmd->in;
		__entry->out = cmd->out;
	),
	TP_printk(QMS_PR_FMT ", cmd: %p, in: %p, out: %p",
		  QMS_PR_ARG, __entry->cmd, __entry->in, __entry->out)
);

DEFINE_EVENT(qms_cmd_queue, qm35_enqueue_cmd,
	TP_PROTO(struct qm35_spi *qms, struct qm35_work *cmd),
	TP_ARGS(qms, cmd)
);

DEFINE_EVENT(qms_cmd_queue, qm35_enqueue_cmd_exec,
	TP_PROTO(struct qm35_spi *qms, struct qm35_work *cmd),
	TP_ARGS(qms, cmd)
);

DEFINE_EVENT(qms_evt_with_return, qm35_enqueue_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_cmd_queue, qm35_process_cmd,
	TP_PROTO(struct qm35_spi *qms, struct qm35_work *cmd),
	TP_ARGS(qms, cmd)
);

DEFINE_EVENT(qms_evt_with_return, qm35_process_cmd_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_enqueue_irq,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

#endif /* QM35_THREAD_TRACES */

/****************************************
 *	qm35_spi PM functions traces	*
 ****************************************/
#ifdef QM35_SPI_PM_TRACES

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_setup,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_remove,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_start,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_stop,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_resume,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_spi_pm_idle,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qm35_pm_runtime_suspend,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_pm_runtime_suspend_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_pm_runtime_resume,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_pm_runtime_resume_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_pm_runtime_idle,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_pm_runtime_idle_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_system_suspend,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_system_suspend_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qm35_system_resume,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qm35_system_resume_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

#endif /* QM35_SPI_PM_TRACES */

/****************************************
 *	qmrom_wrapper functions traces	*
 ****************************************/
#ifdef QM35_QMROM_TRACES

TRACE_EVENT(qmrom_spi_set_freq,
	TP_PROTO(int freq),
	TP_ARGS(freq),
	TP_STRUCT__entry(
		__field(int, freq)
	),
	TP_fast_assign(
		__entry->freq = freq;
	),
	TP_printk("freq: %d", __entry->freq)
);

DEFINE_EVENT(qms_only_evt, qmrom_spi_transfer,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qmrom_spi_transfer_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

TRACE_EVENT(qmrom_spi_set_cs_level,
	TP_PROTO(struct qm35_spi *qms, int level),
	TP_ARGS(qms, level),
	TP_STRUCT__entry(
		QMS_ENTRY
		__field(int, level)
	),
	TP_fast_assign(
		QMS_ASSIGN;
		__entry->level = level;
	),
	TP_printk(QMS_PR_FMT ", level: %d", QMS_PR_ARG, __entry->level)
);

DEFINE_EVENT(qms_evt_with_return, qmrom_spi_set_cs_level_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qmrom_spi_reset_device,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_only_evt, qmrom_spi_wait_for_irq_line,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qmrom_spi_wait_for_irq_line_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

DEFINE_EVENT(qms_only_evt, qmrom_spi_read_irq_line,
	TP_PROTO(struct qm35_spi *qms),
	TP_ARGS(qms)
);

DEFINE_EVENT(qms_evt_with_return, qmrom_spi_read_irq_line_return,
	TP_PROTO(struct qm35_spi *qms, int ret),
	TP_ARGS(qms, ret)
);

#endif /* QM35_QMROM_TRACES */

/* clang-format on */
#endif /* !__QM35_SPI_TRACE || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
/* clang-format off */
#define TRACE_INCLUDE_PATH drivers/spi/qm
/* clang-format on */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE qm35_spi_trc
#include <trace/define_trace.h>
