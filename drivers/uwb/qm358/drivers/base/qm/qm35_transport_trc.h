/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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
#if !defined(QM35_TRANSPORT_TRC_H) || defined(TRACE_HEADER_MULTI_READ)
#define QM35_TRANSPORT_TRC_H

/* QM35 transport type */

#define TRANSPORT_MSG_TYPE_ENTRY __field(u8, type)
#define TRANSPORT_MSG_TYPE_ASSIGN(x) __entry->type = (x)
#define TRANSPORT_MSG_TYPE_PR_FMT ", type: %s"
/* clang-format off */
#define transport_msg_type_name(name) { QM35_TRANSPORT_MSG_##name, #name }
#define TRANSPORT_MSG_TYPE_SYMBOLS			\
	transport_msg_type_name(RESERVED_HSSPI),	\
	transport_msg_type_name(BOOTLOADER),		\
	transport_msg_type_name(UCI),			\
	transport_msg_type_name(COREDUMP),		\
	transport_msg_type_name(LOG),			\
	transport_msg_type_name(QTRACE),		\
	transport_msg_type_name(AWAKE)
/* clang-format on */
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_RESERVED_HSSPI);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_BOOTLOADER);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_UCI);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_COREDUMP);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_LOG);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_QTRACE);
TRACE_DEFINE_ENUM(QM35_TRANSPORT_MSG_AWAKE);
#define TRANSPORT_MSG_TYPE_PR_ARG \
	__print_symbolic(__entry->type, TRANSPORT_MSG_TYPE_SYMBOLS)

#endif /* !QM35_TRANSPORT_TRC_H || TRACE_HEADER_MULTI_READ */
