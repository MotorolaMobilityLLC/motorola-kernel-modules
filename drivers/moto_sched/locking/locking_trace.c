// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Moto. All rights reserved.
 *
 * This file contains the tracepoint implementations for the
 * custom locking vendor hooks. By defining CREATE_TRACE_POINTS
 * and including the trace header, we instantiate the trace events.
 */

#define CREATE_TRACE_POINTS
#include "locking_trace.h"

/*
 * Export the tracepoints to make them available to other modules.
 * Use _GPL for GPL-licensed modules. This is crucial for the linker
 * to find the tracepoint symbols at build time.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(rwsem_pi_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(rwsem_pi_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(lock_pi_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(lock_pi_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(set_user_nice);
EXPORT_TRACEPOINT_SYMBOL_GPL(locking_debug_trace);


