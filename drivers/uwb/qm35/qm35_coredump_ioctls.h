/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __QM35_COREDUMP_IOCTLS_H___
#define __QM35_COREDUMP_IOCTLS_H___

#include <asm/ioctl.h>

#define COREDUMP_DEV_NAME "qm35-coredump"
#define COREDUMP_IOC_TYPE 'U'

#define QM35_COREDUMP_FORCE _IO(COREDUMP_IOC_TYPE, 1)
#define QM35_COREDUMP_GET_SIZE _IOR(COREDUMP_IOC_TYPE, 2, unsigned int)

#endif /* __QM35_COREDUMP_IOCTLS_H___ */
