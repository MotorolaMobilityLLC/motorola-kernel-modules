/*
 * Copyright (c) 2024, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _PHONE_CASE_DETECTION_NOTIFY_H_
#define _PHONE_CASE_DETECTION_NOTIFY_H_

#include <linux/notifier.h>

/* The event of phone case being mounted */
#define PHONE_CASE_DETECTION_MOUNTED		0x01
/* The event of phone case being unmounted */
#define PHONE_CASE_DETECTION_UNMOUNTED		0x00

int phone_case_detection_register_client(struct notifier_block *nb);
int phone_case_detection_unregister_client(struct notifier_block *nb);
#endif

