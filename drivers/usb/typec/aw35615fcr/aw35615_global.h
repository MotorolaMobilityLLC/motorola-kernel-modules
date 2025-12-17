/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************/
#ifndef _AW35615_GLOBAL_H_
#define _AW35615_GLOBAL_H_

#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/alarmtimer.h>

#include "Port.h"
#include "modules/dpm.h"
#include "../../drivers/misc/mediatek/typec/tcpc/inc/tcpci.h"
#include "../../drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"

#ifdef AW_KERNEL_VER_OVER_4_19_1
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#ifdef AW_DEBUG
#include <linux/debugfs.h>
#endif // AW_DEBUG

#define TICK_SCALE_TO_MS			(1)
#define AW35615FSlaveAddr			(0x24)
#define AW35615_VENDOR_ID			(0x344f)
#define AW35615_CHIP_ID				(0x91)
#define AW35615P_CHIP_ID			(0x92)
#define AW35615F_CHIP_ID			(0x9a)

struct aw35615_chip {
	struct i2c_client *client;
	struct device *dev;
	struct tcpc_device *tcpc;
	struct tcpc_desc *tcpc_desc;
	struct pm_qos_request pm_gos_request;
	AW_U16 vendor_id;
	AW_BOOL	is_vbus_present;
	AW_U8 old_event;

	AW_BOOL wakelock_flag;
#ifdef AW_KERNEL_VER_OVER_4_19_1
	struct wakeup_source *aw35615_wakelock;          // Wake lock
#else
	struct wake_lock aw35615_wakelock;               // Wake lock
#endif

	struct semaphore suspend_lock;

	/* Internal config data */
	AW_S32 numRetriesI2C;

	/* GPIO */
	AW_S32 gpio_IntN; /* INT_N GPIO pin */
	AW_S32 gpio_IntN_irq; /* IRQ assigned to INT_N GPIO pin */

	/* Threads */
	struct work_struct sm_worker; /* Main state machine actions */
	struct workqueue_struct *highpri_wq;
	struct delayed_work init_delay_work;
	struct work_struct bist_work;
	struct work_struct lpd_check_work;
	AW_BOOL queued;

	/* Timers */
	struct hrtimer lpd_timer;
	struct hrtimer bist_timer;
	struct hrtimer sm_timer;
	struct alarm alarmtimer;
	AW_U32 sink_timer;
	AW_U32 source_timer;
	AW_U32 source_end_timer;
	AW_U8 sink_reg_bist;
	AW_U8 source_reg_bist;
	AW_U16 lpd_check_num_bak;
	AW_U16 lpd_check_num;
	AW_U16 toggle_check_num;
	AW_U32 lpd_check_timer;
	AW_BOOL lpd_check_enable;
	AW_BOOL lpd_notice;
	AW_BOOL lpd_wait_recovery;

	/* Port Object */
	Port_t port;

	/* chip status*/
	AW_BOOL shutdown;
};

extern struct aw35615_chip *g_chip[NUM_PORTS];

struct aw35615_chip *aw35615_GetChip(AW_U8 portId);         // Getter for the global chip structure
void aw35615_SetChip(struct aw35615_chip *newChip); // Setter for the global chip structure
extern int tcpci_notify_wd_status(struct tcpc_device *tcpc, bool water_detected); //waterproof send event

#endif /* _AW35615_GLOBAL_H_ */

