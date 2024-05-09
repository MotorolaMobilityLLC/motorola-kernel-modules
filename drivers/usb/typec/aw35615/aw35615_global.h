/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AW35615_GLOBAL_H_
#define _AW35615_GLOBAL_H_

#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>

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

#define AW35615_CHIP_ID				(0x344f)
#define TICK_SCALE_TO_MS			(1)

struct aw35615_chip {
	struct i2c_client *client;
	struct tcpc_device *tcpc;
	struct tcpc_desc *tcpc_desc;
	AW_U16 chip_id;
	AW_BOOL	is_vbus_present;
	AW_U8 old_event;

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
	struct delayed_work bist_delay_work;
	AW_BOOL queued;

	/* Timers */
	struct hrtimer sm_timer;
	AW_U32 sink_timer;
	AW_U32 source_timer;
	AW_U32 source_end_timer;
	AW_U8 sink_reg_bist;
	AW_U8 source_reg_bist;

	/* Port Object */
	Port_t port;
	DevicePolicyPtr_t dpm;
};

extern struct aw35615_chip *g_chip;

struct aw35615_chip *aw35615_GetChip(void);         // Getter for the global chip structure
void aw35615_SetChip(struct aw35615_chip *newChip); // Setter for the global chip structure

#endif /* _AW35615_GLOBAL_H_ */

