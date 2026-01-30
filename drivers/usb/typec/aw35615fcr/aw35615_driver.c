// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 * Copyright (c) 2025 Motorola Mobility LLC.
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-09
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
/* Standard Linux includes */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/usb/typec.h>

/* Driver-specific includes */
#include "aw35615_global.h"
#include "platform_helpers.h"
#include "../../drivers/misc/mediatek/typec/tcpc/inc/tcpci.h"
#include "core.h"
#include "TypeC.h"
#include "vendor_info.h"

#ifdef AW_DEBUG
#include "dfs.h"
#endif // AW_DEBUG

#include "aw35615_driver.h"

#define AW35615_DRIVER_VERSION		"V1.7.4"
extern void tcpm_set_aw_ops(struct aw_tcpm_ops_ptr *pps_ops);
static struct aw_tcpm_ops_ptr aw_tcpm_ops;
/******************************************************************************
 * Driver functions
 ******************************************************************************/
#define AW35615_LPD_LONGER_THRESHOLD            (6)     /* the number of continuous liquid feedings */
#define AW35615_LPD_RECOVERY_DETE		(10000) /* time ms */
#define AW35615_LPD_RECOVERY_L_DETE		(30000) /* time ms */

int aw35615_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35615_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35615_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	AW_LOG("enter\n");
	return 0;
}

#ifdef AW_KERNEL_VER_OVER_6_6_0
static int aw35615_get_power_status(struct tcpc_device *tcpc)
{
	AW_LOG("enter\n");
	return 0;
}
#else
static int aw35615_get_power_status(struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	struct aw35615_chip *chip = tcpc_get_dev_data(tcpc);
	AW_LOG("enter\n");

	if (chip->port.Registers.Status.VBUSOK) {
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
	} else {
		*pwr_status |= TCPC_REG_POWER_STATUS_EXT_VSAFE0V;
	}
	return 0;
}
#endif

int aw35615_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	struct aw35615_chip *chip = tcpc_get_dev_data(tcpc);

	AW_LOG("enter\n");

	if (chip->port.sourceOrSink == SINK) {
		if (chip->port.CCTerm == CCTypeRd3p0) {
			if (chip->tcpc->typec_polarity) {
				chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_OPEN;
				chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_SNK_3_0;
			} else {
				chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_SNK_3_0;
				chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_OPEN;
			}
		} else if (chip->port.CCTerm == CCTypeRdUSB) {
			if (chip->tcpc->typec_polarity) {
				chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_OPEN;
				chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_SNK_DFT;
			} else {
				chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_SNK_DFT;
				chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_OPEN;
			}
		}
	} else if (chip->port.sourceOrSink == SOURCE) {
		if (chip->tcpc->typec_polarity) {
			chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_OPEN;
			chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_RD;
		} else {
			chip->tcpc->typec_remote_cc[0] = TYPEC_CC_VOLT_RD;
			chip->tcpc->typec_remote_cc[1] = TYPEC_CC_VOLT_OPEN;
		}
	} else {
		chip->tcpc->typec_remote_cc[0] = TYPEC_CC_DRP_TOGGLING;
		chip->tcpc->typec_remote_cc[1] = TYPEC_CC_DRP_TOGGLING;
	}

	return 0;
}

static int aw35615_set_cc(struct tcpc_device *tcpc, int pull)
{
	struct aw35615_chip *chip = tcpc_get_dev_data(tcpc);

	AW_LOG("enter pull=%d\n", pull);

	if (pull == TYPEC_CC_RP)
		core_set_source(&chip->port);
	else if (pull == TYPEC_CC_RD)
		core_set_sink(&chip->port);
	else if (pull == TYPEC_CC_DRP)
		core_set_drp(&chip->port);

	return 0;
}

static int aw35615_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_vconn(struct tcpc_device *tcpc, int enable)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_low_power_mode(struct tcpc_device *tcpc_dev, bool en, int pull)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_msg_header(struct tcpc_device *tcpc,
		uint8_t power_role, uint8_t data_role)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_protocol_reset(struct tcpc_device *tcpc_dev)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_get_message(struct tcpc_device *tcpc, uint32_t *payload,
		uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_transmit(struct tcpc_device *tcpc, enum tcpm_transmit_type type,
		uint16_t header, const uint32_t *data)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_bist_carrier_mode(struct tcpc_device *tcpc, uint8_t pattern)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_retransmit(struct tcpc_device *tcpc)
{
	AW_LOG("enter\n");
	return 0;
}

#ifndef AW_KERNEL_VER_OVER_6_6_0
int aw35615_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35615_is_low_power_mode(struct tcpc_device *tcpc_dev)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35615_set_watchdog(struct tcpc_device *tcpc_dev, bool en)
{
	AW_LOG("enter\n");
	return 0;
}
#endif

static struct tcpc_ops aw35615_tcpc_ops = {
	.init = aw35615_tcpc_init,
	.alert_status_clear = aw35615_alert_status_clear,
	.fault_status_clear = aw35615_fault_status_clear,
	.get_alert_mask = aw35615_get_alert_mask,
	.get_power_status = aw35615_get_power_status,
	.get_fault_status = aw35615_get_fault_status,
	.get_cc = aw35615_get_cc,
	.set_cc = aw35615_set_cc,
	.set_polarity = aw35615_set_polarity,
	.set_vconn = aw35615_set_vconn,
	.deinit = aw35615_tcpc_deinit,
	.set_low_power_mode = aw35615_set_low_power_mode,
	.set_msg_header = aw35615_set_msg_header,
	.set_rx_enable = aw35615_set_rx_enable,
	.protocol_reset = aw35615_protocol_reset,
	.get_message = aw35615_get_message,
	.transmit = aw35615_transmit,
	.set_bist_test_mode = aw35615_set_bist_test_mode,
	.set_bist_carrier_mode = aw35615_set_bist_carrier_mode,
	.retransmit = aw35615_retransmit,
#ifndef AW_KERNEL_VER_OVER_6_6_0
	.get_alert_status = aw35615_get_alert_status,
	.set_low_rp_duty = aw35615_set_low_rp_duty,
	.is_low_power_mode = aw35615_is_low_power_mode,
	.set_watchdog = aw35615_set_watchdog,
#endif
};

static const struct of_device_id aw35615_dt_match[] = {
	{ .compatible = AW35615_I2C_DEVICETREE_NAME },
	{},
};

static const struct i2c_device_id aw35615_i2c_device_id[] = {
	{ AW35615_I2C_DRIVER_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw35615_i2c_device_id);

static void aw35615_init_delay_work(struct work_struct *work)
{
	struct aw35615_chip *chip =
			container_of(work, struct aw35615_chip, init_delay_work.work);
	int ret;

	if (!chip)
		return;

	/*
	 * Initialize the core and enable the state machine
	 * Interrupt must be enabled before starting initialization
	 */
	aw_InitializeCore(chip);

	/* Enable interrupts after successful core/GPIO initialization */
	ret = aw_EnableInterrupts(chip);
	if (ret) {
		AW_LOG("Unable to enable interrupts! Error code: %d\n", ret);
		return;
	}

	if (platform_get_device_irq_state(chip->port.PortID)) {
		/* Plug-in typec power-on interrupt detection */
		if (!chip->queued) {
			chip->queued = AW_TRUE;
			AW_LOG("Plug-in typec power-on interrupt detection\n");
			queue_work(chip->highpri_wq, &chip->sm_worker);
		}
	}

	AW_LOG("Core is initialized!\n");
}

static void aw35615_bist_work(struct work_struct *work)
{
	struct aw35615_chip *chip =
			container_of(work, struct aw35615_chip, bist_work);
	AW_U8 data_buf[3];

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return;
	}

	if (chip->port.PolicyIsSource == 0) {
		DeviceRead(&chip->port, regInterruptb, 2, &data_buf[0]);
		DeviceRead(&chip->port, regInterrupt, 1, &data_buf[2]);
		AW_LOG("regInterruptb = 0x%x regStatus0 = 0x%x regInterrupt = 0x%x\n", data_buf[0], data_buf[1], data_buf[2]);
		if (data_buf[0] & 0x1) {
			hrtimer_start(&chip->bist_timer, ktime_set(chip->source_end_timer / 1000, chip->source_end_timer * 1000000), HRTIMER_MODE_REL);
			AW_LOG("go sink check\n");
		} else {
			AW_LOG("go sink set SDAC\n");
			chip->port.Registers.Slice.byte = chip->sink_reg_bist;
			DeviceWrite(&chip->port, regSlice, 1, &chip->port.Registers.Slice.byte);
		}
	}

	if (chip->port.PolicyIsSource == 1) {
		chip->port.SOURCE_Flag_end = 1;
		DeviceRead(&chip->port, regInterruptb, 2, &data_buf[0]);
		DeviceRead(&chip->port, regInterrupt, 1, &data_buf[2]);
		AW_LOG("regInterruptb = 0x%x regStatus0 = 0x%x regInterrupt = 0x%x\n", data_buf[0], data_buf[1], data_buf[2]);
		if (data_buf[0] & 0x1) {
			hrtimer_start(&chip->bist_timer, ktime_set(chip->source_end_timer / 1000, chip->source_end_timer * 1000000), HRTIMER_MODE_REL);
			AW_LOG("go source check\n");
		} else {
			AW_LOG("go source set SDAC\n");
			chip->port.Registers.Slice.byte = chip->source_reg_bist;
			DeviceWrite(&chip->port, regSlice, 1, &chip->port.Registers.Slice.byte);
		}
	}
}

static enum hrtimer_restart aw35615_bist_timer_func(struct hrtimer *p_hrtimer)
{
	struct aw35615_chip *chip = container_of(p_hrtimer, struct aw35615_chip, bist_timer);

	schedule_work(&chip->bist_work);
	return HRTIMER_NORESTART;
}

#ifdef AW_HAVE_LPD
static void aw35615_lpd_check_work(struct work_struct *work)
{
	struct aw35615_chip *chip =
			container_of(work, struct aw35615_chip, lpd_check_work);
	AW_U32 aw35615_recovery_time = AW35615_LPD_RECOVERY_DETE;

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return;
	}
	AW_LOG("lpd_check_num = %d, lpd_check_enable=%d, lpd_recovery_num=%d\n",
		chip->lpd_check_num, chip->lpd_check_enable, chip->lpd_recovery_num);
	if (chip->lpd_check_num == 0) {
		if (!chip->lpd_check_enable) {
			AW_LOG("notify ldp\n");
			chip->lpd_check_enable = AW_TRUE;
			chip->lpd_wait_recovery = AW_FALSE;
			core_set_sink(&chip->port);
			notify_observers(LPD_NOTICE_WATER, chip->port.PortID);
			if (chip->lpd_recovery_num < AW35615_LPD_LONGER_THRESHOLD) {
				chip->lpd_recovery_num ++;
			} else {
				aw35615_recovery_time = AW35615_LPD_RECOVERY_L_DETE;
			}
			hrtimer_start(&chip->lpd_timer, ktime_set(aw35615_recovery_time / 1000, 0), HRTIMER_MODE_REL);
		} else {
			chip->lpd_check_num = chip->lpd_check_num_bak;
			if (chip->port.ConnState == Unattached) {
				AW_LOG("cc unattached, cc recovery\n");
				chip->lpd_check_enable = AW_FALSE;
				core_set_try_snk(&chip->port);
				chip->lpd_wait_recovery = AW_FALSE;
				hrtimer_start(&chip->lpd_timer, ktime_set(chip->lpd_check_timer / 1000, 0), HRTIMER_MODE_REL);
			} else {
				AW_LOG("cc attached, waitting for lpd recovery\n");
#ifdef AW_HAVE_DRP
				chip->port.PortConfig.PortType = USBTypeC_DRP;
				chip->port.PortConfig.SnkPreferred = AW_TRUE;
				chip->port.PortConfig.SrcPreferred = AW_FALSE;
				chip->port.PortConfig.audioAccSupport = AW_TRUE;
				chip->port.PortConfig.poweredAccSupport = Type_C_Is_VCONN_Powered_Accessory;
#endif /* AW_HAVE_DRP */
				chip->lpd_wait_recovery = AW_TRUE;
			}
		}
	} else {
		chip->lpd_check_num = chip->lpd_check_num_bak;
		chip->lpd_check_enable = AW_TRUE;
		if (chip->port.ConnState == Unattached) {
			AW_LOG("lpd recovery\n");
			chip->lpd_wait_recovery = AW_FALSE;
			chip->lpd_recovery_num = 0;
			notify_observers(LPD_NOTICE_NOWATER, chip->port.PortID);
		} else if (chip->lpd_notice) {
			AW_LOG("cc attached, maintain lpd\n");
#ifdef AW_HAVE_DRP
			chip->port.PortConfig.PortType = USBTypeC_DRP;
			chip->port.PortConfig.SnkPreferred = AW_TRUE;
			chip->port.PortConfig.SrcPreferred = AW_FALSE;
			chip->port.PortConfig.audioAccSupport = AW_TRUE;
			chip->port.PortConfig.poweredAccSupport = Type_C_Is_VCONN_Powered_Accessory;
#endif /* AW_HAVE_DRP */
			chip->lpd_wait_recovery = AW_TRUE;
		}
	}
}

static enum hrtimer_restart aw35615_lpd_timer_func(struct hrtimer *p_hrtimer)
{
	struct aw35615_chip *chip = container_of(p_hrtimer, struct aw35615_chip, lpd_timer);

	AW_LOG("enter\n");
	schedule_work(&chip->lpd_check_work);
	return HRTIMER_NORESTART;
}
#endif /* AW_HAVE_LPD */

#ifdef AW_KERNEL_VER_OVER_6_6_0
static int aw35615_probe(struct i2c_client *client)
#else
static int aw35615_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct aw35615_chip *chip;
	struct i2c_adapter *adapter = client->adapter;
	struct tcpc_desc *typec_desc;
	int ret = 0;
	char buf[50] = {0};

	AW_LOG("enter\n");
	/* Make sure probe was called on a compatible device */
	if (!of_match_device(aw35615_dt_match, &client->dev)) {
		dev_err(&client->dev,
				"AWINIC  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}

	/* Allocate space for our chip structure (devm_* is managed by the device) */
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		/* dev_err(&client->dev, "%s allocate memory\n", __func__); */
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	/* Assign our struct as the client's driverdata */
	i2c_set_clientdata(client, chip);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"I2C/SMBus block functionality not supported!\n");
		return -ENODEV;
	}

	/* Verify that our device exists and that it's what we expect */
	if (!aw_IsDeviceValid(chip)) {
		dev_err(&client->dev,
				"AWINIC  %s - Error: Unable to communicate with device!\n",
				__func__);
		devm_kfree(&client->dev, chip);
		chip = NULL;
		return -EIO;
	}

#ifdef AW_KERNEL_VER_OVER_5_7_0
	cpu_latency_qos_add_request(&chip->pm_gos_request, PM_QOS_DEFAULT_VALUE);
#else
	pm_qos_add_request(&chip->pm_gos_request,
			PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#endif

	/* Initialize the chip's data members */
	aw_InitChipData(chip);

	/* Initialize the platform's GPIO pins and IRQ */
	ret = aw_parse_dts(chip);
	if (ret < 0) {
		dev_err(&client->dev,
				"AWINIC  %s - Error: Unable to initialize GPIO!\n", __func__);
		devm_kfree(&client->dev, chip);
		chip = NULL;
		return ret;
	}

	aw35615_SetChip(chip);

	typec_desc = devm_kzalloc(&client->dev, sizeof(*typec_desc), GFP_KERNEL);
	if (!typec_desc)
		return -ENOMEM;

	typec_desc->name = devm_kzalloc(&client->dev, 13, GFP_KERNEL);
	if (!typec_desc->name)
		return -ENOMEM;

	snprintf(buf, 50, "type_c_port%d", chip->port.PortID);
	strcpy((char *)typec_desc->name, buf);
	typec_desc->role_def = TYPEC_ROLE_TRY_SNK;
	typec_desc->rp_lvl = TYPEC_CC_RP_1_5;
	chip->tcpc_desc = typec_desc;

	dev_info(&client->dev, "%s: type_c_port%d, role=%d\n",
		__func__, chip->port.PortID, typec_desc->role_def);

	chip->tcpc = tcpc_device_register(&client->dev,
			typec_desc, &aw35615_tcpc_ops, chip);
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;

	if (chip->port.PortID == 0)
		aw35615_init_event_handler();

	/* Initialize semaphore*/
	sema_init(&chip->suspend_lock, 1);

	/* Initialize sysfs file accessors */
	aw_Sysfs_Init(chip);

	chip->shutdown = AW_FALSE;
#ifdef AW_DEBUG
	/* Initialize debugfs file accessors */
	aw_DFS_Init(chip);
	AW_LOG(" DebugFS nodes created!\n");
#endif // AW_DEBUG

#ifdef AW_HAVE_LPD
	INIT_WORK(&chip->lpd_check_work, aw35615_lpd_check_work);
	hrtimer_init(&chip->lpd_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->lpd_timer.function = aw35615_lpd_timer_func;
#endif /* AW_HAVE_LPD */

	INIT_WORK(&chip->bist_work, aw35615_bist_work);
	hrtimer_init(&chip->bist_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->bist_timer.function = aw35615_bist_timer_func;

	/* delay init */
	INIT_DELAYED_WORK(&chip->init_delay_work, aw35615_init_delay_work);
	schedule_delayed_work(&chip->init_delay_work, msecs_to_jiffies(3000));

	aw_tcpm_ops.get_pps_status = aw_get_pps_status;
	aw_tcpm_ops.pd_get_status = aw_pd_get_status;
	aw_tcpm_ops.request_pdo = aw_request_pdo;
	aw_tcpm_ops.request_apdo = aw_request_apdo;
	aw_tcpm_ops.get_power_cap = aw_get_power_cap;
	aw_tcpm_ops.pd_pe_ready = aw_pd_pe_ready;
	aw_tcpm_ops.get_source_apdo = aw_get_source_apdo;
	aw_tcpm_ops.pd_connected = aw_pd_connected;
	aw_tcpm_ops.pd_comm_capable = aw_pd_comm_capable;
	aw_tcpm_ops.request_dr_swap = aw_request_dr_swap;
	aw_tcpm_ops.request_pr_swap = aw_request_pr_swap;
	aw_tcpm_ops.get_source_cap_ext = aw_get_source_cap_ext;
	tcpm_set_aw_ops(&aw_tcpm_ops);

	AW_LOG(" AWINIC Driver loaded successfully!\n");

	return ret;
}

#ifdef AW_KERNEL_VER_OVER_6_1_0
static void aw35615_remove(struct i2c_client *client)
#else
static int aw35615_remove(struct i2c_client *client)
#endif
{
	struct aw35615_chip *chip = i2c_get_clientdata(client);

	AW_LOG(" Removing aw35615 device!\n");

#ifdef AW_DEBUG
	/* Remove debugfs file accessors */
	aw_DFS_Cleanup(chip);
	AW_LOG(" DebugFS nodes removed.\n");
#endif // AW_DEBUG

	aw_Sysfs_DeInit(chip);
	aw_GPIO_Cleanup(chip);
#ifdef AW_KERNEL_VER_OVER_5_7_0
	if (cpu_latency_qos_request_active(&chip->pm_gos_request))
		cpu_latency_qos_remove_request(&chip->pm_gos_request);
#else
	if (pm_qos_request_active(&chip->pm_gos_request))
		pm_qos_remove_request(&chip->pm_gos_request);
#endif

	AW_LOG(" AW35615 device removed from driver\n");
#ifndef AW_KERNEL_VER_OVER_6_1_0
	return 0;
#endif
}

static void aw35615_shutdown(struct i2c_client *client)
{
	AW_U8 reset = 0x01;
	AW_U8 data = 0x40;
	AW_U8 length = 0x01;
	AW_U8 reg_data;
	AW_BOOL ret = 0;
	struct aw35615_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		pr_err("AW35615 shutdown - Chip structure is NULL!\n");
		return;
	}

	if (chip->shutdown) {
		pr_err("AW35615 shutdown - Chip already shutdown !\n");
		return;
	}

	chip->shutdown = AW_TRUE;
	core_enable_typec(&chip->port, AW_FALSE);
	if (chip->gpio_IntN_irq)
		disable_irq(chip->gpio_IntN_irq);
	cancel_work_sync(&chip->sm_worker);
	alarm_cancel(&chip->alarmtimer);
	aw_GPIO_Cleanup(chip);
	SetTypeCState(&chip->port, Unattached);
	SetPEState(&chip->port, peDisabled);

	ret = DeviceWrite(&chip->port, regControl3, length, &data);
	if (ret < 0)
		pr_err("send hardreset failed, ret = %d\n", ret);

	//SetStateUnattached(&chip->port);

	/* keep the cc open status 20ms */
	mdelay(5);
	ret = DeviceWrite(&chip->port, regReset, length, &reset);
	if (ret < 0)
		pr_err("device Reset failed, ret = %d\n", ret);

	reg_data = 0x00;
	DeviceWrite(&chip->port, regPower, 1, &reg_data);
	reg_data = 0x03;
	DeviceWrite(&chip->port, regSwitches0, 1, &reg_data);

	AW_LOG("aw35615 device shutdown!\n");
}

static int aw35615_i2c_resume(struct device *dev)
{
	struct aw35615_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			up(&chip->suspend_lock);
	}

	return 0;
}

static int aw35615_i2c_suspend(struct device *dev)
{
	struct aw35615_chip *chip = NULL;
	struct i2c_client *client =  to_i2c_client(dev);
	int ret = 0;
	AW_U8 val[10];
	AW_U8 val1;

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			down(&chip->suspend_lock);
	}

	if (!chip) {
		pr_err("Chip structure is NULL!\n");
		return 0;
	}

	AW_LOG("port%d TC_ST=%d PE_ST=%d\n", chip->port.PortID, chip->port.ConnState, chip->port.PolicyState);
	ret = DeviceRead(&chip->port, regSwitches0, 10, val);
	if (ret < 0) {
		pr_err("error: read register fail\n");
		return 0;
	}
	AW_LOG("reg:[2:0x%02X 4:0x%02X 6:0x%02X 8:0x%02X a:0x%02X b:0x%02X ]\n",
			val[0], val[2], val[4], val[6], val[8], val[9]);

	ret = DeviceRead(&chip->port, regStatus0, 1, &val1);
	if (ret < 0) {
		pr_err("error: read register fail\n");
		return 0;
	}
	AW_LOG("reg:[40:0x%02X]\n", val1);

	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops aw35615_dev_pm_ops = {
	.suspend = aw35615_i2c_suspend,
	.resume  = aw35615_i2c_resume,
};
#endif

static struct i2c_driver aw35615_driver = {
	.driver = {
		.name = AW35615_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw35615_dt_match),
#ifdef CONFIG_PM
		.pm = &aw35615_dev_pm_ops,
#endif
	},
	.probe = aw35615_probe,
	.remove = aw35615_remove,
	.shutdown = aw35615_shutdown,
	.id_table = aw35615_i2c_device_id,
};

static int __init aw35615_init(void)
{
	AW_LOG("Start driver init version %s\n", AW35615_DRIVER_VERSION);

	return i2c_add_driver(&aw35615_driver);
}

static void __exit aw35615_exit(void)
{
	i2c_del_driver(&aw35615_driver);
	AW_LOG(" Driver deleted\n");
}

/*******************************************************************************
 * Driver macros
 ******************************************************************************/
module_init(aw35615_init);
module_exit(aw35615_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AWINIC AW35615 Driver");
MODULE_AUTHOR("AWINIC");
