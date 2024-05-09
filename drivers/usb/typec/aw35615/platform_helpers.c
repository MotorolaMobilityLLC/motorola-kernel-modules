// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/timekeeping.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "aw35615_global.h"

#ifdef AW_KERNEL_VER_OVER_4_19_1
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#else
#include <linux/wakelock.h>
#endif

#include "core.h"
#include "platform_helpers.h"

#include "PD_Types.h"
#include "TypeC_Types.h"
#include "sysfs_header.h"

/*****************************************************************************/
/*****************************************************************************/
/************************        GPIO Interface         **********************/
/*****************************************************************************/
/*****************************************************************************/
/*Name of the INT_N interrupt in the Device Tree */
const char *AW_DT_INTERRUPT_INTN =    "aw_interrupt_int_n";

/* Name of the Int_N GPIO pin in the Device Tree */
#define AW_DT_GPIO_INTN			"awinic,int_n"

/* Internal forward declarations */
static irqreturn_t _aw_isr_intn(int irq, void *dev_id);
static void work_function(struct work_struct *work);
static enum hrtimer_restart aw_sm_timer_callback(struct hrtimer *timer);

const AW_U8 aw35615_reg_access[AW35615_REG_MAX] = {
	[regDeviceID]   = (REG_RD_ACCESS),
	[regSwitches0]  = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regSwitches1]  = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regMeasure]    = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regSlice]      = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl0]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl1]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl2]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl3]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regMask]       = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regPower]      = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regReset]      = (REG_WR_ACCESS),
	[regOCPreg]     = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regMaska]      = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regMaskb]      = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl4]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl5]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl7]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regControl8]   = (REG_RD_ACCESS | REG_WR_ACCESS),
	[regVdl]        = (REG_RD_ACCESS),
	[regVdh]        = (REG_RD_ACCESS),
	[regStatus0a]   = (REG_RD_ACCESS),
	[regStatus1a]   = (REG_RD_ACCESS),
	[regInterrupta] = (REG_RD_ACCESS),
	[regInterruptb] = (REG_RD_ACCESS),
	[regStatus0]    = (REG_RD_ACCESS),
	[regStatus1]    = (REG_RD_ACCESS),
	[regInterrupt]  = (REG_RD_ACCESS),
	[regFIFO]       = (REG_RD_ACCESS | REG_WR_ACCESS),
};

AW_S32 aw_parse_dts(void)
{
	AW_S32 ret = 0;
	AW_U32 val;
	AW_U8 i = 0;
	struct device_node *node;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	/* Get our device tree node */
	node = chip->client->dev.of_node;

#ifdef AW_KERNEL_VER_OVER_4_19_1
	chip->aw35615_wakelock = wakeup_source_create("aw35615wakelock");
	wakeup_source_add(chip->aw35615_wakelock);
#else
	wake_lock_init(&chip->aw35615_wakelock, WAKE_LOCK_SUSPEND,
			"aw35615wakelock");
#endif

	chip->gpio_IntN = of_get_named_gpio(node, AW_DT_GPIO_INTN, 0);
	if (!gpio_is_valid(chip->gpio_IntN)) {
		dev_err(&chip->client->dev,
			"%s - Error: Could not get named GPIO for Int_N! Error code: %d\n",
			__func__, chip->gpio_IntN);
		return chip->gpio_IntN;
	}

	/* Request our GPIO to reserve it in the system - this should help */
	/* ensure we have exclusive access (not guaranteed) */

	ret = gpio_request(chip->gpio_IntN, AW_DT_GPIO_INTN);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s - Error: Could not request GPIO for Int_N! Error code: %d\n",
			__func__, ret);
		return ret;
	}

	ret = gpio_direction_input(chip->gpio_IntN);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s Error:Could not set GPIO direction to input for Int_N!Error code:%d\n",
			__func__, ret);
		return ret;
	}
	AW_LOG(" INT_N GPIO initialized as pin '%d'\n", chip->gpio_IntN);

	ret = of_property_read_u32(node, "aw35615,snk_pdo_size",
			(u32 *)&chip->port.snk_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get sink pdo size fail\n", __func__);

	ret = of_property_read_u32_array(node, "aw35615,snk_pdo_vol",
			(u32 *)chip->port.snk_pdo_vol, chip->port.snk_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get sink pdo vol fail\n", __func__);

	ret = of_property_read_u32_array(node, "aw35615,snk_pdo_cur",
			(u32 *)chip->port.snk_pdo_cur, chip->port.snk_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get sink pdo cur fail\n", __func__);

	for (i = 0; i < chip->port.snk_pdo_size; i++) {
		AW_LOG("snk_pdo_vol[%d] = %d; snk_pdo_cur[%d] = %d\n", i,
				chip->port.snk_pdo_vol[i], i, chip->port.snk_pdo_cur[i]);
	}

	ret = of_property_read_u32(node, "aw35615,src_pdo_size",
			(u32 *)&chip->port.src_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get source pdo size fail\n", __func__);

	ret = of_property_read_u32_array(node, "aw35615,src_pdo_vol",
			(u32 *)chip->port.src_pdo_vol, chip->port.src_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get source pdo vol fail\n", __func__);

	ret = of_property_read_u32_array(node, "aw35615,src_pdo_cur",
			(u32 *)chip->port.src_pdo_cur, chip->port.src_pdo_size);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s get source pdo cur fail\n", __func__);

	for (i = 0; i < chip->port.src_pdo_size; i++) {
		AW_LOG("src_pdo_vol[%d] = %d; src_pdo_cur[%d] = %d\n", i,
				chip->port.src_pdo_vol[i], i, chip->port.src_pdo_cur[i]);
	}

	ret = of_property_read_u32(node, "aw35615,snk_tog_time", &val);
	if (ret < 0) {
		chip->port.snk_tog_time = 0;
		dev_err(&chip->client->dev, "%s: no aw35615,snk_tog_time\n", __func__);
	} else {
		chip->port.snk_tog_time = (AW_U8)val;
		AW_LOG("%s:aw35615,snk_tog_time, val = %d\n", __func__, val);
	}

	ret = of_property_read_u32(node, "aw35615,src_tog_time", &val);
	if (ret < 0) {
		chip->port.src_tog_time = 0;
		dev_err(&chip->client->dev, "%s: no aw35615,src_tog_time\n", __func__);
	} else {
		chip->port.src_tog_time = (AW_U8)val;
		AW_LOG("%s:aw35615,snk_tog_time, val = %d\n", __func__, val);
	}

	return 0;
}

AW_BOOL aw_GPIO_Get_IntN(void)
{
	AW_S32 ret = 0;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}

	/* If your system routes GPIO calls through a queue of some kind, then*/
	/* it may need to be able to sleep. If so, this call must be used.*/
	if (gpio_cansleep(chip->gpio_IntN))
		ret = !gpio_get_value_cansleep(chip->gpio_IntN);
	else
		ret = !gpio_get_value(chip->gpio_IntN); /* Int_N is active low */

	return (ret != 0);
}

void aw_GPIO_Cleanup(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	/* -1 indicates that we don't have an IRQ to free */
	if (gpio_is_valid(chip->gpio_IntN) && chip->gpio_IntN_irq != -1)
		devm_free_irq(&chip->client->dev, chip->gpio_IntN_irq, chip);

#ifdef AW_KERNEL_VER_OVER_4_19_1
	wakeup_source_remove(chip->aw35615_wakelock);
	wakeup_source_destroy(chip->aw35615_wakelock);
#else
	wake_lock_destroy(&chip->aw35615_wakelock);
#endif

	if (gpio_is_valid(chip->gpio_IntN))
		gpio_free(chip->gpio_IntN);
}

/*****************************************************************************/
/*****************************************************************************/
/************************         I2C Interface         **********************/
/*****************************************************************************/
/*****************************************************************************/
int DeviceWrite(Port_t *port, AW_U8 reg_addr, AW_U8 length, AW_U8 *buf)
{
	struct aw35615_chip *chip = container_of(port, struct aw35615_chip, port);
	int ret = 0;
	unsigned char *wdbuf;

	wdbuf = kmalloc(length+1, GFP_KERNEL);
	if (wdbuf == NULL) {
		/* pr_err("%s: can not allocate memory\n", __func__); */
		return -ENOMEM;
	}

	wdbuf[0] = reg_addr;
	memcpy(&wdbuf[1], buf, length);

	ret = i2c_master_send(chip->client, wdbuf, length+1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(wdbuf);

	return ret;
}

int DeviceRead(Port_t *port, AW_U8 reg_addr, AW_U8 length, AW_U8 *buf)
{
	struct aw35615_chip *chip = container_of(port, struct aw35615_chip, port);
	int ret = 0;
	unsigned char *rdbuf = NULL;

	struct i2c_msg msgs[] = {
		{
			.addr	= chip->client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg_addr,
		},
		{
			.addr	= chip->client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
		},
	};

	rdbuf = kmalloc(length, GFP_KERNEL);
	if (rdbuf == NULL) {
		/* pr_err("%s: can not allocate memory\n", __func__); */
		return  -ENOMEM;
	}

	msgs[1].buf = rdbuf;

	ret = i2c_transfer(chip->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	if (buf != NULL)
		memcpy(buf, rdbuf, length);
	kfree(rdbuf);

	return ret;
}

/*****************************************************************************/
/*****************************************************************************/
/************************        Timer Interface        **********************/
/*****************************************************************************/
/*****************************************************************************/

/*******************************************************************************
 * Function:        _aw_TimerHandler
 * Input:           timer: hrtimer struct to be handled
 * Return:          HRTIMER_RESTART to restart the timer, or HRTIMER_NORESTART otherwise
 * Description:     Ticks state machine timer counters and rearms itself
 ********************************************************************************/

/* Get the max value that we can delay in 10us increments at compile time */
static const AW_U32 MAX_DELAY_10US = (UINT_MAX / 10);
void aw_Delay10us(AW_U32 delay10us)
{
	AW_U32 us = 0;

	if (delay10us > MAX_DELAY_10US) {
		pr_err("%s - Error: Delay of '%u' is too long! Must be less than '%u'.\n",
			__func__, delay10us, MAX_DELAY_10US);
		return;
	}

	/* Convert to microseconds (us) */
	us = delay10us * 10;
	/* Best practice is to use udelay() for < ~10us times */
	if (us <= 10) {
		// BLOCKING delay for < 10us
		udelay(us);
	} else if (us < 20000) {/* Best practice is to use usleep_range() for 10us-20ms */
		/* TODO - optimize this range, probably per-platform */
		/* Non-blocking sleep for at least the requested time, */
		/* and up to the requested time + 10% */
		usleep_range(us, us + (us / 10));
	} else {
	// Best practice is to use msleep() for > 20ms
		// Convert to ms. Non-blocking, low-precision sleep
		msleep(us / 1000);
	}
}

void platform_delay_10us(AW_U32 delayCount)
{
	aw_Delay10us(delayCount);
}

AW_BOOL platform_get_device_irq_state(AW_U8 port)
{
	return aw_GPIO_Get_IntN() ? AW_TRUE : AW_FALSE;
}

void platform_set_pps_voltage(AW_U8 port, AW_U32 mv)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if ((mv == SET_VOUT_0000MV) && (chip->is_vbus_present == AW_TRUE)) {
		tcpci_source_vbus(chip->tcpc,
				TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
		chip->is_vbus_present = AW_FALSE;
		AW_LOG("disable vbus\n");
	} else if ((mv > SET_VOUT_0000MV) && (chip->is_vbus_present == AW_FALSE)) {
		tcpci_source_vbus(chip->tcpc,
				TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 1000);
		chip->is_vbus_present = AW_TRUE;
		AW_LOG("enable vbus\n");
	}
}

AW_U16 platform_get_pps_voltage(AW_U8 port)
{
	return 0;
}

void platform_set_pps_current(AW_U8 port, AW_U32 ma)
{
}

#ifdef AW_HAVE_DP
AW_BOOL platform_dp_enable_pins(AW_BOOL enable, AW_U32 config)
{
	return 0;
}

void platform_dp_status_update(AW_U32 status)
{
}
#endif

/*******************************************************************************
 * Function:        aw_Sysfs_Handle_Read
 * Input:           output: Buffer to which the output will be written
 * Return:          Number of chars written to output
 * Description:     Reading this file will output the most recently saved hostcomm output buffer
 * NOTE: Not used right now - separate functions for state logs.
 ********************************************************************************/
// Arbitrary temp buffer for parsing out driver data to sysfs
#define AW_MAX_BUF_SIZE 256

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	AW_U8 val = 0;
	AW_U8 i = 0;
	struct aw35615_chip *chip = aw35615_GetChip();

	for (i = 0; i < AW35615_REG_MAX; i++) {
		if (!(aw35615_reg_access[i] & REG_RD_ACCESS))
			continue;
		ret = DeviceRead(&chip->port, i, 1, &val);
		if (ret < 0) {
			pr_err("error: read register fail\n");
			return len;
		}
		len += snprintf(buf + len, PAGE_SIZE - len,
			"reg[0x%02X]=0x%02X\n", i, val);
	}

	return len;
}

static ssize_t reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	AW_U32 databuf[2] = {0};
	AW_U8 reg_addr = 0;
	AW_U8 reg_data = 0;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		reg_addr = (AW_U8) databuf[0];
		reg_data = (AW_U8) databuf[1];
		if (aw35615_reg_access[reg_addr] & REG_WR_ACCESS) {
			ret = DeviceWrite(&chip->port, reg_addr, 1, &reg_data);
			if (ret < 0)
				pr_err("error: write register fail\n");
		} else {
			AW_LOG("no permission to write 0x%04X register\n", reg_addr);
		}
	}

	return count;
}

/* Reinitialize the AW35615 */
static ssize_t reinitialize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (chip == NULL)
		return sprintf(buf, "AW35615 Error: Internal chip structure pointer is NULL!\n");

	/* Make sure that we are doing this in a thread-safe manner */
	/* Waits for current IRQ handler to return, then disables it */
	disable_irq(chip->gpio_IntN_irq);

	core_initialize(&chip->port);
	pr_debug("AWINIC  %s - Core is initialized!\n", __func__);
	core_enable_typec(&chip->port, AW_TRUE);
	pr_debug("AWINIC  %s - Type-C State Machine is enabled!\n", __func__);

	enable_irq(chip->gpio_IntN_irq);

	return sprintf(buf, "AW35615 Reinitialized!\n");
}

static ssize_t typec_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (chip->port.ConnState < NUM_TYPEC_STATES)
		return sprintf(buf, "%d %s\n", chip->port.ConnState,
				TYPEC_STATE_TBL[chip->port.ConnState]);
	return 0;
}

static ssize_t pe_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (chip->port.PolicyState < NUM_PE_STATES)
		return sprintf(buf, "%d %s\n", chip->port.PolicyState,
				PE_STATE_TBL[chip->port.PolicyState]);

	return 0;
}

static ssize_t port_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", PORT_TYPE_TBL[chip->port.PortConfig.PortType]);
}

static ssize_t cc_term_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", CC_TERM_TBL[chip->port.CCTerm % NUM_CC_TERMS]);
}

static ssize_t vconn_term_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", CC_TERM_TBL[chip->port.VCONNTerm % NUM_CC_TERMS]);
}

static ssize_t cc_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n",
			(chip->port.CCPin == CC1) ? "CC1" :
			(chip->port.CCPin == CC2) ? "CC2" : "None");
}

static ssize_t pwr_role_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", chip->port.PolicyIsSource == AW_TRUE ? "Source" : "Sink");
}

static ssize_t data_role_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", chip->port.PolicyIsDFP == AW_TRUE ? "DFP" : "UFP");
}

static ssize_t vconn_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.IsVCONNSource);
}

static ssize_t pe_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.USBPDEnabled);
}

static ssize_t
pe_enabled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	int enabled;

	if (sscanf(buf, "%1d", &enabled) == 1)
		chip->port.USBPDEnabled = enabled;
	return count;
}

static ssize_t pd_specrev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;

	len = sprintf(buf, "3\n");

	return len;
}

static ssize_t src_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.SourceCurrent);
}

static ssize_t
src_current_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	int src_cur;

	if (sscanf(buf, "%3d", &src_cur) == 1)
		core_set_advertised_current(&chip->port, src_cur);

	return count;
}

static ssize_t sink_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%s\n", SINK_GET_Rp_TBL[chip->port.SinkCurrent]);
}

static ssize_t acc_support_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.PortConfig.audioAccSupport);
}

static ssize_t acc_support_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (buf[0] == '1')
		chip->port.PortConfig.audioAccSupport = AW_TRUE;
	else if (buf[0] == '0')
		chip->port.PortConfig.audioAccSupport = AW_FALSE;

	return count;
}

static ssize_t src_pref_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.PortConfig.SrcPreferred);
}

static ssize_t
src_pref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (buf[0] == '1') {
		chip->port.PortConfig.SrcPreferred = AW_TRUE;
		chip->port.PortConfig.SnkPreferred = AW_FALSE;
	} else if (buf[0] == '0')
		chip->port.PortConfig.SrcPreferred = AW_FALSE;

	return count;
}

static ssize_t sink_pref_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "%d\n", chip->port.PortConfig.SnkPreferred);
}

static ssize_t
sink_pref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (buf[0] == '1') {
		chip->port.PortConfig.SnkPreferred = AW_TRUE;
		chip->port.PortConfig.SrcPreferred = AW_FALSE;
	} else if (buf[0] == '0')
		chip->port.PortConfig.SnkPreferred = AW_FALSE;

	return count;
}

static ssize_t pdo_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	ssize_t len = 0;
	int i;

	for (i = 0; i < chip->port.SrcCapsHeaderReceived.NumDataObjects; i++) {
		if (chip->port.SrcCapsReceived[i].PDO.SupplyType == pdoTypeFixed) {
			len +=
			snprintf(buf + len, PAGE_SIZE - len, "Fixed :%uV, %uA",
				chip->port.SrcCapsReceived[i].FPDOSupply.Voltage * 50 / 1000,
				chip->port.SrcCapsReceived[i].FPDOSupply.MaxCurrent * 10 / 1000);
		} else if (chip->port.SrcCapsReceived[i].PDO.SupplyType == pdoTypeAugmented) {
			len +=
			snprintf(buf + len, PAGE_SIZE - len, "Pps   :%u.%uV ~ %uV, %uA",
				chip->port.SrcCapsReceived[i].PPSAPDO.MinVoltage * 100 / 1000,
				chip->port.SrcCapsReceived[i].PPSAPDO.MinVoltage * 100 % 1000 / 100,
				chip->port.SrcCapsReceived[i].PPSAPDO.MaxVoltage * 100 / 1000,
				chip->port.SrcCapsReceived[i].PPSAPDO.MaxCurrent * 50 / 1000);
		}

		if (chip->port.SinkRequest.FVRDO.ObjectPosition == (i + 1))
			len += snprintf(buf + len, PAGE_SIZE - len, " <-\n");
		else
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	return len;
}

static ssize_t
pdo_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int obj_num, obj_cur;

	if (sscanf(buf, "%d %d", &obj_num, &obj_cur) == 2) {
		if (obj_num <= 5)
			aw_set_pdo((AW_U16)obj_num, (AW_U16)obj_cur);
		else
			aw_set_apdo((AW_U16)obj_num, (AW_U16)obj_cur);
	}

	return count;
}

static ssize_t bist_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	return sprintf(buf, "snk = %d src = %d src_end = %d sink_bist_reg = %x snk_reg = %x src_reg = %x\n",
		(int)chip->sink_timer, (int)chip->source_timer, (int)chip->source_end_timer,
		(int)chip->port.sink_bist_reg, (int)chip->sink_reg_bist, (int)chip->source_reg_bist);

}

static ssize_t
bist_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	int databuf[6] = {0};

	if (sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5]) == 6) {
		chip->sink_timer = (AW_U32)databuf[0];
		chip->source_timer = (AW_U32)databuf[1];
		chip->source_end_timer = (AW_U32)databuf[2];
		chip->port.sink_bist_reg = (AW_U8)databuf[3];
		chip->sink_reg_bist = (AW_U8)databuf[4];
		chip->source_reg_bist = (AW_U8)databuf[5];
	}

	return count;
}
/* Define our device attributes to export them to sysfs */
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RO(reinitialize);
static DEVICE_ATTR_RO(typec_state);
static DEVICE_ATTR_RO(port_type);
static DEVICE_ATTR_RO(cc_term);
static DEVICE_ATTR_RO(vconn_term);
static DEVICE_ATTR_RO(cc_pin);
static DEVICE_ATTR_RO(pe_state);
static DEVICE_ATTR_RW(pe_enabled);
static DEVICE_ATTR_RO(pwr_role);
static DEVICE_ATTR_RO(data_role);
static DEVICE_ATTR_RO(pd_specrev);
static DEVICE_ATTR_RO(vconn_source);
static DEVICE_ATTR_RW(src_current);
static DEVICE_ATTR_RO(sink_current);
static DEVICE_ATTR_RW(acc_support);
static DEVICE_ATTR_RW(src_pref);
static DEVICE_ATTR_RW(sink_pref);
static DEVICE_ATTR_RW(pdo_set);
static DEVICE_ATTR_RW(bist_test);

static struct attribute *aw35615_sysfs_attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_reinitialize.attr,
	&dev_attr_typec_state.attr,
	&dev_attr_port_type.attr,
	&dev_attr_cc_term.attr,
	&dev_attr_vconn_term.attr,
	&dev_attr_cc_pin.attr,
	&dev_attr_pe_state.attr,
	&dev_attr_pe_enabled.attr,
	&dev_attr_pwr_role.attr,
	&dev_attr_data_role.attr,
	&dev_attr_pd_specrev.attr,
	&dev_attr_vconn_source.attr,
	&dev_attr_src_current.attr,
	&dev_attr_sink_current.attr,
	&dev_attr_acc_support.attr,
	&dev_attr_src_pref.attr,
	&dev_attr_sink_pref.attr,
	&dev_attr_pdo_set.attr,
	&dev_attr_bist_test.attr,
	NULL
};

static struct attribute_group aw35615_sysfs_attr_grp = {
	.name = "control",
	.attrs = aw35615_sysfs_attrs,
};

void aw_Sysfs_Init(void)
{
	AW_S32 ret = 0;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (chip == NULL) {
		pr_err("%s - Chip structure is null!\n", __func__);
		return;
	}

	/* create attribute group for accessing the AW35615 */
	ret = sysfs_create_group(&chip->client->dev.kobj, &aw35615_sysfs_attr_grp);
	if (ret)
		pr_err("AWINIC %s - Error creating sysfs attributes!\n", __func__);
}

/*****************************************************************************/
/*****************************************************************************/
/************************        Driver Helpers         **********************/
/*****************************************************************************/
/*****************************************************************************/
void aw_InitializeCore(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	core_initialize(&chip->port);
	chip->sink_timer = 31500;
	chip->source_timer = 28900;
	chip->source_end_timer = 31500;
	chip->port.sink_bist_reg = 0x60;
	chip->sink_reg_bist = 0x54;
	chip->source_reg_bist = 0x65;
	AW_LOG(" Core is initialized!\n");
}

AW_BOOL aw_IsDeviceValid(struct aw35615_chip *chip)
{
	int ret = 0;
	int i;
	AW_U8 val = 0;
	AW_BOOL id_ret = AW_FALSE;

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return id_ret;
	}

	for (i = 0; i <= 5; i++) {
		/* Read chip id */
		ret = DeviceRead(&chip->port, (AW_U8)0x01, 1, &val);
		if (ret < 0) {
			dev_err(&chip->client->dev,
					"%s - I2C error reading chip id Return: '%d'\n",
					__func__, ret);
			usleep_range(2000, 3000);
			id_ret = AW_FALSE;
		} else if ((val >> 4) == 0x9) {
			id_ret = AW_TRUE;
			chip->chip_id = 0x344f;
			break;
		}
	}

	AW_LOG("Device check passed chip id = 0x%x id_ret = %d\n", val, id_ret);

	return id_ret;
}

void aw_InitChipData(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (chip == NULL) {
		pr_err("%s - Chip structure is null!\n", __func__);
		return;
	}

	/* GPIO Defaults */
	chip->gpio_IntN = -1;

	/* DPM Setup - TODO - Not the best place for this. */
	chip->port.PortID = 0;
	DPM_Init(&chip->dpm);
	DPM_AddPort(chip->dpm, &chip->port);
	chip->port.dpm = chip->dpm;

	chip->gpio_IntN_irq = -1;

	/* I2C Configuration */
	// Number of times to retry I2C reads and writes
	chip->numRetriesI2C = RETRIES_I2C;

	/* Worker thread setup */
	INIT_WORK(&chip->sm_worker, work_function);

	chip->queued = AW_FALSE;

	chip->highpri_wq = alloc_workqueue("AWINIC WQ", WQ_HIGHPRI|WQ_UNBOUND, 1);

	if (chip->highpri_wq == NULL) {
		pr_err("%s - Unable to allocate new work queue!\n", __func__);
		return;
	}

	/* HRTimer Setup */
	hrtimer_init(&chip->sm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->sm_timer.function = aw_sm_timer_callback;
}


/*****************************************************************************/
/*****************************************************************************/
/**********************      IRQ/Threading Helpers       *********************/
/*****************************************************************************/
/*****************************************************************************/
AW_S32 aw_EnableInterrupts(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	AW_S32 ret = 0;

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	/* Set up IRQ for INT_N GPIO */
	ret = gpio_to_irq(chip->gpio_IntN);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s - Error: Unable to request IRQ for INT_N GPIO! Error code: %d\n",
			__func__, ret);
		chip->gpio_IntN_irq = -1;
		aw_GPIO_Cleanup();
		return ret;
	}
	chip->gpio_IntN_irq = ret;
	AW_LOG(" Success: Requested INT_N IRQ: '%d'\n", chip->gpio_IntN_irq);

	/* Use NULL thread_fn as we will be queueing a work function in the handler.
	 * Trigger is active-low, don't handle concurrent interrupts.
	 * devm_* allocation/free handled by system
	 */
	ret = devm_request_threaded_irq(&chip->client->dev, chip->gpio_IntN_irq,
			_aw_isr_intn, NULL, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			AW_DT_INTERRUPT_INTN, chip);
	if (ret) {
		dev_err(&chip->client->dev,
		"%s - Error: Unable to request threaded IRQ for INT_N GPIO! Error code: %d\n",
		__func__, ret);
		aw_GPIO_Cleanup();
		return ret;
	}

	enable_irq_wake(chip->gpio_IntN_irq);

	return 0;
}

void aw_StartTimer(struct hrtimer *timer, AW_U32 time_ms)
{
	ktime_t ktime;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return;
	}

	/* Set time in (seconds, nanoseconds) */
	ktime = ktime_set(time_ms / 1000, time_ms * 1000000);
	hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
}

void aw_StopTimer(struct hrtimer *timer)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return;
	}

	hrtimer_cancel(timer);
}

AW_U64 get_system_time_ms(void)
{
	AW_U64 time;

	time = ktime_get_ns();
	return (time / 1000000); /* return ms */
}

/*******************************************************************************
 * Function:        _aw_isr_intn
 * Input:           irq - IRQ that was triggered
 *                  dev_id - Ptr to driver data structure
 * Return:          irqreturn_t - IRQ_HANDLED on success, IRQ_NONE on failure
 * Description:     Activates the core
 ********************************************************************************/
static irqreturn_t _aw_isr_intn(AW_S32 irq, void *dev_id)
{
	struct aw35615_chip *chip = dev_id;

	if (!chip) {
		pr_err("AWINIC  %s - Error: Chip structure is NULL!\n", __func__);
		return IRQ_NONE;
	}

	aw_StopTimer(&chip->sm_timer);

	/* Schedule the process to handle the state machine processing */
	if (!chip->queued) {
		chip->queued = AW_TRUE;
		queue_work(chip->highpri_wq, &chip->sm_worker);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart aw_sm_timer_callback(struct hrtimer *timer)
{
	struct aw35615_chip *chip = container_of(timer, struct aw35615_chip, sm_timer);

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return HRTIMER_NORESTART;
	}

	/* Schedule the process to handle the state machine processing */
	if (!chip->queued) {
		chip->queued = AW_TRUE;
		queue_work(chip->highpri_wq, &chip->sm_worker);
	}

	return HRTIMER_NORESTART;
}

AW_BOOL aw_set_pdo(AW_U16 pdo_num, AW_U16 pdo_cur)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		AW_LOG("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return AW_FALSE;
	}

	if (!pdo_num) {
		AW_LOG("pdo_num(%d) is not supported\n", pdo_num);
		return AW_FALSE;
	}

	AW_LOG("Received.NumDataObjects = %d\n",
			chip->port.SrcCapsHeaderReceived.NumDataObjects);

	if ((chip->port.SrcCapsReceived[pdo_num - 1].PDO.SupplyType == pdoTypeAugmented) ||
			(pdo_num > chip->port.SrcCapsHeaderReceived.NumDataObjects) ||
			(pdo_num == 0)) {
		AW_LOG("The selected pdo is not supported\n");
		return AW_FALSE;
	}

	chip->port.USBPDTxFlag = AW_TRUE;
	chip->port.PDTransmitHeader.word = 0;
	chip->port.PDTransmitHeader.MessageType = DMTRequest;
	chip->port.PDTransmitHeader.NumDataObjects = 1;

	chip->port.PDTransmitObjects[0].object = 0;
	chip->port.PDTransmitObjects[0].FVRDO.ObjectPosition = pdo_num;
	chip->port.PDTransmitObjects[0].FVRDO.GiveBack =
			chip->port.PortConfig.SinkGotoMinCompatible;
	chip->port.PDTransmitObjects[0].FVRDO.CapabilityMismatch = AW_FALSE;
	chip->port.PDTransmitObjects[0].FVRDO.USBCommCapable =
			chip->port.PortConfig.SinkUSBCommCapable;
	chip->port.PDTransmitObjects[0].FVRDO.NoUSBSuspend =
			chip->port.PortConfig.SinkUSBSuspendOperation;
	chip->port.PDTransmitObjects[0].FVRDO.UnChnkExtMsgSupport = AW_FALSE;
	chip->port.PDTransmitObjects[0].FVRDO.OpCurrent = pdo_cur / 10;
	chip->port.PDTransmitObjects[0].FVRDO.MinMaxCurrent =
			chip->port.SrcCapsReceived[pdo_num - 1].FPDOSupply.MaxCurrent;

	if ((!chip->queued) && (chip->port.PolicyState == peSinkReady)) {
		chip->queued = AW_TRUE;
		queue_work(chip->highpri_wq, &chip->sm_worker);
		platform_delay_10us(500);
		AW_LOG("queue_work --> send request pdo\n");
		do {
			if (chip->port.PolicyState == peSinkSendSoftReset) {
				AW_LOG("request pdo fail\n");
				return AW_FALSE;
			}
			platform_delay_10us(100);
		} while (!(chip->port.PolicyState == peSinkReady));
	}

	return AW_TRUE;
}

AW_BOOL aw_set_apdo(AW_U16 apdo_vol, AW_U16 apdo_cur)
{
	struct aw35615_chip *chip = aw35615_GetChip();
	AW_U8 i = 0;
	AW_U8 apdo_num = 0;

	if (!chip) {
		AW_LOG("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return AW_FALSE;
	}

	AW_LOG("Received.NumDataObjects = %d\n", chip->port.SrcCapsHeaderReceived.NumDataObjects);

	for (i = chip->port.SrcCapsHeaderReceived.NumDataObjects; i > 0; i--) {
		if (chip->port.SrcCapsReceived[i - 1].PDO.SupplyType == pdoTypeAugmented) {
			apdo_num = i;
			break;
		}

		if (i == 1) {
			AW_LOG("The source does not support the PPS function\n");
			return AW_FALSE;
		}
	}

	chip->port.USBPDTxFlag = AW_TRUE;
	chip->port.PDTransmitHeader.word = 0;
	chip->port.PDTransmitHeader.MessageType = DMTRequest;
	chip->port.PDTransmitHeader.NumDataObjects = 1;

	chip->port.PDTransmitObjects[0].object = 0;
	chip->port.PDTransmitObjects[0].PPSRDO.ObjectPosition = apdo_num;
	chip->port.PDTransmitObjects[0].PPSRDO.CapabilityMismatch = AW_FALSE;
	chip->port.PDTransmitObjects[0].PPSRDO.USBCommCapable =
			chip->port.PortConfig.SinkUSBCommCapable;
	chip->port.PDTransmitObjects[0].PPSRDO.NoUSBSuspend =
			chip->port.PortConfig.SinkUSBSuspendOperation;
	chip->port.PDTransmitObjects[0].PPSRDO.UnChnkExtMsgSupport = AW_FALSE;
	chip->port.PDTransmitObjects[0].PPSRDO.Voltage = apdo_vol / 20;
	chip->port.PDTransmitObjects[0].PPSRDO.OpCurrent = apdo_cur / 50;

	if ((!chip->queued) && (chip->port.PolicyState == peSinkReady)) {
		chip->queued = AW_TRUE;
		queue_work(chip->highpri_wq, &chip->sm_worker);
		platform_delay_10us(500);
		AW_LOG("queue_work --> send request apdo\n");
		do {
			if (chip->port.PolicyState == peSinkSendSoftReset) {
				AW_LOG("request apdo fail\n");
				return AW_FALSE;
			}
			platform_delay_10us(100);
		} while (!(chip->port.PolicyState == peSinkReady));
	}

	return AW_TRUE;
}

static void work_function(struct work_struct *work)
{
	AW_U32 timeout = 0;

	struct aw35615_chip *chip =
		container_of(work, struct aw35615_chip, sm_worker);

	if (!chip) {
		pr_err("AWINIC  %s - Chip structure is NULL!\n", __func__);
		return;
	}

	/* Disable timer while processing */
	aw_StopTimer(&chip->sm_timer);

#ifdef AW_KERNEL_VER_OVER_4_19_1
	__pm_stay_awake(chip->aw35615_wakelock);
#else
	wake_lock(&chip->aw35615_wakelock);
#endif

	down(&chip->suspend_lock);

	/* Run the state machine */
	core_state_machine(&chip->port);

	/* Double check the interrupt line before exiting */
	if (platform_get_device_irq_state(chip->port.PortID)) {
		queue_work(chip->highpri_wq, &chip->sm_worker);
	} else {
		chip->queued = AW_FALSE;
		/* Scan through the timers to see if we need a timer callback */
		timeout = core_get_next_timeout(&chip->port);

		if (timeout > 0) {
			if (timeout == 1) {
				/* A value of 1 indicates that a timer has expired
				 * or is about to expire and needs further processing.
				 */
				queue_work(chip->highpri_wq, &chip->sm_worker);
			} else {
				/* A non-zero time requires a future timer interrupt */
				aw_StartTimer(&chip->sm_timer, timeout);
			}
		}
	}

	up(&chip->suspend_lock);
#ifdef AW_KERNEL_VER_OVER_4_19_1
	__pm_relax(chip->aw35615_wakelock);
#else
	wake_unlock(&chip->aw35615_wakelock);
#endif
}

void stop_usb_host(struct aw35615_chip *chip)
{
	AW_LOG("aw35615 - typec_attach_old= %d\n", chip->tcpc->typec_attach_old);

	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
	tcpci_notify_typec_state(chip->tcpc);
	if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC)
		chip->is_vbus_present = AW_FALSE;
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
}

void stop_otg_vbus(void)
{
	struct aw35615_chip *chip = aw35615_GetChip();

	AW_LOG("aw35615 - typec_attach_old= %d\n", chip->tcpc->typec_attach_old);

	if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC)
		chip->is_vbus_present = AW_FALSE;
}

void start_usb_host(struct aw35615_chip *chip, bool ss)
{
	AW_LOG("aw35615 - ss=%d\n", ss);
	chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
	if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
		chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
		chip->is_vbus_present = AW_TRUE;
		tcpci_notify_typec_state(chip->tcpc);
		chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
	}
}
void stop_usb_peripheral(struct aw35615_chip *chip)
{
	AW_LOG("aw35615 - typec_attach_old= %d\n", chip->tcpc->typec_attach_old);

	tcpci_notify_pd_state(chip->tcpc, PD_CONNECT_NONE);
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
	tcpci_notify_typec_state(chip->tcpc);

	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->port.vbus_flag_last = VBUS_DIS;
}

void stop_acc_audio(struct aw35615_chip *chip)
{
	AW_LOG("aw35615 - typec_attach_old= %d\n", chip->tcpc->typec_attach_old);

	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
	tcpci_notify_typec_state(chip->tcpc);
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
}

void start_usb_peripheral(struct aw35615_chip *chip)
{
	if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
		chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
		tcpci_notify_typec_state(chip->tcpc);
		chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
		chip->port.vbus_flag_last = VBUS_OK;
	}
}

#ifdef AW_HAVE_VBUS_ONLY
void start_usb_vbus_only(struct aw35615_chip *chip)
{
	if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
		chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
		tcpci_notify_typec_state(chip->tcpc);
		chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
		chip->port.vbus_flag_last = VBUS_OK;
	}
}
#endif /* AW_HAVE_VBUS_ONLY */

void start_usb_rp_rp(struct aw35615_chip *chip)
{
	if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
		chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
		tcpci_notify_typec_state(chip->tcpc);
		chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
		chip->port.vbus_flag_last = VBUS_OK;
	}
}

void start_acc_audio(struct aw35615_chip *chip)
{
	if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_AUDIO) {
		chip->tcpc->typec_attach_new = TYPEC_ATTACHED_AUDIO;
		tcpci_notify_typec_state(chip->tcpc);
		chip->tcpc->typec_attach_old = TYPEC_ATTACHED_AUDIO;
	}
}

void handle_core_event(AW_U32 event, AW_U8 portId, void *usr_ctx, void *app_ctx)
{
	static int usb_state;
	struct aw35615_chip *chip = aw35615_GetChip();

	if (!chip) {
		AW_LOG("aw35615 - Error: Chip structure is NULL!\n");
		return;
	}

	if (event != chip->old_event) {
		AW_LOG("event = %d\n", event);
		chip->old_event = event;
	}
	switch (event) {
	case CC1_ORIENT:
	case CC2_ORIENT:
		AW_LOG("aw35615 :CC Changed=0x%x\n", event);

		if (event == CC1_ORIENT)
			chip->tcpc->typec_polarity = false;
		else
			chip->tcpc->typec_polarity = true;

		if (chip->port.sourceOrSink == SINK) {
			start_usb_peripheral(chip);
			usb_state = 1;
			AW_LOG("aw35615 start_usb_peripheral\n");
		} else if (chip->port.sourceOrSink == SOURCE) {
			start_usb_host(chip, true);
			usb_state = 2;
			AW_LOG("aw35615 start_usb_host\n");
		}

		break;
	case CC1_AND_CC2:
		AW_LOG("aw35615 :detect double 56k cable\n");
		usb_state = 1;
		start_usb_rp_rp(chip);
		break;
#ifdef AW_HAVE_VBUS_ONLY
	case VBUS_OK:
		AW_LOG("aw35615 :detect vbus ok\n");
		usb_state = 1;
		start_usb_vbus_only(chip);
		break;
#endif /* AW_HAVE_VBUS_ONLY */
	case CC_AUDIO_ORIENT:
		AW_LOG("aw35615 :CC_AUDIO_ORIENT\n");
		start_acc_audio(chip);
		break;
	case CC_NO_ORIENT:
		AW_LOG("aw35615 CC_NO_ORIENT=0x%x\n", event);
		if (usb_state == 1) {
			stop_usb_peripheral(chip);
			usb_state = 0;
			AW_LOG("aw35615 - stop_usb_peripheral,event=0x%x,usb_state=%d\n",
					event, usb_state);
		} else if (usb_state == 2) {
			stop_usb_host(chip);
			usb_state = 0;
			AW_LOG("aw35615 - stop_usb_host,event=0x%x,usb_state=%d\n",
					event, usb_state);
		}
		break;
	case CC_AUDIO_OPEN:
		AW_LOG("aw35615 CC_AUDIO_OPEN=0x%x\n", event);
		stop_acc_audio(chip);
		break;
	case PD_STATE_CHANGED:
		AW_LOG("aw35615 :PD_STATE_CHANGED=0x%x, PE_ST=%d\n",
				event, chip->port.PolicyState);

		if (chip->port.PolicyState == peSinkSendHardReset)
			tcpci_notify_pd_state(chip->tcpc, PD_CONNECT_HARD_RESET);

		if (chip->port.PolicyState == peSinkReady &&
			chip->port.PolicyHasContract == AW_TRUE) {
			if (!chip->port.pd_state) {
				chip->port.pd_state = AW_TRUE;
				chip->tcpc->pd_port.data_role = PD_ROLE_UFP;
				chip->tcpc->pd_port.power_role = PD_ROLE_SINK;
				if (chip->port.src_support_pps)
					tcpci_notify_pd_state(chip->tcpc, PD_CONNECT_PE_READY_SNK_APDO);
				else
					tcpci_notify_pd_state(chip->tcpc, PD_CONNECT_PE_READY_SNK_PD30);
			}
			AW_LOG("aw35615 req_obj=0x%x, sel_src_caps=0x%x\n",
					chip->port.USBPDContract.FVRDO.ObjectPosition,
				chip->port.SrcCapsReceived[
				chip->port.USBPDContract.FVRDO.ObjectPosition - 1].object);
		} else if (chip->port.PolicyState == peSourceReady &&
			chip->port.PolicyHasContract == AW_TRUE) {
			if (!chip->port.pd_state) {
				chip->port.pd_state = AW_TRUE;
				chip->tcpc->pd_port.data_role = PD_ROLE_DFP;
				chip->tcpc->pd_port.power_role = PD_ROLE_SOURCE;
				tcpci_notify_pd_state(chip->tcpc, PD_CONNECT_PE_READY_SRC_PD30);
			}
		}
		break;
	case PD_NO_CONTRACT:
		AW_LOG("aw35615 :PD_NO_CONTRACT=0x%x, PE_ST=%d\n",
				event, chip->port.PolicyState);

		break;
	case PD_NEW_CONTRACT:
		AW_LOG("aw35615 :PD_NEW_CONTRACT=0x%x, PE_ST=%d\n",
				event, chip->port.PolicyState);
		if (chip->port.SrcCapsReceived[chip->port.SinkRequest.FVRDO.ObjectPosition - 1].PDO.SupplyType == pdoTypeAugmented) {
			AW_LOG("SinkRequestpps ObjectPosition = %d,Voltage = %d,Current = %d\n",
				chip->port.SinkRequest.PPSRDO.ObjectPosition,
				chip->port.SinkRequest.PPSRDO.Voltage * 20,
				chip->port.SinkRequest.PPSRDO.OpCurrent * 50);
			tcpci_sink_vbus(chip->tcpc, TCP_VBUS_CTRL_PD | TCP_VBUS_CTRL_PD_DETECT,
				chip->port.SinkRequest.PPSRDO.Voltage * 20, chip->port.SinkRequest.PPSRDO.OpCurrent * 50);
		} else {
			AW_LOG("SinkRequest ObjectPosition = %d,Voltage = %d,Current = %d\n",
				chip->port.SinkRequest.FVRDO.ObjectPosition,
				chip->port.SrcCapsReceived[chip->port.SinkRequest.FVRDO.ObjectPosition - 1].FPDOSupply.Voltage * 50,
				chip->port.SinkRequest.FVRDO.MinMaxCurrent * 10);
			tcpci_sink_vbus(chip->tcpc, TCP_VBUS_CTRL_PD | TCP_VBUS_CTRL_PD_DETECT,
				chip->port.SrcCapsReceived[chip->port.SinkRequest.FVRDO.ObjectPosition - 1].FPDOSupply.Voltage * 50,
				chip->port.SinkRequest.FVRDO.MinMaxCurrent * 10);
		}
		break;
	case DATA_ROLE:
		AW_LOG("aw35615 :DATA_ROLE=0x%x\n", event);

		if (chip->tcpc->pd_port.data_role == PD_ROLE_DFP) {
			chip->tcpc->pd_port.data_role = PD_ROLE_UFP;
			tcpci_notify_role_swap(chip->tcpc, TCP_NOTIFY_DR_SWAP, PD_ROLE_UFP);
		} else if (chip->tcpc->pd_port.data_role == PD_ROLE_UFP) {
			chip->tcpc->pd_port.data_role = PD_ROLE_DFP;
			tcpci_notify_role_swap(chip->tcpc, TCP_NOTIFY_DR_SWAP, PD_ROLE_DFP);
		}
		break;
	case POWER_ROLE:
		AW_LOG("aw35615 - POWER_ROLE event=0x%x", event);
		if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC) {
			usb_state = 1;
			chip->port.usb_commcapable = chip->port.USBPDContract.FVRDO.USBCommCapable;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
			chip->tcpc->pd_port.power_role = PD_ROLE_SINK;
			tcpci_notify_role_swap(chip->tcpc, TCP_NOTIFY_PR_SWAP, PD_ROLE_SINK);
		} else if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SNK) {
			usb_state = 2;
			chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			chip->tcpc->pd_port.power_role = PD_ROLE_SOURCE;
			tcpci_notify_role_swap(chip->tcpc, TCP_NOTIFY_PR_SWAP, PD_ROLE_SOURCE);
		}

		break;
	default:
		AW_LOG("aw35615 - default=0x%x", event);
		break;
	}
}

void aw35615_init_event_handler(void)
{
	/* max observer is 10 */
	register_observer(CC_ORIENT_ALL|PD_CONTRACT_ALL|POWER_ROLE|
			PD_STATE_CHANGED|DATA_ROLE|AUDIO_ACC|CUSTOM_SRC|
			EVENT_ALL,
			handle_core_event, NULL);
}
