#ifndef _FP_LINUX_DIRVER_H_
#define _FP_LINUX_DIRVER_H_
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#define DRIVER_VERSION "20210331"
#define ET721
#define USE_PIN_CTRL
//#define PLATFORM_SPI
#define MTK_PLATFORM
//#define LSI_PLATFORM
#define EGIS_DEBUG

#define DEBUG_PRINT(fmt, arg...) \
	printk(KERN_DEBUG "[EGISFP]" fmt, ##arg)

#define INFO_PRINT(fmt, arg...) \
	printk("[EGISFP]" fmt, ##arg)
#define ERROR_PRINT(fmt, arg...) \
	printk(KERN_ERR "[EGISFP]" fmt, ##arg)

#define EGIS_FP_MAJOR 100 /* assigned */
#define N_SPI_MINORS 32	  /* ... up to 256 */

#define EGIS_DEV_NAME "esfp0"
#define EGIS_INPUT_NAME "uinput-egis" /*"uinput-egis" */

#define EGIS_CHRD_DRIVER_NAME "egis_fp"
#define EGIS_CLASS_NAME "egis_fp"

/* ------------------------- Opcode -------------------------------------*/
#define FP_REGISTER_READ 0x01
#define FP_REGISTER_WRITE 0x02
#define FP_GET_ONE_IMG 0x03
#define FP_SENSOR_RESET 0x04
#define FP_POWER_ONOFF 0x05
#define FP_SET_SPI_CLOCK 0x06
#define FP_RESET_SET 0x07
#define FP_WAKELOCK_ENABLE 0x08
#define FP_WAKELOCK_DISABLE 0x09
#define FP_GET_RESOURCE 0x11
#define FP_FREE_RESOURCE 0x12

#define GET_IO_STUS 0x21
#define SEND_NAVI_EVENT 0x31
/* trigger signal initial routine*/
#define INT_TRIGGER_INIT 0xa4
/* trigger signal close routine*/
#define INT_TRIGGER_CLOSE 0xa5
/* read trigger status*/
#define INT_TRIGGER_READ 0xa6
/* polling trigger status*/
#define INT_TRIGGER_POLLING 0xa7
/* polling abort*/
#define INT_TRIGGER_ABORT 0xa8

#define FP_FREE_GPIO 0xaf
#define FP_SPICLK_ENABLE 0xaa
#define FP_SPICLK_DISABLE 0xab
#define DELETE_DEVICE_NODE 0xac

#define FP_SPIPIN_SETTING 0xad
#define FP_SPIPIN_PULLLOW 0xae

#define FP_POWERSETUP 0xb0
#define FP_WAKELOCK_TIMEOUT_ENABLE 0xb1
#define FP_WAKELOCK_TIMEOUT_DISABLE 0xb2
#define GET_SCREEN_ONOFF 0xb3

#define DRDY_IRQ_ENABLE 1
#define DRDY_IRQ_DISABLE 0

#define EDGE_TRIGGER_FALLING 0x0
#define EDGE_TRIGGER_RISING 0x1
#define LEVEL_TRIGGER_LOW 0x2
#define LEVEL_TRIGGER_HIGH 0x3
#define WAKE_HOLD_TIME 2000 //ms

#define SPI_DEFAULT_SPEED (1000000 * 20) //20M

#define Tpwr_off_delay 3
#define Tpwr_on_delay 3
#define Rst_off_delay 3
#define Rst_on_delay 12

#define KEY_FPS_DOWN   614
#define KEY_FPS_UP     615
#define KEY_FPS_TAP    616
#define KEY_FPS_HOLD   617
#define KEY_FPS_YPLUS  618
#define KEY_FPS_YMINUS 619
#define KEY_FPS_XPLUS  620
#define KEY_FPS_XMINUS 621
#define KEY_FPS_DOUBLE_TAP 622

#define EGIS_NAV_INPUT_UP KEY_FPS_YPLUS
#define EGIS_NAV_INPUT_DOWN KEY_FPS_YMINUS
#define EGIS_NAV_INPUT_LEFT KEY_FPS_XMINUS
#define EGIS_NAV_INPUT_RIGHT KEY_FPS_XPLUS
#define EGIS_NAV_INPUT_CLICK KEY_FPS_TAP
#define EGIS_NAV_INPUT_DOUBLE_CLICK KEY_FPS_DOUBLE_TAP
#define EGIS_NAV_INPUT_LONG_PRESS KEY_HOME
#define EGIS_NAV_INPUT_FINGER_DOWN KEY_FPS_DOWN
#define EGIS_NAV_INPUT_FINGER_UP KEY_FPS_UP



enum egis_navi_event
{
	NAVI_EVENT_ON,
	NAVI_EVENT_OFF,
	NAVI_EVENT_UP,
	NAVI_EVENT_DOWN,
	NAVI_EVENT_LEFT,
	NAVI_EVENT_RIGHT,
	NAVI_EVENT_CLICK,
	NAVI_EVENT_DOUBLE_CLICK,
	NAVI_EVENT_LONG_PRESS,
};

struct egis_key_map_t
{
	unsigned int type;
	unsigned int code;
};

struct egisfp_interrupt_desc_t
{
	int gpio;
	int number;
	char *name;
	int int_count;
	struct timer_list timer;
	int finger_on;
	int detect_period;
	int detect_threshold;
	int drdy_irq_flag;
	int drdy_irq_abort;
};

struct egisfp_ioctl_cmd_t
{
	uint32_t int_mode;
	uint32_t detect_period;
	uint32_t detect_threshold;
	uint32_t power_on;
};

/* ------------------------- Structure ------------------------------*/
struct egisfp_dev_t
{
	dev_t devt;
	spinlock_t spi_lock;
	struct platform_device *dd;
	struct list_head device_entry;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 *buffer;
	unsigned int irqPin;	  /* interrupt GPIO pin number */
	unsigned int rstPin;	  /* Reset GPIO pin number */
	unsigned int vdd_18v_Pin; /* Vdd GPIO pin number */
	unsigned int vcc_33v_Pin; /* Vcc GPIO pin number */
	struct input_dev *input_dev;
	int navigation_enable;
	int pars_dtsi_done;
	int platforminit_done;
	int gpio_irq;
	spinlock_t irq_lock;
	int request_irq_done;
	struct wakeup_source wakeup_source_fp;
	int ctrl_power;
	int pwr_by_gpio;
	int power_enable;
	int screen_onoff;
	struct regulator *vcc;
	struct regulator *vdd;
	uint32_t regulator_current;
	uint32_t regulator_voltage_max;
	uint32_t regulator_voltage_min;
	int clk_enabled;
	struct egisfp_interrupt_desc_t fps_ints;
	int clk_speed;
	struct pinctrl *pinctrl;
	struct pinctrl_state *spi_active;
	struct pinctrl_state *spi_default;
	struct pinctrl_state *reset_high;
	struct pinctrl_state *reset_low;
	struct pinctrl_state *vcc_high;
	struct pinctrl_state *vcc_low;
	struct pinctrl_state *irq_active;
	struct notifier_block notifier;
};

int egisfp_platforminit(struct egisfp_dev_t *egis_dev);
int egisfp_platformfree(struct egisfp_dev_t *egis_dev);
/* interrupt init */
int egisfp_interrupt_init(struct egisfp_dev_t *egis_dev, int int_mode, int detect_period, int detect_threshold);
/* interrupt free */
int egisfp_interrupt_free(struct egisfp_dev_t *egis_dev);
void egisfp_interrupt_abort(struct egisfp_dev_t *egis_dev);
unsigned int egisfp_interrupt_poll(struct file *file, struct poll_table_struct *wait);
int egisfp_parse_dt(struct egisfp_dev_t *egis_dev);
int do_egisfp_power_onoff(struct egisfp_dev_t *egis_dev, struct egisfp_ioctl_cmd_t *ioctl_data);
int do_egisfp_reset(struct egisfp_dev_t *egis_dev);
int do_egisfp_reset_set(struct egisfp_dev_t *egis_dev, int reset_high_low);
int egisfp_check_ioctl_permission(struct egisfp_dev_t *egis_dev, unsigned int cmd);
int egisfp_set_screen_onoff(int on);
#endif
