/*! \file sx9377_multi.c
 * \brief  SX9377 Driver
 *
 * Driver for the SX9377
 * Copyright (c) 2026 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>

#include <linux/device.h>
#include <linux/device/class.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/extcon.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/sensors.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <linux/input/sx9377.h>
#define MODULE_VER "3.6"

//=================================================================================================
// Features
//=================================================================================================
#define CFG_STAY_AWAKE       0
#define CFG_ESD_RECOVERY     1
#define CFG_EXTRA_LOG_TIME   1

#define SIMULATE_I2C_ERR     0 //for driver debug only

#if CFG_STAY_AWAKE
#ifdef CONFIG_PM
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#define WAKE_TMOUT 2000 //keep system wakeup for 2 seconds after a IRQ is received
#else
#error CONFIG_PM is not enabled in the Kconfig
#endif
#endif //CFG_STAY_AWAKE

#if CFG_EXTRA_LOG_TIME
#include <linux/timekeeping.h>
#include <linux/time64.h>
#include <linux/rtc.h>
#include <linux/time.h>
#endif //CFG_EXTRA_LOG_TIME

//#define CONFIG_CAPSENSE_HEADSET_STATE
//#define CONFIG_CAPSENSE_USB_CAL
//#define CONFIG_CAPSENSE_FLIP_CAL
//#define CONFIG_CAPSENSE_ATTACH_CAL
//#define CONFIG_CAPSENSE_POWER_CONTROL_SUPPORT

//=================================================================================================
// Const and structures
//=================================================================================================
#define NUM_IRQ_BITS            8
#define ESD_CHECK_INTERVAL_NOR  10  //seconds
#define ESD_CHECK_INTERVAL_ERR  2   //seconds
#define NUM_RETRY_ON_I2C_ERR    5   //times
#define SLEEP_BETWEEN_RETRY     10  //seconds

#define ESD_FAIL_CHECK_TIMES 3
#if ESD_FAIL_CHECK_TIMES < 3
#error At least check 3 times
#endif

typedef enum {
    NOT_USED = 0,   //this phase is not used.
    MAIN = 1,       //its CS is connected to an antenna for detecting human proximity.
    REF = 2,        //Reference phase. Used to correct the temperature drift of the correspoinding MAIN phase
}PHASE_USAGE;

//Refer to register 0x4280
typedef enum{
    CMD_ACTIVATE    = 0xF,
    CMD_COMPENSATE  = 0xE,
    CMD_PAUSE       = 0xD,
    CMD_RESUME      = 0xC,
}COMMANDS;

typedef enum {
    PROX0 = 0,
    PROX1 = 1,
    PROX2 = 2,
    PROX3 = 3,
    PROX4 = 4
}PROX_STATUS;

struct self_s;
typedef struct reg_val_s
{
    u32 addr;
    u32 val;
}reg_val_t, *reg_val_p;

typedef struct phase_s
{
    const char *name;

    u32 prox4_mask;
    u32 prox3_mask;
    u32 prox2_mask;
    u32 prox1_mask;

    bool enabled;
    PHASE_USAGE usage;
    PROX_STATUS state;

    struct input_dev *input;
    struct sensors_classdev sensor_class;

    struct self_s *self;
}phase_t, *phase_p;


typedef struct moto_data_s
{
#ifdef CONFIG_CAPSENSE_HEADSET_STATE
    int headset_operate_reg_num;
    reg_val_t *headset_operate_reg;
    reg_val_t *headset_operate_reg_bck;
#endif

#ifdef CONFIG_CAPSENSE_USB_CAL
    struct work_struct ps_notify_work;
    struct notifier_block ps_notif;
    bool ps_is_present;
#endif

#ifdef CONFIG_CAPSENSE_ATTACH_CAL
    bool phone_is_present;
#endif

#ifdef CONFIG_CAPSENSE_FLIP_CAL
    struct notifier_block flip_notif;
    struct extcon_dev *ext_flip_det;
    bool phone_flip_state;
    bool phone_flip_update_regs;
    int phone_flip_open_val;
    int num_flip_closed_regs;
    int num_flip_open_regs;
    reg_val_p flip_open_regs;
    reg_val_p flip_close_regs;
#endif

}moto_data_t, *moto_data_p;

typedef struct variables_s
{
    u16 reading_reg;
    int tcmd_read_flag; /* used for dump specified register*/
    u16 tcmd_read_reg; /* record reg address which want to read*/
    u32 irq_mask;
    u32 phen_reg_val;
    int cfg_forete_detected;
    bool esd_reinit_on;
}variables_t, *variables_p;

typedef struct power_supply_s
{
    POWER_SUPPLY type;
    struct regulator *cap_vdd;
    bool cap_vdd_en;
    bool eldo_vdd_en;
    int eldo_gpio;

}power_supply_t, *power_supply_p;

typedef struct debug_s
{
    bool log_hex_data;
    bool simulate_i2c_err;
}debug_t, *debug_p;

typedef struct self_s
{
    struct device *dev;
    struct device *class_dev;
    struct i2c_client *client;
    struct gpio_desc *gpio;

    int reg_ver;
    int num_dts_regs;
    reg_val_p dts_regs;

    u32 chip_id;
    int irq_id;
    int id;

    variables_t variables;
    debug_t dbg_flag;
    moto_data_t moto_data;
    power_supply_t power_supply;
    s8 main_map_ref[NUM_PHASES];
    /*Each bit represents whether its correspoding phase is used for(1) this runction or not(0)
    for example: main_phases=0x5=0b0101, means PHASE 2 and 0 are used as the main phase*/
    u16 main_phases;
    u16 ref_phases;
    phase_t phases[NUM_PHASES];
    int register_read_addr;
    u32 register_read_val;

    struct mutex phen_lock;

#if CFG_ESD_RECOVERY
    int esd_check_interval;
    struct delayed_work esd_worker;
    int useful_update_idx;
    u32 ph_useful[NUM_PHASES][ESD_FAIL_CHECK_TIMES];
    int i2c_error_times;
#endif

    char irq_disabled;
    void (*irq_handler[NUM_IRQ_BITS])(struct self_s* self);

}self_t, *Self;

//=================================================================================================
// Extra log time
//=================================================================================================
#define LOG_CHIP_NAME SMTC_CHIP_NAME "."

#if CFG_EXTRA_LOG_TIME
#define EXTRA_LOG_DATE_TIME 0

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 1, 0)
static char* get_log_time(void)
{
    int ms;
    struct timespec64 ts;
    struct rtc_time tm;

#if EXTRA_LOG_DATE_TIME
    static char log_time[] = "11-30 11:29:00.123 ";
#else
    static char log_time[] = "11:29:00.123 ";
#endif

    ktime_get_real_ts64(&ts);
    rtc_time64_to_tm(ts.tv_sec, &tm);

    ms = ts.tv_nsec;
    while(ms > 999){
        ms = (int)(ms/1000);
    }

#if EXTRA_LOG_DATE_TIME
    sprintf(log_time, "%02d-%02d %02d:%02d:%02d.%03d ",
        tm.tm_mon+1, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec, ms);
#else
    sprintf(log_time, "%02d:%02d:%02d.%03d ",
        tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
#endif

    return log_time;
}
#else //LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
static char* get_log_time(void)
{
    struct timex txc;
    struct rtc_time tm;
    int ms;
#if EXTRA_LOG_DATE_TIME
    static char log_time[] = "11-30 11:29:00.123 ";
#else
    static char log_time[] = "11:29:00.123 ";
#endif

    do_gettimeofday(&txc.time);
    rtc_time_to_tm(txc.time.tv_sec, &tm);

    ms = txc.time.tv_usec;
    while(ms > 999){
        ms = (int)(ms/1000);
    }

#if EXTRA_LOG_DATE_TIME
    sprintf(log_time, "%02d-%02d %02d:%02d:%02d.%03d ",
        tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec, ms);
#else
    sprintf(log_time, "%02d:%02d:%02d.%03d ",
        tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
#endif

    return log_time;
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)

#define LOG_DBG(fmt, args...)   pr_debug("%s" LOG_CHIP_NAME "%s(%d):" fmt "\n", get_log_time(), __func__, __LINE__, ##args)
#define LOG_INF(fmt, args...)   pr_info ("%s" LOG_CHIP_NAME "%s(%d):" fmt "\n", get_log_time(), __func__, __LINE__, ##args)
#define LOG_WRN(fmt, args...)   pr_warn ("%s" LOG_CHIP_NAME "%s(%d):" fmt "\n", get_log_time(), __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)   pr_err  ("%s" LOG_CHIP_NAME "%s(%d):" fmt "\n", get_log_time(), __func__, __LINE__, ##args)

#else //CFG_EXTRA_LOG_TIME
#define LOG_DBG(fmt, args...)   pr_debug(LOG_CHIP_NAME "%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define LOG_INF(fmt, args...)   pr_info (LOG_CHIP_NAME "%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define LOG_WRN(fmt, args...)   pr_warn (LOG_CHIP_NAME "%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)   pr_err  (LOG_CHIP_NAME "%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define get_log_time() ""
#endif //CFG_EXTRA_LOG_TIME

#define DEPRECATED_WRN(new_api) LOG_WRN("This function is deprecated, use %s instead.", #new_api)

//=================================================================================================
static const phase_t smtc_phase_table_template[] =
//=================================================================================================
{
    {
        .name = "Moto CapSense Ch0",
        .prox1_mask = 1 << 24,
        .prox2_mask = 1 << 16,
        .prox3_mask = 1 << 8,
        .prox4_mask = 1 << 0
    },
    {
        .name = "Moto CapSense Ch1",
        .prox1_mask = 1 << 25,
        .prox2_mask = 1 << 17,
        .prox3_mask = 1 << 9,
        .prox4_mask = 1 << 1
    },
    {
        .name = "Moto CapSense Ch2",
        .prox1_mask = 1 << 26,
        .prox2_mask = 1 << 18,
        .prox3_mask = 1 << 10,
        .prox4_mask = 1 << 2
    },
    {
        .name = "Moto CapSense Ch3",
        .prox1_mask = 1 << 27,
        .prox2_mask = 1 << 19,
        .prox3_mask = 1 << 11,
        .prox4_mask = 1 << 3
    },
    {
        .name = "Moto CapSense Ch4",
        .prox1_mask = 1 << 28,
        .prox2_mask = 1 << 20,
        .prox3_mask = 1 << 12,
        .prox4_mask = 1 << 4
    },
    {
        .name = "Moto CapSense Ch5",
        .prox1_mask = 1 << 29,
        .prox2_mask = 1 << 21,
        .prox3_mask = 1 << 13,
        .prox4_mask = 1 << 5

    },
    {
        .name = "Moto CapSense Ch6",
        .prox1_mask = 1 << 30,
        .prox2_mask = 1 << 22,
        .prox3_mask = 1 << 14,
        .prox4_mask = 1 << 6
    },
    {
        .name = "Moto CapSense Ch7",
        .prox1_mask = 1 << 31,
        .prox2_mask = 1 << 23,
        .prox3_mask = 1 << 15,
        .prox4_mask = 1 << 7
    }
};

static inline u32 smtc_get_chip_id(u32 chip_id){
    return (chip_id >> 8) & 0xFFFF;
}

static int is_irq_pin_low(Self self)
{
    return  !gpiod_get_value(self->gpio);
}

//=================================================================================================
// i2c read and write
//=================================================================================================
static int smtc_i2c_write(Self self, u16 reg_addr, u32 reg_val)
{
    int ret = 0;
    int num_retried = 0;
    struct i2c_msg msg;
    u8 wr_buf[6];
    struct i2c_client *i2c = self->client;

#ifdef EMULATION
    return 4;
#endif

#ifdef SIMULATE_I2C_ERR
    if (self->dbg_flag.simulate_i2c_err){
        LOG_WRN("Simulating i2c error");
        return -1;
    }
#endif

    wr_buf[0] = (u8)(reg_addr>>8);
    wr_buf[1] = (u8)(reg_addr);

    wr_buf[2] = (u8)(reg_val>>24);
    wr_buf[3] = (u8)(reg_val>>16);
    wr_buf[4] = (u8)(reg_val>>8);
    wr_buf[5] = (u8)(reg_val);

    msg.addr = i2c->addr;
    msg.flags = 0;
    msg.len = 6; //2 bytes regaddr + 4 bytes data
    msg.buf = (u8 *)wr_buf;

    while(true)
    {
        ret = i2c_transfer(i2c->adapter, &msg, 1);
        if (ret == 1){
            ret = 0;
            break;
        }

        if (num_retried++ < NUM_RETRY_ON_I2C_ERR)
        {
            LOG_ERR("i2c write reg 0x%x error %d. Goint to retry", reg_addr, ret);
            if(SLEEP_BETWEEN_RETRY != 0)
                msleep(SLEEP_BETWEEN_RETRY);
        }
        else{
            LOG_ERR("i2c write reg 0x%x error %d after retried %d times", reg_addr, ret, NUM_RETRY_ON_I2C_ERR);
             break;
        }
    }

    return ret;
}

//=================================================================================================
static int smtc_i2c_read(Self self, u16 reg_addr, u32 *reg_val)
{
    int ret = 0;
    int num_retried = 0;
    struct i2c_client *i2c = self->client;
    struct i2c_msg msg[2];
    u8 wr_buf[2], rd_buf[4];

#ifdef EMULATION
    *reg_val = 0;
    return 4;
#endif

#ifdef SIMULATE_I2C_ERR
    if (self->dbg_flag.simulate_i2c_err){
        LOG_WRN("Simulating i2c error");
        return -1;
    }
#endif

    wr_buf[0] = (u8)(reg_addr>>8);
    wr_buf[1] = (u8)(reg_addr);

    msg[0].addr = i2c->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = (u8 *)wr_buf;

    msg[1].addr = i2c->addr;;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 4;
    msg[1].buf = (u8 *)rd_buf;

    while(true)
    {
        ret = i2c_transfer(i2c->adapter, msg, 2);
        if (ret == 2){
            *reg_val = (u32)rd_buf[0]<<24 | (u32)rd_buf[1]<<16 | (u32)rd_buf[2]<<8 | (u32)rd_buf[3];
            ret = 0;
            break;
        }

        if (num_retried++ < NUM_RETRY_ON_I2C_ERR)
        {
            LOG_ERR("i2c read reg 0x%x error %d. Goint to retry", reg_addr, ret);
            if(SLEEP_BETWEEN_RETRY != 0){
                msleep(SLEEP_BETWEEN_RETRY);
            }
        }
        else{
            LOG_ERR("i2c read reg 0x%x error %d after retried %d times",
                reg_addr, ret, NUM_RETRY_ON_I2C_ERR);
            break;
        }
    }

    return ret;
}

//=================================================================================================
static int wait_reset_done(Self self)
{
    int ret = 0, i;
    u32 irq_src=0;

    for (i=0; i<5; i++)
    {
        msleep(10);
        ret = smtc_i2c_read(self, REG_IRQ_SRC, &irq_src);
        if (ret == 0 && irq_src != 0){
            LOG_INF("irq_src=0x%X", irq_src);
            return 0;
        }
    }

    LOG_WRN("No reset IRQ is detected");
    return 0;
}

static int smtc_send_cmd(Self self, COMMANDS cmd)
{
    u32 state;
    int retry_times=100, ret;
    while(1)
    {
        ret = smtc_i2c_read(self, REG_CMD_STATE, &state);
        if (ret < 0){
            LOG_ERR("Failed to send command.");
            return -EIO;
        }
        if ((state & 1) == 0){
            LOG_DBG("Chip is free and capable to process new command.");
            break;
        }

        if (--retry_times == 0){
            LOG_WRN("Chip keeps busy after 10 times retry.");
            break;
        }
        msleep(10);
        LOG_DBG("Chip is busy, go to retry.");
    }

    ret = smtc_i2c_write(self, REG_CMD, (u32)cmd);
    if (ret < 0){
        LOG_ERR("Failed to send command.");
        return -EIO;
    }

    return 0;
}

static int calibrate(Self self)
{
    int ret = 0;
    LOG_INF("Enter");
    ret = smtc_i2c_write(self, REG_CMD, 0xE);
    return ret;

}
const char* const IRQ_SRC_NAME[] = {
    "", "", "", "Conv", "Comp", "Far", "Close", "Reset"};

static void print_irq_src(int irq_src)
{
    int off;
    char msg_buf[64] = {0};
    sprintf(msg_buf, "0x%X: ", irq_src&0xFF);

    if (irq_src != 0){
        for (off=0; off<8; off++){
            if(irq_src & 1<<off ){
                strcat(msg_buf, IRQ_SRC_NAME[off]);
                strcat(msg_buf, " ");
            }
        }
        LOG_INF("%s", msg_buf);
    }
}

static int read_and_clear_irq(Self self, u32 *irq_src)
{
    int ret = 0;
    u32 reg_val;

    ret = smtc_i2c_read(self, REG_IRQ_SRC, &reg_val);
    if (ret < 0){
        ret = -EIO;
    }
    else{
        reg_val &= 0xFF;

        if (reg_val == 1<<3){
            LOG_DBG("irq_src= 0x%X", reg_val);
        }else{
            print_irq_src(reg_val);
        }

        if (irq_src){
            *irq_src = reg_val;
        }
        ret = 0;
    }

    return ret;
}

static int check_hardware(Self self)
{
    int ret=0, retry=1;
    u32 chip_id;

RETRY:
    ret = smtc_i2c_read(self, REG_WHOAMI, &chip_id);
    if(ret < 0){
        LOG_ERR("Failed to read chip id. ret= %d.", ret);
        return ret;
    }
    LOG_INF("chip_id=0x%X", chip_id);
    chip_id = smtc_get_chip_id(chip_id);
    //chip_id = 0x12; //simulate chip id error
    if (chip_id != self->chip_id)
    {
        LOG_WRN("read chip id 0x%X != 0x%X expected id.", chip_id, self->chip_id);

        if (retry){
            LOG_INF("Going to reset and retry");
            smtc_i2c_write(self, REG_RESET, 0xDE);
            ret = wait_reset_done(self);
            retry = 0;
            goto RETRY;
        }
        return -ENODEV;
    }

    return 0;
}

//=================================================================================================
static int __enable_phases_only(Self self, u32 phen_reg_val)
{
    int ret=0, phid;
    u32 prev_phen;
    LOG_DBG("phen_reg_val=0x%X", phen_reg_val);

    ret = smtc_i2c_read(self, REG_PHEN, &prev_phen);
    if (ret < 0){
        LOG_ERR("Failed to eanble phases. read reg");
        return ret;
    }

    for (phid=0; phid<NUM_PHASES; phid++)
    {
        if (phen_reg_val & 1<<phid){
            self->phases[phid].enabled = true;
            LOG_DBG("Enable phase= %d", phid);
        }else{
            self->phases[phid].enabled = false;
            LOG_DBG("Disable phase= %d", phid);
        }
    }
    LOG_INF("phase enable 0x%X ==> 0x%X", prev_phen, phen_reg_val);

    ret = smtc_i2c_write(self, REG_PHEN, phen_reg_val);
    if (ret < 0){
        LOG_ERR("Failed to eanble phases. write reg");
        return ret;
    }
    self->variables.phen_reg_val = phen_reg_val;

    ret = 0;
    if ((prev_phen & PHEN_MASK) == 0 && (phen_reg_val & PHEN_MASK) != 0){
        LOG_INF("activate chip");
        ret = smtc_send_cmd(self, CMD_ACTIVATE);
    }

    return ret;
}

static int __init_reg_probe(Self self)
{
    int i = 0, ret=0;
    u16 reg_addr;
    u32 reg_val=0, phen_reg_val = COMPENSATION_MASK;

    LOG_INF("Initing registers configured in the DTS. num_regs=%d", self->num_dts_regs);
    for (i=0; i < self->num_dts_regs; i++)
    {
        reg_addr = self->dts_regs[i].addr;
        reg_val  = self->dts_regs[i].val;

        if (reg_addr == REG_IRQ_MASK){
            self->variables.irq_mask = reg_val;

            if ((reg_val & 1<<4)==0){
                LOG_WRN("0x4004=0x%X, compensation IRQ should be enabled.", reg_val);
            }
        }

        if (reg_addr == REG_PHEN){
            phen_reg_val = reg_val;
        }
        else if (reg_addr == REG_CMD){
            LOG_WRN("Skip writing REG_CMD 0x%X=0x%X at probe", reg_addr, reg_val);
        }
        else{
            //LOG_DBG("0x%X=0x%08X", reg_addr, reg_val);
            ret = smtc_i2c_write(self, reg_addr, reg_val);
        }

        if (ret < 0){
            LOG_ERR("Failed to write reg=0x%x value=0x%X", reg_addr, reg_val);
            return ret;
        }
    }

    ret = __enable_phases_only(self, phen_reg_val);
    return ret;
}

//-------------------------------------------------------------------------------------------------
static int __init_reg_esd(Self self)
{
    int i = 0, ret=0;
    u16 reg_addr;
    u32 reg_val;
    LOG_INF("Initializing registers configured in the DTS. num_regs=%d", self->num_dts_regs);

    for (i=0; i < self->num_dts_regs; i++)
    {
        reg_addr = self->dts_regs[i].addr;
        reg_val  = self->dts_regs[i].val;

        if (reg_addr == REG_IRQ_MASK && (reg_val & 1<<4)==0){
            LOG_WRN("0x4004=0x%X, usually compensation IRQ should be enabled.", reg_val);
        }

        if (reg_addr != REG_PHEN && reg_addr != REG_CMD){
            //LOG_DBG("0x%X=0x%X", reg_addr, reg_val);
            ret = smtc_i2c_write(self, reg_addr, reg_val);
        }

        if (ret < 0){
            LOG_ERR("Failed to write reg=0x%x value=0x%X", reg_addr, reg_val);
            return ret;
        }
    }

    mutex_lock(&self->phen_lock);

    reg_val = self->variables.phen_reg_val;
    ret = __enable_phases_only(self, reg_val);
    mutex_unlock(&self->phen_lock);

    return 0;
}

#ifdef CONFIG_CAPSENSE_USB_CAL
static void ps_notify_callback_work(struct work_struct *work)
{
    u32 phen = 0;
    moto_data_p moto_data = container_of(work, moto_data_t, ps_notify_work);
    Self self = container_of(moto_data, self_t, moto_data);

    smtc_i2c_read(self, REG_PHEN, &phen);
    if (phen & 0xFF) {
        LOG_INF("USB state changed, force calibration");
        calibrate(self);
    }
}

static int ps_get_state(struct power_supply *psy, bool *present)
{
    int ret;
    union power_supply_propval pval = {0};

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &pval);
#else
    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &pval);
#endif

    if (ret) {
        LOG_DBG("Failed to get power supply property=%s", psy->desc->name);
        return ret;
    }
    *present = (pval.intval) ? true : false;

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
    LOG_DBG("%s is %s", psy->desc->name,
        (*present) ? "online" : "not online");
#else
    LOG_DBG("%s is %s", psy->desc->name,
        (*present) ? "present" : "not present");
#endif

    return 0;
}

static int ps_notify_callback(struct notifier_block *block,
    unsigned long event, void *p)
{
    int ret;
    struct power_supply *psy = p;
    bool present, is_target_event;
    moto_data_p moto_data = container_of(block, moto_data_t, ps_notif);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
    is_target_event = (event == PSY_EVENT_PROP_CHANGED);
#else
    is_target_event = (event == PSY_EVENT_PROP_ADDED || event == PSY_EVENT_PROP_CHANGED)
#endif
    if (is_target_event && psy && moto_data &&
        psy->desc->get_property && psy->desc->name &&
        !strncmp(psy->desc->name, "usb", sizeof("usb")))
    {
        LOG_DBG("ps notification: event = %lu", event);
        ret = ps_get_state(psy, &present);
        if (ret) {return ret;}

        if (event == PSY_EVENT_PROP_CHANGED) {
            if (moto_data->ps_is_present == present) {
                LOG_DBG("ps present state not change");
                return 0;
            }
        }
        moto_data->ps_is_present = present;
        schedule_work(&moto_data->ps_notify_work);
    }

#ifdef CONFIG_CAPSENSE_ATTACH_CAL
    if (event == PSY_EVENT_PROP_CHANGED && psy && moto_data &&
        psy->desc->get_property && psy->desc->name &&
        !strncmp(psy->desc->name, "phone", sizeof("phone")))
    {
        LOG_DBG("phone ps notification: event = %lu", event);

        ret = ps_get_state(psy, &present);
        if (ret){return ret;}

        if (moto_data->phone_is_present != present) {
            moto_data->phone_is_present = present;
            schedule_work(&moto_data->ps_notify_work);
        }
    }
#endif

    return 0;
}
#endif //CONFIG_CAPSENSE_USB_CAL

#ifdef CONFIG_CAPSENSE_FLIP_CAL
static void write_flip_regs(Self self, int num_regs, reg_val_p regs)
{
    int i;
    u32 addr, val;
    for(i=0; i < num_regs; i++)
    {
        addr = regs[i].addr;
        val = regs[i].val;
        LOG_DBG("Writing flip registers: 0x%X= 0x%X", addr, val);
        smtc_i2c_write(self, addr, val);
    }
}

static void update_flip_regs(Self self, unsigned long state)
{
    moto_data_p moto_data = &self->moto_data;
    if (moto_data->phone_flip_update_regs)
    {
        if (state == moto_data->phone_flip_open_val)
        {
            LOG_DBG("Writing %d regs on flip open", moto_data->num_flip_open_regs);
            write_flip_regs(self,
            moto_data->num_flip_open_regs, moto_data->flip_open_regs);
        }
        else
        {
            LOG_DBG("Writing %d regs on flip close", moto_data->num_flip_closed_regs);
            write_flip_regs(self,
            moto_data->num_flip_closed_regs, moto_data->flip_close_regs);
        }
    }
}

static int flip_notify_callback(struct notifier_block *notifier,
    unsigned long state, void *p)
{
    struct extcon_dev *edev = p;
    moto_data_p moto_data = container_of(notifier, moto_data_t, flip_notif);
    Self self = container_of(moto_data, self_t, moto_data);

    if(moto_data->ext_flip_det == edev)
    {
        if(moto_data->phone_flip_state != state)
        {
            update_flip_regs(self, state);
            moto_data->phone_flip_state = state;
            schedule_work(&moto_data->ps_notify_work);
        }
    }

    return 0;
}
#endif
//=================================================================================================
#ifdef CONFIG_CAPSENSE_HEADSET_STATE
static int init_headset_reg(Self self)
{
    int i = 0, ret;
    moto_data_p moto_data = &self->moto_data;
    LOG_INF("num headset reg= %d", moto_data->headset_operate_reg_num);

    for (i=0; i < moto_data->headset_operate_reg_num; i++)
    {
        ret = smtc_i2c_read(self, moto_data->headset_operate_reg_bck[i].addr,
            &(moto_data->headset_operate_reg_bck[i].val));

        if (ret<0){return ret;}
    }
    return 0;
}
#endif
//=================================================================================================
static int reset_and_init_chip_probe(Self self)
{
    int ret = 0;
    LOG_INF("Enter");

    ret = smtc_i2c_write(self, REG_RESET, 0xDE);
    if(ret<0){goto SUB_OUT;}

    ret = wait_reset_done(self);
    if(ret){goto SUB_OUT;}

    ret = __init_reg_probe(self);
    if(ret){goto SUB_OUT;}

#ifdef CONFIG_CAPSENSE_HEADSET_STATE
    ret = init_headset_reg(self);
    if(ret){goto SUB_OUT;}
#endif

SUB_OUT:
    ret = read_and_clear_irq(self, NULL);
    return ret;
}

//-------------------------------------------------------------------------------------------------
static int reset_and_init_chip_esd(Self self)
{
    int ret = 0;
    LOG_INF("Enter");

    disable_irq(self->irq_id);
    ret = smtc_i2c_write(self, REG_RESET, 0xDE);
    if(ret<0){goto SUB_OUT;}

    ret = wait_reset_done(self);
    if(ret){goto SUB_OUT;}

    ret = __init_reg_esd(self);
    if(ret){goto SUB_OUT;}

#ifdef CONFIG_CAPSENSE_FLIP_CAL
    update_flip_regs(self, self->moto_data.phone_flip_state);
#endif

SUB_OUT:
    enable_irq(self->irq_id);
    return ret;
}

static u32 off_to_dcap(u16 offset)
{
    u32 hig, low;

    hig = (offset >> 7 & 0x7F) * 31000;
    low = (offset & 0x7F) * 540;
    return (hig + low)/10000;
}

static void get_prox_state(Self self, u32 prox_reg_val, PROX_STATUS ph_state[NUM_PHASES])
{
    int ph;
    phase_p phase;
    LOG_DBG("prox_reg_val= 0x%X", prox_reg_val);

    for (ph=0; ph<NUM_PHASES; ph++)
    {
        phase = &self->phases[ph];
        //The prox4_mask of sx933x is always 0
        if (prox_reg_val & phase->prox4_mask){
            ph_state[ph] = PROX4;
        }
        else if (prox_reg_val & phase->prox3_mask){
            ph_state[ph] = PROX3;
        }
        else if (prox_reg_val & phase->prox2_mask){
            ph_state[ph] = PROX2;
        }
        else if (prox_reg_val & phase->prox1_mask){
            ph_state[ph] = PROX1;
        }
        else{
            ph_state[ph] = PROX0;
        }
    }
}

//=================================================================================================
static void smtc_log_dbg_data(Self self, int phid, PROX_STATUS state)
{
    u16 off;
    u32 reg_val;
    phase_p phase;
    u32 use_hex, avg_hex, dif_hex, dlt_hex;
    s32 main_use, main_raw, avg, diff, ref_ph, ref_use, dlt_var;

    smtc_i2c_read(self, REG_RAW_DATA, &reg_val);
    main_raw = (s32)reg_val>>10;

    smtc_i2c_read(self, REG_DLT_VAR, &dlt_hex);
    dlt_var = (s32)dlt_hex>>4;

    smtc_i2c_read(self, REG_USE_PH0 + phid*4, &use_hex);
    main_use = (s32)use_hex>>10;

    smtc_i2c_read(self, REG_AVG_PH0 + phid*4, &avg_hex);
    avg = (s32)avg_hex>>10;

    smtc_i2c_read(self, REG_DIF_PH0 + phid*4, &dif_hex);
    diff = (s32)dif_hex>>10;

    smtc_i2c_read(self, REG_OFF_PH0 + phid*4*OFFSET_PH_REG_SHIFT, &reg_val);
    off = (u16)(reg_val & OFFSET_VAL_MASK);

    phase = &self->phases[phid];
    ref_ph = self->main_map_ref[phid];
    if (phase->usage != MAIN || ref_ph == -1){
        ref_ph = phid;
        ref_use = main_use;
    }else{
        smtc_i2c_read(self, REG_USE_PH0 + ref_ph*4, &reg_val);
        ref_use = (s32)reg_val >> 10;
    }

    pr_info(
    "%sSMTC_DBG PH=%d DIFF=%d PH%d_USE=%d RAW=%d USE=%d AVG=%d STATE=%d OFF=%d DLT=%d SMTC_END\n",
    get_log_time(),
    phid, diff, ref_ph, ref_use, main_raw, main_use, avg, state, off, dlt_var);

    if (self->dbg_flag.log_hex_data){
        pr_info(
        "%sSMTC_HEX PH=%d USE=0x%X AVG=0x%X DIF=0x%X DLT=0x%X SMTC_END\n",
        get_log_time(),
        phid, use_hex, avg_hex, dif_hex, dlt_hex);
    }
}

//=================================================================================================
static void log_raw_data(Self self)
{
    u16 offset;
    phase_p phase;
    int phid, state, addr_off;
    u32 reg_val, dbg_ph, dcap;
    s32 useful, average, diff, ref_ph, ref_use;
    PROX_STATUS ph_prox_state[NUM_PHASES];

    smtc_i2c_read(self, REG_PROX_STATUS, &reg_val);
    LOG_DBG("prox_state= 0x%X", reg_val);
    get_prox_state(self, reg_val, ph_prox_state);

    smtc_i2c_read(self, REG_DBG_SEL, &dbg_ph);
    dbg_ph = (dbg_ph >> 3) & 0x7;

    for(phid =0; phid<NUM_PHASES; phid++)
    {
        phase = &self->phases[phid];

        addr_off = phid*4;
        smtc_i2c_read(self, REG_USE_PH0 + addr_off, &reg_val);
        useful = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_AVG_PH0 + addr_off, &reg_val);
        average = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_DIF_PH0 + addr_off, &reg_val);
        diff = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_OFF_PH0 + addr_off*OFFSET_PH_REG_SHIFT,  &reg_val);
        offset = (u16)(reg_val & OFFSET_VAL_MASK);
        dcap = off_to_dcap(offset);
        state = ph_prox_state[phid];

        ref_ph = self->main_map_ref[phid];
        if (phase->usage != MAIN || ref_ph == -1){
            ref_ph = phid;
            ref_use = useful;
        }else{
            smtc_i2c_read(self, REG_USE_PH0 + ref_ph*4, &reg_val);
            ref_use = (s32)reg_val >> 10;
        }

        pr_info(
        "%sSMTC_DAT PH=%d DIFF=%d PH%d_USE=%d USE=%d AVG=%d STATE=%d OFF=%d CAP=%d SMTC_END\n",
        get_log_time(),
        phid, diff, ref_ph, ref_use, useful, average, state, offset, dcap);
    }

    smtc_log_dbg_data(self, dbg_ph, ph_prox_state[dbg_ph]);
}

//=================================================================================================
/*phases: include the the compensation flags*/
static int smtc_enable_phases(Self self, u32 phases)
{
    int ret;
    LOG_INF("phases=0x%X", phases);

#if CFG_ESD_RECOVERY
    if (self->variables.esd_reinit_on)
    {
        cancel_delayed_work_sync(&self->esd_worker);
        schedule_delayed_work(&self->esd_worker,
            msecs_to_jiffies(ESD_CHECK_INTERVAL_NOR*1000));
    }
#endif

    mutex_lock(&self->phen_lock);
    ret = __enable_phases_only(self, phases);
    mutex_unlock(&self->phen_lock);

    return ret;
}

//=================================================================================================
// class nodes
//=================================================================================================
#define DEVICE_ATTR_SHOW(api_name) \
static ssize_t api_name##_show(struct device *dev, \
    struct device_attribute *attr, \
    char *buf)

#define DEVICE_ATTR_STORE(api_name) \
static ssize_t api_name##_store(struct device *dev, \
    struct device_attribute *attr, \
    const char *buf, size_t count)

//=================================================================================================
#if CFG_ESD_RECOVERY
static void cfg_esd_recovery(Self self, bool enable)
{
    variables_p variables = &self->variables;
    if (enable)
    {
        if (variables->esd_reinit_on){
            LOG_WRN("ESD recovery is already enabled");
        }else{
            LOG_INF("Enable ESD recovery");
            variables->esd_reinit_on = true;
            schedule_delayed_work(&self->esd_worker,
                msecs_to_jiffies(ESD_CHECK_INTERVAL_NOR*1000));
        }
    }
    else
    {
        if (!variables->esd_reinit_on){
            LOG_WRN("ESD recovery is already disabled");
        }else{
            LOG_INF("Disable ESD recovery");
            variables->esd_reinit_on = false;
            cancel_delayed_work_sync(&self->esd_worker);
        }
    }
}
#endif
//=================================================================================================
// raw_data
//=================================================================================================
DEVICE_ATTR_SHOW(raw_data)
{
    u16 offset;
    u32 uData, dcap;
    int phid=0, count=0;
    s32 useful, average, diff;
    Self self = dev_get_drvdata(dev);

    for(phid =0; phid<8; phid++)
    {
        smtc_i2c_read(self, REG_USE_PH0 + phid*4, &uData);
        useful = (s32)uData>>10;
        smtc_i2c_read(self, REG_AVG_PH0 + phid*4, &uData);
        average = (s32)uData>>10;
        smtc_i2c_read(self, REG_DIF_PH0 + phid*4, &uData);
        diff = (s32)uData>>10;
        smtc_i2c_read(self, REG_OFF_PH0 + phid*12, &uData);
        offset = (u16)(uData & 0x3FFF);
        dcap = off_to_dcap(offset);

        count += snprintf(buf+count, PAGE_SIZE - count,
        "PH= %d Useful= %d Average= %d DIFF= %d Offset= %d DCAP= %d\n",
            phid, useful, average, diff, offset, dcap);
    }

    return count;
}

//=================================================================================================
// reg_write
//=================================================================================================
static ssize_t smtc_reg_write(Self self, const char *buf, size_t count)
{
    u32 reg_addr = 0, reg_val = 0;

    if (sscanf(buf, "%x,%x", &reg_addr, &reg_val) != 2){
        LOG_ERR("Invalid command format. Example: ehco '0x4280,0xE' > reg_write");
        return -EINVAL;
    }
    LOG_INF("0x%X= 0x%X", reg_addr, reg_val);

    if (reg_addr == REG_PHEN){
        smtc_enable_phases(self, reg_val);
    }
    else{
        if (reg_addr == REG_IRQ_MASK){
            self->variables.irq_mask = reg_val;
        }

        smtc_i2c_write(self, reg_addr, reg_val);
    }

    return count;
}

//=================================================================================================
DEVICE_ATTR_STORE(reg_write)
{
    Self self = dev_get_drvdata(dev);
    return smtc_reg_write(self, buf, count);
}

//=================================================================================================
DEVICE_ATTR_SHOW(reg_write)
{
    return sprintf(buf,
        "\nUsage: echo reg_addr,reg_val > reg_write\n"
        "Example: echo 0x4004,0x78 > reg_write\n");
}

//=================================================================================================
// reg_read
//=================================================================================================
DEVICE_ATTR_SHOW(reg_read)
{
    int count=0, ret;
    u32 reg_val = 0, reading_reg;
    Self self = dev_get_drvdata(dev);
    reading_reg = self->variables.reading_reg;

    if (reading_reg == 0xFFFF){
        count = sprintf(buf,
            "\nUsage: echo reg_addr > reg_read; cat reg_read\n"
            "Example: echo 0x4000 > reg_read; cat reg_read\n");
    }else{
        ret = smtc_i2c_read(self, reading_reg, &reg_val);

        if (ret < 0){
            count = sprintf(buf, "Failed to read reg=0x%X\n", reading_reg);
        }else{
            count = sprintf(buf, "0x%X= 0x%08X\n", reading_reg, reg_val);
        }
    }

    return count;
}
//=================================================================================================
DEVICE_ATTR_STORE(reg_read)
{
    int ret;
    u32 addr=0,val=0;
    Self self = dev_get_drvdata(dev);

    if (sscanf(buf, "%x", &addr) != 1)
    {
        LOG_ERR(
            "echo reg_addr > reg_read; cat reg_read\n"
            "Example: echo 0x4000 > reg_read; cat reg_read\n");
        return -EINVAL;
    }

    self->variables.reading_reg = addr;
    ret = smtc_i2c_read(self, addr, &val);

    if (ret < 0){
        LOG_ERR("Failed to read register 0x%X", addr);
    }else{
        LOG_INF("0x%X= 0x%X", addr, val);
    }

    return count;
}

//=================================================================================================
// calibrate
//=================================================================================================
DEVICE_ATTR_SHOW(fac_compensation)
{
    int i, count=0, ph, shift;
    u32 dcap, reg_val = 0;
    u16 offset;
    Self self = dev_get_drvdata(dev);

    for (i=0; i<60; i++)
    {
        smtc_i2c_read(self, 0x8004,  &reg_val);
        if (reg_val & COMPENSATION_MASK){
            msleep(50);
        }else{
            break;
        }
    }

    if (i==60){
        count = sprintf(buf, "%s", "Compensation did not complete within 3 seconds.");
        LOG_WRN("%s", buf);
        return count;
    }else{
        LOG_INF("chip completed the compensation after waiting for %d ms", i*50);
    }

    for(ph =0; ph < NUM_PHASES; ph++)
    {
        shift = ph*4;
        smtc_i2c_read(self, REG_OFF_PH0 + shift*OFFSET_PH_REG_SHIFT,  &reg_val);
        offset = (u16)(reg_val & OFFSET_VAL_MASK);
        dcap = off_to_dcap(offset);
        count += sprintf(buf+count, "PH%d=%d dcap=%d, ", ph, offset, dcap);
    }
    count += sprintf(buf+count, "\n");
    LOG_INF("%s", buf);
    return count;
}
//=================================================================================================
DEVICE_ATTR_STORE(fac_cal)
{
    Self self = dev_get_drvdata(dev);
    LOG_INF("Manual calibrating");
    calibrate(self);
    return count;
}

//=================================================================================================
// headset
//=================================================================================================
#ifdef CONFIG_CAPSENSE_HEADSET_STATE
DEVICE_ATTR_STORE(headset)
{
    int i;
    u16 addr;
    u32 phen = 0, val;
    Self self = dev_get_drvdata(dev);
    moto_data_p moto_data = &self->moto_data;

    if (!count){
        LOG_ERR("Invalid count= %zu", count);
        return -EINVAL;
    }
    smtc_i2c_read(self, REG_PHEN, &phen);

    if (!strncmp(buf, "1", 1))
    {
        LOG_INF("headset in update reg num:%d", moto_data->headset_operate_reg_num);
        for (i = 0; i < moto_data->headset_operate_reg_num; i++)
        {
            addr = moto_data->headset_operate_reg[i].addr;
            val = moto_data->headset_operate_reg[i].val;
            smtc_i2c_write(self, addr, val);
            LOG_DBG("set Reg 0x%X=0x%X", addr, val);
        }

        //calibrate sensor
        if (phen & 0xFF) {
            LOG_INF("manual cailibrate");
            calibrate(self);
        }
    }
    else if (!strncmp(buf, "0", 1))
    {
        LOG_INF("headset out back reg num=%d", moto_data->headset_operate_reg_num);
        for (i = 0; i < moto_data->headset_operate_reg_num; i++)
        {
            addr = moto_data->headset_operate_reg_bck[i].addr;
            val = moto_data->headset_operate_reg_bck[i].val;

            smtc_i2c_write(self, addr, val);
            LOG_DBG("set Reg 0x%X=0x%X", addr, val);
        }
        //cal sensor
        if (phen & 0xFF) {
            LOG_INF("manual cailibrate");
            calibrate(self);
        }
    }
    return count;
}
#endif //CONFIG_CAPSENSE_HEADSET_STATE

static void smtc_test_func(Self self)
{
    LOG_INF("Add your debug function here");
}

//=================================================================================================
// debug
//=================================================================================================
DEVICE_ATTR_STORE(debug)
{
    int cmd, arg;
    Self self = dev_get_drvdata(dev);
    debug_p dbg_flag = &self->dbg_flag;

    if (sscanf(buf, "%d,%d", &cmd, &arg) != 2){
        LOG_ERR("Invalid command format. Use 'cat debug' to show the usages.");
        return -EINVAL;
    }

    switch (cmd){
    case 0:
        smtc_test_func(self);
        break;
    case 3:
        dbg_flag->log_hex_data = arg;
        LOG_INF("%s log hex data", arg ? "Enable" : "Disable");
        break;

#if CFG_ESD_RECOVERY
    case 5:
        cfg_esd_recovery(self, arg);
        break;
#endif

#if SIMULATE_I2C_ERR
    case 20:
        LOG_INF("%s simulating i2c error", arg ? "Enable" : "Disable");
        self->dbg_flag.simulate_i2c_err = arg;
        break;
#endif

   default:
        LOG_ERR("Invalid command=%d. Use 'cat debug' to show the usages.", cmd);
    }

    return count;
}
//=================================================================================================
DEVICE_ATTR_SHOW(debug)
{
    return sprintf(buf, "%s",
        "Usage: echo 'cmd,arg' > debug\n"
        "cmd:\n"

        "3: Turn on(arg=1) | off(arg=0), default=0\n"
        "   Log some of registers value(Useful, average, etc.) in hex format.\n"

#if CFG_ESD_RECOVERY
        "5: Turn on(arg=1) | off(arg=0), default=1\n"
        "   Enable(arg=1) | disable(arg=0) ESD recovery.\n"
#endif

#if SIMULATE_I2C_ERR
        "20: Enable(arg=1) | disable(arg=0), default=0\n"
        "   Simulating i2c read/write error.\n"
#endif
        "\n"
    );
}

//=================================================================================================
// registers
//=================================================================================================
DEVICE_ATTR_SHOW(registers)
{
    int i, bytes=0;
    u32 addr, val;
    Self self = dev_get_drvdata(dev);

    for (i=0; i<self->num_dts_regs; i++)
    {
        addr = self->dts_regs[i].addr;

        if (smtc_i2c_read(self, addr, &val) == 0){
            bytes += sprintf(buf+bytes, "0x%X=0x%08X\n", addr, val);
        }else{
            bytes += sprintf(buf+bytes, "0x%X=FAILED\n", addr);
        }
    }
    return bytes;
}

//=================================================================================================
// int_state
//=================================================================================================
DEVICE_ATTR_SHOW(fac_irq_state)
{
    Self self = dev_get_drvdata(dev);
    LOG_DBG("Reading INT line state\n");
    return sprintf(buf, "%d\n", gpiod_get_value(self->gpio));
}

//=================================================================================================
// reinitialize
//=================================================================================================
DEVICE_ATTR_STORE(reinitialize)
{
    Self self = dev_get_drvdata(dev);
    reset_and_init_chip_esd(self);
    return count;
}

//=================================================================================================
// tcmd registers
//=================================================================================================
DEVICE_ATTR_SHOW(tcmd_reg)
{
    u32 *p = (u32*)buf;
    Self self = dev_get_drvdata(dev);
    variables_p variables = &self->variables;

#ifdef CONFIG_CAPSENSE_POWER_CONTROL_SUPPORT
    moto_data_p moto_data = &self->moto_data;
    if (!moto_data->cap_vdd_en)
        if ( power_on_chip(self) != 0)
            return -1;
#endif

    if(variables->tcmd_read_flag)
    {
        variables->tcmd_read_flag = 0;
        if (smtc_i2c_read(self, variables->tcmd_read_reg, p) < 0){
            return -1;
        }
        LOG_DBG("read_reg = 0x%x val = 0x%x\n", variables->tcmd_read_reg, *p);
        return 4;
    }
    return -1;
}
//=================================================================================================
DEVICE_ATTR_STORE(tcmd_reg)
{
/*
    reg attr is for TCMD on MTK only,
    buf[6]-read_flag:
    0-real write,
    1-just transfer the reg value want to be readed
*/
    int i = 0;
    u32 reg_val = 0;
    u16 reg_addr = 0;
    Self self = dev_get_drvdata(dev);
    variables_p variables = &self->variables;

#ifdef CONFIG_CAPSENSE_POWER_CONTROL_SUPPORT
    moto_data_p moto_data = &self->moto_data;

    if (!moto_data->cap_vdd_en){
        if ( power_on_chip(self) != 0){
            return -1;
        }
    }
#endif

    if( count != 7){
        LOG_ERR("Invalid count= %zu !=7", count);
        return -EINVAL;
    }

    for(i = 0 ; i < count ; i++){
        LOG_DBG("buf[%d] = 0x%X", i, buf[i]);
    }

    if(buf[6] == 0)
    {
        reg_addr = ((u16)buf[0]<<8) | (u16)buf[1];
        reg_val= ((u32)buf[2]<<24) | ((u32)buf[3]<<16) | ((u32)buf[4]<<8) | ((u32)buf[5]);

        if (smtc_i2c_write(self, reg_addr, reg_val) < 0)
            return -1;
    }
    else if(buf[6] == 1)
    {
        variables->tcmd_read_reg = ((u16)buf[0]<<8) | (u16)buf[1];
        variables->tcmd_read_flag = 1;
    }

    return count;
}

//=================================================================================================
// deprecated register read
//=================================================================================================
DEVICE_ATTR_SHOW(register_read)
{
    Self self = dev_get_drvdata(dev);
    DEPRECATED_WRN(reg_read_show);
    return sprintf(buf, "Register 0x%X=0x%08X\n", self->register_read_addr, self->register_read_val);
}

//=================================================================================================
DEVICE_ATTR_STORE(register_read)
{
    int nirq_state = 0;
    Self self = dev_get_drvdata(dev);
    DEPRECATED_WRN("reg_read_store");

    if (sscanf(buf, "%x", &self->register_read_addr) != 1)
    {
        LOG_ERR(" The number of data are wrong\n");
        return -EINVAL;
    }

    smtc_i2c_read(self, self->register_read_addr, &self->register_read_val);
    nirq_state = read_and_clear_irq(self, NULL);

    LOG_INF("Register 0x%X=0x%08X nirq_state(%d)\n", self->register_read_addr, self->register_read_val, nirq_state);
    return count;
}

//=================================================================================================
// deprecated register write
//=================================================================================================
DEVICE_ATTR_STORE(register_write)
{
    Self self = dev_get_drvdata(dev);
    DEPRECATED_WRN(reg_write_store);
    return smtc_reg_write(self, buf, count);
}

//=================================================================================================
// deprecated reset
//=================================================================================================
DEVICE_ATTR_STORE(reset)
{
    u32 phen = 0;
    Self self = dev_get_drvdata(dev);

    if (!count){
        LOG_ERR("Invalid count= %zu", count);
        return -EINVAL;
    }
    DEPRECATED_WRN(calibrate_store);
    smtc_i2c_read(self, REG_PHEN, &phen);

    if (!strncmp(buf, "reset", 5) || !strncmp(buf, "1", 1)) {
        if (phen & 0xFF) {
            LOG_INF("manual cailibrate");
            calibrate(self);
        }
    }

    return count;
}

//=================================================================================================
DEVICE_ATTR_SHOW(reset)
{
    u32 reg_value = 0;
    Self self = dev_get_drvdata(dev);

    DEPRECATED_WRN(calibrate_show);
    LOG_DBG("Reading IRQSTAT_REG\n");
    smtc_i2c_read(self,REG_IRQ_SRC,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}

//=================================================================================================
//deprecated tcmd register
//=================================================================================================
DEVICE_ATTR_SHOW(reg)
{
    u32 *p = (u32*)buf;
    Self self = dev_get_drvdata(dev);
    variables_p variables = &self->variables;

#ifdef CONFIG_CAPSENSE_POWER_CONTROL_SUPPORT
    moto_data_p moto_data = &self->moto_data;
    if (!moto_data->cap_vdd_en)
        if ( power_on_chip(self) != 0)
            return -1;
#endif
    DEPRECATED_WRN(tcmd_reg_show);

    if(variables->tcmd_read_flag)
    {
        variables->tcmd_read_flag = 0;
        if (smtc_i2c_read(self, variables->tcmd_read_reg, p) < 0){
            return -1;
        }
        LOG_DBG("read_reg = 0x%x val = 0x%x\n", variables->tcmd_read_reg, *p);
        return 4;
    }
    return -1;
}

/*
    reg attr is for TCMD on MTK only,
    buf[6]-read_flag:
    0-real write,
    1-just transfer the reg value want to be readed
*/
DEVICE_ATTR_STORE(reg)
{
    int i = 0;
    u32 reg_val = 0;
    u16 reg_addr = 0;
    Self self = dev_get_drvdata(dev);
    variables_p variables = &self->variables;
#ifdef CONFIG_CAPSENSE_POWER_CONTROL_SUPPORT
    moto_data_p moto_data = &self->moto_data;

    if (!moto_data->cap_vdd_en){
        if ( power_on_chip(self) != 0){
            return -1;
        }
    }
#endif

    DEPRECATED_WRN(tcmd_reg_store);

    if( count != 7){
        LOG_ERR("Invalid count= %lu !=7", count);
        return -EINVAL;
    }

    for(i = 0 ; i < count ; i++){
        LOG_DBG("buf[%d] = 0x%X", i, buf[i]);
    }

    if(buf[6] == 0)
    {
        reg_addr = ((u16)buf[0]<<8) | (u16)buf[1];
        reg_val= ((u32)buf[2]<<24) | ((u32)buf[3]<<16) | ((u32)buf[4]<<8) | ((u32)buf[5]);

        if (smtc_i2c_write(self, reg_addr, reg_val) < 0)
            return -1;
    }
    else if(buf[6] == 1)
    {
        variables->tcmd_read_reg = ((u16)buf[0]<<8) | (u16)buf[1];
        variables->tcmd_read_flag = 1;
    }

    return count;
}

//=================================================================================================
// deprecated calibrate
//=================================================================================================
DEVICE_ATTR_SHOW(manual_calibrate)
{
    u32 reg_value = 0;
    Self self = dev_get_drvdata(dev);

    DEPRECATED_WRN(calibrate_show);
    LOG_DBG("Reading IRQSTAT_REG\n");
    smtc_i2c_read(self,REG_IRQ_SRC,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}

//=================================================================================================
DEVICE_ATTR_STORE(manual_calibrate)
{
    Self self = dev_get_drvdata(dev);
    DEPRECATED_WRN(calibrate_store);
    calibrate(self);
    return count;
}

//=================================================================================================
// fac_raw
//=================================================================================================
DEVICE_ATTR_SHOW(fac_raw)
{
    u16 reg_addr;
    u32 uData;
    int phid=0, count=0, ret=0;
    s32 diff;
    Self self = dev_get_drvdata(dev);
    reg_addr = REG_DIF_PH0;
    u8 data[NUM_PHASES*4] = {0};

    for(phid =0; phid<8; phid++)
    {
        ret = smtc_i2c_read(self, reg_addr+phid*4, &uData);
        if (ret < 0){
            LOG_ERR("Failed to read reg=0x%X, ret=%d", reg_addr+phid*4, ret);
            return -EIO;
        }
        LOG_INF("sx9377 phid=%d, reg_addr:0x%x", phid, reg_addr+phid*4);
        diff = (s32)uData>>10;
        data[4 * phid] = (u8)(diff >> 24);
        data[1 + 4 * phid] = (u8)(diff >> 16);
        data[2 + 4 * phid] = (u8)(diff >> 8);
        data[3 + 4 * phid] = (u8)(diff);
        LOG_INF("sx9377 diff=%x, data[%d]=%x, data[%d]=%x, data[%d]=%x, data[%d]=%x",
                diff,
                (4*phid), data[4 * phid],
                (1+4*phid), data[1 + 4 * phid],
                (2+4*phid), data[2 + 4 * phid],
                (3+4*phid), data[3 + 4 * phid]);
        count += snprintf(buf+count, PAGE_SIZE - count, "DIFF= %d\n", diff);
    }

    return count;
}

//=================================================================================================
// fac_enable
//=================================================================================================
DEVICE_ATTR_SHOW(fac_enable)
{
    Self self = dev_get_drvdata(dev);
    int ret = 0;
    u32 phen;
    bool is_enabled = false;

    ret = smtc_i2c_read(self, REG_PHEN, &phen);
    if (ret < 0){
        LOG_ERR("Failed to read phase enable reg=0x%X, ret=%d", REG_PHEN, ret);
        return -EIO;
    }
    is_enabled = (phen & 0xFF) ? true : false;
    return snprintf(buf, PAGE_SIZE, "%d\n", is_enabled);

}

//=================================================================================================
// fac_enable
//=================================================================================================
DEVICE_ATTR_STORE(fac_enable)
{
    Self self = dev_get_drvdata(dev);
    int ret = 0;
    int phid = 0;
    u32 phen;

    ret = smtc_i2c_read(self, REG_PHEN, &phen);
    if (ret < 0){
        LOG_ERR("Failed to read phase enable reg=0x%X, ret=%d", REG_PHEN, ret);
        return -EIO;
    }

    if ( !strncmp(buf, "1", 1)) {
        LOG_INF("enable cap sensor\n");
        phen |= self->main_phases;
        smtc_enable_phases(self, phen);
    } else if ( !strncmp(buf, "0", 1)) {
        LOG_INF("disnable cap sensor\n");
        phen &= ~(self->main_phases);
        smtc_enable_phases(self, phen);
        for(phid = 0; phid < NUM_PHASES; phid++) {
            phase_p phase = &self->phases[phid];
            if (phase->usage == MAIN && phase->input != NULL) {
                input_report_abs(phase->input, ABS_DISTANCE, -1);
                input_sync(phase->input);
            }
        }
    } else {
        LOG_ERR("Invalid command=%s", buf);
        return -EINVAL;
    }
    return count;
}

//=================================================================================================
// fac_detect
//=================================================================================================
DEVICE_ATTR_SHOW(fac_detect)
{
    Self self = dev_get_drvdata(dev);
    u32 chip_id = 0;
    int ret;
    ret = smtc_i2c_read(self, REG_WHOAMI, &chip_id);
    if(ret < 0){
        LOG_ERR("Failed to read chip id. ret= %d.", ret);
        return ret;
    }
    LOG_INF("Reading device id chip_id=%X", chip_id);
    if (((chip_id >> 12) & 0xFFF) == 0x937) {
        LOG_INF("Detect ic sx937x\n");
        return snprintf(buf, PAGE_SIZE, "%d\n", 1);
    }else{
        LOG_INF("Not found ic sx937x\n");
        return snprintf(buf, PAGE_SIZE, "%d\n", 0);
    }
}

//=================================================================================================
// chip_id
//=================================================================================================
DEVICE_ATTR_SHOW(chip_id)
{
    Self self = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%X\n", self->chip_id);
}

//=================================================================================================
static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR_RO(fac_raw);
static DEVICE_ATTR_RO(registers);
static DEVICE_ATTR_RO(fac_irq_state);
static DEVICE_ATTR_RW(fac_enable);
static DEVICE_ATTR_RO(fac_detect);
static DEVICE_ATTR_RO(fac_compensation);
static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RW(reg_read);
static DEVICE_ATTR_RW(reg_write);
static DEVICE_ATTR_RW(tcmd_reg);
static DEVICE_ATTR_WO(fac_cal);
static DEVICE_ATTR_RW(debug);
static DEVICE_ATTR_WO(reinitialize);

#ifdef CONFIG_CAPSENSE_HEADSET_STATE
static DEVICE_ATTR_WO(headset);
#endif

//deprecated
static DEVICE_ATTR_RW(reset);
static DEVICE_ATTR_RW(register_read);
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RW(manual_calibrate);

static struct attribute *capsense_dev_attrs[] = {
    &dev_attr_raw_data.attr,
    &dev_attr_fac_irq_state.attr,
    &dev_attr_fac_compensation.attr,
    &dev_attr_fac_detect.attr,
    &dev_attr_chip_id.attr,
    &dev_attr_fac_enable.attr,
    &dev_attr_fac_raw.attr,
    &dev_attr_registers.attr,
    &dev_attr_reg_read.attr,
    &dev_attr_reg_write.attr,
    &dev_attr_tcmd_reg.attr,
    &dev_attr_fac_cal.attr,
    &dev_attr_debug.attr,
    &dev_attr_reinitialize.attr,
#ifdef CONFIG_CAPSENSE_HEADSET_STATE
    &dev_attr_headset.attr,
#endif

    //deprecated
    &dev_attr_reset.attr,
    &dev_attr_register_read.attr,
    &dev_attr_register_write.attr,
    &dev_attr_reg.attr,
    &dev_attr_manual_calibrate.attr,

    NULL,
};
ATTRIBUTE_GROUPS(capsense_dev);

struct class capsense_class = {
    .name= "capsense",
};
static struct class *g_capsense_class_ptr = NULL;
static int g_capsense_refcount = 0;

static void process_touch_status(Self self)
{
    int phid;
    u32 prox_state = 0;
    bool need_sync = false, status_changed=false;
    phase_p phase;
    struct input_dev *input;
    char msg_updated_status[128];
    char msg_prox_status[128];
    int msg_len = 0;

    if (self->variables.cfg_forete_detected == 0){
        smtc_i2c_read(self, REG_PROX_STATUS, &prox_state);
    }else if (self->variables.cfg_forete_detected == 1){
        LOG_INF("Forete detected to PROX1");
        prox_state = 0xFF000000;
    }else{
        LOG_INF("Forete released");
        prox_state = 0x0;
    }

    LOG_DBG("prox_state= 0x%X", prox_state);
    log_raw_data(self);

    for (phid = 0; phid < NUM_PHASES; phid++)
    {
        phase = &self->phases[phid];
        if (phase->usage != MAIN){
            continue;
        }
        if (!phase->enabled){
            LOG_DBG("Skip disabled phasse=%s", phase->name);
            continue;
       }

        input = phase->input;
        need_sync = false;

        if (prox_state & phase->prox4_mask)
        {
            if (phase->state == PROX4){
                LOG_DBG("%s is PROX4 already", phase->name);
            }
            else{
                LOG_DBG("%s reports PROX4", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=4", phid);
                phase->state = PROX4;
                input_report_abs(input, ABS_DISTANCE, (int)PROX4);
                need_sync = true;
            }
        }
        else if (prox_state & phase->prox3_mask)
        {
            if (phase->state == PROX3){
                LOG_DBG("%s is PROX3 already", phase->name);
            }
            else{
                LOG_DBG("%s reports PROX3", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=3", phid);
                phase->state = PROX3;
                input_report_abs(input, ABS_DISTANCE, (int)PROX3);
                need_sync = true;
            }
        }
        else if (prox_state & phase->prox2_mask)
        {
            if (phase->state == PROX2){
                LOG_DBG("%s is PROX2 already", phase->name);
            }
            else{
                LOG_DBG("%s reports PROX2", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=2", phid);
                phase->state = PROX2;
                input_report_abs(input, ABS_DISTANCE, (int)PROX2);
                need_sync = true;
            }
        }
        else if (prox_state & phase->prox1_mask)
        {
            if (phase->state == PROX1){
                LOG_DBG("%s is PROX1 already", phase->name);
            }
            else{
                LOG_DBG("%s reports PROX1", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=1", phid);
                phase->state = PROX1;
                input_report_abs(input, ABS_DISTANCE, (int)PROX1);
                need_sync = true;
            }
        }else{
            if (phase->state == PROX0){
                LOG_DBG("%s is PROX0 already", phase->name);
            }else{
                LOG_DBG("%s reports PROX0", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=0", phid);
                phase->state = PROX0;
                input_report_abs(input, ABS_DISTANCE, (int)PROX0);
                need_sync = true;
            }
        }

        if (need_sync){
            input_sync(input);
            status_changed = true;
        }
    }

    if (!status_changed){
        LOG_INF("No proximity state is updated");
    }else{
        msg_len = 0;
        for (phid = 0; phid < NUM_PHASES; phid++)
        {
            phase = &self->phases[phid];
            if (phase->usage == MAIN){
                msg_len += sprintf(msg_prox_status+msg_len, " PH%d=%d", phid, phase->state);
            }
        }

        LOG_INF("Prox status= 0x%08X %s Updated:%s", prox_state, msg_prox_status, msg_updated_status);
    }
}

#define DSI_DISPALY_MAX_LEN 64
const char *get_dsi_display_name(void)
{
    const char *bootargs = NULL;
    char *end = NULL;
    char *idx = NULL;
    struct device_node *np;
    static char display[DSI_DISPALY_MAX_LEN] = {'\0'};

    if (display[0] != '\0')
        return display;

    np = of_find_node_by_path("/chosen");
    if (np == NULL)
        return NULL;

    if (of_property_read_string(np, "bootargs", &bootargs) != 0)
        goto putnode;

    idx = strstr(bootargs, "msm_drm.dsi_display0=");
    if (idx) {
        end = strpbrk(idx, " ");
        idx = strpbrk(idx, "=");
        if (idx && end > idx)
            strscpy(display, idx + 1, end - idx);
    }

    return display;

putnode:
    of_node_put(np);
    return NULL;
}

//=================================================================================================
#ifdef CONFIG_CAPSENSE_FLIP_CAL
static int read_flip_cali_regs(Self self, struct device_node *of_node,
    const char *dt_field, reg_val_p *out_regs)
{
    int ret, num_regs;
    int num_bytes;
    reg_val_p regs;

    num_regs = of_property_count_u32_elems(of_node, dt_field);
    LOG_DBG("number of registers= %d", self->num_dts_regs);
    if (num_regs <= 0 || num_regs % 2 != 0){
        LOG_ERR("Invalid reg_num= %d", num_regs);
        return -EINVAL;
    }

    num_regs /= 2;
    num_bytes = num_regs * sizeof(reg_val_t);
    regs = devm_kzalloc(self->dev, num_bytes, GFP_KERNEL);
    if (regs == NULL){
        LOG_ERR("Failed to alloc memory size=%d, num_reg= %d", num_bytes, num_regs);
        return -ENOMEM;
    }

    ret = of_property_read_u32_array(of_node, dt_field,
    (u32 *)regs,num_bytes / sizeof(u32));
    if (ret < 0) {
        LOG_ERR("Failed to parse dts %s, ret=%d", dt_field, ret);
        return -EINVAL;
    }

    *out_regs = regs;
    return num_regs;
}

static int parse_flip_dt_params(Self self, struct device_node *of_node)
{
    int ret;
    moto_data_p moto_data = &self->moto_data;
    moto_data->phone_flip_update_regs = false;

    ret = of_property_read_u32(of_node,"flip-gpio-when-open",
    &moto_data->phone_flip_open_val);
    if (ret){
        LOG_ERR("Failed to parse dts flip-gpio-when-open");
        return ret;
    }

    ret = read_flip_cali_regs(self, of_node, "flip-open-regs",
    &moto_data->flip_open_regs);
    if (ret<0){return ret;}
    moto_data->num_flip_open_regs = ret;

    ret = read_flip_cali_regs(self, of_node, "flip-closed-regs",
    &moto_data->flip_close_regs);
    if (ret<0){return ret;}
    moto_data->num_flip_closed_regs = ret;

    moto_data->phone_flip_update_regs = true;
    return 0;
}
#endif

static int main_map_ref_dts(Self self, struct device_node *of_node)
{
    int num_phases, phid;
    s8 main_phid, ref_phid;
    u32 main_map_ref[NUM_PHASES];

    for (phid=0; phid<NUM_PHASES; phid++){
        self->main_map_ref[phid] = -1;
    }

    num_phases = of_property_count_u32_elems(of_node, "phase-map");
    if (num_phases <= 0){
        LOG_ERR("Invalid dts item num: %d", num_phases);
        return -EINVAL;
    }
    if (num_phases % 2 != 0){
        LOG_ERR("Invalid dts item num: phase-map");
        return -EINVAL;
    }
    LOG_DBG("num_phases=%d", num_phases);

    if (of_property_read_u32_array(of_node, "phase-map",
        main_map_ref, num_phases))
    {
        LOG_ERR("Invalid dts item: phase-map");
        return -EINVAL;
    }

    for (phid=0; phid<num_phases/2; phid++)
    {
        main_phid = (s8)main_map_ref[phid*2];
        ref_phid  = (s8)main_map_ref[phid*2+1];
        LOG_DBG("main_phid=%d ref_phid=%d", main_phid, ref_phid);

        if (main_phid >= NUM_PHASES || (ref_phid != 0xFF && ref_phid >= NUM_PHASES)){
            LOG_ERR("Invalid dts item: main-map-ref main_phid=%d ref_phid=%d",
                main_phid, ref_phid);
            return -EINVAL;
        }
        self->main_phases |= 1 << main_phid;
        if (ref_phid != -1){
            self->ref_phases  |= 1 << ref_phid;
        }
        self->main_map_ref[main_phid] = ref_phid;
    }
    LOG_INF("main_phases=0x%X ref_phases=0x%X", self->main_phases, self->ref_phases);
    return 0;
}

static int parse_dts_power_supply(Self self, struct device_node *of_node)
{
    int ret = 0;
    power_supply_p power_supply = &self->power_supply;

    ret = of_property_read_u32(of_node,
    "semtech,power-supply-type",&power_supply->type);
    if(ret < 0){
        power_supply->type = PMIC_LDO;
        ret = 0;
        LOG_INF("pmic ldo is the default if not set power-supply-type in dt");
    }

    switch(power_supply->type)
    {
        case PMIC_LDO:
            /* using regulator_get() to fetch power_supply in sx9377_probe()*/
            break;
        case ALWAYS_ON:
            /* power supply always on: no need fetch others control  in drivers */
            break;
        case EXTERNAL_LDO:
            /* parse the gpio number for external LDO enable pin*/

            power_supply->eldo_gpio = of_get_named_gpio(of_node,
            "semtech,eldo-gpio",0);
            LOG_DBG("use ELDO gpio= %d", power_supply->eldo_gpio);
        break;

        default:
            LOG_ERR("Invalid power_supply_type= %d", power_supply->type);
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int get_init_reg_group(Self self, struct device_node *of_node,
    const char **reg_group_name)
{
    int i, num_panels;
    const char *this_panel, *target_panel;

    //Get LCD panel. Different panel might need different register settings
    target_panel = get_dsi_display_name();
    /*
    if three's no "support-panel-num" in dts or
    there's no matched panel_name(maybe it is a bare board),
    the default reg_group_name will be "Semtech,reg-init"
    */
    if(of_property_read_u32(of_node,
    "support-panel-num",&num_panels))
    {
        LOG_DBG("multi panels isn't supported, use the default reg init");
        *reg_group_name = "semtech,reg-init";
        return 0;
    }

    LOG_DBG("support_panel_num= %d", num_panels);
    for (i = 0; i < num_panels; i++)
    {
        if(of_property_read_string_index(of_node,
        "support-panel-names", i, &this_panel))
        {
            LOG_ERR("Failed to get support-panel-names");
            return -EINVAL;
        }

        if(strstr(target_panel, this_panel))
        {
            if(of_property_read_string_index(of_node,
            "reg-groups-names", i, reg_group_name))
            {
                LOG_ERR("Failed to get reg-groups-names");
                return -EINVAL;
            }
            LOG_INF("dsi display matched, index= %d, support dsi= %s, reg group= %s",
                i, this_panel, *reg_group_name);
            break;
        }
    }

    return 0;
}

static int parse_init_regs(Self self, struct device_node *of_node,
    const char *reg_group_name)
{
    self->num_dts_regs = of_property_count_u32_elems(of_node, reg_group_name);
    LOG_DBG("number of registers= %d", self->num_dts_regs);
    if (self->num_dts_regs <= 0 || self->num_dts_regs %2 != 0){
        LOG_ERR("Invalid reg_num= %d, reg_group_name=%s, please check dts config",
            self->num_dts_regs, reg_group_name);
        return -EINVAL;
    }
    self->num_dts_regs /= 2;
    self->dts_regs = devm_kzalloc(self->dev,
        sizeof(reg_val_t)*self->num_dts_regs, GFP_KERNEL);
    if (!self->dts_regs){
        LOG_ERR("Failed to alloc memory, num_reg= %d", self->num_dts_regs);
        return -ENOMEM;
    }

    if (of_property_read_u32_array(of_node, reg_group_name,
        (u32*)self->dts_regs,
        sizeof(reg_val_t) * self->num_dts_regs/sizeof(u32)))
    {
        LOG_ERR("Failed to load registers from the dts");
        return -EINVAL;
    }

    return 0;
}

#ifdef CONFIG_CAPSENSE_HEADSET_STATE
static int parse_headset_opts(Self self, struct device_node *of_node)
{
    int ret;
    moto_data_p moto_data = &self->moto_data;

    ret = of_property_read_u32(of_node, "semtech,headset-reg-num",
    &moto_data->headset_operate_reg_num);
    if (ret<0){
        LOG_WRN("Failed to read semtech,headset-reg-num");
        moto_data->headset_operate_reg_num = 0;
        return 0;
    }

    LOG_DBG("headset_operate_reg_num= %d", moto_data->headset_operate_reg_num);
    if (moto_data->headset_operate_reg_num <= 0){
        LOG_WRN("headset_operate_reg_num=%d", moto_data->headset_operate_reg_num);
        return 0;
    }

    moto_data->headset_operate_reg = devm_kzalloc(self->dev,
    sizeof(reg_val_t)*moto_data->headset_operate_reg_num,
    GFP_KERNEL);

    if (moto_data->headset_operate_reg == NULL){
        LOG_ERR("Failed to alloc memory for headset_operate_reg, headset-reg-num= %d",
            moto_data->headset_operate_reg_num);
        return -ENOMEM;
    }

    moto_data->headset_operate_reg_bck = devm_kzalloc(self->dev,
    sizeof(reg_val_t)*moto_data->headset_operate_reg_num,
    GFP_KERNEL);
    if (moto_data->headset_operate_reg_bck == NULL){
        LOG_ERR("Failed to alloc memory for headset_operate_reg_bck, headset-reg-num= %d",
            moto_data->headset_operate_reg_num);
        return -ENOMEM;
    }

    if (of_property_read_u32_array(of_node, "semtech,headset-reg",
        (u32*)&(moto_data->headset_operate_reg[0]),
        sizeof(reg_val_t)*moto_data->headset_operate_reg_num/sizeof(u32)))
    {
        LOG_ERR("Failed to read semtech,headset-reg");
        return -EINVAL;
    }
    if (of_property_read_u32_array(of_node,
        "semtech,headset-reg",(u32*)&(moto_data->headset_operate_reg_bck[0]),
        sizeof(reg_val_t)*moto_data->headset_operate_reg_num/sizeof(u32)))
    {
        LOG_ERR("Failed to get semtech,headset-reg");
        return -EINVAL;
    }

    return 0;
}
#endif

static int parse_dts(Self self)
{
    int ret;
    const char *reg_group_name;
    u32 dev_id;
    variables_p variables = &self->variables;
    struct device_node *of_node = self->client->dev.of_node;

    if (of_node == NULL){
        LOG_ERR("of_node is NULL");
        return -EINVAL;
    }

    ret = parse_dts_power_supply(self, of_node);
    if (ret < 0){return ret;}

    ret = main_map_ref_dts(self, of_node);
    if (ret < 0){return ret;}

    ret = get_init_reg_group(self, of_node, &reg_group_name);
    if (ret < 0){return ret;}

    ret = parse_init_regs(self, of_node, reg_group_name);
    if (ret < 0){return ret;}

#ifdef CONFIG_CAPSENSE_HEADSET_STATE
    ret = parse_headset_opts(self, of_node);
    if (ret < 0){return ret;}
#endif

#ifdef CONFIG_CAPSENSE_FLIP_CAL
    ret = parse_flip_dt_params(self, of_node);
    if (ret < 0){return ret;}
#endif

    variables->esd_reinit_on = of_property_read_bool(of_node, "esd-reinit-on");
    LOG_DBG("cfg_esd_recovery= %d", variables->esd_reinit_on);

    if(of_property_read_u32(of_node, "reg-ver", &self->reg_ver)){
        LOG_ERR("Failed to read reg-ver from dts.");
        return -EINVAL;
    }
    LOG_DBG("reg_ver= %d", self->reg_ver);

    if(of_property_read_u32(of_node, "chip-id", &self->chip_id)){
        LOG_ERR("Failed to read chip-id from dts.");
        return -EINVAL;
    }
    LOG_DBG("chip_id= 0x%X", self->chip_id);

    if (of_property_read_u32(of_node, "semtech,id", &dev_id)) {
        LOG_INF("Failed to read semtech,id from dts, default to 0");
        self->id = 0;
    } else {
        self->id = dev_id;
    }
    LOG_DBG("device id= %d", self->id);

    return 0;
}

static int init_irq_gpio(Self self)
{
    self->gpio = devm_gpiod_get(self->dev, "irq", GPIOD_IN);

    if (IS_ERR(self->gpio)){
        /*
        add below properity in the dts.
        irq-gpios = <&gpio 4 0>;

        `gpio` is platform-dependent.
        `4 ` is the gpio to which the irq pin connected
        */
        LOG_ERR("Failed to get irq GPIO. Please check DTS stup.");
        return PTR_ERR(self->gpio);
    }

    return 0;
}

static int capsensor_set_enable(struct sensors_classdev *sensors_cdev,
    unsigned int enable)
{
    int ret;
    bool all_main_disabled = true;
    int phid = 0;
    u32 phen;
        phase_p phase_c = container_of(sensors_cdev, phase_t, sensor_class);
        Self self = phase_c->self;

    for (phid = 0; phid < NUM_PHASES; phid++)
    {
            if (&self->phases[phid] == phase_c) {
                break;
            }
        }
        if (phid < NUM_PHASES) {
            ret = smtc_i2c_read(self, REG_PHEN, &phen);
            if (ret < 0){
                LOG_ERR("Failed to read phase enable reg=0x%X, ret=%d", REG_PHEN, ret);
                return -EIO;
            }

            if (enable == 1) {
                LOG_INF("enable cap sensor=%s phid=%d", sensors_cdev->name, phid);
                phen |= 0x7F;
                //enable a phase will trigger a auto compensation
                //status will be updated by the compensation irq
                smtc_enable_phases(self, phen);
            }
            else if (enable == 0) {
                LOG_INF("disable cap sensor=%s phid=%d", sensors_cdev->name, phid);
                phen &= ~(1<<phid);
                smtc_enable_phases(self, phen);
                    input_report_abs(phase_c->input, ABS_DISTANCE, -1);
                    input_sync(phase_c->input);
            } else {
                LOG_ERR("Invalid command=%d", enable);
                return -EINVAL;
            }
    }

    //if all chs disabled, then disable all. The main purpose is for disabling REF phase
    for (phid = 0; phid < NUM_PHASES; phid++)
    {
            if (self->phases[phid].usage == MAIN && self->phases[phid].enabled) {
            all_main_disabled = false;
            break;
        }
    }

    if (all_main_disabled) {
        LOG_INF("disable all phases");
        phen &= 0xFFFFFF00;
        smtc_enable_phases(self, phen);
    }
    return 0;
}

static void smtc_update_status(Self self)
{
    u32 irq_src=0;
    int ret=0, irq_bit=0;

    ret = read_and_clear_irq(self, &irq_src);
    if (ret){
        LOG_ERR("Failed to read irq source. ret=%d. Disable the IRQ=%d", ret, self->irq_id);
        /* We are using low level to trigger the irq.
        IRQ will be triggered all the time when failed to clear the IRQ, which will slow down the system.
        ESD recover worker will enable it when it is recovered.
        */
        disable_irq_nosync(self->irq_id);
        self->irq_disabled = 1;

        goto SUB_OUT;
    }

    for (irq_bit=0; irq_bit<NUM_IRQ_BITS; irq_bit++)
    {
        if (irq_src >> irq_bit & 0x1)
        {
            if(self->irq_handler[irq_bit]){
                //call to smtc_process_touch_status() or smtc_log_raw_data()
                self->irq_handler[irq_bit](self);
            }else{
                LOG_ERR("No handler to IRQ bit= %d", irq_bit);
            }
        }
    }

    if (irq_src == 0){
        LOG_INF("Force to update touch status");
        process_touch_status(self);
    }

SUB_OUT:
#if CFG_STAY_AWAKE && !WAKE_TMOUT
    LOG_DBG("Release wake lock");
    __pm_relax(self->wake_lock);
#endif

    LOG_DBG("Exit");
    return;
}

static irqreturn_t smtc_irq_isr(int irq, void *pvoid)
{
    Self self = (Self)pvoid;
    LOG_DBG("received IRQ= %d", irq);

    if (!is_irq_pin_low(self))
    {
        LOG_ERR("GPIO=%d must be low when an IRQ is received.", desc_to_gpio(self->gpio));
        read_and_clear_irq(self, NULL);
        return IRQ_HANDLED;
    }

#if CFG_STAY_AWAKE
#if WAKE_TMOUT
    LOG_DBG("Stay awake for %d seconds", WAKE_TMOUT);
    __pm_wakeup_event(self->wake_lock, WAKE_TMOUT);
#else
    LOG_DBG("Stay awake");
    __pm_stay_awake(self->wake_lock);
#endif
#endif

    LOG_DBG("Update status with thread IRQ");
    smtc_update_status(self);

    return IRQ_HANDLED;
}



//#################################################################################################
#if CFG_ESD_RECOVERY

static void vdd_power_off_on(Self self, bool on)
{
    int ret = 0;
    power_supply_p power_supply = &self->power_supply;

    if(!power_supply->eldo_vdd_en){
        LOG_WRN("Power on/off only supported by ELDO gpio");
        return;
    }

    LOG_INF("Power %s with ELDO gpio", on ? "on":"off");
    ret = gpio_direction_output(power_supply->eldo_gpio, on);

    if(ret < 0){
        LOG_ERR("Failed to ELDO gpio=%d to value=%d ret=%d", power_supply->eldo_gpio, on, ret);
    }
}

static bool check_i2c_error(Self self)
{
    //return true when an error has been detected and hanlded, else false
    u32 irq_mask;
    power_supply_p power_supply = &self->power_supply;

    if (smtc_i2c_read(self, REG_IRQ_MASK, &irq_mask) < 0)
    {
        LOG_ERR("Failed to read I2C");
        if (!power_supply->eldo_vdd_en)
        {
            LOG_ERR("I2C error can only be recovered by the controlable VDD");
            if (self->esd_check_interval < 60*60){
                LOG_DBG("Increace the check interval by %d seconds", ESD_CHECK_INTERVAL_NOR);
                self->esd_check_interval += ESD_CHECK_INTERVAL_NOR;
            }
            return true;
        }

        if(++self->i2c_error_times < 2){
            LOG_DBG("Change the check interval to %d seconds", ESD_CHECK_INTERVAL_ERR);
            self->esd_check_interval = ESD_CHECK_INTERVAL_ERR;
            return true;
        }

        //this error can only be recovered by power off/on
        LOG_WRN("I2C error, power off/on SAR sensor");
        vdd_power_off_on(self, false);
        msleep(10);
        vdd_power_off_on(self, true);
        msleep(10);

        reset_and_init_chip_esd(self);
        if (self->esd_check_interval < 60*60){
            LOG_DBG("Increace the check interval by %d seconds", ESD_CHECK_INTERVAL_NOR);
            self->esd_check_interval += ESD_CHECK_INTERVAL_NOR;
        }

        return true;
    }

    //default value of 0x4004 is 0x60 and usually will be set to 0x70 in the dts
    if (irq_mask == 0x60)
    {
        LOG_WRN("Register= 0x%X has been reset to default value=0x%X.", REG_IRQ_MASK, irq_mask);
        reset_and_init_chip_esd(self);

        if (self->esd_check_interval < 60*60){
            LOG_DBG("Increace the check interval by %d seconds", ESD_CHECK_INTERVAL_NOR);
            self->esd_check_interval += ESD_CHECK_INTERVAL_NOR;
        }
        return true;
    }

    self->i2c_error_times = 0;
    return false;
}

static void smtc_esd_recover(Self self)
{
    int phid=0, idx=0, num_same_val;
    u32 reg_val, phen, tmp_u32;
    LOG_DBG("Checking ESD failure");

    if (check_i2c_error(self)){
        return;
    }

    phen = self->variables.phen_reg_val;
    //update useful of each enabled phase
    for(phid=0; phid<NUM_PHASES; phid++)
    {
        if(phen & 1<<phid){
            smtc_i2c_read(self, REG_USE_PH0 + phid*4, &reg_val);
            self->ph_useful[phid][self->useful_update_idx] = reg_val;
        }
    }
    self->useful_update_idx = (self->useful_update_idx + 1) % ESD_FAIL_CHECK_TIMES;

    //reset if any phase read the same value by ESD_FAIL_CHECK_TIMES
    for(phid=0; phid<NUM_PHASES; phid++)
    {
        num_same_val = 0;

        if(phen & 1<<phid)
        {
            if (ESD_FAIL_CHECK_TIMES == 3){
                LOG_DBG("ph%d useful=[0x%X, 0x%X, 0x%X]",
                    phid, self->ph_useful[phid][0], self->ph_useful[phid][1], self->ph_useful[phid][2]);
            }else{
                LOG_DBG("ph%d useful=[0x%X, 0x%X, 0x%X, ...]",
                    phid, self->ph_useful[phid][0], self->ph_useful[phid][1], self->ph_useful[phid][2]);
             }

            for(idx=1; idx<ESD_FAIL_CHECK_TIMES; idx++)
            {
                tmp_u32 = self->ph_useful[phid][ESD_FAIL_CHECK_TIMES-1];
                if (tmp_u32 == SATURATED_USEFUL){
                    //Usually happend when gripping the antenna tightly
                    LOG_DBG("ph=%d useful=0x%X is saturated, skip checking this phase",
                        phid, tmp_u32);
                    break;
                }
                if (tmp_u32 == 0){
                    //Usually happend when phase is not enabled
                    LOG_DBG("ph=%d useful=0, skip checking this phase", phid);
                    break;
                }

                if(self->ph_useful[phid][idx-1] == self->ph_useful[phid][idx])
                {
                    if (ESD_FAIL_CHECK_TIMES == 3){
                        LOG_INF("ph%d useful=[0x%X, 0x%X, 0x%X]",
                            phid, self->ph_useful[phid][0], self->ph_useful[phid][1], self->ph_useful[phid][2]);
                    }else{
                        LOG_INF("ph%d useful=[0x%X, 0x%X, 0x%X, ...]",
                            phid, self->ph_useful[phid][0], self->ph_useful[phid][1], self->ph_useful[phid][2]);
                    }

                    if(++num_same_val >= ESD_FAIL_CHECK_TIMES-1)
                    {
                        LOG_WRN("Detected number of %d same useful values", ESD_FAIL_CHECK_TIMES);
                        reset_and_init_chip_esd(self);

                        LOG_DBG("Increace the check interval by %d seconds", ESD_CHECK_INTERVAL_NOR);
                        self->esd_check_interval += ESD_CHECK_INTERVAL_NOR;
                        return;
                    }
                }
            }
        }
    }

    if (self->irq_disabled){
        LOG_INF("Enable the irq=%d", self->irq_id);
        enable_irq(self->irq_id);
        self->irq_disabled = 0;
    }

    self->esd_check_interval = ESD_CHECK_INTERVAL_NOR;
    LOG_DBG("ESD check PASSED.");
}

static void esd_worker_func(struct work_struct *work)
{
    Self self = container_of(work, self_t, esd_worker.work);

    LOG_DBG("Enter");
    smtc_esd_recover(self);

    LOG_DBG("The next check will be performed in %d seconds",
        self->esd_check_interval);
    schedule_delayed_work(&self->esd_worker,
        msecs_to_jiffies(self->esd_check_interval*1000));
}
#endif //CFG_ESD_RECOVERY

static void init_variables(Self self)
{
    int i;
    self->chip_id = 0x9377;
    self->reg_ver = -1;
    mutex_init(&self->phen_lock);
    self->irq_disabled = 0;
    self->dbg_flag.log_hex_data = false;
    self->esd_check_interval = ESD_CHECK_INTERVAL_NOR;

    memcpy(self->phases, smtc_phase_table_template, sizeof(self->phases));
    for (i = 0; i < NUM_PHASES; i++) {
        self->phases[i].self = self;
    }

    //refer to register 0x4000
    self->irq_handler[0] = 0; /* UNUSED */
    self->irq_handler[1] = 0; /* UNUSED */
    self->irq_handler[2] = 0; /* UNUSED */
    self->irq_handler[3] = log_raw_data;    //CONVEDONEIRQ
    self->irq_handler[4] = process_touch_status; //COMPDONEIRQ
    self->irq_handler[5] = process_touch_status; //FARANYIRQ
    self->irq_handler[6] = process_touch_status; //CLOSEANYIRQ
    self->irq_handler[7] = 0;   //RESET_STAT
}

static int init_power_supply(Self self)
{
    int ret = 0;
    power_supply_p power_supply = &self->power_supply;

    switch(power_supply->type)
    {
    case PMIC_LDO:
        power_supply->cap_vdd = regulator_get(self->dev, "cap_vdd");
        if (IS_ERR(power_supply->cap_vdd))
        {
            if (PTR_ERR(power_supply->cap_vdd) == -EPROBE_DEFER) {
                return PTR_ERR(power_supply->cap_vdd);
            }
            LOG_INF("Failed to get regulator");
            return -ENODEV;
        }

        LOG_INF("power on with cap_vdd");
        ret = regulator_enable(power_supply->cap_vdd);
        if (ret) {
            regulator_put(power_supply->cap_vdd);
            LOG_ERR("Failed to enable regulator, ret=%d", ret);
            return ret;
        }
        power_supply->cap_vdd_en = true;
        LOG_INF("cap_vdd regulator is %s",
            regulator_is_enabled(power_supply->cap_vdd) ? "on" : "off");
        msleep(10);
        break;
    case ALWAYS_ON:
        LOG_INF("using always on power supply");
        break;
    case EXTERNAL_LDO:
        LOG_DBG("enable ELDO, en_gpio= %d", power_supply->eldo_gpio);
        ret = gpio_request(power_supply->eldo_gpio, "sx9377_eldo_gpio");
        if (ret < 0){
            LOG_ERR("Failed to request ELDO gpio, ret= %d", ret);
            return ret;
        }
        ret = gpio_direction_output(power_supply->eldo_gpio,1);
        if(ret < 0){
            LOG_ERR("Failed to enable ELDO, ret= %d", ret);
            return ret;
        }
        power_supply->eldo_vdd_en = true;
        msleep(20);
        break;
    }

    return 0;
}

static int create_sys_nodes(Self self)
{
    int ret = 0, phid;
    struct input_dev *input = NULL;

    if (!g_capsense_class_ptr) {
        ret = class_register(&capsense_class);
        if (ret < 0) {
            LOG_ERR("Failed to register capsense class");
            return ret;
        }
        g_capsense_class_ptr = &capsense_class;
    }

    self->class_dev = device_create_with_groups(g_capsense_class_ptr, NULL,
            MKDEV(0, 0), self, capsense_dev_groups, "capsense%d", self->id);
    if (IS_ERR(self->class_dev)) {
        LOG_ERR("Failed to create sys class device");
        if (g_capsense_refcount <= 0) {
            class_unregister(g_capsense_class_ptr);
            g_capsense_class_ptr = NULL;
        }
        return PTR_ERR(self->class_dev);
    }
    g_capsense_refcount++;

    for (phid = 0; phid < NUM_PHASES; phid++)
    {
        phase_p phase = &self->phases[phid];

        //main phases were specified in the dts
        if ((self->main_phases & 1<<phid) == 0)
        {
            if (self->ref_phases & 1<<phid){
                phase->usage = REF;
            }
            continue;
        }

        input = devm_input_allocate_device(self->dev);
        if (!input){
            LOG_ERR("Failed to create input device %s.", phase->name);
            ret = -ENOMEM;
            goto FREE_INPUTS;
        }

        input->name = phase->name;
        input->id.bustype = BUS_I2C;
        input_set_abs_params(input, ABS_DISTANCE, 0, 4, 0, 0);
        ret = input_register_device(input);
        if (ret){
            LOG_ERR("Failed to register input device=%s. ret=%d", phase->name, ret);
            goto FREE_INPUTS;
        }

        phase->input = input;
        phase->usage = MAIN;
        phase->state = PROX0;

        phase->sensor_class.sensors_enable = capsensor_set_enable;
        phase->sensor_class.sensors_poll_delay = NULL;
        phase->sensor_class.name = phase->name;
        phase->sensor_class.vendor = "semtech";
        phase->sensor_class.version = 1;
        phase->sensor_class.type = SENSOR_TYPE_MOTO_CAPSENSE;
        phase->sensor_class.max_range = "5";
        phase->sensor_class.resolution = "5.0";
        phase->sensor_class.sensor_power = "3";
        phase->sensor_class.min_delay = 0;
        phase->sensor_class.fifo_reserved_event_count = 0;
        phase->sensor_class.fifo_max_event_count = 0;
        phase->sensor_class.delay_msec = 100;
        phase->sensor_class.enabled = 0;

        ret = sensors_classdev_register(&input->dev, &phase->sensor_class);
        if (ret < 0){
            LOG_ERR("Failed to create cap sensor_class for %s, ret=%d", phase->name, ret);
            goto FREE_INPUTS;
        }
    }
    return 0;

FREE_INPUTS:
    for (phid=0; phid<NUM_PHASES; phid++)
    {
        if (self->phases[phid].input){
            input_unregister_device(self->phases[phid].input);
        }
    }
    device_unregister(self->class_dev);
    g_capsense_refcount--;
    if (g_capsense_class_ptr && g_capsense_refcount <= 0) {
        class_unregister(g_capsense_class_ptr);
        g_capsense_class_ptr = NULL;
    }

    return ret;
}

#ifdef CONFIG_CAPSENSE_USB_CAL
static int init_auto_cali(Self self)
{
    int ret=0;
    struct power_supply *psy = NULL;
    moto_data_p moto_data = &self->moto_data;

    INIT_WORK(&moto_data->ps_notify_work, ps_notify_callback_work);
    moto_data->ps_notif.notifier_call = ps_notify_callback;
    ret = power_supply_reg_notifier(&moto_data->ps_notif);
    if (ret){
        LOG_ERR("Failed to register ps_notifier, ret=%d", ret);
        return ret;
    }

    psy = power_supply_get_by_name("usb");
    if (psy == NULL) {
        LOG_ERR("Failed to get power supply");
        return -ENODEV;
    }
    ret = ps_get_state(psy, &moto_data->ps_is_present);
    if (ret) {
        LOG_ERR("Failed to get power supply property failed ret=%d", ret);
        goto FREE_NOTIFIER;
    }

#ifdef CONFIG_CAPSENSE_FLIP_CAL
    if (!of_property_read_bool(self->dev->of_node, "extcon"))
    {
        LOG_ERR("extcon is not in the dts");
        ret = -EINVAL;
        goto FREE_NOTIFIER;
    }

    moto_data->flip_notif.notifier_call = flip_notify_callback;
    moto_data->ext_flip_det = extcon_get_edev_by_phandle(self->dev, 0);

    if (IS_ERR(moto_data->ext_flip_det)) {
        LOG_ERR("Failed to get extcon flip dev");
        ret = -ENODEV;
        goto FREE_NOTIFIER;
    }

    if(extcon_register_notifier(moto_data->ext_flip_det,
        EXTCON_MECHANICAL, &moto_data->flip_notif))
    {
        LOG_ERR("failed to register extcon flip dev notifier");
        ret = -EINVAL;
        goto FREE_NOTIFIER;
    }

    moto_data->phone_flip_state =
        extcon_get_state(moto_data->ext_flip_det, EXTCON_MECHANICAL);
    update_flip_regs(self, moto_data->phone_flip_state);

#endif //CONFIG_CAPSENSE_FLIP_CAL
    return 0;

FREE_NOTIFIER:
    power_supply_unreg_notifier(&moto_data->ps_notif);
    return ret;

}
#endif //CONFIG_CAPSENSE_USB_CAL

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int sx9377_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int sx9377_probe(struct i2c_client *client)
#endif
{
    int phid;
    int ret = 0;
    Self self = 0;
    phase_p phase;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    power_supply_p power_supply;

    LOG_INF("Enter");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
    {
        LOG_ERR("Failed to check i2c functionality.");
        return -EIO;
    }

    self = devm_kzalloc(&client->dev,sizeof(self_t), GFP_KERNEL);
    if (!self){
        LOG_ERR("Failed to create self, size= %zu", sizeof(self_t));
        return -ENOMEM;
    }

    init_variables(self);
    i2c_set_clientdata(client, self);
    client->dev.platform_data = self;
    self->client = client;
    self->dev = &client->dev;
    self->irq_id = client->irq;

    ret = parse_dts(self);
    if (ret){
        LOG_ERR("Failed to parse_dts");
        return -EINVAL;
    }

    ret = init_power_supply(self);
    if (ret){
        LOG_ERR("Failed to init_power_supply");
        return ret;
    }
    ret = check_hardware(self);
    if (ret) {
        LOG_ERR("Failed to check hardware");
        goto FREE_PMIC;
    }

    ret = reset_and_init_chip_probe(self);
    if (ret) {
        LOG_ERR("Failed to init registers");
        goto FREE_PMIC;
    }
    ret = init_irq_gpio(self);
    if (ret) {
        LOG_ERR("Failed to init_irq_gpio");
        goto FREE_PMIC;
    }
    ret = create_sys_nodes(self);
    if (ret) {
        LOG_ERR("Failed to create_sys_nodes");
        goto FREE_PMIC;
    }

    self->irq_disabled = 0;
    ret = request_threaded_irq(self->irq_id, NULL, smtc_irq_isr,
    IRQF_TRIGGER_LOW | IRQF_ONESHOT,
    self->dev->driver->name, self);
    if (ret){
        LOG_ERR("Failed to request irq= %d", self->irq_id);
        goto FREE_SYS_NODES;
    }
    LOG_INF("request_threaded_irq= %d", self->irq_id);

    INIT_DELAYED_WORK(&self->esd_worker, esd_worker_func);
    if(self->variables.esd_reinit_on) {
        schedule_delayed_work(&self->esd_worker,
        msecs_to_jiffies(ESD_CHECK_INTERVAL_NOR*1000));
    }

#ifdef CONFIG_CAPSENSE_USB_CAL
    init_auto_cali(self);
#endif

    LOG_INF("Passed to load driver version=%s register version=%d",
        MODULE_VER, self->reg_ver);
    return 0;

FREE_SYS_NODES:
    LOG_DBG("FREE_SYS_NODES");

    for (phid=0; phid<NUM_PHASES; phid++)
    {
            phase = &self->phases[phid];
        if (phase->usage == MAIN){
            LOG_DBG("%s", phase->name);
            if (!IS_ERR_OR_NULL(phase->sensor_class.dev))
                sensors_classdev_unregister(&phase->sensor_class);
            input_unregister_device(phase->input);
        }
    }
        device_unregister(self->class_dev);

FREE_PMIC:
    LOG_DBG("FREE_PMIC");
    power_supply = &self->power_supply;
    if (power_supply->cap_vdd_en) {
        regulator_disable(power_supply->cap_vdd);
        regulator_put(power_supply->cap_vdd);
    }

    if(power_supply->eldo_vdd_en){
        gpio_direction_output(power_supply->eldo_gpio,0);
    }
    LOG_ERR("Failed to load semtech sx9377 sar driver");
    return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
static void sx9377_remove(struct i2c_client *client)
#else
static int sx9377_remove(struct i2c_client *client)
#endif
{
    int phid;
    phase_p phase;
    Self self = i2c_get_clientdata(client);
    power_supply_p power_supply = &self->power_supply;
#ifdef CONFIG_CAPSENSE_USB_CAL
    moto_data_p moto_data = &self->moto_data;
#endif
    LOG_INF("Enter");

    free_irq(self->irq_id, self);
    smtc_i2c_write(self, REG_PHEN, 0x0);
    read_and_clear_irq(self, NULL);

    if (self->variables.esd_reinit_on){
        cancel_delayed_work_sync(&self->esd_worker);
    }

#ifdef CONFIG_CAPSENSE_USB_CAL
    cancel_work_sync(&moto_data->ps_notify_work);
    power_supply_unreg_notifier(&moto_data->ps_notif);
#endif

    for (phid = 0; phid <NUM_PHASES; phid++)
    {
        phase = &self->phases[phid];
        if (phase->input){
            if (!IS_ERR_OR_NULL(phase->sensor_class.dev))
                sensors_classdev_unregister(&phase->sensor_class);
            input_unregister_device(phase->input);
        }
    }

    device_unregister(self->class_dev);
    g_capsense_refcount--;
    if (g_capsense_class_ptr && g_capsense_refcount <= 0) {
        class_unregister(g_capsense_class_ptr);
        g_capsense_class_ptr = NULL;
    }

    if (power_supply->cap_vdd_en) {
        regulator_disable(power_supply->cap_vdd);
        regulator_put(power_supply->cap_vdd);
    }

    if(power_supply->eldo_vdd_en){
        gpio_direction_output(power_supply->eldo_gpio,0);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,1,0)
    return 0;
#endif
}

static int sx9377_suspend(struct device *dev)
{
    Self self = dev_get_drvdata(dev);

    if (self->variables.esd_reinit_on){
        //If we happen to reinitialize during suspend we might fail so wait for it to end
        cancel_delayed_work_sync(&self->esd_worker);
    }

    disable_irq(self->irq_id);
    //don't raise IRQ during suspend
    smtc_i2c_write(self, REG_IRQ_MASK, 0x0);
    //release IRQ pin to avoid leakage of current
    read_and_clear_irq(self, NULL);

    return 0;
}

static int sx9377_resume(struct device *dev)
{
    Self self = dev_get_drvdata(dev);

    smtc_i2c_write(self, REG_IRQ_MASK, self->variables.irq_mask);
    enable_irq(self->irq_id);
    smtc_update_status(self);

    if (self->variables.esd_reinit_on){
        schedule_delayed_work(&self->esd_worker,
            msecs_to_jiffies(ESD_CHECK_INTERVAL_ERR*1000));
    }

    return 0;
}

static struct i2c_device_id sx9377_idtable[] =
{
    { SMTC_DRIVER_NAME, 0 },
    { "sx9377", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx9377_idtable);

static struct of_device_id sx9377_match_table[] =
{
    { .compatible = "semtech,sx9377",},
    { },
};

static const struct dev_pm_ops sx9377_pm_ops =
{
    .suspend = sx9377_suspend,
    .resume = sx9377_resume,
};
static struct i2c_driver sx9377_driver =
{
    .driver = {
        .owner            = THIS_MODULE,
        .name            = SMTC_DRIVER_NAME,
        .of_match_table    = sx9377_match_table,
        .pm                = &sx9377_pm_ops,
    },
    .id_table        = sx9377_idtable,
    .probe            = sx9377_probe,
    .remove            = sx9377_remove,
};
static int __init sx9377_I2C_init(void)
{
    return i2c_add_driver(&sx9377_driver);
}
static void __exit sx9377_I2C_exit(void)
{
    i2c_del_driver(&sx9377_driver);
}

module_init(sx9377_I2C_init);
module_exit(sx9377_I2C_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9377 Capacitive Proximity Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MODULE_VER);
