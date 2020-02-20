#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#define DEFAULT_STEP_FREQ 2400
#define MOTOR_CLASS_NAME  "aw8646"
#define MOTOR_CONTROL  "control"
#define MOTOR_DEFAULT_EXPIRE 750 //750ms clock time.
#define MOTOR_HW_CLK         9600 //9.6KHz clock
#define MOTOR_HW_CLK_NAME "gcc_gp3"

enum gpios_index {
    MOTOR_POWER_EN = 0,
    MOTOR_FAULT_INT,
#ifndef CONFIG_USE_HW_CLK
    MOTOR_STEP,
#endif
    MOTOR_HW_STEP,
    MOTOR_DIR,
    MOTOR_MODE1,
    MOTOR_MODE2,
    MOTOR_EN,
    MOTOR_SLEEP,
    MOTOR_T0,
    MOTOR_T1,
    MOTOR_UNKNOWN
};

static const char* const gpios_labels[] = {
    "MOTOR_POWER_EN",
    "MOTOR_FAULT_INT",
#ifndef CONFIG_USE_HW_CLK
    "MOTOR_STEP",
#endif
    "MOTOR_HW_STEP",
    "MOTOR_DIR",
    "MOTOR_MODE1",
    "MOTOR_MODE2",
    "MOTOR_EN",
    "MOTOR_SLEEP",
    "MOTOR_T0",
    "MOTOR_T1",
    "MOTOR_UNKNOWN"
};

#ifndef CONFIG_USE_HW_CLK
static const int def_gpios_table[] = {
    116, 41, 122, 59, 37, 47, 49, 26, 85, 36, 27
};
#else
static const int def_gpios_table[] = {
    52, 57, 21, 32, 30, 51, 29, 50, 49, 33
};
#endif

static const unsigned long hw_clocks[] = {
    DEFAULT_STEP_FREQ,      //0,0        software clock
    4800,                   //0,Z        2*2400
    9600,                   //0,1        4*2400
    19200,                  //Z,0        8*2400
    9600,                   //Z,Z        4*2400
    38400,                  //Z,1        16*2400
    19200,                  //1,0        8*2400
    76800,                  //1,Z        32*2400
    38400,                  //1,1        16*2400
};

enum motor_step_mode {
    FULL_STEP = 0,          //0,0
    STEP_2_RISING,          //0,Z
    STEP_4_RISING,          //0,1
    STEP_8_RISING,          //Z,0
    STEP_8_BOTH_EDEG,       //Z,Z
    STEP_16_RISING,         //Z,1
    STEP_16_BOTH_EDGE,      //1,0
    STEP_32_RISING,         //1,Z
    STEP_32_BOTH_EDEG,      //1,1
};

enum motor_torque {
    TORQUE_FULL = 0,        //0,0
    TORQUE_87,              //0,Z
    TORQUE_75,              //0,1
    TORQUE_62,              //Z,0
    TORQUE_50,              //Z,Z
    TORQUE_37,              //Z,1
    TORQUE_25,              //1,0
    TORQUE_12,              //1,Z
    TORQUE_DISABLE,         //1,1
};

//Keep in sequence with dtsi.
enum motor_pins_mode {
    MODE0_LOW = 0,
    MODE0_HIGH,
    MODE0_DISABLE,
    MODE1_LOW,
    MODE1_HIGH,
    MODE1_DISABLE,
    T0_LOW,
    T0_HIGH,
    T0_DISABLE,
    T1_LOW,
    T1_HIGH,
    T1_DISABLE,
    INT_DEFAULT,
    CLK_ACTIVE,
    CLK_SLEEP,
#ifndef CONFIG_USE_HW_CLK
    SW_CLK_ACTIVE,
    SW_CLK_DISABLE,
#endif
    PINS_END
};

//Keep in same name with the pinctrl-names of dtsi
static const char* const pins_state[] = {
    "m0_low", "m0_high", "m0_disable",
    "m1_low", "m1_high", "m1_disable",
    "t0_low", "t0_high", "t0_disable",
    "t1_low", "t1_high", "t1_disable",
    "aw8646_int_default",
    "aw8646_clk_active", "aw8646_clk_sleep",
#ifndef CONFIG_USE_HW_CLK
    "aw8646_sw_clk_active", "aw8646_sw_clk_disable",
#endif
    NULL
};

#define MAX_GPIOS 20
typedef struct motor_control {
    struct regulator    *vdd;
    uint32_t ptable[MAX_GPIOS];
    size_t tab_cells;
    struct pinctrl* pins;
    struct pinctrl_state *pin_state[PINS_END];
    const char* const * plabels;
}motor_control;

#define CLOCK_NAME_LEN 16
typedef struct motor_device {
    char clock_name[CLOCK_NAME_LEN];
    struct device*  dev;
    struct device*  sysfs_dev;
    struct class*   aw8646_class;
    struct workqueue_struct* motor_wq;
    struct clk * pwm_clk;
    struct work_struct motor_irq_work;
    struct task_struct *motor_task;
    wait_queue_head_t sync_complete;
    struct hrtimer stepping_timer;
    motor_control mc;
    spinlock_t mlock;
    struct mutex mx_lock;
    int fault_irq;
    bool faulting;
    unsigned step_freq;
    unsigned long step_period;
    unsigned long step_ceiling;
    unsigned long cur_clk;
    atomic_t step_count;
    atomic_t stepping;
    unsigned mode, cur_mode;
    unsigned torque;
    unsigned time_out;
    unsigned half;
    unsigned position;
    bool     double_edge;
    bool     hw_clock;
    bool     power_default_off;
    int      level:1;
    unsigned power_en:1;
    unsigned nsleep:1;
    unsigned nEN:1;
    unsigned dir:1;
    unsigned user_sync_complete:1;
}motor_device;

static int set_pinctrl_state(motor_device* md, unsigned state_index);

static ktime_t adapt_time_helper(ktime_t usec)
{
    ktime_t ret;

    if(usec < 1000) {
        ret = ns_to_ktime(usec * 1000);
    } else if(usec < 1000000){
        ret = ms_to_ktime(usec);
    } else { //Should not to be here.
        ret = ms_to_ktime(usec * 1000);
    }

    return ret;
}

static inline bool is_hw_clk(motor_device* md)
{
    return (md->hw_clock && (md->cur_clk != hw_clocks[0]));
}

void sleep_helper(unsigned usec)
{
    if(usec < 500) { //<500us, udelay
        udelay(usec);
    } else if(usec >= 500 && usec < 20000) { //500us - 20ms, usleep_range
        usleep_range(usec, usec + 10);
    } else { //>= 20ms, msleep
        msleep(usec);
    }
}

static int moto_aw8646_set_regulator_power(motor_device* md, bool en)
{
    motor_control * mc = &md->mc;
    int err = 0;

    if(en) {
        err = regulator_enable(mc->vdd);
        if (err) {
            dev_err(md->dev, "Failed to enable VDD ret=%d\n", err);
            goto exit;
        }
    } else {
        err = regulator_disable(mc->vdd);
        if (err) {
            dev_err(md->dev, "Failed to disable VDD ret=%d\n", err);
            goto exit;
        }
    }

    return 0;

exit:
    return err;
}

#define L13A_VDD_MIN_UV       1650000
#define L13A_VDD_MAX_UV       3600000
static int moto_aw8646_init_regulator(motor_device* md)
{
    motor_control * mc = &md->mc;
    int err = 0;

    mc->vdd = devm_regulator_get(md->dev, "vdd");
    if (IS_ERR(mc->vdd)) {
        err = PTR_ERR(mc->vdd);
        dev_err(md->dev, "Failed to get VDD ret=%d\n", err);
        goto exit;
    }

    if (regulator_count_voltages(mc->vdd) > 0) {
        err = regulator_set_voltage(mc->vdd,
                L13A_VDD_MIN_UV,
                L13A_VDD_MAX_UV);
        if (err) {
            dev_err(md->dev, "Failed to set VDD range ret=%d\n", err);
            goto restore_vdd;
        }
    } else {
        dev_info(md->dev, "WARING: No VDD range set\n");
    }
    return 0;

restore_vdd:
    devm_regulator_put(mc->vdd);
exit:
    return err;
}

static int set_motor_clk(motor_device* md, bool en)
{
    int ret = 0;

    if(en) {
        set_pinctrl_state(md, CLK_ACTIVE);
        ret = clk_set_rate(md->pwm_clk, md->cur_clk);
        if(ret < 0) {
            dev_err(md->dev, "Failed to set clk rate to %ld\n", md->cur_clk);
            ret = -ENODEV;
            goto soft_clk;
        }
        ret = clk_prepare_enable(md->pwm_clk);
        if(ret < 0) {
            dev_err(md->dev, "Failed to clk_prepare_enable\n");
            ret = -ENODEV;
            goto soft_clk;
        }
    } else {
        clk_disable_unprepare(md->pwm_clk);
        dev_info(md->dev, "disable clock");
        set_pinctrl_state(md, CLK_SLEEP);
    }

    return 0;

soft_clk:
    md->hw_clock = false;
    return ret;
}

static int init_motor_clk(motor_device* md)
{
    int ret = 0;

    md->pwm_clk = devm_clk_get(md->dev, md->clock_name);
    if(IS_ERR(md->pwm_clk)) {
        dev_err(md->dev, "Get clk error, motor is not drived\n");
        ret = -ENODEV;
        goto soft_clk;
    }

    return 0;
soft_clk:
    md->hw_clock = false;
    return ret;
}

static int set_pinctrl_state(motor_device* md, unsigned state_index)
{
    motor_control* mc = &md->mc;
    int ret = 0;

    if(state_index >= PINS_END) {
        dev_err(md->dev, "Illegal pin index\n");
        goto err;
    }

    mc->pins = devm_pinctrl_get(md->dev);
    if(IS_ERR_OR_NULL(mc->pins)) {
        ret = PTR_ERR(mc->pins);
        dev_err(md->dev, "Failed to get pinctrl %d\n", ret);
        goto err;
    }

    mc->pin_state[state_index] = pinctrl_lookup_state(mc->pins, pins_state[state_index]);
    if (IS_ERR_OR_NULL(mc->pin_state[state_index])) {
        ret = PTR_ERR(mc->pin_state[state_index]);
        dev_err(md->dev, "Failed to lookup pin_state[%d] %d\n", state_index, ret);
        goto err_pin_state;
    }

    ret = pinctrl_select_state(mc->pins, mc->pin_state[state_index]);
    if(ret) {
        dev_err(md->dev, "Failed to set pin_state[%d] %d\n", state_index, ret);
    }

err_pin_state:
    pinctrl_put(mc->pins);
err:
    return ret;
}

static void moto_aw8646_set_motor_torque(motor_device* md)
{
    //motor_control* mc = &md->mc;

    switch(md->torque) {
        case TORQUE_FULL:
            set_pinctrl_state(md, T0_LOW);
            set_pinctrl_state(md, T1_LOW);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 0);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 0);
            break;
        case TORQUE_87:
            set_pinctrl_state(md, T0_DISABLE);
            set_pinctrl_state(md, T1_LOW);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 0);
            //gpio_direction_input(mc->ptable[MOTOR_T1]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_DISABLE]);
            break;
        case TORQUE_75:
            set_pinctrl_state(md, T0_HIGH);
            set_pinctrl_state(md, T1_LOW);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_LOW]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 0);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 1);
            break;
        case TORQUE_62:
            set_pinctrl_state(md, T0_LOW);
            set_pinctrl_state(md, T1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_T0]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 0);
            break;
        case TORQUE_50:
            set_pinctrl_state(md, T0_DISABLE);
            set_pinctrl_state(md, T1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_T0]);
            //gpio_direction_input(mc->ptable[MOTOR_T1]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_DISABLE]);
            break;
        case TORQUE_37:
            set_pinctrl_state(md, T0_HIGH);
            set_pinctrl_state(md, T1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_T0]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 1);
            break;
        case TORQUE_25:
            set_pinctrl_state(md, T0_LOW);
            set_pinctrl_state(md, T1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_HIGH]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 1);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 0);
            break;
        case TORQUE_12:
            set_pinctrl_state(md, T0_DISABLE);
            set_pinctrl_state(md, T1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 0);
            //gpio_direction_input(mc->ptable[MOTOR_T1]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_DISABLE]);
            break;
        case TORQUE_DISABLE:
            set_pinctrl_state(md, T0_HIGH);
            set_pinctrl_state(md, T1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[T0_HIGH]);
            //pinctrl_select_state(mc->pins, mc->pin_state[T1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_T0], 1);
            //gpio_direction_output(mc->ptable[MOTOR_T1], 1);
            break;
        default:
            pr_err("Don't supported torque\n");
            return;
    }
}

static void moto_aw8646_set_motor_mode(motor_device* md)
{
    switch(md->mode) {
        case FULL_STEP:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_LOW);
            break;
        case STEP_2_RISING:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_LOW);
            break;
        case STEP_4_RISING:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_LOW);
            break;
        case STEP_8_RISING:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_DISABLE);
            break;
        case STEP_8_BOTH_EDEG:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_DISABLE);
            break;
        case STEP_16_RISING:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_DISABLE);
            break;
        case STEP_16_BOTH_EDGE:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_HIGH);
            break;
        case STEP_32_RISING:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_HIGH);
            break;
        case STEP_32_BOTH_EDEG:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_HIGH);
            break;
        default:
            pr_err("Don't supported mode\n");
            return;
    }

    if(md->cur_mode != md->mode) {
        if(md->hw_clock) {
            if(md->mode == FULL_STEP) {
                md->cur_clk = hw_clocks[md->mode];
                set_pinctrl_state(md, CLK_SLEEP);
#ifndef CONFIG_USE_HW_CLK
                set_pinctrl_state(md, SW_CLK_ACTIVE);
#endif
                dev_info(md->dev, "md->mode is FULL_STEP\n");
            } else {
#ifndef CONFIG_USE_HW_CLK
                //Only set software clk once.
                if(md->cur_mode == FULL_STEP) {
                    set_pinctrl_state(md, SW_CLK_DISABLE);
                }
#endif
                md->cur_clk = hw_clocks[md->mode];
                dev_info(md->dev, "Switch hw clock\n");
            }
        }
        md->cur_mode = md->mode;
    }
    usleep_range(800, 900);
}

static void moto_aw8646_set_motor_dir(motor_device* md)
{
    motor_control* mc = &md->mc;

    gpio_direction_output(mc->ptable[MOTOR_DIR], md->dir);
    usleep_range(800, 900);
}

/* Set operating modes, only control nSleep
 * Set nSleep & nEn  | H-Bridge | Vreg | Sequencer
 *       1       0   |   Op     |  Op  |   Op
 *       1       1   |   Dis    |  Op  |   Op
 *       0       X   |   Dis    |  Dis |   Dis
 *         Fault     |   Dis    |Depends on Fault
 */
static void moto_aw8646_set_motor_opmode(motor_device* md)
{
    motor_control* mc = &md->mc;

    gpio_direction_output(mc->ptable[MOTOR_SLEEP], md->nsleep);
    //Twake 0.5ms Tsleep 0.7ms
    usleep_range(800, 1000);
    gpio_direction_output(mc->ptable[MOTOR_EN], md->nEN);
}

static int moto_aw8646_set_opmode(motor_device* md, unsigned opmode)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);

    if(md->nsleep == opmode) {
        dev_info(md->dev, "Unchanged the opmode status, ignore\n");
        ret = -EINVAL;
        goto exit;
    }
    md->nsleep = !!opmode;
    md->nEN = !md->nsleep;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

//Set VDD
static void moto_aw8646_set_power_en(motor_device* md)
{
    motor_control* mc = &md->mc;

    gpio_direction_output(mc->ptable[MOTOR_POWER_EN], md->power_en);
    //Tpower 0.5ms
    usleep_range(500, 1000);
}

//Power sequence: PowerEn On->nSleep On->nSleep Off->PowerEn Off
static int moto_aw8646_set_power(motor_device* md, unsigned power)
{
    unsigned long flags;
    int ret = 0;

    if(md->power_en == power) {
        dev_info(md->dev, "Unchanged the power status, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    spin_lock_irqsave(&md->mlock, flags);
    md->power_en = !!power;
    spin_unlock_irqrestore(&md->mlock, flags);

    ret =  moto_aw8646_set_opmode(md, md->power_en);
    if(ret < 0 ) {
        goto exit;
    }
    if(!md->power_en) {
        moto_aw8646_set_motor_opmode(md);
    }
    moto_aw8646_set_power_en(md);

    return 0;

exit:
    return ret;
}

static int moto_aw8646_enable_clk(motor_device* md, bool en)
{
    return set_motor_clk(md, en);
}

static int moto_aw8646_drive_sequencer(motor_device* md)
{
    unsigned long flags;

    atomic_set(&md->stepping, 1);
    atomic_set(&md->step_count, 0);

    spin_lock_irqsave(&md->mlock, flags);
    md->double_edge = false;
    md->half = md->step_period >> 1;
    md->level = 1;
    if(md->mode == STEP_8_BOTH_EDEG
        || md->mode == STEP_16_BOTH_EDGE
        || md->mode == STEP_32_BOTH_EDEG) {
            md->double_edge = true;
    }
    spin_unlock_irqrestore(&md->mlock, flags);

    moto_aw8646_set_motor_torque(md);
    moto_aw8646_set_motor_dir(md);
    moto_aw8646_set_motor_mode(md);
    moto_aw8646_set_motor_opmode(md);

    if(atomic_read(&md->stepping)) {
        sysfs_notify(&md->dev->kobj, NULL, "status");
        if(is_hw_clk(md)) {
            moto_aw8646_enable_clk(md, true);
            hrtimer_start(&md->stepping_timer, ms_to_ktime(md->time_out), HRTIMER_MODE_REL);
            dev_info(md->dev, "driver hw clock\n");
        } else {
#ifndef CONFIG_USE_HW_CLK
            motor_control* mc = &md->mc;
            gpio_direction_output(mc->ptable[MOTOR_STEP], md->level);
#endif
            atomic_inc(&md->step_count);
            hrtimer_start(&md->stepping_timer, adapt_time_helper(md->half), HRTIMER_MODE_REL);
            dev_info(md->dev, "advance the motor half of period %d, %lld\n",
                md->half, adapt_time_helper(md->half));
        }
    }

    return 0;
}

static __ref int motor_kthread(void *arg)
{
    motor_device* md = (motor_device*)arg;
    struct sched_param param = {.sched_priority = MAX_USER_RT_PRIO - 1};
    int ret = 0;

    sched_setscheduler(current, SCHED_FIFO, &param);
    while (!kthread_should_stop()) {
        do {
            ret = wait_event_interruptible(md->sync_complete,
                        md->user_sync_complete || kthread_should_stop());
        } while (ret != 0);

        if(kthread_should_stop()) {
            break;
        }

        md->user_sync_complete = false;
        if(is_hw_clk(md)) {
            moto_aw8646_enable_clk(md, false);
        }
        moto_aw8646_set_power(md, 0);
        if (md->power_default_off) {
            dev_info(md->dev, "vdd power off\n");
            moto_aw8646_set_regulator_power(md, false);
        }
        sysfs_notify(&md->dev->kobj, NULL, "status");
    }

    return 0;
}

static int disable_motor(struct device* dev)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    ktime_t time_rem;
    int ret = 0;

    if(atomic_read(&md->stepping)) {
        atomic_set(&md->stepping, 0);
        atomic_set(&md->step_count, 0);

        time_rem = hrtimer_get_remaining(&md->stepping_timer);
        if(ktime_to_us(time_rem) > 0) {
            hrtimer_try_to_cancel(&md->stepping_timer);
        }

        if(is_hw_clk(md)) {
            moto_aw8646_enable_clk(md, false);
        }
    }

    return ret;
}

static int motor_set_enable(struct device* dev, bool enable)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    if(atomic_read(&md->stepping)) {
        disable_motor(dev);
    }

    if(enable) {
       moto_aw8646_drive_sequencer(md);
    }

    return 0;
}

static enum hrtimer_restart motor_stepping_timer_action(struct hrtimer *h)
{
    motor_device * md = container_of(h, motor_device, stepping_timer);
    unsigned long flags;
    enum hrtimer_restart ret;

    spin_lock_irqsave(&md->mlock, flags);
    if(is_hw_clk(md)) {
        ret = HRTIMER_NORESTART;
        md->user_sync_complete = true;
        wake_up(&md->sync_complete);
        atomic_set(&md->stepping, 0);
    } else {
        ret = HRTIMER_RESTART;
        if(!atomic_read(&md->stepping)) {
            if(md->power_en) {
                md->user_sync_complete = true;
                wake_up(&md->sync_complete);
            }
#ifndef CONFIG_USE_HW_CLK
            gpio_direction_output(md->mc.ptable[MOTOR_STEP], 0);
#endif
            spin_unlock_irqrestore(&md->mlock, flags);
            return HRTIMER_NORESTART;;
        }

        md->level = !md->level;
#ifndef CONFIG_USE_HW_CLK
        gpio_direction_output(md->mc.ptable[MOTOR_STEP], md->level);
#endif
        if(md->double_edge) {
            atomic_inc(&md->step_count);
        } else if(md->level) {
            atomic_inc(&md->step_count);
        }

        if(md->step_ceiling && atomic_read(&md->step_count) >= md->step_ceiling) {
            atomic_set(&md->step_count, 0);
            atomic_set(&md->stepping, 0);
            md->user_sync_complete = true;
            wake_up(&md->sync_complete);
            ret = HRTIMER_NORESTART;;
        } else {
            hrtimer_forward_now(h, adapt_time_helper(md->half));
        }
    }
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

#define RESET_TIME 1000
static void motor_fault_work(struct work_struct *work)
{
    motor_device* md = container_of(work, motor_device, motor_irq_work);

    disable_irq(md->fault_irq);
    if(atomic_read(&md->stepping)) {
        disable_motor(md->dev);
    }
    moto_aw8646_set_power(md, 0);
    msleep(RESET_TIME);
    moto_aw8646_set_power(md, 1);
    md->faulting = false;
    enable_irq(md->fault_irq);
}

static irqreturn_t motor_fault_irq(int irq, void *pdata)
{
    motor_device * md = (motor_device*) pdata;
    int value = gpio_get_value(md->mc.ptable[MOTOR_FAULT_INT]);

    if(value) {
        dev_info(md->dev, "dummy motor irq event\n");
    }
#if 0
    md->faulting = true;
    dev_err(md->dev, "Motor fault irq is happened\n");
    queue_work(md->motor_wq, &md->motor_irq_work);
#endif
    return IRQ_HANDLED;
}

//module node interface
static void moto_aw8646_set_torque(motor_device* md, unsigned torque)
{
    unsigned long flags;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->torque == torque) {
        dev_info(md->dev, "Unchanged the torque, ignore\n");
        goto exit;
    }

    md->torque = torque;
exit:
    spin_unlock_irqrestore(&md->mlock, flags);
}

static int moto_aw8646_set_mode(motor_device* md, unsigned mode)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->mode == mode) {
        dev_info(md->dev, "Unchanged the mode, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->mode = mode;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

static int moto_aw8646_set_dir(motor_device* md, unsigned dir)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->dir == dir) {
        dev_info(md->dev, "Unchanged the dir, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->dir = dir;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

//Spec step max frequency 250KHz
#define STEP_MAX_FREQ 250000
static void moto_aw8646_set_step_freq(motor_device* md, unsigned freq)
{
    unsigned long flags;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->step_freq == freq) {
        dev_err(md->dev, "Unchanged the freq, ignore\n");
        goto exit;
    }
    if(freq > STEP_MAX_FREQ)
        freq = STEP_MAX_FREQ;
    md->step_freq = freq;
    if(md->step_freq <= 1000) { //ms
        md->step_period = 1000 / md->step_freq;
        md->step_period *= 1000; //us
    } else { //us
        md->step_period = 1000000 / md->step_freq;
    }

    dev_info(md->dev, "freq %d period %ld\n", md->step_freq, md->step_period);
exit:
    spin_unlock_irqrestore(&md->mlock, flags);
}

static int moto_aw8646_set_ceiling(motor_device* md, unsigned ceiling)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->step_ceiling == ceiling) {
        dev_info(md->dev, "Unchanged the ceiling, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->step_ceiling = ceiling;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

static ssize_t motor_enable_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->power_en);
}

static ssize_t motor_enable_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned enable = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &enable)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    enable = !!enable;

    if (md->power_default_off && (enable != 0)) {
        dev_info(md->dev, "vdd power on\n");
        moto_aw8646_set_regulator_power(md, true);
        msleep(1);
    }

    if(!moto_aw8646_set_power(md, enable)) {
        motor_set_enable(dev, enable);
    }

exit:
    return len;
}

static ssize_t motor_dir_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->dir);
}

static ssize_t motor_dir_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_aw8646_set_dir(md, value);

exit:
    return len;
}

static ssize_t motor_step_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->step_freq);
}

static ssize_t motor_step_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_aw8646_set_step_freq(md, value);

exit:
    return len;
}

static ssize_t motor_mode_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->mode);
}

static ssize_t motor_mode_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_aw8646_set_mode(md, value);

exit:
    return len;
}

static ssize_t motor_torque_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->torque);
}

static ssize_t motor_torque_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_aw8646_set_torque(md, value);

exit:
    return len;
}

static ssize_t motor_ceiling_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%ld\n", md->step_ceiling);
}

static ssize_t motor_ceiling_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_aw8646_set_ceiling(md, value);

exit:
    return len;
}

static ssize_t motor_time_out_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    return snprintf(buf, 20, "%d\n", md->time_out);
}

#define STEP_TIME_OUT 3000 //3s for stop motor
static ssize_t motor_time_out_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;
    unsigned long flags;

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    spin_lock_irqsave(&md->mlock, flags);
    if(value > STEP_TIME_OUT) {
        value = STEP_TIME_OUT;
    }
    md->time_out = value;
    spin_unlock_irqrestore(&md->mlock, flags);

exit:
    return len;
}

static ssize_t motor_reset_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned value = 0;

    mutex_lock(&md->mx_lock);

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }
    disable_motor(md->dev);
    moto_aw8646_set_power(md, 0);
    moto_aw8646_set_regulator_power(md, false);
    msleep(10);
    moto_aw8646_set_regulator_power(md, true);
    msleep(10);
    moto_aw8646_set_power(md, 0);
exit:
    mutex_unlock(&md->mx_lock);
    return len;
}

static ssize_t motor_position_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned long flags;
    unsigned value = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }
    md->position = !!value;
exit:
    spin_unlock_irqrestore(&md->mlock, flags);

    return len;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, motor_enable_show, motor_enable_store);
static DEVICE_ATTR(dir,    S_IRUGO|S_IWUSR|S_IWGRP, motor_dir_show, motor_dir_store);
static DEVICE_ATTR(step,   S_IRUGO|S_IWUSR|S_IWGRP, motor_step_show, motor_step_store);
static DEVICE_ATTR(ceiling, S_IRUGO|S_IWUSR|S_IWGRP, motor_ceiling_show, motor_ceiling_store);
static DEVICE_ATTR(mode,   S_IRUGO|S_IWUSR|S_IWGRP, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(torque, S_IRUGO|S_IWUSR|S_IWGRP, motor_torque_show, motor_torque_store);
static DEVICE_ATTR(time_out, S_IRUGO|S_IWUSR|S_IWGRP, motor_time_out_show, motor_time_out_store);
static DEVICE_ATTR(reset,  S_IRUGO|S_IWUSR|S_IWGRP, NULL, motor_reset_store);
static DEVICE_ATTR(position,  S_IRUGO|S_IWUSR|S_IWGRP, NULL, motor_position_store);

static struct attribute *motor_attributes[] = {
    &dev_attr_dir.attr,
    &dev_attr_enable.attr,
    &dev_attr_step.attr,
    &dev_attr_ceiling.attr,
    &dev_attr_mode.attr,
    &dev_attr_torque.attr,
    &dev_attr_time_out.attr,
    &dev_attr_reset.attr,
    &dev_attr_position.attr,
    NULL
};

static struct attribute_group motor_attribute_group = {
    .attrs = motor_attributes
};

static const struct attribute_group * motor_attr_groups[] = {
    &motor_attribute_group,
    NULL
};

static ssize_t motor_status_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned long flags;
    int power = 0;

    spin_lock_irqsave(&md->mlock, flags);
    power = md->power_en;
    spin_unlock_irqrestore(&md->mlock, flags);

    return snprintf(buf, 20, "%d\n", power);
}
static DEVICE_ATTR(status, S_IRUGO|S_IWUSR|S_IWGRP, motor_status_show, NULL);

static struct attribute *status_attributes[] = {
    &dev_attr_status.attr,
    NULL
};
static struct attribute_group status_attribute_group = {
    .attrs = status_attributes
};

static int moto_aw8646_init_from_dt(motor_device* md)
{
    struct device* pdev = md->dev;
    struct device_node *np = pdev->of_node;
    motor_control* mc = &md->mc;
    uint32_t temp_value;
    int rc = 0;
    const char *clock_name;

    rc = of_property_read_u32(np, "aw8646-gpios-cells", &temp_value);
    if (rc) {
        dev_err(pdev, "%d:Failed to get gpios cells\n", rc);
        goto exit;
    }
    mc->tab_cells = temp_value;

    if(mc->tab_cells > MAX_GPIOS) {
        dev_err(pdev, "Occupied too many gpios, max limited is %d\n", MAX_GPIOS);
        mc->tab_cells = MAX_GPIOS;
    }

    rc = of_property_read_u32_array(np, "aw8646-gpios", mc->ptable, mc->tab_cells);
    if (rc) {
        dev_err(pdev, "%d:Failed to get gpios list\n", rc);
        goto exit;
    }

    if (!of_property_read_string(np, "clock-names", &clock_name))
        strlcpy(md->clock_name, clock_name, CLOCK_NAME_LEN);
    else
        strlcpy(md->clock_name, MOTOR_HW_CLK_NAME, CLOCK_NAME_LEN);
    dev_info(pdev, "hw clock name: %s\n", md->clock_name);

    md->hw_clock = of_property_read_bool(np, "enable-hw-clock");
    dev_info(pdev, "Enable hw clock %d\n", md->hw_clock);

    md->power_default_off = of_property_read_bool(np, "power-default-off");
    dev_info(pdev, "power is default off:  %d\n", md->power_default_off);

exit:
    return rc;
}

#if 0
static int moto_aw8646_pinctrl_init(motor_device* md)
{
    motor_control * mc = &md->mc;
    int ret = 0;
    int i = 0;

    mc->pins = devm_pinctrl_get(md->dev);
    if(IS_ERR_OR_NULL(mc->pins)) {
        ret = PTR_ERR(mc->pins);
        dev_err(md->dev, "Failed to get pinctrl %d\n", ret);
        goto err;
    }

    for(i = MODE0_LOW; i < PINS_END; i++) {
        mc->pin_state[i] = pinctrl_lookup_state(mc->pins, pins_state[i]);
        if (IS_ERR_OR_NULL(mc->pin_state[i])) {
            ret = PTR_ERR(mc->pin_state[i]);
            dev_err(md->dev, "Failed to lookup pin_state[%d] %d\n", i, ret);
            goto err_pin_state;
        }
        dev_err(md->dev, "pin_state[%d] %p %d\n", i, mc->pin_state[i], ret);
    }

err_pin_state:
    devm_pinctrl_put(mc->pins);
err:
    return ret;
}
#endif

static int moto_aw8646_probe(struct platform_device *pdev)
{
    struct device* dev = &pdev->dev;
    motor_device* md;
    int i = MOTOR_POWER_EN;
    int ret = 0;

    md = kzalloc(sizeof(motor_device), GFP_KERNEL);
    if(!md) {
        dev_err(dev, "probe: Out of memory\n");
        return -ENOMEM;
    }

    md->dev = dev;
    md->mc.plabels = gpios_labels;
    spin_lock_init(&md->mlock);
    mutex_init(&md->mx_lock);
    platform_set_drvdata(pdev, md);
    moto_aw8646_set_step_freq(md, DEFAULT_STEP_FREQ);
    md->time_out = MOTOR_DEFAULT_EXPIRE;

    if(moto_aw8646_init_from_dt(md)) {
        memcpy((void*)md->mc.ptable, def_gpios_table, sizeof(def_gpios_table));
    }

    ret = moto_aw8646_init_regulator(md);
    if(ret) {
        dev_err(dev, "Failed init regulator\n");
        goto failed_mem;
    }

    if (!md->power_default_off) {
        ret = moto_aw8646_set_regulator_power(md, true);
        if(ret) {
            regulator_put(md->mc.vdd);
            dev_err(dev, "Failed enable regulator\n");
            goto failed_mem;
        }
    }

    md->motor_wq = alloc_workqueue("motor_wq", WQ_HIGHPRI, 0);
    if(!md->motor_wq) {
        dev_err(dev, "Out of memory for work queue\n");
        goto failed_mem;
    }
    INIT_WORK(&md->motor_irq_work, motor_fault_work);

    md->user_sync_complete = false;
    init_waitqueue_head(&md->sync_complete);
    md->motor_task = kthread_create(motor_kthread, md, "motor_task");
    if (IS_ERR(md->motor_task)) {
        ret = PTR_ERR(md->motor_task);
        dev_err(dev, "Failed create motor kthread\n");
        goto failed_work;
    }

    hrtimer_init(&md->stepping_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    md->stepping_timer.function = motor_stepping_timer_action;

    md->aw8646_class = class_create(THIS_MODULE, MOTOR_CLASS_NAME);
    if(IS_ERR(md->aw8646_class)) {
        dev_err(dev, "Failed to create class\n");
        goto failed_sys;
    }

    md->sysfs_dev = device_create_with_groups(md->aw8646_class,
                 dev, MKDEV(0, 0), md, motor_attr_groups,
                 "%s", MOTOR_CONTROL);
    if(IS_ERR(md->sysfs_dev)) {
        dev_err(dev, "Failed to create device\n");
        goto failed_sys_dev;
    }

    if(sysfs_create_group(&md->dev->kobj, &status_attribute_group)) {
        dev_err(dev, "Failed to create status attribute");
        goto failed_device;
    }

    if(md->hw_clock) {
        if(init_motor_clk(md) < 0) {
            //Should get this gpio from DTB
            dev_err(dev, "HW clock is not setup\n");
        } else {
            md->cur_clk = hw_clocks[0];
            dev_info(dev, "HW clock is setup\n");
        }
    }

    //Init software clock pin.
    set_pinctrl_state(md, CLK_SLEEP);
#ifndef CONFIG_USE_HW_CLK
    set_pinctrl_state(md, SW_CLK_ACTIVE);
#endif
    do {
        ret = devm_gpio_request(dev, md->mc.ptable[i], gpios_labels[i]);
        if(ret < 0) {
            pr_err("Failed to request %s, errno %d\n", gpios_labels[i--], ret);
            goto failed_gpio;
        }
        gpio_direction_output(md->mc.ptable[i], 0);
        i++;
    } while (i < MOTOR_UNKNOWN);

    wake_up_process(md->motor_task);

    if(!set_pinctrl_state(md, INT_DEFAULT)) {
        md->fault_irq = gpio_to_irq(md->mc.ptable[MOTOR_FAULT_INT]);
        ret = devm_request_threaded_irq(&pdev->dev, md->fault_irq, motor_fault_irq,
                motor_fault_irq, IRQF_TRIGGER_FALLING, "motor_irq", md);
        if(ret < 0) {
            dev_err(dev, "Failed to request irq %d\n", ret);
            goto failed_gpio;
        }
    } else {
        /*Here motor can work,  but have not irq*/
        dev_info(dev, "Failed to set device irq\n");
    }

    dev_info(dev, "Success init device\n");
    return 0;

failed_gpio:
    while(i >= MOTOR_POWER_EN) {
        devm_gpio_free(dev, md->mc.ptable[i--]);
    }
    sysfs_remove_group(&md->dev->kobj, &status_attribute_group);
failed_device:
    device_destroy(md->aw8646_class, MKDEV(0, 0));
failed_sys_dev:
    class_destroy(md->aw8646_class);
failed_sys:
    kthread_stop(md->motor_task);
failed_work:
    destroy_workqueue(md->motor_wq);
failed_mem:
    kfree(md);

    return ret;
}

static int moto_aw8646_remove(struct platform_device *pdev)
{
    motor_device* md = (motor_device*)platform_get_drvdata(pdev);

    disable_irq(md->fault_irq);
    moto_aw8646_set_power(md, 0);
    moto_aw8646_set_regulator_power(md, false);
    device_destroy(md->aw8646_class, MKDEV(0, 0));
    class_destroy(md->aw8646_class);
    sysfs_remove_group(&md->dev->kobj, &status_attribute_group);
    disable_motor(md->dev);
    kthread_stop(md->motor_task);
    if(is_hw_clk(md) && __clk_is_enabled(md->pwm_clk)) {
        clk_disable_unprepare(md->pwm_clk);
    }
    destroy_workqueue(md->motor_wq);
    kfree(md);

    return 0;
}

void moto_aw8646_platform_shutdown(struct platform_device *pdev)
{
    motor_device* md = (motor_device*)platform_get_drvdata(pdev);

    if(md->position) {
        md->dir = 1;
        moto_aw8646_set_motor_dir(md);
        md->power_en = 0;
        moto_aw8646_set_power(md, 1);
        motor_set_enable(md->dev, true);
    }
    msleep(800);
}

static const struct of_device_id moto_aw8646_match_table[] = {
	{.compatible = "moto,aw8646"},
	{},
};
MODULE_DEVICE_TABLE(of, moto_aw8646_match_table);

static struct platform_driver moto_aw8646_driver = {
	.probe = moto_aw8646_probe,
	.remove = moto_aw8646_remove,
	.shutdown = moto_aw8646_platform_shutdown,
	.driver = {
		.name = "moto,aw8646",
		.owner = THIS_MODULE,
		.of_match_table = moto_aw8646_match_table,
	},
};

static int __init moto_aw8646_haptic_init(void)
{
	return platform_driver_register(&moto_aw8646_driver);
}

module_init(moto_aw8646_haptic_init);

static void __exit moto_aw8646_haptic_exit(void)
{
	platform_driver_unregister(&moto_aw8646_driver);
}
module_exit(moto_aw8646_haptic_exit);

MODULE_DESCRIPTION("Motorola AW8646 Haptic Driver");
MODULE_AUTHOR("zoujc <zoujc@lenovo.com>");
MODULE_LICENSE("GPL v2");
