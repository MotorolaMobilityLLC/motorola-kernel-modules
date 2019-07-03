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
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#define DEFAULT_STEP_FREQ 2400
#define MOTOR_CLASS_NAME  "aw8646"
#define MOTOR_CONTROL  "control"

enum gpios_index {
    MOTOR_POWER_EN = 0,
    MOTOR_FAULT_INT,
    MOTOR_STEP,
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
    "MOTOR_STEP",
    "MOTOR_DIR",
    "MOTOR_MODE1",
    "MOTOR_MODE2",
    "MOTOR_EN",
    "MOTOR_SLEEP",
    "MOTOR_T0",
    "MOTOR_T1",
    "MOTOR_UNKNOWN"
};

static const int def_gpios_table[] = {
    116, 41, 122, 37, 47, 49, 26, 85, 36, 27
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
    PINS_END
};

//Keep in same name with the pinctrl-names of dtsi
static const char* const pins_state[] = {
    "m0_low", "m0_high", "m0_disable",
    "m1_low", "m1_high", "m1_disable",
    "t0_low", "t0_high", "t0_disable",
    "t1_low", "t1_high", "t1_disable",
    "aw8646_int_default",
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

typedef struct motor_device {
    struct device*  dev;
    struct device*  sysfs_dev;
    struct class*   aw8646_class;
    struct workqueue_struct* motor_wq;
    struct work_struct motor_work;
    struct work_struct motor_irq_work;
    struct hrtimer step_timer;
    motor_control mc;
    spinlock_t mlock;
    struct mutex mx_lock;
    int fault_irq;
    bool faulting;
    unsigned step_freq;
    unsigned long step_period;
    unsigned long step_ceiling;
    atomic_t step_count;
    atomic_t stepping;
    unsigned mode;
    unsigned torque;
    unsigned time_out;
    unsigned power_en:1;
    unsigned nsleep:1;
    unsigned nEN:1;
    unsigned dir:1;
}motor_device;

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
    //motor_control* mc = &md->mc;

    switch(md->mode) {
        case FULL_STEP:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_LOW);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_LOW]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_LOW]);
            //gpio_direction_output(md->mc.ptable[MOTOR_MODE1], 0);
            //gpio_direction_output(md->mc.ptable[MOTOR_MODE2], 0);
            break;
        case STEP_2_RISING:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_LOW);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE1], 0);
            //gpio_direction_input(mc->ptable[MOTOR_MODE2]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_DISABLE]);
            break;
        case STEP_4_RISING:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_LOW);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_LOW]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE1], 0);
            //gpio_direction_output(mc->ptable[MOTOR_MODE2], 1);
            break;
        case STEP_8_RISING:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_MODE1]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE2], 0);
            break;
        case STEP_8_BOTH_EDEG:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_MODE1]);
            //gpio_direction_input(mc->ptable[MOTOR_MODE2]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_DISABLE]);
            break;
        case STEP_16_RISING:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_DISABLE);
            //gpio_direction_input(mc->ptable[MOTOR_MODE1]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_DISABLE]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE2], 1);
            break;
        case STEP_16_BOTH_EDGE:
            set_pinctrl_state(md, MODE0_LOW);
            set_pinctrl_state(md, MODE1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_HIGH]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE1], 1);
            //gpio_direction_output(mc->ptable[MOTOR_MODE2], 0);
            break;
        case STEP_32_RISING:
            set_pinctrl_state(md, MODE0_DISABLE);
            set_pinctrl_state(md, MODE1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_LOW]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE1], 0);
            //gpio_direction_input(mc->ptable[MOTOR_MODE2]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_DISABLE]);
            break;
        case STEP_32_BOTH_EDEG:
            set_pinctrl_state(md, MODE0_HIGH);
            set_pinctrl_state(md, MODE1_HIGH);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE0_HIGH]);
            //pinctrl_select_state(mc->pins, mc->pin_state[MODE1_HIGH]);
            //gpio_direction_output(mc->ptable[MOTOR_MODE1], 1);
            //gpio_direction_output(mc->ptable[MOTOR_MODE2], 1);
            break;
        default:
            pr_err("Don't supported mode\n");
            return;
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

static int moto_aw8646_drive_sequencer(motor_device* md)
{
    motor_control* mc = &md->mc;
    unsigned long half = md->step_period >> 1;
    bool double_edge = false;
    unsigned long flags;

    atomic_set(&md->stepping, 1);
    atomic_set(&md->step_count, 0);

    spin_lock_irqsave(&md->mlock, flags);
    if(md->mode == STEP_8_BOTH_EDEG
        || md->mode == STEP_16_BOTH_EDGE
        || md->mode == STEP_32_BOTH_EDEG) {
            double_edge = true;
    }
    spin_unlock_irqrestore(&md->mlock, flags);

    moto_aw8646_set_motor_torque(md);
    moto_aw8646_set_motor_dir(md);
    moto_aw8646_set_motor_mode(md);
    moto_aw8646_set_motor_opmode(md);

    if(atomic_read(&md->stepping)) {
        dev_info(md->dev, "advance the motor half of period %ld\n", half);
        do {
            gpio_direction_output(mc->ptable[MOTOR_STEP], 1);
            sleep_helper(half);
            atomic_inc(&md->step_count);
            gpio_direction_output(mc->ptable[MOTOR_STEP], 0);
            sleep_helper(half);

            if(double_edge) {
                atomic_inc(&md->step_count);
            }

            if(md->step_ceiling && atomic_read(&md->step_count) >= md->step_ceiling) {
                atomic_set(&md->step_count, 0);
                atomic_set(&md->stepping, 0);
                moto_aw8646_set_power(md, 0);
                //sysfs_notify(&md->sysfs_dev->kobj, NULL, "status");
                dev_info(md->dev, "Stopped motor, count to ceiling %s\n", md->sysfs_dev->kobj.name);
                break;
            }
        } while(atomic_read(&md->stepping));
    }

    return 0;
}

static void advance_motor_work(struct work_struct *work)
{
    motor_device* md = container_of(work, motor_device, motor_work);

    moto_aw8646_drive_sequencer(md);
}

static int disable_motor(struct device* dev)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    ktime_t time_rem;
    int ret = 0;

    if(atomic_read(&md->stepping)) {
        time_rem = hrtimer_get_remaining(&md->step_timer);
        if(ktime_to_us(time_rem) > 0) {
            hrtimer_try_to_cancel(&md->step_timer);
        }
        atomic_set(&md->stepping, 0);
        atomic_set(&md->step_count, 0);
        ret = cancel_work_sync(&md->motor_work);
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
        if(md->time_out) {
            ktime_t time_out = ms_to_ktime(md->time_out);
            hrtimer_start(&md->step_timer, time_out, HRTIMER_MODE_REL);
        }
        queue_work(md->motor_wq, &md->motor_work);
    }

    return 0;
}

static enum hrtimer_restart motor_timer_action(struct hrtimer *h)
{
    motor_device * md = container_of(h, motor_device, step_timer);

    disable_motor(md->dev);
    dev_info(md->dev, "Stop motor, timer is out\n");

    return HRTIMER_NORESTART;
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

    dev_err(md->dev, "Motor fault irq is happened\n");
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

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, motor_enable_show, motor_enable_store);
static DEVICE_ATTR(dir,    S_IRUGO|S_IWUSR|S_IWGRP, motor_dir_show, motor_dir_store);
static DEVICE_ATTR(step,   S_IRUGO|S_IWUSR|S_IWGRP, motor_step_show, motor_step_store);
static DEVICE_ATTR(ceiling, S_IRUGO|S_IWUSR|S_IWGRP, motor_ceiling_show, motor_ceiling_store);
static DEVICE_ATTR(mode,   S_IRUGO|S_IWUSR|S_IWGRP, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(torque, S_IRUGO|S_IWUSR|S_IWGRP, motor_torque_show, motor_torque_store);
static DEVICE_ATTR(time_out, S_IRUGO|S_IWUSR|S_IWGRP, motor_time_out_show, motor_time_out_store);
static DEVICE_ATTR(reset,  S_IRUGO|S_IWUSR|S_IWGRP, NULL, motor_reset_store);

static struct attribute *motor_attributes[] = {
    &dev_attr_dir.attr,
    &dev_attr_enable.attr,
    &dev_attr_step.attr,
    &dev_attr_ceiling.attr,
    &dev_attr_mode.attr,
    &dev_attr_torque.attr,
    &dev_attr_time_out.attr,
    &dev_attr_reset.attr,
    NULL
};

static struct attribute_group motor_attribute_group = {
    .attrs = motor_attributes
};

static const struct attribute_group * motor_attr_groups[] = {
    &motor_attribute_group,
    NULL
};

static int moto_aw8646_init_from_dt(motor_device* md)
{
    struct device* pdev = md->dev;
    struct device_node *np = pdev->of_node;
    motor_control* mc = &md->mc;
    uint32_t temp_value;
    int rc = 0;

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

    if(moto_aw8646_init_from_dt(md)) {
        memcpy((void*)md->mc.ptable, def_gpios_table, sizeof(def_gpios_table));
    }

    ret = moto_aw8646_init_regulator(md);
    if(ret) {
        dev_err(dev, "Failed init regulator\n");
        goto failed_mem;
    }

    ret = moto_aw8646_set_regulator_power(md, true);
    if(ret) {
        regulator_put(md->mc.vdd);
        dev_err(dev, "Failed enable regulator\n");
        goto failed_mem;
    }

    md->motor_wq = alloc_workqueue("motor_wq", WQ_HIGHPRI, 0);
    if(!md->motor_wq) {
        dev_err(dev, "Out of memory for work queue\n");
        goto failed_mem;
    }
    INIT_WORK(&md->motor_work, advance_motor_work);
    INIT_WORK(&md->motor_irq_work, motor_fault_work);

    hrtimer_init(&md->step_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    md->step_timer.function = motor_timer_action;

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

    do {
        ret = devm_gpio_request(dev, md->mc.ptable[i], gpios_labels[i]);
        if(ret < 0) {
            pr_err("Failed to request %s, errno %d\n", gpios_labels[i--], ret);
            goto failed_gpio;
        }
        gpio_direction_output(md->mc.ptable[i], 0);
        i++;
    } while (i < MOTOR_UNKNOWN);

    if(!set_pinctrl_state(md, INT_DEFAULT)) {
        md->fault_irq = gpio_to_irq(md->mc.ptable[MOTOR_FAULT_INT]);
        ret = devm_request_threaded_irq(&pdev->dev, md->fault_irq, motor_fault_irq,
                motor_fault_irq, IRQF_TRIGGER_FALLING, "motor_irq", md);
        if(ret < 0) {
            dev_err(dev, "Failed to request irq %d\n", ret);
            goto failed_gpio;
        }
    } else {
        dev_info(dev, "Failed to set device irq\n");
    }

    dev_info(dev, "Success init device\n");
    return 0;

failed_gpio:
    while(i >= MOTOR_POWER_EN) {
        devm_gpio_free(dev, md->mc.ptable[i--]);
    }
    device_destroy(md->aw8646_class, MKDEV(0, 0));
failed_sys_dev:
    class_destroy(md->aw8646_class);
failed_sys:
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
    disable_motor(md->dev);
    destroy_workqueue(md->motor_wq);
    kfree(md);

    return 0;
}

static const struct of_device_id moto_aw8646_match_table[] = {
	{.compatible = "moto,aw8646"},
	{},
};
MODULE_DEVICE_TABLE(of, moto_aw8646_match_table);

static struct platform_driver moto_aw8646_driver = {
	.probe = moto_aw8646_probe,
	.remove = moto_aw8646_remove,
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
