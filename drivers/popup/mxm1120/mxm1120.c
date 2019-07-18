#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <uapi/linux/sched/types.h>
#include "mxm1120.h"

#define M1120_DETECTION_MODE             M1120_DETECTION_MODE_POLLING
#define M1120_INTERRUPT_TYPE             M1120_VAL_INTSRS_INTTYPE_BESIDE
#define M1120_SENSITIVITY_TYPE           M1120_VAL_INTSRS_SRS_10BIT_0_068mT
#define M1120_PERSISTENCE_COUNT          M1120_VAL_PERSINT_COUNT(4)
#define M1120_OPERATION_FREQUENCY        M1120_VAL_OPF_FREQ_80HZ
#define M1120_OPERATION_RESOLUTION       M1120_VAL_OPF_BIT_10
#define M1120_DETECT_RANGE_HIGH          (60)/*Need change via test.*/
#define M1120_DETECT_RANGE_LOW           (50)/*Need change via test.*/
#define M1120_RESULT_STATUS_A            (0x01)  // result status A ----> ==180Degree.
#define M1120_RESULT_STATUS_B            (0x02)  // result status B ----> != 180Degree.
#define M1120_EVENT_TYPE                 EV_ABS  // EV_KEY
#define M1120_EVENT_CODE                 ABS_X   // KEY_F1
#define M1120_EVENT_DATA_CAPABILITY_MIN  (-32768)
#define M1120_EVENT_DATA_CAPABILITY_MAX  (32767)

/*MagnaChip Hall Sensor power supply VDD 2.7V~3.6V, VIO 1.65~VDD*/
#define M1120_VDD_MIN_UV       2700000
#define M1120_VDD_MAX_UV       3600000
#define M1120_VIO_MIN_UV       1650000
#define M1120_VIO_MAX_UV       3600000
#define M1120_CONTROL          "control"

/***********************************************************/
/*debug macro*/
/***********************************************************/
#ifdef M1120_DBG_ENABLE
#define dbg(fmt, args...)  printk("[DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)
#define dbgn(fmt, args...)
#endif // M1120_DBG_ENABLE
#define dbg_func_in()     dbg("[M1120-DBG-F.IN] %s", __func__)
#define dbg_func_out()    dbg("[M1120-DBG-F.OUT] %s", __func__)
#define dbg_line()        dbg("[LINE] %d(%s)", __LINE__, __func__)

#define mxerr(pdev, fmt, args...)          \
    dev_err(pdev, "[ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define mxinfo(pdev, fmt, args...)        \
    dev_info(pdev, "[INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)

static int m1120_set_detection_mode(struct device *dev, u8 mode);
static int m1120_set_operation_mode(struct device *dev, int mode);
static int m1120_get_delay(struct device *dev);
static int m1120_measure(m1120_data_t *p_data, short *raw);

static void m1120_convdata_short_to_2byte(u8 opf, short x,
    unsigned char *hbyte, unsigned char *lbyte)
{
    if ((opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
        /*8 bit resolution*/
        if (x < -128)
            x = -128;
        else if (x > 127)
            x = 127;

        if (x >= 0) {
            *lbyte = x & 0x7F;
        } else {
            *lbyte = ((0x80 - (x*(-1))) & 0x7F) | 0x80;
        }
        *hbyte = 0x00;
    } else {
        /*10 bit resolution*/
        if (x < -512)
            x = -512;
        else if (x > 511)
            x = 511;

        if (x >= 0) {
            *lbyte = x & 0xFF;
            *hbyte = (((x&0x100)>>8)&0x01) << 6;
        } else {
            *lbyte = (0x0200 - (x*(-1))) & 0xFF;
            *hbyte = ((((0x0200 - (x*(-1))) & 0x100)>>8)<<6) | 0x80;
        }
    }
}

static short m1120_convdata_2byte_to_short(u8 opf, unsigned char hbyte, unsigned char lbyte)
{
    short x;

    if ((opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
        /*8 bit resolution*/
        x = lbyte & 0x7F;
        if (lbyte & 0x80) {
            x -= 0x80;
        }
    } else {
        /*10 bit resolution*/
        x = (((hbyte & 0x40) >> 6) << 8) | lbyte;
        if (hbyte&0x80) {
            x -= 0x200;
        }
    }

    return x;
}

static int m1120_get_enable(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    return atomic_read(&p_data->atm.enable);
}

static void m1120_set_enable(struct device *dev, int enable)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    mutex_lock(&p_data->mtx.enable);

    if (enable) {                  /*enable if state will be changed*/
        if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
            m1120_set_detection_mode(dev, p_data->reg.map.intsrs & M1120_DETECTION_MODE_POLLING);
            m1120_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);
        }
    } else {                        /*disable if state will be changed*/
        if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
            //cancel_delayed_work_sync(&p_data->work);
            hrtimer_cancel(&p_data->sample_timer);
            m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);
        }
    }
    atomic_set(&p_data->atm.enable, enable);

    mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_delay(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    int delay = 0;

    delay = atomic_read(&p_data->atm.delay);

    return delay;
}

static void m1120_set_delay(struct device *dev, int delay)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    ktime_t time_out;

    if (delay < M1120_DELAY_MIN)
        delay = M1120_DELAY_MIN;
    atomic_set(&p_data->atm.delay, delay);

    mutex_lock(&p_data->mtx.enable);

    if (m1120_get_enable(dev)) {
        if (!(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)) {
            //cancel_delayed_work_sync(&p_data->work);
            hrtimer_cancel(&p_data->sample_timer);
            //schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
            time_out = ms_to_ktime(delay);
            hrtimer_start(&p_data->sample_timer, time_out, HRTIMER_MODE_REL);
        }
    }

    mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_debug(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    return atomic_read(&p_data->atm.debug);
}

static void m1120_set_debug(struct device *dev, int debug)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    atomic_set(&p_data->atm.debug, debug);
}

static irqreturn_t m1120_irq_handler(int irq, void *dev_id)
{
    struct i2c_client *client = (struct i2c_client *)dev_id;
    m1120_data_t *p_data = i2c_get_clientdata(client);

    queue_work(p_data->m1120_wq, &p_data->m1120_work);
    //schedule_delayed_work(&p_data->work, 0);

    return IRQ_HANDLED;
}

static int m1120_power_ctl(struct i2c_client *client, bool on)
{
    int ret = 0;
    struct device* pdev = &client->dev;
    m1120_data_t *p_data = i2c_get_clientdata(client);

    if (!on && p_data->power_enabled) {
        ret = regulator_disable(p_data->vdd);
        if (ret) {
            mxerr(pdev, "Failed to disable %s VDD ret=%d\n", p_data->type, ret);
            return ret;
        }

        ret = regulator_disable(p_data->vio);
        if (ret) {
            mxerr(pdev, "Failed to disable %s VIO ret=%d\n", p_data->type, ret);
            ret = regulator_enable(p_data->vdd);
            return ret;
        }
        p_data->power_enabled = on;
    } else if (on && !p_data->power_enabled) {
        ret = regulator_enable(p_data->vdd);
        if (ret) {
            mxerr(pdev, "Failed to enable %s VDD ret=%d\n", p_data->type, ret);
            return ret;
        }
        msleep(8); ////>=5ms OK.
        ret = regulator_enable(p_data->vio);
        if (ret) {
            mxerr(pdev, "Failed to enable %s VIO ret=%d\n", p_data->type, ret);
            regulator_disable(p_data->vdd);
            return ret;
        }
        msleep(10); // wait 10ms
        p_data->power_enabled = on;
    } else {
        mxerr(pdev, "%s: Power on=%d. enabled=%d\n",
            p_data->type, on, p_data->power_enabled);
    }

    return ret;
}

static int m1120_power_init(struct i2c_client *client)
{
    int ret = 0;
    struct device* pdev = &client->dev;
    m1120_data_t *p_data = i2c_get_clientdata(client);

    p_data->vdd = regulator_get(pdev, "vdd");
    if (IS_ERR(p_data->vdd)) {
        ret = PTR_ERR(p_data->vdd);
        mxerr(pdev, "Failed to get %s VDD ret=%d\n", p_data->type, ret);
        goto exit;
    }

    if (regulator_count_voltages(p_data->vdd) > 0) {
        ret = regulator_set_voltage(p_data->vdd,
                M1120_VDD_MIN_UV,
                M1120_VDD_MAX_UV);
        if (ret) {
            mxerr(pdev, "Failed to set %s vdd range ret=%d\n", p_data->type, ret);
            goto err_put_vdd;
        }
    } else {
        mxinfo(pdev, "WARING: No VDD range set, default\n");
    }

    p_data->vio = regulator_get(pdev, "vio");
    if (IS_ERR(p_data->vio)) {
        ret = PTR_ERR(p_data->vio);
        mxerr(pdev, "Failed to get %s vio ret=%d\n", p_data->type, ret);
        goto err_restore_vdd;
    }

    if (regulator_count_voltages(p_data->vio) > 0) {
        ret = regulator_set_voltage(p_data->vio,
                M1120_VIO_MIN_UV,
                M1120_VIO_MAX_UV);
        if (ret) {
            mxerr(pdev, "Failed to set %s vio range ret=%d\n", p_data->type, ret);
            goto err_put_vio;
        }
    } else {
        mxinfo(pdev, "WARING: No VIO range set, default\n");
    }

    return 0;

err_put_vio:
    regulator_put(p_data->vio);
err_restore_vdd:
    if (regulator_count_voltages(p_data->vdd) > 0)
        regulator_set_voltage(p_data->vdd, 0, M1120_VDD_MAX_UV);
err_put_vdd:
    regulator_put(p_data->vdd);
exit:
    return ret;
}

static int m1120_parse_dt(struct i2c_client *client)
{
    u32 temp_val;
    int rc = 0;
    struct device* pdev = &client->dev;
    m1120_data_t *p_data = i2c_get_clientdata(client);
    struct device_node *np = client->dev.of_node;

    rc = of_property_read_string(np, "label", &p_data->type);
    if (rc) {
        mxerr(pdev, "Failed to get lable\n");
        goto exit;
    }

    rc = of_property_read_u32(np, "magnachip,init-interval", &temp_val);
    if (rc && (rc != -EINVAL)) {
        mxerr(pdev, "Failed to get lable\n");
        goto exit;
    }

    if(temp_val < M1120_DELAY_MIN) {
        temp_val = M1120_DELAY_MIN;
    }
    atomic_set(&p_data->atm.delay, temp_val);

    p_data->int_en = of_property_read_bool(np, "magnachip,use-interrupt");

    p_data->igpio = of_get_named_gpio_flags(pdev->of_node,
                "magnachip,gpio-int", 0, NULL);

    p_data->use_hrtimer = of_property_read_bool(np, "magnachip,use-hrtimer");

exit:
    return rc;
}

#define M1120_I2C_BUF_SIZE                (17)
static int m1120_i2c_read(struct i2c_client *client, u8 reg, u8 *rdata, u8 len)
{
    i2c_smbus_read_i2c_block_data(client, reg, len, rdata);
    return 0;
}

static int m1120_i2c_write(struct i2c_client *client, u8 reg, u8 *wdata, u8 len)
{
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8  buf[M1120_I2C_BUF_SIZE];
    int rc, i;

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = len+1,
            .buf = buf,
        },
    };

    buf[0] = reg;
    if (len > M1120_I2C_BUF_SIZE) {
        mxerr(&client->dev, "%s:i2c buffer size must be less than %d",
            p_data->type, M1120_I2C_BUF_SIZE);
        return -EIO;
    }
    for (i = 0 ; i < len; i++)
        buf[i+1] = wdata[i];

    rc = i2c_transfer(client->adapter, msg, 1);
    if (rc < 0) {
        mxerr(&client->dev, "%s: i2c_transfer was failed (%d)", p_data->type, rc);
        return rc;
    }

    if (len == 1) {
        switch (reg) {
        case M1120_REG_PERSINT:
            p_data->reg.map.persint = wdata[0];
            break;
        case M1120_REG_INTSRS:
            p_data->reg.map.intsrs = wdata[0];
            break;
        case M1120_REG_LTHL:
            p_data->reg.map.lthl = wdata[0];
            break;
        case M1120_REG_LTHH:
            p_data->reg.map.lthh = wdata[0];
            break;
        case M1120_REG_HTHL:
            p_data->reg.map.hthl = wdata[0];
            break;
        case M1120_REG_HTHH:
            p_data->reg.map.hthh = wdata[0];
            break;
        case M1120_REG_I2CDIS:
            p_data->reg.map.i2cdis = wdata[0];
            break;
        case M1120_REG_SRST:
            p_data->reg.map.srst = wdata[0];
            msleep(1);
            break;
        case M1120_REG_OPF:
            p_data->reg.map.opf = wdata[0];
            break;
        }
    }

#ifdef M1120_DBG_ENABLE
    for (i = 0; i < len; i++)
        dbg("reg=0x%02X data=0x%02X", buf[0]+(u8)i, buf[i+1]);
#endif

    return 0;
}

static int m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata)
{
    return m1120_i2c_write(client, reg, &wdata, sizeof(wdata));
}

static int m1120_clear_interrupt(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 wdata = p_data->reg.map.persint | 0x01;

    return m1120_i2c_set_reg(client, M1120_REG_PERSINT, wdata);
}

static int m1120_update_interrupt_threshold(struct device *dev, short raw)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 lthh = 0, lthl = 0, hthh = 0, hthl = 0;
    int err = 0;

    if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {

        dbg("reg.map.intsrs = 0x%02X", p_data->reg.map.intsrs);
        if (!(p_data->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_WITHIN)) {
            dbg("BESIDE raw = %d", raw);
            if ((raw >= -512) && (raw < p_data->thrhigh)) {
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrhigh, &hthh, &hthl);
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, -512, &lthh, &lthl);
            } else if ((raw >= p_data->thrlow) && (raw <= 511)) {
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrlow, &lthh, &lthl);
            }
        } else {
            // to do another condition
        }

        err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHH, hthh);
        if (err)
            return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHL, hthl);
        if (err)
            return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHH, lthh);
        if (err)
            return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHL, lthl);
        if (err)
            return err;

        dbg("threshold : (0x%02X%02X, 0x%02X%02X)\n", hthh, hthl, lthh, lthl);

        err = m1120_clear_interrupt(dev);
        if (err)
            return err;
    }

    return err;
}

static int m1120_set_operation_mode(struct device *dev, int mode)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 opf = p_data->reg.map.opf;
    int err = 0;

    switch (mode) {
    case OPERATION_MODE_POWERDOWN:
        if (p_data->irq_enabled) {

            /*disable irq*/
            disable_irq(p_data->irq);
            free_irq(p_data->irq, NULL);
            p_data->irq_enabled = 0;
        }
        opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
        err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
        mxinfo(dev, "%s: Changed to POWERDOWN", p_data->type);
        break;
    case OPERATION_MODE_MEASUREMENT:
        opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
        opf |= M1120_VAL_OPF_HSSON_ON;
        err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
        if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
            if (!p_data->irq_enabled) {
                /*enable irq*/
                err = request_irq(p_data->irq, &m1120_irq_handler,
                            IRQF_TRIGGER_FALLING, M1120_IRQ_NAME, client);
                if (err) {
                    mxerr(dev, "%s: request_irq was failed", p_data->type);
                    return err;
                }
                enable_irq(p_data->irq);
                p_data->irq_enabled = 1;
            }
        }
        mxinfo(dev, "%s: Changed to MEASUREMENT", p_data->type);
        break;
    case OPERATION_MODE_FUSEROMACCESS:
        opf |= M1120_VAL_OPF_EFRD_ON | M1120_VAL_OPF_HSSON_ON;
        err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
        mxinfo(dev, "%s Changed to FUSEROMACCESS", p_data->type);
        break;
    }

    return err;
}

static int m1120_set_detection_mode(struct device *dev, u8 mode)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 data;

    if (mode & M1120_DETECTION_MODE_INTERRUPT) {
        /*config threshold*/
        m1120_update_interrupt_threshold(dev, p_data->last_data);

        /*write intsrs*/
        data = p_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;
    } else {
        /*write intsrs*/
        data = p_data->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);
    }

    return m1120_i2c_set_reg(client, M1120_REG_INTSRS, data);
}

static int m1120_get_result_status(m1120_data_t *p_data, int raw)
{
    int status;

    if (p_data->thrhigh <= raw) {
        status = M1120_RESULT_STATUS_B;
    } else if (p_data->thrlow >= raw) {
        status = M1120_RESULT_STATUS_A;
    } else {
        status = p_data->last_data;
    }

    return status;
}

static int m1120_measure(m1120_data_t *p_data, short *raw)
{
    struct i2c_client *client = p_data->client;
    int err = 0;
    u8 buf[3];
    int st1_is_ok = 0;

    // (1) read data
    err = m1120_i2c_read(client, M1120_REG_ST1, buf, sizeof(buf));
    if (err)
        return err;

    // check st1 DRDY bit.
    if (buf[0] & 0x01) {
        st1_is_ok = 1;
    }

    if (st1_is_ok) {
        *raw = m1120_convdata_2byte_to_short(p_data->reg.map.opf, buf[2], buf[1]);
    } else {
        mxerr(&client->dev, "st1(0x%02X) is not DRDY", buf[0]);
        err = -1;
    }

    if (m1120_get_debug(&client->dev)) {
        mxinfo(&client->dev, "raw data (%d)\n", *raw);
    }

    return err;
}

static void m1120_func(m1120_data_t *p_data)
{
    ktime_t time_out = ms_to_ktime(m1120_get_delay(&p_data->client->dev));
    //unsigned long delay = msecs_to_jiffies(m1120_get_delay(&p_data->client->dev));
    short raw = 0;
    int err = 0;

    dbg_func_in();
    err = m1120_measure(p_data, &raw);

    if (!err) {
        if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
            p_data->last_data = m1120_get_result_status(p_data, raw);
        } else {
            ////James add here: polling mode:
            ////to call m1120_get_result_status(p_data, raw) get the status of Camera here.
            p_data->last_data = (int)raw;
        }

#if (M1120_EVENT_TYPE == EV_ABS)
        input_report_abs(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
#elif (M1120_EVENT_TYPE == EV_KEY)
        input_report_key(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

        input_sync(p_data->input_dev);
    }

    if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
        m1120_update_interrupt_threshold(&p_data->client->dev, raw);
    } else {
        if(atomic_read(&p_data->atm.enable))
            hrtimer_start(&p_data->sample_timer, time_out, HRTIMER_MODE_REL);
        //schedule_delayed_work(&p_data->work, delay);
        dbg("run schedule_delayed_work");
    }
}

static void m1120_work_func(struct work_struct *work)
{
    //m1120_data_t *p_data = container_of((struct delayed_work *)work, m1120_data_t, work);
    m1120_data_t *p_data = container_of(work, m1120_data_t, m1120_work);
    m1120_func(p_data);
}

static int m1120_reset_device(struct device *dev)
{
    int err = 0;
    u8  id = 0xFF, data = 0x00;

    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    /*(1) sw reset*/
    err = m1120_i2c_set_reg(client, M1120_REG_SRST, M1120_VAL_SRST_RESET);
    if (err) {
        mxerr(dev, "%s sw-reset was failed(%d)", p_data->type, err);
        goto exit;
    }
    msleep(5);

    /*(2) check id*/
    err = m1120_i2c_read(client, M1120_REG_DID, &id, sizeof(id));
    if (err < 0) {
        mxerr(dev, "Failed to get %s id", p_data->type);
        goto exit;
    }

    if (id != M1120_VAL_DID) {
        mxerr(dev, "current device id(0x%02X) is not M1120 device id(0x%02X)",
            id, M1120_VAL_DID);
        err = -ENXIO;
        goto exit;
    }

    /*(3) init variables*/
    /*(3-1) persint*/
    /* default consecutive interrupt events is 4 times. */
    data = M1120_PERSISTENCE_COUNT;
    err = m1120_i2c_write(client, M1120_REG_PERSINT, &data, sizeof(data));
    /*(3-2) intsrs*/
    /* don't enable interrupt, INTSRS register is set 0 */
    data = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
    if (data & M1120_DETECTION_MODE_INTERRUPT) {
        data |= M1120_INTERRUPT_TYPE;
    }
    err = m1120_i2c_write(client, M1120_REG_INTSRS, &data, sizeof(data));
    /*(3-3) opf*/
    /* 10bit, 80Hz */
    data = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;
    err = m1120_i2c_write(client, M1120_REG_OPF, &data, sizeof(data));

    /*(4) write variable to register*/
    /* polling mode */
    err = m1120_set_detection_mode(dev, M1120_DETECTION_MODE);
    if (err) {
        mxerr(dev, "%s: Failed to set detection mode (%d)",
            p_data->type, err);
        goto exit;
    }

    /*(5) set power-down mode*/
    err = m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);
    if (err) {
        mxerr(dev, "%s: Failed to set mode(%d)", p_data->type, err);
        goto exit;
    }

exit:
    return err;
}

static int m1120_init_device(struct i2c_client *client)
{
    int err = 0;
    struct device* pdev = &client->dev;
    m1120_data_t *p_data = i2c_get_clientdata(client);

    /*(1) vdd and vid power up*/
    err = m1120_power_ctl(client, true);
    if (err) {
        mxerr(pdev, "Failed to power up %s (%d)", p_data->type, err);
        return err;
    }

    /*(2) init variables*/
    atomic_set(&p_data->atm.enable, 0);
    atomic_set(&p_data->atm.delay, M1120_DELAY_MIN);
#ifdef M1120_DBG_ENABLE
    atomic_set(&p_data->atm.debug, 1);
#else
    atomic_set(&p_data->atm.debug, 0);
#endif
    p_data->calibrated_data = 0;
    p_data->last_data = 0;
    p_data->irq_enabled = 0;
    p_data->irq_first = 1;
    p_data->thrhigh = M1120_DETECT_RANGE_HIGH;
    p_data->thrlow = M1120_DETECT_RANGE_LOW;
    m1120_set_delay(pdev, M1120_DELAY_MAX);
    m1120_set_debug(pdev, 0);

    /*(3) reset registers*/
    err = m1120_reset_device(pdev);
    if (err) {
        mxerr(pdev, "Failed to reset %s (%d)", p_data->type, err);
        return err;
    }

    mxinfo(pdev, "%s initializing device was success", p_data->type);

    return 0;
}

/* input device interface */
static int m1120_input_dev_init(m1120_data_t *p_data)
{
    struct input_dev *dev;
    int err;

    dev = input_allocate_device();
    if (!dev) {
        return -ENOMEM;
    }
    dev->name = p_data->type;
    dev->id.bustype = BUS_I2C;

#if (M1120_EVENT_TYPE == EV_ABS)
    input_set_drvdata(dev, p_data);
    input_set_capability(dev, M1120_EVENT_TYPE, ABS_MISC);
    input_set_abs_params(dev, M1120_EVENT_CODE,
                M1120_EVENT_DATA_CAPABILITY_MIN,
                M1120_EVENT_DATA_CAPABILITY_MAX, 0, 0);
#elif (M1120_EVENT_TYPE == EV_KEY)
    input_set_drvdata(dev, p_data);
    input_set_capability(dev, M1120_EVENT_TYPE, M1120_EVENT_CODE);
#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        return err;
    }

    p_data->input_dev = dev;

    return 0;
}

static void m1120_input_dev_terminate(m1120_data_t *p_data)
{
    struct input_dev *dev = p_data->input_dev;

    input_unregister_device(dev);
    input_free_device(dev);
}

static void m1120_get_reg(struct device *dev, int *regdata)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err;

    u8 rega = (((*regdata) >> 8) & 0xFF);
    u8 regd = 0;
    err = m1120_i2c_read(client, rega, &regd, 1);

    *regdata = 0;
    *regdata |= (err == 0) ? 0x0000 : 0xFF00;
    *regdata |= regd;
}

/* sysfs group interface */
static ssize_t m1120_enable_show(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;

    return snprintf(buf, 20, "%d\n", m1120_get_enable(cdev));
}

static ssize_t m1120_enable_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;
    unsigned enable = 0;

    if(kstrtouint(buf, 10, &enable)) {
        mxerr(dev, "Error value: %s\n", buf);
        goto err;
    }

    enable = !!enable;

    if(enable == atomic_read(&p_data->atm.enable)) {
        mxinfo(cdev, "ignore duplicate set\n");
        goto err;
    }

    m1120_set_enable(cdev, enable);
err:
    return count;
}

static ssize_t m1120_delay_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;

    return snprintf(buf, 20, "%d\n", m1120_get_delay(cdev));
}

static ssize_t m1120_delay_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;
    unsigned delay = 0;

    if(kstrtouint(buf, 10, &delay)) {
        mxerr(dev, "Error value: %s\n", buf);
        goto err;
    }

    if (delay > M1120_DELAY_MAX) {
        delay = M1120_DELAY_MAX;
    }

    if(delay == atomic_read(&p_data->atm.delay)) {
        mxinfo(cdev, "ignore duplicate set\n");
        goto err;
    }

    m1120_set_delay(cdev, delay);

err:
    return count;
}

static ssize_t m1120_debug_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;

    return snprintf(buf, 20, "%d\n", m1120_get_debug(cdev));
}

static ssize_t m1120_debug_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;
    unsigned debug = 0;

    if(kstrtouint(buf, 10, &debug)) {
        mxerr(dev, "Error value: %s\n", buf);
        goto err;
    }

    debug = !!debug;

    if(debug == atomic_read(&p_data->atm.debug)) {
        mxinfo(cdev, "ignore duplicate set\n");
        goto err;
    }

    m1120_set_debug(dev, debug);

err:
    return count;
}

static ssize_t m1120_data_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    short raw = 0;

    m1120_measure(p_data, &raw);
    return snprintf(buf, 10, "%d\n", raw);
}

static enum hrtimer_restart mx1120_timer_action(struct hrtimer *h)
{
    m1120_data_t * p_data = container_of(h, m1120_data_t, sample_timer);

    //queue_work(p_data->m1120_wq, &p_data->m1120_work);
    //schedule_delayed_work(&p_data->work, 0);

    p_data->sync_flag = true;
    wake_up(&p_data->sync_complete);

    return HRTIMER_NORESTART;
}

static __ref int m1120_thread(void *arg)
{
    m1120_data_t * p_data = (m1120_data_t*)arg;
    int ret = 0;

    while (!kthread_should_stop()) {
        do {
            ret = wait_event_interruptible(p_data->sync_complete,
                        p_data->sync_flag || kthread_should_stop());
        } while (ret != 0);

        if(kthread_should_stop()) {
            break;
        }

        p_data->sync_flag = false;

        m1120_func(p_data);
    }

    return 0;
}

static ssize_t m1120_dump_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    m1120_data_t * p_data = (m1120_data_t*)dev_get_drvdata(dev);
    struct i2c_client* client = p_data->client;
    struct device *cdev = &client->dev;
    int reg = 0;
    int reg_l = M1120_REG_HSL;
    int reg_h = M1120_REG_HSH;
    int i = 0;

    for (i = 0; i < 11; i++) {
        reg = i<<8;
        m1120_get_reg(cdev, &reg);
        printk(KERN_ERR"dkk: the reg 0x%02X value: 0x%02X\n", i, reg);
    }

    m1120_get_reg(cdev, &reg_l);
    printk(KERN_ERR"dkk: the reg_l is 0x%02X\n", (u8)(reg_l&0xFF));

    m1120_get_reg(cdev, &reg_h);
    printk(KERN_ERR"dkk: the reg_h is 0x%02X", (u8)(reg_h&0xFF));

    reg = ((reg_h&0xC0) << 2)|reg_l;
    printk(KERN_ERR"dkk: the up hall reg measure is 0x%02X\n", reg);

    return snprintf(buf, 10, "%d\n", reg);
}

static DEVICE_ATTR(enable,  S_IRUGO|S_IWUSR|S_IWGRP, m1120_enable_show, m1120_enable_store);
static DEVICE_ATTR(delay,   S_IRUGO|S_IWUSR|S_IWGRP, m1120_delay_show,  m1120_delay_store);
static DEVICE_ATTR(debug,   S_IRUGO|S_IWUSR|S_IWGRP, m1120_debug_show,  m1120_debug_store);
static DEVICE_ATTR(rawdata, S_IRUGO|S_IWUSR|S_IWGRP, m1120_data_show,   NULL);
static DEVICE_ATTR(dump,    S_IRUGO|S_IWUSR|S_IWGRP, m1120_dump_show,   NULL);

static struct attribute *m1120_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_debug.attr,
    &dev_attr_rawdata.attr,
    &dev_attr_dump.attr,
    NULL
};

static struct attribute_group m1120_attribute_group = {
    .attrs = m1120_attributes
};

static const struct attribute_group * m1120_attr_groups[] = {
    &m1120_attribute_group,
    NULL
};

static int m1120_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    m1120_data_t *p_data = NULL;

    dbg_func_in();

    p_data = kzalloc(sizeof(m1120_data_t), GFP_KERNEL);
    if (!p_data) {
        mxerr(&client->dev, "kernel memory alocation was failed");
        err = -ENOMEM;
        goto err_nomem;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        mxerr(&client->dev, "i2c_check_functionality was failed");
        err = -ENODEV;
        goto err_nodev;
    }

    mutex_init(&p_data->mtx.enable);
    mutex_init(&p_data->mtx.data);
    p_data->power_enabled = false;
    p_data->client = client;

    i2c_set_clientdata(client, p_data);

    err = m1120_parse_dt(client);
    if (err) {
        mxerr(&client->dev, "Failed to parse device tree\n");
        err = -ENODEV;
        goto err_nodev;
    }

    err = m1120_power_init(client);
    if (err) {
        mxerr(&client->dev, "Failed to get sensor regulators\n");
        err = -EINVAL;
        goto err_nodev;
    }

    err = m1120_init_device(client);
    if (err) {
        mxerr(&client->dev, "m1120_init_device was failed(%d)", err);
        goto err_nodev;
    }
    mxinfo(&client->dev, "%s was found", p_data->type);

    /*(7) config work function*/
    INIT_DELAYED_WORK(&p_data->work, m1120_work_func);
    INIT_WORK(&p_data->m1120_work, m1120_work_func);
    p_data->m1120_wq = alloc_workqueue("mx1120_wq", WQ_HIGHPRI, 0);
    if(!p_data->m1120_wq) {
        mxerr(&client->dev, "failed alloc wq");
        goto err_nodev;
    }
    p_data->sync_flag = false;
    init_waitqueue_head(&p_data->sync_complete);
    p_data->m1120_task = kthread_create(m1120_thread, p_data, "%s_task", p_data->type);
    if (IS_ERR(p_data->m1120_task)) {
        mxerr(&client->dev, "failed create %s task", p_data->type);
        goto err_wk;
    }
    wake_up_process(p_data->m1120_task);

    hrtimer_init(&p_data->sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    p_data->sample_timer.function = mx1120_timer_action;

    /*(8) init input device*/
    err = m1120_input_dev_init(p_data);
    if (err) {
        mxerr(&client->dev, "%s: Failed to create input dev(%d)", p_data->type, err);
        goto err_task;
    }
    mxinfo(&client->dev, "%s was initialized", p_data->type);

    /*(9) create sysfs group*/
#if 0
    err = sysfs_create_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
    if (err) {
        mxerr(&client->dev, "sysfs_create_group was failed(%d)", err);
        goto err_group;
    }
#endif

    p_data->m1120_class = class_create(THIS_MODULE, p_data->type);
    if(IS_ERR(p_data->m1120_class)) {
        mxerr(&client->dev, "Failed to create class\n");
        goto err_group;
    }

    p_data->sysfs_dev = device_create_with_groups(p_data->m1120_class,
                 &client->dev, MKDEV(0, 0), p_data, m1120_attr_groups,
                 "%s", M1120_CONTROL);
    if(IS_ERR(p_data->sysfs_dev)) {
        mxerr(&client->dev, "Failed to create device\n");
        goto err_groups;
    }
    return 0;

err_groups:
    class_destroy(p_data->m1120_class);
err_group:
    m1120_input_dev_terminate(p_data);
err_task:
    kthread_stop(p_data->m1120_task);
err_wk:
    destroy_workqueue(p_data->m1120_wq);
err_nodev:
    kfree(p_data);
err_nomem:
    p_data = NULL;
    return err;
}

static int m1120_remove(struct i2c_client *client)
{
    m1120_data_t *p_data = i2c_get_clientdata(client);

    m1120_set_enable(&client->dev, 0);
    destroy_workqueue(p_data->m1120_wq);
    kthread_stop(p_data->m1120_task);
    regulator_put(p_data->vdd);
    regulator_put(p_data->vio);
    device_destroy(p_data->m1120_class, MKDEV(0, 0));
    class_destroy(p_data->m1120_class);
#if 0
    sysfs_remove_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
#endif
    m1120_input_dev_terminate(p_data);
    if (p_data->igpio != -1) {
        gpio_free(p_data->igpio);
    }
    kfree(p_data);

    return 0;
}

static const struct i2c_device_id m1120_i2c_drv_id_table[] = {
    {M1120_DRIVER_NAME, 0 },
    { }
};

static const struct of_device_id m1120_of_match[] = {
    { .compatible = "magnachip,mxm1120", },
    { },
};

static struct i2c_driver m1120_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = M1120_DRIVER_NAME,
        .of_match_table = m1120_of_match,
    },
    .probe    = m1120_probe,
    .remove  = m1120_remove,
    .id_table   = m1120_i2c_drv_id_table,
    //.suspend  = m1120_i2c_drv_suspend,
    //.resume      = m1120_i2c_drv_resume,
};

static int __init m1120_driver_init(void)
{
    return i2c_add_driver(&m1120_driver);
}
module_init(m1120_driver_init);

static void __exit m1120_driver_exit(void)
{
    i2c_del_driver(&m1120_driver);
}
module_exit(m1120_driver_exit);

MODULE_AUTHOR("shpark <seunghwan.park@magnachip.com>");
MODULE_AUTHOR("zoujc <zoujc@lenovo.com>");
MODULE_VERSION(MOTO_M1120_DRIVER_VERSION);
MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");
