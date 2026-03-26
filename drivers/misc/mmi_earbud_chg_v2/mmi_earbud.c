#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include "mmi_earbud.h"

#define DRIVER_NAME          "mmi_earbud_chg"
#define ENABLE_CHG_NAME      "enable-uart-chg"
#define ENABLE_LCHG_NAME     "enable-lchg"
#define ENABLE_RCHG_NAME     "enable-rchg"
#define CHG_IRQ_NAME         "chg-irq"
#define DEFAULT_HB_INTERVAL  10000 /* milliseconds */
#define DEFAULT_I_CHG        4 /* millisamps */

#define MMI_CHG_STATE_HB_DELAY_MS 0
#define EBUD_PRESENCE_ADC_THRESHOLD 30

static struct mmi_earbud_chg_data *g_earbud_chg_pdata = NULL;

struct ebud_tlv rx_data = {0};

int mmi_register_ichg_chip(struct mmi_ichg_chip *chip) {
    if (!g_earbud_chg_pdata) return -EPROBE_DEFER;
    g_earbud_chg_pdata->ichg_chip = chip;
    pr_info("%s: MMI earbud chip registration done", __func__);
    return 0;
}
EXPORT_SYMBOL(mmi_register_ichg_chip);

static void mmi_enable_chg(struct mmi_earbud_chg_data *pdata, bool enable) {
    pdata->enable_chg = enable;
    gpio_set_value(pdata->en_chg_gpio, pdata->enable_chg);
    pr_info("%s: Get GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_chg_gpio));
}

static void mmi_enable_lchg(struct mmi_earbud_chg_data *pdata, bool enable) {
    pdata->enable_lchg = enable;
    gpio_set_value(pdata->en_lchg_gpio, pdata->enable_lchg);
    pr_info("%s: Get LCHG GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_lchg_gpio));
}
static void mmi_enable_rchg(struct mmi_earbud_chg_data *pdata, bool enable) {
    pdata->enable_rchg = enable;
    gpio_set_value(pdata->en_rchg_gpio, pdata->enable_rchg);
    pr_info("%s: Get RCHG GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_rchg_gpio));
}

static int mmi_adc_average(struct mmi_earbud_chg_data *pdata, enum chip_idx idx, int *out_adc) {
    int ret = 0, count = 0, avg = 0, chg_enbflag = 0;

    if(g_earbud_chg_pdata->ichg_chip == NULL) return -1;

    if((idx == LEFT) && (!pdata->enable_lchg)) {
        mmi_earbud_lchg_enable(pdata, EBUD_GPIO_ENABLE);
        chg_enbflag = EBUD_GPIO_ENABLE;
    }
    else if((idx == RIGHT) && (!pdata->enable_rchg)) {
        mmi_earbud_rchg_enable(pdata, EBUD_GPIO_ENABLE);
        chg_enbflag = EBUD_GPIO_ENABLE;
    }

    for(count =0; count < 10; count++) {
        ret = g_earbud_chg_pdata->ichg_chip->read_ichg(g_earbud_chg_pdata->ichg_chip->pdata, idx, out_adc);
        mdelay(10);
        avg += *out_adc;
    }
    *out_adc = (int)(avg/10);
    if((idx == LEFT) && (chg_enbflag))
       mmi_earbud_lchg_enable(pdata, EBUD_GPIO_DISABLE);
    else if((idx == RIGHT) && (chg_enbflag))
       mmi_earbud_rchg_enable(pdata, EBUD_GPIO_DISABLE);

    return ret;
}

static void mmi_update_chg_state(
    struct mmi_earbud_chg_data *pdata,
    enum charging_state new_state
) {
    pr_info("%s: update chg state : %d", __func__, new_state);
    pdata->chg_state = new_state;
    switch(pdata->chg_state) {
        case NO_CHG:
            mmi_enable_chg(pdata, 0);
            mmi_enable_lchg(pdata, 0);
            mmi_enable_rchg(pdata, 0);
            break;
        case L_CHG:
            mmi_enable_chg(pdata, 0);
            mmi_enable_lchg(pdata, 1);
            mmi_enable_rchg(pdata, 0);
            break;
        case R_CHG:
            mmi_enable_chg(pdata, 0);
            mmi_enable_lchg(pdata, 0);
            mmi_enable_rchg(pdata, 1);
            break;
        case LR_CHG:
            mmi_enable_chg(pdata, 0);
            mmi_enable_lchg(pdata, 1);
            mmi_enable_rchg(pdata, 1);
	    break;
        default:
            break;
    }
}

static void mmi_earbud_chg_hb_work(struct work_struct *work) {
    struct mmi_earbud_chg_data *pdata = container_of(
                                            work,
                                            struct mmi_earbud_chg_data,
                                            heartbeat_work.work);
    int should_work;
    enum charging_state new_state = pdata->chg_state;
    int lval = 0, lret = 0, rret = 0;
    int rval = 0, irqval = 0, lbud_avail = 0, rbud_avail = 0;
    if(pdata->user_opt) {
         schedule_delayed_work(&pdata->heartbeat_work,
            msecs_to_jiffies(pdata->hb_interval));
         return;
    }

    mutex_lock(&pdata->lock);
    mmi_earbud_get_irq_gpio_state(pdata, &irqval);

    lret = mmi_adc_average(pdata, LEFT, &lval);
    rret = mmi_adc_average(pdata, RIGHT, &rval);
    if(lret < 0 || rret < 0) {
        dev_err(pdata->dev, "%s: adc data read raw lval %d  rval %d lret %d rret %d", __func__, lval, rval, lret, rret);
    }
    if(lval > EBUD_PRESENCE_ADC_THRESHOLD) lbud_avail = 1;
    if(rval > EBUD_PRESENCE_ADC_THRESHOLD) rbud_avail = 1;

    if(irqval == EBUD_CASE_CLOSED) {
        if(!lbud_avail && !rbud_avail) {
            // Case closed and no earbud don't schedule the queue..
            pr_info("%s: No Earbud present\n", __func__);
            pdata->c_state = CLOSED;
            goto end;
        }
        // Issue Bud closed command to both Left/Right Bud
        if(pdata->c_state != CLOSED) {
            if(lbud_avail)lret = mmi_uart_tx(pdata, EBUD_CMD_INQUIRY_LID_CLOSE, LEFT);
            if(rbud_avail)rret = mmi_uart_tx(pdata, EBUD_CMD_INQUIRY_LID_CLOSE, RIGHT);
        } else  pr_info("%s: Already in closed state\n", __func__);
        pdata->c_state = CLOSED;
    } else if(irqval == EBUD_CASE_OPENED) {
        // Issue Bud Open Command to both Left/Right Bud
        if(pdata->c_state != OPEN) {
            if(lbud_avail)lret = mmi_uart_tx(pdata, EBUD_CMD_INQUIRY_LID_OPEN, LEFT);
            if(rbud_avail)rret = mmi_uart_tx(pdata, EBUD_CMD_INQUIRY_LID_OPEN, RIGHT);
        } else pr_info("%s: Already in opened state\n", __func__);
        pdata->c_state = OPEN;
    }

    switch (pdata->chg_state) {
        case NO_CHG:
            should_work = 0;
            break;
        case L_CHG:
            should_work = 1;
            if (lval < pdata->termination_current) {
                new_state = NO_CHG;
            }
            break;
        case R_CHG:
            should_work = 1;
            if (rval < pdata->termination_current) {
                new_state = NO_CHG;
            }
            break;
        case LR_CHG:
            should_work = 1;
            if (lval < pdata->termination_current &&
                rval > pdata->termination_current) {
                new_state = R_CHG;
            }
            else if (lval > pdata->termination_current &&
                rval < pdata->termination_current) {
                new_state = L_CHG;
            }
            else if (lval < pdata->termination_current &&
                rval < pdata->termination_current) {
                new_state = NO_CHG;
            }
            break;
        default:
            break;
    }

    if (pdata->chg_state != new_state) {
        mmi_update_chg_state(pdata, new_state);
    }

    if (should_work) {
        schedule_delayed_work(&pdata->heartbeat_work,
            msecs_to_jiffies(pdata->hb_interval));
    }
end:
    mutex_unlock(&pdata->lock);
}

static void mmi_change_hb_interval(
    struct mmi_earbud_chg_data *pdata,
    int interval
) {
    pdata->hb_interval = interval;
    cancel_delayed_work(&pdata->heartbeat_work);
    schedule_delayed_work(&pdata->heartbeat_work,
        msecs_to_jiffies(pdata->hb_interval));
}

static void mmi_set_chg_state(
    struct mmi_earbud_chg_data *pdata,
    enum charging_state chg_state
) {
    mmi_update_chg_state(pdata, chg_state);
    cancel_delayed_work(&pdata->heartbeat_work);
    schedule_delayed_work(&pdata->heartbeat_work, msecs_to_jiffies(MMI_CHG_STATE_HB_DELAY_MS));
}

static void mmi_trigger_chg(struct mmi_earbud_chg_data *pdata) {
    schedule_delayed_work(&pdata->heartbeat_work, msecs_to_jiffies(0));
}

static int mmi_get_termination_current(struct mmi_earbud_chg_data *pdata) {
    return pdata->termination_current;
}

static void mmi_set_termination_current(
    struct mmi_earbud_chg_data *pdata,
    int i_chg
) {
    pdata->termination_current = i_chg;
}

static ssize_t hb_interval_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    int interval;

    if (kstrtoint(buf, 0, &interval) || interval < 0) {
        return -EINVAL;
    }

    mmi_change_hb_interval(pdata, interval);

    return count;
}

static ssize_t hb_interval_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", pdata->hb_interval);
}
DEVICE_ATTR_RW(hb_interval);

static ssize_t charging_state_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    int chg_state;

    if (kstrtoint(buf, 0, &chg_state)) {
        return -EINVAL;
    }

    if (chg_state < NO_CHG || chg_state > LR_CHG) {
        return -EINVAL;
    }

    mmi_set_chg_state(pdata, chg_state);

    return count;
}

static ssize_t charging_state_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", pdata->chg_state);
}
DEVICE_ATTR_RW(charging_state);

static ssize_t trigger_charging_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    bool trigger;

    if (kstrtobool(buf, &trigger)) {
        return -EINVAL;
    }

    mmi_trigger_chg(pdata);

    return count;
}
DEVICE_ATTR_WO(trigger_charging);

static ssize_t enable_charging_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    bool en_chg;

    if (kstrtobool(buf, &en_chg)) {
        return -EINVAL;
    }

    mmi_enable_chg(pdata, en_chg);

    return count;
}
DEVICE_ATTR_WO(enable_charging);

static ssize_t enable_lcharging_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    bool en_chg;

    if (kstrtobool(buf, &en_chg)) {
        return -EINVAL;
    }

    mmi_enable_lchg(pdata, en_chg);

    return count;
}
DEVICE_ATTR_WO(enable_lcharging);

static ssize_t enable_rcharging_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    bool en_chg;

    if (kstrtobool(buf, &en_chg)) {
        return -EINVAL;
    }

    mmi_enable_rchg(pdata, en_chg);

    return count;
}
DEVICE_ATTR_WO(enable_rcharging);

static ssize_t termination_current_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    int i_chg;

    if (kstrtoint(buf, 0, &i_chg) || i_chg < 0) {
        return -EINVAL;
    }

    mmi_set_termination_current(pdata, i_chg);

    return count;
}

static ssize_t termination_current_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    int i_chg;

    i_chg = mmi_get_termination_current(pdata);

    return sprintf(buf, "%d\n", i_chg);
}
DEVICE_ATTR_RW(termination_current);

static ssize_t uart_comm_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    int cmd = 0, channel = 0;
    int ret = 0;
    pr_info("%s: Entered", __func__);
    // Get Channel No and Command
    if (sscanf(buf, "%d %d", &channel, &cmd) == 2) {
        dev_info(dev, ": %d and %d\n", channel, cmd);
    } else {
        dev_err(dev, "Requires two input Channel NO and command");
        return -EINVAL;
    }
    if( (channel < LEFT) || (channel > RIGHT) ) return -EINVAL;
    if( (cmd < EBUD_CMD_PARING) || (cmd > EBUD_CMDSET_COUNT) ) return -EINVAL;
    pdata->user_opt = 1;
    mutex_lock(&pdata->lock);
    ret = mmi_uart_tx(pdata, cmd, channel);
    mutex_unlock(&pdata->lock);
    pdata->user_opt = 0;
    pr_info("%s: Successfully transfered %d", __func__, ret);

    return count;
}

static ssize_t uart_comm_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
) {
    struct mmi_earbud_chg_data *pdata = dev_get_drvdata(dev);
    char recv[64];
    int size = 0, len = 0, cnt = 0, ret = 0;
    mutex_lock(&pdata->lock);
    ret = mmi_uart_rx(pdata, recv, &size);
    if (ret >= 0 ) {
        for(cnt =0; cnt < size; cnt++)
            len += scnprintf(buf + len, PAGE_SIZE - len, "0x%X ", recv[cnt]);
    }
    mutex_unlock(&pdata->lock);
    return len;
}

DEVICE_ATTR_RW(uart_comm);

static struct attribute *mmi_earbud_chg_attrs[] = {
    &dev_attr_uart_comm.attr,
    &dev_attr_enable_charging.attr,
    &dev_attr_enable_lcharging.attr,
    &dev_attr_enable_rcharging.attr,
    &dev_attr_termination_current.attr,
    &dev_attr_charging_state.attr,
    &dev_attr_trigger_charging.attr,
    &dev_attr_hb_interval.attr,
    NULL
};

static const struct attribute_group mmi_earbud_chg_group = {
    .attrs = mmi_earbud_chg_attrs,
};

static int mmi_earbud_chg_init_gpio(struct mmi_earbud_chg_data *pdata) {
    int rc;
    struct device *dev = pdata->dev;
    struct device_node *np = dev->of_node;

    pdata->en_chg_gpio = of_get_named_gpio(np, "enable-uart-chg-gpios", 0);
    if (pdata->en_chg_gpio < 0) {
        dev_err(dev, "%s : failed to get enable-uart-chg-gpios dt node: %d", __func__, pdata->en_chg_gpio);
	return pdata->en_chg_gpio;
    }
    if (gpio_is_valid(pdata->en_chg_gpio)) {
        rc = devm_gpio_request(dev, pdata->en_chg_gpio, "enable-uart-chg-gpios");
        if (rc) {
            dev_err(dev, "%s : failed to request en chg gpio: %d", __func__, rc);
            return rc;
        }

        rc = gpio_direction_output(pdata->en_chg_gpio, 0);
        if (rc) {
           dev_err(dev, "%s: err to set gpio: %d", __func__, rc);
        }
    }
    pdata->en_lchg_gpio = of_get_named_gpio(np, "enable-lchg-gpios", 0);
    if (pdata->en_lchg_gpio < 0) {
        dev_err(dev, "%s : failed to get enable-lchg-gpios dt node: %d", __func__, pdata->en_lchg_gpio);
        return pdata->en_lchg_gpio;
    }
    if (gpio_is_valid(pdata->en_lchg_gpio)) {
        rc = devm_gpio_request(dev, pdata->en_lchg_gpio, "mmi_lchg_en_pin");
        if (rc) {
            dev_err(dev, "%s : failed to request en lchg gpio: %d", __func__, rc);
            return rc;
        }

        rc = gpio_direction_output(pdata->en_lchg_gpio, 0);
        if (rc) {
           dev_err(dev, "%s: err to set gpio: %d", __func__, rc);
        }
    }
    pdata->en_rchg_gpio = of_get_named_gpio(np, "enable-rchg-gpios", 0);
    if (pdata->en_rchg_gpio < 0) {
        dev_err(dev, "%s : failed to get enable-rchg-gpios dt node: %d", __func__, pdata->en_rchg_gpio);
        return pdata->en_rchg_gpio;
    }

    if (gpio_is_valid(pdata->en_rchg_gpio)) {
        rc = devm_gpio_request(dev, pdata->en_rchg_gpio, "mmi_rchg_en_pin");
        if (rc) {
            dev_err(dev, "%s : failed to request en rchg gpio: %d", __func__, rc);
            return rc;
        }
	rc = gpio_direction_output(pdata->en_rchg_gpio, 0);
        if (rc) {
           dev_err(dev, "%s: err to set gpio: %d", __func__, rc);
        }
    }

    return 0;
}

static irqreturn_t mmi_earbud_chg_irq_handler(int irq, void *data) {
    struct mmi_earbud_chg_data *pdata = data;
    int lid_state;

    lid_state = gpiod_get_value(pdata->chg_irq_gpio);
    mmi_set_chg_state(pdata, (lid_state==EBUD_CASE_CLOSED)? LR_CHG : NO_CHG);

    return IRQ_HANDLED;
}

static int mmi_earbud_chg_init_irq(struct mmi_earbud_chg_data *pdata) {
    int rc;
    struct device *dev = pdata->dev;
    struct gpio_desc *desc;
    int irq_num;

    desc = devm_gpiod_get(dev, CHG_IRQ_NAME, GPIOD_IN);
    if (IS_ERR(desc)) {
        rc = PTR_ERR(ERR_CAST(desc));
        pr_err("%s : failed to get IRQ GPIO: %d", __func__, rc);
        return rc;
    }

    irq_num = gpiod_to_irq(desc);
    if (irq_num < 0) {
        pr_err("%s : failed to get IRQ number: %d", __func__, irq_num);
        return irq_num;
    }

    rc = devm_request_irq(
        dev,
        irq_num,
        &mmi_earbud_chg_irq_handler,
        (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
        "en-chg-irq",
        pdata
    );
    if (rc < 0) {
        pr_err("%s : failed to request IRQ: %d", __func__, rc);
        return rc;
    }

    pdata->chg_irq_gpio = desc;
    pdata->chg_irq = irq_num;

    return 0;
}

static int mmi_earbud_chg_probe(struct platform_device *pdev) {
    int rc;
    int hb_interval = 0;
    int ichg = 0, lid_state = 0;
    struct device *dev = &pdev->dev;
    struct mmi_earbud_chg_data *pdata;

    pr_info("%s : start", __func__);

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        pr_err("%s : failed to allocate memory", __func__);
        return -ENOMEM;
    }

    pdata->dev = dev;
    g_earbud_chg_pdata = pdata;
    g_earbud_chg_pdata->ichg_chip = NULL;
    platform_set_drvdata(pdev, pdata);

    rc = mmi_earbud_chg_init_gpio(pdata);
    if (rc) {
        pr_err("%s : failed to get GPIOs : %d", __func__, rc);
        return rc;
    }

    rc = mmi_earbud_chg_init_irq(pdata);
    if (rc) {
        pr_err("%s : failed to set IRQ : %d", __func__, rc);
        return rc;
    }

    of_property_read_u32(dev->of_node, "mmi,hb-interval", &hb_interval);
    if (hb_interval <= 0) {
        pdata->hb_interval = DEFAULT_HB_INTERVAL;
    }
    else {
        pdata->hb_interval = hb_interval;
    }

    of_property_read_u32(dev->of_node, "mmi,t-ichg", &ichg);
    if (ichg <= 0) {
        pdata->termination_current = DEFAULT_I_CHG;
    }
    else {
        pdata->termination_current = ichg;
    }

    rc = of_property_read_string(dev->of_node, "mmi,uart-dev-lchannel", &pdata->dev_lchan);
    if(rc) {
        pr_err("%s : Failed to get L Channel device node", __func__);
        return rc;
    } else pr_info("%s: lchannel %s", __func__, pdata->dev_lchan);

    rc = of_property_read_string(dev->of_node, "mmi,uart-dev-rchannel", &pdata->dev_rchan);
    if(rc) {
        pr_err("%s : Failed to get R Channel device node", __func__);
        return rc;
    } else pr_info("%s: rchannel %s", __func__, pdata->dev_rchan);

    rc = sysfs_create_group(&dev->kobj, &mmi_earbud_chg_group);
    if (rc) {
        pr_err("%s : failed to create sysfs group: %d", __func__, rc);
        return rc;
    }
    mutex_init(&pdata->lock);
    INIT_DELAYED_WORK(&pdata->heartbeat_work, mmi_earbud_chg_hb_work);
    lid_state = gpiod_get_value(pdata->chg_irq_gpio);
    mmi_set_chg_state(pdata, (lid_state==EBUD_CASE_CLOSED)? LR_CHG : NO_CHG);

    pr_info("%s : finished", __func__);

    return rc;
}

static void mmi_earbud_chg_remove(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct mmi_earbud_chg_data *pdata = platform_get_drvdata(pdev);
    cancel_delayed_work_sync(&pdata->heartbeat_work);
    sysfs_remove_group(&dev->kobj, &mmi_earbud_chg_group);
    g_earbud_chg_pdata = NULL;
}

int mmi_earbud_suspend(struct platform_device *pdev, pm_message_t state) {
    struct mmi_earbud_chg_data *pdata = platform_get_drvdata(pdev);
    pr_info("%s device suspend\n", __func__);
    enable_irq_wake(pdata->chg_irq);
    return 0;
}

int mmi_earbud_resume(struct platform_device *pdev) {
    struct mmi_earbud_chg_data *pdata = platform_get_drvdata(pdev);
    pr_info("%s device resume\n", __func__);
    disable_irq_wake(pdata->chg_irq);
    return 0;
}

static const struct of_device_id of_mmi_earbud_chg_match[] = {
    { .compatible = "mmi,earbud_chg", },
    { },
};
MODULE_DEVICE_TABLE(of, of_mmi_earbud_chg_match);

static struct platform_driver mmi_earbud_chg_driver = {
    .probe = mmi_earbud_chg_probe,
    .remove = mmi_earbud_chg_remove,
    .suspend = mmi_earbud_suspend,
    .resume = mmi_earbud_resume,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = of_mmi_earbud_chg_match,
    },
};
module_platform_driver(mmi_earbud_chg_driver);

MODULE_DESCRIPTION("Motorola earbud charging Driver");
MODULE_LICENSE("GPL v2");
