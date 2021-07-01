/**
 * plat-mt6768.c
 *
**/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/clk.h>

#include "ff_log.h"
#include "ff_err.h"

# undef LOG_TAG
#define LOG_TAG "mt6768"

/* TODO: */
#define FF_COMPATIBLE_NODE_1 "mediatek,focaltech-fp"
//#define FF_COMPATIBLE_NODE_2 "mediatek,goodix-fp"

/* Define pinctrl state types. */
typedef enum {
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fingerprint_reset_low",
    "fingerprint_reset_high",
};

/* Native context and its singleton instance. */
typedef struct {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
    bool b_spiclk_enabled;
} ff_mt6768_context_t;
static ff_mt6768_context_t ff_mt6768_context, *g_context = &ff_mt6768_context;

/* ... */
extern struct spi_device *g_spidev;
extern void mt_spi_enable_master_clk(struct spi_device *ms);
extern void mt_spi_disable_master_clk(struct spi_device *ms);

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0, i;
    int gpio;
    struct device_node *dev_node = NULL;
    struct platform_device *pdev = NULL;
    FF_LOGV("'%s' enter.", __func__);

    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_1);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_1);
        return (-ENODEV);
    }

    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGE("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        FF_LOGE("devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            FF_LOGE("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }

    /* Initialize the INT pin. */
    //err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT]);

    /* Retrieve the irq number. */
//    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_2);
     gpio = of_get_named_gpio(dev_node, "int-gpios", 0);
     *irq_num = gpio_to_irq(gpio);
/*
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_2);
        return (-ENODEV);
    }

    *irq_num = irq_of_parse_and_map(dev_node, 0);
*/

    FF_LOGI("irq number is %d.", *irq_num);

    /* Initialize the RESET pin. */
    pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
	
	if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

	devm_pinctrl_put(g_context->pinctrl);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

    if (unlikely(!g_spidev)) {
        return (-ENOSYS);
    }

#if 0
    struct mt_spi_t *ms = NULL;

    ms = spi_master_get_devdata(g_spidev->master);
    if (!ms) {
        FF_LOGE("spi_master_get_devdata(..) = '%d'.", FF_ERR_NULL_PTR);
        return FF_ERR_NULL_PTR;
    }

    /* Control the clock source. */
    if (on && !g_context->b_spiclk_enabled) {
        mt_spi_enable_clk(ms);
        g_context->b_spiclk_enabled = true;
    } else if (!on && g_context->b_spiclk_enabled) {
        mt_spi_disable_clk(ms);
        g_context->b_spiclk_enabled = false;
    }
#else
	/* Control the clock source. */
    if (on && !g_context->b_spiclk_enabled) {
		mt_spi_enable_master_clk(g_spidev);
        g_context->b_spiclk_enabled = true;
    } else if (!on && g_context->b_spiclk_enabled) {
		mt_spi_disable_master_clk(g_spidev);
        g_context->b_spiclk_enabled = false;
    }
#endif

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("power: '%s'.", on ? "on" : "off");

    /*
    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

    if (on) {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_POWER_ON]);
    } else {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_POWER_OFF]);
    }
    */

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_reset_device(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

    /* 3-1: Pull down RST pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);

    /* 3-2: Delay for 10ms. */
    mdelay(10);

    /* Pull up RST pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    return CONFIG_MTK_PLATFORM;
}

