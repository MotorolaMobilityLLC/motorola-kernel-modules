/*
 * ESWIN EPH861X series Touchscreen driver
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

// uncomment to enable the dev_dbg prints to dmesg
#define DEBUG
// uncomment to test with input forced open
//#define INPUT_DEVICE_ALWAYS_OPEN
#include <linux/types.h>



#include <uapi/asm-generic/errno-base.h>
#include <linux/sysfs.h>

#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <uapi/linux/stat.h>
#include <linux/jiffies.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "eswin_eph861x_project_config.h"
#include "eswin_eph861x_tlv.h"
#include "eswin_eph861x_types.h"
#include "eswin_eph861x_eswin.h"
#include "eswin_eph861x_comms.h"
#include "eswin_eph861x.h"
#include "eswin_eph861x_tlv_command.h"

#define TRIGGER_FRAME_CNT 80
#define MAX_FRAME_LENGTH 2500

int eph_check_firmware_format(struct device *dev, const struct firmware *fw)
{
    unsigned int pos = 0;
    char c;

    while (pos < fw->size)
    {
        c = *(fw->data + pos);

        if (c < '0' || (c > '9' && c < 'A') || c > 'F')
        {
            return 0;
        }

        pos++;
    }

    dev_err(dev, "Aborting: firmware file must be in binary format\n");
    dev_err(dev, "xxd -r -p EPH861x_XXXXX_XXXX.enc > EPH861x_XXXXX_XXXX.bin\n");

    return -EINVAL;
}

int eph_update_file_name(struct device *dev,
                                char **out_file_name,
                                const char *in_file_name,
                                size_t in_str_len)
{
    char *file_name_tmp;

    /* Simple sanity check */
    if (in_str_len > 64)
    {
        dev_warn(dev, "File name too long\n");
        return -EINVAL;
    }

    file_name_tmp = (char *)krealloc(*out_file_name, in_str_len + 1, GFP_KERNEL);
    if (!file_name_tmp)
    {
        return -ENOMEM;
    }

    *out_file_name = file_name_tmp;
    memcpy(*out_file_name, in_file_name, in_str_len);

    /* Echo into the sysfs entry may append newline at the end of buf */
    if (in_file_name[in_str_len - 1] == '\n')
    {
        (*out_file_name)[in_str_len - 1] = '\0';
    }
    else
    {
        (*out_file_name)[in_str_len] = '\0';
    }

    return 0;
}

#ifdef CONFIG_OF // Open Firmware (Device Tree)
#define PRIM_PANEL_NAME    "mmi,panel_name"
const struct eph_platform_data *eph_platform_data_get_from_device_tree(struct comms_device *commsdevice)
{
    struct eph_platform_data *ephplatform;
    struct device_node *devnode = commsdevice->dev.of_node;
    int ret_val;
    struct device_node *chosen;
    const char *supplier;
    int num_of_panel_supplier;

    dev_dbg(&commsdevice->dev, "%s using device tree > \n", __func__);

    if (!devnode)
    {
        return (struct eph_platform_data *)ERR_PTR(-ENOENT); /* no device tree device */
    }

    ephplatform = (struct eph_platform_data *)devm_kzalloc(&commsdevice->dev, sizeof(struct eph_platform_data), GFP_KERNEL);
    if (!ephplatform)
    {
        return (struct eph_platform_data *)ERR_PTR(-ENOMEM);
    }

    ephplatform->gpio_reset = of_get_named_gpio_flags(devnode, "eswin,reset-gpio", 0, NULL);
    if (ephplatform->gpio_reset < 0)
        dev_err(&commsdevice->dev,  "Unable to get gpio_reset");

    ephplatform->gpio_chg_irq = of_get_named_gpio_flags(devnode, "eswin,irq-gpio", 0, NULL);
    if (ephplatform->gpio_chg_irq< 0)
        dev_err(&commsdevice->dev,  "Unable to get gpio_chg_irq");

    ephplatform->gpio_avdd = of_get_named_gpio_flags(devnode, "eswin,avdd-gpio", 0, NULL);
    if (ephplatform->gpio_avdd< 0)
        dev_err(&commsdevice->dev,  "Unable to get gpio_avdd");

    /* returns pointer to already allocated memory containing the string */
    ret_val = of_property_read_string(devnode, "eswin,regulator_dvdd", &ephplatform->regulator_dvdd);
    if (ret_val)
    {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,regulator_dvdd: %d\n", ret_val);
    }

    ret_val = of_property_read_string(devnode, "eswin,regulator_avdd", &ephplatform->regulator_avdd);
    if (ret_val)
    {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,regulator_avdd: %d\n", ret_val);
    }

    ret_val = of_property_read_string(devnode, "eswin,device_settings_name", &ephplatform->device_settings_name);
    if (ret_val)
    {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,device_settings_name: %d\n", ret_val);
    }

    ret_val = of_property_read_string(devnode, "eswin,fw_name", &ephplatform->fw_name);
    if (ret_val)
    {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,fw_name: %d\n", ret_val);
    }

    ephplatform->edge_ctrl = of_property_read_bool(devnode,
                    "eswin,edge-ctrl");
    if (ephplatform->edge_ctrl)
        ts_info("support eswin edge mode");

    ephplatform->interpolation_ctrl = of_property_read_bool(devnode,
                    "eswin,interpolation-ctrl");
    if (ephplatform->interpolation_ctrl)
        ts_info("support eswin interpolation mode");

    ephplatform->report_rate_ctrl = of_property_read_bool(devnode,
                    "eswin,report_rate-ctrl");
    if (ephplatform->report_rate_ctrl)
        ts_info("support eswin report rate switch mode");

    ephplatform->sample_ctrl = of_property_read_bool(devnode,
                    "eswin,sample-ctrl");
    if (ephplatform->sample_ctrl)
        ts_info("support eswin sample mode");

    ephplatform->stowed_mode_ctrl = of_property_read_bool(devnode,
                    "eswin,stowed-mode-ctrl");
    if (ephplatform->stowed_mode_ctrl)
        ts_info("Support eswin touch stowed mode");

    ephplatform->sensitivity_ctrl = of_property_read_bool(devnode,
                    "eswin,sensitivity-ctrl");
    if (ephplatform->sensitivity_ctrl)
        ts_info("Support eswin touch sensitivity control mode");

    ephplatform->stylus_mode_ctrl = of_property_read_bool(devnode,
                    "eswin,stylus-mode-ctrl");
    if (ephplatform->stylus_mode_ctrl)
        ts_info("Support eswin stylus mode");

    of_property_read_string(devnode, "eswin,input_name", &ephplatform->input_name);

    of_property_read_u32(devnode, "eswin,suspend-mode", &ephplatform->suspend_mode);

    of_property_read_u32(devnode, "eswin,panel-invert-x", &ephplatform->panel_invert_x);

    of_property_read_u32(devnode, "eswin,panel-invert-y", &ephplatform->panel_invert_y);

    ret_val = of_property_read_u32(devnode, "eswin,panel-max-x", &ephplatform->panel_max_x);
    if (ret_val) {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,panel_max_x: %d\n", ret_val);
        return (struct eph_platform_data *)ERR_PTR(-EINVAL);
    }
    ret_val = of_property_read_u32(devnode, "eswin,panel-max-y", &ephplatform->panel_max_y);
    if (ret_val) {
        dev_err(&commsdevice->dev,  "Couldn't read eswin,panel_max_y: %d\n", ret_val);
        return (struct eph_platform_data *)ERR_PTR(-EINVAL);
    }

    dev_info(&commsdevice->dev, "panel X%uY%u\n", ephplatform->panel_max_x,
            ephplatform->panel_max_y);

    chosen = of_find_node_by_name(NULL, "chosen");
    if (chosen) {
        ret_val = of_property_read_string(chosen, PRIM_PANEL_NAME,
                    (const char **)&supplier);
        if (ret_val) {
            dev_err(&commsdevice->dev, "%s cannot read %s %d\n", __func__, PRIM_PANEL_NAME, ret_val);
        }
        dev_info(&commsdevice->dev,"%s: %s %s",
                    __func__, PRIM_PANEL_NAME, supplier);
    }

    num_of_panel_supplier = of_property_count_strings(devnode, "eswin,panel-supplier");
    dev_info(&commsdevice->dev,"get eswin,panel-supplier count=%d", num_of_panel_supplier);
    if (num_of_panel_supplier > 1) {
        int j;
        for (j = 0; j < num_of_panel_supplier; j++) {
            ret_val = of_property_read_string_index(devnode, "eswin,panel-supplier", j, &ephplatform->panel_supplier);
            if (ret_val < 0) {
                dev_info(&commsdevice->dev,"cannot parse panel-supplier: %d\n", ret_val);
                break;
            } else if (ephplatform->panel_supplier && strstr(supplier, ephplatform->panel_supplier)) {
                dev_info(&commsdevice->dev,"matched panel_supplier: %s", ephplatform->panel_supplier);
                goto exit;
            }
        }
    } else {
        ret_val = of_property_read_string(devnode, "eswin,panel-supplier",
            &ephplatform->panel_supplier);
        if (ret_val < 0) {
            ephplatform->panel_supplier = NULL;
            dev_err(&commsdevice->dev,"Unable to read panel supplier\n");
        } else if (ephplatform->panel_supplier && strstr(supplier, ephplatform->panel_supplier)) {
            dev_info(&commsdevice->dev,"panel_supplier:%s matched!\n", ephplatform->panel_supplier);
            goto exit;
        } else {
            if (ephplatform->panel_supplier)
                dev_info(&commsdevice->dev,":%s not actived\n", ephplatform->panel_supplier);
            else
                dev_info(&commsdevice->dev,"panel_supplier NULL!\n");
        }
    }

exit:
    dev_dbg(&commsdevice->dev, "%s using device tree <\n", __func__);

    return ephplatform;
}
#else // CONFIG_OF
const struct eph_platform_data *eph_platform_data_get_from_device_tree(struct comms_device *commsdevice)
{
    return (struct eph_platform_data *)ERR_PTR(-ENOENT);
}
#endif // CONFIG_OF

struct eph_platform_data *eph_platform_data_get_default(struct comms_device *commsdevice)
{
    struct eph_platform_data *ephplatform = (struct eph_platform_data *)devm_kzalloc(&commsdevice->dev, sizeof(struct eph_platform_data), GFP_KERNEL);
    if (!ephplatform)
    {
        return (struct eph_platform_data *)ERR_PTR(-ENOMEM);
    }

    /* Set default parameters */

    dev_dbg(&commsdevice->dev, "%s <\n", __func__);

    return ephplatform;
}

const struct eph_platform_data * eph_platform_data_get(struct comms_device *commsdevice)
{
    const struct eph_platform_data *ephplatform;

    dev_dbg(&commsdevice->dev, "%s >\n", __func__);

    ephplatform = (struct eph_platform_data *)dev_get_platdata(&commsdevice->dev);
    if (ephplatform)
    {
        return ephplatform;
    }

    ephplatform = eph_platform_data_get_from_device_tree(commsdevice);
    if (!IS_ERR(ephplatform) || PTR_ERR(ephplatform) != -ENOENT)
    {
        return ephplatform;
    }

    ephplatform = eph_platform_data_get_default(commsdevice);
    if (!IS_ERR(ephplatform))
    {
        return ephplatform;
    }

    dev_err(&commsdevice->dev, "No platform data specified\n");
    return (struct eph_platform_data *)ERR_PTR(-EINVAL);
}

int eph_gpio_setup(struct eph_data *ephdata)
{
    int ret_val;
    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    ret_val = gpio_request(ephdata->ephplatform->gpio_chg_irq, "irq-gpio");
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_request %lu (%d)", ephdata->ephplatform->gpio_chg_irq, ret_val);
        return ret_val;
    }
    ret_val = gpio_direction_input(ephdata->ephplatform->gpio_chg_irq);
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_direction_input %lu (%d)", ephdata->ephplatform->gpio_chg_irq, ret_val);
        return ret_val;
    }
    dev_dbg(&ephdata->commsdevice->dev, "gpio_chg_irq %lu IN %d\n", ephdata->ephplatform->gpio_chg_irq, (u8)gpio_get_value(ephdata->ephplatform->gpio_chg_irq));

    ret_val = gpio_request(ephdata->ephplatform->gpio_reset, "reset-gpio");
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_request %lu (%d)", ephdata->ephplatform->gpio_reset, ret_val);
        return ret_val;
    }
    /* Initialise so that we are holding the device in reset until power has been applied */
    ret_val = gpio_direction_output(ephdata->ephplatform->gpio_reset, GPIO_RESET_YES_LOW);
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_direction_output %lu (%d)", ephdata->ephplatform->gpio_reset, ret_val);
        return ret_val;
    }

    dev_dbg(&ephdata->commsdevice->dev, "gpio_reset %lu OUT %d\n", ephdata->ephplatform->gpio_reset, GPIO_RESET_YES_LOW);

    ret_val = gpio_request(ephdata->ephplatform->gpio_avdd, "avdd-gpio");
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_request %lu (%d)", ephdata->ephplatform->gpio_avdd, ret_val);
        return ret_val;
    }
    /* Initialise so that we are holding the device in reset until power has been applied */
    ret_val = gpio_direction_output(ephdata->ephplatform->gpio_avdd, 0);
    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "gpio_direction_output %lu (%d)", ephdata->ephplatform->gpio_avdd, ret_val);
        return ret_val;
    }

    dev_dbg(&ephdata->commsdevice->dev, "gpio_avdd %lu OUT %s\n", ephdata->ephplatform->gpio_avdd, "0");

    dev_dbg(&ephdata->commsdevice->dev, "%s <\n", __func__);

    return 0;
}

int eph_wait_for_completion(struct eph_data *ephdata,
                                   struct completion *comp,
                                   unsigned int timeout_ms,
                                   const char *dbg_str)
{
    struct device *dev = &ephdata->commsdevice->dev;
    unsigned long timeout = msecs_to_jiffies(timeout_ms);
    long ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s %s >\n", __func__, dbg_str);

    ret_val = wait_for_completion_interruptible_timeout(comp, timeout);
    if (ret_val < 0)
    {
        return ret_val;
    }
    if (ret_val == 0)
    {
        dev_err(dev, "wait_for_completion timeout %s\n", dbg_str);
        return -ETIMEDOUT;
    }
    return 0;
}

void eph_regulator_enable(struct eph_data *ephdata)
{
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (!ephdata->reg_vdd)
    {
        dev_dbg(&ephdata->commsdevice->dev, " No valid reg_vdd");
        return;
    }

    gpio_set_value(ephdata->ephplatform->gpio_reset, GPIO_RESET_YES_LOW);

    ret_val = regulator_enable(ephdata->reg_vdd);
    if (ret_val)
    {
        dev_dbg(&ephdata->commsdevice->dev, " regulator reg_vdd failed");
        return;
    }

    if (ephdata->ephplatform->gpio_avdd > 0) {
        dev_dbg(&ephdata->commsdevice->dev, "valid gpio_avdd then set gpio 1");
        gpio_direction_output(ephdata->ephplatform->gpio_avdd, 1);
    } else {
        dev_dbg(&ephdata->commsdevice->dev, "invalid gpio_avdd then enable reg_avdd");
        ret_val = regulator_enable(ephdata->reg_avdd);
    }

    /* According to power sequencing specification, RESET line must be kept
     * low until some time after regulators come up to voltage */
    msleep(EPH_REGULATOR_DELAY);
    gpio_set_value(ephdata->ephplatform->gpio_reset, GPIO_RESET_NO_HIGH);
    //TODO this additional delay should not be needed
    /* Delay to prevent poor signals after power up. This will allow device time to "settle" before baseline */
    msleep(EPH_POWERON_DELAY);

retry_wait:
    reinit_completion(&ephdata->chg_completion);
    ephdata->in_bootloader = true;
    ret_val = eph_wait_for_completion(ephdata, &ephdata->chg_completion, EPH_POWERON_DELAY, "CHG");
    if (ret_val == -EINTR)
    {
        dev_dbg(&ephdata->commsdevice->dev, "wait CHG failed");
        goto retry_wait;
    }
    ephdata->in_bootloader = false;
    dev_dbg(&ephdata->commsdevice->dev, "%s <\n", __func__);
 }

void eph_regulator_disable(struct eph_data *ephdata)
{
    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (ephdata->reg_vdd)
    {
       dev_dbg(&ephdata->commsdevice->dev, "disable reg_vdd");
       regulator_disable(ephdata->reg_vdd);
    }

    if (ephdata->ephplatform->gpio_avdd > 0) {
        dev_dbg(&ephdata->commsdevice->dev, "set gpio_avdd 0");
        gpio_direction_output(ephdata->ephplatform->gpio_avdd, 0);
    } else {
        dev_dbg(&ephdata->commsdevice->dev, "disable reg_avdd");
        regulator_disable(ephdata->reg_avdd);
    }
    dev_dbg(&ephdata->commsdevice->dev, "%s <\n", __func__);
}

void eph_reset_device(struct eph_data *ephdata)
{

    dev_dbg(&ephdata->commsdevice->dev, "%s gpio is %ld >\n", __func__, ephdata->ephplatform->gpio_reset);
    gpio_set_value(ephdata->ephplatform->gpio_reset, GPIO_RESET_YES_LOW);
    msleep(1);
    gpio_set_value(ephdata->ephplatform->gpio_reset, GPIO_RESET_NO_HIGH);
    msleep(EPH_POWERON_DELAY);

    return;

}

/**
 * eswin_power_on - turn on power to the touch device
 * ephdata - pointer to eswin touch data
 * return: 0 ok, <0 failed
*/
int eph_power_on(struct eph_data *ephdata)
{
    int ret = 0;
    dev_info(&ephdata->commsdevice->dev, "device power on");
    if (ephdata->power_on)
    {
        dev_dbg(&ephdata->commsdevice->dev, "already power on");
    }
    else
    {
        dev_dbg(&ephdata->commsdevice->dev, "power on execute");
        eph_regulator_enable(ephdata);
        ephdata->power_on = 1;
        atomic_set(&ephdata->heartbeat_on, 1);
    }

    return ret;
}

/**
 * eswin_power_off - turn off power to the touch device
 * ephdata - pointer to eswin touch data
 * return: 0 ok, <0 failed
*/
int eph_power_off(struct eph_data *ephdata)
{
    int ret = 0;
    dev_info(&ephdata->commsdevice->dev, "device power off");
    if (!ephdata->power_on)
    {
        dev_dbg(&ephdata->commsdevice->dev, "already power off");
    }
    else
    {
        dev_dbg(&ephdata->commsdevice->dev, "power off execute");
        eph_regulator_disable(ephdata);
        ephdata->power_on = 0;
        atomic_set(&ephdata->heartbeat_on, 0);
    }
    return ret;
}

#define POWER_DELAY_US 1000*500
void eph_recovery_device(struct eph_data *ephdata)
{
    dev_dbg(&ephdata->commsdevice->dev, "%s gpio is %ld >\n", __func__, ephdata->ephplatform->gpio_reset);
    eph_power_off(ephdata);
    usleep_range(POWER_DELAY_US, (POWER_DELAY_US + 200));
    eph_power_on(ephdata);

    return;
}

bool eph_is_fod_resume(struct eph_data *ephdata)
{
    unsigned long fod_timeout = msecs_to_jiffies(3000);

    fod_timeout += ephdata->fod_jiffies;
    if (time_before(jiffies, fod_timeout)) {
        return true;
    }

    return false;
}

int eph_irq_enable(struct eph_data *ephdata, bool enable)
{
    if (enable && !atomic_cmpxchg(&ephdata->irq_enabled, 0, 1)) {
        enable_irq(ephdata->chg_irq);
        ts_info("Irq enabled");
        return 0;
    }

    if (!enable && atomic_cmpxchg(&ephdata->irq_enabled, 1, 0)) {
        disable_irq(ephdata->chg_irq);
        ts_info("Irq disabled");
        return 0;
    }
    ts_info("warnning: irq deepth inbalance!");
    return 0;
}

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
static struct frame_log_t {
    u8 *buf;
    int used;
} frame_log;

static int discard_frames;
static int frame_cnt;
static int freq_index;
static int process;
static size_t total_cnt;
#define CMD_LEN          11
#define CMD_VAL_IDX      7
static int eph_output_debug_data(struct eph_data *ephdata, bool enable)
{
    int ret_val;
    u8 cfg_signal[CMD_LEN] = {TLV_CONTROL_DATA_WRITE, 0x08, 0x00, 0x8c, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00};
    u8 cfg_delta[CMD_LEN] = {TLV_CONTROL_DATA_WRITE, 0x08, 0x00, 0x0e, 0x01, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00};
    u8 cfg_sys[8] = {TLV_CONTROL_DATA_WRITE, 0x05, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00};
    u16 length = CMD_LEN;
    if(enable)
    {
        cfg_signal[CMD_VAL_IDX] = 0x03; //mct + sct signal
        cfg_delta[CMD_VAL_IDX] = 0x03;  //mct + sct delta
        cfg_sys[CMD_VAL_IDX] = 0x01;  //system active
    }
    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, &cfg_signal[0]);
    if(ret_val)
    {
        ts_err("cfg_signal %s failed", enable ? "enable" : "disable");
    }
    ret_val = eph_write_control_config(ephdata, length, &cfg_delta[0]);
    if(ret_val)
    {
        ts_err("cfg_delta %s failed", enable ? "enable" : "disable");
    }
    ret_val = eph_write_control_config(ephdata, 8, &cfg_sys[0]);
    if(ret_val)
    {
        ts_err("cfg_sys %s failed", enable ? "enable" : "disable");
    }
    mutex_unlock(&ephdata->comms_mutex);

    return ret_val;
}

void eph_cache_debug_log(struct eph_data *ephdata)
{
    u8 *frame_ptr = ephdata->trigger_buf;
    int frame_len = le16_to_cpup((__le16 *)(frame_ptr + 1));
    ts_info("[log %d] - %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", frame_cnt, frame_ptr[0], frame_ptr[1], frame_ptr[2], frame_ptr[3], frame_ptr[4], frame_ptr[5], frame_ptr[6], frame_ptr[7], frame_ptr[8]);

    if (discard_frames > 0) {
        discard_frames--;
        return;
    }
    if (0x81 != frame_ptr[0]) {
        ts_info("[log %d] - type %02x, discard!\n", frame_cnt, frame_ptr[0]);
        return;
    }
    frame_len = frame_len + 3;
    //judge if frame_len is within limit
    if (frame_len > MAX_FRAME_LENGTH) {
        ts_info("frame_len is too long, %d", frame_len);
        frame_len = MAX_FRAME_LENGTH;
        ephdata->data_valid = 0;
    }

    memcpy(frame_log.buf + frame_log.used, frame_ptr, frame_len);
    frame_log.used += frame_len;

    total_cnt += frame_log.used;
    ts_info("frame log used:%d, total cnt:%zu", frame_log.used, total_cnt);
    put_fifo_with_discard(frame_log.buf, frame_log.used);
    memset(frame_log.buf, 0x0, sizeof(frame_log.used));
    frame_log.used = 0;

    frame_cnt++;
    if (frame_cnt >= TRIGGER_FRAME_CNT) {
        frame_cnt = 0;
        process = 0;
        freq_index = 0;
        eph_reset_device(ephdata);
        atomic_set(&ephdata->trigger_enable, 0);
        total_cnt = 0;
        vfree(frame_log.buf);
        ts_info("Notify raw data capture down, data_valid:%d", ephdata->data_valid);
        if (ephdata->data_valid == 1)
            sysfs_notify(ephdata->imports->kobj_notify, NULL, "log_trigger");
    }
}

int frame_log_capture_stop(struct eph_data *ephdata)
{
    eph_output_debug_data(ephdata, false);
    vfree(frame_log.buf);
    ts_info("stop ghost log capture, trigger_enable state change to 0\n");
    return 0;
}

int frame_log_capture_start(struct eph_data *ephdata)
{

    if (atomic_read(&ephdata->allow_capture) == 0) {
        ts_info("Touch not active, not allow ghost log capture\n");
        return 0;
    }

    if (atomic_read(&ephdata->trigger_enable) != 0)
        return 0;

    frame_log.buf = vmalloc(4 * 1024);
    if (!frame_log.buf)
        return -ENOMEM;

    //init parameters
    frame_log.used = 0;
    ephdata->data_valid = 1;
    discard_frames = 4;
    frame_cnt = 0;
    freq_index = 0;
    process = 0;
    total_cnt = 0;

    eph_output_debug_data(ephdata, true);
    atomic_set(&ephdata->trigger_enable, 1);
    ts_info("start ghost log capture\n");
    return 0;
}
#endif
