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

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <uapi/linux/input-event-codes.h>
#include <uapi/linux/stat.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hardirq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include "eswin_eph861x_project_config.h"
#include "eswin_eph861x_types.h"
#include "eswin_eph861x_comms.h"
#include "eswin_eph861x_eswin.h"
#include "eswin_eph861x_bootloader.h"
#include "eswin_eph861x_tlv_command.h"
#include "eswin_eph861x_tlv_report.h"
#include "eswin_eph861x.h"
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
#include "eswin_ts_mmi.h"
#endif
#include <linux/mmi_device.h>

/* device settings file format version expected*/
#define EPH_DEVICE_SETTINGS_FORMAT       "<product>EPH8610</product>"

/* Touchscreen absolute values */
#define EPH_MAX_HEIGHT_WIDTH                255u
#define EPH_HOST_REPORTING_TOUCH            1u
#define EPH_HOST_REPORTING_GESTURE          2u
#define EPH_ESD_RECOVERY

static int eph_sysfs_mem_access_init(struct eph_data *ephdata);
static void eph_sysfs_mem_access_remove(struct eph_data *ephdata);
static int eph_configure_components(struct eph_data *ephdata, const struct firmware *device_settings);

#if defined EPH_ESD_RECOVERY
void heartbeat_work_start(struct eph_data *ephdata);
void heartbeat_work_stop(struct eph_data *ephdata);
#endif

static int eph_check_mem_access_params(struct eph_data *ephdata,
                                       loff_t off,
                                       size_t *count)
{
    if (off >= PAGE_SIZE)
    {
        return -EIO;
    }

    if (off + *count > PAGE_SIZE)
    {
        return -EIO;
    }

    return 0;
}

static int eph_probe_bootloader(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val;

    dev_dbg(dev, "%s >\n", __func__);

    ret_val = eph_comms_specific_bootloader_checks(ephdata);
    if (ret_val)
    {
        return ret_val;
    }

    /* Check bootloader status and version information */
    ret_val = eph_read_bootloader_information(ephdata);

    dev_info(dev, "%s, product_id = %x, variant_id = %x, bootloader_version = %x, error = %d\n",
            __func__,
            ephdata->ephdeviceinfo.product_id,
            ephdata->ephdeviceinfo.variant_id,
            ephdata->ephdeviceinfo.bootloader_version,
            ret_val);

    if (ret_val)
    {
        /* Force device into bootloader mode using chg line held low while toggle reset */
        ret_val = eph_chg_force_bootloader(ephdata);

        /* now forced into bootloader check whether we get a valid read */
        ret_val = eph_read_bootloader_information(ephdata);

        dev_info(dev, "%s, product_id = %x, variant_id = %x, bootloader_version = %x, error = %d\n",
                __func__,
                ephdata->ephdeviceinfo.product_id,
                ephdata->ephdeviceinfo.variant_id,
                ephdata->ephdeviceinfo.bootloader_version,
                ret_val);

        /* Release CHG line as no more bootloader reads required - Can return to app (reset to app not strictly required */
        (void)eph_bootloader_release_chg(ephdata);

        if (ret_val)
        {
            return ret_val;
        }
    }

    return 0;
}

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
static int eph_read_engineering_messages(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val;

    mutex_lock(&ephdata->frame_log_lock);
    /* Read report */
    ret_val = eph_read_report(ephdata, ephdata->trigger_buf);
    eph_cache_debug_log(ephdata);
    mutex_unlock(&ephdata->frame_log_lock);

    if (0 > ret_val)
    {
        dev_err(dev, "Failed to read engineering message (%d)\n", ret_val);
        return ret_val;
    }

    /* return 0 is success */
    return ret_val;
}
#endif

static int eph_read_and_process_messages(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val;

    mutex_lock(&ephdata->comms_mutex);
    /* Read report */
    ret_val = eph_read_report(ephdata, ephdata->report_buf);
    mutex_unlock(&ephdata->comms_mutex);

    if (0 > ret_val)
    {
        dev_err(dev, "Failed to read message (%d)\n", ret_val);
        return ret_val;
    }

    ret_val = eph_handle_report(ephdata, ephdata->report_buf);

    memset(ephdata->report_buf, 0, COMMS_BUF_SIZE);
    /* return 0 is success */
    return ret_val;
}

static irqreturn_t eph_interrupt(int irq, void *dev_id)
{
    struct eph_data *ephdata = (struct eph_data *)dev_id;
    struct device *dev = &ephdata->commsdevice->dev;
#if !defined(ESWIN_SYSTEM_SUSPEND)
    int ret = 0;
    if ((ephdata->suspended) && (ephdata->pm_suspend)) {
        ret = wait_for_completion_timeout(
                  &ephdata->pm_completion,
                  msecs_to_jiffies(EPH_TIMEOUT_COMERR_PM));
        if (!ret) {
            ts_err("Bus don't resume from pm(deep),timeout,skip irq");
            return IRQ_HANDLED;
        }
    }
#endif

    if (dev == NULL)
        return IRQ_HANDLED;

    pm_stay_awake(dev);
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
	if (atomic_read(&ephdata->trigger_enable) == 1) {
        eph_read_engineering_messages(ephdata);
	} else {
	    eph_read_and_process_messages(ephdata);
	}
#else
    eph_read_and_process_messages(ephdata);
#endif
    /* will not unblock any other threads until message has been read */
    complete(&ephdata->chg_completion);

    pm_relax(dev);

    return IRQ_HANDLED;
}

static int eph_trigger_baseline(struct eph_data *ephdata)
{
    int ret_val;
    u8 cfg[8] =    {TLV_CONTROL_DATA_WRITE, 0x05, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01};
    u16 length = (sizeof(cfg)/sizeof(cfg[0]));
    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, &cfg[0]);
    mutex_unlock(&ephdata->comms_mutex);

    return ret_val;

}

static void eph_trigger_baseline_work(struct work_struct *work)
{
    int ret_val = 0;
    struct eph_data *ephdata = container_of(work, struct eph_data, force_baseline_work);
#if 0
    struct backlight_device *bd = ephdata->bl;
    int brightness = 0;

    printk("%s\n", __func__);

    if (ephdata->last_brightness == 0) {
        do {
            if (bd->ops && bd->ops->get_brightness)
                brightness = bd->ops->get_brightness(bd);
            else
                brightness = bd->props.brightness;

            if (brightness)
                break;

            dev_info(&ephdata->commsdevice->dev, "eph wait 5 msec\n");
            msleep(5);

        } while (brightness == 0);

        ret_val = eph_trigger_baseline(ephdata);

        if (ret_val)
            dev_err(&ephdata->commsdevice->dev, "eph set baseline fail %d\n", ret_val);

        ephdata->last_brightness = brightness;

        dev_info(&ephdata->commsdevice->dev, "brightness %d\n", brightness);
    } else {
        dev_info(&ephdata->commsdevice->dev, "no need force baseline\n");
    }
#else
    ret_val = eph_trigger_baseline(ephdata);
    if (ret_val)
        dev_err(&ephdata->commsdevice->dev, "eph set baseline fail %d\n", ret_val);
#endif
}

static int eph_finger_print_enable(struct eph_data *ephdata, bool enable)
{
    int ret_val;
    u8 cfg[8] = {TLV_CONTROL_DATA_WRITE, 0x05, 0x00, 0x0B, 0x00, 0x03, 0x00, 0x00};
    u16 length = (sizeof(cfg)/sizeof(cfg[0]));

    if (enable)
    {
        cfg[7] = 0x08;
    }

    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, &cfg[0]);
    mutex_unlock(&ephdata->comms_mutex);

    return ret_val;
}

static int eph_trigger_backup_to_nvm(struct eph_data *ephdata)
{
    int ret_val;
    u8 cfg[8] =    {TLV_CONTROL_DATA_WRITE, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    u16 length = (sizeof(cfg)/sizeof(cfg[0]));
    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, &cfg[0]);
    mutex_unlock(&ephdata->comms_mutex);

    return ret_val;

}

/* eph_update_device_settings - download device settings to chip
 * The file consists of repeating patterns of the following:
 *   <TYPE> - 1-byte type
 *   <LENGTH> - 2-byte length
 *   <DATA> - length-bytes of data
 */
static int eph_update_device_settings(struct eph_data *ephdata, const struct firmware *device_settings_image)
{
    struct device *dev = &ephdata->commsdevice->dev;
    struct eph_device_settings device_settings;
    int ret_val = 0;
    u8* cfg_ptr;
    u16 component;
    u16 offset;
    u16 msg_count = 0u;
    u8 retry = 0u;
    u16 pos = 0u;
    struct tlv_header tlvheader;

    dev_dbg(dev, "%s >\n", __func__);

    /* Allocate space for zero terminated copy of the device settings file */
    device_settings.raw = (u8 *)kzalloc(device_settings_image->size + 1, GFP_KERNEL);
    if (!device_settings.raw)
    {
        dev_err(dev, "ESWIN Couldnt allocate memory for device settings file %d.\n", -ENOMEM);
        return -ENOMEM;
    }

    /* Copy from the firmware image into local buffer */
    memcpy(device_settings.raw, device_settings_image->data, device_settings_image->size);
    /* Pad last entry as a zero. We are about to loop through all config and control tlvs */
    /* The 0 will not match an expected T and the configuration loading will terminate */
    device_settings.raw[device_settings_image->size] = 0;
    device_settings.raw_size = device_settings_image->size;
    cfg_ptr = &device_settings.raw[0];

    //TODO add format checking - Currently no version information in settings file.
    while (pos < device_settings_image->size)
    {
        tlvheader = eph_get_tl_header_info(ephdata, cfg_ptr);
        /* if item next in the data stream is not configuration or control break out */
        if((TLV_CONFIG_DATA_WRITE == tlvheader.type) || (TLV_CONTROL_DATA_WRITE == tlvheader.type))
        {
            if (pos > device_settings_image->size)
            {
                /* Have reached the end of the file but did not find the terminating character */
                ret_val = -ENOMEM;
                dev_err(dev, "ESWIN went out of bounds of device settings file %d.\n", ret_val);
                /* Ensure memory released before returning */
                goto fail_release;
            }
            component = *(cfg_ptr + TLV_HEADER_SIZE + TLV_WRITE_COMPONENT_FIELD) | (*(cfg_ptr + TLV_HEADER_SIZE + TLV_WRITE_COMPONENT_FIELD + 1) << 8);
            offset = *(cfg_ptr + TLV_HEADER_SIZE + TLV_WRITE_OFFSET_FIELD) | (*(cfg_ptr + TLV_HEADER_SIZE + TLV_WRITE_OFFSET_FIELD + 1) << 8);
            dev_dbg(dev, "TYPE: %d LEGNTH: %d COMPONENT_ID: %d OFFSET: %d\n", tlvheader.type, tlvheader.length, component, offset);
            msg_count++;
            dev_dbg(dev, "Configuration/control write: BLOCK NUMBER: %d.\n", msg_count);

            mutex_lock(&ephdata->comms_mutex);
            ret_val = eph_write_control_config(ephdata, tlvheader.length, cfg_ptr);
            mutex_unlock(&ephdata->comms_mutex);

            retry = 0u;
            while (ret_val)
            {
                dev_info(&ephdata->commsdevice->dev,
                         "Retry control config write, It failed for some reason. msg_count: %u",
                         msg_count);

                mutex_lock(&ephdata->comms_mutex);
                ret_val = eph_write_control_config(ephdata, tlvheader.length, cfg_ptr);
                mutex_unlock(&ephdata->comms_mutex);

                retry++;
                if (10 < retry)
                {
                    ret_val = -EIO;
                    dev_err(dev, "ESWIN failed to send settings to TIC %d.\n", ret_val);
                    /* Ensure memory released before returning */
                    goto fail_release;

                }
            }

            cfg_ptr = cfg_ptr + tlvheader.length;
            pos = pos + tlvheader.length;

            if(ret_val)
            {
                dev_info(dev, "Configuration/Contol write failure on: BLOCK NUMBER: %d.\n", msg_count);
                /* Ensure memory released before returning */
                goto fail_release;
            }

        }
        else
        {
            break;
        }
    }

    ret_val = eph_trigger_backup_to_nvm(ephdata);
    if(ret_val)
    {
        dev_err(dev, "Failed to trigger_backup_to_nvm %d.\n", ret_val);
    }
    ret_val = eph_trigger_baseline(ephdata);
    if(ret_val)
    {
        dev_err(dev, "Failed to trigger_baseline %d.\n", ret_val);
    }

    dev_info(dev, "Config successfully updated\n");

fail_release:
    kfree(device_settings.raw);

    return ret_val;
}

static int eph_acquire_irq(struct eph_data *ephdata)
{
    int ret_val;
    char *commsdevice_name;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (!ephdata->chg_irq)
    {
        ephdata->chg_irq = gpio_to_irq(ephdata->ephplatform->gpio_chg_irq);

        commsdevice_name = eph_comms_devicename_get(ephdata);

        /* configure a thread that is triggered on the interrupt */
        /* This specific IRQ remains disabled while handler/thread is active */
        /* Thread happens outside of interrupt handler and so frees up the general interupt hardware */
        ret_val = request_threaded_irq(ephdata->chg_irq,
                                       NULL,
                                       eph_interrupt,
                                       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                       commsdevice_name,
                                       ephdata);
        if (ret_val)
        {
            dev_err(&ephdata->commsdevice->dev, "request_threaded_irq (%d)\n", ret_val);
            return ret_val;
        }

        /* Presence of ephdata->chg_irq means IRQ initialised */
        dev_info(&ephdata->commsdevice->dev, "gpio_to_irq %lu -> %d\n", ephdata->ephplatform->gpio_chg_irq, ephdata->chg_irq);
    }
    else
    {
        eph_irq_enable(ephdata, true);
    }


    dev_info(&ephdata->commsdevice->dev, "%s <\n", __func__);

    return 0;
}

static int eph_input_open(struct input_dev *inputdev);
static void eph_input_close(struct input_dev *inputdev);

static void eph_unregister_input_device(struct eph_data *ephdata)
{
    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (ephdata->inputdev)
    {
#ifdef INPUT_DEVICE_ALWAYS_OPEN
        eph_input_close(ephdata->inputdev);
#endif //INPUT_DEVICE_ALWAYS_OPEN

        input_unregister_device(ephdata->inputdev);
    }
}

#define ESW_I2C_VTG_MIN_UV                  1800000
#define ESW_I2C_VTG_MAX_UV                  1800000

static int eph_probe_regulators(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val;

    dev_dbg(dev, "%s >\n", __func__);

    /* Must have reset GPIO to use regulator support */
    if (!gpio_is_valid(ephdata->ephplatform->gpio_reset))
    {
        ret_val = -EINVAL;
        goto fail;
    }
    if (!gpio_is_valid(ephdata->ephplatform->gpio_chg_irq))
    {
        ret_val = -EINVAL;
        goto fail;
    }

    ephdata->reg_vdd = regulator_get(dev, ephdata->ephplatform->regulator_dvdd);
    if (!IS_ERR_OR_NULL(ephdata->reg_vdd)) {
        if (regulator_count_voltages(ephdata->reg_vdd) > 0) {
            ret_val = regulator_set_voltage(ephdata->reg_vdd,
                                        ESW_I2C_VTG_MIN_UV,
                                        ESW_I2C_VTG_MAX_UV);
            if (ret_val)
            {
                ret_val = PTR_ERR(ephdata->reg_vdd);
                dev_err(dev, "Error %d getting vdd regulator\n", ret_val);
                goto fail;
            }
        }
    }

    ephdata->reg_avdd = regulator_get(dev, ephdata->ephplatform->regulator_avdd);

    if (IS_ERR(ephdata->reg_avdd))
    {
        ret_val = PTR_ERR(ephdata->reg_avdd);
        dev_err(dev, "Error %d getting avdd regulator\n", ret_val);
    }
    ephdata->power_on = 0;
    eph_power_on(ephdata);
    dev_dbg(dev, "%s <\n", __func__);
    return 0;

fail:
    ephdata->reg_vdd = NULL;
    ephdata->reg_avdd = NULL;
    gpio_free(ephdata->ephplatform->gpio_reset);
    gpio_free(ephdata->ephplatform->gpio_chg_irq);
    return ret_val;
}


static int eph_input_device_initialize(struct eph_data *ephdata)
{
    const struct eph_platform_data *ephplatform = ephdata->ephplatform;
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val;
    unsigned int mt_flags = 0;
    u16 max_resoultion;

    dev_dbg(dev, "%s >\n", __func__);

    dev_info(dev, "Touchscreen resolution X%uY%u\n", ephplatform->panel_max_x, ephplatform->panel_max_y);

    ephdata->inputdev = devm_input_allocate_device(dev);
    if (!ephdata->inputdev)
    {
        dev_err(dev, "allocate device failed %d\n", -ENOMEM);
        return -ENOMEM;
    }
#if 0
    if (ephplatform->input_name)
    {
        ephdata->inputdev->name = ephplatform->input_name;
    }
    else
    {
        ephdata->inputdev->name = "ESWIN EPH861X Touchscreen";
    }
#endif
    ephdata->inputdev->name = "eswin_ts";
    ephdata->inputdev->phys = ephdata->phys;
    ephdata->inputdev->id.bustype = EPH_COMMS_BUS_TYPE;
    ephdata->inputdev->id.product = 0xDEAD;
    ephdata->inputdev->id.vendor = 0xBEEF;
    ephdata->inputdev->id.version = 10427;
    ephdata->inputdev->dev.parent = dev;

#ifndef INPUT_DEVICE_ALWAYS_OPEN
    ephdata->inputdev->open = eph_input_open;
    ephdata->inputdev->close = eph_input_close;
#endif //INPUT_DEVICE_ALWAYS_OPEN


    /* this will confuse android into thinking we support a feature we dont - IDEALLY should no be set */
    /* Cannot remove EV_KEY from get event so appears to be required for pixel at least */
    input_set_capability(ephdata->inputdev, EV_KEY/*event type*/, BTN_TOUCH/*event code*/);


    input_set_capability(ephdata->inputdev, EV_ABS, ABS_MT_POSITION_X);
    input_set_capability(ephdata->inputdev, EV_ABS, ABS_MT_POSITION_Y);

    input_set_capability(ephdata->inputdev, EV_KEY, BTN_TRIGGER_HAPPY1);
    input_set_capability(ephdata->inputdev, EV_KEY, BTN_TRIGGER_HAPPY2);

    /* direct device, e.g. touchscreen */
    mt_flags |= INPUT_MT_DIRECT;

    /* multi touch */
    ret_val = input_mt_init_slots(ephdata->inputdev, CONFIG_SUPPORTED_TOUCHES, mt_flags);
    if (ret_val)
    {
        dev_err(dev, "Error %d initialising slots\n", ret_val);
        return ret_val;
    }

    /* reports co-ordinates of the tool */
    /* Height appears to always be Y */
    input_set_abs_params(ephdata->inputdev, ABS_MT_POSITION_X, 0, ephplatform->panel_max_x, 0, 0);
    input_set_abs_params(ephdata->inputdev, ABS_MT_POSITION_Y, 0, ephplatform->panel_max_y, 0, 0);

    max_resoultion = (ephplatform->panel_max_y > ephplatform->panel_max_x) ? ephplatform->panel_max_y :
        ephplatform->panel_max_x;

    /* The minor and major has no particular orientation just the longest/shortest axis */
    input_set_abs_params(ephdata->inputdev, ABS_MT_TOUCH_MAJOR, 0, (EPH_MAX_HEIGHT_WIDTH*max_resoultion), 0, 0);
    input_set_abs_params(ephdata->inputdev, ABS_MT_TOUCH_MINOR, 0, (EPH_MAX_HEIGHT_WIDTH*max_resoultion), 0, 0);
    input_set_abs_params(ephdata->inputdev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#ifdef CONFIG_ENABLE_ESWIN_PALM_CANCEL
    input_set_abs_params(ephdata->inputdev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER, MT_TOOL_PALM, 0, 0);
#endif
    input_set_drvdata(ephdata->inputdev, ephdata);

    dev_dbg(dev, "input_register_device\n");

    input_set_capability(ephdata->inputdev, EV_KEY, KEY_WAKEUP);
    ret_val = eph_gesture_init(ephdata);
    if (ret_val) {
        dev_err(dev, "Error %d initial gesture capability\n", ret_val);
    }

    ret_val = input_register_device(ephdata->inputdev);
    if (ret_val)
    {
        dev_err(dev, "Error %d registering input device\n", ret_val);
        return ret_val;
    }

#ifdef INPUT_DEVICE_ALWAYS_OPEN
    eph_input_open(ephdata->inputdev);
#endif //INPUT_DEVICE_ALWAYS_OPEN

    dev_info(dev, "%s <\n", __func__);

    return 0;
}

static int eph_enter_bootloader(struct eph_data *ephdata);

static int burn_2_ic(struct eph_data *ephdata, const struct firmware *ic_fw)
{
    int ret_val = 0;
    struct eph_flash flash;
    struct device *dev = (struct device *)&ephdata->commsdevice->dev;

    ephdata->updating_firmware = true;
    mutex_lock(&ephdata->fw_upgrade_mutex);
    ret_val = eph_enter_bootloader(ephdata);
    if (ret_val)
    {
        dev_err(dev, "enter bl fail %d\n", ret_val);
        goto err;
    }

    flash.ephdata = ephdata;
    flash.fw = ic_fw;
    ret_val = eph_send_frames(ephdata, &flash);
    if (ret_val)
    {
        dev_err(dev, "send frames fail %d\n", ret_val);
        goto err;
    }

err:
    flash.ephdata = NULL;
    flash.fw = NULL;
    mutex_unlock(&ephdata->fw_upgrade_mutex);
    ephdata->updating_firmware = false;
    return ret_val;
}

static int eph_initialize(struct eph_data *ephdata);

void eph_request_fw_cb(const struct firmware *ic_firmware, void *ctx)
{
    int ret_val = 0;
    struct eph_data *ephdata = (struct eph_data *)ctx;
    struct device *dev = &ephdata->commsdevice->dev;
    struct firmware *fw_data;

    dev_info(dev, "%s\n", __func__);

    fw_data = (struct firmware*)devm_kzalloc(dev, sizeof(struct firmware), GFP_KERNEL);
    if (!fw_data)
    {
        dev_err(dev, "couldnt allocate memory for firmware %d\n", ret_val);
        goto err;
    }

    if (NULL == ic_firmware)
    {
        dev_err(dev, "%s eph_flash = NULL - no fw image found\n", __func__);
        goto err;
    }

    memcpy(fw_data, ic_firmware, sizeof(struct firmware));

    ret_val = eph_check_firmware_format(dev, fw_data);
    if (ret_val)
    {
        dev_err(dev, "check fw fail %d\n", ret_val);
        goto err;
    }

    eph_get_bin_firmware_version(ephdata, fw_data);

    ret_val = eph_firmware_version_compare(ephdata);
    if (!ret_val)
    {
        dev_err(dev, "firmware no need upgrade %d\n", ret_val);
        goto err;
    }
    else
    {
        dev_info(dev, "firmware need upgrade %d\n", ret_val);
    }

    ret_val = burn_2_ic(ephdata, fw_data);
    if (ret_val) {
        dev_err(dev, "burn to ic fail %d\n", ret_val);
        goto err;
    }

    /* must wait 200ms for touch ic stable */
    msleep(200);

    ephdata->suspended = false;

    ret_val = eph_initialize(ephdata);
    if (ret_val)
    {
        dev_err(dev, "eph init fail %d\n", ret_val);
        goto err;
    }

    ret_val = eph_firmware_version_compare(ephdata);
    if (!ret_val)
    {
        dev_info(dev, "firmware upgraded successfully%d\n", ret_val);
    }
    else
    {
        dev_info(dev, "firmware upgraded failed%d\n", ret_val);
    }

    // (void)eph_input_device_initialize(ephdata);

err:
    release_firmware(ic_firmware);
    devm_kfree(dev, fw_data);
    return;
}

static int eph_initialize(struct eph_data *ephdata)
{
    struct comms_device *commsdevice = ephdata->commsdevice;
    int comms_attempts = 0;
    int ret_val;
    int device_info_read_retry = 0;

    dev_dbg(&commsdevice->dev, "%s >\n", __func__);

    while (2 > comms_attempts)
    {
        mutex_lock(&ephdata->comms_mutex);
        ret_val = eph_read_device_information(ephdata);
        while(ret_val && (device_info_read_retry <= COMMS_READ_RETRY_NUM))
        {
            /* retry we didnt get a device information response */
            ret_val = eph_read_device_information(ephdata);
            device_info_read_retry++;

        }
        mutex_unlock(&ephdata->comms_mutex);

        dev_info(&ephdata->commsdevice->dev,
                 "Product ID: %u Variant ID: %u Application_version_major %u Application_version_minor: %u Bootloader_version: %u Protocol_version: %u CRC:%u\n",
                 ephdata->ephdeviceinfo.product_id, ephdata->ephdeviceinfo.variant_id,
                 ephdata->ephdeviceinfo.application_version_major, ephdata->ephdeviceinfo.application_version_minor,
                 ephdata->ephdeviceinfo.bootloader_version, ephdata->ephdeviceinfo.protocol_version, ephdata->ephdeviceinfo.crc);

        if ((0 == ret_val) )
        {
            if((ephdata->ephdeviceinfo.application_version_major != 0) || (ephdata->ephdeviceinfo.application_version_minor != 0))
            {
                /* sucessfully read from device info and confirmed in application mode */
                ts_info("Identify device in application mode");
                ephdata->in_bootloader = false;
                break;
            }
            else if(ephdata->ephdeviceinfo.variant_id == 0xF0)
            {
                ts_info("Identify device in bootloader mode");
                ephdata->in_bootloader = true;
                break;
            }
            else
            {
                ts_err("Invalid device info!");
            }
        }

        /* Failed to read the device information - try bootloader */
        ret_val = eph_chg_force_bootloader(ephdata);

        /* Check bootloader state */
        ret_val = eph_probe_bootloader(ephdata);

        /* release chg as bootloader actions complete  */
        (void)eph_bootloader_release_chg(ephdata);

        if (ret_val)
        {
            /* Chip is not in appmode or bootloader mode */
            return ret_val;
        }
        comms_attempts++;
        /* OK, we are in bootloader, see if we can recover */
        if (comms_attempts > 1)
        {
            dev_err(&commsdevice->dev, "Could not recover from bootloader mode\n");
            /*
             * We can reflash from this state, so do not
             * abort initialization.
             */
            ephdata->in_bootloader = true;
            return 0;
        }

        /* Attempt to exit bootloader into app mode */
#if 0
        eph_reset_device(ephdata);
        msleep(EPH_FW_RESET_TIME);
#else
        for (size_t i = 0; i < 5; i++)
        {
            eph_power_off(ephdata);
            msleep(400u);
            eph_power_on(ephdata);
            msleep(100u);
        }
#endif
    }

    ret_val = eph_acquire_irq(ephdata);
    if (ret_val)
    {
        return ret_val;
    }

    ret_val = eph_sysfs_mem_access_init(ephdata);
    if (ret_val)
    {
        return ret_val;
    }

    dev_info(&commsdevice->dev, "%s <\n", __func__);

    return ret_val;
}


static int eph_configure_components(struct eph_data *ephdata, const struct firmware *device_settings)
{
    struct device *dev = &ephdata->commsdevice->dev;
    int ret_val = 0;

    dev_dbg(dev, "%s %s >\n", __func__, device_settings ? "device_settings":"-");

    if (device_settings)
    {
        ret_val = eph_update_device_settings(ephdata, device_settings);
        if (ret_val)
        {
            dev_warn(dev, "Error %d updating device_settings\n", ret_val);
        }
    }

    return ret_val;
}

/* Firmware Version is reported as Major.Minor.bootloaderversion */
static ssize_t eph_devattr_fw_version_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u.%u\n",
                     ephdata->ephdeviceinfo.application_version_major, ephdata->ephdeviceinfo.application_version_minor,
                     ephdata->ephdeviceinfo.bootloader_version);
}

/* Hardware Version is reported as DevicefamilyID.DevicevariantID */
static ssize_t eph_devattr_hw_version_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
                     ephdata->ephdeviceinfo.product_id, ephdata->ephdeviceinfo.variant_id);
}


static int eph_enter_bootloader(struct eph_data *ephdata)
{
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (!ephdata->in_bootloader)
    {
        if (ephdata->suspended)
        {
            if (ephdata->ephplatform->suspend_mode == EPH_SUSPEND_REGULATOR)
            {
                eph_power_on(ephdata);
            }
            ephdata->suspended = false;
        }

        /* only disable interrupt and unregister if we have not done so before */
        eph_irq_enable(ephdata, false);

        /* Force device into bootloader mode using chg line held low while toggle reset */
        ret_val = eph_chg_force_bootloader(ephdata);

        if (ret_val)
        {
            /* failed to enter bootloader correctly - restore CHG */
            (void)eph_bootloader_release_chg(ephdata);
            return ret_val;
        }

        ret_val = eph_comms_specific_bootloader_checks(ephdata);
        if (ret_val)
        {
            /* failed to enter bootloader correctly - restore CHG */
            (void)eph_bootloader_release_chg(ephdata);
            return ret_val;
        }

        ephdata->in_bootloader = true;
    }
    else
    {
        /* device already in bootloader mode, reset device to get intial bootloader feedback message */
        ts_info("device already in bootloader mode, reset");
        /* disable irq to avoid 0x50 report readed by irq */
        eph_irq_enable(ephdata, false);
        eph_reset_device(ephdata);
        msleep(EPH_FW_RESET_TIME);
    }

    eph_sysfs_mem_access_remove(ephdata);
    dev_info(&ephdata->commsdevice->dev, "Entered bootloader\n");

    return 0;
}

static int eph_load_fw(struct device *dev)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    ephdata->ephflash = (struct eph_flash*)devm_kzalloc(dev, sizeof(struct eph_flash), GFP_KERNEL);
    if (!ephdata->ephflash)
    {
        return -ENOMEM;
    }

    ephdata->ephflash->ephdata = ephdata;

    dev_dbg(&ephdata->commsdevice->dev, "%s request_firmware name %s \n", __func__, ephdata->fw_name);

    /* Finds fw under the requested name */
    ret_val = request_firmware(&ephdata->ephflash->fw, ephdata->fw_name, dev);
    if (ret_val)
    {
        dev_err(dev, "request_firmware %d %s\n", ret_val, ephdata->fw_name);
        goto free;
    }

    /* Check for incorrect enc file */
    ret_val = eph_check_firmware_format(dev, ephdata->ephflash->fw);
    if (ret_val)
    {
        goto release_firmware;
    }

    eph_get_bin_firmware_version(ephdata, ephdata->ephflash->fw);

    ret_val = eph_firmware_version_compare(ephdata);
    if (!ret_val)
    {
        dev_err(dev, "firmware version same %d\n", ret_val);
        //goto release_firmware;
    }
    else
    {
        dev_info(dev, "firmware version different %d\n", ret_val);
    }

    ret_val = eph_enter_bootloader(ephdata);
    if (ret_val)
    {
        goto release_firmware;
    }

    ret_val = eph_send_frames(ephdata, ephdata->ephflash);
    if (ret_val)
    {
        goto release_firmware;
    }

release_firmware:
    release_firmware(ephdata->ephflash->fw);
free:
    devm_kfree(dev, ephdata->ephflash);
    return ret_val;
}

/* gesture_mode: BIT0 enable/disable
 *               BIT1 tap
 *               BIT2 doubel tap
 *               BIT4 swipe
 */
int eph_gesture_mode_set(struct eph_data *ephdata, u8 gesture_mode)
{
    int ret_val;

    u8 type = TLV_CONFIG_DATA_WRITE;
    u8 prepayload_len = 4;
    u8 comp_id_low = (u8)450;
    u8 comp_id_high = (u8)(450 > 8);
    u8 offset = 24;
    u8 data_len = 1;
    volatile u8 data[] = { gesture_mode & 0xf };
    volatile u8 tlv[] = {type, data_len + prepayload_len, 0x00, comp_id_low, comp_id_high, offset, 0x00, data[0]};
    u16 length = (sizeof(tlv)/sizeof(tlv[0]));

    if (gesture_mode & 0xf0) {
        dev_err(&ephdata->commsdevice->dev, "Invaild gesture mode");
        return -EINVAL;
    }

    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, (u8 *)&tlv[0]);
    mutex_unlock(&ephdata->comms_mutex);

    if (likely(!ret_val))
        ephdata->gesture_mode = gesture_mode;
    else
        dev_err(&ephdata->commsdevice->dev, "Set/Clr gesture mode fail\n");

    return ret_val;
}

int eph_gesture_mode_enable(struct device *dev, u8 gesture_mode)
{
    int ret_val;

    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    u8 type = TLV_CONTROL_DATA_WRITE;
    u8 prepayload_len = 4;
    u8 comp_id_low = (u8)450;
    u8 comp_id_high = (u8)(450 > 8);
    u8 offset = 0;
    u8 data_len = 1;
    volatile u8 data[] = { gesture_mode & 0xf };
    volatile u8 tlv[] = {type, data_len + prepayload_len, 0x00, comp_id_low, comp_id_high, offset, 0x00, data[0]};
    u16 length = (sizeof(tlv)/sizeof(tlv[0]));

    if (gesture_mode & 0xf0) {
        dev_err(&ephdata->commsdevice->dev, "Invaild gesture mode");
        return -EINVAL;
    }

    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_write_control_config(ephdata, length, (u8 *)&tlv[0]);
    mutex_unlock(&ephdata->comms_mutex);

    if (likely(!ret_val))
        ephdata->gesture_mode = gesture_mode;
    else
        dev_err(&ephdata->commsdevice->dev, "Set/Clr gesture mode fail\n");

    return ret_val;
}

static ssize_t eph_devattr_update_fw_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    int ret_val;

    /* Pass in file name and length of name and if accepted put into fw_name */
    ret_val = eph_update_file_name(dev, &ephdata->fw_name, buf, count);
    if (ret_val)
    {
        return ret_val;
    }

    ret_val = eph_load_fw(dev);
    if (ret_val)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret_val);
        count = ret_val;
    }
    else
    {
        dev_info(dev, "The firmware update succeeded\n");

        /* TODO re-check - initial bootloader appears to require an extended delay before moving to communicating with App */
        msleep(EPH_RESET_TIME*2);
        ephdata->suspended = false;

        ret_val = eph_initialize(ephdata);
        if (ret_val)
        {
            return ret_val;
        }
    }

    return count;
}

int eph_update_fw(struct device *dev, const char* name)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    int ret_val;
    int count;

    count = strlen(name);

    /* Pass in file name and length of name and if accepted put into fw_name */
    ret_val = eph_update_file_name(dev, &ephdata->fw_name, name, count);
    if (ret_val)
    {
        return ret_val;
    }

    ret_val = eph_load_fw(dev);
    if (ret_val)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret_val);
    }
    else
    {
        dev_info(dev, "The firmware update succeeded\n");

        /* TODO re-check - initial bootloader appears to require an extended delay before moving to communicating with App */
        msleep(EPH_RESET_TIME*2);
        ephdata->suspended = false;

        ret_val = eph_initialize(ephdata);
    }

    return ret_val;
}

static ssize_t eph_devattr_update_device_settings_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf,
                                            size_t count)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    const struct eph_platform_data *ephplatform = ephdata->ephplatform;
    const struct firmware *device_settings;
    int ret_val;

    ret_val = eph_update_file_name(dev, &ephdata->device_settings_name, buf, count);
    if (ret_val)
    {
        return ret_val;
    }

    /* find the device settings file under the following name */
    ret_val = request_firmware(&device_settings, ephdata->device_settings_name, dev);
    if (ret_val)
    {
        dev_err(dev, "request_firmware %d %s\n", ret_val, ephdata->device_settings_name);
        goto out;
    }

    ephdata->updating_device_settings = true;


    if (ephdata->suspended)
    {
        dev_info(dev, "ESWIN device was in suspend %d\n", ret_val);
        if (ephplatform->suspend_mode == EPH_SUSPEND_REGULATOR)
        {
            eph_irq_enable(ephdata, true);
            eph_power_on(ephdata);
        }
        else if (ephplatform->suspend_mode == EPH_SUSPEND_DEEP_SLEEP)
        {
            /* do nothing as TIC does not currently support sleep */
        }

        ephdata->suspended = false;
    }

    ret_val = eph_configure_components(ephdata, device_settings);
    if (!ret_val)
    {
        /* no error so return count */
        ret_val = count;
    }

    release_firmware(device_settings);
out:
    ephdata->updating_device_settings = false;
    return ret_val;
}


static ssize_t eph_devattr_comms_read(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata;
    int ret_val;
    size_t count = TLV_HEADER_SIZE;

    ephdata = (struct eph_data*)dev_get_drvdata(dev);


    /* Wait for any pending IRQ handler to complete - Gives priority for interrupt handler */
    synchronize_irq(ephdata->chg_irq);

    mutex_lock(&ephdata->comms_mutex);
    ret_val = eph_comms_two_stage_read(ephdata, (u8*)buf);
    mutex_unlock(&ephdata->comms_mutex);


    if(ret_val)
    {
        /* Return NULL message if there is no message or an error occured */
        /* clear 3 bytes to mimic null read */
        memset(buf, 0, TLV_HEADER_SIZE);
    }
    else
    {
        /* decode and return the legth of message */
        count = (size_t)((buf[TLV_LENGTH_FIELD] | ((u16)buf[TLV_LENGTH_FIELD + 1u] << 8u)) + TLV_HEADER_SIZE);
    }

    dev_info(&ephdata->commsdevice->dev, "ESWIN sysfs read the following %d, %d , %d<\n", buf[0], buf[1], buf[2]);

    return ret_val == 0 ? count : ret_val;
}


static ssize_t eph_devattr_comms_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    ssize_t ret_val;

    ret_val = eph_check_mem_access_params(ephdata, 0, &count);
    if (0 < ret_val)
    {
        return ret_val;
    }

    if (0 < count)
    {
        /* Wait for any pending IRQ handler to complete - Gives priority for interrupt handler */
        synchronize_irq(ephdata->chg_irq);

        mutex_lock(&ephdata->comms_mutex);
        ret_val = eph_comms_write(ephdata, count, (u8 *)buf);
        mutex_unlock(&ephdata->comms_mutex);

        dev_info(&ephdata->commsdevice->dev, "ESWIN sysfs: %s. Write the following %d, %d , %d<\n", __func__, buf[0], buf[1], buf[2]);
    }

    return ret_val == 0 ? count : ret_val;
}


static ssize_t eph_devattr_device_report_read(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata;
    struct tlv_header tlvheader;
    ephdata = (struct eph_data*)dev_get_drvdata(dev);



    mutex_lock(&ephdata->sysfs_report_buffer_lock);
    tlvheader = eph_get_tl_header_info(ephdata, sysfs_report_buf);


    if (TLV_HEADER_SIZE>tlvheader.length)
    {
        /* if length was 0 for any reason generate NULL message - Ensures null message copied to the buffer */
        tlvheader.length = TLV_HEADER_SIZE;
        memset(&sysfs_report_buf[0], 0, tlvheader.length);
    }

    /* copy message into page buffer */
    memcpy(buf,&sysfs_report_buf[0],tlvheader.length);

    /* Flush buffer once consumed - to prevent same message being read twice */
    /* relies on message being smaller than a page and the tooling processing the header in that single read */
    memset(&sysfs_report_buf[0], 0, tlvheader.length);

    /* Only generate a message if valid message is being returned. */
    if (0!=tlvheader.type)
    {
        dev_dbg(&ephdata->commsdevice->dev, "ESWIN sysfs read buffer. Type: %d, length: %d", tlvheader.type, tlvheader.length);
    }
    mutex_unlock(&ephdata->sysfs_report_buffer_lock);


    return (size_t)tlvheader.length;

  }


static ssize_t eph_devattr_reset_device(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata;
    int ret_val;
    ephdata = (struct eph_data*)dev_get_drvdata(dev);

    dev_dbg(&ephdata->commsdevice->dev, "%s start gpio_reset \n > \n", __func__);
    eph_reset_device(ephdata);

    dev_dbg(&ephdata->commsdevice->dev, "%s start trigger_baseline \n > \n", __func__);
    ret_val = eph_trigger_baseline(ephdata);
    eph_clear_all_host_touch_slots(ephdata);

    dev_dbg(&ephdata->commsdevice->dev, "%s < \n", __func__);
    return ret_val;

  }

static ssize_t eph_devattr_gesture_wakeup_read(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct eph_data *ephdata;
    ephdata = (struct eph_data*)dev_get_drvdata(dev);

    if (!ephdata)
        return -EIO;

    return sprintf(buf, "%s mode[%x]\n", ephdata->gesture_wakeup_enable ? "enable" : "disable",
            ephdata->gesture_mode);
}

static ssize_t eph_devattr_gesture_wakeup_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct eph_data *ephdata;
    int ret_val = -1;
    int input = 0;
    u8 gesture_mode = 0;
    u8 set_mode = 0;
    ephdata = (struct eph_data*)dev_get_drvdata(dev);

    if (!ephdata)
        return -EIO;

    if (kstrtoint(buf, 10, &input))
        return -EINVAL;

    switch (input) {
    case 1:
        /* enable tap */
        if (ephdata->gesture_mode & BIT(1)) {
            dev_info(&ephdata->commsdevice->dev, "tap already set\n");
            goto exit;
        }
        gesture_mode = BIT(1);
        break;
    case 2:
        /* enable double tap */
        if (ephdata->gesture_mode & BIT(2)) {
            dev_info(&ephdata->commsdevice->dev, "double tap already set\n");
            goto exit;
        }
        gesture_mode = BIT(2);
        break;
    case 3:
        /* enable swipe */
        if (ephdata->gesture_mode & BIT(3)) {
            dev_info(&ephdata->commsdevice->dev, "swipe already set\n");
            goto exit;
        }
        gesture_mode = BIT(3);
        break;
    case -1:
        /* disable tap */
        if ((ephdata->gesture_mode & BIT(1)) == 0) {
            dev_info(&ephdata->commsdevice->dev, "tap not set\n");
            goto exit;
        }
        gesture_mode |= ~BIT(1);
        break;
    case -2:
        /* disable double tap */
        if ((ephdata->gesture_mode & BIT(2)) == 0) {
            dev_info(&ephdata->commsdevice->dev, "double tap not set\n");
            goto exit;
        }
        gesture_mode |= ~BIT(2);
        break;
    case -3:
        /* disable swipe */
        if ((ephdata->gesture_mode & BIT(3)) == 0) {
            dev_info(&ephdata->commsdevice->dev, "swipe not set\n");
            goto exit;
        }
        gesture_mode |= ~BIT(3);
        break;
    case 0:
    default:
        dev_err(&ephdata->commsdevice->dev, "Invaild Para set\n");
        return -EINVAL;
    }

    set_mode = (input > 0) ? (ephdata->gesture_mode | gesture_mode) :
            (ephdata->gesture_mode & gesture_mode);

    ret_val = eph_gesture_mode_set(ephdata, set_mode);

    if (ret_val) {
        dev_err(&ephdata->commsdevice->dev, "mode set/clr fail %x", ret_val);
        goto exit;
    }

    if (ephdata->gesture_mode & 0xE) {
        ephdata->gesture_wakeup_enable = true;
        dev_info(&ephdata->commsdevice->dev, "mode %x enabled\n", ephdata->gesture_mode);
    } else {
        ephdata->gesture_wakeup_enable = false;
        dev_info(&ephdata->commsdevice->dev, "gesture disabled\n");
    }

exit:
    return count;
}

static ssize_t eph_devattr_finger_print_enable(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    ssize_t ret_val = 0;
    int input = 0;

    if (!ephdata)
    {
        return -EIO;
    }

    if (0 < count)
    {
        if (kstrtoint(buf, 10, &input))
        {
            return -EINVAL;
        }
        else
        {
            if (0 == input)
            {
                dev_dbg(&ephdata->commsdevice->dev, "%s disable FOD \n > \n", __func__);
                ret_val = eph_finger_print_enable(ephdata, false);
            }
            else
            {
                dev_dbg(&ephdata->commsdevice->dev, "%s enable FOD \n > \n", __func__);
                ret_val = eph_finger_print_enable(ephdata, true);
            }
        }
    }
    return ret_val == 0 ? count : ret_val;
}

int eph_fod_mode_enable(struct device *dev, bool enable)
{
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    ssize_t ret_val = 0;

    ret_val = eph_finger_print_enable(ephdata, enable);

    return ret_val;
}

int eph_deepsleep_enable(struct device *dev, int enable)
{
    int ret = 0;
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    u8 cfg[] = { 0x8, 0x5, 0x0, 0x0, 0x0, 0x15, 0x0, 0x0 };
    u16 length = sizeof(cfg)/sizeof(cfg[0]);

    if (enable == 1)
        cfg[7] = 0x2;
    else
        cfg[7] = 0x0;

    mutex_lock(&ephdata->comms_mutex);
    ret = eph_write_control_config(ephdata, length, &cfg[0]);
    mutex_unlock(&ephdata->comms_mutex);

    dev_info(&ephdata->commsdevice->dev, "deepsleep %d, %d\n", enable, ret);
    return ret;
}

int eph_screen_on_reporting(struct device *dev, int enable)
{
    int ret = 0;
    struct eph_data *ephdata = (struct eph_data*)dev_get_drvdata(dev);
    u8 cfg[] = { 0x8, 0x6, 0x0, 0xc, 0x0, 0x0, 0x0, 0x0, 0x0 };
    u16 length = sizeof(cfg)/sizeof(cfg[0]);

    if (enable == 1)
    {
        cfg[7] = 0x1;
        cfg[8] = EPH_HOST_REPORTING_TOUCH;
#if defined EPH_ESD_RECOVERY
        if(ephdata->power_on)
        {
            heartbeat_work_start(ephdata);
        }
#endif
    }
    else
    {
        cfg[7] = 0x0;
        cfg[8] = EPH_HOST_REPORTING_GESTURE;
#if defined EPH_ESD_RECOVERY
        if(ephdata->power_on)
        {
            heartbeat_work_stop(ephdata);
        }
#endif
    }

    mutex_lock(&ephdata->comms_mutex);
    ret = eph_write_control_config(ephdata, length, &cfg[0]);
    mutex_unlock(&ephdata->comms_mutex);

    dev_info(&ephdata->commsdevice->dev, "screen_on %d, %d\n", enable, ret);
    return ret;
}

/* debug level show */
static ssize_t eswin_ts_debug_log_show(struct device *dev,
                       struct device_attribute *attr,
                       char *buf)
{
    int r = 0;

    r = snprintf(buf, PAGE_SIZE, "state:%s\n",
            debug_log_flag ?
            "enabled" : "disabled");

    return r;
}

/* debug level store */
static ssize_t eswin_ts_debug_log_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    if (!buf || count <= 0)
        return -EINVAL;

    if (buf[0] != '0')
        debug_log_flag = true;
    else
        debug_log_flag = false;
    return count;
}

/* .attr.name, .attr.mode, .show, .store  */
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, eph_devattr_update_fw_store);
/* S_IRUGO - read-only attributes  */
static DEVICE_ATTR(fw_version, S_IRUGO, eph_devattr_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, eph_devattr_hw_version_show, NULL);
/* S_IWUSR - write access to root only */
static DEVICE_ATTR(update_device_settings, S_IWUSR, NULL, eph_devattr_update_device_settings_store);
static DEVICE_ATTR(write_device_message, S_IWUSR, NULL, eph_devattr_comms_write);
static DEVICE_ATTR(read_device_message, S_IRUGO, eph_devattr_comms_read, NULL);
static DEVICE_ATTR(read_device_report, S_IRUGO, eph_devattr_device_report_read, NULL);
static DEVICE_ATTR(reset_device, S_IRUGO, eph_devattr_reset_device, NULL);
static DEVICE_ATTR(gesture_wakeup, (S_IWUSR|S_IRUGO), eph_devattr_gesture_wakeup_read, eph_devattr_gesture_wakeup_store);
static DEVICE_ATTR(finger_print_enable, S_IWUSR, NULL, eph_devattr_finger_print_enable);
static DEVICE_ATTR(debug_log, 0664, eswin_ts_debug_log_show, eswin_ts_debug_log_store);

static struct attribute *eph_fw_attrs[] =
{
    /* update_fw */
    &dev_attr_update_fw.attr,
    NULL
};

static const struct attribute_group eph_fw_attr_group =
{
    .attrs = eph_fw_attrs,
};

static struct attribute *eph_attrs[] =
{
    /* fw_version */
    &dev_attr_fw_version.attr,
    /* hw_version */
    &dev_attr_hw_version.attr,
    /* update_device_settings */
    &dev_attr_update_device_settings.attr,
    /* write message to device */
    &dev_attr_write_device_message.attr,
    /* read message from device */
    &dev_attr_read_device_message.attr,
    /* read buffered report from device */
    &dev_attr_read_device_report.attr,

    &dev_attr_reset_device.attr,

    &dev_attr_gesture_wakeup.attr,
    /* FOD enable switch */
    &dev_attr_finger_print_enable.attr,

    &dev_attr_debug_log.attr,

    NULL
};

static const struct attribute_group eph_attr_group =
{
    .attrs = eph_attrs,
};

static int eph_sysfs_mem_access_init(struct eph_data *ephdata)
{
    struct comms_device *commsdevice = ephdata->commsdevice;
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    ret_val = sysfs_create_group(&commsdevice->dev.kobj, &eph_attr_group);
    if (ret_val)
    {
        dev_err(&commsdevice->dev, "Failure %d creating sysfs group\n", ret_val);
        sysfs_remove_group(&commsdevice->dev.kobj, &eph_attr_group);
        return ret_val;
    }

    return ret_val;
}

static void eph_sysfs_mem_access_remove(struct eph_data *ephdata)
{
    struct comms_device *commsdevice = ephdata->commsdevice;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    sysfs_remove_group(&commsdevice->dev.kobj, &eph_attr_group);
}


static int eph_start(struct eph_data *ephdata)
{
    struct comms_device *commsdevice = ephdata->commsdevice;

    dev_info(&commsdevice->dev, "%s, suspend_mode %d >\n", __func__, ephdata->ephplatform->suspend_mode);

    if (!ephdata->suspended || ephdata->in_bootloader)
    {
        dev_info(&commsdevice->dev, "%s, suspended %d, in_bootloader = %d <\n", __func__, ephdata->suspended, ephdata->in_bootloader);
        return 0;
    }

    switch (ephdata->ephplatform->suspend_mode)
    {
        case EPH_SUSPEND_REGULATOR:
            eph_irq_enable(ephdata, true);
            #if (ESWIN_EPH861X_I2C)
            eph_power_on(ephdata);
            #endif
            break;

        case EPH_SUSPEND_DEEP_SLEEP:
        default:
            /* do nothing for the moment as no tic sleep */

            break;
    }

    ephdata->suspended = false;

    dev_info(&commsdevice->dev, "%s <\n",__func__);

    return 0;
}

static int eph_stop(struct eph_data *ephdata)
{

    struct comms_device *commsdevice = ephdata->commsdevice;

    dev_info(&commsdevice->dev, "%s, suspend mode %d >\n", __func__, ephdata->ephplatform->suspend_mode);

    if (ephdata->suspended || ephdata->in_bootloader || ephdata->updating_device_settings)
    {
        dev_info(&commsdevice->dev, "%s, suspended %d, in_bootloader %d, updating_device_settings %d <\n",__func__, ephdata->suspended, ephdata->in_bootloader, ephdata->updating_device_settings);
        return 0;
    }

    switch (ephdata->ephplatform->suspend_mode)
    {
        case EPH_SUSPEND_REGULATOR:
            eph_irq_enable(ephdata, false);
            #if (ESWIN_EPH861X_I2C)
            eph_power_off(ephdata);
            #endif
            eph_clear_all_host_touch_slots(ephdata);
            break;

        case EPH_SUSPEND_DEEP_SLEEP:
        default:

            /* For now does nothing as sleep not implemented in TIC */

            /* Clear all the touch slots on UI */
            eph_clear_all_host_touch_slots(ephdata);

    }

    ephdata->suspended = true;

    dev_info(&commsdevice->dev, "%s <\n",__func__);

    return 0;
}

static int eph_input_open(struct input_dev *inputdev)
{
    struct eph_data *ephdata = (struct eph_data *)input_get_drvdata(inputdev);
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    ret_val = eph_start(ephdata);

    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "%s failed rc=%d\n", __func__, ret_val);
    }

    return ret_val;
}

static void eph_input_close(struct input_dev *inputdev)
{
    struct eph_data *ephdata = (struct eph_data *)input_get_drvdata(inputdev);
    int ret_val;

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    ret_val = eph_stop(ephdata);

    if (ret_val)
    {
        dev_err(&ephdata->commsdevice->dev, "%s failed rc=%d\n", __func__, ret_val);
    }
}
#if defined(ESWIN_BOARD_FLORAL) || defined(ESWIN_BOARD_CLOUDRIPPER)// pixel4XL OR pixel7pro plat
static int eph_dev_enter_lp_mode(struct eph_data *ephdata);
static int eph_dev_enter_normal_mode(struct eph_data *ephdata);
#endif
#if defined(ESWIN_BOARD_FLORAL) //pixel4XL platform
static int eph_notifier_callback(struct notifier_block *nb, unsigned long event,
        void *data)
{
    struct eph_data *ephdata = container_of(nb, struct eph_data, notifier);
    struct msm_drm_notifier *evdata = data;
    struct comms_device *commsdevice;
    struct backlight_device *bd = ephdata->bl;
    unsigned int blank;
    int brightness = 0;

    if (!evdata || evdata->id != 0)
        return 0;

    commsdevice = ephdata->commsdevice;

    dev_dbg(&commsdevice->dev, "%s >>>\n", __func__);
    dev_dbg(&commsdevice->dev, "event %x\n", event);

    if (event != MSM_DRM_EVENT_BLANK)
        return 0;

    if (evdata->data && event == MSM_DRM_EVENT_BLANK && ephdata) {
        blank = *(int *)(evdata->data);

        switch (blank) {
        case MSM_DRM_BLANK_POWERDOWN:
            eph_dev_enter_lp_mode(ephdata);
            if (bd->ops && bd->ops->get_brightness)
                brightness = bd->ops->get_brightness(bd);
            else
                brightness = bd->props.brightness;
            ephdata->last_brightness = brightness;
            dev_info(&commsdevice->dev, "brightness %d\n", brightness);
            break;
        case MSM_DRM_BLANK_UNBLANK:
            eph_dev_enter_normal_mode(ephdata);
            schedule_work(&ephdata->force_baseline_work);
            break;
        default:
            break;
        }
    }

    return NOTIFY_OK;
}
#elif defined(ESWIN_BOARD_CLOUDRIPPER) // pixel7Pro platform
struct drm_connector *eph_get_bridge_connector(struct drm_bridge *bridge)
{
    struct drm_connector *connector;
    struct drm_connector_list_iter conn_iter;

    drm_connector_list_iter_begin(bridge->dev, &conn_iter);
    drm_for_each_connector_iter(connector, &conn_iter) {
        if (connector->encoder == bridge->encoder)
            break;
    }
    drm_connector_list_iter_end(&conn_iter);
    return connector;
}

static bool eph_bridge_is_lp_mode(struct drm_connector *connector)
{
    if (connector && connector->state) {
        struct exynos_drm_connector_state *s =
            to_exynos_connector_state(connector->state);
        return s->exynos_mode.is_lp_mode;
    }
    return false;
}

static void eph_panel_bridge_enable(struct drm_bridge *bridge)
{
    struct eph_data *ephdata =
                container_of(bridge, struct eph_data, panel_bridge);
    struct device *dev = &ephdata->commsdevice->dev;

    dev_info(dev, "%s\n", __func__);
    ephdata->is_panel_lp_mode = eph_bridge_is_lp_mode(ephdata->connector);
    if (!ephdata->is_panel_lp_mode) {
        eph_dev_enter_normal_mode(ephdata);
        schedule_work(&ephdata->force_baseline_work);
    }
}

static void eph_panel_bridge_disable(struct drm_bridge *bridge)
{
    struct eph_data *ephdata =
            container_of(bridge, struct eph_data, panel_bridge);
    struct device *dev = &ephdata->commsdevice->dev;

    if (bridge->encoder && bridge->encoder->crtc) {
        const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

        if (drm_atomic_crtc_effectively_active(crtc_state))
            return;
    }

    dev_info(dev, "%s\n", __func__);

    eph_dev_enter_lp_mode(ephdata);
    ephdata->last_brightness = backlight_get_brightness(ephdata->bl);
}

static void eph_panel_bridge_mode_set(struct drm_bridge *bridge,
                        const struct drm_display_mode *mode,
                        const struct drm_display_mode *adjusted_mode)
{
    struct eph_data *ephdata = container_of(bridge, struct eph_data, panel_bridge);
    struct device *dev = &ephdata->commsdevice->dev;

    dev_info(dev, "%s\n", __func__);

    if (!ephdata->connector || !ephdata->connector->state) {
        ephdata->connector = eph_get_bridge_connector(bridge);
        dev_err(dev, "%s: Get bridge connector.\n", __func__);
    }

}

static const struct drm_bridge_funcs panel_bridge_funcs = {
    .enable = eph_panel_bridge_enable,
    .disable = eph_panel_bridge_disable,
    .mode_set = eph_panel_bridge_mode_set,
};

static int eph_register_panel_bridge(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;
#ifdef CONFIG_OF
    ephdata->panel_bridge.of_node = dev->of_node;
#endif
    ephdata->panel_bridge.funcs = &panel_bridge_funcs;
    drm_bridge_add(&ephdata->panel_bridge);

    dev_info(dev, "%s\n", __func__);

    return 0;
}

static void eph_unregister_panel_bridge(struct eph_data *ephdata)
{
    struct drm_bridge *node;
    struct drm_bridge *bridge = &ephdata->panel_bridge;

    drm_bridge_remove(bridge);

    if (!bridge->dev) /* not attached */
        return;

    drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
    list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node)
        if (node == bridge) {
            if (bridge->funcs->detach)
                bridge->funcs->detach(bridge);
            list_del(&bridge->chain_node);
            break;
        }
    drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
    bridge->dev = NULL;
}
#endif //USE_DRM_BRIDGE

static int eph_pinctrl_configure(struct eph_data *ephdata, bool enable)
{
    struct pinctrl_state *state;

    if (IS_ERR_OR_NULL(ephdata->pinctrl)) {
        dev_warn(&ephdata->commsdevice->dev, "Invalid pinctrl\n");
        return -EINVAL;
    }

    dev_dbg(&ephdata->commsdevice->dev, "%s enable %d >>>\n", __func__, enable);

    if (enable) {
        state = pinctrl_lookup_state(ephdata->pinctrl, "ts_active");
        if (IS_ERR(state))
            dev_err(&ephdata->commsdevice->dev, "Could not get ts_active pinstate!\n");
    } else {
        state = pinctrl_lookup_state(ephdata->pinctrl, "ts_suspend");
        if (IS_ERR(state))
            dev_err(&ephdata->commsdevice->dev, "Could not get ts_suspend pinstate!\n");
    }
    if (!IS_ERR_OR_NULL(state))
        return pinctrl_select_state(ephdata->pinctrl, state);

    return 0;
}
#if defined EPH_ESD_RECOVERY
void heartbeat_work_start(struct eph_data *ephdata)
{
    struct device *dev = &ephdata->commsdevice->dev;

    if (atomic_read(&ephdata->heartbeat_on)) {
        dev_info(dev, "heartbeat already on\n");
        return;
    }

    atomic_set(&ephdata->heartbeat_on, 1);
    schedule_delayed_work(&ephdata->heartbeat_work, msecs_to_jiffies(300));

    return;
}
void heartbeat_work_stop(struct eph_data *ephdata)
{
    atomic_set(&ephdata->heartbeat_on, 0);
    cancel_delayed_work(&ephdata->heartbeat_work);
    flush_delayed_work(&ephdata->heartbeat_work);

    return;
}
static int send_heartbeat(struct eph_data *ephdata)
{
    int ret;
    mutex_lock(&ephdata->comms_mutex);
    ret = eph_read_device_information(ephdata);
    mutex_unlock(&ephdata->comms_mutex);
    return ret;
}
static void heartbeat_work_handler(struct work_struct *work)
{
    int ret = 0;
    struct eph_data *ephdata = container_of(work, struct eph_data, heartbeat_work.work);
    if(NULL == ephdata)
    {
        ts_err("heartbeat_work_handler faield...get no valid ephdata!!!\n");
        return;
    }

    ts_debug("ic heartbeat work...\n");

    if (!atomic_read(&ephdata->heartbeat_on)) {
        ts_info("ic heartbeat off\n");
        return;
    }
#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
    if (atomic_read(&ephdata->trigger_enable)) {
        ts_info("log triggered, heartbeat off\n");
        return;
    }
#endif
    if (ephdata->suspended)
    {
        if (ephdata->power_on)
        {
            if (ephdata->in_bootloader)
            {
                ts_info("system suspend, ic in boot mode!\n");
                goto ic_reset;
            }
            else
            {
                // ic in gesture mode or deep mode or idle
                ret = send_heartbeat(ephdata);
                if (ret) {
                    ts_err("suspend, heartbeat fail %d\n", ret);
                    goto ic_recovery;
                }
            }
        }
        else
        {
            ts_info("system power off do not check heartbeat!\n");
        }

    }
    else
    {
        if (ephdata->in_bootloader)
        {
            /* may loading firmware or setting */
            ts_info("system active, ic in boot mode!\n");
        }
        else
        {
            // ic in active or idle mode
            ret = send_heartbeat(ephdata);
            if (ret)
            {
                ts_err("suspend, heartbeat fail %d\n", ret);
                goto ic_recovery;
            }
        }
    }

    schedule_delayed_work(&ephdata->heartbeat_work, msecs_to_jiffies(5000));
    return;
ic_recovery:
    eph_recovery_device(ephdata);
    eph_clear_all_host_touch_slots(ephdata);
    schedule_delayed_work(&ephdata->heartbeat_work, msecs_to_jiffies(3000));
    return;
ic_reset:
    eph_reset_device(ephdata);
    eph_clear_all_host_touch_slots(ephdata);
    schedule_delayed_work(&ephdata->heartbeat_work, msecs_to_jiffies(3000));
    return;
}
#endif

// TODO: alloc in ephdata
u8 tmp_buf[4096];

static int eph_proc_debug_open(struct inode *inode, struct file *file)
{
        file->private_data = pde_data(inode);
        return 0;
}

static ssize_t eph_proc_debug_read(struct file *file, char __user *usr_buf, size_t count, loff_t *pos)
{
    int ret = 0;
    struct eph_data *ephdata = (struct eph_data *)file->private_data;
    int msg_len = 0;

    if (!ephdata) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }

    if (!ephdata->power_on) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }

    ts_debug("proc read count %zu, pos %lld\n", count, *pos);

    synchronize_irq(ephdata->chg_irq);

    mutex_lock(&ephdata->comms_mutex);
    ret = eph_comms_two_stage_read(ephdata, tmp_buf);
    mutex_unlock(&ephdata->comms_mutex);
    if(ret) {
        msg_len = 0;
    } else {
        msg_len = (size_t)((tmp_buf[TLV_LENGTH_FIELD] | ((u16)tmp_buf[TLV_LENGTH_FIELD + 1u] << 8u)) + TLV_HEADER_SIZE);
    }
    ts_debug("read %d bytes from ic\n", msg_len);
    if (msg_len) {
        ts_info("read: %d %d %d\n", tmp_buf[0], tmp_buf[1], tmp_buf[2]);
    }

    if(msg_len && !copy_to_user(usr_buf, tmp_buf, msg_len))
    {
        ts_debug("success copy message to user!");
        ret = msg_len;
    }
    else
    {
        ts_err("Failed Read or copy message to user!");
        ret = -EFAULT;
    }
    memset(tmp_buf, 0, 4096);

    return ret;
}

static ssize_t eph_proc_debug_write(struct file *file, const char __user *usr_buf, size_t count, loff_t *pos)
{
    int ret = 0;
    struct eph_data *ephdata = (struct eph_data *)file->private_data;
    if (!ephdata) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }

    if (!ephdata->power_on) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }

    memset(tmp_buf, 0, 4096);

    if (copy_from_user(tmp_buf, usr_buf, count)) {
        ts_err("copy data fail\n");
        return -EFAULT;
    }

    tmp_buf[4095] = '\0';

    ts_debug("eswin proc write\n");

    mutex_lock(&ephdata->comms_mutex);
    ret = eph_comms_write(ephdata, count, (u8 *)tmp_buf);
    mutex_unlock(&ephdata->comms_mutex);

    if (ret) {
        ts_err("eph proc debug write fail\n");
    }

    return (ret != 0) ? ret : count;
}

static ssize_t eph_proc_report_read(struct file *file, char __user *usr_buf, size_t count, loff_t *pos)
{
    struct eph_data *ephdata = (struct eph_data *)file->private_data;
    struct tlv_header tlvheader;

    if (!ephdata) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }
    if (!ephdata->power_on) {
        ts_err("debug read get null private data\n");
        return -EFAULT;
    }

    mutex_lock(&ephdata->sysfs_report_buffer_lock);
    tlvheader = eph_get_tl_header_info(ephdata, sysfs_report_buf);
    if (TLV_HEADER_SIZE > tlvheader.length) {
        tlvheader.length = TLV_HEADER_SIZE;
        memset(&sysfs_report_buf[0], 0, tlvheader.length);
    }

    if (copy_to_user(usr_buf, &sysfs_report_buf[0], tlvheader.length)) {
        ts_err("eph proc report read fail\n");
        return -EFAULT;
    }
    memset(&sysfs_report_buf[0], 0, tlvheader.length);

    if (0 != tlvheader.type) {
        ts_info("ESWIN proc read buffer. Type: %d, length: %d", tlvheader.type, tlvheader.length);
    }
    mutex_unlock(&ephdata->sysfs_report_buffer_lock);

    return (size_t)tlvheader.length;
}

struct proc_ops eph_debug_ops = {
    .proc_open = eph_proc_debug_open,
    .proc_read = eph_proc_debug_read,
    .proc_write = eph_proc_debug_write,
};

struct proc_ops eph_report_ops = {
    .proc_open = eph_proc_debug_open,
    .proc_read = eph_proc_report_read,
};

#if (ESWIN_EPH861X_SPI)
static int eph_probe(struct comms_device *commsdevice)
#endif
#if (ESWIN_EPH861X_I2C)
static int eph_probe(struct comms_device *commsdevice, const struct comms_device_id *id)
#endif
{
    struct eph_data *ephdata;
    const struct eph_platform_data *ephplatform;
    int ret_val;
    struct proc_dir_entry *procfs_entry;

    dev_dbg(&commsdevice->dev, "%s >>>\n", __func__);

    /* init spi_device */
    struct spi_delay d;
    d.value = 100;
    d.unit = SPI_DELAY_UNIT_USECS;
    commsdevice->cs_setup        = d;
    commsdevice->mode            = SPI_MODE_3;
    commsdevice->bits_per_word   = 8;
    commsdevice->chip_select = 0;
    ret_val = spi_setup(commsdevice);
    if (ret_val) {
        dev_err(&commsdevice->dev, "failed set spi mode, %d\n", ret_val);
        return ret_val;
    }
    ret_val = eph_comms_specific_checks(commsdevice);
    if (ret_val)
    {
        return -EINVAL;
    }

    if (commsdevice->dev.of_node && !mmi_device_is_available(commsdevice->dev.of_node))
    {
        dev_err(&commsdevice->dev, "mmi: device not supported\n");
        return -ENODEV;
    }

    ephplatform = eph_platform_data_get(commsdevice);
    if (IS_ERR(ephplatform))
    {
        return PTR_ERR(ephplatform);
    }

    ephdata = (struct eph_data *)kzalloc(sizeof(struct eph_data), GFP_KERNEL);
    if (!ephdata)
    {
        return -ENOMEM;
    }

    ephdata->bl = backlight_device_get_by_type(BACKLIGHT_RAW);

    if (ephdata->bl) {
        dev_info(&commsdevice->dev, "backlight brightness %x\n", ephdata->bl->props.brightness);
        dev_info(&commsdevice->dev, "backlight max brightness %x\n", ephdata->bl->props.max_brightness);
    } else {
        dev_info(&commsdevice->dev, "backlight not ready!\n");
    }

    INIT_WORK(&ephdata->force_baseline_work, eph_trigger_baseline_work);

    ephdata->commsdevice = commsdevice;
    ephdata->ephplatform = ephplatform;
    eph_comms_driver_data_set(commsdevice, ephdata);

    ephdata->pinctrl = devm_pinctrl_get(&commsdevice->dev);
    if (IS_ERR_OR_NULL(ephdata->pinctrl)) {
        dev_warn(&commsdevice->dev, "Could not get pinctrl\n");
    } else {
        eph_pinctrl_configure(ephdata, true);
    }

    ret_val = eph_allocate_comms_memory(commsdevice, ephdata);


    if (ephdata->ephplatform->device_settings_name)
    {
        eph_update_file_name(&ephdata->commsdevice->dev,
                             &ephdata->device_settings_name,
                             ephdata->ephplatform->device_settings_name,
                             strlen(ephdata->ephplatform->device_settings_name));
    }

    if (ephdata->ephplatform->fw_name)
    {
        eph_update_file_name(&ephdata->commsdevice->dev,
                             &ephdata->fw_name,
                             ephdata->ephplatform->fw_name,
                             strlen(ephdata->ephplatform->fw_name));
    }

    dev_info(&commsdevice->dev, "%s ephdata->fw_name: %s, ephdata->device_settings_name: %s \n", __func__, ephdata->fw_name, ephdata->device_settings_name);

    init_completion(&ephdata->chg_completion);
    init_completion(&ephdata->reset_completion);
#if !defined(ESWIN_SYSTEM_SUSPEND)
    init_completion(&ephdata->pm_completion);
    ephdata->pm_suspend = false;
#endif


    mutex_init(&ephdata->comms_mutex);
    mutex_init(&ephdata->sysfs_report_buffer_lock);
    mutex_init(&ephdata->fw_upgrade_mutex);

    device_init_wakeup(&commsdevice->dev, true);

    ret_val = eph_gpio_setup(ephdata);
    if (ret_val)
    {
        goto err_free_irq;
    }

    ret_val = eph_acquire_irq(ephdata);
    if (ret_val)
    {
        goto err_free_mem;
    }

    ret_val = eph_probe_regulators(ephdata);
    if (ret_val)
    {
        goto err_free_irq;
    }

    /* Need to have IRQ disabled before calling eph_initialize() as it re-enables it */
    eph_irq_enable(ephdata, false);

    ret_val = sysfs_create_group(&commsdevice->dev.kobj, &eph_fw_attr_group);
    if (ret_val)
    {
        dev_err(&commsdevice->dev, "Failure %d creating fw sysfs group\n", ret_val);
        return ret_val;
    }

    ret_val = eph_initialize(ephdata);
    if (ret_val)
    {
        goto err_free_irq;
    }

#if defined(ESWIN_BOARD_FLORAL) // pixel4XL plat
    ephdata->notifier.notifier_call = eph_notifier_callback;
    ret_val = msm_drm_register_client(&ephdata->notifier);
    if (ret_val < 0)
        dev_err(&commsdevice->dev, "Failure %d register msm drm client\n", ret_val);
    else
        dev_info(&commsdevice->dev, "success register norifier\n");
#elif defined(ESWIN_BOARD_CLOUDRIPPER) // pixel7Pro plat
    ret_val = eph_register_panel_bridge(ephdata);
    if (ret_val < 0)
        dev_err(&commsdevice->dev, "Failure %d panel bridge\n", ret_val);
    else
        dev_info(&commsdevice->dev, "success register panel bridge\n");
#endif

    /* Async load ic firmware */
    dev_info(&commsdevice->dev, "fw_name: %s\n", ephdata->fw_name);
    if(ephdata->fw_name)
    {
        ret_val = request_firmware_nowait(THIS_MODULE,
                                          true,
                                          ephdata->fw_name,
                                          &ephdata->commsdevice->dev,
                                          GFP_KERNEL,
                                          ephdata,
                                          eph_request_fw_cb);
        if (ret_val)
        {
            dev_err(&commsdevice->dev, "Failed to load ic firmware: %d\n", ret_val);
        }
    }

    ret_val = eph_input_device_initialize(ephdata);
    if (ret_val) {
        dev_err(&commsdevice->dev, "eph input dev init fail: %d\n", ret_val);
    }

    /* default disable gesture */
    ephdata->gesture_mode = 0x0;
    ephdata->gesture_wakeup_enable = false;

    ephdata->lp = false;
    ephdata->irq_wake = false;
#if defined EPH_ESD_RECOVERY
    INIT_DELAYED_WORK(&ephdata->heartbeat_work, heartbeat_work_handler);
    heartbeat_work_start(ephdata);
    //schedule_delayed_work(&ephdata->heartbeat_work, msecs_to_jiffies(5000));
#endif
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
    dev_info(&commsdevice->dev, "%s: eswin_ts_mmi_dev_register\n", __func__);
    ret_val = eswin_ts_mmi_dev_register(commsdevice);
    if (ret_val) {
        dev_err(&commsdevice->dev, "Failure %d register mmi\n", ret_val);
    } else {
        dev_info(&commsdevice->dev, "success register mmi\n");
    }
#endif

    proc_mkdir("eph_ts", NULL);
    procfs_entry = proc_create_data("eph_ts/eph_debug", 0666, NULL, &eph_debug_ops, ephdata);
    if (!procfs_entry) {
        ts_err("eswin create proc fail\n");
    }
    procfs_entry = proc_create_data("eph_ts/eph_report", 0666, NULL, &eph_report_ops, ephdata);
    if (!procfs_entry) {
        ts_err("eswin create proc fail\n");
    }

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
    ret_val = eswin_log_capture_register_misc(ephdata);
    if (ret_val)
        ts_err("Failed register log device, %d\n", ret_val);

    atomic_set(&ephdata->allow_capture, 1);
    ts_info("Enable ghost log capture after probe\n");
#endif
    debug_log_flag = false;
    dev_info(&commsdevice->dev, "%s <\n", __func__);
    return 0;

err_free_irq:
    if (ephdata->chg_irq)
    {
        free_irq(ephdata->chg_irq, ephdata);
    }

    gpio_free(ephdata->ephplatform->gpio_reset);
    gpio_free(ephdata->ephplatform->gpio_chg_irq);
    gpio_free(ephdata->ephplatform->gpio_avdd);
    if(ephdata->reg_vdd)
    {
        regulator_put(ephdata->reg_vdd);
    }
    if(ephdata->reg_avdd)
    {
        regulator_put(ephdata->reg_avdd);
    }
err_free_mem:
    kfree(ephdata);
    dev_info(&commsdevice->dev, "%s error\n", __func__);
    return ret_val;
}

static void eph_remove(struct comms_device *commsdevice)
{
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);
    dev_info(&commsdevice->dev, "%s >\n", __func__);
#if defined EPH_ESD_RECOVERY
    cancel_delayed_work_sync(&ephdata->heartbeat_work);
#endif

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
    eswin_log_capture_unregister_misc(ephdata);
#endif

    sysfs_remove_group(&commsdevice->dev.kobj, &eph_fw_attr_group);
    eph_sysfs_mem_access_remove(ephdata);
    remove_proc_entry("eph_ts/eph_debug", NULL);
    remove_proc_entry("eph_ts/eph_report", NULL);
    remove_proc_entry("eph_ts", NULL);

#if defined(ESWIN_BOARD_FLORAL)
    msm_drm_unregister_client(&ephdata->notifier);
#elif defined(ESWIN_BOARD_CLOUDRIPPER)
    eph_unregister_panel_bridge(ephdata);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
    dev_info(&commsdevice->dev, "%s: eswin_ts_mmi_dev_unregister\n",__func__);
    eswin_ts_mmi_dev_unregister(commsdevice);
#endif

    if (ephdata->chg_irq)
    {
        free_irq(ephdata->chg_irq, ephdata);
    }

    gpio_free(ephdata->ephplatform->gpio_reset);
    gpio_free(ephdata->ephplatform->gpio_chg_irq);
    gpio_free(ephdata->ephplatform->gpio_avdd);

    if(ephdata->reg_avdd)
    {
        regulator_put(ephdata->reg_avdd);
    }
    if(ephdata->reg_vdd)
    {
        regulator_put(ephdata->reg_vdd);
    }
    eph_unregister_input_device(ephdata);

#if (ESWIN_EPH861X_SPI && ESWIN_EPH861X_SPI_USE_DMA)
    dma_pool_free(pool_rx, ephdata->comms_receive_buf, ephdata->comms_dma_handle_rx);
    dma_pool_free(pool_tx, ephdata->comms_send_buf, ephdata->comms_dma_handle_tx);

    dma_pool_destroy(pool_rx);
    dma_pool_destroy(pool_tx);
#elif (ESWIN_EPH861X_SPI)
    kfree(ephdata->comms_receive_buf);
    kfree(ephdata->comms_send_buf);
#endif

    kfree(ephdata->comms_send_crc_buf);
    kfree(ephdata->report_buf);
    kfree(ephdata);


    return;
}
#if defined(ESWIN_BOARD_FLORAL) || defined(ESWIN_BOARD_CLOUDRIPPER)// pixel4XL OR pixel7pro plat
static int eph_dev_enter_lp_mode(struct eph_data *ephdata)
{
    int ret_val = 0;
    struct device *dev = &ephdata->commsdevice->dev;

    if (ephdata->lp)
        return 0;

    if (!ephdata->lp) {
        if (!ephdata->irq_wake) {
            enable_irq_wake(ephdata->chg_irq);
            ephdata->irq_wake = true;
        }
        if (ephdata->gesture_wakeup_enable)
            ret_val = eph_gesture_mode_set(ephdata, ephdata->gesture_mode | BIT(0));

        ephdata->lp = true;
    }

    if (ret_val)
        dev_err(dev, "Failed to enter lp mode (%d)\n", ret_val);

    return ret_val;
}

static int eph_dev_enter_normal_mode(struct eph_data *ephdata)
{
    int ret_val = 0;
    struct device *dev = &ephdata->commsdevice->dev;

    if (!ephdata->lp)
        return 0;

    if (ephdata->lp) {
        if (ephdata->irq_wake) {
            disable_irq_wake(ephdata->chg_irq);
            ephdata->irq_wake = false;
        }

        if (ephdata->gesture_wakeup_enable)
            ret_val = eph_gesture_mode_set(ephdata, ephdata->gesture_mode & (~BIT(0)));

        ephdata->lp = false;
    }

    if (ret_val)
        dev_err(dev, "Failed to enter normal mode (%d)\n", ret_val);

    return ret_val;
}
#endif

#if defined(ESWIN_SYSTEM_SUSPEND)
static int __maybe_unused eph_suspend(struct device *dev)
{
    struct comms_device *commsdevice = eph_comms_device_get(dev);
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (!ephdata->inputdev)
    {
        return 0;
    }

    if (ephdata->suspended == true)
        return 0;

    cancel_work_sync(&ephdata->force_baseline_work);

    mutex_lock(&ephdata->inputdev->mutex);

    if (ephdata->inputdev->users)
    {
        (void)eph_stop(ephdata);
    }

    mutex_unlock(&ephdata->inputdev->mutex);

    eph_pinctrl_configure(ephdata, false);

    return 0;
}

static int __maybe_unused eph_resume(struct device *dev)
{
    struct comms_device *commsdevice = eph_comms_device_get(dev);
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    dev_dbg(&ephdata->commsdevice->dev, "%s >\n", __func__);

    if (!ephdata->inputdev)
    {
        return 0;
    }

    if (ephdata->suspended == false)
        return 0;

    eph_pinctrl_configure(ephdata, true);

    mutex_lock(&ephdata->inputdev->mutex);

    if (ephdata->inputdev->users)
    {
        (void)eph_start(ephdata);
    }

    mutex_unlock(&ephdata->inputdev->mutex);

    /* TIC report gesture event need 150 ~ 170ms delay 200ms for irq
     * process gesture event
    */
    mdelay(200);

    {

        eph_clear_all_host_touch_slots(ephdata);
    }

    return 0;
}
#else
static int __maybe_unused eph_suspend(struct device *dev)
{
    struct comms_device *commsdevice = eph_comms_device_get(dev);
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    ts_info("system enters into pm_suspend");
    ephdata->pm_suspend = true;
    reinit_completion(&ephdata->pm_completion);
    return 0;
}

static int __maybe_unused eph_resume(struct device *dev)
{
    struct comms_device *commsdevice = eph_comms_device_get(dev);
    struct eph_data *ephdata = eph_comms_driver_data_get(commsdevice);

    ts_info("system resumes from pm_suspend");
    ephdata->pm_suspend = false;
    complete(&ephdata->pm_completion);
    return 0;
}
#endif
static SIMPLE_DEV_PM_OPS(eph_pm_ops, eph_suspend, eph_resume);

#ifdef CONFIG_OF // Open Firmware (Device Tree)
static const struct of_device_id eph_of_match[] =
{
    { .compatible = "eswin,eph861x", },
    {},
};
MODULE_DEVICE_TABLE(of, eph_of_match);
#endif // CONFIG_OF

static const struct comms_device_id eph_id[] =
{
    { "eswin_eph861x", 0 },
    { }
};
MODULE_DEVICE_TABLE(comms_mode_type, eph_id);

static struct comms_driver eph_driver =
{
    .id_table   = eph_id,
    .probe      = eph_probe,
    .remove     = eph_remove,
    .driver = {
        .name   = "eswin_eph861x",
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(eph_of_match),
        .pm = &eph_pm_ops,

    },

};

module_comms_driver(eph_driver);

/* Module information */
MODULE_AUTHOR("chris.ollerenshaw@eswin.com>");
MODULE_DESCRIPTION("ESWIN EPH861 series Touchscreen driver");
MODULE_LICENSE("GPL");
