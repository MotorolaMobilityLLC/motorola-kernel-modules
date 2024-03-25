#define LOG_TAG         "Sysfs"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_tcs.h"
#include "cts_test.h"
#include "cts_sfctrl.h"
#include "cts_spi_flash.h"
#include "cts_firmware.h"
#include "cts_strerror.h"

#ifdef CONFIG_CTS_SYSFS

#define SPLIT_LINE_STR \
    "-----------------------------------------------"\
    "------------------------------------------------\n"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%4u "
#define DATA_FORMAT_STR     "%5d"

#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->hwdata->num_row * cts_dev->hwdata->num_col * 2)

#define MAX_ARG_NUM                 (100)
#define MAX_ARG_LENGTH              (1024)
#define HW_STUB_ADDR                (0XF000)

static char cmdline_param[MAX_ARG_LENGTH + 1];
int argc;
char *argv[MAX_ARG_NUM];

static int jitter_test_frame = 10;


int parse_arg(const char *buf, size_t count)
{
    char *p;

    memcpy(cmdline_param, buf, min((size_t)MAX_ARG_LENGTH, count));
    cmdline_param[count] = '\0';

    argc = 0;
    p = strim(cmdline_param);
    if (p == NULL || p[0] == '\0')
        return 0;

    while (p && p[0] != '\0' && argc < MAX_ARG_NUM)
        argv[argc++] = strsep(&p, " ,");

    return argc;
}


#ifdef CFG_CTS_FW_UPDATE_SYS
static ssize_t cts_panel_supplier_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *data = dev_get_drvdata(dev);

    if (data->pdata && data->pdata->panel_supplier) {
        return scnprintf(buf, PAGE_SIZE, "%s\n",
                data->pdata->panel_supplier);
    }
    return 0;
}

static ssize_t buildid_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "0000-%04x\n",
            cts_data->cts_dev.fwdata.version);
}

static ssize_t forcereflash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    unsigned int input;

    if (kstrtouint(buf, 10, &input) != 0)
        return -EINVAL;

    cts_data->force_reflash = (input == 0) ? false : true;

    cts_info("%s force_reflash=%d, count=%zu", __func__,
            (cts_data->force_reflash ? 1 : 0), count);
    return count;
}

static ssize_t flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%d\n",
            cts_is_firmware_updating(&cts_data->cts_dev) ? 1 : 0);
}

static bool is_reflash_filename_valid(const struct chipone_ts_data *cts_data,
        const char *filename)
{
    char prefix[CFG_CTS_FW_FILE_NAME_MAX_LEN];

    if (cts_data->pdata->panel_supplier != NULL) {
        snprintf(prefix, sizeof(prefix), "%s-%s-%s-",
                CFG_CTS_FW_FILE_NAME_VENDOR,
                cts_data->pdata->panel_supplier,
                cts_data->cts_dev.hwdata->name);
    } else {
        /* panel supplier not set, just check vendor. */
        snprintf(prefix, sizeof(prefix), "%s",
                CFG_CTS_FW_FILE_NAME_VENDOR);
    }

    cts_info("%s: prefix=%s", __func__, prefix);
    if (strncmp(filename, prefix, strlen(prefix))) {
        cts_err("%s: invalid FW file.", __func__);
        return false;
    }

    return true;
}

static ssize_t doreflash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    int ret;

    cts_info("doreflash FW filename: %s len: %zu", buf, count);

    if (count > CFG_CTS_FW_FILE_NAME_MAX_LEN) {
        cts_err("doreflash FW filename is too long %zu > %d",
            count, CFG_CTS_FW_FILE_NAME_MAX_LEN);
        return -EINVAL;
    }

    if (cts_is_device_suspended(&cts_data->cts_dev)) {
        cts_err("In suspend state, try again later");
        return -EAGAIN;
    }

    if (cts_is_firmware_updating(&cts_data->cts_dev)) {
        cts_err("In FW flashing state, try again later");
        return -EAGAIN;
    }

    if (!cts_data->force_reflash) {
        /* Check filename if force_reflash is false */
        if (!is_reflash_filename_valid(cts_data, buf)) {
            cts_err("Invalid firmware filename '%*.s'", (int)count, buf);
            return -EINVAL;
        }
    }

    strncpy(cts_data->cts_dev.config_fw_name, buf, count);

    /* If use echo xxx > doreflash, 0x0A will append to the string,
     *  if use echo -n xxx > doreflash, nothing will append.
     */
    if (cts_data->cts_dev.config_fw_name[count - 1] == '\n')
        cts_data->cts_dev.config_fw_name[count - 1] = '\0';
    else
        cts_data->cts_dev.config_fw_name[count] = '\0';

    cts_stop_device(&cts_data->cts_dev);

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_update_firmware_from_file(&cts_data->cts_dev,
            cts_data->cts_dev.config_fw_name, false);
    cts_unlock_device(&cts_data->cts_dev);

    if (ret)
        cts_err("Update firmware from file '%s' failed %d",
            cts_data->cts_dev.config_fw_name, ret);

    cts_start_device(&cts_data->cts_dev);

    cts_data->force_reflash = false;

    cts_info("%s: end", __func__);

    return ret ? ret : count;
}

static ssize_t cts_poweron_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    bool val;

    /* TBD: check if cts is power to ready for flash.
     * set "1" if power on ready.
     */
    val = cts_is_device_suspended(&cts_data->cts_dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", val == false);
}

static ssize_t cts_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    /* set chip IC type to productinfo */
    return scnprintf(buf, PAGE_SIZE, "%s\n",
            cts_data->cts_dev.hwdata->name);
}

/* add sys entries for FW update */
/* static DEVICE_ATTR(drv_irq, S_IRUGO | S_IWUSR, drv_irq_show, drv_irq_store); */
/* static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, reset_store); */
static DEVICE_ATTR(panel_supplier, 0444, cts_panel_supplier_show, NULL);
static DEVICE_ATTR(buildid, S_IRUGO, buildid_show, NULL);
static DEVICE_ATTR(forcereflash, S_IWUSR | S_IWGRP, NULL, forcereflash_store);
static DEVICE_ATTR(flashprog, S_IRUGO, flashprog_show, NULL);
static DEVICE_ATTR(doreflash, S_IWUSR | S_IWGRP, NULL, doreflash_store);
static DEVICE_ATTR(poweron, S_IRUGO, cts_poweron_show, NULL);
static DEVICE_ATTR(productinfo, S_IRUGO, cts_productinfo_show, NULL);

static struct attribute *cts_dev_fw_up_atts[] = {
    /**
     * &dev_attr_drv_irq.attr,
     * &dev_attr_reset.attr,
     */
    &dev_attr_buildid.attr,
    &dev_attr_forcereflash.attr,
    &dev_attr_flashprog.attr,
    &dev_attr_doreflash.attr,
    &dev_attr_poweron.attr,
    &dev_attr_productinfo.attr,
    &dev_attr_panel_supplier.attr,
    NULL
};

static const struct attribute_group cts_dev_fw_up_attr_group = {
    .attrs = cts_dev_fw_up_atts,
};
#endif


static ssize_t write_tcs_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr;
    int i, ret;
    u8 *data = NULL;

    parse_arg(buf, count);

    cts_info("Write firmware register '%.*s'", (int)count, buf);

    if (argc < 2) {
        cts_err("Too few args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        cts_err("Invalid address %s", argv[0]);
        return -EINVAL;
    }

    data = (u8 *) kmalloc(argc - 1, GFP_KERNEL);
    if (data == NULL) {
        cts_err("Allocate buffer for write data failed\n");
        return -ENOMEM;
    }

    for (i = 1; i < argc; i++) {
        ret = kstrtou8(argv[i], 0, data + i - 1);
        if (ret) {
            cts_err("Invalid value %s", argv[i]);
            goto free_data;
        }
    }

    ret = cts_tcs_write(cts_dev, addr, data, argc - 1);
    if (ret) {
        cts_err("Write tcs register addr: 0x%04x size: %d failed",
            addr, argc - 1);
        goto free_data;
    }

free_data:
    kfree(data);

    return (ret < 0 ? ret : count);
}
static DEVICE_ATTR(write_tcs_reg, S_IWUSR, NULL, write_tcs_register_store);

static ssize_t read_tcs_register_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
#define PRINT_ROW_SIZE          (16)
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;

    cts_info("Read tcs register ");

    if (argc != 2) {
        return snprintf(buf, PAGE_SIZE,
                "Invalid num args %d\n"
                "  1. echo (0x)addr size > read_reg\n"
                "  2. cat read_reg\n", argc);
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        return snprintf(buf, PAGE_SIZE, "Invalid address: %s\n", argv[0]);
    }
    ret = kstrtou16(argv[1], 0, &size);
    if (ret)
        return snprintf(buf, PAGE_SIZE, "Invalid size: %s\n", argv[1]);

    data = (u8 *) kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return snprintf(buf, PAGE_SIZE, "Allocate buffer for read data failed\n");
    }

    cts_info("Read tcs register from 0x%04x size %u", addr, size);
    cts_lock_device(cts_dev);
    ret = cts_tcs_read(cts_dev, addr, data, (size_t)size);
    cts_unlock_device(cts_dev);
    if (ret) {
        count = snprintf(buf, PAGE_SIZE,
            "Read tcs register from 0x%04x size %u failed %d\n", addr, size, ret);
        goto err_free_data;
    }

    remaining = size;
    for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
        size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);

        remaining -= PRINT_ROW_SIZE;

        count += snprintf(buf + count, PAGE_SIZE - count, "%04x: ", addr);

        /* Lower version kernel return void */
        hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                buf + count, PAGE_SIZE - count, true);
        count += strlen(buf + count);

        if (count < PAGE_SIZE) {
            buf[count++] = '\n';
            addr += PRINT_ROW_SIZE;
        } else {
            break;
        }
    }

err_free_data:
    kfree(data);

    return count;
#undef PRINT_ROW_SIZE
}

static ssize_t read_tcs_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return (argc == 0 ? 0 : count);
}

static DEVICE_ATTR(read_tcs_reg, S_IWUSR | S_IRUSR,
        read_tcs_register_show, read_tcs_register_store);

static ssize_t read_hw_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define PRINT_ROW_SIZE          (16)
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;

    cts_info("Read hw register");

    if (argc != 2) {
        return snprintf(buf, PAGE_SIZE,
                "Invalid num args %d\n"
                "  1. echo addr size > read_hw_reg\n"
                "  2. cat read_hw_reg\n", argc);
    }

    ret = kstrtou32(argv[0], 0, &addr);
    if (ret) {
        return snprintf(buf, PAGE_SIZE, "Invalid address: %s\n", argv[0]);
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret)
        return snprintf(buf, PAGE_SIZE, "Invalid size: %s\n", argv[1]);

    data = (u8 *) kmalloc(size, GFP_KERNEL);
    if (data == NULL)
        return snprintf(buf, PAGE_SIZE, "Allocate buffer for read data failed\n");

    cts_info("Read hw register from 0x%04x size %u", addr, size);
    cts_lock_device(cts_dev);

    if (cts_dev->rtdata.program_mode) {
        for (i = 0; i < size; i++) {
            ret = cts_dev_readb(cts_dev, addr + i, data + i, 3, 10);
            if (ret) {
                count = snprintf(buf, PAGE_SIZE, "Write hw register error\n");
                goto err_free_data;
            }
        }
    } else {
        ret = cts_tcs_read_hw_reg(cts_dev, addr, data, size);
        if (ret < 0) {
            count = snprintf(buf, PAGE_SIZE, "Read hw register error\n");
            goto err_free_data;
        }
    }

    remaining = size;
    for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
        size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);

        remaining -= PRINT_ROW_SIZE;

        count += snprintf(buf + count, PAGE_SIZE - count, "%04x: ", addr);

        /* Lower version kernel return void */
        hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
            buf + count, PAGE_SIZE - count, true);
        count += strlen(buf + count);

        if (count < PAGE_SIZE) {
            buf[count++] = '\n';
            addr += PRINT_ROW_SIZE;
        } else {
            break;
        }
    }

err_free_data:
    cts_unlock_device(cts_dev);
    kfree(data);

    return count;
#undef PRINT_ROW_SIZE
}

/* echo addr size > read_reg */
static ssize_t read_hw_reg_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return (argc == 0 ? 0 : count);
}

static DEVICE_ATTR(read_hw_reg, S_IRUSR | S_IWUSR, read_hw_reg_show,
        read_hw_reg_store);

static ssize_t write_hw_reg_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 addr;
    int i, ret;
    u8 *data = NULL;

    parse_arg(buf, count);

    cts_info("Write hw register");

    if (argc < 2) {
        cts_err("Too few args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &addr);
    if (ret) {
        cts_err("Invalid address %s", argv[0]);
        return -EINVAL;
    }

    data = (u8 *) kmalloc(argc - 1, GFP_KERNEL);
    if (data == NULL) {
        cts_err("Allocate buffer for write data failed\n");
        return -ENOMEM;
    }

    for (i = 1; i < argc; i++) {
        ret = kstrtou8(argv[i], 0, data + i - 1);
        if (ret) {
            cts_err("Invalid value %s", argv[i]);
            goto free_data;
        }
    }

    cts_lock_device(cts_dev);

    if (cts_dev->rtdata.program_mode) {
        for (i = 0; i < argc - 1; i++) {
            ret = cts_dev_writeb(cts_dev, addr + i, data[i], 3, 10);
            if (ret) {
                cts_err("Write hw register error");
                break;
            }
        }
    } else {
        ret = cts_tcs_write_hw_reg(cts_dev, addr, data, argc - 1);
        if (ret < 0)
            cts_err("Write hw register error");
    }

    cts_unlock_device(cts_dev);
free_data:
    kfree(data);

    return (ret < 0 ? ret : count);
}

static DEVICE_ATTR(write_hw_reg, S_IWUSR, NULL, write_hw_reg_store);

static ssize_t curr_firmware_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Current firmware version: %04x\n",
        cts_data->cts_dev.fwdata.version);
}

static DEVICE_ATTR(curr_version, S_IRUGO, curr_firmware_version_show, NULL);

static ssize_t curr_ddi_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Current ddi version: %02x\n",
            cts_data->cts_dev.fwdata.ddi_version);
}

static DEVICE_ATTR(curr_ddi_version, S_IRUGO, curr_ddi_version_show, NULL);

static ssize_t rows_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Num rows: %u\n",
            cts_data->cts_dev.fwdata.rows);
}

static DEVICE_ATTR(rows, S_IRUGO, rows_show, NULL);

static ssize_t cols_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Num cols: %u\n",
            cts_data->cts_dev.fwdata.cols);
}

static DEVICE_ATTR(cols, S_IRUGO, cols_show, NULL);

static ssize_t res_x_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "X Resolution: %u\n",
            cts_data->cts_dev.fwdata.res_x);
}

static DEVICE_ATTR(res_x, S_IRUGO, res_x_show, NULL);

static ssize_t res_y_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Y Resolution: %u\n",
            cts_data->cts_dev.fwdata.res_y);
}

static DEVICE_ATTR(res_y, S_IRUGO, res_y_show, NULL);

static ssize_t esd_protection_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8 esd_protection;

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_esd_protection(cts_dev, &esd_protection);
    cts_unlock_device(cts_dev);
    if (ret)
        return snprintf(buf, PAGE_SIZE,
                "Read firmware ESD protection register failed %d\n", ret);

    return snprintf(buf, PAGE_SIZE, "ESD protection: %u\n", esd_protection);
}

static DEVICE_ATTR(esd_protection, S_IRUGO, esd_protection_show, NULL);

static ssize_t monitor_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8 value;

    cts_lock_device(cts_dev);
    ret = cts_tcs_is_mnt_enabled(cts_dev, &value);
    cts_unlock_device(cts_dev);
    if (ret)
        return snprintf(buf, PAGE_SIZE,
            "Read firmware monitor enable register failed %d\n", ret);

    return snprintf(buf, PAGE_SIZE, "Monitor mode: %s\n",
        value & BIT(0) ? "Enable" : "Disable");
}

static ssize_t monitor_mode_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8 value, enable = 0;

    if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1')
        enable = 1;

    cts_info("Write firmware monitor mode to '%c', %s",
            buf[0], enable ? "Enable" : "Disable");

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(&cts_data->cts_dev, 0x8000 + 344, &value);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("Write firmware monitor enable register failed %d", ret);
        return -EIO;
    }

    if ((value & BIT(0)) && enable)
        cts_info("Monitor mode already enabled");
    else if ((value & BIT(0)) == 0 && enable == 0)
        cts_info("Monitor mode already disabled");
    else {
        if (enable)
            value |= BIT(0);
        else
            value &= ~BIT(0);

        cts_lock_device(cts_dev);
        ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x8000 + 344, value);
        cts_unlock_device(cts_dev);
        if (ret) {
            cts_err("Write firmware monitor enable register failed %d", ret);
            return -EIO;
        }
    }

    return count;
}

static DEVICE_ATTR(monitor_mode, S_IRUGO, monitor_mode_show,
        monitor_mode_store);

static ssize_t auto_compensate_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8 value;

    cts_lock_device(cts_dev);
    ret = cts_tcs_is_cneg_enabled(&cts_data->cts_dev, &value);
    cts_unlock_device(cts_dev);
    if (ret)
        return snprintf(buf, PAGE_SIZE,
            "Read auto compensate enable register failed %d\n", ret);

    return snprintf(buf, PAGE_SIZE, "Auto compensate: %s\n",
            value ? "Enable" : "Disable");
}

static DEVICE_ATTR(auto_compensate, S_IRUGO, auto_compensate_show, NULL);

#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
static ssize_t driver_builtin_firmware_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, count = 0;

    count += snprintf(buf + count, PAGE_SIZE - count,
        "Total %d builtin firmware:\n", cts_get_num_driver_builtin_firmware());

    for (i = 0; i < cts_get_num_driver_builtin_firmware(); i++) {
        const struct cts_firmware *firmware =
                cts_request_driver_builtin_firmware_by_index(i);
        if (firmware)
            count += snprintf(buf + count, PAGE_SIZE - count,
                    "%-2d: hwid: %04x fwid: %04x ver: %04x size: %6zu desc: %s\n",
                    i, firmware->hwid, firmware->fwid,
                    FIRMWARE_VERSION(firmware),
                    firmware->size, firmware->name);
        else
            count += snprintf(buf + count, PAGE_SIZE - count,
                    "%-2d: INVALID\n", i);
    }

    return count;
}

/* echo index/name [flash/sram] > driver_builtin_firmware */
static ssize_t driver_builtin_firmware_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret, index = -1;

    parse_arg(buf, count);

    if (argc != 1 && argc != 2) {
        cts_err("Invalid num args %d\n"
            "  echo index/name [flash/sram] > driver_builtin_firmware\n",
            argc);
        return -EFAULT;
    }

    if (isdigit(*argv[0]))
        index = simple_strtoul(argv[0], NULL, 0);

    if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0)
            to_flash = true;
        else if (strncasecmp(argv[1], "sram", 4) == 0)
            to_flash = false;
        else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'", argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update driver builtin firmware '%s' to %s",
         argv[1], to_flash ? "flash" : "sram");

    if (index >= 0 && index < cts_get_num_driver_builtin_firmware())
        firmware = cts_request_driver_builtin_firmware_by_index(index);
    else
        firmware = cts_request_driver_builtin_firmware_by_name(argv[0]);

    if (firmware) {
        ret = cts_stop_device(cts_dev);
        if (ret) {
            cts_err("Stop device failed %d", ret);
            return ret;
        }

        cts_lock_device(cts_dev);
        ret = cts_update_firmware(cts_dev, firmware, to_flash);
        cts_unlock_device(cts_dev);

        if (ret) {
            cts_err("Update firmware failed %d", ret);
            goto err_start_device;
        }

        ret = cts_start_device(cts_dev);
        if (ret) {
            cts_err("Start device failed %d", ret);
            return ret;
        }
    } else {
        cts_err("Firmware '%s' NOT found", argv[0]);
        return -ENOENT;
    }

    return count;

err_start_device:
    cts_start_device(cts_dev);

    return ret;
}

static DEVICE_ATTR(driver_builtin_firmware, S_IWUSR | S_IRUGO,
        driver_builtin_firmware_show, driver_builtin_firmware_store);
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */

#ifdef CFG_CTS_FIRMWARE_IN_FS
/* echo filepath [flash/sram] > update_firmware_from_file */
static ssize_t update_firmware_from_file_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret;

    parse_arg(buf, count);

    if (argc > 2) {
        cts_err("Invalid num args %d\n"
            "  echo filepath [flash/sram] > update_from_file\n",
            argc);
        return -EFAULT;
    } else if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0)
            to_flash = true;
        else if (strncasecmp(argv[1], "sram", 4) == 0)
            to_flash = false;
        else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'", argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update firmware from file '%s'", argv[0]);

    firmware = cts_request_firmware_from_fs(cts_dev, argv[0]);
    if (firmware == NULL) {
        cts_err("Request firmware from file '%s' failed", argv[0]);
        return -ENOENT;
    }

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d", ret);
        goto err_release_firmware;
    }

    cts_lock_device(cts_dev);
    ret = cts_update_firmware(cts_dev, firmware, to_flash);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Update firmware failed %d", ret);
        goto err_release_firmware;
    }

    ret = cts_start_device(cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        goto err_release_firmware;
    }

    ret = count;

err_release_firmware:
    cts_release_firmware(firmware);

    return ret;
}

static DEVICE_ATTR(update_from_file, S_IWUSR, NULL,
        update_firmware_from_file_store);
#endif /* CFG_CTS_FIRMWARE_IN_FS */

static ssize_t updating_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Updating: %s\n",
            cts_data->cts_dev.rtdata.updating ? "Y" : "N");
}

static DEVICE_ATTR(updating, S_IRUGO, updating_show, NULL);

static struct attribute *cts_dev_firmware_atts[] = {
    &dev_attr_curr_version.attr,
    &dev_attr_curr_ddi_version.attr,
    &dev_attr_rows.attr,
    &dev_attr_cols.attr,
    &dev_attr_res_x.attr,
    &dev_attr_res_y.attr,
    &dev_attr_esd_protection.attr,
    &dev_attr_monitor_mode.attr,
    &dev_attr_auto_compensate.attr,
#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
    &dev_attr_driver_builtin_firmware.attr,
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */
#ifdef CFG_CTS_FIRMWARE_IN_FS
    &dev_attr_update_from_file.attr,
#endif /* CFG_CTS_FIRMWARE_IN_FS */
    &dev_attr_updating.attr,
    NULL
};

static const struct attribute_group cts_dev_firmware_attr_group = {
    .name = "cts_firmware",
    .attrs = cts_dev_firmware_atts,
};

static ssize_t flash_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_flash *flash;

    if (cts_dev->flash == NULL) {
        bool program_mode;
        bool enabled;
        int ret;

        program_mode = cts_is_device_program_mode(cts_dev);
        enabled = cts_is_device_enabled(cts_dev);

        ret = cts_prepare_flash_operation(cts_dev);
        if (ret)
            return snprintf(buf, PAGE_SIZE,
                    "Prepare flash operation failed %d", ret);

        cts_post_flash_operation(cts_dev);

        if (!program_mode) {
            ret = cts_enter_normal_mode(cts_dev);
            if (ret)
                return snprintf(buf, PAGE_SIZE,
                        "Enter normal mode failed %d", ret);
        }

        if (enabled) {
            ret = cts_start_device(cts_dev);
            if (ret)
                return snprintf(buf, PAGE_SIZE,
                        "Start device failed %d", ret);
        }

        if (cts_dev->flash == NULL)
            return snprintf(buf, PAGE_SIZE, "Flash not found\n");
    }

    flash = cts_dev->flash;
    return snprintf(buf, PAGE_SIZE,
            "%s:\n"
            "  JEDEC ID   : %06X\n"
            "  Page size  : 0x%zx\n"
            "  Sector size: 0x%zx\n"
            "  Block size : 0x%zx\n"
            "  Total size : 0x%zx\n",
            flash->name, flash->jedec_id, flash->page_size,
            flash->sector_size, flash->block_size,
            flash->total_size);
}

static DEVICE_ATTR(info, S_IRUGO, flash_info_show, NULL);

static ssize_t read_flash_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;
    bool program_mode;
    bool enabled;
#ifndef CFG_CTS_FOR_GKI
    loff_t pos = 0;
#endif

    if (argc != 2 && argc != 3)
        return snprintf(buf, PAGE_SIZE, "Invalid num args %d\n", argc);

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret)
        return snprintf(buf, PAGE_SIZE, "Invalid flash addr: %s\n", argv[0]);
    ret = kstrtou32(argv[1], 0, &size);
    if (ret)
        return snprintf(buf, PAGE_SIZE, "Invalid size: %s\n", argv[1]);

    data = (u8 *) kmalloc(size, GFP_KERNEL);
    if (data == NULL)
        return snprintf(buf, PAGE_SIZE, "Allocate buffer for read data failed\n");

    cts_info("Read flash from 0x%06x size %u%s%s", flash_addr, size,
        argc == 3 ? " to file " : "", argc == 3 ? argv[2] : "");

    cts_lock_device(cts_dev);
    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        count += snprintf(buf, PAGE_SIZE, "Prepare flash operation failed %d", ret);
        goto err_free_data;
    }

    ret = cts_read_flash(cts_dev, flash_addr, data, size);
    if (ret) {
        count = snprintf(buf, PAGE_SIZE, "Read flash data failed %d\n", ret);
        goto err_post_flash_operation;
    }

    if (argc == 3) {
#ifdef CFG_CTS_FOR_GKI
        cts_info("%s(): some functions are forbiddon with GKI Version!", __func__);
#else
        struct file *file;

        cts_info("Write flash data to file '%s'", argv[2]);

        file = filp_open(argv[2], O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (IS_ERR(file)) {
            count += snprintf(buf, PAGE_SIZE, "Open file '%s' failed %ld",
                argv[2], PTR_ERR(file));
            goto err_post_flash_operation;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
        ret = kernel_write(file, data, size, &pos);
#else
        ret = kernel_write(file, data, size, pos);
#endif
        if (ret != size)
            count += snprintf(buf, PAGE_SIZE,
                "Write flash data to file '%s' failed %d", argv[2], ret);

        ret = filp_close(file, NULL);
        if (ret)
            count += snprintf(buf, PAGE_SIZE,
                "Close file '%s' failed %d", argv[2], ret);
#endif
    } else {
#define PRINT_ROW_SIZE          (16)
        remaining = size;
        for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
            size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);

            remaining -= PRINT_ROW_SIZE;

            count += snprintf(buf + count, PAGE_SIZE - count - 1,
                "%04x-%04x: ", flash_addr >> 16, flash_addr & 0xFFFF);
            /* Lower version kernel return void */
            hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                buf + count, PAGE_SIZE - count - 1, true);
            count += strlen(buf + count);
            buf[count++] = '\n';
            flash_addr += linelen;
#undef PRINT_ROW_SIZE
        }
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);

        if (r)
            count += snprintf(buf, PAGE_SIZE, "Enter normal mode failed %d", r);
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);

        if (r) {
            cts_unlock_device(cts_dev);
            return snprintf(buf, PAGE_SIZE, "Start device failed %d", r);
        }
    }
err_free_data:
    cts_unlock_device(cts_dev);
    kfree(data);

    return (ret < 0 ? ret : count);
}

/* echo start_addr size [filepath] > read */
static ssize_t read_flash_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}

static DEVICE_ATTR(read, S_IWUSR | S_IRUGO, read_flash_show, read_flash_store);

/* echo addr size > erase */
static ssize_t erase_flash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size;
    int ret;
    bool program_mode;
    bool enabled;

    parse_arg(buf, count);

    if (argc != 2) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret) {
        cts_err("Invalid flash addr: %s", argv[0]);
        return -EINVAL;
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        cts_err("Invalid size: %s", argv[1]);
        return -EINVAL;
    }

    cts_info("Erase flash from 0x%06x size %u", flash_addr, size);

    cts_lock_device(cts_dev);
    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        cts_err("Prepare flash operation failed %d", ret);
        cts_unlock_device(cts_dev);
        return ret;
    }

    ret = cts_erase_flash(cts_dev, flash_addr, size);
    if (ret) {
        cts_err("Erase flash from 0x%06x size %u failed %d",
            flash_addr, size, ret);
        goto err_post_flash_operation;
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);

        if (r)
            cts_err("Enter normal mode failed %d", r);
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);

        if (r)
            cts_err("Start device failed %d", r);
    }
    cts_unlock_device(cts_dev);

    return (ret < 0 ? ret : count);
}

static DEVICE_ATTR(erase, S_IWUSR, NULL, erase_flash_store);

static struct attribute *cts_dev_flash_attrs[] = {
    &dev_attr_info.attr,
    &dev_attr_read.attr,
    &dev_attr_erase.attr,

    NULL
};

static const struct attribute_group cts_dev_flash_attr_group = {
    .name = "flash",
    .attrs = cts_dev_flash_attrs,
};

static ssize_t open_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_OPEN,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                CTS_TEST_FLAG_VALIDATE_MIN |
                CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = "/sdcard/open-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
    int min = 0;
    int ret;
    ktime_t start_time, end_time, delta_time;

    if (argc != 1) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args %d\n", argc);
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE, "Invalid min thres: %s\n", argv[0]);
    }

    cts_info("Open test, threshold = %u", min);

    test_param.min = &min;

    start_time = ktime_get();

    ret = cts_test_open(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret > 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Open test has %d nodes FAIL, min: %u, ELAPSED TIME: %lldms\n",
            ret, min, ktime_to_ms(delta_time));
    } else if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Open test FAIL %d(%s), ELAPSED TIME: %lldms\n",
            ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Open test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }

}

/* echo threshod > open_test */
static ssize_t open_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}

static DEVICE_ATTR(open_test, S_IWUSR | S_IRUGO, open_test_show,
        open_test_store);

static ssize_t short_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_SHORT,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                CTS_TEST_FLAG_VALIDATE_MIN |
                CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = "/sdcard/short-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
    int min = 0;
    int ret;
    ktime_t start_time, end_time, delta_time;

    if (argc != 1) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args %d\n", argc);
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE, "Invalid min thres: %s\n", argv[0]);
    }

    cts_info("Short test, threshold = %u", min);

    test_param.min = &min;

    start_time = ktime_get();

    ret = cts_test_short(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret > 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Short test has %d nodes FAIL, min: %u, ELAPSED TIME: %lldms\n",
            ret, min, ktime_to_ms(delta_time));
    } else if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Short test FAIL %d(%s), ELAPSED TIME: %lldms\n",
            ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Short test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }
}

/* echo threshod > short_test */
static ssize_t short_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}

static DEVICE_ATTR(short_test, S_IWUSR | S_IRUGO,
        short_test_show, short_test_store);

static ssize_t testing_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Testting: %s\n",
            cts_data->cts_dev.rtdata.testing ? "Y" : "N");
}

static DEVICE_ATTR(testing, S_IRUGO, testing_show, NULL);

#ifdef CFG_CTS_HAS_RESET_PIN
static ssize_t reset_pin_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef CFG_CTS_HAS_RESET_PIN
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_RESET_PIN,
        .flags = 0,
    };
    int ret;
    ktime_t start_time, end_time, delta_time;

    start_time = ktime_get();

    ret = cts_test_reset_pin(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
                "Reset-Pin test FAIL %d(%s), ELAPSED TIME: %lldms\n",
                ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Reset-Pin test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }
#else /* CFG_CTS_HAS_RESET_PIN */
    return scnprintf(buf, PAGE_SIZE,
        "Reset-Pin test NOT supported(CFG_CTS_HAS_RESET_PIN not defined)\n");
#endif
}
static DEVICE_ATTR(reset_pin_test, S_IRUGO, reset_pin_test_show, NULL);
#endif

static ssize_t int_pin_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_INT_PIN,
        .flags = 0,
    };
    int ret;
    ktime_t start_time, end_time, delta_time;

    start_time = ktime_get();

    ret = cts_test_int_pin(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Int-Pin test FAIL %d(%s), ELAPSED TIME: %lldms\n",
                ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Int-Pin test PASS, ELAPSED TIME: %lldms\n",
                ktime_to_ms(delta_time));
    }
}
static DEVICE_ATTR(int_pin_test, S_IRUGO, int_pin_test_show, NULL);

static ssize_t compensate_cap_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_COMPENSATE_CAP,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/comp-cap-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
    int min = 0, max = 0;
    int ret;
    ktime_t start_time, end_time, delta_time;

    if (argc != 2) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args\n"
                "USAGE:\n"
                "  1. echo min max > compensate_cap_test\n"
                "  2. cat compensate_cap_test\n");
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
                "Invalid min thres: %s\n", argv[0]);
    }

    ret = kstrtoint(argv[1], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: %s\n", argv[1]);
    }

    cts_info("Compensate cap test, min: %u, max: %u", min, max);

    test_param.min = &min;
    test_param.max = &max;

    start_time = ktime_get();

    ret = cts_test_compensate_cap(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret > 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Compensate cap test has %d nodes FAIL, "
            "threshold[%u, %u], ELAPSED TIME: %lldms\n",
            ret, min, max, ktime_to_ms(delta_time));
    } else if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Compensate cap test FAIL %d(%s), ELAPSED TIME: %lldms\n",
            ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Compensate cap test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }
}

/* echo threshod > short_test */
static ssize_t compensate_cap_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}

static DEVICE_ATTR(compensate_cap_test, S_IWUSR | S_IRUGO,
        compensate_cap_test_show, compensate_cap_test_store);

static ssize_t rawdata_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_rawdata_test_priv_param priv_param = {
        .frames = 16,
        //.work_mode = 0,
    };
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = "/sdcard/rawdata-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .priv_param = &priv_param,
        .priv_param_size = sizeof(priv_param),
    };

    int min, max;
    int ret;
    ktime_t start_time, end_time, delta_time;

    if (argc < 2 || argc > 3) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args\n"
            "USAGE:\n"
            "  1. echo min max [frames] > rawdata_test\n"
            "  2. cat rawdata_test\n");
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid min thres: %s\n", argv[0]);
    }

    ret = kstrtoint(argv[1], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: %s\n", argv[1]);
    }

    if (argc > 2) {
        ret = kstrtou32(argv[2], 0, &priv_param.frames);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Invalid frames: %s\n", argv[2]);
        }
    }
    cts_info("Rawdata test, frames: %u min: %d, max: %d",
        priv_param.frames, min, max);

    test_param.min = &min;
    test_param.max = &max;

    start_time = ktime_get();

    ret = cts_test_rawdata(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret > 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Rawdata test has %d nodes FAIL, threshold[%u, %u], "
            "ELAPSED TIME: %lldms\n",
            ret, min, max, ktime_to_ms(delta_time));
    } else if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Rawdata test FAIL %d(%s), ELAPSED TIME: %lldms\n",
            ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Rawdata test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }
}

/* echo threshod > short_test */
static ssize_t rawdata_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}

static DEVICE_ATTR(rawdata_test, S_IWUSR | S_IRUGO,
        rawdata_test_show, rawdata_test_store);

static ssize_t noise_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_noise_test_priv_param priv_param = {
        .frames = 50,
        //.work_mode = 0,
    };
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/noise-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .priv_param = &priv_param,
        .priv_param_size = sizeof(priv_param),
    };

    int max;
    int ret;
    ktime_t start_time, end_time, delta_time;

    if (argc < 1 || argc > 2) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args\n"
            "USAGE:\n"
            "  1. echo threshold [frames] > noise_test\n"
            "  2. cat noise_test\n");
    }

    ret = kstrtoint(argv[0], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: %s\n", argv[0]);
    }

    if (argc > 1) {
        ret = kstrtou32(argv[1], 0, &priv_param.frames);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Invalid frames: %s\n", argv[1]);
        }
    }
    cts_info("Noise test, frames: %u threshold: %d",
        priv_param.frames, max);

    test_param.max = &max;

    start_time = ktime_get();

    ret = cts_test_noise(cts_dev, &test_param);

    end_time = ktime_get();
    delta_time = ktime_sub(end_time, start_time);

    if (ret > 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Noise test has %d nodes FAIL, max: %u, ELAPSED TIME: %lldms\n",
            ret, max, ktime_to_ms(delta_time));
    } else if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE,
            "Noise test FAIL %d(%s), ELAPSED TIME: %lldms\n",
            ret, cts_strerror(ret), ktime_to_ms(delta_time));
    } else {
        return scnprintf(buf, PAGE_SIZE,
            "Noise test PASS, ELAPSED TIME: %lldms\n",
            ktime_to_ms(delta_time));
    }
}

static ssize_t noise_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(noise_test, S_IWUSR | S_IRUGO,
        noise_test_show, noise_test_store);

static struct attribute *cts_dev_test_atts[] = {
    &dev_attr_open_test.attr,
    &dev_attr_short_test.attr,
    &dev_attr_testing.attr,
#ifdef CFG_CTS_HAS_RESET_PIN
    &dev_attr_reset_pin_test.attr,
#endif
    &dev_attr_int_pin_test.attr,
    &dev_attr_compensate_cap_test.attr,
    &dev_attr_rawdata_test.attr,
    &dev_attr_noise_test.attr,
    NULL
};

static const struct attribute_group cts_dev_test_attr_group = {
    .name = "test",
    .attrs = cts_dev_test_atts,
};

static ssize_t ic_type_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "IC Type : %s\n",
            cts_data->cts_dev.hwdata->name);
}

static DEVICE_ATTR(ic_type, S_IRUGO, ic_type_show, NULL);

static ssize_t program_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "Program mode: %s\n",
            cts_data->cts_dev.rtdata.program_mode ? "Y" : "N");
}

static ssize_t program_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    int ret;

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    if (*argv[0] == '1' || tolower(*argv[0]) == 'y') {
        ret = cts_enter_program_mode(&cts_data->cts_dev);
        if (ret) {
            cts_err("Enter program mode failed %d", ret);
            return ret;
        }
    } else if (*argv[0] == '0' || tolower(*argv[0]) == 'n') {
        ret = cts_enter_normal_mode(&cts_data->cts_dev);
        if (ret) {
            cts_err("Exit program mode failed %d", ret);
            return ret;
        }
    } else
        cts_err("Invalid args");

    return count;
}

static DEVICE_ATTR(program_mode, S_IWUSR | S_IRUGO,
    program_mode_show, program_mode_store);

static ssize_t rawdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 *rawdata = NULL;
    int ret, r, c, count = 0;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Show rawdata");

    rawdata = (u16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL)
        return snprintf(buf, PAGE_SIZE,
                "Allocate memory for rawdata failed\n");

    cts_lock_device(cts_dev);
    ret = cts_tcs_top_get_rawdata(cts_dev, (u8 *) rawdata,
            RAWDATA_BUFFER_SIZE(cts_dev));
    if (ret < 0) {
        cts_err("Get rawdata failed");
        goto err_free_rawdata;
    }
    data_valid = true;

    if (data_valid) {
        max = min = rawdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                u16 val = rawdata[r * cts_dev->hwdata->num_col + c];
                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += snprintf(buf + count, PAGE_SIZE - count,
                  SPLIT_LINE_STR
                  "Raw data MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
                  SPLIT_LINE_STR
                  "   |  ", min_r, min_c, min, max_r, max_c,
                  max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++)
            count += snprintf(buf + count, PAGE_SIZE - count,
                    COL_NUM_FORMAT_STR, c);

        count += snprintf(buf + count, PAGE_SIZE - count,
                "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows && count < PAGE_SIZE; r++) {
            count += snprintf(buf + count, PAGE_SIZE - count,
                    ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols && count < PAGE_SIZE; c++)
                count += snprintf(buf + count, PAGE_SIZE - count - 1,
                    DATA_FORMAT_STR, rawdata[r * cts_dev->hwdata->num_col + c]);

            buf[count++] = '\n';
        }
    }

err_free_rawdata:
    cts_unlock_device(cts_dev);
    kfree(rawdata);

    return data_valid ? count : ret;
}

static DEVICE_ATTR(rawdata, S_IRUGO, rawdata_show, NULL);

static ssize_t diffdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *diffdata = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Show diffdata");

    diffdata = (s16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (diffdata == NULL) {
        cts_err("Allocate memory for diffdata failed");
        return -ENOMEM;
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_top_get_real_diff(cts_dev, (u8 *) diffdata,
            RAWDATA_BUFFER_SIZE(cts_dev));
    cts_unlock_device(cts_dev);
    if (ret < 0) {
        cts_err("Get diffdata failed");
        goto err_free_diffdata;
    }
    data_valid = true;

    if (data_valid) {
        max = min = diffdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val = diffdata[r * cts_dev->hwdata->num_col + c];

                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += snprintf(buf + count, PAGE_SIZE - count,
                  SPLIT_LINE_STR
                  "Diff data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d\n"
                  SPLIT_LINE_STR
                  "   |  ", min_r, min_c, min, max_r, max_c,
                  max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++)
            count += snprintf(buf + count, PAGE_SIZE - count,
                    COL_NUM_FORMAT_STR, c);

        count += snprintf(buf + count, PAGE_SIZE - count,
                "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += snprintf(buf + count, PAGE_SIZE - count,
                    ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++)
                count += snprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, diffdata[r * cts_dev->hwdata->num_col + c]);

            buf[count++] = '\n';
        }
    }

err_free_diffdata:
    kfree(diffdata);

    return data_valid ? count : ret;
}

static DEVICE_ATTR(diffdata, S_IRUGO, diffdata_show, NULL);


static ssize_t manualdiff_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *manualdiff = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Show manualdiff");

    manualdiff = (s16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (manualdiff == NULL) {
        cts_err("Allocate memory for manualdiff failed");
        return -ENOMEM;
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_top_get_manual_diff(cts_dev, (u8 *) manualdiff,
            RAWDATA_BUFFER_SIZE(cts_dev));
    cts_unlock_device(cts_dev);
    if (ret < 0) {
        cts_err("Get manualdiff failed");
        goto err_free_manualdiff;
    }
    data_valid = true;

    if (data_valid) {
        max = min = manualdiff[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val = manualdiff[r * cts_dev->hwdata->num_col + c];

                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += snprintf(buf + count, PAGE_SIZE - count,
                  SPLIT_LINE_STR
                  "Diff data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d\n"
                  SPLIT_LINE_STR
                  "   |  ", min_r, min_c, min, max_r, max_c,
                  max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++)
            count += snprintf(buf + count, PAGE_SIZE - count,
                    COL_NUM_FORMAT_STR, c);

        count += snprintf(buf + count, PAGE_SIZE - count,
                "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += snprintf(buf + count, PAGE_SIZE - count,
                    ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++)
                count += snprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, manualdiff[r * cts_dev->hwdata->num_col + c]);

            buf[count++] = '\n';
        }
    }

err_free_manualdiff:
    kfree(manualdiff);

    return data_valid ? count : ret;
}

static DEVICE_ATTR(manualdiff, S_IRUGO, manualdiff_show, NULL);

static ssize_t jitter_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u16 cnt;
    int ret;

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &cnt);
    if (ret == 0) {
        if (cnt > 2 && cnt < 10000)
            jitter_test_frame = cnt;
    }

    cts_info("jitter test frame: %d", jitter_test_frame);
    return count;
}

static ssize_t jitter_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *rawdata = NULL;
    s16 *rawdata_min = NULL;
    s16 *rawdata_max = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = false;
    int i;

    cts_info("Show jitter");
    cts_lock_device(cts_dev);
    rawdata = (s16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        cts_err("Allocate memory for rawdata failed");
        ret = -ENOMEM;
        goto err_jitter_show_exit;
    }
    rawdata_min = (s16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata_min == NULL) {
        cts_err("Allocate memory for rawdata failed");
        ret = -ENOMEM;
        goto err_free_rawdata;
    }
    rawdata_max = (s16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata_max == NULL) {
        cts_err("Allocate memory for rawdata failed");
        ret = -ENOMEM;
        goto err_free_rawdata_min;
    }

    for (i = 0; i < RAWDATA_BUFFER_SIZE(cts_dev) / 2; i++) {
        rawdata_min[i] = 32767;
        rawdata_max[i] = -32768;
    }

    for (i = 0; i < jitter_test_frame; i++) {
        ret = cts_tcs_top_get_rawdata(cts_dev, (u8 *) rawdata,
                RAWDATA_BUFFER_SIZE(cts_dev));
        if (ret) {
            cts_err("Get raw data failed %d", ret);
            msleep(5);
            continue;
        }
        for (r = 0; r < cts_dev->hwdata->num_row; r++) {
            for (c = 0; c < cts_dev->hwdata->num_col; c++) {
                int index;

                index = r * cts_dev->hwdata->num_col + c;
                if (rawdata_min[index] > rawdata[index])
                    rawdata_min[index] = rawdata[index];
                else if (rawdata_max[index] < rawdata[index])
                    rawdata_max[index] = rawdata[index];
            }
        }
    }
    data_valid = true;

    if (data_valid) {
        max = -32768;
        min = 32767;
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val;
                int index = r * cts_dev->hwdata->num_col + c;

                val = rawdata_max[index] - rawdata_min[index];
                rawdata[index] = val;
                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += snprintf(buf + count, PAGE_SIZE - count,
                  SPLIT_LINE_STR
                  "Jitter data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d, TOTAL FRAME=%d\n"
                  SPLIT_LINE_STR
                  "   |  ", min_r, min_c, min, max_r, max_c,
                  max, average, jitter_test_frame);
        for (c = 0; c < cts_dev->fwdata.cols; c++)
            count += snprintf(buf + count, PAGE_SIZE - count, COL_NUM_FORMAT_STR, c);

        count += snprintf(buf + count, PAGE_SIZE - count, "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += snprintf(buf + count, PAGE_SIZE - count, ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++)
                count += snprintf(buf + count, PAGE_SIZE - count,
                        DATA_FORMAT_STR, rawdata[r * cts_dev->hwdata->num_col + c]);

            buf[count++] = '\n';
        }
    }

    //err_free_rawdata_max:
    kfree(rawdata_max);
err_free_rawdata_min:
    kfree(rawdata_min);
err_free_rawdata:
    kfree(rawdata);
err_jitter_show_exit:
    cts_unlock_device(cts_dev);
    return data_valid ? count : ret;
}

static DEVICE_ATTR(jitter, S_IRUSR | S_IWUSR, jitter_show, jitter_store);

static ssize_t compensate_cap_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 *cap = NULL;
    int ret;
    ssize_t count = 0;
    int r, c, min, max, max_r, max_c, min_r, min_c, sum, average;

    cts_info("Read '%s'", attr->attr.name);

    cap = kzalloc(cts_dev->hwdata->num_row * cts_dev->hwdata->num_col, GFP_KERNEL);
    if (cap == NULL)
        return scnprintf(buf, PAGE_SIZE,
                "Allocate mem for compensate cap failed\n");

    cts_lock_device(cts_dev);
    ret = cts_tcs_top_get_cnegdata(cts_dev, cap,
            cts_dev->hwdata->num_row * cts_dev->hwdata->num_col);
    cts_unlock_device(cts_dev);
    if (ret) {
        kfree(cap);
        return scnprintf(buf, PAGE_SIZE, "Get compensate cap failed %d\n", ret);
    }

    max = min = cap[0];
    sum = 0;
    max_r = max_c = min_r = min_c = 0;
    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            u16 val = cap[r * cts_dev->hwdata->num_col + c];

            sum += val;
            if (val > max) {
                max = val;
                max_r = r;
                max_c = c;
            } else if (val < min) {
                min = val;
                min_r = r;
                min_c = c;
            }
        }
    }
    average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

    count += scnprintf(buf + count, PAGE_SIZE - count,
            "--------------------------------------"\
            "--------------------------------------\n"
            " Compensatete Cap MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
            "---+----------------------------------"\
            "--------------------------------------\n"
            "   |", min_r, min_c, min, max_r, max_c, max,
            average);
    for (c = 0; c < cts_dev->fwdata.cols; c++)
        count += scnprintf(buf + count, PAGE_SIZE - count, " %3u", c);

    count += scnprintf(buf + count, PAGE_SIZE - count,
            "\n"
            "---+----------------------------------"\
            "--------------------------------------\n");

    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        count += scnprintf(buf + count, PAGE_SIZE - count, "%2u |", r);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count, " %3u",
                    cap[r * cts_dev->hwdata->num_col + c]);
        }
        buf[count++] = '\n';
    }
    count += scnprintf(buf + count, PAGE_SIZE - count,
            "---+----------------------------------"\
            "--------------------------------------\n");

    kfree(cap);

    return count;
}

static DEVICE_ATTR(compensate_cap, S_IRUGO, compensate_cap_show, NULL);

#ifdef CFG_CTS_HAS_RESET_PIN
static ssize_t reset_pin_show(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read RESET-PIN");

    return snprintf(buf, PAGE_SIZE,
            "Reset pin: %d, status: %d\n",
            cts_data->pdata->rst_gpio,
            gpio_get_value(cts_data->pdata->rst_gpio));
}

static ssize_t reset_pin_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    cts_info("Write RESET-PIN");
    cts_info("Chip staus maybe changed");

    cts_plat_set_reset(cts_dev->pdata, (buf[0] == '1') ? 1 : 0);
    return count;
}

static DEVICE_ATTR(reset_pin, S_IRUSR | S_IWUSR, reset_pin_show,
        reset_pin_store);
#endif

static ssize_t irq_pin_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read IRQ-PIN");

    return snprintf(buf, PAGE_SIZE,
            "IRQ pin: %d, status: %d\n",
            cts_data->pdata->int_gpio,
            gpio_get_value(cts_data->pdata->int_gpio));
}

static DEVICE_ATTR(irq_pin, S_IRUGO, irq_pin_show, NULL);

static ssize_t irq_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct irq_desc *desc;

    cts_info("Read IRQ-INFO");

    desc = irq_to_desc(cts_data->pdata->irq);
    if (desc == NULL) {
        return snprintf(buf, PAGE_SIZE,
                "IRQ: %d descriptor not found\n",
                cts_data->pdata->irq);
    }

    return scnprintf(buf, PAGE_SIZE,
            "IRQ num: %d, depth: %u, "
            "count: %u, unhandled: %u, last unhandled eslape: %lu, irq flags: 0x%x, int_mode: %s\n",
            cts_data->pdata->irq, desc->depth,
            desc->irq_count, desc->irqs_unhandled,
            desc->last_unhandled, desc->action->flags,
            (desc->action->flags & IRQF_TRIGGER_MASK ) == IRQF_TRIGGER_RISING ? "IRQF_TRIGGER_RISING" :
            (desc->action->flags & IRQF_TRIGGER_MASK ) == IRQF_TRIGGER_FALLING ? "IRQF_TRIGGER_FALLING" :
            (desc->action->flags & IRQF_TRIGGER_MASK ) == IRQF_TRIGGER_HIGH ? "IRQF_TRIGGER_HIGH" :
            (desc->action->flags & IRQF_TRIGGER_MASK ) == IRQF_TRIGGER_LOW ? "IRQF_TRIGGER_LOW " : "IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING");
}

static DEVICE_ATTR(irq_info, S_IRUGO, irq_info_show, NULL);

#ifdef CFG_CTS_FW_LOG_REDIRECT
static ssize_t fw_log_redirect_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    return snprintf(buf, PAGE_SIZE, "Fw log redirect is %s\n",
            cts_is_fw_log_redirect(cts_dev) ? "enable" : "disable");
}

static ssize_t fw_log_redirect_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 enable = 0;

    if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1')
        enable = 1;
    if (enable)
        cts_enable_fw_log_redirect(cts_dev);
    else
        cts_disable_fw_log_redirect(cts_dev);

    return count;
}

static DEVICE_ATTR(fw_log_redirect, S_IRUSR | S_IWUSR, fw_log_redirect_show,
        fw_log_redirect_store);
#endif

#ifndef CONFIG_CTS_I2C_HOST
static ssize_t debug_spi_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    return snprintf(buf, PAGE_SIZE, "spi_speed=%d\n",
            cts_dev->pdata->spi_speed);
}

static ssize_t debug_spi_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u16 s = 0;
    int ret = 0;
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &s);
    if (ret) {
        cts_err("Invalid spi speed: %s", argv[0]);
        return -EINVAL;
    }

    cts_dev->pdata->spi_speed = s;

    return count;
}

static DEVICE_ATTR(debug_spi, S_IRUSR | S_IWUSR, debug_spi_show,
        debug_spi_store);
#endif

#ifdef CFG_CTS_GESTURE
static ssize_t gesture_en_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    return sprintf(buf, "Gesture wakup is %s\n",
            cts_is_gesture_wakeup_enabled(cts_dev) ? "enable" : "disable");
}

static ssize_t gesture_en_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 enable = 0;

    if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1')
        enable = 1;

    if (enable)
        cts_enable_gesture_wakeup(cts_dev);
    else
        cts_disable_gesture_wakeup(cts_dev);

    return count;
}

static DEVICE_ATTR(gesture_en, S_IRUSR | S_IWUSR, gesture_en_show,
        gesture_en_store);
#endif

static ssize_t int_data_types_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%#04x\n",
            cts_data->cts_dev.fwdata.int_data_types);
}

static ssize_t int_data_types_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u16 type = 0;
    int ret = 0;

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &type);
    if (ret) {
        cts_err("Invalid int data types: %s", argv[0]);
        return -EINVAL;
    }

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_set_int_data_types(&cts_data->cts_dev, type);
    cts_unlock_device(&cts_data->cts_dev);
    if (ret)
        return -EIO;
    return count;
}

static DEVICE_ATTR(int_data_types, S_IWUSR | S_IRUGO,
        int_data_types_show, int_data_types_store);

static ssize_t int_data_method_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%d\n",
            cts_data->cts_dev.fwdata.int_data_method);
}

static ssize_t int_data_method_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u8 method = 0;
    int ret = 0;

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou8(argv[0], 0, &method);
    if (ret) {
        cts_err("Invalid int data method: %s", argv[0]);
        return -EINVAL;
    } else if (method >= INT_DATA_METHOD_CNT) {
        cts_err("Invalid int data method: %s", argv[0]);
        return -EINVAL;
    }

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_set_int_data_method(&cts_data->cts_dev, method);
    cts_unlock_device(&cts_data->cts_dev);
    if (ret)
        return -EIO;
    return count;
}

static DEVICE_ATTR(int_data_method, S_IWUSR | S_IRUGO,
        int_data_method_show, int_data_method_store);


static ssize_t cts_charger_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8  val;
    int ret;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_charger_plug(cts_dev, &val);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Get charger state failed %d(%s)", ret, cts_strerror(ret));
        return -1;
    }
    switch (val) {
        case 0:  return scnprintf(buf, PAGE_SIZE, "Detached\n");
        case 1:  return scnprintf(buf, PAGE_SIZE, "Attached\n");
        default: return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }
}

static ssize_t cts_charger_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int state;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    switch (buf[0]) {
        case '0': state = 0; break;
        case '1': state = 1; break;
        default:
            cts_err("Invalid arg for state");
            return -EINVAL;
    }

    cts_info("state = %d", state);

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_charger_plug(cts_dev, state);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Set charger state failed %d(%s)", ret, cts_strerror(ret));
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(charger_state, S_IRUGO | S_IWUSR,
        cts_charger_state_show, cts_charger_state_store);

static ssize_t cts_earjack_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8  val;
    int ret;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_earjack_plug(cts_dev, &val);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Get earjack state failed %d(%s)",
            ret, cts_strerror(ret));
        return -1;
    }
    switch (val) {
        case 0:  return scnprintf(buf, PAGE_SIZE, "Detached\n");
        case 1:  return scnprintf(buf, PAGE_SIZE, "Attached\n");
        default: return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }
}

static ssize_t cts_earjack_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int state;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    switch (buf[0]) {
        case '0': state = 0; break;
        case '1': state = 1; break;
        default:
            cts_err("Invalid arg for state");
            return -EINVAL;
    }

    cts_info("state = %d", state);

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_earjack_plug(cts_dev, state);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Set earjack state failed %d(%s)", ret, cts_strerror(ret));
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(earjack_state, S_IRUGO | S_IWUSR,
        cts_earjack_state_show, cts_earjack_state_store);

static ssize_t cts_edge_restain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 direction;
    int ret;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_panel_direction(cts_dev, &direction);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("get panel direction failed!");
        return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }

    return scnprintf(buf, PAGE_SIZE, "direction: 0x%02x\n", direction);
}

static ssize_t cts_edge_restain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 direction;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    switch (buf[0]) {
        case '0': direction = 0; break;//normal
        case '1': direction = 1; break;//notch left
        case '2': direction = 2; break;//notch right
        default:
            cts_err("Invalid arg for mode");
            return -EINVAL;
    }
    cts_info("direction = %d", direction);

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_panel_direction(cts_dev, direction);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Set edge restain failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(edge_restain, S_IWUSR | S_IRUGO,
        cts_edge_restain_show, cts_edge_restain_store);

static ssize_t cts_game_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 enabled;
    int ret;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_game_mode(cts_dev, &enabled);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("get game mode failed!");
        return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }

    return scnprintf(buf, PAGE_SIZE, "game mode: 0x%02x\n", enabled);
}

static ssize_t cts_game_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int enable = -1;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    if (count >= 1) {
        if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1') {
            enable = 1;
        } else if (buf[0] == 'N' || buf[0] == 'n' || buf[0] == '0') {
            enable = 0;
        }
    }
    if (enable == -1) {
        cts_err("Invalid arg for game mode enable");
        return -EINVAL;
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_game_mode(cts_dev, enable);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("Set game mode failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(game_mode,  S_IWUSR | S_IRUGO,
            cts_game_mode_show, cts_game_mode_store);

#ifdef CONFIG_CTS_TP_PROXIMITY
static ssize_t cts_proximity_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_firmware_status *status = (struct cts_firmware_status *)
            &cts_dev->rtdata.firmware_status;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "proximity mode: %d\n",
            status->proximity);
}

static ssize_t cts_proximity_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int enable = -1;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    if (count >= 1) {
        if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1') {
            enable = 1;
        } else if (buf[0] == 'N' || buf[0] == 'n' || buf[0] == '0') {
            enable = 0;
        }
    }
    if (enable == -1) {
        cts_err("Invalid arg for proximity mode enable");
        return -EINVAL;
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_proximity_mode(cts_dev, enable);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("Set proximity mode failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(proximity_mode,  S_IWUSR | S_IRUGO,
            cts_proximity_mode_show, cts_proximity_mode_store);
#endif


static ssize_t cts_tcs_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    return snprintf(buf, PAGE_SIZE, "chipone_cmds.bin size:%zu\n",
                cts_dev->rtdata.tcscmd_len * sizeof(u16));
}

static ssize_t cts_tcs_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct firmware *tcscmds;
    char *update = "update";
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    ret = memcmp(update, buf, count - 1);
    if (ret) {
        cts_err("Not update tcs cmd buf, %s", buf);
        return count;
    }

    ret = request_firmware(&tcscmds, "chipone_cmds.bin", dev);
    if (ret) {
        cts_err("Could not load firmware from chipone_cmds.bin: %d", ret);
        return ret;
    }

    if (tcscmds->size > PAGE_SIZE) {
        cts_err("tcscmds length is over one PAGE_SIZE");
        return count;
    }

    memcpy((u8 *)cts_dev->rtdata.tcscmd, tcscmds->data, tcscmds->size);
    cts_dev->rtdata.tcscmd_len = tcscmds->size / sizeof(u16);

    release_firmware(tcscmds);
    cts_info("update tcscmds successfully");

    return count;
}
static DEVICE_ATTR(tcs_cmd, S_IWUSR | S_IRUSR,
        cts_tcs_cmd_show, cts_tcs_cmd_store);

static u16 cts_find_tcs_cmd(struct cts_device *cts_dev, u8 class_id, u8 cmd_id)
{
    struct cts_tcs_cmd *tcmd;
    u8 cmd_buf[2];
    int i;

    for (i = 0; i < cts_dev->rtdata.tcscmd_len; i++) {
        put_unaligned_le16(cts_dev->rtdata.tcscmd[i], cmd_buf);
        tcmd = (struct cts_tcs_cmd *)cmd_buf;
        if (class_id == tcmd->class_id && cmd_id == tcmd->cmd_id) {
            cts_info("Found tcs cmd:0x%04x: { %d, %d, %d, %d, %d}",
                cts_dev->rtdata.tcscmd[i], tcmd->base_flag,
                tcmd->class_id, tcmd->cmd_id,
                tcmd->is_read, tcmd->is_write);
            return cts_dev->rtdata.tcscmd[i];
        }
    }

    return 0;
}
static ssize_t cts_read_tcs_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int count = 0;
    u8 *read_buf;
    int i;
    int ret;

    read_buf = kzalloc(cts_dev->rtdata.curr_len, GFP_KERNEL);
    if (read_buf == NULL) {
        return snprintf(buf, PAGE_SIZE, "kzalloc read_buf failed\n");
    }

    if (!(cts_dev->rtdata.curr_cmd & BIT(14))) {
        return snprintf(buf, PAGE_SIZE, "This cmd %04x is not readable!\n",
                cts_dev->rtdata.curr_cmd);
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_read(cts_dev, cts_dev->rtdata.curr_cmd, read_buf,
            cts_dev->rtdata.curr_len);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("read tcscmd: 0x%04x, failed", cts_dev->rtdata.curr_cmd);
        return ret;
    }

    count += snprintf(buf + count, PAGE_SIZE - count, "read_tcs(0x%02x): ",
            cts_dev->rtdata.curr_cmd);
    for (i = 0; i < cts_dev->rtdata.curr_len; i++) {
        count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", read_buf[i]);
    }
    count += snprintf(buf + count, PAGE_SIZE - count, "\n");
    return count;
}
static ssize_t cts_read_tcs_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 cmd;
    u8 class_id, cmd_id, read_len;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);
    parse_arg(buf, count);

    if (argc != 3) {
        cts_err("usage: echo class_id cmd_id read_len > read_tcs");
        return count;
    }

    ret = kstrtou8(argv[0], 0, &class_id);
    if (ret) {
        cts_err("kstrtou8 class_id %d failed", class_id);
        return ret;
    }

    ret = kstrtou8(argv[1], 0, &cmd_id);
    if (ret) {
        cts_err("kstrtou8 cmd_id %d failed", cmd_id);
        return ret;
    }

    ret = kstrtou8(argv[2], 0, &read_len);
    if (ret) {
        cts_err("kstrtou8 read_len %d failed", read_len);
        return ret;
    }

    cmd = cts_find_tcs_cmd(cts_dev, class_id, cmd_id);
    if (cmd == 0) {
        cts_err("Not found classid %d, cmdid %d", class_id, cmd_id);
        return count;
    }

    cts_dev->rtdata.curr_cmd = cmd;
    cts_dev->rtdata.curr_len = read_len;
    cts_info("tcs cmd:%02x", cmd);

    return count;
}
static DEVICE_ATTR(read_tcs_cmd, S_IWUSR | S_IRUSR,
        cts_read_tcs_cmd_show, cts_read_tcs_cmd_store);


static ssize_t cts_write_tcs_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 class_id, cmd_id;
    u16 cmd;
    u8 wbuf[32];
    int wlen;
    int i = 0;
    int ret;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);
    parse_arg(buf, count);

    if (argc < 3) {
        cts_err("usage: echo class_id cmd_id x1 x2 .. > write_tcs");
        return count;
    }

    ret = kstrtou8(argv[0], 0, &class_id);
    if (ret) {
        cts_err("kstrtou8 class_id %d failed", i);
        return ret;
    }

    ret = kstrtou8(argv[1], 0, &cmd_id);
    if (ret) {
        cts_err("kstrtou8 cmd_id %d failed", i);
        return ret;
    }

    wlen = argc - 2;
    if (wlen > sizeof(wbuf)) {
        cts_warn("write size is limit %zu bytes.", sizeof(wbuf));
        wlen = sizeof(wbuf);
    }

    for (i = 0; i < wlen; i++) {
        ret = kstrtou8(argv[i + 2], 0, wbuf + i);
        if (ret) {
            cts_err("kstrtou8 wbuf %d failed", i);
            return ret;
        }
    }

    cmd = cts_find_tcs_cmd(cts_dev, class_id, cmd_id);
    if (cmd == 0) {
        cts_err("Not found classid %d, cmdid %d", class_id, cmd_id);
        return count;
    }

    cts_lock_device(cts_dev);
    ret = cts_tcs_write(cts_dev, cmd, wbuf, wlen);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("write tcscmd: 0x%04x, failed", cmd);
        return ret;
    }

    return count;
}
static DEVICE_ATTR(write_tcs_cmd, S_IWUSR, NULL, cts_write_tcs_cmd_store);

static struct attribute *cts_dev_misc_atts[] = {
    &dev_attr_ic_type.attr,
    &dev_attr_program_mode.attr,
    &dev_attr_rawdata.attr,
    &dev_attr_diffdata.attr,
    &dev_attr_manualdiff.attr,
    &dev_attr_jitter.attr,
#ifdef CFG_CTS_HAS_RESET_PIN
    &dev_attr_reset_pin.attr,
#endif
    &dev_attr_irq_pin.attr,
    &dev_attr_irq_info.attr,
#ifdef CFG_CTS_FW_LOG_REDIRECT
    &dev_attr_fw_log_redirect.attr,
#endif
    &dev_attr_compensate_cap.attr,
    &dev_attr_read_tcs_reg.attr,
    &dev_attr_write_tcs_reg.attr,
    &dev_attr_read_hw_reg.attr,
    &dev_attr_write_hw_reg.attr,
#ifndef CONFIG_CTS_I2C_HOST
    &dev_attr_debug_spi.attr,
#endif
#ifdef CFG_CTS_GESTURE
    &dev_attr_gesture_en.attr,
#endif /* CFG_CTS_GESTURE */
    &dev_attr_int_data_types.attr,
    &dev_attr_int_data_method.attr,
    &dev_attr_charger_state.attr,
    &dev_attr_earjack_state.attr,
    &dev_attr_edge_restain.attr,
    &dev_attr_game_mode.attr,
#ifdef CONFIG_CTS_TP_PROXIMITY
    &dev_attr_proximity_mode.attr,
#endif
    &dev_attr_tcs_cmd.attr,
    &dev_attr_read_tcs_cmd.attr,
    &dev_attr_write_tcs_cmd.attr,
    NULL
};

static const struct attribute_group cts_dev_misc_attr_group = {
    .name = "misc",
    .attrs = cts_dev_misc_atts,
};

static const struct attribute_group *cts_dev_attr_groups[] = {
    &cts_dev_firmware_attr_group,
#ifdef CFG_CTS_FW_UPDATE_SYS
    &cts_dev_fw_up_attr_group,
#endif
    &cts_dev_flash_attr_group,
    &cts_dev_test_attr_group,
    &cts_dev_misc_attr_group,
    NULL
};

extern struct chipone_ts_data *g_cts_data;
extern int cts_suspend(struct chipone_ts_data *cts_data);
extern int cts_resume(struct chipone_ts_data *cts_data);
static ssize_t cts_suspend_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = g_cts_data;
    struct cts_device *cts_dev = &cts_data->cts_dev;
    return  snprintf(buf, PAGE_SIZE,  "%s\n",
            cts_dev->rtdata.suspended ? "suspend" : "resume");
}
static ssize_t cts_suspend_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    if (count >= 1) {
        if (buf[0] == '1') {
            cts_suspend(g_cts_data);
        } else if (buf[0] == '0') {
            cts_resume(g_cts_data);
        } else {
            cts_err("usage: echo 0/1 > ts_suspend");
        }
    }

    return count;
}
static DEVICE_ATTR(ts_suspend,  S_IWUSR | S_IRUGO,
            cts_suspend_show, cts_suspend_store);

static struct attribute *cts_dev_ts_suspend_atts[] = {
    &dev_attr_ts_suspend.attr,
    NULL
};

static struct attribute_group ts_suspend_attr_group = {
    .attrs = cts_dev_ts_suspend_atts,
};


#include <linux/major.h>
#include <linux/kdev_t.h>

static char *ts_mmi_kobject_get_path(struct kobject *kobj, gfp_t gfp_mask)
{
	char *path;
	int len = 1;
	struct kobject *parent = kobj;

	do {
		if (parent->name == NULL) {
			len = 0;
			break;
		}
		len += strlen(parent->name) + 1;
		parent = parent->parent;
	} while (parent);

	if (len == 0)
		return NULL;

	path = kzalloc(len, gfp_mask);
	if (!path)
		return NULL;

	--len;
	for (parent = kobj; parent; parent = parent->parent) {
		int cur = strlen(parent->name);
		len -= cur;
		memcpy(path + len, parent->name, cur);
		*(path + --len) = '/';
	}
	pr_debug("kobject: '%s' (%p): %s: path = '%s'\n", kobj->name,
		kobj, __func__, path);

	return path;
}

/* Attribute: path (RO) */
static ssize_t path_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *data = dev_get_drvdata(dev);
    ssize_t blen;
    const char *path;

    if (!data) {
        cts_err("Read 'path' with chipone_ts_data NULL");
        return (ssize_t) 0;
    }

#ifdef CONFIG_CTS_I2C_HOST
	path = ts_mmi_kobject_get_path(&data->i2c_client->dev.kobj, GFP_KERNEL);
#else
	path = ts_mmi_kobject_get_path(&data->spi_client->dev.kobj, GFP_KERNEL);
#endif
    blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
    kfree(path);
    return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "chipone");
}

/* Attribute: vendor (RO) */
static ssize_t ic_ver_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *ts = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%04x\n%s%04x\n",
             "Product ID: ", ts->cts_dev.hwdata->name,
             "Build ID: ", ts->cts_dev.fwdata.version
             ? ts->cts_dev.fwdata.version
             : ts->pdata->build_id,
             "Config ID: ", ts->cts_dev.fwdata.ddi_version
             ? ts->cts_dev.fwdata.ddi_version
             : ts->pdata->config_id);
}

static struct device_attribute touchscreen_attributes[] = {
    __ATTR_RO(path),
    __ATTR_RO(vendor),
    __ATTR_RO(ic_ver),
    __ATTR_NULL
};

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

/*******************************************************
 *Description:
 *    Chipone touchscreen FW function class. file node
 *    initial function.
 *
 * return:
 *    Executive outcomes. 0---succeed. -1---failed.
 *******************************************************/
static int cts_fw_class_init(void *_data, bool create)
{
    struct chipone_ts_data *data = _data;
    struct device_attribute *attrs = touchscreen_attributes;
    int i, error = 0;
    static struct class *touchscreen_class;
    static struct device *ts_class_dev;
    dev_t devno;
    const char* node_name;

    cts_info("%s touchscreen class files", create ? "Add" : "Remove");

    if (create) {
#ifdef CFG_CTS_CHIP_PRIMARY
        node_name = CFG_CTS_CHIP_PRIMARY;
#else
        if (data->cts_dev.hwdata->name != NULL)
            node_name = data->cts_dev.hwdata->name;
        else
            node_name = CFG_CTS_CHIP_NAME;
#endif

        error = alloc_chrdev_region(&devno, 0, 1, node_name);

        if (error) {
            cts_info("Alloc input devno failed %d", error);
            return error;
        }

        cts_info("Create class 'touchscreen'");
        touchscreen_class = class_create(THIS_MODULE, "touchscreen");
        if (IS_ERR(touchscreen_class)) {
            cts_err("Create class 'touchscreen' failed %ld",
                PTR_ERR(touchscreen_class));
            error = PTR_ERR(touchscreen_class);
            touchscreen_class = NULL;
            return error;
        }

        ts_class_dev = device_create(touchscreen_class, NULL,
                devno, data, "%s", node_name);
        cts_info("Create device for %s", node_name);

        if (IS_ERR(ts_class_dev)) {
            cts_err("Create device %s failed %ld", node_name, PTR_ERR(ts_class_dev));
            error = PTR_ERR(ts_class_dev);
            ts_class_dev = NULL;
            return error;
        }

        cts_info("Create attr files");
        for (i = 0; attrs[i].attr.name != NULL; ++i) {
            cts_info("  Create attr file '%s'", attrs[i].attr.name);
            error = device_create_file(ts_class_dev, &attrs[i]);
            if (error) {
                cts_err("Create attr file '%s' failed %d",
                    attrs[i].attr.name, error);
                break;
            }
        }

        if (error)
            goto device_destroy;
        else
            cts_info("Create /sys/class/touchscreen/ Succeeded");
    } else {
        if (!touchscreen_class || !ts_class_dev)
            return -ENODEV;

        for (i = 0; attrs[i].attr.name != NULL; ++i) {
            cts_info("Remove device file '%s'", attrs[i].attr.name);
            device_remove_file(ts_class_dev, &attrs[i]);
        }
        device_unregister(ts_class_dev);
        class_unregister(touchscreen_class);
    }

    return 0;

device_destroy:
    for (--i; i >= 0; --i)
        device_remove_file(ts_class_dev, &attrs[i]);
    device_destroy(touchscreen_class, devno);
    ts_class_dev = NULL;
    class_unregister(touchscreen_class);
    cts_err("Creating touchscreen class failed %d", error);

    return -ENODEV;
}

int cts_sysfs_add_device(struct device *dev)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    int ret, i;

    cts_info("Add device attr groups");

    /*Low version kernel NOT support sysfs_create_groups() */
    for (i = 0; cts_dev_attr_groups[i]; i++) {
        ret = sysfs_create_group(&dev->kobj, cts_dev_attr_groups[i]);
        if (ret) {
            while (--i >= 0)
                sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);
            break;
        }
    }

    if (ret) {
        cts_err("Add device attr failed %d", ret);
        return ret;
    }

    ret = sysfs_create_link(NULL, &dev->kobj, "chipone-tddi");
    if (ret)
        cts_err("Create sysfs link error:%d", ret);

    ret = cts_fw_class_init(cts_data, true);
    if (ret) {
        cts_err("Create touchscreen class failed. ret=%d", ret);
        return ret;
    }

    /* Only for sprd platform  */
    cts_data->suspend_kobj = kobject_create_and_add("touchscreen", NULL);
    if (cts_data->suspend_kobj == NULL) {
        cts_err("Create touchscreen failed");
        return 0;
    }

    ret = sysfs_create_group(cts_data->suspend_kobj, &ts_suspend_attr_group);
    if (ret) {
        kobject_put(cts_data->suspend_kobj);
        cts_info("Create ts_suspend failed");
        return 0;
    }
    cts_info("Create ts_suspend successfully");

    return 0;
}

void cts_sysfs_remove_device(struct device *dev)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    int i;

    cts_info("Remove device attr groups");

    sysfs_remove_link(NULL, "chipone-tddi");
    /*Low version kernel NOT support sysfs_remove_groups() */
    for (i = 0; cts_dev_attr_groups[i]; i++)
        sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);

    cts_fw_class_init(cts_data, false);

    sysfs_remove_group(cts_data->suspend_kobj, &ts_suspend_attr_group);
    kobject_put(cts_data->suspend_kobj);
}

#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR

#endif /* CONFIG_CTS_SYSFS */
