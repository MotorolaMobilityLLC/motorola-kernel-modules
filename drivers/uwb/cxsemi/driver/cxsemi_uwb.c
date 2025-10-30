/*
 * Copyright (C) 2025 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "cxsemi_uwb.h"
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>

/* Macro definitions and constants */
#define CXSEMI_UWB_UCI_MSG_HDR_SIZE    4
#define CXSEMI_UWB_SPI_BUFF_SIZE       4096
#define CXSEMI_UWB_MAX_RETRY_COUNT     3
#define CXSEMI_UWB_RETRY_DELAY_MS      10
#define CXSEMI_UWB_RESET_DELAY_MS      2

/* Debugging flags */
#define CXSEMI_UWB_DEBUG_SPI           0x01
#define CXSEMI_UWB_DEBUG_IRQ           0x02
#define CXSEMI_UWB_DEBUG_STATE         0x04

static int cxsemi_uwb_debug_mask = 0;
module_param_named(debug_mask, cxsemi_uwb_debug_mask, int, 0644);
MODULE_PARM_DESC(debug_mask, "Debug mask for UWB driver (SPI=1, IRQ=2, STATE=4)");

static int cxsemi_uwb_delay = 0;
module_param_named(delay, cxsemi_uwb_delay, int, 0644);
MODULE_PARM_DESC(delay, "SPI delay");

/* Logging macro definitions with debug levels */
#define CXSEMI_UWB_LOG_INFO(dev, ...)   dev_info((dev), __VA_ARGS__)
#define CXSEMI_UWB_LOG_WARN(dev, ...)   dev_warn((dev), __VA_ARGS__)
#define CXSEMI_UWB_LOG_ERROR(dev, ...)  dev_err((dev), __VA_ARGS__)

#define CXSEMI_UWB_LOG_DEBUG(dev, ...) \
    do { \
        if (cxsemi_uwb_debug_mask & CXSEMI_UWB_DEBUG_STATE) \
            dev_info((dev), "DEBUG: " __VA_ARGS__); \
    } while (0)

#define CXSEMI_UWB_LOG_DEBUG_SPI(dev, ...) \
    do { \
        if (cxsemi_uwb_debug_mask & CXSEMI_UWB_DEBUG_SPI) \
            dev_info((dev), "SPI: " __VA_ARGS__); \
    } while (0)

#define CXSEMI_UWB_LOG_DEBUG_IRQ(dev, ...) \
    do { \
        if (cxsemi_uwb_debug_mask & CXSEMI_UWB_DEBUG_IRQ) \
            dev_info((dev), "IRQ: " __VA_ARGS__); \
    } while (0)

/**
 * cxsemi_uwb_set_device_state - Safely set device state with memory barrier
 * @uwb_dev: UWB device pointer
 * @state: New device state
 */
static void cxsemi_uwb_set_device_state(struct cxsemi_uwb_device *uwb_dev,
                                       enum cxsemi_uwb_device_state state)
{
    atomic_set(&uwb_dev->device_state, state);
    smp_mb(); /* Ensure state is visible to other CPUs */
}

/**
 * cxsemi_uwb_get_device_state - Safely get device state
 * @uwb_dev: UWB device pointer
 *
 * Return: Current device state
 */
static enum cxsemi_uwb_device_state cxsemi_uwb_get_device_state(
    struct cxsemi_uwb_device *uwb_dev)
{
    smp_rmb(); /* Ensure we read the latest state */
    return atomic_read(&uwb_dev->device_state);
}

/**
 * cxsemi_uwb_validate_device - Comprehensive device state validation
 * @uwb_dev: UWB device pointer
 *
 * Return: 0 if device is valid and ready, negative error code otherwise
 */
static int cxsemi_uwb_validate_device(struct cxsemi_uwb_device *uwb_dev)
{
    enum cxsemi_uwb_device_state state;

    if (!uwb_dev) {
        pr_err_ratelimited("UWB device pointer is NULL\n");
        return -ENODEV;
    }

    if (!uwb_dev->device_initialized) {
        pr_err_ratelimited("UWB device not properly initialized\n");
        return -ENODEV;
    }

    if (!uwb_dev->spi_dev) {
        pr_err_ratelimited("SPI device not attached\n");
        return -ENODEV;
    }

    state = cxsemi_uwb_get_device_state(uwb_dev);
    switch (state) {
        case CXSEMI_UWB_STATE_READY:
            return 0;
        case CXSEMI_UWB_STATE_SUSPENDED:
            return -EAGAIN;
        case CXSEMI_UWB_STATE_ERROR:
            return -EIO;
        case CXSEMI_UWB_STATE_UNINITIALIZED:
        default:
            return -ENODEV;
    }
}

#ifdef CONFIG_OF
/**
 * cxsemi_uwb_parse_spi_setting - Parse SPI configuration from device tree
 * @node: Device tree node
 * @board_data: Board data structure to populate
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_parse_spi_setting(struct device_node *node,
                                       struct cxsemi_uwb_board_data *board_data)
{
    int ret;
    u32 value;
    struct cxsemi_uwb_spi_setting *spi_setting;

    if (!node || !board_data) {
        pr_err("Invalid parameters for SPI parsing\n");
        return -EINVAL;
    }

    spi_setting = &board_data->spi_setting;

    /* Parse maximum SPI frequency */
    ret = of_property_read_u32(node, "spi-max-frequency", &value);
    if (ret) {
        pr_err("Failed to read spi-max-frequency: %d\n", ret);
        return ret;
    }

    /* Validate frequency range */
    if (value < 1000000 || value > 50000000) {
        pr_err("Invalid SPI frequency: %u Hz (valid range: 1-50 MHz)\n", value);
        return -EINVAL;
    }

    spi_setting->spi_max_speed = value;

    return 0;
}

/**
 * cxsemi_uwb_parse_dt - Parse complete board configuration from device tree
 * @spi: SPI device
 * @board_data: Board data structure to populate
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_parse_dt(struct spi_device *spi,
                              struct cxsemi_uwb_board_data *board_data)
{
    struct device_node *node = spi->dev.of_node;
    int ret;

    if (!board_data) {
        dev_err(&spi->dev, "Invalid board data pointer\n");
        return -EINVAL;
    }

    if (!node) {
        dev_err(&spi->dev, "No device tree node available\n");
        return -ENODEV;
    }

    /* Parse SPI configuration properties */
    ret = cxsemi_uwb_parse_spi_setting(node, board_data);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to parse SPI configuration: %d\n", ret);
        return ret;
    }

    /* Parse reset GPIO property */
    board_data->rst_gpio = of_get_named_gpio(node, "cxsemi,rstn-gpio", 0);
    if (!gpio_is_valid(board_data->rst_gpio)) {
        dev_err(&spi->dev, "Invalid reset GPIO: %d\n", board_data->rst_gpio);
        return -EINVAL;
    }
    CXSEMI_UWB_LOG_DEBUG(&spi->dev, "Reset GPIO configured: %d\n", board_data->rst_gpio);

    /* Parse interrupt GPIO property */
    board_data->irq_gpio = of_get_named_gpio(node, "cxsemi,irq-gpio", 0);
    if (!gpio_is_valid(board_data->irq_gpio)) {
        dev_err(&spi->dev, "Invalid interrupt GPIO: %d\n", board_data->irq_gpio);
        return -EINVAL;
    }
    CXSEMI_UWB_LOG_DEBUG(&spi->dev, "Interrupt GPIO configured: %d\n", board_data->irq_gpio);

    /* Parse interrupt trigger flags */
    ret = of_property_read_u32(node, "cxsemi,irq-flags", &board_data->irq_flags);
    if (ret) {
        /* Use falling edge as default if not specified */
        board_data->irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        dev_warn(&spi->dev, "Using default interrupt flags: 0x%x\n",
                board_data->irq_flags);
    }

    return 0;
}
#endif

/**
 * cxsemi_uwb_handler - UWB device interrupt service routine
 * @irq: Interrupt number
 * @dev_id: Device identifier
 *
 * Return: IRQ_HANDLED if interrupt was processed, IRQ_NONE otherwise
 */
static irqreturn_t cxsemi_uwb_handler(int irq, void *dev_id)
{
    struct cxsemi_uwb_device *uwb_dev = dev_id;
    enum cxsemi_uwb_device_state state;

    if (!uwb_dev) {
        pr_err_ratelimited("Invalid device context in interrupt handler\n");
        return IRQ_NONE;
    }

    /* Verify this is our interrupt and device is ready */
    if (irq != uwb_dev->board_data.irq) {
        return IRQ_NONE;
    }

    state = cxsemi_uwb_get_device_state(uwb_dev);
    if (state != CXSEMI_UWB_STATE_READY) {
        CXSEMI_UWB_LOG_DEBUG_IRQ(&uwb_dev->spi_dev->dev,
            "Interrupt received in state %d, ignoring\n", state);
        return IRQ_NONE;
    }

    atomic_long_inc(&uwb_dev->irq_count);
    atomic_set(&uwb_dev->irq_occurred, 1);

    /* Memory barrier to ensure flag is set before waking waiters */
    smp_wmb();

    wake_up_interruptible(&uwb_dev->read_wait);

    CXSEMI_UWB_LOG_DEBUG_IRQ(&uwb_dev->spi_dev->dev,
        "Interrupt handled, total IRQs: %ld\n",
        atomic_long_read(&uwb_dev->irq_count));

    return IRQ_HANDLED;
}

/**
 * cxsemi_uwb_enable_irq - Safely enable interrupt
 * @uwb_dev: UWB device pointer
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_enable_irq(struct cxsemi_uwb_device *uwb_dev)
{
    int ret = 0;

    if (!uwb_dev->irq_enabled && uwb_dev->gpio_configured) {
        enable_irq(uwb_dev->board_data.irq);
        uwb_dev->irq_enabled = true;
        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev, "IRQ enabled\n");
    }

    return ret;
}

/**
 * cxsemi_uwb_disable_irq - Safely disable interrupt
 * @uwb_dev: UWB device pointer
 */
static void cxsemi_uwb_disable_irq(struct cxsemi_uwb_device *uwb_dev)
{
    if (uwb_dev->irq_enabled && uwb_dev->gpio_configured) {
        disable_irq(uwb_dev->board_data.irq);
        uwb_dev->irq_enabled = false;
        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev, "IRQ disabled\n");
    }
}

/**
 * cxsemi_uwb_open - Device open operation
 * @inode: Inode structure
 * @file: File structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_open(struct inode *inode, struct file *file)
{
    struct cxsemi_uwb_device *uwb_dev = container_of(file->private_data,
            struct cxsemi_uwb_device, miscdev);
    int ret;

    if (!uwb_dev) {
        pr_err("Failed to get device context in open operation\n");
        return -ENODEV;
    }

    /* Validate device state before allowing open */
    ret = cxsemi_uwb_validate_device(uwb_dev);
    if (ret) {
        return ret;
    }

    /* Enable IRQ when device is opened */
    ret = cxsemi_uwb_enable_irq(uwb_dev);
    if (ret) {
        return ret;
    }

    file->private_data = uwb_dev;
    CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
        "UWB device opened successfully\n");
    return 0;
}

/**
 * cxsemi_uwb_release - Device release operation
 * @inode: Inode structure
 * @file: File structure
 *
 * Return: 0 on success
 */
static int cxsemi_uwb_release(struct inode *inode, struct file *file)
{
    struct cxsemi_uwb_device *uwb_dev = file->private_data;

    if (uwb_dev) {
        /* Disable IRQ when device is closed */
        cxsemi_uwb_disable_irq(uwb_dev);

        /* Clear any pending interrupt flag */
        atomic_set(&uwb_dev->irq_occurred, 0);

        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
            "UWB device released\n");
    }

    return 0;
}

/**
 * cxsemi_uwb_spi_transfer - Enhanced SPI transfer with timeout and retry
 * @uwb_dev: UWB device pointer
 * @tx_buf: Transmit buffer
 * @rx_buf: Receive buffer
 * @len: Transfer length
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_spi_transfer(struct cxsemi_uwb_device *uwb_dev,
                                  void *tx_buf, void *rx_buf, size_t len)
{
    struct spi_message message;
    int ret, retry;
    struct spi_transfer transfer;
    spi_message_init(&message);

    transfer.tx_buf = tx_buf;
    transfer.rx_buf = rx_buf;
    transfer.len = len;
    transfer.cs_change = 0;
    spi_message_add_tail(&transfer, &message);

    for (retry = 0; retry < CXSEMI_UWB_MAX_RETRY_COUNT; retry++) {
        /* Use mutex to protect SPI bus access */
        if (mutex_lock_interruptible(&uwb_dev->spi_mutex)) {
            return -ERESTARTSYS;
        }

        ret = spi_sync(uwb_dev->spi_dev, &message);

        mutex_unlock(&uwb_dev->spi_mutex);

        if (ret == 0) {
            CXSEMI_UWB_LOG_DEBUG_SPI(&uwb_dev->spi_dev->dev,
                "SPI transfer successful, %zu bytes\n", len);
            return 0;
        }

        atomic_long_inc(&uwb_dev->spi_transfer_errors);
        CXSEMI_UWB_LOG_WARN(&uwb_dev->spi_dev->dev,
            "SPI transfer failed (attempt %d/%d): %d\n",
            retry + 1, CXSEMI_UWB_MAX_RETRY_COUNT, ret);

        if (retry < CXSEMI_UWB_MAX_RETRY_COUNT - 1) {
            msleep(CXSEMI_UWB_RETRY_DELAY_MS);
        }
    }

    CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
        "All SPI transfer attempts failed: %d\n", ret);
    return ret;
}

/**
 * Perform a dual SPI transfer with two data buffers in a single transaction
 *
 * This function executes two consecutive SPI transfers without deasserting
 * chip select between them, ensuring continuous CS signal throughout the
 * entire transaction. An optional delay can be added after the first transfer.
 *
 * @uwb_dev: Pointer to the UWB device structure
 * @tx_buf1: First transmit buffer (typically header data)
 * @len1: Length of first transmit buffer in bytes
 * @tx_buf2: Second transmit buffer (typically payload data)
 * @len2: Length of second transmit buffer in bytes
 * @cs_change_delay_value: Delay in microseconds after first transfer.
 *                               Set to 0 for no delay.
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_spi_transfer_dual(struct cxsemi_uwb_device *uwb_dev,
                                       void *tx_buf1, size_t len1,
                                       void *tx_buf2, size_t len2,
                                       u16 cs_change_delay_value)
{
    struct spi_message message;
    struct spi_transfer transfer[2] = {0};
    int ret, retry;

    spi_message_init(&message);

    /* Configure first transfer (typically header) */
    transfer[0].tx_buf = tx_buf1;
    transfer[0].rx_buf = NULL;
    transfer[0].len = len1;
    transfer[0].cs_change = 0; /* Keep CS asserted after this transfer */
    if (cs_change_delay_value > 0) {
        transfer[0].word_delay.value = cs_change_delay_value;
        transfer[0].word_delay.unit = SPI_DELAY_UNIT_USECS;
    }
    spi_message_add_tail(&transfer[0], &message);

    /* Configure second transfer (typically payload) */
    transfer[1].tx_buf = tx_buf2;
    transfer[1].rx_buf = NULL;
    transfer[1].len = len2;
    transfer[1].cs_change = 0; /* Keep CS asserted, will be deasserted after message */
    spi_message_add_tail(&transfer[1], &message);

    for (retry = 0; retry < CXSEMI_UWB_MAX_RETRY_COUNT; retry++) {
        if (mutex_lock_interruptible(&uwb_dev->spi_mutex)) {
            return -ERESTARTSYS;
        }

        ret = spi_sync(uwb_dev->spi_dev, &message);
        mutex_unlock(&uwb_dev->spi_mutex);

        if (ret == 0) {
            CXSEMI_UWB_LOG_DEBUG_SPI(&uwb_dev->spi_dev->dev,
                "Dual SPI transfer successful, %zu + %zu bytes\n", len1, len2);
            return 0;
        }

        atomic_long_inc(&uwb_dev->spi_transfer_errors);
        CXSEMI_UWB_LOG_WARN(&uwb_dev->spi_dev->dev,
            "Dual SPI transfer failed (attempt %d/%d): %d\n",
            retry + 1, CXSEMI_UWB_MAX_RETRY_COUNT, ret);

        if (retry < CXSEMI_UWB_MAX_RETRY_COUNT - 1) {
            msleep(CXSEMI_UWB_RETRY_DELAY_MS);
        }
    }

    CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
        "All dual SPI transfer attempts failed: %d\n", ret);
    return ret;
}

/**
 * cxsemi_uwb_read_single_transfer
 */
static ssize_t cxsemi_uwb_read_single_transfer(struct cxsemi_uwb_device *uwb_dev,
                                              char __user *buf, size_t count)
{
    int ret;
    ssize_t bytes_read = 0;
    u16 payload_length;

    ret = cxsemi_uwb_spi_transfer(uwb_dev, NULL, uwb_dev->rx_buff, count);
    if (ret) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Single transfer read failed: %d\n", ret);
        return ret;
    }

    if (count >= CXSEMI_UWB_UCI_MSG_HDR_SIZE) {
        payload_length = (uwb_dev->rx_buff[2] << 8) | uwb_dev->rx_buff[3];
        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
            "UCI read message header: %*ph\n",
            CXSEMI_UWB_UCI_MSG_HDR_SIZE, uwb_dev->rx_buff);
#if 0
        if (count != (CXSEMI_UWB_UCI_MSG_HDR_SIZE + payload_length)) {
            CXSEMI_UWB_LOG_INFO(&uwb_dev->spi_dev->dev,
                "Length mismatch: header claims %u, actual read %zu\n",
                CXSEMI_UWB_UCI_MSG_HDR_SIZE + payload_length, count);
        }
#endif
    }

    if (copy_to_user(buf, uwb_dev->rx_buff, count)) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Failed to copy data to userspace in single transfer mode\n");
        return -EFAULT;
    }

    bytes_read = count;
    CXSEMI_UWB_LOG_DEBUG_SPI(&uwb_dev->spi_dev->dev,
        "Single transfer read completed: %zd bytes\n", bytes_read);

    return bytes_read;
}

/**
 * cxsemi_uwb_read_dual_transfer
 */
static ssize_t cxsemi_uwb_read_dual_transfer(struct cxsemi_uwb_device *uwb_dev,
                                            char __user *buf, size_t count)
{
    u16 payload_length;
    ssize_t bytes_to_read;
    int payload_read;
    int ret;

    ret = cxsemi_uwb_spi_transfer(uwb_dev, NULL,
                                 uwb_dev->rx_buff, CXSEMI_UWB_UCI_MSG_HDR_SIZE);
    if (ret) {
        return ret;
    }

    CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
        "UCI read message header: %*ph\n",
        CXSEMI_UWB_UCI_MSG_HDR_SIZE, uwb_dev->rx_buff);

    payload_length = (uwb_dev->rx_buff[2] << 8) | uwb_dev->rx_buff[3];

    if (payload_length > (CXSEMI_UWB_SPI_BUFF_SIZE - CXSEMI_UWB_UCI_MSG_HDR_SIZE)) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Invalid payload length: %u\n", payload_length);
        return -EIO;
    }

    bytes_to_read = CXSEMI_UWB_UCI_MSG_HDR_SIZE + payload_length;
    if (bytes_to_read > count) {
        bytes_to_read = count;
        CXSEMI_UWB_LOG_WARN(&uwb_dev->spi_dev->dev,
            "Response truncated to %zd bytes\n", bytes_to_read);
    }

    if (payload_length > 0 && bytes_to_read > CXSEMI_UWB_UCI_MSG_HDR_SIZE) {
        payload_read = bytes_to_read - CXSEMI_UWB_UCI_MSG_HDR_SIZE;

        ret = cxsemi_uwb_spi_transfer(uwb_dev, NULL,
                                     uwb_dev->rx_buff + CXSEMI_UWB_UCI_MSG_HDR_SIZE,
                                     payload_read);
        if (ret) {
            return ret;
        }

        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
            "UCI read message body: %*ph\n",
            payload_read, (uwb_dev->rx_buff + CXSEMI_UWB_UCI_MSG_HDR_SIZE));
    }

    if (copy_to_user(buf, uwb_dev->rx_buff, bytes_to_read)) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Failed to copy data to userspace\n");
        return -EFAULT;
    }

    CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
        "Dual transfer read completed: %zd bytes\n", bytes_to_read);

    return bytes_to_read;
}

/**
 * cxsemi_uwb_read
 * @file: File structure
 * @buf: User space buffer
 * @count: Number of bytes to read
 * @ppos: File position pointer
 *
 * Return: Number of bytes read on success, negative error code on failure
 */
static ssize_t cxsemi_uwb_read(struct file *file, char __user *buf,
                              size_t count, loff_t *ppos)
{
    struct cxsemi_uwb_device *uwb_dev = file->private_data;
    bool single_read_mode;
    int ret;
    ssize_t bytes_read;

    /* Validate device state */
    ret = cxsemi_uwb_validate_device(uwb_dev);
    if (ret) {
        return ret;
    }

    /* Parameter validation */
    if (!buf) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Invalid user buffer pointer\n");
        return -EINVAL;
    }

    if (count == 0) {
        return 0;
    }

    /* Buffer size constraint check */
    if (count > CXSEMI_UWB_SPI_BUFF_SIZE) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Read request exceeds buffer size: %zu > %d\n",
            count, CXSEMI_UWB_SPI_BUFF_SIZE);
        return -EINVAL;
    }

    if (mutex_lock_interruptible(&uwb_dev->mutex)) {
        return -ERESTARTSYS;
    }

    /* Wait for interrupt if no data available */
    if (!atomic_read(&uwb_dev->irq_occurred)) {
        mutex_unlock(&uwb_dev->mutex);

        if (file->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        ret = wait_event_interruptible(uwb_dev->read_wait,
                atomic_read(&uwb_dev->irq_occurred));
        if (ret) {
            return ret;
        }

        if (mutex_lock_interruptible(&uwb_dev->mutex)) {
            return -ERESTARTSYS;
        }
    }

    mutex_lock(&uwb_dev->read_mode_mutex);
    single_read_mode = uwb_dev->single_read_mode;
    mutex_unlock(&uwb_dev->read_mode_mutex);

    if (single_read_mode) {
        bytes_read = cxsemi_uwb_read_single_transfer(uwb_dev, buf, count);
    } else {
        bytes_read = cxsemi_uwb_read_dual_transfer(uwb_dev, buf, count);
    }

    if (bytes_read > 0) {
        atomic_set(&uwb_dev->irq_occurred, 0);
    }

    mutex_unlock(&uwb_dev->mutex);
    return bytes_read;
}

/**
 * cxsemi_uwb_write - Enhanced device write operation
 * @file: File structure
 * @buf: User space buffer containing data to write
 * @count: Number of bytes to write
 * @ppos: File position pointer
 *
 * Return: Number of bytes written on success, negative error code on failure
 */
static ssize_t cxsemi_uwb_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *ppos)
{
    struct cxsemi_uwb_device *uwb_dev = file->private_data;
    u16 payload_length;
    ssize_t bytes_written = 0;
    int ret;

    /* Validate device state */
    ret = cxsemi_uwb_validate_device(uwb_dev);
    if (ret) {
        return ret;
    }

    /* Parameter validation */
    if (!buf) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Invalid user buffer pointer\n");
        return -EINVAL;
    }

    if (count == 0) {
        return 0;
    }

    if (count > CXSEMI_UWB_SPI_BUFF_SIZE) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Write request exceeds buffer size: %zu > %d\n",
            count, CXSEMI_UWB_SPI_BUFF_SIZE);
        return -EINVAL;
    }

    if (count < CXSEMI_UWB_UCI_MSG_HDR_SIZE) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Write request too small: %zu < %d\n",
            count, CXSEMI_UWB_UCI_MSG_HDR_SIZE);
        return -EINVAL;
    }

    if (mutex_lock_interruptible(&uwb_dev->mutex)) {
        return -ERESTARTSYS;
    }

    /* Copy data from user space */
    if (copy_from_user(uwb_dev->tx_buff, buf, count)) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Failed to copy data from userspace\n");
        ret = -EFAULT;
        goto cleanup;
    }

    payload_length = (uwb_dev->tx_buff[2] << 8) | uwb_dev->tx_buff[3];
    if (payload_length != (count - CXSEMI_UWB_UCI_MSG_HDR_SIZE)) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Payload length mismatch: header claims %u, actual %zu\n",
            payload_length, count - CXSEMI_UWB_UCI_MSG_HDR_SIZE);
        ret = -EINVAL;
        goto cleanup;
    }

    if (payload_length > 0) {
        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev, "UCI write message header: %*ph\n",
                CXSEMI_UWB_UCI_MSG_HDR_SIZE, uwb_dev->tx_buff);
        CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev, "UCI write message body: %*ph\n",
                payload_length, (uwb_dev->tx_buff+CXSEMI_UWB_UCI_MSG_HDR_SIZE));
        ret = cxsemi_uwb_spi_transfer_dual(uwb_dev,
                                          uwb_dev->tx_buff, CXSEMI_UWB_UCI_MSG_HDR_SIZE,
                                          uwb_dev->tx_buff + CXSEMI_UWB_UCI_MSG_HDR_SIZE,
                                          payload_length, cxsemi_uwb_delay);
    }

    if (ret) {
        CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
            "Failed to write message: %d\n", ret);
        goto cleanup;
    }

    bytes_written = count;
    CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
        "Successfully wrote %zd bytes (header: %d, payload: %u)\n",
        bytes_written, CXSEMI_UWB_UCI_MSG_HDR_SIZE, payload_length);

    ret = bytes_written;

cleanup:
    mutex_unlock(&uwb_dev->mutex);
    return ret;
}

/**
 * cxsemi_uwb_ioctl - Enhanced device IOCTL operation
 * @file: File structure
 * @cmd: IOCTL command
 * @arg: Command argument
 *
 * Return: 0 on success, negative error code on failure
 */
static long cxsemi_uwb_ioctl(struct file *file, unsigned int cmd,
                            unsigned long arg)
{
    struct cxsemi_uwb_device *uwb_dev = file->private_data;
    int ret = 0;
    int state;

    /* Validate device state */
    ret = cxsemi_uwb_validate_device(uwb_dev);
    if (ret) {
        return ret;
    }

    if (mutex_lock_interruptible(&uwb_dev->mutex)) {
        return -ERESTARTSYS;
    }

    switch (cmd) {
        case CXSEMI_UWB_DISABLE:
            if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
                gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 0);
                cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_SUSPENDED);
                CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
                    "UWB device disabled\n");
            }
            break;

        case CXSEMI_UWB_ENABLE:
            if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
                gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 1);
                msleep(CXSEMI_UWB_RESET_DELAY_MS);
                cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_READY);
                CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
                    "UWB device enabled\n");
            }
            break;

        case CXSEMI_UWB_RESET:
            if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
                gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 0);
                msleep(1);
                gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 1);
                msleep(CXSEMI_UWB_RESET_DELAY_MS);
                cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_READY);
                CXSEMI_UWB_LOG_DEBUG(&uwb_dev->spi_dev->dev,
                    "UWB device reset completed\n");
            }
            break;

        case CXSEMI_UWB_GET_STATE:
            state = cxsemi_uwb_get_device_state(uwb_dev);
            if (copy_to_user((void __user *)arg, &state, sizeof(state))) {
                ret = -EFAULT;
            }
            break;

        default:
            ret = -ENOTTY;
            CXSEMI_UWB_LOG_ERROR(&uwb_dev->spi_dev->dev,
                "Invalid IOCTL command: 0x%x\n", cmd);
            break;
    }

    mutex_unlock(&uwb_dev->mutex);
    return ret;
}

/**
 * cxsemi_uwb_poll - Device poll operation
 * @file: File structure
 * @wait: Poll table
 *
 * Return: Poll mask indicating available operations
 */
static unsigned int cxsemi_uwb_poll(struct file *file, poll_table *wait)
{
    struct cxsemi_uwb_device *uwb_dev = file->private_data;
    unsigned int mask = 0;

    if (!uwb_dev) {
        return POLLERR;
    }

    poll_wait(file, &uwb_dev->read_wait, wait);

    if (atomic_read(&uwb_dev->irq_occurred)) {
        mask |= POLLIN | POLLRDNORM;
    }

    /* Device is always ready for writing in non-blocking mode */
    mask |= POLLOUT | POLLWRNORM;

    return mask;
}

/* Character device file operations structure */
static const struct file_operations cxsemi_uwb_fops = {
    .owner          = THIS_MODULE,
    .open           = cxsemi_uwb_open,
    .release        = cxsemi_uwb_release,
    .read           = cxsemi_uwb_read,
    .write          = cxsemi_uwb_write,
    .unlocked_ioctl = cxsemi_uwb_ioctl,
    .compat_ioctl   = compat_ptr_ioctl,
    .poll           = cxsemi_uwb_poll,
};

/**
 * cxsemi_uwb_gpio_init - Initialize GPIO pins for UWB device
 * @uwb_dev: UWB device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_gpio_init(struct cxsemi_uwb_device *uwb_dev)
{
    struct spi_device *spi = uwb_dev->spi_dev;
    int ret;

    /* Initialize reset GPIO */
    ret = devm_gpio_request_one(&spi->dev,
                              uwb_dev->board_data.rst_gpio,
                              GPIOF_OUT_INIT_LOW,
                              CXSEMI_UWB_RST_GPIO_NAME);
    if (ret) {
        dev_err(&spi->dev, "Failed to request reset GPIO %d: %d\n",
                uwb_dev->board_data.rst_gpio, ret);
        return ret;
    }

    /* Initialize interrupt GPIO */
    ret = devm_gpio_request_one(&spi->dev,
                              uwb_dev->board_data.irq_gpio,
                              GPIOF_IN,
                              CXSEMI_UWB_IRQ_GPIO_NAME);
    if (ret) {
        dev_err(&spi->dev, "Failed to request IRQ GPIO %d: %d\n",
                uwb_dev->board_data.irq_gpio, ret);
        return ret;
    }

    /* Map GPIO to IRQ number */
    uwb_dev->board_data.irq = gpio_to_irq(uwb_dev->board_data.irq_gpio);
    if (uwb_dev->board_data.irq < 0) {
        dev_err(&spi->dev, "Failed to get IRQ number for GPIO %d: %d\n",
                uwb_dev->board_data.irq_gpio, uwb_dev->board_data.irq);
        return uwb_dev->board_data.irq;
    }

    /* Register interrupt handler */
    ret = devm_request_irq(&spi->dev,
                          uwb_dev->board_data.irq,
                          cxsemi_uwb_handler,
                          uwb_dev->board_data.irq_flags,
                          CXSEMI_UWB_DEVICE_NAME,
                          uwb_dev);
    if (ret) {
        dev_err(&spi->dev, "Failed to request IRQ %d: %d\n",
                uwb_dev->board_data.irq, ret);
        return ret;
    }

    /* Start with IRQ disabled - will be enabled on open() */
    disable_irq(uwb_dev->board_data.irq);

    uwb_dev->gpio_configured = true;
    uwb_dev->irq_enabled = false;

    CXSEMI_UWB_LOG_INFO(&spi->dev, "GPIO initialization completed\n");
    return 0;
}

/**
 * cxsemi_uwb_reset_store - Sysfs store function for reset operation
 * @dev: Device structure
 * @attr: Device attribute
 * @buf: Buffer containing reset command
 * @count: Size of buffer
 *
 * Return: Number of bytes processed on success, negative error code on failure
 */
static ssize_t cxsemi_uwb_reset_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    unsigned long value;
    int ret;

    if (!uwb_dev) {
        dev_err(dev, "Device not initialized\n");
        return -ENODEV;
    }

    /* Parse input value */
    ret = kstrtoul(buf, 10, &value);
    if (ret) {
        dev_err(dev, "Invalid reset value: %s\n", buf);
        return ret;
    }

    /* Only proceed if value is 1 (trigger reset) */
    if (value != 1) {
        dev_err(dev, "Reset value must be 1, got %lu\n", value);
        return -EINVAL;
    }

    /* Validate device state */
    ret = cxsemi_uwb_validate_device(uwb_dev);
    if (ret) {
        dev_err(dev, "Device validation failed: %d\n", ret);
        return ret;
    }

    if (mutex_lock_interruptible(&uwb_dev->mutex)) {
        return -ERESTARTSYS;
    }

    cxsemi_uwb_enable_irq(uwb_dev);

    /* Perform reset sequence */
    if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
        /* Pull reset low */
        gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 0);

        /* Add delay to ensure reset pulse width */
        usleep_range(10, 100);

        /* Pull reset high */
        gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 1);

        /* Wait for device to stabilize */
        msleep(CXSEMI_UWB_RESET_DELAY_MS);

        /* Update device state */
        cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_READY);

        CXSEMI_UWB_LOG_DEBUG(dev,
            "Reset sequence completed: low->delay->high\n");
    } else {
        dev_err(dev, "Reset GPIO not valid\n");
        ret = -EINVAL;
    }

    mutex_unlock(&uwb_dev->mutex);

    return ret ? ret : count;
}

/**
 * cxsemi_uwb_reset_show - Sysfs show function for reset status
 * @dev: Device structure
 * @attr: Device attribute
 * @buf: Buffer to store output
 *
 * Return: Number of bytes written to buffer
 */
static ssize_t cxsemi_uwb_reset_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    int gpio_value = -1;

    if (uwb_dev && gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
        gpio_value = gpio_get_value(uwb_dev->board_data.rst_gpio);
    }

    return scnprintf(buf, PAGE_SIZE,
                    "Reset GPIO: %s, Current value: %d\n",
                    gpio_is_valid(uwb_dev->board_data.rst_gpio) ?
                    "Valid" : "Invalid",
                    gpio_value);
}

/* Define device attribute for reset */
static DEVICE_ATTR(reset, 0644, cxsemi_uwb_reset_show, cxsemi_uwb_reset_store);

static ssize_t spi_bits_per_word_show(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);

    if (!uwb_dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE, "%d\n", uwb_dev->spi_bits_per_word);
}

static ssize_t spi_bits_per_word_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    u8 bits;
    int ret;

    if (!uwb_dev)
        return -ENODEV;

    ret = kstrtou8(buf, 0, &bits);
    if (ret)
        return ret;

    if (bits != 8 && bits != 16 && bits != 32) {
        dev_err(dev, "Unsupported SPI bits per word: %d\n", bits);
        return -EINVAL;
    }

    if (mutex_lock_interruptible(&uwb_dev->spi_mutex))
        return -ERESTARTSYS;

    uwb_dev->spi_bits_per_word = bits;
    spi->bits_per_word = bits;

    ret = spi_setup(spi);
    if (ret) {
        dev_err(dev, "Failed to set SPI bits per word to %d: %d\n", bits, ret);
        mutex_unlock(&uwb_dev->spi_mutex);
        return ret;
    }

    mutex_unlock(&uwb_dev->spi_mutex);

    CXSEMI_UWB_LOG_DEBUG(dev, "SPI bits per word set to %d\n", bits);
    return count;
}

static DEVICE_ATTR(spi_bits_per_word, 0644,
                  spi_bits_per_word_show, spi_bits_per_word_store);

static ssize_t spi_max_speed_hz_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);

    if (!uwb_dev)
        return -ENODEV;

    return scnprintf(buf, PAGE_SIZE, "%u\n", uwb_dev->spi_max_speed_hz_current);
}

static ssize_t spi_max_speed_hz_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    u32 speed;
    int ret;

    if (!uwb_dev)
        return -ENODEV;

    ret = kstrtou32(buf, 0, &speed);
    if (ret)
        return ret;

    if (speed < 100000 || speed > 50000000) {
        dev_err(dev, "SPI speed out of range: %u Hz (valid: 100kHz - 50MHz)\n", speed);
        return -EINVAL;
    }

    if (mutex_lock_interruptible(&uwb_dev->spi_mutex))
        return -ERESTARTSYS;

    uwb_dev->spi_max_speed_hz_current = speed;
    spi->max_speed_hz = speed;

    ret = spi_setup(spi);
    if (ret) {
        dev_err(dev, "Failed to set SPI speed to %u Hz: %d\n", speed, ret);
        spi->max_speed_hz = uwb_dev->spi_max_speed_hz_original;
        uwb_dev->spi_max_speed_hz_current = uwb_dev->spi_max_speed_hz_original;
        spi_setup(spi);
        mutex_unlock(&uwb_dev->spi_mutex);
        return ret;
    }
    uwb_dev->spi_max_speed_hz_original = speed;
    mutex_unlock(&uwb_dev->spi_mutex);

    CXSEMI_UWB_LOG_DEBUG(dev, "SPI max speed set to %u Hz\n", speed);
    return count;
}

static DEVICE_ATTR(spi_max_speed_hz, 0644,
                  spi_max_speed_hz_show, spi_max_speed_hz_store);

/**
 * single_read_mode_show
 */
static ssize_t single_read_mode_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    ssize_t count;

    if (!uwb_dev)
        return -ENODEV;

    mutex_lock(&uwb_dev->read_mode_mutex);
    count = scnprintf(buf, PAGE_SIZE, "%d\n", uwb_dev->single_read_mode);
    mutex_unlock(&uwb_dev->read_mode_mutex);

    return count;
}

/**
 * single_read_mode_store
 */
static ssize_t single_read_mode_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);
    unsigned long value;
    int ret;

    if (!uwb_dev)
        return -ENODEV;

    ret = kstrtoul(buf, 0, &value);
    if (ret)
        return ret;

    mutex_lock(&uwb_dev->read_mode_mutex);
    uwb_dev->single_read_mode = (value != 0);
    mutex_unlock(&uwb_dev->read_mode_mutex);

    CXSEMI_UWB_LOG_DEBUG(dev, "Single read mode %s\n",
                        uwb_dev->single_read_mode ? "enabled" : "disabled");

    return count;
}

static DEVICE_ATTR(single_read_mode, 0644,
                  single_read_mode_show, single_read_mode_store);

/**
 * cxsemi_uwb_cleanup - Comprehensive device cleanup
 * @uwb_dev: UWB device structure
 */
static void cxsemi_uwb_cleanup(struct cxsemi_uwb_device *uwb_dev)
{
    if (!uwb_dev) {
        return;
    }

    /* cleanup sysfs */
    if (uwb_dev->spi_dev) {
        /* Delete sysfs */
        device_remove_file(&uwb_dev->spi_dev->dev, &dev_attr_spi_bits_per_word);
        device_remove_file(&uwb_dev->spi_dev->dev, &dev_attr_spi_max_speed_hz);
        device_remove_file(&uwb_dev->spi_dev->dev, &dev_attr_reset);
        device_remove_file(&uwb_dev->spi_dev->dev, &dev_attr_single_read_mode);
    }

    /* Disable IRQ if enabled */
    if (uwb_dev->irq_enabled) {
        disable_irq(uwb_dev->board_data.irq);
        uwb_dev->irq_enabled = false;
    }

    /* Deregister misc device */
    if (uwb_dev->device_initialized) {
        misc_deregister(&uwb_dev->miscdev);
        uwb_dev->device_initialized = false;
    }

    /* Clean up synchronization primitives */
    mutex_destroy(&uwb_dev->mutex);
    mutex_destroy(&uwb_dev->spi_mutex);
    mutex_destroy(&uwb_dev->read_mode_mutex);

    /* Reset device state */
    cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_UNINITIALIZED);
}

/**
 * cxsemi_uwb_spi_probe - Enhanced SPI device probe function
 * @spi: SPI device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int cxsemi_uwb_spi_probe(struct spi_device *spi)
{
    struct cxsemi_uwb_device *uwb_dev;
    int ret;

    CXSEMI_UWB_LOG_DEBUG(&spi->dev, "Probing CXSEMI UWB SPI device\n");

    /* Validate SPI device */
    if (!spi) {
        dev_err(&spi->dev, "Invalid SPI device\n");
        return -EINVAL;
    }

    /* Allocate device memory */
    uwb_dev = devm_kzalloc(&spi->dev, sizeof(*uwb_dev), GFP_KERNEL);
    if (!uwb_dev) {
        dev_err(&spi->dev, "Failed to allocate device memory\n");
        return -ENOMEM;
    }

    /* Store device context */
    uwb_dev->spi_dev = spi;
    spi_set_drvdata(spi, uwb_dev);

    /* Initialize device state */
    cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_UNINITIALIZED);

    /* Initialize synchronization primitives */
    mutex_init(&uwb_dev->mutex);
    mutex_init(&uwb_dev->spi_mutex);
    mutex_init(&uwb_dev->read_mode_mutex);
    init_waitqueue_head(&uwb_dev->read_wait);
    atomic_set(&uwb_dev->irq_occurred, 0);

    /* dual read mode by default */
    uwb_dev->single_read_mode = false;

    /* Initialize statistics */
    atomic_long_set(&uwb_dev->spi_transfer_errors, 0);
    atomic_long_set(&uwb_dev->irq_count, 0);

    /* Allocate SPI transfer buffers */
    uwb_dev->tx_buff = devm_kzalloc(&spi->dev, CXSEMI_UWB_SPI_BUFF_SIZE, GFP_KERNEL);
    uwb_dev->rx_buff = devm_kzalloc(&spi->dev, CXSEMI_UWB_SPI_BUFF_SIZE, GFP_KERNEL);
    if (!uwb_dev->tx_buff || !uwb_dev->rx_buff) {
        dev_err(&spi->dev, "Failed to allocate SPI transfer buffers\n");
        ret = -ENOMEM;
        goto cleanup;
    }

    /* Parse device tree configuration */
    if (IS_ENABLED(CONFIG_OF) && spi->dev.of_node) {
        ret = cxsemi_uwb_parse_dt(spi, &uwb_dev->board_data);
        if (ret) {
            dev_err(&spi->dev, "Failed to parse device tree: %d\n", ret);
            goto cleanup;
        }
    } else {
        dev_err(&spi->dev, "No device tree support or node found\n");
        ret = -ENODEV;
        goto cleanup;
    }

    /* Initialize GPIO */
    ret = cxsemi_uwb_gpio_init(uwb_dev);
    if (ret) {
        dev_err(&spi->dev, "GPIO initialization failed: %d\n", ret);
        goto cleanup;
    }

    /* Configure SPI device */
    spi->mode = SPI_MODE_0;

    uwb_dev->spi_max_speed_hz_original = uwb_dev->board_data.spi_setting.spi_max_speed;
    uwb_dev->spi_max_speed_hz_current = uwb_dev->spi_max_speed_hz_original;
    spi->max_speed_hz = uwb_dev->spi_max_speed_hz_current;

    uwb_dev->spi_bits_per_word = 8;
    spi->bits_per_word = uwb_dev->spi_bits_per_word;

    ret = spi_setup(spi);
    if (ret) {
        dev_err(&spi->dev, "Failed to setup SPI device: %d\n", ret);
        goto cleanup;
    }

    /* Initialize misc device */
    uwb_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
    uwb_dev->miscdev.name = CXSEMI_UWB_DEVICE_NAME;
    uwb_dev->miscdev.fops = &cxsemi_uwb_fops;
    uwb_dev->miscdev.parent = &spi->dev;
    uwb_dev->miscdev.mode = 0660;

    ret = misc_register(&uwb_dev->miscdev);
    if (ret) {
        dev_err(&spi->dev, "Failed to register misc device: %d\n", ret);
        goto cleanup;
    }
    uwb_dev->device_initialized = true;

    ret = device_create_file(&spi->dev, &dev_attr_reset);
    if (ret) {
        dev_err(&spi->dev, "Failed to create reset sysfs file: %d\n", ret);
    }
    ret = device_create_file(&spi->dev, &dev_attr_spi_bits_per_word);
    if (ret) {
        dev_err(&spi->dev, "Failed to create spi_bits_per_word sysfs file: %d\n", ret);
    }
    ret = device_create_file(&spi->dev, &dev_attr_spi_max_speed_hz);
    if (ret) {
        dev_err(&spi->dev, "Failed to create spi_max_speed_hz sysfs file: %d\n", ret);
    }
    ret = device_create_file(&spi->dev, &dev_attr_single_read_mode);
    if (ret) {
        dev_err(&spi->dev, "Failed to create single_read_mode sysfs file: %d\n", ret);
    }

    /* Set device to ready state */
    cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_READY);

    /* Enable runtime power management */
    pm_runtime_enable(&spi->dev);

    CXSEMI_UWB_LOG_DEBUG(&spi->dev, "CXSEMI UWB device probe completed successfully\n");
    return 0;

cleanup:
    cxsemi_uwb_cleanup(uwb_dev);
    return ret;
}

/**
 * cxsemi_uwb_spi_remove - SPI device remove function
 * @spi: SPI device structure
 *
 * Return: 0 on success
 */
static void cxsemi_uwb_spi_remove(struct spi_device *spi)
{
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);

    CXSEMI_UWB_LOG_DEBUG(&spi->dev, "Removing CXSEMI UWB driver\n");

    if (uwb_dev) {
        /* Disable runtime PM */
        pm_runtime_disable(&spi->dev);

        /* Ensure device is suspended */
        pm_runtime_dont_use_autosuspend(&spi->dev);

        /* Clean up resources */
        cxsemi_uwb_cleanup(uwb_dev);
    }
}

/**
 * cxsemi_uwb_spi_suspend - Runtime suspend function
 * @dev: Device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int __maybe_unused cxsemi_uwb_spi_suspend(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);

    if (!uwb_dev) {
        return -ENODEV;
    }

    /* Disable IRQ */
    cxsemi_uwb_disable_irq(uwb_dev);

    /* Set reset GPIO to low to save power */
    if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
        gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 0);
    }

    cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_SUSPENDED);

    dev_dbg(dev, "UWB device suspended\n");
    return 0;
}

/**
 * cxsemi_uwb_spi_resume - Runtime resume function
 * @dev: Device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int __maybe_unused cxsemi_uwb_spi_resume(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct cxsemi_uwb_device *uwb_dev = spi_get_drvdata(spi);

    if (!uwb_dev) {
        return -ENODEV;
    }

    /* Enable device */
    if (gpio_is_valid(uwb_dev->board_data.rst_gpio)) {
        gpio_set_value_cansleep(uwb_dev->board_data.rst_gpio, 1);
        msleep(CXSEMI_UWB_RESET_DELAY_MS);
    }

    /* Re-enable IRQ if device was opened */
    if (!uwb_dev->irq_enabled) {
        cxsemi_uwb_enable_irq(uwb_dev);
    }

    cxsemi_uwb_set_device_state(uwb_dev, CXSEMI_UWB_STATE_READY);

    dev_dbg(dev, "UWB device resumed\n");
    return 0;
}

/**
 * cxsemi_uwb_spi_suspend_noirq - System sleep suspend function
 * @dev: Device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int __maybe_unused cxsemi_uwb_spi_suspend_noirq(struct device *dev)
{
    return cxsemi_uwb_spi_suspend(dev);
}

/**
 * cxsemi_uwb_spi_resume_noirq - System sleep resume function
 * @dev: Device structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int __maybe_unused cxsemi_uwb_spi_resume_noirq(struct device *dev)
{
    return cxsemi_uwb_spi_resume(dev);
}

static const struct dev_pm_ops cxsemi_uwb_pm_ops = {
    SET_RUNTIME_PM_OPS(cxsemi_uwb_spi_suspend, cxsemi_uwb_spi_resume, NULL)
    SET_SYSTEM_SLEEP_PM_OPS(cxsemi_uwb_spi_suspend_noirq, cxsemi_uwb_spi_resume_noirq)
};

/* Device tree match table */
#ifdef CONFIG_OF
static const struct of_device_id cxsemi_uwb_of_match[] = {
    { .compatible = "cxsemi,uwb", },
    {},
};
MODULE_DEVICE_TABLE(of, cxsemi_uwb_of_match);
#endif

/* SPI device ID table */
static const struct spi_device_id cxsemi_uwb_spi_id[] = {
    { CXSEMI_UWB_DRIVER_NAME, 0 },
    {},
};
MODULE_DEVICE_TABLE(spi, cxsemi_uwb_spi_id);

/* SPI driver structure */
static struct spi_driver cxsemi_uwb_spi_driver = {
    .driver = {
        .name = CXSEMI_UWB_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(cxsemi_uwb_of_match),
        .pm = &cxsemi_uwb_pm_ops,
    },
    .id_table = cxsemi_uwb_spi_id,
    .probe = cxsemi_uwb_spi_probe,
    .remove = cxsemi_uwb_spi_remove,
};

module_spi_driver(cxsemi_uwb_spi_driver);

MODULE_DESCRIPTION("CXSEMI UWB Driver");
MODULE_AUTHOR("Yue Sun <sunyue5@motorola.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
