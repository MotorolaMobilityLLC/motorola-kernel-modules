/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FCNT LIMITED 2021
/*----------------------------------------------------------------------------*/

#ifndef _CYPSOC_PICOLEAF_H
#define _CYPSOC_PICOLEAF_H

#define CYPSOC_PICOLEAF_ENABLE

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>

/////////////////////////////////////////////////////////
///  Register addresses of cypress PSoC for Picoleaf  ///
#define CYPSOC_PICOLEAF_REG_INTEGRATED_PRESS_L (0x0000U)
#define CYPSOC_PICOLEAF_REG_INTEGRATED_PRESS_H (0x0001U)
#define CYPSOC_PICOLEAF_REG_ADC_DATA_L         (0x0002U)
#define CYPSOC_PICOLEAF_REG_ADC_DATA_H         (0x0003U)
#define CYPSOC_PICOLEAF_REG_ADC_OFFSET_L       (0x0004U)
#define CYPSOC_PICOLEAF_REG_ADC_OFFSET_H       (0x0005U)
#define CYPSOC_PICOLEAF_REG_CLEAR_INTEGRA      (0x000AU)
#define CYPSOC_PICOLEAF_REG_RESET              (0x000CU)
#define CYPSOC_PICOLEAF_REG_ENTER_BOOTLOADER   (0x000FU)
#define CYPSOC_PICOLEAF_REG_FIRMWARE_VER       (0x0010U)
#define CYPSOC_PICOLEAF_REG_MICON_STATUS       (0x0011U)
#define CYPSOC_PICOLEAF_REG_HREST_STATUS       (0x0013U)
#define CYPSOC_PICOLEAF_REG_TEST_MODE          (0x0024U)
#define CYPSOC_PICOLEAF_REG_TEST_COMMAND       (0x0028U)
#define CYPSOC_PICOLEAF_REG_NOISE_DATA_L       (0x0029U)
#define CYPSOC_PICOLEAF_REG_ADC_DATA_P2P_L     (0x0034U)

#define CYPSOC_PICOLEAF_DEEP_SLEEP_ENABLE      (1U << 4)
#define CYPSOC_PICOLEAF_DEEP_SLEEP_DISABLE     (0U)
#define CYPSOC_PICOLEAF_CLEAR_INTEGRA          (1U)
#define CYPSOC_PICOLEAF_CLEAR_OFFSET_INTEGRA   (1U << 1)
#define CYPSOC_PICOLEAF_BOOTLOADER_MODE        (1U)
#define CYPSOC_PICOLEAF_START_NOISE_TEST       (1U)
#define CYPSOC_PICOLEAF_TEST_MODE_OFF          (0U)
#define CYPSOC_PICOLEAF_TEST_MODE_ON           (1U)
#define CYPSOC_PICOLEAF_HRST_RELEASE           (1U)
#define CYPSOC_PICOLEAF_MAX_PRBUF_SIZE PIPE_BUF
#define CYPSOC_PICOLEAF_MAX_FILE_SIZE  0x18000

///////////////////////////////////////////////////////
///  Move to linux/cypsoc_core.h this declaration   ///
#define CYPSOC_DRIVER_VERSION 3
#define CYPSOC_PICOLEAF_NAME "cypsoc_adapter"

#define CYPSOC_FW_FILE_NAME  "pico.cyacd"
#define CYPSOC_BIN_FILE_PATH "/vendor/firmware/pico.cyacd"

enum {
	CYPSOC_PICOLEAF_PT_I2C_PULL_UP_OK,
	CYPSOC_PICOLEAF_PT_I2C_PULL_UP_NG = 1
};

enum {
	CYPSOC_PICOLEAF_STATUS_AVAILABLE,
	CYPSOC_PICOLEAF_STATUS_BROKEN,
	CYPSOC_PICOLEAF_STATUS_INITIALIZING,
	CYPSOC_PICOLEAF_STATUS_FW_UPDATING,
	CYPSOC_PICOLEAF_STATUS_I2C_RETRYING,
	CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP,
	CYPSOC_PICOLEAF_STATUS_RAISE_ERROR,
	CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE,
};

enum {
	CYPSOC_PICOLEAF_RET_NG = -1,
	CYPSOC_PICOLEAF_RET_OK,
	CYPSOC_PICOLEAF_RET_NG_I2C_PULL_UP = 1,
	CYPSOC_PICOLEAF_RET_NG_PROCESSING,
	CYPSOC_PICOLEAF_RET_NG_POWER_OFF,
	CYPSOC_PICOLEAF_RET_NG_SLEEPING,
	CYPSOC_PICOLEAF_RET_NG_I2C_RETRY,
	CYPSOC_PICOLEAF_RET_NG_FW_FF,
};

#define CYPSOC_DRIVER_I2C_RETRY_MAX 3U
#define CYPSOC_NOTIFY_FW_TO_RK_OFF  0x010000U
#define CYPSOC_DEFAULT_PRESS_Z      0x01U
#define CYPSOC_TEST_MODE_PRESS_Z    0x0U
#define CYPSOC_STATUS_MAX_LEN       30U
#define CYPSOC_ON                   "on"
#define CYPSOC_SLEEP                "off"
#define CYPSOC_OK_STATE             "ok"
#define CYPSOC_I2C_ERROR            "i2c_error"
#define CYPSOC_ACCESS_MODE_READ     'r'
#define CYPSOC_ACCESS_MODE_WRITE    'w' // contain err
#define CYPSOC_ACCESS_MODE_SUCCESS  's'

struct cypsoc_picoleaf_module {
	struct list_head node;
	char *name;
	int (*probe)(struct device *dev, void **data);
	void (*release)(struct device *dev, void *data);
};

struct cypsoc_picoleaf_data {
    struct device *dev;
	struct mutex sysfs_lock;
	struct mutex psoc_status_lock;
	struct mutex prev_status_lock;
	struct mutex i2c_power_changed_lock;

	int     debug_level;
	char    acc_mode;
	int     acc_add;
	int     acc_arg;

	uint8_t i2c_pull_up;
	int     psoc_status;
	int     prev_status;
	int     probe_readid_not_esd_reset;

	struct class  *sysfs_class;
	struct device *sysfs_dev;

	struct work_struct i2c_retry_work;

	int rst_gpio;
	int vdd_gpio;
	int vref_gpio;

	struct pt_core_data *pt_core_data;
};

struct cypsoc_picoleaf_i2c_cmds {
	uint8_t *bootload_cmd;
	uint8_t *erase_row_cmd;
	uint8_t *send_data_cmd;
	uint8_t *prog_row_cmd;
	uint8_t *verify_row_cmd;
	uint8_t *verify_chksum_cmd;
	uint8_t *exit_cmd;
};

struct cypsoc_fw_parsed_row {
	uint8_t  array_id;
	uint16_t row_num;
	uint8_t *row_data;
	uint16_t row_size;
	uint8_t  row_chksum;
};

/////////////////////////////////////////////////
////    Interfaces For touchscreen driver    ////
int  cypsoc_picoleaf_probe_cont(struct cypsoc_picoleaf_data *cpd);
int  cypsoc_picoleaf_sysclass_group_register(struct cypsoc_picoleaf_data *cpd);
void cypsoc_picoleaf_shutdown_cont(struct cypsoc_picoleaf_data *cpd);
void cypsoc_picoleaf_i2c_power_turned_on(struct cypsoc_picoleaf_data *cpd);
void cypsoc_picoleaf_i2c_power_turned_off(struct cypsoc_picoleaf_data *cpd);
int cypsoc_picoleaf_i2c_readied(struct cypsoc_picoleaf_data *cpd);
int  cypsoc_picoleaf_get_press_z(int *press);
int  cypsoc_picoleaf_notification_enabled(void);
int  cypsoc_picoleaf_firmware_update(struct cypsoc_picoleaf_data *cpd);
void cypsoc_picoleaf_suspend(void);
void cypsoc_picoleaf_resume(void);
int cypsoc_picoleaf_force_power_on_hw(struct cypsoc_picoleaf_data *cpd);

#endif /* _CYPSOC_PICOLEAF_H */