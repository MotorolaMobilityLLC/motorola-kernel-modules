
#ifndef __LINUX_PLATFORM_DATA_ESWIN_EPH_ESWIN_H
#define __LINUX_PLATFORM_DATA_ESWIN_EPH_ESWIN_H

#include "eswin_eph861x_types.h"

#define GPIO_RESET_NO_HIGH  1
#define GPIO_RESET_YES_LOW  0
#define CHG_HELD_LOW        0

extern int eph_check_firmware_format(struct device* dev, const struct firmware* fw);
extern int eph_update_file_name(struct device* dev,
                                char** out_file_name,
                                const char* in_file_name,
                                size_t in_str_len);
extern const struct eph_platform_data* eph_platform_data_get_from_device_tree(struct comms_device* commsdevice);
extern struct eph_platform_data* eph_platform_data_get_default(struct comms_device* commsdevice);
extern const struct eph_platform_data* eph_platform_data_get(struct comms_device* commsdevice);
extern int eph_gpio_setup(struct eph_data* ephdata);
extern int eph_wait_for_completion(struct eph_data *ephdata,
                                   struct completion *comp,
                                   unsigned int timeout_ms,
                                   const char *dbg_str);
extern void eph_regulator_enable(struct eph_data *ephdata);
extern void eph_regulator_disable(struct eph_data *ephdata);
extern void eph_reset_device(struct eph_data *ephdata);
extern int eph_power_on(struct eph_data *ephdata);
extern int eph_power_off(struct eph_data *ephdata);
extern void eph_recovery_device(struct eph_data *ephdata);
extern bool eph_is_fod_resume(struct eph_data *ephdata);
extern int eph_irq_enable(struct eph_data *ephdata, bool enable);

#ifdef CONFIG_ESWIN_GHOST_LOG_CAPTURE
void eph_cache_debug_log(struct eph_data *ephdata);
int frame_log_capture_start(struct eph_data *ephdata);
int frame_log_capture_stop(struct eph_data *ephdata);
void put_fifo_with_discard(char *log_buf, int len);
void clear_kfifo(void);
int eswin_log_capture_register_misc(struct eph_data *ephdata);
int eswin_log_capture_unregister_misc(struct eph_data *ephdata);
#endif

#endif /* __LINUX_PLATFORM_DATA_ESWIN_EPH_ESWIN_ */
