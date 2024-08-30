#ifndef __STK501XX_QUALCOMM_H__
#define __STK501XX_QUALCOMM_H__

#include "stk501xx.h"
#include "common_define.h"

#ifdef CONFIG_CAPSENSE_FLIP_CAL
#include <linux/extcon.h>
#endif
#define STK_HEADER_VERSION          "0.0.7"//C+D+E+F
#define STK_C_VERSION               "0.0.3"
#define STK_DRV_I2C_VERSION         "0.0.2"
#define STK_QUALCOMM_VERSION        "0.0.1"
#define STK_ATTR_VERSION            "0.0.1"

#define ch_num 5

#define INPUT_DEVICE_CH0 "Moto CapSense Ch0"
#define INPUT_DEVICE_CH1 "Moto CapSense Ch1"
#define INPUT_DEVICE_CH2 "Moto CapSense Ch2"
#define INPUT_DEVICE_CH3 "Moto CapSense Ch3"
#define INPUT_DEVICE_CH4 "Moto CapSense Ch4"
#define INPUT_DEVICE_CH5 "Moto CapSense Ch5"
#define INPUT_DEVICE_CH6 "Moto CapSense Ch6"
#define INPUT_DEVICE_CH7 "Moto CapSense Ch7"

char input_dev_name[8][20] = {INPUT_DEVICE_CH0, INPUT_DEVICE_CH1, INPUT_DEVICE_CH2, INPUT_DEVICE_CH3,
                              INPUT_DEVICE_CH4, INPUT_DEVICE_CH5, INPUT_DEVICE_CH6, INPUT_DEVICE_CH7
                             };

typedef struct channels_t
{
    struct input_dev        *input_dev;   /* data */

#if defined STK_QUALCOMM || defined STK_SPREADTRUM
#ifdef STK_QUALCOMM
#ifdef STK_SENSORS_DEV
    struct sensors_classdev     sar_cdev;
#endif // STK_SENSORS_DEV
    u64                         fifo_start_ns;
#endif /* STK_QUALCOMM */
    ktime_t                     timestamp;
#elif defined STK_MTK
    //struct sar_hw               hw;
    //struct hwmsen_convert       cvt;
#endif /* STK_QUALCOMM, STK_SPREADTRUM, STK_MTK */
} channels_t;

typedef struct stk501xx_wrapper
{
    struct i2c_manager      i2c_mgr;
    struct stk_data         stk;
    channels_t              channels[ch_num];
#ifdef CONFIG_CAPSENSE_USB_CAL
    struct work_struct ps_notify_work;
    struct notifier_block ps_notif;
    bool ps_is_present;
#ifdef CONFIG_CAPSENSE_ATTACH_CAL
    bool phone_is_present;
#endif

#ifdef CONFIG_CAPSENSE_FLIP_CAL
    struct notifier_block flip_notif;
    struct extcon_dev *ext_flip_det;
    bool phone_flip_state;
    bool phone_flip_update_regs;
    int phone_flip_open_val;
    int num_flip_closed_regs;
    int num_flip_open_regs;
    struct smtc_reg_data *flip_open_regs;
    struct smtc_reg_data *flip_closed_regs;
#endif
#endif
} stk501xx_wrapper;

int stk_i2c_probe(struct i2c_client* client,
                  struct common_function *common_fn);
int stk_i2c_remove(struct i2c_client* client);
int stk501xx_suspend(struct device* dev);
int stk501xx_resume(struct device* dev);
#endif // __STK501XX_QUALCOMM_H__
