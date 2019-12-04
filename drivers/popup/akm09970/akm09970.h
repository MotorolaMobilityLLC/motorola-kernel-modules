#ifndef __AKM09970_H__
#define __AKM09970_H__

/********************************** CNTL1 *************************************/
// Spec 12.3.5
#define AKM09970_SET_CNTL1_DRDY		0x0001 // disable 0x0000, enable 0x0001
#define AKM09970_SET_CNTL1_SWEN		0x0000 // disable 0x0000, enable 0x0001
#define AKM09970_SET_CNTL1_ERRXYEN	0x0080 // disable 0x0000, enable 0x0080
#define AKM09970_SET_CNTL1_ERRADCEN	0x0100 // disable 0x0000, enable 0x0100
#define AKM09970_SET_CNTL1_INTEN	0x0000 // disable 0x0000, enable 0x0200
#define AKM09970_SET_CNTL1_ODINTEN	0x0000 // disable 0x0000, enable 0x0400
#define AKM09970_SET_CNTL1		(AKM09970_SET_CNTL1_DRDY     | \
					 AKM09970_SET_CNTL1_SWEN     | \
					 AKM09970_SET_CNTL1_ERRXYEN  | \
					 AKM09970_SET_CNTL1_ERRADCEN | \
					 AKM09970_SET_CNTL1_INTEN    | \
					 AKM09970_SET_CNTL1_ODINTEN)

/********************************** CNTL2 *************************************/
/* AKM09970_REG_SDR (spec 12.3.6)
 * 0x00 : Low noise drive
 * 0x10 : Low power drive */
#define AKM09970_SET_SDR     (0x00)

/* AKM09970_REG_SMR (spec 12.3.6)
 * 0x00 : High sensitivity
 * 0x20 : Wide measurement range */
#ifdef AKM09970_WIDE_RANGE
#define AKM09970_SET_SMR     (0x20)
#define AKM09970_SENS_Q14    ((int32_t)(50790)) /* 3.1uT in Q14 format, 3.1 * 16384 */
#define AKM09970_VAR_LIMIT_HIGH 260
#define AKM09970_VAR_LIMIT_LOW  0
#else // AKM09970_WIDE_RANGE
#define AKM09970_SET_SMR     (0x00)
#define AKM09970_SENS_Q14    ((int32_t)(18022))  /* 1.1uT in Q14 format, 1.1 * 16384 */
#define AKM09970_VAR_LIMIT_HIGH 2066
#define AKM09970_VAR_LIMIT_LOW  0
#endif // AKM09970_WIDE_RANGE

#define AKM09970_SET_MODE(ctrl2)  (AKM09970_SET_SDR | AKM09970_SET_SMR | ctrl2)

/********************************* Constant ***********************************/
#define AKM09970_BDATA_SIZE                      8
#define AKM09970_SOFT_RESET                      0x01
#define AKM09970_WIA_VAL                         0xC048

/***************************** Operation modes ********************************/
#define AKM09970_MODE_POWER_DOWN                 0x00
#define AKM09970_MODE_SNG_MEASURE                0x01
#define AKM09970_MODE_CONT_MEASURE_MODE1         0x02 // 0.25Hz
#define AKM09970_MODE_CONT_MEASURE_MODE2         0x04 // 0.5Hz
#define AKM09970_MODE_CONT_MEASURE_MODE3         0x06 // 1Hz
#define AKM09970_MODE_CONT_MEASURE_MODE4         0x08 // 10Hz
#define AKM09970_MODE_CONT_MEASURE_MODE5         0x0A // 20Hz
#define AKM09970_MODE_CONT_MEASURE_MODE6         0x0C // 50Hz
#define AKM09970_MODE_CONT_MEASURE_MODE7         0x0E // 100Hz
#define AKM09970_MODE_SELF_TEST                  0x10

/******************************* Registers ************************************/
#define AKM09970_REG_WIA                         0x00
#define AKM09970_REG_ST1                         0x10
#define AKM09970_REG_ST1_XYZ                     0x17
#define AKM09970_REG_CNTL1                       0x20
#define AKM09970_REG_CNTL2                       0x21
#define AKM09970_REG_SRST                        0x30

#define AKM09970_WIA_SIZE 		2
#define AKM09970_ST1_SIZE		2
#define AKM09970_BDATA_SIZE		8
#define AKM09970_CNTL1_SIZE		2
#define AKM09970_CNTL2_SIZE		1
#define AKM09970_SRST_SIZE		1

/* ******************event property************************* */
#define DEFAULT_EVENT_DATA_CAPABILITY_MIN   (-32768)
#define DEFAULT_EVENT_DATA_CAPABILITY_MAX   (32767)

/* ******************delay property************************* */
#define AKM09970_ODR_10HZ			10
#define AKM09970_ODR_20HZ			20
#define AKM09970_ODR_50HZ			50
#define AKM09970_ODR_100HZ			100
#define AKM09970_DELAY_025HZ			4000000000LL
#define AKM09970_DELAY_05HZ			2000000000LL
#define AKM09970_DELAY_1HZ			1000000000LL
#define AKM09970_DELAY_10HZ			100000000LL
#define AKM09970_DELAY_20HZ			50000000LL
#define AKM09970_DELAY_50HZ			20000000LL
#define AKM09970_DELAY_100HZ			10000000LL
#define AKM09970_MIN_DELAY_NS			AKM09970_DELAY_100HZ// 10ms, 100Hz
#define AKM09970_MAX_DELAY_NS			AKM09970_DELAY_025HZ// 4s, 0.25Hz
#define AKM09970_DEFAULT_DELAY_NS		AKM09970_DELAY_100HZ// 10ms

/* MagnaChip Hall Sensor power supply VDD 1.7V~3.6V, typ.1.8V */
#define AKM09970_VDD_MIN_UV			1700000
#define AKM09970_VDD_MAX_UV			3600000
#define AKM09970_CONTROL			"control"

#define AKM09970_RESET_DELAY_MS		1
#define AKM09970_DRDY_DELAY_MS		2
#define AKM09970_DRAY_RETRY_MAX		3
#define AKM09970_SOFT_RESET_VALUE	0x01

#define AKM09970_WIA_VALUE		0xC048
#define AKM09970_ST1_IS_DRDY_VALUE	0xFC01
#define AKM09970_ST1_NO_DRDY_VALUE	0xFC00

#define AKM09970_CHECK_ERR (-1)

typedef struct {
    struct mutex enable;
    struct mutex data;
} akm09970_mutex_t;

typedef struct {
    atomic64_t delay;
    atomic_t   enable;
    atomic_t   debug;
} akm09970_atomic_t;

struct mag_data_type {
    int16_t x;
    int16_t y;
    int16_t z;
};

typedef struct {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct class *akm09970_class;
    struct device *sysfs_dev;
    const char *type;
    akm09970_mutex_t  mtx;
    akm09970_atomic_t atm;

    struct work_struct akm09970_work;
    struct workqueue_struct *akm09970_wq;
    struct hrtimer sample_timer;
    struct task_struct *akm09970_task;
    wait_queue_head_t sync_complete;
    bool sync_flag;

    int32_t igpio;
    int32_t gpio_rst;
    int32_t int_en;
    int32_t irq;
    int32_t use_hrtimer;
    struct regulator *vdd;
    int32_t power_enabled;
    int32_t power_always_on;
} akm09970_i2c_data;

#endif /* __AKM09970_H__ */
