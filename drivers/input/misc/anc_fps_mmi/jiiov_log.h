#ifndef JIIOV_LOG_H
#define JIIOV_LOG_H

#define ANC_LOGD(format, ...) \
    printk(KERN_DEBUG "[D][ANC_DRIVER][%s] " format "\n", __func__, ##__VA_ARGS__)
#define ANC_LOGI(format, ...) \
    printk(KERN_INFO "[I][ANC_DRIVER][%s] " format "\n", __func__, ##__VA_ARGS__)
#define ANC_LOGW(format, ...) \
    printk(KERN_WARNING "[W][ANC_DRIVER][%s] " format "\n", __func__, ##__VA_ARGS__)
#define ANC_LOGE(format, ...) \
    printk(KERN_ERR "[E][ANC_DRIVER][%s] " format "\n", __func__, ##__VA_ARGS__)

#define CHECK_PTR_PARAM(_ptr)                                                            \
    do {                                                                                 \
        if (_ptr == NULL) {                                                              \
            ANC_LOGE("%s:%d, invalid parameter, %s is NULL", __func__, __LINE__, #_ptr); \
            return -1;                                                                   \
        }                                                                                \
    } while (0)

#define CHECK_INT_PARAM(_integer)                                                         \
    do {                                                                                  \
        if (_integer == 0) {                                                              \
            ANC_LOGE("%s:%d, invalid parameter, %s is 0", __func__, __LINE__, #_integer); \
            return -1;                                                                    \
        }                                                                                 \
    } while (0)

#endif
