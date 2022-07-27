DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(CHIPONE_GPIO_CTRL_VDD),true)
    KERNEL_CFLAGS  += CONFIG_CHIPONE_GPIO_CTRL_VDD=y
endif

ifeq ($(CHIPONE_MTK_KERNEL5XX_WAKE_TYPE), true)
    KERNEL_CFLAGS  += CONFIG_CHIPONE_MTK_KERNEL5XX_WAKE_TYPE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := fpsensor_mtk_spi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

