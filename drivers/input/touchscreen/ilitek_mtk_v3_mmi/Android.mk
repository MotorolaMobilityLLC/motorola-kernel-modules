DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(ILITEK_CHARGE_DETECTION),true)
        KERNEL_CFLAGS += CONFIG_ILITEK_CHARGE_DETECT=y
endif

ifeq ($(ILITEK_GESTURE),true)
        KERNEL_CFLAGS += CONFIG_ILITEK_GESTURE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := ilitek_mtk_v3_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
