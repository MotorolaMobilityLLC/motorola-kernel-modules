DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(AW9610_MTK_CHARGER),true)
    KERNEL_CFLAGS  += CONFIG_AW9610_MTK_CHARGER=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := aw9610x.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

