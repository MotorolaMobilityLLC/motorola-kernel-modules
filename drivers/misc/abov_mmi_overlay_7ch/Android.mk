DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(ABOV_MTK_CHARGER),true)
    KERNEL_CFLAGS  += CONFIG_ABOV_MTK_CHARGER=y
endif

ifeq ($(ABOV_MTK_KERNEL419_CHARGER_TYPE), true)
    KERNEL_CFLAGS  += CONFIG_ABOV_MTK_KERNEL419_CHARGER_TYPE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := abov_sar_mmi_overlay_7ch.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

