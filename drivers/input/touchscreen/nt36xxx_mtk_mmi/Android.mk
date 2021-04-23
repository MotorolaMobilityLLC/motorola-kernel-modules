DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(NOVATEK_CHARGE_DETECTION),true)
        KERNEL_CFLAGS += CONFIG_NVT_CHARGE_DETECT=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := nt36xxx_mtk_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
