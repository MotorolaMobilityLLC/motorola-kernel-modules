DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(EGIS_SCREEN_EVENT_DISABLE),true)
     KERNEL_CFLAGS += CONFIG_EGIS_SCREEN_EVENT_DISABLE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := ets_bix_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

