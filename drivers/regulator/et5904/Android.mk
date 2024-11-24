DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(PICOLEAF_FOR_TOUCH),)
	KBUILD_OPTIONS += PICOLEAF_DATA_FOR_TOUCH=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := et5904.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

