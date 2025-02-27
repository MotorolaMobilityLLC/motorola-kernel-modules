DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(WL2868C_ET5904_ORDER),)
	KBUILD_OPTIONS += WL2868C_ET5904_ORDER_EN=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := wl2868c.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

