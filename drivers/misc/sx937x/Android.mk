DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := sx937x_sar.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

ifeq ($(SX937X_USB_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX937X_USB_CAL=y
endif

ifeq ($(SX937X_FLIP_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX937X_FLIP_CAL=y
endif
