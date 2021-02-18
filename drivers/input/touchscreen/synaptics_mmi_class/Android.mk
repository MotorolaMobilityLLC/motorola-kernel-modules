DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_MMI=y

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/mmi_info.ko
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_mmi.ko
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_dsx_fw_update.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_dsx_rmi_dev.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_dsx_test_reporting.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
endif
