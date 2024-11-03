# Test module, can only be used on debugging build.
ifneq (,$(filter userdebug eng,$(TARGET_BUILD_VARIANT)))

DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mrkp_test.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
endif

