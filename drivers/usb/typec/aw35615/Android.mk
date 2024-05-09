DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

include $(CLEAR_VARS)
LOCAL_MODULE := aw35615.ko
LOCAL_MODULE_TAGS := optional

KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
