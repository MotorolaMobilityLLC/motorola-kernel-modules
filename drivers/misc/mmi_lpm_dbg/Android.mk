DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(MMI_LPM_DBG),true)
    KERNEL_CFLAGS  += CONFIG_MMI_LPM_DBG=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_lpm_dbg.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
