DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := mmi_sc8549.ko


LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)



include $(DLKM_DIR)/AndroidKernelModule.mk
