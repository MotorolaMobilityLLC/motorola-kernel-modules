DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_discrete_turbo_charger.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(MMI_QC3P_WT6670_DETECTED),true)
        KERNEL_CFLAGS += CONFIG_MMI_QC3P_WT6670_DETECTED=y
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

include $(DLKM_DIR)/AndroidKernelModule.mk
