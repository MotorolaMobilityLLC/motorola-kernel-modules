DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(HIMAX_MTK_IC_HX83112),true)
	KERNEL_CFLAGS += CONFIG_INPUT_HIMAX_MTK_MMI_IC_NAME=hx83112
endif

KERNEL_CFLAGS += HIMAX_CONFIG_KALLSYMS_ALL=y

include $(CLEAR_VARS)
LOCAL_MODULE := himax_mtk_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
