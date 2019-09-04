DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
NOVA_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
NOVA_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifneq ($(BOARD_USES_DOUBLE_TAP),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_ENABLE_DOUBLE_TAP=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := nova_0flash_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(NOVA_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk
