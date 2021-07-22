DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_HAS_MULTI_FPS),true)
	ifneq ($(KERNEL_CFLAGS),)
		KERNEL_CFLAGS += CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE=y
	else
		KBUILD_OPTIONS += CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE=y
	endif
endif

include $(CLEAR_VARS)
LOCAL_MODULE := fpc1020_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

