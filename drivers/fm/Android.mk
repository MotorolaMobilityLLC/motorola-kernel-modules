ifeq ($(BOARD_HAS_FM_ELNA), true)
DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := fm_ctrl.ko
LOCAL_MODULE_TAGS := optional
ifeq ($(BOARD_HAS_FM_ELNA_LDO), true)
KERNEL_CFLAGS += KCFLAGS=-DSUPPORT_FM_ELNA_LDO
KBUILD_OPTIONS += KCFLAGS=-DSUPPORT_FM_ELNA_LDO
endif
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
endif
