DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_HAS_MULTI_FPS),true)
	KERNEL_CFLAGS += CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE=y
endif

ifeq ($(FPC_TEE_BOOST),true)
        KERNEL_CFLAGS += FPC_TEE_BOOST=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := fpc_mtk_tee.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

