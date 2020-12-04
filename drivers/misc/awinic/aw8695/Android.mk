DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(AF_NOISE_ELIMINATION_ENABLE), true)
	KERNEL_CFLAGS += CONFIG_AF_NOISE_ELIMINATION=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := aw8695.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

include $(DLKM_DIR)/AndroidKernelModule.mk
