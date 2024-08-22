DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := aw862x_haptic_nv.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(TARGET_PRODUCT), kobe)
    LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/camera.ko
    KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(shell pwd)/$(PRODUCT_OUT)/obj/DLKM_OBJ/vendor/qcom/opensource/camera-kernel/Module.symvers
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif
include $(DLKM_DIR)/AndroidKernelModule.mk
