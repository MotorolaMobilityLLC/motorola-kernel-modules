DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(GOODIX_FPS_DRM_PANEL_NOTIFICATIONS),true)
    KERNEL_CFLAGS += CONFIG_GOODIX_DRM_PANEL_NOTIFICATIONS=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_fod_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
ifneq ($(findstring mmi_relay.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    KERNEL_CFLAGS += CONFIG_MMI_RELAY=y
    LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_relay.ko
endif
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

