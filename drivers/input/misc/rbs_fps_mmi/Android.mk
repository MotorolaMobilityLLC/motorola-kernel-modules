DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
ETS_FPS_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
ETS_FPS_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifeq ($(EGIS_FPS_PANEL_NOTIFICATIONS),true)
    KERNEL_CFLAGS += CONFIG_EGIS_PANEL_NOTIFICATIONS=y
endif

ifeq ($(BOARD_SUPPORT_EGIS_FOD),true)
    KERNEL_CFLAGS += CONFIG_ET721_FOD=y
endif
include $(CLEAR_VARS)
LOCAL_MODULE := rbs_fps_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(ETS_FPS_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk

