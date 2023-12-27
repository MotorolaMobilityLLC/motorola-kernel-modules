DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(TOUCHSCREEN_GOODIX_BRL_SPI),true)
ifeq ($(call is-board-platform-in-list,taro kalama parrot crow pineapple), true)
	KBUILD_OPTIONS += CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI=y
else
	KERNEL_CFLAGS += CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI=y
endif
endif

ifeq ($(DRM_PANEL_EVENT_NOTIFICATIONS),true)
    KBUILD_OPTIONS += CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS=y
endif

ifeq ($(GTP_LIMIT_USE_SUPPLIER),true)
    KBUILD_OPTIONS += CONFIG_GTP_LIMIT_USE_SUPPLIER=y
endif

ifeq ($(TOUCHSCREEN_LAST_TIME),true)
    KBUILD_OPTIONS += CONFIG_GTP_LAST_TIME=y
endif

ifeq ($(ENABLE_GTP_PALM_CANCEL),true)
    KBUILD_OPTIONS += CONFIG_ENABLE_GTP_PALM_CANCEL=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_gt96x_u_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
ifneq ($(findstring touchscreen_u_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
	KBUILD_OPTIONS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
	LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_u_mmi.ko
endif

KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
