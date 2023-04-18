DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DRM_PANEL_EVENT_NOTIFICATIONS),true)
	KBUILD_OPTIONS += CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS=y
endif

ifeq ($(DRM_PANEL_NOTIFICATIONS),true)
	KBUILD_OPTIONS += CONFIG_DRM_PANEL_NOTIFICATIONS=y
endif

ifeq ($(TOUCHSCREEN_GOODIX_BRL_SPI),true)
ifeq ($(call is-board-platform-in-list,taro kalama parrot), true)
	KBUILD_OPTIONS += CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI=y
else
	KERNEL_CFLAGS += CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI=y
endif
endif

ifeq ($(GTP_LIMIT_USE_SUPPLIER),true)
	KBUILD_OPTIONS += CONFIG_GTP_LIMIT_USE_SUPPLIER=y
endif

ifeq ($(GTP_ENABLE_PM_QOS),true)
	KBUILD_OPTIONS += CONFIG_GTP_ENABLE_PM_QOS=y
endif

ifeq ($(GOODIX_ESD_ENABLE),true)
	KBUILD_OPTIONS += CONFIG_GOODIX_ESD_ENABLE=y
endif

ifeq ($(findstring factory, $(TARGET_PRODUCT)), factory)
	KBUILD_OPTIONS += CONFIG_TARGET_BUILD_FACROTY=y
endif

ifeq ($(TOUCHSCREEN_FOD),true)
	KBUILD_OPTIONS += CONFIG_GTP_FOD=y
endif

ifeq ($(TOUCHSCREEN_LAST_TIME),true)
	KBUILD_OPTIONS += CONFIG_GTP_LAST_TIME=y
endif

ifeq ($(GTP_ENABLE_DDA_STYLUS),true)
	KBUILD_OPTIONS += CONFIG_GTP_DDA_STYLUS=y
endif

ifeq ($(BOARD_USES_DOUBLE_TAP_CTRL),true)
	KBUILD_OPTIONS += CONFIG_BOARD_USES_DOUBLE_TAP_CTRL=y
endif

ifneq ($(GTP_ENABLE_TOUCH_PALM),)
	KBUILD_OPTIONS += CONFIG_INPUT_GOODIX_MMI_ENABLE_PALM=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_brl_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/mmi_info.ko
ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
	KBUILD_OPTIONS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
	LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_mmi.ko
endif
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
