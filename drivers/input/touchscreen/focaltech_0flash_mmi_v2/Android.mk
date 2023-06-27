DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(FOCALTECH_TOUCH_IC_NAME),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_IC_NAME=$(FOCALTECH_TOUCH_IC_NAME)
else
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_IC_NAME=ft8725
endif

ifneq ($(BOARD_USES_DOUBLE_TAP),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_0FLASH_MMI_ENABLE_DOUBLE_TAP=y
endif

ifneq ($(FOCALTECH_ESDCHECK_ENABLE),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_ESD_EN=y
endif

ifneq ($(FOCALTECH_DEBUG_ENABLE),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_DEBUG_EN=y
endif

ifeq ($(CONFIG_INPUT_HIGH_RESOLUTION_4),true)
        KERNEL_CFLAGS += CONFIG_INPUT_HIGH_RESOLUTION_4=y
endif

ifneq ($(BOARD_USES_PANEL_NOTIFICATIONS),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_PANEL_NOTIFICATIONS=y
endif

ifneq ($(BOARD_USES_MTK_CHECK_PANEL),)
	KERNEL_CFLAGS += CONFIG_FTS_MTK_CHECK_PANEL=y
endif

ifneq ($(FOCALTECH_MULTI_IC_ENABLE),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_MULTI_IC_EN=y
endif

ifneq ($(FOCALTECH_MULTI_FT87XX),)
	KERNEL_CFLAGS += CONFIG_INPUT_FOCALTECH_MULTI_FT87XX=y
endif

ifeq ($(FT_FHD_MMI_GET_PANEL),true)
	KERNEL_CFLAGS += CONFIG_FT_FHD_MMI_GET_PANEL=y
endif

ifeq ($(MTK_PANEL_NOTIFICATIONS),true)
	KERNEL_CFLAGS += CONFIG_MTK_PANEL_NOTIFICATIONS=y
endif

ifeq ($(TOUCHSCREEN_LAST_TIME),true)
	KERNEL_CFLAGS += CONFIG_GTP_LAST_TIME=y
endif

ifeq ($(BOARD_USES_DOUBLE_TAP_CTRL),true)
	KERNEL_CFLAGS += CONFIG_BOARD_USES_DOUBLE_TAP_CTRL=y
endif

ifneq ($(CONFIG_FOCALTECH_MTK_SPI),)
	KERNEL_CFLAGS += CONFIG_FOCALTECH_MTK_SPI=y
endif

include $(CLEAR_VARS)
ifneq ($(BOARD_USES_DOUBLE_TAP),)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/sensors_class.ko
endif
LOCAL_MODULE := focaltech_0flash_mmi_v2.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/

include $(DLKM_DIR)/AndroidKernelModule.mk
