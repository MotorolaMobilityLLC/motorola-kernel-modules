# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for Synaptics TouchComm touchscreen driver.
#

NAME=

DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

TCM_CORE=tcm/
TCM_TESTING_LIB=testing/

ccflags-y += -I$(DIR)
ccflags-y += -I$(DIR)$(TCM_CORE)
ccflags-y += -I$(DIR)$(TCM_TESTING_LIB)


synaptics_tcm2$(NAME)-objs := syna_tcm2.o
synaptics_tcm2$(NAME)-objs += syna_tcm2_cdev.o
synaptics_tcm2$(NAME)-objs += \
 					$(TCM_CORE)synaptics_touchcom_func_base.o \
 					$(TCM_CORE)synaptics_touchcom_func_touch.o

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_TOUCHCOMM_VERSION_1),y)
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_core_v1.o
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_TOUCHCOMM_VERSION_2),y)
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_core_v2.o
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH),y)
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_image_parsing.o
ifeq (${CONFIG_TOUCHSCREEN_SYNA_TCM2_TDDI},y)
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_func_reflash_tddi.o
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_ihex_parsing.o
else
	synaptics_tcm2$(NAME)-objs += $(TCM_CORE)synaptics_touchcom_func_reflash.o
endif
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_I2C),y)
	synaptics_tcm2$(NAME)-objs += syna_tcm2_platform_i2c.o
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_SPI),y)
	synaptics_tcm2$(NAME)-objs += syna_tcm2_platform_spi.o
endif

	synaptics_tcm2$(NAME)-objs += syna_tcm2_sysfs.o

ifeq ($(CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING),y)
	synaptics_tcm2$(NAME)-objs += syna_tcm2_testing.o
	synaptics_tcm2$(NAME)-objs += $(TCM_TESTING_LIB)synaptics_touchcom_testing_0001_build_id.o
	synaptics_tcm2$(NAME)-objs += $(TCM_TESTING_LIB)synaptics_touchcom_testing_0002_config_id.o
	synaptics_tcm2$(NAME)-objs += $(TCM_TESTING_LIB)synaptics_touchcom_testing_0100_trxtrx_short.o
	synaptics_tcm2$(NAME)-objs += $(TCM_TESTING_LIB)synaptics_touchcom_testing_0500_full_raw.o
	synaptics_tcm2$(NAME)-objs += $(TCM_TESTING_LIB)synaptics_touchcom_testing_0A00_noise.o
endif

obj-$(CONFIG_TOUCHSCREEN_SYNA_TCM2) += synaptics_tcm2$(NAME).o

