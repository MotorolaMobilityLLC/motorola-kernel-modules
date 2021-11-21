#inherit from MTK build_ko.mk
include $(MTK_KERNEL_MODULE)

ifneq (,$(strip $(GEN_KERNEL_BUILD_CONFIG)))
$(linked_module): OPTS += $(KERNEL_CFLAGS) ANDROID_BUILD_TOP=$$(pwd)/.. TOP=$$(pwd)/.. KERNEL_DIR_PATH=$(LINUX_KERNEL_VERSION)
else
$(linked_module): OPTS += $(KERNEL_CFLAGS) ANDROID_BUILD_TOP=$$(pwd) TOP=$$(pwd) KERNEL_DIR_PATH=$(LINUX_KERNEL_VERSION)
endif

