include $(MTK_KERNEL_MODULE)
$(linked_module): OPTS += $(KERNEL_CFLAGS) ANDROID_BUILD_TOP=$$(pwd) TOP=$$(pwd)

