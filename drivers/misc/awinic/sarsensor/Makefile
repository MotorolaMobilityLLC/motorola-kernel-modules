include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

EXTRA_SYMBOLS += $(OUT_DIR)/../motorola/kernel/modules/drivers/sensors/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

