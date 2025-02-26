include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

EXTRA_SYMBOLS += $(OUT_DIR)/../motorola/kernel/modules/drivers/sensors/Module.symvers
ifneq ((filter m y,$(CONFIG_CAPSENSE_HALL_CAL)),)
EXTRA_SYMBOLS += $(OUT_DIR)/../motorola/kernel/modules/drivers/misc/hall/Module.symvers
endif

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

