include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

EXTRA_SYMBOLS += $(OUT_DIR)/../motorola/kernel/modules/drivers/mmi_info/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/../motorola/kernel/modules/drivers/input/touchscreen/touchscreen_u_mmi/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

KBUILD_EXTRA_SYMBOLS += $(M)/../../../mmi_info/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(M)/../touchscreen_u_mmi/$(GKI_OBJ_MODULE_DIR)/Module.symvers
