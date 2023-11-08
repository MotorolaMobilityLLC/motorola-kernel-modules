include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

KBUILD_EXTRA_SYMBOLS += $(M)/../../../mmi_info/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(M)/../touchscreen_mmi/$(GKI_OBJ_MODULE_DIR)/Module.symvers
