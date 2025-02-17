all: modules

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions

KBUILD_EXTRA_SYMBOLS += $(CURDIR)/$(KBUILD_EXTMOD)/../../mmi_relay/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(CURDIR)/$(KBUILD_EXTMOD)/../../mmi_info/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(CURDIR)/$(KBUILD_EXTMOD)/../bm_adsp_ulog/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(CURDIR)/$(KBUILD_EXTMOD)/qti_glink_charger_v2/$(GKI_OBJ_MODULE_DIR)/Module.symvers
ifneq ($(filter m y,$(CONFIG_MMI_PHONE_CASE_SUPPORT)),)
    KBUILD_EXTRA_SYMBOLS += $(CURDIR)/$(KBUILD_EXTMOD)/../../misc/hall/$(GKI_OBJ_MODULE_DIR)/Module.symvers
endif
