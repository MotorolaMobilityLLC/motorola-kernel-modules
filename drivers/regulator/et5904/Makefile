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

ifneq ($(filter m y, $(WL2868C_ET5904_ORDER_EN)),)
KBUILD_EXTRA_SYMBOLS += $(M)/../wl2868c/$(GKI_OBJ_MODULE_DIR)/Module.symvers
endif

