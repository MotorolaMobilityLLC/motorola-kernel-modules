all: modules

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) V=1 modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) V=1 INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers modules.order
