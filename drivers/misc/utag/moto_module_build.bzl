load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")

def define_moto_module_build():

    moto_ddk_module(
        name = "utags",
        srcs = native.glob([
            "drivers/misc/utag/*.c",
            "drivers/misc/utag/*.h",
        ]),
    )
