load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")
load("//soc-repo:moto_product.bzl", "build_target", "build_variant")

def define_modules():
    deps_list = [
        "//soc-repo:mm/zsmalloc",
    ]

    if build_target == "canoe":
        deps_list += [
            "//soc-repo:drivers/soc/qcom/qpace/qpace_drv",
        ]

    moto_ddk_module(
        name = "moto_swap",
        srcs = ([
            # do not sort
            # TODO: Should not hardcode zram-6.12, subsequent fix.
            "zram-6.12/zcomp.c",
            "zram-6.12/zram_drv.c",
            "zram-6.12/zcomp.h",
            "zram-6.12/zram_drv.h",
        ]),
        conditional_srcs = {
            "CONFIG_QTI_PAGE_COMPRESSION_ENGINE": {
                True: [
                    "//soc-repo:drivers/soc/qcom/qpace/qpace.h",
                ],
            },
        },
        deps_ext = deps_list,
    )
