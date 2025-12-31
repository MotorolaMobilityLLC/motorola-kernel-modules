load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")
load("//soc-repo:moto_product.bzl", "build_target", "build_variant")

def define_modules():
    cflags = []
    deps_list = [
        "//soc-repo:mm/zsmalloc",
    ]

    if build_target == "canoe":
        deps_list += [
            "//soc-repo:drivers/soc/qcom/qpace/qpace_drv",
        ]

    src_hybrid = ([
        "hybridswap/hybridswap_main.c",
        "hybridswap/hybridswap_swapd.c",
        "hybridswap/hybridswap_eswap.c",
        "hybridswap/hybridswap.h",
        "hybridswap/hybridswap_internal.h",
    ])

    cflags.append("-Wframe-larger-than=4096");

    moto_ddk_module(
        name = "moto_swap",
        srcs = src_hybrid,
        conditional_srcs = {
            "CONFIG_QTI_PAGE_COMPRESSION_ENGINE": {
                True: [
                    "//soc-repo:drivers/soc/qcom/qpace/qpace.h",
                ],
            },
            "CONFIG_MOTO_SWAP_KERNEL_FLAG": {
                True: [
                    "zram-6.12/zcomp.c",
                    "zram-6.12/zram_drv.c",
                    "zram-6.12/zcomp.h",
                    "zram-6.12/zram_drv.h",
                    "zram-6.12/zram_drv_internal.h",
                    "zram-6.12/backend_lz4.c",
                    "zram-6.12/backend_lz4.h",
                    "zram-6.12/backend_lzo.c",
                    "zram-6.12/backend_lzo.h",
                    "zram-6.12/backend_lzorle.c",
                    "zram-6.12/backend_lzorle.h",
               ],
            },
        },
        copts = cflags,
        deps_ext = deps_list,
    )
