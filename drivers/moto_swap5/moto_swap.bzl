load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")
load("//soc-repo:moto_product.bzl", "build_target", "build_variant")

def define_modules():
    cflags = [
    	"-DCONFIG_VENDOR_ZRAM_WRITEBACK",
    	"-DCONFIG_ZRAM_EXT",
    	]
    deps_list = [
        "//soc-repo:mm/zsmalloc",
    ]

    if build_target == "canoe":
        deps_list += [

        ]

    src_hybrid = ([
        
    ])

    cflags.append("-Wframe-larger-than=4096");

    moto_ddk_module(
        name = "moto_swap5",
        srcs = src_hybrid,
        conditional_srcs = {

            "CONFIG_MOTO_SWAP_KERNEL_FLAG": {
                True: [
                    "zram-6.12/zcomp.c",
                    "zram-6.12/zram_drv.c",
                    "zram-6.12/zcomp.h",
                    "zram-6.12/zram_drv.h",
                    "zram-6.12/zram_drv_internal.h",
		    "zram-6.12/zram_ext.c",
		    "zram-6.12/zram_ext.h",
		    "zram-6.12/madvise.c",
		    "zram-6.12/backend_deflate.c",
		    "zram-6.12/backend_lz4.c",
		    "zram-6.12/backend_lz4hc.c",
		    "zram-6.12/backend_lzo.c",
		    "zram-6.12/backend_lzorle.c",
		    "zram-6.12/backend_zstd.c",		    		    
		    "zram-6.12/backend_deflate.h",
		    "zram-6.12/backend_lz4.h",
		    "zram-6.12/backend_lz4hc.h",
		    "zram-6.12/backend_lzo.h",
		    "zram-6.12/backend_lzorle.h",
		    "zram-6.12/backend_zstd.h",			    
                ],
            },
        },
        copts = cflags,
        deps_ext = deps_list,
    )
