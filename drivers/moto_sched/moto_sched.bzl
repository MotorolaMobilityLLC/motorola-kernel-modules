# Load the custom rule definition, similar to the provided sample.
load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")
load("//soc-repo:moto_product.bzl", "build_target", "build_variant")

# Define the kernel module target.
# The 'name' is inferred from your source files (e.g., msched_main.c).
def define_modules():
    moto_ddk_module(
        name = "moto_sched",

        # Unconditional source files that are always included.
        srcs = [
            "msched_main.c",
            "msched_common.c",
            "msched_common.h",
            "msched_sysfs.c",
            "msched_sysfs.h",
            "msched_oemdata.c",
            "msched_oemdata.h",
            "locking/locking_main.c",
            "locking/locking_main.h",
            "locking/locking_trace.c",
            "locking/locking_trace.h",
        ],

        # Use the 'conditional_srcs' attribute to control which files are included
        # based on the kernel's Kconfig flags.
        conditional_srcs = {
            "CONFIG_MOTO_MUTEX_INHERIT": {
                True: ["locking/mutex.c"],
            },
            "CONFIG_MOTO_RWSEM_INHERIT": {
                True: ["locking/rwsem.c"],
            },
            "CONFIG_MOTO_FUTEX_INHERIT": {
                True: ["locking/futex.c"],
            },
            "CONFIG_MOTO_ENABLE_MDPF": {
                True: [
                    "msched_trace.h",
                    "msched_trace.c",
                    "msched_uclamp.c",
                    "msched_uclamp.h",
                    "mdpf/mdpf_sysfs.c",
                    "mdpf/mdpf_sysfs.h",
                    "//common:kernel/sched/autogroup.h",
                ],
            }
        },

        includes = [
                    ".",
                ],

        # You will need to add the external dependencies for your module here.
        # These are often found in your Android.mk or Kconfig files.
        # The paths should point to other Bazel targets.
        # For example: "//path/to/dependency:target_name"
        deps_ext = [
            "//soc-repo:kernel/sched/walt/sched-walt",
        ],
    )
