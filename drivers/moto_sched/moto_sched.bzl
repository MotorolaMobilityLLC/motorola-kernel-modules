# Load the custom rule definition, similar to the provided sample.
load("//soc-repo:kleaf-scripts/moto/moto_modules_define.bzl", "moto_ddk_module")
load("//soc-repo:moto_product.bzl", "build_target", "build_variant")

# Mapping of SoC/build targets to their respective kernel versions.
# Art platform (e.g., SM8975) uses 6.18; Canoe platform (e.g., SM8850) uses 6.12.
_TARGET_KERNEL_VERSION_MAP = {
    "art": "6.18",
    "canoe": "6.12",
}

# Define the kernel module target.
# The 'name' is inferred from your source files (e.g., msched_main.c).
def define_modules():
    # Determine the kernel version based on build_target
    kernel_version = _TARGET_KERNEL_VERSION_MAP.get(build_target, "6.12")

    # Parse kernel version major and minor safely, stripping potential suffixes
    _parts = kernel_version.split(".")
    _major = int(_parts[0])
    _minor_str = _parts[1].split("-")[0].split("+")[0] if len(_parts) > 1 else "0"
    _minor = int(_minor_str) if _minor_str.isdigit() else 0

    is_6_18_or_above = (_major > 6) or (_major == 6 and _minor >= 18)

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
            "msched_trace.h",
            "msched_trace.c",
	    "msched_boost.c",
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
                    "msched_uclamp.c",
                    "msched_uclamp.h",
                    "mdpf/mdpf_sysfs.c",
                    "mdpf/mdpf_sysfs.h",
                    "//common:kernel/sched/autogroup.h",
                ],
            }
        },

        local_defines = [] if is_6_18_or_above else ["ENABLE_INHERITE=1"],

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
