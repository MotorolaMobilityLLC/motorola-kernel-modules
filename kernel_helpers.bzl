# kernel_helpers.bzl

def get_kernel_headers():
    headers = []
    headers.append("//motorola/kernel/modules:public_headers")
    if native.existing_rule("//kernel_device_modules-6.6:mtk_public_headers"):
        headers.append("//kernel_device_modules-6.6:mtk_public_headers")
    elif native.existing_rule("//kernel_device_modules-6.1:mtk_public_headers"):
        headers.append("//kernel_device_modules-6.1:mtk_public_headers")
    return headers
