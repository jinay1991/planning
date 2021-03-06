load("@rules_cc//cc:defs.bzl", "cc_toolchain", "cc_toolchain_suite")
load(":clang_toolchain_config.bzl", "clang_toolchain_config")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
    name = "sysroot",
    srcs = [
        "@sysroot//:kernel_headers",
        "@sysroot//:libc",
        "@sysroot//:libcrypt",
        "@sysroot//:libgcc",
        "@sysroot//:libstdc++",
    ],
)

filegroup(
    name = "linker_files",
    srcs = [
        "//bazel/toolchains/clang/bin:ar",
        "//bazel/toolchains/clang/bin:lcov",
        "//bazel/toolchains/clang/bin:lld",
    ],
)

filegroup(
    name = "compiler_files",
    srcs = [
        ":sysroot",
        "//bazel/toolchains/clang/bin:ar",
        "//bazel/toolchains/clang/bin:cc",
        "//bazel/toolchains/clang/bin:cpp",
        "//bazel/toolchains/clang/bin:lcov",
        "//bazel/toolchains/clang/bin:nm",
        "@llvm_toolchain//:compiler_pieces",
    ],
)

filegroup(
    name = "all_files",
    srcs = [
        ":compiler_files",
        ":linker_files",
        ":sysroot",
    ],
)

clang_toolchain_config(name = "clang_toolchain_config")

cc_toolchain(
    name = "clang_toolchain",
    all_files = ":compiler_files",
    ar_files = ":compiler_files",
    compiler_files = ":compiler_files",
    dwp_files = ":empty",
    linker_files = ":compiler_files",
    objcopy_files = ":compiler_files",
    strip_files = ":compiler_files",
    supports_param_files = 0,
    toolchain_config = ":clang_toolchain_config",
    toolchain_identifier = "clang-linux",
)

cc_toolchain_suite(
    name = "clang",
    toolchains = {
        "k8": "//bazel/toolchains/clang:clang_toolchain",
        "k8|clang": "//bazel/toolchains/clang:clang_toolchain",
    },
)
