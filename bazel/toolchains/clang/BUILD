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

clang_toolchain_config(name = "clang_toolchain_config")

cc_toolchain(
    name = "clang_toolchain",
    all_files = ":compiler_files",
    ar_files = ":compiler_files",
    compiler_files = ":compiler_files",
    dwp_files = ":empty",
    linker_files = ":compiler_files",
    objcopy_files = ":empty",
    strip_files = ":empty",
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
