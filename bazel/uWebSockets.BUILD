load("@rules_cc//cc:defs.bzl", "cc_library")

config_setting(
    name = "darwin",
    constraint_values = ["@bazel_tools//platforms:osx"],
)

cc_library(
    name = "libuv",
    srcs = select({
        ":darwin": ["local/lib/libuv.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libuv.so"],
    }),
    hdrs = select({
        ":darwin": glob(["local/include/**/*.h"]),
        "//conditions:default": glob(["include/**/*.h"]),
    }),
    includes = select({
        ":darwin": ["local/include/"],
        "//conditions:default": ["include"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libz",
    srcs = select({
        ":darwin": ["lib/libz.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libz.so"],
    }),
    hdrs = select({
        ":darwin": glob(["include/**/*.h"]),
        "//conditions:default": glob(["include/*.h"]),
    }),
    includes = select({
        ":darwin": ["include/"],
        "//conditions:default": ["include"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libssl",
    srcs = select({
        ":darwin": ["local/opt/openssl/lib/libssl.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libssl.so"],
    }),
    hdrs = select({
        ":darwin": glob(["local/opt/openssl/include/**/*.h"]),
        "//conditions:default": glob(["include/openssl/*.h"]),
    }),
    includes = select({
        ":darwin": ["local/opt/openssl/include"],
        "//conditions:default": ["include"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libuWS",
    srcs = select({
        ":darwin": ["local/lib/libuWS.dylib"],
        "//conditions:default": ["lib/libuWS.so"],
    }),
    includes = select({
        ":darwin": ["local/include"],
        "//conditions:default": ["include"],
    }),
    visibility = ["//visibility:public"],
    deps = [
        ":libssl",
        ":libuv",
        ":libz",
    ],
)
