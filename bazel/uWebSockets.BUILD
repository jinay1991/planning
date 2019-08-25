config_setting(
    name = "darwin",
    constraint_values = ["@bazel_tools//platforms:osx"],
)

cc_library(
    name = "libuv",
    srcs = select({
        ":darwin": ["local/Cellar/libuv/1.31.0/lib/libuv.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libuv.so"],
    }),
    includes = select({
        ":darwin": ["local/Cellar/libuv/1.31.0/include/"],
        "//conditions:default": ["include"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libz",
    srcs = select({
        ":darwin": ["local/Cellar/zlib/1.2.11/lib/libz.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libz.so"],
    }),
    includes = select({
        ":darwin": ["local/Cellar/zlib/1.2.11/include/"],
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
