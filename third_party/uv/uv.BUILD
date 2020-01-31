package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uv",
    srcs = select({
        "@//bazel/platforms:macos": ["local/opt/libuv/lib/libuv.dylib"],
        "//conditions:default": ["lib/x86_64-linux-gnu/libuv.so"],
    }),
    hdrs = select({
        "@//bazel/platforms:macos": glob(["local/opt/libuv/include/**/*.h"]),
        "//conditions:default": glob(["include/openssl/*.h"]),
    }),
    includes = select({
        "@//bazel/platforms:macos": ["local/opt/libuv/include"],
        "//conditions:default": ["include"],
    }),
)
