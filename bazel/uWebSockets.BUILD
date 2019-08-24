cc_library(
    name = "libuv",
    srcs = glob(["local/Cellar/libuv/**/lib/libuv.a"]),
    hdrs = glob(["local/Cellar/libuv/**/include/**/*.h"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libz",
    srcs = glob(["local/lib/Cellar/zlib/**/lib/libz.a"]),
    hdrs = glob(["local/Cellar/zlib/**/include/**/*.h"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libssl",
    srcs = glob(["local/opt/openssl/lib/libssl.a"]),
    hdrs = glob(["local/opt/openssl/include/**/*.h"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libuWS",
    srcs = ["local/lib/libuWS.dylib"],
    hdrs = glob(["local/include/uWS/*.h"]),
    includes = [
        "local/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":libssl",
        ":libuv",
        ":libz",
    ],
)
