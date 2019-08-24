cc_library(
    name = "ssl",
    srcs = ["local/opt/openssl/lib/libssl.so"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "uWS",
    srcs = ["libuWS.dylib"],
    includes = [
        "local/include",
        "local/opt/openssl/include",
    ],
    linkopts = [
        "-luv",
        "-lz",
        "-lssl",
    ],
    visibility = ["//visibility:public"],
    deps = [
    ],
)
