load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "argument_parser",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    visibility = ["//visibility:public"],
    deps = ["//planning/common/logging"],
)
