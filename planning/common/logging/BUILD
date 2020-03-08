load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "logging",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    visibility = ["//visibility:public"],
)
