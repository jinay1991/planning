load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    copts = [
        "-Wall",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@glog",
    ],
)
