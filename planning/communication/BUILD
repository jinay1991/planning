load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "topics",
    hdrs = ["topics.h"],
    copts = [
        "-Wall",
        "-Werror",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@middleware//:communication",
    ],
)
