load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "node",
    srcs = ["motion_planning_node.cpp"],
    hdrs = ["motion_planning_node.h"],
    copts = [
        "-Wall",
        "-Werror",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//planning/communication:topics",
        "//planning/motion_planning",
        "@middleware//:communication",
        "@middleware//:lifecycle",
    ],
)
