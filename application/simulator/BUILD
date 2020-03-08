load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "simulator",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    copts = [
        "-std=c++14",
        "-Wall",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//planning/common/argument_parser",
        "//planning/common/logging",
        "//planning/motion_planning",
        "@eigen",
        "@nlohmann//:json",
        "@spline",
        "@uwebsocket",
    ],
)
