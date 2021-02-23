load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "motion_planning",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "//planning/common",
        "//planning/datatypes",
        "@nholthaus//:units",
        "@spline",
    ],
)
