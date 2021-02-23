load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "support",
    testonly = True,
    hdrs = [
        "data_source_builder.h",
        "sensor_fusion_builder.h",
        "trajectory_builder.h",
    ],
    visibility = ["//planning/motion_planning:__subpackages__"],
    deps = [
        "//planning/datatypes",
        "//planning/motion_planning",
        "@nholthaus//:units",
    ],
)
