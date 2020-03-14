load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "support",
    srcs = [],
    hdrs = [
        "data_source_builder.h",
        "sensor_fusion_builder.h",
        "trajectory_builder.h",
    ],
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    deps = [
        "//planning/motion_planning",
    ],
)

cc_test(
    name = "motion_planning_tests",
    srcs = glob(
        ["*.cpp"],
        exclude = ["motion_planning_test.cpp"],
    ),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    tags = ["unit"],
    deps = [
        ":support",
        "//planning/motion_planning",
        "@googletest//:gtest_main",
        "@nholthaus//:units",
    ],
)

cc_test(
    name = "motion_planning_component_tests",
    srcs = ["motion_planning_test.cpp"],
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    tags = ["component"],
    deps = [
        ":support",
        "//planning/motion_planning",
        "@googletest//:gtest_main",
    ],
)
