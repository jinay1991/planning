load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "support",
    testonly = True,
    srcs = [
        "motion_planning_consumer_node.cpp",
        "motion_planning_simulator_node.cpp",
    ],
    hdrs = [
        "data_source_builder.h",
        "map_coordinates.h",
        "motion_planning_consumer_node.h",
        "motion_planning_simulator_node.h",
        "object_fusion_builder.h",
        "sensor_fusion_builder.h",
        "trajectory_builder.h",
    ],
    visibility = ["//planning/motion_planning/test:__subpackages__"],
    deps = [
        "//planning/communication:topics",
        "//planning/datatypes",
        "//planning/motion_planning",
        "@middleware//:communication",
        "@middleware//:lifecycle",
        "@nholthaus//:units",
    ],
)
