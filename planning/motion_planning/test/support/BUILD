load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "builders",
    testonly = True,
    hdrs = [
        "builders/data_source_builder.h",
        "builders/object_fusion_builder.h",
        "builders/sensor_fusion_builder.h",
        "builders/trajectory_builder.h",
    ],
    visibility = ["//planning/motion_planning/test:__subpackages__"],
    deps = [
        "//planning/datatypes",
        "//planning/motion_planning",
        "@nholthaus//:units",
    ],
)

cc_library(
    name = "support",
    testonly = True,
    hdrs = [
        "map_coordinates.h",
    ],
    visibility = ["//planning/motion_planning/test:__subpackages__"],
    deps = [
        ":builders",
        "//planning/datatypes",
        "//planning/motion_planning",
        "@nholthaus//:units",
    ],
)
