load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "support",
    srcs = [
        "grid_location.cpp",
        "grid_with_weights.cpp",
        "grid_with_weights_builder.cpp",
        "square_grid.cpp",
        "square_grid_builder.cpp",
    ],
    hdrs = [
        "draw_grid.h",
        "grid_location.h",
        "grid_with_weights.h",
        "grid_with_weights_builder.h",
        "simple_graph.h",
        "simple_graph_builder.h",
        "square_grid.h",
        "square_grid_builder.h",
        "test_data.h",
    ],
    visibility = ["//planning/path_planning/test:__subpackages__"],
    deps = [
        "@nholthaus//:units",
    ],
)
