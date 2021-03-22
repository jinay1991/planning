load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "path_planning",
    srcs = [
        "astar_search_algorithm.cpp",
        "astar_search_algorithm.h",
        "grid.cpp",
        "node.cpp",
        "node.h",
    ],
    hdrs = [
        "grid.h",
        "i_node.h",
        "i_search_algorithm.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//planning/datatypes",
        "@nholthaus//:units",
    ],
)
