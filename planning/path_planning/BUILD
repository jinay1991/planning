load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "path_planning",
    srcs = [
        "astar.h",
        "breadth_first_search.h",
        "dijkstra.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//planning/datatypes",
        "@nholthaus//:units",
    ],
)
