load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "support",
    hdrs = [
        "astar.h",
        "breadth_first_search.h",
        "dijkstra.h",
    ],
    deps = [
        "@nholthaus//:units",
    ],
)

cc_library(
    name = "path_planning",
    srcs = glob(["*.cpp"]),
    visibility = ["//visibility:public"],
    deps = [
        ":support",
        "//planning/datatypes",
    ],
)
