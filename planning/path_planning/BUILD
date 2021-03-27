load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "path_planning",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "//planning/datatypes",
        "@nholthaus//:units",
    ],
)
