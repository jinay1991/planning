load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "support",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "@nholthaus//:units",
    ],
)
