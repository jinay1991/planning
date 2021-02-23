load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "argument_parser",
    srcs = ["argument_parser.cpp"],
    hdrs = [
        "argument_parser.h",
        "cli_options.h",
        "i_argument_parser.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//planning/common/logging"],
)
