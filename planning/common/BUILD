load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "common",
    srcs = [
        "argument_parser.cpp",
        "chrono_timer.cpp",
    ],
    hdrs = [
        "argument_parser.h",
        "chrono_timer.h",
        "cli_options.h",
        "i_argument_parser.h",
        "i_timer.h",
        "logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@glog",
    ],
)
