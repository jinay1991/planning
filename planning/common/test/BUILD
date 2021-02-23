load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = [
        "argument_parser_tests.cpp",
        "chrono_timer_tests.cpp",
        "logging_tests.cpp",
    ],
    tags = ["unit"],
    deps = [
        "//planning/common",
        "@googletest//:gtest_main",
    ],
)
