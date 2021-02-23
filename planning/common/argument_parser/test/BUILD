load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = ["argument_parser_tests.cpp"],
    deps = [
        "//planning/common/argument_parser",
        "@googletest//:gtest_main",
    ],
)
