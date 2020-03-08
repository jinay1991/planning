load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "argument_parser_tests",
    srcs = glob(["*.cpp"]),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    deps = [
        "//planning/common/argument_parser",
        "@googletest//:gtest_main",
    ],
)
