load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "logging_tests",
    srcs = glob(["*.cpp"]),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    deps = [
        "//planning/common/logging",
        "@googletest//:gtest_main",
    ],
)
