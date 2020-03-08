load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "timer_tests",
    srcs = glob(["*.cpp"]),
    copts = [
        "-Wall",
        "-std=c++14",
    ],
    deps = [
        "//planning/common/timer",
        "@googletest//:gtest_main",
    ],
)
