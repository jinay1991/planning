load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = ["chrono_timer_tests.cpp"],
    tags = ["unit"],
    deps = [
        "//planning/common/timer",
        "@googletest//:gtest_main",
    ],
)
