load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = ["astar_tests.cpp"],
    deps = [
        "//planning/path_planning",
        "//planning/path_planning/test/support:astar",
        "@googletest//:gtest_main",
    ],
)
