load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = glob(["*.cpp"]),
    deps = [
        "//planning/path_planning",
        "@googletest//:gtest_main",
    ],
)
