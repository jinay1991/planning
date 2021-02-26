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

cc_test(
    name = "fuzzer_tests",
    srcs = ["chrono_timer_fuzzer_tests.cpp"],
    tags = [
        "fuzzer",
        "manual",
        "unit",
    ],
    deps = [
        "//planning/common",
    ],
)
