cc_library(
    name = "motion_planning",
    srcs = glob([
        "src/**/*.cpp",
        "include/**/*.h",
    ]),
    hdrs = glob([
        "include/**/*.h",
    ]),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["include"],
    tags = ["lib"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@nholthaus//:units",
        "@spline",
    ],
)

cc_test(
    name = "motion_planning_tests",
    srcs = glob(
        [
            "test/**/*.cpp",
            "test/**/*.h",
        ],
    ),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["test"],
    tags = ["test"],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning",
        "@googletest//:gtest_main",
    ],
)
