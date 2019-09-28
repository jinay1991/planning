config_setting(
    name = "debug_build",
    values = {
        "compilation_mode": "dbg",
    },
)

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
    defines = select({
        ":debug_build": ["NDEBUG"],
        "//conditions:default": [],
    }),
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
    srcs = glob([
        "test/**/*.cpp",
        "test/**/*.h",
    ]),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["test/include"],
    tags = ["test"],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning",
        "@googletest//:gtest_main",
    ],
)
