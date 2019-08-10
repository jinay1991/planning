cc_library(
    name = "motion_planning",
    srcs = glob([
        "src/**/*.cpp",
    ]),
    hdrs = glob([
        "include/**/*.h",
    ]),
    includes = ["include"],
    tags = ["lib"],
    visibility = [
        "//visibility:public",
    ],
)

cc_test(
    name = "unit_tests",
    srcs = glob([
        "test/**/*.cpp",
        # "test/**/*.h",
    ]),
    # includes = ["test"],
    # tags = ["test"],
    deps = [
        ":motion_planning",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
