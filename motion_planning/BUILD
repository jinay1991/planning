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

cc_library(
    name = "motion_planning_test_utils",
    hdrs = glob([
        "test/**/*.h",
    ]),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    strip_include_prefix = "test/include",
    tags = ["test"],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning",
    ],
)

cc_test(
    name = "motion_planning_unit_tests",
    srcs = glob(
        [
            "test/**/*.cpp",
        ],
        exclude = ["test/src/motion_planning_test.cpp"],
    ),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["test/include"],
    tags = [
        "test",
        "unit",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning",
        ":motion_planning_test_utils",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_planning_component_tests",
    srcs = [
        "test/src/motion_planning_test.cpp",
    ],
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["test/include"],
    tags = [
        "component",
        "test",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning",
        ":motion_planning_test_utils",
        "@googletest//:gtest_main",
    ],
)

# ---------
# Packaging
# ---------
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_deb", "pkg_tar")

pkg_tar(
    name = "motion_planning_bin_pkg",
    testonly = True,
    srcs = [
        ":client-app",
        "//lib:motion_planning_tests",
    ],
    extension = "tar.gz",
    package_dir = "/opt/planning/bin",
    strip_prefix = "/",
    tags = ["manual"],
)

pkg_tar(
    name = "motion_planning_lib_pkg",
    testonly = True,
    srcs = [
        "//lib:planning",
    ],
    extension = "tar.gz",
    package_dir = "/opt/planning",
    strip_prefix = "/",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "motion_planning_hdrs_pkg",
    testonly = True,
    srcs = glob([
        "lib/include/**/*.h",
    ]),
    extension = "tar.gz",
    package_dir = "/opt/planning",
    strip_prefix = "/",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "motion_planning_tar_pkg",
    testonly = True,
    extension = "tar.gz",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":motion_planning_bin_pkg",
        ":motion_planning_hdrs_pkg",
        ":motion_planning_lib_pkg",
    ],
)

pkg_deb(
    name = "motion_planning_deb_pkg",
    testonly = True,
    architecture = "amd64",
    data = ":motion_planning_tar_pkg",
    description = "Motion Planning for Automonous Vehicle Driving",
    homepage = "https://gitlab.com/jinay1991/planning",
    maintainer = "https://gitlab.com/jinay1991",
    package = "planning",
    tags = ["manual"],
    version = "0.0.2",
    visibility = ["//visibility:public"],
)
