load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_deb", "pkg_tar")

cc_library(
    name = "simulation",
    srcs = glob(
        include = ["src/**/*.cpp"],
        exclude = ["src/main.cpp"],
    ),
    hdrs = glob([
        "include/**/*.h",
    ]),
    copts = [
        "-std=c++14",
    ],
    includes = ["include"],
    tags = ["lib"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@eigen3",
        "@nlohmann//:json",
        "@spline",
        "@uWebSockets//:libuWS",
    ],
)

cc_binary(
    name = "client-app",
    srcs = ["src/main.cpp"],
    copts = ["-std=c++14"],
    includes = ["include"],
    linkstatic = True,
    tags = ["bin"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":simulation",
    ],
)

pkg_tar(
    name = "motion_planning_bin_pkg",
    testonly = True,
    srcs = [
        ":client-app",
        "//lib:motion_planning_tests",
    ],
    extension = "tar.gz",
    package_dir = "/opt/motion_planning/bin",
    strip_prefix = "/",
)

pkg_tar(
    name = "motion_planning_lib_pkg",
    testonly = True,
    srcs = [
        "//lib:motion_planning",
    ],
    extension = "tar.gz",
    package_dir = "/opt/motion_planning/lib",
    strip_prefix = "/",
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "motion_planning_hdrs_pkg",
    testonly = True,
    srcs = glob([
        "lib/include/**/*.h",
    ]),
    extension = "tar.gz",
    package_dir = "/opt/motion_planning",
    strip_prefix = "/",
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "motion_planning_tar_pkg",
    testonly = True,
    extension = "tar.gz",
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
    homepage = "https://gitlab.com/jinay1991/motion_planning",
    maintainer = "https://gitlab.com/jinay1991",
    package = "motion_planning",
    version = "0.0.1",
    visibility = ["//visibility:public"],
)
