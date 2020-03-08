load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spline",
    hdrs = ["src/spline.h"],
    copts = [
        "-Wno-error=unused-function",
    ],
    linkstatic = False,
    strip_include_prefix = "src",
)
