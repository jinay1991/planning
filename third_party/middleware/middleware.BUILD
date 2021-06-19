load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "middleware",
    visibility = ["//visibility:public"],
    deps = [
        ":communication",
        ":lifecycle",
    ],
)

cc_library(
    name = "lifecycle",
    srcs = [
        "lib/liblifecycle.a",
        "lib/liblifecycle.so",
    ],
    hdrs = glob([
        "include/middleware/lifecycle/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "support",
    testonly = True,
    hdrs = glob([
        "include/middleware/lifecycle/test/support/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "communication",
    srcs = [
        "lib/libcommunication.a",
        "lib/libcommunication.so",
    ],
    hdrs = glob([
        "include/middleware/communication/**/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
