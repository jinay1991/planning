load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "simulator_client",
    srcs = ["main.cpp"],
    copts = [
        "-std=c++14",
        "-Wall",
    ],
    data = ["//:testdata"],
    includes = ["include"],
    deps = [
        "//application/simulator",
        "//planning/common",
    ],
)
