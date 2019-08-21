cc_library(
    name = "gtest",
    srcs = glob(
        ["src/*.cc"],
        exclude = [
            "src/gtest-all.cc",
            "src/gtest-main.cc",
        ],
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h",
    ]),
    copts = [
        "-Iexternal/gtest/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtest_main",
    srcs = glob(
        ["src/gtest_main.cc"],
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h",
    ]),
    copts = [
        "-Iexternal/gtest/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gtest",
    ],
)
