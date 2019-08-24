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
        "@nlohmann//:json",
        "@uWebSockets//:uWS",
    ],
)

cc_binary(
    name = "client",
    srcs = ["src/main.cpp"],
    copts = ["-std=c++14"],
    includes = ["include"],
    tags = ["bin"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":simulation",
    ],
)
