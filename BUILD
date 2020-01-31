cc_library(
    name = "simulation",
    srcs = glob(
        ["src/**/*.cpp"],
        exclude = ["src/main.cpp"],
    ),
    hdrs = glob([
        "include/**/*.h",
    ]),
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["include"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//lib:motion_planning",
        "@eigen",
        "@nlohmann//:json",
        "@spline",
        "@uwebsocket",
    ],
)

cc_binary(
    name = "simulation_client",
    srcs = ["src/main.cpp"],
    copts = [
        "-std=c++14",
        "-Wall",
        "-Werror",
    ],
    includes = ["include"],
    linkstatic = True,
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":simulation",
    ],
)
