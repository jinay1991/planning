cc_library(
    name = "planning-lib",
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
        "//motion_planning",
        "@eigen",
        "@nlohmann//:json",
        "@spline",
        "@uwebsocket",
    ],
)

cc_binary(
    name = "planning-app",
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
        ":planning-lib",
    ],
)
