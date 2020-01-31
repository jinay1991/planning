package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uwebsocket",
    srcs = [
        "src/Extensions.cpp",
        "src/Group.cpp",
        "src/HTTPSocket.cpp",
        "src/Hub.cpp",
        "src/Networking.cpp",
        "src/Node.cpp",
        "src/Socket.cpp",
        "src/WebSocket.cpp",
        "src/WebSocketImpl.cpp",
        "src/uUV.cpp",
    ],
    hdrs = [
        "src/Extensions.h",
        "src/Group.h",
        "src/HTTPSocket.h",
        "src/Hub.h",
        "src/Networking.h",
        "src/Node.h",
        "src/Socket.h",
        "src/WebSocket.h",
        "src/WebSocketProtocol.h",
        "src/uUV.h",
        "src/uWS.h",
    ],
    copts = [
        "-std=c++11",
    ],
    includes = ["src"],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "@openssl//:crypto",
        "@openssl//:ssl",
        "@uv",
        "@zlib",
    ],
)
