load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def uwebsocket():
    if "uwebsocket" not in native.existing_rules():
        new_git_repository(
            name = "uwebsocket",
            build_file = "//third_party/uwebsocket:uwebsocket.BUILD",
            commit = "e94b6e1",
            remote = "https://github.com/uNetworking/uWebSockets.git",
        )
