load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

new_git_repository(
    name = "googletest",
    build_file = "//bazel:gtest.BUILD",
    remote = "https://github.com/google/googletest",
    tag = "release-1.8.1",
)

new_git_repository(
    name = "nholthaus",
    build_file = "//bazel:nholthaus.BUILD",
    remote = "https://github.com/nholthaus/units",
    tag = "v2.3.1",
)

new_git_repository(
    name = "nlohmann",
    build_file = "//bazel:nlohmann.BUILD",
    remote = "https://github.com/nlohmann/json",
    tag = "v3.7.0",
)
