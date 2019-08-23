load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

new_git_repository(
    name = "googletest",
    build_file = "//bazel:gtest.BUILD",
    remote = "https://github.com/google/googletest",
    tag = "release-1.8.1",
)

new_git_repository(
    name = "units",
    build_file = "//bazel:units.BUILD",
    remote = "https://github.com/nholthaus/units",
    tag = "v2.3.1",
)