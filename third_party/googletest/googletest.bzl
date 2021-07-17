load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def googletest():
    if "googletest" not in native.existing_rules():
        http_archive(
            name = "googletest",
            url = "https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz",
            sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
            strip_prefix = "googletest-release-1.11.0",
        )
