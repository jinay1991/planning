load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def eigen():
    if "eigen" not in native.existing_rules():
        http_archive(
            name = "eigen",
            build_file = "//third_party/eigen:eigen.BUILD",
            url = "https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz",
            sha256 = "146a480b8ed1fb6ac7cd33fec9eb5e8f8f62c3683b3f850094d9d5c35a92419a",
        )
