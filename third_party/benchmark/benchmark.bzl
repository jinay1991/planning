load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def benchmark():
    if "benchmark" not in native.existing_rules():
        http_archive(
            name = "benchmark",
            url = "https://github.com/google/benchmark/archive/refs/tags/v1.6.1.tar.gz",
            sha256 = "6132883bc8c9b0df5375b16ab520fac1a85dc9e4cf5be59480448ece74b278d4",
            strip_prefix = "benchmark-1.6.1",
        )
