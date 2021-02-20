load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def benchmark():
    if "benchmark" not in native.existing_rules():
        http_archive(
            name = "benchmark",
            url = "https://github.com/google/benchmark/archive/v1.5.2.tar.gz",
            sha256 = "dccbdab796baa1043f04982147e67bb6e118fe610da2c65f88912d73987e700c",
            strip_prefix = "benchmark-1.5.2",
        )
