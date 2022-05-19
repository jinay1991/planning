load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def zlib():
    if "zlib" not in native.existing_rules():
        http_archive(
            name = "zlib",
            build_file = "//third_party/zlib:zlib.BUILD",
            sha256 = "91844808532e5ce316b3c010929493c0244f3d37593afd6de04f71821d5136d9",
            strip_prefix = "zlib-1.2.12",
            url = "https://zlib.net/zlib-1.2.12.tar.gz",
        )
