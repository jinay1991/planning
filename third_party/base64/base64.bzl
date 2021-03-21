load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def base64():
    if "base64" not in native.existing_rules():
        http_archive(
            name = "base64",
            url = "https://github.com/ReneNyffenegger/cpp-base64/archive/refs/tags/V2.rc.08.tar.gz",
            sha256 = "0a7ada789a99c2664437f1db8c38b60fe5815cf82b75bea0f0c08933c1317828",
            build_file = "//third_party/base64:base64.BUILD",
            strip_prefix = "cpp-base64-2.rc.08",
        )
