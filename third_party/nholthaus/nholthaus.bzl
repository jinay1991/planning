load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def nholthaus():
    if "nholthaus" not in native.existing_rules():
        http_archive(
            name = "nholthaus",
            build_file = "//third_party/nholthaus:nholthaus.BUILD",
            url = "https://github.com/nholthaus/units/archive/v2.3.1.zip",
            sha256 = "7d7819532e65e5afb589c0b795b94881742b22e9141e5d7e5bfe9ae0bbf343f4",
            strip_prefix = "units-2.3.1",
        )
