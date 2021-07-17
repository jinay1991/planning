load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def middleware():
    """ Load middleware """
    if "middleware" not in native.existing_rules():
        http_archive(
            name = "middleware",
            url = "https://github.com/jinay1991/artifactory/releases/download/v1.0/libmiddleware_dev-linux.tar.gz",
            sha256 = "50b1b7cbe0392648d3be940c847ea5ae4b5df51cde2ac4bcff49f7ce8ceefa7c",
            build_file = "//third_party/middleware:middleware.BUILD",
        )
