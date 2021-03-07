load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def spline():
    if "spline" not in native.existing_rules():
        http_archive(
            name = "spline",
            build_file = "//third_party/spline:spline.BUILD",
            strip_prefix = "spline-master",
            url = "https://github.com/ttk592/spline/archive/master.zip",
            sha256 = "a26df8c582302692354d971a4cbdabf8831539cbe73b82d6a20fef9a7fc6dee9",
        )
