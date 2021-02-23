load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def spline():
    if "spline" not in native.existing_rules():
        http_archive(
            name = "spline",
            build_file = "//third_party/spline:spline.BUILD",
            strip_prefix = "spline-master",
            url = "https://github.com/ttk592/spline/archive/master.zip",
            sha256 = "54534b42671702c67dff4103d56f108129753d14d8b8d54b3ca27f1553d1da7d",
        )
