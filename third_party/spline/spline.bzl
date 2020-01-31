load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def spline():
    if "spline" not in native.existing_rules():
        http_archive(
            name = "spline",
            build_file = "//third_party/spline:spline.BUILD",
            strip_prefix = "spline-master",
            url = "https://github.com/ttk592/spline/archive/master.zip",
            sha256 = "9309186ebd3b0845e9f377d4d610598df43591674d57a2f2de81bfde96f2d001",
        )
