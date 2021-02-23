load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def spline():
    if "spline" not in native.existing_rules():
        http_archive(
            name = "spline",
            build_file = "//third_party/spline:spline.BUILD",
            strip_prefix = "spline-master",
            url = "https://github.com/ttk592/spline/archive/master.zip",
            sha256 = "c2d9144335f99eecd1015f4c08dc8ef260244d6e2c4b253b79b8579bc7488c98",
        )
