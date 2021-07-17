load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def spline():
    if "spline" not in native.existing_rules():
        new_git_repository(
            name = "spline",
            build_file = "//third_party/spline:spline.BUILD",
            branch = "master",
            remote = "https://github.com/ttk592/spline.git",
        )
