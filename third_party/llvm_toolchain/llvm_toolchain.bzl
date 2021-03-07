load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def llvm_toolchain():
    if "llvm_toolchain" not in native.existing_rules():
        http_archive(
            name = "llvm_toolchain",
            url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz",
            sha256 = "829f5fb0ebda1d8716464394f97d5475d465ddc7bea2879c0601316b611ff6db",
            strip_prefix = "clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04",
            build_file = "//third_party/llvm_toolchain:llvm_toolchain.BUILD",
        )
