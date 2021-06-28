load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def llvm_toolchain():
    if "llvm_toolchain" not in native.existing_rules():
        http_archive(
            name = "llvm_toolchain",
            url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-12.0.0/clang+llvm-12.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz",
            sha256 = "a9ff205eb0b73ca7c86afc6432eed1c2d49133bd0d49e47b15be59bbf0dd292e",
            strip_prefix = "clang+llvm-12.0.0-x86_64-linux-gnu-ubuntu-20.04",
            build_file = "//third_party/llvm_toolchain:llvm_toolchain.BUILD",
        )
