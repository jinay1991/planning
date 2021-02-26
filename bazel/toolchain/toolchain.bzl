def register_llvm_toolchain():
    """
    Register LLVM Toolchain to bazel
    """
    native.register_toolchains(
        "//bazel/toolchain/clang:clang_toolchain",
    )
