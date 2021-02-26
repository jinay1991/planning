load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "feature", "flag_group", "flag_set", "tool_path", "with_feature_set")
load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")

def _impl(ctx):
    all_compile_actions = [
        ACTION_NAMES.assemble,
        ACTION_NAMES.c_compile,
        ACTION_NAMES.clif_match,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.lto_backend,
        ACTION_NAMES.preprocess_assemble,
    ]

    all_cpp_compile_actions = [
        ACTION_NAMES.clif_match,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.lto_backend,
    ]

    all_link_actions = [
        ACTION_NAMES.cpp_link_dynamic_library,
        ACTION_NAMES.cpp_link_executable,
        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
        ACTION_NAMES.lto_index_for_dynamic_library,
        ACTION_NAMES.lto_index_for_executable,
        ACTION_NAMES.lto_index_for_nodeps_dynamic_library,
    ]

    dbg_feature = feature(name = "dbg")
    fastbuild_feature = feature(name = "fastbuild")
    opt_feature = feature(name = "opt")

    supports_dynamic_linker_feature = feature(name = "supports_dynamic_linker", enabled = True)
    supports_fission_feature = feature(name = "supports_fission", enabled = True)
    supports_legacy_compile_flags_feature = feature(name = "legacy_compile_flags", enabled = False)
    supports_legacy_link_flags_feature = feature(name = "legacy_link_flags", enabled = False)
    supports_pic_feature = feature(name = "supports_pic", enabled = True)
    supports_start_end_lib_feature = feature(name = "supports_start_end_lib", enabled = True)

    default_link_flags_feature = feature(
        name = "default_link_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-pipe",
                            "-fuse-id=lld",
                            "-Wl,-z,relro,-z,now,-z,notext",
                            "-no-canonical-prefixes",
                            "-pthrea",
                        ],
                    ),
                ],
                with_features = [
                    with_feature_set(features = ["dbg"]),
                ],
            ),
            flag_set(
                actions = all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fprofile-instr-generate",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wl,--gc-sections",
                        ],
                    ),
                ],
                with_features = [
                    with_feature_set(features = ["opt"]),
                ],
            ),
        ],
    )

    default_compile_flags_feature = feature(
        name = "default_compile_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-pipe",
                            "-U_FORTIFY_SOURCE",
                            "-fstack-protector-strong",
                            "-fcolor-diagnostics",
                            "-fno-omit-frame-pointer",
                            "-march=nehalem",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fprofile-instr-generate",
                            "-fcoverage-mapping",
                        ],
                    ),
                ],
                with_features = [
                    with_feature_set(features = ["dbg"]),
                ],
            ),
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-O1",
                        ],
                    ),
                ],
                with_features = [
                    with_feature_set(features = ["fastbuild"]),
                ],
            ),
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-O2",
                            "-D_FORTIFY_SOURCE=2",
                            "-DNDEBUG",
                            "-ffunction-sections",
                            "-fdata-sections",
                        ],
                    ),
                ],
                with_features = [
                    with_feature_set(features = ["opt"]),
                ],
            ),
        ],
    )

    debug_symbols_feature = feature(
        name = "debug_symbols",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [flag_group(flags = ["-g"])],
                with_features = [with_feature_set(features = ["dbg"])],
            ),
            flag_set(
                actions = all_compile_actions,
                flag_groups = [flag_group(flags = ["-g0"])],
                with_features = [with_feature_set(features = ["fastbuild"])],
            ),
            flag_set(
                actions = all_compile_actions,
                flag_groups = [flag_group(flags = ["-gline-tables-only"])],
                with_features = [with_feature_set(features = ["opt"])],
            ),
        ],
    )

    unfiltered_compile_flags_feature = feature(
        name = "unfiltered_compile_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-no-canonical-prefixes",
                            "-Wno-builtin-macro-redefined",
                            "-D__DATE__=\"redacted\"",
                            "-D__TIMESTAMP__=\"redacted\"",
                            "-D__TIME__=\"redacted\"",
                        ],
                    ),
                ],
            ),
        ],
    )

    user_compile_flags_feature = feature(
        name = "user_compile_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        expand_if_available = "user_compile_flags",
                        iterate_over = "user_compile_flags",
                        flags = ["%{user_compile_flags}"],
                    ),
                ],
            ),
        ],
    )

    force_c_mode_feature = feature(
        name = "force_c_mode",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-xc",
                        ],
                    ),
                ],
            ),
        ],
    )

    sanitizer_feature = feature(
        name = "sanitizer",
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fsanitize=undefined,address",
                            "-fno-sanitize=float-divide-by-zero",
                            "-fno-sanitize-recover=undefined,address",
                            "-fno-optimize-sibling-calls",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fsanitize=undefined,address",
                        ],
                    ),
                ],
            ),
        ],
    )

    fuzzer_feature = feature(
        name = "fuzzer",
        implies = ["sanitizer"],
        flag_sets = [
            flag_set(
                actions = all_compile_actions + all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fsanitize=fuzzer",
                        ],
                    ),
                ],
            ),
        ],
    )

    minimal_warnings_feature = feature(
        name = "minimal_warnings",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wall",
                            "-Wno-deprecated-declarations",
                            "-Wno-cpp",
                            "-Wno-error=self-assign-overloaded",  # see https://bugs.llvm.org/show_bug.cgi?id=43124 for details
                        ],
                    ),
                ],
            ),
        ],
    )

    strict_warnings_feature = feature(
        name = "strict_warnings",
        implies = ["minimal_warnings"],
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wextra",
                            "-pedantic",
                            "-Wconversion",
                            "-Wfloat-equal",
                            "-Wformat-security",
                            "-Wsign-conversion",
                        ],
                    ),
                ],
            ),
        ],
    )

    test_warnings_feature = feature(
        name = "test_warnings",
        implies = ["minimal_warnings"],
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wno-zero-as-null-pointer-constant",  # Used by GTEST
                            "-Wno-gnu-zero-variadic-macro-arguments",  # Used by GTEST
                        ],
                    ),
                ],
            ),
        ],
    )

    third_party_warnings_feature = feature(
        name = "third_party_warnings",
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-w",
                        ],
                    ),
                ],
            ),
        ],
    )

    treat_warnings_as_errors_feature = feature(
        name = "treat_warnings_as_errors",
        flag_sets = [
            flag_set(
                actions = all_compile_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Werror",
                        ],
                    ),
                ],
            ),
        ],
    )

    sysroot_feature = feature(
        name = "sysroot",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = all_compile_actions + all_link_actions,
                flag_groups = [
                    flag_group(
                        flags = [
                            "--sysroot=external/sysroot",
                        ],
                    ),
                ],
            ),
        ],
    )

    features = [
        dbg_feature,
        fastbuild_feature,
        opt_feature,
        supports_dynamic_linker_feature,
        supports_fission_feature,
        supports_legacy_compile_flags_feature,
        supports_legacy_link_flags_feature,
        supports_start_end_lib_feature,
        supports_pic_feature,
        sysroot_feature,
        default_compile_flags_feature,
        debug_symbols_feature,
        minimal_warnings_feature,
        strict_warnings_feature,
        test_warnings_feature,
        user_compile_flags_feature,
        unfiltered_compile_flags_feature,
        default_link_flags_feature,
        sanitizer_feature,
        fuzzer_feature,
        treat_warnings_as_errors_feature,
        third_party_warnings_feature,
        force_c_mode_feature,
    ]

    tool_paths = [
        tool_path(name = "ar", path = "llvm/llvm-ar"),
        tool_path(name = "compat-ld", path = "bin/false"),
        tool_path(name = "cpp", path = "llvm/clang-cpp"),
        tool_path(name = "dwp", path = "bin/false"),
        tool_path(name = "gcc", path = "llvm/clang++"),
        tool_path(name = "gcov", path = "llvm/llvm-cov"),
        tool_path(name = "ld", path = "llvm/lld"),
        tool_path(name = "nm", path = "llvm/llvm-nm"),
        tool_path(name = "objcopy", path = "bin/false"),
        tool_path(name = "objdump", path = "bin/false"),
        tool_path(name = "strip", path = "bin/false"),
    ]

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, "Fake executable")
    return [
        cc_common.create_cc_toolchain_config_info(
            ctx = ctx,
            features = features,
            action_configs = [],
            artifact_name_patterns = [],
            cxx_builtin_include_directories = [],
            toolchain_identifier = "clang-linux",
            host_system_name = "local",
            target_system_name = "local",
            target_cpu = "local",
            target_libc = "local",
            compiler = "clang",
            abi_version = "local",
            abi_libc_version = "local",
            tool_paths = tool_paths,
            make_variables = [],
            builtin_sysroot = None,
            cc_target_os = None,
        ),
        DefaultInfo(
            executable = out,
        ),
    ]

clang_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
    executable = True,
)
