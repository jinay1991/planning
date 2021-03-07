def sysroot():
    if "sysroot" not in native.existing_rules():
        native.new_local_repository(
            name = "sysroot",
            path = "/",
            build_file = "//third_party/sysroot:sysroot.BUILD",
        )
