def uv():
    if "uv" not in native.existing_rules():
        native.new_local_repository(
            name = "uv",
            build_file = "//third_party/uv:uv.BUILD",
            path = "/usr/",
        )
