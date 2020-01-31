def openssl():
    if "openssl" not in native.existing_rules():
        native.new_local_repository(
            name = "openssl",
            build_file = "//third_party/openssl:openssl.BUILD",
            path = "/usr/",
        )
