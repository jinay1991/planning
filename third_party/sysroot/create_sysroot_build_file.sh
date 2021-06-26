#!/usr/bin/env bash

set -euo pipefail

die () {
    >&2 echo "Fatal: ${@}"
    exit 1
}

write_filegroup () {
    [[ $# -lt 2 ]] && die "invoke with \"write_filegroup <name> [Debian package]...\""

    echo "
filegroup(
    name = \"${1}\",
    srcs = [" >> "${FILE}"
  while read f; do [ -f "${f}" ] && echo "        \"${f:1}\"," >> "${FILE}" ; done < <(dpkg -L "${@:2}" | sort)
  echo "    ],
)" >> "${FILE}"
}

FILE=sysroot.BUILD
echo "# DO NOT MODIFY. This file is auto-generated.
package(default_visibility = [\"//visibility:public\"])" > "${FILE}"

write_filegroup libstdc++ libstdc++6 libstdc++-9-dev
write_filegroup libgcc libgcc-s1 libgcc-9-dev
write_filegroup libcrypt libcrypt1 libcrypt-dev
write_filegroup libc libc6 libc6-dev
write_filegroup kernel_headers linux-libc-dev

# awk -i inplace '!/\.(amd64|gz)"/ { print }' "${FILE}"
