#! /usr/bin/env python3

import os
import argparse
import sys
from textwrap import dedent

targets = {
    "generic": {"flags": ["-fPIC", "-mtune=generic"], "release_flags": ["-O3", "-DNDEBUG"], "params": []},
    "native": {
        "flags": ["-fPIC", "-march=native", "-mtune=native"],
        "release_flags": ["-O3", "-DNDEBUG"],
        "params": [],
    },
    "nuc7i7bnh": {
        "flags": [
            "-fPIC",
            "-march=broadwell",
            "-mtune=broadwell",
            "-mmmx",
            "-mno-3dnow",
            "-msse",
            "-msse2",
            "-msse3",
            "-mssse3",
            "-mno-sse4a",
            "-mcx16",
            "-msahf",
            "-mmovbe",
            "-maes",
            "-mno-sha",
            "-mpclmul",
            "-mpopcnt",
            "-mabm",
            "-mno-lwp",
            "-mfma",
            "-mno-fma4",
            "-mno-xop",
            "-mbmi",
            "-mbmi2",
            "-mno-tbm",
            "-mavx",
            "-mavx2",
            "-msse4.2",
            "-msse4.1",
            "-mlzcnt",
            "-mno-rtm",
            "-mno-hle",
            "-mrdrnd",
            "-mf16c",
            "-mfsgsbase",
            "-mrdseed",
            "-mprfchw",
            "-madx",
            "-mfxsr",
            "-mxsave",
            "-mxsaveopt",
            "-mno-avx512f",
            "-mno-avx512er",
            "-mno-avx512cd",
            "-mno-avx512pf",
            "-mno-prefetchwt1",
            "-mclflushopt",
            "-mxsavec",
            "-mxsaves",
            "-mno-avx512dq",
            "-mno-avx512bw",
            "-mno-avx512vl",
            "-mno-avx512ifma",
            "-mno-avx512vbmi",
            "-mno-clwb",
            "-mno-mwaitx",
        ],
        "release_flags": ["-O3", "-DNDEBUG"],
        "params": ["--param l1-cache-size=32", "--param l1-cache-line-size=64", "--param l2-cache-size=4096"],
    },
}


def generate_cmake_toolchain(target, output_path):

    template = dedent(
        """\
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_C_COMPILER gcc)
        set(CMAKE_CXX_COMPILER g++)
        set(CMAKE_FIND_ROOT_PATH
               "/usr/local"
               "/usr")
        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

        {compile_options}

        set(CMAKE_C_FLAGS "{compile_params}" CACHE STRING "")
        set(CMAKE_CXX_FLAGS "{compile_params}" CACHE STRING "")

        set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "")
        set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON CACHE STRING "Should pkg-config use the cmake prefix path for finding modules")
        set(CMAKE_PREFIX_PATH "/usr/local" CACHE STRING "")
        """
    )

    with open(output_path, "w") as f:
        f.write(
            template.format(
                compile_options="\n".join(
                    ["add_compile_options({})".format(flag) for flag in targets[target]["flags"]]
                ),
                compile_params=" ".join(targets[target]["params"]),
                arch=target,
            )
        )


def generate_meson_cross_file(target, output_path):

    template = dedent(
        """\
        [host_machine]
        system = 'linux'
        cpu_family = 'x86_64'
        cpu = 'x86_64'
        endian = 'little'

        [properties]
        c_args = [{flags}]
        cpp_args = [{flags}]
        fortran_args = [{flags}]
        growing_stack = false

        [binaries]
        c = 'gcc'
        cpp = 'g++'
        fortran = 'gfortran'
        ar = 'ar'
        strip = 'strip'
        pkgconfig = 'pkg-config'
        """
    )

    with open(output_path, "w") as f:
        f.write(
            template.format(
                flags=", ".join(
                    [
                        '"{}"'.format(flag)
                        for flag in targets[target]["release_flags"]
                        + targets[target]["flags"]
                        + [param.replace(" ", '", "') for param in targets[target]["params"]]
                    ]
                )
            )
        )


def generate_toolchain_script(target, output_path):

    template = dedent(
        """\
        #!/bin/sh

        # Set our compilers
        export CC=/usr/bin/gcc
        export CXX=/usr/bin/g++
        export FC=/usr/bin/gfortran
        export F77=/usr/bin/gfortran

        # Set our package config so it finds things in the toolchain
        export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

        # Set our optimisation flags
        export CFLAGS="{flags}"
        export CXXFLAGS="${{CFLAGS}}"
        export CPPFLAGS="${{CFLAGS}}"
        export FFLAGS="${{CFLAGS}}"
        export FCFLAGS="${{CFLAGS}}"
        """
    )

    with open(output_path, "w") as f:
        f.write(
            template.format(
                flags=" ".join(targets[target]["release_flags"] + targets[target]["flags"] + targets[target]["params"])
            )
        )


def generate(prefix, targets=[target for target in targets.keys()]):
    for target in targets:
        print("Generating toolchain script for {} in {}".format(target, prefix))
        generate_toolchain_script(target, os.path.join(prefix, "toolchain.sh"))
        print("Generating meson cross file for {} in {}".format(target, prefix))
        generate_meson_cross_file(target, os.path.join(prefix, "meson.cross"))
        print("Generating cmake toolchain file for {} in {}".format(target, prefix))
        generate_cmake_toolchain(target, os.path.join(prefix, "toolchain.cmake"))


def list_toolchains():
    return [target for target in targets.keys()]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate toolchain files for the docker images")
    parser.add_argument("--list", action="store_true", help="Print available toolchains and exit")
    parser.add_argument(
        "--prefix", default=os.path.join("usr", "local"), help="Prefix path to store generated files in"
    )
    parser.add_argument(
        "--targets",
        nargs="*",
        default=[target for target in targets.keys()],
        choices=[target for target in targets.keys()],
        help="Targets to generate toolchain files for",
    )

    args = parser.parse_args()

    if args.list:
        print("\n".join(list_toolchains()))
    else:
        generate(args.prefix, args.targets)
