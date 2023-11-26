#! /usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import json
import os
from textwrap import dedent


def generate_cmake_toolchain(target, prefix):
    template = dedent(
        """\
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR x86_64)
        set(CMAKE_C_COMPILER /usr/bin/gcc)
        set(CMAKE_CXX_COMPILER /usr/bin/g++)
        set(CMAKE_FIND_ROOT_PATH
               {prefix}
               "/usr/local"
               "/usr")
        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

        set(CMAKE_C_FLAGS_INIT "" CACHE STRING "Flags used by the C compiler during all built types.")
        set(CMAKE_CXX_FLAGS_INIT "" CACHE STRING "Flags used by the CXX compiler during all built types.")
        set(CMAKE_NASM_ASM_FLAGS_INIT "" CACHE STRING "Flags used by the ASM NASM compiler during all built types.")

        {c_compile_options}
        {cxx_compile_options}
        {asm_compile_options}

        set(CMAKE_ASM_NASM_OBJECT_FORMAT "{asm_object}" CACHE STRING "Output object format of the ASM NASM compiler.")

        set(CMAKE_INSTALL_PREFIX "{prefix}" CACHE STRING "Install path prefix, prepended onto install directories.")
        set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON CACHE STRING "Should pkg-config use the cmake prefix path for finding modules.")
        set(CMAKE_PREFIX_PATH "{prefix}" CACHE STRING "")
        set(CMAKE_INSTALL_LIBDIR "lib" CACHE STRING "Directory into which object files and object code libraries should be installed (Default: lib).")

        set(CMAKE_COLOR_MAKEFILE ON CACHE STRING "Enable/Disable color output during build.")

        mark_as_advanced(PKG_CONFIG_USE_CMAKE_PREFIX_PATH CMAKE_PREFIX_PATH CMAKE_NASM_ASM_FLAGS_INIT CMAKE_INSTALL_PREFIX CMAKE_INSTALL_LIBDIR CMAKE_C_FLAGS_INIT CMAKE_CXX_FLAGS_INIT CMAKE_COLOR_MAKEFILE CMAKE_ASM_NASM_OBJECT_FORMAT)
        """
    )

    return template.format(
        c_compile_options="\n".join(
            ['string(APPEND CMAKE_C_FLAGS_INIT "{} ")'.format(flag) for flag in target["flags"]]
        ),
        cxx_compile_options="\n".join(
            ['string(APPEND CMAKE_CXX_FLAGS_INIT "{} ")'.format(flag) for flag in target["flags"]]
        ),
        asm_compile_options="\n".join(
            ['string(APPEND CMAKE_NASM_ASM_FLAGS_INIT "{} ")'.format(flag) for flag in target["asm_flags"]]
        ),
        asm_object=target["asm_object"],
        prefix=prefix,
    )


def generate_meson_cross_file(target):
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

    return template.format(
        flags=", ".join(
            [
                "'{}'".format(flag)
                for flag in target["release_flags"] + [param.replace(" ", "', '") for param in target["flags"]]
            ]
        )
    )


def generate_json_env(target, prefix):
    flags = " ".join(target["release_flags"] + target["flags"])
    return json.dumps(
        {
            # Set our compilers
            "CC": "/usr/bin/gcc",
            "CXX": "/usr/bin/g++",
            # Set our package config so it finds things in the toolchain
            "PKG_CONFIG_PATH": f"{prefix}/lib/pkgconfig",
            # Set our optimisation flags
            "CFLAGS": flags,
            "CXXFLAGS": flags,
            "CPPFLAGS": flags,
        }
    )


def generate_toolchain_script(target):
    template = dedent(
        """\
        #!/bin/sh

        # Set our compilers
        export CC=/usr/bin/gcc
        export CXX=/usr/bin/g++

        # Set our package config so it finds things in the toolchain
        export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

        # Set our optimisation flags
        export CFLAGS="${{CFLAGS}} {flags}"
        export CXXFLAGS="${{CXXFLAG}} ${{CFLAGS}}"
        export CPPFLAGS="${{CPPFLAGS}} ${{CFLAGS}}"
        """
    )

    return template.format(flags=" ".join(target["release_flags"] + target["flags"]))


def generate(prefix, toolchain, target):
    print("Generating json toolchain script for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "toolchain.json"), "w") as f:
        f.write(generate_json_env(target, prefix))

    print("Generating toolchain script for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "toolchain.sh"), "w") as f:
        f.write(generate_toolchain_script(target))

    print("Generating meson cross file for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "meson.cross"), "w") as f:
        f.write(generate_meson_cross_file(target))

    print("Generating cmake toolchain file for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "toolchain.cmake"), "w") as f:
        f.write(generate_cmake_toolchain(target, prefix))
