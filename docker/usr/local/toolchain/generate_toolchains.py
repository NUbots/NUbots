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
    sysroot = target.get("sysroot", "")
    sysroot_block = (
        dedent(
            """\
            set(CMAKE_SYSROOT "{sysroot}")
            set(CMAKE_FIND_ROOT_PATH
                    "{sysroot}"
                    "{sysroot}/usr/aarch64-linux-gnu"
                    "{sysroot}/usr/lib"
                    "{toolchain_path}/aarch64-buildroot-linux-gnu/sysroot"
                    "{toolchain_path}"
                    {prefix}
                    "/usr/local"
                    "/usr")
            """
        ).format(sysroot=sysroot, prefix=prefix, toolchain_path=target.get("toolchain_path", ""))
        if sysroot
        else "\n"
    )

    linker_flags = target.get("linker_flags", [])
    linker_flags_block = (
        'set(CMAKE_EXE_LINKER_FLAGS_INIT "{} " CACHE STRING "Flags used by the linker during all built types.")\n'.format(
            " ".join(linker_flags)
        )
        + 'set(CMAKE_SHARED_LINKER_FLAGS_INIT "{} " CACHE STRING "Flags used by the linker during all built types.")\n'.format(
            " ".join(linker_flags)
        )
        if linker_flags
        else "\n"
    )

    target_block = (
        'set(CMAKE_C_COMPILER_TARGET "{target_c_compiler}")\n'.format(
            target_c_compiler=target.get("target_c_compiler", "")
        )
        + 'set(CMAKE_CXX_COMPILER_TARGET "{target_cxx_compiler}")\n'.format(
            target_cxx_compiler=target.get("target_cxx_compiler", "")
        )
        if target.get("target_c_compiler") and target.get("target_cxx_compiler")
        else "\n"
    )

    template = dedent(
        """\
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR {arch})
        set(CMAKE_C_COMPILER {c_compiler})
        set(CMAKE_CXX_COMPILER {cxx_compiler})

        {target_block}
        {sysroot_block}
        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

        set(CMAKE_C_FLAGS_INIT "" CACHE STRING "Flags used by the C compiler during all built types.")
        set(CMAKE_CXX_FLAGS_INIT "" CACHE STRING "Flags used by the CXX compiler during all built types.")
        set(CMAKE_NASM_ASM_FLAGS_INIT "" CACHE STRING "Flags used by the ASM NASM compiler during all built types.")

        if(CMAKE_CROSSCOMPILING){
            {c_target_compile_options}
            {cxx_target_compile_options}
            {asm_target_compile_options}
        }
        else{
            {c_host_compile_options}
            {cxx_host_compile_options}
            {asm_host_compile_options}
        }
        {linker_flags_block}

        set(CMAKE_ASM_NASM_OBJECT_FORMAT "{asm_object}" CACHE STRING "Output object format of the ASM NASM compiler.")

        set(CMAKE_INSTALL_PREFIX "{sysroot}{prefix}" CACHE STRING "Install path prefix, prepended onto install directories.")
        set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON CACHE STRING "Should pkg-config use the cmake prefix path for finding modules.")
        set(CMAKE_PREFIX_PATH "{sysroot}{prefix}" CACHE STRING "")
        set(CMAKE_INSTALL_LIBDIR "lib" CACHE STRING "Directory into which object files and object code libraries should be installed (Default: lib).")

        set(CMAKE_COLOR_MAKEFILE ON CACHE STRING "Enable/Disable color output during build.")

        mark_as_advanced(PKG_CONFIG_USE_CMAKE_PREFIX_PATH CMAKE_PREFIX_PATH CMAKE_NASM_ASM_FLAGS_INIT CMAKE_INSTALL_PREFIX CMAKE_INSTALL_LIBDIR CMAKE_C_FLAGS_INIT CMAKE_CXX_FLAGS_INIT CMAKE_COLOR_MAKEFILE CMAKE_ASM_NASM_OBJECT_FORMAT)
        """
    )

    return template.format(
        c_target_compile_options="\n".join(
            [
                'string(APPEND CMAKE_C_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["target_flags"]
            ]
        ),
        cxx_target_compile_options="\n".join(
            [
                'string(APPEND CMAKE_CXX_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["target_flags"]
            ]
        ),
        asm_target_compile_options="\n".join(
            [
                'string(APPEND CMAKE_NASM_ASM_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["asm_target_flags"]
            ]
        ),
        c_host_compile_options="\n".join(
            [
                'string(APPEND CMAKE_C_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["host_flags"]
            ]
        ),
        cxx_host_compile_options="\n".join(
            [
                'string(APPEND CMAKE_CXX_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["host_flags"]
            ]
        ),
        asm_host_compile_options="\n".join(
            [
                'string(APPEND CMAKE_NASM_ASM_FLAGS_INIT "{} ")'.format(flag)
                for flag in target["asm_host_flags"]
            ]
        ),
        asm_object=target["asm_object"],
        prefix=prefix,
        arch=target["arch"],
        c_compiler=target.get("c_compiler", "/usr/bin/gcc"),
        cxx_compiler=target.get("cxx_compiler", "/usr/bin/g++"),
        target_block=target_block,
        sysroot_block=sysroot_block,
        sysroot=sysroot,
        linker_flags_block=linker_flags_block
    )


def generate_meson_cross_file(target):
    sysroot = target.get("sysroot", "")
    sysroot_line = "sys_root = '{}'".format(sysroot) if sysroot else ""

    c_compiler = target.get("c_compiler", "gcc")
    cxx_compiler = target.get("cxx_compiler", "g++")
    cross_compile_prefix = target.get("cross_compile_prefix", "")
    ar = cross_compile_prefix + "ar" if cross_compile_prefix else "ar"
    strip = cross_compile_prefix + "strip" if cross_compile_prefix else "strip"

    template = dedent(
        """\
        [host_machine]
        system = 'linux'
        cpu_family = '{arch}'
        cpu = '{arch}'
        endian = 'little'

        [properties]
        c_args = [{flags}]
        cpp_args = [{flags}]
        fortran_args = [{flags}]
        growing_stack = false
        {sysroot_line}

        [binaries]
        c = '{c_compiler}'
        cpp = '{cxx_compiler}'
        fortran = 'gfortran'
        ar = '{ar}'
        strip = '{strip}'
        pkgconfig = 'pkg-config'
        """
    )

    flags = ", ".join(
        [
            "'{}'".format(flag)
            for flag in target["release_flags"]
            + [param.replace(" ", "', '") for param in target["target_flags"]]
        ]
    )

    return template.format(
        flags=flags,
        arch=target["arch"],
        sysroot_line=sysroot_line,
        c_compiler=c_compiler,
        cxx_compiler=cxx_compiler,
        ar=ar,
        strip=strip,
    )


def generate_json_env(target, prefix):
    flags = " ".join(target["release_flags"] + target["target_flags"])
    return json.dumps(
        {
            # Set our compilers
            "CC": target.get("c_compiler", "/usr/bin/gcc"),
            "CXX": target.get("cxx_compiler", "/usr/bin/g++"),
            # Set our package config so it finds things in the toolchain
            "PKG_CONFIG_PATH": f"{prefix}/lib/pkgconfig",
            "CPU_ARCH": target.get("arch", "x86_64"),
            # Set our optimisation flags
            "CFLAGS": flags,
            "CXXFLAGS": flags,
            "CPPFLAGS": flags,
        }
    )


def generate_toolchain_script(target):
    c_compiler = target.get("c_compiler", "/usr/bin/gcc")
    cxx_compiler = target.get("cxx_compiler", "/usr/bin/g++")
    template = dedent(
        """\
        #!/bin/sh

        # Set our compilers
        export CC={c_compiler}
        export CXX={cxx_compiler}

        # Set our package config so it finds things in the toolchain
        export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

        # Set our optimisation flags
        export CFLAGS="${{CFLAGS}} {flags}"
        export CXXFLAGS="${{CXXFLAGS}} ${{CFLAGS}}"
        export CPPFLAGS="${{CPPFLAGS}} ${{CFLAGS}}"
        """
    )

    return template.format(
        c_compiler=c_compiler,
        cxx_compiler=cxx_compiler,
        flags=" ".join(target["release_flags"] + target["target_flags"]),
    )


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
