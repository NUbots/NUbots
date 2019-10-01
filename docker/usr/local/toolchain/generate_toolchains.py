#! /usr/bin/env python3

import os
from textwrap import dedent


def generate_cmake_toolchain(target, prefix):

    template = dedent(
        """\
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR x86_64)
        set(CMAKE_C_COMPILER gcc)
        set(CMAKE_CXX_COMPILER g++)
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
        set(CMAKE_INSTALL_LIBDIR "lib" CACHE STRING "Direcxtory into which object files and object code libraries should be installed (Default: lib).")

        set(CMAKE_COLOR_MAKEFILE ON CACHE STRING "Enable/Disable color output during build.")
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
                '"{}"'.format(flag)
                for flag in target["release_flags"] + [param.replace(" ", '", "') for param in target["flags"]]
            ]
        )
    )


def generate_toolchain_script(target):

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

    return template.format(flags=" ".join(target["release_flags"] + target["flags"]))


def generate(prefix, toolchain, target):
    print("Generating toolchain script for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "toolchain.sh"), "w") as f:
        f.write(generate_toolchain_script(target))

    print("Generating meson cross file for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "meson.cross"), "w") as f:
        f.write(generate_meson_cross_file(target))

    print("Generating cmake toolchain file for {} in {}".format(toolchain, prefix))
    with open(os.path.join(prefix, "toolchain.cmake"), "w") as f:
        f.write(generate_cmake_toolchain(target, prefix))
