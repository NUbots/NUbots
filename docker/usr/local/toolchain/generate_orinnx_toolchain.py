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

import argparse
import os

import generate_toolchains

_TOOLCHAIN_DIR = "/l4t/toolchain/aarch64--glibc--stable-2022.08-1"
_CROSS_PREFIX = "aarch64-buildroot-linux-gnu"
_TARGETFS_DIR = "/l4t/targetfs"

target = {
    "flags": [
        "-fPIC",
        "-mtune=cortex-a78ae",
        "-march=armv8.2-a",
        f"-isystem {_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/include/c++/11.3.0",
        f"-isystem {_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/include/c++/11.3.0/{_CROSS_PREFIX}",
        f"-isystem {_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/include/c++/11.3.0/backward",
        f"-isystem {_TOOLCHAIN_DIR}/lib/gcc/{_CROSS_PREFIX}/11.3.0/include",
        f"-isystem {_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/sysroot/usr/include",
        f"-isystem {_TARGETFS_DIR}/usr/include",
        f"-isystem {_TARGETFS_DIR}/usr/include/aarch64-linux-gnu"
    ],
    "release_flags": ["-O3", "-DNDEBUG"],
    "asm_flags": ["-DELF", "-D__aarch64__", "-DPIC"],
    "linker_flags":[
        f"--sysroot={_TARGETFS_DIR}",
        f"-L{_TARGETFS_DIR}/usr/lib/aarch64-linux-gnu",
        f"-L{_TARGETFS_DIR}/usr/lib",
        f"-L{_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/sysroot/usr/lib",
        "-L/usr/local/cuda-12.6/targets/aarch64-linux/lib",
        f"-B{_TARGETFS_DIR}/usr/lib/aarch64-linux-gnu",
        f"-B{_TARGETFS_DIR}/usr/lib",
        f"-B{_TOOLCHAIN_DIR}/{_CROSS_PREFIX}/sysroot/usr/lib",
        f"-Wl,-rpath-link,{_TARGETFS_DIR}/usr/lib/aarch64-linux-gnu",
        f"-Wl,-rpath-link,{_TARGETFS_DIR}/usr/lib",
        "-Wl,--allow-shlib-undefined"
    ],
    "asm_object": "elf64",
    "arch": "aarch64",
    "sysroot": "/l4t/targetfs",
    "c_compiler": f"{_TOOLCHAIN_DIR}/bin/{_CROSS_PREFIX}-gcc",
    "cxx_compiler": f"{_TOOLCHAIN_DIR}/bin/{_CROSS_PREFIX}-g++",
    "target_c_compiler": f"{_TOOLCHAIN_DIR}/bin/{_CROSS_PREFIX}-gcc",
    "target_cxx_compiler": f"{_TOOLCHAIN_DIR}/bin/{_CROSS_PREFIX}-g++",
    "cross_compile_prefix": f"{_TOOLCHAIN_DIR}/bin/{_CROSS_PREFIX}",
    "toolchain_path": _TOOLCHAIN_DIR,
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate toolchain files for the docker images"
    )
    parser.add_argument(
        "--prefix",
        default=os.path.join("usr", "local"),
        help="Prefix path to store generated files in",
    )

    args = parser.parse_args()

    generate_toolchains.generate(args.prefix, "orinnx", target)
