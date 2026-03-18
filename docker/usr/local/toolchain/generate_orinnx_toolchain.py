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

_TOOLCHAIN_BIN = "/l4t/toolchain/aarch64--glibc--stable-2022.08-1/bin"
_CROSS_PREFIX = "aarch64-buildroot-linux-gnu-"

target = {
    "flags": ["-fPIC", "-mtune=cortex-a78ae", "-march=armv8.2-a"],
    "release_flags": ["-O3", "-DNDEBUG"],
    "asm_flags": ["-DELF", "-D__aarch64__", "-DPIC"],
    "asm_object": "elf64",
    "arch": "aarch64",
    "sysroot": "/l4t/targetfs",
    "c_compiler": f"{_TOOLCHAIN_BIN}/{_CROSS_PREFIX}gcc",
    "cxx_compiler": f"{_TOOLCHAIN_BIN}/{_CROSS_PREFIX}g++",
    "cross_compile_prefix": f"{_TOOLCHAIN_BIN}/{_CROSS_PREFIX}",
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
