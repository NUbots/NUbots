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

target = {
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
        "--param l1-cache-size=32",
        "--param l1-cache-line-size=64",
        "--param l2-cache-size=4096",
    ],
    "release_flags": ["-O3", "-DNDEBUG"],
    "asm_flags": ["-DELF", "-D__x86_64__", "-DPIC"],
    "asm_object": "elf64",
}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate toolchain files for the docker images")
    parser.add_argument(
        "--prefix", default=os.path.join("usr", "local"), help="Prefix path to store generated files in"
    )

    args = parser.parse_args()

    generate_toolchains.generate(args.prefix, "nuc7i7bnh", target)
