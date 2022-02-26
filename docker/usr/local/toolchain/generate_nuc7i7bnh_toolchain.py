#! /usr/bin/env python3

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
