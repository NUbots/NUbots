#! /usr/bin/env python3

import argparse
import os

import generate_toolchains

target = {
    "flags": [
        "-march=skylake",
        "-mtune=skylake",
        "-mabm",
        "--param l1-cache-size=32",
        "--param l1-cache-line-size=64",
        "--param l2-cache-size=8192",
        "-fPIC",
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

    generate_toolchains.generate(args.prefix, "nuc8i7beh", target)
