#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2023 NUbots
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
import json
import multiprocessing as mp
import os
import re
import shutil
import subprocess
import sys
import tarfile
import urllib
import zipfile

import magic
import requests
from tqdm import tqdm


def download_file(url, output_folder):
    # Make sure output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # Local file specified with file protocol, use urllib to copy it
    if url.startswith(("file://")):
        filename = os.path.join(output_folder, os.path.basename(url))
        with urllib.request.urlopen(url) as remote:
            with open(filename, "wb") as local:
                local.write(remote.read())
        return filename

    # Local file with no protocol specified, use shutil to copy it
    if ":" not in url:
        return shutil.copy2(url, output_folder)

    response = requests.get(url, stream=True, allow_redirects=True)
    if response.status_code != 200:
        raise RuntimeError(f"Request to {url} returned status code {response.status_code}")

    total_size = int(response.headers.get("content-length", 0))

    content_disposition = response.headers.get("content-disposition", "")
    match = re.search(r'filename="?([^";]*)"?', content_disposition)
    if match is not None:
        filename = match.group(1)
    else:
        filename = urllib.parse.unquote_plus(urllib.parse.urlparse(url).path)
        filename = os.path.basename(filename)
    filename = os.path.join(output_folder, filename)

    with tqdm(total=total_size, unit="B", unit_scale=True, unit_divisor=1024, desc="Downloading") as pbar:
        with open(filename, "wb") as f:
            for data in response.iter_content(1024):
                pbar.update(len(data))
                f.write(data)

    return filename


def extract_tar(archive, output_folder):
    with tarfile.open(archive, "r") as tar:
        for member in tqdm(tar.getmembers(), dynamic_ncols=True, unit="file", desc="Extracting"):
            tar.extract(member, path=output_folder)
        return os.path.commonpath(tar.getnames())


def extract_zip(archive, output_folder):
    with zipfile.ZipFile(archive, "r") as z:
        for member in tqdm(z.infolist(), dynamic_ncols=True, unit="file", desc="Extracting"):
            z.extract(member, path=output_folder)
        return os.path.commonpath(z.namelist())


def detect_build_system(args, extracted_path):
    configure_path = os.path.join(extracted_path, args.configure_path)
    if args.build_system is not None:
        return args.build_system, configure_path

    for root, _, files in os.walk(configure_path):
        if "CMakeLists.txt" in files:
            return "cmake", root
        if "meson.build" in files:
            return "meson", root
        if "configure" in files or "configure.ac" in files or "autogen.sh" in files:
            return "autotools", root
        if "Jamroot" in files:
            return "bjam", root
        if "Makefile" in files or "makefile" in files:
            return "make", root
        if "setup.py" in files:
            return "python", root
    raise "Unable to determine build system to use"


def handle_autotools(archive, args, env):
    # Check for autogen files
    root_files = os.listdir(args.configure_path)
    if "configure" not in root_files or args.autotools_force_regenerate:
        if "autogen.sh" in root_files:
            # Some autogen scripts will automatically run configure after they have generated the configure files
            # "NOCONFIGURE=1" and "--no-configure" are two ways that we can disable this behaviour
            # (different packagers have different switches)
            subprocess.run(
                ["./autogen.sh", "--no-configure"], env={**env, "NOCONFIGURE": "1"}, cwd=args.configure_path, check=True
            )
        elif "configure.ac" in root_files:
            subprocess.run(["autoreconf", "-fiv"], cwd=args.configure_path, env=env, check=True)

    # Configure the library
    subprocess.run(
        ["./configure", f"--prefix={args.prefix}", *args.extra_args], cwd=args.configure_path, env=env, check=True
    )

    # Build the library
    subprocess.run(["make", f"-j{args.jobs}"], cwd=args.configure_path, env=env, check=True)

    # Install the library
    subprocess.run(["make", f"-j{args.jobs}", "install"], cwd=args.configure_path, env=env, check=True)


def handle_bjam(archive, args, env):
    # Configure the library
    subprocess.run(
        ["./bootstrap.sh", f"--prefix={args.prefix}", f"--with-python={sys.executable}"],
        cwd=args.configure_path,
        env=env,
        check=True,
    )

    # Build and install the library
    subprocess.run(
        [
            "./b2",
            *args.extra_args,
            f"include={os.path.join(args.prefix, args.header_path)}",
            f"library-path={os.path.join(args.prefix, 'lib')}",
            f"-j{args.jobs}",
            "-q",
            f"cflags={env['CFLAGS']}",
            f"cxxflags={env['CXXFLAGS']}",
            "install",
        ],
        cwd=args.configure_path,
        env=env,
        check=True,
    )


def handle_cmake(archive, args, env):
    # Create the build folder
    build = os.path.join(args.build_folder, "build")
    os.makedirs(build, exist_ok=True)

    # Configure the library
    subprocess.run(
        [
            "cmake",
            "-S",
            f"{args.configure_path}",
            "-B",
            f"{build}",
            *args.extra_args,
            f"-DCMAKE_BUILD_TYPE=Release",
            *(
                [f"-DCMAKE_TOOLCHAIN_FILE={os.path.join(args.prefix, 'toolchain.cmake')}"]
                if not args.no_toolchain
                else []
            ),
            "-Wno-dev",
            "-GNinja",
        ],
        cwd=args.configure_path,
        env=env,
        check=True,
    )

    # Build the library
    subprocess.run(["ninja", f"-j{args.jobs}"], cwd=build, env=env, check=True)

    # Install the library
    subprocess.run(["ninja", f"-j{args.jobs}", "install"], cwd=build, env=env, check=True)


def handle_header(header_file, args, env):
    header_path = os.path.join(args.prefix, args.header_path)
    os.makedirs(header_path, exist_ok=True)
    os.rename(header_file, os.path.join(header_path, os.path.basename(header_file)))


def handle_make(archive, args, env):
    # Build the library
    # Some things use the command line provided CFLAGS while others overwrite it with their own values
    # Some of the libraries that overwrite CFLAGS allow you to set EXTRA_CFLAGS
    subprocess.run(
        [
            "make",
            "-B",
            *args.extra_args,
            f"-j{args.jobs}",
            f"CFLAGS={env['CFLAGS']}",
            f"CPPFLAGS={env['CPPFLAGS']}",
            f"CXXFLAGS={env['CXXFLAGS']}",
        ],
        cwd=args.configure_path,
        env=env,
        check=True,
    )

    # Install the library
    # Some scripts use "prefix" while others use "PREFIX"
    subprocess.run(
        ["make", f"prefix={args.prefix}", f"PREFIX={args.prefix}", f"-j{args.jobs}", "install"],
        cwd=args.configure_path,
        env=env,
        check=True,
    )


def handle_meson(archive, args, env):
    # Create the build folder
    build = os.path.join(args.build_folder, "build")
    os.makedirs(build, exist_ok=True)

    # Configure the library
    subprocess.run(
        [
            "meson",
            "setup",
            build,
            args.configure_path,
            *args.extra_args,
            *([f"--cross-file={os.path.join(args.prefix, 'meson.cross')}"] if not args.no_toolchain else []),
            "--buildtype=release",
            f"--pkg-config-path={env['PKG_CONFIG_PATH']}",
            "-Dstrip=true",
        ],
        cwd=args.configure_path,
        env=env,
        check=True,
    )

    # Build the library
    subprocess.run(["ninja", f"-j{args.jobs}"], cwd=build, env=env, check=True)

    # Install the library
    subprocess.run(["ninja", f"-j{args.jobs}", "install"], cwd=build, env=env, check=True)


def handle_python(archive, args, env):
    # Build the library
    subprocess.run(["python", "setup.py", "build", *args.extra_args], cwd=args.configure_path, env=env, check=True)

    # Install the library
    subprocess.run(
        ["python", "setup.py", "build", *args.extra_args, f"--prefix={args.prefix}"],
        cwd=args.configure_path,
        env=env,
        check=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser("")
    parser.add_argument(
        "--patch", action="append", default=[], help="Path or URL to a patch file to be applied to this library"
    )
    parser.add_argument(
        "--build-system",
        choices=["autotools", "bjam", "cmake", "header", "make", "meson", "python"],
        help="The build system to use to build this library",
    )
    parser.add_argument("--prefix", default="/usr/local", help="Location to install libraries")
    parser.add_argument("--build-folder", default="/var/tmp/build", help="Location to use for build artifacts")
    parser.add_argument("--configure-path", default="", help="Location of configuration file to use")
    parser.add_argument("--header-path", default="include", help="Location to install lone header files")
    parser.add_argument("--jobs", default=mp.cpu_count(), type=int, help="Number of build jobs to run in parallel")
    parser.add_argument("--no-toolchain", action="store_true", help="Don't load any toolchain settings")
    parser.add_argument(
        "--autotools-force-regenerate", action="store_true", help="Force autotools to regenerate configuration files"
    )
    parser.add_argument("--extra-cflags", help="Extra flags to append to CFLAGS")
    parser.add_argument("--extra-cxxflags", help="Extra flags to append to CXXFLAGS")
    parser.add_argument("--extra-cppflags", help="Extra flags to append to CPPFLAGS")
    parser.add_argument("url", help="The URL of the source code to download")

    args, extra_args = parser.parse_known_args()
    args.extra_args = extra_args

    # Make build directory
    os.makedirs(args.build_folder, exist_ok=True)

    # Make install directory
    os.makedirs(args.prefix, exist_ok=True)

    # Download the source code
    archive = download_file(args.url, args.build_folder)

    # Extract source code
    extension = os.path.splitext(archive)[1]

    # Figure out the type of file we downloaded so we know how to extract it
    mime = magic.from_file(archive, mime=True)
    extracted_path = args.build_folder
    if mime in (
        "application/gzip",
        "application/x-bzip2",
        "application/x-xz",
    ):
        # Check what sort of compressed file we have
        m = magic.Magic(mime=True, uncompress=True)
        mime = m.from_file(archive)

        # We have a tar.XX file
        if mime == "application/x-tar":
            extracted_path = os.path.join(args.build_folder, extract_tar(archive, args.build_folder))
        else:
            raise RuntimeError(
                "Error 418: I'm a teapot\nI don't know how to handle compressed files that are not tar files"
            )
    # We have a zip file
    elif mime == "application/zip":
        extracted_path = os.path.join(args.build_folder, extract_zip(archive, args.build_folder))
    # We have a tar file
    elif mime == "application/x-tar":
        extracted_path = os.path.join(args.build_folder, extract_tar(archive, args.build_folder))
    # File is HTML, scream loudly
    elif mime == "text/html":
        raise RuntimeError(
            "Error 418: I'm a teapot\nI don't know how to handle HTML files, did you forget to get raw from github??)"
        )
    # File is just plain text, probably a header file, nothing to extract
    elif mime.startswith("text/"):
        args.build_system = "header"
    else:
        raise RuntimeError(f"Unknown mime type {mime}")

    # Patch source files
    for patch_url in args.patch:
        # Download each patch file and apply it
        patch = download_file(patch_url, extracted_path)
        subprocess.run(["patch", "-Np1", "-i", patch], cwd=extracted_path, check=True)

    # Load toolchain environment variables
    env = {}
    if not args.no_toolchain:
        with open(os.path.join(args.prefix, "toolchain.json"), "r") as f:
            env = json.load(f)

    # Cherry pick host environment
    env["PATH"] = os.environ.get("PATH", "")

    # Concatenate compiler flags with any extra compiler flags that were provided
    if args.extra_cflags:
        env["CFLAGS"] = f"{env['CFLAGS']} {args.extra_cflags}"
    if args.extra_cxxflags:
        env["CXXFLAGS"] = f"{env['CXXFLAGS']} {args.extra_cxxflags}"
    if args.extra_cppflags:
        env["CPPFLAGS"] = f"{env['CPPFLAGS']} {args.extra_cppflags}"

    # Configure, build, and install the library
    args.method, args.configure_path = detect_build_system(args, extracted_path)
    {
        "autotools": handle_autotools,
        "bjam": handle_bjam,
        "cmake": handle_cmake,
        "header": handle_header,
        "make": handle_make,
        "meson": handle_meson,
        "python": handle_python,
    }[args.method](archive, args, env)

    # Build was successful, purge the build folder
    shutil.rmtree(args.build_folder)
