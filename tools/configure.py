#!/usr/bin/env python3

import os

import b
from utility.dockerise import run_on_docker
from utility.shell import WrapPty


@run_on_docker
def register(command):
    # Install help
    command.description = "Configure the project in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, args, **kwargs):
    pty = WrapPty()

    default_args = [
        "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake",
        "-DCMAKE_C_COMPILER_LAUNCHER=/usr/bin/ccache",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache",
    ]

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    # If interactive then run ccmake else just run cmake
    os.chdir(os.path.join(b.project_dir, "..", "build"))
    if interactive:
        exit(pty.spawn(["ccmake", "-GNinja", *default_args, *args, b.project_dir]))
    else:
        exit(pty.spawn(["cmake", "-GNinja", *default_args, *args, b.project_dir]))
