#!/usr/bin/env python3

import os

import b
from utility.dockerise import run_on_docker
from utility.shell import WrapPty


@run_on_docker
def register(command):
    # Install help
    command.help = "Configure the project in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )

    command.add_argument(
        "-r",
        "--roles",
        action="store",
        dest="roles",
        default=None,
        help="Colon-separated list of roles to enable",
    )

    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, roles, args, **kwargs):
    pty = WrapPty()

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # If we have selected specific roles to enable, we do that
    if roles:
        # The roles are a colon separated list, so we split on that character
        roles = roles.split(":")

        args += b.get_cmake_role_flags(roles)

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    # If interactive then run ccmake else just run cmake
    if interactive:
        exit(
            pty.spawn(["ccmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir])
        )
    else:
        exit(pty.spawn(["cmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir]))
