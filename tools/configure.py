#!/usr/bin/env python3

import os

import b
from utility.dockerise import run_on_docker
from utility.roles import all_role_names
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
        "--set_roles",
        dest="set_roles",
        default=[],
        choices=all_role_names(),
        nargs="*",
        help="Set roles via command line"
    )
    command.add_argument(
        "--unset_roles",
        dest="unset_roles",
        default=[],
        choices=all_role_names(),
        nargs="*",
        help="Unset roles via command line"
    )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, set_roles, unset_roles, args, **kwargs):
    pty = WrapPty()

    # If interactive then run ccmake else just run cmake
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    for role in set_roles:
        role = role.replace("/","-")
        args.append(f"-DROLE_{role}:BOOL=ON")
    for role in unset_roles:
        role = role.replace("/","-")
        args.append(f"-DROLE_{role}:BOOL=OFF")

    if interactive:
        exit(
            pty.spawn(["ccmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir])
        )
    else:
        exit(pty.spawn(["cmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir]))
